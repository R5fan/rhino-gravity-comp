"""
rhino_gravity_comp.py - position-aware "anti-gravity" force for a VPforce Rhino.

Problem: a Rhino mounted at an angle and/or with an S-shaped extension has
the grip's centre of mass off the pivot. Gravity then pulls the stick one
way - but by how much depends on where the stick is: aft of centre the pull
is strong, and as the stick goes forward the weight swings over the pivot
and the pull fades or even reverses. A single constant force (like the
configurator's static force) therefore helps aft but over-pushes forward.

This script measures the gravity pull at several stick positions, fits a
smooth curve through them, and while flying applies exactly the force that
cancels gravity at the stick's CURRENT position (updated ~200x per second).
The force is sent straight to the Rhino over HID, the way VPforce's TelemFFB
does it, on top of whatever the sim and the configurator do.

    python rhino_gravity_comp.py --calibrate   # once, hands off (~2 min)
    python rhino_gravity_comp.py --tune        # optional: nudge the curve
    python rhino_gravity_comp.py               # run while flying

The live window (stick, force, curves and all messages) opens by default
while calibrating; press G to show or hide it in any mode.

Unofficial community tool, not made or supported by VPforce. Use at your own
risk and keep the Rhino's emergency stop within reach.

License: GPL-3.0 (see LICENSE). The Rhino HID protocol details are taken from
VPforce TelemFFB (https://github.com/walmis/VPforce-TelemFFB), GPL-3.0,
Copyright (c) 2023 Valmantas Paliksa.

Requirements:  pip install hidapi
The live window uses tkinter, which comes with Python on Windows.
"""

import argparse
import collections
import json
import math
import os
import struct
import sys
import signal
import threading
import time

try:
    import hid  # 'hidapi' package (pip install hidapi) or 'hid' package
except ImportError as e:
    sys.exit(f"Can't import the HID library ({e}).\n"
             f"This Python is: {sys.executable}\n"
             f"Install it into THIS Python with:\n"
             f'  "{sys.executable}" -m pip install hidapi')

try:
    import msvcrt  # Windows console keyboard
except ImportError:
    msvcrt = None

try:
    import winsound  # Windows: beep through the PC's speakers
except ImportError:
    winsound = None


def pc_beep(freq, ms=50):
    """A short beep on the PC, played in the background so nothing waits."""
    if not winsound:
        return
    threading.Thread(target=lambda: winsound.Beep(int(clamp(freq, 37, 32767)), int(ms)),
                     daemon=True).start()

# --------------------------------------------------------------------------
# Rhino HID protocol (from VPforce TelemFFB, telemffb/hw/ffb_rhino.py)
# --------------------------------------------------------------------------
RHINO_VID, RHINO_PID = 0xFFFF, 0x2055

REP_INPUT = 1                  # input: id, X, Y (int16, +-4096), ...
REP_PID_STATE = 2              # input: byte1 bit5 = device reset event
REP_CREATE_EFFECT = 5          # feature: [5, type, 0, 0]
REP_BLOCK_LOAD = 6             # feature read: [6, effect_id, status, ...]
REP_SET_EFFECT = 101           # <BBBHHHHBBBBH
REP_SET_CONDITION = 103        # <BBBhhhHHH
REP_SET_CONSTANT_FORCE = 105   # <BBh  magnitude -4096..4096
REP_EFFECT_OPERATION = 110     # <BBBB
REP_BLOCK_FREE = 111           # <BB
REP_DEVICE_CONTROL = 112       # [112, 4] = reset: wipes ALL effects on the stick
REP_GET_GAINS = 0x56           # feature: id, master, periodic, spring, damper,
                               #   inertia, friction, constant (whole percent)
REP_SET_GAIN = 0x57            # feature: id, slider id, value (0-100 percent)
GAIN_MASTER = 1                # configurator slider ids (TelemFFB FFB_GAIN_*)

# extra axes in the input report: <BhhhBBBB = id, X, Y, Z, Rz, Ry, Rx, Slider
POT_AXES = ("z", "rz", "ry", "rx", "slider")
POT_RAW = {"spec": None}       # set from config when --find-pot located the pot
CONTROL_RESET = 4

EFFECT_CONSTANT, EFFECT_SPRING = 1, 8
LOAD_SUCCESS = 1
OP_START, OP_STOP = 1, 3
AXIS_X, AXIS_Y, AXIS_DIR = 1, 2, 4
SOURCE_TELEMFFB = 0x08         # bit 3 of axesEnable: 0 = game, 1 = TelemFFB

AXES = {"roll": 0, "pitch": 1}  # name -> index into (x, y) and condition block

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "rhino_gravity_comp.json")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# --------------------------------------------------------------------------
# State shared with the optional GUI (written by the worker, read by the GUI)
# --------------------------------------------------------------------------
class Shared:
    def __init__(self):
        self.pos = None                     # (roll, pitch) -1..1
        self.force = (0.0, 0.0)             # (roll, pitch) being applied
        self.cfg = None                     # curves
        self.live = {"pitch": [], "roll": []}   # calibration points so far
        self.target = None                  # (axis, u) during calibration
        self.status = "starting..."
        self.stop = threading.Event()       # Ctrl+C -> stop the worker
        self.done = False
        self.keys = collections.deque()     # key presses for the worker
        self.toggle_gui = False             # 'G' pressed
        self.gui_available = False
        self.log = collections.deque(maxlen=500)   # finished output lines
        self.live_line = ""                 # line being overwritten with '\r'
        self.log_rev = 0                    # bumps when log changes
        self.lock = threading.Lock()


SH = Shared()


def read_keys():
    """Collect console key presses: G toggles the window, the rest go to the worker."""
    if not msvcrt:
        return
    try:
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("g", "G"):
                if SH.gui_available:
                    SH.toggle_gui = True
                else:
                    print("\n[INFO] The live window needs tkinter, which this Python doesn't have.")
            else:
                SH.keys.append(ch)
    except Exception:
        pass


class Tee:
    """Mirror everything printed into SH.log for the GUI's log panel."""
    def __init__(self, real):
        self.real = real

    def write(self, text):
        try:
            self.real.write(text)
        except Exception:
            pass
        with SH.lock:
            buf = SH.live_line
            for ch in text:
                if ch == "\n":
                    if buf.strip():
                        SH.log.append(buf)
                        SH.log_rev += 1
                    buf = ""
                elif ch == "\r":
                    buf = ""
                else:
                    buf += ch
            SH.live_line = buf
        return len(text)

    def flush(self):
        try:
            self.real.flush()
        except Exception:
            pass


def nap(seconds):
    """time.sleep that also handles keys and stops on Ctrl+C."""
    end = time.time() + seconds
    while True:
        read_keys()
        if SH.stop.is_set():
            raise KeyboardInterrupt
        left = end - time.time()
        if left <= 0:
            return
        time.sleep(min(left, 0.05))


def say(msg, end="\n", **_):
    print(msg, end=end, flush=True)
    SH.status = msg.strip()


# --------------------------------------------------------------------------
# Config: per axis a curve force(u) = c0 + c1*u + c2*u^2, valid on [lo, hi]
# (u = stick position -1..1). Old single-number configs still load.
# --------------------------------------------------------------------------
def empty_curve():
    return {"c": [0.0, 0.0, 0.0], "lo": -1.0, "hi": 1.0, "points": []}


def load_config():
    cfg = {"roll": empty_curve(), "pitch": empty_curve()}
    try:
        with open(CONFIG_FILE, "r") as f:
            raw = json.load(f)
    except (OSError, ValueError):
        return cfg
    for axis in AXES:
        v = raw.get(axis)
        if isinstance(v, (int, float)):          # old constant-force config
            cfg[axis]["c"][0] = float(v)
        elif isinstance(v, dict):
            cfg[axis].update(v)
    for key in ("gain", "gain_pot", "gain_range", "gain_pot_raw", "beep"):  # see GainKnob
        if key in raw:
            cfg[key] = raw[key]
    return cfg


def save_config(cfg):
    with open(CONFIG_FILE, "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"\nSaved to {CONFIG_FILE}")


def curve_force(curve, u):
    u = clamp(u, curve["lo"], curve["hi"])      # never extrapolate
    c0, c1, c2 = curve["c"]
    return c0 + c1 * u + c2 * u * u


def fit_curve(points):
    """Least-squares quadratic through [(u, force), ...] (linear if < 4 pts)."""
    n = len(points)
    if n == 0:
        return [0.0, 0.0, 0.0]
    if n == 1:
        return [points[0][1], 0.0, 0.0]
    deg = 2 if n >= 4 else 1
    m = deg + 1
    # normal equations A^T A c = A^T y
    ata = [[sum(u ** (i + j) for u, _ in points) for j in range(m)] for i in range(m)]
    aty = [sum(f * u ** i for u, f in points) for i in range(m)]
    # gaussian elimination
    for col in range(m):
        piv = max(range(col, m), key=lambda r: abs(ata[r][col]))
        ata[col], ata[piv] = ata[piv], ata[col]
        aty[col], aty[piv] = aty[piv], aty[col]
        for r in range(m):
            if r != col and ata[col][col]:
                k = ata[r][col] / ata[col][col]
                ata[r] = [a - k * b for a, b in zip(ata[r], ata[col])]
                aty[r] -= k * aty[col]
    c = [aty[i] / ata[i][i] if ata[i][i] else 0.0 for i in range(m)]
    return c + [0.0] * (3 - m)


# --------------------------------------------------------------------------
# Thin wrapper so both Python HID libraries work
# --------------------------------------------------------------------------
class HidDev:
    def __init__(self, path):
        if hasattr(hid, "device"):          # 'hidapi' (cython-hidapi)
            self.d = hid.device()
            self.d.open_path(path)
            self.d.set_nonblocking(1)
        else:                               # 'hid' (ctypes, needs hidapi.dll)
            self.d = hid.Device(path=path)
            self.d.nonblocking = True

    def write(self, data):
        n = self.d.write(bytes(data))
        if n is not None and n < 0:
            raise IOError("HID write failed")

    def send_feature(self, data):
        n = self.d.send_feature_report(bytes(data))
        if n is not None and n < 0:
            raise IOError("HID feature write failed")

    def get_feature(self, report_id, size):
        return bytes(self.d.get_feature_report(report_id, size))

    def read(self):
        r = self.d.read(64)
        return bytes(r) if r else b""

    def close(self):
        try:
            self.d.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# The Rhino: position input + our constant-force effect (+ a calibration spring)
# --------------------------------------------------------------------------
class Rhino:
    SLOT = 20         # park our force here, far above the slots sims use
    SPRING_SLOT = 21  # calibration spring, also out of the sim's way
    OUR_SLOTS = range(20, 26)   # anything here is ours (maybe left by a killed run)

    def __init__(self):
        self.dev = None
        self.eid = None
        self.started = False
        self.sent_dir = self.sent_mag = None
        self.fx = self.fy = 0.0          # wanted force (roll, pitch), -1..1
        self.pos = None                  # filtered (roll, pitch) -1..1
        self.raw_pos = None
        self.last_report_t = 0.0
        self.spring_id = None
        self.closing = False
        self.extra = {}                  # pot / extra axes, raw values
        self.last_input = b""
        self.connect()

    # ---- connection / allocation ----------------------------------------
    def connect(self):
        infos = list(hid.enumerate(RHINO_VID, RHINO_PID))
        if not infos:
            raise IOError("Rhino not found (VID FFFF / PID 2055). Is it plugged in?")
        infos.sort(key=lambda d: 0 if (d.get("usage_page"), d.get("usage")) == (1, 4) else 1)
        last_err = None
        for info in infos:
            dev = None
            try:
                dev = HidDev(info["path"])
                self.dev = dev
                self._clear_our_slots()
                eid = self._create_high(EFFECT_CONSTANT)
                if eid:
                    self.eid, self.started = eid, False
                    self.sent_dir = self.sent_mag = None
                    print(f"[INFO] Rhino opened, constant-force effect in slot {eid}")
                    self._apply(force=True)
                    return
                last_err = "effect pool full / create refused"
            except Exception as e:
                last_err = e
            if dev:
                dev.close()
            self.dev = None
        raise IOError(f"Could not create an effect on the Rhino ({last_err})")

    def _create(self, etype):
        self.dev.send_feature([REP_CREATE_EFFECT, etype, 0, 0])
        r = self.dev.get_feature(REP_BLOCK_LOAD, 5)
        if len(r) >= 3 and r[0] == REP_BLOCK_LOAD and r[2] == LOAD_SUCCESS and r[1]:
            return r[1]
        return None

    def _free(self, eid):
        self.dev.write(struct.pack("<BB", REP_BLOCK_FREE, eid))

    def _clear_our_slots(self):
        """Remove forces a previous run left behind (e.g. killed before it
        could clean up). Only touches our own high slots, never the sim's."""
        for e in self.OUR_SLOTS:
            self.dev.write(struct.pack("<BBBB", REP_EFFECT_OPERATION, e, OP_STOP, 0))
            self._free(e)

    def _create_high(self, etype, slot=None):
        """The Rhino hands out the lowest free slot, so by allocating up to
        SLOT and keeping the top one, the sim's effects never land on ours."""
        got = []
        for _ in range(64):
            eid = self._create(etype)
            if eid is None:
                break
            got.append(eid)
            if eid >= (slot or self.SLOT):
                break
        if not got:
            return None
        mine = got.pop()
        for e in got:
            self._free(e)
        return mine

    # ---- position input -------------------------------------------------
    def poll(self, tau=0.015):
        """Drain input reports. Updates self.pos (low-pass filtered).
        Returns True if the Rhino flagged an FFB reset (effects wiped)."""
        read_keys()
        if SH.stop.is_set() and not self.closing:
            raise KeyboardInterrupt
        reset = False
        latest = None
        try:
            for _ in range(500):
                r = self.dev.read()
                if not r:
                    break
                if r[0] == REP_INPUT and len(r) >= 5:
                    latest = r
                elif r[0] == REP_PID_STATE and len(r) >= 2 and (r[1] & 0x20):
                    reset = True
        except Exception as e:
            print(f"\n[WARN] HID read error ({e}); reconnecting...")
            self._reconnect_loop()
            return True
        if latest:
            _, x, y = struct.unpack_from("<Bhh", latest)
            self.last_input = latest
            if len(latest) >= 11:
                z, rz, ry, rx, sl = struct.unpack_from("<hBBBB", latest, 5)
                self.extra = {"z": z, "rz": rz, "ry": ry, "rx": rx, "slider": sl}
            raw_pot = POT_RAW.get("spec")
            if raw_pot and len(latest) >= raw_pot["offset"] + struct.calcsize(raw_pot["fmt"]):
                self.extra["raw"] = struct.unpack_from(raw_pot["fmt"], latest, raw_pot["offset"])[0]
            now = time.perf_counter()
            raw = (clamp(x / 4096, -1, 1), clamp(y / 4096, -1, 1))
            if self.pos is None:
                self.pos = raw
            else:
                a = clamp((now - self.last_report_t) / tau, 0.0, 1.0)
                self.pos = tuple(p + a * (q - p) for p, q in zip(self.pos, raw))
            self.raw_pos = raw
            self.last_report_t = now
            SH.pos = self.pos
        return reset

    def average_pos(self, seconds):
        n, s = 0, [0.0, 0.0]
        end = time.time() + seconds
        while time.time() < end:
            self.poll()
            if self.raw_pos:
                s[0] += self.raw_pos[0]
                s[1] += self.raw_pos[1]
                n += 1
            time.sleep(0.002)
        if not n:
            raise IOError("No position reports from the Rhino.")
        return s[0] / n, s[1] / n

    # ---- force output -------------------------------------------------
    def set_force(self, roll, pitch):
        self.fx, self.fy = clamp(roll, -1, 1), clamp(pitch, -1, 1)
        SH.force = (self.fx, self.fy)
        self._guarded(self._apply)

    def _apply(self, force=False):
        # direction kept in [0,180) degrees, sign carried by the magnitude
        mag = math.hypot(self.fx, self.fy)
        ang = math.degrees(math.atan2(self.fx, self.fy)) % 360 if mag > 0 else 0.0
        if ang >= 180.0 - 1e-9:
            ang -= 180.0
            mag = -mag
        dir_byte = round(ang * 255 / 360) & 0xFF
        mag_i = round(clamp(mag, -1, 1) * 4096)
        if force or dir_byte != self.sent_dir or not self.started:
            # SetEffect: id, block, type, duration(0=inf), trigRepeat, samplePeriod,
            #            gain, trigButton, axesEnable(+source), dirX, dirY, startDelay
            self.dev.write(struct.pack("<BBBHHHHBBBBH", REP_SET_EFFECT, self.eid,
                                       EFFECT_CONSTANT, 0, 0, 0, 4096, 0,
                                       AXIS_DIR | SOURCE_TELEMFFB, dir_byte, 0, 0))
            self.sent_dir = dir_byte
        if force or mag_i != self.sent_mag or not self.started:
            self.dev.write(struct.pack("<BBh", REP_SET_CONSTANT_FORCE, self.eid, mag_i))
            self.sent_mag = mag_i
        if not self.started:
            self.dev.write(struct.pack("<BBBB", REP_EFFECT_OPERATION, self.eid, OP_START, 1))
            self.started = True

    # ---- calibration spring --------------------------------------------
    def spring_set(self, cp=(0.0, 0.0), coef=(1.0, 1.0)):
        """A firmware spring that holds the stick at cp during calibration."""
        def _do():
            if self.spring_id is None:
                self.spring_id = self._create_high(EFFECT_SPRING, self.SPRING_SLOT)
                if not self.spring_id:
                    raise IOError("couldn't create calibration spring")
                self.dev.write(struct.pack("<BBBHHHHBBBBH", REP_SET_EFFECT, self.spring_id,
                                           EFFECT_SPRING, 0, 0, 0, 4096, 0,
                                           AXIS_X | AXIS_Y | SOURCE_TELEMFFB, 0, 0, 0))
                self.dev.write(struct.pack("<BBBB", REP_EFFECT_OPERATION, self.spring_id,
                                           OP_START, 1))
            for block in (0, 1):
                k = round(clamp(coef[block], 0, 1) * 4096)
                off = round(clamp(cp[block], -1, 1) * 4096)
                self.dev.write(struct.pack("<BBBhhhHHH", REP_SET_CONDITION, self.spring_id,
                                           block, off, k, k, 4096, 4096, 0))
        self._guarded(_do)

    # ---- configurator gain sliders ---------------------------------------
    def get_gains(self):
        """(master %, constant-force %) as the configurator sliders stand,
        or None if the stick doesn't answer."""
        try:
            r = self.dev.get_feature(REP_GET_GAINS, 8)
            if len(r) >= 8 and r[0] == REP_GET_GAINS:
                return r[1], r[7]
        except Exception:
            pass
        return None

    def effective_gain(self):
        g = self.get_gains()
        return g[0] * g[1] / 100.0 if g else None

    def set_master_gain(self, percent):
        """Move the configurator's Master Gain slider (0-100)."""
        self._guarded(lambda: self.dev.send_feature(
            [REP_SET_GAIN, GAIN_MASTER, int(clamp(percent, 0, 100))]))

    def reset_device(self):
        """Wipe every effect on the stick (leftovers from an interrupted run,
        too), then re-create ours. Only safe with every sim closed."""
        def _do():
            # free our own blocks first, in case the reset doesn't go through
            self.spring_free()
            if self.eid:
                self._free(self.eid)
            self.dev.write(bytes([REP_DEVICE_CONTROL, CONTROL_RESET]))
            time.sleep(0.2)
            self.spring_id = None
            self.eid = self._create_high(EFFECT_CONSTANT)
            if not self.eid:
                raise IOError("couldn't re-create the effect after reset")
            self.started = False
            self._apply(force=True)
        self._guarded(_do)

    def spring_free(self):
        if self.spring_id is not None and self.dev:
            try:
                self.dev.write(struct.pack("<BBBB", REP_EFFECT_OPERATION,
                                           self.spring_id, OP_STOP, 0))
                self._free(self.spring_id)
            except Exception:
                pass
        self.spring_id = None

    # ---- keeping the effect alive ---------------------------------------
    def check_alive(self):
        return self._guarded(self._check_alive)

    def _check_alive(self):
        probes, reclaimed = [], False
        for _ in range(64):
            eid = self._create(EFFECT_CONSTANT)
            if eid is None:
                break
            if eid == self.eid:
                reclaimed = True
                break
            probes.append(eid)
            if eid > self.eid:
                break
        for p in probes:
            self._free(p)
        if reclaimed:
            print(f"\n[INFO] Effect had been wiped (the sim took or released the stick)"
                  f" - restored in slot {self.eid}")
        self.started = False
        self._apply(force=True)
        return reclaimed

    def recreate(self):
        """'R': replace our force effect with a fresh one. The old one is
        removed first - our slots are never the sim's, so that's safe, and
        it stops the forces stacking up on repeated presses."""
        def _do():
            if self.eid:
                self.dev.write(struct.pack("<BBBB", REP_EFFECT_OPERATION, self.eid, OP_STOP, 0))
                self._free(self.eid)
            eid = self._create_high(EFFECT_CONSTANT)
            if eid:
                self.eid, self.started = eid, False
                self.sent_dir = self.sent_mag = None
                self._apply(force=True)
                print(f"\n[INFO] Force effect re-created in slot {eid}")
            else:
                print("\n[WARN] Couldn't re-create the force effect (effect pool full?)")
        self._guarded(_do)

    def _guarded(self, fn):
        try:
            return fn()
        except Exception as e:
            print(f"\n[WARN] HID error ({e}); reconnecting...")
            self._reconnect_loop()

    def _reconnect_loop(self):
        if self.dev:
            self.dev.close()
        self.dev = None
        self.spring_id = None
        while True:
            time.sleep(1.0)
            try:
                self.connect()
                return
            except Exception as e:
                print(f"\r[WARN] waiting for Rhino: {e}   ", end="", flush=True)

    def ramp_to(self, roll, pitch, seconds=1.0):
        r0, p0 = self.fx, self.fy
        steps = max(1, int(seconds / 0.02))
        for i in range(1, steps + 1):
            t = i / steps
            self.set_force(r0 + (roll - r0) * t, p0 + (pitch - p0) * t)
            self.poll()
            time.sleep(0.02)

    def close(self):
        if not self.dev:
            return
        self.closing = True
        # Remove our forces FIRST and fast: the 'py' launcher may kill this
        # process moments after Ctrl+C, so no fade-out, and ignore further Ctrl+C.
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except Exception:
            pass                                    # not in the main thread (GUI)
        try:
            self.dev.write(struct.pack("<BBh", REP_SET_CONSTANT_FORCE, self.eid, 0))
            self.dev.write(struct.pack("<BBBB", REP_EFFECT_OPERATION, self.eid, OP_STOP, 0))
            self._free(self.eid)
            self.spring_free()
        except Exception:
            pass
        self.dev.close()
        self.dev = None


# --------------------------------------------------------------------------
# Calibration
# --------------------------------------------------------------------------
def calibrate(args):
    cfg = load_config()
    SH.cfg = cfg
    rh = Rhino()
    rh.reset_device()          # clear leftovers (e.g. a spring from an interrupted run)
    gains = rh.get_gains()
    restore_master = None
    if gains and cfg.get("gain_pot"):
        # The pot drives master gain, so its current position says nothing about
        # the user's normal level: calibrate at a fixed 50%, then put it back.
        restore_master = gains[0]
        rh.set_master_gain(50)
        time.sleep(0.2)
        gains = rh.get_gains()
        say(f"Master gain set to 50% for calibration (it goes back to {restore_master}% "
            f"afterwards).")

    def put_gain_back():
        if restore_master is not None:
            rh.set_master_gain(restore_master)
            say(f"Master gain back to {restore_master}%.")

    if gains and gains[0] * gains[1] / 100.0 < 20:
        if cfg.get("gain_pot"):
            say(f"The constant-force slider is at {gains[1]}%: too low to calibrate (at 50% "
                f"master gain it needs at least 40%). Raise it in the configurator, then run "
                f"--calibrate again.")
        else:
            say(f"Master gain is {gains[0]}% and the constant-force slider {gains[1]}%: too low "
                f"to calibrate. Set both to your normal flying level in the configurator (at "
                f"least 20%), then run --calibrate again.")
        put_gain_back()
        rh.close()
        return
    axes = ["pitch"] + (["roll"] if not args.pitch_only else [])
    measured = []
    # --max is the force AT THE STICK (as at 100% gain): at lower gain the same
    # physical limit allows a proportionally bigger command
    eff = gains[0] * gains[1] / 100.0 if gains else 100.0
    lim = min(1.0, args.max * 100.0 / max(eff, 1.0))
    targets = [-args.range + 2 * args.range * i / (args.points - 1) for i in range(args.points)]
    say("\n=== CALIBRATION ===")
    say("Close all flight sims and TelemFFB first. The stick will be moved to several positions by a")
    say("temporary spring while the script finds the force that cancels gravity there.")
    say("LET GO of the stick completely. Starting in 5 s... (Ctrl+C aborts)")
    try:
        nap(5)
        rh.poll()
        # gently bring the holding spring in
        for i in range(1, 41):
            rh.spring_set((0, 0), (i / 40, i / 40))
            rh.poll()
            nap(0.03)
        nap(1.0)

        for axis in axes:
            idx = AXES[axis]
            other = 1 - idx
            force = [0.0, 0.0]                 # (roll, pitch) constant force

            def setf():
                rh.set_force(force[0], force[1])

            def hold(cp_axis):
                cp = [0.0, 0.0]
                cp[idx] = cp_axis
                rh.spring_set(tuple(cp))

            # -- which way does a positive force move this axis, and how far?
            hold(0.0)
            nap(1.0)
            force[idx] = +0.1; setf(); nap(1.2)
            p_plus = rh.average_pos(0.6)[idx]
            force[idx] = -0.1; setf(); nap(1.2)
            p_minus = rh.average_pos(0.6)[idx]
            force[idx] = 0.0; setf(); nap(0.8)
            sens = (p_plus - p_minus) / 0.2
            if abs(sens) < 0.05:
                say(f"[{axis}] the stick barely reacts to force - is FFB on and the "
                      f"constant-force slider above 0? Skipping {axis}.")
                continue
            # -- does the spring centre move the stick the same way as the axis?
            base = rh.average_pos(0.6)[idx]
            hold(0.3); nap(1.2)
            moved = rh.average_pos(0.6)[idx] - base
            if abs(moved) < 0.6 * 0.3:      # should follow at least 60% of the way
                say(f"[{axis}] the calibration spring can't move the stick (moved {moved:+.3f}).\n"
                    f"Something else is holding it: is a sim, TelemFFB or a spring in the configurator's "
                    f"Effects tab active? Close/disable it, or unplug and replug the Rhino, "
                    f"then try again.")
                raise KeyboardInterrupt
            cp_sign = 1.0 if moved >= 0 else -1.0
            say(f"[{axis}] +force moves stick {'+' if sens > 0 else '-'} "
                  f"({abs(sens):.2f}/unit), spring centre sign {cp_sign:+.0f}")

            # -- measure the gravity-cancelling force at each target position
            points = []
            cp_now = 0.3                    # where hold(0.3) left the spring centre
            for t in targets:
                # glide the spring centre to the next target
                cp_goal = t * cp_sign
                for i in range(1, 26):
                    hold((cp_now + (cp_goal - cp_now) * i / 25) * 1.0)
                    rh.poll()
                    nap(0.02)
                cp_now = cp_goal
                SH.target = (axis, t)
                nap(0.6)
                good = 0
                for _ in range(60):
                    u = rh.average_pos(0.35)[idx]
                    err = u - t
                    say(f"\r[{axis}] target {t:+.2f}  stick {u:+.3f}  force {force[idx]:+.3f}   ",
                          end="", flush=True)
                    if abs(err) < args.tolerance:
                        good += 1
                        if good >= 3:
                            break
                        continue
                    good = 0
                    step = clamp(-0.7 * err / sens, -0.05, 0.05)
                    force[idx] = clamp(force[idx] + step, -lim, lim)
                    setf()
                    nap(0.25)
                u = rh.average_pos(0.5)[idx]
                ok = abs(u - t) < 3 * args.tolerance and abs(force[idx]) < lim - 1e-6
                say(f"\r[{axis}] at {u:+.3f} gravity needs {force[idx]:+.3f}"
                      f"{'' if ok else '   (not settled - skipped)'}                  ")
                if ok:
                    points.append((round(u, 4), round(force[idx], 4)))
                    SH.live[axis] = list(points)

            if len(points) < 2:
                say(f"[{axis}] not enough good points, keeping the old setting.")
                continue
            c = fit_curve(points)
            lo, hi = min(p[0] for p in points), max(p[0] for p in points)
            cfg[axis] = {"c": [round(v, 5) for v in c], "lo": lo, "hi": hi, "points": points}
            measured.append(axis)
            worst = max(abs(curve_force(cfg[axis], u) - f) for u, f in points)
            say(f"[{axis}] curve: {c[0]:+.3f} {c[1]:+.3f}*u {c[2]:+.3f}*u^2   "
                  f"(fit error at most {worst:.3f})")
            # glide back to centre before the next axis
            for i in range(1, 26):
                hold(cp_now * (1 - i / 25))
                nap(0.02)
            force[idx] = 0.0
            setf()
            nap(0.8)
        SH.target = None
        if not measured:
            say("Nothing could be measured - the old calibration is kept, nothing saved.")
        else:
            cfg["gain"] = rh.effective_gain()
            save_config(cfg)
            SH.status = "Calibration done and saved - close the window."
    except KeyboardInterrupt:
        say("\nAborted, nothing saved.")
    finally:
        rh.closing = True                 # poll() mustn't raise during cleanup
        try:
            put_gain_back()
        except Exception:
            pass
        rh.close()


# --------------------------------------------------------------------------
# Running
# --------------------------------------------------------------------------
def make_forces(cfg, args, knob=None):
    """force(pos) from the calibrated curves, rescaled when the gain sliders
    differ from what they were at calibration (so the push AT THE STICK stays
    the same when you turn the gain down or up)."""
    def forces(pos):
        u = pos if pos else (0.0, 0.0)
        scale = knob.scale() if knob else 1.0
        eff = knob.effective() if knob else None
        # --max is the force at the stick (as at 100% gain)
        lim = min(1.0, args.max * 100.0 / max(eff, 1.0)) if eff else min(args.max, 1.0)
        r = curve_force(cfg["roll"], u[0]) * scale
        p = curve_force(cfg["pitch"], u[1]) * scale
        if knob:
            knob.clipped = max(abs(r), abs(p)) > lim
        return clamp(r, -lim, lim), clamp(p, -lim, lim)
    return forces


class GainKnob:
    """Optional: a pot assigned to an axis (e.g. Rx) drives the configurator's
    Master Gain slider. Also tracks the gains so forces can be rescaled."""

    def __init__(self, rh, cfg):
        self.rh, self.cfg = rh, cfg
        self.axis = cfg.get("gain_pot")
        self.lo, self.hi = cfg.get("gain_range") or [0, 100]
        g = rh.get_gains()
        self.master, self.const = g if g else (None, None)
        self.set_val = None
        self.exact = None
        self.next_write = 0.0
        self.next_read = time.time() + 1.0
        self.clipped = False
        self.warned_clip = 0.0
        self.warned_no_axis = False
        self.log_at = 0.0
        self.armed = False
        self.first_v = None
        self.beep_on = cfg.get("beep", True)
        self.beep_bucket = None
        if self.axis == "raw":
            POT_RAW["spec"] = cfg.get("gain_pot_raw")
            if not POT_RAW["spec"]:
                self.axis = None

    def pot_fraction(self):
        raw = self.rh.extra.get(self.axis)
        if raw is None:
            return None
        if self.axis == "raw":                     # located by --find-pot
            spec = POT_RAW["spec"]
            span = spec["max"] - spec["min"]
            return clamp((raw - spec["min"]) / span, 0.0, 1.0) if span else None
        if self.axis == "z":                       # int16, -4096..4096
            return clamp((raw + 4096) / 8192, 0.0, 1.0)
        return clamp(raw / 255, 0.0, 1.0)          # uint8 axes

    def update(self):
        now = time.time()
        if self.axis:
            v = self.pot_fraction()
            # Safety: don't touch master gain until the pot has visibly moved,
            # so a wrong or dead axis can never zero the gain.
            if v is not None and not self.armed:
                if self.first_v is None:
                    self.first_v = v
                    print("\n[GAIN] Waiting for you to turn the pot before it takes over "
                          "master gain.")
                if abs(v - self.first_v) >= 0.03:
                    self.armed = True
                    print("\n[GAIN] Pot detected - master gain now follows it.")
                    if self.beep_on:
                        pc_beep(880, 60)
                else:
                    v = "wait"
            if v == "wait":
                pass
            elif v is None:
                if not self.warned_no_axis and now > self.next_read:
                    self.warned_no_axis = True
                    print(f"\n[WARN] No value from the {self.axis} axis yet - is the pot "
                          f"assigned to it in the configurator?")
            else:
                exact = self.lo + v * (self.hi - self.lo)
                self.exact = exact if self.exact is None else self.exact + 0.2 * (exact - self.exact)
                target = int(round(self.exact))
                # change only on a real move (hysteresis) and at most 10x a second
                if (self.set_val is None or abs(self.exact - self.set_val) > 0.7) \
                        and now >= self.next_write:
                    first = self.set_val is None
                    self.rh.set_master_gain(target)
                    self.set_val = self.master = target
                    self.next_write = now + 0.1
                    self.maybe_beep(target)
                    self.log_at = now + 0.6          # report once the pot stops moving
                    if first:
                        self.log_at = 0.0
                        print(f"\n[GAIN] Master gain set to {target}% from the pot "
                              f"({self.axis} = {self.rh.extra.get(self.axis)})")
        if self.log_at and now >= self.log_at:
            self.log_at = 0.0
            print(f"\n[GAIN] Master gain now {self.set_val}% "
                  f"({self.axis} = {self.rh.extra.get(self.axis)})")
        if now >= self.next_read:                  # pick up slider changes made elsewhere
            g = self.rh.get_gains()
            if g:
                if self.axis and self.set_val is not None and g[0] != self.set_val \
                        and now >= self.next_write + 0.3:
                    self.rh.set_master_gain(self.set_val)   # the pot is in charge
                else:
                    self.master = g[0]
                self.const = g[1]
            self.next_read = now + 1.0
        if self.clipped and now - self.warned_clip > 10:
            self.warned_clip = now
            print(f"\n[WARN] At {self.master}% master gain the script can't push hard enough "
                  f"to fully hold the stick (limit --max). Turn the gain up, or allow more "
                  f"with --max.")

    def maybe_beep(self, gain):
        """A beep each time the gain crosses a 10% step, higher for more gain."""
        bucket = int(gain // 10)
        if self.beep_bucket is None:
            self.beep_bucket = bucket
            return
        if bucket != self.beep_bucket:
            self.beep_bucket = bucket
            if self.beep_on:
                pc_beep(400 + 6 * gain, 40)        # 400 Hz at 0% ... 1000 Hz at 100%

    def effective(self):
        if self.master is None or self.const is None:
            return None
        return self.master * self.const / 100.0

    def scale(self):
        cal, now = self.cfg.get("gain"), self.effective()
        if not cal or now is None:
            return 1.0                             # calibrated before gain tracking
        if now <= 0:
            return 0.0
        return cal / now

    def text(self):
        if self.master is None:
            return ""
        t = f" | gain {self.master}%"
        if self.axis:
            raw = self.rh.extra.get(self.axis)
            t += f" (pot {self.axis} {raw if raw is not None else '?'})"
        return t


def run(args):
    cfg = load_config()
    if not cfg["pitch"]["points"] and not any(cfg["pitch"]["c"]):
        print("[INFO] Not calibrated yet - run with --calibrate first.")
    SH.cfg = cfg
    rh = Rhino()
    knob = GainKnob(rh, cfg)
    forces = make_forces(cfg, args, knob)
    rh.average_pos(0.2)
    say("Gravity compensation ON - force follows the stick position.")
    if knob.axis:
        print(f"Master gain follows the pot on axis {knob.axis} "
              f"({knob.lo}-{knob.hi}%). It stays where it is when the script stops.")
    if not cfg.get("gain"):
        print("[INFO] Calibrated before gain tracking existed: the force won't follow gain "
              "changes until you run --calibrate once more.")
    print("Keep this running while flying.  R = re-create the effect if the stick sags again,"
          "  G = show/hide the live window,  Ctrl+C = stop.")
    try:
        rh.ramp_to(*forces(rh.pos))
        next_check = time.time() + args.check
        next_status = 0.0
        while True:
            if rh.poll():                       # the sim wiped the effects
                time.sleep(0.3)
                rh.check_alive()
                next_check = time.time() + args.check
            knob.update()
            rh.set_force(*forces(rh.pos))
            while SH.keys:
                if SH.keys.popleft().lower() == "r":
                    rh.recreate()
            now = time.time()
            if args.check > 0 and now >= next_check:
                rh.check_alive()
                next_check = now + args.check
            if now >= next_status and rh.pos:
                print(f"\rstick roll {rh.pos[0]:+.3f} pitch {rh.pos[1]:+.3f} | "
                      f"force roll {rh.fx:+.3f} pitch {rh.fy:+.3f}{knob.text()}   ",
                      end="", flush=True)
                next_status = now + 0.25
            time.sleep(0.004)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        rh.close()


def tune(args):
    """Shift the whole curve up/down per axis (fine trim after calibrating)."""
    if not msvcrt and not SH.gui_available:
        sys.exit("--tune needs a Windows console or the live window for its keys.")
    cfg = load_config()
    SH.cfg = cfg
    rh = Rhino()
    knob = GainKnob(rh, cfg)
    forces = make_forces(cfg, args, knob)
    print("\n=== TRIM ===  let go of the stick; it should stay where you leave it")
    print("  w/s  shift pitch force +/- (W/S = big steps)")
    print("  a/d  shift roll force  +/- (A/D = big steps)")
    print("  ENTER save & exit     q quit without saving     G show/hide window\n")
    save = False
    try:
        rh.average_pos(0.2)
        rh.ramp_to(*forces(rh.pos))
        next_status = 0.0
        while True:
            rh.poll()
            knob.update()
            rh.set_force(*forces(rh.pos))
            if rh.pos and time.time() >= next_status:
                print(f"\rshift roll {cfg['roll']['c'][0]:+.3f} pitch {cfg['pitch']['c'][0]:+.3f}"
                      f" | stick roll {rh.pos[0]:+.3f} pitch {rh.pos[1]:+.3f}   ",
                      end="", flush=True)
                next_status = time.time() + 0.1
            while SH.keys:
                ch = SH.keys.popleft()
                step = 0.01 if ch.isupper() else 0.002
                c = ch.lower()
                if ch in "\r\n":
                    save = True
                    raise KeyboardInterrupt
                if c == "q":
                    raise KeyboardInterrupt
                k = knob.scale() or 1.0            # trim in "at the stick" units
                if c == "w": cfg["pitch"]["c"][0] += step / k
                if c == "s": cfg["pitch"]["c"][0] -= step / k
                if c == "d": cfg["roll"]["c"][0] += step / k
                if c == "a": cfg["roll"]["c"][0] -= step / k
            time.sleep(0.004)
    except KeyboardInterrupt:
        pass
    finally:
        rh.close()
    if save:
        save_config(cfg)
    else:
        print("\nNot saved.")


# --------------------------------------------------------------------------
# Live window (tkinter, ships with Python on Windows). The force work runs in
# a background thread; the window only shows what it's doing.
# --------------------------------------------------------------------------
def app(worker, title, show, keep_open_when_done):
    try:
        import tkinter as tk
    except ImportError:
        SH.gui_available = False
        if show:
            print("[INFO] tkinter not available - running without the live window.")
        try:
            worker()
        except KeyboardInterrupt:
            pass
        return
    SH.gui_available = True

    BG, PANEL, GRID, AX, CURVE, PTS, LIVE, NOW, TXT, WARN = (
        "#1e1f24", "#16171b", "#33353d", "#6b6f7b", "#c084fc", "#f5f5f5",
        "#facc15", "#22d3ee", "#d4d4d8", "#fb923c")

    root = tk.Tk()
    root.title(title)
    root.configure(bg=BG)
    cv = tk.Canvas(root, width=900, height=470, bg=BG, highlightthickness=0)
    cv.pack(padx=8, pady=(8, 0))
    live = tk.Label(root, text="", bg=BG, fg=NOW, anchor="w", font=("Consolas", 10))
    live.pack(fill="x", padx=12, pady=(4, 2))
    logbox = tk.Text(root, height=11, bg=PANEL, fg=TXT, insertbackground=TXT, relief="flat",
                     font=("Consolas", 9), wrap="word", padx=8, pady=6)
    logbox.pack(fill="both", expand=True, padx=8, pady=(0, 4))
    logbox.tag_configure("warn", foreground=WARN)
    logbox.configure(state="disabled")
    hint = tk.Label(root, bg=BG, fg=AX, anchor="w", font=("Segoe UI", 9),
                    text="G: hide this window (G in the console brings it back)   "
                         "Ctrl+C in the console: stop")
    hint.pack(fill="x", padx=12, pady=(0, 6))

    state = {"visible": show, "log_rev": -1}
    if not show:
        root.withdraw()

    # --- stick box ----------------------------------------------------
    BX, BY, BS = 30, 50, 380

    def box_xy(r, p):
        return BX + (r + 1) / 2 * BS, BY + (p + 1) / 2 * BS

    # --- plots ----------------------------------------------------------
    PLOTS = {"pitch": (470, 40, 410, 180), "roll": (470, 270, 410, 180)}  # x, y, w, h

    def plot_scale(axis):
        cfg = SH.cfg or {}
        cur = cfg.get(axis) or empty_curve()
        vals = [abs(f) for _, f in cur.get("points", [])] + [abs(f) for _, f in SH.live[axis]]
        vals += [abs(curve_force(cur, u / 10)) for u in range(-10, 11)]
        vals.append(abs(SH.force[AXES[axis]]))
        return max(vals + [0.1]) * 1.25

    def to_plot(axis, u, f, m):
        x0, y0, w, h = PLOTS[axis]
        return x0 + (u + 1) / 2 * w, y0 + h / 2 - f / m * (h / 2)

    def draw():
        cv.delete("all")
        cv.create_text(BX, 22, text="Stick position  (dot)   +   applied force  (arrow)",
                       fill=TXT, anchor="w", font=("Segoe UI", 10, "bold"))
        cv.create_rectangle(BX, BY, BX + BS, BY + BS, outline=AX)
        for i in range(1, 4):
            g = BX + BS * i / 4
            cv.create_line(g, BY, g, BY + BS, fill=GRID)
            g = BY + BS * i / 4
            cv.create_line(BX, g, BX + BS, g, fill=GRID)
        cx, cy = box_xy(0, 0)
        cv.create_line(cx, BY, cx, BY + BS, fill=AX)
        cv.create_line(BX, cy, BX + BS, cy, fill=AX)
        cv.create_text(cx, BY + BS + 14, text="roll", fill=AX, font=("Segoe UI", 9))
        cv.create_text(BX - 14, cy, text="pitch", fill=AX, angle=90, font=("Segoe UI", 9))
        if SH.target:
            axis, t = SH.target
            r, p = (t, 0) if axis == "roll" else (0, t)
            tx, ty = box_xy(r, p)
            cv.create_oval(tx - 9, ty - 9, tx + 9, ty + 9, outline=LIVE, width=2)
        if SH.pos:
            px, py = box_xy(*SH.pos)
            fr, fp = SH.force
            cv.create_line(px, py, px + fr * BS * 0.4, py + fp * BS * 0.4,
                           fill=CURVE, width=3, arrow="last")
            cv.create_oval(px - 7, py - 7, px + 7, py + 7, fill=NOW, outline="")
            cv.create_text(BX + BS, BY - 10, anchor="e", fill=TXT, font=("Consolas", 9),
                           text=f"roll {SH.pos[0]:+.3f}  pitch {SH.pos[1]:+.3f}")
        for axis, (x0, y0, w, h) in PLOTS.items():
            idx = AXES[axis]
            m = plot_scale(axis)
            cv.create_text(x0, y0 - 14, anchor="w", fill=TXT, font=("Segoe UI", 10, "bold"),
                           text=f"{axis.capitalize()}: force needed vs stick position")
            cv.create_text(x0 + w, y0 - 14, anchor="e", fill=AX, font=("Consolas", 9),
                           text=f"+-{m:.2f}")
            cv.create_rectangle(x0, y0, x0 + w, y0 + h, outline=AX)
            cv.create_line(x0, y0 + h / 2, x0 + w, y0 + h / 2, fill=GRID)
            cv.create_line(x0 + w / 2, y0, x0 + w / 2, y0 + h, fill=GRID)
            cur = (SH.cfg or {}).get(axis)
            if cur and (cur.get("points") or any(cur["c"])):
                pts = []
                for i in range(101):
                    u = -1 + 2 * i / 100
                    pts += to_plot(axis, u, curve_force(cur, u), m)
                cv.create_line(*pts, fill=CURVE, width=2, smooth=True)
                for u, f in cur.get("points", []):
                    x, y = to_plot(axis, u, f, m)
                    cv.create_oval(x - 3, y - 3, x + 3, y + 3, outline=PTS)
            for u, f in SH.live[axis]:
                x, y = to_plot(axis, u, f, m)
                cv.create_oval(x - 4, y - 4, x + 4, y + 4, fill=LIVE, outline="")
            if SH.pos:
                u, f = SH.pos[idx], SH.force[idx]
                x, _ = to_plot(axis, u, 0, m)
                cv.create_line(x, y0, x, y0 + h, fill=NOW, dash=(3, 3))
                x, y = to_plot(axis, u, f, m)
                cv.create_oval(x - 5, y - 5, x + 5, y + 5, fill=NOW, outline="")
                cv.create_text(x0 + 4, y0 + h - 4, anchor="sw", fill=TXT, font=("Consolas", 9),
                               text=f"at {u:+.3f}: force {f:+.3f}")

    def refresh_log():
        with SH.lock:
            rev, lines, cur = SH.log_rev, list(SH.log), SH.live_line
        live.config(text=cur.strip()[:150])
        if rev == state["log_rev"]:
            return
        state["log_rev"] = rev
        at_bottom = logbox.yview()[1] > 0.99
        logbox.configure(state="normal")
        logbox.delete("1.0", "end")
        for ln in lines:
            bad = any(w in ln for w in ("[WARN]", "[ERROR]", "Aborted", "not settled",
                                        "can't", "Something else"))
            logbox.insert("end", ln.strip() + "\n", "warn" if bad else ())
        logbox.configure(state="disabled")
        if at_bottom or state["log_rev"] < 3:
            logbox.see("end")

    def set_visible(v):
        state["visible"] = v
        if v:
            root.deiconify()
            root.lift()
        else:
            root.withdraw()

    def tick():
        if SH.toggle_gui:
            SH.toggle_gui = False
            set_visible(not state["visible"])
        if SH.done and not (keep_open_when_done and state["visible"]):
            root.quit()
            return
        if state["visible"]:
            draw()
            refresh_log()
        root.after(40, tick)

    def on_close():
        if SH.done:
            root.quit()
        else:
            set_visible(False)
            print("\n[INFO] Live window hidden - press G in the console to show it again.")

    def on_key(e):
        if e.char in ("g", "G"):
            set_visible(False)
        elif e.char:
            SH.keys.append(e.char)

    def run_worker():
        try:
            worker()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"\n[ERROR] {e}")
        finally:
            SH.done = True
            if keep_open_when_done:
                print("Finished - close the window to exit.")

    # Ctrl+C in the console: ask the worker to stop and clean up
    try:
        signal.signal(signal.SIGINT, lambda *_: SH.stop.set())
    except Exception:
        pass
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.bind("<Key>", on_key)
    t = threading.Thread(target=run_worker, daemon=True)
    t.start()
    root.after(40, tick)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    # let the worker remove its effects from the stick before exiting
    SH.stop.set()
    t.join(timeout=10)
    try:
        root.destroy()
    except Exception:
        pass


def find_pot(args):
    """Locate the pot in the stick's input report: the user turns it end to
    end while the stick is left alone, and we look at which bytes changed."""
    rh = Rhino()
    print("\n=== FIND THE POT ===")
    print("Leave the stick and buttons alone. Within the next 15 seconds, turn the pot slowly")
    print("from one end to the other and back, twice. (Ctrl+C aborts)\n")
    samples, end, next_dot = [], time.time() + 15, 0
    try:
        while time.time() < end:
            rh.poll()
            if rh.last_input:
                samples.append(bytes(rh.last_input))
            if time.time() >= next_dot:
                print(f"\r{int(end - time.time()):2d} s left, {len(samples)} reports   ",
                      end="", flush=True)
                next_dot = time.time() + 0.5
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\nAborted.")
        rh.close()
        return
    rh.close()
    print()
    if len(samples) < 50:
        print("Too few reports from the stick to tell anything.")
        return
    n = min(len(r) for r in samples)
    changed = [i for i in range(5, n) if len({r[i] for r in samples}) > 1]   # skip id, X, Y
    print("Bytes that changed:", ", ".join(f"{i} ({min(r[i] for r in samples)}-"
                                            f"{max(r[i] for r in samples)})" for i in changed)
          or "none")
    # candidates: single bytes and little-endian 16-bit pairs
    cands = []
    for i in changed:
        for fmt, size in (("<B", 1), ("<H", 2), ("<h", 2)):
            if i + size > n:
                continue
            vals = [struct.unpack_from(fmt, r, i)[0] for r in samples]
            lo, hi = min(vals), max(vals)
            if hi - lo < 3:
                continue
            # a real pot moves smoothly: big jumps between reports mean we are
            # looking at the low byte of a wider value, or a wrong sign
            steps = sorted(abs(a - b) for a, b in zip(vals, vals[1:]))
            typical_big = steps[int(0.99 * (len(steps) - 1))]   # ignore rare spikes
            smooth = typical_big <= 0.15 * (hi - lo)
            full = 255 if size == 1 else 65535
            cands.append({"offset": i, "fmt": fmt, "min": lo, "max": hi,
                          "smooth": smooth, "score": (hi - lo) / full * (10 if smooth else 1)
                          + (0.5 if size == 2 and smooth else 0)})
    good = [c for c in cands if c["smooth"]]
    if not good:
        print("Couldn't find anything that moves like a pot. Is it assigned to an axis in the "
              "configurator (Pot#1 Function)?")
        return
    best = max(good, key=lambda c: c["score"])
    kind = {"<B": "8-bit", "<H": "16-bit", "<h": "16-bit signed"}[best["fmt"]]
    print(f"Found the pot: byte {best['offset']}, {kind}, range {best['min']}-{best['max']}.")
    cfg = load_config()
    cfg["gain_pot"] = "raw"
    cfg["gain_pot_raw"] = {k: best[k] for k in ("offset", "fmt", "min", "max")}
    save_config(cfg)
    print("Master gain will follow this pot from now on (--gain-pot off to stop).")


def main():
    ap = argparse.ArgumentParser(description="Position-aware anti-gravity force for a VPforce Rhino")
    ap.add_argument("--calibrate", action="store_true", help="measure gravity across the travel")
    ap.add_argument("--tune", action="store_true", help="trim the curve by hand")
    ap.add_argument("--pitch-only", action="store_true", help="calibrate pitch only")
    ap.add_argument("--gui", action="store_true",
                    help="start with the live window open (default when calibrating)")
    ap.add_argument("--no-gui", action="store_true", help="start with the live window hidden")
    ap.add_argument("--clear", action="store_true",
                    help="remove every force effect from the stick (close all sims first)")
    ap.add_argument("--points", type=int, default=7, help="positions per axis (7)")
    ap.add_argument("--range", type=float, default=0.7,
                    help="how far out to measure, fraction of full travel (0.7)")
    ap.add_argument("--max", type=float, default=0.6, help="force limit, fraction of full (0.6)")
    ap.add_argument("--tolerance", type=float, default=0.006, help="calibration: close enough")
    ap.add_argument("--find-pot", action="store_true",
                    help="locate the pot in the stick's data (turn it end to end) and use it "
                         "for master gain")
    ap.add_argument("--gain-pot", choices=POT_AXES + ("off",),
                    help="let a pot assigned to this axis drive the Master Gain slider "
                         "(saved; 'off' to stop)")
    ap.add_argument("--gain-range", type=int, nargs=2, metavar=("LOW", "HIGH"),
                    help="master gain at the pot's two ends, in percent (saved; default 0 100)")
    ap.add_argument("--beep", choices=("on", "off"),
                    help="beep on the PC when the pot changes master gain (saved)")
    ap.add_argument("--check", type=float, default=1.0,
                    help="seconds between 'effect still alive?' checks, 0 = off")
    args = ap.parse_args()
    args.points = max(3, args.points)
    if args.gain_pot or args.gain_range or args.beep:
        cfg = load_config()
        if args.beep:
            cfg["beep"] = args.beep == "on"
        if args.gain_pot:
            cfg["gain_pot"] = None if args.gain_pot == "off" else args.gain_pot
        if args.gain_range:
            lo, hi = (int(clamp(v, 0, 100)) for v in args.gain_range)
            cfg["gain_range"] = [lo, hi]
        save_config(cfg)
        print(f"Gain pot: {cfg.get('gain_pot') or 'off'}, range {cfg.get('gain_range') or [0, 100]}%,"
              f" beeps {'on' if cfg.get('beep', True) else 'off'}")
        return
    sys.stdout = Tee(sys.stdout)
    if args.find_pot:
        find_pot(args)
        return
    if args.clear:
        rh = Rhino()
        rh.reset_device()
        rh.close()
        print("Stick cleared: all force effects removed.")
        return
    if args.calibrate:
        work, title = (lambda: calibrate(args)), "Rhino gravity compensation - calibrating"
    elif args.tune:
        work, title = (lambda: tune(args)), "Rhino gravity compensation - trim"
    else:
        work, title = (lambda: run(args)), "Rhino gravity compensation"
    show = (args.gui or args.calibrate) and not args.no_gui
    app(work, title, show, keep_open_when_done=args.calibrate)


if __name__ == "__main__":
    main()
