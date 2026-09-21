"""Parity: the native (C++ Track) chase camera == the pure-Python ChaseCamera.

Goal (d) is to unify the cvcGL and grl-snam cameras so they "work the same" in
C++ AND Python. The C++ side is pinned by cvcgl_track_parity (Track vs the
ChaseCamera algorithm, 7e-15). This pins the PYTHON side: one position stream fed
to both pycvc_gl.camera.ChaseCamera (pure Python) and NativeChaseCamera (delegates
to the C++ Track controller) must return the same pose. It is the seam a demo
swaps between with one import.

Needs the built pycvc_gl extension for the native side; if it cannot construct
(no extension / no headless CameraController), the test SKIPs cleanly.
"""

import math
import os
import sys

import pycvc
from pycvc_gl.camera import ChaseCamera, NativeChaseCamera

fails = 0


def check(what, ok):
    global fails
    print("  [%s] %s" % ("PASS" if ok else "FAIL", what))
    if not ok:
        fails += 1


# Deliberately non-default params, to exercise the state plumbing on the native
# side too (not just the shared defaults).
params = dict(
    back=48.0,
    height=28.0,
    look_ahead=6.0,
    look_up=4.0,
    pos_tau=0.12,
    vel_tau=0.5,
    cam_tau=0.35,
    min_speed=0.08,
)

pure = ChaseCamera(**params)
try:
    native = NativeChaseCamera(**params, app=pycvc.make_app(), state_path="parity.cam")
except Exception as e:  # extension missing / no headless controller
    print("SKIP: native chase camera unavailable — %s" % e)
    sys.stdout.flush()
    os._exit(0)

# A continuous, varying-speed wander (dips toward a near-stop, never teleports),
# on irregular tick spacing — the same shape as the C++ parity test.
max_err = 0.0
phase = 0.0
for i in range(200):
    dt = 0.03 + 0.012 * math.sin(1.3 * i * 0.05)
    speed = 0.6 + 0.55 * math.cos(0.08 * i)
    phase += speed * dt * 0.25
    pos = (60.0 * math.cos(phase), 40.0 * math.sin(1.3 * phase), 3.0 + 2.0 * math.sin(0.5 * phase))
    pe, pt, _ = pure.update(pos, dt)
    ne, nt, _ = native.update(pos, dt)
    for k in range(3):
        max_err = max(max_err, abs(pe[k] - ne[k]), abs(pt[k] - nt[k]))

print("  max abs pose error over 200 frames: %.3e" % max_err)
check("native chase camera matches pure-Python ChaseCamera", max_err < 1e-6)

print("\n%s (%d failures)" % ("ALL PASS" if fails == 0 else "FAILED", fails))
# Hard-exit past cvcGL's harmless static-teardown race, like the other pycvc_gl tests.
sys.stdout.flush()
os._exit(1 if fails else 0)
