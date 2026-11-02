"""form.py — a two-way form: form.ari's inputs go to Python computations, and the results come back.

    python form.py                          # a window: move the sliders, press the buttons
    python form.py --offscreen --png form.png
    python form.py --headless

- (trajectory SPEED ANGLE DRAG) is a function: it runs inline in the frame and returns a dict,
  which the document writes as a subtree (state-set-tree) and shows.
- (spread N SPEED ANGLE DRAG) is an async function: a Monte Carlo over noisy launches that runs on
  the app's background task pool while the calling program waits and the UI keeps drawing.
- A watch on the inputs recomputes a one-line `preview` whenever a slider moves, with no button.

The document passes its inputs as state text (state-get), so the functions convert them.
"""

import math
import os
import sys

import numpy as np

from pycvc_gl.ariadne import Host, main_args

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81


def fly(speed, angle_deg, drag, dt=0.01, max_t=60.0):
    """A point mass with linear drag, launched from the ground: (xs, ys, flight time)."""
    a = math.radians(angle_deg)
    vx, vy = speed * math.cos(a), speed * math.sin(a)
    x = y = t = 0.0
    xs, ys = [0.0], [0.0]
    while t < max_t:
        vx += -drag * vx * dt
        vy += (-G - drag * vy) * dt
        x, y, t = x + vx * dt, y + vy * dt, t + dt
        if y < 0.0:
            break
        xs.append(x)
        ys.append(y)
    return xs, ys, t


def main(argv=None):
    args = main_args(argv)
    host = Host(mode=args.mode, name="form", width=args.width, height=args.height)

    @host.function("trajectory")
    def trajectory(speed, angle, drag):
        xs, ys, t = fly(float(speed), float(angle), float(drag))
        step = max(1, len(ys) // 120)  # the plot: 120 points or fewer
        return {
            "range": round(xs[-1], 2),
            "apex": round(max(ys), 2),
            "flight": round(t, 2),
            "heights": ",".join(f"{h:.2f}" for h in ys[::step]),
        }

    @host.async_function("spread")
    def spread(n, speed, angle, drag):
        # Runs on a pool thread. It is plain Python, so it shares the GIL with the frame loop (the
        # interpreter switches between them), and the frames keep drawing while it runs.
        rng = np.random.default_rng(1)
        n = max(1, int(float(n)))
        ranges = np.empty(n)
        for i in range(n):
            s = float(speed) * (1 + 0.05 * rng.standard_normal())
            a = float(angle) + 2.0 * rng.standard_normal()
            ranges[i] = fly(s, a, float(drag), dt=0.02)[0][-1]
        return {"mean": round(float(ranges.mean()), 2), "std": round(float(ranges.std()), 2),
                "n": n}

    @host.watch("speed")
    @host.watch("angle")
    @host.watch("drag")
    def preview(_value):
        s, a = host.get_float("speed"), host.get_float("angle")
        host.set("preview", f"no-drag range {s * s * math.sin(math.radians(2 * a)) / G:.1f} m")

    host.load(os.path.join(HERE, "form.ari"))  # its warnings are logged

    if args.mode != "window":  # press both buttons, as a user would
        host.set("samples", 400)
        host.run_program('(state-set-tree "result" (trajectory (state-get "speed") '
                         '(state-get "angle") (state-get "drag")))')
        host.run_program('(begin (state-set "mc.status" "running") (state-set-tree "spread" '
                         '(spread (state-get "samples") (state-get "speed") (state-get "angle") '
                         '(state-get "drag"))) (state-set "mc.status" "done"))')
        host.run(until=lambda: host.get("mc.status") == "done", frames=3000, realtime=False)
    host.run(frames=args.frames or None)
    if args.png:
        host.snapshot(args.png)
    print("result:", host.get_tree("result"))
    print("spread:", host.get_tree("spread"))
    print("preview:", host.get("preview"))
    code = host.exit_code()
    host.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
