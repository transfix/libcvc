"""data_feed.py — Python pushes data into data_feed.ari every frame: two series the document plots,
a table, a numpy picture, and a live video stream; and reads the document's controls back.

    python data_feed.py                           # a window
    python data_feed.py --offscreen --png feed.png
    python data_feed.py --headless --frames 60    # no GL: no picture or stream drawn

The Python counterpart of examples/ariadne/hosts/ariadne_stream.cpp, with plots and a table too.
"""

import collections
import math
import os
import sys

import numpy as np

from pycvc_gl.ariadne import Host, main_args

HERE = os.path.dirname(os.path.abspath(__file__))
HISTORY = 240  # samples each plot keeps
HEAT = 128  # the picture's size
STREAM_W, STREAM_H = 320, 240


def heat_picture(t):
    """A moving pair of Gaussians as an RGB picture (a cheap colormap: blue -> red)."""
    y, x = np.mgrid[0:HEAT, 0:HEAT] / (HEAT - 1.0)
    a = np.exp(-((x - 0.5 - 0.3 * math.cos(t)) ** 2 + (y - 0.5 - 0.3 * math.sin(t)) ** 2) / 0.02)
    b = np.exp(-((x - 0.5 + 0.25 * math.sin(1.3 * t)) ** 2 + (y - 0.5) ** 2) / 0.05)
    v = np.clip(a + 0.6 * b, 0.0, 1.0)
    return np.stack([v, 0.25 * np.sqrt(v), 1.0 - v], axis=2)  # float in [0, 1]


def stream_frame(t):
    """A moving test pattern, (H, W, 4) uint8 RGBA. A new array each frame: publish() hands the
    stream the array's own buffer (no copy), so it must not change while a frame shows it."""
    frame = np.empty((STREAM_H, STREAM_W, 4), np.uint8)
    x = np.arange(STREAM_W)[None, :]
    y = np.arange(STREAM_H)[:, None]
    frame[..., 0] = (128 + 127 * np.sin(x / 17.0 + 3 * t)).astype(np.uint8)
    frame[..., 1] = (128 + 127 * np.sin(y / 11.0 - 2 * t)).astype(np.uint8)
    frame[..., 2] = ((x + y + int(60 * t)) % 256).astype(np.uint8)
    frame[..., 3] = 255
    return frame


def main(argv=None):
    args = main_args(argv)
    host = Host(mode=args.mode, name="feed", width=args.width, height=args.height)
    stream = None
    if args.mode != "headless":  # before load(): the document's stream_view widget is registered
        stream = host.open_stream("feed", STREAM_W, STREAM_H)
    host.load(os.path.join(HERE, "data_feed.ari"))  # its warnings are logged

    rng = np.random.default_rng(7)
    signal = collections.deque(maxlen=HISTORY)
    walk = collections.deque([0.0], maxlen=HISTORY)
    clock = {"t": 0.0}

    @host.on_frame
    def feed(dt):
        if host.get_bool("paused"):  # the document's checkbox
            return
        t = clock["t"] = clock["t"] + dt
        freq, noise = host.get_float("freq", 0.5), host.get_float("noise", 0.0)
        signal.append(math.sin(2 * math.pi * freq * t) + noise * rng.standard_normal())
        walk.append(walk[-1] + rng.standard_normal() * 0.1)
        host.set_series("signal", signal)
        host.set_series("walk", walk)
        host.set("signal_caption", f"{signal[-1]:+.2f}")
        host.set("t", f"{t:.2f} s")
        if host.frame_count % 10 == 0:
            rows = []
            for name, s in (("signal", signal), ("walk", walk)):
                a = np.fromiter(s, float)
                rows.append({"series": name, "mean": f"{a.mean():.3f}", "min": f"{a.min():.3f}",
                             "max": f"{a.max():.3f}"})
            host.set_table("stats", rows)
        if host.frame_count % 5 == 0:
            host.publish_image("heat", heat_picture(t))
        if stream is not None:
            stream.publish(stream_frame(t), t)

    host.run(fps=30, frames=args.frames or None)
    if args.png:
        host.snapshot(args.png)
    print("samples:", len(host.get_series("signal")))
    print("stats rows:", host.get("stats"))
    code = host.exit_code()
    host.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
