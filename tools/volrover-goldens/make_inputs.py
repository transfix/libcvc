#!/usr/bin/env python3
"""Write the committed inputs the legacy libraries run on (tools/volrover-goldens/README.md).

Run once; the files it writes are committed, so the goldens never depend on how this machine's libm
rounds a Gaussian. Each GenSeg case is inputs/genseg/<case>.raw (float32, little-endian, x fastest)
and <case>.json (its dimensions, thresholds and seeds). Pure Python, no numpy.

    python3 tools/volrover-goldens/make_inputs.py
"""

import json
import math
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))


def lcg(seed):
    """A fixed generator (numerical recipes' LCG), the same on every platform."""
    state = seed & 0xFFFFFFFF
    while True:
        state = (1664525 * state + 1013904223) & 0xFFFFFFFF
        yield state / 4294967296.0


def volume(n, blobs, background=0.02, noise=0.0, seed=1):
    """n^3 floats: Gaussian blobs (centre, sigma, height) over a background, plus uniform noise."""
    rnd = lcg(seed)
    out = []
    for k in range(n):
        for j in range(n):
            for i in range(n):
                v = background
                for (cx, cy, cz), s, h in blobs:
                    d2 = (i - cx) ** 2 + (j - cy) ** 2 + (k - cz) ** 2
                    v += h * math.exp(-d2 / (2.0 * s * s))
                if noise:
                    v += noise * (next(rnd) - 0.5)
                out.append(max(v, 0.0))
    return out


CASES = {
    # Two touching blobs, a seed in each: two labels meeting between them.
    "two_blobs": dict(
        n=24,
        blobs=[((9, 10, 12), 3.5, 1.0), ((15, 14, 12), 3.5, 0.9)],
        noise=0.0,
        tlow=40,
        thigh=200,
        seeds=[(0, 9, 10, 12), (1, 15, 14, 12)],
    ),
    # Three classes on noisy data, so the automatic seeds (GenSeg's local maxima) join in.
    "three_classes_noisy": dict(
        n=24,
        blobs=[((6, 6, 6), 3.0, 1.0), ((17, 7, 15), 3.5, 0.8), ((10, 17, 11), 3.0, 0.7)],
        noise=0.06,
        tlow=30,
        thigh=180,
        seeds=[(0, 6, 6, 6), (1, 17, 7, 15), (2, 10, 17, 11)],
    ),
    # Two seeds in one class, one in the other: a class may have several seeds.
    "two_seeds_one_class": dict(
        n=20,
        blobs=[((5, 5, 10), 2.5, 1.0), ((14, 6, 10), 2.5, 0.95), ((10, 15, 10), 3.0, 0.85)],
        noise=0.02,
        tlow=35,
        thigh=200,
        seeds=[(0, 5, 5, 10), (0, 14, 6, 10), (1, 10, 15, 10)],
    ),
}


def main():
    out_dir = os.path.join(HERE, "inputs", "genseg")
    os.makedirs(out_dir, exist_ok=True)
    for name, c in CASES.items():
        vals = volume(c["n"], c["blobs"], noise=c["noise"], seed=len(name))
        with open(os.path.join(out_dir, name + ".raw"), "wb") as f:
            f.write(struct.pack("<%df" % len(vals), *vals))
        meta = {
            "library": "GenSeg",
            "dims": [c["n"]] * 3,
            "dtype": "float32-le",
            "tlow": c["tlow"],
            "thigh": c["thigh"],
            "seeds": [list(s) for s in c["seeds"]],
            "made_by": "tools/volrover-goldens/make_inputs.py",
        }
        with open(os.path.join(out_dir, name + ".json"), "w", newline="\n") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        print("wrote", name, c["n"], "^3")


if __name__ == "__main__":
    main()
