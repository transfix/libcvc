#!/usr/bin/env python3
"""Write the committed inputs the legacy libraries run on (tools/volrover-goldens/README.md).

Run once; the files it writes are committed, so the goldens never depend on how this machine's libm
rounds a Gaussian or a sine. Each GenSeg case is inputs/genseg/<case>.raw (float32, little-endian,
x fastest) and <case>.json (its dimensions, thresholds and seeds); each Skeletonization case is
inputs/skeleton/<case>.pts (a point a line) and <case>.json. Pure Python, no numpy.

    python3 tools/volrover-goldens/make_inputs.py                  # every library
    python3 tools/volrover-goldens/make_inputs.py --only skeleton  # one library
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


# ---- Skeletonization: point clouds sampled from closed surfaces ----
# Each case is inputs/skeleton/<case>.pts, one point a line, written as the exact decimal of a
# float (the legacy Geometry keeps floats), so a float parser and a double parser read the same
# value; and <case>.json, the case and the parameters both sides run with (the legacy names).

def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def torus(nu, nv, R, r, seed):
    rnd = lcg(seed)
    pts = []
    for i in range(nu):
        for k in range(nv):
            u = 2 * math.pi * (i + 0.3 * (next(rnd) - 0.5)) / nu
            v = 2 * math.pi * (k + 0.3 * (next(rnd) - 0.5)) / nv
            pts.append(((R + r * math.cos(v)) * math.cos(u), (R + r * math.cos(v)) * math.sin(u),
                        r * math.sin(v)))
    return pts


def capsule(length, r, nring, nalong, seed):
    rnd = lcg(seed)
    pts = []
    for i in range(nalong + 1):
        jx = 0.3 * (next(rnd) - 0.5) if 0 < i < nalong else 0.0
        for k in range(nring):
            a = 2 * math.pi * (k + 0.3 * (next(rnd) - 0.5)) / nring
            pts.append((length * (i + jx) / nalong, r * math.cos(a), r * math.sin(a)))
    ncap = nring // 4
    for side in (0, 1):
        for i in range(1, ncap + 1):
            t = (math.pi / 2) * (i + 0.1 * (next(rnd) - 0.5)) / (ncap + 0.5)
            nr = max(1, int(round(nring * math.cos(t))))
            for k in range(nr):
                a = 2 * math.pi * (k + 0.3 * (next(rnd) - 0.5)) / nr
                x = -r * math.sin(t) if side == 0 else length + r * math.sin(t)
                pts.append((x, r * math.cos(t) * math.cos(a), r * math.cos(t) * math.sin(a)))
    return pts


def dumbbell(seed):
    """Two spheres of radius 2 at x = 0 and x = 8, joined by a rod of radius 0.7: each part's
    sample points that lie inside neither other part."""
    rnd = lcg(seed)
    R, rod = 2.0, 0.7
    pts = []

    def inside_other(p, part):
        x, y, z = p
        in_a = x * x + y * y + z * z < (R - 1e-9) ** 2
        in_b = (x - 8) ** 2 + y * y + z * z < (R - 1e-9) ** 2
        in_rod = 0 < x < 8 and y * y + z * z < (rod - 1e-9) ** 2
        return (part != "a" and in_a) or (part != "b" and in_b) or (part != "rod" and in_rod)

    nphi, nth = 40, 20
    for cx, part in ((0.0, "a"), (8.0, "b")):
        for i in range(nth):
            th = math.pi * (i + 0.5 + 0.3 * (next(rnd) - 0.5)) / nth
            n = max(3, int(round(nphi * math.sin(th))))
            for k in range(n):
                ph = 2 * math.pi * (k + 0.3 * (next(rnd) - 0.5)) / n
                p = (cx + R * math.cos(th), R * math.sin(th) * math.cos(ph),
                     R * math.sin(th) * math.sin(ph))
                if not inside_other(p, part):
                    pts.append(p)
    nring, nalong = 14, 32
    for i in range(nalong + 1):
        for k in range(nring):
            a = 2 * math.pi * (k + 0.3 * (next(rnd) - 0.5)) / nring
            p = (8.0 * (i + 0.3 * (next(rnd) - 0.5)) / nalong, rod * math.cos(a), rod * math.sin(a))
            if not inside_other(p, "rod"):
                pts.append(p)
    return pts


def box_grid(nx, ny, nz, h):
    """A box's surface sampled on a lattice: exactly cospherical subsets everywhere."""
    pts = []
    for i in range(nx + 1):
        for j in range(ny + 1):
            for k in range(nz + 1):
                if 0 < i < nx and 0 < j < ny and 0 < k < nz:
                    continue
                pts.append((i * h, j * h, k * h))
    return pts


SKELETON_CASES = {
    # In general position: the samples are jittered.
    "torus": dict(points=lambda: torus(48, 16, 4.0, 1.2, 11), general=True),
    "capsule": dict(points=lambda: capsule(8.0, 1.0, 20, 40, 12), general=True),
    "dumbbell": dict(points=lambda: dumbbell(13), general=True),
    # Exactly cospherical: compared by topology.
    "box_grid": dict(points=lambda: box_grid(8, 6, 2, 0.5), general=False),
}


def write_skeleton_inputs():
    out_dir = os.path.join(HERE, "inputs", "skeleton")
    os.makedirs(out_dir, exist_ok=True)
    for name, c in SKELETON_CASES.items():
        pts = [tuple(f32(v) for v in p) for p in c["points"]()]
        with open(os.path.join(out_dir, name + ".pts"), "w", newline="\n") as f:
            for p in pts:
                f.write("%.17g %.17g %.17g\n" % p)
        meta = {
            "library": "Skeletonization",
            "points": len(pts),
            "general_position": c["general"],
            "parameters": {},
            "made_by": "tools/volrover-goldens/make_inputs.py",
        }
        with open(os.path.join(out_dir, name + ".json"), "w", newline="\n") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        print("wrote", name, len(pts), "points")


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--only", choices=["genseg", "skeleton"],
                    help="write only one library's inputs (the others stay as committed)")
    args = ap.parse_args()
    if args.only != "genseg":
        write_skeleton_inputs()
    if args.only == "skeleton":
        return
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
