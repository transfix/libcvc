#!/usr/bin/env python3
"""Write the committed inputs the legacy libraries run on (tools/volrover-goldens/README.md).

Run once; the files it writes are committed, so the goldens never depend on how this machine's libm
rounds a Gaussian or a sine. Each GenSeg case is inputs/genseg/<case>.raw (float32, little-endian,
x fastest) and <case>.json (its dimensions, thresholds and seeds); each SegMed case is
inputs/segmed/<case>.raw and <case>.json (its dimensions and tlow); each Skeletonization case is
inputs/skeleton/<case>.pts (a point a line) and <case>.json; each PocketTunnel case is
inputs/pockettunnel/<case>.pts and <case>.json; each SecondaryStructures case is
inputs/secondary/<case>.pts and <case>.json; each TightCocone case is
inputs/tightcocone/<case>.json, with its own <case>.pts where it does not run on another
library's. Pure Python, no numpy.

    python3 tools/volrover-goldens/make_inputs.py                  # every library
    python3 tools/volrover-goldens/make_inputs.py --only skeleton  # one library
    python3 tools/volrover-goldens/make_inputs.py --only genseg --case auto_seeds_sweep
                                                                   # one case (the rest as
                                                                   # committed)
    python3 tools/volrover-goldens/make_inputs.py --only segmed    # SegMed's cases
    python3 tools/volrover-goldens/make_inputs.py --only tiler     # ContourTiler's series
    python3 tools/volrover-goldens/make_inputs.py --only secondary # SecondaryStructures' cases
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


def volume(n, blobs, background=0.02, noise=0.0, seed=1, shells=(), plateaus=()):
    """n^3 floats: Gaussian blobs (centre, sigma, height), spherical shells (centre, radius,
    sigma, height: a Gaussian profile across the shell) and flat-topped plateaus (centre, radius,
    sharpness p, height: h / (1 + (d / r)^p)) over a background, plus uniform noise."""
    rnd = lcg(seed)
    out = []
    for k in range(n):
        for j in range(n):
            for i in range(n):
                v = background
                for (cx, cy, cz), s, h in blobs:
                    d2 = (i - cx) ** 2 + (j - cy) ** 2 + (k - cz) ** 2
                    v += h * math.exp(-d2 / (2.0 * s * s))
                for (cx, cy, cz), r, s, h in shells:
                    d = math.sqrt((i - cx) ** 2 + (j - cy) ** 2 + (k - cz) ** 2) - r
                    v += h * math.exp(-d * d / (2.0 * s * s))
                for (cx, cy, cz), r, pw, h in plateaus:
                    d = math.sqrt((i - cx) ** 2 + (j - cy) ** 2 + (k - cz) ** 2)
                    v += h / (1.0 + (d / r) ** pw)
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
    # Automatic seeds away from every user seed, so the threshold sweep classifies them: two
    # chains of overlapping blobs, each seeded at one end, a blob between the chains (its seed
    # goes to whichever class reaches it at the higher threshold), an isolated blob no front
    # reaches, a seed on the volume's face, and class 1 with no seed at all.
    "auto_seeds_sweep": dict(
        n=28,
        blobs=[((6, 6, 6), 2.5, 1.0), ((11, 8, 7), 2.5, 0.8), ((16, 10, 9), 2.5, 0.7),
               ((20, 20, 20), 3.0, 0.9), ((14, 22, 18), 2.5, 0.75), ((12, 16, 13), 2.5, 0.6),
               ((24, 5, 22), 2.0, 0.8), ((0, 14, 12), 2.5, 0.7)],
        noise=0.05,
        tlow=25,
        thigh=110,
        seeds=[(0, 6, 6, 6), (2, 20, 20, 20), (2, 0, 14, 12)],
    ),
    # A blob touching a shell (the roadmap's case), a seed in each: the shell's ridge is full of
    # automatic seeds (its noise's local maxima), which the sweep gives the shell's class.
    "blob_touching_shell": dict(
        n=24,
        blobs=[((20, 12, 12), 2.2, 1.0)],
        shells=[((11, 12, 12), 7.0, 1.3, 0.8)],
        noise=0.08,
        tlow=35,
        thigh=140,
        seeds=[(0, 20, 12, 12), (1, 4, 12, 12)],
    ),
    # Heavy noise on a broad blob: dozens of automatic seeds (the noise's local maxima), reached
    # and classified one by one as the sweep comes down.
    "noisy_many_seeds": dict(
        n=24,
        blobs=[((12, 12, 12), 30.0, 1.0)],
        noise=0.45,
        tlow=60,
        thigh=140,
        seeds=[(0, 7, 12, 12), (1, 17, 12, 12), (2, 12, 12, 18)],
    ),
}


# ---- SegMed: the paper's critical points (GVF_Compute) ----
# Each case is inputs/segmed/<case>.raw (float32, little-endian, x fastest) and <case>.json (its
# dimensions and tlow, on the 0..255 scale the legacy reader rescales the volume to). SegMed keeps
# its field's minima above tlow (the field is the diffused gradient of |grad I|, so its minima are
# the flat regions), then thins each 26-connected run of them to one in 21.
SEGMED_CASES = {
    # Eight small blobs and every source (tlow -1): the blobs' cores and the background's flat
    # stretches between them.
    "many_blobs": dict(n=20, blobs=[((4, 4, 4), 1.5, 1.0), ((10, 4, 6), 1.5, 0.9),
                                    ((15, 5, 4), 1.5, 0.8), ((5, 12, 8), 1.5, 0.95),
                                    ((12, 12, 12), 1.5, 1.0), ((16, 15, 15), 1.5, 0.85),
                                    ((6, 16, 15), 1.5, 0.9), ((14, 8, 16), 1.5, 0.7)],
                       noise=0.02, tlow=-1),
    # A flat-topped plateau and every source: the background around it is one long run of
    # sources, which the thinning samples one in 21.
    "plateau": dict(n=20, blobs=[], plateaus=[((10, 10, 10), 7.0, 6.0, 1.0)], noise=0.02,
                    tlow=-1),
    # A broad blob under heavy noise: sources everywhere, tlow keeping the brighter.
    "noisy_blob": dict(n=18, blobs=[((9, 9, 9), 5.0, 1.0)], noise=0.6, tlow=20),
    # A noisy flat-topped slab: runs of sources on its top and around it.
    "noisy_slab": dict(n=20, blobs=[], plateaus=[((10, 10, 10), 8.0, 4.0, 1.0)], noise=0.3,
                       tlow=20),
}


def write_segmed_inputs():
    out_dir = os.path.join(HERE, "inputs", "segmed")
    os.makedirs(out_dir, exist_ok=True)
    for name, c in SEGMED_CASES.items():
        vals = volume(c["n"], c["blobs"], noise=c["noise"], seed=len(name) + 100,
                      shells=c.get("shells", ()), plateaus=c.get("plateaus", ()))
        with open(os.path.join(out_dir, name + ".raw"), "wb") as f:
            f.write(struct.pack("<%df" % len(vals), *vals))
        meta = {
            "library": "SegMed",
            "dims": [c["n"]] * 3,
            "dtype": "float32-le",
            "tlow": c["tlow"],
            "made_by": "tools/volrover-goldens/make_inputs.py",
        }
        with open(os.path.join(out_dir, name + ".json"), "w", newline="\n") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        print("wrote segmed", name, c["n"], "^3")


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


# ---- PocketTunnel: point clouds sampled from a ball with chambers and necks cut out ----
# Each case is inputs/pockettunnel/<case>.pts (float-exact, as the skeleton's) and <case>.json. A
# solid ball of radius R has spherical chambers inside it, each joined to the outside by
# cylindrical necks narrower than the chamber: a chamber's centre is then a maximum of the distance
# to the surface, and its stable manifold is a pocket with one neck, a tunnel with two, and an
# enclosed cavity with none. The points are on the solid's boundary: the outer sphere outside every
# neck, each chamber's sphere outside its necks, and each neck's wall between its chamber and the
# outer sphere.

GOLDEN = math.pi * (3.0 - math.sqrt(5.0))


def _norm(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / n, v[1] / n, v[2] / n)


def _basis(d):
    """Two unit vectors orthogonal to d and each other."""
    a = (1.0, 0.0, 0.0) if abs(d[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = _norm((d[1] * a[2] - d[2] * a[1], d[2] * a[0] - d[0] * a[2], d[0] * a[1] - d[1] * a[0]))
    w = (d[1] * u[2] - d[2] * u[1], d[2] * u[0] - d[0] * u[2], d[0] * u[1] - d[1] * u[0])
    return u, w


def sphere_points(c, r, h, rnd):
    """A jittered Fibonacci sample of a sphere, spacing about h."""
    n = max(8, int(round(4 * math.pi * r * r / (h * h))))
    pts = []
    for i in range(n):
        z = 1.0 - 2.0 * (i + 0.5) / n
        s = math.sqrt(max(0.0, 1.0 - z * z))
        ph = i * GOLDEN
        p = (s * math.cos(ph), s * math.sin(ph), z)
        j = tuple(0.12 * h / r * (2 * next(rnd) - 1) for _ in range(3))
        p = _norm((p[0] + j[0], p[1] + j[1], p[2] + j[2]))
        pts.append((c[0] + r * p[0], c[1] + r * p[1], c[2] + r * p[2]))
    return pts


def chambered_ball(R, chambers, h_outer, h_inner, seed):
    """chambers: [(centre, radius, [neck directions], neck radius)]."""
    rnd = lcg(seed)
    necks = []  # (chamber index, start, direction, radius)
    for ci, (c, rc, dirs, rn) in enumerate(chambers):
        for d in dirs:
            necks.append((ci, c, _norm(d), rn))

    def in_neck(p, k, margin=1e-9):
        _, c, d, rn = necks[k]
        q = (p[0] - c[0], p[1] - c[1], p[2] - c[2])
        t = q[0] * d[0] + q[1] * d[1] + q[2] * d[2]
        if t <= 0:
            return False
        e = (q[0] - t * d[0], q[1] - t * d[1], q[2] - t * d[2])
        return e[0] * e[0] + e[1] * e[1] + e[2] * e[2] < (rn - margin) ** 2

    def in_chamber(p, ci, margin=1e-9):
        c, rc = chambers[ci][0], chambers[ci][1]
        return (p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2 + (p[2] - c[2]) ** 2 < (rc - margin) ** 2

    def in_ball(p):
        return p[0] * p[0] + p[1] * p[1] + p[2] * p[2] < R * R

    pts = []
    for p in sphere_points((0.0, 0.0, 0.0), R, h_outer, rnd):
        if not any(in_neck(p, k) for k in range(len(necks))):
            pts.append(p)
    for ci, (c, rc, _, _) in enumerate(chambers):
        for p in sphere_points(c, rc, h_inner, rnd):
            if any(in_neck(p, k) for k in range(len(necks)) if necks[k][0] == ci):
                continue
            if any(in_chamber(p, o) for o in range(len(chambers)) if o != ci) or not in_ball(p):
                continue
            pts.append(p)
    for k, (ci, c, d, rn) in enumerate(necks):
        u, w = _basis(d)
        nring = max(6, int(round(2 * math.pi * rn / h_inner)))
        t = 0.0
        while t < R + h_inner:
            for j in range(nring):
                a = 2 * math.pi * (j + 0.3 * (next(rnd) - 0.5)) / nring
                tt = t + 0.3 * h_inner * (next(rnd) - 0.5)
                p = tuple(c[i] + tt * d[i] + rn * (math.cos(a) * u[i] + math.sin(a) * w[i])
                          for i in range(3))
                if in_ball(p) and not in_chamber(p, ci):
                    pts.append(p)
            t += h_inner
    return pts


POCKETTUNNEL_CASES = {
    # one chamber with one neck: a pocket
    "pocket": lambda: chambered_ball(3.0, [((0.0, 0.0, 0.7), 1.4, [(0, 0, 1)], 0.55)],
                                     0.3, 0.2, 21),
    # one chamber with two necks: a tunnel
    "tunnel": lambda: chambered_ball(3.0, [((0.0, 0.0, 0.0), 1.4, [(0, 0, 1), (0, 0, -1)], 0.55)],
                                     0.3, 0.2, 22),
    # one chamber with no neck: a cavity, which Tight Cocone marks inside the surface
    "cavity": lambda: chambered_ball(3.0, [((0.2, -0.1, 0.1), 1.4, [], 0.55)], 0.3, 0.2, 23),
    # a tunnel bent through a chamber, a pocket, and a second pocket off a shared ball
    "three_chambers": lambda: chambered_ball(
        4.0, [((-1.5, 0.3, 0.0), 1.2, [(-0.2, 1, 0.1), (0.1, -0.3, -1)], 0.5),
              ((1.7, -0.4, 0.6), 1.1, [(1, 0.2, 0.4)], 0.45),
              ((0.6, 1.2, -1.8), 0.9, [(0.1, 0.3, -1)], 0.4)], 0.32, 0.2, 24),
}


# inputs/pockettunnel/1grl.pts, a protein's surface (PDB 1GRL's blur map's isosurface), needs libcvc
# to make, so this script leaves it alone: src/cvc/tests/pocket_tunnel_test.cpp's OnAProtein makes
# it again, checks it, and writes it with CVC_POCKETTUNNEL_WRITE_1GRL=<path>.


def write_pockettunnel_inputs():
    out_dir = os.path.join(HERE, "inputs", "pockettunnel")
    os.makedirs(out_dir, exist_ok=True)
    for name, make in POCKETTUNNEL_CASES.items():
        pts = [tuple(f32(v) for v in p) for p in make()]
        with open(os.path.join(out_dir, name + ".pts"), "w", newline="\n") as f:
            for p in pts:
                f.write("%.17g %.17g %.17g\n" % p)
        meta = {
            "library": "PocketTunnel",
            "points": len(pts),
            "parameters": {"num_pockets": 3, "num_tunnels": 3},
            "made_by": "tools/volrover-goldens/make_inputs.py",
        }
        with open(os.path.join(out_dir, name + ".json"), "w", newline="\n") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        print("wrote", name, len(pts), "points")


# SecondaryStructures' cases are surfaces sampled as points, inputs/secondary/<case>.pts (as the
# skeleton's), with the sizes of a protein's density map in Å: a helix's density is a tube about
# 2.5 Å in radius, a sheet's a slab about 2.5 Å thick (the legacy width windows are 2 to 3).
def frame(d):
    """An orthonormal frame (u, v) around the unit direction d."""
    u = (1.0, 0.0, 0.0) if abs(d[0]) < 0.9 else (0.0, 1.0, 0.0)
    dd = sum(u[k] * d[k] for k in range(3))
    u = tuple(u[k] - dd * d[k] for k in range(3))
    ul = math.sqrt(sum(x * x for x in u))
    u = tuple(x / ul for x in u)
    v = (d[1] * u[2] - d[2] * u[1], d[2] * u[0] - d[0] * u[2], d[0] * u[1] - d[1] * u[0])
    return u, v


def tube(origin, direction, length, r, h, rnd):
    """A capsule of radius r from `origin` along `direction`, sampled about every h, jittered."""
    dl = math.sqrt(sum(x * x for x in direction))
    d = tuple(x / dl for x in direction)
    u, v = frame(d)
    pts = []

    def put(t, a, rr):
        pts.append(tuple(origin[k] + t * d[k] + rr * (math.cos(a) * u[k] + math.sin(a) * v[k])
                         for k in range(3)))

    nring = max(8, int(round(2 * math.pi * r / h)))
    nalong = max(2, int(round(length / h)))
    for i in range(nalong + 1):
        jx = 0.3 * (next(rnd) - 0.5) if 0 < i < nalong else 0.0
        for k in range(nring):
            put(length * (i + jx) / nalong, 2 * math.pi * (k + 0.3 * (next(rnd) - 0.5)) / nring, r)
    ncap = max(2, int(round(math.pi * r / 2 / h)))
    for side in (0, 1):
        for i in range(1, ncap + 1):
            t = (math.pi / 2) * (i + 0.1 * (next(rnd) - 0.5)) / (ncap + 0.5)
            nr = max(1, int(round(nring * math.cos(t))))
            for k in range(nr):
                a = 2 * math.pi * (k + 0.3 * (next(rnd) - 0.5)) / nr
                put(-r * math.sin(t) if side == 0 else length + r * math.sin(t), a,
                    r * math.cos(t))
    return pts


def slab(origin, a, b, c, h, rnd):
    """A closed box a x b x c at `origin`, its faces sampled every h, jittered in and off them."""
    na, nb, nc = int(round(a / h)), int(round(b / h)), int(round(c / h))
    pts = []
    for i in range(na + 1):
        for j in range(nb + 1):
            for k in range(nc + 1):
                if 0 < i < na and 0 < j < nb and 0 < k < nc:
                    continue
                p = [a * i / na, b * j / nb, c * k / nc]
                for q, n, s in ((0, na, a / na), (1, nb, b / nb), (2, nc, c / nc)):
                    idx = (i, j, k)[q]
                    on_face = idx == 0 or idx == n
                    p[q] += (0.04 if on_face else 0.4) * s * (next(rnd) - 0.5)
                pts.append(tuple(origin[q] + p[q] for q in range(3)))
    return pts


def helices_and_sheet():
    rnd = lcg(31)
    pts = []
    pts += tube((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 20.0, 2.5, 0.8, rnd)
    pts += tube((0.0, 12.0, 2.0), (0.8, 0.0, 0.6), 18.0, 2.5, 0.8, rnd)
    pts += tube((-4.0, -12.0, 6.0), (0.3, 0.5, 0.81), 16.0, 2.5, 0.8, rnd)
    pts += slab((-4.0, 22.0, -2.0), 20.0, 14.0, 2.5, 0.8, rnd)
    return pts


SECONDARY_CASES = {
    "helices_and_sheet": dict(points=helices_and_sheet,
                              made_from="three capsules of radius 2.5 (16 to 20 long, at three "
                                        "angles) and a 20 x 14 x 2.5 slab, sampled every 0.8 "
                                        "and jittered (make_inputs.py)"),
}


def write_secondary_inputs():
    out_dir = os.path.join(HERE, "inputs", "secondary")
    os.makedirs(out_dir, exist_ok=True)
    for name, c in SECONDARY_CASES.items():
        pts = [tuple(f32(v) for v in p) for p in c["points"]()]
        with open(os.path.join(out_dir, name + ".pts"), "w", newline="\n") as f:
            for p in pts:
                f.write("%.17g %.17g %.17g\n" % p)
        meta = {
            "library": "SecondaryStructures",
            "points": len(pts),
            "made_from": c["made_from"],
            "parameters": {},
            "made_by": "tools/volrover-goldens/make_inputs.py",
        }
        with open(os.path.join(out_dir, name + ".json"), "w", newline="\n") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        print("wrote", name, len(pts), "points")


# ContourTiler's cases are Reconstruct series: inputs/tiler/<case>/<case>.ser and its sections
# <case>.<n>, every trace closed, counterclockwise (the legacy curation requires it) and in an
# identity transform, with coordinates multiples of 2^-8 (so they are exact in decimal, and the
# port's snapping grid holds them unchanged); and inputs/tiler/<case>.json, the parameters and the
# section range the legacy tiler runs on.
def q8(v):
    return round(v * 256) / 256.0


def blob(cx, cy, a, b, wobble, phase, n):
    """A wobbly ellipse, counterclockwise: n vertices at r(t) = 1 + wobble cos(3t + phase)."""
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        r = 1 + wobble * math.cos(3 * t + phase)
        pts.append((q8(cx + a * r * math.cos(t)), q8(cy + b * r * math.sin(t))))
    out = []
    for p in pts:
        if not out or out[-1] != p:
            out.append(p)
    if out[0] == out[-1]:
        out.pop()
    return out


# case: {"sections": {n: [(component, points), ...]}, "first", "last", "parameters"}
def tiler_cases():
    cases = {}
    # A tube that tiles between contour vertices only: no section range past its ends (no caps)
    # and no untiled regions.
    tube = {}
    for k in range(1, 7):
        tube[k] = [("a001", blob(8 + 0.25 * k, 8, 3 + 0.125 * (k % 3), 2.5, 0.08, 0.3 * k, 28))]
    cases["tube"] = {"sections": tube, "first": 1, "last": 6,
                     "parameters": {"curation_delta": 0.01, "collinear_epsilon": 1e-15,
                                    "z_scale": 1}}
    # A branching component (one contour, then two), an annulus (a contour and the hole inside it)
    # and the caps a section range past their ends makes: correspondence, branching, tiling regions
    # and untiled regions.
    br = {k: [] for k in range(0, 8)}
    for k in range(1, 4):
        br[k].append(("d001", blob(6, 6, 3.5 - 0.25 * (k - 1), 2, 0.05, 0.4 * k, 32)))
    for k in range(4, 7):
        br[k].append(("d001", blob(4.25, 6, 1.5, 1.5, 0.06, 0.5 * k, 24)))
        br[k].append(("d001", blob(7.75 + 0.125 * k, 6.25, 1.5, 1.25, 0.06, 0.7 * k, 24)))
    for k in range(2, 6):
        br[k].append(("r001", blob(6, 13, 3, 2.5, 0.04, 0.2 * k, 30)))
        br[k].append(("r001", blob(6.125, 13, 1.25, 1, 0.05, 0.3 * k, 18)))
    cases["branch"] = {"sections": br, "first": 0, "last": 7,
                       "parameters": {"curation_delta": 0.01, "collinear_epsilon": 1e-15,
                                      "z_scale": 1}}
    # Two components whose traces overlap on some sections: curation pushes them apart, and
    # intersection removal separates their tiles.
    tc = {k: [] for k in range(1, 6)}
    for k in range(1, 6):
        tc[k].append(("a002", blob(6, 6, 2.5, 2.25, 0.05, 0.3 * k, 26)))
        tc[k].append(("a003", blob(9.75 - 0.25 * k, 6.5, 2, 2.25, 0.05, 0.6 * k, 26)))
    cases["touching"] = {"sections": tc, "first": 1, "last": 5,
                         "parameters": {"curation_delta": 0.05, "collinear_epsilon": 1e-15,
                                        "remove_intersections": 1, "z_scale": 1}}
    return cases


SER_HEAD = """<?xml version="1.0"?>
<!DOCTYPE Series SYSTEM "series.dtd">

<Series index="%d" viewport="0 0 0.00254" units="microns" defaultThickness="1">
</Series>
"""


def write_tiler_inputs():
    root = os.path.join(HERE, "inputs", "tiler")
    os.makedirs(root, exist_ok=True)
    for name, c in tiler_cases().items():
        d = os.path.join(root, name)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, name + ".ser"), "w", newline="\n") as f:
            f.write(SER_HEAD % c["first"])
        vertices = 0
        for k in range(c["first"], c["last"] + 1):
            lines = ['<?xml version="1.0"?>', '<!DOCTYPE Section SYSTEM "section.dtd">', "",
                     '<Section index="%d" thickness="1" alignLocked="true">' % k,
                     '<Transform dim="0"', ' xcoef=" 0 1 0 0 0 0"', ' ycoef=" 0 0 1 0 0 0">']
            for comp, pts in c["sections"].get(k, []):
                vertices += len(pts)
                body = ",\n\t".join("%.17g %.17g" % p for p in pts)
                lines.append('<Contour name="%s" hidden="false" closed="true" simplified="true" '
                             'border="1 0.5 0" fill="1 0.5 0" mode="9"' % comp)
                lines.append(' points="%s,\n\t"/>' % body)
            lines += ["</Transform>", "</Section>", ""]
            with open(os.path.join(d, "%s.%d" % (name, k)), "w", newline="\n") as f:
                f.write("\n".join(lines))
        meta = {
            "library": "ContourTiler",
            "series": "%s/%s.ser" % (name, name),
            "first": c["first"],
            "last": c["last"],
            "vertices": vertices,
            "parameters": c["parameters"],
            "made_by": "tools/volrover-goldens/make_inputs.py",
        }
        with open(os.path.join(root, name + ".json"), "w", newline="\n") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        print("wrote", name, vertices, "vertices")


# ---- TightCocone: the other libraries' point clouds, and a noisy sphere for Robust Cocone ----
# Each case is inputs/tightcocone/<case>.json: the points it runs on ("input", another library's
# committed .pts, relative to inputs/; or inputs/tightcocone/<case>.pts, which this script writes)
# and the parameters both sides run with (the legacy names).

def noisy_sphere():
    """A sphere of radius 10 sampled about 0.8 apart, each point up to 0.1 off the sphere, and 60
    points scattered through its inside: the noise Robust Cocone is for."""
    rnd = lcg(31)
    pts = []
    for (x, y, z) in sphere_points((0.0, 0.0, 0.0), 10.0, 0.8, rnd):
        s = 1.0 + 0.01 * (2 * next(rnd) - 1)
        pts.append((x * s, y * s, z * s))
    for _ in range(60):
        while True:
            p = tuple(16.0 * (next(rnd) - 0.5) for _ in range(3))
            if p[0] * p[0] + p[1] * p[1] + p[2] * p[2] < 7.5 * 7.5:
                break
        pts.append(p)
    return pts


def outlier_sphere():
    """noisy_sphere's surface with 12 points well outside it (radius 13 to 16): the legacy's box,
    never reset between Robust Cocone's two triangulations, still spans them after Robust Cocone
    leaves them out."""
    rnd = lcg(37)
    pts = []
    for (x, y, z) in sphere_points((0.0, 0.0, 0.0), 10.0, 0.8, rnd):
        s = 1.0 + 0.01 * (2 * next(rnd) - 1)
        pts.append((x * s, y * s, z * s))
    for i in range(12):
        d = _norm(tuple(2 * next(rnd) - 1 for _ in range(3)))
        r = 13.0 + 3.0 * next(rnd)
        pts.append((r * d[0], r * d[1], r * d[2]))
    return pts


TIGHTCOCONE_CASES = {
    "capsule": {"input": "skeleton/capsule.pts"},
    "torus": {"input": "skeleton/torus.pts"},
    "dumbbell": {"input": "skeleton/dumbbell.pts"},
    "box_grid": {"input": "skeleton/box_grid.pts"},
    "pocket": {"input": "pockettunnel/pocket.pts"},
    "1grl": {"input": "pockettunnel/1grl.pts"},
    "helices_and_sheet": {"input": "secondary/helices_and_sheet.pts"},
    "dumbbell_narrow": {"input": "skeleton/dumbbell.pts",
                        "parameters": {"cocone_phi": 0.3, "flat_phi": 0.9,
                                       "flatness_ratio": 2.0}},
    "capsule_robust": {"input": "skeleton/capsule.pts", "parameters": {"robust": 1}},
    "noisy_sphere": {"points": noisy_sphere, "parameters": {"robust": 1}},
    "outlier_sphere": {"points": outlier_sphere, "parameters": {"robust": 1}},
}


def write_tightcocone_inputs():
    out_dir = os.path.join(HERE, "inputs", "tightcocone")
    os.makedirs(out_dir, exist_ok=True)
    for name, c in TIGHTCOCONE_CASES.items():
        if "points" in c:
            pts = [tuple(f32(v) for v in p) for p in c["points"]()]
            with open(os.path.join(out_dir, name + ".pts"), "w", newline="\n") as f:
                for p in pts:
                    f.write("%.17g %.17g %.17g\n" % p)
            source, n = "tightcocone/%s.pts" % name, len(pts)
        else:
            source = c["input"]
            with open(os.path.join(HERE, "inputs", source)) as f:
                n = sum(1 for line in f if line.strip())
        meta = {
            "library": "TightCocone",
            "input": source,
            "points": n,
            "parameters": c.get("parameters", {}),
            "made_by": "tools/volrover-goldens/make_inputs.py",
        }
        with open(os.path.join(out_dir, name + ".json"), "w", newline="\n") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        print("wrote", name, n, "points")


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--only",
                    choices=["genseg", "segmed", "skeleton", "secondary", "tiler",
                             "pockettunnel", "tightcocone"],
                    help="write only one library's inputs (the others stay as committed)")
    ap.add_argument("--case", action="append",
                    help="write only this GenSeg case (repeatable; the others stay as committed)")
    args = ap.parse_args()
    if args.only in (None, "tiler"):
        write_tiler_inputs()
    if args.only in (None, "skeleton"):
        write_skeleton_inputs()
    if args.only in (None, "segmed"):
        write_segmed_inputs()
    if args.only in (None, "pockettunnel"):
        write_pockettunnel_inputs()
    if args.only in (None, "secondary"):
        write_secondary_inputs()
    if args.only in (None, "tightcocone"):
        write_tightcocone_inputs()
    if args.only in ("skeleton", "segmed", "secondary", "tiler", "pockettunnel", "tightcocone"):
        return
    out_dir = os.path.join(HERE, "inputs", "genseg")
    os.makedirs(out_dir, exist_ok=True)
    for name, c in CASES.items():
        if args.case and name not in args.case:
            continue
        vals = volume(c["n"], c["blobs"], noise=c["noise"], seed=len(name),
                      shells=c.get("shells", ()))
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
