#!/usr/bin/env python3
"""Bake a boat model's hull into stations for cvc::physics::loft_hull.

Reads a glTF binary (.glb), fits it as cvcGL's fit_vehicle_geometry does (vehicle_view.h: up the
axis it stands on, else its shortest extent; its length along x, its nose at +x, scaled to
--length; centred along and across; its lowest point at z = 0), slices it at --stations stations
from --from to --to (x in that frame, m), and takes each station's half-section from its keel up to
its sheer (the widest point at or under --sheer-max): --points (half width, height) pairs, closer
together low on the hull, where it meets the water. The heights are then moved so the hull's box
is centred on z = 0: the hull body's origin. Prints a C++ table for the preset and a summary
(length, beam, volume, the draft at --mass in fresh water).

The model's other parts (a cabin, rails, outboards past the transom) are left out by the
stations' range and the sheer. The hull is the model's own frame, so the boat node draws the model
fitted to the same --length (dyn_boat_desc::length_overall) over the hull without moving it.

Standard library only:

    python3 -I tools/boat_hulls/bake_loft.py TacticalBoat.glb --length 7.8 \
        --from -3.05 --to 3.8 --stations 16 --points 8 --mass 2800
"""
import argparse
import hashlib
import json
import math
import struct
import sys


def read_glb(path):
    data = open(path, "rb").read()
    if data[:4] != b"glTF":
        sys.exit("bake_loft: %s is not a glTF binary" % path)
    jlen = struct.unpack_from("<I", data, 12)[0]
    j = json.loads(data[20:20 + jlen])
    binary = data[20 + jlen + 8:]

    def accessor(i):
        a = j["accessors"][i]
        bv = j["bufferViews"][a["bufferView"]]
        off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        if a["type"] == "VEC3":
            stride = bv.get("byteStride", 12)
            return [struct.unpack_from("<3f", binary, off + k * stride) for k in range(a["count"])]
        fmt = {5121: "<B", 5123: "<H", 5125: "<I"}[a["componentType"]]
        size = struct.calcsize(fmt)
        return [struct.unpack_from(fmt, binary, off + k * size)[0] for k in range(a["count"])]

    pts, tris = [], []
    for mesh in j["meshes"]:
        for prim in mesh["primitives"]:
            base = len(pts)
            pos = accessor(prim["attributes"]["POSITION"])
            idx = accessor(prim["indices"]) if "indices" in prim else list(range(len(pos)))
            pts += pos
            tris += [(base + idx[k], base + idx[k + 1], base + idx[k + 2])
                     for k in range(0, len(idx) - 2, 3)]
    return data, pts, tris


def fit(pts, length):
    """fit_vehicle_geometry's frame (vehicle_view.cpp), found from the shape."""
    lo = [min(p[k] for p in pts) for k in range(3)]
    hi = [max(p[k] for p in pts) for k in range(3)]
    ext = [hi[k] - lo[k] for k in range(3)]
    big = max(ext)
    up = -1
    for k in range(3):
        if up >= 0:
            break
        if abs(lo[k]) <= 0.02 * big and all(
                lo[o] < -0.1 * ext[o] and hi[o] > 0.1 * ext[o] for o in range(3) if o != k):
            up = k
    if up < 0:
        up = min(range(3), key=lambda k: (ext[k], k))
    fwd, side = (up + 1) % 3, (up + 2) % 3
    if ext[side] > ext[fwd]:
        fwd, side = side, fwd
    sc = length / ext[fwd] if ext[fwd] > 1e-9 else 1.0
    cf, cs = 0.5 * (lo[fwd] + hi[fwd]), 0.5 * (lo[side] + hi[side])
    n_pos = sum(1 for p in pts if p[fwd] - cf > 0)
    n_neg = len(pts) - n_pos
    top = lo[up] + 0.6 * ext[up]
    tp = [p[fwd] - cf for p in pts if p[up] >= top and p[fwd] - cf > 0]
    tn = [p[fwd] - cf for p in pts if p[up] >= top and p[fwd] - cf <= 0]
    front = 1 if n_pos < n_neg else -1
    if len(tp) > 8 and len(tn) > 8:
        front = 1 if sum(tp) / len(tp) < abs(sum(tn) / len(tn)) else -1
    even = (fwd + 1) % 3 == side
    flip = 1.0 if (1.0 if even else -1.0) * front > 0 else -1.0
    out = [((p[fwd] - cf) * sc * front, (p[side] - cs) * sc * flip, (p[up] - lo[up]) * sc)
           for p in pts]
    return out, "xyz"[up], "xyz"[fwd], front


def section(f, tris, x):
    """The model cut by the plane at x: its segments, as ((y, z), (y, z))."""
    segs = []
    for t in tris:
        cut = []
        for e in range(3):
            a, b = f[t[e]], f[t[(e + 1) % 3]]
            da, db = a[0] - x, b[0] - x
            if (da < 0) == (db < 0):
                continue
            s = da / (da - db)
            cut.append((a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2])))
        if len(cut) >= 2:
            segs.append((cut[0], cut[1]))
    return segs


def width_at(segs, z):
    """The section's half width at height z (its widest crossing), or None where nothing is."""
    w = None
    for (a, b) in segs:
        if (a[1] - z < 0) == (b[1] - z < 0):
            continue
        s = (z - a[1]) / (b[1] - a[1])
        y = abs(a[0] + s * (b[0] - a[0]))
        w = y if w is None else max(w, y)
    return w


def half_section(segs, points, sheer_max, power):
    zs = [q[1] for s in segs for q in s]
    keel = min(zs)
    # The sheer: the widest point at or under sheer_max (above it, a cabin or rails are narrower).
    best_z, best_w, z = keel, 0.0, keel + 0.005
    while z <= min(sheer_max, max(zs)):
        w = width_at(segs, z)
        if w is not None and w >= best_w:
            best_z, best_w = z, w
        z += 0.005
    out, last = [], 0.0
    for k in range(points):
        t = (k / (points - 1)) ** power
        zk = keel + (best_z - keel) * t
        if k == 0:
            w = 0.0
        else:
            w = width_at(segs, zk)
            w = last if w is None else w
            w = max(w, last, 0.01)  # never narrower going up: no inner faces, no zero-width quads
        out.append((w, zk))
        last = w
    return out


def loft(stations):
    """loft_hull's mesh (hull.cpp), for the summary: vertices and triangles."""
    verts, tris = [], []
    k = len(stations[0][1])
    ring = 2 * k - 1
    for x, sec in stations:
        verts.append((x, 0.0, sec[0][1]))
        verts += [(x, w, z) for (w, z) in sec[1:]]
        verts += [(x, -w, z) for (w, z) in reversed(sec[1:])]
    for i in range(len(stations) - 1):
        for r in range(ring):
            a, b = i * ring + r, i * ring + (r + 1) % ring
            c, d = (i + 1) * ring + (r + 1) % ring, (i + 1) * ring + r
            tris += [(a, b, c), (a, c, d)]
    for i, outward in ((0, -1), (len(stations) - 1, 1)):
        ids = list(range(i * ring, (i + 1) * ring))
        cx = sum(verts[v][0] for v in ids) / ring
        cy = sum(verts[v][1] for v in ids) / ring
        cz = sum(verts[v][2] for v in ids) / ring
        c = len(verts)
        verts.append((cx, cy, cz))
        for r in range(ring):
            a, b = ids[r], ids[(r + 1) % ring]
            tris.append((c, a, b) if outward > 0 else (c, b, a))
    return verts, tris


def volume_below(verts, tris, d):
    v = 0.0
    for t in tris:
        p = [verts[t[0]], verts[t[1]], verts[t[2]]]
        q = []
        for k in range(3):
            a, b = p[k], p[(k + 1) % 3]
            if a[2] < d:
                q.append(a)
            if (a[2] < d) != (b[2] < d):
                s = (d - a[2]) / (b[2] - a[2])
                q.append(tuple(a[m] + s * (b[m] - a[m]) for m in range(3)))
        for k in range(1, len(q) - 1):
            u = [q[k][m] - q[0][m] for m in range(3)]
            w = [q[k + 1][m] - q[0][m] for m in range(3)]
            area_z = 0.5 * (u[0] * w[1] - u[1] * w[0])
            depth = d - (q[0][2] + q[k][2] + q[k + 1][2]) / 3.0
            v -= depth * area_z
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("glb")
    ap.add_argument("--length", type=float, required=True, help="overall length to fit (m)")
    ap.add_argument("--from", dest="x0", type=float, required=True, help="aftmost station (m)")
    ap.add_argument("--to", dest="x1", type=float, required=True, help="foremost station (m)")
    ap.add_argument("--stations", type=int, default=16)
    ap.add_argument("--points", type=int, default=8, help="points in each half-section")
    ap.add_argument("--sheer-max", type=float, default=1.3, help="no sheer above this height (m)")
    ap.add_argument("--power", type=float, default=1.25,
                    help="section points' spacing: (k / (n - 1)) ** power of the depth")
    ap.add_argument("--mass", type=float, default=0.0, help="for the summary's draft (kg)")
    ap.add_argument("--name", default="kLoft")
    a = ap.parse_args()
    data, pts, tris = read_glb(a.glb)
    f, up, fwd, nose = fit(pts, a.length)
    raw = []
    for i in range(a.stations):
        x = a.x0 + (a.x1 - a.x0) * i / (a.stations - 1)
        segs = section(f, tris, x)
        if not segs:
            sys.exit("bake_loft: nothing at station x = %.3f" % x)
        raw.append((x, half_section(segs, a.points, a.sheer_max, a.power)))
    zlo = min(s[0][1] for _, s in raw)
    zhi = max(s[-1][1] for _, s in raw)
    zc = 0.5 * (zlo + zhi)
    stations = [(x, [(w, z - zc) for (w, z) in s]) for x, s in raw]
    verts, ltris = loft(stations)
    vol = volume_below(verts, ltris, 1e9)
    sha = hashlib.sha256(data).hexdigest()
    print("// %s: %d stations of %d points, from tools/boat_hulls/bake_loft.py:" %
          (a.name, a.stations, a.points))
    print("//   %s (sha256 %s...), fitted to %.3f m (up %s, length along %s, nose at %s%s)" %
          (a.glb.replace("\\", "/").split("/")[-1], sha[:16], a.length, up, fwd,
           "+" if nose > 0 else "-", fwd))
    print("//   --from %g --to %g --stations %d --points %d --sheer-max %g --power %g" %
          (a.x0, a.x1, a.stations, a.points, a.sheer_max, a.power))
    print("// Each row: x, then (half width, height) from the keel to the sheer; the hull body's")
    print("// frame (m): the model's, its heights moved down by %.4f m (the keel at %.4f)." %
          (zc, zlo - zc))
    print("constexpr double %s[%d][%d] = {" % (a.name, a.stations, 1 + 2 * a.points))
    for x, s in stations:
        nums = ["%.4f" % x] + ["%.4f, %.4f" % (w, z) for (w, z) in s]
        print("    {%s}," % ", ".join(nums))
    print("};")
    beam = 2 * max(s[-1][0] for _, s in stations)
    print("// hull %.3f m long, %.3f m in beam, %.3f m deep, %.3f m^3 under its sheer; %d vertices, "
          "%d triangles" % (a.x1 - a.x0, beam, zhi - zlo, vol, len(verts), len(ltris)),
          file=sys.stderr)
    if a.mass > 0:
        want = a.mass / 1000.0
        lo, hi = zlo - zc, zhi - zc
        for _ in range(100):
            mid = 0.5 * (lo + hi)
            if volume_below(verts, ltris, mid) < want:
                lo = mid
            else:
                hi = mid
        print("// at %.0f kg in fresh water: its waterline at body z %.4f, %.3f m over the keel" %
              (a.mass, mid, mid - (zlo - zc)), file=sys.stderr)


if __name__ == "__main__":
    main()
