#!/usr/bin/env python3
"""Make (or check) the VolumeRover ports' T1 goldens from the legacy libraries.

docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7 T1: a port's output must equal the legacy
library's on committed inputs. This script:

  1. fetches transfix/volrover at the sha in volrover.sha (only the directories the libraries
     need), or uses --volrover DIR;
  2. builds the legacy libraries with this directory's CMakeLists.txt (never part of libcvc's
     build; GPL code such as the legacy tree's own CGAL may be built here and only here);
  3. runs them on the committed inputs (inputs/<library>/<case>.json + .raw);
  4. writes the goldens to src/cvc/tests/data/volrover-goldens/<library>/ (or --out), or, with
     --check, compares what it made with what is committed and fails on any difference.

GenSeg (src/Segmentation/GenSeg) first. Its goldens per case:
  <case>.labels.u8    the class of every voxel (uint8, x fastest; 255: no class), read from the
                      legacy <prefix>_subunit_NN.rawiv files (a voxel is in class NN where that
                      file's value is nonzero: every labelled voxel is above tlow, so above 0)
  <case>.indices.txt  the legacy <prefix>_indices.txt as written (the voxels where fronts met)
  <case>.json         the case, the volrover sha, the platform, each class's voxel count and the
                      files' SHA-256

PocketTunnel (src/PocketTunnel), per case:
  <case>.ptv.txt      what pockettunnel_driver.cpp writes: the clusters outside the surface, the
                      returned geometry (points, colours, triangles), each triangle's cluster,
                      and Tight Cocone's marking (the outside cells, the cocone facets); --check
                      compares it by content (a point by its coordinates, volumes to within
                      1e-9), not byte for byte
  <case>.json         the case, the volrover sha, the platform, the counts and the SHA-256

SegMed (src/Segmentation/SegMed), the paper's critical points, per case:
  <case>.critical.txt the legacy GVF_Compute's critical list, "x y z" a line, in its order
  <case>.field.f32    the diffused field gvfflow() leaves (x, y, z float32 little-endian per
                      voxel, x fastest); --check compares it to within 1e-6, not byte for byte
  <case>.json         the case, the volrover sha, the platform, the count and the SHA-256

SecondaryStructures (src/SecondaryStructures), per case:
  <case>.ss.txt       what secondary_driver.cpp writes: the skeleton's counts and widths, the
                      candidate runs, the helix axes and the sheets' polygons
  <case>.json         the case, the volrover sha, the platform, the counts and the SHA-256

TightCocone (src/TightCocone), per case:
  <case>.tc.txt       what tightcocone_driver.cpp writes: the legacy bounding box, the returned
                      surface's counts and its triangles by their corners' coordinates; --check
                      compares it by content (the triangles as a set), not byte for byte
  <case>.json         the case, the volrover sha, the platform, the counts, the SHA-256 and, for a
                      robust case, whether resetting the box between the two triangulations (the
                      port does; the legacy does not) changed the surface

    python3 tools/volrover-goldens/make_goldens.py            # write the goldens
    python3 tools/volrover-goldens/make_goldens.py --check    # CI: they must not change

Needs git, cmake (with a C++ compiler and, by default, Ninja if it is on PATH) and Python 3.8+.
"""

import argparse
import hashlib
import json
import os
import platform
import shutil
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
VOLROVER_URL = "https://github.com/transfix/volrover.git"
# What each library needs of the legacy tree (a sparse checkout).
SPARSE = {
    "genseg": ["src/Segmentation/GenSeg", "inc/Segmentation/GenSeg", "LICENSE", "COPYING"],
    "segmed": ["src/Segmentation/SegMed", "inc/Segmentation/SegMed", "LICENSE", "COPYING"],
    "skeleton": ["src/Skeletonization", "inc/Skeletonization", "LICENSE", "COPYING"],
    "secondary": ["src/SecondaryStructures", "inc/SecondaryStructures", "LICENSE", "COPYING"],
    "tiler": ["src/ContourTiler", "inc/ContourTiler", "LICENSE", "COPYING"],
    "pockettunnel": ["src/PocketTunnel", "inc/PocketTunnel", "LICENSE", "COPYING"],
    "tightcocone": ["src/TightCocone", "inc/TightCocone", "LICENSE", "COPYING"],
}
# The executable each library's legacy build makes (CMakeLists.txt).
TARGETS = {"genseg": "genseg_legacy", "segmed": "segmed_legacy", "skeleton": "skeleton_legacy",
           "secondary": "secondary_legacy", "tiler": "tiler_legacy",
           "pockettunnel": "pockettunnel_legacy", "tightcocone": "tightcocone_legacy"}
# The largest difference --check allows between a made and a committed SegMed field: the legacy
# code's exp() is the platform's expf, which may round a last bit differently.
FIELD_TOLERANCE = 1e-6


def pinned_sha():
    with open(os.path.join(HERE, "volrover.sha")) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                return line
    raise SystemExit("volrover.sha holds no sha")


def run(cmd, cwd=None, quiet=False):
    print("+", " ".join(cmd), flush=True)
    r = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE if quiet else None,
                       stderr=subprocess.STDOUT if quiet else None, text=True)
    if r.returncode != 0:
        if quiet and r.stdout:
            sys.stdout.write(r.stdout)
        raise SystemExit("failed (%d): %s" % (r.returncode, " ".join(cmd)))
    return r


def fetch(sha, work, libs):
    """A sparse, shallow checkout of volrover at `sha` in work/volrover-<sha>."""
    dst = os.path.join(work, "volrover-" + sha[:12])
    stamp = os.path.join(dst, ".volrover-goldens-sha")
    want = sorted({p for lib in libs for p in SPARSE[lib]})
    if os.path.exists(stamp):
        with open(stamp) as f:
            if f.read().split() == [sha] + want:
                return dst
        shutil.rmtree(dst)
    os.makedirs(dst, exist_ok=True)
    run(["git", "init", "-q"], cwd=dst)
    run(["git", "remote", "add", "origin", VOLROVER_URL], cwd=dst)
    run(["git", "sparse-checkout", "init", "--no-cone"], cwd=dst)
    run(["git", "sparse-checkout", "set", "--no-cone"] + ["/" + p for p in want], cwd=dst)
    run(["git", "fetch", "-q", "--depth", "1", "origin", sha], cwd=dst)
    run(["git", "checkout", "-q", "FETCH_HEAD"], cwd=dst)
    head = run(["git", "rev-parse", "HEAD"], cwd=dst, quiet=True).stdout.strip()
    if head != sha:
        raise SystemExit("fetched %s, not the pinned %s" % (head, sha))
    with open(stamp, "w") as f:
        f.write(" ".join([sha] + want))
    return dst


def build(volrover, work, libs):
    """Build each library's legacy executable; their paths by library. Skeletonization needs
    CGAL, GMP, MPFR and Boost where CMake finds them (CMAKE_PREFIX_PATH)."""
    bdir = os.path.join(work, "build")
    cmd = ["cmake", "-S", HERE, "-B", bdir, "-DCMAKE_BUILD_TYPE=Release",
           "-DVOLROVER_DIR=" + volrover]
    if shutil.which("ninja") and not os.environ.get("CMAKE_GENERATOR"):
        cmd += ["-G", "Ninja"]
    run(cmd, quiet=True)
    exes = {}
    for lib in libs:
        target = TARGETS[lib]
        run(["cmake", "--build", bdir, "--config", "Release", "--target", target], quiet=True)
        for cand in (os.path.join(bdir, target), os.path.join(bdir, target + ".exe"),
                     os.path.join(bdir, "Release", target + ".exe")):
            if os.path.exists(cand):
                exes[lib] = cand
                break
        else:
            raise SystemExit(target + " was not built in " + bdir)
    return exes


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def write_rawiv(path, dims, values):
    """A big-endian float rawiv file over the unit grid (origin 0, span 1)."""
    nx, ny, nz = dims
    n = nx * ny * nz
    with open(path, "wb") as f:
        f.write(struct.pack(">3f", 0.0, 0.0, 0.0))
        f.write(struct.pack(">3f", nx - 1.0, ny - 1.0, nz - 1.0))
        f.write(struct.pack(">2i", n, max(nx - 1, 1) * max(ny - 1, 1) * max(nz - 1, 1)))
        f.write(struct.pack(">3I", nx, ny, nz))
        f.write(struct.pack(">3f", 0.0, 0.0, 0.0))
        f.write(struct.pack(">3f", 1.0, 1.0, 1.0))
        f.write(struct.pack(">%df" % n, *values))


def read_rawiv_floats(path, n):
    with open(path, "rb") as f:
        data = f.read()
    header = 68
    if len(data) != header + 4 * n:
        raise SystemExit("%s: %d bytes, not a %d-voxel float rawiv" % (path, len(data), n))
    return struct.unpack(">%df" % n, data[header:])


def genseg_case(exe, inputs_dir, case, out_dir):
    with open(os.path.join(inputs_dir, case + ".json")) as f:
        meta = json.load(f)
    dims = meta["dims"]
    n = dims[0] * dims[1] * dims[2]
    with open(os.path.join(inputs_dir, case + ".raw"), "rb") as f:
        values = struct.unpack("<%df" % n, f.read())
    classes = 1 + max(s[0] for s in meta["seeds"])
    with tempfile.TemporaryDirectory(prefix="genseg-") as tmp:
        write_rawiv(os.path.join(tmp, "input.rawiv"), dims, values)
        with open(os.path.join(tmp, "seeds.txt"), "w", newline="\n") as f:
            for c, x, y, z in meta["seeds"]:
                f.write("%d %d %d %d\n" % (c, x, y, z))
        run([exe, "input.rawiv", str(meta["tlow"]), str(meta["thigh"]), "out", "seeds.txt"],
            cwd=tmp, quiet=True)
        labels = bytearray([255]) * n
        counts = []
        for c in range(classes):
            sub = read_rawiv_floats(os.path.join(tmp, "out_subunit_%02d.rawiv" % c), n)
            count = 0
            for i, v in enumerate(sub):
                if v != 0.0:
                    if labels[i] != 255:
                        raise SystemExit("%s: voxel %d is in classes %d and %d" %
                                         (case, i, labels[i], c))
                    labels[i] = c
                    count += 1
            counts.append(count)
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, case + ".labels.u8"), "wb") as f:
            f.write(bytes(labels))
        with open(os.path.join(tmp, "out_indices.txt"), "rb") as src, \
                open(os.path.join(out_dir, case + ".indices.txt"), "wb") as dst:
            dst.write(src.read().replace(b"\r\n", b"\n"))
    return meta, counts


def segmed_case(exe, inputs_dir, case, out_dir):
    """Run the legacy SegMed on inputs/segmed/<case>.raw; write <case>.critical.txt and
    <case>.field.f32 (segmed_driver.cpp) and return the case's metadata and its count."""
    with open(os.path.join(inputs_dir, case + ".json")) as f:
        meta = json.load(f)
    dims = meta["dims"]
    n = dims[0] * dims[1] * dims[2]
    with open(os.path.join(inputs_dir, case + ".raw"), "rb") as f:
        values = struct.unpack("<%df" % n, f.read())
    with tempfile.TemporaryDirectory(prefix="segmed-") as tmp:
        write_rawiv(os.path.join(tmp, "input.rawiv"), dims, values)
        run([exe, "input.rawiv", str(meta["tlow"]), "critical.txt", "field.f32"], cwd=tmp,
            quiet=True)
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(tmp, "critical.txt"), "rb") as src:
            text = src.read().replace(b"\r\n", b"\n")
        with open(os.path.join(out_dir, case + ".critical.txt"), "wb") as dst:
            dst.write(text)
        with open(os.path.join(tmp, "field.f32"), "rb") as src:
            field = src.read()
        if len(field) != 12 * n:
            raise SystemExit("%s: the field has %d bytes, not %d" % (case, len(field), 12 * n))
        with open(os.path.join(out_dir, case + ".field.f32"), "wb") as dst:
            dst.write(field)
    return meta, len(text.splitlines())


def fields_differ(made, kept):
    """The largest difference between two .field.f32 files (inf if their sizes differ)."""
    with open(made, "rb") as f:
        a = f.read()
    with open(kept, "rb") as f:
        b = f.read()
    if len(a) != len(b):
        return float("inf")
    va = struct.unpack("<%df" % (len(a) // 4), a)
    vb = struct.unpack("<%df" % (len(b) // 4), b)
    return max((abs(x - y) for x, y in zip(va, vb)), default=0.0)


def skeleton_case(exe, inputs_dir, case, out_dir):
    """Run the legacy Skeletonization on inputs/skeleton/<case>.pts; write <case>.skel.txt (what
    skeleton_driver.cpp writes) and return the case's metadata and its counts. It runs in a
    scratch directory: the legacy code writes debug files to the current one."""
    with open(os.path.join(inputs_dir, case + ".json")) as f:
        meta = json.load(f)
    args = ["%s=%s" % (k, v) for k, v in sorted(meta.get("parameters", {}).items())]
    with tempfile.TemporaryDirectory(prefix="skeleton-") as tmp:
        shutil.copy(os.path.join(inputs_dir, case + ".pts"), os.path.join(tmp, "in.pts"))
        run([exe, "in.pts", "out.txt"] + args, cwd=tmp, quiet=True)
        with open(os.path.join(tmp, "out.txt"), "rb") as f:
            text = f.read().replace(b"\r\n", b"\n")
        debug = sorted(n for n in os.listdir(tmp) if n not in ("in.pts", "out.txt"))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, case + ".skel.txt"), "wb") as f:
        f.write(text)
    counts = {}
    for line in text.decode().splitlines():
        counts[line[0]] = counts.get(line[0], 0) + 1
    return meta, {"stars": counts.get("S", 0), "polyline_segments": counts.get("L", 0),
                  "helix_strips": counts.get("H", 0), "polygons": counts.get("P", 0),
                  "debug_files": debug}


def secondary_case(exe, inputs_dir, case, out_dir):
    """Run the legacy SecondaryStructures on inputs/secondary/<case>.pts; write <case>.ss.txt
    (what secondary_driver.cpp writes) and return the case's metadata and its counts. It runs in a
    scratch directory: the legacy code writes debug files to the current one."""
    with open(os.path.join(inputs_dir, case + ".json")) as f:
        meta = json.load(f)
    args = ["%s=%s" % (k, v) for k, v in sorted(meta.get("parameters", {}).items())]
    with tempfile.TemporaryDirectory(prefix="secondary-") as tmp:
        shutil.copy(os.path.join(inputs_dir, case + ".pts"), os.path.join(tmp, "in.pts"))
        run([exe, "in.pts", "out.txt"] + args, cwd=tmp, quiet=True)
        with open(os.path.join(tmp, "out.txt"), "rb") as f:
            text = f.read().replace(b"\r\n", b"\n")
        debug = sorted(n for n in os.listdir(tmp) if n not in ("in.pts", "out.txt"))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, case + ".ss.txt"), "wb") as f:
        f.write(text)
    kinds = {}
    head = []
    for line in text.decode().splitlines():
        kinds[line[0]] = kinds.get(line[0], 0) + 1
        if line[0] == "N":
            head = [int(x) for x in line.split()[1:]]
    nv, ne, nf, comps, helices, drawn, sheets, sheet_candidates = head
    return meta, {"skeleton_points": nv, "skeleton_edges": ne, "polygons": nf,
                  "components": comps, "runs": kinds.get("R", 0), "helices": helices,
                  "helices_drawn": drawn, "sheets_drawn": sheets,
                  "sheet_candidates": sheet_candidates, "sheet_polygons": kinds.get("P", 0),
                  "debug_files": debug}


# How far --check lets a remade tiler golden's coordinates be from the committed one's: the
# legacy ContourTiler runs on CGAL::Cartesian<double> (inexact predicates and constructions), and
# its untiled-region fill interpolates, so the platform's floating point can move the last bits.
TILES_TOLERANCE = 1e-9


def read_rawc(path):
    """A legacy .rawc: its vertices (x, y, z) and triangles (vertex indices)."""
    with open(path) as f:
        tokens = f.read().split()
    nv, nt = int(tokens[0]), int(tokens[1])
    at = 2
    per = (len(tokens) - 2 - 3 * nt) // nv if nv else 3
    verts = []
    for _ in range(nv):
        verts.append(tuple(float(t) for t in tokens[at:at + 3]))
        at += per
    tris = []
    for _ in range(nt):
        tris.append(tuple(int(t) for t in tokens[at:at + 3]))
        at += 3
    return verts, tris


def write_tiles(path, verts, tris):
    with open(path, "w", newline="\n") as f:
        f.write("%d %d\n" % (len(verts), len(tris)))
        for v in verts:
            f.write("%.17g %.17g %.17g\n" % v)
        for t in tris:
            f.write("%d %d %d\n" % t)


def tiles_differ(made, kept):
    """The largest coordinate difference between two tiles files, or infinity when their vertex
    counts or triangles differ."""
    a = open(made).read().split()
    b = open(kept).read().split()
    if len(a) != len(b) or a[:2] != b[:2]:
        return float("inf")
    nv = int(a[0])
    worst = 0.0
    for i in range(2, 2 + 3 * nv):
        worst = max(worst, abs(float(a[i]) - float(b[i])))
    if a[2 + 3 * nv:] != b[2 + 3 * nv:]:
        return float("inf")
    return worst


def tiler_case(exe, inputs_dir, case, out_dir):
    """Run the legacy ContourTiler on inputs/tiler/<case>/<case>.ser, sections first to last;
    write <case>.<component>.tiles.txt for each component it tiled (its vertices with the tiler's
    own z, the section index on a contour, and its triangles, in the legacy order) and return the
    case's metadata and each component's counts. It runs in a scratch directory: the legacy code
    writes its output files there."""
    with open(os.path.join(inputs_dir, case + ".json")) as f:
        meta = json.load(f)
    args = ["%s=%r" % (k, v) for k, v in sorted(meta.get("parameters", {}).items())]
    base = os.path.join(inputs_dir, case, case)
    counts = {}
    os.makedirs(out_dir, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="tiler-") as tmp:
        run([exe, base, str(meta["first"]), str(meta["last"]), tmp] + args, cwd=tmp, quiet=True)
        for name in sorted(os.listdir(tmp)):
            if not name.endswith("_tiles.rawc") or name == "_tiles.rawc":
                continue
            component = name[:-len("_tiles.rawc")]
            verts, tris = read_rawc(os.path.join(tmp, name))
            write_tiles(os.path.join(out_dir, "%s.%s.tiles.txt" % (case, component)), verts, tris)
            counts[component] = {"vertices": len(verts), "triangles": len(tris)}
    return meta, counts


def pockettunnel_case(exe, inputs_dir, case, out_dir):
    """Run the legacy PocketTunnel on inputs/pockettunnel/<case>.pts; write <case>.ptv.txt (what
    pockettunnel_driver.cpp writes) and return the case's metadata and its counts."""
    with open(os.path.join(inputs_dir, case + ".json")) as f:
        meta = json.load(f)
    args = ["%s=%s" % (k, v) for k, v in sorted(meta.get("parameters", {}).items())]
    with tempfile.TemporaryDirectory(prefix="pockettunnel-") as tmp:
        shutil.copy(os.path.join(inputs_dir, case + ".pts"), os.path.join(tmp, "in.pts"))
        run([exe, "in.pts", "out.txt"] + args, cwd=tmp, quiet=True)
        with open(os.path.join(tmp, "out.txt"), "rb") as f:
            text = f.read().replace(b"\r\n", b"\n")
        debug = sorted(n for n in os.listdir(tmp) if n not in ("in.pts", "out.txt"))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, case + ".ptv.txt"), "wb") as f:
        f.write(text)
    ptv = read_ptv(os.path.join(out_dir, case + ".ptv.txt"))
    kinds = [k for k, _, _, _, _ in ptv["features"]]
    return meta, {"pockets": kinds.count("P"), "tunnels": kinds.count("T"),
                  "voids": kinds.count("V"), "points": len(ptv["points"]),
                  "triangles": len(ptv["triangles"]), "debug_files": debug}


# How far --check lets a remade PocketTunnel golden's cluster volumes be from the committed ones',
# relative: they are sums of many cells' volumes in doubles.
PTV_TOLERANCE = 1e-9


def read_ptv(path):
    """A pockettunnel_driver.cpp output: features (kind, mouths, volume, cells, maxima), points
    ((x, y, z), (r, g, b)), triangles ((a, b, c), feature, surface), outside cells and cocone
    facets (vertex indices)."""
    features, points, triangles, outside, cocone = [], [], [], [], []
    with open(path) as f:
        for line in f:
            w = line.split()
            if not w:
                continue
            if w[0] == "F":
                features.append((w[1], int(w[2]), float(w[3]), int(w[4]), int(w[5])))
            elif w[0] == "V":
                points.append((tuple(float(v) for v in w[1:4]), tuple(float(v) for v in w[4:7])))
            elif w[0] == "T":
                triangles.append(((int(w[1]), int(w[2]), int(w[3])), int(w[4]), int(w[5])))
            elif w[0] == "O":
                outside.append(tuple(int(v) for v in w[1:5]))
            elif w[0] == "K":
                cocone.append(tuple(int(v) for v in w[1:4]))
    return {"features": features, "points": points, "triangles": triangles, "outside": outside,
            "cocone": cocone}


def ptv_problems(made, kept):
    """What differs between two PocketTunnel goldens, by content: the clusters in order (volumes
    to within PTV_TOLERANCE), each point's colour by its coordinates, and the triangles by their
    corners' coordinates (each rotated to start at its least corner) with their cluster and
    class."""
    a, b = read_ptv(made), read_ptv(kept)
    out = []
    if len(a["features"]) != len(b["features"]):
        return ["%d clusters, not %d" % (len(a["features"]), len(b["features"]))]
    for k, (x, y) in enumerate(zip(a["features"], b["features"])):
        if x[0] != y[0] or x[1] != y[1] or x[3] != y[3] or x[4] != y[4] or \
                abs(x[2] - y[2]) > PTV_TOLERANCE * max(1.0, abs(y[2])):
            out.append("cluster %d: %s, not %s" % (k, x, y))
    if dict(a["points"]) != dict(b["points"]):
        out.append("the points or their colours differ")

    def tris(g):
        pts = [p for p, _ in g["points"]]
        res = []
        for (i, j, k), feat, surf in g["triangles"]:
            c = [pts[i], pts[j], pts[k]]
            m = c.index(min(c))
            res.append((tuple(c[m:] + c[:m]), feat, surf))
        return sorted(res)
    if tris(a) != tris(b):
        out.append("the triangles differ")

    def marking(g, key, n):
        pts = [p for p, _ in g["points"]]
        return sorted(tuple(sorted(pts[i] for i in c)) for c in g[key])
    if marking(a, "outside", 4) != marking(b, "outside", 4) or \
            marking(a, "cocone", 3) != marking(b, "cocone", 3):
        out.append("Tight Cocone's marking differs")
    return out


def tightcocone_run(exe, pts, params, tmp):
    """The legacy TightCocone's output on the points file `pts`, as bytes (LF line ends)."""
    shutil.copy(pts, os.path.join(tmp, "in.pts"))
    args = ["%s=%s" % (k, v) for k, v in sorted(params.items())]
    run([exe, "in.pts", "out.txt"] + args, cwd=tmp, quiet=True)
    with open(os.path.join(tmp, "out.txt"), "rb") as f:
        return f.read().replace(b"\r\n", b"\n")


def read_tc(text):
    """A tightcocone_driver.cpp output: its header values, and the triangles as a set of corner
    triples, each rotated to start at its smallest corner (so the set keeps the orientation)."""
    head, verts, tris = {}, [], set()
    for line in text.decode().splitlines():
        f = line.split()
        if not f:
            continue
        if f[0] == "v":
            verts.append(tuple(float(v) for v in f[1:4]))
        elif f[0] == "t":
            c = [verts[int(v)] for v in f[1:4]]
            i = c.index(min(c))
            tris.add(tuple(c[i:] + c[:i]))
        else:
            head[f[0]] = f[1:]
    return head, tris


def tightcocone_case(exe, inputs_dir, case, out_dir):
    """Run the legacy TightCocone on the case's points; write <case>.tc.txt (what
    tightcocone_driver.cpp writes) and return the case's metadata, its counts and, for a robust
    case, what a second run with reset_box=1 showed: whether resetting the legacy box between the
    two triangulations changes the surface."""
    with open(os.path.join(inputs_dir, case + ".json")) as f:
        meta = json.load(f)
    pts = os.path.join(HERE, "inputs", meta["input"])
    params = meta.get("parameters", {})
    with tempfile.TemporaryDirectory(prefix="tightcocone-") as tmp:
        text = tightcocone_run(exe, pts, params, tmp)
        debug = sorted(n for n in os.listdir(tmp) if n not in ("in.pts", "out.txt"))
        reset = None
        if params.get("robust"):
            again = tightcocone_run(exe, pts, dict(params, reset_box=1), tmp)
            h0, t0 = read_tc(text)
            h1, t1 = read_tc(again)
            reset = {"surface_changes": t0 != t1, "box": [float(v) for v in h0["box"]],
                     "box_reset": [float(v) for v in h1["box"]]}
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, case + ".tc.txt"), "wb") as f:
        f.write(text)
    head, tris = read_tc(text)
    counts = {"vertices": int(head["vertices"][0]), "triangles": int(head["triangles"][0]),
              "distinct_triangles": len(tris), "debug_files": debug}
    return meta, counts, reset


def tc_problems(made, kept):
    """What differs between two TightCocone goldens, by content."""
    with open(made, "rb") as f:
        hm, tm = read_tc(f.read())
    with open(kept, "rb") as f:
        hk, tk = read_tc(f.read())
    out = []
    for k in ("points", "robust", "vertices", "triangles"):
        if hm.get(k) != hk.get(k):
            out.append("%s %s, committed %s" % (k, hm.get(k), hk.get(k)))
    if tm != tk:
        out.append("%d triangles not committed, %d committed not made" % (len(tm - tk),
                                                                          len(tk - tm)))
    return out


def describe_platform():
    return "%s %s, %s, Python %s" % (platform.system(), platform.machine(),
                                    platform.platform(terse=True), platform.python_version())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--work", default=os.path.join(tempfile.gettempdir(), "volrover-goldens"),
                    help="where the volrover checkout and the build go")
    ap.add_argument("--volrover", help="an existing volrover checkout (must be at the pinned sha)")
    ap.add_argument("--out", default=os.path.join(REPO, "src", "cvc", "tests", "data",
                                                   "volrover-goldens"))
    ap.add_argument("--check", action="store_true",
                    help="make the goldens in a scratch directory and compare them with --out")
    ap.add_argument("--libs",
                    default="genseg,segmed,skeleton,secondary,tiler,pockettunnel,tightcocone",
                    help="the libraries, comma-separated (genseg, segmed, skeleton, secondary, "
                         "tiler, pockettunnel, tightcocone; all but genseg and segmed need CGAL)")
    args = ap.parse_args()

    sha = pinned_sha()
    libs = [lib for lib in args.libs.split(",") if lib]
    for lib in libs:
        if lib not in SPARSE:
            raise SystemExit("no such library: " + lib)
    work = os.path.abspath(args.work)
    os.makedirs(work, exist_ok=True)
    volrover = os.path.abspath(args.volrover) if args.volrover else fetch(sha, work, libs)
    exes = build(volrover, work, libs)

    target = os.path.join(work, "check") if args.check else args.out
    if args.check and os.path.exists(target):
        shutil.rmtree(target)
    compared = []  # (library dir, file name) pairs --check compares byte for byte
    near = []  # (library dir, file name) pairs --check compares to within FIELD_TOLERANCE
    near_tiles = []  # (library dir, file name) pairs --check compares to within TILES_TOLERANCE

    if "genseg" in libs:
        exe = exes["genseg"]
        inputs_dir = os.path.join(HERE, "inputs", "genseg")
        out_dir = os.path.join(target, "genseg")
        cases = sorted(f[:-5] for f in os.listdir(inputs_dir) if f.endswith(".json"))
        for case in cases:
            meta, counts = genseg_case(exe, inputs_dir, case, out_dir)
            golden = {
                "library": "GenSeg",
                "volrover": sha,
                "case": case,
                "input": "tools/volrover-goldens/inputs/genseg/%s.raw" % case,
                "input_sha256": sha256(os.path.join(inputs_dir, case + ".raw")),
                "dims": meta["dims"],
                "tlow": meta["tlow"],
                "thigh": meta["thigh"],
                "seeds": meta["seeds"],
                "class_voxels": counts,
                "labels_sha256": sha256(os.path.join(out_dir, case + ".labels.u8")),
                "indices_sha256": sha256(os.path.join(out_dir, case + ".indices.txt")),
                "made_by": "tools/volrover-goldens/make_goldens.py",
                "platform": describe_platform(),
            }
            with open(os.path.join(out_dir, case + ".json"), "w", newline="\n") as f:
                json.dump(golden, f, indent=2)
                f.write("\n")
            print("GenSeg %s: classes %s, %d voxels unlabelled" %
                  (case, counts, golden["dims"][0] * golden["dims"][1] * golden["dims"][2] -
                   sum(counts)))
            compared.append(("genseg", case + ".labels.u8"))
            compared.append(("genseg", case + ".indices.txt"))

    if "segmed" in libs:
        inputs_dir = os.path.join(HERE, "inputs", "segmed")
        out_dir = os.path.join(target, "segmed")
        for case in sorted(f[:-5] for f in os.listdir(inputs_dir) if f.endswith(".json")):
            meta, count = segmed_case(exes["segmed"], inputs_dir, case, out_dir)
            golden = {
                "library": "SegMed",
                "volrover": sha,
                "case": case,
                "input": "tools/volrover-goldens/inputs/segmed/%s.raw" % case,
                "input_sha256": sha256(os.path.join(inputs_dir, case + ".raw")),
                "dims": meta["dims"],
                "tlow": meta["tlow"],
                "critical_points": count,
                "critical_sha256": sha256(os.path.join(out_dir, case + ".critical.txt")),
                "field_sha256": sha256(os.path.join(out_dir, case + ".field.f32")),
                "made_by": "tools/volrover-goldens/make_goldens.py",
                "platform": describe_platform(),
            }
            with open(os.path.join(out_dir, case + ".json"), "w", newline="\n") as f:
                json.dump(golden, f, indent=2)
                f.write("\n")
            compared.append(("segmed", case + ".critical.txt"))
            near.append(("segmed", case + ".field.f32"))
            print("SegMed %s: %d critical points" % (case, count))

    if "skeleton" in libs:
        inputs_dir = os.path.join(HERE, "inputs", "skeleton")
        out_dir = os.path.join(target, "skeleton")
        for case in sorted(f[:-5] for f in os.listdir(inputs_dir) if f.endswith(".json")):
            meta, counts = skeleton_case(exes["skeleton"], inputs_dir, case, out_dir)
            golden = {
                "library": "Skeletonization",
                "volrover": sha,
                "case": case,
                "input": "tools/volrover-goldens/inputs/skeleton/%s.pts" % case,
                "input_sha256": sha256(os.path.join(inputs_dir, case + ".pts")),
                "points": meta["points"],
                "general_position": meta["general_position"],
                "parameters": meta.get("parameters", {}),
                "counts": counts,
                "skel_sha256": sha256(os.path.join(out_dir, case + ".skel.txt")),
                "made_by": "tools/volrover-goldens/make_goldens.py",
                "platform": describe_platform(),
            }
            with open(os.path.join(out_dir, case + ".json"), "w", newline="\n") as f:
                json.dump(golden, f, indent=2)
                f.write("\n")
            compared.append(("skeleton", case + ".skel.txt"))
            print("Skeletonization %s: %d polyline segments, %d stars, %d polygons, %d helix "
                  "strips" % (case, counts["polyline_segments"], counts["stars"],
                              counts["polygons"], counts["helix_strips"]))

    if "secondary" in libs:
        inputs_dir = os.path.join(HERE, "inputs", "secondary")
        out_dir = os.path.join(target, "secondary")
        for case in sorted(f[:-5] for f in os.listdir(inputs_dir) if f.endswith(".json")):
            meta, counts = secondary_case(exes["secondary"], inputs_dir, case, out_dir)
            golden = {
                "library": "SecondaryStructures",
                "volrover": sha,
                "case": case,
                "input": "tools/volrover-goldens/inputs/secondary/%s.pts" % case,
                "input_sha256": sha256(os.path.join(inputs_dir, case + ".pts")),
                "points": meta["points"],
                "made_from": meta["made_from"],
                "parameters": meta.get("parameters", {}),
                "counts": counts,
                "ss_sha256": sha256(os.path.join(out_dir, case + ".ss.txt")),
                "made_by": "tools/volrover-goldens/make_goldens.py",
                "platform": describe_platform(),
            }
            with open(os.path.join(out_dir, case + ".json"), "w", newline="\n") as f:
                json.dump(golden, f, indent=2)
                f.write("\n")
            compared.append(("secondary", case + ".ss.txt"))
            print("SecondaryStructures %s: %d helices (%d drawn), %d of %d sheet candidates drawn "
                  "(%d polygons), %d polygons in %d components" %
                  (case, counts["helices"], counts["helices_drawn"], counts["sheets_drawn"],
                   counts["sheet_candidates"], counts["sheet_polygons"], counts["polygons"],
                   counts["components"]))

    if "tiler" in libs:
        inputs_dir = os.path.join(HERE, "inputs", "tiler")
        out_dir = os.path.join(target, "tiler")
        for case in sorted(f[:-5] for f in os.listdir(inputs_dir) if f.endswith(".json")):
            meta, counts = tiler_case(exes["tiler"], inputs_dir, case, out_dir)
            series = os.path.join(inputs_dir, case)
            golden = {
                "library": "ContourTiler",
                "volrover": sha,
                "case": case,
                "input": "tools/volrover-goldens/inputs/tiler/%s/%s.ser" % (case, case),
                "input_sha256": {n: sha256(os.path.join(series, n))
                                 for n in sorted(os.listdir(series))},
                "first": meta["first"],
                "last": meta["last"],
                "parameters": meta.get("parameters", {}),
                "components": counts,
                "tiles_sha256": {c: sha256(os.path.join(out_dir, "%s.%s.tiles.txt" % (case, c)))
                                 for c in counts},
                "made_by": "tools/volrover-goldens/make_goldens.py",
                "platform": describe_platform(),
            }
            with open(os.path.join(out_dir, case + ".json"), "w", newline="\n") as f:
                json.dump(golden, f, indent=2)
                f.write("\n")
            for c in counts:
                near_tiles.append(("tiler", "%s.%s.tiles.txt" % (case, c)))
            print("ContourTiler %s: %s" % (case, ", ".join(
                "%s %d triangles" % (c, n["triangles"]) for c, n in sorted(counts.items()))))

    near_ptv = []  # (library dir, file name) pairs --check compares by content (ptv_problems)
    if "pockettunnel" in libs:
        inputs_dir = os.path.join(HERE, "inputs", "pockettunnel")
        out_dir = os.path.join(target, "pockettunnel")
        for case in sorted(f[:-5] for f in os.listdir(inputs_dir) if f.endswith(".json")):
            meta, counts = pockettunnel_case(exes["pockettunnel"], inputs_dir, case, out_dir)
            golden = {
                "library": "PocketTunnel",
                "volrover": sha,
                "case": case,
                "input": "tools/volrover-goldens/inputs/pockettunnel/%s.pts" % case,
                "input_sha256": sha256(os.path.join(inputs_dir, case + ".pts")),
                "points": meta["points"],
                "parameters": meta.get("parameters", {}),
                "counts": counts,
                "ptv_sha256": sha256(os.path.join(out_dir, case + ".ptv.txt")),
                "made_by": "tools/volrover-goldens/make_goldens.py",
                "platform": describe_platform(),
            }
            with open(os.path.join(out_dir, case + ".json"), "w", newline="\n") as f:
                json.dump(golden, f, indent=2)
                f.write("\n")
            near_ptv.append(("pockettunnel", case + ".ptv.txt"))
            print("PocketTunnel %s: %d pockets, %d tunnels, %d voids; %d triangles shown" %
                  (case, counts["pockets"], counts["tunnels"], counts["voids"],
                   counts["triangles"]))

    near_tc = []  # (library dir, file name) pairs --check compares by content (tc_problems)
    if "tightcocone" in libs:
        inputs_dir = os.path.join(HERE, "inputs", "tightcocone")
        out_dir = os.path.join(target, "tightcocone")
        for case in sorted(f[:-5] for f in os.listdir(inputs_dir) if f.endswith(".json")):
            meta, counts, reset = tightcocone_case(exes["tightcocone"], inputs_dir, case, out_dir)
            golden = {
                "library": "TightCocone",
                "volrover": sha,
                "case": case,
                "input": "tools/volrover-goldens/inputs/" + meta["input"],
                "input_sha256": sha256(os.path.join(HERE, "inputs", meta["input"])),
                "points": meta["points"],
                "parameters": meta.get("parameters", {}),
                "counts": counts,
                "tc_sha256": sha256(os.path.join(out_dir, case + ".tc.txt")),
                "made_by": "tools/volrover-goldens/make_goldens.py",
                "platform": describe_platform(),
            }
            if reset is not None:
                golden["reset_box"] = reset
            with open(os.path.join(out_dir, case + ".json"), "w", newline="\n") as f:
                json.dump(golden, f, indent=2)
                f.write("\n")
            near_tc.append(("tightcocone", case + ".tc.txt"))
            print("TightCocone %s: %d triangles over %d vertices%s" %
                  (case, counts["triangles"], counts["vertices"],
                   "" if reset is None else
                   "; resetting the box %s the surface" %
                   ("changes" if reset["surface_changes"] else "does not change")))

    if args.check:
        bad = []
        for lib_dir, name in compared:
            made = os.path.join(target, lib_dir, name)
            kept = os.path.join(args.out, lib_dir, name)
            if not os.path.exists(kept):
                bad.append("%s/%s: not committed" % (lib_dir, name))
            elif sha256(made) != sha256(kept):
                bad.append("%s/%s: differs from the committed golden" % (lib_dir, name))
        for lib_dir, name in near:
            made = os.path.join(target, lib_dir, name)
            kept = os.path.join(args.out, lib_dir, name)
            if not os.path.exists(kept):
                bad.append("%s/%s: not committed" % (lib_dir, name))
            else:
                d = fields_differ(made, kept)
                if d > FIELD_TOLERANCE:
                    bad.append("%s/%s: differs from the committed golden by %g" %
                               (lib_dir, name, d))
        for lib_dir, name in near_tiles:
            made = os.path.join(target, lib_dir, name)
            kept = os.path.join(args.out, lib_dir, name)
            if not os.path.exists(kept):
                bad.append("%s/%s: not committed" % (lib_dir, name))
            else:
                d = tiles_differ(made, kept)
                if d > TILES_TOLERANCE:
                    bad.append("%s/%s: differs from the committed golden by %g" %
                               (lib_dir, name, d))
        for lib_dir, name in near_ptv:
            made = os.path.join(target, lib_dir, name)
            kept = os.path.join(args.out, lib_dir, name)
            if not os.path.exists(kept):
                bad.append("%s/%s: not committed" % (lib_dir, name))
            else:
                bad += ["%s/%s: %s" % (lib_dir, name, p) for p in ptv_problems(made, kept)]
        for lib_dir, name in near_tc:
            made = os.path.join(target, lib_dir, name)
            kept = os.path.join(args.out, lib_dir, name)
            if not os.path.exists(kept):
                bad.append("%s/%s: not committed" % (lib_dir, name))
            else:
                bad += ["%s/%s: %s" % (lib_dir, name, p) for p in tc_problems(made, kept)]
        if bad:
            print("\n".join(bad))
            print("The made goldens are in", target)
            return 1
        print("the goldens match (%d files: %s)" % (len(compared) + len(near) + len(near_tiles) +
                                                    len(near_ptv) + len(near_tc),
                                                    ", ".join(libs)))
    return 0

if __name__ == "__main__":
    sys.exit(main())
