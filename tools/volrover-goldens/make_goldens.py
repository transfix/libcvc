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
}


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


def build(volrover, work):
    bdir = os.path.join(work, "build")
    cmd = ["cmake", "-S", HERE, "-B", bdir, "-DCMAKE_BUILD_TYPE=Release",
           "-DVOLROVER_DIR=" + volrover]
    if shutil.which("ninja") and not os.environ.get("CMAKE_GENERATOR"):
        cmd += ["-G", "Ninja"]
    run(cmd, quiet=True)
    run(["cmake", "--build", bdir, "--config", "Release", "--target", "genseg_legacy"], quiet=True)
    for cand in (os.path.join(bdir, "genseg_legacy"), os.path.join(bdir, "genseg_legacy.exe"),
                 os.path.join(bdir, "Release", "genseg_legacy.exe")):
        if os.path.exists(cand):
            return cand
    raise SystemExit("genseg_legacy was not built in " + bdir)


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
    args = ap.parse_args()

    sha = pinned_sha()
    libs = ["genseg"]
    work = os.path.abspath(args.work)
    os.makedirs(work, exist_ok=True)
    volrover = os.path.abspath(args.volrover) if args.volrover else fetch(sha, work, libs)
    exe = build(volrover, work)

    target = os.path.join(work, "check") if args.check else args.out
    if args.check and os.path.exists(target):
        shutil.rmtree(target)
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

    if args.check:
        bad = []
        for case in cases:
            for ext in (".labels.u8", ".indices.txt"):
                made = os.path.join(out_dir, case + ext)
                kept = os.path.join(args.out, "genseg", case + ext)
                if not os.path.exists(kept):
                    bad.append("%s%s: not committed" % (case, ext))
                elif sha256(made) != sha256(kept):
                    bad.append("%s%s: differs from the committed golden" % (case, ext))
        if bad:
            print("\n".join(bad))
            print("The made goldens are in", out_dir)
            return 1
        print("the goldens match (%d GenSeg cases)" % len(cases))
    return 0


if __name__ == "__main__":
    sys.exit(main())
