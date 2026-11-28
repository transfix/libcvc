#!/usr/bin/env python3
"""Make (or check) cvc::mol's T1 goldens from TexMol's own PDB parser and GOABlur.

docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §1 item 5: each TexMol port is checked against its
TexMol original. This script:

  1. fetches transfix/TexMol at the sha in texmol.sha (only the directories the parser and GOABlur
     need), or uses --texmol DIR;
  2. builds blur_legacy and parser_legacy with this directory's CMakeLists.txt (never part of
     libcvc's build);
  3. runs blur_legacy on each case below, and parser_legacy once, on
     examples/data/molecules/1grl.pdb;
  4. writes the goldens to src/cvc/tests/data/texmol-goldens/blur/ (or --out) and .../parser/ (or
     --parser-out), or, with --check, makes them in a scratch directory and compares them with
     what is committed.

The goldens:
  1grl.atoms.txt        the atoms TexMol's parser read, in its order (x y z radius, %.9g); the same
                        for every case, so written once
  <case>.f32            the density (float32 little-endian, x fastest)
  <case>.grid.txt       the grid's dims, then its box's min and max (%.9g)
  <case>.json           the case, the TexMol sha, the platform and the files' SHA-256
  parser/1grl.tree.txt  the parser's group tree, a line a group, each residue with its phi and psi
                        (parser_driver.cpp), and 1grl.tree.json beside it

--check compares the atoms, grids and tree exactly, each density to within 1e-5 relative and each
torsion to within 1e-6, since a compiler may round a float sum or an acos differently.

    python3 tools/texmol-goldens/make_goldens.py            # write the goldens
    python3 tools/texmol-goldens/make_goldens.py --check    # they must not change

Needs git, cmake (with a C++ compiler; Ninja when it is on PATH) and Python 3.8+.
"""

import argparse
import array
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
TEXMOL_URL = "https://github.com/transfix/TexMol.git"
SPARSE = ["inc/PDBParser", "src/PDBParser", "inc/Blurmaps", "src/Blurmaps", "inc/Utility",
          "src/Utility", "inc/UsefulMath", "src/UsefulMath", "inc/SimpleVolumeData",
          "src/SimpleVolumeData", "inc/DPG", "inc/ByteOrder", "src/ByteOrder", "inc/ColorManager",
          "src/ColorManager", "inc/TexMol"]
PDB = os.path.join(REPO, "examples", "data", "molecules", "1grl.pdb")
# name: (resolution in Å, blobbiness)
CASES = {
    "1grl_r1.5": (1.5, -2.3442),  # TexMol's default blobbiness
    "1grl_coarse": (2.0, -0.5),   # a low-resolution map, the skeleton demo's kind
}
TOLERANCE = 1e-5
TORSION_TOLERANCE = 1e-6  # radians


def pinned_sha():
    with open(os.path.join(HERE, "texmol.sha")) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                return line
    raise SystemExit("texmol.sha holds no sha")


def run(cmd, cwd=None, quiet=False):
    print("+", " ".join(cmd), flush=True)
    r = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE if quiet else None,
                       stderr=subprocess.STDOUT if quiet else None, text=True)
    if r.returncode != 0:
        if quiet and r.stdout:
            sys.stdout.write(r.stdout)
        raise SystemExit("failed (%d): %s" % (r.returncode, " ".join(cmd)))
    return r


def fetch(sha, work):
    """A sparse, shallow checkout of TexMol at `sha` in work/texmol-<sha>."""
    dst = os.path.join(work, "texmol-" + sha[:12])
    stamp = os.path.join(dst, ".texmol-goldens-sha")
    if os.path.exists(stamp):
        with open(stamp) as f:
            if f.read().split() == [sha] + SPARSE:
                return dst
        shutil.rmtree(dst)
    os.makedirs(dst, exist_ok=True)
    run(["git", "init", "-q"], cwd=dst)
    run(["git", "remote", "add", "origin", TEXMOL_URL], cwd=dst)
    run(["git", "sparse-checkout", "init", "--no-cone"], cwd=dst)
    run(["git", "sparse-checkout", "set", "--no-cone"] + ["/" + p for p in SPARSE], cwd=dst)
    run(["git", "fetch", "-q", "--depth", "1", "origin", sha], cwd=dst)
    run(["git", "checkout", "-q", "FETCH_HEAD"], cwd=dst)
    head = run(["git", "rev-parse", "HEAD"], cwd=dst, quiet=True).stdout.strip()
    if head != sha:
        raise SystemExit("fetched %s, not the pinned %s" % (head, sha))
    with open(stamp, "w") as f:
        f.write(" ".join([sha] + SPARSE))
    return dst


def build(texmol, work, target="blur_legacy"):
    bdir = os.path.join(work, "build")
    cmd = ["cmake", "-S", HERE, "-B", bdir, "-DCMAKE_BUILD_TYPE=Release", "-DTEXMOL_DIR=" + texmol]
    if shutil.which("ninja") and not os.environ.get("CMAKE_GENERATOR"):
        cmd += ["-G", "Ninja"]
    run(cmd, quiet=True)
    run(["cmake", "--build", bdir, "--config", "Release", "--target", target], quiet=True)
    for cand in (os.path.join(bdir, target), os.path.join(bdir, target + ".exe"),
                 os.path.join(bdir, "Release", target + ".exe")):
        if os.path.exists(cand):
            return cand
    raise SystemExit(target + " was not built in " + bdir)


def trees_differ(made, kept):
    """Why two parser trees (parser_driver.cpp's) differ, or None: every line the same but a
    residue's phi and psi, which may differ by TORSION_TOLERANCE (another compiler's acos)."""
    with open(made) as f, open(kept) as g:
        a, b = f.read().splitlines(), g.read().splitlines()
    if len(a) != len(b):
        return "%d lines, not %d" % (len(a), len(b))
    for i, (x, y) in enumerate(zip(a, b)):
        if x == y:
            continue
        xs, ys = x.split(), y.split()
        if (len(xs) != len(ys) or not xs or xs[0] != "residue" or xs[:-2] != ys[:-2] or
                x[:len(x) - len(x.lstrip())] != y[:len(y) - len(y.lstrip())]):
            return "line %d: %r, not %r" % (i + 1, x, y)
        for u, v in zip(xs[-2:], ys[-2:]):
            if u == "-" or v == "-":
                if u != v:
                    return "line %d: %r, not %r" % (i + 1, x, y)
            elif abs(float(u) - float(v)) > TORSION_TOLERANCE:
                return "line %d: a torsion %s, not %s" % (i + 1, u, v)
    return None


def sha256(path, text=False):
    """The file's SHA-256; a text file's with CRLF read as LF, as git checks it out on Windows."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        data = f.read()
    h.update(data.replace(b"\r\n", b"\n") if text else data)
    return h.hexdigest()


def describe_platform():
    return "%s %s, %s, Python %s" % (platform.system(), platform.machine(),
                                    platform.platform(terse=True), platform.python_version())


def read_f32(path):
    a = array.array("f")
    with open(path, "rb") as f:
        a.frombytes(f.read())
    if sys.byteorder != "little":
        a.byteswap()
    return a


def densities_differ(made, kept):
    """Why two densities differ by more than TOLERANCE, or None."""
    a, b = read_f32(made), read_f32(kept)
    if len(a) != len(b):
        return "%d voxels, not %d" % (len(a), len(b))
    worst = 0.0
    for x, y in zip(a, b):
        if x != y:
            worst = max(worst, abs(x - y) / max(1e-30, abs(y)))
    return None if worst <= TOLERANCE else "a voxel %.3g off (relative)" % worst


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--work", default=os.path.join(tempfile.gettempdir(), "texmol-goldens"),
                    help="where the TexMol checkout and the build go")
    ap.add_argument("--texmol", help="an existing TexMol checkout (must be at the pinned sha)")
    ap.add_argument("--out", default=os.path.join(REPO, "src", "cvc", "tests", "data",
                                                   "texmol-goldens", "blur"))
    ap.add_argument("--parser-out", default=os.path.join(REPO, "src", "cvc", "tests", "data",
                                                          "texmol-goldens", "parser"))
    ap.add_argument("--check", action="store_true",
                    help="make the goldens in a scratch directory and compare them with --out")
    args = ap.parse_args()

    sha = pinned_sha()
    work = os.path.abspath(args.work)
    os.makedirs(work, exist_ok=True)
    texmol = os.path.abspath(args.texmol) if args.texmol else fetch(sha, work)
    exe = build(texmol, work)
    parser_exe = build(texmol, work, "parser_legacy")

    out = os.path.join(work, "check") if args.check else args.out
    parser_out = os.path.join(work, "check-parser") if args.check else args.parser_out
    for d in (out, parser_out):
        if args.check and os.path.exists(d):
            shutil.rmtree(d)
        os.makedirs(d, exist_ok=True)

    # The parser's group tree and torsions (parser_driver.cpp).
    tree = os.path.join(parser_out, "1grl.tree.txt")
    run([parser_exe, PDB, tree])
    with open(os.path.join(parser_out, "1grl.tree.json"), "w", newline="\n") as f:
        json.dump({
            "library": "TexMol PDBParser (group tree, protein torsions)",
            "texmol": sha,
            "input": "examples/data/molecules/1grl.pdb",
            "input_sha256": sha256(PDB, text=True),
            "tree_sha256": sha256(tree, text=True),
            "made_by": "tools/texmol-goldens/make_goldens.py",
            "platform": describe_platform(),
        }, f, indent=2)
        f.write("\n")
    atoms = None
    for case, (resolution, blobbiness) in sorted(CASES.items()):
        prefix = os.path.join(out, case)
        run([exe, PDB, repr(resolution), repr(blobbiness), prefix])
        with open(prefix + ".atoms.txt", "rb") as f:
            these = f.read()
        os.remove(prefix + ".atoms.txt")
        if atoms is None:
            atoms = these
            with open(os.path.join(out, "1grl.atoms.txt"), "wb") as f:
                f.write(atoms)
        elif these != atoms:
            raise SystemExit(case + ": the parser read other atoms than the first case")
        golden = {
            "library": "TexMol PDBParser + Blurmaps GOABlur (electron density, van der Waals radii)",
            "texmol": sha,
            "case": case,
            "input": "examples/data/molecules/1grl.pdb",
            "input_sha256": sha256(PDB, text=True),
            "resolution": resolution,
            "blobbiness": blobbiness,
            "density_sha256": sha256(prefix + ".f32"),
            "grid_sha256": sha256(prefix + ".grid.txt", text=True),
            "atoms_sha256": sha256(os.path.join(out, "1grl.atoms.txt"), text=True),
            "made_by": "tools/texmol-goldens/make_goldens.py",
            "platform": describe_platform(),
        }
        with open(prefix + ".json", "w", newline="\n") as f:
            json.dump(golden, f, indent=2)
            f.write("\n")

    if not args.check:
        print("wrote the goldens to " + out + " and " + parser_out)
        return
    problems = []
    why = trees_differ(tree, os.path.join(args.parser_out, "1grl.tree.txt"))
    if why:
        problems.append("1grl.tree.txt: " + why)
    for name in ["1grl.atoms.txt"] + [c + ".grid.txt" for c in sorted(CASES)]:
        with open(os.path.join(out, name), "rb") as f, open(os.path.join(args.out, name), "rb") as g:
            if f.read().replace(b"\r\n", b"\n") != g.read().replace(b"\r\n", b"\n"):
                problems.append(name + " differs")
    for case in sorted(CASES):
        why = densities_differ(os.path.join(out, case + ".f32"), os.path.join(args.out, case + ".f32"))
        if why:
            problems.append(case + ".f32: " + why)
    if problems:
        raise SystemExit("the goldens changed (the new ones are in %s):\n  %s" %
                         (out, "\n  ".join(problems)))
    print("the goldens match")


if __name__ == "__main__":
    main()
