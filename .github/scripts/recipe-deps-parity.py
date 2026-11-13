#!/usr/bin/env python3
"""Fail when a -cuda recipe's dependencies differ from its CPU peer's.

Each -cuda recipe under cvcpkg/recipes/ builds its CPU peer's sources with the same CMake, against
libcvc-cuda instead of libcvc, so the GPU build links what the CPU build links. Its `depends:` is
what `cvcpkg pack` bakes into the bundle's required_deps, and `cvcpkg install` is flat, so a
dependency the CPU recipe declares and the -cuda one leaves out ships a library that does not
load in a consumer's prefix. The lists are hand copies and drifted: libcvc-cuda lost assimp, then
curl, yaml-cpp, ftxui, qhull and more; pycvc-gl-cp311-cuda never had vtk-python-cp311; cvcgl-cuda
never had imgui-w32, sdl3 or ffmpeg-lgpl.

For each pair in PAIRS and each platform in the -cuda recipe's build matrix, the CPU recipe's
(kind, name) pairs that apply there, after the pair's swaps, must be the -cuda recipe's. An entry
with no `platforms:` applies everywhere. Scopes for platforms the -cuda recipe does not build are
not compared. host_tools must be the same. Every *-cuda recipe directory must be listed in PAIRS.

Usage: recipe-deps-parity.py [repo-root]   (needs PyYAML)
"""

import sys
from pathlib import Path

import yaml

RECIPES = "cvcpkg/recipes"

# -cuda recipe -> (its CPU peer, {CPU dependency name: the -cuda recipe's name for it},
#                  {(kind, name) the CPU recipe declares that the -cuda recipe leaves out}).
# A name not in the swap map must appear unchanged in both recipes.
PAIRS = {
    # libcvc-cuda's scripts run libcvc's with CVC_ENABLE_CUDA=ON. googletest only builds the test
    # suite, and nothing builds libcvc-cuda's tests: its scripts leave CVC_BUILD_TESTS OFF.
    "libcvc-cuda": ("libcvc", {}, {("build", "googletest")}),
    # Not built or published today (publish-cvcpkg.yml installs the one cvcgl); kept in step so a
    # build of it is cvcgl's build.
    "cvcgl-cuda": ("cvcgl", {"libcvc": "libcvc-cuda"}, set()),
    "cvc-cli-cuda": ("cvc-cli", {"libcvc": "libcvc-cuda"}, set()),
    "pycvc-cp311-cuda": ("pycvc-cp311", {"libcvc": "libcvc-cuda"}, set()),
    # cvcgl is not swapped: cvcGL has no CUDA code and cvc::volume's ABI is variant-invariant, so
    # pycvc-gl-cp311-cuda links the one published cvcgl (#174).
    "pycvc-gl-cp311-cuda": (
        "pycvc-gl-cp311",
        {"libcvc": "libcvc-cuda", "pycvc-cp311": "pycvc-cp311-cuda"},
        set(),
    ),
}


def load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def deps_on(recipe, platform):
    """The (kind, name) dependencies a recipe declares for one platform (None: any platform)."""
    out = set()
    depends = recipe.get("depends") or {}
    for kind in ("build", "runtime"):
        for dep in depends.get(kind) or []:
            platforms = dep.get("platforms")
            if platform is None or platforms is None or platform in platforms:
                out.add((kind, dep["name"]))
    return out


def check(root, gpu_name, cpu_name, swaps, cpu_only):
    """Print an ::error line per difference; return True when the two recipes agree."""
    cpu_path = f"{RECIPES}/{cpu_name}/recipe.yaml"
    gpu_path = f"{RECIPES}/{gpu_name}/recipe.yaml"
    cpu = load(root / cpu_path)
    gpu = load(root / gpu_path)
    ok = True

    cpu_names = {name for _, name in deps_on(cpu, None)}
    for old in sorted(set(swaps) - cpu_names):
        print(f"::error file={gpu_path}::swap {old} -> {swaps[old]} is stale: "
              f"{cpu_name} declares no {old}")
        ok = False

    platforms = sorted({entry["platform"] for entry in gpu["build"]["matrix"]})
    counts = []
    for platform in platforms:
        want = {(kind, swaps.get(name, name)) for kind, name in deps_on(cpu, platform) - cpu_only}
        got = deps_on(gpu, platform)
        for kind, name in sorted(want - got):
            print(f"::error file={gpu_path}::{platform}: {cpu_name} declares {kind} dependency "
                  f"{name}, {gpu_name} does not")
            ok = False
        for kind, name in sorted(got - want):
            print(f"::error file={gpu_path}::{platform}: {gpu_name} declares {kind} dependency "
                  f"{name}, {cpu_name} does not")
            ok = False
        counts.append(f"{platform} {len(got)}/{len(want)}")

    cpu_tools = sorted((cpu.get("depends") or {}).get("host_tools") or [])
    gpu_tools = sorted((gpu.get("depends") or {}).get("host_tools") or [])
    if cpu_tools != gpu_tools:
        print(f"::error file={gpu_path}::host_tools {gpu_tools} differ from {cpu_name}'s "
              f"{cpu_tools}")
        ok = False

    swapped = ", ".join(f"{a} -> {b}" for a, b in sorted(swaps.items())) or "no swaps"
    left_out = ", ".join(name for _, name in sorted(cpu_only))
    note = f"; {left_out} left out" if left_out else ""
    print(f"{gpu_name} vs {cpu_name}: {'ok' if ok else 'DIFFERS'} "
          f"({', '.join(counts)} -cuda/CPU deps; {swapped}{note})")
    return ok


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]

    bad = False
    on_disk = sorted(p.parent.name for p in (root / RECIPES).glob("*-cuda/recipe.yaml"))
    for name in on_disk:
        if name not in PAIRS:
            print(f"::error file={RECIPES}/{name}/recipe.yaml::{name} is not in "
                  f".github/scripts/recipe-deps-parity.py's PAIRS: add it with its CPU peer "
                  f"and swap map")
            bad = True
    for name in sorted(PAIRS):
        if name not in on_disk:
            print(f"::error::PAIRS lists {name}, but {RECIPES}/{name}/recipe.yaml does not exist")
            bad = True

    for gpu_name in sorted(set(PAIRS) & set(on_disk)):
        cpu_name, swaps, cpu_only = PAIRS[gpu_name]
        if not check(root, gpu_name, cpu_name, swaps, cpu_only):
            bad = True

    if bad:
        print("::error::A -cuda recipe's depends: must match its CPU peer's on every platform it "
              "builds, apart from the swaps in .github/scripts/recipe-deps-parity.py. Copy the "
              "change from the CPU recipe.")
        return 1
    print("every -cuda recipe's dependencies match its CPU peer's")
    return 0


if __name__ == "__main__":
    sys.exit(main())
