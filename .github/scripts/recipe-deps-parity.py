#!/usr/bin/env python3
"""Fail when libcvc-cuda's recipe dependencies differ from libcvc's.

libcvc-cuda's build.sh and build.ps1 run libcvc's scripts with CVC_ENABLE_CUDA=ON, and the
publish workflow's CUDA jobs install libcvc's dependency closure, so the GPU build links what the
CPU build links. libcvc-cuda's `depends:` is what `cvcpkg pack` bakes into the bundle's
required_deps, and `cvcpkg install` is flat, so a dependency libcvc declares and libcvc-cuda
leaves out ships a libcvc.so that does not load in a consumer's prefix. The list was a hand copy
and drifted twice: first assimp, then curl, yaml-cpp, ftxui, ncurses, joltphysics, qhull and more.

For each platform in libcvc-cuda's build matrix, the (kind, name) pairs that apply there must be
the same in both recipes. An entry with no `platforms:` applies everywhere. Scopes for platforms
libcvc-cuda does not build are not compared.

Usage: recipe-deps-parity.py [repo-root]   (needs PyYAML)
"""

import sys
from pathlib import Path

import yaml

# Build deps of libcvc's that libcvc-cuda leaves out on purpose. googletest only builds the test
# suite, and nothing builds libcvc-cuda's tests: its scripts leave CVC_BUILD_TESTS OFF.
CPU_ONLY = {("build", "googletest")}


def load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def deps_on(recipe, platform):
    """The (kind, name) dependencies a recipe declares for one platform."""
    out = set()
    depends = recipe.get("depends") or {}
    for kind in ("build", "runtime"):
        for dep in depends.get(kind) or []:
            platforms = dep.get("platforms")
            if platforms is None or platform in platforms:
                out.add((kind, dep["name"]))
    return out


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]
    cpu_path = "cvcpkg/recipes/libcvc/recipe.yaml"
    gpu_path = "cvcpkg/recipes/libcvc-cuda/recipe.yaml"
    cpu = load(root / cpu_path)
    gpu = load(root / gpu_path)

    platforms = sorted({entry["platform"] for entry in gpu["build"]["matrix"]})
    bad = False
    for platform in platforms:
        want = deps_on(cpu, platform) - CPU_ONLY
        got = deps_on(gpu, platform)
        for kind, name in sorted(want - got):
            print(f"::error file={gpu_path}::{platform}: libcvc declares {kind} dependency "
                  f"{name}, libcvc-cuda does not")
            bad = True
        for kind, name in sorted(got - want):
            print(f"::error file={gpu_path}::{platform}: libcvc-cuda declares {kind} dependency "
                  f"{name}, libcvc does not")
            bad = True
        print(f"{platform}: libcvc-cuda {len(got)}, libcvc {len(want)} "
              f"(build and runtime, googletest left out)")

    cpu_tools = sorted((cpu.get("depends") or {}).get("host_tools") or [])
    gpu_tools = sorted((gpu.get("depends") or {}).get("host_tools") or [])
    if cpu_tools != gpu_tools:
        print(f"::error file={gpu_path}::host_tools {gpu_tools} differ from libcvc's {cpu_tools}")
        bad = True

    if bad:
        print("::error::libcvc-cuda's depends: must match libcvc's on every platform it builds. "
              "Copy the change from cvcpkg/recipes/libcvc/recipe.yaml.")
        return 1
    print("libcvc-cuda's dependencies match libcvc's on " + ", ".join(platforms))
    return 0


if __name__ == "__main__":
    sys.exit(main())
