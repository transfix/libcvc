#!/usr/bin/env python3
"""Verify every test executable in tests/CMakeLists.txt reaches TEST_TARGETS.

The `check` custom target builds only what is listed in TEST_TARGETS, so a
target declared with add_executable() but never added to the list silently
drops out of `make check` / `ninja check`. It still runs under a bare `ctest`
if a previous build happened to produce the binary, which is exactly what
makes the drift hard to notice.

This script parses the file with a small CMake-aware scanner that tracks the
`if()` nesting, so a target guarded by `if(NOT WIN32)` is only considered
covered by a `list(APPEND TEST_TARGETS ...)` reachable under a compatible
guard. Exits non-zero on drift.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ADD_EXE = re.compile(r"^\s*add_executable\(\s*([A-Za-z0-9_]+)")
SET_LIST = re.compile(r"^\s*set\(\s*TEST_TARGETS\b")
APPEND_LIST = re.compile(r"^\s*list\(\s*APPEND\s+TEST_TARGETS\b(.*)$")
IF_OPEN = re.compile(r"^\s*if\(\s*(.*?)\s*\)\s*$", re.IGNORECASE)
ELSE_MID = re.compile(r"^\s*(else|elseif)\(", re.IGNORECASE)
IF_CLOSE = re.compile(r"^\s*endif\(", re.IGNORECASE)

# Targets that are intentionally not gtest_discover_tests()-registered but must
# still be built by `check`.
NON_GTEST_TARGETS = {"procedural_geometry_test"}


def normalize(cond: str) -> str:
    """Collapse a guard expression to a comparable key."""
    return " ".join(cond.split()).upper()


def scan(path: Path):
    declared: dict[str, tuple[str, ...]] = {}
    listed: dict[str, tuple[str, ...]] = {}
    stack: list[str] = []
    in_set_block = False

    for lineno, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.split("#", 1)[0] if not raw.lstrip().startswith("#") else ""

        if in_set_block:
            for tok in re.findall(r"[A-Za-z0-9_]+", line):
                listed.setdefault(tok, tuple(stack))
            if ")" in line:
                in_set_block = False
            continue

        if SET_LIST.match(line):
            body = line.split("(", 1)[1]
            for tok in re.findall(r"[A-Za-z0-9_]+", body.replace("TEST_TARGETS", "", 1)):
                listed.setdefault(tok, tuple(stack))
            in_set_block = ")" not in body
            continue

        m = APPEND_LIST.match(line)
        if m:
            body = m.group(1)
            closed = ")" in body
            for tok in re.findall(r"[A-Za-z0-9_]+", body.split(")", 1)[0]):
                listed.setdefault(tok, tuple(stack))
            in_set_block = not closed
            continue

        m = ADD_EXE.match(line)
        if m:
            declared[m.group(1)] = tuple(stack)
            continue

        m = IF_OPEN.match(line)
        if m:
            stack.append(normalize(m.group(1)))
            continue
        if ELSE_MID.match(line):
            if stack:
                stack[-1] = "!" + stack[-1]
            continue
        if IF_CLOSE.match(line):
            if stack:
                stack.pop()
            continue

    return declared, listed


def main() -> int:
    path = Path(__file__).with_name("CMakeLists.txt")
    declared, listed = scan(path)

    missing = []
    weaker = []
    for target, guards in sorted(declared.items()):
        if target not in listed:
            missing.append(target)
            continue
        list_guards = set(listed[target])
        # `if(TARGET foo)` is self-guarding: it is true exactly when foo was
        # declared, so it subsumes whatever guarded the add_executable.
        if ("TARGET %s" % target).upper() in list_guards:
            continue
        # Otherwise every guard protecting the add_executable must also
        # protect the list entry, or `check` depends on a nonexistent target.
        unguarded = [g for g in guards if g not in list_guards]
        if unguarded:
            weaker.append((target, unguarded))

    stale = sorted(set(listed) - set(declared))

    if missing:
        print("TEST_TARGETS is missing %d target(s) declared in the file:" % len(missing))
        for t in missing:
            print("  - %s" % t)
    if weaker:
        print("\nTEST_TARGETS entries listed under a weaker guard than the")
        print("add_executable() that declares them:")
        for t, guards in weaker:
            print("  - %s (needs %s)" % (t, ", ".join(guards)))
    if stale:
        print("\nTEST_TARGETS names no add_executable() declares:")
        for t in stale:
            print("  - %s" % t)

    if missing or weaker or stale:
        return 1

    print("OK: all %d test executables reach TEST_TARGETS." % len(declared))
    return 0


if __name__ == "__main__":
    sys.exit(main())
