#!/usr/bin/env python3
"""The writable-data allowlist only shrinks: fail on any line a change adds.

libcvc keeps no process-wide state (VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3). The scan (scan.py)
fails on a writable symbol that is not on the allowlist; this check stops the way around it,
adding the symbol's line. A change may delete lines, and may not add one, renamed or not.

Usage: ratchet.py BASE_ALLOWLIST ALLOWLIST

BASE_ALLOWLIST is the list on the change's base (``git show <base>:tools/singleton_scan/...``).
Exits 0 when every line of ALLOWLIST is on BASE_ALLOWLIST, 1 when some are new, 2 when either
file does not parse.
"""

from __future__ import annotations

import os
import sys
from typing import Sequence

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scan  # noqa: E402


def added(base_text: str, text: str) -> tuple[list[scan.Entry], list[str]]:
    """The entries of `text` that `base_text` does not have, and parse errors in either."""
    base, base_errors = scan.parse_allowlist(base_text)
    head, errors = scan.parse_allowlist(text)
    problems = [f"base: {e}" for e in base_errors] + errors
    keys = {e.key for e in base}
    return [e for e in head if e.key not in keys], problems


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    with open(args[0], encoding="utf-8") as fh:
        base_text = fh.read()
    with open(args[1], encoding="utf-8") as fh:
        text = fh.read()
    new, problems = added(base_text, text)
    if problems:
        for p in problems:
            print(f"error: {p}", file=sys.stderr)
        return 2
    if not new:
        print("ok: the allowlist gained no line.")
        return 0
    print(f"{len(new)} line{'' if len(new) == 1 else 's'} added to {args[1]}:")
    for e in new:
        print(f"  + line {e.lineno}: {e.line()}")
    print(
        "\nlibcvc keeps no process-wide state. Move each object onto the app, or into an object"
        " the caller owns, and leave the allowlist as it was: it only shrinks."
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
