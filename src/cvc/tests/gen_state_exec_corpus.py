#!/usr/bin/env python3
"""Regenerate the state_exec parse corpus (state_exec_corpus/scripts.txt).

The corpus is a frozen snapshot of the DSL source text the repo already carries, so the
grammar-driven parser (docs/STATE_EXEC_GRAMMAR.md) can be checked against how the
pre-grammar parser read every one of them. It collects:

* every YAML scalar in a tracked ``*.ari`` file whose text starts with ``(`` (``init``,
  ``on``, ``bind``, ``*_when`` and friends),
* every ``lisp``/``scheme`` fenced block in a tracked ``*.md`` file (unlabeled blocks are
  skipped: the ones that start with ``(`` are ASCII diagrams),
* every C++ string literal (adjacent literals concatenated, escapes decoded) that starts
  with ``(`` and ends with ``)`` in a tracked ``src/**/*.cpp``/``*.h`` file that mentions
  ``state_exec`` or ``ariadne``. A one-line literal must start with ``(`` itself, which
  skips messages like ``" (expected path:...)"``, and the parentheses must balance, which
  skips prefixes a test completes at run time,
* every Python string constant of that shape in ``bindings/pycvc/test_pycvc_*.py``.

Identical scripts are kept once, under the first source that has them. The expected parse
of each script is not computed here: ``state_exec_corpus_test`` writes
``state_exec_corpus/expected.txt`` when run with ``CVC_UPDATE_GOLDENS=1``.

Usage (from anywhere in the checkout; needs PyYAML):

    python src/cvc/tests/gen_state_exec_corpus.py
"""

from __future__ import annotations

import ast
import re
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent / "state_exec_corpus" / "scripts.txt"

# A corpus entry is "@@@ script <source>" + the script's lines; no script line may start with
# "@@@ ", which the writer checks.
MARK = "@@@ "


def tracked(*patterns: str) -> list[str]:
    out = subprocess.run(
        ["git", "ls-files", "--", *patterns],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return sorted(p for p in out.splitlines() if p)


def balanced(text: str) -> bool:
    """True if the parentheses outside strings and ; comments balance."""
    depth, i, n = 0, 0, len(text)
    while i < n:
        c = text[i]
        if c == ";":
            while i < n and text[i] != "\n":
                i += 1
        elif c == '"':
            i += 1
            while i < n and text[i] != '"':
                i += 2 if text[i] == "\\" else 1
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth < 0:
                return False
        i += 1
    return depth == 0


def looks_like_program(text: str) -> bool:
    # Only for literals in C++ and Python sources, where a fragment (a prefix the test
    # completes at run time) is common: those are skipped as unbalanced.
    s = text.strip()
    if "\n" not in s and not text.startswith("("):
        return False
    return len(s) >= 3 and s.startswith("(") and s.endswith(")") and balanced(s)


# -- .ari ------------------------------------------------------------------------------------


def ari_scripts(path: str):
    # BaseLoader keeps every scalar a string, so the key `on:` is not read as YAML 1.1 `true`.
    doc = yaml.load((ROOT / path).read_text(encoding="utf-8"), Loader=yaml.BaseLoader)

    def walk(node, where):
        if isinstance(node, dict):
            for k, v in node.items():
                yield from walk(v, f"{where}.{k}" if where else str(k))
        elif isinstance(node, list):
            for i, v in enumerate(node):
                yield from walk(v, f"{where}[{i}]")
        elif isinstance(node, str) and node.lstrip().startswith("("):
            yield f"{path}:{where}", node

    yield from walk(doc, "")


# -- Markdown --------------------------------------------------------------------------------

FENCE = re.compile(r"^\s*(`{3,})\s*([\w+-]*)\s*$")


def md_scripts(path: str):
    lines = (ROOT / path).read_text(encoding="utf-8").split("\n")
    i = 0
    while i < len(lines):
        m = FENCE.match(lines[i])
        if not m:
            i += 1
            continue
        ticks, lang = m.group(1), m.group(2)
        j = i + 1
        while j < len(lines) and not lines[j].strip().startswith(ticks):
            j += 1
        body = "\n".join(lines[i + 1 : j])
        if lang in ("lisp", "scheme"):
            yield f"{path}:{i + 1}", body
        i = j + 1


# -- C++ -------------------------------------------------------------------------------------

CPP_TOKEN = re.compile(
    r"""
      (?P<comment>//[^\n]*|/\*.*?\*/)
    | (?P<raw>(?:u8|u|U|L)?R"(?P<delim>[^()\\\s]{0,16})\((?P<rawbody>.*?)\)(?P=delim)")
    | (?P<str>(?:u8|u|U|L)?"(?P<body>(?:[^"\\\n]|\\.)*)")
    | (?P<chr>'(?:[^'\\\n]|\\.)*')
    | (?P<ws>\s+)
    | (?P<other>.)
    """,
    re.DOTALL | re.VERBOSE,
)
CPP_ESC = re.compile(
    r"\\(x[0-9A-Fa-f]+|u[0-9A-Fa-f]{4}|U[0-9A-Fa-f]{8}|[0-7]{1,3}|.)", re.DOTALL
)
SIMPLE = {
    "n": "\n",
    "t": "\t",
    "r": "\r",
    "a": "\a",
    "b": "\b",
    "f": "\f",
    "v": "\v",
    "\\": "\\",
    '"': '"',
    "'": "'",
    "?": "?",
}


def cpp_unescape(body: str) -> str:
    # Escapes become the bytes the compiler would put in a narrow (UTF-8) literal; the result is
    # decoded as UTF-8 so a "\xC3\xA9" pair reads back as the character it encodes.
    out = bytearray()

    def emit_cp(cp: int):
        out.extend(chr(cp).encode("utf-8", "surrogatepass"))

    pos = 0
    for m in CPP_ESC.finditer(body):
        out.extend(body[pos : m.start()].encode("utf-8"))
        e = m.group(1)
        if e[0] == "x":
            out.append(int(e[1:], 16) & 0xFF)
        elif e[0] in "uU":
            emit_cp(int(e[1:], 16))
        elif e[0] in "01234567":
            out.append(int(e, 8) & 0xFF)
        else:
            out.extend(SIMPLE.get(e, "\\" + e).encode("utf-8"))
        pos = m.end()
    out.extend(body[pos:].encode("utf-8"))
    return out.decode("utf-8", "surrogateescape")


def cpp_scripts(path: str):
    text = (ROOT / path).read_text(encoding="utf-8")
    if "state_exec" not in text and "ariadne" not in text:
        return
    pending, start = None, 0  # concatenation of adjacent literals, and where it began
    for m in CPP_TOKEN.finditer(text):
        kind = m.lastgroup
        if kind in ("raw", "str"):
            piece = (
                m.group("rawbody") if kind == "raw" else cpp_unescape(m.group("body"))
            )
            if pending is None:
                pending, start = piece, m.start()
            else:
                pending += piece
        elif kind in ("ws", "comment"):
            continue
        else:
            if pending is not None and looks_like_program(pending):
                yield f"{path}:{text.count(chr(10), 0, start) + 1}", pending
            pending = None
    if pending is not None and looks_like_program(pending):
        yield f"{path}:{text.count(chr(10), 0, start) + 1}", pending


# -- Python ----------------------------------------------------------------------------------


def py_scripts(path: str):
    tree = ast.parse((ROOT / path).read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Constant)
            and isinstance(node.value, str)
            and looks_like_program(node.value)
        ):
            yield f"{path}:{node.lineno}", node.value


# -- main ------------------------------------------------------------------------------------


def main() -> int:
    sources = []
    sources += [s for p in tracked("*.ari") for s in ari_scripts(p)]
    sources += [s for p in tracked("*.md") for s in md_scripts(p)]
    sources += [s for p in tracked("src/*.cpp", "src/*.h") for s in cpp_scripts(p)]
    sources += [
        s for p in tracked("bindings/pycvc/test_pycvc_*.py") for s in py_scripts(p)
    ]

    seen: set[str] = set()
    entries = []
    for where, script in sources:
        script = script.replace("\r\n", "\n")
        if script in seen:
            continue
        seen.add(script)
        if any(line.startswith(MARK) for line in script.split("\n")):
            print(
                f"{where}: a line starts with {MARK!r}; cannot store it",
                file=sys.stderr,
            )
            return 1
        entries.append((where, script))

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", encoding="utf-8", newline="\n", errors="surrogateescape") as f:
        f.write(
            "# state_exec parse corpus. Generated by src/cvc/tests/gen_state_exec_corpus.py;\n"
        )
        f.write(
            "# do not edit by hand. Each entry is a '@@@ script <source>' line, then the script.\n"
        )
        for where, script in entries:
            f.write(f"{MARK}script {where}\n{script}\n")
    print(f"{len(entries)} scripts -> {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
