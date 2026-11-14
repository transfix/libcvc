"""The scripting hosts' Python console (ARIADNE.md 9.6): cvc://components/py_console.ari's engine.

The engine (python_engine.cpp) compiles this into the host and imports it as ``_ariadne_console``.
A document whose ``python:`` block says ``console: true`` gets ``consoles_for``'s ``Consoles``, and
two natives over it: ``(python-console ID SOURCE)`` runs SOURCE as one cell of console ID, and
``(python-console-clear ID)`` empties that console's scrollback.

A cell runs as a notebook runs one, in the namespace the document's block ran in, so it sees the
block's names and a name one cell sets is there for the next (in every console of the document):

- its stdout and stderr are captured, up to MAX_OUTPUT characters;
- a last statement that is an expression shows its value's repr, kept as ``_`` in the namespace;
- ``input()`` reads an empty stdin and raises EOFError, instead of reading the host's terminal;
- an exception is reported with its traceback from the cell's own frames on, and ``ok`` is false;
  SystemExit too: the document keeps running.

The capture is the cell's thread's alone. Other threads go on while a cell runs (it gives up the
GIL: an async export on the pool, a thread of the document's), so a cell never reassigns
``sys.stdin``, ``sys.stdout`` or ``sys.stderr``. The first document with a console puts a
``_Stream`` in each instead, which hands the running cell's stream to the cell's thread and the
stream it stands in for to every other thread, and between cells.

The scrollback echoes each cell, ``>>> `` before its first line and ``... `` before the rest, then
its output, and keeps the last MAX_LINES lines. A cell's source is in linecache, for tracebacks,
until its console is cleared or its document goes. Keep this file ASCII: the build embeds it in raw
string literals of 8000 bytes, which a character could straddle.
"""

import ast
import contextvars
import io
import linecache
import os
import sys
import traceback

import _ariadne_python

MAX_LINES = 2000  # a console's scrollback, in lines; the oldest go first
MAX_TEXT = 1 << 20  # ... and in characters
MAX_OUTPUT = 64 * 1024  # what one cell's output keeps, in characters

_FILE = "ariadne_console.py"  # this module's frames, left out of a cell's traceback

# The running cell's (stdin, output), in the context of the thread that runs it; None elsewhere.
_cell = contextvars.ContextVar("ariadne_console_cell", default=None)


def _clean(text):
    """``text`` as the DSL can take it: UTF-8, with a lone surrogate written as its escape."""
    return text.encode("utf-8", "backslashreplace").decode("utf-8")


class _Output(io.TextIOBase):
    """A cell's stdout and stderr: the first ``cap`` characters, and a count of the rest."""

    def __init__(self, cap):
        super().__init__()
        self._parts = []
        self._room = cap
        self._dropped = 0

    def writable(self):
        return True

    def write(self, s):
        if not isinstance(s, str):
            raise TypeError(f"write() argument must be str, not {type(s).__name__}")
        keep = s[: max(self._room, 0)]
        if keep:
            self._parts.append(keep)
            self._room -= len(keep)
        self._dropped += len(s) - len(keep)
        return len(s)

    def text(self):
        out = "".join(self._parts)
        if self._dropped:
            if out and not out.endswith("\n"):
                out += "\n"
            out += f"... {self._dropped} more characters\n"
        return out


class _Stream:
    """``sys.stdin``, ``sys.stdout`` or ``sys.stderr`` once a document has a console: the running
    cell's stream on the cell's thread, and ``original`` everywhere else. A stream a cell left
    behind (a context copied into a task that outlives it) is closed, so it falls back too."""

    __slots__ = ("_index", "_original")

    def __init__(self, index, original):
        self._index = index  # 0 for stdin; 1 for stdout and stderr, which a cell shares
        self._original = original

    def _target(self):
        cell = _cell.get()
        if cell is not None and not cell[self._index].closed:
            return cell[self._index]
        return self._original

    def __getattr__(self, name):
        if name in _Stream.__slots__:  # not set yet (a copy): don't look through _target
            raise AttributeError(name)
        return getattr(self._target(), name)

    def __iter__(self):
        return iter(self._target())

    def __repr__(self):
        return f"<console stream over {self._target()!r}>"


def _install():
    """Put a _Stream in sys.stdin, sys.stdout and sys.stderr, over what each holds now (a stream
    that is None, with no console, as one that reads nothing and writes nowhere). Each load with a
    console does it, so a stream something has put there since gets one too."""
    for name, index in (("stdin", 0), ("stdout", 1), ("stderr", 1)):
        stream = getattr(sys, name)
        if isinstance(stream, _Stream):
            continue
        if stream is None:
            stream = io.StringIO() if index == 0 else _Output(0)
        setattr(sys, name, _Stream(index, stream))


def _describe(exc):
    """``exc`` with its traceback from the cell's frames on: this module's are left out."""
    tb = exc.__traceback__
    while tb is not None and tb.tb_frame.f_code.co_filename == _FILE:
        tb = tb.tb_next
    return "".join(traceback.format_exception(type(exc), exc, tb))


class Console:
    """One console: its cells run in ``namespace``, and its scrollback is ``text()``. ``tag`` sets
    its cells' file names apart from another load's (they key linecache)."""

    def __init__(self, namespace, name="console", tag=""):
        self.namespace = namespace
        self.name = name
        self._tag = f" #{tag}" if tag else ""
        self._cells = 0
        self._files = []  # its cells' entries in linecache
        self._lines = []
        self._size = 0  # the characters in _lines, newlines included

    def _compile(self, source, filename):
        """The cell as code for its body and, when it ends in an expression, code for that."""
        tree = ast.parse(source, filename, "exec")
        last = None
        if tree.body and isinstance(tree.body[-1], ast.Expr):
            last = ast.Expression(tree.body.pop().value)
        body = compile(tree, filename, "exec")
        return body, (compile(last, filename, "eval") if last is not None else None)

    def _execute(self, source, filename):
        out = _Output(MAX_OUTPUT)
        stdin = io.StringIO()  # input() gets EOFError, not the host's terminal
        ok = True
        token = _cell.set((stdin, out))  # this thread's streams (_Stream), not sys.*
        try:
            try:
                body, last = self._compile(source, filename)
            except Exception as e:  # a SyntaxError, or a cell too deep to compile
                lines = source.splitlines()
                at = getattr(e, "lineno", None) or 0
                if isinstance(e, SyntaxError) and not e.text and 0 < at <= len(lines):
                    e.text = lines[at - 1] + "\n"  # one compile() found: show its line
                out.write("".join(traceback.format_exception_only(type(e), e)))
                return False, out.text()
            try:
                exec(body, self.namespace)
                if last is not None:
                    value = eval(last, self.namespace)
                    if value is not None:
                        self.namespace["_"] = value
                        out.write(repr(value) + "\n")
            except SystemExit as e:  # sys.exit() in a cell ends the cell, not the document
                code = "" if e.code is None else f": {e.code!r}"
                out.write(f"SystemExit{code}\n")
                ok = False
            except BaseException as e:
                out.write(_describe(e))
                ok = False
        finally:
            _cell.reset(token)
            stdin.close()
            out.close()
        return ok, out.text()

    def run(self, source):
        """Run ``source`` as the next cell. Returns ``(ok, output)``: whether it ran to its end,
        and what it wrote (its prints, its value's repr, or its traceback)."""
        self._cells += 1
        filename = f"<{self.name}:{self._cells}{self._tag}>"
        # Its lines, for the tracebacks of this cell and of the functions it defines.
        linecache.cache[filename] = (len(source), None, source.splitlines(True), filename)
        self._files.append(filename)
        ok, output = self._execute(source, filename)
        output = _clean(output)
        lines = source.rstrip().splitlines() or [""]
        self._append([">>> " + lines[0]] + ["... " + line for line in lines[1:]])
        if output:
            self._append((output[:-1] if output.endswith("\n") else output).split("\n"))
        return ok, output

    def _append(self, lines):
        for line in lines:
            self._lines.append(line)
            self._size += len(line) + 1
        drop = 0
        while drop < len(self._lines) - 1 and (
            len(self._lines) - drop > MAX_LINES or self._size > MAX_TEXT
        ):
            self._size -= len(self._lines[drop]) + 1
            drop += 1
        if drop:
            del self._lines[:drop]

    def text(self):
        """The scrollback: the cells and their output, newline-joined."""
        return "\n".join(self._lines)

    def forget(self):
        """Take its cells' source out of linecache: nothing else would, while the process lives."""
        for filename in self._files:
            linecache.cache.pop(filename, None)
        self._files = []

    def clear(self):
        """Empty the scrollback, and let go of the cells' source (forget): a traceback through a
        function a cleared cell defined shows no lines."""
        self._lines = []
        self._size = 0
        self.forget()


class Consoles:
    """A document's consoles by id, every one in the document's namespace."""

    def __init__(self, namespace):
        self.namespace = namespace
        self._consoles = {}
        self._tag = os.urandom(3).hex()  # this load's, in its cells' file names

    def _console(self, cid):
        c = self._consoles.get(cid)
        if c is None:
            c = self._consoles[cid] = Console(self.namespace, f"console {cid}", self._tag)
        return c

    def run(self, cid, source):
        """Run ``source`` as a cell of console ``cid``: ``(ok, output, log)``, with log the
        console's scrollback."""
        c = self._console(cid)
        ok, output = c.run(source)
        return bool(ok), output, c.text()

    def clear(self, cid):
        c = self._consoles.get(cid)
        if c is not None:
            c.clear()

    def close(self):
        """The document goes (the engine lets go of these consoles): its cells leave linecache."""
        for c in self._consoles.values():
            c.forget()


def consoles_for(block, path):
    """The consoles of the document at ``path``, whose ``python:`` block is ``block`` and has just
    run (load_document): None unless the block asks for them with ``console: true``."""
    if not isinstance(block, dict) or not _ariadne_python._flag(block.get("console")):
        return None
    _install()
    return Consoles(vars(sys.modules[_ariadne_python.document_module_name(path)]))
