"""Guard: no SWIG proxy function references an undefined name.

A %pythonappend / %pythonprepend body is pasted verbatim into the generated proxy,
whose signature SWIG picks per function: `def f(self, *args)` when the C++ function
is OVERLOADED, NAMED parameters (`def f(self, viewer)`) when it has a single
signature. A hook written for one shape breaks on the other -- `args[0]` in a
named-parameter proxy (or `viewer` in an *args one) is a NameError on EVERY call,
e.g. ImGuiOverlay(view) / AriRuntime(view, cam, ui) could not be constructed at all.
Adding or removing a C++ overload silently flips the shape, so check the generated
modules statically instead of hoping each hook is exercised by some test.

Resolution is deliberately simple: a name is defined if it is a parameter or bound
anywhere inside the function (nested lambdas/comprehensions included), a global of
the imported module, or a builtin.
"""

import ast
import builtins
import importlib

fails = 0


def _bound_names(fn):
    names = set()
    for node in ast.walk(fn):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
            a = node.args
            for arg in a.posonlyargs + a.args + a.kwonlyargs:
                names.add(arg.arg)
            if a.vararg:
                names.add(a.vararg.arg)
            if a.kwarg:
                names.add(a.kwarg.arg)
            if not isinstance(node, ast.Lambda):
                names.add(node.name)
        elif isinstance(node, ast.Name) and isinstance(node.ctx, (ast.Store, ast.Del)):
            names.add(node.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                names.add((alias.asname or alias.name).split(".")[0])
        elif isinstance(node, ast.ExceptHandler) and node.name:
            names.add(node.name)
        elif isinstance(node, (ast.Global, ast.Nonlocal)):
            names.update(node.names)
    return names


def _functions(tree):
    """Yield (qualname, FunctionDef) for module-level functions and class methods."""
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            yield node.name, node
        elif isinstance(node, ast.ClassDef):
            for item in node.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    yield "%s.%s" % (node.name, item.name), item


def check_module(modname):
    global fails
    try:
        mod = importlib.import_module(modname)
    except ImportError as exc:
        print("  skip: %s not importable (%s)" % (modname, exc))
        return
    path = mod.__file__
    with open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read(), filename=path)
    known = set(vars(mod)) | set(dir(builtins))
    bad = {}  # (qualname, name) -> first line, one report per offending function+name
    for qual, fn in _functions(tree):
        bound = _bound_names(fn)
        for node in ast.walk(fn):
            if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load):
                if node.id not in bound and node.id not in known:
                    bad.setdefault((qual, node.id), node.lineno)
    bad = ["%s:%d %s() uses undefined name %r" % (path, line, qual, name)
           for (qual, name), line in sorted(bad.items(), key=lambda kv: kv[1])]
    if bad:
        fails += len(bad)
        for b in bad:
            print("  [FAIL] " + b)
    else:
        print("  ok: %s -- every proxy function's names resolve" % modname)


if __name__ == "__main__":
    check_module("pycvc")
    check_module("pycvc_gl")
    if fails:
        raise SystemExit("%d undefined-name reference(s) in SWIG proxies" % fails)
    print("PASS")
