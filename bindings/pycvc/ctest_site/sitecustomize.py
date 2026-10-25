"""The pycvc ctest entries' interpreter setup on Windows (bindings/pycvc/CMakeLists.txt,
cvc_pycvc_test_env, puts this directory first on PYTHONPATH there; nowhere else).

Python 3.8+ loads an extension module's DLLs without consulting PATH, so `import _pycvc` cannot
find libcvc's dependencies unless their directories are added with os.add_dll_directory. The
test environment lists them in CVC_DLL_DIRS (os.pathsep-separated): the build's runtime output
directories and the bin/ of each CMAKE_PREFIX_PATH entry. Python imports this module at startup,
so the test script, and any Python it starts with the same environment, sees them.
"""

import importlib.machinery
import importlib.util
import os
import sys

if sys.platform == "win32":
    for _d in os.environ.get("CVC_DLL_DIRS", "").split(os.pathsep):
        if _d and os.path.isdir(_d):
            os.add_dll_directory(os.path.abspath(_d))

# This module shadows any sitecustomize the interpreter has: run that one too.
_here = os.path.normcase(os.path.dirname(os.path.abspath(__file__)))
_rest = [p for p in sys.path if os.path.normcase(os.path.abspath(p or os.curdir)) != _here]
_spec = importlib.machinery.PathFinder.find_spec("sitecustomize", _rest)
if _spec is not None and _spec.loader is not None:
    _spec.loader.exec_module(importlib.util.module_from_spec(_spec))
