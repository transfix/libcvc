"""pycvc.ExecPool and the per-Exec groups of threaded execution, Stage 1
(docs/STATE_EXEC_THREADED.md 4.12; S1-Q4, S1-Q5).

ExecPool(app, threads=None, size=None, prefix="exec_pool", fresh_env=False,
idle_timeout=60.0) runs many programs at once on the app's exec_pool
(run_all, register_fn, register_async_fn, threads, close). The cases are S1.md
section 4's pycvc ones: GR-5, IS-2, AR-4, FF-3 and PO-1. The sanitizer jobs
run this script (ci.yml's SAN_PY_TESTS) and require each case's "ok:" line.
"""

import pycvc  # noqa: F401


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc exec_pool tests: OK")
