"""Robust, deterministic tests for pycvc.Exec — the state_exec DSL runtime.

pycvc.Exec wraps cvc::state_exec: a Lisp-like program evaluator driven by a
round-robin process scheduler, bound to the SAME process state-tree root as
pycvc.State. run_program(src) parses, runs to completion, and returns the
program's result rendered as a string.

The headline tests prove the Python <-> state <-> DSL round trip: state written
from Python is readable by a DSL program via (state-get ...), and state written
by a DSL program via (state-set ...) is then readable from Python via
State.get — because both facades share cvc::state::instance(process_ctx()).
"""

import pycvc


# ── literals / arithmetic / result rendering ────────────────────────


def test_literal_int_result():
    ex = pycvc.Exec()
    assert ex.run_program("(begin (set x 5) x)") == "5"
    print("  ok: (begin (set x 5) x) -> \"5\"")


def test_arithmetic():
    ex = pycvc.Exec()
    assert ex.run_program("(+ 2 3)") == "5"
    assert ex.run_program("(* 6 7)") == "42"
    assert ex.run_program("(- 10 4)") == "6"
    print("  ok: arithmetic programs render their integer results")


def test_string_result_is_unquoted():
    ex = pycvc.Exec()
    # str-concat yields a string; the facade returns it raw (no surrounding
    # quotes) so values round-trip cleanly through the state tree.
    assert ex.run_program('(str-concat "ab" "cd")') == "abcd"
    print("  ok: string results come back unquoted")


def test_bool_result():
    ex = pycvc.Exec()
    assert ex.run_program("(< 1 2)") == "#t"
    assert ex.run_program("(> 1 2)") == "#f"
    print("  ok: boolean results render as the DSL's #t / #f literals")


def test_nil_result_is_empty_string():
    ex = pycvc.Exec()
    # state-set evaluates to nil; the facade renders "no value" as "".
    assert ex.run_program('(state-set "nilcheck.a" "1")') == ""
    print("  ok: a nil (no-value) result renders as an empty string")


def test_parse_error_raises():
    ex = pycvc.Exec()
    try:
        ex.run_program("(begin (set x")  # unbalanced
    except Exception:
        print("  ok: a malformed program raises")
        return
    raise AssertionError("expected a malformed program to raise")


# ── Python -> state -> DSL: a DSL program reads state written from Python ──


def test_dsl_reads_state_written_from_python():
    st = pycvc.State()
    ex = pycvc.Exec()
    st.set("shared.x", "hello-from-python")
    result = ex.run_program('(state-get "shared.x")')
    assert result == "hello-from-python", result
    print("  ok: DSL (state-get) reads a value written from Python")


# ── DSL -> state -> Python: Python reads state written by a DSL program ──


def test_python_reads_state_written_by_dsl():
    st = pycvc.State()
    ex = pycvc.Exec()
    ex.run_program('(state-set "dsl.out" "wrote-from-dsl")')
    assert st.get("dsl.out") == "wrote-from-dsl"
    print("  ok: Python State.get reads a value written by a DSL (state-set)")


# ── Full Python <-> state <-> DSL round trip ────────────────────────


def test_full_roundtrip():
    st = pycvc.State()
    ex = pycvc.Exec()
    # Python writes the input.
    st.set("round.in", "21")
    # DSL reads it, transforms it, writes the output, and returns it.
    prog = (
        '(begin '
        '  (set v (state-get "round.in")) '
        '  (state-set "round.out" (str-concat v v)) '
        '  (state-get "round.out"))'
    )
    result = ex.run_program(prog)
    assert result == "2121", result
    # Python reads the DSL's output back out of the shared tree.
    assert st.get("round.out") == "2121"
    print("  ok: full Python<->state<->DSL round trip (write, transform, read back)")


def test_roundtrip_numeric_with_int_coercion():
    st = pycvc.State()
    ex = pycvc.Exec()
    st.set("calc.a", "7")
    # state values are strings; the DSL coerces with (int ...) then does math.
    result = ex.run_program('(+ (int (state-get "calc.a")) 3)')
    assert result == "10", result
    print("  ok: DSL reads a Python-set numeric string, coerces, and computes")


def test_poll_changes_sees_dsl_writes():
    # A State facade that has written a path is subscribed to it, so a
    # subsequent DSL write to that same node is observed by poll_changes.
    st = pycvc.State()
    ex = pycvc.Exec()
    st.set("observed.p", "seed")  # subscribe via the facade
    assert set(st.poll_changes()) == {"observed.p"}
    ex.run_program('(state-set "observed.p" "changed-by-dsl")')
    assert set(st.poll_changes()) == {"observed.p"}
    assert st.get("observed.p") == "changed-by-dsl"
    print("  ok: poll_changes observes a DSL write to a facade-watched node")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
    print("pycvc.Exec tests: OK")
