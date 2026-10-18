"""state_exec parity, part 1: parsing, values, parse_error and the snapshot codec.

pycvc wraps cvc::state_exec under the C++ names; docs/PYCVC_STATE_EXEC_PARITY.md has
the approach and the full C++ vs Python inventory. Values convert to native Python
types, with pycvc.symbol for symbols and the opaque pycvc.value for closures and
other non-data values.
"""

import pycvc
from pycvc import symbol


def _raises(exc, fn, *args):
    try:
        fn(*args)
    except exc as e:
        return e
    raise AssertionError(f"{fn.__name__}{args!r} did not raise {exc.__name__}")


# ── parse / parse_all ───────────────────────────────────────────────────


def test_parse_returns_python_values():
    assert pycvc.parse("42") == 42
    assert pycvc.parse("-2.5") == -2.5
    assert pycvc.parse('"café"') == "café"
    assert pycvc.parse("#t") is True
    assert pycvc.parse("nil") is None
    assert pycvc.parse("") is None  # nil, as in C++
    assert pycvc.parse('b"\\x00\\xff"') == b"\x00\xff"
    assert pycvc.parse('(define x "s")') == [symbol("define"), symbol("x"), "s"]
    assert pycvc.parse("'x") == [symbol("quote"), symbol("x")]


def test_parse_all_returns_every_form():
    assert pycvc.parse_all("1 (2 3) ; c\n x") == [1, [2, 3], symbol("x")]
    assert pycvc.parse_all("  ; nothing\n") == []


def test_parse_options_apply():
    o = pycvc.parse_options()
    o.max_depth = 2
    assert pycvc.parse("((1))", o) == [[1]]
    _raises(pycvc.parse_error, pycvc.parse, "(((1)))", o)


# ── parse_error ─────────────────────────────────────────────────────────


def test_parse_error_carries_its_position():
    o = pycvc.parse_options()
    o.max_depth = 2
    e = _raises(pycvc.parse_error, pycvc.parse, "(a\n  ((b)))", o)
    assert isinstance(e, RuntimeError)  # existing `except RuntimeError` still works
    assert (e.line, e.col) == (2, 4)
    assert "nesting deeper than 2" in str(e)
    e = _raises(pycvc.parse_error, pycvc.parse_all, '(a "unterminated')
    assert "unterminated string" in str(e)


def test_exec_run_raises_parse_error_either_way():
    ex = pycvc.Exec(pycvc.make_app())
    # Exec reads a script inside its process by default (step_tokens 1024)...
    e = _raises(pycvc.parse_error, ex.run, "(begin\n  (set x")
    assert "unclosed parenthesis" in str(e) and e.line == 2
    # ...or parses it at submit (step_tokens 0): the same error, at the same place.
    o = pycvc.parse_options()
    o.step_tokens = 0
    ex.set_parser_defaults(o)
    e2 = _raises(pycvc.parse_error, ex.run, "(begin\n  (set x")
    assert (str(e2), e2.line, e2.col) == (str(e), e.line, e.col)


# ── symbol ──────────────────────────────────────────────────────────────


def test_symbol_is_the_cpp_struct():
    s = symbol("x")
    assert s.name == "x"
    assert s == symbol("x") and s != symbol("y")
    assert s != "x"  # a symbol is not a string
    assert hash(s) == hash(symbol("x"))
    assert repr(s) == "symbol('x')"
    assert {symbol("a"): 1}[symbol("a")] == 1


# ── to_string / values_equal ────────────────────────────────────────────


def test_to_string_and_values_equal():
    assert pycvc.to_string([symbol("a"), 1, "s", None]) == '(a 1 "s" nil)'
    assert pycvc.to_string(b"\x00A") == 'b"\\x00A"'
    assert pycvc.to_string((1, 2)) == "(1 2)"  # a tuple converts as a list
    assert pycvc.to_string(bytearray(b"ab")) == 'b"ab"'
    assert pycvc.values_equal([1, {"k": "v"}], [1, {"k": "v"}])
    assert not pycvc.values_equal([1], [2])


# ── strict conversion ───────────────────────────────────────────────────


def test_conversion_is_strict():
    _raises(TypeError, pycvc.to_string, {1: 2})  # dict keys must be str
    _raises(OverflowError, pycvc.to_string, 2**70)
    _raises(TypeError, pycvc.to_string, {1, 2})  # a set has no state_exec equivalent
    deep = []
    for _ in range(1100):
        deep = [deep]
    _raises(ValueError, pycvc.to_string, deep)
    loop = []
    loop.append(loop)
    _raises(ValueError, pycvc.to_string, loop)


def test_registered_fns_get_symbols_and_strict_returns():
    ex = pycvc.Exec(pycvc.make_app())
    seen = []
    ex.register_fn("see", lambda x: seen.append(x) or x)
    assert ex.run("(see 'abc)") == "abc"
    assert seen[0] == symbol("abc")  # was the plain string "abc"
    ex.register_fn("bad", lambda: {1, 2})
    e = _raises(RuntimeError, ex.run, "(bad)")
    assert "cannot convert set" in str(e), str(e)


# ── opaque values ───────────────────────────────────────────────────────


def test_closures_cross_as_opaque_values_and_come_back():
    ex = pycvc.Exec(pycvc.make_app())
    kept = []
    ex.register_fn("keep", lambda f: kept.append(f) or 1)
    ex.register_fn("give", lambda: kept[0])
    ex.run("(keep (lambda (x) (+ x 1)))")
    f = kept[0]
    assert isinstance(f, pycvc.value)  # was the text "<closure>"
    assert f.type_name() == "closure"
    assert not f.is_nil() and f.is_truthy()
    assert "closure" in repr(f)
    assert ex.run("(begin (set g (give)) (g 41))") == "42"


# ── snapshot codec ──────────────────────────────────────────────────────


def test_encode_and_decode_value_on_a_state_node():
    app = pycvc.make_app()
    node = pycvc.State.instance(app).child("snap.v")
    original = {
        "name": "café",
        "items": [1, 2.5, None, True, symbol("s")],
        "blob": b"\x00\x01\xff",
    }
    pycvc.encode_value(node, original)
    assert pycvc.decode_value(node) == original


# ── script_reader ───────────────────────────────────────────────────────


def test_script_reader_reads_in_slices():
    r = pycvc.script_reader('(a 1) (b "two")', pycvc.parse_options())
    steps = 1
    while not r.step(1):
        steps += 1
    assert r.forms() == [[symbol("a"), 1], [symbol("b"), "two"]]
    assert steps > 3
    assert r.source() == '(a 1) (b "two")'
    assert r.options().max_depth == pycvc.parse_options().max_depth  # the C++ default


def test_script_reader_raises_parse_error():
    r = pycvc.script_reader("(a", pycvc.parse_options())
    _raises(pycvc.parse_error, r.step, 100)


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc state_exec parity tests: OK")
