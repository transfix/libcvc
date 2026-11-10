"""pycvc's parameter tables (pycvc_params.i): keywords, validate(), from_state / to_state and
from_dict / to_dict generated from a cvc::param_table, on contour_tree_options
(cvc/volume/contour_tree_params.h). The VolumeRover ports' T8 / T11 rows for the shared piece
(docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4, §1.5)."""

import pycvc


def test_keywords_set_fields():
    o = pycvc.contour_tree_options(max_dim=64, quantize_u8=True)
    assert o.max_dim == 64
    assert o.quantize_u8 is True
    d = pycvc.contour_tree_options()
    assert d.max_dim == 0 and d.quantize_u8 is False, "the defaults"
    print("  ok: keywords over the defaults")


def test_unknown_keyword_names_the_valid_ones():
    try:
        pycvc.contour_tree_options(max_dims=64)
    except TypeError as e:
        msg = str(e)
        assert "unknown keyword 'max_dims'" in msg, msg
        assert "valid: max_dim, quantize_u8" in msg, msg
    else:
        raise AssertionError("an unknown keyword must raise TypeError")
    print("  ok: unknown keyword -> TypeError listing", list(pycvc.contour_tree_options.fields()))


def test_fields_dict_and_repr():
    assert list(pycvc.contour_tree_options.fields()) == ["max_dim", "quantize_u8"]
    o = pycvc.contour_tree_options.from_dict({"max_dim": 32})
    assert o.to_dict() == {"max_dim": 32, "quantize_u8": False}
    assert repr(o) == "contour_tree_options(max_dim=32, quantize_u8=False)", repr(o)
    assert o.validate() == []
    print("  ok: fields / from_dict / to_dict / repr / validate")


def test_state_round_trip_and_properties():
    app = pycvc.make_app()
    node = pycvc.State.instance(app).child("doc.ct")
    pycvc.contour_tree_options(max_dim=48, quantize_u8=True).to_state(node)
    assert node.child("max_dim").value() == "48"
    assert node.child("quantize_u8").value() == "1"
    assert node.child("max_dim").property("unit") == "voxel"
    assert node.child("max_dim").property("type") == "int"
    back = pycvc.contour_tree_options.from_state(node)
    assert back.max_dim == 48 and back.quantize_u8 is True
    # Missing keys are seeded with their defaults.
    fresh = pycvc.State.instance(app).child("doc.fresh")
    d = pycvc.contour_tree_options.from_state(fresh)
    assert d.max_dim == 0
    assert fresh.child("max_dim").value() == "0"
    assert fresh.child("quantize_u8").property("default") == "0"
    print("  ok: to_state / from_state round trip, seeding and properties")


def test_bad_state_value_raises_value_error():
    app = pycvc.make_app()
    node = pycvc.State.instance(app).child("n")
    node.child("max_dim").value("lots")
    try:
        pycvc.contour_tree_options.from_state(node)
    except ValueError as e:
        assert "max_dim: 'lots' is not a non-negative integer" in str(e), str(e)
    else:
        raise AssertionError("an unparseable key must raise ValueError")
    print("  ok: a bad key -> ValueError naming it")


def test_text_and_markdown():
    o = pycvc.contour_tree_options()
    o.set_text("max_dim", "128")
    assert o.max_dim == 128
    assert o.get_text("quantize_u8") == "0"
    try:
        o.set_text("max_dim", "-3")
    except ValueError as e:
        assert "max_dim" in str(e)
    else:
        raise AssertionError("bad text must raise ValueError")
    md = pycvc.contour_tree_options.markdown()
    assert "| `max_dim` | int | `0` | voxel |" in md, md
    print("  ok: set_text / get_text / markdown")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("test_pycvc_params: all passed")
