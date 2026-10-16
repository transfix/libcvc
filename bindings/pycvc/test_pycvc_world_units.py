"""pycvc world_units tests — the SI unit base + display regime from Python.

world_units is the spatial counterpart to world_clock: a canonical SI store
(everything in metres/kg/s/N/J internally) with a display *regime* (SI or
imperial) and a `metres_per_world_unit` factor that pins the app's world-space
doubles to real metres. The binding reshapes the C++ surface (see pycvc.i):

  * the `system`/`dimension`/`config` enums+struct become STRINGS — regime is
    "si"|"imperial"; a dimension is its registry name ("length", "velocity",
    "temperature", "energy_per_length", ... or one the application added);
  * the scalar ctor is world_units(metres_per_world_unit, regime="si");
  * to_display/from_display/unit_symbol/format take the dimension string
    (…_d suffix), and format()/world_point_to_real() return the flatnested
    `measurement{value,unit}` / `coordinate{x,y,z,unit}` proxies.

These pin that contract plus the behaviours the base exists to provide:
SI is stored canonically and displayed 1:1; imperial is a DISPLAY-only
projection with exact factors; a world point is shown in ONE length tier
(m/km or ft/mi) chosen from its largest component so the axes never disagree;
and the base is a per-`app` instance, never a process-global.
"""

import math

import pycvc

# Exact conversion constants, mirrored from inc/cvc/core/world_units.h so the
# assertions below are independent of the implementation's own arithmetic.
M_PER_FOOT = 0.3048
M_PER_MILE = 1609.344
KG_PER_POUND = 0.45359237
N_PER_LBF = 4.4482216152605
J_PER_FTLBF = N_PER_LBF * M_PER_FOOT  # 1.3558179483314004
MPS_PER_MPH = 0.44704
M_PER_KM = 1000.0

app = pycvc.make_app()


def test_defaults():
    u = pycvc.world_units()
    assert u.regime_name() == "si", "default regime is SI"
    assert u.metres_per_world_unit() == 1.0, "default is 1 metre per world unit"
    print("  ok: defaults — SI, 1 m per world unit")


def test_scalar_ctor():
    u = pycvc.world_units(1000.0, "imperial")
    assert u.metres_per_world_unit() == 1000.0
    assert u.regime_name() == "imperial"
    # regime defaults to SI, and any non-"imperial" spelling is SI (the ctor is
    # lenient; set_regime_name is the strict path — see test_regime_setter).
    assert pycvc.world_units(2.0).regime_name() == "si"
    print("  ok: scalar ctor honours metres_per_world_unit + regime")


def test_regime_setter():
    u = pycvc.world_units()
    u.set_regime_name("imperial")
    assert u.regime_name() == "imperial"
    u.set_regime_name("si")
    assert u.regime_name() == "si"
    # An unknown regime must surface as a Python exception, not silently pass.
    try:
        u.set_regime_name("furlongs")
    except Exception:
        pass
    else:
        raise AssertionError("set_regime_name('furlongs') should have raised")
    print("  ok: set_regime_name toggles SI/imperial; bad regime raises")


def test_world_to_metres_scaling():
    # world<->metres uses ONLY metres_per_world_unit; the regime is irrelevant.
    u = pycvc.world_units(1000.0)  # 1 world unit == 1 km
    assert math.isclose(u.world_to_metres(2.0), 2000.0)
    assert math.isclose(u.metres_to_world(2000.0), 2.0)
    # round-trip
    assert math.isclose(u.metres_to_world(u.world_to_metres(3.5)), 3.5, rel_tol=1e-12)
    print("  ok: world<->metres scales by metres_per_world_unit only")


def test_si_display_is_identity():
    u = pycvc.world_units()  # SI
    for dim in ("length", "velocity", "acceleration", "force", "energy", "mass"):
        assert math.isclose(u.to_display_d(5.0, dim), 5.0), f"SI {dim} is 1:1"
        assert math.isclose(u.from_display_d(5.0, dim), 5.0)
    assert u.unit_symbol_d("length") == "m"
    assert u.unit_symbol_d("velocity") == "m/s"
    assert u.unit_symbol_d("acceleration") == "m/s^2"
    assert u.unit_symbol_d("force") == "N"
    assert u.unit_symbol_d("energy") == "J"
    assert u.unit_symbol_d("mass") == "kg"
    assert u.unit_symbol_d("time") == "s"
    # Angle is canonically radians but displayed in DEGREES in both regimes.
    assert u.unit_symbol_d("angle") == "deg"
    assert math.isclose(u.to_display_d(math.pi, "angle"), 180.0, rel_tol=1e-12)
    print("  ok: SI display is 1:1 (except angle=deg); symbols correct")


def test_imperial_display_factors():
    u = pycvc.world_units(1.0, "imperial")
    cases = {
        "length": (M_PER_FOOT, "ft"),
        "velocity": (MPS_PER_MPH, "mph"),
        "acceleration": (M_PER_FOOT, "ft/s^2"),
        "force": (N_PER_LBF, "lbf"),
        "energy": (J_PER_FTLBF, "ft-lbf"),
        "mass": (KG_PER_POUND, "lb"),
    }
    for dim, (si_per_unit, sym) in cases.items():
        # one display unit's worth of SI reads as 1.0 of that unit
        assert math.isclose(u.to_display_d(si_per_unit, dim), 1.0, rel_tol=1e-12), dim
        assert u.unit_symbol_d(dim) == sym, dim
        # from_display is the exact inverse
        assert math.isclose(u.from_display_d(1.0, dim), si_per_unit, rel_tol=1e-12), dim
    # time + angle are regime-independent
    assert u.unit_symbol_d("time") == "s"
    assert u.unit_symbol_d("angle") == "deg"
    print("  ok: imperial display uses exact ft/mph/lbf/ft-lbf/lb factors")


def test_format_length_tiers():
    # SI: metres below a km, kilometres at/above.
    si = pycvc.world_units()
    m = si.format_d(500.0, "length")
    assert m.unit == "m" and math.isclose(m.value, 500.0)
    m = si.format_d(2000.0, "length")
    assert m.unit == "km" and math.isclose(m.value, 2.0)
    # imperial: feet below a mile, miles at/above.
    imp = pycvc.world_units(1.0, "imperial")
    m = imp.format_d(M_PER_FOOT, "length")
    assert m.unit == "ft" and math.isclose(m.value, 1.0, rel_tol=1e-12)
    m = imp.format_d(M_PER_MILE, "length")
    assert m.unit == "mi" and math.isclose(m.value, 1.0, rel_tol=1e-12)
    m = imp.format_d(2.0 * M_PER_MILE, "length")
    assert m.unit == "mi" and math.isclose(m.value, 2.0, rel_tol=1e-12)
    print("  ok: format() auto-selects m/km and ft/mi length tiers")


def test_world_point_to_real_single_tier():
    # SI, 1 world unit == 1 metre. A point at 1500 m picks the km tier from its
    # LARGEST component and reports every axis in that same unit (never mixed).
    u = pycvc.world_units(1.0)
    c = u.world_point_to_real(1500.0, 3.0, 0.0)
    assert c.unit == "km"
    assert math.isclose(c.x, 1.5, rel_tol=1e-12)
    assert math.isclose(c.y, 0.003, rel_tol=1e-9)
    assert math.isclose(c.z, 0.0, abs_tol=1e-12)
    # A small point stays in metres.
    c = u.world_point_to_real(500.0, 0.0, 0.0)
    assert c.unit == "m" and math.isclose(c.x, 500.0)
    # metres_per_world_unit scales BEFORE the tier choice: 2 world units at
    # 1000 m/unit == 2000 m == 2 km.
    u2 = pycvc.world_units(1000.0)
    c = u2.world_point_to_real(2.0, 0.0, 0.0)
    assert c.unit == "km" and math.isclose(c.x, 2.0, rel_tol=1e-12)
    print("  ok: world_point_to_real reports one tier chosen from the max axis")


def test_world_point_to_real_imperial():
    u = pycvc.world_units(1.0, "imperial")
    c = u.world_point_to_real(M_PER_MILE, 0.0, 0.0)  # exactly one mile
    assert c.unit == "mi" and math.isclose(c.x, 1.0, rel_tol=1e-12)
    c = u.world_point_to_real(M_PER_FOOT, 0.0, 0.0)  # one foot -> feet tier
    assert c.unit == "ft" and math.isclose(c.x, 1.0, rel_tol=1e-12)
    print("  ok: imperial world_point_to_real reports ft/mi")


# ── the base is a per-app instance, not a process-global ─────────────────────


def test_app_world_units_is_per_app_with_si_default():
    a = pycvc.make_app()
    assert a.world_units().regime_name() == "si"
    assert a.world_units().metres_per_world_unit() == 1.0
    # The accessor returns the app's OWN stable instance: a mutation through it
    # is visible on the next call (same underlying object, not a fresh copy).
    a.world_units().set_regime_name("imperial")
    a.world_units().set_metres_per_world_unit(1000.0)
    assert a.world_units().regime_name() == "imperial"
    assert a.world_units().metres_per_world_unit() == 1000.0
    print("  ok: app.world_units() is a stable per-app instance, SI by default")


def test_app_world_clock_is_per_app():
    a = pycvc.make_app()
    c = a.world_clock()
    assert abs(c.fixed_dt() - 1.0 / 120.0) < 1e-12, "default 120 Hz quantum"
    assert c.tick() == 0
    print("  ok: app.world_clock() is a stable per-app 120 Hz instance")


def test_world_bases_are_not_global_singletons():
    a = pycvc.make_app()
    b = pycvc.make_app()
    # Distinct apps hold distinct world bases: mutating one never bleeds over.
    a.world_units().set_regime_name("imperial")
    assert b.world_units().regime_name() == "si", "b's units untouched by a"
    a.world_clock().advance(0.5)  # advances a's clock only
    assert a.world_clock().tick() > 0
    assert b.world_clock().tick() == 0, "b's clock untouched by a"
    print("  ok: world_clock/world_units are per-app, not process singletons")


# ── the unit registry: built-ins, custom units, Python function units ────────


def test_registry_builtins_and_exact_anchors():
    r = pycvc.unit_registry()
    assert r.to_si(1.0, "nmi") == 1852.0
    assert r.to_si(1.0, "kWh") == 3.6e6
    assert math.isclose(r.convert(60.0, "mph", "km/h"), 96.56064, rel_tol=1e-12)
    assert math.isclose(r.convert(100.0, "degC", "degF"), 212.0, rel_tol=1e-12)
    assert math.isclose(r.to_si(30.0, "dBm"), 1.0, rel_tol=1e-12)  # 30 dBm = 1 W
    assert r.unit("degF").kind_name() == "affine"
    assert r.unit("dBm").kind_name() == "function"
    assert r.unit("ft").kind_name() == "linear"
    assert r.dimension("velocity").si_symbol == "m/s"
    assert r.dimension("curvature").length_exponent == -1
    assert "nmi" in list(r.units("length"))
    assert r.dimension_of("psi") == "pressure"
    print("  ok: registry built-ins convert with exact anchors (nmi, kWh, degF, dBm)")


def test_registry_custom_linear_and_dimension():
    r = pycvc.unit_registry()
    r.add_dimension("data_rate", "bit/s")
    r.add_linear("Mbit/s", "data_rate", 1.0e6, "megabit per second")
    assert r.convert(54.0, "Mbit/s", "bit/s") == 54.0e6
    assert r.unit("Mbit/s").name == "megabit per second"
    assert list(r.units("data_rate")) == ["Mbit/s", "bit/s"]
    for bad in (
        lambda: r.add_linear("ft", "length", 0.3048),  # duplicate symbol
        lambda: r.add_linear("x", "nosuchdim", 1.0),
        lambda: r.add_linear("zero", "length", 0.0),
        lambda: r.convert(1.0, "ft", "kg"),  # across dimensions
        lambda: r.to_si(1.0, "furlongs"),
    ):
        try:
            bad()
        except RuntimeError:
            pass
        else:
            raise AssertionError("expected RuntimeError")
    print("  ok: custom dimensions and linear units drop in; bad ones raise")


def test_registry_python_function_unit():
    r = pycvc.unit_registry()
    # A nonlinear unit defined in Python: decibel-watts.
    r.add_function("dBW", "power", lambda v: 10.0 ** (v / 10.0), lambda w: 10.0 * math.log10(w))
    assert r.unit("dBW").kind_name() == "function"
    assert math.isclose(r.to_si(10.0, "dBW"), 10.0, rel_tol=1e-12)
    assert math.isclose(r.convert(0.0, "dBW", "dBm"), 30.0, rel_tol=1e-12)
    # An exception in the callable surfaces as a RuntimeError naming it.
    r.add_function("broken", "power", lambda v: 1.0 / 0.0, lambda w: w)
    try:
        r.to_si(1.0, "broken")
    except RuntimeError as e:
        assert "ZeroDivisionError" in str(e), str(e)
    else:
        raise AssertionError("a raising conversion should raise")
    try:
        r.add_function("nope", "power", 3, lambda w: w)
    except RuntimeError:
        pass
    else:
        raise AssertionError("a non-callable conversion should raise")
    print("  ok: Python callables register as function units; their errors surface")


def test_registry_empty_and_copies_are_independent():
    e = pycvc.unit_registry.empty()
    assert len(e.dimensions()) == 0 and not e.has_unit("m")
    a = pycvc.unit_registry()
    b = pycvc.unit_registry(a)
    b.add_linear("furlong", "length", 201.168)
    assert b.has_unit("furlong") and not a.has_unit("furlong")
    print("  ok: empty() is empty; a copied registry is independent")


# ── world_units on the registry: per-dimension display, custom dimensions ────


def test_display_unit_override_and_new_dimensions():
    u = pycvc.world_units()
    assert u.display_unit_d("velocity") == "m/s"
    u.set_display_unit_d("velocity", "kn")
    assert u.unit_symbol_d("velocity") == "kn"
    assert math.isclose(u.to_display_d(1852.0 / 3600.0, "velocity"), 1.0, rel_tol=1e-12)
    u.set_display_unit_d("length", "nmi")  # a length override turns off promotion
    m = u.format_d(3704.0, "length")
    assert m.unit == "nmi" and math.isclose(m.value, 2.0, rel_tol=1e-12)
    u.clear_display_unit_d("length")
    assert u.format_d(3704.0, "length").unit == "km"
    imp = pycvc.world_units(1.0, "imperial")
    assert imp.unit_symbol_d("temperature") == "degF"
    assert math.isclose(imp.to_display_d(373.15, "temperature"), 212.0, rel_tol=1e-12)
    assert imp.unit_symbol_d("energy_per_length") == "Wh/mi"
    print("  ok: per-dimension display overrides; new dimensions have regime defaults")


def test_world_units_registry_is_per_instance():
    u = pycvc.world_units()
    reg = u.registry()
    reg.add_dimension("data_rate", "bit/s")
    reg.add_linear("Mbit/s", "data_rate", 1.0e6)
    u.set_display_unit_d("data_rate", "Mbit/s")
    m = u.format_d(2.5e6, "data_rate")
    assert m.unit == "Mbit/s" and math.isclose(m.value, 2.5)
    assert math.isclose(u.convert(1.0, "Mbit/s", "bit/s"), 1.0e6)
    assert not pycvc.world_units().registry().has_dimension("data_rate")
    # The registry proxy keeps its world_units alive.
    reg2 = pycvc.world_units().registry()
    assert reg2.has_unit("m")
    print("  ok: each world_units owns its registry; custom dimensions work by name")


# ── SI <-> nav's normalized frame ─────────────────────────────────────────────


def test_normalized_frame_conversions():
    u = pycvc.world_units()
    f = pycvc.world_units_normalized_frame()
    f.scale = 0.05
    assert math.isclose(u.to_normalized_d(2.0, "velocity", f), 0.1, rel_tol=1e-15)
    assert math.isclose(u.from_normalized_d(0.1, "velocity", f), 2.0, rel_tol=1e-15)
    assert math.isclose(u.to_normalized_d(0.5, "curvature", f), 10.0, rel_tol=1e-15)
    assert u.to_normalized_d(0.4, "angle", f) == 0.4
    km = pycvc.world_units(1000.0)
    g = pycvc.world_units_normalized_frame()
    g.scale = 2.0
    assert math.isclose(km.to_normalized_d(500.0, "length", g), 1.0, rel_tol=1e-15)
    # Points: m = n / S + centre at one metre per world unit (nav's snapshot).
    f.cx, f.cy = 37.5, -12.0
    p = u.normalized_point_to_si(0.3125, -0.75, 0.0, f)
    assert p.x == 0.3125 / 0.05 + 37.5 and p.y == -0.75 / 0.05 + -12.0
    n = u.si_point_to_normalized(p.x, p.y, 0.0, f)
    assert math.isclose(n.x, 0.3125, rel_tol=1e-14) and math.isclose(n.y, -0.75, rel_tol=1e-14)
    f.scale = 0.0
    try:
        u.to_normalized_d(1.0, "length", f)
    except RuntimeError:
        pass
    else:
        raise AssertionError("a zero frame scale should raise")
    print("  ok: SI <-> normalized frame by length exponent; points match nav's o/S + c")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc world_units tests: OK")
