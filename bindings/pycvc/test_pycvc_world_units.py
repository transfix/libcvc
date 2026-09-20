"""pycvc world_units tests — the SI unit base + display regime from Python.

world_units is the spatial counterpart to world_clock: a canonical SI store
(everything in metres/kg/s/N/J internally) with a display *regime* (SI or
imperial) and a `metres_per_world_unit` factor that pins the app's world-space
doubles to real metres. The binding reshapes the C++ surface (see pycvc.i):

  * the `system`/`dimension`/`config` enums+struct become STRINGS — regime is
    "si"|"imperial"; a dimension is "length"|"mass"|"time"|"velocity"|
    "acceleration"|"force"|"energy"|"angle";
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


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc world_units tests: OK")
