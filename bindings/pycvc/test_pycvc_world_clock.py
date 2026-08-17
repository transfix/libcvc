"""pycvc world_clock tests — the fixed-quantum simulation clock from Python.

The binding reshapes part of the C++ surface (see pycvc.i): `step_result` is
wrapped as itself (via flatnested) so advance()/step_once() return .steps /
.alpha / .dropped_steps, while the nested `config` struct and the `mode` enum
become a scalar constructor and a string mode respectively.
These tests pin that contract plus the behaviours the clock exists to provide:
determinism independent of frame pacing, a stall clamp that REPORTS what it
dropped, and modes that actually gate time.
"""

import math

import pycvc


def test_defaults():
    c = pycvc.world_clock()
    assert abs(c.fixed_dt() - 1.0 / 120.0) < 1e-12, "default quantum should be 120 Hz"
    assert c.tick() == 0 and c.t() == 0.0
    assert c.mode_name() == "live"
    assert c.scale() == 1.0
    assert c.total_dropped() == 0
    print("  ok: defaults — 120 Hz, live, at tick 0")


def test_scalar_ctor_and_validation():
    c = pycvc.world_clock(1.0 / 60.0, 2.0, 4)
    assert abs(c.fixed_dt() - 1.0 / 60.0) < 1e-12
    assert c.scale() == 2.0
    # An invalid quantum must surface as a Python exception, not an abort.
    for bad in (0.0, -1.0, float("nan")):
        try:
            pycvc.world_clock(bad)
        except Exception:
            pass
        else:
            raise AssertionError("world_clock(%r) should have raised" % bad)
    print("  ok: scalar ctor honours fixed_dt/scale; invalid config raises")


def test_advance_returns_steps_alpha_dropped():
    c = pycvc.world_clock(0.1)  # 10 Hz quantum, easy arithmetic
    r = c.advance(0.25)
    assert r.steps == 2, r.steps
    assert abs(r.alpha - 0.5) < 1e-9, r.alpha
    assert r.dropped_steps == 0
    assert c.tick() == 2
    assert abs(c.t() - 0.2) < 1e-12, "world seconds derive from the tick"
    # The banked ~0.05 carries into the next call rather than being lost: 0.06
    # alone is under one quantum, so a step here proves the remainder survived.
    # (Deliberately not 0.05 — that lands exactly on the quantum boundary, where
    # binary rounding of 0.25 - 2*0.1 decides the outcome, not the clock.)
    r = c.advance(0.06)
    assert r.steps == 1 and r.dropped_steps == 0
    assert c.tick() == 3
    print("  ok: advance -> .steps/.alpha/.dropped_steps; remainder banks across calls")


def test_determinism_across_frame_pacing():
    """The whole point: same world time regardless of how it was chopped up."""
    steady = pycvc.world_clock(1.0 / 120.0)
    for _ in range(60):
        steady.advance(1.0 / 60.0)  # a clean 60 Hz host
    ragged = pycvc.world_clock(1.0 / 120.0)
    for dt in (0.004, 0.021, 0.009, 0.004, 0.012) * 20:  # 0.05 s * 20 = same 1.0 s
        ragged.advance(dt)
    # Within one tick, not bitwise equal: summing 100 binary floats does not land
    # on exactly the same real as summing 60 of them. The clock is deterministic
    # for a given input sequence; it cannot make two different sequences sum
    # identically. Landing within a quantum is the property that actually matters.
    assert abs(steady.tick() - ragged.tick()) <= 1, (steady.tick(), ragged.tick())
    assert abs(steady.t() - 1.0) < 1e-9
    # Determinism proper: replaying the SAME sequence reproduces the tick exactly.
    again = pycvc.world_clock(1.0 / 120.0)
    for dt in (0.004, 0.021, 0.009, 0.004, 0.012) * 20:
        again.advance(dt)
    assert again.tick() == ragged.tick()
    print("  ok: determinism — jittery pacing lands on the same tick as steady")


def test_no_drift_over_many_steps():
    c = pycvc.world_clock(1.0 / 120.0)
    for _ in range(12000):
        c.advance(1.0 / 120.0)
    assert c.tick() == 12000, c.tick()
    # t() is tick * fixed_dt, never an accumulated sum — so it does not drift.
    assert abs(c.t() - 100.0) < 1e-9, c.t()
    print("  ok: 12000 steps, no drift in t()")


def test_stall_clamp_reports_drops():
    c = pycvc.world_clock(0.01, 1.0, 4)  # at most 4 quanta per advance
    r = c.advance(1.0)  # 100 quanta demanded
    assert r.steps == 4, r.steps
    assert r.dropped_steps > 0, "the clamp must report what it discarded, not hide it"
    assert c.total_dropped() == r.dropped_steps
    print("  ok: stall clamp caps steps at 4 and reports %d dropped" % r.dropped_steps)


def test_scale():
    # 2 world seconds per wall second. The quantum is 0.125 (a power of two, so
    # every value here is binary-exact) — with 0.1 the accumulator's repeated
    # subtraction lands a hair under the last quantum and the step count is
    # decided by rounding rather than by the scale under test. max_steps is
    # raised so the stall clamp (default 8) does not mask it either.
    c = pycvc.world_clock(0.125, 2.0, 16)
    assert c.advance(0.5).steps == 8  # -> 1.0 world seconds -> 8 quanta
    c.set_scale(0.0)  # 0 pauses
    assert c.advance(10.0).steps == 0
    c.set_scale(-5.0)  # negative clamps to 0, it does not run time backwards
    assert c.scale() == 0.0
    print("  ok: scale fast-forwards, 0 pauses, negative clamps to 0")


def test_modes():
    c = pycvc.world_clock(0.1)
    c.set_mode_name("paused")
    assert c.mode_name() == "paused"
    assert c.advance(1.0).steps == 0 and c.tick() == 0, "paused must bank no time"

    c.set_mode_name("stepping")
    assert c.advance(1.0).steps == 0, "stepping ignores wall time"
    assert c.step_once().steps == 1 and c.tick() == 1

    c.set_mode_name("replay")
    c.seek_tick(41)
    assert c.tick() == 41
    assert abs(c.t() - 4.1) < 1e-12

    try:
        c.set_mode_name("sideways")
    except Exception:
        pass
    else:
        raise AssertionError("an unknown mode name should raise")
    print("  ok: paused/stepping/replay gate time; bad mode name raises")


def test_pending_and_reset():
    c = pycvc.world_clock(0.1)
    c.advance(0.25)
    assert 0.0 <= c.pending_seconds() < 0.1
    assert abs(c.pending_seconds() - 0.05) < 1e-9
    c.reset()
    assert c.tick() == 0 and c.pending_seconds() == 0.0
    assert c.fixed_dt() == 0.1, "reset keeps config"
    print("  ok: pending_seconds exposes the banked remainder; reset clears it")


def test_alpha_is_a_presentation_fraction():
    c = pycvc.world_clock(0.1)
    for i in range(1, 10):
        alpha = c.advance(0.01).alpha
        assert 0.0 <= alpha < 1.0, alpha
        assert abs(alpha - (i * 0.1)) < 1e-9, (i, alpha)
    assert not math.isnan(alpha)
    print("  ok: alpha stays in [0,1) and tracks the fraction into the next quantum")


if __name__ == "__main__":
    test_defaults()
    test_scalar_ctor_and_validation()
    test_advance_returns_steps_alpha_dropped()
    test_determinism_across_frame_pacing()
    test_no_drift_over_many_steps()
    test_stall_clamp_reports_drops()
    test_scale()
    test_modes()
    test_pending_and_reset()
    test_alpha_is_a_presentation_fraction()
    print("pycvc world_clock tests: OK")
