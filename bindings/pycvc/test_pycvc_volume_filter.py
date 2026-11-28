"""pycvc's volume filter jobs (bindings/pycvc/pycvc_volume_filter.i; inc/cvc/volume/volume_filter.h;
docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md V0), the VolumeRover port plan's §1.7 T11:

  - filter_volume and volume_filter_job take volume_filter_params or its fields as keywords, an
    enum by its name, and leave the input alone;
  - each filter equals the legacy in-place method (contrast_enhancement, anisotropic_diffusion,
    gdtv_filter) exactly, as volume_filter_test checks in C++;
  - the job steps with progress and named phases, and a cancel leaves no result;
  - bad parameters raise ValueError, an unknown keyword TypeError;
  - another Python thread runs while a filter does (the GIL is released).
"""

import threading
import time

import numpy as np
import pycvc

app = pycvc.make_app()


def _volume(seed=7, n=(14, 12, 10)):
    nx, ny, nz = n
    rng = np.random.default_rng(seed)
    k, j, i = np.meshgrid(np.arange(nz), np.arange(ny), np.arange(nx), indexing="ij")
    field = np.where(i + j < (nx + ny) // 2, 60.0, 170.0) + 2.0 * k + 1.5 * j
    field = np.clip(field + rng.uniform(-12, 12, size=field.shape), 0, 250)
    vol = pycvc.volume(app)
    vol.set_float_grid(field.ravel(order="C").tolist(), nx, ny, nz, 0, 0, 0, 1, 1, 1)
    return vol


def _grid(vol):
    return np.asarray(vol.grid(), dtype=np.float64).copy()


def test_params_take_keywords_and_enum_names():
    p = pycvc.volume_filter_params(contrast_resistor=0.5, gdtv_iterations=3)
    p.set_text("filter", "gdtv")
    p.set_text("gdtv_neighbours", "twenty_six")  # a dotted field by its keyword
    assert p.get_text("filter") == "gdtv"
    assert p.get_text("gdtv.neighbours") == "twenty_six"
    assert p.contrast_resistor == 0.5 and p.gdtv_iterations == 3
    assert p.validate() == []
    d = pycvc.volume_filter_params().to_dict()
    # VolumeRover2's dialogs' defaults
    assert d["anisotropic_iterations"] == 20 and d["contrast_resistor"] == 0.95
    assert d["gdtv_exponent"] == 1.0 and d["gdtv_lambda"] == 1e-4 and d["gdtv_iterations"] == 1
    assert d["radiometric_sigma"] == 200.0 and d["spatial_sigma"] == 1.5 and d["radius"] == 2
    print("volume_filter_params: keywords, enum names, VolumeRover2's defaults")


def test_each_filter_equals_the_legacy_method():
    vol = _volume()
    before = _grid(vol)
    cases = [
        (dict(filter="contrast", contrast_resistor=0.9),
         lambda v: v.contrast_enhancement(0.9)),
        (dict(filter="anisotropic", anisotropic_iterations=3),
         lambda v: v.anisotropic_diffusion(3)),
        # VolumeRover2's dialog: gdtv_filter(2 - exponent, lambda, iterations, nhood - 1)
        (dict(filter="gdtv", gdtv_exponent=1.0, gdtv_lambda=0.01, gdtv_iterations=2),
         lambda v: v.gdtv_filter(1.0, 0.01, 2, 0)),
        (dict(filter="gdtv", gdtv_exponent=1.5, gdtv_lambda=0.01, gdtv_neighbours="twenty_six"),
         lambda v: v.gdtv_filter(0.5, 0.01, 1, 1)),
    ]
    for kw, legacy in cases:
        out = pycvc.filter_volume(vol, **kw)
        ref = _volume()
        legacy(ref)
        assert np.array_equal(_grid(out), _grid(ref)), kw
        assert np.array_equal(_grid(vol), before), "the input is never written"
    print("filter_volume: contrast, anisotropic and gdtv equal the legacy methods exactly")


def test_the_job_steps_and_cancels():
    vol = _volume()
    job = pycvc.volume_filter_job(vol, filter="contrast")
    phases, last = set(), 0.0
    while not job.step(64):
        phases.add(job.phase())
        assert job.progress() >= last
        last = job.progress()
    assert {"contrast: mapping", "contrast: up", "contrast: down"} <= phases, phases
    out = job.result()
    assert np.array_equal(_grid(out), _grid(pycvc.filter_volume(vol, filter="contrast")))

    p = pycvc.volume_filter_params(anisotropic_iterations=500)
    p.set_text("filter", "anisotropic")
    job = pycvc.volume_filter_job(vol, p)
    job.step(64)
    job.cancel()
    assert job.step(64)
    assert job.cancelled()
    try:
        job.result()
    except RuntimeError:
        pass
    else:
        raise AssertionError("a cancelled job has no result")
    print("volume_filter_job: phases, rising progress, the same result; a cancel leaves none")


def test_bad_parameters_raise():
    vol = _volume()
    for kw in (dict(filter="contrast", contrast_resistor=2.0),
               dict(filter="gdtv", gdtv_lambda=0.0),
               dict(filter="anisotropic", anisotropic_iterations=0)):
        try:
            pycvc.filter_volume(vol, **kw)
        except ValueError as e:
            assert any(k.split("_")[0] in str(e) for k in kw if k != "filter"), str(e)
        else:
            raise AssertionError("no ValueError for %r" % kw)
    for bad in (dict(filter="sharpen"),):
        try:
            pycvc.filter_volume(vol, **bad)
        except ValueError:
            pass
        else:
            raise AssertionError("no ValueError for an unknown filter")
    try:
        pycvc.filter_volume(vol, gdtv_exponant=1.0)
    except TypeError as e:
        assert "gdtv_exponent" in str(e), str(e)
    else:
        raise AssertionError("no TypeError for an unknown keyword")
    print("filter_volume: bad parameters raise ValueError, an unknown keyword TypeError")


def test_another_thread_runs_meanwhile():
    vol = _volume(n=(40, 40, 40))
    ticks = []
    stop = threading.Event()

    def ticker():
        while not stop.is_set():
            ticks.append(time.perf_counter())
            time.sleep(0.001)

    t = threading.Thread(target=ticker)
    t.start()
    t0 = time.perf_counter()
    pycvc.filter_volume(vol, filter="gdtv", gdtv_neighbours="twenty_six", gdtv_iterations=6)
    t1 = time.perf_counter()
    stop.set()
    t.join()
    during = [x for x in ticks if t0 <= x <= t1]
    assert len(during) >= 2 or t1 - t0 < 0.01, (len(during), t1 - t0)
    print("filter_volume: %d ticks of another thread during %.0f ms" % (len(during),
                                                                       1e3 * (t1 - t0)))


if __name__ == "__main__":
    print("=== pycvc volume filter job tests ===")
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("pycvc volume filter job tests: OK")
