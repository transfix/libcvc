"""Robust tests for pycvc's memory-ownership model.

These prove the *zero-copy* + *shared_ptr-lifetime* contract of the SWIG
bindings, independent of any threading:

  * Identity / no-copy — `Geometry.vertices()` / `Volume.grid()` return numpy
    arrays that VIEW the C++ buffer. Two calls alias the same storage; a write
    through one array is visible through the other and via the scalar
    accessors (`value(i,j,k)`).
  * Layout — exact dtype / shape / C-contiguity / writability of every view.
  * Lifetime — a view (or a slice/reshape of it) stays valid and correct after
    the facade object is `del`'d and garbage-collected, because its `base` is a
    PyCapsule owning a `std::shared_ptr` to the C++ storage.
  * Refcount sanity (CPython) — the owning capsule is referenced by exactly the
    arrays that view it; deleting an array releases exactly one reference.
  * CUDA unified memory (only exercised when `cuda_available()`): one
    `cudaMallocManaged` buffer backs BOTH the host numpy view and the device
    pointer, so `grid()`'s host address == `cuda_ptr()` == the
    `__cuda_array_interface__` data pointer. `enable_cuda`/`disable_cuda`
    *migrate* (reallocate) the buffer and free the previous block, so callers
    should re-fetch after each transition to stay coherent with the live data.
  * Fail-safe stale views — `grid()` pins the EXACT block it views (not just
    the volume facade), so a view taken before a migration (or before a
    `set_float_grid()` that reallocates) never dangles: dereferencing it is a
    valid, decoupled *snapshot* of the pre-migration data, never a segfault.
    Re-fetching is still required for coherence with the live buffer, but a
    missed re-fetch fails SAFE instead of faulting.

Style matches the sibling test files: plain asserts + prints, run under
`__main__`; any failure raises and exits nonzero.
"""

import gc
import sys

import numpy as np
import pycvc


def _ptr(a):
    """Data pointer of a numpy array (as an int)."""
    return a.__array_interface__["data"][0]


# ── Zero-copy identity / no-copy ────────────────────────────────────


def test_grid_two_calls_alias_same_buffer():
    nx, ny, nz = 4, 3, 2
    vol = pycvc.Volume()
    vol.set_float_grid([float(i) for i in range(nx * ny * nz)], nx, ny, nz, 0, 0, 0, 1, 1, 1)
    a = vol.grid()
    b = vol.grid()
    # No copy: two independent views over the SAME storage.
    assert _ptr(a) == _ptr(b), "grid() must not copy — both calls alias one buffer"
    a[1, 2, 3] = -5.0
    assert b[1, 2, 3] == -5.0, "write through one view must show in the other"
    assert vol.value(3, 2, 1) == -5.0, "and via value(i,j,k) (grid[k,j,i] == value(i,j,k))"
    print("  ok: grid() two calls alias the same buffer (no copy)")


def test_vertices_two_calls_alias_same_buffer():
    g = pycvc.Geometry()
    g.add_vertices([float(i) for i in range(30)])  # 10 verts
    a = g.vertices()
    b = g.vertices()
    assert _ptr(a) == _ptr(b), "vertices() must not copy — both calls alias one buffer"
    a[4, 1] = 123.5
    assert b[4, 1] == 123.5
    # C++ -> numpy: a bulk build is visible in a freshly fetched view.
    assert g.vertices()[0, 0] == 0.0 and g.vertices()[9, 2] == 29.0
    print("  ok: vertices() two calls alias the same buffer (no copy)")


def test_grid_mutation_is_bidirectional_with_scalar_accessor():
    vol = pycvc.Volume()
    vol.set_float_grid([0.0] * 24, 2, 3, 4, 0, 0, 0, 1, 1, 1)
    grid = vol.grid()
    # numpy -> C++
    grid[3, 2, 1] = 42.0
    assert vol.value(1, 2, 3) == 42.0
    # C++ -> numpy: rebuild the field, existing-alias fresh fetch reflects it
    vol.set_float_grid([7.0] * 24, 2, 3, 4, 0, 0, 0, 1, 1, 1)
    assert vol.grid()[0, 0, 0] == 7.0
    print("  ok: grid <-> value(i,j,k) mutation is bidirectional")


# ── Layout: dtype / shape / contiguity / writability ────────────────


def test_grid_layout_is_nz_ny_nx_float32_c_contig_writable():
    nx, ny, nz = 5, 4, 3
    vol = pycvc.Volume()
    vol.set_float_grid([1.0] * (nx * ny * nz), nx, ny, nz, 0, 0, 0, 1, 1, 1)
    grid = vol.grid()
    assert grid.shape == (nz, ny, nx), "grid shape must be (nz, ny, nx)"
    assert grid.dtype == np.float32
    assert grid.flags["C_CONTIGUOUS"] is True
    assert grid.flags["WRITEABLE"] is True
    assert grid.flags["OWNDATA"] is False, "view must not own its data"
    assert grid.base is not None and type(grid.base).__name__ == "PyCapsule"
    print("  ok: grid() layout is (nz,ny,nx) float32 C-contiguous writable view")


def test_vertices_layout_is_n3_float64_c_contig():
    g = pycvc.Geometry()
    g.add_vertices([float(i) for i in range(12)])  # 4 verts
    v = g.vertices()
    assert v.shape == (4, 3)
    assert v.dtype == np.float64
    assert v.flags["C_CONTIGUOUS"] is True
    assert v.flags["WRITEABLE"] is True
    assert v.flags["OWNDATA"] is False
    assert type(v.base).__name__ == "PyCapsule"
    print("  ok: vertices() layout is (N,3) float64 C-contiguous view")


def test_colors_layout_and_values():
    g = pycvc.Geometry()
    g.add_vertices([0, 0, 0, 1, 1, 1])
    g.set_colors([1, 0, 0, 0, 1, 0])
    c = g.vertex_colors()
    assert c.shape == (2, 3) and c.dtype == np.float64
    assert c.flags["C_CONTIGUOUS"] and type(c.base).__name__ == "PyCapsule"
    assert np.allclose(c[0], [1, 0, 0]) and np.allclose(c[1], [0, 1, 0])
    print("  ok: vertex_colors() layout (N,3) float64 with correct values")


def test_empty_views_are_safe_and_shaped():
    g = pycvc.Geometry()
    assert g.vertices().shape == (0, 3) and g.vertices().size == 0
    assert g.vertex_colors().shape == (0, 3)
    print("  ok: empty geometry views are (0,3) and safe")


# ── Lifetime: the shared_ptr in the capsule base ────────────────────


def test_volume_view_outlives_facade_del_gc():
    vol = pycvc.Volume()
    vol.set_float_grid([float(i) for i in range(8)], 2, 2, 2, 0, 0, 0, 1, 1, 1)
    grid = vol.grid()
    del vol
    gc.collect()  # facade gone; capsule's shared_ptr keeps the storage alive
    assert grid.shape == (2, 2, 2)
    assert float(grid.sum()) == float(sum(range(8)))
    grid[0, 0, 0] = 99.0  # still writable into the (still-owned) C++ buffer
    assert grid[0, 0, 0] == 99.0
    print("  ok: grid() view stays valid + writable after facade del + gc")


def test_vertices_view_outlives_facade_del_gc():
    g = pycvc.Geometry()
    g.add_vertices([9, 8, 7, 6, 5, 4])
    v = g.vertices()
    del g
    gc.collect()
    assert v.shape == (2, 3) and float(v.sum()) == 39.0
    print("  ok: vertices() view stays valid after facade del + gc")


def test_slice_and_reshape_keep_base_alive_after_facade_gone():
    g = pycvc.Geometry()
    g.add_vertices([float(i) for i in range(30)])  # 10 verts
    v = g.vertices()
    s = v[2:5]  # slice: base chains up to the parent array
    r = v.reshape(-1)  # reshape: another view sharing the capsule
    assert s.base is v and type(v.base).__name__ == "PyCapsule"
    del g, v
    gc.collect()  # only the derived views remain; capsule must still be alive
    assert s.shape == (3, 3)
    assert float(s.sum()) == float(sum(range(6, 15)))
    assert float(r.sum()) == float(sum(range(30)))
    print("  ok: slice/reshape keep the owning capsule alive after facade gone")


def test_many_views_then_drop_source_all_valid_and_shared():
    g = pycvc.Geometry()
    g.add_vertices(list(range(3 * 100)))  # 100 verts
    views = [g.vertices() for _ in range(50)]
    del g
    gc.collect()
    for v in views:
        assert v.shape == (100, 3)
    # All views alias the one buffer, so a write through any is seen by all.
    views[0][0, 0] = -1.0
    assert all(v[0, 0] == -1.0 for v in views)
    print("  ok: 50 views survive source drop and all share one buffer")


def test_dropping_all_views_is_safe():
    # Best-effort: the facade owns the data; dropping every view must not free
    # anything the facade still uses, and dropping the last view must not crash.
    g = pycvc.Geometry()
    g.add_vertices([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    v = g.vertices()
    del v
    gc.collect()  # no view left; facade still owns its data
    assert g.vertices().shape == (2, 3)  # facade still fully usable
    assert g.vertices()[1, 2] == 6.0
    # Reverse: keep only the capsule (base) after the array is gone — no crash.
    a = g.vertices()
    cap = a.base
    del a
    gc.collect()
    assert type(cap).__name__ == "PyCapsule"  # still a live capsule, no fault
    print("  ok: dropping views / holding a bare capsule is crash-free")


# ── Fail-safe stale views: buffer swapped out from under a view ──────
# grid() pins the specific voxel block it aliases, so a view taken BEFORE the
# buffer is reallocated (set_float_grid rebuild, or a CUDA migration) stays
# valid — a decoupled snapshot — instead of dangling into freed memory.


def test_grid_view_survives_host_buffer_reallocation():
    # set_float_grid() rebuilds the volume, replacing (and freeing) the old
    # voxel buffer. A grid() view captured beforehand must not fault on access.
    nx, ny, nz = 4, 3, 2
    vol = pycvc.Volume()
    vol.set_float_grid([float(i) for i in range(nx * ny * nz)], nx, ny, nz, 0, 0, 0, 1, 1, 1)
    stale = vol.grid()  # view over the ORIGINAL block
    old_ptr = _ptr(stale)
    snapshot = np.array(stale, copy=True)  # what the old block held

    # Rebuild: a fresh, differently-valued grid. Under the old (facade-owning)
    # model this frees the block `stale` points at -> use-after-free.
    vol.set_float_grid([100.0 + i for i in range(nx * ny * nz)], nx, ny, nz, 0, 0, 0, 1, 1, 1)
    fresh = vol.grid()
    assert _ptr(fresh) != old_ptr, "rebuild must reallocate (else nothing to prove)"

    # The pinned view is still valid memory: a decoupled snapshot, not a fault.
    assert np.array_equal(stale, snapshot), "stale view must hold its pre-rebuild snapshot"
    assert np.array_equal(fresh, np.arange(100.0, 100.0 + nx * ny * nz).reshape(nz, ny, nx))
    stale[0, 0, 0] = -7.0  # still safely writable into the old (orphaned) block
    assert stale[0, 0, 0] == -7.0
    assert fresh[0, 0, 0] == 100.0, "writing the stale snapshot must not touch the live buffer"

    del vol, fresh
    gc.collect()  # facade gone; the pinned old block keeps the stale view alive
    assert stale[1, 2, 3] == snapshot[1, 2, 3]
    print("  ok: grid() view stays valid (snapshot) after host buffer realloc")


# ── Refcount sanity (CPython-specific; robust, not over-fit) ─────────


def test_capsule_refcount_drops_by_one_when_array_deleted():
    if sys.implementation.name != "cpython":
        print("  skip: refcount test (non-CPython interpreter)")
        return
    g = pycvc.Geometry()
    g.add_vertices([float(i) for i in range(30)])
    a = g.vertices()
    cap = a.base
    r0 = sys.getrefcount(cap)
    del a  # the array held exactly one reference to its owning capsule
    r1 = sys.getrefcount(cap)
    assert r1 == r0 - 1, (r0, r1)
    print("  ok: deleting a view drops its capsule refcount by exactly one")


def test_each_call_mints_distinct_capsule_over_same_data():
    g = pycvc.Geometry()
    g.add_vertices([float(i) for i in range(30)])
    a = g.vertices()
    b = g.vertices()
    assert a.base is not b.base, "each view call mints its own owner capsule"
    assert _ptr(a) == _ptr(b), "...but both capsules guard the same C++ buffer"
    print("  ok: each view call mints a distinct capsule over the same buffer")


# ── CUDA unified memory (guarded; runs iff cuda_available()) ─────────


def test_cuda_unified_pointer_identity():
    if not pycvc.Volume.cuda_available():
        print("  skip: CUDA not available")
        return
    nx, ny, nz = 4, 4, 4
    vol = pycvc.Volume()
    vol.set_float_grid([float(i) for i in range(nx * ny * nz)], nx, ny, nz, 0, 0, 0, 1, 1, 1)
    assert vol.on_gpu() is False and vol.cuda_ptr() == 0
    vol.enable_cuda()
    try:
        assert vol.using_cuda() and vol.on_gpu()
        assert vol.cuda_ptr() != 0
        grid = vol.grid()  # fresh view AFTER the migration
        # Unified memory: the host numpy address IS the device pointer.
        assert _ptr(grid) == vol.cuda_ptr(), "grid() host ptr must equal cuda_ptr()"
        cai = vol.__cuda_array_interface__
        assert cai["version"] == 3
        assert cai["shape"] == (nz, ny, nx)
        assert cai["typestr"] == "<f4"
        assert cai["data"][0] == vol.cuda_ptr(), "CAI data ptr must equal cuda_ptr()"
        assert cai["data"][1] is False  # not read-only
        assert cai["strides"] is None  # C-contiguous
    finally:
        vol.disable_cuda()
    print("  ok: host grid() ptr == cuda_ptr() == CAI data ptr (one unified buffer)")


def test_cuda_host_device_coherence_and_migration_preserves_data():
    if not pycvc.Volume.cuda_available():
        print("  skip: CUDA not available")
        return
    nx, ny, nz = 4, 4, 4
    vol = pycvc.Volume()
    vol.set_float_grid([float(i) for i in range(nx * ny * nz)], nx, ny, nz, 0, 0, 0, 1, 1, 1)
    host_ptr_before = _ptr(vol.grid())

    vol.enable_cuda()
    # enable_cuda migrates (reallocates): the new buffer is a different address.
    assert vol.cuda_ptr() != host_ptr_before, "migration reallocates the buffer"
    grid = vol.grid()  # MUST re-fetch after the transition
    # Host write through the unified buffer is coherent with value().
    grid[1, 1, 1] = 777.0
    assert vol.value(1, 1, 1) == 777.0, "host write must be visible via value()"
    del grid  # drop the view before the next migration (never deref it stale)

    vol.disable_cuda()
    # Data survives the round trip back to host.
    assert vol.on_gpu() is False and vol.using_cuda() is False
    assert vol.value(1, 1, 1) == 777.0, "data preserved across disable_cuda()"
    fresh = vol.grid()  # re-fetch again after migrating back
    assert fresh[1, 1, 1] == 777.0
    assert fresh[0, 0, 0] == 0.0
    print("  ok: unified host/device coherence + enable/disable preserves data")


def test_cuda_array_interface_present_only_when_on_gpu():
    if not pycvc.Volume.cuda_available():
        print("  skip: CUDA not available")
        return
    vol = pycvc.Volume()
    vol.set_float_grid([1.0] * 8, 2, 2, 2, 0, 0, 0, 1, 1, 1)
    # Host-resident: no device interface (correct signal for cupy/torch).
    try:
        _ = vol.__cuda_array_interface__
    except AttributeError:
        pass
    else:
        raise AssertionError("host-resident volume must not expose CAI")
    vol.enable_cuda()
    try:
        cai = vol.__cuda_array_interface__  # present once on-device
        assert cai["data"][0] == vol.cuda_ptr()
    finally:
        vol.disable_cuda()
    # Absent again after migrating back to host.
    try:
        _ = vol.__cuda_array_interface__
    except AttributeError:
        pass
    else:
        raise AssertionError("CAI must disappear after disable_cuda()")
    print("  ok: __cuda_array_interface__ present iff on_gpu(), absent on host")


def test_cuda_pre_migration_device_view_stays_valid_after_disable():
    # THE migration hazard: a grid() taken while GPU-resident points at the CUDA
    # unified block, which disable_cuda() frees. The view pins that exact block,
    # so dereferencing it afterward is a valid snapshot, never a use-after-free.
    if not pycvc.Volume.cuda_available():
        print("  skip: CUDA not available")
        return
    nx, ny, nz = 4, 4, 4
    vol = pycvc.Volume()
    vol.set_float_grid([float(i) for i in range(nx * ny * nz)], nx, ny, nz, 0, 0, 0, 1, 1, 1)
    vol.enable_cuda()
    dev_view = vol.grid()  # view over the CUDA unified block
    dev_view[1, 1, 1] = 555.0
    assert vol.value(1, 1, 1) == 555.0
    snapshot = np.array(dev_view, copy=True)

    vol.disable_cuda()  # migrates back to host and FREES the unified block
    # Old model: dev_view now dangles into freed device memory -> segfault.
    # Pinned model: the block is still owned by the view -> valid snapshot.
    assert np.array_equal(dev_view, snapshot), "device view must survive disable_cuda()"

    # Decoupled: mutating the live (host) buffer must not touch the snapshot.
    live = vol.grid()
    live[1, 1, 1] = 999.0
    assert dev_view[1, 1, 1] == 555.0, "stale device view is a decoupled snapshot"
    assert vol.value(1, 1, 1) == 999.0

    del vol, live
    gc.collect()  # facade gone; pinned unified block keeps the stale view alive
    assert dev_view[1, 1, 1] == 555.0
    print("  ok: pre-migration device view survives disable_cuda() (no fault)")


def test_cuda_pre_migration_host_view_stays_valid_after_enable():
    # The reverse transition: a host view taken before enable_cuda() must also
    # stay valid (a snapshot) once the volume migrates onto the GPU.
    if not pycvc.Volume.cuda_available():
        print("  skip: CUDA not available")
        return
    nx, ny, nz = 4, 4, 4
    vol = pycvc.Volume()
    vol.set_float_grid([float(i) for i in range(nx * ny * nz)], nx, ny, nz, 0, 0, 0, 1, 1, 1)
    host_view = vol.grid()  # view over the host block
    snapshot = np.array(host_view, copy=True)

    vol.enable_cuda()  # migrates to CUDA unified memory
    try:
        assert np.array_equal(host_view, snapshot), "host view must survive enable_cuda()"
        # Live data now lives on the GPU block; the host view is decoupled.
        dev = vol.grid()
        dev[2, 2, 2] = 321.0
        assert host_view[2, 2, 2] == snapshot[2, 2, 2], "host view is a decoupled snapshot"
        assert vol.value(2, 2, 2) == 321.0
    finally:
        vol.disable_cuda()
    assert np.array_equal(host_view, snapshot)
    print("  ok: pre-migration host view survives enable_cuda() (no fault)")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
    print("pycvc memory-ownership tests: OK")
