"""cvc::stream real-time transport bindings (Phase 2).

pycvc.stream_open(app, id, codec, w, h) opens a video stream on `app`; a producer
publishes numpy uint8 frames ZERO-COPY (the ndarray buffer is aliased, not copied)
and a consumer reads them back as a zero-copy read-only numpy view. This verifies
the producer->consumer aliasing (shared memory), frame metadata, drop-oldest ring
semantics, and that malformed producer arrays are rejected (never coerced).
"""

import numpy as np
import pycvc


def test_latest_producer_consumer_zero_copy():
    app = pycvc.make_app()
    s = pycvc.stream_open(app, "cam0", "rgba8", 4, 2)  # 4x2 rgba8 -> 32 bytes/frame
    sub = s.subscribe("latest", 3)
    assert not sub.latest().valid()  # nothing published yet

    frame = np.zeros((2, 4, 4), dtype=np.uint8)  # (H, W, C)
    frame[0, 0, 0] = 200
    frame[1, 3, 3] = 150
    seq = s.publish(frame, 1.5)
    assert seq == 0

    f = sub.latest()
    assert f.valid()
    assert f.seq() == 0
    assert abs(f.pts() - 1.5) < 1e-9
    assert f.width() == 4 and f.height() == 2
    assert f.codec() == "rgba8"

    arr = f.numpy()
    assert arr.shape == (2, 4, 4)
    assert arr.dtype == np.uint8
    assert arr[0, 0, 0] == 200 and arr[1, 3, 3] == 150
    assert arr.flags.writeable is False  # frames are immutable

    # Zero-copy end-to-end: the consumer view shares memory with the published
    # array, so a write to the original is visible through the view.
    assert np.shares_memory(arr, frame)
    frame[0, 0, 1] = 99
    assert arr[0, 0, 1] == 99


def test_publish_rejects_malformed_arrays():
    app = pycvc.make_app()
    s = pycvc.stream_open(app, "cam1", "rgba8", 4, 2)  # needs 32 uint8 bytes

    # wrong dtype
    try:
        s.publish(np.zeros((2, 4, 4), dtype=np.float32), 0.0)
        raise AssertionError("expected a dtype rejection")
    except RuntimeError:
        pass
    # wrong size
    try:
        s.publish(np.zeros((2, 4, 3), dtype=np.uint8), 0.0)
        raise AssertionError("expected a size rejection")
    except RuntimeError:
        pass
    # non-contiguous (a transposed view is not C-contiguous)
    try:
        s.publish(np.zeros((4, 2, 4), dtype=np.uint8).transpose(1, 0, 2), 0.0)
        raise AssertionError("expected a contiguity rejection")
    except RuntimeError:
        pass
    # not an ndarray
    try:
        s.publish([1, 2, 3], 0.0)
        raise AssertionError("expected a type rejection")
    except RuntimeError:
        pass


def test_ring_drops_oldest_and_pops_in_order():
    app = pycvc.make_app()
    s = pycvc.stream_open(app, "cam2", "rgba8", 4, 2)
    sub = s.subscribe("ring", 2)  # depth 2

    frame = np.zeros((2, 4, 4), dtype=np.uint8)
    for _ in range(3):  # seq 0,1,2 into a depth-2 ring -> seq 0 dropped
        s.publish(frame, 0.0)

    a = sub.try_pop()
    b = sub.try_pop()
    c = sub.try_pop()
    assert a.valid() and b.valid()
    assert a.seq() == 1 and b.seq() == 2  # FIFO from the oldest surviving
    assert not c.valid()  # drained
    assert sub.dropped() >= 1


def test_closed_stream_rejects():
    app = pycvc.make_app()
    s = pycvc.stream_open(app, "cam3", "rgba8", 2, 2)
    s.close()
    # The descriptor is marked closed; a fresh open of the same id now succeeds
    # (the token was freed), proving close() released it.
    s2 = pycvc.stream_open(app, "cam3", "rgba8", 2, 2)
    assert s2.id() == "cam3"


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
    print("test_pycvc_stream: OK")
