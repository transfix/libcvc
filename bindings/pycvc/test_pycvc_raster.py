"""A raster as a stream (cvc/ariadne/stream/raster.h): pycvc.raster_open paints an RGBA8 raster and
publishes the rectangles that changed as "rgba8.patch" frames, keeping them in state at its token
(and at a mirror path) for a consumer that subscribes late or a relay's client.
"""

import math

import numpy as np
import pycvc


def test_paint_publish_and_subscribe():
    app = pycvc.make_app()
    r = pycvc.raster_open(app, "tint", 32, 16, 0, "fog.tint")  # key_every, mirror
    assert r.token() == "streams.tint"
    assert r.mirror() == "fog.tint"
    assert r.width() == 32 and r.height() == 16
    sub = r.subscribe("ring", 8)
    assert r.publish() == 0  # nothing painted

    r.fill(0, 0, 32, 16, 255, 0, 0)
    assert r.publish() == 1  # the first frame: a key frame, the whole raster
    whole = r.bytes_sent()
    px = r.pixels()
    assert px.shape == (16, 32, 4) and px.dtype == np.uint8
    assert tuple(px[3, 5]) == (255, 0, 0, 255)

    # Written directly through the view, then marked: only that rectangle goes.
    px[2:4, 2:6] = (0, 0, 255, 255)
    r.mark(2, 2, 4, 2)
    assert r.publish() == 2
    assert (r.bytes_sent() - whole) * 10 < whole

    # A 2-D grid through a colour map; NaN is transparent.
    r.paint_grid(np.array([[0.0, 1.0], [math.nan, 0.5]]), 10, 10, "gray", 0.0, 1.0, 200)
    assert r.publish() == 3
    assert tuple(px[10, 11, :]) == (255, 255, 255, 200)
    assert px[11, 10, 3] == 0

    frames = []
    while True:
        f = sub.try_pop()
        if not f.valid():
            break
        frames.append(f)
    assert [f.codec() for f in frames] == ["rgba8.patch"] * 3
    assert frames[0].width() == 32 and frames[0].height() == 16


def test_bad_arguments_raise():
    app = pycvc.make_app()
    try:
        pycvc.raster_open(app, "big", 0, 4)
        raise AssertionError("a raster of width 0 opened")
    except RuntimeError as e:
        assert "pixels a side" in str(e)
    r = pycvc.raster_open(app, "t2", 4, 4)
    try:
        r.paint_grid(np.zeros((2, 2)), 0, 0, "no-such-map")
        raise AssertionError("an unknown colour map was taken")
    except RuntimeError as e:  # pycvc raises a C++ std::exception as RuntimeError
        assert "unknown colour map" in str(e)
    r.close()
    try:
        r.publish()
        raise AssertionError("a closed raster published")
    except RuntimeError:
        pass


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
    print("test_pycvc_raster: OK")
