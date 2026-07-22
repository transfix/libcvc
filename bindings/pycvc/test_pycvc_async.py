"""AsyncStateObserver: state changes (fired on a C++ writer thread) are
dispatched to async handlers on a bounded coroutine pool.

The director's on_changed() runs synchronously on whatever thread writes state;
AsyncStateObserver marshals each path onto the asyncio loop with
call_soon_threadsafe, and a fixed pool of worker coroutines drains a bounded
queue. Here a background (non-loop) thread plays the writer, exactly like a C++
host thread would.
"""

import asyncio
import threading

import pycvc


def _write_from_background_thread(app, paths):
    """Write state from a NON-loop thread (like a C++ writer thread)."""
    done = threading.Event()

    def run():
        for p, v in paths:
            pycvc.state_set(app, p, v)
        done.set()

    threading.Thread(target=run, daemon=True).start()
    return done


def test_async_dispatch_to_pool():
    app = pycvc.make_app()

    async def main():
        got = []

        async def handler(path):
            # Re-read the current value — the point of async handlers.
            got.append((path, pycvc.state_get(app, path)))

        obs = pycvc.AsyncStateObserver(handler, concurrency=3, maxsize=256)
        obs.start(app)

        done = _write_from_background_thread(
            app, [(f"async.k{i}", str(i)) for i in range(5)]
        )
        while not done.is_set():
            await asyncio.sleep(0.01)
        # Let in-flight cross-thread marshaling (on_changed -> call_soon_threadsafe
        # -> _offer) settle onto the queue before draining it.
        await asyncio.sleep(0.1)
        await obs.drain()
        await obs.stop()
        return got

    got = asyncio.run(main())
    seen = {p for p, _ in got}
    for i in range(5):
        assert f"async.k{i}" in seen, (i, seen)
    # handlers re-read state, so values are consistent
    assert dict(got)["async.k3"] == "3"


def test_subclass_override_handle():
    app = pycvc.make_app()

    class Collector(pycvc.AsyncStateObserver):
        def __init__(self):
            super().__init__(concurrency=2, maxsize=64)
            self.seen = []

        async def handle(self, path):
            self.seen.append(path)

    async def main():
        obs = Collector()
        obs.start(app)
        done = _write_from_background_thread(app, [("sub.a", "1"), ("sub.b", "2")])
        while not done.is_set():
            await asyncio.sleep(0.01)
        # Let in-flight cross-thread marshaling (on_changed -> call_soon_threadsafe
        # -> _offer) settle onto the queue before draining it.
        await asyncio.sleep(0.1)
        await obs.drain()
        await obs.stop()
        return obs.seen

    seen = asyncio.run(main())
    assert "sub.a" in seen and "sub.b" in seen, seen


def test_bounded_queue_drops_under_flood_but_survives():
    app = pycvc.make_app()

    async def main():
        processed = []

        async def slow(path):
            await asyncio.sleep(0.001)  # can't keep up with a flood
            processed.append(path)

        # Tiny queue + single worker so a burst overflows and drops.
        obs = pycvc.AsyncStateObserver(slow, concurrency=1, maxsize=4,
                                       overflow="drop_oldest")
        obs.start(app)
        done = _write_from_background_thread(
            app, [(f"flood.k{i}", str(i)) for i in range(200)]
        )
        while not done.is_set():
            await asyncio.sleep(0.01)
        # Let in-flight cross-thread marshaling (on_changed -> call_soon_threadsafe
        # -> _offer) settle onto the queue before draining it.
        await asyncio.sleep(0.1)
        await obs.drain()
        await obs.stop()
        return processed, obs.dropped

    processed, dropped = asyncio.run(main())
    # It must not crash, must process SOME, and must have dropped under overflow.
    assert len(processed) > 0
    assert dropped > 0, "a 200-change flood into a maxsize=4 queue must drop"
    assert len(processed) + dropped >= 200 - 4  # accounting is consistent-ish


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print("  ok:", t.__name__)
    print("pycvc async-pool tests: OK")
