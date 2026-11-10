"""A program with no ASGI app (ctest arisrvx_program, and publish-cvcpkg.yml's smoke test of the
arisrvx package): it writes the relay's tree, waits until the relay has applied the write, and stops
the host, which stops the relay with status 0. It needs nothing beyond arisrvx's own Python."""

import asyncio

from pycvc.ariadne import Host

host = Host()


async def main():
    host.set_tree("job", {"state": "done", "steps": [1, 2, 3]})
    # The relay writes the subtree leaf by leaf on its own thread, in the value's order, so the last
    # leaf says the whole tree is there (job.state alone can be read while steps is half written).
    while host.get("job.steps.2") != "3":
        await asyncio.sleep(0.01)
    print(f"program: the relay applied {host.get_tree('job')}", flush=True)
    host.stop(0)


host.loop.create_task(main())
host.run()
