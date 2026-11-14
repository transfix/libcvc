"""ariglx-web's browser smoke: a Python program in place of the document (?src=program.py, with
program.ari in the page config's preload). Host() adopts the page's host, gives program.ari a
function and loads it. In a browser host.run() returns at once and the page runs the frames after
the program returns, so the program reports from a frame hook."""

import os
import sys

from pycvc.ari import Host, current_host

host = Host()
assert host.embedded and current_host() is host, "Host() adopts the scripting host"


@host.function("twice")
def twice(x):
    return 2 * x


frames = []


def frame(dt):
    frames.append(dt)
    if len(frames) == 3:
        print(f"program.py: result {host.get('result')}, frames {len(frames)}", file=sys.stderr)


host.on_frame(frame)
host.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "program.ari"))
print(f"program.py: run() returned {host.run()}", file=sys.stderr)
