"""A Python program in place of the document (`arix snapshot app.py`, `ariglx snapshot --png out.png
app.py`): Host() adopts the host, gives app.ari a function, loads it and runs the host's frames.
Exits 0 when the document saw the function's value and the frames ran the program's hook."""

import os
import sys

from pycvc_gl.ariadne import Host, current_host

host = Host()
assert host.embedded and current_host() is host, "Host() adopts the scripting host"


@host.function("twice")
def twice(x):
    return 2 * x


frames = []
host.on_frame(frames.append)
host.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "app.ari"))
code = host.run()
result = host.get("result")
print(f"app.py: result {result}, frames {len(frames)}, exit {code}, args {sys.argv[1:]}")
sys.exit(0 if result == "14" and frames and code == 0 else 3)
