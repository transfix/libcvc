"""hello.py — run hello.ari from Python: a button calls Python, and Python updates the document.

    python hello.py                              # a window; close it or App > Quit
    python hello.py --offscreen --png hello.png  # 30 frames offscreen, the last one saved
    python hello.py --headless                   # no GL: the programs and Python calls only

The counterpart of the C++ host examples/ariadne/hosts/ariadne_hello.cpp. Off a window, nobody can
click, so the script presses both buttons itself (it runs Greet's program and emits Count) and
prints what the document then shows.
"""

import os
import platform
import sys

from pycvc.ari import Host, main_args

HERE = os.path.dirname(os.path.abspath(__file__))


def main(argv=None):
    args = main_args(argv)
    host = Host(mode=args.mode, name="hello", width=args.width, height=args.height)

    # (greet NAME) in the document: the argument comes in as a str, the value goes back.
    @host.function("greet")
    def greet(name):
        return f"Hello, {name}! (from Python {platform.python_version()})"

    # Count's on: is the bare event "count".
    @host.on("count")
    def count():
        host.set("clicks", host.get_int("clicks") + 1)

    # Python feeds the document every frame.
    @host.on_frame
    def tick(dt):
        host.set("python.frame", f"frame {host.frame_count}")

    host.load(os.path.join(HERE, "hello.ari"))  # its warnings are logged

    if args.mode != "window":  # press the buttons ourselves
        host.set("name", "offscreen")
        host.run_program('(state-set "message" (greet (state-get "name")))')
        host.emit("count")  # as the Count button raises it
        host.emit("count")
    host.run(frames=args.frames or None)
    if args.png:
        host.snapshot(args.png)
    print("message:", host.get("message"))
    print("clicks:", host.get("clicks"))
    code = host.exit_code()
    host.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
