"""An unmodified matplotlib program in place of the document (ctest ariglx_matplotlib_show:
`ariglx snapshot --frames 3 --png ariglx_mpl.png mpl_show.py`): matplotlib's default backend in
ariglx is pycvc_gl's, and a blocking plt.show() loads figures.ari, opens the figure in one of its
windows and runs the host's frames (the snapshot's), then returns. Before that the program steps a
frame of its own, whose hook calls host.run(): the host refuses it, and its loop is left to run.

With --close (ctest ariglx_matplotlib_close: `ariglx snapshot --frames 60 mpl_show.py --close`) a
single-shot timer closes the figure on the second frame: figures.ari, with no window left, closes
the host, so plt.show() returns long before the snapshot's frames run out.

Prints SKIP when this Python has no matplotlib; exits 3 when something is wrong."""

import sys

try:
    import matplotlib
except ImportError:
    print("mpl_show.py: SKIP: no matplotlib")
    sys.exit(0)
import matplotlib.pyplot as plt
import numpy as np

from pycvc_gl.ariadne import current_host

close = "--close" in sys.argv[1:]
backend = matplotlib.get_backend()
x = np.linspace(0.0, 2.0 * np.pi, 200)
fig, ax = plt.subplots()
ax.plot(x, np.sin(x), lw=3, label="sin")
ax.plot(x, np.cos(x), lw=3, label="cos")
ax.set_title("ariglx: matplotlib")
ax.legend()
frames = []
host = current_host()
host.on_frame(frames.append)
refused = []
if not close:  # host.run() from a frame of the program's own is refused, and leaves the loop be
    def run_from_a_frame(dt):
        if not refused:
            try:
                host.run()
                refused.append("it ran")
            except RuntimeError as e:
                refused.append(str(e))

    host.on_frame(run_from_a_frame)
    host.step()
    frames.clear()
if close:  # due in the first frame with any time behind it, the second
    closer = fig.canvas.new_timer(interval=1)
    closer.single_shot = True
    closer.add_callback(plt.close, fig)
    closer.start()
plt.show()  # blocks: the host's loop, here the snapshot's frames

if close:
    ok = (not plt.fignum_exists(fig.number) and host.should_close() and 2 <= len(frames) < 60
          and host.get("figures.count") == "0")
    # The ctest passes on this line: print it only when the rest holds.
    print(f"mpl_show.py: {'' if ok else 'FAILED: '}closed: plt.show() returned after "
          f"{len(frames)} frames, the figure gone, the host closing")
    sys.exit(0 if ok else 3)

image = host.images.get("figure-1")
size = f"{image.shape[1]}x{image.shape[0]}" if image is not None else "none"
ok = (host.document is not None and host.document.endswith("figures.ari")
      and host.get("tree.ari_figure_0.open") == "1" and len(frames) >= 3
      and len(refused) == 1 and "inside one of the host's frames" in refused[0])
# The ctest passes on this line, whatever the exit status: print it only when the rest holds.
print(f"mpl_show.py: {'' if ok else 'FAILED: '}backend {backend}, figures "
      f"{host.get('figures.count')}, image {size}")
if not ok:
    print(f"mpl_show.py: document {host.document}, frames {len(frames)}, window open "
          f"{host.get('tree.ari_figure_0.open')}, host.run() from a frame: {refused}")
sys.exit(0 if ok else 3)
