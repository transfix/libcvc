"""An unmodified matplotlib program in place of the document (ctest ariglx_matplotlib_show:
`ariglx snapshot --frames 3 --png ariglx_mpl.png mpl_show.py`): matplotlib's default backend in
ariglx is pycvc_gl's, and a blocking plt.show() loads figures.ari, opens the figure in one of its
windows and runs the host's frames (the snapshot's), then returns. Prints SKIP when this Python has
no matplotlib; exits 3 when something is wrong."""

import sys

try:
    import matplotlib
except ImportError:
    print("mpl_show.py: SKIP: no matplotlib")
    sys.exit(0)
import matplotlib.pyplot as plt
import numpy as np

from pycvc_gl.ariadne import current_host

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
plt.show()  # blocks: the host's loop, here the snapshot's frames

image = host.images.get("figure-1")
size = f"{image.shape[1]}x{image.shape[0]}" if image is not None else "none"
ok = (host.document is not None and host.document.endswith("figures.ari")
      and host.get("tree.figure_0.open") == "1" and len(frames) >= 3)
# The ctest passes on this line, whatever the exit status: print it only when the rest holds.
print(f"mpl_show.py: {'' if ok else 'FAILED: '}backend {backend}, figures "
      f"{host.get('figures.count')}, image {size}")
if not ok:
    print(f"mpl_show.py: document {host.document}, frames {len(frames)}, window open "
          f"{host.get('tree.figure_0.open')}")
sys.exit(0 if ok else 3)
