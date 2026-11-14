"""matplotlib keeps its own default backend where no GL host has lent a window (ctest
arix_matplotlib_backend: `arix snapshot mpl_backend.py`): arix's finder leaves the backend alone, so
plt.show() is matplotlib's own there, not pycvc_gl.ariadne_mpl's. Prints SKIP when this Python has
no matplotlib."""

import sys

try:
    import matplotlib
except ImportError:
    print("mpl_backend.py: SKIP: no matplotlib")
    sys.exit(0)

backend = matplotlib.get_backend()  # resolves matplotlib's own default (agg with no display)
print(f"mpl_backend.py: backend {backend}")
sys.exit(3 if backend.startswith("module://pycvc_gl") else 0)
