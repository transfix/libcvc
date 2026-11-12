"""matplotlib in an Ariadne host: figures as Ariadne windows (docs/ARIADNE.md §9.6).

``module://pycvc_gl.ariadne_mpl`` is a matplotlib backend. ariglx makes it matplotlib's default
when it has lent a GL window (a program in place of the document, a document's ``python:`` block),
unless ``MPLBACKEND`` or a matplotlibrc names another; anywhere else select it with
``matplotlib.use("module://pycvc_gl.ariadne_mpl")``.

* **The canvas** is matplotlib's Agg one. Each draw publishes its pixels (``buffer_rgba()``,
  copied, since Agg reuses its buffer) with ``Host.publish_image``, as the picture
  ``figure-<num>``: Python and the window share a process, so nothing is PNG-encoded.
* **plt.show()** shows each figure in a window: a program with no document of its own gets
  ``figures.ari`` (beside this file), one window per figure. A blocking ``plt.show()`` runs the
  host's loop until those windows are closed, as matplotlib's GUI backends do; one called during a
  load, from a function the document calls, or from a frame neither blocks nor loads a document.
  Closing a figure's window closes the figure.
* **bind(fig, "name")** publishes a figure under a name that a document's own ``image:`` widget
  shows (``src: name``), with no window of its own.
* **draw_idle()** draws once, in the host's next frame, however often it was asked, and timers
  (``fig.canvas.new_timer()``, so ``FuncAnimation``) tick on the host's frames.

The windows and their state are pycvc_gl.ariadne_display's ``Figures``, which display() shares.
Which host: the one ``attach(host)`` named (a plain ``python`` program with a Host of its own),
else the scripting host's. With neither the canvas is plain Agg, and ``plt.show()`` warns. The
mouse, the wheel, keys and NavigationToolbar2's pan and zoom come next (roadmap C2).
"""

from __future__ import annotations

import io
import warnings
import weakref

import matplotlib
import numpy as np
from matplotlib._pylab_helpers import Gcf
from matplotlib.backend_bases import CloseEvent, FigureManagerBase, TimerBase, _Backend
from matplotlib.backends.backend_agg import FigureCanvasAgg

from .ariadne_display import attach, figures_of, host_for, loading, program_level

__all__ = ["BACKEND", "FigureCanvasAriadne", "FigureManagerAriadne", "TimerAriadne", "attach",
           "bind", "figure_rgba"]

BACKEND = "module://pycvc_gl.ariadne_mpl"
NO_HOST = ("pycvc_gl.ariadne_mpl: no Ariadne host; run under ariglx or "
           "ariadne_mpl.attach(host)")


class FigureCanvasAriadne(FigureCanvasAgg):
    """matplotlib's Agg canvas, whose draws go to an Ariadne host as a picture."""

    def __init__(self, figure=None):
        super().__init__(figure)
        self._ari_draws = 0  # the draws published
        self._ari_bound = None  # the picture's name, when bind() gave one
        self._ari_host = None  # a weak reference to the host bind() named

    @property
    def _ari_name(self):
        """The name the figure is published under: bind()'s, else ``figure-<num>``."""
        if self._ari_bound:
            return self._ari_bound
        num = getattr(self.manager, "num", None)
        return f"figure-{num}" if num is not None else f"figure-{id(self.figure):x}"

    def draw(self):
        super().draw()
        self._publish()

    def blit(self, bbox=None):
        self._publish()  # restore_region and draw_artist drew into the buffer: send it

    def _publish(self):
        host = host_for(self)
        if host is None:
            return
        self._ari_draws += 1
        host.publish_image(self._ari_name, np.array(self.buffer_rgba()))  # a copy: Agg reuses it

    def draw_idle(self, *args, **kwargs):
        """Draw once in the host's next frame (plain Agg's draw with no host). As a GUI backend's,
        it asks for a draw even inside ``_idle_draw_cntx``, where pyplot calls it; the frame draws
        inside one, so what changes during that draw asks for no other."""
        host = host_for(self)
        if host is None:
            super().draw_idle(*args, **kwargs)
            return
        figures_of(host).dirty[self] = None

    def new_timer(self, interval=None, callbacks=None):
        return TimerAriadne(self, interval=interval, callbacks=callbacks)

    def flush_events(self):
        pass  # the host's frames deliver what there is


class TimerAriadne(TimerBase):
    """A timer the host's frames tick: due once its interval of frame time has passed, and run at
    most once a frame. A canvas with no host has no loop, so nothing ticks it."""

    def __init__(self, canvas=None, interval=None, callbacks=None):
        self._ari_canvas = canvas
        self._ari_figures = None  # the Figures ticking it while it runs
        self._ari_left = None  # seconds until it is due, while it runs
        super().__init__(interval=interval, callbacks=callbacks)

    def _timer_start(self):
        host = host_for(self._ari_canvas)
        if host is None:
            return
        self._ari_figures = figures_of(host)
        if self not in self._ari_figures.timers:
            self._ari_figures.timers.append(self)
        self._ari_left = self.interval / 1000.0

    def _timer_stop(self):
        figures = getattr(self, "_ari_figures", None)  # also from __del__, perhaps half made
        if figures is not None and self in figures.timers:
            figures.timers.remove(self)
        self._ari_figures = None
        self._ari_left = None

    def _timer_set_interval(self):
        if getattr(self, "_ari_left", None) is not None:
            self._ari_left = self.interval / 1000.0

    def _ari_tick(self, dt):
        if self._ari_left is None:
            return
        self._ari_left -= dt
        if self._ari_left > 0:
            return
        if self.single_shot:
            self._timer_stop()
        else:  # on time on average, with no backlog when the frames come slower than it
            self._ari_left = max(0.0, self._ari_left + self.interval / 1000.0)
        self._on_timer()


class FigureManagerAriadne(FigureManagerBase):
    """A figure in an Ariadne window while it is shown: one of figures.ari's ``figure_<k>``."""

    _toolbar2_class = None  # pan and zoom (NavigationToolbar2) come with the pointer events (C2)

    def __init__(self, canvas, num):
        self._title = f"Figure {num}"
        self._ari_shown = None  # the Figures showing it
        self._ari_close_sent = False
        super().__init__(canvas, num)
        if matplotlib.is_interactive():  # plt.ion(): shown as it is made, as a GUI window is
            self.show()

    def show(self):
        canvas = self.canvas
        host = host_for(canvas)
        if host is None:
            warnings.warn(NO_HOST, stacklevel=2)
            return
        figures = figures_of(host)
        self._ari_shown = figures
        if canvas._ari_bound is None:
            figures.open(self, "image", self._title, src=canvas._ari_name)
        figures.dirty.pop(canvas, None)
        canvas.draw()

    def _ari_window_closed(self):
        Gcf.destroy(self)  # the user closed its window: the figure goes, as plt.close() takes it

    def destroy(self):
        if not self._ari_close_sent:  # as a GUI window's closing sends it (FuncAnimation stops)
            self._ari_close_sent = True
            CloseEvent("close_event", self.canvas)._process()
        figures = self._ari_shown
        if figures is not None and figures.slot_of(self) is not None:
            figures.close(self)
            figures.drop_image(self.canvas._ari_name)

    def get_window_title(self):
        return self._title

    def set_window_title(self, title):
        self._title = str(title)
        if self._ari_shown is not None:
            self._ari_shown.retitle(self, self._title)

    @classmethod
    def start_main_loop(cls):
        _BackendAriadne.mainloop()


FigureCanvasAriadne.manager_class = FigureManagerAriadne


@_Backend.export
class _BackendAriadne(_Backend):
    backend_version = "1"
    FigureCanvas = FigureCanvasAriadne
    FigureManager = FigureManagerAriadne

    @staticmethod
    def mainloop():
        """A blocking ``plt.show()``: the host's loop (``Host.run()``) until its document closes,
        which figures.ari does once its windows are closed. Not from inside a frame, a load or a
        call the document makes, nor where the host has no loop (``check``) or has run it."""
        host = host_for()
        if host is None or host._in_frame or loading() or host.should_close():
            return
        if not program_level():
            return
        try:
            host.run()
        except RuntimeError as e:
            if "has no frame loop" not in str(e) and "already run" not in str(e):
                raise


def bind(figure, image_name, host=None):
    """Publish ``figure`` as the picture ``image_name``, which a document's own ``image:`` widget
    shows (``src: image_name``), instead of in a window of its own: ``plt.show()`` opens none for
    it, and each draw (``draw_idle``, an animation's frame) publishes it again. ``host`` defaults to
    this process's. A figure made without pyplot gets this backend's canvas. Returns the canvas."""
    if not image_name:
        raise ValueError("ariadne_mpl.bind: the picture needs a name")
    canvas = figure.canvas
    if not isinstance(canvas, FigureCanvasAriadne):
        if getattr(canvas, "manager", None) is not None:
            raise ValueError(
                "ariadne_mpl.bind: the figure belongs to another backend's window; use "
                f'matplotlib.use("{BACKEND}") first')
        canvas = FigureCanvasAriadne(figure)
    canvas._ari_bound = str(image_name)
    canvas._ari_host = weakref.ref(host) if host is not None else None
    manager = canvas.manager
    shown = getattr(manager, "_ari_shown", None)
    if shown is not None:
        shown.close(manager)  # plt.show() had opened a window for it
    canvas.draw()
    return canvas


def figure_rgba(figure):
    """``figure`` drawn now, as an (H, W, 4) uint8 array of its own (not published)."""
    canvas = figure.canvas
    if isinstance(canvas, FigureCanvasAgg):
        FigureCanvasAgg.draw(canvas)  # Agg's own draw: ours would publish it too
        return np.array(canvas.buffer_rgba())
    import matplotlib.image as mimage

    buf = io.BytesIO()
    figure.savefig(buf, format="png", dpi=figure.dpi)
    buf.seek(0)
    rgba = mimage.imread(buf, format="png")  # floats in [0, 1]
    return (np.clip(rgba, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
