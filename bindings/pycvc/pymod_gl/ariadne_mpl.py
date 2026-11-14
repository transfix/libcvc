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
  host's frames until those windows are closed, as matplotlib's GUI backends do, where someone can
  close them: in a window, or in a scripting host's loop (which ends then, so later figures can't
  show). On an offscreen or headless Host it returns at once, as Agg's does. One called during a
  load, from a function the document calls, or from a frame neither blocks nor loads a document.
  Closing a figure's window closes the figure.
* **bind(fig, "name")** publishes a figure under a name that a document's own ``image:`` widget
  shows (``src: name``), with no window of its own. In a document of its own (``ariglx run
  doc.ari``), that, or ``src: figure-<num>``, is how a figure shows.
* **draw_idle()** draws once, in the host's next frame, however often it was asked, and timers
  (``fig.canvas.new_timer()``, so ``FuncAnimation``) tick on the host's frames. ``plt.pause()``
  runs the host's frames for its interval where the program runs them (not in a frame, a load or a
  document's function).

The windows and their state are pycvc_gl.ariadne_display's ``Figures``, which display() shares.
Which host: the one a figure was given, else the scripting host's where it has lent a window. A
plain ``python`` program with a Host of its own gives its figures that Host: ``show_in(host)`` in
place of ``plt.show()``, or ``attach(host)`` once it has made a figure (pyplot's next figures
follow its current one), or ``bind(fig, name, host)``. The host is the figure's: nothing is the
process's. With no host the canvas is plain Agg, and ``plt.show()`` warns. The mouse, the wheel,
keys and NavigationToolbar2's pan and zoom come next (roadmap C2).
"""

from __future__ import annotations

import io
import math
import time
import warnings
import weakref

import matplotlib
import numpy as np
from matplotlib._pylab_helpers import Gcf
from matplotlib.backend_bases import CloseEvent, FigureManagerBase, TimerBase, _Backend
from matplotlib.backends.backend_agg import FigureCanvasAgg

from .ariadne_display import figures_of, host_for, program_owns

__all__ = ["BACKEND", "FigureCanvasAriadne", "FigureManagerAriadne", "TimerAriadne", "attach",
           "bind", "figure_rgba", "show_in"]

BACKEND = "module://pycvc_gl.ariadne_mpl"
NO_HOST = ("pycvc_gl.ariadne_mpl: no Ariadne host for this figure; run under ariglx, or give it "
           "your Host: ariadne_mpl.show_in(host) or ariadne_mpl.attach(host)")
LOOP_ENDED = ("pycvc_gl.ariadne_mpl: the host's loop has run and ended (an earlier plt.show() "
              "returned once its windows closed), so plt.show() can't show these figures")


class FigureCanvasAriadne(FigureCanvasAgg):
    """matplotlib's Agg canvas, whose draws go to an Ariadne host as a picture."""

    def __init__(self, figure=None):
        super().__init__(figure)
        self._ari_draws = 0  # the draws published
        self._ari_bound = None  # the picture's name, when bind() gave one
        self._ari_host = None  # a weak reference to the host attach() or bind() gave it

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
        pass  # the host's frames deliver what there is; start_event_loop runs them

    def start_event_loop(self, timeout=0):
        """``plt.pause(interval)``'s wait: the host's frames for ``timeout`` seconds (until
        ``stop_event_loop()`` with none), paced in real time, so the window repaints, timers tick
        and draw_idle draws. Only where the program runs the host's frames (``program_owns``);
        elsewhere, and with no host, matplotlib's own wait, which runs no frame."""
        host = host_for(self)
        if host is None or not program_owns(host) or host.should_close():
            super().start_event_loop(timeout)
            return
        end = time.monotonic() + (timeout if timeout > 0 else math.inf)
        self._looping = True
        host.run(until=lambda: not self._looping or time.monotonic() >= end, realtime=True)


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
    """A figure in an Ariadne window while it is shown: one of figures.ari's ``ari_figure_<k>``."""

    _toolbar2_class = None  # pan and zoom (NavigationToolbar2) come with the pointer events (C2)

    def __init__(self, canvas, num):
        self._title = f"Figure {num}"
        self._ari_shown = None  # the Figures showing it
        self._ari_close_sent = False
        if getattr(canvas, "_ari_host", None) is None:  # pyplot's: where its current figure goes
            current = Gcf.get_active()
            canvas._ari_host = getattr(getattr(current, "canvas", None), "_ari_host", None)
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
        if canvas._ari_bound is None:  # no window in a document of the host's own: by name only
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


def _shown_host():
    """The host plt.show()'s figures went to: the first open figure's, else the scripting
    host's."""
    for manager in Gcf.get_all_fig_managers():
        h = host_for(manager.canvas)
        if h is not None:
            return h
    return host_for()


@_Backend.export
class _BackendAriadne(_Backend):
    backend_version = "1"
    FigureCanvas = FigureCanvasAriadne
    FigureManager = FigureManagerAriadne

    @staticmethod
    def mainloop():
        """A blocking ``plt.show()``: the host's frames until the figures' windows are closed.
        Only where the program runs the host (``program_owns``: not from a frame, a load or a call
        the document makes), and someone can close a window:

        * embedded, the scripting host's own loop (``Host.run()``), which figures.ari ends once its
          windows are closed. It runs once: a later blocking plt.show() warns. Where the host has
          no loop (``check``) it returns;
        * a plain Host in a window: its frames until figures.ari shows nothing (or, with a
          document of its own, until it closes), and figures.ari stays for the next plt.show();
        * a plain offscreen or headless Host: no one can close a window, so it returns at once,
          as Agg's plt.show() does.
        """
        host = _shown_host()
        if host is None or not program_owns(host):
            return
        if host.embedded:
            if host.should_close():
                warnings.warn(LOOP_ENDED, stacklevel=3)
                return
            try:
                host.run()
            except RuntimeError as e:  # _arihost.run refused: no loop here, or it has run
                if "already run" in str(e):
                    warnings.warn(LOOP_ENDED, stacklevel=3)
                elif not str(e).startswith("_arihost.run:"):
                    raise
            return
        if not _closable(host):
            return
        figures = figures_of(host)
        if figures.windows():
            host.run(until=lambda: not figures.showing())
        else:
            host.run()


def _closable(host):
    """Whether someone can close a plain Host's windows, so a blocking ``plt.show()`` ends: a Host
    in a window; not an offscreen or headless one."""
    return host.mode == "window"


def attach(host, *figures):
    """Show matplotlib figures in ``host`` (a ``pycvc_gl.ariadne.Host``): ``figures``, else every
    figure pyplot has open, and after them each figure pyplot makes while one of them is its
    current figure. Their draws, windows and timers go to ``host``. What a plain ``python`` program
    with a Host of its own calls once it has made a figure; a program in ariglx needs nothing. The
    host is the figures', not the process's: nothing else is changed. ``attach(None)`` lets them
    go. Returns the host's ``Figures`` (None for None)."""
    managers = [f.canvas.manager for f in figures] if figures else Gcf.get_all_fig_managers()
    ref = weakref.ref(host) if host is not None else None
    for manager in managers:
        canvas = getattr(manager, "canvas", None)
        if isinstance(canvas, FigureCanvasAriadne):
            canvas._ari_host = ref
    return figures_of(host) if host is not None else None


def show_in(host, *, block=None):
    """``plt.show()`` in ``host``: every figure pyplot has open is attached to it (``attach``),
    then shown (``plt.show(block=block)``). What a plain ``python`` program with a Host of its own
    calls in place of ``plt.show()``, so a figure it made once none was open goes there too."""
    import matplotlib.pyplot as plt

    attach(host)
    plt.show(block=block)


def bind(figure, image_name, host=None):
    """Publish ``figure`` as the picture ``image_name``, which a document's own ``image:`` widget
    shows (``src: image_name``), instead of in a window of its own: ``plt.show()`` opens none for
    it, and each draw (``draw_idle``, an animation's frame) publishes it again. ``host`` defaults
    to the figure's own (``attach``), else the scripting host's. A figure made without pyplot gets
    this backend's canvas. Returns the canvas."""
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
    if host is not None:
        canvas._ari_host = weakref.ref(host)
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
