"""display(obj): an object's richest form, in an Ariadne window (docs/ARIADNE.md §9.6).

As IPython's ``display()`` does in a notebook, the object's MIME bundle decides what is shown:
``_repr_mimebundle_``, ``_repr_png_``, ``_repr_jpeg_``, ``_repr_markdown_``, ``_repr_latex_``, and
``repr()`` as ``text/plain``. Of those the host shows the richest it can draw:

* a matplotlib figure, a numpy picture (uint8 (H, W), (H, W, 1|3|4), or float (H, W, 3|4) in
  [0, 1]) and a PIL image go in as pixels, with nothing encoded;
* ``image/png`` and ``image/jpeg`` data are decoded with PIL (text when there is no PIL);
* ``text/markdown``, ``text/latex`` and ``text/plain`` show as text, not rendered yet.

Each output gets a window of its own beside matplotlib's figures (pycvc_gl.ariadne_mpl), in
``figures.ari`` (beside this file), which a program with no document of its own gets. A document
of its own keeps its windows to itself: there, as with no host at all, display() prints the
text/plain form, as IPython does outside a notebook (a figure still shows through the document's
own ``image:``). ``display_id`` and the returned handle's ``update()`` replace an output in place.

The windows are figures.ari's state: ``figures.count``, and per window ``k`` ``figures.<k>.kind``
(``image``, ``text``, or ``""`` once closed), ``.src`` (the picture's published name), ``.title``
and ``.text``, with the window's own ``tree.ari_figure_<k>.open``. ``Figures``, one per Host
(``host._ari_figures``), keeps them, the matplotlib draws and timers waiting for the host's next
frame, and the windows the user closed.

Which host: the one named (``host=``; a figure's, from ``ariadne_mpl.attach`` or ``bind``), else
the scripting host's where it has lent a window (ariglx: ``pycvc_gl.ariadne.current_host()``).
arix, ``ariglx --terminal`` and arisrvx lend none, so display() prints there. Nothing here is the
process's: a host is named, or it is the scripting host's. This module imports no matplotlib,
numpy or PIL unless the object is one of theirs.
"""

from __future__ import annotations

import base64
import io
import os
import sys
import threading
import traceback
import uuid
import weakref

from . import ariadne as _ariadne

FIGURES_DOCUMENT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures.ari")
WINDOW = "ari_figure_"  # figures.ari's window k is tree.ari_figure_<k>: no document's own id


def live(host):
    """Whether ``host`` still takes writes: not closed, and, embedded, still lent by its scripting
    host (at exit, matplotlib closes its figures after the host and its app have gone)."""
    if host is None or host._closed:
        return False
    lent = getattr(host, "_host", None)  # an embedded Host's _arihost
    return lent is None or bool(lent.attached())


def _lent():
    """This interpreter's scripting host (``_arihost``) once it has lent its document, or None."""
    try:
        import _arihost
    except ImportError:
        return None
    return _arihost if _arihost.attached() else None


def lent_window():
    """Whether this interpreter's scripting host has lent a GL window (ariglx), which matplotlib's
    figures and display() show in: not arix, ``ariglx --terminal``, arisrvx or ``<host> python``."""
    lent = _lent()
    return lent is not None and lent.gl() is not None


def host_for(canvas=None):
    """The Host a canvas's draws, and display()'s output, go to: the one its figure was given
    (``ariadne_mpl.attach``, ``bind``), else the scripting host's where it has lent a window; None
    when there is none. A host with no window (arix, arisrvx) is never adopted."""
    ref = getattr(canvas, "_ari_host", None)
    h = ref() if ref is not None else None
    if live(h):
        return h
    if not lent_window():
        return None
    h = _ariadne.current_host()
    return h if live(h) else None


def figures_of(host):
    """``host``'s Figures, made on first use."""
    f = getattr(host, "_ari_figures", None)
    if f is None:
        f = Figures(host)
        host._ari_figures = f
    return f


def loading():
    """Whether the scripting host is running a document's ``python:`` block (its load), as its app
    keeps it (``_arihost.loading()``)."""
    lent = _lent()
    return lent is not None and bool(lent.loading())


def program_level():
    """Whether the caller runs as a program's own code: a module's top level is on the stack (a
    script, an import, a REPL line) and no Host call that runs the document is. A function or
    handler the document calls, a frame hook, a load's init:, the host calling in with no program
    on the stack, and another thread are not: there a ``plt.show()`` neither blocks nor loads."""
    host = _ariadne.Host
    runs = {host.run.__code__, host.step.__code__, host.settle.__code__, host.load.__code__,
            host._embedded_frame.__code__}
    top = False
    f = sys._getframe(1)
    while f is not None:
        if f.f_code in runs:
            return False
        top = top or f.f_code.co_name == "<module>"
        f = f.f_back
    return top


def program_owns(host):
    """Whether the caller may run ``host``'s frames itself (a blocking ``plt.show()``,
    ``plt.pause()``) or load it figures.ari: it is the program's own code (``program_level()``), on
    the host's thread, outside a frame and a load, and the host's document is not the one a
    scripting host loaded (``ariglx run doc.ari``), whose functions may exec() code that has a top
    level of its own."""
    if not live(host) or host._in_frame or loading():
        return False
    if threading.get_ident() != host._thread:
        return False
    if host.embedded and host.document is None and "argv" in host:
        return False  # every load writes its argv node: the scripting host loaded this document
    return program_level()


class Figures:
    """One host's figure and output windows (figures.ari's ``ari_figure_<k>``), the matplotlib
    canvases to draw and the timers to tick in its next frame. Made by ``figures_of(host)``."""

    def __init__(self, host):
        self.host = host
        self.slots = []  # window k's owner (a figure's manager, an output), or None once closed
        self.dirty = {}  # canvases to draw in the next frame, once each (insertion-ordered)
        self.timers = []  # running matplotlib timers (ariadne_mpl.TimerAriadne)
        self.outputs = {}  # display_id -> its output, while its window is open
        self._outputs_made = 0
        host.on_frame(self._frame)

    def slot_of(self, owner):
        return next((k for k, s in enumerate(self.slots) if s is owner), None)

    def windows(self):
        """Whether the host shows figures.ari, whose windows show the slots. A document of the
        host's own has none of them."""
        return self.host.document == FIGURES_DOCUMENT

    def showing(self):
        """Whether one of figures.ari's windows shows a figure or an output."""
        return self.windows() and any(s is not None for s in self.slots)

    def can_show(self):
        """Whether an output can have a window now: figures.ari is loaded, or loads now."""
        if not live(self.host):
            return False
        self.ensure_document()
        return self.windows()

    def open(self, owner, kind, title, src="", text="", front=True):
        """Show ``owner`` (a manager or an output) in a window: the one it has, else the first
        closed one, else a new one. Returns its index ``k``, or None where figures.ari isn't the
        host's document (a document of its own), which then shows nothing of it."""
        if not self.can_show():  # first: a new document's load starts its windows afresh
            return None
        k = self.slot_of(owner)
        if k is None:
            k = next((i for i, s in enumerate(self.slots) if s is None), len(self.slots))
            if k == len(self.slots):
                self.slots.append(None)
            self.slots[k] = owner
        h = self.host
        h.set(f"figures.{k}.kind", kind)
        h.set(f"figures.{k}.src", src)
        h.set(f"figures.{k}.title", title)
        h.set(f"figures.{k}.text", text)
        h.set("figures.count", len(self.slots))
        win = f"tree.{WINDOW}{k}"
        if h.get(f"{win}.geometry") is None:  # each new window a step down and right of the last
            step = 28 * (k % 10)
            h.set(f"{win}.geometry", f"{24 + step},{40 + step},0,0")
        h.set(f"{win}.open", 1)
        if front:
            h.set(f"{win}.raise", 1)
        return k

    def retitle(self, owner, title):
        k = self.slot_of(owner)
        if k is not None and live(self.host):
            self.host.set(f"figures.{k}.title", title)

    def close(self, owner):
        """Close ``owner``'s window; its slot goes to the next window opened."""
        k = self.slot_of(owner)
        if k is None:
            return
        self.slots[k] = None
        while self.slots and self.slots[-1] is None:
            self.slots.pop()
        h = self.host
        if not live(h):
            return
        h.set(f"tree.{WINDOW}{k}.open", 0)
        h.set(f"figures.{k}.kind", "")
        h.set("figures.count", len(self.slots))

    def drop_image(self, name):
        """Forget the picture ``name`` (its window has closed). The window's own copy stays: its
        GL texture can only go inside a draw, and the name's next picture replaces it (figure
        numbers and output windows are reused, so there are few names)."""
        self.host.images.pop(name, None)

    def ensure_document(self):
        """Load figures.ari when the host has no document and none has loaded: a program in place
        of the document, or a plain ``python`` Host that has loaded nothing. Never from inside a
        frame, a load, or a call the document makes (``program_owns``)."""
        h = self.host
        if h.document is not None or "argv" in h:  # every load writes its argv node
            return
        if not program_owns(h):
            return
        h.load(FIGURES_DOCUMENT)

    def _frame(self, dt):
        """The host's frame (``Host.on_frame``): windows the user closed close what they show,
        timers tick, the canvases draw_idle asked for draw once each, and an embedded host's
        figures.ari closes itself once it shows nothing, which ends the scripting host's loop, so
        a blocking ``plt.show()`` returns. A timer's or a draw's exception is printed, as
        matplotlib's GUI backends print one, and the frame goes on."""
        h = self.host
        for k, owner in enumerate(list(self.slots)):
            shown = h.get(f"tree.{WINDOW}{k}.open")
            if owner is None:
                if shown not in (None, "0"):  # a closed one opened from the Figures menu
                    h.set(f"tree.{WINDOW}{k}.open", 0)
            elif shown == "0":
                try:
                    owner._ari_window_closed()
                except Exception:  # noqa: BLE001 - a close_event handler's: printed
                    traceback.print_exc()
        for t in list(self.timers):
            try:
                t._ari_tick(dt)
            except Exception:  # noqa: BLE001 - a timer's callback (an animation's update)
                traceback.print_exc()
        dirty, self.dirty = self.dirty, {}
        for canvas in dirty:
            try:
                with canvas._idle_draw_cntx():  # its draw's own draw_idle calls ask for no more
                    canvas.draw()
            except Exception:  # noqa: BLE001 - one figure's draw: the others still draw
                traceback.print_exc()
        if h.embedded and self.windows() and self.slots_closed():
            h.request_close()

    def slots_closed(self):
        return all(s is None for s in self.slots)

    # ── display() ────────────────────────────────────────────────────────────────────────────────

    def show_output(self, display_id, kind, payload, title=None, update=False):
        """Show ``payload`` (pixels, or text) as the output ``display_id``: in its window, or in a
        new one. ``update`` only replaces one still open, and leaves it where it is. False when
        there is no window for it (the caller prints it instead)."""
        out = self.outputs.get(display_id)
        if out is None and update:
            return True
        if out is None:
            out = _Output(self, display_id, title or f"Display {self._outputs_made + 1}")
        elif title:
            out.title = title
        k = self.open(out, kind, out.title, text=payload if kind == "text" else "",
                      front=not update)
        if k is None:
            return False
        if display_id not in self.outputs:
            self._outputs_made += 1
            self.outputs[display_id] = out
        src = f"display-{k}"
        if kind == "image":
            self.host.publish_image(src, payload)
            self.host.set(f"figures.{k}.src", src)
        else:  # text now: no picture of an earlier output is kept for it
            self.drop_image(src)
        return True


class _Output:
    """One display_id's window."""

    def __init__(self, figures, display_id, title):
        self.figures, self.display_id, self.title = figures, display_id, title

    def _ari_window_closed(self):
        f = self.figures
        k = f.slot_of(self)
        f.close(self)
        f.outputs.pop(self.display_id, None)
        if k is not None:
            f.drop_image(f"display-{k}")


class DisplayHandle:
    """What ``display()`` returns: ``update(obj)`` replaces the output in its window, and
    ``display(obj)`` shows ``obj`` there, opening the window again if it was closed."""

    def __init__(self, display_id, host=None):
        self.display_id = display_id
        self._host = weakref.ref(host) if host is not None else None

    def _target(self):
        return self._host() if self._host is not None else None

    def update(self, obj, **kwargs):
        display(obj, display_id=self.display_id, host=self._target(), _update=True, **kwargs)

    def display(self, obj, **kwargs):
        display(obj, display_id=self.display_id, host=self._target(), **kwargs)

    def __repr__(self):
        return f"<DisplayHandle display_id={self.display_id}>"


def display(obj, *, title=None, display_id=None, host=None, _update=False):
    """Show ``obj`` in an Ariadne window, as IPython's ``display()`` shows it in a notebook: see
    the module docstring for what it draws. ``display_id`` names the output, so a later display()
    with it, or the handle's ``update()``, replaces it (``True`` or None: a new one). ``host``
    defaults to the scripting host's. Where there is no window for it (no host, a host that has
    closed, a document of the host's own) it prints the text/plain form. Returns a
    ``DisplayHandle``."""
    did = display_id if isinstance(display_id, str) and display_id else uuid.uuid4().hex
    target = host if host is not None else host_for()
    if target is not None and not live(target):
        target = None  # closed, or let go by its scripting host: as with no host
    figures = figures_of(target) if target is not None else None
    if figures is not None and figures.can_show():
        kind, payload = render(obj)
        if figures.show_output(did, kind, payload, title=title, update=_update):
            return DisplayHandle(did, target)
    if not _update:  # as IPython outside a notebook: the plain text
        print(mime_bundle(obj)["text/plain"])
    return DisplayHandle(did, target)


# ── what an object offers, and what is drawn of it ───────────────────────────────────────────────

_REPR_METHODS = (("image/png", "_repr_png_"), ("image/jpeg", "_repr_jpeg_"),
                 ("text/markdown", "_repr_markdown_"), ("text/latex", "_repr_latex_"))


def _call_repr(obj, name):
    # Looked up on the class: an instance whose __getattr__ answers any name offers nothing.
    if getattr(type(obj), name, None) is None:
        return None
    try:
        fn = getattr(obj, name)
        if name == "_repr_mimebundle_":
            try:
                return fn(include=None, exclude=None)
            except TypeError:
                return fn()
        return fn()
    except Exception:  # noqa: BLE001 - a broken formatter offers nothing, as in IPython
        return None


def mime_bundle(obj):
    """The object's MIME bundle as IPython's formatters build it: ``_repr_mimebundle_``'s, then
    each ``_repr_<format>_`` it has for a type the bundle lacks, and ``repr()`` as text/plain."""
    data = {}
    bundle = _call_repr(obj, "_repr_mimebundle_")
    if isinstance(bundle, tuple):  # (data, metadata)
        bundle = bundle[0]
    if isinstance(bundle, dict):
        data.update((k, v) for k, v in bundle.items() if v is not None)
    for mime, name in _REPR_METHODS:
        if mime not in data:
            v = _call_repr(obj, name)
            if isinstance(v, tuple):
                v = v[0]
            if v is not None:
                data[mime] = v
    if "text/plain" not in data:
        data["text/plain"] = repr(obj)
    return data


def _picture(obj):
    """``obj`` as pixels publish_image takes, when it is a picture of matplotlib's, numpy's or
    PIL's; else None. Their modules are looked up, never imported: an object of theirs means they
    are loaded."""
    figure = sys.modules.get("matplotlib.figure")
    if figure is not None and isinstance(obj, figure.Figure):
        from . import ariadne_mpl

        return ariadne_mpl.figure_rgba(obj)
    np = sys.modules.get("numpy")
    if np is not None and isinstance(obj, np.ndarray):
        channels = obj.shape[2] if obj.ndim == 3 else (1 if obj.ndim == 2 else 0)
        if obj.dtype == np.uint8:
            ok = channels in (1, 3, 4)
        else:  # a 2-D float array is data, not a grey picture
            ok = obj.dtype.kind == "f" and obj.ndim == 3 and channels in (3, 4)
        return obj if ok and obj.size > 0 else None  # any other array shows as its repr
    pil = sys.modules.get("PIL.Image")
    if pil is not None and isinstance(obj, pil.Image):
        import numpy

        return numpy.asarray(obj.convert("RGBA"))
    return None


def _decode(data):
    """PNG or JPEG data (bytes, or base64 text as a JSON bundle carries it) as RGBA pixels, or None
    without PIL."""
    if isinstance(data, str):
        try:
            data = base64.b64decode(data)
        except ValueError:
            return None
    if not isinstance(data, (bytes, bytearray, memoryview)):
        return None
    try:
        import numpy
        from PIL import Image
    except ImportError:
        return None
    try:
        with Image.open(io.BytesIO(bytes(data))) as im:
            return numpy.asarray(im.convert("RGBA"))
    except Exception:  # noqa: BLE001 - not a picture PIL reads: shown as its text instead
        return None


def render(obj):
    """What display() shows of ``obj``: ``("image", pixels)`` or ``("text", str)``."""
    pixels = _picture(obj)
    if pixels is not None:
        return "image", pixels
    bundle = mime_bundle(obj)
    for mime in ("image/png", "image/jpeg"):
        if mime in bundle:
            pixels = _decode(bundle[mime])
            if pixels is not None:
                return "image", pixels
    for mime in ("text/markdown", "text/latex", "text/plain"):
        if mime in bundle:
            return "text", str(bundle[mime])
    return "text", repr(obj)
