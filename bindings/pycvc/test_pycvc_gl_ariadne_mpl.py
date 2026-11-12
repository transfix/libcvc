"""pycvc_gl.ariadne_mpl and pycvc_gl.ariadne_display: matplotlib's figures and display() in an
Ariadne host (docs/ARIADNE.md §9.6), with headless Hosts (no GL: nothing is drawn, and
host.images keeps what is published).

* the canvas: a draw publishes a copy of its RGBA pixels as figure-<num>, and draw_idle draws
  once, in the next frame;
* windows: plt.show() opens one per figure, in figures.ari for a host with no document; plt.close()
  and a window the user closed close it; set_window_title retitles it; a figure bind() named
  publishes under that name, with no window;
* the loop: timers tick on the frames (a single-shot one once, FuncAnimation's too), a blocking
  plt.show() runs the host's loop until its figures are closed, and a call from a frame or another
  thread is not the program's own (where plt.show() neither blocks nor loads a document);
* display(): numpy pictures, PIL images, _repr_png_, a mimebundle, markdown, plain text, a figure,
  a handle's update and display_id, a window the user closed;
* with no host the canvas is plain Agg, plt.show() warns, and display() prints.

Prints SKIP and exits 0 when this Python has no matplotlib. ctest: pycvc_gl_ariadne_mpl.
"""

import contextlib
import io
import os
import sys
import tempfile
import threading
import warnings

try:
    import matplotlib
except ImportError:
    print("SKIP: no matplotlib")
    sys.exit(0)

matplotlib.use("module://pycvc_gl.ariadne_mpl")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402

from pycvc_gl import ariadne_display, ariadne_mpl  # noqa: E402
from pycvc_gl.ariadne import Host, display  # noqa: E402

checks = 0

DOC = """meta:
  name: matplotlib host test
  min_libcvc: 3.7.13
windows:
  - window: Mine
    id: mine
    size: [660, 540]
    children:
      - image: ""
        src: plot
        size: "100%"
"""


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


def new_host(name):
    """A headless Host this process's figures go to, with none of the last test's left."""
    plt.close("all")
    host = Host(mode="headless", name=name)
    ariadne_mpl.attach(host)
    return host


def plot(title="sin"):
    fig, ax = plt.subplots()
    x = np.linspace(0.0, 6.3, 64)
    ax.plot(x, np.sin(x))
    ax.set_title(title)
    return fig


def write_doc():
    """A document of the test's own: one window whose image: shows the picture "plot"."""
    path = os.path.join(tempfile.mkdtemp(prefix="pycvc_gl_ariadne_mpl_"), "doc.ari")
    with open(path, "w", encoding="utf-8") as f:
        f.write(DOC)
    return path


def slot(host, handle):
    """The window index display()'s output `handle` is shown in."""
    figures = ariadne_display.figures_of(host)
    return figures.slot_of(figures.outputs[handle.display_id])


def no_host_acts_as_agg_and_warns():
    plt.close("all")
    ariadne_mpl.attach(None)
    fig = plot()
    fig.canvas.draw()
    check(type(fig.canvas) is ariadne_mpl.FigureCanvasAriadne, "pyplot makes the backend's canvas")
    check(fig.canvas._ari_draws == 0, "with no host a draw publishes nothing")
    check(np.asarray(fig.canvas.buffer_rgba()).shape == (480, 640, 4), "... and is Agg's")
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        plt.show()  # returns: there is no loop to run
    check(any("no Ariadne host" in str(w.message) for w in caught), "plt.show() warns with no host")
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        h = display({"a": 1})
    check(out.getvalue().strip() == "{'a': 1}", "display() prints text/plain with no host")
    check(isinstance(h, ariadne_display.DisplayHandle) and h.display_id, "... and returns a handle")
    plt.close("all")


def canvas_draw_publishes_an_rgba_copy():
    host = new_host("draw")
    fig = plot()
    fig.canvas.draw()
    img = host.images.get("figure-1")
    check(img is not None and img.shape == (480, 640, 4) and img.dtype == np.uint8,
          "a draw publishes figure-1, 640x480 RGBA uint8")
    check((img[..., :3] < 200).any() and (img[..., :3] > 250).any(), "the plot is drawn on white")
    before = img.copy()
    fig.axes[0].plot([0.0, 6.3], [1.0, -1.0], color="red", lw=12)
    fig.canvas.draw()
    check(np.array_equal(img, before), "the published array is a copy: a redraw leaves it be")
    check(not np.array_equal(host.images["figure-1"], before), "the redraw published new pixels")
    check(fig.canvas._ari_draws == 2, f"two draws, two publishes ({fig.canvas._ari_draws})")


def draw_idle_coalesces_into_one_draw_per_frame():
    host = new_host("idle")
    fig = plot()
    for _ in range(5):
        fig.canvas.draw_idle()
    check(fig.canvas._ari_draws == 0, "draw_idle doesn't draw at once")
    host.step()
    check(fig.canvas._ari_draws == 1, f"five draw_idles, one draw in the next frame "
                                      f"({fig.canvas._ari_draws})")
    host.step()
    check(fig.canvas._ari_draws == 1, "and none in the frame after")
    plt.ion()  # pyplot asks for a draw on each change (stale), and a new figure shows at once
    try:
        live = plot("interactive")
        check(host.get("figures.count") == "1" and host.get("figures.0.src") == "figure-2",
              "interactive: a new figure opens its window")
        host.step()
        n = live.canvas._ari_draws
        live.axes[0].set_title("changed")
        host.step()
        host.step()
        check(live.canvas._ari_draws == n + 1, f"a change, one draw ({live.canvas._ari_draws - n})")
    finally:
        plt.ioff()


def show_opens_a_window_slot():
    host = new_host("show")
    plot()
    plt.show(block=False)
    check(host.document == ariadne_display.FIGURES_DOCUMENT, "a host with no document gets "
                                                             "figures.ari")
    check(host.get("figures.count") == "1", "one figure, one window")
    check(host.get("figures.0.kind") == "image" and host.get("figures.0.src") == "figure-1"
          and host.get("figures.0.title") == "Figure 1", "window 0 shows figure-1, titled Figure 1")
    check(host.get("tree.figure_0.open") == "1" and host.get("tree.figure_0.raise") == "1",
          "its window is open, and comes to the front")
    check("figure-1" in host.images, "show() drew it")
    plot("cos")
    plt.show(block=False)
    check(host.get("figures.count") == "2" and host.get("figures.1.src") == "figure-2",
          "a second figure, a second window")
    check(host.get("tree.figure_1.geometry") != host.get("tree.figure_0.geometry"),
          "each window opens a step from the last")
    host.step()
    check(not host.should_close(), "figures.ari stays while it shows a figure")


def close_closes_its_window():
    host = new_host("close")
    plot()
    second = plot("cos")
    plt.show(block=False)
    plt.close(second)
    check(host.get("tree.figure_1.open") == "0" and host.get("figures.count") == "1",
          "plt.close() closes its window, and the last slot goes")
    check("figure-2" not in host.images, "its picture is dropped")
    third = plot("tan")
    plt.show(block=False)
    check(host.get("figures.count") == "2" and host.get("figures.1.src") == f"figure-{third.number}"
          and host.get("tree.figure_1.open") == "1", "the next figure takes the free window")


def a_window_the_user_closed_destroys_the_figure():
    host = new_host("user")
    fig = plot()
    closed = []
    fig.canvas.mpl_connect("close_event", closed.append)
    plt.show(block=False)
    host.set("tree.figure_0.open", 0)  # what its close button writes
    host.step()
    check(not plt.fignum_exists(fig.number), "closing its window closes the figure")
    check(len(closed) == 1, f"a close_event, once ({len(closed)})")
    check(host.get("figures.count") == "0", "no window left")
    check(host.should_close(), "figures.ari closes once it shows nothing")


def set_window_title():
    host = new_host("title")
    fig = plot()
    fig.canvas.manager.set_window_title("Before")
    plt.show(block=False)
    check(host.get("figures.0.title") == "Before", "a title set before show() is the window's")
    fig.canvas.manager.set_window_title("Sines")
    check(host.get("figures.0.title") == "Sines"
          and fig.canvas.manager.get_window_title() == "Sines", "set_window_title retitles it")


def bound_figure_publishes_by_name_without_a_window():
    host = new_host("bound")
    doc = write_doc()
    host.load(doc)
    fig = plot()
    canvas = ariadne_mpl.bind(fig, "plot")
    check(canvas is fig.canvas and host.images["plot"].shape == (480, 640, 4),
          "bind() publishes the figure under its name at once")
    plt.show(block=False)
    check(host.get("figures.count") is None, "plt.show() opens no window for a bound figure")
    check(host.document == doc, "and a host with a document keeps it")
    before = host.images["plot"].copy()
    fig.axes[0].plot([0.0, 6.3], [-1.0, 1.0], color="green", lw=10)
    fig.canvas.draw_idle()
    host.step()
    check(not np.array_equal(host.images["plot"], before), "draw_idle publishes it again")
    from matplotlib.figure import Figure

    raw = Figure(figsize=(2, 1), dpi=50)
    raw.add_subplot().plot([0, 1], [1, 0])
    ariadne_mpl.bind(raw, "raw", host=host)
    check(host.images["raw"].shape == (50, 100, 4), "a figure made without pyplot binds too")


def timer_fires_on_frames_and_single_shot_stops():
    host = new_host("timer")
    fig = plot()
    ticks = []
    t = fig.canvas.new_timer(interval=100)
    t.add_callback(lambda: ticks.append(1))
    t.start()
    for _ in range(10):
        host.step(0.05)  # half a second of frames
    check(len(ticks) == 5, f"a 100 ms timer fires 5 times in 0.5 s of frames ({len(ticks)})")
    t.stop()
    host.step(0.2)
    check(len(ticks) == 5, "stopped, it fires no more")
    once = []
    s = fig.canvas.new_timer(interval=10)
    s.single_shot = True
    s.add_callback(lambda: once.append(1))
    s.start()
    for _ in range(5):
        host.step(0.05)
    check(once == [1], "a single-shot timer fires once")
    check(not ariadne_display.figures_of(host).timers, "and nothing is left ticking")

    frames = []
    line = fig.axes[0].lines[0]

    def update(i):
        frames.append(i)
        line.set_ydata(np.sin(np.linspace(0.0, 6.3, 64) + i))
        return (line,)

    anim = FuncAnimation(fig, update, frames=3, interval=50, repeat=False)
    plt.show(block=False)  # its first draw starts the animation's timer
    draws = fig.canvas._ari_draws
    for _ in range(10):
        host.step(0.05)
    check(frames[-3:] == [0, 1, 2], f"FuncAnimation runs on the frames ({frames})")
    check(fig.canvas._ari_draws > draws, "and each of its frames is published")
    del anim


def blocking_show_returns_when_its_figures_close():
    host = new_host("block")
    fig = plot()
    closer = fig.canvas.new_timer(interval=50)
    closer.single_shot = True
    closer.add_callback(plt.close, fig)
    closer.start()
    hung = []
    watchdog = fig.canvas.new_timer(interval=20000)
    watchdog.add_callback(lambda: (hung.append(1), host.request_close()))
    watchdog.start()
    plt.show()  # blocks: the host's frames run until figures.ari, with no window left, closes
    watchdog.stop()
    check(not hung, "plt.show() returned once its figure closed, not on the watchdog")
    check(not plt.fignum_exists(fig.number) and host.should_close(),
          "the figure closed, and with it figures.ari")
    check(host.frame_count >= 3, f"the host's loop ran the frames ({host.frame_count})")


def calls_from_frames_and_threads_are_not_the_programs():
    host = new_host("level")
    seen, shown = [], []

    @host.function("replot")
    def replot():
        seen.append(ariadne_display.program_level())
        plot("from the document")
        plt.show()  # neither blocks nor loads a document: the document called it
        shown.append(host.document)
        return 1

    host.load(write_doc())
    check(ariadne_display.program_level(), "a script's own code is the program's")
    t = threading.Thread(target=lambda: seen.append(ariadne_display.program_level()))
    t.start()
    t.join()
    check(seen == [False], "another thread's is not")
    host.on_frame(lambda dt: seen.append(ariadne_display.program_level()))
    host.run_program('(state-set "r" (replot))')
    host.step()
    check(seen[1:] == [False, False], f"a function the document calls and a frame hook are not "
                                      f"({seen})")
    check(shown == [host.document] and host.document.endswith("doc.ari") and host.get("r") == "1",
          "plt.show() from there returns, and loads no figures.ari")


def display_shows_pictures_and_text():
    from PIL import Image

    host = new_host("display")
    h = display(np.full((8, 16, 3), 200, np.uint8), title="Grey")
    k = slot(host, h)
    check(host.get(f"figures.{k}.kind") == "image"
          and host.get(f"figures.{k}.src") == f"display-{k}"
          and host.get(f"figures.{k}.title") == "Grey", "a numpy picture is an image, titled")
    check(host.images[f"display-{k}"].shape == (8, 16, 4), "... published as RGBA")
    check(host.document == ariadne_display.FIGURES_DOCUMENT, "display() loads figures.ari too")

    k = slot(host, display(Image.new("RGB", (5, 3), (255, 0, 0))))
    px = host.images[f"display-{k}"]
    check(px.shape == (3, 5, 4) and tuple(px[0, 0]) == (255, 0, 0, 255), "a PIL image")

    class Png:
        def _repr_png_(self):
            buf = io.BytesIO()
            Image.new("RGBA", (4, 2), (0, 255, 0, 255)).save(buf, "PNG")
            return buf.getvalue()

    k = slot(host, display(Png()))
    px = host.images[f"display-{k}"]
    check(host.get(f"figures.{k}.kind") == "image" and px.shape == (2, 4, 4)
          and tuple(px[1, 3]) == (0, 255, 0, 255), "_repr_png_ is decoded")

    class Md:
        def _repr_markdown_(self):
            return "# Title"

    k = slot(host, display(Md()))
    check(host.get(f"figures.{k}.kind") == "text" and host.get(f"figures.{k}.text") == "# Title",
          "markdown is text")

    class Bundle:
        def _repr_mimebundle_(self, include=None, exclude=None):
            return {"text/plain": "plain", "text/markdown": "**md**"}, {}

    k = slot(host, display(Bundle()))
    check(host.get(f"figures.{k}.text") == "**md**", "a mimebundle's richest form we draw")
    k = slot(host, display({"a": 1}))
    check(host.get(f"figures.{k}.kind") == "text" and host.get(f"figures.{k}.text") == "{'a': 1}",
          "anything else is its repr")
    k = slot(host, display(np.arange(6.0).reshape(2, 3)))
    check(host.get(f"figures.{k}.kind") == "text", "a 2-D float array is data, not a picture")

    fig = plot()
    k = slot(host, display(fig))
    check(host.images[f"display-{k}"].shape == (480, 640, 4), "a figure, drawn now")
    check("figure-1" not in host.images and fig.canvas._ari_draws == 0,
          "... and not published as the figure's own")


def display_handle_update():
    host = new_host("update")
    h = display("first", display_id="out")
    k = slot(host, h)
    check(h.display_id == "out" and host.get(f"figures.{k}.text") == "'first'", "a display_id")
    host.set(f"tree.figure_{k}.raise", "")
    h.update("second")
    check(slot(host, h) == k and host.get(f"figures.{k}.text") == "'second'",
          "update() replaces it in its window")
    check(host.get(f"tree.figure_{k}.raise") == "", "... without bringing it to the front")
    display(np.zeros((2, 2), np.uint8), display_id="out")
    check(slot(host, h) == k and host.get(f"figures.{k}.kind") == "image",
          "display() with its id replaces it too, a picture now")
    other = display("other")
    check(slot(host, other) != k, "a display with no id gets a window of its own")
    host.set(f"tree.figure_{k}.open", 0)  # the user closed it
    host.step()
    check("out" not in ariadne_display.figures_of(host).outputs
          and host.get(f"figures.{k}.kind") == "", "a window the user closed ends its output")
    h.update("ignored")
    check("out" not in ariadne_display.figures_of(host).outputs, "update() of a closed one: no-op")
    h.display("back")
    check(host.get(f"figures.{slot(host, h)}.text") == "'back'", "display() shows it again")


def main():
    no_host_acts_as_agg_and_warns()
    canvas_draw_publishes_an_rgba_copy()
    draw_idle_coalesces_into_one_draw_per_frame()
    show_opens_a_window_slot()
    close_closes_its_window()
    a_window_the_user_closed_destroys_the_figure()
    set_window_title()
    bound_figure_publishes_by_name_without_a_window()
    timer_fires_on_frames_and_single_shot_stops()
    blocking_show_returns_when_its_figures_close()
    calls_from_frames_and_threads_are_not_the_programs()
    display_shows_pictures_and_text()
    display_handle_update()
    plt.close("all")
    ariadne_mpl.attach(None)
    print(f"pycvc_gl.ariadne_mpl test: OK ({checks} checks, matplotlib {matplotlib.__version__})")


if __name__ == "__main__":
    main()
