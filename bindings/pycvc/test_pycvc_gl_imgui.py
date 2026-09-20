"""pycvc_gl Dear ImGui overlay + state-bound ui panels — Python HUDs/UIs.

Python builds cvcGL HUDs WITHOUT raw ImGui:: (which would crash: the extension
has its own null GImGui). It sets a draw callback and calls the state-bound
ui_* widgets/panels, whose bodies run inside cvcGL against the overlay's context.

Binding-surface assertions always run (no GL). Constructing an ImGuiOverlay needs
an offscreen viewer, so that block is SKIP-guarded; the ui_* widgets are NEVER
invoked here (they need a live ImGui frame). os._exit avoids the known, harmless
boost::lock_error in cvcGL's static teardown at interpreter exit.
"""

import os
import sys

import pycvc
import pycvc_gl

fails = 0


def check(what, ok):
    global fails
    print("  [%s] %s" % ("PASS" if ok else "FAIL", what))
    if not ok:
        fails += 1


app = pycvc.make_app()

print("A. ImGuiOverlay binding surface present (no GL needed)")
OV = pycvc_gl.ImGuiOverlay
for m in (
    "setDrawCallback",
    "attachCamera",
    "setVisible",
    "visible",
    "enabled",
    "wantsMouse",
    "wantsKeyboard",
    "setUiScale",
    "uiScale",
    "setTouchMode",
    "touchMode",
    "setToggleButtonEnabled",
    "toggleButtonEnabled",
    "setPanelsOpen",
    "panelsOpen",
):
    check("ImGuiOverlay.%s present" % m, hasattr(OV, m))
# imguiContext() is deliberately NOT bound (opaque ImGuiContext*; Python issues no
# raw ImGui, so it never needs SetCurrentContext).
check("ImGuiOverlay.imguiContext deliberately absent", not hasattr(OV, "imguiContext"))

print("B. state-bound ui_* widgets/panels present at module scope")
for fn in (
    "ui_slider_double",
    "ui_slider_int",
    "ui_drag_double",
    "ui_checkbox",
    "ui_menu_item",
    "ui_combo",
    "ui_text",
    "ui_camera_menu_items",
    "ui_scene_panel",
    "ui_scene_panel_open",
    "ui_stage_lighting_panel",
    "ui_stage_lighting_panel_open",
    "ui_scene_menu_items",
    "ui_scene_menu_items_open",
):
    check("pycvc_gl.%s present" % fn, hasattr(pycvc_gl, fn) and callable(getattr(pycvc_gl, fn)))

print("B2. curated raw imgui_* subset present + collision-free with ui_*")
imgui_fns = (
    "imgui_begin",
    "imgui_end",
    "imgui_same_line",
    "imgui_separator",
    "imgui_spacing",
    "imgui_push_id",
    "imgui_push_id_int",
    "imgui_pop_id",
    "imgui_text",
    "imgui_text_disabled",
    "imgui_button",
    "imgui_small_button",
    "imgui_selectable",
    "imgui_checkbox",
    "imgui_slider_float",
    "imgui_slider_int",
    "imgui_drag_float",
    "imgui_begin_main_menu_bar",
    "imgui_end_main_menu_bar",
    "imgui_begin_menu_bar",
    "imgui_end_menu_bar",
    "imgui_begin_menu",
    "imgui_end_menu",
    "imgui_menu_item",
    "imgui_menu_item_toggle",
    "imgui_collapsing_header",
    "imgui_begin_disabled",
    "imgui_end_disabled",
    "imgui_is_item_hovered",
    "imgui_set_tooltip",
)
for fn in imgui_fns:
    check("pycvc_gl.%s present" % fn, hasattr(pycvc_gl, fn) and callable(getattr(pycvc_gl, fn)))
# The imgui_* raw subset and the ui_* state-bound helpers must not collide (a
# flattened same-name would silently clobber one). They are disjoint by prefix;
# assert it so a future rename that crosses them fails loudly here.
ui_names = {n for n in dir(pycvc_gl) if n.startswith("ui_")}
imgui_names = {n for n in dir(pycvc_gl) if n.startswith("imgui_")}
check("imgui_* and ui_* namespaces are disjoint", ui_names.isdisjoint(imgui_names))
# The raw widgets need a live ImGui frame; NEVER call one here (would crash/no-op
# outside an overlay draw callback). Presence is the headless contract.

print("C. construct an overlay on an offscreen viewer (SKIP if no GL)")
try:
    sg = pycvc_gl.SceneGraph(app, "scene")
    view = pycvc_gl.SceneRenderer(sg, 64, 48, True, "left")  # offscreen
    ov = pycvc_gl.ImGuiOverlay(view)
    check("ImGuiOverlay constructed", ov is not None)
    ov.setDrawCallback(lambda: None)  # a Python draw callback (reuses the callable typemap)
    check("draw callback accepted", True)
    ov.setUiScale(1.5)
    check("uiScale round-trips", abs(ov.uiScale() - 1.5) < 1e-6)
    # exercise the bool controls; their inert-mode (CVC_ENABLE_IMGUI off) value is
    # not asserted, only that the getters marshal a bool.
    ov.setVisible(True)
    ov.setPanelsOpen(True)
    ov.setTouchMode(False)
    check(
        "bool getters marshal bools",
        isinstance(ov.visible(), bool) and isinstance(ov.panelsOpen(), bool),
    )
    ov.attachCamera(pycvc_gl.CameraController(view))
    check("attachCamera stashes keepalive", getattr(ov, "_pycvc_camera", None) is not None)
except Exception as e:  # noqa: BLE001 — offscreen GL may be unavailable
    print("  [SKIP] ImGuiOverlay offscreen path (GL unavailable): %s" % e)

print("\n%s (%d failures)" % ("ALL PASS" if fails == 0 else "FAILED", fails))

# cvcGL's static GL/state teardown races at interpreter exit (harmless
# boost::lock_error after results print); exit hard with the real status.
sys.stdout.flush()
os._exit(1 if fails else 0)
