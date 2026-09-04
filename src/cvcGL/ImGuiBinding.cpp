// ImGuiBinding — state-bound ImGui widgets (see the header).
//
// The whole point is the write policy: continuous widgets (sliders/drags) edit a
// per-path cache every frame and commit to cvc::state only when the edit ends,
// so a drag costs ONE state write instead of one per frame. State writes fan out
// to observers and replicated peers, so per-frame writes are not free.

#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiBinding.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/Settings.h>
#include <cvc/gl/StageLighting.h>

#ifdef CVC_ENABLE_IMGUI

#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#define IMGUI_DEFINE_MATH_OPERATORS // must precede imgui.h (imgui_internal.h asserts it)
#include <imgui.h>
#include <imgui_internal.h> // ImHashStr: stable per-path key into the context's storage
#include <stdexcept>

namespace cvc {
namespace gl {
namespace ui {

namespace {

// Read a state value, seeding it with `def` when the path has no value yet.
template <typename T> T read_or_seed(cvc::app &ctx, const std::string &path, const T &def) {
  try {
    cvc::state &s = cvc::state::instance(ctx)(path);
    const std::string raw = s.value();
    if (raw.empty()) {
      s.value(def);
      return def;
    }
    return s.value<T>();
  } catch (const std::exception &) {
    return def; // unreadable/unconvertible: fall back, never throw into a frame
  }
}

template <typename T> void write(cvc::app &ctx, const std::string &path, const T &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
    // read-only or otherwise unwritable — the widget just won't stick.
  }
}

// Per-path edit cache for continuous widgets, held in ImGui's storage rather than
// a file-static map: a static would be a process-wide singleton shared by every
// viewer and every context, and it would outlive them.
//
// GetStateStorage() is the current WINDOW's storage, not the context's
// (imgui_internal.h: "Current persistent per-window storage", backed by
// ImGuiWindow::StateStorage) — so it lives and dies with the window, and the same
// state path drawn in two windows keeps two independent caches. That is fine for
// an in-progress drag, which is what this holds.
//
// It does mean ImGui's OWN keys share this map — a CollapsingHeader's open flag
// is stored in the same window under ImHashStr of its label — so the key is
// salted to keep a state path from ever landing on one of them.
//
// The cache holds the in-progress edit; while the widget is NOT being dragged it
// follows state, so an external write (script, config, replicated peer) moves
// the widget. Returns a pointer into ImGui's storage, stable for the frame.
// Salt for the cache keys — anything nonzero that ImGui itself will not use.
constexpr ImGuiID kCacheSalt = 0x63766367; // 'cvcg'

float *cache_float(const std::string &path, float seed, bool active) {
  ImGuiStorage *store = ImGui::GetStateStorage();
  const ImGuiID key = ImHashStr(path.c_str(), path.size(), kCacheSalt);
  float *slot = store->GetFloatRef(key, seed);
  if (!active)
    *slot = seed;
  return slot;
}
int *cache_int(const std::string &path, int seed, bool active) {
  ImGuiStorage *store = ImGui::GetStateStorage();
  const ImGuiID key = ImHashStr(path.c_str(), path.size(), kCacheSalt);
  int *slot = store->GetIntRef(key, seed);
  if (!active)
    *slot = seed;
  return slot;
}

} // namespace

bool SliderDouble(cvc::app &ctx, const char *label, const std::string &path, double lo, double hi,
                  double def, const char *fmt) {
  const double cur = read_or_seed<double>(ctx, path, def);
  float &v = *cache_float(path, static_cast<float>(cur), ImGui::IsAnyItemActive());
  const bool changed =
      ImGui::SliderFloat(label, &v, static_cast<float>(lo), static_cast<float>(hi), fmt);
  if (ImGui::IsItemDeactivatedAfterEdit())
    write<double>(ctx, path, v); // commit once, at the end of the drag
  return changed;
}

bool DragDouble(cvc::app &ctx, const char *label, const std::string &path, double speed, double def,
                const char *fmt) {
  const double cur = read_or_seed<double>(ctx, path, def);
  float &v = *cache_float(path, static_cast<float>(cur), ImGui::IsAnyItemActive());
  const bool changed = ImGui::DragFloat(label, &v, static_cast<float>(speed), 0.0f, 0.0f, fmt);
  if (ImGui::IsItemDeactivatedAfterEdit())
    write<double>(ctx, path, v);
  return changed;
}

bool SliderInt(cvc::app &ctx, const char *label, const std::string &path, int lo, int hi, int def) {
  const int cur = read_or_seed<int>(ctx, path, def);
  int &v = *cache_int(path, cur, ImGui::IsAnyItemActive());
  const bool changed = ImGui::SliderInt(label, &v, lo, hi);
  if (ImGui::IsItemDeactivatedAfterEdit())
    write<int>(ctx, path, v);
  return changed;
}

bool Checkbox(cvc::app &ctx, const char *label, const std::string &path, bool def) {
  bool v = read_or_seed<int>(ctx, path, def ? 1 : 0) != 0;
  if (ImGui::Checkbox(label, &v)) { // discrete: commit immediately
    write<int>(ctx, path, v ? 1 : 0);
    return true;
  }
  return false;
}

bool MenuItem(cvc::app &ctx, const char *label, const std::string &path, bool def) {
  bool v = read_or_seed<int>(ctx, path, def ? 1 : 0) != 0;
  if (ImGui::MenuItem(label, nullptr, &v)) {
    write<int>(ctx, path, v ? 1 : 0);
    return true;
  }
  return false;
}

bool Combo(cvc::app &ctx, const char *label, const std::string &path,
           const std::vector<std::string> &options, const std::string &def) {
  if (options.empty())
    return false;
  const std::string fallback = def.empty() ? options.front() : def;
  const std::string cur = read_or_seed<std::string>(ctx, path, fallback);
  int idx = 0;
  for (std::size_t i = 0; i < options.size(); ++i)
    if (options[i] == cur) {
      idx = static_cast<int>(i);
      break;
    }
  bool changed = false;
  if (ImGui::BeginCombo(label, options[idx].c_str())) {
    for (std::size_t i = 0; i < options.size(); ++i) {
      const bool sel = (static_cast<int>(i) == idx);
      if (ImGui::Selectable(options[i].c_str(), sel)) {
        write<std::string>(ctx, path, options[i]); // store the TEXT, not an index
        changed = true;
      }
      if (sel)
        ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
  return changed;
}

void Text(cvc::app &ctx, const char *label, const std::string &path) {
  std::string v;
  try {
    v = cvc::state::instance(ctx)(path).value();
  } catch (const std::exception &) {
    v = "<unset>";
  }
  ImGui::Text("%s: %s", label, v.c_str());
}

namespace {
// Commit-every-frame slider. The normal SliderDouble commits only on release,
// which is right for most settings but wrong for lighting: dragging the key
// azimuth with no visible change looks broken, and you cannot aim a light you
// cannot see move. Safe here because the rig now rebuilds its lights in ONE
// batched pass rather than one renderer-wide rebuild per light.
bool SliderLive(cvc::app &ctx, const char *label, const std::string &path, double lo, double hi,
                double def, const char *fmt = "%.3f") {
  float v = static_cast<float>(read_or_seed<double>(ctx, path, def));
  if (ImGui::SliderFloat(label, &v, static_cast<float>(lo), static_cast<float>(hi), fmt)) {
    write<double>(ctx, path, v);
    return true;
  }
  return false;
}
bool SliderIntLive(cvc::app &ctx, const char *label, const std::string &path, int lo, int hi,
                   int def) {
  int v = read_or_seed<int>(ctx, path, def);
  if (ImGui::SliderInt(label, &v, lo, hi)) {
    write<int>(ctx, path, v);
    return true;
  }
  return false;
}
} // namespace

// ---- StageLightingPanel ----------------------------------------------------
// Deliberately preset-first: most people want a LOOK, not sixteen sliders. The
// tooltips carry the one non-obvious fact — that the cone is the shadow-map
// frustum, so narrowing it is what sharpens shadows.
void StageLightingPanel(StageLighting &rig, bool *open, bool ownWindow) {
  // Honour `open` OURSELVES. ImGui::Begin(name, p_open) draws the close button
  // and clears *p_open when it is clicked, but it does NOT skip the window on
  // the next call — the caller is expected to. Callers reasonably assume a
  // library panel handles its own close box, and when it did not, clicking X
  // appeared to do nothing at all.
  if (open && !*open)
    return;
  if (ownWindow) {
    // Offset from the top-left so this does not open exactly on top of the
    // host demo's own window, which also defaults near the corner.
    const ImGuiViewport *vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 24.0f, vp->WorkPos.y + 120.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 0), ImGuiCond_FirstUseEver);
    // Same per-rig scoping as ScenePanel — see there.
    const std::string id = "Stage lighting###cvcgl.rig." + rig.statePath();
    if (!ImGui::Begin(id.c_str(), open)) {
      ImGui::End();
      return;
    }
  }

  ImGui::TextUnformatted("Preset");
  const struct {
    StageLighting::Preset p;
    const char *label;
    const char *tip;
  } kPresets[] = {
      {StageLighting::Preset::ThreePoint, "3-point", "Key + fill + back. The default."},
      {StageLighting::Preset::Overhead, "Overhead", "Even wash from above; few hard shadows."},
      {StageLighting::Preset::Dramatic, "Dramatic",
       "Narrow hard key, little fill. Sharpest shadows: a tight\n"
       "cone spends the whole shadow map on the subject."},
      {StageLighting::Preset::Flat, "Flat", "Wash only - read the geometry, not the mood."},
  };
  for (int i = 0; i < 4; ++i) {
    if (i)
      ImGui::SameLine();
    if (ImGui::Button(kPresets[i].label))
      rig.applyPreset(kPresets[i].p);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("%s", kPresets[i].tip);
  }

  ImGui::Separator();

  bool on = rig.enabled();
  if (ImGui::Checkbox("Rig on", &on))
    rig.setEnabled(on);
  ImGui::SameLine();
  const int casters = rig.shadowCasterCount();
  ImGui::TextDisabled("%d shadow caster%s", casters, casters == 1 ? "" : "s");
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Each shadow-casting light re-renders the whole scene\n"
                      "depth every bake. Fill and wash cast nothing on purpose.");

  bool giz = rig.gizmosVisible();
  if (ImGui::Checkbox("Show lights", &giz))
    rig.setGizmosVisible(giz);
  // Must sit directly under the Checkbox: IsItemHovered() names the LAST item
  // submitted, so below the conditional slider it described the wrong widget
  // whenever gizmos were on.
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Draw each light as a fixture, an aim line and its CONE.\n"
                      "The cone is the shadow-map frustum VTK bakes, so it shows\n"
                      "exactly what that light can shadow - and whether the map\n"
                      "is being spent on empty space.");
  if (giz)
    SliderLive(rig.appContext(), "Beam alpha", rig.statePath() + ".gizmo_beam_alpha", 0.0, 1.0,
               0.18, "%.2f");

  cvc::app &ctx = rig.appContext();
  const std::string p = rig.statePath() + ".";

  if (ImGui::CollapsingHeader("Roles", ImGuiTreeNodeFlags_DefaultOpen)) {
    SliderLive(ctx, "Key", p + "key_intensity", 0.0, 2.5, 1.0);
    SliderLive(ctx, "Fill", p + "fill_intensity", 0.0, 1.5, 0.35);
    SliderLive(ctx, "Back", p + "back_intensity", 0.0, 2.0, 0.55);
    SliderLive(ctx, "Wash", p + "wash_intensity", 0.0, 2.0, 0.30);
    SliderIntLive(ctx, "Wash lights", p + "wash_count", 0, 8, 4);
  }

  if (ImGui::CollapsingHeader("Key angle & cone", ImGuiTreeNodeFlags_DefaultOpen)) {
    SliderLive(ctx, "Azimuth", p + "key_azimuth", -180.0, 180.0, -50.0, "%.0f deg");
    SliderLive(ctx, "Elevation", p + "key_elevation", 5.0, 85.0, 38.0, "%.0f deg");
    SliderLive(ctx, "Cone", p + "key_cone", 8.0, 70.0, 32.0, "%.0f deg");
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("The cone IS the shadow-map frustum.\n"
                        "Narrower = sharper, because the same map covers less ground.");
  }

  // Per-light switches. This is a DEBUGGING surface: turn lights off one at a
  // time to attribute a shadow artifact to the light that casts it.
  if (ImGui::CollapsingHeader("Lights (debug)")) {
    const auto names = rig.lightNames();
    if (names.empty())
      ImGui::TextDisabled("rig is off");
    for (const auto &n : names) {
      bool on = rig.lightEnabled(n);
      if (ImGui::Checkbox(n.c_str(), &on))
        rig.setLightEnabled(n, on);
      ImGui::SameLine();
      ImGui::PushID(n.c_str());
      if (ImGui::SmallButton("solo"))
        rig.soloLight(n);
      ImGui::PopID();
    }
    if (!names.empty() && ImGui::Button("All on"))
      rig.soloLight("");
    ImGui::TextDisabled("off = light AND its shadow map removed");
  }

  if (ImGui::CollapsingHeader("Look")) {
    SliderLive(ctx, "Environment", p + "env_intensity", 0.0, 1.5, 0.30);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Shadow-free fill for everything OUTSIDE the cones.\nWater needs this: a "
                        "highlight can only show where the surface is lit.");
    SliderLive(ctx, "Ambient", p + "ambient", 0.0, 0.8, 0.22);
    SliderLive(ctx, "Warm key", p + "warm_key", 0.0, 1.0, 0.35);
    SliderDouble(ctx, "Stage radius", p + "stage_radius", 1.0, 500.0, 10.0);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Size of the ACTING AREA, not of the scene.\n"
                        "Sizing this to the whole scene is what makes shadows mushy.");
  }

  if (ownWindow)
    ImGui::End();
}

void SceneMenuItems(SceneGraph &sg, bool *scenePanelOpen, bool *lightingPanelOpen) {
  // Driven through the scene, not through its state path, for two reasons that
  // both end in a control that quietly does nothing.
  //
  // SceneGraph builds its ShadowSettings LAZILY, inside syncShadowState(), which
  // only ever runs from a shadow setter. Until one has been called there is no
  // subscriber on <prefix>.shadows.*, so writing the path is a no-op with no
  // error and no log: `bunny_shadow --no-shadows` short-circuits
  // setShadowsEnabled() entirely, and every shadow control here would be inert
  // for the whole run.
  //
  // And setShadowsEnabled() can refuse — it returns false when there is no
  // render target yet — without writing that refusal back to state, so a
  // state-bound tick would read ON over a scene that has no shadows. Reading
  // the scene back is what actually stops the tick lying; state-binding does
  // not. The setters still publish to state, so Python and replicated peers see
  // every change exactly as before.
  bool shadowsOn = sg.shadowsEnabled();
  if (ImGui::MenuItem("Shadows", nullptr, &shadowsOn))
    sg.setShadowsEnabled(shadowsOn);

  ImGui::Separator();

  // Chrome reads its own state back — there is no second copy to desync.
  bool grid = sg.gridVisible();
  if (ImGui::MenuItem("Grid", nullptr, &grid))
    sg.setGridVisible(grid);
  bool axis = sg.axisVisible();
  if (ImGui::MenuItem("Origin axis", nullptr, &axis))
    sg.setAxisVisible(axis);

  // Scene-wide, unlike GraphicsNode::setShowBBox which is per-node. These read
  // back as "is any box drawn?", which is what lets them be ticks at all: they
  // were a pair of on/off actions each while SceneGraph had no getter.
  bool boxes = sg.bboxesVisible();
  if (ImGui::MenuItem("Bounding boxes", nullptr, &boxes))
    sg.setBBoxesVisible(boxes);
  // The labels are drawn BY the bbox node, so with boxes off they are invisible
  // whatever this says. Disabled rather than silently ticked-but-not-showing.
  bool labels = sg.extentLabelsVisible();
  if (ImGui::MenuItem("Extent labels", nullptr, &labels, boxes))
    sg.setExtentLabelsVisible(labels);
  if (!boxes && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip("Drawn by the bounding boxes - turn those on first.");

  ImGui::Separator();
  // Master toggle. Ticked when ANY of the four is showing, so clearing it always
  // means "get all of this off my screen" and setting it brings everything back.
  bool chrome = sg.diagnosticChromeVisible();
  if (ImGui::MenuItem("All chrome", nullptr, &chrome))
    sg.setDiagnosticChromeVisible(chrome);

  if (scenePanelOpen || lightingPanelOpen)
    ImGui::Separator();
  if (scenePanelOpen)
    ImGui::MenuItem("Scene panel", nullptr, scenePanelOpen);
  if (lightingPanelOpen)
    ImGui::MenuItem("Stage lighting", nullptr, lightingPanelOpen);
}

void CameraMenuItems(CameraController &cam, double moveSpeedMax, double moveSpeedDefault) {
  cvc::app &ctx = cam.appContext();
  // stateName() is the controller's OWN path, so this cannot drift from
  // CameraController::viewerStatePath(). Three demos spliced
  // `prefix + ".viewers.main.camera.settings."` as a literal instead.
  const std::string p = cam.stateName("settings") + ".";

  SliderDouble(ctx, "Look sens", p + "mouse_sensitivity", 0.02, 2.0, 0.25);
  SliderDouble(ctx, "Move speed", p + "move_speed", 1.0, moveSpeedMax, moveSpeedDefault);
  ui::Checkbox(ctx, "Invert pitch", p + "invert_pitch");
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Flip vertical look. Applies to fly AND orbit.");

  // Read off the controller rather than state: it is the object that acts on the
  // value, and it already publishes every change back to that path.
  bool pans = cam.primaryDragPans();
  if (ImGui::MenuItem("Drag pans", nullptr, &pans))
    cam.setPrimaryDragPans(pans);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Left-drag slides the view instead of turning it.\n"
                      "The middle button always pans regardless.");
}

void ScenePanel(SceneGraph &sg, bool *open, bool ownWindow) {
  // Same close-box discipline as StageLightingPanel: ImGui clears *p_open but
  // does not skip the window next frame, so honour it here.
  if (open && !*open)
    return;
  if (ownWindow) {
    const ImGuiViewport *vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + 388.0f, vp->WorkPos.y + 120.0f),
                            ImGuiCond_FirstUseEver);
    // 400, not the lighting panel's 340: the resolution radio row and the
    // chrome buttons are laid out on one line each and clip below this.
    ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_FirstUseEver);
    // "###" keeps the visible title "Scene" while making the window ID unique per
    // scene. SceneGraph deliberately has no default ctor because a process can
    // run several; with a fixed name they all drew into ONE merged window.
    const std::string id = "Scene###cvcgl.scene." + sg.getStatePrefix();
    if (!ImGui::Begin(id.c_str(), open)) {
      ImGui::End();
      return;
    }
  }

  // Same as SceneMenuItems: through the scene, not through the state path, so
  // the controls work on a scene that has never had a shadow setter called and
  // the tick cannot outlive a refused enable.
  ImGui::TextUnformatted("Shadows");
  bool shadowsOn = sg.shadowsEnabled();
  if (ImGui::Checkbox("Enabled", &shadowsOn))
    sg.setShadowsEnabled(shadowsOn);
  // Resolution is the knob you actually reach for when a shadow looks wrong:
  // a tight light cone spends the whole map on the subject, a wide one wastes
  // it. Powers of two only — the map is square and allocated per light.
  static const int kRes[] = {512, 1024, 2048, 4096};
  int res = sg.shadowResolution();
  ImGui::TextUnformatted("Map resolution");
  for (int i = 0; i < 4; ++i) {
    if (i)
      ImGui::SameLine();
    char lbl[16];
    std::snprintf(lbl, sizeof(lbl), "%d", kRes[i]);
    if (ImGui::RadioButton(lbl, res == kRes[i]))
      sg.setShadowResolution(kRes[i]);
  }
  int interval = sg.shadowUpdateInterval();
  if (ImGui::SliderInt("Bake every", &interval, 1, 30))
    sg.setShadowUpdateInterval(interval);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Re-bake the shadow maps every N frames.\n"
                      "Raise it when the lights and the geometry are both still — the bake"
                      " is a whole extra scene pass per shadow-casting light.");

  ImGui::Separator();
  ImGui::TextUnformatted("Diagnostic chrome");
  bool grid = sg.gridVisible();
  if (ImGui::Checkbox("Grid", &grid))
    sg.setGridVisible(grid);
  ImGui::SameLine();
  bool axis = sg.axisVisible();
  if (ImGui::Checkbox("Origin axis", &axis))
    sg.setAxisVisible(axis);
  bool boxes = sg.bboxesVisible();
  if (ImGui::Checkbox("Bounding boxes", &boxes))
    sg.setBBoxesVisible(boxes);
  ImGui::SameLine();
  // Same dependency as the menu: labels belong to the bbox node.
  bool labels = sg.extentLabelsVisible();
  ImGui::BeginDisabled(!boxes);
  if (ImGui::Checkbox("Extent labels", &labels))
    sg.setExtentLabelsVisible(labels);
  ImGui::EndDisabled();
  if (!boxes && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip("Drawn by the bounding boxes - turn those on first.");
  bool chrome = sg.diagnosticChromeVisible();
  if (ImGui::Checkbox("All of it", &chrome))
    sg.setDiagnosticChromeVisible(chrome);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Ticked while ANY of the four is showing. Clearing it takes "
                      "grid, axis, boxes and labels off in one go.");

  if (ownWindow)
    ImGui::End();
}

} // namespace ui
} // namespace gl
} // namespace cvc

#else // !CVC_ENABLE_IMGUI — inert stubs

namespace cvc {
namespace gl {
namespace ui {

bool SliderDouble(cvc::app &, const char *, const std::string &, double, double, double,
                  const char *) {
  return false;
}
bool DragDouble(cvc::app &, const char *, const std::string &, double, double, const char *) {
  return false;
}
bool SliderInt(cvc::app &, const char *, const std::string &, int, int, int) { return false; }
bool Checkbox(cvc::app &, const char *, const std::string &, bool) { return false; }
bool MenuItem(cvc::app &, const char *, const std::string &, bool) { return false; }
bool Combo(cvc::app &, const char *, const std::string &, const std::vector<std::string> &,
           const std::string &) {
  return false;
}
void Text(cvc::app &, const char *, const std::string &) {}
void StageLightingPanel(StageLighting &, bool *, bool) {}
void ScenePanel(SceneGraph &, bool *, bool) {}
void SceneMenuItems(SceneGraph &, bool *, bool *) {}
void CameraMenuItems(CameraController &, double, double) {}

} // namespace ui
} // namespace gl
} // namespace cvc

#endif // CVC_ENABLE_IMGUI
