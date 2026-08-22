// ImGuiBinding — state-bound ImGui widgets (see the header).
//
// The whole point is the write policy: continuous widgets (sliders/drags) edit a
// per-path cache every frame and commit to cvc::state only when the edit ends,
// so a drag costs ONE state write instead of one per frame. State writes fan out
// to observers and replicated peers, so per-frame writes are not free.

#include <cvc/gl/ImGuiBinding.h>
#include <cvc/gl/StageLighting.h>

#ifdef CVC_ENABLE_IMGUI

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

// Per-path edit cache for continuous widgets, held in the ImGui CONTEXT's own
// storage (ImGui::GetStateStorage) rather than a file-static map: a static would
// be a process-wide singleton shared by every viewer and every ImGui context,
// and it would outlive them. This lives and dies with the context that owns the
// widget, and two viewers cannot collide.
//
// The cache holds the in-progress edit; while the widget is NOT being dragged it
// follows state, so an external write (script, config, replicated peer) moves
// the widget. Returns a pointer into ImGui's storage, stable for the frame.
float *cache_float(const std::string &path, float seed, bool active) {
  ImGuiStorage *store = ImGui::GetStateStorage();
  const ImGuiID key = ImHashStr(path.c_str(), path.size());
  float *slot = store->GetFloatRef(key, seed);
  if (!active)
    *slot = seed;
  return slot;
}
int *cache_int(const std::string &path, int seed, bool active) {
  ImGuiStorage *store = ImGui::GetStateStorage();
  const ImGuiID key = ImHashStr(path.c_str(), path.size());
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
    if (!ImGui::Begin("Stage lighting", open)) {
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
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Draw each light as a fixture, an aim line and its CONE.\n"
                      "The cone is the shadow-map frustum VTK bakes, so it shows\n"
                      "exactly what that light can shadow - and whether the map\n"
                      "is being spent on empty space.");

  cvc::app &ctx = rig.appContext();
  const std::string p = rig.statePath() + ".";

  if (ImGui::CollapsingHeader("Roles", ImGuiTreeNodeFlags_DefaultOpen)) {
    SliderDouble(ctx, "Key", p + "key_intensity", 0.0, 2.5, 1.0);
    SliderDouble(ctx, "Fill", p + "fill_intensity", 0.0, 1.5, 0.35);
    SliderDouble(ctx, "Back", p + "back_intensity", 0.0, 2.0, 0.55);
    SliderDouble(ctx, "Wash", p + "wash_intensity", 0.0, 2.0, 0.30);
    SliderInt(ctx, "Wash lights", p + "wash_count", 0, 8, 4);
  }

  if (ImGui::CollapsingHeader("Key angle & cone", ImGuiTreeNodeFlags_DefaultOpen)) {
    SliderDouble(ctx, "Azimuth", p + "key_azimuth", -180.0, 180.0, -50.0);
    SliderDouble(ctx, "Elevation", p + "key_elevation", 5.0, 85.0, 38.0);
    SliderDouble(ctx, "Cone", p + "key_cone", 8.0, 70.0, 32.0);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("The cone IS the shadow-map frustum.\n"
                        "Narrower = sharper, because the same map covers less ground.");
  }

  if (ImGui::CollapsingHeader("Look")) {
    SliderDouble(ctx, "Ambient", p + "ambient", 0.0, 0.8, 0.22);
    SliderDouble(ctx, "Warm key", p + "warm_key", 0.0, 1.0, 0.35);
    SliderDouble(ctx, "Stage radius", p + "stage_radius", 1.0, 500.0, 10.0);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Size of the ACTING AREA, not of the scene.\n"
                        "Sizing this to the whole scene is what makes shadows mushy.");
  }

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

} // namespace ui
} // namespace gl
} // namespace cvc

#endif // CVC_ENABLE_IMGUI
