// ImGuiBinding — state-bound ImGui widgets (see the header).
//
// The whole point is the write policy: continuous widgets (sliders/drags) edit a
// per-path cache every frame and commit to cvc::state only when the edit ends,
// so a drag costs ONE state write instead of one per frame. State writes fan out
// to observers and replicated peers, so per-frame writes are not free.

#include <cvc/gl/ImGuiBinding.h>

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

} // namespace ui
} // namespace gl
} // namespace cvc

#endif // CVC_ENABLE_IMGUI
