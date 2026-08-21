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
#include <imgui.h>
#include <map>
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

// Per-path edit cache for continuous widgets. Keyed by path so two widgets on
// the same path stay consistent; entries are tiny and bounded by the UI's size.
template <typename T> T &cache_for(const std::string &path, const T &seed, bool active) {
  static std::map<std::string, T> cache;
  auto it = cache.find(path);
  if (it == cache.end())
    it = cache.emplace(path, seed).first;
  else if (!active)
    it->second = seed; // not being dragged: follow state (scripts can move it)
  return it->second;
}

} // namespace

bool SliderDouble(cvc::app &ctx, const char *label, const std::string &path, double lo, double hi,
                  double def, const char *fmt) {
  const double cur = read_or_seed<double>(ctx, path, def);
  float &v = cache_for<float>(path, static_cast<float>(cur), ImGui::IsAnyItemActive());
  const bool changed =
      ImGui::SliderFloat(label, &v, static_cast<float>(lo), static_cast<float>(hi), fmt);
  if (ImGui::IsItemDeactivatedAfterEdit())
    write<double>(ctx, path, v); // commit once, at the end of the drag
  return changed;
}

bool DragDouble(cvc::app &ctx, const char *label, const std::string &path, double speed, double def,
                const char *fmt) {
  const double cur = read_or_seed<double>(ctx, path, def);
  float &v = cache_for<float>(path, static_cast<float>(cur), ImGui::IsAnyItemActive());
  const bool changed = ImGui::DragFloat(label, &v, static_cast<float>(speed), 0.0f, 0.0f, fmt);
  if (ImGui::IsItemDeactivatedAfterEdit())
    write<double>(ctx, path, v);
  return changed;
}

bool SliderInt(cvc::app &ctx, const char *label, const std::string &path, int lo, int hi, int def) {
  const int cur = read_or_seed<int>(ctx, path, def);
  int &v = cache_for<int>(path, cur, ImGui::IsAnyItemActive());
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
