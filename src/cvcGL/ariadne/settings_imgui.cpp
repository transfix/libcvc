/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// ImGuiBackend::settings — Kind::Settings as ImGui widgets. Top to bottom: the label, the search
// box, then the nodes in the core's order: a branch under the root is a collapsing header and a
// deeper one a tree node, each with a "reset all" at the right of its line while a key under it can
// be reset; a key is a row of its name, its editor from the name column (42% of the width) to the
// reset's place at the right, and a "reset" while it is not its default. Hovering a name, an editor
// or a header shows its tooltip. A slider's value is held here while it is dragged (so `commit:
// release` shows the drag before the write); a text field holds its text until Enter. The groups
// open and close by ImGui's own state, every one open while a search or a filter narrows the list.
// The core owns every cvc::state read and write.

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h> // to_bool_int: a flag as the core reads it
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <string>
#include <vector>

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace cvc {
namespace gl {

#ifdef CVC_ENABLE_IMGUI
namespace {

namespace ari = cvc::ariadne;

// A number as the shortest text of up to 17 digits that reads back as the same double.
std::string number_text(double v) {
  char b[40];
  std::snprintf(b, sizeof b, "%.15g", v);
  if (std::strtod(b, nullptr) != v)
    std::snprintf(b, sizeof b, "%.17g", v);
  return b;
}

double number_of(const std::string &s) { return s.empty() ? 0.0 : std::strtod(s.c_str(), nullptr); }

int int_of(double d) {
  if (!std::isfinite(d))
    return 0;
  return static_cast<int>(std::clamp(std::round(d), static_cast<double>(INT_MIN) + 1.0,
                                     static_cast<double>(INT_MAX) - 1.0));
}

// InputText into a std::string, grown as the user types (as ImGuiBackend::input_text does).
struct TextBuf {
  std::string *s;
};
int text_resize(ImGuiInputTextCallbackData *d) {
  if (d->EventFlag == ImGuiInputTextFlags_CallbackResize) {
    std::string *s = static_cast<TextBuf *>(d->UserData)->s;
    s->resize(static_cast<std::size_t>(d->BufTextLen));
    d->Buf = s->data();
  }
  return 0;
}
bool input_string(const char *id, const char *hint, std::string &buf, ImGuiInputTextFlags flags) {
  TextBuf ud{&buf};
  flags |= ImGuiInputTextFlags_CallbackResize;
  return hint ? ImGui::InputTextWithHint(id, hint, buf.data(), buf.capacity() + 1, flags,
                                         text_resize, &ud)
              : ImGui::InputText(id, buf.data(), buf.capacity() + 1, flags, text_resize, &ud);
}

// The last item's tooltip, wrapped, while it is hovered (a disabled one too).
void tooltip(const std::string &text) {
  if (text.empty() || !ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    return;
  ImGui::BeginTooltip();
  ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
  ImGui::TextUnformatted(text.c_str());
  ImGui::PopTextWrapPos();
  ImGui::EndTooltip();
}

void last_rect(float r[4]) {
  const ImVec2 a = ImGui::GetItemRectMin(), z = ImGui::GetItemRectMax();
  r[0] = a.x;
  r[1] = a.y;
  r[2] = z.x;
  r[3] = z.y;
}

} // namespace
#endif

ariadne::SettingsEdit ImGuiBackend::settings(const char *label, const char *id,
                                             const ariadne::SettingsSpec &spec) {
  ariadne::SettingsEdit out;
#ifdef CVC_ENABLE_IMGUI
  const ImGuiStyle &st = ImGui::GetStyle();
  if (label && *label)
    ImGui::TextUnformatted(label);
  if (spec.search) {
    // The text follows the search in state while nothing is being edited, and each keystroke
    // narrows the list.
    std::string &buf = m_textEdits[std::string(id ? id : "") + ".search"];
    if (!ImGui::IsAnyItemActive())
      buf = spec.search_text;
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (input_string("##search", "search", buf, ImGuiInputTextFlags_None) &&
        buf != spec.search_text) {
      out.search_changed = true;
      out.search = buf;
    }
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Show the keys with each word in their path or description\n(any case; "
                        "-word leaves out the keys that have it)");
  }

  // Window-local x: where the rows start, the editors' column, and the right edge, kept a whole
  // pixel in: at a fractional UI scale an item ending on the edge itself rounds past it and gives
  // the window a horizontal scrollbar.
  const float x0 = ImGui::GetCursorPosX();
  const float avail = ImGui::GetContentRegionAvail().x;
  const float right = std::floor(x0 + avail) - 1.0f;
  const float col = x0 + 0.42f * avail;
  const float reset_w = ImGui::CalcTextSize("reset").x + 2.0f * st.FramePadding.x;
  const float reset_all_w = ImGui::CalcTextSize("reset all").x + 2.0f * st.FramePadding.x;

  int skip_deeper = INT_MAX; // a closed group's depth: the nodes under it are not drawn
  std::vector<int> trees;    // the depths of the tree nodes open now, popped when left
  for (std::size_t i = 0; i < spec.nodes.size(); ++i) {
    const ariadne::SettingsNode &n = spec.nodes[i];
    const int at = static_cast<int>(i);
    while (!trees.empty() && trees.back() >= n.depth) {
      ImGui::TreePop();
      trees.pop_back();
    }
    if (n.depth > skip_deeper)
      continue;
    skip_deeper = INT_MAX;

    if (n.group) {
      if (spec.narrowed)
        ImGui::SetNextItemOpen(true, ImGuiCond_Always);
      else if (n.open)
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
      bool shown = false;
      if (n.depth == 0) {
        const std::string text = n.name + "###" + n.id;
        shown = ImGui::CollapsingHeader(text.c_str(), ImGuiTreeNodeFlags_AllowOverlap);
      } else {
        shown =
            ImGui::TreeNodeEx(n.id.c_str(), ImGuiTreeNodeFlags_AllowOverlap, "%s", n.name.c_str());
        if (shown)
          trees.push_back(n.depth);
      }
      tooltip(n.tooltip);
      if (n.resettable > 0) {
        ImGui::SameLine(std::max(right - reset_all_w, x0));
        ImGui::PushID(n.id.c_str());
        if (ImGui::SmallButton("reset all"))
          out.reset = at;
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("%d %s under %s back to %s default", n.resettable,
                            n.resettable == 1 ? "key" : "keys", n.id.c_str(),
                            n.resettable == 1 ? "its" : "their");
        ImGui::PopID();
      }
      if (!shown)
        skip_deeper = n.depth;
      continue;
    }

    ImGui::PushID(n.path.c_str());
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(n.name.c_str());
    tooltip(n.tooltip);
    ImGui::SameLine(std::max(col, ImGui::GetCursorPosX()));
    ImGui::SetNextItemWidth(
        std::max(right - reset_w - st.ItemSpacing.x - ImGui::GetCursorPosX(), 48.0f));
    float editor[4] = {0.0f, 0.0f, 0.0f, 0.0f}, reset[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const auto edit = [&](std::string v, bool changed, bool committed) {
      out.node = at;
      out.value = std::move(v);
      out.changed = changed;
      out.committed = committed;
    };
    switch (n.kind) {
    case ariadne::SettingKind::Readout:
      if (n.unit.empty() || n.unit == "1")
        ImGui::TextDisabled("%s", n.value.c_str());
      else
        ImGui::TextDisabled("%s %s", n.value.c_str(), n.unit.c_str());
      break;
    case ariadne::SettingKind::Toggle: {
      bool v = ariadne::to_bool_int(n.value, 0) != 0;
      if (ImGui::Checkbox("##v", &v))
        edit(v ? "1" : "0", true, true);
      break;
    }
    case ariadne::SettingKind::Int: {
      double &held = m_settingNumbers[n.path];
      if (!ImGui::IsAnyItemActive())
        held = number_of(n.value);
      int v = int_of(held);
      const bool changed =
          n.has_range ? ImGui::SliderInt("##v", &v, int_of(n.min), int_of(n.max), n.fmt.c_str(),
                                         n.log ? ImGuiSliderFlags_Logarithmic : 0)
                      : ImGui::DragInt("##v", &v, 1.0f, 0, 0, n.fmt.c_str());
      held = v;
      const bool committed = ImGui::IsItemDeactivatedAfterEdit();
      if (changed || committed)
        edit(std::to_string(v), changed, committed);
      break;
    }
    case ariadne::SettingKind::Number: {
      double &held = m_settingNumbers[n.path];
      if (!ImGui::IsAnyItemActive())
        held = number_of(n.value);
      double v = held;
      const bool changed =
          n.has_range ? ImGui::SliderScalar("##v", ImGuiDataType_Double, &v, &n.min, &n.max,
                                            n.fmt.c_str(), n.log ? ImGuiSliderFlags_Logarithmic : 0)
                      : ImGui::DragScalar("##v", ImGuiDataType_Double, &v,
                                          static_cast<float>(std::max(1e-3, std::fabs(v) * 0.01)),
                                          nullptr, nullptr, n.fmt.c_str());
      held = v;
      const bool committed = ImGui::IsItemDeactivatedAfterEdit();
      if (changed || committed)
        edit(number_text(v), changed, committed);
      break;
    }
    case ariadne::SettingKind::Text: {
      // Applied on Enter; leaving the field without it puts the value in state back.
      std::string &buf = m_textEdits[n.path];
      if (!ImGui::IsAnyItemActive())
        buf = n.value;
      if (input_string("##v", nullptr, buf, ImGuiInputTextFlags_EnterReturnsTrue) && buf != n.value)
        edit(buf, true, true);
      break;
    }
    case ariadne::SettingKind::Choice:
      if (ImGui::BeginCombo("##v", n.value.c_str())) {
        for (const std::string &c : n.choices) {
          const bool sel = c == n.value;
          if (ImGui::Selectable(c.c_str(), sel) && !sel)
            edit(c, true, true);
          if (sel)
            ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
      }
      break;
    }
    last_rect(editor);
    tooltip(n.tooltip);
    if (n.changed && n.kind != ariadne::SettingKind::Readout) {
      ImGui::SameLine(std::max(right - reset_w, ImGui::GetCursorPosX()));
      if (ImGui::SmallButton("reset"))
        out.reset = at;
      last_rect(reset);
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("back to %s", n.def.empty() ? "(empty)" : n.def.c_str());
    }
    ImGui::PopID();
    settings_row_drawn(n, editor, reset);
  }
  while (!trees.empty()) {
    ImGui::TreePop();
    trees.pop_back();
  }
  if (spec.nodes.empty() && !spec.empty.empty())
    ImGui::TextDisabled("%s", spec.empty.c_str());
#else
  (void)label;
  (void)id;
  (void)spec;
#endif
  return out;
}

} // namespace gl
} // namespace cvc
