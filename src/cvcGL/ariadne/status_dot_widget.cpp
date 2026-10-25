/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstdio>
#include <cvc/ariadne/widget.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/status_dot_widget.h>
#include <string>

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace cvc {
namespace gl {
namespace ariadne {

#ifdef CVC_ENABLE_IMGUI
namespace {

// COLOUR's RGB: a name, or "r,g,b" in 0..1. False for anything else.
bool dot_rgb(const std::string &colour, float &r, float &g, float &b) {
  struct named {
    const char *name;
    float r, g, b;
  };
  static const named kNamed[] = {{"green", 0.25f, 0.85f, 0.35f},
                                 {"red", 0.92f, 0.22f, 0.20f},
                                 {"black", 0.0f, 0.0f, 0.0f},
                                 {"amber", 0.98f, 0.70f, 0.15f},
                                 {"grey", 0.55f, 0.55f, 0.55f}};
  for (const named &n : kNamed)
    if (colour == n.name) {
      r = n.r, g = n.g, b = n.b;
      return true;
    }
  return std::sscanf(colour.c_str(), "%f,%f,%f", &r, &g, &b) == 3;
}

} // namespace
#endif

void register_status_dot_widget(ImGuiBackend &backend) {
#ifdef CVC_ENABLE_IMGUI
  backend.register_widget(
      "status_dot",
      [](const std::string &current, const cvc::ariadne::Widget &w) -> cvc::ariadne::CustomEdit {
        float d = static_cast<float>(w.props.num("size", 10.0));
        if (!(d > 0.0f) || d > 64.0f)
          d = 10.0f;
        const float line = ImGui::GetTextLineHeight();
        const ImVec2 at = ImGui::GetCursorScreenPos();
        // "COLOUR" or "COLOUR TOOLTIP...".
        const std::size_t space = current.find(' ');
        const std::string colour = current.substr(0, space);
        float r = 0.0f, g = 0.0f, b = 0.0f;
        const bool shown = !colour.empty() && dot_rgb(colour, r, g, b);
        if (shown) {
          ImDrawList *dl = ImGui::GetWindowDrawList();
          const ImVec2 c(at.x + d * 0.5f, at.y + line * 0.5f);
          dl->AddCircleFilled(c, d * 0.5f, ImGui::GetColorU32(ImVec4(r, g, b, 1.0f)), 16);
          // A thin rim, so black still shows on a dark window.
          dl->AddCircle(c, d * 0.5f, ImGui::GetColorU32(ImVec4(0.6f, 0.6f, 0.6f, 1.0f)), 16, 1.0f);
        }
        ImGui::Dummy(ImVec2(d, line)); // its space either way, so the row's cells stay lined up
        if (shown && space != std::string::npos && ImGui::IsItemHovered())
          ImGui::SetTooltip("%s", current.c_str() + space + 1);
        return {};
      });
#else
  (void)backend;
#endif
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
