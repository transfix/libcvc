/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_ARIADNE_STATUS_DOT_WIDGET_H__
#define __CVC_GL_ARIADNE_STATUS_DOT_WIDGET_H__

// Ariadne "status_dot" widget: a small filled circle whose colour comes from state -- a status
// light beside a name (the conference roster shows each person's video rung: green full, red
// reduced, black none). It is drawn with ImGui's draw list, so it needs no font glyph (the web
// build's UI font is ASCII-only, and a "●" there draws as "?"). Declare it in a DSL document:
//
//   - type: status_dot
//     bind: <state key>   # "COLOUR" or "COLOUR TOOLTIP...", e.g. "red half video"
//     size: 10            # diameter in px (default 10)
//
// COLOUR is green, red, black, amber or grey, or "r,g,b" in 0..1. The words after it (if any) show
// as a tooltip on hover. An empty value draws nothing but keeps the dot's space, so a row's cells
// stay lined up.

namespace cvc {
namespace gl {

class ImGuiBackend;

namespace ariadne {

// Register the "status_dot" type on `backend` (call once, before the document renders).
void register_status_dot_widget(ImGuiBackend &backend);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_STATUS_DOT_WIDGET_H__
