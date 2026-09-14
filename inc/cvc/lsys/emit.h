/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// emit.h — turn a derived word / structure into text outputs.
//
//   stats : module counts (per symbol, per level) — the determinism oracle.
//   svg   : a self-contained SVG (no deps) — makes a recipe reviewable in a diff.
//   off   : a Geomview .off mesh (cylinders + boxes) for a 3D look.
//
// The SVG element count is exactly segments + leaves + boxes, so a test can
// cross-check the interpreted geometry counts against the rendered element
// count (two independent paths to the same number).

#ifndef CVC_LSYS_EMIT_H
#define CVC_LSYS_EMIT_H

#include <cstddef>
#include <cstdint>
#include <cvc/lsys/grammar.h>
#include <cvc/lsys/interp.h>
#include <cvc/lsys/module.h>
#include <map>
#include <string>

namespace cvc {
namespace lsys {

struct stats {
  std::size_t modules = 0;
  std::size_t segments = 0, leaves = 0, boxes = 0, paints = 0;
  std::array<std::uint32_t, max_levels> level_counts{};
  std::map<std::string, std::uint32_t> by_symbol; // symbol name -> count
};

stats compute_stats(const ruleset &rs, const word &w, const structure &s);

enum class projection { front, side, top };
enum class svg_color { order, level, klass };

struct svg_options {
  projection proj = projection::front;
  int width_px = 900;
  svg_color color = svg_color::klass;
  int margin_px = 20;
};

// Number of drawable elements a given structure produces in svg (== segments +
// leaves + boxes). Exposed so tests can assert without parsing the SVG.
std::size_t svg_element_count(const structure &s);

std::string emit_svg(const structure &s, const svg_options &opt = svg_options{});

// A Geomview .off mesh: each segment -> a short prism, each box -> a cuboid.
std::string emit_off(const structure &s, int sides = 5);

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_EMIT_H
