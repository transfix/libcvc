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

// symbol.h — interned symbol table for the L-system alphabet.
//
// A symbol is a name (a single-char operator like "+"/"["  or a multi-char
// module name like "F"/"Trans"/"Subdiv") interned to a small integer. Built-in
// alphabet symbols are registered at FIXED ids in every table, so the
// interpreter can switch on `builtin` values while user nonterminals (A, B, X…)
// get ids assigned after the built-ins and carry no geometric meaning.
//
// Tokenizing rule (see parse.cpp): every maximal [A-Za-z_][A-Za-z0-9_]* run is
// one identifier symbol (so "F" and "Trans" and "A" are all identifiers, and
// "R"/"T"/"S" are unambiguous single-letter identifiers), and each of the
// operator punctuation characters is its own single symbol.

#ifndef CVC_LSYS_SYMBOL_H
#define CVC_LSYS_SYMBOL_H

#include <cstdint>
#include <cvc/lsys/module.h> // symbol_t
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace lsys {

// Fixed ids for the built-in alphabet. Values are stable (serialized indirectly
// through the interpreter's switch); append new symbols before `_first_user`.
enum class builtin : symbol_t {
  // motion (turtle)
  fwd = 0, // "F" draw a segment forward (+local Y) of length p0 (or step)
  move,    // "f" move forward without drawing
  fwd_g,   // "G" draw forward, no polygon vertex
  // rotation (degrees, converted to radians once at parse)
  yaw_pos,  // "+"
  yaw_neg,  // "-"
  pitch_dn, // "&"
  pitch_up, // "^"
  roll_pos, // "/"
  roll_neg, // "\\"
  turn_180, // "|"
  relevel,  // "$"  Honda re-level: roll so the local left is horizontal
  // legacy plant tilt/roll aliases used by the extracted forest grammar
  tilt, // "T"  pitch down by the grammar tilt angle
  roll, // "R"  roll by the grammar roll angle
  leaf, // "L"  emit a leaf/foliage marker at the current scope
  // state
  set_width, // "!"  set line width p0
  set_class, // ";"  set current surface class id p0
  adv_color, // "'"  advance the colour index
  // structure
  push, // "["
  pop,  // "]"
  cut,  // "%"  prune: delete the rest of this branch (non-nested)
  // polygon
  poly_open,  // "{"
  poly_point, // "."
  poly_close, // "}"
  // scope grammar (buildings / rocks) — Müller CGA-style
  s_trans,  // "Trans"(x,y,z)  translate the scope along its own axes
  s_scale,  // "Scale"(x,y,z)  scale the scope size vector
  s_rot,    // "Rot"(ax,ay,az) rotate the scope frame (degrees)
  s_subdiv, // "Subdiv"(axis, n) split into n equal children along axis
  s_repeat, // "Repeat"(axis, d) tile children of size d along axis
  s_box,    // "Box"(class)     emit a filled box the size of the scope
  s_inst,   // "Inst"(asset)    instance a named sub-asset
  // paint terminals (write the material/class raster in one walk)
  paint_disc, // "P"(class, r)
  paint_wide, // "Pw"(class, w)  paint a capsule of half-width w along the path
  paint_band, // "Pb"(class, w, f)
  stamp2d,    // "Stamp2D"(class, rect...)
  _first_user // user nonterminals start here
};

constexpr symbol_t builtin_id(builtin b) noexcept { return static_cast<symbol_t>(b); }

class symbol_table {
public:
  symbol_table() { register_builtins(); }

  // Intern a name, assigning a new id if unseen. Returns the symbol id.
  symbol_t intern(const std::string &name) {
    auto it = by_name_.find(name);
    if (it != by_name_.end())
      return it->second;
    const symbol_t id = static_cast<symbol_t>(names_.size());
    by_name_.emplace(name, id);
    names_.push_back(name);
    return id;
  }

  // Look up an existing name; returns true and sets `out` if present.
  bool find(const std::string &name, symbol_t &out) const {
    auto it = by_name_.find(name);
    if (it == by_name_.end())
      return false;
    out = it->second;
    return true;
  }

  const std::string &name(symbol_t id) const { return names_[id]; }
  std::size_t size() const noexcept { return names_.size(); }

  // True if `id` is one of the fixed built-in symbols.
  static bool is_builtin(symbol_t id) noexcept {
    return id < static_cast<symbol_t>(builtin::_first_user);
  }

private:
  void register_builtins() {
    // Order MUST match the `builtin` enum exactly.
    const char *names[] = {"F",      "f",   "G",    "+", "-",  "&",     "^",      "/",   "\\",
                           "|",      "$",   "T",    "R", "L",  "!",     ";",      "'",   "[",
                           "]",      "%",   "{",    ".", "}",  "Trans", "Scale",  "Rot", "Subdiv",
                           "Repeat", "Box", "Inst", "P", "Pw", "Pb",    "Stamp2D"};
    for (const char *n : names)
      intern(n);
  }

  std::vector<std::string> names_;
  std::unordered_map<std::string, symbol_t> by_name_;
};

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_SYMBOL_H
