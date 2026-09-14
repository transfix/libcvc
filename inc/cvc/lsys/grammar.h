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

// grammar.h — the ruleset the deriver executes.
//
// One engine, three orthogonal escape hatches (roadmap §5.1): derivation_mode
// (parallel vs sequential-priority), containment (none for plants, strict for
// buildings), and context (string-neighbour vs spatial, via context_provider).

#ifndef CVC_LSYS_GRAMMAR_H
#define CVC_LSYS_GRAMMAR_H

#include <array>
#include <cstdint>
#include <cvc/lsys/expr.h>
#include <cvc/lsys/module.h>
#include <cvc/lsys/symbol.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace lsys {

struct diagnostic {
  enum class level { info, warning, error };
  level sev = level::error;
  std::string message;
  int line = 0;
  int col = 0;
};

enum class derivation_mode : std::uint8_t { parallel, sequential_priority };
enum class containment : std::uint8_t { none, strict };
enum class asset_kind : std::uint8_t { plant, terrain, cloud, rock, building };

// A module template in an axiom or a production successor: a symbol plus
// parameter expressions evaluated at rewrite time.
struct module_expr {
  symbol_t sym = 0;
  std::vector<expr> params;
};

// Named scalar parameters (global to the ruleset).
class param_table {
public:
  void set(const std::string &name, double v) { m_[name] = v; }
  double get(const std::string &name, double dflt = 0.0) const {
    auto it = m_.find(name);
    return it == m_.end() ? dflt : it->second;
  }
  bool has(const std::string &name) const { return m_.count(name) != 0; }
  const std::unordered_map<std::string, double> &all() const noexcept { return m_; }

private:
  std::unordered_map<std::string, double> m_;
};

struct production {
  symbol_t pred = 0;
  std::vector<std::string> pred_params; // formal names bound from the matched module's params
  std::vector<symbol_t> left_ctx;       // empty == context-free
  std::vector<symbol_t> right_ctx;
  expr guard;       // empty == always
  expr probability; // empty == 1.0
  std::uint8_t priority = 0;
  std::vector<module_expr> successor;
  bool deletes = false; // successor empty or contains '%' (makes the ruleset non-nested)
};

struct ruleset {
  std::string name;
  std::string cite;
  std::string parent; // variant lineage
  asset_kind kind = asset_kind::plant;
  derivation_mode mode = derivation_mode::parallel;
  containment contain = containment::none;

  std::vector<symbol_t> ignore; // #ignore for context matching
  std::vector<module_expr> axiom;
  std::vector<production> prods;
  param_table params;

  // Turtle/scope interpretation defaults (degrees; converted to radians in the
  // interpreter). The extracted forest grammar uses tilt "T" and roll "R".
  double angle_deg = 22.5; // default +/-/&/^ turn
  double tilt_deg = 30.0;  // "T"
  double roll_deg = 90.0;  // "R"
  double step = 1.0;       // default "F" length
  double width = 0.1;      // default "!" width
  double taper = 0.0;      // per-segment radius decay fraction (0 == none)

  int preview_gen = 6;
  int build_gen = 10;
  std::array<int, 5> lod_gens{{0, 0, 0, 0, 0}};

  std::uint64_t grammar_hash = 0; // hash of the verbatim source block
  bool gen_nested = true;         // set false by derive() for %/context/deleting rules

  std::vector<std::string> comments; // leading '#' comment lines, preserved for round-trip

  symbol_table syms; // the alphabet, shared across axiom/prods
};

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_GRAMMAR_H
