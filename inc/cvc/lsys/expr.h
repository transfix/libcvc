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

// expr.h — a tiny expression VM for parametric L-systems.
//
// Guards, probabilities and successor-module parameters are arithmetic
// expressions over the predecessor module's formal parameters (bound by name)
// plus the ruleset's global named params. An empty() expr means "absent" — a
// guard that is always true, a probability of 1, or a literal-0 parameter.
//
// The language: numeric literals, identifiers, ( ), the binary operators
// + - * / with the usual precedence, unary - and !, the comparisons
// < <= > >= == != (yielding 1.0/0.0), the logical && ||, and a small set of
// functions: sin cos tan sqrt pow abs min max floor ceil exp log clamp.
// Angles for sin/cos/tan are RADIANS (source degrees are converted at parse of
// rotation modules, not here). No randomness lives in expressions — stochastic
// rule choice and jitter are the deriver's job, via named RNG streams.

#ifndef CVC_LSYS_EXPR_H
#define CVC_LSYS_EXPR_H

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace cvc {
namespace lsys {

struct diagnostic; // fwd (parse.h)

class expr {
public:
  enum class opcode : std::uint8_t {
    push_const,
    push_var,
    neg,
    lnot,
    add,
    sub,
    mul,
    div,
    lt,
    le,
    gt,
    ge,
    eq,
    ne,
    land,
    lor,
    call
  };

  struct op {
    opcode code;
    std::uint16_t arg = 0; // const index, var index, or (func<<3 | argc)
  };

  bool empty() const noexcept { return ops_.empty(); }

  // Evaluate against a variable lookup. Unknown variables resolve to 0.0.
  double eval(const std::function<double(const std::string &)> &lookup) const;

  // Convenience: evaluate a guard (empty => true).
  bool eval_guard(const std::function<double(const std::string &)> &lookup) const {
    return empty() ? true : eval(lookup) != 0.0;
  }

  // Builder API used by the compiler (compile_expr in expr.cpp).
  void emit(opcode c, std::uint16_t a = 0) { ops_.push_back({c, a}); }
  std::uint16_t add_const(double v) {
    consts_.push_back(v);
    return static_cast<std::uint16_t>(consts_.size() - 1);
  }
  std::uint16_t add_var(const std::string &name) {
    vars_.push_back(name);
    return static_cast<std::uint16_t>(vars_.size() - 1);
  }

  const std::vector<op> &ops() const noexcept { return ops_; }
  const std::vector<std::string> &vars() const noexcept { return vars_; }

  // Trimmed original source, retained for byte-exact `.lsys` writing.
  const std::string &source() const noexcept { return src_; }
  void set_source(std::string s) { src_ = std::move(s); }

private:
  std::vector<op> ops_;
  std::vector<double> consts_;
  std::vector<std::string> vars_;
  std::string src_;
};

// Compile a source expression. On error, appends to `diags` (if non-null) and
// returns an empty expr. `where` is a human label for diagnostics.
expr compile_expr(const std::string &src, std::vector<diagnostic> *diags, const std::string &where);

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_EXPR_H
