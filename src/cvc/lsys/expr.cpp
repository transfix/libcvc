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

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cvc/lsys/expr.h>
#include <cvc/lsys/grammar.h> // diagnostic
#include <vector>

namespace cvc {
namespace lsys {

namespace {

// Function ids encoded into a call op's arg as (func << 3 | argc).
enum func : std::uint16_t {
  f_sin = 0,
  f_cos,
  f_tan,
  f_sqrt,
  f_abs,
  f_floor,
  f_ceil,
  f_exp,
  f_log,
  f_pow,
  f_min,
  f_max,
  f_clamp
};

int func_argc(func fn) {
  switch (fn) {
  case f_pow:
  case f_min:
  case f_max:
    return 2;
  case f_clamp:
    return 3;
  default:
    return 1;
  }
}

bool func_by_name(const std::string &n, func &out) {
  if (n == "sin")
    out = f_sin;
  else if (n == "cos")
    out = f_cos;
  else if (n == "tan")
    out = f_tan;
  else if (n == "sqrt")
    out = f_sqrt;
  else if (n == "abs")
    out = f_abs;
  else if (n == "floor")
    out = f_floor;
  else if (n == "ceil")
    out = f_ceil;
  else if (n == "exp")
    out = f_exp;
  else if (n == "log")
    out = f_log;
  else if (n == "pow")
    out = f_pow;
  else if (n == "min")
    out = f_min;
  else if (n == "max")
    out = f_max;
  else if (n == "clamp")
    out = f_clamp;
  else
    return false;
  return true;
}

// A recursive-descent expression compiler.
struct compiler {
  const std::string &s;
  std::size_t i = 0;
  expr &e;
  std::vector<diagnostic> *diags;
  const std::string &where;
  bool ok = true;
  int depth = 0;
  // Bounds BOTH the parser's native recursion and eval's fixed 64-slot operand
  // stack: operand-stack depth <= nesting depth, so capping nesting < 64 makes
  // the eval stack overflow (silent-wrong) unreachable. Deep generated grammars
  // now get a loud diagnostic instead of a truncated wrong value.
  static constexpr int kMaxDepth = 48;

  compiler(const std::string &src, expr &out, std::vector<diagnostic> *d, const std::string &w)
      : s(src), e(out), diags(d), where(w) {}

  void err(const std::string &m) {
    ok = false;
    if (diags)
      diags->push_back({diagnostic::level::error, "expr[" + where + "]: " + m, 0, (int)i});
  }

  void ws() {
    while (i < s.size() && std::isspace((unsigned char)s[i]))
      ++i;
  }
  bool eof() {
    ws();
    return i >= s.size();
  }
  char peek() {
    ws();
    return i < s.size() ? s[i] : '\0';
  }
  bool accept(char c) {
    ws();
    if (i < s.size() && s[i] == c) {
      ++i;
      return true;
    }
    return false;
  }
  bool accept2(char a, char b) {
    ws();
    if (i + 1 < s.size() && s[i] == a && s[i + 1] == b) {
      i += 2;
      return true;
    }
    return false;
  }

  void parse_primary() {
    ws();
    if (++depth > kMaxDepth) {
      err("expression nested too deeply");
      --depth;
      return;
    }
    struct pop_depth {
      int &d;
      ~pop_depth() { --d; }
    } _pd{depth};
    if (accept('(')) {
      parse_or();
      if (!accept(')'))
        err("expected ')'");
      return;
    }
    char c = peek();
    if (c == '-') {
      ++i;
      parse_primary();
      e.emit(expr::opcode::neg);
      return;
    }
    if (c == '!') {
      ++i;
      parse_primary();
      e.emit(expr::opcode::lnot);
      return;
    }
    if (std::isdigit((unsigned char)c) || c == '.') {
      const char *start = s.c_str() + i;
      char *end = nullptr;
      double v = std::strtod(start, &end);
      if (end == start) {
        err("bad number");
        return;
      }
      i += (std::size_t)(end - start);
      e.emit(expr::opcode::push_const, e.add_const(v));
      return;
    }
    if (std::isalpha((unsigned char)c) || c == '_') {
      std::string ident;
      while (i < s.size() && (std::isalnum((unsigned char)s[i]) || s[i] == '_'))
        ident.push_back(s[i++]);
      if (peek() == '(') {
        func fn;
        if (!func_by_name(ident, fn)) {
          err("unknown function '" + ident + "'");
          return;
        }
        accept('(');
        int argc = 0;
        if (peek() != ')') {
          parse_or();
          ++argc;
          while (accept(',')) {
            parse_or();
            ++argc;
          }
        }
        if (!accept(')'))
          err("expected ')' after args");
        if (argc != func_argc(fn))
          err("wrong arg count for '" + ident + "'");
        e.emit(expr::opcode::call, (std::uint16_t)(((std::uint16_t)fn << 3) | (std::uint16_t)argc));
        return;
      }
      e.emit(expr::opcode::push_var, e.add_var(ident));
      return;
    }
    err("unexpected token");
    if (i < s.size())
      ++i;
  }

  void parse_mul() {
    parse_primary();
    for (;;) {
      ws();
      if (accept('*')) {
        parse_primary();
        e.emit(expr::opcode::mul);
      } else if (accept('/')) {
        parse_primary();
        e.emit(expr::opcode::div);
      } else
        break;
    }
  }
  void parse_add() {
    parse_mul();
    for (;;) {
      ws();
      // Do not consume a leading '-' that belongs to unary context; here it is binary.
      if (i < s.size() && s[i] == '+') {
        ++i;
        parse_mul();
        e.emit(expr::opcode::add);
      } else if (i < s.size() && s[i] == '-') {
        ++i;
        parse_mul();
        e.emit(expr::opcode::sub);
      } else
        break;
    }
  }
  void parse_cmp() {
    parse_add();
    for (;;) {
      ws();
      if (accept2('<', '=')) {
        parse_add();
        e.emit(expr::opcode::le);
      } else if (accept2('>', '=')) {
        parse_add();
        e.emit(expr::opcode::ge);
      } else if (i < s.size() && s[i] == '<') {
        ++i;
        parse_add();
        e.emit(expr::opcode::lt);
      } else if (i < s.size() && s[i] == '>') {
        ++i;
        parse_add();
        e.emit(expr::opcode::gt);
      } else
        break;
    }
  }
  void parse_eq() {
    parse_cmp();
    for (;;) {
      ws();
      if (accept2('=', '=')) {
        parse_cmp();
        e.emit(expr::opcode::eq);
      } else if (accept2('!', '=')) {
        parse_cmp();
        e.emit(expr::opcode::ne);
      } else
        break;
    }
  }
  void parse_and() {
    parse_eq();
    while (accept2('&', '&')) {
      parse_eq();
      e.emit(expr::opcode::land);
    }
  }
  void parse_or() {
    parse_and();
    while (accept2('|', '|')) {
      parse_and();
      e.emit(expr::opcode::lor);
    }
  }
};

} // namespace

expr compile_expr(const std::string &src, std::vector<diagnostic> *diags,
                  const std::string &where) {
  expr e;
  // Empty/whitespace source => empty expr (absent).
  bool only_ws = true;
  for (char c : src)
    if (!std::isspace((unsigned char)c)) {
      only_ws = false;
      break;
    }
  if (only_ws)
    return e;
  compiler c(src, e, diags, where);
  c.parse_or();
  if (!c.eof())
    c.err("trailing characters");
  if (!c.ok)
    return expr{}; // empty on error
  // Retain trimmed source for byte-exact writing.
  std::size_t a = 0, b = src.size();
  while (a < b && std::isspace((unsigned char)src[a]))
    ++a;
  while (b > a && std::isspace((unsigned char)src[b - 1]))
    --b;
  e.set_source(src.substr(a, b - a));
  return e;
}

double expr::eval(const std::function<double(const std::string &)> &lookup) const {
  double stack[64];
  int sp = 0;
  auto push = [&](double v) {
    if (sp < 64)
      stack[sp++] = v;
  };
  auto pop = [&]() -> double { return sp > 0 ? stack[--sp] : 0.0; };
  for (const op &o : ops_) {
    switch (o.code) {
    case opcode::push_const:
      push(consts_[o.arg]);
      break;
    case opcode::push_var:
      push(lookup ? lookup(vars_[o.arg]) : 0.0);
      break;
    case opcode::neg:
      push(-pop());
      break;
    case opcode::lnot:
      push(pop() == 0.0 ? 1.0 : 0.0);
      break;
    case opcode::add: {
      double b = pop(), a = pop();
      push(a + b);
      break;
    }
    case opcode::sub: {
      double b = pop(), a = pop();
      push(a - b);
      break;
    }
    case opcode::mul: {
      double b = pop(), a = pop();
      push(a * b);
      break;
    }
    case opcode::div: {
      double b = pop(), a = pop();
      push(b != 0.0 ? a / b : 0.0);
      break;
    }
    case opcode::lt: {
      double b = pop(), a = pop();
      push(a < b ? 1.0 : 0.0);
      break;
    }
    case opcode::le: {
      double b = pop(), a = pop();
      push(a <= b ? 1.0 : 0.0);
      break;
    }
    case opcode::gt: {
      double b = pop(), a = pop();
      push(a > b ? 1.0 : 0.0);
      break;
    }
    case opcode::ge: {
      double b = pop(), a = pop();
      push(a >= b ? 1.0 : 0.0);
      break;
    }
    case opcode::eq: {
      double b = pop(), a = pop();
      push(a == b ? 1.0 : 0.0);
      break;
    }
    case opcode::ne: {
      double b = pop(), a = pop();
      push(a != b ? 1.0 : 0.0);
      break;
    }
    case opcode::land: {
      double b = pop(), a = pop();
      push((a != 0.0 && b != 0.0) ? 1.0 : 0.0);
      break;
    }
    case opcode::lor: {
      double b = pop(), a = pop();
      push((a != 0.0 || b != 0.0) ? 1.0 : 0.0);
      break;
    }
    case opcode::call: {
      const func fn = static_cast<func>(o.arg >> 3);
      const int argc = o.arg & 0x7;
      double a[3] = {0, 0, 0};
      for (int k = argc - 1; k >= 0; --k)
        a[k] = pop();
      double r = 0.0;
      switch (fn) {
      case f_sin:
        r = std::sin(a[0]);
        break;
      case f_cos:
        r = std::cos(a[0]);
        break;
      case f_tan:
        r = std::tan(a[0]);
        break;
      case f_sqrt:
        r = std::sqrt(a[0] >= 0 ? a[0] : 0.0);
        break;
      case f_abs:
        r = std::fabs(a[0]);
        break;
      case f_floor:
        r = std::floor(a[0]);
        break;
      case f_ceil:
        r = std::ceil(a[0]);
        break;
      case f_exp:
        r = std::exp(a[0]);
        break;
      case f_log:
        r = std::log(a[0] > 0 ? a[0] : 2.220446049250313e-16);
        break;
      case f_pow:
        r = std::pow(a[0], a[1]);
        break;
      case f_min:
        r = a[0] < a[1] ? a[0] : a[1];
        break;
      case f_max:
        r = a[0] > a[1] ? a[0] : a[1];
        break;
      case f_clamp:
        r = a[0] < a[1] ? a[1] : (a[0] > a[2] ? a[2] : a[0]);
        break;
      }
      push(r);
      break;
    }
    }
  }
  return sp > 0 ? stack[sp - 1] : 0.0;
}

} // namespace lsys
} // namespace cvc
