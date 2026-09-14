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
#include <cstdlib>
#include <cvc/lsys/parse.h>
#include <sstream>

namespace cvc {
namespace lsys {

namespace {

const char *kOperators = "+-&^/\\|$!;'[]%{.}";

bool is_operator_char(char c) {
  for (const char *p = kOperators; *p; ++p)
    if (*p == c)
      return true;
  return false;
}

std::string trim(const std::string &s) {
  std::size_t a = 0, b = s.size();
  while (a < b && std::isspace((unsigned char)s[a]))
    ++a;
  while (b > a && std::isspace((unsigned char)s[b - 1]))
    --b;
  return s.substr(a, b - a);
}

// Split at top-level commas (parenthesis-aware).
std::vector<std::string> split_top_commas(const std::string &s) {
  std::vector<std::string> out;
  int depth = 0;
  std::string cur;
  for (char c : s) {
    if (c == '(')
      ++depth;
    else if (c == ')')
      --depth;
    if (c == ',' && depth == 0) {
      out.push_back(trim(cur));
      cur.clear();
    } else
      cur.push_back(c);
  }
  if (!trim(cur).empty() || !out.empty())
    out.push_back(trim(cur));
  return out;
}

struct parser {
  ruleset &rs;
  std::vector<diagnostic> &diags;
  int line = 0;

  parser(ruleset &r, std::vector<diagnostic> &d) : rs(r), diags(d) {}

  void err(const std::string &m) { diags.push_back({diagnostic::level::error, m, line, 0}); }

  // Read one module (op or identifier + optional arg list) starting at i.
  bool read_module(const std::string &s, std::size_t &i, module_expr &m) {
    while (i < s.size() && std::isspace((unsigned char)s[i]))
      ++i;
    if (i >= s.size())
      return false;
    std::string name;
    if (is_operator_char(s[i])) {
      name.push_back(s[i++]);
    } else if (std::isalnum((unsigned char)s[i]) || s[i] == '_') {
      while (i < s.size() && (std::isalnum((unsigned char)s[i]) || s[i] == '_'))
        name.push_back(s[i++]);
    } else {
      err("unexpected character '" + std::string(1, s[i]) + "' in module string");
      ++i;
      return false;
    }
    m.sym = rs.syms.intern(name);
    m.params.clear();
    // Optional argument list.
    while (i < s.size() && std::isspace((unsigned char)s[i]))
      ++i;
    if (i < s.size() && s[i] == '(') {
      int depth = 0;
      std::size_t start = i + 1;
      std::size_t j = i;
      for (; j < s.size(); ++j) {
        if (s[j] == '(')
          ++depth;
        else if (s[j] == ')') {
          if (--depth == 0)
            break;
        }
      }
      if (j >= s.size()) {
        err("unbalanced '(' in module '" + name + "'");
        i = s.size();
        return true;
      }
      std::string inside = s.substr(start, j - start);
      i = j + 1;
      for (const std::string &arg : split_top_commas(inside))
        m.params.push_back(compile_expr(arg, &diags, name));
    }
    return true;
  }

  std::vector<module_expr> read_module_seq(const std::string &s) {
    std::vector<module_expr> out;
    std::size_t i = 0;
    module_expr m;
    while (read_module(s, i, m))
      out.push_back(m);
    return out;
  }

  // Read a predecessor pattern: Name or Name(f1, f2) with formal NAMES.
  bool read_predecessor(const std::string &s, symbol_t &sym, std::vector<std::string> &formals) {
    std::string t = trim(s);
    std::size_t p = t.find('(');
    std::string name = p == std::string::npos ? t : trim(t.substr(0, p));
    if (name.empty())
      return false;
    sym = rs.syms.intern(name);
    formals.clear();
    if (p != std::string::npos) {
      std::size_t q = t.rfind(')');
      if (q == std::string::npos || q < p)
        return false;
      for (const std::string &f : split_top_commas(t.substr(p + 1, q - p - 1)))
        formals.push_back(trim(f));
    }
    return true;
  }

  // Context is a sequence of symbol names (params ignored for matching).
  std::vector<symbol_t> read_context(const std::string &s) {
    std::vector<symbol_t> out;
    std::size_t i = 0;
    module_expr m;
    while (read_module(s, i, m))
      out.push_back(m.sym);
    return out;
  }

  void parse_production(const std::string &lineStr) {
    std::size_t arrow = lineStr.find("->");
    std::string lhs = trim(lineStr.substr(0, arrow));
    std::string rhs = arrow == std::string::npos ? "" : trim(lineStr.substr(arrow + 2));

    production prod;

    // RHS: optional (prob) prefix, then successor modules.
    if (!rhs.empty() && rhs[0] == '(') {
      int depth = 0;
      std::size_t j = 0;
      for (; j < rhs.size(); ++j) {
        if (rhs[j] == '(')
          ++depth;
        else if (rhs[j] == ')') {
          if (--depth == 0)
            break;
        }
      }
      prod.probability = compile_expr(rhs.substr(1, j - 1), &diags, "prob");
      rhs = trim(rhs.substr(j + 1));
    }
    prod.successor = read_module_seq(rhs);

    // LHS: [ctxL <] pred [> ctxR] [: guard]
    std::string guardStr;
    std::size_t colon = lhs.find(':');
    if (colon != std::string::npos) {
      guardStr = trim(lhs.substr(colon + 1));
      lhs = trim(lhs.substr(0, colon));
    }
    std::string predStr = lhs;
    std::size_t lt = lhs.find('<');
    if (lt != std::string::npos) {
      prod.left_ctx = read_context(lhs.substr(0, lt));
      predStr = trim(lhs.substr(lt + 1));
    }
    std::size_t gt = predStr.find('>');
    if (gt != std::string::npos) {
      prod.right_ctx = read_context(predStr.substr(gt + 1));
      predStr = trim(predStr.substr(0, gt));
    }
    if (!read_predecessor(predStr, prod.pred, prod.pred_params)) {
      err("bad predecessor '" + predStr + "'");
      return;
    }
    if (!guardStr.empty())
      prod.guard = compile_expr(guardStr, &diags, "guard");

    // deletes / cut detection => grammar is not gen_nested.
    if (prod.successor.empty())
      prod.deletes = true;
    for (const module_expr &m : prod.successor)
      if (m.sym == builtin_id(builtin::cut))
        prod.deletes = true;
    if (prod.deletes || !prod.left_ctx.empty() || !prod.right_ctx.empty())
      rs.gen_nested = false;

    rs.prods.push_back(std::move(prod));
  }

  void run(const std::string &src) {
    std::uint64_t h = 0xcbf29ce484222325ull;
    for (char c : src) {
      h ^= (unsigned char)c;
      h *= 0x100000001b3ull;
    }
    rs.grammar_hash = h;

    std::istringstream in(src);
    std::string raw;
    while (std::getline(in, raw)) {
      ++line;
      std::string s = trim(raw);
      if (s.empty())
        continue;
      if (s[0] == '#') {
        rs.comments.push_back(s);
        continue;
      }
      // header key: value  (but not a production, which contains "->")
      std::size_t colon = s.find(':');
      std::size_t arrow = s.find("->");
      if (colon != std::string::npos && (arrow == std::string::npos || colon < arrow) &&
          s.compare(0, 6, "axiom:") != 0 && !looks_like_production(s, colon, arrow)) {
        std::string key = trim(s.substr(0, colon));
        std::string val = trim(s.substr(colon + 1));
        apply_header(key, val);
        continue;
      }
      if (s.compare(0, 6, "param ") == 0) {
        std::string body = trim(s.substr(6));
        std::size_t eq = body.find('=');
        if (eq == std::string::npos) {
          err("param needs '='");
          continue;
        }
        rs.params.set(trim(body.substr(0, eq)),
                      std::strtod(trim(body.substr(eq + 1)).c_str(), nullptr));
        continue;
      }
      if (s.compare(0, 6, "axiom:") == 0) {
        rs.axiom = read_module_seq(trim(s.substr(6)));
        continue;
      }
      if (arrow != std::string::npos) {
        parse_production(s);
        continue;
      }
      err("unrecognized line: " + s);
    }
  }

  // A production line has '->'; a header 'key: value' has ':' before any '->'.
  bool looks_like_production(const std::string &s, std::size_t colon, std::size_t arrow) {
    (void)s;
    (void)colon;
    return arrow != std::string::npos;
  }

  void apply_header(const std::string &key, const std::string &val) {
    if (key == "name")
      rs.name = val;
    else if (key == "cite")
      rs.cite = val;
    else if (key == "parent")
      rs.parent = val;
    else if (key == "kind") {
      if (val == "plant")
        rs.kind = asset_kind::plant;
      else if (val == "terrain")
        rs.kind = asset_kind::terrain;
      else if (val == "cloud")
        rs.kind = asset_kind::cloud;
      else if (val == "rock")
        rs.kind = asset_kind::rock;
      else if (val == "building")
        rs.kind = asset_kind::building;
      else
        err("unknown kind '" + val + "'");
    } else if (key == "mode") {
      if (val == "parallel")
        rs.mode = derivation_mode::parallel;
      else if (val == "sequential" || val == "sequential_priority")
        rs.mode = derivation_mode::sequential_priority;
      else
        err("unknown mode '" + val + "'");
    } else if (key == "contain") {
      rs.contain = (val == "strict") ? containment::strict : containment::none;
    } else if (key == "angle")
      rs.angle_deg = std::strtod(val.c_str(), nullptr);
    else if (key == "tilt")
      rs.tilt_deg = std::strtod(val.c_str(), nullptr);
    else if (key == "roll")
      rs.roll_deg = std::strtod(val.c_str(), nullptr);
    else if (key == "step")
      rs.step = std::strtod(val.c_str(), nullptr);
    else if (key == "width")
      rs.width = std::strtod(val.c_str(), nullptr);
    else if (key == "taper")
      rs.taper = std::strtod(val.c_str(), nullptr);
    else if (key == "preview_gen")
      rs.preview_gen = std::atoi(val.c_str());
    else if (key == "build_gen")
      rs.build_gen = std::atoi(val.c_str());
    else if (key == "ignore")
      rs.ignore = read_context(val);
    else
      err("unknown header key '" + key + "'");
  }
};

} // namespace

parse_result parse_lsys(const std::string &src) {
  parse_result r;
  parser p(r.rs, r.diags);
  p.run(src);
  bool has_error = false;
  for (const diagnostic &d : r.diags)
    if (d.sev == diagnostic::level::error)
      has_error = true;
  r.ok = !has_error;
  return r;
}

} // namespace lsys
} // namespace cvc
