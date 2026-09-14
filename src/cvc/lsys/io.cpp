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

#include <cstdio>
#include <cvc/lsys/io.h>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace lsys {

namespace {

std::string fmtnum(double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%g", v);
  return buf;
}

const char *kind_name(asset_kind k) {
  switch (k) {
  case asset_kind::plant:
    return "plant";
  case asset_kind::terrain:
    return "terrain";
  case asset_kind::cloud:
    return "cloud";
  case asset_kind::rock:
    return "rock";
  case asset_kind::building:
    return "building";
  }
  return "plant";
}

std::string module_str(const ruleset &rs, const module_expr &m) {
  std::string s = rs.syms.name(m.sym);
  if (!m.params.empty()) {
    s += "(";
    for (std::size_t i = 0; i < m.params.size(); ++i) {
      if (i)
        s += ", ";
      s += m.params[i].source();
    }
    s += ")";
  }
  return s;
}

std::string seq_str(const ruleset &rs, const std::vector<module_expr> &seq) {
  std::string s;
  for (std::size_t i = 0; i < seq.size(); ++i) {
    if (i)
      s += " ";
    s += module_str(rs, seq[i]);
  }
  return s;
}

std::string ctx_str(const ruleset &rs, const std::vector<symbol_t> &c) {
  std::string s;
  for (std::size_t i = 0; i < c.size(); ++i) {
    if (i)
      s += " ";
    s += rs.syms.name(c[i]);
  }
  return s;
}

} // namespace

std::string write_lsys(const ruleset &rs) {
  std::ostringstream o;
  for (const std::string &c : rs.comments)
    o << c << "\n";
  if (!rs.name.empty())
    o << "name: " << rs.name << "\n";
  if (!rs.cite.empty())
    o << "cite: " << rs.cite << "\n";
  if (!rs.parent.empty())
    o << "parent: " << rs.parent << "\n";
  o << "kind: " << kind_name(rs.kind) << "\n";
  o << "mode: " << (rs.mode == derivation_mode::parallel ? "parallel" : "sequential") << "\n";
  o << "contain: " << (rs.contain == containment::strict ? "strict" : "none") << "\n";
  o << "angle: " << fmtnum(rs.angle_deg) << "\n";
  o << "tilt: " << fmtnum(rs.tilt_deg) << "\n";
  o << "roll: " << fmtnum(rs.roll_deg) << "\n";
  o << "step: " << fmtnum(rs.step) << "\n";
  o << "width: " << fmtnum(rs.width) << "\n";
  o << "taper: " << fmtnum(rs.taper) << "\n";
  o << "preview_gen: " << rs.preview_gen << "\n";
  o << "build_gen: " << rs.build_gen << "\n";
  if (!rs.ignore.empty())
    o << "ignore: " << ctx_str(rs, rs.ignore) << "\n";
  // params sorted by name for determinism.
  std::map<std::string, double> sorted(rs.params.all().begin(), rs.params.all().end());
  for (const auto &kv : sorted)
    o << "param " << kv.first << " = " << fmtnum(kv.second) << "\n";
  o << "axiom: " << seq_str(rs, rs.axiom) << "\n";

  for (const production &p : rs.prods) {
    std::string line;
    if (!p.left_ctx.empty())
      line += ctx_str(rs, p.left_ctx) + " < ";
    line += rs.syms.name(p.pred);
    if (!p.pred_params.empty()) {
      line += "(";
      for (std::size_t i = 0; i < p.pred_params.size(); ++i) {
        if (i)
          line += ", ";
        line += p.pred_params[i];
      }
      line += ")";
    }
    if (!p.right_ctx.empty())
      line += " > " + ctx_str(rs, p.right_ctx);
    if (!p.guard.empty())
      line += " : " + p.guard.source();
    line += " -> ";
    if (!p.probability.empty())
      line += "(" + p.probability.source() + ") ";
    line += seq_str(rs, p.successor);
    o << line << "\n";
  }
  return o.str();
}

parse_result parse_lsys_file(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot open .lsys file: " + path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return parse_lsys(ss.str());
}

void write_lsys_file(const std::string &path, const ruleset &rs) {
  std::ofstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot write .lsys file: " + path);
  f << write_lsys(rs);
}

} // namespace lsys
} // namespace cvc
