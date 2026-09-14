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

// cvc-lsys — derive / render / validate `.lsys` recipes, headless and GL-free.
//
//   cvc-lsys list
//   cvc-lsys validate <recipe|file.lsys>
//   cvc-lsys derive   <recipe|file.lsys> [--seed N] [--gen K] [--stats] [--dump-word]
//   cvc-lsys svg      <recipe|file.lsys> --out t.svg [--gen K] [--seed N]
//                       [--proj front|side|top] [--width 900] [--colour order|level|class]
//   cvc-lsys mesh     <recipe|file.lsys> --out t.off [--gen K] [--seed N]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/lsys/derive.h>
#include <cvc/lsys/emit.h>
#include <cvc/lsys/interp.h>
#include <cvc/lsys/io.h>
#include <cvc/lsys/recipes.h>
#include <fstream>
#include <string>

using namespace cvc::lsys;

namespace {

const char *opt(int argc, char **argv, const char *name, const char *dflt) {
  for (int i = 0; i < argc - 1; ++i)
    if (std::strcmp(argv[i], name) == 0)
      return argv[i + 1];
  return dflt;
}
bool flag(int argc, char **argv, const char *name) {
  for (int i = 0; i < argc; ++i)
    if (std::strcmp(argv[i], name) == 0)
      return true;
  return false;
}

int usage() {
  std::fprintf(stderr,
               "usage: cvc-lsys <list|validate|derive|svg|mesh> <recipe|file.lsys> [options]\n"
               "  --seed N  --gen K  --out FILE  --proj front|side|top  --width PX\n"
               "  --colour order|level|class  --stats  --dump-word\n");
  return 2;
}

// Load a ruleset from either a built-in recipe name or a .lsys file path.
bool load(const std::string &what, ruleset &rs) {
  if (has_recipe(what)) {
    rs = load_recipe(what);
    return true;
  }
  parse_result pr = parse_lsys_file(what);
  for (const diagnostic &d : pr.diags)
    std::fprintf(stderr, "%s:%d: %s\n", what.c_str(), d.line, d.message.c_str());
  if (!pr.ok)
    return false;
  rs = pr.rs;
  return true;
}

int generations_for(const ruleset &rs, int cli_gen) {
  if (cli_gen >= 0)
    return cli_gen;
  return rs.kind == asset_kind::plant ? rs.build_gen : 3;
}

} // namespace

int main(int argc, char **argv) try {
  if (argc < 2)
    return usage();
  const std::string cmd = argv[1];

  if (cmd == "list") {
    for (const std::string &n : recipe_names())
      std::printf("%s\n", n.c_str());
    return 0;
  }
  if (argc < 3)
    return usage();
  const std::string what = argv[2];

  ruleset rs;
  if (!load(what, rs))
    return 1;

  const int cli_gen = std::atoi(opt(argc, argv, "--gen", "-1"));
  derive_options dopt;
  dopt.master_seed = std::strtoull(opt(argc, argv, "--seed", "0"), nullptr, 10);
  dopt.generations = generations_for(rs, cli_gen);

  if (cmd == "validate") {
    derive_options d0 = dopt;
    d0.generations = std::min(dopt.generations, 4);
    derive_result r = derive(rs, d0);
    structure s = interpret(rs, r.w);
    std::printf("OK  name=%s kind=%d mode=%s gen_nested=%d prods=%zu\n", rs.name.c_str(),
                (int)rs.kind, rs.mode == derivation_mode::parallel ? "parallel" : "sequential",
                rs.gen_nested, rs.prods.size());
    std::printf("    gen%d modules=%zu segments=%zu leaves=%zu boxes=%zu%s\n", d0.generations,
                r.w.size(), s.segments.size(), s.leaves.size(), s.boxes.size(),
                r.truncated ? "  [TRUNCATED]" : "");
    return 0;
  }

  derive_result r = derive(rs, dopt);
  structure s = interpret(rs, r.w);

  if (cmd == "derive") {
    stats st = compute_stats(rs, r.w, s);
    std::printf("modules=%zu segments=%zu leaves=%zu boxes=%zu paints=%zu gen=%u%s\n", st.modules,
                st.segments, st.leaves, st.boxes, st.paints, r.generations_reached,
                r.truncated ? "  [TRUNCATED]" : "");
    if (flag(argc, argv, "--stats")) {
      std::printf("by symbol:\n");
      for (const auto &kv : st.by_symbol)
        std::printf("  %-10s %u\n", kv.first.c_str(), kv.second);
      std::printf("by level:");
      for (int i = 0; i < 12; ++i)
        std::printf(" %u", st.level_counts[i]);
      std::printf("\n");
    }
    if (flag(argc, argv, "--dump-word")) {
      for (std::size_t i = 0; i < r.w.size() && i < 4000; ++i) {
        const module_t &m = r.w[i];
        std::printf("%s", rs.syms.name(m.sym).c_str());
        if (m.nparams) {
          std::printf("(");
          for (int k = 0; k < m.nparams; ++k)
            std::printf("%s%.3g", k ? "," : "", m.p[k]);
          std::printf(")");
        }
        std::printf(" ");
      }
      std::printf("\n");
    }
    return 0;
  }

  if (cmd == "svg") {
    const char *out = opt(argc, argv, "--out", nullptr);
    if (!out) {
      std::fprintf(stderr, "svg needs --out FILE\n");
      return 2;
    }
    svg_options so;
    so.width_px = std::atoi(opt(argc, argv, "--width", "900"));
    const std::string proj = opt(argc, argv, "--proj", "front");
    so.proj = proj == "side"  ? projection::side
              : proj == "top" ? projection::top
                              : projection::front;
    const std::string col = opt(argc, argv, "--colour", opt(argc, argv, "--color", "class"));
    so.color = col == "order"   ? svg_color::order
               : col == "level" ? svg_color::level
                                : svg_color::klass;
    std::ofstream f(out, std::ios::binary);
    if (!f) {
      std::fprintf(stderr, "cannot write %s\n", out);
      return 1;
    }
    f << emit_svg(s, so);
    std::printf("wrote %s  (%zu elements)\n", out, svg_element_count(s));
    return 0;
  }

  if (cmd == "mesh") {
    const char *out = opt(argc, argv, "--out", nullptr);
    if (!out) {
      std::fprintf(stderr, "mesh needs --out FILE\n");
      return 2;
    }
    std::ofstream f(out, std::ios::binary);
    if (!f) {
      std::fprintf(stderr, "cannot write %s\n", out);
      return 1;
    }
    f << emit_off(s);
    std::printf("wrote %s\n", out);
    return 0;
  }

  return usage();
} catch (const std::exception &e) {
  std::fprintf(stderr, "error: %s\n", e.what());
  return 1;
}
