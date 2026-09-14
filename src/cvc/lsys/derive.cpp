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

#include <algorithm>
#include <chrono>
#include <cvc/lsys/derive.h>
#include <cvc/lsys/rng.h>
#include <functional>
#include <vector>

namespace cvc {
namespace lsys {

namespace {

bool in_set(symbol_t s, const std::vector<symbol_t> &v) {
  for (symbol_t x : v)
    if (x == s)
      return true;
  return false;
}

// Variable lookup binding a matched module's params to a production's formals,
// falling back to the ruleset's global params.
std::function<double(const std::string &)> make_env(const module_t &m, const production &p,
                                                    const ruleset &rs) {
  return [&m, &p, &rs](const std::string &name) -> double {
    for (std::size_t k = 0; k < p.pred_params.size() && k < static_cast<std::size_t>(max_params);
         ++k)
      if (p.pred_params[k] == name)
        return m.p[k];
    return rs.params.get(name, 0.0);
  };
}

// Left context: near-to-far neighbours on the current path, skipping #ignore
// symbols and skipping completed bracketed branches.
bool left_ctx_matches(const std::vector<module_t> &wd, std::size_t pos, const production &p,
                      const ruleset &rs) {
  if (p.left_ctx.empty())
    return true;
  std::size_t need = p.left_ctx.size();
  std::size_t matched = 0;
  int skip = 0;
  long j = static_cast<long>(pos) - 1;
  while (j >= 0 && matched < need) {
    symbol_t s = wd[static_cast<std::size_t>(j)].sym;
    if (s == builtin_id(builtin::pop)) {
      ++skip;
      --j;
      continue;
    }
    if (s == builtin_id(builtin::push)) {
      if (skip > 0)
        --skip;
      // else: stepping out to the branch parent
      --j;
      continue;
    }
    if (skip > 0) {
      --j;
      continue;
    }
    if (in_set(s, rs.ignore)) {
      --j;
      continue;
    }
    // p.left_ctx is written parent..near; the nearest neighbour matches the LAST.
    if (s != p.left_ctx[need - 1 - matched])
      return false;
    ++matched;
    --j;
  }
  return matched == need;
}

// Right context: forward neighbours, skipping #ignore and treating brackets as
// transparent. v1 SIMPLIFICATION: unlike left_ctx_matches (which skips completed
// sibling branches), this does not model bracket structure on the right — it is
// adequate for context-free and simple single-symbol right contexts (all the
// shipped recipes), and full bracket-aware right context is deferred. Documented
// so the asymmetry with left_ctx_matches is intentional, not an oversight.
bool right_ctx_matches(const std::vector<module_t> &wd, std::size_t pos, const production &p,
                       const ruleset &rs) {
  if (p.right_ctx.empty())
    return true;
  std::size_t matched = 0;
  for (std::size_t j = pos + 1; j < wd.size() && matched < p.right_ctx.size(); ++j) {
    symbol_t s = wd[j].sym;
    if (s == builtin_id(builtin::push) || s == builtin_id(builtin::pop))
      continue;
    if (in_set(s, rs.ignore))
      continue;
    if (s != p.right_ctx[matched])
      return false;
    ++matched;
  }
  return matched == p.right_ctx.size();
}

bool prod_applies(const std::vector<module_t> &wd, std::size_t pos, const production &p,
                  const ruleset &rs) {
  const module_t &m = wd[pos];
  if (p.pred != m.sym)
    return false;
  if (!left_ctx_matches(wd, pos, p, rs))
    return false;
  if (!right_ctx_matches(wd, pos, p, rs))
    return false;
  if (!p.guard.empty()) {
    auto env = make_env(m, p, rs);
    if (!p.guard.eval_guard(env))
      return false;
  }
  return true;
}

// Choose one production among the applicable ones: restrict to the highest
// priority, then probability-weighted by the rule_choice stream (module path).
const production *choose(const std::vector<const production *> &apps, const module_t &m,
                         const ruleset &rs, std::uint64_t seed, int gen) {
  if (apps.empty())
    return nullptr;
  std::uint8_t best_prio = 0;
  for (const production *p : apps)
    best_prio = std::max(best_prio, p->priority);
  std::vector<const production *> top;
  for (const production *p : apps)
    if (p->priority == best_prio)
      top.push_back(p);
  if (top.size() == 1)
    return top[0];
  double total = 0.0;
  std::vector<double> w(top.size());
  for (std::size_t i = 0; i < top.size(); ++i) {
    auto env = make_env(m, *top[i], rs);
    w[i] = top[i]->probability.empty() ? 1.0 : top[i]->probability.eval(env);
    if (w[i] < 0.0)
      w[i] = 0.0;
    total += w[i];
  }
  if (total <= 0.0)
    return top[0];
  double u = uni(seed, stream::rule_choice, m.path, static_cast<std::uint32_t>(gen)) * total;
  double acc = 0.0;
  for (std::size_t i = 0; i < top.size(); ++i) {
    acc += w[i];
    if (u < acc)
      return top[i];
  }
  return top.back();
}

module_t make_module(const module_expr &se, const module_t &parent, int level, std::uint64_t path,
                     const std::function<double(const std::string &)> &env) {
  module_t out;
  out.sym = se.sym;
  out.level = static_cast<std::uint8_t>(level < 255 ? level : 255);
  out.path = path;
  out.nparams = static_cast<std::uint8_t>(std::min<std::size_t>(se.params.size(), max_params));
  for (std::size_t k = 0; k < out.nparams; ++k)
    out.p[k] = se.params[k].eval(env);
  (void)parent;
  return out;
}

std::vector<module_t> axiom_to_modules(const ruleset &rs) {
  std::vector<module_t> out;
  auto none = [](const std::string &) { return 0.0; };
  for (std::size_t i = 0; i < rs.axiom.size(); ++i) {
    const module_expr &se = rs.axiom[i];
    module_t m;
    m.sym = se.sym;
    m.level = 0;
    m.path = path_id(1u, static_cast<std::uint32_t>(i));
    m.nparams = static_cast<std::uint8_t>(std::min<std::size_t>(se.params.size(), max_params));
    for (std::size_t k = 0; k < m.nparams; ++k)
      m.p[k] = se.params[k].eval(none); // axiom params are constant expressions
    out.push_back(m);
  }
  return out;
}

// One parallel generation. Returns false if the module budget was hit.
bool parallel_gen(const std::vector<module_t> &cur, std::vector<module_t> &next, const ruleset &rs,
                  std::uint64_t seed, int gen, std::size_t max_modules, std::size_t &dropped) {
  next.clear();
  next.reserve(cur.size() * 2);
  std::vector<const production *> apps;
  for (std::size_t pos = 0; pos < cur.size(); ++pos) {
    const module_t &m = cur[pos];
    apps.clear();
    for (const production &p : rs.prods)
      if (prod_applies(cur, pos, p, rs))
        apps.push_back(&p);
    if (apps.empty()) {
      next.push_back(m); // copied through, level & path preserved
      continue;
    }
    const production *p = choose(apps, m, rs, seed, gen);
    auto env = make_env(m, *p, rs);
    for (std::size_t c = 0; c < p->successor.size(); ++c) {
      if (next.size() >= max_modules) {
        dropped += (cur.size() - pos);
        return false;
      }
      next.push_back(make_module(p->successor[c], m, gen,
                                 path_id(m.path, static_cast<std::uint32_t>(c)), env));
    }
  }
  return true;
}

} // namespace

derive_result derive(const ruleset &rs, const derive_options &opt) {
  const auto t0 = std::chrono::steady_clock::now();
  derive_result r;

  std::vector<module_t> cur = axiom_to_modules(rs);

  if (rs.mode == derivation_mode::parallel) {
    for (int g = 1; g <= opt.generations; ++g) {
      std::vector<module_t> next;
      if (!parallel_gen(cur, next, rs, opt.master_seed, g, opt.max_modules, r.modules_dropped)) {
        r.truncated = true;
        cur.swap(next);
        r.generations_reached = static_cast<std::uint32_t>(g);
        break;
      }
      cur.swap(next);
      r.generations_reached = static_cast<std::uint32_t>(g);
    }
  } else {
    // Sequential-priority (Müller Algorithm 1): rewrite the leftmost
    // highest-priority applicable module, repeat until only terminals remain.
    std::uint32_t steps = 0;
    for (;;) {
      // Find the best module to rewrite.
      long best = -1;
      std::uint8_t best_prio = 0;
      std::vector<const production *> apps;
      for (std::size_t pos = 0; pos < cur.size(); ++pos) {
        for (const production &p : rs.prods) {
          if (prod_applies(cur, pos, p, rs)) {
            if (best < 0 || p.priority > best_prio) {
              best = static_cast<long>(pos);
              best_prio = p.priority;
            }
          }
        }
      }
      if (best < 0)
        break;
      const std::size_t pos = static_cast<std::size_t>(best);
      apps.clear();
      for (const production &p : rs.prods)
        if (prod_applies(cur, pos, p, rs))
          apps.push_back(&p);
      const module_t m = cur[pos];
      // Draw ordinal is a FIXED 0, not `steps`: m.path is unique per rewrite, so
      // keying on the global step counter (a loop counter, forbidden by rng.h)
      // would make an unrelated earlier rewrite move this module's stochastic
      // choice — a violation of insertion stability. See rng.h §5.2.
      const production *p = choose(apps, m, rs, opt.master_seed, 0);
      auto env = make_env(m, *p, rs);
      std::vector<module_t> repl;
      repl.reserve(p->successor.size());
      for (std::size_t c = 0; c < p->successor.size(); ++c)
        repl.push_back(make_module(p->successor[c], m, m.level + 1,
                                   path_id(m.path, static_cast<std::uint32_t>(c)), env));
      cur.erase(cur.begin() + best);
      cur.insert(cur.begin() + best, repl.begin(), repl.end());
      ++steps;
      if (steps >= opt.max_steps || cur.size() >= opt.max_modules) {
        r.truncated = true;
        break;
      }
    }
    r.generations_reached = steps;
  }

  r.w = word(std::move(cur));
  const auto t1 = std::chrono::steady_clock::now();
  r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  return r;
}

deriver::deriver(const ruleset &rs, const derive_options &opt) : rs_(rs), opt_(opt) {
  result_.w = word(axiom_to_modules(rs_));
}

bool deriver::step() {
  if (done_ || cancelled_) {
    done_ = true;
    return false;
  }
  if (rs_.mode == derivation_mode::sequential_priority) {
    // Sequential mode does not chunk cleanly; run to completion in one step.
    result_ = derive(rs_, opt_);
    done_ = true;
    return false;
  }
  ++gen_;
  std::vector<module_t> cur = result_.w.modules();
  std::vector<module_t> next;
  if (!parallel_gen(cur, next, rs_, opt_.master_seed, gen_, opt_.max_modules,
                    result_.modules_dropped)) {
    result_.truncated = true;
    done_ = true;
  }
  result_.w = word(std::move(next));
  result_.generations_reached = static_cast<std::uint32_t>(gen_);
  if (gen_ >= opt_.generations)
    done_ = true;
  return !done_;
}

} // namespace lsys
} // namespace cvc
