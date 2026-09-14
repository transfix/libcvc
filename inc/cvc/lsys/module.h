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

// module.h — the derived word: a flat, contiguous sequence of parametric
// modules. A `word` is the output of derive() and the input to interpret().
//
// The `level` field records the derivation depth at which a module first
// appeared; `filter_level(k)` selects the modules with level <= k, which — for
// a MONOTONE (gen_nested) grammar — is exactly the word at generation k, giving
// a memory-free LOD rung. The `path` field is the stable element id from
// rng::path_id, so a module's random draws do not depend on its flat position.

#ifndef CVC_LSYS_MODULE_H
#define CVC_LSYS_MODULE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace cvc {
namespace lsys {

using symbol_t = std::uint16_t;

inline constexpr int max_params = 6;
inline constexpr int max_levels = 32; // level_counts fixed width; clamps deeper levels

// A single parametric module in a derived word.
struct module_t {
  symbol_t sym = 0;
  std::uint8_t nparams = 0;
  std::uint8_t level = 0;
  std::uint32_t _pad = 0;
  double p[max_params] = {0, 0, 0, 0, 0, 0};
  std::uint64_t path = 0;
};

// A derived word. Flat and contiguous; no per-module allocation.
class word {
public:
  word() = default;
  explicit word(std::vector<module_t> m) : m_(std::move(m)) {}

  std::size_t size() const noexcept { return m_.size(); }
  bool empty() const noexcept { return m_.empty(); }
  const module_t &operator[](std::size_t i) const noexcept { return m_[i]; }
  const std::vector<module_t> &modules() const noexcept { return m_; }
  std::vector<module_t> &modules() noexcept {
    dirty_ = true;
    return m_;
  }

  void push_back(const module_t &mod) {
    m_.push_back(mod);
    dirty_ = true;
  }
  void clear() {
    m_.clear();
    dirty_ = true;
  }
  void reserve(std::size_t n) { m_.reserve(n); }

  // Modules with level <= k, in order. This is the LOD rung: it selects the
  // DRAWABLE geometry born by generation k (terminals keep their birth level;
  // non-terminals emit no geometry). NOTE: it is NOT byte-identical to the full
  // generation-k word for a recursive grammar — a frontier non-terminal created
  // at level k is rewritten at k+1 and so is absent here — but since it carries
  // no geometry, the rendered result is the coarser tree, which is the point.
  // For non-nested grammars (deletes / cut / context) even the geometry rung is
  // only an approximation and the caller should re-derive per rung.
  void filter_level(int k, std::vector<module_t> &out) const {
    out.clear();
    for (const module_t &mod : m_)
      if (static_cast<int>(mod.level) <= k)
        out.push_back(mod);
  }

  // Order-sensitive content hash over (sym, nparams, params). Excludes `level`
  // and `path` so two derivations that differ only in provenance bookkeeping
  // hash identically.
  std::uint64_t content_hash() const noexcept {
    refresh();
    return content_hash_;
  }

  // Per-level module counts (levels >= max_levels fold into the last bucket).
  const std::array<std::uint32_t, max_levels> &level_counts() const noexcept {
    refresh();
    return level_counts_;
  }

private:
  void refresh() const noexcept {
    if (!dirty_)
      return;
    std::uint64_t h = 0xcbf29ce484222325ull; // FNV-1a offset basis
    auto mix = [&h](std::uint64_t v) {
      h ^= v;
      h *= 0x100000001b3ull;
    };
    level_counts_.fill(0);
    for (const module_t &mod : m_) {
      mix(mod.sym);
      mix(mod.nparams);
      for (int i = 0; i < mod.nparams && i < max_params; ++i) {
        std::uint64_t bits = 0;
        static_assert(sizeof(bits) == sizeof(mod.p[0]), "double is 64-bit");
        std::memcpy(&bits, &mod.p[i], sizeof(bits));
        mix(bits);
      }
      const int lvl = mod.level < max_levels ? mod.level : max_levels - 1;
      ++level_counts_[static_cast<std::size_t>(lvl)];
    }
    content_hash_ = h;
    dirty_ = false;
  }

  std::vector<module_t> m_;
  mutable std::array<std::uint32_t, max_levels> level_counts_{};
  mutable std::uint64_t content_hash_ = 0;
  mutable bool dirty_ = true;
};

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_MODULE_H
