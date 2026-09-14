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

// derive.h — apply a ruleset to produce a word.
//
// Determinism: every stochastic choice (production selection, param jitter) is a
// pure function of (master_seed, stream, module.path, generation) via the hashed
// RNG — there is no sequential generator state, so a word is a function of its
// seed and generation count, not of iteration order. Two consequences the tests
// enforce: byte-identical re-derivation, and salt-audit independence.

#ifndef CVC_LSYS_DERIVE_H
#define CVC_LSYS_DERIVE_H

#include <cstddef>
#include <cstdint>
#include <cvc/lsys/context.h>
#include <cvc/lsys/grammar.h>
#include <cvc/lsys/module.h>

namespace cvc {
namespace lsys {

struct derive_options {
  std::uint64_t master_seed = 0;
  int generations = 6;
  double fractional = 0.0;           // reserved (Houdini-style fractional gen)
  std::size_t max_modules = 200000;  // HARD budget; truncation is REPORTED
  std::uint32_t max_steps = 4000000; // sequential-mode application cap
  const context_provider *ctx = nullptr;
};

struct derive_result {
  word w;
  bool truncated = false;
  std::size_t modules_dropped = 0;
  std::uint32_t generations_reached = 0;
  double ms = 0.0;
};

derive_result derive(const ruleset &rs, const derive_options &opt);

// Resumable driver for the wasm render loop and for cancellation. step() advances
// one generation (parallel) or a bounded chunk of applications (sequential) and
// returns false when the derivation is complete.
class deriver {
public:
  deriver(const ruleset &rs, const derive_options &opt);
  bool step();
  void cancel() noexcept { cancelled_ = true; }
  const derive_result &result() const noexcept { return result_; }

private:
  const ruleset &rs_;
  derive_options opt_;
  derive_result result_;
  int gen_ = 0;
  bool done_ = false;
  bool cancelled_ = false;
};

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_DERIVE_H
