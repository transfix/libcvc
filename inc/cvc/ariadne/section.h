/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef __CVC_ARIADNE_SECTION_H__
#define __CVC_ARIADNE_SECTION_H__

#include <cstdint>
#include <initializer_list>
#include <string>

namespace cvc {
class app;
namespace ariadne {

// ----------------
// Revisioned sections (ARIADNE_WORLDS.md §3)
// ----------------
// A section is the leaves under one base path of the state tree that a reader takes whole: the
// world's physics, the avatars' archetypes, their policy, the controllers. Its writer sets the
// leaves, then commits, which writes three leaves of its own, the revision last:
//
//   <base>.epoch   the writer's run. A restarted relay's revisions start again at 1, so a reader
//                  that outlived it tells the new section from the old by its epoch.
//   <base>.at      the world-clock tick the section takes effect on (0: from the start). Sections
//                  that change the simulation carry one, so every holon switches on the same tick.
//   <base>.rev     bumped by every commit, written last: a reader that sees a new revision sees
//                  everything it covers (the leaves replicate in the order they were written).
//
// Each leaf write journals one mutation when its value changes and none when it does not; a new or
// still empty leaf is touched first (the distributed-state adapter's attach race).

struct section_stamp {
  std::uint64_t epoch = 0;
  std::uint64_t rev = 0; // 0: never written
  std::uint64_t at = 0;
  bool written() const noexcept { return rev != 0; }
  // The same version: a reader re-reads the section only when this is false.
  bool same(const section_stamp &o) const noexcept { return epoch == o.epoch && rev == o.rev; }
};

// APP's epoch: chosen once per app, at random, non-zero and below 2^53 (a DSL reads it as a
// number). A writer that starts again (a new app) starts a new run.
std::uint64_t section_epoch(cvc::app &app);

// The stamp under BASE (all zero when the section was never written).
section_stamp read_section_stamp(cvc::app &app, const std::string &base);

class section_writer {
public:
  section_writer(cvc::app &app, std::string base);

  // BASE.<rel...>, joined with cvc::state::SEPARATOR.
  std::string path(std::initializer_list<std::string> rel) const;

  void set(std::initializer_list<std::string> rel, const std::string &value);
  void set_double(std::initializer_list<std::string> rel, double v); // round-trip exact
  void set_uint(std::initializer_list<std::string> rel, std::uint64_t v);
  void set_flag(std::initializer_list<std::string> rel, bool b); // "1" | "0"

  // Write the epoch (EPOCH, or the app's when 0), AT, then the revision: one more than the
  // last, or 1 when the epoch differs from the one the section had (a new run). Returns the stamp.
  section_stamp commit(std::uint64_t at = 0, std::uint64_t epoch = 0);

private:
  cvc::app &_app;
  std::string _base;
};

// Reads the leaves under BASE; each getter returns DEF for a missing or empty leaf.
class section_reader {
public:
  section_reader(cvc::app &app, std::string base);

  std::string path(std::initializer_list<std::string> rel) const;
  bool has(std::initializer_list<std::string> rel) const; // present and not empty

  std::string text(std::initializer_list<std::string> rel, const std::string &def = {}) const;
  double number(std::initializer_list<std::string> rel, double def) const;
  std::uint64_t uint(std::initializer_list<std::string> rel, std::uint64_t def) const;
  bool flag(std::initializer_list<std::string> rel, bool def) const;

private:
  cvc::app &_app;
  std::string _base;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SECTION_H__
