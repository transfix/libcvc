/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol's group tree (cvc/mol/groups.h), by PDBParser's rules (TexMol's
// parserPDBtoGOA::parseATOM).

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cvc/mol/groups.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace mol {

namespace {

constexpr const char *kNucleotides[] = {"A",  "C",  "G",  "T",  "U",  "I",
                                        "DA", "DC", "DG", "DT", "DU", "DI"};

bool is_nucleotide(const std::string &residue) {
  for (const char *n : kNucleotides)
    if (residue == n)
      return true;
  return false;
}

// A residue as a structure record names its ends.
struct residue_key {
  char chain = ' ';
  int seq = 0;
  char insertion = ' ';
  bool operator==(const residue_key &o) const {
    return chain == o.chain && seq == o.seq && insertion == o.insertion;
  }
};

// A sheet as PDBParser grouped SHEET records: a strand 1 starts one, and the strands after it join
// it (its last strand's end is its end).
struct sheet_group {
  int record = -1; // its first strand's record
  residue_key start, end;
  std::vector<std::pair<int, int>> strands; // residue number ranges
  std::size_t group = 0;                    // its group in the tree, once made
  bool made = false;
};

const char *kind_name(group::structure_kind k) {
  switch (k) {
  case group::structure_kind::helix:
    return "helix";
  case group::structure_kind::sheet:
    return "sheet";
  case group::structure_kind::turn:
    return "turn";
  default:
    return "none";
  }
}

char show(char c) { return c == ' ' || c == '\0' ? '_' : c; }

} // namespace

struct group_tree_stepper::impl {
  std::shared_ptr<const molecule> mol;
  group_tree out;
  std::size_t next = 0; // the next atom
  std::atomic<bool> cancel_requested{false};
  bool cancelled = false, finished = false;

  // The structure records, as PDBParser listed them: helices and turns in file order, sheets as
  // grouped. Used for the first model only.
  std::vector<int> helices, turns;
  std::vector<sheet_group> sheets;

  // Where the walk is.
  bool started = false;
  int model_value = 0;
  std::size_t model = 0, chain = 0, residue = 0;
  bool have_chain = false, have_residue = false;
  char chain_id = ' ';
  int seq = 0;
  char insertion = ' ';
  bool first_model = true;
  std::size_t helix_next = 0, turn_next = 0;
  long current = -1;       // the current structure group, -1 for none
  long current_sheet = -1; // when the current group is a sheet: its sheets index

  explicit impl(std::shared_ptr<const molecule> m) : mol(std::move(m)) {
    if (!mol)
      throw std::invalid_argument("build_group_tree: no molecule");
    for (std::size_t i = 0; i < mol->structures.size(); ++i) {
      const secondary_structure &s = mol->structures[i];
      const int r = static_cast<int>(i);
      if (s.type == secondary_structure::kind::helix) {
        helices.push_back(r);
      } else if (s.type == secondary_structure::kind::turn) {
        turns.push_back(r);
      } else if (s.strand == 1 || sheets.empty()) {
        sheet_group g;
        g.record = r;
        g.start = {s.chain, s.start_seq, s.start_insertion};
        g.end = {s.chain, s.end_seq, s.end_insertion};
        g.strands.push_back({s.start_seq, s.end_seq});
        sheets.push_back(g);
      } else {
        sheets.back().strands.push_back({s.start_seq, s.end_seq});
        sheets.back().end = {s.chain, s.end_seq, s.end_insertion};
      }
    }
  }

  std::size_t add(group g, std::size_t parent) {
    out.groups.push_back(std::move(g));
    const std::size_t i = out.groups.size() - 1;
    out.groups[parent].children.push_back(i);
    return i;
  }

  std::size_t new_structure(group::structure_kind k, int record) {
    group g;
    g.type = group::kind::structure;
    g.structure = k;
    g.record = record;
    g.model = model_value;
    g.chain = chain_id;
    return add(std::move(g), chain);
  }

  const secondary_structure &rec(int i) const {
    return mol->structures[static_cast<std::size_t>(i)];
  }

  // A residue begins: which structure group it joins (parseATOM's residue section).
  void place_residue(const residue_key &key) {
    group::structure_kind kind = group::structure_kind::none;
    if (current >= 0)
      kind = out.groups[static_cast<std::size_t>(current)].structure;
    bool in_sheet = false;
    if (current >= 0 &&
        (kind == group::structure_kind::helix || kind == group::structure_kind::turn)) {
      // An open helix or turn takes every residue until its last.
    } else if (first_model) {
      if (helix_next < helices.size()) {
        const secondary_structure &h = rec(helices[helix_next]);
        if (residue_key{h.chain, h.start_seq, h.start_insertion} == key) {
          current =
              static_cast<long>(new_structure(group::structure_kind::helix, helices[helix_next]));
          ++helix_next;
        }
      }
      if (turn_next < turns.size()) {
        const secondary_structure &t = rec(turns[turn_next]);
        if (residue_key{t.chain, t.start_seq, t.start_insertion} == key) {
          current = static_cast<long>(new_structure(group::structure_kind::turn, turns[turn_next]));
          ++turn_next;
        }
      }
      // A residue in a helix's range never joins a sheet.
      bool in_helix = false;
      for (int h : helices) {
        const secondary_structure &s = rec(h);
        if (s.chain == key.chain && s.start_seq <= key.seq && s.end_seq >= key.seq)
          in_helix = true;
      }
      if (!in_helix)
        for (std::size_t i = 0; i < sheets.size() && !in_sheet; ++i) {
          sheet_group &sg = sheets[i];
          for (const auto &[from, to] : sg.strands)
            if (key.seq >= from && key.seq <= to && sg.start.chain == key.chain)
              in_sheet = true;
          if (!in_sheet)
            continue;
          // One group a sheet, in the chain once.
          if (!sg.made ||
              std::find(out.groups[chain].children.begin(), out.groups[chain].children.end(),
                        sg.group) == out.groups[chain].children.end()) {
            if (!sg.made) {
              sg.group = new_structure(group::structure_kind::sheet, sg.record);
              sg.made = true;
            } else {
              out.groups[chain].children.push_back(sg.group);
            }
          }
          current = static_cast<long>(sg.group);
          current_sheet = static_cast<long>(i);
        }
    }
    if (current >= 0)
      kind = out.groups[static_cast<std::size_t>(current)].structure;
    // No structure, or a sheet the residue is not in: a `none` group (an open one is reused).
    if (current < 0 || (kind == group::structure_kind::sheet && !in_sheet)) {
      current = static_cast<long>(new_structure(group::structure_kind::none, -1));
      current_sheet = -1;
      kind = group::structure_kind::none;
    }
    group g;
    g.type = group::kind::residue;
    g.model = model_value;
    g.chain = key.chain;
    g.residue_seq = key.seq;
    g.insertion = key.insertion;
    residue = add(std::move(g), static_cast<std::size_t>(current));
    have_residue = true;
    // A structure ends at its last residue (a sheet at its last strand's end).
    if (kind != group::structure_kind::none) {
      residue_key end;
      if (kind == group::structure_kind::sheet) {
        end = sheets[static_cast<std::size_t>(current_sheet)].end;
      } else {
        const secondary_structure &s = rec(out.groups[static_cast<std::size_t>(current)].record);
        end = {s.chain, s.end_seq, s.end_insertion};
      }
      if (end == key) {
        current = -1;
        current_sheet = -1;
      }
    }
  }

  void atom_step(std::size_t i) {
    const atom &a = mol->atoms[i];
    if (!started) {
      group root;
      root.type = group::kind::collection;
      out.groups.push_back(std::move(root));
      started = true;
    }
    if (out.groups[0].children.empty() || a.model != model_value) {
      if (!out.groups[0].children.empty())
        first_model = false; // PDBParser forgot the structures after the first model
      model_value = a.model;
      group g;
      g.type = group::kind::model;
      g.model = a.model;
      model = add(std::move(g), 0);
      have_chain = have_residue = false;
      current = current_sheet = -1;
    }
    bool new_chain = !have_chain || a.chain != chain_id;
    if (new_chain) {
      chain_id = a.chain;
      group g;
      g.type = group::kind::chain;
      g.model = model_value;
      g.chain = a.chain;
      g.nucleic = is_nucleotide(a.residue);
      chain = add(std::move(g), model);
      have_chain = true;
      have_residue = false;
      current = current_sheet = -1;
    }
    if (!have_residue || a.residue_seq != seq || a.insertion != insertion) {
      seq = a.residue_seq;
      insertion = a.insertion;
      place_residue({a.chain, a.residue_seq, a.insertion});
      out.groups[residue].residue = a.residue;
    }
    out.groups[residue].atoms.push_back(i);
  }
};

group_tree_stepper::group_tree_stepper(std::shared_ptr<const molecule> m)
    : _impl(std::make_unique<impl>(std::move(m))) {}
group_tree_stepper::~group_tree_stepper() = default;
group_tree_stepper::group_tree_stepper(group_tree_stepper &&) noexcept = default;
group_tree_stepper &group_tree_stepper::operator=(group_tree_stepper &&) noexcept = default;

bool group_tree_stepper::step(std::size_t budget) {
  impl &d = *_impl;
  if (d.finished || d.cancelled)
    return true;
  if (d.cancel_requested.load(std::memory_order_acquire)) {
    d.cancelled = true;
    d.out = group_tree();
    return true;
  }
  std::size_t left = budget == 0 ? std::numeric_limits<std::size_t>::max() : budget;
  const std::size_t n = d.mol->atoms.size();
  while (d.next < n && left > 0) {
    d.atom_step(d.next);
    ++d.next;
    --left;
  }
  if (d.next >= n)
    d.finished = true;
  return d.finished;
}

bool group_tree_stepper::done() const { return _impl->finished; }

const char *group_tree_stepper::phase() const {
  return _impl->cancelled ? "cancelled" : _impl->finished ? "done" : "grouping";
}

double group_tree_stepper::progress() const {
  const impl &d = *_impl;
  if (d.finished)
    return 1.0;
  return d.mol->atoms.empty()
             ? 0.0
             : static_cast<double>(d.next) / static_cast<double>(d.mol->atoms.size());
}

void group_tree_stepper::cancel() {
  _impl->cancel_requested.store(true, std::memory_order_release);
}

group_tree group_tree_stepper::take() {
  if (!_impl->finished)
    throw std::logic_error("group_tree_stepper::take: not done");
  return std::move(_impl->out);
}

group_tree build_group_tree(const molecule &m) {
  group_tree_stepper s(std::make_shared<const molecule>(m));
  s.step(0);
  return s.take();
}

std::unique_ptr<result_job<group_tree>> group_tree_job(std::shared_ptr<const molecule> m) {
  return make_result_job(std::make_unique<group_tree_stepper>(std::move(m)));
}

std::vector<std::size_t> flatten_atoms(const group_tree &t) {
  std::vector<std::size_t> out;
  if (t.groups.empty() || t.groups[0].children.empty())
    return out;
  // Depth first from the first model, each group's children before its own atoms.
  std::vector<std::pair<std::size_t, std::size_t>> stack{{t.groups[0].children[0], 0}};
  while (!stack.empty()) {
    auto &[g, next] = stack.back();
    const group &grp = t.groups[g];
    if (next < grp.children.size()) {
      const std::size_t c = grp.children[next++];
      stack.push_back({c, 0});
      continue;
    }
    out.insert(out.end(), grp.atoms.begin(), grp.atoms.end());
    stack.pop_back();
  }
  return out;
}

std::string describe(const group_tree &t) {
  std::string out;
  if (t.groups.empty())
    return out;
  char b[160];
  std::vector<std::pair<std::size_t, int>> stack{{0, 0}};
  while (!stack.empty()) {
    const auto [g, depth] = stack.back();
    stack.pop_back();
    const group &grp = t.groups[g];
    const std::string pad(static_cast<std::size_t>(depth), ' ');
    switch (grp.type) {
    case group::kind::collection:
      std::snprintf(b, sizeof b, "collection %zu", grp.children.size());
      break;
    case group::kind::model:
      std::snprintf(b, sizeof b, "model %zu", grp.children.size());
      break;
    case group::kind::chain:
      std::snprintf(b, sizeof b, "chain %c %s %zu", show(grp.chain),
                    grp.nucleic ? "nucleic" : "protein", grp.children.size());
      break;
    case group::kind::structure:
      std::snprintf(b, sizeof b, "%s %zu", kind_name(grp.structure), grp.children.size());
      break;
    case group::kind::residue:
      std::snprintf(b, sizeof b, "residue %c %d %c %zu", show(grp.chain), grp.residue_seq,
                    show(grp.insertion), grp.atoms.size());
      break;
    }
    out += pad + b + "\n";
    for (std::size_t i = grp.children.size(); i-- > 0;)
      stack.push_back({grp.children[i], depth + 1});
  }
  return out;
}

} // namespace mol
} // namespace cvc
