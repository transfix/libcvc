/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol: the molecule model and its readers (molecule.h). The PDB reader follows TexMol's
// parserPDBtoGOA (transfix/TexMol ed20eae, src/PDBParser) in what it keeps and how it assigns
// radii, but reads the PDB's columns itself and reports a malformed line instead of reading it as
// zeros.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/param_table.h> // format_double
#include <cvc/mol/molecule.h>
#include <filesystem>
#include <fstream>
#include <istream>
#include <limits>
#include <map>
#include <ostream>
#include <set>
#include <sstream>
#include <utility>

namespace cvc {
namespace mol {

namespace {

namespace fs = std::filesystem;

constexpr residue_atom_info kTable[] = {
#include "residue_atom_table.inc"
};
constexpr std::size_t kRows = sizeof(kTable) / sizeof(kTable[0]);

std::string_view trim(std::string_view s) {
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t'))
    s.remove_prefix(1);
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r'))
    s.remove_suffix(1);
  return s;
}

std::string join(const std::vector<std::string> &v, const char *sep) {
  std::string out;
  for (std::size_t i = 0; i < v.size(); ++i)
    out += (i ? sep : "") + v[i];
  return out;
}

std::string lower(std::string s) {
  for (char &c : s)
    if (c >= 'A' && c <= 'Z')
      c = static_cast<char>(c - 'A' + 'a');
  return s;
}

std::string extension_of(const std::string &filename) {
  const std::string name = fs::path(filename).filename().string();
  const std::size_t dot = name.find_last_of('.');
  return dot == std::string::npos || dot == 0 ? std::string() : lower(name.substr(dot));
}

// Columns `first` to `last` of a PDB line, 1-based and inclusive as the format documents them;
// what of them the line has.
std::string_view cols(std::string_view line, std::size_t first, std::size_t last) {
  if (first > line.size())
    return {};
  const std::size_t from = first - 1;
  return line.substr(from, std::min(last, line.size()) - from);
}

char col(std::string_view line, std::size_t c) { return c <= line.size() ? line[c - 1] : ' '; }

bool to_int(std::string_view s, int &out) {
  s = trim(s);
  if (s.empty())
    return false;
  const std::string t(s);
  char *end = nullptr;
  const long v = std::strtol(t.c_str(), &end, 10);
  if (end != t.c_str() + t.size() || v < std::numeric_limits<int>::min() ||
      v > std::numeric_limits<int>::max())
    return false;
  out = static_cast<int>(v);
  return true;
}

bool to_double(std::string_view s, double &out) {
  s = trim(s);
  if (s.empty())
    return false;
  const std::string t(s);
  char *end = nullptr;
  const double v = std::strtod(t.c_str(), &end);
  if (end != t.c_str() + t.size() || !std::isfinite(v))
    return false;
  out = v;
  return true;
}

bool is_alpha(char c) { return std::isalpha(static_cast<unsigned char>(c)) != 0; }

char upper(char c) { return static_cast<char>(std::toupper(static_cast<unsigned char>(c))); }

// The element an atom name implies, by the PDB's convention for its columns 13-16: a one-letter
// element is in column 14 (column 13 blank or a digit), a two-letter one in 13-14. A name that
// fills all four columns starting with H, in a residue the table knows, is a hydrogen (old files
// wrote HG21 for a valine's hydrogen, not mercury).
std::string element_from_name(std::string_view field, std::string_view residue) {
  if (field.size() < 2)
    return field.empty() || !is_alpha(field[0]) ? std::string() : std::string(1, upper(field[0]));
  const char c0 = field[0], c1 = field[1];
  if (c0 == ' ' || std::isdigit(static_cast<unsigned char>(c0)))
    return is_alpha(c1) ? std::string(1, upper(c1)) : std::string();
  if (!is_alpha(c0))
    return std::string();
  if (upper(c0) == 'H' || upper(c0) == 'D') {
    bool known = false;
    for (std::size_t r = 1; r < kRows && !known; ++r)
      known = trim(kTable[r].residue) == trim(residue);
    if (known)
      return std::string(1, upper(c0));
  }
  return is_alpha(c1) ? std::string{upper(c0), upper(c1)} : std::string(1, upper(c0));
}

bool is_water(const std::string &residue) {
  return residue == "HOH" || residue == "WAT" || residue == "DOD";
}

bool is_hydrogen(const std::string &element) { return element == "H" || element == "D"; }

// The table's radius and charge for an atom read from a PDB file.
void assign_from_table(atom &a, const molecule_read_params &p) {
  a.table_row = find_residue_atom(a.name, a.residue);
  if (a.table_row < 0) {
    a.radius = p.unknown_radius;
    a.charge = 0.0;
    return;
  }
  const residue_atom_info &r = kTable[static_cast<std::size_t>(a.table_row)];
  a.radius = p.radii == radius_kind::solvent_enlarged ? r.solvent_enlarged_radius : r.radius;
  a.charge = r.charge;
}

// What read_pdb and read_pqr share: the model they are in, which atoms to keep, and alternate
// locations.
class atom_filter {
public:
  explicit atom_filter(const molecule_read_params &p) : _p(p) {}

  // A MODEL record: the next model, counting from 1 in file order (its serial is a label).
  void model() { ++_model; }
  int models() const { return _model; }
  // The model an atom read now is in: 0 before any MODEL record, else the last one's number
  // (an atom after ENDMDL and before the next MODEL stays in the one that ended).
  int current_model() const { return _model; }

  bool keep(const atom &a) {
    if (_p.model > 0 && a.model != 0 && a.model != _p.model)
      return false;
    if (_p.model > 1 && a.model == 0) // a file without MODEL records has only model 1
      return false;
    if (!_p.hetero && a.hetero)
      return false;
    if (!_p.water && is_water(a.residue))
      return false;
    if (!_p.hydrogens && is_hydrogen(a.element))
      return false;
    if (!_p.alt_locs && a.alt_loc != ' ') {
      std::string key = std::to_string(a.model) + '|' + a.chain + '|' +
                        std::to_string(a.residue_seq) + a.insertion + '|' + a.name;
      const auto it = _alt.find(key);
      if (it == _alt.end())
        _alt.emplace(std::move(key), a.alt_loc);
      else if (it->second != a.alt_loc)
        return false;
    }
    return true;
  }

private:
  const molecule_read_params &_p;
  int _model = 0;
  std::map<std::string, char> _alt;
};

void check_params(const char *who, const molecule_read_params &p) {
  const std::vector<std::string> bad = params_validate(p);
  if (!bad.empty())
    throw std::invalid_argument(std::string(who) + ": " + join(bad, "; "));
}

std::string where(const std::string &name, std::size_t line) {
  return name + ":" + std::to_string(line);
}

// A residue number with its insertion code, as PQR files write it ("52" or "52A").
bool residue_with_insertion(std::string_view s, int &seq, char &insertion) {
  insertion = ' ';
  if (!s.empty() && is_alpha(s.back())) {
    insertion = s.back();
    s.remove_suffix(1);
  }
  return to_int(s, seq);
}

// Write `m` to `filename` with `writer`, saying which file when it can't.
void write_to(const std::string &filename, const molecule &m,
              void (*writer)(std::ostream &, const molecule &)) {
  std::ofstream out(fs::path(filename), std::ios::binary);
  if (!out)
    throw molecule_file_error("write_molecule: can't open '" + filename + "' to write");
  writer(out, m);
  out.flush();
  if (!out)
    throw molecule_file_error("write_molecule: can't write '" + filename + "'");
}

// A residue's name, looked up in the atoms by chain, number and insertion code ("" when none has
// it): what a HELIX, SHEET or TURN record names its ends by.
std::string residue_name_at(const molecule &m, char chain, int seq, char insertion) {
  for (const atom &a : m.atoms)
    if (a.chain == chain && a.residue_seq == seq && a.insertion == insertion)
      return a.residue;
  return std::string();
}

// An atom's name in PDB columns 13-16: a four-character name fills them; a shorter one starts in
// column 14 when its element is one letter (" CA " for a C-alpha), else in column 13 ("FE  ").
std::string pdb_atom_name(const atom &a) {
  std::string n = a.name.substr(0, 4);
  if (n.size() < 4 && a.element.size() <= 1)
    n = " " + n;
  n.resize(4, ' ');
  return n;
}

// Call f(model) for each model the atoms come from, in the order they first come.
template <class F> void each_model(const molecule &m, F &&f) {
  std::vector<int> models;
  for (const atom &a : m.atoms)
    if (std::find(models.begin(), models.end(), a.model) == models.end())
      models.push_back(a.model);
  for (int model : models)
    f(model);
}

class pdb_io : public molecule_file_io {
public:
  bool can_write() const override { return true; }
  void write(app &, const molecule &m, const std::string &filename) const override {
    write_to(filename, m, write_pdb);
  }
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  molecule read(app &, const std::string &filename, const molecule_read_params &p) const override {
    std::ifstream in(fs::path(filename), std::ios::binary);
    if (!in)
      throw molecule_file_error("read_molecule: can't open '" + filename + "'");
    return read_pdb(in, p, filename);
  }

private:
  std::string _id = "pdb : v1";
  std::vector<std::string> _ext{".pdb", ".ent", ".pdb1"};
};

class pqr_io : public molecule_file_io {
public:
  bool can_write() const override { return true; }
  void write(app &, const molecule &m, const std::string &filename) const override {
    write_to(filename, m, write_pqr);
  }
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  molecule read(app &, const std::string &filename, const molecule_read_params &p) const override {
    std::ifstream in(fs::path(filename), std::ios::binary);
    if (!in)
      throw molecule_file_error("read_molecule: can't open '" + filename + "'");
    return read_pqr(in, p, filename);
  }

private:
  std::string _id = "pqr : v1";
  std::vector<std::string> _ext{".pqr"};
};

class xyzr_io : public molecule_file_io {
public:
  bool can_write() const override { return true; }
  void write(app &, const molecule &m, const std::string &filename) const override {
    write_to(filename, m, write_xyzr);
  }
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  molecule read(app &, const std::string &filename, const molecule_read_params &) const override {
    std::ifstream in(fs::path(filename), std::ios::binary);
    if (!in)
      throw molecule_file_error("read_molecule: can't open '" + filename + "'");
    return read_xyzr(in, filename);
  }

private:
  std::string _id = "xyzr : v1";
  std::vector<std::string> _ext{".xyzr"};
};

class pts_io : public molecule_file_io {
public:
  bool can_write() const override { return true; }
  void write(app &, const molecule &m, const std::string &filename) const override {
    write_to(filename, m, write_pts);
  }
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  molecule read(app &, const std::string &filename, const molecule_read_params &) const override {
    std::ifstream in(fs::path(filename), std::ios::binary);
    if (!in)
      throw molecule_file_error("read_molecule: can't open '" + filename + "'");
    return read_pts(in, filename);
  }

private:
  std::string _id = "pts : v1";
  std::vector<std::string> _ext{".pts"};
};

// The app's molecule handlers: app.slot<T>() makes one per app, with the built-in formats, the
// first time it is asked (§1.3: no process-wide table).
struct molecule_io_slot {
  static constexpr const char *app_slot = "cvc.mol.molecule_handlers";
  file_handler_registry<molecule_file_io> handlers;
  molecule_io_slot() {
    handlers.insert(boost::shared_ptr<molecule_file_io>(new pdb_io));
    handlers.insert(boost::shared_ptr<molecule_file_io>(new pqr_io));
    handlers.insert(boost::shared_ptr<molecule_file_io>(new xyzr_io));
    handlers.insert(boost::shared_ptr<molecule_file_io>(new pts_io));
  }
};

} // namespace

// ---- the table ----

const residue_atom_info *residue_atom_table(std::size_t *rows) {
  if (rows)
    *rows = kRows;
  return kTable;
}

int find_residue_atom(std::string_view atom_name, std::string_view residue) {
  atom_name = trim(atom_name);
  residue = trim(residue);
  if (atom_name.empty() || residue.empty())
    return -1;
  for (std::size_t r = 0; r < kRows; ++r)
    if (trim(kTable[r].atom) == atom_name && trim(kTable[r].residue) == residue)
      return static_cast<int>(r);
  return -1;
}

// ---- the types ----

bool secondary_structure::contains(char c, int seq, char insertion) const {
  if (c != chain)
    return false;
  const auto key = [](int s, char i) { return std::pair<int, char>(s, i == ' ' ? '\0' : i); };
  const auto k = key(seq, insertion);
  return key(start_seq, start_insertion) <= k && k <= key(end_seq, end_insertion);
}

std::array<double, 6> molecule::bounds(bool with_radii) const {
  std::array<double, 6> b{0, 0, 0, 0, 0, 0};
  bool first = true;
  for (const atom &a : atoms) {
    const double r = with_radii ? a.radius : 0.0;
    const double lo[3] = {a.x - r, a.y - r, a.z - r}, hi[3] = {a.x + r, a.y + r, a.z + r};
    for (int k = 0; k < 3; ++k) {
      b[k] = first ? lo[k] : std::min(b[k], lo[k]);
      b[k + 3] = first ? hi[k] : std::max(b[k + 3], hi[k]);
    }
    first = false;
  }
  return b;
}

int molecule::structure_of(const atom &a) const {
  for (std::size_t s = 0; s < structures.size(); ++s)
    if (structures[s].contains(a.chain, a.residue_seq, a.insertion))
      return static_cast<int>(s);
  return -1;
}

// ---- the readers ----

molecule read_pdb(std::istream &in, const molecule_read_params &p, const std::string &name) {
  check_params("read_pdb", p);
  molecule m;
  atom_filter filter(p);
  std::string raw;
  std::size_t n = 0;
  const auto num = [&](std::string_view field, const char *what, double fallback,
                       bool required) -> double {
    double v = fallback;
    if (trim(field).empty()) {
      if (required)
        throw molecule_file_error(where(name, n) + ": the atom has no " + what);
      return v;
    }
    if (!to_double(field, v))
      throw molecule_file_error(where(name, n) + ": the atom's " + what + " '" +
                                std::string(trim(field)) + "' is not a number");
    return v;
  };
  const auto integer = [&](std::string_view field, const char *what) -> int {
    int v = 0;
    if (trim(field).empty())
      return 0;
    if (!to_int(field, v))
      throw molecule_file_error(where(name, n) + ": " + what + " '" + std::string(trim(field)) +
                                "' is not a whole number");
    return v;
  };
  while (std::getline(in, raw)) {
    ++n;
    std::string_view line(raw);
    if (const std::size_t cr = line.find('\r'); cr != std::string_view::npos)
      line = line.substr(0, cr);
    if (line.size() < 3)
      continue;
    const std::string_view rec = cols(line, 1, 6);
    if (rec.substr(0, 4) == "ATOM" || rec == "HETATM") {
      atom a;
      a.hetero = rec == "HETATM";
      // The serial is a label only; a file past 99,999 atoms may write it in hybrid-36.
      if (!to_int(cols(line, 7, 11), a.serial))
        a.serial = static_cast<int>(m.atoms.size()) + 1;
      const std::string_view name_field = cols(line, 13, 16);
      a.name = std::string(trim(name_field));
      a.alt_loc = col(line, 17);
      a.residue = std::string(trim(cols(line, 18, 20)));
      a.chain = col(line, 22);
      a.residue_seq = integer(cols(line, 23, 26), "the residue number");
      a.insertion = col(line, 27);
      a.x = num(cols(line, 31, 38), "x", 0.0, true);
      a.y = num(cols(line, 39, 46), "y", 0.0, true);
      a.z = num(cols(line, 47, 54), "z", 0.0, true);
      a.occupancy = num(cols(line, 55, 60), "occupancy", 1.0, false);
      a.b_factor = num(cols(line, 61, 66), "temperature factor", 0.0, false);
      const std::string_view el = trim(cols(line, 77, 78));
      if (!el.empty() && std::all_of(el.begin(), el.end(), is_alpha)) {
        a.element.clear();
        for (char c : el)
          a.element += upper(c);
      } else {
        a.element = element_from_name(name_field, a.residue);
      }
      a.model = filter.current_model();
      if (!filter.keep(a))
        continue;
      assign_from_table(a, p);
      m.atoms.push_back(std::move(a));
    } else if (rec.substr(0, 5) == "MODEL") {
      filter.model();
      m.models = filter.models();
    } else if (rec.substr(0, 5) == "HELIX") {
      secondary_structure s;
      s.type = secondary_structure::kind::helix;
      s.id = std::string(trim(cols(line, 12, 14)));
      s.chain = col(line, 20);
      s.start_seq = integer(cols(line, 22, 25), "HELIX's first residue");
      s.start_insertion = col(line, 26);
      s.end_seq = integer(cols(line, 34, 37), "HELIX's last residue");
      s.end_insertion = col(line, 38);
      s.helix_class = integer(cols(line, 39, 40), "HELIX's class");
      m.structures.push_back(std::move(s));
    } else if (rec.substr(0, 5) == "SHEET") {
      secondary_structure s;
      s.type = secondary_structure::kind::sheet;
      s.strand = integer(cols(line, 8, 10), "SHEET's strand");
      s.id = std::string(trim(cols(line, 12, 14)));
      s.chain = col(line, 22);
      s.start_seq = integer(cols(line, 23, 26), "SHEET's first residue");
      s.start_insertion = col(line, 27);
      s.end_seq = integer(cols(line, 34, 37), "SHEET's last residue");
      s.end_insertion = col(line, 38);
      s.sense = integer(cols(line, 39, 40), "SHEET's sense");
      m.structures.push_back(std::move(s));
    } else if (rec.substr(0, 4) == "TURN") {
      secondary_structure s;
      s.type = secondary_structure::kind::turn;
      s.id = std::string(trim(cols(line, 12, 14)));
      s.chain = col(line, 20);
      s.start_seq = integer(cols(line, 21, 24), "TURN's first residue");
      s.start_insertion = col(line, 25);
      s.end_seq = integer(cols(line, 32, 35), "TURN's last residue");
      s.end_insertion = col(line, 36);
      m.structures.push_back(std::move(s));
    } else if (rec == "HEADER") {
      m.id = std::string(trim(cols(line, 63, 66)));
    } else if (rec.substr(0, 5) == "TITLE") {
      const std::string_view t = trim(cols(line, 11, 80));
      if (!t.empty())
        m.title += (m.title.empty() ? "" : " ") + std::string(t);
    }
  }
  if (in.bad())
    throw molecule_file_error(name + ": can't read it");
  return m;
}

molecule read_pqr(std::istream &in, const molecule_read_params &p, const std::string &name) {
  check_params("read_pqr", p);
  molecule m;
  atom_filter filter(p);
  std::string raw;
  std::size_t n = 0;
  while (std::getline(in, raw)) {
    ++n;
    std::istringstream ss(raw);
    std::vector<std::string> t;
    for (std::string w; ss >> w;)
      t.push_back(w);
    if (t.empty())
      continue;
    if (t[0] == "MODEL") {
      filter.model();
      m.models = filter.models();
      continue;
    }
    if (t[0] != "ATOM" && t[0] != "HETATM")
      continue;
    // ATOM serial name residue [chain] residue_seq x y z charge radius
    if (t.size() != 10 && t.size() != 11)
      throw molecule_file_error(where(name, n) + ": an ATOM line has " + std::to_string(t.size()) +
                                " fields, not 10 or 11 (record serial name residue [chain] "
                                "residue_seq x y z charge radius)");
    const std::size_t k = t.size();
    atom a;
    a.hetero = t[0] == "HETATM";
    if (!to_int(t[1], a.serial))
      a.serial = static_cast<int>(m.atoms.size()) + 1;
    a.name = t[2];
    a.residue = t[3];
    if (k == 11) {
      if (t[4].size() != 1)
        throw molecule_file_error(where(name, n) + ": the chain '" + t[4] +
                                  "' is not one character");
      a.chain = t[4][0];
    }
    if (!residue_with_insertion(t[k - 6], a.residue_seq, a.insertion))
      throw molecule_file_error(where(name, n) + ": the residue number '" + t[k - 6] +
                                "' is not a whole number");
    double *const v[5] = {&a.x, &a.y, &a.z, &a.charge, &a.radius};
    const char *const what[5] = {"x", "y", "z", "charge", "radius"};
    for (int i = 0; i < 5; ++i)
      if (!to_double(t[k - 5 + static_cast<std::size_t>(i)], *v[i]))
        throw molecule_file_error(where(name, n) + ": the atom's " + what[i] + " '" +
                                  t[k - 5 + static_cast<std::size_t>(i)] + "' is not a number");
    if (a.radius < 0)
      throw molecule_file_error(where(name, n) + ": the atom's radius is negative");
    const std::string padded = a.name.size() < 4 ? " " + a.name : a.name;
    a.element = element_from_name(padded, a.residue);
    a.model = filter.current_model();
    if (filter.keep(a))
      m.atoms.push_back(std::move(a));
  }
  if (in.bad())
    throw molecule_file_error(name + ": can't read it");
  return m;
}

molecule read_xyzr(std::istream &in, const std::string &name) {
  molecule m;
  std::string raw;
  std::size_t n = 0;
  while (std::getline(in, raw)) {
    ++n;
    const std::string_view line = trim(raw);
    if (line.empty() || line.front() == '#')
      continue;
    std::istringstream ss{std::string(line)};
    std::vector<std::string> t;
    for (std::string w; ss >> w;)
      t.push_back(w);
    if (t.size() != 4)
      throw molecule_file_error(where(name, n) + ": " + std::to_string(t.size()) +
                                " fields, not four (x y z radius)");
    atom a;
    a.serial = static_cast<int>(m.atoms.size()) + 1;
    double *const v[4] = {&a.x, &a.y, &a.z, &a.radius};
    for (int i = 0; i < 4; ++i)
      if (!to_double(t[static_cast<std::size_t>(i)], *v[i]))
        throw molecule_file_error(where(name, n) + ": '" + t[static_cast<std::size_t>(i)] +
                                  "' is not a number");
    if (a.radius < 0)
      throw molecule_file_error(where(name, n) + ": the radius is negative");
    m.atoms.push_back(std::move(a));
  }
  if (in.bad())
    throw molecule_file_error(name + ": can't read it");
  return m;
}

// ---- files ----

molecule read_pts(std::istream &in, const std::string &name) {
  molecule m;
  std::string raw;
  std::size_t n = 0;
  bool counted = false;
  while (std::getline(in, raw)) {
    ++n;
    const std::string_view line = trim(raw);
    if (line.empty() || line.front() == '#')
      continue;
    std::istringstream ss{std::string(line)};
    std::vector<std::string> t;
    for (std::string w; ss >> w;)
      t.push_back(w);
    if (!counted) {
      int count = 0;
      if (t.size() != 1 || !to_int(t[0], count) || count < 0)
        throw molecule_file_error(where(name, n) + ": '" + std::string(line) +
                                  "' is not a count of points");
      counted = true;
      m.atoms.reserve(static_cast<std::size_t>(count));
      continue;
    }
    if (t.size() != 4)
      throw molecule_file_error(where(name, n) + ": " + std::to_string(t.size()) +
                                " fields, not four (x y z radius)");
    atom a;
    a.serial = static_cast<int>(m.atoms.size()) + 1;
    double *const v[4] = {&a.x, &a.y, &a.z, &a.radius};
    for (int i = 0; i < 4; ++i)
      if (!to_double(t[static_cast<std::size_t>(i)], *v[i]))
        throw molecule_file_error(where(name, n) + ": '" + t[static_cast<std::size_t>(i)] +
                                  "' is not a number");
    if (a.radius < 0)
      throw molecule_file_error(where(name, n) + ": the radius is negative");
    m.atoms.push_back(std::move(a));
  }
  if (in.bad())
    throw molecule_file_error(name + ": can't read it");
  return m;
}

molecule molecule_file_io::read(app &, const std::string &filename,
                                const molecule_read_params &) const {
  throw molecule_file_error("read_molecule: '" + filename + "': the " + id() +
                            " handler can't read");
}

file_handler_registry<molecule_file_io> &molecule_handlers(app &ctx) {
  return ctx.slot<molecule_io_slot>().handlers;
}

std::vector<std::string> molecule_read_extensions(app &ctx) {
  std::set<std::string> out;
  for (const auto &h : molecule_handlers(ctx).all())
    if (h->can_read())
      for (const std::string &e : h->extensions())
        out.insert(e);
  return std::vector<std::string>(out.begin(), out.end());
}

molecule read_molecule(app &ctx, const std::string &filename, const molecule_read_params &p) {
  check_params("read_molecule", p);
  const std::string ext = extension_of(filename);
  std::vector<std::string> errors;
  bool any = false;
  for (const auto &h : molecule_handlers(ctx).for_extension(ext)) {
    if (!h->can_read())
      continue;
    any = true;
    try {
      molecule m = h->read(ctx, filename, p);
      if (m.empty()) {
        std::string why = "read_molecule: '" + filename + "' has no atoms";
        if (p.model > 0 && m.models > 0)
          why += " in MODEL " + std::to_string(p.model) + " (it has " + std::to_string(m.models) +
                 " models)";
        else if (p.model > 1)
          why += " in MODEL " + std::to_string(p.model) + " (it has no MODEL records)";
        throw molecule_file_error(why);
      }
      return m;
    } catch (const std::exception &e) {
      errors.push_back(e.what());
    }
  }
  if (!any)
    throw molecule_file_error("read_molecule: '" + filename + "': no reader for '" + ext +
                              "' (this app reads " + join(molecule_read_extensions(ctx), ", ") +
                              ")");
  throw molecule_file_error(errors.size() == 1 ? errors.front() : join(errors, "; "));
}

void molecule_file_io::write(app &, const molecule &, const std::string &filename) const {
  throw molecule_file_error("write_molecule: '" + filename + "': the " + id() +
                            " handler can't write");
}

std::vector<std::string> molecule_write_extensions(app &ctx) {
  std::set<std::string> out;
  for (const auto &h : molecule_handlers(ctx).all())
    if (h->can_write())
      for (const std::string &e : h->extensions())
        out.insert(e);
  return std::vector<std::string>(out.begin(), out.end());
}

void write_molecule(app &ctx, const molecule &m, const std::string &filename) {
  const std::string ext = extension_of(filename);
  for (const auto &h : molecule_handlers(ctx).for_extension(ext))
    if (h->can_write()) {
      h->write(ctx, m, filename);
      return;
    }
  throw molecule_file_error("write_molecule: '" + filename + "': no writer for '" + ext +
                            "' (this app writes " + join(molecule_write_extensions(ctx), ", ") +
                            ")");
}

void write_pdb(std::ostream &out, const molecule &m) {
  char b[128];
  if (!m.id.empty()) {
    std::snprintf(b, sizeof b, "HEADER    %-40s%-9s   %-4.4s", "", "", m.id.c_str());
    out << b << "\n";
  }
  // TITLE: 70 characters on the first line, 68 on each continuation line (numbered from 2),
  // broken at a blank where there is one.
  for (std::size_t at = 0, line = 1; at < m.title.size(); ++line) {
    std::size_t take = std::min<std::size_t>(line == 1 ? 70 : 68, m.title.size() - at);
    if (at + take < m.title.size()) {
      const std::size_t sp = m.title.rfind(' ', at + take);
      if (sp != std::string::npos && sp > at)
        take = sp - at;
    }
    const std::string part = m.title.substr(at, take);
    if (line == 1)
      out << "TITLE     " << part << "\n";
    else
      out << "TITLE   " << (line < 10 ? " " : "") << line << " " << part << "\n";
    at += take;
    while (at < m.title.size() && m.title[at] == ' ')
      ++at;
  }
  int helices = 0, turns = 0;
  for (const secondary_structure &s : m.structures) {
    const std::string r0 = residue_name_at(m, s.chain, s.start_seq, s.start_insertion);
    const std::string r1 = residue_name_at(m, s.chain, s.end_seq, s.end_insertion);
    switch (s.type) {
    case secondary_structure::kind::helix:
      std::snprintf(b, sizeof b, "HELIX  %3d %3.3s %3.3s %c %4d%c %3.3s %c %4d%c%2d", ++helices,
                    s.id.c_str(), r0.c_str(), s.chain, s.start_seq, s.start_insertion, r1.c_str(),
                    s.chain, s.end_seq, s.end_insertion, s.helix_class);
      break;
    case secondary_structure::kind::sheet: {
      int strands = 0;
      for (const secondary_structure &t : m.structures)
        strands += t.type == secondary_structure::kind::sheet && t.id == s.id ? 1 : 0;
      std::snprintf(b, sizeof b, "SHEET  %3d %3.3s%2d %3.3s %c%4d%c %3.3s %c%4d%c%2d", s.strand,
                    s.id.c_str(), strands, r0.c_str(), s.chain, s.start_seq, s.start_insertion,
                    r1.c_str(), s.chain, s.end_seq, s.end_insertion, s.sense);
      break;
    }
    case secondary_structure::kind::turn:
      std::snprintf(b, sizeof b, "TURN   %3d %3.3s %3.3s %c%4d%c %3.3s %c%4d%c", ++turns,
                    s.id.c_str(), r0.c_str(), s.chain, s.start_seq, s.start_insertion, r1.c_str(),
                    s.chain, s.end_seq, s.end_insertion);
      break;
    }
    out << b << "\n";
  }
  each_model(m, [&](int model) {
    if (model > 0) {
      std::snprintf(b, sizeof b, "MODEL     %4d", model);
      out << b << "\n";
    }
    for (const atom &a : m.atoms) {
      if (a.model != model)
        continue;
      std::snprintf(b, sizeof b,
                    "%-6s%5d %4s%c%3.3s %c%4d%c   %8.3f%8.3f%8.3f%6.2f%6.2f          %2.2s",
                    a.hetero ? "HETATM" : "ATOM", a.serial, pdb_atom_name(a).c_str(), a.alt_loc,
                    a.residue.c_str(), a.chain, a.residue_seq, a.insertion, a.x, a.y, a.z,
                    a.occupancy, a.b_factor, a.element.c_str());
      out << b << "\n";
    }
    if (model > 0)
      out << "ENDMDL\n";
  });
  out << "END\n";
}

void write_pqr(std::ostream &out, const molecule &m) {
  char b[192];
  each_model(m, [&](int model) {
    if (model > 0) {
      std::snprintf(b, sizeof b, "MODEL     %4d", model);
      out << b << "\n";
    }
    for (const atom &a : m.atoms) {
      if (a.model != model)
        continue;
      const std::string name = a.name.empty() ? "X" : a.name;
      const std::string residue = a.residue.empty() ? "UNK" : a.residue;
      std::string seq = std::to_string(a.residue_seq);
      if (a.insertion != ' ')
        seq += a.insertion;
      const std::string chain = a.chain == ' ' ? std::string() : std::string(1, a.chain) + " ";
      std::snprintf(b, sizeof b, "%-6s%5d %-4s %3s %s%5s    %.3f %.3f %.3f %.4f %.4f",
                    a.hetero ? "HETATM" : "ATOM", a.serial, name.c_str(), residue.c_str(),
                    chain.c_str(), seq.c_str(), a.x, a.y, a.z, a.charge, a.radius);
      out << b << "\n";
    }
    if (model > 0)
      out << "ENDMDL\n";
  });
  out << "END\n";
}

void write_pts(std::ostream &out, const molecule &m) {
  out << m.atoms.size() << "\n";
  write_xyzr(out, m);
}

void write_xyzr(std::ostream &out, const molecule &m) {
  for (const atom &a : m.atoms)
    out << param_detail::format_double(a.x) << ' ' << param_detail::format_double(a.y) << ' '
        << param_detail::format_double(a.z) << ' ' << param_detail::format_double(a.radius) << "\n";
}

} // namespace mol
} // namespace cvc
