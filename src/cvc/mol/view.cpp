/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol views (cvc/mol/view.h): the spheres, bonds and cartoon a molecule is drawn with.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cvc/mol/view.h>
#include <exception>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc {
namespace mol {

namespace {

// -- the tables ----------------------------------------------------------------------------------

// An element's colour and covalent radius (Å, a single bond), and whether it is a metal drawn
// without bonds. The colours follow the usual convention (carbon grey, nitrogen blue, oxygen red,
// sulphur yellow, phosphorus orange) with values chosen for libcvc; the radii are the commonly
// tabulated single-bond covalent radii.
struct element_row {
  const char *symbol;
  std::uint8_t r, g, b;
  float covalent;
  bool metal;
};
constexpr element_row kElements[] = {
    {"H", 236, 236, 236, 0.31f, false}, {"D", 236, 236, 236, 0.31f, false},
    {"C", 150, 150, 150, 0.76f, false}, {"N", 52, 86, 236, 0.71f, false},
    {"O", 228, 30, 30, 0.66f, false},   {"S", 236, 196, 48, 1.05f, false},
    {"P", 248, 132, 16, 1.07f, false},  {"F", 136, 214, 92, 0.57f, false},
    {"CL", 44, 220, 44, 1.02f, false},  {"BR", 160, 48, 40, 1.20f, false},
    {"I", 140, 20, 150, 1.39f, false},  {"SE", 242, 156, 20, 1.20f, false},
    {"B", 244, 176, 176, 0.84f, false}, {"SI", 228, 196, 156, 1.11f, false},
    {"NA", 164, 96, 232, 1.66f, true},  {"K", 136, 70, 204, 2.03f, true},
    {"LI", 196, 132, 244, 1.28f, true}, {"MG", 128, 236, 20, 1.41f, true},
    {"CA", 72, 236, 20, 1.76f, true},   {"MN", 152, 120, 196, 1.39f, true},
    {"FE", 216, 104, 52, 1.32f, true},  {"CO", 232, 140, 156, 1.26f, true},
    {"NI", 84, 200, 84, 1.24f, true},   {"CU", 196, 128, 56, 1.32f, true},
    {"ZN", 124, 128, 172, 1.22f, true}, {"CD", 244, 212, 140, 1.44f, true},
    {"HG", 180, 180, 204, 1.32f, true}, {"PT", 204, 204, 220, 1.36f, true},
    {"AU", 244, 204, 40, 1.36f, true},  {"AG", 196, 196, 196, 1.45f, true},
    {"SR", 40, 244, 120, 1.95f, true},  {"BA", 32, 196, 56, 2.15f, true},
};
constexpr rgb8 kUnknownElement = {240, 120, 220};
constexpr float kUnknownCovalent = 0.77f;

// A chain's colour by the order it comes in.
constexpr rgb8 kChains[] = {
    {92, 160, 236}, {236, 128, 72},  {112, 196, 96},  {216, 88, 160},
    {236, 204, 76}, {124, 108, 220}, {72, 196, 196},  {196, 76, 76},
    {160, 196, 64}, {236, 156, 196}, {120, 140, 160}, {196, 148, 100},
};

// A residue's colour: amino acids by their side chain's kind (aliphatic greens, aromatic purples,
// polar oranges and teals, acidic reds, basic blues, glycine and proline apart), nucleotides by
// base.
struct residue_row {
  const char *name;
  rgb8 color;
};
constexpr residue_row kResidues[] = {
    {"ALA", {176, 208, 160}}, {"VAL", {96, 172, 80}},   {"LEU", {72, 148, 64}},
    {"ILE", {52, 128, 48}},   {"MET", {140, 188, 72}},  {"PHE", {120, 92, 196}},
    {"TYR", {148, 116, 212}}, {"TRP", {96, 64, 168}},   {"SER", {244, 172, 92}},
    {"THR", {228, 148, 64}},  {"ASN", {72, 196, 188}},  {"GLN", {48, 168, 168}},
    {"ASP", {220, 56, 48}},   {"GLU", {188, 36, 36}},   {"LYS", {64, 108, 228}},
    {"ARG", {40, 76, 196}},   {"HIS", {120, 148, 236}}, {"GLY", {236, 236, 236}},
    {"PRO", {212, 180, 148}}, {"CYS", {232, 220, 72}},  {"MSE", {140, 188, 72}},
    {"A", {228, 96, 96}},     {"DA", {228, 96, 96}},    {"G", {96, 196, 96}},
    {"DG", {96, 196, 96}},    {"C", {236, 212, 80}},    {"DC", {236, 212, 80}},
    {"U", {88, 172, 228}},    {"DT", {88, 172, 228}},   {"DU", {88, 172, 228}},
};
constexpr rgb8 kOtherResidue = {196, 168, 204};

constexpr rgb8 kHelix = {228, 72, 112}, kSheet = {244, 200, 44}, kTurn = {188, 212, 236},
               kCoil = {216, 216, 216};

std::string upper(std::string s) {
  for (char &c : s)
    if (c >= 'a' && c <= 'z')
      c = static_cast<char>(c - 'a' + 'A');
  return s;
}

const element_row *find_element(const std::string &element) {
  const std::string e = upper(element);
  for (const element_row &r : kElements)
    if (e == r.symbol)
      return &r;
  return nullptr;
}

bool is_water(const std::string &residue) {
  return residue == "HOH" || residue == "WAT" || residue == "DOD" || residue == "H2O";
}

bool is_hydrogen(const std::string &element) {
  return element == "H" || element == "D" || element == "h" || element == "d";
}

// The colour of each atom under a scheme, with what the scheme needs worked out once.
struct colorer {
  color_scheme scheme = color_scheme::element;
  const molecule *m = nullptr;
  std::string chains; // chain_ids
  double b_lo = 0.0, b_hi = 1.0;

  rgb8 operator()(std::size_t i) const {
    const atom &a = m->atoms[i];
    switch (scheme) {
    case color_scheme::element:
      return element_color(a.element);
    case color_scheme::chain: {
      const std::size_t at = chains.find(a.chain);
      return chain_color(at == std::string::npos ? 0 : at);
    }
    case color_scheme::residue:
      return residue_color(a.residue);
    case color_scheme::b_factor:
      return b_factor_color(b_hi > b_lo ? (a.b_factor - b_lo) / (b_hi - b_lo) : 0.5);
    case color_scheme::structure: {
      const int s = m->structure_of(a);
      return structure_color(s < 0 ? -1 : static_cast<int>(m->structures[s].type));
    }
    }
    return kUnknownElement;
  }
};

// -- small vector arithmetic
// -----------------------------------------------------------------------

struct vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};
vec3 operator+(vec3 a, vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
vec3 operator-(vec3 a, vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
vec3 operator*(double s, vec3 a) { return {s * a.x, s * a.y, s * a.z}; }
double dot(vec3 a, vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
vec3 cross(vec3 a, vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double length(vec3 a) { return std::sqrt(dot(a, a)); }
vec3 normalized(vec3 a, vec3 fallback = {1.0, 0.0, 0.0}) {
  const double l = length(a);
  return l > 1e-12 ? (1.0 / l) * a : fallback;
}
// Any unit vector perpendicular to the unit vector t.
vec3 perpendicular(vec3 t) {
  const vec3 axis = std::fabs(t.x) < 0.9 ? vec3{1.0, 0.0, 0.0} : vec3{0.0, 1.0, 0.0};
  return normalized(cross(t, axis));
}
vec3 position(const atom &a) { return {a.x, a.y, a.z}; }

// A unit sphere tessellated: an icosahedron subdivided `detail` times.
struct unit_sphere {
  std::vector<vec3> verts;
  std::vector<std::array<int, 3>> tris;

  explicit unit_sphere(int detail) {
    const double t = (1.0 + std::sqrt(5.0)) / 2.0;
    const double v[12][3] = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0},
                             {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t},
                             {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    for (const auto &p : v)
      verts.push_back(normalized({p[0], p[1], p[2]}));
    tris = {{0, 11, 5},  {0, 5, 1},  {0, 1, 7},  {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
            {11, 10, 2}, {10, 7, 6}, {7, 1, 8},  {3, 9, 4},  {3, 4, 2},   {3, 2, 6}, {3, 6, 8},
            {3, 8, 9},   {4, 9, 5},  {2, 4, 11}, {6, 2, 10}, {8, 6, 7},   {9, 8, 1}};
    for (int d = 0; d < detail; ++d) {
      std::map<std::pair<int, int>, int> mid;
      const auto middle = [&](int a, int b) {
        const std::pair<int, int> key = std::minmax(a, b);
        const auto it = mid.find(key);
        if (it != mid.end())
          return it->second;
        verts.push_back(normalized(verts[a] + verts[b]));
        const int i = static_cast<int>(verts.size()) - 1;
        mid.emplace(key, i);
        return i;
      };
      std::vector<std::array<int, 3>> next;
      next.reserve(tris.size() * 4);
      for (const auto &f : tris) {
        const int a = middle(f[0], f[1]), b = middle(f[1], f[2]), c = middle(f[2], f[0]);
        next.push_back({f[0], a, c});
        next.push_back({f[1], b, a});
        next.push_back({f[2], c, b});
        next.push_back({a, b, c});
      }
      tris.swap(next);
    }
  }
};

// The trace, built an atom at a time (chain_trace runs it to the end; the stepper slices it).
struct trace_builder {
  const molecule &m;
  std::vector<trace_residue> out;
  std::size_t next = 0; // the next atom to look at
  // The residue being read: its first atom, and its trace and guide atoms so far.
  std::size_t res_first = 0;
  int res_trace = -1, res_guide = -1;
  bool res_nucleotide = false;

  explicit trace_builder(const molecule &mol) : m(mol) {}

  static bool same_residue(const atom &a, const atom &b) {
    return a.model == b.model && a.chain == b.chain && a.residue_seq == b.residue_seq &&
           a.insertion == b.insertion && a.residue == b.residue;
  }

  void close_residue() {
    if (res_trace < 0)
      return;
    trace_residue r;
    r.atom = res_trace;
    r.guide = res_nucleotide ? -1 : res_guide;
    r.structure = m.structure_of(m.atoms[res_trace]);
    if (out.empty()) {
      r.start = true;
    } else {
      const atom &prev = m.atoms[out.back().atom];
      const atom &cur = m.atoms[res_trace];
      const bool prev_nucleotide = prev.name == "P";
      const double gap = res_nucleotide ? 8.0 : 4.2;
      r.start = prev.model != cur.model || prev.chain != cur.chain ||
                prev_nucleotide != res_nucleotide || length(position(cur) - position(prev)) > gap;
    }
    out.push_back(r);
  }

  // Look at up to `budget` atoms; true once every atom is seen.
  bool step(std::size_t &budget) {
    const std::size_t n = m.atoms.size();
    while (next < n && budget > 0) {
      const atom &a = m.atoms[next];
      if (next == 0 || !same_residue(a, m.atoms[res_first])) {
        close_residue();
        res_first = next;
        res_trace = res_guide = -1;
        res_nucleotide = false;
      }
      if (res_trace < 0 && a.name == "CA" && upper(a.element) == "C") {
        res_trace = static_cast<int>(next);
      } else if (res_trace < 0 && a.name == "P") {
        res_trace = static_cast<int>(next);
        res_nucleotide = true;
      } else if (res_guide < 0 && a.name == "O") {
        res_guide = static_cast<int>(next);
      }
      ++next;
      --budget;
    }
    if (next < n)
      return false;
    if (n > 0 && res_first < n) {
      close_residue();
      res_first = n; // closed once
    }
    return true;
  }
};

// The bonds among a set of atoms, a cell grid's worth at a time.
struct bond_finder {
  const molecule &m;
  double tolerance;
  std::vector<int> candidates; // the atoms that may bond, ascending
  double cell = 1.0;
  std::unordered_map<std::uint64_t, std::vector<int>> grid;
  std::size_t inserted = 0, searched = 0;
  std::vector<bond> out;
  std::vector<int> scratch;

  bond_finder(const molecule &mol, double tol) : m(mol), tolerance(tol) {}

  // Set the atoms (ascending) and the cell size before stepping.
  void start(std::vector<int> atoms) {
    candidates = std::move(atoms);
    double largest = 0.0;
    for (int i : candidates)
      largest = std::max(largest, covalent_radius(m.atoms[i].element));
    cell = std::max(0.5, 2.0 * largest + tolerance);
  }

  std::int64_t cell_of(double v) const { return static_cast<std::int64_t>(std::floor(v / cell)); }
  static std::uint64_t key(std::int64_t i, std::int64_t j, std::int64_t k) {
    const std::uint64_t mask = (1u << 21) - 1;
    return ((static_cast<std::uint64_t>(i) & mask) << 42) |
           ((static_cast<std::uint64_t>(j) & mask) << 21) | (static_cast<std::uint64_t>(k) & mask);
  }

  bool bonded(int i, int j) const {
    const atom &a = m.atoms[i], &b = m.atoms[j];
    if (a.model != b.model)
      return false;
    if (a.alt_loc != ' ' && b.alt_loc != ' ' && a.alt_loc != b.alt_loc)
      return false;
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    const double d2 = dx * dx + dy * dy + dz * dz;
    const double reach = covalent_radius(a.element) + covalent_radius(b.element) + tolerance;
    return d2 > 0.16 && d2 < reach * reach;
  }

  // Up to `budget` units (an atom inserted, then an atom's neighbours searched); true once done.
  bool step(std::size_t &budget) {
    while (inserted < candidates.size() && budget > 0) {
      const atom &a = m.atoms[candidates[inserted]];
      grid[key(cell_of(a.x), cell_of(a.y), cell_of(a.z))].push_back(candidates[inserted]);
      ++inserted;
      --budget;
    }
    while (inserted == candidates.size() && searched < candidates.size() && budget > 0) {
      const int i = candidates[searched];
      const atom &a = m.atoms[i];
      const std::int64_t ci = cell_of(a.x), cj = cell_of(a.y), ck = cell_of(a.z);
      scratch.clear();
      for (std::int64_t di = -1; di <= 1; ++di)
        for (std::int64_t dj = -1; dj <= 1; ++dj)
          for (std::int64_t dk = -1; dk <= 1; ++dk) {
            const auto it = grid.find(key(ci + di, cj + dj, ck + dk));
            if (it == grid.end())
              continue;
            for (int j : it->second)
              if (j > i && bonded(i, j))
                scratch.push_back(j);
          }
      std::sort(scratch.begin(), scratch.end());
      for (int j : scratch)
        out.push_back({i, j});
      ++searched;
      --budget;
    }
    return inserted == candidates.size() && searched == candidates.size();
  }
};

// Mesh output with its bounds kept as it grows.
struct mesh_out {
  geometry &g;
  std::array<double, 6> &bounds;
  bool &any;

  int add(vec3 p, vec3 n, rgb8 c) {
    g.points().push_back({p.x, p.y, p.z});
    g.normals().push_back({n.x, n.y, n.z});
    g.colors().push_back({c[0] / 255.0, c[1] / 255.0, c[2] / 255.0});
    grow(p, 0.0);
    return static_cast<int>(g.points().size()) - 1;
  }
  void tri(int a, int b, int c) {
    g.tris().push_back({static_cast<geometry::index_t>(a), static_cast<geometry::index_t>(b),
                        static_cast<geometry::index_t>(c)});
  }
  void grow(vec3 p, double r) {
    if (!any) {
      bounds = {p.x - r, p.y - r, p.z - r, p.x + r, p.y + r, p.z + r};
      any = true;
      return;
    }
    bounds[0] = std::min(bounds[0], p.x - r);
    bounds[1] = std::min(bounds[1], p.y - r);
    bounds[2] = std::min(bounds[2], p.z - r);
    bounds[3] = std::max(bounds[3], p.x + r);
    bounds[4] = std::max(bounds[4], p.y + r);
    bounds[5] = std::max(bounds[5], p.z + r);
  }
};

} // namespace

// -- the palettes ---------------------------------------------------------------------------------

rgb8 element_color(const std::string &element) {
  const element_row *r = find_element(element);
  return r ? rgb8{r->r, r->g, r->b} : kUnknownElement;
}

rgb8 chain_color(std::size_t chain_order) {
  return kChains[chain_order % (sizeof(kChains) / sizeof(kChains[0]))];
}

rgb8 residue_color(const std::string &residue) {
  const std::string r = upper(residue);
  for (const residue_row &row : kResidues)
    if (r == row.name)
      return row.color;
  return kOtherResidue;
}

rgb8 structure_color(int kind) {
  switch (kind) {
  case static_cast<int>(secondary_structure::kind::helix):
    return kHelix;
  case static_cast<int>(secondary_structure::kind::sheet):
    return kSheet;
  case static_cast<int>(secondary_structure::kind::turn):
    return kTurn;
  default:
    return kCoil;
  }
}

rgb8 b_factor_color(double t) {
  if (!(t > 0.0))
    t = 0.0;
  if (t > 1.0)
    t = 1.0;
  const rgb8 lo = {40, 72, 228}, mid = {244, 244, 244}, hi = {228, 40, 40};
  const rgb8 &a = t < 0.5 ? lo : mid;
  const rgb8 &b = t < 0.5 ? mid : hi;
  const double s = t < 0.5 ? t * 2.0 : (t - 0.5) * 2.0;
  rgb8 out;
  for (int c = 0; c < 3; ++c)
    out[c] = static_cast<std::uint8_t>(std::lround(a[c] + s * (b[c] - a[c])));
  return out;
}

double covalent_radius(const std::string &element) {
  const element_row *r = find_element(element);
  return r ? r->covalent : kUnknownCovalent;
}

bool is_metal(const std::string &element) {
  const element_row *r = find_element(element);
  return r && r->metal;
}

std::string chain_ids(const molecule &m) {
  std::string out;
  for (const atom &a : m.atoms)
    if (out.find(a.chain) == std::string::npos)
      out.push_back(a.chain);
  return out;
}

std::vector<rgb8> atom_colors(const molecule &m, color_scheme scheme,
                              const std::vector<double> &b_range) {
  colorer c;
  c.scheme = scheme;
  c.m = &m;
  c.chains = chain_ids(m);
  if (b_range.size() == 2) {
    c.b_lo = b_range[0];
    c.b_hi = b_range[1];
  } else if (!m.atoms.empty()) {
    c.b_lo = c.b_hi = m.atoms[0].b_factor;
    for (const atom &a : m.atoms) {
      c.b_lo = std::min(c.b_lo, a.b_factor);
      c.b_hi = std::max(c.b_hi, a.b_factor);
    }
  }
  std::vector<rgb8> out(m.atoms.size());
  for (std::size_t i = 0; i < out.size(); ++i)
    out[i] = c(i);
  return out;
}

std::vector<bond> find_bonds(const molecule &m, double tolerance) {
  bond_finder f(m, tolerance);
  std::vector<int> atoms;
  for (std::size_t i = 0; i < m.atoms.size(); ++i)
    if (!is_metal(m.atoms[i].element))
      atoms.push_back(static_cast<int>(i));
  f.start(std::move(atoms));
  std::size_t budget = std::numeric_limits<std::size_t>::max();
  f.step(budget);
  return std::move(f.out);
}

std::vector<trace_residue> chain_trace(const molecule &m) {
  trace_builder t(m);
  std::size_t budget = std::numeric_limits<std::size_t>::max();
  t.step(budget);
  return std::move(t.out);
}

// -- the stepper ----------------------------------------------------------------------------------

struct build_view_stepper::impl {
  enum class stage {
    selecting,
    coloring,
    bonding,
    spheres,
    sticks,
    tracing,
    cartoon,
    done,
    failed,
    cancelled
  };

  app &ctx;
  std::shared_ptr<const molecule> mol;
  view_params p;
  stage st = stage::selecting;
  std::exception_ptr error;
  std::atomic<bool> cancel_requested{false};
  bool checked = false;

  molecule_view out;
  bool any_bounds = false;

  // selecting / coloring
  std::size_t cursor = 0;
  std::vector<char> drawn;
  std::vector<rgb8> color;
  colorer paint;
  bool b_seen = false;
  double b_lo = 0.0, b_hi = 0.0;
  const atom *last_residue = nullptr;

  // bonding, spheres, sticks
  std::unique_ptr<bond_finder> bonds;
  std::unique_ptr<unit_sphere> sphere;

  // tracing, cartoon
  std::unique_ptr<trace_builder> trace;
  std::vector<trace_residue> residues; // the drawn ones
  std::size_t piece = 0;               // the residue the next piece starts at

  // Units for progress: done, and the whole as estimated at the start.
  double units_done = 0.0, units_total = 1.0;
  double last_progress = 0.0;

  impl(app &c, std::shared_ptr<const molecule> m, view_params params)
      : ctx(c), mol(std::move(m)), p(std::move(params)), out() {
    out.spheres = geometry(ctx);
    out.sticks = geometry(ctx);
    out.cartoon = geometry(ctx);
  }

  bool wants_spheres() const { return p.style != view_style::cartoon; }
  bool wants_bonds() const {
    return p.style == view_style::ball_and_stick || p.style == view_style::sticks;
  }

  void check() {
    if (!mol)
      throw std::invalid_argument("build_view: no molecule");
    std::vector<std::string> problems = params_validate(p);
    if (!problems.empty()) {
      std::string msg = "build_view: ";
      for (std::size_t i = 0; i < problems.size(); ++i)
        msg += (i ? "; " : "") + problems[i];
      throw std::invalid_argument(msg);
    }
    const double n = static_cast<double>(mol->atoms.size());
    units_total = 2.0 * n;
    if (wants_bonds())
      units_total += 2.0 * n + 1.1 * n; // the grid and its search, then about a bond an atom
    if (wants_spheres())
      units_total += n;
    if (p.style == view_style::cartoon)
      units_total += n + n / 8.0; // the trace, then about a residue in eight atoms
    drawn.assign(mol->atoms.size(), 0);
    color.assign(mol->atoms.size(), rgb8{0, 0, 0});
    out.chains = chain_ids(*mol);
    paint.scheme = p.color_by;
    paint.m = mol.get();
    paint.chains = out.chains;
    checked = true;
  }

  bool selected(const atom &a) const {
    if (p.atoms == atom_set::polymer && a.hetero)
      return false;
    if (p.atoms == atom_set::hetero && !a.hetero)
      return false;
    if (!p.hydrogens && is_hydrogen(a.element))
      return false;
    if (!p.water && is_water(a.residue))
      return false;
    return p.hidden_chains.find(a.chain) == std::string::npos;
  }

  void select_step(std::size_t &left) {
    const std::vector<atom> &atoms = mol->atoms;
    while (cursor < atoms.size() && left > 0) {
      const atom &a = atoms[cursor];
      if (selected(a)) {
        drawn[cursor] = 1;
        ++out.atoms_drawn;
        if (!b_seen) {
          b_lo = b_hi = a.b_factor;
          b_seen = true;
        } else {
          b_lo = std::min(b_lo, a.b_factor);
          b_hi = std::max(b_hi, a.b_factor);
        }
        if (!last_residue || !trace_builder::same_residue(*last_residue, a))
          ++out.residues;
        last_residue = &a;
      }
      ++cursor;
      --left;
      units_done += 1.0;
    }
    if (cursor == atoms.size()) {
      if (p.b_range.size() == 2) {
        paint.b_lo = p.b_range[0];
        paint.b_hi = p.b_range[1];
      } else {
        paint.b_lo = b_lo;
        paint.b_hi = b_hi;
      }
      cursor = 0;
      st = stage::coloring;
    }
  }

  void color_step(std::size_t &left) {
    const std::size_t n = mol->atoms.size();
    while (cursor < n && left > 0) {
      if (drawn[cursor])
        color[cursor] = paint(cursor);
      ++cursor;
      --left;
      units_done += 1.0;
    }
    if (cursor < n)
      return;
    cursor = 0;
    if (wants_bonds()) {
      bonds = std::make_unique<bond_finder>(*mol, p.bond_tolerance);
      std::vector<int> atoms;
      for (std::size_t i = 0; i < n; ++i)
        if (drawn[i] && !is_metal(mol->atoms[i].element))
          atoms.push_back(static_cast<int>(i));
      bonds->start(std::move(atoms));
      st = stage::bonding;
    } else {
      next_after_bonds();
    }
  }

  void next_after_bonds() {
    cursor = 0;
    if (wants_spheres()) {
      if (p.sphere_mesh)
        sphere = std::make_unique<unit_sphere>(p.sphere_detail);
      st = stage::spheres;
    } else {
      trace = std::make_unique<trace_builder>(*mol);
      st = stage::tracing;
    }
  }

  void bond_step(std::size_t &left) {
    const std::size_t before = left;
    const bool finished = bonds->step(left);
    units_done += static_cast<double>(before - left);
    if (!finished)
      return;
    out.bonds = std::move(bonds->out);
    bonds.reset();
    next_after_bonds();
  }

  double sphere_radius(const atom &a) const {
    switch (p.style) {
    case view_style::ball_and_stick:
      return p.atom_scale * a.radius;
    case view_style::spacefill:
      return a.radius;
    default:
      return p.bond_radius;
    }
  }

  void sphere_step(std::size_t &left) {
    const std::size_t n = mol->atoms.size();
    mesh_out mesh{out.spheres, out.bounds, any_bounds};
    while (cursor < n && left > 0) {
      if (drawn[cursor]) {
        const atom &a = mol->atoms[cursor];
        const double r = sphere_radius(a);
        out.sphere_atoms.push_back(static_cast<int>(cursor));
        out.centers.push_back(static_cast<float>(a.x));
        out.centers.push_back(static_cast<float>(a.y));
        out.centers.push_back(static_cast<float>(a.z));
        out.radii.push_back(static_cast<float>(r));
        out.colors.push_back(color[cursor]);
        mesh.grow(position(a), r);
        if (sphere) {
          const int base = static_cast<int>(out.spheres.points().size());
          for (const vec3 &v : sphere->verts) {
            out.spheres.points().push_back({a.x + r * v.x, a.y + r * v.y, a.z + r * v.z});
            out.spheres.normals().push_back({v.x, v.y, v.z});
            out.spheres.colors().push_back(
                {color[cursor][0] / 255.0, color[cursor][1] / 255.0, color[cursor][2] / 255.0});
          }
          for (const auto &f : sphere->tris)
            mesh.tri(base + f[0], base + f[1], base + f[2]);
        }
      }
      ++cursor;
      --left;
      units_done += 1.0;
    }
    if (cursor < n)
      return;
    cursor = 0;
    sphere.reset();
    if (wants_bonds())
      st = stage::sticks;
    else
      st = stage::done;
  }

  // One bond: two cylinders, from each atom to the midpoint, in that atom's colour.
  void stick(const bond &b, int sides) {
    mesh_out mesh{out.sticks, out.bounds, any_bounds};
    const atom &a0 = mol->atoms[b.a], &a1 = mol->atoms[b.b];
    const vec3 p0 = position(a0), p1 = position(a1), mid = 0.5 * (p0 + p1);
    const vec3 t = normalized(p1 - p0);
    const vec3 u = perpendicular(t), w = cross(t, u);
    const double r = p.bond_radius;
    const vec3 ends[2][2] = {{p0, mid}, {mid, p1}};
    const rgb8 cols[2] = {color[b.a], color[b.b]};
    for (int half = 0; half < 2; ++half) {
      const int base = static_cast<int>(out.sticks.points().size());
      for (int e = 0; e < 2; ++e)
        for (int j = 0; j < sides; ++j) {
          const double th = 2.0 * 3.14159265358979323846 * j / sides;
          const vec3 n = std::cos(th) * u + std::sin(th) * w;
          mesh.add(ends[half][e] + r * n, n, cols[half]);
        }
      for (int j = 0; j < sides; ++j) {
        const int j1 = (j + 1) % sides;
        mesh.tri(base + j, base + j1, base + sides + j);
        mesh.tri(base + j1, base + sides + j1, base + sides + j);
      }
    }
  }

  void stick_step(std::size_t &left) {
    const int sides = std::clamp(p.sides, 4, 12);
    while (cursor < out.bonds.size() && left > 0) {
      stick(out.bonds[cursor], sides);
      ++cursor;
      --left;
      units_done += 1.0;
    }
    if (cursor == out.bonds.size())
      st = stage::done;
  }

  void trace_step(std::size_t &left) {
    const std::size_t before = left;
    const bool finished = trace->step(left);
    units_done += static_cast<double>(before - left);
    if (!finished)
      return;
    // The drawn residues, a piece starting wherever one was left out.
    bool broke = true;
    for (const trace_residue &r : trace->out) {
      if (!drawn[r.atom]) {
        broke = true;
        continue;
      }
      residues.push_back(r);
      if (broke)
        residues.back().start = true;
      broke = false;
    }
    trace.reset();
    out.residues = residues.size();
    piece = 0;
    st = stage::cartoon;
  }

  // The cross-section's half axes for a residue: across (along the guide) and through.
  void profile(const trace_residue &r, double &across, double &through) const {
    const int kind = r.structure < 0 ? -1 : static_cast<int>(mol->structures[r.structure].type);
    if (kind == static_cast<int>(secondary_structure::kind::helix)) {
      across = p.helix_width / 2.0;
      through = p.ribbon_thickness / 2.0;
    } else if (kind == static_cast<int>(secondary_structure::kind::sheet)) {
      across = p.sheet_width / 2.0;
      through = p.ribbon_thickness / 2.0;
    } else {
      across = through = p.coil_radius;
    }
  }

  bool is_sheet(const trace_residue &r) const {
    return r.structure >= 0 &&
           mol->structures[r.structure].type == secondary_structure::kind::sheet;
  }

  // -- the cartoon: each piece of trace a tube along a Catmull-Rom spline through its trace atoms,
  // drawn a residue at a time.

  // The piece being drawn: residues [pb, pe) of `residues`, the next one to draw (pk), the guides
  // worked out so far, and its rings so far.
  bool in_piece = false;
  std::size_t pb = 0, pe = 0, pk = 0;
  std::vector<vec3> guides;
  std::vector<double> cs, sn; // the ring's cosines and sines
  int first = -1, prev = -1;
  vec3 first_c, first_T, last_c, last_T;
  rgb8 first_col{}, last_col{};
  int first_atom = -1, last_atom = -1;

  std::size_t piece_size() const { return pe - pb; }
  vec3 point(std::ptrdiff_t k) const {
    const std::ptrdiff_t last = static_cast<std::ptrdiff_t>(piece_size()) - 1;
    return position(
        mol->atoms[residues[pb + static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(k, 0, last))]
                       .atom]);
  }

  // The guide of residue k (and of every one before it): its carbonyl (C-alpha to O), else towards
  // its neighbours' middle, flipped to keep the ribbon from twisting where a strand's carbonyls
  // alternate.
  const vec3 &guide(std::size_t k) {
    while (guides.size() <= k) {
      const std::size_t i = guides.size();
      const trace_residue &r = residues[pb + i];
      const std::ptrdiff_t is = static_cast<std::ptrdiff_t>(i);
      const vec3 here = point(is);
      vec3 g;
      if (r.guide >= 0)
        g = position(mol->atoms[r.guide]) - here;
      else if (i > 0 && i + 1 < piece_size())
        g = 0.5 * (point(is - 1) + point(is + 1)) - here;
      if (length(g) < 1e-9)
        g = i > 0 ? guides[i - 1] : perpendicular(normalized(point(1) - point(0)));
      g = normalized(g);
      if (i > 0 && dot(g, guides[i - 1]) < 0.0)
        g = -1.0 * g;
      guides.push_back(g);
    }
    return guides[k];
  }

  // A ring at a sample: the index of its first vertex. `flat` gives every vertex the normal flat_n.
  int ring(vec3 c, vec3 T, vec3 G, double across, double through, rgb8 col, int atom, bool flat,
           vec3 flat_n) {
    mesh_out mesh{out.cartoon, out.bounds, any_bounds};
    const vec3 N = normalized(G - dot(G, T) * T, perpendicular(T));
    const vec3 B = cross(T, N);
    const int base = static_cast<int>(out.cartoon.points().size());
    for (std::size_t j = 0; j < cs.size(); ++j) {
      const vec3 off = (across * cs[j]) * N + (through * sn[j]) * B;
      const vec3 nrm = flat ? flat_n : normalized((cs[j] / across) * N + (sn[j] / through) * B);
      mesh.add(c + off, nrm, col);
      out.cartoon_atoms.push_back(atom);
    }
    return base;
  }

  void connect(int r0, int r1) {
    mesh_out mesh{out.cartoon, out.bounds, any_bounds};
    const int K = static_cast<int>(cs.size());
    for (int j = 0; j < K; ++j) {
      const int j1 = (j + 1) % K;
      mesh.tri(r0 + j, r0 + j1, r1 + j);
      mesh.tri(r0 + j1, r1 + j1, r1 + j);
    }
  }

  // Close the tube at ring r: a fan facing nrm, on its own copy of the rim so it shades flat.
  void cap(int r, vec3 c, vec3 nrm, rgb8 col, int atom, bool start) {
    mesh_out mesh{out.cartoon, out.bounds, any_bounds};
    const int K = static_cast<int>(cs.size());
    const int centre = mesh.add(c, nrm, col);
    out.cartoon_atoms.push_back(atom);
    const int rim = static_cast<int>(out.cartoon.points().size());
    for (int j = 0; j < K; ++j) {
      const geometry::point_t q = out.cartoon.points()[r + j];
      mesh.add({q[0], q[1], q[2]}, nrm, col);
      out.cartoon_atoms.push_back(atom);
    }
    for (int j = 0; j < K; ++j) {
      const int j1 = (j + 1) % K;
      if (start)
        mesh.tri(centre, rim + j1, rim + j);
      else
        mesh.tri(centre, rim + j, rim + j1);
    }
  }

  void begin_piece(std::size_t b, std::size_t e) {
    pb = b;
    pe = e;
    pk = 0;
    guides.clear();
    first = prev = -1;
    first_atom = last_atom = -1;
    in_piece = true;
  }

  // Residue k's rings: the samples from it towards the next residue (the last residue: one).
  void piece_residue(std::size_t k) {
    const std::size_t n = piece_size();
    const trace_residue &r = residues[pb + k];
    const bool last = k + 1 == n;
    const trace_residue &rn = last ? r : residues[pb + k + 1];
    double a0, t0, a1, t1;
    profile(r, a0, t0);
    profile(rn, a1, t1);
    // A strand's last residue: an arrow from it to the next residue.
    const bool arrow = !last && is_sheet(r) && rn.structure != r.structure;
    const std::ptrdiff_t ks = static_cast<std::ptrdiff_t>(k);
    const vec3 P0 = point(ks - 1), P1 = point(ks), P2 = point(ks + 1), P3 = point(ks + 2);
    const vec3 g0 = guide(k), g1 = guide(last ? k : k + 1);
    const int S = p.subdivisions;
    const int steps = last ? 1 : S;
    for (int s = 0; s < steps; ++s) {
      const double t = static_cast<double>(s) / S;
      // Catmull-Rom (uniform), and its derivative; the last residue's tangent comes from behind.
      const double t2 = t * t, t3 = t2 * t;
      const vec3 c =
          0.5 * ((2.0 * P1) + t * (P2 - P0) + t2 * (2.0 * P0 - 5.0 * P1 + 4.0 * P2 - P3) +
                 t3 * (-1.0 * P0 + 3.0 * P1 - 3.0 * P2 + P3));
      const vec3 d = last ? P1 - P0
                          : 0.5 * ((P2 - P0) + (2.0 * t) * (2.0 * P0 - 5.0 * P1 + 4.0 * P2 - P3) +
                                   (3.0 * t2) * (-1.0 * P0 + 3.0 * P1 - 3.0 * P2 + P3));
      const vec3 T = normalized(d, normalized(P2 - P1, vec3{0.0, 0.0, 1.0}));
      const vec3 G = normalized((1.0 - t) * g0 + t * g1, g0);
      const double sm = t * t * (3.0 - 2.0 * t);
      double across = a0 + sm * (a1 - a0), through = t0 + sm * (t1 - t0);
      if (arrow) {
        across = std::max(1.6 * a0 * (1.0 - t), 1e-3);
        through = t0;
      }
      const bool first_half = t < 0.5 || last;
      const int atom = first_half ? r.atom : rn.atom;
      const rgb8 col = color[atom];
      if (arrow && s == 0) {
        // The strand runs to the arrow's base; the arrow's back reaches from the strand's width
        // out to the arrow's, facing back.
        const int base = ring(c, T, G, a0, t0, col, atom, false, T);
        if (prev >= 0)
          connect(prev, base);
        else
          first = base;
        const vec3 back = -1.0 * T;
        const int inner = ring(c, T, G, a0, t0, col, atom, true, back);
        const int outer = ring(c, T, G, across, through, col, atom, true, back);
        connect(inner, outer);
        prev = ring(c, T, G, across, through, col, atom, false, T);
      } else {
        const int cur = ring(c, T, G, across, through, col, atom, false, T);
        if (prev >= 0)
          connect(prev, cur);
        else
          first = cur;
        prev = cur;
      }
      if (first_atom < 0) {
        first_c = c;
        first_T = T;
        first_col = col;
        first_atom = atom;
      }
      last_c = c;
      last_T = T;
      last_col = col;
      last_atom = atom;
    }
  }

  void end_piece() {
    cap(first, first_c, -1.0 * first_T, first_col, first_atom, true);
    cap(prev, last_c, last_T, last_col, last_atom, false);
    in_piece = false;
  }

  // A unit per residue drawn (a lone residue, too short for a tube, is skipped as one unit).
  void cartoon_step(std::size_t &left) {
    if (cs.empty()) {
      const int K = p.sides;
      const double two_pi = 2.0 * 3.14159265358979323846;
      for (int j = 0; j < K; ++j) {
        cs.push_back(std::cos(two_pi * j / K));
        sn.push_back(std::sin(two_pi * j / K));
      }
    }
    while (left > 0) {
      if (!in_piece) {
        if (piece >= residues.size())
          break;
        std::size_t e = piece + 1;
        while (e < residues.size() && !residues[e].start)
          ++e;
        if (e - piece < 2) {
          piece = e;
          units_done += 1.0;
          --left;
          continue;
        }
        begin_piece(piece, e);
      }
      piece_residue(pk);
      ++pk;
      --left;
      units_done += 1.0;
      if (pk == piece_size()) {
        end_piece();
        piece = pe;
      }
    }
    if (!in_piece && piece >= residues.size())
      st = stage::done;
  }
};

build_view_stepper::build_view_stepper(app &ctx, std::shared_ptr<const molecule> m,
                                       view_params params)
    : _impl(std::make_unique<impl>(ctx, std::move(m), std::move(params))) {}
build_view_stepper::~build_view_stepper() = default;
build_view_stepper::build_view_stepper(build_view_stepper &&) noexcept = default;
build_view_stepper &build_view_stepper::operator=(build_view_stepper &&) noexcept = default;

bool build_view_stepper::step(std::size_t budget) {
  impl &d = *_impl;
  if (d.st == impl::stage::failed)
    std::rethrow_exception(d.error);
  if (d.st == impl::stage::done || d.st == impl::stage::cancelled)
    return true;
  if (d.cancel_requested.load(std::memory_order_acquire)) {
    d.st = impl::stage::cancelled;
    d.out = molecule_view();
    return true;
  }
  std::size_t left = budget == 0 ? std::numeric_limits<std::size_t>::max() : budget;
  try {
    if (!d.checked)
      d.check();
    while (left > 0 && d.st != impl::stage::done) {
      if (d.cancel_requested.load(std::memory_order_acquire))
        break;
      switch (d.st) {
      case impl::stage::selecting:
        d.select_step(left);
        break;
      case impl::stage::coloring:
        d.color_step(left);
        break;
      case impl::stage::bonding:
        d.bond_step(left);
        break;
      case impl::stage::spheres:
        d.sphere_step(left);
        break;
      case impl::stage::sticks:
        d.stick_step(left);
        break;
      case impl::stage::tracing:
        d.trace_step(left);
        break;
      case impl::stage::cartoon:
        d.cartoon_step(left);
        break;
      default:
        left = 0;
        break;
      }
    }
  } catch (...) {
    d.error = std::current_exception();
    d.st = impl::stage::failed;
    d.out = molecule_view();
    throw;
  }
  return d.st == impl::stage::done;
}

bool build_view_stepper::done() const { return _impl->st == impl::stage::done; }

const char *build_view_stepper::phase() const {
  switch (_impl->st) {
  case impl::stage::selecting:
  case impl::stage::coloring:
    return "selecting";
  case impl::stage::bonding:
    return "bonding";
  case impl::stage::spheres:
    return "spheres";
  case impl::stage::sticks:
    return "sticks";
  case impl::stage::tracing:
  case impl::stage::cartoon:
    return "cartoon";
  case impl::stage::done:
    return "done";
  case impl::stage::failed:
    return "failed";
  case impl::stage::cancelled:
    return "cancelled";
  }
  return "failed";
}

double build_view_stepper::progress() const {
  impl &d = *_impl;
  if (d.st == impl::stage::done)
    return 1.0;
  const double p = std::min(0.99, d.units_done / std::max(1.0, d.units_total));
  d.last_progress = std::max(d.last_progress, p);
  return d.last_progress;
}

std::string build_view_stepper::status() const {
  const impl &d = *_impl;
  if (d.st == impl::stage::done) {
    std::string s = std::to_string(d.out.atoms_drawn) + " atoms";
    if (!d.out.bonds.empty())
      s += ", " + std::to_string(d.out.bonds.size()) + " bonds";
    if (d.p.style == view_style::cartoon)
      s += ", " + std::to_string(d.out.residues) + " residues";
    return s;
  }
  return phase();
}

void build_view_stepper::cancel() {
  _impl->cancel_requested.store(true, std::memory_order_release);
}

molecule_view build_view_stepper::take() {
  impl &d = *_impl;
  if (d.st == impl::stage::failed)
    std::rethrow_exception(d.error);
  if (d.st != impl::stage::done)
    throw std::logic_error("build_view_stepper::take: not done");
  if (!d.any_bounds)
    d.out.bounds = {0, 0, 0, 0, 0, 0};
  return std::move(d.out);
}

molecule_view build_view(app &ctx, std::shared_ptr<const molecule> m, const view_params &p) {
  build_view_stepper s(ctx, std::move(m), p);
  s.step(0);
  return s.take();
}

molecule_view build_view(app &ctx, const molecule &m, const view_params &p) {
  return build_view(ctx, std::make_shared<const molecule>(m), p);
}

std::unique_ptr<result_job<molecule_view>>
build_view_job(app &ctx, std::shared_ptr<const molecule> m, const view_params &p) {
  return make_result_job(std::make_unique<build_view_stepper>(ctx, std::move(m), p));
}

int pick_sphere(const molecule_view &v, const double origin[3], const double dir[3], double *t,
                double *hit) {
  const vec3 o{origin[0], origin[1], origin[2]};
  const vec3 d = normalized({dir[0], dir[1], dir[2]}, vec3{0.0, 0.0, 0.0});
  if (length(d) == 0.0)
    return -1;
  int best = -1;
  double best_t = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < v.radii.size(); ++i) {
    const vec3 c{v.centers[3 * i], v.centers[3 * i + 1], v.centers[3 * i + 2]};
    const double r = v.radii[i];
    const vec3 oc = o - c;
    const double b = dot(oc, d), cc = dot(oc, oc) - r * r;
    const double disc = b * b - cc;
    if (disc < 0.0)
      continue;
    const double s = std::sqrt(disc);
    double ti = -b - s;
    if (ti < 0.0)
      ti = -b + s; // the ray starts inside the sphere
    if (ti < 0.0 || ti >= best_t)
      continue;
    best_t = ti;
    best = v.sphere_atoms[i];
  }
  if (best >= 0) {
    if (t)
      *t = best_t;
    if (hit) {
      hit[0] = o.x + best_t * d.x;
      hit[1] = o.y + best_t * d.y;
      hit[2] = o.z + best_t * d.z;
    }
  }
  return best;
}

int nearest_drawn_atom(const molecule &m, const molecule_view &v, const double p[3]) {
  const vec3 q{p[0], p[1], p[2]};
  int best = -1;
  double best_d = std::numeric_limits<double>::infinity();
  const auto consider = [&](int i) {
    if (i < 0 || static_cast<std::size_t>(i) >= m.atoms.size())
      return;
    const double d = length(position(m.atoms[i]) - q);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  };
  for (int i : v.sphere_atoms)
    consider(i);
  int last = -1;
  for (int i : v.cartoon_atoms)
    if (i != last) {
      consider(i);
      last = i;
    }
  for (const bond &b : v.bonds) {
    consider(b.a);
    consider(b.b);
  }
  return best;
}

} // namespace mol
} // namespace cvc
