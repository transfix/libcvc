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

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// fem.cpp -- the cvc::geometry side of fem.h: element selection (domain,
// degenerate elements, tet orientation), boundary conditions and argument
// checks. Assembly and solves happen behind igl/igl_detail.h; like
// mesh_ops.cpp this TU never sees Eigen or libigl.

#include <algorithm>
#include <cmath>
#include <cvc/geometry/fem.h>
#include <exception>
#include <limits>
#include <memory>
#include <numeric>

#ifdef CVC_ENABLE_LIBIGL
#include "igl/igl_bridge.h"
#include "igl/igl_detail.h"
#endif

namespace cvc {
namespace fem {

#ifdef CVC_ENABLE_LIBIGL

using igl_bridge::guarded;

namespace {

// The elements of a mesh, as fem.h selects them.
struct element_set {
  std::vector<double> V;
  int simplex_size = 3;
  std::size_t nv = 0;
  std::vector<int> all;           // every element, in element order
  std::vector<int> kept;          // the non-degenerate ones; tets positively oriented
  std::vector<std::size_t> which; // position in `all` of each kept element
  std::vector<char> active;       // vertex is a corner of a kept element
};

element_set gather(const geometry &mesh, domain d) {
  if (mesh.num_points() == 0)
    throw mesh_ops_error("fem: the mesh has no points");
  element_set es;
  es.V = igl_bridge::positions(mesh);
  es.nv = mesh.num_points();
  const bool volume = d == domain::VOLUME || (d == domain::AUTO && !mesh.const_tets().empty());
  es.simplex_size = volume ? 4 : 3;
  es.all = volume ? igl_bridge::tetrahedra(mesh) : igl_bridge::surface_triangles(mesh);
  if (es.all.empty())
    throw mesh_ops_error(volume ? "fem: the mesh has no tets" : "fem: the mesh has no triangles");
  igl_bridge::check_fem_count(es.all.size() / std::size_t(es.simplex_size), "fem");

  // Degenerate: |volume| (or area) below 1e-12 of the bounding-box scale, or
  // too flat for igl::cotmatrix's edge-length formulas (see fem.h).
  const double diag = igl_bridge::diagonal(es.V);
  const int ss = es.simplex_size;
  std::vector<double> measure;
  double tol;
  if (volume) {
    igl_detail::tet_volumes(es.V, es.all, measure);
    tol = 1e-12 * diag * diag * diag;
  } else {
    igl_detail::double_areas(es.V, es.all, measure);
    for (double &a : measure)
      a *= 0.5;
    tol = 1e-12 * diag * diag;
  }
  std::vector<char> shaped;
  igl_detail::well_shaped(es.V, es.all, ss, shaped);
  es.active.assign(es.nv, 0);
  for (std::size_t e = 0; e < measure.size(); ++e) {
    const double m = std::abs(measure[e]);
    if (!(m > 0) || m < tol || !std::isfinite(m) || !shaped[e])
      continue;
    const std::size_t at = es.kept.size();
    es.kept.insert(es.kept.end(), es.all.begin() + std::ptrdiff_t(ss * e),
                   es.all.begin() + std::ptrdiff_t(ss * (e + 1)));
    if (volume && measure[e] < 0)
      std::swap(es.kept[at + 2], es.kept[at + 3]);
    es.which.push_back(e);
    for (int c = 0; c < ss; ++c)
      es.active[std::size_t(es.all[ss * e + c])] = 1;
  }
  return es;
}

sparse_matrix to_sparse(const igl_detail::triplets &t, std::size_t n) {
  sparse_matrix s;
  s.rows = s.cols = n;
  s.row.assign(t.row.begin(), t.row.end());
  s.col.assign(t.col.begin(), t.col.end());
  s.value = t.val;
  return s;
}

// Known vertices of a solve: the Dirichlet ones (first value wins on a repeat),
// then every vertex no kept element touches, held at 0.
struct constraints {
  std::vector<int> known;
  std::vector<double> values;
};

constraints make_constraints(const element_set &es, const dirichlet &bc) {
  if (!bc.vertices.empty() && bc.values.size() != 1 && bc.values.size() != bc.vertices.size())
    throw mesh_ops_error("fem: dirichlet needs one value, or one per vertex");
  constraints c;
  std::vector<char> seen(es.nv, 0);
  for (std::size_t i = 0; i < bc.vertices.size(); ++i) {
    const index_t v = bc.vertices[i];
    if (v >= es.nv)
      throw mesh_ops_error("fem: dirichlet vertex " + std::to_string(v) + " >= num_points()");
    if (seen[v])
      continue;
    seen[v] = 1;
    c.known.push_back(int(v));
    c.values.push_back(bc.values.size() == 1 ? bc.values[0] : bc.values[i]);
  }
  for (std::size_t v = 0; v < es.nv; ++v)
    if (!es.active[v] && !seen[v]) {
      c.known.push_back(int(v));
      c.values.push_back(0.0);
    }
  return c;
}

// K alone is singular on any connected part with no Dirichlet vertex.
void require_anchored(const element_set &es, const dirichlet &bc) {
  std::vector<int> parent(es.nv);
  std::iota(parent.begin(), parent.end(), 0);
  const auto root = [&parent](int i) {
    while (parent[std::size_t(i)] != i)
      i = parent[std::size_t(i)] = parent[std::size_t(parent[std::size_t(i)])];
    return i;
  };
  const int ss = es.simplex_size;
  for (std::size_t e = 0; e < es.kept.size(); e += std::size_t(ss))
    for (int c = 1; c < ss; ++c) {
      const int a = root(es.kept[e]), b = root(es.kept[e + std::size_t(c)]);
      if (a != b)
        parent[std::size_t(std::max(a, b))] = std::min(a, b);
    }
  std::vector<char> anchored(es.nv, 0);
  for (index_t v : bc.vertices)
    anchored[std::size_t(root(int(v)))] = 1;
  for (std::size_t v = 0; v < es.nv; ++v)
    if (es.active[v] && !anchored[std::size_t(root(int(v)))])
      throw mesh_ops_error("fem: a connected part of the mesh has no Dirichlet vertex "
                           "(the problem would be singular)");
}

} // namespace

std::vector<index_t> boundary_vertices(const geometry &mesh, domain d) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    std::vector<int> B;
    igl_detail::boundary_facets(es.all, es.simplex_size, B);
    std::vector<index_t> out(B.begin(), B.end());
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
  });
}

sparse_matrix stiffness_matrix(const geometry &mesh, domain d) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    igl_detail::triplets K;
    igl_detail::stiffness(es.V, es.kept, es.simplex_size, K);
    return to_sparse(K, es.nv);
  });
}

sparse_matrix mass_matrix(const geometry &mesh, domain d, bool lumped) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    igl_detail::triplets M;
    igl_detail::mass(es.V, es.kept, es.simplex_size, lumped, M);
    return to_sparse(M, es.nv);
  });
}

std::vector<double> gradient(const geometry &mesh, const std::vector<double> &u, domain d) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    if (u.size() != es.nv)
      throw mesh_ops_error("fem::gradient: u needs one value per point");
    std::vector<double> g;
    igl_detail::gradient(es.V, es.kept, es.simplex_size, u, g);
    // Degenerate elements have no gradient; report 0 for them.
    std::vector<double> out(3 * (es.all.size() / std::size_t(es.simplex_size)), 0.0);
    for (std::size_t k = 0; k < es.which.size(); ++k)
      for (int c = 0; c < 3; ++c)
        out[3 * es.which[k] + std::size_t(c)] = g[3 * k + std::size_t(c)];
    return out;
  });
}

std::vector<double> solve_poisson(const geometry &mesh, const std::vector<double> &f,
                                  const dirichlet &bc, domain d) {
  if (bc.vertices.empty())
    throw mesh_ops_error("fem::solve_poisson: no Dirichlet condition (the problem is singular)");
  return guarded([&] {
    const element_set es = gather(mesh, d);
    std::vector<double> rhs;
    if (f.size() == 1)
      rhs.assign(es.nv, f[0]);
    else if (f.size() == es.nv)
      rhs = f;
    else
      throw mesh_ops_error("fem::solve_poisson: f needs one value, or one per point");
    const constraints c = make_constraints(es, bc);
    require_anchored(es, bc);
    const auto sys =
        igl_detail::make_quadratic_system(es.V, es.kept, es.simplex_size, 0.0, 1.0, c.known);
    std::vector<double> u;
    igl_detail::solve_quadratic(*sys, rhs, c.values, u);
    return u;
  });
}

std::vector<double> solve_laplace(const geometry &mesh, const dirichlet &bc, domain d) {
  if (bc.vertices.empty())
    throw mesh_ops_error("fem::solve_laplace: no Dirichlet condition (the problem is singular)");
  return solve_poisson(mesh, std::vector<double>(1, 0.0), bc, d);
}

std::vector<double> solve_heat(const geometry &mesh, const std::vector<double> &u0,
                               const heat_params &p, const dirichlet &bc, domain d,
                               const heat_progress &progress) {
  if (!(p.dt > 0) || !std::isfinite(p.dt) || !(p.kappa >= 0) || !std::isfinite(p.kappa))
    throw mesh_ops_error("fem::solve_heat: dt must be positive and kappa non-negative");
  return guarded([&] {
    const element_set es = gather(mesh, d);
    if (u0.size() != es.nv)
      throw mesh_ops_error("fem::solve_heat: u0 needs one value per point");
    if (p.steps <= 0)
      return u0;
    const constraints c = make_constraints(es, bc);
    // Backward Euler, (M + dt kappa K) u+ = M u: one factorization for all steps.
    const auto sys = igl_detail::make_quadratic_system(es.V, es.kept, es.simplex_size, 1.0,
                                                       p.dt * p.kappa, c.known);
    std::vector<double> u = u0, next;
    for (int step = 1; step <= p.steps; ++step) {
      igl_detail::solve_quadratic(*sys, u, c.values, next);
      u.swap(next);
      if (progress && !progress(step, u))
        break;
    }
    return u;
  });
}

eigen_result laplacian_eigenmodes(const geometry &mesh, int k, domain d) {
  return guarded([&] {
    const element_set es = gather(mesh, d);
    eigen_result r;
    igl_detail::eigenmodes(es.V, es.kept, es.simplex_size, es.active, k, r.values, r.vectors);
    return r;
  });
}

// --- stepped_solve -------------------------------------------------------------------------------

namespace {

constexpr std::size_t kElementUnits = 40; // one element's measure, shape test and assembly
constexpr std::size_t kChunk = 256;       // elements assembled per libigl call

} // namespace

struct stepped_solve::impl {
  enum class kind { poisson, heat, modes };
  enum class stage {
    start,
    prepare,   // the elements' measures and shapes, a chunk at a time
    assemble,  // K and the lumped M, a chunk of elements at a time
    build,     // the sparse matrix from the triplets
    constrain, // Dirichlet vertices eliminated (poisson, heat) or the active set (modes)
    order,     // the AMD ordering: one step
    factor,
    solve,   // poisson's solve, a heat step's, or one of a modes iteration's columns
    iterate, // modes: the update after an iteration's solves
    done,
    failed
  };

  kind what = kind::poisson;
  stage st = stage::start;
  std::exception_ptr error;
  double failed_progress = 0.0;
  bool stop_asked = false;

  std::shared_ptr<const geometry> mesh;
  domain dom = domain::AUTO;
  std::vector<double> f;
  dirichlet bc;
  heat_params hp;
  int k = 0;

  // prepare
  element_set es;
  std::vector<double> measure;
  std::vector<char> shaped;
  double tol = 0.0;
  bool volume = false;
  std::size_t pos = 0;
  // assemble
  std::vector<int> tr, tc;
  std::vector<double> tv, mdiag;
  std::unique_ptr<sliced_detail::csc_builder> builder;
  sliced_detail::csc K; // stiff_coef K + mass_coef M (poisson, heat), or K (modes)
  // constrain
  constraints con;
  std::vector<int> local, global; // vertex -> unknown (or active) index, and back
  sliced_detail::csc A;           // the reduced system
  std::vector<double> g;          // A_uk y (poisson, heat)
  std::vector<int> perm;
  std::unique_ptr<sliced_detail::ldlt> fac;
  // solve / heat
  std::vector<double> u; // the field (poisson's result, heat's current)
  int steps = 0;
  bool solving = false;
  // modes
  sliced_detail::csc C; // K + shift M
  std::vector<double> mloc, X, Y, theta, prev;
  std::uint64_t seed = 0;
  double lmax = 0.0;
  int p = 0, col = 0, iteration = 0;
  eigen_result modes_out;

  std::size_t left = 0;
  std::uint64_t work = 0;

  void spend(std::size_t units) {
    work += units;
    left = units >= left ? 0 : left - units;
  }

  void begin() {
    if (!mesh || mesh->num_points() == 0)
      throw mesh_ops_error("fem: the mesh has no points");
    es.V = igl_bridge::positions(*mesh);
    es.nv = mesh->num_points();
    volume = dom == domain::VOLUME || (dom == domain::AUTO && !mesh->const_tets().empty());
    es.simplex_size = volume ? 4 : 3;
    es.all = volume ? igl_bridge::tetrahedra(*mesh) : igl_bridge::surface_triangles(*mesh);
    if (es.all.empty())
      throw mesh_ops_error(volume ? "fem: the mesh has no tets" : "fem: the mesh has no triangles");
    igl_bridge::check_fem_count(es.all.size() / std::size_t(es.simplex_size), "fem");
    const double diag = igl_bridge::diagonal(es.V);
    tol = volume ? 1e-12 * diag * diag * diag : 1e-12 * diag * diag;
    es.active.assign(es.nv, 0);
    mdiag.assign(es.nv, 0.0);
    if (what == kind::poisson) {
      if (f.size() == 1)
        f.assign(es.nv, f[0]);
      else if (f.size() != es.nv)
        throw mesh_ops_error("fem::solve_poisson: f needs one value, or one per point");
    }
    if (what == kind::heat) {
      if (f.size() != es.nv)
        throw mesh_ops_error("fem::solve_heat: u0 needs one value per point");
      u = f;
      if (hp.steps <= 0) {
        st = stage::done;
        return;
      }
    }
    if (what == kind::modes && k <= 0) {
      st = stage::done;
      return;
    }
    st = stage::prepare;
    pos = 0;
  }

  // gather()'s selection, a chunk of elements at a time.
  void prepare() {
    const int ss = es.simplex_size;
    const std::size_t ne = es.all.size() / std::size_t(ss);
    while (left > 0 && pos < ne) {
      const std::size_t e0 = pos, e1 = std::min(ne, pos + kChunk);
      const std::vector<int> chunk(es.all.begin() + std::ptrdiff_t(ss * e0),
                                   es.all.begin() + std::ptrdiff_t(ss * e1));
      if (volume) {
        igl_detail::tet_volumes(es.V, chunk, measure);
      } else {
        igl_detail::double_areas(es.V, chunk, measure);
        for (double &a : measure)
          a *= 0.5;
      }
      igl_detail::well_shaped(es.V, chunk, ss, shaped);
      for (std::size_t e = e0; e < e1; ++e) {
        const double m = std::abs(measure[e - e0]);
        if (!(m > 0) || m < tol || !std::isfinite(m) || !shaped[e - e0])
          continue;
        const std::size_t at = es.kept.size();
        es.kept.insert(es.kept.end(), es.all.begin() + std::ptrdiff_t(ss * e),
                       es.all.begin() + std::ptrdiff_t(ss * (e + 1)));
        if (volume && measure[e - e0] < 0)
          std::swap(es.kept[at + 2], es.kept[at + 3]);
        es.which.push_back(e);
        for (int c = 0; c < ss; ++c)
          es.active[std::size_t(es.all[std::size_t(ss) * e + std::size_t(c)])] = 1;
      }
      spend((e1 - e0) * 4);
      pos = e1;
    }
    if (pos == ne) {
      measure = std::vector<double>();
      shaped = std::vector<char>();
      st = stage::assemble;
      pos = 0;
    }
  }

  double stiff_coef() const { return what == kind::heat ? hp.dt * hp.kappa : 1.0; }

  // K (scaled) and the lumped mass, libigl's formulas on a chunk of the kept elements: both are
  // sums over the elements, so chunks add up to the one-shot matrices.
  void assemble() {
    const int ss = es.simplex_size;
    const std::size_t ne = es.kept.size() / std::size_t(ss);
    igl_detail::triplets Kt, Mt;
    while (left > 0 && pos < ne) {
      const std::size_t e0 = pos, e1 = std::min(ne, pos + kChunk);
      const std::vector<int> chunk(es.kept.begin() + std::ptrdiff_t(ss * e0),
                                   es.kept.begin() + std::ptrdiff_t(ss * e1));
      igl_detail::stiffness(es.V, chunk, ss, Kt);
      igl_detail::mass(es.V, chunk, ss, true, Mt);
      const double sc = what == kind::modes ? 1.0 : stiff_coef();
      for (std::size_t t = 0; t < Kt.val.size(); ++t) {
        tr.push_back(Kt.row[t]);
        tc.push_back(Kt.col[t]);
        tv.push_back(sc * Kt.val[t]);
      }
      for (std::size_t t = 0; t < Mt.val.size(); ++t)
        if (Mt.row[t] == Mt.col[t]) // lumped: diagonal
          mdiag[std::size_t(Mt.row[t])] += Mt.val[t];
      spend((e1 - e0) * kElementUnits);
      pos = e1;
    }
    if (pos < ne)
      return;
    if (what == kind::heat) // A = M + dt kappa K
      for (std::size_t v = 0; v < es.nv; ++v)
        if (mdiag[v] != 0.0) {
          tr.push_back(int(v));
          tc.push_back(int(v));
          tv.push_back(mdiag[v]);
        }
    builder = std::make_unique<sliced_detail::csc_builder>(int(es.nv), std::move(tr), std::move(tc),
                                                           std::move(tv));
    tr.clear();
    tc.clear();
    tv.clear();
    st = stage::build;
  }

  void build() {
    const std::size_t before = left;
    const bool built = builder->step(left);
    spend(before); // the builder used about its budget
    if (!built)
      return;
    K = builder->take();
    builder.reset();
    st = stage::constrain;
    pos = 0;
    setup_done = false;
  }

  // poisson / heat: eliminate the known vertices (Dirichlet, and those no element touches): A is
  // K's unknown-unknown block and g = K_uk y. A column at a time, after a one-step setup.
  bool setup_done = false;
  void constrain() {
    if (what == kind::modes) {
      constrain_modes();
      return;
    }
    if (!setup_done) {
      setup_done = true;
      con = make_constraints(es, bc);
      if (what == kind::poisson)
        require_anchored(es, bc);
      local.assign(es.nv, -1);
      std::vector<char> known(es.nv, 0);
      u.resize(es.nv, 0.0);
      for (std::size_t q = 0; q < con.known.size(); ++q) {
        known[std::size_t(con.known[q])] = 1;
        u[std::size_t(con.known[q])] = con.values[q]; // u holds y on the known vertices
      }
      global.clear();
      for (std::size_t v = 0; v < es.nv; ++v)
        if (!known[v]) {
          local[v] = int(global.size());
          global.push_back(int(v));
        }
      A.n = int(global.size());
      A.p.assign(std::size_t(A.n) + 1, 0);
      g.assign(std::size_t(A.n), 0.0);
      spend(es.nv);
    }
    while (left > 0 && pos < global.size()) {
      const int j = global[pos];
      for (int q = K.p[std::size_t(j)]; q < K.p[std::size_t(j) + 1]; ++q) {
        const int i = K.i[std::size_t(q)];
        const int li = local[std::size_t(i)];
        if (li >= 0) {
          A.i.push_back(li);
          A.x.push_back(K.x[std::size_t(q)]);
        } else {
          g[pos] += K.x[std::size_t(q)] * u[std::size_t(i)];
        }
      }
      A.p[pos + 1] = int(A.i.size());
      spend(std::size_t(K.p[std::size_t(j) + 1] - K.p[std::size_t(j)]) + 1);
      ++pos;
    }
    if (pos < global.size())
      return;
    if (A.n == 0) { // every vertex is known
      finish_field();
      return;
    }
    st = stage::order;
  }

  // modes: the active vertices' K and m, and C = K + 1e-6 lmax M to factor.
  void constrain_modes() {
    local.assign(es.nv, -1);
    global.clear();
    for (std::size_t v = 0; v < es.nv; ++v)
      if (es.active[v]) {
        local[v] = int(global.size());
        global.push_back(int(v));
      }
    const int na = int(global.size());
    if (na == 0 || es.kept.empty()) {
      release();
      st = stage::done;
      return;
    }
    k = std::min(k, na);
    sliced_detail::csc Kl;
    Kl.n = na;
    Kl.p.assign(std::size_t(na) + 1, 0);
    mloc.assign(std::size_t(na), 0.0);
    for (int a = 0; a < na; ++a) {
      const int j = global[std::size_t(a)];
      for (int q = K.p[std::size_t(j)]; q < K.p[std::size_t(j) + 1]; ++q) {
        const int li = local[std::size_t(K.i[std::size_t(q)])];
        if (li < 0)
          continue;
        Kl.i.push_back(li);
        Kl.x.push_back(K.x[std::size_t(q)]);
      }
      Kl.p[std::size_t(a) + 1] = int(Kl.i.size());
      mloc[std::size_t(a)] = mdiag[std::size_t(j)];
    }
    spend(K.x.size() + es.nv);
    K = std::move(Kl);
    double mmin = HUGE_VAL;
    for (double mi : mloc)
      mmin = std::min(mmin, mi);
    if (!(mmin > 0))
      throw mesh_ops_error("laplacian_eigenmodes: a vertex has no mass");
    if (na <= 400) { // laplacian_eigenmodes' dense path: one step
      igl_detail::dense_modes(K, mloc, k, theta, modes_out.vectors);
      spend(std::size_t(na) * std::size_t(na));
      finish_modes();
      return;
    }
    p = std::min(na, std::max(2 * k, k + 8));
    lmax = 0.0;
    for (int a = 0; a < na; ++a)
      for (int q = K.p[std::size_t(a)]; q < K.p[std::size_t(a) + 1]; ++q)
        if (K.i[std::size_t(q)] == a)
          lmax = std::max(lmax, std::abs(K.x[std::size_t(q)]) / mloc[std::size_t(a)]);
    C = K;
    for (int a = 0; a < na; ++a)
      for (int q = C.p[std::size_t(a)]; q < C.p[std::size_t(a) + 1]; ++q)
        if (C.i[std::size_t(q)] == a)
          C.x[std::size_t(q)] += 1e-6 * lmax * mloc[std::size_t(a)];
    seed = 0;
    igl_detail::subspace_start(na, p, X, seed);
    Y.assign(X.size(), 0.0);
    st = stage::order;
  }

  const sliced_detail::csc &system() const { return what == kind::modes ? C : A; }

  void order() {
    igl_detail::amd_ordering(system(), perm);
    spend(system().x.size() * 4);
    fac = std::make_unique<sliced_detail::ldlt>(&system(), perm);
    st = stage::factor;
  }

  void factor() {
    try {
      const std::size_t before = left;
      const bool ok = fac->step(left);
      spend(before);
      if (!ok)
        return;
    } catch (const std::runtime_error &e) {
      throw mesh_ops_error(e.what());
    }
    st = stage::solve;
    solving = false;
    col = 0;
  }

  // The right-hand side of the next solve.
  std::vector<double> rhs() const {
    if (what == kind::modes) {
      const int n = K.n;
      std::vector<double> b(static_cast<std::size_t>(n), 0.0);
      for (int i = 0; i < n; ++i)
        b[std::size_t(i)] =
            mloc[std::size_t(i)] * X[std::size_t(col) * std::size_t(n) + std::size_t(i)];
      return b;
    }
    // min_quad_with_fixed's A_uu z = -B_u - A_uk y with B = -M f: (M f)_u - g.
    const std::vector<double> &src = what == kind::poisson ? f : u;
    std::vector<double> b(global.size());
    for (std::size_t a = 0; a < global.size(); ++a) {
      const std::size_t v = std::size_t(global[a]);
      b[a] = mdiag[v] * src[v] - g[a];
    }
    return b;
  }

  void solve() {
    if (!solving) {
      fac->start_solve(rhs());
      solving = true;
      spend(global.size() + 1);
    }
    const std::size_t before = left;
    const bool solved = fac->solve_step(left);
    spend(before);
    if (!solved)
      return;
    solving = false;
    const std::vector<double> &x = fac->solution();
    for (double xi : x)
      if (!std::isfinite(xi))
        throw mesh_ops_error("the solve produced non-finite values (singular system?)");
    if (what == kind::modes) {
      const int n = K.n;
      std::copy(x.begin(), x.end(), Y.begin() + std::ptrdiff_t(col) * n);
      if (++col == p)
        st = stage::iterate;
      return;
    }
    for (std::size_t a = 0; a < global.size(); ++a)
      u[std::size_t(global[a])] = x[a];
    if (what == kind::poisson) {
      finish_field();
      return;
    }
    ++steps;
    if (steps >= hp.steps || stop_asked)
      finish_field();
  }

  void iterate() {
    const bool converged =
        igl_detail::subspace_update(K, mloc, k, lmax, p, Y, X, theta, prev, seed);
    spend(std::size_t(K.n) * std::size_t(p) * std::size_t(p) + K.x.size() * std::size_t(p));
    ++iteration;
    if (converged) {
      modes_out.vectors.assign(std::size_t(k), std::vector<double>());
      const int n = K.n;
      for (int i = 0; i < k; ++i)
        modes_out.vectors[std::size_t(i)].assign(X.begin() + std::ptrdiff_t(i) * n,
                                                 X.begin() + std::ptrdiff_t(i + 1) * n);
      finish_modes();
      return;
    }
    if (iteration >= 500)
      throw mesh_ops_error("laplacian_eigenmodes: the subspace iteration did not converge in 500 "
                           "iterations (a very clustered spectrum? ask for fewer modes)");
    col = 0;
    st = stage::solve;
  }

  void finish_field() {
    release();
    st = stage::done;
  }

  // The local vectors back on every vertex, each with the sign laplacian_eigenmodes gives it.
  void finish_modes() {
    const std::size_t kk = std::min<std::size_t>(std::size_t(k), theta.size());
    modes_out.values.assign(theta.begin(), theta.begin() + std::ptrdiff_t(kk));
    std::vector<std::vector<double>> out(kk, std::vector<double>(es.nv, 0.0));
    for (std::size_t i = 0; i < kk && i < modes_out.vectors.size(); ++i) {
      const std::vector<double> &v = modes_out.vectors[i];
      std::size_t big = 0;
      for (std::size_t j = 1; j < v.size(); ++j)
        if (std::abs(v[j]) > std::abs(v[big]))
          big = j;
      const double sign = !v.empty() && v[big] < 0 ? -1.0 : 1.0;
      for (std::size_t j = 0; j < v.size(); ++j)
        out[i][std::size_t(global[j])] = sign * v[j];
    }
    modes_out.vectors = std::move(out);
    release();
    st = stage::done;
  }

  void release() {
    fac.reset();
    builder.reset();
    A = sliced_detail::csc();
    C = sliced_detail::csc();
    K = sliced_detail::csc();
    X = Y = std::vector<double>();
    es = element_set();
    mesh.reset();
  }

  void run(std::size_t budget) {
    left = budget;
    if (st == stage::start)
      begin();
    while (left > 0) {
      switch (st) {
      case stage::prepare:
        prepare();
        break;
      case stage::assemble:
        assemble();
        break;
      case stage::build:
        build();
        break;
      case stage::constrain:
        constrain();
        break;
      case stage::order:
        order();
        break;
      case stage::factor:
        factor();
        break;
      case stage::solve:
        solve();
        break;
      case stage::iterate:
        iterate();
        break;
      default:
        return;
      }
    }
  }

  double progress() const {
    switch (st) {
    case stage::start:
      return 0.0;
    case stage::done:
      return 1.0;
    case stage::failed:
      return failed_progress;
    default:
      break;
    }
    // Rough shares: selection 5%, assembly 15%, build 5%, constraints 5%, ordering 5%, the
    // factorization 45%, the solves 20% (heat: its steps; modes: its iterations, up to ~20).
    const double ss = double(std::max(es.simplex_size, 1));
    const double ne = std::max(1.0, double(es.all.size()) / ss);
    const double nk = std::max(1.0, double(es.kept.size()) / ss);
    switch (st) {
    case stage::prepare:
      return 0.05 * double(pos) / ne;
    case stage::assemble:
      return 0.05 + 0.15 * double(pos) / nk;
    case stage::build:
      return 0.20 + 0.05 * (builder ? builder->progress() : 0.0);
    case stage::constrain:
      return 0.25 + 0.05 * double(pos) / std::max(1.0, double(global.size()));
    case stage::order:
      return 0.30;
    case stage::factor:
      return 0.35 + 0.45 * (fac ? fac->progress() : 0.0);
    default:
      break;
    }
    double tail = 0.0;
    if (what == kind::heat)
      tail = double(steps) / std::max(1.0, double(hp.steps));
    else if (what == kind::modes)
      tail = std::min(0.95, (double(iteration) + double(col) / std::max(1, p)) / 20.0);
    return std::min(0.999, 0.80 + 0.20 * tail);
  }

  void fail(std::exception_ptr e) {
    failed_progress = progress();
    error = std::move(e);
    st = stage::failed;
    release();
  }
};

stepped_solve::stepped_solve() : _impl(std::make_unique<impl>()) {}
stepped_solve::~stepped_solve() = default;

std::unique_ptr<stepped_solve> stepped_solve::poisson(std::shared_ptr<const geometry> mesh,
                                                      std::vector<double> f, dirichlet bc,
                                                      domain d) {
  if (bc.vertices.empty())
    throw mesh_ops_error("fem::solve_poisson: no Dirichlet condition (the problem is singular)");
  std::unique_ptr<stepped_solve> s(new stepped_solve());
  s->_impl->what = impl::kind::poisson;
  s->_impl->mesh = std::move(mesh);
  s->_impl->f = std::move(f);
  s->_impl->bc = std::move(bc);
  s->_impl->dom = d;
  return s;
}

std::unique_ptr<stepped_solve> stepped_solve::heat(std::shared_ptr<const geometry> mesh,
                                                   std::vector<double> u0, heat_params p,
                                                   dirichlet bc, domain d) {
  if (!(p.dt > 0) || !std::isfinite(p.dt) || !(p.kappa >= 0) || !std::isfinite(p.kappa))
    throw mesh_ops_error("fem::solve_heat: dt must be positive and kappa non-negative");
  std::unique_ptr<stepped_solve> s(new stepped_solve());
  s->_impl->what = impl::kind::heat;
  s->_impl->mesh = std::move(mesh);
  s->_impl->f = std::move(u0);
  s->_impl->hp = p;
  s->_impl->bc = std::move(bc);
  s->_impl->dom = d;
  return s;
}

std::unique_ptr<stepped_solve> stepped_solve::modes(std::shared_ptr<const geometry> mesh, int k,
                                                    domain d) {
  std::unique_ptr<stepped_solve> s(new stepped_solve());
  s->_impl->what = impl::kind::modes;
  s->_impl->mesh = std::move(mesh);
  s->_impl->k = k;
  s->_impl->dom = d;
  return s;
}

bool stepped_solve::step(std::size_t budget) {
  impl &s = *_impl;
  if (s.st == impl::stage::failed)
    std::rethrow_exception(s.error);
  if (s.st == impl::stage::done)
    return true;
  try {
    guarded([&] {
      s.run(budget == 0 ? std::numeric_limits<std::size_t>::max() : budget);
      return 0;
    });
  } catch (...) {
    s.fail(std::current_exception());
    throw;
  }
  return s.st == impl::stage::done;
}

bool stepped_solve::done() const {
  return _impl->st == impl::stage::done || _impl->st == impl::stage::failed;
}

double stepped_solve::progress() const { return _impl->progress(); }

const char *stepped_solve::phase() const {
  switch (_impl->st) {
  case impl::stage::start:
  case impl::stage::prepare:
    return "preparing";
  case impl::stage::assemble:
    return "assembling";
  case impl::stage::build:
    return "building";
  case impl::stage::constrain:
    return "constraining";
  case impl::stage::order:
    return "ordering";
  case impl::stage::factor:
    return "factoring";
  case impl::stage::solve:
    return _impl->what == impl::kind::heat    ? "stepping"
           : _impl->what == impl::kind::modes ? "iterating"
                                              : "solving";
  case impl::stage::iterate:
    return "iterating";
  case impl::stage::done:
  case impl::stage::failed:
    break;
  }
  return "done";
}

void stepped_solve::stop() { _impl->stop_asked = true; }

int stepped_solve::steps_done() const { return _impl->steps; }

const std::vector<double> &stepped_solve::field() const { return _impl->u; }

eigen_result stepped_solve::take_modes() { return std::move(_impl->modes_out); }

#else // !CVC_ENABLE_LIBIGL

namespace {
[[noreturn]] void unavailable() {
  throw mesh_ops_unavailable("libcvc was built without CVC_ENABLE_LIBIGL");
}
} // namespace

std::vector<index_t> boundary_vertices(const geometry &, domain) { unavailable(); }
sparse_matrix stiffness_matrix(const geometry &, domain) { unavailable(); }
sparse_matrix mass_matrix(const geometry &, domain, bool) { unavailable(); }
std::vector<double> gradient(const geometry &, const std::vector<double> &, domain) {
  unavailable();
}
std::vector<double> solve_poisson(const geometry &, const std::vector<double> &, const dirichlet &,
                                  domain) {
  unavailable();
}
std::vector<double> solve_laplace(const geometry &, const dirichlet &, domain) { unavailable(); }
std::vector<double> solve_heat(const geometry &, const std::vector<double> &, const heat_params &,
                               const dirichlet &, domain, const heat_progress &) {
  unavailable();
}
eigen_result laplacian_eigenmodes(const geometry &, int, domain) { unavailable(); }

struct stepped_solve::impl {};
stepped_solve::stepped_solve() = default;
stepped_solve::~stepped_solve() = default;
std::unique_ptr<stepped_solve> stepped_solve::poisson(std::shared_ptr<const geometry>,
                                                      std::vector<double>, dirichlet, domain) {
  unavailable();
}
std::unique_ptr<stepped_solve> stepped_solve::heat(std::shared_ptr<const geometry>,
                                                   std::vector<double>, heat_params, dirichlet,
                                                   domain) {
  unavailable();
}
std::unique_ptr<stepped_solve> stepped_solve::modes(std::shared_ptr<const geometry>, int, domain) {
  unavailable();
}
bool stepped_solve::step(std::size_t) { unavailable(); }
bool stepped_solve::done() const { return true; }
double stepped_solve::progress() const { return 1.0; }
const char *stepped_solve::phase() const { return "done"; }
void stepped_solve::stop() {}
int stepped_solve::steps_done() const { return 0; }
const std::vector<double> &stepped_solve::field() const {
  static const std::vector<double> none;
  return none;
}
eigen_result stepped_solve::take_modes() { return {}; }

#endif // CVC_ENABLE_LIBIGL

} // namespace fem
} // namespace cvc
