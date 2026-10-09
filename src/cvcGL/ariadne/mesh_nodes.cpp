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

// mesh_nodes.cpp -- the mesh_lab / fe_lab scene node realizers (see mesh_nodes.h). Each copies the
// forest_trees idiom (seed keys from props, poll them every frame) with node-scoped keys like
// VolRenNode's, and runs the cvc::mesh_ops / cvc::fem kernels on a per-node async_lane.
//
// Threading. The tick runs on the render thread; it marshals every input into a job before the
// submit and applies the job's result after it finishes. A job never touches cvc::state, VTK or
// the runtime object: it captures only its handoff struct and immutable inputs. Capturing the
// runtime would let the last reference drop on the lane thread, and ~async_lane would then join
// itself. Teardown sets each job's cancel flag; a heat solve stops at its next step, a queued job
// returns at once, and the lane's destructor waits for the running one.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/geometry/fem.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/mesh_ops.h>
#include <cvc/geometry/simplify.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/mesh_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/utility/algorithm.h> // cvc::tetrahedralize (CVC_ENABLE_MESHER)
#include <cvc/volume/volume.h>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace gl {
namespace ariadne {

#ifdef CVC_ENABLE_LIBIGL
namespace {

namespace ari = cvc::ariadne;

// --- shared helpers ------------------------------------------------------------------------------

std::string seed_num(cvc::app &a, const std::string &base, const char *key, double def) {
  const std::string p = ari::resolve_bind(base, key);
  ari::read_or_seed<double>(a, p, def);
  return p;
}
std::string seed_str(cvc::app &a, const std::string &base, const char *key,
                     const std::string &def) {
  const std::string p = ari::resolve_bind(base, key);
  ari::read_or_seed<std::string>(a, p, def);
  return p;
}
std::string seed_bool(cvc::app &a, const std::string &base, const char *key, bool def) {
  const std::string p = ari::resolve_bind(base, key);
  ari::read_bool_or_seed(a, p, def ? 1 : 0);
  return p;
}
std::string prop_str(const ari::Value &props, const char *key, const char *def) {
  const std::string s = props.str(key);
  return s.empty() ? std::string(def) : s;
}

double ms_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
std::string fmt_ms(double ms) { return std::to_string(static_cast<long long>(std::lround(ms))); }

// A count or index read from state, where any program or prop can write "nan", "inf" or "1e30":
// NaN is the default and the rest is clamped in double BEFORE the conversion (converting an
// out-of-range double to an integer is undefined behaviour, and a saturated count would hang).
int to_int_clamped(double v, int lo, int hi, int def) {
  if (std::isnan(v))
    return def;
  return static_cast<int>(std::clamp(v, static_cast<double>(lo), static_cast<double>(hi)));
}
std::uint64_t to_count_clamped(double v, double lo, double hi, std::uint64_t def) {
  if (std::isnan(v))
    return def;
  return static_cast<std::uint64_t>(std::clamp(v, lo, hi));
}

std::size_t tri_count(const cvc::geometry &g) {
  return g.const_tris().size() + 2 * g.const_quads().size();
}

// Min/max of the finite values ((NaN, NaN) when there are none).
std::pair<double, double> finite_range(const std::vector<double> &v) {
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  for (double x : v)
    if (std::isfinite(x)) {
      lo = std::min(lo, x);
      hi = std::max(hi, x);
    }
  if (lo > hi)
    return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  return {lo, hi};
}

cvc::colormap_kind colormap_or_viridis(const std::string &name) {
  cvc::colormap_kind k = cvc::colormap_kind::VIRIDIS;
  cvc::colormap_from_string(name, k);
  return k;
}

// The node's `material:` block, as the built-in geometry node applies it.
void apply_material(GeometryNode &g, const ari::SceneNode &n) {
  if (!n.has_material)
    return;
  g.setUseSingleColor(n.use_single_color);
  g.setColor(n.color[0], n.color[1], n.color[2]);
  g.setAmbient(n.ambient);
  g.setDiffuse(n.diffuse);
  if (n.has_specular) {
    g.setSpecular(n.specular);
    g.setSpecularPower(n.specular_power);
  }
}

std::shared_ptr<GeometryNode> make_geometry_node(SceneGraph &sg, GraphicsNode *parent,
                                                 const std::string &id, const cvc::geometry &g) {
  return parent ? parent->createChild<GeometryNode>(id, g)
                : std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(id, g));
}

// Replace a node's mesh. setGeometry re-derives the render mode from the geometry type, so the
// mode the user picked (e.g. a wireframe) is put back unless `keep_mode` is false.
void swap_geometry(GeometryNode &node, const cvc::geometry &g, bool keep_mode = true) {
  const GeometryRenderMode mode = node.getRenderMode();
  node.setGeometry(g);
  if (keep_mode)
    node.setRenderMode(mode);
}

// Show a scalar field, or go back to the material colour. use_single_color wins over a field in
// GeometryNode, so it is switched off while a field shows and restored after.
void show_field(GeometryNode &node, const std::vector<double> &field, cvc::colormap_kind cm,
                double lo, double hi) {
  node.setUseSingleColor(false);
  node.setScalarField(field, cm, lo, hi);
}
void hide_field(GeometryNode &node, bool single_color) {
  if (node.hasScalarField())
    node.clearScalarField();
  node.setUseSingleColor(single_color);
}

// Run queued work on the render thread when the lane has no thread (single-threaded wasm).
void pump(cvc::async_lane &lane) {
  if (!lane.threaded())
    lane.run_deferred(1);
}

// --- mesh_lab ------------------------------------------------------------------------------------

bool is_curvature_mode(const std::string &m) {
  return m == "mean" || m == "gaussian" || m == "k1" || m == "k2";
}
bool is_mesh_op(const std::string &op) {
  return op == "smooth" || op == "decimate" || op == "repair" || op == "orient";
}

// Everything a job reads, copied out of state on the render thread before the submit.
struct MeshParams {
  cvc::smooth_params smooth;
  std::uint64_t target_faces = 5000;
  cvc::repair_params repair;
  std::string color_mode;
  cvc::index_t source = 0;
  std::shared_ptr<const cvc::geodesic_solver> solver; // cached for this mesh version, or null
};

// The lane -> tick handoff. Results are written before `done` is set and read after.
struct MeshJob {
  std::string op; // smooth | decimate | repair | orient | color
  std::uint64_t version = 0;
  std::atomic<bool> done{false};
  std::atomic<bool> cancel{false};
  std::shared_ptr<const cvc::geometry> geom; // the new mesh (mesh ops)
  std::vector<double> field;                 // the colour scalar (op == "color")
  double lo = 0, hi = 0;
  std::shared_ptr<const cvc::geodesic_solver> solver;
  std::string status;
};

struct MeshKeys {
  std::string request, busy, status, nv, nf;
  std::string smooth_method, smooth_iterations, smooth_lambda, smooth_fix_boundary;
  std::string target_faces, weld, remove_degenerate, remove_unreferenced, orient;
  std::string color_mode, colormap, color_min, color_max, geodesic_source;
};

struct MeshLabRT {
  cvc::app *app = nullptr;
  std::weak_ptr<GeometryNode> node;
  MeshKeys k;
  bool single_color = true; // the material's use_single_color, restored when no field shows
  std::shared_ptr<const cvc::geometry> source, current; // immutable snapshots
  std::uint64_t version = 0;                            // bumps on every mesh change
  std::string field_sig; // "<version>|<mode>[|<source>]" the shown colour was computed for
  std::string colormap;  // the colormap last applied
  std::shared_ptr<const cvc::geodesic_solver> solver;
  std::uint64_t solver_version = 0;
  std::shared_ptr<MeshJob> job;
  std::unique_ptr<cvc::async_lane> lane; // last: destroyed (drained, joined) before the rest

  ~MeshLabRT() {
    if (job)
      job->cancel = true;
  }
};

void run_mesh_job(MeshJob &job, const cvc::geometry &in, const MeshParams &p) {
  const auto t0 = std::chrono::steady_clock::now();
  try {
    if (job.cancel) {
      job.status = "cancelled";
    } else if (job.op == "color") {
      if (p.color_mode == "geodesic") {
        std::shared_ptr<const cvc::geodesic_solver> s = p.solver;
        if (!s)
          s = std::make_shared<cvc::geodesic_solver>(in);
        job.solver = s;
        job.field = s->distance({p.source});
        const std::pair<double, double> r = finite_range(job.field);
        job.lo = 0.0;
        job.hi = r.second;
      } else {
        const cvc::curvature_kind kind = p.color_mode == "gaussian" ? cvc::curvature_kind::GAUSSIAN
                                         : p.color_mode == "k1" ? cvc::curvature_kind::MAX_PRINCIPAL
                                         : p.color_mode == "k2" ? cvc::curvature_kind::MIN_PRINCIPAL
                                                                : cvc::curvature_kind::MEAN;
        job.field = cvc::vertex_curvature(in, kind);
        const std::pair<double, double> r = cvc::robust_range(job.field);
        job.lo = r.first;
        job.hi = r.second;
      }
      job.status = "coloured by " + p.color_mode + " (" + fmt_ms(ms_since(t0)) + " ms)";
    } else {
      auto g = std::make_shared<cvc::geometry>();
      if (job.op == "decimate") {
        cvc::simplify_params sp;
        sp.target_tris = p.target_faces;
        cvc::simplify_result sr;
        *g = cvc::simplify(in, sp, &sr);
        job.status = "decimated " + std::to_string(sr.in_tris) + " -> " +
                     std::to_string(sr.out_tris) + " faces";
      } else {
        g->copy(in, true);
        if (job.op == "smooth") {
          cvc::smooth(*g, p.smooth);
          job.status = "smoothed (" + std::to_string(p.smooth.iterations) + " iterations)";
        } else if (job.op == "repair") {
          const cvc::repair_report r = cvc::repair(*g, p.repair);
          job.status = "repaired: " + std::to_string(r.vertices_removed) + " vertices and " +
                       std::to_string(r.faces_removed) + " faces removed, " +
                       std::to_string(r.faces_flipped) + " faces flipped";
        } else {
          job.status = "oriented: " + std::to_string(cvc::orient_outward(*g)) + " faces flipped";
        }
      }
      job.geom = g;
      job.status += " (" + fmt_ms(ms_since(t0)) + " ms)";
    }
  } catch (const std::exception &e) {
    job.geom.reset();
    job.field.clear();
    job.status = std::string("error: ") + e.what();
  } catch (...) {
    job.geom.reset();
    job.field.clear();
    job.status = "error: unknown exception";
  }
  job.done.store(true, std::memory_order_release);
}

cvc::index_t geodesic_source(MeshLabRT &rt) {
  const std::size_t nv = rt.current->const_points().size();
  const double s = ari::read_or<double>(*rt.app, rt.k.geodesic_source, 0.0);
  if (nv == 0 || !(s > 0.0))
    return 0;
  return static_cast<cvc::index_t>(std::min(s, static_cast<double>(nv - 1)));
}

void launch_mesh_job(MeshLabRT &rt, const std::string &op) {
  cvc::app &a = *rt.app;
  MeshParams p;
  const std::string method = ari::read_or<std::string>(a, rt.k.smooth_method, "cotan");
  p.smooth.method = method == "uniform"  ? cvc::smooth_params::UNIFORM_LAPLACIAN
                    : method == "taubin" ? cvc::smooth_params::TAUBIN
                                         : cvc::smooth_params::COTAN_IMPLICIT;
  p.smooth.iterations =
      to_int_clamped(ari::read_or<double>(a, rt.k.smooth_iterations, 3.0), 1, 1000, 3);
  p.smooth.lambda = ari::read_or<double>(a, rt.k.smooth_lambda, 1e-3);
  p.smooth.fix_boundary = ari::read_bool_or(a, rt.k.smooth_fix_boundary, 0) != 0;
  p.target_faces =
      to_count_clamped(ari::read_or<double>(a, rt.k.target_faces, 5000.0), 1.0, 1e12, 5000);
  p.repair.weld_epsilon = ari::read_bool_or(a, rt.k.weld, 1) ? 0.0 : -1.0;
  p.repair.remove_degenerate = ari::read_bool_or(a, rt.k.remove_degenerate, 1) != 0;
  p.repair.remove_unreferenced = ari::read_bool_or(a, rt.k.remove_unreferenced, 1) != 0;
  p.repair.orient = ari::read_bool_or(a, rt.k.orient, 1) != 0;
  p.color_mode = ari::read_or<std::string>(a, rt.k.color_mode, "none");
  p.source = geodesic_source(rt);
  if (rt.solver && rt.solver_version == rt.version)
    p.solver = rt.solver;

  auto job = std::make_shared<MeshJob>();
  job->op = op;
  job->version = rt.version;
  rt.job = job;
  ari::write<int>(a, rt.k.busy, 1);
  if (op != "color")
    ari::write<std::string>(a, rt.k.status, op + "...");
  std::shared_ptr<const cvc::geometry> in = rt.current;
  if (!rt.lane->submit([job, in, p]() { run_mesh_job(*job, *in, p); })) {
    job->status = "error: the mesh_lab lane is stopped";
    job->done.store(true, std::memory_order_release);
  }
}

void publish_mesh_stats(MeshLabRT &rt) {
  ari::write<double>(*rt.app, rt.k.nv, static_cast<double>(rt.current->const_points().size()));
  ari::write<double>(*rt.app, rt.k.nf, static_cast<double>(tri_count(*rt.current)));
}

void set_mesh(MeshLabRT &rt, GeometryNode &node, std::shared_ptr<const cvc::geometry> g) {
  rt.current = std::move(g);
  ++rt.version;
  hide_field(node, rt.single_color); // the colour follower recomputes it for the new mesh
  swap_geometry(node, *rt.current);
  publish_mesh_stats(rt);
}

void apply_mesh_job(MeshLabRT &rt, GeometryNode &node, MeshJob &j) {
  cvc::app &a = *rt.app;
  if (j.op == "color") {
    if (j.solver) {
      rt.solver = j.solver;
      rt.solver_version = j.version;
    }
    if (!j.field.empty()) {
      rt.colormap = ari::read_or<std::string>(a, rt.k.colormap, "viridis");
      show_field(node, j.field, colormap_or_viridis(rt.colormap), j.lo, j.hi);
      ari::write<double>(a, rt.k.color_min, j.lo);
      ari::write<double>(a, rt.k.color_max, j.hi);
    }
  } else if (j.geom) {
    set_mesh(rt, node, j.geom);
  }
  ari::write<std::string>(a, rt.k.status, j.status);
  ari::write<int>(a, rt.k.busy, 0);
}

// Follow color.mode / color.colormap / geodesic.source: a changed signature recomputes the scalar
// off-thread; a colormap change alone recolours in place.
void follow_mesh_color(MeshLabRT &rt, GeometryNode &node) {
  cvc::app &a = *rt.app;
  const std::string mode = ari::read_or<std::string>(a, rt.k.color_mode, "none");
  std::string sig = std::to_string(rt.version) + "|" + mode;
  if (mode == "geodesic")
    sig += "|" + std::to_string(geodesic_source(rt));
  if (sig != rt.field_sig) {
    rt.field_sig = sig;
    if (is_curvature_mode(mode) || mode == "geodesic") {
      launch_mesh_job(rt, "color");
      return;
    }
    if (!mode.empty() && mode != "none")
      ari::write<std::string>(a, rt.k.status, "unknown color.mode '" + mode + "'");
    hide_field(node, rt.single_color);
    ari::write<std::string>(a, rt.k.color_min, "-");
    ari::write<std::string>(a, rt.k.color_max, "-");
  }
  const std::string cm = ari::read_or<std::string>(a, rt.k.colormap, "viridis");
  if (cm != rt.colormap) {
    rt.colormap = cm;
    if (node.hasScalarField())
      node.setColorMap(colormap_or_viridis(cm));
  }
}

void tick_mesh_lab(MeshLabRT &rt) {
  std::shared_ptr<GeometryNode> node = rt.node.lock();
  if (!node)
    return;
  cvc::app &a = *rt.app;
  pump(*rt.lane);
  if (rt.job) {
    if (!rt.job->done.load(std::memory_order_acquire))
      return; // one job at a time; a request waits in its key
    std::shared_ptr<MeshJob> j = std::move(rt.job);
    apply_mesh_job(rt, *node, *j);
  }

  const std::string req = ari::read_or<std::string>(a, rt.k.request, "");
  if (!req.empty()) {
    ari::write<std::string>(a, rt.k.request, std::string());
    if (is_mesh_op(req)) {
      launch_mesh_job(rt, req);
      return;
    }
    if (req == "reset") {
      set_mesh(rt, *node, rt.source);
      ari::write<std::string>(a, rt.k.status, "reset to the source mesh");
    } else if (req == "curvature") {
      // Colour by curvature now: switch a non-curvature mode to mean, or recompute this one.
      if (is_curvature_mode(ari::read_or<std::string>(a, rt.k.color_mode, "none")))
        rt.field_sig.clear();
      else
        ari::write<std::string>(a, rt.k.color_mode, "mean");
    } else {
      ari::write<std::string>(a, rt.k.status, "unknown request '" + req + "'");
    }
  }
  follow_mesh_color(rt, *node);
}

// type: mesh_lab
std::shared_ptr<GraphicsNode> realize_mesh_lab(SceneGraph &sg, const ari::SceneNode &n,
                                               GraphicsNode *parent, RealizedScene &out,
                                               std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: mesh_lab '" + n.id + "': " + m);
  };
  cvc::geometry geom;
  if (!read_node_geometry(sg, n, geom, warnings))
    return nullptr;
  if (tri_count(geom) == 0) {
    warn("the source has no triangles");
    return nullptr;
  }
  auto rt = std::make_shared<MeshLabRT>();
  try {
    rt->lane = std::make_unique<cvc::async_lane>("mesh_lab." + n.id);
  } catch (const std::exception &e) {
    warn(std::string("could not start its worker lane: ") + e.what());
    return nullptr;
  }
  std::shared_ptr<GeometryNode> g = make_geometry_node(sg, parent, n.id, geom);
  if (!g)
    return nullptr;
  apply_material(*g, n);
  rt->app = &sg.appContext();
  rt->node = g;
  rt->single_color = g->getUseSingleColor();
  auto snapshot = std::make_shared<cvc::geometry>();
  snapshot->copy(geom, true);
  rt->source = rt->current = snapshot;

  cvc::app &a = *rt->app;
  const std::string base = g->stateName("mesh_ops");
  const ari::Value &pr = n.props;
  MeshKeys &k = rt->k;
  k.request = seed_str(a, base, "request", "");
  k.busy = ari::resolve_bind(base, "busy");
  k.status = ari::resolve_bind(base, "status");
  k.nv = ari::resolve_bind(base, "stats.vertices");
  k.nf = ari::resolve_bind(base, "stats.faces");
  k.smooth_method = seed_str(a, base, "smooth.method", prop_str(pr, "smooth_method", "cotan"));
  k.smooth_iterations = seed_num(a, base, "smooth.iterations", pr.num("smooth_iterations", 3));
  k.smooth_lambda = seed_num(a, base, "smooth.lambda", pr.num("smooth_lambda", 1e-3));
  k.smooth_fix_boundary = seed_bool(a, base, "smooth.fix_boundary", pr.flag("fix_boundary"));
  k.target_faces = seed_num(a, base, "decimate.target_faces", pr.num("target_faces", 5000));
  k.weld = seed_bool(a, base, "repair.weld", pr.flag("weld", true));
  k.remove_degenerate =
      seed_bool(a, base, "repair.remove_degenerate", pr.flag("remove_degenerate", true));
  k.remove_unreferenced =
      seed_bool(a, base, "repair.remove_unreferenced", pr.flag("remove_unreferenced", true));
  k.orient = seed_bool(a, base, "repair.orient", pr.flag("orient", true));
  k.color_mode = seed_str(a, base, "color.mode", prop_str(pr, "color_mode", "none"));
  k.colormap = seed_str(a, base, "color.colormap", prop_str(pr, "colormap", "viridis"));
  k.color_min = ari::resolve_bind(base, "color.min");
  k.color_max = ari::resolve_bind(base, "color.max");
  k.geodesic_source = seed_num(a, base, "geodesic.source", pr.num("geodesic_source", 0));
  ari::write<int>(a, k.busy, 0);
  ari::write<std::string>(a, k.status, "idle");
  ari::write<std::string>(a, k.color_min, "-");
  ari::write<std::string>(a, k.color_max, "-");
  rt->colormap = ari::read_or<std::string>(a, k.colormap, "viridis");
  publish_mesh_stats(*rt);

  out.custom_ticks.push_back([rt](vtkRenderer *) { tick_mesh_lab(*rt); });
  return g;
}

// --- fe_lab --------------------------------------------------------------------------------------

struct FeParams {
  double isovalue = 0.0;
  bool inside_below = true; // mesh {v < isovalue} (an SDF, negative inside) vs {v > isovalue}
  int improve = 0;
  int improve_iterations = 1;
  std::string problem = "poisson";
  double f = 1.0;
  cvc::fem::heat_params heat;
};

struct FeJob {
  std::string op;         // mesh | solve
  bool stoppable = false; // a heat solve: the only job that polls `cancel` mid-run
  std::string running;    // the status shown while it runs ("meshing...", "heat...")
  std::atomic<bool> done{false};
  std::atomic<bool> cancel{false};
  std::atomic<int> progress{0}; // percent
  std::shared_ptr<const cvc::geometry> tetmesh, surface;
  std::vector<double> field;
  std::string status;
  // Heat solves publish intermediate fields here (lane writes, tick takes).
  std::mutex snap_mtx;
  std::vector<double> snapshot;
  bool fresh = false;
};

struct SliceJob {
  std::atomic<bool> done{false};
  bool colored = false;
  std::shared_ptr<const cvc::geometry> slice;
};

struct FeKeys {
  std::string request, busy, stoppable, status, progress, tets, nv, field_min, field_max;
  std::string isovalue, inside, improve, improve_iterations;
  std::string problem, poisson_f, heat_kappa, heat_dt, heat_steps;
  std::string colormap, show_surface, show_slice, slice_axis, slice_offset, surface_opacity;
};

struct FeLabRT {
  cvc::app *app = nullptr;
  std::weak_ptr<GeometryNode> node, slice_node;
  FeKeys k;
  bool single_color = true;
  bool auto_solve = false;
  cvc::volume volume;
  cvc::geometry placeholder; // the volume's bounds as lines, shown until the first tet mesh
  std::shared_ptr<const cvc::geometry> tetmesh, surface;
  double lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0}; // tet mesh bounds
  std::vector<double> field;
  double field_lo = 0, field_hi = 0;
  std::uint64_t mesh_version = 0, field_version = 0;
  std::string surface_sig, color_sig, slice_sig, slice_cm;
  double opacity = -1.0;
  std::shared_ptr<FeJob> job;
  std::shared_ptr<SliceJob> slice_job;
  std::unique_ptr<cvc::async_lane> lane;       // tet meshing + solves
  std::unique_ptr<cvc::async_lane> slice_lane; // cuts, so they follow a long solve

  explicit FeLabRT(cvc::app &a) : app(&a), volume(a) {}
  ~FeLabRT() {
    if (job)
      job->cancel = true;
  }
};

// The twelve edges of a box, as lines.
cvc::geometry box_lines(const cvc::bounding_box &b) {
  cvc::geometry g;
  for (int i = 0; i < 8; ++i)
    g.points().push_back(
        {(i & 1) ? b.maxx : b.minx, (i & 2) ? b.maxy : b.miny, (i & 4) ? b.maxz : b.minz});
  const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                        {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto &l : e)
    g.lines().push_back({static_cast<cvc::index_t>(l[0]), static_cast<cvc::index_t>(l[1])});
  return g;
}

#ifdef CVC_ENABLE_MESHER
// -v as a Float volume on the same grid: LBIE meshes the side ABOVE its isovalue.
cvc::volume negated(const cvc::volume &v) {
  const std::uint64_t n = v.XDim() * v.YDim() * v.ZDim();
  std::vector<float> buf(n);
  for (std::uint64_t i = 0; i < n; ++i)
    buf[i] = static_cast<float>(-v(i));
  return cvc::volume(v.ctx(), reinterpret_cast<const unsigned char *>(buf.data()),
                     v.voxel_dimensions(), cvc::Float, v.boundingBox());
}
#endif

void run_fe_mesh(FeJob &job, const cvc::volume &vol, const FeParams &p) {
#ifdef CVC_ENABLE_MESHER
  // LBIE meshes {v > isovalue}. For the inside of an SDF (negative inside) mesh -v above
  // -isovalue, so mesh.isovalue stays in the volume's own units either way.
  const cvc::volume v = p.inside_below ? negated(vol) : vol;
  const double iso = p.inside_below ? -p.isovalue : p.isovalue;
  auto tm = std::make_shared<cvc::geometry>(
      cvc::tetrahedralize(v, iso, cvc::DUALLIB, static_cast<cvc::improvement_method>(p.improve),
                          cvc::BSPLINE_CONVOLUTION, p.improve_iterations));
  if (tm->const_tets().empty())
    throw std::runtime_error("no tets: the isovalue does not cut the volume");
  cvc::orient_tets(*tm);
  auto surf = std::make_shared<cvc::geometry>(cvc::tet_boundary_surface(*tm));
  cvc::compute_vertex_normals(*surf); // LBIE's gradient normals point the wrong way on -v
  job.status = "meshed: " + std::to_string(tm->const_tets().size()) + " tets";
  job.tetmesh = tm;
  job.surface = surf;
#else
  (void)vol;
  (void)p;
  job.status = "mesher unavailable";
#endif
}

void run_fe_solve(FeJob &job, const cvc::geometry &tm, const FeParams &p) {
  const cvc::fem::domain vol = cvc::fem::domain::VOLUME;
  const std::vector<cvc::index_t> bnd = cvc::fem::boundary_vertices(tm, vol);
  const cvc::geometry::points_t &P = tm.const_points();
  if (p.problem == "heat") {
    // A hot floor: boundary vertices in the lowest 5% of the height held at 1, the rest of the
    // boundary insulated, the body starting at 0.
    double zlo = std::numeric_limits<double>::infinity(), zhi = -zlo;
    for (cvc::index_t v : bnd) {
      zlo = std::min(zlo, P[v][2]);
      zhi = std::max(zhi, P[v][2]);
    }
    cvc::fem::dirichlet bc;
    for (cvc::index_t v : bnd)
      if (P[v][2] <= zlo + 0.05 * (zhi - zlo))
        bc.vertices.push_back(v);
    bc.values.push_back(1.0);
    if (bc.vertices.empty())
      throw std::runtime_error("no boundary vertices to heat");
    const int steps = std::max(1, p.heat.steps);
    const int every = std::max(1, steps / 20);
    int last = 0;
    const cvc::fem::heat_progress progress = [&job, &last, steps,
                                              every](int s, const std::vector<double> &u) {
      last = s;
      job.progress = static_cast<int>(100.0 * s / steps);
      if (s % every == 0 && s < steps) {
        std::lock_guard<std::mutex> lock(job.snap_mtx);
        job.snapshot = u;
        job.fresh = true;
      }
      return !job.cancel.load();
    };
    job.field =
        cvc::fem::solve_heat(tm, std::vector<double>(P.size(), 0.0), p.heat, bc, vol, progress);
    job.status = job.cancel ? "heat: stopped at step " + std::to_string(last)
                            : "heat: " + std::to_string(steps) + " steps";
  } else {
    cvc::fem::dirichlet bc;
    bc.vertices = bnd;
    bc.values.push_back(0.0);
    job.field = cvc::fem::solve_poisson(tm, {p.f}, bc, vol);
    job.status = "poisson: solved";
  }
}

void run_fe_job(FeJob &job, const cvc::volume &vol, std::shared_ptr<const cvc::geometry> tm,
                const FeParams &p) {
  const auto t0 = std::chrono::steady_clock::now();
  try {
    if (job.cancel)
      job.status = "cancelled";
    else if (job.op == "mesh")
      run_fe_mesh(job, vol, p);
    else
      run_fe_solve(job, *tm, p);
    if (!job.cancel)
      job.status += " (" + fmt_ms(ms_since(t0)) + " ms)";
  } catch (const std::exception &e) {
    job.tetmesh.reset();
    job.field.clear();
    job.status = std::string("error: ") + e.what();
  } catch (...) {
    job.tetmesh.reset();
    job.field.clear();
    job.status = "error: unknown exception";
  }
  job.progress = 100;
  job.done.store(true, std::memory_order_release);
}

void launch_fe_job(FeLabRT &rt, const std::string &op) {
  cvc::app &a = *rt.app;
  FeParams p;
  p.isovalue = ari::read_or<double>(a, rt.k.isovalue, 0.0);
  p.inside_below = ari::read_or<std::string>(a, rt.k.inside, "below") != "above";
  p.improve = to_int_clamped(ari::read_or<double>(a, rt.k.improve, 0), 0, 5, 0);
  p.improve_iterations =
      to_int_clamped(ari::read_or<double>(a, rt.k.improve_iterations, 1), 0, 100, 1);
  p.problem = ari::read_or<std::string>(a, rt.k.problem, "poisson");
  p.f = ari::read_or<double>(a, rt.k.poisson_f, 1.0);
  // heat.kappa is relative to the domain: the diffusivity used is kappa * extent^2, so the same
  // settings spread heat across the whole body whatever the model's scale.
  const double ext = std::max({rt.hi[0] - rt.lo[0], rt.hi[1] - rt.lo[1], rt.hi[2] - rt.lo[2]});
  p.heat.kappa = ari::read_or<double>(a, rt.k.heat_kappa, 1.0) * ext * ext;
  p.heat.dt = ari::read_or<double>(a, rt.k.heat_dt, 0.01);
  p.heat.steps = to_int_clamped(ari::read_or<double>(a, rt.k.heat_steps, 100), 1, 1000000, 100);

  auto job = std::make_shared<FeJob>();
  job->op = op;
  // Only a heat solve polls the cancel flag (its progress callback); tet meshing and a Poisson
  // solve run to the end, so `stop` is refused for them rather than claimed.
  job->stoppable = op == "solve" && p.problem == "heat";
  job->running = op == "mesh" ? "meshing..." : p.problem + "...";
  rt.job = job;
  ari::write<int>(a, rt.k.busy, 1);
  ari::write<int>(a, rt.k.stoppable, job->stoppable ? 1 : 0);
  ari::write<int>(a, rt.k.progress, 0);
  ari::write<std::string>(a, rt.k.status, job->running);
  const cvc::volume vol = rt.volume;
  std::shared_ptr<const cvc::geometry> tm = rt.tetmesh;
  if (!rt.lane->submit([job, vol, tm, p]() { run_fe_job(*job, vol, tm, p); })) {
    job->status = "error: the fe_lab lane is stopped";
    job->done.store(true, std::memory_order_release);
  }
}

void set_field(FeLabRT &rt, std::vector<double> field) {
  rt.field = std::move(field);
  const std::pair<double, double> r = finite_range(rt.field);
  rt.field_lo = r.first;
  rt.field_hi = r.second;
  ++rt.field_version;
  ari::write<double>(*rt.app, rt.k.field_min, rt.field_lo);
  ari::write<double>(*rt.app, rt.k.field_max, rt.field_hi);
}

void apply_fe_job(FeLabRT &rt, FeJob &j) {
  cvc::app &a = *rt.app;
  if (j.op == "mesh" && j.tetmesh) {
    rt.tetmesh = j.tetmesh;
    rt.surface = j.surface;
    for (int d = 0; d < 3; ++d) {
      rt.lo[d] = std::numeric_limits<double>::infinity();
      rt.hi[d] = -rt.lo[d];
    }
    for (const cvc::geometry::point_t &p : rt.surface->const_points())
      for (int d = 0; d < 3; ++d) {
        rt.lo[d] = std::min(rt.lo[d], p[d]);
        rt.hi[d] = std::max(rt.hi[d], p[d]);
      }
    ++rt.mesh_version;
    rt.field.clear();
    ++rt.field_version;
    ari::write<double>(a, rt.k.tets, static_cast<double>(rt.tetmesh->const_tets().size()));
    ari::write<double>(a, rt.k.nv, static_cast<double>(rt.tetmesh->const_points().size()));
    ari::write<std::string>(a, rt.k.field_min, "-");
    ari::write<std::string>(a, rt.k.field_max, "-");
  } else if (j.op == "solve" && !j.field.empty()) {
    set_field(rt, std::move(j.field));
  }
  ari::write<std::string>(a, rt.k.status, j.status);
  ari::write<int>(a, rt.k.progress, 100);
  ari::write<int>(a, rt.k.stoppable, 0);
  ari::write<int>(a, rt.k.busy, 0);
  // auto_solve queues a solve only when nothing is waiting: a request written while the mesh job
  // ran (a re-mesh after an isovalue edit, a solve) runs next instead -- a click is never lost. A
  // `stop` never waits (the tick consumes it at once), and a waiting mesh auto-solves in turn.
  if (j.op == "mesh" && j.tetmesh && rt.auto_solve &&
      ari::read_or<std::string>(a, rt.k.request, "").empty())
    ari::write<std::string>(a, rt.k.request, "solve");
}

void run_slice_job(SliceJob &job, const cvc::geometry &tm, const std::vector<double> &field,
                   const cvc::geometry::vector_t &normal, double offset) {
  try {
    cvc::geometry t(tm);
    t.functions() = field; // interpolated onto the cut (an empty field clears it)
    auto s = std::make_shared<cvc::geometry>(cvc::slice_tets(t, normal, offset));
    cvc::compute_vertex_normals(*s);
    job.slice = s;
  } catch (...) {
    job.slice.reset();
  }
  job.done.store(true, std::memory_order_release);
}

// Keep the surface, the cut and their colours in step with the view keys and the latest field.
void follow_fe_view(FeLabRT &rt, GeometryNode &node, GeometryNode *slice) {
  cvc::app &a = *rt.app;
  const bool show_surface = ari::read_bool_or(a, rt.k.show_surface, 1) != 0;
  const bool show_slice = ari::read_bool_or(a, rt.k.show_slice, 1) != 0;
  const std::string cm_name = ari::read_or<std::string>(a, rt.k.colormap, "viridis");
  const cvc::colormap_kind cm = colormap_or_viridis(cm_name);

  // The boundary surface (or the bounds placeholder before the first mesh).
  const std::string ssig = std::to_string(rt.mesh_version) + "|" + (show_surface ? "1" : "0");
  if (ssig != rt.surface_sig) {
    rt.surface_sig = ssig;
    rt.color_sig.clear();
    hide_field(node, rt.single_color);
    if (!rt.surface) {
      swap_geometry(node, rt.placeholder, false);
      node.setRenderMode(GeometryRenderMode::LINES);
    } else {
      swap_geometry(node, show_surface ? *rt.surface : cvc::geometry(), false);
    }
  }
  const std::string csig = std::to_string(rt.field_version) + "|" + cm_name;
  if (rt.surface && show_surface && csig != rt.color_sig) {
    rt.color_sig = csig;
    if (rt.field.size() == rt.surface->const_points().size())
      show_field(node, rt.field, cm, rt.field_lo, rt.field_hi);
    else
      hide_field(node, rt.single_color);
  }
  // A visible cut needs to be seen through the surface.
  double opacity =
      (show_slice && rt.surface) ? ari::read_or<double>(a, rt.k.surface_opacity, 0.35) : 1.0;
  opacity = std::isnan(opacity) ? 0.35 : std::max(0.0, std::min(1.0, opacity));
  if (opacity != rt.opacity) {
    rt.opacity = opacity;
    node.setOpacity(opacity);
  }

  if (!slice)
    return;
  // The cut is a child node, and the document can bind only the fe_lab node's own visibility, so
  // fold that in: a hidden fe_lab hides its cut too. The key write is a no-op when unchanged, and
  // SceneNode::propagateVisible flips the child's flag without touching its key (showing the
  // fe_lab again forces the child on), so reconcile the flag itself as well.
  const bool want_slice = show_slice && rt.surface && node.isVisibleInHierarchy();
  ari::write<int>(a, slice->stateName("visible"), want_slice ? 1 : 0);
  if (slice->isVisible() != want_slice)
    slice->setVisible(want_slice);
  if (rt.slice_job && rt.slice_job->done.load(std::memory_order_acquire)) {
    std::shared_ptr<SliceJob> sj = std::move(rt.slice_job);
    hide_field(*slice, rt.single_color);
    swap_geometry(*slice, sj->slice ? *sj->slice : cvc::geometry(), false);
    if (sj->slice && sj->colored &&
        sj->slice->const_functions().size() == sj->slice->const_points().size()) {
      show_field(*slice, sj->slice->const_functions(), cm, rt.field_lo, rt.field_hi);
      rt.slice_cm = cm_name;
    }
  }
  if (cm_name != rt.slice_cm && slice->hasScalarField()) {
    rt.slice_cm = cm_name;
    slice->setColorMap(cm);
  }
  if (!rt.tetmesh || !show_slice || rt.slice_job)
    return;
  const int axis = to_int_clamped(ari::read_or<double>(a, rt.k.slice_axis, 0), 0, 2, 0);
  const double s = std::max(-1.0, std::min(1.0, ari::read_or<double>(a, rt.k.slice_offset, 0.0)));
  const std::string sig = std::to_string(rt.mesh_version) + "|" + std::to_string(rt.field_version) +
                          "|" + std::to_string(axis) + "|" + std::to_string(s);
  if (sig == rt.slice_sig)
    return;
  rt.slice_sig = sig;
  cvc::geometry::vector_t normal = {{0.0, 0.0, 0.0}};
  normal[axis] = 1.0;
  const double offset = 0.5 * (rt.lo[axis] + rt.hi[axis]) + s * 0.5 * (rt.hi[axis] - rt.lo[axis]);
  auto sj = std::make_shared<SliceJob>();
  sj->colored = rt.field.size() == rt.tetmesh->const_points().size();
  rt.slice_job = sj;
  std::shared_ptr<const cvc::geometry> tm = rt.tetmesh;
  const std::vector<double> field = sj->colored ? rt.field : std::vector<double>();
  if (!rt.slice_lane->submit(
          [sj, tm, field, normal, offset]() { run_slice_job(*sj, *tm, field, normal, offset); }))
    sj->done.store(true, std::memory_order_release);
}

void tick_fe_lab(FeLabRT &rt) {
  std::shared_ptr<GeometryNode> node = rt.node.lock();
  if (!node)
    return;
  std::shared_ptr<GeometryNode> slice = rt.slice_node.lock();
  cvc::app &a = *rt.app;
  pump(*rt.lane);
  pump(*rt.slice_lane);

  const std::string req = ari::read_or<std::string>(a, rt.k.request, "");
  if (rt.job) {
    FeJob &j = *rt.job;
    if (req == "stop") {
      ari::write<std::string>(a, rt.k.request, std::string());
      if (j.stoppable) {
        j.cancel = true; // the heat solve's progress callback sees it at its next step
        ari::write<std::string>(a, rt.k.status, "stopping...");
      } else {
        // Tet meshing and a Poisson solve cannot be interrupted: say so, and that it still runs.
        ari::write<std::string>(a, rt.k.status, j.running + " (only a heat solve can be stopped)");
      }
    }
    {
      std::vector<double> snap;
      {
        std::lock_guard<std::mutex> lock(j.snap_mtx);
        if (j.fresh) {
          snap.swap(j.snapshot);
          j.fresh = false;
        }
      }
      if (!snap.empty())
        set_field(rt, std::move(snap));
    }
    ari::write<int>(a, rt.k.progress, j.progress.load());
    if (j.done.load(std::memory_order_acquire)) {
      std::shared_ptr<FeJob> done = std::move(rt.job);
      apply_fe_job(rt, *done);
    }
  } else if (!req.empty()) {
    ari::write<std::string>(a, rt.k.request, std::string());
    if (req == "mesh") {
#ifdef CVC_ENABLE_MESHER
      launch_fe_job(rt, "mesh");
#else
      ari::write<std::string>(a, rt.k.status, "mesher unavailable");
#endif
    } else if (req == "solve") {
      if (rt.tetmesh)
        launch_fe_job(rt, "solve");
      else
        ari::write<std::string>(a, rt.k.status, "tet-mesh the volume first");
    } else if (req == "stop") {
      ari::write<std::string>(a, rt.k.status, "nothing to stop");
    } else {
      ari::write<std::string>(a, rt.k.status, "unknown request '" + req + "'");
    }
  }
  follow_fe_view(rt, *node, slice.get());
}

// type: fe_lab
std::shared_ptr<GraphicsNode> realize_fe_lab(SceneGraph &sg, const ari::SceneNode &n,
                                             GraphicsNode *parent, RealizedScene &out,
                                             std::vector<std::string> *warnings) {
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: fe_lab '" + n.id + "': " + m);
  };
  if (n.source_file.empty() && n.source_sdf_mesh.empty()) {
    warn("has no source (a volume file or { sdf: { mesh, dim } })");
    return nullptr;
  }
  auto rt = std::make_shared<FeLabRT>(sg.appContext());
  try {
    rt->volume = load_node_volume(sg, n, warnings);
    rt->lane = std::make_unique<cvc::async_lane>("fe_lab." + n.id);
    rt->slice_lane = std::make_unique<cvc::async_lane>("fe_lab." + n.id + ".slice");
  } catch (const std::exception &e) {
    warn(e.what());
    return nullptr;
  }
  rt->placeholder = box_lines(rt->volume.boundingBox());
  std::shared_ptr<GeometryNode> g = make_geometry_node(sg, parent, n.id, rt->placeholder);
  if (!g)
    return nullptr;
  g->setRenderMode(GeometryRenderMode::LINES);
  apply_material(*g, n);
  std::shared_ptr<GeometryNode> s = g->createChild<GeometryNode>(n.id + "_slice", cvc::geometry());
  if (s)
    apply_material(*s, n);
  rt->node = g;
  rt->slice_node = s;
  rt->single_color = g->getUseSingleColor();
  rt->auto_solve = n.props.flag("auto_solve");
  rt->surface_sig = "0|1"; // the placeholder is already up

  cvc::app &a = *rt->app;
  const std::string base = g->stateName("fe");
  const ari::Value &pr = n.props;
  FeKeys &k = rt->k;
  k.request = seed_str(a, base, "request", pr.flag("auto_mesh") ? "mesh" : "");
  k.busy = ari::resolve_bind(base, "busy");
  k.stoppable = ari::resolve_bind(base, "stoppable");
  k.status = ari::resolve_bind(base, "status");
  k.progress = ari::resolve_bind(base, "progress");
  k.tets = ari::resolve_bind(base, "stats.tets");
  k.nv = ari::resolve_bind(base, "stats.vertices");
  k.field_min = ari::resolve_bind(base, "field.min");
  k.field_max = ari::resolve_bind(base, "field.max");
  k.isovalue = seed_num(a, base, "mesh.isovalue", pr.num("isovalue", 0.0));
  k.inside = seed_str(a, base, "mesh.inside",
                      prop_str(pr, "inside", n.source_sdf_mesh.empty() ? "above" : "below"));
  k.improve = seed_num(a, base, "mesh.improve", pr.num("improve", 0));
  k.improve_iterations =
      seed_num(a, base, "mesh.improve_iterations", pr.num("improve_iterations", 1));
  k.problem = seed_str(a, base, "problem", prop_str(pr, "problem", "poisson"));
  k.poisson_f = seed_num(a, base, "poisson.f", pr.num("poisson_f", 1.0));
  k.heat_kappa = seed_num(a, base, "heat.kappa", pr.num("heat_kappa", 1.0));
  k.heat_dt = seed_num(a, base, "heat.dt", pr.num("heat_dt", 0.01));
  k.heat_steps = seed_num(a, base, "heat.steps", pr.num("heat_steps", 100));
  k.colormap = seed_str(a, base, "view.colormap", prop_str(pr, "colormap", "viridis"));
  k.show_surface = seed_bool(a, base, "view.show_surface", pr.flag("show_surface", true));
  k.show_slice = seed_bool(a, base, "view.show_slice", pr.flag("show_slice", true));
  k.slice_axis = seed_num(a, base, "view.slice_axis", pr.num("slice_axis", 0));
  k.slice_offset = seed_num(a, base, "view.slice_offset", pr.num("slice_offset", 0.0));
  k.surface_opacity = seed_num(a, base, "view.surface_opacity", pr.num("surface_opacity", 0.35));
  ari::write<int>(a, k.busy, 0);
  ari::write<int>(a, k.stoppable, 0);
  ari::write<int>(a, k.progress, 0);
  ari::write<double>(a, k.tets, 0.0);
  ari::write<double>(a, k.nv, 0.0);
  ari::write<std::string>(a, k.status, "volume ready");
  ari::write<std::string>(a, k.field_min, "-");
  ari::write<std::string>(a, k.field_max, "-");

  out.custom_ticks.push_back([rt](vtkRenderer *) { tick_fe_lab(*rt); });
  return g;
}

} // namespace
#endif // CVC_ENABLE_LIBIGL

void register_mesh_node_types() {
#ifdef CVC_ENABLE_LIBIGL
  register_scene_node_type("mesh_lab", realize_mesh_lab);
  register_scene_node_type("fe_lab", realize_fe_lab);
#endif
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
