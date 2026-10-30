// Prove cvc::gl::ariadne::realize_scene turns a backend-neutral Scene spec into the
// right live cvc::gl::SceneGraph, at the level the loader gtests can't reach (they
// only check parsing — no VTK). Runs offscreen, inspecting the realized vtkLights and
// node world transforms. Covers the §9 realize invariants an adversarial review
// flagged as untested:
//   * the light BATCH bakes a light's moved world position (not the origin),
//   * directional -> non-positional (az/el) vs spot -> positional (pos/target),
//   * a StageLighting rig actually adds lights,
//   * a nested node's transform is LOCAL (world = parent ∘ local).
#include <algorithm>
#include <chrono> // mesh_lab test: bounded waits on the node's worker lane
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h> // field colours: pump the scheduler their overlay tasks run on
#include <cvc/core/world_clock.h> // clock test: app.world_clock() drive via tick_scene
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/geometry/mesh_ops.h> // field colours: the colormap a seeded ramp follows
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/LodGraphicsNode.h> // LOD test: RealizeOptions realizes mesh nodes as ladders
#include <cvc/gl/LodPyramidCache.h>
#include <cvc/gl/LowMemoryPolyDataMapper.h> // clip test: the low-memory mapper honours no planes
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/lsystem_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/image/image.h>
#include <cvc/state/state.h> // clock test: read/write the bound sim.* keys
#include <cvc/state/state_exec/async_scheduler.h> // fe_lab: its jobs are scheduler processes
#include <cvc/tf/model.h>    // mesh_lab / fe_lab field colours
#include <cvc/tf/overlays.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <filesystem> // mesh_lab / fe_lab sources: a temp "document directory"
#include <thread>
#include <vtkActor.h> // LOD test: a rung's material
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkPlane.h> // clip test: the planes handed to a node's renderer
#include <vtkPlaneCollection.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>

using cvc::ariadne::Scene;
using cvc::ariadne::SceneClipPlane;
using cvc::ariadne::SceneHeightColorBand;
using cvc::ariadne::SceneHeightLayer;
using cvc::ariadne::SceneIsosurface;
using cvc::ariadne::SceneLight;
using cvc::ariadne::SceneNode;
using cvc::ariadne::SceneTFPoint;
using cvc::ariadne::Value;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

// Observables for the custom-node-type test — namespace scope so the registered
// realizer (process-global) captures nothing (no dangling into a test local).
static int g_custom_ticks = 0;
static double g_custom_radius = -1.0;

static int fails = 0;
static void chk(bool ok, const std::string &w) {
  printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok)
    ++fails;
}
static bool approx(double a, double b) { return std::fabs(a - b) < 1e-3; }

// Walk the renderer's lights, counting positional vs directional and capturing the
// (single) positional light's position.
struct LightSurvey {
  int positional = 0;
  int directional = 0;
  double posLightPos[3] = {0, 0, 0};
};
static LightSurvey survey(vtkRenderer *r) {
  LightSurvey s;
  vtkLightCollection *lc = r->GetLights();
  lc->InitTraversal();
  while (vtkLight *l = lc->GetNextItem()) {
    if (l->GetPositional()) {
      ++s.positional;
      const double *p = l->GetPosition();
      s.posLightPos[0] = p[0];
      s.posLightPos[1] = p[1];
      s.posLightPos[2] = p[2];
    } else {
      ++s.directional;
    }
  }
  return s;
}

#ifdef CVC_ENABLE_LIBIGL
// ── mesh_lab / fe_lab: switching sources at run time, eigenmodes, interval and hex meshes, the
// elements view. The data sits in a temp directory the nodes treat as the document's
// (SceneNode::base_dir), named by relative paths.
static void test_lab_sources(cvc::app &app) {
  namespace fs = std::filesystem;
  using cvc::gl::GeometryNode;
  printf("== mesh_lab / fe_lab: sources, modes, intervals, elements ==\n");
  std::error_code ec;
  const fs::path dir = fs::temp_directory_path(ec) / "cvcgl_ari_lab_sources";
  fs::create_directories(dir, ec);
  // An ellipsoid twice as tall along y as it is wide (a lat-long mesh), and a density ball.
  {
    cvc::geometry g;
    const int nu = 24, nv = 12;
    g.points().push_back({0.0, 2.0, 0.0});
    for (int v = 1; v < nv; ++v)
      for (int u = 0; u < nu; ++u) {
        const double th = 3.14159265358979 * v / nv, ph = 2.0 * 3.14159265358979 * u / nu;
        g.points().push_back(
            {std::sin(th) * std::cos(ph), 2.0 * std::cos(th), std::sin(th) * std::sin(ph)});
      }
    g.points().push_back({0.0, -2.0, 0.0});
    const auto ring = [&](int v, int u) {
      return static_cast<cvc::index_t>(1 + (v - 1) * nu + ((u + nu) % nu));
    };
    const auto bottom = static_cast<cvc::index_t>(g.points().size() - 1);
    for (int u = 0; u < nu; ++u) {
      g.tris().push_back({0, ring(1, u + 1), ring(1, u)});
      for (int v = 1; v + 1 < nv; ++v) {
        g.tris().push_back({ring(v, u), ring(v, u + 1), ring(v + 1, u + 1)});
        g.tris().push_back({ring(v, u), ring(v + 1, u + 1), ring(v + 1, u)});
      }
      g.tris().push_back({bottom, ring(nv - 1, u), ring(nv - 1, u + 1)});
    }
    cvc::write_geometry(g, (dir / "egg.off").string());
    const unsigned n = 24;
    cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float,
                    cvc::bounding_box(-1, -1, -1, 1, 1, 1));
    for (unsigned k = 0; k < n; ++k)
      for (unsigned j = 0; j < n; ++j)
        for (unsigned i = 0; i < n; ++i) {
          const double x = -1.0 + 2.0 * i / (n - 1), y = -1.0 + 2.0 * j / (n - 1),
                       z = -1.0 + 2.0 * k / (n - 1);
          const double r = std::sqrt(x * x + y * y + z * z);
          vol(i, j, k, r >= 0.9 ? 0.0 : 1.0 - r / 0.9); // 1 at the centre, 0 at r = 0.9
        }
    vol.min(0.0);
    vol.max(1.0);
    cvc::writeVolumeFile(app, vol, (dir / "ball.rawiv").string());
  }

  cvc::state &st = cvc::state::instance(app);
  SceneGraph sg(app, "lsrc");
  SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
  Scene scene;
  SceneNode m;
  m.id = "mesh";
  m.type = "mesh_lab";
  m.source_file = "x.bunny";
  m.base_dir = dir.string();
  scene.nodes.push_back(m);
  SceneNode d;
  d.id = "domain";
  d.type = "fe_lab";
  d.source_sdf_mesh = "x.bunny";
  d.sdf_dim = 16;
  d.has_fit = true;
  d.fit_up_y = true;
  d.fit_height = 100.0f;
  d.base_dir = dir.string();
  scene.nodes.push_back(d);
  std::vector<std::string> warnings;
  auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "lsrc", &warnings);
  for (const std::string &w : warnings)
    printf("     warning: %s\n", w.c_str());
  const std::string mb = cvc::ariadne::resolve_bind("lsrc", "graphics.root.children.mesh.mesh_ops");
  const std::string fb = cvc::ariadne::resolve_bind("lsrc", "graphics.root.children.domain.fe");
  auto val = [&](const std::string &path) { return st(path).value(); };
  auto num = [&](const std::string &path) {
    try {
      return st(path).value<double>();
    } catch (...) {
      return std::nan("");
    }
  };
  auto set = [&](const std::string &path, const std::string &v) { st(path).value(v); };
  auto tick = [&]() {
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
    cvc::pump_exec_frame(app.exec_scheduler()); // fe_lab / mesh_lab jobs run as scheduler tasks
  };
  auto settle = [&](const std::string &base) {
    for (int i = 0; i < 12000; ++i) {
      tick();
      if (val(base + ".request").empty() && val(base + ".busy") == "0")
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  };
  auto starts = [&](const std::string &path, const std::string &prefix) {
    return val(path).compare(0, prefix.size(), prefix) == 0;
  };
  bool ok = false;
  auto mesh = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("mesh"));
  auto domain = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("domain"));
  if (!mesh) {
    chk(false, "mesh_lab realized");
    return;
  }
  if (domain) // before any tick: the bounds are up, the field is the first tick's job
    chk(starts(fb + ".status", "loading x.bunny") && domain->getGeometry() &&
            domain->getGeometry()->num_lines() == 12,
        "fe_lab realize: an SDF source shows its box at once, its field loads on the lane");
  chk(val(mb + ".source.file") == "x.bunny" && val(mb + ".source.height") == "0",
      "mesh_lab seeded source.* from source: (no fit: height 0)");

  // mesh_lab: load another file, by a path relative to the document's directory, fitted.
  set(mb + ".source.file", "egg.off");
  set(mb + ".source.up", "y");
  set(mb + ".source.height", "50");
  set(mb + ".request", "load");
  ok = settle(mb); // before the message: MSVC builds the arguments first
  chk(ok && starts(mb + ".status", "loaded egg.off"),
      "mesh_lab load: a relative source.file resolves against base_dir (status '" +
          val(mb + ".status") + "')");
  const auto zrange = [](const cvc::geometry &g) {
    double lo = 1e300, hi = -1e300;
    for (const auto &p : g.const_points()) {
      lo = std::min(lo, p[2]);
      hi = std::max(hi, p[2]);
    }
    return std::make_pair(lo, hi);
  };
  chk(mesh->getGeometry() && approx(num(mb + ".stats.vertices"), 24.0 * 11 + 2) &&
          std::fabs(zrange(*mesh->getGeometry()).first) < 1e-6 &&
          std::fabs(zrange(*mesh->getGeometry()).second - 50.0) < 1e-6,
      "mesh_lab load: the egg turned y-up to z-up and fitted: z in [0, 50]");
  set(mb + ".request", "load");
  tick();
  chk(val(mb + ".busy") == "0" && val(mb + ".status") == "the source mesh (unchanged)",
      "mesh_lab load with unchanged source.* starts no job");
  set(mb + ".smooth.iterations", "1");
  set(mb + ".request", "load smooth");
  ok = settle(mb);
  chk(ok && starts(mb + ".status", "smoothed"),
      "mesh_lab: a chained request (load smooth) runs both, in order (status '" +
          val(mb + ".status") + "')");
  set(mb + ".source.file", "no_such_mesh.off");
  set(mb + ".request", "load");
  ok = settle(mb);
  chk(ok && starts(mb + ".status", "error:") && approx(num(mb + ".stats.vertices"), 24.0 * 11 + 2),
      "mesh_lab load of a missing file reports an error and keeps the mesh");
  set(mb + ".source.file", "egg.off");
  set(mb + ".request", "load"); // the source keys match the loaded mesh again: a reset
  ok = settle(mb);
  chk(ok, "mesh_lab back to the egg");

  // mesh_lab: Laplace-Beltrami eigenfunctions (followed like the other colour modes).
  set(mb + ".modes.index", "2");
  set(mb + ".color.mode", "modes");
  ok = settle(mb);
  chk(ok && starts(mb + ".status", "coloured by mode 2") && mesh->hasScalarField() &&
          num(mb + ".color.min") < 0.0 && num(mb + ".color.max") > 0.0,
      "color.mode modes: a surface eigenfunction, both signs (status '" + val(mb + ".status") +
          "')");
  set(mb + ".modes.index", "4");
  ok = settle(mb);
  chk(ok && starts(mb + ".status", "coloured by mode 4"), "modes.index 4 recolours");
  set(mb + ".geodesic.source", "-1");
  set(mb + ".color.mode", "geodesic");
  ok = settle(mb);
  chk(ok && num(mb + ".color.max") > 0.0, "geodesic.source -1: from the highest vertex");

  // Field colours (<node>.field_colors, a transfer function a tf_editor edits): its range and a
  // histogram of the values are published beside it, the colormap key resets its ramp, and an
  // explicit domain is the range the field is coloured over.
  const auto field_colors = [&](const std::string &T, const std::string &cm_key,
                                const std::string &min_key, const std::string &max_key,
                                GeometryNode &node, const char *who) {
    cvc::tf::model m;
    double lo = 0.0, hi = 0.0;
    std::vector<double> bins;
    double total = 0.0;
    const bool published = cvc::tf::from_string(val(T), m) && m.color.size() > 2 &&
                           cvc::tf::range_from_string(val(T + ".range"), lo, hi) &&
                           cvc::tf::histogram_from_string(val(T + ".histogram"), bins);
    for (const double b : bins)
      total += b;
    chk(published && approx(lo, num(min_key)) && approx(hi, num(max_key)) && bins.size() == 128 &&
            total > 0.0,
        std::string(who) + ": field_colors holds a ramp; its range is the values' (" +
            val(T + ".range") + ") and a 128-bin histogram is published");
    // Every colormap seeds a few colour bars an editor can grab (at most 12), within 0.02 (about
    // 5/255) of the colormap everywhere where 12 suffice; turbo, the busiest, within 0.05. Viridis
    // is the seed; the colormap key resets to the others.
    std::vector<double> ts(256);
    for (std::size_t i = 0; i < ts.size(); ++i)
      ts[i] = double(i) / double(ts.size() - 1);
    for (const char *name :
         {"viridis", "magma", "plasma", "inferno", "turbo", "jet", "parula", "gray"}) {
      cvc::colormap_kind kind = cvc::colormap_kind::VIRIDIS;
      cvc::colormap_from_string(name, kind);
      set(cm_key, name);
      tick();
      cvc::tf::model cm;
      const std::vector<unsigned char> want = cvc::colormap_rgb(ts, kind, 0.0, 1.0);
      float miss = cvc::tf::from_string(val(T), cm) && want.size() == 3 * ts.size() ? 0.f : 1.f;
      for (std::size_t i = 0; i < ts.size() && miss < 1.f; ++i) {
        float rgb[3];
        cm.color_at(ts[i], rgb);
        for (int c = 0; c < 3; ++c)
          miss = std::max(miss, std::fabs(rgb[c] - want[3 * i + std::size_t(c)] / 255.f));
      }
      chk(cm.color.size() >= 2 && cm.color.size() <= 12 &&
              (miss <= 0.0201f || (cm.color.size() == 12 && miss <= 0.05f)),
          std::string(who) + ": the " + name + " ramp is " + std::to_string(cm.color.size()) +
              " colour nodes, within " + std::to_string(miss) + " of the colormap");
    }
    set(cm_key, "viridis");
    tick();
    // The contour spectrum and tree an editor asks for: an overlay task on the field's own mesh,
    // over the published range, publishes them beside it (the scheduler pumped as a host's frames
    // would pump it).
    set(T + ".overlay_request", "contour_spectrum,contour_tree");
    cvc::tf::spectrum_overlay sp;
    cvc::tf::tree_overlay tr;
    for (int f = 0; f < 4000 && (sp.empty() || tr.empty()); ++f) {
      tick();
      cvc::pump_exec_frame(app.exec_scheduler());
      cvc::tf::spectrum_from_string(val(T + ".contour_spectrum"), sp);
      cvc::tf::tree_from_string(val(T + ".contour_tree"), tr);
    }
    const double peak = sp.empty() ? 0.0 : *std::max_element(sp.area.begin(), sp.area.end());
    chk(sp.size() == 256 && peak > 0.0 && tr.vertices.size() >= 2 &&
            val(T + ".overlay_status").empty(),
        std::string(who) + ": a contour spectrum / tree request is computed on the field's mesh (" +
            std::to_string(sp.size()) + " samples, peak area " + std::to_string(peak) + ", " +
            std::to_string(tr.vertices.size()) + " tree nodes, status '" +
            val(T + ".overlay_status") + "')");
    set(cm_key, "gray");
    tick();
    float c0[3] = {1.f, 1.f, 1.f}, c1[3] = {0.f, 0.f, 0.f};
    if (cvc::tf::from_string(val(T), m)) {
      m.color_at(0.0, c0);
      m.color_at(1.0, c1);
    }
    chk(c0[0] < 0.05f && c1[0] > 0.95f && m.domain.automatic,
        std::string(who) + ": the colormap key gray resets field_colors to black..white");
    m.domain.automatic = false;
    m.domain.min = lo - 1.0;
    m.domain.max = hi + 1.0;
    set(T, cvc::tf::to_string(m));
    tick();
    const std::pair<double, double> r = node.scalarRange();
    chk(node.hasScalarField() && approx(r.first, lo - 1.0) && approx(r.second, hi + 1.0),
        std::string(who) + ": an explicit field_colors domain is the colour range (" +
            std::to_string(r.first) + ".." + std::to_string(r.second) + ")");
    set(cm_key, "viridis"); // back to the viridis ramp, auto domain
    tick();
    chk(node.hasScalarField() && approx(node.scalarRange().first, lo) &&
            approx(node.scalarRange().second, hi),
        std::string(who) + ": the colormap key resets the domain to auto");
    chk(!val(T + ".contour_spectrum").empty() && !val(T + ".contour_tree").empty(),
        std::string(who) + ": recolouring the same values keeps their spectrum and tree");
  };
  field_colors(cvc::ariadne::resolve_bind("lsrc", "graphics.root.children.mesh.field_colors"),
               mb + ".color.colormap", mb + ".color.min", mb + ".color.max", *mesh, "mesh_lab");

#ifdef CVC_ENABLE_MESHER
  if (!domain) { // the SDF needs CVC_ENABLE_SDF
    printf("     (fe_lab skipped: no volume -- a build without CVC_ENABLE_SDF?)\n");
    fs::remove_all(dir, ec);
    return;
  }
  chk(val(fb + ".source.type") == "sdf" && val(fb + ".source.file") == "x.bunny" &&
          approx(num(fb + ".source.dim"), 16.0) && val(fb + ".source.up") == "y" &&
          approx(num(fb + ".source.height"), 100.0),
      "fe_lab seeded source.* from source: { sdf } and fit: (type, file, dim, up, height)");

  // fe_lab: load a volume file (relative path), fitted, then mesh and solve on their own.
  set(fb + ".source.type", "volume");
  set(fb + ".source.file", "ball.rawiv");
  set(fb + ".source.dim", "16");
  set(fb + ".source.up", "z");
  set(fb + ".source.height", "40");
  set(fb + ".mesh.inside", "above");
  set(fb + ".mesh.isovalue", "0.3");
  set(fb + ".auto.mesh", "1");
  set(fb + ".auto.solve", "1");
  set(fb + ".problem", "poisson");
  set(fb + ".request", "load");
  ok = settle(fb);
  chk(ok && starts(fb + ".status", "poisson: solved") && num(fb + ".stats.tets") > 0.0 &&
          num(fb + ".field.max") > 0.0,
      "fe_lab load: a volume file, then auto.mesh and auto.solve (status '" + val(fb + ".status") +
          "', " + val(fb + ".stats.tets") + " tets)");
  {
    const auto z =
        domain->getGeometry() ? zrange(*domain->getGeometry()) : std::make_pair(1e300, -1e300);
    // The isosurface v = 0.3 of the ball (r = 0.63 of the box's half-width 1) sits inside the
    // box, which the fit stands on z = 0 and scales to 40: z in about [7.4, 32.6].
    chk(z.first > 3.0 && z.first < 12.0 && z.second > 28.0 && z.second < 37.0,
        "fe_lab: a volume's fit: places its elements (z " + std::to_string(z.first) + ".." +
            std::to_string(z.second) + ")");
  }
  const std::string tets = val(fb + ".stats.tets");
  set(fb + ".request", "load");
  ok = settle(fb);
  chk(ok && val(fb + ".stats.tets") == tets && starts(fb + ".status", "poisson: solved"),
      "fe_lab load with unchanged source and mesh keys only re-solves (auto.solve)");

  // Vibration modes, the eigenpairs cached across modes.index.
  set(fb + ".problem", "modes");
  set(fb + ".modes.index", "1");
  set(fb + ".request", "solve");
  ok = settle(fb);
  chk(ok && starts(fb + ".status", "modes: mode 1") && num(fb + ".field.min") < 0.0 &&
          num(fb + ".field.max") > 0.0,
      "problem modes: mode 1 of the solid, both signs (status '" + val(fb + ".status") + "')");
  set(fb + ".modes.index", "3");
  set(fb + ".request", "solve");
  ok = settle(fb);
  chk(ok && starts(fb + ".status", "modes: mode 3"), "modes.index 3 solves");

  // A chained request: mesh, then solve, with auto.solve off.
  set(fb + ".auto.solve", "0");
  set(fb + ".problem", "poisson");
  set(fb + ".request", "mesh solve");
  ok = settle(fb);
  chk(ok && starts(fb + ".status", "poisson: solved"),
      "fe_lab: a chained request (mesh solve) runs both");
  chk(!val(fb + ".stats.work_ms").empty() && num(fb + ".stats.work_ms") >= 0.0,
      "fe_lab publishes stats.work_ms (" + val(fb + ".stats.work_ms") + ")");
  tick();
  field_colors(cvc::ariadne::resolve_bind("lsrc", "graphics.root.children.domain.field_colors"),
               fb + ".view.colormap", fb + ".field.min", fb + ".field.max", *domain, "fe_lab");

  // Interval meshing: the shell between two densities is a solid layer with a hole.
  set(fb + ".mesh.region", "interval");
  set(fb + ".mesh.isovalue2", "0.6");
  set(fb + ".request", "mesh solve");
  const bool interval_ok =
      settle(fb) && num(fb + ".stats.tets") > 0.0 && starts(fb + ".status", "poisson: solved");
  chk(interval_ok,
      "mesh.region interval: the shell between isovalue and isovalue2 meshes and solves (" +
          val(fb + ".stats.tets") + " tets, status '" + val(fb + ".status") + "')");
  set(fb + ".mesh.region", "solid");

  // Hexahedra: the elements view only. A solve is refused with a reason.
  set(fb + ".mesh.elements", "hex");
  set(fb + ".request", "mesh");
  ok = settle(fb);
  chk(ok && num(fb + ".stats.hexes") > 0.0 && approx(num(fb + ".stats.tets"), 0.0),
      "mesh.elements hex: stats.hexes > 0, stats.tets 0 (" + val(fb + ".stats.hexes") + " hexes)");
  chk(num(fb + ".stats.quality_min") <= num(fb + ".stats.quality_mean") &&
          num(fb + ".stats.quality_mean") <= 1.0 + 1e-9,
      "hex quality stats: the scaled Jacobian (min <= mean <= 1)");
  set(fb + ".request", "solve");
  tick();
  chk(val(fb + ".busy") == "0" && val(fb + ".status").find("needs tetrahedra") != std::string::npos,
      "a solve on hexahedra is refused (status '" + val(fb + ".status") + "')");

  // The elements view: the cut-away elements on the node, outlined by <id>_edges.
  auto edges = std::dynamic_pointer_cast<GeometryNode>(domain->findChildByName("domain_edges"));
  auto ghost = std::dynamic_pointer_cast<GeometryNode>(domain->findChildByName("domain_ghost"));
  chk(edges && ghost, "fe_lab made its <id>_edges and <id>_ghost children");
  set(fb + ".view.mode", "elements");
  set(fb + ".view.color_by", "quality");
  set(fb + ".view.show_slice", "1");
  set(fb + ".view.ghost", "0.2");
  bool shown = false;
  for (int i = 0; i < 3000 && !shown; ++i) {
    tick();
    shown = edges && edges->isVisible() && domain->hasScalarField();
    if (!shown)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  chk(shown && edges->getGeometry() && edges->getGeometry()->num_lines() > 0 && ghost->isVisible(),
      "view.mode elements: coloured elements, their outlines and the ghost show");
  const std::size_t cut_faces = domain->getGeometry() ? domain->getGeometry()->num_tris() : 0;
  set(fb + ".view.show_slice", "0"); // no cut: every element is kept
  bool whole = false;
  for (int i = 0; i < 3000 && !whole; ++i) {
    tick();
    whole = domain->getGeometry() && domain->getGeometry()->num_tris() != cut_faces;
    if (!whole)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  chk(whole, "elements view: turning the cut off shows the whole element mesh");
  set(fb + ".view.fixed_range", "1");
  set(fb + ".view.range_min", "0");
  set(fb + ".view.range_max", "1");
  tick();
  tick();
  chk(approx(domain->scalarRange().first, 0.0) && approx(domain->scalarRange().second, 1.0),
      "view.fixed_range maps the colours over [range_min, range_max]");
  set(fb + ".view.mode", "field");
  tick();
  tick();
  chk(!edges->isVisible() && !ghost->isVisible(), "view.mode field hides the outlines and ghost");
  set(fb + ".mesh.elements", "tet");
  set(fb + ".view.fixed_range", "0");

  // An interval with hexahedra is refused by the mesh job (LBIE makes interval tetrahedra only).
  set(fb + ".mesh.elements", "hex");
  set(fb + ".mesh.region", "interval");
  set(fb + ".request", "mesh");
  ok = settle(fb);
  chk(ok && starts(fb + ".status", "error: interval meshing makes tetrahedra"),
      "interval + hex is refused (status '" + val(fb + ".status") + "')");
  set(fb + ".mesh.elements", "tet");
  set(fb + ".mesh.region", "solid");

  // Back to an SDF source: load reads it again, and the old mesh is dropped meanwhile.
  set(fb + ".source.type", "sdf");
  set(fb + ".source.file", "egg.off");
  set(fb + ".source.up", "y");
  set(fb + ".source.dim", "16");
  set(fb + ".source.height", "60");
  set(fb + ".mesh.inside", "below");
  set(fb + ".mesh.isovalue", "0");
  set(fb + ".auto.solve", "1");
  set(fb + ".request", "load");
  tick(); // launches the load: the mesh is still there until it finishes
  chk(val(fb + ".busy") == "1" && starts(fb + ".status", "loading egg.off"),
      "fe_lab load runs on the lane (status '" + val(fb + ".status") + "')");
  ok = settle(fb);
  chk(ok && starts(fb + ".status", "poisson: solved") && num(fb + ".stats.tets") > 0.0,
      "fe_lab: the SDF of a mesh file, meshed and solved (status '" + val(fb + ".status") + "')");
#else
  (void)domain;
  (void)starts;
#endif
  fs::remove_all(dir, ec);
}
#endif

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");

  // ── lights: batch position bake + directional-vs-spot routing ───────────────
  {
    SceneGraph sg(app, "lights");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");

    Scene scene;
    SceneLight key;    // a spot whose trailing setters are all no-ops (defaults), so
    key.id = "key";    // ONLY the light batch can bake its moved position — this is
    key.kind = "spot"; // exactly the batch-removal regression.
    key.pos[0] = 10;
    key.pos[1] = 20;
    key.pos[2] = 30;
    key.target[0] = 0;
    key.target[1] = 0;
    key.target[2] = 0;
    key.cone = 30; // == LightNode's internal default (a no-op setCone)
    scene.lights.push_back(key);

    SceneLight sun; // a directional sun — az/el, must come out NON-positional
    sun.id = "sun";
    sun.kind = "directional";
    sun.azimuth = 45;
    sun.elevation = 60;
    scene.lights.push_back(sun);

    cvc::gl::ariadne::realize_scene(sg, scene, "lights");
    sg.processEvents();

    LightSurvey s = survey(view.renderer());
    printf("== lights realize: batch bake + kind routing ==\n");
    printf("     positional=%d directional=%d posLightPos=(%g,%g,%g)\n", s.positional,
           s.directional, s.posLightPos[0], s.posLightPos[1], s.posLightPos[2]);
    chk(s.positional == 1, "spot -> exactly one positional vtkLight");
    chk(s.directional == 1, "directional -> exactly one non-positional vtkLight");
    chk(approx(s.posLightPos[0], 10) && approx(s.posLightPos[1], 20) &&
            approx(s.posLightPos[2], 30),
        "light batch baked the spot's moved position (10,20,30), not the origin");
  }

  // ── a declared directional sun LIGHTS the scene, from where its angles say ──────
  {
    // The block above counts the sun as one non-positional vtkLight, and a sun that lit nothing
    // was exactly that: its vtkLight sat at the node's origin aimed at its (0,0,0) target, a zero
    // direction, so a scene with a declared sun drew ambient-only and every azimuth gave the same
    // frame (hello.ari's bunny came out a dark silhouette). Only the pixels show it. A unit box
    // seen from (+x,-y,+z): a sun at azimuth 135 sits towards +x+y, lighting the +x face and
    // leaving the -y face in shade; at azimuth -45 it sits towards -x-y and the two swap. The top
    // faces both. Each frame is shot with shadows off and on, since the bug ignored both.
    struct Faces {
      double px = 0, my = 0, top = 0; // mean of R,G,B at each visible face's centre
      bool drawn = false;
    };
    auto shoot = [&](const char *name, float azimuth, bool shadows) {
      SceneGraph sg(app, name);
      SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");
      Scene scene;
      SceneNode box;
      box.id = "box";
      box.type = "geometry";
      box.source_primitive = "box"; // unit edges, centred on the origin
      box.has_material = true;
      box.color[0] = box.color[1] = box.color[2] = 0.8f;
      scene.nodes.push_back(box);
      SceneLight sun;
      sun.id = "sun";
      sun.kind = "directional";
      sun.azimuth = azimuth;
      sun.elevation = 35;
      scene.lights.push_back(sun);
      scene.has_shadows = true;
      scene.shadows_enabled = shadows;
      scene.has_chrome = true; // no grid or bounds box over the faces
      scene.chrome_visible = false;
      cvc::gl::ariadne::realize_scene(sg, scene, name);
      sg.processEvents();
      view.setBackground(0, 0, 0);
      view.setCamera(4, -5, 3.5, 0, 0, 0, 0, 0, 1, 30.0, 0.1, 100.0);
      const std::vector<unsigned char> f = view.frameRGB();
      const int w = view.frameWidth(), h = view.frameHeight();
      Faces out;
      if (f.size() != std::size_t(w) * h * 3)
        return out;
      auto at = [&](double x, double y, double z) {
        view.renderer()->SetWorldPoint(x, y, z, 1);
        view.renderer()->WorldToDisplay();
        double d[3];
        view.renderer()->GetDisplayPoint(d);
        const int px = std::clamp(int(d[0]), 0, w - 1), py = std::clamp(int(d[1]), 0, h - 1);
        const unsigned char *c = &f[3 * (std::size_t(py) * w + px)];
        return (c[0] + c[1] + c[2]) / 3.0;
      };
      out.px = at(0.5, 0, 0);
      out.my = at(0, -0.5, 0);
      out.top = at(0, 0, 0.5);
      out.drawn = out.px > 0 || out.my > 0 || out.top > 0;
      return out;
    };
    printf("== a declared directional sun lights the scene, from its azimuth ==\n");
    for (const bool shadows : {false, true}) {
      const Faces ne = shoot(shadows ? "sun135s" : "sun135", 135.0f, shadows);
      const Faces sw = shoot(shadows ? "sunm45s" : "sunm45", -45.0f, shadows);
      printf("     shadows %s: azimuth 135 +x=%.0f -y=%.0f top=%.0f | azimuth -45 +x=%.0f -y=%.0f "
             "top=%.0f\n",
             shadows ? "on " : "off", ne.px, ne.my, ne.top, sw.px, sw.my, sw.top);
      if (!ne.drawn || !sw.drawn) {
        printf("     (skipped: this build did not rasterise)\n");
        continue;
      }
      const std::string tag = shadows ? " (shadows on)" : " (shadows off)";
      // 40 of 255 is far above rasteriser noise and far below the ~90 a 35-degree sun adds.
      chk(ne.px > ne.my + 40, "azimuth 135 lights the +x face, the -y face stays in shade" + tag);
      chk(sw.my > sw.px + 40, "azimuth -45 lights the -y face, the +x face stays in shade" + tag);
      chk(ne.top > ne.my + 40 && sw.top > sw.px + 40,
          "both suns light the top above the shaded side" + tag);
    }
  }

  // ── nesting: a child's transform is LOCAL (world = parent ∘ local) ──────────
  {
    SceneGraph sg(app, "geo");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");

    Scene scene;
    SceneNode convoy; // a transformed group...
    convoy.id = "convoy";
    convoy.type = "group";
    convoy.has_transform = true;
    convoy.position[0] = 100;
    SceneNode truck; // ...with a child at a LOCAL offset (embedded bunny, no file)
    truck.id = "truck";
    truck.type = "geometry";
    truck.source_file = "x.bunny";
    truck.has_transform = true;
    truck.position[0] = 5;
    convoy.children.push_back(truck);
    scene.nodes.push_back(convoy);

    // A rig needs geometry bounds, so add one here too.
    SceneLight rig;
    rig.id = "studio";
    rig.rig = "three_point";
    scene.lights.push_back(rig);

    cvc::gl::ariadne::realize_scene(sg, scene, "geo");
    sg.processEvents();

    printf("== nesting: local transform composes through the parent ==\n");
    auto group = sg.getGraphics("convoy");
    chk(group != nullptr, "top-level group is registered and found");
    std::shared_ptr<cvc::gl::GraphicsNode> child =
        group ? group->findChildByName("truck") : nullptr;
    chk(child != nullptr, "child 'truck' is nested UNDER the convoy group");
    if (child) {
      const double origin[3] = {0, 0, 0};
      double world[3] = {0, 0, 0};
      child->localToWorld(origin, world);
      printf("     truck world origin = (%g,%g,%g)\n", world[0], world[1], world[2]);
      chk(approx(world[0], 105), "child local [5,0,0] under parent [100,0,0] -> world [105,..]");
    }

    printf("== rig: a StageLighting preset adds lights ==\n");
    chk(survey(view.renderer()).positional + survey(view.renderer()).directional >= 1,
        "rig: three_point realized at least one light");
  }

  // ── volren + volslice realize + the per-frame tick seam ─────────────────────
  {
    SceneGraph sg(app, "vol");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");

    // The realizer loads volumes from source.file, so write a small density ball to
    // a temp rawiv (the app ctor registered the rawiv handler). n=32 keeps it fast.
    const std::string volPath = "ari_realize_ball.rawiv";
    {
      const unsigned n = 32;
      cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float,
                      cvc::bounding_box(-1, -1, -1, 1, 1, 1));
      for (unsigned k = 0; k < n; ++k)
        for (unsigned j = 0; j < n; ++j)
          for (unsigned i = 0; i < n; ++i) {
            const double x = -1.0 + 2.0 * i / (n - 1);
            const double y = -1.0 + 2.0 * j / (n - 1);
            const double z = -1.0 + 2.0 * k / (n - 1);
            const double r = std::sqrt(x * x + y * y + z * z);
            vol(i, j, k, r >= 0.9 ? 0.0 : 1.0 - r / 0.9);
          }
      vol.min(0.0);
      vol.max(1.0);
      cvc::writeVolumeFile(app, vol, volPath);
    }

    Scene scene;
    SceneNode vr;
    vr.id = "vr";
    vr.type = "volren";
    vr.source_file = volPath;
    vr.has_volren = true;
    vr.volren.ambient = 0.6f; // lit without needing a light -> not a black silhouette
    vr.volren.steps = 128;
    {
      SceneIsosurface iso; // a shell at density 0.5
      iso.value = 0.5f;
      iso.opacity = 1.0f;
      iso.color[0] = 1.0f;
      iso.color[1] = 0.3f;
      iso.color[2] = 0.2f;
      iso.shininess = 32.0f;
      vr.volren.isosurfaces.push_back(iso);
    }
    scene.nodes.push_back(vr);

    SceneNode vs;
    vs.id = "vs";
    vs.type = "volslice";
    vs.source_file = volPath;
    vs.has_volslice = true;
    vs.volslice.quality = 0.5f;
    {
      SceneTFPoint p;
      p.value = 0.0f;
      p.color[0] = 1.0f;
      vs.volslice.tf.points.push_back(p);
    }
    {
      SceneTFPoint p;
      p.value = 1.0f;
      p.color[0] = 1.0f;
      p.color[3] = 0.8f;
      vs.volslice.tf.points.push_back(p);
    }
    scene.nodes.push_back(vs);

    // A `type: volume` node (the VTK GPU mapper) with a transfer function + shading toggle —
    // the `volume:` block, fed to VolumeNode::setTransferFunction/setShading by the realizer.
    SceneNode vn;
    vn.id = "vn";
    vn.type = "volume";
    vn.source_file = volPath;
    vn.has_volume = true;
    vn.volume.has_shaded = true;
    vn.volume.shaded = false; // VolumeNode defaults shading TRUE -> this asserts a real flip
    {
      SceneTFPoint p; // low end: a translucent blue
      p.value = 0.0;
      p.color[0] = 0.00f;
      p.color[1] = 0.42f;
      p.color[2] = 0.78f;
      p.color[3] = 0.0f;
      vn.volume.tf.points.push_back(p);
    }
    {
      SceneTFPoint p; // high end: a darker blue at half opacity
      p.value = 1.0;
      p.color[0] = 0.01f;
      p.color[1] = 0.09f;
      p.color[2] = 0.22f;
      p.color[3] = 0.5f;
      vn.volume.tf.points.push_back(p);
    }
    scene.nodes.push_back(vn);

    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "vol");
    std::remove(volPath.c_str()); // the volumes are already read into memory

    printf("== volren/volslice realize + tick seam ==\n");
    chk(realized.volren_ticks.size() == 1, "volren node recorded as a per-frame ticker");
    chk(realized.volslice_ticks.size() == 1, "volslice node recorded as a per-frame ticker");
    auto vrn = std::dynamic_pointer_cast<cvc::gl::VolRenNode>(sg.getGraphics("vr"));
    auto vsn = std::dynamic_pointer_cast<cvc::gl::VolSliceNode>(sg.getGraphics("vs"));
    chk(vrn != nullptr, "type: volren -> a VolRenNode in the graph");
    chk(vsn != nullptr, "type: volslice -> a VolSliceNode in the graph");

    // Drive the tick seam. VolRenNode raycasts on a worker thread, so tick+render
    // until it converges (one initial render sets up GL first, per the volren tests).
    view.render();
    bool converged = false;
    for (int i = 0; i < 400 && !converged; ++i) {
      cvc::gl::ariadne::tick_scene(realized, view.renderer());
      view.render();
      if (auto n = realized.volren_ticks[0].lock())
        converged = n->converged();
    }
    chk(converged, "volren tick seam -> the raycast produced a frame (converged)");
    chk(vsn && vsn->planesRendered() > 0, "volslice tick seam -> slice planes were built");

    auto vnn = std::dynamic_pointer_cast<cvc::gl::VolumeNode>(sg.getGraphics("vn"));
    chk(vnn != nullptr, "type: volume -> a VolumeNode in the graph");
    if (vnn) {
      chk(!vnn->getShading(), "volume: { shaded: false } flips VolumeNode's default-true shading");
      // 2 TF points -> color table [s,r,g,b, s,r,g,b]; assert OUR colour reached the mapper
      // (not the default black->white ramp, which would have 0 in the green slot).
      auto col = vnn->getTransferFunctionColorTable();
      chk(col.size() == 8, "volume transfer_function -> 2 colour control points applied");
      chk(col.size() == 8 && std::abs(col[2] - 0.42) < 1e-6,
          "volume transfer_function -> our green channel (0.42) reached the mapper");
      auto op = vnn->getTransferFunctionOpacityTable();
      chk(op.size() == 4 && std::abs(op[3] - 0.5) < 1e-6,
          "volume transfer_function -> our opacity (0.5) reached the mapper");
    }
  }

  // ── a procedural heightfield source -> a displaced grid mesh (verts/uvs/colours) ──
  {
    SceneGraph sg(app, "hf");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    SceneNode t;
    t.id = "terrain";
    t.type = "geometry";
    t.has_heightfield = true;
    t.heightfield.size = 100.0f;
    t.heightfield.resolution = 25; // ODD -> a vertex lands exactly at the centre (0,0)
    {
      SceneHeightLayer d; // a central dome peaking at z = amplitude
      d.kind = "dome";
      d.amplitude = 20.0f;
      d.radius = 30.0f;
      t.heightfield.layers.push_back(d);
    }
    {
      SceneHeightColorBand b; // low band (green)
      b.max_height = 5.0f;
      b.color[1] = 1.0f;
      t.heightfield.colors.push_back(b);
    }
    {
      SceneHeightColorBand b; // high band (red)
      b.max_height = 100.0f;
      b.color[0] = 1.0f;
      t.heightfield.colors.push_back(b);
    }
    scene.nodes.push_back(t);
    cvc::gl::ariadne::realize_scene(sg, scene, "hf");

    printf("== heightfield geometry source ==\n");
    auto gn = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("terrain"));
    chk(gn != nullptr, "type: geometry + heightfield -> a GeometryNode in the graph");
    if (gn && gn->getGeometry()) {
      const cvc::geometry &geom = *gn->getGeometry();
      chk(geom.const_points().size() == 25u * 25u, "heightfield -> res*res vertices");
      chk(geom.const_tris().size() == 2u * 24u * 24u, "heightfield -> 2*(res-1)^2 triangles");
      chk(geom.const_uvs().size() == 25u * 25u, "heightfield -> a uv per vertex");
      chk(geom.const_colors().size() == 25u * 25u, "heightfield colours -> a colour per vertex");
      double maxz = -1e30;
      for (const auto &p : geom.const_points())
        maxz = std::max(maxz, p[2]);
      chk(std::abs(maxz - 20.0) < 1e-6,
          "heightfield dome -> the centre vertex reaches the amplitude");
    }
  }

  // ── material: { texture } -> an image is read + applied through the mesh UVs ──
  {
    SceneGraph sg(app, "tex");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    const std::string imgPath = "ari_realize_tex.png"; // a tiny RGBA image the realizer reads back
    {
      cvc::image img(4, 4); // 4x4 RGBA u8 (ctor defaults)
      cvc::write_image(img, imgPath);
    }
    Scene scene;
    SceneNode t;
    t.id = "ground";
    t.type = "geometry";
    t.has_heightfield = true; // a heightfield carries UVs, so the texture samples
    t.heightfield.size = 10.0f;
    t.heightfield.resolution = 4;
    t.has_material = true;
    t.material_texture = imgPath;
    scene.nodes.push_back(t);
    std::vector<std::string> warnings;
    cvc::gl::ariadne::realize_scene(sg, scene, "tex", &warnings);
    std::remove(imgPath.c_str());

    printf("== material texture ==\n");
    auto gn = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("ground"));
    chk(gn != nullptr, "textured geometry -> a GeometryNode in the graph");
    bool texWarn = false;
    for (const std::string &w : warnings)
      if (w.find("texture") != std::string::npos)
        texWarn = true;
    chk(!texWarn, "a valid texture URI -> no warning (image read + applied)");
  }

  // ── a bad texture URI warns but still creates the node (graceful, never throws) ──
  {
    SceneGraph sg(app, "texbad");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    SceneNode t;
    t.id = "ground";
    t.type = "geometry";
    t.has_heightfield = true;
    t.heightfield.size = 10.0f;
    t.heightfield.resolution = 4;
    t.has_material = true;
    t.material_texture = "ari_realize_nope.png"; // does not exist
    scene.nodes.push_back(t);
    std::vector<std::string> warnings;
    cvc::gl::ariadne::realize_scene(sg, scene, "texbad", &warnings);
    auto gn = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("ground"));
    chk(gn != nullptr, "bad texture -> the GeometryNode is still created (graceful)");
    bool texWarn = false;
    for (const std::string &w : warnings)
      if (w.find("texture") != std::string::npos)
        texWarn = true;
    chk(texWarn, "a bad texture URI -> a warning (never a throw)");
  }

  // ── §9 shaders: a declarative shader: block applies a preset / warns on an unknown one ──
  {
    cvc::gl::ariadne::register_default_shader_presets();
    printf("== shader DSL (presets + inline) ==\n");
    chk(cvc::gl::ariadne::has_shader_preset("terrain_bump"),
        "default preset terrain_bump registered");
    chk(cvc::gl::ariadne::has_shader_preset("bark"), "default preset bark registered");

    SceneGraph sg(app, "shd");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    SceneNode g; // a known preset applies cleanly
    g.id = "ground";
    g.type = "geometry";
    g.has_heightfield = true;
    g.heightfield.size = 10.0f;
    g.heightfield.resolution = 4;
    g.shader.present = true;
    g.shader.preset = "terrain_bump";
    SceneNode bad; // an unknown preset warns but still realizes
    bad.id = "bad";
    bad.type = "geometry";
    bad.source_primitive = "plane";
    bad.plane_size = 4.0f;
    bad.shader.present = true;
    bad.shader.preset = "does_not_exist";
    scene.nodes.push_back(g);
    scene.nodes.push_back(bad);
    std::vector<std::string> warnings;
    cvc::gl::ariadne::realize_scene(sg, scene, "shd", &warnings);
    chk(sg.getGraphics("ground") != nullptr, "a node with a known shader preset realizes");
    chk(sg.getGraphics("bad") != nullptr, "an unknown shader preset still realizes the node");
    bool known_warned = false, unknown_warned = false;
    for (const std::string &w : warnings) {
      if (w.find("unknown shader preset 'does_not_exist'") != std::string::npos)
        unknown_warned = true;
      if (w.find("terrain_bump") != std::string::npos)
        known_warned = true;
    }
    chk(unknown_warned, "an unknown shader preset -> a warning");
    chk(!known_warned, "a known shader preset -> no warning");
  }

  // ── §9 L-system node types + the cvcGL extension bundle ──────────────────────
  {
    cvc::gl::ariadne::register_cvcgl_extensions(app);
    printf("== lsystem node types (forest_trees) + extension bundle ==\n");
    chk(cvc::gl::ariadne::has_scene_node_type("forest_trees"),
        "register_cvcgl_extensions installed the forest_trees node type");
    chk(cvc::gl::ariadne::has_shader_preset("bark"),
        "register_cvcgl_extensions installed the bark shader preset");

    SceneGraph sg(app, "forest");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    SceneNode terrain; // a flat heightfield to plant on
    terrain.id = "terrain";
    terrain.type = "geometry";
    terrain.has_heightfield = true;
    terrain.heightfield.size = 120.0f;
    terrain.heightfield.resolution = 24;
    SceneNode trees;
    trees.id = "trees";
    trees.type = "forest_trees";
    trees.props.kind = Value::Kind::Map;
    auto pnum = [&](const char *k, double v) {
      Value val;
      val.kind = Value::Kind::Scalar;
      val.scalar = std::to_string(v);
      trees.props.entries.emplace_back(k, val);
    };
    auto pstr = [&](const char *k, const char *v) {
      Value val;
      val.kind = Value::Kind::Scalar;
      val.scalar = v;
      trees.props.entries.emplace_back(k, val);
    };
    pstr("species", "pine");
    pnum("count", 8);
    pnum("span", 40);
    pnum("sea_level", -10); // everything is "dry land" so the small forest plants
    pstr("ground", "terrain");
    trees.shader.present = true;
    trees.shader.preset = "bark"; // the bark shader is declared in the scene, not host code
    chk(cvc::gl::ariadne::has_scene_node_type("wave_sea"), "the wave_sea node type is registered");
    chk(cvc::gl::ariadne::has_scene_node_type("cloud_sky"),
        "the cloud_sky node type is registered");
    // The sea + sky are VolumeNodes sharing the terrain; add them to the same scene.
    SceneNode sea;
    sea.id = "sea";
    sea.type = "wave_sea";
    sea.props.kind = Value::Kind::Map;
    {
      Value v;
      v.kind = Value::Kind::Scalar;
      v.scalar = "40";
      sea.props.entries.emplace_back("half", v);
    }
    SceneNode sky;
    sky.id = "sky";
    sky.type = "cloud_sky";
    sky.props.kind = Value::Kind::Map;
    {
      Value v;
      v.kind = Value::Kind::Scalar;
      v.scalar = "0";
      sky.props.entries.emplace_back("shadow", v); // skip the shadow bake to keep the test quick
    }
    scene.nodes.push_back(terrain);
    scene.nodes.push_back(trees);
    scene.nodes.push_back(sea);
    scene.nodes.push_back(sky);
    std::vector<std::string> warnings;
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "forest", &warnings);
    chk(sg.getGraphics("trees") != nullptr, "forest_trees realized a wood GeometryNode");
    chk(sg.getGraphics("trees_needles") != nullptr, "forest_trees realized a needle-LINES node");
    chk(std::dynamic_pointer_cast<cvc::gl::VolumeNode>(sg.getGraphics("sea")) != nullptr,
        "wave_sea realized a VolumeNode");
    chk(std::dynamic_pointer_cast<cvc::gl::VolumeNode>(sg.getGraphics("sky")) != nullptr,
        "cloud_sky realized a VolumeNode");
    // forest wind + sea refill + cloud drift each push a tick.
    chk(realized.custom_ticks.size() >= 3u, "forest/sea/sky each registered a per-frame tick");
    bool emptyForest = false;
    for (const std::string &w : warnings)
      if (w.find("no trees planted") != std::string::npos)
        emptyForest = true;
    chk(!emptyForest, "forest_trees planted trees on the heightfield (no empty-forest warning)");
    // Live controls: the realizers SEED their forest.*/sea.* keys from the node props, so a bound
    // control panel starts at the authored value. (Prefix "forest" here -> forest.forest.count.)
    chk(approx(cvc::state::instance(app)(cvc::ariadne::resolve_bind("forest", "forest.count"))
                   .value<double>(),
               8.0),
        "forest_trees seeded forest.count from its props (live-control binding)");
    chk(approx(cvc::state::instance(app)(cvc::ariadne::resolve_bind("forest", "sea.wave_amp"))
                   .value<double>(),
               2.4),
        "wave_sea seeded sea.wave_amp from its props");
  }

  // ── two top-level volren nodes share an id -> last wins, no leaked orphan ────
  {
    SceneGraph sg(app, "dup");
    const std::string p = "ari_realize_dup.rawiv";
    {
      const unsigned n = 16;
      cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float,
                      cvc::bounding_box(-1, -1, -1, 1, 1, 1));
      for (unsigned k = 0; k < n; ++k)
        for (unsigned j = 0; j < n; ++j)
          for (unsigned i = 0; i < n; ++i)
            vol(i, j, k, 0.5);
      vol.min(0.0);
      vol.max(1.0);
      cvc::writeVolumeFile(app, vol, p);
    }
    Scene scene;
    for (int i = 0; i < 2; ++i) { // same id "dup" twice
      SceneNode vr;
      vr.id = "dup";
      vr.type = "volren";
      vr.source_file = p;
      vr.has_volren = true;
      SceneIsosurface iso;
      iso.value = 0.5;
      vr.volren.isosurfaces.push_back(iso);
      scene.nodes.push_back(vr);
    }
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "dup");
    std::remove(p.c_str());
    printf("== duplicate top-level id: last wins, orphan unlinked ==\n");
    chk(realized.volren_ticks.size() == 2, "both realize attempts recorded a ticker");
    chk(realized.volren_ticks.size() == 2 && realized.volren_ticks[0].expired(),
        "the first same-id node was unlinked/destroyed (its weak_ptr expired)");
    chk(!realized.volren_ticks.empty() && !realized.volren_ticks.back().expired(),
        "the last same-id node survives");
    chk(std::dynamic_pointer_cast<cvc::gl::VolRenNode>(sg.getGraphics("dup")) != nullptr,
        "getGraphics(id) resolves to the surviving VolRenNode");
  }

  // ── custom scene node type via register_scene_node_type ─────────────────────
  {
    // A capture-free realizer (process-global): builds a geometry node from the
    // embedded bunny, records that it read its props bag, and registers a per-frame
    // tick — exercising the whole extension seam (props in, GraphicsNode out, tick).
    cvc::gl::ariadne::register_scene_node_type(
        "beacon",
        [](SceneGraph &sg, const cvc::ariadne::SceneNode &n, cvc::gl::GraphicsNode *parent,
           cvc::gl::ariadne::RealizedScene &out,
           std::vector<std::string> *) -> std::shared_ptr<cvc::gl::GraphicsNode> {
          g_custom_radius = n.props.num("radius", -1.0);           // reads the neutral props bag
          cvc::geometry geom = cvc::read_geometry("beacon.bunny"); // embedded, no file
          std::shared_ptr<cvc::gl::GraphicsNode> node;
          if (parent)
            node = parent->createChild<cvc::gl::GeometryNode>(n.id, geom);
          else
            node = sg.addGraphics(n.id, geom);
          out.custom_ticks.push_back([](vtkRenderer *) { ++g_custom_ticks; });
          return node;
        });

    SceneGraph sg(app, "ext");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");

    Scene scene;
    SceneNode b;
    b.id = "b1";
    b.type = "beacon";
    b.has_transform = true;
    b.position[0] = 100;             // the custom node's own (local) transform
    b.props.kind = Value::Kind::Map; // as the loader would have captured `radius: 2.5`
    {
      Value radius;
      radius.kind = Value::Kind::Scalar;
      radius.scalar = "2.5";
      b.props.entries.emplace_back("radius", radius);
    }
    // A built-in child under the custom node — proves the shared tail (transform +
    // visibility + children) runs on a custom node just like a built-in one.
    SceneNode child;
    child.id = "truck";
    child.type = "geometry";
    child.source_file = "x.bunny";
    child.has_transform = true;
    child.position[0] = 5; // local to the beacon -> world 105
    b.children.push_back(child);
    scene.nodes.push_back(b);

    const int before = g_custom_ticks;
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "ext");

    printf("== custom scene node type (register_scene_node_type) ==\n");
    chk(cvc::gl::ariadne::has_scene_node_type("beacon"), "custom type is registered");
    chk(std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("b1")) != nullptr,
        "custom realizer built a GraphicsNode addressable in the graph");
    chk(std::fabs(g_custom_radius - 2.5) < 1e-9, "custom realizer read its props bag (radius=2.5)");
    chk(realized.custom_ticks.size() == 1, "custom realizer registered a per-frame tick");
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
    chk(g_custom_ticks == before + 1, "tick_scene runs the custom per-frame tick");
    // The shared tail (transform + children) runs on a custom node like a built-in.
    auto beacon = sg.getGraphics("b1");
    auto truck = beacon ? beacon->findChildByName("truck") : nullptr;
    chk(truck != nullptr, "a built-in child nests UNDER a custom node type");
    if (truck) {
      const double origin[3] = {0, 0, 0};
      double world[3] = {0, 0, 0};
      truck->localToWorld(origin, world);
      chk(approx(world[0], 105),
          "shared tail composes transforms: custom node [100] ∘ child local [5] -> world [105]");
    }
  }

  // ── §9 time: a `clock:` scene drives app.world_clock() through tick_scene ────
  {
    // A clock-only scene (defaults: bind sim.speed / sim.paused, publish sim.time). realize_scene
    // resolves the key paths with the host prefix; tick_scene then steers app.world_clock() from
    // them each frame. We inject a fixed wall_dt (the deterministic-capture path) of one quantum,
    // so the arithmetic is exact: scale 1 → +1 step/quantum, paused → none, scale 2 → +2.
    SceneGraph sg(app, "clk");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    scene.clock.present = true; // defaults carry sim.speed / sim.paused / sim.time
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "clk");

    printf("== scene clock drives app.world_clock() ==\n");
    chk(realized.clock.present, "clock declaration carried into the realized scene");
    chk(realized.clock.speed_key == "clk.sim.speed", "clock speed_key resolved with host prefix");
    chk(realized.clock.paused_key == "clk.sim.paused",
        "clock paused_key resolved with host prefix");
    chk(realized.clock.time_key == "clk.sim.time", "clock time_key resolved with host prefix");

    cvc::world_clock &wc = app.world_clock();
    wc.reset(); // deterministic start (shared per-app clock; no other section declares one)
    const double q = wc.fixed_dt();
    for (int i = 0; i < 4; ++i)
      cvc::gl::ariadne::tick_scene(realized, view.renderer(), q); // 4 quanta of wall time, scale 1
    chk(approx(wc.t(), 4.0 * q), "scale 1: four injected quanta advance the clock by 4·fixed_dt");
    chk(approx(cvc::state::instance(app)("clk.sim.time").value<double>(), wc.t()),
        "sim.time publishes the live world_clock t()");

    cvc::state::instance(app)("clk.sim.paused").value(1); // the sim_transport checkbox 'Paused'
    const double tp = wc.t();
    cvc::gl::ariadne::tick_scene(realized, view.renderer(), q);
    chk(approx(wc.t(), tp), "sim.paused freezes the clock (banks no time)");

    cvc::state::instance(app)("clk.sim.paused").value(0);
    cvc::state::instance(app)("clk.sim.speed").value(2.0); // the Speed slider at 2×
    const double t2 = wc.t();
    cvc::gl::ariadne::tick_scene(realized, view.renderer(), q); // q wall × 2 = 2 quanta world
    chk(wc.t() > t2 + 1.5 * q, "sim.speed=2 advances ~two quanta per quantum of wall time");

    // The drive goes through a world_clock_binding: the keys beside sim.speed (mode,
    // lockstep_steps, step) steer it too, and tick_scene returns the quanta it ran.
    const cvc::world_clock_keys k = cvc::gl::ariadne::scene_clock_keys(realized.clock);
    chk(k.mode == "clk.sim.mode" && k.step == "clk.sim.step" && k.tick.empty(),
        "scene_clock_keys: the rest of the binding sits beside sim.speed; tick stays off");
    chk(cvc::state::instance(app)("clk.sim.mode").value() == "live",
        "the binding seeded clk.sim.mode for the transport widget");

    cvc::state::instance(app)("clk.sim.speed").value(1.0);
    cvc::state::instance(app)("clk.sim.mode").value(std::string("lockstep"));
    cvc::state::instance(app)("clk.sim.lockstep_steps").value(3);
    const std::uint64_t k0 = wc.tick();
    const cvc::world_clock::step_result ls =
        cvc::gl::ariadne::tick_scene(realized, view.renderer(), 10.0);
    chk(ls.steps == 3 && wc.tick() == k0 + 3,
        "sim.mode=lockstep: 3 quanta per frame, wall ignored");
    chk(ls.first_tick == k0, "tick_scene returns the frame's step_result (first_tick)");

    cvc::state::instance(app)("clk.sim.mode").value(std::string("stepping"));
    chk(cvc::gl::ariadne::tick_scene(realized, view.renderer(), 1.0).steps == 0,
        "sim.mode=stepping: wall time runs nothing");
    cvc::state::instance(app)("clk.sim.step").value(2);
    chk(cvc::gl::ariadne::tick_scene(realized, view.renderer(), 1.0).steps == 2,
        "sim.step: each increment runs one quantum");
    cvc::state::instance(app)("clk.sim.mode").value(std::string("live"));
  }

  // ── verify_scene_customs: the customs: gate for NODE types (cvcGL side) ──────
  {
    using cvc::ariadne::CustomRequirement;
    using K = cvc::ariadne::CustomRequirement::Kind;
    printf("== verify_scene_customs: node customs gate ==\n");

    // "beacon" was registered above; an unregistered node type is not.
    cvc::ariadne::LoadResult good;
    good.customs.push_back({K::Node, "beacon", true});
    chk(cvc::gl::ariadne::verify_scene_customs(good, nullptr),
        "required node custom that IS registered -> verify passes");

    cvc::ariadne::LoadResult bad;
    bad.customs.push_back({K::Node, "unregistered_node_zzz", true});
    std::vector<std::string> errs;
    chk(!cvc::gl::ariadne::verify_scene_customs(bad, &errs),
        "required node custom that is MISSING -> verify fails");
    chk(!errs.empty(), "... and appends an error message");

    cvc::ariadne::LoadResult opt;
    opt.customs.push_back({K::Node, "unregistered_optional", false});
    std::vector<std::string> notes;
    chk(cvc::gl::ariadne::verify_scene_customs(opt, &notes),
        "non-required missing node custom -> verify passes (degrade)");
    chk(!notes.empty(), "... with a note logged");

    cvc::ariadne::LoadResult widgetOnly; // widget/block customs are the loader's job
    widgetOnly.customs.push_back({K::Widget, "anything", true});
    chk(cvc::gl::ariadne::verify_scene_customs(widgetOnly, nullptr),
        "verify_scene_customs ignores widget customs (checked at load) -> passes");
  }

  // ── §9 scene enrichments: plane primitive, fit, specular material, shadow res, rig tuning ──
  {
    using cvc::gl::GeometryNode;
    SceneGraph sg(app, "enrich");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "enrich");
    Scene scene;

    // A procedural `plane` primitive — a geometry node with NO file source.
    SceneNode ground;
    ground.id = "ground";
    ground.type = "geometry";
    ground.source_primitive = "plane";
    ground.plane_size = 20.0f; // spans [-10,10]^2 at z=0
    ground.has_material = true;
    ground.use_single_color = true;
    ground.color[0] = 0.3f;
    ground.color[1] = 0.3f;
    ground.color[2] = 0.35f;
    ground.has_specular = true; // exercises the setSpecular/Power forwarding (no crash / node made)
    ground.specular = 0.25f;
    ground.specular_power = 24.0f;
    scene.nodes.push_back(ground);

    // A `fit`ted mesh: the embedded bunny, Y-up -> Z-up, base on z=0, tallest extent == 100.
    SceneNode bunny;
    bunny.id = "bunny";
    bunny.type = "geometry";
    bunny.source_file = "x.bunny";
    bunny.has_fit = true;
    bunny.fit_up_y = true;
    bunny.fit_height = 100.0f;
    scene.nodes.push_back(bunny);

    // A tuned StageLighting rig (stage + key + fill + ambient) — must still add lights, no crash.
    SceneLight rig;
    rig.rig = "three_point";
    rig.has_stage = true;
    rig.stage_center[2] = 50.0f;
    rig.stage_radius = 62.0f;
    rig.has_key = true;
    rig.key_intensity = 1.9f;
    rig.key_azimuth = -38.0f;
    rig.key_elevation = 52.0f;
    rig.key_cone = 34.0f;
    rig.has_fill = true;
    rig.fill = 0.85f;
    rig.has_rig_ambient = true;
    rig.rig_ambient = 0.4f;
    scene.lights.push_back(rig);

    scene.has_shadows = true;
    scene.shadows_enabled = true;
    scene.has_shadow_resolution = true;
    scene.shadow_resolution = 2048;
    scene.has_shadow_interval = true;
    scene.shadow_interval = 1;

    cvc::gl::ariadne::realize_scene(sg, scene, "enrich");
    printf("== §9 enrichments: plane / fit / specular / shadow-res / rig tuning ==\n");

    auto gnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("ground"));
    chk(gnode != nullptr, "plane primitive -> a GeometryNode with no file source");
    const cvc::geometry *pg = gnode ? gnode->getGeometry() : nullptr;
    chk(pg && pg->points().size() == 4 && pg->tris().size() == 2,
        "plane geometry is a 4-vert / 2-tri quad");
    if (pg && pg->points().size() == 4) {
      double lo = 1e30, hi = -1e30;
      for (const auto &p : pg->points()) {
        lo = std::min(lo, p[0]);
        hi = std::max(hi, p[0]);
      }
      chk(approx(lo, -10.0) && approx(hi, 10.0), "plane spans [-size/2, +size/2] in X");
    }

    auto bnode = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("bunny"));
    const cvc::geometry *bg = bnode ? bnode->getGeometry() : nullptr;
    chk(bg != nullptr && bg->points().size() > 100, "fit: the bunny mesh realized");
    if (bg && !bg->points().empty()) {
      double zlo = 1e30, zhi = -1e30, ext[3] = {0, 0, 0};
      double lo3[3] = {1e30, 1e30, 1e30}, hi3[3] = {-1e30, -1e30, -1e30};
      for (const auto &p : bg->points()) {
        for (int k = 0; k < 3; ++k) {
          lo3[k] = std::min(lo3[k], p[k]);
          hi3[k] = std::max(hi3[k], p[k]);
        }
      }
      zlo = lo3[2];
      zhi = hi3[2];
      for (int k = 0; k < 3; ++k)
        ext[k] = hi3[k] - lo3[k];
      const double tallest = std::max({ext[0], ext[1], ext[2]});
      chk(approx(zlo, 0.0), "fit: base sits on z=0");
      chk(approx(tallest, 100.0), "fit: tallest extent scaled to height (100)");
      (void)zhi;
    }

    chk(sg.shadowsEnabled(), "shadows enabled");
    chk(sg.shadowResolution() == 2048, "shadow resolution applied from the DSL");
    chk(sg.shadowUpdateInterval() == 1, "shadow update interval applied from the DSL");
    chk(survey(view.renderer()).positional + survey(view.renderer()).directional >= 1,
        "tuned rig still adds lights");
  }

  // ── §9 clip: planes in the node's own coordinates, a group as a clip region, a bound offset ────
  {
    SceneGraph sg(app, "clp");
    SceneRenderer view(sg, 128, 128, /*offscreen=*/true, "main");
    sg.setDiagnosticChromeVisible(false);
    Scene scene;
    SceneNode region; // a group moved to x = 10, clipping its subtree to y >= 0 (its own frame)
    region.id = "region";
    region.type = "group";
    region.has_transform = true;
    region.position[0] = 10;
    region.clip.present = true;
    SceneClipPlane keepUp;
    keepUp.normal[0] = 0, keepUp.normal[1] = 1, keepUp.normal[2] = 0;
    region.clip.planes.push_back(keepUp);
    SceneNode quad; // inside it, at local x = 0.5, its own cut x >= offset (bound to sim.cut)
    quad.id = "quad";
    quad.type = "geometry";
    quad.source_primitive = "plane";
    quad.plane_size = 4.0f;
    quad.has_transform = true;
    quad.position[0] = 0.5;
    quad.clip.present = true;
    SceneClipPlane cut;
    cut.normal[0] = 2, cut.normal[1] = 0, cut.normal[2] = 0; // not unit: normalized on realize
    cut.offset = 0.25;
    cut.offset_bind = "sim.cut";
    quad.clip.planes.push_back(cut);
    region.children.push_back(quad);
    SceneNode host; // clip.children: its child is clipped to the host's own (mesh) bounds
    host.id = "host";
    host.type = "geometry";
    host.source_primitive = "plane";
    host.plane_size = 2.0f;
    host.clip.present = true;
    host.clip.children = true;
    SceneNode guest;
    guest.id = "guest";
    guest.type = "geometry";
    guest.source_primitive = "plane";
    guest.plane_size = 6.0f;
    host.children.push_back(guest);
    SceneNode junk; // a zero normal and an inverted box: warned about and dropped
    junk.id = "junk";
    junk.type = "geometry";
    junk.source_primitive = "plane";
    junk.clip.present = true;
    junk.clip.has_box = true;
    const double inverted[6] = {1, 0, 0, -1, 1, 1};
    std::copy(inverted, inverted + 6, junk.clip.box);
    SceneClipPlane zero;
    zero.normal[0] = zero.normal[1] = zero.normal[2] = 0;
    junk.clip.planes.push_back(zero);
    SceneNode many; // a box (6) + one plane on a mesh: over the poly-data mapper's 6
    many.id = "many";
    many.type = "geometry";
    many.source_primitive = "plane";
    many.clip.present = true;
    many.clip.has_box = true;
    const double unit[6] = {-1, -1, -1, 1, 1, 1};
    std::copy(unit, unit + 6, many.clip.box);
    many.clip.planes.push_back(keepUp);
    scene.nodes = {region, host, junk, many};
    std::vector<std::string> warnings;
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "clp", &warnings);
    sg.processEvents();

    printf("== clip: own coordinates, accumulation, bound offset ==\n");
    auto rg = sg.getGraphics("region");
    std::shared_ptr<cvc::gl::GraphicsNode> qd = rg ? rg->findChildByName("quad") : nullptr;
    chk(qd != nullptr, "clip: the quad realized under the region");
    vtkPlaneCollection *pc = qd ? qd->getAppliedClipPlanes() : nullptr;
    chk(pc && pc->GetNumberOfItems() == 2,
        "clip: the quad is clipped by its own plane + the region's");
    if (pc && pc->GetNumberOfItems() == 2) {
      double *o = pc->GetItem(0)->GetOrigin(), *nrm = pc->GetItem(0)->GetNormal();
      printf("     quad cut: origin (%g,%g,%g) normal (%g,%g,%g)\n", o[0], o[1], o[2], nrm[0],
             nrm[1], nrm[2]);
      chk(approx(o[0], 10.75) && approx(nrm[0], 1.0),
          "clip: the quad's plane is in ITS frame — 10 (region) + 0.5 (quad) + 0.25 (offset)");
      chk(approx(pc->GetItem(1)->GetNormal()[1], 1.0) &&
              approx(pc->GetItem(1)->GetOrigin()[0], 10.0),
          "clip: the region's plane is in the region's frame (through x = 10, keep y >= 0)");
      chk(realized.clip.size() == 1, "clip: one bound offset -> one SceneClipBinding");
      cvc::state::instance(app)("clp.sim.cut").value(1.0);
      cvc::gl::ariadne::tick_scene(realized, view.renderer());
      sg.processEvents();
      chk(qd->getAppliedClipPlanes() == pc && approx(pc->GetItem(0)->GetOrigin()[0], 11.5),
          "clip: sim.cut = 1 -> tick_scene slides the quad's cut to x = 11.5, in place");
    }
    auto hs = sg.getGraphics("host");
    std::shared_ptr<cvc::gl::GraphicsNode> gs = hs ? hs->findChildByName("guest") : nullptr;
    chk(hs && gs && hs->getAppliedClipPlanes() == nullptr && gs->getAppliedClipPlanes() &&
            gs->getAppliedClipPlanes()->GetNumberOfItems() == 6,
        "clip.children: the child is clipped to the host's box, the host is not");
    bool zeroWarned = false, boxWarned = false, capWarned = false;
    for (const std::string &w : warnings) {
      zeroWarned =
          zeroWarned || w.find("'junk': clip.planes[0] has a zero normal") != std::string::npos;
      boxWarned = boxWarned || w.find("'junk': clip.box has min > max") != std::string::npos;
      capWarned = capWarned || w.find("'many' is clipped by 7 planes but its renderer honours 6") !=
                                   std::string::npos;
    }
    chk(zeroWarned && boxWarned, "clip: a zero normal and an inverted box are warned about");
    auto jk = sg.getGraphics("junk");
    chk(jk && jk->clipPlaneCount() == 0, "clip: ... and dropped");
    chk(capWarned, "clip: 7 planes on a mesh -> a warning that the nearest 6 apply");

    printf("== clip: the cut on screen ==\n");
    // Just the quad, viewed head-on: x >= 11.5 (and y >= 0) is all that is left of it.
    for (const char *id : {"host", "junk", "many"})
      if (auto n = sg.getGraphics(id))
        n->setVisible(false);
    view.setBackground(0, 0, 0);
    view.setCamera(10.5, 0, 12, 10.5, 0, 0, 0, 1, 0, 30.0, 1.0, 100.0);
    const std::vector<unsigned char> f = view.frameRGB();
    const int w = view.frameWidth(), h = view.frameHeight();
    auto litAt = [&](double x, double y) {
      view.renderer()->SetWorldPoint(x, y, 0, 1);
      view.renderer()->WorldToDisplay();
      double d[3];
      view.renderer()->GetDisplayPoint(d);
      const int px = std::clamp(int(d[0]), 0, w - 1), py = std::clamp(int(d[1]), 0, h - 1);
      const unsigned char *c = &f[3 * (std::size_t(py) * w + px)];
      return c[0] > 40 || c[1] > 40 || c[2] > 40;
    };
    if (f.size() == std::size_t(w) * h * 3 && litAt(12.0, 1.0)) {
      chk(!litAt(11.0, 1.0), "clip: on screen, nothing of the quad left of its cut (x < 11.5)");
      chk(!litAt(12.0, -1.0), "clip: ... nor below the region's plane (y < 0)");
    } else {
      printf("     (skipped: this build did not rasterise)\n");
    }
  }
  {
    // VTK's low-memory mapper (the WebGL/GLES default) honours no planes: the realizer says so.
    cvc::gl::setLowMemoryMapperPolicy(cvc::gl::LowMemoryMapperPolicy::Force);
    SceneGraph sg(app, "clplm");
    Scene scene;
    SceneNode q;
    q.id = "q";
    q.type = "geometry";
    q.source_primitive = "plane";
    q.clip.present = true;
    q.clip.planes.push_back(SceneClipPlane());
    scene.nodes.push_back(q);
    std::vector<std::string> warnings;
    cvc::gl::ariadne::realize_scene(sg, scene, "clplm", &warnings);
    bool warned = false;
    for (const std::string &w : warnings)
      warned = warned || w.find("'q' is clipped (1 plane(s)) but its renderer has no clip-plane "
                                "support") != std::string::npos;
    chk(warned, "clip: under the low-memory mapper -> a warning that the node draws unclipped");
    cvc::gl::setLowMemoryMapperPolicy(cvc::gl::LowMemoryMapperPolicy::Auto);
  }

  // ── LOD: RealizeOptions realizes a mesh-file node as a LodGraphicsNode, and only such a node ──
  {
    using cvc::gl::GeometryNode;
    using cvc::gl::LodGraphicsNode;
    using cvc::gl::LodPyramidCache;
    // The embedded bunny (69,473 triangles), stood up 1.7 m tall, in an orange material.
    auto bunny = [](const char *id) {
      SceneNode b;
      b.id = id;
      b.type = "geometry";
      b.source_file = "x.bunny";
      b.has_fit = true;
      b.fit_up_y = true;
      b.fit_height = 1.7f;
      b.has_material = true;
      b.color[0] = 0.9f;
      b.color[1] = 0.4f;
      b.color[2] = 0.1f;
      b.ambient = 0.3f;
      b.diffuse = 0.6f;
      b.has_specular = true;
      b.specular = 0.25f;
      b.specular_power = 12.0f;
      return b;
    };
    const SceneNode spec = bunny("spec");
    // The node's material, on a rung or a plain GeometryNode.
    auto material_matches = [&spec](const std::shared_ptr<GeometryNode> &g) {
      vtkActor *a = g ? vtkActor::SafeDownCast(g->prop()) : nullptr;
      if (!a)
        return false;
      const double *c = a->GetProperty()->GetColor();
      return approx(c[0], spec.color[0]) && approx(c[1], spec.color[1]) &&
             approx(c[2], spec.color[2]) && approx(a->GetProperty()->GetAmbient(), spec.ambient) &&
             approx(a->GetProperty()->GetDiffuse(), spec.diffuse) &&
             approx(a->GetProperty()->GetSpecular(), spec.specular);
    };
    auto is_lod = [](SceneGraph &sg, const std::string &id) {
      return std::dynamic_pointer_cast<LodGraphicsNode>(sg.getGraphics(id)) != nullptr;
    };
    auto is_geometry = [](SceneGraph &sg, const std::string &id) {
      return std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(id)) != nullptr;
    };

    printf("== LOD: a bunny becomes a LodGraphicsNode with RealizeOptions ==\n");
    {
      // Deferred, so the build runs exactly when this block polls.
      LodPyramidCache cache(cvc::lod::pyramid_params(), LodPyramidCache::default_max_build_tris(),
                            cvc::lane_mode::deferred);
      cvc::gl::ariadne::RealizeOptions opts;
      opts.lod = &cache;
      SceneGraph sg(app, "lodr");
      Scene scene;
      SceneNode top = bunny("bunny");
      top.has_transform = true;
      top.position[0] = 5.0f;
      scene.nodes.push_back(top);
      SceneNode grp; // a group holding a second, hidden bunny
      grp.id = "grp";
      grp.type = "group";
      grp.has_transform = true;
      grp.position[1] = 10.0f;
      SceneNode inner = bunny("inner");
      inner.visible_default = false;
      grp.children.push_back(inner);
      scene.nodes.push_back(grp);
      std::vector<std::string> warnings;
      auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "lodr", &warnings, opts);
      sg.processEvents();

      auto lod = std::dynamic_pointer_cast<LodGraphicsNode>(sg.getGraphics("bunny"));
      chk(lod != nullptr, "LOD: the bunny is a LodGraphicsNode, registered under its id");
      auto g = sg.getGraphics("grp");
      auto nested =
          g ? std::dynamic_pointer_cast<LodGraphicsNode>(g->findChildByName("inner")) : nullptr;
      chk(nested != nullptr, "LOD: a nested bunny is a LodGraphicsNode under its parent");
      chk(realized.lod_nodes.size() == 2 && realized.lod_nodes[0].lock() == lod &&
              realized.lod_nodes[1].lock() == nested,
          "LOD: both are listed in lod_nodes");
      chk(warnings.empty(), "LOD: no warnings");
      if (lod && nested) {
        chk(lod->rungCount() == 1 && lod->rungTriangles(0) == 69473,
            "LOD: the full bunny draws at once, as rung 0");
        chk(cache.builds() == 1 && cache.in_flight() == 1,
            "LOD: one build for the two nodes (one mesh)");
        chk(material_matches(lod->rung(0)), "LOD: rung 0 wears the node's material");
        const double origin[3] = {0, 0, 0};
        double world[3] = {0, 0, 0};
        lod->localToWorld(origin, world);
        double nestedWorld[3] = {0, 0, 0};
        nested->localToWorld(origin, nestedWorld);
        chk(approx(world[0], 5.0) && approx(nestedWorld[1], 10.0),
            "LOD: the node's transform applies (and composes under its parent)");
        chk(lod->isVisible() && !nested->isVisible() && nested->rung(0) &&
                !nested->rung(0)->prop()->GetVisibility(),
            "LOD: visible: false hides the ladder");

        chk(cache.poll() == 1, "LOD: one poll runs the deferred build");
        chk(lod->rungCount() > 1 && nested->rungCount() == lod->rungCount(),
            "LOD: both nodes get the coarser rungs");
        bool all = true;
        for (int k = 0; k < lod->rungCount(); ++k)
          all = all && material_matches(lod->rung(k)) && material_matches(nested->rung(k));
        chk(all, "LOD: every rung wears the node's material");
        int shown = 0;
        for (int k = 0; k < nested->rungCount(); ++k)
          shown += nested->rung(k)->prop()->GetVisibility() ? 1 : 0;
        chk(shown == 0, "LOD: the hidden node's new rungs stay hidden");
      }

      // A re-realize of the same scene (a room's whole-scene rev bump) needs no build.
      SceneGraph again(app, "lodr2");
      auto realized2 = cvc::gl::ariadne::realize_scene(again, scene, "lodr2", nullptr, opts);
      auto lod2 = std::dynamic_pointer_cast<LodGraphicsNode>(again.getGraphics("bunny"));
      chk(lod2 && lod2->rungCount() > 1 && cache.builds() == 1 && cache.in_flight() == 0,
          "LOD: realizing the same meshes again attaches the cached ladder");
      (void)realized2;
    }

    printf("== LOD: ineligible nodes stay GeometryNodes ==\n");
    {
      LodPyramidCache cache(cvc::lod::pyramid_params(), LodPyramidCache::default_max_build_tris(),
                            cvc::lane_mode::deferred);
      cvc::gl::ariadne::RealizeOptions opts;
      opts.lod = &cache;
      // A mesh file under lod_min_tris: a 12-triangle cube.
      const std::string smallPath = "ari_realize_lod_small.off";
      {
        cvc::geometry cube(app);
        for (int i = 0; i < 8; ++i) {
          const double x = (i & 1) ? 0.5 : -0.5, y = (i & 2) ? 0.5 : -0.5, z = (i & 4) ? 0.5 : -0.5;
          cube.points().push_back({x, y, z});
        }
        const int faces[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                                  {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
        for (const auto &f : faces)
          cube.tris().push_back({std::uint64_t(f[0]), std::uint64_t(f[1]), std::uint64_t(f[2])});
        cvc::write_geometry(cube, smallPath);
      }
      // (The texture makes the node ineligible whether or not the image is readable.)
      const std::string texPath = "ari_realize_lod_tex.png";
      try {
        cvc::write_image(cvc::image(4, 4), texPath);
      } catch (const std::exception &) {
      }

      SceneGraph sg(app, "lodi");
      Scene scene;
      SceneNode box; // a primitive
      box.id = "box";
      box.type = "geometry";
      box.source_primitive = "box";
      scene.nodes.push_back(box);
      SceneNode hf; // a heightfield
      hf.id = "hf";
      hf.type = "geometry";
      hf.has_heightfield = true;
      hf.heightfield.size = 10.0f;
      hf.heightfield.resolution = 128; // ~32k triangles: big enough, but terrain
      scene.nodes.push_back(hf);
      SceneNode tex = bunny("tex"); // a texture
      tex.material_texture = texPath;
      scene.nodes.push_back(tex);
      SceneNode shd = bunny("shd"); // a shader
      shd.shader.present = true;
      scene.nodes.push_back(shd);
      SceneNode clip = bunny("clip"); // a clip
      clip.clip.present = true;
      clip.clip.planes.push_back(SceneClipPlane());
      scene.nodes.push_back(clip);
      SceneNode body = bunny("body"); // a body
      body.has_transform = true;
      body.position[0] = 20.0f;
      body.body.present = true;
      body.body.motion = "static";
      body.body.shape = "box";
      scene.nodes.push_back(body);
      SceneNode parent = bunny("parent"); // children
      SceneNode kid;
      kid.id = "kid";
      kid.type = "group";
      parent.children.push_back(kid);
      scene.nodes.push_back(parent);
      SceneNode small; // under lod_min_tris
      small.id = "small";
      small.type = "geometry";
      small.source_file = smallPath;
      scene.nodes.push_back(small);
      std::vector<std::string> warnings;
      auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "lodi", &warnings, opts);
      std::remove(smallPath.c_str());
      std::remove(texPath.c_str());

      bool allPlain = true;
      for (const char *id : {"box", "hf", "tex", "shd", "clip", "body", "parent", "small"}) {
        const bool plain = is_geometry(sg, id) && !is_lod(sg, id);
        if (!plain)
          printf("     '%s' is not a plain GeometryNode\n", id);
        allPlain = allPlain && plain;
      }
      chk(allPlain, "LOD: a primitive, a heightfield, a textured, shaded, clipped, bodied or "
                    "parent mesh, and one under lod_min_tris, all stay GeometryNodes");
      chk(realized.lod_nodes.empty() && cache.builds() == 0, "LOD: ... and none is built");
      chk(material_matches(std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("shd"))),
          "LOD: the plain path still applies the material (apply_node_material)");
      chk(realized.body("body").valid(), "LOD: the bodied mesh still gets its body");
    }

    printf("== LOD: the four-argument realize_scene never makes a LOD node ==\n");
    {
      Scene scene;
      scene.nodes.push_back(bunny("bunny"));
      SceneGraph sg(app, "lod4");
      auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "lod4");
      chk(is_geometry(sg, "bunny") && !is_lod(sg, "bunny") && realized.lod_nodes.empty(),
          "LOD: four arguments -> a plain GeometryNode");
      chk(material_matches(std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("bunny"))),
          "LOD: ... wearing the node's material, as before");
      SceneGraph sg2(app, "lod5");
      auto realized2 = cvc::gl::ariadne::realize_scene(sg2, scene, "lod5", nullptr,
                                                       cvc::gl::ariadne::RealizeOptions());
      chk(is_geometry(sg2, "bunny") && !is_lod(sg2, "bunny") && realized2.lod_nodes.empty(),
          "LOD: default RealizeOptions (no cache) -> a plain GeometryNode too");
    }
  }

#ifdef CVC_ENABLE_LIBIGL
  // ── mesh_lab / fe_lab: a request key runs a job off the render thread; a tick applies it ──────
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  printf("== mesh_lab / fe_lab node types ==\n");
  chk(cvc::gl::ariadne::has_scene_node_type("mesh_lab") &&
          cvc::gl::ariadne::has_scene_node_type("fe_lab"),
      "register_cvcgl_extensions installed mesh_lab + fe_lab (CVC_ENABLE_LIBIGL)");
  if (cvc::gl::ariadne::has_scene_node_type("mesh_lab")) {
    using cvc::gl::GeometryNode;
    cvc::state &st = cvc::state::instance(app);
    SceneGraph sg(app, "mlab");
    SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");
    Scene scene;
    SceneNode m;
    m.id = "mesh";
    m.type = "mesh_lab";
    m.source_file = "x.bunny"; // the embedded bunny
    m.has_fit = true;
    m.fit_up_y = true;
    m.fit_height = 100.0f;
    m.props.kind = Value::Kind::Map;
    {
      Value v;
      v.kind = Value::Kind::Scalar;
      v.scalar = "2";
      m.props.entries.emplace_back("smooth_iterations", v);
    }
    scene.nodes.push_back(m);
    SceneNode d;
    d.id = "domain";
    d.type = "fe_lab";
    d.source_sdf_mesh = "x.bunny";
    d.sdf_dim = 16;
    d.has_fit = true;
    d.fit_up_y = true;
    d.fit_height = 100.0f;
    d.has_transform = true;
    d.position[0] = 130.0f;
    d.props.kind = Value::Kind::Map;
    {
      Value v; // every successful mesh queues a solve -- unless a request is already waiting
      v.kind = Value::Kind::Scalar;
      v.scalar = "true";
      d.props.entries.emplace_back("auto_solve", v);
    }
    scene.nodes.push_back(d);
    std::vector<std::string> warnings;
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "mlab", &warnings);
    for (const std::string &w : warnings)
      printf("     warning: %s\n", w.c_str());

    const std::string mb =
        cvc::ariadne::resolve_bind("mlab", "graphics.root.children.mesh.mesh_ops");
    const std::string fb = cvc::ariadne::resolve_bind("mlab", "graphics.root.children.domain.fe");
    auto val = [&](const std::string &path) { return st(path).value(); };
    auto num = [&](const std::string &path) {
      try {
        return st(path).value<double>();
      } catch (...) {
        return std::nan("");
      }
    };
    // Tick until the request was taken and its job (if any) applied; bounded at ~2 minutes.
    auto settle = [&](const std::string &base) {
      for (int i = 0; i < 12000; ++i) {
        cvc::gl::ariadne::tick_scene(realized, view.renderer());
        cvc::pump_exec_frame(app.exec_scheduler()); // fe_lab / mesh_lab jobs run as scheduler tasks
        if (val(base + ".request").empty() && val(base + ".busy") == "0")
          return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return false;
    };

    auto mesh = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("mesh"));
    chk(mesh != nullptr, "mesh_lab realized a GeometryNode");
    chk(val(mb + ".smooth.method") == "cotan" && approx(num(mb + ".smooth.iterations"), 2.0) &&
            val(mb + ".color.mode") == "none" && val(mb + ".status") == "idle",
        "mesh_lab seeded mesh_ops.* (smooth.iterations from its prop)");
    chk(num(mb + ".stats.faces") > 60000.0, "mesh_lab published stats.faces for the bunny");

    st(mb + ".decimate.target_faces").value(2000);
    st(mb + ".request").value(std::string("decimate"));
    const bool decimated = settle(mb);
    printf("     status: %s\n", val(mb + ".status").c_str());
    chk(decimated, "decimate: the request was consumed and the job finished");
    chk(num(mb + ".stats.faces") <= 2000.0 && num(mb + ".stats.faces") > 0.0,
        "decimate: stats.faces <= target_faces (2000)");
    chk(mesh && mesh->getGeometry() && mesh->getGeometry()->num_tris() <= 2000,
        "decimate: the node shows the decimated mesh");

    st(mb + ".color.mode").value(std::string("gaussian"));
    chk(settle(mb), "color.mode gaussian: the colour job finished");
    chk(mesh && mesh->hasScalarField(), "color.mode gaussian -> the node has a scalar field");
    chk(num(mb + ".color.min") < num(mb + ".color.max"), "... and published its colour range");

    st(mb + ".geodesic.source").value(5);
    st(mb + ".color.mode").value(std::string("geodesic"));
    chk(settle(mb) && num(mb + ".color.max") > 0.0,
        "color.mode geodesic -> distances from vertex 5");

    st(mb + ".request").value(std::string("reset"));
    chk(settle(mb) && num(mb + ".stats.faces") > 60000.0, "reset restores the source mesh");

    auto domain = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("domain"));
    if (!domain) {
      printf("     (fe_lab skipped: no volume — a build without CVC_ENABLE_SDF?)\n");
    } else {
      chk(domain->findChildByName("domain_slice") != nullptr, "fe_lab made its <id>_slice child");
      st(fb + ".request").value(std::string("mesh"));
      chk(settle(fb), "fe_lab mesh: the request was consumed and the job finished");
      printf("     status: %s\n", val(fb + ".status").c_str());
      if (val(fb + ".status") == "mesher unavailable") {
        printf("     (fe_lab meshing skipped: built without CVC_ENABLE_MESHER)\n");
      } else {
        chk(num(fb + ".stats.tets") > 0.0, "fe_lab mesh: stats.tets > 0");
        st(fb + ".problem").value(std::string("poisson"));
        st(fb + ".request").value(std::string("solve"));
        chk(settle(fb), "fe_lab solve: the Poisson job finished");
        printf("     status: %s  field [%s, %s]\n", val(fb + ".status").c_str(),
               val(fb + ".field.min").c_str(), val(fb + ".field.max").c_str());
        const double fmax = num(fb + ".field.max"), fmin = num(fb + ".field.min");
        chk(fmax > 0.0, "poisson (f = 1): field.max > 0 inside");
        // The minimum is the u = 0 held on the boundary (a little below it only where obtuse
        // tets break the discrete maximum principle).
        chk(fmin <= 1e-9 * fmax && fmin >= -0.05 * fmax, "poisson: field ~0 on the boundary");
        chk(domain->hasScalarField(), "poisson: the boundary surface is coloured by the field");

        st(fb + ".problem").value(std::string("heat"));
        st(fb + ".heat.steps").value(5);
        st(fb + ".request").value(std::string("solve"));
        chk(settle(fb), "fe_lab solve: the heat job finished");
        const double hmax = num(fb + ".field.max");
        chk(hmax >= 1.0 - 1e-9 && hmax <= 1.05, "heat: the hot floor is held at 1");

        auto tick = [&]() {
          cvc::gl::ariadne::tick_scene(realized, view.renderer());
          cvc::pump_exec_frame(
              app.exec_scheduler()); // fe_lab / mesh_lab jobs run as scheduler tasks
        };
        auto starts = [&](const std::string &path, const std::string &prefix) {
          return val(path).compare(0, prefix.size(), prefix) == 0;
        };

        // State counts are range-checked: NaN is the default, not a wrapped integer (x86 turns
        // static_cast<int>(NaN) into INT_MIN, which the old lower clamp made 1 step).
        st(fb + ".heat.steps").value(std::string("nan"));
        st(fb + ".request").value(std::string("solve"));
        chk(settle(fb), "heat.steps = nan: the heat job finished");
        chk(starts(fb + ".status", "heat: 100 steps"),
            "heat.steps = nan -> the default 100 steps (status '" + val(fb + ".status") + "')");

        // Stop is honest. A Poisson solve cannot be interrupted: fe.stoppable stays 0 (the Stop
        // button's enabled_when), a stop is consumed without claiming "stopping...", and the
        // solve completes with its timing.
        st(fb + ".problem").value(std::string("poisson"));
        st(fb + ".request").value(std::string("solve"));
        tick(); // launches it; the job is applied on a later tick at the earliest
        chk(val(fb + ".busy") == "1" && val(fb + ".stoppable") == "0",
            "a running Poisson solve is not stoppable (fe.stoppable 0)");
        {
          bool listed = false;
          for (const auto &pi : app.exec_scheduler().list_processes())
            listed = listed || pi.name.rfind("fe_lab/", 0) == 0;
          chk(listed,
              "the solve runs as a sliced task on the app's scheduler (an fe_lab/ process)");
        }
        st(fb + ".request").value(std::string("stop"));
        tick();
        chk(val(fb + ".request").empty() &&
                val(fb + ".status").find("stopping") == std::string::npos,
            "stop during a Poisson solve is consumed, not claimed (status '" + val(fb + ".status") +
                "')");
        // Settle before building the message: argument evaluation order is unspecified, and
        // MSVC would otherwise report the status from before the job finished.
        const bool poisson_ran = settle(fb) && starts(fb + ".status", "poisson: solved (") &&
                                 val(fb + ".stoppable") == "0";
        chk(poisson_ran, "the Poisson solve ran to the end (status '" + val(fb + ".status") + "')");

        // A heat solve is stoppable. heat.steps = 1e30 is clamped to the 1e6 cap -- not wrapped
        // to one step -- so the solve is still running when stop arrives.
        st(fb + ".problem").value(std::string("heat"));
        st(fb + ".heat.steps").value(std::string("1e30"));
        st(fb + ".request").value(std::string("solve"));
        tick();
        chk(val(fb + ".busy") == "1" && val(fb + ".stoppable") == "1",
            "a running heat solve is stoppable (fe.stoppable 1)");
        // Stop it once it is under way. A stop that reaches the job while it still waits for the
        // lane cancels it before its first step ('cancelled'), which proves nothing about the
        // clamp; a loaded macOS runner did that. 1% of 1e6 steps is a few seconds, while a solve
        // wrapped to one step would be done (busy 0) instead.
        for (int i = 0; i < 12000 && val(fb + ".busy") == "1" && num(fb + ".progress") < 1.0; ++i) {
          tick();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        st(fb + ".request").value(std::string("stop"));
        tick();
        const bool heat_stopped = settle(fb) && starts(fb + ".status", "heat: stopped at step") &&
                                  val(fb + ".stoppable") == "0";
        chk(heat_stopped, "heat.steps = 1e30 is clamped, and stop interrupts it (status '" +
                              val(fb + ".status") + "')");
        st(fb + ".heat.steps").value(5);

        // auto_solve must not overwrite a request queued while the mesh job ran: the waiting
        // re-mesh (an isovalue edit, say) runs next and auto-solves in its turn.
        st(fb + ".problem").value(std::string("poisson"));
        st(fb + ".request").value(std::string("mesh"));
        tick(); // launches the mesh job
        chk(val(fb + ".busy") == "1", "re-mesh: the mesh job is running");
        st(fb + ".request").value(std::string("mesh")); // queued behind it
        bool applied = false;
        for (int i = 0; i < 12000 && !applied; ++i) {
          tick();
          applied = val(fb + ".busy") == "0";
          if (!applied)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        chk(applied && val(fb + ".request") == "mesh",
            "auto_solve left the queued re-mesh waiting (request '" + val(fb + ".request") + "')");
        const bool remesh_solved =
            settle(fb) && starts(fb + ".status", "poisson: solved") && num(fb + ".field.max") > 0.0;
        chk(remesh_solved,
            "the queued re-mesh ran, then auto-solved (status '" + val(fb + ".status") + "')");

        // The <id>_slice child follows the fe_lab node's own visibility (the only one a document
        // can bind): hidden with it, never re-shown by a Slice toggle under a hidden parent, and
        // not brought back by re-showing the parent while the Slice checkbox is off.
        std::shared_ptr<cvc::gl::GraphicsNode> cut = domain->findChildByName("domain_slice");
        const std::string dvis = domain->stateName("visible");
        auto ticks = [&](int n) {
          for (int i = 0; i < n; ++i)
            tick();
        };
        if (!cut) {
          chk(false, "fe_lab still has its <id>_slice child");
        } else {
          st(fb + ".view.show_slice").value(1);
          ticks(3);
          chk(cut->isVisible(), "slice: the cut shows while fe_lab is visible");
          st(dvis).value(0);
          ticks(3);
          chk(!cut->isVisible(), "slice: hiding fe_lab hides its cut");
          st(fb + ".view.show_slice").value(0);
          ticks(3);
          st(fb + ".view.show_slice").value(1);
          ticks(3);
          chk(!cut->isVisible(),
              "slice: Slice off/on under a hidden fe_lab leaves no floating cut");
          st(dvis).value(1);
          ticks(3);
          chk(cut->isVisible(), "slice: showing fe_lab again brings its cut back");
          st(fb + ".view.show_slice").value(0);
          ticks(3);
          chk(!cut->isVisible(), "slice: Slice off hides the cut");
          st(dvis).value(0);
          ticks(1);
          st(dvis).value(1);
          ticks(3);
          chk(!cut->isVisible(),
              "slice: re-showing fe_lab does not bring back a cut Slice turned off");
          st(fb + ".view.show_slice").value(1);
          ticks(3);
          chk(cut->isVisible(), "slice: Slice on again shows it");
        }
      }
    }
  }
  if (cvc::gl::ariadne::has_scene_node_type("fe_lab"))
    test_lab_sources(app);
#endif

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
