/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The scene transfer-function bindings (cvc/gl/ariadne/tf_binding.h), end to end through
// realize_scene on a tiny generated volume: every volume node's <node>.transfer_function model is
// seeded, a legacy document's renderer keys are left exactly as configured (and lowering its seed
// reproduces the same transfer function -- a volren window ignored, as volren ignores it), a model
// edit is lowered once into the volren / volslice / VolumeNode / fe_lab keys, isocontours become
// volren isosurfaces, an explicit domain drives tf_auto_domain and volslice's window (a window
// someone else sets is the LUT's span and theirs: never re-lowered over, never written back),
// VolumeNode applies an edited TF live and keeps it across a reload (a degenerate placeholder's
// seed re-fits instead of blanking), fe.mesh.isovalue follows one isocontour by id and ignores the
// model's position rounding -- and, with fe.mesh.region interval, fe.mesh.isovalue2 follows a
// second (both ways, across each other, re-adopted or re-added when deleted, <tf>.interval
// published and cleared) -- the range is published, and the histogram / contour spectrum / tree
// arrive on request, computed by overlay tasks on the app's state_exec scheduler: nothing arrives
// until the scheduler is pumped (as an Ariadne Runtime pumps it, or by wait_overlays()), a
// cooperative run takes many frames of a slice each and publishes exactly compute_overlays', and a
// binding destroyed mid-computation leaves no process and publishes nothing afterwards (in both
// modes). A VolumeNode's replaced volume is followed even over the same range or at the address of
// the one it replaced, and its old layers are unpublished at once (as are a predecessor's at
// attach); idle, overlay_status keeps every missing layer's reason; and a binding that stops for
// good (destroyed, or its node removed) leaves no progress note behind unless a newer binding owns
// the key. An fe_lab whose SDF is computed after realize, and later re-loaded, has its volume
// followed the same way: the range is published as each load lands and the bars move to the keys'
// values on it, the keys untouched.
//
// No rendering: the bindings' ticks are run directly (tick_scene would also tick the renderers,
// which need a GL context), and nothing pumps app.exec_scheduler() but pump() and
// wait_overlays() (fe_lab's loads, scheduler tasks too, are pumped while the test waits for them).
// Plain main, returns non-zero on failure (NDEBUG is defined, so no assert()).

#include <algorithm>
#include <boost/signals2.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/tf/model.h>
#include <cvc/tf/overlay_tasks.h>
#include <cvc/tf/overlays.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using cvc::ariadne::Scene;
using cvc::ariadne::SceneIsosurface;
using cvc::ariadne::SceneNode;
using cvc::ariadne::SceneTFPoint;
using cvc::ariadne::Value;
using cvc::gl::SceneGraph;
using cvc::gl::ariadne::RealizedScene;
using cvc::gl::ariadne::tf_binding;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

namespace {

// A density ball, `scale` at the centre falling to 0 at r = 0.9: range exactly [0, scale] for an
// odd n (a voxel sits on the centre).
cvc::volume ball(cvc::app &app, unsigned n, double scale) {
  cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float, cvc::bounding_box(-1, -1, -1, 1, 1, 1));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = -1.0 + 2.0 * i / (n - 1), y = -1.0 + 2.0 * j / (n - 1),
                     z = -1.0 + 2.0 * k / (n - 1);
        const double r = std::sqrt(x * x + y * y + z * z);
        vol(i, j, k, r >= 0.9 ? 0.0 : scale * (1.0 - r / 0.9));
      }
  vol.min(0.0);
  vol.max(scale);
  return vol;
}

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}
void put(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}
std::vector<double> nums(const std::string &s) {
  std::vector<double> v;
  if (!s.empty())
    cvc::tf::parse_csv(s, v);
  return v;
}
bool near(double a, double b, double tol = 1e-6) { return std::fabs(a - b) <= tol; }

void tick(RealizedScene &r, int frames = 1) {
  for (int f = 0; f < frames; ++f)
    for (const auto &t : r.custom_ticks)
      t(nullptr);
}

// A flat table (`stride` values per point: the scalar, then channels) sampled piecewise-linearly
// at x and held past its ends -- how the renderers read it. Channel `ch` is 1-based.
double sample(const std::vector<double> &t, std::size_t stride, std::size_t ch, double x) {
  const std::size_t n = t.size() / stride;
  if (n == 0)
    return 0.0;
  if (x <= t[0])
    return t[ch];
  if (x >= t[(n - 1) * stride])
    return t[(n - 1) * stride + ch];
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const double x0 = t[i * stride], x1 = t[(i + 1) * stride];
    if (x >= x0 && x <= x1) {
      const double u = x1 > x0 ? (x - x0) / (x1 - x0) : 0.0;
      return t[i * stride + ch] * (1.0 - u) + t[(i + 1) * stride + ch] * u;
    }
  }
  return t[(n - 1) * stride + ch];
}

// Two colour (stride 4) / opacity (stride 2) table pairs draw the same over [lo, hi].
bool same_tf(const std::vector<double> &c0, const std::vector<double> &o0,
             const std::vector<double> &c1, const std::vector<double> &o1, double lo, double hi) {
  for (int i = 0; i <= 100; ++i) {
    const double x = lo + (hi - lo) * i / 100.0;
    for (std::size_t ch = 1; ch <= 3; ++ch)
      if (!near(sample(c0, 4, ch, x), sample(c1, 4, ch, x), 1e-5))
        return false;
    if (!near(sample(o0, 2, 1, x), sample(o1, 2, 1, x), 1e-5))
      return false;
  }
  return true;
}

bool parse_model(const std::string &s, cvc::tf::model &m) { return cvc::tf::from_string(s, m); }

bool range_is(cvc::app &app, const std::string &tf, double lo, double hi) {
  double a = 0.0, b = 0.0;
  return cvc::tf::range_from_string(val(app, tf + ".range"), a, b) && near(a, lo) && near(b, hi);
}

Value scalar(const std::string &s) {
  Value v;
  v.kind = Value::Kind::Scalar;
  v.scalar = s;
  return v;
}

using steady = std::chrono::steady_clock;

double ms_since(steady::time_point t0) {
  return std::chrono::duration<double, std::milli>(steady::now() - t0).count();
}

// One frame of an Ariadne Runtime's scheduler pump (Runtime::Impl::pump_and_sweep_actions); its
// time in ms.
double pump(cvc::app &app) {
  const steady::time_point t0 = steady::now();
  cvc::state_exec::async_scheduler &s = app.exec_scheduler();
  s.drain_ingress();
  s.wake_awaiting();
  s.sync_run(200000, 0.1);
  return ms_since(t0);
}

// The names of the overlay-task processes on the app's scheduler ("tf-overlays/<mode> <tf>") for
// the model key `tf` ("" = every key); `live_only` skips finished ones not yet reaped.
std::vector<std::string> overlay_procs(cvc::app &app, const std::string &tf,
                                       bool live_only = true) {
  const std::string head = "tf-overlays/", tail = " " + tf;
  std::vector<std::string> out;
  for (const cvc::state_exec::process_info &p : app.exec_scheduler().list_processes()) {
    if (live_only && cvc::state_exec::is_finished(p.status))
      continue;
    if (p.name.rfind(head, 0) != 0)
      continue;
    if (!tf.empty() && (p.name.size() < tail.size() ||
                        p.name.compare(p.name.size() - tail.size(), tail.size(), tail) != 0))
      continue;
    out.push_back(p.name);
  }
  return out;
}

// Pump until no overlay task is live on the scheduler (every binding's), or `timeout_ms` passes.
bool settle(cvc::app &app, double timeout_ms) {
  const steady::time_point t0 = steady::now();
  while (!overlay_procs(app, "").empty()) {
    if (ms_since(t0) > timeout_ms)
      return false;
    pump(app);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

} // namespace

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::state &st = cvc::state::instance(app);
  const std::string ballPath = "cvcgl_tf_binding_ball.rawiv";
  cvc::writeVolumeFile(app, ball(app, 17, 1.0), ballPath);
  const std::string zeroPath = "cvcgl_tf_binding_zero.rawiv";
  cvc::writeVolumeFile(app, ball(app, 5, 0.0), zeroPath); // every voxel 0: a degenerate range

  // ── legacy documents: seeded models, untouched renderer keys ────────────────────────────────
  SceneGraph sg(app, "tfb");
  Scene scene;
  {
    SceneNode vr; // volren with an isosurface and no transfer function (cvcgl_ariadne_realize's)
    vr.id = "vr";
    vr.type = "volren";
    vr.source_file = ballPath;
    vr.has_volren = true;
    SceneIsosurface iso;
    iso.value = 0.5;
    iso.opacity = 1.0f;
    iso.color[0] = 1.0f;
    iso.color[1] = 0.3f;
    iso.color[2] = 0.2f;
    iso.shininess = 32.0f;
    vr.volren.isosurfaces.push_back(iso);
    scene.nodes.push_back(vr);
  }
  {
    SceneNode vr2; // volren with legacy coupled points over the data range
    vr2.id = "vr2";
    vr2.type = "volren";
    vr2.source_file = ballPath;
    vr2.has_volren = true;
    SceneTFPoint a, b;
    a.value = 0.2;
    a.color[0] = 1.0f;
    b.value = 0.8;
    b.color[2] = 1.0f;
    b.color[3] = 0.6f;
    vr2.volren.tf.points = {a, b};
    scene.nodes.push_back(vr2);
  }
  {
    SceneNode vs; // volslice with a window and points outside it
    vs.id = "vs";
    vs.type = "volslice";
    vs.source_file = ballPath;
    vs.has_volslice = true;
    vs.volslice.tf.has_window = true;
    vs.volslice.tf.auto_domain = false;
    vs.volslice.tf.window_min = 0.1;
    vs.volslice.tf.window_max = 0.9;
    SceneTFPoint a, b;
    a.value = 0.0;
    a.color[0] = 1.0f;
    b.value = 1.0;
    b.color[0] = 1.0f;
    b.color[1] = 0.5f;
    b.color[3] = 0.8f;
    vs.volslice.tf.points = {a, b};
    scene.nodes.push_back(vs);
  }
  {
    SceneNode vn; // a VTK volume with the realize test's two-point TF
    vn.id = "vn";
    vn.type = "volume";
    vn.source_file = ballPath;
    vn.has_volume = true;
    SceneTFPoint a, b;
    a.value = 0.0;
    a.color[1] = 0.42f;
    a.color[2] = 0.78f;
    b.value = 1.0;
    b.color[0] = 0.01f;
    b.color[1] = 0.09f;
    b.color[2] = 0.22f;
    b.color[3] = 0.5f;
    vn.volume.tf.points = {a, b};
    scene.nodes.push_back(vn);
  }
  {
    SceneNode plain; // a VTK volume with no transfer_function: (VolumeNode's default ramp)
    plain.id = "plain";
    plain.type = "volume";
    plain.source_file = ballPath;
    scene.nodes.push_back(plain);
  }
  {
    SceneNode hm; // the model form: lowered by the binding at attach
    hm.id = "hm";
    hm.type = "volren";
    hm.source_file = ballPath;
    hm.has_volren = true;
    hm.volren.tf.has_model = true;
    hm.volren.tf.model = cvc::tf::model::preset("hot");
    scene.nodes.push_back(hm);
  }
  {
    SceneNode vr3; // volren with a legacy window: volren ignores it (the loader's auto_domain off)
    vr3.id = "vr3";
    vr3.type = "volren";
    vr3.source_file = ballPath;
    vr3.has_volren = true;
    vr3.volren.tf.has_window = true;
    vr3.volren.tf.auto_domain = false;
    vr3.volren.tf.window_min = 0.2;
    vr3.volren.tf.window_max = 0.8;
    SceneTFPoint a, b;
    a.value = 0.0;
    a.color[0] = 1.0f;
    a.color[3] = 0.1f;
    b.value = 1.0;
    b.color[2] = 1.0f;
    b.color[3] = 0.9f;
    vr3.volren.tf.points = {a, b};
    scene.nodes.push_back(vr3);
  }
  {
    SceneNode vs2; // volslice with legacy points and no window: the TF is fixed in data units
    vs2.id = "vs2";
    vs2.type = "volslice";
    vs2.source_file = ballPath;
    vs2.has_volslice = true;
    SceneTFPoint a, b;
    a.value = 0.1; // transparent below 0.1, opaque from 0.5
    a.color[0] = 1.0f;
    b.value = 0.5;
    b.color[1] = 1.0f;
    b.color[3] = 1.0f;
    vs2.volslice.tf.points = {a, b};
    scene.nodes.push_back(vs2);
  }
  {
    SceneNode late; // a VTK volume over an all-zero placeholder a host fills later (setVolume)
    late.id = "late";
    late.type = "volume";
    late.source_file = zeroPath;
    scene.nodes.push_back(late);
  }

  std::vector<std::string> warnings;
  RealizedScene realized = cvc::gl::ariadne::realize_scene(sg, scene, "tfb", &warnings);
  for (const std::string &w : warnings)
    std::printf("     warning: %s\n", w.c_str());

  const auto node_path = [&](const std::string &id) {
    auto n = sg.getGraphics(id);
    return n ? n->stateName() : std::string();
  };
  const std::string vr = node_path("vr"), vr2 = node_path("vr2"), vs = node_path("vs"),
                    vn = node_path("vn"), plain = node_path("plain"), hm = node_path("hm");
  const std::string vrT = vr + ".transfer_function", vr2T = vr2 + ".transfer_function",
                    vsT = vs + ".transfer_function", vnT = vn + ".transfer_function",
                    plainT = plain + ".transfer_function", hmT = hm + ".transfer_function";
  const std::string vrK = vr + ".volren.volumes.0.";
  const std::string vr2K = vr2 + ".volren.volumes.0.";
  const std::string vr3 = node_path("vr3"), vs2 = node_path("vs2"), late = node_path("late");
  const std::string vr3T = vr3 + ".transfer_function", vs2T = vs2 + ".transfer_function",
                    lateT = late + ".transfer_function";
  const std::string vr3K = vr3 + ".volren.volumes.0.";

  std::printf("== legacy documents: seeded, untouched ==\n");
  std::shared_ptr<tf_binding> bvr = cvc::gl::ariadne::find_transfer_function(app, vrT);
  std::shared_ptr<tf_binding> bvs = cvc::gl::ariadne::find_transfer_function(app, vsT);
  std::shared_ptr<tf_binding> bvn = cvc::gl::ariadne::find_transfer_function(app, vnT);
  chk(bvr && bvs && bvn && cvc::gl::ariadne::find_transfer_function(app, vr2T) &&
          cvc::gl::ariadne::find_transfer_function(app, plainT) &&
          cvc::gl::ariadne::find_transfer_function(app, hmT),
      "every volume node got a binding at <node>.transfer_function");
  chk(bvr && bvr->model_path() == vrT && bvr->target() == cvc::gl::ariadne::tf_target::volren,
      "the volren binding reports its model path and target");

  cvc::tf::model m;
  chk(parse_model(val(app, vrT), m) && cvc::tf::transparent(m) && m.isocontours.size() == 1 &&
          m.isocontours[0].has_rgb && near(m.isocontours[0].g, 0.3) &&
          near(m.isocontours[0].at, 0.5),
      "volren with only an isosurface -> a transparent model with that isocontour");
  chk(parse_model(val(app, vsT), m) && !m.domain.automatic && near(m.domain.min, 0.1) &&
          near(m.domain.max, 0.9),
      "volslice with a window -> an explicit [0.1, 0.9] domain");
  chk(parse_model(val(app, plainT), m) && m.domain.automatic && !cvc::tf::transparent(m),
      "a volume with no transfer_function -> its default ramp, auto domain");

  const std::string vrIso = val(app, vrK + "isosurfaces"),
                    vrCol = val(app, vrK + "transfer_function.color");
  const std::string vr2Col = val(app, vr2K + "transfer_function.color"),
                    vr2Op = val(app, vr2K + "transfer_function.opacity");
  const std::string vsCol = val(app, vs + ".volslice.transfer_function.color"),
                    vsOp = val(app, vs + ".volslice.transfer_function.opacity"),
                    vsWin = val(app, vs + ".volslice.window");
  auto vnn = std::dynamic_pointer_cast<cvc::gl::VolumeNode>(sg.getGraphics("vn"));
  const std::vector<double> vnCol =
      vnn ? vnn->getTransferFunctionColorTable() : std::vector<double>();
  const std::vector<double> vnOp =
      vnn ? vnn->getTransferFunctionOpacityTable() : std::vector<double>();
  tick(realized, 3);
  chk(val(app, vrK + "isosurfaces") == vrIso && val(app, vrK + "transfer_function.color") == vrCol,
      "volren keys untouched by attach + ticks");
  chk(val(app, vr2K + "transfer_function.color") == vr2Col &&
          val(app, vr2K + "transfer_function.opacity") == vr2Op,
      "volren legacy points untouched");
  chk(val(app, vs + ".volslice.transfer_function.color") == vsCol &&
          val(app, vs + ".volslice.transfer_function.opacity") == vsOp &&
          val(app, vs + ".volslice.window") == vsWin,
      "volslice legacy keys untouched");
  chk(vnn && vnn->getTransferFunctionColorTable() == vnCol && vnCol.size() == 8 &&
          near(vnCol[2], 0.42),
      "VolumeNode legacy TF untouched (green 0.42 still drawn)");
  chk(range_is(app, vrT, 0.0, 1.0) && range_is(app, vsT, 0.0, 1.0) && range_is(app, vnT, 0.0, 1.0),
      "range published: volren and volslice the volume's (a window is the LUT's span, not the "
      "domain), volume data_min/max");

  // A legacy volren window is ignored by volren (auto_domain off bakes over the points' extent):
  // the seed spans what volren draws, not the window.
  chk(parse_model(val(app, vr3T), m) && !m.domain.automatic && near(m.domain.min, 0.0) &&
          near(m.domain.max, 1.0),
      "volren with a legacy window -> an explicit domain over the points' extent [0, 1]");
  // A volume with no transfer function over a degenerate (all-zero) volume: VolumeNode's ramp is
  // invisible there, but the seed is the ramp's real shape, so it re-fits to a filled volume.
  chk(parse_model(val(app, lateT), m) && m.domain.automatic && !cvc::tf::transparent(m),
      "a degenerate volume with no transfer_function -> the default ramp's shape, not transparent");
  const std::string vr3Col = val(app, vr3K + "transfer_function.color"),
                    vr3Op = val(app, vr3K + "transfer_function.opacity");
  const std::string vs2Col = val(app, vs2 + ".volslice.transfer_function.color"),
                    vs2Op = val(app, vs2 + ".volslice.transfer_function.opacity");

  // The model form is lowered at attach, before any frame.
  {
    const cvc::tf::model hot = cvc::tf::model::preset("hot");
    const std::vector<double> want = cvc::tf::color_table(hot, 0.0, 1.0);
    const std::vector<double> got =
        nums(val(app, hm + ".volren.volumes.0.transfer_function.color"));
    bool same = want.size() == got.size() && !want.empty();
    for (std::size_t i = 0; same && i < want.size(); ++i)
      same = near(want[i], got[i], 1e-9);
    chk(same, "a transfer_function: model is lowered into the volren keys at attach");
    chk(val(app, hm + ".volren.volumes.0.tf_auto_domain") == "1",
        "... with tf_auto_domain 1 (auto)");
  }

  // ── lowering a legacy seed reproduces the legacy transfer function ──────────────────────────
  std::printf("== lowering a legacy seed is lossless ==\n");
  const auto relower = [&](const std::string &tf) {
    cvc::tf::model s;
    if (!parse_model(val(app, tf), s))
      return false;
    s.resolution = s.resolution == 128 ? 129 : 128; // a different model string, the same tables
    put(app, tf, cvc::tf::to_string(s));
    tick(realized);
    return true;
  };
  chk(relower(vr2T) &&
          same_tf(nums(vr2Col), nums(vr2Op), nums(val(app, vr2K + "transfer_function.color")),
                  nums(val(app, vr2K + "transfer_function.opacity")), 0.0, 1.0),
      "volren legacy points -> model -> tables: the same TF over the data range");
  chk(relower(vsT) &&
          same_tf(nums(vsCol), nums(vsOp), nums(val(app, vs + ".volslice.transfer_function.color")),
                  nums(val(app, vs + ".volslice.transfer_function.opacity")), 0.1, 0.9) &&
          val(app, vs + ".volslice.window") == vsWin,
      "volslice legacy points -> model -> tables: the same TF over the window, the window kept");
  {
    // volren bakes the legacy points over their extent [0, 1] with tf_auto_domain 0; seeding over
    // the window [0.2, 0.8] instead would lower tables held at the window's edges.
    chk(relower(vr3T), "volren legacy window seed re-lowered");
    const std::vector<double> c = nums(val(app, vr3K + "transfer_function.color"));
    chk(same_tf(nums(vr3Col), nums(vr3Op), c, nums(val(app, vr3K + "transfer_function.opacity")),
                0.0, 1.0) &&
            c.size() >= 8 && near(c.front(), 0.0) && near(c[c.size() - 4], 1.0) &&
            val(app, vr3K + "tf_auto_domain") == "0",
        "volren legacy window: an unrelated edit lowers the TF volren drew (over [0, 1], not the "
        "window)");
  }
  chk(relower(vnT) && vnn &&
          same_tf(vnCol, vnOp, vnn->getTransferFunctionColorTable(),
                  vnn->getTransferFunctionOpacityTable(), 0.0, 1.0),
      "VolumeNode legacy points -> model -> VTK: the same TF over the data range");
  {
    chk(relower(vrT), "volren isosurface seed re-lowered");
    const std::vector<double> a = nums(vrIso), b = nums(val(app, vrK + "isosurfaces"));
    bool same = a.size() == 6 && b.size() == 6;
    for (std::size_t i = 0; same && i < 6; ++i)
      same = near(a[i], b[i], 1e-6);
    chk(same, "... the isosurface keeps its value, opacity, colour and shininess 32");
  }

  // ── a model edit is lowered once, isocontours become isosurfaces ────────────────────────────
  std::printf("== volren: edit -> one lowering ==\n");
  {
    int colorWrites = 0;
    boost::signals2::scoped_connection c =
        st(vrK + "transfer_function.color").valueChanged.connect([&colorWrites] { ++colorWrites; });
    cvc::tf::model e;
    e.color = {{0.0, 1.0f, 0.0f, 0.0f}, {1.0, 0.0f, 0.0f, 1.0f}};
    e.alpha = {{0.0, 0.0f}, {1.0, 0.5f}};
    cvc::tf::isocontour kept, bone;
    kept.id = "iso1"; // the document's isosurface: its shininess (32) is kept by id
    kept.at = 0.25;
    bone.id = "bone";
    bone.at = 0.75;
    bone.has_rgb = true;
    bone.r = 0.9f;
    bone.g = 0.8f;
    bone.b = 0.7f;
    e.isocontours = {kept, bone};
    e.normalize();
    cvc::tf::model first = e;
    first.alpha[1].a = 0.9f; // overwritten before the frame: never lowered
    put(app, vrT, cvc::tf::to_string(first));
    put(app, vrT, cvc::tf::to_string(e));
    tick(realized);
    const int afterOne = colorWrites;
    tick(realized, 2);
    chk(afterOne == 1 && colorWrites == 1, "two writes in a frame -> one lowering, none after");
    const std::vector<double> col = nums(val(app, vrK + "transfer_function.color"));
    const std::vector<double> op = nums(val(app, vrK + "transfer_function.opacity"));
    chk(same_tf(col, op, cvc::tf::color_table(e, 0.0, 1.0), cvc::tf::opacity_table(e, 0.0, 1.0),
                0.0, 1.0) &&
            near(sample(op, 2, 1, 1.0), 0.5),
        "the latest model reached volren.volumes.0.transfer_function.{color,opacity}");
    chk(val(app, vrK + "tf_auto_domain") == "1", "auto domain -> tf_auto_domain 1");
    const std::vector<double> iso = nums(val(app, vrK + "isosurfaces"));
    chk(iso.size() == 12 && near(iso[0], 0.25) && near(iso[5], 32.0) && near(iso[6], 0.75) &&
            near(iso[8], 0.9) && near(iso[11], 10.0),
        "isocontours -> isosurfaces (value, opacity, rgb, shininess: 32 kept by id, 10 new)");
    float rgb[3];
    e.isocontour_rgb(e.isocontours[0], rgb);
    chk(iso.size() == 12 && near(iso[1], 1.0) && near(iso[2], rgb[0]) && near(iso[4], rgb[2]),
        "an isocontour without overrides takes the ramp colour and opacity 1");

    cvc::tf::model x = e;
    x.domain.automatic = false;
    x.domain.min = 0.2;
    x.domain.max = 0.8;
    put(app, vrT, cvc::tf::to_string(x));
    tick(realized);
    const std::vector<double> xc = nums(val(app, vrK + "transfer_function.color"));
    const std::vector<double> xi = nums(val(app, vrK + "isosurfaces"));
    chk(val(app, vrK + "tf_auto_domain") == "0" && xc.size() >= 8 && near(xc.front(), 0.2) &&
            near(xc[xc.size() - 4], 0.8),
        "explicit domain [0.2, 0.8] -> tf_auto_domain 0, the tables span it");
    chk(xi.size() == 12 && near(xi[0], 0.2 + 0.6 * 0.25), "... and isovalues map into it");
  }

  // ── volslice: an explicit domain is the window; auto clears it again ────────────────────────
  std::printf("== volslice: domain <-> window ==\n");
  {
    cvc::tf::model a = cvc::tf::model::preset("grayscale");
    put(app, vsT, cvc::tf::to_string(a));
    tick(realized);
    const std::vector<double> c = nums(val(app, vs + ".volslice.transfer_function.color"));
    chk(val(app, vs + ".volslice.window").empty() &&
            val(app, vs + ".volslice.tf_auto_domain") == "1",
        "auto domain -> the window the binding owned is cleared, tf_auto_domain 1");
    chk(range_is(app, vsT, 0.0, 1.0) && c.size() >= 8 && near(c.front(), 0.0) &&
            near(c[c.size() - 4], 1.0),
        "... the range is the data range again and the tables span it (same frame)");
    a.domain.automatic = false;
    a.domain.min = 0.3;
    a.domain.max = 0.6;
    put(app, vsT, cvc::tf::to_string(a));
    tick(realized);
    const std::vector<double> w = nums(val(app, vs + ".volslice.window"));
    const std::vector<double> c2 = nums(val(app, vs + ".volslice.transfer_function.color"));
    chk(w.size() == 2 && near(w[0], 0.3) && near(w[1], 0.6) &&
            val(app, vs + ".volslice.tf_auto_domain") == "0" && range_is(app, vsT, 0.0, 1.0) &&
            c2.size() >= 8 && near(c2.front(), 0.3) && near(c2[c2.size() - 4], 0.6),
        "explicit domain [0.3, 0.6] -> volslice.window 0.3,0.6, the tables span it, range kept");

    // Someone else moves the window: it is theirs from now on.
    put(app, vs + ".volslice.window", "0.2,0.7");
    a.domain.min = 0.25;
    a.domain.max = 0.65;
    put(app, vsT, cvc::tf::to_string(a));
    tick(realized);
    const std::vector<double> c3 = nums(val(app, vs + ".volslice.transfer_function.color"));
    chk(val(app, vs + ".volslice.window") == "0.2,0.7" && c3.size() >= 8 &&
            near(c3.front(), 0.25) && near(c3[c3.size() - 4], 0.65),
        "a window someone else set survives a domain edit; the TF moves in data units");
    a.domain.automatic = true;
    put(app, vsT, cvc::tf::to_string(a));
    tick(realized);
    chk(val(app, vs + ".volslice.window") == "0.2,0.7",
        "... and going back to auto does not clear their window");
    put(app, vs + ".volslice.window", "");
    a.domain.automatic = false;
    put(app, vsT, cvc::tf::to_string(a));
    tick(realized);
    chk(val(app, vs + ".volslice.window").empty(),
        "... nor does the binding take the window back once they cleared it");
  }

  // ── volslice: a legacy TF is fixed in data units, a window only spans the LUT ───────────────
  std::printf("== volslice: the window is the LUT's span ==\n");
  {
    put(app, vs2 + ".volslice.window", "0.2,0.6"); // a script narrows the slices' window
    tick(realized, 2);
    chk(val(app, vs2 + ".volslice.transfer_function.color") == vs2Col &&
            val(app, vs2 + ".volslice.transfer_function.opacity") == vs2Op &&
            range_is(app, vs2T, 0.0, 1.0),
        "a runtime window does not re-lower the legacy seed (the TF stays in data units)");
    chk(relower(vs2T) &&
            same_tf(nums(vs2Col), nums(vs2Op),
                    nums(val(app, vs2 + ".volslice.transfer_function.color")),
                    nums(val(app, vs2 + ".volslice.transfer_function.opacity")), 0.0, 1.0) &&
            val(app, vs2 + ".volslice.window") == "0.2,0.6",
        "an unrelated edit lowers the same TF in data units and keeps the script's window");
  }

  // ── VolumeNode: live, and kept across a reload ──────────────────────────────────────────────
  std::printf("== VolumeNode: two-way, no clobber ==\n");
  if (vnn) {
    cvc::tf::model v;
    v.color = {{0.0, 0.0f, 1.0f, 0.0f}, {1.0, 1.0f, 0.0f, 1.0f}};
    v.alpha = {{0.0, 0.0f}, {0.5, 1.0f}, {1.0, 0.0f}};
    v.normalize();
    put(app, vnT, cvc::tf::to_string(v));
    tick(realized);
    const std::vector<double> op = vnn->getTransferFunctionOpacityTable();
    chk(op.size() == 6 && near(op[2], 0.5) && near(op[3], 1.0) && near(op[5], 0.0),
        "an edited model reaches VTK (getTransferFunctionOpacityTable reads the mapper)");
    put(app, vn + ".transfer_function.opacity", "0,0,1"); // malformed: odd count
    chk(vnn->getTransferFunctionOpacityTable() == op,
        "a malformed transfer_function.opacity is refused, VTK keeps the last good one");
    put(app, vn + ".transfer_function.opacity", "0,0,0.5,1,1,0");

    vnn->setVolume(ball(app, 17, 2.0)); // range [0, 2]
    chk(vnn->getTransferFunctionOpacityTable() == op, "setVolume keeps the authored TF");
    tick(realized);
    const std::vector<double> op2 = vnn->getTransferFunctionOpacityTable();
    chk(range_is(app, vnT, 0.0, 2.0) && op2.size() == 6 && near(op2[2], 1.0) && near(op2[4], 2.0),
        "... the binding publishes the new range and re-fits the auto-domain model to it");

    v.domain.automatic = false;
    v.domain.min = 0.0;
    v.domain.max = 1.0;
    put(app, vnT, cvc::tf::to_string(v));
    tick(realized);
    vnn->setVolume(ball(app, 17, 3.0));
    tick(realized);
    const std::vector<double> op3 = vnn->getTransferFunctionOpacityTable();
    chk(range_is(app, vnT, 0.0, 3.0) && op3.size() == 6 && near(op3[4], 1.0),
        "an explicit domain stays put across a reload");
  }
  {
    auto pn = std::dynamic_pointer_cast<cvc::gl::VolumeNode>(sg.getGraphics("plain"));
    if (pn) {
      pn->setVolume(ball(app, 17, 4.0));
      const std::vector<double> op = pn->getTransferFunctionOpacityTable();
      chk(op.size() == 4 && near(op[2], 4.0),
          "an untouched default ramp still re-fits on setVolume");
    }
  }
  {
    // A placeholder volume (all zero: VolumeNode's ramp is invisible on it) that a host fills
    // later: VolumeNode re-fits a visible ramp, and the binding, following its auto domain, must
    // lower that same ramp -- not a transparent seed that blanks the volume.
    auto ln = std::dynamic_pointer_cast<cvc::gl::VolumeNode>(sg.getGraphics("late"));
    if (!ln) {
      chk(false, "the placeholder volume node realized");
    } else {
      tick(realized, 2);
      ln->setVolume(ball(app, 17, 2.0)); // range [0, 2]
      tick(realized, 2);
      const std::vector<double> op = ln->getTransferFunctionOpacityTable();
      chk(range_is(app, lateT, 0.0, 2.0) && op.size() == 4 && near(op[0], 0.0) &&
              near(op[1], 0.0) && near(op[2], 2.0) && near(op[3], 1.0) &&
              !val(app, late + ".transfer_function.opacity").empty(),
          "a placeholder filled later (setVolume) draws the default 0 -> 1 ramp, not blank");
    }
  }

  // ── overlays: histogram at once, spectrum + tree on request ─────────────────────────────────
  std::printf("== overlays ==\n");
  if (bvr) {
    // Nothing has pumped the scheduler yet: the histogram task attach started waits on it as a
    // process, and nothing is published.
    const std::string mode = cvc::tf::threads_available() ? "threaded" : "cooperative";
    const std::vector<std::string> procs = overlay_procs(app, vrT);
    chk(procs.size() == 1 && procs[0] == "tf-overlays/" + mode + " " + vrT &&
            val(app, vrT + ".histogram").empty(),
        "the histogram is an overlay task on the app's scheduler (automatic = " + mode +
            " here), nothing published before it runs");
    int frames = 0;
    while (frames < 20000 && val(app, vrT + ".histogram").empty()) {
      pump(app);
      tick(realized);
      ++frames;
      if (val(app, vrT + ".histogram").empty())
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // a pool chunk is running
    }
    std::vector<double> bins;
    chk(cvc::tf::histogram_from_string(val(app, vrT + ".histogram"), bins) && !bins.empty() &&
            overlay_procs(app, vrT, false).empty(),
        "the histogram arrives through the scheduler without being asked for (" +
            std::to_string(frames) + " frames), its process reaped");
    put(app, vrT + ".overlay_request", "contour_spectrum,contour_tree");
    tick(realized);
    chk(val(app, vrT + ".overlay_status").rfind("computing", 0) == 0,
        "overlay_status says what is computing (" + val(app, vrT + ".overlay_status") + ")");
    const std::size_t started = overlay_procs(app, vrT).size();
    const bool waited = bvr->wait_overlays();
    chk(started == 1 && waited,
        "the request starts one task, and wait_overlays() pumps the scheduler until it finishes");
    // No tick: wait_overlays() publishes what arrived at once.
    cvc::tf::tree_overlay tree;
    cvc::tf::spectrum_overlay spec;
    const std::string status = val(app, vrT + ".overlay_status");
    chk(cvc::tf::tree_from_string(val(app, vrT + ".contour_tree"), tree) && !tree.empty(),
        "the contour tree is published after wait_overlays()");
    chk((cvc::tf::spectrum_from_string(val(app, vrT + ".contour_spectrum"), spec) &&
         !spec.empty()) ||
            status.find("unavailable") != std::string::npos,
        "the contour spectrum is published (or reported unavailable: '" + status + "')");
    chk(status.rfind("computing", 0) != 0, "overlay_status is idle again");
  }
  if (bvs) {
    bvs->wait_overlays();
    tick(realized);
    std::vector<double> bins;
    chk(cvc::tf::histogram_from_string(val(app, vsT + ".histogram"), bins) && !bins.empty(),
        "volslice's histogram is published over its window");
  }

  // ── overlays on the scheduler: a cooperative run, a binding destroyed mid-computation ───────
  std::printf("== overlays on the scheduler ==\n");
  chk(settle(app, 60000.0), "every overlay task started so far finishes once the scheduler runs");
  {
    // Cooperative (as on single-threaded wasm): a slice of each task per pumped frame.
    const std::size_t jobs0 = cvc::tf::overlay_job::instances();
    const cvc::volume big = ball(app, 41, 1.0);
    SceneGraph og(app, "tfo");
    std::shared_ptr<cvc::gl::GraphicsNode> on = og.addGraphics("coop");
    const std::string oT = on ? on->stateName("transfer_function") : std::string();
    cvc::gl::ariadne::tf_binding_options o;
    o.overlay_threading = cvc::tf::overlay_threading::cooperative;
    RealizedScene orz;
    std::shared_ptr<tf_binding> ob =
        on ? cvc::gl::ariadne::attach_transfer_function(
                 orz, on, cvc::gl::ariadne::tf_target::volren, big, o, &warnings)
           : nullptr;
    if (!ob) {
      chk(false, "a cooperative binding attaches");
    } else {
      put(app, oT + ".overlay_request", "contour_spectrum,contour_tree");
      tick(orz); // starts the spectrum and tree beside attach's histogram
      const std::vector<std::string> procs = overlay_procs(app, oT);
      chk(procs.size() == 2 && std::all_of(procs.begin(), procs.end(),
                                           [&](const std::string &n) {
                                             return n == "tf-overlays/cooperative " + oT;
                                           }),
          "cooperative: the histogram and the spectrum + tree are two cooperative tasks");
      int frames = 0, notes = 0;
      double max_ms = 0.0, total_ms = 0.0;
      std::string first, last;
      while (frames < 20000 && !overlay_procs(app, oT).empty()) {
        const double ms = pump(app);
        tick(orz);
        ++frames;
        max_ms = std::max(max_ms, ms);
        total_ms += ms;
        const std::string s = val(app, oT + ".overlay_status");
        if (frames == 1)
          first = s;
        if (s != last && !s.empty() && s.rfind("computing", 0) != 0)
          ++notes;
        last = s;
      }
      std::printf("     [cooperative] 41^3, every layer, 4 ms slices: %d frames, max pump %.2f ms, "
                  "mean pump %.2f ms, %d progress notes\n",
                  frames, max_ms, frames ? total_ms / frames : 0.0, notes);
      chk(frames > 2 && overlay_procs(app, oT, false).empty(),
          "a cooperative run completes across frames (" + std::to_string(frames) +
              "), its processes reaped");
      chk(!first.empty() && first.rfind("computing", 0) != 0,
          "after the first frame overlay_status is the task's progress note, and tick() leaves it "
          "('" +
              first + "')");
      chk(max_ms < 100.0,
          "each frame runs a slice, not the job (max pump " + std::to_string(max_ms) + " ms)");
      const cvc::tf::overlays ref =
          cvc::tf::compute_overlays(big, cvc::tf::overlay_all, o.overlays);
      cvc::tf::publish_overlays(st("tfo_ref"), ref);
      cvc::tf::tree_overlay tree;
      chk(val(app, oT + ".histogram") == val(app, "tfo_ref.histogram") &&
              val(app, oT + ".contour_spectrum") == val(app, "tfo_ref.contour_spectrum") &&
              val(app, oT + ".contour_tree") == val(app, "tfo_ref.contour_tree") &&
              val(app, oT + ".range") == val(app, "tfo_ref.range") &&
              cvc::tf::tree_from_string(val(app, oT + ".contour_tree"), tree) && !tree.empty(),
          "the published layers are exactly compute_overlays'");
      chk(val(app, oT + ".overlay_status") == ref.status,
          "the final status is compute_overlays' ('" + ref.status + "')");
      chk(cvc::tf::overlay_job::instances() == jobs0, "the finished tasks let go of their jobs");
    }
  }
  {
    // Destroyed mid-computation, in both modes: the destructor cancels; nothing lingers.
    const cvc::volume huge = ball(app, 65, 1.0);
    for (const cvc::tf::overlay_threading mode :
         {cvc::tf::overlay_threading::threaded, cvc::tf::overlay_threading::cooperative}) {
      const std::string mn =
          mode == cvc::tf::overlay_threading::threaded ? "threaded" : "cooperative";
      settle(app, 60000.0);
      const std::size_t jobs0 = cvc::tf::overlay_job::instances();
      SceneGraph dg(app, "tfd_" + mn);
      std::shared_ptr<cvc::gl::GraphicsNode> dn = dg.addGraphics("gone");
      const std::string dT = dn ? dn->stateName("transfer_function") : std::string();
      cvc::gl::ariadne::tf_binding_options o;
      o.overlay_threading = mode;
      auto rz = std::make_unique<RealizedScene>();
      std::shared_ptr<tf_binding> db =
          dn ? cvc::gl::ariadne::attach_transfer_function(
                   *rz, dn, cvc::gl::ariadne::tf_target::volren, huge, o, &warnings)
             : nullptr;
      if (!db) {
        chk(false, mn + ": a binding attaches for the teardown check");
        continue;
      }
      put(app, dT + ".overlay_request", "contour_spectrum,contour_tree");
      tick(*rz);
      for (int f = 0; f < 3; ++f) {
        pump(app);
        tick(*rz);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      const std::size_t live = overlay_procs(app, dT).size();
      chk(live >= 1 && cvc::tf::overlay_job::instances() > jobs0 &&
              val(app, dT + ".contour_tree").empty(),
          mn + ": mid-computation (" + std::to_string(live) + " live task(s), no tree yet)");
      db.reset();
      rz.reset(); // the scene goes, and its binding with it
      chk(overlay_procs(app, dT, false).empty(),
          mn + ": a binding destroyed mid-computation leaves no process on the scheduler");
      // No newer binding owns the key: the cancelled tasks' progress note goes with them.
      chk(val(app, dT + ".overlay_status").empty(),
          mn + ": ... nor its tasks' progress note in overlay_status ('" +
              val(app, dT + ".overlay_status") + "')");
      std::vector<std::string> before;
      for (const char *k :
           {"histogram", "contour_spectrum", "contour_tree", "overlay_status", "range"})
        before.push_back(val(app, dT + "." + k));
      const steady::time_point t0 = steady::now();
      while (cvc::tf::overlay_job::instances() > jobs0 && ms_since(t0) < 10000.0) {
        pump(app);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      const double released_ms = ms_since(t0);
      for (int f = 0; f < 30; ++f) { // anything still on its way would land now
        pump(app);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      chk(cvc::tf::overlay_job::instances() == jobs0,
          mn + ": its jobs are let go of (" + std::to_string(released_ms) + " ms)");
      std::vector<std::string> after;
      for (const char *k :
           {"histogram", "contour_spectrum", "contour_tree", "overlay_status", "range"})
        after.push_back(val(app, dT + "." + k));
      chk(after == before && after[2].empty() && overlay_procs(app, dT, false).empty(),
          mn + ": nothing is published for it afterwards");
    }
  }

  // ── a replaced volume: followed by identity, its old layers unpublished at once ─────────────
  std::printf("== overlays follow a replaced volume ==\n");
  {
    settle(app, 60000.0);
    // A VolumeNode bound alone; A and B are different volumes over the same [0, 1], so setVolume(B)
    // re-publishes the same data_min/data_max.
    const cvc::volume A = ball(app, 17, 1.0), B = ball(app, 9, 1.0);
    SceneGraph rg(app, "tfr");
    std::shared_ptr<cvc::gl::VolumeNode> rn = rg.addGraphics("rv", A);
    const std::string rT = rn ? rn->stateName("transfer_function") : std::string();
    RealizedScene rrz;
    std::shared_ptr<tf_binding> rb =
        rn && rn->getVolume() ? cvc::gl::ariadne::attach_transfer_function(
                                    rrz, rn, cvc::gl::ariadne::tf_target::volume, *rn->getVolume(),
                                    cvc::gl::ariadne::tf_binding_options(), &warnings)
                              : nullptr;
    if (!rb) {
      chk(false, "a volume binding attaches for the replacement checks");
    } else {
      const unsigned layers = cvc::tf::overlay_histogram | cvc::tf::overlay_contour_tree;
      const auto ref = [&](const cvc::volume &v) {
        return cvc::tf::compute_overlays(v, layers, cvc::tf::overlay_options());
      };
      // The histogram and tree published beside rT are exactly `r`'s.
      const auto shows = [&](const cvc::tf::overlays &r) {
        return !r.histogram.empty() && !r.tree.empty() &&
               val(app, rT + ".histogram") == cvc::tf::histogram_to_string(r.histogram) &&
               val(app, rT + ".contour_tree") == cvc::tf::tree_to_string(r.tree);
      };
      const auto settle_binding = [&]() {
        tick(rrz);
        rb->wait_overlays();
        tick(rrz);
      };
      put(app, rT + ".overlay_request", "contour_tree");
      settle_binding();
      const cvc::tf::overlays ra = ref(A), rbv = ref(B);
      chk(shows(ra) && ra.histogram != rbv.histogram,
          "volume A's histogram and contour tree are published (B's differ)");

      rn->setVolume(B);
      chk(val(app, rn->stateName("data_min")) == "0" && val(app, rn->stateName("data_max")) == "1",
          "setVolume(B) re-publishes the same data_min/data_max");
      tick(rrz);
      const bool cleared =
          val(app, rT + ".histogram").empty() && val(app, rT + ".contour_tree").empty();
      const std::size_t started = overlay_procs(app, rT).size();
      rb->wait_overlays();
      tick(rrz);
      chk(started == 1 && range_is(app, rT, 0.0, 1.0) && shows(rbv),
          "a replacement over the same range is followed: B's histogram and tree, recomputed");
      chk(cleared, "... and A's layers were unpublished at the replacement, before B's arrived");

      // A constant volume over another range: its tree cannot be computed, so a tree of B's left
      // published would stay beside C's range for good.
      cvc::volume C(app, cvc::dimension(9, 9, 9), cvc::Float,
                    cvc::bounding_box(-1, -1, -1, 1, 1, 1));
      for (unsigned k = 0; k < 9; ++k)
        for (unsigned j = 0; j < 9; ++j)
          for (unsigned i = 0; i < 9; ++i)
            C(i, j, k, 0.25);
      C.min(0.25);
      C.max(0.25);
      rn->setVolume(C);
      tick(rrz);
      const bool cleared_c =
          val(app, rT + ".histogram").empty() && val(app, rT + ".contour_tree").empty();
      rb->wait_overlays();
      tick(rrz);
      const std::string cst = val(app, rT + ".overlay_status");
      chk(cleared_c && val(app, rT + ".contour_tree").empty() &&
              cst.find("constant") != std::string::npos,
          "a constant replacement (another range): B's tree is unpublished, not left beside '" +
              cst + "'");

      // Two setVolume calls between ticks, the second perhaps handed the block (the address) of
      // the volume the binding last followed: it is still a new volume.
      const cvc::volume A2 = ball(app, 5, 1.0), B2 = ball(app, 5, 2.0), C2 = ball(app, 5, 3.0);
      rn->setVolume(A2);
      settle_binding();
      const cvc::volume *seen = rn->getVolume();
      int tries = 0;
      bool reused = false;
      while (!reused && tries < 1000) {
        rn->setVolume(B2);
        rn->setVolume(C2);
        ++tries;
        reused = rn->getVolume() == seen;
      }
      std::printf("     [aba] the followed volume's address %s by a later setVolume (%d pairs)\n",
                  reused ? "WAS reused" : "was not reused", tries);
      settle_binding();
      chk(range_is(app, rT, 0.0, 3.0) && shows(ref(C2)),
          "the last of two setVolume calls between ticks is followed (C2's layers, not A2's "
          "re-binned), whatever address it got");
    }
  }

  // ── overlay_status: every missing layer's reason, and a binding that stops for good ─────────
  std::printf("== overlay_status ==\n");
  {
    settle(app, 60000.0);
    // A 9 x 9 x 1 slab: the contour spectrum needs two voxels on every axis, the tree does not.
    cvc::volume slab(app, cvc::dimension(9, 9, 1), cvc::Float,
                     cvc::bounding_box(-1, -1, -0.125, 1, 1, 0.125));
    for (unsigned j = 0; j < 9; ++j)
      for (unsigned i = 0; i < 9; ++i) {
        const double x = -1.0 + 2.0 * i / 8.0, y = -1.0 + 2.0 * j / 8.0;
        slab(i, j, 0, std::max(0.0, 1.0 - std::sqrt(x * x + y * y)));
      }
    slab.min(0.0);
    slab.max(1.0);
    SceneGraph ng(app, "tfn");
    std::shared_ptr<cvc::gl::GraphicsNode> nn = ng.addGraphics("slab");
    const std::string nT = nn ? nn->stateName("transfer_function") : std::string();
    RealizedScene nrz;
    std::shared_ptr<tf_binding> nb = nn ? cvc::gl::ariadne::attach_transfer_function(
                                              nrz, nn, cvc::gl::ariadne::tf_target::volren, slab,
                                              cvc::gl::ariadne::tf_binding_options(), &warnings)
                                        : nullptr;
    if (!nb) {
      chk(false, "a binding attaches for the status checks");
    } else {
      // The editor's toggles grow the request one layer at a time.
      put(app, nT + ".overlay_request", "contour_spectrum");
      tick(nrz);
      nb->wait_overlays();
      tick(nrz);
      const std::string s1 = val(app, nT + ".overlay_status");
      put(app, nT + ".overlay_request", "contour_spectrum,contour_tree");
      tick(nrz);
      nb->wait_overlays();
      tick(nrz);
      const std::string s2 = val(app, nT + ".overlay_status");
      const std::string all = cvc::tf::compute_overlays(slab, cvc::tf::overlay_all).status;
      chk(s1.find("contour spectrum") != std::string::npos,
          "the spectrum's task says why it is unavailable ('" + s1 + "')");
      chk(s2 == all && s2.find("contour spectrum") != std::string::npos,
          "after the tree's task too, the spectrum's reason is still there: the status is "
          "compute_overlays' over every layer ('" +
              s2 + "' vs '" + all + "')");
    }
  }
  {
    // A hot reload: the new binding (here over another volume) is attached before the old one is
    // destroyed mid-computation.
    settle(app, 60000.0);
    const cvc::volume first = ball(app, 65, 1.0), second = ball(app, 17, 2.0);
    SceneGraph hg(app, "tfh");
    std::shared_ptr<cvc::gl::GraphicsNode> hn = hg.addGraphics("reload");
    const std::string hT = hn ? hn->stateName("transfer_function") : std::string();
    auto rz1 = std::make_unique<RealizedScene>();
    cvc::gl::ariadne::tf_binding_options o1;
    o1.overlay_threading = cvc::tf::overlay_threading::cooperative; // still running 3 frames on
    std::shared_ptr<tf_binding> b1 =
        hn ? cvc::gl::ariadne::attach_transfer_function(
                 *rz1, hn, cvc::gl::ariadne::tf_target::volren, first, o1, &warnings)
           : nullptr;
    if (!b1) {
      chk(false, "a binding attaches for the reload checks");
    } else {
      tick(*rz1);
      b1->wait_overlays();
      tick(*rz1);
      const bool had_histogram = !val(app, hT + ".histogram").empty();
      put(app, hT + ".overlay_request", "contour_spectrum,contour_tree");
      tick(*rz1);
      for (int f = 0; f < 3; ++f) {
        pump(app);
        tick(*rz1);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      const std::string old_note = val(app, hT + ".overlay_status");
      const std::size_t old_live = overlay_procs(app, hT).size(); // the old binding's tasks
      auto rz2 = std::make_unique<RealizedScene>();
      std::shared_ptr<tf_binding> b2 = cvc::gl::ariadne::attach_transfer_function(
          *rz2, hn, cvc::gl::ariadne::tf_target::volren, second,
          cvc::gl::ariadne::tf_binding_options(), &warnings);
      const bool cleared = val(app, hT + ".histogram").empty();
      const std::string fresh = val(app, hT + ".overlay_status");
      b1.reset();
      rz1.reset();
      chk(had_histogram && cleared,
          "a binding attached over a predecessor's layers unpublishes them (another volume's)");
      chk(old_live >= 1 && fresh.rfind("computing", 0) == 0 &&
              val(app, hT + ".overlay_status") == fresh,
          "the predecessor, destroyed mid-computation after, leaves the new binding's status as "
          "it is ('" +
              fresh + "'; its own was '" + old_note + "', " + std::to_string(old_live) +
              " live task(s))");
      if (b2) {
        tick(*rz2);
        b2->wait_overlays();
        tick(*rz2);
        const cvc::tf::overlays ref2 =
            cvc::tf::compute_overlays(second, cvc::tf::overlay_histogram);
        chk(val(app, hT + ".histogram") == cvc::tf::histogram_to_string(ref2.histogram) &&
                range_is(app, hT, 0.0, 2.0) &&
                val(app, hT + ".overlay_status").rfind("computing", 0) != 0,
            "the new binding publishes its own volume's layers, then goes idle");
      }
      b2.reset();
      rz2.reset();
    }
  }
  {
    // The node is removed mid-computation: the next tick stops the tasks and the progress note.
    settle(app, 60000.0);
    const cvc::volume huge = ball(app, 65, 1.0);
    SceneGraph xg(app, "tfx");
    std::shared_ptr<cvc::gl::GraphicsNode> xn = xg.addGraphics("removed");
    const std::string xT = xn ? xn->stateName("transfer_function") : std::string();
    auto xrz = std::make_unique<RealizedScene>();
    std::shared_ptr<tf_binding> xb = xn ? cvc::gl::ariadne::attach_transfer_function(
                                              *xrz, xn, cvc::gl::ariadne::tf_target::volren, huge,
                                              cvc::gl::ariadne::tf_binding_options(), &warnings)
                                        : nullptr;
    if (!xb) {
      chk(false, "a binding attaches for the node-removal check");
    } else {
      put(app, xT + ".overlay_request", "contour_spectrum,contour_tree");
      tick(*xrz);
      for (int f = 0; f < 3; ++f) {
        pump(app);
        tick(*xrz);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      const std::size_t live = overlay_procs(app, xT).size();
      const std::string note = val(app, xT + ".overlay_status");
      xn.reset();
      xg.removeGraphics("removed"); // the scene's reference was the last: the node is gone
      tick(*xrz);
      chk(live >= 1 && !note.empty() && overlay_procs(app, xT, false).empty() &&
              val(app, xT + ".overlay_status").empty(),
          "a node removed mid-computation: the next tick stops its " + std::to_string(live) +
              " task(s) and clears their note ('" + note + "' -> '" +
              val(app, xT + ".overlay_status") + "')");
    }
    xb.reset();
    xrz.reset();
  }

  // ── the node is gone: the binding is a no-op ────────────────────────────────────────────────
  sg.removeGraphics("vr");
  put(app, vrT, cvc::tf::to_string(cvc::tf::model::preset("cool")));
  tick(realized, 2);
  chk(true, "ticking a binding whose node was removed is a no-op (no crash)");

  // ── the fe_lab binding alone: an isovalue is not "moved" by the model's own rounding ────────
  // A signed range [-1e5, 1e5] (an SDF's): the slider's 0.37 becomes position (0.37 + 1e5) / 2e5,
  // which lowers back to 0.3699999999953434 -- 4.7e-12 off, the same isovalue. An edit elsewhere
  // in the model must keep the slider's exact value and request no remesh.
  std::printf("== fe_lab binding: position rounding ==\n");
  {
    SceneGraph cg(app, "tfc");
    std::shared_ptr<cvc::gl::GraphicsNode> cn = cg.addGraphics("sdf");
    cvc::volume sdf(app, cvc::dimension(3, 3, 3), cvc::Float,
                    cvc::bounding_box(-1, -1, -1, 1, 1, 1));
    sdf(0, 0, 0, -1e5);
    sdf(2, 2, 2, 1e5);
    sdf.min(-1e5);
    sdf.max(1e5);
    const std::string cb = cn ? cn->stateName() : std::string();
    put(app, cb + ".fe.mesh.isovalue", "0");
    cvc::gl::ariadne::tf_binding_options o;
    o.seed.alpha = {{0.0, 0.0f}, {1.0, 0.0f}};
    cvc::tf::isocontour iso;
    iso.id = "iso1";
    iso.at = 0.5;
    o.seed.isocontours = {iso};
    o.seed_is_applied = true;
    o.remesh_debounce = 0.0; // a needless remesh would be requested on the same tick
    RealizedScene cr;
    std::shared_ptr<tf_binding> cbind = cvc::gl::ariadne::attach_transfer_function(
        cr, cn, cvc::gl::ariadne::tf_target::fe_lab, sdf, o, &warnings);
    if (!cbind) {
      chk(false, "a fe_lab binding attaches to a plain node");
    } else {
      put(app, cb + ".fe.mesh.isovalue", "0.37"); // fe_controls' slider
      tick(cr);
      cvc::tf::model e;
      chk(parse_model(val(app, cb + ".transfer_function"), e) && e.isocontours.size() == 1 &&
              near(cvc::tf::to_data(e.isocontours[0].at, -1e5, 1e5), 0.37, 1e-6),
          "the slider moves the isocontour");
      e.resolution = e.resolution == 128 ? 129 : 128; // an unrelated edit
      put(app, cb + ".transfer_function", cvc::tf::to_string(e));
      tick(cr);
      chk(val(app, cb + ".fe.mesh.isovalue") == "0.37" && val(app, cb + ".fe.request").empty(),
          "an unrelated edit keeps the slider's exact 0.37 and requests no remesh (" +
              val(app, cb + ".fe.mesh.isovalue") + ")");

      // A bound past the volume's range is legal (fe_controls' sliders span -10..10 whatever the
      // data): its bar pins at that end of the axis and stands for it, so a later edit of
      // anything else neither clamps the key nor re-meshes.
      put(app, cb + ".fe.mesh.isovalue", "250000"); // past the maximum, 1e5
      tick(cr);
      cvc::tf::model p;
      chk(parse_model(val(app, cb + ".transfer_function"), p) && p.isocontours.size() == 1 &&
              p.isocontours[0].at == 1.0 && val(app, cb + ".fe.mesh.isovalue") == "250000",
          "a slider past the maximum pins the bar at the axis' end; the key keeps 250000");
      p.resolution = p.resolution == 128 ? 129 : 128; // an unrelated edit
      put(app, cb + ".transfer_function", cvc::tf::to_string(p));
      tick(cr);
      chk(val(app, cb + ".fe.mesh.isovalue") == "250000" && val(app, cb + ".fe.request").empty(),
          "an unrelated edit keeps the out-of-range 250000 and requests no remesh (" +
              val(app, cb + ".fe.mesh.isovalue") + ", request '" + val(app, cb + ".fe.request") +
              "')");
      p.isocontours[0].at = 0.75; // the bar dragged off the end
      put(app, cb + ".transfer_function", cvc::tf::to_string(p));
      tick(cr);
      chk(near(st(cb + ".fe.mesh.isovalue").value<double>(), 5e4, 1e-6) &&
              val(app, cb + ".fe.request") == "mesh",
          "dragging the bar off the end writes its value (5e4) and requests a remesh");
      put(app, cb + ".fe.request", "");
      cbind->wait_overlays();
    }
  }

  // ── fe_lab interval: two isocontours <-> fe.mesh.isovalue / isovalue2 ───────────────────────
  // The binding alone on a plain node over the density ball (range [0, 1], so a position is its
  // isovalue); fe.request stands in for fe_lab, which a test clears as fe_lab would consume it.
  std::printf("== fe_lab binding: an interval from two isocontours ==\n");
  {
    SceneGraph ig(app, "tfi");
    std::shared_ptr<cvc::gl::GraphicsNode> in = ig.addGraphics("ivl");
    const std::string ib = in ? in->stateName() : std::string();
    const std::string iT = ib + ".transfer_function", iF = ib + ".fe.";
    put(app, iF + "mesh.isovalue", "0.3");
    put(app, iF + "mesh.isovalue2", "0.6");
    put(app, iF + "mesh.region", "solid");
    cvc::volume dens = ball(app, 17, 1.0);
    cvc::gl::ariadne::tf_binding_options o;
    o.seed.alpha = {{0.0, 0.0f}, {1.0, 0.0f}};
    cvc::tf::isocontour iso;
    iso.id = "iso1";
    iso.at = 0.3;
    o.seed.isocontours = {iso};
    o.seed_is_applied = true;
    o.remesh_debounce = 0.0; // a remesh is requested on the tick that arms it
    o.fe_isocontour = "iso1";
    o.fe_isocontour2 = "iso2";
    RealizedScene ir;
    std::shared_ptr<tf_binding> ibind = cvc::gl::ariadne::attach_transfer_function(
        ir, in, cvc::gl::ariadne::tf_target::fe_lab, dens, o, &warnings);
    // The model after an editor's edit (one tick), and the position of isocontour `id` in it.
    const auto edit = [&](const std::function<void(cvc::tf::model &)> &change) {
      cvc::tf::model e;
      if (!parse_model(val(app, iT), e))
        return false;
      change(e);
      e.normalize();
      put(app, iT, cvc::tf::to_string(e));
      tick(ir);
      return true;
    };
    const auto at = [&](const std::string &id) {
      cvc::tf::model e;
      if (!parse_model(val(app, iT), e) || e.find_isocontour(id) < 0)
        return -1.0;
      return e.isocontours[std::size_t(e.find_isocontour(id))].at;
    };
    const auto count = [&]() {
      cvc::tf::model e;
      return parse_model(val(app, iT), e) ? int(e.isocontours.size()) : -1;
    };
    const auto move = [&](const std::string &id, double t) {
      return edit(
          [&](cvc::tf::model &e) { e.isocontours[std::size_t(e.find_isocontour(id))].at = t; });
    };
    const auto bounds = [&](double a, double b) {
      return near(st(iF + "mesh.isovalue").value<double>(), a) &&
             near(st(iF + "mesh.isovalue2").value<double>(), b);
    };
    const auto requested = [&]() {
      const bool r = val(app, iF + "request") == "mesh";
      put(app, iF + "request", ""); // consumed, as fe_lab would
      return r;
    };
    if (!ibind) {
      chk(false, "a fe_lab binding attaches for the interval checks");
    } else {
      tick(ir, 2);
      chk(val(app, iT + ".interval").empty() && count() == 1,
          "solid: one isocontour, <tf>.interval empty");

      put(app, iF + "mesh.region", "interval");
      tick(ir, 2);
      chk(count() == 2 && near(at("iso1"), 0.3) && near(at("iso2"), 0.6) &&
              val(app, iT + ".interval") == "iso1,iso2",
          "switching to interval with one isocontour adds iso2 at isovalue2 (0.6) and publishes "
          "<tf>.interval = iso1,iso2 ('" +
              val(app, iT + ".interval") + "')");
      chk(bounds(0.3, 0.6) && val(app, iF + "mesh.isovalue2") == "0.6" && !requested(),
          "... the bounds keep their exact values and the switch requests no remesh");

      chk(move("iso2", 0.8) && bounds(0.3, 0.8) && requested(),
          "the editor moves iso2 -> fe.mesh.isovalue2 follows (0.8), isovalue stays, a remesh");
      put(app, iF + "mesh.isovalue2", "0.7"); // fe_controls' second slider
      tick(ir);
      chk(near(at("iso2"), 0.7) && near(at("iso1"), 0.3) && !requested(),
          "the isovalue2 slider moves iso2 back (0.7), iso1 stays, no remesh (it waits for "
          "Tet-mesh)");
      chk(move("iso1", 0.5) && bounds(0.5, 0.7) && requested(),
          "the editor moves iso1 -> fe.mesh.isovalue follows (0.5), isovalue2 stays, a remesh");

      // The bars may cross: each bound keeps its own isocontour; the mesher sorts them.
      chk(move("iso1", 0.9) && bounds(0.9, 0.7) && val(app, iT + ".interval") == "iso1,iso2" &&
              requested(),
          "dragging iso1 past iso2: isovalue 0.9, isovalue2 0.7, still iso1,iso2");
      put(app, iF + "mesh.isovalue", "0.1");
      tick(ir);
      chk(near(at("iso1"), 0.1) && near(at("iso2"), 0.7) &&
              val(app, iT + ".interval") == "iso1,iso2",
          "the isovalue slider takes iso1 back across iso2 (0.1)");

      chk(edit([](cvc::tf::model &e) { e.resolution = e.resolution == 128 ? 129 : 128; }) &&
              val(app, iF + "mesh.isovalue") == "0.1" && val(app, iF + "mesh.isovalue2") == "0.7" &&
              !requested(),
          "an unrelated edit keeps both exact values and requests no remesh");

      // Deleting a bound re-adopts another isocontour, never the other bound's.
      chk(edit([](cvc::tf::model &e) {
            cvc::tf::isocontour i3;
            i3.id = "iso3";
            i3.at = 0.4;
            e.isocontours.push_back(i3);
          }) &&
              bounds(0.1, 0.7) && !requested(),
          "adding a third isocontour moves neither bound");
      chk(edit([](cvc::tf::model &e) {
            e.isocontours.erase(e.isocontours.begin() + e.find_isocontour("iso2"));
          }) &&
              bounds(0.1, 0.4) && val(app, iT + ".interval") == "iso1,iso3" && requested(),
          "deleting iso2 re-adopts the lowest other than iso1 (iso3: isovalue2 0.4)");
      chk(edit([](cvc::tf::model &e) {
            e.isocontours.erase(e.isocontours.begin() + e.find_isocontour("iso1"));
          }),
          "the editor deletes iso1, one of only two bounds");
      {
        cvc::tf::model e;
        const std::string pair = val(app, iT + ".interval");
        const std::size_t comma = pair.find(',');
        const std::string a = comma == std::string::npos ? std::string() : pair.substr(0, comma);
        chk(parse_model(val(app, iT), e) && e.isocontours.size() == 2 && !a.empty() &&
                a != "iso3" && pair == a + ",iso3" && near(at(a), 0.1) && bounds(0.1, 0.4) &&
                !requested(),
            "... a new isocontour takes its place at isovalue (0.1): two distinct ids ('" + pair +
                "'), the bounds unchanged, no remesh");
      }

      // Back to solid: the second bound is no longer followed (and stays in the model).
      put(app, iF + "mesh.region", "solid");
      tick(ir);
      chk(val(app, iT + ".interval").empty() && count() == 2,
          "switching back to solid clears <tf>.interval and keeps both isocontours");
      chk(move("iso3", 0.9) && near(st(iF + "mesh.isovalue2").value<double>(), 0.4) && !requested(),
          "... after which moving the second's bar leaves isovalue2 alone");
      put(app, iF + "mesh.isovalue2", "0.2");
      tick(ir);
      chk(near(at("iso3"), 0.9), "... and the isovalue2 slider moves no isocontour");
      ibind->wait_overlays();
    }
  }

  // ── fe_lab: isocontour <-> fe.mesh.isovalue, debounced remesh ───────────────────────────────
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  if (cvc::gl::ariadne::has_scene_node_type("fe_lab")) {
    std::printf("== fe_lab ==\n");
    SceneGraph fg(app, "tff");
    Scene fs;
    SceneNode d;
    d.id = "domain";
    d.type = "fe_lab";
    d.source_file = ballPath;
    d.props.kind = Value::Kind::Map;
    d.props.entries.emplace_back("isovalue", scalar("0.5"));
    d.props.entries.emplace_back("remesh_debounce", scalar("0.05"));
    fs.nodes.push_back(d);
    RealizedScene fr = cvc::gl::ariadne::realize_scene(fg, fs, "tff", &warnings);
    auto dn = fg.getGraphics("domain");
    if (!dn) {
      chk(false, "fe_lab realized");
    } else {
      const std::string fb = dn->stateName("fe"), fT = dn->stateName("transfer_function");
      cvc::tf::model f;
      chk(parse_model(val(app, fT), f) && f.isocontours.size() == 1 &&
              f.isocontours[0].id == "iso1" && near(f.isocontours[0].at, 0.5) &&
              range_is(app, fT, 0.0, 1.0),
          "fe_lab: the isovalue is isocontour iso1, the volume range published");
      f.isocontours[0].at = 0.7;
      put(app, fT, cvc::tf::to_string(f));
      tick(fr);
      chk(near(st(fb + ".mesh.isovalue").value<double>(), 0.7) && val(app, fb + ".request").empty(),
          "an isocontour move writes fe.mesh.isovalue; the remesh waits for the debounce");
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
      tick(fr);
      chk(val(app, fb + ".request") == "mesh", "... then requests fe.request = mesh");
      put(app, fb + ".mesh.isovalue", "0.3"); // fe_controls' slider
      tick(fr);
      chk(parse_model(val(app, fT), f) && !f.isocontours.empty() && near(f.isocontours[0].at, 0.3),
          "a slider's fe.mesh.isovalue moves the model's isocontour back");

      // An edit that leaves the isocontour alone must not touch a slider's exact isovalue, though
      // the model string keeps positions to 9 significant digits.
      const auto edit = [&](const std::function<void(cvc::tf::model &)> &change) {
        cvc::tf::model e;
        if (!parse_model(val(app, fT), e))
          return false;
        change(e);
        put(app, fT, cvc::tf::to_string(e));
        tick(fr);
        return true;
      };
      const auto unrelated = [](cvc::tf::model &e) {
        e.resolution = e.resolution == 128 ? 129 : 128;
      };
      const auto at_of = [&](const std::string &id, double &at) {
        cvc::tf::model e;
        if (!parse_model(val(app, fT), e) || e.find_isocontour(id) < 0)
          return false;
        at = e.isocontours[std::size_t(e.find_isocontour(id))].at;
        return true;
      };
      put(app, fb + ".mesh.isovalue", "0.123456789123");
      tick(fr);
      chk(edit(unrelated) && val(app, fb + ".mesh.isovalue") == "0.123456789123",
          "an unrelated edit after a slider move keeps the slider's exact isovalue (no remesh)");

      // fe.mesh.isovalue follows ONE isocontour, by id, whatever passes it.
      double at = -1.0;
      chk(edit([](cvc::tf::model &e) {
            cvc::tf::isocontour i2;
            i2.id = "iso2";
            i2.at = 0.6;
            e.isocontours.push_back(i2);
          }) &&
              val(app, fb + ".mesh.isovalue") == "0.123456789123",
          "adding a second isocontour leaves the mesh isovalue alone");
      put(app, fb + ".mesh.isovalue", "0.8"); // the slider moves iso1 past iso2
      tick(fr);
      chk(at_of("iso1", at) && near(at, 0.8) && at_of("iso2", at) && near(at, 0.6),
          "the slider moves iso1 past iso2 (iso2 is now the lowest)");
      chk(edit(unrelated) && val(app, fb + ".mesh.isovalue") == "0.8",
          "... and a later edit does not snap the mesh to iso2");
      chk(edit([](cvc::tf::model &e) {
            e.isocontours[std::size_t(e.find_isocontour("iso2"))].at = 0.2;
          }) &&
              val(app, fb + ".mesh.isovalue") == "0.8",
          "dragging iso2 in the editor does not move the mesh isovalue");
      put(app, fb + ".mesh.isovalue", "0.4");
      tick(fr);
      chk(at_of("iso1", at) && near(at, 0.4) && at_of("iso2", at) && near(at, 0.2),
          "the slider moves iso1 (by id), not the lowest isocontour");
      chk(edit([](cvc::tf::model &e) {
            e.isocontours.erase(e.isocontours.begin() + e.find_isocontour("iso1"));
          }) &&
              near(st(fb + ".mesh.isovalue").value<double>(), 0.2),
          "deleting the followed isocontour adopts the lowest remaining one");
      // Let the mesh job finish so teardown does not wait on it.
      for (int i = 0;
           i < 3000 && (val(app, fb + ".busy") != "0" || !val(app, fb + ".request").empty()); ++i) {
        tick(fr);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  // ── fe_lab with an SDF source: the volume arrives after the binding attached ─────────────────
  // fe_lab computes an SDF on its lane after realize (and a `load` brings another volume), so the
  // binding attaches with no volume: no range, no overlay. It follows each volume the loads bring:
  // <tf>.range is published, the bars move to where the keys' values sit on its range, the keys
  // keep their values and nothing re-meshes; mesh.isovalue2's default, which needs the range, is
  // set by the first load.
  if (cvc::gl::ariadne::has_scene_node_type("fe_lab")) {
    std::printf("== fe_lab: an SDF source loads after the binding attached ==\n");
    SceneGraph sg(app, "tfs");
    Scene ss;
    SceneNode d;
    d.id = "domain";
    d.type = "fe_lab";
    d.source_sdf_mesh = "stanford.bunny";
    d.sdf_dim = 16;
    d.props.kind = Value::Kind::Map;
    d.props.entries.emplace_back("interval", scalar("true"));
    d.props.entries.emplace_back("isovalue", scalar("-0.005"));
    d.props.entries.emplace_back("remesh_debounce", scalar("0.02"));
    ss.nodes.push_back(d);
    std::vector<std::string> sw;
    RealizedScene sr = cvc::gl::ariadne::realize_scene(sg, ss, "tfs", &sw);
    auto dn = sg.getGraphics("domain");
    bool no_sdf = false;
    for (const std::string &w : sw)
      no_sdf = no_sdf || w.find("CVC_ENABLE_SDF") != std::string::npos;
    if (!dn) {
      if (no_sdf)
        std::printf("     (skipped: a build without CVC_ENABLE_SDF)\n");
      else
        chk(false, "fe_lab realized an SDF source");
    } else {
      const std::string fb = dn->stateName("fe"), fT = dn->stateName("transfer_function");
      const auto num = [&](const std::string &path) {
        try {
          return cvc::state::instance(app)(path).value<double>();
        } catch (...) {
          return std::nan("");
        }
      };
      // Tick until the load job (the realize-time SDF, or a `load` request) has been applied.
      const auto load = [&] {
        for (int i = 0; i < 6000; ++i) {
          tick(sr);
          pump(app); // the load is a sliced task on the scheduler
          if (val(app, fb + ".busy") == "0" && val(app, fb + ".request").empty() &&
              val(app, fb + ".status").compare(0, 7, "loading") != 0)
            return val(app, fb + ".status").compare(0, 6, "loaded") == 0;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
      };
      // The bars iso1 / iso2 stand at the keys' values over the published range.
      const auto bars_at_keys = [&](double &lo, double &hi) {
        cvc::tf::model m;
        if (!cvc::tf::range_from_string(val(app, fT + ".range"), lo, hi) ||
            !parse_model(val(app, fT), m) || m.find_isocontour("iso1") < 0 ||
            m.find_isocontour("iso2") < 0)
          return false;
        const auto at = [&](const char *id) {
          return m.isocontours[std::size_t(m.find_isocontour(id))].at;
        };
        const auto pos = [&](double v) {
          return std::clamp(cvc::tf::to_normalized(v, lo, hi), 0.0, 1.0);
        };
        return near(at("iso1"), pos(num(fb + ".mesh.isovalue")), 1e-9) &&
               near(at("iso2"), pos(num(fb + ".mesh.isovalue2")), 1e-9);
      };
      chk(val(app, fT + ".range").empty() && val(app, fb + ".mesh.isovalue2").empty() &&
              val(app, fb + ".mesh.region") == "interval",
          "before its SDF is computed: no range published, mesh.isovalue2's default not set yet");
      const bool loaded = load();
      double lo = 0.0, hi = 0.0;
      const double iso2 = num(fb + ".mesh.isovalue2");
      chk(loaded && bars_at_keys(lo, hi) && lo < -0.005 && -0.005 < hi &&
              num(fb + ".mesh.isovalue") == -0.005 && iso2 < -0.005 && iso2 > lo,
          "the SDF loaded: its range published, isovalue -0.005 kept, isovalue2's default below it "
          "(" +
              val(app, fb + ".mesh.isovalue2") + "), both bars at the keys' values (status '" +
              val(app, fb + ".status") + "')");
      put(app, fb + ".source.dim", "24");
      put(app, fb + ".request", "load");
      const bool reloaded = load();
      double lo2 = 0.0, hi2 = 0.0;
      chk(reloaded && bars_at_keys(lo2, hi2) && (lo2 != lo || hi2 != hi) &&
              num(fb + ".mesh.isovalue") == -0.005 && num(fb + ".mesh.isovalue2") == iso2,
          "a load of another SDF (24^3): the new range published, the keys kept, the bars moved to "
          "them on it");
      std::this_thread::sleep_for(std::chrono::milliseconds(60)); // past the remesh debounce
      tick(sr, 3);
      chk(val(app, fb + ".request").empty() && val(app, fb + ".busy") == "0",
          "no load re-meshed (a range moves the bars, never the keys)");
    }
  }

  std::remove(ballPath.c_str());
  std::remove(zeroPath.c_str());
  std::printf("%s (%d failure%s)\n", fails ? "FAILED" : "OK", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
