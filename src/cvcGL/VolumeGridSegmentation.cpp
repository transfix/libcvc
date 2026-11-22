/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeGridSegmentation. See inc/cvc/gl/VolumeGridSegmentation.h.

#include <algorithm>
#include <boost/any.hpp>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/job_keys.h>
#include <cvc/core/param_table.h>
#include <cvc/core/result_job.h>
#include <cvc/core/sliced_task.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeGridSegmentation.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/image/image.h>
#include <cvc/state/state.h>
#include <cvc/volume/segmentation_params.h>
#include <sstream>
#include <vtkActor.h>

namespace cvc {
namespace gl {

namespace {

// How often the labels so far are copied while the fronts advance.
constexpr double kLiveSeconds = 0.25;

std::string num(double d) { return cvc::param_detail::format_double(d); }

void put(cvc::state &s, const std::string &v) {
  if (s.value() != v)
    s.value(v);
}

// The run, shared with the node for its live labels (the job owns one reference, the node the
// other, so the labels so far stay readable however the job lets go of it).
struct SharedSegmentStepper {
  std::shared_ptr<cvc::volume_grid_segment_stepper> s;
  bool step(std::size_t budget) { return s->step(budget); }
  double progress() const { return s->progress(); }
  const char *phase() const { return s->phase(); }
  std::string status() const { return s->status(); }
  bool done() const { return s->done(); }
  void cancel() { s->cancel(); }
  cvc::volume_grid_segment_result take() { return s->take(); }
};

// A plane's label layer: see-through, so it draws in the translucent pass over its plane.
class LabelLayerNode : public GeometryNode {
public:
  LabelLayerNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
      : GeometryNode(ctx, statePath, name) {}
  void seeThrough() {
    if (auto *actor = vtkActor::SafeDownCast(getProp()))
      actor->SetForceTranslucent(true);
  }
};

// The quad over a plane's voxels, half a voxel past the outer ones (as VolumeSliceNode draws it,
// so each texel lies on its voxel).
cvc::geometry plane_quad(const cvc::slice_grid &g, cvc::slice_axis axis, std::uint64_t index) {
  const int a = static_cast<int>(axis);
  int r = 0, u = 1;
  cvc::slice_plane_axes(axis, r, u);
  const double plane = g.plane(axis, index);
  const double r0 = g.lo(r) - 0.5 * g.spacing(r), r1 = g.hi(r) + 0.5 * g.spacing(r);
  const double u0 = g.lo(u) - 0.5 * g.spacing(u), u1 = g.hi(u) + 0.5 * g.spacing(u);
  cvc::geometry quad;
  const double corners[4][2] = {{r0, u0}, {r1, u0}, {r1, u1}, {r0, u1}};
  const double uv[4][2] = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  for (int c = 0; c < 4; ++c) {
    cvc::geometry::point_t p{{0.0, 0.0, 0.0}};
    p[a] = plane;
    p[r] = corners[c][0];
    p[u] = corners[c][1];
    quad.points().push_back(p);
    cvc::geometry::vector_t n{{0.0, 0.0, 0.0}};
    n[a] = 1.0;
    quad.normals().push_back(n);
    quad.uvs().push_back({{uv[c][0], uv[c][1]}});
  }
  quad.tris().push_back({{0, 1, 2}});
  quad.tris().push_back({{0, 2, 3}});
  return quad;
}

std::string rows_signature(const std::vector<cvc::segment_class_row> &rows) {
  std::ostringstream s;
  for (const cvc::segment_class_row &r : rows)
    s << r.color[0] << ',' << r.color[1] << ',' << r.color[2] << ',' << r.visible << ';';
  return s.str();
}

} // namespace

VolumeGridSegmentation::VolumeGridSegmentation(VolumeGridNode &grid) : m_grid(grid) {
  cvc::state &seg = m_grid.getState("grid.segment");
  cvc::params_describe_state<cvc::seeded_segmentation_params>(seg);
  cvc::params_describe_state<cvc::volume_grid_segment_view_params>(seg);
  cvc::params_describe_state<cvc::class_surface_params>(seg("mesh"));
  m_keys = std::make_unique<cvc::job_keys>(m_grid.app().root(), m_grid.getState("grid").fullName());
}

VolumeGridSegmentation::~VolumeGridSegmentation() {
  if (m_job)
    m_job->cancel();
  if (m_task >= 0)
    cvc::cancel_sliced_job(m_grid.app(), m_task);
}

void VolumeGridSegmentation::status(const std::string &s) { m_keys->write("status", s); }

std::shared_ptr<GeometryNode> VolumeGridSegmentation::labelsOverlay(cvc::slice_axis a) const {
  return m_layers[static_cast<int>(a)];
}

std::shared_ptr<const cvc::volume> VolumeGridSegmentation::shownLabels() const {
  if (m_job && m_live && m_view.live)
    return m_live;
  return m_labels;
}

void VolumeGridSegmentation::seed() {
  if (m_seeded)
    return;
  m_seeded = true;
  cvc::state &seg = m_grid.getState("grid.segment");
  cvc::params_from_state<cvc::seeded_segmentation_params>(seg);
  const auto v = cvc::params_from_state<cvc::volume_grid_segment_view_params>(seg);
  if (v.ok())
    m_view = v.value;
  cvc::params_from_state<cvc::class_surface_params>(seg("mesh"));
  if (seg("rows").value().empty())
    put(seg("rows"), "0");
  if (seg("warnings").value().empty())
    put(seg("warnings"), "0");
  cvc::state &file = m_grid.getState("grid.files.segments"); // what the tab's Save writes
  if (file.value().empty())
    put(file, "segments.rawv");
}

void VolumeGridSegmentation::keyChanged(const std::string &key) {
  if (key == "rows" || key.rfind("rows.", 0) == 0)
    ++m_rowsVersion;
  else
    ++m_viewVersion;
}

bool VolumeGridSegmentation::request(const std::string &cmd, const std::string &arg) {
  (void)arg;
  if (cmd == "segment") {
    if (m_job)
      status("segmenting already: stop it first, or wait for it");
    else
      launch();
    return true;
  }
  if (cmd == "stop") {
    if (!m_job)
      return false;
    m_job->cancel();
    m_keys->stopping();
    return true;
  }
  return false;
}

void VolumeGridSegmentation::launch() {
  const std::shared_ptr<cvc::slice_source> src = m_grid.source();
  if (!src) {
    status("error: segment: there is no volume yet");
    return;
  }
  const std::shared_ptr<const cvc::volume> vol =
      src->in_memory_volume() ? m_grid.histogramVolume() : nullptr;
  if (!vol) {
    status("error: segment: the volume is read plane by plane; a segmentation needs it in memory");
    return;
  }
  cvc::state &seg = m_grid.getState("grid.segment");
  // The shown classes seed the run: the grid's variable and timestep.
  const int var = m_grid.editor().shown_variable(), time = m_grid.editor().shown_timestep();
  put(seg("variable"), std::to_string(var));
  put(seg("timestep"), std::to_string(time));
  const auto sp = cvc::params_from_state<cvc::seeded_segmentation_params>(seg);
  const auto vp = cvc::params_from_state<cvc::volume_grid_segment_view_params>(seg);
  const auto cp = cvc::params_from_state<cvc::class_surface_params>(seg("mesh"));
  std::string problems;
  if (!sp.ok())
    problems = "segment." + sp.message();
  if (!vp.ok())
    problems += (problems.empty() ? "" : "; ") + ("segment." + vp.message());
  if (!cp.ok())
    problems += (problems.empty() ? "" : "; ") + ("segment.mesh." + cp.message());
  if (!problems.empty()) {
    status("error: " + problems);
    return;
  }
  m_view = vp.value;
  m_runSeeds = m_grid.editor().classes();
  m_runVar = var;
  m_runTime = time;
  // The classes' colours while it marches: the result's rows as they will be.
  {
    cvc::seeded_segmentation names(m_grid.app());
    for (const cvc::point_class &c : m_runSeeds.classes)
      if (c.variable == var && c.timestep == time)
        names.class_names.push_back(c.name);
    if (sp.value.automatic == cvc::seed_mode::critical_points) {
      names.class_names.push_back("feature");
      names.class_names.push_back("background");
    }
    m_liveRows = cvc::segment_class_rows(m_runSeeds, var, time, names);
  }
  m_stepper = std::make_shared<cvc::volume_grid_segment_stepper>(
      *vol, m_runSeeds, sp.value, cp.value, vp.value.make_surfaces,
      vp.value.live ? kLiveSeconds : -1.0);
  m_job =
      cvc::make_result_job(std::make_unique<SharedSegmentStepper>(SharedSegmentStepper{m_stepper}));
  m_live.reset();
  m_liveSeen = 0;
  m_keys->started("segmenting...");
  cvc::sliced_task_options o;
  o.kind = "volume_grid";
  o.subject = "segment " + m_grid.getName();
  o.owner = "volume_grid:" + m_grid.getState("grid").fullName();
  std::string why;
  m_task = cvc::schedule_sliced_job(m_grid.app(), m_job, o, &why);
  if (m_task < 0) {
    m_job.reset();
    m_stepper.reset();
    m_keys->finished("error: the job could not start: " + why);
  } else {
    m_keys->pid(m_task);
  }
}

void VolumeGridSegmentation::publishInfo(const cvc::seeded_segmentation &s, std::size_t triangles) {
  cvc::state &info = m_grid.getState("grid.segment.info");
  std::size_t labelled = 0;
  for (std::size_t c : s.class_voxels)
    labelled += c;
  put(info("classes"), std::to_string(s.class_names.size()));
  put(info("labelled"), std::to_string(labelled));
  put(info("border"), std::to_string(s.border.size()));
  put(info("automatic_seeds"), std::to_string(s.automatic_seeds));
  put(info("classified_seeds"), std::to_string(s.classified_seeds));
  put(info("t_low_normalized"), num(s.t_low_normalized));
  put(info("t_high_normalized"), num(s.t_high_normalized));
  put(info("triangles"), std::to_string(triangles));
  cvc::state &w = m_grid.getState("grid.segment.warnings");
  for (std::size_t i = 0; i < s.warnings.size(); ++i)
    put(w(std::to_string(i)), s.warnings[i]);
  for (const std::string &child : w.childNames()) {
    long long i = 0;
    if (cvc::param_detail::parse_integer(child, i) &&
        i >= static_cast<long long>(s.warnings.size()))
      w.removeChild(child);
  }
  put(w, std::to_string(s.warnings.size()));
}

void VolumeGridSegmentation::apply(cvc::volume_grid_segment_result r) {
  cvc::seeded_segmentation &s = r.segmentation;
  m_rows = cvc::segment_class_rows(m_runSeeds, m_runVar, m_runTime, s, r.surfaces);
  cvc::write_segment_rows(m_grid.getState("grid.segment.rows"), m_rows);
  m_rowsSeen = m_rowsVersion; // the rows written are the rows read

  // The data channel: the labels, each class's voxels (named after it), the border, the surfaces.
  m_labels = std::make_shared<const cvc::volume>(s.labels);
  m_grid.getState("grid.segment.labels").data(boost::any(m_labels));
  auto classes = std::make_shared<std::vector<cvc::volume>>(std::move(s.classes));
  for (std::size_t c = 0; c < classes->size() && c < s.class_names.size(); ++c)
    (*classes)[c].desc(s.class_names[c]);
  m_grid.getState("grid.segment.classes")
      .data(boost::any(std::shared_ptr<const std::vector<cvc::volume>>(classes)));
  cvc::point_class border;
  border.name = "border";
  border.variable = m_runVar;
  border.timestep = m_runTime;
  border.points.reserve(3 * s.border.size());
  for (const auto &b : s.border)
    border.points.insert(border.points.end(), {b[0], b[1], b[2]});
  auto pc = std::make_shared<cvc::point_classes>();
  pc->classes.push_back(std::move(border));
  m_grid.getState("grid.segment.border")
      .data(boost::any(std::shared_ptr<const cvc::point_classes>(pc)));

  // The surfaces: one node per class that has one, in the 3D view's group.
  for (const auto &n : m_classNodes)
    if (n && m_segments)
      m_segments->removeGraphicsChild(n);
  m_classNodes.assign(m_rows.size(), nullptr);
  for (std::size_t k = 0; k < m_surfaceKeys; ++k)
    m_grid.getState("grid.segment.surfaces." + std::to_string(k)).data(boost::any());
  m_surfaceKeys = 0;
  for (std::size_t q = 0; q < r.surfaces.labels.size(); ++q) {
    const int label = r.surfaces.labels[q];
    if (label < 0 || static_cast<std::size_t>(label) >= m_rows.size())
      continue;
    const cvc::geometry &g = r.surfaces.surfaces[q];
    m_grid.getState("grid.segment.surfaces." + std::to_string(label)).data(boost::any(g));
    m_surfaceKeys = std::max(m_surfaceKeys, static_cast<std::size_t>(label) + 1);
    if (!m_segments && m_grid.surfaces())
      m_segments = m_grid.surfaces()->addGraphicsChild<NullGraphicNode>("segments");
    if (!m_segments)
      continue;
    std::shared_ptr<GeometryNode> node =
        m_segments->createChild<GeometryNode>("class_" + std::to_string(label), g);
    if (!node)
      continue;
    node->setUseSingleColor(true);
    m_classNodes[static_cast<std::size_t>(label)] = node;
  }
  m_surfaceSignature.clear();
  m_layerSignature.fill(std::string());
  publishInfo(s, r.surfaces.triangle_count());

  std::size_t labelled = 0;
  for (std::size_t c : s.class_voxels)
    labelled += c;
  std::string summary = "segmented: " + std::to_string(s.class_names.size()) + " classes, " +
                        std::to_string(labelled) + " voxels labelled";
  if (!r.surfaces.labels.empty())
    summary += ", " + std::to_string(r.surfaces.labels.size()) + " surfaces (" +
               std::to_string(r.surfaces.triangle_count()) + " triangles)";
  const std::size_t warnings = s.warnings.size() + r.surfaces.warnings.size();
  if (warnings)
    summary += " (" + std::to_string(warnings) + " warning" + (warnings == 1 ? "" : "s") + ")";
  m_keys->finished(summary);
}

void VolumeGridSegmentation::volumeChanged() {
  if (!m_job && !m_labels && m_rows.empty())
    return;
  clear();
}

void VolumeGridSegmentation::clear() {
  if (m_job) {
    m_job->cancel();
    if (m_task >= 0)
      cvc::cancel_sliced_job(m_grid.app(), m_task);
    m_job.reset();
    m_stepper.reset();
    m_task = -1;
    m_keys->finished("stopped: the volume changed");
  }
  m_live.reset();
  m_labels.reset();
  m_rows.clear();
  cvc::write_segment_rows(m_grid.getState("grid.segment.rows"), m_rows);
  m_rowsSeen = m_rowsVersion;
  for (const char *k : {"labels", "classes", "border"})
    m_grid.getState(std::string("grid.segment.") + k).data(boost::any());
  for (std::size_t k = 0; k < m_surfaceKeys; ++k)
    m_grid.getState("grid.segment.surfaces." + std::to_string(k)).data(boost::any());
  m_surfaceKeys = 0;
  for (const auto &n : m_classNodes)
    if (n && m_segments)
      m_segments->removeGraphicsChild(n);
  m_classNodes.clear();
  cvc::state &info = m_grid.getState("grid.segment.info");
  for (const std::string &child : info.childNames())
    put(info(child), "");
  m_surfaceSignature.clear();
  m_layerSignature.fill(std::string());
}

void VolumeGridSegmentation::ensureLayers() {
  for (cvc::slice_axis a : {cvc::slice_axis::x, cvc::slice_axis::y, cvc::slice_axis::z}) {
    const int ax = static_cast<int>(a);
    if (m_layers[ax])
      continue;
    const std::shared_ptr<VolumeSliceNode> s = m_grid.slice(a);
    if (!s)
      continue;
    std::shared_ptr<LabelLayerNode> g = s->addGraphicsChild<LabelLayerNode>("labels");
    if (!g)
      continue;
    g->setUseSingleColor(false);
    g->setAmbient(1.0); // unlit: the texture is the colour
    g->setDiffuse(0.0);
    g->setSpecular(0.0);
    g->setTextureInterpolate(false); // each voxel a sharp square, as the plane's
    // In front of the plane (pushed back by 2), behind its points and lines (not pushed).
    g->setDepthOffset(-1.0);
    g->seeThrough();
    g->setVisible(false);
    m_layers[ax] = g;
  }
}

void VolumeGridSegmentation::readView() {
  const auto v = cvc::params_from_state<cvc::volume_grid_segment_view_params>(
      m_grid.getState("grid.segment"), m_view);
  if (!v.ok()) {
    status("error: segment." + v.message());
    return;
  }
  m_view = v.value;
}

void VolumeGridSegmentation::readRows() {
  std::vector<std::string> problems;
  cvc::read_segment_rows(m_grid.getState("grid.segment.rows"), m_rows, &problems);
  if (!problems.empty())
    status("error: " + problems.front());
}

bool VolumeGridSegmentation::syncLayers() {
  const std::shared_ptr<const cvc::volume> lab = shownLabels();
  const bool live = lab && lab == m_live;
  const std::vector<cvc::segment_class_row> &rows = live ? m_liveRows : m_rows;
  if (lab)
    ensureLayers();
  const std::string common = std::to_string(reinterpret_cast<std::uintptr_t>(lab.get())) + '|' +
                             num(m_view.opacity) + '|' + (m_view.show ? "1" : "0") + '|' +
                             rows_signature(rows);
  bool changed = false;
  for (cvc::slice_axis a : {cvc::slice_axis::x, cvc::slice_axis::y, cvc::slice_axis::z}) {
    const int ax = static_cast<int>(a);
    const std::shared_ptr<GeometryNode> &layer = m_layers[ax];
    if (!layer)
      continue;
    const std::shared_ptr<VolumeSliceNode> plane = m_grid.slice(a);
    const std::shared_ptr<const cvc::volume_slice_data> sd = plane ? plane->slice() : nullptr;
    const std::shared_ptr<cvc::slice_source> src = plane ? plane->source() : nullptr;
    bool want = m_view.show && lab && sd && src;
    if (want) {
      const cvc::slice_grid &g = src->grid();
      want = lab->XDim() == g.dim.xdim && lab->YDim() == g.dim.ydim && lab->ZDim() == g.dim.zdim;
    }
    const std::string sig = want ? common + '|' + std::to_string(static_cast<int>(sd->axis)) + ',' +
                                       std::to_string(sd->index)
                                 : std::string("hidden");
    if (sig == m_layerSignature[ax])
      continue;
    m_layerSignature[ax] = sig;
    changed = true;
    if (!want) {
      layer->setVisible(false);
      continue;
    }
    const cvc::volume_slice_data ls = cvc::extract_slice(*lab, sd->axis, sd->index);
    std::vector<std::uint8_t> px = cvc::color_labels(ls, rows, m_view.opacity);
    layer->setGeometry(plane_quad(src->grid(), sd->axis, sd->index));
    const cvc::image img(static_cast<int>(ls.width), static_cast<int>(ls.height),
                         cvc::image::pixel_format::RGBA, cvc::image::data_type::u8, px.data());
    layer->setTexture(img);
    layer->setVisible(true);
    ++m_layerUploads;
  }
  if (changed)
    if (SceneGraph *sg = m_grid.getSceneGraph())
      sg->requestRender();
  return changed;
}

void VolumeGridSegmentation::syncSurfaces() {
  const std::string sig = std::string(m_view.show_surfaces ? "1" : "0") + '|' +
                          num(m_view.surface_opacity) + '|' + rows_signature(m_rows) + '|' +
                          std::to_string(m_classNodes.size());
  if (sig == m_surfaceSignature)
    return;
  m_surfaceSignature = sig;
  for (std::size_t c = 0; c < m_classNodes.size() && c < m_rows.size(); ++c) {
    const std::shared_ptr<GeometryNode> &n = m_classNodes[c];
    if (!n)
      continue;
    const cvc::segment_class_row &r = m_rows[c];
    n->setColor(r.color[0], r.color[1], r.color[2]);
    n->setOpacity(m_view.surface_opacity);
    n->setVisible(m_view.show_surfaces && r.visible);
  }
  if (SceneGraph *sg = m_grid.getSceneGraph())
    sg->requestRender();
}

bool VolumeGridSegmentation::tick() {
  if (m_job) {
    m_keys->progress(*m_job);
    if (m_view.live && m_stepper) {
      const std::uint64_t v = m_stepper->live_version();
      if (v != m_liveSeen) {
        m_liveSeen = v;
        m_live = m_stepper->live_labels();
      }
    }
    if (m_job->done()) {
      // A stop or a failure keeps what the node had. done() is true at once after a cancel, while
      // a pool chunk may still be inside a step, so a stop is reported without asking the job.
      std::shared_ptr<cvc::result_job<cvc::volume_grid_segment_result>> job = std::move(m_job);
      m_stepper.reset();
      m_task = -1;
      m_live.reset();
      if (job->failed())
        m_keys->finished(*job);
      else if (job->has_result())
        apply(job->take_result());
      else
        m_keys->finished(std::string("cancelled"));
    }
  }
  if (m_viewVersion != m_viewSeen) {
    m_viewSeen = m_viewVersion;
    readView();
  }
  if (m_rowsVersion != m_rowsSeen) {
    m_rowsSeen = m_rowsVersion;
    readRows();
  }
  // The thresholds' sliders span the volume's data range, or GenSeg's 0..255 scale.
  if (const std::shared_ptr<cvc::slice_source> src = m_grid.source()) {
    cvc::state &seg = m_grid.getState("grid.segment");
    bool normalized = false;
    cvc::param_detail::parse_bool(seg("normalized_thresholds").value(), normalized);
    const std::string lo = normalized ? "0" : num(src->min());
    const std::string hi = normalized ? "255" : num(src->max());
    if (lo + ',' + hi != m_rangeSignature) {
      m_rangeSignature = lo + ',' + hi;
      for (const char *k : {"t_low", "t_high"}) {
        seg(k).property("min", lo);
        seg(k).property("max", hi);
      }
    }
  }
  const bool changed = syncLayers();
  syncSurfaces();
  return changed;
}

} // namespace gl
} // namespace cvc
