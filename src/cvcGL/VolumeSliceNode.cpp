/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeSliceNode -- one axis-aligned plane of a volume as a textured quad. See the header.

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/param_table.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/state/state.h>
#include <limits>
#include <vtkActor.h>
#include <vtkMapper.h>

namespace cvc {
namespace gl {

namespace {

std::string num(double d) { return cvc::param_detail::format_double(d); }

// The keys the node publishes itself: writing them must not count as a change to draw.
bool published(const std::string &child) {
  return child == "slice.count" || child == "slice.position" || child == "slice.status";
}

} // namespace

VolumeSliceNode::VolumeSliceNode(cvc::app &ctx, const std::string &statePath,
                                 const std::string &name)
    : GeometryNode(ctx, statePath, name) {
  // Unlit: the texture is the colour (white, ambient only, under it).
  setUseSingleColor(true);
  setColor(1.0, 1.0, 1.0);
  setAmbient(1.0);
  setDiffuse(0.0);
  setSpecular(0.0);
  cvc::params_describe_state<cvc::volume_slice_params>(getState("slice"));
}

VolumeSliceNode::~VolumeSliceNode() = default;

void VolumeSliceNode::setVolume(const cvc::volume &vol) {
  setSource(std::make_shared<cvc::slice_source>(vol));
}

void VolumeSliceNode::setSource(std::shared_ptr<cvc::slice_source> src) {
  m_source = std::move(src);
  ++m_sourceVersion;
  tick(); // the quad exists at once, so the scene frames on it
}

void VolumeSliceNode::handleStateChanged(const std::string &childState) {
  if (childState == "transfer_function") {
    ++m_tfVersion;
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  } else if (childState.rfind("slice.", 0) == 0 && !published(childState)) {
    ++m_keysVersion;
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  }
  GeometryNode::handleStateChanged(childState);
}

void VolumeSliceNode::publishInfo() {
  const cvc::slice_grid &g = m_source->grid();
  getState("info.dims")
      .value(std::to_string(g.dim.xdim) + " " + std::to_string(g.dim.ydim) + " " +
             std::to_string(g.dim.zdim));
  getState("info.min").value(num(m_source->min()));
  getState("info.max").value(num(m_source->max()));
  getState("info.span")
      .value(num(g.spacing(0)) + " " + num(g.spacing(1)) + " " + num(g.spacing(2)));
  getState("info.box")
      .value(num(g.box.minx) + " " + num(g.box.miny) + " " + num(g.box.minz) + " " +
             num(g.box.maxx) + " " + num(g.box.maxy) + " " + num(g.box.maxz));
  getState("info.file").value(m_source->file());
}

void VolumeSliceNode::placeQuad(const cvc::slice_grid &g, const cvc::volume_slice_data &s) {
  const int a = static_cast<int>(s.axis);
  int r = 0, u = 1;
  cvc::slice_plane_axes(s.axis, r, u);
  const double plane = g.plane(s.axis, s.index);
  // Half a voxel past the outer voxel centres on each side: each texel is centred on its voxel.
  const double r0 = g.lo(r) - 0.5 * g.spacing(r), r1 = g.hi(r) + 0.5 * g.spacing(r);
  const double u0 = g.lo(u) - 0.5 * g.spacing(u), u1 = g.hi(u) + 0.5 * g.spacing(u);
  cvc::geometry quad;
  const double corners[4][2] = {{r0, u0}, {r1, u0}, {r1, u1}, {r0, u1}};
  const double uv[4][2] = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  for (int c = 0; c < 4; ++c) {
    cvc::geometry::point_t p;
    p[a] = plane;
    p[r] = corners[c][0];
    p[u] = corners[c][1];
    quad.points().push_back(p);
    cvc::geometry::vector_t n = {0.0, 0.0, 0.0};
    n[a] = 1.0;
    quad.normals().push_back(n);
    quad.uvs().push_back({uv[c][0], uv[c][1]});
  }
  quad.tris().push_back({0, 1, 2});
  quad.tris().push_back({0, 2, 3});
  setGeometry(quad);
  m_quadAxis = a;
  m_quadIndex = s.index;
}

void VolumeSliceNode::applyOffset(double offset) {
  // The quad pushed back (positive GL polygon-offset values are farther), so the overlays drawn
  // in its plane win the depth test.
  if (auto *actor = vtkActor::SafeDownCast(getProp()))
    if (vtkMapper *m = actor->GetMapper())
      m->SetRelativeCoincidentTopologyPolygonOffsetParameters(0.5 * offset, offset);
  m_offset = offset;
}

bool VolumeSliceNode::tick() {
  if (!m_source)
    return false;
  const std::uint64_t kv = m_keysVersion.load(), tv = m_tfVersion.load();
  if (m_drawn && kv == m_drawnKeys && tv == m_drawnTf && m_sourceVersion == m_drawnSource)
    return false;
  const bool new_source = m_sourceVersion != m_drawnSource;
  m_drawnKeys = kv;
  m_drawnTf = tv;
  m_drawnSource = m_sourceVersion;
  if (new_source)
    publishInfo();

  // The parameters: a key that does not parse keeps what was drawn last.
  cvc::state &keys = getState("slice");
  const cvc::param_result<cvc::volume_slice_params> read =
      cvc::params_from_state<cvc::volume_slice_params>(keys, m_params);
  std::string status = read.ok() ? std::string() : read.message();
  cvc::volume_slice_params p = read.value;
  if (!read.ok() && p.index < -1)
    p.index = m_params.index;
  const cvc::slice_grid &g = m_source->grid();
  const std::uint64_t n = g.count(p.axis);
  const std::int64_t last = static_cast<std::int64_t>(n) - 1;
  const std::int64_t idx = p.index < 0 ? last / 2 : std::min(p.index, last);
  if (idx != p.index)
    keys("index").value(std::to_string(idx)); // -1 resolved, or clamped to the last plane
  p.index = idx;
  keys("index").property("max", std::to_string(last));
  if (keys("count").value() != std::to_string(n))
    keys("count").value(std::to_string(n));
  const std::string position = num(g.plane(p.axis, static_cast<std::uint64_t>(idx)));
  if (keys("position").value() != position)
    keys("position").value(position);

  // The transfer function: parsed again only when its string changed.
  const std::string text = getState("transfer_function").value();
  bool recolour = false;
  if (text != m_modelText) {
    cvc::tf::model m;
    std::string err;
    if (cvc::tf::from_string(text, m, &err)) {
      m_model = m;
      m_modelText = text;
      recolour = true;
    } else {
      status += std::string(status.empty() ? "" : "; ") + "transfer_function: " + err;
    }
  }

  std::shared_ptr<const cvc::volume_slice_data> s;
  try {
    s = m_source->slice(p.axis, static_cast<std::uint64_t>(idx));
  } catch (const std::exception &e) {
    status += std::string(status.empty() ? "" : "; ") + e.what();
  }
  if (keys("status").value() != status)
    keys("status").value(status);
  if (!s)
    return false;

  const bool same_plane =
      m_drawn && s == m_slice && m_params.opaque == p.opaque && !recolour && !new_source;
  bool changed = false;
  if (!same_plane) {
    std::vector<std::uint8_t> px =
        cvc::color_slice(*s, m_model, m_source->min(), m_source->max(), p.opaque);
    const int w = static_cast<int>(s->width), h = static_cast<int>(s->height);
    if (!m_image.empty() && m_image.width() == w && m_image.height() == h && hasTexture()) {
      std::memcpy(m_image.storage().get(), px.data(), px.size()); // the texture's own bytes
      texture_modified();
    } else {
      m_image =
          cvc::image(w, h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8, px.data());
      setTexture(m_image);
    }
    m_pixels = std::move(px);
    ++m_uploads;
    changed = true;
  }
  if (new_source || m_quadAxis != static_cast<int>(s->axis) || m_quadIndex != s->index) {
    placeQuad(g, *s);
    changed = true;
  }
  if (m_filter != static_cast<int>(p.filter)) {
    setTextureInterpolate(p.filter == cvc::slice_filter::linear);
    m_filter = static_cast<int>(p.filter);
    changed = true;
  }
  if (m_opaque != (p.opaque ? 1 : 0)) {
    // Opaque: the opaque pass, without VTK scanning every texel for alpha. Otherwise the texture's
    // alpha decides, and the plane blends.
    setForceOpaque(p.opaque);
    if (auto *actor = vtkActor::SafeDownCast(getProp()))
      actor->SetForceTranslucent(!p.opaque);
    m_opaque = p.opaque ? 1 : 0;
    changed = true;
  }
  if (m_offset != p.overlay_offset) {
    applyOffset(p.overlay_offset);
    changed = true;
  }
  m_params = p;
  m_slice = s;
  m_drawn = true;
  if (changed)
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  return changed;
}

bool VolumeSliceNode::pickRay(const double origin[3], const double direction[3], double world[3],
                              std::int64_t ijk[3], double &value, bool &inside) const {
  value = std::numeric_limits<double>::quiet_NaN();
  inside = false;
  if (!m_source || !m_slice)
    return false;
  // The ray in the node's own frame, where the plane is axis-aligned.
  const double tip[3] = {origin[0] + direction[0], origin[1] + direction[1],
                         origin[2] + direction[2]};
  double o[3], t[3];
  worldToLocal(origin, o);
  worldToLocal(tip, t);
  const int a = static_cast<int>(m_slice->axis);
  const double d[3] = {t[0] - o[0], t[1] - o[1], t[2] - o[2]};
  const cvc::slice_grid &g = m_source->grid();
  const double plane = g.plane(m_slice->axis, m_slice->index);
  if (!(std::fabs(d[a]) > 1e-12))
    return false;
  const double s = (plane - o[a]) / d[a];
  if (!(s >= 0.0))
    return false;
  double hit[3] = {o[0] + s * d[0], o[1] + s * d[1], o[2] + s * d[2]};
  hit[a] = plane;
  localToWorld(hit, world);
  inside = g.voxel_of(hit, ijk);
  ijk[a] = static_cast<std::int64_t>(m_slice->index);
  if (inside) {
    int r = 0, u = 1;
    cvc::slice_plane_axes(m_slice->axis, r, u);
    value = m_slice->at(static_cast<std::uint64_t>(ijk[r]), static_cast<std::uint64_t>(ijk[u]));
  }
  return true;
}

} // namespace gl
} // namespace cvc
