/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeRenderNode. See inc/cvc/gl/VolumeRenderNode.h.

#include <boost/any.hpp>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/VolumeRenderNode.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>

namespace cvc {
namespace gl {

namespace {

std::string trim(const std::string &s) {
  const std::size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return std::string();
  return s.substr(b, s.find_last_not_of(" \t\r\n") - b + 1);
}

} // namespace

VolumeRenderNode::VolumeRenderNode(cvc::app &ctx, const std::string &statePath,
                                   const std::string &name)
    : NullGraphicNode(ctx, statePath, name) {}

VolumeRenderNode::~VolumeRenderNode() = default;

const char *VolumeRenderNode::kindName(kind k) {
  switch (k) {
  case kind::vtk:
    return "vtk";
  case kind::volren:
    return "volren";
  case kind::volslice:
    return "volslice";
  default:
    return "none";
  }
}

bool VolumeRenderNode::kindOf(const std::string &name, kind &k) {
  for (kind c : {kind::none, kind::vtk, kind::volren, kind::volslice})
    if (name == kindName(c)) {
      k = c;
      return true;
    }
  return false;
}

void VolumeRenderNode::setVolume(std::shared_ptr<const cvc::volume> vol) {
  if (vol == m_volume)
    return;
  m_volume = std::move(vol);
  m_volumeNew = true;
  getState("volume").data(boost::any(m_volume));
}

void VolumeRenderNode::status(const std::string &s) {
  cvc::state &st = getState("status");
  if (st.value() != s)
    st.value(s);
}

void VolumeRenderNode::handleStateChanged(const std::string &child) {
  if (child == "renderer")
    ++m_keysVersion;
  NullGraphicNode::handleStateChanged(child);
}

// The renderer of kind `k`, drawing the volume (nothing for none, or while there is no volume),
// in place of the one drawing now.
void VolumeRenderNode::make(kind k) {
  if (m_renderer) {
    removeGraphicsChild(m_renderer);
    m_renderer.reset();
  }
  m_made = kind::none;
  if (k == kind::none || !m_volume) {
    ++m_rendererVersion; // nothing to bind to now
    return;
  }
  const cvc::volume &vol = *m_volume;
  switch (k) {
  case kind::vtk: {
    auto n = addGraphicsChild<VolumeNode>(kindName(k));
    n->setVolume(vol);
    m_renderer = n;
    break;
  }
  case kind::volren: {
    auto n = addGraphicsChild<VolRenNode>(kindName(k));
    // A volren switched back to reads the settings it was left with off the tree, the volume's
    // among them: those go with the volume handed over, in its place.
    cvc::volren::volume_settings vs;
    bool left = false;
    try {
      vs = n->volumeConfig(0);
      left = true;
    } catch (const std::exception &) {
    }
    n->clearVolumes();
    n->addVolume(vol, vs);
    cvc::volren::render_settings rs = n->renderConfig();
    if (!left && rs.lights.empty() && rs.ambient == 0.0f) {
      // The first volren here: lit, so its shaded volume reads as a surface (volren's own default,
      // no light and no ambient, draws a black silhouette).
      rs.ambient = 0.35f;
      cvc::volren::light l;
      l.color = {1.0f, 1.0f, 1.0f};
      l.direction = {-0.4f, -0.3f, -0.85f};
      rs.lights.push_back(l);
      n->setRenderConfig(rs);
    }
    m_renderer = n;
    break;
  }
  case kind::volslice: {
    auto n = addGraphicsChild<VolSliceNode>(kindName(k));
    n->setVolume(vol);
    m_renderer = n;
    break;
  }
  default:
    break;
  }
  m_made = k;
  ++m_rendererVersion;
}

// The renderer drawing now takes the new volume, keeping its settings (volren's volume settings
// carry over to the new volume).
void VolumeRenderNode::hand() {
  if (!m_renderer || !m_volume)
    return;
  const cvc::volume &vol = *m_volume;
  if (auto *v = dynamic_cast<VolumeNode *>(m_renderer.get())) {
    v->setVolume(vol);
  } else if (auto *r = dynamic_cast<VolRenNode *>(m_renderer.get())) {
    if (r->volumeCount())
      r->setVolume(0, vol); // the voxels replaced, the volume's settings kept
    else
      r->addVolume(vol);
  } else if (auto *s = dynamic_cast<VolSliceNode *>(m_renderer.get())) {
    s->setVolume(vol);
  }
  ++m_rendererVersion;
}

bool VolumeRenderNode::tick() {
  bool changed = false;
  if (m_keysVersion != m_keysSeen) {
    m_keysSeen = m_keysVersion;
    const std::string name = trim(getState("renderer").value());
    kind k = kind::vtk;
    m_error.clear();
    if (!name.empty() && !kindOf(name, k)) {
      m_error = "error: renderer '" + name + "' is not vtk, volren, volslice or none";
      k = kind::none;
    }
    m_kind = k;
  }
  // What draws: the kind asked for, once there is a volume to draw.
  const kind drawing = m_volume ? m_kind : kind::none;
  if (drawing != m_made) {
    make(drawing);
    m_volumeNew = false;
    changed = true;
  } else if (m_volumeNew) {
    m_volumeNew = false;
    if (m_renderer) {
      hand();
      changed = true;
    }
  }
  status(!m_error.empty()       ? m_error
         : m_kind == kind::none ? std::string("none: nothing drawn")
         : !m_volume            ? std::string("waiting for a volume")
                                : std::string("drawn by ") + kindName(m_kind));
  if (auto *r = dynamic_cast<VolRenNode *>(m_renderer.get()))
    changed = r->tick() || changed;
  else if (auto *s = dynamic_cast<VolSliceNode *>(m_renderer.get()))
    changed = s->tick() || changed;
  return changed;
}

} // namespace gl
} // namespace cvc
