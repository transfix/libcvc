/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeRenderNode — one volume, drawn by the renderer a key picks, switchable while it runs:
//
//   vtk       VolumeNode: VTK's GPU ray-cast mapper
//   volren    VolRenNode: libcvc's raycaster (cvc/volren: shading, isosurfaces, shadows)
//   volslice  VolSliceNode: view-aligned texture slices (cvc/volslice)
//   none      nothing drawn
//
// Every renderer draws the one volume handed over (setVolume): a cvc::volume handle on the same
// voxels (copy-on-write), never a copy of them. <node>.renderer picks; a change replaces the child
// renderer on the next tick(). The child is named for its kind ("vtk", "volren", "volslice"), so
// each keeps its own settings keys under <node>.children.<kind>, and a renderer switched back to
// finds them as it left them. One transfer function drives whichever is drawing: Ariadne's
// `volume_render` and `volume_grid` bind their model to each new renderer
// (tf_binding_options::model_key, tf_binding.h), re-binding whenever rendererVersion() moves; a
// host without Ariadne sets the renderer's own keys.
//
//     auto v = parent->addGraphicsChild<cvc::gl::VolumeRenderNode>("head");
//     v->setVolume(vol);                        // a std::shared_ptr<const cvc::volume>
//     v->getState("renderer").value("volren");  // vtk (the default), volren, volslice or none
//     ... each frame, before rendering, on the owner thread: v->tick();
//
// KEYS, under the node's state path:
//   renderer   vtk | volren | volslice | none; empty is vtk. A name that is none of these draws
//              nothing, and status says so
//   status     what the last switch did, or why nothing is drawn
// DATA CHANNEL: volume, the std::shared_ptr<const cvc::volume> it draws (the very pointer handed
// over).
#ifndef CVC_GL_VOLUMERENDERNODE_H
#define CVC_GL_VOLUMERENDERNODE_H

#include <atomic>
#include <cstdint>
#include <cvc/gl/NullGraphicNode.h>
#include <memory>
#include <string>

namespace cvc {
class volume;
namespace gl {

class VolumeRenderNode : public NullGraphicNode {
public:
  enum class kind { none, vtk, volren, volslice };

  VolumeRenderNode(cvc::app &ctx, const std::string &statePath, const std::string &name);
  ~VolumeRenderNode() override;

  // "none", "vtk", "volren", "volslice"; and back (false for any other name).
  static const char *kindName(kind k);
  static bool kindOf(const std::string &name, kind &k);

  // The volume every renderer draws; null draws nothing. Taken on the next tick(): the renderer
  // drawing then is handed it (made, if it waited for a volume). Owner thread.
  void setVolume(std::shared_ptr<const cvc::volume> vol);
  std::shared_ptr<const cvc::volume> volume() const { return m_volume; }

  // The renderer drawing now (null for none, or before the first volume), its kind, and a count
  // bumped each time a renderer is made or handed a new volume: a transfer function bound to the
  // renderer is bound again when it moves.
  std::shared_ptr<GraphicsNode> renderer() const { return m_renderer; }
  kind rendererKind() const { return m_kind; }
  std::uint64_t rendererVersion() const { return m_rendererVersion; }

  // Apply <node>.renderer and a new volume, then tick the renderer (volren's raycast, volslice's
  // slices). Once per frame before rendering, on the owner thread. True when the drawing changed.
  bool tick();

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void make(kind k);
  void hand();
  void status(const std::string &s);

  std::shared_ptr<const cvc::volume> m_volume;
  bool m_volumeNew = false;
  std::shared_ptr<GraphicsNode> m_renderer;
  kind m_kind = kind::none;                    // what the renderer key asks for
  kind m_made = kind::none;                    // what m_renderer is
  std::string m_error;                         // the renderer key names no renderer
  std::atomic<std::uint64_t> m_keysVersion{1}; // bumped by handleStateChanged (any thread)
  std::uint64_t m_keysSeen = 0;
  std::uint64_t m_rendererVersion = 0;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_VOLUMERENDERNODE_H
