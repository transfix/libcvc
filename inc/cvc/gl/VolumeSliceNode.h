/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeSliceNode — one axis-aligned plane of a volume, drawn as a textured quad coloured through
// the node's transfer function (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §6 item 1). The
// fourth way cvcGL shows a volume, beside VolumeNode (VTK's GPU raycaster), VolRenNode (the
// cvc::volren raycaster) and VolSliceNode (camera-facing slices composited back to front): this
// one cuts a single plane down x, y or z, as VolumeGridRover's slice views and the classic
// three-plane orthoslicer do.
//
//     auto node = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolumeSliceNode>("ct_z");
//     node->setVolume(vol);                          // or setSource(slice_source(app,
//     "big.rawiv"))
//     ... in the render loop, before SceneRenderer::render(): node->tick();
//
// HOW IT DRAWS. tick() cuts the plane from the node's cvc::slice_source
// (cvc/volume/volume_slice.h), colours it on the CPU through the transfer-function model at
// <node>.transfer_function (cvc::color_slice) and uploads it as the quad's texture: one 2D texture
// per change of slice, axis or transfer function, so stepping through a 2k^3 volume never puts the
// volume on the GPU. A file source reads one plane at a time (region reads) and keeps the last few,
// so a volume larger than memory can be scrubbed. A new plane the same size as the last is written
// into the texture's own pixels (zero copy, GeometryNode::texture_modified); another axis gets a
// new texture. The quad covers the plane's voxels with half a voxel to spare on each side, so each
// voxel's texel is centred on the voxel. It is unlit: the texture is the colour.
//
// KEYS (all under the node's state path):
//   slice.axis, slice.index, slice.filter, slice.opaque, slice.overlay_offset
//                  how it draws: cvc::volume_slice_params (param_table: units, ranges and
//                  choices as properties). index -1 is the middle plane, written back as its
//                  number; an index past the last plane is drawn as the last and written back.
//                  slice.index's "max" property is the last plane's number along the axis.
//   transfer_function  the cvc::tf::model the voxels are coloured through (a tf_editor edits it),
//                  over the volume's data range unless the model's domain is explicit
//   slice.count, slice.position  published: how many planes the axis has, and where the plane
//                  lies along it (object space)
//   slice.status   published: "" or what is wrong (a key that does not parse, a failed read)
//   info.dims, info.min, info.max, info.span, info.box, info.file
//                  published: the volume's dimensions ("x y z"), data range, voxel spacing
//                  ("x y z"), bounding box ("x0 y0 z0 x1 y1 z1") and its file ("" in memory)
//
// OVERLAYS. Child nodes of the slice (point glyphs, polylines, crosshairs) are its overlays: the
// quad is drawn pushed back in depth by slice.overlay_offset (VTK's relative coincident-topology
// polygon offset: factor offset / 2, units offset), so geometry drawn in its plane is never
// z-fought away and shows on top, in every view.
//
// PICKING. pickRay() intersects a world ray with the plane, exactly, and says which voxel it hit
// and its value: what a slice view's pick reports (scene_views.h).
#ifndef CVC_GL_VOLUMESLICENODE_H
#define CVC_GL_VOLUMESLICENODE_H

#include <atomic>
#include <cstdint>
#include <cvc/gl/GeometryNode.h>
#include <cvc/image/image.h>
#include <cvc/tf/model.h>
#include <cvc/volume/volume_slice.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace gl {

class VolumeSliceNode : public GeometryNode {
public:
  VolumeSliceNode(cvc::app &ctx, const std::string &statePath, const std::string &name);
  ~VolumeSliceNode() override;

  // What it slices: a volume in memory (its voxels shared), or any slice_source (a file read plane
  // by plane). Replaces the last; the next tick() cuts the plane again. Owner thread.
  void setVolume(const cvc::volume &vol);
  void setSource(std::shared_ptr<cvc::slice_source> src);
  std::shared_ptr<cvc::slice_source> source() const { return m_source; }

  // Cut, colour and upload the plane again if the source, a slice.* key or the transfer function
  // changed since the last call. Call once per frame before rendering, on the owner thread.
  // Returns true when the drawing changed.
  bool tick();

  // What the last tick() drew: its parameters (the index resolved and clamped), the plane, and its
  // pixels (RGBA, image order: the top row first) -- for hosts and tests.
  const cvc::volume_slice_params &params() const { return m_params; }
  std::shared_ptr<const cvc::volume_slice_data> slice() const { return m_slice; }
  const std::vector<std::uint8_t> &pixels() const { return m_pixels; }
  // How many times tick() has uploaded a texture: a new one (another size) or new pixels in the
  // same one.
  std::size_t uploads() const { return m_uploads; }

  // Where a world ray (origin, direction) meets the plane: false when it runs parallel to it or
  // meets it behind the origin, or before the first tick. `world` is the point, `ijk` the voxel
  // nearest it (along the plane's axis, the plane's own index) and `value` that voxel's value
  // (NaN outside the volume); `inside` says whether the voxel lies in the volume.
  bool pickRay(const double origin[3], const double direction[3], double world[3],
               std::int64_t ijk[3], double &value, bool &inside) const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void publishInfo();
  void placeQuad(const cvc::slice_grid &g, const cvc::volume_slice_data &s);
  void applyOffset(double offset);

  std::shared_ptr<cvc::slice_source> m_source;
  std::atomic<std::uint64_t> m_keysVersion{1}; // bumped by any slice.* write
  std::atomic<std::uint64_t> m_tfVersion{1};   // bumped by a transfer_function write
  std::uint64_t m_sourceVersion = 1;           // bumped by setSource
  std::uint64_t m_drawnKeys = 0, m_drawnTf = 0, m_drawnSource = 0;
  cvc::volume_slice_params m_params;
  bool m_drawn = false;
  cvc::tf::model m_model;
  std::string m_modelText = "\x01"; // the model string m_model was parsed from (never a real one)
  std::shared_ptr<const cvc::volume_slice_data> m_slice;
  std::vector<std::uint8_t> m_pixels;
  cvc::image m_image; // the texture's pixels (aliased by the texture: written in place)
  int m_quadAxis = -1;
  std::uint64_t m_quadIndex = ~std::uint64_t(0);
  double m_offset = -1.0;
  int m_filter = -1;
  int m_opaque = -1;
  std::size_t m_uploads = 0;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_VOLUMESLICENODE_H
