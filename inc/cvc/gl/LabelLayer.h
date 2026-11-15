/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef CVC_GL_LABEL_LAYER_H
#define CVC_GL_LABEL_LAYER_H

#include <cstdint>
#include <cvc/gl/GraphicsNode.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <vtkSmartPointer.h>

class vtkActor2D;
class vtkTextMapper;
class vtkViewport;
class vtkWindow;

namespace cvc {
namespace gl {

class SceneGraph;
class LabelLayerProp;

// How one label looks. Distances and offsets are world units; sizes are pixels.
struct LabelStyle {
  enum class Anchor { Top, Center, Origin };
  Anchor anchor = Anchor::Top;  // the top of the node's bounds, their centre, or its origin
  double offset[3] = {0, 0, 0}; // added to the anchor, in world axes
  int size = 14;                // font size at scale 1
  double scale = 1.0;           // the size's multiplier (a chosen node drawn larger)
  float color[3] = {1, 1, 1};   // the text
  bool backdrop = true;         // a box behind the text
  float backdrop_color[3] = {0.05f, 0.05f, 0.07f};
  float backdrop_opacity = 0.6f;
  bool edge = false; // a frame round the box
  float edge_color[3] = {1, 1, 1};
  int edge_width = 1;
  bool bold = false;
  // Fade and cull by the camera's distance: whole nearer than fade_near, gone past fade_far,
  // linear between. fade_far <= 0: neither.
  double fade_near = 0.0, fade_far = 0.0;
  // State keys read at each draw (absolute paths; empty: none). Read, never written.
  std::string text_key;    // the text, while the key holds one
  std::string scale_key;   // the scale, while the key holds a number
  std::string focus_key;   // while it holds focus_value, the label is focus_scale times larger
  std::string focus_value; //   (a node's chosen one: the followed vehicle)
  double focus_scale = 1.4;
};

// A label's opacity at camera distance `d` (LabelStyle's fade rule).
double label_fade(double d, double fade_near, double fade_far);

// ---------------------------------------------------------------------------
// LabelLayerNode -- screen-space text labels anchored to scene nodes, drawn in
// the overlay pass of every renderer the layer's prop is in.
//
// Each draw, for each label, in that renderer: the anchor is read from the
// node's world transform and its bound keys from state (nothing is written);
// a label whose node is hidden,
// or not drawn by that renderer (a view whose tags leave it out), is skipped;
// one past its fade distance is culled, one in its fade band drawn fainter;
// the rest are drawn farthest first, so a nearer label lies over a farther
// one. Distances are from that renderer's camera, so a mirror or a viewport
// fades and orders by its own view. The layer is one node of the scene, so a
// mirror viewport copies its prop like any other.
//
// The text is rasterised again only when it, its size or its (quantised)
// opacity change: a still label costs a projection and a quad.
//
// Owner/render thread for the VTK side; add/set/remove are safe from any
// thread (a mutex), and apply at the next draw.
class LabelLayerNode : public GraphicsNode {
public:
  using Id = std::uint64_t;
  static constexpr const char *kDefaultName = "__labels";

  // One label as the last draw drew it, in draw order.
  struct Drawn {
    Id id = 0;
    std::string text;
    double x = 0, y = 0; // display pixels of the anchor (the text sits above it)
    double distance = 0;
    double opacity = 1;
    int size = 0;
  };

  LabelLayerNode(cvc::app &ctx, const std::string &statePath, const std::string &name = "");
  ~LabelLayerNode() override;

  // The scene's layer: its top-level node `name`, made and registered on first use.
  static std::shared_ptr<LabelLayerNode> of(SceneGraph &sg, const std::string &name = kDefaultName);

  Id add(const std::shared_ptr<GraphicsNode> &node, const std::string &text,
         const LabelStyle &style);
  void setText(Id id, const std::string &text);
  void setScale(Id id, double scale);
  void setStyle(Id id, const LabelStyle &style);
  void remove(Id id);

  std::size_t size() const; // labels whose node still exists
  std::vector<Id> labelsOf(const GraphicsNode *node) const;
  std::string text(Id id) const;
  LabelStyle style(Id id) const;
  // What the last draw drew (any renderer), farthest first.
  std::vector<Drawn> lastDrawn() const;
  // How many times a label's text was rasterised (set up again) so far, for tests.
  std::uint64_t rasterisations() const;

  cvc::bounding_box getBoundingBox() const override; // none: labels have no extent

protected:
  vtkProp *getProp() override;

private:
  friend class LabelLayerProp;
  struct Label;
  int renderOverlay(vtkViewport *viewport);
  void releaseGraphicsResources(vtkWindow *window);

  mutable std::mutex m_mutex;
  std::vector<std::unique_ptr<Label>> m_labels;
  Id m_next = 1;
  vtkSmartPointer<LabelLayerProp> m_prop;
  std::vector<Drawn> m_lastDrawn;
  std::uint64_t m_rasterisations = 0;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_LABEL_LAYER_H
