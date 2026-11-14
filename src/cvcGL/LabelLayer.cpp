/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cvc/gl/LabelLayer.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/state/state.h>
#include <exception>
#include <unordered_set>
#include <vtkActor2D.h>
#include <vtkCamera.h>
#include <vtkCoordinate.h>
#include <vtkObjectFactory.h>
#include <vtkProp.h>
#include <vtkPropCollection.h>
#include <vtkRenderer.h>
#include <vtkTextMapper.h>
#include <vtkTextProperty.h>
#include <vtkViewport.h>
#include <vtkWindow.h>

namespace cvc {
namespace gl {

// The layer's one prop: it has no geometry, and in the overlay pass it draws the layer's labels
// through their own 2-D text actors, which no renderer holds (so a mirror, which leaves 2-D
// actors out, still gets the labels through this prop).
class LabelLayerProp : public vtkProp {
public:
  static LabelLayerProp *New();
  vtkTypeMacro(LabelLayerProp, vtkProp);

  int RenderOverlay(vtkViewport *viewport) override {
    return owner ? owner->renderOverlay(viewport) : 0;
  }
  void ReleaseGraphicsResources(vtkWindow *window) override {
    if (owner)
      owner->releaseGraphicsResources(window);
  }
  vtkTypeBool HasTranslucentPolygonalGeometry() override { return 0; }

  LabelLayerNode *owner = nullptr; // cleared by the node's destructor
};
vtkStandardNewMacro(LabelLayerProp);

struct LabelLayerNode::Label {
  Id id = 0;
  std::weak_ptr<GraphicsNode> node;
  std::string text;
  LabelStyle style;
  vtkSmartPointer<vtkActor2D> actor;
  vtkSmartPointer<vtkTextMapper> mapper;
  // What the text actor was last set up with (-1: nothing yet).
  std::string applied_text;
  int applied_size = -1, applied_alpha = -1;
  bool style_dirty = true;
  // The anchor in the node's own frame, as of the scene's content version `anchor_version`.
  double local_anchor[3] = {0, 0, 0};
  std::uint64_t anchor_version = ~std::uint64_t(0);
};

double label_fade(double d, double fade_near, double fade_far) {
  if (!(fade_far > 0.0))
    return 1.0;
  if (!(d > fade_near))
    return 1.0;
  if (!(d < fade_far))
    return 0.0;
  const double band = fade_far - fade_near;
  return band > 0.0 ? std::clamp((fade_far - d) / band, 0.0, 1.0) : 0.0;
}

LabelLayerNode::LabelLayerNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GraphicsNode(ctx, statePath, name), m_prop(vtkSmartPointer<LabelLayerProp>::New()) {
  m_prop->owner = this;
  m_prop->PickableOff(); // nothing to hit: the labels are 2-D
  setCastsShadow(false);
}

LabelLayerNode::~LabelLayerNode() {
  if (m_prop)
    m_prop->owner = nullptr; // a renderer may still hold the prop a moment longer
}

std::shared_ptr<LabelLayerNode> LabelLayerNode::of(SceneGraph &sg, const std::string &name) {
  if (sg.hasGraphics(name))
    if (auto layer = std::dynamic_pointer_cast<LabelLayerNode>(sg.getGraphics(name)))
      return layer;
  auto layer = sg.getGraphicsRoot()->addGraphicsChild<LabelLayerNode>(name);
  sg.registerGraphics(name, layer);
  return layer;
}

vtkProp *LabelLayerNode::getProp() { return m_prop; }

cvc::bounding_box LabelLayerNode::getBoundingBox() const { return cvc::bounding_box(); }

LabelLayerNode::Id LabelLayerNode::add(const std::shared_ptr<GraphicsNode> &node,
                                       const std::string &text, const LabelStyle &style) {
  auto l = std::make_unique<Label>();
  l->node = node;
  l->text = text;
  l->style = style;
  std::lock_guard<std::mutex> lock(m_mutex);
  l->id = m_next++;
  const Id id = l->id;
  m_labels.push_back(std::move(l));
  return id;
}

void LabelLayerNode::setText(Id id, const std::string &text) {
  std::lock_guard<std::mutex> lock(m_mutex);
  for (auto &l : m_labels)
    if (l->id == id)
      l->text = text;
}

void LabelLayerNode::setScale(Id id, double scale) {
  std::lock_guard<std::mutex> lock(m_mutex);
  for (auto &l : m_labels)
    if (l->id == id)
      l->style.scale = scale;
}

void LabelLayerNode::setStyle(Id id, const LabelStyle &style) {
  std::lock_guard<std::mutex> lock(m_mutex);
  for (auto &l : m_labels)
    if (l->id == id) {
      l->style = style;
      l->style_dirty = true;
      l->anchor_version = ~std::uint64_t(0);
    }
}

void LabelLayerNode::remove(Id id) {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_labels.erase(std::remove_if(m_labels.begin(), m_labels.end(),
                                [id](const std::unique_ptr<Label> &l) { return l->id == id; }),
                 m_labels.end());
}

std::size_t LabelLayerNode::size() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return static_cast<std::size_t>(
      std::count_if(m_labels.begin(), m_labels.end(),
                    [](const std::unique_ptr<Label> &l) { return !l->node.expired(); }));
}

std::vector<LabelLayerNode::Id> LabelLayerNode::labelsOf(const GraphicsNode *node) const {
  std::vector<Id> out;
  std::lock_guard<std::mutex> lock(m_mutex);
  for (const auto &l : m_labels)
    if (l->node.lock().get() == node)
      out.push_back(l->id);
  return out;
}

std::string LabelLayerNode::text(Id id) const {
  std::lock_guard<std::mutex> lock(m_mutex);
  for (const auto &l : m_labels)
    if (l->id == id)
      return l->text;
  return std::string();
}

LabelStyle LabelLayerNode::style(Id id) const {
  std::lock_guard<std::mutex> lock(m_mutex);
  for (const auto &l : m_labels)
    if (l->id == id)
      return l->style;
  return LabelStyle();
}

std::vector<LabelLayerNode::Drawn> LabelLayerNode::lastDrawn() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_lastDrawn;
}

std::uint64_t LabelLayerNode::rasterisations() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_rasterisations;
}

void LabelLayerNode::releaseGraphicsResources(vtkWindow *window) {
  std::lock_guard<std::mutex> lock(m_mutex);
  for (auto &l : m_labels)
    if (l->actor)
      l->actor->ReleaseGraphicsResources(window);
}

namespace {
// Opacity steps: a fading label is rasterised again at most this many times across its band.
constexpr int kAlphaSteps = 32;

// The value a read of `path` sees ("" when it has none), through transparent links.
std::string read_key(cvc::app &app, const std::string &path) {
  try {
    cvc::state *s = cvc::state::instance(app).findThroughLinks(path);
    if (!s)
      return std::string();
    if (s->isLink() && s->linkMode() == cvc::state::link_mode::transparent) {
      const cvc::state::link_resolution lr = s->resolveLink();
      if (lr.kind == cvc::state::link_resolution_kind::resolved && lr.target)
        s = lr.target;
    }
    return s->value();
  } catch (const std::exception &) {
    return std::string();
  }
}
} // namespace

int LabelLayerNode::renderOverlay(vtkViewport *viewport) {
  vtkRenderer *ren = vtkRenderer::SafeDownCast(viewport);
  if (!ren || !ren->GetActiveCamera())
    return 0;
  SceneGraph *sg = getSceneGraph();
  const std::uint64_t version = sg ? sg->contentVersion() : 0;
  double eye[3];
  ren->GetActiveCamera()->GetPosition(eye);
  const int *size = ren->GetSize();
  const int *origin = ren->GetOrigin();

  // What this renderer draws: a label is shown only where its node is (a view's tags).
  std::unordered_set<const vtkProp *> drawn;
  vtkPropCollection *props = ren->GetViewProps();
  vtkCollectionSimpleIterator it;
  props->InitTraversal(it);
  while (vtkProp *p = props->GetNextProp(it))
    drawn.insert(p);

  struct Pending {
    Label *label;
    double x, y, distance, opacity;
  };
  std::vector<Pending> pending;
  std::vector<vtkProp *> nodeProps;
  std::lock_guard<std::mutex> lock(m_mutex);
  m_labels.erase(std::remove_if(m_labels.begin(), m_labels.end(),
                                [](const std::unique_ptr<Label> &l) { return l->node.expired(); }),
                 m_labels.end());
  for (auto &up : m_labels) {
    Label &l = *up;
    std::shared_ptr<GraphicsNode> node = l.node.lock();
    if (!node || !node->isVisibleInHierarchy())
      continue;
    if (!l.style.text_key.empty())
      if (std::string t = read_key(app(), l.style.text_key); !t.empty())
        l.text = std::move(t);
    if (l.text.empty())
      continue;
    nodeProps.clear();
    node->collectViewProps(nodeProps);
    if (!nodeProps.empty() && std::none_of(nodeProps.begin(), nodeProps.end(), [&](vtkProp *p) {
          return drawn.count(p) && p->GetVisibility();
        }))
      continue; // not in this view (or nothing of it shown)
    if (l.anchor_version != version) {
      const cvc::bounding_box b = node->getCombinedBoundingBox();
      if (l.style.anchor == LabelStyle::Anchor::Origin || b.isNull()) {
        l.local_anchor[0] = l.local_anchor[1] = l.local_anchor[2] = 0.0;
      } else {
        l.local_anchor[0] = 0.5 * (b.minx + b.maxx);
        l.local_anchor[1] = 0.5 * (b.miny + b.maxy);
        l.local_anchor[2] =
            l.style.anchor == LabelStyle::Anchor::Top ? b.maxz : 0.5 * (b.minz + b.maxz);
      }
      l.anchor_version = version;
    }
    double w[3];
    node->localToWorld(l.local_anchor, w);
    for (int i = 0; i < 3; ++i)
      w[i] += l.style.offset[i];
    const double dx = w[0] - eye[0], dy = w[1] - eye[1], dz = w[2] - eye[2];
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double opacity = label_fade(distance, l.style.fade_near, l.style.fade_far);
    if (opacity <= 0.0)
      continue; // past its fade: culled
    ren->SetWorldPoint(w[0], w[1], w[2], 1.0);
    ren->WorldToDisplay();
    double d[3];
    ren->GetDisplayPoint(d);
    if (!(d[2] >= 0.0 && d[2] <= 1.0))
      continue; // behind the camera, or past its far plane
    const double margin = 4.0 * l.style.size * std::max(1.0, l.style.scale);
    if (d[0] < origin[0] - margin || d[0] > origin[0] + size[0] + margin ||
        d[1] < origin[1] - margin || d[1] > origin[1] + size[1] + margin)
      continue; // off this view
    pending.push_back({&l, d[0], d[1], distance, opacity});
  }
  // Farthest first: the nearer label is drawn over it.
  std::stable_sort(pending.begin(), pending.end(),
                   [](const Pending &a, const Pending &b) { return a.distance > b.distance; });

  m_lastDrawn.clear();
  int rendered = 0;
  for (const Pending &p : pending) {
    Label &l = *p.label;
    if (!l.actor) {
      l.mapper = vtkSmartPointer<vtkTextMapper>::New();
      l.actor = vtkSmartPointer<vtkActor2D>::New();
      l.actor->SetMapper(l.mapper);
      l.actor->GetPositionCoordinate()->SetCoordinateSystemToDisplay();
      vtkTextProperty *tp = l.mapper->GetTextProperty();
      tp->SetJustificationToCentered();
      tp->SetVerticalJustificationToBottom();
      tp->ShadowOff();
    }
    double scale = l.style.scale;
    if (!l.style.scale_key.empty()) {
      const std::string v = read_key(app(), l.style.scale_key);
      char *end = nullptr;
      const double x = std::strtod(v.c_str(), &end);
      if (end != v.c_str() && std::isfinite(x) && x > 0.0)
        scale = x;
    }
    if (!l.style.focus_key.empty() && read_key(app(), l.style.focus_key) == l.style.focus_value)
      scale *= l.style.focus_scale;
    const int px = std::max(1, static_cast<int>(std::lround(l.style.size * scale)));
    const int alpha = static_cast<int>(std::lround(p.opacity * kAlphaSteps));
    if (l.style_dirty || px != l.applied_size || alpha != l.applied_alpha ||
        l.text != l.applied_text) {
      vtkTextProperty *tp = l.mapper->GetTextProperty();
      const double a = static_cast<double>(alpha) / kAlphaSteps;
      tp->SetFontSize(px);
      tp->SetBold(l.style.bold ? 1 : 0);
      tp->SetColor(l.style.color[0], l.style.color[1], l.style.color[2]);
      tp->SetOpacity(a);
      tp->SetBackgroundColor(l.style.backdrop_color[0], l.style.backdrop_color[1],
                             l.style.backdrop_color[2]);
      tp->SetBackgroundOpacity(l.style.backdrop ? l.style.backdrop_opacity * a : 0.0);
      tp->SetFrame(l.style.edge ? 1 : 0);
      tp->SetFrameColor(l.style.edge_color[0], l.style.edge_color[1], l.style.edge_color[2]);
      tp->SetFrameWidth(std::max(1, l.style.edge_width));
      l.mapper->SetInput(l.text.c_str());
      l.applied_size = px;
      l.applied_alpha = alpha;
      l.applied_text = l.text;
      l.style_dirty = false;
      ++m_rasterisations;
    }
    l.actor->GetPositionCoordinate()->SetValue(p.x, p.y);
    rendered += l.actor->RenderOverlay(viewport);
    m_lastDrawn.push_back({l.id, l.text, p.x, p.y, p.distance, p.opacity, px});
  }
  return rendered;
}

} // namespace gl
} // namespace cvc
