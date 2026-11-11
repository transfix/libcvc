/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// SceneViews — the `viewport` widget. See inc/cvc/gl/ariadne/scene_views.h.

#include <algorithm>
#include <array>
#include <cmath>
#include <cvc/ariadne/bind.h> // resolve_bind, write: where a pick lands
#include <cvc/ariadne/widget.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/TouchScroll.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/scene_views.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkCommand.h>
#include <vtkMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPiecewiseFunction.h>
#include <vtkPlaneCollection.h>
#include <vtkPropCollection.h>
#include <vtkProperty.h>
#include <vtkRenderPass.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkShaderProperty.h>
#include <vtkSmartPointer.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkTransform.h>
#include <vtkVolume.h>
#include <vtkVolumeProperty.h>

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // the window's clip rect: the part of the widget that shows
#endif

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

// An override:'s look: the fields it sets on a node's actors in one view, the rest as the node has
// them. Later entries win field by field, unless an entry's mode is replace. Its transform is kept
// apart (View::draw composes them).
struct Look {
  int rep = -1; // VTK_POINTS / VTK_WIREFRAME / VTK_SURFACE, -1 unset
  double opacity = -1.0, line_width = -1.0, point_size = -1.0;
  bool has_color = false;
  double color[3] = {1.0, 1.0, 1.0};
  bool replace = false;   // mode: replace: start over from the node's own look
  bool has_xform = false; // transform: { position, rotation, scale }
  double xform[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  bool empty() const {
    return rep < 0 && opacity < 0.0 && line_width < 0.0 && point_size < 0.0 && !has_color;
  }
  void merge(const Look &o) {
    if (o.rep >= 0)
      rep = o.rep;
    if (o.opacity >= 0.0)
      opacity = o.opacity;
    if (o.line_width >= 0.0)
      line_width = o.line_width;
    if (o.point_size >= 0.0)
      point_size = o.point_size;
    if (o.has_color) {
      has_color = true;
      std::copy(o.color, o.color + 3, color);
    }
  }
  bool operator==(const Look &o) const {
    return rep == o.rep && opacity == o.opacity && line_width == o.line_width &&
           point_size == o.point_size && has_color == o.has_color &&
           (!has_color || std::equal(color, color + 3, o.color));
  }
};

// A restyled stand-in for one of the main view's actors: its own vtkActor and vtkProperty over the
// SAME mapper (the geometry is not copied), kept in step with the original each frame.
struct Twin {
  vtkSmartPointer<vtkActor> actor;
  vtkSmartPointer<vtkVolume> volume;          // a volume node's stand-in
  vtkSmartPointer<vtkSmartVolumeMapper> vmap; // ... and its own mapper, while it restyles
  vtkSmartPointer<vtkMatrix4x4> placed;       // where a transform: override puts it
  vtkMTimeType copied = 0; // the original's property MTime when it was last copied
  Look look;               // the look last applied
  bool seen = false;       // drawn this frame
};

// One override: entry, resolved for this frame: its look, and the props it covers, each with the
// world matrix of the node the entry names (the frame its transform: applies in). "*" covers all,
// in world space.
struct Entry {
  Look look;
  bool all = false;
  std::unordered_map<vtkProp *, std::array<double, 16>> props;
};

// The scene's volume nodes, by prop: how a view from its own camera draws each.
enum class VolumeKind { slices, raycast, gpu };
struct VolumeProp {
  VolumeKind kind = VolumeKind::gpu;
  GraphicsNode *node = nullptr;
};

void identity(double m[16]) {
  for (int i = 0; i < 16; ++i)
    m[i] = (i % 5 == 0) ? 1.0 : 0.0;
}

// b at a's place, or at X times a's place (a transform: override, a world-space premultiplier).
void place(vtkProp3D *b, vtkProp3D *a, const double *X, vtkSmartPointer<vtkMatrix4x4> &placed) {
  if (!X) {
    b->SetPosition(a->GetPosition());
    b->SetOrientation(a->GetOrientation());
    b->SetScale(a->GetScale());
    b->SetOrigin(a->GetOrigin());
    b->SetUserMatrix(a->GetUserMatrix());
    b->SetUserTransform(a->GetUserTransform());
    return;
  }
  if (!placed)
    placed = vtkSmartPointer<vtkMatrix4x4>::New();
  double m[16];
  vtkMatrix4x4::Multiply4x4(X, a->GetMatrix()->GetData(), m);
  placed->DeepCopy(m);
  b->SetUserTransform(nullptr);
  b->SetPosition(0.0, 0.0, 0.0);
  b->SetOrientation(0.0, 0.0, 0.0);
  b->SetScale(1.0, 1.0, 1.0);
  b->SetOrigin(0.0, 0.0, 0.0);
  b->SetUserMatrix(placed);
}

} // namespace

struct SceneViews::View {
  std::string name;  // the viewport's, and its camera's viewer: <prefix>.viewers.<name>.camera
  std::string scene; // the scene it shows: "" the main one, or a scenes: entry
  Viewport *vp = nullptr;
  std::vector<std::string> show, hide; // tags; show empty = every tag
  bool chrome = true;                  // the scene's grid, axes and bounding box
  ari::SceneCamera start;              // its own camera: mode, track, where its orbit starts
  std::string camera;                  // the camera it draws from: "" its own, "main", or a name
  vtkSmartPointer<vtkCamera> own;      // its own vtkCamera, kept while it draws from another
  // Its shadows: "shared" (the maps its scene's renderer bakes), "own" (a baker of its own, for the
  // casters it draws), or "off".
  std::string shadow_mode = "shared";
  std::pair<const void *, double> shadow_key{nullptr, -1.0}; // what shadow_pass was made from
  std::string shadow_built;                                  // ... and in which mode
  vtkSmartPointer<vtkRenderPass> shadow_pass;
  bool input = true;   // drags and the wheel move its camera
  // input: pick: the left button picks instead of turning the camera, writing where it landed
  // under pick_key (absolute) and raising on_pick.
  bool pick = false;
  std::string pick_key, on_pick;
  bool picking = false; // a left press that picks is down
  long pick_seq = 0;
  bool drawn = false;  // a widget drew it this frame
  bool framed = false; // its camera has been framed on what it shows
  double region[4] = {-1.0, -1.0, -1.0, -1.0};
  std::unordered_set<vtkProp *> hidden; // the main view's props this one leaves out
  // override: entries in order: a node id, a tag, or "*" (everything), and its look.
  std::vector<std::pair<std::string, Look>> overrides;
  std::vector<Entry> entries; // ... resolved to props, this frame
  std::unordered_map<vtkProp *, VolumeProp> volumes;
  bool own_eye = true; // its camera is not the main view's
  std::unordered_map<vtkProp *, Twin> twins;
  bool dragging = false, panning = false;

  // What the mirror draws for the main view's prop p (Viewport::setMirrorFilter).
  vtkProp *draw(vtkProp *p) {
    if (hidden.count(p))
      return nullptr;
    // The volumes drawn for the main view: a volslice node's slices face its camera, so a view from
    // another camera draws slices of its own; a volren node's picture is raycast for the main view
    // and composited into its pixels, so no view draws it.
    vtkProp *src = p;
    auto vi = volumes.find(p);
    if (vi != volumes.end()) {
      if (vi->second.kind == VolumeKind::raycast)
        return nullptr;
      if (own_eye && vi->second.kind == VolumeKind::slices)
        src = static_cast<VolSliceNode *>(vi->second.node)->viewProp(vp->renderer());
      if (!src)
        return nullptr;
    }
    // Its look: the entries that cover it, in order; their transforms compose, each in the frame of
    // the node it names.
    Look look;
    double X[16];
    identity(X);
    bool moved = false;
    for (const Entry &e : entries) {
      const std::array<double, 16> *frame = nullptr;
      if (!e.all) {
        auto it = e.props.find(p);
        if (it == e.props.end())
          continue;
        frame = &it->second;
      }
      if (e.look.replace) {
        look = Look{};
        identity(X);
        moved = false;
      }
      look.merge(e.look);
      if (e.look.has_xform) {
        double F[16], Fi[16], M[16], N[16];
        if (frame)
          std::copy(frame->begin(), frame->end(), F);
        else
          identity(F);
        vtkMatrix4x4::Invert(F, Fi);
        vtkMatrix4x4::Multiply4x4(F, e.look.xform, M);
        vtkMatrix4x4::Multiply4x4(M, Fi, N);
        vtkMatrix4x4::Multiply4x4(N, X, M);
        std::copy(M, M + 16, X);
        moved = true;
      }
    }
    if (look.empty() && !moved)
      return src;
    if (vtkVolume *vol = vtkVolume::SafeDownCast(src))
      return volume_twin(p, vol, look, moved ? X : nullptr);
    vtkActor *a = vtkActor::SafeDownCast(src);
    if (!a || !a->GetMapper())
      return src;
    Twin &t = twins[p];
    if (!t.actor)
      t.actor = vtkSmartPointer<vtkActor>::New();
    vtkActor *b = t.actor;
    if (b->GetMapper() != a->GetMapper())
      b->SetMapper(a->GetMapper());
    b->SetShaderProperty(a->GetShaderProperty()); // the node's shader: replacements, shared
    b->SetTexture(a->GetTexture());
    place(b, a, moved ? X : nullptr, t.placed);
    b->SetVisibility(a->GetVisibility()); // an LOD ladder shows one rung
    b->SetPickable(false);
    const vtkMTimeType m = a->GetProperty()->GetMTime();
    if (m != t.copied || !(look == t.look)) {
      vtkProperty *q = b->GetProperty();
      q->DeepCopy(a->GetProperty());
      if (look.rep >= 0)
        q->SetRepresentation(look.rep);
      if (look.opacity >= 0.0)
        q->SetOpacity(look.opacity);
      if (look.line_width >= 0.0)
        q->SetLineWidth(look.line_width);
      if (look.point_size >= 0.0)
        q->SetPointSize(look.point_size);
      if (look.has_color)
        q->SetColor(look.color[0], look.color[1], look.color[2]);
      t.copied = m;
      t.look = look;
    }
    // A see-through override draws in the translucent pass even where the node is forced opaque.
    const bool see_through = look.opacity >= 0.0 && look.opacity < 1.0;
    b->SetForceOpaque(see_through ? false : a->GetForceOpaque());
    b->SetForceTranslucent(see_through || a->GetForceTranslucent());
    t.seen = true;
    return b;
  }

  // A `type: volume` node's stand-in: moved by a transform: override over the node's own mapper,
  // or, with an opacity, a property and mapper of its own (a GPU volume mapper keeps one set of
  // transfer-function tables, for the property it last drew, so a second property needs a second
  // mapper, which uploads the volume again).
  vtkProp *volume_twin(vtkProp *p, vtkVolume *a, const Look &look, const double *X) {
    vtkVolumeProperty *ap = a->GetProperty();
    if (!a->GetMapper() || !ap)
      return a;
    Twin &t = twins[p];
    if (!t.volume)
      t.volume = vtkSmartPointer<vtkVolume>::New();
    vtkVolume *b = t.volume;
    place(b, a, X, t.placed);
    b->SetVisibility(a->GetVisibility());
    b->SetPickable(false);
    t.seen = true;
    if (look.opacity < 0.0) {
      t.vmap = nullptr;
      if (b->GetMapper() != a->GetMapper())
        b->SetMapper(a->GetMapper());
      if (b->GetProperty() != ap)
        b->SetProperty(ap);
      return b;
    }
    auto *am = vtkSmartVolumeMapper::SafeDownCast(a->GetMapper());
    if (!am)
      return a; // only VolumeNode's mapper is restyled
    if (!t.vmap)
      t.vmap = vtkSmartPointer<vtkSmartVolumeMapper>::New();
    vtkSmartVolumeMapper *bm = t.vmap;
    if (bm->GetInputDataObject(0, 0) != am->GetInputDataObject(0, 0))
      bm->SetInputDataObject(am->GetInputDataObject(0, 0));
    bm->SetBlendMode(am->GetBlendMode());
    bm->SetAutoAdjustSampleDistances(am->GetAutoAdjustSampleDistances());
    bm->SetSampleDistance(am->GetSampleDistance());
    bm->SetVolumetricScatteringBlending(am->GetVolumetricScatteringBlending());
    bm->SetGlobalIlluminationReach(am->GetGlobalIlluminationReach());
    bm->SetClippingPlanes(am->GetClippingPlanes());
    if (b->GetMapper() != bm)
      b->SetMapper(bm);
    // Its property: the node's, with every opacity point scaled, copied again when the node's
    // property or the look changes.
    vtkMTimeType m = ap->GetMTime();
    if (vtkPiecewiseFunction *f = ap->GetScalarOpacity())
      m = std::max(m, f->GetMTime());
    if (b->GetProperty() == ap || m != t.copied || !(look == t.look)) {
      vtkSmartPointer<vtkVolumeProperty> q = vtkSmartPointer<vtkVolumeProperty>::New();
      q->DeepCopy(ap);
      if (vtkPiecewiseFunction *f = ap->GetScalarOpacity()) {
        vtkSmartPointer<vtkPiecewiseFunction> g = vtkSmartPointer<vtkPiecewiseFunction>::New();
        g->DeepCopy(f);
        for (int i = 0; i < g->GetSize(); ++i) {
          double node[4];
          g->GetNodeValue(i, node);
          node[1] = std::clamp(node[1] * look.opacity, 0.0, 1.0);
          g->SetNodeValue(i, node);
        }
        q->SetScalarOpacity(g);
      }
      b->SetProperty(q);
      t.copied = m;
      t.look = look;
    }
    return b;
  }
};

namespace {

// A tag list: a YAML sequence, or one string of names split at commas and spaces.
std::vector<std::string> tag_list(const ari::Value *v) {
  std::vector<std::string> out;
  if (!v)
    return out;
  auto add_words = [&out](const std::string &s) {
    std::string cur;
    for (char c : s + " ") {
      if (c == ',' || c == ' ' || c == '\t') {
        if (!cur.empty())
          out.push_back(cur);
        cur.clear();
      } else {
        cur += c;
      }
    }
  };
  if (v->is_seq()) {
    for (const ari::Value &i : v->items)
      if (i.is_scalar())
        add_words(i.scalar);
  } else if (v->is_scalar()) {
    add_words(v->scalar);
  }
  return out;
}

bool has_any(const std::vector<std::string> &tags, const std::vector<std::string> &names) {
  for (const std::string &t : tags)
    if (std::find(names.begin(), names.end(), t) != names.end())
      return true;
  return false;
}

// Whether a node with `tags` is in a view that shows `show` (every tag when empty) and hides
// `hide`. A node's tags hold for its subtree, so a node out of the view takes its children along;
// an untagged node is in every view.
bool in_view(const std::vector<std::string> &tags, const std::vector<std::string> &show,
             const std::vector<std::string> &hide) {
  if (tags.empty())
    return true;
  if (has_any(tags, hide))
    return false;
  return show.empty() || has_any(tags, show);
}

void collect(const std::shared_ptr<GraphicsNode> &n, std::unordered_set<vtkProp *> &out,
             bool subtree = true) {
  if (!n)
    return;
  std::vector<vtkProp *> props;
  n->collectViewProps(props, subtree);
  out.insert(props.begin(), props.end());
}

// One override: entry's look: { drawstyle: surface | wireframe | points, opacity, color: [r, g, b],
// line_width, point_size }.
Look look_of(const ari::Value &v) {
  Look l;
  if (!v.is_map())
    return l;
  l.replace = v.str("mode") == "replace";
  // transform: { position: [x, y, z], rotation: [rx, ry, rz] (degrees), scale: s | [sx, sy, sz] },
  // as a node's transform: spells it (translate, then rotate z, y, x, then scale).
  if (const ari::Value *tf = v.find("transform"))
    if (tf->is_map()) {
      auto vec = [tf](const char *key, double def, double out[3]) {
        out[0] = out[1] = out[2] = def;
        if (const ari::Value *x = tf->find(key)) {
          if (x->is_seq() && x->items.size() >= 3)
            for (int i = 0; i < 3; ++i)
              out[i] = x->items[i].as_double();
          else if (x->is_scalar())
            out[0] = out[1] = out[2] = x->as_double();
        }
      };
      double pos[3], rot[3], scl[3];
      vec("position", 0.0, pos);
      vec("rotation", 0.0, rot);
      vec("scale", 1.0, scl);
      vtkNew<vtkTransform> t;
      t->Translate(pos);
      t->RotateZ(rot[2]);
      t->RotateY(rot[1]);
      t->RotateX(rot[0]);
      t->Scale(scl);
      const double *d = t->GetMatrix()->GetData();
      std::copy(d, d + 16, l.xform);
      l.has_xform = true;
    }
  const std::string ds = v.str("drawstyle");
  if (ds == "wireframe")
    l.rep = VTK_WIREFRAME;
  else if (ds == "points")
    l.rep = VTK_POINTS;
  else if (ds == "surface")
    l.rep = VTK_SURFACE;
  l.opacity = v.num("opacity", -1.0);
  l.line_width = v.num("line_width", -1.0);
  l.point_size = v.num("point_size", -1.0);
  if (const ari::Value *c = v.find("color"))
    if (c->is_seq() && c->items.size() >= 3) {
      l.has_color = true;
      for (int i = 0; i < 3; ++i)
        l.color[i] = c->items[i].as_double();
    }
  // material: { color, opacity } spells the same fields, as a node's material: does.
  if (const ari::Value *m = v.find("material"))
    if (m->is_map()) {
      Look ml = look_of(*m);
      ml.replace = false;
      ml.has_xform = false;
      l.merge(ml);
    }
  return l;
}

bool same_start(const ari::SceneCamera &a, const ari::SceneCamera &b) {
  return a.mode == b.mode && a.track == b.track && a.has_azimuth == b.has_azimuth &&
         a.azimuth == b.azimuth && a.has_elevation == b.has_elevation &&
         a.elevation == b.elevation && a.zoom == b.zoom;
}

// Frame `cam` on the bounds b as `s` says: a top-down map, or an orbit (then fly or track) that
// starts at s's azimuth / elevation / zoom, set through the camera's own state keys (what a drag
// changes).
void frame_camera(CameraController &cam, const double b[6], const ari::SceneCamera &s,
                  SceneGraph &sg) {
  if (s.mode == "map") {
    const double hw = std::max(0.5 * (b[1] - b[0]), 1e-6);
    const double hh = std::max(0.5 * (b[3] - b[2]), 1e-6);
    cam.frameMap(0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]), hh, hw);
    return;
  }
  cam.frameBounds(b[0], b[2], b[4], b[1], b[3], b[5]);
  cam.setMode(CameraController::Mode::Orbit); // from a map, say
  if (s.has_azimuth)
    cam.getState("orbit.azimuth").value(s.azimuth);
  if (s.has_elevation)
    cam.getState("orbit.elevation").value(s.elevation);
  if (s.zoom > 0.0 && s.zoom != 1.0)
    cam.getState("orbit.distance").value(cam.orbitDistance() / s.zoom);
  if (s.mode == "fly") {
    cam.setMode(CameraController::Mode::Fly);
  } else if (s.mode == "track" && !s.track.empty()) {
    cam.setScene(&sg);
    cam.setTrackTarget(s.track);
    cam.setMode(CameraController::Mode::Track);
  }
}

bool valid_bounds(const double b[6]) { return b[0] <= b[1] && b[2] <= b[3] && b[4] <= b[5]; }

// The bounds of the visible 3-D props among `props`; false when none has any.
bool props_bounds(const std::unordered_set<vtkProp *> &props, double out[6]) {
  bool any = false;
  for (vtkProp *p : props) {
    if (!p || !p->GetVisibility())
      continue;
    const double *b = p->GetBounds();
    if (!b || !valid_bounds(b))
      continue;
    for (int i = 0; i < 3; ++i) {
      out[2 * i] = any ? std::min(out[2 * i], b[2 * i]) : b[2 * i];
      out[2 * i + 1] = any ? std::max(out[2 * i + 1], b[2 * i + 1]) : b[2 * i + 1];
    }
    any = true;
  }
  return any;
}

// The world bounds of the tagged nodes `show` / `hide` let in, with their subtrees (the untagged
// left out): what a view, or the main view, that names tags to show is framed on. False when
// none has any.
bool shown_bounds(const RealizedScene &rs, const std::vector<std::string> &show,
                  const std::vector<std::string> &hide, double out[6]) {
  bool any = false;
  for (const RealizedScene::TaggedNode &t : rs.nodes) {
    if (t.tags.empty() || !in_view(t.tags, show, hide))
      continue;
    std::shared_ptr<GraphicsNode> n = t.node.lock();
    if (!n)
      continue;
    const cvc::bounding_box bb = n->getCombinedWorldBoundingBox();
    if (bb.isNull())
      continue;
    const double b[6] = {bb.minx, bb.maxx, bb.miny, bb.maxy, bb.minz, bb.maxz};
    for (int i = 0; i < 3; ++i) {
      out[2 * i] = any ? std::min(out[2 * i], b[2 * i]) : b[2 * i];
      out[2 * i + 1] = any ? std::max(out[2 * i + 1], b[2 * i + 1]) : b[2 * i + 1];
    }
    any = true;
  }
  return any;
}

} // namespace

// A named camera: its own vtkCamera, which every viewport bound to it draws from, and the
// controller a drag in any of them moves. The controller reads its aspect from the first viewport
// bound to it (`driver`).
struct SceneViews::Camera {
  std::string scene; // the scene that declares it
  ari::SceneCamera spec;
  vtkSmartPointer<vtkCamera> cam;
  std::unique_ptr<CameraController> ctl;
  vtkRenderer *driver = nullptr;
  bool framed = false;
  bool used = false; // a shown view drew from it this frame
};

SceneViews::SceneViews(SceneRenderer &view, ImGuiBackend &backend,
                       std::function<bool(const std::string &, SceneSlot &)> scenes,
                       CameraController *main)
    : view_(&view), backend_(&backend), scenes_(std::move(scenes)), main_(main) {}

SceneViews::~SceneViews() {
  clear();
  if (!view_->isClosed())
    if (vtkRenderer *r = view_->renderer()) {
      if (main_start_tag_)
        r->RemoveObserver(main_start_tag_);
      if (main_end_tag_)
        r->RemoveObserver(main_end_tag_);
    }
}

void SceneViews::main_view_begin() {
  vtkRenderer *r = view_->renderer();
  for (void *q : main_hidden_) {
    vtkProp *p = static_cast<vtkProp *>(q);
    if (r && r->GetViewProps()->IsItemPresent(p)) {
      p->Register(nullptr); // alive until it is back
      r->RemoveViewProp(p);
      main_removed_.push_back(p);
    }
  }
}

void SceneViews::main_view_end() {
  vtkRenderer *r = view_->renderer();
  for (void *q : main_removed_) {
    vtkProp *p = static_cast<vtkProp *>(q);
    if (r)
      r->AddViewProp(p);
    p->UnRegister(nullptr);
  }
  main_removed_.clear();
}

void SceneViews::clear() {
  main_hidden_.clear();
  main_framed_ = nullptr;
  if (view_->isClosed()) { // the viewports went with the view's manager
    views_.clear();
    cameras_.clear();
    anchors_on_.clear();
    return;
  }
  ViewportManager &vm = view_->viewportManager();
  for (const std::string &a : anchors_on_)
    if (vm.hasViewport(a))
      vm.viewport(a).setVisible(false);
  anchors_on_.clear();
  for (auto &kv : views_) {
    View &v = *kv.second;
    if (v.shadow_pass) { // its shadow pass's GL objects, while the window is still there
      v.shadow_pass->ReleaseGraphicsResources(vm.renderWindow());
      v.vp->renderer()->SetPass(nullptr);
    }
    if (vm.hasViewport(kv.first))
      vm.removeViewport(kv.first);
  }
  views_.clear();
  cameras_.clear();
}

SceneViews::Camera *SceneViews::named_camera(const std::string &scene, const std::string &name) {
  const std::string key = scene + "|" + name;
  auto it = cameras_.find(key);
  if (it != cameras_.end())
    return it->second.get();
  SceneSlot slot;
  if (!scenes_ || !scenes_(scene, slot) || !slot.realized || !slot.sg)
    return nullptr;
  for (const ari::SceneCamera &s : slot.realized->cameras) {
    if (s.id != name)
      continue;
    SceneGraph &sg = *slot.sg;
    auto c = std::make_unique<Camera>();
    c->scene = scene;
    c->spec = s;
    c->cam = vtkSmartPointer<vtkCamera>::New();
    c->ctl = std::make_unique<CameraController>(
        sg.appContext(), CameraController::viewerStatePath(sg.getStatePrefix(), name));
    c->ctl->setCamera(c->cam);
    c->ctl->setScene(&sg);
    Camera *raw = c.get();
    cameras_[key] = std::move(c);
    return raw;
  }
  return nullptr;
}

void SceneViews::bind_camera(View &v, const std::string &name) {
  if (name == v.camera)
    return;
  vtkRenderer *r = v.vp->renderer();
  if (!r)
    return;
  vtkCamera *target = nullptr;
  std::string bound = name;
  if (name == "main" && main_) {
    target = view_->renderer() ? view_->renderer()->GetActiveCamera() : nullptr;
  } else if (!name.empty()) {
    if (Camera *c = named_camera(v.scene, name)) {
      target = c->cam;
      if (!c->driver) {
        c->ctl->setRenderer(r);
        c->driver = r;
      }
    }
  }
  if (!target) { // its own camera: "", or a name the scene does not declare
    bound.clear();
    target = v.own;
  }
  if (!v.own)
    v.own = r->GetActiveCamera(); // kept: its own controller still points at it
  if (target && target != r->GetActiveCamera())
    r->SetActiveCamera(target);
  v.camera = bound;
}

bool SceneViews::pick_at(View &v, double x, double y, PickHit &hit) const {
  hit = PickHit{};
  vtkRenderer *ren = v.vp ? v.vp->renderer() : nullptr;
  if (!ren)
    return false;
  // What the scene's nodes draw, each node's own props by prop, and an override's stand-in by its
  // original's: the pick sees only these, never the chrome (grid, axes, bounds).
  std::unordered_map<vtkProp *, std::string> node_of;
  SceneSlot slot;
  if (scenes_ && scenes_(v.scene, slot) && slot.realized)
    for (const RealizedScene::TaggedNode &t : slot.realized->nodes) {
      std::unordered_set<vtkProp *> own;
      collect(t.node.lock(), own, false);
      for (vtkProp *p : own)
        node_of.emplace(p, t.id);
    }
  for (const auto &t : v.twins) {
    const auto it = node_of.find(t.first);
    if (it == node_of.end())
      continue;
    if (t.second.actor)
      node_of.emplace(t.second.actor.GetPointer(), it->second);
    if (t.second.volume)
      node_of.emplace(t.second.volume.GetPointer(), it->second);
  }
  // A real geometry hit test, as SceneRenderer::pickWorld: a miss over empty space is a miss.
  vtkNew<vtkCellPicker> picker;
  picker->SetTolerance(0.0005);
  picker->PickFromListOn();
  vtkPropCollection *props = ren->GetViewProps();
  vtkCollectionSimpleIterator it;
  props->InitTraversal(it);
  int listed = 0;
  while (vtkProp *p = props->GetNextProp(it))
    if (node_of.count(p)) {
      picker->AddPickList(p);
      ++listed;
    }
  if (listed == 0 || !picker->Pick(x, y, 0.0, ren) || picker->GetCellId() < 0)
    return false;
  picker->GetPickPosition(hit.world);
  hit.hit = true;
  if (vtkProp *prop = picker->GetViewProp()) {
    const auto n = node_of.find(prop);
    if (n != node_of.end())
      hit.node = n->second;
  }
  return true;
}

void SceneViews::pick(View &v, const char *phase, ari::CustomEdit &edit) {
#ifdef CVC_ENABLE_IMGUI
  const ImGuiIO &io = ImGui::GetIO();
  // The pointer in VTK display coordinates: framebuffer pixels from the lower left.
  const double x = io.MousePos.x * io.DisplayFramebufferScale.x;
  const double y = (io.DisplaySize.y - io.MousePos.y) * io.DisplayFramebufferScale.y;
  PickHit hit;
  pick_at(v, x, y, hit);
  std::string mods;
  for (const auto &[on, name] : {std::pair<bool, const char *>{io.KeyShift, "shift"},
                                 {io.KeyCtrl, "ctrl"},
                                 {io.KeyAlt, "alt"},
                                 {io.KeySuper, "super"}})
    if (on)
      mods += (mods.empty() ? "" : ",") + std::string(name);
  const auto num = [](double d) {
    char b[64];
    std::snprintf(b, sizeof b, "%.9g", d);
    return std::string(b);
  };
  std::vector<std::pair<std::string, std::string>> f = {{"x", hit.hit ? num(hit.world[0]) : ""},
                                                        {"y", hit.hit ? num(hit.world[1]) : ""},
                                                        {"z", hit.hit ? num(hit.world[2]) : ""},
                                                        {"hit", hit.hit ? "1" : "0"},
                                                        {"node", hit.node},
                                                        {"button", "1"},
                                                        {"mods", mods},
                                                        {"phase", phase},
                                                        {"view", v.name},
                                                        {"seq", std::to_string(++v.pick_seq)}};
  SceneSlot slot;
  if (!v.pick_key.empty() && scenes_ && scenes_(v.scene, slot) && slot.sg) {
    cvc::app &app = slot.sg->appContext();
    for (const auto &kv : f) // seq last: a watcher on it sees the rest written
      ari::write<std::string>(app, v.pick_key + "." + kv.first, kv.second);
  }
  if (!v.on_pick.empty()) {
    edit.fire = v.on_pick;
    edit.fields = std::move(f);
  }
#else
  (void)v, (void)phase, (void)edit;
#endif
}

bool SceneViews::pick(const std::string &view, double x, double y, PickHit &hit) const {
  const auto it = views_.find(view);
  return it != views_.end() && pick_at(*it->second, x, y, hit);
}

CameraController &SceneViews::controller(View &v) {
  if (v.camera == "main" && main_)
    return *main_;
  if (!v.camera.empty()) {
    auto it = cameras_.find(v.scene + "|" + v.camera);
    if (it != cameras_.end())
      return *it->second->ctl;
  }
  return v.vp->camera();
}

SceneViews::View *SceneViews::view_for(const std::string &name, const std::string &scene) {
  ViewportManager &vm = view_->viewportManager();
  auto it = views_.find(name);
  if (it != views_.end()) {
    if (it->second->scene == scene)
      return it->second.get();
    // It shows another scene now: a new mirror of that one.
    View &old = *it->second;
    if (old.shadow_pass)
      old.shadow_pass->ReleaseGraphicsResources(vm.renderWindow());
    if (vm.hasViewport(name))
      vm.removeViewport(name);
    views_.erase(it);
  }
  if (vm.hasViewport(name)) // the host's own viewport of that name: not the widget's to drive
    return nullptr;
  SceneSlot slot;
  if (!scenes_ || !scenes_(scene, slot) || slot.source.empty() || !vm.hasViewport(slot.source))
    return nullptr;
  auto v = std::make_unique<View>();
  v->name = name;
  v->scene = scene;
  const double hidden_region[4] = {0.0, 0.0, 0.0, 0.0};
  // Over the main view (layer 1), and over a scene's own viewport (layer 1, a pixel of it drawn
  // under one of its views): layer 2.
  Viewport &vp =
      vm.addMirrorViewport(name, slot.source, hidden_region, scene.empty() ? 1 : 2, true);
  vp.setVisible(false);
  vp.setInputEnabled(false); // the widget feeds its camera; the manager's router never does
  // Its scene's background, until the widget says otherwise.
  if (vtkRenderer *main = vm.viewport(slot.source).renderer()) {
    double bg[3];
    main->GetBackground(bg);
    vp.setBackground(bg[0], bg[1], bg[2], true);
    if (vtkRenderer *r = vp.renderer()) {
      r->SetGradientBackground(main->GetGradientBackground());
      double bg2[3];
      main->GetBackground2(bg2);
      r->SetBackground2(bg2);
    }
  }
  View *raw = v.get();
  vp.setMirrorFilter([raw](vtkProp *p) { return raw->draw(p); });
  v->vp = &vp;
  views_[name] = std::move(v);
  return raw;
}

void SceneViews::end_walk() {
  walked_ = true;
  // A scenes: entry's own viewport bakes its shadow maps when it draws, so while one of its views
  // shows and it casts shadows, draw one pixel of it in a corner of that view, which draws over it.
  std::vector<std::string> anchors;
  ViewportManager &vm = view_->viewportManager();
  int W = 0, H = 0;
  if (vtkRenderWindow *win = vm.renderWindow()) {
    const int *sz = win->GetSize();
    W = sz[0];
    H = sz[1];
  }
  for (auto &kv : views_) {
    View &v = *kv.second;
    if (!v.drawn || v.scene.empty() || W <= 0 || H <= 0)
      continue;
    SceneSlot slot;
    if (!scenes_ || !scenes_(v.scene, slot) || !slot.anchor || !slot.sg ||
        !slot.sg->shadowsEnabled() || !vm.hasViewport(slot.source))
      continue;
    if (std::find(anchors.begin(), anchors.end(), slot.source) != anchors.end())
      continue;
    Viewport &a = vm.viewport(slot.source);
    a.setRegion(v.region[0], v.region[1], v.region[0] + 1.0 / W, v.region[1] + 1.0 / H);
    if (!a.visible()) {
      // Shown again after frames unseen: bake its maps on this frame, whatever its stride, so the
      // views never draw maps from before (the anchor draws first, a layer under them).
      a.setVisible(true);
      slot.sg->invalidateShadowBake();
    }
    anchors.push_back(slot.source);
  }
  for (const std::string &old : anchors_on_)
    if (std::find(anchors.begin(), anchors.end(), old) == anchors.end() && vm.hasViewport(old))
      vm.viewport(old).setVisible(false);
  anchors_on_ = std::move(anchors);
  for (auto &kv : views_) {
    View &v = *kv.second;
    if (!v.drawn && v.vp->visible())
      v.vp->setVisible(false);
    if (!v.drawn && (v.dragging || v.panning)) { // its window went away mid-drag
      if (v.panning)
        controller(v).endPan();
      else
        controller(v).endDrag();
      v.dragging = v.panning = false;
    }
    v.drawn = false;
  }
}

void SceneViews::update(double dt) {
  if (view_->isClosed()) {
    views_.clear();
    cameras_.clear();
    anchors_on_.clear();
    return;
  }
  // No widget walk last frame (the host hid the UI): no widget drew a view, so none shows, and the
  // scenes: entries' viewports, drawn only under a view, stop too.
  const bool walked = walked_;
  walked_ = false;
  if (!walked) {
    ViewportManager &vm = view_->viewportManager();
    for (const std::string &a : anchors_on_)
      if (vm.hasViewport(a))
        vm.viewport(a).setVisible(false);
    anchors_on_.clear();
  }
  // main_view: { show, hide }: what the main view leaves out, taken out of the main renderer for
  // the length of its draw (main_view_begin / _end), so the views, which mirror it, keep it.
  main_hidden_.clear();
  SceneSlot mainSlot;
  if (scenes_ && scenes_("", mainSlot) && mainSlot.realized && mainSlot.realized->has_main_view) {
    std::unordered_set<vtkProp *> out;
    for (const RealizedScene::TaggedNode &t : mainSlot.realized->nodes)
      if (!in_view(t.tags, mainSlot.realized->main_show, mainSlot.realized->main_hide))
        collect(t.node.lock(), out);
    main_hidden_.assign(out.begin(), out.end());
    if (!main_start_tag_)
      if (vtkRenderer *r = view_->renderer()) {
        main_start_tag_ =
            r->AddObserver(vtkCommand::StartEvent, this, &SceneViews::main_view_begin);
        main_end_tag_ = r->AddObserver(vtkCommand::EndEvent, this, &SceneViews::main_view_end);
      }
    // The host framed the main camera on the whole scene; once per document, frame it on what the
    // main view draws: the tagged nodes it shows (or all it draws, when it names none to show).
    if (main_ && main_framed_ != mainSlot.realized) {
      double b[6];
      bool ok = false;
      if (!mainSlot.realized->main_show.empty())
        ok = shown_bounds(*mainSlot.realized, mainSlot.realized->main_show,
                          mainSlot.realized->main_hide, b);
      if (!ok)
        if (vtkRenderer *r = view_->renderer()) {
          std::unordered_set<vtkProp *> drawn;
          vtkPropCollection *props = r->GetViewProps();
          vtkCollectionSimpleIterator it;
          props->InitTraversal(it);
          while (vtkProp *p = props->GetNextProp(it))
            if (!out.count(p))
              drawn.insert(p);
          ok = props_bounds(drawn, b);
        }
      if (ok)
        main_->frameBounds(b[0], b[2], b[4], b[1], b[3], b[5]);
      main_framed_ = mainSlot.realized;
    }
  }
  for (auto &kv : views_) {
    View &v = *kv.second;
    if (!walked && v.vp->visible())
      v.vp->setVisible(false);
    SceneSlot slot;
    if (!scenes_ || !scenes_(v.scene, slot) || !slot.sg)
      continue;
    SceneGraph &sg = *slot.sg;
    const RealizedScene *rs = slot.realized;
    // What it leaves out: the tagged nodes its show: / hide: rule out, and the chrome.
    v.hidden.clear();
    if (rs && (!v.show.empty() || !v.hide.empty()))
      for (const RealizedScene::TaggedNode &t : rs->nodes)
        if (!in_view(t.tags, v.show, v.hide))
          collect(t.node.lock(), v.hidden);
    // How it restyles the rest: each override: entry's look on the props of the nodes it names (by
    // id or tag, with their subtrees), in order; a transform applies in the named node's frame.
    v.entries.clear();
    for (const auto &o : v.overrides) {
      Entry e;
      e.look = o.second;
      e.all = o.first == "*";
      if (!e.all && rs)
        for (const RealizedScene::TaggedNode &t : rs->nodes) {
          if (t.id != o.first && std::find(t.tags.begin(), t.tags.end(), o.first) == t.tags.end())
            continue;
          std::shared_ptr<GraphicsNode> n = t.node.lock();
          if (!n)
            continue;
          std::array<double, 16> frame;
          identity(frame.data());
          if (o.second.has_xform)
            if (vtkSmartPointer<vtkMatrix4x4> w = n->getWorldTransform())
              std::copy(w->GetData(), w->GetData() + 16, frame.begin());
          std::unordered_set<vtkProp *> props;
          collect(n, props);
          for (vtkProp *p : props)
            e.props.emplace(p, frame); // an outer named node's frame wins over one inside it
        }
      v.entries.push_back(std::move(e));
    }
    // Its volumes, which a view from a camera of its own draws as its own eye needs.
    v.own_eye = v.camera != "main";
    v.volumes.clear();
    if (rs)
      for (const RealizedScene::TaggedNode &t : rs->nodes) {
        std::shared_ptr<GraphicsNode> n = t.node.lock();
        VolumeProp vol;
        if (auto *sl = dynamic_cast<VolSliceNode *>(n.get()))
          vol = {VolumeKind::slices, sl};
        else if (auto *rc = dynamic_cast<VolRenNode *>(n.get()))
          vol = {VolumeKind::raycast, rc};
        else if (dynamic_cast<VolumeNode *>(n.get()))
          vol = {VolumeKind::gpu, n.get()};
        else
          continue;
        if (vtkProp *own = n->prop())
          v.volumes[own] = vol;
      }
    // The twins of props no longer drawn (a node removed, an override dropped) go.
    for (auto it = v.twins.begin(); it != v.twins.end();) {
      if (!it->second.seen)
        it = v.twins.erase(it);
      else
        (it++)->second.seen = false;
    }
    // The scene's shadows: a pass reading the maps its scene's renderer bakes, or a chain of its
    // own that bakes what this view draws, remade when the scene's shadows change.
    const std::pair<const void *, double> key = v.shadow_mode != "off"
                                                    ? sg.shadowReceiverKey()
                                                    : std::pair<const void *, double>(nullptr, 0.0);
    if (key != v.shadow_key || v.shadow_mode != v.shadow_built) {
      vtkRenderer *r = v.vp->renderer();
      if (v.shadow_pass)
        v.shadow_pass->ReleaseGraphicsResources(view_->viewportManager().renderWindow());
      v.shadow_pass = !key.first               ? nullptr
                      : v.shadow_mode == "own" ? sg.makeShadowPasses()
                                               : sg.makeShadowReceiverPass();
      r->SetPass(v.shadow_pass);
      v.shadow_key = key;
      v.shadow_built = v.shadow_mode;
    }
    if (!v.chrome) {
      if (std::shared_ptr<GraphicsNode> root = sg.getGraphicsRoot()) {
        collect(root->findChildByName("grid"), v.hidden);
        collect(root->findChildByName("axis"), v.hidden);
        collect(root, v.hidden, false); // the root's own bounding box, not the scene under it
      }
    }
    if (!v.camera.empty()) { // a named camera, moved below (or the main one, by its host)
      auto it = cameras_.find(v.scene + "|" + v.camera);
      if (it != cameras_.end() && v.vp->visible())
        it->second->used = true;
      continue;
    }
    CameraController &cam = v.vp->camera();
    // Framed once it has drawn something: the bounds of what this view shows (the tagged nodes it
    // names, when it names some to show).
    if (!v.framed && v.vp->visible()) {
      double b[6];
      vtkRenderer *r = v.vp->renderer();
      if (r && r->GetViewProps()->GetNumberOfItems() > 0) {
        if (v.show.empty() || !rs || !shown_bounds(*rs, v.show, v.hide, b))
          r->ComputeVisiblePropBounds(b);
        if (valid_bounds(b)) {
          frame_camera(cam, b, v.start, sg);
          v.framed = true;
        }
      }
    }
    cam.update(dt);
  }

  // The named cameras: each framed on its whole scene the first time a view shows it, then moved
  // once a frame however many views draw from it. Its near / far planes span the whole scene, so
  // no view that draws from it clips what it shows.
  ViewportManager &vm = view_->viewportManager();
  for (auto &kv : cameras_) {
    Camera &c = *kv.second;
    double whole[6] = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};
    SceneSlot slot;
    if (scenes_ && scenes_(c.scene, slot) && vm.hasViewport(slot.source))
      if (vtkRenderer *src = vm.viewport(slot.source).renderer())
        src->ComputeVisiblePropBounds(whole);
    if (!c.framed && c.used && c.driver && valid_bounds(whole) && slot.sg) {
      frame_camera(*c.ctl, whole, c.spec, *slot.sg);
      c.framed = true;
    }
    c.ctl->update(dt);
    if (c.driver && c.driver->GetActiveCamera() == c.cam && valid_bounds(whole))
      c.driver->ResetCameraClippingRange(whole);
    c.used = false;
  }
}

void SceneViews::register_widget() {
#ifdef CVC_ENABLE_IMGUI
  backend_->register_widget(
      "viewport", [this](const std::string &current, const ari::Widget &w) -> ari::CustomEdit {
        std::string name = w.props.str("view");
        if (name.empty())
          name = w.id.empty() ? std::string("view") : w.id;
        const std::string scene = w.props.str("scene"); // "" the document's scene: block
        View *v = view_for(name, scene);
        if (!v) {
          if (!scene.empty())
            ImGui::TextDisabled("[viewport %s: no scene '%s']", name.c_str(), scene.c_str());
          else
            ImGui::TextDisabled("[viewport %s: the host has a viewport of that name]",
                                name.c_str());
          return {};
        }
        v->show = tag_list(w.props.find("show"));
        v->hide = tag_list(w.props.find("hide"));
        if (!w.bind.empty()) { // a bound view shows the tags its state key holds
          ari::Value bound;
          bound.scalar = current;
          v->show = tag_list(&bound);
        }
        v->chrome = w.props.flag("chrome", true);
        // input: true (drags turn the camera) | false | pick (the left button picks).
        v->pick = w.props.str("input") == "pick";
        v->input = v->pick || w.props.flag("input", true);
        v->on_pick = w.props.str("on_pick");
        v->pick_key.clear();
        if (v->pick) {
          SceneSlot slot;
          if (scenes_ && scenes_(v->scene, slot) && slot.sg)
            v->pick_key = ari::resolve_bind(slot.sg->getStatePrefix(),
                                            w.props.str("pick", "viewers." + name + ".pick"));
        }
        // Its camera: `camera: <id>` draws from a named camera (or `main`); otherwise its own,
        // which `mode:` (orbit | fly | map | track), `track:`, `azimuth:`, `elevation:` and `zoom:`
        // start. `camera: orbit` and the like name a mode, as #1005 spelled it.
        std::string camera = w.props.str("camera");
        ari::SceneCamera start;
        start.mode = w.props.str("mode", "orbit");
        if (camera == "orbit" || camera == "fly" || camera == "map" || camera == "track") {
          if (!w.props.has("mode"))
            start.mode = camera;
          camera.clear();
        }
        start.track = w.props.str("track");
        start.has_azimuth = w.props.has("azimuth");
        start.azimuth = w.props.num("azimuth", 0.0);
        start.has_elevation = w.props.has("elevation");
        start.elevation = w.props.num("elevation", 0.0);
        start.zoom = w.props.num("zoom", 1.0);
        if (!same_start(start, v->start)) {
          v->start = start;
          v->framed = false;
        }
        bind_camera(*v, camera);
        // shadows: true (the scene's maps) | own (a bake of what it draws) | false
        const std::string shadows = w.props.str("shadows", "true");
        v->shadow_mode =
            shadows == "own" ? "own"
            : (shadows == "false" || shadows == "0" || shadows == "off" || shadows == "no")
                ? "off"
                : "shared";
        // override: { <node id | tag | "*">: { drawstyle, opacity, color, line_width,
        // point_size }, ... }, in order.
        v->overrides.clear();
        if (const ari::Value *ov = w.props.find("override"))
          if (ov->is_map())
            for (const auto &e : ov->entries)
              v->overrides.emplace_back(e.first, look_of(e.second));
        if (const ari::Value *bg = w.props.find("clear_color"))
          if (bg->is_seq() && bg->items.size() >= 3) {
            v->vp->setBackground(bg->items[0].as_double(), bg->items[1].as_double(),
                                 bg->items[2].as_double(), true);
            if (vtkRenderer *vr = v->vp->renderer()) // one colour, not the main view's gradient
              vr->SetGradientBackground(false);
          }

        // Its rect: `width` (logical px; the window's width when unset) by `height` (3/4 of the
        // width when unset).
        const ImGuiStyle &style = ImGui::GetStyle(); // a logical px at the UI's size (ui_px)
        const float unit = style.FontScaleMain * style.FontScaleDpi;
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        float wpx = static_cast<float>(w.props.num("width", 0.0)) * unit;
        if (!(wpx > 0.0f))
          wpx = std::max(avail.x, 32.0f);
        float hpx = static_cast<float>(w.props.num("height", 0.0)) * unit;
        if (!(hpx > 0.0f))
          hpx = wpx * 0.75f;
        const std::string id = "##viewport." + name;
        ImGui::InvisibleButton(id.c_str(), ImVec2(wpx, hpx),
                               ImGuiButtonFlags_MouseButtonLeft |
                                   ImGuiButtonFlags_MouseButtonRight |
                                   ImGuiButtonFlags_MouseButtonMiddle);
        ImRect r(ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
        r.ClipWithFull(ImGui::GetCurrentWindow()->ClipRect); // the part that shows
        const ImVec2 disp = ImGui::GetIO().DisplaySize;
        if (r.GetWidth() < 1.0f || r.GetHeight() < 1.0f || disp.x <= 0.0f || disp.y <= 0.0f)
          return {}; // scrolled out of sight: not drawn this frame
        backend_->cut_window_background(r.Min.x, r.Min.y, r.Max.x, r.Max.y);
        // Normalized, in VTK's y-up window space.
        const double reg[4] = {r.Min.x / disp.x, 1.0 - r.Max.y / disp.y, r.Max.x / disp.x,
                               1.0 - r.Min.y / disp.y};
        bool moved = false;
        for (int i = 0; i < 4; ++i)
          moved = moved || std::fabs(reg[i] - v->region[i]) > 1e-6;
        if (moved) { // setRegion writes the layout to state, so only when it moves
          v->vp->setRegion(reg[0], reg[1], reg[2], reg[3]);
          std::copy(reg, reg + 4, v->region);
        }
        if (!v->vp->visible())
          v->vp->setVisible(true);
        v->drawn = true;

        // Its caption in the corner, and an outline while the pointer is over it or drags it.
        ImDrawList *dl = ImGui::GetWindowDrawList();
        const std::string caption = w.props.str("caption");
        if (!caption.empty()) {
          const ImVec2 pad(4.0f * unit, 2.0f * unit);
          const ImVec2 at(r.Min.x + pad.x, r.Min.y + pad.y);
          const ImVec2 ts = ImGui::CalcTextSize(caption.c_str());
          dl->AddRectFilled(ImVec2(at.x - pad.x * 0.5f, at.y - pad.y * 0.5f),
                            ImVec2(at.x + ts.x + pad.x * 0.5f, at.y + ts.y + pad.y * 0.5f),
                            IM_COL32(0, 0, 0, 150), 3.0f * unit);
          dl->AddText(at, IM_COL32(235, 235, 235, 255), caption.c_str());
        }
        if (v->input && (ImGui::IsItemHovered() || ImGui::IsItemActive()))
          dl->AddRect(r.Min, r.Max, ImGui::GetColorU32(ImGuiCol_NavHighlight), 0.0f, 0,
                      ImGui::IsItemActive() ? 2.0f * unit : 1.0f * unit);

        // Its camera: a drag orbits (the middle or right button pans), the wheel zooms. With input:
        // pick, the left button picks instead.
        ari::CustomEdit edit;
        if (v->input) {
          cvc::gl::keepTouchDrag(); // a finger's drag here turns the camera, not the window
          CameraController &cam = controller(*v);
          const ImGuiIO &io = ImGui::GetIO();
          if (ImGui::IsItemHovered() && io.MouseWheel != 0.0f)
            cam.mouseWheel(io.MouseWheel);
          if (!v->pick && ImGui::IsItemHovered() &&
              ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            v->framed = false; // back to where it started (a named camera, for all its views)
            auto it = cameras_.find(v->camera);
            if (it != cameras_.end())
              it->second->framed = false;
          }
          const bool moved_ptr = io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f;
          if (ImGui::IsItemActivated()) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && v->pick) {
              v->picking = true;
              pick(*v, "down", edit);
            } else if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
              cam.beginDrag();
              v->dragging = true;
            } else {
              cam.beginPan();
              v->panning = true;
            }
          } else if (v->picking && ImGui::IsItemActive() && moved_ptr) {
            pick(*v, "drag", edit);
          }
          if (ImGui::IsItemActive() && moved_ptr && (v->dragging || v->panning))
            cam.mouseLook(static_cast<int>(std::lround(io.MouseDelta.x)),
                          -static_cast<int>(std::lround(io.MouseDelta.y))); // VTK's y is up
          if (ImGui::IsItemDeactivated()) {
            if (v->picking)
              pick(*v, "up", edit);
            else if (v->panning)
              cam.endPan();
            else if (v->dragging)
              cam.endDrag();
            v->dragging = v->panning = v->picking = false;
          }
        }
        return edit;
      });
#endif
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
