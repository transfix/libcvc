/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// SceneViews — the `viewport` widget. See inc/cvc/gl/ariadne/scene_views.h.

#include <algorithm>
#include <cmath>
#include <cvc/ariadne/widget.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
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
#include <vtkMapper.h>
#include <vtkPropCollection.h>
#include <vtkProperty.h>
#include <vtkRenderPass.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkShaderProperty.h>
#include <vtkSmartPointer.h>

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
// them. Later entries win field by field.
struct Look {
  int rep = -1; // VTK_POINTS / VTK_WIREFRAME / VTK_SURFACE, -1 unset
  double opacity = -1.0, line_width = -1.0, point_size = -1.0;
  bool has_color = false;
  double color[3] = {1.0, 1.0, 1.0};
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
  vtkMTimeType copied = 0; // the original's property MTime when it was last copied
  Look look;               // the look last applied
  bool seen = false;       // drawn this frame
};

} // namespace

struct SceneViews::View {
  std::string name; // the viewport's, and its camera's viewer: <prefix>.viewers.<name>.camera
  Viewport *vp = nullptr;
  std::vector<std::string> show, hide; // tags; show empty = every tag
  bool chrome = true;                  // the scene's grid, axes and bounding box
  ari::SceneCamera start;              // its own camera: mode, track, where its orbit starts
  std::string camera;                  // the camera it draws from: "" its own, "main", or a name
  vtkSmartPointer<vtkCamera> own;      // its own vtkCamera, kept while it draws from another
  bool shadows = true;                 // it receives the scene's shadows
  std::pair<const void *, double> shadow_key{nullptr, -1.0}; // what shadow_pass was made from
  vtkSmartPointer<vtkRenderPass> shadow_pass;
  bool input = true;   // drags and the wheel move its camera
  bool drawn = false;  // a widget drew it this frame
  bool framed = false; // its camera has been framed on what it shows
  double region[4] = {-1.0, -1.0, -1.0, -1.0};
  std::unordered_set<vtkProp *> hidden; // the main view's props this one leaves out
  // override: entries in order: a node id, a tag, or "*" (everything), and its look.
  std::vector<std::pair<std::string, Look>> overrides;
  std::unordered_map<vtkProp *, Look> looks; // each restyled prop's merged look, this frame
  Look all;                                  // the "*" entry's look
  std::unordered_map<vtkProp *, Twin> twins;
  bool dragging = false, panning = false;

  // What the mirror draws for the main view's prop p (Viewport::setMirrorFilter).
  vtkProp *draw(vtkProp *p) {
    if (hidden.count(p))
      return nullptr;
    Look look = all;
    auto it = looks.find(p);
    if (it != looks.end())
      look.merge(it->second);
    vtkActor *a = vtkActor::SafeDownCast(p);
    if (look.empty() || !a || !a->GetMapper())
      return p;
    Twin &t = twins[p];
    if (!t.actor)
      t.actor = vtkSmartPointer<vtkActor>::New();
    vtkActor *b = t.actor;
    if (b->GetMapper() != a->GetMapper())
      b->SetMapper(a->GetMapper());
    b->SetShaderProperty(a->GetShaderProperty()); // the node's shader: replacements, shared
    b->SetTexture(a->GetTexture());
    b->SetPosition(a->GetPosition());
    b->SetOrientation(a->GetOrientation());
    b->SetScale(a->GetScale());
    b->SetOrigin(a->GetOrigin());
    b->SetUserMatrix(a->GetUserMatrix());
    b->SetUserTransform(a->GetUserTransform());
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
    b->SetForceTranslucent(see_through);
    t.seen = true;
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

} // namespace

// A named camera: its own vtkCamera, which every viewport bound to it draws from, and the
// controller a drag in any of them moves. The controller reads its aspect from the first viewport
// bound to it (`driver`).
struct SceneViews::Camera {
  ari::SceneCamera spec;
  vtkSmartPointer<vtkCamera> cam;
  std::unique_ptr<CameraController> ctl;
  vtkRenderer *driver = nullptr;
  bool framed = false;
  bool used = false; // a shown view drew from it this frame
};

SceneViews::SceneViews(SceneRenderer &view, ImGuiBackend &backend,
                       std::function<const RealizedScene *()> realized, CameraController *main)
    : view_(&view), backend_(&backend), realized_(std::move(realized)), main_(main) {}

SceneViews::~SceneViews() { clear(); }

void SceneViews::clear() {
  if (view_->isClosed()) { // the viewports went with the view's manager
    views_.clear();
    cameras_.clear();
    return;
  }
  ViewportManager &vm = view_->viewportManager();
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

SceneViews::Camera *SceneViews::named_camera(const std::string &name) {
  auto it = cameras_.find(name);
  if (it != cameras_.end())
    return it->second.get();
  const RealizedScene *rs = realized_ ? realized_() : nullptr;
  if (!rs)
    return nullptr;
  for (const ari::SceneCamera &s : rs->cameras) {
    if (s.id != name)
      continue;
    SceneGraph &sg = view_->scene();
    auto c = std::make_unique<Camera>();
    c->spec = s;
    c->cam = vtkSmartPointer<vtkCamera>::New();
    c->ctl = std::make_unique<CameraController>(
        sg.appContext(), CameraController::viewerStatePath(sg.getStatePrefix(), name));
    c->ctl->setCamera(c->cam);
    c->ctl->setScene(&sg);
    Camera *raw = c.get();
    cameras_[name] = std::move(c);
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
    if (Camera *c = named_camera(name)) {
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

CameraController &SceneViews::controller(View &v) {
  if (v.camera == "main" && main_)
    return *main_;
  if (!v.camera.empty()) {
    auto it = cameras_.find(v.camera);
    if (it != cameras_.end())
      return *it->second->ctl;
  }
  return v.vp->camera();
}

SceneViews::View *SceneViews::view_for(const std::string &name) {
  auto it = views_.find(name);
  if (it != views_.end())
    return it->second.get();
  ViewportManager &vm = view_->viewportManager();
  if (vm.hasViewport(name)) // the host's own viewport of that name: not the widget's to drive
    return nullptr;
  auto v = std::make_unique<View>();
  v->name = name;
  const double hidden_region[4] = {0.0, 0.0, 0.0, 0.0};
  Viewport &vp = vm.addMirrorViewport(name, vm.primary().name(), hidden_region, 1, true);
  vp.setVisible(false);
  vp.setInputEnabled(false); // the widget feeds its camera; the manager's router never does
  // The main view's background, until the widget says otherwise.
  if (vtkRenderer *main = view_->renderer()) {
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
    return;
  }
  SceneGraph &sg = view_->scene();
  // No widget walk last frame (the host hid the UI): no widget drew a view, so none shows.
  const bool walked = walked_;
  walked_ = false;
  const RealizedScene *rs = realized_ ? realized_() : nullptr;
  for (auto &kv : views_) {
    View &v = *kv.second;
    if (!walked && v.vp->visible())
      v.vp->setVisible(false);
    // What it leaves out: the tagged nodes its show: / hide: rule out, and the chrome.
    v.hidden.clear();
    if (rs && (!v.show.empty() || !v.hide.empty()))
      for (const RealizedScene::TaggedNode &t : rs->nodes)
        if (!in_view(t.tags, v.show, v.hide))
          collect(t.node.lock(), v.hidden);
    // How it restyles the rest: each override: entry's look on the props of the nodes it names (by
    // id or tag, with their subtrees), later entries winning field by field.
    v.looks.clear();
    v.all = Look{};
    for (const auto &o : v.overrides) {
      if (o.first == "*") {
        v.all.merge(o.second);
        continue;
      }
      if (!rs)
        continue;
      for (const RealizedScene::TaggedNode &t : rs->nodes) {
        if (t.id != o.first && std::find(t.tags.begin(), t.tags.end(), o.first) == t.tags.end())
          continue;
        std::unordered_set<vtkProp *> props;
        collect(t.node.lock(), props);
        for (vtkProp *p : props)
          v.looks[p].merge(o.second);
      }
    }
    // The twins of props no longer drawn (a node removed, an override dropped) go.
    for (auto it = v.twins.begin(); it != v.twins.end();) {
      if (!it->second.seen)
        it = v.twins.erase(it);
      else
        (it++)->second.seen = false;
    }
    // The scene's shadows: a pass reading the main renderer's maps, remade when they change.
    const std::pair<const void *, double> key =
        v.shadows ? sg.shadowReceiverKey() : std::pair<const void *, double>(nullptr, 0.0);
    if (key != v.shadow_key) {
      vtkRenderer *r = v.vp->renderer();
      if (v.shadow_pass)
        v.shadow_pass->ReleaseGraphicsResources(view_->viewportManager().renderWindow());
      v.shadow_pass = key.first ? sg.makeShadowReceiverPass() : nullptr;
      r->SetPass(v.shadow_pass);
      v.shadow_key = key;
    }
    if (!v.chrome) {
      if (std::shared_ptr<GraphicsNode> root = sg.getGraphicsRoot()) {
        collect(root->findChildByName("grid"), v.hidden);
        collect(root->findChildByName("axis"), v.hidden);
        collect(root, v.hidden, false); // the root's own bounding box, not the scene under it
      }
    }
    if (!v.camera.empty()) { // a named camera, moved below (or the main one, by its host)
      auto it = cameras_.find(v.camera);
      if (it != cameras_.end() && v.vp->visible())
        it->second->used = true;
      continue;
    }
    CameraController &cam = v.vp->camera();
    // Framed once it has drawn something: the bounds of what this view shows.
    if (!v.framed && v.vp->visible()) {
      double b[6];
      vtkRenderer *r = v.vp->renderer();
      if (r && r->GetViewProps()->GetNumberOfItems() > 0) {
        r->ComputeVisiblePropBounds(b);
        if (valid_bounds(b)) {
          frame_camera(cam, b, v.start, sg);
          v.framed = true;
        }
      }
    }
    cam.update(dt);
  }

  // The named cameras: each framed on the whole scene the first time a view shows it, then moved
  // once a frame however many views draw from it. Its near / far planes span the whole scene, so
  // no view that draws from it clips what it shows.
  double whole[6] = {1.0, -1.0, 1.0, -1.0, 1.0, -1.0};
  if (vtkRenderer *main = view_->renderer())
    main->ComputeVisiblePropBounds(whole);
  for (auto &kv : cameras_) {
    Camera &c = *kv.second;
    if (!c.framed && c.used && c.driver && valid_bounds(whole)) {
      frame_camera(*c.ctl, whole, c.spec, sg);
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
        View *v = view_for(name);
        if (!v) {
          ImGui::TextDisabled("[viewport %s: the host has a viewport of that name]", name.c_str());
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
        v->input = w.props.flag("input", true);
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
        v->shadows = w.props.flag("shadows", true);
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

        // Its camera: a drag orbits (the middle or right button pans), the wheel zooms.
        if (v->input) {
          CameraController &cam = controller(*v);
          const ImGuiIO &io = ImGui::GetIO();
          if (ImGui::IsItemHovered() && io.MouseWheel != 0.0f)
            cam.mouseWheel(io.MouseWheel);
          if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            v->framed = false; // back to where it started (a named camera, for all its views)
            auto it = cameras_.find(v->camera);
            if (it != cameras_.end())
              it->second->framed = false;
          }
          if (ImGui::IsItemActivated()) {
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
              cam.beginDrag();
              v->dragging = true;
            } else {
              cam.beginPan();
              v->panning = true;
            }
          }
          if (ImGui::IsItemActive() && (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f))
            cam.mouseLook(static_cast<int>(std::lround(io.MouseDelta.x)),
                          -static_cast<int>(std::lround(io.MouseDelta.y))); // VTK's y is up
          if (ImGui::IsItemDeactivated()) {
            if (v->panning)
              cam.endPan();
            else if (v->dragging)
              cam.endDrag();
            v->dragging = v->panning = false;
          }
        }
        return {};
      });
#endif
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
