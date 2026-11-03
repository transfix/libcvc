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
#include <unordered_set>
#include <vector>
#include <vtkRenderer.h>

#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // the window's clip rect: the part of the widget that shows
#endif

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

struct SceneViews::View {
  std::string name; // the viewport's, and its camera's viewer: <prefix>.viewers.<name>.camera
  Viewport *vp = nullptr;
  std::vector<std::string> show, hide; // tags; show empty = every tag
  bool chrome = true;                  // the scene's grid, axes and bounding box
  std::string camera = "orbit";        // orbit | fly | map | track
  std::string track;                   // the node a track camera follows
  bool input = true;                   // drags and the wheel move its camera
  bool drawn = false;                  // a widget drew it this frame
  bool framed = false;                 // its camera has been framed on what it shows
  double region[4] = {-1.0, -1.0, -1.0, -1.0};
  std::unordered_set<vtkProp *> hidden; // the main view's props this one leaves out
  bool dragging = false, panning = false;
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
// `hide`. A node's tags hold for its subtree, so a node out of the view takes its children along.
bool in_view(const std::vector<std::string> &tags, const std::vector<std::string> &show,
             const std::vector<std::string> &hide) {
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

} // namespace

SceneViews::SceneViews(SceneRenderer &view, ImGuiBackend &backend,
                       std::function<const RealizedScene *()> realized)
    : view_(&view), backend_(&backend), realized_(std::move(realized)) {}

SceneViews::~SceneViews() { clear(); }

void SceneViews::clear() {
  if (view_->isClosed()) { // the viewports went with the view's manager
    views_.clear();
    return;
  }
  ViewportManager &vm = view_->viewportManager();
  for (auto &kv : views_)
    if (vm.hasViewport(kv.first))
      vm.removeViewport(kv.first);
  views_.clear();
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
  vp.setMirrorFilter([raw](vtkProp *p) { return raw->hidden.count(p) == 0; });
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
        v.vp->camera().endPan();
      else
        v.vp->camera().endDrag();
      v.dragging = v.panning = false;
    }
    v.drawn = false;
  }
}

void SceneViews::update(double dt) {
  if (view_->isClosed()) {
    views_.clear();
    return;
  }
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
      for (const RealizedScene::TaggedNode &t : rs->tagged)
        if (!in_view(t.tags, v.show, v.hide))
          collect(t.node.lock(), v.hidden);
    if (!v.chrome) {
      SceneGraph &sg = view_->scene();
      if (std::shared_ptr<GraphicsNode> root = sg.getGraphicsRoot()) {
        collect(root->findChildByName("grid"), v.hidden);
        collect(root->findChildByName("axis"), v.hidden);
        collect(root, v.hidden, false); // the root's own bounding box, not the scene under it
      }
    }
    CameraController &cam = v.vp->camera();
    // Framed once it has drawn something: the bounds of what this view shows.
    if (!v.framed && v.vp->visible()) {
      double b[6];
      vtkRenderer *r = v.vp->renderer();
      if (r && r->GetViewProps()->GetNumberOfItems() > 0) {
        r->ComputeVisiblePropBounds(b);
        if (b[0] <= b[1] && b[2] <= b[3] && b[4] <= b[5]) {
          if (v.camera == "map") {
            const double hw = std::max(0.5 * (b[1] - b[0]), 1e-6);
            const double hh = std::max(0.5 * (b[3] - b[2]), 1e-6);
            cam.frameMap(0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]), hh, hw);
          } else {
            cam.frameBounds(b[0], b[2], b[4], b[1], b[3], b[5]);
            cam.setMode(CameraController::Mode::Orbit); // from a map, say
            if (v.camera == "fly")
              cam.setMode(CameraController::Mode::Fly);
            else if (v.camera == "track" && !v.track.empty()) {
              cam.setScene(&view_->scene());
              cam.setTrackTarget(v.track);
              cam.setMode(CameraController::Mode::Track);
            }
          }
          v.framed = true;
        }
      }
    }
    cam.update(dt);
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
        const std::string camera = w.props.str("camera", "orbit");
        const std::string track = w.props.str("track");
        if (camera != v->camera || track != v->track) {
          v->camera = camera;
          v->track = track;
          v->framed = false;
        }
        if (const ari::Value *bg = w.props.find("clear_color"))
          if (bg->is_seq() && bg->items.size() >= 3)
            v->vp->setBackground(bg->items[0].as_double(), bg->items[1].as_double(),
                                 bg->items[2].as_double(), true);

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

        // Its camera: a drag orbits (the middle or right button pans), the wheel zooms.
        if (v->input) {
          CameraController &cam = v->vp->camera();
          const ImGuiIO &io = ImGui::GetIO();
          if (ImGui::IsItemHovered() && io.MouseWheel != 0.0f)
            cam.mouseWheel(io.MouseWheel);
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
