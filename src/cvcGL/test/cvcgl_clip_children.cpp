/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_clip_children -- GraphicsNode::setClipChildren clips a node's children
// to its own box, on BOTH mapper families (classic, and the low-memory one
// forced), rendered for real.
//
//   planes  the six planes face INTO the box. VTK keeps the side of a clipping
//           plane its normal points into (a point x survives where
//           n . (x - origin) >= 0), so the box centre evaluates to +half-extent
//           against every plane and a point just past a face is negative
//           against that face's plane only -- also with the parent rotated and
//           moved. Every child's mapper holds the planes while clipping is on,
//           and none after it is switched off. No GL needed.
//   render  children fully inside, straddling (an x/y corner; the z faces via a
//           tilted quad) and fully outside (beside the box, and above it) the
//           parent's box: inside drawn, outside clipped, straddling cut at the
//           face, by sample pixel and by lit-pixel area. The same scene is
//           checked unclipped first (all of it drawn, so the checks can fail)
//           and again after setClipChildren(false); then with the parent
//           rotated and moved while clipping is on (the planes follow it).
//
// VTK 9.5's low-memory mapper (the GLES3/WebGL2 default, forced here) has no
// clipping-plane support at all: there only "not wrongly clipped" is checked.
// Skips the render part (rc 0) where nothing rasterises unless
// CVC_REQUIRE_RENDER=1. Synthetic geometry only.

#include "streaming_test_util.h"

#include <cmath>
#include <cvc/gl/LowMemoryPolyDataMapper.h>
#include <cvc/gl/NullGraphicNode.h>
#include <memory>
#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkPlane.h>
#include <vtkPlaneCollection.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

using namespace cvcgl_test;
using cvc::gl::GeometryNode;
using cvc::gl::LowMemoryMapperPolicy;
using cvc::gl::LowMemoryPolyDataMapper;
using cvc::gl::NullGraphicNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

namespace {

constexpr double C = 500.0; // box centre in x and y: well away from the origin
constexpr double HX = 50.0, HZ = 10.0;
const cvc::bounding_box kBox(C - HX, C - HX, -HZ, C + HX, C + HX, HZ);
// A top view of [C - S, C + S]^2 at one pixel per unit; room for the moved box.
constexpr double S = 150.0;
constexpr int W = 300, H = 300;

const char *policyName(LowMemoryMapperPolicy p) {
  return p == LowMemoryMapperPolicy::Force ? "lowmem" : "classic";
}

// A quad over [x0, x1] x [y0, y1] whose z rises linearly from z0 at x0 to z1 at x1.
cvc::geometry quad(cvc::app &app, double x0, double y0, double x1, double y1, double z0,
                   double z1) {
  cvc::geometry g(app);
  g.points().push_back({x0, y0, z0});
  g.points().push_back({x1, y0, z1});
  g.points().push_back({x1, y1, z1});
  g.points().push_back({x0, y1, z0});
  for (int i = 0; i < 4; ++i)
    g.normals().push_back({0.0, 0.0, 1.0});
  g.tris().push_back({0, 1, 2});
  g.tris().push_back({0, 2, 3});
  g.set_geometry_type(cvc::geometry::SURFACE_TRI);
  return g;
}

// The parent (a fixed box, as NullGraphicNode documents for clipping regions)
// and its five children, flat-coloured so each reads back by hue. Top view:
//
//   inside  red    x [C-40, C-20]  y [C+20, C+40]  z 0          fully inside
//   above   green  x [C+20, C+40]  y [C+20, C+40]  z 30         past +z
//   tilted  blue   x [C-40, C+40]  y [C-10, C+10]  z (x - C)/2  cut by +z / -z at x = C +- 20
//   corner  green  x [C+20, C+80]  y [C-80, C-20]  z 0          cut by +x and -y
//   beside  red    x [C-90, C-60]  y [C-90, C-60]  z 0          past -x and -y
struct Scene {
  SceneGraph sg;
  std::shared_ptr<NullGraphicNode> box;
  std::vector<std::shared_ptr<GeometryNode>> children;

  explicit Scene(cvc::app &app, const std::string &name) : sg(app, name) {
    sg.setDiagnosticChromeVisible(false);
    box = sg.getGraphicsRoot()->addGraphicsChild<NullGraphicNode>("box");
    box->setSyncBoundsWithChildren(false);
    box->setBounds(kBox);
    add("inside", quad(app, C - 40, C + 20, C - 20, C + 40, 0, 0), 1, 0.1, 0.1);
    add("above", quad(app, C + 20, C + 20, C + 40, C + 40, 30, 30), 0.1, 1, 0.1);
    add("tilted", quad(app, C - 40, C - 10, C + 40, C + 10, -20, 20), 0.1, 0.1, 1);
    add("corner", quad(app, C + 20, C - 80, C + 80, C - 20, 0, 0), 0.1, 1, 0.1);
    add("beside", quad(app, C - 90, C - 90, C - 60, C - 60, 0, 0), 1, 0.1, 0.1);
  }
  void add(const std::string &name, const cvc::geometry &g, double r, double gr, double b) {
    auto n = box->addGraphicsChild<GeometryNode>(name);
    n->setGeometry(g);
    flat(*n, r, gr, b);
    children.push_back(n);
  }
};

// A point in the box's frame, the hue of the child drawn there, and whether it
// is inside the box (drawn under clipping). Edge samples sit 4 units either
// side of a face.
struct Sample {
  const char *what;
  double x, y, z;
  int ch;
  bool inside;
};
const Sample kSamples[] = {
    {"inside child", C - 30, C + 30, 0, 0, true},
    {"child above the box (+z)", C + 30, C + 30, 30, 1, false},
    {"child beside the box (-x, -y)", C - 75, C - 75, 0, 0, false},
    {"corner child, inside part", C + 35, C - 35, 0, 1, true},
    {"corner child, just inside +x", C + 46, C - 35, 0, 1, true},
    {"corner child, just past +x", C + 54, C - 35, 0, 1, false},
    {"corner child, just inside -y", C + 35, C - 46, 0, 1, true},
    {"corner child, just past -y", C + 35, C - 54, 0, 1, false},
    {"corner child, past +x and -y", C + 65, C - 65, 0, 1, false},
    {"tilted child, centre", C, C, 0, 2, true},
    {"tilted child, just inside +z", C + 16, C, 8, 2, true},
    {"tilted child, just past +z", C + 24, C, 12, 2, false},
    {"tilted child, just inside -z", C - 16, C, -8, 2, true},
    {"tilted child, just past -z", C - 24, C, -12, 2, false},
};

// The child's mapper, through its actor (GeometryNode::mapper() is protected).
vtkMapper *mapperOf(GeometryNode &n) { return vtkActor::SafeDownCast(n.prop())->GetMapper(); }

// Pixels of a clearly dominant channel.
long huePixels(const Frame &f, int ch) {
  long n = 0;
  for (size_t i = 0; i + 2 < f.rgb.size(); i += 3)
    n += isHue(&f.rgb[i], ch) ? 1 : 0;
  return n;
}

// Within 10% of the expected area (edge pixels go either way).
bool areaNear(long got, long want) { return std::labs(got - want) <= want / 10; }

// Check every sample against `expectDrawn(sample)` in frame f.
void checkSamples(SceneRenderer &sr, const Frame &f, cvc::gl::GraphicsNode &box,
                  const std::string &state, bool (*expectDrawn)(const Sample &)) {
  for (const Sample &s : kSamples) {
    const double local[3] = {s.x, s.y, s.z};
    double w[3];
    box.localToWorld(local, w);
    const bool want = expectDrawn(s);
    const bool drawn = hueNear(f, toPx(sr, w[0], w[1], w[2]), 1, s.ch);
    check(drawn == want, state + ": " + s.what + (want ? " is drawn" : " is clipped"));
  }
}

// A rigid move: 30 deg about z then 20 deg about x, about the box centre, then
// a shift.
vtkSmartPointer<vtkTransform> moved() {
  auto xf = vtkSmartPointer<vtkTransform>::New();
  xf->PostMultiply();
  xf->Translate(-C, -C, 0);
  xf->RotateZ(30);
  xf->RotateX(20);
  xf->Translate(C + 10, C - 5, 3);
  return xf;
}

// ── planes (no GL) ──────────────────────────────────────────────────────────
void checkPlanes(cvc::gl::GraphicsNode &box, const std::string &state) {
  vtkPlaneCollection *pc = box.getClipPlanes();
  check(pc && pc->GetNumberOfItems() == 6, state + ": six planes");
  if (!pc || pc->GetNumberOfItems() != 6)
    return;
  // Order: +X, -X, +Y, -Y, +Z, -Z faces; a point 1 unit past each.
  const double half[6] = {HX, HX, HX, HX, HZ, HZ};
  const double past[6][3] = {{C + HX + 1, C, 0}, {C - HX - 1, C, 0}, {C, C + HX + 1, 0},
                             {C, C - HX - 1, 0}, {C, C, HZ + 1},     {C, C, -HZ - 1}};
  const double centreLocal[3] = {C, C, 0};
  double centre[3];
  box.localToWorld(centreLocal, centre);
  bool centreOk = true, pastOk = true;
  std::string detail;
  for (int i = 0; i < 6; ++i) {
    const double v = pc->GetItem(i)->EvaluateFunction(centre);
    centreOk = centreOk && std::fabs(v - half[i]) < 1e-6;
    detail += std::to_string(v) + " ";
    double p[3];
    box.localToWorld(past[i], p);
    for (int j = 0; j < 6; ++j) {
      const double e = pc->GetItem(j)->EvaluateFunction(p);
      pastOk = pastOk && (j == i ? e < 0.0 : e > 0.0);
    }
  }
  check(centreOk, state + ": the box centre is +half-extent inside every plane", detail);
  check(pastOk, state + ": a point past a face is outside that face's plane only");
}

void testPlanes(cvc::app &app) {
  std::printf("planes: orientation and hand-off (no GL)\n");
  Scene s(app, "clip_planes");
  check(!s.box->getClipChildren(), "clipping is off by default");
  for (auto &c : s.children)
    check(mapperOf(*c)->GetNumberOfClippingPlanes() == 0, c->getName() + ": no planes yet");
  s.box->setClipChildren(true);
  check(s.box->getClipChildren() && s.box->getState("clip_children").value<int>() == 1,
        "setClipChildren(true) is reflected in the getter and the state tree");
  checkPlanes(*s.box, "axis-aligned");
  for (auto &c : s.children)
    check(mapperOf(*c)->GetClippingPlanes() == s.box->getClipPlanes(),
          c->getName() + ": the child's mapper holds the parent's planes");
  s.box->setTransform(moved()->GetMatrix());
  checkPlanes(*s.box, "rotated + moved");
  s.box->setClipChildren(false);
  check(!s.box->getClipChildren() && s.box->getState("clip_children").value<int>() == 0,
        "setClipChildren(false) is reflected in the getter and the state tree");
  for (auto &c : s.children)
    check(mapperOf(*c)->GetNumberOfClippingPlanes() == 0,
          c->getName() + ": planes removed from the child's mapper");
  // Removing them must not empty the parent's own collection (every child's
  // mapper holds that same collection), or clipping can never come back.
  check(s.box->getClipPlanes()->GetNumberOfItems() == 6, "... and the parent keeps its six planes",
        std::to_string(s.box->getClipPlanes()->GetNumberOfItems()));
  s.box->setClipChildren(true);
  bool again = true;
  for (auto &c : s.children)
    again = again && mapperOf(*c)->GetClippingPlanes() == s.box->getClipPlanes() &&
            mapperOf(*c)->GetNumberOfClippingPlanes() == 6;
  check(again, "on again: every child's mapper holds the six planes again");
  checkPlanes(*s.box, "on again");
}

// ── render ──────────────────────────────────────────────────────────────────
bool drawnAlways(const Sample &) { return true; }
bool drawnIfInside(const Sample &s) { return s.inside; }

std::string areas(const Frame &f) {
  return "r " + std::to_string(huePixels(f, 0)) + " g " + std::to_string(huePixels(f, 1)) + " b " +
         std::to_string(huePixels(f, 2));
}

// The axis-aligned box, clipping on: only the parts inside drawn.
void checkClipped(SceneRenderer &sr, const Frame &f, cvc::gl::GraphicsNode &box,
                  const std::string &state) {
  checkSamples(sr, f, box, state, drawnIfInside);
  // inside 20 x 20; above + beside 0; corner 30 x 30 of 60 x 60; tilted 40 x 20 of 80 x 20.
  check(areaNear(huePixels(f, 0), 400) && areaNear(huePixels(f, 1), 900) &&
            areaNear(huePixels(f, 2), 800),
        state + ": only the parts inside the box are lit", areas(f));
}

void testRender(cvc::app &app, LowMemoryMapperPolicy policy) {
  const std::string kind = policyName(policy);
  std::printf("render: children inside / straddling / outside the box [%s]\n", kind.c_str());
  cvc::gl::setLowMemoryMapperPolicy(policy);
  Scene s(app, "clip_" + kind);
  const bool lowmem = policy == LowMemoryMapperPolicy::Force;
  bool mapperOk = true;
  for (auto &c : s.children)
    mapperOk = mapperOk &&
               (LowMemoryPolyDataMapper::SafeDownCast(mapperOf(*c)) != nullptr) == lowmem &&
               (lowmem || mapperOf(*c)->IsA("vtkOpenGLPolyDataMapper"));
  check(mapperOk, std::string("the children draw with the ") +
                      (lowmem ? "low-memory mapper" : "classic vtkOpenGLPolyDataMapper"));
  SceneRenderer sr(s.sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, C, C, S);

  // Unclipped: every sample drawn, the full areas lit.
  Frame f = grab(sr);
  checkSamples(sr, f, *s.box, "unclipped", drawnAlways);
  // inside 20 x 20 + beside 30 x 30; above 20 x 20 + corner 60 x 60; tilted 80 x 20.
  check(areaNear(huePixels(f, 0), 400 + 900) && areaNear(huePixels(f, 1), 400 + 3600) &&
            areaNear(huePixels(f, 2), 1600),
        "unclipped: the full areas are lit", areas(f));

  s.box->setClipChildren(true);
  f = grab(sr);
  if (lowmem) {
    // No clipping planes in this mapper: everything inside must still be drawn.
    for (const Sample &smp : kSamples) {
      if (!smp.inside)
        continue;
      const double local[3] = {smp.x, smp.y, smp.z};
      double w[3];
      s.box->localToWorld(local, w);
      check(hueNear(f, toPx(sr, w[0], w[1], w[2]), 1, smp.ch),
            std::string("clipped: ") + smp.what + " is drawn (not wrongly clipped)");
    }
    std::printf("  note: VTK's low-memory mapper ignores clipping planes (%s lit)\n",
                areas(f).c_str());
  } else {
    checkClipped(sr, f, *s.box, "clipped");
  }

  s.box->setClipChildren(false);
  f = grab(sr);
  checkSamples(sr, f, *s.box, "clipping off again", drawnAlways);

  if (!lowmem) {
    s.box->setClipChildren(true);
    checkClipped(sr, grab(sr), *s.box, "clipping on again");
    // The planes follow the parent while clipping is on.
    s.box->setTransform(moved()->GetMatrix());
    checkSamples(sr, grab(sr), *s.box, "rotated + moved, clipped", drawnIfInside);
  }
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Auto);
}

} // namespace

int main() {
  disableSwapThrottle();
  installErrorCounter();
  cvc::app app;
  testPlanes(app);
  if (renderAvailable(app)) {
    {
      SceneGraph sg(app, "gl_probe");
      SceneRenderer sr(sg, 16, 16, /*offscreen=*/true);
      sr.render();
      printRenderer(sr);
    }
    for (LowMemoryMapperPolicy p : {LowMemoryMapperPolicy::Off, LowMemoryMapperPolicy::Force})
      testRender(app, p);
  }
  check(ErrorCounter::errors() == 0, "no VTK errors overall",
        std::to_string(ErrorCounter::errors()));
  return finish("cvcgl_clip_children");
}
