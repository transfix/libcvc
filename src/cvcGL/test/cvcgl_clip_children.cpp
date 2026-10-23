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
//   follow  the planes follow the children and the box: a child added after
//           clipping is on is clipped, a removed one is detached, and the
//           planes track every way a parent's box changes -- NullGraphicNode
//           setBounds / its "bounds" state key / a group syncing to its
//           children, GeometryNode::setGeometry, VolumeNode / VolSliceNode
//           setVolume, VolRenNode add / move (setVolumeConfig and its state
//           tree) / clear, LodGraphicsNode::setBase. A LodGraphicsNode child
//           hands the planes to its rungs, including rungs added later. No GL
//           needed; then rendered on the classic mapper (a late child and an
//           LOD child cut at the face, a reparented child whole, a grown box).
//
// VTK 9.5's low-memory mapper (the GLES3/WebGL2 default, forced here) has no
// clipping-plane support at all: there only "not wrongly clipped" is checked.
// Skips the render part (rc 0) where nothing rasterises unless
// CVC_REQUIRE_RENDER=1. Synthetic geometry only.

#include "streaming_test_util.h"

#include <cmath>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/LowMemoryPolyDataMapper.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volren/settings.h>
#include <initializer_list>
#include <memory>
#include <sstream>
#include <vtkActor.h>
#include <vtkMatrix4x4.h>
#include <vtkPlane.h>
#include <vtkPlaneCollection.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

using namespace cvcgl_test;
using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::LodGraphicsNode;
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

// A quad whose extent is exactly the box b (it rises from minz to maxz in x).
cvc::geometry quadOver(cvc::app &app, const cvc::bounding_box &b) {
  return quad(app, b.minx, b.miny, b.maxx, b.maxy, b.minz, b.maxz);
}

// A volume spanning b (its voxels do not matter here).
cvc::volume boxVolume(cvc::app &app, const cvc::bounding_box &b) {
  cvc::volume v(app);
  v.boundingBox(b);
  return v;
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

// The child's mapper holds exactly the parent's six box planes -- the SAME plane
// objects (a child gets its own collection of them, so moving the parent's
// planes in place reaches it).
bool holdsBoxPlanes(GeometryNode &c, cvc::gl::GraphicsNode &parent) {
  vtkPlaneCollection *mine = vtkActor::SafeDownCast(c.prop())->GetMapper()->GetClippingPlanes();
  vtkPlaneCollection *box = parent.getClipBoxPlanes();
  if (!mine || mine->GetNumberOfItems() != 6 || box->GetNumberOfItems() != 6)
    return false;
  for (int i = 0; i < 6; ++i)
    if (!mine->IsItemPresent(box->GetItem(i)))
      return false;
  return true;
}

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
// The node's six planes against its expected local box b.
void checkPlanes(GraphicsNode &box, const cvc::bounding_box &b, const std::string &state) {
  vtkPlaneCollection *pc = box.getClipBoxPlanes();
  check(pc && pc->GetNumberOfItems() == 6, state + ": six planes");
  if (!pc || pc->GetNumberOfItems() != 6)
    return;
  const double cx = (b.minx + b.maxx) / 2, cy = (b.miny + b.maxy) / 2, cz = (b.minz + b.maxz) / 2;
  // Order: +X, -X, +Y, -Y, +Z, -Z faces; a point 1 unit past each.
  const double half[6] = {(b.maxx - b.minx) / 2, (b.maxx - b.minx) / 2, (b.maxy - b.miny) / 2,
                          (b.maxy - b.miny) / 2, (b.maxz - b.minz) / 2, (b.maxz - b.minz) / 2};
  const double past[6][3] = {{b.maxx + 1, cy, cz}, {b.minx - 1, cy, cz}, {cx, b.maxy + 1, cz},
                             {cx, b.miny - 1, cz}, {cx, cy, b.maxz + 1}, {cx, cy, b.minz - 1}};
  const double centreLocal[3] = {cx, cy, cz};
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
  checkPlanes(*s.box, kBox, "axis-aligned");
  for (auto &c : s.children)
    check(holdsBoxPlanes(*c, *s.box),
          c->getName() + ": the child's mapper holds the parent's planes");
  s.box->setTransform(moved()->GetMatrix());
  checkPlanes(*s.box, kBox, "rotated + moved");
  s.box->setClipChildren(false);
  check(!s.box->getClipChildren() && s.box->getState("clip_children").value<int>() == 0,
        "setClipChildren(false) is reflected in the getter and the state tree");
  for (auto &c : s.children)
    check(mapperOf(*c)->GetNumberOfClippingPlanes() == 0,
          c->getName() + ": planes removed from the child's mapper");
  // Removing them must not empty the parent's own collection (every child's
  // mapper holds that same collection), or clipping can never come back.
  check(s.box->getClipBoxPlanes()->GetNumberOfItems() == 6,
        "... and the parent keeps its six planes",
        std::to_string(s.box->getClipBoxPlanes()->GetNumberOfItems()));
  s.box->setClipChildren(true);
  bool again = true;
  for (auto &c : s.children)
    again = again && holdsBoxPlanes(*c, *s.box);
  check(again, "on again: every child's mapper holds the six planes again");
  checkPlanes(*s.box, kBox, "on again");
}

// The parent's planes in the child's mapper (the one collection, all six).
bool holdsPlanes(GeometryNode &c, GraphicsNode &parent) { return holdsBoxPlanes(c, parent); }
bool noPlanes(GeometryNode &c) { return mapperOf(c)->GetNumberOfClippingPlanes() == 0; }

std::string csv(const cvc::bounding_box &b) {
  std::ostringstream o;
  o << b.minx << "," << b.miny << "," << b.minz << "," << b.maxx << "," << b.maxy << "," << b.maxz;
  return o.str();
}

// ── follow: children come and go, the box changes (no GL) ────────────────────
void testFollow(cvc::app &app) {
  std::printf("follow: children added / removed, and every way a parent's box changes (no GL)\n");
  Scene s(app, "clip_follow");
  GraphicsNode &root = *s.sg.getGraphicsRoot();
  s.box->setClipChildren(true);

  auto late = s.box->addGraphicsChild<GeometryNode>("late");
  late->setGeometry(quad(app, C - 80, C - 45, C - 20, C - 25, 0, 0));
  check(holdsPlanes(*late, *s.box), "a child added after setClipChildren(true) is clipped");
  // One clipped from the start: removed, it is detached; added back, clipped again.
  std::shared_ptr<GeometryNode> beside = s.children[4];
  s.box->removeGraphicsChild(beside);
  check(noPlanes(*beside) && s.box->getClipBoxPlanes()->GetNumberOfItems() == 6,
        "a child removed is detached (and the parent keeps its planes)");
  s.box->addGraphicsChild(std::static_pointer_cast<GraphicsNode>(beside));
  check(holdsPlanes(*beside, *s.box), "... and added back, clipped again");

  const cvc::bounding_box big(C - 70, C - 70, -15, C + 70, C + 70, 15);
  s.box->setBounds(big);
  checkPlanes(*s.box, big, "NullGraphicNode::setBounds while clipping");
  check(holdsPlanes(*s.children[0], *s.box), "... the children still hold the parent's planes");
  const cvc::bounding_box mid(C - 60, C - 60, -12, C + 60, C + 60, 12);
  s.box->getState("bounds").value(csv(mid));
  s.sg.processEvents();
  checkPlanes(*s.box, mid, "NullGraphicNode \"bounds\" state key");

  // A group that sizes itself to its children.
  auto group = root.addGraphicsChild<NullGraphicNode>("group");
  group->setClipChildren(true);
  const cvc::bounding_box memberBox(C - 40, C - 10, -20, C + 40, C + 10, 20);
  auto member = group->addGraphicsChild<GeometryNode>("member");
  member->setGeometry(quadOver(app, memberBox));
  checkPlanes(*group, memberBox, "a NullGraphicNode group synced to its children");
  check(holdsPlanes(*member, *group), "... and its child holds the planes");

  // Every data-driven parent: clip, change the box, the planes follow.
  const cvc::bounding_box b0(C - 30, C - 20, -5, C + 30, C + 20, 5);
  const cvc::bounding_box b1(C - 60, C - 40, -8, C + 60, C + 40, 8);

  auto geo = root.addGraphicsChild<GeometryNode>("geo_parent");
  geo->setGeometry(quadOver(app, b0));
  geo->setClipChildren(true);
  checkPlanes(*geo, b0, "GeometryNode parent");
  geo->setGeometry(quadOver(app, b1));
  checkPlanes(*geo, b1, "GeometryNode parent, after setGeometry");

  auto vol = root.addGraphicsChild<cvc::gl::VolumeNode>("vol_parent");
  vol->setVolume(boxVolume(app, b0));
  vol->setClipChildren(true);
  checkPlanes(*vol, b0, "VolumeNode parent");
  vol->setVolume(boxVolume(app, b1));
  checkPlanes(*vol, b1, "VolumeNode parent, after setVolume");

  auto slice = root.addGraphicsChild<cvc::gl::VolSliceNode>("slice_parent");
  slice->setVolume(boxVolume(app, b0));
  slice->setClipChildren(true);
  checkPlanes(*slice, b0, "VolSliceNode parent");
  slice->setVolume(boxVolume(app, b1));
  checkPlanes(*slice, b1, "VolSliceNode parent, after setVolume");

  auto ren = root.addGraphicsChild<cvc::gl::VolRenNode>("volren_parent");
  ren->addVolume(boxVolume(app, b0), cvc::volren::volume_settings());
  ren->setClipChildren(true);
  checkPlanes(*ren, b0, "VolRenNode parent");
  cvc::volren::volume_settings shifted = ren->volumeConfig(0);
  const double t[16] = {1, 0, 0, 10, 0, 1, 0, -20, 0, 0, 1, 4, 0, 0, 0, 1};
  shifted.model_transform = cvc::volren::mat4::from_row_major(t);
  ren->setVolumeConfig(0, shifted);
  checkPlanes(*ren,
              cvc::bounding_box(b0.minx + 10, b0.miny - 20, b0.minz + 4, b0.maxx + 10, b0.maxy - 20,
                                b0.maxz + 4),
              "VolRenNode parent, volume moved by setVolumeConfig");
  ren->getState("volren.volumes.0.matrix").value(std::string("1,0,0,-5,0,1,0,15,0,0,1,-3,0,0,0,1"));
  s.sg.processEvents();
  checkPlanes(*ren,
              cvc::bounding_box(b0.minx - 5, b0.miny + 15, b0.minz - 3, b0.maxx - 5, b0.maxy + 15,
                                b0.maxz - 3),
              "VolRenNode parent, volume moved through its state tree");
  ren->clearVolumes();
  checkPlanes(*ren, cvc::bounding_box(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5),
              "VolRenNode parent, after clearVolumes (its default box)");

  auto lodParent = root.addGraphicsChild<LodGraphicsNode>("lod_parent");
  lodParent->setBase(quadOver(app, b0));
  lodParent->setClipChildren(true);
  checkPlanes(*lodParent, b0, "LodGraphicsNode parent");
  lodParent->setBase(quadOver(app, b1));
  checkPlanes(*lodParent, b1, "LodGraphicsNode parent, after setBase");

  // A LodGraphicsNode CHILD draws nothing itself: its rungs take the planes.
  auto lod = s.box->addGraphicsChild<LodGraphicsNode>("lod");
  const cvc::geometry finest = quad(app, C - 10, C + 30, C + 10, C + 80, 0, 0);
  lod->setBase(finest);
  check(lod->rungCount() == 1 && holdsPlanes(*lod->rung(0), *s.box),
        "a LodGraphicsNode child hands the planes to its rung");
  cvc::lod::mesh_pyramid pyr;
  pyr.rungs = {finest, quad(app, C - 10, C + 30, C + 10, C + 80, 0, 0)};
  pyr.world_error_m = {0.0, 1.0};
  check(lod->appendRungs(pyr) && lod->rungCount() == 2 && holdsPlanes(*lod->rung(0), *s.box) &&
            holdsPlanes(*lod->rung(1), *s.box),
        "... and to a rung added later");
  s.box->setClipChildren(false);
  check(noPlanes(*lod->rung(0)) && noPlanes(*lod->rung(1)), "... and takes them off again");
  s.box->setClipChildren(true);
  check(holdsPlanes(*lod->rung(0), *s.box) && holdsPlanes(*lod->rung(1), *s.box),
        "... and back on");
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

// A world point, the hue drawn there, whether it should be.
struct Pt {
  const char *what;
  double x, y, z;
  int ch;
  bool drawn;
};
void expectAt(SceneRenderer &sr, const Frame &f, const std::string &state,
              std::initializer_list<Pt> pts) {
  for (const Pt &p : pts) {
    const bool drawn = hueNear(f, toPx(sr, p.x, p.y, p.z), 1, p.ch);
    check(drawn == p.drawn, state + ": " + p.what + (p.drawn ? " is drawn" : " is clipped"));
  }
}

void testRenderFollow(cvc::app &app) {
  std::printf("render: the planes follow children and the box [classic]\n");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Off);
  Scene s(app, "clip_follow_render");
  SceneRenderer sr(s.sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, C, C, S);
  s.box->setClipChildren(true);
  sr.render();

  // Added while clipping is on: a red quad across the -x face, and an LOD whose
  // (green) rung crosses the +y face.
  auto late = s.box->addGraphicsChild<GeometryNode>("late");
  late->setGeometry(quad(app, C - 80, C - 45, C - 20, C - 25, 0, 0));
  flat(*late, 1, 0.1, 0.1);
  auto lod = s.box->addGraphicsChild<LodGraphicsNode>("lod");
  lod->setRungStyle([](GeometryNode &n) { flat(n, 0.1, 1, 0.1); });
  lod->setBase(quad(app, C - 10, C + 30, C + 10, C + 80, 0, 0));
  expectAt(sr, grab(sr), "added after clipping on",
           {{"late child, inside", C - 35, C - 35, 0, 0, true},
            {"late child, just inside -x", C - 46, C - 35, 0, 0, true},
            {"late child, just past -x", C - 54, C - 35, 0, 0, false},
            {"LOD child's rung, inside", C, C + 40, 0, 1, true},
            {"LOD child's rung, just inside +y", C, C + 46, 0, 1, true},
            {"LOD child's rung, just past +y", C, C + 54, 0, 1, false}});

  // Moved out from under the box: drawn whole.
  std::shared_ptr<GeometryNode> corner = s.children[3];
  s.box->removeGraphicsChild(corner);
  s.sg.getGraphicsRoot()->addGraphicsChild(std::static_pointer_cast<GraphicsNode>(corner));
  expectAt(sr, grab(sr), "corner child moved to the root",
           {{"corner child, inside the box", C + 35, C - 35, 0, 1, true},
            {"corner child, past +x", C + 65, C - 35, 0, 1, true},
            {"corner child, past -y", C + 35, C - 65, 0, 1, true}});

  // The box grows: every cut moves out with it.
  s.box->setBounds(cvc::bounding_box(C - 70, C - 70, -15, C + 70, C + 70, 15));
  expectAt(sr, grab(sr), "box grown with setBounds",
           {{"late child, past the old -x face", C - 54, C - 35, 0, 0, true},
            {"late child, past the new -x face", C - 74, C - 35, 0, 0, false},
            {"LOD child's rung, past the old +y face", C, C + 54, 0, 1, true},
            {"LOD child's rung, past the new +y face", C, C + 74, 0, 1, false},
            {"tilted child, past the old +z face", C + 26, C, 13, 2, true},
            {"tilted child, past the new +z face", C + 34, C, 17, 2, false},
            {"beside child, now partly inside", C - 65, C - 65, 0, 0, true},
            {"beside child, still outside", C - 80, C - 80, 0, 0, false}});
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Auto);
}

} // namespace

int main() {
  installErrorCounter();
  cvc::app app;
  testPlanes(app);
  testFollow(app);
  if (renderAvailable(app)) {
    {
      SceneGraph sg(app, "gl_probe");
      SceneRenderer sr(sg, 16, 16, /*offscreen=*/true);
      sr.render();
      printRenderer(sr);
    }
    for (LowMemoryMapperPolicy p : {LowMemoryMapperPolicy::Off, LowMemoryMapperPolicy::Force})
      testRender(app, p);
    testRenderFollow(app);
  }
  check(ErrorCounter::errors() == 0, "no VTK errors overall",
        std::to_string(ErrorCounter::errors()));
  return finish("cvcgl_clip_children");
}
