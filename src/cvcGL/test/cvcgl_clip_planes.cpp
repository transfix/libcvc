/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_clip_planes -- GraphicsNode::setClipPlanes: a node and everything below
// it clipped by half-spaces in its local frame (keep the side each normal points
// into), accumulated down the tree and capped per renderer.
//
//   wiring  (no GL) what each renderer is handed: its own planes first, then its
//           parent's, and so on up (setClipPlanes, and the box of a
//           setClipChildren ancestor, both reach every level below); the cap (6
//           poly-data, 8 volume / volren, 0 low-memory mapper or a node that
//           draws nothing) keeps the nearest; the planes are world space and
//           follow their owner's transform IN PLACE -- a moved plane is the same
//           object, nothing is re-handed -- while changing their number re-hands
//           everything below; a child added later / removed gains / loses them;
//           the "clip_planes" state key works both ways; a VolRenNode's
//           billboard quad is never clipped, a VolumeNode's mapper is.
//   local   a plane is in the coordinates of the node that declares it (where
//           its geometry / volume is defined, before its transform): for points
//           in that frame, the local side and the world-space plane's side agree
//           under rotation, non-uniform scale and translation, nested, after
//           setPoseMatrix and after an ancestor re-scales; an inherited plane
//           stays in its owner's frame.
//   render  one half-space x >= 0 on a geometry quad (classic mapper), a
//           VolumeNode (VTK's GPU raycast), a VolSliceNode and a VolRenNode (the
//           software raycaster, through its cut planes): drawn right of the cut,
//           nothing left of it, and all of it unclipped first (so the checks can
//           fail). On the quad, the plane moved in place and then the node
//           moved: the cut follows both.
//
// Skips the render part (rc 0) where nothing rasterises unless
// CVC_REQUIRE_RENDER=1. Synthetic data only.

#include "streaming_test_util.h"

#include <chrono>
#include <cmath>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/LowMemoryPolyDataMapper.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/volren/settings.h>
#include <cvc/volslice/settings.h>
#include <memory>
#include <thread>
#include <vtkAbstractVolumeMapper.h>
#include <vtkActor.h>
#include <vtkPlane.h>
#include <vtkPlaneCollection.h>
#include <vtkPolyDataMapper.h>
#include <vtkVolume.h>

using namespace cvcgl_test;
using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::LowMemoryMapperPolicy;
using cvc::gl::NullGraphicNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using Plane = cvc::gl::GraphicsNode::ClipPlane;

namespace {

constexpr int W = 160, H = 160;

Plane plane(double ox, double oy, double oz, double nx, double ny, double nz) {
  Plane p;
  p.origin = {ox, oy, oz};
  p.normal = {nx, ny, nz};
  return p;
}
const Plane kKeepPlusX = plane(0, 0, 0, 1, 0, 0);

// A quad over [x0, x1] x [y0, y1] at z.
cvc::geometry quad(cvc::app &app, double x0, double y0, double x1, double y1, double z = 0) {
  cvc::geometry g(app);
  g.points().push_back({x0, y0, z});
  g.points().push_back({x1, y0, z});
  g.points().push_back({x1, y1, z});
  g.points().push_back({x0, y1, z});
  for (int i = 0; i < 4; ++i)
    g.normals().push_back({0.0, 0.0, 1.0});
  g.tris().push_back({0, 1, 2});
  g.tris().push_back({0, 2, 3});
  g.set_geometry_type(cvc::geometry::SURFACE_TRI);
  return g;
}

// A radial density ball over [-1,1]^3: 1 at the centre, 0 from r = 0.8.
cvc::volume ball(cvc::app &app, unsigned n = 32) {
  cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float, cvc::bounding_box(-1, -1, -1, 1, 1, 1));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = -1.0 + 2.0 * i / (n - 1), y = -1.0 + 2.0 * j / (n - 1),
                     z = -1.0 + 2.0 * k / (n - 1);
        const double r = std::sqrt(x * x + y * y + z * z);
        vol(i, j, k, r >= 0.8 ? 0.0 : 1.0 - r / 0.8);
      }
  vol.min(0.0);
  vol.max(1.0);
  return vol;
}

// The signed distance to a sphere of radius r (inside negative), over [-1,1]^3.
cvc::volume sphereSdf(cvc::app &app, double r, unsigned n = 32) {
  cvc::volume vol(app, cvc::dimension(n, n, n), cvc::Float, cvc::bounding_box(-1, -1, -1, 1, 1, 1));
  for (unsigned k = 0; k < n; ++k)
    for (unsigned j = 0; j < n; ++j)
      for (unsigned i = 0; i < n; ++i) {
        const double x = -1 + i * vol.XSpan(), y = -1 + j * vol.YSpan(), z = -1 + k * vol.ZSpan();
        vol(i, j, k, std::sqrt(x * x + y * y + z * z) - r);
      }
  return vol;
}

vtkMapper *mapperOf(GraphicsNode &n) {
  if (auto *a = vtkActor::SafeDownCast(n.prop()))
    return a->GetMapper();
  return nullptr;
}

int applied(GraphicsNode &n) {
  vtkPlaneCollection *pc = n.getAppliedClipPlanes();
  return pc ? pc->GetNumberOfItems() : 0;
}

vtkPlane *appliedPlane(GraphicsNode &n, int i) { return n.getAppliedClipPlanes()->GetItem(i); }

bool near3(const double *a, double x, double y, double z) {
  return std::fabs(a[0] - x) < 1e-9 && std::fabs(a[1] - y) < 1e-9 && std::fabs(a[2] - z) < 1e-9;
}
bool normalIs(vtkPlane *p, double x, double y, double z) { return near3(p->GetNormal(), x, y, z); }
bool originIs(vtkPlane *p, double x, double y, double z) { return near3(p->GetOrigin(), x, y, z); }

// The planes live in the LOCAL frame of the node that declares them: for points
// given in that frame, the side n . (x - o) says, and the side the world-space
// plane handed to the renderer says (at the point's world position), agree --
// under any rotation, non-uniform scale and translation, nested.
bool sidesAgree(GraphicsNode &owner, const Plane &local, vtkPlane *world, std::string &detail) {
  int n = 0;
  for (double x : {-1.7, -0.4, 0.9, 2.2})
    for (double y : {-1.3, 0.2, 1.6})
      for (double z : {-0.8, 0.6, 1.9}) {
        const double p[3] = {x, y, z};
        const double s = local.normal[0] * (x - local.origin[0]) +
                         local.normal[1] * (y - local.origin[1]) +
                         local.normal[2] * (z - local.origin[2]);
        if (std::fabs(s) < 1e-3)
          continue; // on the plane: no side to compare
        double w[3];
        owner.localToWorld(p, w);
        const double e = world->EvaluateFunction(w);
        ++n;
        if ((s > 0) != (e > 0)) {
          detail = "local (" + std::to_string(x) + "," + std::to_string(y) + "," +
                   std::to_string(z) + "): " + std::to_string(s) + " vs world " + std::to_string(e);
          return false;
        }
      }
  detail = std::to_string(n) + " points";
  return n > 20;
}

void testLocalFrame(cvc::app &app) {
  std::printf("local frame: a plane is in the declaring node's own coordinates (no GL)\n");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Off);
  SceneGraph sg(app, "clip_local");
  auto parent = sg.getGraphicsRoot()->addGraphicsChild<NullGraphicNode>("parent");
  parent->setPosition(5, -3, 2);
  parent->setRotation(0, 0, 30);
  parent->setScale(1.5, 1.5, 1.5);
  auto geo = parent->addGraphicsChild<GeometryNode>("geo");
  geo->setGeometry(quad(app, -1, -1, 1, 1));
  geo->setPosition(1, 2, 3);
  geo->setRotation(10, 20, 30);
  geo->setScale(2, 0.5, 3); // non-uniform: normals need the inverse transpose
  const Plane own = plane(0.3, -0.2, 0.5, 1, 2, -1);      // oblique, in geo's frame
  const Plane fromParent = plane(-0.5, 0.25, 0, 0, 1, 1); // in the parent's frame
  geo->setClipPlanes({own});
  parent->setClipPlanes({fromParent});
  std::string d;
  check(applied(*geo) == 2 && sidesAgree(*geo, own, appliedPlane(*geo, 0), d),
        "a node's own plane: the same side in its local frame and in world space", d);
  check(sidesAgree(*parent, fromParent, appliedPlane(*geo, 1), d),
        "an inherited plane stays in its owner's (the parent's) frame", d);
  // The per-frame pose path moves the planes too.
  const double pose[16] = {0, 0, 1, -4, 1, 0, 0, 7, 0, 1, 0, 0.5, 0, 0, 0, 1};
  geo->setPoseMatrix(pose);
  sg.processEvents();
  check(sidesAgree(*geo, own, appliedPlane(*geo, 0), d),
        "after setPoseMatrix: still the node's own frame", d);
  parent->setScale(0.5, 2.0, 1.0);
  check(sidesAgree(*geo, own, appliedPlane(*geo, 0), d) &&
            sidesAgree(*parent, fromParent, appliedPlane(*geo, 1), d),
        "after the parent is re-scaled non-uniformly: both still in their own frames", d);
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Auto);
}

// ── wiring (no GL) ──────────────────────────────────────────────────────────
void testWiring(cvc::app &app) {
  std::printf("wiring: accumulation, cap, in-place moves, add/remove, state (no GL)\n");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Off);
  SceneGraph sg(app, "clipw");
  sg.setDiagnosticChromeVisible(false);
  GraphicsNode &root = *sg.getGraphicsRoot();

  // group (keep +x) > geo (keep +y) > leaf.
  auto group = root.addGraphicsChild<NullGraphicNode>("group");
  group->setClipPlanes({kKeepPlusX});
  auto geo = group->addGraphicsChild<GeometryNode>("geo");
  geo->setGeometry(quad(app, -1, -1, 1, 1));
  geo->setClipPlanes({plane(0, 0, 0, 0, 1, 0)});
  auto leaf = geo->addGraphicsChild<GeometryNode>("leaf");
  leaf->setGeometry(quad(app, -1, -1, 1, 1));

  check(group->maxClipPlanes() == 0 && applied(*group) == 0 && group->clipPlaneCount() == 1,
        "a group draws nothing: clipped by 1 plane, hands its renderer none");
  check(geo->maxClipPlanes() == 6 && applied(*geo) == 2 &&
            normalIs(appliedPlane(*geo, 0), 0, 1, 0) && normalIs(appliedPlane(*geo, 1), 1, 0, 0),
        "a geometry node: its own plane first, then its parent's");
  check(mapperOf(*geo)->GetClippingPlanes() == geo->getAppliedClipPlanes(),
        "... handed to its mapper");
  check(applied(*leaf) == 2 && appliedPlane(*leaf, 0) == appliedPlane(*geo, 0) &&
            appliedPlane(*leaf, 1) == appliedPlane(*geo, 1),
        "a grandchild: the same plane objects, nearest first");

  // The owner moves: its planes move with it, in place.
  vtkPlaneCollection *before = geo->getAppliedClipPlanes();
  group->setPosition(10, 0, 0);
  group->setRotation(0, 0, 90);
  check(geo->getAppliedClipPlanes() == before && originIs(appliedPlane(*geo, 1), 10, 0, 0) &&
            normalIs(appliedPlane(*geo, 1), 0, 1, 0),
        "the parent rotated + moved: its plane follows in place (+x -> +y, through (10,0,0))");
  group->setClipPlanes({plane(2, 0, 0, 1, 0, 0)});
  check(geo->getAppliedClipPlanes() == before && originIs(appliedPlane(*geo, 1), 10, 2, 0),
        "the parent's plane moved by setClipPlanes (same count): updated in place, not re-handed");
  group->setClipPlanes({kKeepPlusX, plane(0, 0, 0, 0, 0, 1)});
  check(geo->getAppliedClipPlanes() != before && applied(*geo) == 3 && applied(*leaf) == 3,
        "a plane added to the parent: everything below re-handed (3 planes)");
  group->setRotation(0, 0, 0);
  group->setPosition(0, 0, 0);

  // The cap keeps the nearest.
  std::vector<Plane> six;
  for (int i = 0; i < 6; ++i)
    six.push_back(plane(0, 0, -i, 0, 0, 1));
  geo->setClipPlanes(six);
  check(geo->clipPlaneCount() == 8 && applied(*geo) == 6 &&
            originIs(appliedPlane(*geo, 5), 0, 0, -5),
        "8 planes on a poly-data mapper: its own 6 apply, the parent's 2 are dropped");
  check(leaf->clipPlaneCount() == 8 && applied(*leaf) == 6, "... and the same below it");
  auto vol = group->addGraphicsChild<cvc::gl::VolumeNode>("vol");
  vol->setVolume(ball(app, 8));
  std::vector<Plane> seven(7, kKeepPlusX);
  vol->setClipPlanes(seven);
  check(vol->maxClipPlanes() == 8 && vol->clipPlaneCount() == 9 && applied(*vol) == 8,
        "9 planes on a volume: 8 apply");
  auto *volMapper = vtkVolume::SafeDownCast(vol->prop())->GetMapper();
  check(volMapper->GetClippingPlanes() == vol->getAppliedClipPlanes(),
        "... handed to the volume mapper");
  geo->setClipPlanes({plane(0, 0, 0, 0, 1, 0)});

  // Children come and go.
  group->removeGraphicsChild(geo);
  check(applied(*geo) == 1 && applied(*leaf) == 1,
        "removed from the group: only its own plane, and the same below");
  group->addGraphicsChild(std::static_pointer_cast<GraphicsNode>(geo));
  check(applied(*geo) == 3 && applied(*leaf) == 3, "added back: the group's planes again");
  auto lod = group->addGraphicsChild<cvc::gl::LodGraphicsNode>("lod");
  lod->setBase(quad(app, -1, -1, 1, 1));
  check(lod->maxClipPlanes() == 0 && applied(*lod->rung(0)) == 2,
        "a LodGraphicsNode child: its rung is clipped by the group's planes");

  // setClipChildren reaches every level below too.
  auto host = root.addGraphicsChild<GeometryNode>("host");
  host->setGeometry(quad(app, -1, -1, 1, 1, 0));
  auto kid = host->addGraphicsChild<GeometryNode>("kid");
  auto grandkid = kid->addGraphicsChild<GeometryNode>("grandkid");
  host->setClipChildren(true);
  check(applied(*host) == 0 && applied(*kid) == 6 && applied(*grandkid) == 6,
        "setClipChildren: the box planes clip a grandchild too, not the node itself");
  host->setClipChildren(false);
  check(applied(*kid) == 0 && applied(*grandkid) == 0, "... and leave it again");

  // The state key, both ways.
  check(geo->getState("clip_planes").value<std::string>() == "0,0,0,0,1,0",
        "setClipPlanes mirrors into the \"clip_planes\" state key",
        geo->getState("clip_planes").value<std::string>());
  geo->getState("clip_planes").value(std::string("0,0,0.25,0,0,-1"));
  sg.processEvents();
  check(geo->clipPlanes().size() == 1 && geo->clipPlanes()[0].normal[2] == -1.0 &&
            normalIs(appliedPlane(*geo, 0), 0, 0, -1) &&
            originIs(appliedPlane(*geo, 0), 0, 0, 0.25),
        "a \"clip_planes\" state write sets the planes");
  geo->getState("clip_planes").value(std::string("1,2,3"));
  sg.processEvents();
  check(geo->clipPlanes().size() == 1 && geo->clipPlanes()[0].normal[2] == -1.0,
        "a malformed write is ignored");
  geo->setClipPlanes({});
  check(geo->clipPlanes().empty() && applied(*geo) == 2 &&
            geo->getState("clip_planes").value<std::string>().empty(),
        "setClipPlanes({}) clears them (the parent's still apply)");

  // A volren node clips its VOLUME (cut planes), never its billboard quad.
  auto ren = group->addGraphicsChild<cvc::gl::VolRenNode>("ren");
  check(ren->maxClipPlanes() == 8 && applied(*ren) == 2 &&
            mapperOf(*ren)->GetNumberOfClippingPlanes() == 0,
        "a VolRenNode takes the planes, and its quad's mapper does not");
}

void testLowMemory(cvc::app &app) {
  std::printf("low-memory mapper: honours no planes (no GL)\n");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Force);
  SceneGraph sg(app, "clip_lowmem");
  auto geo = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("geo");
  geo->setGeometry(quad(app, -1, -1, 1, 1));
  geo->setClipPlanes({kKeepPlusX});
  check(geo->maxClipPlanes() == 0 && geo->clipPlaneCount() == 1 && applied(*geo) == 0 &&
            mapperOf(*geo)->GetNumberOfClippingPlanes() == 0,
        "clipped by 1 plane, hands the low-memory mapper none");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Auto);
}

// ── render ──────────────────────────────────────────────────────────────────
// Lit pixels in columns [x0, x1).
long litIn(const Frame &f, int x0, int x1) {
  long n = 0;
  for (int y = 0; y < f.h; ++y)
    for (int x = std::max(0, x0); x < std::min(f.w, x1); ++x)
      n += isBg(f.at(x, y)) ? 0 : 1;
  return n;
}

// Unclipped: lit both sides of the centre column (world x = 0 there). Clipped
// (keep x >= 0): nothing left of it (two columns of slack for the edge), lit on
// the right.
void checkHalf(const std::string &what, const Frame &whole, const Frame &cut) {
  const long l0 = litIn(whole, 0, W / 2 - 2), r0 = litIn(whole, W / 2 + 2, W);
  const long l1 = litIn(cut, 0, W / 2 - 2), r1 = litIn(cut, W / 2 + 2, W);
  const std::string d = "left/right " + std::to_string(l0) + "/" + std::to_string(r0) + " -> " +
                        std::to_string(l1) + "/" + std::to_string(r1);
  check(l0 > 100 && r0 > 100, what + ": unclipped, drawn on both sides", d);
  check(l1 == 0 && r1 > r0 / 2,
        what + ": keep x >= 0 -- nothing left of the cut, drawn right of it", d);
}

void sideView(SceneRenderer &sr) {
  sr.setBackground(0, 0, 0);
  sr.setCamera(0, 0, 6, 0, 0, 0, 0, 1, 0, /*viewAngle=*/30.0, /*near=*/0.5, /*far=*/60.0);
}

void testRenderGeometry(cvc::app &app) {
  std::printf("render: a geometry quad (classic mapper)\n");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Off);
  SceneGraph sg(app, "clipr_geo");
  sg.setDiagnosticChromeVisible(false);
  auto geo = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("geo");
  geo->setGeometry(quad(app, -1, -1, 1, 1));
  flat(*geo, 1.0, 0.1, 0.1);
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sideView(sr);
  const Frame whole = grab(sr);
  geo->setClipPlanes({kKeepPlusX});
  checkHalf("geometry", whole, grab(sr));

  // Moved in place (same count): the cut moves to x = 0.5.
  geo->setClipPlanes({plane(0.5, 0, 0, 1, 0, 0)});
  Frame f = grab(sr);
  check(bgNear(f, toPx(sr, 0.25, 0, 0), 1) && hueNear(f, toPx(sr, 0.75, 0, 0), 1, 0),
        "the plane moved in place to x = 0.5: the cut follows");
  // The node moves -0.5: the quad spans x [-1.5, 0.5], its plane (local 0.5)
  // sits at world 0.
  geo->setPosition(-0.5, 0, 0);
  f = grab(sr);
  check(hueNear(f, toPx(sr, 0.25, 0, 0), 1, 0) && bgNear(f, toPx(sr, -0.25, 0, 0), 1),
        "the node moved: its plane moved with it (local frame)");
  // Rotated a quarter turn and squashed in x: local keep x >= 0 is now world
  // keep y >= 0, over the quad's (unchanged) height.
  geo->setClipPlanes({kKeepPlusX});
  geo->setPosition(0, 0, 0);
  geo->setRotation(0, 0, 90);
  geo->setScale(0.5, 1, 1);
  f = grab(sr);
  check(hueNear(f, toPx(sr, 0, 0.25, 0), 1, 0) && hueNear(f, toPx(sr, 0.3, 0.25, 0), 1, 0) &&
            bgNear(f, toPx(sr, 0, -0.25, 0), 1) && bgNear(f, toPx(sr, 0.3, -0.25, 0), 1),
        "rotated 90 deg and scaled: the cut turned with the node (local x >= 0 = world y >= 0)");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Auto);
}

void testRenderVolume(cvc::app &app) {
  std::printf("render: a VolumeNode (VTK GPU raycast)\n");
  SceneGraph sg(app, "clipr_vol");
  sg.setDiagnosticChromeVisible(false);
  auto vol = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolumeNode>("vol");
  vol->setVolume(ball(app));
  // Solid red wherever the ball has any density (the default grayscale TF is
  // too faint to measure), unlit.
  vol->setTransferFunction({0.0, 1, 0.1, 0.1, 1.0, 1, 0.1, 0.1}, {0.0, 0.0, 0.05, 1.0, 1.0, 1.0});
  vol->setShading(false);
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sideView(sr);
  sr.render();
  const Frame whole = grab(sr);
  vol->setClipPlanes({kKeepPlusX});
  checkHalf("VolumeNode", whole, grab(sr));
}

void testRenderVolSlice(cvc::app &app) {
  std::printf("render: a VolSliceNode\n");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Off);
  SceneGraph sg(app, "clipr_slice");
  sg.setDiagnosticChromeVisible(false);
  auto vs = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolSliceNode>("slice");
  sg.registerGraphics("slice", vs);
  vs->setVolume(ball(app));
  cvc::volslice::render_settings s;
  s.tf.add({0.0, 1.f, 0.f, 0.f, 0.f});
  s.tf.add({1.0, 1.f, 0.f, 0.f, 0.6f});
  vs->setConfig(s);
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sideView(sr);
  auto pump = [&]() {
    for (int i = 0; i < 4; ++i) {
      sg.processEvents();
      vs->tick();
      sr.render();
    }
    return grab(sr);
  };
  const Frame whole = pump();
  vs->setClipPlanes({kKeepPlusX});
  checkHalf("VolSliceNode", whole, pump());
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Auto);
}

void testRenderVolRen(cvc::app &app) {
  std::printf("render: a VolRenNode (software raycaster, cut planes)\n");
  SceneGraph sg(app, "clipr_ren");
  sg.setDiagnosticChromeVisible(false);
  auto ren = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolRenNode>("ren");
  sg.registerGraphics("ren", ren);
  cvc::volren::volume_settings v;
  cvc::volren::isosurface iso;
  iso.value = 0.0;
  iso.opacity = 1.0f;
  iso.color = {1.f, 0.1f, 0.1f};
  v.isosurfaces.push_back(iso);
  ren->addVolume(sphereSdf(app, 0.6), v);
  ren->setResolutionScale(1.0);
  {
    cvc::volren::render_settings rs = ren->renderConfig();
    cvc::volren::light l;
    l.color = {1.f, 1.f, 1.f};
    l.direction = {0.3, 0.4, 0.85};
    rs.lights = {l};
    rs.ambient = 0.5f;
    rs.two_sided_lighting = true;
    rs.steps = 128;
    ren->setRenderConfig(rs);
  }
  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sideView(sr);
  sr.render();
  // Until what is on screen matches the live camera, settings and clip planes.
  auto settle = [&]() {
    const auto start = std::chrono::steady_clock::now();
    int settled = 0;
    while (settled < 3 && std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
      sr.processUIEvents();
      ren->tick();
      sr.render();
      settled = ren->converged() && ren->framesRendered() > 0 ? settled + 1 : 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return settled >= 3;
  };
  if (!check(settle(), "the raycast converges"))
    return;
  const Frame whole = grab(sr);
  ren->setClipPlanes({kKeepPlusX});
  ren->tick();
  check(!ren->converged(), "a new clip plane makes the frame stale (it re-raycasts)");
  settle();
  checkHalf("VolRenNode", whole, grab(sr));
}

} // namespace

int main() {
  installErrorCounter();
  cvc::app app;
  testWiring(app);
  testLocalFrame(app);
  testLowMemory(app);
  if (renderAvailable(app)) {
    {
      SceneGraph sg(app, "gl_probe");
      SceneRenderer sr(sg, 16, 16, /*offscreen=*/true);
      sr.render();
      printRenderer(sr);
    }
    testRenderGeometry(app);
    testRenderVolume(app);
    testRenderVolSlice(app);
    testRenderVolRen(app);
  }
  check(ErrorCounter::errors() == 0, "no VTK errors overall",
        std::to_string(ErrorCounter::errors()));
  return finish("cvcgl_clip_planes");
}
