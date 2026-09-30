// cvcgl_lod_node -- LodGraphicsNode and the SceneGraph LOD pass, end to end.
//
// select() is the seam between cvc::lod's pure math and the scene graph, and the
// scene graph is where LOD used to go wrong: the math was right and the node
// still drew the wrong thing. So most of this is about WHAT DRAWS:
//
//   * distance selection -- close picks the finest rung, far the coarsest,
//     monotone in between;
//   * user visibility (B4) -- a hidden LOD node, or one under a hidden parent,
//     stays hidden through selectLOD, and re-showing the parent shows exactly ONE
//     rung, not the whole ladder SceneNode::setVisible used to propagate to --
//     at once, without a select(), even for a node attached or moved under an
//     already-hidden parent;
//   * switching (B5) -- a switch flips actor visibility and every rung stays
//     registered, so the renderer's prop count never moves;
//   * re-population (B6) -- setPyramid replaces the old rungs rather than
//     leaking them, and the rung style reaches every current rung;
//   * progressive attach -- setBase shows rung 0 at once and appendRungs keeps
//     that very node when the pyramid's rung 0 matches;
//   * make_view_params -- a real vtkCamera's vertical ViewAngle and
//     ParallelScale become the numbers select() needs;
//   * setLODEnabled / lod_stats, and a FEATURE-ON synthetic city where the far
//     poses must actually save triangles.
//
// Everything above runs headless (a bare vtkRenderer needs no window). The last
// section renders offscreen -- rung colours on screen, a zero-error switch is
// pixel-identical, and the shadow-bake hook -- and skips itself when this build
// cannot rasterise.
//
// NOT assert(): these tests build Release (NDEBUG), where assert() compiles to
// nothing and a "passing" run would prove nothing.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/select.h>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCameraPass.h>
#include <vtkNew.h>
#include <vtkProp.h>
#include <vtkPropCollection.h>
#include <vtkProperty.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSequencePass.h>
#include <vtkShadowMapBakerPass.h>

using cvc::gl::GeometryNode;
using cvc::gl::lod_stats;
using cvc::gl::LodGraphicsNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

namespace {

int g_failures = 0;
void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

constexpr double kPi = 3.14159265358979323846;

cvc::geometry unit_box(cvc::app &app, double cx = 0.0, double cy = 0.0, double cz = 0.0) {
  cvc::geometry g(app);
  cvc::geometry::points_t &P = g.points();
  for (int i = 0; i < 8; ++i)
    P.push_back(
        {cx + ((i & 1) ? 0.5 : -0.5), cy + ((i & 2) ? 0.5 : -0.5), cz + ((i & 4) ? 0.5 : -0.5)});
  const int faces[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                            {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  cvc::geometry::tris_t &T = g.tris();
  for (const auto &f : faces)
    T.push_back({std::uint64_t(f[0]), std::uint64_t(f[1]), std::uint64_t(f[2])});
  g.set_geometry_type(cvc::geometry::SURFACE_TRI);
  return g;
}

// One box per entry of `errors`, with that error ladder. The ladder, not the
// geometry, drives selection; rung k keeps only the first 12 >> k of the box's
// faces (12, 6, 3, 1) so the triangle accounting is observable.
cvc::lod::mesh_pyramid box_pyramid(cvc::app &app, const std::vector<double> &errors,
                                   double cx = 0.0, double cy = 0.0, double cz = 0.0) {
  cvc::lod::mesh_pyramid pyr;
  for (std::size_t k = 0; k < errors.size(); ++k) {
    cvc::geometry g = unit_box(app, cx, cy, cz);
    g.tris().resize(12 >> k); // 12, 6, 3, 1: a coarser rung has fewer triangles
    pyr.rungs.push_back(g);
  }
  pyr.world_error_m = errors;
  return pyr;
}

// How many of the node's rung actors are visible, and which (-1 none, -2 many).
int visible_count(const LodGraphicsNode &lod) {
  int n = 0;
  for (int k = 0; k < lod.rungCount(); ++k)
    if (lod.rung(k)->prop()->GetVisibility())
      ++n;
  return n;
}
int visible_rung(const LodGraphicsNode &lod) {
  int which = -1;
  for (int k = 0; k < lod.rungCount(); ++k)
    if (lod.rung(k)->prop()->GetVisibility())
      which = which == -1 ? k : -2;
  return which;
}

cvc::lod::view_params view_at(double x, double y, double z) {
  cvc::lod::view_params v = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  v.eye[0] = x;
  v.eye[1] = y;
  v.eye[2] = z;
  return v;
}

int props_in(vtkRenderer *ren) { return ren->GetViewProps()->GetNumberOfItems(); }

// Every rung's actor is in the renderer, drawn or not.
bool rungs_registered(vtkRenderer *ren, const LodGraphicsNode &lod) {
  for (int k = 0; k < lod.rungCount(); ++k)
    if (!ren->GetViewProps()->IsItemPresent(lod.rung(k)->prop()))
      return false;
  return true;
}

// --- distance selection --------------------------------------------------------

void test_distance_selection(cvc::app &app) {
  std::printf("distance selection\n");
  auto node = std::make_shared<LodGraphicsNode>(app, "test.lod", "lod");
  CHECK(node->select(view_at(0, 0, 1)) == -1); // empty: no rung
  CHECK(node->activeRung() == -1);

  node->setPyramid(box_pyramid(app, {0.0, 1.0, 4.0}));
  CHECK(node->rungCount() == 3);
  // Nothing selected yet, but rung 0 draws so the first frame has content.
  CHECK(node->selectedRung() == -1);
  CHECK(node->activeRung() == 0);
  CHECK(visible_rung(*node) == 0);
  CHECK(node->rungTriangles(0) == 12 && node->rungTriangles(2) == 3);
  CHECK(node->rungTriangles(3) == 0 && node->rungError(-1) == 0.0);
  CHECK(node->rung(3) == nullptr);

  CHECK(node->select(view_at(0, 0, 1.0)) == 0);   // close: the finest rung
  CHECK(node->select(view_at(0, 0, 1.0e6)) == 2); // very far: the coarsest
  CHECK(node->selectedRung() == 2);
  CHECK(visible_rung(*node) == 2);

  // Monotone coarsening as the camera retreats; exactly one rung visible.
  int prev = node->select(view_at(0, 0, 0.6));
  for (double d : {2.0, 10.0, 50.0, 200.0, 1000.0, 20000.0, 1.0e6}) {
    const int r = node->select(view_at(0, 0, d));
    CHECK(r >= prev);
    CHECK(visible_count(*node) == 1 && visible_rung(*node) == r);
    prev = r;
  }

  // setRung pins, clamped, and select() resumes from it.
  CHECK(node->setRung(7) == 2);
  CHECK(node->setRung(-3) == 0 && visible_rung(*node) == 0);
  CHECK(node->select(view_at(0, 0, 1.0e6)) == 2);
}

// --- visibility (B4) + switching keeps every rung registered (B5) --------------

void test_visibility_and_registration(cvc::app &app) {
  std::printf("visibility + registration\n");
  vtkNew<vtkRenderer> ren; // no window: prop bookkeeping only; outlives the scene
  SceneGraph sg(app, "lodvis");
  sg.setRenderer(ren);

  auto group = sg.addGraphics("group");
  auto lod = group->addGraphicsChild<LodGraphicsNode>("lod");
  lod->setPyramid(box_pyramid(app, {0.0, 1.0, 4.0}));
  const int props = props_in(ren);
  CHECK(rungs_registered(ren, *lod));

  const auto near = view_at(0, 0, 1.0);
  const auto far = view_at(0, 0, 1.0e6);

  // First selection lands on the rung 0 it already drew: no change.
  CHECK(sg.selectLOD(near) == 0);
  CHECK(sg.selectLOD(far) == 1);
  CHECK(visible_rung(*lod) == 2);

  // Many switches: one rung visible each time, and nothing is ever removed
  // from (or added to) the renderer -- RemoveViewProp would release the rung's
  // GPU buffers and force a re-upload on the switch back.
  for (int i = 0; i < 6; ++i) {
    sg.selectLOD(i % 2 ? far : near);
    CHECK(visible_count(*lod) == 1);
    CHECK(props_in(ren) == props);
  }

  // Hide the PARENT: nothing draws, and selectLOD must not bring it back. (The
  // group's own actor leaves the renderer, as any plain node's does; the rungs
  // stay registered.)
  sg.selectLOD(far);
  group->setVisible(false);
  CHECK(visible_count(*lod) == 0);
  CHECK(rungs_registered(ren, *lod));
  sg.selectLOD(near);
  CHECK(visible_count(*lod) == 0);
  CHECK(lod->selectedRung() == 0); // ...but the choice kept tracking the camera
  // Show the parent: exactly ONE rung -- the current one -- not all three.
  group->setVisible(true);
  CHECK(visible_count(*lod) == 1);
  CHECK(visible_rung(*lod) == 0);

  // The node's own visibility behaves the same.
  lod->setVisible(false);
  CHECK(visible_count(*lod) == 0);
  sg.selectLOD(far);
  CHECK(visible_count(*lod) == 0);
  lod->setVisible(true);
  CHECK(visible_rung(*lod) == 2);
  CHECK(props_in(ren) == props);

  // A LOD node added under an ALREADY-hidden parent: setVisible never reached
  // it, so only the hierarchy walk keeps it dark.
  auto dark = sg.addGraphics("dark");
  dark->setVisible(false);
  auto lod2 = dark->addGraphicsChild<LodGraphicsNode>("lod2");
  lod2->setPyramid(box_pyramid(app, {0.0, 1.0, 4.0}));
  CHECK(visible_count(*lod2) == 0);
  lod_stats st;
  sg.selectLOD(near, &st);
  CHECK(visible_count(*lod2) == 0);
  CHECK(st.nodes == 2 && st.hidden == 1);
  CHECK(st.drawn_tris == 12 && st.full_tris == 12); // the hidden node draws nothing

  // Showing that parent brings back exactly one rung with NO select(): lod2's
  // own flag never changed, so only the ancestor notification reaches it. A
  // caller that selects only when the camera moves depends on this.
  dark->setVisible(true);
  CHECK(visible_count(*lod2) == 1 && visible_rung(*lod2) == 0);
  dark->setVisible(false);
  CHECK(visible_count(*lod2) == 0);

  // Moving a DRAWING node under a hidden parent darkens it at once, and
  // showing that parent brings back its current rung -- again, no select().
  sg.selectLOD(far);
  CHECK(visible_rung(*lod) == 2);
  group->removeGraphicsChild(lod);
  dark->addGraphicsChild(lod);
  CHECK(lod->isVisible() && visible_count(*lod) == 0);
  CHECK(rungs_registered(ren, *lod));
  dark->setVisible(true);
  CHECK(visible_rung(*lod) == 2 && visible_rung(*lod2) == 2);

  // Two levels down: a plain group attached under a hidden grandparent keeps
  // its own `true`, so showing the grandparent flips no flag below it at all.
  auto grand = sg.addGraphics("grand");
  grand->setVisible(false);
  auto mid = grand->createChild("mid");
  auto plain = mid->createChild("plain");
  plain->setVisible(false); // hidden on its own
  auto lod3 = mid->addGraphicsChild<LodGraphicsNode>("lod3");
  lod3->setPyramid(box_pyramid(app, {0.0, 1.0, 4.0}));
  CHECK(mid->isVisible() && visible_count(*lod3) == 0);
  grand->setVisible(true);
  CHECK(visible_count(*lod3) == 1 && visible_rung(*lod3) == 0);
  CHECK(!plain->isVisible()); // an ordinary node's own flag is left alone, as before

  // Shown on its own while an ancestor is hidden: the ancestor wins until it is
  // shown as well, by which time the node's flag already matches.
  grand->setVisible(false);
  CHECK(!lod3->isVisible() && visible_count(*lod3) == 0);
  lod3->setVisible(true);
  CHECK(visible_count(*lod3) == 0);
  grand->setVisible(true);
  CHECK(visible_count(*lod3) == 1 && visible_rung(*lod3) == 0);

  // Detaching the scene's renderer takes every rung with it.
  sg.setRenderer(nullptr);
  CHECK(!ren->GetViewProps()->IsItemPresent(lod->rung(0)->prop()));
  CHECK(!ren->GetViewProps()->IsItemPresent(lod->rung(2)->prop()));
}

// --- re-population (B6) and the rung style -------------------------------------

bool styled(const std::shared_ptr<GeometryNode> &g, double r, double gr, double b) {
  const double *c = vtkActor::SafeDownCast(g->prop())->GetProperty()->GetColor();
  return g->hasMetadata("styled") && c[0] == r && c[1] == gr && c[2] == b;
}

void test_repopulate_and_style(cvc::app &app) {
  std::printf("setPyramid twice + setRungStyle\n");
  vtkNew<vtkRenderer> ren;
  SceneGraph sg(app, "lodstyle");
  sg.setRenderer(ren);
  auto lod = sg.getGraphicsRoot()->addGraphicsChild<LodGraphicsNode>("lod");
  const int props0 = props_in(ren);

  lod->setRungStyle([](GeometryNode &g) {
    g.setColor(1.0, 0.0, 0.0);
    g.setMetadata("styled", true);
  });
  lod->setPyramid(box_pyramid(app, {0.0, 1.0, 4.0}));
  CHECK(lod->getGraphicsChildren().size() == 3);
  for (int k = 0; k < 3; ++k)
    CHECK(styled(lod->rung(k), 1.0, 0.0, 0.0));
  sg.selectLOD(view_at(0, 0, 1.0e6));
  CHECK(lod->selectedRung() == 2);

  std::vector<std::weak_ptr<GeometryNode>> old;
  for (int k = 0; k < 3; ++k)
    old.push_back(lod->rung(k));

  // Again, with a different ladder. The old rungs must be gone -- destroyed,
  // not just hidden -- and every new rung styled.
  lod->setPyramid(box_pyramid(app, {0.0, 0.5, 2.0, 8.0}));
  CHECK(lod->rungCount() == 4);
  CHECK(lod->getGraphicsChildren().size() == 4);
  for (const auto &w : old)
    CHECK(w.expired());
  for (int k = 0; k < 4; ++k)
    CHECK(styled(lod->rung(k), 1.0, 0.0, 0.0));
  CHECK(lod->selectedRung() == -1 && visible_rung(*lod) == 0); // selection reset
  CHECK(props_in(ren) == props0 + 4);                          // old actors left

  // A new style reaches the existing rungs too.
  lod->setRungStyle([](GeometryNode &g) {
    g.setColor(0.0, 1.0, 0.0);
    g.setMetadata("styled", true);
  });
  for (int k = 0; k < 4; ++k)
    CHECK(styled(lod->rung(k), 0.0, 1.0, 0.0));
  lod->setRungStyle(nullptr); // stops styling new rungs; existing ones keep it
  CHECK(styled(lod->rung(3), 0.0, 1.0, 0.0));

  // An empty pyramid empties the node.
  lod->setPyramid(cvc::lod::mesh_pyramid());
  CHECK(lod->rungCount() == 0 && lod->getGraphicsChildren().empty());
  CHECK(props_in(ren) == props0);
  CHECK(lod->getBoundingBox().isNull());
}

// --- progressive attach -------------------------------------------------------

void test_progressive_attach(cvc::app &app) {
  std::printf("setBase + appendRungs\n");
  vtkNew<vtkRenderer> ren;
  SceneGraph sg(app, "lodappend");
  sg.setRenderer(ren);
  auto lod = sg.getGraphicsRoot()->addGraphicsChild<LodGraphicsNode>("lod");
  int styleCalls = 0;
  lod->setRungStyle([&styleCalls](GeometryNode &g) {
    ++styleCalls;
    g.setMetadata("styled", true);
  });

  const cvc::geometry base = unit_box(app);
  lod->setBase(base);
  CHECK(lod->rungCount() == 1 && visible_rung(*lod) == 0);
  CHECK(lod->selectedRung() == -1);
  CHECK(sg.selectLOD(view_at(0, 0, 1.0e6)) == 0); // one rung: nowhere to go
  const std::shared_ptr<GeometryNode> rung0 = lod->rung(0);
  vtkProp *const actor0 = rung0->prop();
  CHECK(styleCalls == 1);

  // The pyramid built from the base: its rung 0 is a copy that shares storage.
  cvc::lod::mesh_pyramid pyr = box_pyramid(app, {0.0, 1.0, 4.0});
  pyr.rungs[0] = base;
  CHECK(lod->appendRungs(pyr));
  CHECK(lod->rungCount() == 3 && lod->getGraphicsChildren().size() == 3);
  CHECK(lod->rung(0) == rung0 && lod->rung(0)->prop() == actor0); // same node, same actor
  CHECK(styleCalls == 3);                                         // only the new rungs
  CHECK(lod->rung(1)->hasMetadata("styled") && lod->rung(2)->hasMetadata("styled"));
  CHECK(lod->rungError(2) == 4.0);
  CHECK(sg.selectLOD(view_at(0, 0, 1.0e6)) == 1 && visible_rung(*lod) == 2);

  // A DEEP copy with identical content still matches (content, not identity),
  // replaces the coarser rungs, and keeps the current selection in range.
  cvc::geometry deep(app);
  deep.copy(base, /*deepCopy=*/true);
  cvc::lod::mesh_pyramid pyr2 = box_pyramid(app, {0.0, 2.0});
  pyr2.rungs[0] = deep;
  std::weak_ptr<GeometryNode> oldRung2 = lod->rung(2);
  CHECK(lod->appendRungs(pyr2));
  CHECK(lod->rungCount() == 2 && lod->getGraphicsChildren().size() == 2);
  CHECK(lod->rung(0) == rung0);
  CHECK(oldRung2.expired());
  CHECK(lod->selectedRung() == 1 && visible_rung(*lod) == 1);

  // A different rung 0 cannot reuse the node: full replace, selection reset.
  cvc::lod::mesh_pyramid other = box_pyramid(app, {0.0, 1.0}, 5.0, 0.0, 0.0);
  CHECK(!lod->appendRungs(other));
  CHECK(lod->rungCount() == 2 && lod->rung(0) != rung0);
  CHECK(lod->selectedRung() == -1 && visible_rung(*lod) == 0);

  // An empty pyramid attaches nothing and disturbs nothing.
  const std::shared_ptr<GeometryNode> keep = lod->rung(0);
  CHECK(!lod->appendRungs(cvc::lod::mesh_pyramid()));
  CHECK(lod->rungCount() == 2 && lod->rung(0) == keep);

  // appendRungs on an empty node is setPyramid.
  auto fresh = sg.getGraphicsRoot()->addGraphicsChild<LodGraphicsNode>("fresh");
  CHECK(!fresh->appendRungs(pyr));
  CHECK(fresh->rungCount() == 3);
}

// --- make_view_params from a real VTK camera -----------------------------------

void test_make_view_params(cvc::app &app) {
  std::printf("make_view_params\n");
  // A window gives the renderer a pixel size; nothing here renders, so no GL
  // context is ever created.
  vtkNew<vtkRenderWindow> win;
  win->SetOffScreenRendering(1);
  win->SetSize(320, 240);
  vtkNew<vtkRenderer> ren;
  win->AddRenderer(ren);
  vtkCamera *cam = ren->GetActiveCamera();
  cam->SetPosition(1.0, 2.0, 3.0);
  cam->SetFocalPoint(0.0, 0.0, 0.0);
  cam->SetViewAngle(42.0);

  const auto base = cvc::lod::preset_view(cvc::lod::quality_preset::aggressive);
  cvc::lod::view_params v = cvc::gl::make_view_params(ren, base);
  const double t21 = std::tan(21.0 * kPi / 180.0);
  CHECK(std::fabs(v.tan_half_fov - t21) < 1e-12); // ViewAngle is VERTICAL, degrees
  CHECK(v.viewport_h_px == 240.0);
  CHECK(std::fabs(cvc::lod::k_px(v) - 240.0 / (2.0 * t21)) < 1e-9);
  CHECK(v.eye[0] == 1.0 && v.eye[1] == 2.0 && v.eye[2] == 3.0);
  CHECK(v.ortho_px_per_m == 0.0);
  CHECK(v.desired_pixel_error == base.desired_pixel_error); // the budget is the caller's
  CHECK(v.hysteresis == base.hysteresis && v.z_near == base.z_near);

  // A horizontal ViewAngle is converted to the vertical half-angle.
  cam->UseHorizontalViewAngleOn();
  v = cvc::gl::make_view_params(ren, base);
  CHECK(std::fabs(v.tan_half_fov - t21 * 240.0 / 320.0) < 1e-12);
  cam->UseHorizontalViewAngleOff();

  // The renderer's viewport, not the window: a half-height split view.
  ren->SetViewport(0.0, 0.0, 1.0, 0.5);
  CHECK(cvc::gl::make_view_params(ren, base).viewport_h_px == 120.0);
  ren->SetViewport(0.0, 0.0, 1.0, 1.0);

  // PARALLEL: px per metre = h / (2 * ParallelScale).
  cam->ParallelProjectionOn();
  cam->SetParallelScale(1.0);
  const cvc::lod::view_params zoomedIn = cvc::gl::make_view_params(ren, base);
  CHECK(zoomedIn.ortho_px_per_m == 120.0);
  cam->SetParallelScale(1000.0);
  const cvc::lod::view_params zoomedOut = cvc::gl::make_view_params(ren, base);
  CHECK(std::fabs(zoomedOut.ortho_px_per_m - 0.12) < 1e-12);

  // Two scales, two rungs -- and in parallel projection the distance is moot.
  auto lod = std::make_shared<LodGraphicsNode>(app, "test.ortho", "lod");
  lod->setPyramid(box_pyramid(app, {0.0, 0.05, 1.0}));
  CHECK(lod->select(zoomedIn) == 0);  // 0.05 m at 120 px/m = 6 px: too coarse
  CHECK(lod->select(zoomedOut) == 2); // 1 m at 0.12 px/m: well under budget
  cvc::lod::view_params farIn = zoomedIn;
  farIn.eye[2] = 1.0e7; // a perspective camera here would pick the coarsest
  CHECK(lod->select(farIn) == 0);

  // No renderer: the base comes back untouched.
  const cvc::lod::view_params none = cvc::gl::make_view_params(nullptr, base);
  CHECK(none.viewport_h_px == base.viewport_h_px && none.tan_half_fov == base.tan_half_fov);
  // A renderer with no window keeps base's height (its size is 0 x 0).
  vtkNew<vtkRenderer> bare;
  CHECK(cvc::gl::make_view_params(bare, base).viewport_h_px == base.viewport_h_px);
  CHECK(!bare->IsActiveCameraCreated());

  // A renderer with no camera yet does not get one from here -- VTK auto-frames
  // only the camera it creates during the first render. Its height is read; the
  // eye and projection stay base's.
  vtkNew<vtkRenderWindow> win2;
  win2->SetOffScreenRendering(1);
  win2->SetSize(200, 100);
  vtkNew<vtkRenderer> fresh;
  win2->AddRenderer(fresh);
  cvc::lod::view_params seeded = base;
  seeded.eye[0] = 7.0;
  const cvc::lod::view_params vf = cvc::gl::make_view_params(fresh, seeded);
  CHECK(!fresh->IsActiveCameraCreated());
  CHECK(vf.viewport_h_px == 100.0);
  CHECK(vf.eye[0] == 7.0 && vf.tan_half_fov == base.tan_half_fov && vf.ortho_px_per_m == 0.0);
}

// --- setLODEnabled + lod_stats -------------------------------------------------

void test_enable_and_stats(cvc::app &app) {
  std::printf("setLODEnabled + stats\n");
  SceneGraph sg(app, "lodenable");
  auto root = sg.getGraphicsRoot();
  auto nearNode = root->addGraphicsChild<LodGraphicsNode>("near");
  auto farNode = root->addGraphicsChild<LodGraphicsNode>("far");
  nearNode->setPyramid(box_pyramid(app, {0.0, 1.0, 4.0}));
  farNode->setPyramid(box_pyramid(app, {0.0, 1.0, 4.0}, 1.0e6, 0.0, 0.0));
  const auto eye = view_at(0, 0, 2.0); // beside the near node

  lod_stats st;
  CHECK(sg.lodEnabled());
  CHECK(sg.selectLOD(eye, &st) == 1); // the far node leaves rung 0
  CHECK(nearNode->selectedRung() == 0 && farNode->selectedRung() == 2);
  CHECK(st.nodes == 2 && st.hidden == 0 && st.changes == 1);
  CHECK(std::accumulate(st.rung_nodes.begin(), st.rung_nodes.end(), 0) == st.nodes);
  CHECK(st.rung_nodes.size() == 3 && st.rung_nodes[0] == 1 && st.rung_nodes[2] == 1);
  CHECK(st.full_tris == 24 && st.drawn_tris == 12 + 3);

  // Off: everything back to full detail, counted as switches.
  sg.setLODEnabled(false);
  CHECK(!sg.lodEnabled());
  CHECK(sg.selectLOD(eye, &st) == 1);
  CHECK(nearNode->selectedRung() == 0 && farNode->selectedRung() == 0);
  CHECK(visible_rung(*farNode) == 0);
  CHECK(st.rung_nodes[0] == 2 && st.rung_nodes[2] == 0); // storage kept, zeroed
  CHECK(st.drawn_tris == st.full_tris);
  // Nodes added while off are pinned too.
  auto late = root->addGraphicsChild<LodGraphicsNode>("late");
  late->setPyramid(box_pyramid(app, {0.0, 1.0, 4.0}, -1.0e6, 0.0, 0.0));
  CHECK(sg.selectLOD(eye, &st) == 0 && late->selectedRung() == 0);
  CHECK(st.nodes == 3 && st.rung_nodes[0] == 3);

  // Back on: the far nodes coarsen again.
  sg.setLODEnabled(true);
  sg.setLODEnabled(true); // idempotent
  CHECK(sg.selectLOD(eye, &st) == 2);
  CHECK(farNode->selectedRung() == 2 && late->selectedRung() == 2);
  CHECK(nearNode->selectedRung() == 0);
  CHECK(st.drawn_tris < st.full_tris);

  // Empty LOD nodes are not counted.
  root->addGraphicsChild<LodGraphicsNode>("empty");
  CHECK(sg.selectLOD(eye, &st) == 0 && st.nodes == 3);

  sg.invalidateShadowBake(); // no renderer, no shadows: a harmless no-op
}

// --- FEATURE-ON: a synthetic tiled city ---------------------------------------

// One city tile: an n x n heightfield over a `size` square with a smooth,
// non-planar relief, so decimation really removes detail.
cvc::geometry city_tile(cvc::app &app, double x0, double y0, double size, int n) {
  cvc::geometry g(app);
  cvc::geometry::points_t &P = g.points();
  for (int j = 0; j <= n; ++j)
    for (int i = 0; i <= n; ++i) {
      const double x = x0 + size * i / n;
      const double y = y0 + size * j / n;
      const double z =
          6.0 + 4.0 * std::sin(0.21 * x) * std::cos(0.17 * y) + 2.0 * std::sin(0.53 * (x + y));
      P.push_back({x, y, z});
    }
  cvc::geometry::tris_t &T = g.tris();
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const std::uint64_t a = std::uint64_t(j) * (n + 1) + i, b = a + 1, c = a + (n + 1), d = c + 1;
      T.push_back({a, b, d});
      T.push_back({a, d, c});
    }
  g.set_geometry_type(cvc::geometry::SURFACE_TRI);
  return g;
}

void test_feature_on_city(cvc::app &app) {
  std::printf("feature-on synthetic city\n");
  const int G = 6;          // 6 x 6 tiles
  const double pitch = 100; // metres per tile; the city spans [0, 600]^2
  const int N = G * G;

  cvc::lod::pyramid_params pp;
  pp.max_rungs = 3;
  pp.preserve_boundary = false;

  // Switch radii are AUTHORED (400 m / 1.2 km / 3.6 km against the balanced
  // reference view) rather than read off the simplifier, so the fixed poses
  // below mean the same thing whatever error metric cvc::simplify reports.
  // The triangle counts are the real decimated ones.
  const cvc::lod::view_params ref = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  const double radii[4] = {0.0, 400.0, 1200.0, 3600.0};

  vtkNew<vtkRenderer> ren;
  SceneGraph sg(app, "lodcity");
  sg.setRenderer(ren);
  auto city = sg.addGraphics("city");
  std::vector<cvc::lod::mesh_pyramid> real;
  for (int ty = 0; ty < G; ++ty)
    for (int tx = 0; tx < G; ++tx) {
      cvc::lod::mesh_pyramid pyr =
          cvc::lod::build_mesh_pyramid(city_tile(app, tx * pitch, ty * pitch, pitch, 32), pp);
      real.push_back(pyr);
      for (std::size_t k = 1; k < pyr.world_error_m.size() && k < 4; ++k)
        pyr.world_error_m[k] = cvc::lod::world_error_for_switch_radius(radii[k], ref);
      auto tile = city->addGraphicsChild<LodGraphicsNode>("tile" + std::to_string(ty * G + tx));
      tile->setPyramid(pyr);
    }
  CHECK(real.front().rungs.size() == 4); // 2048 -> 717 -> 251 -> 88 targets
  const int props = props_in(ren);

  lod_stats st;
  // NEAR: street level over the middle of town -- every tile within 400 m.
  CHECK(sg.selectLOD(view_at(300, 300, 20), &st) == 0);
  CHECK(st.nodes == N && st.rung_nodes.size() == 1 && st.rung_nodes[0] == N);
  CHECK(st.drawn_tris == st.full_tris && st.full_tris == std::uint64_t(N) * 2048);

  // FAR, overhead at 2.5 km: every tile beyond 1.2 km.
  CHECK(sg.selectLOD(view_at(300, 300, 2500), &st) == N);
  CHECK(std::accumulate(st.rung_nodes.begin(), st.rung_nodes.end(), 0) == N);
  CHECK(st.rung_nodes[0] == 0 && st.rung_nodes[2] == N);
  CHECK(st.drawn_tris * 3 <= st.full_tris);
  std::printf("  overhead 2.5 km: %llu of %llu tris\n", (unsigned long long)st.drawn_tris,
              (unsigned long long)st.full_tris);

  // FAR, 8 km out and 1 km up: every tile beyond 3.6 km, coarsest rung.
  sg.selectLOD(view_at(300, -8000, 1000), &st);
  CHECK(st.rung_nodes.size() == 4 && st.rung_nodes[3] == N);
  CHECK(st.drawn_tris * 3 <= st.full_tris);
  std::printf("  8 km out: %llu of %llu tris\n", (unsigned long long)st.drawn_tris,
              (unsigned long long)st.full_tris);

  // A low oblique view from just outside town: near rows fine, far rows coarse.
  sg.selectLOD(view_at(300, -300, 50), &st);
  int distinct = 0;
  for (int c : st.rung_nodes)
    distinct += c > 0 ? 1 : 0;
  CHECK(distinct >= 2 && st.rung_nodes[0] > 0);
  CHECK(st.drawn_tris < st.full_tris);

  // Back to street level: all fine again; and not one prop came or went.
  sg.selectLOD(view_at(300, 300, 20), &st);
  CHECK(st.rung_nodes[0] == N && st.drawn_tris == st.full_tris);
  CHECK(props_in(ren) == props);

  // The simplifier's OWN ladders, unmodified: whatever their scale, from far
  // enough away every tile lands on its coarsest rung and the saving is real.
  SceneGraph sg2(app, "lodcity2");
  for (int t = 0; t < N; ++t)
    sg2.getGraphicsRoot()
        ->addGraphicsChild<LodGraphicsNode>("t" + std::to_string(t))
        ->setPyramid(real[t]);
  sg2.selectLOD(view_at(300, 300, 1.0e7), &st);
  CHECK(st.rung_nodes.size() == 4 && st.rung_nodes[3] == N);
  CHECK(st.drawn_tris * 3 <= st.full_tris);
}

// --- offscreen renders ----------------------------------------------------------

struct counts {
  long red = 0, green = 0, blue = 0, lit = 0;
};

counts classify(SceneRenderer &sr) {
  counts c;
  const std::vector<unsigned char> px = sr.frameRGB();
  for (std::size_t i = 0; i + 2 < px.size(); i += 3) {
    const int r = px[i], g = px[i + 1], b = px[i + 2];
    if (r + g + b > 60)
      ++c.lit;
    if (r > 150 && g < 80 && b < 80)
      ++c.red;
    if (g > 150 && r < 80 && b < 80)
      ++c.green;
    if (b > 150 && r < 80 && g < 80)
      ++c.blue;
  }
  return c;
}

void flat(GeometryNode &g, double r, double gr, double b) {
  g.setColor(r, gr, b);
  g.setAmbient(1.0); // paint the colour flat: no dependence on the lights
  g.setDiffuse(0.0);
  g.setSpecular(0.0);
}

vtkShadowMapBakerPass *find_baker(vtkRenderPass *p) {
  if (!p)
    return nullptr;
  if (auto *b = vtkShadowMapBakerPass::SafeDownCast(p))
    return b;
  if (auto *cam = vtkCameraPass::SafeDownCast(p))
    return find_baker(cam->GetDelegatePass());
  if (auto *seq = vtkSequencePass::SafeDownCast(p))
    if (vtkRenderPassCollection *passes = seq->GetPasses()) {
      passes->InitTraversal();
      while (vtkRenderPass *child = passes->GetNextRenderPass())
        if (auto *b = find_baker(child))
          return b;
    }
  return nullptr;
}

void test_offscreen_render(cvc::app &app) {
  std::printf("offscreen render\n");
  SceneGraph sg(app, "lodrender");
  sg.setDiagnosticChromeVisible(false);
  auto group = sg.addGraphics("group");
  auto lod = group->addGraphicsChild<LodGraphicsNode>("lod");
  lod->setPyramid(box_pyramid(app, {0.0, 1.0, 4.0}));
  lod->setScale(4.0, 4.0, 4.0);
  flat(*lod->rung(0), 1.0, 0.0, 0.0); // red
  flat(*lod->rung(1), 0.0, 1.0, 0.0); // green
  flat(*lod->rung(2), 0.0, 0.0, 1.0); // blue

  SceneRenderer sr(sg, 64, 64, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.setCamera(6, 5, 12, 0, 0, 0, 0, 1, 0, 40.0, 0.5, 100.0);

  counts c = classify(sr);
  if (c.lit == 0) {
    std::printf("  skipped: this build did not rasterise\n");
    return;
  }
  // Before any selection rung 0 draws, alone.
  CHECK(c.red > 0 && c.green == 0 && c.blue == 0);

  // The selection view is independent of the render camera, so the rung can
  // be driven while the picture stays framed.
  sg.selectLOD(view_at(0, 0, 1.0e6));
  c = classify(sr);
  CHECK(c.blue > 0 && c.red == 0 && c.green == 0);

  group->setVisible(false);
  sg.selectLOD(view_at(0, 0, 1.0));
  CHECK(classify(sr).lit == 0); // hidden stays hidden through selectLOD
  group->setVisible(true);
  c = classify(sr);
  CHECK(c.red > 0 && c.green == 0 && c.blue == 0); // exactly one rung comes back

  // A zero-error switch between identical, identically styled rungs is
  // invisible: the style reached every rung before its first draw.
  auto same = sg.addGraphics("same");
  auto twin = same->addGraphicsChild<LodGraphicsNode>("twin");
  twin->setRungStyle([](GeometryNode &g) { flat(g, 0.9, 0.9, 0.2); });
  // Every rung the same full box, so only the rung index differs.
  cvc::lod::mesh_pyramid full;
  for (int k = 0; k < 3; ++k)
    full.rungs.push_back(unit_box(app, 3.0, 0.0, 0.0));
  full.world_error_m = {0.0, 0.0, 0.0};
  twin->setPyramid(full);
  twin->setRung(0);
  sr.render();
  const std::vector<unsigned char> a = sr.frameRGB();
  twin->setRung(2);
  const std::vector<unsigned char> b = sr.frameRGB();
  int maxDiff = 0;
  for (std::size_t i = 0; i < a.size() && i < b.size(); ++i)
    maxDiff = std::max(maxDiff, std::abs(int(a[i]) - int(b[i])));
  CHECK(a.size() == b.size() && maxDiff <= 2);

  // The shadow hook. With a long update interval a rung switch is NOT baked
  // on its own -- the stale map is reused -- until invalidateShadowBake().
  sg.addDirectionalLight(-40.0, 50.0);
  if (!sg.setShadowsEnabled(true)) {
    std::printf("  shadow hook skipped: shadows unavailable\n");
    return;
  }
  sg.setShadowUpdateInterval(1000);
  vtkShadowMapBakerPass *baker = find_baker(sr.renderer()->GetPass());
  CHECK(baker != nullptr);
  if (!baker)
    return;
  sr.render(); // counter 0: the first frame always bakes
  lod->setRung(1);
  sr.render();
  CHECK(!baker->GetNeedUpdate()); // skipped: the maps predate the switch
  lod->setRung(2);
  sg.invalidateShadowBake();
  sr.render();
  CHECK(baker->GetNeedUpdate()); // baked, because a prop changed since the last bake
  sr.render();
  CHECK(!baker->GetNeedUpdate()); // one-shot: back on the interval
}

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  cvc::app app;
  test_distance_selection(app);
  test_visibility_and_registration(app);
  test_repopulate_and_style(app);
  test_progressive_attach(app);
  test_make_view_params(app);
  test_enable_and_stats(app);
  test_feature_on_city(app);
  test_offscreen_render(app);
  std::printf("%s: cvcgl_lod_node (%d check%s failed)\n", g_failures == 0 ? "PASS" : "FAIL",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
