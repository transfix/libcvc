// A GraphicsNode's rotation and scale (docs/ARIADNE.md §5.4, Transforms).
//
// The local matrix setPosition, setRotation and setScale make is T Rz Ry Rx S: a point is scaled
// along the node's own axes, turned about the parent's x, then y, then z, then moved. These checks
// pin that:
//   1. it is the matrix scene_node_local_matrix makes (the physics world's), and a view entry's
//      transform: (vtkTransform's Translate, RotateZ, RotateY, RotateX, Scale), whichever of
//      setRotation and setScale comes first, and through realize_scene;
//   2. a rotation keeps the scale, a scale of 0 and one below 0 included, and a scale keeps the
//      rotation;
//   3. after a whole matrix (setTransform, setPoseMatrix, the "matrix" key), a rotation keeps the
//      matrix's scale and a scale keeps its rotation, pitched straight up and mirrored included;
//   4. the "rotation" and "scale" keys do what setRotation and setScale do;
//   5. resetTransform starts the rotation and scale over.
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include <array>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_geometry.h>
#include <cvc/core/app.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <memory>
#include <string>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

using cvc::gl::GraphicsNode;
using cvc::gl::NullGraphicNode;
using cvc::gl::SceneGraph;

static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

namespace {

using Mat = std::array<double, 16>;

struct Parts {
  double t[3] = {0.0, 0.0, 0.0};
  double r[3] = {0.0, 0.0, 0.0}; // degrees about x, y, z
  double s[3] = {1.0, 1.0, 1.0};
};

// T Rz Ry Rx S, as a view entry's transform: builds it.
Mat trs(const Parts &p) {
  auto x = vtkSmartPointer<vtkTransform>::New();
  x->Translate(p.t);
  x->RotateZ(p.r[2]);
  x->RotateY(p.r[1]);
  x->RotateX(p.r[0]);
  x->Scale(p.s);
  Mat m;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      m[i * 4 + j] = x->GetMatrix()->GetElement(i, j);
  return m;
}

Mat of(const vtkMatrix4x4 *a) {
  Mat m;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      m[i * 4 + j] = a->GetElement(i, j);
  return m;
}

bool near(const Mat &a, const Mat &b, double tol) {
  for (int k = 0; k < 16; ++k)
    if (!(std::fabs(a[k] - b[k]) <= tol))
      return false;
  return true;
}

// Rotations and scales a document or a host gives a node, each in a float (a SceneNode's).
const Parts kCases[] = {
    {{1, 2, 3}, {0, 0, 30}, {4, 1, 0.5}},
    {{-2, 0.5, 7}, {10, 20, 30}, {2, 0.5, 3}},
    {{0, 0, 0}, {-45, 60, 135}, {1.5, 1.5, 1.5}},
    {{3, -1, 0}, {0, 90, 0}, {1, 2, 4}},
};

std::shared_ptr<NullGraphicNode> node(SceneGraph &sg, const std::string &name) {
  return sg.getGraphicsRoot()->addGraphicsChild<NullGraphicNode>(name);
}

void test_matches_the_scene_and_view_matrices() {
  std::printf("test_matches_the_scene_and_view_matrices\n");
  cvc::app app;
  SceneGraph sg(app, "xf_match");
  int k = 0;
  for (const Parts &p : kCases) {
    cvc::ariadne::SceneNode sn;
    sn.has_transform = true;
    for (int i = 0; i < 3; ++i) {
      sn.position[i] = static_cast<float>(p.t[i]);
      sn.rotation[i] = static_cast<float>(p.r[i]);
      sn.scale[i] = static_cast<float>(p.s[i]);
    }
    Mat scene;
    cvc::ariadne::scene_node_local_matrix(sn, scene.data());
    CHECK(near(scene, trs(p), 1e-12));

    auto a = node(sg, "a" + std::to_string(k)); // as realize_scene sets it
    a->setPosition(p.t[0], p.t[1], p.t[2]);
    a->setRotation(p.r[0], p.r[1], p.r[2]);
    a->setScale(p.s[0], p.s[1], p.s[2]);
    CHECK(near(of(a->getTransform()), scene, 1e-12));

    auto b = node(sg, "b" + std::to_string(k)); // scale first
    b->setScale(p.s[0], p.s[1], p.s[2]);
    b->setRotation(p.r[0], p.r[1], p.r[2]);
    b->setPosition(p.t[0], p.t[1], p.t[2]);
    CHECK(of(b->getTransform()) == of(a->getTransform()));
    ++k;
  }
}

void test_realize_scene_places_a_node_so() {
  std::printf("test_realize_scene_places_a_node_so\n");
  cvc::app app;
  SceneGraph sg(app, "xf_realize");
  cvc::ariadne::Scene scene;
  cvc::ariadne::SceneNode sn;
  sn.id = "beam";
  sn.type = "group";
  sn.has_transform = true;
  sn.position[2] = 5.0f;
  sn.rotation[2] = 30.0f;
  sn.scale[0] = 4.0f;
  sn.scale[2] = 0.5f;
  scene.nodes.push_back(sn);
  cvc::gl::ariadne::realize_scene(sg, scene, "");
  sg.processEvents();
  std::shared_ptr<GraphicsNode> g = sg.getGraphics("beam");
  CHECK(g != nullptr);
  if (!g)
    return;
  Parts p;
  p.t[2] = 5.0;
  p.r[2] = 30.0;
  p.s[0] = 4.0;
  p.s[2] = 0.5;
  CHECK(near(of(g->getTransform()), trs(p), 1e-12));
  // Its own x, stretched 4 times, turned 30 degrees: (4 cos 30, 4 sin 30, 0).
  const Mat m = of(g->getTransform());
  CHECK(std::fabs(m[0] - 4.0 * std::cos(30.0 * 3.141592653589793 / 180.0)) < 1e-12);
  CHECK(std::fabs(m[4] - 4.0 * std::sin(30.0 * 3.141592653589793 / 180.0)) < 1e-12);
}

void test_rotation_and_scale_keep_each_other() {
  std::printf("test_rotation_and_scale_keep_each_other\n");
  cvc::app app;
  SceneGraph sg(app, "xf_keep");
  auto n = node(sg, "n");
  n->setPosition(1, 2, 3);
  n->setScale(2, 3, 4);
  n->setRotation(0, 0, 90);
  Parts p;
  p.t[0] = 1;
  p.t[1] = 2;
  p.t[2] = 3;
  p.r[2] = 90;
  p.s[0] = 2;
  p.s[1] = 3;
  p.s[2] = 4;
  CHECK(near(of(n->getTransform()), trs(p), 1e-12)); // the rotation kept the scale
  n->setRotation(0, 0, 30);
  p.r[2] = 30;
  CHECK(near(of(n->getTransform()), trs(p), 1e-12)); // and again

  // A scale of 0 (a node shrunk out of sight) and back keeps the rotation.
  n->setScale(0, 0, 0);
  n->setScale(1, 1, 1);
  p.s[0] = p.s[1] = p.s[2] = 1;
  CHECK(near(of(n->getTransform()), trs(p), 1e-12));
  n->setScale(0, 1, 1);
  n->setRotation(0, 0, 45);
  n->setScale(1, 1, 1);
  p.r[2] = 45;
  CHECK(near(of(n->getTransform()), trs(p), 1e-12));

  // A mirror stays on the axis it was given through a rotation, and comes off with the scale.
  n->setScale(-1, 1, 1);
  n->setRotation(0, 0, 60);
  p.r[2] = 60;
  p.s[0] = -1;
  CHECK(near(of(n->getTransform()), trs(p), 1e-12));
  n->setScale(1, 1, 1);
  p.s[0] = 1;
  CHECK(near(of(n->getTransform()), trs(p), 1e-12));
}

void test_a_whole_matrix_keeps_its_rotation_and_scale() {
  std::printf("test_a_whole_matrix_keeps_its_rotation_and_scale\n");
  cvc::app app;
  SceneGraph sg(app, "xf_whole");
  auto n = node(sg, "n");
  Parts p;
  p.t[0] = 7;
  p.t[1] = -8;
  p.t[2] = 9;
  p.r[0] = 15;
  p.r[1] = -25;
  p.r[2] = 40;
  p.s[0] = 2;
  p.s[1] = 3;
  p.s[2] = 0.5;
  const Mat m = trs(p);

  n->setTransform(m.data());
  n->setScale(1, 1, 1); // keeps its rotation
  Parts q = p;
  q.s[0] = q.s[1] = q.s[2] = 1;
  CHECK(near(of(n->getTransform()), trs(q), 1e-9));

  n->setTransform(m.data());
  n->setRotation(0, 0, 0); // keeps its scale
  q = p;
  q.r[0] = q.r[1] = q.r[2] = 0;
  CHECK(near(of(n->getTransform()), trs(q), 1e-9));

  n->setPoseMatrix(m.data()); // the per-frame pose path, on the owner thread
  n->setRotation(0, 0, 0);
  CHECK(near(of(n->getTransform()), trs(q), 1e-9));

  // The "matrix" key, written from outside.
  std::string text;
  for (int k = 0; k < 16; ++k)
    text += (k ? "," : "") + std::to_string(m[k]);
  cvc::state::instance(app)(n->getState("matrix").fullName()).value(text);
  sg.processEvents();
  n->setRotation(0, 0, 0);
  CHECK(near(of(n->getTransform()), trs(q), 1e-6)); // to_string's six decimals

  // Pitched straight up: the turns about z and x are one, and the matrix is kept.
  Parts up;
  up.r[0] = 10;
  up.r[1] = 90;
  up.r[2] = 30;
  up.s[0] = up.s[1] = up.s[2] = 2;
  n->setTransform(trs(up).data());
  n->setScale(1, 1, 1);
  Parts upq = up;
  upq.s[0] = upq.s[1] = upq.s[2] = 1;
  CHECK(near(of(n->getTransform()), trs(upq), 1e-9));

  // A mirrored matrix: the mirror is kept on z through a rotation.
  Parts mir;
  mir.r[2] = 20;
  mir.s[2] = -2;
  n->setTransform(trs(mir).data());
  n->setRotation(0, 0, 0);
  Parts mirq;
  mirq.s[2] = -2;
  CHECK(near(of(n->getTransform()), trs(mirq), 1e-9));
}

void test_the_keys_do_the_same() {
  std::printf("test_the_keys_do_the_same\n");
  cvc::app app;
  SceneGraph sg(app, "xf_keys");
  auto a = node(sg, "a");
  auto b = node(sg, "b");
  a->setScale(2, 1, 0.5);
  a->setRotation(0, 0, 30);
  cvc::state &st = cvc::state::instance(app);
  st(b->getState("scale").fullName()).value(std::string("2,1,0.5"));
  sg.processEvents();
  st(b->getState("rotation").fullName()).value(std::string("0,0,30"));
  sg.processEvents();
  CHECK(near(of(b->getTransform()), of(a->getTransform()), 1e-12));
}

void test_reset_starts_over() {
  std::printf("test_reset_starts_over\n");
  cvc::app app;
  SceneGraph sg(app, "xf_reset");
  auto n = node(sg, "n");
  n->setScale(3, 3, 3);
  n->setRotation(10, 20, 30);
  n->resetTransform();
  n->setRotation(0, 0, 90);
  Parts p;
  p.r[2] = 90;
  CHECK(near(of(n->getTransform()), trs(p), 1e-12));
}

} // namespace

int main() {
  test_matches_the_scene_and_view_matrices();
  test_realize_scene_places_a_node_so();
  test_rotation_and_scale_keep_each_other();
  test_a_whole_matrix_keeps_its_rotation_and_scale();
  test_the_keys_do_the_same();
  test_reset_starts_over();

  if (g_failures) {
    std::printf("cvcgl_node_transform: %d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_node_transform: OK\n");
  return 0;
}
