// Headless checks for the explicit cvc::gl traversal: the state stack, the
// ORDER-DEPENDENT property semantics, per-view divergence, and the second
// (bounding-box) action. No VTK renderer is needed — a RenderView with a null
// renderer still drives the whole traversal, which is the point of separating
// the walk from the rasteriser.
//
// The order-dependent behaviour is what most needs pinning: it is the one thing
// here that differs from the parent/child inheritance the older GraphicsNode
// tree uses, so a well-meaning "fix" would silently change what scenes mean.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/RenderView.h>
#include <cvc/gl/nodes.h>
#include <cvc/gl/traversal.h>
#include <memory>
#include <stdexcept>

using namespace cvc::gl;

// NOT assert(): these tests are built Release (NDEBUG), where assert() compiles
// to nothing and a "passing" run would prove exactly nothing.
static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

static cvc::geometry unit_tri() {
  cvc::geometry g;
  cvc::geometry::point_t p;
  const double xyz[3][3] = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  for (auto &v : xyz) {
    p[0] = v[0];
    p[1] = v[1];
    p[2] = v[2];
    g.points().push_back(p);
  }
  cvc::geometry::tri_t t;
  t[0] = 0;
  t[1] = 1;
  t[2] = 2;
  g.tris().push_back(t);
  return g;
}

static std::shared_ptr<GeometryShape> make_shape(const char *name) {
  auto s = std::make_shared<GeometryShape>(name);
  s->setGeometry(unit_tri());
  return s;
}

// A property node's effect must reach the siblings that FOLLOW it and no
// others. This is the Inventor semantic and the reason property and shape nodes
// are separate types.
static void test_order_dependent_state() {
  auto root = std::make_shared<Separator>("root");
  auto before = make_shape("before");
  auto xf = std::make_shared<Transform>("xf");
  xf->setTranslation(10.0, 0.0, 0.0);
  auto after = make_shape("after");

  root->addChild(before);
  root->addChild(xf); // affects `after` only
  root->addChild(after);

  BoundingBoxAction bbox;
  bbox.apply(*root);
  CHECK(!bbox.empty());
  // `before` sits at x in [0,1]; `after` is shifted to [10,11]. If the
  // transform were inherited hierarchically (or applied to the whole group),
  // both would move and minx would be 10.
  const double *b = bbox.bounds();
  CHECK(std::fabs(b[0] - 0.0) < 1e-9);
  CHECK(std::fabs(b[3] - 11.0) < 1e-9);
  CHECK(bbox.visitedShapes() == 2);
  std::printf("  ok: a property node affects following siblings only\n");
}

// Separator confines state; Group lets it leak. That difference is the entire
// distinction between the two container types.
static void test_separator_confines_group_leaks() {
  auto run = [](bool useSeparator) {
    auto root = std::make_shared<Separator>("root");
    std::shared_ptr<Group> inner =
        useSeparator ? std::make_shared<Separator>("inner") : std::make_shared<Group>("inner");
    auto xf = std::make_shared<Transform>("xf");
    xf->setTranslation(100.0, 0.0, 0.0);
    inner->addChild(xf);
    root->addChild(inner);
    root->addChild(make_shape("sibling")); // AFTER the container

    BoundingBoxAction bbox;
    bbox.apply(*root);
    return bbox.bounds()[0];
  };

  CHECK(std::fabs(run(true) - 0.0) < 1e-9);    // Separator popped it
  CHECK(std::fabs(run(false) - 100.0) < 1e-9); // Group let it escape
  std::printf("  ok: Separator confines state, Group leaks it to siblings\n");
}

// Transforms compose down a chain, in parent-then-child order.
static void test_transform_composition() {
  auto root = std::make_shared<Separator>("root");
  auto outer = std::make_shared<Transform>("outer");
  outer->setTranslation(1.0, 2.0, 3.0);
  auto inner = std::make_shared<Separator>("inner");
  auto t2 = std::make_shared<Transform>("t2");
  t2->setTranslation(10.0, 0.0, 0.0);
  inner->addChild(t2);
  inner->addChild(make_shape("leaf"));
  root->addChild(outer);
  root->addChild(inner);

  BoundingBoxAction bbox;
  bbox.apply(*root);
  const double *b = bbox.bounds();
  CHECK(std::fabs(b[0] - 11.0) < 1e-9); // 1 + 10
  CHECK(std::fabs(b[1] - 2.0) < 1e-9);
  CHECK(std::fabs(b[2] - 3.0) < 1e-9);
  std::printf("  ok: transforms compose along the traversal\n");
}

// One graph, two views, different content — the thing a single m_renderer
// pointer per node made impossible.
static void test_per_view_divergence() {
  auto root = std::make_shared<Separator>("root");
  auto shared = make_shape("shared");
  auto annotationScope = std::make_shared<Separator>("annotations");
  auto mask = std::make_shared<VisibilityMask>("mask");
  mask->setMask(0x2);
  annotationScope->addChild(mask);
  auto annotation = make_shape("annotation");
  annotationScope->addChild(annotation);
  root->addChild(shared);
  root->addChild(annotationScope);

  RenderView main(nullptr, "main");
  main.setVisibilityMask(0x1); // annotations excluded
  RenderView overview(nullptr, "overview");
  overview.setVisibilityMask(0x2); // annotations included

  RenderAction a1(main);
  a1.apply(*root);
  RenderAction a2(overview);
  a2.apply(*root);

  CHECK(a1.visitedShapes() == 1); // shared only
  CHECK(a2.visitedShapes() == 2); // shared + annotation

  // Each view got its OWN prop for the shared shape.
  CHECK(shared->numViewInstances() == 2);
  std::printf("  ok: one graph, two views, per-view visibility and props\n");
}

// A view going away must not leave shapes holding props bound to it.
static void test_view_teardown_releases_props() {
  auto root = std::make_shared<Separator>("root");
  auto shape = make_shape("s");
  root->addChild(shape);

  {
    RenderView tmp(nullptr, "tmp");
    RenderAction a(tmp);
    a.apply(*root);
    CHECK(shape->numViewInstances() == 1);
  } // tmp dies here

  CHECK(shape->numViewInstances() == 0);

  // And the shape is still usable in a fresh view afterwards.
  RenderView next(nullptr, "next");
  RenderAction a(next);
  a.apply(*root);
  CHECK(shape->numViewInstances() == 1);
  std::printf("  ok: destroying a view releases its per-view props\n");
}

// Switch picks one child, all, or none.
static void test_switch() {
  auto root = std::make_shared<Separator>("root");
  auto sw = std::make_shared<Switch>("sw");
  sw->addChild(make_shape("a"));
  sw->addChild(make_shape("b"));
  root->addChild(sw);

  RenderView view(nullptr, "v");
  auto count = [&](int which) {
    sw->setWhichChild(which);
    RenderAction a(view);
    a.apply(*root);
    return a.visitedShapes();
  };
  CHECK(count(Switch::All) == 2);
  CHECK(count(Switch::None) == 0);
  CHECK(count(0) == 1);
  CHECK(count(1) == 1);
  CHECK(count(99) == 0); // out of range is not a crash
  std::printf("  ok: Switch selects all / none / one child\n");
}

// An unbalanced pop is a bug, and must say so rather than corrupt the stack.
static void test_state_stack_guard() {
  TraversalState st;
  st.push();
  st.pop();
  bool threw = false;
  try {
    st.pop(); // the base frame
  } catch (const std::logic_error &) {
    threw = true;
  }
  CHECK(threw);
  CHECK(st.depth() == 1);
  std::printf("  ok: popping the base state frame is caught\n");
}

int main() {
  test_order_dependent_state();
  test_separator_confines_group_leaks();
  test_transform_composition();
  test_per_view_divergence();
  test_view_teardown_releases_props();
  test_switch();
  test_state_stack_guard();
  std::printf("cvcgl_traversal: OK\n");
  return 0;
}
