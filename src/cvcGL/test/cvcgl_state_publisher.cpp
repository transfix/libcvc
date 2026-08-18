// The publisher must keep the state tree correct, not merely fast.
//
// It trades immediacy for throughput: a write is visible after the next flush,
// not instantly. These pin the parts that must still hold — the value DOES
// arrive, the last value wins, and an external write still reaches the node.
#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/context.h>
#include <cvc/gl/state_publisher.h>
#include <string>
#include <thread>

namespace {

cvc::geometry dot() {
  cvc::geometry g;
  cvc::geometry::point_t p;
  p[0] = p[1] = p[2] = 0;
  g.points().push_back(p);
  return g;
}

std::string read(const std::string &path) {
  return cvc::state::instance(cvc::gl::context())(path).value();
}

// A posed node's position must reach the state tree.
void test_position_reaches_state() {
  SceneGraph sg;
  auto n = sg.addGraphics("pub1", dot());
  n->setPosition(3.0, 4.0, 5.0);
  cvc::gl::state_publisher::instance().flush();
  const std::string v = read(n->getState("position").fullName());
  assert(v == "3,4,5");
  std::printf("  ok: a posed node's position reaches the state tree (%s)\n", v.c_str());
}

// Animation: many writes between flushes collapse to the last one, and the
// value stored is the LAST, not some earlier frame.
void test_coalescing_keeps_the_last_value() {
  SceneGraph sg;
  auto n = sg.addGraphics("pub2", dot());
  auto &pub = cvc::gl::state_publisher::instance();
  pub.flush();
  const std::uint64_t before = pub.coalesced();
  for (int i = 0; i < 200; ++i)
    n->setPosition(static_cast<double>(i), 0.0, 0.0);
  pub.flush();
  assert(pub.coalesced() > before); // writes really were superseded
  const std::string v = read(n->getState("position").fullName());
  assert(v == "199,0,0");
  std::printf("  ok: 200 poses coalesced to the last value (%s)\n", v.c_str());
}

// The echo guard must not deafen the node to EXTERNAL writes — the dashboard
// and scripts drive nodes this way.
void test_external_write_still_moves_the_node() {
  SceneGraph sg;
  auto n = sg.addGraphics("pub3", dot());
  n->setPosition(1.0, 1.0, 1.0);
  cvc::gl::state_publisher::instance().flush();

  cvc::state::instance(cvc::gl::context())(n->getState("position").fullName())
      .value(std::string("9,8,7"));
  sg.processEvents(); // node handlers marshal to the owner thread
  auto w = n->getWorldTransform();
  assert(std::abs(w->GetElement(0, 3) - 9.0) < 1e-9);
  assert(std::abs(w->GetElement(1, 3) - 8.0) < 1e-9);
  std::printf("  ok: an external state write still moves the node\n");
}

// The background flusher drains without anyone calling flush().
void test_background_flush() {
  SceneGraph sg;
  auto n = sg.addGraphics("pub4", dot());
  auto &pub = cvc::gl::state_publisher::instance();
  assert(pub.running());
  n->setPosition(2.5, 0.0, 0.0);
  const std::string path = n->getState("position").fullName();
  for (int i = 0; i < 100 && read(path) != "2.5,0,0"; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  assert(read(path) == "2.5,0,0");
  std::printf("  ok: the pooled worker drains the queue on its own\n");
}

// Back pressure: past the cap the queue must stop growing, and shedding must
// not starve anyone — every path has to land EVENTUALLY.
void test_back_pressure_sheds_without_starving() {
  auto &pub = cvc::gl::state_publisher::instance();
  pub.flush();
  const std::size_t cap = 64;
  const std::size_t paths = 500; // deliberately far past the cap
  pub.set_max_pending(cap);

  const std::uint64_t dropped0 = pub.dropped();
  for (std::size_t i = 0; i < paths; ++i)
    pub.publish("bp.p" + std::to_string(i), "0");
  // The whole point: bounded, not merely coalesced.
  assert(pub.pending() <= cap);
  assert(pub.dropped() > dropped0);
  std::printf("  ok: %zu offers held to a %zu cap (%llu shed)\n", paths, cap,
              static_cast<unsigned long long>(pub.dropped() - dropped0));

  // Eventual consistency: republish every round, as an animated node does, and
  // every path must eventually be stored. Starvation would leave some never set.
  for (int round = 0; round < 400; ++round) {
    for (std::size_t i = 0; i < paths; ++i)
      pub.publish("bp.p" + std::to_string(i), std::to_string(round));
    pub.flush();
  }
  std::size_t landed = 0;
  for (std::size_t i = 0; i < paths; ++i)
    if (!read("bp.p" + std::to_string(i)).empty())
      ++landed;
  assert(landed == paths);
  std::printf("  ok: all %zu paths landed despite shedding (no starvation)\n", landed);
  pub.set_max_pending(8192); // restore
}

} // namespace

int main() {
  test_position_reaches_state();
  test_coalescing_keeps_the_last_value();
  test_external_write_still_moves_the_node();
  test_background_flush();
  test_back_pressure_sheds_without_starving();
  std::printf("cvcgl_state_publisher: OK\n");
  return 0;
}
