// The publisher must keep the state tree correct, not merely fast.
//
// It trades immediacy for throughput: a write is visible after the next flush,
// not instantly. These pin the parts that must still hold — the value DOES
// arrive, the last value wins, and an external write still reaches the node.
//
// The publisher is scene-owned (SceneGraph::publisher()), running under the
// scene's app — there is no process-wide instance().
#undef NDEBUG
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/state_publisher.h>
#include <cvc/state/state.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;

namespace {

cvc::geometry dot() {
  cvc::geometry g;
  cvc::geometry::point_t p;
  p[0] = p[1] = p[2] = 0;
  g.points().push_back(p);
  return g;
}

std::string read(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

// A posed node's position must reach the state tree.
void test_position_reaches_state() {
  cvc::app app;
  SceneGraph sg(app);
  auto n = sg.addGraphics("pub1", dot());
  n->setPosition(3.0, 4.0, 5.0);
  sg.publisher().flush();
  const std::string v = read(app, n->getState("position").fullName());
  assert(v == "3,4,5");
  std::printf("  ok: a posed node's position reaches the state tree (%s)\n", v.c_str());
}

// Animation: many writes between flushes collapse to the last one, and the
// value stored is the LAST, not some earlier frame.
void test_coalescing_keeps_the_last_value() {
  cvc::app app;
  SceneGraph sg(app);
  auto n = sg.addGraphics("pub2", dot());
  auto &pub = sg.publisher();
  pub.flush();
  const std::uint64_t before = pub.coalesced();
  for (int i = 0; i < 200; ++i)
    n->setPosition(static_cast<double>(i), 0.0, 0.0);
  pub.flush();
  assert(pub.coalesced() > before); // writes really were superseded
  const std::string v = read(app, n->getState("position").fullName());
  assert(v == "199,0,0");
  std::printf("  ok: 200 poses coalesced to the last value (%s)\n", v.c_str());
}

// The echo guard must not deafen the node to EXTERNAL writes — the dashboard
// and scripts drive nodes this way.
void test_external_write_still_moves_the_node() {
  cvc::app app;
  SceneGraph sg(app);
  auto n = sg.addGraphics("pub3", dot());
  n->setPosition(1.0, 1.0, 1.0);
  sg.publisher().flush();

  cvc::state::instance(app)(n->getState("position").fullName()).value(std::string("9,8,7"));
  sg.processEvents(); // node handlers marshal to the owner thread
  auto w = n->getWorldTransform();
  assert(std::abs(w->GetElement(0, 3) - 9.0) < 1e-9);
  assert(std::abs(w->GetElement(1, 3) - 8.0) < 1e-9);
  std::printf("  ok: an external state write still moves the node\n");
}

// The background flusher drains without anyone calling flush().
void test_background_flush() {
  cvc::app app;
  SceneGraph sg(app);
  auto n = sg.addGraphics("pub4", dot());
  auto &pub = sg.publisher();
  assert(pub.running()); // the scene starts its publisher
  n->setPosition(2.5, 0.0, 0.0);
  const std::string path = n->getState("position").fullName();
  for (int i = 0; i < 100 && read(app, path) != "2.5,0,0"; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  assert(read(app, path) == "2.5,0,0");
  std::printf("  ok: the pooled worker drains the queue on its own\n");
}

// Back pressure: past the cap the queue must stop growing, and shedding must
// not starve anyone — every path has to land EVENTUALLY. Exercised directly on
// a standalone publisher (no scene needed) under its own app.
void test_back_pressure_sheds_without_starving() {
  cvc::app app;
  cvc::gl::state_publisher pub(app);
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
    if (!read(app, "bp.p" + std::to_string(i)).empty())
      ++landed;
  assert(landed == paths);
  std::printf("  ok: all %zu paths landed despite shedding (no starvation)\n", landed);
}

// Two scenes under ONE app — the case the injected-app model exists for (split /
// comparison views). Each owns its own publisher; constructing the second must not
// interrupt or join the first's running worker (a shared pool key would deadlock
// the second scene's ctor on the first's std-CV-parked worker). Both publish and
// flush independently.
void test_two_scenes_one_app() {
  cvc::app app;
  SceneGraph a(app, "sceneA");
  SceneGraph b(app, "sceneB"); // must NOT hang on sceneA's worker
  assert(a.publisher().running());
  assert(b.publisher().running());
  auto na = a.addGraphics("n", dot());
  auto nb = b.addGraphics("n", dot());
  na->setPosition(1.0, 2.0, 3.0);
  nb->setPosition(4.0, 5.0, 6.0);
  a.publisher().flush();
  b.publisher().flush();
  assert(read(app, na->getState("position").fullName()) == "1,2,3");
  assert(read(app, nb->getState("position").fullName()) == "4,5,6");
  std::printf("  ok: two scenes share one app without colliding publishers\n");
}

// Rapid create/destroy while the worker is parked mid-drain: ~SceneGraph -> stop()
// must JOIN the worker before the publisher (and the nodes it would post to) are
// freed. Deterministic with the join; a use-after-free without it.
void test_teardown_joins_worker() {
  cvc::app app;
  for (int i = 0; i < 200; ++i) {
    SceneGraph sg(app, "churn");
    auto n = sg.addGraphics("n", dot());
    n->setPosition(static_cast<double>(i), 0.0, 0.0); // give the worker something to drain
    // destroyed here with the worker very likely still parked/mid-flush
  }
  std::printf("  ok: scene teardown joins its publisher worker (no UAF over churn)\n");
}

// A publisher's worker must NOT depend on a free app thread-pool slot. Shrink the
// pool to ONE, then run two scenes: a pooled long-lived worker would let the first
// scene pin the only slot forever, so the second scene's worker could never be
// scheduled and its teardown (a mandatory join) would hang. Dedicated per-publisher
// threads make the pool irrelevant.
void test_publisher_independent_of_pool() {
  cvc::app app;
  app.setThreadPoolSize(1);
  {
    SceneGraph a(app, "poolA");
    SceneGraph b(app, "poolB"); // must run + tear down despite the pool being size 1
    assert(a.publisher().running());
    assert(b.publisher().running());
    auto n = b.addGraphics("n", dot());
    n->setPosition(7.0, 0.0, 0.0);
    b.publisher().flush();
    assert(read(app, n->getState("position").fullName()) == "7,0,0");
    // both scenes tear down here; neither may hang on a pool slot
  }
  std::printf("  ok: publisher is independent of the app pool (size 1, two scenes)\n");
}

// Two flushers must not put an older value on the tree after a newer one.
//
// The worker is not the only flusher: a host can drain by hand from its frame
// loop while the worker runs (examples/cvcgl/bunny_shadow.cpp and
// volren_bunny.cpp both do). Taking the batch and writing it are two steps, so
// two flushers can interleave between them — one takes "5", the other takes "6"
// and writes it, then the first writes "5" over it. Coalescing cannot save that:
// the two values were never in the map at the same time. And nothing afterwards
// disagrees, so the tree stays on the superseded value for good.
//
// Checked as a MONOTONICITY invariant rather than by trying to hit the
// interleaving: the published counter only ever goes up, so the value on the
// tree must never go down. One inversion is one violation.
void test_concurrent_flushers_keep_write_order() {
  cvc::app app;
  cvc::gl::state_publisher pub(app); // no worker: this test provides both flushers
  const std::string path = "flushorder.counter";

  std::atomic<bool> stop{false};
  std::atomic<int> inversions{0};
  std::atomic<int> highest{0};

  // Two hand flushers, exactly the shape of "the worker plus a frame loop".
  auto flusher = [&] {
    while (!stop.load(std::memory_order_relaxed))
      pub.flush();
  };
  // A reader that only ever needs to see the value climb.
  auto watcher = [&] {
    while (!stop.load(std::memory_order_relaxed)) {
      const std::string v = read(app, path);
      if (v.empty())
        continue;
      const int seen = std::stoi(v);
      int prev = highest.load(std::memory_order_relaxed);
      while (seen < prev) { // went BACKWARDS: an older write landed last
        inversions.fetch_add(1, std::memory_order_relaxed);
        break;
      }
      while (seen > prev && !highest.compare_exchange_weak(prev, seen))
        ;
    }
  };

  // Publish a whole batch per round, not one path. The inversion window is the
  // gap between a flusher taking the batch and finishing its writes, so a fat
  // batch widens it from one state write to sixty-odd and turns a rare
  // interleaving into a reliable one. Only path 0 is watched; the rest are
  // there to make the window real.
  std::thread f1(flusher), f2(flusher), w(watcher);
  for (int round = 1; round <= 4000; ++round) {
    pub.publish(path, std::to_string(round));
    for (int k = 1; k < 64; ++k)
      pub.publish("flushorder.pad" + std::to_string(k), std::to_string(round));
  }
  stop.store(true);
  f1.join();
  f2.join();
  w.join();

  assert(inversions.load() == 0 && "a flush must not write an older value over a newer one");
  std::printf("  ok: two concurrent flushers never invert write order\n");
}

// publish_matrix defers the text: the queue holds 16 doubles and flush()
// writes format_matrix() of the LAST one, which parses back bit-for-bit. It
// shares the per-path slot with publish(), so mixing the two on one path still
// means last-writer-wins.
void test_publish_matrix_formats_at_flush() {
  cvc::app app;
  SceneGraph sg(app);
  auto &pub = sg.publisher();
  pub.stop(); // drive the flushes by hand
  const std::string path = "pubmat.node.matrix";

  const double a[16] = {1, 0, 0, 0.1, 0, 1, 0, 1.0 / 3.0, 0, 0, 1, -2.5e7, 0, 0, 0, 1};
  double b[16];
  for (int i = 0; i < 16; ++i)
    b[i] = a[i] * 0.7 + i;
  pub.publish_matrix(path, a);
  pub.publish_matrix(path, b); // coalesced: only b is formatted and written
  assert(pub.pending() == 1);
  pub.flush();
  const std::string v = read(app, path);
  assert(v == cvc::gl::state_publisher::format_matrix(b));
  std::istringstream iss(v);
  char comma;
  for (int i = 0; i < 16; ++i) {
    double x;
    if (i > 0)
      iss >> comma;
    iss >> x;
    assert(x == b[i] && "17 significant digits round-trip exactly");
  }

  pub.publish_matrix(path, a);
  pub.publish(path, "text wins"); // the later write owns the slot
  pub.flush();
  assert(read(app, path) == "text wins");
  pub.publish(path, "text first");
  pub.publish_matrix(path, a);
  pub.flush();
  assert(read(app, path) == cvc::gl::state_publisher::format_matrix(a));
  std::printf("  ok: publish_matrix formats the last matrix at flush, exactly\n");
}

// flushing() names the publisher whose flush the calling thread is in. Node
// echo guards compare it with their own scene's publisher, so a write by a
// different scene's publisher is not mistaken for their own. Nested flushes
// report the innermost publisher, then the outer one again.
void test_flushing_names_the_innermost_publisher() {
  using cvc::gl::state_publisher;
  cvc::app app;
  state_publisher outer(app), inner(app); // no workers: flushed by hand here
  assert(state_publisher::flushing(app) == nullptr && !state_publisher::in_flush(app));

  const std::string outerPath = "flushing.outer", innerPath = "flushing.inner";
  cvc::state::instance(app)(outerPath).value(std::string("0"));
  cvc::state::instance(app)(innerPath).value(std::string("0"));
  const state_publisher *seenOuter = nullptr, *seenInner = nullptr, *seenOuterAgain = nullptr;
  const state_publisher *seenOtherThread = &outer;
  boost::signals2::scoped_connection ci = cvc::state::instance(app)(innerPath).valueChanged.connect(
      [&]() { seenInner = state_publisher::flushing(app); });
  boost::signals2::scoped_connection co =
      cvc::state::instance(app)(outerPath).valueChanged.connect([&]() {
        seenOuter = state_publisher::flushing(app);
        // Per thread: another thread is not inside this flush.
        std::thread([&]() { seenOtherThread = state_publisher::flushing(app); }).join();
        inner.flush(); // a handler that drains a second publisher
        seenOuterAgain = state_publisher::flushing(app);
      });

  inner.publish(innerPath, "1");
  outer.publish(outerPath, "1");
  outer.flush();
  assert(seenOuter == &outer);
  assert(seenInner == &inner);
  assert(seenOuterAgain == &outer);
  assert(seenOtherThread == nullptr);
  assert(state_publisher::flushing(app) == nullptr && !state_publisher::in_flush(app));
  std::printf("  ok: flushing() names the innermost publisher, per thread\n");
}

// A publisher's flush is its own app's: a handler of another app, fired while it runs on the same
// thread, sees none of it, and two apps' publishers flushing on two threads at once each see only
// their own. In both orders, and the later app starts clean.
void test_flushing_is_per_app() {
  using cvc::gl::state_publisher;
  const auto nested = [](bool aFirst) {
    cvc::app a, b;
    state_publisher pa(a), pb(b);
    cvc::state::instance(a)("p.a").value(std::string("0"));
    cvc::state::instance(b)("p.b").value(std::string("0"));
    const state_publisher *bSeesOfA = &pa, *bSeesOfB = nullptr, *aSeesOfB = &pb;
    boost::signals2::scoped_connection cb =
        cvc::state::instance(b)("p.b").valueChanged.connect([&]() {
          bSeesOfA = state_publisher::flushing(a); // b's flush runs inside a's handler
          bSeesOfB = state_publisher::flushing(b);
        });
    boost::signals2::scoped_connection ca =
        cvc::state::instance(a)("p.a").valueChanged.connect([&]() {
          aSeesOfB = state_publisher::flushing(b);
          pb.flush();
        });
    pb.publish("p.b", "1");
    pa.publish("p.a", "1");
    if (aFirst) {
      pa.flush();
    } else {
      pb.flush(); // b alone first: a's state never changes, so its handler does not run
      pb.publish("p.b", "2");
      pa.flush();
    }
    assert(aSeesOfB == nullptr && "a's handler: b is not flushing");
    assert(bSeesOfA == &pa && "a is still flushing (the outer flush) when b's runs inside it");
    assert(bSeesOfB == &pb && "b's handler sees b's publisher");
    assert(!state_publisher::in_flush(a) && !state_publisher::in_flush(b));
  };
  nested(true);
  nested(false);

  // Two apps on two threads at once: each sees its own publisher only.
  std::atomic<int> wrong{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 4; ++t)
    threads.emplace_back([&wrong, t]() {
      cvc::app app;
      state_publisher pub(app);
      const std::string path = "p.t" + std::to_string(t);
      cvc::state::instance(app)(path).value(std::string("0"));
      boost::signals2::scoped_connection c =
          cvc::state::instance(app)(path).valueChanged.connect([&]() {
            if (state_publisher::flushing(app) != &pub)
              ++wrong;
          });
      for (int i = 1; i <= 50; ++i) {
        pub.publish(path, std::to_string(i));
        pub.flush();
      }
      if (state_publisher::in_flush(app))
        ++wrong;
    });
  for (std::thread &t : threads)
    t.join();
  assert(wrong.load() == 0);
  {
    cvc::app later; // made after the others are gone: no flush of theirs
    assert(state_publisher::flushing(later) == nullptr && !state_publisher::in_flush(later));
  }
  std::printf("  ok: flushing() is per app (nested both ways, on threads at once, a later app)\n");
}

} // namespace

int main() {
  test_position_reaches_state();
  test_coalescing_keeps_the_last_value();
  test_external_write_still_moves_the_node();
  test_background_flush();
  test_back_pressure_sheds_without_starving();
  test_two_scenes_one_app();
  test_teardown_joins_worker();
  test_publisher_independent_of_pool();
  test_concurrent_flushers_keep_write_order();
  test_publish_matrix_formats_at_flush();
  test_flushing_names_the_innermost_publisher();
  test_flushing_is_per_app();
  std::printf("cvcgl_state_publisher: OK\n");
  return 0;
}
