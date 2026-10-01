/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvcgl_streaming_races -- interleavings a plain test cannot force, forced at
// chosen lock acquisitions. This binary interposes pthread_mutex_lock (Linux):
// an ARMED thread pauses when it is about to lock a mutex that lives inside a
// chosen object, until another thread lets it go. Headless; no GL.
//
//   * teardown: a producer that already holds the scene's event sink is paused
//     right before it locks the sink to post, the owner destroys the scene, then
//     the producer posts. With a raw SceneGraph* that post locked freed memory;
//     now it is refused, and the staged write is applied at the next attach.
//   * link bounds, owner first: the owner (beforeComputeBounds) is paused
//     mid-derive while a producer's setStyle runs. The producer's newer box must
//     survive -- not be overwritten by the owner's older one.
//   * link bounds, producer first: a producer's setStyle has derived its box from
//     the OLD heights and is paused before committing it; the heights change and
//     the owner re-derives. The final box must reflect the new heights.
//
// The two bounds cases bound their pause (kGrace): with the fix the paused
// thread holds the lock the other one needs, and the grace period is what lets
// it go on. Without the fix there is no such lock and the interleaving happens
// exactly as described -- the checks below then fail.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/gl/DrapedLinkNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/SceneEventSink.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StreamingGeometryNode.h>
#include <dlfcn.h>
#include <memory>
#include <pthread.h>
#include <string>
#include <thread>
#include <vector>
#include <vtkProp.h>

namespace {

int g_checks = 0, g_failures = 0;

bool check(bool ok, const std::string &what, const std::string &detail = "") {
  ++g_checks;
  if (!ok)
    ++g_failures;
  std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(), detail.empty() ? "" : " -- ",
              detail.c_str());
  std::fflush(stdout);
  return ok;
}

constexpr std::chrono::milliseconds kGrace(300);

// Where an armed thread pauses: at its first lock of a mutex inside [lo, hi)
// -- once it has locked one inside [afterLo, afterHi), when that is set.
struct Gate {
  std::atomic<const char *> lo{nullptr}, hi{nullptr};
  std::atomic<const char *> afterLo{nullptr}, afterHi{nullptr};
  std::atomic<bool> reached{false};
  std::atomic<bool> release{false};
  std::atomic<long> waitMs{0}; // 0: until released

  template <typename T> void target(const T *obj) {
    lo = reinterpret_cast<const char *>(obj);
    hi = reinterpret_cast<const char *>(obj) + sizeof(T);
  }
  template <typename T> void after(const T *obj) {
    afterLo = reinterpret_cast<const char *>(obj);
    afterHi = reinterpret_cast<const char *>(obj) + sizeof(T);
  }
  void reset() {
    lo = hi = afterLo = afterHi = nullptr;
    reached = false;
    release = false;
    waitMs = 0;
  }
};
Gate g_gate;
thread_local bool t_armed = false;
thread_local bool t_seenAfter = false;

void arm() {
  t_seenAfter = g_gate.afterLo.load() == nullptr;
  t_armed = true;
}

bool waitFlag(const std::atomic<bool> &flag, long ms) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
  while (!flag.load()) {
    if (ms > 0 && std::chrono::steady_clock::now() > end)
      return false;
    std::this_thread::yield();
  }
  return true;
}

bool inside(const char *p, const std::atomic<const char *> &lo,
            const std::atomic<const char *> &hi) {
  const char *l = lo.load(), *h = hi.load();
  return l && p >= l && p < h;
}

} // namespace

extern "C" int pthread_mutex_lock(pthread_mutex_t *m) {
  using fn_t = int (*)(pthread_mutex_t *);
  static fn_t real = reinterpret_cast<fn_t>(dlsym(RTLD_NEXT, "pthread_mutex_lock"));
  if (t_armed) {
    const char *p = reinterpret_cast<const char *>(m);
    if (!t_seenAfter && inside(p, g_gate.afterLo, g_gate.afterHi)) {
      t_seenAfter = true;
    } else if (t_seenAfter && inside(p, g_gate.lo, g_gate.hi)) {
      t_armed = false; // pause once
      g_gate.reached = true;
      waitFlag(g_gate.release, g_gate.waitMs.load());
    }
  }
  return real(m);
}

using cvc::gl::DrapedLinkNode;
using cvc::gl::HeightFieldTexture;
using cvc::gl::SceneEventSink;
using cvc::gl::SceneGraph;
using cvc::gl::StreamingGeometryNode;
using cvc::gl::StreamingLayout;
using cvc::gl::StreamingMapperKind;

namespace {

// ── a producer posting while the owner destroys the scene ──────────────────
void testTeardownWhilePosting(cvc::app &app) {
  std::printf("teardown: a producer posts while the owner destroys the scene\n");
  StreamingLayout l;
  l.capacity_points = 8;
  l.triangles = {{{0, 1, 2}}};
  l.reserved_bounds = cvc::bounding_box(-1, -1, -1, 1, 1, 1);
  l.mapper = StreamingMapperKind::Classic;
  auto node = std::make_shared<StreamingGeometryNode>(app, "races_teardown.n", "n", l);
  auto sg = std::make_unique<SceneGraph>(app, "races_teardown");
  sg->getGraphicsRoot()->addGraphicsChild(node);
  const std::uint64_t applies0 = node->streamStats().applies;

  g_gate.reset();
  g_gate.target(sg->eventSink().get()); // address only: hold no reference here
  std::atomic<bool> done{false};
  std::thread producer([&]() {
    arm();
    const float p[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    node->writePoints(0, p, 3); // pauses just before locking the sink to post
    done = true;
  });
  const bool reached = waitFlag(g_gate.reached, 5000);
  check(reached, "the producer reached the post with the scene still alive");
  sg.reset(); // the owner destroys the scene under the paused producer
  std::vector<char> scribble(4096, '\xAB'); // reuse what the scene freed
  g_gate.release = true;
  producer.join();
  check(done.load() && node->streamStats().applies == applies0,
        "the post was refused, nothing applied, nothing touched freed memory");
  SceneGraph sg2(app, "races_teardown2");
  sg2.getGraphicsRoot()->addGraphicsChild(node);
  check(node->streamStats().applies == applies0 + 1,
        "the staged write is applied when the node is attached again");
}

// ── draped link bounds ──────────────────────────────────────────────────────
struct LinkRig {
  SceneGraph sg;
  std::shared_ptr<HeightFieldTexture> hf;
  std::shared_ptr<DrapedLinkNode> link;
  LinkRig(cvc::app &app, const char *name)
      : sg(app, name), hf(std::make_shared<HeightFieldTexture>(4, 4, 0.0, 0.0, 1.0, 1.0)) {
    link = sg.getGraphicsRoot()->addGraphicsChild<DrapedLinkNode>("l", hf, 8,
                                                                  StreamingMapperKind::Classic);
    sg.processEvents();
  }
  double vtkMaxZ() { return link->prop()->GetBounds()[5]; }
};

void testOwnerDeriveVsStyle(cvc::app &app) {
  std::printf("link bounds: the owner re-derives while a producer restyles\n");
  LinkRig r(app, "races_owner_first");
  r.hf->setHeights(std::vector<float>(16, 2.0f)); // the next GetBounds re-derives
  g_gate.reset();
  g_gate.target(r.hf.get()); // the owner pauses where it reads the extent
  g_gate.waitMs = kGrace.count();
  std::thread producer([&]() {
    waitFlag(g_gate.reached, 5000);
    r.link->setStyle(1.0f, 50.0f, 1, 0, 0, 1.0f); // lift 0.5 -> 50
    g_gate.release = true;
  });
  arm();
  r.vtkMaxZ(); // ComputeBounds -> beforeComputeBounds, paused mid-derive
  t_armed = false;
  producer.join();
  r.sg.processEvents(); // the producer's posted apply
  const double want = 2.0 + 50.0 + 1.0;
  check(r.link->reservedBounds().maxz == want && r.vtkMaxZ() == want,
        "the producer's newer box survives the owner's older derive",
        "maxz " + std::to_string(r.link->reservedBounds().maxz) + ", VTK " +
            std::to_string(r.vtkMaxZ()) + ", want " + std::to_string(want));
}

void testStyleVsNewHeights(cvc::app &app) {
  std::printf("link bounds: a restyle derived from old heights races new heights\n");
  LinkRig r(app, "races_producer_first");
  r.hf->setHeights(std::vector<float>(16, 2.0f));
  r.vtkMaxZ(); // derive at these heights
  g_gate.reset();
  g_gate.after(r.hf.get());    // once the producer has read the extent ...
  g_gate.target(r.link.get()); // ... pause at its next lock in the link: the commit
  g_gate.waitMs = kGrace.count();
  std::thread producer([&]() {
    arm();
    r.link->setStyle(1.0f, 3.0f, 1, 0, 0, 1.0f); // derives from heights 2
  });
  waitFlag(g_gate.reached, 5000);
  r.hf->setHeights(std::vector<float>(16, 100.0f)); // terrain changes meanwhile
  r.vtkMaxZ();                                      // the owner re-derives
  g_gate.release = true;
  producer.join();
  r.sg.processEvents();
  const double want = 100.0 + 3.0 + 1.0;
  check(r.vtkMaxZ() == want && r.link->reservedBounds().maxz == want,
        "the box ends up derived from the new heights (no stale box marked current)",
        "VTK maxz " + std::to_string(r.vtkMaxZ()) + ", want " + std::to_string(want));
}

} // namespace

int main() {
  cvc::app app;
  testTeardownWhilePosting(app);
  testOwnerDeriveVsStyle(app);
  testStyleVsNewHeights(app);
  std::printf("%s: cvcgl_streaming_races (%d checks, %d failed)\n",
              g_failures == 0 ? "PASS" : "FAIL", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
