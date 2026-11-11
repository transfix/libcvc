// The ServerLink (src/ariadne_python/server_link.h), arisrvx's queue between its script thread and
// the relay, with no Python: what queued writes coalesce to, the order they land in, the per-step
// budget, subtree writes as (state-set-tree) makes them, the relay's events, and the startup
// states.

#include "server_link.h"

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/exceptions.h> // field
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using cvc::ariadne::python::ServerLink;
namespace se = cvc::state_exec;

ServerLink::Tree leaf(std::string text) {
  ServerLink::Tree t;
  t.text = std::move(text);
  return t;
}

ServerLink::Tree dict(std::vector<std::pair<std::string, ServerLink::Tree>> children) {
  ServerLink::Tree t;
  t.compound = true;
  t.children = std::move(children);
  return t;
}

ServerLink::Tree list(std::vector<ServerLink::Tree> items) {
  ServerLink::Tree t;
  t.compound = true;
  t.text = std::to_string(items.size());
  for (std::size_t i = 0; i < items.size(); ++i)
    t.children.emplace_back(std::to_string(i), std::move(items[i]));
  return t;
}

std::string value_at(cvc::app &app, const std::string &path) {
  const cvc::state::state_ptr n = app.root().findDescendantShared(path);
  return n ? n->value() : "<none>";
}

bool has(cvc::app &app, const std::string &path) {
  return static_cast<bool>(app.root().findDescendantShared(path));
}

// The order writes land in, as the relay's shard would journal them: each value change, by path.
struct Recorder {
  explicit Recorder(cvc::app &app, const std::string &root) {
    conn = app.root()(root).childChanged.connect(
        [this](const std::string &child) { seen.push_back(child); });
  }
  ~Recorder() { conn.disconnect(); }
  std::vector<std::string> seen;
  boost::signals2::connection conn;
};

} // namespace

TEST(ServerLinkTest, SetsLandOnApply) {
  cvc::app app;
  ServerLink link;
  link.set("svc.status", "up");
  EXPECT_FALSE(has(app, "svc.status")); // queued, not written
  EXPECT_EQ(link.pending(), 1u);
  const ServerLink::Applied a = link.apply(app.root(), 0);
  EXPECT_EQ(a.writes, 1u);
  EXPECT_EQ(a.left, 0u);
  EXPECT_EQ(a.bytes, std::string("svc.status").size() + 2);
  EXPECT_EQ(value_at(app, "svc.status"), "up");
}

TEST(ServerLinkTest, ASetReplacesTheQueuedOneAndMovesToTheBack) {
  cvc::app app;
  ServerLink link;
  link.set("svc.a", "1");
  link.set("svc.rev", "1");
  link.set("svc.a", "2");   // the field again: it now lands after rev...
  link.set("svc.rev", "2"); // ...and rev, written last, lands last
  EXPECT_EQ(link.pending(), 2u);
  // A budget of one write a step shows the order.
  link.apply(app.root(), 1);
  EXPECT_EQ(value_at(app, "svc.a"), "2");
  EXPECT_FALSE(has(app, "svc.rev"));
  link.apply(app.root(), 1);
  EXPECT_EQ(value_at(app, "svc.rev"), "2");
}

TEST(ServerLinkTest, TheBudgetCarriesTheRestToTheNextStepInOrder) {
  cvc::app app;
  ServerLink link;
  for (int i = 0; i < 10; ++i)
    link.set("svc.k" + std::to_string(i), "0123456789"); // 16 bytes each
  ServerLink::Applied a = link.apply(app.root(), 40);
  EXPECT_EQ(a.writes, 2u);
  EXPECT_EQ(a.left, 8u);
  EXPECT_TRUE(has(app, "svc.k1"));
  EXPECT_FALSE(has(app, "svc.k2"));
  a = link.apply(app.root(), 40);
  EXPECT_EQ(a.writes, 2u);
  EXPECT_TRUE(has(app, "svc.k3"));
  EXPECT_FALSE(has(app, "svc.k4"));
  a = link.apply(app.root(), 0); // no budget: the rest at once
  EXPECT_EQ(a.writes, 6u);
  EXPECT_EQ(a.left, 0u);
  EXPECT_EQ(value_at(app, "svc.k9"), "0123456789");
}

TEST(ServerLinkTest, AWriteLargerThanTheBudgetStillLands) {
  cvc::app app;
  ServerLink link;
  link.set("svc.big", std::string(1000, 'x'));
  link.set("svc.after", "1");
  ServerLink::Applied a = link.apply(app.root(), 64);
  EXPECT_EQ(a.writes, 1u); // the first write of a step always goes
  EXPECT_EQ(value_at(app, "svc.big").size(), 1000u);
  EXPECT_FALSE(has(app, "svc.after"));
  a = link.apply(app.root(), 64);
  EXPECT_EQ(a.writes, 1u);
  EXPECT_EQ(value_at(app, "svc.after"), "1");
}

TEST(ServerLinkTest, ATreeSupersedesWhatIsQueuedUnderIt) {
  cvc::app app;
  ServerLink link;
  link.set("svc.items.0.name", "stale");
  link.set("svc.other", "kept");
  link.set_tree("svc.items", list({leaf("a")}));
  link.set("svc.items.0", "after"); // after the tree: kept, and after it
  EXPECT_EQ(link.pending(), 3u);
  link.apply(app.root(), 0);
  EXPECT_EQ(value_at(app, "svc.items"), "1");
  EXPECT_EQ(value_at(app, "svc.items.0"), "after");
  EXPECT_FALSE(has(app, "svc.items.0.name"));
  EXPECT_EQ(value_at(app, "svc.other"), "kept");
}

TEST(ServerLinkTest, ATreeRemovesWhatItDoesNotName) {
  cvc::app app;
  ServerLink link;
  link.set_tree("svc.jobs", dict({{"a", leaf("1")}, {"b", dict({{"x", leaf("2")}})}}));
  link.apply(app.root(), 0);
  EXPECT_EQ(value_at(app, "svc.jobs.b.x"), "2");
  // b becomes a scalar (its children go), a goes, c is new.
  link.set_tree("svc.jobs", dict({{"b", leaf("done")}, {"c", leaf("3")}}));
  link.apply(app.root(), 0);
  EXPECT_FALSE(has(app, "svc.jobs.a"));
  EXPECT_EQ(value_at(app, "svc.jobs.b"), "done");
  EXPECT_FALSE(has(app, "svc.jobs.b.x"));
  EXPECT_EQ(value_at(app, "svc.jobs.c"), "3");
  // A list that shrinks keeps 0..n-1 only.
  link.set_tree("svc.ring", list({leaf("a"), leaf("b"), leaf("c")}));
  link.apply(app.root(), 0);
  link.set_tree("svc.ring", list({leaf("z")}));
  link.apply(app.root(), 0);
  EXPECT_EQ(value_at(app, "svc.ring"), "1");
  EXPECT_EQ(value_at(app, "svc.ring.0"), "z");
  EXPECT_FALSE(has(app, "svc.ring.1"));
  EXPECT_FALSE(has(app, "svc.ring.2"));
}

TEST(ServerLinkTest, ARewriteOfTheSameTreeChangesNothing) {
  cvc::app app;
  ServerLink link;
  link.set_tree("svc.t", dict({{"a", leaf("1")}, {"b", leaf("2")}}));
  link.apply(app.root(), 0);
  Recorder rec(app, "svc.t");
  link.set_tree("svc.t", dict({{"a", leaf("1")}, {"b", leaf("2")}}));
  link.apply(app.root(), 0);
  EXPECT_TRUE(rec.seen.empty()) << "a write of what is there already signals no change";
  link.set_tree("svc.t", dict({{"a", leaf("1")}, {"b", leaf("3")}}));
  link.apply(app.root(), 0);
  EXPECT_EQ(rec.seen, std::vector<std::string>{"b"});
}

TEST(ServerLinkTest, SizeOfCountsEveryNode) {
  EXPECT_EQ(ServerLink::size_of("p", leaf("abc")), 4u);
  // "p" + "2" (the list's text) + "0" "." "x" + "1" "." "yz"
  EXPECT_EQ(ServerLink::size_of("p", list({leaf("x"), leaf("yz")})), 1u + 1u + 3u + 4u);
}

TEST(ServerLinkTest, WritesFromAnotherThreadWhileTheRelayApplies) {
  cvc::app app;
  ServerLink link;
  std::thread script([&] {
    for (int i = 1; i <= 2000; ++i)
      link.set("svc.count", std::to_string(i));
  });
  for (int steps = 0; steps < 100000 && value_at(app, "svc.count") != "2000"; ++steps) {
    link.apply(app.root(), 64);
    if (link.pending() == 0)
      std::this_thread::yield();
  }
  script.join();
  link.apply(app.root(), 0);
  EXPECT_EQ(value_at(app, "svc.count"), "2000");
  EXPECT_EQ(link.pending(), 0u);
}

TEST(ServerLinkTest, EventsReachTheWakerUntilClosed) {
  ServerLink link;
  std::mutex mu;
  std::vector<ServerLink::Event> got;
  std::thread waker([&] {
    ServerLink::Event e;
    while (link.next(e)) {
      std::lock_guard<std::mutex> lk(mu);
      got.push_back(e);
    }
  });
  link.post("shutdown");
  // An event's value is a DSL value, kept as it is: an int stays an int, a float a float.
  link.post(ServerLink::Event{
      "doc_call", 7,
      se::make_dict({{"fn", se::value_t(std::string("f"))},
                     {"args", se::make_list({se::value_t(std::int64_t(3)), se::value_t(3.0)})}})});
  // close() lets the waker take what is queued, then ends it; later posts are dropped.
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    std::lock_guard<std::mutex> lk(mu);
    if (got.size() == 2)
      break;
  }
  link.close();
  link.post("dropped");
  waker.join();
  ASSERT_EQ(got.size(), 2u);
  EXPECT_EQ(got[0].kind, "shutdown");
  EXPECT_EQ(got[0].id, 0u);
  EXPECT_TRUE(got[0].value.is_nil());
  EXPECT_EQ(got[1].kind, "doc_call");
  EXPECT_EQ(got[1].id, 7u);
  const se::value_t *args = se::field(got[1].value, "args");
  ASSERT_NE(args, nullptr);
  const se::list_ptr *l = std::get_if<se::list_ptr>(&args->v);
  ASSERT_TRUE(l && *l && (*l)->size() == 2);
  EXPECT_TRUE(std::holds_alternative<std::int64_t>((**l)[0].v));
  EXPECT_TRUE(std::holds_alternative<double>((**l)[1].v));
}

TEST(ServerLinkTest, RepliesWaitForTheRelayInOrder) {
  ServerLink link;
  ServerLink::Reply a;
  a.pid = "p1";
  a.id = "c1";
  a.kind = ServerLink::Reply::Kind::progress;
  a.fraction = 0.5;
  a.message = "half";
  link.reply(a);
  ServerLink::Reply b;
  b.pid = "p1";
  b.id = "c1";
  b.value = se::value_t(std::int64_t(42));
  link.reply(b);
  const std::vector<ServerLink::Reply> r = link.take_replies();
  ASSERT_EQ(r.size(), 2u);
  EXPECT_EQ(r[0].kind, ServerLink::Reply::Kind::progress);
  EXPECT_EQ(r[0].message, "half");
  EXPECT_EQ(r[1].kind, ServerLink::Reply::Kind::result);
  EXPECT_EQ(std::get<std::int64_t>(r[1].value.v), 42);
  EXPECT_TRUE(link.take_replies().empty());
}

TEST(ServerLinkTest, AnAnswerSettlesItsWaiterOnce) {
  ServerLink link;
  int calls = 0;
  se::value_t got;
  const std::uint64_t id =
      link.await_answer([&](bool ok, const se::value_t &v, const std::string &) {
        ++calls;
        EXPECT_TRUE(ok);
        got = v;
      });
  EXPECT_TRUE(link.answer(id, true, se::value_t(2.5)));
  EXPECT_FALSE(link.answer(id, true, se::value_t(1.0))) << "answered already";
  EXPECT_EQ(calls, 1);
  EXPECT_EQ(std::get<double>(got.v), 2.5);

  const std::uint64_t gone = link.await_answer([&](bool, const se::value_t &, const std::string &) {
    ADD_FAILURE() << "a forgotten waiter is never called";
  });
  link.forget(gone);
  EXPECT_FALSE(link.answer(gone, true, se::value_t()));

  // close() fails whatever still waits: nothing will answer it now.
  std::string why;
  link.await_answer([&](bool ok, const se::value_t &, const std::string &error) {
    EXPECT_FALSE(ok);
    why = error;
  });
  link.close();
  EXPECT_EQ(why, "the Python program has ended");
}

TEST(ServerLinkTest, StartStates) {
  {
    ServerLink link;
    EXPECT_EQ(link.wait_start(std::chrono::milliseconds(1)), ServerLink::Start::waiting);
    link.ready(true, "");
    link.ready(false, "too late"); // once only
    EXPECT_EQ(link.start(), ServerLink::Start::serving);
    EXPECT_TRUE(link.why().empty());
    EXPECT_FALSE(link.finished());
    link.finish(3);
    EXPECT_EQ(link.start(), ServerLink::Start::serving);
    EXPECT_TRUE(link.finished());
    EXPECT_EQ(link.status(), 3);
  }
  {
    ServerLink link;
    std::thread t([&] { link.ready(false, "the port is taken"); });
    EXPECT_EQ(link.wait_start(std::chrono::seconds(10)), ServerLink::Start::failed);
    t.join();
    EXPECT_EQ(link.why(), "the port is taken");
  }
  {
    ServerLink link; // ended before it served or failed
    EXPECT_FALSE(link.wait_finished(std::chrono::milliseconds(1)));
    std::thread t([&] { link.finish(1); });
    EXPECT_TRUE(link.wait_finished(std::chrono::seconds(10)));
    t.join();
    EXPECT_EQ(link.wait_start(std::chrono::milliseconds(1)), ServerLink::Start::finished);
    EXPECT_EQ(link.status(), 1);
  }
}
