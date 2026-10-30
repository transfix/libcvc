/// @file state_exec_intrinsics_test.cpp
/// @brief Tests for Phase 5: Intrinsics, Resource Policy, Stdlib.

#include <boost/any.hpp>
#include <boost/thread/thread.hpp>
#include <cmath>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/resource_policy.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/stdlib.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::state_exec;

// ===========================================================================
// IntrinsicsContextTest — verify intrinsics_context defaults
// ===========================================================================

class IntrinsicsContextTest : public ::testing::Test {};

TEST_F(IntrinsicsContextTest, DefaultValues) {
  intrinsics_context ctx;
  EXPECT_EQ(ctx.sched, nullptr);
  EXPECT_EQ(ctx.root, nullptr);
  EXPECT_EQ(ctx.tracker, nullptr);
  EXPECT_EQ(ctx.proc, nullptr);
  EXPECT_EQ(ctx.pid, -1);
  EXPECT_TRUE(ctx.uid.empty());
  EXPECT_TRUE(ctx.cluster_id.empty());
  EXPECT_TRUE(ctx.node_id.empty());
}

// ===========================================================================
// StateTreeIntrinsicsTest — state-get, state-set, state-exists, etc.
// ===========================================================================

class StateTreeIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::ready;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    ictx.uid = "test-user";
    ictx.cluster_id = "cluster-1";
    ictx.node_id = "node-1";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    EXPECT_NE(fn_val, nullptr) << "Function not found: " << name;
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    EXPECT_NE(fn, nullptr) << "Not a function: " << name;
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(StateTreeIntrinsicsTest, StateSetAndGet) {
  call("state-set", {std::string("test.key"), std::string("hello")});
  auto result = call("state-get", {std::string("test.key")});
  ASSERT_TRUE(std::holds_alternative<std::string>(result.v));
  EXPECT_EQ(std::get<std::string>(result.v), "hello");
}

TEST_F(StateTreeIntrinsicsTest, StateGetNonexistent) {
  auto result = call("state-get", {std::string("nonexistent.path")});
  EXPECT_TRUE(result.is_nil());
}

TEST_F(StateTreeIntrinsicsTest, StateExists) {
  call("state-set", {std::string("exists.test"), std::string("val")});
  auto yes = call("state-exists", {std::string("exists.test")});
  auto no = call("state-exists", {std::string("nope.path")});
  EXPECT_TRUE(std::get<bool>(yes.v));
  EXPECT_FALSE(std::get<bool>(no.v));
}

TEST_F(StateTreeIntrinsicsTest, StateChildren) {
  call("state-set", {std::string("parent.child1"), std::string("a")});
  call("state-set", {std::string("parent.child2"), std::string("b")});
  auto result = call("state-children", {std::string("parent")});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  EXPECT_GE((*lst)->size(), 2u);
}

TEST_F(StateTreeIntrinsicsTest, StateChildrenListsEveryDescendantByFullPath) {
  // The developer guide's example: every node under the node, in pre-order, by full path.
  call("state-set", {std::string("myapp.config.timeout"), std::string("30")});
  call("state-set", {std::string("myapp.users.ann.role"), std::string("admin")});
  const auto names = [&](const char *path) {
    const value_t got = call("state-children", {std::string(path)});
    std::vector<std::string> out;
    for (const value_t &v : *std::get<list_ptr>(got.v))
      out.push_back(std::get<std::string>(v.v));
    return out;
  };
  EXPECT_EQ(names("myapp.config"), std::vector<std::string>{"myapp.config.timeout"});
  EXPECT_EQ(names("myapp.users"),
            (std::vector<std::string>{"myapp.users.ann", "myapp.users.ann.role"}));
}

TEST_F(StateTreeIntrinsicsTest, StateChildrenNonexistent) {
  auto result = call("state-children", {std::string("no.such.path")});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  EXPECT_TRUE((*lst)->empty());
}

// --- §7.8.3 read-through: a transparent link node is a "named hole" a read sees through -----

TEST_F(StateTreeIntrinsicsTest, TransparentLinkReadsThroughValueAndChildren) {
  cvc::state &root = cvc::state::instance(app_ctx);
  root("tgt").value(std::string("through"));
  root("tgt.a").value(std::string("1"));
  root("tgt.b").value(std::string("2"));
  root("hole").linkTo("tgt", cvc::state::link_mode::transparent);
  // state-get follows the transparent link to the TARGET's value.
  EXPECT_EQ(std::get<std::string>(call("state-get", {std::string("hole")}).v), "through");
  // state-children lists the TARGET's children, not the (childless) link node's own.
  auto ch = call("state-children", {std::string("hole")});
  auto *lst = std::get_if<list_ptr>(&ch.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_NE(*lst, nullptr);
  EXPECT_EQ((*lst)->size(), 2u);
}

TEST_F(StateTreeIntrinsicsTest, OpaqueLinkReadsItsOwnValue) {
  cvc::state &root = cvc::state::instance(app_ctx);
  root("tgt_op").value(std::string("through"));
  root("hole_op").value(std::string("own")).linkTo("tgt_op", cvc::state::link_mode::opaque);
  // Opaque link -> the node's OWN value, not the target's (only transparent reads through).
  EXPECT_EQ(std::get<std::string>(call("state-get", {std::string("hole_op")}).v), "own");
}

TEST_F(StateTreeIntrinsicsTest, BrokenTransparentLinkFallsBackToOwnValue) {
  cvc::state &root = cvc::state::instance(app_ctx);
  root("hole_broken")
      .value(std::string("fallback"))
      .linkTo("no.such.target", cvc::state::link_mode::transparent);
  EXPECT_EQ(std::get<std::string>(call("state-get", {std::string("hole_broken")}).v), "fallback");
}

TEST_F(StateTreeIntrinsicsTest, TransparentLinkReadsThroughData) {
  cvc::state &root = cvc::state::instance(app_ctx);
  call("state-data-set", {std::string("dtgt"), value_t(std::string("dv"))});
  root("dhole").linkTo("dtgt", cvc::state::link_mode::transparent);
  auto d = call("state-data-get", {std::string("dhole")});
  ASSERT_TRUE(std::holds_alternative<std::string>(d.v));
  EXPECT_EQ(std::get<std::string>(d.v), "dv");
}

// A hole on an interior node (an Ariadne `link:` on a subtree): a program chrooted to "mod" reads
// the target's descendants through it, writes them through a writable one, and keeps its writes
// local under a read-only one. Nothing outside the target's subtree becomes reachable.
TEST_F(StateTreeIntrinsicsTest, AHoleExposesTheTargetsDescendantsToReads) {
  cvc::state &root = cvc::state::instance(app_ctx);
  root("parent.res.packages.0.name").value(std::string("zstd"));
  root("parent.res.packages.1.name").value(std::string("lz4"));
  root("parent.secret").value(std::string("no"));
  root("mod.tree").linkTo("parent.res", cvc::state::link_mode::transparent);
  ictx.root = &root("mod");
  call("state-data-set", {std::string("tree.meta"), value_t(int64_t(3))}); // read-only: local
  root("parent.res.meta").data(boost::any(value_t(int64_t(4))));
  EXPECT_EQ(std::get<std::string>(call("state-get", {std::string("tree.packages.0.name")}).v),
            "zstd");
  auto ch = call("state-children", {std::string("tree.packages")});
  auto *lst = std::get_if<list_ptr>(&ch.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_NE(*lst, nullptr);
  // The target's whole subtree by full name (state::children): packages.0, .0.name, .1, .1.name.
  ASSERT_EQ((*lst)->size(), 4u);
  EXPECT_EQ(std::get<std::string>((*lst)->at(3).v), "parent.res.packages.1.name");
  EXPECT_TRUE(std::get<bool>(call("state-exists", {std::string("tree.packages.1.name")}).v));
  EXPECT_FALSE(std::get<bool>(call("state-exists", {std::string("tree.packages.2")}).v));
  auto d = call("state-data-get", {std::string("tree.meta")});
  ASSERT_TRUE(std::holds_alternative<int64_t>(d.v));
  EXPECT_EQ(std::get<int64_t>(d.v), 4); // the target's, not the local write
  auto j = call("state-json", {std::string("tree.packages.1")});
  ASSERT_TRUE(std::holds_alternative<std::string>(j.v));
  EXPECT_NE(std::get<std::string>(j.v).find("lz4"), std::string::npos);
  // Default-deny: the parent outside the hole's target is still out of reach.
  EXPECT_TRUE(call("state-get", {std::string("parent.secret")}).is_nil());
  EXPECT_TRUE(call("state-get", {std::string("tree.secret")}).is_nil());
}

TEST_F(StateTreeIntrinsicsTest, AWritableHoleTakesDescendantWritesOnTheTarget) {
  cvc::state &root = cvc::state::instance(app_ctx);
  root("parent.res.a").value(std::string("1"));
  root("mod.tree").linkTo("parent.res", cvc::state::link_mode::transparent).setLinkWritable(true);
  ictx.root = &root("mod");
  call("state-set", {std::string("tree.a"), std::string("2")});
  call("state-set", {std::string("tree.new.leaf"), std::string("x")});
  call("state-data-set", {std::string("tree.d"), value_t(std::string("dv"))});
  call("state-incr", {std::string("tree.n")});
  EXPECT_TRUE(std::get<bool>(
      call("state-cas", {std::string("tree.a"), std::string("2"), std::string("3")}).v));
  EXPECT_EQ(root("parent.res.a").value(), "3");
  EXPECT_EQ(root("parent.res.new.leaf").value(), "x");
  EXPECT_EQ(root("parent.res.n").value(), "1");
  EXPECT_FALSE(root("parent.res.d").data().empty());
  EXPECT_TRUE(root("mod.tree").children().empty()); // nothing landed on the hole
}

TEST_F(StateTreeIntrinsicsTest, AReadOnlyHoleKeepsDescendantWritesLocal) {
  cvc::state &root = cvc::state::instance(app_ctx);
  root("parent.res.a").value(std::string("1"));
  root("mod.tree").linkTo("parent.res", cvc::state::link_mode::transparent); // read-only
  ictx.root = &root("mod");
  call("state-set", {std::string("tree.a"), std::string("2")});
  call("state-set", {std::string("tree.new"), std::string("x")});
  call("state-data-set", {std::string("tree.d"), value_t(std::string("dv"))});
  call("state-from-json", {std::string("tree.j"), std::string("{\"k\": \"v\"}")});
  EXPECT_EQ(root("parent.res.a").value(), "1");
  EXPECT_EQ(root.findDescendant("parent.res.new"), nullptr);
  EXPECT_EQ(root.findDescendant("parent.res.d"), nullptr);
  EXPECT_EQ(root.findDescendant("parent.res.j"), nullptr);
  EXPECT_EQ(root.findDescendant("mod.tree.a")->value(), "2"); // the hole's own child
  // A read still sees the target.
  EXPECT_EQ(std::get<std::string>(call("state-get", {std::string("tree.a")}).v), "1");
  // The atomics refuse: their write would not land on the node a read sees.
  EXPECT_ANY_THROW(call("state-incr", {std::string("tree.n")}));
  EXPECT_ANY_THROW(call("state-cas", {std::string("tree.a"), std::string("1"), std::string("5")}));
  EXPECT_ANY_THROW(call("state-swap", {std::string("tree.a"), std::string("5")}));
  EXPECT_EQ(root("parent.res.a").value(), "1");
  EXPECT_EQ(root.findDescendant("parent.res.n"), nullptr);
}

TEST_F(StateTreeIntrinsicsTest, StateDelete) {
  call("state-set", {std::string("del.target"), std::string("bye")});
  EXPECT_TRUE(std::get<bool>(call("state-exists", {std::string("del.target")}).v));
  call("state-delete", {std::string("del.target")});
  EXPECT_FALSE(std::get<bool>(call("state-exists", {std::string("del.target")}).v));
}

TEST_F(StateTreeIntrinsicsTest, StateDeleteNonexistent) {
  // Should not throw
  EXPECT_NO_THROW(call("state-delete", {std::string("nothing.here")}));
}

TEST_F(StateTreeIntrinsicsTest, StateDataRoundTripsTheValue) {
  // A DSL value set via state-data-set comes back from state-data-get transparently (the
  // original value_t, NOT an opaque data_object handle) — so structured data round-trips.
  call("state-data-set", {std::string("data.node"), std::string("data-val")});
  auto s = call("state-data-get", {std::string("data.node")});
  ASSERT_TRUE(std::holds_alternative<std::string>(s.v));
  EXPECT_EQ(std::get<std::string>(s.v), "data-val");

  // a structured value (a list) round-trips intact
  auto lst = make_list({value_t{int64_t{1}}, value_t{int64_t{2}}, value_t{int64_t{3}}});
  call("state-data-set", {std::string("data.list"), lst});
  auto got = call("state-data-get", {std::string("data.list")});
  auto *gl = std::get_if<list_ptr>(&got.v);
  ASSERT_NE(gl, nullptr);
  EXPECT_EQ((*gl)->size(), 3u);
}

TEST_F(StateTreeIntrinsicsTest, StateDataSetRejectsCallables) {
  // Storing a callable is a lifetime hazard (it captures a context/env that may not outlive
  // the node) and an invocation channel that bypasses env allowlists — refuse it.
  auto *fn = env->lookup("state-get"); // a native_fn value_t
  ASSERT_NE(fn, nullptr);
  EXPECT_THROW(call("state-data-set", {std::string("bad"), *fn}), std::runtime_error);
}

TEST_F(StateTreeIntrinsicsTest, StateDataGetReturnsPrivateCopy) {
  // The fetched structure must NOT alias the node's stored storage: mutating it must not
  // change what a later state-data-get returns (else a "read" could mutate persistent state).
  call("state-data-set",
       {std::string("dc"), make_list({value_t{int64_t{1}}, value_t{int64_t{2}}})});
  auto got = call("state-data-get", {std::string("dc")});
  auto *gl = std::get_if<list_ptr>(&got.v);
  ASSERT_NE(gl, nullptr);
  (*gl)->push_back(value_t{int64_t{99}}); // mutate the returned handle
  auto again = call("state-data-get", {std::string("dc")});
  auto *gl2 = std::get_if<list_ptr>(&again.v);
  ASSERT_NE(gl2, nullptr);
  EXPECT_EQ((*gl2)->size(), 2u); // stored data is untouched -> the get returned a private copy
}

TEST_F(StateTreeIntrinsicsTest, StateDataGetOfSharedDagIsBoundedAndPreservesSharing) {
  // A physically-tiny shared DAG (40 doublings = 2^40 logical) round-trips via a MEMOIZED
  // deep copy: bounded (no exponential unfold), and the returned copy preserves the DAG's
  // sharing (both children are the SAME copied node) while aliasing none of the stored value.
  value_t d{int64_t{0}};
  for (int i = 0; i < 40; ++i)
    d = make_list({d, d});
  call("state-data-set", {std::string("dag"), d});
  value_t got = call("state-data-get", {std::string("dag")}); // must not hang/OOM
  auto *gl = std::get_if<list_ptr>(&got.v);
  ASSERT_NE(gl, nullptr);
  ASSERT_EQ((*gl)->size(), 2u);
  auto *c0 = std::get_if<list_ptr>(&(**gl)[0].v);
  auto *c1 = std::get_if<list_ptr>(&(**gl)[1].v);
  ASSERT_NE(c0, nullptr);
  ASSERT_NE(c1, nullptr);
  EXPECT_EQ(c0->get(), c1->get()); // sharing preserved -> the copy is a DAG, not a 2^40 tree
}

TEST_F(StateTreeIntrinsicsTest, StateDataGetOfDeepValueCopies) {
  // The deep copy keeps its work on the heap, so depth has no cap: a value 100000 levels deep
  // copies on a 256 KiB stack, into a private copy equal to the original.
  value_t d{int64_t{0}};
  for (int i = 0; i < 100000; ++i)
    d = make_list({d});
  call("state-data-set", {std::string("deep"), d});
  value_t got;
  std::string error;
  boost::thread::attributes small_stack;
  small_stack.set_stack_size(256 * 1024);
  boost::thread t(small_stack, [&] {
    try {
      got = call("state-data-get", {std::string("deep")});
    } catch (const std::exception &e) {
      error = e.what();
    }
  });
  t.join();
  EXPECT_EQ(error, "");
  ASSERT_TRUE(std::holds_alternative<list_ptr>(got.v));
  EXPECT_NE(std::get<list_ptr>(got.v).get(), std::get<list_ptr>(d.v).get());
  EXPECT_TRUE(values_equal(got, d));
}

TEST_F(StateTreeIntrinsicsTest, StateSetCoercesScalarsToString) {
  // state-set stores string-typed scalars; non-string values are coerced to their natural
  // lexical form rather than throwing — (state-set "n" 10) stores "10".
  call("state-set", {std::string("c.int"), value_t{int64_t{10}}});
  EXPECT_EQ(std::get<std::string>(call("state-get", {std::string("c.int")}).v), "10");
  call("state-set", {std::string("c.dbl"), value_t{1.5}});
  EXPECT_EQ(std::get<std::string>(call("state-get", {std::string("c.dbl")}).v), "1.5");
  call("state-set", {std::string("c.bool"), value_t{true}});
  EXPECT_EQ(std::get<std::string>(call("state-get", {std::string("c.bool")}).v), "true");
  // a string is stored raw — NOT to_string's quoted form
  call("state-set", {std::string("c.str"), std::string("hi")});
  EXPECT_EQ(std::get<std::string>(call("state-get", {std::string("c.str")}).v), "hi");
}

TEST_F(StateTreeIntrinsicsTest, StateSetStoresAFloatInItsShortestRoundTripForm) {
  // A float stores the shortest text that reads back as it, as state-incr's sums do, not six
  // significant digits: a balance of 1234567.5 is "1234567.5", not "1.23457e+06".
  const auto stored = [&](const char *name, double d) {
    call(name, {std::string("f"), value_t{d}});
    return std::get<std::string>(call("state-get", {std::string("f")}).v);
  };
  EXPECT_EQ(stored("state-set", 1234567.5), "1234567.5");
  EXPECT_EQ(stored("state-set", 0.1), "0.1");
  EXPECT_EQ(stored("state-set", 1e21), "1e+21");
  EXPECT_EQ(stored("state-set", -2.5), "-2.5");
  // Of the fixed and scientific texts the shorter is written, so 100000.0 is "1e+05" (six
  // digits wrote "100000"), as state-incr writes it.
  EXPECT_EQ(stored("state-set", 1e5), "1e+05");
  // 0.1 + 0.2 is not 0.3: six digits stored "0.3", which reads back as a different number.
  EXPECT_EQ(stored("state-set", 0.1 + 0.2), "0.30000000000000004");
  for (const double d : {0.1 + 0.2, 1.0 / 3, 6.02214076e23, 5e-324, 123456789.123})
    EXPECT_EQ(std::strtod(stored("state-set", d).c_str(), nullptr), d) << d;
  // state-swap and state-cas store a float as state-set does.
  EXPECT_EQ(stored("state-swap", 1234567.5), "1234567.5");
  // str prints a float as state-set stores it, so (state-set k (str d)) stores the same text.
  for (const double d : {1234567.5, 0.1 + 0.2, 2.0, 1e5, 1e21})
    EXPECT_EQ(std::get<std::string>(call("str", {value_t{d}}).v), stored("state-set", d)) << d;
}

TEST_F(StateTreeIntrinsicsTest, StateDataGetNonexistent) {
  auto result = call("state-data-get", {std::string("no.data")});
  EXPECT_TRUE(result.is_nil());
}

TEST_F(StateTreeIntrinsicsTest, StateDataGetReadsRawStringBlobAsBytes) {
  // A RAW std::string parked on the data channel — what the ?data URI channel (uri_state.cpp) and
  // the §13.9 HTTP cache write — reads back as the DSL `bytes` type, NOT an opaque data_object.
  // This is the coherence the cache depends on: a byte blob is the same on both sides.
  cvc::state &root = cvc::state::instance(app_ctx);
  root("blob").data(boost::any(std::string("raw-octets")));
  auto got = call("state-data-get", {std::string("blob")});
  ASSERT_TRUE(std::holds_alternative<bytes_value>(got.v));
  EXPECT_EQ(std::get<bytes_value>(got.v).data, "raw-octets");
}

TEST_F(StateTreeIntrinsicsTest, StateDataSetBytesStoredAsRawStringForDataChannel) {
  // A `bytes` value set from the DSL is stored as a RAW std::string on the data channel (the ?data
  // / cache representation), so it round-trips as `bytes` AND is byte-serializable via
  // state://…?data.
  const std::string octets("A\0B\0C", 5); // embedded NULs prove byte transparency
  call("state-data-set", {std::string("b"), make_bytes(octets)});

  auto got = call("state-data-get", {std::string("b")}); // round-trips as bytes, byte-exact
  ASSERT_TRUE(std::holds_alternative<bytes_value>(got.v));
  EXPECT_EQ(std::get<bytes_value>(got.v).data, octets);

  // the on-node payload is a raw std::string — exactly what uri_state.cpp's ?data any_cast reads
  cvc::state *n = cvc::state::instance(app_ctx).findDescendant("b");
  ASSERT_NE(n, nullptr);
  const boost::any d = n->data();
  const std::string *raw = boost::any_cast<std::string>(&d);
  ASSERT_NE(raw, nullptr) << "a bytes value must land as a raw std::string for ?data compatibility";
  EXPECT_EQ(*raw, octets);
}

TEST_F(StateTreeIntrinsicsTest, StateDataStringAndBytesTracksAreDistinct) {
  // The text `string` track and the `bytes` track stay distinct through state-data-set/get: a text
  // string comes back a string (stored as a value_t), a bytes value comes back bytes.
  call("state-data-set", {std::string("t"), value_t(std::string("text"))});
  auto t = call("state-data-get", {std::string("t")});
  ASSERT_TRUE(std::holds_alternative<std::string>(t.v)); // string track (unchanged)
  EXPECT_EQ(std::get<std::string>(t.v), "text");

  call("state-data-set", {std::string("y"), make_bytes(std::string("bin"))});
  auto y = call("state-data-get", {std::string("y")});
  ASSERT_TRUE(std::holds_alternative<bytes_value>(y.v)); // bytes track
  EXPECT_EQ(std::get<bytes_value>(y.v).data, "bin");
}

TEST_F(StateTreeIntrinsicsTest, StateGetWrongArgCount) {
  EXPECT_THROW(call("state-get", {}), std::runtime_error);
  EXPECT_THROW(call("state-get", {std::string("a"), std::string("b")}), std::runtime_error);
}

TEST_F(StateTreeIntrinsicsTest, StateSetWrongArgCount) {
  EXPECT_THROW(call("state-set", {std::string("a")}), std::runtime_error);
}

// --- state-set-tree: a value of dicts, lists and scalars as a subtree --------------------------

TEST_F(StateTreeIntrinsicsTest, StateSetTreeWritesDictsListsAndScalars) {
  cvc::state &root = cvc::state::instance(app_ctx);
  call("state-set-tree",
       {std::string("t"),
        make_dict({{"s", value_t(std::string("text"))},
                   {"i", value_t(int64_t{-3})},
                   {"f", value_t(0.1 + 0.2)},
                   {"big", value_t(1234567.5)},
                   {"yes", value_t(true)},
                   {"no", value_t(false)},
                   {"none", nil_value},
                   {"items", make_list({make_dict({{"text", value_t(std::string("a"))}}),
                                        make_dict({{"text", value_t(std::string("b"))}})})},
                   {"empty", make_list()}})});
  EXPECT_EQ(root("t.s").value(), "text");
  EXPECT_EQ(root("t.i").value(), "-3");
  EXPECT_EQ(root("t.f").value(), "0.30000000000000004"); // every digit, unlike state-set
  EXPECT_EQ(root("t.big").value(), "1234567.5");
  EXPECT_EQ(root("t.yes").value(), "true");
  EXPECT_EQ(root("t.no").value(), "false");
  ASSERT_NE(root.findDescendant("t.none"), nullptr); // nil: the node, empty
  EXPECT_EQ(root("t.none").value(), "");
  EXPECT_EQ(root("t").value(), "");        // a dict's node
  EXPECT_EQ(root("t.items").value(), "2"); // a list's node holds its length, for repeat:
  EXPECT_EQ(root("t.items.0.text").value(), "a");
  EXPECT_EQ(root("t.items.1.text").value(), "b");
  EXPECT_EQ(root("t.empty").value(), "0");
  EXPECT_EQ(root("t.items").childNames(), (std::vector<std::string>{"0", "1"}));
}

TEST_F(StateTreeIntrinsicsTest, StateSetTreeReplacesTheSubtree) {
  cvc::state &root = cvc::state::instance(app_ctx);
  root("t").value(std::string("old text"));
  root("t.stale.deep").value(std::string("x"));
  const auto items = [](int n) {
    std::vector<value_t> v;
    for (int i = 0; i < n; ++i)
      v.push_back(make_dict({{"id", value_t(int64_t{i})}, {"extra", value_t(true)}}));
    return make_list(std::move(v));
  };
  call("state-set-tree",
       {std::string("t"), make_dict({{"items", items(12)}, {"error", nil_value}})});
  EXPECT_EQ(root.findDescendant("t.stale"), nullptr);
  EXPECT_EQ(root("t").value(), "");
  EXPECT_EQ(root("t.items.11.id").value(), "11");
  // Fewer items, a key gone, an element that lost a key: only what the value has is left.
  call("state-set-tree",
       {std::string("t"), make_dict({{"items", make_list({make_dict({{"id", value_t(int64_t{7})}}),
                                                          value_t(std::string("plain"))})}})});
  EXPECT_EQ(root("t.items").value(), "2");
  EXPECT_EQ(root("t.items").childNames(), (std::vector<std::string>{"0", "1"}));
  EXPECT_EQ(root("t.items.0.id").value(), "7");
  EXPECT_EQ(root.findDescendant("t.items.0.extra"), nullptr);
  EXPECT_EQ(root("t.items.1").value(), "plain");
  EXPECT_EQ(root("t.items.1").numChildren(), 0u); // a scalar where a dict was
  EXPECT_EQ(root.findDescendant("t.error"), nullptr);
  // A scalar replaces the whole subtree; a list replaces a dict's keys.
  call("state-set-tree", {std::string("t"), value_t(int64_t{5})});
  EXPECT_EQ(root("t").value(), "5");
  EXPECT_EQ(root("t").numChildren(), 0u);
  call("state-set-tree", {std::string("t"), make_dict({{"10", value_t(true)}, {"x", nil_value}})});
  call("state-set-tree", {std::string("t"), make_list({nil_value, nil_value})});
  EXPECT_EQ(root("t").childNames(), (std::vector<std::string>{"0", "1"})); // "10" isn't below 2
}

TEST_F(StateTreeIntrinsicsTest, StateSetTreeLeavesUnchangedNodesUnwritten) {
  cvc::state &root = cvc::state::instance(app_ctx);
  const value_t v = make_dict({{"a", value_t(std::string("same"))}, {"b", value_t(int64_t{1})}});
  call("state-set-tree", {std::string("t"), v});
  int a_writes = 0, b_writes = 0;
  auto ca = root("t.a").valueChanged.connect([&] { ++a_writes; });
  auto cb = root("t.b").valueChanged.connect([&] { ++b_writes; });
  call("state-set-tree", {std::string("t"), make_dict({{"a", value_t(std::string("same"))},
                                                       {"b", value_t(int64_t{2})}})});
  EXPECT_EQ(a_writes, 0);
  EXPECT_EQ(b_writes, 1);
  ca.disconnect();
  cb.disconnect();
}

TEST_F(StateTreeIntrinsicsTest, StateSetTreeChecksTheValueBeforeWritingAnything) {
  cvc::state &root = cvc::state::instance(app_ctx);
  const auto refuses = [&](const value_t &v, const std::string &cls, const std::string &what) {
    try {
      call("state-set-tree", {std::string("t"), v});
      ADD_FAILURE() << "accepted " << to_string(v);
    } catch (const dsl_error &e) {
      EXPECT_EQ(e.class_name(), cls) << e.what();
      EXPECT_NE(std::string(e.what()).find(what), std::string::npos) << e.what();
    }
    EXPECT_EQ(root.findDescendant("t"), nullptr) << "wrote part of " << to_string(v);
  };
  // The bad part comes last, after values that would have been written.
  const value_t good = make_dict({{"ok", value_t(int64_t{1})}});
  for (const char *key : {"", "a.b", " a", "a ", "  "})
    refuses(make_list({good, make_dict({{key, nil_value}})}), "ValueError", "can't name a node");
  refuses(make_list({good, *env->lookup("+")}), "TypeError", "has no place in the state tree");
  refuses(make_list({good, make_bytes("x")}), "TypeError", "a bytes");
  value_t self = make_list({good});
  std::get<list_ptr>(self.v)->push_back(self);
  refuses(self, "ValueError", "a list that contains itself");
  std::get<list_ptr>(self.v)->pop_back();
  value_t deep = make_list();
  for (std::size_t i = 0; i < kJsonMaxDepth; ++i)
    deep = make_list({deep});
  refuses(make_list({good, deep}), "ValueError", "nested deeper than 512");
  // The path must name a node below the root.
  root("keep").value(std::string("me"));
  for (const char *path : {"", ".", " . "})
    EXPECT_THROW(call("state-set-tree", {std::string(path), make_dict()}), dsl_error) << path;
  EXPECT_EQ(root("keep").value(), "me");
  // A value held twice is written twice.
  call("state-set-tree", {std::string("t"), make_list({good, good})});
  EXPECT_EQ(root("t.1.ok").value(), "1");
}

TEST_F(StateTreeIntrinsicsTest, StateSetTreeWritesThroughAWritableLink) {
  cvc::state &root = cvc::state::instance(app_ctx);
  root("target"); // a link resolves to a node that exists
  root("hole").linkTo("target", cvc::state::link_mode::transparent);
  root("hole").setLinkWritable(true);
  call("state-set-tree", {std::string("hole"), make_dict({{"k", value_t(std::string("v"))}})});
  EXPECT_EQ(root("target.k").value(), "v");
}

// ===========================================================================
// SchedulerIntrinsicsTest
// ===========================================================================

class SchedulerIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::running;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    ictx.uid = "admin";
    ictx.cluster_id = "c1";
    ictx.node_id = "n1";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args = {}) {
    auto *fn_val = env->lookup(name);
    EXPECT_NE(fn_val, nullptr) << "Function not found: " << name;
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    EXPECT_NE(fn, nullptr) << "Not a function: " << name;
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(SchedulerIntrinsicsTest, SelfPid) {
  auto result = call("self-pid");
  EXPECT_EQ(std::get<int64_t>(result.v), 1);
}

TEST_F(SchedulerIntrinsicsTest, SelfUid) {
  auto result = call("self-uid");
  EXPECT_EQ(std::get<std::string>(result.v), "admin");
}

TEST_F(SchedulerIntrinsicsTest, ClusterId) {
  auto result = call("cluster-id");
  EXPECT_EQ(std::get<std::string>(result.v), "c1");
}

TEST_F(SchedulerIntrinsicsTest, NodeId) {
  auto result = call("node-id");
  EXPECT_EQ(std::get<std::string>(result.v), "n1");
}

TEST_F(SchedulerIntrinsicsTest, SpawnReturnsValidPid) {
  auto result = call("spawn", {std::string("(+ 1 2)")});
  auto pid = std::get<int64_t>(result.v);
  EXPECT_GT(pid, 0);
}

TEST_F(SchedulerIntrinsicsTest, SpawnWithName) {
  auto result = call("spawn", {std::string("(+ 1 2)"), std::string("adder")});
  auto pid = std::get<int64_t>(result.v);
  auto info = sched.get_process_info(static_cast<int>(pid));
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->name, "adder");
}

TEST_F(SchedulerIntrinsicsTest, KillProcess) {
  auto pid_val = call("spawn", {std::string("(+ 1 2)")});
  auto pid = std::get<int64_t>(pid_val.v);
  auto result = call("kill", {pid});
  EXPECT_TRUE(std::get<bool>(result.v));
}

TEST_F(SchedulerIntrinsicsTest, KillNonexistent) {
  auto result = call("kill", {int64_t(9999)});
  EXPECT_FALSE(std::get<bool>(result.v));
}

TEST_F(SchedulerIntrinsicsTest, PauseAndResume) {
  auto pid_val = call("spawn", {std::string("(+ 1 2)")});
  auto pid = std::get<int64_t>(pid_val.v);
  auto pause_result = call("pause", {pid});
  EXPECT_TRUE(std::get<bool>(pause_result.v));
  auto resume_result = call("resume", {pid});
  EXPECT_TRUE(std::get<bool>(resume_result.v));
}

TEST_F(SchedulerIntrinsicsTest, SleepPutsProcessToWaiting) {
  // Spawn a process in the actual scheduler so sleep can find it.
  auto pid_val = call("spawn", {std::string("(begin 1 2 3 4 5)")});
  auto pid = std::get<int64_t>(pid_val.v);
  // Step once to get the process started.
  sched.step();
  // Override ictx.pid so sleep() targets the spawned process.
  ictx.pid = static_cast<int>(pid);
  auto result = call("sleep", {0.01}); // 10ms
  EXPECT_TRUE(std::get<bool>(result.v));
  auto info = sched.get_process_info(static_cast<int>(pid));
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::waiting);
}

TEST_F(SchedulerIntrinsicsTest, PsList) {
  call("spawn", {std::string("(+ 1 2)")});
  auto result = call("ps");
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  EXPECT_GE((*lst)->size(), 1u);
}

TEST_F(SchedulerIntrinsicsTest, InspectProcess) {
  auto pid_val = call("spawn", {std::string("(+ 1 2)"), std::string("inspectable")});
  auto pid = std::get<int64_t>(pid_val.v);
  auto result = call("inspect", {pid});
  auto *d = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(d, nullptr);
  // Verify has expected keys
  bool found_pid = false, found_name = false;
  for (auto &[k, v] : **d) {
    if (k == "pid") {
      EXPECT_EQ(std::get<int64_t>(v.v), pid);
      found_pid = true;
    }
    if (k == "name") {
      EXPECT_EQ(std::get<std::string>(v.v), "inspectable");
      found_name = true;
    }
  }
  EXPECT_TRUE(found_pid);
  EXPECT_TRUE(found_name);
}

TEST_F(SchedulerIntrinsicsTest, InspectNonexistent) {
  auto result = call("inspect", {int64_t(9999)});
  EXPECT_TRUE(result.is_nil());
}

TEST_F(SchedulerIntrinsicsTest, SchedulerStats) {
  call("spawn", {std::string("(+ 1 2)")});
  auto result = call("scheduler-stats");
  auto *d = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(d, nullptr);
  bool found_total = false;
  for (auto &[k, v] : **d) {
    if (k == "total") {
      EXPECT_GE(std::get<int64_t>(v.v), int64_t(1));
      found_total = true;
    }
  }
  EXPECT_TRUE(found_total);
}

// ===========================================================================
// ResourceIntrinsicsTest
// ===========================================================================

class ResourceIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::running;
    proc->max_memory = 1024 * 1024;
    proc->max_time = 60;
    proc->max_messages = 100;
    proc->message_count = 5;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args = {}) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(ResourceIntrinsicsTest, MemoryUsage) {
  auto result = call("memory-usage");
  EXPECT_EQ(std::get<int64_t>(result.v), 0);
}

TEST_F(ResourceIntrinsicsTest, MemoryLimit) {
  auto result = call("memory-limit");
  EXPECT_EQ(std::get<int64_t>(result.v), 1024 * 1024);
}

TEST_F(ResourceIntrinsicsTest, TimeElapsed) {
  auto result = call("time-elapsed");
  EXPECT_GE(std::get<double>(result.v), 0.0);
}

TEST_F(ResourceIntrinsicsTest, TimeLimit) {
  auto result = call("time-limit");
  EXPECT_EQ(std::get<double>(result.v), 60.0);
}

TEST_F(ResourceIntrinsicsTest, MessageCount) {
  auto result = call("message-count");
  EXPECT_EQ(std::get<int64_t>(result.v), 5);
}

TEST_F(ResourceIntrinsicsTest, MessageLimit) {
  auto result = call("message-limit");
  EXPECT_EQ(std::get<int64_t>(result.v), 100);
}

TEST_F(ResourceIntrinsicsTest, StepCount) {
  auto result = call("step-count");
  EXPECT_GE(std::get<int64_t>(result.v), 0);
}

TEST_F(ResourceIntrinsicsTest, NullProcess) {
  // With no process bound, resource intrinsics return 0
  intrinsics_context ctx2;
  ctx2.sched = &sched;
  auto env2 = builtins::make_default_environment();
  register_intrinsics(env2, &ctx2);
  auto *fn_val = env2->lookup("memory-limit");
  auto *fn = std::get_if<native_fn>(&fn_val->v);
  auto result = (*fn)(std::span<const value_t>{});
  EXPECT_EQ(std::get<int64_t>(result.v), 0);
}

// ===========================================================================
// ExpiryIntrinsicsTest
// ===========================================================================

class ExpiryIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.sched = &sched;
    ictx.pid = 1;

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(ExpiryIntrinsicsTest, HasExpiryFalseByDefault) {
  call("state-set", {std::string("exp.node"), std::string("val")});
  auto result = call("state-has-expiry", {std::string("exp.node")});
  EXPECT_FALSE(std::get<bool>(result.v));
}

TEST_F(ExpiryIntrinsicsTest, SetExpiry) {
  call("state-set", {std::string("exp.node2"), std::string("val")});
  call("state-expire", {std::string("exp.node2"), 3600.0}); // 1hr
  auto result = call("state-has-expiry", {std::string("exp.node2")});
  EXPECT_TRUE(std::get<bool>(result.v));
}

TEST_F(ExpiryIntrinsicsTest, ClearExpiry) {
  call("state-set", {std::string("exp.clear"), std::string("val")});
  call("state-expire", {std::string("exp.clear"), 100.0});
  EXPECT_TRUE(std::get<bool>(call("state-has-expiry", {std::string("exp.clear")}).v));
  call("state-clear-expiry", {std::string("exp.clear")});
  EXPECT_FALSE(std::get<bool>(call("state-has-expiry", {std::string("exp.clear")}).v));
}

TEST_F(ExpiryIntrinsicsTest, IsNotExpiredByDefault) {
  call("state-set", {std::string("exp.fresh"), std::string("val")});
  auto result = call("state-is-expired", {std::string("exp.fresh")});
  EXPECT_FALSE(std::get<bool>(result.v));
}

TEST_F(ExpiryIntrinsicsTest, IsNotExpiredForFutureExpiry) {
  call("state-set", {std::string("exp.future"), std::string("val")});
  call("state-expire", {std::string("exp.future"), 3600.0});
  auto result = call("state-is-expired", {std::string("exp.future")});
  EXPECT_FALSE(std::get<bool>(result.v));
}

TEST_F(ExpiryIntrinsicsTest, ExpireNonexistentThrows) {
  EXPECT_THROW(call("state-expire", {std::string("no.path"), 100.0}), std::runtime_error);
}

TEST_F(ExpiryIntrinsicsTest, SweepExpiredRoot) {
  auto result = call("state-sweep-expired", {});
  EXPECT_EQ(std::get<int64_t>(result.v), 0);
}

TEST_F(ExpiryIntrinsicsTest, SweepNonexistent) {
  auto result = call("state-sweep-expired", {std::string("no.path")});
  EXPECT_EQ(std::get<int64_t>(result.v), 0);
}

TEST_F(ExpiryIntrinsicsTest, HasExpiryNonexistent) {
  auto result = call("state-has-expiry", {std::string("no.path")});
  EXPECT_FALSE(std::get<bool>(result.v));
}

// ===========================================================================
// MsgSendIntrinsicsTest
// ===========================================================================

class MsgSendIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::running;
    proc->message_count = 0;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.proc = proc;
    ictx.pid = 1;

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(MsgSendIntrinsicsTest, SendMessage) {
  auto result = call("msg-send", {std::string("msg.target"), std::string("hello")});
  auto *d = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(d, nullptr);
  EXPECT_EQ(proc->message_count, 1u);
}

TEST_F(MsgSendIntrinsicsTest, SendWithContentType) {
  auto result = call("msg-send", {std::string("msg.typed"), std::string("{\"a\":1}"),
                                  std::string("application/json")});
  auto *d = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(d, nullptr);
}

TEST_F(MsgSendIntrinsicsTest, SendIncreasesCount) {
  call("msg-send", {std::string("msg.a"), std::string("1")});
  call("msg-send", {std::string("msg.b"), std::string("2")});
  call("msg-send", {std::string("msg.c"), std::string("3")});
  EXPECT_EQ(proc->message_count, 3u);
}

namespace {
const value_t *dict_field(const dict_ptr &d, const char *key) {
  for (const auto &kv : *d)
    if (kv.first == key)
      return &kv.second;
  return nullptr;
}
} // namespace

// msg-send accepts a BINARY payload (previously threw "expected string"), and a receiver on the
// channel gets an ENVELOPE {status, path, content_type, payload} with the bytes preserved as bytes.
TEST_F(MsgSendIntrinsicsTest, SendBytesDeliversBinaryEnvelope) {
  const std::string raw("\x00\x01\x02\xff"
                        "bin",
                        7); // arbitrary octets incl. NUL + high byte

  auto sent = call("msg-send", {std::string("msg.bin"), make_bytes(raw)});
  auto *sd = std::get_if<dict_ptr>(&sent.v);
  ASSERT_NE(sd, nullptr);
  // Return contract UNCHANGED: {status, path}, no payload.
  EXPECT_NE(dict_field(*sd, "status"), nullptr);
  EXPECT_NE(dict_field(*sd, "path"), nullptr);
  EXPECT_EQ(dict_field(*sd, "payload"), nullptr);

  // The receiver drains the delivered envelope (park_on_channel returns a pending message).
  auto recv = call("msg-recv", {std::string("msg.bin")});
  auto *rd = std::get_if<dict_ptr>(&recv.v);
  ASSERT_NE(rd, nullptr);

  const value_t *ct = dict_field(*rd, "content_type");
  ASSERT_NE(ct, nullptr);
  EXPECT_EQ(std::get<std::string>(ct->v), "application/octet-stream"); // defaulted for bytes

  const value_t *pl = dict_field(*rd, "payload");
  ASSERT_NE(pl, nullptr);
  auto *bv = std::get_if<bytes_value>(&pl->v);
  ASSERT_NE(bv, nullptr);   // preserved AS BYTES, not stringified
  EXPECT_EQ(bv->data, raw); // exact octets round-trip
}

// A text msg-send still delivers a text envelope (content_type text/plain, payload a string).
TEST_F(MsgSendIntrinsicsTest, SendStringDeliversTextEnvelope) {
  call("msg-send", {std::string("msg.txt"), std::string("hello")});
  auto recv = call("msg-recv", {std::string("msg.txt")});
  auto *rd = std::get_if<dict_ptr>(&recv.v);
  ASSERT_NE(rd, nullptr);
  const value_t *ct = dict_field(*rd, "content_type");
  ASSERT_NE(ct, nullptr);
  EXPECT_EQ(std::get<std::string>(ct->v), "text/plain");
  const value_t *pl = dict_field(*rd, "payload");
  ASSERT_NE(pl, nullptr);
  ASSERT_NE(std::get_if<std::string>(&pl->v), nullptr);
  EXPECT_EQ(std::get<std::string>(pl->v), "hello");
}

TEST_F(MsgSendIntrinsicsTest, WrongArgCount) {
  EXPECT_THROW(call("msg-send", {std::string("only-path")}), std::runtime_error);
}

// ===========================================================================
// MsgRecvIntrinsicsTest
// ===========================================================================

class MsgRecvIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::running;
    proc->message_count = 0;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.proc = proc;
    ictx.pid = 1;

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(MsgRecvIntrinsicsTest, RecvWithInboxReturnsImmediately) {
  // Push a message into the process inbox
  auto msg = make_dict({{"payload", value_t(std::string("hi"))}});
  proc->inbox.push(msg);
  auto result = call("msg-recv", {std::string("any.path")});
  auto *dp = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(dp, nullptr);
  EXPECT_TRUE(proc->inbox.empty());
}

TEST_F(MsgRecvIntrinsicsTest, RecvWithEmptyInboxSuspendsViaScheduler) {
  // Spawn a process in the scheduler so receive_message can find it
  auto pid_val = call("spawn", {std::string("(begin 1 2 3 4 5)")});
  auto pid = std::get<int64_t>(pid_val.v);
  sched.step(); // start the process
  ictx.pid = static_cast<int>(pid);
  auto result = call("msg-recv", {std::string("test.path")});
  EXPECT_TRUE(result.is_nil()); // placeholder before delivery
  auto info = sched.get_process_info(static_cast<int>(pid));
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::waiting);
}

TEST_F(MsgRecvIntrinsicsTest, WrongArgCount) {
  EXPECT_THROW(call("msg-recv", {}), std::runtime_error);
}

// ===========================================================================
// RegisterIntrinsicsTest — verify all expected names exist
// ===========================================================================

class RegisterIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.pid = 1;
    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }
};

TEST_F(RegisterIntrinsicsTest, AllStateIntrinsicsRegistered) {
  for (auto &name : {"state-get", "state-set", "state-children", "state-exists", "state-delete",
                     "state-data-get", "state-data-set", "state-set-tree"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllSchedulerIntrinsicsRegistered) {
  for (auto &name :
       {"spawn", "fork", "self-pid", "self-uid", "kill", "pause", "resume", "ps", "inspect"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllResourceIntrinsicsRegistered) {
  for (auto &name : {"memory-usage", "memory-limit", "time-elapsed", "time-limit", "message-count",
                     "message-limit", "step-count"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllSystemIntrinsicsRegistered) {
  for (auto &name : {"cluster-id", "node-id", "scheduler-stats"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllExpiryIntrinsicsRegistered) {
  for (auto &name : {"state-expire", "state-expire-at", "state-has-expiry", "state-is-expired",
                     "state-clear-expiry", "state-sweep-expired"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllMsgIntrinsicsRegistered) {
  EXPECT_NE(env->lookup("msg-send"), nullptr);
  EXPECT_NE(env->lookup("msg-recv"), nullptr);
  EXPECT_NE(env->lookup("msg-pending"), nullptr);
}

// ===========================================================================
// ResourcePolicyTest — validate_limits()
// ===========================================================================

class ResourcePolicyTest : public ::testing::Test {};

TEST_F(ResourcePolicyTest, NoConstraints) {
  resource_policy policy;
  process_limits req{1000, 60.0, 1024, 50};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 1000u);
  EXPECT_EQ(result.max_time, 60.0);
  EXPECT_EQ(result.max_memory, 1024u);
  EXPECT_EQ(result.max_messages, 50u);
}

TEST_F(ResourcePolicyTest, DefaultsApplied) {
  resource_policy policy;
  policy.max_steps_default = 500;
  policy.max_time_default = 30.0;
  policy.max_memory_default = 2048;
  policy.max_messages_default = 20;

  process_limits req{};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 500u);
  EXPECT_EQ(result.max_time, 30.0);
  EXPECT_EQ(result.max_memory, 2048u);
  EXPECT_EQ(result.max_messages, 20u);
}

TEST_F(ResourcePolicyTest, DefaultsNotOverride) {
  resource_policy policy;
  policy.max_steps_default = 500;
  process_limits req{1000, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 1000u);
}

TEST_F(ResourcePolicyTest, ClampAboveMax) {
  resource_policy policy;
  policy.max_steps_max = 100;
  policy.enforce = resource_policy::mode::clamp;
  process_limits req{200, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 100u);
}

TEST_F(ResourcePolicyTest, ClampBelowMin) {
  resource_policy policy;
  policy.max_steps_min = 50;
  policy.enforce = resource_policy::mode::clamp;
  process_limits req{10, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 50u);
}

TEST_F(ResourcePolicyTest, StrictAboveMaxThrows) {
  resource_policy policy;
  policy.max_steps_max = 100;
  policy.enforce = resource_policy::mode::strict;
  process_limits req{200, 0, 0, 0};
  EXPECT_THROW(validate_limits(policy, req), std::runtime_error);
}

TEST_F(ResourcePolicyTest, StrictBelowMinThrows) {
  resource_policy policy;
  policy.max_steps_min = 50;
  policy.enforce = resource_policy::mode::strict;
  process_limits req{10, 0, 0, 0};
  EXPECT_THROW(validate_limits(policy, req), std::runtime_error);
}

TEST_F(ResourcePolicyTest, StrictWithinRange) {
  resource_policy policy;
  policy.max_steps_min = 50;
  policy.max_steps_max = 200;
  policy.enforce = resource_policy::mode::strict;
  process_limits req{100, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 100u);
}

TEST_F(ResourcePolicyTest, WarnModeClampsLikeClamp) {
  resource_policy policy;
  policy.max_time_max = 30.0;
  policy.enforce = resource_policy::mode::warn;
  process_limits req{0, 60.0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_time, 30.0);
}

TEST_F(ResourcePolicyTest, AllFieldsClamped) {
  resource_policy policy;
  policy.max_steps_max = 100;
  policy.max_time_max = 10.0;
  policy.max_memory_max = 500;
  policy.max_messages_max = 20;
  policy.enforce = resource_policy::mode::clamp;

  process_limits req{999, 99.0, 999, 999};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 100u);
  EXPECT_EQ(result.max_time, 10.0);
  EXPECT_EQ(result.max_memory, 500u);
  EXPECT_EQ(result.max_messages, 20u);
}

TEST_F(ResourcePolicyTest, ZeroValueNotClamped) {
  // A value of 0 means "unlimited" and should not be clamped
  resource_policy policy;
  policy.max_steps_min = 50;
  policy.max_steps_max = 200;
  policy.enforce = resource_policy::mode::clamp;
  process_limits req{0, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 0u);
}

// ===========================================================================
// StdlibRegistryTest — module registration and lookup
// ===========================================================================

class StdlibRegistryTest : public ::testing::Test {
protected:
  stdlib_registry registry;
};

TEST_F(StdlibRegistryTest, BuiltinModulesExist) {
  auto mods = registry.list_modules();
  EXPECT_GE(mods.size(), 3u);
  auto has = [&](const std::string &m) {
    return std::find(mods.begin(), mods.end(), m) != mods.end();
  };
  EXPECT_TRUE(has("string"));
  EXPECT_TRUE(has("math"));
  EXPECT_TRUE(has("collections"));
}

TEST_F(StdlibRegistryTest, StringModuleFunctions) {
  auto fns = registry.list_functions("string");
  EXPECT_GE(fns.size(), 10u);
  auto has = [&](const std::string &f) {
    return std::find(fns.begin(), fns.end(), f) != fns.end();
  };
  EXPECT_TRUE(has("string.split"));
  EXPECT_TRUE(has("string.join"));
  EXPECT_TRUE(has("string.replace"));
  EXPECT_TRUE(has("string.trim"));
  EXPECT_TRUE(has("string.upper"));
  EXPECT_TRUE(has("string.lower"));
  EXPECT_TRUE(has("string.starts-with"));
  EXPECT_TRUE(has("string.ends-with"));
  EXPECT_TRUE(has("string.contains"));
  EXPECT_TRUE(has("string.substring"));
  EXPECT_TRUE(has("string.char-at"));
  EXPECT_TRUE(has("string.length"));
}

TEST_F(StdlibRegistryTest, MathModuleFunctions) {
  auto fns = registry.list_functions("math");
  EXPECT_GE(fns.size(), 14u);
}

TEST_F(StdlibRegistryTest, CollectionsModuleFunctions) {
  auto fns = registry.list_functions("collections");
  EXPECT_GE(fns.size(), 10u);
}

TEST_F(StdlibRegistryTest, ListEmpty) {
  auto fns = registry.list_functions("nonexistent");
  EXPECT_TRUE(fns.empty());
}

// A bare environment: a default one already reaches every stdlib function through the shared
// stdlib scope, so it can't show what import_module() adds.
TEST_F(StdlibRegistryTest, ImportAll) {
  auto env = std::make_shared<environment>();
  registry.import_module("string", env);
  EXPECT_NE(env->lookup("string.split"), nullptr);
  EXPECT_NE(env->lookup("string.join"), nullptr);
  EXPECT_NE(env->lookup("string.upper"), nullptr);
}

TEST_F(StdlibRegistryTest, ImportSpecific) {
  auto env = std::make_shared<environment>();
  registry.import_module("math", env, {"math.sqrt", "math.abs"});
  EXPECT_NE(env->lookup("math.sqrt"), nullptr);
  EXPECT_NE(env->lookup("math.abs"), nullptr);
  // Not imported
  EXPECT_EQ(env->lookup("math.sin"), nullptr);
}

TEST_F(StdlibRegistryTest, ImportUnknownModuleThrows) {
  auto env = builtins::make_default_environment();
  EXPECT_THROW(registry.import_module("bogus", env), std::runtime_error);
}

TEST_F(StdlibRegistryTest, ImportUnknownFunctionThrows) {
  auto env = builtins::make_default_environment();
  EXPECT_THROW(registry.import_module("string", env, {"no.such.fn"}), std::runtime_error);
}

TEST_F(StdlibRegistryTest, LookupQualified) {
  auto *fn = registry.lookup_qualified("string.split");
  EXPECT_NE(fn, nullptr);
}

TEST_F(StdlibRegistryTest, LookupQualifiedNotFound) {
  auto *fn = registry.lookup_qualified("nonexistent.fn");
  EXPECT_EQ(fn, nullptr);
}

TEST_F(StdlibRegistryTest, RegisterCustomFunction) {
  registry.register_function("custom", "custom.noop",
                             [](std::span<const value_t>) -> value_t { return nil_value; });
  auto mods = registry.list_modules();
  auto has = std::find(mods.begin(), mods.end(), "custom") != mods.end();
  EXPECT_TRUE(has);
  auto fns = registry.list_functions("custom");
  EXPECT_EQ(fns.size(), 1u);
}

// ===========================================================================
// StdlibStringTest — individual string functions
// ===========================================================================

class StdlibStringTest : public ::testing::Test {
protected:
  stdlib_registry registry;
  environment_ptr env;

  void SetUp() override {
    env = builtins::make_default_environment();
    registry.import_module("string", env);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    EXPECT_NE(fn_val, nullptr) << "Function not found: " << name;
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(StdlibStringTest, Split) {
  auto result = call("string.split", {std::string("a,b,c"), std::string(",")});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 3u);
  EXPECT_EQ(std::get<std::string>((**lst)[0].v), "a");
  EXPECT_EQ(std::get<std::string>((**lst)[1].v), "b");
  EXPECT_EQ(std::get<std::string>((**lst)[2].v), "c");
}

TEST_F(StdlibStringTest, SplitDefault) {
  auto result = call("string.split", {std::string("a b c")});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 3u);
}

TEST_F(StdlibStringTest, Join) {
  auto lst = make_list({std::string("x"), std::string("y"), std::string("z")});
  auto result = call("string.join", {lst, std::string("-")});
  EXPECT_EQ(std::get<std::string>(result.v), "x-y-z");
}

TEST_F(StdlibStringTest, Replace) {
  auto result = call("string.replace",
                     {std::string("hello world"), std::string("world"), std::string("there")});
  EXPECT_EQ(std::get<std::string>(result.v), "hello there");
}

TEST_F(StdlibStringTest, Trim) {
  auto result = call("string.trim", {std::string("  hello  ")});
  EXPECT_EQ(std::get<std::string>(result.v), "hello");
}

TEST_F(StdlibStringTest, TrimAllWhitespace) {
  auto result = call("string.trim", {std::string("   ")});
  EXPECT_EQ(std::get<std::string>(result.v), "");
}

TEST_F(StdlibStringTest, Upper) {
  auto result = call("string.upper", {std::string("hello")});
  EXPECT_EQ(std::get<std::string>(result.v), "HELLO");
}

TEST_F(StdlibStringTest, Lower) {
  auto result = call("string.lower", {std::string("HELLO")});
  EXPECT_EQ(std::get<std::string>(result.v), "hello");
}

TEST_F(StdlibStringTest, StartsWith) {
  auto yes = call("string.starts-with", {std::string("hello"), std::string("he")});
  auto no = call("string.starts-with", {std::string("hello"), std::string("lo")});
  EXPECT_TRUE(std::get<bool>(yes.v));
  EXPECT_FALSE(std::get<bool>(no.v));
}

TEST_F(StdlibStringTest, EndsWith) {
  auto yes = call("string.ends-with", {std::string("hello"), std::string("lo")});
  auto no = call("string.ends-with", {std::string("hello"), std::string("he")});
  EXPECT_TRUE(std::get<bool>(yes.v));
  EXPECT_FALSE(std::get<bool>(no.v));
}

TEST_F(StdlibStringTest, Contains) {
  auto yes = call("string.contains", {std::string("hello world"), std::string("lo w")});
  auto no = call("string.contains", {std::string("hello"), std::string("xyz")});
  EXPECT_TRUE(std::get<bool>(yes.v));
  EXPECT_FALSE(std::get<bool>(no.v));
}

TEST_F(StdlibStringTest, Substring) {
  auto result = call("string.substring", {std::string("hello"), int64_t(1), int64_t(3)});
  EXPECT_EQ(std::get<std::string>(result.v), "ell");
}

TEST_F(StdlibStringTest, SubstringNoLen) {
  auto result = call("string.substring", {std::string("hello"), int64_t(2)});
  EXPECT_EQ(std::get<std::string>(result.v), "llo");
}

TEST_F(StdlibStringTest, CharAt) {
  auto result = call("string.char-at", {std::string("hello"), int64_t(1)});
  EXPECT_EQ(std::get<std::string>(result.v), "e");
}

TEST_F(StdlibStringTest, CharAtOutOfRange) {
  EXPECT_THROW(call("string.char-at", {std::string("hi"), int64_t(5)}), std::runtime_error);
}

TEST_F(StdlibStringTest, Length) {
  auto result = call("string.length", {std::string("hello")});
  EXPECT_EQ(std::get<int64_t>(result.v), 5);
}

// ===========================================================================
// StdlibMathTest — individual math functions
// ===========================================================================

class StdlibMathTest : public ::testing::Test {
protected:
  stdlib_registry registry;
  environment_ptr env;

  void SetUp() override {
    env = builtins::make_default_environment();
    registry.import_module("math", env);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(StdlibMathTest, Sqrt) {
  auto result = call("math.sqrt", {16.0});
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), 4.0);
}

TEST_F(StdlibMathTest, Abs) {
  auto d = call("math.abs", {-3.5});
  EXPECT_DOUBLE_EQ(std::get<double>(d.v), 3.5);
  auto i = call("math.abs", {int64_t(-7)});
  EXPECT_EQ(std::get<int64_t>(i.v), 7);
}

TEST_F(StdlibMathTest, Floor) {
  auto result = call("math.floor", {3.7});
  EXPECT_EQ(std::get<int64_t>(result.v), 3);
}

TEST_F(StdlibMathTest, Ceil) {
  auto result = call("math.ceil", {3.2});
  EXPECT_EQ(std::get<int64_t>(result.v), 4);
}

TEST_F(StdlibMathTest, Round) {
  auto result = call("math.round", {3.5});
  EXPECT_EQ(std::get<int64_t>(result.v), 4);
}

TEST_F(StdlibMathTest, Pow) {
  auto result = call("math.pow", {2.0, 10.0});
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), 1024.0);
}

TEST_F(StdlibMathTest, Log) {
  auto result = call("math.log", {1.0});
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), 0.0);
}

TEST_F(StdlibMathTest, Trig) {
  auto s = call("math.sin", {0.0});
  auto c = call("math.cos", {0.0});
  auto t = call("math.tan", {0.0});
  EXPECT_NEAR(std::get<double>(s.v), 0.0, 1e-10);
  EXPECT_NEAR(std::get<double>(c.v), 1.0, 1e-10);
  EXPECT_NEAR(std::get<double>(t.v), 0.0, 1e-10);
}

TEST_F(StdlibMathTest, MinMax) {
  auto mn = call("math.min", {3.0, 1.0, 2.0});
  auto mx = call("math.max", {3.0, 1.0, 2.0});
  EXPECT_DOUBLE_EQ(std::get<double>(mn.v), 1.0);
  EXPECT_DOUBLE_EQ(std::get<double>(mx.v), 3.0);
}

TEST_F(StdlibMathTest, Clamp) {
  auto below = call("math.clamp", {-1.0, 0.0, 10.0});
  auto above = call("math.clamp", {15.0, 0.0, 10.0});
  auto inside = call("math.clamp", {5.0, 0.0, 10.0});
  EXPECT_DOUBLE_EQ(std::get<double>(below.v), 0.0);
  EXPECT_DOUBLE_EQ(std::get<double>(above.v), 10.0);
  EXPECT_DOUBLE_EQ(std::get<double>(inside.v), 5.0);
}

TEST_F(StdlibMathTest, Pi) {
  auto result = call("math.pi", {});
  EXPECT_NEAR(std::get<double>(result.v), 3.14159265, 1e-6);
}

TEST_F(StdlibMathTest, E) {
  auto result = call("math.e", {});
  EXPECT_NEAR(std::get<double>(result.v), 2.71828182, 1e-6);
}

TEST_F(StdlibMathTest, Random) {
  auto result = call("math.random", {});
  double val = std::get<double>(result.v);
  EXPECT_GE(val, 0.0);
  EXPECT_LT(val, 1.0);
}

// ===========================================================================
// StdlibCollectionsTest — individual collections functions
// ===========================================================================

class StdlibCollectionsTest : public ::testing::Test {
protected:
  stdlib_registry registry;
  environment_ptr env;

  void SetUp() override {
    env = builtins::make_default_environment();
    registry.import_module("collections", env);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(StdlibCollectionsTest, MapWithNative) {
  native_fn double_it = [](std::span<const value_t> args) -> value_t {
    return value_t(std::get<int64_t>(args[0].v) * 2);
  };
  auto lst = make_list({int64_t(1), int64_t(2), int64_t(3)});
  auto result = call("collections.map", {lst, value_t(double_it)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 3u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 2);
  EXPECT_EQ(std::get<int64_t>((**r)[1].v), 4);
  EXPECT_EQ(std::get<int64_t>((**r)[2].v), 6);
}

TEST_F(StdlibCollectionsTest, FilterWithNative) {
  native_fn is_positive = [](std::span<const value_t> args) -> value_t {
    return value_t(std::get<int64_t>(args[0].v) > 0);
  };
  auto lst = make_list({int64_t(-1), int64_t(2), int64_t(-3), int64_t(4)});
  auto result = call("collections.filter", {lst, value_t(is_positive)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 2u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 2);
  EXPECT_EQ(std::get<int64_t>((**r)[1].v), 4);
}

TEST_F(StdlibCollectionsTest, ReduceSum) {
  native_fn add = [](std::span<const value_t> args) -> value_t {
    return value_t(std::get<int64_t>(args[0].v) + std::get<int64_t>(args[1].v));
  };
  auto lst = make_list({int64_t(1), int64_t(2), int64_t(3)});
  auto result = call("collections.reduce", {lst, value_t(add), int64_t(0)});
  EXPECT_EQ(std::get<int64_t>(result.v), 6);
}

TEST_F(StdlibCollectionsTest, Zip) {
  auto a = make_list({int64_t(1), int64_t(2)});
  auto b = make_list({std::string("a"), std::string("b")});
  auto result = call("collections.zip", {a, b});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 2u);
}

TEST_F(StdlibCollectionsTest, Flatten) {
  auto inner = make_list({int64_t(2), int64_t(3)});
  auto lst = make_list({int64_t(1), inner, int64_t(4)});
  auto result = call("collections.flatten", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 4u);
}

TEST_F(StdlibCollectionsTest, Sort) {
  auto lst = make_list({int64_t(3), int64_t(1), int64_t(2)});
  auto result = call("collections.sort", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 3u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 1);
  EXPECT_EQ(std::get<int64_t>((**r)[1].v), 2);
  EXPECT_EQ(std::get<int64_t>((**r)[2].v), 3);
}

TEST_F(StdlibCollectionsTest, SortStrings) {
  auto lst = make_list({std::string("c"), std::string("a"), std::string("b")});
  auto result = call("collections.sort", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(std::get<std::string>((**r)[0].v), "a");
}

TEST_F(StdlibCollectionsTest, Reverse) {
  auto lst = make_list({int64_t(1), int64_t(2), int64_t(3)});
  auto result = call("collections.reverse", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 3);
  EXPECT_EQ(std::get<int64_t>((**r)[2].v), 1);
}

TEST_F(StdlibCollectionsTest, Range) {
  auto result = call("collections.range", {int64_t(5)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 5u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 0);
  EXPECT_EQ(std::get<int64_t>((**r)[4].v), 4);
}

TEST_F(StdlibCollectionsTest, RangeStartStop) {
  auto result = call("collections.range", {int64_t(2), int64_t(5)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 3u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 2);
}

TEST_F(StdlibCollectionsTest, RangeStep) {
  auto result = call("collections.range", {int64_t(0), int64_t(10), int64_t(3)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 4u);
  EXPECT_EQ(std::get<int64_t>((**r)[3].v), 9);
}

TEST_F(StdlibCollectionsTest, RangeZeroStepThrows) {
  EXPECT_THROW(call("collections.range", {int64_t(0), int64_t(5), int64_t(0)}), std::runtime_error);
}

TEST_F(StdlibCollectionsTest, Unique) {
  auto lst = make_list({int64_t(1), int64_t(2), int64_t(1), int64_t(3), int64_t(2)});
  auto result = call("collections.unique", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ((*r)->size(), 3u);
}

TEST_F(StdlibCollectionsTest, DictKeys) {
  auto d = make_dict({{"a", int64_t(1)}, {"b", int64_t(2)}});
  auto result = call("collections.dict-keys", {d});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ((*r)->size(), 2u);
}

TEST_F(StdlibCollectionsTest, DictValues) {
  auto d = make_dict({{"a", int64_t(1)}, {"b", int64_t(2)}});
  auto result = call("collections.dict-values", {d});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ((*r)->size(), 2u);
}

TEST_F(StdlibCollectionsTest, DictMerge) {
  auto d1 = make_dict({{"a", int64_t(1)}, {"b", int64_t(2)}});
  auto d2 = make_dict({{"b", int64_t(99)}, {"c", int64_t(3)}});
  auto result = call("collections.dict-merge", {d1, d2});
  auto *r = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  // b should be updated to 99, c should be added
  EXPECT_EQ((*r)->size(), 3u);
  bool found_b = false, found_c = false;
  for (auto &[k, v] : **r) {
    if (k == "b") {
      EXPECT_EQ(std::get<int64_t>(v.v), 99);
      found_b = true;
    }
    if (k == "c") {
      EXPECT_EQ(std::get<int64_t>(v.v), 3);
      found_c = true;
    }
  }
  EXPECT_TRUE(found_b);
  EXPECT_TRUE(found_c);
}

TEST_F(StdlibCollectionsTest, DictKeysNotDict) {
  EXPECT_THROW(call("collections.dict-keys", {int64_t(42)}), std::runtime_error);
}

// ===========================================================================
// IntrinsicsViaEvaluatorTest — test intrinsics end-to-end through DSL eval
// ===========================================================================

class IntrinsicsViaEvaluatorTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 42;
    proc->status = process_status::running;
    proc->max_memory = 1024;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 42;
    ictx.uid = "eval-user";
    ictx.cluster_id = "test-cluster";
    ictx.node_id = "test-node";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t eval(const std::string &code) {
    evaluator e(env);
    return e.evaluate_script(code);
  }
};

TEST_F(IntrinsicsViaEvaluatorTest, SelfPidViaDSL) {
  auto result = eval("(self-pid)");
  EXPECT_EQ(std::get<int64_t>(result.v), 42);
}

TEST_F(IntrinsicsViaEvaluatorTest, SelfUidViaDSL) {
  auto result = eval("(self-uid)");
  EXPECT_EQ(std::get<std::string>(result.v), "eval-user");
}

TEST_F(IntrinsicsViaEvaluatorTest, ClusterIdViaDSL) {
  auto result = eval("(cluster-id)");
  EXPECT_EQ(std::get<std::string>(result.v), "test-cluster");
}

TEST_F(IntrinsicsViaEvaluatorTest, StateSetGetViaDSL) {
  eval("(state-set \"dsl.key\" \"dsl-value\")");
  auto result = eval("(state-get \"dsl.key\")");
  EXPECT_EQ(std::get<std::string>(result.v), "dsl-value");
}

TEST_F(IntrinsicsViaEvaluatorTest, StateExistsViaDSL) {
  eval("(state-set \"dsl.exists\" \"yes\")");
  auto result = eval("(state-exists \"dsl.exists\")");
  EXPECT_TRUE(std::get<bool>(result.v));
}

TEST_F(IntrinsicsViaEvaluatorTest, MemoryLimitViaDSL) {
  auto result = eval("(memory-limit)");
  EXPECT_EQ(std::get<int64_t>(result.v), 1024);
}

TEST_F(IntrinsicsViaEvaluatorTest, SpawnViaDSL) {
  auto result = eval("(spawn \"(+ 1 2)\")");
  EXPECT_GT(std::get<int64_t>(result.v), 0);
}

// ===========================================================================
// StdlibViaEvaluatorTest — test stdlib through DSL
// ===========================================================================

class StdlibViaEvaluatorTest : public ::testing::Test {
protected:
  stdlib_registry registry;
  environment_ptr env;

  void SetUp() override {
    env = builtins::make_default_environment();
    registry.import_module("string", env);
    registry.import_module("math", env);
    registry.import_module("collections", env);
  }

  value_t eval(const std::string &code) {
    evaluator e(env);
    return e.evaluate_script(code);
  }
};

TEST_F(StdlibViaEvaluatorTest, StringUpperViaDSL) {
  auto result = eval("(string.upper \"hello\")");
  EXPECT_EQ(std::get<std::string>(result.v), "HELLO");
}

TEST_F(StdlibViaEvaluatorTest, MathSqrtViaDSL) {
  auto result = eval("(math.sqrt 25)");
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), 5.0);
}

TEST_F(StdlibViaEvaluatorTest, MathPiViaDSL) {
  auto result = eval("(math.pi)");
  EXPECT_NEAR(std::get<double>(result.v), 3.14159265, 1e-6);
}

TEST_F(StdlibViaEvaluatorTest, StringSplitAndLength) {
  auto result = eval("(string.length \"hello world\")");
  EXPECT_EQ(std::get<int64_t>(result.v), 11);
}

TEST_F(StdlibViaEvaluatorTest, CollectionsRangeViaDSL) {
  auto result = eval("(collections.range 3)");
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  EXPECT_EQ((*lst)->size(), 3u);
}

// ===========================================================================
// §12 message-channel scoping — the pure-string key rule (resolve_channel_key).
// ===========================================================================
// resolve_channel_key mirrors state's chroot for message channels. The intrinsic-side resolver
// (resolve_channel, file-local) adds transparent-link grant-following on top of these rules; that
// path is covered end-to-end by ChannelScopeIntegrationTest in state_exec_integration_test.cpp.

TEST(ChannelScopeKey, RootPrefixIsIdentity) {
  // Empty chroot -> the channel is used verbatim. This is the backward-compat guarantee that keeps
  // every existing raw-key msg-* caller (all use prefix "") green.
  EXPECT_EQ(resolve_channel_key("", "nav.done"), "nav.done");
}

TEST(ChannelScopeKey, PrivateByPrefix) {
  // A chrooted document's channel lives under its own reserved channels. subtree.
  EXPECT_EQ(resolve_channel_key("ui.docs.a", "nav.done"), "ui.docs.a.channels.nav.done");
}

TEST(ChannelScopeKey, DistinctPrefixesDoNotCollide) {
  // Two documents using the SAME channel name resolve to distinct keys -> no cross-talk. Isolation
  // is the inability to name another scope's key, exactly as state chroot gets it.
  EXPECT_NE(resolve_channel_key("ui.docs.a", "sig"), resolve_channel_key("ui.docs.b", "sig"));
}

TEST(ChannelScopeKey, SlashIsAppRootGlobalEscape) {
  // A leading '/' is the explicit app-root-global escape (the '/' is stripped). Whether a chrooted
  // document MAY use it is a loader-enforcement concern (PR-B), not this mechanical resolver.
  EXPECT_EQ(resolve_channel_key("ui.docs.a", "/app.quit"), "app.quit");
}

TEST(ChannelScopeKey, HashChannelsAreRuntimeGlobalExempt) {
  // The runtime's own owner#tick / #key / #pointer channels contain '#' and are never scoped, so
  // the input/tick residents keep working unchanged under any prefix.
  EXPECT_EQ(resolve_channel_key("ui.docs.a", "ari#7:ui.docs.a#tick"), "ari#7:ui.docs.a#tick");
}

// ===========================================================================
// SpawnContext — what a spawned child sees (td 4.15; locks plan L1b.1)
// ===========================================================================

// A host context chrooted at "doc" on a synchronous scheduler. Each test sets the context's fields
// (its spawn_context among them), then run() registers the intrinsics and runs a parent and its
// children to the end.
class SpawnContext : public ::testing::Test {
protected:
  cvc::app app_ctx;
  cvc::state &tree = cvc::state::instance(app_ctx);
  scheduler sched;
  memory_tracker tracker;
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    sched.set_watch_root(&tree);
    ictx.sched = &sched;
    ictx.tracker = &tracker;
    ictx.proc = make_process();
    ictx.uid = "parent";
    apply_chroot(ictx, tree, "doc");
  }

  // The parent's pid; its result is its last child's pid.
  int run(const std::string &script, execute_options opts = {}) {
    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
    opts.env = env;
    const int pid = sched.execute(script, opts);
    sched.run(std::nullopt, 10.0);
    return pid;
  }

  int child_of(int parent) {
    const std::optional<value_t> r = sched.get_result(parent);
    EXPECT_TRUE(r.has_value()) << sched.get_exit_error(parent).value_or("no result");
    const auto *pid = r ? std::get_if<int64_t>(&r->v) : nullptr;
    EXPECT_NE(pid, nullptr);
    return pid ? static_cast<int>(*pid) : -1;
  }

  // `inner`, noting each child context it makes.
  std::function<spawn_setup(const intrinsics_context &)>
  watching(std::function<spawn_setup(const intrinsics_context &)> inner,
           std::vector<std::weak_ptr<void>> &made) {
    return [inner = std::move(inner), &made](const intrinsics_context &parent) {
      spawn_setup s = inner(parent);
      made.push_back(s.context);
      return s;
    };
  }
};

TEST_F(SpawnContext, NoCallbackKeepsTheBareEnv) {
  const int child = child_of(run(R"S((spawn "(state-get \"x\")"))S"));
  const std::string err = sched.get_exit_error(child).value_or("");
  EXPECT_NE(err.find("NameError"), std::string::npos) << err;
}

TEST_F(SpawnContext, DefaultGivesIntrinsicsAndChroot) {
  ictx.spawn_context = make_spawn_context();
  const int child =
      child_of(run(R"S((spawn "(begin (state-set \"x\" \"1\") (msg-send \"c\" \"hi\") 7)"))S"));
  ASSERT_TRUE(sched.get_result(child).has_value()) << sched.get_exit_error(child).value_or("");
  EXPECT_EQ(std::get<int64_t>(sched.get_result(child)->v), 7);
  EXPECT_EQ(tree("doc.x").value(), "1");
  EXPECT_EQ(tree.findDescendant("x"), nullptr);
  // The parent's channel key: chroot-scoped, as the parent's own msg-recv would wait on.
  EXPECT_EQ(sched.pending_message_count(resolve_channel_key("doc", "c")), 1u);
}

TEST_F(SpawnContext, ChildKeepsOwner) {
  // On the async scheduler, whose kill_owner a host reaps a document with. No callback: the owner
  // is the parent's either way.
  async_scheduler async;
  ictx.sched = &async;
  env = builtins::make_default_environment();
  register_intrinsics(env, &ictx);
  execute_options opts;
  opts.env = env;
  opts.owner = "doc-1";
  const int parent = async.execute(std::string(R"S((spawn "(while t (sleep 0))"))S"), opts);
  for (int i = 0; i < 1000 && !is_finished(async.get_process_info(parent)->status); ++i) {
    async.drain_ingress();
    async.sync_step();
  }
  const std::optional<value_t> r = async.get_result(parent);
  ASSERT_TRUE(r.has_value());
  const int child = static_cast<int>(std::get<int64_t>(r->v));
  ASSERT_FALSE(is_finished(async.get_process_info(child)->status));
  EXPECT_GE(async.kill_owner("doc-1"), 1);
  EXPECT_EQ(async.get_process_info(child)->status, process_status::killed);
}

TEST_F(SpawnContext, ContextLivesWithTheProcess) {
  std::vector<std::weak_ptr<void>> made;
  ictx.spawn_context = watching(make_spawn_context(), made);
  const int child = child_of(run(R"S((spawn "(+ 1 2)"))S"));
  ASSERT_EQ(made.size(), 1u);
  ASSERT_TRUE(is_finished(sched.get_process_info(child)->status));
  EXPECT_FALSE(made[0].expired()); // finished, not reaped: the process object keeps it
  ASSERT_TRUE(sched.reap(child));
  EXPECT_TRUE(made[0].expired());
}

TEST_F(SpawnContext, GrandchildrenGetOne) {
  ictx.spawn_context = make_spawn_context();
  run(R"S((spawn "(spawn \"(state-set \\\"g\\\" \\\"1\\\")\")"))S");
  EXPECT_EQ(tree("doc.g").value(), "1");
}

TEST_F(SpawnContext, CopiesLockFields) {
  ictx.lock_wait_allowed = false;
  ictx.lock_default_timeout = 0.0;
  ictx.cluster_id = "c1";
  std::vector<std::weak_ptr<void>> made;
  ictx.spawn_context = watching(make_spawn_context(), made);
  const int child = child_of(run(R"S((spawn "(+ 1 2)"))S"));
  ASSERT_EQ(made.size(), 1u);
  const auto c = std::static_pointer_cast<intrinsics_context>(made[0].lock());
  ASSERT_NE(c, nullptr);
  EXPECT_FALSE(c->lock_wait_allowed);
  ASSERT_TRUE(c->lock_default_timeout.has_value());
  EXPECT_EQ(*c->lock_default_timeout, 0.0);
  EXPECT_EQ(c->sched, &sched);
  EXPECT_EQ(c->root, ictx.root);
  EXPECT_EQ(c->root_path, "doc");
  EXPECT_EQ(c->uid, "parent");
  EXPECT_EQ(c->cluster_id, "c1");
  EXPECT_TRUE(static_cast<bool>(c->spawn_context));
  EXPECT_NE(c->proc, nullptr);
  EXPECT_NE(c->proc, ictx.proc);
  (void)child;
}

TEST_F(SpawnContext, AfterRegisterRuns) {
  ictx.spawn_context =
      make_spawn_context(nullptr, [](const environment_ptr &e, intrinsics_context &c) {
        EXPECT_EQ(c.root_path, "doc");
        builtins::register_fn(e, "host-thing",
                              [](std::span<const value_t>) { return value_t(int64_t{5}); });
        // After the intrinsics, so it overrides one.
        builtins::register_fn(
            e, "state-get", [](std::span<const value_t>) { return value_t(std::string("host")); });
      });
  const int child = child_of(run(R"S((spawn "(list (host-thing) (state-get \"x\"))"))S"));
  const std::optional<value_t> r = sched.get_result(child);
  ASSERT_TRUE(r.has_value()) << sched.get_exit_error(child).value_or("");
  const auto *l = std::get_if<list_ptr>(&r->v);
  ASSERT_NE(l, nullptr);
  ASSERT_EQ((*l)->size(), 2u);
  EXPECT_EQ(std::get<int64_t>((**l)[0].v), 5);
  EXPECT_EQ(std::get<std::string>((**l)[1].v), "host");
}
