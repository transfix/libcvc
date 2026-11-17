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
*/

// The document a relay runs (cvc/ariadne/server_document.h, arisrv --doc): its server: block, the
// namespaces it publishes through holes in its chroot, the room it reads, what a relay's shard
// scoped by it sends, and the programs and guests its clients send it to run (client_programs:,
// client_guests:, the latter with a fake engine here).
// room_relay_integration_test.cpp has the same over a relay's transports.

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cvc/ariadne/script_engine.h> // a client guest's engine (client_guests:)
#include <cvc/ariadne/server_document.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_exec/exceptions.h> // dsl_error
#include <cvc/state/state_exec/types.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <initializer_list>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace ari = cvc::ariadne;

namespace {

constexpr const char *kRoom = "standup";

// A fresh directory, removed on scope-out.
struct TempDir {
  fs::path path;
  TempDir() {
    std::random_device rd;
    path = fs::temp_directory_path() /
           ("srvdoc-test-" + std::to_string(rd()) + "-" + std::to_string(rd()));
    fs::create_directories(path);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
  std::string write(const std::string &name, const std::string &text) const {
    std::ofstream(path / name, std::ios::binary) << text;
    return (path / name).string();
  }
};

ari::Value scalar(const std::string &s) {
  ari::Value v;
  v.scalar = s;
  return v;
}

ari::Value seq(const std::vector<std::string> &items) {
  ari::Value v;
  v.kind = ari::Value::Kind::Sequence;
  for (const std::string &s : items)
    v.items.push_back(scalar(s));
  return v;
}

ari::Value map(std::vector<ari::Value::Entry> entries) {
  ari::Value v;
  v.kind = ari::Value::Kind::Map;
  v.entries = std::move(entries);
  return v;
}

std::string value_at(cvc::app &app, const std::string &path) {
  cvc::state *n = cvc::state::instance(app).findDescendant(path);
  return n ? n->value() : std::string("<absent>");
}

// A server document that publishes `echo`, writes it and its own state at init:, keeps a tick
// count in echo.ticks, and reads the room's topic through its room view.
const char *const kEchoDoc = R"(meta: { name: echo server, min_libcvc: 3.7.5 }
server:
  publish: [echo]
init: |
  (state-set "echo.status" "up")
  (state-set "own.note" "the document's own")
  (state-set-tree "echo.jobs" (dict "a" 1 "b" 2))
  (state-set "seen_topic" (state-get "room.topic"))
  (state-set "room.topic" "rewritten by the document")
on_tick: (state-incr "echo.ticks")
)";

} // namespace

TEST(ServerBlockTest, PublishTakesAListOrOneName) {
  ari::ServerBlock b;
  std::vector<std::string> w;
  EXPECT_EQ(
      ari::parse_server_block(map({{"publish", seq({"cyprov", "apps.board", "cyprov"})}}), b, &w),
      "");
  EXPECT_TRUE(b.present);
  EXPECT_EQ(b.publish, (std::vector<std::string>{"cyprov", "apps.board"})) << "repeats kept once";
  EXPECT_TRUE(w.empty());

  EXPECT_EQ(ari::parse_server_block(map({{"publish", scalar("cyprov")}}), b), "");
  EXPECT_EQ(b.publish, (std::vector<std::string>{"cyprov"}));

  EXPECT_EQ(ari::parse_server_block(scalar(""), b), "") << "a bare server: publishes nothing";
  EXPECT_TRUE(b.present);
  EXPECT_TRUE(b.publish.empty());

  EXPECT_EQ(ari::parse_server_block(map({{"publish", seq({"x"})}, {"listen", scalar("1")}}), b, &w),
            "");
  ASSERT_EQ(w.size(), 1u);
  EXPECT_NE(w[0].find("unknown key 'listen'"), std::string::npos) << w[0];
}

TEST(ServerBlockTest, RefusesWhatItCannotPublish) {
  ari::ServerBlock b;
  const auto error_for = [&](const std::vector<std::string> &names) {
    return ari::parse_server_block(map({{"publish", seq(names)}}), b);
  };
  // The relay's own roots.
  EXPECT_NE(error_for({"conf"}).find("the relay's room tree"), std::string::npos);
  EXPECT_NE(error_for({"conf.rooms.standup.extra"}).find("under conf"), std::string::npos);
  EXPECT_NE(error_for({"ui.docs.arisrv.own"}).find("under ui"), std::string::npos);
  EXPECT_NE(error_for({"ariadne.limits"}).find("the host's settings"), std::string::npos);
  EXPECT_NE(error_for({"state_exec"}).find("the host's settings"), std::string::npos);
  EXPECT_NE(error_for({"room"}).find("view of the relay's room"), std::string::npos);
  // Names that are not dotted identifiers.
  for (const char *bad : {"9lives", "a..b", "a b", ".a", "a.", "a-b"})
    EXPECT_NE(error_for({bad}).find("not a dotted path of identifiers"), std::string::npos) << bad;
  EXPECT_NE(error_for({""}).find("empty"), std::string::npos);
  // One namespace inside another.
  EXPECT_NE(error_for({"cyprov", "cyprov.jobs"}).find("overlap"), std::string::npos);
  EXPECT_NE(error_for({"cyprov.jobs", "cyprov"}).find("overlap"), std::string::npos);
  EXPECT_EQ(error_for({"cyprov", "cyprovx"}), "") << "a dot-segment boundary, not an overlap";
  // Not a list of names.
  EXPECT_NE(ari::parse_server_block(seq({"cyprov"}), b).find("expects a map"), std::string::npos);
  EXPECT_NE(
      ari::parse_server_block(map({{"publish", map({{"cyprov", scalar("1")}})}}), b).find("list"),
      std::string::npos);
}

// The whole round: read the server: block, scope a relay's shard by it, load the document, run a
// few relay steps. What the document writes under echo lands on the app tree's echo (through the
// hole), its own state stays under its prefix, the room is read-only to it, and the shard sends
// echo and the room tree but nothing of the document's own.
TEST(ServerDocumentTest, PublishesThroughItsHolesAndKeepsItsOwnState) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("server.ari", kEchoDoc);
  cvc::app app;
  cvc::state::instance(app)("conf.rooms.standup.topic").value(std::string("standup at ten"));

  cvc::state_cluster_shard shard(app, "conf", "server");
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  EXPECT_EQ(doc.block().publish, (std::vector<std::string>{"echo"}));
  doc.scope(shard);
  EXPECT_EQ(shard.publish_prefixes(), (std::vector<std::string>{"conf", "echo"}));
  shard.attach();

  const std::vector<std::string> warnings = doc.load(path);
  for (const std::string &w : warnings)
    ADD_FAILURE() << "warning: " << w;
  for (int i = 0; i < 6; ++i)
    doc.step(240 + i, (240 + i) / 120.0); // the relay's world clock, at 120 Hz

  // Published, at the app tree's own paths.
  EXPECT_EQ(value_at(app, "echo.status"), "up");
  EXPECT_EQ(value_at(app, "echo.jobs.a"), "1");
  EXPECT_EQ(value_at(app, "echo.jobs.b"), "2");
  EXPECT_GE(std::atoi(value_at(app, "echo.ticks").c_str()), 3) << "on_tick: runs every step";
  EXPECT_EQ(value_at(app, "ui.docs.arisrv.sim.tick"), "245")
      << "the world clock, as programs see it";
  EXPECT_NEAR(std::atof(value_at(app, "ui.docs.arisrv.sim.time").c_str()), 245 / 120.0, 1e-9);
  // The document's own state, under its prefix; and the hole is a link, not a copy.
  EXPECT_EQ(doc.prefix(), "ui.docs.arisrv");
  EXPECT_EQ(value_at(app, "ui.docs.arisrv.own.note"), "the document's own");
  EXPECT_EQ(value_at(app, "own.note"), "<absent>");
  cvc::state *hole = cvc::state::instance(app).findDescendant("ui.docs.arisrv.echo");
  ASSERT_NE(hole, nullptr);
  EXPECT_TRUE(hole->isLink());
  EXPECT_EQ(hole->linkTarget(), "echo");
  // The room, read through `room`, and never written through it.
  EXPECT_EQ(value_at(app, "ui.docs.arisrv.seen_topic"), "standup at ten");
  EXPECT_EQ(value_at(app, "conf.rooms.standup.topic"), "standup at ten");

  // What the relay would send: the published writes, nothing of the document's own.
  std::set<std::string> sent;
  for (const cvc::state_mutation &m : shard.drain_local()) {
    sent.insert(m.path);
    EXPECT_TRUE(m.path.rfind("echo", 0) == 0 || m.path.rfind("conf", 0) == 0)
        << m.path << " would leave the relay";
  }
  EXPECT_TRUE(sent.count("echo.status"));
  EXPECT_TRUE(sent.count("echo.ticks"));
  for (const auto &e : shard.snapshot())
    EXPECT_EQ(e.path.rfind("ui", 0), std::string::npos) << e.path << " in a joiner's snapshot";
  shard.detach();
}

// The documented way to take entries out of a published namespace: rewrite their parent with
// state-set-tree, which replaces the subtree through the hole. (state-delete walks the physical
// path, ARIADNE.md 7.4, so it does not reach through.)
TEST(ServerDocumentTest, SetTreeThroughTheHoleRemovesEntries) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("server.ari", R"(meta: { name: jobs, min_libcvc: 3.7.5 }
server: { publish: [echo] }
init: |
  (state-set-tree "echo.jobs" (dict "a" 1 "b" 2))
  (state-set-tree "echo.jobs" (dict "a" 3))
)");
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  doc.load(path);
  EXPECT_EQ(value_at(app, "echo.jobs.a"), "3");
  EXPECT_EQ(value_at(app, "echo.jobs.b"), "<absent>");
}

// The relay was set up (its shard scoped) for the server: block it read. A document whose block
// says otherwise by the time it loads does not run.
TEST(ServerDocumentTest, LoadRefusesAServerBlockThatChanged) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("server.ari", kEchoDoc);
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  dir.write("server.ari", R"(meta: { name: changed, min_libcvc: 3.7.5 }
server: { publish: [other] }
init: (state-set "other.x" "1")
)");
  try {
    doc.load(path);
    FAIL() << "the load ran a document the relay was not set up for";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("publishes [other] now"), std::string::npos) << e.what();
  }
  EXPECT_EQ(value_at(app, "other.x"), "<absent>");
}

TEST(ServerDocumentTest, ReadReportsABadServerBlock) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  const std::string bad = dir.write("bad.ari", "meta: { name: bad, min_libcvc: 3.7.5 }\n"
                                               "server: { publish: [conf] }\n");
  EXPECT_NE(doc.read(bad).find("server: publish: 'conf' is under conf"), std::string::npos);
  const std::string broken = dir.write("broken.ari", "server: [unclosed\n");
  EXPECT_NE(doc.read(broken), "");
  EXPECT_NE(doc.read((dir.path / "missing.ari").string()), "");
  // A document without a server: block publishes nothing, and runs.
  const std::string plain = dir.write("plain.ari", "meta: { name: plain, min_libcvc: 3.7.5 }\n"
                                                   "init: (state-set \"x\" \"1\")\n");
  ASSERT_EQ(doc.read(plain), "");
  EXPECT_FALSE(doc.block().present);
  EXPECT_TRUE(doc.block().publish.empty());
  doc.load(plain);
  EXPECT_EQ(value_at(app, "ui.docs.arisrv.x"), "1");
}

// (quit [code]) is how a server document stops its relay: arisrv exits with the code.
TEST(ServerDocumentTest, QuitAsksTheRelayToStop) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("quit.ari", "meta: { name: quit, min_libcvc: 3.7.5 }\n"
                                                 "on_tick: (quit 3)\n");
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  doc.load(path);
  for (int i = 0; i < 5 && !doc.should_close(); ++i)
    doc.step(i, i / 120.0);
  EXPECT_TRUE(doc.should_close());
  EXPECT_EQ(doc.exit_code(), 3);
}

// The relay publishes a document's scene as the room's world (WD1) but simulates none of its
// physics yet: the parsed scene, its physics and bodies included, is kept from read() on for the
// world runtime that will attach (ARIADNE_WORLDS.md §14), and the load says it is not simulated.
TEST(ServerDocumentTest, KeepsTheSceneAndSaysItIsNotSimulated) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("world.ari", R"(meta: { name: world, min_libcvc: 3.7.5 }
scene:
  physics: {gravity: [0, 0, -9.80665]}
  nodes:
    - node: ground
      source: {plane: {size: 20}}
      body: static
    - node: crate
      source: {box: {size: [0.6, 0.6, 0.6]}}
      body: {motion: dynamic, mass: 20}
)");
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  // read() has it already, so the relay can publish the world before the document runs.
  EXPECT_EQ(doc.scene().nodes.size(), 2u);
  const std::vector<std::string> warnings = doc.load(path);
  bool said = false;
  for (const std::string &w : warnings) {
    said = said || w.find("physics (2 bodies) is kept but not simulated") != std::string::npos;
    EXPECT_EQ(w.find("scene ignored"), std::string::npos) << w;
  }
  EXPECT_TRUE(said) << "the load says the physics is not simulated";
  EXPECT_TRUE(doc.scene().physics.present);
  ASSERT_EQ(doc.scene().nodes.size(), 2u);
  EXPECT_EQ(doc.scene().nodes[1].id, "crate");
  EXPECT_TRUE(doc.scene().nodes[1].body.present);
}

// The document's own state is never published, whatever prefix it binds under: an entry that is
// that prefix, holds it or lies inside it is refused (the default prefix is under ui, refused by
// name already).
TEST(ServerDocumentTest, RefusesToPublishItsOwnState) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  cvc::app app;
  ari::ServerDocument doc(app, kRoom, "srv.doc");
  for (const char *ns : {"srv", "srv.doc", "srv.doc.board"}) {
    const std::string path =
        dir.write("own.ari", std::string("meta: { name: own, min_libcvc: 3.7.5 }\n"
                                         "server: { publish: [") +
                                 ns + "] }\n");
    EXPECT_NE(doc.read(path).find("overlaps the document's own state, srv.doc"), std::string::npos)
        << ns;
  }
  const std::string ok = dir.write("ok.ari", "meta: { name: ok, min_libcvc: 3.7.5 }\n"
                                             "server: { publish: [srvx] }\n");
  EXPECT_EQ(doc.read(ok), "") << "a dot-segment boundary, not an overlap";
}

// requires: [relay] holds in a relay's server document and nowhere else; elsewhere the load fails
// and says where such a document runs.
TEST(ServerDocumentTest, TheRelayTokenHoldsOnlyOnTheRelay) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("relay.ari", R"(meta: { name: relay, min_libcvc: 3.7.5 }
requires: [relay]
init: |
  (if (has? "relay") (state-set "on_relay" "yes") #f)
)");
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  doc.load(path);
  EXPECT_EQ(value_at(app, "ui.docs.arisrv.on_relay"), "yes");

  cvc::app other;
  ari::AppRuntime client(other, "ui.docs.ari");
  try {
    client.load(path);
    FAIL() << "a host that is not the relay ran a document that requires it";
  } catch (const std::runtime_error &e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("relay"), std::string::npos) << what;
    EXPECT_NE(what.find("arisrv --doc"), std::string::npos) << what;
  }
}

// client_programs: on or off, or a map of caps, which turns them on.
TEST(ServerBlockTest, ClientProgramsTakeOnOffOrCaps) {
  ari::ServerBlock b;
  EXPECT_EQ(ari::parse_server_block(map({{"publish", seq({"board"})}}), b), "");
  EXPECT_FALSE(b.client_programs.on) << "off unless asked for";
  EXPECT_EQ(ari::parse_server_block(map({{"client_programs", scalar("on")}}), b), "");
  EXPECT_TRUE(b.client_programs.on);
  EXPECT_EQ(b.client_programs.max_steps, 0u) << "the action lane's caps";
  EXPECT_EQ(ari::parse_server_block(map({{"client_programs", scalar("off")}}), b), "");
  EXPECT_FALSE(b.client_programs.on);
  std::vector<std::string> warnings;
  EXPECT_EQ(ari::parse_server_block(map({{"client_programs", map({{"steps", scalar("200")},
                                                                  {"time", scalar("0.02")},
                                                                  {"memory_mb", scalar("1")},
                                                                  {"lanes", scalar("2")}})}}),
                                    b, &warnings),
            "");
  EXPECT_TRUE(b.client_programs.on);
  EXPECT_EQ(b.client_programs.max_steps, 200u);
  EXPECT_DOUBLE_EQ(b.client_programs.max_time, 0.02);
  EXPECT_EQ(b.client_programs.max_memory, 1024u * 1024u);
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NE(warnings[0].find("lanes"), std::string::npos) << warnings[0];
  EXPECT_NE(ari::parse_server_block(map({{"client_programs", scalar("maybe")}}), b), "");
  EXPECT_NE(ari::parse_server_block(map({{"client_programs", map({{"steps", scalar("-1")}})}}), b),
            "");
  EXPECT_NE(ari::parse_server_block(map({{"client_programs", seq({"on"})}}), b), "");
}

namespace {
// A server document that runs its clients' programs, under a step cap of its own.
const char *const kClientProgramsDoc = R"(meta: { name: client programs, min_libcvc: 3.7.5 }
server:
  client_programs: { steps: 2000 }
init: (state-set "status" "up")
)";

struct ClientRun {
  bool done = false;
  ari::ServerDocument::ClientOutcome outcome;
};

// Start `source` as `pid`'s program; its outcome lands in the returned run.
std::shared_ptr<ClientRun> run_client(ari::ServerDocument &doc, const std::string &pid,
                                      const std::string &source,
                                      cvc::state_exec::value_t args = cvc::state_exec::value_t()) {
  auto run = std::make_shared<ClientRun>();
  doc.run_client_program(pid, source, std::move(args),
                         [run](const ari::ServerDocument::ClientOutcome &o) {
                           run->done = true;
                           run->outcome = o;
                         });
  return run;
}

// Step the document until every run has its outcome (or a few steps have gone by).
void step_until_done(ari::ServerDocument &doc,
                     std::initializer_list<std::shared_ptr<ClientRun>> runs) {
  for (int i = 0; i < 20; ++i) {
    bool all = true;
    for (const auto &r : runs)
      all = all && r->done;
    if (all)
      return;
    doc.step(240 + i, (240 + i) / 120.0);
  }
}

std::string text(const cvc::state_exec::value_t &v) { return cvc::state_exec::to_string(v); }
} // namespace

// A client's program runs as the document's action, chrooted to the client's own participant: it
// reads and writes there, gets its arguments as `args`, and returns its last form's value.
TEST(ServerDocumentTest, RunsAClientProgramInItsOwnParticipant) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  namespace se = cvc::state_exec;
  TempDir dir;
  const std::string path = dir.write("server.ari", kClientProgramsDoc);
  cvc::app app;
  cvc::state::instance(app)("conf.rooms.standup.topic").value(std::string("standup at ten"));
  cvc::state::instance(app)("conf.rooms.standup.participants.p2.score").value(std::string("7"));
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  EXPECT_TRUE(doc.block().client_programs.on);
  doc.load(path);

  const auto r =
      run_client(doc, "p1",
                 "(state-set \"score\" (+ (car args) 1))\n"
                 "(state-set \"seen\" (if (is-null (state-get \"topic\")) \"nothing\" "
                 "\"the topic\"))  ; not its to see\n"
                 "(list (+ (car args) 1) (nth args 1))",
                 se::make_list({se::value_t(std::int64_t(41)), se::value_t(std::string("hi"))}));
  step_until_done(doc, {r});
  ASSERT_TRUE(r->done);
  ASSERT_TRUE(r->outcome.ok) << r->outcome.code << ": " << r->outcome.message;
  EXPECT_EQ(text(r->outcome.value),
            text(se::make_list({se::value_t(std::int64_t(42)), se::value_t(std::string("hi"))})));
  EXPECT_EQ(value_at(app, "conf.rooms.standup.participants.p1.score"), "42")
      << "written in the client's own participant";
  EXPECT_EQ(value_at(app, "conf.rooms.standup.participants.p1.seen"), "nothing")
      << "the room's topic is outside its chroot";
  EXPECT_EQ(value_at(app, "conf.rooms.standup.participants.p2.score"), "7");
  EXPECT_EQ(value_at(app, "score"), "<absent>");
  EXPECT_EQ(value_at(app, "ui.docs.arisrv.score"), "<absent>");
  EXPECT_EQ(value_at(app, "ui.docs.arisrv.status"), "up");
}

// Its writes are the ones the client could make itself: never a key the relay owns.
TEST(ServerDocumentTest, AClientProgramWritesOnlyWhatItsClientMay) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("server.ari", kClientProgramsDoc);
  cvc::app app;
  cvc::state::instance(app)("conf.rooms.standup.participants.p1.pose").value(std::string("relay"));
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  doc.load(path);

  const auto pose = run_client(doc, "p1", "(state-set \"pose\" \"teleported\")");
  const auto padded = run_client(doc, "p2", "(state-set \"device \" \"x\")");
  step_until_done(doc, {pose, padded});
  ASSERT_TRUE(pose->done && padded->done);
  EXPECT_FALSE(pose->outcome.ok);
  EXPECT_EQ(pose->outcome.code, "failed");
  EXPECT_NE(pose->outcome.message.find("PermissionError"), std::string::npos)
      << pose->outcome.message;
  EXPECT_EQ(value_at(app, "conf.rooms.standup.participants.p1.pose"), "relay");
  EXPECT_FALSE(padded->outcome.ok) << "a path the tree would trim onto the relay's key";
  EXPECT_EQ(value_at(app, "conf.rooms.standup.participants.p2.device"), "<absent>");
}

// Its environment is default-deny: nothing that spawns, messages, watches or stops the relay.
TEST(ServerDocumentTest, AClientProgramHasNoWayOut) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("server.ari", kClientProgramsDoc);
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  doc.load(path);
  const char *const sources[] = {"(spawn \"(state-set \\\"/escaped\\\" 1)\")",
                                 "(quit 3)",
                                 "(msg-send \"/ui\" \"hi\")",
                                 "(state-watch \"score\" (lambda (p) p))",
                                 "(state-delete \"score\")",
                                 "(sleep 1)"};
  int i = 0;
  for (const char *src : sources) {
    const auto r = run_client(doc, "p" + std::to_string(i++), src);
    step_until_done(doc, {r});
    ASSERT_TRUE(r->done) << src;
    EXPECT_FALSE(r->outcome.ok) << src << " ran";
    EXPECT_EQ(r->outcome.code, "failed") << src;
  }
  EXPECT_FALSE(doc.should_close()) << "(quit) is the document's, not a client's";
  EXPECT_EQ(value_at(app, "escaped"), "<absent>");
}

// The refusals: a document that did not opt in, one that has not loaded, a program that does not
// parse, a second program while the client's first runs, and one stopped by its cap.
TEST(ServerDocumentTest, AClientProgramsRefusalsAndCaps) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string off = dir.write("off.ari", R"(meta: { name: no clients, min_libcvc: 3.7.5 }
server:
  publish: [board]
)");
  {
    cvc::app app;
    ari::ServerDocument doc(app, kRoom);
    ASSERT_EQ(doc.read(off), "");
    doc.load(off);
    const auto r = run_client(doc, "p1", "1");
    ASSERT_TRUE(r->done) << "refused at once";
    EXPECT_EQ(r->outcome.code, "unavailable");
  }

  const std::string path = dir.write("server.ari", kClientProgramsDoc);
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  const auto early = run_client(doc, "p1", "1");
  ASSERT_TRUE(early->done);
  EXPECT_EQ(early->outcome.code, "unavailable") << "before the document loads";
  doc.load(path);

  const auto bad = run_client(doc, "p1", "(+ 1");
  ASSERT_TRUE(bad->done);
  EXPECT_EQ(bad->outcome.code, "invalid");
  const auto closer = run_client(doc, "p1", "1)) (quit 4) ((");
  ASSERT_TRUE(closer->done) << "a program that closes the wrapper does not parse on its own";
  EXPECT_EQ(closer->outcome.code, "invalid");
  const auto huge =
      run_client(doc, "p1", std::string(ari::ServerDocument::kClientProgramMax + 1, ' '));
  ASSERT_TRUE(huge->done);
  EXPECT_EQ(huge->outcome.code, "invalid");

  const auto first = run_client(doc, "p1", "(state-set \"n\" 1) 1");
  const auto second = run_client(doc, "p1", "2");
  const auto other = run_client(doc, "p2", "3");
  ASSERT_TRUE(second->done) << "refused at once";
  EXPECT_EQ(second->outcome.code, "rate_limited");
  step_until_done(doc, {first, other});
  EXPECT_TRUE(first->outcome.ok) << first->outcome.message;
  EXPECT_TRUE(other->outcome.ok) << other->outcome.message;
  const auto after = run_client(doc, "p1", "4");
  step_until_done(doc, {after});
  EXPECT_TRUE(after->outcome.ok) << "its first has ended";

  const auto spin = run_client(doc, "p1", "(while (= 1 1) (state-set \"n\" 2))");
  step_until_done(doc, {spin});
  ASSERT_TRUE(spin->done);
  EXPECT_EQ(spin->outcome.code, "resource_exhausted") << spin->outcome.message;
  EXPECT_FALSE(doc.should_close());
}

// client_guests: as client_programs: takes, and on its own.
TEST(ServerBlockTest, ClientGuestsTakeOnOffOrCaps) {
  ari::ServerBlock b;
  EXPECT_EQ(ari::parse_server_block(map({{"client_programs", scalar("on")}}), b), "");
  EXPECT_FALSE(b.client_guests.on) << "programs are not guests";
  std::vector<std::string> warnings;
  EXPECT_EQ(ari::parse_server_block(map({{"client_guests", map({{"steps", scalar("300")},
                                                                {"memory_mb", scalar("4")},
                                                                {"lanes", scalar("2")}})}}),
                                    b, &warnings),
            "");
  EXPECT_TRUE(b.client_guests.on);
  EXPECT_FALSE(b.client_programs.on);
  EXPECT_EQ(b.client_guests.max_steps, 300u);
  EXPECT_EQ(b.client_guests.max_memory, 4u * 1024u * 1024u);
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NE(warnings[0].find("client_guests"), std::string::npos) << warnings[0];
  EXPECT_EQ(ari::parse_server_block(map({{"client_guests", scalar("off")}}), b), "");
  EXPECT_FALSE(b.client_guests.on);
  const std::string bad = ari::parse_server_block(map({{"client_guests", scalar("maybe")}}), b);
  EXPECT_NE(bad.find("client_guests"), std::string::npos) << bad;
}

namespace {
namespace se = cvc::state_exec;

// A client's code for these tests' engine, in place of a WebAssembly guest (ariadne_wasm_test runs
// the real one): its entry says what the activation does through the slice, as a guest's imports
// would.
//   echo   finish with its arguments
//   write  (state-set "score" 42), then finish with (state-get "score"), both called in the slice
//   call   call (car args) with (cdr args), then finish with (ok value) or (#f "Class: message")
//   spin   charge its budget, and never finish
//   held   echo, once the engine's `compiled` is set: not ready() till then
class FakeClientCode : public ari::ScriptActivation {
public:
  FakeClientCode(std::string entry, se::value_t args, std::shared_ptr<std::atomic<bool>> compiled)
      : entry_(std::move(entry)), args_(std::move(args)), compiled_(std::move(compiled)) {}
  bool ready() const override { return entry_ != "held" || compiled_->load(); }
  void step(ari::ScriptSlice &s) override {
    const auto &items = *std::get<se::list_ptr>(args_.v);
    if (entry_ == "echo" || entry_ == "held")
      return s.finish(args_);
    if (entry_ == "write") {
      const se::value_t set[] = {se::value_t(std::string("score")), se::value_t(std::int64_t(42))};
      if (!s.call_now("state-set", set) || s.failed())
        throw se::dsl_error("RuntimeError", "state-set: " + s.error());
      const se::value_t get[] = {se::value_t(std::string("score"))};
      if (!s.call_now("state-get", get) || s.failed())
        throw se::dsl_error("RuntimeError", "state-get: " + s.error());
      return s.finish(s.reply());
    }
    if (entry_ == "call") {
      if (!s.has_reply()) {
        const std::string fn = std::get<std::string>(items.at(0).v);
        return s.call(fn, std::vector<se::value_t>(items.begin() + 1, items.end()));
      }
      return s.finish(s.failed() ? se::make_list({se::value_t(false), se::value_t(s.error())})
                                 : se::make_list({se::value_t(true), s.reply()}));
    }
    s.charge(s.budget()); // spin
  }

private:
  std::string entry_;
  se::value_t args_;
  std::shared_ptr<std::atomic<bool>> compiled_;
};

class FakeEngine : public ari::ScriptEngine {
public:
  std::string language() const override { return "fake"; }
  void load_block(const ari::ScriptBlock &, ari::ScriptLane &,
                  std::vector<std::string> &) override {}
  std::shared_ptr<ari::ScriptActivation> client_activation(cvc::app &, ari::ClientCode code,
                                                           se::value_t args,
                                                           std::string &error) override {
    last_memory = code.max_memory;
    if (code.bytes != "fake code") {
      error = "not fake code";
      return nullptr;
    }
    return std::make_shared<FakeClientCode>(
        code.entry, args.is_nil() ? se::make_list() : std::move(args), compiled);
  }
  std::uint64_t last_memory = 0;
  std::shared_ptr<std::atomic<bool>> compiled = std::make_shared<std::atomic<bool>>(false);
};

const char *const kClientGuestsDoc = R"(meta: { name: client guests, min_libcvc: 3.7.5 }
server:
  client_programs: { steps: 2000 }
  client_guests: { steps: 2000, memory_mb: 4 }
init: (state-set "status" "up")
)";

std::shared_ptr<ClientRun> run_guest(ari::ServerDocument &doc, const std::string &pid,
                                     const std::string &entry, se::value_t args = se::value_t(),
                                     std::string module = "fake code") {
  auto run = std::make_shared<ClientRun>();
  doc.run_client_guest(pid, std::move(module), entry, std::move(args),
                       [run](const ari::ServerDocument::ClientOutcome &o) {
                         run->done = true;
                         run->outcome = o;
                       });
  return run;
}
} // namespace

// A client's guest runs as its client's program: in the participant's chroot, its value back, and
// the memory cap the document set handed to the engine.
TEST(ServerDocumentTest, AClientGuestRunsAsItsClientsProgram) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("server.ari", kClientGuestsDoc);
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  EXPECT_TRUE(doc.block().client_guests.on);
  const auto engine = std::make_shared<FakeEngine>();
  doc.use_client_guests(engine);
  doc.load(path);

  const auto echo =
      run_guest(doc, "p1", "echo",
                se::make_list({se::value_t(std::int64_t(41)), se::value_t(std::string("hi"))}));
  const auto write = run_guest(doc, "p2", "write");
  step_until_done(doc, {echo, write});
  ASSERT_TRUE(echo->done && write->done);
  ASSERT_TRUE(echo->outcome.ok) << echo->outcome.code << ": " << echo->outcome.message;
  EXPECT_EQ(text(echo->outcome.value),
            text(se::make_list({se::value_t(std::int64_t(41)), se::value_t(std::string("hi"))})));
  EXPECT_EQ(engine->last_memory, 4u * 1024u * 1024u) << "client_guests: memory_mb";
  ASSERT_TRUE(write->outcome.ok) << write->outcome.code << ": " << write->outcome.message;
  EXPECT_EQ(text(write->outcome.value), "\"42\"");
  EXPECT_EQ(value_at(app, "conf.rooms.standup.participants.p2.score"), "42");
  EXPECT_EQ(value_at(app, "score"), "<absent>");
  EXPECT_EQ(value_at(app, "ui.docs.arisrv.score"), "<absent>");
}

// Its calls reach what its client's program has, and no more: no spawn, not the natives that run
// it, no key the relay owns, nothing outside its participant.
TEST(ServerDocumentTest, AClientGuestsCallsReachOnlyItsProgramsEnvironment) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("server.ari", kClientGuestsDoc);
  cvc::app app;
  cvc::state::instance(app)("conf.rooms.standup.topic").value(std::string("standup at ten"));
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  doc.use_client_guests(std::make_shared<FakeEngine>());
  doc.load(path);

  const auto call = [&](const std::string &pid, std::vector<se::value_t> args) {
    const auto r = run_guest(doc, pid, "call", se::make_list(std::move(args)));
    step_until_done(doc, {r});
    EXPECT_TRUE(r->done);
    EXPECT_TRUE(r->outcome.ok) << r->outcome.code << ": " << r->outcome.message;
    return text(r->outcome.value);
  };
  const std::string spawn = call(
      "p1", {se::value_t(std::string("spawn")), se::value_t(std::string("(state-set \"/x\" 1)"))});
  EXPECT_NE(spawn.find("NameError"), std::string::npos) << spawn;
  const std::string result =
      call("p2", {se::value_t(std::string("__client-result")), se::value_t(std::int64_t(1))});
  EXPECT_NE(result.find("PermissionError"), std::string::npos) << result;
  const std::string guest = call("p3", {se::value_t(std::string("__client-guest"))});
  EXPECT_NE(guest.find("PermissionError"), std::string::npos) << guest;
  const std::string pose =
      call("p4", {se::value_t(std::string("state-set")), se::value_t(std::string("pose")),
                  se::value_t(std::int64_t(1))});
  EXPECT_NE(pose.find("PermissionError"), std::string::npos) << pose;
  const std::string topic =
      call("p5", {se::value_t(std::string("state-get")), se::value_t(std::string("topic"))});
  EXPECT_EQ(topic, "(#t nil)") << "the room's topic is outside its chroot";
  EXPECT_EQ(value_at(app, "x"), "<absent>");
  EXPECT_EQ(value_at(app, "conf.rooms.standup.participants.p4.pose"), "<absent>");
}

// The refusals: no client_guests:, no engine, not loaded, code the engine won't take, too large, no
// export, a second run while the client's first runs (a program or a guest), and its cap.
TEST(ServerDocumentTest, AClientGuestsRefusalsAndCaps) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string programs = dir.write("programs.ari", kClientProgramsDoc);
  {
    cvc::app app;
    ari::ServerDocument doc(app, kRoom);
    ASSERT_EQ(doc.read(programs), "");
    doc.use_client_guests(std::make_shared<FakeEngine>());
    doc.load(programs);
    const auto r = run_guest(doc, "p1", "echo");
    ASSERT_TRUE(r->done) << "refused at once";
    EXPECT_EQ(r->outcome.code, "unavailable") << "client_programs: is not client_guests:";
  }
  const std::string path = dir.write("server.ari", kClientGuestsDoc);
  {
    cvc::app app;
    ari::ServerDocument doc(app, kRoom);
    ASSERT_EQ(doc.read(path), "");
    doc.load(path);
    const auto r = run_guest(doc, "p1", "echo");
    ASSERT_TRUE(r->done);
    EXPECT_EQ(r->outcome.code, "unavailable") << "no engine";
    EXPECT_NE(r->outcome.message.find("arisrvx"), std::string::npos) << r->outcome.message;
  }
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  doc.use_client_guests(std::make_shared<FakeEngine>());
  const auto early = run_guest(doc, "p1", "echo");
  ASSERT_TRUE(early->done);
  EXPECT_EQ(early->outcome.code, "unavailable") << "before the document loads";
  doc.load(path);

  const auto foreign = run_guest(doc, "p1", "echo", se::value_t(), "not fake code");
  ASSERT_TRUE(foreign->done);
  EXPECT_EQ(foreign->outcome.code, "invalid");
  EXPECT_EQ(foreign->outcome.message, "not fake code") << "the engine's own reason";
  const auto huge = run_guest(doc, "p1", "echo", se::value_t(),
                              std::string(ari::ServerDocument::kClientGuestMax + 1, 'x'));
  ASSERT_TRUE(huge->done);
  EXPECT_EQ(huge->outcome.code, "invalid");
  const auto unnamed = run_guest(doc, "p1", "");
  ASSERT_TRUE(unnamed->done);
  EXPECT_EQ(unnamed->outcome.code, "invalid");

  const auto program = run_client(doc, "p1", "(state-set \"n\" 1) 1");
  const auto guest = run_guest(doc, "p1", "echo");
  ASSERT_TRUE(guest->done) << "refused at once";
  EXPECT_EQ(guest->outcome.code, "rate_limited") << "one program or guest per client";
  step_until_done(doc, {program});
  EXPECT_TRUE(program->outcome.ok) << program->outcome.message;

  const auto spin = run_guest(doc, "p1", "spin");
  step_until_done(doc, {spin});
  ASSERT_TRUE(spin->done);
  EXPECT_EQ(spin->outcome.code, "resource_exhausted") << spin->outcome.message;
  const auto after = run_guest(doc, "p1", "echo");
  step_until_done(doc, {after});
  EXPECT_TRUE(after->outcome.ok) << "the capped one has ended";
}

// A guest whose activation isn't ready (its module still compiling) is held, its program not yet
// started, so the wait costs none of its caps; meanwhile it counts as running; it starts at the
// first step() after it is ready.
TEST(ServerDocumentTest, AClientGuestIsHeldUntilItIsReady) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  TempDir dir;
  const std::string path = dir.write("server.ari", kClientGuestsDoc);
  cvc::app app;
  ari::ServerDocument doc(app, kRoom);
  ASSERT_EQ(doc.read(path), "");
  const auto engine = std::make_shared<FakeEngine>();
  doc.use_client_guests(engine);
  doc.load(path);

  const auto held = run_guest(doc, "p1", "held", se::make_list({se::value_t(std::int64_t(7))}));
  for (int i = 0; i < 50; ++i)
    doc.step(240 + i, (240 + i) / 120.0);
  EXPECT_FALSE(held->done) << "held while its module compiles, however many steps go by";
  const auto again = run_guest(doc, "p1", "echo");
  ASSERT_TRUE(again->done) << "refused at once";
  EXPECT_EQ(again->outcome.code, "rate_limited");
  engine->compiled->store(true);
  step_until_done(doc, {held});
  ASSERT_TRUE(held->done);
  ASSERT_TRUE(held->outcome.ok) << held->outcome.code << ": " << held->outcome.message;
  EXPECT_EQ(text(held->outcome.value), text(se::make_list({se::value_t(std::int64_t(7))})));
}
