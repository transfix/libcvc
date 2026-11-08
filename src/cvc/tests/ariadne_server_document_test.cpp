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
// namespaces it publishes through holes in its chroot, the room it reads, and what a relay's shard
// scoped by it sends. room_relay_integration_test.cpp has the same over a relay's transports.

#include <cstdint>
#include <cstdlib>
#include <cvc/ariadne/server_document.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
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

// The relay draws no scene and runs no world from a document yet: the parsed scene, its physics and
// bodies included, is kept for the world runtime that will attach (ARIADNE_WORLDS.md §14), and the
// load says it is not simulated.
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
