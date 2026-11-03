/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Room chat and direct messages (room_chat.h, docs/roadmap/ARIADNE_CHAT.md): the relay's rules,
// who is pushed what, pagination, and a client's view, over an in-process wire that does what a
// relay's transports do -- a client's request reaches the relay's shard, and each push lands in the
// tree of the client holding that pid. The real transports' part (ttl_hops 0 stays at the relay, a
// push reaches only the pid's holder) is in state_transport_{ws,grpc}_test.

#include <algorithm>
#include <cvc/ariadne/ariadne.h> // Runtime, clear_action_intrinsics
#include <cvc/ariadne/conference/room_chat.h>
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_write_gate.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_message.h>
#include <cvc/state/state_transport.h>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace conf = cvc::ariadne::conference;
using entry = cvc::state_transport::snapshot_entry;

namespace {

const char *const kRoom = "r";

// The wire's two ends, as a relay's transports behave. Nothing else of a transport is used.
class wire_base : public cvc::state_transport {
public:
  void register_shard(cvc::state_cluster_shard *) override {}
  void unregister_shard(cvc::state_cluster_shard *) override {}
  publish_stats publish(const cvc::state_mutation &) override { return {}; }
  std::size_t pump_shard(cvc::state_cluster_shard &) override { return 0; }
  std::size_t pump_all() override { return 0; }
  void flush() override {}
};

// The relay's end: each push is recorded, and applied to the tree of the client holding the pid
// (as an unsolicited snapshot is), unless the test drops it.
class relay_wire final : public wire_base {
public:
  std::map<std::string, cvc::state_cluster_shard *> clients;
  std::vector<std::pair<std::string, std::vector<entry>>> pushes;
  std::set<std::string> drop; // pids whose pushes are lost

  std::size_t push_snapshot_to_holder(const std::string &pid,
                                      const std::vector<entry> &entries) override {
    pushes.emplace_back(pid, entries);
    auto it = clients.find(pid);
    if (it == clients.end() || drop.count(pid))
      return 0;
    for (const entry &e : entries)
      it->second->ingest_snapshot_entry(e);
    return 1;
  }
  // The paths pushed to PID so far.
  std::vector<std::string> paths_to(const std::string &pid) const {
    std::vector<std::string> out;
    for (const auto &[p, entries] : pushes)
      if (p == pid)
        for (const entry &e : entries)
          out.push_back(e.path);
    return out;
  }
};

// A client's end: its shard's messages reach the relay's shard.
class client_wire final : public wire_base {
public:
  cvc::state_cluster_shard *relay = nullptr;
  publish_message_stats publish_message(const cvc::state_message &m) override {
    publish_message_stats s;
    if (relay && relay->ingest_remote_message(m))
      s.delivered = 1;
    return s;
  }
};

std::string leaf(cvc::app &app, const std::string &path) {
  cvc::state *n = app.root().findDescendant(path);
  return n ? n->value() : std::string();
}

std::string chat(const std::string &rest) { return conf::chat_root_path(kRoom) + "." + rest; }

// A relay and its clients, each client with its own app and shard on the wire.
struct harness {
  explicit harness(conf::chat_limits limits = {}) : relay_shard(relay_app, "conf", "server") {
    relay_shard.attach();
    chat = std::make_unique<conf::room_chat_relay>(relay_app, relay_shard, kRoom, limits);
    chat->add_transport(&wire);
  }

  struct client {
    cvc::app app;
    std::unique_ptr<cvc::state_cluster_shard> shard;
    client_wire wire;
  };

  // PID joins: present in the relay's roster, with a client on the wire.
  client &join(const std::string &pid, const std::string &name) {
    conf::room_join(relay_app, kRoom, pid, name);
    auto c = std::make_unique<client>();
    c->shard = std::make_unique<cvc::state_cluster_shard>(c->app, "conf", pid);
    c->shard->attach();
    c->shard->set_transport(&c->wire);
    c->wire.relay = &relay_shard;
    wire.clients[pid] = c->shard.get();
    client &ref = *c;
    clients[pid] = std::move(c);
    sync();
    return ref;
  }
  void leave(const std::string &pid) {
    conf::room_leave(relay_app, kRoom, pid);
    chat->participant_left(pid);
  }

  // What the relay's drain would replicate to every client: the epoch and the room's counters.
  void sync() {
    for (auto &[pid, c] : clients)
      for (const char *k : {"epoch", "room.head", "room.first"}) {
        const std::string v = leaf(relay_app, ::chat(k));
        if (!v.empty())
          c->app.root()(::chat(k)).value(v);
      }
  }

  // A request as it arrives from PID's connection.
  void send(const std::string &pid, cvc::state_message m) {
    m.cluster_id = "conf";
    m.origin_node_id = pid;
    m.message_id = pid + ":" + std::to_string(++ids);
    relay_shard.ingest_remote_message(m);
  }
  void post(const std::string &pid, const std::string &text, const std::string &to = "") {
    send(pid, conf::make_chat_post(kRoom, pid, ++cseq, to, text));
  }
  void step(double now_s = 0.0) {
    chat->step(now_s);
    sync();
  }

  cvc::app relay_app;
  cvc::state_cluster_shard relay_shard;
  relay_wire wire;
  std::unique_ptr<conf::room_chat_relay> chat;
  std::map<std::string, std::unique_ptr<client>> clients;
  int ids = 0;
  std::uint64_t cseq = 0;
};

conf::chat_message message_at(cvc::app &app, const std::string &path) {
  conf::chat_message m;
  EXPECT_TRUE(conf::decode_chat_message(leaf(app, path), m)) << path;
  return m;
}

} // namespace

// A message is one JSON leaf; whatever its text holds comes back as it went.
TEST(RoomChat, AMessageRoundTripsAsJson) {
  conf::chat_message m;
  m.seq = 41;
  m.from = "alice_1";
  m.name = "Al \"the\" Ice";
  m.t = 1730512345.25;
  m.text = "line one\nline\ttwo \\ back\x01slash \xc3\xa9t\xc3\xa9 \xf0\x9f\x98\x80";
  m.epoch = "1730512000000";
  conf::chat_message back;
  ASSERT_TRUE(conf::decode_chat_message(conf::encode_chat_message(m), back));
  EXPECT_EQ(back.seq, m.seq);
  EXPECT_EQ(back.from, m.from);
  EXPECT_EQ(back.name, m.name);
  EXPECT_DOUBLE_EQ(back.t, m.t);
  EXPECT_EQ(back.text, m.text);
  EXPECT_EQ(back.epoch, m.epoch);
  EXPECT_FALSE(conf::decode_chat_message("{\"seq\":1}", back)) << "no sender";
  EXPECT_FALSE(conf::decode_chat_message("not json", back));
  EXPECT_FALSE(conf::decode_chat_message("{\"seq\":-1,\"from\":\"a\"}", back));
}

// The relay's cleaning: control characters but newline and tab go, trailing blanks and leading
// blank lines too; what is left must be text, not empty, and short enough.
TEST(RoomChat, TextIsCleanedAsTheRelayDoes) {
  std::string why;
  std::string t = "\n\nhi\r\n\x07there\t \n  ";
  ASSERT_TRUE(conf::clean_chat_text(t, 100, why)) << why;
  EXPECT_EQ(t, "hi\nthere");
  t = " \n\t\r";
  EXPECT_FALSE(conf::clean_chat_text(t, 100, why));
  EXPECT_EQ(why, "the message is empty");
  t = std::string(101, 'x');
  EXPECT_FALSE(conf::clean_chat_text(t, 100, why));
  t = "bad \xff byte";
  EXPECT_FALSE(conf::clean_chat_text(t, 100, why));
  EXPECT_EQ(why, "the message is not UTF-8 text");
  t = "\xc3\xa9";
  EXPECT_TRUE(conf::clean_chat_text(t, 2, why)) << "the cap is in bytes";
}

// A room post is stamped by the relay (the sender's pid and current name, the seq, the epoch) and
// pushed to everyone present; the room's head and first are written to the relay's tree, for its
// drain to replicate. chat.rev comes last in every push.
TEST(RoomChat, ARoomPostReachesEveryoneInTheRoom) {
  harness h;
  harness::client &alice = h.join("alice", "Alice");
  harness::client &bob = h.join("bob", "Bob");
  EXPECT_EQ(leaf(h.relay_app, chat("epoch")), h.chat->epoch());
  EXPECT_EQ(leaf(h.relay_app, chat("room.head")), "0");

  h.post("alice", "hello room");
  h.step();
  EXPECT_EQ(leaf(h.relay_app, chat("room.head")), "1");
  EXPECT_EQ(leaf(h.relay_app, chat("room.first")), "0");
  for (harness::client *c : {&alice, &bob}) {
    const conf::chat_message m = message_at(c->app, chat("room.m.0"));
    EXPECT_EQ(m.seq, 0u);
    EXPECT_EQ(m.from, "alice");
    EXPECT_EQ(m.name, "Alice");
    EXPECT_EQ(m.text, "hello room");
    EXPECT_EQ(m.epoch, h.chat->epoch());
    EXPECT_GT(m.t, 1.0e9);
  }
  for (const auto &[pid, entries] : h.wire.pushes)
    EXPECT_EQ(entries.back().path, chat("rev")) << "pushed to " << pid;
  const auto st = h.chat->take_stats();
  EXPECT_EQ(st.posts, 1u);
  EXPECT_EQ(st.pushes, 2u);
}

// A DM reaches its two participants and no one else; each holds it under the other's pid, and
// each is told who it has conversations with.
TEST(RoomChat, ADirectMessageReachesOnlyItsTwo) {
  harness h;
  harness::client &alice = h.join("alice", "Alice");
  harness::client &bob = h.join("bob", "Bob");
  h.join("carol", "Carol");

  h.post("alice", "psst bob", "bob");
  h.post("bob", "hi alice", "alice");
  h.step();
  EXPECT_TRUE(h.wire.paths_to("carol").empty()) << "carol is pushed nothing";
  EXPECT_EQ(message_at(alice.app, chat("dm.bob.m.0")).text, "psst bob");
  EXPECT_EQ(message_at(alice.app, chat("dm.bob.m.1")).text, "hi alice");
  EXPECT_EQ(message_at(bob.app, chat("dm.alice.m.0")).text, "psst bob");
  EXPECT_EQ(message_at(bob.app, chat("dm.alice.m.1")).from, "bob");
  EXPECT_EQ(leaf(alice.app, chat("dm.bob.head")), "2");
  EXPECT_EQ(leaf(bob.app, chat("dms")), "alice");
  EXPECT_EQ(leaf(alice.app, chat("dms")), "bob");
  EXPECT_EQ(leaf(h.relay_app, chat("room.head")), "0") << "the room's history is untouched";
  for (const char *k : {"dms", "dm"})
    EXPECT_EQ(h.relay_app.root().findDescendant(chat(k)), nullptr)
        << "nothing of a conversation is in the relay's shared tree";
  EXPECT_EQ(h.chat->take_stats().dms, 2u);
}

// What the relay refuses comes back to the poster alone, as chat.error with its own count.
TEST(RoomChat, TheRelayRefusesWhatItMust) {
  conf::chat_limits limits;
  limits.text_max = 10;
  limits.rate_burst = 3;
  limits.rate_per_s = 1;
  harness h(limits);
  harness::client &alice = h.join("alice", "Alice");
  h.join("bob", "Bob");
  const auto error = [&](harness::client &c) { return leaf(c.app, chat("error")); };

  h.post("alice", "   ");
  h.step();
  EXPECT_NE(error(alice).find("the message is empty"), std::string::npos) << error(alice);
  h.post("alice", "far too long for ten");
  h.step();
  EXPECT_NE(error(alice).find("longer than 10 bytes"), std::string::npos);
  h.post("alice", "hi", "alice");
  h.step();
  EXPECT_NE(error(alice).find("to yourself"), std::string::npos);
  h.post("alice", "hi", "nobody");
  h.step();
  EXPECT_NE(error(alice).find("not in the room"), std::string::npos);
  EXPECT_NE(error(alice).find("\"to\":\"nobody\""), std::string::npos) << "names the channel";

  // The rate limit: a burst of 3, then one a second.
  for (int i = 0; i < 4; ++i)
    h.post("alice", "m" + std::to_string(i));
  h.step(10.0);
  EXPECT_EQ(leaf(h.relay_app, chat("room.head")), "3");
  EXPECT_NE(error(alice).find("too fast"), std::string::npos);
  h.post("alice", "later");
  h.step(11.0);
  EXPECT_EQ(leaf(h.relay_app, chat("room.head")), "4") << "refilled";

  // Someone not in the room cannot post.
  h.leave("bob");
  h.post("bob", "ghost");
  h.step(20.0);
  EXPECT_EQ(leaf(h.relay_app, chat("room.head")), "4");
  EXPECT_GE(h.chat->take_stats().refused, 6u);

  // A relay with DMs off (no bound writes) refuses them.
  conf::chat_limits off;
  off.dms = false;
  harness h2(off);
  harness::client &a2 = h2.join("alice", "Alice");
  h2.join("bob", "Bob");
  h2.post("alice", "hi", "bob");
  h2.step();
  EXPECT_NE(leaf(a2.app, chat("error")).find("direct messages are off"), std::string::npos);
  EXPECT_TRUE(h2.wire.paths_to("bob").empty());
}

// A request counts only from the sender's own path, as itself, for the relay alone.
TEST(RoomChat, RequestsAreTakenOnlyFromTheSendersOwnPath) {
  harness h;
  h.join("alice", "Alice");
  h.join("bob", "Bob");
  cvc::state_message as_bob = conf::make_chat_post(kRoom, "bob", 1, "", "spoofed");
  h.send("alice", as_bob); // alice's connection, bob's path
  cvc::state_message fanned = conf::make_chat_post(kRoom, "alice", 2, "", "for everyone");
  fanned.ttl_hops = 8;
  h.send("alice", fanned);
  h.send("alice", conf::make_chat_post("other", "alice", 3, "", "another room"));
  h.step();
  EXPECT_EQ(leaf(h.relay_app, chat("room.head")), "0");
  EXPECT_EQ(h.chat->take_stats().dropped, 2u) << "the third is not this room's";

  // The room's write gate: a chat request only with ttl_hops 0, only on the sender's own path.
  conf::room_write_gate gate(kRoom);
  cvc::state_message ok = conf::make_chat_post(kRoom, "alice", 4, "", "x");
  EXPECT_TRUE(gate.admit("alice", ok));
  EXPECT_FALSE(gate.admit("bob", ok));
  ok.ttl_hops = 1;
  EXPECT_FALSE(gate.admit("alice", ok));
  cvc::state_message other =
      cvc::state_message::make_text(conf::room_participant_path(kRoom, "alice") + ".note", "x");
  EXPECT_TRUE(gate.admit("alice", other)) << "other messages on its own path are as before";
}

// A page is the messages before `before`, at most `page` of them, never older than what the relay
// keeps; an empty page still moves chat.rev, so the client knows it was answered.
TEST(RoomChat, PagesAreBoundedByWhatTheRelayKeeps) {
  conf::chat_limits limits;
  limits.room_keep = 50;
  limits.page = 16;
  limits.rate_burst = 1000;
  harness h(limits);
  h.join("alice", "Alice");
  harness::client &bob = h.join("bob", "Bob");
  for (int i = 0; i < 80; ++i)
    h.post("alice", "m" + std::to_string(i));
  h.step();
  EXPECT_EQ(leaf(h.relay_app, chat("room.head")), "80");
  EXPECT_EQ(leaf(h.relay_app, chat("room.first")), "30");

  h.wire.pushes.clear();
  h.send("bob", conf::make_chat_page(kRoom, "bob", "", 40));
  h.step();
  std::vector<std::string> want;
  for (int s = 30; s < 40; ++s)
    want.push_back(chat("room.m." + std::to_string(s)));
  want.push_back(chat("rev"));
  EXPECT_EQ(h.wire.paths_to("bob"), want) << "the page stops at first";
  EXPECT_EQ(message_at(bob.app, chat("room.m.30")).text, "m30");

  h.wire.pushes.clear();
  h.send("bob", conf::make_chat_page(kRoom, "bob", "", 30));
  h.step();
  EXPECT_EQ(h.wire.paths_to("bob"), std::vector<std::string>{chat("rev")});
  EXPECT_EQ(h.chat->take_stats().pages, 2u);
}

// A conversation is forgotten once neither of its two is in the room.
TEST(RoomChat, AConversationEndsWhenBothHaveLeft) {
  harness h;
  h.join("alice", "Alice");
  h.join("bob", "Bob");
  h.post("alice", "one", "bob");
  h.step();
  h.leave("alice");
  h.wire.pushes.clear();
  h.send("bob", conf::make_chat_page(kRoom, "bob", "alice", 10));
  h.step();
  EXPECT_EQ(h.wire.paths_to("bob").size(), 4u) << "bob still has it: the message, head, first, rev";
  h.leave("bob");
  conf::room_join(h.relay_app, kRoom, "bob", "Bob"); // back, as the same pid
  h.wire.pushes.clear();
  h.send("bob", conf::make_chat_page(kRoom, "bob", "alice", 10));
  h.step();
  EXPECT_EQ(h.wire.paths_to("bob"), std::vector<std::string>{chat("rev")}) << "gone";
}

// Every ~5 s the relay re-pushes each present participant's conversation heads.
TEST(RoomChat, ConversationHeadsArePushedAgain) {
  harness h;
  h.join("alice", "Alice");
  h.join("bob", "Bob");
  h.post("alice", "one", "bob");
  h.step(100.0);
  h.wire.pushes.clear();
  h.step(102.0);
  EXPECT_TRUE(h.wire.pushes.empty());
  h.step(105.5);
  const std::vector<std::string> to_bob = h.wire.paths_to("bob");
  EXPECT_NE(std::find(to_bob.begin(), to_bob.end(), chat("dm.alice.head")), to_bob.end());
  EXPECT_NE(std::find(to_bob.begin(), to_bob.end(), chat("dms")), to_bob.end());
}

// A participant who joins after 100 messages sees the newest page, pages back to the oldest the
// relay keeps, and then gets the next live message -- all as one contiguous run.
TEST(RoomChat, ALateJoinerPagesBackThroughTheRoom) {
  conf::chat_limits limits;
  limits.rate_burst = 1000;
  harness h(limits);
  h.join("alice", "Alice");
  for (int i = 0; i < 100; ++i)
    h.post("alice", "m" + std::to_string(i));
  h.step();
  harness::client &carol = h.join("carol", "Carol");
  conf::room_chat_client view(carol.app, kRoom, "carol");

  conf::room_chat_client::info i = view.refresh("", 1.0);
  EXPECT_TRUE(i.available);
  EXPECT_EQ(i.head, 100u);
  EXPECT_EQ(i.count, 0u);
  EXPECT_TRUE(i.loading) << "the first view asks for the newest page";
  h.step();
  i = view.refresh("", 1.1);
  EXPECT_FALSE(i.loading);
  EXPECT_EQ(i.count, 32u);
  EXPECT_TRUE(i.more);
  std::vector<conf::chat_message> run = view.messages("");
  ASSERT_EQ(run.size(), 32u);
  EXPECT_EQ(run.front().text, "m68");
  EXPECT_EQ(run.back().text, "m99");

  const std::uint64_t rev = i.rev;
  int pages = 0;
  while (view.refresh("", 2.0).more) {
    ASSERT_TRUE(view.more("", 2.0));
    h.step();
    ++pages;
    ASSERT_LT(pages, 10);
  }
  EXPECT_EQ(pages, 3);
  i = view.refresh("", 2.0);
  EXPECT_EQ(i.count, 100u);
  EXPECT_NE(i.rev, rev);
  EXPECT_FALSE(view.more("", 2.0)) << "nothing older to ask for";
  run = view.messages("");
  EXPECT_EQ(run.front().text, "m0");

  h.post("alice", "live one");
  h.step();
  i = view.refresh("", 3.0);
  EXPECT_EQ(i.count, 101u);
  EXPECT_EQ(view.messages("").back().text, "live one");
}

// A lost push leaves a gap: the client sees the room's head run ahead and, 2 s on, asks for the
// newest page again. A lost DM push is noticed from the heads the relay re-pushes.
TEST(RoomChat, AClientRecoversWhatAPushLost) {
  harness h;
  harness::client &alice = h.join("alice", "Alice");
  harness::client &bob = h.join("bob", "Bob");
  conf::room_chat_client view(bob.app, kRoom, "bob");
  h.post("alice", "first");
  h.step(1.0);
  EXPECT_EQ(view.refresh("", 1.0).count, 1u);

  h.wire.drop.insert("bob");
  h.post("alice", "lost");
  h.post("alice", "lost too", "bob");
  h.step(1.5);
  h.wire.drop.clear();
  conf::room_chat_client::info i = view.refresh("", 1.5);
  EXPECT_EQ(i.head, 2u);
  EXPECT_EQ(i.count, 1u);
  EXPECT_FALSE(i.loading) << "not yet: a push may still be on its way";
  i = view.refresh("", 3.6);
  EXPECT_TRUE(i.loading);
  h.step(3.6);
  i = view.refresh("", 3.7);
  EXPECT_EQ(i.count, 2u);
  EXPECT_EQ(view.messages("").back().text, "lost");

  EXPECT_TRUE(view.conversations().empty()) << "the DM's push was lost";
  h.step(7.0); // the heads come again
  const std::vector<conf::room_chat_client::conversation> convs = view.conversations();
  ASSERT_EQ(convs.size(), 1u);
  EXPECT_EQ(convs[0].peer, "alice");
  EXPECT_EQ(convs[0].head, 1u);
  i = view.refresh("alice", 7.0);
  EXPECT_TRUE(i.loading) << "a conversation with nothing held asks for its newest page";
  h.step(7.0);
  i = view.refresh("alice", 7.1);
  EXPECT_EQ(i.count, 1u);
  EXPECT_EQ(view.messages("alice").front().text, "lost too");
  (void)alice;
}

// A refused post is reported on its channel's info, and cleared by the next send there.
TEST(RoomChat, AClientShowsARefusalOnItsChannel) {
  harness h;
  harness::client &alice = h.join("alice", "Alice");
  h.join("bob", "Bob");
  conf::room_chat_client view(alice.app, kRoom, "alice");
  std::string why;
  EXPECT_FALSE(view.post("  ", "", why));
  EXPECT_EQ(why, "the message is empty") << "checked before it is sent";
  ASSERT_TRUE(view.post("hi", "carol", why)) << why;
  h.step();
  EXPECT_NE(view.refresh("carol", 1.0).error.find("not in the room"), std::string::npos);
  EXPECT_EQ(view.refresh("", 1.0).error, "") << "another channel's";
  ASSERT_TRUE(view.post("hi", "bob", why));
  ASSERT_TRUE(view.post("hello", "carol", why));
  EXPECT_EQ(view.refresh("carol", 1.0).error, "");
}

// A new relay process (another epoch) is another history: what the old one pushed is dropped.
TEST(RoomChat, AnotherEpochStartsTheViewAfresh) {
  harness h;
  harness::client &bob = h.join("bob", "Bob");
  h.join("alice", "Alice");
  conf::room_chat_client view(bob.app, kRoom, "bob");
  h.post("alice", "old");
  h.step();
  EXPECT_EQ(view.refresh("", 1.0).count, 1u);
  bob.app.root()(chat("epoch")).value("another");
  bob.app.root()(chat("room.head")).value("0");
  const conf::room_chat_client::info i = view.refresh("", 1.0);
  EXPECT_EQ(i.count, 0u);
  EXPECT_TRUE(view.messages("").empty());
}

// The DSL verbs over the same wire: post from a program, read the room's messages and info, page.
TEST(RoomChat, TheVerbsPostReadAndPage) {
  conf::chat_limits limits;
  limits.rate_burst = 1000;
  harness h(limits);
  h.join("alice", "Alice");
  for (int i = 0; i < 40; ++i)
    h.post("alice", "m" + std::to_string(i));
  h.step();
  harness::client &bob = h.join("bob", "Bob");
  conf::register_room_intrinsics(bob.app);
  cvc::ariadne::Runtime rt(bob.app, "");
  const auto run = [&](const std::string &program) {
    std::string result = "pending";
    rt.run_program(program, [&](const std::string &r) { result = r; });
    for (int i = 0; i < 200 && result == "pending"; ++i)
      rt.drain();
    return result;
  };
  EXPECT_EQ(run("(begin (set h (room-join \"\" \"r\" (dict \"name\" \"Bob\")))"
                " (state-set \"tok\" (get-attr h \"token\"))"
                " (state-set \"posted\" (str (get-attr (chat-post h \"from the DSL\") \"ok\")))"
                " (state-set \"bad\" (get-attr (chat-post h \"   \") \"error\"))"
                " (state-set \"loading\" (str (get-attr (chat-info h \"\") \"loading\"))))"),
            "");
  EXPECT_EQ(leaf(bob.app, "posted"), "#t");
  EXPECT_EQ(leaf(bob.app, "bad"), "the message is empty");
  EXPECT_EQ(leaf(bob.app, "loading"), "#t") << "the first chat-info asked for the newest page";
  h.step();
  EXPECT_EQ(run("(begin (set i (chat-info (state-get \"tok\") \"\"))"
                " (state-set \"count\" (str (get-attr i \"count\")))"
                " (state-set \"more\" (str (get-attr i \"more\")))"
                " (set ms (chat-messages (state-get \"tok\") \"\"))"
                " (state-set \"last\" (get-attr (nth ms (- (length ms) 1)) \"text\"))"
                " (state-set \"mine\" (str (get-attr (nth ms (- (length ms) 1)) \"mine\")))"
                " (state-set \"page\" (str (get-attr (chat-more (state-get \"tok\") \"\") "
                "\"loading\"))))"),
            "");
  EXPECT_EQ(leaf(bob.app, "count"), "33")
      << "the newest page (asked before the post arrived) and the post";
  EXPECT_EQ(leaf(bob.app, "more"), "#t");
  EXPECT_EQ(leaf(bob.app, "last"), "from the DSL");
  EXPECT_EQ(leaf(bob.app, "mine"), "#t");
  EXPECT_EQ(leaf(bob.app, "page"), "#t");
  h.step();
  EXPECT_EQ(run("(state-set \"count\" (str (get-attr (chat-info (state-get \"tok\") \"\") "
                "\"count\")))"),
            "");
  EXPECT_EQ(leaf(bob.app, "count"), "41");
  cvc::ariadne::clear_action_intrinsics();
}
