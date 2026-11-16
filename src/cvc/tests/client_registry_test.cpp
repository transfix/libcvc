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

// client_registry (docs/roadmap/ARISRV_IDENTITY.md, ID1): the client id, the connection log and
// its active flags, the caps, usernames, and what it writes into the tree.

#include <cvc/ariadne/conference/client_registry.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_participant_path
#include <cvc/ariadne/conference/room_net.h>        // room_device_tag
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_link_stats.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace conf = cvc::ariadne::conference;
namespace se = cvc::state_exec;
using reg_t = conf::client_registry;

namespace {

reg_t::options opts(std::size_t max_clients = 4096, std::size_t max_connections = 32) {
  reg_t::options o;
  o.room = "r";
  o.salt = "test-salt";
  o.max_clients = max_clients;
  o.max_connections = max_connections;
  return o;
}

// An accepted, identified link: a relay's connection to the client PID.
cvc::state_link_report link(std::uint64_t serial, const std::string &pid,
                            const std::string &device = "dev-1",
                            const std::string &peer = "ipv4:203.0.113.7:50123",
                            const std::string &local = "192.168.1.20") {
  cvc::state_link_report r;
  r.transport = "grpc";
  r.connection = serial;
  r.remote_node_id = pid;
  r.remote_device_id = device;
  r.remote_address = peer;
  r.remote_local_address = local;
  r.accepted = true;
  r.holds_id = true;
  return r;
}

std::string value_at(cvc::app &app, const std::string &path) {
  cvc::state *n = app.root().findDescendant(path);
  return n ? n->value() : std::string("<absent>");
}

std::string field(const se::value_t &v, const std::string &key) {
  const se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v);
  if (!d || !*d)
    return "<not a dict>";
  for (const auto &[k, x] : **d)
    if (k == key) {
      if (const std::string *s = std::get_if<std::string>(&x.v))
        return *s;
      if (const bool *b = std::get_if<bool>(&x.v))
        return *b ? "true" : "false";
      return "<other>";
    }
  return "<missing>";
}

} // namespace

TEST(ClientRegistry, TheIdIsTheDeviceAndAddressesAreItsFallback) {
  reg_t reg(opts());
  const std::string a = reg.client_id("dev-1", "10.0.0.2", "203.0.113.7");
  EXPECT_EQ(a.size(), 32u);
  // Another network: the same device is the same client (D1).
  EXPECT_EQ(a, reg.client_id("dev-1", "192.168.5.9", "198.51.100.1"));
  EXPECT_NE(a, reg.client_id("dev-2", "10.0.0.2", "203.0.113.7"));
  // No device id: the addresses are hashed instead, and differ from any device's id.
  const std::string b = reg.client_id("", "10.0.0.2", "203.0.113.7");
  EXPECT_NE(a, b);
  EXPECT_NE(b, reg.client_id("", "10.0.0.3", "203.0.113.7"));
  EXPECT_NE(b, reg.client_id("", "10.0.0.2", "203.0.113.8"));
  // The fields can't run into each other.
  EXPECT_NE(reg.client_id("", "1.2", "3.4"), reg.client_id("", "1.23", ".4"));
  // Another salt (another relay, or a restart) gives another id.
  reg_t::options o = opts();
  o.salt = "other";
  EXPECT_NE(a, reg_t(o).client_id("dev-1", "10.0.0.2", "203.0.113.7"));
  // A random salt when none is given.
  reg_t::options r1 = opts(), r2 = opts();
  r1.salt.clear();
  r2.salt.clear();
  EXPECT_NE(reg_t(r1).client_id("dev-1", "", ""), reg_t(r2).client_id("dev-1", "", ""));
}

TEST(ClientRegistry, AConnectionOpensAtItsHelloAndClosesWhenItGoes) {
  reg_t reg(opts());
  // A dialed link, and one that has not sent its Hello, are not clients.
  cvc::state_link_report dialed = link(1, "relay2");
  dialed.accepted = false;
  cvc::state_link_report early = link(2, "");
  reg.update({dialed, early}, 1.0, "2026-11-16T00:00:01Z");
  EXPECT_TRUE(reg.records().empty());

  reg.update({link(3, "bob")}, 2.0, "2026-11-16T00:00:02Z");
  ASSERT_EQ(reg.records().size(), 1u);
  const std::string id = reg.identity_key("bob");
  EXPECT_EQ(id, reg.client_id("dev-1", "", ""));
  const reg_t::record *rec = reg.find(id);
  ASSERT_NE(rec, nullptr);
  EXPECT_TRUE(rec->active);
  EXPECT_EQ(rec->username, "guest-" + id.substr(1, 4));
  EXPECT_EQ(id.front(), 'c');
  EXPECT_TRUE(cvc::state::isValidStateName(id));
  EXPECT_EQ(rec->first_seen, "2026-11-16T00:00:02Z");
  ASSERT_EQ(rec->connections.size(), 1u);
  const reg_t::connection &c = rec->connections.front();
  EXPECT_EQ(c.pid, "bob");
  EXPECT_EQ(c.transport, "grpc");
  EXPECT_EQ(c.public_address, "203.0.113.7"); // without the scheme and the port
  EXPECT_EQ(c.local_address, "192.168.1.20");
  EXPECT_EQ(c.opened, "2026-11-16T00:00:02Z");
  EXPECT_TRUE(c.active);
  EXPECT_TRUE(c.closed.empty());

  // A later pass with it still there changes nothing but when it was last seen.
  reg.update({link(3, "bob")}, 3.0, "2026-11-16T00:00:03Z");
  EXPECT_EQ(reg.find(id)->connections.size(), 1u);
  EXPECT_EQ(reg.find(id)->last_seen, "2026-11-16T00:00:03Z");

  reg.update({}, 4.0, "2026-11-16T00:00:04Z");
  rec = reg.find(id);
  ASSERT_NE(rec, nullptr); // kept after it leaves
  EXPECT_FALSE(rec->active);
  EXPECT_FALSE(rec->connections.front().active);
  EXPECT_EQ(rec->connections.front().closed, "2026-11-16T00:00:04Z");
  EXPECT_TRUE(reg.identity_key("bob").empty());
  EXPECT_EQ(reg.record_of("bob"), nullptr);
}

TEST(ClientRegistry, AReturningClientKeepsItsRecordAndName) {
  reg_t reg(opts());
  reg.update({link(1, "bob")}, 1.0, "t1");
  reg_t::refusal why;
  ASSERT_TRUE(reg.set_username("bob", "Bob", 1.0, why)) << why.message;
  reg.update({}, 2.0, "t2");
  // The same device from another network, under another pid.
  reg.update({link(2, "bob2", "dev-1", "ipv4:198.51.100.1:4000", "10.9.9.9")}, 3.0, "t3");
  const reg_t::record *rec = reg.record_of("bob2");
  ASSERT_NE(rec, nullptr);
  EXPECT_EQ(rec->username, "Bob");
  EXPECT_EQ(rec->first_seen, "t1");
  ASSERT_EQ(rec->connections.size(), 2u);
  EXPECT_FALSE(rec->connections[0].active);
  EXPECT_TRUE(rec->connections[1].active);
  EXPECT_EQ(rec->connections[1].public_address, "198.51.100.1");
  EXPECT_EQ(rec->connections[1].seq, rec->connections[0].seq + 1);
}

TEST(ClientRegistry, AReturningClientWhoseNameWasTakenGetsASuffix) {
  reg_t reg(opts());
  reg_t::refusal why;
  reg.update({link(1, "a", "dev-a")}, 1.0, "t1");
  ASSERT_TRUE(reg.set_username("a", "ana", 1.0, why));
  reg.update({}, 2.0, "t2");
  reg.update({link(2, "b", "dev-b")}, 3.0, "t3");
  ASSERT_TRUE(reg.set_username("b", "ana", 3.0, why)) << why.message; // a has left
  reg.update({link(2, "b", "dev-b"), link(3, "a2", "dev-a")}, 4.0, "t4");
  EXPECT_EQ(reg.record_of("b")->username, "ana");
  EXPECT_EQ(reg.record_of("a2")->username, "ana-2");
}

TEST(ClientRegistry, TheLogAndTheRecordsAreCapped) {
  reg_t reg(opts(/*max_clients=*/2, /*max_connections=*/3));
  for (std::uint64_t i = 1; i <= 5; ++i) {
    reg.update({link(i, "bob")}, double(i), "t" + std::to_string(i));
    reg.update({}, double(i) + 0.5, "c" + std::to_string(i));
  }
  const reg_t::record *rec = reg.find(reg.client_id("dev-1", "", ""));
  ASSERT_NE(rec, nullptr);
  ASSERT_EQ(rec->connections.size(), 3u); // the newest three
  EXPECT_EQ(rec->connections.front().seq, 3u);
  EXPECT_EQ(rec->connections.back().seq, 5u);
  EXPECT_EQ(reg.records().size(), 1u); // one record, however often it reconnects

  // Two more clients, both gone by the next: the one seen longest ago is dropped.
  reg.update({link(10, "c", "dev-c")}, 10.0, "t10");
  reg.update({}, 11.0, "t11");
  reg.update({link(12, "d", "dev-d")}, 12.0, "t12");
  EXPECT_EQ(reg.records().size(), 2u);
  EXPECT_EQ(reg.find(reg.client_id("dev-1", "", "")), nullptr); // bob, seen at 5.0
  EXPECT_NE(reg.find(reg.client_id("dev-c", "", "")), nullptr);
  EXPECT_NE(reg.find(reg.client_id("dev-d", "", "")), nullptr);

  // An active record is never dropped: the cap bounds the ones that have left.
  reg_t busy(opts(/*max_clients=*/1));
  busy.update({link(1, "x", "dev-x"), link(2, "y", "dev-y")}, 1.0, "t");
  EXPECT_EQ(busy.records().size(), 2u);
}

TEST(ClientRegistry, UsernamesAreCleanedAndUnique) {
  std::string out;
  EXPECT_TRUE(reg_t::clean_username("  ana  ", out));
  EXPECT_EQ(out, "ana");
  EXPECT_TRUE(reg_t::clean_username("Jos\xC3\xA9", out)); // UTF-8 is fine
  EXPECT_EQ(out, "Jos\xC3\xA9");
  EXPECT_FALSE(reg_t::clean_username("", out));
  EXPECT_FALSE(reg_t::clean_username("   ", out));
  EXPECT_TRUE(reg_t::clean_username(std::string(32, 'a'), out));
  EXPECT_FALSE(reg_t::clean_username(std::string(33, 'a'), out));
  EXPECT_FALSE(reg_t::clean_username("a\nb", out));         // C0 control
  EXPECT_FALSE(reg_t::clean_username("a\x7F", out));        // DEL
  EXPECT_FALSE(reg_t::clean_username("a\xC2\x85", out));    // C1 control (NEL)
  EXPECT_FALSE(reg_t::clean_username("a\xC3", out));        // cut off
  EXPECT_FALSE(reg_t::clean_username("a\xC0\xAF", out));    // overlong
  EXPECT_FALSE(reg_t::clean_username("\xED\xA0\x80", out)); // a surrogate

  reg_t reg(opts());
  reg.update({link(1, "a", "dev-a"), link(2, "b", "dev-b")}, 1.0, "t");
  reg_t::refusal why;
  EXPECT_FALSE(reg.set_username("nobody", "x", 1.0, why));
  EXPECT_EQ(why.code, "unavailable");
  EXPECT_FALSE(reg.set_username("a", "a\tb", 1.0, why));
  EXPECT_EQ(why.code, "invalid");
  ASSERT_TRUE(reg.set_username("a", "Ana", 1.0, why));
  // Taken by another active client, in any case: refused, or with a suffix when asked.
  EXPECT_FALSE(reg.set_username("b", "ana", 1.0, why));
  EXPECT_EQ(why.code, "invalid");
  EXPECT_NE(why.message.find("taken"), std::string::npos);
  ASSERT_TRUE(reg.set_username("b", "ana", 1.0, why, /*make_unique=*/true));
  EXPECT_EQ(reg.record_of("b")->username, "ana-2");
  // A client may change the case of its own name.
  EXPECT_TRUE(reg.set_username("a", "ANA", 1.0, why));
  EXPECT_EQ(reg.record_of("a")->username, "ANA");
  // The suffix still fits in 32 bytes, and never splits a UTF-8 sequence.
  const std::string longname = std::string(30, 'x') + "\xC3\xA9"; // 32 bytes
  ASSERT_TRUE(reg.set_username("a", longname, 2.0, why)) << why.message;
  ASSERT_TRUE(reg.set_username("b", longname, 2.0, why, true)) << why.message;
  EXPECT_EQ(reg.record_of("b")->username, std::string(30, 'x') + "-2");
}

TEST(ClientRegistry, RenamesAreRateLimited) {
  reg_t reg(opts());
  reg.update({link(1, "a")}, 0.0, "t");
  reg_t::refusal why;
  for (int i = 0; i < 4; ++i)
    ASSERT_TRUE(reg.set_username("a", "n" + std::to_string(i), 1.0 + i, why)) << i;
  EXPECT_FALSE(reg.set_username("a", "n9", 5.0, why));
  EXPECT_EQ(why.code, "rate_limited");
  // Asking for the name it has is not a rename.
  EXPECT_TRUE(reg.set_username("a", "n3", 5.0, why));
  // A minute after the first, one more.
  EXPECT_TRUE(reg.set_username("a", "n9", 61.5, why)) << why.message;
}

TEST(ClientRegistry, APidFollowsTheLinkThatHoldsIt) {
  reg_t reg(opts());
  cvc::state_link_report first = link(1, "bob", "dev-1");
  reg.update({first}, 1.0, "t1");
  EXPECT_EQ(reg.identity_key("bob"), reg.client_id("dev-1", "", ""));
  // Another device claims bob and takes it over (its holder went quiet).
  first.holds_id = false;
  cvc::state_link_report second = link(2, "bob", "dev-2");
  reg.update({first, second}, 2.0, "t2");
  EXPECT_EQ(reg.identity_key("bob"), reg.client_id("dev-2", "", ""));
  // A pid no link holds has no client.
  second.holds_id = false;
  reg.update({first, second}, 3.0, "t3");
  EXPECT_TRUE(reg.identity_key("bob").empty());
}

TEST(ClientRegistry, AVerifiedDeviceGetsItsRoomTag) {
  reg_t reg(opts());
  cvc::state_link_report r = link(1, "bob", "dev-1");
  reg.update({r}, 1.0, "t");
  EXPECT_FALSE(reg.record_of("bob")->verified);
  EXPECT_TRUE(reg.record_of("bob")->device.empty());
  r.device_verified = true; // its proof came after the Hello
  reg.update({r}, 2.0, "t");
  EXPECT_TRUE(reg.record_of("bob")->verified);
  EXPECT_EQ(reg.record_of("bob")->device, conf::room_device_tag("r", "dev-1"));

  const se::value_t who = reg.whoami("bob");
  EXPECT_EQ(field(who, "client_id"), reg.identity_key("bob"));
  EXPECT_EQ(field(who, "username"), reg.record_of("bob")->username);
  EXPECT_EQ(field(who, "verified"), "true");
  EXPECT_EQ(field(who, "device"), conf::room_device_tag("r", "dev-1"));
  EXPECT_EQ(field(who, "admin"), "false");
  EXPECT_TRUE(std::holds_alternative<std::monostate>(reg.whoami("nobody").v));
}

TEST(ClientRegistry, ItWritesTheNameAndMirrorsTheRecordsPrivately) {
  cvc::app app;
  reg_t reg(opts(/*max_clients=*/4096, /*max_connections=*/2));
  reg.update({link(1, "bob")}, 1.0, "t1");
  reg_t::refusal why;
  ASSERT_TRUE(reg.set_username("bob", "Bob", 1.0, why));
  reg.write(app, {"bob"});
  const std::string name = conf::room_participant_path("r", "bob") + ".profile.name";
  EXPECT_EQ(value_at(app, name), "Bob");
  // A pid that isn't present gets no name written.
  reg.update({link(1, "bob"), link(2, "eve", "dev-e")}, 2.0, "t2");
  reg.write(app, {"bob"});
  EXPECT_EQ(value_at(app, conf::room_participant_path("r", "eve") + ".profile.name"), "<absent>");

  const std::string id = reg.identity_key("bob");
  const std::string base = std::string(reg_t::kRoot) + "." + id;
  EXPECT_EQ(value_at(app, base + ".username"), "Bob");
  EXPECT_EQ(value_at(app, base + ".active"), "1");
  EXPECT_EQ(value_at(app, base + ".verified"), "0");
  EXPECT_EQ(value_at(app, base + ".first_seen"), "t1");
  EXPECT_EQ(value_at(app, base + ".connections.n1.pid"), "bob");
  EXPECT_EQ(value_at(app, base + ".connections.n1.public"), "203.0.113.7");
  EXPECT_EQ(value_at(app, base + ".connections.n1.local"), "192.168.1.20");
  EXPECT_EQ(value_at(app, base + ".connections.n1.active"), "1");

  // A client's own write is overwritten by the next pass.
  app.root()(name).value("Mallory");
  reg.write(app, {"bob"});
  EXPECT_EQ(value_at(app, name), "Bob");

  // It leaves, and comes back twice: its log keeps two, and the one trimmed leaves the tree.
  reg.update({}, 3.0, "t3");
  reg.write(app, {});
  EXPECT_EQ(value_at(app, base + ".active"), "0");
  EXPECT_EQ(value_at(app, base + ".connections.n1.closed"), "t3");
  EXPECT_EQ(value_at(app, base + ".connections.n1.active"), "0");
  reg.update({link(3, "bob")}, 4.0, "t4");
  reg.update({}, 5.0, "t5");
  reg.update({link(4, "bob")}, 6.0, "t6");
  reg.write(app, {"bob"});
  EXPECT_EQ(value_at(app, base + ".connections.n1.pid"), "<absent>");
  EXPECT_EQ(value_at(app, base + ".connections.n2.closed"), "t5");
  EXPECT_EQ(value_at(app, base + ".connections.n3.active"), "1");
}

TEST(ClientRegistry, AnEvictedRecordLeavesTheTree) {
  cvc::app app;
  reg_t reg(opts(/*max_clients=*/1));
  reg.update({link(1, "a", "dev-a")}, 1.0, "t1");
  reg.update({}, 2.0, "t2");
  reg.write(app, {});
  const std::string a = std::string(reg_t::kRoot) + "." + reg.client_id("dev-a", "", "");
  EXPECT_EQ(value_at(app, a + ".active"), "0");
  reg.update({link(2, "b", "dev-b")}, 3.0, "t3");
  reg.write(app, {"b"});
  EXPECT_EQ(value_at(app, a + ".active"), "<absent>");
  EXPECT_EQ(
      value_at(app, std::string(reg_t::kRoot) + "." + reg.client_id("dev-b", "", "") + ".active"),
      "1");
}

TEST(ClientRegistry, AddressHostDropsTheSchemeBracketsAndPort) {
  EXPECT_EQ(reg_t::address_host("ipv4:127.0.0.1:50051"), "127.0.0.1");
  EXPECT_EQ(reg_t::address_host("ipv6:[::1]:50051"), "::1");
  EXPECT_EQ(reg_t::address_host("ipv6:%5B::1%5D:50051"), "::1");
  EXPECT_EQ(reg_t::address_host("203.0.113.7:8080"), "203.0.113.7");
  EXPECT_EQ(reg_t::address_host("[2001:db8::1]:443"), "2001:db8::1");
  EXPECT_EQ(reg_t::address_host("2001:db8::1"), "2001:db8::1");
  EXPECT_EQ(reg_t::address_host("ipv6:%5Bfe80::1%25eth0%5D:1"), "fe80::1%eth0");
  EXPECT_EQ(reg_t::address_host(""), "");
}

TEST(ClientRegistry, UtcIsoIsToTheSecond) {
  const auto epoch = std::chrono::system_clock::from_time_t(0);
  EXPECT_EQ(reg_t::utc_iso(epoch), "1970-01-01T00:00:00Z");
  EXPECT_EQ(reg_t::utc_iso(epoch + std::chrono::seconds(86400 + 3661)), "1970-01-02T01:01:01Z");
}
