/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Avatar looks (room_avatar.h): the relay's catalog and each participant's choice in the room tree,
// and the look every client resolves from them -- the same for every viewer, the participant
// itself included. The two-node test uses room_intrinsics_test's harness (two apps + two shards +
// an inproc transport).

#include <algorithm>
#include <cvc/ariadne/conference/avatar_panel.h>
#include <cvc/ariadne/conference/room_avatar.h>
#include <cvc/ariadne/conference/room_intrinsics.h>
#include <cvc/ariadne/conference/room_write_gate.h>
#include <cvc/ariadne/holon.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace conf = cvc::ariadne::conference;

namespace {

bool same_rgb(const float a[3], const float b[3]) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

} // namespace

// The default catalog: the automatic colour and nothing worn come first (the defaults), ids are
// unique, and every part of every accessory lies inside the envelope.
TEST(RoomAvatar, TheDefaultCatalogFitsTheEnvelope) {
  const conf::avatar_catalog c = conf::default_avatar_catalog();
  ASSERT_FALSE(c.colors.empty());
  ASSERT_FALSE(c.accessories.empty());
  EXPECT_EQ(c.colors.front().id, "auto");
  EXPECT_TRUE(c.colors.front().automatic);
  EXPECT_EQ(c.accessories.front().id, "none");
  EXPECT_TRUE(c.accessories.front().parts.empty());
  for (const conf::avatar_accessory_option &a : c.accessories) {
    EXPECT_LE(a.parts.size(), conf::kMaxAccessoryParts) << a.id;
    for (const conf::avatar_part &p : a.parts)
      EXPECT_TRUE(conf::avatar_part_fits(p)) << a.id << " has a part outside the envelope";
  }
  for (std::size_t i = 0; i < c.colors.size(); ++i)
    for (std::size_t j = i + 1; j < c.colors.size(); ++j)
      EXPECT_NE(c.colors[i].id, c.colors[j].id);
}

// The catalog round-trips through the tree in its order; seeding is once only; a rewrite retires
// what it no longer offers.
TEST(RoomAvatar, TheCatalogRoundTripsAndSeedsOnce) {
  cvc::app app;
  EXPECT_EQ(conf::avatar_catalog_revision(app, "r"), 0u);
  EXPECT_TRUE(conf::read_avatar_catalog(app, "r").empty());
  ASSERT_TRUE(conf::seed_avatar_catalog(app, "r"));
  EXPECT_FALSE(conf::seed_avatar_catalog(app, "r")) << "a seeded room keeps its catalog";
  EXPECT_EQ(conf::avatar_catalog_revision(app, "r"), 1u);

  const conf::avatar_catalog want = conf::default_avatar_catalog();
  const conf::avatar_catalog got = conf::read_avatar_catalog(app, "r");
  ASSERT_EQ(got.colors.size(), want.colors.size());
  ASSERT_EQ(got.accessories.size(), want.accessories.size());
  for (std::size_t i = 0; i < want.colors.size(); ++i) {
    EXPECT_EQ(got.colors[i].id, want.colors[i].id) << "in the catalog's order";
    EXPECT_EQ(got.colors[i].label, want.colors[i].label);
    EXPECT_EQ(got.colors[i].automatic, want.colors[i].automatic);
    EXPECT_TRUE(same_rgb(got.colors[i].color, want.colors[i].color)) << want.colors[i].id;
  }
  for (std::size_t i = 0; i < want.accessories.size(); ++i) {
    const conf::avatar_accessory_option &g = got.accessories[i], &w = want.accessories[i];
    EXPECT_EQ(g.id, w.id);
    EXPECT_EQ(g.label, w.label);
    ASSERT_EQ(g.parts.size(), w.parts.size()) << w.id;
    for (std::size_t k = 0; k < w.parts.size(); ++k) {
      EXPECT_EQ(g.parts[k].shape, w.parts[k].shape);
      EXPECT_EQ(g.parts[k].tint, w.parts[k].tint);
      EXPECT_TRUE(same_rgb(g.parts[k].pos, w.parts[k].pos)) << w.id << " part " << k;
      EXPECT_TRUE(same_rgb(g.parts[k].size, w.parts[k].size)) << w.id << " part " << k;
      EXPECT_EQ(g.parts[k].radius, w.parts[k].radius);
    }
  }

  // A smaller catalog: the options it drops are no longer offered, and a shorter accessory reads
  // back without the parts its longer self had.
  conf::avatar_catalog fewer;
  fewer.colors.push_back(want.colors[1]);
  conf::avatar_accessory_option hat = want.accessories[2]; // the top hat: two parts
  hat.parts.resize(1);
  fewer.accessories.push_back(hat);
  conf::write_avatar_catalog(app, "r", fewer);
  EXPECT_EQ(conf::avatar_catalog_revision(app, "r"), 2u);
  const conf::avatar_catalog now = conf::read_avatar_catalog(app, "r");
  ASSERT_EQ(now.colors.size(), 1u);
  EXPECT_EQ(now.colors[0].id, want.colors[1].id);
  ASSERT_EQ(now.accessories.size(), 1u);
  EXPECT_EQ(now.accessories[0].parts.size(), 1u);
  EXPECT_THROW(
      conf::write_avatar_catalog(
          app, "r", conf::avatar_catalog{{conf::avatar_color_option{"a.b", "bad", false, {}}}, {}}),
      std::invalid_argument);
}

// What a choice resolves to: the automatic colour is the pid's, the same on every client and for
// the participant itself; a named colour overrides it; a tint takes the body's or the visor's
// colour; a choice the catalog lacks draws as the default; labels pick as ids do.
TEST(RoomAvatar, AChoiceResolvesTheSameForEveryone) {
  const conf::avatar_catalog c = conf::default_avatar_catalog();
  float auto_a[3], auto_b[3];
  conf::auto_avatar_color("alice", auto_a);
  conf::auto_avatar_color("bob", auto_b);
  EXPECT_FALSE(same_rgb(auto_a, auto_b)) << "people who never chose still differ";

  const conf::avatar_look unchosen = conf::resolve_avatar_look(c, {}, "alice");
  EXPECT_TRUE(same_rgb(unchosen.body, auto_a));
  EXPECT_EQ(unchosen.color_id, "auto");
  EXPECT_EQ(unchosen.accessory_id, "none");
  EXPECT_TRUE(unchosen.parts.empty());

  const conf::avatar_look teal = conf::resolve_avatar_look(c, {"teal", "cap"}, "alice");
  EXPECT_TRUE(same_rgb(teal.body, c.find_color("teal")->color));
  EXPECT_EQ(teal.accessory_id, "cap");
  ASSERT_FALSE(teal.parts.empty());
  EXPECT_TRUE(same_rgb(teal.parts[0].color, teal.visor)) << "the cap is tinted like the visor";
  EXPECT_NE(teal.key, unchosen.key);
  EXPECT_EQ(conf::resolve_avatar_look(c, {"Teal", "Cap"}, "alice").key, teal.key)
      << "a label picks what its id does";

  const conf::avatar_look pack = conf::resolve_avatar_look(c, {"crimson", "backpack"}, "x");
  bool body_tinted = false;
  for (const conf::avatar_part &p : pack.parts)
    body_tinted |= p.tint == "body" && same_rgb(p.color, pack.body);
  EXPECT_TRUE(body_tinted) << "a body-tinted part takes the chosen colour";

  const conf::avatar_look unknown = conf::resolve_avatar_look(c, {"plaid", "jetpack"}, "alice");
  EXPECT_EQ(unknown.key, unchosen.key) << "what the catalog does not offer draws as the default";

  const conf::avatar_look bare = conf::resolve_avatar_look(conf::avatar_catalog{}, {"teal"}, "bob");
  EXPECT_TRUE(same_rgb(bare.body, auto_b)) << "no catalog yet: the automatic colour";
  EXPECT_TRUE(bare.parts.empty());
}

// The envelope is the body grown 0.15 m on every side and 0.3 m on top: a part reaching past it is
// dropped, and so is any part past kMaxAccessoryParts.
TEST(RoomAvatar, PartsOutsideTheEnvelopeAreNotDrawn) {
  conf::avatar_part inside;
  inside.size[0] = inside.size[1] = inside.size[2] = 0.2f;
  inside.pos[0] = inside.pos[1] = 0.0f;
  inside.pos[2] = conf::kAvatarHeight + 0.1f;
  EXPECT_TRUE(conf::avatar_part_fits(inside));
  conf::avatar_part wide = inside;
  wide.size[0] = 2.0f * conf::kAccessoryReach + 0.01f;
  EXPECT_FALSE(conf::avatar_part_fits(wide)) << "wider than the envelope";
  conf::avatar_part tall = inside;
  tall.pos[2] = conf::kAccessoryTop;
  EXPECT_FALSE(conf::avatar_part_fits(tall)) << "above the envelope";
  conf::avatar_part sunk = inside;
  sunk.pos[2] = 0.05f;
  EXPECT_FALSE(conf::avatar_part_fits(sunk)) << "below the feet";
  conf::avatar_part ball;
  ball.shape = "sphere";
  ball.radius = 0.05f;
  ball.pos[2] = conf::kAccessoryTop - 0.05f;
  EXPECT_TRUE(conf::avatar_part_fits(ball));
  conf::avatar_part odd = inside;
  odd.shape = "teapot";
  EXPECT_FALSE(conf::avatar_part_fits(odd)) << "a shape no renderer draws";

  conf::avatar_catalog c;
  c.colors.push_back(conf::avatar_color_option{"auto", "Automatic", true, {}});
  conf::avatar_accessory_option big{"big", "Big", {inside, wide, tall}};
  for (int i = 0; i < 10; ++i)
    big.parts.push_back(inside);
  c.accessories.push_back(big);
  const conf::avatar_look look = conf::resolve_avatar_look(c, {"auto", "big"}, "p");
  EXPECT_EQ(look.parts.size(), conf::kMaxAccessoryParts)
      << "the two outside parts dropped, then the rest capped";
}

// The relay's policy (§5k) round-trips through the tree, bumping its revision; a reserve reads back
// as written, in whole joules.
TEST(RoomAvatar, ThePolicyAndAReserveRoundTrip) {
  cvc::app app;
  cvc::ariadne::avatar_energy_policy p;
  EXPECT_FALSE(conf::read_avatar_policy(app, "r", p)) << "none written yet";
  EXPECT_EQ(conf::avatar_policy_revision(app, "r"), 0u);
  p.enabled = true;
  p.force_max = 600.0;
  p.power_rest = 150.0;
  p.reserve_max = 12000.0;
  p.transport_cost = 0.25;
  p.empty = cvc::ariadne::avatar_energy_policy::when_empty::stop;
  p.recover_fraction = 0.5;
  conf::write_avatar_policy(app, "r", p);
  EXPECT_EQ(conf::avatar_policy_revision(app, "r"), 1u);
  cvc::ariadne::avatar_energy_policy q;
  ASSERT_TRUE(conf::read_avatar_policy(app, "r", q));
  EXPECT_TRUE(q.enabled);
  EXPECT_DOUBLE_EQ(q.force_max, 600.0);
  EXPECT_DOUBLE_EQ(q.power_rest, 150.0);
  EXPECT_DOUBLE_EQ(q.reserve_max, 12000.0);
  EXPECT_FLOAT_EQ(static_cast<float>(q.transport_cost), 0.25f);
  EXPECT_EQ(q.empty, cvc::ariadne::avatar_energy_policy::when_empty::stop);
  EXPECT_DOUBLE_EQ(q.recover_fraction, 0.5);
  p.enabled = false;
  conf::write_avatar_policy(app, "r", p);
  EXPECT_EQ(conf::avatar_policy_revision(app, "r"), 2u);
  ASSERT_TRUE(conf::read_avatar_policy(app, "r", q));
  EXPECT_FALSE(q.enabled);

  double j = 0.0;
  std::uint64_t t = 0;
  EXPECT_FALSE(conf::read_avatar_reserve(app, "r", "A", j, t));
  conf::write_avatar_reserve(app, "r", "A", 1234.4, 77);
  ASSERT_TRUE(conf::read_avatar_reserve(app, "r", "A", j, t));
  EXPECT_DOUBLE_EQ(j, 1234.0);
  EXPECT_EQ(t, 77u);
}

// Across the wire: the relay's catalog reaches a client, the client's choice reaches the relay and
// the other client, and both clients resolve the chooser's look the same -- the chooser included.
// The relay's write gate lets a participant write its own choice and nothing else.
TEST(RoomAvatar, AChoiceReplicatesAndOnlyItsOwnerMayWriteIt) {
  cvc::app aS, aA, aB;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard sS(aS, "C", "S"), sA(aA, "C", "A"), sB(aB, "C", "B");
  for (cvc::state_cluster_shard *s : {&sS, &sA, &sB}) {
    s->attach();
    tp.register_shard(s);
  }

  ASSERT_TRUE(conf::seed_avatar_catalog(aS, "room"));
  conf::room_join(aA, "room", "A", "Alice");
  conf::room_join(aB, "room", "B", "Bob");
  tp.pump_all();
  EXPECT_EQ(conf::read_avatar_catalog(aA, "room").colors.size(),
            conf::default_avatar_catalog().colors.size())
      << "the relay's catalog reached the client";

  conf::write_avatar_choice(aA, "room", "A", {"violet", "halo"});
  tp.pump_all();
  for (cvc::app *a : {&aS, &aB}) {
    const conf::avatar_choice ch = conf::read_avatar_choice(*a, "room", "A");
    EXPECT_EQ(ch.color, "violet");
    EXPECT_EQ(ch.accessory, "halo");
  }
  const conf::avatar_look seen_by_a = conf::resolve_avatar_look(
      conf::read_avatar_catalog(aA, "room"), conf::read_avatar_choice(aA, "room", "A"), "A");
  const conf::avatar_look seen_by_b = conf::resolve_avatar_look(
      conf::read_avatar_catalog(aB, "room"), conf::read_avatar_choice(aB, "room", "A"), "A");
  EXPECT_EQ(seen_by_a.key, seen_by_b.key) << "Alice sees herself as Bob sees her";
  EXPECT_EQ(seen_by_a.accessory_id, "halo");

  const conf::room_write_gate gate("room");
  const std::string own = conf::room_participant_path("room", "A") + cvc::state::SEPARATOR +
                          "avatar" + cvc::state::SEPARATOR + "color";
  const std::string theirs = conf::room_participant_path("room", "B") + cvc::state::SEPARATOR +
                             "avatar" + cvc::state::SEPARATOR + "color";
  const std::string catalog = conf::room_avatars_path("room") + cvc::state::SEPARATOR + "rev";
  EXPECT_TRUE(gate.may_write("A", own, cvc::state_mutation_op::set_value))
      << "a participant writes its own choice";
  EXPECT_FALSE(gate.may_write("A", theirs, cvc::state_mutation_op::set_value))
      << "but not someone else's";
  EXPECT_FALSE(gate.may_write("A", catalog, cvc::state_mutation_op::set_value))
      << "nor the catalog: only the relay supplies it";
  const std::string reserve = conf::room_participant_path("room", "A") + cvc::state::SEPARATOR +
                              "energy" + cvc::state::SEPARATOR + "reserve";
  EXPECT_FALSE(gate.may_write("A", reserve, cvc::state_mutation_op::set_value))
      << "nor its own reserve: the relay counts it";

  for (cvc::state_cluster_shard *s : {&sS, &sA, &sB}) {
    tp.unregister_shard(s);
    s->detach();
  }
}

// The Avatar window's model (avatar_panel.h): from the client holon and the room tree it publishes
// the look, the body, the relay's policy, the reserve, balanced books and a history that grows a
// sample per period, under <prefix>.self.avatar.
TEST(RoomAvatar, ThePanelPublishesTheBudgetLedgerAndHistory) {
  cvc::app app;
  conf::seed_avatar_catalog(app, "room");
  conf::write_avatar_choice(app, "room", "A", {"teal", "cap"});
  cvc::ariadne::holon_config cfg;
  cfg.physics = true;
  cvc::ariadne::holon client(cvc::ariadne::holon_role::client, cfg);
  conf::avatar_panel panel(/*period=*/0.25, /*capacity=*/8);
  client.add_participant("A", /*local=*/true);
  panel.update(client, app, "room", "A", 0.0);
  if (!client.physics()) {
    EXPECT_FALSE(panel.ready()) << "no physics, no energy to show";
    GTEST_SKIP() << "no Jolt backend in this build";
  }
  cvc::ariadne::avatar_static floor;
  floor.center = cvc::physics::vec3{0.0, 0.0, -0.1};
  floor.half_extents = cvc::physics::vec3{200.0, 200.0, 0.1};
  client.set_statics({floor}, 1);
  client.set_intent("A", cvc::ariadne::avatar_intent{cvc::physics::vec3{4.0, 0.0, 0.0}});
  const double dt = client.clock().fixed_dt();
  for (int i = 0; i < 240; ++i) { // 2 s: eight samples of a quarter second
    client.advance(dt);
    panel.update(client, app, "room", "A", dt);
  }
  ASSERT_TRUE(panel.ready());
  EXPECT_EQ(panel.reserve_history().size(), 8u) << "capped at its capacity";
  EXPECT_GT(panel.power_history().back(), 500.0) << "walking at 4 m/s spends ~549 W";
  panel.publish(app, "ui.docs.conf");
  const auto get = [&](const std::string &k) {
    cvc::state *n = app.root().findDescendant("ui.docs.conf.self.avatar." + k);
    return n ? n->value() : std::string("<missing>");
  };
  EXPECT_EQ(get("ready"), "1");
  EXPECT_EQ(get("look.color"), "Teal");
  EXPECT_EQ(get("look.accessory"), "Cap");
  EXPECT_EQ(get("controller"), "walker");
  EXPECT_EQ(get("body"), "0.50 x 0.50 x 1.70 m box, 70 kg");
  EXPECT_EQ(get("policy.reserve"), "30.0 kJ");
  EXPECT_EQ(get("policy.sustains"), "2.2 m/s on resting power");
  EXPECT_EQ(get("energy.state"), "spending");
  EXPECT_LT(std::stod(get("energy.fraction")), 1.0);
  EXPECT_EQ(get("ledger.start"), "30.0 kJ");
  EXPECT_EQ(get("ledger.residual"), "0.000 J") << "the books balance";
  EXPECT_NE(get("ledger.work").find("J"), std::string::npos);
  EXPECT_GT(std::stod(get("ledger.share.carried")), 0.5) << "a steady walk is mostly carrying";
  const std::string reserve = get("history.reserve");
  EXPECT_EQ(std::count(reserve.begin(), reserve.end(), ','), 7) << "eight samples";
  EXPECT_EQ(get("history.window"), "the last 2 s");
}

// The panel's look follows the room tree without re-reading the catalog every frame: it reads the
// catalog once, then again only when its revision moves, so frames that change nothing read none of
// it, while the catalog's arrival, a new choice and a rewritten catalog each reach the published
// look on the next update -- the same labels a fresh read of the tree resolves.
TEST(RoomAvatar, ThePanelRereadsTheCatalogOnlyWhenItChanges) {
  cvc::app app;
  conf::write_avatar_choice(app, "room", "A", {"teal", "cap"});
  cvc::ariadne::holon client(cvc::ariadne::holon_role::client);
  client.add_participant("A", /*local=*/true);
  conf::avatar_panel panel;
  const auto shown = [&] {
    panel.publish(app, "ui.docs.conf");
    const auto get = [&](const std::string &k) {
      cvc::state *n = app.root().findDescendant("ui.docs.conf.self.avatar.look." + k);
      return n ? n->value() : std::string("<missing>");
    };
    return get("color") + " / " + get("accessory");
  };
  const auto fresh = [&] { // what update() resolved every frame before it kept the catalog
    const conf::avatar_catalog cat = conf::read_avatar_catalog(app, "room");
    const conf::avatar_look look =
        conf::resolve_avatar_look(cat, conf::read_avatar_choice(app, "room", "A"), "A");
    const conf::avatar_color_option *c = cat.find_color(look.color_id);
    const conf::avatar_accessory_option *a = cat.find_accessory(look.accessory_id);
    return (c ? c->label : std::string()) + " / " + (a ? a->label : std::string());
  };
  const auto frames = [&](int n) {
    for (int i = 0; i < n; ++i)
      panel.update(client, app, "room", "A", 1.0 / 60.0);
  };

  frames(5); // joined before the relay's catalog arrived: no labels yet
  EXPECT_EQ(shown(), " / ");
  EXPECT_EQ(panel.catalog_reads(), 1u) << "five frames of nothing, one read";

  conf::seed_avatar_catalog(app, "room"); // the catalog arrives
  frames(1);
  EXPECT_EQ(shown(), "Teal / Cap");
  EXPECT_EQ(shown(), fresh());
  frames(60);
  EXPECT_EQ(panel.catalog_reads(), 2u) << "a second of unchanged frames reads no catalog";

  conf::write_avatar_choice(app, "room", "A", {"gold", "halo"}); // our pick
  frames(1);
  EXPECT_EQ(shown(), "Gold / Halo");
  EXPECT_EQ(shown(), fresh());
  EXPECT_EQ(panel.catalog_reads(), 2u) << "a choice needs no catalog read";

  // The relay rewrites the catalog: gold relabelled, the halo withdrawn (so the first accessory).
  conf::avatar_catalog cat = conf::default_avatar_catalog();
  for (conf::avatar_color_option &c : cat.colors)
    if (c.id == "gold")
      c.label = "Sunflower";
  cat.accessories.erase(
      std::remove_if(cat.accessories.begin(), cat.accessories.end(),
                     [](const conf::avatar_accessory_option &a) { return a.id == "halo"; }),
      cat.accessories.end());
  conf::write_avatar_catalog(app, "room", cat);
  frames(1);
  EXPECT_EQ(shown(), "Sunflower / None");
  EXPECT_EQ(shown(), fresh());
  frames(60);
  EXPECT_EQ(panel.catalog_reads(), 3u);

  // Another room is another catalog (none written there yet).
  panel.update(client, app, "elsewhere", "A", 1.0 / 60.0);
  EXPECT_EQ(shown(), " / ");
  EXPECT_EQ(panel.catalog_reads(), 4u);
}
