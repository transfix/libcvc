/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::ariadne::holon (H1 slice 3): the server-authoritative,
// client-replicable clock + avatar-pose bundle. The properties worth pinning:
// each holon owns its OWN clock (two never share the app clock); a server holon
// and a client holon stepped in-process reproduce slice 2's prediction /
// reconciliation through the holon API (perfect prediction converges bit-for-bit;
// a server override snaps the client); a client never drives a peer; and a peer is
// interpolated through the holon's frame alpha, never overshooting.

#include <cstdint>
#include <cstring>
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/ariadne/holon.h>
#include <cvc/physics/body.h>
#include <cvc/physics/types.h>
#include <gtest/gtest.h>

using cvc::ariadne::avatar_intent;
using cvc::ariadne::holon;
using cvc::ariadne::holon_role;
using cvc::physics::body_state;
using cvc::physics::vec3;

namespace {
std::uint64_t bits(double d) {
  std::uint64_t u = 0;
  std::memcpy(&u, &d, sizeof(u));
  return u;
}
void expect_pos_bit_eq(const vec3 &a, const vec3 &b) {
  EXPECT_EQ(bits(a.x), bits(b.x));
  EXPECT_EQ(bits(a.y), bits(b.y));
  EXPECT_EQ(bits(a.z), bits(b.z));
}
} // namespace

// ── each holon owns its own clock ───────────────────────────────────────────

TEST(HolonTest, OwnsItsOwnClockNotTheAppClock) {
  holon a(holon_role::server);
  holon b(holon_role::server);
  EXPECT_NE(&a.clock(), &b.clock()) << "each holon must own a distinct world_clock";
  const double dt = a.clock().fixed_dt();
  a.advance(dt); // advance a by one quantum
  EXPECT_EQ(a.tick(), 1u);
  EXPECT_EQ(b.tick(), 0u) << "advancing one holon must not tick another";
}

// ── server + client: perfect prediction converges bit-for-bit ───────────────

TEST(HolonTest, ServerClientPerfectPredictionConverges) {
  holon server(holon_role::server);
  holon client(holon_role::client);
  server.add_participant("A");                 // authoritative on the server
  client.add_participant("A", /*local=*/true); // the client controls A, predicted
  const double dt = server.clock().fixed_dt();
  const avatar_intent mv{vec3{2.0, 0.0, 0.0}};

  for (int frame = 1; frame <= 10; ++frame) {
    server.set_intent("A", mv);
    client.set_intent("A", mv); // the client predicts the same input it sends
    server.advance(dt);
    client.advance(dt);
    // Replicate the authoritative state to the client and reconcile.
    client.apply_authoritative("A", server.state("A"), server.tick());
    // A matching prediction holds (no snap) and is bit-identical to authoritative.
    expect_pos_bit_eq(client.state("A").position, server.state("A").position);
  }
  EXPECT_GT(server.state("A").position.x, 0.0); // actually moved
}

// ── server override snaps the client onto the authoritative trajectory ──────

TEST(HolonTest, ServerOverrideReconcilesOnClient) {
  holon server(holon_role::server);
  holon client(holon_role::client);
  server.add_participant("A");
  client.add_participant("A", /*local=*/true);
  const double dt = server.clock().fixed_dt();
  const avatar_intent mv_client{vec3{2.0, 0.0, 0.0}}; // client thinks A moves +x
  const avatar_intent mv_server{vec3{0.0, 2.0, 0.0}}; // server says A moves +y

  for (int frame = 1; frame <= 10; ++frame) {
    client.set_intent("A", mv_client);
    server.set_intent("A", mv_server);
    client.advance(dt);
    server.advance(dt);
  }
  // The client predicted +x; the authoritative state is +y. Reconcile snaps it.
  client.apply_authoritative("A", server.state("A"), server.tick());
  expect_pos_bit_eq(client.state("A").position, server.state("A").position);
  EXPECT_GT(client.state("A").position.y, 0.1) << "client adopted the server's +y motion";
  EXPECT_NEAR(client.state("A").position.x, 0.0, 1e-12)
      << "the client's wrong +x was corrected away";
}

// ── a client cannot drive a peer ────────────────────────────────────────────

TEST(HolonTest, ClientCannotDriveAPeer) {
  holon client(holon_role::client);
  client.add_participant("peer", /*local=*/false);
  client.set_intent("peer", avatar_intent{vec3{9.0, 9.0, 9.0}}); // should be ignored
  const double dt = client.clock().fixed_dt();
  for (int i = 0; i < 5; ++i)
    client.advance(dt);
  // No authoritative samples arrived, so the peer stays at its start (origin); the
  // ignored intent did not move it.
  EXPECT_DOUBLE_EQ(client.state("peer").position.x, 0.0);
}

// ── a peer is interpolated through the holon's frame alpha, no overshoot ────

TEST(HolonTest, PeerInterpolatesThroughFrameAlpha) {
  holon client(holon_role::client);
  client.add_participant("B", /*local=*/false);
  const double dt = client.clock().fixed_dt();

  body_state s1;
  s1.position = vec3{0.0, 0.0, 0.0};
  body_state s2;
  s2.position = vec3{10.0, 0.0, 0.0};
  client.apply_authoritative("B", s1, 1);
  client.apply_authoritative("B", s2, 2); // window is now [s1, s2]

  // Advance half a quantum: no step falls due, but the frame alpha becomes ~0.5.
  client.advance(0.5 * dt);
  const double x_half = client.display_pose("B").position.x;
  EXPECT_GT(x_half, 0.0);
  EXPECT_LT(x_half, 10.0) << "interpolated strictly between the two samples";
  EXPECT_NEAR(x_half, 5.0, 1e-9);

  // A near-full-quantum frame approaches but never reaches/passes the newest sample.
  holon c2(holon_role::client);
  c2.add_participant("B", false);
  c2.apply_authoritative("B", s1, 1);
  c2.apply_authoritative("B", s2, 2);
  c2.advance(0.999 * dt);
  EXPECT_LT(c2.display_pose("B").position.x, 10.0) << "peer interpolation never overshoots newest";
}
