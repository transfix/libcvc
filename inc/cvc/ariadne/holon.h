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

#ifndef __CVC_ARIADNE_HOLON_H__
#define __CVC_ARIADNE_HOLON_H__

#include <cstdint>
#include <cvc/ariadne/avatar_physics.h>
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/core/clock_driver.h>
#include <cvc/core/namespace.h>
#include <cvc/core/world_clock.h>
#include <cvc/physics/body.h>
#include <cvc/physics/pose_tracker.h>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

// ----------------
// cvc::ariadne::holon (H1 slice 3)
// ----------------
// A holon is a server-authoritative, client-replicable bundle of ONE world_clock
// and the participant avatars that live on it. It is the object the roadmap's §5h
// names: the server drives the clock and owns every avatar's authoritative pose; a
// client runs a replica -- it PREDICTS its own avatar locally (slice 2) and
// RECONCILES to the authoritative state when it arrives, and INTERPOLATES peers.
//
// This slice is the headless clock + avatar-pose core. The holon owns its own
// world_clock and cvc::clock_driver BY VALUE (never the app-global clock, so two
// holons in one process never fight over time), and it drives avatar integration
// as a clock_driver STEP observer and presentation as a FRAME observer -- the same
// step-then-present shape tick_scene uses. The shared SCENE geometry, the Jolt
// cvc::physics::world (replacing the linear avatar_sim behind the same shape), and
// the cvcGL rendering of all this layer on top in later slices; none of that is
// here, so the model stays headless and deterministically testable.
//
// Authority (slice 4 adds the verb path): a client NEVER writes another avatar's
// pose; it writes only its own intent, the server validates/vetoes/overrides and
// is the sole writer of the authoritative pose, and the client respects that on
// reconcile. This class embodies that split -- set_intent on a client only drives
// its own (local) avatar's prediction; apply_authoritative is how the server's
// word reaches the client.

enum class holon_role { server, client };

struct holon_config {
  // A holon is advanced at its HOST's cadence, not a render loop's: arisrv pumps its server holon
  // at ~5 Hz (24 quanta due per advance at 120 Hz) and a throttled browser tab may run at 10 fps.
  // world_clock's default catch-up cap (8 quanta, sized for a 60+ Hz frame loop) would silently
  // drop the rest -- running the room's avatars, and its tick, at a third of real time -- so a
  // holon's clock allows 0.5 s of catch-up per advance.
  holon_config() { clock.max_steps_per_advance = 60; }

  cvc::world_clock::config clock; // the holon's own quantum / scale / mode
  double reconcile_tol = 0.05;    // metres; the client's snap threshold (slice 2)
  // CLIENT: how far (in ticks) the replica's clock may drift from the server's tick before
  // apply_authoritative re-aligns it. 60 = 0.5 s at the default 120 Hz. See apply_authoritative.
  std::uint64_t max_skew_ticks = 60;
  // CLIENT with a lead (set_lead_ticks > 0): how far the replica may fall BEHIND its lead target
  // (late inputs: the server would apply them after their tick, a correction each time) or run
  // AHEAD of it (only delays what peers see) before it re-aligns.
  std::uint64_t lead_slack_behind_ticks = 6;
  std::uint64_t lead_slack_ahead_ticks = 24;
  // CLIENT: the longest walk between two peer samples, in ticks. A server that publishes only
  // what changed (write_pose's heartbeat) leaves a long gap before a peer's first sample after
  // standing still; walking that sample in over the whole gap would leave the peer far behind.
  std::uint64_t max_peer_span_ticks = 30;
  // SERVER: how far ahead of its own tick a stamped intent may be (queue_intent); a stamp
  // further out (a client whose clock is off) takes effect at once instead of waiting.
  std::uint64_t max_intent_lead_ticks = 240;
  // Rigid-body avatars in a Jolt world (avatar_world): mass, inertia, gravity, collisions with
  // each other and the room's statics (set_statics). Off = the kinematic avatar_sim (no
  // collisions). A build without the Jolt backend falls back to avatar_sim with a warning
  // (physics() reports which is running).
  bool physics = false;
  avatar_body body; // with physics: every avatar's body
  // SERVER with physics: an avatar that fell off the world (below this z) respawns where it
  // joined.
  double respawn_below_z = -10.0;
  // The VETO (sanitize_intent): the fastest an intent may ask an avatar to move horizontally (m/s;
  // a faster ask is scaled down to it), and whether it may ask to move vertically at all (avatars
  // walk; gravity, not the client, owns z).
  double max_speed = 4.0;
  bool allow_vertical = false;
};

class holon {
public:
  explicit holon(holon_role role, holon_config cfg = holon_config{});

  // Non-copyable and non-movable: the clock_driver observers capture `this` and
  // _driver holds a reference to _clock, so a moved holon would dangle both. This
  // is already implicit (world_clock's mutex deletes the defaults), but the
  // explicit deletes pin the invariant against a future refactor that makes the
  // clock movable.
  holon(const holon &) = delete;
  holon &operator=(const holon &) = delete;
  holon(holon &&) = delete;
  holon &operator=(holon &&) = delete;

  holon_role role() const noexcept { return _role; }
  cvc::world_clock &clock() noexcept { return _clock; }
  const cvc::world_clock &clock() const noexcept { return _clock; }
  cvc::clock_driver &driver() noexcept { return _driver; }
  std::uint64_t tick() const { return _clock.tick(); }

  // Register a participant. `local` marks the CLIENT's own avatar (predicted); on a
  // server every participant is authoritative, and on a client a non-local
  // participant is an interpolated peer.
  //
  // POSE_KNOWN = false (CLIENT): `start` is a placeholder, not a pose -- the host saw the
  // participant in the roster before its first authoritative sample. It is a member like any
  // other, but pose_known() stays false until apply_authoritative brings that sample, so a
  // renderer can leave it undrawn. With physics a non-local one has no kinematic body until then
  // (one held at the placeholder would be an invisible obstacle, at the origin, inside the room's
  // reference object); the first sample creates it there. The local avatar keeps its body (its
  // prediction steps it). A server ignores the flag: it is the authority, so every pose it holds
  // is known.
  void add_participant(const std::string &pid, bool local = false,
                       const cvc::physics::body_state &start = cvc::physics::body_state{},
                       bool pose_known = true);
  void remove_participant(const std::string &pid);
  bool has_participant(const std::string &pid) const;
  // Every registered pid, in pid order.
  std::vector<std::string> participants() const;

  // Whether `pid`'s pose is real: it was added with pose_known, or a sample has arrived since.
  // False for an unknown pid.
  bool pose_known(const std::string &pid) const;
  // Whether `pid` has a rigid body in the physics world. False without physics, for an unknown
  // pid, and for a client peer whose pose is not known yet.
  bool has_body(const std::string &pid) const;

  // Set a participant's latest intent. Server: the validated input it integrates
  // into the authoritative pose, from the next quantum on (any queued stamped
  // intents are dropped). Client: only the LOCAL avatar's own input (a non-local
  // pid is ignored -- a client cannot drive a peer). Either way the intent passes
  // sanitize_intent first.
  void set_intent(const std::string &pid, const avatar_intent &in);

  // The server's VETO over a proposed intent (§5h propose -> veto -> override): a
  // non-finite move is no move, a non-finite yaw faces +X, the vertical is dropped
  // unless allow_vertical, the horizontal speed is capped at max_speed and the yaw
  // is wrapped to [-pi, pi]. An honest client applies the same rule to its own
  // prediction, so it agrees with what the server will integrate (no correction);
  // a client that asks for more is overruled on reconcile.
  avatar_intent sanitize_intent(const avatar_intent &in) const;
  // SERVER: how many intents the veto has changed.
  std::uint64_t vetoes() const noexcept { return _vetoes; }

  // SERVER OVERRIDES (§5h): the authority acts on an avatar at will, and the result
  // replicates like any authoritative pose (the owning client reconciles to it).
  // Teleport `pid` to `feet` (pose + velocities). False for an unknown pid or on a
  // client holon.
  bool override_state(const std::string &pid, const cvc::physics::body_state &feet);
  // Re-mass `pid` (kg, finite and > 0). Physics only: false without a world, for an
  // unknown pid, a bad mass, or on a client holon.
  bool set_mass(const std::string &pid, double kg);

  // SERVER: a participant's intent STAMPED with the tick the client applied it from
  // in its prediction. It takes effect at that tick -- so the authoritative pose
  // matches the prediction exactly when it arrives in time (the client runs ahead by
  // its lead, set_lead_ticks) -- or at once when it arrives late (that tick is
  // already integrated) or stamped too far ahead (max_intent_lead_ticks). Re-queueing
  // the stamp already queued last is a no-op, so a host may re-read the tree every
  // frame. A client holon ignores this.
  void queue_intent(const std::string &pid, const avatar_intent &in, std::uint64_t tick);

  // Whether avatars are rigid bodies in a Jolt world (holon_config::physics, and a build that
  // has the backend).
  bool physics() const noexcept { return _world != nullptr; }

  // With physics: the room's static objects (the ground, the cube, ...), from the room scene at
  // `revision`. Avatars stand still until the first set_statics (there is nothing to stand on).
  void set_statics(const std::vector<avatar_static> &statics, std::uint64_t revision);
  std::uint64_t statics_revision() const noexcept { return _statics_rev; }

  // CLIENT: run the replica's clock `ticks` AHEAD of the server's (default 0: in step
  // with the samples). With a lead, an intent the client applies at its tick T reaches
  // the server before the server integrates T, so the server's pose agrees with the
  // prediction and reconcile has nothing to correct. The host sets it from the measured
  // round trip plus a margin (one way for the sample, one way for the intent).
  void set_lead_ticks(std::uint64_t ticks) { _lead = ticks; }
  std::uint64_t lead_ticks() const { return _lead; }

  // CLIENT ONLY: an authoritative state for `pid`, valid as of `server_tick`,
  // arrived via replication. The local avatar reconciles to it (snap + replay on
  // divergence); a peer takes it as its newest interpolation sample. No-op on a
  // server holon or an unknown pid.
  //
  // A sample at or below the newest one already seen for `pid` is ignored outright
  // (a re-read of an unchanged pose is not news, and must not move the clock). The
  // first one makes the pose known (pose_known), and gives a peer registered without
  // one its kinematic body, at the sample.
  //
  // Reconcile compares ticks, so the replica's clock must count the server's ticks.
  // The two clocks start independently (a client joins a server that has run for
  // hours), so a sample more than max_skew_ticks from the replica's tick -- the first
  // one after joining, or one after a backgrounded browser tab stalled the frame
  // loop -- re-aligns the replica: its clock seeks to server_tick (+ the lead) and the
  // local avatar's unacknowledged input log is dropped (its ticks no longer mean
  // anything; its state is kept). With a lead the target is server_tick + lead and the
  // tolerance the lead slack (holon_config). Between re-alignments both clocks run
  // from wall time at the same rate.
  void apply_authoritative(const std::string &pid, const cvc::physics::body_state &auth,
                           std::uint64_t server_tick);

  // CLIENT: how many times apply_authoritative has re-aligned the replica's clock (each seek also
  // drops the local avatar's unacknowledged inputs: a visible jump). The first sample after
  // joining is one; after that the count should stay flat, and a rising one means samples keep
  // arriving with ticks far from the replica's (a stalled frame loop, or a sender stamping stale
  // ticks).
  std::uint64_t clock_realigns() const noexcept { return _realigns; }

  // Advance the holon one frame. SERVER: drives its clock from wall time (wall_dt<0)
  // or an injected dt, then integrates every authoritative avatar over the quanta
  // that fell due. CLIENT: drives its clock to predict its own avatar over the due
  // quanta; peers are sample-driven (apply_authoritative) and only re-interpolated.
  void advance(double wall_dt = -1.0);

  // A participant's authoritative body_state: the server's integrated state, the
  // client-local avatar's current prediction, or a peer's newest sample. Throws
  // std::out_of_range for an unknown pid.
  cvc::physics::body_state state(const std::string &pid) const;

  // A participant's DISPLAY pose this frame: predicted for the local avatar, the
  // integrated pose on a server, and for a peer an interpolation between its last
  // two samples (never extrapolated). Samples arrive far less often than quanta (the
  // relay publishes ~5 Hz against a 120 Hz clock), so a peer is shown ONE SAMPLE
  // INTERVAL behind: the newest sample's arrival starts a walk from `prev` to
  // `newest` that takes as long as the server took between the two (their
  // server_tick gap), measured on this holon's own clock (tick + alpha).
  cvc::physics::pose display_pose(const std::string &pid) const;

  // CLIENT: the position jumps reconcile has applied to the local avatar `pid`
  // (snap + replay on divergence) since the last call, summed, then cleared. A
  // renderer eases this out so a correction reads as a glide rather than a jump; the
  // holon itself stays exact. Zero for a peer, on a server, or for an unknown pid.
  cvc::physics::vec3 take_correction(const std::string &pid);

private:
  struct participant {
    bool local = false;
    // CLIENT: its pose is real (added with one, or a sample has arrived); see add_participant.
    bool pose_known = true;
    avatar_sim sim;             // server authoritative integrator
    avatar_intent intent;       // latest intent (server input / client-local input)
    // SERVER: stamped intents waiting for their tick (queue_intent), in tick order.
    std::deque<std::pair<std::uint64_t, avatar_intent>> queued;
    bool have_stamp = false;      // a stamped intent has been queued (last_stamp is meaningful)
    std::uint64_t last_stamp = 0; // the stamp queued last (re-queueing it is a no-op)
    // CLIENT: the newest sample tick seen for this participant. A host re-reads an unchanged pose
    // every frame (and a server publishing only changes leaves a still avatar's tick old for up to
    // a heartbeat), so a sample already seen must not count as a clock reading.
    bool have_seen = false;
    std::uint64_t last_seen = 0;
    predicted_avatar predicted; // client-local prediction + reconcile
    cvc::physics::vec3 correction{0.0, 0.0, 0.0}; // reconcile jumps not yet taken
    // peer interpolation: the last two authoritative samples + a monotonic guard
    // (mirroring predicted_avatar's stale/duplicate-ack rejection) so a reordered
    // or duplicate peer sample never rewinds or freezes the interpolation window.
    bool have_sample = false;
    std::uint64_t last_sample_tick = 0;
    cvc::physics::body_state prev, newest;
    std::uint64_t newest_arrival_tick = 0; // this clock's tick when `newest` arrived
    std::uint64_t sample_span = 0;         // server ticks from prev to newest (0 = one sample)
    // physics: where it joined (a SERVER respawn point), and the CLIENT-local avatar's
    // prediction log -- predicted_avatar's, over the Jolt world instead of avatar_sim.
    cvc::physics::body_state spawn;
    struct logged {
      std::uint64_t tick = 0;
      avatar_intent in;
      cvc::physics::body_state after; // its state after the step at `tick`
    };
    std::deque<logged> log;
    bool acked = false;
    std::uint64_t last_acked = 0;
  };

  // physics: one quantum of the world (the drive, the peers' kinematic poses, the step).
  void world_step_client(const std::string &self, const avatar_intent &in);
  bool world_reconcile(const std::string &pid, participant &p, const cvc::physics::body_state &auth,
                       std::uint64_t server_tick);

  void on_step(const cvc::clock_driver::clock_frame &f);
  void on_present(const cvc::clock_driver::clock_frame &f);

  holon_role _role;
  holon_config _cfg;
  cvc::world_clock _clock; // declared before _driver: _driver references it
  cvc::clock_driver _driver;
  std::map<std::string, participant> _participants;
  double _alpha = 0.0;                  // the last frame's interpolation alpha, for display_pose
  std::uint64_t _lead = 0;              // CLIENT: ticks ahead of the server (set_lead_ticks)
  std::unique_ptr<avatar_world> _world; // physics (holon_config::physics + a Jolt build)
  std::uint64_t _vetoes = 0;            // SERVER: intents sanitize_intent changed
  std::uint64_t _realigns = 0;          // CLIENT: clock seeks in apply_authoritative
  std::uint64_t _statics_rev = 0;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_HOLON_H__
