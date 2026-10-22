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
#include <cvc/ariadne/avatar_sim.h>
#include <cvc/core/clock_driver.h>
#include <cvc/core/namespace.h>
#include <cvc/core/world_clock.h>
#include <cvc/physics/body.h>
#include <cvc/physics/pose_tracker.h>
#include <map>
#include <string>

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
  cvc::world_clock::config clock; // the holon's own quantum / scale / mode
  double reconcile_tol = 0.05;    // metres; the client's snap threshold (slice 2)
};

class holon {
public:
  explicit holon(holon_role role, holon_config cfg = holon_config{});

  holon_role role() const noexcept { return _role; }
  cvc::world_clock &clock() noexcept { return _clock; }
  const cvc::world_clock &clock() const noexcept { return _clock; }
  cvc::clock_driver &driver() noexcept { return _driver; }
  std::uint64_t tick() const { return _clock.tick(); }

  // Register a participant. `local` marks the CLIENT's own avatar (predicted); on a
  // server every participant is authoritative, and on a client a non-local
  // participant is an interpolated peer.
  void add_participant(const std::string &pid, bool local = false,
                       const cvc::physics::body_state &start = cvc::physics::body_state{});
  void remove_participant(const std::string &pid);
  bool has_participant(const std::string &pid) const;

  // Set a participant's latest intent. Server: the validated input it integrates
  // into the authoritative pose. Client: only the LOCAL avatar's own input (a
  // non-local pid is ignored -- a client cannot drive a peer).
  void set_intent(const std::string &pid, const avatar_intent &in);

  // CLIENT ONLY: an authoritative state for `pid`, valid as of `server_tick`,
  // arrived via replication. The local avatar reconciles to it (snap + replay on
  // divergence); a peer takes it as its newest interpolation sample. No-op on a
  // server holon or an unknown pid.
  void apply_authoritative(const std::string &pid, const cvc::physics::body_state &auth,
                           std::uint64_t server_tick);

  // Advance the holon one frame. SERVER: drives its clock from wall time (wall_dt<0)
  // or an injected dt, then integrates every authoritative avatar over the quanta
  // that fell due. CLIENT: drives its clock to predict its own avatar over the due
  // quanta; peers are sample-driven (apply_authoritative) and only re-interpolated.
  void advance(double wall_dt = -1.0);

  // A participant's authoritative body_state: the server's integrated state, the
  // client-local avatar's current prediction, or a peer's newest sample. Throws
  // std::out_of_range for an unknown pid.
  cvc::physics::body_state state(const std::string &pid) const;

  // A participant's DISPLAY pose at the current frame's interpolation alpha:
  // predicted for the local avatar, interpolated between the last two samples for a
  // peer (never extrapolated), the integrated pose on a server.
  cvc::physics::pose display_pose(const std::string &pid) const;

private:
  struct participant {
    bool local = false;
    avatar_sim sim;             // server authoritative integrator
    avatar_intent intent;       // latest intent (server input / client-local input)
    predicted_avatar predicted; // client-local prediction + reconcile
    bool have_newest = false, have_prev = false;
    cvc::physics::body_state prev, newest; // peer interpolation samples
  };

  void on_step(const cvc::clock_driver::clock_frame &f);
  void on_present(const cvc::clock_driver::clock_frame &f);

  holon_role _role;
  holon_config _cfg;
  cvc::world_clock _clock; // declared before _driver: _driver references it
  cvc::clock_driver _driver;
  std::map<std::string, participant> _participants;
  double _alpha = 0.0; // the last frame's interpolation alpha, for display_pose
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_HOLON_H__
