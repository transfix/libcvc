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

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_AOI_RELAY_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_AOI_RELAY_H__

#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/conference/room_aoi.h>
#include <cvc/physics/body.h> // body_state
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace cvc {
class app;
class media_endpoint;
class state_cluster_shard;
class state_transport;

namespace ariadne {
class holon;

namespace conference {

// ----------------
// The relay's area-of-interest driver
// ----------------
// Runs a room_aoi against the relay's SERVER holon and applies its decisions to the relay's
// transports, once per room step (step(), or relay_room_step below):
//
//   STATE   every present participant is a peer in every added transport's registry, subscribed
//           to "conf", with an aoi_state_filter bound to the latest snapshot; a departed one is
//           unregistered again (unknown peers deliver everything). Transports consult the registry
//           on every room-state send -- publish, relay forward, snapshot reply, broadcast -- so
//           the filter applies to the relay's own writes and to the clients' alike. A peer the
//           registry knew before this driver registered it keeps its entry; only the filter is
//           installed and later cleared.
//   POSES   the pushes room_aoi says are due go to each receiver as ONE targeted snapshot frame
//           (state_transport::push_snapshot_to, on every transport: only the one holding the
//           receiver's connection writes) of the subjects' pose subtrees, intermediate nodes left
//           out. Each subject's server_tick is re-stamped with the holon's CURRENT tick and is
//           its last entry: the tree's tick of a subject standing still can be a heartbeat old,
//           and a client re-aligns its whole clock on a stale one. The other leaves equal the
//           current state to within write_pose's 1e-4, as server_pump has just written them.
//   MEDIA   a gate on every added media endpoint asks the snapshot per frame and receiver
//           (aoi_snapshot::media): deliver, drop, or a thumbnail -- the first frame once the
//           pair's decision becomes a thumbnail, then thumb_fps of them.
//
// A snapshot is rebuilt only when room_aoi reports a change, and the registries and gate are
// updated only then. aoi_params::enabled = false makes the driver a pass-through: nothing is
// registered, every frame is delivered and nothing is pushed.
//
// LIFETIME: the shard, the transports and the media endpoints must outlive the driver. Its
// destructor clears the gates it installed and the registrations and filters it made. The gate
// closure owns the state it reads (never the driver), so a reader thread still inside a copy of
// it when the driver goes away finishes safely, and from then on that copy delivers everything.
//
// THREADING: construction, add_*, step, take_stats and destruction on the relay's main loop (the
// thread that writes the room tree and calls publish()). The gate runs on publisher reader
// threads: it reads an immutable snapshot through a mutex-guarded pointer, keeps its thumbnail
// timers under a mutex of their own, and holds no lock across a socket write.
class room_aoi_relay {
public:
  struct stats {
    std::size_t inner = 0; // present pairs (unordered) in each tier, as of now
    std::size_t middle = 0;
    std::size_t outer = 0;
    std::uint64_t pushes = 0;         // push frames written (one per receiving connection)
    std::uint64_t push_entries = 0;   // snapshot entries in them
    std::uint64_t media_full = 0;     // frames the gate passed to a receiver whole
    std::uint64_t media_thumb = 0;    // thumbnail frames it passed
    std::uint64_t media_dropped = 0;  // frames it withheld: outer, or decimated to the thumb rate
    std::uint64_t state_filtered = 0; // registry deliveries filtered for registered participants
  };

  room_aoi_relay(cvc::app &app, cvc::state_cluster_shard &shard, std::string room, aoi_params p);
  ~room_aoi_relay();

  room_aoi_relay(const room_aoi_relay &) = delete;
  room_aoi_relay &operator=(const room_aoi_relay &) = delete;

  // A transport whose registry gets the participants' filters and which carries the pushes. One
  // added after a snapshot exists gets that snapshot's filters at once. Null or a repeat: ignored.
  void add_transport(cvc::state_transport *tp);

  // An endpoint whose fan-out the media gate decides (replacing any gate it had). Null or a
  // repeat: ignored.
  void add_media_endpoint(cvc::media_endpoint *ep);

  // One room step, after server_pump has written this step's poses and BEFORE the step's drain
  // (relay_room_step keeps that order): read every participant's position and motion from SERVER
  // (no tree walk), update the AOI, re-install filters and the gate's snapshot if it changed,
  // then write the pushes that are due. NOW_S: seconds on a steady clock, >= 0.
  void step(const cvc::ariadne::holon &server, double now_s);

  // Counters since the last call (the tier counts are a reading of the current snapshot).
  stats take_stats();

  std::shared_ptr<const aoi_snapshot> snapshot() const;
  const aoi_params &params() const noexcept;

private:
  struct gate_state;
  struct plane {
    cvc::state_transport *tp = nullptr;
    std::set<std::string> added;    // pids this driver registered (removed again on departure)
    std::set<std::string> filtered; // pids carrying this driver's filter
    std::map<std::string, std::uint64_t> filtered_seen; // deliveries_filtered at the last stats
  };

  using filter_map = std::map<std::string, std::shared_ptr<const cvc::state_delivery_filter>>;

  // One filter per present participant, bound to SNAP; none when SNAP is disabled.
  static filter_map filters_of(const std::shared_ptr<const aoi_snapshot> &snap);
  void apply(const std::shared_ptr<const aoi_snapshot> &snap);
  void install(plane &pl, const filter_map &filters);
  void release(plane &pl, const std::string &pid);
  void push_poses(std::uint64_t tick);

  cvc::app &_app;
  cvc::state_cluster_shard &_shard;
  room_aoi _aoi;
  std::vector<plane> _planes;
  std::vector<cvc::media_endpoint *> _endpoints;
  std::map<std::string, cvc::physics::body_state> _pose_ref; // the pose at its last version
  std::map<std::string, std::uint64_t> _pose_version;        // tick of the last move > 1e-4
  std::uint64_t _pushes = 0;
  std::uint64_t _push_entries = 0;
  std::shared_ptr<gate_state> _gate;
};

// The relay's per-step fan-out of its OWN room-tree writes (poses, roster retractions, stream
// markers, net stats, the scene): AOI->step first, so every push precedes this step's live writes
// on each connection; then ONE drain_local of SHARD, each mutation published to every plane in
// order (publish writes only to that transport's connections and never crosses the bridge, so
// each client gets it once, and two transports sharing the shard's one publish cursor both send
// it); then, with TRIM_JOURNAL, the published journal is erased (shard.trim_published) -- for a
// relay whose late joiners bootstrap from the admit snapshot, i.e. with backfill on connect off.
// With AOI null and TRIM_JOURNAL false it is exactly that drain and publish, nothing else (SERVER
// and NOW_S are then unused). Returns the number of mutations drained. Main loop only.
std::size_t relay_room_step(cvc::state_cluster_shard &shard,
                            const std::vector<cvc::state_transport *> &planes, room_aoi_relay *aoi,
                            const cvc::ariadne::holon &server, double now_s, bool trim_journal);

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_AOI_RELAY_H__
