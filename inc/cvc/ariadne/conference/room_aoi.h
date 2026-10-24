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

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_AOI_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_AOI_H__

#include <cstddef>
#include <cstdint>
#include <cvc/state/state_peer_registry.h> // state_delivery_filter
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
class app;

namespace ariadne {
namespace conference {

// ----------------
// Spatial area of interest (AOI) in the shared room
// ----------------
// The relay decides, for every ordered pair (receiver r, subject q) of present participants, how
// much of q the client r needs, from their distance in the room's XY plane:
//
//   inner   everything: q's live pose stream, full video, audio
//   middle  q's pose as relay PUSHES at a reduced cadence, camera thumbnails, audio
//   outer   q's pose as pushes, no media at all
//
// Tiers have hysteresis (fast attack, slow release): an upgrade happens at once when the distance
// falls inside the better tier's enter radius (outer can go straight to inner), while a downgrade
// moves ONE tier and only after the distance has stayed beyond the current tier's exit radius for
// downgrade_dwell_s. A pair pacing along a boundary therefore flips at most about once per dwell.
//
// STATE is filtered per receiver by a DENY-only predicate (aoi_state_filter, installed on the
// transports' peer registries), and it fails open: an unknown participant, an unknown leaf family,
// a receiver the relay has not placed yet and any path outside the participants subtree all
// deliver, so a new leaf family is never dropped silently. Under
// conf.rooms.<room>.participants.<q>.<family>..., a receiver r is denied only:
//   - another participant's intent (q != r): only the relay and the intent's owner read it
//   - q's whole pose subtree, server_tick included, unless tier(r, q) == inner (non-inner pairs
//     get the pose as pushes instead, so a receiver never sees a fresh tick over stale leaves)
//   - a departed q's pose, intent, net and voice (its soft-left subtree lingers as a ghost)
// The receiver's own subtree always delivers, as do the presence, profile, net, streams and voice
// of present participants (the roster stays room-wide), the bare participants.<q> node, and the
// room's scene, server and aoi subtrees.
//
// MEDIA is decided per frame (aoi_snapshot::media):
//   inner   delivered; with video_full_max > 0, a camera outside the receiver's nearest
//           video_full_max subjects (swapped only past video_rank_margin_m) is a thumbnail
//   middle  a camera in mjpeg is a thumbnail (MJPEG is intra-only, so any subset of its frames
//           decodes; the relay's gate sends thumb_fps of them); any other video codec is
//           delivered (never decimated) and so is the mic
//   outer   dropped
// An unknown receiver or publisher is delivered.
//
// POSE PUSHES (room_aoi::take_pose_pushes) carry a non-inner subject's whole pose subtree to a
// receiver: immediately when a pair enters inner and for a new pair that starts middle or outer;
// every aoi_pose_push_interval_ticks while the subject moves; and every aoi_pose_refresh_ticks as a
// refresh while it stands still. enabled = false delivers everything and pushes nothing.
//
// THREADING: room_aoi runs on the relay's main loop only. An aoi_snapshot is immutable once
// published and an aoi_state_filter only reads one, so both are safe on any thread at once;
// deliver_path, media and tier are noexcept and allocate nothing.

// Not near/far: <windows.h> (minwindef.h) defines both as macros.
enum class aoi_tier : std::uint8_t { inner, middle, outer };

struct aoi_params {
  bool enabled = true; // false: deliver everything, push nothing
  std::string preset = "conference";
  // Tier radii (metres, XY distance). Each enter radius is at most its exit radius, and
  // inner_exit_m is at most middle_enter_m. The defaults are the conference preset.
  double inner_enter_m = 30.0;
  double inner_exit_m = 32.0;
  double middle_enter_m = 40.0;
  double middle_exit_m = 44.0;
  double downgrade_dwell_s = 1.0;
  double pose_push_hz = 5.0;   // pushes to non-inner receivers while the subject moves; 0 = off
  double pose_refresh_s = 5.0; // pushes to non-inner receivers while it stands still; 0 = off
  double thumb_fps = 2.0;      // the relay's thumbnail rate for a decimated camera
  double video_rank_margin_m = 1.0;
  double audio_fade_m = 2.0;  // the client's gain reaches 0 at middle_enter_m over this distance
  int video_full_max = 0;     // 0 = every inner camera at full rate
  bool audio_rolloff = false; // the client's gain falls off as inner_enter_m / d past inner_enter_m
};

// The 20 m standup room: every pair on the [-10,10]^2 plane (diagonal 28.3 m) is inner at first
// sight and stays inner, so pose, video and audio are exactly what they were without AOI.
// 30/32 m inner, 40/44 m middle, no rolloff.
aoi_params conference_aoi_params();

// A hall to spread out in: 10/12 m inner (the 4 m spawn ring keeps every joiner inner), 16/18 m
// middle, and a 1/d audio rolloff that fades to silence at 16 m.
aoi_params hall_aoi_params();

// The tier at first sight: the best tier whose enter radius holds D_M.
aoi_tier initial_tier(double d_m, const aoi_params &p) noexcept;

// One hysteresis step from CUR at distance D_M and time NOW_S (seconds, >= 0). BEYOND_SINCE_S is
// the pair's dwell timer, owned by the caller: negative while the distance is within the current
// tier's exit radius (start every pair at -1), otherwise when it went beyond.
aoi_tier step_tier(aoi_tier cur, double d_m, double now_s, double &beyond_since_s,
                   const aoi_params &p) noexcept;

// The client's playback gain for a peer D_M away: 1 when !p.enabled; with audio_rolloff,
// (std::min)(1, inner_enter_m / d); either way times a linear fade from 1 at
// middle_enter_m - audio_fade_m to 0 at middle_enter_m. The fade ends at the UPGRADE threshold, so
// both the relay's cut (outer) and its resume (back to middle) happen in silence.
float aoi_audio_gain(double d_m, const aoi_params &p) noexcept;

// The pose push cadence in holon ticks at TICK_HZ (the holon's quantum, 120 Hz by default):
// round(tick_hz / pose_push_hz) and round(pose_refresh_s * tick_hz). 0 = that push is off.
std::uint64_t aoi_pose_push_interval_ticks(const aoi_params &p, double tick_hz = 120.0) noexcept;
std::uint64_t aoi_pose_refresh_ticks(const aoi_params &p, double tick_hz = 120.0) noexcept;

enum class aoi_media : std::uint8_t { deliver, thumbnail, drop };

// One room's AOI decisions at one moment. Built by room_aoi and never changed after it is
// published, so any thread may query it while the relay builds the next one.
class aoi_snapshot {
public:
  // An empty snapshot knows no participant, so it delivers everything.
  aoi_snapshot() = default;

  // Whether RECEIVER gets the state node at PATH (the rules above).
  bool deliver_path(const std::string &receiver, const std::string &path) const noexcept;

  // What RECEIVER gets of PUBLISHER's STREAM_ID ("camera" | "mic") frames in CODEC.
  aoi_media media(const std::string &receiver, const std::string &publisher,
                  const std::string &stream_id, const std::string &codec) const noexcept;

  // The pair's tier; inner for oneself and for any pair this snapshot does not know.
  aoi_tier tier(const std::string &receiver, const std::string &subject) const noexcept;

  // 0 for an empty snapshot, then +1 per snapshot a room_aoi publishes.
  std::uint64_t generation() const noexcept;

  // False when built from aoi_params::enabled == false (everything delivers).
  bool enabled() const noexcept;

  // The present participants, in pid order.
  const std::vector<std::string> &participants() const noexcept;

private:
  friend class room_aoi;

  static constexpr std::size_t npos = static_cast<std::size_t>(-1);

  // The index of PID in _pids, or npos.
  std::size_t index_of(std::string_view pid) const noexcept;
  bool is_departed(std::string_view pid) const noexcept;

  bool _enabled = true;
  bool _video_capped = false; // video_full_max > 0: _ranked applies to inner cameras
  std::uint64_t _generation = 0;
  std::string _prefix;            // conf.rooms.<room>.participants. (with the trailing separator)
  std::vector<std::string> _pids; // present, sorted
  std::vector<aoi_tier> _tiers;   // _pids.size()^2, row = receiver, column = subject
  std::vector<std::uint8_t> _ranked;  // same layout: 1 = in the receiver's full-video set
  std::vector<std::string> _departed; // sorted
};

// The registry filter for one receiver's connection: a snapshot's deliver_path bound to RECEIVER.
// A null snapshot delivers everything.
class aoi_state_filter final : public cvc::state_delivery_filter {
public:
  aoi_state_filter(std::shared_ptr<const aoi_snapshot> s, std::string receiver);
  bool deliver(const std::string &path) const noexcept override;

private:
  std::shared_ptr<const aoi_snapshot> _snap;
  std::string _receiver;
};

// The subjects whose whole pose subtree is due to RECEIVER now.
struct aoi_push {
  std::string receiver;
  std::vector<std::string> subjects; // in pid order
};

// The relay's AOI state for one room: per-pair tiers with their dwell timers, each receiver's
// full-video set, the pose push schedule, and the latest published snapshot. Main loop only.
class room_aoi {
public:
  struct position {
    std::string pid;
    double x = 0.0;
    double y = 0.0;
  };

  room_aoi(std::string room, aoi_params p);

  // Take the present participants and their positions (a pid listed twice counts once, at its
  // first position): new pairs start at initial_tier, existing ones step_tier at NOW_S, and anyone
  // no longer present becomes departed (a rejoin under the same pid clears that). O(N^2). Returns
  // true when the decisions changed, in which case snapshot() is a new one; otherwise snapshot()
  // returns the same object as before.
  bool update(const std::vector<position> &present, std::uint64_t now_tick, double now_s);

  // The pose pushes due at NOW_TICK (the relay holon's tick), one entry per receiver with any due,
  // in pid order. POSE_VERSION[q] is the tick at which q last moved (a missing q counts as 0). A
  // pair is due at once after update() made it enter inner, or created it middle or outer; a
  // non-inner pair is also due when q's version changed since that pair's last push and the push
  // interval has passed, or when the refresh interval has passed. Inner pairs are otherwise never
  // due (the live stream carries them). Records the pushes as sent; nothing when !enabled.
  std::vector<aoi_push> take_pose_pushes(std::uint64_t now_tick,
                                         const std::map<std::string, std::uint64_t> &pose_version);

  std::shared_ptr<const aoi_snapshot> snapshot() const;

  // Everyone who was present at some update and is not now, in pid order.
  const std::set<std::string> &departed() const;

  const aoi_params &params() const noexcept;
  const std::string &room() const noexcept;

private:
  struct pair_state {
    aoi_tier tier = aoi_tier::inner;
    double beyond_since_s = -1.0;
    double d_m = 0.0;
    bool push_due = false; // a full pose push is owed at the next take_pose_pushes
    std::uint64_t last_push_tick = 0;
    std::uint64_t last_version = 0;
  };
  struct member {
    double x = 0.0;
    double y = 0.0;
    std::map<std::string, pair_state> subjects; // every other present participant
    std::set<std::string> ranked;               // the full-video set (video_full_max > 0)
  };

  bool rerank(member &r) const;
  void publish();

  std::string _room;
  aoi_params _p;
  std::string _prefix;
  std::uint64_t _generation = 0;
  std::map<std::string, member> _members;
  std::set<std::string> _departed;
  std::shared_ptr<const aoi_snapshot> _snap;
};

// conf.rooms.<room-id>.aoi
std::string room_aoi_path(const std::string &room_id);

// Replicate P under conf.rooms.<room>.aoi.<field> (one leaf per aoi_params field, named after it)
// so clients can derive their audio gain. A leaf that already holds its value is not rewritten.
void write_aoi_params(cvc::app &app, const std::string &room, const aoi_params &p);

// Read them back. Returns false (OUT untouched) if they were never written; a missing leaf keeps
// OUT's value.
bool read_aoi_params(cvc::app &app, const std::string &room, aoi_params &out);

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_AOI_H__
