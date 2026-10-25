/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_RUNGS_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_RUNGS_H__

#include <cstdint>
#include <cvc/media/media_rungs.h>
#include <cvc/state/state_link_stats.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {
namespace conference {

// ----------------
// room_rung_controller: each viewer's video rung, adapted to its link (roadmap §5j)
// ----------------
// A relay calls step() about once a second with its link reports (state_transport::link_reports).
// For each viewer (an accepted link that holds its node id) it reads how the link coped since the
// last call:
//   - queueing: the round trip above its minimum, which is the time the viewer's frames wait in
//     the send queue (the median round trip and the latest one, whichever is higher);
//   - drops: the share of media frames the transport dropped for it because the link could not
//     take them (a backed-up send buffer, a stalled write);
//   - throughput: what the relay sent it, against rung_params::viewer_kbps_max when one is set.
// A viewer goes DOWN one rung after `down_after` bad passes in a row, and UP one after
// `up_after_s` of good ones: fast down, slow up. A step up that turns bad again before its wait
// was over doubles the next wait (to max_up_after_s); one that holds for twice the wait resets it.
// Rungs stay between the server's best (max_rung) and its floor (floor_rung).
struct rung_params {
  bool adapt = true;                         // false: every viewer at max_rung
  video_rung max_rung = video_rung::full;    // the best any viewer gets
  video_rung floor_rung = video_rung::thumb; // the lowest adaptation goes (off: no video)
  double viewer_kbps_max = 0.0;              // a viewer's budget, all it is sent; 0 = none
  double down_queue_ms = 150.0;              // bad: queueing above this
  double down_drop = 0.05;                   // bad: more than this share of frames dropped
  int down_after = 2;                        // bad passes in a row before a step down
  double up_queue_ms = 50.0;                 // good: queueing below this, and nothing dropped
  double up_after_s = 10.0;                  // good this long before a step up
  double max_up_after_s = 60.0;              // the longest the wait grows to
};

// What step() saw of one viewer this pass (for the relay's log and tests).
struct rung_sample {
  video_rung rung = video_rung::full;
  double queue_ms = 0.0;
  double drop = 0.0;     // share of media frames dropped this pass
  double out_kbps = 0.0; // what the relay sent it
  bool changed = false;  // the rung moved this pass
};

class room_rung_controller {
public:
  explicit room_rung_controller(rung_params p);
  room_rung_controller();
  ~room_rung_controller();
  room_rung_controller(const room_rung_controller &) = delete;
  room_rung_controller &operator=(const room_rung_controller &) = delete;

  const rung_params &params() const noexcept { return _p; }

  // One pass over LINKS at NOW_S (seconds, monotonic): every viewer's rung after it. A viewer
  // missing from LINKS is forgotten.
  std::map<std::string, rung_sample> step(const std::vector<cvc::state_link_report> &links,
                                          double now_s);

private:
  struct table; // per-viewer state, defined in the .cpp (keeps it out of cvc.dll's exports)
  rung_params _p;
  std::unique_ptr<table> _t;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_RUNGS_H__
