/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/ariadne/conference/room_rungs.h>
#include <memory>
#include <unordered_map>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

int index(video_rung r) { return static_cast<int>(r); }
video_rung at(int i) { return static_cast<video_rung>(std::clamp(i, 0, kVideoRungs - 1)); }

} // namespace

struct room_rung_controller::table {
  struct viewer {
    video_rung rung = video_rung::full;
    int bad = 0;              // bad passes in a row
    double good_since = -1.0; // when the good run began (-1: not good now)
    double up_wait = 0.0;     // how long a good run must last before a step up
    double last_up = -1.0;    // when it last stepped up
    // The counters at the last pass, for this pass's share and rate.
    bool seen = false;
    std::uint64_t sent = 0, dropped = 0, bytes_out = 0;
    double at_s = 0.0;
  };
  std::unordered_map<std::string, viewer> viewers;
};

room_rung_controller::room_rung_controller(rung_params p) : _p(p), _t(std::make_unique<table>()) {
  if (index(_p.floor_rung) < index(_p.max_rung))
    _p.floor_rung = _p.max_rung; // a floor above the best is the best
}
room_rung_controller::room_rung_controller() : room_rung_controller(rung_params()) {}
room_rung_controller::~room_rung_controller() = default;

std::map<std::string, rung_sample>
room_rung_controller::step(const std::vector<cvc::state_link_report> &links, double now_s) {
  std::map<std::string, rung_sample> out;
  std::unordered_map<std::string, table::viewer> next;
  for (const cvc::state_link_report &r : links) {
    if (!r.accepted || !r.holds_id || r.remote_node_id.empty() || out.count(r.remote_node_id))
      continue;
    const auto it = _t->viewers.find(r.remote_node_id);
    table::viewer v;
    if (it != _t->viewers.end()) {
      v = it->second;
    } else {
      v.rung = _p.max_rung;
      v.up_wait = _p.up_after_s;
    }
    rung_sample s;
    const video_rung before = v.rung;

    // This pass's share of frames dropped and rate sent, from the counters' change since the last.
    const std::uint64_t ds = v.seen ? r.stats.media_sent - v.sent : 0;
    const std::uint64_t dd = v.seen ? r.stats.media_dropped - v.dropped : 0;
    if (ds + dd >= 10) // too few frames say nothing
      s.drop = static_cast<double>(dd) / static_cast<double>(ds + dd);
    if (v.seen && now_s > v.at_s)
      s.out_kbps =
          static_cast<double>(r.stats.bytes_out - v.bytes_out) * 8.0 / 1000.0 / (now_s - v.at_s);
    v.seen = true;
    v.sent = r.stats.media_sent;
    v.dropped = r.stats.media_dropped;
    v.bytes_out = r.stats.bytes_out;
    v.at_s = now_s;
    // Queueing: the higher of the median and the latest round trip, above the lowest seen.
    if (r.stats.rtt_min_ms >= 0.0)
      s.queue_ms =
          std::max(0.0, std::max(r.stats.rtt_ms, r.stats.rtt_last_ms) - r.stats.rtt_min_ms);

    if (!_p.adapt) {
      v.rung = _p.max_rung;
    } else {
      const bool over = _p.viewer_kbps_max > 0.0 && s.out_kbps > _p.viewer_kbps_max;
      const bool bad = s.queue_ms > _p.down_queue_ms || s.drop > _p.down_drop || over;
      const bool good = s.queue_ms < _p.up_queue_ms && dd == 0 &&
                        (_p.viewer_kbps_max <= 0.0 || s.out_kbps < 0.8 * _p.viewer_kbps_max);
      if (bad) {
        v.good_since = -1.0;
        if (++v.bad >= _p.down_after && index(v.rung) < index(_p.floor_rung)) {
          // A step up that failed before its wait was over: wait longer next time.
          if (v.last_up >= 0.0 && now_s - v.last_up < v.up_wait)
            v.up_wait = std::min(2.0 * v.up_wait, _p.max_up_after_s);
          v.rung = at(index(v.rung) + 1);
          v.bad = 0;
        }
      } else {
        v.bad = 0;
        if (!good) {
          v.good_since = -1.0;
        } else if (v.good_since < 0.0) {
          v.good_since = now_s;
        } else if (now_s - v.good_since >= v.up_wait && index(v.rung) > index(_p.max_rung)) {
          v.rung = at(index(v.rung) - 1);
          v.last_up = now_s;
          v.good_since = now_s;
        }
        if (v.last_up >= 0.0 && now_s - v.last_up > 2.0 * v.up_wait)
          v.up_wait = _p.up_after_s; // the last step up held: back to the base wait
      }
      // Within the server's settings.
      v.rung = at(std::clamp(index(v.rung), index(_p.max_rung), index(_p.floor_rung)));
    }
    s.rung = v.rung;
    s.changed = v.rung != before;
    out[r.remote_node_id] = s;
    next[r.remote_node_id] = v;
  }
  _t->viewers.swap(next); // a viewer no longer linked is forgotten
  return out;
}

} // namespace conference
} // namespace ariadne
} // namespace cvc
