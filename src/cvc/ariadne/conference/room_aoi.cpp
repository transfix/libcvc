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

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/conference/room_aoi.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h> // cvc::state, SEPARATOR, findDescendant
#include <initializer_list>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

constexpr std::size_t kNpos = std::string_view::npos;

// A path BASE.<segs...>, joined with cvc::state::SEPARATOR.
std::string sub(const std::string &base, std::initializer_list<const char *> segs) {
  std::string s = base;
  for (const char *seg : segs) {
    s += cvc::state::SEPARATOR;
    s += seg;
  }
  return s;
}

// conf.rooms.<room>.participants. -- room_participant_path(room, q) is this followed by q.
std::string participants_prefix(const std::string &room) {
  return sub("conf", {"rooms", room.c_str(), "participants"}) + cvc::state::SEPARATOR;
}

// The segment of PATH that starts at POS and runs to the next separator or the end. POS moves past
// that separator, or to kNpos when the segment was the last one.
std::string_view next_segment(std::string_view path, std::size_t &pos) noexcept {
  if (pos == kNpos || pos > path.size())
    return {};
  const std::string_view sep(cvc::state::SEPARATOR);
  const std::size_t end = path.find(sep, pos);
  const std::string_view seg = path.substr(pos, end == kNpos ? kNpos : end - pos);
  pos = end == kNpos ? kNpos : end + sep.size();
  return seg;
}

double exit_radius(aoi_tier t, const aoi_params &p) noexcept {
  return t == aoi_tier::inner ? p.inner_exit_m : p.middle_exit_m;
}

// SECONDS of holon ticks at TICK_HZ, at least 1; 0 when SECONDS is not a positive finite span.
std::uint64_t to_ticks(double seconds, double tick_hz) noexcept {
  const double t = seconds * tick_hz;
  if (!(t > 0.0) || !std::isfinite(t))
    return 0;
  const long long n = std::llround((std::min)(t, 1e15));
  return (std::max)(std::uint64_t{1}, static_cast<std::uint64_t>(n));
}

// Round-trip-safe double text (17 significant digits exactly recover a binary64).
std::string fmt(double d) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", d);
  return std::string(buf);
}

// touch-before-value, so a just-created node's first write is not lost to the distributed-state
// adapter's attach race (the room tree's set_replicated idiom). A leaf that already holds VAL is
// not rewritten, so re-applying the same parameters (a relay restart) adds no replication traffic.
void set_replicated(cvc::app &app, const std::string &path, const std::string &val) {
  cvc::state *n = app.root().findDescendant(path);
  if (n && n->value() == val)
    return;
  cvc::state &leaf = n ? *n : app.root()(path);
  leaf.touch();
  leaf.value(val);
}

cvc::state *find(cvc::app &app, const std::string &path) { return app.root().findDescendant(path); }
std::string get_s(cvc::app &app, const std::string &path, const std::string &def) {
  cvc::state *n = find(app, path);
  return n ? n->value() : def;
}
double get_d(cvc::app &app, const std::string &path, double def) {
  cvc::state *n = find(app, path);
  return n && !n->value().empty() ? std::strtod(n->value().c_str(), nullptr) : def;
}
int get_i(cvc::app &app, const std::string &path, int def) {
  cvc::state *n = find(app, path);
  return n && !n->value().empty() ? static_cast<int>(std::strtol(n->value().c_str(), nullptr, 10))
                                  : def;
}
bool get_b(cvc::app &app, const std::string &path, bool def) {
  cvc::state *n = find(app, path);
  return n && !n->value().empty() ? n->value() == "1" : def;
}

} // namespace

// ---------------- presets and per-pair rules ----------------

aoi_params conference_aoi_params() {
  aoi_params p;
  p.preset = "conference";
  p.inner_enter_m = 30.0;
  p.inner_exit_m = 32.0;
  p.middle_enter_m = 40.0;
  p.middle_exit_m = 44.0;
  p.audio_rolloff = false;
  return p;
}

aoi_params hall_aoi_params() {
  aoi_params p;
  p.preset = "hall";
  p.inner_enter_m = 10.0;
  p.inner_exit_m = 12.0;
  p.middle_enter_m = 16.0;
  p.middle_exit_m = 18.0;
  p.audio_rolloff = true;
  return p;
}

aoi_tier initial_tier(double d_m, const aoi_params &p) noexcept {
  if (d_m <= p.inner_enter_m)
    return aoi_tier::inner;
  if (d_m <= p.middle_enter_m)
    return aoi_tier::middle;
  return aoi_tier::outer;
}

aoi_tier step_tier(aoi_tier cur, double d_m, double now_s, double &beyond_since_s,
                   const aoi_params &p) noexcept {
  // Upgrades are immediate, as far as the enter radii allow (outer straight to inner).
  const aoi_tier best = initial_tier(d_m, p);
  if (best < cur) {
    beyond_since_s = -1.0;
    return best;
  }
  if (cur == aoi_tier::outer || !(d_m > exit_radius(cur, p))) {
    beyond_since_s = -1.0; // back within the exit radius: the dwell starts over
    return cur;
  }
  if (beyond_since_s < 0.0)
    beyond_since_s = now_s;
  if (now_s - beyond_since_s < p.downgrade_dwell_s)
    return cur;
  // One tier per dwell: the next tier's own dwell starts now if the pair is beyond its exit too.
  const aoi_tier next = cur == aoi_tier::inner ? aoi_tier::middle : aoi_tier::outer;
  beyond_since_s = next != aoi_tier::outer && d_m > exit_radius(next, p) ? now_s : -1.0;
  return next;
}

float aoi_audio_gain(double d_m, const aoi_params &p) noexcept {
  if (!p.enabled || !(d_m > 0.0))
    return 1.0f;
  double g = 1.0;
  if (p.audio_rolloff && p.inner_enter_m > 0.0 && d_m > p.inner_enter_m)
    g = p.inner_enter_m / d_m;
  if (p.audio_fade_m > 0.0)
    g *= (std::max)(0.0, (std::min)(1.0, (p.middle_enter_m - d_m) / p.audio_fade_m));
  else if (d_m >= p.middle_enter_m)
    g = 0.0;
  return static_cast<float>(g);
}

std::uint64_t aoi_pose_push_interval_ticks(const aoi_params &p, double tick_hz) noexcept {
  return p.pose_push_hz > 0.0 ? to_ticks(1.0 / p.pose_push_hz, tick_hz) : 0;
}

std::uint64_t aoi_pose_refresh_ticks(const aoi_params &p, double tick_hz) noexcept {
  return to_ticks(p.pose_refresh_s, tick_hz);
}

// ---------------- aoi_snapshot ----------------

std::size_t aoi_snapshot::index_of(std::string_view pid) const noexcept {
  const auto it = std::lower_bound(
      _pids.begin(), _pids.end(), pid,
      [](const std::string &a, std::string_view b) { return std::string_view(a) < b; });
  return it != _pids.end() && std::string_view(*it) == pid
             ? static_cast<std::size_t>(it - _pids.begin())
             : npos;
}

bool aoi_snapshot::is_departed(std::string_view pid) const noexcept {
  return std::binary_search(_departed.begin(), _departed.end(), pid,
                            [](std::string_view a, std::string_view b) { return a < b; });
}

bool aoi_snapshot::deliver_path(const std::string &receiver,
                                const std::string &path) const noexcept {
  if (!_enabled || _prefix.empty())
    return true;
  const std::string_view p(path);
  if (p.size() <= _prefix.size() || p.substr(0, _prefix.size()) != _prefix)
    return true; // outside this room's participants: scene, server, aoi, other rooms
  std::size_t pos = _prefix.size();
  const std::string_view q = next_segment(p, pos);
  if (q.empty() || q == receiver)
    return true; // malformed, or the receiver's own subtree
  if (pos == kNpos)
    return true; // the bare participants.<q> node
  const std::string_view family = next_segment(p, pos);
  const bool pose = family == "pose";
  const bool intent = family == "intent";
  if (is_departed(q))
    return !(pose || intent || family == "net" || family == "voice");
  const std::size_t qi = index_of(q);
  if (qi == npos)
    return true; // a pid this snapshot has never seen
  if (intent)
    return false;
  if (pose) {
    const std::size_t ri = index_of(receiver);
    return ri == npos || _tiers[ri * _pids.size() + qi] == aoi_tier::inner;
  }
  return true;
}

aoi_media aoi_snapshot::media(const std::string &receiver, const std::string &publisher,
                              const std::string &stream_id,
                              const std::string &codec) const noexcept {
  if (!_enabled || receiver == publisher)
    return aoi_media::deliver;
  const std::size_t ri = index_of(receiver);
  const std::size_t qi = index_of(publisher);
  if (ri == npos || qi == npos)
    return aoi_media::deliver;
  const std::size_t cell = ri * _pids.size() + qi;
  const aoi_tier t = _tiers[cell];
  if (t == aoi_tier::outer)
    return aoi_media::drop;
  // Only an intra-only camera can be decimated; anything else is all or nothing.
  if (stream_id != "camera" || codec != "mjpeg")
    return aoi_media::deliver;
  if (t == aoi_tier::middle)
    return aoi_media::thumbnail;
  return _video_capped && !_ranked[cell] ? aoi_media::thumbnail : aoi_media::deliver;
}

aoi_tier aoi_snapshot::tier(const std::string &receiver,
                            const std::string &subject) const noexcept {
  const std::size_t ri = index_of(receiver);
  const std::size_t qi = index_of(subject);
  if (ri == npos || qi == npos || ri == qi)
    return aoi_tier::inner;
  return _tiers[ri * _pids.size() + qi];
}

std::uint64_t aoi_snapshot::generation() const noexcept { return _generation; }

bool aoi_snapshot::enabled() const noexcept { return _enabled; }

const std::vector<std::string> &aoi_snapshot::participants() const noexcept { return _pids; }

// ---------------- aoi_state_filter ----------------

aoi_state_filter::aoi_state_filter(std::shared_ptr<const aoi_snapshot> s, std::string receiver)
    : _snap(std::move(s)), _receiver(std::move(receiver)) {}

bool aoi_state_filter::deliver(const std::string &path) const noexcept {
  return !_snap || _snap->deliver_path(_receiver, path);
}

// ---------------- room_aoi ----------------

room_aoi::room_aoi(std::string room, aoi_params p)
    : _room(std::move(room)), _p(std::move(p)), _prefix(participants_prefix(_room)) {
  auto s = std::make_shared<aoi_snapshot>();
  s->_enabled = _p.enabled;
  s->_video_capped = _p.video_full_max > 0;
  s->_prefix = _prefix;
  _snap = std::move(s);
}

bool room_aoi::update(const std::vector<position> &present, std::uint64_t now_tick, double now_s) {
  std::map<std::string, const position *> here; // emplace keeps a repeated pid's first entry
  for (const position &pos : present)
    if (!pos.pid.empty())
      here.emplace(pos.pid, &pos);

  bool changed = false;
  for (auto it = _members.begin(); it != _members.end();) {
    if (here.count(it->first)) {
      ++it;
      continue;
    }
    for (auto &kv : _members) {
      kv.second.subjects.erase(it->first);
      kv.second.ranked.erase(it->first);
    }
    _departed.insert(it->first);
    it = _members.erase(it);
    changed = true;
  }
  for (const auto &kv : here) {
    const auto [it, fresh] = _members.try_emplace(kv.first);
    if (fresh) {
      _departed.erase(kv.first);
      changed = true;
    }
    it->second.x = kv.second->x;
    it->second.y = kv.second->y;
  }

  // Each unordered pair is stepped once and mirrored, so tier(r, q) == tier(q, r) always.
  for (auto a = _members.begin(); a != _members.end(); ++a) {
    for (auto b = std::next(a); b != _members.end(); ++b) {
      const double d = std::hypot(b->second.x - a->second.x, b->second.y - a->second.y);
      const auto [ab_it, fresh] = a->second.subjects.try_emplace(b->first);
      pair_state &ab = ab_it->second;
      pair_state &ba = b->second.subjects[a->first];
      ab.d_m = d;
      if (fresh) {
        ab.tier = initial_tier(d, _p);
        ab.beyond_since_s = -1.0;
        // A receiver meeting a non-inner subject has none of its pose: send it now rather than
        // leave it at the origin until the refresh.
        ab.push_due = ab.tier != aoi_tier::inner;
        ab.last_push_tick = now_tick;
        ab.last_version = 0;
        ba = ab;
        changed = true;
        continue;
      }
      const aoi_tier t = step_tier(ab.tier, d, now_s, ab.beyond_since_s, _p);
      if (t != ab.tier) {
        // Entering inner turns the live pose stream back on; the push brings the whole subtree
        // current first, so the receiver never pairs a live tick with leaves it missed.
        if (t == aoi_tier::inner)
          ab.push_due = ba.push_due = true;
        ab.tier = t;
        changed = true;
      }
      ba.tier = ab.tier;
      ba.beyond_since_s = ab.beyond_since_s;
      ba.d_m = d;
    }
  }

  if (_p.video_full_max > 0)
    for (auto &kv : _members)
      changed = rerank(kv.second) || changed;

  if (changed)
    publish();
  return changed;
}

bool room_aoi::rerank(member &r) const {
  const std::size_t k = static_cast<std::size_t>(_p.video_full_max);
  // A non-negative margin, so a swap always moves the set strictly nearer and the loop ends.
  const double margin = (std::max)(0.0, _p.video_rank_margin_m);
  // The inner subjects, nearest first; subjects iterate in pid order, so ties stay in pid order.
  std::vector<std::pair<double, const std::string *>> inner;
  for (const auto &kv : r.subjects)
    if (kv.second.tier == aoi_tier::inner)
      inner.emplace_back(kv.second.d_m, &kv.first);
  std::stable_sort(inner.begin(), inner.end(),
                   [](const auto &a, const auto &b) { return a.first < b.first; });

  // Keep the members that are still inner, fill up with the nearest others, then swap a member
  // out only for an outsider nearer by more than the margin (no flapping on near-equal ranks).
  std::set<std::string> next;
  for (const auto &c : inner)
    if (r.ranked.count(*c.second))
      next.insert(*c.second);
  for (const auto &c : inner) {
    if (next.size() >= k)
      break;
    next.insert(*c.second);
  }
  for (;;) {
    const std::pair<double, const std::string *> *outsider = nullptr;
    const std::pair<double, const std::string *> *farthest = nullptr;
    for (const auto &c : inner)
      if (!next.count(*c.second)) {
        outsider = &c;
        break;
      }
    for (auto it = inner.rbegin(); it != inner.rend(); ++it)
      if (next.count(*it->second)) {
        farthest = &*it;
        break;
      }
    if (!outsider || !farthest || !(outsider->first + margin < farthest->first))
      break;
    next.erase(*farthest->second);
    next.insert(*outsider->second);
  }
  if (next == r.ranked)
    return false;
  r.ranked = std::move(next);
  return true;
}

void room_aoi::publish() {
  auto s = std::make_shared<aoi_snapshot>();
  s->_enabled = _p.enabled;
  s->_video_capped = _p.video_full_max > 0;
  s->_generation = ++_generation;
  s->_prefix = _prefix;
  const std::size_t n = _members.size();
  s->_pids.reserve(n);
  for (const auto &kv : _members)
    s->_pids.push_back(kv.first); // std::map order: sorted, as index_of requires
  s->_tiers.assign(n * n, aoi_tier::inner);
  s->_ranked.assign(n * n, 0);
  std::size_t ri = 0;
  for (const auto &kv : _members) {
    for (const auto &sv : kv.second.subjects) {
      const std::size_t qi = s->index_of(sv.first);
      if (qi == aoi_snapshot::npos)
        continue;
      s->_tiers[ri * n + qi] = sv.second.tier;
      s->_ranked[ri * n + qi] = kv.second.ranked.count(sv.first) ? 1 : 0;
    }
    ++ri;
  }
  s->_departed.assign(_departed.begin(), _departed.end());
  _snap = std::move(s);
}

std::vector<aoi_push>
room_aoi::take_pose_pushes(std::uint64_t now_tick,
                           const std::map<std::string, std::uint64_t> &pose_version) {
  std::vector<aoi_push> out;
  if (!_p.enabled)
    return out;
  const std::uint64_t every = aoi_pose_push_interval_ticks(_p);
  const std::uint64_t refresh = aoi_pose_refresh_ticks(_p);
  for (auto &rm : _members) {
    aoi_push push;
    for (auto &qs : rm.second.subjects) {
      pair_state &ps = qs.second;
      const auto v_it = pose_version.find(qs.first);
      const std::uint64_t v = v_it == pose_version.end() ? 0 : v_it->second;
      if (now_tick < ps.last_push_tick)
        ps.last_push_tick = now_tick; // a tick that went back: count the intervals from now
      bool due = ps.push_due;
      if (!due && ps.tier == aoi_tier::inner) {
        // The live stream keeps an inner receiver current, so it counts as a push every step and
        // a later downgrade's schedule starts from here.
        ps.last_push_tick = now_tick;
        ps.last_version = v;
        continue;
      }
      if (!due) {
        const std::uint64_t since = now_tick - ps.last_push_tick;
        due = (every != 0 && v != ps.last_version && since >= every) ||
              (refresh != 0 && since >= refresh);
      }
      if (!due)
        continue;
      push.subjects.push_back(qs.first);
      ps.push_due = false;
      ps.last_push_tick = now_tick;
      ps.last_version = v;
    }
    if (!push.subjects.empty()) {
      push.receiver = rm.first;
      out.push_back(std::move(push));
    }
  }
  return out;
}

std::shared_ptr<const aoi_snapshot> room_aoi::snapshot() const { return _snap; }

const std::set<std::string> &room_aoi::departed() const { return _departed; }

const aoi_params &room_aoi::params() const noexcept { return _p; }

const std::string &room_aoi::room() const noexcept { return _room; }

// ---------------- replicated parameters ----------------

std::string room_aoi_path(const std::string &room_id) {
  return sub("conf", {"rooms", room_id.c_str(), "aoi"});
}

void write_aoi_params(cvc::app &app, const std::string &room, const aoi_params &p) {
  const std::string base = room_aoi_path(room);
  set_replicated(app, sub(base, {"preset"}), p.preset);
  set_replicated(app, sub(base, {"inner_enter_m"}), fmt(p.inner_enter_m));
  set_replicated(app, sub(base, {"inner_exit_m"}), fmt(p.inner_exit_m));
  set_replicated(app, sub(base, {"middle_enter_m"}), fmt(p.middle_enter_m));
  set_replicated(app, sub(base, {"middle_exit_m"}), fmt(p.middle_exit_m));
  set_replicated(app, sub(base, {"downgrade_dwell_s"}), fmt(p.downgrade_dwell_s));
  set_replicated(app, sub(base, {"pose_push_hz"}), fmt(p.pose_push_hz));
  set_replicated(app, sub(base, {"pose_refresh_s"}), fmt(p.pose_refresh_s));
  set_replicated(app, sub(base, {"thumb_fps"}), fmt(p.thumb_fps));
  set_replicated(app, sub(base, {"video_rank_margin_m"}), fmt(p.video_rank_margin_m));
  set_replicated(app, sub(base, {"audio_fade_m"}), fmt(p.audio_fade_m));
  set_replicated(app, sub(base, {"video_full_max"}), std::to_string(p.video_full_max));
  set_replicated(app, sub(base, {"audio_rolloff"}), p.audio_rolloff ? "1" : "0");
  // Written LAST and read as the presence check: the journal replicates in write order, so a
  // reader that sees it also sees the fields of that first write.
  set_replicated(app, sub(base, {"enabled"}), p.enabled ? "1" : "0");
}

bool read_aoi_params(cvc::app &app, const std::string &room, aoi_params &out) {
  const std::string base = room_aoi_path(room);
  if (!find(app, sub(base, {"enabled"})))
    return false;
  aoi_params p = out;
  p.preset = get_s(app, sub(base, {"preset"}), p.preset);
  p.inner_enter_m = get_d(app, sub(base, {"inner_enter_m"}), p.inner_enter_m);
  p.inner_exit_m = get_d(app, sub(base, {"inner_exit_m"}), p.inner_exit_m);
  p.middle_enter_m = get_d(app, sub(base, {"middle_enter_m"}), p.middle_enter_m);
  p.middle_exit_m = get_d(app, sub(base, {"middle_exit_m"}), p.middle_exit_m);
  p.downgrade_dwell_s = get_d(app, sub(base, {"downgrade_dwell_s"}), p.downgrade_dwell_s);
  p.pose_push_hz = get_d(app, sub(base, {"pose_push_hz"}), p.pose_push_hz);
  p.pose_refresh_s = get_d(app, sub(base, {"pose_refresh_s"}), p.pose_refresh_s);
  p.thumb_fps = get_d(app, sub(base, {"thumb_fps"}), p.thumb_fps);
  p.video_rank_margin_m = get_d(app, sub(base, {"video_rank_margin_m"}), p.video_rank_margin_m);
  p.audio_fade_m = get_d(app, sub(base, {"audio_fade_m"}), p.audio_fade_m);
  p.video_full_max = get_i(app, sub(base, {"video_full_max"}), p.video_full_max);
  p.audio_rolloff = get_b(app, sub(base, {"audio_rolloff"}), p.audio_rolloff);
  p.enabled = get_b(app, sub(base, {"enabled"}), p.enabled);
  out = std::move(p);
  return true;
}

} // namespace conference
} // namespace ariadne
} // namespace cvc
