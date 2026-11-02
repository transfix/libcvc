/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The fast lane over WebRTC data channels. Without CVC_ENABLE_RTC every class is an inert stub
// (no lane ever opens, an offer is never answered), so callers need no #if of their own.

#include <atomic>
#include <cvc/net/fast_lane_rtc.h>
#include <map>
#include <mutex>
#include <utility>

#if defined(CVC_ENABLE_RTC)
#include <array>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <rtc/rtc.hpp>
#include <thread>
#include <variant>
#endif

namespace cvc {

#if defined(CVC_ENABLE_RTC)

namespace {

using clock_type = std::chrono::steady_clock;

std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(clock_type::now().time_since_epoch())
      .count();
}

double seconds_since(std::int64_t ns) { return static_cast<double>(steady_ns() - ns) * 1e-9; }

const std::byte *as_bytes(const std::uint8_t *p) { return reinterpret_cast<const std::byte *>(p); }

// CVC_LANE_TRACE=1: the lanes' signaling and state changes on stderr (field diagnosis).
bool tracing() {
  static const bool on = [] {
    const char *e = std::getenv("CVC_LANE_TRACE");
    return e && *e && std::strcmp(e, "0") != 0;
  }();
  return on;
}

template <class... A> void trace(const char *fmt, A... a) {
  if (tracing()) {
    std::fprintf(stderr, fmt, a...);
    std::fflush(stderr);
  }
}

// The data channel every lane uses: unordered; never retransmitted, or (LIFETIME > 0) retransmitted
// only while younger than LIFETIME.
rtc::DataChannelInit lane_channel_init(std::chrono::milliseconds lifetime) {
  rtc::DataChannelInit init;
  init.reliability.unordered = true;
  if (lifetime.count() > 0)
    init.reliability.maxPacketLifeTime = lifetime;
  else
    init.reliability.maxRetransmits = 0u;
  return init;
}

// Send on DC unless its buffer is past MAX. False when it is closed, full or the send throws.
bool send_on(const std::shared_ptr<rtc::DataChannel> &dc, const std::uint8_t *data, std::size_t n,
             std::size_t max) {
  if (!dc || !dc->isOpen() || dc->bufferedAmount() > max)
    return false;
  try {
    dc->send(as_bytes(data), n); // false only means "buffered", which is still sent
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

} // namespace

bool fast_lane_rtc_available() noexcept { return true; }

void set_fast_lane_sctp(const fast_lane_sctp &t) {
#if !defined(__EMSCRIPTEN__)
  rtc::SctpSettings s;
  if (t.send_buffer > 0)
    s.sendBufferSize = t.send_buffer;
  if (t.delayed_sack.count() > 0)
    s.delayedSackTime = t.delayed_sack;
  if (t.cc_module >= 0)
    s.congestionControlModule = static_cast<unsigned int>(t.cc_module);
  if (t.initial_cwnd_mtus > 0)
    s.initialCongestionWindow = t.initial_cwnd_mtus;
  rtc::SetSctpSettings(s);
#else
  (void)t;
#endif
}

#if !defined(__EMSCRIPTEN__)

// ---------------- the relay's lanes ----------------

namespace {

// SDP with a host candidate on PORT for each of ADDRS, before the end of its candidates.
std::string with_candidates(std::string sdp, const std::vector<std::string> &addrs,
                            std::uint16_t port) {
  if (addrs.empty())
    return sdp;
  std::string lines;
  int foundation = 900;
  for (const std::string &a : addrs)
    lines += "a=candidate:" + std::to_string(foundation++) + " 1 UDP 2130706431 " + a + " " +
             std::to_string(port) + " typ host\r\n";
  const std::size_t eoc = sdp.find("a=end-of-candidates");
  if (eoc != std::string::npos) {
    sdp.insert(eoc, lines);
  } else {
    if (!sdp.empty() && sdp.back() != '\n')
      sdp += "\r\n";
    sdp += lines;
  }
  return sdp;
}

struct server_lane {
  std::string pid;
  std::shared_ptr<rtc::PeerConnection> pc;
  std::mutex dc_mu;
  std::shared_ptr<rtc::DataChannel> dc;
  std::atomic<bool> open{false};
  std::atomic<bool> down{false};
  cvc::state_link_stats stats;
  std::int64_t born_ns = steady_ns();
  std::atomic<std::int64_t> rx_ns{steady_ns()};

  std::shared_ptr<rtc::DataChannel> channel() {
    std::lock_guard<std::mutex> lk(dc_mu);
    return dc;
  }
};

} // namespace

struct fast_lane_server::impl : std::enable_shared_from_this<impl> {
  explicit impl(fast_lane_server_config c) : cfg(std::move(c)) {}

  fast_lane_server_config cfg;
  mutable std::mutex mu;
  std::map<std::string, std::shared_ptr<server_lane>> lanes;
  // Lanes that went down, released by the prober (never inside the lane's own callbacks).
  std::vector<std::pair<std::int64_t, std::shared_ptr<server_lane>>> retired;
  std::atomic<std::uint64_t> generation{0};

  std::mutex handler_mu;
  std::array<datagram_fn, 8> handlers;

  std::mutex media_mu;
  std::shared_ptr<const media_sink_fn> sink;
  std::shared_ptr<const media_gate_fn> gate;
  std::shared_ptr<const media_shape_fn> shaper;

  std::atomic<std::uint64_t> offers{0}, opened{0}, dg_in{0}, dg_out{0}, dropped_full{0},
      media_in{0}, media_out{0}, media_gated{0}, refused{0}, idle_closed{0};

  std::mutex stop_mu;
  std::condition_variable stop_cv;
  bool stopping = false;
  std::thread prober;

  bool send(server_lane &l, const std::uint8_t *data, std::size_t n) {
    if (!l.open.load())
      return false;
    const std::shared_ptr<rtc::DataChannel> dc = l.channel();
    if (dc && dc->isOpen() && dc->bufferedAmount() > cfg.send_buffer_max) {
      dropped_full.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    if (!send_on(dc, data, n, cfg.send_buffer_max))
      return false;
    l.stats.note_tx(n);
    dg_out.fetch_add(1, std::memory_order_relaxed);
    return true;
  }

  void lane_down(const std::shared_ptr<server_lane> &l) {
    if (l->down.exchange(true))
      return;
    if (l->open.exchange(false))
      generation.fetch_add(1);
    std::lock_guard<std::mutex> lk(mu);
    const auto it = lanes.find(l->pid);
    if (it != lanes.end() && it->second == l)
      lanes.erase(it);
    retired.emplace_back(steady_ns(), l);
  }

  void on_message(const std::shared_ptr<server_lane> &l, const std::uint8_t *d, std::size_t n) {
    l->stats.note_rx(n);
    l->rx_ns.store(steady_ns());
    dg_in.fetch_add(1, std::memory_order_relaxed);
    const std::uint8_t k = lane_kind_of(d, n);
    if (k == static_cast<std::uint8_t>(lane_kind::ping) ||
        k == static_cast<std::uint8_t>(lane_kind::pong)) {
      lane_kind kind{};
      std::uint64_t seq = 0;
      std::int64_t sent = 0;
      if (!decode_lane_probe(d, n, kind, seq, sent)) {
        refused.fetch_add(1, std::memory_order_relaxed);
      } else if (kind == lane_kind::ping) {
        const std::vector<std::uint8_t> pong = encode_lane_probe(lane_kind::pong, seq, sent);
        send(*l, pong.data(), pong.size());
      } else {
        l->stats.note_rtt(steady_ns() - sent);
      }
      return;
    }
    if (k == static_cast<std::uint8_t>(lane_kind::media)) {
      media_frame f;
      // A lane speaks only as the pid whose reliable connection offered it.
      if (!decode_lane_media(d, n, f) || f.origin_node_id != l->pid) {
        refused.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      media_in.fetch_add(1, std::memory_order_relaxed);
      std::shared_ptr<const media_sink_fn> s;
      {
        std::lock_guard<std::mutex> lk(media_mu);
        s = sink;
      }
      if (s && *s)
        (*s)(f, l.get());
      return;
    }
    datagram_fn h;
    if (k < handlers.size()) {
      std::lock_guard<std::mutex> lk(handler_mu);
      h = handlers[k];
    }
    if (h)
      h(l->pid, d, n);
    else
      refused.fetch_add(1, std::memory_order_relaxed);
  }

  void attach(const std::shared_ptr<server_lane> &l, std::shared_ptr<rtc::DataChannel> dc) {
    const std::weak_ptr<impl> wi = shared_from_this();
    const std::weak_ptr<server_lane> wl = l;
    const auto mark_open = [wi, wl] {
      const auto i = wi.lock();
      const auto ln = wl.lock();
      if (!i || !ln || ln->down.load() || ln->open.exchange(true))
        return;
      i->opened.fetch_add(1);
      i->generation.fetch_add(1);
    };
    dc->onOpen(mark_open);
    dc->onClosed([wi, wl] {
      const auto i = wi.lock();
      const auto ln = wl.lock();
      if (i && ln)
        i->lane_down(ln);
    });
    dc->onMessage(
        [wi, wl](rtc::binary b) {
          const auto i = wi.lock();
          const auto ln = wl.lock();
          if (i && ln)
            i->on_message(ln, reinterpret_cast<const std::uint8_t *>(b.data()), b.size());
        },
        [](rtc::string) {});
    {
      std::lock_guard<std::mutex> lk(l->dc_mu);
      l->dc = dc;
    }
    // A channel the remote opened can be open by the time it is handed over.
    if (dc->isOpen())
      mark_open();
  }

  void probe_loop() {
    std::unique_lock<std::mutex> lk(stop_mu);
    while (!stopping) {
      stop_cv.wait_for(lk, cfg.probe_interval);
      if (stopping)
        break;
      lk.unlock();
      std::vector<std::shared_ptr<server_lane>> ls, silent;
      std::vector<std::shared_ptr<server_lane>> release;
      const std::int64_t idle_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(cfg.idle_timeout).count();
      {
        std::lock_guard<std::mutex> g(mu);
        for (const auto &kv : lanes)
          if (kv.second->open.load()) {
            if (idle_ns > 0 && steady_ns() - kv.second->rx_ns.load() > idle_ns)
              silent.push_back(kv.second);
            else
              ls.push_back(kv.second);
          }
        const std::int64_t old = steady_ns() - 2'000'000'000;
        for (auto it = retired.begin(); it != retired.end();)
          if (it->first < old) {
            release.push_back(std::move(it->second));
            it = retired.erase(it);
          } else {
            ++it;
          }
      }
      for (const auto &l : silent) {
        trace("[lane] relay: %s silent past the idle timeout; closing it\n", l->pid.c_str());
        idle_closed.fetch_add(1, std::memory_order_relaxed);
        lane_down(l);
      }
      for (const auto &l : ls) {
        const std::vector<std::uint8_t> ping =
            encode_lane_probe(lane_kind::ping, l->stats.next_ping_seq(), steady_ns());
        send(*l, ping.data(), ping.size());
      }
      for (const auto &l : release)
        if (l->pc)
          l->pc->close();
      release.clear(); // the last references: the peer connections go here, on this thread
      lk.lock();
    }
  }
};

fast_lane_server::fast_lane_server(fast_lane_server_config cfg)
    : _impl(std::make_shared<impl>(std::move(cfg))) {
  _impl->prober = std::thread([i = _impl.get()] { i->probe_loop(); });
}

fast_lane_server::~fast_lane_server() {
  {
    std::lock_guard<std::mutex> lk(_impl->stop_mu);
    _impl->stopping = true;
  }
  _impl->stop_cv.notify_all();
  if (_impl->prober.joinable())
    _impl->prober.join();
  std::vector<std::shared_ptr<server_lane>> all;
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    for (auto &kv : _impl->lanes)
      all.push_back(std::move(kv.second));
    _impl->lanes.clear();
    for (auto &r : _impl->retired)
      all.push_back(std::move(r.second));
    _impl->retired.clear();
  }
  for (const auto &l : all) {
    l->down.store(true);
    if (l->pc)
      l->pc->close();
  }
}

void fast_lane_server::offer(const std::string &pid, const std::string &sdp,
                             std::function<void(const std::string &answer)> reply) {
  if (pid.empty() || sdp.empty())
    return;
  _impl->offers.fetch_add(1, std::memory_order_relaxed);
  const fast_lane_server_config &cfg = _impl->cfg;
  rtc::Configuration c;
  c.bindAddress = cfg.bind_address;
  c.portRangeBegin = cfg.port;
  c.portRangeEnd = cfg.port;
  c.enableIceUdpMux = true;
  c.maxMessageSize = kLaneMaxDatagram;
  auto l = std::make_shared<server_lane>();
  l->pid = pid;
  try {
    l->pc = std::make_shared<rtc::PeerConnection>(c);
  } catch (const std::exception &) {
    return;
  }
  const std::weak_ptr<impl> wi = _impl;
  const std::weak_ptr<server_lane> wl = l;
  auto replied = std::make_shared<std::atomic<bool>>(false);
  l->pc->onGatheringStateChange([wl, replied, reply = std::move(reply), addrs = cfg.advertise,
                                 port = cfg.port](rtc::PeerConnection::GatheringState s) {
    if (s != rtc::PeerConnection::GatheringState::Complete)
      return;
    const auto ln = wl.lock();
    if (!ln || replied->exchange(true))
      return;
    const auto d = ln->pc->localDescription();
    trace("[lane] relay: answer for %s gathered (%s)\n", ln->pid.c_str(),
          d ? "sending" : "no local description");
    if (d && reply)
      reply(with_candidates(std::string(*d), addrs, port));
  });
  l->pc->onDataChannel([wi, wl](std::shared_ptr<rtc::DataChannel> dc) {
    const auto i = wi.lock();
    const auto ln = wl.lock();
    if (i && ln)
      i->attach(ln, std::move(dc));
  });
  l->pc->onStateChange([wi, wl](rtc::PeerConnection::State s) {
    if (const auto ln = wl.lock())
      trace("[lane] relay: %s peer connection state %d\n", ln->pid.c_str(), static_cast<int>(s));
    if (s != rtc::PeerConnection::State::Failed && s != rtc::PeerConnection::State::Closed &&
        s != rtc::PeerConnection::State::Disconnected)
      return;
    const auto i = wi.lock();
    const auto ln = wl.lock();
    if (i && ln)
      i->lane_down(ln);
  });
  std::shared_ptr<server_lane> old;
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    std::shared_ptr<server_lane> &slot = _impl->lanes[pid];
    old = std::move(slot);
    slot = l;
  }
  if (old)
    _impl->lane_down(old);
  trace("[lane] relay: offer from %s (%zu bytes)\n", pid.c_str(), sdp.size());
  try {
    l->pc->setRemoteDescription(rtc::Description(sdp, "offer"));
  } catch (const std::exception &e) {
    trace("[lane] relay: offer from %s refused: %s\n", pid.c_str(), e.what());
    _impl->lane_down(l);
  }
}

void fast_lane_server::close(const std::string &pid) {
  std::shared_ptr<server_lane> l;
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    const auto it = _impl->lanes.find(pid);
    if (it == _impl->lanes.end())
      return;
    l = it->second;
  }
  _impl->lane_down(l);
}

bool fast_lane_server::lane_open(const std::string &pid) const {
  std::lock_guard<std::mutex> lk(_impl->mu);
  const auto it = _impl->lanes.find(pid);
  return it != _impl->lanes.end() && it->second->open.load();
}

std::uint64_t fast_lane_server::lane_generation() const noexcept {
  return _impl->generation.load();
}

bool fast_lane_server::send_datagram(const std::string &pid, const std::uint8_t *data,
                                     std::size_t n) {
  std::shared_ptr<server_lane> l;
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    const auto it = _impl->lanes.find(pid);
    if (it == _impl->lanes.end())
      return false;
    l = it->second;
  }
  return _impl->send(*l, data, n);
}

void fast_lane_server::set_on_datagram(lane_kind kind, datagram_fn fn) {
  const auto k = static_cast<std::size_t>(kind);
  if (k >= _impl->handlers.size())
    return;
  std::lock_guard<std::mutex> lk(_impl->handler_mu);
  _impl->handlers[k] = std::move(fn);
}

void fast_lane_server::fan_media(const media_frame &f, const void *exclude) {
  std::shared_ptr<const media_gate_fn> gate;
  std::shared_ptr<const media_shape_fn> shaper;
  {
    std::lock_guard<std::mutex> lk(_impl->media_mu);
    gate = _impl->gate;
    shaper = _impl->shaper;
  }
  std::vector<std::shared_ptr<server_lane>> ls;
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    for (const auto &kv : _impl->lanes)
      if (kv.second.get() != exclude && kv.first != f.origin_node_id && kv.second->open.load())
        ls.push_back(kv.second);
  }
  if (ls.empty())
    return;
  // One encoding per frame sent (the frame as is, or each variant the shaper made), kept alive
  // with its frame so a pointer is never reused within this fan.
  std::vector<std::pair<std::shared_ptr<const media_frame>, std::vector<std::uint8_t>>> made;
  std::vector<std::uint8_t> as_is;
  for (const auto &l : ls) {
    if (gate && *gate && !(*gate)(f, l->pid)) {
      _impl->media_gated.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    media_shape sh;
    if (shaper && *shaper)
      sh = (*shaper)(f, l->pid);
    if (sh.drop) {
      _impl->media_gated.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    const std::vector<std::uint8_t> *bytes = nullptr;
    if (!sh.variant) {
      if (as_is.empty())
        as_is = encode_lane_media(f);
      bytes = &as_is;
    } else {
      for (const auto &m : made)
        if (m.first == sh.variant)
          bytes = &m.second;
      if (!bytes) {
        made.emplace_back(sh.variant, encode_lane_media(*sh.variant));
        bytes = &made.back().second;
      }
    }
    if (bytes->empty())
      continue; // larger than a lane datagram
    if (_impl->send(*l, bytes->data(), bytes->size())) {
      l->stats.note_media_sent();
      _impl->media_out.fetch_add(1, std::memory_order_relaxed);
    } else {
      l->stats.note_media_dropped();
    }
  }
}

void fast_lane_server::set_media_sink(media_sink_fn sink) {
  std::lock_guard<std::mutex> lk(_impl->media_mu);
  _impl->sink = sink ? std::make_shared<const media_sink_fn>(std::move(sink)) : nullptr;
}

void fast_lane_server::set_media_gate(media_gate_fn gate) {
  std::lock_guard<std::mutex> lk(_impl->media_mu);
  _impl->gate = gate ? std::make_shared<const media_gate_fn>(std::move(gate)) : nullptr;
}

void fast_lane_server::set_media_shaper(media_shape_fn shaper) {
  std::lock_guard<std::mutex> lk(_impl->media_mu);
  _impl->shaper = shaper ? std::make_shared<const media_shape_fn>(std::move(shaper)) : nullptr;
}

std::vector<state_link_report> fast_lane_server::link_reports() const {
  std::vector<state_link_report> out;
  std::lock_guard<std::mutex> lk(_impl->mu);
  for (const auto &kv : _impl->lanes) {
    const server_lane &l = *kv.second;
    if (!l.open.load())
      continue;
    state_link_report r;
    r.transport = _impl->cfg.transport_name;
    r.remote_node_id = l.pid;
    r.accepted = true;
    r.age_s = seconds_since(l.born_ns);
    r.idle_s = seconds_since(l.rx_ns.load());
    r.stats = l.stats.read();
    out.push_back(std::move(r));
  }
  return out;
}

fast_lane_server::totals fast_lane_server::counters() const {
  totals t;
  t.offers = _impl->offers.load();
  t.opened = _impl->opened.load();
  t.datagrams_in = _impl->dg_in.load();
  t.datagrams_out = _impl->dg_out.load();
  t.dropped_full = _impl->dropped_full.load();
  t.media_in = _impl->media_in.load();
  t.media_out = _impl->media_out.load();
  t.media_gated = _impl->media_gated.load();
  t.refused = _impl->refused.load();
  t.idle_closed = _impl->idle_closed.load();
  return t;
}

const fast_lane_server_config &fast_lane_server::config() const noexcept { return _impl->cfg; }

#endif // !__EMSCRIPTEN__

// ---------------- a client's lane ----------------

struct fast_lane_client::impl : std::enable_shared_from_this<impl> {
  offer_fn send_offer;
  mutable std::mutex mu;
  std::shared_ptr<rtc::PeerConnection> pc;
  std::shared_ptr<rtc::DataChannel> dc;
  std::vector<std::shared_ptr<rtc::PeerConnection>> retired; // released at the next start/close
  std::uint64_t epoch = 0; // which start() the live callbacks belong to
  std::atomic<bool> open{false};
  datagram_fn on_dg;
  std::function<void(bool)> on_state;
  std::size_t send_buffer_max = 64 * 1024;
  std::chrono::milliseconds lifetime{0};
  cvc::state_link_stats stats;
  std::int64_t born_ns = steady_ns();
  std::atomic<std::int64_t> rx_ns{steady_ns()};

  bool current(std::uint64_t e) {
    std::lock_guard<std::mutex> lk(mu);
    return e == epoch;
  }

  void set_open(std::uint64_t e, bool v) {
    if (!current(e) || open.exchange(v) == v)
      return;
    std::function<void(bool)> cb;
    {
      std::lock_guard<std::mutex> lk(mu);
      cb = on_state;
    }
    if (cb)
      cb(v);
  }

  bool send(const std::uint8_t *data, std::size_t n) {
    std::shared_ptr<rtc::DataChannel> c;
    std::size_t max = 0;
    {
      std::lock_guard<std::mutex> lk(mu);
      c = dc;
      max = send_buffer_max;
    }
    if (!open.load() || !send_on(c, data, n, max))
      return false;
    stats.note_tx(n);
    return true;
  }

  void on_message(std::uint64_t e, const std::uint8_t *d, std::size_t n) {
    if (!current(e))
      return;
    stats.note_rx(n);
    rx_ns.store(steady_ns());
    lane_kind kind{};
    std::uint64_t seq = 0;
    std::int64_t sent = 0;
    if (decode_lane_probe(d, n, kind, seq, sent)) {
      if (kind == lane_kind::ping) {
        const std::vector<std::uint8_t> pong = encode_lane_probe(lane_kind::pong, seq, sent);
        send(pong.data(), pong.size());
      } else {
        stats.note_rtt(steady_ns() - sent);
      }
      return;
    }
    datagram_fn cb;
    {
      std::lock_guard<std::mutex> lk(mu);
      cb = on_dg;
    }
    if (cb)
      cb(d, n);
  }

  // Detach the live connection under MU: its callbacks stop counting at once. The caller closes
  // what this returns after letting go of MU (a close may call back synchronously).
  std::pair<std::shared_ptr<rtc::DataChannel>, std::shared_ptr<rtc::PeerConnection>>
  detach_locked() {
    ++epoch;
    return {std::exchange(dc, nullptr), std::exchange(pc, nullptr)};
  }
};

namespace {

void close_quietly(const std::shared_ptr<rtc::DataChannel> &dc,
                   const std::shared_ptr<rtc::PeerConnection> &pc) {
  try {
    if (dc)
      dc->close();
    if (pc)
      pc->close();
  } catch (const std::exception &) {
  }
}

} // namespace

fast_lane_client::fast_lane_client(offer_fn send_offer) : _impl(std::make_shared<impl>()) {
  _impl->send_offer = std::move(send_offer);
}

fast_lane_client::~fast_lane_client() { close(); }

void fast_lane_client::start() {
  std::vector<std::shared_ptr<rtc::PeerConnection>> old;
  std::uint64_t e = 0;
  std::shared_ptr<rtc::PeerConnection> pc;
  auto prev = [&] {
    std::lock_guard<std::mutex> lk(_impl->mu);
    old.swap(_impl->retired);
    return _impl->detach_locked();
  }();
  close_quietly(prev.first, prev.second);
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    // The connection just closed is released at the next start or close, outside its callbacks.
    if (prev.second)
      _impl->retired.push_back(std::move(prev.second));
    e = _impl->epoch;
    rtc::Configuration c; // no ICE servers: the relay's answer names its reachable address
#if !defined(__EMSCRIPTEN__)
    c.maxMessageSize = kLaneMaxDatagram;
#endif
    try {
      pc = std::make_shared<rtc::PeerConnection>(c);
    } catch (const std::exception &) {
      return;
    }
    _impl->pc = pc;
  }
  _impl->set_open(e, false);
  _impl->rx_ns.store(steady_ns()); // a new lane has heard nothing yet, and is not silent either
  old.clear();
  const std::weak_ptr<impl> wi = _impl;
  pc->onGatheringStateChange([wi, e](rtc::PeerConnection::GatheringState s) {
    if (s != rtc::PeerConnection::GatheringState::Complete)
      return;
    const auto i = wi.lock();
    if (!i)
      return;
    std::shared_ptr<rtc::PeerConnection> p;
    offer_fn f;
    {
      std::lock_guard<std::mutex> lk(i->mu);
      if (e != i->epoch)
        return;
      p = i->pc;
      f = i->send_offer;
    }
    const auto d = p ? p->localDescription() : std::nullopt;
    trace("[lane] client: offer gathered (%s)\n", d ? "sending" : "no local description");
    if (d && f)
      f(std::string(*d));
  });
  pc->onStateChange([wi, e](rtc::PeerConnection::State s) {
    trace("[lane] client: peer connection state %d\n", static_cast<int>(s));
    if (s != rtc::PeerConnection::State::Failed && s != rtc::PeerConnection::State::Closed &&
        s != rtc::PeerConnection::State::Disconnected)
      return;
    if (const auto i = wi.lock())
      i->set_open(e, false);
  });
  std::shared_ptr<rtc::DataChannel> dc;
  try {
    std::chrono::milliseconds lifetime{0};
    {
      std::lock_guard<std::mutex> lk(_impl->mu);
      lifetime = _impl->lifetime;
    }
    dc = pc->createDataChannel("lane", lane_channel_init(lifetime));
  } catch (const std::exception &) {
    return;
  }
  dc->onOpen([wi, e] {
    if (const auto i = wi.lock())
      i->set_open(e, true);
  });
  dc->onClosed([wi, e] {
    if (const auto i = wi.lock())
      i->set_open(e, false);
  });
  dc->onMessage(
      [wi, e](rtc::binary b) {
        if (const auto i = wi.lock())
          i->on_message(e, reinterpret_cast<const std::uint8_t *>(b.data()), b.size());
      },
      [](rtc::string) {});
  std::lock_guard<std::mutex> lk(_impl->mu);
  if (e == _impl->epoch)
    _impl->dc = std::move(dc);
}

void fast_lane_client::on_answer(const std::string &sdp) {
  std::shared_ptr<rtc::PeerConnection> pc;
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    pc = _impl->pc;
  }
  if (!pc || sdp.empty())
    return;
  trace("[lane] client: answer (%zu bytes)\n", sdp.size());
  try {
    pc->setRemoteDescription(rtc::Description(sdp, "answer"));
  } catch (const std::exception &e) {
    trace("[lane] client: answer refused: %s\n", e.what());
  }
}

void fast_lane_client::close() {
  std::vector<std::shared_ptr<rtc::PeerConnection>> old;
  bool was_open = false;
  auto prev = [&] {
    std::lock_guard<std::mutex> lk(_impl->mu);
    old.swap(_impl->retired);
    was_open = _impl->open.exchange(false);
    return _impl->detach_locked();
  }();
  close_quietly(prev.first, prev.second);
  std::function<void(bool)> cb;
  {
    std::lock_guard<std::mutex> lk(_impl->mu);
    cb = _impl->on_state;
  }
  if (was_open && cb)
    cb(false);
}

bool fast_lane_client::open() const { return _impl->open.load(); }

bool fast_lane_client::send(const std::uint8_t *data, std::size_t n) {
  return _impl->send(data, n);
}

void fast_lane_client::set_on_datagram(datagram_fn fn) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->on_dg = std::move(fn);
}

void fast_lane_client::set_on_state(std::function<void(bool open)> fn) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->on_state = std::move(fn);
}

void fast_lane_client::probe() {
  const std::vector<std::uint8_t> ping =
      encode_lane_probe(lane_kind::ping, _impl->stats.next_ping_seq(), steady_ns());
  _impl->send(ping.data(), ping.size());
}

state_link_report fast_lane_client::link_report() const {
  state_link_report r;
  r.transport = "webrtc";
  r.age_s = seconds_since(_impl->born_ns);
  r.idle_s = seconds_since(_impl->rx_ns.load());
  r.stats = _impl->stats.read();
  return r;
}

void fast_lane_client::set_send_buffer_max(std::size_t bytes) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->send_buffer_max = bytes;
}

void fast_lane_client::set_lifetime(std::chrono::milliseconds lifetime) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->lifetime = lifetime;
}

#else // !CVC_ENABLE_RTC: inert stubs

bool fast_lane_rtc_available() noexcept { return false; }
void set_fast_lane_sctp(const fast_lane_sctp &) {}

#if !defined(__EMSCRIPTEN__)
struct fast_lane_server::impl {
  fast_lane_server_config cfg;
};
fast_lane_server::fast_lane_server(fast_lane_server_config cfg)
    : _impl(std::make_shared<impl>(impl{std::move(cfg)})) {}
fast_lane_server::~fast_lane_server() = default;
void fast_lane_server::offer(const std::string &, const std::string &,
                             std::function<void(const std::string &)>) {}
void fast_lane_server::close(const std::string &) {}
bool fast_lane_server::lane_open(const std::string &) const { return false; }
std::uint64_t fast_lane_server::lane_generation() const noexcept { return 0; }
bool fast_lane_server::send_datagram(const std::string &, const std::uint8_t *, std::size_t) {
  return false;
}
void fast_lane_server::set_on_datagram(lane_kind, datagram_fn) {}
void fast_lane_server::fan_media(const media_frame &, const void *) {}
void fast_lane_server::set_media_sink(media_sink_fn) {}
void fast_lane_server::set_media_gate(media_gate_fn) {}
void fast_lane_server::set_media_shaper(media_shape_fn) {}
std::vector<state_link_report> fast_lane_server::link_reports() const { return {}; }
fast_lane_server::totals fast_lane_server::counters() const { return {}; }
const fast_lane_server_config &fast_lane_server::config() const noexcept { return _impl->cfg; }
#endif

struct fast_lane_client::impl {};
fast_lane_client::fast_lane_client(offer_fn) : _impl(std::make_shared<impl>()) {}
fast_lane_client::~fast_lane_client() = default;
void fast_lane_client::start() {}
void fast_lane_client::on_answer(const std::string &) {}
void fast_lane_client::close() {}
bool fast_lane_client::open() const { return false; }
bool fast_lane_client::send(const std::uint8_t *, std::size_t) { return false; }
void fast_lane_client::set_on_datagram(datagram_fn) {}
void fast_lane_client::set_on_state(std::function<void(bool)>) {}
void fast_lane_client::probe() {}
state_link_report fast_lane_client::link_report() const {
  state_link_report r;
  r.transport = "webrtc";
  return r;
}
void fast_lane_client::set_send_buffer_max(std::size_t) {}
void fast_lane_client::set_lifetime(std::chrono::milliseconds) {}

#endif // CVC_ENABLE_RTC

} // namespace cvc
