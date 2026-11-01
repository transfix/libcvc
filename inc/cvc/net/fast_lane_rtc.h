/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_NET_FAST_LANE_RTC_H__
#define __CVC_NET_FAST_LANE_RTC_H__

// The fast lane over WebRTC data channels (CVC_ENABLE_RTC): one peer connection per client, one
// data channel on it, unordered with maxRetransmits 0 -- SCTP partial reliability, so a message
// whose packet is lost is abandoned, never retransmitted, and later messages never wait for it.
// Natively it is libdatachannel (ICE by libjuice, DTLS by OpenSSL); in the browser the client is
// datachannel-wasm over the page's own RTCPeerConnection. The design and the measurements are in
// docs/roadmap/CONFERENCE_UNRELIABLE_TRANSPORT.md.
//
// SIGNALING rides the client's reliable connection (the caller carries the SDP both ways): the
// client offers, the relay answers once its candidates are gathered. The relay has a public
// address, so its answer's host candidates are enough: no STUN, no TURN. A client whose network
// blocks UDP never opens its lane and simply stays on its reliable connection.
//
// The relay binds ONE UDP port for every lane (ICE UDP mux). Behind a NAT or a firewall it is
// that one port to forward, and fast_lane_server_config::advertise names the public address to
// put in each answer.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/media/media_endpoint.h>
#include <cvc/net/fast_lane.h>
#include <cvc/state/state_link_stats.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

// Whether this build has the WebRTC lane (libdatachannel natively, datachannel-wasm in a browser).
bool fast_lane_rtc_available() noexcept;

// SCTP tuning for every lane in this process (libdatachannel's settings are process-wide; a
// browser keeps its own). Call before the first lane is made; native only (a no-op in a browser or
// without CVC_ENABLE_RTC). Zero leaves libdatachannel's default.
struct fast_lane_sctp {
  // SCTP's own send buffer. SCTP takes messages into it until its congestion window lets them go,
  // so what sits there waits out loss recovery unseen (libdatachannel: 1 MiB). A small one makes a
  // congested lane refuse sooner, and the lane drops (send_buffer_max) instead of queueing.
  std::size_t send_buffer = 0;
  // How long a receiver may hold a SACK (libdatachannel: 20 ms). The sender's congestion window
  // grows, after a loss halves it, one SACK at a time.
  std::chrono::milliseconds delayed_sack{0};
  // usrsctp's congestion control: 0 RFC 2581 (libdatachannel's default), 1 HSTCP, 2 H-TCP, 3 RTCC.
  int cc_module = -1;
  std::size_t initial_cwnd_mtus = 0; // libdatachannel: 10
};
void set_fast_lane_sctp(const fast_lane_sctp &s);

#if !defined(__EMSCRIPTEN__)

struct fast_lane_server_config {
  std::string bind_address = "0.0.0.0"; // where the lanes' UDP port is bound
  std::uint16_t port = 7703;            // the one UDP port every lane shares
  // Addresses written into each answer as host candidates on PORT, ahead of the ones gathered: the
  // relay's public address when it sits behind a NAT that forwards PORT to it. Empty: only the
  // addresses gathered on the host.
  std::vector<std::string> advertise;
  // A lane whose send buffer holds more than this drops what it is handed: the newer datagram
  // replaces it, instead of queueing behind it.
  std::size_t send_buffer_max = 64 * 1024;
  std::chrono::milliseconds probe_interval{1000}; // a ping on every open lane this often
  // A lane that has sent nothing for this long is closed: its client answers every ping and acks
  // every pose datagram, so silence means the path is gone, and that viewer falls back to its
  // reliable connection at once instead of when ICE gives up. 0: never.
  std::chrono::milliseconds idle_timeout{3000};
  // The transport its link reports name: a relay with a pose and a media lane per participant
  // names them apart ("webrtc/pose", "webrtc/media"), so their rates are metered apart.
  std::string transport_name = "webrtc";
};

// The relay's lanes: the fast_lane_hub the pose lane sends through (room_lane.h) and the
// media_endpoint its media_router fans through, with the per-receiver gate (area of interest) and
// shaper (video rungs) every endpoint applies. A media frame is sent only to lanes whose pid is not
// its origin. Thread-safe.
class fast_lane_server final : public fast_lane_hub, public media_endpoint {
public:
  struct totals {
    std::uint64_t offers = 0; // offers taken
    std::uint64_t opened = 0; // lanes that opened
    std::uint64_t datagrams_in = 0;
    std::uint64_t datagrams_out = 0;
    std::uint64_t dropped_full = 0; // datagrams dropped at a full send buffer
    std::uint64_t media_in = 0;     // media frames taken from lanes
    std::uint64_t media_out = 0;    // media frames sent on lanes
    std::uint64_t media_gated = 0;  // media frames the gate or shaper withheld
    std::uint64_t refused = 0;      // datagrams refused: malformed, or media as another pid
    std::uint64_t idle_closed = 0;  // lanes closed for silence (idle_timeout)
  };

  explicit fast_lane_server(fast_lane_server_config cfg = {});
  ~fast_lane_server() override;
  fast_lane_server(const fast_lane_server &) = delete;
  fast_lane_server &operator=(const fast_lane_server &) = delete;

  // PID's offer, the SDP its lane made. REPLY gets the answer once this end's candidates are
  // gathered (on a lane thread), and is not called when the offer cannot be used. Replaces any lane
  // PID had.
  void offer(const std::string &pid, const std::string &sdp,
             std::function<void(const std::string &answer)> reply);
  // Close PID's lane (a participant that left).
  void close(const std::string &pid);

  // fast_lane_hub
  bool lane_open(const std::string &pid) const override;
  std::uint64_t lane_generation() const noexcept override;
  bool send_datagram(const std::string &pid, const std::uint8_t *data, std::size_t n) override;
  void set_on_datagram(lane_kind kind, datagram_fn fn) override;

  // media_endpoint
  void fan_media(const media_frame &f, const void *exclude = nullptr) override;
  void set_media_sink(media_sink_fn sink) override;
  const char *media_protocol() const noexcept override { return "webrtc"; }
  void set_media_gate(media_gate_fn gate) override;
  void set_media_shaper(media_shape_fn shaper) override;

  // One report per lane, transport config().transport_name: its round trip (lane pings), bytes and
  // media counts.
  std::vector<state_link_report> link_reports() const;
  totals counters() const;
  const fast_lane_server_config &config() const noexcept;

private:
  struct impl;
  std::shared_ptr<impl> _impl;
};

#endif // !__EMSCRIPTEN__

// A client's lane to its relay. Natively callbacks run on lane threads; in a browser, on the page's
// event loop. Thread-safe.
class fast_lane_client {
public:
  using offer_fn = std::function<void(const std::string &offer_sdp)>;
  using datagram_fn = std::function<void(const std::uint8_t *data, std::size_t n)>;

  // SEND_OFFER carries this lane's offer to the relay over the reliable connection.
  explicit fast_lane_client(offer_fn send_offer);
  ~fast_lane_client();
  fast_lane_client(const fast_lane_client &) = delete;
  fast_lane_client &operator=(const fast_lane_client &) = delete;

  // The channel's partial reliability, for the next start(): 0 (the default) never retransmits a
  // message (maxRetransmits 0); LIFETIME > 0 lets SCTP retransmit a lost fragment until the message
  // is LIFETIME old and abandons it then -- even still queued, never sent (maxPacketLifeTime).
  void set_lifetime(std::chrono::milliseconds lifetime);
  // Make the peer connection and its channel; the offer goes out once the candidates are gathered.
  // Again after a close (or a failure) starts a new lane.
  void start();
  // The relay's answer to the latest offer.
  void on_answer(const std::string &sdp);
  void close();

  bool open() const;
  // Send one datagram: false when the lane is not open or its send buffer is past SEND_BUFFER_MAX
  // (it is dropped).
  bool send(const std::uint8_t *data, std::size_t n);
  // The datagrams that arrive, other than link probes (which the lane answers itself).
  void set_on_datagram(datagram_fn fn);
  // Called with true when the lane opens and false when it closes or fails.
  void set_on_state(std::function<void(bool open)> fn);
  // Send a ping now (call it about once a second); the round trip shows in link_report().
  void probe();
  // Transport "webrtc", remote "" (the relay): round trip, bytes and frames each way.
  state_link_report link_report() const;
  void set_send_buffer_max(std::size_t bytes);

private:
  struct impl;
  std::shared_ptr<impl> _impl;
};

} // namespace cvc

#endif // __CVC_NET_FAST_LANE_RTC_H__
