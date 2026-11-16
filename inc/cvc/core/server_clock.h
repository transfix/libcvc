/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The time on the server this host is joined to (an arisrv relay), as far as this host can tell:
// its clock's offset from this host's, measured by the transport's Ping/Pong probe
// (state_link_stats::note_peer_clock). One reading per cvc::app, never a process-wide one. The
// relay session writes it once a second while joined and clears it when it leaves; Ariadne's tray
// clock shows the server's time in its tooltip while it is fresh (docs/ARIADNE.md §5.22).
//
//   cvc::set_server_clock(app, offset_ms, rtt_ms, "wss://relay.example/room");
//   if (const cvc::server_clock_reading r = cvc::server_clock(app); r.known)
//     ... now + r.offset_ms is the server's time ...

#ifndef CVC_CORE_SERVER_CLOCK_H
#define CVC_CORE_SERVER_CLOCK_H

#include <chrono>
#include <cvc/core/app.h>
#include <mutex>
#include <string>
#include <utility>

namespace cvc {

struct server_clock_reading {
  bool known = false;     // set, and set again within the last kServerClockFresh
  double offset_ms = 0.0; // the server's clock minus this host's
  double rtt_ms = -1.0;   // the round trip the reading came from: its error bound
  std::string name;       // the server, as the host names it (its URL)
};

namespace detail {
struct server_clock_slot {
  static constexpr const char *app_slot = "cvc.server_clock";
  std::mutex m;
  server_clock_reading reading;
  std::chrono::steady_clock::time_point at{};
};
} // namespace detail

// A reading older than this is stale (the link stopped answering): not known.
inline constexpr std::chrono::seconds kServerClockFresh{15};

inline void set_server_clock(app &a, double offset_ms, double rtt_ms, std::string name) {
  auto &s = a.slot<detail::server_clock_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  s.reading.known = true;
  s.reading.offset_ms = offset_ms;
  s.reading.rtt_ms = rtt_ms;
  s.reading.name = std::move(name);
  s.at = std::chrono::steady_clock::now();
}

inline void clear_server_clock(app &a) {
  auto &s = a.slot<detail::server_clock_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  s.reading = server_clock_reading{};
}

inline server_clock_reading server_clock(app &a) {
  auto &s = a.slot<detail::server_clock_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  server_clock_reading r = s.reading;
  if (r.known && std::chrono::steady_clock::now() - s.at > kServerClockFresh)
    r.known = false;
  return r;
}

} // namespace cvc

#endif
