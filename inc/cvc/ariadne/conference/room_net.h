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

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_NET_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_NET_H__

#include <string>

namespace cvc {
class app;

namespace ariadne {
namespace conference {

// ----------------
// Network quality in the shared room tree
// ----------------
// What each end measures about its links (state_transport::link_reports: the Ping/Pong round trip
// and traffic both ways), published as scalar leaves beside the rest of the room so every
// participant can see everyone's connection quality:
//
//   conf.rooms.<room>.participants.<pid>.net.{rtt_ms, in_kbps, out_kbps}
//       the participant's OWN measurement of its link to the relay -- written by that client (like
//       its .intent), shown next to its name in everyone's roster
//   conf.rooms.<room>.participants.<pid>.net.relay.{rtt_ms, in_kbps, out_kbps}
//       the RELAY's measurement of its link to that participant (in = what the relay receives from
//       it) -- written by the relay, like .pose
//   conf.rooms.<room>.server.net.{in_kbps, out_kbps, links}
//       the relay's totals over all of its links
//
// Values are kilobits/second and milliseconds; negative = not measured yet.

struct net_sample {
  double rtt_ms = -1.0;
  double in_kbps = -1.0;
  double out_kbps = -1.0;
};

// A participant's own measurement (client side).
void write_participant_net(cvc::app &app, const std::string &room, const std::string &pid,
                           const net_sample &s);
bool read_participant_net(cvc::app &app, const std::string &room, const std::string &pid,
                          net_sample &out);

// The relay's measurement of its link to a participant (server side).
void write_relay_net(cvc::app &app, const std::string &room, const std::string &pid,
                     const net_sample &s);
bool read_relay_net(cvc::app &app, const std::string &room, const std::string &pid,
                    net_sample &out);

// The relay's totals.
struct server_net {
  double in_kbps = 0.0;
  double out_kbps = 0.0;
  int links = 0;
};
void write_server_net(cvc::app &app, const std::string &room, const server_net &s);
bool read_server_net(cvc::app &app, const std::string &room, server_net &out);

// Compact display text for a roster cell: "42 ms" / "--"; "850k" / "1.2M" / "--" (bits/s).
std::string format_rtt(double rtt_ms);
std::string format_rate(double kbps);

// ----------------
// Voice activity in the shared room tree
// ----------------
//   conf.rooms.<room>.participants.<pid>.voice.talking   "1" / "0"
//       whether the participant is talking, as its OWN client measures its mic -- written by that
//       client (like its .net), so a peer whose audio the relay does not send it (out of range in
//       the room) can still light that participant's name. Absent = not talking.
//
// The write is changed-only and journals ONE mutation per change: a repeated value writes nothing,
// and a changed one sets the value without the touch() that would first re-send the old one.
// FORCE is for a writer that tracks its own flips (arigl): the tree may already hold TALKING
// without the relay having it -- a periodic re-broadcast can echo a stale copy of the client's own
// leaf back -- so a forced write of an unchanged value re-sends it (one touch) instead of nothing.
void write_participant_voice(cvc::app &app, const std::string &room, const std::string &pid,
                             bool talking, bool force = false);
// False (TALKING untouched) when the leaf was never written.
bool read_participant_voice(cvc::app &app, const std::string &room, const std::string &pid,
                            bool &talking);

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_NET_H__
