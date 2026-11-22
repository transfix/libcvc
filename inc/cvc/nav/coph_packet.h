/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH-Stagewise's packets, in C++: part of C4 of docs/roadmap/NAV-COPH-ROADMAP.md (§4.4).
//
// coph_packet is the demos' packet: cooperative.Packet's fields, in their order, then
// maze_learning.MazePacket's two (GRL-SNAM's grl_snam.coph). Its bytes() is what the Python
// counts against the byte budget and prices in the rate decision (D9): the length of
// json.dumps(asdict(packet), sort_keys=True, separators=(',', ':'), allow_nan=False). So json()
// writes the keys sorted, tuples as arrays, strings with JSON's escapes (\uXXXX for anything
// outside ASCII) and every double as Python's repr (coph_repr).
//
// coph_exchange_state holds what a learner keeps of the packets it was sent: the received packets
// (CooperativeLearner), the route evidence (TerrainLearner) and the reported geometry
// (MazeLearner). coph_exchange.h exports and assimilates; coph_channel.h carries packets between
// agents.

#ifndef CVC_NAV_COPH_PACKET_H
#define CVC_NAV_COPH_PACKET_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/nav/coph_ledger.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace nav {

// cooperative.CONVENTION: what H and the units mean. A packet with another is refused.
inline constexpr const char *coph_convention =
    "unit_mass; H=kinetic+navigation_shaping; physical_H=kinetic; SI; world_xy";

// Python's repr of a double: the shortest digits that read back as it (the closest such string
// when two have that length), written fixed when the decimal exponent e of the first digit is
// -4 <= e < 16 (with ".0" on a whole number: "1.0", "1000000000000000.0"), else as d.ddde+XX or
// d.ddde-XX with at least two exponent digits ("1e-05", "1.5e+16"). -0.0 keeps its sign; NaN and
// the infinities are "nan", "inf" and "-inf". The digits are std::to_chars's where the library
// has floating to_chars, else found with snprintf and strtod, the same digits.
std::string coph_repr(double v);

// A risk sample with its gradient: value at (x, y), gradient (gx, gy). A terrain anchor
// (terrain_warning) is the first-order estimate value + g . (p - (x, y)), supported within 0.65
// of (x, y); a packet's anchors are the sender's field there.
struct coph_anchor {
  double x = 0, y = 0, value = 0, gx = 0, gy = 0;
};

// A geometry report (MazeLearner.local_geometry_reports' values): a sensed shape, the sensor's
// centre and radius when it saw it, the time, and the report's source ('<agent>:geometry:<k>').
// In a packet's JSON it is the tuple (*shape, sensor x, sensor y, sensor radius, time, source),
// a disk's shape (x, y, r) and a slab's (x, y, r, hx, hy).
struct coph_geometry_report {
  coph_shape shape;
  coph_vec2 sensor{};
  double sensor_radius = 0;
  double time = 0;
  std::string source;
};

// The demos' packet (cooperative.Packet, then maze_learning.MazePacket's fields).
struct coph_packet {
  std::int64_t schema = 1;
  std::string sender;
  std::string recipient;
  std::string identity; // '<sender>:<timestamp, 9 decimals>:<stage id>'
  double timestamp = 0;
  double expires = 0;
  std::int64_t version = 0; // the sender's version
  std::string convention = coph_convention;
  coph_vec2 q{};     // the sender's position
  coph_vec2 p{};     // its momentum (unit mass)
  coph_vec2 v{};     // its velocity, equal to p
  double H = 0;      // kinetic energy plus the navigation potential (the shaping, not physics)
  double V_task = 0; // |q - target|^2 / 2
  coph_vec2 goal{};
  std::array<double, 4> bounds{}; // the newest stage's box: x0, x1, y0, y1
  std::vector<coph_anchor> anchors;
  // The anchors' joint (value, gx, gy) covariance, anchor-major: 3n x 3n for n anchors. Rows of
  // another length are held as given (assimilation refuses them).
  std::vector<std::vector<double>> covariance;
  std::vector<std::string> sources; // the uids of the readings the field was fit to
  std::int64_t stage = 0;
  std::vector<coph_vec2> support_points; // those readings' positions, first occurrences in order
  std::string uncertainty;
  std::vector<std::string> acknowledged_sources;
  std::int64_t geometry_schema = 1;
  std::vector<coph_geometry_report> geometry;

  // The compact, key-sorted JSON the Python counts. Throws std::domain_error on a NaN or an
  // infinity (allow_nan=False).
  std::string json() const;
  // json()'s length in bytes (D9).
  std::size_t bytes() const { return json().size(); }
};

// What a learner holds of the packets it was sent, in the Python dicts' insertion orders.
struct coph_exchange_state {
  // CooperativeLearner.received: each sender's last accepted packet. A sender that sends again
  // keeps its place; past received_cap the oldest timestamp goes (the first such).
  std::vector<std::pair<std::string, coph_packet>> received;
  // CooperativeLearner.last_sequences: each sender's last accepted (timestamp, stage).
  struct sequence {
    std::string sender;
    double timestamp = 0;
    std::int64_t stage = 0;
  };
  std::vector<sequence> last_sequences;
  // TerrainLearner.remote_terrain: route evidence by (sender, stage), at most 8.
  struct route_evidence {
    std::string sender;
    std::int64_t stage = 0;
    coph_packet packet;
  };
  std::vector<route_evidence> remote_terrain;
  // TerrainLearner.remote_cells: each received anchor by (sender, its 0.2 m cell), at most 128.
  struct cell {
    std::string sender;
    long long ix = 0, iy = 0; // round_half_even(x / 0.2), round_half_even(y / 0.2)
    double expires = 0;
    coph_anchor anchor;
    std::string identity;
    std::vector<std::string> sources;
  };
  std::vector<cell> remote_cells;
  // TerrainLearner.local_model_anchors: the learner's own terrain anchors, from its last update.
  std::vector<coph_anchor> local_model_anchors;
  // MazeLearner.local_geometry_reports: the shapes this learner sensed, one report per shape.
  std::vector<coph_geometry_report> local_geometry;
  // MazeLearner.remote_geometry_reports: the shapes packets reported, one per shape, at most 32.
  struct remote_report {
    coph_shape shape;
    double expires = 0;
    std::string packet; // the packet's identity
    std::string source;
    double observation_time = 0;
    std::string sender;
  };
  std::vector<remote_report> remote_geometry;
};

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_PACKET_H
