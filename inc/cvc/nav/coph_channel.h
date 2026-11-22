/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// How CoPH packets travel between agents: part of C4 of docs/roadmap/NAV-COPH-ROADMAP.md (§4.4).
//
// coph_channel is the transport the fleet driver (C5) sends through: send(from, to, packet,
// bytes, at_s) hands a packet over at time at_s, and poll(agent, now_s) returns what has reached
// the agent by now_s. bytes is what the radio carries, the packet's coph_packet::bytes() unless
// the caller prices it otherwise. libcvc ships coph_ideal_channel; cvcdbg adapts its radio
// network (Network::send(agent, bits, tag, at_s) -> MessageReception) on its own side, so libcvc
// gains no dependency on cvcdbg.

#ifndef CVC_NAV_COPH_CHANNEL_H
#define CVC_NAV_COPH_CHANNEL_H

#include <cstddef>
#include <cstdint>
#include <cvc/nav/coph_packet.h>
#include <memory>
#include <optional>
#include <random>
#include <vector>

namespace cvc {
namespace nav {

// A packet that reached an agent.
struct coph_delivery {
  int from = 0;
  int to = 0;
  std::shared_ptr<const coph_packet> packet;
  std::size_t bytes = 0;
  double sent_s = 0;
  double at_s = 0; // when it arrived
};

class coph_channel {
public:
  virtual ~coph_channel() = default;
  // Hand a packet from agent `from` to agent `to` at time at_s.
  virtual void send(int from, int to, std::shared_ptr<const coph_packet> packet, std::size_t bytes,
                    double at_s) = 0;
  // Everything that reached `agent` by now_s and was not polled before, by arrival time (then
  // in the order it was sent). Each delivery is returned once.
  virtual std::vector<coph_delivery> poll(int agent, double now_s) = 0;
};

// coph_ideal_channel's options. Every draw comes from one std::mt19937 seeded with seed, in send
// order, and is the same on every standard library, so a seed is one run everywhere.
struct coph_ideal_channel_config {
  std::uint32_t seed = 0;
  double delay_s = 0.0;   // every copy's fixed latency
  double jitter_s = 0.0;  // plus a uniform extra in [0, jitter_s)
  double drop = 0.0;      // the probability that a copy is lost
  double duplicate = 0.0; // the probability that a second copy follows, with its own delay
  // Beyond this distance between the agents' positions at the send, nothing arrives. Unset: no
  // range. Positions come from set_position.
  std::optional<double> range_m;
};

// A channel with seeded delay, drops, duplicates and an optional range: not a radio model.
class coph_ideal_channel : public coph_channel {
public:
  explicit coph_ideal_channel(coph_ideal_channel_config cfg = {});

  void send(int from, int to, std::shared_ptr<const coph_packet> packet, std::size_t bytes,
            double at_s) override;
  std::vector<coph_delivery> poll(int agent, double now_s) override;

  // An agent's position, for the range. An agent without one is out of range of everyone.
  void set_position(int agent, const coph_vec2 &xy);

  // What it did: copies sent (each send's first copy and any duplicate), lost (dropped or out
  // of range) and delivered so far.
  struct counts {
    long long sends = 0, copies = 0, dropped = 0, out_of_range = 0, delivered = 0;
  };
  const counts &statistics() const { return counts_; }

private:
  double uniform(); // [0, 1), 53 bits from two draws
  coph_ideal_channel_config cfg_;
  std::mt19937 rng_;
  std::vector<std::optional<coph_vec2>> positions_;
  struct pending {
    coph_delivery d;
    long long order = 0;
  };
  std::vector<pending> pending_;
  long long next_ = 0;
  counts counts_;
};

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_CHANNEL_H
