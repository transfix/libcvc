/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::agent_intent / intent_channel: what each agent announces of where it is going, and the
// channel those announcements travel on, for agents that get out of each other's way
// (sim_world::set_yield, nav/yield.h) and, later, a policy that reads them.
//
// Every tick sim_world publishes each agent's intent to the channel, then asks it, agent by agent,
// what that agent has heard of the others. Yielding acts only on what was heard, at the age it was
// heard, so a channel that drops or delays messages changes what the agents do:
//
//   - perfect_intent_channel (the default): every agent hears every other's intent of this tick,
//     at once.
//   - A host's channel: the DBG case sends each vehicle's intent to its team over the emulated
//     radios, in the same frames as the belief it shares, and delivers what each radio received
//     (with its latency and losses). cvc::nav knows nothing of radios; the host implements
//     intent_channel.
//
// Who gives way in a conflict is a second hook (yield_resolver): by default the lower priority
// yields to the higher (priority_resolver), but a host may negotiate it, say over the same radios.

#ifndef __CVC_NAV_INTENT_H__
#define __CVC_NAV_INTENT_H__

#include <cstdint>
#include <functional>
#include <vector>

namespace cvc {
namespace nav {

// The most path points an intent carries (fixed, so an intent is plain data a radio can carry).
constexpr int kIntentPathMax = 8;

// One agent's announcement, in nav's normalized frame.
struct agent_intent {
  int agent = -1; // the sender's index in its sim_world
  int team = 0;   // its team (a convoy), for a channel or resolver that cares
  double t = 0.0; // s: the sim time it was published at (sim_world::tick() x dt)
  float x = 0.0f, y = 0.0f, heading = 0.0f, speed = 0.0f; // where it is, and how it moves
  float priority = 0.0f;                                  // higher goes first
  float radius = 0.0f;                                    // its body's half-width
  // The path it means to drive next, from (x, y): the first n_path points of `path`, (x, y)
  // interleaved, about speed x the yield horizon long (sim_world builds it; see set_intent_path).
  int n_path = 0;
  float path[2 * kIntentPathMax] = {};
};

class intent_channel {
public:
  virtual ~intent_channel() = default;
  // Every agent's intent of this tick (all[i].agent == i), published at sim time t.
  virtual void publish(const agent_intent *all, int n, double t) = 0;
  // What agent i has heard of the others: the latest intent it holds from each, replacing out's
  // contents. An agent's own intent is never in it.
  virtual void heard(int i, std::vector<agent_intent> &out) const = 0;
};

// Perfect comms: every agent hears every other's intent at once.
class perfect_intent_channel : public intent_channel {
public:
  void publish(const agent_intent *all, int n, double) override { _all.assign(all, all + n); }
  void heard(int i, std::vector<agent_intent> &out) const override {
    out.clear();
    for (const agent_intent &a : _all)
      if (a.agent != i)
        out.push_back(a);
  }

private:
  std::vector<agent_intent> _all;
};

// How much `self` gives way to `other` in a conflict, in [0, 1]: it takes that share of moving
// out of the other's path, and the other is counted on for the rest.
using yield_resolver = std::function<float(const agent_intent &self, const agent_intent &other)>;

// The default: by priority, through a logistic of width `width` (equal priorities split 1/2).
yield_resolver priority_resolver(float width = 0.5f);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_INTENT_H__
