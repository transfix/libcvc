/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// CoPH packets' transport (C4 of docs/roadmap/NAV-COPH-ROADMAP.md, §4.4); see coph_channel.h.

#include <algorithm>
#include <cmath>
#include <cvc/nav/coph_channel.h>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace nav {

coph_ideal_channel::coph_ideal_channel(coph_ideal_channel_config cfg)
    : cfg_(std::move(cfg)), rng_(cfg_.seed) {
  if (!(cfg_.delay_s >= 0) || !(cfg_.jitter_s >= 0) || !(cfg_.drop >= 0 && cfg_.drop <= 1) ||
      !(cfg_.duplicate >= 0 && cfg_.duplicate <= 1) || (cfg_.range_m && !(*cfg_.range_m >= 0)))
    throw std::invalid_argument("coph_ideal_channel: a delay, probability or range out of range");
}

double coph_ideal_channel::uniform() {
  // 27 + 26 bits, as the classic genrand_res53: the same on every standard library.
  const std::uint32_t a = static_cast<std::uint32_t>(rng_()) >> 5;
  const std::uint32_t b = static_cast<std::uint32_t>(rng_()) >> 6;
  return (a * 67108864.0 + b) * (1.0 / 9007199254740992.0);
}

void coph_ideal_channel::set_position(int agent, const coph_vec2 &xy) {
  if (agent < 0)
    throw std::invalid_argument("coph_ideal_channel: a negative agent");
  if (static_cast<std::size_t>(agent) >= positions_.size())
    positions_.resize(static_cast<std::size_t>(agent) + 1);
  positions_[static_cast<std::size_t>(agent)] = xy;
}

void coph_ideal_channel::send(int from, int to, std::shared_ptr<const coph_packet> packet,
                              std::size_t bytes, double at_s) {
  ++counts_.sends;
  bool in_range = true;
  if (cfg_.range_m) {
    const auto at = [&](int a) -> std::optional<coph_vec2> {
      if (a < 0 || static_cast<std::size_t>(a) >= positions_.size())
        return std::nullopt;
      return positions_[static_cast<std::size_t>(a)];
    };
    const std::optional<coph_vec2> p = at(from), q = at(to);
    in_range = p && q && std::hypot((*p)[0] - (*q)[0], (*p)[1] - (*q)[1]) <= *cfg_.range_m;
  }
  // The draws for each copy are taken whether or not it is lost, so one send's outcome does not
  // shift the next's.
  const double dup = uniform();
  const int copies = dup < cfg_.duplicate ? 2 : 1;
  for (int c = 0; c < copies; ++c) {
    ++counts_.copies;
    const double lost = uniform();
    const double extra = uniform();
    if (!in_range) {
      ++counts_.out_of_range;
      continue;
    }
    if (lost < cfg_.drop) {
      ++counts_.dropped;
      continue;
    }
    coph_delivery d{from, to, packet, bytes, at_s, at_s + cfg_.delay_s + cfg_.jitter_s * extra};
    pending_.push_back({std::move(d), next_++});
  }
}

std::vector<coph_delivery> coph_ideal_channel::poll(int agent, double now_s) {
  std::vector<pending> due, rest;
  for (pending &p : pending_)
    (p.d.to == agent && p.d.at_s <= now_s ? due : rest).push_back(std::move(p));
  pending_ = std::move(rest);
  std::sort(due.begin(), due.end(), [](const pending &a, const pending &b) {
    return a.d.at_s != b.d.at_s ? a.d.at_s < b.d.at_s : a.order < b.order;
  });
  std::vector<coph_delivery> out;
  for (pending &p : due)
    out.push_back(std::move(p.d));
  counts_.delivered += static_cast<long long>(out.size());
  return out;
}

} // namespace nav
} // namespace cvc
