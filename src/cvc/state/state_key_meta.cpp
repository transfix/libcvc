/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_key_meta.h>

namespace cvc {

namespace {

std::string num(double v) {
  char b[32];
  std::snprintf(b, sizeof(b), "%.10g", v);
  return b;
}

} // namespace

void describe_state_key(cvc::app &ctx, const std::string &path, const state_key_meta &m) {
  try {
    cvc::state &s = cvc::state::instance(ctx)(path);
    s.comment(m.comment);
    s.property("access", m.access);
    if (m.unit)
      s.property("unit", m.unit);
    if (std::isfinite(m.min))
      s.property("min", num(m.min));
    if (std::isfinite(m.max))
      s.property("max", num(m.max));
  } catch (const std::exception &) {
    // a read-only or unreachable node keeps its metadata
  }
}

void describe_state_keys(cvc::app &ctx, const std::string &base, const state_key_meta *table,
                         std::size_t n, bool once) {
  for (std::size_t i = 0; i < n; ++i) {
    const state_key_meta &m = table[i];
    const std::string path = base.empty() ? std::string(m.key) : base + "." + m.key;
    if (once) {
      try {
        if (!cvc::state::instance(ctx)(path).comment().empty())
          continue;
      } catch (const std::exception &) {
        continue;
      }
    }
    describe_state_key(ctx, path, m);
  }
}

} // namespace cvc
