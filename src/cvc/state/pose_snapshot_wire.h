/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::pose_snapshot <-> its wire PoseSnapshot (proto/state_transport.proto). Private to the gRPC
// and WebSocket transports, which both carry the Frame envelope.

#ifndef __CVC_STATE_POSE_SNAPSHOT_WIRE_H__
#define __CVC_STATE_POSE_SNAPSHOT_WIRE_H__

#include "proto/state_transport.pb.h"

#include <cvc/state/pose_snapshot.h>

namespace cvc {
namespace pose_wire {

inline void encode(const cvc::pose_snapshot &in, ::cvc::transport::v1::PoseSnapshot *out) {
  out->set_server_tick(in.server_tick);
  out->set_seq(in.seq);
  out->set_base_seq(in.base_seq);
  for (const cvc::pose_entry &e : in.entries) {
    ::cvc::transport::v1::PoseEntry *o = out->add_entries();
    o->set_slot(e.slot);
    if (!e.pid.empty())
      o->set_pid(e.pid);
    o->set_x(e.x);
    o->set_y(e.y);
    o->set_z(e.z);
    o->set_vx(e.vx);
    o->set_vy(e.vy);
    o->set_vz(e.vz);
    o->set_qw(e.qw);
    o->set_qx(e.qx);
    o->set_qy(e.qy);
    o->set_qz(e.qz);
  }
  for (std::uint32_t g : in.gone)
    out->add_gone(g);
}

inline cvc::pose_snapshot decode(const ::cvc::transport::v1::PoseSnapshot &in) {
  cvc::pose_snapshot out;
  out.server_tick = in.server_tick();
  out.seq = in.seq();
  out.base_seq = in.base_seq();
  out.entries.reserve(static_cast<std::size_t>(in.entries_size()));
  for (const auto &o : in.entries()) {
    cvc::pose_entry e;
    e.slot = o.slot();
    e.pid = o.pid();
    e.x = o.x();
    e.y = o.y();
    e.z = o.z();
    e.vx = o.vx();
    e.vy = o.vy();
    e.vz = o.vz();
    e.qw = o.qw();
    e.qx = o.qx();
    e.qy = o.qy();
    e.qz = o.qz();
    out.entries.push_back(std::move(e));
  }
  out.gone.assign(in.gone().begin(), in.gone().end());
  return out;
}

} // namespace pose_wire
} // namespace cvc

#endif // __CVC_STATE_POSE_SNAPSHOT_WIRE_H__
