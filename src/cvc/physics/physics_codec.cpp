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

#include <boost/any.hpp>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/physics/body.h>
#include <cvc/physics/physics_codec.h>
#include <cvc/physics/pose_tracker.h>
#include <cvc/physics/types.h>
#include <cvc/state/state_codec_registry.h>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace physics {

const char *const kBodyStateTypeName = "cvc::physics::body_state";
const char *const kPoseTypeName = "cvc::physics::pose";

namespace {

// Little-endian, fixed-width framing (mirrors cvc::ariadne::stream::stream_codec)
// so the content is byte-identical across the little-endian fleet.
void put_u8(std::vector<unsigned char> &o, std::uint8_t v) { o.push_back(v); }
void put_u64(std::vector<unsigned char> &o, std::uint64_t v) {
  for (int i = 0; i < 8; ++i)
    o.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}
// A double as its IEEE-754 binary64 bit pattern, 8 LE bytes. memcpy (not a
// reinterpret or std::bit_cast) keeps it well-defined and C++17-clean; encode and
// decode are exact inverses on any single host and byte-identical across LE hosts,
// matching the built-in "cvc.f64.v1" double codec.
void put_f64(std::vector<unsigned char> &o, double d) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &d, sizeof(bits));
  put_u64(o, bits);
}

struct reader {
  const unsigned char *p;
  const unsigned char *e;
  bool ok = true;
  explicit reader(const std::vector<unsigned char> &b) : p(b.data()), e(b.data() + b.size()) {}
  std::uint8_t u8() {
    if (p + 1 > e) {
      ok = false;
      return 0;
    }
    return *p++;
  }
  std::uint64_t u64() {
    if (p + 8 > e) {
      ok = false;
      return 0;
    }
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
      v |= static_cast<std::uint64_t>(p[i]) << (8 * i);
    p += 8;
    return v;
  }
  double f64() {
    std::uint64_t bits = u64();
    double d = 0.0;
    std::memcpy(&d, &bits, sizeof(d));
    return d;
  }
};

void put_vec3(std::vector<unsigned char> &o, const vec3 &v) {
  put_f64(o, v.x);
  put_f64(o, v.y);
  put_f64(o, v.z);
}
void put_quat(std::vector<unsigned char> &o, const quat &q) {
  put_f64(o, q.w);
  put_f64(o, q.x);
  put_f64(o, q.y);
  put_f64(o, q.z);
}
vec3 read_vec3(reader &r) {
  vec3 v;
  v.x = r.f64();
  v.y = r.f64();
  v.z = r.f64();
  return v;
}
quat read_quat(reader &r) {
  quat q;
  q.w = r.f64();
  q.x = r.f64();
  q.y = r.f64();
  q.z = r.f64();
  return q;
}

std::vector<unsigned char> encode_body_state(const boost::any &v) {
  const body_state s = boost::any_cast<body_state>(v);
  std::vector<unsigned char> o;
  put_vec3(o, s.position);
  put_quat(o, s.rotation);
  put_vec3(o, s.linear_velocity);
  put_vec3(o, s.angular_velocity);
  put_u8(o, s.active ? 1u : 0u);
  return o;
}

boost::any decode_body_state(const std::vector<unsigned char> &b) {
  reader r(b);
  body_state s;
  s.position = read_vec3(r);
  s.rotation = read_quat(r);
  s.linear_velocity = read_vec3(r);
  s.angular_velocity = read_vec3(r);
  s.active = r.u8() != 0;
  if (!r.ok)
    throw std::runtime_error("cvc::physics::body_state decode: truncated payload");
  return boost::any(s);
}

std::vector<unsigned char> encode_pose(const boost::any &v) {
  const pose p = boost::any_cast<pose>(v);
  std::vector<unsigned char> o;
  put_vec3(o, p.position);
  put_quat(o, p.rotation);
  return o;
}

boost::any decode_pose(const std::vector<unsigned char> &b) {
  reader r(b);
  pose p;
  p.position = read_vec3(r);
  p.rotation = read_quat(r);
  if (!r.ok)
    throw std::runtime_error("cvc::physics::pose decode: truncated payload");
  return boost::any(p);
}

} // namespace

void register_physics_data_types(app &ctx) {
  ctx.registerDataType<body_state>(kBodyStateTypeName);
  ctx.registerDataType<pose>(kPoseTypeName);
}

void register_physics_codecs(state_codec_registry &registry) {
  registry.register_codec(kBodyStateTypeName, encode_body_state, decode_body_state,
                          "cvc.physics.body_state.v1");
  registry.register_codec(kPoseTypeName, encode_pose, decode_pose, "cvc.physics.pose.v1");
}

} // namespace physics
} // namespace cvc
