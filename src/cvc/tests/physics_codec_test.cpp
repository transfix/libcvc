/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Tests for cvc::physics::physics_codec -- the body_state / pose wire codec that
// lets a holon replicate a participant's avatar pose as a node's data(). The
// properties worth pinning: an exact (bit-for-bit, including NaN / -0.0 /
// denormals) round-trip, a truncated payload that throws rather than reading out
// of bounds, a byte-identical little-endian wire, and -- guarding the #767 S1
// class of bug -- that the codec-registry key equals dataTypeName<T>() so the
// emit path's lookup actually finds the codec.

#include <boost/any.hpp>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/physics/body.h>
#include <cvc/physics/physics_codec.h>
#include <cvc/physics/pose_tracker.h>
#include <cvc/physics/types.h>
#include <cvc/state/state_codec_registry.h>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

using cvc::physics::body_state;
using cvc::physics::pose;
using cvc::physics::quat;
using cvc::physics::vec3;

namespace {
// The type names come from the public inline accessors
// cvc::physics::body_state_type_name() / pose_type_name() -- being inline (not an
// exported data symbol) they link across a shared-library boundary on every
// platform, and there is no static global to carry them.

// Bit pattern of a double, so NaN / -0.0 / denormals compare exactly.
std::uint64_t bits(double d) {
  std::uint64_t u = 0;
  std::memcpy(&u, &d, sizeof(u));
  return u;
}
void expect_vec3_bit_eq(const vec3 &a, const vec3 &b) {
  EXPECT_EQ(bits(a.x), bits(b.x));
  EXPECT_EQ(bits(a.y), bits(b.y));
  EXPECT_EQ(bits(a.z), bits(b.z));
}
void expect_quat_bit_eq(const quat &a, const quat &b) {
  EXPECT_EQ(bits(a.w), bits(b.w));
  EXPECT_EQ(bits(a.x), bits(b.x));
  EXPECT_EQ(bits(a.y), bits(b.y));
  EXPECT_EQ(bits(a.z), bits(b.z));
}

// A body_state with every field distinct and several nasty doubles.
body_state sample_body_state() {
  body_state s;
  s.position = {1.5, -2.25, 3.0e300};
  s.rotation = {0.5, -0.5, 0.5, -0.5};
  s.linear_velocity = {std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity(), -0.0};
  s.angular_velocity = {std::numeric_limits<double>::denorm_min(), -1e-9, 42.0};
  s.active = true;
  return s;
}
} // namespace

// ── a shard-registry round-trip (the real usage) ────────────────────────────

class PhysicsCodecTest : public ::testing::Test {
protected:
  cvc::app ctx;
  cvc::state_codec_registry reg;
  void SetUp() override {
    cvc::physics::register_physics_data_types(ctx);
    cvc::physics::register_physics_codecs(reg);
  }
};

TEST_F(PhysicsCodecTest, BodyStateRoundTripsBitForBit) {
  const body_state in = sample_body_state();
  const std::vector<unsigned char> wire =
      reg.encode(cvc::physics::body_state_type_name(), boost::any(in));
  const boost::any back = reg.decode(cvc::physics::body_state_type_name(), wire);
  ASSERT_FALSE(back.empty());
  const body_state out = boost::any_cast<body_state>(back);
  expect_vec3_bit_eq(in.position, out.position);
  expect_quat_bit_eq(in.rotation, out.rotation);
  expect_vec3_bit_eq(in.linear_velocity, out.linear_velocity); // NaN / Inf / -0.0 survive
  expect_vec3_bit_eq(in.angular_velocity, out.angular_velocity);
  EXPECT_EQ(in.active, out.active);
}

TEST_F(PhysicsCodecTest, PoseRoundTripsBitForBit) {
  pose in;
  in.position = {-7.0, 0.0, 12345.678};
  in.rotation = {1.0, 0.0, 0.0, 0.0};
  const std::vector<unsigned char> wire =
      reg.encode(cvc::physics::pose_type_name(), boost::any(in));
  const pose out = boost::any_cast<pose>(reg.decode(cvc::physics::pose_type_name(), wire));
  expect_vec3_bit_eq(in.position, out.position);
  expect_quat_bit_eq(in.rotation, out.rotation);
}

TEST_F(PhysicsCodecTest, RegistryHasBothCodecs) {
  EXPECT_TRUE(reg.has(cvc::physics::body_state_type_name()));
  EXPECT_TRUE(reg.has(cvc::physics::pose_type_name()));
}

// The #767 S1 class of bug: if the registry key differed from what dataTypeName()
// returns, the emit path's _codecs->encode(dataTypeName(), ...) would miss and the
// object would silently fail to replicate. Pin that they are identical.
TEST_F(PhysicsCodecTest, RegistryKeyEqualsDataTypeName) {
  EXPECT_EQ(ctx.dataTypeName<body_state>(), std::string(cvc::physics::body_state_type_name()));
  EXPECT_EQ(ctx.dataTypeName<pose>(), std::string(cvc::physics::pose_type_name()));
}

// ── wire shape: fixed-width little-endian, field-by-field ───────────────────

TEST_F(PhysicsCodecTest, WireIsFixedWidthLittleEndian) {
  body_state s; // all zero, then set position.x = 1.0 and active = true
  s.position.x = 1.0;
  s.active = true;
  const std::vector<unsigned char> wire =
      reg.encode(cvc::physics::body_state_type_name(), boost::any(s));
  // 13 doubles (pos.xyz, rot.wxyz, linvel.xyz, angvel.xyz) * 8 + 1 active byte.
  ASSERT_EQ(wire.size(), 13u * 8u + 1u);
  // position.x == 1.0 -> IEEE-754 0x3FF0000000000000, little-endian first field.
  const unsigned char one_le[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F};
  for (int i = 0; i < 8; ++i)
    EXPECT_EQ(wire[i], one_le[i]) << "byte " << i;
  // the very next field (position.y == 0.0) is eight zero bytes.
  for (int i = 8; i < 16; ++i)
    EXPECT_EQ(wire[i], 0x00) << "byte " << i;
  EXPECT_EQ(wire.back(), 0x01); // active = true, the trailing byte
}

TEST_F(PhysicsCodecTest, PoseWireIsSevenDoubles) {
  const std::vector<unsigned char> wire =
      reg.encode(cvc::physics::pose_type_name(), boost::any(pose{}));
  EXPECT_EQ(wire.size(), 7u * 8u);
}

// ── truncation is rejected, not read out of bounds ──────────────────────────

TEST_F(PhysicsCodecTest, TruncatedBodyStateThrows) {
  const std::vector<unsigned char> full =
      reg.encode(cvc::physics::body_state_type_name(), boost::any(sample_body_state()));
  for (std::size_t n : {std::size_t{0}, std::size_t{1}, full.size() - 1, full.size() - 8}) {
    const std::vector<unsigned char> short_wire(full.begin(), full.begin() + n);
    EXPECT_THROW(reg.decode(cvc::physics::body_state_type_name(), short_wire), std::exception)
        << "n=" << n;
  }
}

TEST_F(PhysicsCodecTest, TruncatedPoseThrows) {
  const std::vector<unsigned char> full =
      reg.encode(cvc::physics::pose_type_name(), boost::any(pose{vec3{1, 2, 3}, quat{}}));
  const std::vector<unsigned char> short_wire(full.begin(), full.end() - 1);
  EXPECT_THROW(reg.decode(cvc::physics::pose_type_name(), short_wire), std::exception);
}
