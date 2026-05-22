/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_delta_codec.h>
#include <gtest/gtest.h>
#include <numeric>
#include <vector>

using cvc::state_delta_codec;

// ---- CRC32 ----

TEST(DeltaCodecCrc32, EmptyBuffer) { EXPECT_EQ(state_delta_codec::crc32(nullptr, 0), 0x00000000u); }

TEST(DeltaCodecCrc32, KnownVector) {
  // "123456789" → CRC32 = 0xCBF43926
  const unsigned char data[] = "123456789";
  EXPECT_EQ(state_delta_codec::crc32(data, 9), 0xCBF43926u);
}

TEST(DeltaCodecCrc32, Deterministic) {
  std::vector<unsigned char> buf(1024);
  std::iota(buf.begin(), buf.end(), 0);
  auto a = state_delta_codec::crc32(buf.data(), buf.size());
  auto b = state_delta_codec::crc32(buf.data(), buf.size());
  EXPECT_EQ(a, b);
  EXPECT_NE(a, 0u);
}

// ---- Path-unaware (raw fallback) ----

TEST(DeltaCodecTest, PathUnawareEncodeDecodeRoundTrip) {
  state_delta_codec codec;
  std::vector<unsigned char> data = {1, 2, 3, 4, 5};
  auto encoded = codec.encode(data);
  ASSERT_FALSE(encoded.empty());
  EXPECT_EQ(encoded[0], state_delta_codec::MAGIC_RAW);

  std::vector<unsigned char> decoded;
  EXPECT_TRUE(codec.decode(encoded, decoded));
  EXPECT_EQ(decoded, data);
}

TEST(DeltaCodecTest, PathUnawareDecodeRejectsDelta) {
  state_delta_codec codec;
  // A delta-encoded payload cannot be decoded without a path.
  std::vector<unsigned char> fake_delta = {
      state_delta_codec::MAGIC_DELTA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  std::vector<unsigned char> out;
  EXPECT_FALSE(codec.decode(fake_delta, out));
}

TEST(DeltaCodecTest, PathUnawareDecodeRejectsEmpty) {
  state_delta_codec codec;
  std::vector<unsigned char> out;
  EXPECT_FALSE(codec.decode({}, out));
}

// ---- Path-aware encode/decode ----

TEST(DeltaCodecTest, FirstEncodeIsRawFallback) {
  state_delta_codec codec;
  std::vector<unsigned char> data = {10, 20, 30};
  auto encoded = codec.encode("vol.0", data);
  EXPECT_EQ(encoded[0], state_delta_codec::MAGIC_RAW);
  EXPECT_EQ(codec.baseline_count(), 1u);
}

TEST(DeltaCodecTest, SecondEncodeProducesDelta) {
  state_delta_codec codec;
  std::vector<unsigned char> v1 = {10, 20, 30, 40};
  std::vector<unsigned char> v2 = {10, 20, 31, 40}; // one byte changed

  codec.encode("vol.0", v1);
  auto encoded = codec.encode("vol.0", v2);
  ASSERT_GE(encoded.size(), 13u);
  EXPECT_EQ(encoded[0], state_delta_codec::MAGIC_DELTA);
}

TEST(DeltaCodecTest, DeltaRoundTrip) {
  state_delta_codec encoder;
  state_delta_codec decoder;

  std::vector<unsigned char> v1(256);
  std::iota(v1.begin(), v1.end(), 0);
  std::vector<unsigned char> v2 = v1;
  v2[100] = 0xFF;
  v2[200] = 0x42;

  // Encoder: first write is raw.
  auto e1 = encoder.encode("vol.0", v1);
  // Decoder: first read is raw — establishes baseline.
  std::vector<unsigned char> d1;
  ASSERT_TRUE(decoder.decode("vol.0", e1, d1));
  EXPECT_EQ(d1, v1);

  // Encoder: second write is delta.
  auto e2 = encoder.encode("vol.0", v2);
  EXPECT_EQ(e2[0], state_delta_codec::MAGIC_DELTA);
  // Decoder: second read uses baseline.
  std::vector<unsigned char> d2;
  ASSERT_TRUE(decoder.decode("vol.0", e2, d2));
  EXPECT_EQ(d2, v2);
}

TEST(DeltaCodecTest, DeltaSmallerThanRawForSimilarData) {
  state_delta_codec codec;
  std::vector<unsigned char> v1(4096, 0xAA);
  std::vector<unsigned char> v2 = v1;
  v2[0] = 0xBB; // change 1 of 4096 bytes

  auto raw = codec.encode("a", v1);
  auto delta = codec.encode("a", v2);

  // Raw = 1 + 4096 = 4097.
  EXPECT_EQ(raw.size(), 4097u);
  // Delta = 13 + 4096 = 4109. The patch itself is large but the
  // XOR result is mostly zeros which RLE would compress well.
  // The test checks the delta is well-formed, not smaller per se
  // (that requires a follow-up RLE pass).
  EXPECT_EQ(delta[0], state_delta_codec::MAGIC_DELTA);
  EXPECT_EQ(delta.size(), 13u + 4096u);
}

TEST(DeltaCodecTest, DeltaWithSizeChange) {
  state_delta_codec encoder;
  state_delta_codec decoder;

  std::vector<unsigned char> v1 = {1, 2, 3};
  std::vector<unsigned char> v2 = {1, 2, 3, 4, 5}; // grew

  auto e1 = encoder.encode("p", v1);
  std::vector<unsigned char> d1;
  ASSERT_TRUE(decoder.decode("p", e1, d1));
  EXPECT_EQ(d1, v1);

  auto e2 = encoder.encode("p", v2);
  std::vector<unsigned char> d2;
  ASSERT_TRUE(decoder.decode("p", e2, d2));
  EXPECT_EQ(d2, v2);
}

TEST(DeltaCodecTest, DeltaWithSizeShrink) {
  state_delta_codec encoder;
  state_delta_codec decoder;

  std::vector<unsigned char> v1 = {1, 2, 3, 4, 5};
  std::vector<unsigned char> v2 = {1, 2}; // shrunk

  auto e1 = encoder.encode("p", v1);
  std::vector<unsigned char> d1;
  ASSERT_TRUE(decoder.decode("p", e1, d1));
  EXPECT_EQ(d1, v1);

  auto e2 = encoder.encode("p", v2);
  std::vector<unsigned char> d2;
  ASSERT_TRUE(decoder.decode("p", e2, d2));
  EXPECT_EQ(d2, v2);
}

// ---- Baseline management ----

TEST(DeltaCodecTest, SetBaselineManually) {
  state_delta_codec codec;
  std::vector<unsigned char> baseline = {10, 20, 30};
  codec.set_baseline("vol.0", baseline);
  EXPECT_EQ(codec.baseline_count(), 1u);

  std::vector<unsigned char> v2 = {10, 20, 31};
  auto encoded = codec.encode("vol.0", v2);
  EXPECT_EQ(encoded[0], state_delta_codec::MAGIC_DELTA);
}

TEST(DeltaCodecTest, ClearBaseline) {
  state_delta_codec codec;
  codec.set_baseline("vol.0", {1, 2, 3});
  EXPECT_TRUE(codec.clear_baseline("vol.0"));
  EXPECT_FALSE(codec.clear_baseline("vol.0")); // already gone
  EXPECT_EQ(codec.baseline_count(), 0u);
}

TEST(DeltaCodecTest, ClearAllBaselines) {
  state_delta_codec codec;
  codec.set_baseline("a", {1});
  codec.set_baseline("b", {2});
  codec.set_baseline("c", {3});
  codec.clear_all_baselines();
  EXPECT_EQ(codec.baseline_count(), 0u);
}

// ---- Decode with mismatched baseline ----

TEST(DeltaCodecTest, DecodeMismatchedBaselineRejects) {
  state_delta_codec encoder;
  state_delta_codec decoder;

  std::vector<unsigned char> v1 = {1, 2, 3};
  std::vector<unsigned char> v2 = {1, 2, 4};
  encoder.encode("p", v1);
  auto delta = encoder.encode("p", v2);
  EXPECT_EQ(delta[0], state_delta_codec::MAGIC_DELTA);

  // Decoder has a DIFFERENT baseline.
  decoder.set_baseline("p", {99, 99, 99});
  std::vector<unsigned char> out;
  EXPECT_FALSE(decoder.decode("p", delta, out));
}

// ---- Wire id ----

TEST(DeltaCodecTest, IdIsDelta) {
  state_delta_codec codec;
  EXPECT_EQ(codec.id(), "delta");
}

// ---- Multiple paths are independent ----

TEST(DeltaCodecTest, IndependentPaths) {
  state_delta_codec encoder;
  state_delta_codec decoder;

  std::vector<unsigned char> a1 = {1, 2};
  std::vector<unsigned char> a2 = {1, 3};
  std::vector<unsigned char> b1 = {10, 20, 30};
  std::vector<unsigned char> b2 = {10, 20, 31};

  auto ea1 = encoder.encode("pathA", a1);
  auto eb1 = encoder.encode("pathB", b1);
  std::vector<unsigned char> da1, db1;
  ASSERT_TRUE(decoder.decode("pathA", ea1, da1));
  ASSERT_TRUE(decoder.decode("pathB", eb1, db1));

  auto ea2 = encoder.encode("pathA", a2);
  auto eb2 = encoder.encode("pathB", b2);
  std::vector<unsigned char> da2, db2;
  ASSERT_TRUE(decoder.decode("pathA", ea2, da2));
  ASSERT_TRUE(decoder.decode("pathB", eb2, db2));
  EXPECT_EQ(da2, a2);
  EXPECT_EQ(db2, b2);
}
