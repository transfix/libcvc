/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <cvc/core/state_compression_registry.h>
#include <gtest/gtest.h>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::vector<unsigned char> random_bytes(std::size_t n, std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> d(0, 255);
  std::vector<unsigned char> v(n);
  for (auto &b : v)
    b = static_cast<unsigned char>(d(rng));
  return v;
}

} // namespace

TEST(StateCompressionRegistry, BuiltInCodecsRegistered) {
  cvc::state_compression_registry r;
  EXPECT_TRUE(r.has("raw"));
  EXPECT_TRUE(r.has("rle"));
  EXPECT_TRUE(r.has("zstd"));
  EXPECT_GE(r.size(), 3u);
}

TEST(StateCompressionRegistry, RawIsIdentity) {
  cvc::state_raw_compression_codec c;
  std::vector<unsigned char> in = {1, 2, 3, 4, 0, 0, 7};
  EXPECT_EQ(c.id(), "raw");
  EXPECT_EQ(c.encode(in), in);
  std::vector<unsigned char> out;
  ASSERT_TRUE(c.decode(in, out));
  EXPECT_EQ(out, in);
}

TEST(StateCompressionRegistry, RleEncodesAndDecodesRuns) {
  cvc::state_rle_compression_codec c;
  std::vector<unsigned char> in(300, 0xAB); // long run
  in.push_back(0x01);
  in.push_back(0x02);
  in.push_back(0x02);

  auto enc = c.encode(in);
  // 300 of 0xAB: runs of 255+45 = 2 pairs.
  // 0x01 single + 0x02x2 = 2 more pairs. Total 4 pairs = 8 bytes.
  EXPECT_EQ(enc.size(), 8u);
  EXPECT_LT(enc.size(), in.size());

  std::vector<unsigned char> dec;
  ASSERT_TRUE(c.decode(enc, dec));
  EXPECT_EQ(dec, in);
}

TEST(StateCompressionRegistry, RleRejectsOddLengthInput) {
  cvc::state_rle_compression_codec c;
  std::vector<unsigned char> bad = {1, 2, 3};
  std::vector<unsigned char> out;
  EXPECT_FALSE(c.decode(bad, out));
  EXPECT_TRUE(out.empty());
}

TEST(StateCompressionRegistry, RleRejectsZeroRunCount) {
  cvc::state_rle_compression_codec c;
  std::vector<unsigned char> bad = {0, 5}; // run=0 is invalid
  std::vector<unsigned char> out;
  EXPECT_FALSE(c.decode(bad, out));
  EXPECT_TRUE(out.empty());
}

TEST(StateCompressionRegistry, RleRoundTripsRandomData) {
  cvc::state_rle_compression_codec c;
  for (std::uint32_t seed = 0; seed < 4; ++seed) {
    auto in = random_bytes(257 + seed * 13, seed + 1);
    auto enc = c.encode(in);
    std::vector<unsigned char> dec;
    ASSERT_TRUE(c.decode(enc, dec)) << "seed=" << seed;
    EXPECT_EQ(dec, in) << "seed=" << seed;
  }
}

TEST(StateCompressionRegistry, RleRoundTripsEmpty) {
  cvc::state_rle_compression_codec c;
  std::vector<unsigned char> empty;
  auto enc = c.encode(empty);
  EXPECT_TRUE(enc.empty());
  std::vector<unsigned char> dec;
  ASSERT_TRUE(c.decode(enc, dec));
  EXPECT_TRUE(dec.empty());
}

TEST(StateCompressionRegistry, RegisterCustomCodec) {
  struct xor_codec : public cvc::state_compression_codec {
    std::string id() const override { return "xor55"; }
    std::vector<unsigned char> encode(const std::vector<unsigned char> &in) const override {
      auto out = in;
      for (auto &b : out)
        b ^= 0x55;
      return out;
    }
    bool decode(const std::vector<unsigned char> &in,
                std::vector<unsigned char> &out) const override {
      out = in;
      for (auto &b : out)
        b ^= 0x55;
      return true;
    }
  };

  cvc::state_compression_registry r;
  EXPECT_FALSE(r.has("xor55"));
  r.register_codec(std::make_shared<xor_codec>());
  EXPECT_TRUE(r.has("xor55"));

  std::vector<unsigned char> in = {0x01, 0x02, 0x03};
  auto enc = r.encode("xor55", in);
  EXPECT_EQ(enc[0], 0x54);
  std::vector<unsigned char> dec;
  ASSERT_TRUE(r.decode("xor55", enc, dec));
  EXPECT_EQ(dec, in);
}

TEST(StateCompressionRegistry, RegisterIsIdempotentReplace) {
  cvc::state_compression_registry r;
  auto raw_a = r.get("raw");
  r.register_codec(std::make_shared<cvc::state_raw_compression_codec>());
  auto raw_b = r.get("raw");
  EXPECT_NE(raw_a.get(), raw_b.get()); // replaced, not duplicated
  EXPECT_EQ(r.size(), 3u);             // still raw + rle + zstd
}

TEST(StateCompressionRegistry, RegisterNullIsNoOp) {
  cvc::state_compression_registry r;
  const auto before = r.size();
  r.register_codec(nullptr);
  EXPECT_EQ(r.size(), before);
}

TEST(StateCompressionRegistry, EncodeEmptyIdIsIdentity) {
  cvc::state_compression_registry r;
  std::vector<unsigned char> in = {1, 2, 3};
  EXPECT_EQ(r.encode("", in), in);
  std::vector<unsigned char> out;
  ASSERT_TRUE(r.decode("", in, out));
  EXPECT_EQ(out, in);
}

TEST(StateCompressionRegistry, StrictUnknownIdThrows) {
  cvc::state_compression_registry r;
  std::vector<unsigned char> in = {1, 2, 3};
  EXPECT_THROW(r.encode("nope", in), std::runtime_error);
  std::vector<unsigned char> out;
  EXPECT_THROW(r.decode("nope", in, out), std::runtime_error);
}

TEST(StateCompressionRegistry, NonStrictUnknownIdFallsBackToIdentity) {
  cvc::state_compression_registry r;
  std::vector<unsigned char> in = {1, 2, 3};
  EXPECT_EQ(r.encode("nope", in, false), in);
  std::vector<unsigned char> out;
  ASSERT_TRUE(r.decode("nope", in, out, false));
  EXPECT_EQ(out, in);
}

TEST(StateCompressionRegistry, IdsListReflectsRegistered) {
  cvc::state_compression_registry r;
  auto ids = r.ids();
  std::sort(ids.begin(), ids.end());
  ASSERT_EQ(ids.size(), 3u);
  EXPECT_EQ(ids[0], "raw");
  EXPECT_EQ(ids[1], "rle");
  EXPECT_EQ(ids[2], "zstd");
}

TEST(StateCompressionRegistry, SharedSingletonHasBuiltins) {
  auto &r = cvc::state_compression_registry::shared();
  EXPECT_TRUE(r.has("raw"));
  EXPECT_TRUE(r.has("rle"));
  // Same instance returned across calls.
  auto &r2 = cvc::state_compression_registry::shared();
  EXPECT_EQ(&r, &r2);
}

TEST(StateCompressionRegistry, ConcurrentEncodeDecodeIsSafe) {
  cvc::state_compression_registry r;
  std::atomic<int> ok{0};
  std::vector<std::thread> ts;
  for (int t = 0; t < 4; ++t) {
    ts.emplace_back([&, t] {
      for (int i = 0; i < 200; ++i) {
        auto in = random_bytes(64, static_cast<std::uint32_t>(t * 1000 + i));
        auto enc = r.encode("rle", in);
        std::vector<unsigned char> dec;
        if (r.decode("rle", enc, dec) && dec == in) {
          ok.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  for (auto &th : ts)
    th.join();
  EXPECT_EQ(ok.load(), 4 * 200);
}
