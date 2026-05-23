/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_compression_registry.h>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

using cvc::state_compression_registry;

class StateZstdRoundTripTest : public ::testing::Test {
protected:
  state_compression_registry registry;
};

TEST_F(StateZstdRoundTripTest, ZstdIsRegistered) {
  auto ids = registry.ids();
  bool has_zstd = false;
  for (const auto &id : ids)
    if (id == "zstd") has_zstd = true;
  EXPECT_TRUE(has_zstd) << "zstd codec not found in registry";
}

TEST_F(StateZstdRoundTripTest, CompressDecompressEmpty) {
  std::vector<unsigned char> input;
  auto compressed = registry.encode("zstd", input);
  std::vector<unsigned char> decompressed;
  EXPECT_TRUE(registry.decode("zstd", compressed, decompressed));
  EXPECT_EQ(decompressed, input);
}

TEST_F(StateZstdRoundTripTest, CompressDecompressSmall) {
  std::string text = "Hello, zstd compression codec!";
  std::vector<unsigned char> input(text.begin(), text.end());
  auto compressed = registry.encode("zstd", input);
  // Compressed should be non-empty.
  EXPECT_FALSE(compressed.empty());
  std::vector<unsigned char> decompressed;
  EXPECT_TRUE(registry.decode("zstd", compressed, decompressed));
  EXPECT_EQ(decompressed, input);
}

TEST_F(StateZstdRoundTripTest, CompressDecompressLargeRepetitive) {
  // Repetitive data should compress well.
  std::vector<unsigned char> input(100000, static_cast<unsigned char>('A'));
  auto compressed = registry.encode("zstd", input);
  EXPECT_LT(compressed.size(), input.size() / 2)
      << "zstd should compress repetitive data significantly";
  std::vector<unsigned char> decompressed;
  EXPECT_TRUE(registry.decode("zstd", compressed, decompressed));
  EXPECT_EQ(decompressed, input);
}

TEST_F(StateZstdRoundTripTest, CompressDecompressRandom) {
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> dist(0, 255);
  std::vector<unsigned char> input(8192);
  for (auto &b : input)
    b = static_cast<unsigned char>(dist(rng));
  auto compressed = registry.encode("zstd", input);
  std::vector<unsigned char> decompressed;
  EXPECT_TRUE(registry.decode("zstd", compressed, decompressed));
  EXPECT_EQ(decompressed, input);
}

TEST_F(StateZstdRoundTripTest, CompressDecompressSingleByte) {
  std::vector<unsigned char> input{0x42};
  auto compressed = registry.encode("zstd", input);
  std::vector<unsigned char> decompressed;
  EXPECT_TRUE(registry.decode("zstd", compressed, decompressed));
  EXPECT_EQ(decompressed, input);
}

TEST_F(StateZstdRoundTripTest, SharedRegistryHasZstd) {
  auto &shared = state_compression_registry::shared();
  auto ids = shared.ids();
  bool has_zstd = false;
  for (const auto &id : ids)
    if (id == "zstd") has_zstd = true;
  EXPECT_TRUE(has_zstd);
}
