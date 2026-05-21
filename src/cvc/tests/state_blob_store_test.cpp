/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License version
  2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cvc/state_blob_store.h>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

std::vector<unsigned char> bytes_from(const std::string &s) {
  return std::vector<unsigned char>(s.begin(), s.end());
}

} // namespace

TEST(Sha256Test, KnownVectorEmpty) {
  EXPECT_EQ(cvc::sha256_hex(std::vector<unsigned char>{}),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(Sha256Test, KnownVectorAbc) {
  EXPECT_EQ(cvc::sha256_hex(bytes_from("abc")),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(Sha256Test, KnownVectorLongerString) {
  EXPECT_EQ(cvc::sha256_hex(bytes_from("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

TEST(Sha256Test, KnownVectorOneMillionA) {
  std::vector<unsigned char> a(1000000, 'a');
  EXPECT_EQ(cvc::sha256_hex(a), "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(MemoryBlobStoreTest, PutGetRoundtrip) {
  cvc::memory_state_blob_store store;
  auto data = bytes_from("hello-blob");
  cvc::state_blob_ref ref = store.put(data, "raw");
  EXPECT_EQ(ref.size_bytes, data.size());
  EXPECT_EQ(ref.codec, "raw");
  EXPECT_FALSE(ref.digest.empty());

  std::vector<unsigned char> out;
  EXPECT_TRUE(store.get(ref.digest, out));
  EXPECT_EQ(out, data);
  EXPECT_TRUE(store.has(ref.digest));
}

TEST(MemoryBlobStoreTest, DedupsIdenticalContent) {
  cvc::memory_state_blob_store store;
  auto a = bytes_from("same-bytes");
  auto b = bytes_from("same-bytes");
  cvc::state_blob_ref ra = store.put(a);
  cvc::state_blob_ref rb = store.put(b);
  EXPECT_EQ(ra.digest, rb.digest);
  EXPECT_EQ(store.size(), 1u);
  EXPECT_EQ(store.bytes_stored(), a.size());
}

TEST(MemoryBlobStoreTest, DifferentContentDifferentDigest) {
  cvc::memory_state_blob_store store;
  cvc::state_blob_ref ra = store.put(bytes_from("aaa"));
  cvc::state_blob_ref rb = store.put(bytes_from("bbb"));
  EXPECT_NE(ra.digest, rb.digest);
  EXPECT_EQ(store.size(), 2u);
}

TEST(MemoryBlobStoreTest, EraseRemoves) {
  cvc::memory_state_blob_store store;
  cvc::state_blob_ref r = store.put(bytes_from("transient"));
  EXPECT_TRUE(store.erase(r.digest));
  EXPECT_FALSE(store.has(r.digest));
  EXPECT_FALSE(store.erase(r.digest));
  EXPECT_EQ(store.size(), 0u);
  EXPECT_EQ(store.bytes_stored(), 0u);
}

TEST(MemoryBlobStoreTest, GetMissingReturnsFalse) {
  cvc::memory_state_blob_store store;
  std::vector<unsigned char> out;
  EXPECT_FALSE(store.get("deadbeef", out));
}

TEST(MemoryBlobStoreTest, LargeBlobRoundtrip) {
  cvc::memory_state_blob_store store;
  std::vector<unsigned char> big(1 << 20, 0); // 1 MiB
  std::mt19937 rng(42);
  for (auto &b : big)
    b = static_cast<unsigned char>(rng() & 0xff);

  cvc::state_blob_ref r = store.put(big, "binary");
  EXPECT_EQ(r.size_bytes, big.size());

  std::vector<unsigned char> got;
  ASSERT_TRUE(store.get(r.digest, got));
  EXPECT_EQ(got, big);
  EXPECT_EQ(store.bytes_stored(), big.size());
}

TEST(MemoryBlobStoreStressTest, OptionalConcurrentPutGetStress) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run blob store "
                    "stress tests";
  }
  cvc::memory_state_blob_store store;
  const int kThreads = 8;
  const int kPerThread = 2000;
  std::atomic<int> errors{0};
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < kPerThread; ++i) {
        std::string s = "thr" + std::to_string(t) + "_i" + std::to_string(i);
        auto bytes = bytes_from(s);
        cvc::state_blob_ref r = store.put(bytes);
        std::vector<unsigned char> got;
        if (!store.get(r.digest, got) || got != bytes)
          errors.fetch_add(1);
      }
    });
  }
  for (auto &th : threads)
    th.join();
  EXPECT_EQ(errors.load(), 0);
  EXPECT_EQ(store.size(), static_cast<std::size_t>(kThreads * kPerThread));
}

TEST(MemoryBlobStorePerformanceTest, OptionalPutThroughputSmoke) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run blob store "
                    "performance smoke tests";
  }
  cvc::memory_state_blob_store store;
  const int kCount = 5000;
  const int kSize = 4096;
  std::vector<unsigned char> payload(kSize, 0xab);

  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kCount; ++i) {
    payload[0] = static_cast<unsigned char>(i & 0xff);
    payload[1] = static_cast<unsigned char>((i >> 8) & 0xff);
    store.put(payload);
  }
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
  double mibps = (kCount * kSize) / (1024.0 * 1024.0) / secs;
  std::cerr << "[blob perf] " << kCount << " x " << kSize << "B put in " << secs << "s (" << mibps
            << " MiB/s)\n";
  EXPECT_LT(secs, 10.0);
  EXPECT_EQ(store.size(), static_cast<std::size_t>(kCount));
}
