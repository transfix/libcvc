/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License version
  2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_codec_registry.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

} // namespace

TEST(StateCodecRegistryTest, EmptyRegistryHasNoCodecs) {
  cvc::state_codec_registry reg;
  EXPECT_EQ(reg.size(), 0u);
  EXPECT_FALSE(reg.has("int32_t"));
  EXPECT_EQ(reg.codec_id_for("int32_t"), "");
}

TEST(StateCodecRegistryTest, BuiltinCodecsCoverScalars) {
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();
  for (const char *t :
       {"bool", "int32_t", "int64_t", "uint32_t", "uint64_t", "float",
        "double", "std::string"}) {
    EXPECT_TRUE(reg.has(t)) << "missing codec for " << t;
  }
  EXPECT_GE(reg.size(), 8u);
}

TEST(StateCodecRegistryTest, RoundtripInt32) {
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();
  std::int32_t in = -123456;
  auto bytes = reg.encode("int32_t", boost::any(in));
  EXPECT_EQ(bytes.size(), sizeof(std::int32_t));
  boost::any out = reg.decode("int32_t", bytes);
  EXPECT_EQ(boost::any_cast<std::int32_t>(out), in);
}

TEST(StateCodecRegistryTest, RoundtripDouble) {
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();
  double in = 3.14159265358979323846;
  auto bytes = reg.encode("double", boost::any(in));
  EXPECT_EQ(bytes.size(), sizeof(double));
  boost::any out = reg.decode("double", bytes);
  EXPECT_EQ(boost::any_cast<double>(out), in);
}

TEST(StateCodecRegistryTest, RoundtripBool) {
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();
  auto bt = reg.encode("bool", boost::any(true));
  auto bf = reg.encode("bool", boost::any(false));
  EXPECT_EQ(bt.size(), 1u);
  EXPECT_EQ(bf.size(), 1u);
  EXPECT_TRUE(boost::any_cast<bool>(reg.decode("bool", bt)));
  EXPECT_FALSE(boost::any_cast<bool>(reg.decode("bool", bf)));
}

TEST(StateCodecRegistryTest, RoundtripString) {
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();
  std::string in = "hello, distributed state";
  auto bytes = reg.encode("std::string", boost::any(in));
  EXPECT_EQ(bytes.size(), in.size());
  std::string out = boost::any_cast<std::string>(reg.decode("std::string", bytes));
  EXPECT_EQ(out, in);
}

TEST(StateCodecRegistryTest, CodecIdsAreStable) {
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();
  EXPECT_EQ(reg.codec_id_for("int32_t"), "cvc.i32.v1");
  EXPECT_EQ(reg.codec_id_for("double"), "cvc.f64.v1");
  EXPECT_EQ(reg.codec_id_for("std::string"), "cvc.str.v1");
}

TEST(StateCodecRegistryTest, UnknownTypeThrows) {
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();
  EXPECT_THROW(reg.encode("not::a::type", boost::any(0)), std::runtime_error);
  EXPECT_THROW(reg.decode("not::a::type", {}), std::runtime_error);
}

TEST(StateCodecRegistryTest, CustomCodecOverrides) {
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();
  reg.register_codec(
      "int32_t",
      [](const boost::any &v) {
        // Stash a one-byte marker so we can detect the override.
        return std::vector<unsigned char>{
            0xAB, static_cast<unsigned char>(boost::any_cast<std::int32_t>(v))};
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(static_cast<std::int32_t>(b.size() > 1 ? b[1] : 0));
      },
      "test.override.v1");
  EXPECT_EQ(reg.codec_id_for("int32_t"), "test.override.v1");
  auto bytes = reg.encode("int32_t", boost::any(std::int32_t{7}));
  ASSERT_EQ(bytes.size(), 2u);
  EXPECT_EQ(bytes[0], 0xAB);
  EXPECT_EQ(bytes[1], 7u);
}

TEST(StateCodecRegistryStressTest, OptionalConcurrentLookupStress) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run codec "
                    "registry stress tests";
  }
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();

  const int kThreads = 8;
  const int kPerThread = 20000;
  std::atomic<int> errors{0};
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&]() {
      for (int i = 0; i < kPerThread; ++i) {
        auto bytes = reg.encode("int64_t",
                                boost::any(static_cast<std::int64_t>(i)));
        std::int64_t v =
            boost::any_cast<std::int64_t>(reg.decode("int64_t", bytes));
        if (v != static_cast<std::int64_t>(i))
          errors.fetch_add(1);
      }
    });
  }
  for (auto &th : threads)
    th.join();
  EXPECT_EQ(errors.load(), 0);
}

TEST(StateCodecRegistryPerformanceTest,
     OptionalEncodeDecodeThroughputSmoke) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run codec registry "
                    "performance smoke tests";
  }
  cvc::state_codec_registry reg;
  reg.register_builtin_codecs();
  const int kIters = 200000;
  auto start = std::chrono::steady_clock::now();
  std::int64_t acc = 0;
  for (int i = 0; i < kIters; ++i) {
    auto bytes =
        reg.encode("int64_t", boost::any(static_cast<std::int64_t>(i)));
    acc += boost::any_cast<std::int64_t>(reg.decode("int64_t", bytes));
  }
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs =
      std::chrono::duration_cast<std::chrono::duration<double>>(elapsed)
          .count();
  std::cerr << "[codec perf] " << kIters << " encode+decode in " << secs
            << "s (" << (kIters / secs) << "/s)\n";
  EXPECT_LT(secs, 10.0);
  EXPECT_EQ(acc,
            static_cast<std::int64_t>(kIters) * (kIters - 1) / 2);
}
