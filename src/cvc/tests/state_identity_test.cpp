/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The peer identity helpers (state_identity.h): the constant-time token compare, the room-token
// rule and the default device id.

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cvc/state/state_identity.h>
#include <gtest/gtest.h>
#include <string>

namespace {

void set_env(const char *name, const char *value) {
#if defined(_WIN32)
  _putenv_s(name, value ? value : "");
#else
  if (value)
    setenv(name, value, 1);
  else
    unsetenv(name);
#endif
}

// Runs with CVC_DEVICE_ID unset, and puts back what was there.
class DeviceIdEnv : public ::testing::Test {
protected:
  void SetUp() override {
    if (const char *v = std::getenv("CVC_DEVICE_ID")) {
      _had = true;
      _saved = v;
    }
    set_env("CVC_DEVICE_ID", nullptr);
  }
  void TearDown() override { set_env("CVC_DEVICE_ID", _had ? _saved.c_str() : nullptr); }

private:
  bool _had = false;
  std::string _saved;
};

} // namespace

TEST(StateIdentity, ConstantTimeEqualComparesWholeStrings) {
  EXPECT_TRUE(cvc::constant_time_equal("", ""));
  EXPECT_TRUE(cvc::constant_time_equal("Bearer abc", "Bearer abc"));
  EXPECT_FALSE(cvc::constant_time_equal("Bearer abc", "Bearer abd"));
  EXPECT_FALSE(cvc::constant_time_equal("Bearer abc", "Bearer ab"));
  EXPECT_FALSE(cvc::constant_time_equal("", "x"));
}

TEST(StateIdentity, ARoomTokenFitsASubprotocol) {
  EXPECT_TRUE(cvc::valid_room_token("abcdefghijklmnop"));
  EXPECT_TRUE(cvc::valid_room_token("A-Z_0.9~a-z_0.9~"));
  EXPECT_TRUE(cvc::valid_room_token(std::string(256, 'x')));
  EXPECT_FALSE(cvc::valid_room_token("short")) << "under 16";
  EXPECT_FALSE(cvc::valid_room_token(std::string(257, 'x'))) << "over 256";
  for (const char *bad : {"abcdefghijklmnop q", "abcdefghijklmnop,q", "abcdefghijklmnop/q",
                          "abcdefghijklmnop=q", "abcdefghijklmnop\"q"})
    EXPECT_FALSE(cvc::valid_room_token(bad)) << bad;
}

TEST_F(DeviceIdEnv, TheDefaultDeviceIdIsStableHex) {
  const std::string id = cvc::default_device_id();
  ASSERT_EQ(id.size(), 32u) << "this test host has a machine id, host name or user";
  EXPECT_TRUE(std::all_of(id.begin(), id.end(), [](unsigned char c) {
    return std::isdigit(c) || (c >= 'a' && c <= 'f');
  })) << id;
  EXPECT_EQ(cvc::default_device_id(), id) << "the same on every call";
}

TEST(StateIdentity, ADeviceIdOnTheWireIsShortAndPlain) {
  EXPECT_TRUE(cvc::valid_device_id("0123456789abcdef0123456789abcdef"));
  EXPECT_TRUE(cvc::valid_device_id(std::string(64, 'a')));
  EXPECT_FALSE(cvc::valid_device_id(""));
  EXPECT_FALSE(cvc::valid_device_id(std::string(65, 'a')));
  EXPECT_FALSE(cvc::valid_device_id("abc def"));
}

TEST(StateIdentity, AScopedDeviceIdDiffersPerRelay) {
  const std::string dev = "0123456789abcdef0123456789abcdef";
  const std::string a = cvc::scoped_device_id(dev, "wss://one.example/cyprty-ws");
  ASSERT_EQ(a.size(), 32u);
  EXPECT_TRUE(cvc::valid_device_id(a));
  EXPECT_EQ(cvc::scoped_device_id(dev, "wss://one.example/cyprty-ws"), a) << "stable";
  EXPECT_NE(cvc::scoped_device_id(dev, "wss://two.example/cyprty-ws"), a)
      << "another relay sees another id";
  EXPECT_NE(a, dev);
  EXPECT_EQ(cvc::scoped_device_id("", "wss://one.example/cyprty-ws"), "");
}

TEST_F(DeviceIdEnv, CvcDeviceIdOverridesIt) {
  set_env("CVC_DEVICE_ID", "0123456789abcdef0123456789abcdef");
  EXPECT_EQ(cvc::default_device_id(), "0123456789abcdef0123456789abcdef");
}
