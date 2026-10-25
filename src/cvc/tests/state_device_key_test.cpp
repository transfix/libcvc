/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Device keys (state_device_key.h): per-relay Ed25519 keys derived from one seed, the proof a
// device signs, the seed a device keeps, and the relay's trust-on-first-use pins.

#include <cstdlib>
#include <cvc/state/state_device_key.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <random>
#include <string>

namespace {

namespace fs = std::filesystem;

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

// Saves an environment variable and puts it back.
class saved_env {
public:
  explicit saved_env(const char *name) : _name(name) {
    if (const char *v = std::getenv(name)) {
      _had = true;
      _value = v;
    }
  }
  ~saved_env() { set_env(_name, _had ? _value.c_str() : nullptr); }

private:
  const char *_name;
  bool _had = false;
  std::string _value;
};

// A fresh directory under the system temp directory, removed afterwards.
class temp_dir {
public:
  explicit temp_dir(const std::string &tag) {
    // Unique per run: several worktrees' test runs may share the temp directory.
    std::random_device rd;
    _path = fs::temp_directory_path() /
            ("cvc-device-key-" + tag + "-" + std::to_string(rd()) + std::to_string(rd()));
    fs::remove_all(_path);
    fs::create_directories(_path);
  }
  ~temp_dir() {
    std::error_code ec;
    fs::remove_all(_path, ec);
  }
  const fs::path &path() const { return _path; }

private:
  fs::path _path;
};

std::string seed_of(char fill) { return std::string(32, fill); }

std::string first_line(const fs::path &p) {
  std::ifstream in(p);
  std::string line;
  std::getline(in, line);
  return line;
}

} // namespace

TEST(DeviceKey, ABuildWithoutKeysVerifiesNothing) {
  if (cvc::device_keys_available())
    GTEST_SKIP() << "this build has device keys";
  EXPECT_EQ(cvc::state_device_signer::derive(seed_of('s'), "ws://relay"), nullptr);
  EXPECT_FALSE(cvc::verify_device_proof(std::string(32, 'k'), "m", std::string(64, 's')));
  EXPECT_EQ(cvc::device_random_bytes(32), "");
}

TEST(DeviceKey, EachRelayGetsItsOwnKey) {
  if (!cvc::device_keys_available())
    GTEST_SKIP() << "no device keys in this build";
  const auto a = cvc::state_device_signer::derive(seed_of('s'), "ws://one.example");
  const auto a2 = cvc::state_device_signer::derive(seed_of('s'), "ws://one.example");
  const auto b = cvc::state_device_signer::derive(seed_of('s'), "ws://two.example");
  const auto other = cvc::state_device_signer::derive(seed_of('t'), "ws://one.example");
  ASSERT_NE(a, nullptr);
  ASSERT_NE(a2, nullptr);
  ASSERT_NE(b, nullptr);
  ASSERT_NE(other, nullptr);
  EXPECT_EQ(a->public_key().size(), 32u);
  EXPECT_EQ(a->public_key(), a2->public_key()) << "stable for one seed and relay";
  EXPECT_NE(a->public_key(), b->public_key()) << "another relay sees an unrelated key";
  EXPECT_NE(a->public_key(), other->public_key()) << "another device has another key";
  EXPECT_EQ(cvc::state_device_signer::derive(std::string(31, 's'), "ws://one.example"), nullptr)
      << "a seed is 32 bytes";
}

TEST(DeviceKey, AProofVerifiesOnlyAsSigned) {
  if (!cvc::device_keys_available())
    GTEST_SKIP() << "no device keys in this build";
  const auto key = cvc::state_device_signer::derive(seed_of('s'), "ws://relay");
  const auto stranger = cvc::state_device_signer::derive(seed_of('x'), "ws://relay");
  ASSERT_NE(key, nullptr);
  ASSERT_NE(stranger, nullptr);
  const std::string nonce = cvc::device_random_bytes(cvc::kDeviceNonceBytes);
  ASSERT_EQ(nonce.size(), cvc::kDeviceNonceBytes);
  const std::string msg = cvc::device_proof_message(nonce, "relay", "alice", "dev-a");
  const std::string sig = key->sign(msg);
  ASSERT_EQ(sig.size(), 64u);
  EXPECT_TRUE(cvc::verify_device_proof(key->public_key(), msg, sig));

  EXPECT_FALSE(cvc::verify_device_proof(stranger->public_key(), msg, sig)) << "another key";
  EXPECT_FALSE(cvc::verify_device_proof(
      key->public_key(), cvc::device_proof_message(nonce, "relay", "alice", "dev-b"), sig))
      << "another device id";
  EXPECT_FALSE(cvc::verify_device_proof(
      key->public_key(), cvc::device_proof_message(nonce, "relay2", "alice", "dev-a"), sig))
      << "another relay";
  EXPECT_FALSE(cvc::verify_device_proof(
      key->public_key(),
      cvc::device_proof_message(cvc::device_random_bytes(32), "relay", "alice", "dev-a"), sig))
      << "another challenge";
  std::string bent = sig;
  bent[10] = static_cast<char>(bent[10] ^ 1);
  EXPECT_FALSE(cvc::verify_device_proof(key->public_key(), msg, bent)) << "a changed signature";
  EXPECT_FALSE(cvc::verify_device_proof(key->public_key(), msg, sig.substr(0, 63)));
  EXPECT_FALSE(cvc::verify_device_proof(key->public_key().substr(0, 31), msg, sig));
  EXPECT_FALSE(cvc::verify_device_proof("", msg, sig));
}

// The parts are length-prefixed, so moving bytes from one part to its neighbour changes the
// message, even when the (binary) nonce holds a NUL.
TEST(DeviceKey, TheProofMessageSeparatesItsParts) {
  const std::string m = cvc::device_proof_message("ab", "c", "d", "e");
  EXPECT_NE(m, cvc::device_proof_message("a", "bc", "d", "e"));
  EXPECT_NE(m, cvc::device_proof_message("ab", "", "cd", "e"));
  const std::string nul(1, '\0');
  EXPECT_NE(cvc::device_proof_message("n" + nul + "r", "s", "d", "e"),
            cvc::device_proof_message("n", "r" + nul + "s", "d", "e"));
  EXPECT_EQ(m, cvc::device_proof_message("ab", "c", "d", "e"));
  EXPECT_EQ(m.rfind("cvc-device-proof/v1", 0), 0u) << "labelled";
}

TEST(DeviceKey, ASeedComesFromTheEnvironmentOrAKeptFile) {
  if (!cvc::device_keys_available())
    GTEST_SKIP() << "no device keys in this build";
  saved_env key("CVC_DEVICE_KEY"), file("CVC_DEVICE_KEY_FILE");
  const std::string hex(64, 'a');
  set_env("CVC_DEVICE_KEY", hex.c_str());
  EXPECT_EQ(cvc::load_device_seed(), std::string(32, static_cast<char>(0xaa)));
  set_env("CVC_DEVICE_KEY", "not-hex");
  EXPECT_EQ(cvc::load_device_seed(), "") << "a malformed key is no key";
  set_env("CVC_DEVICE_KEY", std::string(62, 'a').c_str());
  EXPECT_EQ(cvc::load_device_seed(), "") << "nor is a short one";

  set_env("CVC_DEVICE_KEY", nullptr);
  temp_dir dir("seed");
  const fs::path path = dir.path() / "nested" / "device-key";
  set_env("CVC_DEVICE_KEY_FILE", path.string().c_str());
  const std::string seed = cvc::load_device_seed();
  ASSERT_EQ(seed.size(), 32u) << "made on first use";
  ASSERT_TRUE(fs::exists(path));
  EXPECT_EQ(first_line(path).size(), 64u) << "kept as hex";
  EXPECT_EQ(cvc::load_device_seed(), seed) << "and read back after";
  std::size_t files = 0;
  for (const auto &e : fs::directory_iterator(path.parent_path())) {
    (void)e;
    ++files;
  }
  EXPECT_EQ(files, 1u) << "no temporary file is left beside it";
#if !defined(_WIN32)
  EXPECT_EQ(fs::status(path).permissions() & (fs::perms::group_all | fs::perms::others_all),
            fs::perms::none)
      << "readable by its owner only";
#endif
}

TEST(DeviceKey, PinsTrustTheFirstKeyAndKeepIt) {
  const std::string k1(32, '1'), k2(32, '2');
  temp_dir dir("pins");
  const std::string path = (dir.path() / "pins").string();
  {
    cvc::state_device_pins pins(path);
    EXPECT_EQ(pins.size(), 0u) << "a missing file is an empty store";
    EXPECT_EQ(pins.check("dev-a", k1), cvc::state_device_pins::verdict::pinned);
    EXPECT_EQ(pins.check("dev-a", k1), cvc::state_device_pins::verdict::matched);
    EXPECT_EQ(pins.check("dev-a", k2), cvc::state_device_pins::verdict::mismatched);
    EXPECT_EQ(pins.check("dev-a", k1), cvc::state_device_pins::verdict::matched)
        << "a mismatch does not move the pin";
    EXPECT_EQ(pins.check("dev-b", k2), cvc::state_device_pins::verdict::pinned);
    EXPECT_EQ(pins.size(), 2u);
    EXPECT_EQ(pins.check("dev c", k1), cvc::state_device_pins::verdict::refused) << "one word";
    EXPECT_EQ(pins.check("dev\nc", k1), cvc::state_device_pins::verdict::refused);
    EXPECT_EQ(pins.check("", k1), cvc::state_device_pins::verdict::refused);
    EXPECT_EQ(pins.check("dev-c", k1.substr(1)), cvc::state_device_pins::verdict::refused);
    EXPECT_EQ(pins.size(), 2u);
  }
  {
    std::ofstream junk(path, std::ios::app);
    junk << "# a comment\n\nno-key-here\ndev-d nothex\n";
  }
  cvc::state_device_pins reloaded(path);
  EXPECT_EQ(reloaded.size(), 2u) << "the pins survive a restart; malformed lines are skipped";
  EXPECT_EQ(reloaded.check("dev-a", k2), cvc::state_device_pins::verdict::mismatched);
  EXPECT_EQ(reloaded.check("dev-b", k2), cvc::state_device_pins::verdict::matched);
}

TEST(DeviceKey, AFullPinStoreRefusesNewDevices) {
  cvc::state_device_pins pins; // memory only
  const std::string key(32, 'k');
  for (std::size_t i = 0; i < cvc::state_device_pins::kMaxPins; ++i)
    ASSERT_EQ(pins.check("dev-" + std::to_string(i), key), cvc::state_device_pins::verdict::pinned);
  EXPECT_EQ(pins.check("dev-new", key), cvc::state_device_pins::verdict::refused);
  EXPECT_EQ(pins.check("dev-0", key), cvc::state_device_pins::verdict::matched)
      << "the devices it holds still match";
}
