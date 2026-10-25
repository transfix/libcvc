/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_DEVICE_KEY_H__
#define __CVC_STATE_DEVICE_KEY_H__

#include <cstddef>
#include <cvc/core/namespace.h>
#include <memory>
#include <string>

namespace cvc {

// ----------------
// Device keys (ARIADNE_CONFERENCE_ROADMAP §5f): authority for the device id
// ----------------
// The device id (state_identity.h) names a device but proves nothing. A device key makes it a
// claim the relay can check:
//   - A device keeps one 32-byte master SEED (load_device_seed).
//   - For each relay it derives an Ed25519 key from it (state_device_signer::derive: HMAC-SHA256
//     of the seed and the relay's address), so two relays see unrelated public keys, as they see
//     unrelated device ids (scoped_device_id).
//   - The relay's Hello carries a fresh nonce. The device answers with a DeviceProof: its public
//     key and a signature over device_proof_message (the nonce, both node ids and its device id).
//   - The relay verifies it and pins the public key to the device id on first use
//     (state_device_pins). A device whose proof checks against its pin is VERIFIED; one that
//     shows another key for a pinned id is not, and never gains what a verified device has.
// Built on OpenSSL's libcrypto; without it (device_keys_available() false) no key is derived, no
// proof verifies and every device is unverified.

// Whether this build has device keys (OpenSSL).
bool device_keys_available() noexcept;

// N cryptographically random bytes; "" when device keys are unavailable.
std::string device_random_bytes(std::size_t n);

// A relay's Hello nonce is kDeviceNonceBytes long; a device signs none longer than
// kDeviceNonceMaxBytes.
constexpr std::size_t kDeviceNonceBytes = 32;
constexpr std::size_t kDeviceNonceMaxBytes = 64;

// The bytes a device signs to answer NONCE: a fixed label, then the nonce, the relay's and the
// device's node ids and the device id, each prefixed with its length.
std::string device_proof_message(const std::string &nonce, const std::string &relay_node_id,
                                 const std::string &device_node_id, const std::string &device_id);

// Whether SIGNATURE (64 bytes) is PUBLIC_KEY's (32 raw Ed25519 bytes) over MESSAGE.
bool verify_device_proof(const std::string &public_key, const std::string &message,
                         const std::string &signature);

// One relay's Ed25519 key, derived from a device's seed.
class state_device_signer {
public:
  // The key for SCOPE (the relay's address) from SEED (32 bytes); nullptr when device keys are
  // unavailable or the seed is not 32 bytes.
  static std::shared_ptr<const state_device_signer> derive(const std::string &seed,
                                                           const std::string &scope);
  ~state_device_signer();
  state_device_signer(const state_device_signer &) = delete;
  state_device_signer &operator=(const state_device_signer &) = delete;

  // 32 raw bytes.
  const std::string &public_key() const noexcept { return _public_key; }
  // A 64-byte signature over MESSAGE; "" on failure.
  std::string sign(const std::string &message) const;

private:
  state_device_signer() = default;
  struct impl;
  std::unique_ptr<impl> _impl;
  std::string _public_key;
};

// The default path of a native device's seed: %LOCALAPPDATA%\cvc\device-key on Windows (UTF-8),
// else $XDG_CONFIG_HOME/cvc/device-key or ~/.config/cvc/device-key. "" when none can be named.
std::string default_device_key_path();

// This device's 32-byte master seed:
//   - CVC_DEVICE_KEY when set (64 hex digits: a test, or a browser page that keeps one in
//     localStorage);
//   - else the file CVC_DEVICE_KEY_FILE, or default_device_key_path(), created with 32 random
//     bytes (owner-only where the OS allows) when missing; two processes creating it at once
//     agree on one seed.
// "" when device keys are unavailable or no seed could be read or made. Under Emscripten only
// CVC_DEVICE_KEY counts (a browser has no file the page could keep).
std::string load_device_seed();

// Device id -> public key, trusted on first use. Optionally kept in a file, one
// "<device id> <public key hex>" line per pin, appended as pins are made, so a relay keeps its
// pins across restarts. Thread-safe.
class state_device_pins {
public:
  enum class verdict {
    pinned,     // first use: KEY is now this device's pin
    matched,    // KEY is the device's pin
    mismatched, // the device's pin is another key
    refused     // not pinned: a malformed id or key, or the store already holds kMaxPins
  };
  static constexpr std::size_t kMaxPins = 1u << 16;

  // PATH "" keeps the pins in memory only; otherwise its lines are loaded (a missing file is an
  // empty one) and new pins appended.
  explicit state_device_pins(std::string path = std::string());
  ~state_device_pins();
  state_device_pins(const state_device_pins &) = delete;
  state_device_pins &operator=(const state_device_pins &) = delete;

  verdict check(const std::string &device_id, const std::string &public_key);
  std::size_t size() const;
  const std::string &path() const noexcept { return _path; }

private:
  struct table;
  std::unique_ptr<table> _table;
  std::string _path;
};

} // namespace cvc

#endif // __CVC_STATE_DEVICE_KEY_H__
