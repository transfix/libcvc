/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_blob_store.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace CVC_NAMESPACE {

namespace {

// ---- Self-contained SHA-256 implementation (FIPS 180-4) ----
// This avoids a hard dependency on OpenSSL for the distributed-
// state layer. It is intentionally simple and unrolled-free; we
// can swap in a vectorized backend later behind sha256_hex().

constexpr std::uint32_t k_sha256_k[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu,
    0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u,
    0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u,
    0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
    0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u,
    0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
    0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu,
    0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u,
    0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u, 0x1e376c08u,
    0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu,
    0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
    0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};

inline std::uint32_t rotr32(std::uint32_t x, std::uint32_t n) {
  return (x >> n) | (x << (32 - n));
}

void sha256_compress(std::uint32_t state[8], const unsigned char block[64]) {
  std::uint32_t w[64];
  for (int i = 0; i < 16; ++i) {
    w[i] = (static_cast<std::uint32_t>(block[i * 4]) << 24) |
           (static_cast<std::uint32_t>(block[i * 4 + 1]) << 16) |
           (static_cast<std::uint32_t>(block[i * 4 + 2]) << 8) |
           (static_cast<std::uint32_t>(block[i * 4 + 3]));
  }
  for (int i = 16; i < 64; ++i) {
    std::uint32_t s0 = rotr32(w[i - 15], 7) ^ rotr32(w[i - 15], 18) ^
                       (w[i - 15] >> 3);
    std::uint32_t s1 = rotr32(w[i - 2], 17) ^ rotr32(w[i - 2], 19) ^
                       (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  std::uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
  std::uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
  for (int i = 0; i < 64; ++i) {
    std::uint32_t S1 = rotr32(e, 6) ^ rotr32(e, 11) ^ rotr32(e, 25);
    std::uint32_t ch = (e & f) ^ (~e & g);
    std::uint32_t temp1 = h + S1 + ch + k_sha256_k[i] + w[i];
    std::uint32_t S0 = rotr32(a, 2) ^ rotr32(a, 13) ^ rotr32(a, 22);
    std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    std::uint32_t temp2 = S0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + temp1;
    d = c;
    c = b;
    b = a;
    a = temp1 + temp2;
  }
  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;
}

std::string sha256_hex_impl(const unsigned char *data, std::size_t len) {
  std::uint32_t state[8] = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u,
                            0xa54ff53au, 0x510e527fu, 0x9b05688cu,
                            0x1f83d9abu, 0x5be0cd19u};
  std::uint64_t total_bits = static_cast<std::uint64_t>(len) * 8u;

  // Process full 64-byte blocks.
  std::size_t i = 0;
  while (i + 64 <= len) {
    sha256_compress(state, data + i);
    i += 64;
  }
  // Final block(s): copy remainder, append 0x80, pad, append length.
  unsigned char buf[128] = {0};
  std::size_t rem = len - i;
  if (rem > 0)
    std::memcpy(buf, data + i, rem);
  buf[rem] = 0x80;
  std::size_t pad_end = (rem < 56) ? 56 : 120;
  for (int b = 7; b >= 0; --b) {
    buf[pad_end + b] = static_cast<unsigned char>(total_bits & 0xffu);
    total_bits >>= 8;
  }
  sha256_compress(state, buf);
  if (pad_end == 120)
    sha256_compress(state, buf + 64);

  static const char hex[] = "0123456789abcdef";
  std::string out;
  out.resize(64);
  for (int w = 0; w < 8; ++w) {
    std::uint32_t v = state[w];
    for (int b = 0; b < 4; ++b) {
      unsigned char byte =
          static_cast<unsigned char>((v >> (24 - b * 8)) & 0xffu);
      out[w * 8 + b * 2] = hex[byte >> 4];
      out[w * 8 + b * 2 + 1] = hex[byte & 0xf];
    }
  }
  return out;
}

} // anonymous namespace

std::string sha256_hex(const unsigned char *data, std::size_t len) {
  return sha256_hex_impl(data, len);
}

std::string sha256_hex(const std::vector<unsigned char> &bytes) {
  return sha256_hex_impl(bytes.data(), bytes.size());
}

// ---------------- memory_state_blob_store ----------------

memory_state_blob_store::memory_state_blob_store() : _bytes_stored(0) {}

memory_state_blob_store::~memory_state_blob_store() = default;

state_blob_ref
memory_state_blob_store::put(const std::vector<unsigned char> &bytes,
                             const std::string &codec) {
  std::string digest = sha256_hex(bytes);
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _blobs.find(digest);
  if (it == _blobs.end()) {
    _bytes_stored += bytes.size();
    _blobs.emplace(digest, bytes);
  }
  state_blob_ref ref;
  ref.digest = digest;
  ref.size_bytes = bytes.size();
  ref.codec = codec;
  return ref;
}

bool memory_state_blob_store::get(const std::string &digest,
                                  std::vector<unsigned char> &out) const {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _blobs.find(digest);
  if (it == _blobs.end())
    return false;
  out = it->second;
  return true;
}

bool memory_state_blob_store::has(const std::string &digest) const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _blobs.find(digest) != _blobs.end();
}

bool memory_state_blob_store::erase(const std::string &digest) {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _blobs.find(digest);
  if (it == _blobs.end())
    return false;
  _bytes_stored -= it->second.size();
  _blobs.erase(it);
  return true;
}

std::size_t memory_state_blob_store::size() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _blobs.size();
}

std::uint64_t memory_state_blob_store::bytes_stored() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _bytes_stored;
}

std::vector<std::string> memory_state_blob_store::digests() const {
  std::lock_guard<std::mutex> lk(_mutex);
  std::vector<std::string> out;
  out.reserve(_blobs.size());
  for (const auto &kv : _blobs)
    out.push_back(kv.first);
  return out;
}

} // namespace CVC_NAMESPACE
