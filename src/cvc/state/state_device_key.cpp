/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/state/state_device_key.h>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <system_error>
#include <unordered_map>

#if defined(CVC_HAVE_DEVICE_KEYS)
#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/rand.h>
#endif

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h> // MoveFileExW, GetCurrentProcessId
#elif !defined(__EMSCRIPTEN__)
#include <fcntl.h>
#include <unistd.h>
#endif

namespace cvc {

namespace {

constexpr std::size_t kSeedBytes = 32;
constexpr std::size_t kPublicKeyBytes = 32;
constexpr std::size_t kSignatureBytes = 64;

std::string to_hex(const std::string &bytes) {
  static const char *digits = "0123456789abcdef";
  std::string out;
  out.reserve(bytes.size() * 2);
  for (unsigned char b : bytes) {
    out += digits[b >> 4];
    out += digits[b & 15];
  }
  return out;
}

// HEX's bytes, or "" when it is not an even run of hex digits.
std::string from_hex(const std::string &hex) {
  if (hex.size() % 2 != 0)
    return {};
  const auto nibble = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    return -1;
  };
  std::string out;
  out.reserve(hex.size() / 2);
  for (std::size_t i = 0; i < hex.size(); i += 2) {
    const int hi = nibble(hex[i]), lo = nibble(hex[i + 1]);
    if (hi < 0 || lo < 0)
      return {};
    out += static_cast<char>(hi * 16 + lo);
  }
  return out;
}

std::string trimmed(std::string s) {
  const auto space = [](unsigned char c) { return std::isspace(c) != 0; };
  while (!s.empty() && space(static_cast<unsigned char>(s.back())))
    s.pop_back();
  std::size_t i = 0;
  while (i < s.size() && space(static_cast<unsigned char>(s[i])))
    ++i;
  return s.substr(i);
}

namespace fs = std::filesystem;

// The 32-byte seed in the first line of PATH ("" when absent or malformed).
[[maybe_unused]] std::string read_seed_file(const fs::path &path) {
  std::ifstream in(path);
  std::string line;
  if (!in || !std::getline(in, line))
    return {};
  const std::string seed = from_hex(trimmed(line));
  return seed.size() == kSeedBytes ? seed : std::string();
}

// Make PATH hold SEED's hex, unless it already exists. The seed is written in full to a file of
// this process's own first and then published under PATH in one step that fails if PATH exists,
// so two first launches agree on one seed and neither can read a half-written file. Owner-only on
// POSIX. False when PATH existed (or appeared meanwhile) or could not be made.
[[maybe_unused]] bool create_seed_file(const fs::path &path, const std::string &seed) {
  std::error_code ec;
  fs::create_directories(path.parent_path(), ec);
  const std::string text = to_hex(seed) + "\n";
#if defined(_WIN32)
  fs::path tmp = path;
  tmp += L".tmp." + std::to_wstring(::GetCurrentProcessId());
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    out << text;
    if (!out.flush()) {
      fs::remove(tmp, ec);
      return false;
    }
  }
  // Without MOVEFILE_REPLACE_EXISTING the move fails when PATH exists.
  const bool ok = ::MoveFileExW(tmp.c_str(), path.c_str(), MOVEFILE_WRITE_THROUGH) != 0;
  if (!ok)
    fs::remove(tmp, ec);
  return ok;
#elif !defined(__EMSCRIPTEN__)
  const std::string tmp = path.string() + ".tmp." + std::to_string(::getpid());
  ::unlink(tmp.c_str());
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (fd < 0)
    return false;
  bool ok = ::write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size());
  ok = ::close(fd) == 0 && ok;
  // link() fails when PATH exists, and publishes the whole file at once.
  ok = ok && ::link(tmp.c_str(), path.c_str()) == 0;
  ::unlink(tmp.c_str());
  return ok;
#else
  (void)text;
  return false; // a browser keeps its seed in the page (CVC_DEVICE_KEY)
#endif
}

// The default seed file (default_device_key_path), as a path: wide on Windows, so a profile
// directory outside the ANSI code page still works.
fs::path device_key_file() {
#if defined(__EMSCRIPTEN__)
  return {};
#elif defined(_WIN32)
  // Local, not roaming: the seed is this machine's and should not sync to others.
  if (const wchar_t *dir = ::_wgetenv(L"LOCALAPPDATA"); dir != nullptr && *dir != L'\0')
    return fs::path(dir) / L"cvc" / L"device-key";
  return {};
#else
  if (const char *xdg = std::getenv("XDG_CONFIG_HOME"); xdg != nullptr && *xdg != '\0')
    return fs::path(xdg) / "cvc" / "device-key";
  if (const char *home = std::getenv("HOME"); home != nullptr && *home != '\0')
    return fs::path(home) / ".config" / "cvc" / "device-key";
  return {};
#endif
}

} // namespace

bool device_keys_available() noexcept {
#if defined(CVC_HAVE_DEVICE_KEYS)
  return true;
#else
  return false;
#endif
}

std::string device_random_bytes(std::size_t n) {
#if defined(CVC_HAVE_DEVICE_KEYS)
  std::string out(n, '\0');
  if (n == 0 || RAND_bytes(reinterpret_cast<unsigned char *>(&out[0]), static_cast<int>(n)) == 1)
    return out;
#else
  (void)n;
#endif
  return {};
}

std::string device_proof_message(const std::string &nonce, const std::string &relay_node_id,
                                 const std::string &device_node_id, const std::string &device_id) {
  // Each part length-prefixed (4 bytes, big-endian): the nonce is binary, so no separator byte
  // could keep its bytes from being read as part of the next field.
  std::string m = "cvc-device-proof/v1";
  for (const std::string *part : {&nonce, &relay_node_id, &device_node_id, &device_id}) {
    const std::uint32_t n = static_cast<std::uint32_t>(part->size());
    for (int shift = 24; shift >= 0; shift -= 8)
      m.push_back(static_cast<char>((n >> shift) & 0xff));
    m += *part;
  }
  return m;
}

bool verify_device_proof(const std::string &public_key, const std::string &message,
                         const std::string &signature) {
#if defined(CVC_HAVE_DEVICE_KEYS)
  if (public_key.size() != kPublicKeyBytes || signature.size() != kSignatureBytes)
    return false;
  EVP_PKEY *key = EVP_PKEY_new_raw_public_key(
      EVP_PKEY_ED25519, nullptr, reinterpret_cast<const unsigned char *>(public_key.data()),
      public_key.size());
  if (key == nullptr)
    return false;
  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  bool ok = ctx != nullptr && EVP_DigestVerifyInit(ctx, nullptr, nullptr, nullptr, key) == 1 &&
            EVP_DigestVerify(
                ctx, reinterpret_cast<const unsigned char *>(signature.data()), signature.size(),
                reinterpret_cast<const unsigned char *>(message.data()), message.size()) == 1;
  EVP_MD_CTX_free(ctx);
  EVP_PKEY_free(key);
  return ok;
#else
  (void)public_key;
  (void)message;
  (void)signature;
  return false;
#endif
}

struct state_device_signer::impl {
#if defined(CVC_HAVE_DEVICE_KEYS)
  EVP_PKEY *key = nullptr;
  ~impl() { EVP_PKEY_free(key); }
#endif
};

state_device_signer::~state_device_signer() = default;

std::shared_ptr<const state_device_signer> state_device_signer::derive(const std::string &seed,
                                                                       const std::string &scope) {
#if defined(CVC_HAVE_DEVICE_KEYS)
  if (seed.size() != kSeedBytes)
    return nullptr;
  // The relay's key: HMAC-SHA256(seed, label || scope), so one seed gives each relay its own key.
  const std::string data = std::string("cvc-device-key/v1") + '\0' + scope;
  unsigned char derived[32];
  unsigned int derived_len = 0;
  if (HMAC(EVP_sha256(), seed.data(), static_cast<int>(seed.size()),
           reinterpret_cast<const unsigned char *>(data.data()), data.size(), derived,
           &derived_len) == nullptr ||
      derived_len != sizeof(derived))
    return nullptr;
  EVP_PKEY *key = EVP_PKEY_new_raw_private_key(EVP_PKEY_ED25519, nullptr, derived, sizeof(derived));
  OPENSSL_cleanse(derived, sizeof(derived));
  if (key == nullptr)
    return nullptr;
  unsigned char pub[kPublicKeyBytes];
  std::size_t pub_len = sizeof(pub);
  if (EVP_PKEY_get_raw_public_key(key, pub, &pub_len) != 1 || pub_len != sizeof(pub)) {
    EVP_PKEY_free(key);
    return nullptr;
  }
  std::shared_ptr<state_device_signer> s(new state_device_signer());
  s->_impl = std::make_unique<impl>();
  s->_impl->key = key;
  s->_public_key.assign(reinterpret_cast<const char *>(pub), pub_len);
  return s;
#else
  (void)seed;
  (void)scope;
  return nullptr;
#endif
}

std::string state_device_signer::sign(const std::string &message) const {
#if defined(CVC_HAVE_DEVICE_KEYS)
  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  std::string sig(kSignatureBytes, '\0');
  std::size_t sig_len = sig.size();
  const bool ok = ctx != nullptr &&
                  EVP_DigestSignInit(ctx, nullptr, nullptr, nullptr, _impl->key) == 1 &&
                  EVP_DigestSign(ctx, reinterpret_cast<unsigned char *>(&sig[0]), &sig_len,
                                 reinterpret_cast<const unsigned char *>(message.data()),
                                 message.size()) == 1 &&
                  sig_len == kSignatureBytes;
  EVP_MD_CTX_free(ctx);
  return ok ? sig : std::string();
#else
  (void)message;
  return {};
#endif
}

std::string default_device_key_path() {
  const fs::path p = device_key_file();
#if defined(_WIN32)
  const auto u8 = p.u8string(); // std::string in C++17, std::u8string in C++20
  return std::string(u8.begin(), u8.end());
#else
  return p.string();
#endif
}

std::string load_device_seed() {
  if (!device_keys_available())
    return {};
  if (const char *v = std::getenv("CVC_DEVICE_KEY"); v != nullptr && *v != '\0') {
    const std::string seed = from_hex(trimmed(v));
    return seed.size() == kSeedBytes ? seed : std::string();
  }
#if defined(__EMSCRIPTEN__)
  return {};
#else
  fs::path path;
  if (const char *f = std::getenv("CVC_DEVICE_KEY_FILE"); f != nullptr && *f != '\0')
    path = f;
  else
    path = device_key_file();
  if (path.empty())
    return {};
  if (std::string seed = read_seed_file(path); !seed.empty())
    return seed;
  const std::string fresh = device_random_bytes(kSeedBytes);
  if (fresh.size() != kSeedBytes)
    return {};
  if (create_seed_file(path, fresh))
    return fresh;
  return read_seed_file(path); // another process made it first
#endif
}

struct state_device_pins::table {
  mutable std::mutex mu;
  std::unordered_map<std::string, std::string> pins; // device id -> raw public key
};

state_device_pins::state_device_pins(std::string path)
    : _table(std::make_unique<table>()), _path(std::move(path)) {
  if (_path.empty())
    return;
  std::ifstream in(_path);
  std::string line;
  while (std::getline(in, line)) {
    line = trimmed(line);
    const std::size_t space = line.find(' ');
    if (line.empty() || line[0] == '#' || space == std::string::npos)
      continue;
    const std::string key = from_hex(trimmed(line.substr(space + 1)));
    if (key.size() == kPublicKeyBytes && _table->pins.size() < kMaxPins)
      _table->pins.emplace(line.substr(0, space), key); // the first pin of an id wins
  }
}

state_device_pins::~state_device_pins() = default;

state_device_pins::verdict state_device_pins::check(const std::string &device_id,
                                                    const std::string &public_key) {
  // An id is one word (a line of the file holds the id, a space and the key).
  const bool word = !device_id.empty() && device_id.size() <= 256 &&
                    std::none_of(device_id.begin(), device_id.end(), [](unsigned char c) {
                      return std::isspace(c) != 0 || std::iscntrl(c) != 0;
                    });
  if (!word || public_key.size() != kPublicKeyBytes)
    return verdict::refused;
  std::lock_guard<std::mutex> lk(_table->mu);
  if (const auto it = _table->pins.find(device_id); it != _table->pins.end())
    return it->second == public_key ? verdict::matched : verdict::mismatched;
  if (_table->pins.size() >= kMaxPins)
    return verdict::refused;
  _table->pins.emplace(device_id, public_key);
  if (!_path.empty()) {
    std::ofstream out(_path, std::ios::app);
    out << device_id << ' ' << to_hex(public_key) << '\n';
    if (!out.flush())
      std::fprintf(stderr,
                   "cvc: device pins: could not add to %s; this pin lasts until a restart\n",
                   _path.c_str());
  }
  return verdict::pinned;
}

std::size_t state_device_pins::size() const {
  std::lock_guard<std::mutex> lk(_table->mu);
  return _table->pins.size();
}

} // namespace cvc
