/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cctype>
#include <cvc/nav/policy_library.h>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {

namespace {

constexpr const char *kWho = "cvc::nav::policy_library: ";

bool is_sha256(const std::string &s) {
  return s.size() == 64 &&
         std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isxdigit(c) != 0; });
}

std::string lower(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

// A path or a file:// URI as a path. Another scheme (http:, cvc:, ...) is the caller's to fetch.
std::string to_path(const std::string &uri) {
  const std::string file = "file://";
  if (uri.compare(0, file.size(), file) == 0) {
    std::string p = uri.substr(file.size());
    // file:///C:/x on Windows: the drive letter follows the slash.
    if (p.size() > 2 && p[0] == '/' && std::isalpha(static_cast<unsigned char>(p[1])) &&
        p[2] == ':')
      p.erase(0, 1);
    return p;
  }
  const std::size_t colon = uri.find(':');
  // A scheme is two or more letters before a colon; one letter is a Windows drive.
  if (colon != std::string::npos && colon > 1 &&
      std::all_of(uri.begin(), uri.begin() + static_cast<std::ptrdiff_t>(colon),
                  [](unsigned char c) { return std::isalpha(c) != 0; }))
    throw std::runtime_error(std::string(kWho) + "can't load '" + uri +
                             "': only a path or a file:// URI");
  return uri;
}

} // namespace

policy_library::entry policy_library::get(const policy_ref &ref) {
  if (ref.uri.empty()) {
    if (!ref.sha256.empty())
      throw std::invalid_argument(std::string(kWho) + "a sha256 with no uri");
    std::lock_guard<std::mutex> lock(mu_);
    auto &m = by_key_["default"];
    if (!m)
      m = std::make_shared<const coef_mlp>(coef_mlp::default_biased());
    return {m, "default"};
  }
  std::string want;
  if (!ref.sha256.empty()) {
    if (!is_sha256(ref.sha256))
      throw std::invalid_argument(std::string(kWho) + "sha256 '" + ref.sha256 +
                                  "' is not 64 hex digits");
    want = "sha256:" + lower(ref.sha256);
    std::lock_guard<std::mutex> lock(mu_);
    const auto it = by_key_.find(want);
    if (it != by_key_.end())
      return {it->second, want};
  }
  const std::string path = to_path(ref.uri);
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error(std::string(kWho) + "can't read '" + path + "'");
  const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(in)),
                                         std::istreambuf_iterator<char>());
  const std::string key = "sha256:" + lower(sha256_hex(bytes));
  if (!want.empty() && key != want)
    throw std::runtime_error(std::string(kWho) + "'" + path + "' hashes to " + key.substr(7) +
                             ", not " + want.substr(7));
  {
    std::lock_guard<std::mutex> lock(mu_);
    const auto it = by_key_.find(key);
    if (it != by_key_.end())
      return {it->second, key};
  }
  // Parse outside the lock; a second thread loading the same bytes keeps the first copy.
  auto model =
      std::make_shared<const coef_mlp>(coef_mlp::load_from_memory(bytes.data(), bytes.size()));
  std::lock_guard<std::mutex> lock(mu_);
  auto &slot = by_key_[key];
  if (!slot)
    slot = std::move(model);
  return {slot, key};
}

std::size_t policy_library::size() const {
  std::lock_guard<std::mutex> lock(mu_);
  return by_key_.size();
}

void policy_library::clear() {
  std::lock_guard<std::mutex> lock(mu_);
  by_key_.clear();
}

} // namespace nav
} // namespace cvc
