/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_compression_registry.h>

#include <stdexcept>

namespace CVC_NAMESPACE {

// ---------- state_rle_compression_codec ----------

std::vector<unsigned char> state_rle_compression_codec::encode(
    const std::vector<unsigned char> &in) const {
  std::vector<unsigned char> out;
  out.reserve(in.size());
  std::size_t i = 0;
  while (i < in.size()) {
    unsigned char b = in[i];
    std::size_t run = 1;
    while (i + run < in.size() && in[i + run] == b && run < 255) {
      ++run;
    }
    out.push_back(static_cast<unsigned char>(run));
    out.push_back(b);
    i += run;
  }
  return out;
}

bool state_rle_compression_codec::decode(
    const std::vector<unsigned char> &in,
    std::vector<unsigned char> &out) const {
  out.clear();
  if (in.size() % 2 != 0) {
    return false;
  }
  out.reserve(in.size() * 2);
  for (std::size_t i = 0; i < in.size(); i += 2) {
    unsigned char run = in[i];
    if (run == 0) {
      out.clear();
      return false;
    }
    out.insert(out.end(), run, in[i + 1]);
  }
  return true;
}

// ---------- state_compression_registry ----------

state_compression_registry::state_compression_registry() {
  register_codec(std::make_shared<state_raw_compression_codec>());
  register_codec(std::make_shared<state_rle_compression_codec>());
}

void state_compression_registry::register_codec(
    std::shared_ptr<state_compression_codec> codec) {
  if (!codec) {
    return;
  }
  std::lock_guard<std::mutex> lock(_mutex);
  _codecs[codec->id()] = std::move(codec);
}

std::shared_ptr<state_compression_codec>
state_compression_registry::get(const std::string &id) const {
  std::lock_guard<std::mutex> lock(_mutex);
  auto it = _codecs.find(id);
  if (it == _codecs.end()) {
    return nullptr;
  }
  return it->second;
}

bool state_compression_registry::has(const std::string &id) const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _codecs.find(id) != _codecs.end();
}

std::vector<std::string> state_compression_registry::ids() const {
  std::lock_guard<std::mutex> lock(_mutex);
  std::vector<std::string> out;
  out.reserve(_codecs.size());
  for (const auto &kv : _codecs) {
    out.push_back(kv.first);
  }
  return out;
}

std::size_t state_compression_registry::size() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _codecs.size();
}

std::vector<unsigned char> state_compression_registry::encode(
    const std::string &id, const std::vector<unsigned char> &in,
    bool strict) const {
  // Empty id is treated as "no codec" -> identity.
  if (id.empty()) {
    return in;
  }
  auto codec = get(id);
  if (!codec) {
    if (strict) {
      throw std::runtime_error(
          "state_compression_registry: unknown codec id: " + id);
    }
    return in;
  }
  return codec->encode(in);
}

bool state_compression_registry::decode(
    const std::string &id, const std::vector<unsigned char> &in,
    std::vector<unsigned char> &out, bool strict) const {
  if (id.empty()) {
    out = in;
    return true;
  }
  auto codec = get(id);
  if (!codec) {
    if (strict) {
      throw std::runtime_error(
          "state_compression_registry: unknown codec id: " + id);
    }
    out = in;
    return true;
  }
  return codec->decode(in, out);
}

state_compression_registry &state_compression_registry::shared() {
  static state_compression_registry instance;
  return instance;
}

} // namespace CVC_NAMESPACE
