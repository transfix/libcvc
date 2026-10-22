/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <boost/any.hpp>
#include <boost/cstdint.hpp>
#include <cstdint>
#include <cstring>
#include <cvc/state/state_codec_registry.h>
#include <stdexcept>
#include <string>

namespace cvc {

namespace {

// Little-endian helpers.
template <typename T> std::vector<unsigned char> encode_pod(T value) {
  static_assert(std::is_trivially_copyable<T>::value, "POD only");
  std::vector<unsigned char> out(sizeof(T));
  unsigned char *p = reinterpret_cast<unsigned char *>(&value);
  // Reorder to little-endian by detecting at runtime; libcvc supports
  // little-endian hosts in practice but be explicit on the wire.
  std::uint16_t probe = 1;
  bool host_le = *reinterpret_cast<unsigned char *>(&probe) == 1;
  if (host_le) {
    std::memcpy(out.data(), p, sizeof(T));
  } else {
    for (std::size_t i = 0; i < sizeof(T); ++i)
      out[i] = p[sizeof(T) - 1 - i];
  }
  return out;
}

template <typename T> T decode_pod(const std::vector<unsigned char> &bytes) {
  if (bytes.size() < sizeof(T))
    throw std::runtime_error("codec: insufficient bytes for POD decode");
  T value{};
  std::uint16_t probe = 1;
  bool host_le = *reinterpret_cast<unsigned char *>(&probe) == 1;
  unsigned char *p = reinterpret_cast<unsigned char *>(&value);
  if (host_le) {
    std::memcpy(p, bytes.data(), sizeof(T));
  } else {
    for (std::size_t i = 0; i < sizeof(T); ++i)
      p[i] = bytes[sizeof(T) - 1 - i];
  }
  return value;
}

} // anonymous namespace

state_codec_registry::state_codec_registry() = default;

void state_codec_registry::register_codec(const std::string &type_name, encode_func encode,
                                          decode_func decode, const std::string &codec_id) {
  std::lock_guard<std::mutex> lk(_mutex);
  codec_entry e;
  e.codec_id = codec_id.empty() ? type_name : codec_id;
  e.encode = std::move(encode);
  e.decode = std::move(decode);
  _codecs[type_name] = std::move(e);
}

bool state_codec_registry::has(const std::string &type_name) const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _codecs.find(type_name) != _codecs.end();
}

std::vector<unsigned char> state_codec_registry::encode(const std::string &type_name,
                                                        const boost::any &value) const {
  encode_func fn;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _codecs.find(type_name);
    if (it == _codecs.end())
      throw std::runtime_error("state_codec_registry::encode: no codec for '" + type_name + "'");
    fn = it->second.encode;
  }
  return fn(value);
}

boost::any state_codec_registry::decode(const std::string &type_name,
                                        const std::vector<unsigned char> &bytes) const {
  decode_func fn;
  {
    std::lock_guard<std::mutex> lk(_mutex);
    auto it = _codecs.find(type_name);
    if (it == _codecs.end())
      throw std::runtime_error("state_codec_registry::decode: no codec for '" + type_name + "'");
    fn = it->second.decode;
  }
  return fn(bytes);
}

std::string state_codec_registry::codec_id_for(const std::string &type_name) const {
  std::lock_guard<std::mutex> lk(_mutex);
  auto it = _codecs.find(type_name);
  if (it == _codecs.end())
    return std::string();
  return it->second.codec_id;
}

std::size_t state_codec_registry::size() const {
  std::lock_guard<std::mutex> lk(_mutex);
  return _codecs.size();
}

void state_codec_registry::register_builtin_codecs() {
  register_codec(
      "bool",
      [](const boost::any &v) -> std::vector<unsigned char> {
        unsigned char b = boost::any_cast<bool>(v) ? 1u : 0u;
        return std::vector<unsigned char>{b};
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        if (b.empty())
          throw std::runtime_error("bool decode: empty");
        return boost::any(b[0] != 0);
      },
      "cvc.bool.v1");

  register_codec(
      "int32_t",
      [](const boost::any &v) {
        return encode_pod<std::int32_t>(boost::any_cast<std::int32_t>(v));
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<std::int32_t>(b));
      },
      "cvc.i32.v1");

  register_codec(
      "int64_t",
      [](const boost::any &v) {
        return encode_pod<std::int64_t>(boost::any_cast<std::int64_t>(v));
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<std::int64_t>(b));
      },
      "cvc.i64.v1");

  register_codec(
      "uint32_t",
      [](const boost::any &v) {
        return encode_pod<std::uint32_t>(boost::any_cast<std::uint32_t>(v));
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<std::uint32_t>(b));
      },
      "cvc.u32.v1");

  register_codec(
      "uint64_t",
      [](const boost::any &v) {
        return encode_pod<std::uint64_t>(boost::any_cast<std::uint64_t>(v));
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<std::uint64_t>(b));
      },
      "cvc.u64.v1");

  register_codec(
      "float", [](const boost::any &v) { return encode_pod<float>(boost::any_cast<float>(v)); },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<float>(b));
      },
      "cvc.f32.v1");

  register_codec(
      "double", [](const boost::any &v) { return encode_pod<double>(boost::any_cast<double>(v)); },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<double>(b));
      },
      "cvc.f64.v1");

  register_codec(
      "std::string",
      [](const boost::any &v) {
        const std::string &s = boost::any_cast<const std::string &>(v);
        return std::vector<unsigned char>(s.begin(), s.end());
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(std::string(b.begin(), b.end()));
      },
      "cvc.str.v1");

  // Integer data() under the app's REGISTERED spellings. state::dataTypeName() returns the name
  // app::registerDefaultTypes gave the type — "int", "unsigned int", "unsigned short", "int64",
  // "uint64" — NOT the C++ "_t" spellings the codecs above are keyed under. Without these aliases,
  // _codecs->encode()/decode() miss for an integer data(), the exception is swallowed in the sync
  // adapter, and the integer object silently fails to replicate (value() + structure still flow).
  // Each alias any_casts the EXACT stored type (boost::any_cast is exact-typeid, not conversion):
  // "int64"/"uint64" are boost::int64_t/uint64_t (cvc::int64/uint64 in core/types.h). float,
  // double, std::string and bool already match the app's spelling and need no alias.
  register_codec(
      "int", [](const boost::any &v) { return encode_pod<int>(boost::any_cast<int>(v)); },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<int>(b));
      },
      "cvc.int.v1");

  register_codec(
      "unsigned int",
      [](const boost::any &v) {
        return encode_pod<unsigned int>(boost::any_cast<unsigned int>(v));
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<unsigned int>(b));
      },
      "cvc.uint.v1");

  register_codec(
      "unsigned short",
      [](const boost::any &v) {
        return encode_pod<unsigned short>(boost::any_cast<unsigned short>(v));
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<unsigned short>(b));
      },
      "cvc.ushort.v1");

  register_codec(
      "int64",
      [](const boost::any &v) {
        return encode_pod<boost::int64_t>(boost::any_cast<boost::int64_t>(v));
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<boost::int64_t>(b));
      },
      "cvc.int64.v1");

  register_codec(
      "uint64",
      [](const boost::any &v) {
        return encode_pod<boost::uint64_t>(boost::any_cast<boost::uint64_t>(v));
      },
      [](const std::vector<unsigned char> &b) -> boost::any {
        return boost::any(decode_pod<boost::uint64_t>(b));
      },
      "cvc.uint64.v1");
}

} // namespace cvc
