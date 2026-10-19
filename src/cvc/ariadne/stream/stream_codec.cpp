/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <boost/any.hpp>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream_codec.h>
#include <cvc/core/app.h>
#include <cvc/state/state_codec_registry.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {
namespace stream {

const char *const kFormatDescTypeName = "cvc::ariadne::stream::format_desc";

namespace {

// Little-endian, fixed-width framing so the content is byte-identical across
// platforms (a prerequisite for content-addressed blob offload of large
// descriptors and for digests to agree between the live and snapshot paths).
void put_u8(std::vector<unsigned char> &o, std::uint8_t v) { o.push_back(v); }
void put_u32(std::vector<unsigned char> &o, std::uint32_t v) {
  for (int i = 0; i < 4; ++i)
    o.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}
void put_u64(std::vector<unsigned char> &o, std::uint64_t v) {
  for (int i = 0; i < 8; ++i)
    o.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}
void put_i32(std::vector<unsigned char> &o, std::int32_t v) {
  put_u32(o, static_cast<std::uint32_t>(v));
}
void put_str(std::vector<unsigned char> &o, const std::string &s) {
  put_u32(o, static_cast<std::uint32_t>(s.size()));
  o.insert(o.end(), s.begin(), s.end());
}

struct reader {
  const unsigned char *p;
  const unsigned char *e;
  bool ok = true;
  explicit reader(const std::vector<unsigned char> &b) : p(b.data()), e(b.data() + b.size()) {}
  std::uint8_t u8() {
    if (p + 1 > e) {
      ok = false;
      return 0;
    }
    return *p++;
  }
  std::uint32_t u32() {
    if (p + 4 > e) {
      ok = false;
      return 0;
    }
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i)
      v |= static_cast<std::uint32_t>(p[i]) << (8 * i);
    p += 4;
    return v;
  }
  std::uint64_t u64() {
    if (p + 8 > e) {
      ok = false;
      return 0;
    }
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
      v |= static_cast<std::uint64_t>(p[i]) << (8 * i);
    p += 8;
    return v;
  }
  std::int32_t i32() { return static_cast<std::int32_t>(u32()); }
  std::string str() {
    std::uint32_t n = u32();
    if (!ok || p + n > e) {
      ok = false;
      return {};
    }
    std::string s(reinterpret_cast<const char *>(p), n);
    p += n;
    return s;
  }
};

std::vector<unsigned char> encode_format_desc(const boost::any &v) {
  const format_desc d = boost::any_cast<format_desc>(v);
  std::vector<unsigned char> o;
  put_u8(o, static_cast<std::uint8_t>(d.kind));
  put_str(o, d.codec);
  put_i32(o, d.w);
  put_i32(o, d.h);
  put_i32(o, d.stride);
  put_u8(o, static_cast<std::uint8_t>(d.origin));
  put_i32(o, d.sample_rate);
  put_i32(o, d.channels);
  put_u64(o, static_cast<std::uint64_t>(d.bytes));
  put_str(o, d.extra); // opaque; carried verbatim
  return o;
}

boost::any decode_format_desc(const std::vector<unsigned char> &b) {
  reader r(b);
  format_desc d;
  d.kind = static_cast<frame_kind>(r.u8());
  d.codec = r.str();
  d.w = r.i32();
  d.h = r.i32();
  d.stride = r.i32();
  d.origin = static_cast<frame_origin>(r.u8());
  d.sample_rate = r.i32();
  d.channels = r.i32();
  d.bytes = static_cast<std::size_t>(r.u64());
  d.extra = r.str();
  if (!r.ok)
    throw std::runtime_error("cvc::ariadne::stream::format_desc decode: truncated payload");
  return boost::any(d);
}

} // namespace

void register_stream_data_types(app &ctx) {
  ctx.registerDataType<format_desc>(kFormatDescTypeName);
}

void register_stream_codecs(state_codec_registry &registry) {
  registry.register_codec(kFormatDescTypeName, encode_format_desc, decode_format_desc,
                          "cvc.ariadne.stream.format_desc.v1");
}

} // namespace stream
} // namespace ariadne
} // namespace cvc
