/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
// cvc::archive (archive.h): tar, gzip, zstd and zip from memory. The deflate decoder follows
// RFC 1951 the way Mark Adler's puff does (canonical codes decoded by counting), with a 10-bit
// lookup table in front of it for the common short codes.

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <cvc/utility/archive.h>
#include <optional>
#include <stdexcept>
#include <zstd.h>

namespace cvc {
namespace archive {

namespace {

[[noreturn]] void fail(const std::string &what) { throw std::runtime_error("archive: " + what); }

const std::array<std::uint32_t, 256> &crc_table() {
  static const std::array<std::uint32_t, 256> table = [] {
    std::array<std::uint32_t, 256> t{};
    for (std::uint32_t i = 0; i < 256; ++i) {
      std::uint32_t c = i;
      for (int k = 0; k < 8; ++k)
        c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      t[i] = c;
    }
    return t;
  }();
  return table;
}

// ── byte sources ─────────────────────────────────────────────────────────────────────────────────

// A stream of bytes: read() fills up to n, fewer only at the end.
class source {
public:
  virtual ~source() = default;
  virtual std::size_t read(char *dst, std::size_t n) = 0;
};

class memory_source : public source {
public:
  explicit memory_source(std::string_view s) : s_(s) {}
  std::size_t read(char *dst, std::size_t n) override {
    const std::size_t k = std::min(n, s_.size() - pos_);
    std::memcpy(dst, s_.data() + pos_, k);
    pos_ += k;
    return k;
  }

private:
  std::string_view s_;
  std::size_t pos_ = 0;
};

// A canonical Huffman code (RFC 1951 3.2.2): `count[len]` codes of each length and the symbols
// in code order, plus a table indexed by the next kFast bits (least significant first) holding
// sym | len << 9 for the codes no longer than that, 0 for a longer one.
struct huffman {
  static constexpr int kMaxBits = 15;
  static constexpr int kFast = 10;
  std::array<std::uint16_t, kMaxBits + 1> count{};
  std::vector<std::uint16_t> symbol;
  std::vector<std::uint16_t> fast;

  void build(const std::uint8_t *lengths, int n) {
    count.fill(0);
    for (int i = 0; i < n; ++i)
      ++count[lengths[i]];
    count[0] = 0;
    int left = 1;
    for (int len = 1; len <= kMaxBits; ++len) {
      left = (left << 1) - count[len];
      if (left < 0)
        fail("corrupt deflate data (an over-subscribed code)");
    }
    std::array<std::uint16_t, kMaxBits + 2> offs{};
    for (int len = 1; len <= kMaxBits; ++len)
      offs[len + 1] = static_cast<std::uint16_t>(offs[len] + count[len]);
    symbol.assign(static_cast<std::size_t>(n), 0);
    for (int sym = 0; sym < n; ++sym)
      if (lengths[sym])
        symbol[offs[lengths[sym]]++] = static_cast<std::uint16_t>(sym);
    std::array<int, kMaxBits + 1> next{};
    int code = 0;
    for (int len = 1; len <= kMaxBits; ++len) {
      code = (code + count[len - 1]) << 1;
      next[len] = code;
    }
    fast.assign(std::size_t(1) << kFast, 0);
    for (int sym = 0; sym < n; ++sym) {
      const int len = lengths[sym];
      if (!len)
        continue;
      const int c = next[len]++;
      if (len > kFast)
        continue;
      int rev = 0;
      for (int b = 0; b < len; ++b)
        rev |= ((c >> b) & 1) << (len - 1 - b);
      for (int k = rev; k < (1 << kFast); k += 1 << len)
        fast[static_cast<std::size_t>(k)] = static_cast<std::uint16_t>(sym | (len << 9));
    }
  }
};

// Bits of a deflate stream, least significant first, over the whole input in memory.
class bit_reader {
public:
  bit_reader(const unsigned char *p, const unsigned char *end) : p_(p), end_(end) {}

  void fill() {
    while (cnt_ <= 56 && p_ < end_) {
      buf_ |= std::uint64_t(*p_++) << cnt_;
      cnt_ += 8;
    }
  }
  std::uint32_t bits(int n) {
    if (cnt_ < n) {
      fill();
      if (cnt_ < n)
        fail("truncated deflate data");
    }
    const auto v = static_cast<std::uint32_t>(buf_ & ((std::uint64_t(1) << n) - 1));
    buf_ >>= n;
    cnt_ -= n;
    return v;
  }
  int decode(const huffman &h) {
    if (cnt_ < huffman::kMaxBits)
      fill();
    const std::uint16_t e = h.fast[static_cast<std::size_t>(buf_ & ((1u << huffman::kFast) - 1))];
    if (e) {
      const int len = e >> 9;
      if (len > cnt_)
        fail("truncated deflate data");
      buf_ >>= len;
      cnt_ -= len;
      return e & 0x1FF;
    }
    int code = 0, first = 0, index = 0;
    for (int len = 1; len <= huffman::kMaxBits; ++len) {
      if (len > cnt_)
        fail("truncated deflate data");
      code |= static_cast<int>((buf_ >> (len - 1)) & 1u);
      const int c = h.count[static_cast<std::size_t>(len)];
      if (code - c < first) {
        buf_ >>= len;
        cnt_ -= len;
        return h.symbol[static_cast<std::size_t>(index + (code - first))];
      }
      index += c;
      first = (first + c) << 1;
      code <<= 1;
    }
    fail("corrupt deflate data (an invalid code)");
  }
  void align() {
    const int r = cnt_ & 7;
    buf_ >>= r;
    cnt_ -= r;
  }
  // Whole bytes, once aligned: the buffered ones first, then the input's.
  void bytes(char *dst, std::size_t n) {
    while (n && cnt_ >= 8) {
      *dst++ = static_cast<char>(bits(8));
      --n;
    }
    if (static_cast<std::size_t>(end_ - p_) < n)
      fail("truncated deflate data");
    std::memcpy(dst, p_, n);
    p_ += n;
  }
  std::size_t remaining() const { return static_cast<std::size_t>(cnt_ / 8) + (end_ - p_); }
  int peek_byte(std::size_t i) { // the i-th byte still unread, once aligned (-1 past the end)
    const std::size_t held = static_cast<std::size_t>(cnt_ / 8);
    if (i < held)
      return static_cast<int>((buf_ >> (8 * i)) & 0xFF);
    i -= held;
    return i < static_cast<std::size_t>(end_ - p_) ? p_[i] : -1;
  }

private:
  const unsigned char *p_, *end_;
  std::uint64_t buf_ = 0;
  int cnt_ = 0;
};

const std::uint16_t kLenBase[29] = {3,  4,  5,  6,  7,  8,  9,  10, 11,  13,  15,  17,  19,  23, 27,
                                    31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
const std::uint8_t kLenExtra[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                                    2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
const std::uint16_t kDistBase[30] = {1,    2,    3,    4,    5,    7,    9,    13,    17,    25,
                                     33,   49,   65,   97,   129,  193,  257,  385,   513,   769,
                                     1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
const std::uint8_t kDistExtra[30] = {0, 0, 0, 0, 1, 1, 2, 2,  3,  3,  4,  4,  5,  5,  6,
                                     6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

// The fixed codes (RFC 1951 3.2.6), built into a decoder's own `lit` and `dist` at its first
// fixed block (a few microseconds), so nothing is shared between decoders.
void build_fixed(huffman &lit, huffman &dist) {
  std::uint8_t l[288];
  std::fill(l, l + 144, 8);
  std::fill(l + 144, l + 256, 9);
  std::fill(l + 256, l + 280, 7);
  std::fill(l + 280, l + 288, 8);
  lit.build(l, 288);
  std::fill(l, l + 30, 5);
  dist.build(l, 30);
}

// A deflate stream (raw, as zip holds it, or gzip's members) decoded a block at a time into a
// window that keeps the last 32 KiB a back-reference can reach.
class inflate_source : public source {
public:
  inflate_source(std::string_view in, bool gzip, std::uint64_t max_bytes)
      : in_(reinterpret_cast<const unsigned char *>(in.data()),
            reinterpret_cast<const unsigned char *>(in.data()) + in.size()),
        gzip_(gzip), header_(gzip), max_(max_bytes) {}

  std::size_t read(char *dst, std::size_t n) override {
    std::size_t got = 0;
    while (got < n) {
      if (pos_ == out_.size()) {
        if (done_)
          break;
        step();
        continue;
      }
      const std::size_t k = std::min(n - got, out_.size() - pos_);
      std::memcpy(dst + got, out_.data() + pos_, k);
      pos_ += k;
      got += k;
    }
    if (pos_ > (std::size_t(4) << 20)) { // keep the window, drop what was read before it
      const std::size_t cut = pos_ - 32768;
      out_.erase(0, cut);
      pos_ -= cut;
      crc_pos_ -= cut;
    }
    return got;
  }

private:
  void step() {
    if (header_) {
      gzip_header();
      header_ = false;
    }
    const bool last = in_.bits(1) != 0;
    switch (in_.bits(2)) {
    case 0:
      stored();
      break;
    case 1:
      if (fixed_lit_.symbol.empty())
        build_fixed(fixed_lit_, fixed_dist_);
      codes(fixed_lit_, fixed_dist_);
      break;
    case 2:
      dynamic();
      break;
    default:
      fail("corrupt deflate data (block type 3)");
    }
    crc_ = crc32(std::string_view(out_.data() + crc_pos_, out_.size() - crc_pos_), crc_);
    crc_pos_ = out_.size();
    if (!last)
      return;
    if (!gzip_) {
      done_ = true;
      return;
    }
    in_.align();
    unsigned char t[8];
    in_.bytes(reinterpret_cast<char *>(t), 8);
    const std::uint32_t want_crc = t[0] | t[1] << 8 | t[2] << 16 | std::uint32_t(t[3]) << 24;
    const std::uint32_t want_size = t[4] | t[5] << 8 | t[6] << 16 | std::uint32_t(t[7]) << 24;
    if (want_crc != crc_)
      fail("gzip data fails its CRC check");
    if (want_size != static_cast<std::uint32_t>(member_bytes_))
      fail("gzip data has the wrong length");
    // Another member may follow (concatenated gzip files); anything else ends the stream.
    if (in_.peek_byte(0) == 0x1F && in_.peek_byte(1) == 0x8B) {
      header_ = true;
      crc_ = 0;
      member_bytes_ = 0;
    } else {
      done_ = true;
    }
  }

  void gzip_header() {
    unsigned char h[10];
    in_.bytes(reinterpret_cast<char *>(h), 10);
    if (h[0] != 0x1F || h[1] != 0x8B)
      fail("not gzip data");
    if (h[2] != 8)
      fail("gzip data with an unknown compression method");
    const unsigned flags = h[3];
    if (flags & 0xE0)
      fail("gzip data with reserved flags set");
    char b[2];
    if (flags & 4) { // FEXTRA
      in_.bytes(b, 2);
      std::size_t xlen = static_cast<unsigned char>(b[0]) | static_cast<unsigned char>(b[1]) << 8;
      std::string skip(xlen, '\0');
      in_.bytes(skip.data(), xlen);
    }
    for (unsigned bit : {8u, 16u}) // FNAME, FCOMMENT: zero-terminated
      if (flags & bit)
        do
          in_.bytes(b, 1);
        while (b[0] != 0);
    if (flags & 2) // FHCRC
      in_.bytes(b, 2);
  }

  void produce(std::size_t n) {
    member_bytes_ += n;
    total_ += n;
    if (total_ > max_)
      fail("the unpacked data passes its size cap (" + std::to_string(max_) + " bytes)");
  }

  void stored() {
    in_.align();
    const std::uint32_t len = in_.bits(16);
    const std::uint32_t nlen = in_.bits(16);
    if ((len ^ 0xFFFFu) != nlen)
      fail("corrupt deflate data (a stored block's length)");
    produce(len);
    const std::size_t at = out_.size();
    out_.resize(at + len);
    in_.bytes(out_.data() + at, len);
  }

  void dynamic() {
    static const std::uint8_t order[19] = {16, 17, 18, 0, 8,  7, 9,  6, 10, 5,
                                           11, 4,  12, 3, 13, 2, 14, 1, 15};
    const int nlen = static_cast<int>(in_.bits(5)) + 257;
    const int ndist = static_cast<int>(in_.bits(5)) + 1;
    const int ncode = static_cast<int>(in_.bits(4)) + 4;
    if (nlen > 286 || ndist > 30)
      fail("corrupt deflate data (too many codes)");
    std::uint8_t lengths[320] = {};
    for (int i = 0; i < ncode; ++i)
      lengths[order[i]] = static_cast<std::uint8_t>(in_.bits(3));
    huffman lencode;
    lencode.build(lengths, 19);
    std::fill(lengths, lengths + 320, 0);
    int index = 0;
    while (index < nlen + ndist) {
      const int sym = in_.decode(lencode);
      if (sym < 16) {
        lengths[index++] = static_cast<std::uint8_t>(sym);
        continue;
      }
      std::uint8_t val = 0;
      int rep;
      if (sym == 16) {
        if (index == 0)
          fail("corrupt deflate data (a repeat with nothing before it)");
        val = lengths[index - 1];
        rep = 3 + static_cast<int>(in_.bits(2));
      } else if (sym == 17) {
        rep = 3 + static_cast<int>(in_.bits(3));
      } else {
        rep = 11 + static_cast<int>(in_.bits(7));
      }
      if (index + rep > nlen + ndist)
        fail("corrupt deflate data (too many lengths)");
      std::fill(lengths + index, lengths + index + rep, val);
      index += rep;
    }
    if (lengths[256] == 0)
      fail("corrupt deflate data (no end-of-block code)");
    huffman lit, dist;
    lit.build(lengths, nlen);
    dist.build(lengths + nlen, ndist);
    codes(lit, dist);
  }

  void codes(const huffman &lit, const huffman &dist) {
    for (;;) {
      int sym = in_.decode(lit);
      if (sym < 256) {
        produce(1);
        out_.push_back(static_cast<char>(sym));
        continue;
      }
      if (sym == 256)
        return;
      sym -= 257;
      if (sym >= 29)
        fail("corrupt deflate data (a bad length code)");
      const std::size_t len = kLenBase[sym] + in_.bits(kLenExtra[sym]);
      const int dsym = in_.decode(dist);
      if (dsym >= 30)
        fail("corrupt deflate data (a bad distance code)");
      const std::size_t d = kDistBase[dsym] + in_.bits(kDistExtra[dsym]);
      if (d > out_.size())
        fail("corrupt deflate data (a distance too far back)");
      produce(len);
      const std::size_t at = out_.size();
      out_.resize(at + len);
      char *o = out_.data();
      for (std::size_t i = 0; i < len; ++i) // byte by byte: a copy may overlap itself
        o[at + i] = o[at - d + i];
    }
  }

  bit_reader in_;
  bool gzip_, header_, done_ = false;
  std::uint64_t max_, total_ = 0, member_bytes_ = 0;
  std::string out_;
  std::size_t pos_ = 0, crc_pos_ = 0;
  std::uint32_t crc_ = 0;
  huffman fixed_lit_, fixed_dist_; // build_fixed's, at the first fixed block
};

class zstd_source : public source {
public:
  zstd_source(std::string_view in, std::uint64_t max_bytes)
      : ctx_(ZSTD_createDCtx()), buf_(ZSTD_DStreamOutSize(), '\0'), max_(max_bytes) {
    if (!ctx_)
      fail("out of memory for zstd");
    in_ = {in.data(), in.size(), 0};
  }
  ~zstd_source() override { ZSTD_freeDCtx(ctx_); }
  zstd_source(const zstd_source &) = delete;
  zstd_source &operator=(const zstd_source &) = delete;

  std::size_t read(char *dst, std::size_t n) override {
    std::size_t got = 0;
    while (got < n) {
      if (pos_ == len_) {
        if (!refill())
          break;
        continue;
      }
      const std::size_t k = std::min(n - got, len_ - pos_);
      std::memcpy(dst + got, buf_.data() + pos_, k);
      pos_ += k;
      got += k;
    }
    return got;
  }

private:
  bool refill() {
    for (;;) {
      if (in_.pos == in_.size && frame_done_)
        return false;
      ZSTD_outBuffer out = {buf_.data(), buf_.size(), 0};
      const std::size_t r = ZSTD_decompressStream(ctx_, &out, &in_);
      if (ZSTD_isError(r))
        fail(std::string("corrupt zstd data (") + ZSTD_getErrorName(r) + ")");
      frame_done_ = r == 0;
      if (out.pos) {
        total_ += out.pos;
        if (total_ > max_)
          fail("the unpacked data passes its size cap (" + std::to_string(max_) + " bytes)");
        pos_ = 0;
        len_ = out.pos;
        return true;
      }
      if (in_.pos == in_.size && !frame_done_)
        fail("truncated zstd data");
    }
  }

  ZSTD_DCtx *ctx_;
  ZSTD_inBuffer in_{};
  std::string buf_;
  std::size_t pos_ = 0, len_ = 0;
  bool frame_done_ = false;
  std::uint64_t max_, total_ = 0;
};

std::size_t read_full(source &s, char *dst, std::size_t n) {
  std::size_t got = 0;
  while (got < n) {
    const std::size_t k = s.read(dst + got, n - got);
    if (!k)
      break;
    got += k;
  }
  return got;
}

void skip(source &s, std::uint64_t n) {
  char scratch[16384];
  while (n) {
    const std::size_t want = static_cast<std::size_t>(std::min<std::uint64_t>(n, sizeof scratch));
    if (read_full(s, scratch, want) != want)
      fail("truncated tar data");
    n -= want;
  }
}

std::unique_ptr<source> stream_of(std::string_view bytes, format f, std::uint64_t max_bytes) {
  if (f == format::gzip)
    return std::make_unique<inflate_source>(bytes, true, max_bytes);
  if (f == format::zstd)
    return std::make_unique<zstd_source>(bytes, max_bytes);
  return std::make_unique<memory_source>(bytes);
}

// ── tar ──────────────────────────────────────────────────────────────────────────────────────────

std::string field(const char *h, std::size_t at, std::size_t len) {
  const char *p = h + at;
  return std::string(p, strnlen(p, len));
}

std::uint64_t parse_number(const char *p, std::size_t len) {
  const auto *u = reinterpret_cast<const unsigned char *>(p);
  if (u[0] & 0x80) { // GNU base-256
    std::uint64_t v = u[0] & 0x7F;
    for (std::size_t i = 1; i < len; ++i)
      v = (v << 8) | u[i];
    return v;
  }
  std::uint64_t v = 0;
  std::size_t i = 0;
  while (i < len && (p[i] == ' ' || p[i] == '\0'))
    ++i;
  for (; i < len && p[i] >= '0' && p[i] <= '7'; ++i)
    v = (v << 3) | static_cast<std::uint64_t>(p[i] - '0');
  return v;
}

bool tar_checksum_ok(const char *h) {
  const auto *u = reinterpret_cast<const unsigned char *>(h);
  long unsigned_sum = 0, signed_sum = 0;
  for (int i = 0; i < 512; ++i) {
    const bool in_field = i >= 148 && i < 156;
    unsigned_sum += in_field ? 0x20 : u[i];
    signed_sum += in_field ? 0x20 : static_cast<signed char>(h[i]);
  }
  const auto want = static_cast<long>(parse_number(h + 148, 8));
  return want == unsigned_sum || want == signed_sum;
}

bool looks_like_tar(std::string_view first) {
  if (first.size() < 512)
    return false;
  const char *h = first.data();
  if (std::all_of(h, h + 512, [](char c) { return c == 0; }))
    return true; // an empty archive: its end blocks
  return tar_checksum_ok(h);
}

void parse_pax(const std::string &data, std::string &path, std::optional<std::uint64_t> &size) {
  std::size_t pos = 0;
  while (pos < data.size()) {
    const std::size_t sp = data.find(' ', pos);
    if (sp == std::string::npos)
      break;
    std::size_t len = 0;
    for (std::size_t i = pos; i < sp; ++i) {
      if (data[i] < '0' || data[i] > '9')
        fail("a corrupt pax header");
      len = len * 10 + static_cast<std::size_t>(data[i] - '0');
    }
    if (len == 0 || pos + len > data.size())
      fail("a corrupt pax header");
    const std::string rec = data.substr(sp + 1, pos + len - sp - 1); // "key=value\n"
    const std::size_t eq = rec.find('=');
    if (eq != std::string::npos) {
      const std::string key = rec.substr(0, eq);
      std::string value = rec.substr(eq + 1);
      if (!value.empty() && value.back() == '\n')
        value.pop_back();
      if (key == "path")
        path = value;
      else if (key == "size")
        size = std::stoull(value);
    }
    pos += len;
  }
}

std::vector<file> read_tar(source &src, const limits &lim,
                           const std::function<bool(const std::string &)> &keep) {
  std::vector<file> files;
  std::uint64_t total = 0;
  std::string long_name, pax_path;
  std::optional<std::uint64_t> pax_size;
  char h[512];
  for (;;) {
    const std::size_t got = read_full(src, h, 512);
    if (got == 0)
      break; // no end blocks: accepted, as tar does
    if (got < 512)
      fail("truncated tar header");
    if (std::all_of(h, h + 512, [](char c) { return c == 0; }))
      break; // the end
    if (!tar_checksum_ok(h))
      fail("a tar header fails its checksum");
    std::uint64_t size = pax_size ? *pax_size : parse_number(h + 124, 12);
    const char type = h[156];
    const std::uint64_t pad = (512 - size % 512) % 512;
    if (type == 'L' || type == 'x') { // the next member's long name / pax attributes
      if (size > (1u << 20))
        fail("a tar long-name or pax record over 1 MiB");
      std::string data(static_cast<std::size_t>(size), '\0');
      if (read_full(src, data.data(), data.size()) != data.size())
        fail("truncated tar data");
      skip(src, pad);
      if (type == 'L')
        long_name = std::string(data.c_str());
      else
        parse_pax(data, pax_path, pax_size);
      continue;
    }
    std::string name;
    if (!pax_path.empty())
      name = pax_path;
    else if (!long_name.empty())
      name = long_name;
    else {
      name = field(h, 0, 100);
      const std::string prefix = std::memcmp(h + 257, "ustar", 5) == 0 ? field(h, 345, 155) : "";
      if (!prefix.empty())
        name = prefix + "/" + name;
    }
    long_name.clear();
    pax_path.clear();
    pax_size.reset();
    const bool regular = type == '0' || type == '\0' || type == '7';
    std::string path;
    if (regular) {
      path = safe_path(name);
      if (path.empty())
        fail("an unsafe path in the archive: \"" + name + "\"");
    }
    if (!regular || (keep && !keep(path))) { // directories, links, devices, global pax, ...
      skip(src, size + pad);
      continue;
    }
    if (files.size() >= lim.max_entries)
      fail("the archive has more than " + std::to_string(lim.max_entries) + " files");
    total += size;
    if (total > lim.max_bytes)
      fail("the unpacked files pass their size cap (" + std::to_string(lim.max_bytes) + " bytes)");
    auto bytes = std::make_shared<std::string>(static_cast<std::size_t>(size), '\0');
    if (read_full(src, bytes->data(), bytes->size()) != bytes->size())
      fail("truncated tar data");
    skip(src, pad);
    files.push_back({std::move(path), std::move(bytes)});
  }
  return files;
}

// ── zip ──────────────────────────────────────────────────────────────────────────────────────────

class zip_reader {
public:
  explicit zip_reader(std::string_view z) : z_(z) {}

  std::vector<file> read(const limits &lim, const std::function<bool(const std::string &)> &keep) {
    if (z_.size() < 22)
      fail("a truncated zip");
    std::size_t eocd = std::string::npos;
    const std::size_t lowest = z_.size() > 22 + 65535 ? z_.size() - 22 - 65535 : 0;
    for (std::size_t i = z_.size() - 22 + 1; i-- > lowest;)
      if (u32(i) == 0x06054b50u) {
        eocd = i;
        break;
      }
    if (eocd == std::string::npos)
      fail("a zip with no end-of-central-directory record");
    std::uint64_t entries = u16(eocd + 10);
    std::uint64_t cd = u32(eocd + 16);
    if ((entries == 0xFFFF || cd == 0xFFFFFFFFu) && eocd >= 20 && u32(eocd - 20) == 0x07064b50u) {
      const std::uint64_t z64 = u64(eocd - 20 + 8); // the zip64 end record
      if (u32(z64) != 0x06064b50u)
        fail("a corrupt zip64 end record");
      entries = u64(z64 + 32);
      cd = u64(z64 + 48);
    }
    std::vector<file> files;
    std::uint64_t total = 0;
    std::uint64_t p = cd;
    for (std::uint64_t e = 0; e < entries; ++e) {
      if (u32(p) != 0x02014b50u)
        fail("a corrupt zip central directory");
      const unsigned flags = u16(p + 8);
      const unsigned method = u16(p + 10);
      const std::uint32_t crc = u32(p + 16);
      std::uint64_t csize = u32(p + 20), usize = u32(p + 24);
      const std::size_t nlen = u16(p + 28), xlen = u16(p + 30), clen = u16(p + 32);
      const std::uint32_t attrs = u32(p + 38);
      std::uint64_t local = u32(p + 42);
      const std::string name(span(p + 46, nlen));
      zip64_sizes(p + 46 + nlen, xlen, usize, csize, local);
      p += 46 + nlen + xlen + clen;
      if (!name.empty() && (name.back() == '/' || name.back() == '\\'))
        continue; // a directory
      if (((attrs >> 16) & 0xF000u) == 0xA000u)
        continue; // a symbolic link
      const std::string path = safe_path(name);
      if (path.empty())
        fail("an unsafe path in the archive: \"" + name + "\"");
      if (keep && !keep(path))
        continue;
      if (flags & 1u)
        fail("an encrypted zip entry: " + path);
      if (files.size() >= lim.max_entries)
        fail("the archive has more than " + std::to_string(lim.max_entries) + " files");
      total += usize;
      if (total > lim.max_bytes)
        fail("the unpacked files pass their size cap (" + std::to_string(lim.max_bytes) +
             " bytes)");
      if (u32(local) != 0x04034b50u)
        fail("a corrupt zip local header: " + path);
      const std::uint64_t data = local + 30 + u16(local + 26) + u16(local + 28);
      const std::string_view packed = span(data, csize);
      std::string bytes;
      if (method == 0) {
        if (csize != usize)
          fail("a stored zip entry with two sizes: " + path);
        bytes.assign(packed);
      } else if (method == 8) {
        inflate_source s(packed, false, usize);
        bytes.resize(static_cast<std::size_t>(usize));
        if (read_full(s, bytes.data(), bytes.size()) != bytes.size())
          fail("a zip entry shorter than its size: " + path);
      } else {
        fail("zip compression method " + std::to_string(method) +
             " (only stored and deflate): " + path);
      }
      if (crc32(bytes) != crc)
        fail("a zip entry fails its CRC check: " + path);
      files.push_back({path, std::make_shared<const std::string>(std::move(bytes))});
    }
    return files;
  }

private:
  std::string_view span(std::uint64_t at, std::uint64_t n) const {
    if (at > z_.size() || n > z_.size() - at)
      fail("a truncated zip");
    return z_.substr(static_cast<std::size_t>(at), static_cast<std::size_t>(n));
  }
  std::uint32_t u16(std::uint64_t at) const {
    const auto s = span(at, 2);
    return static_cast<unsigned char>(s[0]) | static_cast<unsigned char>(s[1]) << 8;
  }
  std::uint32_t u32(std::uint64_t at) const { return u16(at) | u16(at + 2) << 16; }
  std::uint64_t u64(std::uint64_t at) const { return u32(at) | std::uint64_t(u32(at + 4)) << 32; }
  // The zip64 extra field (id 1) holds, in order, the sizes and offset whose 32-bit fields are
  // all ones.
  void zip64_sizes(std::uint64_t at, std::size_t len, std::uint64_t &usize, std::uint64_t &csize,
                   std::uint64_t &local) const {
    std::uint64_t p = at;
    while (p + 4 <= at + len) {
      const unsigned id = u16(p), n = u16(p + 2);
      if (id == 1) {
        std::uint64_t q = p + 4;
        for (std::uint64_t *v : {&usize, &csize, &local})
          if (*v == 0xFFFFFFFFu && q + 8 <= p + 4 + n) {
            *v = u64(q);
            q += 8;
          }
        return;
      }
      p += 4 + n;
    }
  }

  std::string_view z_;
};

} // namespace

// ── the public calls ─────────────────────────────────────────────────────────────────────────────

std::uint32_t crc32(std::string_view bytes, std::uint32_t crc) {
  const auto &t = crc_table();
  crc = ~crc;
  for (unsigned char b : bytes)
    crc = t[(crc ^ b) & 0xFFu] ^ (crc >> 8);
  return ~crc;
}

format sniff(std::string_view b) {
  const auto at = [&](std::size_t i) { return static_cast<unsigned char>(b[i]); };
  if (b.size() >= 2 && at(0) == 0x1F && at(1) == 0x8B)
    return format::gzip;
  if (b.size() >= 4 && at(0) == 0x28 && at(1) == 0xB5 && at(2) == 0x2F && at(3) == 0xFD)
    return format::zstd;
  if (b.size() >= 4 && b[0] == 'P' && b[1] == 'K' &&
      ((at(2) == 3 && at(3) == 4) || (at(2) == 5 && at(3) == 6)))
    return format::zip;
  if (looks_like_tar(b))
    return format::tar;
  return format::none;
}

const char *format_name(format f) {
  switch (f) {
  case format::gzip:
    return "gzip";
  case format::zstd:
    return "zstd";
  case format::tar:
    return "tar";
  case format::zip:
    return "zip";
  default:
    return "none";
  }
}

kind classify(std::string_view bytes) {
  const format f = sniff(bytes);
  if (f == format::tar)
    return kind::tar;
  if (f == format::zip)
    return kind::zip;
  if (f != format::gzip && f != format::zstd)
    return kind::plain;
  std::string first(512, '\0');
  try {
    auto s = stream_of(bytes, f, limits{}.max_bytes);
    first.resize(read_full(*s, first.data(), first.size()));
  } catch (const std::exception &) {
    return kind::compressed; // unpack/decompress report what is wrong with it
  }
  return looks_like_tar(first) ? kind::tar : kind::compressed;
}

std::vector<file> unpack(std::string_view bytes, const limits &lim,
                         const std::function<bool(const std::string &)> &keep) {
  switch (classify(bytes)) {
  case kind::tar: {
    auto s = stream_of(bytes, sniff(bytes), lim.max_bytes + 1024 * 1024);
    return read_tar(*s, lim, keep);
  }
  case kind::zip:
    return zip_reader(bytes).read(lim, keep);
  case kind::compressed:
    fail(std::string("a single ") + format_name(sniff(bytes)) + " file, not an archive");
  default:
    fail("not an archive (tar, gzip or zstd tar, or zip)");
  }
}

std::string decompress(std::string_view bytes, std::uint64_t max_bytes) {
  const format f = sniff(bytes);
  if (f != format::gzip && f != format::zstd)
    fail("not gzip or zstd data");
  auto s = stream_of(bytes, f, max_bytes);
  std::string out;
  char chunk[65536];
  for (std::size_t k; (k = s->read(chunk, sizeof chunk)) != 0;)
    out.append(chunk, k);
  return out;
}

std::string safe_path(std::string_view path) {
  std::string s(path);
  std::replace(s.begin(), s.end(), '\\', '/');
  if (!s.empty() && s[0] == '/')
    return "";
  std::string out;
  std::size_t pos = 0;
  bool first = true;
  while (pos <= s.size()) {
    std::size_t end = s.find('/', pos);
    if (end == std::string::npos)
      end = s.size();
    const std::string comp = s.substr(pos, end - pos);
    pos = end + 1;
    if (comp.empty() || comp == ".")
      continue;
    if (comp == "..")
      return "";
    if (first && comp.size() >= 2 && comp[1] == ':')
      return ""; // a drive: C:, C:foo
    first = false;
    if (!out.empty())
      out += '/';
    out += comp;
  }
  return out;
}

std::string pack_tar(const std::vector<std::pair<std::string, std::string>> &files) {
  std::string out;
  const auto header = [&](const std::string &name, std::uint64_t size, char type) {
    char h[512] = {};
    std::memcpy(h, name.data(), std::min<std::size_t>(name.size(), 100));
    std::memcpy(h + 100, "0000644", 7);
    std::memcpy(h + 108, "0000000", 7);
    std::memcpy(h + 116, "0000000", 7);
    std::snprintf(h + 124, 12, "%011llo", static_cast<unsigned long long>(size));
    std::memcpy(h + 136, "00000000000", 11);
    h[156] = type;
    std::memcpy(h + 257, "ustar", 6);
    std::memcpy(h + 263, "00", 2);
    std::memset(h + 148, ' ', 8);
    unsigned sum = 0;
    for (char c : h)
      sum += static_cast<unsigned char>(c);
    std::snprintf(h + 148, 8, "%06o", sum);
    h[155] = ' ';
    out.append(h, 512);
  };
  const auto body = [&](const std::string &data) {
    out += data;
    out.append((512 - data.size() % 512) % 512, '\0');
  };
  for (const auto &[path, data] : files) {
    if (path.size() > 100) {
      header("././@LongLink", path.size() + 1, 'L');
      body(path + '\0');
    }
    header(path, data.size(), '0');
    body(data);
  }
  out.append(1024, '\0');
  return out;
}

std::string gzip(std::string_view bytes) {
  std::string out("\x1F\x8B\x08\x00\x00\x00\x00\x00\x00\xFF", 10);
  std::size_t pos = 0;
  do {
    const std::size_t n = std::min<std::size_t>(bytes.size() - pos, 65535);
    const bool last = pos + n == bytes.size();
    out += static_cast<char>(last ? 1 : 0); // BFINAL, BTYPE 00 (stored), then byte-aligned
    out += static_cast<char>(n & 0xFF);
    out += static_cast<char>(n >> 8);
    out += static_cast<char>(~n & 0xFF);
    out += static_cast<char>((~n >> 8) & 0xFF);
    out.append(bytes.substr(pos, n));
    pos += n;
  } while (pos < bytes.size());
  const std::uint32_t crc = crc32(bytes);
  const auto size = static_cast<std::uint32_t>(bytes.size());
  for (std::uint32_t v : {crc, size})
    for (int i = 0; i < 4; ++i)
      out += static_cast<char>((v >> (8 * i)) & 0xFF);
  return out;
}

std::string zstd(std::string_view bytes, int level) {
  std::string out(ZSTD_compressBound(bytes.size()), '\0');
  const std::size_t n = ZSTD_compress(out.data(), out.size(), bytes.data(), bytes.size(), level);
  if (ZSTD_isError(n))
    fail(std::string("zstd: ") + ZSTD_getErrorName(n));
  out.resize(n);
  return out;
}

} // namespace archive
} // namespace cvc
