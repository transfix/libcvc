// Terminal pixel graphics (term_graphics.h): the Kitty and Sixel encoders, the probe-reply parser
// and the rectangle cover. Pure functions, so no terminal is needed.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/ariadne/term_graphics.h>
#include <gtest/gtest.h>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace gfx = cvc::ariadne::gfx;
using cvc::ariadne::TermGraphics;

namespace {

std::string utf8(std::uint32_t cp) {
  std::string s;
  if (cp < 0x80) {
    s += static_cast<char>(cp);
  } else if (cp < 0x800) {
    s += static_cast<char>(0xC0 | (cp >> 6));
    s += static_cast<char>(0x80 | (cp & 0x3F));
  } else if (cp < 0x10000) {
    s += static_cast<char>(0xE0 | (cp >> 12));
    s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    s += static_cast<char>(0x80 | (cp & 0x3F));
  } else {
    s += static_cast<char>(0xF0 | (cp >> 18));
    s += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
    s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    s += static_cast<char>(0x80 | (cp & 0x3F));
  }
  return s;
}

std::vector<unsigned char> base64_decode(const std::string &s) {
  auto val = [](char c) -> int {
    if (c >= 'A' && c <= 'Z')
      return c - 'A';
    if (c >= 'a' && c <= 'z')
      return c - 'a' + 26;
    if (c >= '0' && c <= '9')
      return c - '0' + 52;
    if (c == '+')
      return 62;
    if (c == '/')
      return 63;
    return -1;
  };
  std::vector<unsigned char> out;
  std::uint32_t acc = 0;
  int bits = 0;
  for (const char c : s) {
    const int v = val(c);
    if (v < 0)
      continue; // '=' padding
    acc = (acc << 6) | static_cast<std::uint32_t>(v);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<unsigned char>((acc >> bits) & 0xFF));
    }
  }
  return out;
}

// One APC escape of a kitty transmission: its control keys and its payload.
struct KittyChunk {
  std::map<std::string, std::string> keys;
  std::string payload;
};

std::vector<KittyChunk> split_kitty(const std::string &s) {
  std::vector<KittyChunk> out;
  std::size_t pos = 0;
  while ((pos = s.find("\x1b_G", pos)) != std::string::npos) {
    const std::size_t end = s.find("\x1b\\", pos);
    if (end == std::string::npos)
      break;
    const std::string body = s.substr(pos + 3, end - pos - 3);
    const std::size_t semi = body.find(';');
    KittyChunk c;
    const std::string keys = body.substr(0, semi);
    std::size_t k = 0;
    while (k < keys.size()) {
      std::size_t comma = keys.find(',', k);
      if (comma == std::string::npos)
        comma = keys.size();
      const std::string kv = keys.substr(k, comma - k);
      const std::size_t eq = kv.find('=');
      c.keys[kv.substr(0, eq)] = eq == std::string::npos ? "" : kv.substr(eq + 1);
      k = comma + 1;
    }
    if (semi != std::string::npos)
      c.payload = body.substr(semi + 1);
    out.push_back(std::move(c));
    pos = end + 2;
  }
  return out;
}

// A small Sixel decoder: the image as palette colours (r, g, b in percent), -1 where unpainted.
struct Decoded {
  int w = 0, h = 0;    // raster attributes
  int painted_h = 0;   // rows actually reached
  std::vector<int> px; // register index per pixel, -1 = unpainted
  std::map<int, std::array<int, 3>> palette;
  bool ok = false;
};

Decoded decode_sixel(const std::string &s) {
  Decoded d;
  const std::size_t q = s.find('q');
  if (s.compare(0, 2, "\x1bP") != 0 || q == std::string::npos)
    return d;
  std::size_t i = q + 1;
  auto number = [&]() {
    int v = 0;
    while (i < s.size() && s[i] >= '0' && s[i] <= '9')
      v = v * 10 + (s[i++] - '0');
    return v;
  };
  if (i < s.size() && s[i] == '"') {
    ++i;
    std::vector<int> ra;
    ra.push_back(number());
    while (i < s.size() && s[i] == ';') {
      ++i;
      ra.push_back(number());
    }
    if (ra.size() == 4) {
      d.w = ra[2];
      d.h = ra[3];
    }
  }
  d.px.assign(static_cast<std::size_t>(d.w) * d.h, -1);
  int x = 0, y = 0, color = 0;
  while (i < s.size()) {
    const char c = s[i];
    if (c == '\x1b') {
      d.ok = s.compare(i, 2, "\x1b\\") == 0 && i + 2 == s.size();
      break;
    }
    if (c == '#') {
      ++i;
      const int reg = number();
      if (i < s.size() && s[i] == ';') {
        std::vector<int> v;
        while (i < s.size() && s[i] == ';') {
          ++i;
          v.push_back(number());
        }
        if (v.size() == 4 && v[0] == 2)
          d.palette[reg] = {v[1], v[2], v[3]};
      }
      color = reg;
      continue;
    }
    if (c == '$') {
      x = 0;
      ++i;
      continue;
    }
    if (c == '-') {
      x = 0;
      y += 6;
      ++i;
      continue;
    }
    int repeat = 1;
    if (c == '!') {
      ++i;
      repeat = number();
    }
    const char six = s[i++];
    if (six < 63 || six > 126)
      return d; // malformed
    const int bits = six - 63;
    for (int r = 0; r < repeat; ++r, ++x)
      for (int b = 0; b < 6; ++b)
        if (bits & (1 << b)) {
          const int yy = y + b;
          if (x < d.w && yy < d.h)
            d.px[static_cast<std::size_t>(yy) * d.w + x] = color;
          d.painted_h = std::max(d.painted_h, yy + 1);
        }
  }
  return d;
}

std::vector<unsigned char> random_rgb(int w, int h, unsigned seed) {
  std::mt19937 rng(seed);
  std::vector<unsigned char> v(static_cast<std::size_t>(w) * h * 3);
  for (unsigned char &b : v)
    b = static_cast<unsigned char>(rng() & 0xFF);
  return v;
}

} // namespace

// ---- base64 / kitty -------------------------------------------------------------------------

TEST(TermGraphics, Base64MatchesRfc4648) {
  auto enc = [](const std::string &s) {
    return gfx::base64_encode(reinterpret_cast<const unsigned char *>(s.data()), s.size());
  };
  EXPECT_EQ(enc(""), "");
  EXPECT_EQ(enc("f"), "Zg==");
  EXPECT_EQ(enc("fo"), "Zm8=");
  EXPECT_EQ(enc("foo"), "Zm9v");
  EXPECT_EQ(enc("foob"), "Zm9vYg==");
  EXPECT_EQ(enc("fooba"), "Zm9vYmE=");
  EXPECT_EQ(enc("foobar"), "Zm9vYmFy");
}

TEST(TermGraphics, KittyPlaceholderIsTheBaseCharacterWithRowAndColumnDiacritics) {
  EXPECT_EQ(gfx::kitty_diacritic(0), 0x0305u);
  EXPECT_EQ(gfx::kitty_diacritic(1), 0x030Du);
  EXPECT_EQ(gfx::kitty_diacritic(2), 0x030Eu);
  EXPECT_EQ(gfx::kitty_diacritic(gfx::kKittyMaxCells - 1), 0x1D244u);
  EXPECT_EQ(gfx::kitty_diacritic(gfx::kKittyMaxCells), 0u);
  EXPECT_EQ(gfx::kitty_diacritic(-1), 0u);

  EXPECT_EQ(gfx::kitty_placeholder(0, 0), utf8(0x10EEEE) + utf8(0x0305) + utf8(0x0305));
  EXPECT_EQ(gfx::kitty_placeholder(1, 2), utf8(0x10EEEE) + utf8(0x030D) + utf8(0x030E));
  EXPECT_EQ(gfx::kitty_placeholder(296, 0), utf8(0x10EEEE) + utf8(0x1D244) + utf8(0x0305));
  EXPECT_EQ(gfx::kitty_placeholder(297, 0), "");
  EXPECT_EQ(gfx::kitty_placeholder(0, -1), "");

  // Every diacritic is distinct (a row number must not read as another).
  std::map<std::uint32_t, int> seen;
  for (int i = 0; i < gfx::kKittyMaxCells; ++i)
    EXPECT_EQ(seen[gfx::kitty_diacritic(i)]++, 0) << i;
}

TEST(TermGraphics, KittyTransmitChunksTheBase64PayloadAndCarriesThePlacement) {
  const int w = 37, h = 23;
  const std::vector<unsigned char> rgb = random_rgb(w, h, 7);
  const std::string s = gfx::kitty_transmit(rgb.data(), w, h, 0, 4242, 12, 5, 1000);
  const std::vector<KittyChunk> chunks = split_kitty(s);
  ASSERT_GE(chunks.size(), 2u); // 37 * 23 * 3 = 2553 bytes -> 3404 base64 chars, 1000 per chunk
  EXPECT_EQ(chunks.size(), 4u);

  const KittyChunk &first = chunks.front();
  EXPECT_EQ(first.keys.at("a"), "T"); // transmit and place
  EXPECT_EQ(first.keys.at("U"), "1"); // a virtual placement, for Unicode placeholders
  EXPECT_EQ(first.keys.at("f"), "24");
  EXPECT_EQ(first.keys.at("s"), "37");
  EXPECT_EQ(first.keys.at("v"), "23");
  EXPECT_EQ(first.keys.at("c"), "12");
  EXPECT_EQ(first.keys.at("r"), "5");
  EXPECT_EQ(first.keys.at("i"), "4242");
  EXPECT_EQ(first.keys.at("q"), "2"); // no replies: they would land in the input stream

  std::string payload;
  for (std::size_t k = 0; k < chunks.size(); ++k) {
    const KittyChunk &c = chunks[k];
    EXPECT_EQ(c.keys.at("m"), k + 1 < chunks.size() ? "1" : "0") << k;
    if (k > 0) {
      // Continuation chunks carry only m (and q).
      EXPECT_EQ(c.keys.size(), 2u) << k;
      EXPECT_EQ(c.keys.count("a"), 0u);
    }
    EXPECT_LE(c.payload.size(), 1000u);
    EXPECT_EQ(c.payload.size() % 4, 0u); // each chunk is whole base64 quanta
    payload += c.payload;
  }
  EXPECT_EQ(base64_decode(payload), rgb);
}

TEST(TermGraphics, KittyTransmitHonoursTheRowStride) {
  // A 3 x 2 window into a 5 x 4 image: only the window's bytes are sent.
  const int W = 5, H = 4;
  const std::vector<unsigned char> img = random_rgb(W, H, 11);
  const unsigned char *origin = img.data() + (1 * W + 2) * 3; // window at (2, 1)
  const std::string s = gfx::kitty_transmit(origin, 3, 2, W * 3, 9, 1, 1);
  std::string payload;
  for (const KittyChunk &c : split_kitty(s))
    payload += c.payload;
  std::vector<unsigned char> want;
  for (int y = 1; y < 3; ++y)
    for (int x = 2; x < 5; ++x)
      for (int c = 0; c < 3; ++c)
        want.push_back(img[static_cast<std::size_t>((y * W + x) * 3 + c)]);
  EXPECT_EQ(base64_decode(payload), want);
}

TEST(TermGraphics, KittyDeleteFreesTheImageData) {
  EXPECT_EQ(gfx::kitty_delete(77), "\x1b_Ga=d,d=I,i=77,q=2\x1b\\");
}

// ---- probing --------------------------------------------------------------------------------

TEST(TermGraphics, ProbeQueryAsksKittyCellSizeThenDa1) {
  const std::string q = gfx::probe_query(31);
  EXPECT_EQ(q.find("\x1b_Gi=31,"), 0u);
  EXPECT_NE(q.find("a=q"), std::string::npos);
  EXPECT_NE(q.find("\x1b[16t"), std::string::npos);
  EXPECT_EQ(q.substr(q.size() - 3), "\x1b[c"); // DA1 last: its reply ends the exchange
}

TEST(TermGraphics, ParseProbeReplyReadsKittyCellSizeAndSixel) {
  TermGraphics g;
  // kitty: graphics OK, a 10 x 21 px cell, DA1 without Sixel.
  EXPECT_TRUE(gfx::parse_probe_reply("\x1b_Gi=31;OK\x1b\\\x1b[6;21;10t\x1b[?62;c", 31, g));
  EXPECT_TRUE(g.kitty);
  EXPECT_FALSE(g.sixel);
  EXPECT_EQ(g.cell_w, 10);
  EXPECT_EQ(g.cell_h, 21);
  EXPECT_TRUE(g.answered);

  // xterm -ti vt340: no kitty reply, Sixel (4) in DA1.
  TermGraphics x;
  EXPECT_TRUE(gfx::parse_probe_reply("\x1b[6;16;8t\x1b[?63;1;2;4;6;9;15;22c", 31, x));
  EXPECT_FALSE(x.kitty);
  EXPECT_TRUE(x.sixel);
  EXPECT_EQ(x.cell_w, 8);

  // The colour register follow-up.
  EXPECT_TRUE(gfx::parse_probe_reply("\x1b[?1;0;1024S\x1b[?63;4c", 31, x));
  EXPECT_EQ(x.sixel_colors, 1024);
}

TEST(TermGraphics, ParseProbeReplyIgnoresOtherIdsErrorsAndTypedKeys) {
  TermGraphics g;
  // A reply for a different id, an error for ours, keys the user typed meanwhile.
  EXPECT_TRUE(gfx::parse_probe_reply(
      "ab\x1b_Gi=7;OK\x1b\\x\x1b_Gi=31;ENOTSUPPORTED:no\x1b\\\x1b[A\x1b[?1;2c", 31, g));
  EXPECT_FALSE(g.kitty);
  EXPECT_TRUE(g.answered);
  EXPECT_EQ(g.cell_w, 0);
}

TEST(TermGraphics, ParseProbeReplyIsNotDoneUntilDa1) {
  TermGraphics g;
  EXPECT_FALSE(gfx::parse_probe_reply("\x1b_Gi=31;OK\x1b\\\x1b[6;20", 31, g));
  EXPECT_TRUE(g.kitty);
  EXPECT_FALSE(g.answered);
  EXPECT_FALSE(gfx::parse_probe_reply("", 31, g));
  // A DA2 reply (CSI > ... c) is not DA1.
  TermGraphics h;
  EXPECT_FALSE(gfx::parse_probe_reply("\x1b[>1;4000;0c", 31, h));
  EXPECT_FALSE(h.answered);
}

// ---- sixel ----------------------------------------------------------------------------------

TEST(TermGraphics, SixelPaletteFitsTheRegisters) {
  EXPECT_EQ(gfx::sixel_palette(256).size(), 252);
  EXPECT_EQ(gfx::sixel_palette(1024).size(), 252);
  EXPECT_EQ(gfx::sixel_palette(251).size(), 216);
  EXPECT_EQ(gfx::sixel_palette(64).size(), 64);
  EXPECT_EQ(gfx::sixel_palette(16).size(), 16);
  EXPECT_EQ(gfx::sixel_palette(8).size(), 8);
  EXPECT_EQ(gfx::sixel_palette(0).size(), 8);
}

TEST(TermGraphics, SixelEncodesSolidColoursExactly) {
  // Pure primaries / black / white sit on the palette's cube corners, so no dither touches them.
  const int w = 13, h = 14; // 14 rows: two full bands and a partial one
  std::vector<unsigned char> rgb(static_cast<std::size_t>(w) * h * 3);
  const unsigned char kColours[4][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 255}};
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const unsigned char *c = kColours[(x / 4 + y / 5) % 4];
      for (int k = 0; k < 3; ++k)
        rgb[static_cast<std::size_t>((y * w + x) * 3 + k)] = c[k];
    }
  const std::string s = gfx::sixel_encode(rgb.data(), w, h, 0, 256);
  EXPECT_EQ(s.find("\x1bP0;1;0q\"1;1;13;14"), 0u); // 1:1 aspect, raster attributes
  const Decoded d = decode_sixel(s);
  ASSERT_TRUE(d.ok);
  EXPECT_EQ(d.w, w);
  EXPECT_EQ(d.h, h);
  EXPECT_EQ(d.palette.size(), 4u); // only the registers the image uses are defined
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const int reg = d.px[static_cast<std::size_t>(y) * w + x];
      ASSERT_GE(reg, 0) << x << "," << y;
      const std::array<int, 3> pc = d.palette.at(reg);
      const unsigned char *c = kColours[(x / 4 + y / 5) % 4];
      for (int k = 0; k < 3; ++k)
        EXPECT_EQ(pc[k], c[k] ? 100 : 0) << x << "," << y;
    }
  EXPECT_EQ(d.painted_h, h); // the partial last band paints no rows below the image
}

TEST(TermGraphics, SixelDithersBetweenTheNearestLevels) {
  // Any colour decodes, per channel, to one of the two cube levels bracketing it, and a 4x4 block
  // of one flat colour averages back near it.
  const int w = 32, h = 24;
  const std::vector<unsigned char> rgb = random_rgb(w, h, 3);
  for (const int colors : {256, 64, 16}) {
    const gfx::SixelPalette pal = gfx::sixel_palette(colors);
    const int lv[3] = {pal.r, pal.g, pal.b};
    const Decoded d = decode_sixel(gfx::sixel_encode(rgb.data(), w, h, 0, colors));
    ASSERT_TRUE(d.ok) << colors;
    EXPECT_LE(static_cast<int>(d.palette.size()), pal.size());
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const int reg = d.px[static_cast<std::size_t>(y) * w + x];
        ASSERT_GE(reg, 0);
        const std::array<int, 3> pc = d.palette.at(reg);
        for (int k = 0; k < 3; ++k) {
          const int v = rgb[static_cast<std::size_t>((y * w + x) * 3 + k)];
          const double step = 100.0 / (lv[k] - 1);
          const double pv = v * 100.0 / 255.0;
          EXPECT_LE(std::abs(pc[k] - pv), step + 1.0) << colors << " " << x << "," << y;
        }
      }
  }

  std::vector<unsigned char> flat(4 * 4 * 3);
  for (std::size_t i = 0; i < flat.size(); i += 3) {
    flat[i] = 100;
    flat[i + 1] = 150;
    flat[i + 2] = 200;
  }
  const Decoded f = decode_sixel(gfx::sixel_encode(flat.data(), 4, 4, 0, 256));
  ASSERT_TRUE(f.ok);
  double sum[3] = {0, 0, 0};
  for (const int reg : f.px)
    for (int k = 0; k < 3; ++k)
      sum[k] += f.palette.at(reg)[k] / 16.0;
  EXPECT_NEAR(sum[0], 100 * 100.0 / 255, 4.0);
  EXPECT_NEAR(sum[1], 150 * 100.0 / 255, 4.0);
  EXPECT_NEAR(sum[2], 200 * 100.0 / 255, 4.0);
}

TEST(TermGraphics, SixelRunLengthCodesFlatRuns) {
  std::vector<unsigned char> black(static_cast<std::size_t>(200) * 6 * 3, 0);
  const std::string s = gfx::sixel_encode(black.data(), 200, 6, 0, 256);
  EXPECT_NE(s.find("!200~"), std::string::npos); // one register, all six bits, 200 columns
  EXPECT_LT(s.size(), 60u);
}

TEST(TermGraphics, SixelHonoursTheRowStride) {
  const int W = 9, H = 8;
  const std::vector<unsigned char> img = random_rgb(W, H, 5);
  const unsigned char *origin = img.data() + (2 * W + 3) * 3;
  const std::string window = gfx::sixel_encode(origin, 4, 5, W * 3, 256);
  std::vector<unsigned char> packed;
  for (int y = 2; y < 7; ++y)
    for (int x = 3; x < 7; ++x)
      for (int c = 0; c < 3; ++c)
        packed.push_back(img[static_cast<std::size_t>((y * W + x) * 3 + c)]);
  // The same pixels at the same dither phase encode identically.
  EXPECT_EQ(window, gfx::sixel_encode(packed.data(), 4, 5, 0, 256));
}

// ---- rectangle cover ------------------------------------------------------------------------

TEST(TermGraphics, CoverRectsCoversTheMaskWithDisjointRectangles) {
  EXPECT_TRUE(gfx::cover_rects({}, 0, 0).empty());
  EXPECT_TRUE(gfx::cover_rects(std::vector<unsigned char>(12, 0), 4, 3).empty());
  const auto full = gfx::cover_rects(std::vector<unsigned char>(12, 1), 4, 3);
  ASSERT_EQ(full.size(), 1u);
  EXPECT_EQ(full[0].x, 0);
  EXPECT_EQ(full[0].w, 4);
  EXPECT_EQ(full[0].h, 3);

  // A window over the image's lower right: an L of visible cells, two rectangles. Column first:
  // the full-height left part, then the strip above the window.
  // 1111
  // 1100
  // 1100
  const std::vector<unsigned char> ell = {1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0};
  const auto r = gfx::cover_rects(ell, 4, 3);
  ASSERT_EQ(r.size(), 2u);
  EXPECT_EQ(r[0].x, 0);
  EXPECT_EQ(r[0].y, 0);
  EXPECT_EQ(r[0].w, 2);
  EXPECT_EQ(r[0].h, 3);
  EXPECT_EQ(r[1].x, 2);
  EXPECT_EQ(r[1].y, 0);
  EXPECT_EQ(r[1].w, 2);
  EXPECT_EQ(r[1].h, 1);

  std::mt19937 rng(1);
  for (int trial = 0; trial < 50; ++trial) {
    const int cols = 1 + static_cast<int>(rng() % 20), rows = 1 + static_cast<int>(rng() % 12);
    std::vector<unsigned char> mask(static_cast<std::size_t>(cols) * rows);
    for (unsigned char &m : mask)
      m = (rng() % 4) != 0;
    std::vector<int> hit(mask.size(), 0);
    for (const gfx::CellRect &c : gfx::cover_rects(mask, cols, rows))
      for (int y = c.y; y < c.y + c.h; ++y)
        for (int x = c.x; x < c.x + c.w; ++x)
          ++hit[static_cast<std::size_t>(y) * cols + x];
    for (std::size_t i = 0; i < mask.size(); ++i)
      EXPECT_EQ(hit[i], mask[i] ? 1 : 0) << trial << " cell " << i;
  }
}
