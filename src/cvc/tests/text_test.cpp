// Unit tests for cvc::text — UTF-8 codepoint iteration + display width (§17).
// Byte sequences are written with \x escapes so the test is independent of the
// source file's encoding.

#include <cvc/core/text.h>

#include <gtest/gtest.h>

#include <string>

using namespace cvc;

namespace {
// Handy UTF-8 byte strings.
const std::string kCafe = "caf\xC3\xA9";                         // "café" (é = U+00E9)
const std::string kNihongo = "\xE6\x97\xA5\xE6\x9C\xAC\xE8\xAA\x9E"; // "日本語"
const std::string kEuro = "\xE2\x82\xAC";                        // "€" (U+20AC)
const std::string kGrin = "\xF0\x9F\x98\x80";                    // "😀" (U+1F600)
const std::string kCombining = "e\xCC\x81";                      // "e" + U+0301 combining acute
} // namespace

TEST(CvcText, DecodeAscii) {
  std::size_t p = 0;
  EXPECT_EQ(text::decode("A", p), U'A');
  EXPECT_EQ(p, 1u);
}

TEST(CvcText, DecodeMultibyte) {
  std::size_t p = 0;
  EXPECT_EQ(text::decode(kEuro, p), 0x20ACu); // 3-byte
  EXPECT_EQ(p, 3u);
  p = 0;
  EXPECT_EQ(text::decode(kGrin, p), 0x1F600u); // 4-byte
  EXPECT_EQ(p, 4u);
}

TEST(CvcText, DecodeMalformedYieldsReplacementAndAdvancesOne) {
  // A lone continuation byte.
  std::string bad = "\x80";
  std::size_t p = 0;
  EXPECT_EQ(text::decode(bad, p), text::kReplacement);
  EXPECT_EQ(p, 1u);
  // A truncated 2-byte sequence at end of string.
  std::string trunc = "\xC3";
  p = 0;
  EXPECT_EQ(text::decode(trunc, p), text::kReplacement);
  EXPECT_EQ(p, 1u);
  // An overlong encoding of '/' (0x2F) must be rejected.
  std::string overlong = "\xC0\xAF";
  p = 0;
  EXPECT_EQ(text::decode(overlong, p), text::kReplacement);
  EXPECT_EQ(p, 1u);
}

TEST(CvcText, DecodeLoopTerminatesOnGarbage) {
  std::string garbage = "\xFF\xFE\x80\x80";
  std::size_t p = 0, iters = 0;
  while (p < garbage.size()) {
    text::decode(garbage, p);
    ASSERT_LT(++iters, 100u); // must terminate
  }
  EXPECT_EQ(p, garbage.size());
}

TEST(CvcText, EncodeRoundTrip) {
  for (char32_t cp : {U'A', char32_t(0x00E9), char32_t(0x20AC), char32_t(0x1F600)}) {
    std::string s;
    text::encode(cp, s);
    std::size_t p = 0;
    EXPECT_EQ(text::decode(s, p), cp);
    EXPECT_EQ(p, s.size());
  }
  // A surrogate encodes as U+FFFD, not a broken sequence.
  std::string s;
  text::encode(0xD800, s);
  std::size_t p = 0;
  EXPECT_EQ(text::decode(s, p), text::kReplacement);
}

TEST(CvcText, CodepointWidth) {
  EXPECT_EQ(text::codepoint_width(U'A'), 1);
  EXPECT_EQ(text::codepoint_width(0x00E9), 1); // é
  EXPECT_EQ(text::codepoint_width(0x20AC), 1); // €
  EXPECT_EQ(text::codepoint_width(0x65E5), 2); // 日 (CJK)
  EXPECT_EQ(text::codepoint_width(0x1F600), 2); // 😀
  EXPECT_EQ(text::codepoint_width(0x0301), 0); // combining acute
  EXPECT_EQ(text::codepoint_width(0x200B), 0); // zero-width space
  EXPECT_EQ(text::codepoint_width(0x09), 0);   // tab (control)
}

TEST(CvcText, DisplayWidthAndCount) {
  EXPECT_EQ(text::display_width(kCafe), 4u);    // c a f é
  EXPECT_EQ(text::codepoint_count(kCafe), 4u);
  EXPECT_EQ(kCafe.size(), 5u);                  // é is 2 bytes

  EXPECT_EQ(text::display_width(kNihongo), 6u); // 3 wide glyphs
  EXPECT_EQ(text::codepoint_count(kNihongo), 3u);

  EXPECT_EQ(text::display_width("a" + kGrin + "b"), 4u); // 1 + 2 + 1

  EXPECT_EQ(text::display_width(kCombining), 1u); // e + combining = one column
  EXPECT_EQ(text::codepoint_count(kCombining), 2u);
}

TEST(CvcText, IsValidUtf8) {
  EXPECT_TRUE(text::is_valid_utf8(kCafe));
  EXPECT_TRUE(text::is_valid_utf8(kNihongo));
  EXPECT_TRUE(text::is_valid_utf8(""));
  EXPECT_FALSE(text::is_valid_utf8("\x80"));       // lone continuation
  EXPECT_FALSE(text::is_valid_utf8("a\xFF"));      // invalid lead
  EXPECT_FALSE(text::is_valid_utf8("\xE6\x97"));   // truncated 3-byte
  EXPECT_TRUE(text::is_valid_utf8("\xEF\xBF\xBD")); // a genuine U+FFFD is valid
}

TEST(CvcText, CodepointBoundaries) {
  // kCafe = c a f <é:2 bytes>, size 5. next from 0 → 1; prev from 5 → 3 (start of é).
  EXPECT_EQ(text::next_codepoint(kCafe, 0), 1u);
  EXPECT_EQ(text::next_codepoint(kCafe, 3), 5u); // skips the whole é
  EXPECT_EQ(text::prev_codepoint(kCafe, 5), 3u); // back to start of é
  EXPECT_EQ(text::prev_codepoint(kCafe, 4), 3u); // mid-é snaps back to its start
  EXPECT_EQ(text::prev_codepoint(kCafe, 0), 0u);
  EXPECT_EQ(text::next_codepoint(kCafe, 5), 5u);
}

TEST(CvcText, TruncateToWidth) {
  EXPECT_EQ(text::truncate_to_width("hello", 10), "hello"); // fits
  EXPECT_EQ(text::truncate_to_width("hello", 3), "hel");
  EXPECT_EQ(text::truncate_to_width("hello", 4, "\xE2\x80\xA6"), "hel\xE2\x80\xA6"); // "hel…"
  // A wide glyph is never split: budget 3 for "日本語" (each width 2) fits only one.
  EXPECT_EQ(text::display_width(text::truncate_to_width(kNihongo, 3)), 2u);
  // Never emit a partial UTF-8 sequence.
  EXPECT_TRUE(text::is_valid_utf8(text::truncate_to_width(kNihongo, 3)));
}

TEST(CvcText, PadToWidth) {
  EXPECT_EQ(text::pad_to_width("hi", 5), "hi   ");
  EXPECT_EQ(text::pad_to_width("hello", 3), "hello"); // already wider
  // Padding accounts for display width, not bytes: kCafe is 5 bytes / 4 columns.
  EXPECT_EQ(text::display_width(text::pad_to_width(kCafe, 6)), 6u);
}
