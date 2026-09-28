// SE-0 of the state_exec Unicode roadmap: unit-tests the shared UTF-8 codepoint helper
// (cvc/core/state_exec/utf8.h) AND pins the paths that are ALREADY Unicode-clean (byte-transparent
// storage): parse -> value_t -> to_string round-trip and the cvc::state snapshot codec. Those
// round-trips PASS today — this locks them so SE-1 (wiring the helper into the byte-oriented string
// builtins) cannot silently regress the transparent paths.
//
// Corpus strings are built from explicit \xNN byte escapes (not literal non-ASCII source), so the
// bytes are exact and portable and MSVC never emits its code-page-1252 C4566 warning.

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/state_value_codec.h>
#include <cvc/core/state_exec/types.h>
#include <cvc/core/state_exec/utf8.h>
#include <gtest/gtest.h>
#include <string>

using namespace cvc::state_exec;

// --- corpus (byte-exact UTF-8) ------------------------------------------------
// "cafe" + U+00E9 (e-acute, 2 bytes)
static const std::string kCafe = "caf\xC3\xA9";
// U+1F600 GRINNING FACE (4 bytes)
static const std::string kEmoji = "\xF0\x9F\x98\x80";
// U+4E2D U+6587 (CJK, 3 bytes each)
static const std::string kCjk = "\xE4\xB8\xAD\xE6\x96\x87";

// ===========================================================================
// utf8 helper: count
// ===========================================================================
TEST(StateExecUtf8, CountCountsCodepointsNotBytes) {
  EXPECT_EQ(utf8::count(""), 0u);
  EXPECT_EQ(utf8::count("abc"), 3u);
  EXPECT_EQ(utf8::count(kCafe), 4u);  // 5 bytes, 4 codepoints
  EXPECT_EQ(utf8::count(kEmoji), 1u); // 4 bytes, 1 codepoint
  EXPECT_EQ(utf8::count(kCjk), 2u);   // 6 bytes, 2 codepoints
  EXPECT_EQ(kCafe.size(), 5u);        // guard: the byte length really differs
}

// ===========================================================================
// utf8 helper: decode (each byte-length class)
// ===========================================================================
TEST(StateExecUtf8, DecodeAsciiTwoThreeFourByte) {
  char32_t cp;
  std::size_t len;
  ASSERT_TRUE(utf8::decode("A", 0, cp, len));
  EXPECT_EQ(cp, 0x41u);
  EXPECT_EQ(len, 1u);
  // the e-acute at byte offset 3 of "cafe\xC3\xA9"
  ASSERT_TRUE(utf8::decode(kCafe, 3, cp, len));
  EXPECT_EQ(cp, 0x00E9u);
  EXPECT_EQ(len, 2u);
  ASSERT_TRUE(utf8::decode(kCjk, 0, cp, len));
  EXPECT_EQ(cp, 0x4E2Du);
  EXPECT_EQ(len, 3u);
  ASSERT_TRUE(utf8::decode(kEmoji, 0, cp, len));
  EXPECT_EQ(cp, 0x1F600u);
  EXPECT_EQ(len, 4u);
}

// ===========================================================================
// utf8 helper: byte_offset (codepoint index -> byte offset)
// ===========================================================================
TEST(StateExecUtf8, ByteOffsetMapsCodepointIndex) {
  EXPECT_EQ(utf8::byte_offset(kCafe, 0), 0u);
  EXPECT_EQ(utf8::byte_offset(kCafe, 1), 1u);
  EXPECT_EQ(utf8::byte_offset(kCafe, 3), 3u);  // the e-acute starts at byte 3
  EXPECT_EQ(utf8::byte_offset(kCafe, 4), 5u);  // one past last cp -> end (byte length)
  EXPECT_EQ(utf8::byte_offset(kCafe, 99), 5u); // clamped to end
  EXPECT_EQ(utf8::byte_offset(kCjk, 1), 3u);   // second CJK char starts at byte 3
}

// ===========================================================================
// utf8 helper: encode / round-trip
// ===========================================================================
TEST(StateExecUtf8, EncodeRoundTripsEveryLengthClass) {
  for (char32_t want : {char32_t{0x41}, char32_t{0x00E9}, char32_t{0x4E2D}, char32_t{0x1F600}}) {
    std::string enc;
    utf8::encode(want, enc);
    char32_t got;
    std::size_t len;
    ASSERT_TRUE(utf8::decode(enc, 0, got, len)) << "cp=" << static_cast<uint32_t>(want);
    EXPECT_EQ(got, want);
    EXPECT_EQ(len, enc.size());
  }
  // exact byte encodings
  std::string e;
  utf8::encode(0x00E9, e);
  EXPECT_EQ(e, "\xC3\xA9");
  e.clear();
  utf8::encode(0x1F600, e);
  EXPECT_EQ(e, "\xF0\x9F\x98\x80");
  // out-of-range / surrogate -> U+FFFD
  e.clear();
  utf8::encode(0x110000, e); // > max
  EXPECT_EQ(e, "\xEF\xBF\xBD");
  e.clear();
  utf8::encode(0xD800, e); // surrogate half
  EXPECT_EQ(e, "\xEF\xBF\xBD");
}

// ===========================================================================
// utf8 helper: malformed input is rejected by is_valid but decoded LENIENTLY
// (one byte consumed, U+FFFD yielded) so iteration always progresses.
// ===========================================================================
TEST(StateExecUtf8, MalformedIsInvalidButDecodeProgresses) {
  const std::string lone_cont = "\x80";           // stray continuation byte
  const std::string truncated = "\xC3";           // 2-byte lead, no continuation
  const std::string overlong = "\xC0\x80";        // overlong NUL
  const std::string surrogate = "\xED\xA0\x80";   // U+D800 (UTF-16 surrogate)
  const std::string too_big = "\xF5\x80\x80\x80"; // > U+10FFFF
  for (const std::string *s : {&lone_cont, &truncated, &overlong, &surrogate, &too_big}) {
    EXPECT_FALSE(utf8::is_valid(*s));
    char32_t cp;
    std::size_t len;
    EXPECT_FALSE(utf8::decode(*s, 0, cp, len));
    EXPECT_EQ(cp, utf8::kReplacement);
    EXPECT_EQ(len, 1u); // advanced exactly one byte
  }
  EXPECT_TRUE(utf8::is_valid(kCafe));
  EXPECT_TRUE(utf8::is_valid(kEmoji));
  EXPECT_TRUE(utf8::is_valid(kCjk));
  EXPECT_TRUE(utf8::is_valid("plain ascii"));
  EXPECT_TRUE(utf8::is_valid(""));
  // a bad byte in the middle does not stall count(): a + <bad> + b = 3 codepoints
  EXPECT_EQ(utf8::count("a\x80"
                        "b"),
            3u);
}

// ===========================================================================
// TRANSPARENT PATH 1: the parser preserves UTF-8 string-literal bytes verbatim.
// ===========================================================================
TEST(StateExecUnicodeRoundTrip, ParserPreservesUtf8Literals) {
  EXPECT_EQ(std::get<std::string>(parse("\"" + kCafe + "\"").v), kCafe);
  EXPECT_EQ(std::get<std::string>(parse("\"" + kEmoji + "\"").v), kEmoji);
  EXPECT_EQ(std::get<std::string>(parse("\"" + kCjk + "\"").v), kCjk);
}

// ===========================================================================
// TRANSPARENT PATH 2: to_string emits UTF-8 string bytes unchanged (the bytes
// appear verbatim in the rendering), so parse -> to_string -> parse round-trips.
// ===========================================================================
TEST(StateExecUnicodeRoundTrip, ToStringEmitsUtf8BytesVerbatim) {
  for (const std::string &s : {kCafe, kEmoji, kCjk}) {
    const std::string rendered = to_string(value_t(s));
    EXPECT_NE(rendered.find(s), std::string::npos) << "to_string dropped the UTF-8 bytes";
    // parse(to_string(literal)) recovers the exact bytes
    EXPECT_EQ(std::get<std::string>(parse(to_string(value_t(s))).v), s);
  }
}

// ===========================================================================
// TRANSPARENT PATH 3: the cvc::state snapshot codec round-trips arbitrary UTF-8.
// ===========================================================================
class Utf8CodecTest : public ::testing::Test {
protected:
  cvc::app ctx;
  int counter_ = 0;
  value_t roundtrip(const value_t &val) {
    auto &node = ctx.root()("__utf8_codec_" + std::to_string(counter_++) + "__");
    encode_value(node, val);
    return decode_value(node);
  }
};

TEST_F(Utf8CodecTest, CodecRoundTripsUtf8Strings) {
  for (const std::string &s : {kCafe, kEmoji, kCjk}) {
    auto r = roundtrip(value_t(s));
    ASSERT_TRUE(std::holds_alternative<std::string>(r.v));
    EXPECT_EQ(std::get<std::string>(r.v), s);
  }
}
