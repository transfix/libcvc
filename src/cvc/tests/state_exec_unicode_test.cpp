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
#include <cvc/core/state_exec/builtins.h> // SE-1: builtin_length (A6)
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/state_value_codec.h>
#include <cvc/core/state_exec/stdlib.h> // SE-1: string.* module builtins (A1-A5)
#include <cvc/core/state_exec/types.h>
#include <cvc/core/state_exec/utf8.h>
#include <gtest/gtest.h>
#include <span>
#include <string>
#include <vector>

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

// ===========================================================================
// SE-1 — Track A: the string builtins are now codepoint-correct (A1-A6).
// ASCII behavior is unchanged (codepoint == byte for ASCII); the non-ASCII
// cases below FAIL under the old byte-oriented implementations.
// ===========================================================================
namespace {
// Call a registered string.* module builtin directly via the stdlib registry.
value_t call_str(stdlib_registry &reg, const char *fn, std::vector<value_t> args) {
  const native_fn *f = reg.lookup_qualified(fn);
  EXPECT_NE(f, nullptr) << fn << " not registered";
  if (!f)
    return nil_value;
  return (*f)(std::span<const value_t>(args.data(), args.size()));
}
value_t vi(int64_t i) { return value_t(i); }
} // namespace

TEST(StateExecUnicodeBuiltins, LengthCountsCodepointsAndByteLengthCountsBytes) {
  stdlib_registry reg;
  auto len = [&](const std::string &s) {
    return std::get<int64_t>(call_str(reg, "string.length", {value_t(s)}).v);
  };
  EXPECT_EQ(len("hello"), 5); // ASCII: unchanged
  EXPECT_EQ(len(kCafe), 4);   // was 5 (bytes) before SE-1
  EXPECT_EQ(len(kEmoji), 1);
  EXPECT_EQ(len(kCjk), 2);
  // raw byte size still available
  EXPECT_EQ(std::get<int64_t>(call_str(reg, "string.byte-length", {value_t(kCafe)}).v), 5);
  EXPECT_EQ(std::get<int64_t>(call_str(reg, "string.byte-length", {value_t(kEmoji)}).v), 4);
}

TEST(StateExecUnicodeBuiltins, CharAtReturnsWholeCodepoint) {
  stdlib_registry reg;
  auto at = [&](const std::string &s, int64_t i) {
    return std::get<std::string>(call_str(reg, "string.char-at", {value_t(s), vi(i)}).v);
  };
  EXPECT_EQ(at("hello", 1), "e"); // ASCII: unchanged
  EXPECT_EQ(at(kCafe, 0), "c");
  EXPECT_EQ(at(kCafe, 3), "\xC3\xA9");    // the whole e-acute, not a partial byte
  EXPECT_EQ(at(kEmoji, 0), kEmoji);       // the whole 4-byte emoji
  EXPECT_EQ(at(kCjk, 1), "\xE6\x96\x87"); // second CJK char
  // index past the codepoint count throws
  EXPECT_THROW(call_str(reg, "string.char-at", {value_t(kCafe), vi(4)}), std::runtime_error);
  EXPECT_THROW(call_str(reg, "string.char-at", {value_t(kEmoji), vi(1)}), std::runtime_error);
}

TEST(StateExecUnicodeBuiltins, SubstringSlicesByCodepoint) {
  stdlib_registry reg;
  auto sub = [&](const std::string &s, int64_t p, int64_t l) {
    return std::get<std::string>(call_str(reg, "string.substring", {value_t(s), vi(p), vi(l)}).v);
  };
  EXPECT_EQ(sub("hello", 1, 3), "ell");    // ASCII: unchanged
  EXPECT_EQ(sub(kCafe, 3, 1), "\xC3\xA9"); // just the e-acute
  EXPECT_EQ(sub(kCafe, 0, 3), "caf");
  EXPECT_EQ(sub(kCjk, 1, 1), "\xE6\x96\x87");
  // pos past the codepoint count -> ""
  EXPECT_EQ(std::get<std::string>(call_str(reg, "string.substring", {value_t(kCafe), vi(10)}).v),
            "");
  // no-length form: from a codepoint index to the end
  EXPECT_EQ(std::get<std::string>(call_str(reg, "string.substring", {value_t(kCafe), vi(3)}).v),
            "\xC3\xA9");
}

TEST(StateExecUnicodeBuiltins, SplitEmptyDelimYieldsWholeCodepoints) {
  stdlib_registry reg;
  auto parts = call_str(reg, "string.split", {value_t(kCafe), value_t(std::string())});
  auto &lst = *std::get<list_ptr>(parts.v);
  ASSERT_EQ(lst.size(), 4u); // c a f é — not 5 bytes
  EXPECT_EQ(std::get<std::string>(lst[0].v), "c");
  EXPECT_EQ(std::get<std::string>(lst[3].v), "\xC3\xA9");
  auto emoji_parts = call_str(reg, "string.split", {value_t(kEmoji), value_t(std::string())});
  ASSERT_EQ(std::get<list_ptr>(emoji_parts.v)->size(), 1u); // one 4-byte codepoint, not 4
}

TEST(StateExecUnicodeBuiltins, UpperLowerFoldAsciiOnlyAndLeaveUtf8Intact) {
  stdlib_registry reg;
  // ASCII folds; the multibyte e-acute is left byte-for-byte untouched (no corruption / no UB).
  EXPECT_EQ(std::get<std::string>(call_str(reg, "string.upper", {value_t(kCafe)}).v),
            "CAF\xC3\xA9");
  EXPECT_EQ(
      std::get<std::string>(call_str(reg, "string.lower", {value_t(std::string("CAF\xC3\xA9"))}).v),
      kCafe);
  // a pure-multibyte string is returned unchanged by both
  EXPECT_EQ(std::get<std::string>(call_str(reg, "string.upper", {value_t(kCjk)}).v), kCjk);
  EXPECT_EQ(std::get<std::string>(call_str(reg, "string.lower", {value_t(kEmoji)}).v), kEmoji);
}

TEST(StateExecUnicodeBuiltins, CoreLengthBuiltinCountsCodepointsForStrings) {
  // the core (length ...) builtin (A6), not the string module
  std::vector<value_t> a{value_t(kCafe)};
  EXPECT_EQ(std::get<int64_t>(builtin_length(std::span<const value_t>(a.data(), a.size())).v), 4);
  std::vector<value_t> ascii{value_t(std::string("hello"))};
  EXPECT_EQ(
      std::get<int64_t>(builtin_length(std::span<const value_t>(ascii.data(), ascii.size())).v), 5);
}

// ===========================================================================
// SE-2 — Track A parser: \u/\x/\u{} escapes decode to UTF-8, and identifiers
// may contain non-ASCII (UTF-8) bytes (A7/A8/A11).
// ===========================================================================
TEST(StateExecUnicodeParser, UnicodeEscapesDecodeToUtf8) {
  // \uXXXX (4 hex) -> codepoint -> UTF-8 bytes
  EXPECT_EQ(std::get<std::string>(parse("\"\\u00e9\"").v), "\xC3\xA9"); // é
  EXPECT_EQ(std::get<std::string>(parse("\"caf\\u00e9\"").v), kCafe);
  // \u{H..H} braces, astral plane
  EXPECT_EQ(std::get<std::string>(parse("\"\\u{1f600}\"").v), kEmoji); // 😀
  EXPECT_EQ(std::get<std::string>(parse("\"\\u{4e2d}\\u{6587}\"").v), kCjk);
  // \xNN -> codepoint U+00NN (text semantics), encoded UTF-8
  EXPECT_EQ(std::get<std::string>(parse("\"\\x41\"").v), "A");        // U+0041
  EXPECT_EQ(std::get<std::string>(parse("\"\\xe9\"").v), "\xC3\xA9"); // U+00E9 -> UTF-8
  // \0 NUL
  EXPECT_EQ(std::get<std::string>(parse("\"\\x00\"").v), std::string(1, '\0'));
  // a surrogate codepoint is replaced with U+FFFD by utf8::encode
  EXPECT_EQ(std::get<std::string>(parse("\"\\ud800\"").v), "\xEF\xBF\xBD");
}

TEST(StateExecUnicodeParser, MalformedEscapesError) {
  EXPECT_THROW(parse("\"\\u00zz\""), parse_error); // non-hex
  EXPECT_THROW(parse("\"\\u12\""), parse_error);   // too few hex digits
  EXPECT_THROW(parse("\"\\u{}\""), parse_error);   // empty braces
  EXPECT_THROW(parse("\"\\x1\""), parse_error);    // \x needs 2 hex
}

TEST(StateExecUnicodeParser, IdentifiersMayContainUtf8) {
  // a symbol whose name carries UTF-8 parses whole (was truncated at the first high byte before).
  // to_string(symbol) emits the name verbatim, so it round-trips the exact bytes.
  EXPECT_EQ(to_string(parse(kCafe)), kCafe);
  EXPECT_EQ(to_string(parse("na\xC3\xAFve")), "na\xC3\xAFve"); // naïve (ï = U+00EF)
  EXPECT_EQ(to_string(parse(kCjk)), kCjk);
}

// ===========================================================================
// SE-3 — Track B: the `bytes` type (opaque octets, byte-semantic), distinct
// from the (text, codepoint-semantic) `string`.
// ===========================================================================
TEST(StateExecBytes, ParserBytesLiteralAndByteEscapes) {
  // b"..." is a bytes value; \xNN is a RAW byte (not a codepoint like in a text string).
  auto v = parse("b\"caf\\xc3\\xa9\"");
  ASSERT_TRUE(std::holds_alternative<bytes_value>(v.v));
  EXPECT_EQ(std::get<bytes_value>(v.v).data, kCafe); // c a f 0xC3 0xA9
  // contrast: in a TEXT string, \xe9 is codepoint U+00E9 -> 2 UTF-8 bytes; in bytes, \xe9 is 1
  // byte.
  EXPECT_EQ(std::get<bytes_value>(parse("b\"\\xe9\"").v).data, std::string("\xe9"));
  EXPECT_EQ(std::get<std::string>(parse("\"\\xe9\"").v), "\xC3\xA9");
  // NUL + high bytes
  EXPECT_EQ(std::get<bytes_value>(parse("b\"\\x00\\xff\"").v).data, std::string("\x00\xff", 2));
  // a lone `b` is still a symbol, not a bytes literal
  EXPECT_EQ(to_string(parse("b")), "b");
}

TEST(StateExecBytes, ToStringAndTypeName) {
  EXPECT_EQ(make_bytes(kCafe).type_name(), "bytes");
  EXPECT_EQ(value_t(std::string(kCafe)).type_name(), "string"); // still text
  // b"..." rendering: printable ASCII verbatim, other bytes hex-escaped
  EXPECT_EQ(to_string(make_bytes("caf\xC3\xA9")), "b\"caf\\xc3\\xa9\"");
  EXPECT_EQ(to_string(make_bytes(std::string("\x00\x1f\xff", 3))), "b\"\\x00\\x1f\\xff\"");
}

TEST(StateExecBytes, EqualityIsByteWiseAndTypeDistinct) {
  EXPECT_TRUE(values_equal(make_bytes("ab"), make_bytes("ab")));
  EXPECT_FALSE(values_equal(make_bytes("ab"), make_bytes("ac")));
  // bytes and a string with the same bytes are NOT equal (distinct types)
  EXPECT_FALSE(values_equal(make_bytes("ab"), value_t(std::string("ab"))));
}

TEST(StateExecBytes, BytesModuleBuiltinsAreByteSemantic) {
  stdlib_registry reg;
  auto call = [&](const char *fn, std::vector<value_t> args) {
    const native_fn *f = reg.lookup_qualified(fn);
    EXPECT_NE(f, nullptr) << fn;
    return (*f)(std::span<const value_t>(args.data(), args.size()));
  };
  // bytes.length is BYTES (contrast string.length = codepoints): "café" bytes = 5
  EXPECT_EQ(std::get<int64_t>(call("bytes.length", {make_bytes(kCafe)}).v), 5);
  EXPECT_EQ(std::get<int64_t>(call("bytes.byte-at", {make_bytes(kCafe), value_t((int64_t)3)}).v),
            0xC3); // first byte of é
  EXPECT_EQ(
      std::get<bytes_value>(
          call("bytes.slice", {make_bytes(kCafe), value_t((int64_t)3), value_t((int64_t)2)}).v)
          .data,
      "\xC3\xA9");
  EXPECT_EQ(std::get<bytes_value>(
                call("bytes.concat", {make_bytes("ca"), make_bytes("f"), make_bytes("\xC3\xA9")}).v)
                .data,
            kCafe);
}

TEST(StateExecBytes, EncodeDecodeRoundTrip) {
  stdlib_registry reg;
  auto call = [&](const char *fn, std::vector<value_t> args) {
    const native_fn *f = reg.lookup_qualified(fn);
    EXPECT_NE(f, nullptr) << fn;
    return (*f)(std::span<const value_t>(args.data(), args.size()));
  };
  // string.encode -> bytes; bytes.decode -> string, round-trips a UTF-8 string
  value_t enc = call("string.encode", {value_t(kCafe)});
  ASSERT_TRUE(std::holds_alternative<bytes_value>(enc.v));
  EXPECT_EQ(std::get<bytes_value>(enc.v).data, kCafe); // 5 bytes
  EXPECT_EQ(std::get<std::string>(call("bytes.decode", {enc}).v), kCafe);
  // bytes.decode of invalid UTF-8 errors (strict)
  EXPECT_THROW(call("bytes.decode", {make_bytes(std::string("\xff", 1))}), std::runtime_error);
  // unsupported encoding errors
  EXPECT_THROW(call("string.encode", {value_t(std::string("x")), value_t(std::string("latin-1"))}),
               std::runtime_error);
}

TEST_F(Utf8CodecTest, CodecRoundTripsBytesIncludingBinary) {
  // arbitrary octets (NUL, control, high bytes, non-UTF-8) survive the state snapshot codec
  // (base64)
  const std::string bin = std::string("\x00\x01\x1f\x80\xfe\xff", 6);
  auto r = roundtrip(make_bytes(bin));
  ASSERT_TRUE(std::holds_alternative<bytes_value>(r.v));
  EXPECT_EQ(std::get<bytes_value>(r.v).data, bin);
  // a bytes value stays bytes through the codec (not silently a string)
  EXPECT_EQ(r.type_name(), "bytes");
}
