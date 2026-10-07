/**
 * @file utf8.h
 * @brief Minimal, dependency-free UTF-8 codepoint helper for the state_exec DSL.
 *
 * SE-0 of the state_exec Unicode roadmap (docs/roadmap/UNICODE_SUPPORT_ROADMAP.md,
 * "state_exec" section). state_exec stores every string as a raw UTF-8 std::string;
 * operations that merely store / copy / concatenate / substring-search bytes are already
 * Unicode-clean. The gaps are the operations that ascribe CHARACTER semantics to a byte —
 * length, indexing, slicing, case, the lexer's identifier class, \u escapes. This header is
 * the one shared primitive those fixes (SE-1/SE-2) build on: decode / count / index / encode
 * over UTF-8, with no ICU and no allocation.
 *
 * Policy (the "text vs bytes" split, mirrored in the roadmap): a `string` is TEXT — a sequence
 * of Unicode codepoints stored as UTF-8, so `length`/`char-at`/`substring` count and slice by
 * codepoint (SE-1 wiring). Raw binary belongs in a future `bytes` type (SE-3), never in a
 * `string`. This header deliberately does NOT touch the value_t builtins yet — it is added and
 * unit-tested in isolation (no behavior change) so SE-1 can wire it in as a pure swap of
 * byte-arithmetic for codepoint-arithmetic.
 *
 * Decoding is LENIENT and always progresses: on a malformed lead/continuation byte, an overlong
 * encoding, a surrogate, or a truncated tail, decode() yields U+FFFD and consumes exactly ONE
 * byte, so an iterator can never stall or read out of bounds on invalid input. Use is_valid()
 * when strict well-formedness matters.
 */
#ifndef CVC_STATE_EXEC_UTF8_H
#define CVC_STATE_EXEC_UTF8_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace cvc::state_exec::utf8 {

/// The Unicode replacement character, yielded for any malformed sequence.
inline constexpr char32_t kReplacement = 0xFFFD;
/// The largest valid Unicode scalar value.
inline constexpr char32_t kMaxCodepoint = 0x10FFFF;

/// True if `b` is a UTF-8 continuation byte (10xxxxxx).
inline bool is_continuation(unsigned char b) { return (b & 0xC0) == 0x80; }

/// Decode the codepoint that begins at byte offset `pos` in `s`.
///
/// Precondition: pos < s.size(). On a well-formed sequence, sets `cp` to the codepoint and `len`
/// to its byte length (1..4) and returns true. On any malformed byte (bad lead, missing/invalid
/// continuation, overlong form, surrogate, out-of-range, or truncation at the end of `s`), sets
/// cp = U+FFFD, len = 1, and returns false. `len` is always >= 1, so callers advance safely.
inline bool decode(std::string_view s, std::size_t pos, char32_t &cp, std::size_t &len) {
  const std::size_t n = s.size();
  const unsigned char b0 = static_cast<unsigned char>(s[pos]);
  auto bad = [&]() {
    cp = kReplacement;
    len = 1;
    return false;
  };
  if (b0 < 0x80) { // ASCII fast path
    cp = b0;
    len = 1;
    return true;
  }
  std::size_t need; // continuation bytes required
  char32_t acc;     // accumulator seeded with the lead bits
  char32_t min_cp;  // smallest value NOT overlong for this length
  if ((b0 & 0xE0) == 0xC0) {
    need = 1;
    acc = b0 & 0x1F;
    min_cp = 0x80;
  } else if ((b0 & 0xF0) == 0xE0) {
    need = 2;
    acc = b0 & 0x0F;
    min_cp = 0x800;
  } else if ((b0 & 0xF8) == 0xF0) {
    need = 3;
    acc = b0 & 0x07;
    min_cp = 0x10000;
  } else {
    return bad(); // 0x80-0xBF stray continuation, or 0xF8-0xFF invalid lead
  }
  if (need > n - pos - 1)
    return bad(); // truncated: not enough bytes left for the continuations
  for (std::size_t i = 1; i <= need; ++i) {
    const unsigned char bi = static_cast<unsigned char>(s[pos + i]);
    if (!is_continuation(bi))
      return bad();
    acc = (acc << 6) | (bi & 0x3F);
  }
  if (acc < min_cp)
    return bad(); // overlong encoding
  if (acc > kMaxCodepoint || (acc >= 0xD800 && acc <= 0xDFFF))
    return bad(); // out of range, or a UTF-16 surrogate half (never valid in UTF-8)
  cp = acc;
  len = need + 1;
  return true;
}

/// Number of Unicode codepoints in `s`. A malformed byte counts as one codepoint (matching
/// decode()'s one-byte advance), so this never disagrees with iterating decode().
inline std::size_t count(std::string_view s) {
  std::size_t cps = 0, pos = 0;
  while (pos < s.size()) {
    char32_t cp;
    std::size_t len;
    decode(s, pos, cp, len);
    pos += len;
    ++cps;
  }
  return cps;
}

/// Byte offset of the `index`-th codepoint (0-based). Returns s.size() when `index` is at or
/// past the codepoint count (so a caller can treat the result as a clamped slice boundary).
inline std::size_t byte_offset(std::string_view s, std::size_t index) {
  std::size_t pos = 0, i = 0;
  while (pos < s.size() && i < index) {
    char32_t cp;
    std::size_t len;
    decode(s, pos, cp, len);
    pos += len;
    ++i;
  }
  return pos;
}

/// Append the UTF-8 encoding of `cp` to `out`. An out-of-range value or a surrogate is encoded
/// as U+FFFD (so encode/decode round-trips only well-formed scalar values).
inline void encode(char32_t cp, std::string &out) {
  if (cp > kMaxCodepoint || (cp >= 0xD800 && cp <= 0xDFFF))
    cp = kReplacement;
  if (cp < 0x80) {
    out.push_back(static_cast<char>(cp));
  } else if (cp < 0x800) {
    out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else if (cp < 0x10000) {
    out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  }
}

/// True iff every byte of `s` is part of a well-formed UTF-8 sequence (no replacement fallback).
inline bool is_valid(std::string_view s) {
  std::size_t pos = 0;
  while (pos < s.size()) {
    char32_t cp;
    std::size_t len;
    if (!decode(s, pos, cp, len))
      return false;
    pos += len;
  }
  return true;
}

} // namespace cvc::state_exec::utf8

#endif // CVC_STATE_EXEC_UTF8_H
