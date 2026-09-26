// cvc::text — UTF-8 utilities (see the header). Roadmap §17 phase 1.
//
// The width classification is wcwidth-style: a curated set of zero-width
// (combining/format) ranges and East-Asian-wide / emoji ranges, everything else
// width 1. The ranges cover the common real cases (Latin + diacritics, CJK,
// Hangul, kana, the main emoji planes, variation selectors, zero-width/format
// controls); the roadmap's goal (§17.3) is to GENERATE the full table from the
// Unicode UCD (EastAsianWidth + the combining set) — a data refresh, not a code
// change. The lookup is a binary search over sorted, non-overlapping ranges.

#include <cvc/core/text.h>

#include <algorithm>
#include <cstdint>

namespace cvc {
namespace text {

namespace {

struct Range {
  char32_t lo, hi;
};

// True if `cp` falls in one of the sorted, non-overlapping ranges.
bool in_ranges(char32_t cp, const Range *table, std::size_t n) {
  std::size_t lo = 0, hi = n;
  while (lo < hi) {
    const std::size_t mid = lo + (hi - lo) / 2;
    if (cp < table[mid].lo)
      hi = mid;
    else if (cp > table[mid].hi)
      lo = mid + 1;
    else
      return true;
  }
  return false;
}

// Zero-width: combining marks, format/zero-width chars, variation selectors.
// Curated (not the full UCD set) — the common blocks that actually appear.
const Range kZeroWidth[] = {
    {0x0300, 0x036F}, // Combining Diacritical Marks
    {0x0483, 0x0489}, // Cyrillic combining
    {0x0591, 0x05BD}, {0x05BF, 0x05BF}, {0x05C1, 0x05C2},
    {0x05C4, 0x05C5}, {0x05C7, 0x05C7},                 // Hebrew points
    {0x0610, 0x061A}, {0x064B, 0x065F}, {0x0670, 0x0670},
    {0x06D6, 0x06DC}, {0x06DF, 0x06E4}, {0x06E7, 0x06E8},
    {0x06EA, 0x06ED},                                   // Arabic marks
    {0x0711, 0x0711}, {0x0730, 0x074A},                 // Syriac
    {0x07A6, 0x07B0}, {0x07EB, 0x07F3},                 // Thaana / NKo
    {0x0900, 0x0902}, {0x093A, 0x093A}, {0x093C, 0x093C},
    {0x0941, 0x0948}, {0x094D, 0x094D}, {0x0951, 0x0957},
    {0x0962, 0x0963},                                   // Devanagari
    {0x0981, 0x0981}, {0x09BC, 0x09BC}, {0x09C1, 0x09C4},
    {0x09CD, 0x09CD},                                   // Bengali
    {0x0E31, 0x0E31}, {0x0E34, 0x0E3A}, {0x0E47, 0x0E4E}, // Thai
    {0x0EB1, 0x0EB1}, {0x0EB4, 0x0EBC}, {0x0EC8, 0x0ECD}, // Lao
    {0x1AB0, 0x1AFF},                                   // Combining Diacriticals Ext
    {0x1DC0, 0x1DFF},                                   // Combining Diacriticals Supplement
    {0x200B, 0x200F},                                   // ZWSP, ZWNJ, ZWJ, LRM, RLM
    {0x202A, 0x202E}, {0x2060, 0x2064},                 // bidi / word joiner
    {0x20D0, 0x20F0},                                   // Combining Marks for Symbols
    {0xFE00, 0xFE0F},                                   // Variation Selectors
    {0xFE20, 0xFE2F},                                   // Combining Half Marks
    {0xE0100, 0xE01EF},                                 // Variation Selectors Supplement
};

// East-Asian wide & fullwidth, plus the emoji/pictograph planes (width 2).
const Range kWide[] = {
    {0x1100, 0x115F},   // Hangul Jamo
    {0x2329, 0x232A},   // angle brackets
    {0x2E80, 0x303E},   // CJK Radicals, Kangxi, CJK Symbols
    {0x3041, 0x33FF},   // Hiragana .. CJK Compat
    {0x3400, 0x4DBF},   // CJK Ext A
    {0x4E00, 0x9FFF},   // CJK Unified Ideographs
    {0xA000, 0xA4CF},   // Yi
    {0xA960, 0xA97F},   // Hangul Jamo Extended-A
    {0xAC00, 0xD7A3},   // Hangul Syllables
    {0xF900, 0xFAFF},   // CJK Compat Ideographs
    {0xFE10, 0xFE19},   // Vertical Forms
    {0xFE30, 0xFE6F},   // CJK Compat Forms / Small Form Variants
    {0xFF00, 0xFF60},   // Fullwidth Forms
    {0xFFE0, 0xFFE6},   // Fullwidth Signs
    {0x1F004, 0x1F004}, // MAHJONG TILE RED DRAGON
    {0x1F0CF, 0x1F0CF}, // PLAYING CARD BLACK JOKER
    {0x1F1E6, 0x1F1FF}, // Regional Indicator Symbols (flags)
    {0x1F300, 0x1F64F}, // Misc Symbols & Pictographs, Emoticons
    {0x1F900, 0x1F9FF}, // Supplemental Symbols & Pictographs
    {0x1FA00, 0x1FAFF}, // Symbols & Pictographs Ext-A
    {0x20000, 0x3FFFD}, // CJK Ext B..F + supplement
};

} // namespace

char32_t decode(std::string_view s, std::size_t &pos) {
  if (pos >= s.size())
    return 0;
  const auto b0 = static_cast<unsigned char>(s[pos]);
  auto cont = [&](std::size_t k) -> int {
    if (pos + k >= s.size())
      return -1;
    const auto b = static_cast<unsigned char>(s[pos + k]);
    return (b & 0xC0) == 0x80 ? (b & 0x3F) : -1;
  };
  if (b0 < 0x80) {
    pos += 1;
    return b0;
  }
  if ((b0 & 0xE0) == 0xC0) {
    const int c1 = cont(1);
    const char32_t cp = (static_cast<char32_t>(b0 & 0x1F) << 6) | static_cast<char32_t>(c1);
    if (c1 < 0 || cp < 0x80) { // bad continuation / overlong
      pos += 1;
      return kReplacement;
    }
    pos += 2;
    return cp;
  }
  if ((b0 & 0xF0) == 0xE0) {
    const int c1 = cont(1), c2 = cont(2);
    const char32_t cp = (static_cast<char32_t>(b0 & 0x0F) << 12) |
                        (static_cast<char32_t>(c1) << 6) | static_cast<char32_t>(c2);
    if (c1 < 0 || c2 < 0 || cp < 0x800 || (cp >= 0xD800 && cp <= 0xDFFF)) {
      pos += 1; // bad continuation / overlong / surrogate
      return kReplacement;
    }
    pos += 3;
    return cp;
  }
  if ((b0 & 0xF8) == 0xF0) {
    const int c1 = cont(1), c2 = cont(2), c3 = cont(3);
    const char32_t cp = (static_cast<char32_t>(b0 & 0x07) << 18) |
                        (static_cast<char32_t>(c1) << 12) | (static_cast<char32_t>(c2) << 6) |
                        static_cast<char32_t>(c3);
    if (c1 < 0 || c2 < 0 || c3 < 0 || cp < 0x10000 || cp > 0x10FFFF) {
      pos += 1; // bad continuation / overlong / out of range
      return kReplacement;
    }
    pos += 4;
    return cp;
  }
  pos += 1; // invalid lead byte (0x80..0xBF or 0xF8..0xFF)
  return kReplacement;
}

void encode(char32_t cp, std::string &out) {
  if (cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF))
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

int codepoint_width(char32_t cp) {
  if (cp == 0)
    return 0;
  if (cp < 0x20 || (cp >= 0x7F && cp < 0xA0))
    return 0; // C0 / DEL / C1 control chars
  if (in_ranges(cp, kZeroWidth, sizeof(kZeroWidth) / sizeof(Range)))
    return 0;
  if (in_ranges(cp, kWide, sizeof(kWide) / sizeof(Range)))
    return 2;
  return 1;
}

std::size_t display_width(std::string_view s) {
  std::size_t w = 0, pos = 0;
  while (pos < s.size())
    w += static_cast<std::size_t>(codepoint_width(decode(s, pos)));
  return w;
}

std::size_t codepoint_count(std::string_view s) {
  std::size_t n = 0, pos = 0;
  while (pos < s.size()) {
    decode(s, pos);
    ++n;
  }
  return n;
}

bool is_valid_utf8(std::string_view s) {
  std::size_t pos = 0;
  while (pos < s.size()) {
    const std::size_t start = pos;
    const char32_t cp = decode(s, pos);
    // A genuine U+FFFD in the input advances by 3 bytes; a malformed byte that
    // decode() replaced advances by 1 — that single-byte replacement is the tell.
    if (cp == kReplacement && pos - start == 1)
      return false;
  }
  return true;
}

std::size_t prev_codepoint(std::string_view s, std::size_t pos) {
  if (pos == 0)
    return 0;
  if (pos > s.size())
    pos = s.size();
  // Step back over any continuation bytes (0x80..0xBF) to the lead byte.
  std::size_t i = pos - 1;
  while (i > 0 && (static_cast<unsigned char>(s[i]) & 0xC0) == 0x80)
    --i;
  return i;
}

std::size_t next_codepoint(std::string_view s, std::size_t pos) {
  if (pos >= s.size())
    return s.size();
  std::size_t p = pos;
  decode(s, p); // advances p by the codepoint (or 1 for a malformed byte)
  return p;
}

std::string truncate_to_width(std::string_view s, std::size_t max_cols, std::string_view ellipsis) {
  if (display_width(s) <= max_cols)
    return std::string(s);
  const std::size_t ell_w = display_width(ellipsis);
  // Room for content once the ellipsis is reserved (if it fits at all).
  const std::size_t budget = (ell_w <= max_cols) ? max_cols - ell_w : max_cols;
  std::string out;
  std::size_t w = 0, pos = 0;
  while (pos < s.size()) {
    const std::size_t start = pos;
    const char32_t cp = decode(s, pos);
    const std::size_t cw = static_cast<std::size_t>(codepoint_width(cp));
    if (w + cw > budget)
      break;
    out.append(s.substr(start, pos - start));
    w += cw;
  }
  if (ell_w <= max_cols)
    out.append(ellipsis);
  return out;
}

std::string pad_to_width(std::string_view s, std::size_t cols, char fill) {
  std::string out(s);
  const std::size_t w = display_width(s);
  if (w < cols)
    out.append(cols - w, fill);
  return out;
}

} // namespace text
} // namespace cvc
