#ifndef CVC_CORE_TEXT_H
#define CVC_CORE_TEXT_H

// cvc::text — UTF-8 text utilities (roadmap §17, phase 1). The one place the
// byte-vs-column truth lives: codepoint iteration + display width (a wcwidth-style
// classification), shared by every UI backend and by layout so Unicode renders
// correctly regardless of backend.
//
// The encoding contract (§17.2): every text std::string crossing a libcvc/pycvc
// boundary is UTF-8. These utilities treat input as UTF-8; a malformed byte
// decodes to U+FFFD (the replacement character) and never crashes or loops.

#include <cstddef>
#include <string>
#include <string_view>

namespace cvc {
namespace text {

constexpr char32_t kReplacement = 0xFFFD; // U+FFFD REPLACEMENT CHARACTER

// Decode the UTF-8 codepoint at byte offset `pos` in `s`, advancing `pos` past
// it. A malformed lead/continuation byte, an overlong form, a surrogate, or an
// out-of-range value yields kReplacement and advances by exactly one byte — so a
// `while (pos < s.size()) decode(s, pos);` loop always terminates. Returns 0 and
// leaves `pos` unchanged when `pos >= s.size()`.
char32_t decode(std::string_view s, std::size_t &pos);

// Append the UTF-8 encoding of `cp` to `out`. A value that is not a valid Unicode
// scalar (> U+10FFFF or a surrogate) is encoded as kReplacement.
void encode(char32_t cp, std::string &out);

// Display width, in terminal columns, of one codepoint: 0 for a combining mark /
// zero-width / format / control char, 2 for East-Asian wide & fullwidth and most
// emoji, 1 otherwise (§17.3).
int codepoint_width(char32_t cp);

// Display width of a UTF-8 string = sum of its codepoints' widths.
std::size_t display_width(std::string_view s);

// Number of codepoints in a UTF-8 string (a malformed byte counts as one).
std::size_t codepoint_count(std::string_view s);

// Whether `s` is entirely well-formed UTF-8.
bool is_valid_utf8(std::string_view s);

// Byte offset of the start of the codepoint at or before `pos`, moving left (for
// cursor-left / backspace). Snaps `pos` back onto a boundary; returns 0 at start.
std::size_t prev_codepoint(std::string_view s, std::size_t pos);

// Byte offset just past the codepoint at `pos`, moving right (cursor-right /
// delete). Returns s.size() at the end.
std::size_t next_codepoint(std::string_view s, std::size_t pos);

// Truncate `s` to at most `max_cols` display columns on a codepoint boundary
// (never mid-codepoint, never splitting a wide glyph). When truncation happens
// and `ellipsis` fits within `max_cols`, it is appended and counted toward the
// width. Returns `s` unchanged when it already fits.
std::string truncate_to_width(std::string_view s, std::size_t max_cols,
                              std::string_view ellipsis = std::string_view());

// Right-pad `s` with `fill` (a single-column ASCII char) to exactly `cols`
// display columns. A no-op when display_width(s) >= cols.
std::string pad_to_width(std::string_view s, std::size_t cols, char fill = ' ');

} // namespace text
} // namespace cvc

#endif // CVC_CORE_TEXT_H
