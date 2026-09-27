// pycvc_text.i — expose cvc::text (the UTF-8 display-width utilities, roadmap
// §17) to Python, so a pycvc-driven UI (a VolRover3 panel, a pycvc Ariadne host)
// does column/width math the SAME way the C++ side does — display columns, not
// bytes or Python len() (which counts codepoints, still not columns for CJK/emoji).
//
// Encoding boundary (§17.6, audited): pycvc's std::string typemap (std_string.i)
// round-trips UTF-8 in BOTH directions on Python 3; a C++ std::string that is not
// valid UTF-8 comes back to Python via surrogateescape (lone \udcXX surrogates —
// lossless and round-trippable), never raising. So these take/return ordinary
// Python str.

%{
#include <cvc/core/text.h>
%}

%inline %{
// Display width in terminal columns (wcwidth-style): 0 for combining/zero-width/
// control, 2 for East-Asian wide & fullwidth + emoji, 1 otherwise.
unsigned long text_display_width(const std::string &s) { return cvc::text::display_width(s); }

// Column width of one Unicode codepoint (pass the integer code point).
int text_codepoint_width(unsigned int cp) {
  return cvc::text::codepoint_width(static_cast<char32_t>(cp));
}

// Number of codepoints in a UTF-8 string (Python len() already gives this for a
// valid str; provided for parity + to count a surrogate-escaped value's scalars).
unsigned long text_codepoint_count(const std::string &s) { return cvc::text::codepoint_count(s); }

// Whether the string is well-formed UTF-8 (false for a surrogate-escaped value).
bool text_is_valid_utf8(const std::string &s) { return cvc::text::is_valid_utf8(s); }

// Truncate to at most max_cols display columns on a codepoint boundary (never
// splits a wide glyph); appends `ellipsis` when it truncates and the ellipsis fits.
std::string text_truncate_to_width(const std::string &s, unsigned long max_cols,
                                   const std::string &ellipsis = std::string()) {
  return cvc::text::truncate_to_width(s, max_cols, ellipsis);
}

// Right-pad with spaces to exactly `cols` display columns (no-op when already wider).
std::string text_pad_to_width(const std::string &s, unsigned long cols) {
  return cvc::text::pad_to_width(s, cols);
}
%}
