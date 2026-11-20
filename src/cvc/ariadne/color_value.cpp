// Colours a document gives its widgets (docs/ARIADNE.md §5.26) — see the header.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cvc/ariadne/color_value.h>
#include <string>

namespace cvc {
namespace ariadne {

namespace {

std::string_view trim(std::string_view s) {
  const auto space = [](char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; };
  while (!s.empty() && space(s.front()))
    s.remove_prefix(1);
  while (!s.empty() && space(s.back()))
    s.remove_suffix(1);
  return s;
}

void rgb_of(unsigned hex, float out[3]) {
  out[0] = static_cast<float>((hex >> 16) & 0xff) / 255.0f;
  out[1] = static_cast<float>((hex >> 8) & 0xff) / 255.0f;
  out[2] = static_cast<float>(hex & 0xff) / 255.0f;
}

} // namespace

bool is_no_color(std::string_view s) {
  s = trim(s);
  return s.empty() || s == "none" || s == "default";
}

bool parse_color_text(std::string_view s, float out[3]) {
  s = trim(s);
  if ((s.size() == 7 || s.size() == 4) && s[0] == '#') {
    const std::string digits(s.substr(1));
    if (!std::all_of(digits.begin(), digits.end(),
                     [](char c) { return std::isxdigit(static_cast<unsigned char>(c)) != 0; }))
      return false;
    unsigned hex = static_cast<unsigned>(std::strtoul(digits.c_str(), nullptr, 16));
    if (digits.size() == 3) // #rgb: each digit twice, #f80 = #ff8800
      hex = ((hex >> 8) & 0xf) * 0x110000 + ((hex >> 4) & 0xf) * 0x1100 + (hex & 0xf) * 0x11;
    rgb_of(hex, out);
    return true;
  }
  if (s.size() == 2 && (s[0] == 'c' || s[0] == 'C') && s[1] >= '0' && s[1] <= '7') {
    rgb_of(kColorSlots[s[1] - '0'], out);
    return true;
  }
  if (s.size() >= 2 && s.front() == '[' && s.back() == ']')
    s = trim(s.substr(1, s.size() - 2));
  // Three numbers, separated by commas and/or spaces, and nothing else.
  std::string t(s);
  for (char &c : t)
    if (c == ',')
      c = ' ';
  const char *p = t.c_str();
  float v[3];
  for (float &x : v) {
    char *end = nullptr;
    x = std::strtof(p, &end);
    if (end == p || !std::isfinite(x))
      return false;
    p = end;
  }
  while (*p == ' ' || *p == '\t')
    ++p;
  if (*p != '\0')
    return false;
  for (int i = 0; i < 3; ++i)
    out[i] = std::clamp(v[i], 0.0f, 1.0f);
  return true;
}

bool parse_color_list(std::string_view s, std::vector<ItemColor> &out, std::string *bad) {
  out.clear();
  if (is_no_color(s))
    return true;
  std::size_t start = 0;
  for (;;) {
    const std::size_t semi = s.find(';', start);
    const std::string_view e =
        s.substr(start, semi == std::string_view::npos ? std::string_view::npos : semi - start);
    ItemColor c;
    if (!is_no_color(e)) {
      if (!parse_color_text(e, c.rgb)) {
        if (bad)
          *bad = std::string(trim(e));
        out.clear();
        return false;
      }
      c.set = true;
    }
    out.push_back(c);
    if (semi == std::string_view::npos)
      break;
    start = semi + 1;
  }
  return true;
}

} // namespace ariadne
} // namespace cvc
