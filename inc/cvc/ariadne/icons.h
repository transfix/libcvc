// Named icons for the menubar's tray (§5.21). A `tray_icon:` whose value is one of these names
// draws the name's glyph, or its ASCII stand-in on a backend without the glyphs
// (Backend::unicode_glyphs: the Linux console, a dumb or a VT terminal). Any other value is drawn
// as written.
//
//   menubar:
//     - tray_icon: warning     # ⚠, or ! without the glyphs
//     - tray_icon: "♞"         # not a name: drawn as written
//
// The glyphs are ones the UI fonts carry (Arial and Segoe UI with the bundled DejaVu Sans Mono
// under them, in ImGui; a terminal's own font), not emoji, which need a colour emoji font.
#ifndef CVC_ARIADNE_ICONS_H
#define CVC_ARIADNE_ICONS_H

#include <cstddef>
#include <string_view>

namespace cvc {
namespace ariadne {

struct NamedIcon {
  const char *name;
  const char *glyph; // UTF-8
  const char *ascii; // one or two printable ASCII characters
};

inline constexpr NamedIcon kNamedIcons[] = {
    {"warning", "\xE2\x9A\xA0", "!"}, // ⚠
    {"error", "\xE2\x9C\x96", "x"},   // ✖
    {"info", "\xE2\x84\xB9", "i"},    // ℹ
    {"check", "\xE2\x9C\x94", "v"},   // ✔
    {"gear", "\xE2\x9A\x99", "%"},    // ⚙
    {"mail", "\xE2\x9C\x89", "@"},    // ✉
    {"sync", "\xE2\x9F\xB3", "~"},    // ⟳
    {"user", "\xE2\x98\xBA", "&"},    // ☺
    {"network", "\xE2\x87\x85", "^"}, // ⇅
    {"star", "\xE2\x98\x85", "+"},    // ★
    {"clock", "\xE2\x97\xB7", "o"},   // ◷
    {"menu", "\xE2\x98\xB0", "="},    // ☰
    {"more", "\xC2\xBB", ">>"},       // » (the tray's overflow)
};

// The named icon called `name`, or nullptr when it is not a name.
inline const NamedIcon *named_icon(std::string_view name) {
  for (const NamedIcon &i : kNamedIcons)
    if (name == i.name)
      return &i;
  return nullptr;
}

// What a tray icon draws for `label`: a name's glyph (or its ASCII stand-in when `unicode` is
// false), else the label as written.
inline std::string_view icon_glyph(std::string_view label, bool unicode) {
  if (const NamedIcon *i = named_icon(label))
    return unicode ? i->glyph : i->ascii;
  return label;
}

} // namespace ariadne
} // namespace cvc

#endif
