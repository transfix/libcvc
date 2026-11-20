#ifndef CVC_ARIADNE_COLOR_VALUE_H
#define CVC_ARIADNE_COLOR_VALUE_H
// Colours a document gives its widgets (docs/ARIADNE.md §5.26): a widget's `color:` and a group's
// `background: { color: }`, written in the document or held in state. One colour is written as
//
//   - "r,g,b" or "r g b", each channel 0..1 (what a `color` picker writes to its key);
//   - "[r, g, b]", the same in brackets;
//   - "#rrggbb";
//   - "c0".."c7", the chart's categorical slots (blue, orange, aqua, yellow, magenta, green,
//     violet, red), so a row and its curve in a chart can share one.
//
// "", "none" and "default" are no colour: the widget's own. A list of colours (a radio's, one per
// option) is the colours separated by ';', an entry left empty taking the default.
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
namespace ariadne {

// A colour the document gave an item, each channel 0..1. `set` false: the backend's own colour.
struct ItemColor {
  bool set = false;
  float rgb[3] = {1.0f, 1.0f, 1.0f};
  bool operator==(const ItemColor &o) const {
    return set == o.set &&
           (!set || (rgb[0] == o.rgb[0] && rgb[1] == o.rgb[1] && rgb[2] == o.rgb[2]));
  }
};

// The chart's categorical slots c0..c7 as 0xRRGGBB.
inline constexpr unsigned kColorSlots[8] = {0x2a78d6, 0xeb6834, 0x1baf7a, 0xeda100,
                                            0xe87ba4, 0x008300, 0x6250d6, 0xe34948};

// Whether `s` (spaces around it ignored) says no colour: "", "none" or "default".
bool is_no_color(std::string_view s);

// One colour written as above into `out` (channels clamped to 0..1). False, `out` untouched, for
// anything else, no colour included.
bool parse_color_text(std::string_view s, float out[3]);

// A colour or a ';'-separated list of them: one entry per colour, an empty or "none" entry unset.
// "" (or "none") is no entries. False when an entry is neither a colour nor empty; `bad` (when not
// null) is then that entry.
bool parse_color_list(std::string_view s, std::vector<ItemColor> &out, std::string *bad = nullptr);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_COLOR_VALUE_H
