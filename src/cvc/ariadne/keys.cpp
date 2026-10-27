// Rebindable keys -- see keys.h.
#include <cctype>
#include <cvc/ariadne/keys.h>
#include <map>

namespace cvc {
namespace ariadne {

std::string join_keys(const std::vector<std::string> &keys) {
  std::string out;
  for (const std::string &k : keys) {
    if (k.empty())
      continue;
    if (!out.empty())
      out += ", ";
    out += stored_key_name(k); // a document's "," as well as a captured one
  }
  return out;
}

std::vector<std::string> split_keys(std::string_view stored) {
  std::vector<std::string> out;
  std::size_t at = 0;
  while (at <= stored.size()) {
    const std::size_t comma = stored.find(',', at);
    std::string_view k =
        stored.substr(at, comma == std::string_view::npos ? stored.npos : comma - at);
    while (!k.empty() && std::isspace(static_cast<unsigned char>(k.front())))
      k.remove_prefix(1);
    while (!k.empty() && std::isspace(static_cast<unsigned char>(k.back())))
      k.remove_suffix(1);
    if (!k.empty())
      out.emplace_back(k);
    if (comma == std::string_view::npos)
      break;
    at = comma + 1;
  }
  return out;
}

std::string canonical_key(std::string_view name) {
  if (name == "_") // the one key whose name is an underscore
    return "underscore";
  // Lower case, without the spaces and underscores the two spellings differ by...
  std::string k;
  for (char c : name)
    if (c != ' ' && c != '_')
      k += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  // ...then each spelling of a key to one: VTK's X11 keysyms to SDL's names, and SDL's
  // punctuation (a character) to the keysym's word.
  static const std::map<std::string, std::string> same = {{"shiftl", "leftshift"},
                                                          {"shiftr", "rightshift"},
                                                          {"controll", "leftctrl"},
                                                          {"controlr", "rightctrl"},
                                                          {"altl", "leftalt"},
                                                          {"altr", "rightalt"},
                                                          {"superl", "leftgui"},
                                                          {"superr", "rightgui"},
                                                          {"prior", "pageup"},
                                                          {"next", "pagedown"},
                                                          {"numlock", "numlockclear"},
                                                          {"print", "printscreen"},
                                                          {",", "comma"},
                                                          {".", "period"},
                                                          {"/", "slash"},
                                                          {"\\", "backslash"},
                                                          {";", "semicolon"},
                                                          {"'", "apostrophe"},
                                                          {"[", "bracketleft"},
                                                          {"]", "bracketright"},
                                                          {"-", "minus"},
                                                          {"=", "equal"},
                                                          {"`", "grave"},
                                                          {"kpenter", "keypadenter"},
                                                          {"kpadd", "keypad+"},
                                                          {"kpsubtract", "keypad-"},
                                                          {"kpmultiply", "keypad*"},
                                                          {"kpdivide", "keypad/"},
                                                          {"kpdecimal", "keypad."},
                                                          {"kpequal", "keypad="},
                                                          // shifted punctuation, and VTK's
                                                          // Win32 / Cocoa quote names
                                                          {"+", "plus"},
                                                          {"<", "less"},
                                                          {">", "greater"},
                                                          {"?", "question"},
                                                          {":", "colon"},
                                                          {"\"", "quotedbl"},
                                                          {"!", "exclam"},
                                                          {"@", "at"},
                                                          {"#", "numbersign"},
                                                          {"$", "dollar"},
                                                          {"%", "percent"},
                                                          {"^", "asciicircum"},
                                                          {"&", "ampersand"},
                                                          {"*", "asterisk"},
                                                          {"(", "parenleft"},
                                                          {")", "parenright"},
                                                          {"{", "braceleft"},
                                                          {"}", "braceright"},
                                                          {"|", "bar"},
                                                          {"~", "asciitilde"},
                                                          {"quoteright", "apostrophe"},
                                                          {"quoteleft", "grave"}};
  if (k.size() == 3 && k[0] == 'k' && k[1] == 'p' && std::isdigit(static_cast<unsigned char>(k[2])))
    return "keypad" + k.substr(2); // KP_0 .. KP_9 -> SDL's "Keypad 0" ..
  const auto it = same.find(k);
  return it == same.end() ? k : it->second;
}

std::string stored_key_name(std::string_view pressed) {
  return pressed == "," ? std::string("Comma") : std::string(pressed);
}

bool key_matches(std::string_view stored, std::string_view key) {
  const std::string pressed = canonical_key(key);
  if (pressed.empty())
    return false;
  for (const std::string &k : split_keys(stored))
    if (canonical_key(k) == pressed)
      return true;
  return false;
}

} // namespace ariadne
} // namespace cvc
