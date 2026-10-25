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
    out += k;
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
  // Lower case, without the spaces and underscores the two spellings differ by...
  std::string k;
  for (char c : name)
    if (c != ' ' && c != '_')
      k += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  // ...then VTK's X11-style keysyms to SDL's names.
  static const std::map<std::string, std::string> vtk = {
      {"shiftl", "leftshift"},   {"shiftr", "rightshift"},  {"controll", "leftctrl"},
      {"controlr", "rightctrl"}, {"altl", "leftalt"},       {"altr", "rightalt"},
      {"superl", "leftgui"},     {"superr", "rightgui"},    {"prior", "pageup"},
      {"next", "pagedown"},      {"kpenter", "keypadenter"}};
  const auto it = vtk.find(k);
  return it == vtk.end() ? k : it->second;
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
