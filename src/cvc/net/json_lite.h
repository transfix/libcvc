// cvc::net — a small JSON reader for the replies the OAuth 2.0 / OpenID Connect client reads (a
// discovery document, a token or device-authorization reply, an ID token's claims). Private to
// src/cvc/net: it reads whole documents of at most a few kilobytes into a tree, which is all those
// need. Strings are UTF-8 (\u escapes decoded, a lone surrogate becomes U+FFFD); numbers are
// doubles, and `integral` says whether one was written without a fraction or an exponent and fits
// in 64 bits. Nesting deeper than 64 is refused.
#ifndef CVC_NET_JSON_LITE_H
#define CVC_NET_JSON_LITE_H

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace net {
namespace json_lite {

struct Value {
  enum class Kind { null, boolean, number, string, array, object };
  Kind kind = Kind::null;
  bool b = false;
  double num = 0.0;
  bool integral = false;
  std::int64_t i = 0;
  std::string str;
  std::vector<Value> items;                           // an array
  std::vector<std::pair<std::string, Value>> members; // an object, in document order

  bool is_object() const { return kind == Kind::object; }
  bool is_string() const { return kind == Kind::string; }
  // The member named `key` (the last one, when it repeats), or null.
  const Value *get(const std::string &key) const;
  // The string member `key`, or nullopt when there is none or it is not a string.
  std::optional<std::string> string_at(const std::string &key) const;
  // The number member `key` as a whole number (a string of digits too: some providers write
  // expires_in as "3600"), or nullopt.
  std::optional<std::int64_t> int_at(const std::string &key) const;
  // The member `key` as a list of strings: an array of strings, or one string. Empty otherwise.
  std::vector<std::string> strings_at(const std::string &key) const;
};

// Parse `text`. nullopt, with `error` saying where, when it is not one JSON value (trailing
// non-space is an error too).
std::optional<Value> parse(const std::string &text, std::string *error = nullptr);

} // namespace json_lite
} // namespace net
} // namespace cvc

#endif // CVC_NET_JSON_LITE_H
