// cvc::net — the small JSON reader. See json_lite.h.

#include "json_lite.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace cvc {
namespace net {
namespace json_lite {

namespace {

constexpr int kMaxDepth = 64;

class Parser {
public:
  explicit Parser(const std::string &s) : s_(s) {}

  std::optional<Value> run(std::string *error) {
    Value v;
    ws();
    if (!value(v, 0) || (ws(), pos_ != s_.size())) {
      if (error)
        *error = (why_.empty() ? std::string("unexpected text") : why_) + " at offset " +
                 std::to_string(pos_);
      return std::nullopt;
    }
    return v;
  }

private:
  bool fail(const char *why) {
    if (why_.empty())
      why_ = why;
    return false;
  }
  void ws() {
    while (pos_ < s_.size() &&
           (s_[pos_] == ' ' || s_[pos_] == '\t' || s_[pos_] == '\n' || s_[pos_] == '\r'))
      ++pos_;
  }
  bool literal(const char *word) {
    std::size_t n = 0;
    while (word[n])
      ++n;
    if (s_.compare(pos_, n, word) != 0)
      return fail("not a JSON value");
    pos_ += n;
    return true;
  }

  bool value(Value &v, int depth) {
    if (depth > kMaxDepth)
      return fail("nested too deeply");
    if (pos_ >= s_.size())
      return fail("unexpected end");
    const char c = s_[pos_];
    if (c == '{')
      return object(v, depth);
    if (c == '[')
      return array(v, depth);
    if (c == '"') {
      v.kind = Value::Kind::string;
      return string(v.str);
    }
    if (c == 't') {
      v.kind = Value::Kind::boolean;
      v.b = true;
      return literal("true");
    }
    if (c == 'f') {
      v.kind = Value::Kind::boolean;
      return literal("false");
    }
    if (c == 'n') {
      v.kind = Value::Kind::null;
      return literal("null");
    }
    return number(v);
  }

  bool object(Value &v, int depth) {
    v.kind = Value::Kind::object;
    ++pos_; // {
    ws();
    if (pos_ < s_.size() && s_[pos_] == '}') {
      ++pos_;
      return true;
    }
    for (;;) {
      ws();
      if (pos_ >= s_.size() || s_[pos_] != '"')
        return fail("expected a member name");
      std::string key;
      if (!string(key))
        return false;
      ws();
      if (pos_ >= s_.size() || s_[pos_] != ':')
        return fail("expected ':'");
      ++pos_;
      ws();
      Value member;
      if (!value(member, depth + 1))
        return false;
      v.members.emplace_back(std::move(key), std::move(member));
      ws();
      if (pos_ < s_.size() && s_[pos_] == ',') {
        ++pos_;
        continue;
      }
      if (pos_ < s_.size() && s_[pos_] == '}') {
        ++pos_;
        return true;
      }
      return fail("expected ',' or '}'");
    }
  }

  bool array(Value &v, int depth) {
    v.kind = Value::Kind::array;
    ++pos_; // [
    ws();
    if (pos_ < s_.size() && s_[pos_] == ']') {
      ++pos_;
      return true;
    }
    for (;;) {
      ws();
      Value item;
      if (!value(item, depth + 1))
        return false;
      v.items.push_back(std::move(item));
      ws();
      if (pos_ < s_.size() && s_[pos_] == ',') {
        ++pos_;
        continue;
      }
      if (pos_ < s_.size() && s_[pos_] == ']') {
        ++pos_;
        return true;
      }
      return fail("expected ',' or ']'");
    }
  }

  static void utf8(std::string &out, unsigned cp) {
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

  bool hex4(unsigned &cp) {
    if (pos_ + 4 > s_.size())
      return fail("a short \\u escape");
    cp = 0;
    for (int k = 0; k < 4; ++k) {
      const char h = s_[pos_++];
      cp <<= 4;
      if (h >= '0' && h <= '9')
        cp |= static_cast<unsigned>(h - '0');
      else if (h >= 'a' && h <= 'f')
        cp |= static_cast<unsigned>(h - 'a' + 10);
      else if (h >= 'A' && h <= 'F')
        cp |= static_cast<unsigned>(h - 'A' + 10);
      else
        return fail("a bad \\u escape");
    }
    return true;
  }

  bool string(std::string &out) {
    ++pos_; // "
    while (pos_ < s_.size()) {
      const char c = s_[pos_++];
      if (c == '"')
        return true;
      if (static_cast<unsigned char>(c) < 0x20)
        return fail("a control character in a string");
      if (c != '\\') {
        out.push_back(c);
        continue;
      }
      if (pos_ >= s_.size())
        break;
      const char e = s_[pos_++];
      switch (e) {
      case '"':
      case '\\':
      case '/':
        out.push_back(e);
        break;
      case 'b':
        out.push_back('\b');
        break;
      case 'f':
        out.push_back('\f');
        break;
      case 'n':
        out.push_back('\n');
        break;
      case 'r':
        out.push_back('\r');
        break;
      case 't':
        out.push_back('\t');
        break;
      case 'u': {
        unsigned cp = 0;
        if (!hex4(cp))
          return false;
        if (cp >= 0xD800 && cp <= 0xDBFF && pos_ + 1 < s_.size() && s_[pos_] == '\\' &&
            s_[pos_ + 1] == 'u') {
          const std::size_t back = pos_;
          pos_ += 2;
          unsigned lo = 0;
          if (!hex4(lo))
            return false;
          if (lo >= 0xDC00 && lo <= 0xDFFF) {
            cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
          } else {
            pos_ = back; // not a pair: this one is lone, the next is read on its own
            cp = 0xFFFD;
          }
        } else if (cp >= 0xD800 && cp <= 0xDFFF) {
          cp = 0xFFFD;
        }
        utf8(out, cp);
        break;
      }
      default:
        return fail("a bad escape");
      }
    }
    return fail("an unterminated string");
  }

  bool number(Value &v) {
    const std::size_t start = pos_;
    bool frac = false;
    if (pos_ < s_.size() && s_[pos_] == '-')
      ++pos_;
    if (pos_ >= s_.size() || s_[pos_] < '0' || s_[pos_] > '9')
      return fail("not a JSON value");
    while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9')
      ++pos_;
    if (pos_ < s_.size() && s_[pos_] == '.') {
      frac = true;
      ++pos_;
      if (pos_ >= s_.size() || s_[pos_] < '0' || s_[pos_] > '9')
        return fail("a bad number");
      while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9')
        ++pos_;
    }
    if (pos_ < s_.size() && (s_[pos_] == 'e' || s_[pos_] == 'E')) {
      frac = true;
      ++pos_;
      if (pos_ < s_.size() && (s_[pos_] == '+' || s_[pos_] == '-'))
        ++pos_;
      if (pos_ >= s_.size() || s_[pos_] < '0' || s_[pos_] > '9')
        return fail("a bad number");
      while (pos_ < s_.size() && s_[pos_] >= '0' && s_[pos_] <= '9')
        ++pos_;
    }
    const std::string text = s_.substr(start, pos_ - start);
    v.kind = Value::Kind::number;
    v.num = std::strtod(text.c_str(), nullptr);
    if (!frac) {
      errno = 0;
      const long long i = std::strtoll(text.c_str(), nullptr, 10);
      if (errno == 0) {
        v.integral = true;
        v.i = static_cast<std::int64_t>(i);
      }
    }
    return true;
  }

  const std::string &s_;
  std::size_t pos_ = 0;
  std::string why_;
};

} // namespace

const Value *Value::get(const std::string &key) const {
  const Value *found = nullptr;
  for (const auto &m : members)
    if (m.first == key)
      found = &m.second;
  return found;
}

std::optional<std::string> Value::string_at(const std::string &key) const {
  const Value *v = get(key);
  if (!v || v->kind != Kind::string)
    return std::nullopt;
  return v->str;
}

std::optional<std::int64_t> Value::int_at(const std::string &key) const {
  const Value *v = get(key);
  if (!v)
    return std::nullopt;
  if (v->kind == Kind::number) {
    if (v->integral)
      return v->i;
    if (std::isfinite(v->num) && std::fabs(v->num) < 9.0e15)
      return static_cast<std::int64_t>(v->num);
    return std::nullopt;
  }
  if (v->kind == Kind::string && !v->str.empty() && v->str.size() < 18) {
    std::int64_t n = 0;
    for (char c : v->str) {
      if (c < '0' || c > '9')
        return std::nullopt;
      n = n * 10 + (c - '0');
    }
    return n;
  }
  return std::nullopt;
}

std::vector<std::string> Value::strings_at(const std::string &key) const {
  std::vector<std::string> out;
  const Value *v = get(key);
  if (!v)
    return out;
  if (v->kind == Kind::string) {
    out.push_back(v->str);
  } else if (v->kind == Kind::array) {
    for (const Value &e : v->items)
      if (e.kind == Kind::string)
        out.push_back(e.str);
  }
  return out;
}

std::optional<Value> parse(const std::string &text, std::string *error) {
  return Parser(text).run(error);
}

} // namespace json_lite
} // namespace net
} // namespace cvc
