// JSON for the DSL: (json-parse text) and (json-encode value [indent]), as data jobs that run a
// slice at a time ("Deep data on the executor" in docs/STATE_EXEC_GRAMMAR.md). Documents get
// their JSON from outside (REST replies), so both keep their work on the heap, nest at most
// kJsonMaxDepth levels and hold what they build to the allocation limit.
//
// json-parse reads RFC 8259 text:
// - an object is a dict with its keys in document order. A key given twice keeps its first place
//   and its last value, as JavaScript's JSON.parse and Python's json.loads do;
// - an array is a list; true and false are bools; null is nil;
// - a number without a fraction or exponent is an int when it fits in 64 bits, and otherwise a
//   float (inexact past 2^53, as in JavaScript); one with a fraction or exponent is a float; one
//   too large for a float is an error;
// - strings are UTF-8. A byte that isn't part of valid UTF-8, or a \u escape of a lone
//   surrogate, becomes U+FFFD, so a reply with a truncated emoji still reads (a browser's fetch
//   decodes bytes the same way);
// - anything else, a raw control character in a string included, is a ValueError naming the line
//   and column (in code points), with `line`, `col` and `offset` (in bytes) as fields.
//
// json-encode writes the reverse: nil is null, an int as digits, a float in its shortest
// round-trip form with ".0" when it would otherwise read back as an int, a string escaped
// (non-ASCII stays as UTF-8), a list as an array and a dict as an object in its own order.

#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h> // detail::format_number_shortest_fallback
#include <cvc/state/state_exec/utf8.h>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace cvc::state_exec {

namespace {

// -- parse -------------------------------------------------------------------

// What a parsed value takes on the heap, roughly, for the allocation limit: its slot in a list
// or its entry (key included) in a dict, a string's bytes, and a list's or dict's own block.
constexpr std::size_t kContainerBytes = 64;

// A dict being filled looks a repeated key up by scanning its entries until it has this many,
// and through an index after that.
constexpr std::size_t kIndexedKeys = 16;

class json_parse_walk final : public data_job {
public:
  explicit json_parse_walk(std::string text) : own_(std::move(text)), doc_(own_) {}
  // The text belongs to the call's argument until own_inputs().
  struct borrow {};
  json_parse_walk(std::string_view text, borrow) : doc_(text) {}

  bool step(std::size_t budget) override {
    const std::size_t start = nodes_;
    const std::optional<std::size_t> limit = eval_alloc_limit();
    while (want_ != want::done && (budget == 0 || nodes_ - start < budget)) {
      eval_deadline_check("json-parse");
      advance();
      nodes_ = values_ + pos_ / kDataNodeBytes;
      if (limit && bytes_ > *limit)
        eval_alloc_check(bytes_, "json-parse"); // throws
    }
    eval_alloc_note(bytes_ - noted_);
    noted_ = bytes_;
    return want_ == want::done;
  }

  value_t result() override { return std::move(root_); }

  void own_inputs() override {
    if (doc_.data() != own_.data()) {
      own_.assign(doc_);
      doc_ = own_;
    }
  }

  // The root holds everything parsed so far.
  void held_values(const std::function<void(const value_t &)> &visit) const override {
    if (!root_.is_nil())
      visit(root_);
  }

  std::size_t held_bytes() const override {
    return sizeof(*this) + own_.capacity() + stack_.capacity() * sizeof(frame);
  }

private:
  enum class want {
    value,      // a value: the document's, an array's element or an object's member's
    first_item, // after '[': an element or ']'
    first_key,  // after '{': a key or '}'
    key,        // after ',' in an object
    next,       // after an element or member: ',' or the closing bracket
    end,        // after the document's value: nothing but whitespace
    done
  };

  struct frame {
    value_t container; // the list or dict being filled
    bool array;
    std::string key;                                    // the member being read
    std::unordered_map<std::string, std::size_t> index; // key -> entry, once the dict is big
  };

  void advance() {
    skip_ws();
    switch (want_) {
    case want::value:
      value();
      return;
    case want::first_item:
      if (have(']'))
        close();
      else
        want_ = want::value;
      return;
    case want::first_key:
      if (have('}'))
        close();
      else
        key("expected a string key or '}'");
      return;
    case want::key:
      key("expected a string key");
      return;
    case want::next: {
      const bool array = stack_.back().array;
      if (have(','))
        want_ = array ? want::value : want::key;
      else if (have(array ? ']' : '}'))
        close();
      else
        fail(array ? "expected ',' or ']'" : "expected ',' or '}'");
      return;
    }
    case want::end:
      if (pos_ < doc_.size())
        fail("unexpected text after the value");
      want_ = want::done;
      return;
    case want::done:
      return;
    }
  }

  void value() {
    if (pos_ >= doc_.size())
      fail("expected a value");
    const char c = doc_[pos_];
    switch (c) {
    case '{':
    case '[':
      open(c == '[');
      return;
    case '"': {
      ++pos_;
      std::string s;
      string(s);
      bytes_ += s.size();
      put(value_t(std::move(s)));
      return;
    }
    case 't':
      literal("true");
      put(value_t(true));
      return;
    case 'f':
      literal("false");
      put(value_t(false));
      return;
    case 'n':
      literal("null");
      put(nil_value);
      return;
    default:
      if (c == '-' || (c >= '0' && c <= '9')) {
        put(number());
        return;
      }
      fail("expected a value");
    }
  }

  // Put `v` where the document is: its root, the open array's next element or the open object's
  // member.
  void put(value_t v) {
    ++values_;
    if (stack_.empty()) {
      root_ = std::move(v);
      want_ = want::end;
      return;
    }
    frame &f = stack_.back();
    if (f.array) {
      bytes_ += sizeof(value_t);
      std::get<list_ptr>(f.container.v)->push_back(std::move(v));
    } else {
      bytes_ += sizeof(std::pair<std::string, value_t>) + f.key.size();
      member(f, std::move(v));
    }
    want_ = want::next;
  }

  // A repeated key keeps its first place and takes the new value.
  void member(frame &f, value_t v) {
    auto &d = *std::get<dict_ptr>(f.container.v);
    if (f.index.empty()) {
      for (auto &[k, old] : d)
        if (k == f.key) {
          old = std::move(v);
          return;
        }
      d.emplace_back(std::move(f.key), std::move(v));
      if (d.size() == kIndexedKeys)
        for (std::size_t i = 0; i < d.size(); ++i)
          f.index.emplace(d[i].first, i);
      return;
    }
    const auto [it, fresh] = f.index.try_emplace(f.key, d.size());
    if (!fresh) {
      d[it->second].second = std::move(v);
      return;
    }
    d.emplace_back(std::move(f.key), std::move(v));
  }

  void open(bool array) {
    if (stack_.size() >= kJsonMaxDepth)
      fail("nested deeper than " + std::to_string(kJsonMaxDepth) + " levels");
    ++pos_;
    value_t c = array ? make_list() : make_dict();
    bytes_ += kContainerBytes;
    put(c); // into its parent, which holds it from now on
    stack_.push_back(frame{std::move(c), array, {}, {}});
    want_ = array ? want::first_item : want::first_key;
  }

  void close() {
    stack_.pop_back();
    want_ = stack_.empty() ? want::end : want::next;
  }

  void key(const char *msg) {
    if (!have('"'))
      fail(msg);
    frame &f = stack_.back();
    f.key.clear();
    string(f.key);
    skip_ws();
    if (!have(':'))
      fail("expected ':'");
    want_ = want::value;
  }

  // The rest of a string, after its opening quote.
  void string(std::string &out) {
    const std::size_t quote = pos_ - 1;
    for (;;) {
      std::size_t run = pos_;
      while (run < doc_.size()) {
        const auto c = static_cast<unsigned char>(doc_[run]);
        if (c == '"' || c == '\\' || c < 0x20 || c >= 0x80)
          break;
        ++run;
      }
      out.append(doc_.data() + pos_, run - pos_);
      pos_ = run;
      if (pos_ >= doc_.size())
        fail_at(quote, "unterminated string");
      const auto c = static_cast<unsigned char>(doc_[pos_]);
      if (c == '"') {
        ++pos_;
        return;
      }
      if (c == '\\') {
        ++pos_;
        escape(out, quote);
        continue;
      }
      if (c < 0x20)
        fail("control character in a string (it must be escaped)");
      char32_t cp = 0;
      std::size_t len = 0;
      if (utf8::decode(doc_, pos_, cp, len))
        out.append(doc_.data() + pos_, len);
      else
        utf8::encode(utf8::kReplacement, out);
      pos_ += len;
    }
  }

  // An escape, after its backslash.
  void escape(std::string &out, std::size_t quote) {
    if (pos_ >= doc_.size())
      fail_at(quote, "unterminated string");
    switch (doc_[pos_++]) {
    case '"':
      out += '"';
      return;
    case '\\':
      out += '\\';
      return;
    case '/':
      out += '/';
      return;
    case 'b':
      out += '\b';
      return;
    case 'f':
      out += '\f';
      return;
    case 'n':
      out += '\n';
      return;
    case 'r':
      out += '\r';
      return;
    case 't':
      out += '\t';
      return;
    case 'u':
      break;
    default:
      fail_at(pos_ - 2, "invalid escape");
    }
    const char32_t cp = hex4();
    if (cp >= 0xD800 && cp <= 0xDBFF && pos_ + 1 < doc_.size() && doc_[pos_] == '\\' &&
        doc_[pos_ + 1] == 'u') {
      // A high surrogate and a low one are one code point. A high one before any other escape
      // is alone, and that escape stands on its own.
      const std::size_t after = pos_;
      pos_ += 2;
      const char32_t low = hex4();
      if (low >= 0xDC00 && low <= 0xDFFF) {
        utf8::encode(0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00), out);
        return;
      }
      pos_ = after;
    }
    utf8::encode(cp, out); // a lone surrogate is U+FFFD
  }

  char32_t hex4() {
    char32_t cp = 0;
    for (int i = 0; i < 4; ++i, ++pos_) {
      const char c = pos_ < doc_.size() ? doc_[pos_] : '\0';
      int v = -1;
      if (c >= '0' && c <= '9')
        v = c - '0';
      else if (c >= 'a' && c <= 'f')
        v = c - 'a' + 10;
      else if (c >= 'A' && c <= 'F')
        v = c - 'A' + 10;
      if (v < 0)
        fail("invalid \\u escape (four hex digits)");
      cp = cp * 16 + static_cast<char32_t>(v);
    }
    return cp;
  }

  value_t number() {
    const std::size_t start = pos_;
    const auto digit = [this] {
      return pos_ < doc_.size() && doc_[pos_] >= '0' && doc_[pos_] <= '9';
    };
    const auto digits = [&] {
      if (!digit())
        fail("invalid number");
      while (digit())
        ++pos_;
    };
    bool integral = true;
    if (doc_[pos_] == '-')
      ++pos_;
    if (pos_ < doc_.size() && doc_[pos_] == '0')
      ++pos_; // no leading zeros: "01" is 0, then text the caller refuses
    else
      digits();
    if (pos_ < doc_.size() && doc_[pos_] == '.') {
      integral = false;
      ++pos_;
      digits();
    }
    if (pos_ < doc_.size() && (doc_[pos_] == 'e' || doc_[pos_] == 'E')) {
      integral = false;
      ++pos_;
      if (pos_ < doc_.size() && (doc_[pos_] == '+' || doc_[pos_] == '-'))
        ++pos_;
      digits();
    }
    const std::string_view text = doc_.substr(start, pos_ - start);
    if (integral) {
      int64_t i = 0;
      const auto [end, ec] = std::from_chars(text.data(), text.data() + text.size(), i);
      if (ec == std::errc{} && end == text.data() + text.size())
        return value_t(i);
      // Past 64 bits: a float, as JavaScript reads it.
    }
    // The grammar is checked, so strtod reads all of it. (from_chars for double isn't everywhere.)
    const std::string copy(text);
    const double d = std::strtod(copy.c_str(), nullptr);
    if (!std::isfinite(d))
      fail_at(start, "number out of range");
    return value_t(d);
  }

  void literal(std::string_view word) {
    if (doc_.substr(pos_, word.size()) != word)
      fail("expected a value");
    pos_ += word.size();
  }

  void skip_ws() {
    while (pos_ < doc_.size() &&
           (doc_[pos_] == ' ' || doc_[pos_] == '\t' || doc_[pos_] == '\n' || doc_[pos_] == '\r'))
      ++pos_;
  }

  bool have(char c) {
    if (pos_ < doc_.size() && doc_[pos_] == c) {
      ++pos_;
      return true;
    }
    return false;
  }

  [[noreturn]] void fail(const std::string &msg) const { fail_at(pos_, msg); }

  [[noreturn]] void fail_at(std::size_t at, const std::string &msg) const {
    int64_t line = 1, col = 1;
    for (std::size_t i = 0; i < at && i < doc_.size(); ++i) {
      const auto c = static_cast<unsigned char>(doc_[i]);
      if (c == '\n') {
        ++line;
        col = 1;
      } else if (!utf8::is_continuation(c)) {
        ++col;
      }
    }
    throw dsl_error("ValueError",
                    "json-parse: line " + std::to_string(line) + ", column " + std::to_string(col) +
                        ": " + msg,
                    "builtin",
                    {{"line", value_t(line)},
                     {"col", value_t(col)},
                     {"offset", value_t(static_cast<int64_t>(at))}});
  }

  std::string own_;
  std::string_view doc_;
  std::size_t pos_ = 0;
  want want_ = want::value;
  std::vector<frame> stack_;
  value_t root_;
  std::size_t values_ = 0;
  std::size_t bytes_ = 0; // the estimate held to the allocation limit
  std::size_t noted_ = 0; // bytes reported to heap accounting so far
};

// -- encode ------------------------------------------------------------------

// The shortest text that reads back as `d`, with ".0" when it has neither a point nor an
// exponent, so json-parse reads it back as a float.
std::string float_text(double d) {
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  char buf[32];
  const auto r = std::to_chars(buf, buf + sizeof(buf), d);
  std::string s(buf, r.ptr);
#else
  std::string s = detail::format_number_shortest_fallback(d);
#endif
  if (s.find_first_of(".eE") == std::string::npos)
    s += ".0";
  return s;
}

class json_encode_walk final : public data_job {
public:
  json_encode_walk(value_t v, std::size_t indent) : root_(std::move(v)), indent_(indent) {
    if (indent > kJsonMaxIndent)
      throw dsl_error("ValueError", "json-encode: indent is 0 to " +
                                        std::to_string(kJsonMaxIndent) + " spaces, got " +
                                        std::to_string(indent));
  }

  bool step(std::size_t budget) override {
    const std::size_t start = nodes_;
    const std::optional<std::size_t> limit = eval_alloc_limit();
    if (!started_) {
      started_ = true;
      emit(root_);
      account(limit);
    }
    while (!open_.empty() && (budget == 0 || nodes_ - start < budget)) {
      cursor &c = open_.back();
      const std::size_t i = c.next++;
      if (i >= c.size()) {
        close();
        continue;
      }
      ++elements_;
      eval_deadline_check("json-encode");
      if (i > 0)
        out_ += ',';
      newline();
      if (c.list) {
        emit((*c.list)[i]); // may push, invalidating c
      } else {
        const auto &[key, value] = (*c.dict)[i];
        string(key);
        out_ += indent_ ? ": " : ":";
        emit(value); // may push, invalidating c
      }
      account(limit);
    }
    eval_alloc_note(out_.size() - noted_);
    noted_ = out_.size();
    return open_.empty();
  }

  value_t result() override { return value_t(std::move(out_)); }

  void held_values(const std::function<void(const value_t &)> &visit) const override {
    visit(root_);
  }

  std::size_t held_bytes() const override {
    return sizeof(*this) + out_.capacity() + open_.capacity() * sizeof(cursor) +
           on_path_.size() * 2 * sizeof(void *);
  }

private:
  // A list or dict being written, held by owning pointer and read by index.
  struct cursor {
    list_ptr list;
    dict_ptr dict;
    std::size_t next = 0;
    std::size_t size() const { return list ? list->size() : dict->size(); }
    const void *id() const {
      return list ? static_cast<const void *>(list.get()) : static_cast<const void *>(dict.get());
    }
  };

  // A node per element, and per kDataNodeBytes written; the text is held to the allocation limit.
  void account(const std::optional<std::size_t> &limit) {
    nodes_ = elements_ + out_.size() / kDataNodeBytes;
    if (limit && out_.size() > *limit)
      eval_alloc_check(out_.size(), "json-encode"); // throws
  }

  // Write `v`; a non-empty list or dict is only opened, and step() writes its elements.
  void emit(const value_t &v) {
    if (v.is_nil()) {
      out_ += "null";
    } else if (const auto *b = std::get_if<bool>(&v.v)) {
      out_ += *b ? "true" : "false";
    } else if (const auto *i = std::get_if<int64_t>(&v.v)) {
      out_ += std::to_string(*i);
    } else if (const auto *d = std::get_if<double>(&v.v)) {
      if (!std::isfinite(*d))
        throw dsl_error("ValueError", "json-encode: " + to_string(v) + " is not a JSON number");
      out_ += float_text(*d);
    } else if (const auto *s = std::get_if<std::string>(&v.v)) {
      string(*s);
    } else if (const auto *l = std::get_if<list_ptr>(&v.v)) {
      if (!*l || (*l)->empty())
        empty("[]");
      else
        open(cursor{*l, nullptr, 0});
    } else if (const auto *m = std::get_if<dict_ptr>(&v.v)) {
      if (!*m || (*m)->empty())
        empty("{}");
      else
        open(cursor{nullptr, *m, 0});
    } else {
      throw dsl_error("TypeError", "json-encode: a " + v.type_name() + " has no JSON form");
    }
  }

  // A list or dict is a level, empty or not, as json-parse counts them.
  void check_depth() const {
    if (open_.size() >= kJsonMaxDepth)
      throw dsl_error("ValueError", "json-encode: nested deeper than " +
                                        std::to_string(kJsonMaxDepth) + " levels");
  }

  void empty(const char *brackets) {
    check_depth();
    out_ += brackets;
  }

  void open(cursor c) {
    check_depth();
    if (!on_path_.insert(c.id()).second)
      throw dsl_error("ValueError", std::string("json-encode: a ") + (c.list ? "list" : "dict") +
                                        " that contains itself");
    out_ += c.list ? '[' : '{';
    open_.push_back(std::move(c));
  }

  void close() {
    const bool list = open_.back().list != nullptr;
    on_path_.erase(open_.back().id());
    open_.pop_back();
    newline();
    out_ += list ? ']' : '}';
  }

  // With an indent, a line break and the indent of the open level.
  void newline() {
    if (!indent_)
      return;
    out_ += '\n';
    out_.append(indent_ * open_.size(), ' ');
  }

  void string(const std::string &s) {
    if (!utf8::is_valid(s))
      throw dsl_error("ValueError", "json-encode: a string that isn't UTF-8 text (bytes?)");
    static const char kHex[] = "0123456789abcdef";
    out_ += '"';
    for (const char ch : s) {
      const auto c = static_cast<unsigned char>(ch);
      switch (c) {
      case '"':
        out_ += "\\\"";
        break;
      case '\\':
        out_ += "\\\\";
        break;
      case '\b':
        out_ += "\\b";
        break;
      case '\f':
        out_ += "\\f";
        break;
      case '\n':
        out_ += "\\n";
        break;
      case '\r':
        out_ += "\\r";
        break;
      case '\t':
        out_ += "\\t";
        break;
      default:
        if (c < 0x20) {
          out_ += "\\u00";
          out_ += kHex[c >> 4];
          out_ += kHex[c & 0x0F];
        } else {
          out_ += ch;
        }
      }
    }
    out_ += '"';
  }

  value_t root_;
  std::size_t indent_;
  bool started_ = false;
  std::string out_;
  std::vector<cursor> open_;
  std::unordered_set<const void *> on_path_; // the lists and dicts open now, for cycles
  std::size_t elements_ = 0;
  std::size_t noted_ = 0; // text bytes reported to heap accounting so far
};

} // namespace

std::unique_ptr<data_job> make_json_parse_job(std::string text) {
  return std::make_unique<json_parse_walk>(std::move(text));
}

std::unique_ptr<data_job> make_json_encode_job(value_t v, std::size_t indent) {
  return std::make_unique<json_encode_walk>(std::move(v), indent);
}

value_t builtin_json_parse(std::span<const value_t> args) {
  if (args.size() != 1)
    throw dsl_error("TypeError",
                    "json-parse: expected 1 argument(s), got " + std::to_string(args.size()));
  std::string_view text;
  if (const auto *s = std::get_if<std::string>(&args[0].v))
    text = *s;
  else if (const auto *b = std::get_if<bytes_value>(&args[0].v))
    text = b->data; // a body that came as bytes: JSON is UTF-8
  else
    throw dsl_error("TypeError",
                    "json-parse: expected string or bytes, got " + args[0].type_name());
  return run_data_job(std::make_unique<json_parse_walk>(text, json_parse_walk::borrow{}));
}

value_t builtin_json_encode(std::span<const value_t> args) {
  if (args.empty() || args.size() > 2)
    throw dsl_error("TypeError", "json-encode: expected a value and an optional indent, got " +
                                     std::to_string(args.size()) + " argument(s)");
  std::size_t indent = 0;
  if (args.size() == 2) {
    const value_t &a = args[1];
    if (const auto *b = std::get_if<bool>(&a.v)) {
      indent = *b ? 2 : 0;
    } else if (const auto *i = std::get_if<int64_t>(&a.v)) {
      if (*i < 0 || *i > static_cast<int64_t>(kJsonMaxIndent))
        throw dsl_error("ValueError", "json-encode: indent is 0 to " +
                                          std::to_string(kJsonMaxIndent) + " spaces, got " +
                                          std::to_string(*i));
      indent = static_cast<std::size_t>(*i);
    } else if (!a.is_nil()) {
      throw dsl_error("TypeError", "json-encode: indent is true, nil or a number of spaces, got " +
                                       a.type_name());
    }
  }
  return run_data_job(std::make_unique<json_encode_walk>(args[0], indent));
}

} // namespace cvc::state_exec
