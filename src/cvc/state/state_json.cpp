/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <boost/format.hpp>
#include <boost/property_tree/json_parser/error.hpp>
#include <boost/thread/thread.hpp>
#include <cvc/core/app.h>
#include <cvc/state/state_json.h>

namespace cvc {

// ---------------------------------------------------------------------------
// state_subtree_cursor
// ---------------------------------------------------------------------------

state_subtree_cursor::state_subtree_cursor(state &start)
    : keep_(start.weak_from_this().lock()), start_(&start) {}

state_subtree_cursor::level state_subtree_cursor::open(state &n, const std::string &rel,
                                                       std::string &value) {
  level l;
  {
    boost::mutex::scoped_lock lock(n._mutex);
    value = n._value;
    l.kids.reserve(n._children.size());
    for (const state::child_map::value_type &kv : n._children)
      if (kv.second)
        l.kids.emplace_back(kv.second, rel.empty() ? kv.first : rel + state::SEPARATOR + kv.first);
  }
  n._ctx.log(6, boost::str(boost::format("cvc::state_subtree_cursor :: %s = %s\n") % rel % value));
  return l;
}

state_subtree_cursor::visit state_subtree_cursor::next(std::string &path, std::string &value) {
  boost::this_thread::interruption_point();
  if (!started_) {
    started_ = true;
    ++visited_;
    path.clear();
    stack_.push_back(open(*start_, path, value));
    return value.empty() ? visit::unlisted : visit::listed;
  }
  while (!stack_.empty()) {
    level &l = stack_.back();
    if (l.next == l.kids.size()) {
      stack_.pop_back();
      continue;
    }
    std::pair<state::state_ptr, std::string> kid = std::move(l.kids[l.next++]);
    ++visited_;
    path = std::move(kid.second);
    level opened = open(*kid.first, path, value);
    const bool childless = opened.kids.empty();
    stack_.push_back(std::move(opened)); // invalidates `l`
    return !value.empty() || childless ? visit::listed : visit::unlisted;
  }
  return visit::done;
}

// ---------------------------------------------------------------------------
// state_json_writer
// ---------------------------------------------------------------------------

namespace {

// boost::property_tree::json_parser::create_escapes(), byte for byte: printable ASCII and every
// byte from 0x5D up (so UTF-8 passes through) stay; " \ / and the named controls get their
// escapes; any other control byte becomes \u00XX.
void append_escaped(std::string &out, const std::string &s) {
  static const char kHex[] = "0123456789ABCDEF";
  for (const char ch : s) {
    const unsigned char c = static_cast<unsigned char>(ch);
    if (c == 0x20 || c == 0x21 || (c >= 0x23 && c <= 0x2E) || (c >= 0x30 && c <= 0x5B) || c >= 0x5D)
      out += ch;
    else if (ch == '\b')
      out += "\\b";
    else if (ch == '\f')
      out += "\\f";
    else if (ch == '\n')
      out += "\\n";
    else if (ch == '\r')
      out += "\\r";
    else if (ch == '\t')
      out += "\\t";
    else if (ch == '/')
      out += "\\/";
    else if (ch == '"')
      out += "\\\"";
    else if (ch == '\\')
      out += "\\\\";
    else {
      out += "\\u00";
      out += kHex[c >> 4];
      out += kHex[c & 0x0F];
    }
  }
}

} // namespace

// write_json()'s pretty layout for a one-level tree: "{\n", then each entry as
// `    "key": "value"` with ",\n" between entries and "\n" after the last, then "}\n".
bool state_json_writer::step(std::size_t nodes) {
  if (done_)
    return true;
  for (std::size_t n = 0; nodes == 0 || n < nodes; ++n) {
    const state_subtree_cursor::visit v = cursor_.next(path_, value_);
    if (v == state_subtree_cursor::visit::done) {
      if (any_)
        out_ += '\n';
      out_ += "}\n";
      done_ = true;
      return true;
    }
    if (v == state_subtree_cursor::visit::unlisted)
      continue;
    out_ += any_ ? ",\n    \"" : "    \"";
    append_escaped(out_, path_);
    out_ += "\": \"";
    append_escaped(out_, value_);
    out_ += '"';
    any_ = true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// state_json_reader
// ---------------------------------------------------------------------------

namespace {

bool is_digit(char c) { return c >= '0' && c <= '9'; }

int hex_value(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return -1;
}

// How many continuation bytes a UTF-8 lead byte takes, or -1 when it cannot lead (read_json's
// utf8_utf8_encoding::trail_table).
int trailing_bytes(unsigned char c) {
  if (c >= 0xC0 && c <= 0xDF)
    return 1;
  if (c >= 0xE0 && c <= 0xEF)
    return 2;
  if (c >= 0xF0 && c <= 0xF7)
    return 3;
  return -1;
}

void append_utf8(std::string &out, unsigned cp) {
  if (cp <= 0x7F) {
    out += static_cast<char>(cp);
  } else if (cp <= 0x7FF) {
    out += static_cast<char>(0xC0 | (cp >> 6));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else if (cp <= 0xFFFF) {
    out += static_cast<char>(0xE0 | (cp >> 12));
    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else {
    out += static_cast<char>(0xF0 | (cp >> 18));
    out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  }
}

} // namespace

state_json_reader::state_json_reader(state &target, std::string_view doc, std::string filename)
    : keep_(target.weak_from_this().lock()), target_(&target), doc_(doc),
      filename_(std::move(filename)) {}

void state_json_reader::fail(const char *msg) const {
  BOOST_PROPERTY_TREE_THROW(
      boost::property_tree::json_parser::json_parser_error(msg, filename_, line_));
}

void state_json_reader::advance() {
  if (doc_[pos_] == '\n')
    ++line_;
  ++pos_;
}

bool state_json_reader::have(char c) {
  if (at_end() || peek() != c)
    return false;
  advance();
  return true;
}

void state_json_reader::expect(char c, const char *msg) {
  if (!have(c))
    fail(msg);
}

char state_json_reader::need(const char *msg) const {
  if (at_end())
    fail(msg);
  return peek();
}

void state_json_reader::skip_ws() {
  while (!at_end()) {
    const char c = peek();
    if (c != ' ' && c != '\t' && c != '\n' && c != '\r')
      return;
    advance();
  }
}

// The rest of a string whose opening quote was read.
void state_json_reader::scan_string(std::string &out) {
  for (;;) {
    const unsigned char c = static_cast<unsigned char>(need("unterminated string"));
    if (c == '"') {
      ++pos_;
      return;
    }
    if (c == '\\') {
      ++pos_;
      scan_escape(out);
      continue;
    }
    if (c <= 0x7F) {
      if (c < 0x20)
        fail("invalid code sequence");
      out += static_cast<char>(c);
      ++pos_;
      continue;
    }
    const int trailing = trailing_bytes(c);
    if (trailing < 0)
      fail("invalid code sequence");
    out += static_cast<char>(c);
    ++pos_;
    for (int i = 0; i < trailing; ++i) {
      if (at_end() || (static_cast<unsigned char>(peek()) & 0xC0) != 0x80)
        fail("invalid code sequence");
      out += peek();
      ++pos_;
    }
  }
}

unsigned state_json_reader::scan_hex_quad() {
  unsigned cp = 0;
  for (int i = 0; i < 4; ++i) {
    const int v = hex_value(need("invalid escape sequence"));
    if (v < 0)
      fail("invalid escape sequence");
    cp = cp * 16 + static_cast<unsigned>(v);
    ++pos_;
  }
  return cp;
}

void state_json_reader::scan_escape(std::string &out) {
  if (have('"'))
    out += '"';
  else if (have('\\'))
    out += '\\';
  else if (have('/'))
    out += '/';
  else if (have('b'))
    out += '\b';
  else if (have('f'))
    out += '\f';
  else if (have('n'))
    out += '\n';
  else if (have('r'))
    out += '\r';
  else if (have('t'))
    out += '\t';
  else if (have('u')) {
    unsigned cp = scan_hex_quad();
    if ((cp & 0xFC00) == 0xDC00)
      fail("invalid codepoint, stray low surrogate");
    if ((cp & 0xFC00) == 0xD800) {
      expect('\\', "invalid codepoint, stray high surrogate");
      expect('u', "expected codepoint reference after high surrogate");
      const unsigned low = scan_hex_quad();
      if ((low & 0xFC00) != 0xDC00)
        fail("expected low surrogate after high surrogate");
      cp = 0x10000 + (((cp & 0x3FF) << 10) | (low & 0x3FF));
    }
    append_utf8(out, cp);
  } else
    fail("invalid escape sequence");
}

// A string, true, false, null or number (kept as its text).
void state_json_reader::scan_scalar(std::string &out) {
  const char c = need("expected value");
  if (c == '"') {
    ++pos_;
    scan_string(out);
    return;
  }
  if (c == 't') {
    ++pos_;
    expect('r', "expected 'true'");
    expect('u', "expected 'true'");
    expect('e', "expected 'true'");
    out = "true";
    return;
  }
  if (c == 'f') {
    ++pos_;
    expect('a', "expected 'false'");
    expect('l', "expected 'false'");
    expect('s', "expected 'false'");
    expect('e', "expected 'false'");
    out = "false";
    return;
  }
  if (c == 'n') {
    ++pos_;
    expect('u', "expected 'null'");
    expect('l', "expected 'null'");
    expect('l', "expected 'null'");
    out = "null";
    return;
  }
  const std::size_t start = pos_;
  const bool minus = have('-');
  if (!have('0')) {
    if (at_end() || peek() < '1' || peek() > '9')
      fail(minus ? "expected digits after -" : "expected value");
    while (!at_end() && is_digit(peek()))
      ++pos_;
  }
  if (have('.')) {
    if (at_end() || !is_digit(peek()))
      fail("need at least one digit after '.'");
    while (!at_end() && is_digit(peek()))
      ++pos_;
  }
  if (!at_end() && (peek() == 'e' || peek() == 'E')) {
    ++pos_;
    if (!at_end() && (peek() == '+' || peek() == '-'))
      ++pos_;
    if (at_end() || !is_digit(peek()))
      fail("need at least one digit in exponent");
    while (!at_end() && is_digit(peek()))
      ++pos_;
  }
  out.assign(doc_.substr(start, pos_ - start));
}

void state_json_reader::emit(const std::string &text) {
  if (!applying_ || stack_.empty() || text.empty())
    return;
  const frame &f = stack_.back();
  (*target_)(f.array ? std::string() : f.key).value(text);
  ++applied_;
}

// The grammar is read_json()'s: a value, optionally preceded by a byte-order mark (a leading
// 0xEF skips three bytes, as read_json's does) and followed by nothing but whitespace (space,
// tab, CR, LF). The first pass checks the document; the second, identical one sets its values.
// The container stack is on the heap, so nesting costs no native stack.
bool state_json_reader::step(std::size_t values) {
  std::size_t n = 0;
  for (;;) {
    switch (want_) {
    case want::start:
      pos_ = 0;
      line_ = 1;
      stack_.clear();
      if (!doc_.empty() && static_cast<unsigned char>(doc_[0]) == 0xEF)
        pos_ = std::min<std::size_t>(3, doc_.size());
      want_ = want::value;
      break;
    case want::value: {
      if (values != 0 && n >= values) {
        bytes_ = scanned_ + pos_;
        return false;
      }
      ++n;
      ++values_;
      skip_ws();
      if (have('{')) {
        stack_.push_back(frame{false, {}});
        want_ = want::key_or_end;
      } else if (have('[')) {
        stack_.push_back(frame{true, {}});
        want_ = want::value_or_end;
      } else {
        std::string text;
        scan_scalar(text);
        emit(text);
        want_ = after_value();
      }
      break;
    }
    case want::key_or_end:
      skip_ws();
      if (have('}')) {
        stack_.pop_back();
        want_ = after_value();
      } else {
        want_ = want::key;
      }
      break;
    case want::value_or_end:
      skip_ws();
      if (have(']')) {
        stack_.pop_back();
        want_ = after_value();
      } else {
        want_ = want::value;
      }
      break;
    case want::key: {
      skip_ws();
      if (!have('"'))
        fail("expected key string");
      std::string &key = stack_.back().key;
      key.clear();
      scan_string(key);
      skip_ws();
      expect(':', "expected ':'");
      want_ = want::value;
      break;
    }
    case want::comma_or_end: {
      skip_ws();
      const bool array = stack_.back().array;
      if (have(',')) {
        want_ = array ? want::value : want::key;
        break;
      }
      expect(array ? ']' : '}', array ? "expected ']' or ','" : "expected '}' or ','");
      stack_.pop_back();
      want_ = after_value();
      break;
    }
    case want::end:
      skip_ws();
      if (!at_end())
        fail("garbage after data");
      scanned_ += pos_;
      bytes_ = scanned_;
      if (!applying_) {
        applying_ = true;
        want_ = want::start;
        break;
      }
      want_ = want::done;
      return true;
    case want::done:
      return true;
    }
  }
}

} // namespace cvc
