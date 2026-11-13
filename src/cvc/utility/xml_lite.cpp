/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See xml_lite.h.

#include "xml_lite.h"

#include <cstdint>

namespace cvc {
namespace xml_lite {

namespace {

bool is_space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

bool is_name_char(char c) {
  const unsigned char u = static_cast<unsigned char>(c);
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
         c == ':' || c == '-' || c == '.' || u >= 0x80;
}

void append_utf8(std::string &out, std::uint32_t cp) {
  if (cp < 0x80) {
    out += static_cast<char>(cp);
  } else if (cp < 0x800) {
    out += static_cast<char>(0xC0 | (cp >> 6));
    out += static_cast<char>(0x80 | (cp & 0x3F));
  } else if (cp < 0x10000) {
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

class parser {
public:
  explicit parser(const std::string &text) : _s(text) {}

  element document() {
    prolog();
    if (!at('<'))
      fail(_i >= _s.size() ? "no root element" : "text before the root element");
    element root = parse_element();
    misc();
    if (_i < _s.size())
      fail("more after the root element");
    return root;
  }

private:
  [[noreturn]] void fail(const std::string &what) const { throw parse_error(what, _line); }

  bool at(char c) const { return _i < _s.size() && _s[_i] == c; }
  bool starts(const char *lit) const {
    return _s.compare(_i, std::char_traits<char>::length(lit), lit) == 0;
  }

  void advance(std::size_t n = 1) {
    for (std::size_t k = 0; k < n && _i < _s.size(); ++k)
      if (_s[_i++] == '\n')
        ++_line;
  }

  void skip_space() {
    while (_i < _s.size() && is_space(_s[_i]))
      advance();
  }

  // Skip past `end`, or fail with `what` when the document ends first.
  void skip_past(const char *end, const char *what) {
    const std::size_t at = _s.find(end, _i);
    if (at == std::string::npos)
      fail(std::string("an unterminated ") + what);
    advance(at - _i + std::char_traits<char>::length(end));
  }

  // Comments, processing instructions and whitespace; with `doctype`, a DOCTYPE too.
  void misc(bool doctype = false) {
    while (true) {
      skip_space();
      if (starts("<!--"))
        skip_past("-->", "comment");
      else if (starts("<?"))
        skip_past("?>", "processing instruction");
      else if (doctype && starts("<!DOCTYPE"))
        skip_doctype();
      else
        return;
    }
  }

  void prolog() {
    if (starts("\xEF\xBB\xBF")) // a UTF-8 byte order mark
      advance(3);
    misc(true);
  }

  // <!DOCTYPE name ... [internal subset]>: skipped, brackets and quotes respected.
  void skip_doctype() {
    advance(9);
    int depth = 0;
    while (_i < _s.size()) {
      const char c = _s[_i];
      if (c == '"' || c == '\'') {
        advance();
        while (_i < _s.size() && _s[_i] != c)
          advance();
        if (_i >= _s.size())
          break;
      } else if (c == '[') {
        ++depth;
      } else if (c == ']') {
        --depth;
      } else if (c == '>' && depth <= 0) {
        advance();
        return;
      }
      advance();
    }
    fail("an unterminated DOCTYPE");
  }

  std::string name() {
    const std::size_t b = _i;
    while (_i < _s.size() && is_name_char(_s[_i]))
      advance();
    if (_i == b)
      fail("a name was expected");
    return _s.substr(b, _i - b);
  }

  // Character data up to `stop` (exclusive), entities decoded.
  std::string chars(char stop) {
    std::string out;
    while (_i < _s.size() && _s[_i] != stop) {
      if (_s[_i] == '&') {
        entity(out);
      } else if (_s[_i] == '<') {
        fail("a '<' inside a value");
      } else {
        out += _s[_i];
        advance();
      }
    }
    return out;
  }

  void entity(std::string &out) {
    const std::size_t semi = _s.find(';', _i);
    if (semi == std::string::npos || semi - _i > 12)
      fail("an unterminated entity");
    const std::string e = _s.substr(_i + 1, semi - _i - 1);
    if (e == "amp")
      out += '&';
    else if (e == "lt")
      out += '<';
    else if (e == "gt")
      out += '>';
    else if (e == "quot")
      out += '"';
    else if (e == "apos")
      out += '\'';
    else if (e.size() > 1 && e[0] == '#') {
      const bool hex = e[1] == 'x' || e[1] == 'X';
      const std::string digits = e.substr(hex ? 2 : 1);
      std::uint32_t cp = 0;
      if (digits.empty())
        fail("an empty character reference");
      for (const char c : digits) {
        int d = -1;
        if (c >= '0' && c <= '9')
          d = c - '0';
        else if (hex && c >= 'a' && c <= 'f')
          d = c - 'a' + 10;
        else if (hex && c >= 'A' && c <= 'F')
          d = c - 'A' + 10;
        if (d < 0)
          fail("a bad character reference '&" + e + ";'");
        cp = cp * (hex ? 16 : 10) + static_cast<std::uint32_t>(d);
        if (cp > 0x10FFFF)
          fail("a character reference out of range");
      }
      append_utf8(out, cp);
    } else {
      fail("an unknown entity '&" + e + ";'");
    }
    advance(semi - _i + 1);
  }

  element parse_element() {
    element el;
    el.line = _line;
    advance(); // '<'
    el.name = name();
    while (true) {
      skip_space();
      if (starts("/>")) {
        advance(2);
        return el;
      }
      if (at('>')) {
        advance();
        break;
      }
      if (_i >= _s.size())
        fail("the document ends inside <" + el.name + ">");
      std::string key = name();
      skip_space();
      if (!at('='))
        fail("attribute '" + key + "' of <" + el.name + "> has no value");
      advance();
      skip_space();
      if (!at('"') && !at('\''))
        fail("attribute '" + key + "' of <" + el.name + "> is not quoted");
      const char q = _s[_i];
      advance();
      std::string value = chars(q);
      if (!at(q))
        fail("attribute '" + key + "' of <" + el.name + "> is not closed");
      advance();
      for (const auto &kv : el.attributes)
        if (kv.first == key)
          fail("attribute '" + key + "' of <" + el.name + "> appears twice");
      el.attributes.emplace_back(std::move(key), std::move(value));
    }
    // Content: text, children, comments, CDATA, until the end tag.
    while (true) {
      if (_i >= _s.size())
        fail("<" + el.name + "> is not closed");
      if (starts("</")) {
        advance(2);
        const std::string end = name();
        if (end != el.name)
          fail("</" + end + "> closes <" + el.name + ">");
        skip_space();
        if (!at('>'))
          fail("</" + end + " is not closed");
        advance();
        return el;
      }
      if (starts("<!--")) {
        skip_past("-->", "comment");
      } else if (starts("<![CDATA[")) {
        advance(9);
        const std::size_t end = _s.find("]]>", _i);
        if (end == std::string::npos)
          fail("an unterminated CDATA section");
        el.text += _s.substr(_i, end - _i);
        advance(end - _i + 3);
      } else if (starts("<?")) {
        skip_past("?>", "processing instruction");
      } else if (at('<')) {
        el.children.push_back(parse_element());
      } else {
        el.text += chars('<');
      }
    }
  }

  const std::string &_s;
  std::size_t _i = 0;
  std::size_t _line = 1;
};

} // namespace

std::string element::attribute(const std::string &name, const std::string &fallback) const {
  for (const auto &kv : attributes)
    if (kv.first == name)
      return kv.second;
  return fallback;
}

bool element::has_attribute(const std::string &name) const {
  for (const auto &kv : attributes)
    if (kv.first == name)
      return true;
  return false;
}

const element *element::child(const std::string &name) const {
  for (const element &c : children)
    if (c.name == name)
      return &c;
  return nullptr;
}

element parse(const std::string &text) { return parser(text).document(); }

std::string escape(const std::string &s) {
  std::string out;
  out.reserve(s.size());
  for (const char c : s) {
    switch (c) {
    case '&':
      out += "&amp;";
      break;
    case '<':
      out += "&lt;";
      break;
    case '>':
      out += "&gt;";
      break;
    case '"':
      out += "&quot;";
      break;
    case '\'':
      out += "&apos;";
      break;
    default:
      out += c;
    }
  }
  return out;
}

} // namespace xml_lite
} // namespace cvc
