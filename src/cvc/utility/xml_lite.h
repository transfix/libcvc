/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::xml_lite: a small XML reader for the legacy formats libcvc reads (Reconstruct series and
// sections, VolumeGridRover's .vgr point classes), private to libcvc. It parses a whole document
// into a tree of elements: names, attributes in order, text and children. Comments, processing
// instructions, the XML declaration and a DOCTYPE are skipped; CDATA becomes text; the five named
// entities and character references are decoded. No namespaces, no DTD, no external entities.
//
// It keeps no state of its own (boost::property_tree's XML parser keeps function-local statics,
// which the writable-data scan would flag: VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.3).

#ifndef CVC_UTILITY_XML_LITE_H
#define CVC_UTILITY_XML_LITE_H

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace xml_lite {

// A document that isn't well-formed XML: what was wrong, and the line it was found on.
class parse_error : public std::runtime_error {
public:
  parse_error(const std::string &what, std::size_t line)
      : std::runtime_error(what + " at line " + std::to_string(line)), _line(line) {}
  std::size_t line() const { return _line; }

private:
  std::size_t _line;
};

struct element {
  std::string name;
  std::vector<std::pair<std::string, std::string>> attributes; // in document order
  std::string text; // the character data directly inside it, concatenated
  std::vector<element> children;
  std::size_t line = 0; // where its start tag is

  // The attribute's value, or `fallback` when it has none of that name.
  std::string attribute(const std::string &name, const std::string &fallback) const;
  bool has_attribute(const std::string &name) const;
  // The first child of that name, or null.
  const element *child(const std::string &name) const;
};

// The document's root element. Throws parse_error.
element parse(const std::string &text);

// `s` with &, <, >, " and ' escaped, for an attribute value or text.
std::string escape(const std::string &s);

} // namespace xml_lite
} // namespace cvc

#endif // CVC_UTILITY_XML_LITE_H
