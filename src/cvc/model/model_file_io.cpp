/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/regex.hpp>
#include <cvc/model/model_file_io.h>

namespace cvc {
// A regex to extract a filename extension
const char *model_file_io::FILE_EXTENSION_EXPR = "^(.*)(\\.\\S*)$";

// ----------------------
// model_file_io::extents
// ----------------------
// Purpose:
//   Returns the smallest bounding box that includes all vertices in the file.
//   This default implementation reads the entire file and computes it.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
bounding_box model_file_io::extents(const std::string &filename) {
  return read(filename).extents();
}

// ---------------------------
// model_file_io::get_handlers
// ---------------------------
// Purpose:
//   Static initialization of handler map.  Clients use the handler_map
//   to add themselves to the collection of objects that are to be used
//   to perform model file i/o operations.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
model_file_io::handler_map &model_file_io::get_handlers() {
  // It's ok to leak: http://www.parashift.com/c++-faq-lite/ctors.html#faq-10.15
  static handler_map *p = initialize_map();

  // Ensure the default model I/O handlers are registered the first time the
  // handler map is requested. The handler register_* helpers call back into
  // insert_handler() and therefore re-enter get_handlers(); an explicit guard
  // avoids re-entering the same function-local static initialiser, which would
  // be undefined behaviour.
  static bool _registering_defaults = false;
  static bool _defaults_registered = false;
  if (!_defaults_registered && !_registering_defaults) {
    _registering_defaults = true;
    register_default_model_handlers();
    _defaults_registered = true;
    _registering_defaults = false;
  }

  return *p;
}

// -----------------------------
// model_file_io::insert_handler
// -----------------------------
// Purpose:
//   Convenence function for adding objects to the map.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
void model_file_io::insert_handler(const ptr &mfio) { insert_handler(get_handlers(), mfio); }

// -----------------------------
// model_file_io::remove_handler
// -----------------------------
// Purpose:
//   Convenence function for removing objects from the map.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
void model_file_io::remove_handler(const ptr &mfio) {
  for (handler_map::iterator i = get_handlers().begin(); i != get_handlers().end(); i++) {
    handlers h;
    for (handlers::iterator j = i->second.begin(); j != i->second.end(); j++) {
      if (*j != mfio)
        h.push_back(*j);
    }
    i->second = h;
  }
}

// -----------------------------
// model_file_io::remove_handler
// -----------------------------
// Purpose:
//   Convenence function for removing objects from the map.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
void model_file_io::remove_handler(const std::string &id) {
  for (handler_map::iterator i = get_handlers().begin(); i != get_handlers().end(); i++) {
    handlers h;
    for (handlers::iterator j = i->second.begin(); j != i->second.end(); j++) {
      if ((*j)->id() != id)
        h.push_back(*j);
    }
    i->second = h;
  }
}

// -----------------------------
// model_file_io::get_extensions
// -----------------------------
// Purpose:
//   Returns the list of supported file extensions.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
std::vector<std::string> model_file_io::get_extensions() {
  std::vector<std::string> ret;
  BOOST_FOREACH (handler_map::value_type &i, get_handlers()) {
    ret.push_back(i.first);
  }
  return ret;
}

// -----------------------------
// model_file_io::initialize_map
// -----------------------------
// Purpose:
//   Adds the standard model_file_io objects to a new handler_map object
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
model_file_io::handler_map *model_file_io::initialize_map() {
  handler_map *map = new handler_map;
  return map;
}

// -----------------------------
// model_file_io::insert_handler
// -----------------------------
// Purpose:
//   Convenence function for adding objects to the specified map.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
void model_file_io::insert_handler(handler_map &hm, const ptr &mfio) {
  if (!mfio)
    return;
  for (model_file_io::extension_list::const_iterator i = mfio->extensions().begin();
       i != mfio->extensions().end(); i++) {
    hm[*i].push_back(mfio);
  }
}

// ----------
// read_model
// ----------
// Purpose:
//   The main read model function.  Refers to the handler map to choose
//   an appropriate IO object for reading the requested model file.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
model read_model(const std::string &filename) {
  using namespace std;
  using namespace boost;
  string errors;
  smatch what;
  const regex file_extension(model_file_io::FILE_EXTENSION_EXPR);
  if (regex_match(filename, what, file_extension)) {
    if (model_file_io::get_handlers()[what[2]].empty())
      throw unsupported_model_file_type(string(BOOST_CURRENT_FUNCTION) + string(": Cannot read ") +
                                        filename);
    model_file_io::handlers &h = model_file_io::get_handlers()[what[2]];
    // use the first handler that succeds
    for (model_file_io::handlers::iterator i = h.begin(); i != h.end(); i++)
      try {
        if (*i)
          return (*i)->read(filename);
      } catch (exception &e) {
        errors += string(" :: ") + e.what();
      }
  }
  throw unsupported_model_file_type(str(boost::format("%1% : Cannot read '%2%'%3%") %
                                        BOOST_CURRENT_FUNCTION % filename % errors));
}

// -----------
// write_model
// -----------
// Purpose:
//   The main write model function.  Refers to the handler map to choose
//   an appropriate IO object for writing the requested model file.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
void write_model(const model &m, const std::string &filename) {
  using namespace std;
  using namespace boost;
  string errors;
  smatch what;
  const regex file_extension(model_file_io::FILE_EXTENSION_EXPR);
  if (regex_match(filename, what, file_extension)) {
    if (model_file_io::get_handlers()[what[2]].empty())
      throw unsupported_model_file_type(string(BOOST_CURRENT_FUNCTION) + string(": Cannot write ") +
                                        filename);
    model_file_io::handlers &h = model_file_io::get_handlers()[what[2]];
    // use the first handler that succeds
    for (model_file_io::handlers::iterator i = h.begin(); i != h.end(); i++)
      try {
        if (*i)
          return (*i)->write(m, filename);
      } catch (exception &e) {
        errors += string(" :: ") + e.what();
      }
  }
  throw unsupported_model_file_type(str(boost::format("%1% : Cannot write '%2%'%3%") %
                                        BOOST_CURRENT_FUNCTION % filename % errors));
}
} // namespace cvc
