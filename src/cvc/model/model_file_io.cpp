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
#include <cvc/core/app.h>
#include <cvc/model/model_file_io.h>

namespace cvc {
// A regex to extract a filename extension
const char *const model_file_io::FILE_EXTENSION_EXPR = "^(.*)(\\.\\S*)$";

// ----------------------
// model_file_io::extents
// ----------------------
// Purpose:
//   Returns the smallest bounding box that includes all vertices in the file.
//   This default implementation reads the entire file and computes it.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
bounding_box model_file_io::extents(app &ctx, const std::string &filename) {
  return read(ctx, filename).extents();
}

// ----------
// read_model
// ----------
// Purpose:
//   The main read model function.  Refers to ctx's model handlers to choose
//   an appropriate IO object for reading the requested model file.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- The handlers come from ctx's own table (app::model_handlers()).
model read_model(app &ctx, const std::string &filename) {
  using namespace std;
  using namespace boost;
  string errors;
  smatch what;
  const regex file_extension(model_file_io::FILE_EXTENSION_EXPR);
  if (regex_match(filename, what, file_extension)) {
    const model_file_io::handlers h = ctx.model_handlers().for_extension(what[2].str());
    if (h.empty())
      throw unsupported_model_file_type(string(BOOST_CURRENT_FUNCTION) + string(": Cannot read ") +
                                        filename);
    // use the first handler that succeds
    for (model_file_io::handlers::const_iterator i = h.begin(); i != h.end(); i++)
      try {
        if (*i)
          return (*i)->read(ctx, filename);
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
//   The main write model function.  Refers to ctx's model handlers to choose
//   an appropriate IO object for writing the requested model file.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- The handlers come from ctx's own table (app::model_handlers()).
void write_model(app &ctx, const model &m, const std::string &filename) {
  using namespace std;
  using namespace boost;
  string errors;
  smatch what;
  const regex file_extension(model_file_io::FILE_EXTENSION_EXPR);
  if (regex_match(filename, what, file_extension)) {
    const model_file_io::handlers h = ctx.model_handlers().for_extension(what[2].str());
    if (h.empty())
      throw unsupported_model_file_type(string(BOOST_CURRENT_FUNCTION) + string(": Cannot write ") +
                                        filename);
    // use the first handler that succeds
    for (model_file_io::handlers::const_iterator i = h.begin(); i != h.end(); i++)
      try {
        if (*i)
          return (*i)->write(ctx, m, filename);
      } catch (exception &e) {
        errors += string(" :: ") + e.what();
      }
  }
  throw unsupported_model_file_type(str(boost::format("%1% : Cannot write '%2%'%3%") %
                                        BOOST_CURRENT_FUNCTION % filename % errors));
}
} // namespace cvc
