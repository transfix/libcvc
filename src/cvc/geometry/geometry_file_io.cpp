#include <boost/foreach.hpp>
#include <boost/regex.hpp>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/volume/io_handlers.h>

namespace cvc {
// A regex to extract a filename extension
const char *const geometry_file_io::FILE_EXTENSION_EXPR = "^(.*)(\\.\\S*)$";

// -------------------------
// geometry_file_io::extents
// -------------------------
// Purpose:
//   Returns the smallest bounding box that includes all vertices in the file.
//   This default implementation reads the entire file and computes it.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
bounding_box geometry_file_io::extents(app &ctx, const std::string &filename) {
  return read(ctx, filename).extents();
}

// -------------
// read_geometry
// -------------
// Purpose:
//   The main read geometry function.  Refers to ctx's geometry handlers to choose
//   an appropriate IO object for reading the requested geometry file.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- The handlers come from ctx's own table (app::geometry_handlers()).
geometry read_geometry(app &ctx, const std::string &filename) {
  using namespace std;
  using namespace boost;
  string errors;
  smatch what;
  const regex file_extension(geometry_file_io::FILE_EXTENSION_EXPR);
  if (regex_match(filename, what, file_extension)) {
    const geometry_file_io::handlers h = ctx.geometry_handlers().for_extension(what[2].str());
    if (h.empty())
      throw unsupported_geometry_file_type(string(BOOST_CURRENT_FUNCTION) +
                                           string(": Cannot read ") + filename);
    // use the first handler that succeds
    for (geometry_file_io::handlers::const_iterator i = h.begin(); i != h.end(); i++)
      try {
        if (*i)
          return (*i)->read(ctx, filename);
      } catch (exception &e) {
        errors += string(" :: ") + e.what();
      }
  }
  throw unsupported_geometry_file_type(str(boost::format("%1% : Cannot read '%2%'%3%") %
                                           BOOST_CURRENT_FUNCTION % filename % errors));
}

// --------------
// write_geometry
// --------------
// Purpose:
//   The main write geometry function.  Refers to ctx's geometry handlers to choose
//   an appropriate IO object for writing the requested geometry file.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- The handlers come from ctx's own table (app::geometry_handlers()).
void write_geometry(app &ctx, const geometry &geo, const std::string &filename) {
  using namespace std;
  using namespace boost;
  string errors;
  smatch what;
  const regex file_extension(geometry_file_io::FILE_EXTENSION_EXPR);
  if (regex_match(filename, what, file_extension)) {
    const geometry_file_io::handlers h = ctx.geometry_handlers().for_extension(what[2].str());
    if (h.empty())
      throw unsupported_geometry_file_type(string(BOOST_CURRENT_FUNCTION) +
                                           string(": Cannot write ") + filename);
    // use the first handler that succeds
    for (geometry_file_io::handlers::const_iterator i = h.begin(); i != h.end(); i++)
      try {
        if (*i)
          return (*i)->write(ctx, geo, filename);
      } catch (exception &e) {
        errors += string(" :: ") + e.what();
      }
  }
  throw unsupported_geometry_file_type(str(boost::format("%1% : Cannot write '%2%'%3%") %
                                           BOOST_CURRENT_FUNCTION % filename % errors));
}

// -----------------------------------
// register_default_geometry_handlers
// -----------------------------------
// Purpose:
//   The built-in geometry handlers, into ctx's own table (app::registerDefaultHandlers()).
// ---- Change History ----
// 11/09/2026 -- Joe R. -- Takes the app. It used to fill a process-wide map on first use, and
//                         app's constructor added a second set of the same handlers to it.
void register_default_geometry_handlers(app &ctx) {
  register_bunny_io(ctx);
  register_off_io(ctx);
  register_cvcraw_io(ctx);
  register_assimp_geometry_io(ctx);
}
} // namespace cvc
