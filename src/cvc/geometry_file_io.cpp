#include <boost/foreach.hpp>
#include <boost/regex.hpp>
#include <cvc/geometry_file_io.h>
#include <cvc/io_handlers.h>

namespace CVC_NAMESPACE {
// A regex to extract a filename extension
const char *geometry_file_io::FILE_EXTENSION_EXPR = "^(.*)(\\.\\S*)$";

// -------------------------
// geometry_file_io::extents
// -------------------------
// Purpose:
//   Returns the smallest bounding box that includes all vertices in the file.
//   This default implementation reads the entire file and computes it.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
bounding_box geometry_file_io::extents(const std::string &filename) {
  return read(filename).extents();
}

// ------------------------------
// geometry_file_io::get_handlers
// ------------------------------
// Purpose:
//   Static initialization of handler map.  Clients use the handler_map
//   to add themselves to the collection of objects that are to be used
//   to perform geometry file i/o operations.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
geometry_file_io::handler_map &geometry_file_io::get_handlers() {
  // It's ok to leak: http://www.parashift.com/c++-faq-lite/ctors.html#faq-10.15
  static handler_map *p = initialize_map();

  // Ensure the default geometry I/O handlers are registered the first
  // time the handler map is requested. The handler register_* helpers
  // call back into insert_handler() and therefore re-enter
  // get_handlers(); an explicit guard avoids re-entering the same
  // function-local static initialiser, which would be undefined
  // behaviour.
  static bool _registering_defaults = false;
  static bool _defaults_registered = false;
  if (!_defaults_registered && !_registering_defaults) {
    _registering_defaults = true;
    register_default_geometry_handlers();
    _defaults_registered = true;
    _registering_defaults = false;
  }

  return *p;
}

// -------------------------------
// geometry_file_io::insertHandler
// -------------------------------
// Purpose:
//   Convenence function for adding objects to the map.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
void geometry_file_io::insert_handler(const ptr &gfio) { insert_handler(get_handlers(), gfio); }

// -------------------------------
// geometry_file_io::removeHandler
// -------------------------------
// Purpose:
//   Convenence function for removing objects from the map.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
void geometry_file_io::remove_handler(const ptr &gfio) {
  for (handler_map::iterator i = get_handlers().begin(); i != get_handlers().end(); i++) {
    handlers h;
    for (handlers::iterator j = i->second.begin(); j != i->second.end(); j++) {
      if (*j != gfio)
        h.push_back(*j);
    }
    i->second = h;
  }
}

// --------------------------------
// geometry_file_io::remove_handler
// --------------------------------
// Purpose:
//   Convenence function for removing objects from the map.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
void geometry_file_io::remove_handler(const std::string &id) {
  for (handler_map::iterator i = get_handlers().begin(); i != get_handlers().end(); i++) {
    handlers h;
    for (handlers::iterator j = i->second.begin(); j != i->second.end(); j++) {
      if ((*j)->id() != id)
        h.push_back(*j);
    }
    i->second = h;
  }
}

// --------------------------------
// geometry_file_io::get_extensions
// --------------------------------
// Purpose:
//   Returns the list of supported file extensions.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
std::vector<std::string> geometry_file_io::get_extensions() {
  std::vector<std::string> ret;
  BOOST_FOREACH (handler_map::value_type &i, get_handlers()) {
    ret.push_back(i.first);
  }
  return ret;
}

// --------------------------------
// geometry_file_io::initialize_map
// --------------------------------
// Purpose:
//   Adds the standard geometry_file_io objects to a new handler_map object
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
geometry_file_io::handler_map *geometry_file_io::initialize_map() {
  handler_map *map = new handler_map;
  return map;
}

// --------------------------------
// geometry_file_io::insert_handler
// --------------------------------
// Purpose:
//   Convenence function for adding objects to the specified map.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
void geometry_file_io::insert_handler(handler_map &hm, const ptr &gfio) {
  if (!gfio)
    return;
  for (geometry_file_io::extension_list::const_iterator i = gfio->extensions().begin();
       i != gfio->extensions().end(); i++) {
    hm[*i].push_back(gfio);
  }
}

// -------------
// read_geometry
// -------------
// Purpose:
//   The main read geometry function.  Refers to the handler map to choose
//   an appropriate IO object for reading the requested geometry file.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
geometry read_geometry(const std::string &filename) {
  using namespace std;
  using namespace boost;
  string errors;
  smatch what;
  const regex file_extension(geometry_file_io::FILE_EXTENSION_EXPR);
  if (regex_match(filename, what, file_extension)) {
    if (geometry_file_io::get_handlers()[what[2]].empty())
      throw unsupported_geometry_file_type(string(BOOST_CURRENT_FUNCTION) +
                                           string(": Cannot read ") + filename);
    geometry_file_io::handlers &h = geometry_file_io::get_handlers()[what[2]];
    // use the first handler that succeds
    for (geometry_file_io::handlers::iterator i = h.begin(); i != h.end(); i++)
      try {
        if (*i)
          return (*i)->read(filename);
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
//   The main write geometry function.  Refers to the handler map to choose
//   an appropriate IO object for writing the requested geometry file.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
void write_geometry(const geometry &geo, const std::string &filename) {
  using namespace std;
  using namespace boost;
  string errors;
  smatch what;
  const regex file_extension(geometry_file_io::FILE_EXTENSION_EXPR);
  if (regex_match(filename, what, file_extension)) {
    if (geometry_file_io::get_handlers()[what[2]].empty())
      throw unsupported_geometry_file_type(string(BOOST_CURRENT_FUNCTION) +
                                           string(": Cannot write ") + filename);
    geometry_file_io::handlers &h = geometry_file_io::get_handlers()[what[2]];
    // use the first handler that succeds
    for (geometry_file_io::handlers::iterator i = h.begin(); i != h.end(); i++)
      try {
        if (*i)
          return (*i)->write(geo, filename);
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
//   Register the built-in geometry I/O handlers without requiring a
//   cvc::app instance. Called from geometry_file_io::get_handlers() the
//   first time the handler map is requested.
void register_default_geometry_handlers() {
  register_bunny_io();
  register_off_io();
  register_cvcraw_io();
}
} // namespace CVC_NAMESPACE
