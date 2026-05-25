#ifndef __CVC_GEOMETRY_FILE_IO__
#define __CVC_GEOMETRY_FILE_IO__

#include <boost/shared_ptr.hpp>
#include <cvc/core/exception.h>
#include <cvc/geometry/geometry.h>
#include <list>
#include <string>
#include <vector>

namespace cvc {
CVC_DEF_EXCEPTION(unsupported_geometry_file_type);

// ----------------
// geometry_file_io
// ----------------
// Purpose:
//   Provides I/O for geometry data. Inspired by volume_file_io, and works much the same.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
struct geometry_file_io {
  static const char *FILE_EXTENSION_EXPR;

  // --------------------
  // geometry_file_io::id
  // --------------------
  // Purpose:
  //   Returns a string that identifies this volume_file_io object.  This should
  //   be unique, but is freeform.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  virtual const std::string &id() const = 0;

  typedef std::list<std::string> extension_list;

  // --------------------------
  // geometry_file_io::extensions
  // --------------------------
  // Purpose:
  //   Returns a list of extensions that this geometry_file_io object supports.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  virtual const extension_list &extensions() const = 0;

  // ----------------------
  // geometry_file_io::read
  // ----------------------
  // Purpose:
  //   Reads a file and outputs a geometry object.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  virtual geometry read(const std::string &filename) const = 0;

  // -----------------------
  // geometry_file_io::write
  // -----------------------
  // Purpose:
  //   Writes geometry to file.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  virtual void write(const geometry &geom, const std::string &filename) const = 0;

  // -------------------------
  // geometry_file_io::extents
  // -------------------------
  // Purpose:
  //   Returns the smallest bounding box that includes all vertices in the file.
  //   Default implementation reads the entire file and computes it.  Certain file
  //   formats might store this info.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  virtual bounding_box extents(const std::string &filename);

  typedef boost::shared_ptr<geometry_file_io> ptr;
  typedef std::vector<ptr> handlers;
  typedef std::map<std::string, /* file extension */
                   handlers>
      handler_map;

  // -----------------------------
  // geometry_file_io::handlers
  // -----------------------------
  // Purpose:
  //   Static initialization of handler map.  Clients use the handler_map
  //   to add themselves to the collection of objects that are to be used
  //   to perform geometry file i/o operations.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  static handler_map &get_handlers();

  // --------------------------------
  // geometry_file_io::insert_handler
  // --------------------------------
  // Purpose:
  //   Convenience function for adding objects to the map.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  static void insert_handler(const ptr &gfio);

  // --------------------------------
  // geometry_file_io::remove_handler
  // --------------------------------
  // Purpose:
  //   Convenience function for removing objects from the map.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  static void remove_handler(const ptr &gfio);

  // --------------------------------
  // geometry_file_io::remove_handler
  // --------------------------------
  // Purpose:
  //   Convenience function for removing objects from the map.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  static void remove_handler(const std::string &name);

  // --------------------------------
  // geometry_file_io::get_extensions
  // --------------------------------
  // Purpose:
  //   Returns the list of supported file extensions.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  static std::vector<std::string> get_extensions();

private:
  // --------------------------------
  // geometry_file_io::initialize_map
  // --------------------------------
  // Purpose:
  //   Adds the standard geometry_file_io objects to a new handler_map object
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  static handler_map *initialize_map();

  // --------------------------------
  // geometry_file_io::insert_handler
  // --------------------------------
  // Purpose:
  //   Convenience function for adding objects to the specified map.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  static void insert_handler(handler_map &hm, const ptr &gfio);
};

// -------------
// read_geometry
// -------------
// Purpose:
//   The main read geometry function.  Refers to the handler map to choose
//   an appropriate IO object for reading the requested geometry file.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
geometry read_geometry(const std::string &filename);

// --------------
// write_geometry
// --------------
// Purpose:
//   The main write geometry function.  Refers to the handler map to choose
//   an appropriate IO object for writing the requested geometry file.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
void write_geometry(const geometry &geo, const std::string &filename);
} // namespace cvc

#endif
