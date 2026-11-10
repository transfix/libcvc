#ifndef __CVC_GEOMETRY_FILE_IO__
#define __CVC_GEOMETRY_FILE_IO__

#include <boost/shared_ptr.hpp>
#include <cvc/core/exception.h>
#include <cvc/core/file_handler_registry.h>
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
  static const char *const FILE_EXTENSION_EXPR;

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
  //   Reads a file and outputs a geometry object.  ctx is the app reading it (read_geometry's).
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  // 11/09/2026 -- Joe R. -- Takes the app, as volume_file_io's readers do.
  virtual geometry read(app &ctx, const std::string &filename) const = 0;

  // -----------------------
  // geometry_file_io::write
  // -----------------------
  // Purpose:
  //   Writes geometry to file.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  // 11/09/2026 -- Joe R. -- Takes the app.
  virtual void write(app &ctx, const geometry &geom, const std::string &filename) const = 0;

  // -------------------------
  // geometry_file_io::extents
  // -------------------------
  // Purpose:
  //   Returns the smallest bounding box that includes all vertices in the file.
  //   Default implementation reads the entire file and computes it.  Certain file
  //   formats might store this info.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  virtual bounding_box extents(app &ctx, const std::string &filename);

  typedef boost::shared_ptr<geometry_file_io> ptr;
  typedef std::vector<ptr> handlers;
  typedef std::map<std::string, /* file extension */
                   handlers>
      handler_map;

  // The handlers live in each app's own table, app::geometry_handlers() (file_handler_registry.h):
  // read_geometry and write_geometry look the format up in the app they are given.
};

// -------------
// read_geometry
// -------------
// Purpose:
//   The main read geometry function.  Picks the reader for the file's extension from ctx's own
//   handlers (app::geometry_handlers()) and returns the first one's result that succeeds.
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- Takes the app whose handlers it uses; there was one process-wide map.
geometry read_geometry(app &ctx, const std::string &filename);

// --------------
// write_geometry
// --------------
// Purpose:
//   The main write geometry function.  Picks the writer for the file's extension from ctx's own
//   handlers (app::geometry_handlers()).
// ---- Change History ----
// 01/11/2014 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- Takes the app whose handlers it uses.
void write_geometry(app &ctx, const geometry &geo, const std::string &filename);
} // namespace cvc

#endif
