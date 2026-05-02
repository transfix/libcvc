#include <cvc/app.h>
#include <cvc/bunny.h>
#include <cvc/exception.h>
#include <cvc/geometry_file_io.h>
#include <cvc/utility.h>
#include <fstream>

namespace CVC_NAMESPACE {
CVC_DEF_EXCEPTION(invalid_bunny_operation);

// --------
// bunny_io
// --------
// Purpose:
//   Reading from bunny_io will always return a Stanford Bunny tri mesh.
//   Used for debugging and demonstration purposes.
// ---- Change History ----
// 01/12/2014 -- Joe R. -- Creation.
struct bunny_io : public geometry_file_io {
  // ------------------
  // bunny_io::bunny_io
  // ------------------
  // Purpose:
  //   Initializes the extension list and id.
  // ---- Change History ----
  // 01/12/2014 -- Joe R. -- Creation.
  bunny_io() : _id("bunny_io : v1.0") {
    _extensions.push_back(".bunny");
    _extensions.push_back(".BUNNY");
  }

  // ------------
  // bunny_io::id
  // ------------
  // Purpose:
  //   Returns a string that identifies this geometry_file_io object.  This should
  //   be unique, but is freeform.
  // ---- Change History ----
  // 01/12/2014 -- Joe R. -- Creation.
  virtual const std::string &id() const { return _id; }

  // --------------------
  // bunny_io::extensions
  // --------------------
  // Purpose:
  //   Returns a list of extensions that this geometry_file_io object supports.
  // ---- Change History ----
  // 01/12/2014 -- Joe R. -- Creation.
  virtual const extension_list &extensions() const { return _extensions; }

  // --------------
  // bunny_io::read
  // --------------
  // Purpose:
  //   Always returns the bunny no matter the filename.
  // ---- Change History ----
  // 06/01/2012 -- Joe R. -- Creation.
  // 01/12/2014 -- Joe R. -- Adapted for geometry_file_io.
  virtual geometry read(const std::string &) const {
    geometry geom;
    for (size_t i = 0; i < 34835; i++) {
      point_t point;
      vector_t normal;
      for (int j = 0; j < 3; j++)
        point[j] = BUNNY_VERTS[i * 3 + j];
      for (int j = 0; j < 3; j++)
        normal[j] = BUNNY_NORMS[i * 3 + j];
      geom.points().push_back(point);
      geom.normals().push_back(normal);
    }

    for (size_t i = 0; i < 69473; i++) {
      tri_t tri;
      for (int j = 0; j < 3; j++)
        tri[j] = BUNNY_TRIS[i * 3 + j];
      geom.tris().push_back(tri);
    }
    return geom;
  }

  // -------------
  // bunny_io::write
  // -------------
  // Purpose:
  //   Supposed to write geometry to OFF file format, but is unimplemented so throws an error for
  //   now.
  // ---- Change History ----
  // 01/11/2014 -- Joe R. -- Creation.
  virtual void write(const geometry &geom, const std::string &filename) const {
    throw write_error(BOOST_CURRENT_FUNCTION);
  }

protected:
  std::string _id;
  extension_list _extensions;
};
} // namespace CVC_NAMESPACE

namespace CVC_NAMESPACE {
void register_bunny_io(app & /*ctx*/) {
  geometry_file_io::insert_handler(geometry_file_io::ptr(new bunny_io));
}

void register_bunny_io() { geometry_file_io::insert_handler(geometry_file_io::ptr(new bunny_io)); }
} // namespace CVC_NAMESPACE
