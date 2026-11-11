/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#ifndef __CVC_MODEL_FILE_IO__
#define __CVC_MODEL_FILE_IO__

#include <boost/shared_ptr.hpp>
#include <cvc/core/exception.h>
#include <cvc/core/file_handler_registry.h>
#include <cvc/model/model.h>
#include <cvc/volume/bounding_box.h>
#include <list>
#include <map>
#include <string>
#include <vector>

namespace cvc {
class app;
CVC_DEF_EXCEPTION(unsupported_model_file_type);

// -------------
// model_file_io
// -------------
// Purpose:
//   Provides I/O for model (multi-mesh scene) data. Mirrors geometry_file_io /
//   volume_file_io: handlers declare the extensions they support and are
//   dispatched by extension, first handler that succeeds wins.
// ---- Change History ----
// 2024 -- Joe R. -- Creation (Phase-3 model surface).
struct model_file_io {
  static const char *const FILE_EXTENSION_EXPR;

  // -----------------
  // model_file_io::id
  // -----------------
  // Purpose:
  //   Returns a string that identifies this model_file_io object.  This should
  //   be unique, but is freeform.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  virtual const std::string &id() const = 0;

  typedef std::list<std::string> extension_list;

  // -------------------------
  // model_file_io::extensions
  // -------------------------
  // Purpose:
  //   Returns a list of extensions that this model_file_io object supports.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  virtual const extension_list &extensions() const = 0;

  // -------------------
  // model_file_io::read
  // -------------------
  // Purpose:
  //   Reads a file and outputs a model object.  ctx is the app reading it (read_model's): its
  //   image handlers read the textures.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  // 11/09/2026 -- Joe R. -- Takes the app.
  virtual model read(app &ctx, const std::string &filename) const = 0;

  // --------------------
  // model_file_io::write
  // --------------------
  // Purpose:
  //   Writes a model to file.  Export is optional; a handler that cannot write
  //   throws unsupported_model_file_type.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  // 11/09/2026 -- Joe R. -- Takes the app.
  virtual void write(app &ctx, const model &m, const std::string &filename) const = 0;

  // ----------------------
  // model_file_io::extents
  // ----------------------
  // Purpose:
  //   Returns the smallest bounding box that includes all vertices in the file.
  //   Default implementation reads the entire file and computes it.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  virtual bounding_box extents(app &ctx, const std::string &filename);

  virtual ~model_file_io() {}

  typedef boost::shared_ptr<model_file_io> ptr;
  typedef std::vector<ptr> handlers;
  typedef std::map<std::string, /* file extension */
                   handlers>
      handler_map;

  // The handlers live in each app's own table, app::model_handlers() (file_handler_registry.h):
  // read_model and write_model look the format up in the app they are given.
};

// ----------
// read_model
// ----------
// Purpose:
//   The main read model function.  Picks the reader for the file's extension from ctx's own
//   handlers (app::model_handlers()) and returns the first one's result that succeeds.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- Takes the app whose handlers it uses; there was one process-wide map.
model read_model(app &ctx, const std::string &filename);

// ----------------------
// read_model_from_memory
// ----------------------
// Purpose:
//   A model from its encoded bytes in memory (a .glb out of an unpacked archive, a download), no
//   file: the format by `extension` (".glb" or "glb"). Only a self-contained file reads this way
//   (a .glb, a .ply, an .stl, an .obj with no .mtl): a reference to another file (a .gltf's .bin,
//   an external texture) has nowhere to resolve. Throws unsupported_model_file_type when no
//   backend reads the format from memory (Assimp off, or an extension it does not know), and
//   std::runtime_error when the bytes do not read. can_read_model_from_memory says which. It
//   reads with the compiled-in backend directly, and its embedded textures with ctx's image
//   handlers.
// ---- Change History ----
// 2026 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- Takes the app whose image handlers read the textures.
model read_model_from_memory(app &ctx, const void *data, std::size_t size,
                             const std::string &extension);
bool can_read_model_from_memory(const std::string &extension);

// -----------
// write_model
// -----------
// Purpose:
//   The main write model function.  Picks the writer for the file's extension from ctx's own
//   handlers (app::model_handlers()).
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- Takes the app whose handlers it uses.
void write_model(app &ctx, const model &m, const std::string &filename);

// ----------------------------------
// register_default_model_handlers
// ----------------------------------
// Purpose:
//   The built-in model handlers, into ctx's own table (app::registerDefaultHandlers()).
//   Registers nothing when no import backend is compiled in (e.g. CVC_ENABLE_ASSIMP off).
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
// 11/09/2026 -- Joe R. -- Takes the app; it used to fill a process-wide map on first use.
void register_default_model_handlers(app &ctx);
} // namespace cvc

#endif
