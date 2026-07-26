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
#include <cvc/model/model.h>
#include <cvc/volume/bounding_box.h>
#include <list>
#include <map>
#include <string>
#include <vector>

namespace cvc {
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
  static const char *FILE_EXTENSION_EXPR;

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
  //   Reads a file and outputs a model object.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  virtual model read(const std::string &filename) const = 0;

  // --------------------
  // model_file_io::write
  // --------------------
  // Purpose:
  //   Writes a model to file.  Export is optional; a handler that cannot write
  //   throws unsupported_model_file_type.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  virtual void write(const model &m, const std::string &filename) const = 0;

  // ----------------------
  // model_file_io::extents
  // ----------------------
  // Purpose:
  //   Returns the smallest bounding box that includes all vertices in the file.
  //   Default implementation reads the entire file and computes it.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  virtual bounding_box extents(const std::string &filename);

  virtual ~model_file_io() {}

  typedef boost::shared_ptr<model_file_io> ptr;
  typedef std::vector<ptr> handlers;
  typedef std::map<std::string, /* file extension */
                   handlers>
      handler_map;

  // -------------------------
  // model_file_io::get_handlers
  // -------------------------
  // Purpose:
  //   Static initialization of handler map.  Clients use the handler_map
  //   to add themselves to the collection of objects that are to be used
  //   to perform model file i/o operations.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  static handler_map &get_handlers();

  // -----------------------------
  // model_file_io::insert_handler
  // -----------------------------
  // Purpose:
  //   Convenience function for adding objects to the map.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  static void insert_handler(const ptr &mfio);

  // -----------------------------
  // model_file_io::remove_handler
  // -----------------------------
  // Purpose:
  //   Convenience function for removing objects from the map.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  static void remove_handler(const ptr &mfio);

  // -----------------------------
  // model_file_io::remove_handler
  // -----------------------------
  // Purpose:
  //   Convenience function for removing objects from the map.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  static void remove_handler(const std::string &name);

  // -----------------------------
  // model_file_io::get_extensions
  // -----------------------------
  // Purpose:
  //   Returns the list of supported file extensions.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  static std::vector<std::string> get_extensions();

private:
  // -----------------------------
  // model_file_io::initialize_map
  // -----------------------------
  // Purpose:
  //   Adds the standard model_file_io objects to a new handler_map object
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  static handler_map *initialize_map();

  // -----------------------------
  // model_file_io::insert_handler
  // -----------------------------
  // Purpose:
  //   Convenience function for adding objects to the specified map.
  // ---- Change History ----
  // 2024 -- Joe R. -- Creation.
  static void insert_handler(handler_map &hm, const ptr &mfio);
};

// ----------
// read_model
// ----------
// Purpose:
//   The main read model function.  Refers to the handler map to choose
//   an appropriate IO object for reading the requested model file.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
model read_model(const std::string &filename);

// -----------
// write_model
// -----------
// Purpose:
//   The main write model function.  Refers to the handler map to choose
//   an appropriate IO object for writing the requested model file.
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
void write_model(const model &m, const std::string &filename);

// ----------------------------------
// register_default_model_handlers
// ----------------------------------
// Purpose:
//   Register the built-in model I/O handlers without requiring a cvc::app
//   instance. Called from model_file_io::get_handlers() the first time the
//   handler map is requested. Registers nothing when no import backend is
//   compiled in (e.g. CVC_ENABLE_ASSIMP off).
// ---- Change History ----
// 2024 -- Joe R. -- Creation.
void register_default_model_handlers();
} // namespace cvc

#endif
