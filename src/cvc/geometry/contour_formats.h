/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The built-in contour_stack formats (cvc/geometry/contours.h), private to libcvc: contours.cpp
// registers them in each app's table.

#ifndef CVC_GEOMETRY_CONTOUR_FORMATS_H
#define CVC_GEOMETRY_CONTOUR_FORMATS_H

#include <cvc/core/file_handler_registry.h>
#include <cvc/geometry/contours.h>
#include <string>

namespace cvc {
namespace contour_formats {

// The file name's last extension, lower-cased, with its dot (".ser"); "" when it has none.
std::string extension_of(const std::string &filename);

// Add the built-in handlers (.ser, .config, .contour, .pcd, .cvcc) to `r`.
void register_builtin(file_handler_registry<contour_file_io> &r);

} // namespace contour_formats
} // namespace cvc

#endif // CVC_GEOMETRY_CONTOUR_FORMATS_H
