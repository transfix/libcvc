/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Private: load_city_bundle's version 2 reader (world_bundle.cpp), called when a bundle has a
// world.json.

#ifndef CVC_WORLD_CITY_BUNDLE_V2_H
#define CVC_WORLD_CITY_BUNDLE_V2_H

#include <cvc/world/city_bundle.h>
#include <string>

namespace cvc {
namespace world {
namespace detail {

city_bundle load_city_bundle_v2(app &ctx, const city_bundle_reader &read, const std::string &label,
                                const std::string &dir, const std::string &manifest_text);

} // namespace detail
} // namespace world
} // namespace cvc

#endif // CVC_WORLD_CITY_BUNDLE_V2_H
