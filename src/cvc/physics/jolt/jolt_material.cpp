/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Built with -fno-rtti on GCC/Clang (see jolt_material.h): no throw, no catch,
// nothing that can throw.

#include "jolt_material.h"

namespace cvc {
namespace physics {
namespace detail {

cvc_material::cvc_material(material_id id, float friction, float restitution)
    : id(id), friction(friction), restitution(restitution) {}

cvc_material::~cvc_material() = default;

} // namespace detail
} // namespace physics
} // namespace cvc
