/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// npy.h — a minimal NumPy .npy v1.0 writer (little-endian, C order). Just enough
// for the world bundle: uint8, uint16 and float32 2-D arrays that np.load reads
// natively. No .npy reader in libcvc, so this is the whole dependency.

#ifndef CVC_WORLD_NPY_H
#define CVC_WORLD_NPY_H

#include <cstdint>
#include <string>
#include <vector>

namespace cvc {
namespace world {

void write_npy(const std::string &path, const std::vector<std::uint8_t> &data, int rows, int cols);
void write_npy(const std::string &path, const std::vector<std::uint16_t> &data, int rows, int cols);
void write_npy(const std::string &path, const std::vector<float> &data, int rows, int cols);

// Serialize to a byte buffer (for tests / in-memory checks).
std::vector<std::uint8_t> npy_bytes(const std::vector<std::uint16_t> &data, int rows, int cols);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_NPY_H
