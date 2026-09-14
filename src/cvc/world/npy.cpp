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

#include <cstdio>
#include <cstring>
#include <cvc/world/npy.h>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace world {

namespace {

std::vector<std::uint8_t> build(const char *descr, int rows, int cols, const void *data,
                                std::size_t nbytes) {
  std::ostringstream hs;
  hs << "{'descr': '" << descr << "', 'fortran_order': False, 'shape': (" << rows << ", " << cols
     << "), }";
  std::string header = hs.str();
  // Pad so (10 + header.size()) is a multiple of 64; header ends with '\n'.
  const std::size_t base = 10 + header.size() + 1;
  const std::size_t pad = (64 - (base % 64)) % 64;
  header.append(pad, ' ');
  header.push_back('\n');

  std::vector<std::uint8_t> buf;
  buf.reserve(10 + header.size() + nbytes);
  const unsigned char magic[] = {0x93, 'N', 'U', 'M', 'P', 'Y', 0x01, 0x00};
  buf.insert(buf.end(), magic, magic + 8);
  const std::uint16_t hlen = static_cast<std::uint16_t>(header.size()); // < 65536
  buf.push_back(std::uint8_t(hlen & 0xFF));
  buf.push_back(std::uint8_t((hlen >> 8) & 0xFF));
  buf.insert(buf.end(), header.begin(), header.end());
  const std::uint8_t *p = static_cast<const std::uint8_t *>(data);
  buf.insert(buf.end(), p, p + nbytes);
  return buf;
}

void write_file(const std::string &path, const std::vector<std::uint8_t> &buf) {
  std::ofstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot write .npy: " + path);
  f.write(reinterpret_cast<const char *>(buf.data()), std::streamsize(buf.size()));
}

} // namespace

void write_npy(const std::string &path, const std::vector<std::uint8_t> &data, int rows, int cols) {
  write_file(path, build("|u1", rows, cols, data.data(), data.size()));
}
void write_npy(const std::string &path, const std::vector<std::uint16_t> &data, int rows,
               int cols) {
  write_file(path, build("<u2", rows, cols, data.data(), data.size() * 2));
}
void write_npy(const std::string &path, const std::vector<float> &data, int rows, int cols) {
  write_file(path, build("<f4", rows, cols, data.data(), data.size() * 4));
}

std::vector<std::uint8_t> npy_bytes(const std::vector<std::uint16_t> &data, int rows, int cols) {
  return build("<u2", rows, cols, data.data(), data.size() * 2);
}

} // namespace world
} // namespace cvc
