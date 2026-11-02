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
#include <iterator>
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

namespace {

// The value of `key` in the header dict, as its raw text up to the next top-level comma.
std::string header_field(const std::string &h, const std::string &key) {
  const std::size_t k = h.find("'" + key + "'");
  if (k == std::string::npos)
    return std::string();
  std::size_t i = h.find(':', k);
  if (i == std::string::npos)
    return std::string();
  ++i;
  int depth = 0;
  std::size_t j = i;
  for (; j < h.size(); ++j) {
    const char c = h[j];
    if (c == '(')
      ++depth;
    else if (c == ')')
      --depth;
    else if ((c == ',' && depth == 0) || c == '}')
      break;
  }
  std::string v = h.substr(i, j - i);
  const std::size_t a = v.find_first_not_of(" \t'"), b = v.find_last_not_of(" \t'");
  return a == std::string::npos ? std::string() : v.substr(a, b - a + 1);
}

template <class T> float load_as(const std::uint8_t *p) {
  T v;
  std::memcpy(&v, p, sizeof(T));
  return static_cast<float>(v);
}

} // namespace

std::vector<float> read_npy(const std::string &path, int &rows, int &cols) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cannot read .npy: " + path);
  std::vector<std::uint8_t> buf((std::istreambuf_iterator<char>(f)),
                                std::istreambuf_iterator<char>());
  const unsigned char magic[] = {0x93, 'N', 'U', 'M', 'P', 'Y'};
  if (buf.size() < 12 || std::memcmp(buf.data(), magic, 6) != 0)
    throw std::runtime_error("not a .npy file: " + path);
  const int major = buf[6];
  std::size_t hlen = 0, off = 0;
  if (major == 1) {
    hlen = std::size_t(buf[8]) | (std::size_t(buf[9]) << 8);
    off = 10;
  } else if (major == 2 || major == 3) {
    hlen = std::size_t(buf[8]) | (std::size_t(buf[9]) << 8) | (std::size_t(buf[10]) << 16) |
           (std::size_t(buf[11]) << 24);
    off = 12;
  } else {
    throw std::runtime_error(".npy version " + std::to_string(major) + " unsupported: " + path);
  }
  if (off + hlen > buf.size())
    throw std::runtime_error("truncated .npy header: " + path);
  const std::string h(buf.begin() + off, buf.begin() + off + hlen);
  const std::string descr = header_field(h, "descr");
  if (header_field(h, "fortran_order") != "False")
    throw std::runtime_error(".npy must be C order: " + path);
  const std::string shape = header_field(h, "shape");
  long r = 0, c = 0;
  if (std::sscanf(shape.c_str(), "(%ld , %ld", &r, &c) != 2 &&
      std::sscanf(shape.c_str(), "(%ld,%ld", &r, &c) != 2)
    throw std::runtime_error(".npy must be 2-D (shape " + shape + "): " + path);
  if (r < 1 || c < 1)
    throw std::runtime_error(".npy is empty: " + path);
  if (descr.size() != 3 || (descr[0] != '<' && descr[0] != '|'))
    throw std::runtime_error(".npy dtype '" + descr +
                             "' unsupported (little-endian only): " + path);
  const std::string t = descr.substr(1);
  float (*conv)(const std::uint8_t *) = nullptr;
  std::size_t width = 0;
  if (t == "f4")
    conv = load_as<float>, width = 4;
  else if (t == "f8")
    conv = load_as<double>, width = 8;
  else if (t == "i1")
    conv = load_as<std::int8_t>, width = 1;
  else if (t == "u1")
    conv = load_as<std::uint8_t>, width = 1;
  else if (t == "i2")
    conv = load_as<std::int16_t>, width = 2;
  else if (t == "u2")
    conv = load_as<std::uint16_t>, width = 2;
  else if (t == "i4")
    conv = load_as<std::int32_t>, width = 4;
  else if (t == "u4")
    conv = load_as<std::uint32_t>, width = 4;
  else
    throw std::runtime_error(".npy dtype '" + descr + "' unsupported: " + path);
  const std::size_t n = std::size_t(r) * std::size_t(c), body = off + hlen;
  if (buf.size() < body + n * width)
    throw std::runtime_error("truncated .npy body: " + path);
  std::vector<float> out(n);
  for (std::size_t i = 0; i < n; ++i)
    out[i] = conv(buf.data() + body + i * width);
  rows = int(r);
  cols = int(c);
  return out;
}

} // namespace world
} // namespace cvc
