/*
  Copyright 2026 The University of Texas at Austin

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

// cvc::compute_contour_spectrum_libcontour: the parity oracle for cvc::compute_contour_spectrum.
// It runs the signature functions of the libContour copy vendored beside this file (Bajaj,
// Pascucci & Schikore, "The Contour Spectrum", IEEE Visualization 1997; libContour by Dan Schikore
// and Emilio Camahort, see THIRD_PARTY_NOTICES.md) as VolumeRover2's ColorTable2 did, but on a
// Datareg3 built directly over the voxels: newDatasetReg would also build a Conplotreg3 (cell
// index trees, seed cells) that the spectrum never uses, and leak it. libContour keeps no
// process-wide state, so concurrent calls need no lock. Compiled only in a CVC_ENABLE_MESHER
// build, inside libcvc.

#include <climits>
#include <cstdlib>
#include <cvc/volume/contour_spectrum.h>
#include <cvc/volume/volume.h>
#include <datareg3.h>
#include <memory>
#include <vector>

namespace cvc {
namespace {

// compFunction returns malloc'd arrays (the sample values and the function) that libContour
// itself never frees.
struct free_deleter {
  void operator()(float *p) const { std::free(p); }
};
using malloc_floats = std::unique_ptr<float, free_deleter>;

} // namespace

bool contour_spectrum_libcontour_available() { return true; }

contour_spectrum compute_contour_spectrum_libcontour(const volume &vol, bool world_units) {
  const uint64 nx = vol.XDim(), ny = vol.YDim(), nz = vol.ZDim();
  if (nx < 2 || ny < 2 || nz < 2)
    throw contour_spectrum_error(
        "compute_contour_spectrum_libcontour: every axis needs at least 2 voxels");
  if (nx * ny * nz > static_cast<uint64>(INT_MAX))
    throw contour_spectrum_error(
        "compute_contour_spectrum_libcontour: libContour indexes voxels with an int");

  // Datareg3 reads uchar, ushort and float; anything else goes through a float copy.
  ::Data::DataType type = ::Data::FLOAT;
  const unsigned char *raw = vol.data_ptr();
  std::vector<float> converted;
  switch (vol.voxelType()) {
  case UChar:
    type = ::Data::UCHAR;
    break;
  case UShort:
    type = ::Data::USHORT;
    break;
  case Float:
    type = ::Data::FLOAT;
    break;
  case Undefined:
    throw contour_spectrum_error("compute_contour_spectrum_libcontour: undefined voxel type");
  default: {
    const uint64 count = nx * ny * nz;
    converted.resize(static_cast<std::size_t>(count));
    for (uint64 i = 0; i < count; ++i)
      converted[static_cast<std::size_t>(i)] = static_cast<float>(vol(i));
    raw = reinterpret_cast<const unsigned char *>(converted.data());
    break;
  }
  }

  contour_spectrum cs;
  const double sx = world_units ? vol.XSpan() : 1.0;
  const double sy = world_units ? vol.YSpan() : 1.0;
  const double sz = world_units ? vol.ZSpan() : 1.0;
  cs.total_volume =
      static_cast<double>(nx - 1) * static_cast<double>(ny - 1) * static_cast<double>(nz - 1) *
      sx * sy * sz;

  int dim[3] = {static_cast<int>(nx), static_cast<int>(ny), static_cast<int>(nz)};
  // Datareg3 borrows the voxels (it only reads them) and copies dim. Its destructor is empty, so
  // the small pointer table and min/max arrays its constructor mallocs leak; freeing them here
  // would double-free once the vendored destructors are fixed.
  ::Datareg3 reg(type, 1, dim, const_cast<u_char *>(raw));
  if (world_units) {
    float orig[3] = {static_cast<float>(vol.XMin()), static_cast<float>(vol.YMin()),
                     static_cast<float>(vol.ZMin())};
    float span[3] = {static_cast<float>(sx), static_cast<float>(sy), static_cast<float>(sz)};
    reg.setOrig(orig);
    reg.setSpan(span);
  }
  cs.domain_min = reg.getMin();
  cs.domain_max = reg.getMax();

  if (!(cs.domain_max > cs.domain_min)) {
    // Every tetrahedron's start bucket divides by (max - min): a constant volume would index with
    // NaN. Answer as compute_contour_spectrum does for a degenerate domain.
    constexpr std::size_t samples = 256; // Datareg3's FSAMPLES
    cs.isovalue.assign(samples, cs.domain_min);
    cs.surface_area.assign(samples, 0.0);
    cs.min_volume.assign(samples, 0.0);
    cs.max_volume.assign(samples, 0.0);
    cs.gradient.assign(samples, 0.0);
    return cs;
  }

  // Datareg3::fName order: "Surface Area", "Min Volume", "Max Volume", "Gradient".
  std::vector<double> *const out[4] = {&cs.surface_area, &cs.min_volume, &cs.max_volume,
                                       &cs.gradient};
  for (int f = 0; f < 4; ++f) {
    u_int len = 0;
    float *fx = nullptr;
    malloc_floats fy(reg.compFunction(f, len, &fx));
    const malloc_floats fx_owner(fx);
    if (!fy || !fx || len == 0)
      throw contour_spectrum_error("compute_contour_spectrum_libcontour: libContour returned no "
                                   "samples");
    out[f]->assign(fy.get(), fy.get() + len);
    if (f == 0)
      cs.isovalue.assign(fx, fx + len);
  }
  return cs;
}

} // namespace cvc
