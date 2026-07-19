// pycvc_volume.cpp — Volume facade implementation.
#include "pycvc_volume.h"

#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/volume/dimension.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <stdexcept>

namespace pycvc {

namespace {
// Process-wide app context (thread pool, state root). Mirrors the CLI's
// `static cvc::app app;` pattern — the public accessor instancePtr() is
// protected, so hosts hold their own instance.
cvc::app &ctx() {
  static cvc::app app;
  return app;
}
} // namespace

Volume::Volume() : vol_(std::make_shared<cvc::volume>(ctx())) {}
Volume::~Volume() = default;

void Volume::set_float_grid(const std::vector<double> &values, unsigned long nx, unsigned long ny,
                            unsigned long nz, double minx, double miny, double minz, double maxx,
                            double maxy, double maxz) {
  const std::size_t n =
      static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz);
  if (nx == 0 || ny == 0 || nz == 0)
    throw std::invalid_argument("set_float_grid: dimensions must be positive");
  if (values.size() != n)
    throw std::invalid_argument("set_float_grid: len(values) must equal nx*ny*nz");

  std::vector<float> fbuf(values.begin(), values.end());
  cvc::dimension dim(nx, ny, nz);
  cvc::bounding_box box(minx, miny, minz, maxx, maxy, maxz);
  *vol_ = cvc::volume(ctx(), reinterpret_cast<const unsigned char *>(fbuf.data()), dim, cvc::Float,
                      box);
}

unsigned long Volume::xdim() const { return vol_->XDim(); }
unsigned long Volume::ydim() const { return vol_->YDim(); }
unsigned long Volume::zdim() const { return vol_->ZDim(); }

double Volume::value(unsigned long i, unsigned long j, unsigned long k) const {
  return (*vol_)(i, j, k);
}

double Volume::min_value() const {
  double m = 0.0;
  bool first = true;
  for (unsigned long k = 0; k < vol_->ZDim(); ++k)
    for (unsigned long j = 0; j < vol_->YDim(); ++j)
      for (unsigned long i = 0; i < vol_->XDim(); ++i) {
        double v = (*vol_)(i, j, k);
        if (first || v < m) {
          m = v;
          first = false;
        }
      }
  return m;
}

double Volume::max_value() const {
  double m = 0.0;
  bool first = true;
  for (unsigned long k = 0; k < vol_->ZDim(); ++k)
    for (unsigned long j = 0; j < vol_->YDim(); ++j)
      for (unsigned long i = 0; i < vol_->XDim(); ++i) {
        double v = (*vol_)(i, j, k);
        if (first || v > m) {
          m = v;
          first = false;
        }
      }
  return m;
}

double Volume::xmin() const { return vol_->XMin(); }
double Volume::xmax() const { return vol_->XMax(); }
double Volume::ymin() const { return vol_->YMin(); }
double Volume::ymax() const { return vol_->YMax(); }
double Volume::zmin() const { return vol_->ZMin(); }
double Volume::zmax() const { return vol_->ZMax(); }

ArrayView Volume::grid() {
  if (vol_->voxelType() != cvc::Float)
    throw std::invalid_argument("grid(): zero-copy view requires a Float volume");
  ArrayView v;
  v.dtype = DType::Float32;
  v.writable = true;
  // Row-major (nz, ny, nx): index = ((k*ny)+j)*nx + i matches operator()(i,j,k).
  v.shape = {static_cast<long>(vol_->ZDim()), static_cast<long>(vol_->YDim()),
             static_cast<long>(vol_->XDim())};
  v.data = vol_->data_ptr();
  v.owner = vol_; // shared_ptr<cvc::volume> -> shared_ptr<void>
  return v;
}

void Volume::load(const std::string &filename) { cvc::readVolumeFile(ctx(), *vol_, filename); }
void Volume::save(const std::string &filename) const {
  cvc::writeVolumeFile(ctx(), *vol_, filename);
}

cvc::volume &Volume::native() { return *vol_; }
const cvc::volume &Volume::native() const { return *vol_; }

} // namespace pycvc
