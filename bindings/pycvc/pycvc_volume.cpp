// pycvc_volume.cpp — Volume facade implementation.
#include "pycvc_volume.h"

#include "pycvc_context.h"

#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/volume/dimension.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <stdexcept>

namespace pycvc {

// The Volume facade binds to the module-wide app context (pycvc::ctx(), in
// pycvc_context.{h,cpp}) rather than a private singleton, so a host-injected
// app and Python share one context / state tree. `ctx()` below resolves to
// pycvc::ctx() by ordinary namespace lookup.

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
  // Pin the EXACT block this view aliases, not the whole cvc::volume. A later
  // enable_cuda()/disable_cuda() migrates by reallocation (host <-> CUDA
  // unified) and frees the old block; owning the volume would keep the object
  // alive but NOT that specific buffer, so the numpy view would dangle and
  // segfault. Owning the block makes a stale view fail SAFE — valid (if
  // decoupled) memory rather than a use-after-free. The shape is already
  // captured by value above, so the buffer alone is a sufficient owner.
  v.owner = vol_->active_storage();
  return v;
}

// cvc::voxels::cuda_data_ptr() only exists when libcvc was built with CUDA
// (CVC_USING_CUDA is propagated via cvc::cvc's interface compile defs). On
// host-only builds there is no GPU residency, so both report "host".
bool Volume::on_gpu() const {
#ifdef CVC_USING_CUDA
  return vol_->cuda_data_ptr() != nullptr;
#else
  return false;
#endif
}

unsigned long long Volume::cuda_ptr() const {
#ifdef CVC_USING_CUDA
  return reinterpret_cast<unsigned long long>(vol_->cuda_data_ptr());
#else
  return 0;
#endif
}

bool Volume::cuda_available() {
#ifdef CVC_USING_CUDA
  return cvc::voxels::cuda_available();
#else
  return false;
#endif
}

void Volume::enable_cuda(int device) {
#ifdef CVC_USING_CUDA
  vol_->enableCUDA(device);
#else
  (void)device;
  throw std::runtime_error("enable_cuda: this libcvc build has CUDA disabled");
#endif
}

void Volume::disable_cuda() {
#ifdef CVC_USING_CUDA
  vol_->disableCUDA();
#endif
}

bool Volume::using_cuda() const {
#ifdef CVC_USING_CUDA
  return vol_->using_cuda();
#else
  return false;
#endif
}

void Volume::load(const std::string &filename) { vol_->read(filename); }
void Volume::save(const std::string &filename) const {
  // Delegate to cvc::volume::write(), which first createVolumeFile()s the
  // (possibly non-existent) target and then fills it in. The free-function
  // writeVolumeFile(app, vol, filename) overload used previously writes only
  // into an *already existing* file, so for a fresh path the format handler
  // threw while trying to read the missing file — surfacing as
  // unsupported_volume_file_type even though the handler was registered.
  vol_->write(filename);
}

cvc::volume &Volume::native() { return *vol_; }
const cvc::volume &Volume::native() const { return *vol_; }

} // namespace pycvc
