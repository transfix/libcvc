// The one piece of legacy VolMagick that legacy GenSeg uses: VolumeFileInfo, read from a rawiv
// file's header, for the world coordinates segment.cpp writes into its border point cloud
// (<prefix>_indices.pcd). The real VolMagick (transfix/volrover src/VolMagick) needs volrover's
// whole CVC core library to build; GenSeg itself never reads a voxel through it (inout.cpp reads
// the rawiv file by hand). The goldens use only GenSeg's labels and its border voxel indices, so
// these coordinates do not reach them. Part of tools/volrover-goldens: never built into libcvc.

#ifndef VOLROVER_GOLDENS_SHIM_VOLMAGICK_H
#define VOLROVER_GOLDENS_SHIM_VOLMAGICK_H

#include <cstdio>
#include <cstring>
#include <string>

namespace VolMagick {

class VolumeFileInfo {
public:
  // A rawiv file's header (big-endian): min[3], max[3] (floats), nverts, ncells (ints), dim[3]
  // (unsigned), origin[3], span[3] (floats). Spans as VolMagick computes them: the extent over
  // dim - 1 steps.
  void read(const std::string &path) {
    std::FILE *f = std::fopen(path.c_str(), "rb");
    if (!f)
      return;
    unsigned char h[68];
    const bool ok = std::fread(h, 1, sizeof h, f) == sizeof h;
    std::fclose(f);
    if (!ok)
      return;
    for (int a = 0; a < 3; ++a) {
      _min[a] = be_float(h + 4 * a);
      _max[a] = be_float(h + 12 + 4 * a);
      _dim[a] = be_u32(h + 32 + 4 * a);
    }
  }
  double XMin() const { return _min[0]; }
  double YMin() const { return _min[1]; }
  double ZMin() const { return _min[2]; }
  double XSpan() const { return span(0); }
  double YSpan() const { return span(1); }
  double ZSpan() const { return span(2); }

private:
  static unsigned be_u32(const unsigned char *p) {
    return (unsigned(p[0]) << 24) | (unsigned(p[1]) << 16) | (unsigned(p[2]) << 8) | unsigned(p[3]);
  }
  static float be_float(const unsigned char *p) {
    const unsigned u = be_u32(p);
    float v;
    std::memcpy(&v, &u, sizeof v);
    return v;
  }
  double span(int a) const { return _dim[a] > 1 ? (_max[a] - _min[a]) / (_dim[a] - 1) : 1.0; }
  double _min[3] = {0, 0, 0}, _max[3] = {0, 0, 0};
  unsigned _dim[3] = {0, 0, 0};
};

} // namespace VolMagick

#endif
