// Runs the legacy SegMed (transfix/volrover src/Segmentation/SegMed, at the sha in ../volrover.sha)
// as its XML-RPC entry medicalSegmentation (medical_segmentation.cpp) did, up to its critical
// points, from the command line:
//
//   segmed_legacy <input.rawiv> <tlow> <critical.txt> <field.f32>
//
// It reads the rawiv file with the legacy read_data (normalising it to 0..255) and then:
//   1. runs GVF_Compute's first steps and gvfflow() as GVF_Compute runs them (GetGradient, InitGVF,
//      gvfflow), and writes the diffused field it leaves in the legacy global `velocity`:
//      <field.f32> holds x, y, z float32 little-endian per voxel, x fastest;
//   2. runs GVF_Compute whole and writes its critical list, "x y z" a line, in its order.
// GVF_Compute returns right after InitGVF at the pinned sha (GVF.cpp:97), so nothing after that
// line ever ran; CMakeLists.txt compiles a copy without that return (the one patch), and this file
// includes that copy, so its globals are visible here. tlow is a float, as medicalSegmentation
// read it. Part of tools/volrover-goldens: never built into libcvc.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// The patched legacy GVF.cpp (CMakeLists.txt writes it into the build tree). It #defines max and
// min, so nothing from the standard library is included after it.
#include "GVF.cpp"

namespace SegMed {
void read_data(int *, int *, int *, float **, const char *); // inout.cpp
}

static bool write_le_floats(std::FILE *f, const float *v, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    unsigned char b[4];
    std::uint32_t u;
    std::memcpy(&u, &v[i], 4);
    b[0] = (unsigned char)(u & 0xff);
    b[1] = (unsigned char)((u >> 8) & 0xff);
    b[2] = (unsigned char)((u >> 16) & 0xff);
    b[3] = (unsigned char)((u >> 24) & 0xff);
    if (std::fwrite(b, 1, 4, f) != 4)
      return false;
  }
  return true;
}

int main(int argc, char **argv) {
  if (argc != 5) {
    std::fprintf(stderr, "usage: segmed_legacy <input.rawiv> <tlow> <critical.txt> <field.f32>\n");
    return 2;
  }
  const float tlow = float(std::atof(argv[2]));
  int xdim = 0, ydim = 0, zdim = 0;
  float *image = nullptr;
  SegMed::read_data(&xdim, &ydim, &zdim, &image, argv[1]);
  const std::size_t n = std::size_t(xdim) * ydim * zdim;
  float *edge_mag = (float *)std::malloc(sizeof(float) * n);

  // 1. The diffused field, as the patched GVF_Compute computes it before its critical points.
  SegMed::XDIM = xdim;
  SegMed::YDIM = ydim;
  SegMed::ZDIM = zdim;
  SegMed::dataset = image;
  SegMed::t_low = tlow;
  SegMed::GetGradient(edge_mag);
  SegMed::velocity = (SegMed::VECTOR *)std::malloc(sizeof(SegMed::VECTOR) * n);
  SegMed::InitGVF(edge_mag);
  SegMed::gvfflow();
  {
    std::FILE *f = std::fopen(argv[4], "wb");
    if (!f) {
      std::fprintf(stderr, "segmed_legacy: cannot write %s\n", argv[4]);
      return 1;
    }
    bool ok = true;
    for (std::size_t v = 0; v < n && ok; ++v) {
      const float c[3] = {SegMed::velocity[v].x, SegMed::velocity[v].y, SegMed::velocity[v].z};
      ok = write_le_floats(f, c, 3);
    }
    std::fclose(f);
    if (!ok) {
      std::fprintf(stderr, "segmed_legacy: cannot write %s\n", argv[4]);
      return 1;
    }
  }
  std::free(SegMed::velocity);
  SegMed::velocity = nullptr;

  // 2. The critical points: GVF_Compute, whole.
  SegMed::CPNT *critical = nullptr;
  SegMed::GVF_Compute(xdim, ydim, zdim, image, edge_mag, &critical, tlow);
  std::FILE *f = std::fopen(argv[3], "wb");
  if (!f) {
    std::fprintf(stderr, "segmed_legacy: cannot write %s\n", argv[3]);
    return 1;
  }
  for (SegMed::CPNT *p = critical; p;) {
    std::fprintf(f, "%d %d %d\n", p->x, p->y, p->z);
    SegMed::CPNT *next = p->next;
    std::free(p);
    p = next;
  }
  std::fclose(f);
  std::free(edge_mag);
  std::free(image);
  return 0;
}
