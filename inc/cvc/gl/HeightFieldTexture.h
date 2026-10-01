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

#ifndef CVC_GL_HEIGHT_FIELD_TEXTURE_H
#define CVC_GL_HEIGHT_FIELD_TEXTURE_H

#include <cstdint>
#include <cvc/volume/bounding_box.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <vtkSmartPointer.h>

class vtkOpenGLRenderWindow;
class vtkTextureObject;

namespace cvc {
namespace gl {

// ---------------------------------------------------------------------------
// HeightFieldTexture -- a terrain height grid on the GPU, for vertex shaders
// that drape geometry over the ground (DrapedLinkNode).
//
// The grid is nx x ny samples, row-major (row j runs along x at y0 + j*dy):
// sample (i, j) is the height at (x0 + i*dx, y0 + j*dy). Between samples the
// height is BILINEAR, outside the grid it is clamped to the edge -- on the GPU
// and in sample() alike, so a CPU caller (a picker, a test, the terrain code
// that placed the vehicles) can know exactly where a draped vertex lands. If
// the terrain's own sampler uses a different grid or interpolation, draped
// geometry will float or sink by the difference.
//
// The texture is R32F read with texelFetch and a highp sampler, interpolated
// by hand: WebGL2 does not guarantee linear filtering of float textures.
//
// Heights may be set and patched from ANY thread; the GL texture is created
// and updated on the render thread by prepare(), lazily, in whatever context
// draws it. One texture may be shared by many nodes. nx and ny must not exceed
// GL_MAX_TEXTURE_SIZE.
class HeightFieldTexture {
public:
  struct Stats {
    std::uint64_t fullUploads = 0; // whole-texture uploads (creation, setHeights)
    std::uint64_t rowUploads = 0;  // glTexSubImage2D row-band uploads (updateRows)
    std::uint64_t bytes = 0;       // bytes in both
  };

  // Starts flat (all zero). Throws std::invalid_argument for a grid with no
  // samples or a non-positive spacing.
  HeightFieldTexture(int nx, int ny, double x0, double y0, double dx, double dy);
  ~HeightFieldTexture();
  HeightFieldTexture(const HeightFieldTexture &) = delete;
  HeightFieldTexture &operator=(const HeightFieldTexture &) = delete;

  int nx() const { return m_nx; }
  int ny() const { return m_ny; }
  double x0() const { return m_x0; }
  double y0() const { return m_y0; }
  double dx() const { return m_dx; }
  double dy() const { return m_dy; }

  // Replace every height (nx * ny values). The buffer is shared, not copied, so
  // it must not change afterwards. Any thread; one full upload at the next draw.
  void setHeights(std::shared_ptr<const std::vector<float>> heights);
  // Convenience copy of the above.
  void setHeights(const std::vector<float> &heights);
  // Replace rows [row0, row0 + rows) (rows * nx values). Any thread; uploaded
  // as one row band at the next draw. Throws std::out_of_range.
  void updateRows(int row0, int rows, const float *heights);

  // The height a draped vertex at (x, y) gets -- the GPU's arithmetic, on the CPU.
  double sample(double x, double y) const;
  // XY extent of the grid; z spans the current heights.
  cvc::bounding_box extent() const;

  // Render thread, `window`'s context current: create the texture on first use
  // (or in a new context) and upload what changed. Returns the texture.
  vtkTextureObject *prepare(vtkOpenGLRenderWindow *window);
  // The GL texture, or nullptr before the first prepare().
  vtkTextureObject *texture() const;

  Stats stats() const;

  // The GLSL the draping shaders splice in: the sampler and grid uniforms, and
  // `float cvc_hf_height(vec2 p)`, the exact counterpart of sample().
  static const char *glsl();
  // Uniform names in glsl(): the sampler, and the grid as two vec3 (x0, y0, 0)
  // and (dx, dy, 0).
  static const char *samplerName() { return "cvcHF"; }
  static const char *originName() { return "cvcHFOrigin"; }
  static const char *spacingName() { return "cvcHFSpacing"; }

private:
  void rangeLocked();

  const int m_nx, m_ny;
  const double m_x0, m_y0, m_dx, m_dy;
  mutable std::mutex m_mutex;
  std::shared_ptr<const std::vector<float>> m_heights; // current heights (shared or m_owned)
  std::shared_ptr<std::vector<float>> m_owned;         // private copy once rows were patched
  float m_hLo = 0.0f, m_hHi = 0.0f;                    // height range (rows only widen it)
  bool m_fullPending = true;
  int m_rowLo = 0, m_rowHi = 0; // pending row band [lo, hi)
  vtkSmartPointer<vtkTextureObject> m_texture;
  vtkOpenGLRenderWindow *m_context = nullptr;
  Stats m_stats;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_HEIGHT_FIELD_TEXTURE_H
