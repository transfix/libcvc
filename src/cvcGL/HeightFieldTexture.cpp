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

#include <algorithm>
#include <cmath>
#include <cvc/gl/HeightFieldTexture.h>
#include <stdexcept>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include <vtkTextureObject.h>
#include <vtk_glad.h>

namespace cvc {
namespace gl {

HeightFieldTexture::HeightFieldTexture(int nx, int ny, double x0, double y0, double dx, double dy)
    : m_nx(nx), m_ny(ny), m_x0(x0), m_y0(y0), m_dx(dx), m_dy(dy) {
  if (nx <= 0 || ny <= 0 || !(dx > 0.0) || !(dy > 0.0))
    throw std::invalid_argument("HeightFieldTexture: need nx, ny >= 1 and dx, dy > 0");
  m_heights = std::make_shared<const std::vector<float>>(static_cast<std::size_t>(nx) * ny, 0.0f);
}

// The texture's own resource callback releases it with its context, whichever
// dies first; m_context is only ever compared, never dereferenced here.
HeightFieldTexture::~HeightFieldTexture() = default;

void HeightFieldTexture::rangeLocked() {
  const std::vector<float> &h = *m_heights;
  if (h.empty())
    return;
  const auto mm = std::minmax_element(h.begin(), h.end());
  m_hLo = *mm.first;
  m_hHi = *mm.second;
}

void HeightFieldTexture::setHeights(std::shared_ptr<const std::vector<float>> heights) {
  if (!heights || heights->size() != static_cast<std::size_t>(m_nx) * m_ny)
    throw std::invalid_argument("HeightFieldTexture::setHeights: need nx * ny = " +
                                std::to_string(static_cast<std::size_t>(m_nx) * m_ny) + " heights");
  std::lock_guard<std::mutex> lock(m_mutex);
  m_heights = std::move(heights);
  m_owned.reset();
  rangeLocked();
  m_fullPending = true;
  m_rowLo = m_rowHi = 0;
}

void HeightFieldTexture::setHeights(const std::vector<float> &heights) {
  setHeights(std::make_shared<const std::vector<float>>(heights));
}

void HeightFieldTexture::updateRows(int row0, int rows, const float *heights) {
  if (rows <= 0)
    return;
  if (row0 < 0 || rows > m_ny - row0 || !heights)
    throw std::out_of_range("HeightFieldTexture::updateRows: rows [" + std::to_string(row0) + ", " +
                            std::to_string(row0 + rows) + ") are outside the " +
                            std::to_string(m_ny) + "-row grid");
  std::lock_guard<std::mutex> lock(m_mutex);
  if (!m_owned) { // copy-on-write: never patch a buffer the caller shared with us
    m_owned = std::make_shared<std::vector<float>>(*m_heights);
    m_heights = m_owned;
  }
  const std::size_t off = static_cast<std::size_t>(row0) * m_nx;
  const std::size_t cnt = static_cast<std::size_t>(rows) * m_nx;
  std::copy(heights, heights + cnt, m_owned->begin() + static_cast<std::ptrdiff_t>(off));
  const auto mm = std::minmax_element(heights, heights + cnt);
  m_hLo = std::min(m_hLo, *mm.first); // widen only: conservative, and O(rows)
  m_hHi = std::max(m_hHi, *mm.second);
  if (m_rowHi <= m_rowLo) {
    m_rowLo = row0;
    m_rowHi = row0 + rows;
  } else {
    m_rowLo = std::min(m_rowLo, row0);
    m_rowHi = std::max(m_rowHi, row0 + rows);
  }
}

double HeightFieldTexture::sample(double x, double y) const {
  std::lock_guard<std::mutex> lock(m_mutex);
  const std::vector<float> &h = *m_heights;
  // Mirrors cvc_hf_height() in glsl() operation for operation.
  const double fx = (x - m_x0) / m_dx, fy = (y - m_y0) / m_dy;
  const double flx = std::floor(fx), fly = std::floor(fy);
  const double rx = fx - flx, ry = fy - fly;
  auto clampi = [](double v, int hi) {
    return static_cast<int>(std::min<double>(std::max<double>(v, 0.0), hi));
  };
  const int i0 = clampi(flx, m_nx - 1), i1 = clampi(flx + 1.0, m_nx - 1);
  const int j0 = clampi(fly, m_ny - 1), j1 = clampi(fly + 1.0, m_ny - 1);
  auto at = [&](int i, int j) {
    return static_cast<double>(h[static_cast<std::size_t>(j) * m_nx + i]);
  };
  const double a = at(i0, j0) + (at(i1, j0) - at(i0, j0)) * rx;
  const double b = at(i0, j1) + (at(i1, j1) - at(i0, j1)) * rx;
  return a + (b - a) * ry;
}

cvc::bounding_box HeightFieldTexture::extent() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return cvc::bounding_box(m_x0, m_y0, m_hLo, m_x0 + (m_nx - 1) * m_dx, m_y0 + (m_ny - 1) * m_dy,
                           m_hHi);
}

vtkTextureObject *HeightFieldTexture::texture() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_texture;
}

HeightFieldTexture::Stats HeightFieldTexture::stats() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_stats;
}

vtkTextureObject *HeightFieldTexture::prepare(vtkOpenGLRenderWindow *window) {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (!window)
    return m_texture;
  if (!m_texture || m_context != window || m_texture->GetHandle() == 0) {
    // First use, another window, or the old context released it: start over.
    m_texture = vtkSmartPointer<vtkTextureObject>::New();
    m_context = window;
    m_fullPending = true;
  }
  const float *data = m_heights->data();
  if (m_fullPending) {
    m_texture->SetContext(window);
    m_texture->SetMinificationFilter(vtkTextureObject::Nearest);
    m_texture->SetMagnificationFilter(vtkTextureObject::Nearest);
    m_texture->SetWrapS(vtkTextureObject::ClampToEdge);
    m_texture->SetWrapT(vtkTextureObject::ClampToEdge);
    // R32F, explicitly: a normalised default would clamp heights to [0, 1].
    m_texture->SetInternalFormat(GL_R32F);
    m_texture->SetFormat(GL_RED);
    m_texture->SetDataType(GL_FLOAT);
    m_texture->Create2DFromRaw(static_cast<unsigned int>(m_nx), static_cast<unsigned int>(m_ny), 1,
                               VTK_FLOAT, const_cast<float *>(data));
    m_fullPending = false;
    m_rowLo = m_rowHi = 0;
    ++m_stats.fullUploads;
    m_stats.bytes += static_cast<std::uint64_t>(m_nx) * m_ny * sizeof(float);
  } else if (m_rowHi > m_rowLo) {
    m_texture->Activate();
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    window->GetState()->vtkglPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, m_rowLo, m_nx, m_rowHi - m_rowLo, GL_RED, GL_FLOAT,
                    data + static_cast<std::size_t>(m_rowLo) * m_nx);
    m_texture->Deactivate();
    ++m_stats.rowUploads;
    m_stats.bytes += static_cast<std::uint64_t>(m_rowHi - m_rowLo) * m_nx * sizeof(float);
    m_rowLo = m_rowHi = 0;
  }
  return m_texture;
}

const char *HeightFieldTexture::glsl() {
  // texelFetch + manual bilinear: identical arithmetic to sample().
  return "uniform highp sampler2D cvcHF;\n"
         "uniform vec3 cvcHFOrigin;\n"
         "uniform vec3 cvcHFSpacing;\n"
         "float cvc_hf_height(vec2 p) {\n"
         "  ivec2 sz = textureSize(cvcHF, 0);\n"
         "  vec2 f = (p - cvcHFOrigin.xy) / cvcHFSpacing.xy;\n"
         "  vec2 fl = floor(f);\n"
         "  vec2 fr = f - fl;\n"
         "  ivec2 i0 = clamp(ivec2(fl), ivec2(0), sz - ivec2(1));\n"
         "  ivec2 i1 = clamp(ivec2(fl) + ivec2(1), ivec2(0), sz - ivec2(1));\n"
         "  float h00 = texelFetch(cvcHF, ivec2(i0.x, i0.y), 0).r;\n"
         "  float h10 = texelFetch(cvcHF, ivec2(i1.x, i0.y), 0).r;\n"
         "  float h01 = texelFetch(cvcHF, ivec2(i0.x, i1.y), 0).r;\n"
         "  float h11 = texelFetch(cvcHF, ivec2(i1.x, i1.y), 0).r;\n"
         "  float a = h00 + (h10 - h00) * fr.x;\n"
         "  float b = h01 + (h11 - h01) * fr.x;\n"
         "  return a + (b - a) * fr.y;\n"
         "}\n";
}

} // namespace gl
} // namespace cvc
