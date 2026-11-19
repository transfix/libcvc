/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  ---------------------------------------------------------------------------
  The spectrum's time evolution, the butterfly IFFT and the assemble pass are OceanFFT's, which
  port ABYSSAL (github.com/Token-Gremlin/natural-disasters, MIT licensed, Copyright (c) 2026 Davi
  (Token-Gremlin)); see OceanFFT.h. MIT notice retained.
*/

#ifndef __CVC_GL_OCEAN_OCEANCASCADES_H__
#define __CVC_GL_OCEAN_OCEANCASCADES_H__

#include "OceanButterfly.h"

#include <memory>
#include <vector>
#include <vtkSmartPointer.h>

class vtkOpenGLRenderWindow;
class vtkTextureObject;
class vtkOpenGLFramebufferObject;
class vtkOpenGLQuadHelper;

namespace cvc {
namespace ocean {
struct h0_field;
}
} // namespace cvc

// OceanCascades — up to three FFT ocean cascades (OceanFFT's arithmetic, texel for texel) run as
// one (OCEAN-AND-WEATHER-ROADMAP.md §10.10, the water's frame cost).
//
// A cascade's passes are small (256^2), so their cost is their count, not their pixels: about
// 50 us each on a laptop GPU however big, and three OceanFFTs take 105 of them a frame. Here every
// texture is an atlas of tiles side by side, and every pass does all the tiles at once:
//   - h0: the cascades' spectra, tile c cascade c (cascades x n by n);
//   - the work buffers: both of each cascade's derivative buffers, tile 2 c its buffer 0 and tile
//     2 c + 1 its buffer 1 (2 cascades x n by n), two of them ping-ponged;
//   - displacement(): tile c cascade c's displacement and Jacobian, OceanFFT's layout per tile
//     (.x = chop Dx, .y = height, .z = chop Dz, .w = J).
//   - slopes(): not an atlas but a 2D texture array, layer c cascade c's height slopes (.x =
//     dh/dx, .y = dh/dz: the spectrum's own derivatives, which the time pass works out for the
//     buffers anyway), half floats, repeating and mipmapped. A fragment reads it through the
//     hardware's filter (textureGrad) at its own footprint, so the slopes a pixel cannot resolve
//     average away instead of aliasing; one sampler for all the cascades. RG16F is filterable in
//     WebGL2 and renderable with the EXT_color_buffer_float the passes need anyway.
// One time pass writes every buffer, 2 log2 n butterfly passes transform every tile
// (OceanButterfly with tiles), one assemble writes every displacement and one more every slope
// layer (a draw buffer a layer), then the slopes' mipmaps: 19 passes for three 256^2 cascades. The
// atlas is nearest and clamped: a shader reads tile c at texel (c n + i, j) with i, j wrapped into
// the tile (texelFetch), which is also one sampler for all the cascades. Each cascade's tiles sit
// together, so the first k cascades are the atlases' left part: with setActive(k) the passes cover
// only that (adaptive quality, scene_quality.h, drops the short cascade so). Fragment ping-pong
// only; texelFetch only but for the slopes' filtered reads (WebGL2).
class OceanCascades {
public:
  // n a power of two; cascades in 1 .. 3.
  OceanCascades(int n, int cascades);
  ~OceanCascades();
  OceanCascades(const OceanCascades &) = delete;
  OceanCascades &operator=(const OceanCascades &) = delete;

  // Allocates the atlases and the passes; a flat sea until setH0. The context must be current and
  // initialised. False if float render targets or a program are unavailable.
  bool init(vtkOpenGLRenderWindow *rw);
  bool ready() const { return m_ready; }
  void releaseResources();

  // Cascade c's h0 (cvc::ocean::build_h0 at this resolution), held and uploaded now if init() has
  // run, else by init(); its tile size becomes the cascade's. False (and nothing changes) for a
  // wrong resolution or cascade.
  bool setH0(int c, const cvc::ocean::h0_field &h0);

  // Every active cascade to time t: 3 + 2 log2 n passes (and the slopes' mipmaps).
  void step(double timeSeconds);

  // How many cascades step() transforms, from the first (1 .. cascades(); all after init). The
  // others' displacement tiles keep what they last held.
  void setActive(int k);
  int active() const { return m_active; }

  vtkTextureObject *displacement() const;          // the displacement atlas (null until init)
  std::vector<float> readbackDisplacement() const; // its texels: (cascades n) x n RGBA, row-major
  vtkTextureObject *slopes() const;                // the slope layers (null until init)
  // Cascade c's slope layer at mip level 0: n x n (dh/dx, dh/dz) pairs, row-major; empty if none.
  std::vector<float> readbackSlopes(int c) const;
  int resolution() const { return m_n; }
  int cascades() const { return m_c; }
  double tileSize(int c) const { return c >= 0 && c < m_c ? m_tile[std::size_t(c)] : 0.0; }
  long long passesRun() const { return m_passes; }

  float depth_m = 1000.0f; // the dispersion relation's depth (OceanFFT's)
  float chop = 1.15f;      // the sideways moves' scale (OceanFFT's)

private:
  void uploadH0();

  int m_n, m_c, m_log2n = 0, m_active;
  bool m_ready = false;
  long long m_passes = 0;
  vtkOpenGLRenderWindow *m_rw = nullptr; // not owned
  std::vector<double> m_tile;
  std::vector<std::vector<float>> m_h0Host; // each cascade's texels (empty: flat)
  OceanButterfly m_fft;
  vtkSmartPointer<vtkTextureObject> m_h0, m_a, m_b, m_disp, m_slope;
  // the slope pass's own framebuffer: a layer of m_slope on each of its color attachments
  vtkSmartPointer<vtkOpenGLFramebufferObject> m_fbo, m_slopeFbo;
  std::unique_ptr<vtkOpenGLQuadHelper> m_time, m_assemble, m_slopePass;
};

#endif // __CVC_GL_OCEAN_OCEANCASCADES_H__
