/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  ---------------------------------------------------------------------------
  The butterfly LUT and pass here are a C++/VTK port of ABYSSAL
  (github.com/Token-Gremlin/natural-disasters), MIT licensed,
  Copyright (c) 2026 Davi (Token-Gremlin), moved out of OceanFFT.cpp so the
  eWave window (EWave.cpp) runs the same transform. MIT notice retained.
*/

#ifndef __CVC_GL_OCEAN_OCEANBUTTERFLY_H__
#define __CVC_GL_OCEAN_OCEANBUTTERFLY_H__

#include <memory>
#include <vtkSmartPointer.h>

class vtkOpenGLRenderWindow;
class vtkTextureObject;
class vtkOpenGLFramebufferObject;
class vtkOpenGLQuadHelper;

// What OceanFFT and EWave share: the full-screen pass setup and the 2-D complex FFT as
// fragment-shader butterfly passes (no compute shaders: the WebGL2/GLES3 portable shape).
//
// The transform is radix-2 Cooley-Tukey, decimation in time, with the bit reversal folded into
// stage 0's indices: N x N complex values in natural order in, natural order out, unnormalized,
//   inverse: a[p] <- sum over m of a[m] exp(+2 pi i m p / N)
//   forward: a[p] <- sum over m of a[m] exp(-2 pi i m p / N)   (the twiddles conjugated)
// along x (log2 N passes) and then along y (log2 N more), the convention of fft.h's run_2d. A
// texel holds two complex values, .rg and .ba, transformed together; a two-channel (RG) target
// holds one, and the pass's .ba is dropped.

// The vertex shader every ocean pass uses (see OceanButterfly.cpp for why it reads texCoordIn).
extern const char *const kOceanQuadVS;

// Binds dst to fbo as its one draw buffer, sets the n x n viewport and disables the scissor test,
// blending and depth testing. The renderer leaves the scissor test on, boxed to the window: a
// window smaller than the field (a 64-pixel offscreen test, a small canvas) would clip every pass
// to its corner and leave the rest of each texture stale. The caller restores the scissor test
// (vtkOpenGLState::ScopedglEnableDisable) and the framebuffer bindings around its passes.
void oceanBeginPass(vtkOpenGLRenderWindow *rw, vtkOpenGLFramebufferObject *fbo,
                    vtkTextureObject *dst, int n);

// A w x h float texture of `comps` channels (1, 2 or 4: R32F, RG32F, RGBA32F), nearest
// filtering, clamped, filled from `seed` (w * h * comps floats) or left undefined. Null when the
// context cannot make it (no float textures).
vtkSmartPointer<vtkTextureObject> oceanFloatTexture(vtkOpenGLRenderWindow *rw, int w, int h,
                                                    int comps, const float *seed);

class OceanButterfly {
public:
  OceanButterfly();
  ~OceanButterfly();
  OceanButterfly(const OceanButterfly &) = delete;
  OceanButterfly &operator=(const OceanButterfly &) = delete;

  // Builds the (log2 N x N) twiddle/index LUT and the pass's program. n a power of two. The
  // context must be current. False if either fails.
  bool init(vtkOpenGLRenderWindow *rw, int n);
  void release();

  int stages() const { return m_log2n; } // log2 N: passes per direction
  int passes() const { return 2 * m_log2n; }

  // One pass: dst <- stage `stage` (0 .. log2 N - 1) of the transform along x (vertical false) or
  // y of src. src and dst must differ. forward conjugates the twiddles.
  void pass(vtkOpenGLFramebufferObject *fbo, vtkTextureObject *src, vtkTextureObject *dst,
            int stage, bool vertical, bool forward);

  // The whole 2-D transform of src: 2 log2 N passes, the first reading src and writing a, then
  // each reading the last one's output and writing b, a, b, ... The pass count is even, so the
  // result is in b; src may be b (a plain ping-pong) or a third texture, which is left as it was.
  // Returns b.
  vtkTextureObject *transform(vtkOpenGLFramebufferObject *fbo, vtkTextureObject *src,
                              vtkTextureObject *a, vtkTextureObject *b, bool forward);

  // The butterfly passes pass() has rendered since construction, for measuring.
  long long passesRun() const { return m_passesRun; }

private:
  long long m_passesRun = 0;
  int m_n = 0;
  int m_log2n = 0;
  vtkOpenGLRenderWindow *m_rw = nullptr;   // not owned
  vtkSmartPointer<vtkTextureObject> m_lut; // (log2N x N): xy = twiddle exp(+i ang), zw = indices
  std::unique_ptr<vtkOpenGLQuadHelper> m_quad;
};

#endif // __CVC_GL_OCEAN_OCEANBUTTERFLY_H__
