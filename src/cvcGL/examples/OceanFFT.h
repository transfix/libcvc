/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  ---------------------------------------------------------------------------
  The GPU FFT ocean here is a C++/VTK port of ABYSSAL
  (github.com/Token-Gremlin/natural-disasters), MIT licensed,
  Copyright (c) 2026 Davi (Token-Gremlin). The spectrum, time evolution,
  butterfly IFFT and assemble math follow that source (see
  docs/roadmap/OCEAN-AND-VOLUMETRIC-TERRAIN-NOTES.md §A.9). MIT notice retained.
*/

#ifndef __CVC_GL_EXAMPLES_OCEANFFT_H__
#define __CVC_GL_EXAMPLES_OCEANFFT_H__

#include <memory>
#include <vector>
#include <vtkSmartPointer.h>

class vtkOpenGLRenderWindow;
class vtkTextureObject;
class vtkOpenGLFramebufferObject;
class vtkOpenGLQuadHelper;

// ---------------------------------------------------------------------------
// OceanFFT — a spectral (JONSWAP + butterfly-IFFT) GPU ocean, single cascade.
// ---------------------------------------------------------------------------
// Entirely fragment-shader ping-pong (no compute shaders — the WebGL2/GLES3
// portable shape). Per the notes' Phase-1 plan:
//   init():  CPU-generate a Gaussian-noise field + the bit-reversal/twiddle
//            butterfly LUT, then a one-time h0 pass builds the JONSWAP spectrum
//            with conjugate symmetry (h0(k) and conj h0(-k)).
//   step():  time-evolve h(k,t); pack 4 complex derivative fields into two
//            RGBA buffers; run an 8-stage Cooley-Tukey butterfly IFFT (2
//            directions) on each buffer; assemble -> displacement(xyz)+foam.
//
// displacement() layout (RGBA32F, N x N):
//   .x = lambda * horizontal displacement X   (choppy)
//   .y = vertical height                       (the wave height)
//   .z = lambda * horizontal displacement Z    (choppy)
//   .w = Jacobian (folding) -> foam where < 1
//
// Depends on VTK only (a vtkOpenGLRenderWindow*), so it is unit-testable
// headless and drops into any cvcGL SceneRenderer via renderWindow().
class OceanFFT {
public:
  explicit OceanFFT(int n = 256);
  ~OceanFFT();

  OceanFFT(const OceanFFT &) = delete;
  OceanFFT &operator=(const OceanFFT &) = delete;

  // Allocate GL resources + build the spectrum. Context must be current AND
  // initialised (draw one frame first — float-RT support is only known after
  // the window's OpenGLInit). Returns false if float RTs are unsupported.
  bool init(vtkOpenGLRenderWindow *rw);

  // Phase-0 proof: a known multi-pass float-RT ping-pong, read back bit-exact.
  bool selfTest();

  // Advance to time `t` and leave the result in displacement().
  void step(double timeSeconds);

  // Re-derive the JONSWAP parameters from the current tunables and rebuild the
  // spectrum (the one-time h0 pass). Call after changing tileSize_m / windSpeed /
  // fetch_m / depth_m / windDir* live (e.g. from the state tree). Cheap: one pass.
  void rebuildSpectrum();

  // The displacement/foam field (see layout above). Null until init()+step().
  vtkTextureObject *displacement() const;

  // Host copy of displacement(): N*N*4 floats, row-major RGBA (empty on failure).
  std::vector<float> readbackDisplacement() const;

  int resolution() const { return m_n; }
  bool ready() const { return m_ready; }
  void releaseResources();

  // Tunables (set before init(); sensible ocean defaults otherwise).
  float tileSize_m = 200.0f; // spatial size of one FFT tile (world metres)
  float windSpeed = 11.0f;   // m/s, drives the JONSWAP peak
  float fetch_m = 120000.0f; // m
  float depth_m = 1000.0f;   // deep water
  float windDirX = 1.0f, windDirZ = 0.55f;
  float chop = 1.15f; // horizontal-displacement (choppiness) scale

private:
  // one full-screen pass: bind `dst` to the FBO, run `quad`, sampling is set up
  // by the caller via the quad's uniforms just before calling.
  void beginPass(vtkTextureObject *dst);
  void endPass();
  void runH0Pass(); // (re)build the spectrum into m_h0; context must be current

  int m_n;
  int m_log2n = 0;
  bool m_ready = false;
  vtkOpenGLRenderWindow *m_rw = nullptr; // not owned

  // spectrum params derived in init()
  float m_alpha = 0.f, m_peakOmega = 0.f, m_gamma = 3.3f;

  vtkSmartPointer<vtkTextureObject> m_noise;     // rg = Gaussian pair
  vtkSmartPointer<vtkTextureObject> m_butterfly; // (log2N x N): xy=twiddle, zw=indices
  vtkSmartPointer<vtkTextureObject> m_h0;        // xy=h0(k), zw=conj h0(-k)
  vtkSmartPointer<vtkTextureObject> m_a0, m_b0;  // buffer-0 ping-pong (Dx+iDz, Dy+iDyDx)
  vtkSmartPointer<vtkTextureObject> m_a1, m_b1;  // buffer-1 ping-pong (DyDz+iDxDx, DzDz+iDxDz)
  vtkSmartPointer<vtkTextureObject> m_disp;      // final displacement + foam
  vtkSmartPointer<vtkOpenGLFramebufferObject> m_fbo;

  std::unique_ptr<vtkOpenGLQuadHelper> m_h0Pass;
  std::unique_ptr<vtkOpenGLQuadHelper> m_time0;
  std::unique_ptr<vtkOpenGLQuadHelper> m_time1;
  std::unique_ptr<vtkOpenGLQuadHelper> m_butterflyPass;
  std::unique_ptr<vtkOpenGLQuadHelper> m_assemble;
};

#endif // __CVC_GL_EXAMPLES_OCEANFFT_H__
