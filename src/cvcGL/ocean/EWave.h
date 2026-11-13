/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_OCEAN_EWAVE_H__
#define __CVC_GL_OCEAN_EWAVE_H__

#include "OceanButterfly.h"

#include <cvc/ocean/ewave.h>
#include <memory>
#include <vector>
#include <vtkSmartPointer.h>

class vtkOpenGLRenderWindow;
class vtkTextureObject;
class vtkOpenGLFramebufferObject;
class vtkOpenGLQuadHelper;

// ---------------------------------------------------------------------------
// EWave — waves that bodies make, on the GPU (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md C1, C3,
// O8b): the same window as cvc::ocean::ewave (cvc/ocean/ewave.h, the CPU), in float.
// ---------------------------------------------------------------------------
//   EWave w(params);          // cvc::ocean::ewave_params: n, dx, origin, depth, damping, sponge
//   w.init(renderWindow);     // after the window's first frame; false without float targets
//   w.setMask(mask);          // 1 open water, 0 land, between: absorbing beach
//   w.addDrop(x, y, -0.2, 1.0);
//   w.step(1.0 / 30.0);
//   w.heightTexture();        // RG32F: .r the height h (m), .g the potential phi (m^2/s)
//
// The state z = h + i phi lives in an RG32F texture, one complex value per texel, cell (i, j) at
// texel (i, j). One step of dt is 4 log2 n + 2 fragment passes, 34 at n = 256 (no compute shaders):
//   - the forward FFT (OceanButterfly, the twiddles conjugated): 2 log2 n passes;
//   - the update pass: each texel reads Z(k) and Z(-k), separates H(k) = (Z(k) + conj Z(-k)) / 2
//     and PHI(k) = (Z(k) - conj Z(-k)) / 2i, moves them on by the exact per-mode update and
//     damping (cvc/ocean/ewave.h has the formulas) and writes H' + i PHI'. In this natural layout
//     the Nyquist index n / 2 is its own mirror, so its row and column pair up like any other and
//     nothing is dropped: the packing is the CPU's exactly. (OceanFFT's centred layout is the one
//     that leaves the Nyquist row without a partner.) k = 0 keeps its mean height and has its mean
//     potential held at 0, as on the CPU;
//   - the inverse FFT: 2 log2 n passes;
//   - the real-space pass: normalises by 1 / n^2, then damps by the mask's beach rate and the
//     sponge as exp(-rate dt) and zeroes land, as the CPU does.
// Each mode's rates (omega, kappa / omega, g / omega and the damping rate) are worked out once in
// double, with the CPU window's formulas, and held in an RGBA32F texture; the update pass takes
// only the cos, sin and exp of a step, so a new dt costs nothing, and the rates don't depend on how
// a driver rounds sqrt and tanh (EWave.cpp says why that matters). The mask is an R32F texture.
//
// Sources go into h when they are added, times the mask, as on the CPU (add_height and
// add_heights change h at once, and the next step moves them on): addDrop and addHeights are one
// pass each. A source added between steps is in the next step's forward transform, so the GPU
// window and the CPU one give the same field for the same calls.
//
// The state is in heightTexture(). It is the same texture after every step(); a source pass moves
// it to another of the three state-sized textures until the next step brings it back, so a
// renderer takes heightTexture() after stepping. Filtering is nearest (a float texture's linear
// filtering is not core in WebGL2); the texture is clamped, as the window does not tile.
//
// Float, not double: against the CPU window the difference is float round-off through two
// transforms a step (cvcgl_ewave measures it). Depends on VTK's OpenGL2 module and cvc::ocean
// (for ewave_params), so it is unit-testable headless.
class EWave {
public:
  // Throws std::invalid_argument for parameters ewave would refuse.
  explicit EWave(const cvc::ocean::ewave_params &p);
  ~EWave();
  EWave(const EWave &) = delete;
  EWave &operator=(const EWave &) = delete;

  // Allocates the textures and programs; the window starts flat (h = phi = 0) with the mask
  // setMask last gave, or open (mask 1) if it was never called. The context must be current and
  // initialised (draw one frame first). False if float render targets or a program are
  // unavailable; the window then does nothing.
  //
  // Unlike the CPU class, only the mask is held before init(): setState, addDrop, addHeights and
  // step check their arguments (and throw as ewave does) but otherwise do nothing until init()
  // has succeeded, and a later init() starts flat again. Call init() first, then set the state.
  bool init(vtkOpenGLRenderWindow *rw);
  bool ready() const { return m_ready; }
  void releaseResources();

  const cvc::ocean::ewave_params &params() const { return m_p; }
  int resolution() const { return m_p.n; }

  // n x n values in [0, 1], row-major, as ewave::set_mask. Throws std::invalid_argument for a
  // wrong size or a value out of range. Held, and uploaded now if init() has run, else by init().
  void setMask(const std::vector<double> &mask);
  // Replaces the state (n x n heights and potentials, row-major), not masked, as ewave::set_state.
  // Ignored before init() (see init()), as are addDrop and addHeights.
  void setState(const std::vector<double> &h, const std::vector<double> &phi);
  // Adds amount (m) at world (x, y) over a bump of the given radius (m), amount (1 - r^2 / R^2)^3,
  // times the mask, as ewave::add_height. Throws std::invalid_argument as it does.
  void addDrop(double x, double y, double amount, double radius);
  // Adds n x n heights (m), row-major, times the mask, as ewave::add_heights (C2's sources).
  void addHeights(const std::vector<double> &dh);

  // Moves the waves on by dt (s, above 0 and finite).
  void step(double dt);
  double time() const { return m_time; }

  // The state texture (see above). Null until init().
  vtkTextureObject *heightTexture() const;
  // Host copies: n x n heights, or (h, phi) pairs (2 n^2 floats), row-major. Empty on failure.
  std::vector<float> readbackHeight() const;
  std::vector<float> readbackState() const;

  // The passes step() is meant to run (4 log2 n + 2), and the passes actually rendered since
  // construction (step()s and sources, counted pass by pass), for measuring.
  int passesPerStep() const;
  long long passesRun() const { return m_passes; }

private:
  // One source pass: h += (a drop at cell (cx, cy) of radius r cells, if drop, plus m_src, if
  // field) times the mask.
  void sourcePass(bool drop, double cx, double cy, double r, double amount, bool field);

  cvc::ocean::ewave_params m_p;
  int m_log2n = 0;
  bool m_ready = false;
  vtkOpenGLRenderWindow *m_rw = nullptr; // not owned
  double m_time = 0.0;
  long long m_passes = 0;

  std::vector<float> m_maskHost; // n x n, 1 until setMask

  OceanButterfly m_fft;
  vtkSmartPointer<vtkTextureObject> m_z[3]; // RG32F: the state and two work textures
  int m_state = 0;                          // which m_z holds the state (0 after a step)
  vtkSmartPointer<vtkTextureObject> m_mask; // R32F
  vtkSmartPointer<vtkTextureObject> m_src;  // R32F, addHeights' field
  vtkSmartPointer<vtkTextureObject> m_rates; // RGBA32F, each mode's rates (EWave.cpp)
  vtkSmartPointer<vtkOpenGLFramebufferObject> m_fbo;

  std::unique_ptr<vtkOpenGLQuadHelper> m_update;
  std::unique_ptr<vtkOpenGLQuadHelper> m_real;
  std::unique_ptr<vtkOpenGLQuadHelper> m_source;
};

#endif // __CVC_GL_OCEAN_EWAVE_H__
