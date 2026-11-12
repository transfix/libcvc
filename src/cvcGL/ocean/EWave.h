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
// O8b, O8c): the same window as cvc::ocean::ewave (cvc/ocean/ewave.h, the CPU), in float.
// ---------------------------------------------------------------------------
//   EWave w(params);          // cvc::ocean::ewave_params: n, dx, origin, depth, damping, sponge
//   w.init(renderWindow);     // after the window's first frame; false without float targets
//   w.setMask(mask);          // 1 open water, 0 wall (no-flux by default), between: beach
//   w.addDrop(x, y, -0.2, 1.0);
//   w.step(1.0 / 30.0);
//   w.heightTexture();        // RG32F: .r the height h (m), .g the potential phi (m^2/s)
//
// The state z = h + i phi lives in an RG32F texture, one complex value per texel, cell (i, j) at
// texel (i, j). All passes are WebGL2-portable fragment shaders (no compute shaders), each with
// the scissor test off (OceanButterfly's oceanBeginPass). A step takes one of the CPU's two paths:
//
// Without ghost cells (no walls, walls = zero): the exact step, 4 log2 n + 2 passes, 34 at n = 256:
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
//
// With ghost cells (walls = no_flux, the default, near water): the CPU's symmetric step
// (Stormer-Verlet on h_t = M^-1 R^T G~ R phi, phi_t = -g h), in ewave_substeps(dt) substeps of
// 4 log2 n + 3 passes each, 35 at n = 256:
//   - the fill pass: water texels write phi_half = phi - (g dt_s / 2) h, ghost texels the
//     weighted sum of their four corners' phi_half (fetched one by one: float textures are not
//     filtered in WebGL2), other walls 0, as the real part of a complex field;
//   - the forward FFT (2 log2 n), the multiply pass (kappa~ / n^2, computed per texel from
//     uniforms), the inverse FFT (2 log2 n);
//   - the gather pass: each water texel adds to its own value of G~ R phi_half, for each of up to
//     ewave_max_readers ghosts that read it, that ghost's weight on it times the value at the
//     ghost (R^T), multiplies by 1 / M and writes h' = h + dt_s M^-1 S phi_half and
//     phi' = phi_half - (g dt_s / 2) h'; walls write 0.
//   Then, with damping_k2, the same four stages once more on h (the filter M^-1 R^T F R, 4 log2 n
//   + 3 passes), and the real-space pass (the beach and sponge on h at twice their rate, the
//   global damping on h and phi, walls 0). So a step is s (4 log2 n + 3) + 1 passes, 36 at
//   n = 256 for one substep (dt up to 1.8 / omega_max: 1/30 s at 0.25 m cells), plus 4 log2 n + 3
//   with damping_k2.
//   The ghosts, weights and readers' limit are the CPU's (cvc::ocean::ewave_wall_ghosts and
//   ewave_lumped_mass, in double at setMask): the corners as an RG32F texture and the weights as
//   an RGBA32F one for the fill; for the gather, each water texel's readers (the ghost's cell as a
//   float index, exact to 2^24 = 4096^2, and its weight, two readers to an RGBA32F texel, in the
//   CPU table's order) in ewave_max_readers / 2 textures, and 1 / M as an R32F one.
//
// The coefficients are computed per texel from uniforms, so a new dt costs nothing; the mask is
// an R32F texture.
//
// Sources go into h when they are added, times the mask, as on the CPU (add_height and
// add_heights change h at once, and the next step moves them on): addDrop and addHeights are one
// pass each. A source added between steps is in the next step's first transform, so the GPU
// window and the CPU one give the same field for the same calls.
//
// The state is in heightTexture(). It is the same texture after every step(); a source pass moves
// it to another of the three state-sized textures until the next step brings it back, so a
// renderer takes heightTexture() after stepping. Filtering is nearest (a float texture's linear
// filtering is not core in WebGL2); the texture is clamped, as the window does not tile. Walls
// hold 0 after a step.
//
// Float, not double: against the CPU window the difference is float round-off through two
// transforms a substep (cvcgl_ewave measures it). Depends on VTK's OpenGL2 module and cvc::ocean
// (for ewave_params and the ghost table), so it is unit-testable headless.
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
  // The ghost cells setMask found (0: the exact step).
  std::size_t ghosts() const { return m_ghosts; }
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

  // The substeps step(dt) takes (1 for the exact step), the passes it is meant to run (see above),
  // and the passes actually rendered since construction (step()s and sources, counted pass by
  // pass), for measuring.
  int substeps(double dt) const;
  int passesPerStep(double dt) const;
  long long passesRun() const { return m_passes; }

  // The reader textures the gather pass reads: two readers to a texel.
  static constexpr int kReaderTextures = (cvc::ocean::ewave_max_readers + 1) / 2;

private:
  // One source pass: h += (a drop at cell (cx, cy) of radius r cells, if drop, plus m_src, if
  // field) times the mask.
  void sourcePass(bool drop, double cx, double cy, double r, double amount, bool field);
  void exactStep(double dt);
  void symmetricStep(double dt);
  // One symmetric stage from the state in m_z[st], into m_z[a] (m_z[b] the other work texture):
  // mode 0 a Stormer-Verlet substep of dts, mode 1 damping_k2's filter on h for a step of dt.
  void stage(int mode, int st, int a, int b, double dts, double dt);

  cvc::ocean::ewave_params m_p;
  int m_log2n = 0;
  bool m_ready = false;
  vtkOpenGLRenderWindow *m_rw = nullptr; // not owned
  double m_time = 0.0;
  long long m_passes = 0;

  std::vector<float> m_maskHost;        // n x n, 1 until setMask
  std::vector<float> m_ghostCornerHost; // n x n (i0, j0), -1 where a cell is not a ghost
  std::vector<float> m_ghostWHost;      // n x n x 4 corner weights
  std::vector<float> m_invMassHost;     // n x n: 1 / M on water, 0 on walls (with ghosts)
  std::vector<float> m_readerHost;      // kReaderTextures x n x n x 4: (cell, w, cell, w)
  std::size_t m_ghosts = 0;             // ghost cells (0: the exact step)

  OceanButterfly m_fft;
  vtkSmartPointer<vtkTextureObject> m_z[3]; // RG32F: the state and two work textures
  int m_state = 0;                          // which m_z holds the state (0 after a step)
  vtkSmartPointer<vtkTextureObject> m_mask; // R32F
  vtkSmartPointer<vtkTextureObject> m_src;  // R32F, addHeights' field
  vtkSmartPointer<vtkTextureObject> m_ghostCorner;              // RG32F
  vtkSmartPointer<vtkTextureObject> m_ghostW;                   // RGBA32F
  vtkSmartPointer<vtkTextureObject> m_invMass;                  // R32F
  vtkSmartPointer<vtkTextureObject> m_readers[kReaderTextures]; // RGBA32F
  vtkSmartPointer<vtkOpenGLFramebufferObject> m_fbo;

  std::unique_ptr<vtkOpenGLQuadHelper> m_update;
  std::unique_ptr<vtkOpenGLQuadHelper> m_real;
  std::unique_ptr<vtkOpenGLQuadHelper> m_source;
  std::unique_ptr<vtkOpenGLQuadHelper> m_fill;
  std::unique_ptr<vtkOpenGLQuadHelper> m_multiply;
  std::unique_ptr<vtkOpenGLQuadHelper> m_gather;
};

#endif // __CVC_GL_OCEAN_EWAVE_H__
