/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  ---------------------------------------------------------------------------
  The turbulence integration, the windrow-carved foam and the shading terms here (GGX with
  height-correlated Smith visibility and a disc-widened lobe, the roughness-softened Fresnel, the
  footprint roughness from Cox-Munk's slopes, the light through backlit crests) follow ABYSSAL
  (github.com/Token-Gremlin/natural-disasters, MIT licensed, Copyright (c) 2026 Davi
  (Token-Gremlin)): src/ocean/OceanFFT.js (the assemble pass's turbulence), src/ocean/OceanMesh.js
  and src/gfx/ShadingGLSL.js, at commit 849ff7f4. MIT notice retained.
*/

#ifndef __CVC_GL_OCEAN_OCEANFOAM_H__
#define __CVC_GL_OCEAN_OCEANFOAM_H__

#include <atomic>
#include <cvc/core/sliced_task.h>
#include <cvc/ocean/whitecaps.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkSmartPointer.h>

namespace cvc {
class app;
}

class vtkOpenGLRenderWindow;
class vtkTextureObject;
class vtkOpenGLFramebufferObject;
class vtkOpenGLQuadHelper;

// OceanFoam — the persistent foam of the sea's long cascades (OCEAN-AND-WEATHER-ROADMAP.md
// section 10.2, O5): foam that builds where a cascade's waves fold and thins over seconds.
//
// The turbulence lives in each cascade's own texels, so it tiles as its cascade does and stays
// where its wave broke while the crest moves on. All the cascades' tiles are side by side in one
// RGBA32F atlas, `tiles` tiles of n x n (tile c at texels c n .. c n + n - 1 across), so the water
// takes one sampler for all of them; two such atlases ping-pong (the one drawn is texture()). Each
// texel holds cvc::ocean::foam_state: (foam, bubbles, crest, spray).
//
// step() is one pass over the atlas: each texel reads its cascade's Jacobian (OceanFFT's
// displacement().w), takes the fold's injection smoothstep((bias - J) / ramp) and integrates it
// as cvc::ocean::foam_step does (the decay factors worked out once a frame on the CPU). Fragment
// ping-pong only, texelFetch only: the WebGL2 shape (the float targets need
// EXT_color_buffer_float, as OceanFFT's do).
//
// oceanFoamGLSL() is what the water's fragment shader reads it with (and the shading terms):
// oFoamTile (one tile of the atlas at world xy, bilinear by hand), oWindrow (the windrow noise),
// oFoamLook (cvc::ocean::foam_look), and oGgxD / oSmith / oFresnel / oFootprintAlpha / oSunSpec /
// oBody for the A4 shading. It declares no uniforms, so a host splices it in once beside its own.
class OceanFoam {
public:
  // n a power of two (each cascade's texels a side), tiles in 1 .. 3.
  OceanFoam(int n, int tiles);
  ~OceanFoam();
  OceanFoam(const OceanFoam &) = delete;
  OceanFoam &operator=(const OceanFoam &) = delete;

  // Allocates the atlases and the pass; starts with no foam. The context must be current and
  // initialised. False if float render targets or the program are unavailable.
  bool init(vtkOpenGLRenderWindow *rw);
  bool ready() const { return m_ready; }
  void releaseResources();

  // One frame, dt_s seconds after the last (clamped as foam_step clamps it): tile c takes
  // disp[c]'s Jacobian (.w). disp holds tiles() textures of n x n.
  void step(const std::vector<vtkTextureObject *> &disp, double dt_s);
  // No foam anywhere.
  void clear();

  vtkTextureObject *texture() const;   // the atlas last written (null until init)
  std::vector<float> readback() const; // texture() on the host: (n tiles) x n RGBA, row-major
  int resolution() const { return m_n; }
  int tiles() const { return m_tiles; }
  long long passesRun() const { return m_passes; }

  double bias = 0.9; // the fold's threshold on each cascade's J (foam_biases::persistent)
  double ramp = 0.1; // and its ramp (foam_params::ramp)
  cvc::ocean::foam_persistence params;

private:
  int m_n, m_tiles;
  bool m_ready = false;
  int m_cur = 0; // the atlas last written
  long long m_passes = 0;
  vtkOpenGLRenderWindow *m_rw = nullptr; // not owned
  vtkSmartPointer<vtkTextureObject> m_atlas[2];
  vtkSmartPointer<vtkOpenGLFramebufferObject> m_fbo;
  std::unique_ptr<vtkOpenGLQuadHelper> m_pass;
};

// The GLSL (a function: a DLL exports no data symbols).
const char *oceanFoamGLSL();

// The persistent foam's two thresholds for Monahan's share (cvc::ocean::foam_history: the
// cascades evolved on the CPU over a few seconds, the long cascades' foam integrated texel by
// texel), found off the frame as a sliced task (cvc/core/sliced_task.h): on the background pool
// natively, a slice a frame in single-threaded wasm. A unit is one cascade's Jacobian at one time
// (about 10 ms); the biases are one last piece (0.2 to 2 s, more at high winds). The host polls
// finished() and then reads biases(); cancel() drops it at its next unit.
class OceanFoamCalibration : public cvc::sliced_job {
public:
  OceanFoamCalibration(const std::vector<cvc::ocean::h0_field> &hs,
                       const cvc::ocean::foam_params &p, const cvc::ocean::foam_persistence &q,
                       double share);
  bool step(std::size_t budget) override;
  bool done() const override { return m_done.load(std::memory_order_acquire); }
  void cancel() override { m_cancel.store(true, std::memory_order_release); }
  bool cancelled() const override { return m_cancel.load(std::memory_order_acquire); }
  const char *phase() const override;
  double progress() const override;
  std::string status() const override;
  bool finished() const { return m_finished.load(std::memory_order_acquire); }
  cvc::ocean::foam_biases biases() const { return m_biases; } // once finished()

  // Submits it to app's scheduler (`subject` names it, `owner` tags it), or, where no task can
  // start, runs it here. Null only if it cannot be made (a bad h0).
  static std::shared_ptr<OceanFoamCalibration>
  start(cvc::app &app, const std::vector<cvc::ocean::h0_field> &hs,
        const cvc::ocean::foam_params &p, const cvc::ocean::foam_persistence &q, double share,
        const std::string &subject, const std::string &owner);

private:
  cvc::ocean::foam_history m_h;
  double m_share;
  cvc::ocean::foam_biases m_biases;
  std::atomic<bool> m_done{false}, m_cancel{false}, m_finished{false};
  std::atomic<int> m_phase{0};
  std::atomic<double> m_progress{0.0};
};

#endif // __CVC_GL_OCEAN_OCEANFOAM_H__
