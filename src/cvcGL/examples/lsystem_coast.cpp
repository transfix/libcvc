/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// lsystem_coast — a trial cvcGL island+ocean demo that hosts the GPU FFT-ocean
// core (OceanFFT), the port target from docs/roadmap/OCEAN-AND-VOLUMETRIC-
// TERRAIN-NOTES.md. It is a SEPARATE demo from lsystem_forest on purpose:
// lsystem_forest is the untouched performance control (Lab roadmap §1.3).
//
// Phase status (see the notes):
//   * OceanFFT runs the real JONSWAP spectrum + butterfly IFFT (3 cascades).
//   * DEFAULT (GPU no-readback): the ocean is a STATIC grid whose shader samples
//     the OceanFFT displacement textures and displaces + shades on the GPU (a
//     Fresnel water BRDF). No per-frame CPU readback / updateVertices — the
//     WebGL2-viable path. GeometryNode::setShaderTexture binds the live textures.
//   * --cpu: a fallback that reads OceanFFT back and CPU-displaces the grid via
//     updateVertices/updateColors (native only — the readback is not a WebGL2 op).

#include "OceanFFT.h"

#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkRenderer.h>

namespace {

constexpr double ISLAND_HALF = 120.0; // island terrain spans [-ISLAND_HALF, ISLAND_HALF]
constexpr int ISLAND_N = 72;          // island mesh resolution (verts per side)
constexpr double OCEAN_HALF = 220.0;  // ocean extends past the island to the horizon
constexpr int OCEAN_N = 224;          // ocean grid resolution (verts per side)
constexpr int FFT_N = 256;            // OceanFFT working resolution (ABYSSAL default)
constexpr double SEA_LEVEL = 0.0;     // wave amplitude / choppiness / tile now live in state
constexpr double PEAK = 34.0, SHELF = -9.0;
constexpr double PI = 3.14159265358979323846;
constexpr double SUN_AZ = -52.0; // sun azimuth (deg, 0 = +Y toward +X) — matches the key light
constexpr double SUN_EL = 34.0;  // sun elevation (deg)

// A simple island dome that drops below sea level at the rim (so there is a real
// shoreline for the ocean to meet), plus two octaves of relief.
double islandH(double x, double y) {
  double r2 = x * x + y * y;
  double h = PEAK * std::exp(-r2 / (0.34 * ISLAND_HALF * ISLAND_HALF)) + SHELF;
  h += 4.5 * std::sin(x * 0.045) * std::cos(y * 0.041);
  h += 2.2 * std::sin(x * 0.11 + 1.3) * std::sin(y * 0.097);
  return h;
}

cvc::geometry::color_t islandAlbedo(double x, double y) {
  double h = islandH(x, y);
  const double rock[3] = {0.46, 0.45, 0.43}, grass[3] = {0.27, 0.44, 0.19},
               sand[3] = {0.68, 0.62, 0.44};
  double rockw = std::min(1.0, std::max(0.0, (h - 18.0) / 14.0));
  double shore = std::max(0.0, 1.0 - std::fabs(h - SEA_LEVEL) / 4.5);
  cvc::geometry::color_t c;
  for (int k = 0; k < 3; ++k)
    c[k] = (grass[k] * (1.0 - rockw) + rock[k] * rockw) * (1.0 - shore) + sand[k] * shore;
  return c;
}

cvc::geometry buildIsland(cvc::app &app) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  for (int j = 0; j < ISLAND_N; ++j) {
    double y = -ISLAND_HALF + 2.0 * ISLAND_HALF * j / (ISLAND_N - 1);
    for (int i = 0; i < ISLAND_N; ++i) {
      double x = -ISLAND_HALF + 2.0 * ISLAND_HALF * i / (ISLAND_N - 1);
      pts.push_back({x, y, islandH(x, y)});
      cols.push_back(islandAlbedo(x, y));
    }
  }
  auto &tris = g.tris();
  for (int j = 0; j < ISLAND_N - 1; ++j)
    for (int i = 0; i < ISLAND_N - 1; ++i) {
      cvc::geometry::index_t v = j * ISLAND_N + i;
      tris.push_back({v, static_cast<cvc::geometry::index_t>(v + 1),
                      static_cast<cvc::geometry::index_t>(v + ISLAND_N)});
      tris.push_back({static_cast<cvc::geometry::index_t>(v + 1),
                      static_cast<cvc::geometry::index_t>(v + ISLAND_N + 1),
                      static_cast<cvc::geometry::index_t>(v + ISLAND_N)});
    }
  return g;
}

// A flat ocean grid at z = SEA_LEVEL. Its (x,y) are fixed; z is rewritten each
// frame from the FFT displacement. `baseXY` captures the lattice so the render
// loop only has to supply z.
cvc::geometry buildOcean(cvc::app &app, std::vector<double> &baseXY) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  baseXY.clear();
  baseXY.reserve(static_cast<size_t>(OCEAN_N) * OCEAN_N * 2);
  for (int j = 0; j < OCEAN_N; ++j) {
    double y = -OCEAN_HALF + 2.0 * OCEAN_HALF * j / (OCEAN_N - 1);
    for (int i = 0; i < OCEAN_N; ++i) {
      double x = -OCEAN_HALF + 2.0 * OCEAN_HALF * i / (OCEAN_N - 1);
      pts.push_back({x, y, SEA_LEVEL});
      cols.push_back({0.05, 0.20, 0.32}); // deep-water blue (Phase 1 replaces with optics)
      baseXY.push_back(x);
      baseXY.push_back(y);
    }
  }
  auto &tris = g.tris();
  for (int j = 0; j < OCEAN_N - 1; ++j)
    for (int i = 0; i < OCEAN_N - 1; ++i) {
      cvc::geometry::index_t v = j * OCEAN_N + i;
      tris.push_back({v, static_cast<cvc::geometry::index_t>(v + 1),
                      static_cast<cvc::geometry::index_t>(v + OCEAN_N)});
      tris.push_back({static_cast<cvc::geometry::index_t>(v + 1),
                      static_cast<cvc::geometry::index_t>(v + OCEAN_N + 1),
                      static_cast<cvc::geometry::index_t>(v + OCEAN_N)});
    }
  return g;
}

// ── GPU ocean shaders (no-readback path) ──────────────────────────────────────
// The sea is a STATIC flat grid; its vertex shader samples the 3 OceanFFT
// displacement textures (bound live via GeometryNode::setShaderTexture) and
// displaces on the GPU; its fragment shader runs the water BRDF. No CPU readback,
// no per-frame updateVertices/Colors. Uniforms/samplers are declared here and set
// each draw via setShaderUniform*/setShaderTexture. World XY comes from vertexMC
// (disableCoordinateShiftScale). textureLod is used (vertex-stage texture fetch).
const char *OCEAN_VS_DEC =
    "//VTK::PositionVC::Dec\n"
    "out vec3 vWorldPos;\n"
    "out vec3 vOceanN;\n"
    "out float vOceanJac;\n"
    "uniform sampler2D uDisp0; uniform sampler2D uDisp1; uniform sampler2D uDisp2;\n"
    "uniform float uTile0; uniform float uTile1; uniform float uTile2;\n"
    "uniform float uWeight0; uniform float uWeight1; uniform float uWeight2;\n"
    "uniform float uWaveAmp; uniform float uChop;\n"
    "vec4 oSum(vec2 w){\n"
    "  vec4 a=textureLod(uDisp0,w/uTile0,0.0), b=textureLod(uDisp1,w/uTile1,0.0), "
    "c=textureLod(uDisp2,w/uTile2,0.0);\n"
    "  return vec4(uWeight0*a.xyz+uWeight1*b.xyz+uWeight2*c.xyz, min(min(a.w,b.w),c.w));\n"
    "}\n"
    "float oHt(vec2 w){ return "
    "uWeight0*textureLod(uDisp0,w/uTile0,0.0).y+uWeight1*textureLod(uDisp1,w/"
    "uTile1,0.0).y+uWeight2*textureLod(uDisp2,w/uTile2,0.0).y; }\n";
// Full replacement of the position impl: vertexMC is an immutable `in` (with
// shift-scale disabled), so displace a mutable COPY and drive gl_Position /
// vertexVCVSOutput from it — replicating VTK's own two lines with the copy.
const char *OCEAN_VS_IMPL =
    "  vec4 dispMC = vertexMC;\n"
    "  vec2 owxy = dispMC.xy;\n"
    "  vec4 od = oSum(owxy);\n"
    "  float hpx=oHt(owxy+vec2(1.0,0.0)), hmx=oHt(owxy-vec2(1.0,0.0));\n"
    "  float hpy=oHt(owxy+vec2(0.0,1.0)), hmy=oHt(owxy-vec2(0.0,1.0));\n"
    "  vec3 oN = normalize(vec3(-uWaveAmp*(hpx-hmx)*0.5, -uWaveAmp*(hpy-hmy)*0.5, 1.0));\n"
    "  dispMC.xy += uChop*vec2(od.x, od.z);\n"
    "  dispMC.z  += uWaveAmp*od.y;\n"
    "  vertexVCVSOutput = MCVCMatrix * dispMC;\n"
    "  gl_Position = MCDCMatrix * dispMC;\n"
    "  vWorldPos = dispMC.xyz;\n"
    "  vOceanN = oN;\n"
    "  vOceanJac = od.w;\n";
const char *OCEAN_FS_DEC = "//VTK::Normal::Dec\n"
                           "in vec3 vWorldPos;\n"
                           "in vec3 vOceanN;\n"
                           "in float vOceanJac;\n"
                           "uniform vec3 uCamPos; uniform vec3 uSunDir; uniform float uFoamBias;\n";
// Consumes //VTK::Light::Impl (VTK's lighting is replaced): writes the final
// water colour to gl_FragData[0] directly.
const char *OCEAN_FS_LIGHT =
    "  vec3 N = normalize(vOceanN);\n"
    "  vec3 Vd = normalize(uCamPos - vWorldPos);\n"
    "  float NdotV = max(dot(N,Vd), 0.0);\n"
    "  float Fr = 0.02 + 0.98*pow(1.0-NdotV, 5.0);\n"
    "  float NdotL = max(dot(N, uSunDir), 0.0);\n"
    "  float lit = 0.30 + 0.70*NdotL;\n"
    "  vec3 Rr = 2.0*NdotV*N - Vd;\n"
    "  float skyT = clamp(0.5+0.5*Rr.z, 0.0, 1.0);\n"
    "  float glint = pow(max(dot(Rr, uSunDir), 0.0), 200.0)*1.8;\n"
    "  vec3 deep = vec3(0.015,0.085,0.130)*lit;\n"
    "  vec3 sky = mix(vec3(0.62,0.66,0.70), vec3(0.24,0.44,0.72), skyT);\n"
    "  vec3 col = mix(deep, sky, Fr) + glint*vec3(1.0,0.96,0.88);\n"
    "  float foam = clamp((uFoamBias - vOceanJac)/0.85, 0.0, 1.0); foam=foam*foam*(3.0-2.0*foam);\n"
    "  col = mix(col, vec3(0.92,0.95,0.96), foam);\n"
    "  gl_FragData[0] = vec4(col, 1.0);\n";

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  bool offscreen = false, verbose = false, useCpu = false;
  int frames = 0, width = 1280, height = 800;
  double fps = 30.0;
  std::string png;
  std::vector<std::string> sets;
  po::options_description desc("lsystem_coast — a cvcGL island + GPU FFT-ocean trial\nOptions");
  desc.add_options()("help,h", "show this help and exit")                                    //
      ("offscreen", po::bool_switch(&offscreen), "render offscreen (no window)")             //
      ("verbose,v", po::bool_switch(&verbose), "show cvcGL debug logging")                   //
      ("cpu", po::bool_switch(&useCpu),                                                      //
       "use the CPU readback ocean path (default: GPU no-readback shader path)")             //
      ("frames", po::value<int>(&frames)->default_value(0),                                  //
       "stop after N frames (0 = until the window closes)")                                  //
      ("png", po::value<std::string>(&png)->default_value(""), "write the final frame here") //
      ("width", po::value<int>(&width)->default_value(1280), "render width")                 //
      ("height", po::value<int>(&height)->default_value(800), "render height")               //
      ("fps", po::value<double>(&fps)->default_value(30.0), "synthetic clock rate")          //
      ("set", po::value<std::vector<std::string>>(&sets)->composing(),                       //
       "override a state knob coast.<key>=<value>, e.g. --set water.wind_speed=20 "
       "(repeatable; keys under coast.water.* and coast.clock.*)");
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  if (!png.empty() && frames == 0)
    frames = 1;             // a --png with no frame count still needs at least one frame
  const bool gpu = !useCpu; // GPU no-readback path by default (the WASM-viable one)

  cvc::app app;
  app.properties("system.log_verbosity", verbose ? "6" : "2");

  // Apply any --set overrides to the state tree BEFORE the knobs are seeded, so
  // a supplied value wins over the default (knob() only defaults when empty).
  for (const std::string &kv : sets) {
    const auto eq = kv.find('=');
    if (eq == std::string::npos) {
      std::fprintf(stderr, "ignoring --set '%s' (want coast.<key>=<value>)\n", kv.c_str());
      continue;
    }
    const std::string key = kv.substr(0, eq), val = kv.substr(eq + 1);
    cvc::state::instance(app)("coast." + key).value(val);
    std::printf("state override: coast.%s = %s\n", key.c_str(), val.c_str());
  }

  SceneGraph sg(app, "coast");

  sg.addGraphics("island", buildIsland(app));
  auto island = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("island"));
  island->setUseSingleColor(false);

  std::vector<double> oceanXY;
  sg.addGraphics("ocean", buildOcean(app, oceanXY));
  auto ocean = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("ocean"));
  ocean->setUseSingleColor(false);
  // The ocean is CPU-shaded: a full water BRDF (Fresnel sky-reflection + sun
  // glint + foam) is baked into the vertex colours each frame, because VTK's
  // Phong lighting cannot express Fresnel or reflection. So VTK lighting is
  // disabled on the sea (ambient shows the colour as authored). GeometryNode::
  // updateNormals remains the path for VTK-LIT deformed meshes; the sea owns its
  // shading and instead uses the CPU normals directly in the BRDF below.
  ocean->setAmbient(1.0);
  ocean->setDiffuse(0.0);
  ocean->setSpecular(0.0);

  // The island stays VTK-lit: a sun (key) + a wide fill.
  sg.addDirectionalLight(SUN_AZ, SUN_EL, 1.0, 0.96, 0.9, 1.15);
  sg.addFillLight(0.0, 0.0, 400.0, 0.0, 0.0, 0.0, 0.6, 0.7, 0.85, 0.35);

  SceneRenderer view(sg, width, height, offscreen, "main");
  view.renderer()->GradientBackgroundOn();
  view.renderer()->SetBackground(0.66, 0.71, 0.74);  // horizon
  view.renderer()->SetBackground2(0.23, 0.44, 0.80); // zenith
  view.setCamera(0.0, -252.0, 58.0, 0.0, 12.0, 2.0, 0.0, 0.0, 1.0, 40.0, 1.0, 5000.0);

  // Draw one frame FIRST so the GL context is fully initialised — vtkTextureObject
  // only knows the context supports float render targets after the window's
  // OpenGLInit has run (otherwise Create2DFromRaw reports IF=0 and we would
  // wrongly fall back to a flat sea).
  view.render();

  // Declutter AFTER the first frame: adding graphics re-runs updateGrid, which
  // would otherwise re-show the world-bounds grid/axis over the scene.
  sg.setGridVisible(false);
  sg.setAxisVisible(false);
  view.renderer()->ResetCameraClippingRange();

  // ── Water knobs + world clock, all in the state tree (scriptable & live) ────
  // Every lever lives at coast.water.* / coast.clock.*, so it is settable from
  // Python (cvc.state) or any state consumer and takes effect on the next frame.
  auto knob = [&](const char *path, double def) -> double {
    cvc::state &n = cvc::state::instance(app)(path);
    if (n.value().empty())
      n.value(def);
    return n.value<double>();
  };

  // A 3-cascade spectral FFT ocean (ABYSSAL-style, non-harmonic tiles): each
  // cascade is an independent OceanFFT at a different tile size, summed for
  // multi-scale detail (swell + wind waves + ripples). Global state knobs scale
  // ALL cascades — each cascade's wind = coast.water.wind_speed * its factor.
  struct Cascade {
    std::unique_ptr<OceanFFT> fft;
    double tile;
    double windFactor;
    double weight;
    std::vector<float> disp;
  };
  std::vector<Cascade> cascades;
  const struct {
    double tile, windF, weight;
  } CASC[] = {
      {251.0, 1.00, 1.00}, // swell
      {83.0, 0.80, 0.55},  // wind waves
      {29.0, 0.60, 0.32},  // ripples
  };
  for (const auto &c : CASC) {
    Cascade cc;
    cc.fft = std::make_unique<OceanFFT>(FFT_N);
    cc.tile = c.tile;
    cc.windFactor = c.windF;
    cc.weight = c.weight;
    cc.fft->tileSize_m = static_cast<float>(c.tile);
    cc.fft->windSpeed = static_cast<float>(knob("coast.water.wind_speed", 11.0) * c.windF);
    cc.fft->fetch_m = static_cast<float>(knob("coast.water.fetch", 120000.0));
    cc.fft->depth_m = static_cast<float>(knob("coast.water.depth", 1000.0));
    cc.fft->windDirX = static_cast<float>(knob("coast.water.wind_dir_x", 1.0));
    cc.fft->windDirZ = static_cast<float>(knob("coast.water.wind_dir_z", 0.55));
    cc.fft->chop = static_cast<float>(knob("coast.water.chop", 1.15));
    cascades.push_back(std::move(cc));
  }
  knob("coast.water.wave_amp", 1.2);   // vertical scale (raw FFT height is metres)
  knob("coast.water.choppiness", 1.0); // horizontal displacement scale
  knob("coast.water.foam_bias", 0.55); // Jacobian threshold below which foam appears
  knob("coast.clock.scale", 1.0);      // world seconds per wall second (0 pauses)
  knob("coast.clock.paused", 0.0);

  auto *ow = vtkOpenGLRenderWindow::SafeDownCast(view.renderWindow());
  bool oceanReady = ow != nullptr;
  for (auto &c : cascades)
    oceanReady = oceanReady && c.fft->init(ow);
  if (oceanReady) {
    const bool st = cascades[0].fft->selfTest();
    std::printf("OceanFFT self-test: %s (%zu-cascade spectral FFT at %dx%d)\n",
                st ? "PASS" : "FAIL", cascades.size(), FFT_N, FFT_N);
  } else {
    std::printf("OceanFFT unavailable (no float render targets?) — rendering a flat sea.\n");
  }

  // GPU no-readback path: bind the cascade displacement textures live and install
  // the displace + BRDF shaders, so the sea is posed and shaded entirely on the
  // GPU — no per-frame CPU readback / updateVertices (the WASM-friendly path).
  if (gpu && oceanReady) {
    ocean->setUseSingleColor(true);       // colour comes from the BRDF, not vertex colours
    ocean->disableCoordinateShiftScale(); // vertexMC.xy == world xy
    const char *dispName[3] = {"uDisp0", "uDisp1", "uDisp2"};
    const char *tileName[3] = {"uTile0", "uTile1", "uTile2"};
    const char *wtName[3] = {"uWeight0", "uWeight1", "uWeight2"};
    for (size_t i = 0; i < cascades.size() && i < 3; ++i) {
      ocean->setShaderTexture(dispName[i], cascades[i].fft->displacement());
      ocean->setShaderUniformf(tileName[i], static_cast<float>(cascades[i].tile));
      ocean->setShaderUniformf(wtName[i], static_cast<float>(cascades[i].weight));
    }
    ocean->addVertexShaderReplacement("//VTK::PositionVC::Dec", OCEAN_VS_DEC);
    ocean->addVertexShaderReplacement("//VTK::PositionVC::Impl", OCEAN_VS_IMPL);
    ocean->addFragmentShaderReplacement("//VTK::Normal::Dec", OCEAN_FS_DEC);
    ocean->addFragmentShaderReplacement("//VTK::Light::Impl", OCEAN_FS_LIGHT);
    std::printf("ocean: GPU no-readback path (%zu cascades, displace + BRDF in-shader)\n",
                cascades.size());
  }

  // World clock: the waves advance in WORLD time (correct speed regardless of
  // frame rate; honours coast.clock.scale/.paused; deterministic in a fixed-frame
  // capture). cvc::world_clock is the authoritative sim clock; publish its time.
  cvc::world_clock clock;
  auto wallPrev = std::chrono::steady_clock::now();
  const bool deterministic = frames > 0; // fixed-frame capture -> reproducible
  double lastWind = knob("coast.water.wind_speed", 11.0),
         lastFetch = knob("coast.water.fetch", 120000.0),
         lastDepth = knob("coast.water.depth", 1000.0),
         lastWdx = knob("coast.water.wind_dir_x", 1.0),
         lastWdz = knob("coast.water.wind_dir_z", 0.55);

  // Sun direction (toward the sun), matching the key light, for the water BRDF.
  const double azr = SUN_AZ * PI / 180.0, elr = SUN_EL * PI / 180.0;
  const double sunx = std::sin(azr) * std::cos(elr), suny = std::cos(azr) * std::cos(elr),
               sunz = std::sin(elr);

  const size_t oceanVerts = static_cast<size_t>(OCEAN_N) * OCEAN_N;
  long frame = 0;
  std::vector<double> xyz(oceanVerts * 3);
  std::vector<unsigned char> rgb(oceanVerts * 3);

  while (!view.windowClosed()) {
    // Live per-frame knobs.
    const double waveAmp = knob("coast.water.wave_amp", 1.2);
    const double choppy = knob("coast.water.choppiness", 1.0);
    const float foamBias = static_cast<float>(knob("coast.water.foam_bias", 0.55));
    for (auto &c : cascades)
      c.fft->chop = static_cast<float>(knob("coast.water.chop", 1.15));

    // Spectrum knobs: rebuild ALL cascades only when one actually changes.
    const double wind = knob("coast.water.wind_speed", 11.0),
                 fch = knob("coast.water.fetch", 120000.0), dep = knob("coast.water.depth", 1000.0),
                 wdx = knob("coast.water.wind_dir_x", 1.0),
                 wdz = knob("coast.water.wind_dir_z", 0.55);
    if (oceanReady && (wind != lastWind || fch != lastFetch || dep != lastDepth || wdx != lastWdx ||
                       wdz != lastWdz)) {
      for (auto &c : cascades) {
        c.fft->windSpeed = static_cast<float>(wind * c.windFactor);
        c.fft->fetch_m = static_cast<float>(fch);
        c.fft->depth_m = static_cast<float>(dep);
        c.fft->windDirX = static_cast<float>(wdx);
        c.fft->windDirZ = static_cast<float>(wdz);
        c.fft->rebuildSpectrum();
      }
      lastWind = wind;
      lastFetch = fch;
      lastDepth = dep;
      lastWdx = wdx;
      lastWdz = wdz;
    }

    // Advance the world clock and drive the ocean by WORLD time.
    clock.set_scale(knob("coast.clock.scale", 1.0));
    clock.set_mode(knob("coast.clock.paused", 0.0) > 0.5 ? cvc::world_clock::mode::paused
                                                         : cvc::world_clock::mode::live);
    double wallDt;
    if (deterministic) {
      wallDt = 1.0 / fps;
    } else {
      const auto now = std::chrono::steady_clock::now();
      wallDt = std::chrono::duration<double>(now - wallPrev).count();
      wallPrev = now;
    }
    clock.advance(wallDt);
    const double wt = clock.t();
    cvc::state::instance(app)("coast.clock.t").value(wt);

    // Live camera position for the water BRDF's view-dependent Fresnel + glint.
    double camp[3] = {0, 0, 0};
    if (view.renderer() && view.renderer()->GetActiveCamera())
      view.renderer()->GetActiveCamera()->GetPosition(camp);

    if (gpu && oceanReady) {
      // GPU path: evolve the cascades on the GPU (no readback) and push the live
      // uniforms; the mesh shader displaces + shades from the bound disp textures.
      for (auto &c : cascades)
        c.fft->step(wt);
      ocean->setShaderUniformf("uWaveAmp", static_cast<float>(waveAmp));
      ocean->setShaderUniformf("uChop", static_cast<float>(choppy));
      ocean->setShaderUniform3f("uCamPos", static_cast<float>(camp[0]), static_cast<float>(camp[1]),
                                static_cast<float>(camp[2]));
      ocean->setShaderUniform3f("uSunDir", static_cast<float>(sunx), static_cast<float>(suny),
                                static_cast<float>(sunz));
      ocean->setShaderUniformf("uFoamBias", foamBias);
    } else if (oceanReady) {
      // Evolve + read back every cascade for this frame.
      bool ok = true;
      for (auto &c : cascades) {
        c.fft->step(wt);
        c.disp = c.fft->readbackDisplacement();
        ok = ok && c.disp.size() == static_cast<size_t>(FFT_N) * FFT_N * 4;
      }
      if (ok) {
        // Sum the cascades at a world position (each tiled at its own metre scale):
        // weighted displacement, and the min Jacobian (most-folded cascade -> foam).
        auto sampleSum = [&](double wx, double wy, double &oDx, double &oH, double &oDz,
                             double &oJac) {
          oDx = oH = oDz = 0.0;
          oJac = 1e9;
          for (const auto &c : cascades) {
            const int fx =
                ((static_cast<int>(std::floor(wx / c.tile * FFT_N)) % FFT_N) + FFT_N) % FFT_N;
            const int fy =
                ((static_cast<int>(std::floor(wy / c.tile * FFT_N)) % FFT_N) + FFT_N) % FFT_N;
            const size_t b = (static_cast<size_t>(fy) * FFT_N + fx) * 4;
            oDx += c.weight * c.disp[b + 0];
            oH += c.weight * c.disp[b + 1];
            oDz += c.weight * c.disp[b + 2];
            oJac = std::min(oJac, static_cast<double>(c.disp[b + 3]));
          }
        };
        auto sumH = [&](double wx, double wy) {
          double h = 0.0;
          for (const auto &c : cascades) {
            const int fx =
                ((static_cast<int>(std::floor(wx / c.tile * FFT_N)) % FFT_N) + FFT_N) % FFT_N;
            const int fy =
                ((static_cast<int>(std::floor(wy / c.tile * FFT_N)) % FFT_N) + FFT_N) % FFT_N;
            h += c.weight * c.disp[(static_cast<size_t>(fy) * FFT_N + fx) * 4 + 1];
          }
          return h;
        };
        const double delta = 1.3; // world metres for the normal's central differences
        for (size_t k = 0; k < oceanVerts; ++k) {
          const double wx = oceanXY[k * 2 + 0], wy = oceanXY[k * 2 + 1];
          double dX, H, dZ, jac;
          sampleSum(wx, wy, dX, H, dZ, jac);

          // Choppy (Gerstner-like) horizontal displacement + vertical height.
          xyz[k * 3 + 0] = wx + choppy * dX;
          xyz[k * 3 + 1] = wy + choppy * dZ;
          xyz[k * 3 + 2] = SEA_LEVEL + waveAmp * H;

          // Smooth normal from the SUMMED-height gradient (world-space central
          // differences). z-up surface z=f(x,y): n = normalize(-df/dx, -df/dy, 1).
          const double gx =
              -waveAmp * (sumH(wx + delta, wy) - sumH(wx - delta, wy)) / (2.0 * delta);
          const double gy =
              -waveAmp * (sumH(wx, wy + delta) - sumH(wx, wy - delta)) / (2.0 * delta);
          const double nl = std::sqrt(gx * gx + gy * gy + 1.0);
          const double Nx = gx / nl, Ny = gy / nl, Nz = 1.0 / nl;

          // ── CPU water BRDF: deep body + Fresnel sky reflection + sun glint + foam ──
          const double px = xyz[k * 3 + 0], py = xyz[k * 3 + 1], pz = xyz[k * 3 + 2];
          double vx = camp[0] - px, vy = camp[1] - py, vz = camp[2] - pz;
          const double vl = std::sqrt(vx * vx + vy * vy + vz * vz) + 1e-9;
          vx /= vl;
          vy /= vl;
          vz /= vl; // view direction (toward camera)
          const double NdotV = std::max(Nx * vx + Ny * vy + Nz * vz, 0.0);
          const double F = 0.02 + 0.98 * std::pow(1.0 - NdotV, 5.0); // Fresnel, F0~=0.02
          const double NdotL = std::max(Nx * sunx + Ny * suny + Nz * sunz, 0.0);
          const double lit = 0.30 + 0.70 * NdotL; // deep-water body lambert term
          // reflected view ray -> a sky gradient and a sharp sun glint
          const double Rx = 2.0 * NdotV * Nx - vx, Ry = 2.0 * NdotV * Ny - vy,
                       Rz = 2.0 * NdotV * Nz - vz;
          const double skyT = std::clamp(0.5 + 0.5 * Rz, 0.0, 1.0);
          const double glint =
              std::pow(std::max(Rx * sunx + Ry * suny + Rz * sunz, 0.0), 200.0) * 1.8;
          const double deepR = 0.015 * lit, deepG = 0.085 * lit, deepB = 0.130 * lit;
          const double skyR = 0.62 * (1 - skyT) + 0.24 * skyT;
          const double skyG = 0.66 * (1 - skyT) + 0.44 * skyT;
          const double skyB = 0.70 * (1 - skyT) + 0.72 * skyT;
          double cr = deepR * (1 - F) + skyR * F + glint * 1.00;
          double cg = deepG * (1 - F) + skyG * F + glint * 0.96;
          double cb = deepB * (1 - F) + skyB * F + glint * 0.88;
          double foam = std::clamp(
              (static_cast<double>(foamBias) - static_cast<double>(jac)) / 0.85, 0.0, 1.0);
          foam = foam * foam * (3.0 - 2.0 * foam); // smoothstep
          cr += (0.92 - cr) * foam;
          cg += (0.95 - cg) * foam;
          cb += (0.96 - cb) * foam;
          rgb[k * 3 + 0] = static_cast<unsigned char>(std::clamp(cr * 255.0, 0.0, 255.0));
          rgb[k * 3 + 1] = static_cast<unsigned char>(std::clamp(cg * 255.0, 0.0, 255.0));
          rgb[k * 3 + 2] = static_cast<unsigned char>(std::clamp(cb * 255.0, 0.0, 255.0));
        }
        ocean->updateVertices(xyz);
        ocean->updateColors(rgb);
      }
    }

    view.processUIEvents();
    view.render();
    ++frame;
    if (frames > 0 && frame >= frames)
      break;
  }

  if (!png.empty()) {
    view.writePNG(png);
    std::printf("wrote %s\n", png.c_str());
  }
  return 0;
}
