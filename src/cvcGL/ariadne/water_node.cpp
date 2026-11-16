/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// water_node.cpp — the Ariadne `water` node (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md §11.2,
// O15): a ground's sea, lakes and ponds drawn as the GPU FFT ocean lsystem_coast drew.
//
//   - { id: water, type: water, world: world, level: 0 }
//
// What it draws, everything below `level` on the ground (a generated_world's height field, or any
// height-field node's, `world:` / `ground:`):
//   - near: a grid `near` metres each way from the camera (a vertex every 2 near / (cells - 1)),
//     displaced by three FFT cascades (OceanFFT, the wind sea's h0 from cvc::ocean::build_h0) and
//     the swell's shore layer, the waves fading toward the grid's edge. It moves with the camera by
//     whole cells, so the waves do not swim.
//   - far: a flat grid over the ground's extent with the same shading, its normal the waves' per
//     fragment (the two long cascades and the swell, fading with distance), for the water past the
//     near grid; it gives way inside the near grid.
// The coast (scene_water.h: a shore grid over the ground, cvc::world::water_bodies) shapes both, as
// it shapes the physics sea: each cascade fades out as the still water shoals (half its band's peak
// wavelength, at most `shore_fade`), a sideways move stays under 0.8 of the way to the shore, and
// with a swell (`swell_height` > 0) the shore layer refracts, shoals and breaks it on the beaches
// that face it (cvc::ocean::shore_layer, the sum the CPU evaluates).
// Both grids shade with lsystem_coast's Fresnel water (sky reflection, sun glint, shallows tinted
// by depth, Jacobian whitecaps with a frayed edge, the swell's breakers, a foam band at the
// waterline that the swell strengthens), the sun the renderer's first scene light. Dry ground
// (above the level) discards its fragments. A generated_world under a water node wants `water: 0`,
// so its flat water rungs do not draw over this.
//
// With a swell, the swash runs up the beach on the ground itself: the node adds it to the fragment
// shader of every mesh under its ground node (the `world:` / `ground:` node, found by id at the top
// of the scene), as the cloud deck adds its shade to its receivers. Each wave's front runs up to
// the shore layer's runup and drains back, a film of water with a lace of foam under it, the sand
// dark and glossy behind it until it dries. The physics sea carries the same swash. `swash: 0`
// leaves it out of both.
//
// With `body:` the water is in the scene's physics world (scene_water.h), and the bodies floating
// in it make waves: an eWave window (EWave, 128 m of 0.5 m cells) around the camera's target takes
// each moving body's immersed columns as its source (WakeSource, cvc::physics::add_immersion),
// steps by the world's time, and adds its height to the near grid's vertices and its slope and foam
// to both grids' fragments. Beaches absorb its waves and steep shores reflect them.
//
// Live keys (water.*, or the node's `keys:`; the water_controls.ari panel): wind_speed (m/s),
// wind_dir (degrees, the way the waves travel, from +x toward +y), fetch (m), chop (the cascades'
// sideways moves), wave_amp (a height scale), foam_bias / foam_ramp / foam_fray (the whitecaps'
// Jacobian threshold, its ramp and the noise on its edge), whitecaps (a share of Monahan's cover),
// shore_fade (m), swell_height (m), swell_period (s), swell_dir (degrees), shore_foam (the
// waterline's and the breakers' foam), wake (0 / 1) and wake_foam. A change to the wind rebuilds
// the cascades' h0 (tens of milliseconds); a change to the swell, the coast's shore layer (once the
// keys have held for a few frames).
//
// The cascades and the wake need float render targets (WebGL2's EXT_color_buffer_float). Without
// them, or before the window is up, the water draws as flat blue.

#include "EWave.h"
#include "OceanFFT.h"
#include "WakeSource.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_generated_world.h> // SceneColliders
#include <cvc/ariadne/scene_water.h>           // its physics (WaterRuntime) and coast
#include <cvc/ariadne/scene_world.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/world_nodes.h>
#include <cvc/gl/physics_scene_binding.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_surface.h>
#include <cvc/ocean/shore.h>
#include <cvc/ocean/spectrum.h>
#include <cvc/ocean/whitecaps.h>
#include <cvc/physics/body.h>
#include <cvc/physics/field.h>
#include <cvc/physics/immersion.h>
#include <cvc/physics/pose_tracker.h>
#include <cvc/physics/world.h>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTextureObject.h>

namespace cvc {
namespace gl {
namespace ariadne {
namespace {

namespace ari = cvc::ariadne;

constexpr int kFftN = 256;                            // each cascade's texels a side
constexpr double kDefaultSun[3] = {0.35, 0.25, 0.90}; // when the renderer has no scene light

// ── shaders: lsystem_coast's ocean (examples/cvcgl/lsystem_coast.cpp), without its swash and boat;
// the near grid's world origin is uCenter (its node's translation) ───────────────────────────────

// What both stages read: the cascades, the coast (the bed, the shore distance, the shore layer) and
// the wake. Every uniform says highp, so the two stages' declarations match under WebGL2.
const char *const kWaterCommonDec =
    "uniform highp sampler2D uDisp0; uniform highp sampler2D uDisp1; uniform highp sampler2D "
    "uDisp2;\n"
    "uniform highp float uTile0; uniform highp float uTile1; uniform highp float uTile2;\n"
    "uniform highp float uHalfTexel; uniform highp float uWaveAmp; uniform highp float uLevel;\n"
    "uniform highp vec3 uFade; uniform highp float uWaves; uniform highp float uNearHalf;\n"
    // the shore grid: the signed distance to the shore (+ offshore), on its grid
    "uniform highp sampler2D uShoreGrid; uniform highp vec3 uShoreGridOrigin;\n"
    "uniform highp vec3 uShoreGridSpacing;\n"
    // the shore layer on the same grid: (phase, amplitude, reflected, steepness) and (direction x,
    // y, wavenumber, breaking), and its trains
    "uniform highp sampler2D uShoreWave; uniform highp sampler2D uShoreDir;\n"
    "uniform highp float uSwellOmega; uniform highp float uShoreTime;\n"
    "uniform highp vec3 uTrainRatio; uniform highp vec3 uTrainAmp; uniform highp vec3 "
    "uTrainPhase;\n"
    "uniform highp vec3 uTrainPhaseR;\n";
const char *const kWaterCommonFuncs =
    // a shore-grid texture at p, bilinear by hand (WebGL2 filters no float texture)
    "vec4 oGrid4(highp sampler2D tex, vec2 p){\n"
    "  ivec2 sz = textureSize(tex, 0);\n"
    "  vec2 f = (p - uShoreGridOrigin.xy) / uShoreGridSpacing.xy;\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i0 = clamp(ivec2(fl), ivec2(0), sz - ivec2(1));\n"
    "  ivec2 i1 = clamp(ivec2(fl) + ivec2(1), ivec2(0), sz - ivec2(1));\n"
    "  vec4 a = mix(texelFetch(tex, i0, 0), texelFetch(tex, ivec2(i1.x, i0.y), 0), fr.x);\n"
    "  vec4 b = mix(texelFetch(tex, ivec2(i0.x, i1.y), 0), texelFetch(tex, i1, 0), fr.x);\n"
    "  return mix(a, b, fr.y);\n"
    "}\n"
    "float oShoreDist(vec2 p){ return oGrid4(uShoreGrid, p).r; }\n"
    // the shore layer at p: (height, sideways x, sideways y, breaking on a crest), the sum
    // cvc::ocean::shore_layer::evaluate does on the CPU
    "vec4 oShoreLayer(vec2 p){\n"
    "  vec4 w = oGrid4(uShoreWave, p);\n"
    "  if (w.y <= 0.0 && w.z <= 0.0) return vec4(0.0);\n"
    "  vec4 d = oGrid4(uShoreDir, p);\n"
    "  vec2 dir = d.xy; float dl = length(dir); if (dl > 1e-6) dir /= dl;\n"
    "  float eta = 0.0, h = 0.0, crest = -1.0;\n"
    "  for (int i = 0; i < 3; ++i) {\n"
    "    float r = uTrainRatio[i];\n"
    "    float th = uSwellOmega*r*uShoreTime - r*w.x + uTrainPhase[i];\n"
    "    float thr = uSwellOmega*r*uShoreTime + r*w.x + uTrainPhaseR[i];\n"
    "    eta += uTrainAmp[i]*(w.y*cos(th) + w.z*cos(thr));\n"
    "    h += uTrainAmp[i]*w.w*w.y*sin(th);\n"
    "    crest = max(crest, cos(th));\n"
    "  }\n"
    "  return vec4(eta, h*dir, d.w*smoothstep(0.55, 1.0, crest));\n"
    "}\n"
    "float oShoreEta(vec2 p){\n"
    "  vec4 w = oGrid4(uShoreWave, p);\n"
    "  if (w.y <= 0.0 && w.z <= 0.0) return 0.0;\n"
    "  float eta = 0.0;\n"
    "  for (int i = 0; i < 3; ++i) {\n"
    "    float r = uTrainRatio[i];\n"
    "    eta += uTrainAmp[i]*(w.y*cos(uSwellOmega*r*uShoreTime - r*w.x + uTrainPhase[i])\n"
    "                       + w.z*cos(uSwellOmega*r*uShoreTime + r*w.x + uTrainPhaseR[i]));\n"
    "  }\n"
    "  return eta;\n"
    "}\n"
    // each cascade's weight at p: 0 on dry ground, rising to 1 as the still water deepens
    "vec3 oDepthW(vec2 p){\n"
    "  float d = uLevel - cvc_hf_height(p);\n"
    "  return vec3(smoothstep(0.0,uFade.x,d), smoothstep(0.0,uFade.y,d), "
    "smoothstep(0.0,uFade.z,d));\n"
    "}\n"
    // a cascade at world w: the texel nearest w (lsystem_coast's oDisp)
    "vec4 wDisp(highp sampler2D t, vec2 w, float tile){ return textureLod(t, w/tile + uHalfTexel, "
    "0.0); }\n"
    "vec4 wSum(vec2 w, vec3 k){\n"
    "  vec4 a=wDisp(uDisp0,w,uTile0), b=wDisp(uDisp1,w,uTile1), c=wDisp(uDisp2,w,uTile2);\n"
    "  vec3 j = vec3(1.0) + k*(vec3(a.w,b.w,c.w) - vec3(1.0));\n" // a faded fold folds less
    "  return vec4(k.x*a.xyz+k.y*b.xyz+k.z*c.xyz, min(min(j.x,j.y),j.z));\n"
    "}\n"
    "float wHt(vec2 w, vec3 k){ return "
    "k.x*wDisp(uDisp0,w,uTile0).y+k.y*wDisp(uDisp1,w,uTile1).y+k.z*wDisp(uDisp2,w,uTile2).y; }\n";

// The wake (lsystem_coast's WAKE_GLSL_ON): an eWave window's state (.r the height h, .g the
// potential phi; cell (i, j) at texel (i, j), its point at uWakeGrid.xy + (i, j) uWakeGrid.z),
// bilinear by hand and faded to 0 over the window's outer 8 cells, inside its sponge.
//   - wkVertex: the height a vertex of a grid of spacing s takes, a [1 2 1] / 4 tent over -s, 0, s,
//     so a grid coarser than the wake's waves does not alias them.
//   - wkSlopeFoam: the slope for the fragment's normal, and foam where a crest is sharp (-laplacian
//     h) or the surface fast (|grad phi| past 0.15 m/s): a dropped crate's splash, a hull's bow.
const char *const kWakeOn =
    "uniform highp sampler2D uWake;\n"
    "uniform highp vec3 uWakeGrid; uniform highp float uWakeN;\n"
    "uniform highp float uWakeOn; uniform highp float uWakeFoam;\n"
    "vec2 wkCell(ivec2 c){\n"
    "  int n = int(uWakeN);\n"
    "  if (c.x < 0 || c.y < 0 || c.x >= n || c.y >= n) return vec2(0.0);\n"
    "  return texelFetch(uWake, c, 0).rg;\n"
    "}\n"
    "vec2 wkAt(vec2 p){\n"
    "  vec2 f = (p - uWakeGrid.xy) / uWakeGrid.z;\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i = ivec2(fl);\n"
    "  vec2 a = mix(wkCell(i), wkCell(i + ivec2(1, 0)), fr.x);\n"
    "  vec2 b = mix(wkCell(i + ivec2(0, 1)), wkCell(i + ivec2(1, 1)), fr.x);\n"
    "  float e = min(min(f.x, uWakeN - 1.0 - f.x), min(f.y, uWakeN - 1.0 - f.y));\n"
    "  return mix(a, b, fr.y) * (uWakeOn * smoothstep(0.0, 8.0, e));\n"
    "}\n"
    "float wkVertex(vec2 p, float s){\n"
    "  float h = 0.0;\n"
    "  for (int j = -1; j <= 1; ++j)\n"
    "    for (int i = -1; i <= 1; ++i) {\n"
    "      float w = (j == 0 ? 2.0 : 1.0) * (i == 0 ? 2.0 : 1.0);\n"
    "      h += w * wkAt(p + s*vec2(float(i), float(j))).x;\n"
    "    }\n"
    "  return h / 16.0;\n"
    "}\n"
    "vec4 wkCellDeriv(ivec2 c){\n"
    "  float d = uWakeGrid.z;\n"
    "  vec2 o = wkCell(c), xp = wkCell(c + ivec2(1, 0)), xm = wkCell(c - ivec2(1, 0));\n"
    "  vec2 yp = wkCell(c + ivec2(0, 1)), ym = wkCell(c - ivec2(0, 1));\n"
    "  return vec4((xp.x - xm.x) / (2.0*d), (yp.x - ym.x) / (2.0*d),\n"
    "              (xp.x + xm.x + yp.x + ym.x - 4.0*o.x) / (d*d),\n"
    "              length(vec2(xp.y - xm.y, yp.y - ym.y)) / (2.0*d));\n"
    "}\n"
    "vec3 wkSlopeFoam(vec2 p){\n"
    "  vec2 f = (p - uWakeGrid.xy) / uWakeGrid.z;\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i = ivec2(fl);\n"
    "  vec4 a = mix(wkCellDeriv(i), wkCellDeriv(i + ivec2(1, 0)), fr.x);\n"
    "  vec4 b = mix(wkCellDeriv(i + ivec2(0, 1)), wkCellDeriv(i + ivec2(1, 1)), fr.x);\n"
    "  float e = min(min(f.x, uWakeN - 1.0 - f.x), min(f.y, uWakeN - 1.0 - f.y));\n"
    "  vec4 q = mix(a, b, fr.y) * (uWakeOn * smoothstep(0.0, 8.0, e));\n"
    "  float foam = uWakeFoam*(0.35*max(-q.z, 0.0) + 2.0*max(q.w - 0.15, 0.0));\n"
    "  return vec3(q.xy, clamp(foam, 0.0, 1.0));\n"
    "}\n";
// No window (no body:, or float targets missing): the same calls, flat.
const char *const kWakeOff = "float wkVertex(vec2 p, float s){ return 0.0; }\n"
                             "vec3 wkSlopeFoam(vec2 p){ return vec3(0.0); }\n";

std::string common_glsl(bool wake) {
  return std::string(kWaterCommonDec) + HeightFieldTexture::glsl() + kWaterCommonFuncs +
         (wake ? kWakeOn : kWakeOff);
}

const char *const kWaterVsDec = "//VTK::PositionVC::Dec\n"
                                "out vec3 vWaterPos;\n"
                                "out vec2 vWaterRest;\n"
                                "out vec3 vWaterN;\n"
                                "out float vWaterJac;\n"
                                "out float vShoreBreak;\n"
                                "uniform highp float uCell;\n"
                                "uniform highp vec3 uCenter;\n";
// The cascades and the swell, each faded by the depth under it and, on the near grid, toward its
// edge; the sideways move capped by the shore distance (sea_surface::displacement on the CPU); the
// wake's height on top.
const char *const kWaterVsImpl =
    "  vec4 dispMC = vertexMC;\n"
    "  vec2 w = vertexMC.xy + uCenter.xy;\n"
    "  float edge = max(abs(vertexMC.x), abs(vertexMC.y)) / max(uNearHalf, 1e-3);\n"
    "  float ke = uWaves * (1.0 - smoothstep(0.75, 1.0, edge));\n"
    "  vec3 k = ke * oDepthW(w);\n"
    "  vec4 od = wSum(w, k);\n"
    // the swell at its true size: uWaveAmp scales the cascades only, so the shore layer's breaking
    // cap holds at any wave height
    "  vec4 osl = ke * oShoreLayer(w);\n"
    "  float hpx = uWaveAmp*wHt(w+vec2(1.0,0.0), k) + ke*oShoreEta(w+vec2(1.0,0.0));\n"
    "  float hmx = uWaveAmp*wHt(w-vec2(1.0,0.0), k) + ke*oShoreEta(w-vec2(1.0,0.0));\n"
    "  float hpy = uWaveAmp*wHt(w+vec2(0.0,1.0), k) + ke*oShoreEta(w+vec2(0.0,1.0));\n"
    "  float hmy = uWaveAmp*wHt(w-vec2(0.0,1.0), k) + ke*oShoreEta(w-vec2(0.0,1.0));\n"
    "  vWaterN = normalize(vec3(-(hpx-hmx)*0.5, -(hpy-hmy)*0.5, 1.0));\n"
    "  vec2 dh = vec2(od.x, od.z) + osl.yz;\n"
    "  float dlim = 0.8*max(oShoreDist(w), 0.0), dlen = length(dh);\n"
    "  if (dlen > dlim) dh *= dlim/max(dlen, 1e-6);\n"
    "  dispMC.xy += dh;\n"
    "  dispMC.z += uWaveAmp*od.y + osl.x + ke*wkVertex(w, uCell);\n"
    "  vertexVCVSOutput = MCVCMatrix * dispMC;\n"
    "  gl_Position = MCDCMatrix * dispMC;\n"
    "  vWaterPos = vec3(dispMC.xy + uCenter.xy, dispMC.z);\n"
    "  vWaterRest = w;\n"
    "  vWaterJac = od.w;\n"
    "  vShoreBreak = osl.w;\n";

const char *const kWaterFsDec =
    "//VTK::Normal::Dec\n"
    "in vec3 vWaterPos;\n"
    "in vec2 vWaterRest;\n"
    "in vec3 vWaterN;\n"
    "in float vWaterJac;\n"
    "in float vShoreBreak;\n"
    "uniform highp vec3 uCamPos; uniform highp vec3 uSunDir;\n"
    "uniform highp float uFoamBias; uniform highp float uFoamRamp; uniform highp float uFoamFray;\n"
    "uniform highp float uShore; uniform highp float uTime;\n"
    "uniform highp vec3 uCutMin; uniform highp vec3 uCutMax;\n"    // the far grid gives way inside
                                                                   // the near one
    "uniform highp vec3 uClipMin; uniform highp vec3 uClipMax;\n"; // no water past the world,
                                                                   // unless its rim is sea
const char *const kWaterFsFuncs =
    "float whash(vec2 p){ return fract(sin(dot(p, vec2(127.1,311.7)))*43758.5453); }\n"
    "float wnoise(vec2 p){\n"
    "  vec2 i=floor(p), f=fract(p); f=f*f*(3.0-2.0*f);\n"
    "  return mix(mix(whash(i),whash(i+vec2(1,0)),f.x),\n"
    "             mix(whash(i+vec2(0,1)),whash(i+vec2(1,1)),f.x), f.y);\n"
    "}\n"
    // The far grid's normal: the long and middle cascades' and the swell's slopes at the fragment,
    // the difference's step growing with distance and the detail fading out (the cascades' texels
    // are read nearest, so a pixel wider than a texel would sparkle): the middle cascade by 3 near,
    // the long one and the swell by 10 near.
    "vec3 wFarNormal(vec2 p, float dist){\n"
    "  float kl = 1.0 - smoothstep(uNearHalf, 10.0*uNearHalf, dist);\n"
    "  float km = 1.0 - smoothstep(uNearHalf, 3.0*uNearHalf, dist);\n"
    "  vec3 k = oDepthW(p) * vec3(kl, km, 0.0);\n"
    "  float e = 1.0 + 0.004*dist;\n"
    "  vec2 ex = vec2(e, 0.0), ey = vec2(0.0, e);\n"
    "  float hx = uWaveAmp*(wHt(p+ex,k) - wHt(p-ex,k)) + kl*(oShoreEta(p+ex) - oShoreEta(p-ex));\n"
    "  float hy = uWaveAmp*(wHt(p+ey,k) - wHt(p-ey,k)) + kl*(oShoreEta(p+ey) - oShoreEta(p-ey));\n"
    "  return normalize(vec3(-hx/(2.0*e), -hy/(2.0*e), 1.0));\n"
    "}\n";
// Consumes //VTK::Light::Impl (VTK's lighting is replaced) and writes the final colour.
// WATER_SUN_SHADE is the share of the sun that reaches the water: 1.0, or, once a cloud_sky lists
// the water in its receivers and has wired its shade in (water_light below), the deck's shade
// there, cloudShade(), which the sky declares at //VTK::Light::Dec.
const char *const kWaterFsLight =
    // The depth at the fragment, from the bed itself: the far grid's cells are metres across.
    "  float sunShade = WATER_SUN_SHADE;\n"
    "  float depthF = uLevel - cvc_hf_height(vWaterRest);\n"
    "  if (depthF < -0.25) discard;\n" // dry ground: the terrain draws there
    "  if (any(lessThan(vWaterRest, uClipMin.xy)) || any(greaterThan(vWaterRest, uClipMax.xy))) "
    "discard;\n"
    "  if (all(greaterThan(vWaterRest, uCutMin.xy)) && all(lessThan(vWaterRest, uCutMax.xy))) "
    "discard;\n"
    "  vec3 N = normalize(vWaterN);\n"
    "  if (uWaves < 0.5) N = wFarNormal(vWaterRest, length(uCamPos.xy - vWaterRest));\n"
    // the wake's slope adds to the sea's (both as -dz/dx, -dz/dy over z of the normal)
    "  vec3 wk = wkSlopeFoam(vWaterRest);\n"
    "  N = normalize(vec3(N.xy/max(N.z, 1e-3) - wk.xy, 1.0));\n"
    "  vec3 Vd = normalize(uCamPos - vWaterPos);\n"
    "  float NdotV = max(dot(N,Vd), 0.0);\n"
    "  float Fr = 0.02 + 0.98*pow(1.0-NdotV, 5.0);\n"
    "  float NdotL = max(dot(N, uSunDir), 0.0);\n"
    "  float lit = 0.30 + 0.70*NdotL*sunShade;\n"
    "  vec3 Rr = 2.0*NdotV*N - Vd;\n"
    "  float skyT = clamp(0.5+0.5*Rr.z, 0.0, 1.0);\n"
    "  float glint = pow(max(dot(Rr, uSunDir), 0.0), 200.0)*1.8*sunShade;\n"
    "  vec3 deep = vec3(0.05,0.22,0.38)*lit;\n"
    "  vec3 sky = mix(vec3(0.66,0.72,0.78), vec3(0.26,0.50,0.80), skyT);\n"
    "  vec3 col = mix(deep, sky, Fr) + glint*vec3(1.0,0.96,0.88);\n"
    // Shallow water is translucent: the bed shows through, fading with the depth under the surface
    // (Beer-Lambert), so the shallows brighten and darken as the waves pass.
    "  float depthS = vWaterPos.z - cvc_hf_height(vWaterPos.xy);\n"
    "  float clarity = exp(-max(depthS,0.0)*0.20);\n"
    "  vec3 shoal = mix(vec3(0.12,0.46,0.54), vec3(0.66,0.64,0.50), clarity*0.55);\n"
    "  col = mix(col, shoal, clarity*0.85);\n"
    // Whitecaps where the cascades fold (cvc/ocean/whitecaps.h has the threshold's meaning).
    "  float fray = 0.65*wnoise(vWaterRest*1.9) + 0.35*wnoise(vWaterRest*4.3 + vec2(17.0, 31.0));\n"
    "  float foam = clamp((uFoamBias - vWaterJac + uFoamFray*(fray - 0.5))/uFoamRamp, 0.0, 1.0);\n"
    "  foam = foam*foam*(3.0-2.0*foam);\n"
    // A noisy band at the waterline, washing up and down with the waves: faint on calm water, full
    // where the swell reaches; and the swell's breakers on their crests.
    "  float nz = wnoise(vWaterPos.xy*0.4 + vec2(uTime*0.6, uTime*0.35));\n"
    "  float surf = clamp(0.25 + oGrid4(uShoreWave, vWaterRest).y/0.25, 0.0, 1.0);\n"
    "  float band = 1.0 - smoothstep(0.0, 1.8, abs(depthS));\n"
    "  foam = max(foam, clamp(band*(0.55+0.75*nz)*step(-0.4, depthS)*surf*uShore, 0.0, 1.0));\n"
    "  foam = max(foam, clamp(vShoreBreak*(0.7+0.6*nz)*uShore, 0.0, 1.0));\n"
    // the wake's: a splash, a bow wave
    "  foam = max(foam, wk.z*(0.75+0.5*nz));\n"
    "  col = mix(col, vec3(0.93,0.96,0.97), foam);\n"
    "  gl_FragData[0] = vec4(col, 1.0);\n";

// The water's lighting: in a cloud_sky's shade (`shaded`: the sky has wired its inputs into the
// mesh, so cloudShade() is declared) or in full sun. In the shade it also takes the weather the sky
// carries (O18, O19, O20): the ambient drops under a storm (cloudAmbient), lightning lights it
// (cloudFlashLit), raindrop rings tilt its normal (cloudRainNormal, procedural, no texture) and the
// fog thickens in the rain (cloudFog).
// Each goes where the lighting has the line it follows, and is left out if that line is not there.
std::string water_light(bool shaded) {
  std::string s = kWaterFsLight;
  const std::string mark = "WATER_SUN_SHADE";
  s.replace(s.find(mark), mark.size(), shaded ? "cloudShade(vertexVC.xyz)" : "1.0");
  if (shaded) {
    auto swap = [&s](const std::string &from, const std::string &to) {
      if (const std::size_t at = s.find(from); at != std::string::npos)
        s.replace(at, from.size(), to);
    };
    const std::string normal = "  N = normalize(vec3(N.xy/max(N.z, 1e-3) - wk.xy, 1.0));\n";
    swap(normal, normal + "  N = cloudRainNormal(vWaterRest, N);\n");
    swap("float lit = 0.30 + 0.70*NdotL*sunShade;",
         "float lit = 0.30*cloudAmbient(vertexVC.xyz) + 0.70*NdotL*sunShade + "
         "cloudFlashLit(vertexVC.xyz);");
    swap("gl_FragData[0] = vec4(col, 1.0);",
         "gl_FragData[0] = vec4(cloudFog(col, vertexVC.xyz), 1.0);");
  }
  return s;
}

// The cloud_sky's shared shader inputs' key on a receiver (lsystem_nodes.cpp's CLOUD_INPUTS).
constexpr const char *kCloudInputs = "cloud_sky";

// ── the swash on the beach (lsystem_coast's, OCEAN-AND-WEATHER-ROADMAP.md A3, decision D4) ─────
// Drawn in the ground's fragment shader, not as geometry: no z-fighting with the sand, on every
// LOD rung. Each wave's front runs up the sand to Hunt's runup (the shore layer's runup field,
// carried up the beach from the waterline) and drains back; under it a thin film of water with
// foam at its edge; behind it the sand stays dark and glossy, drying over the layer's dry time. The
// arithmetic is cvc::ocean::shore_layer::swash's, so the physics sea knows where it is wet.
//   - It reads the shore layer's phase and runup from a texture of the ground's own (binding the
//     water's two in the ground's programs as well was not needed, so it does not).
//   - The fragment's world position comes from its view position (uSwashVCWC, view -> world, as
//     the cloud deck's shade finds it), so the ground's own coordinates (shifted or not) do not
//     matter.
//   - It runs at //VTK::Picking::Impl, after VTK has applied the ground's texture (albedo x light).
//     At //VTK::TCoord::Impl, VTK no longer applied the texture.
//   - Its inputs stay out of the shadow maps' bake (ShaderInputs::skip_shadow_bake): there the
//     fragment's colour is the occluder's depth, and the swash would rewrite it. uSwashOn reads 0
//     in the bake's program.
const char *const kSwashDec =
    "//VTK::CustomUniforms::Dec\n"
    "uniform highp sampler2D uSwash;\n" // (phase, runup) on the shore layer's grid
    "uniform vec3 uSwashOrigin; uniform vec3 uSwashSpacing;\n"
    "uniform float uSwashOmega; uniform float uSwashTime; uniform float uSwashPhase0;\n"
    "uniform float uSwashDry; uniform float uSwashLevel; uniform float uSwashFoam;\n"
    "uniform float uSwashOn; uniform mat4 uSwashVCWC;\n"
    "vec2 swGrid(vec2 p){\n"
    "  ivec2 sz = textureSize(uSwash, 0);\n"
    "  vec2 f = (p - uSwashOrigin.xy) / uSwashSpacing.xy;\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i0 = clamp(ivec2(fl), ivec2(0), sz - ivec2(1));\n"
    "  ivec2 i1 = clamp(ivec2(fl) + ivec2(1), ivec2(0), sz - ivec2(1));\n"
    "  vec2 a = mix(texelFetch(uSwash, i0, 0).rg, texelFetch(uSwash, ivec2(i1.x, i0.y), 0).rg, "
    "fr.x);\n"
    "  vec2 b = mix(texelFetch(uSwash, ivec2(i0.x, i1.y), 0).rg, texelFetch(uSwash, i1, 0).rg, "
    "fr.x);\n"
    "  return mix(a, b, fr.y);\n"
    "}\n"
    // (film, lace, wet) of the ground at world p
    "vec3 oSwash(vec3 p){\n"
    "  float z = p.z - uSwashLevel;\n"
    "  if (z <= 0.0) return vec3(0.0, 0.0, 1.0);\n"
    "  vec2 s = swGrid(p.xy);\n"
    "  float R = s.y;\n"
    "  if (R <= 0.0 || z >= R) return vec3(0.0);\n"
    "  float th = uSwashOmega*uSwashTime - s.x + uSwashPhase0;\n"
    "  float front = R*(0.5 + 0.5*cos(th));\n"
    "  float alpha = acos(clamp(2.0*z/R - 1.0, -1.0, 1.0));\n"
    "  float w = th - 6.2831853*floor(th/6.2831853 + 0.5);\n"
    "  if (abs(w) <= alpha) {\n"
    "    float film = clamp((front - z)/max(0.15*R, 0.05), 0.0, 1.0);\n"
    "    float lace = 1.0 - smoothstep(0.0, max(0.12*R, 0.03), front - z);\n"
    "    return vec3(film, lace, 1.0);\n"
    "  }\n"
    "  float since = w - alpha; if (since < 0.0) since += 6.2831853;\n"
    "  return vec3(0.0, 0.0, exp(-since/uSwashOmega/max(uSwashDry, 1e-3)));\n"
    "}\n";
const char *const kSwashImpl =
    "//VTK::Picking::Impl\n"
    "  if (uSwashOn > 0.5) {\n"
    "    vec3 sw = oSwash((uSwashVCWC * vec4(vertexVC.xyz, 1.0)).xyz);\n"
    "    vec3 c = gl_FragData[0].rgb;\n"
    "    c *= mix(1.0, 0.62, sw.z);\n"                     // wet sand is darker
    "    c = mix(c, vec3(0.70, 0.78, 0.86), 0.22*sw.z);\n" // and glossier
    "    c = mix(c, vec3(0.42, 0.58, 0.64), 0.5*sw.x);\n"  // the film's water
    "    c = mix(c, vec3(0.95, 0.97, 0.98), clamp(0.9*sw.y*uSwashFoam, 0.0, 1.0));\n" // its edge
    "    gl_FragData[0].rgb = c;\n"
    "  }\n";

// The swash's shared shader inputs' key on a ground mesh.
constexpr const char *kSwashInputs = "water_swash";

// ── meshes ───────────────────────────────────────────────────────────────────────────────────

// A flat cells x cells-vertex grid at z, from (x0, y0) to (x1, y1).
cvc::geometry flat_grid(cvc::app &app, int cells, double x0, double y0, double x1, double y1,
                        double z) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  pts.reserve(std::size_t(cells) * std::size_t(cells));
  for (int j = 0; j < cells; ++j) {
    const double y = y0 + (y1 - y0) * j / (cells - 1);
    for (int i = 0; i < cells; ++i) {
      pts.push_back({x0 + (x1 - x0) * i / (cells - 1), y, z});
      cols.push_back({0.05, 0.20, 0.32}); // the flat fallback's colour
    }
  }
  auto &tris = g.tris();
  using idx = cvc::geometry::index_t;
  for (int j = 0; j < cells - 1; ++j)
    for (int i = 0; i < cells - 1; ++i) {
      const idx v = idx(j * cells + i);
      tris.push_back({v, idx(v + 1), idx(v + cells)});
      tris.push_back({idx(v + 1), idx(v + cells + 1), idx(v + cells)});
    }
  return g;
}

// ── runtime ──────────────────────────────────────────────────────────────────────────────────

struct Cascade {
  std::unique_ptr<OceanFFT> fft;
  cvc::ocean::cascade band;
  cvc::ocean::h0_field h0; // what fft was given (the whitecaps' threshold is found from it)
};

struct WaterRT {
  ari::SceneNode node; // its props and keys: (scene_water_spec)
  std::string prefix;
  std::weak_ptr<GeometryNode> near, far;
  std::shared_ptr<const cvc::height_field> ground;
  std::unique_ptr<HeightFieldTexture> bed;
  const float *bedData = nullptr; // the ground's heights the bed was made from (regenerates)
  std::size_t bedSize = 0;
  std::vector<Cascade> cascades;
  std::vector<double> lastSea; // the wind keys h0 was built for
  bool ready = false, failed = false;
  double level = 0.0, nearHalf = 192.0, cell = 2.0;
  double cx = 0.0, cy = 0.0; // the near grid's centre (whole cells)
  bool placed = false;
  // Whether each mesh draws in a cloud_sky's shade (the sky wires receivers on its ticks).
  bool nearShaded = false, farShaded = false;
  // The world's edge: when the ground's whole rim is under water (a generated_world's `edge:
  // shore`, an island), the sea goes on `outer` metres past it; otherwise the water stops at the
  // ground's edge (a wall, or the world falling away). Found again after a regenerate.
  double outer = 6000.0;
  int farCells = 160;
  bool rimSea = false, rimKnown = false;
  std::array<double, 4> farExtent{0.0, 0.0, 0.0, 0.0}; // the far grid's x0, y0, x1, y1
  // The whitecaps' threshold for Monahan's share (cvc/ocean/whitecaps.h), found again once the
  // keys it depends on have held for kSettle frames.
  double foamBias = 0.8;
  std::vector<double> foamSig, foamPending;
  int foamSettled = 0;
  // The coast (scene_water.h): the physics runtime's when there is one, else this node's own,
  // built again when the ground or (once the keys have held for kSettle frames) the swell changes.
  std::shared_ptr<const ari::SceneWaterShore> shore, shoreDrawn;
  std::vector<double> shoreSig, shorePending;
  int shoreSettled = 0;
  const float *shoreGround = nullptr;
  vtkSmartPointer<vtkTextureObject> shoreTex, waveTex, dirTex, swashTex;
  // the drawing-only keys' resolved paths
  std::string biasKey, rampKey, frayKey, capsKey, shoreFoamKey, wakeKey, wakeFoamKey;
  // The swash on the ground node's meshes (`swash:`, on by default): their shared inputs, the
  // scene graph's tree version they were last wired at, and the node that draws the ground.
  bool swash = true;
  SceneGraph *sg = nullptr;
  std::string groundId;
  std::shared_ptr<GeometryNode::ShaderInputs> swashInputs;
  std::uint64_t swashTree = ~std::uint64_t(0);
  // With `body:`, the water in the scene's physics world (scene_water.h): one fluid field over the
  // ground, its surface the sea drawn here, rebuilt as the wind or the ground changes.
  std::unique_ptr<cvc::ariadne::WaterRuntime> physics;
  // The bodies' waves (with body:, once the cascades are up): the window, its sources, where it is
  // and the coast its mask was made from, the world time it was stepped to, and each body's outline
  // by the description it was made from.
  std::unique_ptr<EWave> wake;
  std::unique_ptr<WakeSource> wakeSrc;
  bool wakePlaced = false;
  const ari::SceneWaterShore *wakeShore = nullptr;
  double wakeT = -1.0;
  std::vector<double> wakeDh;
  std::map<const cvc::physics::body_desc *,
           std::pair<std::shared_ptr<const cvc::physics::body_desc>,
                     std::shared_ptr<const cvc::physics::immersion_body>>>
      outlines;
};

constexpr int kSettle = 10;

double key(cvc::app &app, const std::string &path, double def) {
  return ari::read_or<double>(app, path, def);
}

std::string seed(cvc::app &app, const std::string &prefix, const ari::SceneNode &n, const char *k,
                 double def) {
  const std::string path = ari::resolve_bind(prefix, ari::node_key(n, k));
  ari::read_or_seed<double>(app, path, def);
  return path;
}

ari::SceneWaterSpec spec_of(cvc::app &app, const WaterRT &rt) {
  return ari::scene_water_spec(app, rt.node, rt.prefix);
}

// The cascades' wind sea (the swell is the shore layer's).
cvc::ocean::sea_state wind_sea(const ari::SceneWaterSpec &s) {
  cvc::ocean::sea_state sea;
  sea.wind.speed_mps = s.wind_speed;
  sea.wind.fetch_m = s.fetch;
  sea.wind.dir_rad = s.wind_dir * 3.14159265358979323846 / 180.0;
  sea.swell.hs_m = 0.0;
  return sea;
}

// The renderer's first switched-on scene light, as the direction toward it.
std::array<double, 3> sun_direction(vtkRenderer *ren) {
  std::array<double, 3> d{kDefaultSun[0], kDefaultSun[1], kDefaultSun[2]};
  if (!ren || !ren->GetLights())
    return d;
  vtkLightCollection *lights = ren->GetLights();
  lights->InitTraversal();
  while (vtkLight *l = lights->GetNextItem()) {
    if (!l->GetSwitch() || !l->LightTypeIsSceneLight() || l->GetPositional())
      continue;
    double p[3], f[3];
    l->GetPosition(p);
    l->GetFocalPoint(f);
    const double v[3] = {p[0] - f[0], p[1] - f[1], p[2] - f[2]};
    const double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 1e-9)
      return {v[0] / len, v[1] / len, v[2] / len};
  }
  return d;
}

void style_flat(GeometryNode &g) {
  g.setUseSingleColor(true);
  g.setColor(0.05, 0.20, 0.32);
  g.setAmbient(0.6);
  g.setDiffuse(0.5);
  g.setSpecular(0.5);
  g.setSpecularPower(60.0);
}

// A float texture of `comps` channels, nearest and clamped (WebGL2 filters no float texture).
vtkSmartPointer<vtkTextureObject> float_texture(vtkOpenGLRenderWindow *ow, int w, int h, int comps,
                                                const float *data) {
  auto tex = vtkSmartPointer<vtkTextureObject>::New();
  tex->SetContext(ow);
  if (!tex->Create2DFromRaw(w, h, comps, VTK_FLOAT, const_cast<float *>(data)))
    return nullptr;
  tex->SetMinificationFilter(vtkTextureObject::Nearest);
  tex->SetMagnificationFilter(vtkTextureObject::Nearest);
  tex->SetWrapS(vtkTextureObject::ClampToEdge);
  tex->SetWrapT(vtkTextureObject::ClampToEdge);
  return tex;
}

// The coast to the GPU: the shore distance and the shore layer's two planes and trains (a texel of
// nothing without a swell), on both meshes. False when a texture failed.
bool upload_shore(WaterRT &rt, vtkOpenGLRenderWindow *ow, GeometryNode &near, GeometryNode &far) {
  const ari::SceneWaterShore *sh = rt.shore.get();
  const float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  const float farShore[4] = {1.0e6f, 0.0f, 0.0f, 0.0f}; // no coast: no cap on the sideways move
  const cvc::ocean::shore_grid *g = sh ? sh->grid.get() : nullptr;
  rt.shoreTex = g ? float_texture(ow, g->cols, g->rows, 1, g->shore_m.data())
                  : float_texture(ow, 1, 1, 1, farShore);
  const cvc::ocean::shore_layer *L = sh ? sh->layer.get() : nullptr;
  rt.swashTex = nullptr;
  if (L && !L->empty()) {
    const std::vector<float> wave = L->wave_texels(), dir = L->dir_texels();
    rt.waveTex = float_texture(ow, L->cols, L->rows, 4, wave.data());
    rt.dirTex = float_texture(ow, L->cols, L->rows, 4, dir.data());
    // The ground's own texture for the swash, (phase, runup): bound in the ground's programs
    // only. Binding the water's two in them as well left the ground without its colour texture.
    if (rt.swash) {
      std::vector<float> sw(L->phase.size() * 2);
      for (std::size_t i = 0; i < L->phase.size(); ++i) {
        sw[i * 2] = L->phase[i];
        sw[i * 2 + 1] = L->runup[i];
      }
      rt.swashTex = float_texture(ow, L->cols, L->rows, 2, sw.data());
    }
  } else {
    rt.waveTex = float_texture(ow, 1, 1, 4, zero);
    rt.dirTex = float_texture(ow, 1, 1, 4, zero);
  }
  if (!rt.shoreTex || !rt.waveTex || !rt.dirTex)
    return false;
  float v[4][3] = {};
  if (L)
    for (std::size_t i = 0; i < L->trains.size() && i < 3; ++i) {
      v[0][i] = static_cast<float>(L->trains[i].ratio);
      v[1][i] = static_cast<float>(L->trains[i].amplitude);
      v[2][i] = static_cast<float>(L->trains[i].phase);
      v[3][i] = static_cast<float>(L->trains[i].phase_reflected);
    }
  const char *names[4] = {"uTrainRatio", "uTrainAmp", "uTrainPhase", "uTrainPhaseR"};
  const float ox = g ? static_cast<float>(g->min_x) : 0.0f,
              oy = g ? static_cast<float>(g->min_y) : 0.0f;
  const float cs = g ? static_cast<float>(g->cell) : 1.0f;
  for (GeometryNode *m : {&near, &far}) {
    m->setShaderTexture("uShoreGrid", rt.shoreTex);
    m->setShaderTexture("uShoreWave", rt.waveTex);
    m->setShaderTexture("uShoreDir", rt.dirTex);
    m->setShaderUniform3f("uShoreGridOrigin", ox, oy, 0.0f);
    m->setShaderUniform3f("uShoreGridSpacing", cs, cs, 0.0f);
    m->setShaderUniformf("uSwellOmega", L ? static_cast<float>(L->omega) : 0.0f);
    for (int j = 0; j < 4; ++j)
      m->setShaderUniform3f(names[j], v[j][0], v[j][1], v[j][2]);
  }
  rt.shoreDrawn = rt.shore;
  return true;
}

// The shaders, the cascades' textures and the bed on one of the water's meshes.
void style_ocean(GeometryNode &g, const WaterRT &rt, vtkTextureObject *bed) {
  const char *disp[3] = {"uDisp0", "uDisp1", "uDisp2"};
  const char *tile[3] = {"uTile0", "uTile1", "uTile2"};
  for (std::size_t i = 0; i < rt.cascades.size() && i < 3; ++i) {
    g.setShaderTexture(disp[i], rt.cascades[i].fft->displacement());
    g.setShaderUniformf(tile[i], static_cast<float>(rt.cascades[i].band.tile_m));
  }
  g.setShaderUniformf("uHalfTexel", 0.5f / static_cast<float>(kFftN));
  g.setShaderUniformf("uLevel", static_cast<float>(rt.level));
  g.setShaderUniformf("uNearHalf", static_cast<float>(rt.nearHalf));
  g.setShaderUniformf("uCell", static_cast<float>(rt.cell));
  g.setShaderTexture(HeightFieldTexture::samplerName(), bed);
  g.setShaderUniform3f(HeightFieldTexture::originName(), static_cast<float>(rt.bed->x0()),
                       static_cast<float>(rt.bed->y0()), 0.0f);
  g.setShaderUniform3f(HeightFieldTexture::spacingName(), static_cast<float>(rt.bed->dx()),
                       static_cast<float>(rt.bed->dy()), 0.0f);
  const std::string common = common_glsl(rt.wake != nullptr);
  g.addVertexShaderReplacement("//VTK::PositionVC::Dec", std::string(kWaterVsDec) + common);
  g.addVertexShaderReplacement("//VTK::PositionVC::Impl", kWaterVsImpl);
  g.addFragmentShaderReplacement("//VTK::Normal::Dec",
                                 std::string(kWaterFsDec) + common + kWaterFsFuncs);
  g.addFragmentShaderReplacement("//VTK::Light::Impl",
                                 water_light(g.hasSharedShaderInputs(kCloudInputs)));
  if (rt.wake) {
    g.setShaderTexture("uWake", rt.wake->heightTexture());
    g.setShaderUniformf("uWakeN", static_cast<float>(rt.wake->params().n));
    g.setShaderUniformf("uWakeOn", 0.0f); // until the window is placed
  }
}

// Make the bed texture's heights the ground's (again after a regenerate); true when they changed.
bool refresh_bed(WaterRT &rt) {
  const cvc::height_field &hf = *rt.ground;
  const std::vector<float> &h = hf.heights();
  if (h.data() == rt.bedData && h.size() == rt.bedSize && rt.bed)
    return false;
  if (!rt.bed || rt.bed->nx() != hf.cols() || rt.bed->ny() != hf.rows() ||
      rt.bed->x0() != hf.x0() || rt.bed->y0() != hf.y0() || rt.bed->dx() != hf.dx() ||
      rt.bed->dy() != hf.dy())
    rt.bed = std::make_unique<HeightFieldTexture>(hf.cols(), hf.rows(), hf.x0(), hf.y0(), hf.dx(),
                                                  hf.dy());
  rt.bed->setHeights(h);
  rt.bedData = h.data();
  rt.bedSize = h.size();
  return true;
}

// The far grid over the ground, and `outer` metres past it when the rim is sea; the water clipped
// to the ground's rectangle otherwise. When the rim changed (or the first time).
void place_far(WaterRT &rt, GeometryNode &far, GeometryNode &near) {
  const cvc::height_field &hf = *rt.ground;
  const bool sea = ari::water_rim_is_sea(hf, rt.level);
  const double m = sea ? rt.outer : 0.0;
  const double x0 = hf.x0() - m, y0 = hf.y0() - m, x1 = hf.max_x() + m, y1 = hf.max_y() + m;
  const std::array<double, 4> extent{x0, y0, x1, y1};
  if (rt.rimKnown && sea == rt.rimSea && extent == rt.farExtent)
    return;
  const bool first = !rt.rimKnown;
  rt.rimSea = sea;
  rt.rimKnown = true;
  rt.farExtent = extent;
  if (!first) {
    const int n = rt.farCells;
    std::vector<double> xyz;
    xyz.reserve(std::size_t(n) * std::size_t(n) * 3);
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        xyz.push_back(x0 + (x1 - x0) * i / (n - 1));
        xyz.push_back(y0 + (y1 - y0) * j / (n - 1));
        xyz.push_back(rt.level);
      }
    far.updateVertices(xyz);
  }
  const float big = 1e30f;
  for (GeometryNode *g : {&far, &near}) {
    if (sea) {
      g->setShaderUniform3f("uClipMin", -big, -big, 0.0f);
      g->setShaderUniform3f("uClipMax", big, big, 0.0f);
    } else {
      g->setShaderUniform3f("uClipMin", static_cast<float>(hf.x0()), static_cast<float>(hf.y0()),
                            0.0f);
      g->setShaderUniform3f("uClipMax", static_cast<float>(hf.max_x()),
                            static_cast<float>(hf.max_y()), 0.0f);
    }
  }
}

// The cascades and their textures, once the window is up; false when float targets are missing.
bool start_ocean(cvc::app &app, WaterRT &rt, vtkOpenGLRenderWindow *ow) {
  const ari::SceneWaterSpec spec = spec_of(app, rt);
  const cvc::ocean::sea_state s = wind_sea(spec);
  const ari::SceneWaterCascades cd = ari::scene_water_cascades(spec);
  for (const cvc::ocean::cascade &b : cvc::ocean::make_cascades(cd.tiles, cd.seed)) {
    Cascade c;
    c.band = b;
    c.fft = std::make_unique<OceanFFT>(kFftN);
    c.fft->depth_m = 1000.0f;
    c.fft->chop = static_cast<float>(spec.chop);
    if (!c.fft->init(ow))
      return false;
    c.h0 = cvc::ocean::build_h0(kFftN, c.band, s);
    c.fft->setH0(c.h0);
    rt.cascades.push_back(std::move(c));
  }
  rt.lastSea = {s.wind.speed_mps, s.wind.fetch_m, s.wind.dir_rad};
  return true;
}

// What the whitecaps' threshold depends on: the wind (through h0), the chop, the ramp, the fray
// and the share of Monahan's cover asked for.
std::vector<double> foam_signature(cvc::app &app, const WaterRT &rt) {
  const ari::SceneWaterSpec s = spec_of(app, rt);
  return {s.wind_speed,
          s.fetch,
          s.wind_dir,
          s.chop,
          key(app, rt.rampKey, 0.1),
          key(app, rt.frayKey, 0.12),
          key(app, rt.capsKey, 1.0)};
}

// The threshold that makes the open water's foam cover Monahan's share for the wind (times
// water.whitecaps), on the near grid's vertices (cvc/ocean/whitecaps.h; about 0.1-0.25 s).
double find_foam_bias(cvc::app &app, const WaterRT &rt) {
  std::vector<cvc::ocean::h0_field> hs;
  for (const Cascade &c : rt.cascades)
    hs.push_back(c.h0);
  const ari::SceneWaterSpec s = spec_of(app, rt);
  cvc::ocean::foam_params p;
  p.chop = s.chop;
  p.ramp = std::max(key(app, rt.rampKey, 0.1), 1e-3);
  p.fray = std::max(key(app, rt.frayKey, 0.12), 0.0);
  p.mesh_m = rt.cell;
  const double share =
      std::max(key(app, rt.capsKey, 1.0), 0.0) * cvc::ocean::monahan_coverage(s.wind_speed);
  return cvc::ocean::foam_field(hs, p).bias_for(share);
}

// The coast this node draws: the physics runtime's, or its own, built again when the ground
// changed or the swell's keys have held for kSettle frames after a change.
void follow_shore(cvc::app &app, WaterRT &rt) {
  if (rt.physics) {
    rt.shore = rt.physics->shore();
    return;
  }
  const ari::SceneWaterSpec s = spec_of(app, rt);
  const std::vector<double> sig = ari::scene_water_shore_signature(s);
  const bool groundNew = rt.ground->heights().data() != rt.shoreGround;
  bool build = !rt.shore || groundNew;
  if (!build && sig != rt.shoreSig) {
    if (sig != rt.shorePending) {
      rt.shorePending = sig;
      rt.shoreSettled = 0;
    } else if (++rt.shoreSettled >= kSettle) {
      build = true;
    }
  }
  if (!build)
    return;
  rt.shore = ari::scene_water_shore(*rt.ground, s);
  rt.shoreSig = rt.shorePending = sig;
  rt.shoreGround = rt.ground->heights().data();
}

// ── the wake ─────────────────────────────────────────────────────────────────────────────────

// A plane of the shore grid at (x, y), bilinear, clamped to the grid (sea_surface's grid_sample).
double grid_sample(const cvc::ocean::shore_grid &g, const std::vector<float> &f, double x,
                   double y) {
  const double fc = std::clamp((x - g.min_x) / g.cell, 0.0, double(g.cols - 1));
  const double fr = std::clamp((y - g.min_y) / g.cell, 0.0, double(g.rows - 1));
  const int c0 = std::min(g.cols - 2, int(std::floor(fc))),
            r0 = std::min(g.rows - 2, int(std::floor(fr)));
  const double u = fc - c0, v = fr - r0;
  const std::size_t i = std::size_t(r0) * std::size_t(g.cols) + std::size_t(c0);
  return (f[i] * (1 - u) + f[i + 1] * u) * (1 - v) +
         (f[i + std::size_t(g.cols)] * (1 - u) + f[i + std::size_t(g.cols) + 1] * u) * v;
}

// The window's mask (lsystem_coast's seaWakeMask, OCEAN-AND-WEATHER-ROADMAP.md C6): 0 on land; in
// the water a ramp over the last 3 m to the shore from 1 down to 0.5 at the waterline, so beaches
// absorb; where the shore is steep (the shore layer's reflection, or the ground's slope from 0.25
// to 0.6), 1 up to the waterline, so it reflects. And the window's mean water depth.
std::vector<double> wake_mask(const ari::SceneWaterShore &sh, const cvc::ocean::ewave_params &p,
                              double &meanDepth) {
  const cvc::ocean::shore_grid &g = *sh.grid;
  const cvc::ocean::shore_layer *L = sh.layer.get();
  const double W = 3.0, mb = 0.5;
  std::vector<double> m(std::size_t(p.n) * p.n, 0.0);
  double dsum = 0.0;
  int dn = 0;
  for (int j = 0; j < p.n; ++j)
    for (int i = 0; i < p.n; ++i) {
      const double x = p.origin_x + i * p.dx, y = p.origin_y + j * p.dx;
      const double s = grid_sample(g, g.shore_m, x, y);
      if (!(s > 0.0))
        continue; // land
      const double kr = L && !L->empty() ? L->sample_field(L->reflection, x, y) : 0.0;
      const double slope = sh.slope.size() == g.count() ? grid_sample(g, sh.slope, x, y) : 0.0;
      const double ss = std::clamp((slope - 0.25) / 0.35, 0.0, 1.0);
      const double steep =
          std::min(1.0, std::max(std::isfinite(kr) ? kr : 0.0, ss * ss * (3.0 - 2.0 * ss)));
      const double r = std::min(1.0, s / W);
      const double beach = mb + (1.0 - mb) * r * r * (3.0 - 2.0 * r);
      m[std::size_t(j) * p.n + i] = beach + (1.0 - beach) * steep;
      const double d = grid_sample(g, g.depth, x, y);
      if (d > 0.0)
        dsum += d, ++dn;
    }
  meanDepth = dn ? dsum / dn : 5.0;
  return m;
}

// Move the window to the target (tx, ty) when it is 16 cells from its centre (by whole cells, its
// sponge hiding the strip that enters), and give it its new place's mask and depth; the mask again
// when the coast changed.
void place_wake(WaterRT &rt, double tx, double ty) {
  const cvc::ocean::ewave_params &p = rt.wake->params();
  const double half = 0.5 * p.n * p.dx;
  const long wi = std::lround((tx - half) / p.dx), wj = std::lround((ty - half) / p.dx);
  const long ci = std::lround(p.origin_x / p.dx), cj = std::lround(p.origin_y / p.dx);
  const long di = wi - ci, dj = wj - cj;
  const bool move = !rt.wakePlaced || std::labs(di) >= 16 || std::labs(dj) >= 16;
  if (!move && rt.wakeShore == rt.shore.get())
    return;
  if (move) {
    const int si = int(std::clamp(di, -100000L, 100000L));
    const int sj = int(std::clamp(dj, -100000L, 100000L));
    rt.wake->shift(si, sj);
    rt.wakeSrc->shift(si, sj);
    rt.wakePlaced = true;
  }
  rt.wakeShore = rt.shore.get();
  if (rt.shore && rt.shore->grid) {
    double depth = 5.0;
    rt.wake->setMask(wake_mask(*rt.shore, rt.wake->params(), depth));
    rt.wake->setDepth(std::max(0.5, depth));
  }
}

// A body's outline as the water sees it, made once per description (null for one that cannot be
// immersed: a mesh or a terrain).
const cvc::physics::immersion_body *
outline_of(WaterRT &rt, const std::shared_ptr<const cvc::physics::body_desc> &d) {
  auto it = rt.outlines.find(d.get());
  if (it == rt.outlines.end()) {
    std::shared_ptr<const cvc::physics::immersion_body> o;
    try {
      o = std::make_shared<const cvc::physics::immersion_body>(
          cvc::physics::immersion_body::of(*d));
    } catch (const std::exception &) {
    }
    it = rt.outlines.emplace(d.get(), std::make_pair(d, o)).first;
  }
  return it->second.second.get();
}

// One frame of the wake: place the window at the camera's target, add the moving bodies' sources
// (each body where it is drawn, in the water physics feels at the time of that pose), step it by
// the world time since the last frame, and bind it.
void tick_wake(cvc::app &app, WaterRT &rt, vtkRenderer *ren, RealizedScene &scene,
               const cvc::world_clock::step_result &clock) {
  if (!rt.ready || !rt.wake || !rt.physics || !ren)
    return;
  auto near = rt.near.lock();
  auto far = rt.far.lock();
  if (!near || !far)
    return;
  const bool on = key(app, rt.wakeKey, 1.0) > 0.5;
  // Decision D3: the camera's target, the point the view is about; the camera's own place when the
  // target is off past the near grid (a view toward the horizon).
  double tx = rt.cx, ty = rt.cy;
  if (vtkCamera *c = ren->GetActiveCamera()) {
    double f[3], e[3];
    c->GetFocalPoint(f);
    c->GetPosition(e);
    const bool close = std::hypot(f[0] - e[0], f[1] - e[1]) < 0.5 * rt.nearHalf;
    tx = close ? f[0] : e[0];
    ty = close ? f[1] : e[1];
  }
  if (on)
    place_wake(rt, tx, ty);
  const double wt = app.world_clock().t();
  const double dt = rt.wakeT < 0.0 ? 0.0 : wt - rt.wakeT;
  if (on && (rt.wakeT < 0.0 || dt > 0.0)) {
    rt.wakeSrc->begin();
    const std::shared_ptr<const cvc::physics::field> field = rt.physics->field();
    if (scene.world && scene.physics_binding && field && field->volume) {
      cvc::physics::pose_tracker &tracker = scene.physics_binding->tracker();
      const cvc::physics::world &w = scene.world->world();
      // The view lags the world by a step (pose_tracker.h): a body is drawn a of the way from tick
      // k - 1 to k, so the sea it is cut against is read at the field time of that point.
      const double a = tracker.alpha(app.world_clock(), clock.alpha);
      const double fdt = w.config().fixed_dt;
      const double t0 = w.field_time() - double(w.tick()) * fdt;
      const double drawnT = t0 + std::max(0.0, double(tracker.tick()) - 1.0 + a) * fdt;
      const cvc::physics::immersion_water water = cvc::physics::immersion_water::of(*field, drawnT);
      for (const ari::SceneWorldBody &b : scene.world->bodies()) {
        if (!b.moves || !b.desc)
          continue;
        const cvc::physics::immersion_body *o = outline_of(rt, b.desc);
        if (!o)
          continue;
        std::optional<cvc::physics::pose> pose = tracker.sample_one(b.id, a);
        if (!pose) {
          if (!w.contains(b.id))
            continue;
          const cvc::physics::body_state st = w.state(b.id);
          pose = cvc::physics::pose{st.position, st.rotation};
        }
        rt.wakeSrc->add(*o, pose->position, pose->rotation, water);
      }
    }
    if (rt.wakeSrc->finish(rt.wakeDh))
      rt.wake->addHeights(rt.wakeDh);
    if (dt > 0.0)
      rt.wake->step(dt);
    rt.wakeT = wt;
  } else if (!on && rt.wakeT >= 0.0) {
    rt.wakeT = -1.0; // start again from here when it is turned back on
    rt.wakeSrc->reset();
  }
  const cvc::ocean::ewave_params &p = rt.wake->params();
  const float foam = static_cast<float>(std::max(key(app, rt.wakeFoamKey, 0.35), 0.0));
  for (GeometryNode *g : {near.get(), far.get()}) {
    g->setShaderTexture("uWake", rt.wake->heightTexture());
    g->setShaderUniform3f("uWakeGrid", static_cast<float>(p.origin_x),
                          static_cast<float>(p.origin_y), static_cast<float>(p.dx));
    g->setShaderUniformf("uWakeN", static_cast<float>(p.n));
    g->setShaderUniformf("uWakeOn", on && rt.wakePlaced ? 1.0f : 0.0f);
    g->setShaderUniformf("uWakeFoam", foam);
  }
}

// ── the swash's frame ────────────────────────────────────────────────────────────────────────

// Give `g` and every mesh under it the swash, once each.
void wire_swash(WaterRT &rt, GraphicsNode &g) {
  if (auto *m = dynamic_cast<GeometryNode *>(&g); m && !m->hasSharedShaderInputs(kSwashInputs)) {
    m->addFragmentShaderReplacement("//VTK::CustomUniforms::Dec", kSwashDec);
    m->addFragmentShaderReplacement("//VTK::Picking::Impl", kSwashImpl);
    m->setSharedShaderInputs(kSwashInputs, rt.swashInputs);
  }
  for (const std::shared_ptr<GraphicsNode> &c : g.getGraphicsChildren())
    if (c)
      wire_swash(rt, *c);
}

// The swash's inputs for this frame at world time t, and the ground's meshes wired: the first time
// there is a swell, then whenever the scene's tree changes (a regenerated world's chunks, an LOD
// rung made late), so a new mesh takes it on the frame after it joins.
void tick_swash(cvc::app &app, WaterRT &rt, vtkRenderer *ren, double t) {
  if (!rt.swash || !rt.sg)
    return;
  const ari::SceneWaterShore *sh = rt.shore.get();
  const cvc::ocean::shore_layer *L = sh ? sh->layer.get() : nullptr;
  const bool on = L && !L->empty() && rt.swashTex;
  if (!on && !rt.swashInputs)
    return; // never a swell: the ground is left alone
  if (!rt.swashInputs) {
    rt.swashInputs = std::make_shared<GeometryNode::ShaderInputs>();
    rt.swashInputs->skip_shadow_bake = true; // it recolours after VTK: not in the bake
  }
  GeometryNode::ShaderInputs &in = *rt.swashInputs;
  in.f["uSwashOn"] = on ? 1.0f : 0.0f;
  if (on) {
    in.tex["uSwash"] = rt.swashTex;
    in.f3["uSwashOrigin"] = {float(L->min_x), float(L->min_y), 0.0f};
    in.f3["uSwashSpacing"] = {float(L->cell), float(L->cell), 0.0f};
    in.f["uSwashOmega"] = float(L->omega);
    in.f["uSwashPhase0"] = L->trains.empty() ? 0.0f : float(L->trains.front().phase);
    in.f["uSwashDry"] = float(L->dry_time_s);
  }
  in.f["uSwashTime"] = float(t);
  in.f["uSwashLevel"] = float(rt.level);
  in.f["uSwashFoam"] = float(std::max(key(app, rt.shoreFoamKey, 1.0), 0.0));
  if (vtkCamera *cam = ren ? ren->GetActiveCamera() : nullptr) {
    vtkNew<vtkMatrix4x4> vcwc;
    vtkMatrix4x4::Invert(cam->GetModelViewTransformMatrix(), vcwc);
    std::array<double, 16> &m4 = in.m4["uSwashVCWC"];
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        m4[i * 4 + j] = vcwc->GetElement(i, j);
  }
  if (const std::uint64_t tv = rt.sg->treeVersion(); on && tv != rt.swashTree) {
    rt.swashTree = tv;
    if (auto g = rt.sg->getGraphics(rt.groundId))
      wire_swash(rt, *g);
  }
}

// ── the frame ────────────────────────────────────────────────────────────────────────────────

void tick_water(cvc::app &app, WaterRT &rt, vtkRenderer *ren) {
  auto near = rt.near.lock();
  auto far = rt.far.lock();
  if (!near || !far)
    return;
  if (rt.physics)
    rt.physics->tick(); // the field follows the keys and the ground (a regenerate)
  if (!rt.ready && !rt.failed) {
    auto *ow = ren ? vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow()) : nullptr;
    if (!ow || !ow->GetInitialized())
      return;
    refresh_bed(rt);
    vtkTextureObject *bed = rt.bed ? rt.bed->prepare(ow) : nullptr;
    if (!bed || !start_ocean(app, rt, ow)) {
      rt.failed = true; // no float render targets: the far grid's flat blue stays, alone
      rt.cascades.clear();
      return;
    }
    // The bodies' waves, with body: (the window needs float targets too).
    if (rt.physics) {
      cvc::ocean::ewave_params wp;
      wp.n = 256;
      wp.dx = 0.5;
      wp.depth = 5.0;       // until the window is placed: then its wet cells' mean depth
      wp.damping_k2 = 2e-3; // m^2/s: waves 1 m long fade over about 13 s
      wp.damping = 0.02;    // 1/s: everything, over a minute
      wp.sponge_cells = 16.0;
      wp.sponge_rate = 2.0;
      wp.beach_rate = 10.0;
      rt.wake = std::make_unique<EWave>(wp);
      if (rt.wake->init(ow))
        rt.wakeSrc = std::make_unique<WakeSource>(rt.wake->params());
      else
        rt.wake.reset();
    }
    style_ocean(*near, rt, bed);
    style_ocean(*far, rt, bed);
    follow_shore(app, rt);
    if (!upload_shore(rt, ow, *near, *far)) {
      rt.failed = true;
      rt.cascades.clear();
      rt.wake.reset();
      rt.wakeSrc.reset();
      return;
    }
    far->setShaderUniform3f("uCenter", 0.0f, 0.0f, 0.0f);
    far->setShaderUniformf("uWaves", 0.0f); // flat: its cells are metres across
    near->setShaderUniformf("uWaves", 1.0f);
    near->setShaderUniform3f("uCutMin", 1e30f, 1e30f, 0.0f); // the near grid gives way to none
    near->setShaderUniform3f("uCutMax", -1e30f, -1e30f, 0.0f);
    near->setVisible(true);
    far->setVisible(true);
    rt.foamBias = find_foam_bias(app, rt);
    rt.foamSig = rt.foamPending = foam_signature(app, rt);
    rt.ready = true;
  }
  if (!rt.ready)
    return;
  auto *ow = vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow());

  // A cloud_sky that lists the water in its receivers wires its shade into each mesh on one of its
  // ticks: from the next frame that mesh's sun is the deck's shade (O17).
  for (auto [g, shaded] :
       {std::pair{near.get(), &rt.nearShaded}, std::pair{far.get(), &rt.farShaded}}) {
    const bool s = g->hasSharedShaderInputs(kCloudInputs);
    if (s != *shaded) {
      *shaded = s;
      g->addFragmentShaderReplacement("//VTK::Light::Impl", water_light(s));
    }
  }

  // A regenerated world: the bed again.
  if (refresh_bed(rt)) {
    if (vtkTextureObject *bed = ow ? rt.bed->prepare(ow) : nullptr)
      for (GeometryNode *g : {near.get(), far.get()}) {
        g->setShaderTexture(HeightFieldTexture::samplerName(), bed);
        g->setShaderUniform3f(HeightFieldTexture::originName(), static_cast<float>(rt.bed->x0()),
                              static_cast<float>(rt.bed->y0()), 0.0f);
        g->setShaderUniform3f(HeightFieldTexture::spacingName(), static_cast<float>(rt.bed->dx()),
                              static_cast<float>(rt.bed->dy()), 0.0f);
      }
    place_far(rt, *far, *near); // the edge may have changed (world.edge), or the world's size
  }
  // The coast: again after a regenerate or a swell change.
  follow_shore(app, rt);
  if (rt.shore != rt.shoreDrawn && ow)
    upload_shore(rt, ow, *near, *far);

  // The wind: h0 again when it changed; the cascades to the world's time.
  const ari::SceneWaterSpec spec = spec_of(app, rt);
  const cvc::ocean::sea_state s = wind_sea(spec);
  const std::vector<double> sea{s.wind.speed_mps, s.wind.fetch_m, s.wind.dir_rad};
  const bool rebuild = sea != rt.lastSea;
  const float chop = static_cast<float>(spec.chop);
  const double t = app.world_clock().t();
  for (Cascade &c : rt.cascades) {
    if (rebuild) {
      c.h0 = cvc::ocean::build_h0(kFftN, c.band, s);
      c.fft->setH0(c.h0);
    }
    c.fft->chop = chop;
    c.fft->step(t);
  }
  if (rebuild)
    rt.lastSea = sea;
  tick_swash(app, rt, ren, t); // the swash on the ground, at the same time as the swell
  // The whitecaps' threshold, once its keys have settled (a slider drag does not stall each frame).
  std::vector<double> foamNow = foam_signature(app, rt);
  if (foamNow != rt.foamSig) {
    if (foamNow != rt.foamPending) {
      rt.foamPending = std::move(foamNow);
      rt.foamSettled = 0;
    } else if (++rt.foamSettled >= kSettle) {
      rt.foamBias = find_foam_bias(app, rt);
      rt.foamSig = rt.foamPending;
    }
  }

  // The near grid follows the camera by whole cells.
  double cam[3] = {0.0, 0.0, 0.0};
  if (vtkCamera *c = ren->GetActiveCamera())
    c->GetPosition(cam);
  const double qx = std::round(cam[0] / rt.cell) * rt.cell,
               qy = std::round(cam[1] / rt.cell) * rt.cell;
  if (!rt.placed || std::abs(qx - rt.cx) > 0.25 * rt.nearHalf ||
      std::abs(qy - rt.cy) > 0.25 * rt.nearHalf) {
    rt.cx = qx;
    rt.cy = qy;
    rt.placed = true;
    near->setPosition(rt.cx, rt.cy, 0.0);
    near->setShaderUniform3f("uCenter", static_cast<float>(rt.cx), static_cast<float>(rt.cy), 0.0f);
    // the far grid gives way just inside the near grid's edge, where its waves have faded out
    const float h = static_cast<float>(0.97 * rt.nearHalf);
    far->setShaderUniform3f("uCutMin", static_cast<float>(rt.cx) - h, static_cast<float>(rt.cy) - h,
                            0.0f);
    far->setShaderUniform3f("uCutMax", static_cast<float>(rt.cx) + h, static_cast<float>(rt.cy) + h,
                            0.0f);
  }

  const std::array<double, 3> sun = sun_direction(ren);
  const float amp = static_cast<float>(spec.wave_amp);
  const double fixedBias = key(app, rt.biasKey, -1.0); // >= 0 fixes the threshold by hand
  const float bias = static_cast<float>(fixedBias >= 0.0 ? fixedBias : rt.foamBias);
  const float ramp = static_cast<float>(std::max(key(app, rt.rampKey, 0.1), 1e-3));
  const float fray = static_cast<float>(std::max(key(app, rt.frayKey, 0.12), 0.0));
  const float shoreFoam = static_cast<float>(std::max(key(app, rt.shoreFoamKey, 1.0), 0.0));
  const ari::SceneWaterCascades cd = ari::scene_water_cascades(spec);
  for (GeometryNode *g : {near.get(), far.get()}) {
    g->setShaderUniform3f("uCamPos", static_cast<float>(cam[0]), static_cast<float>(cam[1]),
                          static_cast<float>(cam[2]));
    g->setShaderUniform3f("uSunDir", static_cast<float>(sun[0]), static_cast<float>(sun[1]),
                          static_cast<float>(sun[2]));
    g->setShaderUniformf("uWaveAmp", amp);
    g->setShaderUniformf("uFoamBias", bias);
    g->setShaderUniformf("uFoamRamp", ramp);
    g->setShaderUniformf("uFoamFray", fray);
    g->setShaderUniform3f("uFade", static_cast<float>(cd.fade_m[0]),
                          static_cast<float>(cd.fade_m[1]), static_cast<float>(cd.fade_m[2]));
    g->setShaderUniformf("uShore", shoreFoam);
    g->setShaderUniformf("uShoreTime", static_cast<float>(t));
    g->setShaderUniformf("uTime", static_cast<float>(t));
  }
}

std::shared_ptr<GraphicsNode> realize_water(SceneGraph &sg, const ari::SceneNode &n,
                                            GraphicsNode *parent, RealizedScene &out,
                                            std::vector<std::string> *warnings) {
  std::shared_ptr<GraphicsNode> group = parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
  group->setContributesToSceneBounds(false); // the ground frames the view (see the meshes below)
  const std::string world = n.props.str("world");
  const std::string groundId = n.props.str("ground").empty() ? world : n.props.str("ground");
  auto hf = out.height_fields.find(groundId);
  if (groundId.empty() || hf == out.height_fields.end() || !hf->second || hf->second->rows() < 2 ||
      hf->second->cols() < 2) {
    if (warnings)
      warnings->push_back("ari: water '" + n.id + "': no height field '" + groundId +
                          "' (world: or ground: names a generated_world or height-field node)");
    return group;
  }
  cvc::app &app = sg.appContext();
  auto rt = std::make_shared<WaterRT>();
  rt->node = n;
  rt->prefix = sg.getStatePrefix();
  rt->ground = hf->second;
  rt->level = n.props.num("level", 0.0);
  rt->nearHalf = std::max(n.props.num("near", 192.0), 16.0);
  rt->swash = n.props.num("swash", 1.0) != 0.0; // the swash on the ground node's meshes
  rt->sg = &sg;
  rt->groundId = groundId;
  const int cells = std::clamp(static_cast<int>(n.props.num("cells", 192.0)), 16, 512);
  rt->cell = 2.0 * rt->nearHalf / (cells - 1);

  // The sea's keys (wind, chop, wave height, shore fade, swell) are seeded by scene_water_spec;
  // these are the drawing's.
  const std::string &pref = rt->prefix;
  spec_of(app, *rt);
  rt->biasKey = seed(app, pref, n, "water.foam_bias", n.props.num("foam_bias", -1.0));
  rt->capsKey = seed(app, pref, n, "water.whitecaps", n.props.num("whitecaps", 1.0));
  rt->rampKey = seed(app, pref, n, "water.foam_ramp", n.props.num("foam_ramp", 0.1));
  rt->frayKey = seed(app, pref, n, "water.foam_fray", n.props.num("foam_fray", 0.12));
  rt->shoreFoamKey = seed(app, pref, n, "water.shore_foam", n.props.num("shore_foam", 1.0));
  rt->wakeKey = seed(app, pref, n, "water.wake", n.props.num("wake", 1.0));
  rt->wakeFoamKey = seed(app, pref, n, "water.wake_foam", n.props.num("wake_foam", 0.35));

  const cvc::height_field &g = *rt->ground;
  rt->outer = std::max(n.props.num("outer", 6000.0), 0.0);
  rt->farCells = std::clamp(static_cast<int>(n.props.num("far_cells", 160.0)), 2, 1024);
  const double m = ari::water_rim_is_sea(g, rt->level) ? rt->outer : 0.0;
  auto far = std::dynamic_pointer_cast<GeometryNode>(
      group->createChild<GeometryNode>("far", flat_grid(app, rt->farCells, g.x0() - m, g.y0() - m,
                                                        g.max_x() + m, g.max_y() + m, rt->level)));
  auto near = std::dynamic_pointer_cast<GeometryNode>(
      group->createChild<GeometryNode>("near", flat_grid(app, cells, -rt->nearHalf, -rt->nearHalf,
                                                         rt->nearHalf, rt->nearHalf, rt->level)));
  if (!far || !near) {
    if (warnings)
      warnings->push_back("ari: water '" + n.id + "': could not create its meshes");
    return group;
  }
  for (GeometryNode *m : {far.get(), near.get()}) {
    style_flat(*m);
    m->setCastsShadow(false);
    m->setForceOpaque(true);
    // The ground frames the view, not the water: the sea past a shore-edged world runs `outer`
    // metres out, and the near grid moves with the camera. They stay in VTK's bounds, so the
    // clipping range holds them.
    m->setContributesToSceneBounds(false);
    // vertexMC in the shaders is the mesh's own coordinates (world xy for the far grid), from the
    // first upload on: set before the mesh ever draws.
    m->disableCoordinateShiftScale();
  }
  near->setVisible(false); // shown once the cascades are up (or the flat fallback is chosen)
  rt->near = near;
  rt->far = far;
  place_far(*rt, *far, *near); // the clip for the edge the grid was just built for

  // `body:`: the water's field in the scene's physics world, over the same ground (whose shared
  // height field a regenerated world overwrites in place: the runtime sees its new heights).
  if (n.body.present) {
    std::shared_ptr<const cvc::height_field> ground = rt->ground;
    rt->physics =
        std::make_unique<cvc::ariadne::WaterRuntime>(app, n, pref, [ground]() { return ground; });
    out.colliders[n.id] = rt->physics->colliders();
  }

  cvc::app *ap = &app;
  out.custom_ticks.push_back([ap, rt](vtkRenderer *ren) { tick_water(*ap, *rt, ren); });
  if (rt->physics) // the bodies' waves read the scene's world, as the frame left it
    out.scene_ticks.push_back([ap, rt](vtkRenderer *ren, RealizedScene &scene,
                                       const cvc::world_clock::step_result &clock) {
      tick_wake(*ap, *rt, ren, scene, clock);
    });
  return group;
}

} // namespace

void register_water_node_types(cvc::app &app) {
  register_scene_node_type(app, "water", realize_water);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
