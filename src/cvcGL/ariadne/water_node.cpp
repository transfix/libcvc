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
//   - near: a grid `near` metres each way from the camera, displaced by three FFT cascades
//     (OceanCascades: all three in one set of passes, the wind sea's h0 from
//     cvc::ocean::build_h0) and the swell's shore layer, the waves fading toward the grid's edge.
//     It is three rings: a vertex every 2 near / (cells - 1) out to a third of the way, every
//     twice that to two thirds, every four times that past (ring_coords). It moves with the camera
//     by the coarsest spacing, so the waves do not swim; the short cascade fades out by
//     `lod_waves` metres from the camera.
//   - far: a flat grid over the ground's extent with the same shading, its normal the waves' per
//     fragment (the two long cascades and the swell, fading with distance), for the water past the
//     near grid; it gives way inside the near grid.
// The coast (scene_water.h: a shore grid over the ground, cvc::world::water_bodies) shapes both, as
// it shapes the physics sea: each cascade fades out as the still water shoals (half its band's peak
// wavelength, at most `shore_fade`), a sideways move stays under 0.8 of the way to the shore, and
// with a swell (`swell_height` > 0) the shore layer refracts, shoals and breaks it on the beaches
// that face it (cvc::ocean::shore_layer, the sum the CPU evaluates).
// Both grids shade with the A4 water (OceanFoam.h's oWaterFull: a GGX sun on the roughness the
// waves the mesh cannot carry give it, Fresnel, light through backlit crests, shallows tinted by
// depth), giving way past `lod_shade` metres to lsystem_coast's Fresnel water; with Jacobian
// whitecaps with a frayed edge, the long waves' persistent foam carved into windrows (OceanFoam,
// near the camera, within `lod_foam` metres; the sea's mean cover past it and on the far grid),
// the swell's breakers and a foam band at the waterline that the swell strengthens. The sun is the
// renderer's first scene light. Dry ground (above the level) discards its fragments. A
// generated_world under a water node wants `water: 0`, so its flat water rungs do not draw over
// this.
//
// With a swell, the swash runs up the beach on the ground itself: the node adds it to the fragment
// shader of every mesh under its ground node (the `world:` / `ground:` node, found by id at the top
// of the scene), as the cloud deck adds its shade to its receivers. Each wave's front runs up to
// the shore layer's runup and drains back, a film of water with a lace of foam under it, the sand
// dark and glossy behind it until it dries. The physics sea carries the same swash. `swash: 0`
// leaves it out of both.
//
// With `body:` the water is in the scene's physics world (scene_water.h), and the bodies floating
// in it make waves: an eWave window (EWave, 128 m of 0.5 m cells; 64 m at adaptive quality's tiers
// 1 and 2) around the camera's target takes each moving body's immersed columns as its source
// (WakeSource, cvc::physics::add_immersion), steps by the world's time, and adds its height to the
// near grid's vertices and its slope and foam to both grids' fragments. Beaches absorb its waves
// and steep shores reflect them.
//
// Live keys (water.*, or the node's `keys:`; the water_controls.ari panel): wind_speed (m/s),
// wind_dir (degrees, the way the waves travel, from +x toward +y), fetch (m), chop (the cascades'
// sideways moves), wave_amp (a height scale), foam_bias / foam_ramp / foam_fray (the whitecaps'
// Jacobian threshold, its ramp and the noise on its edge), whitecaps (a share of Monahan's cover),
// foam_decay (s: how long the persistent foam lasts; 0 none), shore_fade (m), swell_height (m),
// swell_period (s), swell_dir (degrees), shore_foam (the waterline's and the breakers' foam), wake
// (0 / 1), wake_foam, lod_foam, lod_shade and lod_waves (m: where the foam's detail, the full
// shading and the short cascade end), quality (-1: the scene's adaptive quality tier, else 0 .. 4),
// profile (1: time the node's work into water.stats.*; 2: and wait for the GPU in each part); and
// the wind's (O13): gustiness, lee (0 / 1), sea_lag (s), sea_fade (s) and current (m/s). A change
// to the swell rebuilds the coast's shore layer (once the keys have held for a few frames); and to
// what the foam's thresholds depend on, the thresholds, off the frame (OceanFoamCalibration).
//
// The wind (O13, scene_water.h). The sea lags it: a keyframe of its lagged wind builds the
// cascades' h0 (tens of milliseconds) and, when its direction moved, the coarse fetch map; the old
// sea crossfades into the new over sea_fade seconds, the two h0 mixed on the CPU and uploaded each
// frame only while the crossfade runs. The lee scales each cascade by its fetch (the two
// keyframes' maps, packed into the coast atlas beside its planes, so they take no sampler of their
// own); the gusts move the ripple cascade's weight and roughen the water (cat's paws: a ruffled
// normal near the camera, a darker patch everywhere); the current carries the two short cascades,
// and the middle one's persistent foam with them, through a flow map. The gusts and the current
// are hashed noise in the vertex shader (no texture), per vertex of each grid, their fine detail
// faded out with distance; the physics sea (WaterRuntime) is the same sea at the same clock time.
//
// Adaptive quality (cvc/ariadne/scene_quality.h, O21): the scene's quality.tier and quality.scale
// (AriRuntime's loop on the frame time), or `quality`, pick how far the shading, the persistent
// foam, the short cascade and the far grid's waves reach, the wake's window and whether the
// persistent foam, the wake and the short cascade run at all. Drawing only: the physics sea is the
// same at every tier.
//
// Samplers (WebGL2 gives a draw 16): the vertex stage reads the cascades' atlas, the coast atlas
// (with the wind's fetch maps in it) and the wake; the fragment stage those and the foam atlas
// (and a cloud_sky's two shade maps).
//
// The cascades and the wake need float render targets (WebGL2's EXT_color_buffer_float). Without
// them, or before the window is up, the water draws as flat blue.

#include "EWave.h"
#include "OceanCascades.h"
#include "OceanFoam.h"
#include "OceanWind.h"
#include "WakeSource.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_boat.h>            // the boats the water keeps out of
#include <cvc/ariadne/scene_generated_world.h> // SceneColliders
#include <cvc/ariadne/scene_quality.h>         // adaptive quality's tiers (O21)
#include <cvc/ariadne/scene_water.h>           // its physics (WaterRuntime) and coast
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
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
    // the cascades' displacement atlas (OceanCascades: tile c cascade c, n texels a side)
    "uniform highp sampler2D uDisp;\n"
    "uniform highp float uTile0; uniform highp float uTile1; uniform highp float uTile2;\n"
    "uniform highp float uHalfTexel; uniform highp float uWaveAmp; uniform highp float uLevel;\n"
    "uniform highp vec3 uFade; uniform highp float uWaves; uniform highp float uNearHalf;\n"
    "uniform highp vec3 uCamPos;\n"
    // The coast atlas, on the shore grid: its planes side by side, uCoastN columns and uCoastRows
    // rows each. Plane 0 holds (the signed distance to the shore, + offshore; the still water's
    // depth, the level minus the bed, <= 0 on dry ground); with a swell (uSwellOn) plane 1 the
    // shore layer's (phase, amplitude, reflected, steepness) and plane 2 its (direction x, y,
    // wavenumber, breaking). Past the planes, on their own grid (uWindFetchGrid: origin x, y,
    // cell), the wind's fetch maps (O13; OceanWind::fetch's texels): (the old keyframe's, the new
    // one's) from column uCoastFetch.x, uCoastFetch.y columns and uCoastFetch.z rows. One sampler
    // for the bed, the shore, the swell and the lee (a WebGL2 draw has 16).
    "uniform highp sampler2D uCoast; uniform highp float uCoastN; uniform highp float uSwellOn;\n"
    "uniform highp float uCoastRows; uniform highp vec3 uCoastFetch;\n"
    "uniform highp vec3 uWindFetchGrid;\n"
    "uniform highp vec3 uShoreGridOrigin; uniform highp vec3 uShoreGridSpacing;\n"
    "uniform highp float uSwellOmega; uniform highp float uShoreTime;\n"
    "uniform highp vec3 uTrainRatio; uniform highp vec3 uTrainAmp; uniform highp vec3 "
    "uTrainPhase;\n"
    "uniform highp vec3 uTrainPhaseR;\n";
// OceanWind's fetch read, from the coast atlas (OceanWind::glsl(true) leaves it to its caller):
// the old and the new keyframe's fetch at p (m), bilinear by hand, clamped to the map. The same
// texels and the same arithmetic as OceanWind's own owFetch on its uWindFetch texture.
const char *const kWaterFetchFuncs =
    "vec2 owFetch(vec2 p){\n"
    "  ivec2 sz = ivec2(int(uCoastFetch.y + 0.5), int(uCoastFetch.z + 0.5));\n"
    "  int o = int(uCoastFetch.x + 0.5);\n"
    "  vec2 f = (p - uWindFetchGrid.xy) / uWindFetchGrid.z;\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i0 = clamp(ivec2(fl), ivec2(0), sz - ivec2(1));\n"
    "  ivec2 i1 = clamp(ivec2(fl) + ivec2(1), ivec2(0), sz - ivec2(1));\n"
    "  vec2 a = mix(texelFetch(uCoast, ivec2(o + i0.x, i0.y), 0).rg,\n"
    "               texelFetch(uCoast, ivec2(o + i1.x, i0.y), 0).rg, fr.x);\n"
    "  vec2 b = mix(texelFetch(uCoast, ivec2(o + i0.x, i1.y), 0).rg,\n"
    "               texelFetch(uCoast, ivec2(o + i1.x, i1.y), 0).rg, fr.x);\n"
    "  return mix(a, b, fr.y);\n"
    "}\n";
const char *const kWaterCommonFuncs =
    // a plane of the coast atlas at p, bilinear by hand (WebGL2 filters no float texture),
    // clamped to the grid
    "vec4 oCoast(int plane, vec2 p){\n"
    "  ivec2 sz = ivec2(int(uCoastN + 0.5), int(uCoastRows + 0.5));\n"
    "  vec2 f = (p - uShoreGridOrigin.xy) / uShoreGridSpacing.xy;\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i0 = clamp(ivec2(fl), ivec2(0), sz - ivec2(1));\n"
    "  ivec2 i1 = clamp(ivec2(fl) + ivec2(1), ivec2(0), sz - ivec2(1));\n"
    "  int o = plane*sz.x;\n"
    "  vec4 a = mix(texelFetch(uCoast, ivec2(o + i0.x, i0.y), 0),\n"
    "               texelFetch(uCoast, ivec2(o + i1.x, i0.y), 0), fr.x);\n"
    "  vec4 b = mix(texelFetch(uCoast, ivec2(o + i0.x, i1.y), 0),\n"
    "               texelFetch(uCoast, ivec2(o + i1.x, i1.y), 0), fr.x);\n"
    "  return mix(a, b, fr.y);\n"
    "}\n"
    "float oShoreDist(vec2 p){ return oCoast(0, p).r; }\n"
    "float oDepth(vec2 p){ return oCoast(0, p).g; }\n" // the level minus the bed
    // the shore layer at p: (height, sideways x, sideways y, breaking on a crest), the sum
    // cvc::ocean::shore_layer::evaluate does on the CPU
    "vec4 oShoreLayer(vec2 p){\n"
    "  if (uSwellOn < 0.5) return vec4(0.0);\n"
    "  vec4 w = oCoast(1, p);\n"
    "  if (w.y <= 0.0 && w.z <= 0.0) return vec4(0.0);\n"
    "  vec4 d = oCoast(2, p);\n"
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
    "  if (uSwellOn < 0.5) return 0.0;\n"
    "  vec4 w = oCoast(1, p);\n"
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
    "vec3 depthW(float d){\n"
    "  return vec3(smoothstep(0.0,uFade.x,d), smoothstep(0.0,uFade.y,d), "
    "smoothstep(0.0,uFade.z,d));\n"
    "}\n"
    "vec3 oDepthW(vec2 p){ return depthW(oDepth(p)); }\n"
    // cascade c at world w: the texel nearest w (lsystem_coast's oDisp), wrapped into its tile of
    // the atlas
    "vec4 wTexel(int c, vec2 w, float tile){\n"
    "  float fn = 0.5/uHalfTexel;\n"
    "  vec2 f = floor(w*(fn/tile) + 0.5);\n"
    "  f -= fn*floor(f/fn);\n"
    "  return texelFetch(uDisp, ivec2(int(f.x) + c*int(fn), int(f.y)), 0);\n"
    "}\n"
    // ... read through the current's flow map when the current carries the cascade (adv):
    // OceanWind's owDisp on the atlas, owFlow the current there. Two reads half a period apart,
    // blended so the variance holds.
    "vec4 wDisp(int c, vec2 w, float tile, float adv){\n"
    "  if (adv < 0.5 || uCurrent.x <= 0.0) return wTexel(c, w, tile);\n"
    "  vec2 q0 = w - owFlow*uFlowTau.x + uFlowOff0.xy*tile;\n"
    "  vec2 q1 = w - owFlow*uFlowTau.y + uFlowOff1.xy*tile;\n"
    "  const vec4 one = vec4(0.0, 0.0, 0.0, 1.0);\n" // the Jacobian blends about 1
    "  return uFlowW.x*(wTexel(c, q0, tile) - one) + uFlowW.y*(wTexel(c, q1, tile) - one) + one;\n"
    "}\n"
    // a cascade whose weight is 0 (faded by depth, by distance, or the far grid) is not read
    "vec4 wSum(vec2 w, vec3 k){\n"
    "  vec4 a = k.x > 0.0 ? wDisp(0,w,uTile0,uAdvect.x) : vec4(0.0, 0.0, 0.0, 1.0);\n"
    "  vec4 b = k.y > 0.0 ? wDisp(1,w,uTile1,uAdvect.y) : vec4(0.0, 0.0, 0.0, 1.0);\n"
    "  vec4 c = k.z > 0.0 ? wDisp(2,w,uTile2,uAdvect.z) : vec4(0.0, 0.0, 0.0, 1.0);\n"
    "  vec3 j = vec3(1.0) + k*(vec3(a.w,b.w,c.w) - vec3(1.0));\n" // a faded fold folds less
    "  return vec4(k.x*a.xyz+k.y*b.xyz+k.z*c.xyz, min(min(j.x,j.y),j.z));\n"
    "}\n"
    "float wHt(vec2 w, vec3 k){\n"
    "  float h = 0.0;\n"
    "  if (k.x > 0.0) h += k.x*wDisp(0,w,uTile0,uAdvect.x).y;\n"
    "  if (k.y > 0.0) h += k.y*wDisp(1,w,uTile1,uAdvect.y).y;\n"
    "  if (k.z > 0.0) h += k.z*wDisp(2,w,uTile2,uAdvect.z).y;\n"
    "  return h;\n"
    "}\n";

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
    "  vec2 g = (p - uWakeGrid.xy) / uWakeGrid.z;\n" // outside the window: no taps
    "  if (uWakeOn < 0.5 || any(lessThan(g, vec2(-2.0*s/uWakeGrid.z)))\n"
    "      || any(greaterThan(g, vec2(uWakeN + 2.0*s/uWakeGrid.z)))) return 0.0;\n"
    "  float h = 0.0;\n"
    "  for (int j = -1; j <= 1; ++j)\n"
    "    for (int i = -1; i <= 1; ++i) {\n"
    "      float w = (j == 0 ? 2.0 : 1.0) * (i == 0 ? 2.0 : 1.0);\n"
    "      h += w * wkAt(p + s*vec2(float(i), float(j))).x;\n"
    "    }\n"
    "  return h / 16.0;\n"
    "}\n"
    // a cell's (dh/dx, dh/dy, laplacian h, |grad phi|) from it and its four neighbours
    "vec4 wkDeriv(vec2 o, vec2 xp, vec2 xm, vec2 yp, vec2 ym){\n"
    "  float d = uWakeGrid.z;\n"
    "  return vec4((xp.x - xm.x) / (2.0*d), (yp.x - ym.x) / (2.0*d),\n"
    "              (xp.x + xm.x + yp.x + ym.x - 4.0*o.x) / (d*d),\n"
    "              length(vec2(xp.y - xm.y, yp.y - ym.y)) / (2.0*d));\n"
    "}\n"
    "vec3 wkSlopeFoam(vec2 p){\n"
    "  vec2 f = (p - uWakeGrid.xy) / uWakeGrid.z;\n"
    // off, or outside the window (where its edge's fade is 0): none of the reads
    "  if (uWakeOn < 0.5 || any(lessThan(f, vec2(0.0))) || any(greaterThan(f, vec2(uWakeN - "
    "1.0))))\n"
    "    return vec3(0.0);\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  ivec2 i = ivec2(fl);\n"
    // the four corners' stencils share 12 cells: each read once
    "  vec2 c00 = wkCell(i), c10 = wkCell(i + ivec2(1, 0));\n"
    "  vec2 c01 = wkCell(i + ivec2(0, 1)), c11 = wkCell(i + ivec2(1, 1));\n"
    "  vec2 cm0 = wkCell(i + ivec2(-1, 0)), cm1 = wkCell(i + ivec2(-1, 1));\n"
    "  vec2 c20 = wkCell(i + ivec2(2, 0)), c21 = wkCell(i + ivec2(2, 1));\n"
    "  vec2 c0m = wkCell(i + ivec2(0, -1)), c1m = wkCell(i + ivec2(1, -1));\n"
    "  vec2 c02 = wkCell(i + ivec2(0, 2)), c12 = wkCell(i + ivec2(1, 2));\n"
    "  vec4 a = mix(wkDeriv(c00, c10, cm0, c01, c0m), wkDeriv(c10, c20, c00, c11, c1m), fr.x);\n"
    "  vec4 b = mix(wkDeriv(c01, c11, cm1, c02, c00), wkDeriv(c11, c21, c01, c12, c10), fr.x);\n"
    "  float e = min(min(f.x, uWakeN - 1.0 - f.x), min(f.y, uWakeN - 1.0 - f.y));\n"
    "  vec4 q = mix(a, b, fr.y) * (uWakeOn * smoothstep(0.0, 8.0, e));\n"
    "  float foam = uWakeFoam*(0.35*max(-q.z, 0.0) + 2.0*max(q.w - 0.15, 0.0));\n"
    "  return vec3(q.xy, clamp(foam, 0.0, 1.0));\n"
    "}\n";
// No window (no body:, or float targets missing): the same calls, flat.
const char *const kWakeOff = "float wkVertex(vec2 p, float s){ return 0.0; }\n"
                             "vec3 wkSlopeFoam(vec2 p){ return vec3(0.0); }\n";

std::string common_glsl(bool wake) {
  return std::string(kWaterCommonDec) + kWaterFetchFuncs + OceanWind::glsl(true) +
         kWaterCommonFuncs + (wake ? kWakeOn : kWakeOff);
}

const char *const kWaterVsDec = "//VTK::PositionVC::Dec\n"
                                "out vec3 vWaterPos;\n"
                                "out vec2 vWaterRest;\n"
                                "out vec3 vWaterN;\n"
                                "out float vWaterJac;\n"
                                "out float vShoreBreak;\n"
                                "out vec3 vWaterK;\n" // each cascade's weight (depth, grid edge)
                                "out float vWaterGust;\n" // the gust and the current, per vertex
                                "out vec2 vWaterCur;\n"
                                "uniform highp float uCell;\n"
                                "uniform highp vec3 uCenter;\n"
                                // the near grid's rings: the inner spacing uCell out to uRing.x,
                                // twice it to uRing.y, four times it past; and where the short
                                // cascade has faded out (water.lod_waves, m from the camera)
                                "uniform highp vec3 uRing; uniform highp float uLodWaves;\n";
// The cascades and the swell, each faded by the depth under it and, on the near grid, toward its
// edge; the sideways move capped by the shore distance (sea_surface::displacement on the CPU); the
// wake's height on top.
const char *const kWaterVsImpl =
    "  vec4 dispMC = vertexMC;\n"
    "  vec2 w = vertexMC.xy + uCenter.xy;\n"
    "  float edge = max(abs(vertexMC.x), abs(vertexMC.y)) / max(uNearHalf, 1e-3);\n"
    // the wind (O13), once per vertex of either grid: the current that carries the short cascades
    // (the far grid's fragments read through it too) and the gust (its cat's paws fading out toward
    // the near grid's edge; none on the far grid)
    "  owFlow = owCurrent(w);\n"
    "  float gust = cvc_wind_gust(w, uWaves*(1.0 - smoothstep(0.35, 0.8, edge)));\n"
    "  vWaterGust = gust;\n"
    "  vWaterCur = owFlow;\n"
    "  vec3 k = vec3(0.0);\n"
    "  vec4 od = vec4(0.0, 0.0, 0.0, 1.0), osl = vec4(0.0);\n"
    "  vWaterN = vec3(0.0, 0.0, 1.0);\n"
    // The far grid stays flat (its normal is the fragment's): none of the near grid's reads.
    "  if (uWaves > 0.5) {\n"
    "    float ke = 1.0 - smoothstep(0.75, 1.0, edge);\n"
    // the vertex's own spacing: the ring it is in
    "    float r = max(abs(vertexMC.x), abs(vertexMC.y));\n"
    "    float sp = r <= uRing.x + 1e-3 ? uCell : (r <= uRing.y + 1e-3 ? 2.0*uCell : 4.0*uCell);\n"
    "    vec4 c0 = oCoast(0, w);\n" // (shore distance, depth)
    // each cascade's weight: the depth's fade, the grid's edge, and the wind's fetch and gust
    "    k = ke * depthW(c0.g) * owWindW(w, gust);\n"
    // the short cascade's metre-long waves fade out with distance from the camera (and are not
    // read); a negative uLodWaves leaves them out (adaptive quality's two cascades)
    "    k.z *= uLodWaves > 0.0 ? 1.0 - smoothstep(0.6*uLodWaves, uLodWaves, length(w - "
    "uCamPos.xy))"
    " : 0.0;\n"
    "    od = wSum(w, k);\n"
    // the swell at its true size: uWaveAmp scales the cascades only, so the shore layer's breaking
    // cap holds at any wave height
    "    osl = ke * oShoreLayer(w);\n"
    // the normal from differences half the vertex's spacing either way (1 m on the inner ring)
    "    float e = max(1.0, 0.5*sp);\n"
    "    vec2 ex = vec2(e, 0.0), ey = vec2(0.0, e);\n"
    "    float hpx = uWaveAmp*wHt(w+ex, k) + ke*oShoreEta(w+ex);\n"
    "    float hmx = uWaveAmp*wHt(w-ex, k) + ke*oShoreEta(w-ex);\n"
    "    float hpy = uWaveAmp*wHt(w+ey, k) + ke*oShoreEta(w+ey);\n"
    "    float hmy = uWaveAmp*wHt(w-ey, k) + ke*oShoreEta(w-ey);\n"
    "    vWaterN = normalize(vec3(-(hpx-hmx)/(2.0*e), -(hpy-hmy)/(2.0*e), 1.0));\n"
    "    vec2 dh = vec2(od.x, od.z) + osl.yz;\n"
    "    float dlim = 0.8*max(c0.r, 0.0), dlen = length(dh);\n"
    "    if (dlen > dlim) dh *= dlim/max(dlen, 1e-6);\n"
    "    dispMC.xy += dh;\n"
    "    dispMC.z += uWaveAmp*od.y + osl.x + ke*wkVertex(w, sp);\n"
    "  }\n"
    "  vertexVCVSOutput = MCVCMatrix * dispMC;\n"
    "  gl_Position = MCDCMatrix * dispMC;\n"
    "  vWaterPos = vec3(dispMC.xy + uCenter.xy, dispMC.z);\n"
    "  vWaterRest = w;\n"
    "  vWaterJac = od.w;\n"
    "  vShoreBreak = osl.w;\n"
    "  vWaterK = k;\n";

const char *const kWaterFsDec =
    "//VTK::Normal::Dec\n"
    "in vec3 vWaterPos;\n"
    "in vec2 vWaterRest;\n"
    "in vec3 vWaterN;\n"
    "in float vWaterJac;\n"
    "in float vShoreBreak;\n"
    "in vec3 vWaterK;\n"
    "in float vWaterGust;\n"
    "in vec2 vWaterCur;\n"
    "uniform highp vec3 uSunDir;\n"
    "uniform highp float uFoamBias; uniform highp float uFoamRamp; uniform highp float uFoamFray;\n"
    "uniform highp float uShore; uniform highp float uTime;\n"
    // The persistent foam (OceanFoam's atlas: the long and middle cascades' tiles); whether it is
    // on and whether this mesh draws its detail (the near grid; the far grid draws the sea's mean
    // cover, uFoamCover, in its place); the distances the foam's detail and the full shading end
    // at (water.lod_foam, water.lod_shade); the wind (the roughness, the windrows).
    "uniform highp sampler2D uFoamTex; uniform highp float uFoamOn;\n"
    "uniform highp float uFoamDetail; uniform highp float uFoamCover;\n"
    "uniform highp float uLodFoam; uniform highp float uLodShade; uniform highp float uFarReach;\n"
    "uniform highp float uFoamReach;\n"
    "uniform highp float uWindU; uniform highp vec3 uWindDir; uniform highp float uCell;\n"
    "uniform highp vec3 uCutMin; uniform highp vec3 uCutMax;\n"   // the far grid gives way inside
                                                                  // the near one
    "uniform highp vec3 uClipMin; uniform highp vec3 uClipMax;\n" // no water past the world,
                                                                  // unless its rim is sea
    // Up to four hulls the water stays out of (D5): each its origin and heading (x, y, yaw), its
    // half length, half beam and sheer's height, and its bottom's height (none while the half
    // length is 0). Uniforms, not a texture: a WebGL2 draw has 16 texture units, and the
    // water's samplers and the low-memory mapper's arrays take most of them.
    "uniform highp vec3 uHullP0; uniform highp vec3 uHullP1; uniform highp vec3 uHullP2;\n"
    "uniform highp vec3 uHullP3;\n"
    "uniform highp vec3 uHullE0; uniform highp vec3 uHullE1; uniform highp vec3 uHullE2;\n"
    "uniform highp vec3 uHullE3;\n"
    "uniform highp float uHullB0; uniform highp float uHullB1; uniform highp float uHullB2;\n"
    "uniform highp float uHullB3;\n";
const char *const kWaterFsFuncs =
    // Inside a hull's footprint (its box about its origin, turned to its heading), under its
    // sheer and over its bottom: water a boat holds out. Green water over the sheer still shows,
    // and so does a trough under its bottom (a hull bridges the waves shorter than it is).
    "bool wHull1(vec3 p, vec3 c, vec3 e, float b){\n"
    "  if (e.x <= 0.0) return false;\n"
    "  vec2 d = p.xy - c.xy; float ch = cos(c.z), sh = sin(c.z);\n"
    "  return abs(d.x*ch + d.y*sh) < e.x && abs(d.y*ch - d.x*sh) < e.y && p.z < e.z && p.z > b;\n"
    "}\n"
    "bool wInHull(vec3 p){\n"
    "  return wHull1(p, uHullP0, uHullE0, uHullB0) || wHull1(p, uHullP1, uHullE1, uHullB1) ||\n"
    "         wHull1(p, uHullP2, uHullE2, uHullB2) || wHull1(p, uHullP3, uHullE3, uHullB3);\n"
    "}\n"
    "float whash(vec2 p){ return fract(sin(dot(p, vec2(127.1,311.7)))*43758.5453); }\n"
    "float wnoise(vec2 p){\n"
    "  vec2 i=floor(p), f=fract(p); f=f*f*(3.0-2.0*f);\n"
    "  return mix(mix(whash(i),whash(i+vec2(1,0)),f.x),\n"
    "             mix(whash(i+vec2(0,1)),whash(i+vec2(1,1)),f.x), f.y);\n"
    "}\n"
    // The far grid's cascade weights at distance dist: the middle cascade fades out by 3 near,
    // the long one and the swell by 10 near (the cascades' texels are read nearest, so a pixel
    // wider than a texel would sparkle), both nearer at adaptive quality's lower tiers
    // (uFarReach); the short one is never in its normal.
    "vec3 wFarK(float dist){\n"
    "  float r = max(uFarReach, 0.1);\n"
    "  return vec3(1.0 - smoothstep(uNearHalf, max(10.0*r, 1.01)*uNearHalf, dist),\n"
    "              1.0 - smoothstep(uNearHalf, max(3.0*r, 1.01)*uNearHalf, dist), 0.0);\n"
    "}\n"
    // The far grid's normal: the long and middle cascades' and the swell's slopes at the fragment,
    // the difference's step growing with distance (1 + 0.004 dist), each cascade weighted by k (the
    // caller's: the depth's fade, wFarK's fade with distance and the lee); the swell by wFarK's.
    "vec3 wFarNormal(vec2 p, float dist, vec3 k){\n"
    "  float kl = wFarK(dist).x;\n"
    "  float e = 1.0 + 0.004*dist;\n"
    "  vec2 ex = vec2(e, 0.0), ey = vec2(0.0, e);\n"
    "  float hx = uWaveAmp*(wHt(p+ex,k) - wHt(p-ex,k)) + kl*(oShoreEta(p+ex) - oShoreEta(p-ex));\n"
    "  float hy = uWaveAmp*(wHt(p+ey,k) - wHt(p-ey,k)) + kl*(oShoreEta(p+ey) - oShoreEta(p-ey));\n"
    "  return normalize(vec3(-hx/(2.0*e), -hy/(2.0*e), 1.0));\n"
    "}\n";
// A persistent foam tile at p (OceanFoam's oFoamTile, so after oceanFoamGLSL()), read through the
// current's flow map when the current carries its cascade (adv), as the cascade's waves are
// (wDisp): the foam stays on their folds. The two reads' weights are normalised, since the foam is
// a cover and not a wave.
const char *const kWaterFoamFlow =
    "vec4 oFoamAt(int c, int fn, float tile, vec2 p, float adv){\n"
    "  if (adv < 0.5 || uCurrent.x <= 0.0) return oFoamTile(uFoamTex, c, fn, tile, p);\n"
    "  vec2 q0 = p - owFlow*uFlowTau.x + uFlowOff0.xy*tile;\n"
    "  vec2 q1 = p - owFlow*uFlowTau.y + uFlowOff1.xy*tile;\n"
    "  return (uFlowW.x*oFoamTile(uFoamTex, c, fn, tile, q0) +\n"
    "          uFlowW.y*oFoamTile(uFoamTex, c, fn, tile, q1)) / max(uFlowW.x + uFlowW.y, 1e-6);\n"
    "}\n";
// Consumes //VTK::Light::Impl (VTK's lighting is replaced) and writes the final colour.
// WATER_SUN_SHADE is the share of the sun that reaches the water: 1.0, or, once a cloud_sky lists
// the water in its receivers and has wired its shade in (water_light below), the deck's shade
// there, cloudShade(), which the sky declares at //VTK::Light::Dec.
//
// Level of detail: every cost this adds depends on the distance to the camera.
//   - The shading (A4, OCEAN-AND-WEATHER-ROADMAP.md section 10.11) is ABYSSAL's within
//     water.lod_shade: the sun's GGX lobe on the roughness the normal does not resolve (the
//     pixel's footprint against each cascade's texels, Cox-Munk's slopes for the wind), Fresnel
//     softened as it roughens, the sky's reflection blurred with it, light through backlit crests,
//     and the bubbles a breaker left brightening the water. Past it, over the last quarter, it
//     gives way to the O15 model (a fixed Fresnel, a Phong glint).
//   - The persistent foam is read (two tiles of OceanFoam's atlas, bilinear: 8 fetches) and carved
//     into windrows on the near grid within water.lod_foam; over its last 40 % it gives way to
//     the sea's mean whitecap cover, which is all the far grid draws. The instantaneous whitecaps
//     (the fold itself, at the fragment's resolution) give way with it.
const char *const kWaterFsLight =
    // The depth at the fragment, from the bed itself: the far grid's cells are metres across.
    "  float sunShade = WATER_SUN_SHADE;\n"
    "  float depthF = oDepth(vWaterRest);\n"
    "  if (depthF < -0.25) discard;\n" // dry ground: the terrain draws there
    "  if (any(lessThan(vWaterRest, uClipMin.xy)) || any(greaterThan(vWaterRest, uClipMax.xy))) "
    "discard;\n"
    "  if (all(greaterThan(vWaterRest, uCutMin.xy)) && all(lessThan(vWaterRest, uCutMax.xy))) "
    "discard;\n"
    // The pixel's footprint on the water (m), from the derivatives, taken before any branch: the
    // geometric mean of its axes, the minor one at least a sixteenth of the major (ABYSSAL's).
    "  vec2 fdx = dFdx(vWaterRest), fdy = dFdy(vWaterRest);\n"
    "  float fpA = length(fdx), fpB = length(fdy), fpMaj = max(fpA, fpB);\n"
    "  float fp = sqrt(fpMaj*max(min(fpA, fpB), fpMaj/16.0));\n"
    "  float dist = length(uCamPos - vWaterPos);\n"
    "  float distXY = length(uCamPos.xy - vWaterRest);\n"
    // a hull keeps the water out (after the derivatives above, taken before any branch)
    "  if (wInHull(vWaterPos)) discard;\n"
    "  vec3 N = normalize(vWaterN);\n"
    "  owFlow = vWaterCur;\n" // the current the far grid's cascades are read through
    // the cascades in the normal: the near grid's per vertex (vWaterK), the far grid's here (its
    // detail fading with distance; the lee, O13)
    "  vec3 inN = vWaterK;\n"
    "  if (uWaves < 0.5) {\n"
    "    inN = oDepthW(vWaterRest)*wFarK(distXY)*owWindW(vWaterRest, vWaterGust);\n"
    "    N = wFarNormal(vWaterRest, distXY, inN);\n"
    "  }\n"
    // the wake's slope adds to the sea's (both as -dz/dx, -dz/dy over z of the normal)
    "  vec3 wk = wkSlopeFoam(vWaterRest);\n"
    // Cat's paws (O13, E3): where a gust ruffles the water it roughens. Near the camera a fine
    // ripple tilts the normal (faded out by 300 m); everywhere the patch reads darker (below).
    "  float rough = clamp((vWaterGust - 1.0)*3.0, 0.0, 1.0);\n" // none in a steady wind
    "  float rnear = rough*(1.0 - smoothstep(80.0, 300.0, dist));\n"
    "  if (rnear > 0.0) {\n"
    "    vec3 r1 = cvc_noise(vWaterRest*1.3 + vec2(uTime*0.9, uTime*0.4), 0x9a1u);\n"
    "    vec3 r2 = cvc_noise(vWaterRest*3.1 - vec2(uTime*0.6, uTime*1.1), 0x9a2u);\n"
    "    wk.xy += rnear*0.06*(1.3*r1.yz + 1.55*r2.yz);\n"
    "  }\n"
    "  N = normalize(vec3(N.xy/max(N.z, 1e-3) - wk.xy, 1.0));\n"
    "  vec3 Vd = normalize(uCamPos - vWaterPos);\n"
    // The persistent foam and its bubbles: the stronger long cascade's, each faded as its waves
    // fade (vWaterK), carved into windrows (cvc::ocean::foam_look).
    "  float fd = uFoamDetail*(1.0 - smoothstep(0.6*uLodFoam, uLodFoam, distXY));\n"
    // its reads reach a share of that at adaptive quality's lower tiers (uFoamReach: 1 at full)
    "  float pr = uLodFoam*uFoamReach;\n"
    "  float fdp = pr > 0.0 ? uFoamDetail*(1.0 - smoothstep(0.6*pr, pr, distXY)) : 0.0;\n"
    "  float pf = 0.0, bub = 0.0;\n"
    "  if (uFoamOn > 0.5 && fdp > 0.0) {\n"
    "    int fn = int(0.5/uHalfTexel + 0.5);\n"
    "    vec4 f0 = oFoamAt(0, fn, uTile0, vWaterRest, uAdvect.x);\n"
    "    vec4 f1 = oFoamAt(1, fn, uTile1, vWaterRest, uAdvect.y);\n"
    "    float F = max(vWaterK.x*f0.r, vWaterK.y*f1.r);\n"
    "    if (F > 0.2) pf = oFoamLook(F, oWindrow(vWaterRest, uWindDir.xy, fp))*(fdp/fd);\n"
    "    bub = max(vWaterK.x*f0.g, vWaterK.y*f1.g)*fdp;\n"
    "  }\n"
    // The shading: A4 near (sd 1), the O15 model far (sd 0), mixed between. The cascades in the
    // normal are the near grid's (its vertices 2 m apart) or the far grid's (its differences
    // growing with distance, the short cascade left out).
    "  float sd = 1.0 - smoothstep(0.75*uLodShade, uLodShade, dist);\n"
    "  vec3 col = sd < 1.0 ? oWaterSimple(N, Vd, uSunDir, sunShade) : vec3(0.0);\n"
    "  if (sd > 0.0) {\n"
    "    float nstep = uWaves > 0.5 ? uCell : 1.0 + 0.004*distXY;\n"
    "    vec3 full = oWaterFull(N, Vd, uSunDir, sunShade, vWaterPos.z - uLevel, bub, uWindU, fp,\n"
    "                           nstep, vec3(uTile0, uTile1, uTile2), 0.5/uHalfTexel, inN,\n"
    "                           vWaterRest, uWindDir.xy, uTime);\n"
    "    col = sd >= 1.0 ? full : mix(col, full, sd);\n"
    "  }\n"
    // a ruffled patch (the cat's paws above) mirrors less of the bright sky
    "  col = mix(col, vec3(0.05,0.22,0.38)*(0.30 + 0.70*max(dot(N, uSunDir), 0.0)*sunShade),\n"
    "            0.3*rough);\n"

    // Shallow water is translucent: the bed shows through, fading with the depth under the surface
    // (Beer-Lambert), so the shallows brighten and darken as the waves pass.
    "  float depthS = vWaterPos.z - uLevel + oDepth(vWaterPos.xy);\n"
    "  float clarity = exp(-max(depthS,0.0)*0.20);\n"
    "  vec3 shoal = mix(vec3(0.12,0.46,0.54), vec3(0.66,0.64,0.50), clarity*0.55);\n"
    "  col = mix(col, shoal, clarity*0.85);\n"
    // Whitecaps where the cascades fold (cvc/ocean/whitecaps.h has the threshold's meaning), with
    // the aging foam; far off, the sea's mean cover in their place (no whitecaps in the shallows).
    "  float fray = 0.65*wnoise(vWaterRest*1.9) + 0.35*wnoise(vWaterRest*4.3 + vec2(17.0, 31.0));\n"
    "  float foam = clamp((uFoamBias - vWaterJac + uFoamFray*(fray - 0.5))/uFoamRamp, 0.0, 1.0);\n"
    "  foam = foam*foam*(3.0-2.0*foam);\n"
    "  float cover = uFoamCover*smoothstep(0.0, uFade.y, depthF);\n"
    "  foam = mix(cover, max(foam, pf), fd);\n"
    // A noisy band at the waterline, washing up and down with the waves: faint on calm water, full
    // where the swell reaches; and the swell's breakers on their crests.
    "  float nz = wnoise(vWaterPos.xy*0.4 + vec2(uTime*0.6, uTime*0.35));\n"
    "  float surf = clamp(0.25 + (uSwellOn > 0.5 ? oCoast(1, vWaterRest).y : 0.0)/0.25, 0.0, "
    "1.0);\n"
    "  float band = 1.0 - smoothstep(0.0, 1.8, abs(depthS));\n"
    "  foam = max(foam, clamp(band*(0.55+0.75*nz)*step(-0.4, depthS)*surf*uShore, 0.0, 1.0));\n"
    "  foam = max(foam, clamp(vShoreBreak*(0.7+0.6*nz)*uShore, 0.0, 1.0));\n"
    // the wake's: a splash, a bow wave
    "  foam = max(foam, wk.z*(0.75+0.5*nz));\n"
    "  col = mix(col, oFoamColour(N, uSunDir, sunShade, sd), foam);\n"
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

// A flat grid at z over the coordinates xs by ys (each ascending).
cvc::geometry tensor_grid(cvc::app &app, const std::vector<double> &xs,
                          const std::vector<double> &ys, double z) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  const int nx = int(xs.size()), ny = int(ys.size());
  pts.reserve(std::size_t(nx) * std::size_t(ny));
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      pts.push_back({xs[std::size_t(i)], ys[std::size_t(j)], z});
      cols.push_back({0.05, 0.20, 0.32}); // the flat fallback's colour
    }
  auto &tris = g.tris();
  using idx = cvc::geometry::index_t;
  for (int j = 0; j < ny - 1; ++j)
    for (int i = 0; i < nx - 1; ++i) {
      const idx v = idx(j * nx + i);
      tris.push_back({v, idx(v + 1), idx(v + nx)});
      tris.push_back({idx(v + 1), idx(v + nx + 1), idx(v + nx)});
    }
  return g;
}

// The near grid's coordinates along an axis, from -half to half: spacing s out to r1, 2 s out to
// r2, 4 s out to half (all whole multiples of 4 s), so the vertex count falls off with distance
// from the camera and every vertex stays on its own ring's lattice when the grid moves by 4 s.
std::vector<double> ring_coords(double s, double r1, double r2, double half) {
  std::vector<double> pos{0.0};
  const int n1 = int(std::lround(r1 / s)), n2 = int(std::lround((r2 - r1) / (2.0 * s))),
            n3 = int(std::lround((half - r2) / (4.0 * s)));
  for (int i = 1; i <= n1; ++i)
    pos.push_back(i * s);
  for (int i = 1; i <= n2; ++i)
    pos.push_back(n1 * s + i * 2.0 * s);
  for (int i = 1; i <= n3; ++i)
    pos.push_back(n1 * s + n2 * 2.0 * s + i * 4.0 * s);
  std::vector<double> out;
  for (std::size_t i = pos.size(); i-- > 1;)
    out.push_back(-pos[i]);
  out.insert(out.end(), pos.begin(), pos.end());
  return out;
}

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
  cvc::ocean::cascade band;
  cvc::ocean::h0_field h0; // what the FFT was given (the whitecaps' threshold is found from it)
};

// water.profile: 1 times the node's work a frame, section by section, on the CPU; 2 also waits for
// the GPU at the end of each section (glFinish), so the passes a section queues count in it (and
// the frame slows). Published as water.stats.<section>_ms; 0 (the default) times nothing.
struct WaterLaps {
  int mode = 0;
  vtkOpenGLRenderWindow *ow = nullptr;
  std::chrono::steady_clock::time_point t;
  std::vector<std::pair<const char *, double>> laps;
  void begin(int m, vtkOpenGLRenderWindow *w) {
    mode = m;
    ow = w;
    laps.clear();
    if (mode > 0) {
      fence();
      t = std::chrono::steady_clock::now();
    }
  }
  void fence() const {
    if (mode >= 2 && ow)
      ow->WaitForCompletion();
  }
  void lap(const char *name) {
    if (mode <= 0)
      return;
    fence();
    const auto n = std::chrono::steady_clock::now();
    laps.emplace_back(name, std::chrono::duration<double, std::milli>(n - t).count());
    t = n;
  }
};

// A boat world vehicle's water cut-out, its sheer and its bottom (boat_box).
struct hull_box {
  cvc::physics::waterline_box box;
  double sheer = 0.0, bottom = 0.0;
};

struct WaterRT {
  ari::SceneNode node; // its props and keys: (scene_water_spec)
  std::string prefix;
  std::weak_ptr<GeometryNode> near, far;
  std::shared_ptr<const cvc::height_field> ground;
  const float *bedData = nullptr; // the ground's heights the bed was made from (regenerates)
  std::size_t bedSize = 0;
  std::vector<Cascade> cascades;
  std::unique_ptr<OceanCascades> fft; // every cascade's FFT, in one set of passes
  // The sea's keyframes of its lagged wind (O13, scene_water.h): the physics runtime's when there
  // is one, else this node's own clock's. `seaDrawn` is the one whose fetch maps are in the coast
  // atlas; `seaFinal` says its new h0 is uploaded whole (the crossfade over), so nothing more is
  // uploaded until the next keyframe. h0Mix is the crossfade's scratch.
  std::unique_ptr<ari::SceneWaterSeaClock> seaClock;
  std::shared_ptr<const ari::SceneWaterSea> sea, seaDrawn;
  bool seaFinal = false;
  std::vector<cvc::ocean::h0_field> h0Mix;
  bool ready = false, failed = false;
  double level = 0.0, nearHalf = 192.0, cell = 2.0;
  std::size_t hulls = 0;              // the hulls the water keeps out of this frame (tick_hulls)
  double ring1 = 64.0, ring2 = 128.0; // the near grid's rings (ring_coords)
  std::string lodWavesKey;
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
  // The persistent foam (water.foam_decay > 0, section 10.2): the long cascades' turbulence, the
  // world time it was stepped to, its injection's threshold, and the calibration in flight (both
  // thresholds for Monahan's share, found off the frame; until the first lands, a guess).
  std::unique_ptr<OceanFoam> foam;
  double foamT = -1.0;
  double persistBias = 0.77;
  std::shared_ptr<OceanFoamCalibration> foamJob;
  std::string decayKey, lodFoamKey, lodShadeKey;
  // Adaptive quality (scene_quality.h, O21): water.quality (< 0: the scene's tier and scale,
  // quality.tier and quality.scale; 0 .. 4 that tier) and the scene's keys. With the persistent
  // foam off by a tier, the whitecaps take the bias that covers Monahan's share alone (found with
  // the other two, off the frame; a guess until it lands).
  std::string qualityKey, tierKey, scaleKey;
  double aloneBias = 0.8;
  // water.profile's timings (the frame's tick and the wake's), and the key.
  WaterLaps laps, wakeLaps;
  std::string profileKey;
  ~WaterRT() {
    if (foamJob)
      foamJob->cancel();
  }
  // The coast (scene_water.h): the physics runtime's when there is one, else this node's own,
  // built again when the ground or (once the keys have held for kSettle frames) the swell changes.
  std::shared_ptr<const ari::SceneWaterShore> shore, shoreDrawn;
  std::vector<double> shoreSig, shorePending;
  int shoreSettled = 0;
  const float *shoreGround = nullptr;
  vtkSmartPointer<vtkTextureObject> coastTex, swashTex; // the coast atlas; the ground's swash
  // The coast atlas's planes as upload_shore built them (RGBA, row by row, the planes side by
  // side), kept so that a keyframe's new fetch maps can join them (upload_atlas) without building
  // the coast again; and those fetch maps' texels (OceanWind::fetch).
  std::vector<float> coastPlanes;
  int coastCols = 1, coastRows = 1, coastPlaneCount = 1;
  OceanWind::fetch_texels fetch;
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
  std::map<std::string, hull_box> boat_boxes; // by archetype and mass (boat_box)
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

// What adaptive quality has the water draw now: water.quality's tier, else the scene's tier and
// scale (full when nothing runs the loop: a test, a capture on the fixed clock).
ari::WaterQuality quality_of(cvc::app &app, const WaterRT &rt) {
  const double pinned = key(app, rt.qualityKey, -1.0);
  if (pinned >= 0.0)
    return ari::water_quality(static_cast<int>(std::lround(pinned)), 1.0);
  return ari::water_quality(static_cast<int>(std::lround(key(app, rt.tierKey, 0.0))),
                            key(app, rt.scaleKey, 1.0));
}

// The sea's keyframes now (O13): the physics runtime's, or this node's own clock stepped to the
// world's time over the coast. Call after follow_shore.
void follow_sea(cvc::app &app, WaterRT &rt) {
  if (rt.physics) {
    rt.sea = rt.physics->keyframes();
    return;
  }
  if (!rt.seaClock)
    rt.seaClock = std::make_unique<ari::SceneWaterSeaClock>();
  rt.seaClock->advance(app.world_clock().t(), spec_of(app, rt), rt.shore);
  rt.sea = rt.seaClock->sea();
}

// What each cascade takes from the wind (scene_water.h's split), for keyframes k.
OceanWind::cascades wind_cascades(const ari::SceneWaterSea &k) {
  OceanWind::cascades c;
  for (std::size_t i = 0; i < 3; ++i) {
    c.full[i] = i < k.full.size() ? k.full[i] : 0.0;
    c.full_from[i] = i < k.full_from.size() ? k.full_from[i] : c.full[i];
    c.gust[i] = ari::kWaterGustWeight[i];
    c.advect[i] = ari::kWaterAdvect[i];
  }
  return c;
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

// The coast atlas to the GPU, on both meshes: the coast's planes (rt.coastPlanes, upload_shore's)
// and beside them the wind's fetch maps (rt.fetch: two channels of a texel, the rest 0), each
// from row 0, the atlas as tall as the taller. False when the texture failed.
bool upload_atlas(WaterRT &rt, vtkOpenGLRenderWindow *ow, GeometryNode &near, GeometryNode &far) {
  const OceanWind::fetch_texels &ft = rt.fetch;
  const int pw = rt.coastCols * rt.coastPlaneCount;               // the planes' columns
  const int fc = std::max(ft.cols, 1), fr = std::max(ft.rows, 1); // the fetch maps'
  const bool haveFetch = ft.rg.size() >= std::size_t(fc) * std::size_t(fr) * 2;
  if (rt.coastPlanes.size() < std::size_t(pw) * std::size_t(rt.coastRows) * 4)
    return false; // no coast yet (upload_shore puts the fetch maps in with it)
  const int width = pw + fc, height = std::max(rt.coastRows, fr);
  std::vector<float> atlas(std::size_t(width) * std::size_t(height) * 4, 0.0f);
  for (int r = 0; r < rt.coastRows; ++r)
    std::copy_n(&rt.coastPlanes[std::size_t(r) * std::size_t(pw) * 4], std::size_t(pw) * 4,
                &atlas[std::size_t(r) * std::size_t(width) * 4]);
  for (int r = 0; r < fr; ++r)
    for (int c = 0; c < fc; ++c) {
      float *t = &atlas[(std::size_t(r) * std::size_t(width) + std::size_t(pw + c)) * 4];
      if (haveFetch) {
        const std::size_t i = (std::size_t(r) * std::size_t(fc) + std::size_t(c)) * 2;
        t[0] = ft.rg[i];
        t[1] = ft.rg[i + 1];
      } else { // no map yet: open water, no lee (OceanWind::fetch's one open texel)
        t[0] = t[1] = 1.0e9f;
      }
    }
  rt.coastTex = float_texture(ow, width, height, 4, atlas.data());
  if (!rt.coastTex)
    return false;
  for (GeometryNode *m : {&near, &far}) {
    m->setShaderTexture("uCoast", rt.coastTex);
    m->setShaderUniformf("uCoastN", static_cast<float>(rt.coastCols));
    m->setShaderUniformf("uCoastRows", static_cast<float>(rt.coastRows));
    m->setShaderUniform3f("uCoastFetch", static_cast<float>(pw), static_cast<float>(fc),
                          static_cast<float>(fr));
    m->setShaderUniform3f("uWindFetchGrid", ft.grid[0], ft.grid[1], ft.grid[2]);
  }
  return true;
}

// The coast to the GPU: the shore distance and the shore layer's two planes and trains (a texel of
// nothing without a swell), on both meshes. False when a texture failed.
bool upload_shore(WaterRT &rt, vtkOpenGLRenderWindow *ow, GeometryNode &near, GeometryNode &far) {
  const ari::SceneWaterShore *sh = rt.shore.get();
  const cvc::ocean::shore_grid *g = sh ? sh->grid.get() : nullptr;
  const cvc::ocean::shore_layer *L = sh ? sh->layer.get() : nullptr;
  // The coast atlas (see kWaterCommonDec): plane 0 the shore distance and the still water's depth,
  // and with a swell the shore layer's two planes beside it. No grid: one texel of open, deep
  // water (no cap on the sideways move).
  const bool swell = g && L && !L->empty() && L->cols == g->cols && L->rows == g->rows;
  const int cols = g ? g->cols : 1, rows = g ? g->rows : 1, planes = swell ? 3 : 1;
  std::vector<float> atlas(std::size_t(cols) * planes * std::size_t(rows) * 4, 0.0f);
  auto texel = [&](int plane, std::size_t i) {
    const std::size_t r = i / std::size_t(cols), c = i % std::size_t(cols);
    return &atlas[((r * std::size_t(planes) + std::size_t(plane)) * std::size_t(cols) + c) * 4];
  };
  if (g) {
    for (std::size_t i = 0; i < g->count(); ++i) {
      float *t = texel(0, i);
      const float d = g->depth[i], sd = g->shore_m[i];
      t[0] = std::isfinite(sd) ? sd : -1.0f; // off the grid: dry
      t[1] = std::isfinite(d) ? d : -1.0f;
    }
  } else {
    atlas[0] = 1.0e6f;
    atlas[1] = 1.0e3f;
  }
  rt.swashTex = nullptr;
  if (swell) {
    const std::vector<float> wave = L->wave_texels(), dir = L->dir_texels();
    for (std::size_t i = 0; i < std::size_t(cols) * std::size_t(rows); ++i)
      for (int k = 0; k < 4; ++k) {
        texel(1, i)[k] = wave[i * 4 + std::size_t(k)];
        texel(2, i)[k] = dir[i * 4 + std::size_t(k)];
      }
    // The ground's own texture for the swash, (phase, runup): bound in the ground's programs
    // only. Binding the water's in them as well left the ground without its colour texture.
    if (rt.swash) {
      std::vector<float> sw(L->phase.size() * 2);
      for (std::size_t i = 0; i < L->phase.size(); ++i) {
        sw[i * 2] = L->phase[i];
        sw[i * 2 + 1] = L->runup[i];
      }
      rt.swashTex = float_texture(ow, L->cols, L->rows, 2, sw.data());
    }
  }
  rt.coastPlanes = std::move(atlas);
  rt.coastCols = cols;
  rt.coastRows = rows;
  rt.coastPlaneCount = planes;
  if (!upload_atlas(rt, ow, near, far))
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
    m->setShaderUniformf("uSwellOn", swell ? 1.0f : 0.0f);
    m->setShaderUniform3f("uShoreGridOrigin", ox, oy, 0.0f);
    m->setShaderUniform3f("uShoreGridSpacing", cs, cs, 0.0f);
    m->setShaderUniformf("uSwellOmega", L ? static_cast<float>(L->omega) : 0.0f);
    for (int j = 0; j < 4; ++j)
      m->setShaderUniform3f(names[j], v[j][0], v[j][1], v[j][2]);
  }
  rt.shoreDrawn = rt.shore;
  return true;
}

// The shaders and the cascades' atlas on one of the water's meshes (the coast's atlas, with the
// bed's depth, is upload_shore's).
void style_ocean(GeometryNode &g, const WaterRT &rt) {
  const char *tile[3] = {"uTile0", "uTile1", "uTile2"};
  g.setShaderTexture("uDisp", rt.fft->displacement());
  for (std::size_t i = 0; i < rt.cascades.size() && i < 3; ++i)
    g.setShaderUniformf(tile[i], static_cast<float>(rt.cascades[i].band.tile_m));
  g.setShaderUniformf("uHalfTexel", 0.5f / static_cast<float>(kFftN));
  g.setShaderUniformf("uLevel", static_cast<float>(rt.level));
  g.setShaderUniformf("uNearHalf", static_cast<float>(rt.nearHalf));
  g.setShaderUniformf("uCell", static_cast<float>(rt.cell));
  g.setShaderUniform3f("uRing", static_cast<float>(rt.ring1), static_cast<float>(rt.ring2), 0.0f);
  const std::string common = common_glsl(rt.wake != nullptr);
  g.addVertexShaderReplacement("//VTK::PositionVC::Dec", std::string(kWaterVsDec) + common);
  g.addVertexShaderReplacement("//VTK::PositionVC::Impl", kWaterVsImpl);
  g.addFragmentShaderReplacement("//VTK::Normal::Dec", std::string(kWaterFsDec) + common +
                                                           kWaterFsFuncs + oceanFoamGLSL() +
                                                           kWaterFoamFlow);
  g.addFragmentShaderReplacement("//VTK::Light::Impl",
                                 water_light(g.hasSharedShaderInputs(kCloudInputs)));
  // The persistent foam's atlas (one sampler for both tiles); off until it is stepped.
  g.setShaderUniformf("uFoamOn", 0.0f);
  if (rt.foam && rt.foam->texture())
    g.setShaderTexture("uFoamTex", rt.foam->texture());
  else
    g.setShaderTexture("uFoamTex", rt.fft->displacement()); // never read
  if (rt.wake) {
    g.setShaderTexture("uWake", rt.wake->heightTexture());
    g.setShaderUniformf("uWakeN", static_cast<float>(rt.wake->params().n));
    g.setShaderUniformf("uWakeOn", 0.0f); // until the window is placed
  }
}

// Whether the ground's heights are new (the first time, or a regenerated world).
bool refresh_bed(WaterRT &rt) {
  const std::vector<float> &h = rt.ground->heights();
  if (h.data() == rt.bedData && h.size() == rt.bedSize)
    return false;
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

// The cascades and their textures, once the window is up (the sea's keyframes found first, so
// their h0 is the cascades'); false when float targets are missing.
bool start_ocean(cvc::app &app, WaterRT &rt, vtkOpenGLRenderWindow *ow) {
  const ari::SceneWaterSpec spec = spec_of(app, rt);
  const ari::SceneWaterCascades cd = ari::scene_water_cascades(spec);
  const std::vector<cvc::ocean::cascade> bands = cvc::ocean::make_cascades(cd.tiles, cd.seed);
  if (bands.empty() || bands.size() > 3)
    return false;
  // All the cascades in one set of passes (OceanCascades: 18 for three, where an OceanFFT each
  // took 105).
  rt.fft = std::make_unique<OceanCascades>(kFftN, int(bands.size()));
  rt.fft->depth_m = 1000.0f;
  rt.fft->chop = static_cast<float>(spec.chop);
  if (!rt.fft->init(ow))
    return false;
  std::size_t i = 0;
  for (const cvc::ocean::cascade &b : bands) {
    Cascade c;
    c.band = b;
    if (rt.sea && i < rt.sea->h0.size())
      c.h0 = rt.sea->h0[i];
    rt.cascades.push_back(std::move(c));
    ++i;
  }
  rt.seaDrawn = nullptr; // the frame uploads the keyframes' h0 and fetch maps
  return true;
}

// The fetch maps to the GPU: the old and the new keyframe's (OceanWind::fetch), in the coast atlas
// beside its planes (upload_atlas). Once per keyframe; a texel of open water without a lee.
void upload_fetch(WaterRT &rt, vtkOpenGLRenderWindow *ow, GeometryNode &near, GeometryNode &far) {
  const ari::SceneWaterSea *k = rt.sea.get();
  rt.fetch = OceanWind::fetch(k ? k->fetch_from.get() : nullptr, k ? k->fetch.get() : nullptr);
  upload_atlas(rt, ow, near, far);
}

// The sea's keyframes onto the GPU at world time t: on a new keyframe its fetch maps (and its h0
// kept for the whitecaps); while its crossfade runs, the two keyframes' h0 mixed on the CPU by the
// crossfade's weight and uploaded each frame; once it is over, the new h0 once. Between keyframes
// nothing is uploaded.
void sync_sea(WaterRT &rt, vtkOpenGLRenderWindow *ow, GeometryNode &near, GeometryNode &far,
              double t) {
  const std::shared_ptr<const ari::SceneWaterSea> k = rt.sea;
  if (!k || k->h0.size() < rt.cascades.size())
    return;
  if (k != rt.seaDrawn) {
    for (std::size_t i = 0; i < rt.cascades.size(); ++i)
      rt.cascades[i].h0 = k->h0[i];
    if (ow)
      upload_fetch(rt, ow, near, far);
    rt.seaDrawn = k;
    rt.seaFinal = false;
  }
  const bool fading = k->fading(t) && k->h0_from.size() >= rt.cascades.size();
  if (fading) {
    const double w = k->weight(t);
    rt.h0Mix.resize(rt.cascades.size());
    for (std::size_t i = 0; i < rt.cascades.size(); ++i) {
      OceanWind::mix_h0(k->h0_from[i], k->h0[i], w, rt.h0Mix[i]);
      rt.fft->setH0(int(i), rt.h0Mix[i]);
    }
    rt.seaFinal = false;
  } else if (!rt.seaFinal) {
    for (std::size_t i = 0; i < rt.cascades.size(); ++i)
      rt.fft->setH0(int(i), k->h0[i]);
    rt.seaFinal = true;
  }
}

// What the whitecaps' threshold depends on: the sea's wind (through h0: its new keyframe's), the
// chop, the ramp, the fray, the share of Monahan's cover asked for and how long the persistent
// foam lasts (0: none).
std::vector<double> foam_signature(cvc::app &app, const WaterRT &rt) {
  const ari::SceneWaterSpec s = spec_of(app, rt);
  const cvc::ocean::sea_wind w =
      rt.sea ? rt.sea->to
             : cvc::ocean::sea_wind{s.wind_speed, s.wind_dir * 3.14159265358979323846 / 180.0,
                                    s.fetch};
  return {w.speed_mps,
          w.fetch_m,
          w.dir_rad,
          s.chop,
          key(app, rt.rampKey, 0.1),
          key(app, rt.frayKey, 0.12),
          key(app, rt.capsKey, 1.0),
          std::max(key(app, rt.decayKey, 2.5), 0.0)};
}

// The persistent foam's integration for water.foam_decay (s; 0: off).
cvc::ocean::foam_persistence foam_persistence_of(cvc::app &app, const WaterRT &rt) {
  cvc::ocean::foam_persistence q;
  q.decay_s = std::max(key(app, rt.decayKey, 2.5), 0.0);
  q.bubble_decay_s = 2.4 * q.decay_s;
  return q;
}

// Start finding both thresholds for the persistent foam off the frame (OceanFoamCalibration),
// dropping any search still running. Where no task can start, they are found here.
void start_foam_calibration(cvc::app &app, WaterRT &rt) {
  if (rt.foamJob)
    rt.foamJob->cancel();
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
  rt.foamJob = OceanFoamCalibration::start(app, hs, p, foam_persistence_of(app, rt), share,
                                           "foam thresholds " + rt.node.id,
                                           "water:" + rt.prefix + "." + rt.node.id);
}

// The calibration that landed, if one has: its thresholds taken.
void follow_foam_calibration(WaterRT &rt) {
  if (!rt.foamJob || !rt.foamJob->finished())
    return;
  const cvc::ocean::foam_biases b = rt.foamJob->biases();
  rt.foamBias = b.whitecaps;
  rt.persistBias = b.persistent;
  rt.aloneBias = rt.foamJob->whitecapsAlone();
  rt.foamJob.reset();
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
  const double share = std::max(key(app, rt.capsKey, 1.0), 0.0) *
                       cvc::ocean::monahan_coverage(rt.sea ? rt.sea->to.speed_mps : s.wind_speed);
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

// A boat world vehicle's water cut-out (as BoatRuntime::cutout) and its sheer and bottom over its
// hull body's origin, from its archetype's boat at its mass: made once per archetype and mass.
const hull_box &boat_box(WaterRT &rt, const ari::vehicle_archetype &a, double mass) {
  const std::string key = a.id + "|" + std::to_string(mass);
  auto it = rt.boat_boxes.find(key);
  if (it != rt.boat_boxes.end())
    return it->second;
  hull_box out;
  try {
    const cvc::physics::dyn_boat_desc b = ari::boat_desc_of(a, mass);
    const cvc::physics::hull_mesh &h = *b.boat.hull.hull;
    double x0 = 1e300, x1 = -1e300, y = 0.0;
    out.bottom = 1e300;
    out.sheer = -1e300;
    for (const cvc::physics::vec3 &v : h.vertices) {
      x0 = std::min(x0, v.x);
      x1 = std::max(x1, v.x);
      y = std::max(y, std::fabs(v.y));
      out.bottom = std::min(out.bottom, v.z);
      out.sheer = std::max(out.sheer, v.z);
    }
    out.box = {0.5 * (x0 + x1), 0.5 * (x1 - x0), y};
    const double waterline =
        -cvc::physics::dyn_boat_plant::float_height(b.boat, b.boat.hull.mass, 1000.0);
    if (const auto c = cvc::physics::hull_waterline_box(h, waterline))
      out.box = *c;
  } catch (const std::exception &) {
  }
  return rt.boat_boxes.emplace(key, out).first->second;
}

// The hulls the water keeps out of (D5), each frame: the scene's boats (its boat nodes, and its
// world vehicles on a boat plant) nearest the camera, at most four, each where it is drawn.
void tick_hulls(WaterRT &rt, vtkRenderer *ren, RealizedScene &scene,
                const cvc::world_clock::step_result &clock, cvc::app &app) {
  auto near = rt.near.lock();
  auto far = rt.far.lock();
  if (!near || !far)
    return;
  struct hull {
    double d2, x, y, yaw, hl, hb, top, bottom;
  };
  std::vector<hull> hulls;
  double cam[3] = {rt.cx, rt.cy, 0.0};
  if (ren)
    if (vtkCamera *c = ren->GetActiveCamera())
      c->GetPosition(cam);
  const auto yaw_of = [](const cvc::physics::quat &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };
  if (scene.world && scene.physics_binding) {
    cvc::physics::pose_tracker &tracker = scene.physics_binding->tracker();
    const double a = tracker.alpha(app.world_clock(), clock.alpha);
    const cvc::physics::world &w = scene.world->world();
    for (const auto &b : scene.world->boats()) {
      const cvc::physics::body_id id = b->hull();
      if (!id.valid() || !w.contains(id))
        continue;
      std::optional<cvc::physics::pose> p = tracker.sample_one(id, a);
      if (!p) {
        const cvc::physics::body_state st = w.state(id);
        p = cvc::physics::pose{st.position, st.rotation};
      }
      // The box inside its hull at its waterline, about its centre along the hull.
      const double yaw = yaw_of(p->rotation);
      const cvc::physics::waterline_box &c = b->cutout();
      const double x = p->position.x + c.x * std::cos(yaw), y = p->position.y + c.x * std::sin(yaw);
      const double dx = x - cam[0], dy = y - cam[1];
      hulls.push_back({dx * dx + dy * dy, x, y, yaw, c.half_length, c.half_beam,
                       p->position.z + b->sheer(), p->position.z + b->bottom()});
    }
    if (const ari::WorldVehicles *v = scene.world->vehicles())
      for (const ari::entity_info &e : v->entities()) {
        if (!e.placed || !ari::vehicle_plant_floats(e.plant))
          continue;
        for (const ari::vehicle_archetype &x : v->archetypes())
          if (x.id == e.spec.archetype) {
            const hull_box &c = boat_box(rt, x, e.spec.mass);
            const double bx = e.x + c.box.x * std::cos(e.yaw), by = e.y + c.box.x * std::sin(e.yaw);
            const double dx = bx - cam[0], dy = by - cam[1];
            hulls.push_back({dx * dx + dy * dy, bx, by, e.yaw, c.box.half_length, c.box.half_beam,
                             e.z + c.sheer, e.z + c.bottom});
          }
      }
  }
  std::sort(hulls.begin(), hulls.end(), [](const hull &p, const hull &q) { return p.d2 < q.d2; });
  rt.hulls = std::min<std::size_t>(hulls.size(), 4);
  const char *pn[4] = {"uHullP0", "uHullP1", "uHullP2", "uHullP3"};
  const char *en[4] = {"uHullE0", "uHullE1", "uHullE2", "uHullE3"};
  const char *bn[4] = {"uHullB0", "uHullB1", "uHullB2", "uHullB3"};
  for (GeometryNode *g : {near.get(), far.get()})
    for (std::size_t k = 0; k < 4; ++k) {
      if (k < hulls.size()) {
        const hull &h = hulls[k];
        g->setShaderUniform3f(pn[k], static_cast<float>(h.x), static_cast<float>(h.y),
                              static_cast<float>(h.yaw));
        g->setShaderUniform3f(en[k], static_cast<float>(h.hl), static_cast<float>(h.hb),
                              static_cast<float>(h.top));
        g->setShaderUniformf(bn[k], static_cast<float>(h.bottom));
      } else {
        g->setShaderUniform3f(pn[k], 0.0f, 0.0f, 0.0f);
        g->setShaderUniform3f(en[k], 0.0f, 0.0f, 0.0f);
        g->setShaderUniformf(bn[k], 0.0f);
      }
    }
}

// The bodies' waves' window: `cells` a side of 0.5 m (adaptive quality's tier picks 256 or 128,
// the window's reach halving with it), started flat; placed again on the next frame. False (and no
// window) without float targets.
bool start_wake(WaterRT &rt, vtkOpenGLRenderWindow *ow, int cells) {
  cvc::ocean::ewave_params wp;
  wp.n = cells;
  wp.dx = 0.5;
  wp.depth = 5.0;       // until the window is placed: then its wet cells' mean depth
  wp.damping_k2 = 2e-3; // m^2/s: waves 1 m long fade over about 13 s
  wp.damping = 0.02;    // 1/s: everything, over a minute
  wp.sponge_cells = 16.0;
  wp.sponge_rate = 2.0;
  wp.beach_rate = 10.0;
  rt.wake = std::make_unique<EWave>(wp);
  rt.wakeSrc.reset();
  rt.wakePlaced = false;
  rt.wakeShore = nullptr;
  rt.wakeT = -1.0;
  if (!ow || !rt.wake->init(ow)) {
    rt.wake.reset();
    return false;
  }
  rt.wakeSrc = std::make_unique<WakeSource>(rt.wake->params());
  return true;
}

// water.profile's timings to water.stats.<section>_ms (nothing when it is off).
void publish_laps(cvc::app &app, const WaterRT &rt, const WaterLaps &laps) {
  for (const auto &[name, ms] : laps.laps)
    ari::write<double>(
        app,
        ari::resolve_bind(rt.prefix,
                          ari::node_key(rt.node, std::string("water.stats.") + name + "_ms")),
        ms);
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
  const ari::WaterQuality q = quality_of(app, rt);
  const bool on = key(app, rt.wakeKey, 1.0) > 0.5 && q.wake;
  // the tier's window size (a new window, flat; the old one's waves are dropped)
  if (on && q.wake_cells != rt.wake->params().n &&
      !start_wake(rt, vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow()), q.wake_cells))
    return;
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
  rt.wakeLaps.begin(int(key(app, rt.profileKey, 0.0)),
                    vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow()));
  if (on)
    place_wake(rt, tx, ty);
  rt.wakeLaps.lap("wake_place");
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
      for (const auto &bt : scene.world->boats()) { // the boat nodes' hulls make waves too
        const cvc::physics::body_id id = bt->hull();
        const cvc::physics::immersion_body *o =
            id.valid() ? outline_of(rt, bt->hull_desc()) : nullptr;
        if (!o || !w.contains(id))
          continue;
        std::optional<cvc::physics::pose> pose = tracker.sample_one(id, a);
        if (!pose) {
          const cvc::physics::body_state st = w.state(id);
          pose = cvc::physics::pose{st.position, st.rotation};
        }
        rt.wakeSrc->add(*o, pose->position, pose->rotation, water);
      }
    }
    rt.wakeLaps.lap("wake_sources");
    const bool added = rt.wakeSrc->finish(rt.wakeDh);
    rt.wakeLaps.lap("wake_finish");
    if (added)
      rt.wake->addHeights(rt.wakeDh);
    rt.wakeLaps.lap("wake_upload");
    if (dt > 0.0)
      rt.wake->step(dt);
    rt.wakeLaps.lap("wake_step");
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
  publish_laps(app, rt, rt.wakeLaps);
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
  rt.laps.begin(int(key(app, rt.profileKey, 0.0)),
                ren ? vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow()) : nullptr);
  if (rt.physics)
    rt.physics->tick(); // the field follows the keys and the ground (a regenerate)
  rt.laps.lap("physics_follow");
  if (!rt.ready && !rt.failed) {
    auto *ow = ren ? vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow()) : nullptr;
    if (!ow || !ow->GetInitialized())
      return;
    refresh_bed(rt);
    follow_shore(app, rt);
    follow_sea(app, rt); // the sea's first keyframe: its h0 is the cascades'
    if (!start_ocean(app, rt, ow)) {
      rt.failed = true; // no float render targets: the far grid's flat blue stays, alone
      rt.cascades.clear();
      rt.fft.reset();
      return;
    }
    // The bodies' waves, with body: (the window needs float targets too).
    if (rt.physics)
      start_wake(rt, ow, std::max(quality_of(app, rt).wake_cells, 128));
    // The persistent foam: the long and middle cascades' turbulence, in one atlas.
    if (rt.cascades.size() >= 2) {
      rt.foam = std::make_unique<OceanFoam>(kFftN, 2);
      if (!rt.foam->init(ow))
        rt.foam.reset();
    }
    style_ocean(*near, rt);
    style_ocean(*far, rt);
    if (!upload_shore(rt, ow, *near, *far)) {
      rt.failed = true;
      rt.cascades.clear();
      rt.fft.reset();
      rt.wake.reset();
      rt.wakeSrc.reset();
      return;
    }
    sync_sea(rt, ow, *near, *far, app.world_clock().t()); // its fetch maps and h0 on the GPU
    far->setShaderUniform3f("uCenter", 0.0f, 0.0f, 0.0f);
    far->setShaderUniformf("uWaves", 0.0f); // flat: its cells are metres across
    near->setShaderUniformf("uWaves", 1.0f);
    near->setShaderUniform3f("uCutMin", 1e30f, 1e30f, 0.0f); // the near grid gives way to none
    near->setShaderUniform3f("uCutMax", -1e30f, -1e30f, 0.0f);
    near->setVisible(true);
    far->setVisible(true);
    // The whitecaps' threshold: found here (0.1-0.25 s); with persistent foam both thresholds
    // are found off the frame, guesses standing until they land.
    if (rt.foam && foam_persistence_of(app, rt).decay_s > 0.0) {
      rt.foamBias = 0.75;
      start_foam_calibration(app, rt);
    } else {
      rt.foamBias = find_foam_bias(app, rt);
    }
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

  // A regenerated world (the coast below follows it, the bed's depth with it).
  if (refresh_bed(rt))
    place_far(rt, *far, *near); // the edge may have changed (world.edge), or the world's size
  // The coast: again after a regenerate or a swell change.
  follow_shore(app, rt);
  if (rt.shore != rt.shoreDrawn && ow)
    upload_shore(rt, ow, *near, *far);
  rt.laps.lap("coast");

  // Adaptive quality's tier and scale: what to draw this frame (drawing only; the physics sea is
  // the same at every tier).
  const ari::WaterQuality q = quality_of(app, rt);

  // The wind (O13): the sea's keyframes of its lagged wind, their h0 crossfading on the CPU while a
  // crossfade runs; the cascades to the world's time (the short one only when the tier draws it).
  const ari::SceneWaterSpec spec = spec_of(app, rt);
  const float chop = static_cast<float>(spec.chop);
  const double t = app.world_clock().t();
  follow_sea(app, rt);
  sync_sea(rt, ow, *near, *far, t);
  rt.fft->chop = chop;
  rt.fft->setActive(q.cascades);
  rt.fft->step(t);
  rt.laps.lap("cascades");
  const float ramp = static_cast<float>(std::max(key(app, rt.rampKey, 0.1), 1e-3));
  // The persistent foam: the long cascades' folds since the last frame, at the world's time (none
  // while it is paused; a clock run back starts it empty); emptied when it is turned off, by
  // water.foam_decay or by the tier (which leaves its calibration as it was).
  const cvc::ocean::foam_persistence fq = foam_persistence_of(app, rt);
  const bool keyPersist = rt.foam && fq.decay_s > 0.0;
  const bool persist = keyPersist && q.persistent_foam;
  if (persist) {
    const double dtf = rt.foamT < 0.0 ? 0.0 : t - rt.foamT;
    if (dtf < 0.0)
      rt.foam->clear();
    if (rt.foamT < 0.0 || dtf > 0.0) {
      rt.foam->params = fq;
      rt.foam->bias = rt.persistBias;
      rt.foam->ramp = ramp;
      rt.foam->step(rt.fft->displacement(), dtf);
    }
    rt.foamT = t;
  } else if (rt.foam && rt.foamT >= 0.0) {
    rt.foam->clear();
    rt.foamT = -1.0;
  }
  rt.laps.lap("foam");
  tick_swash(app, rt, ren, t); // the swash on the ground, at the same time as the swell
  rt.laps.lap("swash");
  // The whitecaps' threshold, once its keys have settled (a slider drag does not stall each frame).
  std::vector<double> foamNow = foam_signature(app, rt);
  if (foamNow != rt.foamSig) {
    if (foamNow != rt.foamPending) {
      rt.foamPending = std::move(foamNow);
      rt.foamSettled = 0;
    } else if (++rt.foamSettled >= kSettle) {
      if (keyPersist)
        start_foam_calibration(app, rt);
      else
        rt.foamBias = find_foam_bias(app, rt);
      rt.foamSig = rt.foamPending;
    }
  }
  follow_foam_calibration(rt);
  rt.laps.lap("thresholds");

  // The near grid follows the camera by whole cells.
  double cam[3] = {0.0, 0.0, 0.0};
  if (vtkCamera *c = ren->GetActiveCamera())
    c->GetPosition(cam);
  // (by its coarsest ring's spacing, 4 cells, so every ring's vertices land on their own lattice)
  const double snap = 4.0 * rt.cell;
  const double qx = std::round(cam[0] / snap) * snap, qy = std::round(cam[1] / snap) * snap;
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
  const float bias = static_cast<float>(
      fixedBias >= 0.0 ? fixedBias : (keyPersist && !persist ? rt.aloneBias : rt.foamBias));
  const float fray = static_cast<float>(std::max(key(app, rt.frayKey, 0.12), 0.0));
  const float shoreFoam = static_cast<float>(std::max(key(app, rt.shoreFoamKey, 1.0), 0.0));
  ari::SceneWaterSpec fadeSpec = spec; // the fades for the sea's new keyframe, as the physics sea
  if (rt.sea) {
    fadeSpec.wind_speed = rt.sea->to.speed_mps;
    fadeSpec.wind_dir = rt.sea->to.dir_rad * 180.0 / 3.14159265358979323846;
    fadeSpec.fetch = rt.sea->to.fetch_m;
  }
  const ari::SceneWaterCascades cd = ari::scene_water_cascades(fadeSpec);
  // The level of detail's distances (m), the shading's and the short cascade's scaled by the tier
  // (the persistent foam's by uFoamReach, in the shader: the whitecaps keep theirs, which cost
  // nothing), and what stands in for the foam past them: the sea's mean whitecap cover for its
  // wind. No short cascade: a negative lod_waves.
  const float lodFoam = static_cast<float>(std::max(key(app, rt.lodFoamKey, 150.0), 0.0));
  const float lodShade =
      static_cast<float>(std::max(key(app, rt.lodShadeKey, 1500.0), 0.0) * q.shade);
  const double lodWaves0 = std::max(key(app, rt.lodWavesKey, 150.0), 0.0) * q.waves;
  const float lodWaves =
      q.cascades < 3 || lodWaves0 <= 0.0 ? -1.0f : static_cast<float>(std::max(lodWaves0, 1e-3));
  const float cover = static_cast<float>(std::max(key(app, rt.capsKey, 1.0), 0.0) *
                                         cvc::ocean::monahan_coverage(fadeSpec.wind_speed));
  const double wd = fadeSpec.wind_dir * 3.14159265358979323846 / 180.0;
  // The wind's uniforms (OceanWind): the crossfade, the fetch, the gusts, the current.
  OceanWind::frame wind;
  if (rt.sea)
    wind = OceanWind::at(*ari::scene_water_forcing(spec, *rt.sea), wind_cascades(*rt.sea), t);
  near->setShaderUniformf("uFoamDetail", 1.0f);
  far->setShaderUniformf("uFoamDetail", 0.0f); // the far grid: the mean cover only
  for (GeometryNode *g : {near.get(), far.get()}) {
    OceanWind::apply(*g, wind);
    if (rt.foam)
      g->setShaderTexture("uFoamTex", rt.foam->texture()); // the atlas just written
    g->setShaderUniformf("uFoamOn", persist ? 1.0f : 0.0f);
    g->setShaderUniformf("uFoamCover", cover);
    g->setShaderUniformf("uLodFoam", std::max(lodFoam, 1e-3f));
    g->setShaderUniformf("uLodShade", std::max(lodShade, 1e-3f));
    g->setShaderUniformf("uLodWaves", lodWaves);
    g->setShaderUniformf("uFarReach", static_cast<float>(q.far_normal));
    g->setShaderUniformf("uFoamReach", static_cast<float>(q.foam));
    g->setShaderUniformf("uWindU", static_cast<float>(fadeSpec.wind_speed));
    g->setShaderUniform3f("uWindDir", static_cast<float>(std::cos(wd)),
                          static_cast<float>(std::sin(wd)), 0.0f);
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
  rt.laps.lap("uniforms");
  publish_laps(app, rt, rt.laps);
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
  // Level of detail: the near grid is three rings, its inner spacing (`cells` over `near`) out to
  // a third of the way, twice that to two thirds, four times past (ring_coords): about a third of
  // the vertices of a uniform grid.
  {
    const double q = 4.0 * rt->cell;
    rt->ring1 = q * std::ceil(rt->nearHalf / 3.0 / q);
    rt->ring2 = q * std::ceil(2.0 * rt->nearHalf / 3.0 / q);
    rt->nearHalf = q * std::ceil(rt->nearHalf / q);
  }

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
  // The persistent foam's e-folding time (s; 0: the whitecaps alone, as before O5) and the level
  // of detail's distances (m): the foam's detail and the full shading.
  rt->decayKey = seed(app, pref, n, "water.foam_decay", n.props.num("foam_decay", 2.5));
  rt->lodFoamKey = seed(app, pref, n, "water.lod_foam", n.props.num("lod_foam", 150.0));
  rt->lodShadeKey = seed(app, pref, n, "water.lod_shade", n.props.num("lod_shade", 1500.0));
  rt->profileKey = seed(app, pref, n, "water.profile", 0.0); // 1: time the node; 2: and its GPU
  rt->lodWavesKey = seed(app, pref, n, "water.lod_waves", n.props.num("lod_waves", 150.0));
  rt->qualityKey = seed(app, pref, n, "water.quality", n.props.num("quality", -1.0));
  rt->tierKey = ari::resolve_bind(pref, "quality.tier"); // the scene's (AriRuntime's loop)
  rt->scaleKey = ari::resolve_bind(pref, "quality.scale");

  const cvc::height_field &g = *rt->ground;
  rt->outer = std::max(n.props.num("outer", 6000.0), 0.0);
  rt->farCells = std::clamp(static_cast<int>(n.props.num("far_cells", 160.0)), 2, 1024);
  const double m = ari::water_rim_is_sea(g, rt->level) ? rt->outer : 0.0;
  auto far = std::dynamic_pointer_cast<GeometryNode>(
      group->createChild<GeometryNode>("far", flat_grid(app, rt->farCells, g.x0() - m, g.y0() - m,
                                                        g.max_x() + m, g.max_y() + m, rt->level)));
  auto near = std::dynamic_pointer_cast<GeometryNode>(group->createChild<GeometryNode>(
      "near", tensor_grid(app, ring_coords(rt->cell, rt->ring1, rt->ring2, rt->nearHalf),
                          ring_coords(rt->cell, rt->ring1, rt->ring2, rt->nearHalf), rt->level)));
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
      tick_hulls(*rt, ren, scene, clock, *ap);
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
