/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc. LGPL-2.1. See OceanFoam.h for the ABYSSAL (MIT) attribution that
  applies to the turbulence integration and the shading terms.
*/

#include "OceanFoam.h"

#include "OceanButterfly.h" // kOceanQuadVS, oceanBeginPass, oceanFloatTexture

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vtkOpenGLFramebufferObject.h>
#include <vtkOpenGLQuadHelper.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLShaderCache.h>
#include <vtkOpenGLState.h>
#include <vtkPixelBufferObject.h>
#include <vtkShaderProgram.h>
#include <vtkTextureObject.h>

namespace {
constexpr unsigned int kGL_SCISSOR_TEST = 0x0C11;

// One frame of the turbulence: cvc::ocean::foam_step on every texel of the atlas, tile c reading
// cascade c's Jacobian. The decay factors (uKeep = exp(-dt / decay) ...) and the gain
// (build_per_s dt) come from the CPU.
const char *const kFoamFS =
    "//VTK::System::Dec\n"
    "uniform highp sampler2D uDisp0; uniform highp sampler2D uDisp1;\n"
    "uniform highp sampler2D uDisp2; uniform highp sampler2D uPrev;\n"
    "uniform int uN;\n"
    "uniform float uBias, uRamp, uKeep, uBubbleKeep, uSprayKeep, uGain, uBubbleGain;\n"
    "//VTK::Output::Dec\n"
    "void main(){\n"
    "  ivec2 p = ivec2(gl_FragCoord.xy);\n"
    "  int c = p.x / uN;\n"
    "  ivec2 t = ivec2(p.x - c*uN, p.y);\n"
    "  float J;\n"
    "  if (c == 0) J = texelFetch(uDisp0, t, 0).w;\n"
    "  else if (c == 1) J = texelFetch(uDisp1, t, 0).w;\n"
    "  else J = texelFetch(uDisp2, t, 0).w;\n"
    "  vec4 prev = texelFetch(uPrev, p, 0);\n"
    "  float x = clamp((uBias - J)/uRamp, 0.0, 1.0);\n"
    "  float s = x*x*(3.0 - 2.0*x);\n"
    "  float foam = clamp(prev.r*uKeep + s*uGain, 0.0, 1.0);\n"
    "  float bub = clamp(prev.g*uBubbleKeep + s*uGain*uBubbleGain, 0.0, 1.0);\n"
    "  float y = clamp((s - 0.45)/0.5, 0.0, 1.0);\n"
    "  float spray = max(prev.a*uSprayKeep, y*y*(3.0 - 2.0*y));\n"
    "  gl_FragData[0] = vec4(foam, bub, s, spray);\n"
    "}\n";
} // namespace

// What the water's fragment shader reads the atlas with, and the A4 shading terms (OceanFoam.h).
// World z is up. Colours are the water shaders' display colours (no HDR, no tone map), so
// ABYSSAL's radiometric terms are scaled to them.
const char *const kOceanFoamGLSL =
    "const float O_PI = 3.14159265;\n"
    "float ofHash(vec2 p){ return fract(sin(dot(p, vec2(127.1,311.7)))*43758.5453); }\n"
    "float ofNoise(vec2 p){\n"
    "  vec2 i = floor(p), f = fract(p); f = f*f*(3.0 - 2.0*f);\n"
    "  return mix(mix(ofHash(i), ofHash(i + vec2(1,0)), f.x),\n"
    "             mix(ofHash(i + vec2(0,1)), ofHash(i + vec2(1,1)), f.x), f.y);\n"
    "}\n"
    // Tile `tile` of the atlas (n texels a side over tileM metres) at world p, bilinear by hand
    // and wrapped within the tile (texel i holds the field at i tileM / n).
    "vec4 oFoamTile(highp sampler2D atlas, int tile, int n, float tileM, vec2 p){\n"
    "  float fn = float(n);\n"
    "  vec2 f = p*(fn/tileM);\n"
    "  vec2 fl = floor(f), fr = f - fl;\n"
    "  vec2 w0 = fl - fn*floor(fl/fn);\n"
    "  ivec2 i0 = ivec2(w0);\n"
    "  ivec2 i1 = ivec2(mod(w0 + 1.0, fn));\n"
    "  int o = tile*n;\n"
    "  vec4 a = mix(texelFetch(atlas, ivec2(o + i0.x, i0.y), 0),\n"
    "               texelFetch(atlas, ivec2(o + i1.x, i0.y), 0), fr.x);\n"
    "  vec4 b = mix(texelFetch(atlas, ivec2(o + i0.x, i1.y), 0),\n"
    "               texelFetch(atlas, ivec2(o + i1.x, i1.y), 0), fr.x);\n"
    "  return mix(a, b, fr.y);\n"
    "}\n"
    // The windrow noise at world p, in [0, 1]: three scales of value noise, two in the wind's
    // frame (wd the unit direction the wind blows), long downwind: 12 m by 2.5 m and 3 m by 0.8 m;
    // and a lace 0.5 m across, which fades to its mean as the pixel's footprint fp (m) grows past
    // it, so it does not sparkle.
    "float oWindrow(vec2 p, vec2 wd, float fp){\n"
    "  vec2 q = vec2(dot(p, wd), dot(p, vec2(-wd.y, wd.x)));\n"
    "  float lace = mix(ofNoise(p*2.0 + vec2(41.0, 7.0)), 0.5, smoothstep(0.2, 0.8, fp));\n"
    "  return 0.45*ofNoise(q*vec2(1.0/12.0, 1.0/2.5))\n"
    "       + 0.30*ofNoise(q*vec2(1.0/3.0, 1.0/0.8) + vec2(17.0, 5.0)) + 0.25*lace;\n"
    "}\n"
    // cvc::ocean::foam_look: fresh foam is laced, aging foam thins into the windrows' streaks.
    "float oFoamLook(float f, float w){\n"
    "  float x = clamp((f*(0.1 + 1.6*w) - 0.35)/0.3, 0.0, 1.0);\n"
    "  return x*x*(3.0 - 2.0*x);\n"
    "}\n"
    // GGX (Trowbridge-Reitz), height-correlated Smith visibility, Schlick's Fresnel for water
    // (F0 0.02) losing its grazing peak as the surface roughens.
    "float oGgxD(float NoH, float a){\n"
    "  float a2 = a*a; float d = (NoH*a2 - NoH)*NoH + 1.0;\n"
    "  return a2/max(O_PI*d*d, 1e-8);\n"
    "}\n"
    "float oSmith(float NoV, float NoL, float a){\n"
    "  float a2 = a*a;\n"
    "  float gv = NoL*sqrt(NoV*NoV*(1.0 - a2) + a2), gl = NoV*sqrt(NoL*NoL*(1.0 - a2) + a2);\n"
    "  return 0.5/max(gv + gl, 1e-6);\n"
    "}\n"
    "float oFresnel(float NoV, float rough){\n"
    "  float f = 0.02 + 0.98*pow(clamp(1.0 - NoV, 0.0, 1.0), 5.0);\n"
    "  return mix(f, clamp(f*0.72 + 0.06, 0.0, 1.0), clamp(rough*1.5, 0.0, 1.0));\n"
    "}\n"
    // The mean square slope the normal does not resolve: Cox-Munk's for the wind,
    // 0.003 + 0.00512 U, shared over the cascades (0.06, 0.30, 0.64); a cascade's share is lost
    // as the footprint fp (m) covers more of its texels (log2 of the texels in it, over 6
    // octaves), and wholly where the normal leaves it out (inNormal 0). The GGX alpha of a mean
    // square slope m is sqrt(2 (m + 0.0009)), within [0.012, 0.62] (ABYSSAL's).
    "float oUnresolvedMss(float U, float fp, vec3 tiles, float texels, vec3 inNormal){\n"
    "  float mss = 0.003 + 0.00512*max(U, 0.5);\n"
    "  vec3 lod = log2(max(vec3(fp)*texels/tiles, vec3(1.0)));\n"
    "  vec3 lost = mix(vec3(1.0), clamp(lod/6.0, 0.0, 1.0), clamp(inNormal, 0.0, 1.0));\n"
    "  return mss*dot(vec3(0.06, 0.30, 0.64), lost);\n"
    "}\n"
    "float oAlpha(float mss){ return clamp(sqrt(2.0*(mss + 0.0009)), 0.012, 0.62); }\n"
    // Ripples the normal leaves out, for the sun to glitter on: six short gravity waves (1.7 m
    // down to 0.43 m, spread about the wind wd, each moving at its own speed: sqrt(g k)), of equal
    // steepness, scaled to an RMS slope of sigma on each axis (six unit slopes in spread
    // directions: about sqrt(6 / 4)). Waves rather than a noise's gradient: a lattice's cells print
    // a crosshatch on the glitter. Returns the slope (dz/dx, dz/dy).
    "vec2 oMicroSlope(vec2 p, vec2 wd, float t, float sigma){\n"
    "  const float L[6] = float[6](1.7, 1.23, 0.93, 0.71, 0.55, 0.43);\n"
    "  const float A[6] = float[6](-0.87, 0.35, -0.26, 0.79, -0.52, 0.17);\n" // radians off wd
    "  const float P[6] = float[6](0.0, 2.1, 4.4, 1.3, 5.2, 3.0);\n"
    "  vec2 s = vec2(0.0);\n"
    "  for (int i = 0; i < 6; ++i) {\n"
    "    float c = cos(A[i]), sn = sin(A[i]);\n"
    "    vec2 d = vec2(c*wd.x - sn*wd.y, sn*wd.x + c*wd.y);\n"
    "    float k = 6.2831853/L[i];\n"
    "    s += d*cos(k*dot(d, p) - sqrt(9.81*k)*t + P[i]);\n"
    "  }\n"
    "  return s*(sigma/1.2247449);\n"
    "}\n"
    // The sun's GGX lobe (N, V, L unit; z up), widened by the sun's disc (0.00465 rad radius).
    "float oSunSpec(vec3 N, vec3 V, vec3 L, float a){\n"
    "  float NoL = dot(N, L);\n"
    "  if (NoL <= 0.0) return 0.0;\n"
    "  vec3 H = normalize(L + V);\n"
    "  float NoH = max(dot(N, H), 0.0), VoH = max(dot(V, H), 1e-4), NoV = max(dot(N, V), 1e-4);\n"
    "  float aP = clamp(a + 0.00465*0.5, 0.0, 1.0);\n"
    "  float D = oGgxD(NoH, aP)*(a*a)/(aP*aP);\n"
    "  float Fs = 0.02 + 0.98*pow(1.0 - VoH, 5.0);\n"
    "  return D*oSmith(NoV, NoL, a)*Fs*NoL;\n"
    "}\n"
    // What leaves the water body: the deep water lit by the sun (sunShade its share through the
    // clouds) and the sky, the light through backlit crests (crestH the wave's height over the
    // still level, m: a thin crest with the sun behind it glows), and the bubbles a breaker left.
    "vec3 oBody(vec3 N, vec3 V, vec3 L, float sunShade, float crestH, float bubbles){\n"
    "  vec3 col = vec3(0.05, 0.22, 0.38)*(0.30 + 0.70*max(dot(N, L), 0.0)*sunShade);\n"
    "  float back = clamp(crestH*0.35 + 0.35, 0.0, 1.5)*pow(clamp(dot(L, -V), 0.0, 1.0), 4.0)\n"
    "             * pow(0.5 - 0.5*dot(L, N), 3.0);\n"
    "  col += vec3(0.10, 0.55, 0.50)*back*1.7*sunShade/(1.0 + max(0.0, -L.z)*4.0);\n"
    "  col += vec3(0.12, 0.30, 0.32)*bubbles*(0.55 + 0.45*sunShade);\n"
    "  return col;\n"
    "}\n"
    // The gradient sky the water reflects (no environment map yet), toward R.
    "vec3 oSkyAt(vec3 R){\n"
    "  return mix(vec3(0.66,0.72,0.78), vec3(0.26,0.50,0.80), clamp(0.5 + 0.5*R.z, 0.0, 1.0));\n"
    "}\n"
    // The O15 water (lsystem_coast's): a fixed Fresnel, a Phong glint, the deep colour lit.
    "vec3 oWaterSimple(vec3 N, vec3 V, vec3 L, float sunShade){\n"
    "  float NoV = max(dot(N, V), 0.0);\n"
    "  vec3 R = 2.0*NoV*N - V;\n"
    "  float Fr = 0.02 + 0.98*pow(1.0 - NoV, 5.0);\n"
    "  float lit = 0.30 + 0.70*max(dot(N, L), 0.0)*sunShade;\n"
    "  float glint = pow(max(dot(R, L), 0.0), 200.0)*1.8*sunShade;\n"
    "  return mix(vec3(0.05,0.22,0.38)*lit, oSkyAt(R), Fr) + glint*vec3(1.0,0.96,0.88);\n"
    "}\n"
    // The A4 water. The slopes the normal N leaves out (the cascades' weights in it, inN; the
    // spacing of what it was differenced over, nstep) or the footprint fp covers are Cox-Munk's
    // for the wind U: ripples the pixel still resolves go into the normal for the sun to glitter
    // on (oMicroSlope at p, fading out as the pixel grows past them), the rest widens the sun's
    // lobe; Fresnel and the sky's reflection soften with the roughness; the body takes the light
    // through backlit crests (crestH, m) and the bubbles; the sun's highlight rolls off rather
    // than clipping (the colours are not HDR).
    "vec3 oWaterFull(vec3 N, vec3 V, vec3 L, float sunShade, float crestH, float bubbles,\n"
    "                float U, float fp, float nstep, vec3 tiles, float texels, vec3 inN,\n"
    "                vec2 p, vec2 wd, float t){\n"
    "  float mss = oUnresolvedMss(U, max(fp, nstep), tiles, texels, inN);\n"
    "  float micro = 1.0 - smoothstep(0.12, 0.5, fp);\n"
    "  vec3 Nf = N;\n"
    "  if (micro > 0.0)\n" // a pixel past the ripples skips them
    "    Nf = normalize(vec3(N.xy/max(N.z, 1e-3) - oMicroSlope(p, wd, t, sqrt(0.3*mss*micro)), "
    "1.0));\n"
    "  float a = oAlpha(mss*(1.0 - 0.6*micro));\n"
    "  float rough = sqrt(a);\n"
    "  float NoV = max(dot(Nf, V), 0.0);\n"
    "  vec3 R = 2.0*NoV*Nf - V;\n"
    "  vec3 sky = mix(oSkyAt(R), vec3(0.50,0.62,0.78), clamp(rough*0.8, 0.0, 1.0));\n"
    "  vec3 body = oBody(Nf, V, L, sunShade, crestH, bubbles);\n"
    "  float s = oSunSpec(Nf, V, L, a)*sunShade;\n"
    "  return mix(body, sky, oFresnel(NoV, rough)) + vec3(1.0,0.96,0.88)*(2.6*s/(1.0 + 0.8*s));\n"
    "}\n"
    // Matte foam: lit from all round (wrapped), dimmed in the clouds' shade as the full shading
    // comes in (sd 1), the O15 flat white without it (sd 0).
    "vec3 oFoamColour(vec3 N, vec3 L, float sunShade, float sd){\n"
    "  float wrapL = clamp((dot(N, L) + 0.45)/1.45, 0.0, 1.0);\n"
    "  return vec3(0.93,0.96,0.97)*mix(1.0, 0.62 + 0.38*wrapL*sunShade, sd);\n"
    "}\n";

const char *oceanFoamGLSL() { return kOceanFoamGLSL; }

OceanFoam::OceanFoam(int n, int tiles) : m_n(n), m_tiles(std::clamp(tiles, 1, 3)) {}
OceanFoam::~OceanFoam() { releaseResources(); }

bool OceanFoam::init(vtkOpenGLRenderWindow *rw) {
  if (!rw)
    return false;
  m_rw = rw;
  m_rw->MakeCurrent();
  const std::vector<float> zeros(std::size_t(m_n) * m_tiles * m_n * 4, 0.0f);
  for (auto &a : m_atlas) {
    a = oceanFloatTexture(m_rw, m_n * m_tiles, m_n, 4, zeros.data());
    if (!a) {
      std::fprintf(stderr, "OceanFoam: RGBA32F texture allocation failed.\n");
      return false;
    }
  }
  m_fbo = vtkSmartPointer<vtkOpenGLFramebufferObject>::New();
  m_fbo->SetContext(m_rw);
  m_pass = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kFoamFS, nullptr);
  if (!m_pass->Program) {
    std::fprintf(stderr, "OceanFoam: the turbulence pass failed to build.\n");
    return false;
  }
  m_cur = 0;
  m_ready = true;
  return true;
}

void OceanFoam::clear() {
  if (!m_ready)
    return;
  m_rw->MakeCurrent();
  std::vector<float> zeros(std::size_t(m_n) * m_tiles * m_n * 4, 0.0f);
  for (auto &a : m_atlas)
    a->Create2DFromRaw(m_n * m_tiles, m_n, 4, VTK_FLOAT, zeros.data());
}

void OceanFoam::step(const std::vector<vtkTextureObject *> &disp, double dt_s) {
  if (!m_ready || disp.size() < std::size_t(m_tiles))
    return;
  for (int c = 0; c < m_tiles; ++c)
    if (!disp[std::size_t(c)])
      return;
  m_rw->MakeCurrent();
  const double dt = std::clamp(dt_s, 0.0, std::max(params.max_dt_s, 0.0));
  auto keep = [dt](double tau) { return tau > 0.0 ? std::exp(-dt / tau) : 0.0; };
  vtkOpenGLState::ScopedglEnableDisable scissor(m_rw->GetState(), kGL_SCISSOR_TEST);
  m_rw->GetState()->PushFramebufferBindings();
  const int dst = 1 - m_cur;
  oceanBeginPass(m_rw, m_fbo, m_atlas[dst], m_n); // the viewport is set below (the atlas is wide)
  m_rw->GetState()->vtkglViewport(0, 0, m_n * m_tiles, m_n);
  vtkShaderProgram *p = m_pass->Program;
  m_rw->GetShaderCache()->ReadyShaderProgram(p);
  vtkTextureObject *src = m_atlas[m_cur];
  src->Activate();
  for (int c = 0; c < m_tiles; ++c)
    disp[std::size_t(c)]->Activate();
  const char *names[3] = {"uDisp0", "uDisp1", "uDisp2"};
  for (int c = 0; c < 3; ++c) // unused slots read the first cascade (never sampled)
    p->SetUniformi(names[c], disp[std::size_t(std::min(c, m_tiles - 1))]->GetTextureUnit());
  p->SetUniformi("uPrev", src->GetTextureUnit());
  p->SetUniformi("uN", m_n);
  p->SetUniformf("uBias", static_cast<float>(bias));
  p->SetUniformf("uRamp", static_cast<float>(std::max(ramp, 1e-6)));
  p->SetUniformf("uKeep", static_cast<float>(keep(params.decay_s)));
  p->SetUniformf("uBubbleKeep", static_cast<float>(keep(params.bubble_decay_s)));
  p->SetUniformf("uSprayKeep", static_cast<float>(keep(params.spray_decay_s)));
  p->SetUniformf("uGain", static_cast<float>(std::max(params.build_per_s, 0.0) * dt));
  p->SetUniformf("uBubbleGain", static_cast<float>(params.bubble_gain));
  m_pass->Render();
  for (int c = 0; c < m_tiles; ++c)
    disp[std::size_t(c)]->Deactivate();
  src->Deactivate();
  m_rw->GetState()->PopFramebufferBindings();
  m_cur = dst;
  ++m_passes;
}

vtkTextureObject *OceanFoam::texture() const { return m_ready ? m_atlas[m_cur].Get() : nullptr; }

std::vector<float> OceanFoam::readback() const {
  std::vector<float> out;
  if (!m_ready)
    return out;
  m_rw->MakeCurrent();
  vtkSmartPointer<vtkPixelBufferObject> pbo;
  pbo.TakeReference(m_atlas[m_cur]->Download());
  if (!pbo)
    return out;
  auto *data = static_cast<float *>(pbo->MapPackedBuffer());
  if (data) {
    out.assign(data, data + std::size_t(m_n) * m_tiles * m_n * 4);
    pbo->UnmapPackedBuffer();
  }
  return out;
}

void OceanFoam::releaseResources() {
  m_pass.reset();
  m_fbo = nullptr;
  m_atlas[0] = m_atlas[1] = nullptr;
  m_ready = false;
}

// ── the thresholds, off the frame ────────────────────────────────────────────────────────────

namespace {
constexpr const char *kJacobians = "jacobians";
constexpr const char *kBiases = "biases";
} // namespace

OceanFoamCalibration::OceanFoamCalibration(const std::vector<cvc::ocean::h0_field> &hs,
                                           const cvc::ocean::foam_params &p,
                                           const cvc::ocean::foam_persistence &q, double share)
    : m_h(hs, p, q), m_share(share) {}

bool OceanFoamCalibration::step(std::size_t budget) {
  if (m_done.load(std::memory_order_acquire))
    return true;
  if (m_cancel.load(std::memory_order_acquire)) {
    m_done.store(true, std::memory_order_release);
    return true;
  }
  if (!m_h.ready()) {
    m_h.step(std::max<std::size_t>(budget, 1));
    m_progress.store(double(m_h.units_done()) / double(m_h.units() + 1), std::memory_order_release);
    m_phase.store(m_h.ready() ? 1 : 0, std::memory_order_release);
    return false;
  }
  m_biases = m_h.biases_for(m_share);
  m_finished.store(true, std::memory_order_release);
  m_done.store(true, std::memory_order_release);
  return true;
}

const char *OceanFoamCalibration::phase() const {
  return m_phase.load(std::memory_order_acquire) ? kBiases : kJacobians;
}

double OceanFoamCalibration::progress() const {
  return m_finished.load(std::memory_order_acquire) ? 1.0
                                                    : m_progress.load(std::memory_order_acquire);
}

std::string OceanFoamCalibration::status() const {
  return m_finished.load(std::memory_order_acquire) ? "foam thresholds found"
                                                    : "finding the foam's thresholds";
}

std::shared_ptr<OceanFoamCalibration>
OceanFoamCalibration::start(cvc::app &app, const std::vector<cvc::ocean::h0_field> &hs,
                            const cvc::ocean::foam_params &p, const cvc::ocean::foam_persistence &q,
                            double share, const std::string &subject, const std::string &owner) {
  std::shared_ptr<OceanFoamCalibration> job;
  try {
    job = std::make_shared<OceanFoamCalibration>(hs, p, q, share);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "OceanFoam: no calibration (%s)\n", e.what());
    return nullptr;
  }
  cvc::sliced_task_options o;
  o.kind = "ocean";
  o.subject = subject;
  o.owner = owner;
  if (cvc::schedule_sliced_job(app, job, o) < 0)
    while (!job->step(0)) {
    }
  return job;
}
