/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "EWave.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
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
constexpr double kTwoPi = 6.283185307179586;

// ── the update pass: separate H and PHI at k and -k, move them on, recombine ──
// The same arithmetic as ewave::prepare and ewave::step's mode loop (cvc/ocean/ewave.cpp), with
// the coefficients computed here per texel. Index p is wavenumber p (p < n / 2) or p - n, times
// uDk = 2 pi / (n dx); the mirror of p is (n - p) mod n, so the Nyquist index n / 2 pairs with
// itself.
const char *kUpdateFS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uZ;\n"
    "uniform int uN;\n"
    "uniform float uDk, uDepth, uG, uDt, uGamma, uDamping;\n"
    "//VTK::Output::Dec\n"
    "void main(){\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  ivec2 m = (ivec2(uN) - c) & ivec2(uN - 1);\n"
    "  vec2 z = texelFetch(uZ, c, 0).rg, w = texelFetch(uZ, m, 0).rg;\n"
    // Z(k) = H(k) + i PHI(k) and Z(-k) = conj H(k) + i conj PHI(k), as h and phi are real.
    "  vec2 H = 0.5 * vec2(z.x + w.x, z.y - w.y);\n"
    "  vec2 P = 0.5 * vec2(z.y + w.y, w.x - z.x);\n"
    "  vec2 q = vec2(c.x < uN / 2 ? c.x : c.x - uN, c.y < uN / 2 ? c.y : c.y - uN);\n"
    "  vec2 kv = uDk * q;\n"
    "  float k2 = dot(kv, kv);\n"
    "  float damp = exp(-(uGamma * k2 + uDamping) * uDt);\n"
    "  vec4 a = vec4(damp, 0.0, 0.0, 0.0);\n" // k = 0: the mean height stays, the mean phi is 0
    "  if (k2 > 0.0) {\n"
    "    float k = sqrt(k2);\n"
    "    float kap = k * tanh(min(k * uDepth, 20.0));\n"
    "    float om = sqrt(uG * kap), ph = om * uDt;\n"
    "    float cs = cos(ph), sn = sin(ph);\n"
    "    a = vec4(cs, (kap / om) * sn, -(uG / om) * sn, cs) * damp;\n"
    "  }\n"
    "  vec2 H2 = a.x * H + a.y * P, P2 = a.z * H + a.w * P;\n"
    "  gl_FragData[0] = vec4(H2.x - P2.y, H2.y + P2.x, 0.0, 0.0);\n"
    "}\n";

// ── the real-space pass: normalise, the mask's beach and the sponge, land to 0 ──
// As ewave::step's real-space factor: a rate per cell applied as exp(-rate dt). The exact step
// (uSym 0) normalises by uScale = 1 / n^2 and damps h and phi by f; the symmetric step (uSym 1)
// damps h by f^2 and both by uScale = exp(-damping dt), and zeroes phi on walls.
const char *kRealFS = "//VTK::System::Dec\n"
                      "uniform sampler2D uZ, uMask;\n"
                      "uniform int uN, uSym;\n"
                      "uniform float uScale, uDt, uSpongeW, uSpongeRate, uBeachRate;\n"
                      "//VTK::Output::Dec\n"
                      "void main(){\n"
                      "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
                      "  vec2 z = texelFetch(uZ, c, 0).rg * uScale;\n"
                      "  float m = texelFetch(uMask, c, 0).r;\n"
                      "  float rate = m < 1.0 ? uBeachRate * (1.0 - m) : 0.0;\n"
                      "  float d = float(min(min(c.x, uN - 1 - c.x), min(c.y, uN - 1 - c.y)));\n"
                      "  if (uSpongeW > 0.0 && d < uSpongeW) {\n"
                      "    float r = (uSpongeW - d) / uSpongeW;\n"
                      "    rate += uSpongeRate * (r * r);\n"
                      "  }\n"
                      "  float f = m <= 0.0 ? 0.0 : (rate > 0.0 ? exp(-rate * uDt) : 1.0);\n"
                      "  vec2 o = uSym != 0 ? vec2(z.x * (f * f), f > 0.0 ? z.y : 0.0) : z * f;\n"
                      "  gl_FragData[0] = vec4(o, 0.0, 0.0);\n"
                      "}\n";

// ── a source pass: h += (a drop's bump + a field) * mask ──────────────────────
// The bump is ewave::add_height's, amount (1 - r^2 / R^2)^3, with r and R in cells.
const char *kSourceFS = "//VTK::System::Dec\n"
                        "uniform sampler2D uZ, uMask, uSrc;\n"
                        "uniform int uDrop, uField;\n"
                        "uniform vec2 uCentre;\n"
                        "uniform float uR2, uAmount;\n"
                        "//VTK::Output::Dec\n"
                        "void main(){\n"
                        "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
                        "  vec2 z = texelFetch(uZ, c, 0).rg;\n"
                        "  float add = 0.0;\n"
                        "  if (uDrop != 0) {\n"
                        "    vec2 d = vec2(c) - uCentre;\n"
                        "    float w = 1.0 - dot(d, d) / uR2;\n"
                        "    if (w > 0.0) add += uAmount * (w * w * w);\n"
                        "  }\n"
                        "  if (uField != 0) add += texelFetch(uSrc, c, 0).r;\n"
                        "  gl_FragData[0] = vec4(z.x + add * texelFetch(uMask, c, 0).r, z.y, 0.0, "
                        "0.0);\n"
                        "}\n";

// ── the fill pass: R, the extended field over the window ──────────────────────
// As ewave::fill: a water texel (1 / M above 0) writes its own value v, a ghost texel (uCorner
// its lower corner (i0, j0), -1 elsewhere) the weighted sum of its corners' v (uW: the weights of
// (i0, j0), (i0 + 1, j0), (i0, j0 + 1), (i0 + 1, j0 + 1), 0 on wall corners), other walls 0.
// v is phi_half = phi - uHalf h (uMode 0, a substep) or h (uMode 1, damping_k2's filter).
const char *kFillFS = "//VTK::System::Dec\n"
                      "uniform sampler2D uZ, uCorner, uW, uInvM;\n"
                      "uniform int uMode;\n"
                      "uniform float uHalf;\n"
                      "//VTK::Output::Dec\n"
                      "float val(ivec2 p) {\n"
                      "  vec2 z = texelFetch(uZ, p, 0).rg;\n"
                      "  return uMode == 0 ? z.y - uHalf * z.x : z.x;\n"
                      "}\n"
                      "void main(){\n"
                      "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
                      "  float v = 0.0;\n"
                      "  if (texelFetch(uInvM, c, 0).r > 0.0) {\n"
                      "    v = val(c);\n"
                      "  } else {\n"
                      "    vec2 b = texelFetch(uCorner, c, 0).rg;\n"
                      "    if (b.x >= 0.0) {\n"
                      "      ivec2 o = ivec2(b);\n"
                      "      vec4 w = texelFetch(uW, c, 0);\n"
                      "      v = w.x * val(o) + w.y * val(o + ivec2(1, 0))\n"
                      "        + w.z * val(o + ivec2(0, 1)) + w.w * val(o + ivec2(1, 1));\n"
                      "    }\n"
                      "  }\n"
                      "  gl_FragData[0] = vec4(v, 0.0, 0.0, 0.0);\n"
                      "}\n";

// ── the multiply pass: G~ (uMode 0) or damping_k2's F (uMode 1), and 1 / n^2 ───
// kappa~ = 4 sin^2(omega dt_s / 2) / (g dt_s^2) as ewave::prepare_symmetric; F = exp(-uRate k^2),
// uRate = 2 damping_k2 dt. The field is the transform of a real one and the multipliers are real
// and even, so it stays Hermitian.
const char *kMultiplyFS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uZ;\n"
    "uniform int uN, uMode;\n"
    "uniform float uDk, uDepth, uG, uDts, uRate, uScale;\n"
    "//VTK::Output::Dec\n"
    "void main(){\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  vec2 z = texelFetch(uZ, c, 0).rg;\n"
    "  vec2 q = vec2(c.x < uN / 2 ? c.x : c.x - uN, c.y < uN / 2 ? c.y : c.y - uN);\n"
    "  vec2 kv = uDk * q;\n"
    "  float k2 = dot(kv, kv);\n"
    "  float m = 0.0;\n"
    "  if (uMode == 0) {\n"
    "    if (k2 > 0.0) {\n"
    "      float k = sqrt(k2);\n"
    "      float kap = k * tanh(min(k * uDepth, 20.0));\n"
    "      float s = sin(0.5 * sqrt(uG * kap) * uDts);\n"
    "      m = 4.0 * s * s / (uG * uDts * uDts);\n"
    "    }\n"
    "  } else {\n"
    "    m = exp(-uRate * k2);\n"
    "  }\n"
    "  gl_FragData[0] = vec4(z * (m * uScale), 0.0, 0.0);\n"
    "}\n";

// ── the gather pass: R^T, 1 / M, and the update ──────────────────────────────
// As ewave::fold and the substep's last loop: a water texel (1 / M above 0) adds to its own value
// of uS (G~ R v, the real part) each reader's weight times uS at the reader's cell (uR*: (cell,
// w, cell, w), cell a float index j n + i, w 0 for an empty slot), in the CPU table's order. Then
// uMode 0: h' = h + uDts M^-1 acc, phi' = phi_half - uHalf h'; uMode 1: h' = M^-1 acc, phi
// unchanged. Walls write 0. The reader textures' declarations and reads are generated (one pair
// of readers per texture), since GLSL ES 3.0 cannot index a sampler array by a loop variable.
std::string gatherFS() {
  const int nr = EWave::kReaderTextures;
  std::string s = "//VTK::System::Dec\n"
                  "uniform sampler2D uZ, uS, uInvM";
  for (int r = 0; r < nr; ++r)
    s += ", uR" + std::to_string(r);
  s += ";\n"
       "uniform int uN, uMode;\n"
       "uniform float uHalf, uDts;\n"
       "//VTK::Output::Dec\n"
       "float sAt(float cell) {\n"
       "  int i = int(cell + 0.5);\n"
       "  return texelFetch(uS, ivec2(i % uN, i / uN), 0).r;\n"
       "}\n"
       "void main(){\n"
       "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
       "  float im = texelFetch(uInvM, c, 0).r;\n"
       "  vec4 o = vec4(0.0);\n"
       "  if (im > 0.0) {\n"
       "    float acc = texelFetch(uS, c, 0).r;\n"
       "    vec4 r;\n";
  for (int r = 0; r < nr; ++r)
    s += "    r = texelFetch(uR" + std::to_string(r) +
         ", c, 0);\n"
         "    if (r.y != 0.0) acc += r.y * sAt(r.x);\n"
         "    if (r.w != 0.0) acc += r.w * sAt(r.z);\n";
  s += "    vec2 z = texelFetch(uZ, c, 0).rg;\n"
       "    if (uMode == 0) {\n"
       "      float ph = z.y - uHalf * z.x;\n"
       "      float h = z.x + uDts * (im * acc);\n"
       "      o = vec4(h, ph - uHalf * h, 0.0, 0.0);\n"
       "    } else {\n"
       "      o = vec4(im * acc, z.y, 0.0, 0.0);\n"
       "    }\n"
       "  }\n"
       "  gl_FragData[0] = o;\n"
       "}\n";
  return s;
}

bool finite_value(double v) { return std::isfinite(v); }

// ewave's own checks (cvc/ocean/ewave.cpp), so the two windows take the same parameters.
cvc::ocean::ewave_params checked(const cvc::ocean::ewave_params &p) {
  if (p.n < 4 || p.n > 4096 || (p.n & (p.n - 1)) != 0)
    throw std::invalid_argument("EWave: n must be a power of two in 4..4096, got " +
                                std::to_string(p.n));
  if (!(p.dx > 0.0) || !finite_value(p.dx) || !finite_value(p.origin_x) ||
      !finite_value(p.origin_y))
    throw std::invalid_argument("EWave: dx must be finite and above 0, the origin finite");
  if (!(p.depth > 0.0) || !(p.gravity > 0.0) || !finite_value(p.gravity) || !(p.density > 0.0) ||
      !finite_value(p.density))
    throw std::invalid_argument("EWave: depth, gravity and density must be above 0 (gravity and "
                                "density finite)");
  if (!(p.damping_k2 >= 0.0) || !finite_value(p.damping_k2) || !(p.damping >= 0.0) ||
      !finite_value(p.damping) || !(p.sponge_rate >= 0.0) || !finite_value(p.sponge_rate) ||
      !(p.beach_rate >= 0.0) || !finite_value(p.beach_rate))
    throw std::invalid_argument("EWave: damping_k2, damping, sponge_rate and beach_rate must be "
                                "finite and >= 0");
  if (!(p.sponge_cells >= 0.0) || !(p.sponge_cells <= 0.5 * p.n))
    throw std::invalid_argument("EWave: sponge_cells must be in [0, n / 2]");
  if (p.walls != cvc::ocean::ewave_walls::no_flux && p.walls != cvc::ocean::ewave_walls::zero)
    throw std::invalid_argument("EWave: walls must be no_flux or zero");
  if (p.ghost_cells < 0 || p.ghost_cells > 4096)
    throw std::invalid_argument("EWave: ghost_cells must be in [0, 4096]");
  return p;
}

// Saves the scissor test and the framebuffer bindings, and restores them when it goes.
struct PassScope {
  vtkOpenGLRenderWindow *rw;
  vtkOpenGLState::ScopedglEnableDisable scissor;
  explicit PassScope(vtkOpenGLRenderWindow *w)
      : rw((w->MakeCurrent(), w)), scissor(w->GetState(), kGL_SCISSOR_TEST) {
    rw->GetState()->PushFramebufferBindings();
  }
  ~PassScope() { rw->GetState()->PopFramebufferBindings(); }
};

std::vector<float> download(vtkTextureObject *t, std::size_t count) {
  std::vector<float> out;
  vtkSmartPointer<vtkPixelBufferObject> pbo;
  pbo.TakeReference(t->Download());
  if (!pbo)
    return out;
  auto *data = static_cast<float *>(pbo->MapPackedBuffer());
  if (data) {
    out.assign(data, data + count);
    pbo->UnmapPackedBuffer();
  }
  return out;
}

} // namespace

EWave::EWave(const cvc::ocean::ewave_params &p) : m_p(checked(p)) {
  m_log2n = static_cast<int>(std::lround(std::log2(static_cast<double>(m_p.n))));
  const std::size_t cells = static_cast<std::size_t>(m_p.n) * m_p.n;
  m_maskHost.assign(cells, 1.0f);
  m_ghostCornerHost.assign(cells * 2, -1.0f);
  m_ghostWHost.assign(cells * 4, 0.0f);
  m_invMassHost.assign(cells, 0.0f);
  m_readerHost.assign(cells * 4 * kReaderTextures, 0.0f);
}

EWave::~EWave() { releaseResources(); }

bool EWave::init(vtkOpenGLRenderWindow *rw) {
  releaseResources();
  if (!rw)
    return false;
  m_rw = rw;
  m_rw->MakeCurrent();
  const int n = m_p.n;
  const std::size_t cells = static_cast<std::size_t>(n) * n;
  const std::vector<float> zeros(cells * 2, 0.0f);
  for (auto &z : m_z)
    if (!(z = oceanFloatTexture(m_rw, n, n, 2, zeros.data()))) {
      std::fprintf(stderr, "EWave: RG32F texture allocation failed — float RTs unsupported.\n");
      return false;
    }
  m_mask = oceanFloatTexture(m_rw, n, n, 1, m_maskHost.data());
  m_src = oceanFloatTexture(m_rw, n, n, 1, zeros.data());
  m_ghostCorner = oceanFloatTexture(m_rw, n, n, 2, m_ghostCornerHost.data());
  m_ghostW = oceanFloatTexture(m_rw, n, n, 4, m_ghostWHost.data());
  m_invMass = oceanFloatTexture(m_rw, n, n, 1, m_invMassHost.data());
  bool readers = true;
  for (int r = 0; r < kReaderTextures; ++r)
    readers = readers && (m_readers[r] = oceanFloatTexture(m_rw, n, n, 4,
                                                           m_readerHost.data() + cells * 4 * r));
  if (!m_mask || !m_src || !m_ghostCorner || !m_ghostW || !m_invMass || !readers ||
      !m_fft.init(m_rw, n)) {
    std::fprintf(stderr, "EWave: a float texture or the FFT failed to allocate.\n");
    releaseResources();
    return false;
  }
  m_fbo = vtkSmartPointer<vtkOpenGLFramebufferObject>::New();
  m_fbo->SetContext(m_rw);
  const std::string gather = gatherFS();
  m_update = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kUpdateFS, nullptr);
  m_real = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kRealFS, nullptr);
  m_source = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kSourceFS, nullptr);
  m_fill = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kFillFS, nullptr);
  m_multiply = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kMultiplyFS, nullptr);
  m_gather = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, gather.c_str(), nullptr);
  if (!m_update->Program || !m_real->Program || !m_source->Program || !m_fill->Program ||
      !m_multiply->Program || !m_gather->Program) {
    std::fprintf(stderr, "EWave: a shader program failed to build.\n");
    releaseResources();
    return false;
  }
  m_state = 0;
  m_time = 0.0;
  m_ready = true;
  return true;
}

void EWave::releaseResources() {
  m_update.reset();
  m_real.reset();
  m_source.reset();
  m_fill.reset();
  m_multiply.reset();
  m_gather.reset();
  m_fft.release();
  m_fbo = nullptr;
  for (auto &z : m_z)
    z = nullptr;
  m_mask = m_src = m_ghostCorner = m_ghostW = m_invMass = nullptr;
  for (auto &r : m_readers)
    r = nullptr;
  m_ready = false;
}

void EWave::setMask(const std::vector<double> &mask) {
  if (mask.size() != m_maskHost.size())
    throw std::invalid_argument("EWave: the mask must have n x n values");
  for (double m : mask)
    if (!(m >= 0.0) || !(m <= 1.0))
      throw std::invalid_argument("EWave: mask values must be in [0, 1]");
  const std::size_t cells = mask.size(), n = static_cast<std::size_t>(m_p.n);
  for (std::size_t c = 0; c < cells; ++c)
    m_maskHost[c] = static_cast<float>(mask[c]);
  // No-flux walls: the CPU window's own ghost table and lumped mass (computed in double).
  std::fill(m_ghostCornerHost.begin(), m_ghostCornerHost.end(), -1.0f);
  std::fill(m_ghostWHost.begin(), m_ghostWHost.end(), 0.0f);
  std::fill(m_invMassHost.begin(), m_invMassHost.end(), 0.0f);
  std::fill(m_readerHost.begin(), m_readerHost.end(), 0.0f);
  m_ghosts = 0;
  if (m_p.walls == cvc::ocean::ewave_walls::no_flux) {
    const std::vector<cvc::ocean::ewave_ghost> ghosts =
        cvc::ocean::ewave_wall_ghosts(m_p.n, mask, m_p.ghost_cells);
    if (!ghosts.empty()) {
      const std::vector<double> mass = cvc::ocean::ewave_lumped_mass(m_p.n, mask, ghosts);
      for (std::size_t c = 0; c < cells; ++c)
        if (mask[c] > 0.0)
          m_invMassHost[c] = static_cast<float>(1.0 / mass[c]);
      std::vector<int> count(cells, 0);
      const std::size_t offs[4] = {0, 1, n, n + 1};
      for (const cvc::ocean::ewave_ghost &g : ghosts) {
        m_ghostCornerHost[g.cell * 2] = static_cast<float>(g.corner % n);
        m_ghostCornerHost[g.cell * 2 + 1] = static_cast<float>(g.corner / n);
        for (int k = 0; k < 4; ++k) {
          m_ghostWHost[g.cell * 4 + k] = static_cast<float>(g.w[k]);
          if (g.w[k] == 0.0)
            continue;
          // This ghost reads water cell q: its next reader slot, in the table's order.
          const std::size_t q = g.corner + offs[k];
          const int slot = count[q]++;
          if (slot >= 2 * kReaderTextures)
            throw std::logic_error("EWave: a water cell has more readers than ewave_max_readers");
          float *texel = m_readerHost.data() + cells * 4 * std::size_t(slot / 2) + q * 4 +
                         std::size_t(slot % 2) * 2;
          texel[0] = static_cast<float>(g.cell);
          texel[1] = static_cast<float>(g.w[k]);
        }
        ++m_ghosts;
      }
    }
  }
  if (m_ready) {
    m_rw->MakeCurrent();
    m_mask->Create2DFromRaw(m_p.n, m_p.n, 1, VTK_FLOAT, m_maskHost.data());
    m_ghostCorner->Create2DFromRaw(m_p.n, m_p.n, 2, VTK_FLOAT, m_ghostCornerHost.data());
    m_ghostW->Create2DFromRaw(m_p.n, m_p.n, 4, VTK_FLOAT, m_ghostWHost.data());
    m_invMass->Create2DFromRaw(m_p.n, m_p.n, 1, VTK_FLOAT, m_invMassHost.data());
    for (int r = 0; r < kReaderTextures; ++r)
      m_readers[r]->Create2DFromRaw(m_p.n, m_p.n, 4, VTK_FLOAT,
                                    m_readerHost.data() + cells * 4 * std::size_t(r));
  }
}

void EWave::setState(const std::vector<double> &h, const std::vector<double> &phi) {
  const std::size_t cells = m_maskHost.size();
  if (h.size() != cells || phi.size() != cells)
    throw std::invalid_argument("EWave: the state must have n x n heights and potentials");
  std::vector<float> z(cells * 2);
  for (std::size_t c = 0; c < cells; ++c) {
    if (!finite_value(h[c]) || !finite_value(phi[c]))
      throw std::invalid_argument("EWave: the state must be finite");
    z[c * 2] = static_cast<float>(h[c]);
    z[c * 2 + 1] = static_cast<float>(phi[c]);
  }
  if (!m_ready)
    return;
  m_rw->MakeCurrent();
  m_z[m_state]->Create2DFromRaw(m_p.n, m_p.n, 2, VTK_FLOAT, z.data());
}

void EWave::addDrop(double x, double y, double amount, double radius) {
  if (!finite_value(x) || !finite_value(y) || !finite_value(amount) || !(radius > 0.0) ||
      !finite_value(radius))
    throw std::invalid_argument("EWave: addDrop needs a finite point and amount and a radius "
                                "above 0");
  if (!m_ready)
    return;
  sourcePass(true, (x - m_p.origin_x) / m_p.dx, (y - m_p.origin_y) / m_p.dx, radius / m_p.dx,
             amount, false);
}

void EWave::addHeights(const std::vector<double> &dh) {
  if (dh.size() != m_maskHost.size())
    throw std::invalid_argument("EWave: addHeights needs n x n values");
  std::vector<float> f(dh.size());
  for (std::size_t c = 0; c < dh.size(); ++c) {
    if (!finite_value(dh[c]))
      throw std::invalid_argument("EWave: addHeights needs finite values");
    f[c] = static_cast<float>(dh[c]);
  }
  if (!m_ready)
    return;
  m_rw->MakeCurrent();
  m_src->Create2DFromRaw(m_p.n, m_p.n, 1, VTK_FLOAT, f.data());
  sourcePass(false, 0.0, 0.0, 1.0, 0.0, true);
}

void EWave::sourcePass(bool drop, double cx, double cy, double r, double amount, bool field) {
  PassScope scope(m_rw);
  const int st = m_state, dst = st == 0 ? 1 : 0;
  vtkTextureObject *src = m_z[st];
  oceanBeginPass(m_rw, m_fbo, m_z[dst], m_p.n);
  vtkShaderProgram *p = m_source->Program;
  m_rw->GetShaderCache()->ReadyShaderProgram(p);
  src->Activate();
  m_mask->Activate();
  m_src->Activate();
  p->SetUniformi("uZ", src->GetTextureUnit());
  p->SetUniformi("uMask", m_mask->GetTextureUnit());
  p->SetUniformi("uSrc", m_src->GetTextureUnit());
  p->SetUniformi("uDrop", drop ? 1 : 0);
  p->SetUniformi("uField", field ? 1 : 0);
  const float centre[2] = {static_cast<float>(cx), static_cast<float>(cy)};
  p->SetUniform2f("uCentre", centre);
  p->SetUniformf("uR2", static_cast<float>(r * r));
  p->SetUniformf("uAmount", static_cast<float>(amount));
  m_source->Render();
  src->Deactivate();
  m_mask->Deactivate();
  m_src->Deactivate();
  ++m_passes;
  m_state = dst;
}

void EWave::step(double dt) {
  if (!(dt > 0.0) || !finite_value(dt))
    throw std::invalid_argument("EWave: step needs a dt above 0 and finite");
  if (!m_ready)
    return;
  PassScope scope(m_rw);
  // Passes are counted as they are rendered (the butterflies by m_fft's own counter), so
  // passesRun() measures what step() did rather than repeating passesPerStep()'s formula.
  const long long fftBefore = m_fft.passesRun();
  if (m_ghosts > 0)
    symmetricStep(dt);
  else
    exactStep(dt);
  m_state = 0;
  m_passes += m_fft.passesRun() - fftBefore;
  m_time += dt;
}

void EWave::exactStep(double dt) {
  const int n = m_p.n;
  auto *cache = m_rw->GetShaderCache();
  // The state is in m_z[st]; the step ends in m_z[0]. w1 and w2 are the other two, chosen so the
  // first pass never writes the texture it reads.
  const int st = m_state, w1 = st == 1 ? 2 : 1, w2 = w1 == 1 ? 2 : 1;

  // Forward FFT: from st, through w1 and w2, the spectrum in w2.
  vtkTextureObject *spec = m_fft.transform(m_fbo, m_z[st], m_z[w1], m_z[w2], /*forward=*/true);

  // Update: w2 -> w1.
  oceanBeginPass(m_rw, m_fbo, m_z[w1], n);
  vtkShaderProgram *u = m_update->Program;
  cache->ReadyShaderProgram(u);
  spec->Activate();
  u->SetUniformi("uZ", spec->GetTextureUnit());
  u->SetUniformi("uN", n);
  u->SetUniformf("uDk", static_cast<float>(kTwoPi / (double(n) * m_p.dx)));
  // Deep water may be +infinity; k d only feeds tanh(min(k d, 20)).
  u->SetUniformf("uDepth", static_cast<float>(std::min(m_p.depth, 1e30)));
  u->SetUniformf("uG", static_cast<float>(m_p.gravity));
  u->SetUniformf("uDt", static_cast<float>(dt));
  u->SetUniformf("uGamma", static_cast<float>(m_p.damping_k2));
  u->SetUniformf("uDamping", static_cast<float>(m_p.damping));
  m_update->Render();
  ++m_passes;
  spec->Deactivate();

  // Inverse FFT: from w1, through w2 and w1, the field in w1.
  vtkTextureObject *field = m_fft.transform(m_fbo, m_z[w1], m_z[w2], m_z[w1], /*forward=*/false);

  // Real space: w1 -> 0.
  oceanBeginPass(m_rw, m_fbo, m_z[0], n);
  vtkShaderProgram *r = m_real->Program;
  cache->ReadyShaderProgram(r);
  field->Activate();
  m_mask->Activate();
  r->SetUniformi("uZ", field->GetTextureUnit());
  r->SetUniformi("uMask", m_mask->GetTextureUnit());
  r->SetUniformi("uN", n);
  r->SetUniformi("uSym", 0);
  r->SetUniformf("uScale", static_cast<float>(1.0 / (double(n) * double(n)))); // exact: 2^-2k
  r->SetUniformf("uDt", static_cast<float>(dt));
  r->SetUniformf("uSpongeW", static_cast<float>(m_p.sponge_cells));
  r->SetUniformf("uSpongeRate", static_cast<float>(m_p.sponge_rate));
  r->SetUniformf("uBeachRate", static_cast<float>(m_p.beach_rate));
  m_real->Render();
  ++m_passes;
  field->Deactivate();
  m_mask->Deactivate();
}

void EWave::stage(int mode, int st, int a, int b, double dts, double dt) {
  const int n = m_p.n;
  auto *cache = m_rw->GetShaderCache();
  const float half = static_cast<float>(0.5 * (m_p.gravity * dts));

  // Fill: st -> a.
  oceanBeginPass(m_rw, m_fbo, m_z[a], n);
  vtkShaderProgram *f = m_fill->Program;
  cache->ReadyShaderProgram(f);
  m_z[st]->Activate();
  m_ghostCorner->Activate();
  m_ghostW->Activate();
  m_invMass->Activate();
  f->SetUniformi("uZ", m_z[st]->GetTextureUnit());
  f->SetUniformi("uCorner", m_ghostCorner->GetTextureUnit());
  f->SetUniformi("uW", m_ghostW->GetTextureUnit());
  f->SetUniformi("uInvM", m_invMass->GetTextureUnit());
  f->SetUniformi("uMode", mode);
  f->SetUniformf("uHalf", half);
  m_fill->Render();
  ++m_passes;
  m_z[st]->Deactivate();
  m_ghostCorner->Deactivate();
  m_ghostW->Deactivate();
  m_invMass->Deactivate();

  // Forward FFT: from a, through b, the spectrum back in a.
  vtkTextureObject *spec = m_fft.transform(m_fbo, m_z[a], m_z[b], m_z[a], /*forward=*/true);

  // Multiply: a -> b.
  oceanBeginPass(m_rw, m_fbo, m_z[b], n);
  vtkShaderProgram *m = m_multiply->Program;
  cache->ReadyShaderProgram(m);
  spec->Activate();
  m->SetUniformi("uZ", spec->GetTextureUnit());
  m->SetUniformi("uN", n);
  m->SetUniformi("uMode", mode);
  m->SetUniformf("uDk", static_cast<float>(kTwoPi / (double(n) * m_p.dx)));
  m->SetUniformf("uDepth", static_cast<float>(std::min(m_p.depth, 1e30)));
  m->SetUniformf("uG", static_cast<float>(m_p.gravity));
  m->SetUniformf("uDts", static_cast<float>(dts));
  m->SetUniformf("uRate", static_cast<float>(2.0 * m_p.damping_k2 * dt));
  m->SetUniformf("uScale", static_cast<float>(1.0 / (double(n) * double(n))));
  m_multiply->Render();
  ++m_passes;
  spec->Deactivate();

  // Inverse FFT: from b, through a, the field back in b.
  vtkTextureObject *field = m_fft.transform(m_fbo, m_z[b], m_z[a], m_z[b], /*forward=*/false);

  // Gather: st and b -> a.
  oceanBeginPass(m_rw, m_fbo, m_z[a], n);
  vtkShaderProgram *g = m_gather->Program;
  cache->ReadyShaderProgram(g);
  m_z[st]->Activate();
  field->Activate();
  m_invMass->Activate();
  for (auto &r : m_readers)
    r->Activate();
  g->SetUniformi("uZ", m_z[st]->GetTextureUnit());
  g->SetUniformi("uS", field->GetTextureUnit());
  g->SetUniformi("uInvM", m_invMass->GetTextureUnit());
  for (int r = 0; r < kReaderTextures; ++r)
    g->SetUniformi(("uR" + std::to_string(r)).c_str(), m_readers[r]->GetTextureUnit());
  g->SetUniformi("uN", n);
  g->SetUniformi("uMode", mode);
  g->SetUniformf("uHalf", half);
  g->SetUniformf("uDts", static_cast<float>(dts));
  m_gather->Render();
  ++m_passes;
  m_z[st]->Deactivate();
  field->Deactivate();
  m_invMass->Deactivate();
  for (auto &r : m_readers)
    r->Deactivate();
}

void EWave::symmetricStep(double dt) {
  const int n = m_p.n, sub = cvc::ocean::ewave_substeps(m_p, dt);
  const double dts = dt / double(sub);
  // Each stage reads the state in st and writes the new one into a, never 0 (b may be): the
  // real-space pass then brings it home to m_z[0].
  int st = m_state;
  auto run = [&](int mode) {
    const int a = st == 0 ? 1 : 3 - st, b = st == 0 ? 2 : 0;
    stage(mode, st, a, b, dts, dt);
    st = a;
  };
  for (int s = 0; s < sub; ++s)
    run(0);
  if (m_p.damping_k2 > 0.0)
    run(1);

  // Real space: st -> 0, the beach and sponge on h at twice their rate, the global damping.
  auto *cache = m_rw->GetShaderCache();
  oceanBeginPass(m_rw, m_fbo, m_z[0], n);
  vtkShaderProgram *r = m_real->Program;
  cache->ReadyShaderProgram(r);
  m_z[st]->Activate();
  m_mask->Activate();
  r->SetUniformi("uZ", m_z[st]->GetTextureUnit());
  r->SetUniformi("uMask", m_mask->GetTextureUnit());
  r->SetUniformi("uN", n);
  r->SetUniformi("uSym", 1);
  r->SetUniformf("uScale", static_cast<float>(std::exp(-m_p.damping * dt)));
  r->SetUniformf("uDt", static_cast<float>(dt));
  r->SetUniformf("uSpongeW", static_cast<float>(m_p.sponge_cells));
  r->SetUniformf("uSpongeRate", static_cast<float>(m_p.sponge_rate));
  r->SetUniformf("uBeachRate", static_cast<float>(m_p.beach_rate));
  m_real->Render();
  ++m_passes;
  m_z[st]->Deactivate();
  m_mask->Deactivate();
}

int EWave::substeps(double dt) const {
  return m_ghosts > 0 ? cvc::ocean::ewave_substeps(m_p, dt) : 1;
}

int EWave::passesPerStep(double dt) const {
  if (m_ghosts == 0)
    return 4 * m_log2n + 2;
  const int stagePasses = 4 * m_log2n + 3;
  return substeps(dt) * stagePasses + (m_p.damping_k2 > 0.0 ? stagePasses : 0) + 1;
}

vtkTextureObject *EWave::heightTexture() const { return m_ready ? m_z[m_state].Get() : nullptr; }

std::vector<float> EWave::readbackState() const {
  if (!m_ready)
    return {};
  m_rw->MakeCurrent();
  return download(m_z[m_state], m_maskHost.size() * 2);
}

std::vector<float> EWave::readbackHeight() const {
  std::vector<float> z = readbackState();
  if (z.empty())
    return z;
  std::vector<float> h(z.size() / 2);
  for (std::size_t c = 0; c < h.size(); ++c)
    h[c] = z[c * 2];
  return h;
}
