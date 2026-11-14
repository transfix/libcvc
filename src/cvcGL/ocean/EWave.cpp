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
#include <cvc/algo/det_math.h>
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
// The same arithmetic as ewave::step's mode loop (cvc/ocean/ewave.cpp), with ewave::prepare's
// coefficients for this dt from uCoef (see modeCoefficients()). The mirror of index p is
// (n - p) mod n, so the Nyquist index n / 2 pairs with itself.
const char *kUpdateFS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uZ, uCoef;\n"
    "uniform int uN;\n"
    "//VTK::Output::Dec\n"
    "void main(){\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  ivec2 m = (ivec2(uN) - c) & ivec2(uN - 1);\n"
    "  vec2 z = texelFetch(uZ, c, 0).rg, w = texelFetch(uZ, m, 0).rg;\n"
    // Z(k) = H(k) + i PHI(k) and Z(-k) = conj H(k) + i conj PHI(k), as h and phi are real.
    "  vec2 H = 0.5 * vec2(z.x + w.x, z.y - w.y);\n"
    "  vec2 P = 0.5 * vec2(z.y + w.y, w.x - z.x);\n"
    "  vec4 a = texelFetch(uCoef, c, 0);\n"
    "  vec2 H2 = a.x * H + a.y * P, P2 = a.z * H + a.w * P;\n"
    "  gl_FragData[0] = vec4(H2.x - P2.y, H2.y + P2.x, 0.0, 0.0);\n"
    "}\n";

// Each mode's update for a step of dt, ewave::prepare's: RGBA per texel the a0..a3 of
// [H'; PHI'] = [a0 a1; a2 a3] [H; PHI], the step's damping included, and (damp, 0, 0, 0) at k = 0
// (the mean height stays, the mean potential is held at 0). The 1 / n^2 of the inverse FFT stays
// in the real-space pass. Index p is wavenumber p (p < n / 2) or p - n, times 2 pi / (n dx).
//
// Worked out in double on the host, as the CPU window does, whenever dt changes, so the GPU takes
// no cos, sin, exp, sqrt or tanh of its own. The update pass used to work out the step's cos, sin
// and exp itself, in float. On Apple's software renderer (macOS CI) the GPU window then drifted
// from the CPU's at a steady rate per second of simulated time, whatever dt was: 3e-2 of the RMS
// height in 7.2 s (cvcgl_ewave on transfix/libcvc#1171), where Linux and Windows stay near 2e-5. A
// sin or cos off in proportion to its small angle drifts like that. Taking each mode's rates from
// the host in double, with the cos, sin and exp still in the shader, changed nothing there, to the
// digit (transfix/libcvc#1219's first try).
std::vector<float> modeCoefficients(const cvc::ocean::ewave_params &p, double dt) {
  namespace det = cvc::algo::det;
  const int n = p.n;
  const double dk = kTwoPi / (double(n) * p.dx);
  std::vector<float> cf(static_cast<std::size_t>(n) * n * 4, 0.0f);
  for (int j = 0; j < n; ++j) {
    const double ky = dk * double(j < n / 2 ? j : j - n);
    for (int i = 0; i < n; ++i) {
      const double kx = dk * double(i < n / 2 ? i : i - n);
      const double k2 = kx * kx + ky * ky, k = std::sqrt(k2);
      float *a = cf.data() + (static_cast<std::size_t>(j) * n + i) * 4;
      const double damp = det::exp(-(p.damping_k2 * k2 + p.damping) * dt);
      const double kap = k > 0.0 ? k * det::tanh(std::min(k * p.depth, 20.0)) : 0.0;
      if (!(kap > 0.0)) {
        a[0] = static_cast<float>(damp);
        continue;
      }
      const double w = std::sqrt(p.gravity * kap), ph = w * dt;
      const double cs = det::cos(ph), sn = det::sin(ph);
      a[0] = static_cast<float>(cs * damp);
      a[1] = static_cast<float>((kap / w) * sn * damp);
      a[2] = static_cast<float>(-(p.gravity / w) * sn * damp);
      a[3] = static_cast<float>(cs * damp);
    }
  }
  return cf;
}

// Each cell's real-space factor for a step of dt, ewave::step's: the mask's beach rate and the
// sponge, a rate per cell applied as exp(-rate dt), and 0 on land. Worked out in double on the
// host whenever dt or the mask changes, for the reason modeCoefficients() gives.
std::vector<float> realFactors(const cvc::ocean::ewave_params &p, const std::vector<float> &mask,
                               double dt) {
  namespace det = cvc::algo::det;
  const int n = p.n;
  const double sw = p.sponge_cells;
  std::vector<float> f(static_cast<std::size_t>(n) * n);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const std::size_t c = static_cast<std::size_t>(j) * n + i;
      const double m = mask[c];
      double rate = m < 1.0 ? p.beach_rate * (1.0 - m) : 0.0;
      const double d = double(std::min(std::min(i, n - 1 - i), std::min(j, n - 1 - j)));
      if (sw > 0.0 && d < sw) {
        const double r = (sw - d) / sw;
        rate += p.sponge_rate * (r * r);
      }
      f[c] = static_cast<float>(m <= 0.0 ? 0.0 : rate > 0.0 ? det::exp(-rate * dt) : 1.0);
    }
  return f;
}

// ── the real-space pass: normalise, then the mask's beach, the sponge and land ──
// z / n^2 times the cell's factor from uFactor (see realFactors()).
const char *kRealFS = "//VTK::System::Dec\n"
                      "uniform sampler2D uZ, uFactor;\n"
                      "uniform float uScale;\n"
                      "//VTK::Output::Dec\n"
                      "void main(){\n"
                      "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
                      "  vec2 z = texelFetch(uZ, c, 0).rg * uScale;\n"
                      "  gl_FragData[0] = vec4(z * texelFetch(uFactor, c, 0).r, 0.0, 0.0);\n"
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
  m_maskHost.assign(static_cast<std::size_t>(m_p.n) * m_p.n, 1.0f);
}

EWave::~EWave() { releaseResources(); }

bool EWave::init(vtkOpenGLRenderWindow *rw) {
  releaseResources();
  if (!rw)
    return false;
  m_rw = rw;
  m_rw->MakeCurrent();
  const int n = m_p.n;
  const std::vector<float> zeros(static_cast<std::size_t>(n) * n * 2, 0.0f);
  for (auto &z : m_z)
    if (!(z = oceanFloatTexture(m_rw, n, n, 2, zeros.data()))) {
      std::fprintf(stderr, "EWave: RG32F texture allocation failed — float RTs unsupported.\n");
      return false;
    }
  m_mask = oceanFloatTexture(m_rw, n, n, 1, m_maskHost.data());
  m_src = oceanFloatTexture(m_rw, n, n, 1, zeros.data());
  // Filled by the first step (see step()).
  m_coef = oceanFloatTexture(m_rw, n, n, 4, nullptr);
  m_factor = oceanFloatTexture(m_rw, n, n, 1, nullptr);
  m_dt = 0.0;
  m_factorStale = true;
  if (!m_mask || !m_src || !m_coef || !m_factor || !m_fft.init(m_rw, n)) {
    std::fprintf(stderr, "EWave: R32F/RGBA32F texture or FFT allocation failed.\n");
    return false;
  }
  m_fbo = vtkSmartPointer<vtkOpenGLFramebufferObject>::New();
  m_fbo->SetContext(m_rw);
  m_update = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kUpdateFS, nullptr);
  m_real = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kRealFS, nullptr);
  m_source = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kSourceFS, nullptr);
  if (!m_update->Program || !m_real->Program || !m_source->Program) {
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
  m_fft.release();
  m_fbo = nullptr;
  for (auto &z : m_z)
    z = nullptr;
  m_mask = m_src = m_coef = m_factor = nullptr;
  m_dt = 0.0;
  m_ready = false;
}

void EWave::setMask(const std::vector<double> &mask) {
  if (mask.size() != m_maskHost.size())
    throw std::invalid_argument("EWave: the mask must have n x n values");
  for (double m : mask)
    if (!(m >= 0.0) || !(m <= 1.0))
      throw std::invalid_argument("EWave: mask values must be in [0, 1]");
  for (std::size_t c = 0; c < mask.size(); ++c)
    m_maskHost[c] = static_cast<float>(mask[c]);
  m_factorStale = true; // the beach and land are in the real-space factors
  if (m_ready) {
    m_rw->MakeCurrent();
    m_mask->Create2DFromRaw(m_p.n, m_p.n, 1, VTK_FLOAT, m_maskHost.data());
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
  const int dst = m_state == 0 ? 1 : 0;
  vtkTextureObject *src = m_z[m_state];
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
  m_state = dst;
  ++m_passes;
}

void EWave::step(double dt) {
  if (!(dt > 0.0) || !finite_value(dt))
    throw std::invalid_argument("EWave: step needs a dt above 0 and finite");
  if (!m_ready)
    return;
  PassScope scope(m_rw);
  const int n = m_p.n;
  auto *cache = m_rw->GetShaderCache();
  // The state is in m_z[st]; the step ends in m_z[0]. w1 and w2 are the other two, chosen so the
  // first pass never writes the texture it reads.
  const int st = m_state, w1 = st == 1 ? 2 : 1, w2 = w1 == 1 ? 2 : 1;
  // Passes are counted as they are rendered (the butterflies by m_fft's own counter), so
  // passesRun() measures what step() did rather than repeating passesPerStep()'s formula.
  const long long fftBefore = m_fft.passesRun();

  // A new dt (or mask) works the coefficients (or factors) out again, as ewave::step does.
  if (dt != m_dt) {
    std::vector<float> cf = modeCoefficients(m_p, dt);
    m_coef->Create2DFromRaw(n, n, 4, VTK_FLOAT, cf.data());
    m_dt = dt;
    m_factorStale = true;
  }
  if (m_factorStale) {
    std::vector<float> f = realFactors(m_p, m_maskHost, dt);
    m_factor->Create2DFromRaw(n, n, 1, VTK_FLOAT, f.data());
    m_factorStale = false;
  }

  // Forward FFT: from st, through w1 and w2, the spectrum in w2.
  vtkTextureObject *spec = m_fft.transform(m_fbo, m_z[st], m_z[w1], m_z[w2], /*forward=*/true);

  // Update: w2 -> w1.
  oceanBeginPass(m_rw, m_fbo, m_z[w1], n);
  vtkShaderProgram *u = m_update->Program;
  cache->ReadyShaderProgram(u);
  spec->Activate();
  m_coef->Activate();
  u->SetUniformi("uZ", spec->GetTextureUnit());
  u->SetUniformi("uCoef", m_coef->GetTextureUnit());
  u->SetUniformi("uN", n);
  m_update->Render();
  ++m_passes;
  spec->Deactivate();
  m_coef->Deactivate();

  // Inverse FFT: from w1, through w2 and w1, the field in w1.
  vtkTextureObject *field = m_fft.transform(m_fbo, m_z[w1], m_z[w2], m_z[w1], /*forward=*/false);

  // Real space: w1 -> 0.
  oceanBeginPass(m_rw, m_fbo, m_z[0], n);
  vtkShaderProgram *r = m_real->Program;
  cache->ReadyShaderProgram(r);
  field->Activate();
  m_factor->Activate();
  r->SetUniformi("uZ", field->GetTextureUnit());
  r->SetUniformi("uFactor", m_factor->GetTextureUnit());
  r->SetUniformf("uScale", static_cast<float>(1.0 / (double(n) * double(n)))); // exact: 2^-2k
  m_real->Render();
  ++m_passes;
  field->Deactivate();
  m_factor->Deactivate();

  m_state = 0;
  m_passes += m_fft.passesRun() - fftBefore;
  m_time += dt;
}

int EWave::passesPerStep() const { return 4 * m_log2n + 2; }

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
