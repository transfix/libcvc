/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc. LGPL-2.1. See OceanFFT.h for the ABYSSAL (MIT)
  attribution that applies to the spectrum / IFFT port.
*/

#include "OceanFFT.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>
#include <vtkNew.h>
#include <vtkOpenGLFramebufferObject.h>
#include <vtkOpenGLQuadHelper.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLShaderCache.h>
#include <vtkOpenGLState.h>
#include <vtkPixelBufferObject.h>
#include <vtkShaderProgram.h>
#include <vtkTextureObject.h>

namespace {
constexpr unsigned int kGL_BLEND = 0x0BE2;
constexpr unsigned int kGL_DEPTH_TEST = 0x0B71;
constexpr float kTAU = 6.28318530718f;
constexpr float kG = 9.80665f;

// The quad VAO binds attribute `ndCoordIn`; own VS + integer-coord texelFetch so
// no interpolated varying can mismatch VTK's internal quad shader.
const char *kQuadVS = "//VTK::System::Dec\n"
                      "in vec4 ndCoordIn;\n"
                      "void main() { gl_Position = ndCoordIn; }\n";

// ── one-time h0: JONSWAP spectrum with conjugate symmetry ────────────────────
const char *kH0FS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uNoise;\n"
    "uniform int uN;\n"
    "uniform float uL, uG, uAlpha, uPeakOmega, uGamma, uDepth;\n"
    "uniform vec2 uWindDir;\n"
    "//VTK::Output::Dec\n"
    "const float TAU = 6.28318530718;\n"
    "float jonswap(float w){\n"
    "  if (w < 1e-4) return 0.0;\n"
    "  float sig = (w <= uPeakOmega) ? 0.07 : 0.09;\n"
    "  float rr = exp(-(w-uPeakOmega)*(w-uPeakOmega)/(2.0*sig*sig*uPeakOmega*uPeakOmega));\n"
    "  float iw = 1.0/w; float iw5 = iw*iw*iw*iw*iw;\n"
    "  return uAlpha*uG*uG*iw5*exp(-1.25*pow(uPeakOmega*iw,4.0))*pow(uGamma, rr);\n"
    "}\n"
    "float spectrum(vec2 kv){\n"
    "  float kl = length(kv);\n"
    "  if (kl < 1e-5) return 0.0;\n"
    "  float w = sqrt(uG*kl*tanh(min(kl*uDepth,20.0)));\n"
    "  float dwdk = 0.5*uG/max(w,1e-4);\n" // dω/dk (deep) to map S(ω)->S(k)
    "  float dir = pow(max(dot(kv/kl, uWindDir), 0.0), 2.0) + 0.03;\n"
    "  return jonswap(w) * dwdk / kl * dir;\n"
    "}\n"
    "void main(){\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  vec2  kv  = TAU/uL * (vec2(c) - float(uN)*0.5);\n"
    "  ivec2 m   = ivec2((uN - c.x) % uN, (uN - c.y) % uN);\n"
    "  vec2  kvm = TAU/uL * (vec2(m) - float(uN)*0.5);\n"
    "  float dk = TAU/uL;\n"
    "  vec2 g  = texelFetch(uNoise, c, 0).xy;\n"
    "  vec2 gm = texelFetch(uNoise, m, 0).xy;\n"
    "  float amp  = sqrt(2.0*max(spectrum(kv), 0.0))*dk*0.7071;\n"
    "  float ampm = sqrt(2.0*max(spectrum(kvm),0.0))*dk*0.7071;\n"
    "  gl_FragData[0] = vec4(g*amp, (gm*ampm).x, -(gm*ampm).y);\n"
    "}\n";

// ── time evolution -> buffer 0 (Dx+iDz, Dy+iDyDx) ────────────────────────────
const char *kTime0FS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uH0;\n"
    "uniform int uN;\n"
    "uniform float uL, uG, uTime, uDepth;\n"
    "//VTK::Output::Dec\n"
    "const float TAU = 6.28318530718;\n"
    "vec2 cmul(vec2 a, vec2 b){ return vec2(a.x*b.x-a.y*b.y, a.x*b.y+a.y*b.x); }\n"
    "void main(){\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  vec2 kv = TAU/uL * (vec2(c) - float(uN)*0.5);\n"
    "  float kl = length(kv);\n"
    "  vec4 h0 = texelFetch(uH0, c, 0);\n"
    "  if (kl < 1e-6) { gl_FragData[0] = vec4(0.0); return; }\n"
    "  vec2 kn = kv/kl;\n"
    "  float w = sqrt(uG*kl*tanh(min(kl*uDepth,20.0)));\n"
    "  float ph = w*uTime; vec2 e = vec2(cos(ph), sin(ph)); vec2 ec = vec2(e.x, -e.y);\n"
    "  vec2 h  = cmul(h0.xy, e) + cmul(h0.zw, ec);\n"
    "  vec2 ih = vec2(-h.y, h.x);\n"
    "  vec2 Dx = ih*kn.x, Dz = ih*kn.y, Dy = h, DyDx = ih*kv.x;\n"
    "  vec2 C1 = vec2(Dx.x - Dz.y,   Dx.y + Dz.x);\n"
    "  vec2 C2 = vec2(Dy.x - DyDx.y, Dy.y + DyDx.x);\n"
    "  gl_FragData[0] = vec4(C1, C2);\n"
    "}\n";

// ── time evolution -> buffer 1 (DyDz+iDxDx, DzDz+iDxDz) for the foam Jacobian ─
const char *kTime1FS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uH0;\n"
    "uniform int uN;\n"
    "uniform float uL, uG, uTime, uDepth;\n"
    "//VTK::Output::Dec\n"
    "const float TAU = 6.28318530718;\n"
    "vec2 cmul(vec2 a, vec2 b){ return vec2(a.x*b.x-a.y*b.y, a.x*b.y+a.y*b.x); }\n"
    "void main(){\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  vec2 kv = TAU/uL * (vec2(c) - float(uN)*0.5);\n"
    "  float kl = length(kv);\n"
    "  vec4 h0 = texelFetch(uH0, c, 0);\n"
    "  if (kl < 1e-6) { gl_FragData[0] = vec4(0.0); return; }\n"
    "  vec2 kn = kv/kl;\n"
    "  float w = sqrt(uG*kl*tanh(min(kl*uDepth,20.0)));\n"
    "  float ph = w*uTime; vec2 e = vec2(cos(ph), sin(ph)); vec2 ec = vec2(e.x, -e.y);\n"
    "  vec2 h  = cmul(h0.xy, e) + cmul(h0.zw, ec);\n"
    "  vec2 ih = vec2(-h.y, h.x);\n"
    "  vec2 DyDz = ih*kv.y;\n"
    "  vec2 DxDx = -h*(kv.x*kn.x), DzDz = -h*(kv.y*kn.y), DxDz = -h*(kv.y*kn.x);\n"
    "  vec2 C1 = vec2(DyDz.x - DxDx.y, DyDz.y + DxDx.x);\n"
    "  vec2 C2 = vec2(DzDz.x - DxDz.y, DzDz.y + DxDz.x);\n"
    "  gl_FragData[0] = vec4(C1, C2);\n"
    "}\n";

// ── one butterfly stage (single buffer; 2 complex per texel: .rg and .ba) ─────
const char *kButterflyFS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uSrc, uButterfly;\n"
    "uniform int uStage, uVertical, uN;\n"
    "//VTK::Output::Dec\n"
    "vec2 cmul(vec2 a, vec2 b){ return vec2(a.x*b.x-a.y*b.y, a.x*b.y+a.y*b.x); }\n"
    "void main(){\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  int idx = (uVertical == 0) ? c.x : c.y;\n"
    "  vec4 bf = texelFetch(uButterfly, ivec2(uStage, idx), 0);\n"
    "  vec2 w = bf.xy; int topI = int(bf.z + 0.5); int botI = int(bf.w + 0.5);\n"
    "  vec4 a, b;\n"
    "  if (uVertical == 0) { a = texelFetch(uSrc, ivec2(topI, c.y), 0); b = texelFetch(uSrc, "
    "ivec2(botI, c.y), 0); }\n"
    "  else                { a = texelFetch(uSrc, ivec2(c.x, topI), 0); b = texelFetch(uSrc, "
    "ivec2(c.x, botI), 0); }\n"
    "  gl_FragData[0] = vec4(a.rg + cmul(w, b.rg), a.ba + cmul(w, b.ba));\n"
    "}\n";

// ── assemble: permute, extract real fields, choppy displacement + Jacobian ───
const char *kAssembleFS = "//VTK::System::Dec\n"
                          "uniform sampler2D uBuf0, uBuf1;\n"
                          "uniform int uN;\n"
                          "uniform float uLambda;\n"
                          "//VTK::Output::Dec\n"
                          "void main(){\n"
                          "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
                          "  float perm = (((c.x + c.y) & 1) == 0) ? 1.0 : -1.0;\n"
                          "  vec4 b0 = texelFetch(uBuf0, c, 0) * perm;\n"
                          "  vec4 b1 = texelFetch(uBuf1, c, 0) * perm;\n"
                          "  float Dx = b0.x, Dz = b0.y, Dy = b0.z;\n"
                          "  float DxDx = b1.y, DzDz = b1.z, DxDz = b1.w;\n"
                          "  float lx = uLambda*DxDx, lz = uLambda*DzDz, lxz = uLambda*DxDz;\n"
                          "  float jac = (1.0+lx)*(1.0+lz) - lxz*lxz;\n"
                          "  gl_FragData[0] = vec4(uLambda*Dx, Dy, uLambda*Dz, jac);\n"
                          "}\n";

const char *kSelfTestFS = "//VTK::System::Dec\n"
                          "uniform sampler2D uSrc;\n"
                          "//VTK::Output::Dec\n"
                          "void main(){ gl_FragData[0] = texelFetch(uSrc, ivec2(gl_FragCoord.xy), "
                          "0) + vec4(1.0,2.0,3.0,4.0); }\n";

float hash01(int x, int y) {
  unsigned int h =
      static_cast<unsigned int>(x) * 374761393u + static_cast<unsigned int>(y) * 668265263u;
  h = (h ^ (h >> 13)) * 1274126177u;
  h ^= h >> 16;
  return static_cast<float>(h) / 4294967296.0f;
}
} // namespace

OceanFFT::OceanFFT(int n) : m_n(n) {
  m_log2n = static_cast<int>(std::lround(std::log2(static_cast<double>(n))));
}
OceanFFT::~OceanFFT() { releaseResources(); }

bool OceanFFT::init(vtkOpenGLRenderWindow *rw) {
  if (!rw)
    return false;
  m_rw = rw;
  m_rw->MakeCurrent();

  // Derived JONSWAP parameters.
  m_alpha = 0.076f * std::pow((windSpeed * windSpeed) / (fetch_m * kG), 0.22f);
  m_peakOmega = std::max(22.0f * std::pow((kG * kG) / (windSpeed * fetch_m), 1.0f / 3.0f), 0.30f);
  m_gamma = 3.3f;

  auto mkRT = [&](vtkSmartPointer<vtkTextureObject> &t, int w, int h, const float *seed) -> bool {
    t = vtkSmartPointer<vtkTextureObject>::New();
    t->SetContext(m_rw);
    bool ok = seed ? t->Create2DFromRaw(w, h, 4, VTK_FLOAT, const_cast<float *>(seed))
                   : t->Create2D(w, h, 4, VTK_FLOAT, false);
    if (!ok)
      return false;
    t->SetWrapS(vtkTextureObject::ClampToEdge);
    t->SetWrapT(vtkTextureObject::ClampToEdge);
    t->SetMinificationFilter(vtkTextureObject::Nearest);
    t->SetMagnificationFilter(vtkTextureObject::Nearest);
    return true;
  };

  // Gaussian noise (Box-Muller from a hash), rg = independent pair.
  std::vector<float> noise(static_cast<size_t>(m_n) * m_n * 4, 0.0f);
  for (int y = 0; y < m_n; ++y)
    for (int x = 0; x < m_n; ++x) {
      float u1 = std::max(hash01(x, y), 1e-6f);
      float u2 = hash01(x + 57, y + 131);
      float u3 = std::max(hash01(x + 91, y + 7), 1e-6f);
      float u4 = hash01(x + 13, y + 199);
      float r1 = std::sqrt(-2.0f * std::log(u1)), r2 = std::sqrt(-2.0f * std::log(u3));
      const size_t i = (static_cast<size_t>(y) * m_n + x) * 4;
      noise[i + 0] = r1 * std::cos(kTAU * u2);
      noise[i + 1] = r2 * std::sin(kTAU * u4);
    }

  // Butterfly LUT (Cooley-Tukey DIT, bit-reversal in stage 0). x=stage, y=index.
  auto bitrev = [&](int v) {
    int r = 0;
    for (int b = 0; b < m_log2n; ++b) {
      r = (r << 1) | (v & 1);
      v >>= 1;
    }
    return r;
  };
  std::vector<float> lut(static_cast<size_t>(m_log2n) * m_n * 4, 0.0f);
  for (int stage = 0; stage < m_log2n; ++stage)
    for (int y = 0; y < m_n; ++y) {
      float twoS1 = std::pow(2.0f, static_cast<float>(stage + 1));
      float kf = std::fmod(static_cast<float>(y) * (static_cast<float>(m_n) / twoS1),
                           static_cast<float>(m_n));
      float ang = kTAU * kf / static_cast<float>(m_n); // IFFT twiddle: exp(+i ang)
      int span = 1 << stage;
      bool topWing =
          std::fmod(static_cast<float>(y), twoS1) < std::pow(2.0f, static_cast<float>(stage));
      int top, bot;
      if (stage == 0) {
        if (topWing) {
          top = bitrev(y);
          bot = bitrev(y + 1);
        } else {
          top = bitrev(y - 1);
          bot = bitrev(y);
        }
      } else {
        if (topWing) {
          top = y;
          bot = y + span;
        } else {
          top = y - span;
          bot = y;
        }
      }
      const size_t i = (static_cast<size_t>(y) * m_log2n + stage) * 4;
      lut[i + 0] = std::cos(ang);
      lut[i + 1] = std::sin(ang);
      lut[i + 2] = static_cast<float>(top);
      lut[i + 3] = static_cast<float>(bot);
    }

  if (!mkRT(m_noise, m_n, m_n, noise.data()) || !mkRT(m_butterfly, m_log2n, m_n, lut.data()) ||
      !mkRT(m_h0, m_n, m_n, nullptr) || !mkRT(m_a0, m_n, m_n, nullptr) ||
      !mkRT(m_b0, m_n, m_n, nullptr) || !mkRT(m_a1, m_n, m_n, nullptr) ||
      !mkRT(m_b1, m_n, m_n, nullptr) || !mkRT(m_disp, m_n, m_n, nullptr)) {
    std::fprintf(stderr, "OceanFFT: RGBA32F texture allocation failed — float RTs unsupported.\n");
    return false;
  }

  // The output displacement is sampled by external shaders (the GPU ocean). The
  // FFT field is exactly periodic, so Repeat wrap tiles it seamlessly. Filtering
  // stays NEAREST on purpose: LINEAR filtering of an RGBA32F texture is NOT core
  // in WebGL2 (it needs OES_texture_float_linear, separate from the color-buffer-
  // float extension) — an unsupported LINEAR float sampler goes incomplete and
  // silently returns (0,0,0,1), i.e. a flat, motionless sea, on the wasm/Pages
  // target with no error. Nearest is core-filterable everywhere. (The FFT passes
  // use texelFetch, which ignores filtering, so this does not affect them.)
  m_disp->SetMinificationFilter(vtkTextureObject::Nearest);
  m_disp->SetMagnificationFilter(vtkTextureObject::Nearest);
  m_disp->SetWrapS(vtkTextureObject::Repeat);
  m_disp->SetWrapT(vtkTextureObject::Repeat);

  m_fbo = vtkSmartPointer<vtkOpenGLFramebufferObject>::New();
  m_fbo->SetContext(m_rw);

  m_h0Pass = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kQuadVS, kH0FS, nullptr);
  m_time0 = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kQuadVS, kTime0FS, nullptr);
  m_time1 = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kQuadVS, kTime1FS, nullptr);
  m_butterflyPass = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kQuadVS, kButterflyFS, nullptr);
  m_assemble = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kQuadVS, kAssembleFS, nullptr);
  if (!m_h0Pass->Program || !m_time0->Program || !m_time1->Program || !m_butterflyPass->Program ||
      !m_assemble->Program) {
    std::fprintf(stderr, "OceanFFT: a shader program failed to build.\n");
    return false;
  }

  m_ready = true;
  runH0Pass(); // build the initial spectrum into m_h0
  return true;
}

void OceanFFT::runH0Pass() {
  m_rw->MakeCurrent();
  const float wdl = std::max(std::sqrt(windDirX * windDirX + windDirZ * windDirZ), 1e-6f);
  float wd[2] = {windDirX / wdl, windDirZ / wdl};
  m_rw->GetState()->PushFramebufferBindings();
  beginPass(m_h0);
  m_rw->GetShaderCache()->ReadyShaderProgram(m_h0Pass->Program);
  m_noise->Activate();
  m_h0Pass->Program->SetUniformi("uNoise", m_noise->GetTextureUnit());
  m_h0Pass->Program->SetUniformi("uN", m_n);
  m_h0Pass->Program->SetUniformf("uL", tileSize_m);
  m_h0Pass->Program->SetUniformf("uG", kG);
  m_h0Pass->Program->SetUniformf("uAlpha", m_alpha);
  m_h0Pass->Program->SetUniformf("uPeakOmega", m_peakOmega);
  m_h0Pass->Program->SetUniformf("uGamma", m_gamma);
  m_h0Pass->Program->SetUniformf("uDepth", depth_m);
  m_h0Pass->Program->SetUniform2f("uWindDir", wd);
  m_h0Pass->Render();
  m_noise->Deactivate();
  m_rw->GetState()->PopFramebufferBindings();
}

void OceanFFT::rebuildSpectrum() {
  if (!m_ready)
    return;
  m_alpha = 0.076f * std::pow((windSpeed * windSpeed) / (fetch_m * kG), 0.22f);
  m_peakOmega = std::max(22.0f * std::pow((kG * kG) / (windSpeed * fetch_m), 1.0f / 3.0f), 0.30f);
  runH0Pass();
}

void OceanFFT::beginPass(vtkTextureObject *dst) {
  m_fbo->Bind();
  m_fbo->AddColorAttachment(0, dst);
  m_fbo->ActivateDrawBuffers(1);
  auto *st = m_rw->GetState();
  st->vtkglViewport(0, 0, m_n, m_n);
  st->vtkglDisable(kGL_BLEND);
  st->vtkglDisable(kGL_DEPTH_TEST);
}
void OceanFFT::endPass() {}

void OceanFFT::step(double timeSeconds) {
  if (!m_ready)
    return;
  m_rw->MakeCurrent();
  auto *cache = m_rw->GetShaderCache();
  const float t = static_cast<float>(timeSeconds);

  m_rw->GetState()->PushFramebufferBindings();

  // time evolution -> a0, a1
  auto timePass = [&](vtkOpenGLQuadHelper *q, vtkTextureObject *dst) {
    beginPass(dst);
    cache->ReadyShaderProgram(q->Program);
    m_h0->Activate();
    q->Program->SetUniformi("uH0", m_h0->GetTextureUnit());
    q->Program->SetUniformi("uN", m_n);
    q->Program->SetUniformf("uL", tileSize_m);
    q->Program->SetUniformf("uG", kG);
    q->Program->SetUniformf("uTime", t);
    q->Program->SetUniformf("uDepth", depth_m);
    q->Render();
    m_h0->Deactivate();
  };
  timePass(m_time0.get(), m_a0);
  timePass(m_time1.get(), m_a1);

  // butterfly IFFT (2 directions x log2N stages) on each buffer, ping-pong.
  auto butterfly = [&](vtkTextureObject *&cur, vtkTextureObject *&oth) {
    for (int vert = 0; vert < 2; ++vert)
      for (int stage = 0; stage < m_log2n; ++stage) {
        beginPass(oth);
        cache->ReadyShaderProgram(m_butterflyPass->Program);
        cur->Activate();
        m_butterfly->Activate();
        m_butterflyPass->Program->SetUniformi("uSrc", cur->GetTextureUnit());
        m_butterflyPass->Program->SetUniformi("uButterfly", m_butterfly->GetTextureUnit());
        m_butterflyPass->Program->SetUniformi("uStage", stage);
        m_butterflyPass->Program->SetUniformi("uVertical", vert);
        m_butterflyPass->Program->SetUniformi("uN", m_n);
        m_butterflyPass->Render();
        cur->Deactivate();
        m_butterfly->Deactivate();
        std::swap(cur, oth);
      }
  };
  vtkTextureObject *cur0 = m_a0, *oth0 = m_b0;
  vtkTextureObject *cur1 = m_a1, *oth1 = m_b1;
  butterfly(cur0, oth0);
  butterfly(cur1, oth1);

  // assemble -> disp
  beginPass(m_disp);
  cache->ReadyShaderProgram(m_assemble->Program);
  cur0->Activate();
  cur1->Activate();
  m_assemble->Program->SetUniformi("uBuf0", cur0->GetTextureUnit());
  m_assemble->Program->SetUniformi("uBuf1", cur1->GetTextureUnit());
  m_assemble->Program->SetUniformi("uN", m_n);
  m_assemble->Program->SetUniformf("uLambda", chop);
  m_assemble->Render();
  cur0->Deactivate();
  cur1->Deactivate();

  m_rw->GetState()->PopFramebufferBindings();
}

bool OceanFFT::selfTest() {
  if (!m_ready)
    return false;
  m_rw->MakeCurrent();
  const int K = 8;
  const float seed[4] = {0.25f, 0.5f, 0.75f, 1.0f};
  std::vector<float> seedData(static_cast<size_t>(m_n) * m_n * 4);
  for (int i = 0; i < m_n * m_n; ++i)
    for (int c = 0; c < 4; ++c)
      seedData[static_cast<size_t>(i) * 4 + c] = seed[c];
  m_a0->Create2DFromRaw(m_n, m_n, 4, VTK_FLOAT, seedData.data());

  vtkOpenGLQuadHelper test(m_rw, kQuadVS, kSelfTestFS, nullptr);
  if (!test.Program)
    return false;

  vtkTextureObject *src = m_a0, *dst = m_b0;
  m_rw->GetState()->PushFramebufferBindings();
  for (int p = 0; p < K; ++p) {
    beginPass(dst);
    m_rw->GetShaderCache()->ReadyShaderProgram(test.Program);
    src->Activate();
    test.Program->SetUniformi("uSrc", src->GetTextureUnit());
    test.Render();
    src->Deactivate();
    std::swap(src, dst);
  }
  m_rw->GetState()->PopFramebufferBindings();

  vtkSmartPointer<vtkPixelBufferObject> pbo;
  pbo.TakeReference(src->Download());
  if (!pbo)
    return false;
  auto *data = static_cast<float *>(pbo->MapPackedBuffer());
  if (!data)
    return false;
  const size_t centre = (static_cast<size_t>(m_n / 2) * m_n + (m_n / 2)) * 4;
  bool pass = true;
  for (int c = 0; c < 4; ++c)
    if (std::fabs(data[centre + c] - (seed[c] + K * (c + 1))) > 1e-3f)
      pass = false;
  pbo->UnmapPackedBuffer();
  return pass;
}

vtkTextureObject *OceanFFT::displacement() const { return m_ready ? m_disp.Get() : nullptr; }

std::vector<float> OceanFFT::readbackDisplacement() const {
  std::vector<float> out;
  if (!m_ready)
    return out;
  m_rw->MakeCurrent();
  vtkSmartPointer<vtkPixelBufferObject> pbo;
  pbo.TakeReference(m_disp->Download());
  if (!pbo)
    return out;
  auto *data = static_cast<float *>(pbo->MapPackedBuffer());
  if (data) {
    out.assign(data, data + static_cast<size_t>(m_n) * m_n * 4);
    pbo->UnmapPackedBuffer();
  }
  return out;
}

void OceanFFT::releaseResources() {
  m_h0Pass.reset();
  m_time0.reset();
  m_time1.reset();
  m_butterflyPass.reset();
  m_assemble.reset();
  m_fbo = nullptr;
  m_noise = m_butterfly = m_h0 = m_a0 = m_b0 = m_a1 = m_b1 = m_disp = nullptr;
  m_ready = false;
}
