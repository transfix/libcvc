/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc. LGPL-2.1. See OceanFFT.h for the ABYSSAL (MIT)
  attribution that applies to the spectrum / IFFT port.
*/

#include "OceanFFT.h"

#include "OceanButterfly.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ocean/h0.h>
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
constexpr unsigned int kGL_SCISSOR_TEST = 0x0C11;
constexpr float kG = 9.80665f;

// The vertex shader is OceanButterfly.cpp's (kOceanQuadVS), as are the butterfly passes.

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
    // The Nyquist row and column (k = -N/2) have no partner -k on the grid, so their fields are
    // not real, and the packing (two real fields to one complex transform) leaks one into the
    // other: the height picks up the slope field there, scaled by k_x up to the Nyquist. Drop
    // them, as the CPU's physics band does (cvc/ocean/sea_surface.h).
    "  if (kl < 1e-6 || c.x == 0 || c.y == 0) { gl_FragData[0] = vec4(0.0); return; }\n"
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
    "  if (kl < 1e-6 || c.x == 0 || c.y == 0) { gl_FragData[0] = vec4(0.0); return; }\n" // Nyquist
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

  auto mkRT = [&](vtkSmartPointer<vtkTextureObject> &t) -> bool {
    t = oceanFloatTexture(m_rw, m_n, m_n, 4, nullptr);
    return t != nullptr;
  };
  if (!mkRT(m_h0) || !mkRT(m_a0) || !mkRT(m_b0) || !mkRT(m_a1) || !mkRT(m_b1) || !mkRT(m_disp) ||
      !m_fft.init(m_rw, m_n)) {
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

  m_time0 = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kTime0FS, nullptr);
  m_time1 = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kTime1FS, nullptr);
  m_assemble = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kAssembleFS, nullptr);
  if (!m_time0->Program || !m_time1->Program || !m_assemble->Program) {
    std::fprintf(stderr, "OceanFFT: a shader program failed to build.\n");
    return false;
  }

  m_ready = true;
  uploadH0(); // the held h0, or a flat sea
  return true;
}

bool OceanFFT::setH0(const cvc::ocean::h0_field &h0) {
  if (h0.n != m_n || h0.texels.size() != static_cast<size_t>(m_n) * m_n * 4)
    return false;
  m_h0Host = h0.texels;
  tileSize_m = static_cast<float>(h0.tile_m);
  if (m_ready)
    uploadH0();
  return true;
}

void OceanFFT::uploadH0() {
  m_rw->MakeCurrent();
  std::vector<float> zeros;
  float *data = m_h0Host.data();
  if (m_h0Host.empty()) {
    zeros.assign(static_cast<size_t>(m_n) * m_n * 4, 0.0f);
    data = zeros.data();
  }
  m_h0->Create2DFromRaw(m_n, m_n, 4, VTK_FLOAT, data);
}

void OceanFFT::beginPass(vtkTextureObject *dst) {
  // oceanBeginPass disables the scissor test (the renderer leaves it on, boxed to the window);
  // step() and selfTest() restore it afterwards.
  oceanBeginPass(m_rw, m_fbo, dst, m_n);
}
void OceanFFT::endPass() {}

void OceanFFT::step(double timeSeconds) {
  if (!m_ready)
    return;
  m_rw->MakeCurrent();
  auto *cache = m_rw->GetShaderCache();
  const float t = static_cast<float>(timeSeconds);

  vtkOpenGLState::ScopedglEnableDisable scissor(m_rw->GetState(), kGL_SCISSOR_TEST);
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

  // butterfly IFFT (2 directions x log2N stages) on each buffer, ping-pong: the result lands back
  // in a0 and a1.
  vtkTextureObject *cur0 = m_fft.transform(m_fbo, m_a0, m_b0, m_a0, /*forward=*/false);
  vtkTextureObject *cur1 = m_fft.transform(m_fbo, m_a1, m_b1, m_a1, /*forward=*/false);

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

  vtkOpenGLQuadHelper test(m_rw, kOceanQuadVS, kSelfTestFS, nullptr);
  if (!test.Program)
    return false;

  vtkTextureObject *src = m_a0, *dst = m_b0;
  vtkOpenGLState::ScopedglEnableDisable scissor(m_rw->GetState(), kGL_SCISSOR_TEST);
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
  m_time0.reset();
  m_time1.reset();
  m_assemble.reset();
  m_fft.release();
  m_fbo = nullptr;
  m_h0 = m_a0 = m_b0 = m_a1 = m_b1 = m_disp = nullptr;
  m_ready = false;
}
