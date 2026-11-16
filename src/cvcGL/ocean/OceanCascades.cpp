/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc. LGPL-2.1. See OceanCascades.h and OceanFFT.h for the ABYSSAL (MIT)
  attribution that applies to the spectrum / IFFT port.
*/

#include "OceanCascades.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ocean/h0.h>
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

// The time evolution of every cascade into both of its buffers (OceanFFT's kTime0FS and kTime1FS):
// tile t is cascade t / 2's buffer t % 2.
const char *const kTimeFS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uH0;\n"
    "uniform int uN;\n"
    "uniform vec3 uL;\n"
    "uniform float uG, uTime, uDepth;\n"
    "//VTK::Output::Dec\n"
    "const float TAU = 6.28318530718;\n"
    "vec2 cmul(vec2 a, vec2 b){ return vec2(a.x*b.x-a.y*b.y, a.x*b.y+a.y*b.x); }\n"
    "void main(){\n"
    "  ivec2 p = ivec2(gl_FragCoord.xy);\n"
    "  int t = p.x / uN;\n"
    "  int c = t / 2, buf = t - 2*c;\n"
    "  ivec2 lc = ivec2(p.x - t*uN, p.y);\n"
    "  float L = c == 0 ? uL.x : (c == 1 ? uL.y : uL.z);\n"
    "  vec2 kv = TAU/L * (vec2(lc) - float(uN)*0.5);\n"
    "  float kl = length(kv);\n"
    "  vec4 h0 = texelFetch(uH0, ivec2(c*uN + lc.x, lc.y), 0);\n"
    // the Nyquist row and column have no partner -k: dropped, as OceanFFT drops them
    "  if (kl < 1e-6 || lc.x == 0 || lc.y == 0) { gl_FragData[0] = vec4(0.0); return; }\n"
    "  vec2 kn = kv/kl;\n"
    "  float w = sqrt(uG*kl*tanh(min(kl*uDepth,20.0)));\n"
    "  float ph = w*uTime; vec2 e = vec2(cos(ph), sin(ph)); vec2 ec = vec2(e.x, -e.y);\n"
    "  vec2 h  = cmul(h0.xy, e) + cmul(h0.zw, ec);\n"
    "  vec2 ih = vec2(-h.y, h.x);\n"
    "  if (buf == 0) {\n" // (Dx + i Dz, Dy + i DyDx)
    "    vec2 Dx = ih*kn.x, Dz = ih*kn.y, Dy = h, DyDx = ih*kv.x;\n"
    "    gl_FragData[0] = vec4(Dx.x - Dz.y, Dx.y + Dz.x, Dy.x - DyDx.y, Dy.y + DyDx.x);\n"
    "  } else {\n" // (DyDz + i DxDx, DzDz + i DxDz): the Jacobian's
    "    vec2 DyDz = ih*kv.y;\n"
    "    vec2 DxDx = -h*(kv.x*kn.x), DzDz = -h*(kv.y*kn.y), DxDz = -h*(kv.y*kn.x);\n"
    "    gl_FragData[0] = vec4(DyDz.x - DxDx.y, DyDz.y + DxDx.x, DzDz.x - DxDz.y, DzDz.y + "
    "DxDz.x);\n"
    "  }\n"
    "}\n";

// Every cascade's displacement and Jacobian from its two transformed buffers (OceanFFT's
// kAssembleFS).
const char *const kAssembleFS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uBuf;\n"
    "uniform int uN;\n"
    "uniform float uLambda;\n"
    "//VTK::Output::Dec\n"
    "void main(){\n"
    "  ivec2 p = ivec2(gl_FragCoord.xy);\n"
    "  int c = p.x / uN;\n"
    "  ivec2 lc = ivec2(p.x - c*uN, p.y);\n"
    "  float perm = (((lc.x + lc.y) & 1) == 0) ? 1.0 : -1.0;\n"
    "  vec4 b0 = texelFetch(uBuf, ivec2(2*c*uN + lc.x, lc.y), 0) * perm;\n"
    "  vec4 b1 = texelFetch(uBuf, ivec2((2*c + 1)*uN + lc.x, lc.y), 0) * perm;\n"
    "  float Dx = b0.x, Dz = b0.y, Dy = b0.z;\n"
    "  float DxDx = b1.y, DzDz = b1.z, DxDz = b1.w;\n"
    "  float lx = uLambda*DxDx, lz = uLambda*DzDz, lxz = uLambda*DxDz;\n"
    "  float jac = (1.0+lx)*(1.0+lz) - lxz*lxz;\n"
    "  gl_FragData[0] = vec4(uLambda*Dx, Dy, uLambda*Dz, jac);\n"
    "}\n";
} // namespace

OceanCascades::OceanCascades(int n, int cascades)
    : m_n(n), m_c(std::clamp(cascades, 1, 3)), m_active(m_c), m_tile(std::size_t(m_c), 200.0),
      m_h0Host(std::size_t(m_c)) {
  m_log2n = static_cast<int>(std::lround(std::log2(static_cast<double>(n))));
}
OceanCascades::~OceanCascades() { releaseResources(); }

bool OceanCascades::init(vtkOpenGLRenderWindow *rw) {
  if (!rw)
    return false;
  m_rw = rw;
  m_rw->MakeCurrent();
  m_h0 = oceanFloatTexture(m_rw, m_n * m_c, m_n, 4, nullptr);
  m_a = oceanFloatTexture(m_rw, m_n * m_c * 2, m_n, 4, nullptr);
  m_b = oceanFloatTexture(m_rw, m_n * m_c * 2, m_n, 4, nullptr);
  m_disp = oceanFloatTexture(m_rw, m_n * m_c, m_n, 4, nullptr);
  if (!m_h0 || !m_a || !m_b || !m_disp || !m_fft.init(m_rw, m_n, 2 * m_c)) {
    std::fprintf(stderr, "OceanCascades: RGBA32F texture allocation failed.\n");
    return false;
  }
  m_fbo = vtkSmartPointer<vtkOpenGLFramebufferObject>::New();
  m_fbo->SetContext(m_rw);
  m_time = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kTimeFS, nullptr);
  m_assemble = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kAssembleFS, nullptr);
  if (!m_time->Program || !m_assemble->Program) {
    std::fprintf(stderr, "OceanCascades: a shader program failed to build.\n");
    return false;
  }
  m_ready = true;
  setActive(m_active);
  uploadH0();
  return true;
}

void OceanCascades::setActive(int k) {
  m_active = std::clamp(k, 1, m_c);
  m_fft.setActiveTiles(2 * m_active);
}

bool OceanCascades::setH0(int c, const cvc::ocean::h0_field &h0) {
  if (c < 0 || c >= m_c || h0.n != m_n || h0.texels.size() != std::size_t(m_n) * m_n * 4)
    return false;
  m_h0Host[std::size_t(c)] = h0.texels;
  m_tile[std::size_t(c)] = h0.tile_m;
  if (m_ready)
    uploadH0();
  return true;
}

void OceanCascades::uploadH0() {
  m_rw->MakeCurrent();
  const std::size_t row = std::size_t(m_n) * 4, wide = row * std::size_t(m_c);
  std::vector<float> atlas(wide * std::size_t(m_n), 0.0f);
  for (int c = 0; c < m_c; ++c) {
    const std::vector<float> &h = m_h0Host[std::size_t(c)];
    if (h.empty())
      continue; // a flat sea
    for (int y = 0; y < m_n; ++y)
      std::copy(h.begin() + std::ptrdiff_t(std::size_t(y) * row),
                h.begin() + std::ptrdiff_t(std::size_t(y + 1) * row),
                atlas.begin() + std::ptrdiff_t(std::size_t(y) * wide + std::size_t(c) * row));
  }
  m_h0->Create2DFromRaw(m_n * m_c, m_n, 4, VTK_FLOAT, atlas.data());
}

void OceanCascades::step(double timeSeconds) {
  if (!m_ready)
    return;
  m_rw->MakeCurrent();
  auto *cache = m_rw->GetShaderCache();
  vtkOpenGLState::ScopedglEnableDisable scissor(m_rw->GetState(), kGL_SCISSOR_TEST);
  m_rw->GetState()->PushFramebufferBindings();

  // every buffer of every active cascade, one pass
  oceanBeginPass(m_rw, m_fbo, m_a, m_n);
  m_rw->GetState()->vtkglViewport(0, 0, m_n * m_active * 2, m_n);
  cache->ReadyShaderProgram(m_time->Program);
  vtkShaderProgram *p = m_time->Program;
  m_h0->Activate();
  p->SetUniformi("uH0", m_h0->GetTextureUnit());
  p->SetUniformi("uN", m_n);
  float tiles[3] = {float(m_tile[0]), float(m_c > 1 ? m_tile[1] : m_tile[0]),
                    float(m_c > 2 ? m_tile[2] : m_tile[0])};
  p->SetUniform3f("uL", tiles);
  p->SetUniformf("uG", kG);
  p->SetUniformf("uTime", static_cast<float>(timeSeconds));
  p->SetUniformf("uDepth", depth_m);
  m_time->Render();
  m_h0->Deactivate();
  ++m_passes;

  // every tile's inverse transform (the result lands back in a)
  vtkTextureObject *buf = m_fft.transform(m_fbo, m_a, m_b, m_a, /*forward=*/false);
  m_passes += m_fft.passes();

  // every active cascade's displacement, one pass
  oceanBeginPass(m_rw, m_fbo, m_disp, m_n);
  m_rw->GetState()->vtkglViewport(0, 0, m_n * m_active, m_n);
  cache->ReadyShaderProgram(m_assemble->Program);
  p = m_assemble->Program;
  buf->Activate();
  p->SetUniformi("uBuf", buf->GetTextureUnit());
  p->SetUniformi("uN", m_n);
  p->SetUniformf("uLambda", chop);
  m_assemble->Render();
  buf->Deactivate();
  ++m_passes;

  m_rw->GetState()->PopFramebufferBindings();
}

vtkTextureObject *OceanCascades::displacement() const { return m_ready ? m_disp.Get() : nullptr; }

std::vector<float> OceanCascades::readbackDisplacement() const {
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
    out.assign(data, data + std::size_t(m_n) * m_c * m_n * 4);
    pbo->UnmapPackedBuffer();
  }
  return out;
}

void OceanCascades::releaseResources() {
  m_time.reset();
  m_assemble.reset();
  m_fft.release();
  m_fbo = nullptr;
  m_h0 = m_a = m_b = m_disp = nullptr;
  m_ready = false;
}
