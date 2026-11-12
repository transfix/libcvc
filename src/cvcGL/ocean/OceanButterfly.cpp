/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc. LGPL-2.1. See OceanButterfly.h for the ABYSSAL
  (MIT) attribution that applies to the butterfly port.
*/

#include "OceanButterfly.h"

#include <cmath>
#include <utility>
#include <vector>
#include <vtkOpenGLFramebufferObject.h>
#include <vtkOpenGLQuadHelper.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLShaderCache.h>
#include <vtkOpenGLState.h>
#include <vtkShaderProgram.h>
#include <vtkTextureObject.h>

namespace {
constexpr unsigned int kGL_BLEND = 0x0BE2;
constexpr unsigned int kGL_DEPTH_TEST = 0x0B71;
constexpr unsigned int kGL_SCISSOR_TEST = 0x0C11;
constexpr float kTAU = 6.28318530718f;

// ── one butterfly stage (2 complex per texel: .rg and .ba) ────────────────────
// The LUT holds the inverse's twiddle exp(+i ang); the forward transform conjugates it, which is
// exact, so the two directions use the same table.
const char *kButterflyFS =
    "//VTK::System::Dec\n"
    "uniform sampler2D uSrc, uButterfly;\n"
    "uniform int uStage, uVertical, uForward;\n"
    "//VTK::Output::Dec\n"
    "vec2 cmul(vec2 a, vec2 b){ return vec2(a.x*b.x-a.y*b.y, a.x*b.y+a.y*b.x); }\n"
    "void main(){\n"
    "  ivec2 c = ivec2(gl_FragCoord.xy);\n"
    "  int idx = (uVertical == 0) ? c.x : c.y;\n"
    "  vec4 bf = texelFetch(uButterfly, ivec2(uStage, idx), 0);\n"
    "  vec2 w = (uForward != 0) ? vec2(bf.x, -bf.y) : bf.xy;\n"
    "  int topI = int(bf.z + 0.5); int botI = int(bf.w + 0.5);\n"
    "  vec4 a, b;\n"
    "  if (uVertical == 0) { a = texelFetch(uSrc, ivec2(topI, c.y), 0); b = texelFetch(uSrc, "
    "ivec2(botI, c.y), 0); }\n"
    "  else                { a = texelFetch(uSrc, ivec2(c.x, topI), 0); b = texelFetch(uSrc, "
    "ivec2(c.x, botI), 0); }\n"
    "  gl_FragData[0] = vec4(a.rg + cmul(w, b.rg), a.ba + cmul(w, b.ba));\n"
    "}\n";
} // namespace

// Own VS + integer-coord texelFetch so no interpolated varying can mismatch VTK's
// internal quad shader. vtkOpenGLQuadHelper's VAO UNCONDITIONALLY binds both
// ndCoordIn AND texCoordIn (vtkOpenGLQuadHelper.cxx), so a VS that omits texCoordIn
// makes it log "Error binding texCoordIn to VAO" (a vtkOpenGLVertexArrayObject ERR)
// on every pass — harmless but noisy. Declare texCoordIn and give it a use the
// compiler cannot fold away (a *1e-30 term, far below float precision against the
// ±1 ndCoords, so the full-screen quad is unchanged) so the attribute stays live and
// the VAO binds cleanly. The passes still read nothing from it (texelFetch by
// gl_FragCoord).
const char *const kOceanQuadVS =
    "//VTK::System::Dec\n"
    "in vec4 ndCoordIn;\n"
    "in vec2 texCoordIn;\n"
    "void main() { gl_Position = ndCoordIn + vec4(texCoordIn * 1e-30, 0.0, 0.0); }\n";

void oceanBeginPass(vtkOpenGLRenderWindow *rw, vtkOpenGLFramebufferObject *fbo,
                    vtkTextureObject *dst, int n) {
  fbo->Bind();
  fbo->AddColorAttachment(0, dst);
  fbo->ActivateDrawBuffers(1);
  auto *st = rw->GetState();
  st->vtkglViewport(0, 0, n, n);
  st->vtkglDisable(kGL_SCISSOR_TEST);
  st->vtkglDisable(kGL_BLEND);
  st->vtkglDisable(kGL_DEPTH_TEST);
}

vtkSmartPointer<vtkTextureObject> oceanFloatTexture(vtkOpenGLRenderWindow *rw, int w, int h,
                                                    int comps, const float *seed) {
  auto t = vtkSmartPointer<vtkTextureObject>::New();
  t->SetContext(rw);
  const bool ok = seed ? t->Create2DFromRaw(w, h, comps, VTK_FLOAT, const_cast<float *>(seed))
                       : t->Create2D(w, h, comps, VTK_FLOAT, false);
  if (!ok)
    return nullptr;
  t->SetWrapS(vtkTextureObject::ClampToEdge);
  t->SetWrapT(vtkTextureObject::ClampToEdge);
  t->SetMinificationFilter(vtkTextureObject::Nearest);
  t->SetMagnificationFilter(vtkTextureObject::Nearest);
  return t;
}

OceanButterfly::OceanButterfly() = default;
OceanButterfly::~OceanButterfly() { release(); }

bool OceanButterfly::init(vtkOpenGLRenderWindow *rw, int n) {
  release();
  if (!rw || n < 2 || (n & (n - 1)) != 0)
    return false;
  m_rw = rw;
  m_n = n;
  m_log2n = static_cast<int>(std::lround(std::log2(static_cast<double>(n))));

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
  m_lut = oceanFloatTexture(m_rw, m_log2n, m_n, 4, lut.data());
  if (!m_lut)
    return false;
  m_quad = std::make_unique<vtkOpenGLQuadHelper>(m_rw, kOceanQuadVS, kButterflyFS, nullptr);
  if (!m_quad->Program) {
    release();
    return false;
  }
  return true;
}

void OceanButterfly::release() {
  m_quad.reset();
  m_lut = nullptr;
}

void OceanButterfly::pass(vtkOpenGLFramebufferObject *fbo, vtkTextureObject *src,
                          vtkTextureObject *dst, int stage, bool vertical, bool forward) {
  oceanBeginPass(m_rw, fbo, dst, m_n);
  m_rw->GetShaderCache()->ReadyShaderProgram(m_quad->Program);
  src->Activate();
  m_lut->Activate();
  vtkShaderProgram *p = m_quad->Program;
  p->SetUniformi("uSrc", src->GetTextureUnit());
  p->SetUniformi("uButterfly", m_lut->GetTextureUnit());
  p->SetUniformi("uStage", stage);
  p->SetUniformi("uVertical", vertical ? 1 : 0);
  p->SetUniformi("uForward", forward ? 1 : 0);
  m_quad->Render();
  src->Deactivate();
  m_lut->Deactivate();
}

vtkTextureObject *OceanButterfly::transform(vtkOpenGLFramebufferObject *fbo, vtkTextureObject *src,
                                            vtkTextureObject *a, vtkTextureObject *b,
                                            bool forward) {
  vtkTextureObject *in = src, *out = a, *next = b;
  for (int vert = 0; vert < 2; ++vert)
    for (int stage = 0; stage < m_log2n; ++stage) {
      pass(fbo, in, out, stage, vert == 1, forward);
      in = out;
      std::swap(out, next);
    }
  return in; // the last pass wrote b (an even pass count)
}
