/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "gl_upload_counter.h"

#include <cstdio>
#include <vtk_glad.h>

namespace cvcgl_test {

namespace {
double g_v[K_N];

int bytesPerTexel(GLenum format, GLenum type) {
  int comps = 4;
  switch (format) {
  case GL_RED:
  case GL_RED_INTEGER:
  case GL_DEPTH_COMPONENT:
    comps = 1;
    break;
  case GL_RG:
  case GL_RG_INTEGER:
    comps = 2;
    break;
  case GL_RGB:
  case GL_RGB_INTEGER:
    comps = 3;
    break;
  case GL_DEPTH_STENCIL:
    return 4;
  default:
    comps = 4;
  }
  int size = 1;
  switch (type) {
  case GL_FLOAT:
  case GL_INT:
  case GL_UNSIGNED_INT:
    size = 4;
    break;
  case GL_HALF_FLOAT:
  case GL_SHORT:
  case GL_UNSIGNED_SHORT:
    size = 2;
    break;
  default:
    size = 1;
  }
  return comps * size;
}

#define CVC_GLC_DECL(name) decltype(glad_gl##name) o_##name = nullptr;
CVC_GLC_DECL(BufferData)
CVC_GLC_DECL(BufferSubData)
CVC_GLC_DECL(TexImage2D)
CVC_GLC_DECL(TexSubImage2D)
CVC_GLC_DECL(CopyBufferSubData)
CVC_GLC_DECL(GenBuffers)
CVC_GLC_DECL(GenTextures)
CVC_GLC_DECL(DrawArrays)
CVC_GLC_DECL(DrawElements)
CVC_GLC_DECL(DrawRangeElements)
CVC_GLC_DECL(DrawArraysInstanced)
CVC_GLC_DECL(DrawElementsInstanced)
#undef CVC_GLC_DECL

void GLAD_API_PTR w_BufferData(GLenum t, GLsizeiptr s, const void *d, GLenum u) {
  g_v[K_BUFDATA] += 1;
  g_v[K_BUFDATA_B] += static_cast<double>(d ? s : 0);
  o_BufferData(t, s, d, u);
}
void GLAD_API_PTR w_BufferSubData(GLenum t, GLintptr o, GLsizeiptr s, const void *d) {
  g_v[K_BUFSUB] += 1;
  g_v[K_BUFSUB_B] += static_cast<double>(s);
  o_BufferSubData(t, o, s, d);
}
void GLAD_API_PTR w_TexImage2D(GLenum t, GLint l, GLint ifmt, GLsizei w, GLsizei h, GLint b,
                               GLenum f, GLenum ty, const void *p) {
  g_v[K_TEXIMG] += 1;
  g_v[K_TEXIMG_B] += p ? static_cast<double>(w) * h * bytesPerTexel(f, ty) : 0.0;
  o_TexImage2D(t, l, ifmt, w, h, b, f, ty, p);
}
void GLAD_API_PTR w_TexSubImage2D(GLenum t, GLint l, GLint x, GLint y, GLsizei w, GLsizei h,
                                  GLenum f, GLenum ty, const void *p) {
  g_v[K_TEXSUB] += 1;
  g_v[K_TEXSUB_B] += static_cast<double>(w) * h * bytesPerTexel(f, ty);
  o_TexSubImage2D(t, l, x, y, w, h, f, ty, p);
}
void GLAD_API_PTR w_CopyBufferSubData(GLenum a, GLenum b, GLintptr ro, GLintptr wo, GLsizeiptr s) {
  g_v[K_COPYBUF] += 1;
  g_v[K_COPYBUF_B] += static_cast<double>(s);
  o_CopyBufferSubData(a, b, ro, wo, s);
}
void GLAD_API_PTR w_GenBuffers(GLsizei n, GLuint *b) {
  g_v[K_GENBUF] += n;
  o_GenBuffers(n, b);
}
void GLAD_API_PTR w_GenTextures(GLsizei n, GLuint *b) {
  g_v[K_GENTEX] += n;
  o_GenTextures(n, b);
}
void GLAD_API_PTR w_DrawArrays(GLenum m, GLint f, GLsizei c) {
  g_v[K_DRAWS] += 1;
  o_DrawArrays(m, f, c);
}
void GLAD_API_PTR w_DrawElements(GLenum m, GLsizei c, GLenum t, const void *i) {
  g_v[K_DRAWS] += 1;
  o_DrawElements(m, c, t, i);
}
void GLAD_API_PTR w_DrawRangeElements(GLenum m, GLuint s, GLuint e, GLsizei c, GLenum t,
                                      const void *i) {
  g_v[K_DRAWS] += 1;
  o_DrawRangeElements(m, s, e, c, t, i);
}
void GLAD_API_PTR w_DrawArraysInstanced(GLenum m, GLint f, GLsizei c, GLsizei n) {
  g_v[K_DRAWS] += 1;
  o_DrawArraysInstanced(m, f, c, n);
}
void GLAD_API_PTR w_DrawElementsInstanced(GLenum m, GLsizei c, GLenum t, const void *i, GLsizei n) {
  g_v[K_DRAWS] += 1;
  o_DrawElementsInstanced(m, c, t, i, n);
}
} // namespace

#define CVC_GLC_HOOK(name)                                                                         \
  if (glad_gl##name && glad_gl##name != w_##name) {                                                \
    o_##name = glad_gl##name;                                                                      \
    glad_gl##name = w_##name;                                                                      \
  }

void glcountInstall(){
    CVC_GLC_HOOK(BufferData) CVC_GLC_HOOK(BufferSubData) CVC_GLC_HOOK(TexImage2D)
        CVC_GLC_HOOK(TexSubImage2D) CVC_GLC_HOOK(CopyBufferSubData) CVC_GLC_HOOK(GenBuffers)
            CVC_GLC_HOOK(GenTextures) CVC_GLC_HOOK(DrawArrays) CVC_GLC_HOOK(DrawElements)
                CVC_GLC_HOOK(DrawRangeElements) CVC_GLC_HOOK(DrawArraysInstanced)
                    CVC_GLC_HOOK(DrawElementsInstanced)}
#undef CVC_GLC_HOOK

GLCount glcountRead() {
  GLCount r;
  for (int i = 0; i < K_N; ++i)
    r.v[i] = g_v[i];
  return r;
}

std::string glcountStr(const GLCount &d) {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "bufferData %g (%g B) bufferSubData %g (%g B) texImage2D %g (%g B) "
                "texSubImage2D %g (%g B) copyBuffer %g (%g B) createBuffer %g createTexture %g "
                "draws %g",
                d.v[K_BUFDATA], d.v[K_BUFDATA_B], d.v[K_BUFSUB], d.v[K_BUFSUB_B], d.v[K_TEXIMG],
                d.v[K_TEXIMG_B], d.v[K_TEXSUB], d.v[K_TEXSUB_B], d.v[K_COPYBUF], d.v[K_COPYBUF_B],
                d.v[K_GENBUF], d.v[K_GENTEX], d.v[K_DRAWS]);
  return buf;
}

} // namespace cvcgl_test
