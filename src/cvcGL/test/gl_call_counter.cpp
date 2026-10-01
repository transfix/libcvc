/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "gl_call_counter.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <utility>
#include <vtk_glad.h>

namespace cvcgl_test {

namespace {

// Counted and forwarded unchanged.
#define CVC_GL_PLAIN(X)                                                                            \
  X(ActiveTexture)                                                                                 \
  X(BindTexture)                                                                                   \
  X(TexParameteri)                                                                                 \
  X(TexParameterf)                                                                                 \
  X(TexImage2D)                                                                                    \
  X(TexImage3D)                                                                                    \
  X(TexSubImage2D)                                                                                 \
  X(TexSubImage3D)                                                                                 \
  X(TexBuffer)                                                                                     \
  X(GenerateMipmap)                                                                                \
  X(GenTextures)                                                                                   \
  X(DeleteTextures)                                                                                \
  X(BindBuffer)                                                                                    \
  X(BufferData)                                                                                    \
  X(BufferSubData)                                                                                 \
  X(CopyBufferSubData)                                                                             \
  X(GenBuffers)                                                                                    \
  X(DeleteBuffers)                                                                                 \
  X(BindVertexArray)                                                                               \
  X(GenVertexArrays)                                                                               \
  X(DeleteVertexArrays)                                                                            \
  X(EnableVertexAttribArray)                                                                       \
  X(DisableVertexAttribArray)                                                                      \
  X(VertexAttribPointer)                                                                           \
  X(DrawArrays)                                                                                    \
  X(DrawArraysInstanced)                                                                           \
  X(DrawElements)                                                                                  \
  X(DrawRangeElements)                                                                             \
  X(DrawElementsInstanced)                                                                         \
  X(Enable)                                                                                        \
  X(Disable)                                                                                       \
  X(DepthMask)                                                                                     \
  X(DepthFunc)                                                                                     \
  X(ColorMask)                                                                                     \
  X(BlendFunc)                                                                                     \
  X(BlendFuncSeparate)                                                                             \
  X(BlendEquation)                                                                                 \
  X(BlendEquationSeparate)                                                                         \
  X(Viewport)                                                                                      \
  X(Scissor)                                                                                       \
  X(Clear)                                                                                         \
  X(ClearColor)                                                                                    \
  X(ClearDepth)                                                                                    \
  X(ClearDepthf)                                                                                   \
  X(StencilFunc)                                                                                   \
  X(StencilMask)                                                                                   \
  X(StencilOp)                                                                                     \
  X(CullFace)                                                                                      \
  X(PolygonOffset)                                                                                 \
  X(LineWidth)                                                                                     \
  X(PointSize)                                                                                     \
  X(PixelStorei)                                                                                   \
  X(BindFramebuffer)                                                                               \
  X(BindRenderbuffer)                                                                              \
  X(FramebufferTexture2D)                                                                          \
  X(FramebufferRenderbuffer)                                                                       \
  X(BlitFramebuffer)                                                                               \
  X(ReadBuffer)                                                                                    \
  X(DrawBuffer)                                                                                    \
  X(DrawBuffers)                                                                                   \
  X(CreateProgram)                                                                                 \
  X(CreateShader)                                                                                  \
  X(ShaderSource)                                                                                  \
  X(CompileShader)                                                                                 \
  X(AttachShader)                                                                                  \
  X(Flush)                                                                                         \
  X(Finish)

// Round trips on WebGL (the caller waits for the GPU process).
#define CVC_GL_SYNC(X)                                                                             \
  X(GetIntegerv)                                                                                   \
  X(GetFloatv)                                                                                     \
  X(GetBooleanv)                                                                                   \
  X(GetDoublev)                                                                                    \
  X(IsEnabled)                                                                                     \
  X(GetError)                                                                                      \
  X(GetUniformLocation)                                                                            \
  X(GetAttribLocation)                                                                             \
  X(CheckFramebufferStatus)                                                                        \
  X(GetProgramiv)                                                                                  \
  X(GetShaderiv)                                                                                   \
  X(GetString)                                                                                     \
  X(GetStringi)                                                                                    \
  X(GetTexLevelParameteriv)                                                                        \
  X(GetTexImage)                                                                                   \
  X(ReadPixels)                                                                                    \
  X(IsProgram)

// Counted with their own wrappers below.
#define CVC_GL_OWN(X)                                                                              \
  X(UseProgram)                                                                                    \
  X(LinkProgram)                                                                                   \
  X(DeleteProgram)                                                                                 \
  X(Uniform1i)                                                                                     \
  X(Uniform2i)                                                                                     \
  X(Uniform1f)                                                                                     \
  X(Uniform2f)                                                                                     \
  X(Uniform3f)                                                                                     \
  X(Uniform4f)                                                                                     \
  X(Uniform1iv)                                                                                    \
  X(Uniform1fv)                                                                                    \
  X(Uniform2fv)                                                                                    \
  X(Uniform3fv)                                                                                    \
  X(Uniform4fv)                                                                                    \
  X(UniformMatrix3fv)                                                                              \
  X(UniformMatrix4fv)

enum Entry : int {
#define CVC_E(name) E_##name,
  CVC_GL_PLAIN(CVC_E) CVC_GL_SYNC(CVC_E) CVC_GL_OWN(CVC_E)
#undef CVC_E
      E_COUNT
};

const char *const g_names[E_COUNT] = {
#define CVC_N(name) #name,
    CVC_GL_PLAIN(CVC_N) CVC_GL_SYNC(CVC_N) CVC_GL_OWN(CVC_N)
#undef CVC_N
};

double g_n[E_COUNT];
double g_uniformRedundant = 0;

bool isSync(int i) { return i >= E_GetIntegerv && i <= E_IsProgram; }
bool isUniform(int i) { return i >= E_Uniform1i && i <= E_UniformMatrix4fv; }

template <int Id, typename T> struct Counted;
template <int Id, typename R, typename... A> struct Counted<Id, R(GLAD_API_PTR *)(A...)> {
  static inline R(GLAD_API_PTR *orig)(A...) = nullptr;
  static R GLAD_API_PTR fn(A... a) {
    g_n[Id] += 1;
    return orig(a...);
  }
  static void hook(R(GLAD_API_PTR *&ptr)(A...)) {
    if (ptr && ptr != &fn) {
      orig = ptr;
      ptr = &fn;
    }
  }
};

// Uniform values are per program: (program, location) -> last bytes sent.
GLuint g_program = 0;
std::map<std::pair<GLuint, GLint>, std::string> g_last;

void noteUniform(GLint loc, const void *p, size_t bytes) {
  std::string v(static_cast<const char *>(p), bytes);
  auto key = std::make_pair(g_program, loc);
  auto it = g_last.find(key);
  if (it != g_last.end() && it->second == v)
    g_uniformRedundant += 1;
  else
    g_last[key] = std::move(v);
}
void forgetProgram(GLuint p) {
  for (auto it = g_last.begin(); it != g_last.end();)
    it = it->first.first == p ? g_last.erase(it) : std::next(it);
}

#define CVC_O(name) decltype(glad_gl##name) o_##name = nullptr;
CVC_GL_OWN(CVC_O)
#undef CVC_O

void GLAD_API_PTR w_UseProgram(GLuint p) {
  g_n[E_UseProgram] += 1;
  g_program = p;
  o_UseProgram(p);
}
void GLAD_API_PTR w_LinkProgram(GLuint p) {
  g_n[E_LinkProgram] += 1;
  forgetProgram(p);
  o_LinkProgram(p);
}
void GLAD_API_PTR w_DeleteProgram(GLuint p) {
  g_n[E_DeleteProgram] += 1;
  forgetProgram(p);
  o_DeleteProgram(p);
}
void GLAD_API_PTR w_Uniform1i(GLint l, GLint a) {
  g_n[E_Uniform1i] += 1;
  noteUniform(l, &a, sizeof a);
  o_Uniform1i(l, a);
}
void GLAD_API_PTR w_Uniform2i(GLint l, GLint a, GLint b) {
  g_n[E_Uniform2i] += 1;
  const GLint v[2] = {a, b};
  noteUniform(l, v, sizeof v);
  o_Uniform2i(l, a, b);
}
void GLAD_API_PTR w_Uniform1f(GLint l, GLfloat a) {
  g_n[E_Uniform1f] += 1;
  noteUniform(l, &a, sizeof a);
  o_Uniform1f(l, a);
}
void GLAD_API_PTR w_Uniform2f(GLint l, GLfloat a, GLfloat b) {
  g_n[E_Uniform2f] += 1;
  const GLfloat v[2] = {a, b};
  noteUniform(l, v, sizeof v);
  o_Uniform2f(l, a, b);
}
void GLAD_API_PTR w_Uniform3f(GLint l, GLfloat a, GLfloat b, GLfloat c) {
  g_n[E_Uniform3f] += 1;
  const GLfloat v[3] = {a, b, c};
  noteUniform(l, v, sizeof v);
  o_Uniform3f(l, a, b, c);
}
void GLAD_API_PTR w_Uniform4f(GLint l, GLfloat a, GLfloat b, GLfloat c, GLfloat d) {
  g_n[E_Uniform4f] += 1;
  const GLfloat v[4] = {a, b, c, d};
  noteUniform(l, v, sizeof v);
  o_Uniform4f(l, a, b, c, d);
}
void GLAD_API_PTR w_Uniform1iv(GLint l, GLsizei c, const GLint *v) {
  g_n[E_Uniform1iv] += 1;
  noteUniform(l, v, sizeof(GLint) * c);
  o_Uniform1iv(l, c, v);
}
void GLAD_API_PTR w_Uniform1fv(GLint l, GLsizei c, const GLfloat *v) {
  g_n[E_Uniform1fv] += 1;
  noteUniform(l, v, sizeof(GLfloat) * c);
  o_Uniform1fv(l, c, v);
}
void GLAD_API_PTR w_Uniform2fv(GLint l, GLsizei c, const GLfloat *v) {
  g_n[E_Uniform2fv] += 1;
  noteUniform(l, v, 2 * sizeof(GLfloat) * c);
  o_Uniform2fv(l, c, v);
}
void GLAD_API_PTR w_Uniform3fv(GLint l, GLsizei c, const GLfloat *v) {
  g_n[E_Uniform3fv] += 1;
  noteUniform(l, v, 3 * sizeof(GLfloat) * c);
  o_Uniform3fv(l, c, v);
}
void GLAD_API_PTR w_Uniform4fv(GLint l, GLsizei c, const GLfloat *v) {
  g_n[E_Uniform4fv] += 1;
  noteUniform(l, v, 4 * sizeof(GLfloat) * c);
  o_Uniform4fv(l, c, v);
}
void GLAD_API_PTR w_UniformMatrix3fv(GLint l, GLsizei c, GLboolean t, const GLfloat *v) {
  g_n[E_UniformMatrix3fv] += 1;
  noteUniform(l, v, 9 * sizeof(GLfloat) * c);
  o_UniformMatrix3fv(l, c, t, v);
}
void GLAD_API_PTR w_UniformMatrix4fv(GLint l, GLsizei c, GLboolean t, const GLfloat *v) {
  g_n[E_UniformMatrix4fv] += 1;
  noteUniform(l, v, 16 * sizeof(GLfloat) * c);
  o_UniformMatrix4fv(l, c, t, v);
}

} // namespace

void glcallsInstall() {
#define CVC_H(name) Counted<E_##name, decltype(glad_gl##name)>::hook(glad_gl##name);
  CVC_GL_PLAIN(CVC_H)
  CVC_GL_SYNC(CVC_H)
#undef CVC_H
#define CVC_W(name)                                                                                \
  if (glad_gl##name && glad_gl##name != w_##name) {                                                \
    o_##name = glad_gl##name;                                                                      \
    glad_gl##name = w_##name;                                                                      \
  }
  CVC_GL_OWN(CVC_W)
#undef CVC_W
}

GLCalls glcallsRead() {
  GLCalls r;
  r.n.assign(g_n, g_n + E_COUNT);
  r.uniformRedundant = g_uniformRedundant;
  return r;
}

int glcallsEntries() { return E_COUNT; }
const char *glcallsName(int i) { return i >= 0 && i < E_COUNT ? g_names[i] : "?"; }

GLCalls GLCalls::operator-(const GLCalls &o) const {
  GLCalls r;
  r.n.resize(n.size());
  for (size_t i = 0; i < n.size(); ++i)
    r.n[i] = n[i] - (i < o.n.size() ? o.n[i] : 0.0);
  r.uniformRedundant = uniformRedundant - o.uniformRedundant;
  return r;
}

GLCalls &GLCalls::operator+=(const GLCalls &o) {
  if (n.size() < o.n.size())
    n.resize(o.n.size(), 0.0);
  for (size_t i = 0; i < o.n.size(); ++i)
    n[i] += o.n[i];
  uniformRedundant += o.uniformRedundant;
  return *this;
}

double GLCalls::total() const {
  double s = 0;
  for (double v : n)
    s += v;
  return s;
}

double GLCalls::count(const std::string &entry) const {
  for (int i = 0; i < E_COUNT && i < static_cast<int>(n.size()); ++i)
    if (entry == g_names[i])
      return n[i];
  return 0;
}

double GLCalls::uniforms() const {
  double s = 0;
  for (int i = 0; i < static_cast<int>(n.size()); ++i)
    s += isUniform(i) ? n[i] : 0.0;
  return s;
}

double GLCalls::sync() const {
  double s = 0;
  for (int i = 0; i < static_cast<int>(n.size()); ++i)
    s += isSync(i) ? n[i] : 0.0;
  return s;
}

bool GLCalls::sameAs(const GLCalls &o, std::string *firstDifference) const {
  for (int i = 0; i < E_COUNT; ++i) {
    const double a = i < static_cast<int>(n.size()) ? n[i] : 0.0;
    const double b = i < static_cast<int>(o.n.size()) ? o.n[i] : 0.0;
    if (a != b) {
      if (firstDifference) {
        char buf[128];
        std::snprintf(buf, sizeof buf, "gl%s %g vs %g", g_names[i], a, b);
        *firstDifference = buf;
      }
      return false;
    }
  }
  return true;
}

std::string GLCalls::str(double per, int top) const {
  if (per <= 0)
    per = 1;
  char buf[160];
  std::snprintf(buf, sizeof buf, "total %.1f (uniforms %.1f, redundant %.1f, sync %.1f) |",
                total() / per, uniforms() / per, uniformRedundant / per, sync() / per);
  std::string out = buf;
  std::vector<std::pair<double, int>> v;
  for (int i = 0; i < static_cast<int>(n.size()); ++i)
    if (n[i] != 0)
      v.emplace_back(n[i], i);
  std::sort(v.begin(), v.end(), [](const auto &a, const auto &b) {
    return a.first != b.first ? a.first > b.first : a.second < b.second;
  });
  for (int k = 0; k < static_cast<int>(v.size()) && k < top; ++k) {
    std::snprintf(buf, sizeof buf, " %s=%.2f", g_names[v[k].second], v[k].first / per);
    out += buf;
  }
  return out;
}

} // namespace cvcgl_test
