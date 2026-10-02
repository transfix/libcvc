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
#include <type_traits>
#include <utility>
#include <vtk_glad.h>

namespace cvcgl_test {

namespace {

// ── the wrapped entry points ────────────────────────────────────────────────
// Every GL entry point VTK 9.5.0's Rendering/OpenGL2, Rendering/VolumeOpenGL2
// and Rendering/Core call (a grep of their sources against glad), cvcGL's own,
// and the GLES3/GL4 core families those may grow into.

// Counted and traced with no context.
#define CVC_GL_PLAIN(X)                                                                            \
  X(AttachShader)                                                                                  \
  X(BeginQuery)                                                                                    \
  X(BeginQueryIndexed)                                                                             \
  X(BeginTransformFeedback)                                                                        \
  X(BindBuffer)                                                                                    \
  X(BindBufferBase)                                                                                \
  X(BindBufferRange)                                                                               \
  X(BindFragDataLocation)                                                                          \
  X(BindFramebuffer)                                                                               \
  X(BindRenderbuffer)                                                                              \
  X(BindVertexArray)                                                                               \
  X(BlendEquation)                                                                                 \
  X(BlendEquationSeparate)                                                                         \
  X(BlendFunc)                                                                                     \
  X(BlendFuncSeparate)                                                                             \
  X(BlitFramebuffer)                                                                               \
  X(BufferData)                                                                                    \
  X(BufferSubData)                                                                                 \
  X(ClampColor)                                                                                    \
  X(Clear)                                                                                         \
  X(ClearBufferfi)                                                                                 \
  X(ClearBufferfv)                                                                                 \
  X(ClearBufferiv)                                                                                 \
  X(ClearBufferuiv)                                                                                \
  X(ClearColor)                                                                                    \
  X(ClearDepth)                                                                                    \
  X(ClearDepthf)                                                                                   \
  X(ClearStencil)                                                                                  \
  X(ColorMask)                                                                                     \
  X(ColorMaski)                                                                                    \
  X(CompileShader)                                                                                 \
  X(CopyBufferSubData)                                                                             \
  X(CreateProgram)                                                                                 \
  X(CreateShader)                                                                                  \
  X(CullFace)                                                                                      \
  X(DebugMessageCallback)                                                                          \
  X(DebugMessageControl)                                                                           \
  X(DebugMessageInsert)                                                                            \
  X(DeleteBuffers)                                                                                 \
  X(DeleteFramebuffers)                                                                            \
  X(DeleteQueries)                                                                                 \
  X(DeleteRenderbuffers)                                                                           \
  X(DeleteShader)                                                                                  \
  X(DeleteSync)                                                                                    \
  X(DeleteTextures)                                                                                \
  X(DeleteVertexArrays)                                                                            \
  X(DepthFunc)                                                                                     \
  X(DepthMask)                                                                                     \
  X(DetachShader)                                                                                  \
  X(Disable)                                                                                       \
  X(DisableVertexAttribArray)                                                                      \
  X(Disablei)                                                                                      \
  X(DrawBuffer)                                                                                    \
  X(DrawBuffers)                                                                                   \
  X(Enable)                                                                                        \
  X(EnableVertexAttribArray)                                                                       \
  X(Enablei)                                                                                       \
  X(EndQuery)                                                                                      \
  X(EndQueryIndexed)                                                                               \
  X(EndTransformFeedback)                                                                          \
  X(FenceSync)                                                                                     \
  X(Flush)                                                                                         \
  X(FramebufferRenderbuffer)                                                                       \
  X(FramebufferTexture2D)                                                                          \
  X(FramebufferTexture3D)                                                                          \
  X(FramebufferTextureLayer)                                                                       \
  X(GenBuffers)                                                                                    \
  X(GenFramebuffers)                                                                               \
  X(GenQueries)                                                                                    \
  X(GenRenderbuffers)                                                                              \
  X(GenTextures)                                                                                   \
  X(GenVertexArrays)                                                                               \
  X(InvalidateFramebuffer)                                                                         \
  X(LineWidth)                                                                                     \
  X(MemoryBarrier)                                                                                 \
  X(PatchParameteri)                                                                               \
  X(PixelStorei)                                                                                   \
  X(PointSize)                                                                                     \
  X(PolygonOffset)                                                                                 \
  X(QueryCounter)                                                                                  \
  X(ReadBuffer)                                                                                    \
  X(RenderbufferStorage)                                                                           \
  X(RenderbufferStorageMultisample)                                                                \
  X(Scissor)                                                                                       \
  X(ShaderSource)                                                                                  \
  X(StencilFunc)                                                                                   \
  X(StencilFuncSeparate)                                                                           \
  X(StencilMask)                                                                                   \
  X(StencilMaskSeparate)                                                                           \
  X(StencilOp)                                                                                     \
  X(StencilOpSeparate)                                                                             \
  X(TransformFeedbackVaryings)                                                                     \
  X(UniformBlockBinding)                                                                           \
  X(UnmapBuffer)                                                                                   \
  X(VertexAttribDivisor)                                                                           \
  X(VertexAttribDivisorARB)                                                                        \
  X(VertexAttribIPointer)                                                                          \
  X(VertexAttribPointer)                                                                           \
  X(Viewport)

// Texture-state calls: act on the active unit's binding (traced with the unit).
#define CVC_GL_UNIT(X)                                                                             \
  X(BindTexture)                                                                                   \
  X(CopyTexImage2D)                                                                                \
  X(CopyTexSubImage2D)                                                                             \
  X(GenerateMipmap)                                                                                \
  X(TexBuffer)                                                                                     \
  X(TexImage1D)                                                                                    \
  X(TexImage2D)                                                                                    \
  X(TexImage2DMultisample)                                                                         \
  X(TexImage3D)                                                                                    \
  X(TexParameterf)                                                                                 \
  X(TexParameterfv)                                                                                \
  X(TexParameteri)                                                                                 \
  X(TexStorage2D)                                                                                  \
  X(TexStorage2DMultisample)                                                                       \
  X(TexStorage3D)                                                                                  \
  X(TexSubImage2D)                                                                                 \
  X(TexSubImage3D)

// Draw / dispatch calls (traced with the bound program).
#define CVC_GL_DRAW(X)                                                                             \
  X(DispatchCompute)                                                                               \
  X(DrawArrays)                                                                                    \
  X(DrawArraysInstanced)                                                                           \
  X(DrawArraysInstancedARB)                                                                        \
  X(DrawElements)                                                                                  \
  X(DrawElementsInstanced)                                                                         \
  X(DrawElementsInstancedARB)                                                                      \
  X(DrawRangeElements)

// Desktop round-trips (upper bound for WebGL): the caller waits on the driver.
#define CVC_GL_SYNC(X)                                                                             \
  X(CheckFramebufferStatus)                                                                        \
  X(ClientWaitSync)                                                                                \
  X(Finish)                                                                                        \
  X(GetAttribLocation)                                                                             \
  X(GetBooleanv)                                                                                   \
  X(GetBufferPointerv)                                                                             \
  X(GetDoublev)                                                                                    \
  X(GetError)                                                                                      \
  X(GetFloatv)                                                                                     \
  X(GetFramebufferAttachmentParameteriv)                                                           \
  X(GetIntegerv)                                                                                   \
  X(GetNamedRenderbufferParameteriv)                                                               \
  X(GetProgramInfoLog)                                                                             \
  X(GetProgramiv)                                                                                  \
  X(GetQueryObjectiv)                                                                              \
  X(GetQueryObjectui64v)                                                                           \
  X(GetQueryObjectuiv)                                                                             \
  X(GetRenderbufferParameteriv)                                                                    \
  X(GetShaderInfoLog)                                                                              \
  X(GetShaderiv)                                                                                   \
  X(GetString)                                                                                     \
  X(GetStringi)                                                                                    \
  X(GetTexImage)                                                                                   \
  X(GetTexLevelParameteriv)                                                                        \
  X(GetTextureLevelParameteriv)                                                                    \
  X(GetUniformBlockIndex)                                                                          \
  X(GetUniformLocation)                                                                            \
  X(IsEnabled)                                                                                     \
  X(IsProgram)                                                                                     \
  X(MapBuffer)                                                                                     \
  X(MapBufferRange)                                                                                \
  X(ReadPixels)

// glUniform{1,2,3,4}{f,i,ui}: every argument is a scalar.
#define CVC_GL_USCALAR(X)                                                                          \
  X(Uniform1f)                                                                                     \
  X(Uniform2f)                                                                                     \
  X(Uniform3f)                                                                                     \
  X(Uniform4f)                                                                                     \
  X(Uniform1i)                                                                                     \
  X(Uniform2i)                                                                                     \
  X(Uniform3i)                                                                                     \
  X(Uniform4i)                                                                                     \
  X(Uniform1ui)                                                                                    \
  X(Uniform2ui)                                                                                    \
  X(Uniform3ui)                                                                                    \
  X(Uniform4ui)

// glUniform{1,2,3,4}{f,i,ui}v(location, count, const T *v): (name, T, components).
#define CVC_GL_UVEC(X)                                                                             \
  X(Uniform1fv, GLfloat, 1)                                                                        \
  X(Uniform2fv, GLfloat, 2)                                                                        \
  X(Uniform3fv, GLfloat, 3)                                                                        \
  X(Uniform4fv, GLfloat, 4)                                                                        \
  X(Uniform1iv, GLint, 1)                                                                          \
  X(Uniform2iv, GLint, 2)                                                                          \
  X(Uniform3iv, GLint, 3)                                                                          \
  X(Uniform4iv, GLint, 4)                                                                          \
  X(Uniform1uiv, GLuint, 1)                                                                        \
  X(Uniform2uiv, GLuint, 2)                                                                        \
  X(Uniform3uiv, GLuint, 3)                                                                        \
  X(Uniform4uiv, GLuint, 4)

// glUniformMatrix*fv(location, count, transpose, const GLfloat *v): (name, floats).
#define CVC_GL_UMAT(X)                                                                             \
  X(UniformMatrix2fv, 4)                                                                           \
  X(UniformMatrix3fv, 9)                                                                           \
  X(UniformMatrix4fv, 16)                                                                          \
  X(UniformMatrix2x3fv, 6)                                                                         \
  X(UniformMatrix3x2fv, 6)                                                                         \
  X(UniformMatrix2x4fv, 8)                                                                         \
  X(UniformMatrix4x2fv, 8)                                                                         \
  X(UniformMatrix3x4fv, 12)                                                                        \
  X(UniformMatrix4x3fv, 12)

// Tracked state: the bound program and the active texture unit.
#define CVC_GL_STATE(X)                                                                            \
  X(UseProgram)                                                                                    \
  X(LinkProgram)                                                                                   \
  X(DeleteProgram)                                                                                 \
  X(ActiveTexture)

enum Cls : unsigned char { kPlain, kUnit, kDraw, kSync, kUniform, kState };

enum Entry : int {
#define CVC_E(name, ...) E_##name,
  CVC_GL_PLAIN(CVC_E) CVC_GL_UNIT(CVC_E) CVC_GL_DRAW(CVC_E) CVC_GL_SYNC(CVC_E) CVC_GL_USCALAR(CVC_E)
      CVC_GL_UVEC(CVC_E) CVC_GL_UMAT(CVC_E) CVC_GL_STATE(CVC_E)
#undef CVC_E
          E_COUNT
};

const char *const g_names[E_COUNT] = {
#define CVC_N(name, ...) #name,
    CVC_GL_PLAIN(CVC_N) CVC_GL_UNIT(CVC_N) CVC_GL_DRAW(CVC_N) CVC_GL_SYNC(CVC_N)
        CVC_GL_USCALAR(CVC_N) CVC_GL_UVEC(CVC_N) CVC_GL_UMAT(CVC_N) CVC_GL_STATE(CVC_N)
#undef CVC_N
};

constexpr Cls g_cls[E_COUNT] = {
#define CVC_PLAIN(...) kPlain,
#define CVC_UNIT(...) kUnit,
#define CVC_DRAW(...) kDraw,
#define CVC_SYNC(...) kSync,
#define CVC_UNI(...) kUniform,
#define CVC_STATE(...) kState,
    CVC_GL_PLAIN(CVC_PLAIN) CVC_GL_UNIT(CVC_UNIT) CVC_GL_DRAW(CVC_DRAW) CVC_GL_SYNC(CVC_SYNC)
        CVC_GL_USCALAR(CVC_UNI) CVC_GL_UVEC(CVC_UNI) CVC_GL_UMAT(CVC_UNI) CVC_GL_STATE(CVC_STATE)
#undef CVC_PLAIN
#undef CVC_UNIT
#undef CVC_DRAW
#undef CVC_SYNC
#undef CVC_UNI
#undef CVC_STATE
};

double g_n[E_COUNT];
double g_uniformRedundant = 0;

// Tracked GL state, and the open trace.
GLuint g_program = 0;
unsigned g_unit = 0;
bool g_tracing = false;
Trace g_trace;

// Uniform values are per program: (program, location) -> last bytes sent.
std::map<std::pair<GLuint, GLint>, std::string> g_last;

// argBytes starts with the GLint location; the rest is the value as sent.
void noteUniform(const std::string &argBytes) {
  GLint loc = 0;
  std::memcpy(&loc, argBytes.data(), sizeof loc);
  std::string v = argBytes.substr(sizeof loc);
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

void record(int id, std::string &&args) {
  TraceRec r;
  r.entry = id;
  switch (g_cls[id]) {
  case kUnit:
    r.ctx = g_unit;
    break;
  case kDraw:
  case kUniform:
    r.ctx = g_program;
    break;
  default:
    r.ctx = 0;
  }
  r.args = std::move(args);
  g_trace.push_back(std::move(r));
}

// Floats are recorded as GL computes with them: -0.0 as +0.0. (VTK's camera
// and actor key-matrix caches hand out a zero normal-matrix entry with either
// sign depending on which earlier render last recomputed them -- history, not
// the draw path: two Stock hardware selections in a row differ that way.)
template <typename F> F canonicalZero(F v) { return v == F(0) ? F(0) : v; }

template <typename T> void appendArg(std::string &s, T v) {
  if constexpr (std::is_floating_point_v<T>) {
    const T c = canonicalZero(v);
    char b[sizeof(T)];
    std::memcpy(b, &c, sizeof(T));
    s.append(b, sizeof(T));
  } else if constexpr (std::is_same_v<T, const GLchar *>) {
    // Location / attribute names: the string, so the lookup is identified.
    if (v)
      s.append(v, std::strlen(v) + 1);
    else
      s.push_back('\0');
  } else if constexpr (std::is_pointer_v<T>) {
    s.push_back(v ? '\1' : '\0'); // bulk data / out-params: only whether it is there
  } else {
    char b[sizeof(T)];
    std::memcpy(b, &v, sizeof(T));
    s.append(b, sizeof(T));
  }
}

// The generic wrapper: count, note uniform values, trace.
template <int Id, typename T> struct Wrap;
template <int Id, typename R, typename... A> struct Wrap<Id, R(GLAD_API_PTR *)(A...)> {
  static constexpr bool kIsUniform = g_cls[Id] == kUniform;
  static inline R(GLAD_API_PTR *orig)(A...) = nullptr;
  static R GLAD_API_PTR fn(A... a) {
    g_n[Id] += 1;
    if (kIsUniform || g_tracing) {
      std::string bytes;
      (appendArg(bytes, a), ...);
      if constexpr (kIsUniform)
        noteUniform(bytes);
      if (g_tracing)
        record(Id, std::move(bytes));
    }
    return orig(a...);
  }
  static void hook(R(GLAD_API_PTR *&ptr)(A...)) {
    if (ptr && ptr != &fn) {
      orig = ptr;
      ptr = &fn;
    }
  }
  static void unhook(R(GLAD_API_PTR *&ptr)(A...)) {
    if (ptr == &fn)
      ptr = orig;
  }
};

// glUniform*v / glUniformMatrix*: the values behind the pointer.
#define CVC_O(name, ...) decltype(glad_gl##name) o_##name = nullptr;
CVC_GL_UVEC(CVC_O)
CVC_GL_UMAT(CVC_O)
CVC_GL_STATE(CVC_O)
#undef CVC_O

void uniformArray(int id, GLint l, GLsizei c, const std::string &prefix, const void *v,
                  size_t bytesPerElement, bool isFloat) {
  g_n[id] += 1;
  std::string b;
  appendArg(b, l);
  appendArg(b, c);
  b += prefix;
  if (v && c > 0) {
    const size_t n = bytesPerElement * static_cast<size_t>(c);
    if (isFloat) {
      for (size_t k = 0; k < n; k += sizeof(GLfloat)) {
        GLfloat f;
        std::memcpy(&f, static_cast<const char *>(v) + k, sizeof f);
        appendArg(b, f);
      }
    } else {
      b.append(static_cast<const char *>(v), n);
    }
  }
  noteUniform(b);
  if (g_tracing)
    record(id, std::move(b));
}

#define CVC_UV_W(name, T, k)                                                                       \
  void GLAD_API_PTR w_##name(GLint l, GLsizei c, const T *v) {                                     \
    uniformArray(E_##name, l, c, std::string(), v, sizeof(T) * (k), std::is_same_v<T, GLfloat>);   \
    o_##name(l, c, v);                                                                             \
  }
CVC_GL_UVEC(CVC_UV_W)
#undef CVC_UV_W

#define CVC_UM_W(name, k)                                                                          \
  void GLAD_API_PTR w_##name(GLint l, GLsizei c, GLboolean t, const GLfloat *v) {                  \
    uniformArray(E_##name, l, c, std::string(1, static_cast<char>(t)), v, sizeof(GLfloat) * (k),   \
                 true);                                                                            \
    o_##name(l, c, t, v);                                                                          \
  }
CVC_GL_UMAT(CVC_UM_W)
#undef CVC_UM_W

void GLAD_API_PTR w_UseProgram(GLuint p) {
  g_n[E_UseProgram] += 1;
  g_program = p;
  if (g_tracing) {
    std::string b;
    appendArg(b, p);
    record(E_UseProgram, std::move(b));
  }
  o_UseProgram(p);
}
void GLAD_API_PTR w_LinkProgram(GLuint p) {
  g_n[E_LinkProgram] += 1;
  forgetProgram(p);
  if (g_tracing) {
    std::string b;
    appendArg(b, p);
    record(E_LinkProgram, std::move(b));
  }
  o_LinkProgram(p);
}
void GLAD_API_PTR w_DeleteProgram(GLuint p) {
  g_n[E_DeleteProgram] += 1;
  forgetProgram(p);
  if (g_tracing) {
    std::string b;
    appendArg(b, p);
    record(E_DeleteProgram, std::move(b));
  }
  o_DeleteProgram(p);
}
void GLAD_API_PTR w_ActiveTexture(GLenum t) {
  g_n[E_ActiveTexture] += 1;
  g_unit = t - GL_TEXTURE0;
  if (g_tracing) {
    std::string b;
    appendArg(b, t);
    record(E_ActiveTexture, std::move(b));
  }
  o_ActiveTexture(t);
}

// The real glGetIntegerv / glGetFloatv, whether or not they are wrapped now.
void rawGetIntegerv(GLenum what, GLint *v) {
  using W = Wrap<E_GetIntegerv, decltype(glad_glGetIntegerv)>;
  auto fn = glad_glGetIntegerv == &W::fn ? W::orig : glad_glGetIntegerv;
  if (fn)
    fn(what, v);
}
void rawGetFloatv(GLenum what, GLfloat *v) {
  using W = Wrap<E_GetFloatv, decltype(glad_glGetFloatv)>;
  auto fn = glad_glGetFloatv == &W::fn ? W::orig : glad_glGetFloatv;
  if (fn)
    fn(what, v);
}

} // namespace

bool TraceRec::marker() const { return entry >= kTraceMarkBase; }

void glcallsInstall() {
  // Start from the context's real state (calls made while uninstalled, or in
  // another context, were not tracked).
  GLint program = 0, unit = GL_TEXTURE0;
  rawGetIntegerv(GL_CURRENT_PROGRAM, &program);
  rawGetIntegerv(GL_ACTIVE_TEXTURE, &unit);
  g_program = static_cast<GLuint>(program);
  g_unit = static_cast<unsigned>(unit - GL_TEXTURE0);
#define CVC_H(name, ...) Wrap<E_##name, decltype(glad_gl##name)>::hook(glad_gl##name);
  CVC_GL_PLAIN(CVC_H)
  CVC_GL_UNIT(CVC_H)
  CVC_GL_DRAW(CVC_H)
  CVC_GL_SYNC(CVC_H)
  CVC_GL_USCALAR(CVC_H)
#undef CVC_H
#define CVC_W(name, ...)                                                                           \
  if (glad_gl##name && glad_gl##name != w_##name) {                                                \
    o_##name = glad_gl##name;                                                                      \
    glad_gl##name = w_##name;                                                                      \
  }
  CVC_GL_UVEC(CVC_W)
  CVC_GL_UMAT(CVC_W)
  CVC_GL_STATE(CVC_W)
#undef CVC_W
}

void glcallsUninstall() {
#define CVC_U(name, ...) Wrap<E_##name, decltype(glad_gl##name)>::unhook(glad_gl##name);
  CVC_GL_PLAIN(CVC_U)
  CVC_GL_UNIT(CVC_U)
  CVC_GL_DRAW(CVC_U)
  CVC_GL_SYNC(CVC_U)
  CVC_GL_USCALAR(CVC_U)
#undef CVC_U
#define CVC_R(name, ...)                                                                           \
  if (glad_gl##name == w_##name)                                                                   \
    glad_gl##name = o_##name;
  CVC_GL_UVEC(CVC_R)
  CVC_GL_UMAT(CVC_R)
  CVC_GL_STATE(CVC_R)
#undef CVC_R
}

GLCalls glcallsRead() {
  GLCalls r;
  r.n.assign(g_n, g_n + E_COUNT);
  r.uniformRedundant = g_uniformRedundant;
  return r;
}

int glcallsEntries() { return E_COUNT; }
const char *glcallsName(int i) { return i >= 0 && i < E_COUNT ? g_names[i] : "?"; }

void glcallsTraceStart() {
  g_trace.clear();
  g_tracing = true;
  for (auto [kind, what] : {std::make_pair(kTraceInitPointSize, GLenum(GL_POINT_SIZE)),
                            std::make_pair(kTraceInitLineWidth, GLenum(GL_LINE_WIDTH))}) {
    GLfloat v = 0;
    rawGetFloatv(what, &v);
    TraceRec r;
    r.entry = kTraceMarkBase + kind;
    appendArg(r.args, v);
    g_trace.push_back(std::move(r));
  }
}

Trace glcallsTraceStop() {
  g_tracing = false;
  Trace t;
  t.swap(g_trace);
  return t;
}

void glcallsMark(int kind, int arg) {
  if (!g_tracing)
    return;
  TraceRec r;
  r.entry = kTraceMarkBase + kind;
  r.ctx = static_cast<unsigned>(arg);
  g_trace.push_back(std::move(r));
}

bool glcallsIsDraw(int entry) { return entry >= 0 && entry < E_COUNT && g_cls[entry] == kDraw; }

int glcallsEntry(const std::string &name) {
  for (int i = 0; i < E_COUNT; ++i)
    if (name == g_names[i])
      return i;
  return -1;
}

std::string glcallsDescribe(const TraceRec &r) {
  char buf[96];
  if (r.marker()) {
    std::snprintf(buf, sizeof buf, "mark %d (%u)", r.entry - kTraceMarkBase, r.ctx);
    return buf;
  }
  const Cls c = r.entry >= 0 && r.entry < E_COUNT ? g_cls[r.entry] : kPlain;
  std::snprintf(buf, sizeof buf, "gl%s%s%u [", glcallsName(r.entry),
                c == kUnit                      ? " unit="
                : (c == kDraw || c == kUniform) ? " prog="
                                                : " ",
                r.ctx);
  std::string out = buf;
  for (size_t i = 0; i < r.args.size() && i < 48; ++i) {
    std::snprintf(buf, sizeof buf, i ? " %02x" : "%02x", static_cast<unsigned char>(r.args[i]));
    out += buf;
  }
  out += r.args.size() > 48 ? " ...]" : "]";
  return out;
}

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
  for (int i = 0; i < static_cast<int>(n.size()) && i < E_COUNT; ++i)
    s += g_cls[i] == kUniform ? n[i] : 0.0;
  return s;
}

double GLCalls::roundTrips() const {
  double s = 0;
  for (int i = 0; i < static_cast<int>(n.size()) && i < E_COUNT; ++i)
    s += g_cls[i] == kSync ? n[i] : 0.0;
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
  char buf[192];
  std::snprintf(buf, sizeof buf,
                "total %.1f (uniforms %.1f, redundant %.1f, desktop round-trips (upper bound) "
                "%.1f) |",
                total() / per, uniforms() / per, uniformRedundant / per, roundTrips() / per);
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
