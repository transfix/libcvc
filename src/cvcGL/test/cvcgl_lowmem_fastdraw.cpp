/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::gl::LowMemoryPolyDataMapper -- the WebGL2/GLES mapper with the empty
// cell types and the per-draw shader-cache lookup taken out of its draw.
//
// Runs that mapper natively (policy Force) on cvcGL's real pipeline -- a
// SceneGraph with shadows on (camera -> strided shadow baker -> shadow map ->
// translucent -> volumetric -> overlay) and off -- over GeometryNodes of every
// kind a scene uses (lit single colour, textured ground, translucent with a
// depth offset, wireframe lines, points, per-vertex colour, quads) plus raw VTK
// actors whose mapper counts the GL calls of its own draws: surface, wireframe
// and points representations of a triangle mesh, edge visibility (with and
// without its own offset), cell colours on quads, point colours, texture
// coordinates, translucency, verts, strips, and lines flat / lit / wide / as
// points. Checks:
//   1. the policy: CVCGL_LOWMEM_MAPPER is read, Force/Off/Auto pick the mapper;
//   2. the replica: the ordered GL trace of a DrawPath::AllCellTypes frame --
//      every call, its bound program or texture unit, its argument bytes and
//      the uniform values it sends -- equals the Stock frame's exactly (bake
//      and plain frames, shadows on and off, and the hardware selector's cell
//      and point picking passes);
//   3. Fast's trace equals that same trace with what Fast leaves out removed,
//      DERIVED from the AllCellTypes trace alone (the replica marks where each
//      draw's stages begin and end): a draw's cell-type blocks that issued no
//      draw call go iff replaying the trace's glUniform* calls without them
//      leaves every draw call's uniform state unchanged -- never from the
//      mapper's own skip decision, whose markers are only counted. And Fast's
//      own trace, replayed the same way, shows every draw call the uniform
//      state (every location of its program) it sees on AllCellTypes, and
//      leaves each program what the next frame's draws read before writing. A
//      skipped shader-cache lookup removes no GL call -- the lookup's GL calls
//      are the bind tail the re-bind issues too; what it saves is CPU, which
//      the stats check;
//   4. pixels: AllCellTypes and Fast frames (RGBA) match Stock's -- as do a
//      Stock frame right after Fast, and frames across a recompile and across a
//      new window -- byte for byte, and so do pairs of Stock frames of one GL
//      trace. Only on Apple's renderer (GL version or renderer string naming
//      Apple), which rounds a few bytes 1 LSB apart now and then, in one window
//      and across windows, is every pixel check -- the pairs of Stock frames
//      too -- held to a fixed envelope instead: identical, or no byte off by
//      more than 1 and at most 64 bytes; see checkPixels();
//   5. the saving: Fast draws a single-cell-type mesh in <= 2 VAO binds and a
//      fraction of the stock calls, with no desktop round-trip in any draw
//      beyond Stock's (none, but VTK's own warning on a line width the driver
//      cannot draw), and a steady frame does no shader-cache lookup;
//   6. lifecycle: a shader rebuild (the shadow bake's pass change), a mapper's
//      released resources, GetShader(), programs released in the shader cache
//      and the scene re-opened in a new window all go back through the full
//      lookup (or a recompile) and still match;
//   7. guards: an offset layout where skipping would change a drawn type's
//      depth offset, picking and vertex visibility all draw the stock way (and
//      pick the same; on Apple's renderer point picking may differ by 1 point
//      per prop, see comparePicking());
//   8. a plain GeometryNode's shader replacement added (and cleared) after its
//      first draw reaches the GPU, on every draw path.
// Frames are rendered without MSAA: with VTK's default 8x, NVIDIA's resolve
// varies by 1 LSB at a few edge pixels between identical GL streams.
// Renders for real; skips (rc 0) where nothing rasterises unless
// CVC_REQUIRE_RENDER=1. Set CVCGL_LOWMEM_REPORT=1 for the per-draw breakdown.
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include "gl_call_counter.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LowMemoryPolyDataMapper.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/image/image.h>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataObject.h>
#include <vtkFloatArray.h>
#include <vtkHardwareSelector.h>
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkMatrix4x4.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOutputWindow.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkShader.h>
#include <vtkShaderProgram.h>
#include <vtkSphereSource.h>
#include <vtkTexture.h>
#include <vtkUnsignedCharArray.h>
#include <vtk_glad.h> // GL_POINTS ... (desktop-only test)

using cvc::gl::GeometryNode;
using cvc::gl::GeometryRenderMode;
using cvc::gl::LowMemoryMapperPolicy;
using cvc::gl::LowMemoryPolyDataMapper;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvcgl_test::GLCalls;
using cvcgl_test::glcallsInstall;
using cvcgl_test::glcallsRead;
using cvcgl_test::kTraceMarkBase;
using cvcgl_test::Trace;
using cvcgl_test::TraceRec;
using DrawPath = LowMemoryPolyDataMapper::DrawPath;
using DrawStage = LowMemoryPolyDataMapper::DrawStage;

namespace {

int g_failures = 0, g_checks = 0;
bool g_report = false;

bool check(bool ok, const std::string &what, const std::string &detail = "") {
  ++g_checks;
  if (!ok)
    ++g_failures;
  std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(), detail.empty() ? "" : " -- ",
              detail.c_str());
  std::fflush(stdout);
  return ok;
}

std::string fmt(const char *f, double a, double b = 0, double c = 0) {
  char buf[256];
  std::snprintf(buf, sizeof buf, f, a, b, c);
  return buf;
}

const char *pathName(DrawPath p) {
  return p == DrawPath::Stock ? "Stock" : p == DrawPath::AllCellTypes ? "AllCellTypes" : "Fast";
}

// Counts VTK errors (shader compile/link failures arrive here, not as exceptions).
class ErrorCounter : public vtkOutputWindow {
public:
  static ErrorCounter *New();
  vtkTypeMacro(ErrorCounter, vtkOutputWindow);
  static int &errors() {
    static int n = 0;
    return n;
  }
  void DisplayErrorText(const char *t) override {
    ++errors();
    std::fprintf(stderr, "VTK ERROR: %s\n", t);
  }
  void DisplayWarningText(const char *t) override { std::fprintf(stderr, "VTK WARNING: %s\n", t); }
  void DisplayGenericWarningText(const char *t) override {
    std::fprintf(stderr, "VTK WARNING: %s\n", t);
  }
  void DisplayText(const char *t) override { std::fprintf(stderr, "%s", t); }
};
vtkStandardNewMacro(ErrorCounter);

// ── GL traces ────────────────────────────────────────────────────────────────
// The replica's stage observer: each stage boundary becomes a trace marker.
void onStage(DrawStage stage, int arg) { cvcgl_test::glcallsMark(static_cast<int>(stage), arg); }

bool isStage(const TraceRec &r, DrawStage stage) {
  return r.marker() && r.entry - kTraceMarkBase == static_cast<int>(stage);
}

int countStage(const Trace &t, DrawStage stage) {
  int n = 0;
  for (const auto &r : t)
    n += isStage(r, stage);
  return n;
}

// The GL calls alone.
Trace glOnly(const Trace &t) {
  Trace out;
  out.reserve(t.size());
  for (const auto &r : t)
    if (!r.marker())
      out.push_back(r);
  return out;
}

// Equal GL traces (every call, in order, with its context and bytes)? If not,
// where they first part.
bool sameTrace(const Trace &a, const Trace &b, std::string *why) {
  const size_t n = std::min(a.size(), b.size());
  size_t i = 0;
  while (i < n && a[i] == b[i])
    ++i;
  if (i == n && a.size() == b.size())
    return true;
  if (why) {
    *why = fmt("%.0f vs %.0f calls, first difference at #%.0f: ", double(a.size()),
               double(b.size()), double(i));
    *why += i < a.size() ? cvcgl_test::glcallsDescribe(a[i]) : std::string("(end)");
    *why += "  vs  ";
    *why += i < b.size() ? cvcgl_test::glcallsDescribe(b[i]) : std::string("(end)");
  }
  return false;
}

// vtkOpenGLState caches glPointSize and glLineWidth (desktop GL only: GLES 3 /
// WebGL have neither call). Whether a kept block's vtkglPointSize reaches GL
// therefore depends on what the blocks before it set, so leaving out an empty
// block can add or drop one of these calls without changing what any draw
// sees. Compare them as that: drop the calls, and append to every GL_POINTS
// draw the point size it draws with and to every line draw the line width
// (tracked from the trace's initial-state markers and its calls).
Trace asDrawState(const Trace &t) {
  static const int pointSize = cvcgl_test::glcallsEntry("PointSize");
  static const int lineWidth = cvcgl_test::glcallsEntry("LineWidth");
  static const int dispatch = cvcgl_test::glcallsEntry("DispatchCompute");
  std::string ps(4, '\0'), lw(4, '\0');
  Trace out;
  out.reserve(t.size());
  for (const auto &r : t) {
    if (r.marker()) {
      if (r.entry == kTraceMarkBase + cvcgl_test::kTraceInitPointSize)
        ps = r.args;
      else if (r.entry == kTraceMarkBase + cvcgl_test::kTraceInitLineWidth)
        lw = r.args;
      else
        out.push_back(r);
      continue;
    }
    if (r.entry == pointSize) {
      ps = r.args;
      continue;
    }
    if (r.entry == lineWidth) {
      lw = r.args;
      continue;
    }
    out.push_back(r);
    if (cvcgl_test::glcallsIsDraw(r.entry) && r.entry != dispatch && r.args.size() >= 4) {
      GLenum mode = 0;
      std::memcpy(&mode, r.args.data(), sizeof mode);
      if (mode == GL_POINTS)
        out.back().args += ps;
      else if (mode == GL_LINES || mode == GL_LINE_STRIP || mode == GL_LINE_LOOP)
        out.back().args += lw;
    }
  }
  return out;
}

// ── the uniforms each draw call sees ─────────────────────────────────────────
// A trace's glUniform* calls replayed into (program, location) -> value: at
// every draw call, the bound program's locations the trace has written so far
// (a location it has not written holds what the program had before the trace).
struct UniformView {
  struct DrawCall {
    size_t at = 0; // index in the trace
    unsigned program = 0;
    std::map<int, std::string> written;
  };
  std::vector<DrawCall> calls;
  std::map<unsigned, std::map<int, std::string>> end; // each program, at the trace's end
};

// resetDraw >= 0: forget what the program of the trace's resetDraw-th replica
// draw (DrawBegin) holds when that draw first touches it -- that draw then sees
// only what it writes itself, whatever an earlier draw left.
UniformView replayUniforms(const Trace &t, int resetDraw = -1) {
  static const int link = cvcgl_test::glcallsEntry("LinkProgram");
  static const int del = cvcgl_test::glcallsEntry("DeleteProgram");
  UniformView v;
  auto &state = v.end;
  int draw = -1;
  bool resetPending = false;
  for (size_t i = 0; i < t.size(); ++i) {
    const TraceRec &r = t[i];
    if (r.marker()) {
      if (isStage(r, DrawStage::DrawBegin) && ++draw == resetDraw)
        resetPending = true;
      continue;
    }
    const bool uniform = cvcgl_test::glcallsIsUniform(r.entry);
    const bool drawCall = cvcgl_test::glcallsIsDraw(r.entry);
    if (resetPending && (uniform || drawCall)) {
      state.erase(r.ctx);
      resetPending = false;
    }
    if ((r.entry == link || r.entry == del) && r.args.size() >= sizeof(unsigned)) {
      unsigned program = 0; // a (re)link resets every uniform; a deleted name is reused
      std::memcpy(&program, r.args.data(), sizeof program);
      state.erase(program);
    } else if (uniform) {
      for (auto &w : cvcgl_test::glcallsUniformWrites(r))
        state[r.ctx][w.location] = std::move(w.value);
    } else if (drawCall) {
      v.calls.push_back({i, r.ctx, state[r.ctx]});
    }
  }
  return v;
}

// A UniformWrite value ("3f:<bytes>") as its type and numbers.
std::string uniformValue(const std::string &v) {
  const size_t colon = v.find(':');
  if (colon == std::string::npos)
    return v;
  const std::string type = v.substr(0, colon);
  const bool f = type.find('f') != std::string::npos; // "3f", "Matrix4f", "Matrix4fT"
  const bool u = type.find("ui") != std::string::npos;
  std::string out = type + " (";
  char buf[32];
  for (size_t at = colon + 1, k = 0; at + 4 <= v.size() && k < 16; at += 4, ++k) {
    if (f) {
      float x;
      std::memcpy(&x, v.data() + at, sizeof x);
      std::snprintf(buf, sizeof buf, k ? " %g" : "%g", double(x));
    } else if (u) {
      unsigned x;
      std::memcpy(&x, v.data() + at, sizeof x);
      std::snprintf(buf, sizeof buf, k ? " %u" : "%u", x);
    } else {
      int x;
      std::memcpy(&x, v.data() + at, sizeof x);
      std::snprintf(buf, sizeof buf, k ? " %d" : "%d", x);
    }
    out += buf;
  }
  return out + ")";
}

// Do two traces' draw calls see the same uniforms? The traces issue the same
// draw calls in the same order (the trace checks pin that), and at each one the
// bound program must hold the same value at every location either trace wrote:
// written to the same value in both, or written in neither so far (both see
// the value from before the trace). Where a draw call sees such an inherited
// value it reads what the previous frame left -- for a repeated frame, what this
// frame leaves -- so for those (program, location)s the two frames' end values
// must agree too. End values no draw call reads before writing are not
// compared: on AllCellTypes a lines-only draw leaves the empty polys/strips
// blocks' cellType and primitiveSize in its program, Fast leaves the lines
// values, and every cell-type block writes both before it draws.
bool sameUniformsSeen(const UniformView &a, const UniformView &b, const Trace &ta,
                      std::string *why) {
  const auto describe = [](const std::map<int, std::string> &m, int loc) {
    const auto it = m.find(loc);
    return it == m.end() ? std::string("(not written yet)") : uniformValue(it->second);
  };
  if (a.calls.size() != b.calls.size()) {
    if (why)
      *why = fmt("%.0f vs %.0f draw calls", double(a.calls.size()), double(b.calls.size()));
    return false;
  }
  static const std::map<int, std::string> kNone;
  const auto endOf = [](const UniformView &v,
                        unsigned program) -> const std::map<int, std::string> & {
    const auto it = v.end.find(program);
    return it == v.end.end() ? kNone : it->second;
  };
  std::set<std::pair<unsigned, int>> inherited;
  for (size_t k = 0; k < a.calls.size(); ++k) {
    const auto &x = a.calls[k], &y = b.calls[k];
    if (x.program != y.program) {
      if (why)
        *why = fmt("draw call %.0f: program %.0f vs %.0f", double(k), double(x.program),
                   double(y.program));
      return false;
    }
    std::set<int> locations;
    for (const auto *m : {&x.written, &y.written, &endOf(a, x.program), &endOf(b, x.program)})
      for (const auto &kv : *m)
        locations.insert(kv.first);
    for (int loc : locations) {
      const auto i = x.written.find(loc), j = y.written.find(loc);
      if (i == x.written.end() && j == y.written.end()) {
        inherited.insert({x.program, loc});
        continue;
      }
      if (i != x.written.end() && j != y.written.end() && i->second == j->second)
        continue;
      if (why)
        *why = fmt("draw call %.0f (trace #%.0f, ", double(k), double(x.at)) +
               cvcgl_test::glcallsDescribe(ta[x.at]) + fmt("): location %.0f: ", double(loc)) +
               describe(x.written, loc) + " vs " + describe(y.written, loc);
      return false;
    }
  }
  for (const auto &[program, loc] : inherited) {
    const auto &ea = endOf(a, program), &eb = endOf(b, program);
    const auto i = ea.find(loc), j = eb.find(loc);
    const bool same = (i == ea.end() && j == eb.end()) ||
                      (i != ea.end() && j != eb.end() && i->second == j->second);
    if (!same) {
      if (why)
        *why = fmt("program %.0f location %.0f, read before the frame writes it: the frame "
                   "leaves ",
                   double(program), double(loc)) +
               describe(ea, loc) + " vs " + describe(eb, loc);
      return false;
    }
  }
  return true;
}

// For each replica draw (DrawBegin) of a trace: its cell-type blocks that issued
// no draw call.
std::vector<std::vector<int>> emptyBlocks(const Trace &t) {
  std::vector<std::vector<int>> out;
  bool inBlock = false, drew = false;
  int type = 0;
  for (const auto &r : t) {
    if (!r.marker()) {
      drew = drew || (inBlock && cvcgl_test::glcallsIsDraw(r.entry));
      continue;
    }
    if (isStage(r, DrawStage::DrawBegin)) {
      out.emplace_back();
    } else if (isStage(r, DrawStage::AgentBegin)) {
      inBlock = true;
      drew = false;
      type = static_cast<int>(r.ctx);
    } else if (isStage(r, DrawStage::AgentEnd)) {
      inBlock = false;
      if (!drew && !out.empty())
        out.back().push_back(type);
    }
  }
  return out;
}

// `t` without the cell-type blocks (AgentBegin..AgentEnd, markers included)
// that drop(draw, type) selects; draw counts the trace's replica draws.
template <typename Drop> Trace without(const Trace &t, Drop drop) {
  Trace out;
  out.reserve(t.size());
  int draw = -1;
  bool dropping = false;
  for (const auto &r : t) {
    if (isStage(r, DrawStage::DrawBegin))
      ++draw;
    if (isStage(r, DrawStage::AgentBegin) && drop(draw, static_cast<int>(r.ctx)))
      dropping = true;
    if (!dropping)
      out.push_back(r);
    if (isStage(r, DrawStage::AgentEnd))
      dropping = false;
  }
  return out;
}

// What Fast's GL trace must be, derived from an AllCellTypes trace alone --
// never from the mapper's own skip decision (coincidentSkipSafe). A replica
// draw's empty cell-type blocks may go iff removing them (that draw's alone)
// changes no uniform any draw call sees, with the draw's program state on
// entry treated as unknown (what earlier draws leave in a shared program is not
// the draw's to rely on): the blocks' writes either reach no draw call or are
// written again, to the same value, before one reads them. Everything else
// stays -- including each full lookup's GL calls, which are the
// ReadyShaderProgram(program) tail (compile if released, bind) that the re-bind
// replacing it issues too.
struct Expected {
  Trace trace; // with the markers of what stays
  int blocksRemoved = 0, blocksKept = 0, lookups = 0, draws = 0, fallbacks = 0;
  std::string firstFallback; // why the first draw that keeps its empty blocks must
};

Expected expectFast(const Trace &all) {
  Expected e;
  const auto empty = emptyBlocks(all);
  const auto isEmpty = [&](int d, int type) {
    const auto &v = empty[static_cast<size_t>(d)];
    return std::find(v.begin(), v.end(), type) != v.end();
  };
  std::vector<bool> removable(empty.size(), false);
  for (size_t d = 0; d < empty.size(); ++d) {
    if (empty[d].empty())
      continue;
    const int draw = static_cast<int>(d);
    const Trace removed =
        without(all, [&](int dd, int type) { return dd == draw && isEmpty(dd, type); });
    std::string why;
    removable[d] =
        sameUniformsSeen(replayUniforms(all, draw), replayUniforms(removed, draw), all, &why);
    if (!removable[d]) {
      ++e.fallbacks;
      if (e.firstFallback.empty())
        e.firstFallback = fmt("draw %.0f: ", double(d)) + why;
    }
  }
  e.trace = without(all, [&](int d, int type) {
    return d >= 0 && removable[static_cast<size_t>(d)] && isEmpty(d, type);
  });
  e.draws = static_cast<int>(empty.size());
  for (size_t d = 0; d < empty.size(); ++d)
    (removable[d] ? e.blocksRemoved : e.blocksKept) += static_cast<int>(empty[d].size());
  e.lookups = countStage(all, DrawStage::LookupBegin);
  return e;
}

// The production mapper plus a GL-call tally of its own RenderPieceDraw calls.
class ProbeMapper : public LowMemoryPolyDataMapper {
public:
  static ProbeMapper *New();
  vtkTypeMacro(ProbeMapper, LowMemoryPolyDataMapper);
  GLCalls calls;
  double draws = 0;
  void clearTally() {
    calls = GLCalls();
    draws = 0;
  }
  void RenderPieceDraw(vtkRenderer *ren, vtkActor *act) override {
    const GLCalls before = glcallsRead();
    Superclass::RenderPieceDraw(ren, act);
    calls += glcallsRead() - before;
    draws += 1;
  }

protected:
  ProbeMapper() = default;
};
vtkStandardNewMacro(ProbeMapper);

// A GeometryNode whose actor the test can reach.
class PeekNode : public GeometryNode {
public:
  using GeometryNode::GeometryNode;
  vtkActor *actor() { return vtkActor::SafeDownCast(getProp()); }
};

// ── geometry ─────────────────────────────────────────────────────────────────
// A closed, lit box (separate faces so the normals are flat).
cvc::geometry boxGeometry(double cx, double cy, double cz, double sx, double sy, double sz) {
  cvc::geometry g;
  const double x0 = cx - sx, x1 = cx + sx, y0 = cy - sy, y1 = cy + sy, z0 = cz - sz, z1 = cz + sz;
  const double faces[6][4][3] = {{{x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}}, // top
                                 {{x0, y0, z0}, {x0, y1, z0}, {x1, y1, z0}, {x1, y0, z0}}, // bottom
                                 {{x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}}, // -y
                                 {{x1, y1, z0}, {x0, y1, z0}, {x0, y1, z1}, {x1, y1, z1}}, // +y
                                 {{x0, y1, z0}, {x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}}, // -x
                                 {{x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {x1, y0, z1}}}; // +x
  for (const auto &f : faces) {
    const unsigned base = static_cast<unsigned>(g.points().size());
    for (const auto &p : f)
      g.points().push_back({p[0], p[1], p[2]});
    g.tris().push_back({base, base + 1, base + 2});
    g.tris().push_back({base, base + 2, base + 3});
  }
  return g;
}

// An n x n height-field grid over [-s, s]^2 with UVs.
cvc::geometry groundGeometry(int n, double s) {
  cvc::geometry g;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const double x = -s + 2 * s * i / (n - 1), y = -s + 2 * s * j / (n - 1);
      g.points().push_back({x, y, 0.6 * std::sin(x * 0.35) * std::cos(y * 0.3)});
      g.uvs().push_back({double(i) / (n - 1), double(j) / (n - 1)});
    }
  for (int j = 0; j + 1 < n; ++j)
    for (int i = 0; i + 1 < n; ++i) {
      const unsigned a = j * n + i, b = a + 1, c = a + n, d = c + 1;
      g.tris().push_back({a, b, d});
      g.tris().push_back({a, d, c});
    }
  return g;
}

cvc::geometry ribbonGeometry(int n, double x0, double y0, double z) {
  cvc::geometry g;
  for (int k = 0; k < n; ++k) {
    g.points().push_back({x0 + 0.8 * k, y0 - 0.6, z});
    g.points().push_back({x0 + 0.8 * k, y0 + 0.6, z});
  }
  for (int k = 0; k + 1 < n; ++k) {
    const unsigned l0 = 2 * k, r0 = l0 + 1, l1 = l0 + 2, r1 = l0 + 3;
    g.tris().push_back({l0, r0, r1});
    g.tris().push_back({l0, r1, l1});
  }
  return g;
}

cvc::geometry polylineGeometry(int n, double x0, double y0, double z) {
  cvc::geometry g;
  for (int k = 0; k < n; ++k) {
    g.points().push_back({x0 + 0.5 * k, y0 + std::sin(k * 0.4), z});
    if (k)
      g.lines().push_back({static_cast<unsigned>(k - 1), static_cast<unsigned>(k)});
  }
  return g;
}

cvc::geometry cloudGeometry(int n, double x0, double y0, double z) {
  cvc::geometry g;
  for (int k = 0; k < n; ++k)
    g.points().push_back({x0 + 0.4 * (k % 10), y0 + 0.4 * (k / 10), z + 0.1 * (k % 3)});
  return g;
}

cvc::geometry colouredGeometry(double cx, double cy) {
  cvc::geometry g = boxGeometry(cx, cy, 1.0, 1.0, 1.0, 1.0);
  for (size_t i = 0; i < g.points().size(); ++i)
    g.colors().push_back({(i % 3) / 2.0, ((i / 3) % 3) / 2.0, ((i / 9) % 3) / 2.0});
  return g;
}

cvc::image checkerImage(int n) {
  cvc::image img(n, n, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  unsigned char *p = img.data();
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      unsigned char *q = p + 4 * (j * n + i);
      const bool on = ((i / 8) + (j / 8)) % 2 == 0;
      q[0] = on ? 200 : 60;
      q[1] = static_cast<unsigned char>(i * 255 / n);
      q[2] = static_cast<unsigned char>(j * 255 / n);
      q[3] = 255;
    }
  return img;
}

// Raw VTK poly data for the probe actors.
vtkSmartPointer<vtkPolyData> withNormals(vtkPolyData *pd) {
  vtkNew<vtkPolyDataNormals> f;
  f->SetInputData(pd);
  f->SplittingOff();
  f->ComputePointNormalsOn();
  f->ComputeCellNormalsOff();
  f->Update();
  auto out = vtkSmartPointer<vtkPolyData>::New();
  out->ShallowCopy(pd);
  out->GetPointData()->SetNormals(f->GetOutput()->GetPointData()->GetNormals());
  return out;
}

vtkSmartPointer<vtkPolyData> spherePoly(double r) {
  vtkNew<vtkSphereSource> s;
  s->SetRadius(r);
  s->SetThetaResolution(24);
  s->SetPhiResolution(18);
  s->Update();
  auto pd = vtkSmartPointer<vtkPolyData>::New();
  pd->DeepCopy(s->GetOutput());
  return pd;
}

vtkSmartPointer<vtkPolyData> terrainPoly(int n, double x0, double y0, double s) {
  auto pts = vtkSmartPointer<vtkPoints>::New();
  pts->SetDataTypeToFloat();
  auto tc = vtkSmartPointer<vtkFloatArray>::New();
  tc->SetNumberOfComponents(2);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      pts->InsertNextPoint(x0 + s * i / (n - 1), y0 + s * j / (n - 1), 0.05 * ((i + j) % 3));
      tc->InsertNextTuple2(double(i) / (n - 1), double(j) / (n - 1));
    }
  auto polys = vtkSmartPointer<vtkCellArray>::New();
  for (int j = 0; j + 1 < n; ++j)
    for (int i = 0; i + 1 < n; ++i) {
      const vtkIdType a = j * n + i, b = a + 1, c = a + n, d = c + 1;
      const vtkIdType t1[3] = {a, b, d}, t2[3] = {a, d, c};
      polys->InsertNextCell(3, t1);
      polys->InsertNextCell(3, t2);
    }
  auto pd = vtkSmartPointer<vtkPolyData>::New();
  pd->SetPoints(pts);
  pd->SetPolys(polys);
  pd->GetPointData()->SetTCoords(tc);
  return withNormals(pd);
}

vtkSmartPointer<vtkPolyData> ribbonPoly(int n, double x0, double y0, double z) {
  auto pts = vtkSmartPointer<vtkPoints>::New();
  pts->SetDataTypeToFloat();
  auto polys = vtkSmartPointer<vtkCellArray>::New();
  for (int k = 0; k < n; ++k) {
    pts->InsertNextPoint(x0 + 0.7 * k, y0 - 0.5, z);
    pts->InsertNextPoint(x0 + 0.7 * k, y0 + 0.5, z);
  }
  for (int k = 0; k + 1 < n; ++k) {
    const vtkIdType l0 = 2 * k, r0 = l0 + 1, l1 = l0 + 2, r1 = l0 + 3;
    const vtkIdType t1[3] = {l0, r0, r1}, t2[3] = {l0, r1, l1};
    polys->InsertNextCell(3, t1);
    polys->InsertNextCell(3, t2);
  }
  auto pd = vtkSmartPointer<vtkPolyData>::New();
  pd->SetPoints(pts);
  pd->SetPolys(polys);
  return withNormals(pd);
}

vtkSmartPointer<vtkPolyData> linesPoly(int n, double x0, double y0, double z) {
  auto pts = vtkSmartPointer<vtkPoints>::New();
  pts->SetDataTypeToFloat();
  auto lines = vtkSmartPointer<vtkCellArray>::New();
  for (int k = 0; k < n; ++k) {
    pts->InsertNextPoint(x0 + 0.6 * k, y0 + 0.8 * std::cos(k * 0.5), z);
    if (k) {
      const vtkIdType ids[2] = {k - 1, k};
      lines->InsertNextCell(2, ids);
    }
  }
  auto pd = vtkSmartPointer<vtkPolyData>::New();
  pd->SetPoints(pts);
  pd->SetLines(lines);
  return pd;
}

// One cell type of a 3 x 4 point patch: 'v' verts, 's' a triangle strip. (One
// type per data set: VTK 9.5.0's low-memory mapper crashes on its first upload
// of a data set with two or more cell types -- vtkDrawTexturedElements::
// AppendArrayToTexture flags a not-yet-created buffer dirty; fixed upstream in
// 9.5.1 by 26ef893b5d9. cvcGL nodes always produce a single cell type.)
vtkSmartPointer<vtkPolyData> patchPoly(char kind, double x0, double y0, double z) {
  auto pts = vtkSmartPointer<vtkPoints>::New();
  pts->SetDataTypeToFloat();
  for (int k = 0; k < 12; ++k)
    pts->InsertNextPoint(x0 + (k % 4), y0 + (k / 4), z + 0.2 * (k % 2));
  auto cells = vtkSmartPointer<vtkCellArray>::New();
  auto pd = vtkSmartPointer<vtkPolyData>::New();
  pd->SetPoints(pts);
  if (kind == 'v') {
    for (vtkIdType k = 0; k < 12; ++k)
      cells->InsertNextCell(1, &k);
    pd->SetVerts(cells);
  } else {
    const vtkIdType a[8] = {0, 4, 1, 5, 2, 6, 3, 7}, b[8] = {4, 8, 5, 9, 6, 10, 7, 11};
    cells->InsertNextCell(8, a);
    cells->InsertNextCell(8, b);
    pd->SetStrips(cells);
  }
  return pd;
}

// An n x n patch of quads (4-point polygons: VTK triangulates them, so the
// draw needs the cell map) with one RGB colour per quad.
vtkSmartPointer<vtkPolyData> quadsPoly(int n, double x0, double y0, double z) {
  auto pts = vtkSmartPointer<vtkPoints>::New();
  pts->SetDataTypeToFloat();
  for (int j = 0; j <= n; ++j)
    for (int i = 0; i <= n; ++i)
      pts->InsertNextPoint(x0 + 0.8 * i, y0 + 0.8 * j, z + 0.15 * ((i + j) % 2));
  auto quads = vtkSmartPointer<vtkCellArray>::New();
  auto colours = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colours->SetNumberOfComponents(3);
  colours->SetName("quadColours");
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const vtkIdType a = j * (n + 1) + i, ids[4] = {a, a + 1, a + n + 2, a + n + 1};
      quads->InsertNextCell(4, ids);
      const unsigned char c[3] = {static_cast<unsigned char>(40 + 200 * i / n),
                                  static_cast<unsigned char>(40 + 200 * j / n),
                                  static_cast<unsigned char>((i + j) % 2 ? 220 : 60)};
      colours->InsertNextTypedTuple(c);
    }
  auto pd = vtkSmartPointer<vtkPolyData>::New();
  pd->SetPoints(pts);
  pd->SetPolys(quads);
  pd->GetCellData()->SetScalars(colours);
  return withNormals(pd);
}

// One RGB colour per point.
vtkSmartPointer<vtkPolyData> withPointColours(vtkSmartPointer<vtkPolyData> pd) {
  auto colours = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colours->SetNumberOfComponents(3);
  colours->SetName("pointColours");
  for (vtkIdType k = 0; k < pd->GetNumberOfPoints(); ++k) {
    const unsigned char c[3] = {static_cast<unsigned char>(37 * k % 256),
                                static_cast<unsigned char>(91 * k % 256),
                                static_cast<unsigned char>(151 * k % 256)};
    colours->InsertNextTypedTuple(c);
  }
  pd->GetPointData()->SetScalars(colours);
  return pd;
}

// Lines with a point normal each (VTK lights lines only with normals and
// non-flat interpolation).
vtkSmartPointer<vtkPolyData> withLineNormals(vtkSmartPointer<vtkPolyData> pd) {
  auto normals = vtkSmartPointer<vtkFloatArray>::New();
  normals->SetNumberOfComponents(3);
  normals->SetName("Normals");
  for (vtkIdType k = 0; k < pd->GetNumberOfPoints(); ++k) {
    const double a = 0.3 * k;
    normals->InsertNextTuple3(0.3 * std::cos(a), 0.3 * std::sin(a), 0.9);
  }
  pd->GetPointData()->SetNormals(normals);
  return pd;
}

vtkSmartPointer<vtkTexture> checkerTexture(int n) {
  auto img = vtkSmartPointer<vtkImageData>::New();
  img->SetDimensions(n, n, 1);
  img->AllocateScalars(VTK_UNSIGNED_CHAR, 4);
  auto *px = static_cast<unsigned char *>(img->GetScalarPointer());
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      unsigned char *q = px + 4 * (j * n + i);
      q[0] = static_cast<unsigned char>(i * 255 / n);
      q[1] = ((i / 4 + j / 4) % 2) ? 220 : 40;
      q[2] = 128;
      q[3] = 255;
    }
  auto tex = vtkSmartPointer<vtkTexture>::New();
  tex->SetInputData(img);
  tex->InterpolateOn();
  return tex;
}

// ── the scene ────────────────────────────────────────────────────────────────
struct Probe {
  std::string name;
  vtkSmartPointer<ProbeMapper> mapper;
  vtkSmartPointer<vtkActor> actor;
  // Expected to take the coincident-offset fallback (all four cell types) on
  // the Fast path.
  bool fallback = false;
};

struct Scene {
  explicit Scene(cvc::app &app) : sg(app, "lowmem") { sg.setDiagnosticChromeVisible(false); }
  SceneGraph sg;
  std::unique_ptr<SceneRenderer> sr;
  std::vector<std::shared_ptr<PeekNode>> nodes;
  std::deque<Probe> probes; // stable references across probe()
  int w = 320, h = 240;

  void open() {
    sr = std::make_unique<SceneRenderer>(sg, w, h, /*offscreen=*/true, "lowmem");
    // No MSAA: with VTK's default 8x, NVIDIA's resolve differs by 1 LSB at a few
    // edge pixels from one frame to the next for an identical GL stream, so two
    // stock frames would not compare equal either.
    sr->renderWindow()->SetMultiSamples(0);
    sr->setBackground(0.18, 0.22, 0.30);
    sr->setCamera(-14, -20, 16, 0, 0, 0, 0, 0, 1, 40.0, 1.0, 200.0);
    for (auto &p : probes)
      sr->renderer()->AddActor(p.actor);
  }

  std::shared_ptr<PeekNode> node(const std::string &name) {
    auto n = sg.getGraphicsRoot()->addGraphicsChild<PeekNode>(name);
    nodes.push_back(n);
    return n;
  }

  Probe &probe(const std::string &name, vtkSmartPointer<vtkPolyData> pd) {
    Probe p;
    p.name = name;
    p.mapper = vtkSmartPointer<ProbeMapper>::New();
    p.mapper->SetInputData(pd);
    p.mapper->ScalarVisibilityOff();
    p.actor = vtkSmartPointer<vtkActor>::New();
    p.actor->SetMapper(p.mapper);
    p.actor->GetProperty()->SetSpecular(0.3);
    p.actor->GetProperty()->SetSpecularPower(20);
    probes.push_back(p);
    return probes.back();
  }

  // Every LowMemoryPolyDataMapper drawing in this scene.
  std::vector<LowMemoryPolyDataMapper *> mappers() {
    std::vector<LowMemoryPolyDataMapper *> out;
    vtkActorCollection *actors = sr->renderer()->GetActors();
    actors->InitTraversal();
    while (vtkActor *a = actors->GetNextActor())
      if (auto *m = LowMemoryPolyDataMapper::SafeDownCast(a->GetMapper()))
        out.push_back(m);
    return out;
  }
  LowMemoryPolyDataMapper::Stats stats() {
    LowMemoryPolyDataMapper::Stats s;
    for (auto *m : mappers()) {
      const auto &t = m->stats();
      s.draws += t.draws;
      s.stockDraws += t.stockDraws;
      s.cellTypesSkipped += t.cellTypesSkipped;
      s.coincidentFallbacks += t.coincidentFallbacks;
      s.programLookups += t.programLookups;
      s.programLookupsSkipped += t.programLookupsSkipped;
    }
    return s;
  }
  void resetCounters() {
    for (auto *m : mappers())
      m->resetStats();
    for (auto &p : probes)
      p.mapper->clearTally();
  }

  // Render one frame; the GL calls it issued.
  GLCalls frame() {
    const GLCalls before = glcallsRead();
    sr->render();
    return glcallsRead() - before;
  }
  std::vector<unsigned char> rgba() {
    vtkNew<vtkUnsignedCharArray> px;
    sr->renderWindow()->GetRGBACharPixelData(0, 0, w - 1, h - 1, /*front=*/0, px);
    const unsigned char *p = px->GetPointer(0);
    return std::vector<unsigned char>(p, p + px->GetNumberOfValues());
  }
};

void buildCity(Scene &s) {
  // Nodes: every kind of GeometryNode a scene uses.
  auto veh = s.node("veh");
  veh->setGeometry(boxGeometry(0, 0, 1.2, 1.6, 0.9, 0.7));
  veh->setUseSingleColor(true);
  veh->setColor(0.3, 0.5, 0.9);
  const double pose[16] = {0.8, -0.6, 0, 2.5, 0.6, 0.8, 0, -1.0, 0, 0, 1, 0.1, 0, 0, 0, 1};
  veh->setPoseMatrix(pose);

  auto ground = s.node("ground");
  ground->setGeometry(groundGeometry(40, 14.0));
  ground->setUseSingleColor(true);
  ground->setColor(1, 1, 1);
  ground->setAmbient(0.4);
  ground->setDiffuse(0.7);
  ground->setTexture(checkerImage(64));

  auto ribbon = s.node("ribbon");
  ribbon->setGeometry(ribbonGeometry(30, -9, 4, 0.9));
  ribbon->setUseSingleColor(true);
  ribbon->setColor(0.98, 0.85, 0.3);
  ribbon->setOpacity(0.6);
  ribbon->setDepthOffset(2.0);

  auto trail = s.node("trail");
  trail->setGeometry(polylineGeometry(40, -10, -6, 0.8));
  trail->setRenderMode(GeometryRenderMode::LINES); // after setGeometry, which picks a mode
  trail->setUseSingleColor(true);
  trail->setColor(0.2, 0.95, 0.3);
  trail->setLineWidth(2.0);

  auto cloud = s.node("cloud");
  cloud->setGeometry(cloudGeometry(60, 5, -9, 1.5));
  cloud->setRenderMode(GeometryRenderMode::POINTS);
  cloud->setUseSingleColor(true);
  cloud->setColor(0.95, 0.3, 0.9);
  cloud->setPointSize(4.0);

  auto vcol = s.node("vcol");
  vcol->setUseSingleColor(false);
  vcol->setGeometry(colouredGeometry(-4, -3));

  auto quads = s.node("quads");
  {
    cvc::geometry g;
    for (int j = 0; j < 4; ++j)
      for (int i = 0; i < 4; ++i)
        g.points().push_back({1.0 + 0.9 * i, 3.0 + 0.9 * j, 0.4 + 0.1 * ((i + j) % 2)});
    for (unsigned j = 0; j + 1 < 4; ++j)
      for (unsigned i = 0; i + 1 < 4; ++i) {
        const unsigned a = j * 4 + i;
        g.quads().push_back({a, a + 1, a + 5, a + 4});
      }
    quads->setGeometry(g);
  }
  quads->setRenderMode(GeometryRenderMode::QUADS);
  quads->setUseSingleColor(true);
  quads->setColor(0.6, 0.4, 0.95);

  // Probes: the same kinds as raw actors whose draws are tallied.
  {
    Probe &p = s.probe("lit", spherePoly(1.4));
    p.actor->GetProperty()->SetColor(0.85, 0.35, 0.25);
    vtkNew<vtkMatrix4x4> m;
    m->SetElement(0, 3, 6.0);
    m->SetElement(1, 3, 3.0);
    m->SetElement(2, 3, 1.6);
    p.actor->SetUserMatrix(m);
  }
  {
    Probe &p = s.probe("textured", terrainPoly(24, -13, 6, 7));
    p.actor->GetProperty()->SetColor(1, 1, 1);
    p.actor->SetTexture(checkerTexture(32));
  }
  {
    Probe &p = s.probe("translucent", ribbonPoly(24, -6, 8.5, 0.7));
    p.actor->GetProperty()->SetColor(0.3, 0.9, 0.95);
    p.actor->GetProperty()->SetOpacity(0.5);
    p.mapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(0.0, -2.0);
    p.mapper->SetRelativeCoincidentTopologyLineOffsetParameters(0.0, -2.0);
    p.mapper->SetRelativeCoincidentTopologyPointOffsetParameter(-2.0);
  }
  {
    Probe &p = s.probe("lines", linesPoly(30, -12, -10, 0.5));
    p.actor->GetProperty()->SetColor(0.95, 0.95, 0.2);
  }
  {
    Probe &p = s.probe("verts", patchPoly('v', 8, -12, 0.8));
    p.actor->GetProperty()->SetColor(0.7, 0.7, 0.9);
    p.actor->GetProperty()->SetPointSize(5.0);
  }
  {
    Probe &p = s.probe("strips", patchPoly('s', 2, 9, 0.6));
    p.actor->GetProperty()->SetColor(0.5, 0.9, 0.6);
  }
  // Representations of a triangle mesh other than the surface.
  {
    Probe &p = s.probe("wireframe", spherePoly(1.3));
    p.actor->GetProperty()->SetRepresentationToWireframe();
    p.actor->GetProperty()->SetColor(0.95, 0.6, 0.2);
    p.actor->SetPosition(-8, 0, 2);
  }
  {
    Probe &p = s.probe("pointsRep", spherePoly(1.3)); // lit: point normals, Gouraud
    p.actor->GetProperty()->SetRepresentationToPoints();
    p.actor->GetProperty()->SetPointSize(3.0);
    p.actor->GetProperty()->SetColor(0.3, 0.95, 0.95);
    p.actor->SetPosition(8, -4, 2);
  }
  // Edge visibility on a surface puts every draw under the coincident-offset
  // rules: with VTK's defaults the polys draw sees the -4 units the empty lines
  // block left in the program, so Fast must fall back to all four ...
  {
    Probe &p = s.probe("edges", spherePoly(1.2));
    p.actor->GetProperty()->EdgeVisibilityOn();
    p.actor->GetProperty()->SetEdgeColor(0.1, 0.1, 0.1);
    p.actor->GetProperty()->SetColor(0.8, 0.8, 0.5);
    p.actor->SetPosition(0, 6, 2);
    p.fallback = true;
  }
  // ... while one whose own polygon offset is non-zero skips them.
  {
    Probe &p = s.probe("edgesOffset", ribbonPoly(10, -7, -12, 0.5));
    p.actor->GetProperty()->EdgeVisibilityOn();
    p.actor->GetProperty()->SetEdgeColor(0.9, 0.1, 0.1);
    p.actor->GetProperty()->SetColor(0.4, 0.5, 0.9);
    p.mapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(0.0, -2.0);
    p.mapper->SetRelativeCoincidentTopologyLineOffsetParameters(0.0, -2.0);
    p.mapper->SetRelativeCoincidentTopologyPointOffsetParameter(-2.0);
  }
  // Scalars: cell colours on quads (cell map), point colours on a sphere.
  {
    Probe &p = s.probe("cellColours", quadsPoly(5, -3, -9, 0.3));
    p.mapper->ScalarVisibilityOn();
    p.mapper->SetScalarModeToUseCellData();
    p.mapper->SetColorModeToDirectScalars();
  }
  {
    Probe &p = s.probe("pointColours", withPointColours(spherePoly(1.1)));
    p.mapper->ScalarVisibilityOn();
    p.mapper->SetScalarModeToUsePointData();
    p.mapper->SetColorModeToDirectScalars();
    p.actor->SetPosition(-3, 3, 3);
  }
  // Lines: flat with normals (unlit), Gouraud with normals and wide (lit,
  // instanced), and drawn as points.
  {
    Probe &p = s.probe("flatLines", withLineNormals(linesPoly(20, -9, 8, 1.0)));
    p.actor->GetProperty()->SetInterpolationToFlat();
    p.actor->GetProperty()->SetColor(0.9, 0.3, 0.3);
  }
  {
    Probe &p = s.probe("litLines", withLineNormals(linesPoly(20, -9, 10, 1.2)));
    p.actor->GetProperty()->SetInterpolationToGouraud();
    p.actor->GetProperty()->SetLineWidth(3.0);
    p.actor->GetProperty()->SetColor(0.3, 0.3, 0.9);
  }
  {
    Probe &p = s.probe("linePoints", linesPoly(20, 0, -6, 1.4));
    p.actor->GetProperty()->SetRepresentationToPoints();
    p.actor->GetProperty()->SetPointSize(4.0);
    p.actor->GetProperty()->SetColor(0.95, 0.95, 0.95);
  }
}

void addLights(SceneGraph &sg) {
  sg.addSpotLight(-12, -16, 26, 0, 0, 0, 40.0, 1.0, 0.97, 0.9, 1.0);
  sg.addFillLight(14, 10, 20, 0, 0, 0, 0.8, 0.85, 1.0, 0.4);
}

struct PathRun {
  GLCalls first, steady;                        // whole frame
  Trace firstTrace, steadyTrace;                // whole frame, with stage markers
  std::vector<unsigned char> firstPx, steadyPx; // RGBA
  LowMemoryPolyDataMapper::Stats firstStats, steadyStats;
  std::vector<GLCalls> probeCalls; // steady frame, per probe
  std::vector<double> probeDraws;  // steady frame, per probe
  std::vector<GLCalls> probeFirst; // first frame, per probe
  std::vector<double> probeFirstDraws;
};

// Two frames on one draw path: `first` (a real shadow bake when shadows are on:
// every opaque mapper draws twice, rebuilding its shader for each pass) and
// `steady` (re-uses the bake; nothing rebuilt).
PathRun runPath(Scene &s, DrawPath path) {
  LowMemoryPolyDataMapper::setDrawPath(path);
  // One frame on this path first, so the counted frames start from the GL
  // state this path leaves (on desktop the empty verts agent's glPointSize is
  // cached state, so the first frame after a switch would differ by one call).
  s.sr->render();
  PathRun r;
  // vtkShadowMapBakerPass only re-bakes when a light or prop changed since the
  // last bake; touch one so the first frame really bakes.
  s.sg.invalidateShadowBake();
  if (vtkActor *a = s.sr->renderer()->GetActors()->GetLastActor())
    a->Modified();
  s.resetCounters();
  cvcgl_test::glcallsTraceStart();
  r.first = s.frame();
  r.firstTrace = cvcgl_test::glcallsTraceStop();
  r.firstStats = s.stats();
  for (auto &p : s.probes) {
    r.probeFirst.push_back(p.mapper->calls);
    r.probeFirstDraws.push_back(p.mapper->draws);
  }
  r.firstPx = s.rgba();
  s.resetCounters();
  cvcgl_test::glcallsTraceStart();
  r.steady = s.frame();
  r.steadyTrace = cvcgl_test::glcallsTraceStop();
  r.steadyStats = s.stats();
  for (auto &p : s.probes) {
    r.probeCalls.push_back(p.mapper->calls);
    r.probeDraws.push_back(p.mapper->draws);
  }
  r.steadyPx = s.rgba();
  return r;
}

// ── pixels, against the renderer's own self-variance ─────────────────────────
// Two RGBA frames: how many bytes differ, by how much at most, and where first.
struct PixelDiff {
  long bytes = 0; // -1: frames of different sizes, or empty
  int maxDelta = 0;
  long first = -1; // the pixel (index) of the first differing byte
};

PixelDiff pixelDiff(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b) {
  PixelDiff d;
  if (a.size() != b.size() || a.empty()) {
    d.bytes = -1;
    return d;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    const int delta = std::abs(int(a[i]) - int(b[i]));
    if (!delta)
      continue;
    if (d.first < 0)
      d.first = static_cast<long>(i / 4);
    ++d.bytes;
    d.maxDelta = std::max(d.maxDelta, delta);
  }
  return d;
}

std::string describe(const PixelDiff &d, int w) {
  if (d.bytes < 0)
    return "frames of different sizes (or none)";
  if (d.bytes == 0)
    return "byte-identical";
  return fmt("%.0f differing bytes, max delta %.0f", double(d.bytes), double(d.maxDelta)) +
         fmt(", first at pixel (%.0f, %.0f)", double(d.first % w), double(d.first / w));
}

// Pixels are judged against what the renderer itself does with one GL stream.
// Two Stock frames whose GL traces are identical -- VTK's own draws, every
// call with its arguments and uniform values, in order, in one window -- can
// differ only by what the rasteriser does with that stream. NVIDIA and Mesa
// llvmpipe give the same bytes every time. Apple's software renderer (macOS
// CI, "Apple Software Renderer", GL 4.1 APPLE-23.1.1) does not, and not only
// across windows. Measured on the first attempts of macOS CI runs, every
// difference by a single LSB (max delta 1), in some runs and not in others:
//   - in one window, shadows on (never off): AllCellTypes, Fast and a Stock
//     frame right after Fast against Stock 1 to 7 bytes apart, Fast after a
//     mapper's resources were released 1 to 8 -- in runs where every pair of
//     Stock frames sampled in that window was byte-identical, so no pair a run
//     samples can bound what its other frames do;
//   - across a program recompile (the shader cache's programs released) or a
//     new window (new context, new cache): Fast, and Stock against Stock --
//     the fast path not involved -- 1 to 10 bytes apart.
//
// Exact mode (every renderer but Apple's): every pixel check is byte for byte,
// and a sampled pair -- two Stock frames of one GL trace, in one window,
// neither right after Fast -- that differs fails the run on its own.
//
// Apple-tolerance mode (the GL version or renderer string names Apple): every
// pixel check -- in one window, across a recompile, across a new window --
// passes iff its two frames are byte-identical or differ by LSB rounding
// alone: no byte off by more than 1 (kAppleLsbMaxDelta), at most 64 bytes
// (kAppleLsbMaxBytes, ~6x the most measured). The envelope is the
// renderer's, fixed, not the run's: no frame widens it, so no frame can
// excuse itself, and a shading change (a colour off by 2 or more) or a
// changed region (more than 64 bytes) fails. The pairs of Stock frames are
// held to the same envelope, and those that differ are printed as well, a
// diagnostic of the renderer's self-variance in that run; they excuse
// nothing. Byte counts that are no multiple of 3 (1, 2, 4, 5, ...) mean
// pixels that changed in some colour channels and not the others: rounding,
// not a shading change. Hence the relaxation is Apple's alone, where it is
// needed; the GL traces stay the exact oracle on every renderer, and point
// picking has its own rule (comparePicking()).
constexpr long kAppleLsbMaxBytes = 64;
constexpr int kAppleLsbMaxDelta = 1;

// Apple-tolerance mode: identical, or LSB rounding at a few pixels.
bool withinAppleLsbEnvelope(const PixelDiff &d) {
  return d.bytes >= 0 && d.bytes <= kAppleLsbMaxBytes && d.maxDelta <= kAppleLsbMaxDelta;
}

std::string appleLsbEnvelope() {
  return fmt("Apple LSB envelope (delta<=%.0f, bytes<=%.0f)", double(kAppleLsbMaxDelta),
             double(kAppleLsbMaxBytes));
}

// The renderer the frames are drawn by, as renderAvailable() reads it, and
// whether its pixels are judged in Apple-tolerance mode (exact otherwise).
std::string g_glRenderer, g_glVersion;
bool g_appleTolerance = false;

// Every pixel check's verdict, and its detail: byte for byte, or in
// Apple-tolerance mode the fixed LSB envelope -- the same rule wherever the
// two frames come from.
bool pixelsMatch(const PixelDiff &d) {
  return g_appleTolerance ? withinAppleLsbEnvelope(d) : d.bytes == 0;
}

std::string pixelVerdict(const PixelDiff &d, int w) {
  std::string detail = describe(d, w);
  if (g_appleTolerance && d.bytes > 0)
    detail += (pixelsMatch(d) ? ": within the " : ": BEYOND the ") + appleLsbEnvelope();
  return detail;
}

struct SelfVarianceSample {
  std::string scope, what;
  PixelDiff d;
  int w;
};
std::vector<SelfVarianceSample> g_noisy;   // the sampled pairs that differed
std::map<std::string, int> g_pairs;        // pairs sampled, per scope
std::map<std::string, int> g_pairsSkipped; // not sampled (traces differ), per scope

struct PixelCheck {
  std::string what;
  PixelDiff d;
  int w;
};
std::vector<PixelCheck> g_pixelChecks;

void sampleSelfVariance(const std::string &scope, const std::string &what, const Trace &ta,
                        const Trace &tb, const std::vector<unsigned char> &a,
                        const std::vector<unsigned char> &b, int w) {
  std::string why;
  if (!sameTrace(ta, tb, &why)) {
    ++g_pairsSkipped[scope]; // not one GL stream: what the frames differ by is not noise
    std::printf("  self-variance: %s not sampled, GL traces differ -- %s\n", what.c_str(),
                why.c_str());
    return;
  }
  const PixelDiff d = pixelDiff(a, b);
  if (d.bytes < 0) {
    ++g_pairsSkipped[scope];
    return;
  }
  ++g_pairs[scope];
  if (!g_appleTolerance) { // exact mode: the pair is itself a check
    check(d.bytes == 0, what + " is byte-identical on a deterministic renderer",
          d.bytes ? describe(d, w) + ", identical GL traces" : std::string());
    return;
  }
  // Apple-tolerance mode: the pair is held to the same fixed envelope as every
  // other pixel check, and printed when it differs at all.
  check(withinAppleLsbEnvelope(d),
        what + " is byte-identical or within the " + appleLsbEnvelope() + " on Apple's renderer",
        d.bytes ? describe(d, w) + ", identical GL traces" : std::string());
  if (d.bytes == 0)
    return;
  g_noisy.push_back({scope, what, d, w}); // a diagnostic: it excuses nothing
  std::printf("  self-variance (diagnostic): %s -- %s, identical GL traces\n", what.c_str(),
              pixelVerdict(d, w).c_str());
}

// Two Stock runs of one draw path: the first frames (a shadow bake when
// shadows are on) and the steady frames, which sample that bake -- so the
// steady pair counts only if the first frames' traces match too.
void sampleSelfVariance(const std::string &scope, const std::string &what, const PathRun &a,
                        const PathRun &b, int w) {
  const bool firstSame = sameTrace(a.firstTrace, b.firstTrace, nullptr);
  sampleSelfVariance(scope, what + ", first frame", a.firstTrace, b.firstTrace, a.firstPx,
                     b.firstPx, w);
  if (firstSame)
    sampleSelfVariance(scope, what + ", steady frame", a.steadyTrace, b.steadyTrace, a.steadyPx,
                       b.steadyPx, w);
  else
    ++g_pairsSkipped[scope];
}

// Deferred: judged by checkPixels(), after the run's samples are printed.
void samePixels(const std::string &what, const std::vector<unsigned char> &a,
                const std::vector<unsigned char> &b, int w) {
  g_pixelChecks.push_back({what, pixelDiff(a, b), w});
}

void checkPixels() {
  if (g_pixelChecks.empty())
    return;
  if (!g_appleTolerance) {
    std::printf("pixels (exact mode: byte for byte)\n");
  } else {
    std::printf("pixels (apple-tolerance mode: every check, in one window or across a recompile "
                "or a new window, byte-identical or within the %s; the pairs of Stock frames "
                "sampled are held to the same envelope and excuse nothing)\n",
                appleLsbEnvelope().c_str());
    std::set<std::string> scopes;
    for (const auto &p : g_pairs)
      scopes.insert(p.first);
    for (const auto &p : g_pairsSkipped)
      scopes.insert(p.first);
    for (const auto &scope : scopes) {
      int noisy = 0;
      for (const auto &n : g_noisy)
        noisy += n.scope == scope;
      std::string line = "  self-variance (diagnostic): " + scope +
                         fmt(": %.0f pairs of Stock frames, ", double(g_pairs[scope]));
      line += noisy ? fmt("%.0f differ (listed above)", double(noisy)) : std::string("none differ");
      if (g_pairsSkipped[scope])
        line += fmt("; %.0f not sampled", double(g_pairsSkipped[scope]));
      std::printf("%s\n", line.c_str());
    }
  }
  for (const auto &c : g_pixelChecks)
    check(pixelsMatch(c.d), c.what, pixelVerdict(c.d, c.w));
}

// Every pixel the background colour? (a frame that drew nothing)
bool drewSomething(const std::vector<unsigned char> &px) {
  if (px.size() < 8)
    return false;
  for (size_t i = 4; i < px.size(); i += 4)
    if (px[i] != px[0] || px[i + 1] != px[1] || px[i + 2] != px[2])
      return true;
  return false;
}

// ── 1. policy ────────────────────────────────────────────────────────────────
void testPolicy() {
  std::printf("policy\n");
  check(cvc::gl::lowMemoryMapperPolicy() == LowMemoryMapperPolicy::Force,
        "CVCGL_LOWMEM_MAPPER=force is read on first use");
  check(LowMemoryPolyDataMapper::SafeDownCast(cvc::gl::newPolyDataMapper()) != nullptr,
        "Force: newPolyDataMapper() is a LowMemoryPolyDataMapper");
  const bool factoryLowMem =
      vtkSmartPointer<vtkPolyDataMapper>::New()->IsA("vtkOpenGLLowMemoryPolyDataMapper") != 0;
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Off);
  check(LowMemoryPolyDataMapper::SafeDownCast(cvc::gl::newPolyDataMapper()) == nullptr,
        "Off: newPolyDataMapper() is the factory's mapper");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Auto);
  check((LowMemoryPolyDataMapper::SafeDownCast(cvc::gl::newPolyDataMapper()) != nullptr) ==
            factoryLowMem,
        "Auto: LowMemoryPolyDataMapper exactly where the factory hands out the low-memory one",
        factoryLowMem ? "factory: low-memory" : "factory: classic");
  cvc::gl::setLowMemoryMapperPolicy(LowMemoryMapperPolicy::Force);
  if (!std::getenv("CVCGL_LOWMEM_DRAW"))
    check(LowMemoryPolyDataMapper::drawPath() == DrawPath::Fast, "the default draw path is Fast");
}

// Can this build rasterise at all? Asked of a plain node in a scene of its own.
bool renderAvailable(cvc::app &app) {
  bool drew = false;
  {
    Scene s(app);
    auto n = s.node("control");
    n->setGeometry(boxGeometry(0, 0, 0, 5, 5, 0.5));
    n->setColor(1, 1, 1);
    s.w = 48;
    s.h = 48;
    s.open();
    s.sr->setCamera(0, 0, 50, 0, 0, 0, 0, 1, 0, 30.0, 1.0, 200.0);
    s.sr->render();
    drew = drewSomething(s.rgba());
    // Which renderer the pixel and round-trip checks below ran on, and so how
    // its pixels are judged (see checkPixels()).
    if (auto *win = vtkOpenGLRenderWindow::SafeDownCast(s.sr->renderWindow())) {
      const char *report = win->ReportCapabilities();
      const std::string caps = report ? report : "";
      const auto value = [&caps](const std::string &key) {
        const size_t at = caps.find(key);
        if (at == std::string::npos)
          return std::string();
        const size_t end = std::min(caps.find('\n', at), caps.size());
        const size_t from = std::min(caps.find_first_not_of(' ', at + key.size()), end);
        return caps.substr(from, end - from);
      };
      g_glRenderer = value("OpenGL renderer string:");
      g_glVersion = value("OpenGL version string:");
      const auto apple = [](const std::string &v) {
        return v.find("APPLE") != std::string::npos || v.find("Apple") != std::string::npos;
      };
      g_appleTolerance = apple(g_glVersion) || apple(g_glRenderer);
      std::printf("  maximum hardware line width: %g\n",
                  double(win->GetMaximumHardwareLineWidth()));
    }
    const char *mode = g_appleTolerance ? "apple-tolerance" : "exact";
    std::string rule = "every pair of Stock frames and every check byte for byte";
    if (g_appleTolerance)
      rule = "every check, in one window or across a recompile or a new window, byte-identical or "
             "within the " +
             appleLsbEnvelope() +
             ", pairs of Stock frames held to the same envelope (they excuse nothing); point "
             "picking by <= 1 point per prop, cell picking exact";
    std::printf("  pixel checks: %s (OpenGL renderer \"%s\", version \"%s\") -- %s\n", mode,
                g_glRenderer.c_str(), g_glVersion.c_str(), rule.c_str());
  }
  if (drew)
    return true;
  const char *require = std::getenv("CVC_REQUIRE_RENDER");
  if (require && *require && std::string(require) != "0")
    check(false, "CVC_REQUIRE_RENDER is set, but this build did not rasterise");
  else
    std::printf("  skipped: this build did not rasterise\n");
  return false;
}

// ── 2-5. replica, derived Fast trace, pixels, savings ────────────────────────
// The replica (AllCellTypes) against Stock, call for call, and Fast against the
// trace derived from it, for one frame of each kind.
// CVCGL_LOWMEM_DUMP=<dir>: write the traces of each failing comparison there
// (<n>_<trace>.txt, one call per line), for a diff.
void dumpTraces(const std::string &what, const std::vector<std::pair<std::string, Trace>> &ts) {
  static int dumped = 0;
  const char *dir = std::getenv("CVCGL_LOWMEM_DUMP");
  if (!dir || !*dir)
    return;
  ++dumped;
  for (const auto &[name, t] : ts) {
    const std::string path = std::string(dir) + "/" + std::to_string(dumped) + "_" + name + ".txt";
    if (FILE *f = std::fopen(path.c_str(), "w")) {
      std::fprintf(f, "# %s\n", what.c_str());
      for (const auto &r : t)
        std::fprintf(f, "%s\n", cvcgl_test::glcallsDescribe(r).c_str());
      std::fclose(f);
    }
  }
}

// mayDraw: whether the frame has replica draws at all (a cell-picking trace is
// selection passes only, every one of them the stock draw).
void checkTraces(const std::string &what, const Trace &stock, const Trace &all, const Trace &fast,
                 const LowMemoryPolyDataMapper::Stats &fastStats, bool mayDraw = true) {
  std::string why;
  check(countStage(stock, DrawStage::DrawBegin) == 0 &&
            countStage(all, DrawStage::RebindBegin) == 0 &&
            countStage(all, DrawStage::AgentSkipped) == 0 &&
            (countStage(all, DrawStage::DrawBegin) > 0) == mayDraw,
        what + ": Stock reports no stages; AllCellTypes looks every program up and runs every "
               "cell type");
  // (Each comparison runs before its message is built: argument order is unspecified.)
  const bool replica = sameTrace(glOnly(all), glOnly(stock), &why);
  if (!replica)
    dumpTraces(what, {{"stock", stock}, {"all", all}});
  check(replica,
        what + ": AllCellTypes trace == Stock trace (every call, program/unit, bytes, values)",
        replica ? fmt("%.0f calls", double(stock.size())) : why);
  why.clear();
  // The draw state is read off the full AllCellTypes trace, before the blocks
  // go: what each draw saw there is what it must see on Fast.
  const Trace allState = asDrawState(all), fastState = asDrawState(fast);
  const Expected e = expectFast(allState);
  const Trace got = glOnly(fastState);
  const bool derived = sameTrace(got, glOnly(e.trace), &why);
  if (!derived)
    dumpTraces(what, {{"all", all}, {"fast", fast}, {"expected", e.trace}, {"fast_state", got}});
  check(derived,
        what + ": Fast trace == AllCellTypes trace minus the empty cell-type blocks whose "
               "removal no draw call's uniforms see (cached point size / line width compared "
               "at the draws)",
        derived ? fmt("%.0f calls; %.0f blocks removed, %.0f kept", double(got.size()),
                      double(e.blocksRemoved), double(e.blocksKept)) +
                      (e.firstFallback.empty() ? "" : "; kept e.g. " + e.firstFallback)
                : why);
  why.clear();
  // Fast's own trace, replayed: every draw call sees the uniforms it sees on
  // AllCellTypes, and the frame leaves what the next frame's draws read.
  const UniformView allUniforms = replayUniforms(allState),
                    fastUniforms = replayUniforms(fastState);
  const bool uniforms = sameUniformsSeen(allUniforms, fastUniforms, allState, &why);
  if (!uniforms)
    dumpTraces(what + " (uniforms)", {{"all", allState}, {"fast", fastState}});
  check(uniforms,
        what + ": every Fast draw call sees AllCellTypes' uniform state (every location of "
               "its program), and the frame leaves the values the next frame reads",
        uniforms ? fmt("%.0f draw calls", double(fastUniforms.calls.size())) : why);
  // The mapper's own markers and stats: counts only.
  const int skipped = countStage(fast, DrawStage::AgentSkipped);
  check(e.blocksRemoved == skipped && (skipped > 0) == mayDraw &&
            static_cast<double>(skipped) == double(fastStats.cellTypesSkipped),
        what + ": Fast skipped as many blocks as derived",
        fmt("derived %.0f, Fast skipped %.0f (stats %.0f)", double(e.blocksRemoved),
            double(skipped), double(fastStats.cellTypesSkipped)));
  const int looked = countStage(fast, DrawStage::LookupBegin);
  const int rebound = countStage(fast, DrawStage::RebindBegin);
  int fastFallbacks = 0;
  for (const auto &r : fast)
    fastFallbacks += isStage(r, DrawStage::DrawBegin) && r.ctx == 0;
  check(looked + rebound == e.lookups && countStage(fast, DrawStage::DrawBegin) == e.draws &&
            e.fallbacks == fastFallbacks &&
            static_cast<double>(e.fallbacks) == double(fastStats.coincidentFallbacks),
        what + ": every AllCellTypes lookup is a Fast lookup or re-bind; same draws, same "
               "fallbacks",
        fmt("lookups %.0f = %.0f + %.0f re-bound", double(e.lookups), double(looked),
            double(rebound)) +
            fmt(", fallbacks derived %.0f, Fast %.0f (stats %.0f)", double(e.fallbacks),
                double(fastFallbacks), double(fastStats.coincidentFallbacks)));
}

// The hardware selector's passes on one draw path: what it selected, the GL
// trace of the selection, the stats.
struct Pick {
  std::string signature;
  std::vector<std::pair<double, double>> hits; // (prop id, items), in prop-id order
  Trace trace;
  LowMemoryPolyDataMapper::Stats stats;
};

// Apple-tolerance mode, point picking: the same props, each picked within
// `tolerance` items of the reference. Where the counts differ, *diff lists them.
bool pickCountsWithin(const Pick &ref, const Pick &p, double tolerance, const char *name,
                      std::string *diff) {
  if (p.hits.size() != ref.hits.size())
    return false;
  for (size_t i = 0; i < ref.hits.size(); ++i) {
    if (p.hits[i].first != ref.hits[i].first ||
        std::abs(p.hits[i].second - ref.hits[i].second) > tolerance)
      return false;
    if (p.hits[i].second != ref.hits[i].second)
      *diff += fmt("; prop %.0f: Stock %.0f, ", ref.hits[i].first, ref.hits[i].second) + name +
               fmt(" %.0f", p.hits[i].second);
  }
  return true;
}

Pick pick(Scene &s, DrawPath p, int association) {
  LowMemoryPolyDataMapper::setDrawPath(p);
  s.sr->render(); // the same state before each path's selection
  s.resetCounters();
  vtkNew<vtkHardwareSelector> sel;
  sel->SetRenderer(s.sr->renderer());
  sel->SetArea(0, 0, s.w - 1, s.h - 1);
  sel->SetFieldAssociation(association);
  cvcgl_test::glcallsTraceStart();
  vtkSmartPointer<vtkSelection> res = vtk::TakeSmartPointer(sel->Select());
  Pick out;
  out.trace = cvcgl_test::glcallsTraceStop();
  auto &hits = out.hits;
  for (unsigned i = 0; res && i < res->GetNumberOfNodes(); ++i) {
    vtkSelectionNode *n = res->GetNode(i);
    hits.emplace_back(n->GetProperties()->Get(vtkSelectionNode::PROP_ID()),
                      n->GetSelectionList() ? double(n->GetSelectionList()->GetNumberOfTuples())
                                            : 0.0);
  }
  std::sort(hits.begin(), hits.end());
  for (const auto &h : hits)
    out.signature += fmt("[%.0f:%.0f]", h.first, h.second);
  out.stats = s.stats();
  return out;
}

// Picking takes the stock draw on every path: identical selections and GL.
// (Point picking first renders one ordinary frame for the depth buffer --
// vtkOpenGLHardwareSelector::BeginSelection -- with no selector set, so those
// draws take the path under test; the trace check covers them like a frame.)
void comparePicking(Scene &s, const std::string &label) {
  // Replica draws in one ordinary frame (vertex visibility is stock anyway).
  LowMemoryPolyDataMapper::setDrawPath(DrawPath::Fast);
  s.sr->render();
  s.resetCounters();
  s.sr->render();
  const double frameDraws = double(s.stats().draws - s.stats().stockDraws);
  for (int assoc :
       {vtkDataObject::FIELD_ASSOCIATION_CELLS, vtkDataObject::FIELD_ASSOCIATION_POINTS}) {
    const bool points = assoc == vtkDataObject::FIELD_ASSOCIATION_POINTS;
    const std::string what = label + (points ? ": point picking" : ": cell picking");
    // Warm: the selection passes' shader variants are compiled by the first
    // selection on any path; compare the ones after.
    for (DrawPath p : {DrawPath::Stock, DrawPath::Fast})
      pick(s, p, assoc);
    const Pick stock = pick(s, DrawPath::Stock, assoc);
    const Pick all = pick(s, DrawPath::AllCellTypes, assoc);
    const Pick fast = pick(s, DrawPath::Fast, assoc);
    const Pick stock2 = pick(s, DrawPath::Stock, assoc);
    {
      std::string why;
      const bool reproducible = sameTrace(glOnly(stock2.trace), glOnly(stock.trace), &why);
      if (!reproducible)
        dumpTraces(what + " (Stock twice)", {{"stock", stock.trace}, {"stock2", stock2.trace}});
      check(reproducible, what + ": the selection trace is reproducible (Stock twice)",
            reproducible ? fmt("%.0f calls", double(stock.trace.size())) : why);
    }
    const double ordinary = points ? frameDraws : 0.0;
    check(double(fast.stats.draws - fast.stats.stockDraws) == ordinary &&
              double(all.stats.draws - all.stats.stockDraws) == ordinary &&
              fast.stats.stockDraws > 0,
          what + ": every selection-pass draw takes the stock path, on every path",
          fmt("stock %.0f of %.0f draws", double(fast.stats.stockDraws), double(fast.stats.draws)) +
              fmt(" (+%.0f in the ordinary depth frame)", ordinary));
    const bool sameSelection = !stock.signature.empty() && fast.signature == stock.signature &&
                               all.signature == stock.signature;
    // Apple's renderer, point picking only: the selector's point pass picked
    // 1117 points of a prop on one path and 1118 on another, in runs where
    // AllCellTypes' GL trace equalled Stock's -- the rasteriser's rounding at
    // a point's edge, like its pixels. There a path may pick 1 point more or
    // fewer per prop (the same props); cell picking, and every other renderer,
    // stay exact.
    std::string within;
    const bool nearSelection = !sameSelection && g_appleTolerance && points &&
                               !stock.signature.empty() &&
                               pickCountsWithin(stock, all, 1.0, "AllCellTypes", &within) &&
                               pickCountsWithin(stock, fast, 1.0, "Fast", &within);
    std::string detail = sameSelection ? stock.signature
                                       : "Stock " + stock.signature + " AllCellTypes " +
                                             all.signature + " Fast " + fast.signature;
    if (nearSelection)
      detail += ": within 1 point per prop on Apple's renderer" + within;
    else if (g_appleTolerance && points)
      detail += sameSelection ? " (Apple's renderer: <= 1 point per prop)"
                              : ": BEYOND 1 point per prop on Apple's renderer";
    check(sameSelection || nearSelection, what + " selects the same on every path", detail);
    checkTraces(what, stock.trace, all.trace, fast.trace, fast.stats, points);
  }
  LowMemoryPolyDataMapper::setDrawPath(DrawPath::Fast);
}

void comparePaths(Scene &s, const std::string &label, bool shadows) {
  std::printf("%s\n", label.c_str());
  // Settle: build every shader both ways first so no path pays a first compile
  // (Stock last: the first counted Stock run must not follow a Fast one).
  for (DrawPath p : {DrawPath::Fast, DrawPath::Stock})
    runPath(s, p);
  // Pairs of the Stock runs before Fast's (one after AllCellTypes) are checked
  // byte-identical -- on Apple's renderer, within its LSB envelope, and printed
  // as its self-variance; the Stock run right after Fast is judged, like the
  // AllCellTypes and Fast frames.
  const PathRun stock = runPath(s, DrawPath::Stock);
  const PathRun stock2 = runPath(s, DrawPath::Stock);
  const PathRun all = runPath(s, DrawPath::AllCellTypes);
  const PathRun stock3 = runPath(s, DrawPath::Stock);
  const PathRun fast = runPath(s, DrawPath::Fast);
  const PathRun afterFast = runPath(s, DrawPath::Stock);

  check(drewSomething(stock.steadyPx), label + ": the scene draws");
  std::string why;
  bool reproducible = true;
  for (const PathRun *again : {&stock2, &stock3, &afterFast})
    reproducible = reproducible && sameTrace(again->firstTrace, stock.firstTrace, &why) &&
                   sameTrace(again->steadyTrace, stock.steadyTrace, &why);
  check(reproducible, label + ": the GL trace is reproducible (Stock four times, same trace)",
        reproducible ? fmt("%.0f calls", double(stock.steadyTrace.size())) : why);
  sampleSelfVariance(label, label + ": Stock twice", stock, stock2, s.w);
  sampleSelfVariance(label, label + ": Stock twice", stock, stock3, s.w);
  sampleSelfVariance(label, label + ": Stock twice", stock2, stock3, s.w);
  checkTraces(label + ", first frame" + (shadows ? " (shadow bake)" : ""), stock.firstTrace,
              all.firstTrace, fast.firstTrace, fast.firstStats);
  checkTraces(label + ", steady frame", stock.steadyTrace, all.steadyTrace, fast.steadyTrace,
              fast.steadyStats);
  samePixels(label + ": AllCellTypes pixels == Stock, first frame", stock.firstPx, all.firstPx,
             s.w);
  samePixels(label + ": AllCellTypes pixels == Stock, steady frame", stock.steadyPx, all.steadyPx,
             s.w);
  samePixels(label + ": Fast pixels == Stock, first frame", stock.firstPx, fast.firstPx, s.w);
  samePixels(label + ": Fast pixels == Stock, steady frame", stock.steadyPx, fast.steadyPx, s.w);
  samePixels(label + ": a Stock frame right after Fast == Stock, first frame", stock.firstPx,
             afterFast.firstPx, s.w);
  samePixels(label + ": a Stock frame right after Fast == Stock, steady frame", stock.steadyPx,
             afterFast.steadyPx, s.w);

  std::printf("  frame GL calls, first  : Stock %s\n", stock.first.str().c_str());
  std::printf("                           Fast  %s\n", fast.first.str().c_str());
  std::printf("  frame GL calls, steady : Stock %s\n", stock.steady.str().c_str());
  std::printf("                           Fast  %s\n", fast.steady.str().c_str());
  check(fast.steady.total() < 0.75 * stock.steady.total(),
        label + ": Fast frame issues well under Stock's GL calls",
        fmt("steady %.0f -> %.0f, first %.0f", stock.steady.total(), fast.steady.total(),
            stock.first.total()) +
            fmt(" -> %.0f", fast.first.total()));
  check(fast.steady.roundTrips() == stock.steady.roundTrips(),
        label + ": Fast adds no desktop round-trip to the frame",
        fmt("%.0f vs %.0f", fast.steady.roundTrips(), stock.steady.roundTrips()));
  check(fast.steadyStats.programLookups == 0 && fast.steadyStats.programLookupsSkipped > 0,
        label + ": Fast steady frame re-binds every program without a lookup",
        fmt("lookups %.0f, skipped %.0f", double(fast.steadyStats.programLookups),
            double(fast.steadyStats.programLookupsSkipped)));
  // The only fallbacks are the probes built to need one.
  double expectFallbacks = 0;
  for (size_t i = 0; i < s.probes.size(); ++i)
    expectFallbacks += s.probes[i].fallback ? fast.probeDraws[i] : 0.0;
  check(fast.steadyStats.cellTypesSkipped > 0 &&
            double(fast.steadyStats.coincidentFallbacks) == expectFallbacks &&
            fast.steadyStats.stockDraws == 0,
        label + ": Fast skipped empty cell types; falls back only where an offset needs it",
        fmt("skipped %.0f of %.0f draws x 4, fallbacks %.0f",
            double(fast.steadyStats.cellTypesSkipped), double(fast.steadyStats.draws),
            double(fast.steadyStats.coincidentFallbacks)));
  check(stock.steadyStats.stockDraws == stock.steadyStats.draws && stock.steadyStats.draws > 0,
        label + ": Stock draws through VTK's own path");
  if (shadows)
    check(fast.firstStats.programLookups > 0,
          label + ": a shadow bake (render-pass change) takes the full lookup",
          fmt("lookups %.0f", double(fast.firstStats.programLookups)));

  // Per draw, on the probes (steady frame: one draw per probe and pass).
  // Without the window's limit no width counts as one the driver lacks.
  auto *glWin = vtkOpenGLRenderWindow::SafeDownCast(s.sr->renderWindow());
  const double maxLineWidth =
      glWin ? glWin->GetMaximumHardwareLineWidth() : std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < s.probes.size(); ++i) {
    const Probe &p = s.probes[i];
    const double nStock = stock.probeDraws[i], nFast = fast.probeDraws[i];
    const double nAll = all.probeDraws[i];
    if (nStock <= 0 || nFast <= 0 || nAll <= 0) {
      check(false, label + ": probe " + p.name + " drew");
      continue;
    }
    const GLCalls &cs = stock.probeCalls[i];
    const GLCalls &cf = fast.probeCalls[i];
    const GLCalls &ca = all.probeCalls[i];
    std::printf("  per draw %-12s Stock %s\n", p.name.c_str(), cs.str(nStock).c_str());
    std::printf("  %-21s Fast  %s\n", "", cf.str(nFast).c_str());
    // Desktop round-trips: Stock's draw has none but VTK's own warning about a
    // line width the driver cannot draw -- vtkDrawTexturedElements::PreDraw
    // (9.5.0) logs it with one glGetString(GL_VERSION) per draw of lines. A
    // core profile without wide lines (macOS: GetMaximumHardwareLineWidth() 1)
    // does that for litLines (width 3) on every path; where the driver draws
    // the width (NVIDIA, llvmpipe) Stock has none. Fast and AllCellTypes add
    // none.
    const double lineWidth = p.actor->GetProperty()->GetLineWidth();
    vtkPolyData *input = p.mapper->GetInput();
    const bool drawsLines = (input && input->GetNumberOfLines() > 0) ||
                            p.actor->GetProperty()->GetRepresentation() == VTK_WIREFRAME;
    const bool vtkWarning = drawsLines && lineWidth > maxLineWidth &&
                            cs.roundTrips() == cs.count("GetString") &&
                            cs.count("GetString") == nStock;
    const bool stockExplained = cs.roundTrips() == 0 || vtkWarning;
    const double tripsStock = cs.roundTrips() / nStock;
    check(stockExplained && cf.roundTrips() / nFast <= tripsStock &&
              ca.roundTrips() / nAll <= tripsStock,
          label + ": " + p.name +
              " draws issue no desktop round-trip beyond Stock's (none, but VTK's warning on a "
              "line width the driver lacks)",
          fmt("per draw: Stock %.1f, AllCellTypes %.1f", tripsStock, ca.roundTrips() / nAll) +
              fmt(", Fast %.1f", cf.roundTrips() / nFast) +
              (cs.roundTrips() == 0
                   ? std::string()
                   : fmt("; Stock's glGetString %.1f, line width %g, driver max %g",
                         cs.count("GetString") / nStock, lineWidth, maxLineWidth)));
    check(cf.count("DrawArraysInstanced") == cs.count("DrawArraysInstanced") &&
              cf.count("DrawArraysInstanced") == nFast,
          label + ": " + p.name + " one draw call per draw, as Stock");
    if (p.fallback) {
      check(cf.total() / nFast == cs.total() / nStock,
            label + ": " + p.name + " falls back to all four cell types (Stock's calls)",
            fmt("%.1f vs %.1f per draw", cf.total() / nFast, cs.total() / nStock));
      continue;
    }
    check(cf.count("UseProgram") <= 2 * nFast, label + ": " + p.name + " <= 2 glUseProgram/draw");
    check(cf.count("BindVertexArray") == 2 * nFast,
          label + ": " + p.name + " binds the VAO once per draw (+ unbind)",
          fmt("%.1f per draw", cf.count("BindVertexArray") / nFast));
    const double perFast = cf.total() / nFast, perStock = cs.total() / nStock;
    check(perFast <= 0.45 * perStock, label + ": " + p.name + " Fast <= 45% of Stock calls/draw",
          fmt("%.1f -> %.1f", perStock, perFast));
  }
  if (g_report) {
    LowMemoryPolyDataMapper::setDrawPath(DrawPath::Fast);
    s.resetCounters();
    s.sr->render();
    for (auto *m : s.mappers()) {
      const auto &t = m->stats();
      vtkPolyData *in = m->GetInput();
      std::printf(
          "  mapper %-24s points %5lld v/l/p/s %lld/%lld/%lld/%lld: draws %llu skipped %llu "
          "fallbacks %llu lookups %llu/%llu\n",
          m->GetClassName(), static_cast<long long>(in->GetNumberOfPoints()),
          static_cast<long long>(in->GetNumberOfVerts()),
          static_cast<long long>(in->GetNumberOfLines()),
          static_cast<long long>(in->GetNumberOfPolys()),
          static_cast<long long>(in->GetNumberOfStrips()), static_cast<unsigned long long>(t.draws),
          static_cast<unsigned long long>(t.cellTypesSkipped),
          static_cast<unsigned long long>(t.coincidentFallbacks),
          static_cast<unsigned long long>(t.programLookups),
          static_cast<unsigned long long>(t.programLookupsSkipped));
    }
    for (size_t i = 0; i < s.probes.size(); ++i)
      std::printf("  first frame per draw %-12s Stock %s\n  %-33s Fast  %s\n",
                  s.probes[i].name.c_str(),
                  stock.probeFirst[i].str(std::max(1.0, stock.probeFirstDraws[i])).c_str(), "",
                  fast.probeFirst[i].str(std::max(1.0, fast.probeFirstDraws[i])).c_str());
  }
  // The hardware selector over the whole probe set. Not with shadows on:
  // cvcGL's shadow pass chain renders props outside the selector's
  // BeginRenderProp/EndRenderProp, which VTK rejects ("Too many props") on
  // every path alike.
  if (!shadows)
    comparePicking(s, label);
}

// ── 6. lifecycle ─────────────────────────────────────────────────────────────
// scope: the scene's label, under which this lifecycle's pairs of Stock frames are counted
// and printed.
void testLifecycle(Scene &s, const std::string &scope) {
  std::printf("lifecycle\n");
  LowMemoryPolyDataMapper::setDrawPath(DrawPath::Fast);
  s.sr->render();

  // GetShader() hands out a mutable shader source: that mapper's next draw
  // resolves its program in the cache again; nobody else's does.
  {
    s.sr->render();
    s.resetCounters();
    LowMemoryPolyDataMapper *m = s.probes.front().mapper;
    m->GetShader(vtkShader::Fragment);
    s.sr->render();
    const auto all = s.stats();
    check(m->stats().programLookups == 1 && all.programLookups == 1,
          "GetShader(): that mapper's next draw looks its program up (and only that one)",
          fmt("lookups: mapper %.0f, scene %.0f", double(m->stats().programLookups),
              double(all.programLookups)));
    s.resetCounters();
    m->invalidateProgramCache();
    s.sr->render();
    check(m->stats().programLookups == 1 && s.stats().programLookups == 1,
          "invalidateProgramCache(): the same");
  }

  // A mapper's graphics resources released (what removing a prop from its
  // renderer does): its next draw looks the program up afresh.
  for (auto *m : s.mappers())
    m->ReleaseGraphicsResources(s.sr->renderWindow());
  s.resetCounters();
  s.sr->render();
  const auto rel = s.stats();
  check(rel.programLookups == rel.draws && rel.programLookupsSkipped == 0 && rel.draws > 0,
        "after a mapper's ReleaseGraphicsResources its next draw looks the program up",
        fmt("lookups %.0f of %.0f draws", double(rel.programLookups), double(rel.draws)));
  const PathRun released = runPath(s, DrawPath::Fast);
  // Stock to compare against: two runs, a self-variance pair (checked
  // byte-identical, or on Apple's renderer within its LSB envelope), after one
  // that follows Fast (judged, not sampled).
  const PathRun afterFast = runPath(s, DrawPath::Stock);
  const PathRun before = runPath(s, DrawPath::Stock);
  const PathRun before2 = runPath(s, DrawPath::Stock);
  sampleSelfVariance(scope, "lifecycle: Stock twice", before, before2, s.w);
  samePixels("after mapper release: Fast pixels == Stock", before.steadyPx, released.steadyPx, s.w);
  samePixels("lifecycle: a Stock frame right after Fast == Stock", before.steadyPx,
             afterFast.steadyPx, s.w);

  // The shader cache's programs released and recompiled in place (the cache
  // keeps the objects): the re-bound program is compiled again before use.
  auto *win = vtkOpenGLRenderWindow::SafeDownCast(s.sr->renderWindow());
  win->GetShaderCache()->ReleaseGraphicsResources(win);
  LowMemoryPolyDataMapper::setDrawPath(DrawPath::Fast);
  s.sr->render();
  const PathRun after = runPath(s, DrawPath::Fast);
  // Across the recompile: byte for byte, or on Apple's renderer the LSB
  // envelope, like every pixel check (checkPixels()).
  samePixels("programs released in the cache: Fast recompiles and matches Stock", before.steadyPx,
             after.steadyPx, s.w);

  // The same scene -- the same nodes and mappers -- closed and re-opened in a
  // new window (new context, new shader cache): the first draws resolve fully
  // and the frames match the old window's.
  s.sr.reset();
  s.open();
  s.sg.setShadowsEnabled(true); // the pass chain belongs to the renderer, not the scene
  s.sg.setShadowUpdateInterval(1000);
  s.resetCounters();
  s.sr->render();
  glcallsInstall(); // the new context re-loaded glad
  const auto st = s.stats();
  check(st.programLookups > 0 && st.programLookupsSkipped == 0,
        "re-opened in a new window: the first draws look their programs up",
        fmt("lookups %.0f, skipped %.0f", double(st.programLookups),
            double(st.programLookupsSkipped)));
  // The window's first Stock run follows that Fast frame: neither sampled nor
  // compared. The two after it are a self-variance pair -- in this window,
  // never with the old window's, which the cross-window checks compare.
  runPath(s, DrawPath::Stock);
  const PathRun a = runPath(s, DrawPath::Stock);
  const PathRun a2 = runPath(s, DrawPath::Stock);
  const PathRun b = runPath(s, DrawPath::Fast);
  sampleSelfVariance(scope, "new window: Stock twice", a, a2, s.w);
  samePixels("new window: Fast pixels == Stock", a.steadyPx, b.steadyPx, s.w);
  // Across the windows: byte for byte, or on Apple's renderer the LSB
  // envelope, like every pixel check (checkPixels()).
  samePixels("new window: the Stock frame == the old window's", before.steadyPx, a.steadyPx, s.w);
  samePixels("new window: the Fast frame == the old window's Stock frame", before.steadyPx,
             b.steadyPx, s.w);
}

// ── 7. guards ────────────────────────────────────────────────────────────────
void testGuards(cvc::app &app) {
  std::printf("guards\n");
  Scene s(app);
  // Coincident offsets under the global POLYGON_OFFSET mode, where VTK's
  // defaults give points -8 and lines -4 units but polygons 0:
  //   zeroPoly  polys only, polygon offset 0 -> stock's empty verts/lines agents
  //             leave -4 in the program for the polys draw; skipping them would
  //             not. Must fall back to all four.
  //   shifted   polys only, the same relative offset on every class -> the polys
  //             draw sets its own non-zero value. Safe to skip.
  Probe &zero = s.probe("zeroPoly", spherePoly(1.5));
  zero.actor->GetProperty()->SetColor(0.9, 0.4, 0.3);
  Probe &shifted = s.probe("shifted", ribbonPoly(12, -4, 3, 0.2));
  shifted.mapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(0.0, -3.0);
  shifted.mapper->SetRelativeCoincidentTopologyLineOffsetParameters(0.0, -3.0);
  shifted.mapper->SetRelativeCoincidentTopologyPointOffsetParameter(-3.0);
  Probe &vv = s.probe("vertexVis", terrainPoly(6, -5, -5, 4));
  vv.actor->GetProperty()->VertexVisibilityOn();
  vv.actor->GetProperty()->SetVertexColor(1, 0, 0);
  // The mode is set before the first render: vtkGLSLModCoincidentTopology
  // declares the offset uniforms only if the shader is BUILT under an offset,
  // and nothing rebuilds it when the global mode changes later -- switched on
  // after the build, the offsets never reach the GPU on any path.
  const int savedMode = vtkMapper::GetResolveCoincidentTopology();
  vtkMapper::SetResolveCoincidentTopologyToPolygonOffset();
  s.open();
  s.sr->setCamera(0, -14, 10, 0, 0, 0, 0, 0, 1, 40.0, 1.0, 100.0);
  s.sr->render();
  glcallsInstall();
  for (const Probe *p : {&zero, &shifted}) {
    vtkShaderProgram *prog = p->mapper->vtkDrawTexturedElements::GetShaderProgram();
    check(prog && prog->IsUniformUsed("cOffset") && prog->IsUniformUsed("cFactor"),
          "POLYGON_OFFSET mode: " + p->name + "'s program carries the depth offset uniforms");
  }

  // Stock last in the settle, so that neither sampled Stock run follows Fast.
  for (DrawPath p : {DrawPath::Fast, DrawPath::Stock})
    runPath(s, p);
  const PathRun stock = runPath(s, DrawPath::Stock);
  const PathRun all = runPath(s, DrawPath::AllCellTypes);
  const PathRun stock2 = runPath(s, DrawPath::Stock);
  const PathRun fast = runPath(s, DrawPath::Fast); // last: the mapper stats below are Fast's
  sampleSelfVariance("guards", "POLYGON_OFFSET mode: Stock twice", stock, stock2, s.w);
  samePixels("POLYGON_OFFSET mode: Fast pixels == Stock", stock.steadyPx, fast.steadyPx, s.w);
  checkTraces("POLYGON_OFFSET mode", stock.steadyTrace, all.steadyTrace, fast.steadyTrace,
              fast.steadyStats);
  const auto &zs = zero.mapper->stats();
  const auto &ss = shifted.mapper->stats();
  check(zs.coincidentFallbacks == zs.draws && zs.cellTypesSkipped == 0 && zs.draws > 0,
        "zero polygon offset under defaults that offset points/lines: all four agents run",
        fmt("fallbacks %.0f of %.0f draws", double(zs.coincidentFallbacks), double(zs.draws)));
  check(ss.coincidentFallbacks == 0 && ss.cellTypesSkipped == 3 * ss.draws && ss.draws > 0,
        "an offset the drawn type sets itself: empty types skipped",
        fmt("skipped %.0f over %.0f draws", double(ss.cellTypesSkipped), double(ss.draws)));
  vtkMapper::SetResolveCoincidentTopology(savedMode);

  const auto &vs = vv.mapper->stats();
  check(vs.stockDraws == vs.draws && vs.draws > 0, "vertex visibility draws the stock way");

  // Picking: the selector's passes draw the stock way, and select the same.
  comparePicking(s, "guards scene");
}

// ── 8. shader replacements after the first draw ─────────────────────────────
// VTK's low-memory mapper ignores its actor's shader property once the program
// is built; LowMemoryPolyDataMapper::RenderPieceStart rebuilds it. A plain
// GeometryNode (no streaming), every draw path.
void testLateShaderReplacement(cvc::app &app) {
  std::printf("shader replacement after the first draw\n");
  for (DrawPath p : {DrawPath::Stock, DrawPath::AllCellTypes, DrawPath::Fast}) {
    LowMemoryPolyDataMapper::setDrawPath(p);
    Scene s(app);
    auto n = s.node("plain");
    n->setGeometry(boxGeometry(0, 0, 0, 5, 5, 0.5));
    n->setUseSingleColor(true);
    n->setColor(1, 1, 1);
    s.w = 48;
    s.h = 48;
    s.open();
    s.sr->setCamera(0, 0, 50, 0, 0, 0, 0, 1, 0, 30.0, 1.0, 200.0);
    s.sr->render();
    s.sr->render();
    const auto centre = [&](const std::vector<unsigned char> &px) {
      const size_t i = 4 * (static_cast<size_t>(s.h / 2) * s.w + s.w / 2);
      return std::vector<unsigned char>(px.begin() + i, px.begin() + i + 3);
    };
    const auto corner = [](const std::vector<unsigned char> &px) {
      return std::vector<unsigned char>(px.begin(), px.begin() + 3);
    };
    const auto drawn = s.rgba();
    const bool visible = centre(drawn) != corner(drawn);

    n->addFragmentShaderReplacement("//VTK::UniformFlow::Impl",
                                    "//VTK::UniformFlow::Impl\n  discard;\n");
    s.resetCounters();
    s.sr->render();
    const auto discarded = s.rgba();
    const auto st = s.stats();
    check(visible && centre(discarded) == corner(discarded),
          std::string(pathName(p)) + ": a replacement added after the first draw reaches the GPU",
          fmt("lookups %.0f", double(st.programLookups + st.stockDraws)));

    // Byte for byte, or on Apple's renderer the LSB envelope, like every pixel
    // check (no renderer has been seen to vary on this unshadowed 48x48 scene).
    n->clearShaderReplacements();
    s.sr->render();
    const PixelDiff back = pixelDiff(drawn, s.rgba());
    check(pixelsMatch(back),
          std::string(pathName(p)) + ": cleared again, the original frame comes back",
          pixelVerdict(back, s.w));
  }
  LowMemoryPolyDataMapper::setDrawPath(DrawPath::Fast);
}

} // namespace

int main() {
#ifdef _WIN32
  _putenv_s("CVCGL_LOWMEM_MAPPER", "force");
  if (!std::getenv("__GL_SYNC_TO_VBLANK"))
    _putenv_s("__GL_SYNC_TO_VBLANK", "0");
  if (!std::getenv("vblank_mode"))
    _putenv_s("vblank_mode", "0");
#else
  setenv("CVCGL_LOWMEM_MAPPER", "force", 1);
  setenv("__GL_SYNC_TO_VBLANK", "0", 0); // NVIDIA throttles offscreen swaps otherwise
  setenv("vblank_mode", "0", 0);
#endif
  const char *rep = std::getenv("CVCGL_LOWMEM_REPORT");
  g_report = rep && *rep && std::string(rep) != "0";
  vtkOutputWindow::SetInstance(vtkSmartPointer<ErrorCounter>::New());

  testPolicy();
  cvc::app app;
  if (!LowMemoryPolyDataMapper::replicaActive()) {
    // The shader-property rebuild is not part of the replica: every VTK.
    std::printf("  replica off (VTK is not an unpatched 9.5.0): every path draws the stock way\n");
    if (renderAvailable(app))
      testLateShaderReplacement(app);
    std::printf("%s: cvcgl_lowmem_fastdraw (%d checks, %d failed)\n",
                g_failures == 0 ? "PASS" : "FAIL", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
  }

  LowMemoryPolyDataMapper::setStageObserver(&onStage);
  if (renderAvailable(app)) {
    // Shadows on and off in separate scenes: SceneGraph::setShadowsEnabled(false)
    // drops the shadow passes without releasing their GL resources, which VTK
    // reports as an error when they are destroyed.
    for (bool shadows : {true, false}) {
      Scene s(app);
      buildCity(s);
      addLights(s.sg);
      s.open();
      if (shadows) {
        check(s.sg.setShadowsEnabled(true), "shadows enabled");
        s.sg.setShadowUpdateInterval(1000); // bake only when invalidated
      }
      s.sr->render();
      glcallsInstall();
      check(s.mappers().size() == s.nodes.size() + s.probes.size(),
            "every node and probe draws through LowMemoryPolyDataMapper",
            fmt("%.0f of %.0f", double(s.mappers().size()),
                double(s.nodes.size() + s.probes.size())));
      const std::string label = shadows ? "shadows on" : "shadows off";
      comparePaths(s, label, shadows);
      if (shadows)
        testLifecycle(s, label);
    }
    testGuards(app);
    testLateShaderReplacement(app);
  }
  LowMemoryPolyDataMapper::setStageObserver(nullptr);
  checkPixels();
  check(ErrorCounter::errors() == 0, "no VTK errors",
        fmt("%.0f errors", double(ErrorCounter::errors())));
  std::printf("%s: cvcgl_lowmem_fastdraw (%d checks, %d failed)\n",
              g_failures == 0 ? "PASS" : "FAIL", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
