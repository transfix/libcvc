/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Not a test: prints what cvc::gl::LowMemoryPolyDataMapper saves per frame and
// per draw, DrawPath::Stock against DrawPath::Fast, on desktop GL with the
// low-memory mapper forced (the mapper WebGL2 gets).
//
// Scene (a demo-style frame): a textured ground, a merged city mesh of N boxes,
// V single-colour vehicle nodes, translucent route ribbons with a depth offset,
// line trails, and optionally many extra small single-colour nodes; shadows on
// (camera -> strided shadow baker -> shadow map -> translucent -> volumetric ->
// overlay) and off. Each node's mapper is swapped for one that tallies the GL
// calls or the CPU time of its own draws, so "per draw" is exactly
// RenderPieceDraw. Frames are rendered without glFinish; times are CPU.
//
// Every configuration runs twice per path: a TIMED pass with VTK's own GL
// function pointers (no counting wrapper on any call; each mapper draw only
// reads the clock) and a COUNTED pass with the wrappers installed, which also
// track uniform redundancy per (program, location) -- that costs CPU in
// proportion to the calls, so its times are not reported. Times come from the
// timed pass, call counts from the counted one.
//
//   cvcgl_lowmem_bench [--frames=40] [--boxes=20000] [--vehicles=6] [--extra=0]
//                      [--size=960x540]
#include "gl_call_counter.h"

#include <algorithm>
#include <chrono>
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
#include <memory>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkUnsignedCharArray.h>

using cvc::gl::GeometryNode;
using cvc::gl::GeometryRenderMode;
using cvc::gl::LowMemoryPolyDataMapper;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvcgl_test::GLCalls;
using cvcgl_test::glcallsRead;
using DrawPath = LowMemoryPolyDataMapper::DrawPath;
using clk = std::chrono::steady_clock;

namespace {

double msSince(clk::time_point t0) {
  return std::chrono::duration<double, std::milli>(clk::now() - t0).count();
}

// Tallies the GL calls (counted pass) or the CPU time (timed pass) of every
// draw of every TallyMapper.
bool g_counting = false;
GLCalls g_drawCalls;
double g_drawMs = 0, g_draws = 0;

class TallyMapper : public LowMemoryPolyDataMapper {
public:
  static TallyMapper *New();
  vtkTypeMacro(TallyMapper, LowMemoryPolyDataMapper);
  void RenderPieceDraw(vtkRenderer *ren, vtkActor *act) override {
    g_draws += 1;
    if (g_counting) {
      const GLCalls before = glcallsRead();
      Superclass::RenderPieceDraw(ren, act);
      g_drawCalls += glcallsRead() - before;
      return;
    }
    const auto t0 = clk::now();
    Superclass::RenderPieceDraw(ren, act);
    g_drawMs += msSince(t0);
  }

protected:
  TallyMapper() = default;
};
vtkStandardNewMacro(TallyMapper);

class PeekNode : public GeometryNode {
public:
  using GeometryNode::GeometryNode;
  vtkActor *actor() { return vtkActor::SafeDownCast(getProp()); }
};

// Replace a node's mapper with a TallyMapper carrying the same settings
// (GeometryNode configures nothing else on it that these nodes use).
void swapInTally(PeekNode &n) {
  vtkActor *a = n.actor();
  auto *old = LowMemoryPolyDataMapper::SafeDownCast(a->GetMapper());
  if (!old)
    return;
  vtkNew<TallyMapper> m;
  m->SetInputData(vtkPolyData::SafeDownCast(old->GetInput()));
  m->SetScalarVisibility(old->GetScalarVisibility());
  m->SetScalarMode(old->GetScalarMode());
  m->SetColorMode(old->GetColorMode());
  m->SetVBOShiftScaleMethod(old->GetVBOShiftScaleMethod());
  double f = 0, u = 0;
  old->GetRelativeCoincidentTopologyPolygonOffsetParameters(f, u);
  m->SetRelativeCoincidentTopologyPolygonOffsetParameters(f, u);
  old->GetRelativeCoincidentTopologyLineOffsetParameters(f, u);
  m->SetRelativeCoincidentTopologyLineOffsetParameters(f, u);
  old->GetRelativeCoincidentTopologyPointOffsetParameter(u);
  m->SetRelativeCoincidentTopologyPointOffsetParameter(u);
  a->SetMapper(m);
}

void addBox(cvc::geometry &g, double cx, double cy, double cz, double sx, double sy, double sz) {
  const double x0 = cx - sx, x1 = cx + sx, y0 = cy - sy, y1 = cy + sy, z0 = cz - sz, z1 = cz + sz;
  const double faces[5][4][3] = {{{x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}},
                                 {{x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}},
                                 {{x1, y1, z0}, {x0, y1, z0}, {x0, y1, z1}, {x1, y1, z1}},
                                 {{x0, y1, z0}, {x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}},
                                 {{x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {x1, y0, z1}}};
  for (const auto &f : faces) {
    const unsigned b = static_cast<unsigned>(g.points().size());
    for (const auto &p : f)
      g.points().push_back({p[0], p[1], p[2]});
    g.tris().push_back({b, b + 1, b + 2});
    g.tris().push_back({b, b + 2, b + 3});
  }
}

struct Options {
  int frames = 40, boxes = 20000, vehicles = 6, extra = 0, w = 960, h = 540;
};

struct Bench {
  explicit Bench(cvc::app &app) : sg(app, "lowmem_bench") { sg.setDiagnosticChromeVisible(false); }
  SceneGraph sg;
  std::unique_ptr<SceneRenderer> sr;
  std::vector<std::shared_ptr<PeekNode>> nodes, vehicles;

  std::shared_ptr<PeekNode> node(const std::string &name) {
    auto n = sg.getGraphicsRoot()->addGraphicsChild<PeekNode>(name);
    nodes.push_back(n);
    return n;
  }
};

void build(Bench &b, const Options &o) {
  const double half = 600.0;
  {
    auto ground = b.node("ground");
    cvc::geometry g;
    const int n = 64;
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const double x = -half + 2 * half * i / (n - 1), y = -half + 2 * half * j / (n - 1);
        g.points().push_back({x, y, 3.0 * std::sin(x / 90) * std::cos(y / 70)});
        g.uvs().push_back({double(i) / (n - 1), double(j) / (n - 1)});
      }
    for (int j = 0; j + 1 < n; ++j)
      for (int i = 0; i + 1 < n; ++i) {
        const unsigned a = j * n + i, c = a + n;
        g.tris().push_back({a, a + 1, c + 1});
        g.tris().push_back({a, c + 1, c});
      }
    ground->setGeometry(g);
    ground->setUseSingleColor(true);
    ground->setColor(1, 1, 1);
    ground->setAmbient(0.5);
    ground->setDiffuse(0.6);
    cvc::image img(512, 512, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
    unsigned char *p = img.data();
    for (int j = 0; j < 512; ++j)
      for (int i = 0; i < 512; ++i) {
        unsigned char *q = p + 4 * (j * 512 + i);
        q[0] = static_cast<unsigned char>(i / 2);
        q[1] = static_cast<unsigned char>(j / 2);
        q[2] = 128;
        q[3] = 255;
      }
    ground->setTexture(img);
  }
  if (o.boxes > 0) {
    auto city = b.node("city");
    cvc::geometry g;
    const int side = static_cast<int>(std::ceil(std::sqrt(double(o.boxes))));
    for (int k = 0; k < o.boxes; ++k) {
      const double x = -half * 0.9 + 1.8 * half * (k % side) / side;
      const double y = -half * 0.9 + 1.8 * half * (k / side) / side;
      const double hgt = 2.0 + 10.0 * (0.5 + 0.5 * std::sin(k * 0.37));
      addBox(g, x, y, hgt, 2.5, 2.5, hgt);
    }
    city->setGeometry(g);
    city->setUseSingleColor(true);
    city->setColor(0.72, 0.70, 0.66);
  }
  for (int v = 0; v < o.vehicles; ++v) {
    auto veh = b.node("veh" + std::to_string(v));
    cvc::geometry g;
    addBox(g, 0, 0, 2.0, 4.0, 2.0, 1.5);
    veh->setGeometry(g);
    veh->setUseSingleColor(true);
    veh->setColor(0.2 + 0.1 * v, 0.5, 0.9 - 0.1 * v);
    b.vehicles.push_back(veh);
  }
  for (int r = 0; r < 4; ++r) {
    auto rib = b.node("route" + std::to_string(r));
    cvc::geometry g;
    for (int k = 0; k < 120; ++k) {
      const double x = -400 + 7.0 * k, y = -200 + 120.0 * r + 25 * std::sin(k * 0.1);
      g.points().push_back({x, y - 2.0, 4.0});
      g.points().push_back({x, y + 2.0, 4.0});
    }
    for (unsigned k = 0; k + 1 < 120; ++k) {
      g.tris().push_back({2 * k, 2 * k + 1, 2 * k + 3});
      g.tris().push_back({2 * k, 2 * k + 3, 2 * k + 2});
    }
    rib->setGeometry(g);
    rib->setUseSingleColor(true);
    rib->setColor(0.98, 0.8, 0.25);
    rib->setOpacity(0.55);
    rib->setDepthOffset(2.0);

    auto trail = b.node("trail" + std::to_string(r));
    cvc::geometry t;
    for (int k = 0; k < 200; ++k) {
      t.points().push_back({-450 + 4.0 * k, -150 + 110.0 * r + 15 * std::cos(k * 0.07), 5.0});
      if (k)
        t.lines().push_back({static_cast<unsigned>(k - 1), static_cast<unsigned>(k)});
    }
    trail->setGeometry(t);
    trail->setRenderMode(GeometryRenderMode::LINES); // after setGeometry, which picks a mode
    trail->setUseSingleColor(true);
    trail->setColor(0.3, 1.0, 0.4);
  }
  for (int e = 0; e < o.extra; ++e) {
    auto n = b.node("extra" + std::to_string(e));
    cvc::geometry g;
    addBox(g, -500 + 1000.0 * (e % 20) / 20, -500 + 1000.0 * (e / 20) / 20, 3, 3, 3, 3);
    n->setGeometry(g);
    n->setUseSingleColor(true);
    n->setColor(0.9, 0.4, 0.3);
  }
  b.sg.addSpotLight(-500, -700, 900, 0, 0, 0, 45.0, 1.0, 0.97, 0.9, 1.1);
  b.sg.addFillLight(600, 500, 600, 0, 0, 0, 0.8, 0.85, 1.0, 0.4);
}

void poseVehicles(Bench &b, int frame) {
  for (size_t v = 0; v < b.vehicles.size(); ++v) {
    const double t = 0.02 * frame + 1.1 * v, c = std::cos(t), s = std::sin(t);
    const double m[16] = {
        c, -s, 0, 300 * std::cos(0.3 * t + v), s, c, 0, 250 * std::sin(0.4 * t), 0, 0, 1, 0, 0,
        0, 0,  1};
    b.vehicles[v]->setPoseMatrix(m);
  }
}

// One pass: timed (frameMs, drawMs) or counted (frame, draw).
struct Run {
  double frames = 0, frameMs = 0, drawMs = 0, draws = 0;
  GLCalls frame, draw;
  LowMemoryPolyDataMapper::Stats stats;
  Run &operator+=(const Run &o) {
    frames += o.frames;
    frameMs += o.frameMs;
    drawMs += o.drawMs;
    draws += o.draws;
    frame += o.frame;
    draw += o.draw;
    stats.programLookups += o.stats.programLookups;
    stats.programLookupsSkipped += o.stats.programLookupsSkipped;
    stats.stockDraws += o.stats.stockDraws;
    return *this;
  }
};

LowMemoryPolyDataMapper::Stats sumStats(Bench &b) {
  LowMemoryPolyDataMapper::Stats s;
  for (auto &n : b.nodes)
    if (auto *m = LowMemoryPolyDataMapper::SafeDownCast(n->actor()->GetMapper())) {
      const auto &t = m->stats();
      s.draws += t.draws;
      s.cellTypesSkipped += t.cellTypesSkipped;
      s.programLookups += t.programLookups;
      s.programLookupsSkipped += t.programLookupsSkipped;
      s.stockDraws += t.stockDraws;
    }
  return s;
}

Run measure(Bench &b, DrawPath path, bool bake, int frames, int &frameNo, bool counted) {
  LowMemoryPolyDataMapper::setDrawPath(path);
  if (counted)
    cvcgl_test::glcallsInstall();
  else
    cvcgl_test::glcallsUninstall(); // VTK's own pointers: no wrapper cost in the timing
  g_counting = counted;
  if (counted) {
    // One frame on this path first: the first frame after a path switch differs
    // by a call or two of vtkOpenGLState-cached state (glPointSize).
    poseVehicles(b, frameNo++);
    if (bake)
      b.sg.invalidateShadowBake();
    b.sr->render();
  }
  Run r;
  for (auto &n : b.nodes)
    if (auto *m = LowMemoryPolyDataMapper::SafeDownCast(n->actor()->GetMapper()))
      m->resetStats();
  for (int f = 0; f < frames; ++f) {
    poseVehicles(b, frameNo++);
    if (bake)
      b.sg.invalidateShadowBake();
    g_drawCalls = GLCalls();
    g_drawMs = 0;
    g_draws = 0;
    if (counted) {
      const GLCalls before = glcallsRead();
      b.sr->render();
      r.frame += glcallsRead() - before;
      r.draw += g_drawCalls;
    } else {
      const auto t0 = clk::now();
      b.sr->render();
      r.frameMs += msSince(t0);
      r.drawMs += g_drawMs;
    }
    r.draws += g_draws;
    r.frames += 1;
  }
  r.stats = sumStats(b);
  g_counting = false;
  return r;
}

std::vector<unsigned char> rgba(Bench &b) {
  vtkNew<vtkUnsignedCharArray> px;
  const int *sz = b.sr->renderWindow()->GetSize();
  b.sr->renderWindow()->GetRGBACharPixelData(0, 0, sz[0] - 1, sz[1] - 1, 0, px);
  const unsigned char *p = px->GetPointer(0);
  return std::vector<unsigned char>(p, p + px->GetNumberOfValues());
}

// st/ft: the timed passes; sc/fc: the counted passes (Stock / Fast).
void report(const char *label, const Run &st, const Run &ft, const Run &sc, const Run &fc) {
  const double n = st.frames, nc = sc.frames;
  std::printf("\n== %s (%g timed + %g counted frames per path)\n", label, n, nc);
  std::printf("                           Stock       Fast\n");
  std::printf("  draws/frame           %8.1f   %8.1f\n", st.draws / n, ft.draws / n);
  std::printf("  GL calls/frame        %8.1f   %8.1f   (%+.1f, %+.0f%%)\n", sc.frame.total() / nc,
              fc.frame.total() / nc, (fc.frame.total() - sc.frame.total()) / nc,
              100.0 * (fc.frame.total() - sc.frame.total()) / sc.frame.total());
  std::printf("    in mapper draws     %8.1f   %8.1f\n", sc.draw.total() / nc,
              fc.draw.total() / nc);
  std::printf("    uniforms            %8.1f   %8.1f   (redundant %.1f -> %.1f)\n",
              sc.frame.uniforms() / nc, fc.frame.uniforms() / nc, sc.frame.uniformRedundant / nc,
              fc.frame.uniformRedundant / nc);
  std::printf("    desktop round-trips (upper bound) %8.1f   %8.1f\n", sc.frame.roundTrips() / nc,
              fc.frame.roundTrips() / nc);
  std::printf("  GL calls/draw         %8.1f   %8.1f\n", sc.draw.total() / sc.draws,
              fc.draw.total() / fc.draws);
  std::printf("  timed, no GL wrappers:\n");
  std::printf("  draw-stage CPU ms/frame %6.3f   %8.3f\n", st.drawMs / n, ft.drawMs / n);
  std::printf("  draw-stage CPU ms/draw  %6.4f   %8.4f\n", st.drawMs / st.draws,
              ft.drawMs / ft.draws);
  std::printf("  frame CPU ms (no finish)%6.2f   %8.2f\n", st.frameMs / n, ft.frameMs / n);
  std::printf("  lookups/frame         %8.1f   %8.1f   (skipped %.1f)\n",
              double(st.stats.programLookups + st.stats.stockDraws) / n,
              double(ft.stats.programLookups) / n, double(ft.stats.programLookupsSkipped) / n);
  std::printf("  Stock per frame: %s\n", sc.frame.str(nc).c_str());
  std::printf("  Fast  per frame: %s\n", fc.frame.str(nc).c_str());
  std::printf("  Stock per draw : %s\n", sc.draw.str(sc.draws).c_str());
  std::printf("  Fast  per draw : %s\n", fc.draw.str(fc.draws).c_str());
}

} // namespace

int main(int argc, char **argv) {
#ifdef _WIN32
  _putenv_s("CVCGL_LOWMEM_MAPPER", "force");
#else
  setenv("CVCGL_LOWMEM_MAPPER", "force", 1);
  setenv("__GL_SYNC_TO_VBLANK", "0", 0); // NVIDIA throttles offscreen swaps otherwise
  setenv("vblank_mode", "0", 0);
#endif
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&](const char *k) {
      return a.rfind(k, 0) == 0 ? a.c_str() + std::strlen(k) : nullptr;
    };
    if (const char *v = val("--frames="))
      o.frames = std::atoi(v);
    else if (const char *v = val("--boxes="))
      o.boxes = std::atoi(v);
    else if (const char *v = val("--vehicles="))
      o.vehicles = std::atoi(v);
    else if (const char *v = val("--extra="))
      o.extra = std::atoi(v);
    else if (const char *v = val("--size="))
      std::sscanf(v, "%dx%d", &o.w, &o.h);
  }
  if (!LowMemoryPolyDataMapper::replicaActive()) {
    std::printf("this VTK is not 9.5.0: LowMemoryPolyDataMapper draws the stock way\n");
    return 0;
  }
  cvc::app app;
  Bench b(app);
  build(b, o);
  b.sr = std::make_unique<SceneRenderer>(b.sg, o.w, o.h, /*offscreen=*/true, "bench");
  // No MSAA: with VTK's default 8x, NVIDIA's resolve varies by 1 LSB at a few edge
  // pixels between identical frames, which would hide the Stock/Fast comparison.
  b.sr->renderWindow()->SetMultiSamples(0);
  b.sr->setBackground(0.2, 0.26, 0.36);
  b.sr->setCamera(-700, -900, 650, 0, 0, 0, 0, 0, 1, 42.0, 10.0, 6000.0);
  for (auto &n : b.nodes)
    swapInTally(*n);
  b.sg.setShadowsEnabled(true);
  b.sg.setShadowResolution(2048);
  b.sg.setShadowUpdateInterval(100000); // bake only when invalidated
  b.sr->render();
  b.sr->render();
  cvcgl_test::glcallsInstall();
  std::printf("lowmem bench: %zu nodes, %d boxes merged, %d vehicles, %d extra nodes, %dx%d\n",
              b.nodes.size(), o.boxes, o.vehicles, o.extra, o.w, o.h);

  int frameNo = 0;
  for (int shadows = 1; shadows >= 0; --shadows) {
    if (!shadows)
      b.sg.setShadowsEnabled(false);
    for (DrawPath p : {DrawPath::Stock, DrawPath::Fast}) // warm both paths
      measure(b, p, shadows != 0, 2, frameNo, false);
    const auto kinds = shadows ? std::vector<bool>{true, false} : std::vector<bool>{false};
    for (bool bake : kinds) {
      // Timed: interleave the two paths in halves so drift hits both alike.
      Run st, ft, sc, fc;
      for (int half = 0; half < 2; ++half) {
        st += measure(b, DrawPath::Stock, bake, o.frames / 2, frameNo, false);
        ft += measure(b, DrawPath::Fast, bake, o.frames / 2, frameNo, false);
      }
      // Counted: GL call counts are the same every frame of a kind; a few do.
      const int countedFrames = std::max(2, o.frames / 8);
      sc = measure(b, DrawPath::Stock, bake, countedFrames, frameNo, true);
      fc = measure(b, DrawPath::Fast, bake, countedFrames, frameNo, true);
      cvcgl_test::glcallsUninstall();
      report(shadows ? (bake ? "shadows ON, bake every frame" : "shadows ON, steady (no bake)")
                     : "shadows OFF",
             st, ft, sc, fc);
      // Same pose on both paths, then compare the frames.
      LowMemoryPolyDataMapper::setDrawPath(DrawPath::Stock);
      poseVehicles(b, 7);
      if (bake)
        b.sg.invalidateShadowBake();
      b.sr->render();
      const auto a = rgba(b);
      LowMemoryPolyDataMapper::setDrawPath(DrawPath::Fast);
      if (bake)
        b.sg.invalidateShadowBake();
      b.sr->render();
      const auto c = rgba(b);
      long d = 0;
      for (size_t i = 0; i < a.size() && i < c.size(); ++i)
        d += a[i] != c[i];
      std::printf("  pixels Stock vs Fast: %s (%ld differing bytes of %zu)\n",
                  d == 0 && a.size() == c.size() ? "IDENTICAL" : "DIFFER", d, a.size());
    }
  }
  return 0;
}
