/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Shared harness for the cvcgl_streaming_* tests (and cvcgl_clip_children):
// explicit checks (assert() is a no-op under Release), VTK error capture, a
// can-this-build-rasterise probe that honours CVC_REQUIRE_RENDER, and pixel
// helpers. Synthetic data only.
#ifndef CVCGL_TEST_STREAMING_TEST_UTIL_H
#define CVCGL_TEST_STREAMING_TEST_UTIL_H

#include "gl_upload_counter.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StreamingGeometryNode.h>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOutputWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

namespace cvcgl_test {

inline int &failures() {
  static int n = 0;
  return n;
}
inline int &checks() {
  static int n = 0;
  return n;
}

inline bool check(bool ok, const std::string &what, const std::string &detail = "") {
  ++checks();
  if (!ok)
    ++failures();
  std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(), detail.empty() ? "" : " -- ",
              detail.c_str());
  std::fflush(stdout);
  return ok;
}

// Counts VTK errors so a test can require a clean run (shader compile errors
// arrive here, not as exceptions).
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
inline ErrorCounter *ErrorCounter::New() { VTK_STANDARD_NEW_BODY(ErrorCounter); }

inline void installErrorCounter() {
  vtkSmartPointer<ErrorCounter> out = vtkSmartPointer<ErrorCounter>::New();
  vtkOutputWindow::SetInstance(out);
}

inline const char *kindName(cvc::gl::StreamingMapperKind k) {
  switch (k) {
  case cvc::gl::StreamingMapperKind::Classic:
    return "classic";
  case cvc::gl::StreamingMapperKind::LowMemory:
    return "lowmem";
  default:
    return "auto";
  }
}

// Flat, unlit material so a colour reads back exactly.
inline void flat(cvc::gl::GeometryNode &n, double r, double g, double b) {
  n.setColor(r, g, b);
  n.setAmbient(1.0);
  n.setDiffuse(0.0);
  n.setSpecular(0.0);
}

// A top-down orthographic view of the square [cx - s, cx + s] x [cy - s, cy + s].
inline void topView(cvc::gl::SceneRenderer &sr, double cx, double cy, double s) {
  sr.setCamera(cx, cy, 500, cx, cy, 0, 0, 1, 0, 30.0, 1.0, 2000.0);
  vtkCamera *cam = sr.renderer()->GetActiveCamera();
  cam->ParallelProjectionOn();
  cam->SetParallelScale(s);
}

inline std::array<double, 2> toPx(cvc::gl::SceneRenderer &sr, double x, double y, double z) {
  vtkRenderer *ren = sr.renderer();
  ren->SetWorldPoint(x, y, z, 1.0);
  ren->WorldToDisplay();
  double d[3];
  ren->GetDisplayPoint(d);
  return {d[0], d[1]};
}

struct Frame {
  int w = 0, h = 0;
  std::vector<unsigned char> rgb; // bottom-up, like VTK display coordinates
  const unsigned char *at(int x, int y) const {
    x = std::clamp(x, 0, w - 1);
    y = std::clamp(y, 0, h - 1);
    return &rgb[3 * (static_cast<size_t>(y) * w + x)];
  }
};

inline Frame grab(cvc::gl::SceneRenderer &sr) {
  Frame f;
  f.rgb = sr.frameRGB(); // renders
  f.w = sr.frameWidth();
  f.h = sr.frameHeight();
  return f;
}

// Channel ch (0 r, 1 g, 2 b) clearly dominant.
inline bool isHue(const unsigned char *p, int ch) {
  const int o1 = (ch + 1) % 3, o2 = (ch + 2) % 3;
  return p[ch] > 80 && p[ch] > p[o1] + 30 && p[ch] > p[o2] + 30;
}
inline bool isBg(const unsigned char *p) { return p[0] < 40 && p[1] < 40 && p[2] < 40; }

inline bool hueNear(const Frame &f, std::array<double, 2> px, int r, int ch) {
  for (int dy = -r; dy <= r; ++dy)
    for (int dx = -r; dx <= r; ++dx)
      if (isHue(f.at(int(px[0]) + dx, int(px[1]) + dy), ch))
        return true;
  return false;
}
inline bool bgNear(const Frame &f, std::array<double, 2> px, int r) {
  for (int dy = -r; dy <= r; ++dy)
    for (int dx = -r; dx <= r; ++dx)
      if (!isBg(f.at(int(px[0]) + dx, int(px[1]) + dy)))
        return false;
  return true;
}
inline long litPixels(const Frame &f) {
  long n = 0;
  for (size_t i = 0; i + 2 < f.rgb.size(); i += 3)
    n += isBg(&f.rgb[i]) ? 0 : 1;
  return n;
}

// Pixels that differ by more than `tol` in any channel.
inline long diffPixels(const Frame &a, const Frame &b, int tol = 24) {
  if (a.rgb.size() != b.rgb.size())
    return -1;
  long n = 0;
  for (size_t i = 0; i + 2 < a.rgb.size(); i += 3) {
    bool d = false;
    for (int c = 0; c < 3; ++c)
      d = d || std::abs(int(a.rgb[i + c]) - int(b.rgb[i + c])) > tol;
    n += d ? 1 : 0;
  }
  return n;
}

// Can this build rasterise at all? Asked of a plain GeometryNode CONTROL in a
// scene of its own -- never of the streaming nodes under test, so a regression
// that leaves them dark cannot pass as "no GL here".
inline bool canRasterise(cvc::app &app) {
  cvc::gl::SceneGraph sg(app, "stream_control");
  sg.setDiagnosticChromeVisible(false);
  cvc::geometry g(app);
  g.points().push_back({-5.0, -5.0, 0.0});
  g.points().push_back({5.0, -5.0, 0.0});
  g.points().push_back({0.0, 5.0, 0.0});
  g.tris().push_back({0, 1, 2});
  auto node = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::GeometryNode>("control");
  node->setGeometry(g);
  flat(*node, 1.0, 1.0, 1.0);
  cvc::gl::SceneRenderer sr(sg, 48, 48, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, 0, 0, 8);
  return litPixels(grab(sr)) > 0;
}

// Skip (rc 0) when nothing rasterises, unless CVC_REQUIRE_RENDER=1 (the
// software-GL CI job) makes that a failure.
inline bool renderAvailable(cvc::app &app) {
  if (canRasterise(app))
    return true;
  const char *require = std::getenv("CVC_REQUIRE_RENDER");
  if (require && *require && std::string(require) != "0") {
    check(false, "CVC_REQUIRE_RENDER is set, but this build did not rasterise");
  } else {
    std::printf("  skipped: this build did not rasterise (the plain control drew nothing)\n");
  }
  return false;
}

inline void printRenderer(cvc::gl::SceneRenderer &sr) {
  auto *win = vtkOpenGLRenderWindow::SafeDownCast(sr.renderWindow());
  if (!win)
    return;
  const std::string caps = win->ReportCapabilities();
  for (const char *key : {"OpenGL renderer string:", "OpenGL version string:"}) {
    const auto p = caps.find(key);
    if (p != std::string::npos)
      std::printf("  %s\n", caps.substr(p, caps.find('\n', p) - p).c_str());
  }
}

inline int finish(const char *name) {
  std::printf("%s: %s (%d checks, %d failed)\n", failures() == 0 ? "PASS" : "FAIL", name, checks(),
              failures());
  return failures() == 0 ? 0 : 1;
}

} // namespace cvcgl_test

#endif
