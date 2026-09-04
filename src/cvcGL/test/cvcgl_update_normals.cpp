/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_update_normals — the in-place per-vertex normal fast path
// (GeometryNode::updateNormals, added for the FFT ocean in lsystem_coast). A
// lit quad is re-normal'd toward vs away from a directional light; the render
// must get brighter/darker accordingly, and a count-mismatch must no-op safely.
// Explicit checks + non-zero return (assert() is a NDEBUG no-op under Release).

#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <memory>
#include <vector>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

namespace {
int g_fail = 0;
void check(bool ok, const char *msg) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
  if (!ok)
    ++g_fail;
}

double meanLuma(SceneRenderer &sr) {
  sr.render();
  const std::vector<unsigned char> px = sr.frameRGB();
  if (px.empty())
    return 0.0;
  double s = 0;
  for (size_t i = 0; i + 2 < px.size(); i += 3)
    s += 0.299 * px[i] + 0.587 * px[i + 1] + 0.114 * px[i + 2];
  return s / (px.size() / 3);
}

cvc::geometry quad(cvc::app &app) {
  cvc::geometry g(app);
  auto &p = g.points();
  auto &c = g.colors();
  for (int j = 0; j < 2; ++j)
    for (int i = 0; i < 2; ++i) {
      p.push_back({-5.0 + 10.0 * i, -5.0 + 10.0 * j, 0.0});
      c.push_back({0.8, 0.8, 0.8});
    }
  auto &t = g.tris();
  t.push_back({0, 1, 2});
  t.push_back({1, 3, 2});
  return g;
}
} // namespace

int main() {
  cvc::app app;
  SceneGraph sg(app, "nrmtest");
  sg.addGraphics("q", quad(app));
  auto node = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("q"));
  if (!node) {
    std::printf("FAIL: quad is not a GeometryNode\n");
    return 1;
  }
  node->setUseSingleColor(false);
  node->setAmbient(0.05);
  node->setDiffuse(1.0);
  node->setSpecular(0.0);
  // Directional light from +X, low in the sky (az=90 -> +X per SceneGraph docs).
  sg.addDirectionalLight(90.0, 12.0, 1.0, 1.0, 1.0, 1.5);

  SceneRenderer sr(sg, 96, 96, /*offscreen=*/true);
  sr.setCamera(0, 0, 30, 0, 0, 0, 0, 1, 0, 40, 1, 100); // top-down onto the XY quad
  sr.render();

  const size_t N = 4; // the quad has 4 vertices
  std::vector<double> toward(N * 3, 0.0), away(N * 3, 0.0);
  for (size_t k = 0; k < N; ++k) {
    toward[k * 3 + 0] = 1.0; // +X, toward the light
    away[k * 3 + 0] = -1.0;  // -X, away from the light
  }

  node->updateNormals(toward);
  const double lToward = meanLuma(sr);
  node->updateNormals(away);
  const double lAway = meanLuma(sr);
  std::printf("  (luma toward=%.2f, away=%.2f)\n", lToward, lAway);
  check(lToward > lAway + 2.0, "normals toward the light render brighter than normals away");

  // Count mismatch: must log-and-no-op, not corrupt/crash — a later render works.
  node->updateNormals(std::vector<double>(9, 0.0)); // 3 normals for a 4-vertex mesh
  const double lAfter = meanLuma(sr);
  check(lAfter > 0.0, "mismatched updateNormals no-ops safely (scene still renders)");

  std::printf("%s: cvcgl_update_normals (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL",
              g_fail, g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
