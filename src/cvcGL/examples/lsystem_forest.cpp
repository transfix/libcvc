// lsystem_forest — a pure-C++ cvcGL demo (port of scripts/examples/lsystem_forest.py
// from the volrover repo). Drives cvcGL directly: an island the eye can fly over,
// navigable with the built-in CameraController (orbit / Quake-fly / cinematic track).
//
// NO SINGLETON: owns an explicit cvc::app and injects it into the scene, so the
// whole thing runs under one app the caller controls (cvc::gl::context() unused).
//
// WIP — this increment builds the terrain (matte + a procedural fragment bump map,
// the same surface-gradient technique the tree bark uses), the afternoon sun and
// striped shadows, and the navigable camera. The L-system trees (route-C merge +
// procedural bark), the sea/sky volumes and the sun disc are ported next.
//
// Run (onscreen, navigable):   lsystem_forest
//   Tab toggles orbit/fly; WASD + mouse to fly; Esc releases the pointer.
// Verify (offscreen, headless): lsystem_forest --offscreen --frames 30 --png out.png

#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/volume/bounding_box.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using cvc::gl::CameraController;

namespace {

constexpr double HALF = 120.0; // terrain spans [-HALF, HALF]
constexpr int TN = 96;         // heightfield resolution
constexpr double PEAK = 34.0, SHELF = -9.0, SEA_LEVEL = 0.0;

// Island heightfield: a central dome that drops below sea level at the rim, plus
// two octaves of relief (matches the Python demo's base terrain before patches).
double terrainH(double x, double y) {
  double r2 = x * x + y * y;
  double h = PEAK * std::exp(-r2 / (0.34 * HALF * HALF)) + SHELF;
  h += 4.5 * std::sin(x * 0.045) * std::cos(y * 0.041);
  h += 2.2 * std::sin(x * 0.11 + 1.3) * std::sin(y * 0.097);
  return h;
}

cvc::geometry buildTerrain(cvc::app &app) {
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  const cvc::geometry::color_t rock = {0.46, 0.45, 0.43};
  const cvc::geometry::color_t grass = {0.27, 0.44, 0.19};
  const cvc::geometry::color_t sand = {0.68, 0.62, 0.44};
  for (int j = 0; j < TN; ++j) {
    double y = -HALF + 2.0 * HALF * j / (TN - 1);
    for (int i = 0; i < TN; ++i) {
      double x = -HALF + 2.0 * HALF * i / (TN - 1);
      double h = terrainH(x, y);
      pts.push_back({x, y, h});
      // shade by height: sand at the waterline, grass above, rock on the peaks
      cvc::geometry::color_t c = grass;
      double rockw = std::min(1.0, std::max(0.0, (h - 18.0) / 14.0));
      for (int k = 0; k < 3; ++k)
        c[k] = grass[k] * (1.0 - rockw) + rock[k] * rockw;
      double shore = std::max(0.0, 1.0 - std::fabs(h - SEA_LEVEL) / 4.5);
      for (int k = 0; k < 3; ++k)
        c[k] = c[k] * (1.0 - shore) + sand[k] * shore;
      cols.push_back(c);
    }
  }
  auto &tris = g.tris();
  for (int j = 0; j < TN - 1; ++j)
    for (int i = 0; i < TN - 1; ++i) {
      cvc::geometry::index_t v = j * TN + i;
      tris.push_back({v, v + 1, static_cast<cvc::geometry::index_t>(v + TN)});
      tris.push_back({static_cast<cvc::geometry::index_t>(v + 1),
                      static_cast<cvc::geometry::index_t>(v + TN + 1),
                      static_cast<cvc::geometry::index_t>(v + TN)});
    }
  return g;
}

// Procedural fragment bump map for the ground (verbatim GLSL from the Python demo:
// value-noise height, normal perturbed by its surface gradient — Mikkelsen's
// tangent-free method). Needs world-space vertexMC, hence disableCoordinateShiftScale.
const char *GROUND_GLSL =
    "float ghash(vec2 p){ return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }\n"
    "float gnoise(vec2 p){\n"
    "  vec2 i = floor(p), f = fract(p); f = f*f*(3.0-2.0*f);\n"
    "  float a=ghash(i), b=ghash(i+vec2(1.,0.)), c=ghash(i+vec2(0.,1.)), d=ghash(i+vec2(1.,1.));\n"
    "  return mix(mix(a,b,f.x), mix(c,d,f.x), f.y);\n"
    "}\n"
    "float groundH(vec3 p){\n"
    "  vec2 q = p.xy * 0.35;\n"
    "  float f = 0.0, a = 0.5, fr = 1.0;\n"
    "  for (int i = 0; i < 5; i++){ f += a*gnoise(q*fr); a *= 0.5; fr *= 2.03; }\n"
    "  return f;\n"
    "}\n";

void addTerrainBump(GeometryNode &node) {
  node.disableCoordinateShiftScale(); // vertexMC in the shader becomes world xy
  node.addVertexShaderReplacement("//VTK::Normal::Dec", "//VTK::Normal::Dec\nout vec3 gCoord;");
  node.addVertexShaderReplacement("//VTK::PositionVC::Impl",
                                  "//VTK::PositionVC::Impl\n  gCoord = vertexMC.xyz;");
  node.addFragmentShaderReplacement("//VTK::Normal::Dec",
                                    std::string("//VTK::Normal::Dec\nin vec3 gCoord;\n") +
                                        GROUND_GLSL);
  node.addFragmentShaderReplacement(
      "//VTK::Normal::Impl",
      "//VTK::Normal::Impl\n"
      "  {\n"
      "    float h = groundH(gCoord);\n"
      "    vec3 sS = dFdx(vertexVC.xyz);\n"
      "    vec3 sT = dFdy(vertexVC.xyz);\n"
      "    vec3 vn = normalVCVSOutput;\n"
      "    vec3 R1 = cross(sT, vn), R2 = cross(vn, sS);\n"
      "    float det = dot(sS, R1);\n"
      "    vec3 sg = sign(det) * (dFdx(h)*R1 + dFdy(h)*R2);\n"
      "    normalVCVSOutput = normalize(abs(det)*vn - 1.4*sg);\n"
      "  }\n");
}

} // namespace

int main(int argc, char **argv) {
  bool offscreen = false;
  int frames = 0;
  std::string png;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--offscreen"))
      offscreen = true;
    else if (!std::strcmp(argv[i], "--frames") && i + 1 < argc)
      frames = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--png") && i + 1 < argc)
      png = argv[++i];
  }

  // Own the app and inject it — no global/singleton context.
  cvc::app app;
  SceneGraph sg(app, "forest");

  sg.addGraphics("terrain", buildTerrain(app));
  auto terrain = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("terrain"));
  terrain->setUseSingleColor(false);
  addTerrainBump(*terrain);

  // Afternoon sun (warm) + a dim cool sky fill from the opposite side.
  sg.addDirectionalLight(-52.0, 34.0, 1.0, 0.94, 0.82, 1.0);
  sg.addDirectionalLight(128.0, 52.0, 0.55, 0.66, 0.85, 0.55);

  SceneRenderer view(sg, 1280, 800, offscreen, "main");
  view.setBackground(0.62, 0.76, 0.92);
  sg.setGridVisible(false); // the island is the scene — no lab grid/axis box
  sg.setAxisVisible(false);
  sg.processEvents();

  // Shadows on (a render target now exists); re-baked every 3rd frame (cheap).
  const bool shadows = sg.setShadowsEnabled(true);
  sg.setShadowUpdateInterval(3);

  // Built-in navigation, framed on the island.
  CameraController cam(view);
  cvc::bounding_box b = sg.computeGraphicsBounds();
  cam.frameBounds(b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz);

  std::printf("lsystem_forest: %s, terrain %dx%d, shadows %s. Tab=orbit/fly, WASD+mouse=fly.\n",
              offscreen ? "offscreen" : "onscreen", TN, TN, shadows ? "on" : "unavailable");

  auto start = std::chrono::steady_clock::now();
  double last = 0.0;
  int n = 0;
  while (!view.windowClosed()) {
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    double dt = t - last;
    last = t;
    view.processUIEvents();
    cam.update(dt);
    view.render();
    if (frames > 0 && ++n >= frames)
      break;
  }
  if (!png.empty())
    view.writePNG(png);
  cam.detach(); // stop receiving events before teardown
  return 0;
}
