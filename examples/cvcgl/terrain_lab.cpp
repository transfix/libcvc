// terrain_lab — a pure-C++ cvcGL demo that RENDERS what cvc::world produces.
//
// The forest/coast demos hand-author their terrain (a fixed island + a hardcoded
// tree grammar). Terrain Lab instead drives the real headless generator,
// cvc::world::generate_world (inc/cvc/world/generated_world.h): world_model::generate's
// heightfield + surface registry + scattered L-system props, cvc::world::raster()'s
// occupancy/material/height grid (what downstream training + gym consumers see), and the
// roads-first city. This demo turns that output into a scene so we can SEE and TUNE it: the
// terrain mesh is coloured by the raster's per-cell MATERIAL (or risk / occupancy / height), the
// props are grown from the built-in L-system recipes, water fills every cell that sits below the
// water level (streams, ponds, coast), and a drifting cloud slab sits overhead — all navigable
// with the built-in CameraController.
//
// The ImGui panel re-generates the world live: seed, per-species counts, tree
// generations, relief amplitude, sea level, ontology and a preset — so you can tune
// the generator and watch the scene rebuild. This is the visual QA surface for the
// L-system terrain source (docs/roadmap/LSYSTEM-LABORATORY-ROADMAP.md). The same world as an
// Ariadne document: examples/ariadne/documents/terrain_lab.ari (arigl), on the generated_world
// scene node type.
//
// Run (onscreen, navigable):   terrain_lab
//   Tab toggles orbit/fly; WASD + mouse to fly; Esc releases the pointer.
// Verify (offscreen, headless): terrain_lab --offscreen --frames 30 --png out.png
// Cinematic capture:            terrain_lab --capture fly --frames 900 --out frames

#define _USE_MATH_DEFINES
#include "demos_menu.h" // the gallery's Demos menu

#include <algorithm>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h> // the sea and cloud animate on the app's world clock
#include <cvc/core/world_clock_binding.h>
#include <cvc/core/world_units.h> // realistic world dimensions (metres/km) — §space units
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/FpsHud.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ImGuiBinding.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/gl/TouchGestures.h>
#include <cvc/state/state.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/image/image.h>
#ifdef __EMSCRIPTEN__
#include <cvc/gl/state_publisher.h>
#include <emscripten.h>
#endif
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <cvc/world/building_archetypes.h>
#include <cvc/world/generated_world.h>
#include <deque>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <vtkRenderer.h>

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::GeometryRenderMode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::VolumeNode;
namespace world = cvc::world;
using idx_t = cvc::geometry::index_t;

namespace {

// ── small vector maths (Z-up, metres) ───────────────────────────────────────
struct Vec3d {
  double x = 0, y = 0, z = 0;
};
Vec3d operator+(Vec3d a, Vec3d b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3d operator-(Vec3d a, Vec3d b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3d operator*(Vec3d a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double vlen(Vec3d a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
Vec3d vnorm(Vec3d a) {
  double l = vlen(a);
  return l > 1e-12 ? Vec3d{a.x / l, a.y / l, a.z / l} : Vec3d{0, 0, 1};
}
Vec3d vcross(Vec3d a, Vec3d b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// ── the world scale ──────────────────────────────────────────────────────────
double gHalf = 600.0; // world spans [-gHalf, gHalf]^2 (metres) — set live from the size knob
// The world's unit scale: one world unit == one metre. Realistic, centred-on-origin coordinates;
// used to label spans in the natural regime (metres, promoting to km past 1000 m).
cvc::world_units gUnits{cvc::world_units::config{cvc::world_units::system::si, 1.0}};
const char *const kPresetNames[] = {"standard", "forest", "urban", "sparse", "island"};
const char *const kOntologyNames[] = {"merged_default", "soft_vegetation", "strict_water_mud"};
const char *const kColorModeNames[] = {"material", "risk", "occupancy", "height"};
constexpr int TEX = 2048; // the terrain surface texture (material + roads + shoreline)

// The Austin building archetype library, loaded once. Discovery: $CVC_NAV_BUNDLE (a buildings.glb
// file, or a directory / scene dir containing one), else a buildings.glb beside the working dir.
// Empty without one: the city is then box massing.
const world::building_library &cityLibrary() {
  static const world::building_library lib = [] {
    std::vector<std::string> paths;
    if (const char *env = std::getenv("CVC_NAV_BUNDLE"); env && *env) {
      const std::string e(env);
      paths = {e, e + "/buildings.glb", e + "/austin_south/buildings.glb"};
    }
    paths.push_back("/bundle/buildings.glb"); // emscripten --preload-file mount (wasm)
    paths.push_back("buildings.glb");
    for (const std::string &p : paths) {
      world::building_library L = world::load_building_library(p);
      if (!L.empty()) {
        std::fprintf(stderr, "terrain_lab: %zu building archetypes from %s\n", L.size(), p.c_str());
        return L;
      }
    }
    std::fprintf(stderr,
                 "terrain_lab: no Austin building bundle (set CVC_NAV_BUNDLE) — box massing\n");
    return world::building_library{};
  }();
  return lib;
}

// Procedural fragment bump map for the ground (verbatim GLSL from lsystem_forest / the Python demo:
// fBm value-noise height, normal perturbed by its surface gradient — Mikkelsen's tangent-free
// method). Adds fine surface relief in the lighting without more triangles. Needs world-space
// vertexMC, hence disableCoordinateShiftScale.
#ifdef __EMSCRIPTEN__
#define CVC_FS_NORMAL "normalizedNormalVCVSOutput"
#else
#define CVC_FS_NORMAL "normalVCVSOutput"
#endif
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
  node.addFragmentShaderReplacement(
      "//VTK::Normal::Dec", std::string("//VTK::Normal::Dec\nin vec3 gCoord;\n") + GROUND_GLSL);
  node.addFragmentShaderReplacement("//VTK::Normal::Impl",
                                    "//VTK::Normal::Impl\n"
                                    "  {\n"
                                    "    float h = groundH(gCoord);\n"
                                    "    vec3 sS = dFdx(vertexVC.xyz);\n"
                                    "    vec3 sT = dFdy(vertexVC.xyz);\n"
                                    "    vec3 vn = " CVC_FS_NORMAL ";\n"
                                    "    vec3 R1 = cross(sT, vn), R2 = cross(vn, sS);\n"
                                    "    float det = dot(sS, R1);\n"
                                    "    vec3 sg = sign(det) * (dFdx(h)*R1 + dFdy(h)*R2);\n"
                                    "    " CVC_FS_NORMAL " = normalize(abs(det)*vn - 1.4*sg);\n"
                                    "  }\n");
}

// ── the water: generated_world's depth volume under a gently travelling wave ─
int gSeaN = 160;
constexpr int SEA_NZ = 22;
cvc::volume seaVolume(cvc::app &app, const world::generated_world &w, double t) {
  const std::vector<float> field = world::water_field(w, t, gSeaN, SEA_NZ);
  return cvc::volume(
      app, reinterpret_cast<const unsigned char *>(field.data()),
      cvc::dimension(gSeaN, gSeaN, SEA_NZ), cvc::Float,
      cvc::bounding_box(-gHalf, -gHalf, world::water_floor(w), gHalf, gHalf, world::water_top(w)));
}

// ── the sky: a drifting cloud slab (fixed 60^3 grid, world-scaled extent) ─────
constexpr int SKY_N = 60, SKY_NZ = 26;
double gSkyHalf = 640.0, gSkyBase = 220.0, gSkyTop = 360.0;
constexpr int CLOUD_MAPS = 2, CLOUD_DEPTH = 6;
constexpr double CLOUD_TURN = 32.0, CLOUD_STEP0 = 8.1, CLOUD_STEP_DECAY = 0.9;
constexpr double CLOUD_PUFF0 = 8.8, CLOUD_PUFF_DECAY = 0.88;
constexpr double CLOUD_FLOOR = 0.10;
const char *CLOUD_AXIOM = "[A][+++++A][-----A][++++++++++A][----------A][+++++++++++++++A]";
const char *cloudRule(char c) {
  switch (c) {
  case 'A':
    return "FF[+<B]^F[-<C]<F[+<C]vFA";
  case 'B':
    return "F[+<F]F<[-<F]vB";
  case 'C':
    return "^<F[+<F][-<F]^<FC";
  default:
    return nullptr;
  }
}
inline std::size_t skyIdx(int z, int y, int x) { return (std::size_t(z) * SKY_N + y) * SKY_N + x; }
double percentileSorted(const std::vector<float> &a, double q) {
  if (a.empty())
    return 0.0;
  double rank = (q / 100.0) * (a.size() - 1);
  std::size_t lo = std::size_t(std::floor(rank));
  if (lo + 1 >= a.size())
    return a.back();
  return a[lo] + (rank - lo) * (a[lo + 1] - a[lo]);
}
double vhash3(int x, int y, int z) {
  unsigned h = unsigned(x * 374761393 + y * 668265263 + z * 1274126177);
  h = (h ^ (h >> 13)) * 1274126177u;
  return ((h ^ (h >> 16)) & 0xffffffu) / double(0x1000000);
}
double vnoise3(double x, double y, double z) {
  int xi = int(std::floor(x)), yi = int(std::floor(y)), zi = int(std::floor(z));
  double fx = x - xi, fy = y - yi, fz = z - zi;
  auto sm = [](double t) { return t * t * (3.0 - 2.0 * t); };
  fx = sm(fx);
  fy = sm(fy);
  fz = sm(fz);
  auto L = [](double a, double b, double t) { return a + (b - a) * t; };
  double x00 = L(vhash3(xi, yi, zi), vhash3(xi + 1, yi, zi), fx);
  double x10 = L(vhash3(xi, yi + 1, zi), vhash3(xi + 1, yi + 1, zi), fx);
  double x01 = L(vhash3(xi, yi, zi + 1), vhash3(xi + 1, yi, zi + 1), fx);
  double x11 = L(vhash3(xi, yi + 1, zi + 1), vhash3(xi + 1, yi + 1, zi + 1), fx);
  return L(L(x00, x10, fy), L(x01, x11, fy), fz);
}
double fbm3(double x, double y, double z, int octaves) {
  double f = 0.0, amp = 0.5, tot = 0.0, fr = 1.0;
  for (int i = 0; i < octaves; ++i) {
    f += amp * vnoise3(x * fr, y * fr, z * fr);
    tot += amp;
    amp *= 0.5;
    fr *= 2.02;
  }
  return f / tot;
}
std::vector<float> walkClouds(std::mt19937 &rng) {
  std::uniform_real_distribution<double> U(0.0, 1.0);
  auto uni = [&](double a, double b) { return a + (b - a) * U(rng); };
  const int N = SKY_N, NZ = SKY_NZ;
  std::vector<float> field(std::size_t(NZ) * N * N, 0.0f);
  const double zscale = (double(N) / NZ) * ((gSkyTop - gSkyBase) / (2.0 * gSkyHalf));
  double x = uni(0.25, 0.75) * N, y = uni(0.3, 0.7) * N, z = NZ * 0.42;
  double head = uni(0.0, 360.0);
  double step = CLOUD_STEP0, puff = CLOUD_PUFF0, climb = 0.0;
  int depth = 0;
  struct St {
    double x, y, z, head, step, puff, climb;
    int depth;
  };
  std::vector<St> stack;
  std::deque<char> todo(CLOUD_AXIOM, CLOUD_AXIOM + std::strlen(CLOUD_AXIOM));
  int guard = 0;
  while (!todo.empty() && guard < 4000) {
    ++guard;
    char c = todo.front();
    todo.pop_front();
    if (c == 'F') {
      x = std::fmod(x + step * std::cos(head * M_PI / 180.0), double(N));
      if (x < 0)
        x += N;
      y = std::min(std::max(y + step * std::sin(head * M_PI / 180.0), 0.0), double(N - 1));
      z = std::min(std::max(z + climb, 1.0), double(NZ - 2));
      const double p2 = 2.0 * puff * puff;
      for (int gz = 0; gz < NZ; ++gz) {
        double dz = (gz - z) * zscale, dz2 = dz * dz;
        for (int gy = 0; gy < N; ++gy) {
          double dy = gy - y, dyz2 = dy * dy + dz2;
          for (int gx = 0; gx < N; ++gx) {
            double dx = std::fabs(gx - x);
            dx = std::min(dx, double(N) - dx);
            field[skyIdx(gz, gy, gx)] += float(std::exp(-(dx * dx + dyz2) / p2));
          }
        }
      }
    } else if (c == '+')
      head += CLOUD_TURN;
    else if (c == '-')
      head -= CLOUD_TURN;
    else if (c == '<')
      puff *= CLOUD_PUFF_DECAY;
    else if (c == '^')
      climb += 0.55;
    else if (c == 'v')
      climb -= 0.45;
    else if (c == '[')
      stack.push_back({x, y, z, head, step, puff, climb, depth});
    else if (c == ']') {
      if (!stack.empty()) {
        St s = stack.back();
        stack.pop_back();
        x = s.x;
        y = s.y;
        z = s.z;
        head = s.head;
        step = s.step;
        puff = s.puff;
        climb = s.climb;
        depth = s.depth;
      }
    } else if (cloudRule(c) && depth < CLOUD_DEPTH) {
      ++depth;
      step *= CLOUD_STEP_DECAY;
      const char *r = cloudRule(c);
      todo.insert(todo.begin(), r, r + std::strlen(r));
    }
  }
  std::vector<float> sorted(field);
  std::sort(sorted.begin(), sorted.end());
  double m = percentileSorted(sorted, 99.9);
  if (m > 0)
    for (float &v : field)
      v = std::min(1.0f, std::max(0.0f, v / float(m)));
  std::vector<double> w(N), zf(NZ);
  for (int i = 0; i < N; ++i)
    w[i] = std::sqrt(0.5 - 0.5 * std::cos(2.0 * M_PI * i / (N - 1)));
  for (int k = 0; k < NZ; ++k)
    zf[k] = std::sin(0.12 + (M_PI - 0.24) * k / (NZ - 1));
  for (int gz = 0; gz < NZ; ++gz)
    for (int gy = 0; gy < N; ++gy)
      for (int gx = 0; gx < N; ++gx) {
        float &v = field[skyIdx(gz, gy, gx)];
        v *= float(std::min(w[gy], w[gx]) * zf[gz]);
        if (v > 0.0f) {
          double d = fbm3(gx * 0.30, gy * 0.30, gz * 0.62, 5);
          v = float(std::min(1.0, std::max(0.0, v * (0.32 + 1.5 * d))));
        }
      }
  return field;
}
struct SkyModel {
  std::vector<std::vector<float>> maps;
  double norm = 1.0;
  std::vector<float> raw(double shift, double morph) const {
    const int N = SKY_N, NZ = SKY_NZ;
    long mi = long(std::floor(morph));
    int i = int(((mi % CLOUD_MAPS) + CLOUD_MAPS) % CLOUD_MAPS);
    int j = (i + 1) % CLOUD_MAPS;
    double u = morph - std::floor(morph);
    u = u * u * (3.0 - 2.0 * u);
    long ks = long(std::floor(shift));
    double f = shift - ks;
    int k0 = int(((ks % N) + N) % N), k1 = (k0 + 1) % N;
    const std::vector<float> &A = maps[i], &B = maps[j];
    std::vector<float> out(std::size_t(NZ) * N * N);
    for (int z = 0; z < NZ; ++z)
      for (int y = 0; y < N; ++y)
        for (int x = 0; x < N; ++x) {
          int sx0 = ((x - k0) % N + N) % N, sx1 = ((x - k1) % N + N) % N;
          auto mix = [&](int xx) {
            return (1.0 - u) * A[skyIdx(z, y, xx)] + u * B[skyIdx(z, y, xx)];
          };
          double vol = (1.0 - f) * mix(sx0) + f * mix(sx1);
          double lump = std::min(1.0, std::max(0.0, (vol - CLOUD_FLOOR) / (1.0 - CLOUD_FLOOR)));
          out[skyIdx(z, y, x)] = float(lump * lump);
        }
    return out;
  }
  std::vector<float> field(double shift, double morph) const {
    std::vector<float> r = raw(shift, morph);
    double inv = 1.0 / norm;
    for (float &v : r)
      v = float(v * inv);
    return r;
  }
};
SkyModel buildSky() {
  SkyModel sky;
  std::mt19937 rng(20u);
  for (int m = 0; m < CLOUD_MAPS; ++m)
    sky.maps.push_back(walkClouds(rng));
  double norm = 0.0;
  for (int c = 0; c < SKY_N; c += 8)
    for (double mo : {0.0, 0.5, 1.0}) {
      std::vector<float> r = sky.raw(double(c), mo);
      for (float v : r)
        norm = std::max(norm, double(v));
    }
  sky.norm = norm > 0 ? norm : 1.0;
  return sky;
}
cvc::volume skyVolume(cvc::app &app, const std::vector<float> &field) {
  return cvc::volume(
      app, reinterpret_cast<const unsigned char *>(field.data()),
      cvc::dimension(SKY_N, SKY_N, SKY_NZ), cvc::Float,
      cvc::bounding_box(-gSkyHalf, -gSkyHalf, gSkyBase, gSkyHalf, gSkyHalf, gSkyTop));
}
void skyTransfer(std::vector<double> &color, std::vector<double> &opacity) {
  color = {0.0, 0.80, 0.85, 0.93, 0.45, 0.94, 0.96, 0.98, 1.0, 1.00, 1.00, 1.00};
  opacity = {0.0, 0.0, 0.22, 0.0, 0.55, 0.26, 1.0, 0.54};
}

// ── the sun (a flat-lit disc + halo, camera-relative, at infinity) ───────────
double SUN_AZ = -52.0, SUN_EL = 36.0;
Vec3d sunDir(double azDeg, double elDeg) {
  double az = azDeg * M_PI / 180.0, el = elDeg * M_PI / 180.0;
  return {std::cos(el) * std::sin(az), -std::cos(el) * std::cos(az), std::sin(el)};
}
cvc::geometry discGeom(cvc::app &app, Vec3d c, Vec3d normal, double radius, int seg = 48) {
  Vec3d n = vnorm(normal);
  Vec3d up = std::fabs(n.z) < 0.9 ? Vec3d{0, 0, 1} : Vec3d{1, 0, 0};
  Vec3d u = vnorm(vcross(n, up)), v = vcross(n, u);
  cvc::geometry g(app);
  g.points().push_back({c.x, c.y, c.z});
  for (int i = 0; i < seg; ++i) {
    double th = i * 2.0 * M_PI / seg;
    Vec3d p = c + (u * (radius * std::cos(th))) + (v * (radius * std::sin(th)));
    g.points().push_back({p.x, p.y, p.z});
  }
  for (int i = 0; i < seg; ++i)
    g.tris().push_back({0, idx_t(1 + i), idx_t(1 + (i + 1) % seg)});
  return g;
}

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  bool offscreen = false, noShadows = false, verbose = false, no_ui = false, show_ui = false;
  int frames = 0, width = 1280, height = 800;
  double fps = 30.0, half = 600.0;
  unsigned long seed = 7;
  std::string png, captureStr = "none", outDir = "frames", preset = "standard";
  po::options_description desc("terrain_lab — a pure-C++ cvcGL viewer for cvc::world\nOptions");
  desc.add_options()("help,h", "show this help and exit")                                     //
      ("offscreen", po::bool_switch(&offscreen), "render offscreen (no window)")              //
      ("no-shadows", po::bool_switch(&noShadows), "disable prop shadows")                     //
      ("verbose,v", po::bool_switch(&verbose), "show cvcGL debug logging")                    //
      ("no-ui", po::bool_switch(&no_ui), "hide the ImGui overlay")                            //
      ("show-ui", po::bool_switch(&show_ui), "force the ImGui overlay on (even offscreen)")   //
      ("frames", po::value<int>(&frames)->default_value(0), "stop after N frames (0 = live)") //
      ("png", po::value<std::string>(&png)->default_value(""), "write final frame to PNG")    //
      ("capture", po::value<std::string>(&captureStr)->default_value("none"),                 //
       "cinematic capture: none | orbit | fly (forces --offscreen)")                          //
      ("width", po::value<int>(&width)->default_value(1280), "render width")                  //
      ("height", po::value<int>(&height)->default_value(800), "render height")                //
      ("fps", po::value<double>(&fps)->default_value(30.0), "capture fps (synthetic clock)")  //
      ("out", po::value<std::string>(&outDir)->default_value("frames"), "capture output dir") //
      ("half", po::value<double>(&half)->default_value(600.0), "world half-extent (metres)")  //
      ("seed", po::value<unsigned long>(&seed)->default_value(7), "generation seed")          //
      ("preset", po::value<std::string>(&preset)->default_value("standard"),
       "standard | forest | urban | sparse | island");
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const std::exception &e) {
    std::cerr << "error: " << e.what() << "\n\n" << desc << "\n";
    return 2;
  }
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  enum class Capture { None, Orbit, Fly } capture = Capture::None;
  if (captureStr == "orbit")
    capture = Capture::Orbit;
  else if (captureStr == "fly")
    capture = Capture::Fly;
  else if (captureStr != "none") {
    std::cerr << "unknown --capture '" << captureStr << "'\n";
    return 2;
  }
  const bool capturing = capture != Capture::None;
  if (capturing) {
    offscreen = true;
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);
  }

  gHalf = half;
  // Water/cloud extents follow the world size.
  gSeaN = std::max(80, std::min(160, int(std::lround(2.0 * gHalf / 8.0))));
  gSkyHalf = gHalf * 1.06;

  world::generated_world_params gp;
  gp.seed = seed;
  gp.half_m = half; // the --half CLI value seeds the live world-size knob
  gp.preset = world::world_preset_from_name(preset);

  cvc::app app;
  app.properties("system.log_verbosity", verbose ? "6" : "2");
  SceneGraph sg(app, "terrain_lab");

  int colorMode = 0; // material
  world::generated_world W;

  // Paint the terrain texture (material + dirt + roads + plazas) for the current
  // colour mode and hand it to the terrain node — on rebuild and on colour switch.
  auto applyTexture = [&]() {
    if (!sg.hasGraphics("terrain"))
      return;
    std::vector<unsigned char> rgb = world::paint_surface(W, world::surface_color(colorMode), TEX);
    cvc::image img(TEX, TEX, cvc::image::pixel_format::RGB, cvc::image::data_type::u8, rgb.data());
    std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("terrain"))->setTexture(img, false);
  };

  // A merged mesh into the scene under `name`: replaced in place on a rebuild, created (and styled
  // by `style`) the first time, removed when the new world has none.
  auto putMesh = [&](const char *name, const cvc::geometry &g,
                     const std::function<void(GeometryNode &)> &style) {
    if (g.const_points().empty()) {
      if (sg.hasGraphics(name))
        sg.removeGraphics(name);
      return;
    }
    if (sg.hasGraphics(name)) {
      std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics(name))->setGeometry(g);
      return;
    }
    if (auto n = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics(name, g)))
      style(*n);
  };

  // Build (or rebuild) the whole world into the scene. Called once up front and on every ImGui
  // "Regenerate". Keeps the same node names so nodes are reused.
  auto rebuild = [&](bool reframe) {
    // The world size is a live knob: every size-dependent extent follows it, and the terrain
    // mesh keeps a fixed cell size (capped), so a big terrain stays detailed — the mesh the LOD
    // path is stress-tested on.
    gHalf = std::max(50.0, gp.half_m);
    gSkyHalf = gHalf * 1.06;
    auto _tp = std::chrono::steady_clock::now();
    const bool _prof = std::getenv("CVC_TL_PROF") != nullptr;
    auto lap = [&](const char *name) {
      auto n = std::chrono::steady_clock::now();
      if (_prof)
        std::fprintf(stderr, "  [prof] %-16s %7.0f ms\n", name,
                     std::chrono::duration<double, std::milli>(n - _tp).count());
      _tp = n;
    };
    W = world::generate_world(gp, &cityLibrary());
    lap("generate-world");
    gSeaN = world::water_resolution(W);
    gSkyBase = W.height_max + gHalf * 0.35 + 60.0;
    gSkyTop = gSkyBase + gHalf * 0.30 + 80.0;

    // A plain copy of the terrain's mesh (its arrays shared copy-on-write).
    const cvc::geometry terrain(*W.terrain);
    if (sg.hasGraphics("terrain"))
      std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("terrain"))->setGeometry(terrain);
    else {
      auto t = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("terrain", terrain));
      t->setUseSingleColor(true); // surface colour comes from the baked texture
      t->setColor(1.0, 1.0, 1.0);
      t->setAmbient(0.5);
      t->setDiffuse(0.95);
      addTerrainBump(
          *t); // fragment bump map: fine surface relief in the lighting (from lsystem_forest)
    }
    applyTexture(); // material + dirt + roads + plazas, per the colour mode
    lap("texture");

    putMesh("props", W.props, [](GeometryNode &p) {
      p.setUseSingleColor(false);
      p.setAmbient(0.45);
      p.setDiffuse(0.85);
    });
    putMesh("buildings", W.buildings, [](GeometryNode &b) {
      b.setUseSingleColor(false);
      b.setAmbient(0.45);
      b.setDiffuse(0.85);
    });
    // The extruded 3D road surface (ribbons + junction pads), draped on the terrain.
    putMesh("roads", W.roads, [](GeometryNode &rd) {
      rd.setUseSingleColor(false);
      rd.setAmbient(0.5);
      rd.setDiffuse(0.7);
    });
    putMesh("foliage", W.foliage, [](GeometryNode &fol) {
      fol.setUseSingleColor(false);
      fol.setAmbient(0.6);
      fol.setDiffuse(0.7);
    });
    // Pine needles: a LINES mesh (the native_pine conifer foliage), rendered as dark-green lines.
    putMesh("needles", W.needles, [](GeometryNode &nd) {
      nd.setRenderMode(cvc::gl::GeometryRenderMode::LINES);
      nd.setUseSingleColor(true);
      nd.setColor(0.12, 0.34, 0.14);
      nd.setAmbient(0.55);
      nd.setDiffuse(0.75);
    });

    // water volume
    if (sg.hasGraphics("water"))
      std::dynamic_pointer_cast<VolumeNode>(sg.getGraphics("water"))
          ->setVolume(seaVolume(app, W, 0.0));
    else {
      auto w = sg.addGraphics("water", seaVolume(app, W, 0.0));
      std::vector<double> col, op;
      world::water_transfer(0.0, col, op);
      w->setTransferFunction(col, op);
      w->setShading(true);
      w->setAmbient(0.18);
      w->setDiffuse(0.72);
      w->setSpecular(0.85);
      w->setSpecularPower(70.0);
    }
    lap("scene");
    std::printf("terrain_lab: seed=%llu preset=%s trees=%d rocks=%d buildings=%zu props=%zu "
                "occupied=%.2f%%\n",
                (unsigned long long)gp.seed, kPresetNames[gp.preset], gp.trees, gp.rocks,
                W.building_count, W.model->props().size(),
                100.0 * double(W.raster.occupied_count()) /
                    std::max<std::size_t>(1, W.raster.klass.size()));
    (void)reframe;
  };

  // Recolour the terrain (colour-mode switch) — repaint the texture, no regeneration.
  auto recolor = [&]() { applyTexture(); };

  rebuild(true);

  // Stage lighting (a rig aimed at the world, not a sky).
  cvc::gl::StageLighting rig(sg);
  rig.setStage(0.0, 0.0, 0.0, gHalf);
  rig.applyPreset(cvc::gl::StageLighting::Preset::ThreePoint);
  rig.setKey(1.15, SUN_AZ, SUN_EL, gHalf * 0.3);
  rig.setWarmth(0.4);

  SceneRenderer view(sg, width, height, offscreen, "main");
  // The loop yields to the browser once per frame (emscripten_sleep(0) at its end), so VTK
  // need not yield again inside every render (FrameYield::App: VTK 9.5's in-render
  // emscripten_sleep off; a no-op natively).
  view.setFrameYield(SceneRenderer::FrameYield::App);
  view.renderer()->GradientBackgroundOn();
  view.renderer()->SetBackground(0.66, 0.71, 0.74);
  view.renderer()->SetBackground2(0.23, 0.44, 0.80);

  const bool shadows = !noShadows && sg.setShadowsEnabled(true);
  if (shadows) {
    sg.setShadowResolution(2048);
    sg.setShadowUpdateInterval(capturing ? 1 : 8);
  }

  CameraController cam(view);
  cvc::bounding_box b = sg.computeGraphicsBounds();
  cam.frameBounds(b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz);
  cvc::gl::FpsHud hud(view);
  cvc::gl::TouchGestures touch(view, cam);

  bool wantRegen = false;

#ifdef CVC_ENABLE_IMGUI
  cvc::gl::ImGuiOverlay ui(view);
  ImGui::SetCurrentContext(ui.imguiContext());
  ui.attachCamera(cam);
  ui.setVisible(show_ui || (!no_ui && !capturing && !offscreen));
  bool uiScene = false, uiLighting = false;
  ui.setDrawCallback([&] {
    if (ImGui::BeginMainMenuBar()) {
      cvc_examples::demos_menu("terrain_lab");
      if (ImGui::BeginMenu("Scene")) {
        cvc::gl::ui::SceneMenuItems(sg, &uiScene, &uiLighting);
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    ImGui::SetNextWindowPos(ImVec2(12, 34), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Terrain Lab");
    ImGui::TextWrapped("Live view of cvc::world - the L-system terrain source.");
    ImGui::Separator();

    int seedI = int(gp.seed);
    if (ImGui::InputInt("seed", &seedI)) {
      gp.seed = std::uint64_t(std::max(0, seedI));
    }
    if (ImGui::Combo("preset", &gp.preset, kPresetNames, IM_ARRAYSIZE(kPresetNames))) {
      world::apply_world_preset(gp); // quick-set the sliders for this preset
      wantRegen = true;
    }
    ImGui::Combo("ontology", &gp.ontology, kOntologyNames, IM_ARRAYSIZE(kOntologyNames));
    { // world size: a first-class knob (metres of half-extent). Logarithmic feel via a wide range
      // so the lab spans tight blocks up to very large terrains for the LOD stress test.
      float hm = float(gp.half_m);
      if (ImGui::SliderFloat("world half m", &hm, 150.0f, 10000.0f, "%.0f",
                             ImGuiSliderFlags_Logarithmic))
        gp.half_m = hm;
      const cvc::world_units::measurement span =
          gUnits.format(2.0 * gp.half_m, cvc::world_units::dimension::length);
      const int tn = W.terrain ? W.terrain->rows() : 0;
      ImGui::Text("   span %.2f %s  |  terrain %dx%d", span.value, span.unit.c_str(), tn, tn);
    }
    ImGui::SliderInt("trees", &gp.trees, 0, 3000);
    ImGui::SliderInt("rocks", &gp.rocks, 0, 600);
    ImGui::SliderInt("buildings", &gp.buildings, 0, 600);
    ImGui::SliderInt("tree gens", &gp.tree_gen, 2, 8);
    { // amp / sea sliders (double -> float bridge). Ranges widened for large, dramatic relief.
      float amp = float(gp.amp_m), sea = float(gp.water_level_m);
      if (ImGui::SliderFloat("relief amp m", &amp, 2.0f, 400.0f))
        gp.amp_m = amp;
      if (ImGui::SliderFloat("water level m", &sea, 0.0f, 200.0f))
        gp.water_level_m = sea;
    }
    if (ImGui::Button("Regenerate"))
      wantRegen = true;
    ImGui::SameLine();
    if (ImGui::Button("Randomize")) {
      gp.seed = gp.seed * 6364136223846793005ull + 1442695040888963407ull;
      wantRegen = true;
    }
    ImGui::Separator();
    if (ImGui::Combo("colour by", &colorMode, kColorModeNames, IM_ARRAYSIZE(kColorModeNames)))
      recolor();
    ImGui::Text("%zu props  |  %.1f%% occupied", W.model->props().size(),
                100.0 * double(W.raster.occupied_count()) /
                    std::max<std::size_t>(1, W.raster.klass.size()));
    ImGui::End();

    cvc::gl::ui::ScenePanel(sg, &uiScene);
    cvc::gl::ui::StageLightingPanel(rig, &uiLighting);
  });
#endif
  if (offscreen)
    hud.setEnabled(false);

  // The sun billboard (added after framing so it doesn't inflate the bounds).
  Vec3d sd = sunDir(SUN_AZ, SUN_EL);
  Vec3d face{-sd.x, -sd.y, -sd.z};
  double sunR = gHalf * 0.10, sunRef = gHalf * 4.0, sunCap = gHalf * 1.1;
  sg.addGraphics("sun", discGeom(app, {0, 0, 0}, face, sunR));
  {
    auto disc = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("sun"));
    disc->setColor(1.0, 0.97, 0.88);
    disc->setAmbient(1.0);
    disc->setDiffuse(0.0);
    disc->setOpacity(0.99);
  }
  sg.addGraphics("sun_halo", discGeom(app, {0, 0, 0}, face, sunR * 3.2));
  {
    auto halo = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("sun_halo"));
    halo->setColor(1.0, 0.90, 0.72);
    halo->setAmbient(1.0);
    halo->setDiffuse(0.0);
    halo->setOpacity(0.22);
  }
  auto sunNode = sg.getGraphics("sun");
  auto sunHalo = sg.getGraphics("sun_halo");
  auto placeSky = [&](const Vec3d &eye) {
    double eDotS = eye.x * sd.x + eye.y * sd.y + eye.z * sd.z;
    double dd = std::min(sunRef, sunCap - eDotS);
    bool vis = dd > 1.0;
    sunNode->setVisible(vis);
    sunHalo->setVisible(vis);
    if (!vis)
      return;
    double s = dd / sunRef;
    sunNode->setScale(s, s, s);
    sunHalo->setScale(s, s, s);
    Vec3d p = eye + sd * dd;
    sunNode->setPosition(p.x, p.y, p.z);
    sunHalo->setPosition(p.x, p.y, p.z);
  };

  // The sky slab (added after framing too — it is scenery, not a framing target).
  SkyModel sky = buildSky();
  auto skyNode = sg.addGraphics("sky", skyVolume(app, sky.field(0.0, 0.0)));
  skyNode->setShading(false);
  skyNode->setAmbient(0.95);
  skyNode->setDiffuse(0.35);
  skyNode->setSpecular(0.0);
  skyNode->setVolumetricScattering(0.0);
  {
    std::vector<double> col, op;
    skyTransfer(col, op);
    skyNode->setTransferFunction(col, op);
  }

  sg.setDiagnosticChromeVisible(false);
  sg.processEvents();

  {
    const cvc::world_units::measurement span =
        gUnits.format(2.0 * gHalf, cvc::world_units::dimension::length);
    std::printf(
        "terrain_lab: %s, world %.2f %s (1 world unit = 1 m), %s. Tab=orbit/fly, WASD+mouse=fly.\n",
        offscreen ? "offscreen" : "onscreen", span.value, span.unit.c_str(),
        shadows ? "shadows on" : "no shadows");
  }

  const std::vector<Vec3d> flyEye = {{-gHalf * 1.5, -gHalf * 1.3, gHalf * 0.4},
                                     {-gHalf * 0.6, gHalf * 0.2, gHalf * 0.35},
                                     {gHalf * 0.5, gHalf * 0.6, gHalf * 0.5},
                                     {gHalf * 1.1, -gHalf * 0.3, gHalf * 0.7}};
  const std::vector<Vec3d> flyTgt(4, {0, 0, gHalf * 0.1});
  auto crInterp = [](const std::vector<Vec3d> &p, double s) {
    int m = int(p.size());
    double x = s * (m - 1);
    int i = std::min(int(std::floor(x)), m - 2);
    if (i < 0)
      i = 0;
    double u = x - i, u2 = u * u, u3 = u2 * u;
    auto seg = [&](double a, double b, double c, double d) {
      return 0.5 * (2 * b + (-a + c) * u + (2 * a - 5 * b + 4 * c - d) * u2 +
                    (-a + 3 * b - 3 * c + d) * u3);
    };
    const Vec3d &p0 = p[std::max(i - 1, 0)], &p1 = p[i], &p2 = p[i + 1],
                &p3 = p[std::min(i + 2, m - 1)];
    return Vec3d{seg(p0.x, p1.x, p2.x, p3.x), seg(p0.y, p1.y, p2.y, p3.y),
                 seg(p0.z, p1.z, p2.z, p3.z)};
  };
  const std::string azPath =
      CameraController::viewerStatePath("terrain_lab", "main") + ".orbit.azimuth";
  double orbitAz0 = 0.0;
  if (capture == Capture::Orbit) {
    try {
      orbitAz0 = cvc::state::instance(app)(azPath).value<double>();
    } catch (...) {
    }
  }

  auto start = std::chrono::steady_clock::now();
  double last = 0.0;
  long frame = 0;
  int n = 0;
  const int SEA_STRIDE = 4, CLOUD_STRIDE = 8;
  // The sea and the cloud run on the app's world clock, driven once per frame through a
  // binding (sim.speed / sim.paused, ...): one speed and pause for both, and a capture's
  // fixed 1/fps makes the frames reproducible.
  cvc::world_clock &clock = app.world_clock();
  cvc::world_clock_binding clockDrive(app, clock);
  while (!view.windowClosed() && !cvc_examples::demo_switch_requested()) {
    double t, dt;
    if (capturing) {
      t = frame / fps;
      dt = 1.0 / fps;
    } else {
      t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      dt = t - last;
      last = t;
    }
    view.processUIEvents();
    touch.update();
    clockDrive.drive(dt);
    const double wt = clock.t(); // world time for the sea and the cloud
    if (wantRegen) {
      wantRegen = false;
      rebuild(false);
    }
    const bool seaDue = capturing || frame % SEA_STRIDE == 0;
    const bool cloudDue = capturing || frame % CLOUD_STRIDE == 1;
    if (seaDue && sg.hasGraphics("water")) {
      auto w = std::dynamic_pointer_cast<VolumeNode>(sg.getGraphics("water"));
      w->updateScalars(world::water_field(W, wt, gSeaN, SEA_NZ));
      std::vector<double> col, op;
      world::water_transfer(wt, col, op);
      w->setTransferFunction(col, op);
    }
    if (cloudDue) {
      double shift = wt * (gHalf * 0.010) * SKY_N / (2.0 * gSkyHalf);
      double morph = wt / 60.0 * CLOUD_MAPS;
      skyNode->updateScalars(sky.field(shift, morph));
    }
    if (capture == Capture::Fly) {
      double s = frames > 1 ? double(frame) / (frames - 1) : 0.0;
      s = s * s * (3.0 - 2.0 * s);
      Vec3d e = crInterp(flyEye, s), tg = crInterp(flyTgt, s);
      view.setCamera(e.x, e.y, e.z, tg.x, tg.y, tg.z, 0, 0, 1, 42.0, 1.0, gHalf * 12.0);
      placeSky(e);
    } else {
      if (capture == Capture::Orbit && frames > 0)
        cvc::state::instance(app)(azPath).value(orbitAz0 + 360.0 * double(frame) / frames);
      cam.update(dt);
      double ep[3], fp[3], upv[3];
      cam.getPose(ep, fp, upv);
      placeSky({ep[0], ep[1], ep[2]});
      view.renderer()->ResetCameraClippingRange();
    }
    if (capturing) {
      char path[1024];
      std::snprintf(path, sizeof path, "%s/frame_%05ld.png", outDir.c_str(), frame);
      view.writePNG(path);
    } else {
      view.render();
    }
#ifdef __EMSCRIPTEN__
#ifndef __EMSCRIPTEN_PTHREADS__
    sg.publisher().flush();
#endif
    emscripten_sleep(0);
#endif
    ++frame;
    if (frames > 0 && ++n >= frames)
      break;
  }
  if (!png.empty())
    view.writePNG(png);
  hud.detach();
  cam.detach();
  return 0;
}
