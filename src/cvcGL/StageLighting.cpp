// StageLighting — a key/fill/back/wash rig of aimed spot lights.
// See the header for why aimed spots (not a directional sun) are what make the
// shadow map sharp.

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StageLighting.h>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace gl {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;

// A light as actually placed this pass — what the gizmos draw, so the wireframe
// always shows the rig that is really lighting the scene.
struct Placed {
  double px, py, pz, tx, ty, tz, cone, r, g, b;
};

struct Special {
  double x = 0, y = 0, z = 0;
  double tx = 0, ty = 0, tz = 0;
  double cone = 20.0, intensity = 1.0;
  double r = 1, g = 1, b = 1;
  bool live = true;
};

// Place a light on a sphere around the stage: azimuth is a compass bearing
// (0 = +Y, growing toward +X) to match SceneGraph's directional convention, so
// "the key is at 45 degrees" means the same thing for both light kinds.
inline void orbitPoint(double cx, double cy, double cz, double dist, double azDeg, double elDeg,
                       double &x, double &y, double &z) {
  const double ce = std::cos(elDeg * kDeg), se = std::sin(elDeg * kDeg);
  x = cx + dist * ce * std::sin(azDeg * kDeg);
  y = cy - dist * ce * std::cos(azDeg * kDeg);
  z = cz + dist * se;
}
} // namespace

struct StageLighting::Impl {
  SceneGraph *scene = nullptr;
  cvc::app *ctx = nullptr; // for bound UI (state_object keeps its own privately)
  std::string path;        // ditto: the rig's state path

  // stage
  double cx = 0, cy = 0, cz = 0, radius = 10.0;

  // roles (mirrored to state)
  bool on = true;
  double keyI = 1.0, keyAz = -50.0, keyEl = 38.0, keyCone = 32.0;
  double fillI = 0.35;
  double backI = 0.55;
  double washI = 0.30;
  int washCount = 4;
  double washHeight = 1.8; // multiples of the stage radius
  double ambient = 0.22;
  double envI = 0.30;
  double warmth = 0.35;

  bool gizmos = false;
  bool applying = false; // re-entry guard, see apply()
  double beamAlpha = 0.18;
  std::vector<Special> specials;
  // The rig's lights are NODES now, tracked by name so a rebuild can remove
  // exactly what it created. Each is individually inspectable in the scene
  // hierarchy, parentable, and scriptable through its own state path.
  std::vector<std::string> nodes;

  // What apply() placed this pass, so the gizmos draw exactly the rig that is
  // actually lighting the scene rather than recomputing it.
  std::vector<Placed> placed;
};

std::string StageLighting::sceneStatePath(const std::string &scenePrefix) {
  return scenePrefix + ".lighting";
}

cvc::app &StageLighting::appContext() const { return *m_impl->ctx; }
const std::string &StageLighting::statePath() const { return m_impl->path; }

StageLighting::StageLighting(cvc::app &ctx, const std::string &statePath_, SceneGraph *scene)
    : cvc::state_object<StageLighting>(ctx, statePath_), m_impl(std::make_unique<Impl>()) {
  // Synchronous reactions on the calling thread, as with CameraController: the
  // rig is driven from the render thread and rebuilds VTK lights directly.
  this->setInstanceThreading(false);
  m_impl->scene = scene;
  m_impl->ctx = &ctx;
  m_impl->path = statePath_;
  seedState();
}

StageLighting::StageLighting(SceneGraph &scene)
    : StageLighting(scene.appContext(), sceneStatePath(scene.getStatePrefix()), &scene) {}

StageLighting::~StageLighting() {
  // Drop our lights so a destroyed rig does not leave the scene lit by ghosts.
  if (m_impl->scene)
    for (const auto &n : m_impl->nodes)
      m_impl->scene->removeGraphics(n);
}

void StageLighting::seedState() {
  Impl &s = *m_impl;
  getState("enabled").value(s.on ? 1 : 0);
  getState("key_intensity").value(s.keyI);
  getState("key_azimuth").value(s.keyAz);
  getState("key_elevation").value(s.keyEl);
  getState("key_cone").value(s.keyCone);
  getState("fill_intensity").value(s.fillI);
  getState("back_intensity").value(s.backI);
  getState("wash_intensity").value(s.washI);
  getState("wash_count").value(s.washCount);
  getState("wash_height").value(s.washHeight);
  getState("ambient").value(s.ambient);
  getState("env_intensity").value(s.envI);
  getState("warm_key").value(s.warmth);
  getState("show_gizmos").value(s.gizmos ? 1 : 0);
  getState("gizmo_beam_alpha").value(s.beamAlpha);
  getState("stage_x").value(s.cx);
  getState("stage_y").value(s.cy);
  getState("stage_z").value(s.cz);
  getState("stage_radius").value(s.radius);
}

void StageLighting::readAllFromState() {
  Impl &s = *m_impl;
  try {
    s.on = getState("enabled").value<int>() != 0;
    s.keyI = getState("key_intensity").value<double>();
    s.keyAz = getState("key_azimuth").value<double>();
    s.keyEl = getState("key_elevation").value<double>();
    s.keyCone = getState("key_cone").value<double>();
    s.fillI = getState("fill_intensity").value<double>();
    s.backI = getState("back_intensity").value<double>();
    s.washI = getState("wash_intensity").value<double>();
    s.washCount = getState("wash_count").value<int>();
    s.washHeight = getState("wash_height").value<double>();
    s.ambient = getState("ambient").value<double>();
    s.envI = getState("env_intensity").value<double>();
    s.warmth = getState("warm_key").value<double>();
    s.gizmos = getState("show_gizmos").value<int>() != 0;
    s.beamAlpha = getState("gizmo_beam_alpha").value<double>();
    s.cx = getState("stage_x").value<double>();
    s.cy = getState("stage_y").value<double>();
    s.cz = getState("stage_z").value<double>();
    s.radius = getState("stage_radius").value<double>();
  } catch (const std::exception &) {
    // partially-initialised state: keep what we have
  }
}

void StageLighting::handleStateChanged(const std::string &) {
  readAllFromState();
  apply();
}

void StageLighting::setStage(double cx, double cy, double cz, double radius) {
  Impl &s = *m_impl;
  s.cx = cx;
  s.cy = cy;
  s.cz = cz;
  s.radius = std::max(1e-3, radius);
  getState("stage_x").value(s.cx);
  getState("stage_y").value(s.cy);
  getState("stage_z").value(s.cz);
  getState("stage_radius").value(s.radius);
  apply();
}

void StageLighting::stage(double &cx, double &cy, double &cz, double &radius) const {
  cx = m_impl->cx;
  cy = m_impl->cy;
  cz = m_impl->cz;
  radius = m_impl->radius;
}

void StageLighting::frameBounds(double minX, double minY, double minZ, double maxX, double maxY,
                                double maxZ) {
  // Footprint half-diagonal, not the full 3-D diagonal: a tall scene (a sky
  // dome, a cloud slab) must not widen the cones, because a cone sized to the
  // whole scene is exactly the directional-light problem this class avoids.
  const double dx = maxX - minX, dy = maxY - minY;
  setStage(0.5 * (minX + maxX), 0.5 * (minY + maxY), minZ + 0.15 * (maxZ - minZ),
           0.5 * std::sqrt(dx * dx + dy * dy));
}

const char *StageLighting::presetName(Preset p) {
  switch (p) {
  case Preset::ThreePoint:
    return "three-point";
  case Preset::Overhead:
    return "overhead";
  case Preset::Dramatic:
    return "dramatic";
  case Preset::Flat:
    return "flat";
  }
  return "three-point";
}

void StageLighting::applyPreset(Preset p) {
  Impl &s = *m_impl;
  switch (p) {
  case Preset::ThreePoint:
    s.keyI = 1.0;
    s.keyEl = 38.0;
    s.keyCone = 32.0;
    s.fillI = 0.35;
    s.backI = 0.55;
    s.washI = 0.30;
    s.washCount = 4;
    s.ambient = 0.22;
    s.warmth = 0.35;
    break;
  case Preset::Overhead:
    s.keyI = 0.45;
    s.keyEl = 55.0;
    s.keyCone = 40.0;
    s.fillI = 0.30;
    s.backI = 0.25;
    s.washI = 0.85;
    s.washCount = 6;
    s.ambient = 0.30;
    s.warmth = 0.15;
    break;
  case Preset::Dramatic:
    // Narrow hard key, almost no fill: the shadow is the subject. The narrow
    // cone also concentrates the shadow map, so this preset is the sharpest.
    s.keyI = 1.35;
    s.keyEl = 30.0;
    s.keyCone = 22.0;
    s.fillI = 0.08;
    s.backI = 0.85;
    s.washI = 0.05;
    s.washCount = 2;
    s.ambient = 0.08;
    s.warmth = 0.55;
    break;
  case Preset::Flat:
    s.keyI = 0.0;
    s.fillI = 0.0;
    s.backI = 0.0;
    s.washI = 1.0;
    s.washCount = 6;
    s.ambient = 0.45;
    s.warmth = 0.0;
    break;
  }
  seedState(); // push the preset out through state so bound UI follows
  apply();
}

void StageLighting::setKey(double intensity, double azimuthDeg, double elevationDeg,
                           double coneDeg) {
  Impl &s = *m_impl;
  s.keyI = intensity;
  s.keyAz = azimuthDeg;
  s.keyEl = elevationDeg;
  s.keyCone = coneDeg;
  getState("key_intensity").value(s.keyI);
  getState("key_azimuth").value(s.keyAz);
  getState("key_elevation").value(s.keyEl);
  getState("key_cone").value(s.keyCone);
  apply();
}

void StageLighting::setFill(double intensity) {
  m_impl->fillI = intensity;
  getState("fill_intensity").value(intensity);
  apply();
}

void StageLighting::setBack(double intensity) {
  m_impl->backI = intensity;
  getState("back_intensity").value(intensity);
  apply();
}

void StageLighting::setWash(double intensity, int count, double heightScale) {
  Impl &s = *m_impl;
  s.washI = intensity;
  s.washCount = std::max(0, count);
  s.washHeight = std::max(0.2, heightScale);
  getState("wash_intensity").value(s.washI);
  getState("wash_count").value(s.washCount);
  getState("wash_height").value(s.washHeight);
  apply();
}

void StageLighting::setEnvironment(double intensity) {
  m_impl->envI = intensity < 0.0 ? 0.0 : intensity;
  getState("env_intensity").value(m_impl->envI);
  apply();
}
double StageLighting::environment() const { return m_impl->envI; }

void StageLighting::setAmbient(double a) {
  m_impl->ambient = std::max(0.0, std::min(1.0, a));
  getState("ambient").value(m_impl->ambient);
  apply();
}

void StageLighting::setWarmth(double amount) {
  m_impl->warmth = std::max(0.0, std::min(1.0, amount));
  getState("warm_key").value(m_impl->warmth);
  apply();
}

int StageLighting::addSpecial(double x, double y, double z, double tx, double ty, double tz,
                              double coneDeg, double intensity, double r, double g, double b) {
  Special sp;
  sp.x = x;
  sp.y = y;
  sp.z = z;
  sp.tx = tx;
  sp.ty = ty;
  sp.tz = tz;
  sp.cone = coneDeg;
  sp.intensity = intensity;
  sp.r = r;
  sp.g = g;
  sp.b = b;
  m_impl->specials.push_back(sp);
  apply();
  return static_cast<int>(m_impl->specials.size()) - 1;
}

void StageLighting::moveSpecial(int i, double x, double y, double z) {
  if (i < 0 || i >= static_cast<int>(m_impl->specials.size()))
    return;
  auto &sp = m_impl->specials[static_cast<std::size_t>(i)];
  sp.x = x;
  sp.y = y;
  sp.z = z;
  apply();
}

void StageLighting::aimSpecial(int i, double tx, double ty, double tz) {
  if (i < 0 || i >= static_cast<int>(m_impl->specials.size()))
    return;
  auto &sp = m_impl->specials[static_cast<std::size_t>(i)];
  sp.tx = tx;
  sp.ty = ty;
  sp.tz = tz;
  apply();
}

void StageLighting::removeSpecial(int i) {
  if (i < 0 || i >= static_cast<int>(m_impl->specials.size()))
    return;
  // Tombstone rather than erase: indices handed out earlier stay valid.
  m_impl->specials[static_cast<std::size_t>(i)].live = false;
  apply();
}

int StageLighting::specialCount() const {
  int n = 0;
  for (const auto &sp : m_impl->specials)
    if (sp.live)
      ++n;
  return n;
}

void StageLighting::setEnabled(bool on) {
  m_impl->on = on;
  getState("enabled").value(on ? 1 : 0);
  apply();
}

bool StageLighting::enabled() const { return m_impl->on; }

int StageLighting::shadowCasterCount() const {
  // Key, back and every special cast; fill and wash deliberately do not (a
  // second shadow from a fill light is the classic amateur-lighting tell, and
  // each caster costs a full scene depth re-render per bake).
  int n = 0;
  const Impl &s = *m_impl;
  if (s.on && s.keyI > 0.0)
    ++n;
  if (s.on && s.backI > 0.0)
    ++n;
  if (s.on)
    n += specialCount();
  return n;
}

void StageLighting::setGizmosVisible(bool on) {
  m_impl->gizmos = on;
  getState("show_gizmos").value(on ? 1 : 0);
  apply();
}
bool StageLighting::gizmosVisible() const { return m_impl->gizmos; }

void StageLighting::setGizmoBeamAlpha(double a) {
  m_impl->beamAlpha = a < 0.0 ? 0.0 : (a > 1.0 ? 1.0 : a);
  getState("gizmo_beam_alpha").value(m_impl->beamAlpha);
  apply();
}
double StageLighting::gizmoBeamAlpha() const { return m_impl->beamAlpha; }

namespace {
// Gizmos, as actual objects rather than wireframe: a BLACK CONE fixture with an
// ILLUMINATED BULB at its mouth, and a TRANSLUCENT BEAM cone out to the target.
//
// The beam is the debugging surface — it is the shadow-map frustum VTK bakes for
// that light, so seeing it tells you what the light can shadow and how much of
// the map is landing on empty space. Its alpha is configurable because an opaque
// beam hides the shadows you turned it on to inspect.
//
// Built as three separate meshes so they can carry different materials (the
// fixture is opaque and unlit-black, the bulb is emissive, the beam is
// translucent); merging them would force one material on all three.
struct GizmoMeshes {
  cvc::geometry fixture, bulb, beam;
};

// Orthonormal frame with `d` as the axis.
inline void frame_from_axis(const double d[3], double u[3], double v[3]) {
  double up[3] = {0, 0, 1};
  if (std::abs(d[2]) > 0.9) {
    up[0] = 1;
    up[2] = 0;
  }
  u[0] = up[1] * d[2] - up[2] * d[1];
  u[1] = up[2] * d[0] - up[0] * d[2];
  u[2] = up[0] * d[1] - up[1] * d[0];
  const double ul = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  for (int i = 0; i < 3; ++i)
    u[i] /= (ul > 1e-12 ? ul : 1.0);
  v[0] = d[1] * u[2] - d[2] * u[1];
  v[1] = d[2] * u[0] - d[0] * u[2];
  v[2] = d[0] * u[1] - d[1] * u[0];
}

// A cone as a triangle surface: apex -> a ring of `seg` points at `len` along
// `d`, radius `rad`. Side faces only (open mouth), which is what both the
// fixture barrel and the beam want.
void add_cone(cvc::geometry &g, const double apex[3], const double d[3], double len, double rad,
              int seg, const double rgb[3]) {
  double u[3], v[3];
  frame_from_axis(d, u, v);
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &tris = g.tris();
  const auto base = static_cast<cvc::geometry::index_t>(pts.size());
  pts.push_back({apex[0], apex[1], apex[2]});
  cols.push_back({rgb[0], rgb[1], rgb[2]});
  for (int i = 0; i < seg; ++i) {
    const double a = 2.0 * 3.14159265358979323846 * i / seg;
    const double ca = std::cos(a) * rad, sa = std::sin(a) * rad;
    pts.push_back({apex[0] + d[0] * len + u[0] * ca + v[0] * sa,
                   apex[1] + d[1] * len + u[1] * ca + v[1] * sa,
                   apex[2] + d[2] * len + u[2] * ca + v[2] * sa});
    cols.push_back({rgb[0], rgb[1], rgb[2]});
  }
  for (int i = 0; i < seg; ++i)
    tris.push_back({base, static_cast<cvc::geometry::index_t>(base + 1 + i),
                    static_cast<cvc::geometry::index_t>(base + 1 + (i + 1) % seg)});
}

// A small sphere for the bulb.
void add_sphere(cvc::geometry &g, const double c[3], double r, const double rgb[3], int nu = 10,
                int nv = 8) {
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &tris = g.tris();
  const auto base = static_cast<cvc::geometry::index_t>(pts.size());
  const double PI = 3.14159265358979323846;
  for (int j = 0; j <= nv; ++j) {
    const double th = PI * j / nv;
    for (int i = 0; i <= nu; ++i) {
      const double ph = 2.0 * PI * i / nu;
      pts.push_back({c[0] + r * std::sin(th) * std::cos(ph), c[1] + r * std::sin(th) * std::sin(ph),
                     c[2] + r * std::cos(th)});
      cols.push_back({rgb[0], rgb[1], rgb[2]});
    }
  }
  for (int j = 0; j < nv; ++j)
    for (int i = 0; i < nu; ++i) {
      const auto a = static_cast<cvc::geometry::index_t>(base + j * (nu + 1) + i);
      const auto b = static_cast<cvc::geometry::index_t>(a + nu + 1);
      tris.push_back({a, b, static_cast<cvc::geometry::index_t>(a + 1)});
      tris.push_back({static_cast<cvc::geometry::index_t>(a + 1), b,
                      static_cast<cvc::geometry::index_t>(b + 1)});
    }
}

GizmoMeshes build_gizmos(const std::vector<Placed> &lights) {
  GizmoMeshes gm;
  const double black[3] = {0.04, 0.04, 0.05};
  for (const auto &L : lights) {
    double d[3] = {L.tx - L.px, L.ty - L.py, L.tz - L.pz};
    const double len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (len < 1e-9)
      continue;
    for (int i = 0; i < 3; ++i)
      d[i] /= len;

    // Fixture: a short black barrel, WIDE END TOWARD THE SUBJECT, so it reads as
    // a stage lamp pointing where the light goes. Sized off the throw distance
    // so it stays legible at any stage scale.
    const double fl = 0.06 * len, fr = 0.028 * len;
    const double back[3] = {L.px - d[0] * fl, L.py - d[1] * fl, L.pz - d[2] * fl};
    add_cone(gm.fixture, back, d, fl * 2.0, fr, 20, black);

    // Bulb: a small emissive sphere at the fixture mouth, in the light's colour.
    const double bulbC[3] = {L.px + d[0] * fl * 0.35, L.py + d[1] * fl * 0.35,
                             L.pz + d[2] * fl * 0.35};
    const double bulbRgb[3] = {L.r, L.g, L.b};
    add_sphere(gm.bulb, bulbC, fr * 0.5, bulbRgb, 10, 8);

    // Beam: the cone itself, apex at the bulb, out to the target plane. Its
    // radius is len*tan(cone) — the actual shadow-map frustum.
    const double rad = len * std::tan(L.cone * kDeg);
    add_cone(gm.beam, bulbC, d, len, rad, 32, bulbRgb);
  }
  return gm;
}
} // namespace

void StageLighting::apply() {
  Impl &s = *m_impl;
  if (!s.scene)
    return;
  // RE-ENTRY GUARD. Each rig light is a node now, and configuring one writes its
  // state, which notifies the scene, which can land back here mid-rebuild —
  // while `nodes` is being mutated and half the rig has been removed. That is a
  // use-after-free waiting to happen (it corrupted the heap outright the first
  // time). One rebuild at a time.
  if (s.applying)
    return;
  s.applying = true;
  struct Unset {
    bool *f;
    ~Unset() { *f = false; }
  } unset{&s.applying};

  // ONE light rebuild for the whole rig. Without this, replacing ~14 lights
  // costs ~14x14 renderer-wide light rebuilds and a shadow re-bake per caster
  // per rebuild — seconds of freeze on every slider move.
  s.scene->beginLightBatch();
  struct BatchEnd {
    SceneGraph *sg;
    ~BatchEnd() { sg->endLightBatch(); }
  } batchEnd{s.scene};

  // Do NOT destroy and recreate the light nodes each rebuild. Node creation
  // (state seeding, registration, bounds tracking, child-list touch) is orders
  // of magnitude dearer than updating one — measured at 59 ms per edit, which
  // would put the UI freeze straight back. Keep the previous set, reuse by name,
  // and remove only what the new rig no longer uses.
  std::vector<std::string> previous;
  previous.swap(s.nodes);
  s.placed.clear();
  if (!s.on) {
    s.scene->removeGraphics("stage_gizmos");
    return;
  }

  const double R = std::max(1e-3, s.radius);
  // Throw distance: far enough to clear the subject, close enough that the cone
  // still lands mostly on it. 2.2 radii is a good compromise for the default
  // 32-degree key (tan(32) * 2.2R ~= 1.4R of coverage).
  const double dist = 2.2 * R;

  // Warm key / cool fill — the standard filmic split, scaled by `warmth`.
  const double w = s.warmth;
  const double keyR = 1.0, keyG = 1.0 - 0.10 * w, keyB = 1.0 - 0.26 * w;
  const double filR = 1.0 - 0.22 * w, filG = 1.0 - 0.08 * w, filB = 1.0;

  // Create one rig light as a scene-graph node. `name` is stable per role, so
  // the hierarchy reads "stage_key", "stage_wash_2" rather than opaque ids.
  auto makeLight = [&](const std::string &name, cvc::gl::LightNode::Kind kind, double px, double py,
                       double pz, double tx, double ty, double tz, double cone, double r, double g,
                       double b, double intensity) {
    // Reuse the node if this role already exists; only create when it does not.
    auto ln = std::dynamic_pointer_cast<cvc::gl::LightNode>(s.scene->getGraphics(name));
    if (!ln)
      ln = s.scene->addLight(name);
    if (!ln)
      return;
    ln->setKind(kind);
    ln->setPosition(px, py, pz);
    ln->setTarget(tx, ty, tz);
    ln->setCone(cone);
    ln->setColor(r, g, b);
    ln->setIntensity(intensity);
    s.nodes.push_back(name);
    s.placed.push_back({px, py, pz, tx, ty, tz, cone, r, g, b});
  };

  auto spot = [&](const std::string &name, double az, double el, double cone, double intensity,
                  double r, double g, double b) {
    if (intensity <= 0.0)
      return;
    double x, y, z;
    orbitPoint(s.cx, s.cy, s.cz, dist, az, el, x, y, z);
    makeLight(name, cvc::gl::LightNode::Kind::Spot, x, y, z, s.cx, s.cy, s.cz, cone, r, g, b,
              intensity);
  };

  // KEY — the light you actually read the form by.
  spot("stage_key", s.keyAz, s.keyEl, s.keyCone, s.keyI, keyR, keyG, keyB);
  // FILL — opposite side, lower, wider and softer.
  spot("stage_fill", s.keyAz + 130.0, std::max(12.0, s.keyEl - 16.0),
       std::min(60.0, s.keyCone * 1.5), s.fillI, filR, filG, filB);
  // BACK — behind the subject and high, for the separating rim.
  spot("stage_back", s.keyAz + 180.0, std::min(72.0, s.keyEl + 26.0), s.keyCone * 1.1, s.backI, 1.0,
       1.0, 1.0);

  // WASH — a ring of downlights over the acting area. Wide cones, aimed at the
  // stage centre, so nothing goes black when it walks off the key.
  if (s.washCount > 0 && s.washI > 0.0) {
    const double h = s.cz + s.washHeight * R;
    const double ringR = 0.75 * R;
    const double per = s.washI / static_cast<double>(s.washCount);
    for (int i = 0; i < s.washCount; ++i) {
      const double a = 360.0 * i / s.washCount;
      const double x = s.cx + ringR * std::sin(a * kDeg);
      const double y = s.cy - ringR * std::cos(a * kDeg);
      makeLight("stage_wash_" + std::to_string(i), cvc::gl::LightNode::Kind::Spot, x, y, h, s.cx,
                s.cy, s.cz, 55.0, 1.0, 1.0, 1.0, per);
    }
  }

  // ENVIRONMENT — one wide, shadow-free fill from high above, so water and
  // scenery outside the cones still receive light (and so still show specular).
  if (s.envI > 0.0)
    makeLight("stage_env", cvc::gl::LightNode::Kind::Fill, s.cx, s.cy, s.cz + 3.0 * R, s.cx, s.cy,
              s.cz, 95.0, 0.92, 0.95, 1.0, s.envI);

  // SPECIALS — placed and aimed by the caller.
  int specialIdx = -1;
  for (const auto &sp : s.specials) {
    ++specialIdx;
    if (!sp.live || sp.intensity <= 0.0)
      continue;
    makeLight("stage_special_" + std::to_string(specialIdx), cvc::gl::LightNode::Kind::Spot, sp.x,
              sp.y, sp.z, sp.tx, sp.ty, sp.tz, sp.cone, sp.r, sp.g, sp.b, sp.intensity);
  }

  // Retire roles the new rig no longer has (e.g. the wash shrank).
  for (const auto &old : previous)
    if (std::find(s.nodes.begin(), s.nodes.end(), old) == s.nodes.end())
      s.scene->removeGraphics(old);

  // ---- gizmos ---------------------------------------------------------------
  const char *kGizmoNodes[] = {"stage_gizmo_fixtures", "stage_gizmo_bulbs", "stage_gizmo_beams"};
  if (!s.gizmos) {
    for (const char *n : kGizmoNodes)
      s.scene->removeGraphics(n);
    return;
  }
  GizmoMeshes gm = build_gizmos(s.placed);

  // Fixture: matte black housing. Ambient-lit only, so it stays black instead of
  // being blown out by the very rig it represents.
  if (auto n = std::dynamic_pointer_cast<GeometryNode>(
          s.scene->addGraphics(kGizmoNodes[0], gm.fixture))) {
    n->setUseSingleColor(false);
    n->setAmbient(0.85);
    n->setDiffuse(0.15);
    n->setSpecular(0.0);
    n->setOpacity(0.99); // translucent bucket -> excluded from the shadow bake
  }
  // Bulb: reads as the source, so fully emissive in the light's own colour.
  if (auto n =
          std::dynamic_pointer_cast<GeometryNode>(s.scene->addGraphics(kGizmoNodes[1], gm.bulb))) {
    n->setUseSingleColor(false);
    n->setAmbient(1.0);
    n->setDiffuse(0.0);
    n->setSpecular(0.0);
    n->setOpacity(0.99);
  }
  // Beam: the shadow-map frustum, drawn see-through so it does not hide the
  // shadows it is there to explain.
  if (auto n =
          std::dynamic_pointer_cast<GeometryNode>(s.scene->addGraphics(kGizmoNodes[2], gm.beam))) {
    n->setUseSingleColor(false);
    n->setAmbient(1.0);
    n->setDiffuse(0.0);
    n->setSpecular(0.0);
    n->setOpacity(s.beamAlpha);
  }
}

} // namespace gl
} // namespace cvc
