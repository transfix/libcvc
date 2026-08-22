// StageLighting — a key/fill/back/wash rig of aimed spot lights.
// See the header for why aimed spots (not a directional sun) are what make the
// shadow map sharp.

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
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
  double warmth = 0.35;

  bool gizmos = false;
  std::vector<Special> specials;
  std::vector<int> ids; // every light this rig currently owns

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
    for (int id : m_impl->ids)
      m_impl->scene->removeLight(id);
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
  getState("warm_key").value(s.warmth);
  getState("show_gizmos").value(s.gizmos ? 1 : 0);
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
    s.warmth = getState("warm_key").value<double>();
    s.gizmos = getState("show_gizmos").value<int>() != 0;
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

namespace {
// One merged LINES mesh showing every light: a cross at the fixture, a line
// down its aim, and the CONE it throws — which is the same frustum VTK bakes
// that light's shadow map with, so the wireframe is literally the shadow-map
// volume.
cvc::geometry build_gizmos(const std::vector<Placed> &lights) {
  cvc::geometry g;
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &lines = g.lines();
  auto add = [&](double x, double y, double z, double r, double gg, double b) {
    pts.push_back({x, y, z});
    cols.push_back({r, gg, b});
    return static_cast<cvc::geometry::index_t>(pts.size() - 1);
  };
  auto seg = [&](cvc::geometry::index_t a, cvc::geometry::index_t b) { lines.push_back({a, b}); };

  for (const auto &L : lights) {
    double ax = L.tx - L.px, ay = L.ty - L.py, az = L.tz - L.pz;
    const double len = std::sqrt(ax * ax + ay * ay + az * az);
    if (len < 1e-9)
      continue;
    ax /= len;
    ay /= len;
    az /= len;
    // Any two vectors perpendicular to the aim, for the cone's rim.
    double ux = 0, uy = 0, uz = 1;
    if (std::abs(az) > 0.9) {
      ux = 1;
      uz = 0;
    }
    double sx = uy * az - uz * ay, sy = uz * ax - ux * az, sz = ux * ay - uy * ax;
    double sl = std::sqrt(sx * sx + sy * sy + sz * sz);
    sx /= sl;
    sy /= sl;
    sz /= sl;
    double tx2 = ay * sz - az * sy, ty2 = az * sx - ax * sz, tz2 = ax * sy - ay * sx;

    // The fixture: a small cross so the light reads as an object in the scene.
    const double m = 0.02 * len;
    const auto c0 = add(L.px, L.py, L.pz, L.r, L.g, L.b);
    seg(c0, add(L.px + m, L.py, L.pz, L.r, L.g, L.b));
    seg(c0, add(L.px - m, L.py, L.pz, L.r, L.g, L.b));
    seg(c0, add(L.px, L.py + m, L.pz, L.r, L.g, L.b));
    seg(c0, add(L.px, L.py - m, L.pz, L.r, L.g, L.b));
    seg(c0, add(L.px, L.py, L.pz + m, L.r, L.g, L.b));
    seg(c0, add(L.px, L.py, L.pz - m, L.r, L.g, L.b));
    // The aim.
    seg(c0, add(L.tx, L.ty, L.tz, L.r * 0.6, L.g * 0.6, L.b * 0.6));

    // The cone == the shadow-map frustum. Rim circle at the target plane, plus
    // spokes from the fixture so the volume reads in 3-D.
    const double rad = len * std::tan(L.cone * kDeg);
    const int SEG = 28;
    cvc::geometry::index_t first = 0, prev = 0;
    for (int i = 0; i < SEG; ++i) {
      const double a = 2.0 * 3.14159265358979323846 * i / SEG;
      const double ca = std::cos(a) * rad, sa = std::sin(a) * rad;
      const auto idx = add(L.tx + sx * ca + tx2 * sa, L.ty + sy * ca + ty2 * sa,
                           L.tz + sz * ca + tz2 * sa, L.r, L.g, L.b);
      if (i == 0)
        first = idx;
      else
        seg(prev, idx);
      if (i % 7 == 0)
        seg(c0, idx); // four spokes
      prev = idx;
    }
    seg(prev, first);
  }
  return g;
}
} // namespace

void StageLighting::apply() {
  Impl &s = *m_impl;
  if (!s.scene)
    return;

  for (int id : s.ids)
    s.scene->removeLight(id);
  s.ids.clear();
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

  auto spot = [&](double az, double el, double cone, double intensity, double r, double g,
                  double b) {
    if (intensity <= 0.0)
      return;
    double x, y, z;
    orbitPoint(s.cx, s.cy, s.cz, dist, az, el, x, y, z);
    s.ids.push_back(s.scene->addSpotLight(x, y, z, s.cx, s.cy, s.cz, cone, r, g, b, intensity));
  };

  // KEY — the light you actually read the form by.
  spot(s.keyAz, s.keyEl, s.keyCone, s.keyI, keyR, keyG, keyB);
  // FILL — opposite side, lower, wider and softer.
  spot(s.keyAz + 130.0, std::max(12.0, s.keyEl - 16.0), std::min(60.0, s.keyCone * 1.5), s.fillI,
       filR, filG, filB);
  // BACK — behind the subject and high, for the separating rim.
  spot(s.keyAz + 180.0, std::min(72.0, s.keyEl + 26.0), s.keyCone * 1.1, s.backI, 1.0, 1.0, 1.0);

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
      s.ids.push_back(s.scene->addSpotLight(x, y, h, s.cx, s.cy, s.cz, 55.0, 1.0, 1.0, 1.0, per));
      s.placed.push_back({x, y, h, s.cx, s.cy, s.cz, 55.0, 0.65, 0.72, 0.80});
    }
  }

  // SPECIALS — placed and aimed by the caller.
  for (const auto &sp : s.specials) {
    if (!sp.live || sp.intensity <= 0.0)
      continue;
    s.ids.push_back(s.scene->addSpotLight(sp.x, sp.y, sp.z, sp.tx, sp.ty, sp.tz, sp.cone, sp.r,
                                          sp.g, sp.b, sp.intensity));
    s.placed.push_back({sp.x, sp.y, sp.z, sp.tx, sp.ty, sp.tz, sp.cone, sp.r, sp.g, sp.b});
  }

  // ---- gizmos ---------------------------------------------------------------
  if (!s.gizmos) {
    s.scene->removeGraphics("stage_gizmos");
    return;
  }
  auto node = std::dynamic_pointer_cast<GeometryNode>(
      s.scene->addGraphics("stage_gizmos", build_gizmos(s.placed)));
  if (node) {
    node->setRenderMode(GeometryRenderMode::LINES);
    node->setUseSingleColor(false);
    node->setLineWidth(1.5);
    node->setAmbient(1.0); // flat overlay colour, never shaded by the rig itself
    node->setDiffuse(0.0);
    node->setSpecular(0.0);
    // Just under opaque puts the gizmos in the TRANSLUCENT bucket, which the
    // opaque shadow-map bake skips — so drawing the debug overlay cannot alter
    // the shadows you are using it to debug.
    node->setOpacity(0.99);
  }
}

} // namespace gl
} // namespace cvc
