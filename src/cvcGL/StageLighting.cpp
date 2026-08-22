// StageLighting — a key/fill/back/wash rig of aimed spot lights.
// See the header for why aimed spots (not a directional sun) are what make the
// shadow map sharp.

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/StageLighting.h>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace gl {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;

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

  std::vector<Special> specials;
  std::vector<int> ids; // every light this rig currently owns
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

void StageLighting::apply() {
  Impl &s = *m_impl;
  if (!s.scene)
    return;

  for (int id : s.ids)
    s.scene->removeLight(id);
  s.ids.clear();
  if (!s.on)
    return;

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
    }
  }

  // SPECIALS — placed and aimed by the caller.
  for (const auto &sp : s.specials) {
    if (!sp.live || sp.intensity <= 0.0)
      continue;
    s.ids.push_back(s.scene->addSpotLight(sp.x, sp.y, sp.z, sp.tx, sp.ty, sp.tz, sp.cone, sp.r,
                                          sp.g, sp.b, sp.intensity));
  }
}

} // namespace gl
} // namespace cvc
