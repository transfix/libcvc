#ifndef CVC_GL_STAGE_LIGHTING_H
#define CVC_GL_STAGE_LIGHTING_H

#include <cvc/core/state_object.h>
#include <memory>
#include <string>
#include <vector>

class SceneGraph;

namespace cvc {
class app;

namespace gl {

// ----------------
// StageLighting
// ----------------
// Light a scene the way a stage or a film set is lit, instead of hanging one
// "sun" in the sky and hoping.
//
//     cvc::gl::StageLighting rig(sg);            // scene's own state tree
//     rig.setStage(0, 0, 0, 60.0);               // where the action is, how wide
//     rig.apply();                               // build the lights
//
// WHY THIS IS NOT JUST A METAPHOR — it is what makes shadows sharp.
// VTK bakes a DIRECTIONAL light's shadow map with a parallel projection fitted
// to the whole scene bounding box (vtkShadowMapBakerPass::BuildCameraLight sets
// parallelScale = max(bboxWidth, bboxHeight)/2). One shadow map is therefore
// stretched over everything present — so a scene that also contains a sea plane,
// a sky dome or a distant billboard spends nearly all of its shadow texels on
// empty space, and the shadows that matter get a handful of texels and read as
// mush. Adding scenery makes existing shadows worse, which is a deeply
// unintuitive failure mode.
//
// A SPOT light is baked with a PERSPECTIVE projection whose view angle is the
// light's cone, so its shadow texels land only inside the cone. Aim a 35-degree
// spot at the subject and essentially the whole map is spent on the subject.
// That is the entire trick: a rig of aimed spots is both how lighting is
// actually designed AND how you get shadow-map resolution where it counts.
//
// THE RIG, deliberately small — four named roles, the ones every lighting
// tutorial starts from:
//   KEY   the main light. Off to one side and above; makes the form and casts
//         the shadow you actually read.
//   FILL  softer, opposite the key, lifts the shadow side so it is not black.
//         No shadow of its own (a second shadow from the fill looks wrong).
//   BACK  behind the subject, high, aimed forward-down: separates subject from
//         background with a rim. Also called a hair or kicker light.
//   WASH  an overhead ring of downlights covering the acting area, so nothing
//         falls into a hole when it moves off the key.
// plus SPECIALS — any number of extra spots you place and aim yourself, for
// "highlight that object".
//
// EVERYTHING IS cvc::state, like CameraController and TouchGestures: settings
// live under "<scene prefix>.lighting" and are two-way bound, so a slider, a
// script and a replicated peer all drive the same rig. Keys include
// key_intensity, key_azimuth, key_elevation, key_cone, fill_intensity,
// back_intensity, wash_intensity, wash_count, wash_height, ambient,
// stage_x/y/z, stage_radius, warm_key, enabled.
class StageLighting : public cvc::state_object<StageLighting> {
public:
  // Canonical: state at "<scene prefix>.lighting", lights built into `scene`.
  explicit StageLighting(SceneGraph &scene);
  // Low-level: explicit app context + full state path.
  StageLighting(cvc::app &ctx, const std::string &statePath, SceneGraph *scene);
  ~StageLighting();

  StageLighting(const StageLighting &) = delete;
  StageLighting &operator=(const StageLighting &) = delete;

  // The canonical state path for a scene's lighting rig.
  static std::string sceneStatePath(const std::string &scenePrefix);

  // The app and state path this rig lives in — what a bound UI needs in order to
  // address the same state keys (see cvc::gl::ui::StageLightingPanel).
  cvc::app &appContext() const;
  const std::string &statePath() const;

  // ---- the acting area -----------------------------------------------------
  // Where the rig points and how wide it spreads. Every light is placed
  // relative to this, so moving the stage moves the whole rig coherently.
  // `radius` should be roughly the subject's extent, NOT the whole scene's:
  // it sets the cone widths, and a cone sized to the scene is a cone sized to
  // nothing (see the shadow note above).
  void setStage(double cx, double cy, double cz, double radius);
  void stage(double &cx, double &cy, double &cz, double &radius) const;

  // Frame the rig on a bounding box — convenience for "light what I just built".
  // Uses the box centre and half-diagonal of its FOOTPRINT, so a tall thin scene
  // does not produce an absurdly wide cone.
  void frameBounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ);

  // ---- presets -------------------------------------------------------------
  enum class Preset {
    ThreePoint, // key + fill + back. The default; what you want most of the time.
    Overhead,   // wash-dominant, gentle key. Even, museum-ish, few hard shadows.
    Dramatic,   // hard narrow key, minimal fill, strong back. High contrast.
    Flat,       // wash only, no key. For reading geometry rather than admiring it.
  };
  void applyPreset(Preset p);
  static const char *presetName(Preset p);

  // ---- roles ---------------------------------------------------------------
  // Intensities are multipliers; 0 switches a role off entirely (its lights are
  // removed, so an unused role costs nothing in the shadow bake).
  void setKey(double intensity, double azimuthDeg, double elevationDeg, double coneDeg);
  void setFill(double intensity);
  void setBack(double intensity);
  // count = number of downlights in the overhead ring (0 disables the wash).
  void setWash(double intensity, int count, double heightScale);
  // A flat term added to every material so shadowed sides are not pure black.
  // This is the renderer's ambient, not a light.
  void setAmbient(double a);
  // Tint the key warm and the fill cool, the usual filmic split. 0 = neutral.
  void setWarmth(double amount);

  // ---- specials ------------------------------------------------------------
  // Place a spot yourself and aim it. Returns an index you can move later.
  int addSpecial(double x, double y, double z, double tx, double ty, double tz,
                 double coneDeg = 20.0, double intensity = 1.0, double r = 1.0, double g = 1.0,
                 double b = 1.0);
  void moveSpecial(int index, double x, double y, double z);
  void aimSpecial(int index, double tx, double ty, double tz);
  void removeSpecial(int index);
  int specialCount() const;

  // ---- build ---------------------------------------------------------------
  // Rebuild every light from the current settings. Called for you when a setter
  // or a state write changes something; call it directly after a batch of edits.
  void apply();

  // Master switch. When off the rig removes its lights and leaves the scene's
  // other lights (if any) alone.
  void setEnabled(bool on);
  bool enabled() const;

  // How many shadow-casting lights the rig currently has. Useful in a HUD: each
  // one costs a full scene depth re-render per bake.
  int shadowCasterCount() const;

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();
  void readAllFromState();

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_STAGE_LIGHTING_H
