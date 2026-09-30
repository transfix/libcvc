// SceneAdapter — the 3D-scene seam for a cvc::ariadne::AppRuntime.
//
// Ariadne's widget surface is delegated to a Backend (ImGui-over-VTK, FTXUI, …); its
// optional 3D scene is delegated the same way, to a SceneAdapter. This keeps the
// backend-neutral AppRuntime free of any renderer: only the concrete adapter (e.g.
// cvcGL's, which drives realize_scene / tick_scene over a SceneGraph) knows how to turn
// a parsed cvc::ariadne::Scene into something on screen.
//
// An AppRuntime may hold NO adapter (scene == nullptr): a terminal / no-scene backend
// then runs every non-scene lane unchanged, and a scene-bearing document loads with a
// warning and no realized scene. Every method takes only neutral cvc::ariadne types, so
// this header pulls in no GL/VTK.
#pragma once

#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

struct Scene; // the parsed scene (scene.h)

class SceneAdapter {
public:
  virtual ~SceneAdapter() = default;

  // Realize the parsed `scene` under state `prefix` (and frame the view's camera to it,
  // when the adapter has one). Non-fatal messages are appended to `warns` if non-null.
  // Called from AppRuntime::load() when the document carries a scene.
  virtual void realize(const Scene &scene, const std::string &prefix,
                       std::vector<std::string> *warns) = 0;

  // Mirror bound `visible:` state onto the realized scene's nodes. Called once per
  // AppRuntime::drain(). A no-op until something has been realized.
  virtual void sync_visibility(cvc::app &app) = 0;

  // Per-frame scene service (volume-render/slice tickers, …). Called once per
  // AppRuntime::drain(), after sync_visibility(). A no-op until something has been
  // realized.
  virtual void tick() = 0;
};

} // namespace ariadne
} // namespace cvc
