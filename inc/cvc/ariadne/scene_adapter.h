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

#include <functional>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

struct Scene;      // the parsed scene (scene.h)
struct LoadResult; // the loaded document (loader.h)
class SceneWorld;  // the scene's world (scene_world.h)

class SceneAdapter {
public:
  virtual ~SceneAdapter() = default;

  // Check the document's declared scene-node customs (its customs: block) before realize(). False
  // = a REQUIRED one is not registered here, so AppRuntime::load skips the scene. Messages are
  // appended to `errors` if non-null. The default has nothing to check.
  virtual bool verify(const LoadResult &loaded, std::vector<std::string> *errors) {
    (void)loaded;
    (void)errors;
    return true;
  }

  // How the document's read lane counts an expression (Runtime::eval_count). A scene node's
  // `repeat: (expr)` is counted with it each frame, as a widget's repeat: is. AppRuntime sets it
  // when it is made and clears it (an empty function) when it goes, so an adapter that keeps it
  // never calls into a runtime that has gone. The default ignores it.
  using CountFn = std::function<int(const std::string &expr)>;
  virtual void set_count_fn(CountFn count) { (void)count; }

  // Whether this adapter's renderer compiles the GLSL a document hands its scene (a node's shader:
  // block: presets and vertex / fragment splices). That is the gl.scene.shaders capability, and it
  // is the adapter's, not the backend's: cvcGL's terminal adapter renders offscreen through GL, so
  // the scene runs its shaders while the terminal backend draws none (Capabilities::glsl). The
  // default is false, for an adapter that realizes the scene some other way.
  virtual bool runs_shaders() const { return false; }

  // Realize the parsed `scene` under state `prefix` (and frame the view's camera to it,
  // when the adapter has one). Non-fatal messages are appended to `warns` if non-null.
  // Called from AppRuntime::load() when the document carries a scene.
  virtual void realize(const Scene &scene, const std::string &prefix,
                       std::vector<std::string> *warns) = 0;

  // Realize the document's other scenes (`scenes:`), each a scene of its own (its own nodes,
  // lights, shadows and background) that viewports name. Called from AppRuntime::load() after
  // realize() (or after no_scene() when the document has only these). clear() takes them down too.
  // The default cannot draw them and says so.
  virtual void realize_scenes(const std::vector<Scene> &scenes, const std::string &prefix,
                              std::vector<std::string> *warns) {
    (void)prefix;
    if (warns && !scenes.empty())
      warns->push_back("ari: this host draws one scene: the document's scenes: are ignored");
  }

  // docs/roadmap/ARIADNE_SHELL.md 4.3 (SH3): a mounted document's scene. Realize `scene` (its
  // `scene:`) and `scenes` (its `scenes:`) as a scene of their own, `id`, whose state is under
  // `sub_prefix` (the mount's): its nodes at <sub_prefix>.graphics.root.children.<node>, its
  // physics at <sub_prefix>.physics, its scenes: at <sub_prefix>.scenes.<id>, its camera at
  // <sub_prefix>.viewers.main.camera, and every relative bind in it resolved against
  // <sub_prefix>, as it did when the document ran alone. Its world steps on a clock of its own
  // (cvc::scene_clock at <sub_prefix>), at its `clock: hz` or at the rate of the clock above it.
  // AppRuntime calls it from drain() when a mount with a scene opens (a load: as the document
  // runs, a mounts: entry as it opens), and clear_mount(id) when it closes; clear() takes every
  // mount's down too. False when it was not realized: the default cannot draw it and says so.
  virtual bool realize_mount(const std::string &id, const Scene &scene,
                             const std::vector<Scene> &scenes, const std::string &sub_prefix,
                             std::vector<std::string> *warns) {
    (void)id;
    (void)scene;
    (void)scenes;
    if (warns)
      warns->push_back("ari: this host draws one scene: the scene of the document mounted at '" +
                       sub_prefix + "' is ignored");
    return false;
  }
  virtual void clear_mount(const std::string &id) { (void)id; }
  // Whether mount `id`'s world holds (stops stepping) while its scene is drawn nowhere: its scene
  // window closed or collapsed and no other view of it drawn (ARIADNE_SHELL.md 4.4). False, the
  // default: it keeps stepping. AppRuntime sets it as it changes.
  virtual void hold_mount(const std::string &id, bool hold) {
    (void)id;
    (void)hold;
  }

  // Take down what the last realize() built (its nodes, lights, physics, and the view properties
  // it set), and every mount's scene, so the next document starts on an empty stage.
  // AppRuntime::load() calls it before each load, when a host switches documents as much as on a
  // hot reload. The default keeps everything, which is right only for an adapter that never
  // realizes.
  virtual void clear() {}

  // The document just loaded has no `scene:` (called after clear()). It is its widgets alone, so a
  // host that draws a scene of its own by default (arigl's grid, axes and bounds box) can hide it
  // rather than show it behind them. The default does nothing.
  virtual void no_scene() {}

  // Mirror bound `visible:` state onto the realized scene's nodes. Called once per
  // AppRuntime::drain(). A no-op until something has been realized.
  virtual void sync_visibility(cvc::app &app) = 0;

  // Rebuild the realized scene's world as realize() made it (sim.restart; SceneWorld::restart),
  // with the bodies it draws bound again. AppRuntime::restart_world() has reset the clock to tick
  // 0 first. False when the adapter has no world to rebuild, which is the default.
  virtual bool restart(std::vector<std::string> *warns) {
    (void)warns;
    return false;
  }

  // The world the realized scene runs beside its drawing (its bodies, its vehicles), for a host
  // that reaches into it (AppRuntime::running_world). Null when there is none, which is the
  // default.
  virtual SceneWorld *scene_world() const { return nullptr; }

  // Per-frame scene service (volume-render/slice tickers, …). Called once per
  // AppRuntime::drain(), after sync_visibility(). A no-op until something has been
  // realized.
  virtual void tick() = 0;
};

} // namespace ariadne
} // namespace cvc
