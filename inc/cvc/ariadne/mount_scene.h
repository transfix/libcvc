#ifndef CVC_ARIADNE_MOUNT_SCENE_H
#define CVC_ARIADNE_MOUNT_SCENE_H

// What a mounted document carries besides its widgets (docs/roadmap/ARIADNE_SHELL.md 4.3, SH3):
// its scene:, scenes:, data: and assets: (folded into each scene's assets). The loader parses them
// for a `load:` fragment and a `mounts:` entry alike and keeps them on the mount
// (Widget::mount_scene). Its keys: are its grants' (MountGrants::keys, grants.h, SH7). The Runtime
// binds its datasets at the mount's sub-prefix as the mount opens; a host with a SceneAdapter
// realizes its scene there (SceneAdapter::realize_mount), and the mount's scene window shows it.

#include <cvc/ariadne/data.h> // DataSpec
#include <cvc/ariadne/scene.h> // Scene
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

struct MountScene {
  Scene scene;                // its scene: (empty when it has none)
  std::vector<Scene> scenes;  // its scenes: entries, each with its id
  std::vector<DataSpec> data; // its data:, each relative to its own directory
  // Where its scene is drawn: "window" (its scene window, the default). The main view is SH6's.
  std::string view = "window";
  // Whether its world keeps stepping while its scene is drawn nowhere: "run", "hold", or "" (run,
  // unless the host holds every background world). A load:'s in_background:.
  std::string background;
  bool has_scene() const { return scene.any() || !scenes.empty(); }
};

// The id of the window a mount's scene is drawn in (its scene window): its geometry and open state
// are at <sub-prefix>.tree.ariadne_scene.*.
constexpr const char *kSceneWindowId = "ariadne_scene";

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_MOUNT_SCENE_H
