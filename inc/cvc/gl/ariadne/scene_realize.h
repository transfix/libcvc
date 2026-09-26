#ifndef CVC_GL_ARIADNE_SCENE_REALIZE_H
#define CVC_GL_ARIADNE_SCENE_REALIZE_H

// Realize a parsed Ariadne Scene (roadmap §9) into a live cvc::gl::SceneGraph.
// The backend-neutral Scene spec (cvc::ariadne::Scene, inc/cvc/ariadne/scene.h) is
// GL-agnostic data; this is the cvcGL side that turns it into real GraphicsNodes.
// A geometry node loads its source and gets its transform + single-colour material
// + initial visibility; a group node is an empty hierarchy node. Because every
// SceneGraph node is a cvc::state_object (§9.1), creating a node writes its state
// subtree — so a widget or expression can then drive its props with no extra glue.

#include <string>
#include <vector>

namespace cvc {
namespace ariadne {
struct Scene;
struct SceneNode;
} // namespace ariadne
namespace gl {
class SceneGraph;

namespace ariadne {

// Create/configure SceneGraph nodes from `scene`. Returns the names of the
// realized top-level nodes. When `warnings` is non-null, non-fatal issues (a
// missing source, an unreadable file, a not-yet-supported node type) are appended
// rather than thrown. Dynamic state-bound visibility (`visible: <path>`) is left
// to the caller to sync each frame / via a state watch — a follow-up.
std::vector<std::string> realize_scene(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                                       std::vector<std::string> *warnings = nullptr);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_SCENE_REALIZE_H
