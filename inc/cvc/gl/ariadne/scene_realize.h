#ifndef CVC_GL_ARIADNE_SCENE_REALIZE_H
#define CVC_GL_ARIADNE_SCENE_REALIZE_H

// Realize a parsed Ariadne Scene (roadmap §9) into a live cvc::gl::SceneGraph.
// The backend-neutral Scene spec (cvc::ariadne::Scene, inc/cvc/ariadne/scene.h) is
// GL-agnostic data; this is the cvcGL side that turns it into real GraphicsNodes.
// A geometry node loads its source and gets its transform + single-colour material
// + initial visibility; a group node is an empty hierarchy node. Because every
// SceneGraph node is a cvc::state_object (§9.1), creating a node writes its state
// subtree — so a widget or expression can then drive its props with no extra glue.
//
// §9 increment 2 — dynamic `visible:` sync. When a node's visibility is bound to a
// state path (`visible: demo.show_mesh`), realize resolves that path (with the SAME
// rule the widget Runtime uses, so a checkbox on the same path and the node share
// one key), seeds the node's initial visibility from it, and records a
// cvc::ariadne::SceneVisibilityBinding. The host then calls
// cvc::ariadne::sync_scene_visibility(app, realized.visibility) each frame to mirror
// the live value into the node's own `.visible` key — the node's state_object
// machinery does the setVisible. The binding holds only state-path strings (no node
// pointer), so it can never dangle into a torn-down node.

#include <memory>
#include <string>
#include <vector>

#include <cvc/ariadne/bind.h>        // SceneVisibilityBinding
#include <cvc/gl/StageLighting.h>    // RealizedScene owns any StageLighting rigs

class vtkRenderer;

namespace cvc {
namespace ariadne {
struct Scene;
struct SceneNode;
} // namespace ariadne
namespace gl {
class SceneGraph;
class VolRenNode;
class VolSliceNode;

namespace ariadne {

// The result of realizing a Scene: the top-level node ids created, the visibility
// bindings the host must poll each frame (empty when no node uses `visible: <path>`),
// any StageLighting rigs, and the volume renderers that need per-frame ticking. The
// rigs are OWNED here because `~StageLighting` removes the rig's lights from the scene
// — so the RealizedScene must outlive the render loop (as the host already keeps it)
// or the lights vanish. The volren/volslice tickers are held as weak_ptr — the
// SceneGraph is the sole owner, so a torn-down node simply drops out of the tick
// rather than dangling (consistent with the string-only visibility bindings).
struct RealizedScene {
  std::vector<std::string> created;
  std::vector<cvc::ariadne::SceneVisibilityBinding> visibility;
  std::vector<std::unique_ptr<StageLighting>> rigs;
  std::vector<std::weak_ptr<VolRenNode>> volren_ticks;
  std::vector<std::weak_ptr<VolSliceNode>> volslice_ticks;
};

// Per-frame servicing for realized volren/volslice nodes (§9): each such node needs
// a tick() every frame or it renders nothing, and multiple volslice nodes need a
// back-to-front depth sort. Geometry/group/volume/light nodes need NO ticking. Call
// this each frame from the host render loop, BEFORE SceneRenderer::render(), on the
// owner/render thread, passing the scene's vtkRenderer (SceneRenderer::renderer()) —
// the renderer is used only for the multi-slice depth sort. A no-op when there are no
// volume renderers.
void tick_scene(RealizedScene &realized, vtkRenderer *renderer);

// Create/configure SceneGraph nodes from `scene`. `bind_prefix` is the SAME
// cvc::state prefix the widget Runtime was constructed with (typically
// sg.getStatePrefix()), so a scene `visible:` bind and a widget `bind:` on the same
// relative path resolve to one key. When `warnings` is non-null, non-fatal issues (a
// missing source, an unreadable file, a not-yet-supported node type) are appended
// rather than thrown.
RealizedScene realize_scene(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                            const std::string &bind_prefix,
                            std::vector<std::string> *warnings = nullptr);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_SCENE_REALIZE_H
