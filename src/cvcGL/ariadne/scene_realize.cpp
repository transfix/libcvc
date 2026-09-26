#include <cvc/gl/ariadne/scene_realize.h>

#include <exception>
#include <memory>

#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

// Apply the shared GraphicsNode transform. Material is GeometryNode-only, handled by
// the caller before this; visibility is handled by apply_visibility below.
void apply_transform(GraphicsNode &node, const cvc::ariadne::SceneNode &n) {
  if (n.has_transform) {
    node.setPosition(n.position[0], n.position[1], n.position[2]);
    node.setRotation(n.rotation[0], n.rotation[1], n.rotation[2]);
    node.setScale(n.scale[0], n.scale[1], n.scale[2]);
  }
}

// Set the node's initial visibility, and — for a bound node — record a binding the
// host will poll. Visibility is driven through the node's OWN `.visible` state key
// (not a direct setVisible), so cvc::state stays authoritative (§9.1) and the node's
// state_object machinery performs the actor flip. `bind_prefix` matches the widget
// Runtime's prefix so a `visible:`/`bind:` pair on the same path share one key.
void apply_visibility(GraphicsNode &node, const cvc::ariadne::SceneNode &n,
                      cvc::app &app, const std::string &bind_prefix,
                      std::vector<cvc::ariadne::SceneVisibilityBinding> &binds) {
  const std::string target = node.stateName("visible");
  if (n.visible_bind.empty()) {
    // Literal `visible: true|false` (or default): write the key once.
    cvc::ariadne::write<int>(app, target, n.visible_default ? 1 : 0);
    return;
  }
  // Bound `visible: <path>` — resolve, seed the node from the live value now, and
  // record the binding for per-frame sync.
  const std::string source = cvc::ariadne::resolve_bind(bind_prefix, n.visible_bind);
  const int v0 = cvc::ariadne::read_or_seed<int>(app, source, n.visible_default ? 1 : 0);
  cvc::ariadne::write<int>(app, target, v0);
  binds.push_back({source, target, n.visible_default});
}

void realize_node(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                  const std::string &bind_prefix, RealizedScene &out,
                  std::vector<std::string> *warnings) {
  std::shared_ptr<GraphicsNode> node;

  if (n.type == "geometry") {
    if (n.source_file.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (geometry) has no source.file");
      return;
    }
    cvc::geometry geom;
    try {
      geom = cvc::read_geometry(n.source_file);
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
      return;
    }
    node = sg.addGraphics(n.id, geom);
    if (auto g = std::dynamic_pointer_cast<GeometryNode>(node)) {
      if (n.has_material) {
        g->setUseSingleColor(true);
        g->setColor(n.color[0], n.color[1], n.color[2]);
        g->setAmbient(n.ambient);
        g->setDiffuse(n.diffuse);
      }
    }
  } else if (n.type == "group") {
    node = sg.addGraphics(n.id); // empty hierarchy node
  } else {
    // volume/volren/volslice/light realization is a follow-up (§9); the spec still
    // parses and round-trips so the document is valid — we just don't build it yet.
    warn(warnings, "ari: scene node '" + n.id + "' type '" + n.type +
                       "' not realized yet (geometry/group only)");
    return;
  }

  if (!node)
    return;
  apply_transform(*node, n);
  apply_visibility(*node, n, sg.appContext(), bind_prefix, out.visibility);

  // Children are realized as top-level nodes for now; true parent nesting (a
  // <parent>.children.<child> path) rides on the §11 path binding — a follow-up.
  for (const auto &c : n.children)
    realize_node(sg, c, bind_prefix, out, warnings);
}

} // namespace

RealizedScene realize_scene(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                            const std::string &bind_prefix,
                            std::vector<std::string> *warnings) {
  RealizedScene out;
  out.created.reserve(scene.nodes.size());
  for (const auto &n : scene.nodes) {
    realize_node(sg, n, bind_prefix, out, warnings);
    out.created.push_back(n.id);
  }
  // Lights + shadows realization (LightNode / StageLighting) is a follow-up (§9).
  if (!scene.lights.empty())
    warn(warnings, "ari: scene lights parsed but not realized yet");
  return out;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
