#include <cvc/gl/ariadne/scene_realize.h>

#include <exception>
#include <memory>

#include <cvc/ariadne/scene.h>
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

// Apply the shared GraphicsNode props (transform + initial visibility). Material is
// GeometryNode-only, handled by the caller before this.
void apply_common(GraphicsNode &node, const cvc::ariadne::SceneNode &n) {
  if (n.has_transform) {
    node.setPosition(n.position[0], n.position[1], n.position[2]);
    node.setRotation(n.rotation[0], n.rotation[1], n.rotation[2]);
    node.setScale(n.scale[0], n.scale[1], n.scale[2]);
  }
  // A state-bound visibility (`visible: <path>`) is synced by the caller; until the
  // first read, show the node (its default). A literal `visible:` sets it here.
  node.setVisible(n.visible_bind.empty() ? n.visible_default : true);
}

void realize_node(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
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
  apply_common(*node, n);

  // Children are realized as top-level nodes for now; true parent nesting (a
  // <parent>.children.<child> path) rides on the §11 path binding — a follow-up.
  for (const auto &c : n.children)
    realize_node(sg, c, warnings);
}

} // namespace

std::vector<std::string> realize_scene(SceneGraph &sg, const cvc::ariadne::Scene &scene,
                                       std::vector<std::string> *warnings) {
  std::vector<std::string> created;
  created.reserve(scene.nodes.size());
  for (const auto &n : scene.nodes) {
    realize_node(sg, n, warnings);
    created.push_back(n.id);
  }
  // Lights + shadows realization (LightNode / StageLighting) is a follow-up (§9).
  if (!scene.lights.empty())
    warn(warnings, "ari: scene lights parsed but not realized yet");
  return created;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
