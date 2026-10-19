/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::gl::physics_scene_binding — draws a cvc::physics world's bodies in a
// scene graph, between the world's last two states.
//
//   cvc::physics::world w(cfg);
//   cvc::gl::physics_scene_binding view(w);       // per world and scene; no globals
//   view.bind(crate, sg.addGraphics("crate", crate_mesh));
//   view.bind_group(swarm_ids, [&](const auto &poses) { pack_glyphs(poses); });
//   // every frame, before rendering:
//   const auto s = app.world_clock().advance(wall_dt);
//   view.follow(app.world_clock());               // steps the world
//   view.present(app.world_clock(), s.alpha);     // poses the nodes
//
// One writer per actor. Bind a body here when physics decides where it is: a
// dynamic body (a crate, a vehicle on Jolt). A kinematic actor driven by its own
// source (a nav agent on the kinematic plant, a tracked person, a scripted mover)
// is drawn by that source, which also moves its body with move_kinematic so it
// pushes things; don't bind it as well, or two writers fight over the node.
// Static bodies (terrain, buildings) never move and need no binding.
//
// Frames: the physics world (metres, z up) sits in the scene at the frame node
// set_frame() names, or at the scene root without one. present() puts each bound
// node's WORLD matrix at frame-world x body pose (GraphicsNode::
// setWorldPoseMatrix), composing with the node's own ancestors on the owner
// thread, so a body's node may sit anywhere in the hierarchy and still be drawn
// where physics has it. Give the frame node a render-origin offset or a scale and
// every body follows; draw the static world under the same node so the world and
// the bodies share one frame.
//
// Threads: thread-safe. present() may run on any thread: setPoseMatrix applies
// at once on the scene's owner thread and is posted to it from any other, latest
// wins. Group sinks run on the thread that calls present(). Bind and unbind from
// any thread. The world must outlive the binding.

#ifndef __CVC_GL_PHYSICS_SCENE_BINDING_H__
#define __CVC_GL_PHYSICS_SCENE_BINDING_H__

#include <cstdint>
#include <cvc/physics/pose_tracker.h>
#include <functional>
#include <memory>
#include <vector>

namespace cvc {

class world_clock;

namespace physics {
class world;
}

namespace gl {

class GraphicsNode;

class physics_scene_binding {
public:
  explicit physics_scene_binding(physics::world &w);
  ~physics_scene_binding();
  physics_scene_binding(const physics_scene_binding &) = delete;
  physics_scene_binding &operator=(const physics_scene_binding &) = delete;

  // Draw body `id` with `node`: each present() sets the node's local transform
  // to the body's pose times `body_from_visual` (row-major 4x4; null for
  // geometry modelled about the body's origin, such as a box centred on it).
  // Binding a bound body again replaces its node and offset. The node is held
  // weakly: a node that is destroyed (its scene torn down, removeGraphics) is
  // unbound at the next present(). Throws cvc::physics_error for a body the
  // world does not hold.
  void bind(physics::body_id id, const std::shared_ptr<GraphicsNode> &node,
            const double body_from_visual[16] = nullptr);
  void unbind(physics::body_id id);
  bool bound(physics::body_id id) const;

  // A swarm: many bodies drawn as one merged mesh. Each present() calls `sink`
  // once with the poses of these bodies, in this order (a body not captured yet,
  // or removed from the world, is left out), so it can pack every instance into
  // one GeometryNode::updateVertices or StreamingGeometryNode::writePoints. A
  // body may be in a group and bound to a node at once. Throws like bind().
  using pose_sink = std::function<void(const std::vector<physics::pose_tracker::sample> &)>;
  using group_id = std::uint32_t;
  group_id bind_group(const std::vector<physics::body_id> &ids, pose_sink sink);
  void unbind_group(group_id g);

  // The scene node that stands for the physics world's frame: null (the default)
  // for the scene root. Held weakly; while a frame is set but destroyed, nothing
  // is posed.
  void set_frame(const std::shared_ptr<const GraphicsNode> &frame);

  // The tracker behind the binding: snap() a teleported body here, or capture()
  // after stepping the world yourself instead of calling follow().
  physics::pose_tracker &tracker();

  // Step the world by the steps due on `clock` (pose_tracker::follow). Use this
  // instead of world::follow, never both.
  std::uint64_t follow(const cvc::world_clock &clock);

  // Pose every bound node and call every group's sink, at alpha in [0, 1) between
  // the last two captured states. The clock overload takes the alpha that
  // clock.advance() returned and converts it (pose_tracker::alpha).
  void present(double alpha);
  void present(const cvc::world_clock &clock, double clock_alpha);

  // How many bodies are bound to nodes, how many groups there are, and how many
  // nodes the last present() posed.
  std::size_t node_count() const;
  std::size_t group_count() const;
  std::size_t last_posed() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_PHYSICS_SCENE_BINDING_H__
