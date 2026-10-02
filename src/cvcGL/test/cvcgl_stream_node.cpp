// Phase-2 PR7: the "stream" scene-node realizer (register_stream_node_type). Proves the
// DECLARATIVE wiring end-to-end with no GL hardware (offscreen llvmpipe): a scene node of
// type "stream" naming an open stream realizes to a UV'd GeometryNode, subscribes the
// stream latest-wins, and registers a per-frame StreamTextureBinding tick in the
// RealizedScene; tick_scene() then drives it on the render thread, and dropping the scene
// unsubscribes. Also covers graceful degradation: a missing token rejects the node, and an
// unknown token still builds the (texture-less) quad but wires no tick/subscription. The
// pixel-level zero-copy alias is already covered by cvcgl_stream_texture.cpp; this test owns
// the realizer seam (props -> quad -> resolve+subscribe -> custom_ticks -> teardown).

// cvcpkg builds tests Release (NDEBUG), which would make every assert() vacuous.
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/stream_node.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkRenderer.h>

using cvc::ariadne::Scene;
using cvc::ariadne::SceneNode;
using cvc::ariadne::Value;
using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
namespace sx = cvc::ariadne::stream;

static sx::stream_params video_params(const char *id, int w, int h) {
  sx::stream_params p;
  p.id = id;
  p.format.kind = sx::frame_kind::video_raw;
  p.format.codec = "rgba8";
  p.format.w = w;
  p.format.h = h;
  p.format.stride = w * 4; // dense
  p.subscriber_depth = 2;
  p.expected_subscribers = 1;
  return p;
}

static void publish(sx::stream &s, std::uint8_t tag) {
  auto l = s.channel().pool().acquire();
  assert(l.has_value() && "pool exhausted");
  l->data[0] = tag;
  s.channel().publish(*l, l->cap, 0.0);
}

// A SceneNode of type "stream" whose props bag carries the attrs the realizer reads, exactly
// as the loader would have captured them (scalar entries in an insertion-ordered Map).
static SceneNode stream_node(const std::string &id, const std::string &token) {
  SceneNode n;
  n.id = id;
  n.type = "stream";
  n.props.kind = Value::Kind::Map;
  const auto put = [&](const char *k, const std::string &v) {
    Value s;
    s.kind = Value::Kind::Scalar;
    s.scalar = v;
    n.props.entries.emplace_back(k, s);
  };
  if (!token.empty())
    put("stream", token);
  put("width", "1.6");
  put("height", "0.9");
  return n;
}

int main() {
  cvc::app app;

  cvc::gl::ariadne::register_stream_node_type();
  assert(cvc::gl::ariadne::has_scene_node_type("stream") && "stream type not registered");

  // An open stream with one published frame, resolvable before the scene realizes.
  auto s = sx::stream::open(app, video_params("vid", 8, 8));
  assert(s && "stream::open failed");
  publish(*s, 0xA1);

  SceneGraph sg(app, "s");
  SceneRenderer view(sg, 64, 64, /*offscreen=*/true, "main");

  // ── happy path: node resolves, subscribes, wires exactly one per-frame tick ──
  {
    Scene scene;
    scene.nodes.push_back(stream_node("screen", s->token()));
    std::vector<std::string> warns;
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "s", &warns);

    assert(warns.empty() && "a resolvable stream node should emit no warning");
    assert(std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("screen")) != nullptr &&
           "realizer did not build an addressable GeometryNode");
    assert(realized.custom_ticks.size() == 1 && "stream realizer wired exactly one per-frame tick");
    assert(s->channel().subscriber_count() == 1 && "realizer did not subscribe to the stream");

    // Driving the frame tick pulls the latest frame and sets the texture; a second tick with
    // no newer frame is a safe no-op. Neither may crash.
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
    cvc::gl::ariadne::tick_scene(realized, view.renderer());
  } // ~RealizedScene drops the binding -> ~StreamTextureBinding -> unsubscribe (render thread)
  assert(s->channel().subscriber_count() == 0 &&
         "binding did not unsubscribe when the realized scene was torn down");

  // ── missing token: the node is an authoring error -> rejected with a warning ──
  {
    Scene scene;
    scene.nodes.push_back(stream_node("notoken", /*token=*/""));
    std::vector<std::string> warns;
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "s2", &warns);
    assert(!warns.empty() && "a stream node with no token should warn");
    assert(sg.getGraphics("notoken") == nullptr && "a tokenless stream node must not be realized");
    assert(realized.custom_ticks.empty() && "no tick for a rejected node");
  }

  // ── unknown token: quad still built (texture-less), warned, no tick/subscription ──
  {
    Scene scene;
    scene.nodes.push_back(stream_node("ghost", "streams.does-not-exist"));
    std::vector<std::string> warns;
    auto realized = cvc::gl::ariadne::realize_scene(sg, scene, "s3", &warns);
    assert(!warns.empty() && "an unknown stream token should warn");
    assert(std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("ghost")) != nullptr &&
           "the quad should still be built for an unresolved stream");
    assert(realized.custom_ticks.empty() && "no tick should be wired for an unresolved stream");
    assert(s->channel().subscriber_count() == 0 && "no subscription for an unresolved stream");
  }

  std::printf("cvcgl_stream_node: OK\n");
  return 0;
}
