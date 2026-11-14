/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_ARIADNE_STREAM_TEXTURE_BINDING_H__
#define __CVC_GL_ARIADNE_STREAM_TEXTURE_BINDING_H__

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// StreamTextureBinding — drives a GeometryNode's texture from a cvc::ariadne::stream video
// subscription (roadmap STATE_BINARY_STREAMING.md Phase 2). Once per rendered
// frame the host calls tick() (on the render/owner thread, e.g. from a
// RealizedScene custom_tick): on a newer frame seq it aliases the frame into the
// node's texture zero-copy via cvc::ariadne::stream::to_image + GeometryNode::setTexture;
// when the node is gone it unsubscribes so the stream's pool slab and the
// channel's committed slots are released promptly (not held for the whole scene
// lifetime).

namespace cvc {
class app;
namespace gl {
class GeometryNode;
}
namespace ariadne {
namespace stream {
class subscription;
}
} // namespace ariadne
} // namespace cvc

namespace cvc {
namespace gl {
namespace ariadne {

// THREADING & LIFETIME CONTRACT (load-bearing — the implementation relies on it):
//   - NOT thread-safe. Construction, every tick(), and destruction must all run on
//     the single owner/render thread (the thread that drives the scene). last_seq_
//     and released_ are plain scalars, and release() re-resolves the channel through
//     the registry's raw pointer, both of which are safe only under this rule.
//   - The stream (and its channel) must OUTLIVE the binding, and both must be torn
//     down on that same owner thread. The producer may run on its own thread (it only
//     publish()es); it must never destroy the stream. (If a future model needs
//     cross-thread stream teardown, release() must stop re-resolving via the registry
//     raw pointer and instead hold a safe back-reference to the channel.)
//   - The cvc::app passed to the ctor must outlive the binding (used in release()).
class StreamTextureBinding {
public:
  // `token` is the stream's canonical registry key (stream::registry_key(root, id)
  // == stream.token()), used only to re-resolve the live channel for a clean
  // unsubscribe at teardown. `node` is held weakly so the binding never keeps a
  // torn-down node alive. `sub` is a latest-mode subscription on the stream's channel.
  StreamTextureBinding(cvc::app &app, std::string token, std::weak_ptr<cvc::gl::GeometryNode> node,
                       std::shared_ptr<cvc::ariadne::stream::subscription> sub);
  ~StreamTextureBinding();

  StreamTextureBinding(const StreamTextureBinding &) = delete;
  StreamTextureBinding &operator=(const StreamTextureBinding &) = delete;

  // Render/owner thread. No-op once released. If the node is gone, unsubscribes
  // and stops. Otherwise, if a frame newer than the last applied seq is
  // available, aliases it into the node's texture.
  void tick();

  std::int64_t last_seq() const noexcept { return last_seq_; }
  bool subscribed() const noexcept { return !released_; }

private:
  void release(); // idempotent: unsubscribe + drop the subscription

  cvc::app &app_;
  std::string token_;
  std::weak_ptr<cvc::gl::GeometryNode> node_;
  std::shared_ptr<cvc::ariadne::stream::subscription> sub_;
  std::int64_t last_seq_ = -1;
  bool released_ = false;
};

// LiveTextureBinding — a node's `material: { texture: { stream: <source> } }` (docs/ARIADNE.md
// §5.4, "Live textures"): a GeometryNode's texture fed from a stream, uploading only what changed.
//
// - `sources` are the names to look for, in order: stream tokens, then state paths that may hold a
//   raster mirror (cvc::ariadne::stream::raster_params::mirror; a relay's client has one). The
//   first tick on which one of them is live subscribes; until then, and again after the stream
//   closes, each tick looks again, so the producer may start before or after the scene.
// - A raster stream (codec "rgba8.patch") is drawn into a canvas the texture aliases: the first
//   frame (and one after a gap) uploads the texture whole, every other frame only its changed
//   rectangles (GeometryNode::texture_modified_rect, glTexSubImage2D).
// - A plain raw stream (rgba8, rgb8, ...) shows each new frame whole, as StreamTextureBinding does.
//
// Render/owner thread only, like StreamTextureBinding: construction, tick() and destruction.
class LiveTextureBinding {
public:
  struct Stats {
    std::uint64_t frames = 0;       // frames taken from the stream
    std::uint64_t full_uploads = 0; // whole-texture uploads (setTexture, texture_modified)
    std::uint64_t rect_uploads = 0; // texture_modified_rect calls
    std::uint64_t texels = 0;       // texels those covered
    std::uint64_t skipped = 0;      // delta frames dropped while waiting for a key frame
  };

  LiveTextureBinding(cvc::app &app, std::vector<std::string> sources,
                     std::weak_ptr<cvc::gl::GeometryNode> node);
  ~LiveTextureBinding();
  LiveTextureBinding(const LiveTextureBinding &) = delete;
  LiveTextureBinding &operator=(const LiveTextureBinding &) = delete;

  // Look for the source when not subscribed, then take every new frame onto the node's texture.
  // Stops for good (unsubscribing) once the node is gone.
  void tick();

  bool subscribed() const noexcept;
  bool finished() const noexcept { return finished_; }         // the node is gone
  const std::string &token() const noexcept { return token_; } // the stream it reads ("": none)
  const Stats &stats() const noexcept { return stats_; }

private:
  struct Impl;
  void release();

  cvc::app &app_;
  std::vector<std::string> sources_;
  std::weak_ptr<cvc::gl::GeometryNode> node_;
  std::string token_;
  std::unique_ptr<Impl> m_;
  Stats stats_;
  bool finished_ = false;
};

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_ARIADNE_STREAM_TEXTURE_BINDING_H__
