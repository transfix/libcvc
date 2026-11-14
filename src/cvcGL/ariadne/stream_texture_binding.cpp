/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/ariadne/stream/raster.h>          // rgba8.patch frames, raster mirrors
#include <cvc/ariadne/stream/stream_channel.h>  // subscription::latest
#include <cvc/ariadne/stream/stream_registry.h> // re-resolve the channel for unsubscribe
#include <cvc/ariadne/stream/to_image.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ariadne/stream_texture_binding.h>
#include <exception>

namespace cvc {
namespace gl {
namespace ariadne {

StreamTextureBinding::StreamTextureBinding(cvc::app &app, std::string token,
                                           std::weak_ptr<cvc::gl::GeometryNode> node,
                                           std::shared_ptr<cvc::ariadne::stream::subscription> sub)
    : app_(app), token_(std::move(token)), node_(std::move(node)), sub_(std::move(sub)) {}

StreamTextureBinding::~StreamTextureBinding() { release(); }

void StreamTextureBinding::tick() {
  if (released_)
    return;
  auto node = node_.lock();
  if (!node) {
    // The node was torn down: free the subscription now so the pinned pool slab
    // and the channel's committed slots are released immediately, rather than
    // lingering for the rest of the scene's life.
    release();
    return;
  }
  cvc::ariadne::stream::frame_ptr f = sub_ ? sub_->latest() : nullptr;
  if (!f || f->seq == last_seq_)
    return; // nothing new to show
  last_seq_ = f->seq;
  try {
    // Zero-copy: to_image aliases the frame's bytes (co-owning its keepalive) and
    // setTexture aliases that buffer into the vtkTexture — no pixel copy.
    node->setTexture(cvc::ariadne::stream::to_image(f), /*zeroCopy=*/true);
  } catch (const std::exception &) {
    // Frame not losslessly aliasable (unknown/channel-swapped codec, padded
    // stride, or bottom-left origin): skip it rather than draw garbage. A later
    // frame with a valid layout will apply normally.
  }
}

void StreamTextureBinding::release() {
  if (released_)
    return;
  released_ = true;
  if (sub_) {
    // Re-resolve the channel (it may already be gone if the stream closed first,
    // in which case the subscription is already closed and dropping it suffices).
    // Safe only under the owner-thread contract (no cross-thread teardown races the
    // lookup→use gap). Erase-before-destruct in ~stream (close() uninstalls the
    // key before channel_ is freed) guarantees lookup() never returns a freed
    // channel on this thread.
    // `token_` is the scope-aware canonical key (stream::registry_key(root, id) ==
    // stream.token()), so this resolves the exact channel across §4.1 scoping — the
    // caller passes stream.token(), not the bare id. (A same-id stream reopened
    // under the same scope after the original closed makes this resolve the NEW
    // channel; unsubscribe then harmlessly no-ops since sub_ is not in its subs_.)
    if (auto *ch = cvc::ariadne::stream::stream_registry::for_app(app_).lookup(token_))
      ch->unsubscribe(sub_);
    sub_.reset();
  }
}

// --- LiveTextureBinding
// ---------------------------------------------------------------------------

namespace sx = cvc::ariadne::stream;

struct LiveTextureBinding::Impl {
  sx::stream_channel *ch = nullptr; // the channel subscribed to (re-checked against the registry)
  std::shared_ptr<sx::subscription> sub;
  std::shared_ptr<sx::raster_follower> follower; // the source was a mirror
  bool raster = false;                           // rgba8.patch frames, into `canvas`
  sx::raster_canvas canvas;
  const std::uint8_t *bound = nullptr; // the canvas storage the node's texture aliases
  std::int64_t last_seq = -1;          // a raw stream's last frame shown
  bool caught_up = false;              // the canvas caught up from the raster's state: all of it
};

LiveTextureBinding::LiveTextureBinding(cvc::app &app, std::vector<std::string> sources,
                                       std::weak_ptr<cvc::gl::GeometryNode> node)
    : app_(app), sources_(std::move(sources)), node_(std::move(node)),
      m_(std::make_unique<Impl>()) {}

LiveTextureBinding::~LiveTextureBinding() {
  release();
  m_->follower.reset();
}

bool LiveTextureBinding::subscribed() const noexcept { return m_->sub != nullptr; }

void LiveTextureBinding::release() {
  if (!m_->sub)
    return;
  // As StreamTextureBinding::release: re-resolved, so a stream that closed first is not touched.
  if (sx::stream_channel *ch = sx::stream_registry::for_app(app_).lookup(token_);
      ch && ch == m_->ch)
    ch->unsubscribe(m_->sub);
  m_->sub.reset();
  m_->ch = nullptr;
}

void LiveTextureBinding::tick() {
  if (finished_)
    return;
  std::shared_ptr<cvc::gl::GeometryNode> node = node_.lock();
  if (!node) { // the node went: let the stream's slots go now
    release();
    m_->follower.reset();
    finished_ = true;
    return;
  }
  sx::stream_registry &reg = sx::stream_registry::for_app(app_);
  if (m_->follower)
    m_->follower->poll(); // a mirror's new frames onto its local stream
  if (m_->sub && reg.lookup(token_) != m_->ch)
    release(); // the stream closed (or another took its token): look again
  if (!m_->sub) {
    sx::stream_channel *ch = nullptr;
    std::string token;
    for (const std::string &s : sources_)
      if ((ch = reg.lookup(s))) {
        token = s;
        break;
      }
    if (!ch)
      for (const std::string &s : sources_)
        if (sx::raster_mirror_present(app_, s)) {
          if (!m_->follower || m_->follower->path() != s)
            m_->follower = sx::raster_follower::follow(app_, s);
          m_->follower->poll();
          token = m_->follower->token();
          if (!token.empty() && (ch = reg.lookup(token)))
            break;
        }
    if (!ch)
      return; // not live yet
    const bool raster = ch->format().codec == sx::kRasterCodec;
    std::shared_ptr<sx::subscription> sub;
    if (raster) {
      // Every frame, in order (a raster's frames are its changes): the deepest ring it admits.
      for (std::size_t depth : {8, 4, 2, 1})
        if ((sub = ch->subscribe(sx::deliver_mode::ring, depth)))
          break;
    } else {
      sub = ch->subscribe(sx::deliver_mode::latest);
    }
    if (!sub)
      return; // its pool is full: again next tick
    m_->ch = ch;
    m_->sub = std::move(sub);
    m_->raster = raster;
    m_->last_seq = -1;
    token_ = token;
    // A raster keeps its frames at its token: subscribed first, so nothing falls between, catch
    // up there (the producer may have painted its last frame before this subscribed). The ring
    // then repeats some of them, which the canvas already has and passes over.
    if (raster && sx::raster_catch_up(app_, token_, m_->canvas) > 0)
      m_->caught_up = true;
  }

  if (!m_->raster) {
    sx::frame_ptr f = m_->sub->latest();
    if (!f || f->seq == m_->last_seq)
      return;
    m_->last_seq = f->seq;
    ++stats_.frames;
    try {
      node->setTexture(sx::to_image(f), /*zeroCopy=*/true);
      ++stats_.full_uploads;
    } catch (const std::exception &) {
      // Not a raster this can alias (StreamTextureBinding's rule): skipped.
    }
    return;
  }

  bool all = m_->caught_up;
  m_->caught_up = false;
  std::vector<sx::raster_rect> rects;
  sx::frame_ptr f;
  while (m_->sub->try_pop(f)) {
    ++stats_.frames;
    sx::raster_frame_view v;
    if (!f || !sx::decode_raster_frame(f->data, f->size, v))
      continue;
    const sx::raster_canvas::update u = m_->canvas.apply(v);
    if (!u.applied) {
      ++stats_.skipped;
      continue;
    }
    if (u.all)
      all = true;
    else
      rects.insert(rects.end(), u.rects.begin(), u.rects.end());
  }
  const std::uint8_t *storage = m_->canvas.pixels();
  if (!storage)
    return;
  if (storage != m_->bound) { // the first picture, or a new size: the texture aliases the canvas
    node->setTexture(m_->canvas.image(), /*zeroCopy=*/true);
    m_->bound = storage;
    ++stats_.full_uploads;
    return;
  }
  if (all) {
    node->texture_modified();
    ++stats_.full_uploads;
    return;
  }
  for (const sx::raster_rect &r : rects) {
    node->texture_modified_rect(r.x, r.y, r.x + r.w, r.y + r.h);
    ++stats_.rect_uploads;
    stats_.texels += static_cast<std::uint64_t>(r.w) * static_cast<std::uint64_t>(r.h);
  }
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
