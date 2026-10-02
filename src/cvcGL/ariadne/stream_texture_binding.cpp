/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

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

} // namespace ariadne
} // namespace gl
} // namespace cvc
