/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_STREAM_H__
#define __CVC_ARIADNE_STREAM_STREAM_H__

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/frame_pool.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/core/state.h>
#include <memory>
#include <string>
#include <vector>

// stream — the Phase-1 owner object that wires a real-time stream into cvc::state
// (roadmap STATE_BINARY_STREAMING.md §4). open() sizes a frame_pool to the §3.2
// invariant, builds the stream_channel, installs the registry token, and
// publishes a /streams/<id> descriptor node (format + lifecycle + stats). The
// producer thread borrows channel().publish() for frame data and post_seq() for
// a throttled seq heartbeat; only lightweight EVENTS ever cross onto the pump.
//
// Descriptor writes (open/close/update_stats) happen on the OWNER/pump thread
// only — the producer thread never writes the state tree, it only posts events.
//
// Lifetime contract: stop the producer thread (join) BEFORE destroying the
// stream, so no producer callback races teardown.

namespace cvc {

class app;

namespace ariadne {
namespace stream {

class frame_source;    // a pull source of frames (frame_source.h); the stream drives it
class producer_thread; // the dedicated worker that drives it (producer_thread.h)

// A snapshot of a stream's runtime counters, stored on the descriptor's stats
// child node (read back with data<stream_stats>()).
struct stream_stats {
  std::uint64_t published = 0;
  std::uint64_t borrow_fail = 0;
  std::size_t slabs_in_use = 0;
  std::size_t subscribers = 0;
  std::int64_t last_seq = -1;
};

struct stream_params {
  std::string id; // unique within the scope; the descriptor node name under <root_path>.streams
  std::string root_path; // §4.1 chroot scope (empty = app root / standalone); the opening Ariadne
                         // document passes its ictx.root_path so streams are isolated per document
  format_desc format;    // must be sized (frame_bytes() > 0) or open() returns null
  std::size_t subscriber_depth = 3;         // Q7 default ring/register depth per subscriber
  std::size_t expected_subscribers = 1;     // sizes the pool to the §3.2 invariant
  std::size_t producer_working_set = 2;     // slabs reserved for the producer (double-buffer)
  std::size_t in_flight_per_subscriber = 1; // slab a consumer holds while draining/rendering
  double heartbeat_hz = 10.0;               // cap on seq-tick posts (0 => post every seq)
};

class stream {
public:
  // Returns null if the format is unsized, the id is empty, or a stream is
  // already live under the same (root_path, id) in this app's registry.
  static std::unique_ptr<stream> open(app &ctx, const stream_params &p);
  ~stream();

  // The canonical stream key: the descriptor node's full path under the scope,
  // `<root_path>.streams.<id>` (normalized). This is BOTH the stream_registry key
  // AND the descriptor node path — a single identity that same-scope lookups and a
  // link:-grant-followed resolution both produce (§4.1), so a granted sub-app
  // resolves the identical key. A consumer (e.g. StreamTextureBinding) that knows
  // (root_path, id) uses this to look the channel up; token() returns it.
  static std::string registry_key(const std::string &root_path, const std::string &id);

  stream(const stream &) = delete;
  stream &operator=(const stream &) = delete;

  stream_channel &channel() noexcept { return *channel_; }
  const std::shared_ptr<frame_pool> &pool() const noexcept { return pool_; }
  const std::string &id() const noexcept { return id_; }
  const std::string &token() const noexcept { return token_; }
  const std::string &seq_channel() const noexcept { return seq_channel_; }
  const std::string &evt_channel() const noexcept { return evt_channel_; }

  // Producer thread: post the latest seq onto the pump, throttled to
  // heartbeat_hz. The payload carries the latest seq so a receiver catches up;
  // intermediate ticks are intentionally coalesced (bounds pump ingress).
  void post_seq(std::int64_t seq);
  // Any thread: a lifecycle event onto the pump (rare; posted unconditionally).
  void post_lifecycle(const char *lifecycle);
  // Owner/pump thread only: refresh the descriptor's stats child node.
  void update_stats();

  // Attach a frame_source and start a DEDICATED producer thread that drives it at `hz` frames/sec
  // (roadmap §"Phase 3 — producers"). Each tick acquires a pool slab, has the source fill() it,
  // publishes it on channel(), and posts a throttled seq heartbeat; a tick that cannot get a slab
  // (pool full under a slow consumer) DROPS that frame rather than stalling the source (§3.2). A
  // source that returns stop=true ends the producer loop. Replaces any existing producer (the old
  // one is stopped+joined first). The producer is always stopped (joined) by close()/~stream BEFORE
  // the channel and pool it publishes into are torn down, so no producer callback races teardown.
  // Rejected (no-op) if the stream is closed or the source is unsized / its frame_bytes() exceed
  // the pool slab (a format mismatch), so a bad source fails here rather than silently producing
  // nothing. Called on the OWNER thread (the same thread as close()/drain); not the producer
  // thread.
  void start_producer(std::unique_ptr<frame_source> source, double hz);

  // Idempotent. Stops (joins) the producer if one is running, then closes the channel (unblocking
  // consumers), uninstalls the registry token, and marks the descriptor "closed".
  void close();

private:
  stream(app &ctx, std::string id);

  app &ctx_;
  std::string id_;
  std::string scope_; // root_path this stream was opened under
  std::string token_; // == registry_key(scope_, id_): canonical descriptor path + registry key
  std::string seq_channel_;
  std::string evt_channel_;
  std::int64_t heartbeat_ns_ = 0;
  std::atomic<std::int64_t> pending_seq_{-1};
  std::atomic<std::int64_t> last_post_ns_{0};
  std::shared_ptr<frame_pool> pool_;
  std::unique_ptr<stream_channel> channel_;
  std::vector<state::state_ptr> descriptor_pins_; // root-child..node ancestor pins (H1)
  state::handle descriptor_;
  state::handle stats_node_;
  // True only after a fully successful open(). A failed open() (token collision
  // or descriptor-build throw) destroys a partially-built stream whose evt/seq
  // channels are SHARED by id with any already-live stream of that id; gating
  // the "closed" side effects on live_ keeps that failure from posting a
  // spurious "closed"/final-seq onto a live stream's channel.
  bool live_ = false;
  std::atomic<bool> closed_{false};
  // An optional attached producer and its source (Phase 3). Declared LAST so they destruct FIRST:
  // ~producer_thread stops+joins the worker before source_/channel_/pool_ (which its tick captures)
  // are destroyed. close() also stops the producer up front, before the channel is closed, so the
  // join-before-free holds on both the explicit-close and the destructor paths.
  std::unique_ptr<frame_source> source_;
  std::unique_ptr<producer_thread> producer_;
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_STREAM_H__
