/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 1 tests for the cvc::ariadne::stream in-process real-time frame transport
// (roadmap docs/roadmap/STATE_BINARY_STREAMING.md). Covers the load-bearing
// claims: zero-copy pointer identity + slab recycling, non-blocking drop-at-
// source pool invariant (H3), drop-oldest / latest-wins per-subscriber delivery,
// registry round-trip, dedicated-producer-thread lifecycle, descriptor node
// discovery, event hookup with a throttled seq heartbeat that bounds pump
// ingress, and shutdown ordering with no use-after-free.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cvc/ariadne/stream/audio_sources.h>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/frame_pool.h>
#include <cvc/ariadne/stream/producer_thread.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/ariadne/stream/synthetic_source.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::ariadne::stream;

namespace {

format_desc rgba(int w, int h) {
  format_desc f;
  f.kind = frame_kind::video_raw;
  f.codec = "rgba8";
  f.w = w;
  f.h = h;
  f.stride = w * 4;
  return f;
}

format_desc pcm(int frames, int channels) {
  format_desc f;
  f.kind = frame_kind::audio_pcm;
  f.codec = "s16le";
  f.sample_rate = 48000;
  f.channels = channels;
  f.bytes = static_cast<std::size_t>(frames) * channels * 2; // explicit non-video sizing
  return f;
}

// A frame_source that is always LIVE but never produces a frame (bytes 0, never stop) — models a
// device input that is stalled/warming up. Its frame_bytes() matches a given size so a mix accepts
// it.
struct silent_source : frame_source {
  std::size_t bytes_;
  explicit silent_source(std::size_t bytes) : bytes_(bytes) {}
  std::size_t frame_bytes() const override { return bytes_; }
  produced_frame fill(std::uint8_t *, std::size_t) override {
    return {};
  } // always skip, never stop
};

// A controllable test frame_source: produces `bytes`-sized frames, stops after `limit` (0 = never).
struct test_source : frame_source {
  std::size_t bytes_;
  int limit_;
  int made_ = 0;
  test_source(std::size_t bytes, int limit) : bytes_(bytes), limit_(limit) {}
  std::size_t frame_bytes() const override { return bytes_; }
  produced_frame fill(std::uint8_t *buf, std::size_t cap) override {
    produced_frame out;
    if (bytes_ == 0 || bytes_ > cap)
      return out;
    for (std::size_t i = 0; i < bytes_; ++i)
      buf[i] = 0xAB;
    out.bytes = bytes_;
    out.pts_seconds = made_ * 0.01;
    ++made_;
    out.stop = (limit_ > 0 && made_ >= limit_); // spent after `limit_` frames
    return out;
  }
};

} // namespace

// --------------------------------------------------------------------------
// frame_pool
// --------------------------------------------------------------------------

TEST(StreamFramePool, AcquireExhaustsThenBorrowFails) {
  auto pool = frame_pool::create(64, 3);
  std::vector<frame_pool::lease> held;
  for (int i = 0; i < 3; ++i) {
    auto l = pool->acquire();
    ASSERT_TRUE(l.has_value());
    held.push_back(*l);
  }
  // All three slabs are checked out; the next acquire drops at source.
  EXPECT_FALSE(pool->acquire().has_value());
  EXPECT_EQ(pool->total_borrow_fail(), 1u);
  EXPECT_EQ(pool->in_use(), 3u);
  // The three leases point at distinct backing buffers.
  EXPECT_NE(held[0].data, held[1].data);
  EXPECT_NE(held[1].data, held[2].data);
}

TEST(StreamFramePool, LastDropRecyclesSlabAndReusesBacking) {
  auto pool = frame_pool::create(64, 1);
  auto l = pool->acquire();
  ASSERT_TRUE(l.has_value());
  std::uint8_t *backing = l->data;
  EXPECT_FALSE(pool->acquire().has_value()); // pool exhausted

  frame_ptr f = pool->publish(*l, 64, /*seq*/ 7, /*pts*/ 1.5, rgba(4, 4));
  ASSERT_TRUE(f);
  EXPECT_EQ(f->data, backing); // zero-copy: frame borrows the slab
  EXPECT_EQ(f->seq, 7);
  EXPECT_EQ(f->size, 64u);
  EXPECT_EQ(pool->in_use(), 1u);

  f.reset(); // last consumer drops -> slab recycled by the keepalive deleter
  EXPECT_EQ(pool->in_use(), 0u);
  auto l2 = pool->acquire();
  ASSERT_TRUE(l2.has_value());
  EXPECT_EQ(l2->data, backing); // same backing reused, no new allocation
}

TEST(StreamFramePool, FlatBackingAcrossManyCycles) {
  auto pool = frame_pool::create(32, 4);
  std::vector<std::uint8_t *> seen;
  for (int i = 0; i < 500; ++i) {
    auto l = pool->acquire();
    ASSERT_TRUE(l.has_value());
    if (std::find(seen.begin(), seen.end(), l->data) == seen.end())
      seen.push_back(l->data);
    frame_ptr f = pool->publish(*l, 32, i, 0.0, rgba(2, 4));
    f.reset(); // immediately recycle
  }
  // No more distinct backing buffers than slabs -> zero steady-state growth.
  EXPECT_LE(seen.size(), 4u);
  EXPECT_EQ(pool->total_published(), 500u);
}

// --------------------------------------------------------------------------
// subscription
// --------------------------------------------------------------------------

TEST(StreamSubscription, LatestIsNonDestructiveAndCountsOverwrites) {
  auto pool = frame_pool::create(16, 4);
  subscription sub(deliver_mode::latest, 1);
  EXPECT_FALSE(sub.latest());

  auto mk = [&](std::int64_t seq) {
    auto l = pool->acquire();
    return pool->publish(*l, 16, seq, 0.0, rgba(2, 2));
  };
  sub.deliver(mk(0));
  frame_ptr a = sub.latest();
  frame_ptr b = sub.latest();
  ASSERT_TRUE(a);
  EXPECT_EQ(a.get(), b.get()); // non-destructive: same frame twice
  EXPECT_EQ(a->seq, 0);
  EXPECT_EQ(sub.dropped(), 0u);

  sub.deliver(mk(1)); // supersedes an unread frame
  sub.deliver(mk(2));
  EXPECT_EQ(sub.latest()->seq, 2); // freshest wins
  EXPECT_EQ(sub.dropped(), 2u);    // two overwrites
}

TEST(StreamSubscription, RingDropsOldestAndCloseUnblocksPop) {
  auto pool = frame_pool::create(16, 8);
  subscription sub(deliver_mode::ring, 2);
  auto mk = [&](std::int64_t seq) {
    auto l = pool->acquire();
    return pool->publish(*l, 16, seq, 0.0, pcm(2, 2));
  };
  sub.deliver(mk(0));
  sub.deliver(mk(1));
  sub.deliver(mk(2)); // depth 2 -> seq 0 evicted
  EXPECT_EQ(sub.dropped(), 1u);

  frame_ptr out;
  ASSERT_TRUE(sub.try_pop(out));
  EXPECT_EQ(out->seq, 1); // FIFO from the oldest surviving
  ASSERT_TRUE(sub.try_pop(out));
  EXPECT_EQ(out->seq, 2);
  EXPECT_FALSE(sub.try_pop(out)); // drained

  // A parked blocking pop() must be released by close().
  std::atomic<bool> returned{false};
  std::thread waiter([&] {
    frame_ptr f;
    bool ok = sub.pop(f); // blocks until close()
    EXPECT_FALSE(ok);
    returned.store(true);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(returned.load());
  sub.close();
  waiter.join();
  EXPECT_TRUE(returned.load());
}

// --------------------------------------------------------------------------
// stream_channel
// --------------------------------------------------------------------------

TEST(StreamChannel, PublishFansOutAndIsolatesSlowConsumer) {
  cvc::app app;
  auto pool = frame_pool::create(16, 16);
  stream_channel ch(app, "vid", rgba(2, 2), pool, /*pws*/ 2, /*inflight*/ 1);
  auto fast = ch.subscribe(deliver_mode::latest, 1);
  auto slow = ch.subscribe(deliver_mode::ring, 2);
  ASSERT_TRUE(fast);
  ASSERT_TRUE(slow);

  for (int i = 0; i < 6; ++i) {
    auto l = ch.pool().acquire();
    ASSERT_TRUE(l.has_value());
    ch.publish(*l, 16, static_cast<double>(i));
  }
  EXPECT_EQ(ch.last_seq(), 5);
  // Fast (latest) consumer always sees the newest; slow (ring depth 2) drops
  // ITS OWN oldest without touching the fast consumer.
  ASSERT_TRUE(fast->latest());
  EXPECT_EQ(fast->latest()->seq, 5);
  EXPECT_EQ(fast->dropped(), 5u);
  EXPECT_EQ(slow->dropped(), 4u); // 6 delivered, depth 2 kept
}

TEST(StreamChannel, SubscribeRefusesOverInvariant) {
  cvc::app app;
  auto pool = frame_pool::create(16, 6); // 6 slabs
  stream_channel ch(app, "vid", rgba(2, 2), pool, /*pws*/ 2, /*inflight*/ 1);
  // avail = 6 - 2 = 4 slots. A ring depth-3 needs 3 + 1 = 4 -> admitted.
  auto a = ch.subscribe(deliver_mode::ring, 3);
  ASSERT_TRUE(a);
  // No slots left; another subscriber is refused rather than under-provisioned.
  EXPECT_FALSE(ch.subscribe(deliver_mode::latest, 1));
  // Releasing the first frees its slots.
  ch.unsubscribe(a);
  EXPECT_TRUE(ch.subscribe(deliver_mode::latest, 1));
}

TEST(StreamChannel, ConcurrentSubscribeUnsubscribeDuringPublish) {
  cvc::app app;
  auto pool = frame_pool::create(16, 32);
  stream_channel ch(app, "vid", rgba(2, 2), pool, 2, 1);
  std::atomic<bool> run{true};
  std::thread producer([&] {
    while (run.load()) {
      auto l = ch.pool().acquire();
      if (l.has_value())
        ch.publish(*l, 16, 0.0);
    }
  });
  // Churn subscriptions while the producer fans out (exercises the snapshot-
  // under-lock fan-out; must not race or invalidate iterators).
  for (int i = 0; i < 2000; ++i) {
    auto s = ch.subscribe(deliver_mode::ring, 1);
    if (s)
      ch.unsubscribe(s);
  }
  run.store(false);
  producer.join();
  SUCCEED();
}

TEST(StreamChannel, PublishExternalAliasesCallerStorageZeroCopy) {
  cvc::app app;
  auto pool = frame_pool::create(16, 8);
  stream_channel ch(app, "vid", rgba(2, 2), pool, 2, 1);
  auto sub = ch.subscribe(deliver_mode::latest, 1);
  ASSERT_TRUE(sub);

  auto buf = std::make_shared<std::vector<std::uint8_t>>(16, 0xAB);
  const std::int64_t seq = ch.publish_external(buf->data(), buf->size(), 1.5, buf);
  EXPECT_EQ(seq, 0);
  auto f = sub->latest();
  ASSERT_TRUE(f);
  EXPECT_EQ(f->data, buf->data()); // zero-copy alias, not a slab
  EXPECT_EQ(f->size, 16u);
  EXPECT_EQ(f->pts_seconds, 1.5);
  EXPECT_EQ(ch.pool().in_use(), 0u); // the pool was never touched
}

TEST(StreamChannel, PublishExternalInterleavesSeqWithPool) {
  cvc::app app;
  auto pool = frame_pool::create(16, 8);
  stream_channel ch(app, "vid", rgba(2, 2), pool, 2, 1);
  auto sub = ch.subscribe(deliver_mode::latest, 1);
  ASSERT_TRUE(sub);

  auto l0 = ch.pool().acquire();
  ASSERT_TRUE(l0.has_value());
  EXPECT_EQ(ch.publish(*l0, 16, 0.0), 0); // pool seq 0
  auto buf = std::make_shared<std::vector<std::uint8_t>>(16, 0x01);
  EXPECT_EQ(ch.publish_external(buf->data(), 16, 0.0, buf), 1); // external seq 1
  auto l2 = ch.pool().acquire();
  ASSERT_TRUE(l2.has_value());
  EXPECT_EQ(ch.publish(*l2, 16, 0.0), 2); // pool seq 2

  EXPECT_EQ(ch.last_seq(), 2);
  EXPECT_EQ(ch.total_published(), 3u);
  ASSERT_TRUE(sub->latest());
  EXPECT_EQ(sub->latest()->seq, 2); // seq monotonic across both producer paths
}

TEST(StreamChannel, PublishExternalKeepaliveReleasedWhenFrameDrops) {
  bool released = false;
  std::shared_ptr<std::vector<std::uint8_t>> storage(new std::vector<std::uint8_t>(16, 0x7f),
                                                     [&released](std::vector<std::uint8_t> *v) {
                                                       released = true;
                                                       delete v;
                                                     });
  cvc::app app;
  auto pool = frame_pool::create(16, 8);
  stream_channel ch(app, "vid", rgba(2, 2), pool, 2, 1);
  auto sub = ch.subscribe(deliver_mode::ring, 2);
  ASSERT_TRUE(sub);

  ch.publish_external(storage->data(), storage->size(), 0.0, storage);
  storage.reset(); // the ring's frame_ptr now solely pins the buffer
  EXPECT_FALSE(released);
  frame_ptr out;
  ASSERT_TRUE(sub->try_pop(out));
  EXPECT_FALSE(released); // we hold the frame
  out.reset();
  EXPECT_TRUE(released); // last borrow dropped -> caller storage freed
}

// --------------------------------------------------------------------------
// stream_registry
// --------------------------------------------------------------------------

TEST(StreamRegistry, RoundTripFirstWriterWinsPerApp) {
  cvc::app a1;
  cvc::app a2;
  auto pool = frame_pool::create(16, 4);
  stream_channel c1(a1, "s", rgba(2, 2), pool, 2, 1);
  stream_channel c2(a2, "s", rgba(2, 2), pool, 2, 1);

  auto &r1 = stream_registry::for_app(a1);
  EXPECT_TRUE(r1.install("tok", &c1));
  EXPECT_EQ(r1.lookup("tok"), &c1);
  EXPECT_FALSE(r1.install("tok", &c2)); // first-writer-wins
  EXPECT_EQ(r1.lookup("tok"), &c1);

  // Per-app isolation: same token in another app is a distinct table.
  auto &r2 = stream_registry::for_app(a2);
  EXPECT_EQ(r2.lookup("tok"), nullptr);
  EXPECT_TRUE(r2.install("tok", &c2));
  EXPECT_EQ(r2.lookup("tok"), &c2);

  // uninstall erases only if the mapping still points at us.
  r1.uninstall("tok", &c2); // wrong owner -> no-op
  EXPECT_EQ(r1.lookup("tok"), &c1);
  r1.uninstall("tok", &c1);
  EXPECT_EQ(r1.lookup("tok"), nullptr);
}

// --------------------------------------------------------------------------
// producer_thread
// --------------------------------------------------------------------------

TEST(StreamProducerThread, RunsPublishesAndJoins) {
  cvc::app app;
  auto pool = frame_pool::create(16, 8);
  stream_channel ch(app, "vid", rgba(2, 2), pool, 2, 1);
  auto sub = ch.subscribe(deliver_mode::latest, 1);
  ASSERT_TRUE(sub);

  std::atomic<int> produced{0};
  producer_thread prod(
      [&] {
        auto l = ch.pool().acquire();
        if (l.has_value()) {
          ch.publish(*l, 16, 0.0);
          produced.fetch_add(1);
        }
        return true;
      },
      /*hz*/ 200.0);
  prod.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  prod.stop(); // joins
  EXPECT_FALSE(prod.running());
  EXPECT_GT(prod.ticks(), 0);
  EXPECT_GT(produced.load(), 0);
  ASSERT_TRUE(sub->latest());
}

TEST(StreamProducerThread, DropsAtSourceUnderPinnedPoolNeverBlocks) {
  auto pool = frame_pool::create(16, 2);
  // Pin every slab by holding published frames, so acquire() always fails.
  std::vector<frame_ptr> pinned;
  for (int i = 0; i < 2; ++i) {
    auto l = pool->acquire();
    ASSERT_TRUE(l.has_value());
    pinned.push_back(pool->publish(*l, 16, i, 0.0, rgba(2, 2)));
  }
  std::atomic<int> drops{0};
  producer_thread prod(
      [&] {
        auto l = pool->acquire();
        if (!l.has_value()) {
          drops.fetch_add(1); // drop at source, keep ticking
          return true;
        }
        pool->discard(*l);
        return true;
      },
      500.0);
  prod.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  prod.stop();
  EXPECT_GT(prod.ticks(), 0); // producer never blocked
  EXPECT_GT(drops.load(), 0); // it dropped at source
  EXPECT_GT(pool->total_borrow_fail(), 0u);
}

// --------------------------------------------------------------------------
// stream (descriptor + events + lifecycle)
// --------------------------------------------------------------------------

TEST(Stream, OpenRejectsUnsizedFormatAndEmptyId) {
  cvc::app app;
  stream_params p;
  p.id = "vid";
  p.format.kind = frame_kind::video_raw; // no stride/h/bytes -> unsized
  EXPECT_EQ(stream::open(app, p), nullptr);

  stream_params q;
  q.id = ""; // missing id
  q.format = rgba(4, 4);
  EXPECT_EQ(stream::open(app, q), nullptr);

  stream_params d;
  d.id = "cam.0"; // '.' makes the root/id boundary ambiguous in the canonical key
  d.format = rgba(4, 4);
  EXPECT_EQ(stream::open(app, d), nullptr);
}

TEST(Stream, OpenPublishesDescriptorAndRegistersToken) {
  cvc::app app;
  stream_params p;
  p.id = "cam0";
  p.format = rgba(8, 8);
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);

  // Descriptor node exists with lifecycle "live" and the format round-trips.
  cvc::state &node = cvc::state::instance(app)("streams.cam0");
  EXPECT_EQ(node.value(), "live");
  auto fmt = node.data<format_desc>();
  EXPECT_EQ(fmt.codec, "rgba8");
  EXPECT_EQ(fmt.w, 8);

  // Registry resolves the canonical key (empty root -> "streams.cam0") to the
  // live channel.
  EXPECT_EQ(s->token(), "streams.cam0");
  EXPECT_EQ(stream_registry::for_app(app).lookup(s->token()), &s->channel());

  s->close();
  EXPECT_EQ(cvc::state::instance(app)("streams.cam0").value(), "closed");
  EXPECT_EQ(stream_registry::for_app(app).lookup(s->token()), nullptr);
}

TEST(Stream, EventHookupDeliversSeqAndLifecycleThrottled) {
  cvc::app app;
  stream_params p;
  p.id = "evt0";
  p.format = rgba(4, 4);
  p.heartbeat_hz = 1.0; // 1s window: a fast burst is unambiguously within it
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);
  auto &sched = app.exec_scheduler();

  // open() posts a "live" lifecycle event.
  sched.drain_ingress();
  auto evt = sched.pop_pending_message(s->evt_channel());
  ASSERT_TRUE(evt.has_value());
  EXPECT_EQ(std::get<std::string>(evt->v), "live");

  // A burst of 1000 seq posts within one heartbeat window collapses to a
  // handful of pump messages (leading-edge throttle) -> bounded ingress, NOT a
  // per-frame flood. This is the fix for the unbounded-ingress hazard.
  for (std::int64_t i = 0; i < 1000; ++i)
    s->post_seq(i);
  sched.drain_ingress();
  EXPECT_LE(sched.pending_message_count(s->seq_channel()), 2u);
  auto seq = sched.pop_pending_message(s->seq_channel());
  ASSERT_TRUE(seq.has_value());
  const std::int64_t got = std::get<std::int64_t>(seq->v);
  EXPECT_GE(got, 0);
  EXPECT_LT(got, 1000); // a valid seq from the burst, never frame bytes

  s->close();
}

TEST(Stream, ShutdownWithHeldFrameNoUseAfterFree) {
  cvc::app app;
  stream_params p;
  p.id = "shut0";
  p.format = rgba(4, 4);
  p.expected_subscribers = 1;
  p.subscriber_depth = 2;
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);
  auto sub = s->channel().subscribe(deliver_mode::latest, 1);
  ASSERT_TRUE(sub);

  auto l = s->channel().pool().acquire();
  ASSERT_TRUE(l.has_value());
  s->channel().publish(*l, 16, 0.0);
  frame_ptr held = sub->latest(); // consumer still holds a frame
  ASSERT_TRUE(held);

  auto pool = s->pool(); // keep pool observer alive to check recycle after teardown
  s.reset();             // destroy the stream while a frame is still borrowed
  // The frame is still valid because its keepalive pinned the pool + slab, even
  // though the channel (and the pool's owning stream) are gone.
  EXPECT_EQ(held->seq, 0);
  EXPECT_EQ(held->format.w, 4);
  // The slab stays pinned while ANY borrow lives: `held` plus the subscription's
  // own latest_ register (we still hold `sub`). Dropping both releases it.
  held.reset();
  EXPECT_EQ(pool->in_use(), 1u); // still pinned by sub->latest_
  sub.reset();                   // drop the last borrow
  EXPECT_EQ(pool->in_use(), 0u); // slab recycled, no leak / no UAF
}

TEST(Stream, DuplicateOpenDoesNotDisturbLiveStreamEvents) {
  cvc::app app;
  stream_params p;
  p.id = "dup0";
  p.format = rgba(4, 4);
  auto s1 = stream::open(app, p);
  ASSERT_TRUE(s1);
  auto &sched = app.exec_scheduler();
  sched.drain_ingress();
  auto live = sched.pop_pending_message(s1->evt_channel());
  ASSERT_TRUE(live.has_value());
  EXPECT_EQ(std::get<std::string>(live->v), "live");

  // A second open() of the same live id fails on the token collision and tears
  // its partially-built self down. Because evt/seq channels are shared by id,
  // that teardown must NOT post a spurious "closed" onto the live stream's
  // channel, must not touch its descriptor, and must not evict its token.
  auto s2 = stream::open(app, p);
  EXPECT_EQ(s2, nullptr);
  sched.drain_ingress();
  EXPECT_EQ(sched.pending_message_count(s1->evt_channel()), 0u);
  EXPECT_EQ(cvc::state::instance(app)("streams.dup0").value(), "live");
  EXPECT_EQ(stream_registry::for_app(app).lookup(s1->token()), &s1->channel());

  s1->close(); // the real owner posts the real "closed"
  sched.drain_ingress();
  auto closed = sched.pop_pending_message(s1->evt_channel());
  ASSERT_TRUE(closed.has_value());
  EXPECT_EQ(std::get<std::string>(closed->v), "closed");
}

TEST(Stream, ProducerPublishesAcrossTeardownInHonoredOrder) {
  cvc::app app;
  stream_params p;
  p.id = "prod0";
  p.format = rgba(4, 4);
  p.expected_subscribers = 1;
  p.subscriber_depth = 3;
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);
  auto sub = s->channel().subscribe(deliver_mode::latest, 1);
  ASSERT_TRUE(sub);

  std::atomic<int> published{0};
  auto prod = std::make_unique<producer_thread>(
      [&] {
        auto l = s->channel().pool().acquire();
        if (l.has_value()) {
          std::int64_t seq = s->channel().publish(*l, 16, 0.0);
          s->post_seq(seq);
          published.fetch_add(1);
        }
        return true;
      },
      200.0);
  prod->start();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  // Honored teardown order: STOP (join) the producer BEFORE destroying the
  // stream, exactly as the lifetime contract requires. After join, nothing
  // touches the channel/pool, so destroying the stream is safe.
  prod->stop();
  EXPECT_GT(published.load(), 0);

  frame_ptr held = sub->latest(); // hold a frame across teardown
  ASSERT_TRUE(held);
  auto pool = s->pool();
  s.reset(); // destroy the stream after the producer has joined
  const std::int64_t hs = held->seq;
  EXPECT_GE(hs, 0); // the borrowed frame is still valid post-teardown
  held.reset();
  sub.reset();
  EXPECT_EQ(pool->in_use(), 0u); // every slab recycled
}

// --------------------------------------------------------------------------
// §4.1 scoping: scope-aware open + canonical-path registry
// --------------------------------------------------------------------------

TEST(StreamScoping, RegistryKeyIsCanonicalScopedPath) {
  EXPECT_EQ(cvc::ariadne::stream::stream::registry_key("", "cam0"), "streams.cam0");
  EXPECT_EQ(cvc::ariadne::stream::stream::registry_key("doc.a", "cam0"), "doc.a.streams.cam0");
}

TEST(StreamScoping, SameIdDifferentScopesDoNotCollideAndAreIsolated) {
  cvc::app app;
  stream_params pa;
  pa.id = "cam0";
  pa.root_path = "doc.a";
  pa.format = rgba(4, 2);
  stream_params pb;
  pb.id = "cam0";
  pb.root_path = "doc.b";
  pb.format = rgba(4, 2);

  auto a = stream::open(app, pa);
  auto b = stream::open(app, pb);
  ASSERT_TRUE(a);
  ASSERT_TRUE(b); // same id under different scopes -> no registry collision
  EXPECT_EQ(a->token(), "doc.a.streams.cam0");
  EXPECT_EQ(b->token(), "doc.b.streams.cam0");

  // Independent channels: a publish on a is not seen by b's subscriber.
  auto sa = a->channel().subscribe(deliver_mode::latest, 1);
  auto sb = b->channel().subscribe(deliver_mode::latest, 1);
  ASSERT_TRUE(sa);
  ASSERT_TRUE(sb);
  auto l = a->channel().pool().acquire();
  ASSERT_TRUE(l.has_value());
  a->channel().publish(*l, l->cap, 0.0);
  EXPECT_TRUE(sa->latest());
  EXPECT_FALSE(sb->latest()); // scope isolation

  // Descriptors live under distinct scoped paths; events are chroot-scoped.
  EXPECT_EQ(cvc::state::instance(app)("doc.a.streams.cam0").value(), "live");
  EXPECT_EQ(cvc::state::instance(app)("doc.b.streams.cam0").value(), "live");
  EXPECT_EQ(a->seq_channel(), "doc.a.channels.streams.cam0.seq");
  EXPECT_EQ(b->evt_channel(), "doc.b.channels.streams.cam0.evt");
}

TEST(StreamScoping, EmptyRootIsBackwardCompatibleIdentity) {
  cvc::app app;
  stream_params p;
  p.id = "cam0";
  p.format = rgba(4, 2); // root_path empty
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);
  EXPECT_EQ(s->token(), "streams.cam0");
  EXPECT_EQ(s->seq_channel(), "streams.cam0.seq"); // empty root -> identity
  EXPECT_EQ(s->evt_channel(), "streams.cam0.evt");
  EXPECT_EQ(cvc::state::instance(app)("streams.cam0").value(), "live");
}

// --------------------------------------------------------------------------
// stream producer (Phase 3): the stream owns + drives a frame_source via a
// producer_thread, and stops (joins) it before teardown.
// --------------------------------------------------------------------------

TEST(StreamProducer, SyntheticSourceFeedsFramesAndJoinsOnClose) {
  cvc::app app;
  stream_params p;
  p.id = "cam0";
  p.format = rgba(4, 4);
  p.subscriber_depth = 2;
  p.expected_subscribers = 1;
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);
  auto sub = s->channel().subscribe(deliver_mode::latest, 2);
  ASSERT_TRUE(sub);

  // The stream owns the producer thread that drives the synthetic source at 200 Hz.
  s->start_producer(std::make_unique<synthetic_source>(4, 4), /*hz*/ 200.0);

  // Frames flow (bounded wait, no fixed sleep race).
  for (int i = 0; i < 200 && s->channel().total_published() < 3; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_GE(s->channel().total_published(), 3u) << "producer did not publish frames";
  EXPECT_GE(s->channel().last_seq(), 2) << "seq did not advance";

  // The subscriber sees a full-size rgba frame the source actually filled (alpha = 0xFF).
  frame_ptr a = sub->latest();
  ASSERT_TRUE(a);
  EXPECT_EQ(a->size, static_cast<std::size_t>(4 * 4 * 4));
  EXPECT_EQ(a->format.kind, frame_kind::video_raw);
  ASSERT_NE(a->data, nullptr);
  EXPECT_EQ(a->data[3], 0xFF) << "synthetic source did not fill the slab";

  // close() stops+joins the producer: publishing stops and there is no hang.
  s->close();
  const std::uint64_t after_close = s->channel().total_published();
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  EXPECT_EQ(s->channel().total_published(), after_close) << "producer kept running after close()";
  // ~stream at end of scope joins again (idempotent) — no hang/crash.
}

TEST(StreamProducer, StartProducerOnClosedStreamIsIgnored) {
  cvc::app app;
  stream_params p;
  p.id = "cam1";
  p.format = rgba(2, 2);
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);
  s->close();
  s->start_producer(std::make_unique<synthetic_source>(2, 2), 60.0); // no-op on a closed stream
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(s->channel().total_published(), 0u);
}

TEST(StreamProducer, RestartReplacesThePriorProducer) {
  cvc::app app;
  stream_params p;
  p.id = "cam2";
  p.format = rgba(4, 4);
  p.expected_subscribers = 1;
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);
  s->start_producer(std::make_unique<synthetic_source>(4, 4), 200.0);
  for (int i = 0; i < 100 && s->channel().total_published() < 1; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  // A second start_producer stops+joins the first (no two threads on one pool) and keeps producing.
  s->start_producer(std::make_unique<synthetic_source>(4, 4), 200.0);
  const std::uint64_t at_restart = s->channel().total_published();
  for (int i = 0; i < 100 && s->channel().total_published() < at_restart + 3; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  EXPECT_GT(s->channel().total_published(), at_restart) << "restarted producer did not run";
  s->close();
}

TEST(StreamProducer, SelfStoppingSourceEndsTheProducer) {
  cvc::app app;
  stream_params p;
  p.id = "cam3";
  p.format = rgba(4, 4); // slab = 64 bytes
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);
  // A source that publishes exactly 5 frames then returns stop=true.
  s->start_producer(std::make_unique<test_source>(/*bytes*/ 64, /*limit*/ 5), /*hz*/ 500.0);
  for (int i = 0; i < 200 && s->channel().total_published() < 5; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  std::this_thread::sleep_for(std::chrono::milliseconds(10)); // let the stop tick settle
  EXPECT_EQ(s->channel().total_published(), 5u) << "self-stopping source published the wrong count";
  s->close(); // joins the already-finished thread — no hang
}

TEST(StreamProducer, RejectsASourceWhoseFrameExceedsTheSlab) {
  cvc::app app;
  stream_params p;
  p.id = "cam4";
  p.format = rgba(4, 4); // slab = 64 bytes
  auto s = stream::open(app, p);
  ASSERT_TRUE(s);
  // 128 bytes > 64-byte slab: start_producer must reject it up front (no silent dead stream).
  s->start_producer(std::make_unique<test_source>(/*bytes*/ 128, /*limit*/ 0), 200.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(s->channel().total_published(), 0u) << "oversized source should have been rejected";
}

// --------------------------------------------------------------------------
// pure/virtual audio sources (cvc::ariadne): tone / gain / mix — the composable
// audio-source family (f32 PCM), fully testable with no device.
// --------------------------------------------------------------------------

TEST(AudioSources, ToneGeneratesBoundedSeamlessSine) {
  const int sr = 48000, ch = 2, frames = 256;
  tone_source tone(sr, ch, frames, /*hz*/ 440.0, /*amp*/ 0.25);
  const std::size_t need = audio_f32_bytes(frames, ch);
  EXPECT_EQ(tone.frame_bytes(), need);

  std::vector<float> a(static_cast<std::size_t>(frames) * ch),
      b(static_cast<std::size_t>(frames) * ch);
  auto p1 = tone.fill(reinterpret_cast<std::uint8_t *>(a.data()), a.size() * sizeof(float));
  EXPECT_EQ(p1.bytes, need);
  // Interleaved: both channels carry the same sample; all within [-amp, amp]; not all-zero.
  bool nonzero = false;
  for (int n = 0; n < frames; ++n) {
    EXPECT_FLOAT_EQ(a[n * ch + 0], a[n * ch + 1]);
    EXPECT_LE(std::fabs(a[n * ch]), 0.25f + 1e-6f);
    if (std::fabs(a[n * ch]) > 1e-6f)
      nonzero = true;
  }
  EXPECT_TRUE(nonzero) << "tone produced silence";
  // Phase carries across chunks: the second chunk continues the wave (differs from the first) and
  // its pts advances by one chunk.
  auto p2 = tone.fill(reinterpret_cast<std::uint8_t *>(b.data()), b.size() * sizeof(float));
  EXPECT_EQ(p2.bytes, need);
  EXPECT_GT(p2.pts_seconds, p1.pts_seconds);
  EXPECT_NE(a, b) << "phase did not carry across chunks";
}

TEST(AudioSources, GainScalesEverySample) {
  const int sr = 48000, ch = 1, frames = 128;
  const std::size_t need = audio_f32_bytes(frames, ch);
  // gain 2.0 over a tone vs a reference tone with identical params in lockstep (both from phase 0),
  // so gained[i] == 2 * ref[i].
  gain_source gained(std::make_unique<tone_source>(sr, ch, frames, 440.0, 0.25), ch, frames, 2.0f);
  tone_source ref(sr, ch, frames, 440.0, 0.25);
  std::vector<float> g(frames * ch), r(frames * ch);
  for (int chunk = 0; chunk < 3; ++chunk) {
    auto pg = gained.fill(reinterpret_cast<std::uint8_t *>(g.data()), g.size() * sizeof(float));
    auto pr = ref.fill(reinterpret_cast<std::uint8_t *>(r.data()), r.size() * sizeof(float));
    ASSERT_EQ(pg.bytes, need);
    ASSERT_EQ(pr.bytes, need);
    for (int i = 0; i < frames * ch; ++i)
      EXPECT_NEAR(g[i], 2.0f * r[i], 1e-6f);
  }
}

TEST(AudioSources, MixSumsInputsWithClamp) {
  const int sr = 48000, ch = 1, frames = 128;
  const std::size_t need = audio_f32_bytes(frames, ch);
  std::vector<std::unique_ptr<frame_source>> ins;
  ins.push_back(std::make_unique<tone_source>(sr, ch, frames, 300.0, 0.2));
  ins.push_back(std::make_unique<tone_source>(sr, ch, frames, 700.0, 0.2));
  mix_source mix(std::move(ins), sr, ch, frames);
  // Reference tones in lockstep (same params/order); their sum stays within [-0.4, 0.4] (no clamp).
  tone_source a(sr, ch, frames, 300.0, 0.2), b(sr, ch, frames, 700.0, 0.2);
  std::vector<float> m(frames * ch), ra(frames * ch), rb(frames * ch);
  for (int chunk = 0; chunk < 3; ++chunk) {
    auto pm = mix.fill(reinterpret_cast<std::uint8_t *>(m.data()), m.size() * sizeof(float));
    a.fill(reinterpret_cast<std::uint8_t *>(ra.data()), ra.size() * sizeof(float));
    b.fill(reinterpret_cast<std::uint8_t *>(rb.data()), rb.size() * sizeof(float));
    ASSERT_EQ(pm.bytes, need);
    for (int i = 0; i < frames * ch; ++i)
      EXPECT_NEAR(m[i], ra[i] + rb[i], 1e-6f);
  }
}

// Review must-fix #1: a composite whose input disagrees on frame size must report end-of-stream
// LOUDLY, not discard every frame forever. gain over a 64-frame tone while itself expecting 128.
TEST(AudioSources, GainStopsOnSizeMismatchInsteadOfSkippingForever) {
  const int sr = 48000, ch = 1;
  gain_source gained(std::make_unique<tone_source>(sr, ch, /*frames*/ 64, 440.0, 0.25), ch,
                     /*frames*/ 128, 2.0f);
  std::vector<float> g(128 * ch);
  auto p = gained.fill(reinterpret_cast<std::uint8_t *>(g.data()), g.size() * sizeof(float));
  EXPECT_EQ(p.bytes, 0u);
  EXPECT_TRUE(p.stop) << "size-mismatched input must stop the gain, not skip silently forever";
}

// Review must-fix #2: a tick where every live input skips (produces 0 bytes but does not stop) must
// still emit silence with a MONOTONIC pts off the mix's own time base — never stuck at 0, so a
// stalling device input downstream can't make the output pts go backwards.
TEST(AudioSources, MixAdvancesPtsOnSilentTicks) {
  const int sr = 48000, ch = 1, frames = 128;
  const std::size_t need = audio_f32_bytes(frames, ch);
  std::vector<std::unique_ptr<frame_source>> ins;
  ins.push_back(std::make_unique<silent_source>(need)); // live, always skips
  mix_source mix(std::move(ins), sr, ch, frames);
  std::vector<float> m(frames * ch);
  double prev = -1.0;
  for (int chunk = 0; chunk < 3; ++chunk) {
    auto pm = mix.fill(reinterpret_cast<std::uint8_t *>(m.data()), m.size() * sizeof(float));
    ASSERT_EQ(pm.bytes, need) << "a live-but-skipping input still emits a silent frame";
    EXPECT_FALSE(pm.stop);
    EXPECT_GT(pm.pts_seconds, prev) << "pts must advance even on an all-skip tick";
    prev = pm.pts_seconds;
    for (int i = 0; i < frames * ch; ++i)
      EXPECT_FLOAT_EQ(m[i], 0.0f) << "skipped inputs contribute silence";
  }
}
