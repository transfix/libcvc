/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 1 tests for the cvc::stream in-process real-time frame transport
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
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/async_scheduler.h>
#include <cvc/core/state_exec/types.h>
#include <cvc/stream/frame.h>
#include <cvc/stream/frame_pool.h>
#include <cvc/stream/producer_thread.h>
#include <cvc/stream/stream.h>
#include <cvc/stream/stream_channel.h>
#include <cvc/stream/stream_registry.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::stream;

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

  // Registry resolves the token to the live channel.
  EXPECT_EQ(stream_registry::for_app(app).lookup("cam0"), &s->channel());

  s->close();
  EXPECT_EQ(cvc::state::instance(app)("streams.cam0").value(), "closed");
  EXPECT_EQ(stream_registry::for_app(app).lookup("cam0"), nullptr);
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
  EXPECT_EQ(stream_registry::for_app(app).lookup("dup0"), &s1->channel());

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
