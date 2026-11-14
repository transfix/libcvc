/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
// A raster as a stream (cvc/ariadne/stream/raster.h, docs/ARIADNE.md §5.4 "Live textures"): the
// rgba8.patch frames, the consumer's canvas, the producer and its verbs, and its state mirror,
// followed on the same app and across a relay: a server document's raster in a published
// namespace, replicated to a client over an inproc transport and drawn there from the client's
// own stream.
#include <cmath>
#include <cstdint>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/server_document.h>
#include <cvc/ariadne/stream/raster.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/ariadne/stream_intrinsics.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport_inproc.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

namespace sx = cvc::ariadne::stream;
namespace fs = std::filesystem;

namespace {

// A w x h raster whose pixel (x, y) is (x, y, seed, 255).
std::vector<std::uint8_t> ramp(int w, int h, std::uint8_t seed) {
  std::vector<std::uint8_t> p(static_cast<std::size_t>(w) * h * 4);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      std::uint8_t *px = &p[(static_cast<std::size_t>(y) * w + x) * 4];
      px[0] = static_cast<std::uint8_t>(x);
      px[1] = static_cast<std::uint8_t>(y);
      px[2] = seed;
      px[3] = 255;
    }
  return p;
}

bool same_pixels(const std::uint8_t *a, const std::uint8_t *b, std::size_t n) {
  return std::equal(a, a + n, b);
}

// Every frame the subscription holds, applied to `c`; the updates.
std::vector<sx::raster_canvas::update> drain(sx::subscription &sub, sx::raster_canvas &c) {
  std::vector<sx::raster_canvas::update> out;
  sx::frame_ptr f;
  while (sub.try_pop(f)) {
    sx::raster_frame_view v;
    EXPECT_TRUE(sx::decode_raster_frame(f->data, f->size, v));
    out.push_back(c.apply(v));
  }
  return out;
}

} // namespace

TEST(StreamRaster, FrameRoundTripsAndRefusesBadBytes) {
  const int w = 16, h = 8;
  const std::vector<std::uint8_t> px = ramp(w, h, 7);
  // A delta: two rectangles, one clamped at the edge, one empty (dropped).
  const std::vector<std::uint8_t> bytes = sx::encode_raster_frame(
      px.data(), w, h, 5, false, {{2, 1, 3, 2}, {14, 6, 9, 9}, {0, 0, 0, 4}});
  sx::raster_frame_view v;
  ASSERT_TRUE(sx::decode_raster_frame(bytes.data(), bytes.size(), v));
  EXPECT_FALSE(v.key);
  EXPECT_EQ(v.number, 5u);
  EXPECT_EQ(v.w, w);
  EXPECT_EQ(v.h, h);
  ASSERT_EQ(v.changed.size(), 2u);
  ASSERT_EQ(v.patches.size(), 2u);
  EXPECT_EQ(v.patches[1].rect.x, 14);
  EXPECT_EQ(v.patches[1].rect.w, 2); // clamped to the raster
  EXPECT_EQ(v.patches[1].rect.h, 2);
  // Its pixels are the raster's, row by row.
  const sx::raster_patch &p = v.patches[0];
  for (int j = 0; j < p.rect.h; ++j)
    for (int i = 0; i < p.rect.w; ++i)
      EXPECT_EQ(p.pixels[(j * p.rect.w + i) * 4], static_cast<std::uint8_t>(p.rect.x + i));
  // Only the changed pixels travel: 6 + 4 pixels, not 128.
  EXPECT_LT(bytes.size(), 28u + 2 * 16 + 2 * 16 + 10 * 4 + 1);

  // A key frame carries the whole raster and still says what changed.
  const std::vector<std::uint8_t> key =
      sx::encode_raster_frame(px.data(), w, h, 6, true, {{2, 1, 3, 2}});
  ASSERT_TRUE(sx::decode_raster_frame(key.data(), key.size(), v));
  EXPECT_TRUE(v.key);
  ASSERT_EQ(v.patches.size(), 1u);
  EXPECT_EQ(v.patches[0].rect.w, w);
  ASSERT_EQ(v.changed.size(), 1u);
  EXPECT_TRUE(same_pixels(v.patches[0].pixels, px.data(), px.size()));

  // Bad bytes are refused, never read past.
  std::vector<std::uint8_t> bad = bytes;
  bad[0] = 'X';
  EXPECT_FALSE(sx::decode_raster_frame(bad.data(), bad.size(), v));
  EXPECT_FALSE(sx::decode_raster_frame(bytes.data(), bytes.size() - 1, v)); // truncated
  bad = bytes;
  bad[28 + 16 + 16 + 0] = 200; // the first patch's x, outside the raster
  EXPECT_FALSE(sx::decode_raster_frame(bad.data(), bad.size(), v));
  EXPECT_FALSE(sx::decode_raster_frame(nullptr, 0, v));
}

TEST(StreamRaster, CanvasAppliesInOrderAndWaitsForAKeyAfterAGap) {
  const int w = 8, h = 4;
  std::vector<std::uint8_t> px = ramp(w, h, 1);
  sx::raster_canvas c;
  sx::raster_frame_view v;
  std::vector<std::uint8_t> f = sx::encode_raster_frame(px.data(), w, h, 1, true, {{0, 0, w, h}});
  ASSERT_TRUE(sx::decode_raster_frame(f.data(), f.size(), v));
  sx::raster_canvas::update u = c.apply(v);
  EXPECT_TRUE(u.applied && u.resized && u.all) << "the first frame draws it all";
  EXPECT_TRUE(same_pixels(c.image().data(), px.data(), px.size()));

  // In order: only the rectangle.
  px[(1 * w + 2) * 4 + 2] = 99;
  f = sx::encode_raster_frame(px.data(), w, h, 2, false, {{2, 1, 1, 1}});
  ASSERT_TRUE(sx::decode_raster_frame(f.data(), f.size(), v));
  u = c.apply(v);
  ASSERT_TRUE(u.applied);
  EXPECT_FALSE(u.all);
  ASSERT_EQ(u.rects.size(), 1u);
  EXPECT_EQ(u.rects[0].x, 2);
  EXPECT_TRUE(same_pixels(c.image().data(), px.data(), px.size()));

  // A gap (frame 4 after 2): skipped, and so is what follows, until a key frame.
  px[0] = 50;
  f = sx::encode_raster_frame(px.data(), w, h, 4, false, {{0, 0, 1, 1}});
  ASSERT_TRUE(sx::decode_raster_frame(f.data(), f.size(), v));
  EXPECT_FALSE(c.apply(v).applied);
  f = sx::encode_raster_frame(px.data(), w, h, 5, false, {{0, 0, 1, 1}});
  ASSERT_TRUE(sx::decode_raster_frame(f.data(), f.size(), v));
  EXPECT_FALSE(c.apply(v).applied) << "still out of step";
  EXPECT_EQ(c.skipped(), 2u);
  f = sx::encode_raster_frame(px.data(), w, h, 6, true, {{0, 0, 1, 1}});
  ASSERT_TRUE(sx::decode_raster_frame(f.data(), f.size(), v));
  u = c.apply(v);
  EXPECT_TRUE(u.applied && u.all) << "a key frame after a gap draws it all";
  EXPECT_TRUE(same_pixels(c.image().data(), px.data(), px.size()));

  // A key frame in order uploads only what it says changed.
  px[(3 * w + 7) * 4] = 77;
  f = sx::encode_raster_frame(px.data(), w, h, 7, true, {{7, 3, 1, 1}});
  ASSERT_TRUE(sx::decode_raster_frame(f.data(), f.size(), v));
  u = c.apply(v);
  EXPECT_TRUE(u.applied);
  EXPECT_FALSE(u.all);
  ASSERT_EQ(u.rects.size(), 1u);
  EXPECT_TRUE(same_pixels(c.image().data(), px.data(), px.size()));
}

TEST(StreamRaster, ProducerSendsWhatWasPaintedAndKeysOnSchedule) {
  cvc::app app;
  sx::raster_params p;
  p.id = "tint";
  p.w = 64;
  p.h = 32;
  p.key_every = 4;
  std::string why;
  std::unique_ptr<sx::raster_stream> r = sx::raster_stream::open(app, p, &why);
  ASSERT_NE(r, nullptr) << why;
  EXPECT_EQ(r->token(), "streams.tint");
  sx::stream_channel *ch = sx::stream_registry::for_app(app).lookup(r->token());
  ASSERT_NE(ch, nullptr);
  EXPECT_EQ(ch->format().codec, sx::kRasterCodec);
  std::shared_ptr<sx::subscription> sub = ch->subscribe(sx::deliver_mode::ring, 8);
  ASSERT_NE(sub, nullptr);

  EXPECT_EQ(r->publish(), 0u) << "nothing painted, nothing sent";
  const std::uint8_t red[4] = {255, 0, 0, 255};
  r->fill({0, 0, 64, 32}, red);
  EXPECT_EQ(r->publish(), 1u);
  const std::uint64_t key_bytes = r->bytes_sent();
  const std::uint8_t blue[4] = {0, 0, 255, 255};
  r->fill({10, 10, 4, 4}, blue);
  EXPECT_EQ(r->publish(), 2u);
  const std::uint64_t delta_bytes = r->bytes_sent() - key_bytes;
  EXPECT_LT(delta_bytes * 20, key_bytes) << "a 4 x 4 change sends its 16 pixels, not 2048";

  sx::raster_canvas c;
  std::vector<sx::raster_canvas::update> us = drain(*sub, c);
  ASSERT_EQ(us.size(), 2u);
  EXPECT_TRUE(us[0].all);
  ASSERT_EQ(us[1].rects.size(), 1u);
  EXPECT_EQ(us[1].rects[0].w, 4);
  EXPECT_TRUE(same_pixels(c.image().data(), r->pixels(), 64 * 32 * 4));

  // Frames 3 is a delta and 4 a key frame (every 4th), which in order is still only its change.
  r->fill({0, 0, 1, 1}, blue);
  r->publish();
  r->fill({1, 0, 1, 1}, blue);
  EXPECT_EQ(r->publish(), 4u);
  us = drain(*sub, c);
  ASSERT_EQ(us.size(), 2u);
  EXPECT_FALSE(us[1].all);
  ASSERT_EQ(us[1].rects.size(), 1u);
  EXPECT_TRUE(same_pixels(c.image().data(), r->pixels(), 64 * 32 * 4));

  // A grid through a colour map; NaN is transparent.
  const double nan = std::nan("");
  ASSERT_TRUE(r->paint_grid({0.0, 1.0, nan, 0.5}, 2, 2, 30, 20, "gray", 0.0, 1.0, 200));
  const std::uint8_t *g = r->pixels() + (20 * 64 + 30) * 4;
  EXPECT_EQ(g[0], 0);
  EXPECT_EQ(g[3], 200);
  EXPECT_EQ(g[4], 255) << "1.0 is white on gray";
  EXPECT_EQ(r->pixels()[(21 * 64 + 30) * 4 + 3], 0) << "NaN";
  EXPECT_FALSE(r->paint_grid({1.0}, 2, 2, 0, 0, "gray", 0, 1)) << "not 2 x 2";
  EXPECT_FALSE(r->paint_grid({1.0}, 1, 1, 0, 0, "no-such-map", 0, 1));

  EXPECT_EQ(sx::raster_stream::open(app, p, &why), nullptr) << "the token is live";
  p.w = 0;
  p.id = "other";
  EXPECT_EQ(sx::raster_stream::open(app, p, &why), nullptr);
  EXPECT_NE(why.find("pixels a side"), std::string::npos);
}

// A mirror, followed on the same app: the follower's stream carries the producer's frames, and a
// follower that starts late begins at the last key frame.
TEST(StreamRaster, MirrorFollowedLocally) {
  cvc::app app;
  sx::raster_params p;
  p.id = "tint";
  p.w = 16;
  p.h = 16;
  p.key_every = 0;
  p.mirror = "fog.tint";
  std::unique_ptr<sx::raster_stream> r = sx::raster_stream::open(app, p);
  ASSERT_NE(r, nullptr);
  EXPECT_FALSE(sx::raster_mirror_present(app, "fog.nothing"));
  EXPECT_TRUE(sx::raster_mirror_present(app, "fog.tint"));

  const std::uint8_t a[4] = {10, 20, 30, 255}, b[4] = {200, 100, 50, 255};
  r->fill({0, 0, 16, 16}, a);
  r->publish(); // 1, the key frame
  r->fill({4, 4, 2, 2}, b);
  r->publish(); // 2

  std::shared_ptr<sx::raster_follower> f = sx::raster_follower::follow(app, "fog.tint");
  EXPECT_EQ(sx::raster_follower::follow(app, "fog.tint"), f) << "one follower per mirror";
  EXPECT_EQ(f->poll(), 2) << "its stream opens, and it takes frames 1 and 2 (no one listens yet)";
  EXPECT_EQ(f->number(), 2u) << "it read frames 1 and 2 into its canvas";
  sx::stream_channel *ch = sx::stream_registry::for_app(app).lookup(f->token());
  ASSERT_NE(ch, nullptr);
  std::shared_ptr<sx::subscription> sub = ch->subscribe(sx::deliver_mode::ring, 8);
  ASSERT_NE(sub, nullptr);
  r->fill({8, 8, 1, 1}, b);
  r->publish(); // 3
  r->fill({9, 9, 1, 1}, a);
  r->publish(); // 4
  EXPECT_EQ(f->poll(), 3) << "a key frame of its canvas for the new subscriber, then 3 and 4";
  sx::raster_canvas c;
  std::vector<sx::raster_canvas::update> us = drain(*sub, c);
  ASSERT_EQ(us.size(), 3u);
  EXPECT_TRUE(us[0].all) << "joined at a key frame";
  EXPECT_TRUE(us[1].applied && !us[1].all);
  EXPECT_TRUE(us[2].applied && !us[2].all);
  EXPECT_TRUE(same_pixels(c.image().data(), r->pixels(), 16 * 16 * 4));
  EXPECT_EQ(f->poll(), 0) << "nothing new";

  // More frames than the ring keeps between polls: it starts again at the next key frame.
  for (int k = 0; k < sx::kRasterRing + 2; ++k) {
    r->fill({k, 0, 1, 1}, b);
    r->publish();
  }
  EXPECT_EQ(f->poll(), 0) << "out of the ring's reach, and no newer key frame yet";
  r->publish_key();
  EXPECT_EQ(f->poll(), 1);
  drain(*sub, c);
  EXPECT_TRUE(same_pixels(c.image().data(), r->pixels(), 16 * 16 * 4));
  EXPECT_TRUE(c.synced());
}

// The verbs, from a document's action: open (again, as an on_tick would), paint, publish, close.
TEST(StreamRaster, DocumentVerbsPaintAndPublish) {
  cvc::app app;
  cvc::ariadne::register_stream_intrinsics(app);
  cvc::ariadne::AppRuntime art(app, "doc");
  const fs::path path = fs::temp_directory_path() /
                        ("stream_raster_verbs_" + std::to_string(std::random_device{}()) + ".ari");
  std::ofstream(path) << R"(meta: { min_libcvc: "0.0.0" }
on_tick: |
  (let ((r (raster-open (dict "id" "t" "w" 8 "h" 4 "mirror" "out.t"))))
    (raster-fill r (list 0 0 8 4) (list 1 2 3 255))
    (raster-grid r (list 2 1 2 1) (list 0 1) (dict "colormap" "gray" "lo" 0 "hi" 1))
    (state-set "n" (raster-publish r))
    (state-set "again" (raster-publish r))
    (state-set "token" (get-attr r "token"))
    (state-set "mirror" (get-attr r "mirror"))
    (state-set "bad" (get-attr (raster-grid r (list 0 0 2 2) (list 1)) "ok")))
)";
  const std::vector<std::string> warnings = art.load(path.string());
  for (const std::string &w : warnings)
    ADD_FAILURE() << w;
  art.drain();
  art.drain();
  cvc::state &root = cvc::state::instance(app);
  EXPECT_EQ(root("doc.token").value(), "doc.streams.t");
  EXPECT_EQ(root("doc.mirror").value(), "doc.out.t");
  EXPECT_EQ(root("doc.again").value(), "0") << "nothing painted since";
  EXPECT_EQ(root("doc.bad").value(), "false");
  const std::uint32_t n = static_cast<std::uint32_t>(std::stoul(root("doc.n").value()));
  EXPECT_GE(n, 1u) << "a frame each tick";
  EXPECT_EQ(root("doc.out.t.frame").value(), std::to_string(n));
  sx::raster_frame_view v;
  const std::string key = boost::any_cast<std::string>(root("doc.out.t.key").data());
  ASSERT_TRUE(
      sx::decode_raster_frame(reinterpret_cast<const std::uint8_t *>(key.data()), key.size(), v));
  ASSERT_EQ(v.patches.size(), 1u);
  const std::uint8_t *px = v.patches[0].pixels;
  EXPECT_EQ(px[0], 1);
  EXPECT_EQ(px[2], 3);
  EXPECT_EQ(px[(1 * 8 + 3) * 4], 255) << "the grid's 1, white";
  fs::remove(path);
}

// The relay round trip: a server document's raster, mirrored into the namespace it publishes,
// replicated to a client over an inproc transport, and drawn there from the client's own stream.
TEST(StreamRaster, ServerDocumentRasterReachesARelayClient) {
  if (!cvc::ariadne::AppRuntime::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  const fs::path path = fs::temp_directory_path() /
                        ("stream_raster_server_" + std::to_string(std::random_device{}()) + ".ari");
  std::ofstream(path) << R"(meta: { name: raster server, min_libcvc: "0.0.0" }
server:
  publish: [fog]
on_tick: |
  (let ((r (raster-open (dict "id" "tint" "w" 32 "h" 16 "mirror" "fog.tint" "key_every" 0)))
        (k (int (state-get "sim.tick"))))
    (raster-fill r (list (% k 32) (% k 16) 2 2) (list 200 30 10 180))
    (raster-publish r))
)";
  cvc::app server, client;
  cvc::state::instance(server)("conf.rooms.standup.topic").value(std::string("up"));
  cvc::ariadne::register_stream_intrinsics(server);
  cvc::state_cluster_shard relay(server, "C", "relay");
  cvc::state_cluster_shard peer(client, "C", "client");
  cvc::ariadne::ServerDocument doc(server, "standup");
  ASSERT_EQ(doc.read(path.string()), "");
  doc.scope(relay);
  relay.attach();
  peer.attach();
  cvc::state_transport_inproc tp;
  tp.register_shard(&relay);
  tp.register_shard(&peer);
  for (const std::string &w : doc.load(path.string()))
    ADD_FAILURE() << "warning: " << w;

  std::shared_ptr<sx::raster_follower> follower;
  std::shared_ptr<sx::subscription> sub;
  sx::raster_canvas canvas;
  for (int step = 0; step < 24; ++step) {
    doc.step(240 + 2 * step, (240 + 2 * step) / 120.0); // the relay's room step
    tp.pump_all();                                      // the relay sends; the client applies
    if (!follower && sx::raster_mirror_present(client, "fog.tint"))
      follower = sx::raster_follower::follow(client, "fog.tint");
    if (follower) {
      follower->poll();
      if (!sub)
        if (sx::stream_channel *ch = sx::stream_registry::for_app(client).lookup(follower->token()))
          sub = ch->subscribe(sx::deliver_mode::ring, 8);
      if (sub)
        drain(*sub, canvas);
    }
  }
  ASSERT_NE(follower, nullptr) << "the mirror reached the client";
  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(cvc::state::instance(client)("fog.tint.codec").value(), sx::kRasterCodec);
  EXPECT_GT(follower->number(), 10u);
  EXPECT_TRUE(canvas.synced());
  // The client's raster is the server's, pixel for pixel.
  sx::stream_channel *sch =
      sx::stream_registry::for_app(server).lookup("ui.docs.arisrv.streams.tint");
  ASSERT_NE(sch, nullptr) << "the server document's own stream";
  const std::string key = boost::any_cast<std::string>(
      cvc::state::instance(server)("fog.tint.ring." +
                                   std::to_string(follower->number() % sx::kRasterRing))
          .data());
  sx::raster_frame_view last;
  ASSERT_TRUE(sx::decode_raster_frame(reinterpret_cast<const std::uint8_t *>(key.data()),
                                      key.size(), last));
  EXPECT_EQ(last.number, follower->number());
  EXPECT_EQ(canvas.number(), follower->number());
  // A pixel the server painted at step 3 (tick 246: x 22, y 6), as the client has it.
  const std::uint8_t *px = canvas.image().data() + (6 * 32 + 22) * 4;
  EXPECT_EQ(px[0], 200);
  EXPECT_EQ(px[3], 180);
  EXPECT_EQ(canvas.image().data()[(15 * 32 + 0) * 4 + 3], 0) << "never painted: clear";
  peer.detach();
  relay.detach();
  fs::remove(path);
}
