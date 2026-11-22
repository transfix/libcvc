/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// voice_monitor (cvc/ariadne/conference/voice_monitor.h): a conference client's jitter buffers,
// published for its document's Audio window, and tuned from it.

#include <algorithm>
#include <cvc/ariadne/conference/voice_monitor.h>
#include <cvc/core/app.h>
#include <cvc/media/voice_mixer.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace {

constexpr int kRate = 48000;

std::string get(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}
void set(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}

// 20 ms in step for SECONDS, from T: the voice plays steadily.
double play(cvc::voice_mixer &mx, cvc::ariadne::conference::voice_monitor &mon, double t,
            double seconds) {
  const std::vector<float> pkt(960, 0.25f);
  std::vector<float> out(960);
  for (const double end = t + seconds; t < end; t += 0.02) {
    mx.push("p1", pkt.data(), pkt.size(), t);
    mx.pull(out.data(), out.size(), t);
    mon.step(mx, t, 0.02, 0.06);
  }
  return t;
}

TEST(VoiceMonitor, DescribesItsTuningForASettingsEditor) {
  cvc::app app;
  cvc::ariadne::conference::voice_monitor mon(app, "doc");
  mon.describe();
  EXPECT_EQ(get(app, "doc.audio.jb.mode"), "adaptive");
  EXPECT_EQ(get(app, "doc.audio.jb.adaptive.start"), "0.08");
  auto &st = cvc::state::instance(app);
  EXPECT_EQ(st("doc.audio.jb.mode").property("choices"), "adaptive, fixed");
  EXPECT_EQ(st("doc.audio.jb.adaptive.max").property("unit"), "s");
  EXPECT_EQ(st("doc.audio.jb.adaptive.max").property("access"), "knob");
  EXPECT_EQ(st("doc.audio.jb.adaptive.max").property("default"), "0.4");
  EXPECT_FALSE(st("doc.audio.jb.adaptive.relax").comment().empty());

  // A document that set a value first keeps it.
  cvc::app app2;
  set(app2, "doc.audio.jb.adaptive.max", "0.25");
  cvc::ariadne::conference::voice_monitor mon2(app2, "doc");
  mon2.describe();
  EXPECT_EQ(get(app2, "doc.audio.jb.adaptive.max"), "0.25");
}

TEST(VoiceMonitor, PublishesEachVoicesHistory) {
  cvc::app app;
  cvc::ariadne::conference::voice_monitor mon(app, "doc", /*period_s=*/0.2, /*history=*/5);
  mon.describe();
  cvc::voice_mixer mx(kRate, 1);
  play(mx, mon, 0.0, 2.0);
  const std::string buffered = get(app, "doc.media.p1.jb.buffered");
  const std::string target = get(app, "doc.media.p1.jb.target");
  EXPECT_EQ(std::count(buffered.begin(), buffered.end(), ','), 4) << buffered << ": 5 samples";
  EXPECT_EQ(target, "80,80,80,80,80");
  EXPECT_NE(get(app, "doc.media.p1.jb.now").find("target 80 ms"), std::string::npos)
      << get(app, "doc.media.p1.jb.now");
  EXPECT_EQ(get(app, "doc.audio.device_ms"), "20");
  EXPECT_EQ(get(app, "doc.audio.ahead_ms"), "60");
  EXPECT_EQ(get(app, "doc.audio.window_s"), "1");
}

TEST(VoiceMonitor, AppliesTheTuning) {
  cvc::app app;
  cvc::ariadne::conference::voice_monitor mon(app, "doc");
  mon.describe();
  cvc::voice_mixer mx(kRate, 1);
  mx.set_delay_floor(0.04, 0.04); // a 20 ms device, as the host sets it
  double t = play(mx, mon, 0.0, 0.5);

  set(app, "doc.audio.jb.adaptive.start", "0.12");
  set(app, "doc.audio.jb.adaptive.max", "0.3");
  set(app, "doc.audio.jb.adaptive.relax", "4");
  t = play(mx, mon, t, 0.1);
  cvc::voice_mixer::params p = mx.parameters();
  EXPECT_NEAR(p.start_delay_s, 0.12, 1e-9);
  EXPECT_NEAR(p.max_delay_s, 0.3, 1e-9);
  EXPECT_NEAR(p.relax_after_s, 4.0, 1e-9);

  set(app, "doc.audio.jb.adaptive.min", "0.01"); // under the device's floor: the floor holds
  t = play(mx, mon, t, 0.1);
  EXPECT_NEAR(mx.parameters().min_delay_s, 0.04, 1e-9);

  set(app, "doc.audio.jb.mode", "fixed");
  set(app, "doc.audio.jb.fixed.delay", "0.2");
  t = play(mx, mon, t, 0.1);
  p = mx.parameters();
  EXPECT_NEAR(p.min_delay_s, 0.2, 1e-9);
  EXPECT_NEAR(p.max_delay_s, 0.2, 1e-9);
  cvc::voice_mixer::peer_stats st;
  ASSERT_TRUE(mx.stats("p1", st));
  EXPECT_NEAR(st.target_s, 0.2, 1e-9) << "a known voice is brought to the fixed target";
}

TEST(VoiceMonitor, ResetPutsTargetsBack) {
  cvc::app app;
  cvc::ariadne::conference::voice_monitor mon(app, "doc");
  mon.describe();
  cvc::voice_mixer mx(kRate, 1);
  double t = play(mx, mon, 0.0, 0.5);
  // A late underrun: it runs dry, the device goes 100 ms short, and the audio comes back.
  std::vector<float> out(960);
  for (int i = 0; i < 10; ++i, t += 0.02)
    mx.pull(out.data(), out.size(), t);
  t = play(mx, mon, t, 0.2);
  cvc::voice_mixer::peer_stats st;
  ASSERT_TRUE(mx.stats("p1", st));
  ASSERT_GT(st.target_s, 0.08);

  set(app, "doc.audio.reset", "1");
  mon.step(mx, t, 0.02, 0.0);
  ASSERT_TRUE(mx.stats("p1", st));
  EXPECT_NEAR(st.target_s, 0.08, 1e-9);
  EXPECT_EQ(get(app, "doc.audio.reset"), "0");
}

TEST(VoiceMonitor, ForgetsAVoiceThatLeft) {
  cvc::app app;
  cvc::ariadne::conference::voice_monitor mon(app, "", 0.2, 3);
  cvc::voice_mixer mx(kRate, 1);
  play(mx, mon, 0.0, 1.0);
  EXPECT_FALSE(get(app, "media.p1.jb.buffered").empty()) << "an empty prefix is the root";
  mx.remove("p1");
  mon.step(mx, 10.0, 0.02, 0.0);
  mx.push("p1", std::vector<float>(960, 0.25f).data(), 960, 10.0);
  mon.step(mx, 11.0, 0.02, 0.0);
  EXPECT_EQ(get(app, "media.p1.jb.buffered"), "20") << "a new history: one sample, its 20 ms";
}

} // namespace
