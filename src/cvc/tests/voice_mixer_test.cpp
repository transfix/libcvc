/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// voice_mixer (cvc/media/voice_mixer.h): the conference's per-peer jitter buffers and mix. The
// simulations play a sender and a device on one clock: the sender's audio lands in blocks with a
// random delay (in order, as over one TCP connection), and the device pulls at its own pace.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/media/voice_mixer.h>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr int kRate = 48000;

std::vector<float> tone(std::size_t frames, float amp = 0.5f) {
  std::vector<float> v(frames);
  for (std::size_t i = 0; i < frames; ++i)
    v[i] = amp * static_cast<float>(std::sin(2.0 * 3.14159265358979 * 440.0 * i / kRate));
  return v;
}

std::vector<float> constant(std::size_t frames, float value) {
  return std::vector<float>(frames, value);
}

struct sim_result {
  std::uint64_t underruns = 0, late_underruns = 0, starts = 0;
  std::uint64_t played = 0, sent = 0;
  double target_s = 0.0;
};

// What one link looks like: talk spurts of SPURT_S seconds with GAP_S of silence between them
// (the sender sends nothing through a silence), audio captured in blocks of BLOCK frames and
// landing BASE + U(0, JITTER) after capture, plus SPIKE seconds more for a SPIKE_P share of them
// (audio stuck behind a camera frame on the same connection). It lands in order (one TCP
// connection), and the receiver sees it at its next frame (FRAME_S). A device pulls PULL frames
// at a time.
struct link_shape {
  double seconds = 30.0, spurt_s = 3.0, gap_s = 1.2;
  std::size_t block = 2048;
  double base = 0.03, jitter = 0.04, spike = 0.06, spike_p = 0.1;
  double frame_s = 1.0 / 60.0;
  std::size_t pull = 2048;
};

sim_result simulate(cvc::voice_mixer &mx, const link_shape &l, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  const double block_s = static_cast<double>(l.block) / kRate;
  const double pull_s = static_cast<double>(l.pull) / kRate;
  const std::vector<float> b = constant(l.block, 0.25f);
  std::vector<float> out(l.pull);
  sim_result r;
  // The blocks sent, with the time each one is seen by the receiver.
  std::vector<double> seen;
  double last = 0.0;
  for (double t = block_s; t <= l.seconds; t += block_s) {
    if (std::fmod(t, l.spurt_s + l.gap_s) > l.spurt_s)
      continue; // a silence: nothing sent
    double at = t + l.base + l.jitter * u(rng) + (u(rng) < l.spike_p ? l.spike : 0.0);
    at = std::max(at, last);
    last = at;
    seen.push_back(std::ceil(at / l.frame_s) * l.frame_s);
  }
  std::size_t next = 0;
  for (double t = 0.5 * pull_s; t < l.seconds + 1.0; t += pull_s) {
    for (; next < seen.size() && seen[next] <= t; ++next) {
      mx.push("a", b.data(), b.size(), seen[next]);
      r.sent += l.block;
    }
    mx.pull(out.data(), l.pull, t);
  }
  cvc::voice_mixer::peer_stats st;
  EXPECT_TRUE(mx.stats("a", st));
  r.underruns = st.underruns;
  r.late_underruns = st.late_underruns;
  r.starts = st.starts;
  r.played = st.played_frames;
  r.target_s = st.target_s;
  return r;
}

cvc::voice_mixer::params no_jitter_buffer() {
  // The old mixer: a voice plays as soon as any of it is there.
  cvc::voice_mixer::params none;
  none.start_delay_s = none.min_delay_s = 0.0;
  none.step_s = 0.0;
  return none;
}

// A browser's link: 43 ms blocks pulled by a 43 ms device. Played as it lands (the old mixer) the
// voice runs dry over and over, in every talk spurt; the jitter buffer grows to the link once and
// then plays the rest through.
TEST(VoiceMixer, JitterBufferRidesOutBurstyDelivery) {
  const link_shape l;
  cvc::voice_mixer naive(kRate, 1, no_jitter_buffer());
  const sim_result bad = simulate(naive, l, 7);

  cvc::voice_mixer mx(kRate, 1);
  mx.set_delay_floor(l.pull / static_cast<double>(kRate) + 0.02,
                     2.0 * l.pull / static_cast<double>(kRate)); // as the host does
  const sim_result good = simulate(mx, l, 7);

  const std::uint64_t spurts = good.underruns - good.late_underruns; // one dry end each
  EXPECT_GE(spurts, 6u);
  EXPECT_GE(bad.late_underruns, 3 * spurts) << "several late underruns a spurt without a buffer";
  EXPECT_LE(good.late_underruns, 4u) << "a few while the target grows, then none";
  EXPECT_LE(good.target_s, 0.25);
  EXPECT_EQ(good.played, good.sent) << "nothing dropped: the delay grew instead";
}

// The same with a native client's shapes: 20 ms packets and a 10 ms device.
TEST(VoiceMixer, SmallPacketsSmallDevice) {
  link_shape l;
  l.block = 960;
  l.pull = 480;
  l.jitter = 0.03;
  cvc::voice_mixer naive(kRate, 1, no_jitter_buffer());
  const sim_result bad = simulate(naive, l, 11);
  cvc::voice_mixer mx(kRate, 1);
  const sim_result good = simulate(mx, l, 11);
  EXPECT_GT(bad.late_underruns, 10u);
  EXPECT_LE(good.late_underruns, 3u);
  EXPECT_EQ(good.played, good.sent);
}

TEST(VoiceMixer, StartsAtTheTarget) {
  cvc::voice_mixer::params p;
  p.start_delay_s = 0.06;
  cvc::voice_mixer mx(kRate, 1, p);
  std::vector<float> out(480);
  const std::vector<float> pkt = constant(960, 0.5f); // 20 ms
  mx.push("a", pkt.data(), pkt.size(), 0.00);
  mx.push("a", pkt.data(), pkt.size(), 0.02);
  EXPECT_FALSE(mx.pull(out.data(), out.size(), 0.03)); // 40 ms buffered, short of 60
  EXPECT_FLOAT_EQ(out[300], 0.0f);
  mx.push("a", pkt.data(), pkt.size(), 0.04);
  EXPECT_TRUE(mx.pull(out.data(), out.size(), 0.05)); // 60 ms: it starts
  EXPECT_LT(out[0], 0.1f) << "the start ramps up";
  EXPECT_FLOAT_EQ(out[300], 0.5f);
  cvc::voice_mixer::peer_stats st;
  ASSERT_TRUE(mx.stats("a", st));
  EXPECT_TRUE(st.playing);
  EXPECT_EQ(st.starts, 1u);
  EXPECT_NEAR(mx.buffered_s("a"), 0.05, 1e-9);
}

// A late underrun (its audio came back soon) grows the target; a talk spurt's end (nothing came
// back) leaves it.
TEST(VoiceMixer, OnlyLateUnderrunsGrowTheTarget) {
  cvc::voice_mixer::params p;
  p.start_delay_s = 0.04;
  p.min_delay_s = 0.04;
  p.step_s = 0.02;
  cvc::voice_mixer mx(kRate, 1, p);
  std::vector<float> out(960);
  const std::vector<float> pkt = constant(1920, 0.5f); // 40 ms
  mx.push("a", pkt.data(), pkt.size(), 0.0);
  mx.pull(out.data(), out.size(), 0.01);
  mx.pull(out.data(), out.size(), 0.03);
  mx.pull(out.data(), out.size(), 0.05); // dry
  cvc::voice_mixer::peer_stats st;
  ASSERT_TRUE(mx.stats("a", st));
  EXPECT_EQ(st.underruns, 1u);
  EXPECT_FALSE(st.playing);
  mx.push("a", pkt.data(), pkt.size(), 0.10); // back 50 ms later: it was late
  ASSERT_TRUE(mx.stats("a", st));
  EXPECT_EQ(st.late_underruns, 1u);
  EXPECT_NEAR(st.target_s, 0.06, 1e-9);

  // Play that out and stop sending: the end of a spurt.
  for (double t = 0.11; t < 0.70; t += 0.02)
    mx.pull(out.data(), out.size(), t);
  mx.push("a", pkt.data(), pkt.size(), 2.0); // the next spurt, long after
  ASSERT_TRUE(mx.stats("a", st));
  EXPECT_EQ(st.underruns, 2u);
  EXPECT_EQ(st.late_underruns, 1u);
  EXPECT_NEAR(st.target_s, 0.06, 1e-9);
}

// A spurt shorter than the target plays once its audio stops coming, instead of waiting for (and
// running into) the next one.
TEST(VoiceMixer, ShortSpurtPlaysOut) {
  cvc::voice_mixer mx(kRate, 1); // target 80 ms
  std::vector<float> out(480);
  const std::vector<float> pkt = constant(960, 0.5f);
  mx.push("a", pkt.data(), pkt.size(), 0.0);
  EXPECT_FALSE(mx.pull(out.data(), out.size(), 0.1));
  EXPECT_TRUE(mx.pull(out.data(), out.size(), 0.5)); // 0.5 s with nothing more: play it
  EXPECT_FLOAT_EQ(out[240], 0.5f);
}

// A burst (a stalled sender catching up) is cut back to the target, oldest first.
TEST(VoiceMixer, BurstIsTrimmedToTheTarget) {
  cvc::voice_mixer::params p;
  p.start_delay_s = 0.06;
  p.trim_slack_s = 0.1;
  cvc::voice_mixer mx(kRate, 1, p);
  std::vector<float> burst = constant(kRate / 2, 0.1f); // 500 ms
  std::fill(burst.end() - 2880, burst.end(), 0.7f);     // its newest 60 ms
  mx.push("a", burst.data(), burst.size(), 0.0);
  std::vector<float> out(480);
  mx.pull(out.data(), out.size(), 0.0);
  EXPECT_FLOAT_EQ(out[240], 0.7f) << "the oldest audio went";
  cvc::voice_mixer::peer_stats st;
  ASSERT_TRUE(mx.stats("a", st));
  EXPECT_EQ(st.trimmed_frames, static_cast<std::uint64_t>(kRate / 2 - 2880));
  EXPECT_NEAR(st.buffered_s, 0.05, 1e-9);
}

// A trim while a voice plays cross-fades from what would have played into what it keeps, so the
// cut does not click.
TEST(VoiceMixer, TrimCrossFades) {
  cvc::voice_mixer::params p;
  p.start_delay_s = 0.06;
  p.trim_slack_s = 0.1;
  cvc::voice_mixer mx(kRate, 1, p);
  const std::vector<float> old_audio = constant(2880, 0.1f), burst = constant(kRate / 2, 0.7f);
  std::vector<float> out(480);
  mx.push("a", old_audio.data(), old_audio.size(), 0.0);
  mx.pull(out.data(), out.size(), 0.0); // playing
  mx.push("a", burst.data(), burst.size(), 0.01);
  mx.pull(out.data(), out.size(), 0.01);
  EXPECT_LT(out[0], 0.2f) << "starts from the audio cut";
  EXPECT_GT(out[60], 0.3f);
  EXPECT_LT(out[60], 0.5f) << "half way through the cross-fade";
  EXPECT_FLOAT_EQ(out[240], 0.7f) << "then the audio kept";
  cvc::voice_mixer::peer_stats st;
  ASSERT_TRUE(mx.stats("a", st));
  EXPECT_EQ(st.trimmed_frames, static_cast<std::uint64_t>(2400 + kRate / 2 - 2880));
}

TEST(VoiceMixer, OverflowCapsTheBuffer) {
  cvc::voice_mixer::params p;
  p.max_buffer_s = 0.5;
  cvc::voice_mixer mx(kRate, 1, p);
  const std::vector<float> sec = constant(kRate, 0.1f);
  mx.push("a", sec.data(), sec.size(), 0.0);
  cvc::voice_mixer::peer_stats st;
  ASSERT_TRUE(mx.stats("a", st));
  EXPECT_EQ(st.overflow_frames, static_cast<std::uint64_t>(kRate / 2));
  EXPECT_NEAR(st.buffered_s, 0.5, 1e-9);
}

TEST(VoiceMixer, MixesPeersWithGainAndClamps) {
  cvc::voice_mixer::params p;
  p.start_delay_s = p.min_delay_s = 0.0;
  cvc::voice_mixer mx(kRate, 1, p);
  const std::vector<float> a = constant(960, 0.4f), b = constant(960, 0.3f);
  mx.push("a", a.data(), a.size(), 0.0);
  mx.push("b", b.data(), b.size(), 0.0);
  mx.set_gain("b", 2.0f);
  std::vector<float> out(480);
  ASSERT_TRUE(mx.pull(out.data(), out.size(), 0.0));
  EXPECT_FLOAT_EQ(out[300], 1.0f) << "0.4 + 0.6 clamps at 1";
  mx.set_gain("b", 0.0f);
  ASSERT_TRUE(mx.pull(out.data(), out.size(), 0.01));
  EXPECT_FLOAT_EQ(out[300], 0.4f) << "a peer at gain 0 is consumed unheard";
  EXPECT_NEAR(mx.buffered_s("b"), 0.0, 1e-9);
  mx.remove("a");
  EXPECT_EQ(mx.peers(), std::vector<std::string>{"b"});
}

TEST(VoiceMixer, StereoKeepsFramesWhole) {
  cvc::voice_mixer::params p;
  p.start_delay_s = p.min_delay_s = 0.0;
  cvc::voice_mixer mx(kRate, 2, p);
  std::vector<float> lr(2 * 480 + 1); // a trailing half frame is dropped
  for (std::size_t i = 0; i + 1 < lr.size(); i += 2) {
    lr[i] = 0.25f;
    lr[i + 1] = -0.25f;
  }
  mx.push("a", lr.data(), lr.size(), 0.0);
  EXPECT_NEAR(mx.buffered_s("a"), 0.01, 1e-9);
  std::vector<float> out(2 * 240);
  mx.pull(out.data(), 240, 0.0);
  EXPECT_FLOAT_EQ(out[400], 0.25f); // frame 200, past the start's ramp
  EXPECT_FLOAT_EQ(out[401], -0.25f);
}

// After relax_after_s playing without running dry, the target comes back down a step, never below
// the floor.
TEST(VoiceMixer, TargetRelaxes) {
  cvc::voice_mixer::params p;
  p.start_delay_s = 0.10;
  p.min_delay_s = 0.08;
  p.step_s = 0.02;
  p.relax_after_s = 1.0;
  cvc::voice_mixer mx(kRate, 1, p);
  const std::vector<float> pkt = tone(960);
  std::vector<float> out(960);
  double t = 0.0;
  for (int i = 0; i < 6; ++i) // prime 120 ms
    mx.push("a", pkt.data(), pkt.size(), t);
  for (int i = 0; i < 200; ++i, t += 0.02) { // 4 s, in step
    mx.push("a", pkt.data(), pkt.size(), t);
    mx.pull(out.data(), out.size(), t);
  }
  cvc::voice_mixer::peer_stats st;
  ASSERT_TRUE(mx.stats("a", st));
  EXPECT_EQ(st.underruns, 0u);
  EXPECT_NEAR(st.target_s, 0.08, 1e-9);
}

TEST(VoiceMixer, DelayFloorRaisesTargets) {
  cvc::voice_mixer mx(kRate, 1);
  const std::vector<float> pkt = constant(960, 0.1f);
  mx.push("a", pkt.data(), pkt.size(), 0.0);
  mx.set_delay_floor(0.064, 0.10);
  cvc::voice_mixer::peer_stats st;
  ASSERT_TRUE(mx.stats("a", st));
  EXPECT_NEAR(st.target_s, 0.08, 1e-9) << "a known peer keeps its higher target";
  EXPECT_NEAR(mx.parameters().start_delay_s, 0.10, 1e-9);
  EXPECT_NEAR(mx.parameters().min_delay_s, 0.064, 1e-9);
  mx.push("b", pkt.data(), pkt.size(), 0.0);
  ASSERT_TRUE(mx.stats("b", st));
  EXPECT_NEAR(st.target_s, 0.10, 1e-9);
}

} // namespace
