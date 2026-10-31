/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Per-viewer video rungs (media_rungs.h): the relay's JPEG downscale, and the shaper that sends
// each viewer a camera at its rung -- as sent, smaller (one variant per frame and rung, shared),
// at a thumbnail rate, or not at all. And the Opus codec the mics use (media_opus.h).

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/media/media_opus.h>
#include <cvc/media/media_rungs.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

namespace {

// A WxH camera-like frame: a gradient with some texture, as JPEG.
std::vector<std::uint8_t> camera_jpeg(int w, int h) {
  std::vector<std::uint8_t> rgb(static_cast<std::size_t>(w) * h * 3);
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      std::uint8_t *p = &rgb[(static_cast<std::size_t>(y) * w + x) * 3];
      p[0] = static_cast<std::uint8_t>(x * 255 / w);
      p[1] = static_cast<std::uint8_t>(y * 255 / h);
      p[2] = static_cast<std::uint8_t>(((x / 8 + y / 8) % 2) * 160 + (x * y) % 64);
    }
  return cvc::jpeg_encode_rgb(rgb.data(), w, h, 60);
}

cvc::media_frame camera_frame(const std::string &origin, std::int64_t seq) {
  cvc::media_frame f;
  f.origin_node_id = origin;
  f.stream_id = "camera";
  f.codec = "mjpeg";
  f.width = 320;
  f.height = 240;
  f.seq = seq;
  f.payload = camera_jpeg(320, 240);
  return f;
}

// N frames of a 440 Hz tone at 48 kHz mono, from sample START on, like a mic chunk.
std::vector<float> tone(int start, int n) {
  std::vector<float> s(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i)
    s[static_cast<std::size_t>(i)] =
        0.3f * static_cast<float>(std::sin(2.0 * 3.14159265358979 * 440.0 * (start + i) / 48000.0));
  return s;
}

double rms(const std::vector<float> &s) {
  double sum = 0.0;
  for (float v : s)
    sum += static_cast<double>(v) * v;
  return s.empty() ? 0.0 : std::sqrt(sum / static_cast<double>(s.size()));
}

} // namespace

TEST(MediaRungs, NamesRoundTrip) {
  for (int i = 0; i < cvc::kVideoRungs; ++i) {
    const auto r = static_cast<cvc::video_rung>(i);
    cvc::video_rung back = cvc::video_rung::off;
    ASSERT_TRUE(cvc::parse_video_rung(cvc::video_rung_name(r), back)) << i;
    EXPECT_EQ(back, r);
  }
  cvc::video_rung r = cvc::video_rung::half;
  EXPECT_FALSE(cvc::parse_video_rung("medium", r));
  EXPECT_EQ(r, cvc::video_rung::half) << "untouched";
}

TEST(MediaRungs, AJpegIsDownscaledByHalvesAndShrinks) {
  if (!cvc::jpeg_transcode_available())
    GTEST_SKIP() << "no libjpeg-turbo in this build";
  const std::vector<std::uint8_t> full = camera_jpeg(320, 240);
  ASSERT_FALSE(full.empty());
  int w = 0, h = 0;
  const std::vector<std::uint8_t> half = cvc::jpeg_downscale(full.data(), full.size(), 2, 55, w, h);
  EXPECT_EQ(w, 160);
  EXPECT_EQ(h, 120);
  const std::vector<std::uint8_t> quarter =
      cvc::jpeg_downscale(full.data(), full.size(), 4, 50, w, h);
  EXPECT_EQ(w, 80);
  EXPECT_EQ(h, 60);
  EXPECT_LT(half.size(), full.size());
  EXPECT_LT(quarter.size(), half.size());
  EXPECT_TRUE(cvc::jpeg_downscale(full.data(), full.size(), 3, 50, w, h).empty()) << "1/3 is no "
                                                                                     "DCT scale";
  const std::uint8_t junk[] = {1, 2, 3, 4};
  EXPECT_TRUE(cvc::jpeg_downscale(junk, sizeof(junk), 2, 50, w, h).empty()) << "not a JPEG";

  // A header claiming 65535 x 65535 is refused before anything is allocated for it.
  std::vector<std::uint8_t> huge = full;
  bool patched = false;
  for (std::size_t i = 0; i + 8 < huge.size(); ++i)
    if (huge[i] == 0xFF && huge[i + 1] == 0xC0) { // SOF0: length(2) precision(1) height(2) width(2)
      huge[i + 5] = huge[i + 6] = huge[i + 7] = huge[i + 8] = 0xFF;
      patched = true;
      break;
    }
  ASSERT_TRUE(patched) << "a baseline JPEG has an SOF0 marker";
  EXPECT_TRUE(cvc::jpeg_downscale(huge.data(), huge.size(), 2, 50, w, h).empty());
  // Nor does a bad frame spoil the next: the decoder it failed in is replaced.
  EXPECT_EQ(cvc::jpeg_downscale(full.data(), full.size(), 2, 55, w, h).size(), half.size());
}

// A portrait phone's frame (its shape kept within the 320x240 budget, turned upright) keeps that
// shape on every rung: the rung reads the JPEG's own size, not the stream's 320x240.
TEST(MediaRungs, APortraitFrameKeepsItsShapeOnEveryRung) {
  if (!cvc::jpeg_transcode_available())
    GTEST_SKIP() << "no libjpeg-turbo in this build";
  cvc::media_rung_shaper shaper;
  shaper.set_rung("small", cvc::video_rung::half);
  cvc::media_frame f = camera_frame("phone", 1);
  f.payload = camera_jpeg(180, 320);
  const cvc::media_shape v = shaper.shape(f, "small");
  ASSERT_NE(v.variant, nullptr);
  EXPECT_EQ(v.variant->width, 90u);
  EXPECT_EQ(v.variant->height, 160u);
}

// Each viewer gets the camera at its rung. A smaller rung's variant is made once per frame and
// shared by every viewer on it; the mic and other codecs always go as sent.
TEST(MediaRungs, TheShaperSendsEachViewerItsRung) {
  cvc::media_rung_shaper shaper;
  shaper.set_rung("half1", cvc::video_rung::half);
  shaper.set_rung("half2", cvc::video_rung::half);
  shaper.set_rung("none", cvc::video_rung::off);
  EXPECT_EQ(shaper.rung("someone"), cvc::video_rung::full) << "full until set";

  const cvc::media_frame f = camera_frame("A", 1);
  const cvc::media_shape full = shaper.shape(f, "someone");
  EXPECT_FALSE(full.drop);
  EXPECT_EQ(full.variant, nullptr) << "full: as sent";
  EXPECT_TRUE(shaper.shape(f, "none").drop) << "off: nothing";

  const cvc::media_shape a = shaper.shape(f, "half1");
  const cvc::media_shape b = shaper.shape(f, "half2");
  if (cvc::jpeg_transcode_available()) {
    ASSERT_NE(a.variant, nullptr);
    EXPECT_EQ(a.variant, b.variant) << "one variant per frame and rung, shared";
    EXPECT_EQ(a.variant->width, 160u);
    EXPECT_EQ(a.variant->seq, 1);
    EXPECT_LT(a.variant->payload.size(), f.payload.size());
    EXPECT_EQ(shaper.total_transcoded(), 1u);
    EXPECT_EQ(shaper.total_shaped(), 2u);
    EXPECT_GT(shaper.total_bytes_saved(), 0u);
    const cvc::media_shape next = shaper.shape(camera_frame("A", 2), "half1");
    ASSERT_NE(next.variant, nullptr);
    EXPECT_EQ(next.variant->seq, 2) << "the next frame gets its own";
  } else {
    EXPECT_EQ(a.variant, nullptr) << "without libjpeg-turbo a smaller rung sends as is";
  }

  cvc::media_frame mic;
  mic.origin_node_id = "A";
  mic.stream_id = "mic";
  mic.codec = "f32";
  mic.payload.assign(4096, 0);
  const cvc::media_shape m = shaper.shape(mic, "none");
  EXPECT_FALSE(m.drop) << "audio always flows, whatever the video rung";
  EXPECT_EQ(m.variant, nullptr);
}

// The thumb rung sends a viewer at most thumb_fps frames a second from each publisher.
TEST(MediaRungs, TheThumbRungPacesEachPublisher) {
  cvc::media_rung_shaper::config c;
  c.thumb_fps = 5.0; // a frame every 200 ms
  cvc::media_rung_shaper shaper(c);
  shaper.set_rung("v", cvc::video_rung::thumb);
  EXPECT_FALSE(shaper.shape(camera_frame("A", 1), "v").drop) << "the first goes";
  EXPECT_TRUE(shaper.shape(camera_frame("A", 2), "v").drop) << "the next, too soon, does not";
  std::this_thread::sleep_for(std::chrono::milliseconds(170));
  EXPECT_FALSE(shaper.shape(camera_frame("A", 3), "v").drop)
      << "a little early still goes: an AOI-paced camera arrives with jitter";
  EXPECT_TRUE(shaper.shape(camera_frame("A", 4), "v").drop);
  EXPECT_FALSE(shaper.shape(camera_frame("B", 1), "v").drop)
      << "another publisher has its own pace";
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  const cvc::media_shape later = shaper.shape(camera_frame("A", 5), "v");
  EXPECT_FALSE(later.drop) << "once the interval has passed";
  if (cvc::jpeg_transcode_available()) {
    ASSERT_NE(later.variant, nullptr);
    EXPECT_EQ(later.variant->width, 80u) << "thumbnails are quarter size";
  }
  shaper.forget_viewer("v");
  EXPECT_EQ(shaper.rung("v"), cvc::video_rung::full) << "a viewer that left is forgotten";
}

// The mic's 1024-frame chunks go out as 20 ms (960-frame) packets at speech size, and come back
// as the same sound.
TEST(MediaOpus, MicChunksRoundTripAsSmallPackets) {
  if (!cvc::opus_available())
    GTEST_SKIP() << "no libopus in this build";
  cvc::opus_encoder enc(48000, 1, 32000);
  cvc::opus_decoder dec(48000, 1);
  ASSERT_TRUE(enc.valid());
  ASSERT_TRUE(dec.valid());
  std::vector<cvc::opus_packet> packets;
  for (int c = 0; c < 15; ++c) { // 15 chunks = 15360 frames = 16 packets
    const std::vector<float> chunk = tone(c * 1024, 1024);
    for (cvc::opus_packet &p : enc.encode(chunk.data(), 1024))
      packets.push_back(std::move(p));
  }
  ASSERT_EQ(packets.size(), 16u) << "a packet per 20 ms, the remainder kept for the next chunk";
  std::size_t bytes = 0;
  std::vector<float> out;
  for (const cvc::opus_packet &p : packets) {
    EXPECT_EQ(p.frames, 960);
    bytes += p.bytes.size();
    const std::vector<float> s = dec.decode(p.bytes.data(), p.bytes.size());
    EXPECT_EQ(s.size(), 960u);
    out.insert(out.end(), s.begin(), s.end());
  }
  EXPECT_LT(bytes / packets.size(), 200u) << "about 80 bytes a packet at 32 kbit/s, not 3840";
  // Past the codec's warm-up, the tone comes back at its level.
  const std::vector<float> tail(out.begin() + 4800, out.end());
  EXPECT_NEAR(rms(tail), rms(tone(0, 9600)), 0.05);
  EXPECT_TRUE(dec.decode(nullptr, 0).empty());
  const std::uint8_t junk[] = {0xff, 0xff, 0xff};
  EXPECT_TRUE(dec.decode(junk, sizeof(junk)).empty()) << "not a packet";
  EXPECT_EQ(dec.conceal(960).size(), 960u) << "a lost packet's stand-in";
}

// reset() drops the remainder: a talk spurt does not begin with the end of the last.
TEST(MediaOpus, AResetEncoderStartsAfresh) {
  if (!cvc::opus_available())
    GTEST_SKIP() << "no libopus in this build";
  cvc::opus_encoder enc(48000, 1, 32000);
  const std::vector<float> half = tone(0, 500);
  EXPECT_TRUE(enc.encode(half.data(), 500).empty()) << "under 20 ms: waits";
  enc.reset();
  EXPECT_TRUE(enc.encode(half.data(), 500).empty()) << "500 + 500 would be a packet, 500 is not";
  EXPECT_EQ(enc.encode(half.data(), 500).size(), 1u);
}

TEST(MediaOpus, AnEncoderOnlyOpensAtAnOpusRate) {
  EXPECT_FALSE(cvc::opus_encoder(44100, 1, 32000).valid()) << "Opus has no 44.1 kHz";
  EXPECT_FALSE(cvc::opus_decoder(44100, 1).valid());
}
