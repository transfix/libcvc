/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase-2 PR1: cvc::image zero-copy adopt constructor + the cvc::ariadne::stream::to_image
// bridge (roadmap STATE_BINARY_STREAMING.md §7 H2). Verifies the alias is truly
// copy-free, that a mutation forks a private copy (adopted buffers are logically
// read-only), that the keepalive outlives the frame and is released exactly when
// the last aliasing image drops, and that to_image rejects everything it cannot
// alias losslessly (bad codec, padded stride, bottom-left origin, short buffer,
// non-video kind).

#include <cstdint>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/to_image.h>
#include <cvc/image/image.h>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace cvc::ariadne::stream;

namespace {

// Build a synthetic video frame aliasing `buf` (keepalive owns buf).
frame_ptr make_video_frame(int w, int h, const std::string &codec,
                           const std::shared_ptr<std::vector<std::uint8_t>> &buf, int stride = 0,
                           frame_kind kind = frame_kind::video_raw,
                           frame_origin origin = frame_origin::top_left) {
  auto f = std::make_shared<frame>();
  f->data = buf->data();
  f->size = buf->size();
  f->seq = 0;
  f->format.kind = kind;
  f->format.codec = codec;
  f->format.w = w;
  f->format.h = h;
  f->format.stride = stride;
  f->format.origin = origin;
  f->keepalive = buf;
  return frame_ptr(std::move(f));
}

std::shared_ptr<std::vector<std::uint8_t>> rgba_buf(int w, int h, std::uint8_t fill) {
  return std::make_shared<std::vector<std::uint8_t>>(static_cast<std::size_t>(w) * h * 4, fill);
}

std::shared_ptr<std::vector<std::uint8_t>> packed_buf(int w, int h, int bpp, std::uint8_t fill) {
  return std::make_shared<std::vector<std::uint8_t>>(static_cast<std::size_t>(w) * h * bpp, fill);
}

} // namespace

// --------------------------------------------------------------------------
// cvc::image adopt constructor
// --------------------------------------------------------------------------

TEST(ImageAdoptCtor, AliasesWithoutCopy) {
  auto buf = rgba_buf(4, 4, 0x11);
  cvc::image img(4, 4, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8, buf->data(), buf);
  EXPECT_EQ(img.width(), 4);
  EXPECT_EQ(img.height(), 4);
  EXPECT_EQ(img.size_bytes(), 64u);
  // const data() does NOT detach: it must point at the exact foreign buffer.
  const cvc::image &cimg = img;
  EXPECT_EQ(cimg.data(), buf->data());
  EXPECT_EQ(img.storage().get(), buf->data());
}

TEST(ImageAdoptCtor, MutationForksPrivateCopyLeavingSourceUntouched) {
  auto buf = rgba_buf(2, 2, 0x22);
  cvc::image img(2, 2, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8, buf->data(), buf);
  // Non-const data() must detach (adopted buffers are logically read-only).
  unsigned char *p = img.data();
  EXPECT_NE(p, buf->data());  // forked to a private copy
  p[0] = 0xAB;                // mutate the private copy
  EXPECT_EQ((*buf)[0], 0x22); // source bytes untouched
  // A second data() does not re-fork (now owns a private buffer).
  EXPECT_EQ(img.data(), p);
}

TEST(ImageAdoptCtor, CopyDivergesOnMutationLeavingSibling) {
  auto buf = rgba_buf(2, 2, 0x22);
  cvc::image a(2, 2, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8, buf->data(), buf);
  cvc::image b = a; // COW copy shares the alias

  unsigned char *pa = a.data(); // a forks (adopted)
  EXPECT_NE(pa, buf->data());
  pa[0] = 0xEE;
  // b still aliases the original, unmutated source bytes.
  EXPECT_EQ(static_cast<const cvc::image &>(b).data(), buf->data());
  EXPECT_EQ((*buf)[0], 0x22);
  // b now forks on its own mutation.
  unsigned char *pb = b.data();
  EXPECT_NE(pb, buf->data());
  EXPECT_NE(pb, pa);
}

TEST(ImageAdoptCtor, KeepaliveReleasedOnlyWhenLastAliasDrops) {
  bool released = false;
  const int w = 2, h = 2;
  const std::size_t n = static_cast<std::size_t>(w) * h * 4;
  std::shared_ptr<std::vector<std::uint8_t>> storage(new std::vector<std::uint8_t>(n, 0x7f),
                                                     [&released](std::vector<std::uint8_t> *v) {
                                                       released = true;
                                                       delete v;
                                                     });

  auto f = std::make_shared<frame>();
  f->data = storage->data();
  f->size = n;
  f->format.kind = frame_kind::video_raw;
  f->format.codec = "rgba8";
  f->format.w = w;
  f->format.h = h;
  f->keepalive = storage;
  frame_ptr fp(std::move(f));
  storage.reset(); // only fp holds the storage now

  EXPECT_FALSE(released);
  cvc::image img = to_image(fp); // img co-owns the storage via keepalive
  fp.reset();                    // drop the frame; img still pins storage
  EXPECT_FALSE(released);
  {
    cvc::image copy = img; // COW copy shares the alias
    EXPECT_FALSE(released);
  }
  EXPECT_FALSE(released); // img still holds it
  img = cvc::image();     // drop the last alias
  EXPECT_TRUE(released);  // storage freed exactly now
}

// --------------------------------------------------------------------------
// cvc::ariadne::stream::to_image
// --------------------------------------------------------------------------

TEST(StreamToImage, AliasesDenseTopLeftRgba8) {
  auto buf = rgba_buf(8, 4, 0x33);
  // stride == 0 means "dense".
  frame_ptr fp = make_video_frame(8, 4, "rgba8", buf, /*stride*/ 0);
  cvc::image img = to_image(fp);
  const cvc::image &cimg = img;
  EXPECT_EQ(cimg.data(), buf->data()); // zero-copy alias
  EXPECT_EQ(img.format(), cvc::image::pixel_format::RGBA);
  EXPECT_EQ(img.size_bytes(), 8u * 4u * 4u);
}

TEST(StreamToImage, AcceptsExplicitDenseStride) {
  auto buf = rgba_buf(8, 4, 0x44);
  frame_ptr fp = make_video_frame(8, 4, "rgba8", buf, /*stride*/ 8 * 4);
  cvc::image img = to_image(fp);
  EXPECT_EQ(static_cast<const cvc::image &>(img).data(), buf->data());
}

TEST(StreamToImage, RejectsPaddedStride) {
  auto buf =
      std::make_shared<std::vector<std::uint8_t>>(static_cast<std::size_t>((8 * 4 + 16)) * 4, 0x55);
  frame_ptr fp = make_video_frame(8, 4, "rgba8", buf, /*stride*/ 8 * 4 + 16);
  EXPECT_THROW(to_image(fp), std::runtime_error);
}

TEST(StreamToImage, RejectsSwappedOrUnknownCodec) {
  auto buf = rgba_buf(4, 4, 0x66);
  EXPECT_THROW(to_image(make_video_frame(4, 4, "bgra8", buf)), std::runtime_error);
  EXPECT_THROW(to_image(make_video_frame(4, 4, "yuv420p", buf)), std::runtime_error);
}

TEST(StreamToImage, RejectsBottomLeftOrigin) {
  auto buf = rgba_buf(4, 4, 0x77);
  frame_ptr fp =
      make_video_frame(4, 4, "rgba8", buf, 0, frame_kind::video_raw, frame_origin::bottom_left);
  EXPECT_THROW(to_image(fp), std::runtime_error);
}

TEST(StreamToImage, RejectsNonVideoKind) {
  auto buf = rgba_buf(4, 4, 0x88);
  frame_ptr fp = make_video_frame(4, 4, "rgba8", buf, 0, frame_kind::audio_pcm);
  EXPECT_THROW(to_image(fp), std::runtime_error);
}

TEST(StreamToImage, RejectsShortBuffer) {
  // Buffer holds only half the pixels the format claims.
  auto buf =
      std::make_shared<std::vector<std::uint8_t>>(static_cast<std::size_t>(8) * 4 * 4 / 2, 0x99);
  frame_ptr fp = make_video_frame(8, 4, "rgba8", buf);
  EXPECT_THROW(to_image(fp), std::runtime_error);
}

TEST(StreamToImage, RejectsNullFrame) {
  frame_ptr fp;
  EXPECT_THROW(to_image(fp), std::runtime_error);
}

TEST(StreamToImage, RejectsNullData) {
  // A malformed frame: null data but a claimed size big enough to pass the
  // size check. Must throw, not silently yield an empty image.
  auto f = std::make_shared<frame>();
  f->data = nullptr;
  f->size = static_cast<std::size_t>(4) * 4 * 4;
  f->format.kind = frame_kind::video_raw;
  f->format.codec = "rgba8";
  f->format.w = 4;
  f->format.h = 4;
  frame_ptr fp(std::move(f));
  EXPECT_THROW(to_image(fp), std::runtime_error);
}

TEST(StreamToImage, AliasesNonRgbaMatchingCodecs) {
  struct Case {
    const char *codec;
    int bpp;
    cvc::image::pixel_format pf;
  };
  const Case cases[] = {
      {"rgb8", 3, cvc::image::pixel_format::RGB},
      {"gray8", 1, cvc::image::pixel_format::GRAY},
      {"graya8", 2, cvc::image::pixel_format::GRAY_ALPHA},
  };
  for (const auto &c : cases) {
    auto buf = packed_buf(8, 4, c.bpp, 0x5a);
    frame_ptr fp = make_video_frame(8, 4, c.codec, buf);
    cvc::image img = to_image(fp);
    EXPECT_EQ(img.format(), c.pf) << c.codec;
    EXPECT_EQ(static_cast<const cvc::image &>(img).data(), buf->data()) << c.codec;
    EXPECT_EQ(img.size_bytes(), static_cast<std::size_t>(8) * 4 * c.bpp) << c.codec;
  }
}

TEST(StreamToImage, RoundTripsPixelsByReference) {
  auto buf = rgba_buf(3, 2, 0x00);
  for (std::size_t i = 0; i < buf->size(); ++i)
    (*buf)[i] = static_cast<std::uint8_t>(i);
  frame_ptr fp = make_video_frame(3, 2, "rgba8", buf);
  cvc::image img = to_image(fp);
  const unsigned char *pix = static_cast<const cvc::image &>(img).data();
  ASSERT_EQ(pix, buf->data());
  for (std::size_t i = 0; i < buf->size(); ++i)
    EXPECT_EQ(pix[i], static_cast<unsigned char>(i));
}
