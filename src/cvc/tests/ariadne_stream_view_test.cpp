// stream_view's GL-free half (stream/stream_view.h): the per-widget frame puller every backend's
// stream_view uses, and the terminal widget registered on an FtxuiBackend. Frames are published by
// hand (publish_external), so every case is deterministic; no producer thread, no GL.

#include <cstdint>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/stream/frame_source.h> // fit_frame
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_view.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/image/image.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace cvc::ariadne;
namespace sx = cvc::ariadne::stream;

namespace {

// An open w x h rgba8 video stream.
std::unique_ptr<sx::stream> open_video(cvc::app &app, const std::string &id, int w = 4, int h = 4,
                                       const std::string &codec = "rgba8") {
  sx::stream_params p;
  p.id = id;
  p.format.kind = sx::frame_kind::video_raw;
  p.format.codec = codec;
  p.format.w = w;
  p.format.h = h;
  p.format.stride = w * 4;
  return sx::stream::open(app, p);
}

// Publish one solid-colour rgba8 frame (or `n` arbitrary bytes for a compressed codec).
std::int64_t publish(sx::stream &s, std::uint8_t r, std::uint8_t g, std::uint8_t b,
                     std::size_t n = 0) {
  const sx::format_desc &f = s.channel().format();
  const std::size_t size = n ? n : static_cast<std::size_t>(f.w) * f.h * 4;
  auto bytes = std::make_shared<std::vector<std::uint8_t>>(size);
  for (std::size_t i = 0; i + 3 < size; i += 4) {
    (*bytes)[i] = r;
    (*bytes)[i + 1] = g;
    (*bytes)[i + 2] = b;
    (*bytes)[i + 3] = 255;
  }
  return s.channel().publish_external(bytes->data(), size, 0.0, bytes);
}

std::uint8_t red_at_origin(const cvc::image &img) {
  return static_cast<const std::uint8_t *>(img.data())[0];
}

} // namespace

TEST(AriadneStreamView, PullsOnlyFramesNewerThanTheLastDrawn) {
  cvc::app app;
  auto s = open_video(app, "cam");
  ASSERT_TRUE(s);
  sx::stream_view_puller puller(app);
  cvc::image img;
  EXPECT_FALSE(puller.pull("view", s->token(), img)); // subscribed, but nothing published yet
  publish(*s, 200, 0, 0);
  ASSERT_TRUE(puller.pull("view", s->token(), img));
  EXPECT_EQ(img.width(), 4);
  EXPECT_EQ(red_at_origin(img), 200);
  EXPECT_FALSE(puller.pull("view", s->token(), img)); // the same frame is not drawn twice
  publish(*s, 50, 0, 0);
  ASSERT_TRUE(puller.pull("view", s->token(), img));
  EXPECT_EQ(red_at_origin(img), 50);
  s->close();
}

TEST(AriadneStreamView, FollowsATokenSwitchAndAReopenUnderTheSameToken) {
  cvc::app app;
  auto a = open_video(app, "a"), b = open_video(app, "b");
  sx::stream_view_puller puller(app);
  cvc::image img;
  // A view sees frames published after it subscribes, so each pull here subscribes first.
  EXPECT_FALSE(puller.pull("view", a->token(), img));
  publish(*a, 10, 0, 0);
  ASSERT_TRUE(puller.pull("view", a->token(), img));
  EXPECT_EQ(red_at_origin(img), 10);
  EXPECT_FALSE(puller.pull("view", b->token(), img)); // the bound token switched: re-subscribe
  EXPECT_EQ(a->channel().subscriber_count(), 0u);     // and the old subscription was released
  publish(*b, 20, 0, 0);
  ASSERT_TRUE(puller.pull("view", b->token(), img));
  EXPECT_EQ(red_at_origin(img), 20);

  const std::string token = b->token();
  b->close();
  b.reset();
  EXPECT_FALSE(puller.pull("view", token, img)); // closed: nothing to draw, no crash
  auto again = open_video(app, "b");             // reopened under the same token: a new channel
  ASSERT_EQ(again->token(), token);
  EXPECT_FALSE(puller.pull("view", token, img)); // re-subscribes to the new channel
  publish(*again, 30, 0, 0);
  ASSERT_TRUE(puller.pull("view", token, img));
  EXPECT_EQ(red_at_origin(img), 30);
  a->close();
  again->close();
}

TEST(AriadneStreamView, DecodesACompressedCodecThroughTheHostsDecoder) {
  cvc::app app;
  auto s = open_video(app, "jpg", 4, 4, "mjpeg");
  sx::stream_view_puller puller(app);
  cvc::image img;
  EXPECT_FALSE(puller.pull("view", s->token(), img)); // subscribes
  publish(*s, 0, 0, 0, 16);
  EXPECT_FALSE(puller.pull("view", s->token(), img)); // no decoder: an mjpeg frame is not drawn
  int decoded = 0;
  puller.set_decoder("mjpeg", [&](const void *, std::size_t size, cvc::image &out) {
    ++decoded;
    out = cvc::image(2, 1, cvc::image::pixel_format::RGB, cvc::image::data_type::u8);
    return size == 16;
  });
  publish(*s, 0, 0, 0, 16);
  ASSERT_TRUE(puller.pull("view", s->token(), img));
  EXPECT_EQ(decoded, 1);
  EXPECT_EQ(img.width(), 2);
  EXPECT_FALSE(puller.pull("view", s->token(), img)); // decoded once per frame
  EXPECT_EQ(decoded, 1);
  s->close();
}

TEST(AriadneStreamView, TheTerminalWidgetDrawsLiveFramesAndAPlaceholder) {
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "libcvc built without FTXUI";
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  auto s = open_video(app, "cam", 8, 8);
  FtxuiBackend::Options opts;
  opts.image_color = FtxuiBackend::Options::ImageColor::Color;
  FtxuiBackend be(opts);
  register_stream_view_widget(be, app);
  Runtime rt(app, "ui.docs.sv");
  rt.set_backend(&be);
  LoadResult lr = load_string("meta: {min_libcvc: 3.4.0}\n"
                              "root:\n"
                              "  - type: stream_view\n"
                              "    id: live\n"
                              "    stream: " +
                              s->token() +
                              "\n"
                              "    size: 64\n"
                              "  - type: stream_view\n"
                              "    id: idle\n"
                              "    stream: nobody\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  rt.set_root(std::move(lr.root));
  std::string screen = be.render_to_string(rt, 40, 12); // the first frame subscribes the view
  EXPECT_NE(screen.find("no frame]"), std::string::npos) << screen;
  publish(*s, 255, 0, 0);
  screen = be.render_to_string(rt, 40, 12);
  EXPECT_NE(screen.find("\xe2\x96\x80"), std::string::npos) << screen; // the frame, as half-blocks
  EXPECT_NE(screen.find("[stream_view: nobody - no frame]"), std::string::npos) << screen;
  EXPECT_EQ(screen.find("[stream_view?]"), std::string::npos) << screen; // the backend handled it
  // A new frame replaces the image the widget draws.
  const std::string first = be.render_to_string(rt, 40, 12, /*ansi=*/true);
  publish(*s, 0, 0, 255);
  EXPECT_NE(be.render_to_string(rt, 40, 12, /*ansi=*/true), first);
  s->close();
}

// A camera's frame keeps its shape within the capture budget, the budget turned to the frame: a
// phone held upright in the conference's 320x240 sends 240x320 (180x320 at 9:16), not a stretch.
TEST(AriadneStreamView, FitFrameKeepsTheShapeAndTurnsTheBudget) {
  const auto fit = [](int sw, int sh) {
    int w = 0, h = 0;
    sx::fit_frame(sw, sh, 320, 240, w, h);
    return std::make_pair(w, h);
  };
  EXPECT_EQ(fit(640, 480), std::make_pair(320, 240));
  EXPECT_EQ(fit(1280, 720), std::make_pair(320, 180));
  EXPECT_EQ(fit(480, 640), std::make_pair(240, 320)) << "portrait 3:4";
  EXPECT_EQ(fit(720, 1280), std::make_pair(180, 320)) << "portrait 9:16";
  EXPECT_EQ(fit(100, 100), std::make_pair(100, 100)) << "never upscaled";
  EXPECT_EQ(fit(160, 120), std::make_pair(160, 120));
  EXPECT_EQ(fit(4000, 10), std::make_pair(320, 1)) << "at least a pixel";
  EXPECT_EQ(fit(0, 480), std::make_pair(320, 240)) << "no size yet: the budget";
  int w = 0, h = 0;
  sx::fit_frame(480, 640, 240, 320, w, h); // a portrait budget is the same budget
  EXPECT_EQ(std::make_pair(w, h), std::make_pair(240, 320));
  sx::fit_frame(640, 480, 240, 320, w, h);
  EXPECT_EQ(std::make_pair(w, h), std::make_pair(320, 240));
}

TEST(AriadneStreamView, SizeIsAWidthOrABox) {
  const auto scalar = [](const std::string &s) {
    Value v;
    v.scalar = s;
    return v;
  };
  const auto props = [](Value size) {
    Value p;
    p.kind = Value::Kind::Map;
    p.entries.emplace_back("size", std::move(size));
    return p;
  };
  Value box;
  box.kind = Value::Kind::Sequence;
  box.items = {scalar("300"), scalar("225")};
  float w = 0.0f, h = 0.0f;
  EXPECT_TRUE(sx::stream_view_size(props(box), w, h));
  EXPECT_EQ(w, 300.0f);
  EXPECT_EQ(h, 225.0f);
  EXPECT_FALSE(sx::stream_view_size(props(scalar("320")), w, h));
  EXPECT_EQ(w, 320.0f);
  EXPECT_EQ(h, 0.0f);
  EXPECT_FALSE(sx::stream_view_size(Value(), w, h)) << "no size";
  EXPECT_EQ(w, 256.0f);
  EXPECT_FALSE(sx::stream_view_size(props(scalar("-4")), w, h));
  EXPECT_EQ(w, 256.0f);
  box.items[1] = scalar("0");
  EXPECT_FALSE(sx::stream_view_size(props(box), w, h)) << "a degenerate box";
  EXPECT_EQ(w, 256.0f);
}

// In a box, a portrait frame is drawn no wider than the box's height allows, where a width alone
// lets it stand as tall as its aspect makes it.
TEST(AriadneStreamView, TheTerminalWidgetFitsAFrameInABox) {
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "libcvc built without FTXUI";
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  const auto half_blocks = [](const std::string &size) {
    cvc::app app;
    auto s = open_video(app, "tall", 8, 16);
    FtxuiBackend::Options opts;
    opts.image_color = FtxuiBackend::Options::ImageColor::Color;
    FtxuiBackend be(opts);
    register_stream_view_widget(be, app);
    Runtime rt(app, "ui.docs.sv");
    rt.set_backend(&be);
    LoadResult lr = load_string("meta: {min_libcvc: 3.4.0}\nroot:\n  - type: stream_view\n"
                                "    stream: " +
                                s->token() + "\n    size: " + size + "\n");
    EXPECT_TRUE(lr.ok) << lr.error;
    rt.set_root(std::move(lr.root));
    be.render_to_string(rt, 60, 40); // the first frame subscribes the view
    publish(*s, 255, 0, 0);
    const std::string screen = be.render_to_string(rt, 60, 40);
    std::size_t n = 0;
    for (std::size_t at = screen.find("\xe2\x96\x80"); at != std::string::npos;
         at = screen.find("\xe2\x96\x80", at + 1))
      ++n;
    s->close();
    return n;
  };
  const std::size_t by_width = half_blocks("96"), in_box = half_blocks("[96, 96]");
  EXPECT_GT(in_box, 0u);
  EXPECT_LT(in_box, by_width) << "the 1:2 frame is drawn at half the width in a square box";
}
