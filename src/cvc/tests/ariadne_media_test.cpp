// Ariadne media (docs/ARIADNE.md §5.11): decoding pictures from memory, fitting them to a box, the
// image cache, and an image: widget loading by URL through the host's network and credentials.

#include "local_http_server.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/media.h>
#include <cvc/ariadne/net_host.h>
#include <cvc/ariadne/net_intrinsics.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/image/image.h>
#include <cvc/image/image_decode.h>
#include <cvc/net/credentials.h>
#include <cvc/net/net_policy.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::ariadne;
namespace media = cvc::ariadne::media;
using cvc_test::LocalHttpServer;

namespace {

// An image of `w` x `h` RGBA pixels, each `fn(x, y)`.
template <class Fn> cvc::image make_image(int w, int h, Fn fn) {
  cvc::image img(w, h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  unsigned char *p = img.data();
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const std::array<unsigned char, 4> c = fn(x, y);
      for (int k = 0; k < 4; ++k)
        p[(y * w + x) * 4 + k] = c[k];
    }
  return img;
}

const unsigned char *px(const cvc::image &img, int x, int y) {
  return img.data() + (static_cast<std::size_t>(y) * img.width() + x) * 4;
}

// `img` encoded as `ext` (png, jpg, bmp), as the bytes a server would send.
std::string encode(const cvc::image &img, const std::string &ext) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      ("ariadne_media_test_" +
       std::to_string(std::hash<std::thread::id>()(std::this_thread::get_id())) + "." + ext);
  cvc::write_image(img, path.string(), 95);
  std::ifstream in(path, std::ios::binary);
  std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  std::filesystem::remove(path);
  return bytes;
}

cvc::image checker(int w, int h) {
  return make_image(w, h, [](int x, int y) -> std::array<unsigned char, 4> {
    return ((x / 4 + y / 4) % 2) ? std::array<unsigned char, 4>{255, 0, 0, 255}
                                 : std::array<unsigned char, 4>{0, 0, 255, 255};
  });
}

// A 1x1 GIF and a 1x1 lossless WebP, as small as each format gets.
const std::string kGif1x1("GIF89a\x01\x00\x01\x00\x80\x00\x00\xff\x00\x00\x00\x00\x00"
                          ",\x00\x00\x00\x00\x01\x00\x01\x00\x00\x02\x02\x44\x01\x00;",
                          35);
const std::string kWebp1x1("RIFF\x1a\x00\x00\x00WEBPVP8L\x0d\x00\x00\x00\x2f\x00\x00\x00\x10"
                           "\x07\x10\x11\x11\x88\x88\xfe\x07\x00",
                           34);

} // namespace

// ---- decode_image
// ---------------------------------------------------------------------------------

TEST(MediaDecode, PngJpegBmpGifDecodeFromMemory) {
  const cvc::image src = checker(16, 8);
  for (const char *ext : {"png", "bmp"}) {
    const std::string bytes = encode(src, ext);
    std::string why;
    const cvc::image img = cvc::decode_image(bytes.data(), bytes.size(), &why);
    ASSERT_FALSE(img.empty()) << ext << ": " << why;
    EXPECT_EQ(img.width(), 16);
    EXPECT_EQ(img.height(), 8);
    EXPECT_EQ(px(img, 0, 0)[2], 255) << ext; // blue square at the corner
    EXPECT_EQ(px(img, 4, 0)[0], 255) << ext; // red next to it
  }
  const std::string jpg = encode(src, "jpg");
  EXPECT_EQ(cvc::sniff_image_format(jpg.data(), jpg.size()), "jpeg");
  const cvc::image j = cvc::decode_image(jpg.data(), jpg.size());
  ASSERT_FALSE(j.empty());
  EXPECT_EQ(j.width(), 16);
  EXPECT_EQ(cvc::sniff_image_format(kGif1x1.data(), kGif1x1.size()), "gif");
  const cvc::image g = cvc::decode_image(kGif1x1.data(), kGif1x1.size());
  ASSERT_FALSE(g.empty());
  EXPECT_EQ(g.width(), 1);
  EXPECT_EQ(g.height(), 1);
}

TEST(MediaDecode, WebpDecodesWhereTheBuildHasLibwebp) {
  EXPECT_EQ(cvc::sniff_image_format(kWebp1x1.data(), kWebp1x1.size()), "webp");
  std::string why;
  const cvc::image img = cvc::decode_image(kWebp1x1.data(), kWebp1x1.size(), &why);
  if (!cvc::have_webp_decoder() && img.empty())
    GTEST_SKIP() << "no WebP decoder in this build: " << why;
  ASSERT_FALSE(img.empty()) << why;
  EXPECT_EQ(img.width(), 1);
  EXPECT_EQ(img.height(), 1);
}

TEST(MediaDecode, BadInputIsAnErrorNotACrash) {
  std::string why;
  EXPECT_TRUE(cvc::decode_image("hello, world", 12, &why).empty());
  EXPECT_NE(why.find("not an image"), std::string::npos) << why;
  const std::string png = encode(checker(32, 32), "png");
  why.clear();
  EXPECT_TRUE(cvc::decode_image(png.data(), png.size() / 2, &why).empty()); // truncated
  EXPECT_FALSE(why.empty());
  EXPECT_TRUE(cvc::decode_image(nullptr, 0, &why).empty());
  const std::string webp_cut = kWebp1x1.substr(0, 20);
  EXPECT_TRUE(cvc::decode_image(webp_cut.data(), webp_cut.size(), &why).empty());
}

TEST(MediaDecode, LimitsRefuseBeforeDecoding) {
  // A PNG whose header claims 20000 pixels a side: refused from the header alone.
  std::string png = encode(checker(8, 8), "png");
  ASSERT_GT(png.size(), 24u);
  png[16] = 0, png[17] = 0, png[18] = 0x4e, png[19] = 0x20; // IHDR width = 20000
  std::string why;
  EXPECT_TRUE(cvc::decode_image(png.data(), png.size(), &why).empty());
  EXPECT_NE(why.find("too large"), std::string::npos) << why;
  // And an input over the byte cap.
  cvc::image_decode_limits lim;
  lim.max_bytes = 10;
  const std::string ok = encode(checker(8, 8), "png");
  EXPECT_TRUE(cvc::decode_image(ok.data(), ok.size(), lim, &why).empty());
  EXPECT_NE(why.find("too large"), std::string::npos) << why;
}

// ---- fit_image
// -----------------------------------------------------------------------------------

namespace {
const cvc::image kRgb3x1 = make_image(3, 1, [](int x, int) -> std::array<unsigned char, 4> {
  return x == 0   ? std::array<unsigned char, 4>{255, 0, 0, 255}
         : x == 1 ? std::array<unsigned char, 4>{0, 255, 0, 255}
                  : std::array<unsigned char, 4>{0, 0, 255, 255};
});
} // namespace

TEST(MediaFit, CoverCropsTheCentre) {
  const cvc::image out = media::fit_image(kRgb3x1, 1, 1, media::Fit::Cover, media::Shape::Rect, 1);
  ASSERT_EQ(out.width(), 1);
  ASSERT_EQ(out.height(), 1);
  EXPECT_EQ(px(out, 0, 0)[1], 255); // green, the middle column
  EXPECT_EQ(px(out, 0, 0)[0], 0);
}

TEST(MediaFit, ContainPadsWithTransparency) {
  const cvc::image out =
      media::fit_image(kRgb3x1, 3, 3, media::Fit::Contain, media::Shape::Rect, 1);
  ASSERT_EQ(out.width(), 3);
  ASSERT_EQ(out.height(), 3);
  for (int x = 0; x < 3; ++x) {
    EXPECT_EQ(px(out, x, 0)[3], 0) << x;   // padding above
    EXPECT_EQ(px(out, x, 1)[3], 255) << x; // the image in the middle row
    EXPECT_EQ(px(out, x, 2)[3], 0) << x;   // padding below
  }
  EXPECT_EQ(px(out, 0, 1)[0], 255);
  EXPECT_EQ(px(out, 2, 1)[2], 255);
}

TEST(MediaFit, FillStretches) {
  const cvc::image out = media::fit_image(kRgb3x1, 3, 3, media::Fit::Fill, media::Shape::Rect, 1);
  ASSERT_EQ(out.height(), 3);
  for (int y = 0; y < 3; ++y) {
    EXPECT_EQ(px(out, 0, y)[0], 255);
    EXPECT_EQ(px(out, 2, y)[2], 255);
    EXPECT_EQ(px(out, 0, y)[3], 255);
  }
}

TEST(MediaFit, CircleMasksTheCorners) {
  const cvc::image white = make_image(
      16, 16, [](int, int) -> std::array<unsigned char, 4> { return {255, 255, 255, 255}; });
  const cvc::image out =
      media::fit_image(white, 16, 16, media::Fit::Cover, media::Shape::Circle, 1);
  ASSERT_EQ(out.width(), 16);
  EXPECT_EQ(px(out, 0, 0)[3], 0);
  EXPECT_EQ(px(out, 15, 15)[3], 0);
  EXPECT_EQ(px(out, 8, 8)[3], 255);
  EXPECT_GT(px(out, 8, 0)[3], 200); // the top edge's middle is inside (a soft one-pixel edge)
}

TEST(MediaFit, ScaleIsCappedBySourceAndWidthOnlyKeepsAspect) {
  const cvc::image big = checker(400, 200);
  const cvc::image a = media::fit_image(big, 100, 0, media::Fit::Contain, media::Shape::Rect, 2);
  EXPECT_EQ(a.width(), 200); // twice the box: sharp on HiDPI
  EXPECT_EQ(a.height(), 100);
  const cvc::image small = checker(10, 10);
  const cvc::image b = media::fit_image(small, 100, 100, media::Fit::Cover, media::Shape::Rect, 2);
  EXPECT_EQ(b.width(), 10); // never upscaled past the source; the backend scales it up
  EXPECT_EQ(b.height(), 10);
  EXPECT_TRUE(
      media::fit_image(cvc::image(), 10, 10, media::Fit::Cover, media::Shape::Rect).empty());
}

TEST(MediaFit, PlaceholderHasTheBoxAspect) {
  const cvc::image p = media::placeholder_image(160, 90, media::Shape::Rect, false);
  EXPECT_NEAR(double(p.width()) / p.height(), 160.0 / 90.0, 0.1);
  const cvc::image c = media::placeholder_image(48, 0, media::Shape::Circle, true);
  EXPECT_EQ(c.width(), c.height());
  EXPECT_EQ(px(c, 0, 0)[3], 0);
  EXPECT_EQ(media::placeholder_height(160, 0, media::Shape::Rect), 90);
}

TEST(MediaParse, FitShapeAndUri) {
  media::Fit f = media::Fit::Contain;
  EXPECT_TRUE(media::parse_fit("COVER", f));
  EXPECT_EQ(f, media::Fit::Cover);
  EXPECT_FALSE(media::parse_fit("stretch", f));
  media::Shape sh = media::Shape::Rect;
  EXPECT_TRUE(media::parse_shape("circle", sh));
  EXPECT_EQ(sh, media::Shape::Circle);
  EXPECT_TRUE(media::is_uri("https://example.org/a.png"));
  EXPECT_TRUE(media::is_uri("file:///tmp/a.png"));
  EXPECT_TRUE(media::is_uri("cvc://components/x.png"));
  EXPECT_FALSE(media::is_uri("occupancy"));     // a host-published name
  EXPECT_FALSE(media::is_uri("ftp://x/a.png")); // no handler of ours
  EXPECT_FALSE(media::is_uri("nav:grid"));
}

// ---- ImageCache
// ----------------------------------------------------------------------------------

namespace {

// Polls `pred` while calling `tick` until it holds or 5 s pass.
template <class Tick, class Pred> bool settle(Tick tick, Pred pred) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < end) {
    tick();
    if (pred())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

} // namespace

TEST(MediaCache, LoadsOnceAndSharesTheFetchBetweenBoxes) {
  cvc::app app;
  media::ImageCache cache(app);
  const std::string png = encode(checker(64, 32), "png");
  std::atomic<int> fetches{0};
  cache.set_fetcher([&](const std::string &, const std::string &auth, media::FetchDone done) {
    EXPECT_EQ(auth, "");
    ++fetches;
    std::thread([png, done] { done({true, png, ""}); }).detach(); // answers from another thread
  });
  media::ImageRequest a{"https://m.test/a.png", "", 32, 16, media::Fit::Cover, media::Shape::Rect};
  media::ImageRequest b = a;
  b.w = 16;
  b.h = 16;
  b.shape = media::Shape::Circle;
  media::ImageView va, vb;
  ASSERT_TRUE(settle(
      [&] {
        va = cache.image(a);
        vb = cache.image(b);
      },
      [&] { return va.status == media::Status::Ready && vb.status == media::Status::Ready; }));
  EXPECT_EQ(fetches.load(), 1); // one fetch for both boxes
  EXPECT_EQ(va.image.width(), 64);
  EXPECT_EQ(va.image.height(), 32);
  EXPECT_EQ(vb.image.width(), 32); // 16 px box at 2x, cropped square
  EXPECT_NE(va.name, vb.name);
  EXPECT_EQ(va.name.rfind("media:", 0), 0u);
  EXPECT_GT(va.gen, 0);
  EXPECT_EQ(cache.entries(), 2u);
}

TEST(MediaCache, AFailureIsReportedAndRetried) {
  cvc::app app;
  media::ImageCache cache(app);
  media::ImageCache::Limits lim;
  lim.retry_seconds = 0.05;
  cache.set_limits(lim);
  std::atomic<int> fetches{0};
  cache.set_fetcher([&](const std::string &, const std::string &, media::FetchDone done) {
    ++fetches;
    done({false, "", "http 404"});
  });
  const media::ImageRequest r{"https://m.test/gone.png", "", 32, 32, media::Fit::Contain,
                              media::Shape::Rect};
  media::ImageView v;
  ASSERT_TRUE(
      settle([&] { v = cache.image(r); }, [&] { return v.status == media::Status::Failed; }));
  EXPECT_EQ(v.error, "http 404");
  EXPECT_EQ(fetches.load(), 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  ASSERT_TRUE(settle([&] { v = cache.image(r); }, [&] { return fetches.load() == 2; }));
}

TEST(MediaCache, BytesThatAreNoImageFail) {
  cvc::app app;
  media::ImageCache cache(app);
  cache.set_fetcher([&](const std::string &, const std::string &, media::FetchDone done) {
    done({true, "<html>not found</html>", ""});
  });
  const media::ImageRequest r{"https://m.test/x", "", 8, 8, media::Fit::Contain,
                              media::Shape::Rect};
  media::ImageView v;
  ASSERT_TRUE(
      settle([&] { v = cache.image(r); }, [&] { return v.status == media::Status::Failed; }));
  EXPECT_NE(v.error.find("not an image"), std::string::npos) << v.error;
}

TEST(MediaCache, IdlePicturesAreDroppedAndReported) {
  cvc::app app;
  media::ImageCache cache(app);
  media::ImageCache::Limits lim;
  lim.idle_seconds = 0.05;
  lim.sweep_seconds = 0.0;
  cache.set_limits(lim);
  const std::string png = encode(checker(8, 8), "png");
  cache.set_fetcher([&](const std::string &, const std::string &, media::FetchDone done) {
    done({true, png, ""});
  });
  const media::ImageRequest a{"https://m.test/a.png", "", 8, 8, media::Fit::Contain,
                              media::Shape::Rect};
  media::ImageView va;
  ASSERT_TRUE(
      settle([&] { va = cache.image(a); }, [&] { return va.status == media::Status::Ready; }));
  // Asking for another picture sweeps: `a`, not asked for since, is dropped.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  media::ImageRequest b = a;
  b.uri = "https://m.test/b.png";
  cache.image(b);
  const std::vector<std::string> dropped = cache.take_dropped();
  ASSERT_EQ(dropped.size(), 1u);
  EXPECT_EQ(dropped[0], va.name);
}

TEST(MediaCache, TheNameHoldsNoCredentialOrUrl) {
  cvc::app app;
  media::ImageCache cache(app);
  cache.set_fetcher([](const std::string &, const std::string &, media::FetchDone) {});
  const media::ImageView v = cache.image(
      {"https://m.test/secret-path.png", "svc", 8, 8, media::Fit::Contain, media::Shape::Rect});
  EXPECT_EQ(v.status, media::Status::Loading);
  EXPECT_EQ(v.name.find("secret"), std::string::npos);
  EXPECT_EQ(v.name.find("svc"), std::string::npos);
  EXPECT_EQ(v.name.size(), 6u + 16u); // "media:" + 64-bit hex
}

// ---- the image: widget, end to end over the host's network
// ---------------------------------------

namespace {

const std::string kToken = "eyJhbGciOiJIUzI1NiJ9.eyJzdWIiOiJtZWRpYSJ9.bWVkaWEtdGVzdC1zaWc";

// A backend that records what it is asked to publish and draw.
struct ImageBackend : Backend {
  std::map<std::string, cvc::image> published;
  std::vector<std::string> drawn;    // names, this frame
  std::vector<std::string> tooltips; // this frame
  std::vector<std::string> texts;    // this frame
  std::vector<std::string> dropped;
  Capabilities capabilities() const override {
    Capabilities c;
    c.windows = true;
    c.bitmap = true;
    return c;
  }
  void begin_frame() override {
    drawn.clear();
    tooltips.clear();
    texts.clear();
    backdrops.clear();
  }
  bool begin_main_menu_bar() override { return false; }
  void end_main_menu_bar() override {}
  bool begin_menu(const char *) override { return false; }
  void end_menu() override {}
  bool begin_window(const char *, const char *, const Size &, float, bool *) override {
    return true;
  }
  void end_window() override {}
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  void set_tooltip(const char *t) override { tooltips.push_back(t ? t : ""); }
  void text_line(const char *t) override { texts.push_back(t); }
  void text_value(const char *l, const std::string &v) override {
    texts.push_back(std::string(l) + "=" + v);
  }
  void separator() override {}
  bool button(const char *) override { return false; }
  bool menu_item_action(const char *) override { return false; }
  BoolEdit menu_item_toggle(const char *, bool c) override { return {false, false, c}; }
  BoolEdit checkbox(const char *, bool c) override { return {false, false, c}; }
  IntEdit slider_int(const char *, const char *, int c, int, int) override {
    return {false, false, c};
  }
  DoubleEdit slider_double(const char *, const char *, double c, double, double,
                           const char *) override {
    return {false, false, c};
  }
  IndexEdit combo(const char *, int i, const std::vector<std::string> &) override {
    return {false, false, i};
  }
  StringEdit input_text(const char *, const char *, const std::string &c, bool) override {
    return {false, false, c};
  }
  void set_image(const std::string &name, const cvc::image &img) override { published[name] = img; }
  void drop_image(const std::string &name) override {
    dropped.push_back(name);
    published.erase(name);
  }
  bool draw_image(const char *name, float) override {
    drawn.push_back(name);
    return published.count(name) > 0;
  }
  std::vector<std::string> backdrops; // "begin:<image>:<window?>" / "end", in order, this frame
  void begin_backdrop(const Backdrop &bd) override {
    backdrops.push_back("begin:" + bd.image + (bd.window ? ":window" : ":group"));
  }
  void end_backdrop() override { backdrops.push_back("end"); }
};

struct NetGuard {
  ~NetGuard() {
    disable_net();
    clear_action_intrinsics();
  }
};

// Every value in the app's state tree, for a scan for the token.
std::string all_state(cvc::app &app) {
  cvc::state &root = cvc::state::instance(app);
  std::string out;
  for (const std::string &name : root.children()) {
    try {
      out += name + "=" + root(name).value() + "\n";
    } catch (...) {
    }
  }
  return out;
}

std::string read_state(cvc::app &app, const std::string &path) {
  try {
    return cvc::state::instance(app)(path).value();
  } catch (...) {
    return std::string();
  }
}

} // namespace

TEST(MediaWidget, ImagesLoadByUrlWithPolicyAndCredentials) {
  const std::string png = encode(checker(40, 20), "png");
  LocalHttpServer media_srv([&](const LocalHttpServer::Request &req) -> LocalHttpServer::Reply {
    if (req.target == "/a.png")
      return {200, {"Content-Type: image/png"}, png};
    if (req.target == "/private.png")
      return req.header("Authorization") == "Bearer " + kToken
                 ? LocalHttpServer::Reply{200, {"Content-Type: image/png"}, png}
                 : LocalHttpServer::Reply{401, {}, "{}"};
    return {404, {}, "missing"};
  });
  LocalHttpServer other_srv(
      [&](const LocalHttpServer::Request &) -> LocalHttpServer::Reply { return {200, {}, png}; });
  LocalHttpServer third_srv(
      [&](const LocalHttpServer::Request &) -> LocalHttpServer::Reply { return {200, {}, png}; });
  NetGuard guard;
  cvc::app app;
  NetHostConfig cfg;
  cfg.enabled = true;
  cfg.allow = {media_srv.origin(), third_srv.origin()}; // the credential is bound to one of them
  ASSERT_EQ(enable_net(app, cfg), "");
  const std::optional<NetAccess> access = net_host_access();
  ASSERT_TRUE(access && access->credentials);
  const auto origin = cvc::net::parse_origin(media_srv.origin());
  ASSERT_TRUE(origin);
  access->credentials->bind("media", {*origin});
  std::string why;
  ASSERT_TRUE(access->credentials->provide("media", kToken, &why)) << why;

  const std::string doc = "meta: {min_libcvc: 3.0.0}\n"
                          "windows:\n"
                          "  - window: Feed\n"
                          "    children:\n"
                          "      - image: plain\n"
                          "        src: " +
                          media_srv.url("/a.png") +
                          "\n"
                          "        size: [40, 20]\n"
                          "        fit: cover\n"
                          "        status: img.plain\n"
                          "      - image: private\n"
                          "        src: " +
                          media_srv.url("/private.png") +
                          "\n"
                          "        auth: media\n"
                          "        size: [24, 24]\n"
                          "        shape: circle\n"
                          "        status: img.priv\n"
                          "      - image: missing\n"
                          "        src: " +
                          media_srv.url("/missing.png") +
                          "\n"
                          "        status: img.miss\n"
                          "      - image: elsewhere\n"
                          "        src: " +
                          other_srv.url("/b.png") +
                          "\n"
                          "        status: img.denied\n"
                          "      - image: wrong origin\n"
                          "        src: " +
                          third_srv.url("/c.png") +
                          "\n"
                          "        auth: media\n"
                          "        status: img.unbound\n";
  LoadResult lr = load_string(doc);
  ASSERT_TRUE(lr.ok) << lr.error;
  for (const std::string &w : lr.warnings)
    ADD_FAILURE() << w;
  Runtime rt(app);
  ImageBackend b;
  rt.set_backend(&b);
  rt.set_root(std::move(lr.root));
  const auto settled = [&] {
    for (const char *k : {"img.plain", "img.priv", "img.miss", "img.denied", "img.unbound"})
      if (read_state(app, k) == "loading" || read_state(app, k).empty())
        return false;
    return true;
  };
  ASSERT_TRUE(settle([&] { rt.render(); }, settled)) << all_state(app);
  rt.render(); // one more frame: the ready rasters are published and drawn

  EXPECT_EQ(read_state(app, "img.plain"), "ready");
  EXPECT_EQ(read_state(app, "img.priv"), "ready");
  EXPECT_EQ(read_state(app, "img.miss"), "failed");
  EXPECT_EQ(read_state(app, "img.miss.error"), "http 404");
  EXPECT_EQ(read_state(app, "img.denied"), "failed");
  EXPECT_NE(read_state(app, "img.denied.error").find("not an origin this host allows"),
            std::string::npos)
      << read_state(app, "img.denied.error");
  EXPECT_EQ(read_state(app, "img.unbound"), "failed");
  EXPECT_NE(read_state(app, "img.unbound.error").find("no credential 'media'"), std::string::npos)
      << read_state(app, "img.unbound.error");

  // Drawn as fitted rasters under the cache's names, the failures as placeholders.
  int media_names = 0, placeholders = 0;
  for (const std::string &n : b.drawn) {
    media_names += n.rfind("media:placeholder:", 0) != 0 && n.rfind("media:", 0) == 0;
    placeholders += n.rfind("media:placeholder:failed:", 0) == 0;
  }
  EXPECT_EQ(media_names, 2);
  EXPECT_EQ(placeholders, 3);
  for (const auto &[name, img] : b.published)
    if (name.rfind("media:placeholder:", 0) != 0)
      EXPECT_FALSE(img.empty());

  // The token went to the bound origin only, on the request that named it.
  for (const LocalHttpServer::Request &r : media_srv.requests()) {
    if (r.target == "/private.png")
      EXPECT_EQ(r.header("Authorization"), "Bearer " + kToken);
    else
      EXPECT_EQ(r.header("Authorization"), "") << r.target;
  }
  EXPECT_TRUE(other_srv.requests().empty()); // refused before any request was made
  EXPECT_TRUE(third_srv.requests().empty()); // no credential for it: not sent at all

  // And it is nowhere a document or a user can see: state, tooltips, text, published names.
  EXPECT_EQ(all_state(app).find(kToken), std::string::npos);
  for (const std::string &t : b.tooltips)
    EXPECT_EQ(t.find(kToken), std::string::npos) << t;
  for (const auto &[name, img] : b.published)
    EXPECT_EQ(name.find(kToken), std::string::npos);
  EXPECT_FALSE(b.tooltips.empty()); // a failed image says why on hover
}

TEST(MediaWidget, BindNamesAUrlPerItemAndAPlainNameStillWorks) {
  const std::string png = encode(checker(16, 16), "png");
  LocalHttpServer srv([&](const LocalHttpServer::Request &) -> LocalHttpServer::Reply {
    return {200, {"Content-Type: image/png"}, png};
  });
  NetGuard guard;
  cvc::app app;
  NetHostConfig cfg;
  cfg.enabled = true;
  cfg.allow = {srv.origin()};
  ASSERT_EQ(enable_net(app, cfg), "");
  cvc::state &root = cvc::state::instance(app);
  root("feed.items.0.pic").value(srv.url("/0.png"));
  root("feed.items.1.pic").value(srv.url("/1.png"));
  root("feed.items").value(std::string("2"));
  const std::string doc = "windows:\n"
                          "  - window: Feed\n"
                          "    children:\n"
                          "      - type: group\n"
                          "        repeat: (state-get \"feed.items\")\n"
                          "        children:\n"
                          "          - image: pic {i}\n"
                          "            bind: feed.items.{i}.pic\n"
                          "            size: [16, 16]\n"
                          "            status: feed.items.{i}.pic_status\n"
                          "      - image: host raster\n"
                          "        src: occupancy\n";
  LoadResult lr = load_string(doc);
  ASSERT_TRUE(lr.ok) << lr.error;
  Runtime rt(app);
  ImageBackend b;
  b.published["occupancy"] = checker(4, 4); // a host-published image, by name, as before
  rt.set_backend(&b);
  rt.set_root(std::move(lr.root));
  ASSERT_TRUE(settle([&] { rt.render(); },
                     [&] {
                       return read_state(app, "feed.items.0.pic_status") == "ready" &&
                              read_state(app, "feed.items.1.pic_status") == "ready";
                     }));
  rt.render();
  EXPECT_NE(std::find(b.drawn.begin(), b.drawn.end(), "occupancy"), b.drawn.end());
  std::set<std::string> targets;
  for (const auto &r : srv.requests())
    targets.insert(r.target);
  EXPECT_EQ(targets, (std::set<std::string>{"/0.png", "/1.png"}));
}

TEST(MediaWidget, WithTheNetworkOffAnHttpImageSaysSo) {
  cvc::app app;
  LoadResult lr = load_string("windows:\n"
                              "  - window: W\n"
                              "    children:\n"
                              "      - image: pic\n"
                              "        src: https://example.invalid/a.png\n"
                              "        status: s\n");
  ASSERT_TRUE(lr.ok);
  Runtime rt(app);
  ImageBackend b;
  rt.set_backend(&b);
  rt.set_root(std::move(lr.root));
  ASSERT_TRUE(settle([&] { rt.render(); }, [&] { return read_state(app, "s") == "failed"; }));
  EXPECT_NE(read_state(app, "s.error").find("network off"), std::string::npos)
      << read_state(app, "s.error");
}

TEST(MediaWidget, FileUrisResolveAgainstTheDocumentsDirectory) {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ariadne_media_test_doc";
  std::filesystem::create_directories(dir / "img");
  {
    const std::string png = encode(checker(8, 8), "png");
    std::ofstream(dir / "img" / "a.png", std::ios::binary) << png;
  }
  cvc::app app;
  LoadResult lr = load_string("windows:\n"
                              "  - window: W\n"
                              "    children:\n"
                              "      - image: pic\n"
                              "        src: file://img/a.png\n"
                              "        status: s\n");
  ASSERT_TRUE(lr.ok);
  Runtime rt(app);
  rt.set_media_base(dir.string());
  ImageBackend b;
  rt.set_backend(&b);
  rt.set_root(std::move(lr.root));
  ASSERT_TRUE(settle([&] { rt.render(); }, [&] { return read_state(app, "s") != "loading"; }));
  EXPECT_EQ(read_state(app, "s"), "ready") << read_state(app, "s.error");
  std::filesystem::remove_all(dir);
}

TEST(MediaWidget, LoaderWarnsOnBadFitAndShape) {
  LoadResult lr = load_string("windows:\n"
                              "  - window: W\n"
                              "    children:\n"
                              "      - image: pic\n"
                              "        src: https://x.test/a.png\n"
                              "        fit: stretch\n"
                              "        shape: hexagon\n");
  ASSERT_TRUE(lr.ok);
  int warned = 0;
  for (const std::string &w : lr.warnings)
    warned += w.find("fit:") != std::string::npos || w.find("shape:") != std::string::npos;
  EXPECT_GE(warned, 2);
}

// ---- background: on a window or group (§5.12)
// -----------------------------------------------------

namespace {

std::filesystem::path picture_dir() {
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "ariadne_media_test_backdrop";
  std::filesystem::create_directories(dir);
  const cvc::image img = make_image(32, 16, [](int x, int) -> std::array<unsigned char, 4> {
    return x < 16 ? std::array<unsigned char, 4>{230, 120, 40, 255}
                  : std::array<unsigned char, 4>{40, 160, 230, 255};
  });
  const std::string png = encode(img, "png");
  std::ofstream(dir / "bg.png", std::ios::binary) << png;
  return dir;
}

const char *kBackdropDoc = "meta: {min_libcvc: 3.0.0}\n"
                           "windows:\n"
                           "  - window: Profile\n"
                           "    size: [320, auto]\n"
                           "    background: {src: file://bg.png, opacity: 0.5, status: s.win}\n"
                           "    children:\n"
                           "      - type: group\n"
                           "        id: banner\n"
                           "        background:\n"
                           "          src: file://bg.png\n"
                           "          terminal: banner\n"
                           "          banner_rows: 2\n"
                           "          status: s.banner\n"
                           "        children:\n"
                           "          - text: Grace Hopper\n"
                           "      - text: followers 1024\n";

} // namespace

TEST(MediaBackground, TheLoaderReadsAMapOrAUrl) {
  LoadResult lr = load_string("windows:\n"
                              "  - window: W\n"
                              "    background: https://x.test/bg.jpg\n"
                              "    children:\n"
                              "      - type: group\n"
                              "        background: {bind: p.banner, fit: contain, opacity: 2,\n"
                              "                     terminal: none, auth: svc}\n"
                              "        children: [x]\n"
                              "      - text: hi\n"
                              "        background: https://x.test/no.jpg\n");
  ASSERT_TRUE(lr.ok) << lr.error;
  const Widget &win = lr.root.children.at(0);
  EXPECT_EQ(win.background.src, "https://x.test/bg.jpg");
  EXPECT_EQ(win.background.fit, ImageFit::Cover);
  const Widget &grp = win.children.at(0);
  EXPECT_EQ(grp.background.bind, "p.banner");
  EXPECT_EQ(grp.background.fit, ImageFit::Contain);
  EXPECT_EQ(grp.background.opacity, 1.0f); // clamped
  EXPECT_EQ(grp.background.terminal, BackdropTerminal::Off);
  EXPECT_EQ(grp.background.auth, "svc");
  bool warned = false;
  for (const std::string &w : lr.warnings)
    warned = warned || w.find("background: is for a window") != std::string::npos;
  EXPECT_TRUE(warned);
  EXPECT_FALSE(win.children.at(1).background.any());
}

TEST(MediaBackground, TheCoreBracketsTheContentAndNamesThePictureOnceLoaded) {
  const std::filesystem::path dir = picture_dir();
  cvc::app app;
  LoadResult lr = load_string(kBackdropDoc);
  ASSERT_TRUE(lr.ok) << lr.error;
  Runtime rt(app);
  rt.set_media_base(dir.string());
  ImageBackend b;
  rt.set_backend(&b);
  rt.set_root(std::move(lr.root));
  rt.render();
  ASSERT_EQ(b.backdrops.size(), 4u);
  EXPECT_EQ(b.backdrops[0], "begin::window"); // nothing to draw yet: still bracketed
  EXPECT_EQ(b.backdrops[1], "begin::group");
  EXPECT_EQ(b.backdrops[2], "end");
  EXPECT_EQ(b.backdrops[3], "end");
  ASSERT_TRUE(settle([&] { rt.render(); },
                     [&] {
                       return read_state(app, "s.win") == "ready" &&
                              read_state(app, "s.banner") == "ready";
                     }));
  rt.render();
  ASSERT_EQ(b.backdrops.size(), 4u);
  EXPECT_EQ(b.backdrops[0].rfind("begin:media:", 0), 0u) << b.backdrops[0];
  EXPECT_NE(b.backdrops[0].find(":window"), std::string::npos);
  // Both name the same picture at its own shape (one load), fitted by the backend.
  EXPECT_EQ(b.backdrops[0].substr(0, b.backdrops[0].size() - 7),
            b.backdrops[1].substr(0, b.backdrops[1].size() - 6));
  const std::string name = b.backdrops[0].substr(6, b.backdrops[0].size() - 6 - 7);
  ASSERT_TRUE(b.published.count(name));
  EXPECT_EQ(b.published[name].width(), 32); // the source's own pixels, not upscaled
  std::filesystem::remove_all(dir);
}

TEST(MediaBackground, FtxuiDrawsHalfBlocksBehindTextAndABannerAbove) {
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "built without FTXUI";
  const std::filesystem::path dir = picture_dir();
  cvc::app app;
  LoadResult lr = load_string(kBackdropDoc);
  ASSERT_TRUE(lr.ok) << lr.error;
  Runtime rt(app);
  rt.set_media_base(dir.string());
  FtxuiBackend::Options opt;
  opt.image_color = FtxuiBackend::Options::ImageColor::Color;
  FtxuiBackend b(opt);
  rt.set_backend(&b);
  rt.set_root(std::move(lr.root));
  std::string frame;
  ASSERT_TRUE(settle([&] { frame = b.render_to_string(rt, 60, 0, false); },
                     [&] {
                       return read_state(app, "s.win") == "ready" &&
                              read_state(app, "s.banner") == "ready";
                     }));
  frame = b.render_to_string(rt, 60, 0, false);
  std::vector<std::string> rows;
  std::size_t at = 0;
  while (at < frame.size()) {
    const std::size_t nl = frame.find('\n', at);
    rows.push_back(frame.substr(at, nl == std::string::npos ? std::string::npos : nl - at));
    at = nl == std::string::npos ? frame.size() : nl + 1;
  }
  // The banner: two rows of picture at the top of the window body, then the group's text, which
  // sits on the window's own background, as does the next line.
  int banner = 0, name_row = -1, follow_row = -1;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    if (rows[i].find("Grace Hopper") != std::string::npos)
      name_row = static_cast<int>(i);
    if (rows[i].find("followers 1024") != std::string::npos)
      follow_row = static_cast<int>(i);
    if (name_row < 0 && rows[i].find("▀▀▀▀▀▀▀▀") != std::string::npos)
      ++banner;
  }
  EXPECT_EQ(banner, 2) << frame;
  ASSERT_GE(name_row, 0) << frame;
  ASSERT_GE(follow_row, 0) << frame;
  // Behind: the cells beside the text still show the picture.
  EXPECT_NE(rows[name_row].find("Grace Hopper▀"), std::string::npos) << frame;
  EXPECT_NE(rows[follow_row].find("followers 1024▀"), std::string::npos) << frame;
  std::filesystem::remove_all(dir);
}
