// ari's image: widget by URL end to end (docs/ARIADNE.md §5.11): the ari binary, run as a launcher
// would, against a localhost server, with and without a credential. Each snapshot is checked for
// the picture's own colours (in --ansi, 24-bit), for the failed state's placeholder, and for the
// token, which no output may carry.
#include "../cvc/tests/local_http_server.h"
#include "ari_run.h"

#include <cstdlib>
#include <cvc/image/image.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <string>

namespace fs = std::filesystem;
using ari_test::ari;
using ari_test::AriRun;
using ari_test::quoted;
using ari_test::TempDir;
using cvc_test::LocalHttpServer;

namespace {

const std::string kToken = "eyJhbGciOiJIUzI1NiJ9.eyJzdWIiOiJtZWRpYSJ9.YXJpLW1lZGlhLXRlc3Q";

// A PNG of one flat colour, as a server would send it.
std::string flat_png(const TempDir &dir, unsigned char r, unsigned char g, unsigned char b) {
  cvc::image img(32, 32, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  unsigned char *p = img.data();
  for (int i = 0; i < 32 * 32; ++i) {
    p[i * 4 + 0] = r;
    p[i * 4 + 1] = g;
    p[i * 4 + 2] = b;
    p[i * 4 + 3] = 255;
  }
  const fs::path path = dir.path / ("flat-" + std::to_string(r) + ".png");
  cvc::write_image(img, path.string());
  std::ifstream in(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// The 24-bit foreground or background escape FTXUI writes for this colour.
std::string sgr(unsigned char r, unsigned char g, unsigned char b) {
  return std::to_string(r) + ";" + std::to_string(g) + ";" + std::to_string(b) + "m";
}

bool have_terminal_ari() {
  const AriRun r = ari("info");
  return r.status == 0 && r.output.find("terminal backend (FTXUI)      yes") != std::string::npos &&
         r.output.find(".ari loader (yaml-cpp)        yes") != std::string::npos &&
         r.output.find("HTTP (cvc::net)               yes") != std::string::npos;
}

#define REQUIRE_ARI()                                                                              \
  do {                                                                                             \
    if (!have_terminal_ari())                                                                      \
      GTEST_SKIP() << "ari lacks FTXUI, yaml-cpp or an HTTP backend";                              \
  } while (0)

void set_env(const char *name, const char *value) {
#if defined(_WIN32)
  _putenv_s(name, value ? value : "");
#else
  if (value)
    setenv(name, value, 1);
  else
    unsetenv(name);
#endif
}

} // namespace

TEST(AriMedia, SnapshotDrawsPicturesByUrlAndTheFailedState) {
  REQUIRE_ARI();
  set_env("COLORTERM", "truecolor"); // --ansi prints 24-bit colour, not the nearest palette entry
  TempDir dir;
  const std::string orange = flat_png(dir, 230, 120, 40);
  const std::string teal = flat_png(dir, 20, 170, 160);
  LocalHttpServer srv([&](const LocalHttpServer::Request &req) -> LocalHttpServer::Reply {
    if (req.target == "/avatar.png")
      return {200, {"Content-Type: image/png"}, orange};
    if (req.target == "/private.png")
      return req.header("Authorization") == "Bearer " + kToken
                 ? LocalHttpServer::Reply{200, {"Content-Type: image/png"}, teal}
                 : LocalHttpServer::Reply{401, {}, "{\"detail\": \"Not authenticated\"}"};
    return {404, {}, "no such picture"};
  });
  const fs::path policy = dir.write(
      "media.policy", "allow " + srv.origin() + "\ncredential media " + srv.origin() + "\n");
  const fs::path doc = dir.write(
      "media.ari", "meta: {min_libcvc: 3.7.0}\n"
                   "windows:\n"
                   "  - window: Media\n"
                   "    children:\n"
                   "      - image: avatar\n"
                   "        src: " +
                       srv.url("/avatar.png") +
                       "\n"
                       "        size: [64, 64]\n"
                       "        shape: circle\n"
                       "        status: s.avatar\n"
                       "      - image: private\n"
                       "        src: " +
                       srv.url("/private.png") +
                       "\n"
                       "        auth: media\n"
                       "        size: [64, 32]\n"
                       "        fit: cover\n"
                       "        status: s.private\n"
                       "      - image: gone\n"
                       "        src: " +
                       srv.url("/gone.png") +
                       "\n"
                       "        size: [64, 32]\n"
                       "        status: s.gone\n"
                       "      - text: \"\"\n"
                       "        bind: (string.join (list \"avatar\" (state-get \"s.avatar\") "
                       "\"private\" (state-get \"s.private\") \"gone\" (state-get \"s.gone\") "
                       "(state-get \"s.gone.error\")) \" \")\n");
  const std::string flags = "--net-policy " + quoted(policy) +
                            " --credential-env media=ARI_MEDIA_TEST_TOKEN --width 60 --ansi ";
  set_env("ARI_MEDIA_TEST_TOKEN", kToken.c_str());
  const AriRun r = ari("snapshot " + flags + quoted(doc));
  set_env("ARI_MEDIA_TEST_TOKEN", nullptr);
  EXPECT_EQ(r.status, 0) << r.output;
  EXPECT_NE(r.output.find("avatar ready private ready gone failed http 404"), std::string::npos)
      << r.output;
  EXPECT_NE(r.output.find(sgr(230, 120, 40)), std::string::npos) << "the avatar's colour";
  EXPECT_NE(r.output.find(sgr(20, 170, 160)), std::string::npos) << "the private picture's";
  EXPECT_NE(r.output.find(sgr(92, 72, 72)), std::string::npos) << "the failed placeholder's";
  EXPECT_EQ(r.output.find(kToken), std::string::npos) << r.output;
  bool sent = false;
  for (const LocalHttpServer::Request &req : srv.requests()) {
    if (req.target == "/private.png")
      sent = sent || req.header("Authorization") == "Bearer " + kToken;
    else
      EXPECT_EQ(req.header("Authorization"), "") << req.target;
  }
  EXPECT_TRUE(sent);
}

TEST(AriMedia, WithoutTheNetworkAnHttpPictureFailsAndAFileOneLoads) {
  REQUIRE_ARI();
  set_env("COLORTERM", "truecolor");
  TempDir dir;
  {
    const std::string png = flat_png(dir, 200, 30, 90);
    std::ofstream(dir.path / "local.png", std::ios::binary) << png;
  }
  const fs::path doc =
      dir.write("offline.ari",
                "meta: {min_libcvc: 3.7.0}\n"
                "windows:\n"
                "  - window: Media\n"
                "    children:\n"
                "      - image: local\n"
                "        src: file://local.png\n"
                "        size: [32, 32]\n"
                "        status: s.local\n"
                "      - image: remote\n"
                "        src: https://example.invalid/a.png\n"
                "        status: s.remote\n"
                "      - text: \"\"\n"
                "        bind: (string.join (list \"local\" (state-get \"s.local\") \"remote\" "
                "(state-get \"s.remote\")) \" \")\n");
  const AriRun r = ari("snapshot --width 60 --ansi " + quoted(doc));
  EXPECT_EQ(r.status, 0) << r.output;
  EXPECT_NE(r.output.find("local ready remote failed"), std::string::npos) << r.output;
  EXPECT_NE(r.output.find(sgr(200, 30, 90)), std::string::npos) << r.output;
}

TEST(AriMedia, ACardFromTheComponentLibraryWithAnAvatarAndAMissingAttachment) {
  REQUIRE_ARI();
  set_env("COLORTERM", "truecolor");
  TempDir dir;
  const std::string orange = flat_png(dir, 230, 120, 40);
  LocalHttpServer srv([&](const LocalHttpServer::Request &req) -> LocalHttpServer::Reply {
    if (req.target == "/u/grace.png")
      return {200, {"Content-Type: image/png"}, orange};
    return {404, {}, "gone"};
  });
  const fs::path components = fs::path(__FILE__).parent_path().parent_path() / "cvc" / "ariadne";
  const fs::path doc = dir.write(
      "card.ari", "meta: {min_libcvc: 3.7.0}\n"
                  "import: cvc://components/card.ari\n"
                  "init: |\n"
                  "  (state-set-tree \"post\" (dict \"name\" \"Grace Hopper\" \"handle\" \"@grace\""
                  " \"when\" \"2h\" \"text\" \"Nanoseconds are this long.\" \"avatar\" \"" +
                      srv.url("/u/grace.png") + "\" \"pic\" \"" + srv.url("/p/9.jpg") +
                      "\"))\n"
                      "windows:\n"
                      "  - window: Feed\n"
                      "    size: [480, auto]\n"
                      "    children:\n"
                      "      - include: card\n"
                      "        args: {id: post, avatar: post.avatar, title: post.name,\n"
                      "               subtitle: post.handle, time: post.when, body: post.text}\n"
                      "        slots:\n"
                      "          media:\n"
                      "            - image: attachment\n"
                      "              bind: post.pic\n"
                      "              size: [320, 48]\n"
                      "              status: s.pic\n"
                      "          actions:\n"
                      "            - button: Reply\n"
                      "              on: (state-set \"s.reply\" \"1\")\n");
  const AriRun r = ari("snapshot --net-allow " + srv.origin() + " --component-path " +
                       quoted(components) + " --width 70 --ansi " + quoted(doc));
  EXPECT_EQ(r.status, 0) << r.output;
  for (const char *want : {"Grace Hopper", "@grace", "2h", "Nanoseconds are this long.", "Reply"})
    EXPECT_NE(r.output.find(want), std::string::npos) << want << "\n" << r.output;
  EXPECT_NE(r.output.find(sgr(230, 120, 40)), std::string::npos) << "the avatar's colour";
  EXPECT_NE(r.output.find(sgr(92, 72, 72)), std::string::npos) << "the attachment's failed box";
  EXPECT_EQ(r.output.find("warning"), std::string::npos) << r.output;
}

// ---- video: / audio: (§5.13) -------------------------------------------------------------------

namespace {

// The ari-media binary beside ari (in-tree, where cvcMedia is built), or "". Its sound goes to
// SDL's dummy driver: no sound card needed.
std::string ari_media() {
  const char *b = std::getenv("ARI_BINARY");
  if (!b || !*b)
    return "";
  fs::path p = fs::path(b).parent_path() / "ari-media";
#if defined(_WIN32)
  p += ".exe";
#endif
  if (!fs::exists(p))
    return "";
  set_env("SDL_AUDIO_DRIVER", "dummy");
  return p.string();
}

AriRun run_with(const std::string &binary, const std::string &args) {
  const char *saved = std::getenv("ARI_BINARY");
  const std::string keep = saved ? saved : "";
  set_env("ARI_BINARY", binary.c_str());
  AriRun r = ari(args);
  set_env("ARI_BINARY", keep.empty() ? nullptr : keep.c_str());
  return r;
}

const fs::path kPlayerDoc = fs::path(__FILE__).parent_path() / "tests" / "media_player.ari";

} // namespace

TEST(AriMedia, TheLeanAriHasNoPlayerAndSaysSo) {
  REQUIRE_ARI();
  const AriRun r = ari("snapshot --width 70 " + quoted(kPlayerDoc));
  EXPECT_EQ(r.status, 0) << r.output;
  EXPECT_NE(r.output.find("video unsupported audio unsupported"), std::string::npos) << r.output;
  EXPECT_NE(r.output.find("no player in this host"), std::string::npos) << r.output;
}

TEST(AriMedia, AriMediaPlaysVideoAndShowsItsPictures) {
  REQUIRE_ARI();
  if (ari_media().empty())
    GTEST_SKIP() << "no ari-media in this build";
  if (run_with(ari_media(), "info").output.find("video/audio player            yes") ==
      std::string::npos)
    GTEST_SKIP() << "ari-media without FFmpeg";
  set_env("COLORTERM", "truecolor");
  set_env("SSH_CONNECTION", nullptr);
  set_env("SSH_CLIENT", nullptr);
  set_env("SSH_TTY", nullptr);
  const AriRun r =
      run_with(ari_media(), "snapshot --width 70 --frames 30 --ansi " + quoted(kPlayerDoc));
  EXPECT_EQ(r.status, 0) << r.output;
  EXPECT_TRUE(r.output.find("video playing audio ready") != std::string::npos ||
              r.output.find("video ready audio ready") != std::string::npos)
      << r.output;
  // testsrc's colour bars, from the decoded frames (no placeholder has them).
  int bars = 0;
  for (const char *c : {"255;255;0m", "0;255;255m", "255;0;255m", "0;255;0m"})
    bars += r.output.find(c) != std::string::npos ||
            r.output.find(std::string(c).replace(0, 3, "254")) != std::string::npos;
  EXPECT_GE(bars, 2) << r.output;
}

TEST(AriMedia, OverSshAriMediaDoesNotPlayUnlessTold) {
  REQUIRE_ARI();
  if (ari_media().empty())
    GTEST_SKIP() << "no ari-media in this build";
  set_env("SSH_CONNECTION", "203.0.113.7 51000 198.51.100.2 22");
  AriRun r = run_with(ari_media(), "snapshot --width 70 " + quoted(kPlayerDoc));
  EXPECT_NE(r.output.find("video remote audio remote"), std::string::npos) << r.output;
  EXPECT_NE(r.output.find("not played over SSH"), std::string::npos) << r.output;
  r = run_with(ari_media(),
               "snapshot --media-play always --width 70 --frames 30 " + quoted(kPlayerDoc));
  EXPECT_EQ(r.output.find("remote"), std::string::npos) << r.output;
  set_env("SSH_CONNECTION", nullptr);
  r = run_with(ari_media(), "snapshot --media-play never --width 70 " + quoted(kPlayerDoc));
  EXPECT_NE(r.output.find("video remote"), std::string::npos) << r.output;
  EXPECT_NE(r.output.find("--media-play never"), std::string::npos) << r.output;
  r = run_with(ari_media(), "snapshot --media-play sometimes " + quoted(kPlayerDoc));
  EXPECT_EQ(r.status, 2) << r.output;
}
