// Live textures (docs/ARIADNE.md §5.4): `material: { texture: { stream } }` on any node that draws
// a mesh, fed by a raster stream (cvc/ariadne/stream/raster.h) and uploaded a rectangle at a time.
//
//   A. Pixels: a plane wears the raster, north up; painting a rectangle shows at the next frame,
//      and the rest stays as it was.
//   B. Uploads, counted at the driver (gl_upload_counter): the first picture is one glTexImage2D,
//      and then a frame that changed one rectangle is one glTexSubImage2D of just its texels, with
//      no glTexImage2D; a frame that changed nothing uploads nothing.
//   C. The producer may start after the scene, and the source may be a mirror path (a relay's
//      client has one), resolved against the document's prefix; a heightfield takes one too.
//   D. Stock hosts: AriRuntime registers `type: stream` and the raster verbs by default.
// Renders for real; skips (rc 0) where nothing rasterises unless CVC_REQUIRE_RENDER=1.
#include "gl_upload_counter.h"
#include "streaming_test_util.h"

#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/stream/raster.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/stream_node.h>
#include <string>
#include <vector>

using namespace cvcgl_test;
namespace gla = cvc::gl::ariadne;
namespace sx = cvc::ariadne::stream;

namespace {

constexpr int W = 160, H = 160;

// The plane's colour at world (x, y): its pixel in the frame.
const unsigned char *at(cvc::gl::SceneRenderer &sr, const Frame &f, double x, double y) {
  const std::array<double, 2> p = toPx(sr, x, y, 0.0);
  return f.at(static_cast<int>(p[0]), static_cast<int>(p[1]));
}

bool reddish(const unsigned char *c) { return c[0] > 120 && c[0] > 2 * c[1] && c[0] > 2 * c[2]; }
bool bluish(const unsigned char *c) { return c[2] > 120 && c[2] > 2 * c[0] && c[2] > 2 * c[1]; }
std::string rgb(const unsigned char *c) {
  return std::to_string(c[0]) + "," + std::to_string(c[1]) + "," + std::to_string(c[2]);
}

const char *kDoc = R"(meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: ground
      source: { plane: { size: 2 } }
      material: { texture: { stream: streams.tint }, ambient: 1.0, diffuse: 0.0, color: [1, 1, 1] }
    - node: far
      source: { heightfield: { size: 2, resolution: 9 } }
      transform: { position: [10, 0, 0] }
      material: { texture: { stream: fog.cover }, ambient: 1.0, diffuse: 0.0, color: [1, 1, 1] }
    - node: lamp
      type: group
      material: { texture: { stream: streams.tint } }
)";

} // namespace

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  if (!renderAvailable(app))
    return finish("cvcgl_live_texture");

  cvc::ariadne::LoadResult lr = cvc::ariadne::load_string(app, kDoc);
  check(lr.ok, "the document loads", lr.error);
  check(lr.scene.nodes.size() == 3 && lr.scene.nodes[0].material_texture_stream == "streams.tint",
        "material: { texture: { stream } } is parsed");
  cvc::gl::SceneGraph sg(app, "live");
  sg.setDiagnosticChromeVisible(false);
  cvc::gl::SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  std::vector<std::string> warns;
  gla::register_stream_node_type(app);
  gla::RealizedScene r = gla::realize_scene(sg, lr.scene, "live", &warns);
  bool groupWarned = false;
  for (const std::string &w : warns) {
    if (w.find("'lamp': texture: { stream } needs a node that draws a mesh") != std::string::npos)
      groupWarned = true;
    else
      check(false, "realizes without other warnings", w);
  }
  check(groupWarned, "a group, which draws no mesh, is told so");
  auto ground = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("ground"));
  auto far = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("far"));
  check(ground && far, "the textured nodes exist");
  if (!ground || !far)
    return finish("cvcgl_live_texture");
  topView(sr, 0, 0, 1.0);

  std::printf("== C. the producer starts after the scene ==\n");
  gla::tick_scene(r, sr.renderer());
  sg.processEvents();
  check(!ground->hasTexture(), "no stream yet: no texture, and no warning");

  sx::raster_params p;
  p.id = "tint";
  p.w = 64;
  p.h = 64;
  std::unique_ptr<sx::raster_stream> tint = sx::raster_stream::open(app, p);
  check(tint != nullptr && tint->token() == "streams.tint", "a raster stream at streams.tint");
  const unsigned char red[4] = {255, 0, 0, 255}, blue[4] = {0, 0, 255, 255};
  tint->fill({0, 0, 64, 64}, red);
  tint->fill({0, 0, 16, 16}, blue); // its top-left corner: north-west
  tint->publish();

  std::printf("== A. pixels ==\n");
  gla::tick_scene(r, sr.renderer());
  sg.processEvents();
  check(ground->hasTexture() && ground->textureWidth() == 64,
        "the first frame gives the plane its texture");
  Frame f = grab(sr);
  glcountInstall(); // VTK has loaded GL now
  check(reddish(at(sr, f, 0.3, -0.3)), "the plane is the raster's red", rgb(at(sr, f, 0.3, -0.3)));
  check(bluish(at(sr, f, -0.85, 0.85)), "its top-left corner is at the north-west",
        rgb(at(sr, f, -0.85, 0.85)));
  check(reddish(at(sr, f, 0.85, -0.85)), "and not at the south-east", rgb(at(sr, f, 0.85, -0.85)));

  {
    cvc::ariadne::LoadResult sl = cvc::ariadne::load_string(app, R"(meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - { node: screen, type: stream, stream: streams.tint, transform: { position: [20, 0, 0] } }
)");
    std::vector<std::string> sw;
    gla::RealizedScene rs = gla::realize_scene(sg, sl.scene, "live", &sw);
    gla::tick_scene(rs, sr.renderer());
    sg.processEvents();
    auto screen = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("screen"));
    check(sw.empty() && screen && screen->hasTexture() && screen->textureWidth() == 64,
          "a type: stream node shows a raster stream too");
    sg.removeGraphics("screen");
  }

  std::printf("== B. uploads ==\n");
  // One 8 x 8 rectangle in the middle.
  tint->fill({28, 28, 8, 8}, blue);
  tint->publish();
  GLCount g0 = glcountRead();
  gla::tick_scene(r, sr.renderer());
  sg.processEvents();
  f = grab(sr);
  GLCount d = glcountRead() - g0;
  check(d.v[K_TEXSUB] == 1 && d.v[K_TEXSUB_B] == 8 * 8 * 4 && d.v[K_TEXIMG] == 0,
        "one 8 x 8 rectangle: one 256-byte glTexSubImage2D, no glTexImage2D", glcountStr(d));
  check(bluish(at(sr, f, 0.0, 0.0)), "the painted rectangle shows", rgb(at(sr, f, 0.0, 0.0)));
  check(reddish(at(sr, f, 0.5, 0.5)) && bluish(at(sr, f, -0.85, 0.85)), "the rest is as it was");

  g0 = glcountRead();
  gla::tick_scene(r, sr.renderer());
  sg.processEvents();
  f = grab(sr);
  d = glcountRead() - g0;
  check(d.v[K_TEXSUB] == 0 && d.v[K_TEXIMG] == 0, "a frame with nothing new uploads nothing",
        glcountStr(d));

  // Two frames between ticks: each rectangle once.
  tint->fill({2, 40, 4, 4}, blue);
  tint->publish();
  tint->fill({50, 2, 2, 2}, blue);
  tint->publish();
  g0 = glcountRead();
  gla::tick_scene(r, sr.renderer());
  sg.processEvents();
  f = grab(sr);
  d = glcountRead() - g0;
  check(d.v[K_TEXSUB] == 2 && d.v[K_TEXSUB_B] == (16 + 4) * 4 && d.v[K_TEXIMG] == 0,
        "two frames since the last tick: their two rectangles", glcountStr(d));

  // A grid through a colour map, as a sim paints a field.
  std::vector<double> field(16 * 16);
  for (std::size_t k = 0; k < field.size(); ++k)
    field[k] = static_cast<double>(k % 16) / 15.0;
  tint->paint_grid(field, 16, 16, 40, 40, "gray", 0.0, 1.0);
  tint->publish();
  g0 = glcountRead();
  gla::tick_scene(r, sr.renderer());
  sg.processEvents();
  f = grab(sr);
  d = glcountRead() - g0;
  check(d.v[K_TEXSUB] == 1 && d.v[K_TEXSUB_B] == 16 * 16 * 4 && d.v[K_TEXIMG] == 0,
        "a 16 x 16 grid: its texels only", glcountStr(d));

  std::printf("== C. a mirror path, resolved against the prefix, on a heightfield ==\n");
  sx::raster_params mp;
  mp.id = "cover";
  mp.root_path = "server"; // another document's stream: here only its mirror is read
  mp.w = 8;
  mp.h = 8;
  mp.mirror = "live.fog.cover"; // under the scene's prefix, as a client's subscribed namespace is
  std::unique_ptr<sx::raster_stream> cover = sx::raster_stream::open(app, mp);
  check(cover != nullptr, "a mirrored raster");
  cover->fill({0, 0, 8, 8}, red);
  cover->publish();
  gla::tick_scene(r, sr.renderer()); // the follower opens its stream and takes the frame
  gla::tick_scene(r, sr.renderer()); // and the binding, now subscribed, gets a key frame of it
  sg.processEvents();
  check(far->hasTexture() && far->textureWidth() == 8,
        "the heightfield wears the mirrored raster, from a stream of the client's own");
  topView(sr, 10, 0, 1.0);
  f = grab(sr);
  check(reddish(at(sr, f, 10.0, 0.0)), "red, as the producer painted it", rgb(at(sr, f, 10, 0)));
  cover->fill({0, 0, 8, 8}, blue);
  cover->publish();
  gla::tick_scene(r, sr.renderer());
  sg.processEvents();
  f = grab(sr);
  check(bluish(at(sr, f, 10.0, 0.0)), "and blue once it is repainted", rgb(at(sr, f, 10, 0)));

  std::printf("== a repeat:'s copies, each on its own raster ==\n");
  {
    cvc::ariadne::LoadResult tl = cvc::ariadne::load_string(app, R"(meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: tile_{i}
      repeat: 2
      source: { plane: { size: 1 } }
      transform: { position: [30, 0, 0] }
      material: { texture: { stream: "streams.tile{i}" } }
)");
    std::vector<std::string> tw;
    gla::RealizedScene rt = gla::realize_scene(sg, tl.scene, "live", &tw);
    std::unique_ptr<sx::raster_stream> tiles[2];
    for (int k = 0; k < 2; ++k) {
      sx::raster_params tp;
      tp.id = "tile" + std::to_string(k);
      tp.w = tp.h = 8 * (k + 1); // 8 and 16 a side, to tell them apart
      tiles[k] = sx::raster_stream::open(app, tp);
      tiles[k]->fill({0, 0, tp.w, tp.h}, k ? blue : red);
      tiles[k]->publish();
    }
    gla::tick_scene(rt, sr.renderer());
    sg.processEvents();
    auto t0 = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("tile_0"));
    auto t1 = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.getGraphics("tile_1"));
    check(tw.empty() && t0 && t1 && t0->textureWidth() == 8 && t1->textureWidth() == 16,
          "tile_0 wears streams.tile0 and tile_1 streams.tile1");
    sg.removeGraphics("tile_0");
    sg.removeGraphics("tile_1");
  }

  std::printf("== the stream closes, and opens again ==\n");
  tint.reset();
  gla::tick_scene(r, sr.renderer());
  tint = sx::raster_stream::open(app, p);
  tint->fill({0, 0, 64, 64}, blue);
  tint->publish();
  gla::tick_scene(r, sr.renderer());
  gla::tick_scene(r, sr.renderer());
  sg.processEvents();
  topView(sr, 0, 0, 1.0);
  f = grab(sr);
  check(bluish(at(sr, f, 0.5, -0.5)), "a stream reopened on the token is picked up again",
        rgb(at(sr, f, 0.5, -0.5)));

  std::printf("== D. AriRuntime registers streams by default ==\n");
  {
    cvc::app host;
    cvc::gl::SceneGraph hsg(host, "host");
    cvc::gl::SceneRenderer hsr(hsg, 64, 64, /*offscreen=*/true);
    cvc::gl::CameraController cam(hsr);
    cvc::gl::ImGuiOverlay ui(hsr);
    if (!ui.enabled()) {
      std::printf("  skipped: no ImGui overlay\n");
    } else {
      gla::AriRuntime art(hsr, cam, ui);
      check(gla::has_scene_node_type(host, "stream"), "type: stream");
      check(art.app_runtime().runtime().has("raster-open") &&
                art.app_runtime().runtime().has("stream-open"),
            "(raster-open ...) and (stream-open ...)");
    }
  }
  return finish("cvcgl_live_texture");
}
