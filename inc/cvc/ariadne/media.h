#ifndef CVC_ARIADNE_MEDIA_H
#define CVC_ARIADNE_MEDIA_H

// Ariadne media (docs/ARIADNE.md §5.11; design: docs/roadmap/ARIADNE_MEDIA.md) — pictures a
// document names by URI. An `image:` widget whose src (or bound value) is an http(s), file, cvc or
// state URI asks the Runtime's ImageCache for it each frame. The cache answers at once: loading,
// ready (a raster already fitted to the widget's box) or failed. The fetch, the decode, the
// downscale and the fit run off the frame (the app's background pool; in a browser build without
// threads, a step per Runtime::drain), and the frame thread only publishes the finished raster with
// Backend::set_image and draws it. So every backend that draws a published image draws these:
// ImGui, FTXUI and tvision (Kitty, Sixel, iTerm or half-blocks).
//
// http(s) goes through the host's network (net_host.h): its allowlist, and with `auth: NAME` its
// credential NAME, put in as the request is sent, on the terms the http verbs keep (§10.3). The
// token never reaches state, a cache key, an error or a published name.

#include <cstddef>
#include <cvc/ariadne/widget.h>
#include <cvc/image/image.h>
#include <cvc/image/image_decode.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
namespace media {

// How an image fills its box (the widget's `size: [w, h]`), and the box's shape: `circle` makes
// everything outside the inscribed circle transparent (avatars). widget.h's enums.
using Fit = ImageFit;
using Shape = ImageShape;

// "contain" | "cover" | "fill" -> `out`; false (and `out` untouched) for anything else.
bool parse_fit(const std::string &s, Fit &out);
// "rect" | "circle" -> `out`; false for anything else.
bool parse_shape(const std::string &s, Shape &out);

// Whether an image: reference names a URI the cache loads (http://, https://, file://, cvc://,
// state://) rather than an image the host published by name.
bool is_uri(const std::string &ref);

// Whether a URI names a video by its extension (.mp4, .webm, .mov, .m4v; a query or fragment after
// it is fine), as web clients decide an avatar or a banner is one: an image: or a background: with
// such a URI plays it as a decorative video (§5.13).
bool is_video_uri(const std::string &ref);

// The decoded image `src` as a raster with the box's aspect (`box_w` x `box_h` px, as `fit` and
// `shape` say), `scale` times the box's size (2: sharp on a HiDPI screen) but no larger than the
// source's pixels allow. `box_h` <= 0: the image's own aspect at width `box_w` (`fit` unused). RGBA
// u8. Pure; an empty `src` or box gives an empty image.
cvc::image fit_image(const cvc::image &src, int box_w, int box_h, Fit fit, Shape shape,
                     double scale = 2.0);

// What an image widget shows while its picture loads (`failed` false) or after it failed (true):
// a flat neutral raster with the box's aspect (a disc for Shape::Circle), so the layout does not
// move when the picture arrives. `box_h` <= 0 is 16:9 (1:1 for a circle). Small: the backends
// scale it up.
cvc::image placeholder_image(int box_w, int box_h, Shape shape, bool failed);

// The box a widget's placeholder and fitted raster share, in px: `box_h` <= 0 keeps the width and
// takes 16:9 (1:1 for a circle) until the picture's own aspect is known.
int placeholder_height(int box_w, int box_h, Shape shape);

enum class Status { Loading, Ready, Failed };
const char *status_name(Status s); // "loading" | "ready" | "failed"

struct ImageRequest {
  std::string uri;
  std::string auth; // a credential NAME (never a token); "" = none
  int w = 256;      // the box, px
  int h = 0;        // <= 0: the image's own aspect at width w
  Fit fit = Fit::Contain;
  Shape shape = Shape::Rect;
};

// An animated picture's frames, fitted to the widget's box like ImageView::image (its first).
struct Animation {
  std::vector<cvc::image> frames;
  std::vector<int> delays_ms;
  int total_ms = 0;
  // The frame showing `seconds` into a loop that started at 0.
  std::size_t frame_at(double seconds) const;
};

struct ImageView {
  Status status = Status::Loading;
  // The name the raster is published under (Backend::set_image): stable for the request, a hash
  // of the URI, the credential's name and the box, so it holds no token.
  std::string name;
  cvc::image image;  // the fitted raster when Ready
  long gen = 0;      // bumped each time `image` changes (publish again when it moves)
  std::string error; // why it failed (Failed); never holds a token
  // An animated GIF or WebP's frames (more than one), when Ready; null for a still picture.
  std::shared_ptr<const Animation> animation;
};

// What a fetch brings back: the encoded bytes, or why there are none.
struct FetchResult {
  bool ok = false;
  std::string bytes;
  std::string error;
};
using FetchDone = std::function<void(FetchResult)>;
// How the cache gets a URI's bytes. Called on the frame thread; it may call `done` at once, later
// on that thread, or on any other thread. The default (fetch_bytes) runs on the app's background
// pool.
using Fetcher =
    std::function<void(const std::string &uri, const std::string &auth, FetchDone done)>;

// The default fetch, blocking: an http(s) URI through the host's network when enable_net turned it
// on (its allowlist; `auth` through its credential store, never following a redirect), else
// through whatever http(s) handler the host registered; any other URI through the resolver
// (uri.h), relative file:// paths against `base`. `auth` on anything but http(s), or with no
// credential store, is an error. A reply that is not 2xx is an error ("http 404").
FetchResult fetch_bytes(const std::string &uri, const std::string &auth,
                        const std::string &base = std::string());

// The Runtime's cache of pictures by URI. Frame-thread API; its jobs run on the app's background
// pool (or, in a build without threads, a step per image() call).
class ImageCache {
public:
  struct Limits {
    int max_image_px = 2048;                     // a decoded image's long side, after downscaling
    std::size_t max_bytes = 64u * 1024u * 1024u; // decoded rasters kept, LRU beyond it
    double idle_seconds = 30.0;                  // a picture no widget asked for is dropped
    double retry_seconds = 30.0;                 // a failed picture is asked for again after it
    double sweep_seconds = 1.0;                  // how often idle and over-cap pictures are dropped
    image_decode_limits decode;                  // the decoder's size caps
  };

  explicit ImageCache(cvc::app &app);
  ~ImageCache();
  ImageCache(const ImageCache &) = delete;
  ImageCache &operator=(const ImageCache &) = delete;

  // What `req` shows now. Starts the load on first sight; never waits.
  ImageView image(const ImageRequest &req);
  // In a build without threads, run the next waiting step (a fetch with its decode, or a fit), on
  // this thread: the Runtime calls it from drain(), between frames, never inside the walk. With
  // threads the steps run on the app's background pool and this does nothing.
  void pump();

  // Relative file:// URIs resolve against `dir` (the document's directory).
  void set_base(std::string dir);
  // Replace how bytes are fetched (a test's fake, a host's own transport). Null: fetch_bytes.
  void set_fetcher(Fetcher f);
  void set_limits(const Limits &limits);
  const Limits &limits() const;

  // The names whose rasters the cache dropped since the last call, so the caller unpublishes them
  // (Backend::drop_image).
  std::vector<std::string> take_dropped();

  // Whether a picture asked for is still loading (a host waiting for the document to settle, as
  // ari snapshot does, waits for it too).
  bool busy() const;
  std::size_t entries() const; // pictures kept (each box of each URI)
  std::size_t bytes() const;   // their decoded rasters' size

private:
  struct Impl;
  std::unique_ptr<Impl> m_;
};

} // namespace media
} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_MEDIA_H
