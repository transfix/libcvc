// Ariadne media: pictures by URI. See inc/cvc/ariadne/media.h.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/ariadne/media.h>
#include <cvc/ariadne/net_host.h>
#include <cvc/ariadne/net_intrinsics.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/net/http_client.h>
#include <deque>
#include <exception>
#include <list>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>

namespace cvc {
namespace ariadne {
namespace media {

namespace {

// Whether this build has threads for the background pool (a browser build without -pthread runs
// pool jobs on the frame pump, so the cache runs its steps itself, one per frame).
constexpr bool kThreaded =
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
    false;
#else
    true;
#endif

std::string lower(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

std::string scheme_of(const std::string &ref) {
  const std::size_t at = ref.find("://");
  if (at == std::string::npos || at == 0)
    return std::string();
  for (std::size_t i = 0; i < at; ++i) {
    const unsigned char c = static_cast<unsigned char>(ref[i]);
    if (!(std::isalnum(c) || c == '+' || c == '-' || c == '.'))
      return std::string();
  }
  return lower(ref.substr(0, at));
}

// FNV-1a, 64 bits: a published name from the request's fields.
std::uint64_t fnv1a(const std::string &s, std::uint64_t h = 1469598103934665603ull) {
  for (unsigned char c : s) {
    h ^= c;
    h *= 1099511628211ull;
  }
  return h;
}

std::string hex64(std::uint64_t v) {
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%016llx", static_cast<unsigned long long>(v));
  return buf;
}

// `img` as RGBA u8 (no copy when it already is).
cvc::image rgba8(const cvc::image &img) {
  if (img.format() == cvc::image::pixel_format::RGBA && img.type() == cvc::image::data_type::u8)
    return img;
  return img.converted(cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
}

// The sub-rectangle [x0, x0+w) x [y0, y0+h) of an RGBA u8 image.
cvc::image crop(const cvc::image &src, int x0, int y0, int w, int h) {
  cvc::image out(w, h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  const unsigned char *s = src.data();
  unsigned char *d = out.data();
  const std::size_t sstride = src.row_stride_bytes(), dstride = out.row_stride_bytes();
  for (int y = 0; y < h; ++y)
    std::memcpy(d + y * dstride, s + (y0 + y) * sstride + static_cast<std::size_t>(x0) * 4,
                dstride);
  return out;
}

// Make everything outside the inscribed circle transparent, with a one-pixel soft edge.
void circle_mask(cvc::image &img) {
  const int w = img.width(), h = img.height();
  unsigned char *p = img.data();
  const double cx = w / 2.0, cy = h / 2.0, r = std::min(w, h) / 2.0;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      const double d = std::hypot(x + 0.5 - cx, y + 0.5 - cy);
      const double cover = std::clamp(r - d + 0.5, 0.0, 1.0);
      unsigned char &a = p[(static_cast<std::size_t>(y) * w + x) * 4 + 3];
      a = static_cast<unsigned char>(std::lround(a * cover));
    }
}

cvc::image resize_rgba(const cvc::image &src, int w, int h) {
  if (src.width() == w && src.height() == h)
    return src;
  return src.resized(w, h, cvc::image::resize_filter::box);
}

double now_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

bool parse_fit(const std::string &s, Fit &out) {
  const std::string v = lower(s);
  if (v == "contain")
    out = Fit::Contain;
  else if (v == "cover")
    out = Fit::Cover;
  else if (v == "fill")
    out = Fit::Fill;
  else
    return false;
  return true;
}

bool parse_shape(const std::string &s, Shape &out) {
  const std::string v = lower(s);
  if (v == "rect")
    out = Shape::Rect;
  else if (v == "circle")
    out = Shape::Circle;
  else
    return false;
  return true;
}

bool is_uri(const std::string &ref) {
  const std::string s = scheme_of(ref);
  return s == "http" || s == "https" || s == "file" || s == "cvc" || s == "state";
}

bool is_video_uri(const std::string &ref) {
  if (!is_uri(ref))
    return false;
  const std::string path = lower(ref.substr(0, ref.find_first_of("?#")));
  for (const char *ext : {".mp4", ".webm", ".mov", ".m4v"}) {
    const std::size_t n = std::strlen(ext);
    if (path.size() > n && path.compare(path.size() - n, n, ext) == 0)
      return true;
  }
  return false;
}

const char *status_name(Status s) {
  switch (s) {
  case Status::Loading:
    return "loading";
  case Status::Ready:
    return "ready";
  case Status::Failed:
    return "failed";
  }
  return "loading";
}

int placeholder_height(int box_w, int box_h, Shape shape) {
  if (box_h > 0)
    return box_h;
  return shape == Shape::Circle ? box_w : std::max(1, box_w * 9 / 16);
}

cvc::image fit_image(const cvc::image &src_in, int box_w, int box_h, Fit fit, Shape shape,
                     double scale) {
  if (src_in.empty() || box_w <= 0)
    return cvc::image();
  const cvc::image src = rgba8(src_in);
  const int sw = src.width(), sh = src.height();
  scale = std::max(1.0, scale);
  if (box_h <= 0) { // the image's own aspect at the box's width
    const int w = std::max(1, std::min(sw, static_cast<int>(std::lround(box_w * scale))));
    const int h = std::max(1, static_cast<int>(std::lround(double(w) * sh / sw)));
    cvc::image out = resize_rgba(src, w, h);
    if (shape == Shape::Circle) {
      out = out.converted(cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
      circle_mask(out);
    }
    return out;
  }
  // The raster: the box's aspect, `scale` times the box, but no larger than the source covers.
  const double box_aspect = double(box_w) / double(box_h);
  int rw = static_cast<int>(std::lround(box_w * scale));
  int rh = static_cast<int>(std::lround(box_h * scale));
  const int src_long = fit == Fit::Cover ? std::min(sw, static_cast<int>(sh * box_aspect))
                                         : std::max(sw, static_cast<int>(sh * box_aspect));
  if (rw > src_long) {
    rw = std::max(1, src_long);
    rh = std::max(1, static_cast<int>(std::lround(rw / box_aspect)));
  }
  cvc::image out;
  if (fit == Fit::Fill) {
    out = resize_rgba(src, rw, rh);
  } else if (fit == Fit::Cover) {
    // The largest centred rectangle of the box's aspect inside the source, then scaled.
    int cw = sw, ch = static_cast<int>(std::lround(sw / box_aspect));
    if (ch > sh) {
      ch = sh;
      cw = std::max(1, static_cast<int>(std::lround(sh * box_aspect)));
    }
    ch = std::max(1, ch);
    out = resize_rgba(crop(src, (sw - cw) / 2, (sh - ch) / 2, cw, ch), rw, rh);
  } else { // Contain: the image scaled into the box, centred on transparent pixels
    const double k = std::min(double(rw) / sw, double(rh) / sh);
    const int iw = std::clamp(static_cast<int>(std::lround(sw * k)), 1, rw);
    const int ih = std::clamp(static_cast<int>(std::lround(sh * k)), 1, rh);
    const cvc::image inner = resize_rgba(src, iw, ih);
    out = cvc::image(rw, rh, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
    std::memset(out.data(), 0, out.size_bytes());
    const int ox = (rw - iw) / 2, oy = (rh - ih) / 2;
    const unsigned char *s = inner.data();
    unsigned char *d = out.data();
    for (int y = 0; y < ih; ++y)
      std::memcpy(d + (static_cast<std::size_t>(oy + y) * rw + ox) * 4,
                  s + static_cast<std::size_t>(y) * iw * 4, static_cast<std::size_t>(iw) * 4);
  }
  if (shape == Shape::Circle) {
    out = out.converted(cvc::image::pixel_format::RGBA, cvc::image::data_type::u8); // own buffer
    circle_mask(out);
  }
  return out;
}

cvc::image placeholder_image(int box_w, int box_h, Shape shape, bool failed) {
  box_w = std::max(1, box_w);
  const int h_px = placeholder_height(box_w, box_h, shape);
  // Small (the backends scale it), at the box's aspect.
  const int w = 32;
  const int h = std::max(1, static_cast<int>(std::lround(32.0 * h_px / box_w)));
  cvc::image out(w, h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  unsigned char *p = out.data();
  // Neutral mid-grey while loading (visible on light and dark surfaces); darker, warmer on failure.
  const unsigned char rgb[3] = {static_cast<unsigned char>(failed ? 92 : 128),
                                static_cast<unsigned char>(failed ? 72 : 128),
                                static_cast<unsigned char>(failed ? 72 : 128)};
  for (int i = 0; i < w * h; ++i) {
    p[i * 4 + 0] = rgb[0];
    p[i * 4 + 1] = rgb[1];
    p[i * 4 + 2] = rgb[2];
    p[i * 4 + 3] = 255;
  }
  if (shape == Shape::Circle)
    circle_mask(out);
  return out;
}

FetchResult fetch_bytes(const std::string &uri, const std::string &auth, const std::string &base) {
  FetchResult r;
  const std::string scheme = scheme_of(uri);
  try {
    if (scheme == "http" || scheme == "https") {
      if (const std::optional<NetAccess> access = net_host_access()) {
        HostGet g = host_get(uri, auth, *access);
        r.ok = g.ok;
        r.bytes = std::move(g.body);
        r.error = std::move(g.error);
        return r;
      }
      if (!auth.empty()) {
        r.error = "auth '" + auth + "' needs the host's network (--net and a credential)";
        return r;
      }
    } else if (!auth.empty()) {
      r.error = "auth: is for http(s) URLs only";
      return r;
    }
    UriResult u = resolve(uri, base);
    if (!u.ok) {
      r.error = u.error.empty() ? std::string("could not resolve the URI") : u.error;
      if ((scheme == "http" || scheme == "https") && !has_uri_handler(scheme))
        r.error = "the host has the network off (ari --net-allow ORIGIN)";
      return r;
    }
    r.ok = true;
    r.bytes = std::move(u.content);
  } catch (const std::exception &e) {
    r.error = e.what();
  } catch (...) {
    r.error = "fetch failed";
  }
  return r;
}

namespace {

// fetch_bytes, started rather than sent where the transport starts requests without blocking (a
// browser build without threads, #914): an http(s) URI through the host's network (host_get_start)
// or the http(s) handler's non-blocking form (resolve_async). `done` is called later, from the
// browser's event loop. False when this can't start it (another scheme: local, read in place).
bool start_fetch(const std::string &uri, const std::string &auth, const std::string &base,
                 FetchDone done) {
  const std::string scheme = scheme_of(uri);
  if (!cvc::net::http_starts_async() || (scheme != "http" && scheme != "https"))
    return false;
  if (const std::optional<NetAccess> access = net_host_access()) {
    host_get_start(uri, auth, *access, [done](HostGet g) {
      FetchResult r;
      r.ok = g.ok;
      r.bytes = std::move(g.body);
      r.error = std::move(g.error);
      done(std::move(r));
    });
    return true;
  }
  if (!auth.empty() || !has_uri_async_handler(scheme))
    return false; // fetch_bytes says why, in place
  resolve_async(uri, base, [done](UriResult u) {
    FetchResult r;
    r.ok = u.ok;
    r.bytes = std::move(u.content);
    r.error = u.error;
    done(std::move(r));
  });
  return true;
}

} // namespace

// ---- the cache ----------------------------------------------------------------------------------

struct ImageCache::Impl {
  // One URI under one credential name: its bytes decoded once, shared by every box asking for it.
  struct Base {
    Status status = Status::Loading;
    bool started = false;
    cvc::image decoded;             // downscaled to max_image_px
    std::vector<cvc::image> frames; // an animation's frames (decoded is the first), else empty
    std::vector<int> delays;
    std::string error;
    double failed_at = 0.0;
  };
  // One box (size, fit, shape) of a Base: the raster the widgets draw.
  struct Variant {
    std::string base_key;
    ImageRequest req;
    std::string name;
    Status status = Status::Loading;
    bool preparing = false;
    cvc::image image;
    std::shared_ptr<const Animation> animation;
    long gen = 0;
    std::string error;
    double last_used = 0.0;
    double failed_at = 0.0;
  };

  cvc::app &app;
  Limits limits;
  Fetcher fetcher;
  std::string base_dir;

  // Shared with jobs in flight, which may outlive the cache (they hold it, never the cache).
  struct Shared {
    std::mutex mu;
    std::map<std::string, std::shared_ptr<Base>> bases;
    std::map<std::string, std::shared_ptr<Variant>> variants;
    std::list<std::string> lru; // variant keys, most recently drawn first
    std::vector<std::string> dropped;
    long gen = 0;
  };
  std::shared_ptr<Shared> sh = std::make_shared<Shared>();
  // A build without threads: the steps to run, one per pump() (Runtime::drain, between frames).
  std::deque<std::function<void()>> steps;
  double last_sweep = 0.0;

  explicit Impl(cvc::app &a) : app(a) {}

  static std::string base_key(const ImageRequest &r) { return r.uri + '\n' + r.auth; }
  static std::string variant_key(const ImageRequest &r) {
    return base_key(r) + '\n' + std::to_string(r.w) + 'x' + std::to_string(r.h) + '/' +
           std::to_string(static_cast<int>(r.fit)) + '/' +
           std::to_string(static_cast<int>(r.shape));
  }

  // Run `job` off the frame: on the app's background pool, or as a later step without threads.
  void run(std::function<void()> job) {
    if (kThreaded) {
      app.submitPooled(std::move(job));
      return;
    }
    steps.push_back(std::move(job));
  }

  // Decode the bytes, downscaled to the cache's cap. Off the frame.
  // The fitted raster for `v`, from its base's decoded image: under no lock. Then publish it.
  static void fit_into(const std::shared_ptr<Shared> &s, const std::shared_ptr<Variant> &v,
                       const cvc::image &decoded, const std::vector<cvc::image> &frames = {},
                       const std::vector<int> &delays = {}) {
    cvc::image out = fit_image(decoded, v->req.w, v->req.h, v->req.fit, v->req.shape);
    std::shared_ptr<Animation> anim;
    if (frames.size() > 1 && !out.empty()) { // every frame fitted to the same box
      anim = std::make_shared<Animation>();
      anim->frames.reserve(frames.size());
      for (const cvc::image &f : frames)
        anim->frames.push_back(fit_image(f, v->req.w, v->req.h, v->req.fit, v->req.shape));
      anim->delays_ms = delays;
      for (int d : delays)
        anim->total_ms += d;
    }
    std::lock_guard<std::mutex> lock(s->mu);
    v->preparing = false;
    if (out.empty()) {
      v->status = Status::Failed;
      v->error = "the image has no pixels";
      v->failed_at = now_seconds();
    } else {
      v->status = Status::Ready;
      v->image = std::move(out);
      v->animation = std::move(anim);
      v->gen = ++s->gen;
    }
  }

  // Decode the bytes, downscaled to the cache's cap, then fit them for `v`, the box that asked
  // first. Off the frame.
  static void decode_into(const std::shared_ptr<Shared> &s, const std::shared_ptr<Base> &b,
                          const std::shared_ptr<Variant> &v, FetchResult fr, const Limits &lim) {
    cvc::image img;
    std::vector<cvc::image> frames;
    std::vector<int> delays;
    std::string why = fr.error;
    if (fr.ok) {
      cvc::image_animation anim =
          cvc::decode_image_animation(fr.bytes.data(), fr.bytes.size(), lim.decode, &why);
      const auto shrink = [&](const cvc::image &f) {
        const int lng = std::max(f.width(), f.height());
        if (lng <= lim.max_image_px || lim.max_image_px <= 0)
          return f;
        const double k = double(lim.max_image_px) / lng;
        return f.resized(std::max(1, static_cast<int>(std::lround(f.width() * k))),
                         std::max(1, static_cast<int>(std::lround(f.height() * k))),
                         cvc::image::resize_filter::box);
      };
      if (!anim.frames.empty())
        img = shrink(anim.frames.front());
      if (anim.frames.size() > 1) {
        for (const cvc::image &f : anim.frames)
          frames.push_back(shrink(f));
        delays = anim.delays_ms;
      }
    }
    // The other boxes already waiting on this picture are fitted here too, so they need no
    // frame to start them (ari snapshot settles without drawing).
    // A failure reaches every box waiting on this picture at once, for the same reason.
    std::vector<std::shared_ptr<Variant>> waiting{v};
    {
      std::lock_guard<std::mutex> lock(s->mu);
      for (const auto &kv : s->variants) {
        Variant &w = *kv.second;
        if (kv.second != v && w.status == Status::Loading && !w.preparing &&
            s->bases.count(w.base_key) && s->bases.at(w.base_key) == b)
          waiting.push_back(kv.second);
      }
      if (img.empty()) {
        b->status = Status::Failed;
        b->error = why.empty() ? std::string("could not load the image") : why;
        b->failed_at = now_seconds();
        for (const auto &w : waiting) {
          w->preparing = false;
          w->status = Status::Failed;
          w->error = b->error;
          w->failed_at = b->failed_at;
        }
        return;
      }
      b->status = Status::Ready;
      b->decoded = img;
      b->frames = frames;
      b->delays = delays;
      for (const auto &w : waiting)
        w->preparing = true;
    }
    for (const auto &w : waiting)
      fit_into(s, w, img, frames, delays);
  }

  // Under sh->mu: mark both busy; the caller starts the fetch outside the lock.
  void start_base(const std::shared_ptr<Base> &b, const std::shared_ptr<Variant> &v) {
    std::shared_ptr<Shared> s = sh;
    const Limits lim = limits;
    FetchDone done = [s, b, v, lim](FetchResult fr) { decode_into(s, b, v, std::move(fr), lim); };
    if (fetcher) {
      fetcher(v->req.uri, v->req.auth, std::move(done));
      return;
    }
    const std::string uri = v->req.uri, auth = v->req.auth, dir = base_dir;
    // A browser build without threads starts the request, and the frame loop goes on while it is
    // out; the reply is decoded when the browser hands it over, between frames.
    if (!kThreaded && start_fetch(uri, auth, dir, done))
      return;
    run([uri, auth, dir, done = std::move(done)] { done(fetch_bytes(uri, auth, dir)); });
  }

  void start_variant(const std::shared_ptr<Variant> &v, const cvc::image &decoded,
                     const std::vector<cvc::image> &frames, const std::vector<int> &delays) {
    std::shared_ptr<Shared> s = sh;
    run([s, v, decoded, frames, delays] { fit_into(s, v, decoded, frames, delays); });
  }

  // Drop pictures no widget asked for lately, and the least recently drawn beyond the byte cap.
  // Under sh->mu.
  void sweep(double now) {
    std::size_t total = 0;
    for (auto it = sh->lru.begin(); it != sh->lru.end();) {
      const auto vi = sh->variants.find(*it);
      if (vi == sh->variants.end()) {
        it = sh->lru.erase(it);
        continue;
      }
      Variant &v = *vi->second;
      const bool busy = v.preparing;
      std::size_t size = v.image.size_bytes();
      if (v.animation)
        for (const cvc::image &f : v.animation->frames)
          size += f.size_bytes();
      if (!busy && (now - v.last_used > limits.idle_seconds || total + size > limits.max_bytes)) {
        sh->dropped.push_back(v.name);
        sh->variants.erase(vi);
        it = sh->lru.erase(it);
        continue;
      }
      total += size;
      ++it;
    }
    // A Base no variant uses any more goes too (it is the big one: up to max_image_px).
    for (auto it = sh->bases.begin(); it != sh->bases.end();) {
      bool used = false;
      for (const auto &kv : sh->variants)
        if (kv.second->base_key == it->first) {
          used = true;
          break;
        }
      const bool busy = it->second->status == Status::Loading && it->second->started;
      it = used || busy ? std::next(it) : sh->bases.erase(it);
    }
  }
};

ImageCache::ImageCache(cvc::app &app) : m_(new Impl(app)) {}

ImageCache::~ImageCache() = default;

void ImageCache::set_base(std::string dir) { m_->base_dir = std::move(dir); }
void ImageCache::set_fetcher(Fetcher f) { m_->fetcher = std::move(f); }
void ImageCache::set_limits(const Limits &limits) { m_->limits = limits; }
const ImageCache::Limits &ImageCache::limits() const { return m_->limits; }

void ImageCache::pump() {
  if (m_->steps.empty())
    return;
  std::function<void()> step = std::move(m_->steps.front());
  m_->steps.pop_front();
  step();
}

ImageView ImageCache::image(const ImageRequest &req) {
  Impl &m = *m_;
  const double now = now_seconds();
  const std::string vkey = Impl::variant_key(req);
  const std::string bkey = Impl::base_key(req);
  std::shared_ptr<Impl::Variant> v;
  std::shared_ptr<Impl::Base> b;
  bool start_b = false;
  cvc::image decoded;
  std::vector<cvc::image> frames;
  std::vector<int> delays;
  bool start_v = false;
  ImageView view;
  {
    std::lock_guard<std::mutex> lock(m.sh->mu);
    auto &slot = m.sh->variants[vkey];
    if (!slot) {
      slot = std::make_shared<Impl::Variant>();
      slot->base_key = bkey;
      slot->req = req;
      slot->name = "media:" + hex64(fnv1a(vkey));
      m.sh->lru.push_front(vkey);
    } else {
      // Most recently drawn first (an O(n) move; n is the pictures on screen).
      auto it = std::find(m.sh->lru.begin(), m.sh->lru.end(), vkey);
      if (it != m.sh->lru.end() && it != m.sh->lru.begin())
        m.sh->lru.splice(m.sh->lru.begin(), m.sh->lru, it);
    }
    v = slot;
    v->last_used = now;
    auto &bslot = m.sh->bases[bkey];
    if (!bslot)
      bslot = std::make_shared<Impl::Base>();
    b = bslot;
    // A failure is asked again after retry_seconds.
    if (v->status == Status::Failed && now - v->failed_at > m.limits.retry_seconds) {
      v->status = Status::Loading;
      v->error.clear();
      if (b->status == Status::Failed) {
        *b = Impl::Base{};
      }
    }
    if (v->status == Status::Loading && !v->preparing) {
      if (!b->started) {
        start_b = true;
        b->started = true;
        v->preparing = true;
      } else if (b->status == Status::Failed) {
        v->status = Status::Failed;
        v->error = b->error;
        v->failed_at = b->failed_at;
      } else if (b->status == Status::Ready) {
        start_v = true;
        v->preparing = true;
        decoded = b->decoded;
        frames = b->frames;
        delays = b->delays;
      }
    }
    // After this request touched its picture, so the sweep never drops what is being drawn.
    if (now - m.last_sweep >= m.limits.sweep_seconds) {
      m.last_sweep = now;
      m.sweep(now);
    }
    view.status = v->status;
    view.name = v->name;
    view.image = v->image;
    view.animation = v->animation;
    view.gen = v->gen;
    view.error = v->error;
  }
  // Outside the lock: a fetcher may answer at once, and its answer takes the lock.
  if (start_b)
    m.start_base(b, v);
  if (start_v)
    m.start_variant(v, decoded, frames, delays);
  return view;
}

std::vector<std::string> ImageCache::take_dropped() {
  std::lock_guard<std::mutex> lock(m_->sh->mu);
  std::vector<std::string> out;
  out.swap(m_->sh->dropped);
  return out;
}

std::size_t Animation::frame_at(double seconds) const {
  if (frames.size() < 2 || total_ms <= 0)
    return 0;
  long t = static_cast<long>(std::fmod(std::max(0.0, seconds) * 1000.0, double(total_ms)));
  for (std::size_t i = 0; i < delays_ms.size(); ++i) {
    if (t < delays_ms[i])
      return i;
    t -= delays_ms[i];
  }
  return frames.size() - 1;
}

bool ImageCache::busy() const {
  if (!m_->steps.empty())
    return true;
  std::lock_guard<std::mutex> lock(m_->sh->mu);
  for (const auto &kv : m_->sh->variants)
    if (kv.second->status == Status::Loading)
      return true;
  return false;
}

std::size_t ImageCache::entries() const {
  std::lock_guard<std::mutex> lock(m_->sh->mu);
  return m_->sh->variants.size();
}

std::size_t ImageCache::bytes() const {
  std::lock_guard<std::mutex> lock(m_->sh->mu);
  std::size_t total = 0;
  for (const auto &kv : m_->sh->variants)
    total += kv.second->image.size_bytes();
  for (const auto &kv : m_->sh->bases)
    total += kv.second->decoded.size_bytes();
  return total;
}

} // namespace media
} // namespace ariadne
} // namespace cvc
