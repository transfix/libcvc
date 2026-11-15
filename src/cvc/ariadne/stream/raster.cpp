/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A raster as a stream: the rgba8.patch frames, their producer and consumer, and a mirror in
// state — see the header.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cvc/ariadne/bind.h> // read_node / write_node: a mirror under a hole
#include <cvc/ariadne/stream/raster.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/core/app.h>
#include <cvc/geometry/mesh_ops.h> // colormap_rgb: a grid's colours
#include <cvc/state/state.h>
#include <functional>
#include <limits>
#include <map>
#include <mutex>

namespace cvc {
namespace ariadne {
namespace stream {

namespace {

// The frame's layout, little-endian: magic "RPF1", flags (bit 0: key), 3 reserved bytes, then the
// frame number, w, h, the count of changed rectangles and of patches (u32 each), the changed
// rectangles (x, y, w, h as u32), and each patch: its rectangle, then its pixels.
constexpr std::uint8_t kMagic[4] = {'R', 'P', 'F', '1'};
constexpr std::size_t kHeader = 28;
constexpr int kMaxSide = 8192;

void put_u32(std::vector<std::uint8_t> &out, std::uint32_t v) {
  for (int k = 0; k < 4; ++k)
    out.push_back(static_cast<std::uint8_t>(v >> (8 * k)));
}

std::uint32_t get_u32(const std::uint8_t *p) {
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

void put_rect(std::vector<std::uint8_t> &out, const raster_rect &r) {
  put_u32(out, static_cast<std::uint32_t>(r.x));
  put_u32(out, static_cast<std::uint32_t>(r.y));
  put_u32(out, static_cast<std::uint32_t>(r.w));
  put_u32(out, static_cast<std::uint32_t>(r.h));
}

// Copy rows of `r` between two rasters of width `w` (4 bytes a pixel), or from packed rows.
void copy_rect(std::uint8_t *dst, int dst_w, const std::uint8_t *src, int src_w,
               const raster_rect &r, int src_x, int src_y) {
  const std::size_t row = static_cast<std::size_t>(r.w) * 4;
  for (int j = 0; j < r.h; ++j)
    std::memcpy(dst + (static_cast<std::size_t>(r.y + j) * dst_w + r.x) * 4,
                src + (static_cast<std::size_t>(src_y + j) * src_w + src_x) * 4, row);
}

// Drop the empty rectangles of `in` (clamped) and fold them into at most `most`: past it they
// become their bounding box.
std::vector<raster_rect> tidy(const std::vector<raster_rect> &in, int w, int h,
                              std::size_t most = 64) {
  std::vector<raster_rect> out;
  for (const raster_rect &r : in) {
    const raster_rect c = clamp_rect(r, w, h);
    if (!c.empty())
      out.push_back(c);
  }
  if (out.size() <= most)
    return out;
  int x0 = w, y0 = h, x1 = 0, y1 = 0;
  for (const raster_rect &r : out) {
    x0 = std::min(x0, r.x);
    y0 = std::min(y0, r.y);
    x1 = std::max(x1, r.x + r.w);
    y1 = std::max(y1, r.y + r.h);
  }
  return {raster_rect{x0, y0, x1 - x0, y1 - y0}};
}

std::string sanitized(const std::string &path) {
  std::string s = path;
  for (char &c : s)
    if (!std::isalnum(static_cast<unsigned char>(c)))
      c = '_';
  return s;
}

std::string child(const std::string &path, const std::string &leaf) {
  return path + cvc::state::SEPARATOR + leaf;
}

// The text of the state node at `path` (through links), or "".
std::string text_at(app &ctx, const std::string &path) {
  cvc::state *s = read_node(ctx, path);
  return s ? s->value() : std::string();
}

std::uint32_t number_at(app &ctx, const std::string &path) {
  const std::string v = text_at(ctx, path);
  try {
    return v.empty() ? 0u : static_cast<std::uint32_t>(std::stoul(v));
  } catch (const std::exception &) {
    return 0u;
  }
}

// The bytes the node at `path` holds as data (a std::string), or "".
std::string bytes_at(app &ctx, const std::string &path) {
  cvc::state *s = read_node(ctx, path);
  if (!s)
    return std::string();
  try {
    return boost::any_cast<std::string>(s->data());
  } catch (const boost::bad_any_cast &) {
    return std::string();
  }
}

} // namespace

raster_rect clamp_rect(const raster_rect &r, int w, int h) noexcept {
  const long long x0 = std::max(0LL, static_cast<long long>(r.x));
  const long long y0 = std::max(0LL, static_cast<long long>(r.y));
  const long long x1 = std::min(static_cast<long long>(w), static_cast<long long>(r.x) + r.w);
  const long long y1 = std::min(static_cast<long long>(h), static_cast<long long>(r.y) + r.h);
  if (x1 <= x0 || y1 <= y0 || r.w <= 0 || r.h <= 0)
    return raster_rect{};
  return raster_rect{static_cast<int>(x0), static_cast<int>(y0), static_cast<int>(x1 - x0),
                     static_cast<int>(y1 - y0)};
}

std::vector<std::uint8_t> encode_raster_frame(const std::uint8_t *pixels, int w, int h,
                                              std::uint32_t number, bool key,
                                              const std::vector<raster_rect> &changed) {
  const std::vector<raster_rect> rects = tidy(changed, w, h);
  const std::vector<raster_rect> patches =
      key ? std::vector<raster_rect>{raster_rect{0, 0, w, h}} : rects;
  std::size_t bytes = kHeader + rects.size() * 16;
  for (const raster_rect &r : patches)
    bytes += 16 + static_cast<std::size_t>(r.w) * r.h * 4;
  std::vector<std::uint8_t> out;
  out.reserve(bytes);
  out.insert(out.end(), kMagic, kMagic + 4);
  out.push_back(key ? 1 : 0);
  out.push_back(0);
  out.push_back(0);
  out.push_back(0);
  put_u32(out, number);
  put_u32(out, static_cast<std::uint32_t>(w));
  put_u32(out, static_cast<std::uint32_t>(h));
  put_u32(out, static_cast<std::uint32_t>(rects.size()));
  put_u32(out, static_cast<std::uint32_t>(patches.size()));
  for (const raster_rect &r : rects)
    put_rect(out, r);
  for (const raster_rect &r : patches) {
    put_rect(out, r);
    const std::size_t at = out.size();
    out.resize(at + static_cast<std::size_t>(r.w) * r.h * 4);
    for (int j = 0; j < r.h; ++j)
      std::memcpy(out.data() + at + static_cast<std::size_t>(j) * r.w * 4,
                  pixels + (static_cast<std::size_t>(r.y + j) * w + r.x) * 4,
                  static_cast<std::size_t>(r.w) * 4);
  }
  return out;
}

bool decode_raster_frame(const std::uint8_t *data, std::size_t size, raster_frame_view &out) {
  out = raster_frame_view{};
  if (!data || size < kHeader || std::memcmp(data, kMagic, 4) != 0)
    return false;
  raster_frame_view f;
  f.key = (data[4] & 1u) != 0;
  f.number = get_u32(data + 8);
  const std::uint32_t w = get_u32(data + 12), h = get_u32(data + 16);
  const std::uint32_t nc = get_u32(data + 20), np = get_u32(data + 24);
  if (w == 0 || h == 0 || w > kMaxSide || h > kMaxSide)
    return false;
  f.w = static_cast<int>(w);
  f.h = static_cast<int>(h);
  std::size_t at = kHeader;
  const auto rect = [&](raster_rect &r) {
    if (size - at < 16)
      return false;
    const std::uint32_t x = get_u32(data + at), y = get_u32(data + at + 4);
    const std::uint32_t rw = get_u32(data + at + 8), rh = get_u32(data + at + 12);
    at += 16;
    // Inside the raster, each bound checked on its own (no u32 overflow).
    if (rw == 0 || rh == 0 || x >= w || y >= h || rw > w - x || rh > h - y)
      return false;
    r = raster_rect{static_cast<int>(x), static_cast<int>(y), static_cast<int>(rw),
                    static_cast<int>(rh)};
    return true;
  };
  if (nc > (size - at) / 16)
    return false;
  for (std::uint32_t k = 0; k < nc; ++k) {
    raster_rect r;
    if (!rect(r))
      return false;
    f.changed.push_back(r);
  }
  for (std::uint32_t k = 0; k < np; ++k) {
    raster_patch p;
    if (!rect(p.rect))
      return false;
    const std::size_t bytes = static_cast<std::size_t>(p.rect.w) * p.rect.h * 4;
    if (size - at < bytes)
      return false;
    p.pixels = data + at;
    at += bytes;
    f.patches.push_back(p);
  }
  if (f.key && (f.patches.size() != 1 || f.patches[0].rect.w != f.w || f.patches[0].rect.h != f.h))
    return false; // a key frame is the whole raster, one patch
  out = std::move(f);
  return true;
}

// --- raster_canvas -------------------------------------------------------------------------------

std::uint8_t *raster_canvas::pixels() noexcept {
  // The canvas's own buffer (never an adopted one), written in place: a texture that aliases its
  // storage sees the writes without a copy-on-write fork.
  return image_.empty() ? nullptr : image_.storage().get();
}

raster_canvas::update raster_canvas::apply(const raster_frame_view &f) {
  update u;
  if (f.w <= 0 || f.h <= 0)
    return u;
  if (image_.empty() || image_.width() != f.w || image_.height() != f.h) {
    image_ = cvc::image(f.w, f.h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
    std::memset(image_.storage().get(), 0, image_.size_bytes());
    u.resized = true;
    synced_ = false;
  }
  std::uint8_t *px = pixels();
  const bool in_order = synced_ && f.number == number_ + 1;
  if (f.key) {
    const raster_patch &all = f.patches.front();
    if (in_order) {
      // Only what changed: the rest is already here.
      for (const raster_rect &r : f.changed)
        copy_rect(px, f.w, all.pixels, f.w, r, r.x, r.y);
      u.rects = f.changed;
    } else {
      std::memcpy(px, all.pixels, static_cast<std::size_t>(f.w) * f.h * 4);
      u.all = true;
    }
    synced_ = true;
  } else {
    if (synced_ && f.number <= number_)
      return u; // one it already has (a consumer that caught up from a mirror): nothing to do
    if (!in_order) {
      ++skipped_;
      synced_ = false;
      return u; // a part of a picture this canvas does not have: wait for a key frame
    }
    for (const raster_patch &p : f.patches)
      copy_rect(px, f.w, p.pixels, p.rect.w, p.rect, 0, 0);
    u.rects = f.changed;
  }
  number_ = f.number;
  u.applied = true;
  if (u.resized)
    u.all = true;
  return u;
}

// --- raster_stream -------------------------------------------------------------------------------

raster_stream::raster_stream(app &ctx) : ctx_(ctx) {}

raster_stream::~raster_stream() {
  if (stream_)
    stream_->close();
}

const std::string &raster_stream::token() const { return stream_->token(); }

std::unique_ptr<raster_stream> raster_stream::open(app &ctx, const raster_params &p,
                                                   std::string *error) {
  const auto fail = [&](const std::string &why) -> std::unique_ptr<raster_stream> {
    if (error)
      *error = why;
    return nullptr;
  };
  if (p.w <= 0 || p.h <= 0 || p.w > kMaxSide || p.h > kMaxSide)
    return fail("a raster is 1 to " + std::to_string(kMaxSide) + " pixels a side");
  stream_params sp;
  sp.id = p.id;
  sp.root_path = p.root_path;
  sp.format.kind = frame_kind::video_raw;
  sp.format.codec = kRasterCodec;
  sp.format.w = p.w;
  sp.format.h = p.h;
  sp.format.stride = p.w * 4;
  // Frames are published from their own buffers (publish_external), each its own size; the pool
  // only keeps the subscribers' accounting, so its slabs are a header each.
  sp.format.bytes = kHeader;
  sp.expected_subscribers = std::max<std::size_t>(1, p.expected_subscribers);
  sp.subscriber_depth = std::max<std::size_t>(1, p.subscriber_depth);
  std::unique_ptr<raster_stream> r(new raster_stream(ctx));
  r->stream_ = stream::open(ctx, sp);
  if (!r->stream_)
    return fail("no stream '" + p.id + "' (an empty or dotted id, or one already live there)");
  r->w_ = p.w;
  r->h_ = p.h;
  r->key_every_ = std::max(0, p.key_every);
  r->pixels_ =
      std::make_shared<std::vector<std::uint8_t>>(static_cast<std::size_t>(p.w) * p.h * 4, 0);
  r->mirror_ = p.mirror;
  // Its frames are kept in state at its own token (so a consumer that subscribes late catches up
  // there, raster_catch_up), and at `mirror` too when one is named.
  r->mirrors_.push_back(r->stream_->token());
  if (!r->mirror_.empty() && r->mirror_ != r->stream_->token())
    r->mirrors_.push_back(r->mirror_);
  for (const std::string &path : r->mirrors_) {
    // Every node first touched, so a replicating observer is attached before its first write
    // (as stream::open does for its descriptor), then the raster's size.
    write_node(ctx, path).touch();
    for (const char *leaf : {"w", "h", "codec", "key", "frame"})
      write_node(ctx, child(path, leaf)).touch();
    for (int k = 0; k < kRasterRing; ++k)
      write_node(ctx, child(child(path, "ring"), std::to_string(k))).touch();
    write_node(ctx, child(path, "codec")).value(std::string(kRasterCodec));
    write_node(ctx, child(path, "w")).value(p.w);
    write_node(ctx, child(path, "h")).value(p.h);
  }
  return r;
}

void raster_stream::mark(const raster_rect &r) {
  const raster_rect c = clamp_rect(r, w_, h_);
  if (!c.empty())
    marked_.push_back(c);
}

void raster_stream::fill(const raster_rect &r, const std::uint8_t rgba[4]) {
  const raster_rect c = clamp_rect(r, w_, h_);
  if (c.empty())
    return;
  for (int j = 0; j < c.h; ++j) {
    std::uint8_t *row = pixels_->data() + (static_cast<std::size_t>(c.y + j) * w_ + c.x) * 4;
    for (int i = 0; i < c.w; ++i)
      std::memcpy(row + static_cast<std::size_t>(i) * 4, rgba, 4);
  }
  marked_.push_back(c);
}

void raster_stream::paint(const cvc::image &img, int x, int y) {
  if (img.empty())
    return;
  const cvc::image rgba =
      (img.format() == cvc::image::pixel_format::RGBA && img.type() == cvc::image::data_type::u8)
          ? img
          : img.converted(cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  const raster_rect c = clamp_rect(raster_rect{x, y, rgba.width(), rgba.height()}, w_, h_);
  if (c.empty())
    return;
  copy_rect(pixels_->data(), w_, rgba.data(), rgba.width(), c, c.x - x, c.y - y);
  marked_.push_back(c);
}

bool raster_stream::paint_grid(const std::vector<double> &values, int gw, int gh, int x, int y,
                               const std::string &colormap, double lo, double hi,
                               std::uint8_t alpha) {
  cvc::colormap_kind kind;
  if (gw <= 0 || gh <= 0 || values.size() != static_cast<std::size_t>(gw) * gh ||
      !cvc::colormap_from_string(colormap, kind))
    return false;
  const raster_rect c = clamp_rect(raster_rect{x, y, gw, gh}, w_, h_);
  if (c.empty())
    return true;
  const std::vector<unsigned char> rgb = cvc::colormap_rgb(values, kind, lo, hi);
  for (int j = 0; j < c.h; ++j)
    for (int i = 0; i < c.w; ++i) {
      const std::size_t v = static_cast<std::size_t>(c.y - y + j) * gw + (c.x - x + i);
      std::uint8_t *px = pixels_->data() + (static_cast<std::size_t>(c.y + j) * w_ + c.x + i) * 4;
      px[0] = rgb[3 * v];
      px[1] = rgb[3 * v + 1];
      px[2] = rgb[3 * v + 2];
      px[3] = std::isfinite(values[v]) ? alpha : 0;
    }
  marked_.push_back(c);
  return true;
}

std::uint32_t raster_stream::publish(double pts_seconds) {
  if (marked_.empty())
    return 0;
  // The first, each key_every-th, and the first since a subscriber joined (it has no picture yet).
  const std::size_t subs = stream_->channel().subscriber_count();
  const bool key =
      number_ == 0 || (key_every_ > 0 && (number_ + 1) % key_every_ == 0) || subs > subscribers_;
  subscribers_ = subs;
  return send(key, pts_seconds);
}

std::uint32_t raster_stream::publish_key(double pts_seconds) {
  subscribers_ = stream_->channel().subscriber_count();
  return send(true, pts_seconds);
}

std::uint32_t raster_stream::send(bool key, double pts) {
  const std::uint32_t n = number_ + 1;
  auto bytes = std::make_shared<std::vector<std::uint8_t>>(
      encode_raster_frame(pixels_->data(), w_, h_, n, key, marked_));
  marked_.clear();
  number_ = n;
  bytes_ += bytes->size();
  write_mirror(*bytes, key);
  const std::uint8_t *data = bytes->data();
  const std::size_t size = bytes->size();
  stream_->channel().publish_external(data, size, pts, std::shared_ptr<void>(std::move(bytes)));
  stream_->post_seq(stream_->channel().last_seq());
  return n;
}

void raster_stream::write_mirror(const std::vector<std::uint8_t> &frame, bool key) {
  const std::string bytes(frame.begin(), frame.end());
  const std::string number = std::to_string(number_);
  for (const std::string &path : mirrors_) {
    if (key) {
      cvc::state &k = write_node(ctx_, child(path, "key"));
      k.data(boost::any(bytes));
      k.value(number);
    }
    cvc::state &slot =
        write_node(ctx_, child(child(path, "ring"), std::to_string(number_ % kRasterRing)));
    slot.data(boost::any(bytes));
    slot.value(number);
    // Last: a client that sees the number holds the frames up to it.
    write_node(ctx_, child(path, "frame")).value(number);
  }
}

// --- the mirror, followed ------------------------------------------------------------------------

bool raster_mirror_present(app &ctx, const std::string &path) {
  return !path.empty() && text_at(ctx, child(path, "codec")) == kRasterCodec &&
         number_at(ctx, child(path, "w")) > 0 && number_at(ctx, child(path, "h")) > 0;
}

int raster_catch_up(app &ctx, const std::string &path, raster_canvas &canvas,
                    const std::function<void(const std::string &bytes)> &applied) {
  const std::uint32_t latest = number_at(ctx, child(path, "frame"));
  const std::uint32_t have = canvas.number();
  if (latest == 0 || (canvas.synced() && latest == have))
    return 0;
  int taken = 0;
  const auto take = [&](const std::string &bytes) {
    raster_frame_view f;
    if (bytes.empty() ||
        !decode_raster_frame(reinterpret_cast<const std::uint8_t *>(bytes.data()), bytes.size(),
                             f) ||
        !canvas.apply(f).applied)
      return false;
    ++taken;
    if (applied)
      applied(bytes);
    return true;
  };
  // Not started, gone back (a producer started again), or too far behind for the ring: from the
  // last key frame, unless it is no newer than the canvas, which it would only take back.
  const std::uint32_t key = number_at(ctx, child(path, "key"));
  if (!canvas.synced() || latest < have ||
      latest - have > static_cast<std::uint32_t>(kRasterRing - 1))
    if (!canvas.synced() || latest < have || key > have)
      take(bytes_at(ctx, child(path, "key")));
  // Then each frame after it, in order, while the ring still holds it.
  for (std::uint32_t n = canvas.number() + 1; canvas.synced() && n <= latest; ++n) {
    const std::string slot = child(child(path, "ring"), std::to_string(n % kRasterRing));
    if (number_at(ctx, slot) != n || !take(bytes_at(ctx, slot)))
      break;
  }
  return taken;
}

namespace {

// The app's followers, by mirror path, held weakly: the last consumer to let go stops one.
struct raster_followers {
  static constexpr const char *app_slot = "cvc.ariadne.stream.raster-followers";
  std::mutex m;
  std::map<std::string, std::weak_ptr<raster_follower>> by_path;
};

} // namespace

std::shared_ptr<raster_follower> raster_follower::follow(app &ctx, const std::string &path) {
  raster_followers &all = ctx.slot<raster_followers>();
  std::lock_guard<std::mutex> lock(all.m);
  if (std::shared_ptr<raster_follower> f = all.by_path[path].lock())
    return f;
  auto f = std::make_shared<raster_follower>(ctx, path);
  all.by_path[path] = f;
  return f;
}

raster_follower::raster_follower(app &ctx, std::string path) : ctx_(ctx), path_(std::move(path)) {}

raster_follower::~raster_follower() {
  if (stream_)
    stream_->close();
}

std::string raster_follower::token() const { return stream_ ? stream_->token() : std::string(); }

void raster_follower::send(std::shared_ptr<std::vector<std::uint8_t>> bytes) {
  const std::uint8_t *data = bytes->data();
  const std::size_t size = bytes->size();
  stream_->channel().publish_external(data, size, 0.0, std::shared_ptr<void>(std::move(bytes)));
}

int raster_follower::poll() {
  if (!raster_mirror_present(ctx_, path_))
    return 0;
  const int w = static_cast<int>(number_at(ctx_, child(path_, "w")));
  const int h = static_cast<int>(number_at(ctx_, child(path_, "h")));
  if (!stream_ || w != w_ || h != h_) {
    // A stream of its own, named after the mirror's path, at the mirror's size.
    if (stream_)
      stream_->close();
    stream_params sp;
    sp.id = "mirror_" + sanitized(path_);
    sp.format.kind = frame_kind::video_raw;
    sp.format.codec = kRasterCodec;
    sp.format.w = w;
    sp.format.h = h;
    sp.format.stride = w * 4;
    sp.format.bytes = kHeader;
    sp.expected_subscribers = 4;
    sp.subscriber_depth = 8;
    stream_ = stream::open(ctx_, sp);
    if (!stream_)
      return 0;
    w_ = w;
    h_ = h;
    canvas_ = raster_canvas{};
    subscribers_ = 0;
  }
  int published = 0;
  // A subscriber that joined starts from the picture as it is: a key frame of the canvas.
  const std::size_t subs = stream_->channel().subscriber_count();
  if (subs > subscribers_ && canvas_.synced()) {
    send(std::make_shared<std::vector<std::uint8_t>>(
        encode_raster_frame(canvas_.image().data(), w_, h_, canvas_.number(), true, {})));
    ++published;
  }
  subscribers_ = subs;
  published += raster_catch_up(ctx_, path_, canvas_, [this](const std::string &bytes) {
    // What the canvas took goes out as it came.
    raster_frame_view f;
    if (decode_raster_frame(reinterpret_cast<const std::uint8_t *>(bytes.data()), bytes.size(),
                            f) &&
        f.w == w_ && f.h == h_)
      send(std::make_shared<std::vector<std::uint8_t>>(bytes.begin(), bytes.end()));
  });
  return published;
}

} // namespace stream
} // namespace ariadne
} // namespace cvc
