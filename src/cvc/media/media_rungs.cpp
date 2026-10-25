/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <chrono>
#include <cvc/media/media_rungs.h>
#include <mutex>
#include <string>
#include <unordered_map>

#ifdef CVC_HAVE_TURBOJPEG
#include <turbojpeg.h>
#endif

namespace cvc {

namespace {

constexpr const char *kRungNames[kVideoRungs] = {"full", "half", "quarter", "thumb", "off"};

// The largest JPEG the relay decodes (4096 x 4096). A JPEG's header names its own size, and a frame
// from any client claiming 65535 x 65535 would otherwise allocate gigabytes on a reader thread.
constexpr long long kMaxJpegPixels = 4096LL * 4096LL;

double now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Only a camera in MJPEG is shaped: its frames stand alone, so any of them can be re-encoded or
// skipped. The mic and any other codec go as sent.
bool shapeable(const media_frame &f) {
  return f.stream_id == "camera" && (f.codec == "mjpeg" || f.codec == "jpeg");
}

#ifdef CVC_HAVE_TURBOJPEG
// One TurboJPEG handle per thread per direction (tj3Init is not free); publishers' reader threads
// each keep their own. A handle a call failed on is replaced: a decompressor that stopped partway
// through a bad header refuses every image after it ("two SOI markers"), so one malformed frame
// would otherwise end downscaling on that thread for good.
class tj_handle {
public:
  explicit tj_handle(int init) : _init(init) {}
  ~tj_handle() { reset(); }
  tj_handle(const tj_handle &) = delete;
  tj_handle &operator=(const tj_handle &) = delete;
  tjhandle get() {
    if (!_h)
      _h = tj3Init(_init);
    return _h;
  }
  void reset() {
    if (_h)
      tj3Destroy(_h);
    _h = nullptr;
  }

private:
  int _init;
  tjhandle _h = nullptr;
};
tj_handle &decompressor() {
  thread_local tj_handle h(TJINIT_DECOMPRESS);
  return h;
}
tj_handle &compressor() {
  thread_local tj_handle h(TJINIT_COMPRESS);
  return h;
}
#endif

} // namespace

const char *video_rung_name(video_rung r) noexcept {
  const int i = static_cast<int>(r);
  return i >= 0 && i < kVideoRungs ? kRungNames[i] : "full";
}

bool parse_video_rung(const std::string &name, video_rung &out) noexcept {
  for (int i = 0; i < kVideoRungs; ++i)
    if (name == kRungNames[i]) {
      out = static_cast<video_rung>(i);
      return true;
    }
  return false;
}

bool jpeg_transcode_available() noexcept {
#ifdef CVC_HAVE_TURBOJPEG
  return true;
#else
  return false;
#endif
}

std::vector<std::uint8_t> jpeg_downscale(const std::uint8_t *jpeg, std::size_t size, int denom,
                                         int quality, int &out_w, int &out_h) {
#ifdef CVC_HAVE_TURBOJPEG
  if (!jpeg || size == 0 || (denom != 1 && denom != 2 && denom != 4 && denom != 8))
    return {};
  tj_handle &dh = decompressor();
  tj_handle &ch = compressor();
  tjhandle d = dh.get();
  tjhandle c = ch.get();
  if (!d || !c)
    return {};
  const auto bad_input = [&dh]() {
    dh.reset();
    return std::vector<std::uint8_t>();
  };
  if (tj3DecompressHeader(d, jpeg, size) != 0)
    return bad_input();
  const long long src_w = tj3Get(d, TJPARAM_JPEGWIDTH), src_h = tj3Get(d, TJPARAM_JPEGHEIGHT);
  if (src_w <= 0 || src_h <= 0 || src_w * src_h > kMaxJpegPixels)
    return bad_input();
  const tjscalingfactor scale{1, denom};
  if (tj3SetScalingFactor(d, scale) != 0)
    return bad_input();
  const int w = TJSCALED(tj3Get(d, TJPARAM_JPEGWIDTH), scale);
  const int h = TJSCALED(tj3Get(d, TJPARAM_JPEGHEIGHT), scale);
  if (w <= 0 || h <= 0)
    return bad_input();
  std::vector<std::uint8_t> rgb(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u);
  if (tj3Decompress8(d, jpeg, size, rgb.data(), /*pitch=*/0, TJPF_RGB) != 0)
    return bad_input();
  tj3Set(c, TJPARAM_NOREALLOC, 0); // let it size the output
  tj3Set(c, TJPARAM_QUALITY, quality < 1 ? 1 : (quality > 100 ? 100 : quality));
  tj3Set(c, TJPARAM_SUBSAMP, TJSAMP_420);
  unsigned char *out = nullptr;
  std::size_t out_size = 0;
  if (tj3Compress8(c, rgb.data(), w, /*pitch=*/0, h, TJPF_RGB, &out, &out_size) != 0) {
    tj3Free(out);
    ch.reset();
    return {};
  }
  std::vector<std::uint8_t> result(out, out + out_size);
  tj3Free(out);
  out_w = w;
  out_h = h;
  return result;
#else
  (void)jpeg;
  (void)size;
  (void)denom;
  (void)quality;
  (void)out_w;
  (void)out_h;
  return {};
#endif
}

std::vector<std::uint8_t> jpeg_encode_rgb(const std::uint8_t *rgb, int w, int h, int quality) {
#ifdef CVC_HAVE_TURBOJPEG
  tj_handle &ch = compressor();
  tjhandle c = ch.get();
  if (!c || !rgb || w <= 0 || h <= 0)
    return {};
  tj3Set(c, TJPARAM_NOREALLOC, 0);
  tj3Set(c, TJPARAM_QUALITY, quality < 1 ? 1 : (quality > 100 ? 100 : quality));
  tj3Set(c, TJPARAM_SUBSAMP, TJSAMP_420);
  unsigned char *out = nullptr;
  std::size_t out_size = 0;
  if (tj3Compress8(c, rgb, w, /*pitch=*/0, h, TJPF_RGB, &out, &out_size) != 0) {
    tj3Free(out);
    ch.reset();
    return {};
  }
  std::vector<std::uint8_t> result(out, out + out_size);
  tj3Free(out);
  return result;
#else
  (void)rgb;
  (void)w;
  (void)h;
  (void)quality;
  return {};
#endif
}

struct media_rung_shaper::impl {
  config cfg;
  mutable std::mutex mu;
  std::unordered_map<std::string, video_rung> rungs; // viewer -> rung
  // The variant last made per (publisher, stream, rung), with the frame it was made from; a
  // variant serves every viewer on that rung, across this relay's transports.
  struct made {
    std::int64_t seq = -1;
    std::shared_ptr<const media_frame> variant; // null: transcoding failed, send as is
  };
  std::unordered_map<std::string, made> variants;
  std::unordered_map<std::string, double> last_thumb; // viewer + publisher -> wall s
  std::atomic<std::uint64_t> transcoded{0}, shaped{0}, saved{0};

  static std::string key(const std::string &a, const std::string &b) {
    std::string k = a;
    k.push_back('\0');
    k += b;
    return k;
  }

  std::shared_ptr<const media_frame> variant(const media_frame &f, video_rung r) {
    const std::string k = key(key(f.origin_node_id, f.stream_id), video_rung_name(r));
    {
      std::lock_guard<std::mutex> lk(mu);
      const auto it = variants.find(k);
      if (it != variants.end() && it->second.seq == f.seq)
        return it->second.variant;
    }
    // Made outside the lock: another publisher's frame need not wait for this one.
    const int denom = r == video_rung::half ? 2 : 4;
    const int quality = r == video_rung::half ? cfg.half_quality : cfg.quarter_quality;
    int w = 0, h = 0;
    std::vector<std::uint8_t> bytes =
        jpeg_downscale(f.payload.data(), f.payload.size(), denom, quality, w, h);
    std::shared_ptr<const media_frame> v;
    if (!bytes.empty() && bytes.size() < f.payload.size()) {
      auto m = std::make_shared<media_frame>();
      m->origin_node_id = f.origin_node_id;
      m->stream_id = f.stream_id;
      m->codec = f.codec;
      m->width = static_cast<std::uint32_t>(w);
      m->height = static_cast<std::uint32_t>(h);
      m->seq = f.seq;
      m->pts_seconds = f.pts_seconds;
      m->payload = std::move(bytes);
      v = std::move(m);
      transcoded.fetch_add(1, std::memory_order_relaxed);
    }
    std::lock_guard<std::mutex> lk(mu);
    if (variants.size() > 512) // publishers come and go; a stale entry is one small frame
      variants.clear();
    variants[k] = made{f.seq, v};
    return v;
  }
};

media_rung_shaper::media_rung_shaper(config c) : _impl(std::make_unique<impl>()) { _impl->cfg = c; }
media_rung_shaper::media_rung_shaper() : media_rung_shaper(config()) {}
media_rung_shaper::~media_rung_shaper() = default;

void media_rung_shaper::set_rung(const std::string &viewer, video_rung r) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->rungs[viewer] = r;
}

video_rung media_rung_shaper::rung(const std::string &viewer) const {
  std::lock_guard<std::mutex> lk(_impl->mu);
  const auto it = _impl->rungs.find(viewer);
  return it == _impl->rungs.end() ? video_rung::full : it->second;
}

void media_rung_shaper::forget_viewer(const std::string &viewer) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->rungs.erase(viewer);
  const std::string prefix = viewer + std::string(1, '\0');
  for (auto it = _impl->last_thumb.begin(); it != _impl->last_thumb.end();)
    it = it->first.compare(0, prefix.size(), prefix) == 0 ? _impl->last_thumb.erase(it)
                                                          : std::next(it);
}

void media_rung_shaper::forget_publisher(const std::string &publisher) {
  std::lock_guard<std::mutex> lk(_impl->mu);
  const std::string prefix = publisher + std::string(1, '\0');
  for (auto it = _impl->variants.begin(); it != _impl->variants.end();)
    it = it->first.compare(0, prefix.size(), prefix) == 0 ? _impl->variants.erase(it)
                                                          : std::next(it);
}

media_shape media_rung_shaper::shape(const media_frame &f, const std::string &viewer) {
  media_shape s;
  if (!shapeable(f))
    return s;
  const video_rung r = rung(viewer);
  switch (r) {
  case video_rung::full:
    return s;
  case video_rung::off:
    s.drop = true;
    return s;
  case video_rung::thumb: {
    // A frame about every 1/thumb_fps per viewer and publisher. A quarter of the interval early
    // still goes: the AOI tier may already pace this camera at the same rate on its own schedule,
    // and those frames reach here with a frame period of jitter, half of them a little early.
    const double now = now_s();
    const double gap = _impl->cfg.thumb_fps > 0.0 ? 1.0 / _impl->cfg.thumb_fps : 0.0;
    std::lock_guard<std::mutex> lk(_impl->mu);
    double &last = _impl->last_thumb[impl::key(viewer, f.origin_node_id)];
    if (last > 0.0 && now - last < 0.75 * gap) {
      s.drop = true;
      return s;
    }
    last = now;
    break;
  }
  default:
    break;
  }
  s.variant = _impl->variant(f, r == video_rung::half ? video_rung::half : video_rung::quarter);
  if (s.variant) {
    _impl->shaped.fetch_add(1, std::memory_order_relaxed);
    _impl->saved.fetch_add(f.payload.size() - s.variant->payload.size(), std::memory_order_relaxed);
  }
  return s;
}

std::uint64_t media_rung_shaper::total_transcoded() const noexcept {
  return _impl->transcoded.load(std::memory_order_relaxed);
}
std::uint64_t media_rung_shaper::total_shaped() const noexcept {
  return _impl->shaped.load(std::memory_order_relaxed);
}
std::uint64_t media_rung_shaper::total_bytes_saved() const noexcept {
  return _impl->saved.load(std::memory_order_relaxed);
}

} // namespace cvc
