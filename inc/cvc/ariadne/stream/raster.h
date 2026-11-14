/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_RASTER_H__
#define __CVC_ARIADNE_STREAM_RASTER_H__

// A raster as a stream (docs/ARIADNE.md §5.4, "Live textures"): an RGBA8 picture a producer
// repaints a part at a time, sent as the parts that changed.
//
// - Each frame of a raster stream (codec "rgba8.patch") is the producer's frame number, the
//   rectangles that changed since the frame before, and their pixels. A key frame carries the
//   whole raster instead, so a consumer that joined late, or missed a frame, starts again from it.
//   The frame is self-describing, so it crosses any transport as it is.
// - raster_stream is the producer: a raster in memory, painted (fill, an image, a float grid
//   through a colour map) and published as the rectangles painted since the last publish. It can
//   also write each frame under a state path (`mirror`), so a published namespace carries it to a
//   relay's clients.
// - raster_follower turns such a mirror back into a local stream, on the client.
// - raster_canvas is the consumer side: the raster as the frames build it, and what each frame
//   changed, which is what a texture uploads.

#include <cstddef>
#include <cstdint>
#include <cvc/image/image.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
namespace stream {

class stream;

// The codec of a raster stream's frames.
constexpr const char *kRasterCodec = "rgba8.patch";

// The pixels [x, x + w) x [y, y + h), row 0 at the top.
struct raster_rect {
  int x = 0, y = 0, w = 0, h = 0;
  bool empty() const noexcept { return w <= 0 || h <= 0; }
};

// `r` clamped to a w x h raster (empty when it misses it).
raster_rect clamp_rect(const raster_rect &r, int w, int h) noexcept;

// One rectangle's pixels in a decoded frame: rect.w * rect.h RGBA8 pixels, rows packed.
struct raster_patch {
  raster_rect rect;
  const std::uint8_t *pixels = nullptr; // into the frame's bytes
};

// A decoded frame. Its patches point into the bytes it was decoded from.
struct raster_frame_view {
  bool key = false;                 // the patches cover the whole raster
  std::uint32_t number = 0;         // the producer's frame number: 1, 2, 3, ...
  int w = 0, h = 0;                 // the raster's size
  std::vector<raster_rect> changed; // what changed since frame number - 1
  std::vector<raster_patch> patches;
};

// One frame of the w x h RGBA8 raster `pixels` (top row first, w * 4 bytes a row): the pixels of
// `changed` (each clamped; the empty ones dropped), or, for a key frame, of the whole raster with
// `changed` kept as what changed.
std::vector<std::uint8_t> encode_raster_frame(const std::uint8_t *pixels, int w, int h,
                                              std::uint32_t number, bool key,
                                              const std::vector<raster_rect> &changed);

// The frame in `data`, or false when it is not a well-formed raster frame (`out` is then cleared).
bool decode_raster_frame(const std::uint8_t *data, std::size_t size, raster_frame_view &out);

// The consumer's raster: what the frames, applied in order, have drawn.
class raster_canvas {
public:
  // What one frame changed in the canvas.
  struct update {
    bool applied = false; // false: skipped (a frame after a gap, waiting for a key frame)
    bool resized = false; // the canvas was made again at the frame's size (all of it is new)
    bool all = false;     // every pixel may have changed (a key frame after a gap, or the first)
    std::vector<raster_rect> rects; // otherwise: these did
  };

  // Apply `f`. In order (its number one past the last), its patches are copied in and its changed
  // rectangles reported; a key frame in order copies only those rectangles from its whole raster.
  // Out of order, a key frame starts the canvas again and a delta frame is skipped until one does.
  update apply(const raster_frame_view &f);

  const cvc::image &image() const noexcept { return image_; } // RGBA8; empty before a frame
  // Its pixels, written in place: a texture that aliases the image's storage sees them.
  std::uint8_t *pixels() noexcept;
  std::uint32_t number() const noexcept { return number_; }
  bool synced() const noexcept { return synced_; }
  std::uint64_t skipped() const noexcept { return skipped_; }

private:
  cvc::image image_;
  std::uint32_t number_ = 0;
  bool synced_ = false;
  std::uint64_t skipped_ = 0;
};

struct raster_params {
  std::string id;        // the stream's id (one path segment)
  std::string root_path; // its scope, as stream_params::root_path
  int w = 0, h = 0;      // the raster's size, at most 8192 a side
  // A key frame every this many frames (the first is always one); 0: only the first. A consumer
  // that missed a frame waits for the next key frame, so this bounds how long it shows a stale
  // part.
  int key_every = 30;
  std::size_t expected_subscribers = 4; // as stream_params
  std::size_t subscriber_depth = 8;     // each subscriber's ring of frames
  // Each frame is kept in state at the stream's token: `<token>.w`, `.h`, `.codec`, the last key
  // frame in `.key`, the last kRasterRing frames in `.ring.<n % ring>`, and their number in
  // `.frame`, written last. A consumer that subscribes late catches up there. `mirror` is an
  // absolute state path to keep them under as well (empty: none): inside a server document's
  // published namespace, the relay carries it to the clients that subscribe (raster_follower).
  std::string mirror;
};

// Frames a mirror keeps, so a client that reads a few at once misses none.
constexpr int kRasterRing = 8;

// A producer: a raster in memory, painted and published a part at a time. One thread (the owner
// of its stream) paints and publishes.
class raster_stream {
public:
  // Null (and `error` says why) for a bad size or id, or a stream already live on that token.
  static std::unique_ptr<raster_stream> open(app &ctx, const raster_params &p,
                                             std::string *error = nullptr);
  ~raster_stream();
  raster_stream(const raster_stream &) = delete;
  raster_stream &operator=(const raster_stream &) = delete;

  int width() const noexcept { return w_; }
  int height() const noexcept { return h_; }
  const std::string &token() const;
  const std::string &mirror() const noexcept { return mirror_; }
  ariadne::stream::stream &stream() noexcept { return *stream_; }
  const ariadne::stream::stream &stream() const noexcept { return *stream_; }

  // The raster: RGBA8, top row first, width() * 4 bytes a row. After writing it directly, mark()
  // what was written.
  std::uint8_t *pixels() noexcept { return pixels_->data(); }
  const std::uint8_t *pixels() const noexcept { return pixels_->data(); }
  // The same buffer, shared: a view (a numpy array) may outlive the raster without dangling.
  std::shared_ptr<std::vector<std::uint8_t>> pixel_buffer() const noexcept { return pixels_; }
  // Add `r` (clamped) to what the next publish() sends.
  void mark(const raster_rect &r);
  // Paint `r` one colour, and mark it.
  void fill(const raster_rect &r, const std::uint8_t rgba[4]);
  // Paint `img` (converted to RGBA8) with its top-left at (x, y), and mark what it covers.
  void paint(const cvc::image &img, int x, int y);
  // Paint a gw x gh grid of values, one pixel each, with its top-left at (x, y): each value
  // coloured through `colormap` (mesh_ops' names: viridis, magma, plasma, inferno, turbo, jet,
  // parula, gray) over [lo, hi], at opacity `alpha`; a NaN is transparent. False for an unknown
  // colour map or a grid that is not gw x gh. Marks what it covers.
  bool paint_grid(const std::vector<double> &values, int gw, int gh, int x, int y,
                  const std::string &colormap, double lo, double hi, std::uint8_t alpha = 255);

  // Send what was marked since the last publish as one frame (a key frame when it is due), and
  // write it to the mirror. Nothing marked: nothing sent. Returns the frame's number, or 0.
  std::uint32_t publish(double pts_seconds = 0.0);
  // Send a key frame now, marked or not.
  std::uint32_t publish_key(double pts_seconds = 0.0);

  std::uint32_t frames() const noexcept { return number_; }
  std::uint64_t bytes_sent() const noexcept { return bytes_; } // payload bytes, every frame

private:
  raster_stream(app &ctx);
  std::uint32_t send(bool key, double pts);
  void write_mirror(const std::vector<std::uint8_t> &frame, bool key);

  app &ctx_;
  std::unique_ptr<ariadne::stream::stream> stream_;
  int w_ = 0, h_ = 0, key_every_ = 0;
  std::shared_ptr<std::vector<std::uint8_t>> pixels_;
  std::vector<raster_rect> marked_;
  std::uint32_t number_ = 0;
  std::uint64_t bytes_ = 0;
  std::size_t subscribers_ = 0; // at the last publish: a new one gets a key frame next
  std::string mirror_;
  std::vector<std::string> mirrors_; // its token, then `mirror_`: where its frames are kept
};

// Whether the state at absolute path `path` (read through links) holds a raster mirror. Every
// raster stream keeps one at its token; raster_params::mirror names another.
bool raster_mirror_present(app &ctx, const std::string &path);

// Bring `canvas` up to the mirror at `path`: from its last key frame when the canvas has not
// started or is too far behind for the ring, then each frame after, in order. `applied` hears each
// frame the canvas took. Returns how many it took.
int raster_catch_up(app &ctx, const std::string &path, raster_canvas &canvas,
                    const std::function<void(const std::string &bytes)> &applied = {});

// A raster mirror (raster_params::mirror) as a local stream: each poll() reads the frames the
// mirror got since the last one, in order, into a canvas of its own, and publishes them on its
// stream. A subscriber that joins gets a key frame of the canvas first, so it starts from the
// picture as it is now. Too far behind for the mirror's ring, it starts again from the mirror's
// last key frame. One follower per mirror path and app (follow()), shared by its consumers; it
// stops with the last of them. Owner thread.
class raster_follower {
public:
  static std::shared_ptr<raster_follower> follow(app &ctx, const std::string &path);
  ~raster_follower();
  raster_follower(const raster_follower &) = delete;
  raster_follower &operator=(const raster_follower &) = delete;

  // Read and publish the mirror's new frames. Returns how many it published.
  int poll();
  const std::string &path() const noexcept { return path_; }
  // The local stream's token ("" until the mirror says its size).
  std::string token() const;
  // The frame its canvas has reached.
  std::uint32_t number() const noexcept { return canvas_.number(); }
  const raster_canvas &canvas() const noexcept { return canvas_; }

  // follow()'s use only.
  raster_follower(app &ctx, std::string path);

private:
  void send(std::shared_ptr<std::vector<std::uint8_t>> bytes);

  app &ctx_;
  std::string path_;
  std::unique_ptr<ariadne::stream::stream> stream_;
  int w_ = 0, h_ = 0;
  raster_canvas canvas_;
  std::size_t subscribers_ = 0;
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_RASTER_H__
