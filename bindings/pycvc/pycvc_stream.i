/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// pycvc_stream — Python bindings for the cvc::stream real-time transport
// (roadmap STATE_BINARY_STREAMING.md Phase 2). %included from pycvc.i AFTER the
// ArrayView out-typemap + the pycvc_owner capsule dtor (pycvc.i) and after numpy
// is imported, so StreamFrame.numpy() reuses that zero-copy machinery.
//
// The raw cvc::stream classes (unique_ptr<stream>, shared_ptr<subscription>,
// shared_ptr<const frame>, the pool-lease publish) are not SWIG-friendly, so this
// exposes thin VALUE-HOLDER shims that carry the shared_ptrs internally — SWIG
// copies the shim (= a refcount bump), never the C++ object. The Python surface:
//
//   s   = cvc.stream_open(app, "cam0", "rgba8", 1920, 1080)
//   sub = s.subscribe("latest", 3)          # or "ring"
//   f   = sub.latest()                       # StreamFrame; f.valid() False if none
//   arr = f.numpy()                          # zero-copy (H,W,C) uint8 view, read-only
//   seq = s.publish(ndarray_uint8, pts)      # zero-copy producer (aliases the ndarray)

%{
#include <cvc/core/app.h>
#include <cvc/stream/frame.h>
#include <cvc/stream/stream.h>
#include <cvc/stream/stream_channel.h>
#include "pycvc_buffer.h"
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
%}

// Keepalive: the C++ stream holds a RAW cvc::app& (writes the descriptor,
// uninstalls the registry token, posts lifecycle on teardown), so the Python
// Stream must keep the app alive or ~stream can touch a GC'd app's mutexes
// (boost::lock_error). Stash the app on the returned Stream — the same borrowed-
// app keepalive idiom as GeometryNode._pycvc_app / raycaster._pycvc_app. stream_open
// has a single signature, so its proxy exposes `app` as a named parameter.
%pythonappend pycvc::stream_open %{
    val._pycvc_app = app
%}

%inline %{
namespace pycvc {

// Channels implied by a stream codec (for numpy shape + producer size check).
static int stream_codec_channels(const std::string &codec) {
  if (codec == "rgba8")
    return 4;
  if (codec == "rgb8")
    return 3;
  if (codec == "graya8")
    return 2;
  if (codec == "gray8")
    return 1;
  return 0; // unknown: numpy()/publish will reject
}

// Owns a borrowed ndarray for a producer-published frame's lifetime, DECREF'ing
// it (GIL-safe — the last frame_ptr may drop on a non-Python thread) when the
// frame is finally released.
struct PyArrayKeepalive {
  PyObject *arr = nullptr;
  explicit PyArrayKeepalive(PyObject *a) : arr(a) { Py_XINCREF(arr); }
  ~PyArrayKeepalive() {
    if (!arr)
      return;
    PyGILState_STATE g = PyGILState_Ensure();
    Py_DECREF(arr);
    PyGILState_Release(g);
  }
  PyArrayKeepalive(const PyArrayKeepalive &) = delete;
  PyArrayKeepalive &operator=(const PyArrayKeepalive &) = delete;
};

// A single frame. Immutable; numpy() hands back a READ-ONLY zero-copy view whose
// numpy base pins this exact frame (and so its slab / producer ndarray) alive.
struct StreamFrame {
  cvc::stream::frame_ptr fp;

  bool valid() const { return static_cast<bool>(fp); }
  long seq() const { return fp ? static_cast<long>(fp->seq) : -1; }
  double pts() const { return fp ? fp->pts_seconds : 0.0; }
  int width() const { return fp ? fp->format.w : 0; }
  int height() const { return fp ? fp->format.h : 0; }
  std::string codec() const { return fp ? fp->format.codec : std::string(); }

  pycvc::ArrayView numpy() const {
    pycvc::ArrayView v;
    if (!fp)
      return v; // empty -> the typemap yields an empty array
    const int ch = stream_codec_channels(fp->format.codec);
    if (ch == 0)
      throw std::runtime_error("StreamFrame.numpy: unsupported codec '" + fp->format.codec + "'");
    v.dtype = pycvc::DType::UInt8;
    v.writable = false; // frames are immutable
    v.shape = {static_cast<long>(fp->format.h), static_cast<long>(fp->format.w),
               static_cast<long>(ch)};
    v.data = fp->data;
    // Pin the frame (its keepalive -> pool slab or producer ndarray) for the
    // view's life via an aliasing shared_ptr sharing fp's control block.
    v.owner = std::shared_ptr<void>(fp, const_cast<void *>(static_cast<const void *>(fp.get())));
    return v;
  }
};

// A consumer's subscription.
struct StreamSub {
  std::shared_ptr<cvc::stream::subscription> sub;

  StreamFrame latest() const {
    StreamFrame f;
    if (sub)
      f.fp = sub->latest();
    return f;
  }
  StreamFrame try_pop() {
    StreamFrame f;
    cvc::stream::frame_ptr out;
    if (sub && sub->try_pop(out))
      f.fp = out;
    return f;
  }
  StreamFrame pop() {
    StreamFrame f;
    cvc::stream::frame_ptr out;
    bool ok = false;
    if (sub) {
      // pop() blocks until a frame or close(); release the GIL so other Python
      // threads (incl. the producer) run while we wait.
      Py_BEGIN_ALLOW_THREADS;
      ok = sub->pop(out);
      Py_END_ALLOW_THREADS;
    }
    if (ok)
      f.fp = out;
    return f;
  }
  unsigned long long dropped() const { return sub ? sub->dropped() : 0ULL; }
  void close() {
    if (sub)
      sub->close();
  }
};

// A live stream (owns it) with a consumer + a zero-copy producer surface.
struct Stream {
  std::shared_ptr<cvc::stream::stream> s;

  StreamSub subscribe(const std::string &mode = "latest", std::size_t depth = 3) {
    if (!s)
      throw std::runtime_error("Stream.subscribe: closed stream");
    cvc::stream::deliver_mode m = (mode == "ring") ? cvc::stream::deliver_mode::ring
                                                    : cvc::stream::deliver_mode::latest;
    auto sub = s->channel().subscribe(m, depth);
    if (!sub)
      throw std::runtime_error(
          "Stream.subscribe: refused (would exceed the frame-pool sizing invariant)");
    StreamSub out;
    out.sub = std::move(sub);
    return out;
  }

  // Zero-copy producer: ALIAS the ndarray's buffer into a published frame (no
  // copy). The array must be C-contiguous uint8 with exactly the stream's frame
  // byte size; a wrong dtype/shape/layout is REJECTED, never coerced (a coerce
  // would silently copy and defeat zero-copy). The array is kept alive until the
  // last subscriber drops the frame.
  long publish(PyObject *arr, double pts = 0.0) {
    if (!s)
      throw std::runtime_error("Stream.publish: closed stream");
    if (!arr || !PyArray_Check(arr))
      throw std::invalid_argument("Stream.publish: expected a numpy ndarray");
    PyArrayObject *a = reinterpret_cast<PyArrayObject *>(arr);
    if (PyArray_TYPE(a) != NPY_UINT8)
      throw std::invalid_argument("Stream.publish: array must be uint8 (never coerced)");
    if (!PyArray_ISCARRAY(a))
      throw std::invalid_argument("Stream.publish: array must be C-contiguous");
    const std::size_t need = s->channel().format().frame_bytes();
    if (static_cast<std::size_t>(PyArray_NBYTES(a)) != need)
      throw std::invalid_argument(
          "Stream.publish: array byte size does not match the stream frame size");
    auto keep = std::make_shared<PyArrayKeepalive>(arr); // INCREF now (holding the GIL)
    const std::uint8_t *data = static_cast<const std::uint8_t *>(PyArray_DATA(a));
    return static_cast<long>(
        s->channel().publish_external(data, need, pts, std::shared_ptr<void>(std::move(keep))));
  }

  std::string id() const { return s ? s->id() : std::string(); }
  void update_stats() {
    if (s)
      s->update_stats();
  }
  void close() {
    if (s)
      s->close();
  }
};

// Open a video stream. `stride` is set dense (w*channels) so the stream has a
// non-zero frame size. Throws on a bad codec or if open() fails.
static Stream stream_open(cvc::app &app, const std::string &id, const std::string &codec, int w,
                          int h, double heartbeat_hz = 10.0, std::size_t subscriber_depth = 3,
                          std::size_t expected_subscribers = 1) {
  const int ch = stream_codec_channels(codec);
  if (ch == 0)
    throw std::invalid_argument("stream_open: unsupported codec '" + codec + "'");
  if (w <= 0 || h <= 0)
    throw std::invalid_argument("stream_open: non-positive dimensions");
  cvc::stream::stream_params p;
  p.id = id;
  p.format.kind = cvc::stream::frame_kind::video_raw;
  p.format.codec = codec;
  p.format.w = w;
  p.format.h = h;
  p.format.stride = w * ch; // dense
  p.heartbeat_hz = heartbeat_hz;
  p.subscriber_depth = subscriber_depth;
  p.expected_subscribers = expected_subscribers;
  auto up = cvc::stream::stream::open(app, p);
  if (!up)
    throw std::runtime_error("stream_open: open failed (bad id/format or token in use)");
  Stream out;
  out.s = std::shared_ptr<cvc::stream::stream>(std::move(up));
  return out;
}

} // namespace pycvc
%}
