/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcMedia's FFmpeg + SDL3 player. See inc/cvc/media/player.h.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/media.h> // fetch_bytes: the host's network policy and credentials
#include <cvc/media/playback/audio_output.h>
#include <cvc/media/player.h>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if CVC_MEDIA_FFMPEG
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/channel_layout.h>
#include <libavutil/imgutils.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}
#endif

namespace cvc {
namespace media {

namespace am = cvc::ariadne::media;

bool have_ffmpeg_player() {
#if CVC_MEDIA_FFMPEG
  return true;
#else
  return false;
#endif
}

#if CVC_MEDIA_FFMPEG
namespace {

using clock_type = std::chrono::steady_clock;

constexpr int kOutRate = 48000;
constexpr int kOutChannels = 2;
constexpr double kAudioAhead = 0.25; // seconds of sound queued ahead of the device
constexpr std::size_t kFramesAhead = 6;

std::string av_error(int err) {
  char buf[AV_ERROR_MAX_STRING_SIZE] = {0};
  av_strerror(err, buf, sizeof(buf));
  return buf;
}

// The fetched bytes, read by FFmpeg through an AVIOContext.
struct MemReader {
  std::string bytes;
  std::size_t pos = 0;
  static int read(void *opaque, uint8_t *buf, int size) {
    auto *r = static_cast<MemReader *>(opaque);
    const std::size_t left = r->bytes.size() - r->pos;
    const std::size_t n = std::min<std::size_t>(left, static_cast<std::size_t>(size));
    if (n == 0)
      return AVERROR_EOF;
    std::memcpy(buf, r->bytes.data() + r->pos, n);
    r->pos += n;
    return static_cast<int>(n);
  }
  static int64_t seek(void *opaque, int64_t offset, int whence) {
    auto *r = static_cast<MemReader *>(opaque);
    if (whence & AVSEEK_SIZE)
      return static_cast<int64_t>(r->bytes.size());
    whence &= ~AVSEEK_FORCE;
    int64_t at = offset;
    if (whence == SEEK_CUR)
      at = static_cast<int64_t>(r->pos) + offset;
    else if (whence == SEEK_END)
      at = static_cast<int64_t>(r->bytes.size()) + offset;
    if (at < 0 || at > static_cast<int64_t>(r->bytes.size()))
      return -1;
    r->pos = static_cast<std::size_t>(at);
    return at;
  }
};

// What the decode thread and the frame thread share, under `mu`.
struct Shared {
  std::mutex mu;
  std::condition_variable cv;
  am::PlayerState state = am::PlayerState::Loading;
  std::string error;
  double duration = 0.0;
  bool want_play = false;
  bool muted = false;
  double seek_to = -1.0; // a seek asked for, not yet done
  bool quit = false;
  // Pictures decoded ahead, by time; the one on screen.
  struct Pic {
    cvc::image img;
    double pts = 0.0;
  };
  std::deque<Pic> pics;
  cvc::image shown;
  long shown_gen = 0;
  double shown_pts = -1.0;
  // The clock: `base_pts` at `base_time` while running; frozen at `base_pts` while not.
  bool clock_running = false;
  double base_pts = 0.0;
  clock_type::time_point base_time;
  // Sound: the decode thread asks for a device; the frame thread opens it (SDL wants that) and
  // hands it over; the frame thread closes it too, after the decode thread has stopped.
  bool need_sink = false;
  bool sink_tried = false;
  std::shared_ptr<playback::audio_output> sink;
  double audio_end_pts = -1.0; // the time just after the last sample handed to the sink

  double clock() const { // under mu
    if (!clock_running)
      return base_pts;
    if (sink && audio_end_pts >= 0.0) {
      const double queued =
          static_cast<double>(sink->queued_bytes()) / (sizeof(float) * kOutChannels * kOutRate);
      return std::max(0.0, audio_end_pts - queued);
    }
    return base_pts + std::chrono::duration<double>(clock_type::now() - base_time).count();
  }
  void run_clock(double at) { // under mu
    base_pts = at;
    base_time = clock_type::now();
    clock_running = true;
  }
  void stop_clock() { // under mu
    base_pts = clock();
    clock_running = false;
  }
};

// FFmpeg logs to stderr, which a terminal backend's full screen owns: quiet unless asked for
// (CVC_MEDIA_FFMPEG_LOG=1).
void quiet_ffmpeg() {
  static std::once_flag once;
  std::call_once(once, [] {
    const char *v = std::getenv("CVC_MEDIA_FFMPEG_LOG");
    av_log_set_level(v && *v && *v != '0' ? AV_LOG_WARNING : AV_LOG_QUIET);
  });
}

class FfmpegPlayer : public am::Player {
public:
  FfmpegPlayer(const am::PlayerSource &src, const PlayerOptions &opt) : src_(src), opt_(opt) {
    quiet_ffmpeg();
    sh_.muted = src.muted;
    worker_ = std::thread([this] { run(); });
  }
  ~FfmpegPlayer() override {
    {
      std::lock_guard<std::mutex> lock(sh_.mu);
      sh_.quit = true;
    }
    sh_.cv.notify_all();
    if (worker_.joinable())
      worker_.join();
    std::lock_guard<std::mutex> lock(sh_.mu);
    sh_.sink.reset(); // on this, the frame, thread, as SDL wants
  }

  void play() override {
    service();
    std::lock_guard<std::mutex> lock(sh_.mu);
    if (sh_.state == am::PlayerState::Failed || sh_.state == am::PlayerState::Loading) {
      sh_.want_play = sh_.state == am::PlayerState::Loading; // play once it has opened
      return;
    }
    if (sh_.state == am::PlayerState::Ended)
      sh_.seek_to = 0.0;
    sh_.want_play = true;
    sh_.cv.notify_all();
  }
  void pause() override {
    std::lock_guard<std::mutex> lock(sh_.mu);
    sh_.want_play = false;
    sh_.cv.notify_all();
  }
  void seek(double seconds) override {
    std::lock_guard<std::mutex> lock(sh_.mu);
    sh_.seek_to = std::max(0.0, sh_.duration > 0 ? std::min(seconds, sh_.duration) : seconds);
    sh_.cv.notify_all();
  }
  void set_muted(bool muted) override {
    std::lock_guard<std::mutex> lock(sh_.mu);
    sh_.muted = muted;
  }
  am::PlayerState state() const override {
    const_cast<FfmpegPlayer *>(this)->service();
    std::lock_guard<std::mutex> lock(sh_.mu);
    return sh_.state;
  }
  double position() const override {
    std::lock_guard<std::mutex> lock(sh_.mu);
    const double t = sh_.clock();
    return sh_.duration > 0 ? std::min(t, sh_.duration) : t;
  }
  double duration() const override {
    std::lock_guard<std::mutex> lock(sh_.mu);
    return sh_.duration;
  }
  std::string error() const override {
    std::lock_guard<std::mutex> lock(sh_.mu);
    return sh_.error;
  }
  bool frame(cvc::image &out, long &gen) override {
    service();
    std::lock_guard<std::mutex> lock(sh_.mu);
    // The newest decoded picture whose time has come.
    const double now = sh_.clock();
    bool moved = false;
    while (!sh_.pics.empty() && (sh_.pics.front().pts <= now + 0.01 || sh_.shown.empty())) {
      sh_.shown = std::move(sh_.pics.front().img);
      sh_.shown_pts = sh_.pics.front().pts;
      sh_.pics.pop_front();
      moved = true;
    }
    if (moved) {
      ++sh_.shown_gen;
      sh_.cv.notify_all(); // room for more ahead
    }
    if (sh_.shown.empty() || sh_.shown_gen == gen)
      return false;
    out = sh_.shown;
    gen = sh_.shown_gen;
    return true;
  }

private:
  // On the frame thread: open the sound device the decode thread asked for.
  void service() {
    std::unique_lock<std::mutex> lock(sh_.mu);
    if (!sh_.need_sink || sh_.sink_tried)
      return;
    sh_.sink_tried = true;
    lock.unlock();
    playback::audio_output_open o = playback::open_audio_output({kOutRate, kOutChannels});
    lock.lock();
    if (o.sink)
      sh_.sink = std::shared_ptr<playback::audio_output>(std::move(o.sink));
    sh_.cv.notify_all();
  }

  void fail(const std::string &why) {
    std::lock_guard<std::mutex> lock(sh_.mu);
    sh_.state = am::PlayerState::Failed;
    sh_.error = why;
    sh_.want_play = false;
    sh_.clock_running = false;
  }

  bool quitting() {
    std::lock_guard<std::mutex> lock(sh_.mu);
    return sh_.quit;
  }

  // The decode thread.
  void run() {
    const am::FetchResult fetched = am::fetch_bytes(src_.uri, src_.auth, src_.base);
    if (quitting())
      return;
    if (!fetched.ok) {
      fail(fetched.error.empty() ? std::string("could not fetch the media") : fetched.error);
      return;
    }
    reader_.bytes = fetched.bytes;
    if (!open()) {
      close();
      return;
    }
    loop();
    close();
  }

  bool open() {
    const int io_size = 64 * 1024;
    auto *io_buf = static_cast<unsigned char *>(av_malloc(io_size));
    io_ = avio_alloc_context(io_buf, io_size, 0, &reader_, &MemReader::read, nullptr,
                             &MemReader::seek);
    fmt_ = avformat_alloc_context();
    if (!io_ || !fmt_) {
      fail("out of memory");
      return false;
    }
    fmt_->pb = io_;
    fmt_->flags |= AVFMT_FLAG_CUSTOM_IO;
    int err = avformat_open_input(&fmt_, nullptr, nullptr, nullptr);
    if (err < 0) {
      fmt_ = nullptr; // freed by avformat_open_input on failure
      fail("not a media file this build can read (" + av_error(err) + ")");
      return false;
    }
    if ((err = avformat_find_stream_info(fmt_, nullptr)) < 0) {
      fail("could not read the media's streams (" + av_error(err) + ")");
      return false;
    }
    if (src_.video)
      vidx_ = av_find_best_stream(fmt_, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    // A silent picture (a decorative video) takes no sound: no audio stream, no device.
    aidx_ = src_.audio ? av_find_best_stream(fmt_, AVMEDIA_TYPE_AUDIO, -1, vidx_ >= 0 ? vidx_ : -1,
                                             nullptr, 0)
                       : -1;
    if (vidx_ < 0 && aidx_ < 0) {
      fail(src_.video ? "the file has no video or audio stream" : "the file has no audio stream");
      return false;
    }
    if (vidx_ >= 0 && !open_decoder(vidx_, vdec_))
      vidx_ = -1;
    if (aidx_ >= 0 && !open_decoder(aidx_, adec_))
      aidx_ = -1;
    if (vidx_ < 0 && aidx_ < 0) {
      fail("no decoder in this build for the file's streams");
      return false;
    }
    if (aidx_ >= 0) {
      AVChannelLayout out_layout;
      av_channel_layout_default(&out_layout, kOutChannels);
      err = swr_alloc_set_opts2(&swr_, &out_layout, AV_SAMPLE_FMT_FLT, kOutRate, &adec_->ch_layout,
                                adec_->sample_fmt, adec_->sample_rate, 0, nullptr);
      av_channel_layout_uninit(&out_layout);
      if (err < 0 || swr_init(swr_) < 0) {
        swr_free(&swr_);
        aidx_ = -1; // play it silent rather than not at all
      }
    }
    pkt_ = av_packet_alloc();
    frm_ = av_frame_alloc();
    double dur = fmt_->duration > 0 ? static_cast<double>(fmt_->duration) / AV_TIME_BASE : 0.0;
    std::unique_lock<std::mutex> lock(sh_.mu);
    sh_.duration = dur;
    if (aidx_ >= 0 && opt_.audio) {
      sh_.need_sink = true; // the frame thread opens it (service), at its next call
    }
    return true;
  }

  bool open_decoder(int idx, AVCodecContext *&ctx) {
    const AVCodecParameters *par = fmt_->streams[idx]->codecpar;
    const AVCodec *codec = avcodec_find_decoder(par->codec_id);
    if (!codec)
      return false;
    ctx = avcodec_alloc_context3(codec);
    if (!ctx || avcodec_parameters_to_context(ctx, par) < 0)
      return false;
    ctx->thread_count = 2;
    return avcodec_open2(ctx, codec, nullptr) >= 0;
  }

  void close() {
    av_frame_free(&frm_);
    av_packet_free(&pkt_);
    swr_free(&swr_);
    if (sws_)
      sws_freeContext(sws_);
    sws_ = nullptr;
    avcodec_free_context(&vdec_);
    avcodec_free_context(&adec_);
    if (fmt_)
      avformat_close_input(&fmt_);
    if (io_) {
      av_freep(&io_->buffer);
      avio_context_free(&io_);
    }
  }

  double pts_of(const AVFrame *f, int idx) const {
    const int64_t ts =
        f->best_effort_timestamp != AV_NOPTS_VALUE ? f->best_effort_timestamp : f->pts;
    if (ts == AV_NOPTS_VALUE)
      return -1.0;
    return static_cast<double>(ts) * av_q2d(fmt_->streams[idx]->time_base);
  }

  // A decoded video frame as RGBA, scaled to fit the widget's box (never up).
  cvc::image to_rgba(const AVFrame *f) {
    int w = f->width, h = f->height;
    if (src_.frame_w > 0 && src_.frame_h > 0) {
      const double k = std::min({1.0, double(src_.frame_w) / w, double(src_.frame_h) / h});
      w = std::max(2, static_cast<int>(w * k)) & ~1;
      h = std::max(2, static_cast<int>(h * k)) & ~1;
    }
    sws_ = sws_getCachedContext(sws_, f->width, f->height, static_cast<AVPixelFormat>(f->format), w,
                                h, AV_PIX_FMT_RGBA, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws_)
      return cvc::image();
    cvc::image out(w, h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
    uint8_t *dst[4] = {out.data(), nullptr, nullptr, nullptr};
    int stride[4] = {w * 4, 0, 0, 0};
    sws_scale(sws_, f->data, f->linesize, 0, f->height, dst, stride);
    return out;
  }

  // Sound for `f`, resampled, to the sink (silence when muted, so the clock still runs on it).
  void play_audio(const AVFrame *f, double pts) {
    std::shared_ptr<playback::audio_output> sink;
    bool muted = false;
    {
      std::lock_guard<std::mutex> lock(sh_.mu);
      sink = sh_.sink;
      muted = sh_.muted;
    }
    if (!sink || !swr_)
      return;
    const int max_out = swr_get_out_samples(swr_, f->nb_samples);
    if (max_out <= 0)
      return;
    std::vector<float> buf(static_cast<std::size_t>(max_out) * kOutChannels);
    uint8_t *out[1] = {reinterpret_cast<uint8_t *>(buf.data())};
    const int got = swr_convert(swr_, out, max_out, const_cast<const uint8_t **>(f->extended_data),
                                f->nb_samples);
    if (got <= 0)
      return;
    if (muted)
      std::fill(buf.begin(), buf.begin() + got * kOutChannels, 0.0f);
    sink->play(buf.data(), static_cast<std::size_t>(got) * kOutChannels * sizeof(float));
    std::lock_guard<std::mutex> lock(sh_.mu);
    if (pts >= 0.0)
      sh_.audio_end_pts = pts + static_cast<double>(got) / kOutRate;
    else if (sh_.audio_end_pts >= 0.0)
      sh_.audio_end_pts += static_cast<double>(got) / kOutRate;
  }

  // Decode one packet's frames into the picture queue / the sink. `target` > 0: drop what comes
  // before it (after a seek). Returns false at the end of the file.
  bool step(double target) {
    const int err = av_read_frame(fmt_, pkt_);
    const bool eof = err < 0;
    AVCodecContext *dec = nullptr;
    int idx = -1;
    if (eof) {
      // Drain both decoders.
      for (AVCodecContext *d : {vdec_, adec_})
        if (d)
          avcodec_send_packet(d, nullptr);
    } else if (pkt_->stream_index == vidx_) {
      dec = vdec_;
      idx = vidx_;
    } else if (pkt_->stream_index == aidx_) {
      dec = adec_;
      idx = aidx_;
    }
    if (dec)
      avcodec_send_packet(dec, pkt_);
    av_packet_unref(pkt_);
    const auto receive = [&](AVCodecContext *d, int i) {
      while (d && avcodec_receive_frame(d, frm_) >= 0) {
        const double pts = pts_of(frm_, i);
        if (target > 0.0 && pts >= 0.0 && pts + 0.05 < target) {
          av_frame_unref(frm_);
          continue;
        }
        if (d == vdec_) {
          cvc::image img = to_rgba(frm_);
          std::unique_lock<std::mutex> lock(sh_.mu);
          if (!img.empty()) {
            sh_.pics.push_back({std::move(img), pts});
            if (sh_.shown.empty() && sh_.state == am::PlayerState::Loading)
              sh_.state = am::PlayerState::Ready; // the first picture: the poster
          }
        } else {
          play_audio(frm_, pts);
        }
        av_frame_unref(frm_);
      }
    };
    if (eof) {
      receive(vdec_, vidx_);
      receive(adec_, aidx_);
      return false;
    }
    receive(dec, idx);
    return true;
  }

  void do_seek(double to) {
    const int64_t ts = static_cast<int64_t>(to * AV_TIME_BASE);
    av_seek_frame(fmt_, -1, ts, AVSEEK_FLAG_BACKWARD);
    for (AVCodecContext *d : {vdec_, adec_})
      if (d)
        avcodec_flush_buffers(d);
    std::shared_ptr<playback::audio_output> sink;
    {
      std::lock_guard<std::mutex> lock(sh_.mu);
      sh_.pics.clear();
      sh_.shown = cvc::image(); // the next picture decoded is the one to show
      sh_.audio_end_pts = -1.0;
      sh_.base_pts = to;
      sh_.base_time = clock_type::now();
      sink = sh_.sink;
    }
    if (sink)
      sink->flush();
    eof_ = false;
    // Decode up to the target, so the picture shows the new place even while paused.
    for (int i = 0; i < 400 && !quitting(); ++i) {
      if (!step(to)) {
        eof_ = true;
        break;
      }
      std::lock_guard<std::mutex> lock(sh_.mu);
      if (!sh_.pics.empty() || (vidx_ < 0 && sh_.audio_end_pts >= 0.0))
        break;
    }
  }

  void loop() {
    // Up to the first picture (or, sound only, opened): Ready.
    for (int i = 0; i < 400 && vidx_ >= 0; ++i) {
      {
        std::lock_guard<std::mutex> lock(sh_.mu);
        if (sh_.quit || !sh_.pics.empty())
          break;
      }
      if (!step(0.0)) {
        eof_ = true;
        break;
      }
    }
    {
      std::lock_guard<std::mutex> lock(sh_.mu);
      if (sh_.state == am::PlayerState::Loading)
        sh_.state = am::PlayerState::Ready;
      sh_.base_pts = 0.0;
    }
    for (;;) {
      std::unique_lock<std::mutex> lock(sh_.mu);
      if (sh_.quit)
        return;
      if (sh_.seek_to >= 0.0) {
        const double to = sh_.seek_to;
        sh_.seek_to = -1.0;
        const bool was_running = sh_.clock_running;
        if (sh_.state == am::PlayerState::Ended)
          sh_.state = am::PlayerState::Paused;
        lock.unlock();
        do_seek(to);
        lock.lock();
        if (was_running)
          sh_.run_clock(to);
        continue;
      }
      if (!sh_.want_play) {
        if (sh_.clock_running) {
          sh_.stop_clock();
          if (sh_.sink)
            sh_.sink->flush(); // the queued tail would play on
          sh_.audio_end_pts = -1.0;
        }
        if (sh_.state == am::PlayerState::Playing)
          sh_.state = am::PlayerState::Paused;
        sh_.cv.wait(lock);
        continue;
      }
      // Playing.
      if (aidx_ >= 0 && opt_.audio && !sh_.sink_tried) { // wait for the frame thread's device
        sh_.cv.wait_for(lock, std::chrono::milliseconds(20));
        continue;
      }
      if (!sh_.clock_running) {
        sh_.run_clock(sh_.base_pts);
        // Sound restarts the clock where it stopped.
        if (sh_.sink)
          sh_.audio_end_pts = -1.0;
      }
      sh_.state = am::PlayerState::Playing;
      // Enough ahead? Then wait for the picture or the sound to use some.
      const bool pics_full = vidx_ < 0 || sh_.pics.size() >= kFramesAhead;
      const bool sound_full = !sh_.sink || aidx_ < 0 ||
                              static_cast<double>(sh_.sink->queued_bytes()) /
                                      (sizeof(float) * kOutChannels * kOutRate) >=
                                  kAudioAhead;
      if (eof_ || (pics_full && sound_full)) {
        // At the end, pictures whose time has passed and no one took (frame() not called: the
        // widget not drawn) are stale.
        if (eof_)
          while (!sh_.pics.empty() && sh_.pics.front().pts < sh_.clock() - 0.1)
            sh_.pics.pop_front();
        if (eof_ && sh_.pics.empty() && (!sh_.sink || sh_.sink->queued_bytes() == 0 || aidx_ < 0) &&
            (sh_.duration <= 0 || sh_.clock() >= sh_.shown_pts)) {
          if (src_.loop) {
            sh_.seek_to = 0.0;
            continue;
          }
          sh_.stop_clock();
          sh_.want_play = false;
          sh_.state = am::PlayerState::Ended;
          continue;
        }
        sh_.cv.wait_for(lock, std::chrono::milliseconds(10));
        continue;
      }
      lock.unlock();
      if (!step(0.0))
        eof_ = true;
    }
  }

  am::PlayerSource src_;
  PlayerOptions opt_;
  mutable Shared sh_;
  std::thread worker_;
  // The decode thread's own.
  MemReader reader_;
  AVIOContext *io_ = nullptr;
  AVFormatContext *fmt_ = nullptr;
  AVCodecContext *vdec_ = nullptr, *adec_ = nullptr;
  int vidx_ = -1, aidx_ = -1;
  SwrContext *swr_ = nullptr;
  SwsContext *sws_ = nullptr;
  AVPacket *pkt_ = nullptr;
  AVFrame *frm_ = nullptr;
  bool eof_ = false;
};

} // namespace
#endif // CVC_MEDIA_FFMPEG

std::unique_ptr<am::Player> make_ffmpeg_player(const am::PlayerSource &src,
                                               const PlayerOptions &opt) {
#if CVC_MEDIA_FFMPEG
  return std::make_unique<FfmpegPlayer>(src, opt);
#else
  (void)src;
  (void)opt;
  return nullptr;
#endif
}

bool register_ariadne_player() {
  if (!have_ffmpeg_player())
    return false;
  am::register_player_factory(
      [](const am::PlayerSource &src) { return make_ffmpeg_player(src, PlayerOptions{}); });
  return true;
}

} // namespace media
} // namespace cvc
