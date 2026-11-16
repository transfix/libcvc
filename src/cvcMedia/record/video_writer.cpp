/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcMedia's video writer — see inc/cvc/media/video_writer.h.
//
// In process it is the usual libavcodec / libavformat sequence: an MP4 output context, one video
// stream, an encoder opened at 1 / fps time base, each RGB frame converted to YUV 4:2:0 by
// libswscale (a bottom-up frame read through a negative stride, so nothing is copied to flip it)
// and sent, its packets interleaved into the file. The program path runs `ffmpeg` with raw RGB on
// its stdin: no shell, so no quoting beyond the command line Windows needs.

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cvc/media/video_writer.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#if CVC_MEDIA_FFMPEG
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}
#endif

#if defined(__EMSCRIPTEN__)
// No processes to start, and cvcMedia has no FFmpeg in a browser.
#elif defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
extern char **environ;
#endif

namespace cvc {
namespace media {

namespace {

namespace fs = std::filesystem;

std::string check_options(const VideoWriterOptions &opt) {
  if (opt.width <= 0 || opt.height <= 0 || opt.fps <= 0)
    return "a video needs a width, a height and a frame rate above 0";
  if (opt.width % 2 || opt.height % 2)
    return "a video's width and height must be even (H.264's 4:2:0), not " +
           std::to_string(opt.width) + "x" + std::to_string(opt.height);
  return {};
}

// ── in process (FFmpeg's libraries) ─────────────────────────────────────────────────────────────

#if CVC_MEDIA_FFMPEG
// The in-process H.264 encoders, best first. FFmpeg's own MPEG-4 Part 2 encoder is the last resort.
constexpr const char *kH264Encoders[] = {"libx264", "libopenh264"};

std::string av_error(int code) {
  char buf[AV_ERROR_MAX_STRING_SIZE] = {0};
  av_strerror(code, buf, sizeof buf);
  return buf;
}

bool have_mp4_muxer() { return av_guess_format("mp4", nullptr, nullptr) != nullptr; }

// FFmpeg logs to stderr (libx264's settings and statistics at every file): quiet, as the player
// keeps it, unless asked for (CVC_MEDIA_FFMPEG_LOG=1). Errors come back through error().
void quiet_ffmpeg_log() {
  const char *v = std::getenv("CVC_MEDIA_FFMPEG_LOG");
  av_log_set_level(v && *v && *v != '0' ? AV_LOG_WARNING : AV_LOG_QUIET);
}

// The encoders to try in process, in order: the H.264 ones FFmpeg has, then (with `mpeg4`) its
// MPEG-4 Part 2 encoder.
std::vector<const AVCodec *> library_codecs(bool h264, bool mpeg4) {
  std::vector<const AVCodec *> out;
  if (!have_mp4_muxer())
    return out;
  if (h264)
    for (const char *name : kH264Encoders)
      if (const AVCodec *c = avcodec_find_encoder_by_name(name))
        out.push_back(c);
  if (mpeg4)
    if (const AVCodec *c = avcodec_find_encoder(AV_CODEC_ID_MPEG4))
      out.push_back(c);
  return out;
}

struct LibraryEncoder {
  AVFormatContext *fmt = nullptr;
  AVCodecContext *ctx = nullptr;
  AVStream *st = nullptr;
  SwsContext *sws = nullptr;
  AVFrame *frame = nullptr;
  AVPacket *pkt = nullptr;
  std::int64_t pts = 0;
  bool header = false; // the header is written: the trailer must be too
  int width = 0, height = 0;
  bool bottom_up = false;

  ~LibraryEncoder() {
    av_packet_free(&pkt);
    av_frame_free(&frame);
    sws_freeContext(sws);
    avcodec_free_context(&ctx);
    if (fmt) {
      if (fmt->pb && !(fmt->oformat->flags & AVFMT_NOFILE))
        avio_closep(&fmt->pb);
      avformat_free_context(fmt);
    }
  }

  // Open `codec` for `path`. False with `error` (the caller tries the next codec).
  bool open(const std::string &path, const VideoWriterOptions &opt, const AVCodec *codec,
            std::string &error) {
    quiet_ffmpeg_log();
    width = opt.width;
    height = opt.height;
    bottom_up = opt.bottom_up;
    int r = avformat_alloc_output_context2(&fmt, nullptr, "mp4", path.c_str());
    if (r < 0 || !fmt) {
      error = "no MP4 muxer: " + av_error(r);
      return false;
    }
    st = avformat_new_stream(fmt, nullptr);
    ctx = avcodec_alloc_context3(codec);
    if (!st || !ctx) {
      error = "out of memory";
      return false;
    }
    ctx->width = opt.width;
    ctx->height = opt.height;
    ctx->time_base = AVRational{1, opt.fps};
    ctx->framerate = AVRational{opt.fps, 1};
    ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    ctx->gop_size = 2 * opt.fps; // a key frame every 2 s, to seek to
    // What libswscale writes from RGB by default: BT.601, limited range. Said so in the stream, so
    // a player does not guess.
    ctx->colorspace = AVCOL_SPC_SMPTE170M;
    ctx->color_range = AVCOL_RANGE_MPEG;
    const std::string name = codec->name ? codec->name : "";
    if (name == "libx264") {
      av_opt_set(ctx->priv_data, "preset", "fast", 0);
      av_opt_set(ctx->priv_data, "crf", "20", 0);
    } else if (name == "libopenh264") {
      ctx->bit_rate = static_cast<std::int64_t>(opt.width) * opt.height * opt.fps / 8;
    } else if (codec->id == AV_CODEC_ID_MPEG4) {
      ctx->flags |= AV_CODEC_FLAG_QSCALE; // constant quality, near transparent
      ctx->global_quality = FF_QP2LAMBDA * 3;
    }
    if (fmt->oformat->flags & AVFMT_GLOBALHEADER)
      ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    if ((r = avcodec_open2(ctx, codec, nullptr)) < 0) {
      error = name + ": " + av_error(r);
      return false;
    }
    if ((r = avcodec_parameters_from_context(st->codecpar, ctx)) < 0) {
      error = av_error(r);
      return false;
    }
    st->time_base = ctx->time_base;
    st->avg_frame_rate = ctx->framerate;
    if (!(fmt->oformat->flags & AVFMT_NOFILE) &&
        (r = avio_open(&fmt->pb, path.c_str(), AVIO_FLAG_WRITE)) < 0) {
      error = path + ": " + av_error(r);
      return false;
    }
    AVDictionary *mux = nullptr;
    av_dict_set(&mux, "movflags", "+faststart", 0); // the index first: plays while it downloads
    r = avformat_write_header(fmt, &mux);
    av_dict_free(&mux);
    if (r < 0) {
      error = path + ": " + av_error(r);
      return false;
    }
    header = true;
    frame = av_frame_alloc();
    pkt = av_packet_alloc();
    if (!frame || !pkt) {
      error = "out of memory";
      return false;
    }
    frame->format = ctx->pix_fmt;
    frame->width = ctx->width;
    frame->height = ctx->height;
    if ((r = av_frame_get_buffer(frame, 0)) < 0) {
      error = av_error(r);
      return false;
    }
    sws = sws_getContext(width, height, AV_PIX_FMT_RGB24, width, height, ctx->pix_fmt, SWS_BILINEAR,
                         nullptr, nullptr, nullptr);
    if (!sws) {
      error = "libswscale has no RGB to YUV 4:2:0 conversion";
      return false;
    }
    return true;
  }

  // Send one frame (null: flush) and write the packets it gives back.
  bool send(const AVFrame *f, std::string &error) {
    int r = avcodec_send_frame(ctx, f);
    if (r < 0) {
      error = "encoding: " + av_error(r);
      return false;
    }
    for (;;) {
      r = avcodec_receive_packet(ctx, pkt);
      if (r == AVERROR(EAGAIN) || r == AVERROR_EOF)
        return true;
      if (r < 0) {
        error = "encoding: " + av_error(r);
        return false;
      }
      av_packet_rescale_ts(pkt, ctx->time_base, st->time_base);
      pkt->stream_index = st->index;
      if ((r = av_interleaved_write_frame(fmt, pkt)) < 0) { // takes the packet's data
        error = "writing: " + av_error(r);
        return false;
      }
    }
  }

  bool write(const unsigned char *rgb, std::string &error) {
    int r = av_frame_make_writable(frame);
    if (r < 0) {
      error = av_error(r);
      return false;
    }
    const int row = width * 3;
    const std::uint8_t *src[1] = {bottom_up ? rgb + static_cast<std::size_t>(height - 1) * row
                                            : rgb};
    const int stride[1] = {bottom_up ? -row : row};
    sws_scale(sws, src, stride, 0, height, frame->data, frame->linesize);
    frame->pts = pts++;
    return send(frame, error);
  }

  bool finish(std::string &error) {
    bool ok = send(nullptr, error);
    if (header) {
      const int r = av_write_trailer(fmt);
      header = false;
      if (r < 0 && ok) {
        error = "writing: " + av_error(r);
        ok = false;
      }
    }
    if (fmt && fmt->pb && !(fmt->oformat->flags & AVFMT_NOFILE))
      avio_closep(&fmt->pb);
    return ok;
  }
};
#endif // CVC_MEDIA_FFMPEG

// ── the ffmpeg program ──────────────────────────────────────────────────────────────────────────

#if !defined(__EMSCRIPTEN__)
std::vector<std::string> program_args(const std::string &program, const std::string &path,
                                      const VideoWriterOptions &opt) {
  std::vector<std::string> a = {
      program,       "-hide_banner",
      "-nostats",    "-loglevel",
      "error",       "-y",
      "-f",          "rawvideo",
      "-pix_fmt",    "rgb24",
      "-video_size", std::to_string(opt.width) + "x" + std::to_string(opt.height),
      "-framerate",  std::to_string(opt.fps),
      "-i",          "pipe:0"};
  if (opt.bottom_up)
    a.insert(a.end(), {"-vf", "vflip"});
  // Its MP4 default codec: libx264 where it has it, else MPEG-4 Part 2. The file: protocol keeps
  // a path that starts with '-' from reading as an option.
  a.insert(a.end(),
           {"-pix_fmt", "yuv420p", "-movflags", "+faststart", "-f", "mp4", "file:" + path});
  return a;
}
#endif

#if defined(_WIN32) && !defined(__EMSCRIPTEN__)
// One argument as CommandLineToArgvW (and the C runtime) splits it back out.
std::wstring quote_arg(const std::wstring &arg) {
  if (!arg.empty() && arg.find_first_of(L" \t\n\v\"") == std::wstring::npos)
    return arg;
  std::wstring q = L"\"";
  for (auto it = arg.begin();; ++it) {
    std::size_t slashes = 0;
    while (it != arg.end() && *it == L'\\') {
      ++it;
      ++slashes;
    }
    if (it == arg.end()) {
      q.append(slashes * 2, L'\\'); // before the closing quote, each one doubled
      break;
    }
    if (*it == L'"') {
      q.append(slashes * 2 + 1, L'\\');
      q.push_back(L'"');
    } else {
      q.append(slashes, L'\\');
      q.push_back(*it);
    }
  }
  q.push_back(L'"');
  return q;
}

std::wstring widen(const std::string &s) {
  if (s.empty())
    return {};
  const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
  std::wstring w(static_cast<std::size_t>(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
  return w;
}

struct ProgramEncoder {
  HANDLE in = nullptr; // our end of its stdin
  PROCESS_INFORMATION pi{};

  ~ProgramEncoder() {
    std::string ignored;
    finish(ignored);
  }

  bool start(const std::vector<std::string> &args, std::string &error) {
    SECURITY_ATTRIBUTES sa{sizeof sa, nullptr, TRUE};
    HANDLE rd = nullptr;
    if (!CreatePipe(&rd, &in, &sa, 1 << 20)) {
      error = "could not make a pipe to ffmpeg";
      return false;
    }
    SetHandleInformation(in, HANDLE_FLAG_INHERIT, 0); // only the read end goes to ffmpeg
    std::wstring line;
    for (const std::string &a : args)
      line += (line.empty() ? L"" : L" ") + quote_arg(widen(a));
    STARTUPINFOW si{};
    si.cb = sizeof si;
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = rd;
    si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
    si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
    const std::wstring exe = widen(args.front());
    const BOOL ok = CreateProcessW(exe.c_str(), line.data(), nullptr, nullptr, TRUE, 0, nullptr,
                                   nullptr, &si, &pi);
    CloseHandle(rd);
    if (!ok) {
      error = "could not start " + args.front() + " (error " + std::to_string(GetLastError()) + ")";
      CloseHandle(in);
      in = nullptr;
      return false;
    }
    CloseHandle(pi.hThread);
    pi.hThread = nullptr;
    return true;
  }

  bool write(const unsigned char *p, std::size_t n, std::string &error) {
    while (n > 0) {
      DWORD wrote = 0;
      const DWORD chunk = static_cast<DWORD>(n > (1u << 30) ? (1u << 30) : n);
      if (!WriteFile(in, p, chunk, &wrote, nullptr) || wrote == 0) {
        std::string why;
        finish(why); // it has gone: say how it ended
        error = why.empty() ? "ffmpeg stopped reading frames" : why;
        return false;
      }
      p += wrote;
      n -= wrote;
    }
    return true;
  }

  bool finish(std::string &error) {
    if (in) {
      CloseHandle(in); // end of input: ffmpeg flushes and writes the trailer
      in = nullptr;
    }
    if (!pi.hProcess)
      return true;
    WaitForSingleObject(pi.hProcess, INFINITE);
    DWORD code = 0;
    GetExitCodeProcess(pi.hProcess, &code);
    CloseHandle(pi.hProcess);
    pi.hProcess = nullptr;
    if (code != 0) {
      error = "ffmpeg exited with status " + std::to_string(code);
      return false;
    }
    return true;
  }
};
#elif !defined(__EMSCRIPTEN__)
struct ProgramEncoder {
  int fd = -1; // our end of its stdin
  pid_t pid = -1;

  ~ProgramEncoder() {
    std::string ignored;
    finish(ignored);
  }

  bool start(const std::vector<std::string> &args, std::string &error) {
    int fds[2];
    if (pipe(fds) != 0) {
      error = std::string("could not make a pipe to ffmpeg: ") + std::strerror(errno);
      return false;
    }
    fcntl(fds[1], F_SETFD, FD_CLOEXEC); // only the read end goes to ffmpeg
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, fds[0], 0);
    if (fds[0] != 0)
      posix_spawn_file_actions_addclose(&fa, fds[0]);
    std::vector<char *> argv;
    for (const std::string &a : args)
      argv.push_back(const_cast<char *>(a.c_str()));
    argv.push_back(nullptr);
    const int rc = posix_spawn(&pid, argv[0], &fa, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    close(fds[0]);
    if (rc != 0) {
      error = "could not start " + args.front() + ": " + std::strerror(rc);
      close(fds[1]);
      pid = -1;
      return false;
    }
    fd = fds[1];
    return true;
  }

  // SIGPIPE is held off this thread while it writes, so an ffmpeg that has gone fails the write
  // (EPIPE) rather than ending the process; a SIGPIPE it raised is taken before the mask goes back.
  bool write(const unsigned char *p, std::size_t n, std::string &error) {
    sigset_t pipe_sig, old;
    sigemptyset(&pipe_sig);
    sigaddset(&pipe_sig, SIGPIPE);
    pthread_sigmask(SIG_BLOCK, &pipe_sig, &old);
    bool ok = true;
    while (n > 0) {
      const ssize_t w = ::write(fd, p, n);
      if (w < 0 && errno == EINTR)
        continue;
      if (w <= 0) {
        ok = false;
        break;
      }
      p += w;
      n -= static_cast<std::size_t>(w);
    }
    if (!ok && errno == EPIPE) {
      sigset_t pending;
      sigpending(&pending);
      if (sigismember(&pending, SIGPIPE)) {
        int sig = 0;
        sigwait(&pipe_sig, &sig);
      }
    }
    pthread_sigmask(SIG_SETMASK, &old, nullptr);
    if (!ok) {
      std::string why;
      finish(why); // it has gone: say how it ended
      error = why.empty() ? "ffmpeg stopped reading frames" : why;
    }
    return ok;
  }

  bool finish(std::string &error) {
    if (fd >= 0) {
      close(fd); // end of input: ffmpeg flushes and writes the trailer
      fd = -1;
    }
    if (pid <= 0)
      return true;
    int status = 0;
    while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
    }
    pid = -1;
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      error = WIFEXITED(status) ? "ffmpeg exited with status " + std::to_string(WEXITSTATUS(status))
                                : std::string("ffmpeg was killed");
      return false;
    }
    return true;
  }
};
#endif

#if !defined(__EMSCRIPTEN__)
// The ffmpeg `opt` would run: the one it names, if it is there, else the one on PATH.
std::string program_for(const VideoWriterOptions &opt) {
  if (opt.program.empty())
    return find_ffmpeg_program();
  std::error_code ec;
  return fs::is_regular_file(opt.program, ec) ? opt.program : std::string();
}
#endif

// The first directory of PATH (`all`, split at `sep`) holding the program `name`, or "".
template <class String>
std::string search_path(const String &all, typename String::value_type sep,
                        const typename String::value_type *name) {
  for (std::size_t at = 0; at <= all.size();) {
    std::size_t end = all.find(sep, at);
    if (end == String::npos)
      end = all.size();
    String dir = all.substr(at, end - at);
    at = end + 1;
    if (dir.size() >= 2 && dir.front() == '"' && dir.back() == '"')
      dir = dir.substr(1, dir.size() - 2);
    if (dir.empty())
      continue; // PATH's empty entry (the working directory) is no place to look for a program
    std::error_code ec;
    const fs::path p = fs::path(dir) / name;
    if (!fs::is_regular_file(p, ec))
      continue;
#if !defined(_WIN32) && !defined(__EMSCRIPTEN__)
    if (access(p.c_str(), X_OK) != 0)
      continue;
#endif
    try {
      return p.string();
    } catch (const std::exception &) {
      continue; // a name a narrow string can't hold
    }
  }
  return {};
}

} // namespace

bool have_video_library() {
#if CVC_MEDIA_FFMPEG
  return true;
#else
  return false;
#endif
}

std::string find_ffmpeg_program() {
#if defined(__EMSCRIPTEN__)
  return {};
#else
#if defined(_WIN32)
  const wchar_t *path = _wgetenv(L"PATH");
  return path ? search_path(std::wstring(path), L';', L"ffmpeg.exe") : std::string();
#else
  const char *path = std::getenv("PATH");
  return path ? search_path(std::string(path), ':', "ffmpeg") : std::string();
#endif
#endif
}

std::string video_encoder(const VideoWriterOptions &opt, std::string *why) {
#if defined(__EMSCRIPTEN__)
  (void)opt;
  if (why)
    *why = "no video encoder in a browser";
  return {};
#else
  using Backend = VideoWriterOptions::Backend;
#if CVC_MEDIA_FFMPEG
  if (opt.backend != Backend::Program)
    if (const auto c = library_codecs(true, false); !c.empty())
      return c.front()->name;
#endif
  if (opt.backend != Backend::Library)
    if (const std::string p = program_for(opt); !p.empty())
      return "ffmpeg program (" + p + ")";
#if CVC_MEDIA_FFMPEG
  if (opt.backend != Backend::Program)
    if (const auto c = library_codecs(false, true); !c.empty())
      return c.front()->name;
#endif
  if (why) {
    if (opt.backend == Backend::Program)
      *why = opt.program.empty() ? "no ffmpeg program on PATH" : "no program " + opt.program;
    else if (!have_video_library())
      *why = opt.backend == Backend::Library
                 ? "this cvcMedia was built without FFmpeg"
                 : "this cvcMedia was built without FFmpeg, and there is no ffmpeg program on PATH";
    else
      *why = std::string("this FFmpeg has no H.264 or MPEG-4 encoder, or no MP4 muxer") +
             (opt.backend == Backend::Library ? "" : ", and there is no ffmpeg program on PATH");
  }
  return {};
#endif
}

struct VideoWriter::Impl {
  VideoWriterOptions opt;
  std::string error, encoder;
  int frames = 0;
  bool open = false;
#if CVC_MEDIA_FFMPEG
  std::unique_ptr<LibraryEncoder> lib;
#endif
#if !defined(__EMSCRIPTEN__)
  std::unique_ptr<ProgramEncoder> prog;
#endif

  // Each in-process encoder in `codecs` in turn, until one opens.
  bool try_library(const std::string &path, bool h264, bool mpeg4) {
#if CVC_MEDIA_FFMPEG
    for (const AVCodec *c : library_codecs(h264, mpeg4)) {
      auto e = std::make_unique<LibraryEncoder>();
      std::string why;
      if (e->open(path, opt, c, why)) {
        lib = std::move(e);
        encoder = c->name;
        return true;
      }
      error = why;
    }
#else
    (void)path;
    (void)h264;
    (void)mpeg4;
#endif
    return false;
  }

  bool try_program(const std::string &path) {
#if !defined(__EMSCRIPTEN__)
    const std::string program = program_for(opt);
    if (program.empty())
      return false;
    auto p = std::make_unique<ProgramEncoder>();
    std::string why;
    if (!p->start(program_args(program, path, opt), why)) {
      error = why;
      return false;
    }
    prog = std::move(p);
    encoder = "ffmpeg program (" + program + ")";
    return true;
#else
    (void)path;
    return false;
#endif
  }
};

VideoWriter::VideoWriter() : impl_(std::make_unique<Impl>()) {}

VideoWriter::~VideoWriter() { close(); }

bool VideoWriter::open(const std::string &path, const VideoWriterOptions &opt) {
  using Backend = VideoWriterOptions::Backend;
  close();
  Impl &m = *impl_;
  m.opt = opt;
  m.error.clear();
  m.encoder.clear();
  m.frames = 0;
  if (const std::string bad = check_options(opt); !bad.empty()) {
    m.error = bad;
    return false;
  }
  std::string why;
  if (video_encoder(opt, &why).empty()) {
    m.error = "no video encoder: " + why;
    return false;
  }
  // A path that can't be written fails here, whichever encoder would have written it (a program's
  // would only say so once it ran).
  if (!std::ofstream(path, std::ios::binary | std::ios::trunc)) {
    m.error = path + ": can't write it: " + std::strerror(errno);
    return false;
  }
  const bool library = opt.backend != Backend::Program, program = opt.backend != Backend::Library;
  m.open = (library && m.try_library(path, true, false)) || (program && m.try_program(path)) ||
           (library && m.try_library(path, false, true));
  if (m.open)
    m.error.clear(); // an encoder tried first that failed to open
  else if (m.error.empty())
    m.error = "no video encoder opened";
  return m.open;
}

bool VideoWriter::write_rgb(const unsigned char *rgb, std::size_t bytes) {
  Impl &m = *impl_;
  if (!m.open) {
    if (m.error.empty())
      m.error = "the video is not open";
    return false;
  }
  const std::size_t want = static_cast<std::size_t>(m.opt.width) * m.opt.height * 3;
  if (!rgb || bytes != want) {
    m.error = "a frame of " + std::to_string(bytes) + " bytes, where " +
              std::to_string(m.opt.width) + "x" + std::to_string(m.opt.height) + " RGB is " +
              std::to_string(want);
    close();
    return false;
  }
  bool ok = false;
  std::string why;
#if CVC_MEDIA_FFMPEG
  if (m.lib)
    ok = m.lib->write(rgb, why);
#endif
#if !defined(__EMSCRIPTEN__)
  if (m.prog)
    ok = m.prog->write(rgb, bytes, why);
#endif
  if (!ok) {
    m.error = why;
    close();
    return false;
  }
  ++m.frames;
  return true;
}

bool VideoWriter::close() {
  Impl &m = *impl_;
  if (!m.open)
    return m.error.empty();
  m.open = false;
  bool ok = true;
  std::string why;
#if CVC_MEDIA_FFMPEG
  if (m.lib) {
    ok = m.lib->finish(why);
    m.lib.reset();
  }
#endif
#if !defined(__EMSCRIPTEN__)
  if (m.prog) {
    ok = m.prog->finish(why);
    m.prog.reset();
  }
#endif
  if (!ok && m.error.empty())
    m.error = why;
  return ok && m.error.empty();
}

bool VideoWriter::is_open() const { return impl_->open; }
const std::string &VideoWriter::error() const { return impl_->error; }
const std::string &VideoWriter::encoder() const { return impl_->encoder; }
int VideoWriter::frames() const { return impl_->frames; }

// ── reading one back ────────────────────────────────────────────────────────────────────────────

VideoInfo probe_video(const std::string &path, bool keep_first_and_last) {
  VideoInfo info;
#if CVC_MEDIA_FFMPEG
  quiet_ffmpeg_log();
  AVFormatContext *in = nullptr;
  int r = avformat_open_input(&in, path.c_str(), nullptr, nullptr);
  if (r < 0) {
    info.error = path + ": " + av_error(r);
    return info;
  }
  AVCodecContext *dec = nullptr;
  AVPacket *pkt = av_packet_alloc();
  AVFrame *frame = av_frame_alloc();
  SwsContext *sws = nullptr;
  const auto done = [&] {
    sws_freeContext(sws);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    avcodec_free_context(&dec);
    avformat_close_input(&in);
    return info;
  };
  if ((r = avformat_find_stream_info(in, nullptr)) < 0) {
    info.error = path + ": " + av_error(r);
    return done();
  }
  const int vs = av_find_best_stream(in, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if (vs < 0) {
    info.error = path + ": no video stream";
    return done();
  }
  const AVStream *st = in->streams[vs];
  info.format = in->iformat && in->iformat->name ? in->iformat->name : "";
  info.codec = avcodec_get_name(st->codecpar->codec_id);
  info.width = st->codecpar->width;
  info.height = st->codecpar->height;
  info.fps = st->avg_frame_rate.den ? av_q2d(st->avg_frame_rate) : 0.0;
  const AVCodec *codec = avcodec_find_decoder(st->codecpar->codec_id);
  dec = codec ? avcodec_alloc_context3(codec) : nullptr;
  if (!dec || avcodec_parameters_to_context(dec, st->codecpar) < 0 ||
      avcodec_open2(dec, codec, nullptr) < 0) {
    info.error = path + ": no " + info.codec + " decoder";
    return done();
  }
  std::vector<unsigned char> *keep = nullptr;
  const auto take = [&](const AVFrame *f) {
    ++info.frames;
    if (!keep_first_and_last)
      return;
    keep = info.frames == 1 ? &info.first_rgb : &info.last_rgb;
    sws = sws_getCachedContext(sws, f->width, f->height, static_cast<AVPixelFormat>(f->format),
                               f->width, f->height, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr,
                               nullptr, nullptr);
    if (!sws)
      return;
    keep->assign(static_cast<std::size_t>(f->width) * f->height * 3, 0);
    std::uint8_t *dst[1] = {keep->data()};
    const int stride[1] = {f->width * 3};
    sws_scale(sws, f->data, f->linesize, 0, f->height, dst, stride);
  };
  const auto drain = [&] {
    while (avcodec_receive_frame(dec, frame) == 0) {
      take(frame);
      av_frame_unref(frame);
    }
  };
  while (av_read_frame(in, pkt) >= 0) {
    if (pkt->stream_index == vs && avcodec_send_packet(dec, pkt) >= 0)
      drain();
    av_packet_unref(pkt);
  }
  avcodec_send_packet(dec, nullptr); // flush
  drain();
  if (keep_first_and_last && info.frames == 1)
    info.last_rgb = info.first_rgb;
  info.duration = in->duration > 0 ? static_cast<double>(in->duration) / AV_TIME_BASE
                  : info.fps > 0   ? info.frames / info.fps
                                   : 0.0;
  info.ok = true;
  return done();
#else
  (void)keep_first_and_last;
  info.error = path + ": this cvcMedia was built without FFmpeg";
  return info;
#endif
}

} // namespace media
} // namespace cvc
