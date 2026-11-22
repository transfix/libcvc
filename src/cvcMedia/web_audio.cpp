/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/media/web_audio.h>

#ifdef __EMSCRIPTEN__
#include "cvc_audio_worklet_js.h" // cvc_audio_worklet_js(): web/cvc-audio-worklet.js, for a page without it

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <emscripten.h>
#include <mutex>
#include <vector>

EM_JS_DEPS(cvc_web_audio, "$UTF8ToString,$stringToUTF8");
#endif

namespace cvc {
namespace media {
namespace web_audio {

#ifdef __EMSCRIPTEN__
// The page's side: Module.cvcAudio holds the one AudioContext, the worklets' load, the nodes open
// ({nodes: {id: {kind, node, ...}}}, numbered by the page) and the clocks (the C++ clock objects'
// addresses). A node's messages call back into this side with the C++ object they were opened
// for (cvc_wa_capture_push, cvc_wa_play_tick, below), and closing a node marks its record closed
// before the object goes, so a message already queued for it is dropped. state: 0 off, 1 loading,
// 2 ready, 3 failed, 4 unsupported.
// clang-format off
EM_JS(int, cvc_wa_prepare, (const char *src, int off), {
  var A = Module.cvcAudio;
  if (A) return A.state;
  A = Module.cvcAudio = {state: 0, why: "", ctx: null, nodes: {}, next: 1, clocks: []};
  A.fire = function() {
    var c = A.clocks.slice();
    for (var i = 0; i < c.length; i++)
      if (A.clocks.indexOf(c[i]) >= 0) _cvc_wa_clock(c[i]);
  };
  if (off) { A.why = 'off (CVC_WEB_AUDIO=sdl)'; return 0; }
  var AC = typeof window !== 'undefined' ? (window.AudioContext || window.webkitAudioContext) : null;
  if (!AC || typeof AudioWorkletNode === 'undefined') {
    A.state = 4; A.why = 'unsupported (no AudioWorklet)'; return 4;
  }
  if (!window.isSecureContext || !navigator.mediaDevices) {
    A.state = 4; A.why = 'unsupported (not a secure context)'; return 4;
  }
  try {
    A.ctx = new AC({sampleRate: 48000, latencyHint: 'interactive'});
  } catch (e) {
    try { A.ctx = new AC({latencyHint: 'interactive'}); }
    catch (e2) { A.state = 3; A.why = 'failed: ' + e2; return 3; }
  }
  if (!A.ctx.audioWorklet) { A.state = 4; A.why = 'unsupported (no audioWorklet)'; return 4; }
  A.state = 1;
  A.why = 'loading';
  // The file beside the page first (a page whose Content-Security-Policy allows only its own
  // scripts can load it); else the copy compiled in, from a blob: URL.
  var text = UTF8ToString(src);
  var file = typeof locateFile === 'function' ? locateFile('cvc-audio-worklet.js') : 'cvc-audio-worklet.js';
  // A.loaded settles once they load or fail; the nodes opened meanwhile start then.
  A.loaded = A.ctx.audioWorklet.addModule(file).catch(function() {
    var url = URL.createObjectURL(new Blob([text], {type: 'text/javascript'}));
    return A.ctx.audioWorklet.addModule(url).finally(function() { URL.revokeObjectURL(url); });
  }).then(function() {
    A.state = 2; A.why = 'ready';
  }, function(e) {
    A.state = 3; A.why = 'failed: ' + ((e && e.message) || e);
    console.warn('web audio: the AudioWorklets did not load (' + A.why + '); devices opened from now on use SDL');
    throw e;
  });
  A.loaded.catch(function() {}); // said above; each node waiting on it handles it too
  return 1;
});

EM_JS(int, cvc_wa_state, (), { return Module.cvcAudio ? Module.cvcAudio.state : -1; });

EM_JS(int, cvc_wa_rate, (), {
  var A = Module.cvcAudio;
  return A && A.ctx ? A.ctx.sampleRate : 0;
});

EM_JS(void, cvc_wa_status, (char *buf, int cap), {
  var A = Module.cvcAudio;
  stringToUTF8(A ? A.why : 'not prepared', buf, cap);
});

// The mic: getUserMedia (the browser asks for permission), its source into a cvc-capture node,
// whose blocks go to SRC (a capture_source). The node is connected to the destination so the
// graph pulls it; its output is silence.
EM_JS(int, cvc_wa_open_capture, (void *src, int channels), {
  var A = Module.cvcAudio;
  if (!A || (A.state !== 1 && A.state !== 2)) return 0;
  var id = A.next++;
  var rec = A.nodes[id] = {kind: 'capture', closed: false, node: null, mic: null, stream: null, why: ""};
  var mic = navigator.mediaDevices.getUserMedia({
    audio: {channelCount: channels, echoCancellation: true, noiseSuppression: true, autoGainControl: true},
    video: false
  });
  Promise.all([mic, A.loaded]).then(function(got) {
    var stream = got[0];
    if (rec.closed) { stream.getTracks().forEach(function(t) { t.stop(); }); return; }
    rec.stream = stream;
    rec.mic = A.ctx.createMediaStreamSource(stream);
    var node = rec.node = new AudioWorkletNode(A.ctx, 'cvc-capture', {
      numberOfInputs: 1, numberOfOutputs: 1, outputChannelCount: [1],
      channelCount: channels, channelCountMode: 'explicit', channelInterpretation: 'speakers',
      processorOptions: {channels: channels}
    });
    node.port.onmessage = function(e) {
      var m = e.data;
      if (rec.closed || !m || m.t !== 'cap') return;
      var p = _cvc_wa_capture_buffer(src, m.pcm.length) >>> 0;
      if (!p) return;
      HEAPF32.set(m.pcm, p >>> 2);
      _cvc_wa_capture_push(src, m.pcm.length);
      if (!rec.closed) A.fire();
    };
    rec.mic.connect(node);
    node.connect(A.ctx.destination);
  }, function(e) {
    rec.why = String((e && e.name) || e);
    console.warn('web audio: no mic (' + rec.why + ')');
    mic.then(function(stream) { stream.getTracks().forEach(function(t) { t.stop(); }); }, function() {});
  });
  return id;
});

// The speaker: a cvc-playback node into the destination, once the worklets have loaded. Its ticks
// run the clocks, then ask SINK (a worklet_output) to top the ring up.
EM_JS(int, cvc_wa_open_playback, (void *sink, int channels, int block), {
  var A = Module.cvcAudio;
  if (!A || (A.state !== 1 && A.state !== 2)) return 0;
  var id = A.next++;
  var rec = A.nodes[id] = {kind: 'playback', closed: false, node: null};
  A.loaded.then(function() {
    if (rec.closed) return;
    var node = rec.node = new AudioWorkletNode(A.ctx, 'cvc-playback', {
      numberOfInputs: 0, numberOfOutputs: 1, outputChannelCount: [channels],
      processorOptions: {channels: channels, blockFrames: block}
    });
    node.port.onmessage = function(e) {
      var m = e.data;
      if (rec.closed || !m || m.t !== 'tick') return;
      A.fire();
      if (!rec.closed) _cvc_wa_play_tick(sink, m.read, m.under);
    };
    node.connect(A.ctx.destination);
  }).catch(function(e) { console.warn('web audio: no speaker (' + e + ')'); });
  return id;
});

// N floats at PCM to node ID's ring; 0 when it has no node yet (the worklets are loading).
EM_JS(int, cvc_wa_post, (int id, const float *pcm, int n), {
  var rec = Module.cvcAudio && Module.cvcAudio.nodes[id];
  if (!rec || rec.closed || !rec.node) return 0;
  var a = HEAPF32.slice(pcm >>> 2, (pcm >>> 2) + n);
  rec.node.port.postMessage({t: 'pcm', pcm: a}, [a.buffer]);
  return 1;
});

// To node ID: 1 'drop' FRAMES from the ring's head, 2 'flush' it.
EM_JS(void, cvc_wa_send, (int id, int what, int frames), {
  var rec = Module.cvcAudio && Module.cvcAudio.nodes[id];
  if (!rec || rec.closed || !rec.node) return;
  rec.node.port.postMessage(what === 1 ? {t: 'drop', frames: frames} : {t: 'flush'});
});

EM_JS(void, cvc_wa_close, (int id), {
  var A = Module.cvcAudio;
  var rec = A && A.nodes[id];
  if (!rec) return;
  rec.closed = true;
  delete A.nodes[id];
  try { if (rec.node) { rec.node.port.postMessage({t: 'stop'}); rec.node.disconnect(); } } catch (e) {}
  try { if (rec.mic) rec.mic.disconnect(); } catch (e) {}
  if (rec.stream) rec.stream.getTracks().forEach(function(t) { t.stop(); });
});

EM_JS(void, cvc_wa_add_clock, (void *c), {
  if (Module.cvcAudio) Module.cvcAudio.clocks.push(c);
});

EM_JS(void, cvc_wa_remove_clock, (void *c), {
  var a = Module.cvcAudio ? Module.cvcAudio.clocks : [];
  var i = a.indexOf(c);
  if (i >= 0) a.splice(i, 1);
});
// clang-format on

namespace {

double steady_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// The mic: what the capture node posts, queued, read a chunk at a time by the stream's producer.
// The queue is filled on the main thread and read on the producer's (the main thread too in a
// single-threaded build); each push wakes the producer (set_ready_callback). _cb_mu is held while
// the producer is woken, so once set_ready_callback({}) returns no wake-up runs or will run.
class capture_source : public cvc::ariadne::stream::frame_source {
public:
  capture_source(int rate, int channels, int frames) : _rate(rate), _ch(channels), _frames(frames) {
    // Drop at source past 250 ms queued (four chunks at least), as SDL's mic does.
    const std::size_t chunk = static_cast<std::size_t>(_frames) * static_cast<std::size_t>(_ch);
    const std::size_t quarter_s =
        static_cast<std::size_t>(_rate) * static_cast<std::size_t>(_ch) / 4u;
    _max_queued = chunk * 4u > quarter_s ? chunk * 4u : quarter_s;
    _id = cvc_wa_open_capture(this, _ch);
  }
  ~capture_source() override {
    if (_id)
      cvc_wa_close(_id);
  }
  bool opened() const { return _id != 0; }

  std::size_t frame_bytes() const override {
    return static_cast<std::size_t>(_frames) * static_cast<std::size_t>(_ch) * sizeof(float);
  }

  bool more() const override {
    std::lock_guard<std::mutex> lk(_mu);
    return _queue.size() >= chunk();
  }

  void set_ready_callback(std::function<void()> ready) override {
    std::lock_guard<std::mutex> lk(_cb_mu);
    _ready = std::move(ready);
  }

  cvc::ariadne::stream::produced_frame fill(std::uint8_t *buf, std::size_t cap) override {
    cvc::ariadne::stream::produced_frame out;
    const std::size_t need = chunk();
    if (buf == nullptr || cap < need * sizeof(float))
      return out;
    std::lock_guard<std::mutex> lk(_mu);
    if (_queue.size() < need)
      return out;
    if (_queue.size() > _max_queued) { // a consumer stalled: keep the newest chunk, skip the clock
      const std::size_t drop = _queue.size() - need;
      _queue.erase(_queue.begin(), _queue.begin() + static_cast<std::ptrdiff_t>(drop));
      _consumed += drop / static_cast<std::size_t>(_ch);
    }
    float *dst = reinterpret_cast<float *>(buf);
    std::copy(_queue.begin(), _queue.begin() + static_cast<std::ptrdiff_t>(need), dst);
    _queue.erase(_queue.begin(), _queue.begin() + static_cast<std::ptrdiff_t>(need));
    out.bytes = need * sizeof(float);
    out.pts_seconds = static_cast<double>(_consumed) / static_cast<double>(_rate);
    _consumed += static_cast<std::uint64_t>(_frames);
    return out;
  }

  // The page's side: room for N floats, then push them (cvc_wa_capture_buffer / _push).
  float *buffer(int n) {
    _scratch.resize(n > 0 ? static_cast<std::size_t>(n) : 0u);
    return _scratch.empty() ? nullptr : _scratch.data();
  }
  void push(int n) {
    const std::size_t count =
        n > 0 && static_cast<std::size_t>(n) <= _scratch.size() ? static_cast<std::size_t>(n) : 0u;
    if (count == 0)
      return;
    {
      std::lock_guard<std::mutex> lk(_mu);
      _queue.insert(_queue.end(), _scratch.begin(),
                    _scratch.begin() + static_cast<std::ptrdiff_t>(count));
    }
    std::lock_guard<std::mutex> lk(_cb_mu);
    if (_ready)
      _ready();
  }

private:
  std::size_t chunk() const {
    return static_cast<std::size_t>(_frames) * static_cast<std::size_t>(_ch);
  }

  int _rate, _ch, _frames;
  int _id = 0;
  std::size_t _max_queued = 0; // floats
  std::uint64_t _consumed = 0; // frames read or dropped (the pts clock)
  mutable std::mutex _mu;      // _queue
  std::deque<float> _queue;
  std::mutex _cb_mu; // _ready
  std::function<void()> _ready;
  std::vector<float> _scratch; // the page writes a block here, then push()es it (main thread)
};

// The speaker: a playback node whose ring this keeps TARGET frames ahead of the device. On each
// of the node's ticks (READ: frames it has taken from its ring; UNDER: frames it played as silence
// because the ring was empty), it pulls what the device took from FILL and posts it. Main thread
// only.
class worklet_output : public cvc::media::playback::audio_output {
public:
  worklet_output(int rate, int channels, cvc::media::playback::audio_pull_fn fill)
      : _rate(rate), _ch(channels), _fill(std::move(fill)), _block(rate / 50 > 0 ? rate / 50 : 1) {
    _min_target = 2 * _block;  // 40 ms
    _target = 3 * _block;      // 60 ms to start
    _max_target = 20 * _block; // 400 ms
    _id = cvc_wa_open_playback(this, _ch, _block);
    _changed_at = steady_s();
  }
  ~worklet_output() override {
    if (_id)
      cvc_wa_close(_id);
  }
  bool opened() const { return _id != 0; }

  bool play(const void *data, std::size_t bytes) override {
    const std::size_t frame_sz = static_cast<std::size_t>(_ch) * sizeof(float);
    if (!_id || data == nullptr || bytes == 0 || bytes % frame_sz != 0)
      return false;
    post(static_cast<const float *>(data), bytes / frame_sz);
    return true;
  }

  std::size_t queued_bytes() const override {
    const double frames = _posted - _read;
    return frames > 0.0
               ? static_cast<std::size_t>(frames) * static_cast<std::size_t>(_ch) * sizeof(float)
               : 0u;
  }

  void flush() override {
    if (_id)
      cvc_wa_send(_id, 2, 0);
    _read = _posted;
  }

  double device_period_s() const override {
    return static_cast<double>(_block) / static_cast<double>(_rate);
  }

  void tick(double read, double under) {
    if (read > _read)
      _read = read;
    const double now = steady_s();
    // The main thread came too late, and the ring ran dry: play further ahead, by what it lacked
    // and a block more, so one dry spell covers a stall of its length. A long stall comes back as
    // several queued ticks, each with its share of the silence; the block is added once a spell.
    const long dry = under > _under ? static_cast<long>(under - _under) : 0;
    _under = under;
    if (dry > 0) {
      if (_dry_frames == 0)
        ++_underruns;
      _dry_frames += dry;
      _target = std::min(_max_target, _target + dry + (_dry_frames == dry ? _block : 0));
      _changed_at = now;
    } else if (_dry_frames > 0) { // the spell is over
      std::fprintf(stderr,
                   "web audio: the speaker ran dry for %.0f ms (%llu times); it now plays %.0f ms "
                   "ahead\n",
                   ms(_dry_frames), static_cast<unsigned long long>(_underruns), ms(_target));
      _dry_frames = 0;
    } else if (_target > _min_target && now - _changed_at >= kRelaxS) {
      // Ten seconds without running dry: a block nearer, the ring skipping it cross-faded.
      const long step = std::min(_block, _target - _min_target);
      _target -= step;
      _changed_at = now;
      cvc_wa_send(_id, 1, static_cast<int>(step));
    }
    if (!_fill)
      return;
    const double ahead = _posted - _read;
    long want = _target - static_cast<long>(ahead);
    if (want <= 0)
      return;
    if (want > _max_target)
      want = _max_target;
    _buf.resize(static_cast<std::size_t>(want) * static_cast<std::size_t>(_ch));
    _fill(_buf.data(), static_cast<std::size_t>(want));
    post(_buf.data(), static_cast<std::size_t>(want));
  }

private:
  static constexpr double kRelaxS = 10.0;

  void post(const float *pcm, std::size_t frames) {
    if (cvc_wa_post(_id, pcm, static_cast<int>(frames * static_cast<std::size_t>(_ch))))
      _posted += static_cast<double>(frames);
  }
  double ms(long frames) const { return 1000.0 * static_cast<double>(frames) / _rate; }

  int _rate, _ch;
  cvc::media::playback::audio_pull_fn _fill;
  long _block;
  long _min_target = 0, _target = 0, _max_target = 0; // frames
  int _id = 0;
  double _posted = 0.0, _read = 0.0, _under = 0.0; // frames, as the node counts them
  unsigned long long _underruns = 0;               // dry spells
  long _dry_frames = 0;                            // the silence played in this dry spell, so far
  double _changed_at = 0.0; // steady_s() when the target last changed or the ring last ran dry
  std::vector<float> _buf;
};

} // namespace
#endif // __EMSCRIPTEN__

bool prepare() {
#ifdef __EMSCRIPTEN__
  const char *mode = std::getenv("CVC_WEB_AUDIO");
  const bool off = mode && std::strcmp(mode, "sdl") == 0;
  const int st = cvc_wa_prepare(cvc_audio_worklet_js(), off ? 1 : 0);
  return st == 1 || st == 2;
#else
  return false;
#endif
}

bool ready() {
#ifdef __EMSCRIPTEN__
  return cvc_wa_state() == 2;
#else
  return false;
#endif
}

#ifdef __EMSCRIPTEN__
namespace {
// Loaded, or loading: a node opened now starts once they have loaded.
bool usable() {
  const int st = cvc_wa_state();
  return st == 1 || st == 2;
}
} // namespace
#endif

std::string status() {
#ifdef __EMSCRIPTEN__
  char buf[256];
  cvc_wa_status(buf, static_cast<int>(sizeof buf));
  return buf;
#else
  return "native";
#endif
}

std::unique_ptr<cvc::ariadne::stream::frame_source> open_capture(int rate, int channels,
                                                                 int frames_per_chunk) {
#ifdef __EMSCRIPTEN__
  if (!usable() || rate != cvc_wa_rate() || channels < 1 || channels > 2 || frames_per_chunk < 1)
    return nullptr;
  auto src = std::make_unique<capture_source>(rate, channels, frames_per_chunk);
  if (!src->opened())
    return nullptr;
  return src;
#else
  (void)rate;
  (void)channels;
  (void)frames_per_chunk;
  return nullptr;
#endif
}

std::unique_ptr<cvc::media::playback::audio_output>
open_output_pull(int rate, int channels, const cvc::media::playback::audio_pull_fn &fill) {
#ifdef __EMSCRIPTEN__
  if (!fill || !usable() || rate != cvc_wa_rate() || channels < 1 || channels > 2)
    return nullptr;
  auto out = std::make_unique<worklet_output>(rate, channels, fill);
  if (!out->opened())
    return nullptr;
  return out;
#else
  (void)rate;
  (void)channels;
  (void)fill;
  return nullptr;
#endif
}

audio_clock::audio_clock(std::function<void()> tick) : _tick(std::move(tick)) {
#ifdef __EMSCRIPTEN__
  prepare(); // the page's engine, where the clocks are listed
  cvc_wa_add_clock(this);
#endif
}

audio_clock::~audio_clock() {
#ifdef __EMSCRIPTEN__
  cvc_wa_remove_clock(this);
#endif
}

void audio_clock::run() {
  if (_running || !_tick)
    return;
  _running = true;
  try {
    _tick();
  } catch (...) {
  }
  _running = false;
}

} // namespace web_audio
} // namespace media
} // namespace cvc

#ifdef __EMSCRIPTEN__
// The page's calls (cvc_wa_open_capture, cvc_wa_open_playback, Module.cvcAudio.fire): the C++
// object a node was opened for, or a clock. Never throws into the page.

extern "C" EMSCRIPTEN_KEEPALIVE float *cvc_wa_capture_buffer(void *src, int n) {
  return static_cast<cvc::media::web_audio::capture_source *>(src)->buffer(n);
}

extern "C" EMSCRIPTEN_KEEPALIVE void cvc_wa_capture_push(void *src, int n) {
  try {
    static_cast<cvc::media::web_audio::capture_source *>(src)->push(n);
  } catch (...) {
  }
}

extern "C" EMSCRIPTEN_KEEPALIVE void cvc_wa_play_tick(void *sink, double read, double under) {
  try {
    static_cast<cvc::media::web_audio::worklet_output *>(sink)->tick(read, under);
  } catch (...) {
  }
}

extern "C" EMSCRIPTEN_KEEPALIVE void cvc_wa_clock(void *c) {
  static_cast<cvc::media::web_audio::audio_clock *>(c)->run();
}
#endif
