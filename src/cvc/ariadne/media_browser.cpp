// Ariadne media players in a browser (docs/ARIADNE.md §5.13; design: docs/roadmap/ARIADNE_MEDIA.md
// §4, M6): the page's own <video> and <audio> elements decode and play; FFmpeg is never compiled to
// wasm. See register_browser_player in inc/cvc/ariadne/media_player.h.
//
// Each player is a hidden element the page keeps (Module.cvcMedia[id]). The element plays the sound
// itself, through the browser's output. A video's pictures are drawn, at most once per new frame,
// into a canvas the size of the widget's box, and read back to the widget, which publishes them as
// any picture (an ImGui texture in arigl-web, terminal cells in ari.html). The tradeoff, against a
// <video> laid over the canvas: one copy per frame shown (about 1-3 ms at 480x270), for one path
// that works in both pages, under windows and menus, clipped and scaled like every other widget.
//
// How a frame is read back, in order, the first that gives a picture:
//   - "2d": a 2D canvas kept in memory (willReadFrequently), drawImage and getImageData;
//   - "gl": WebGL (one context for the page's players), texImage2D and readPixels: the GPU's own
//     copy of a hardware-decoded frame;
//   - "gpu": a GPU-backed 2D canvas (no willReadFrequently).
// A frame that reads back empty (nothing drawn) or all one colour (black; the green of a zeroed
// frame) is tried on the others, at most twice a second, and the first to give a picture is kept.
// A phone's hardware decoder can hand out frames that one way reads as black. Until a video has
// shown a picture its blank frames are not shown: the box keeps its placeholder and says why
// (Player::notice). Module.cvcMediaPaths, set before a player opens, is another order (["gl"]
// alone, to try one).
//
// Where the source comes from:
//   - an http(s) URL with no auth: the element loads it itself (it streams, and seeks with range
//     requests), once the host's policy allows its origin; crossOrigin is anonymous, so a picture
//     from another origin needs CORS to be read back (the sound plays either way);
//   - an http(s) URL with auth: NAME: the page fetches it with the credential's bearer token (only
//     for an origin it is bound to, never following a redirect) into a blob the element plays, so
//     the token goes to that request only, never into state or the element's URL;
//   - file://, cvc://, state://: read through the resolver (the page's in-memory files), into a
//     blob.
// A page's Content-Security-Policy must allow its media: media-src 'self' blob: (and the media's
// origin when it is another).

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cvc/ariadne/media.h>
#include <cvc/ariadne/media_player.h>
#include <cvc/ariadne/net_host.h>
#include <cvc/net/credentials.h>
#include <cvc/net/net_policy.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

namespace cvc {
namespace ariadne {
namespace media {

namespace {

// A source that cannot be played at all (refused by the policy, no credential): Failed from the
// start, saying why.
class FailedPlayer : public Player {
public:
  explicit FailedPlayer(std::string why) : why_(std::move(why)) {}
  void play() override {}
  void pause() override {}
  void seek(double) override {}
  void set_muted(bool) override {}
  PlayerState state() const override { return PlayerState::Failed; }
  double position() const override { return 0.0; }
  double duration() const override { return 0.0; }
  std::string error() const override { return why_; }
  bool frame(cvc::image &, long &) override { return false; }

private:
  std::string why_;
};

std::string lower_ext(const std::string &uri) {
  std::string path = uri.substr(0, uri.find_first_of("?#"));
  const std::size_t dot = path.rfind('.');
  if (dot == std::string::npos || path.find('/', dot) != std::string::npos)
    return std::string();
  std::string e = path.substr(dot + 1);
  for (char &c : e)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return e;
}

// The MIME type a blob of `uri`'s bytes plays as ("" lets the browser sniff).
std::string mime_for(const std::string &uri, bool video) {
  const std::string e = lower_ext(uri);
  if (e == "webm")
    return video ? "video/webm" : "audio/webm";
  if (e == "mp4" || e == "m4v")
    return "video/mp4";
  if (e == "m4a")
    return "audio/mp4";
  if (e == "ogg" || e == "ogv" || e == "oga" || e == "opus")
    return video && e != "opus" && e != "oga" ? "video/ogg" : "audio/ogg";
  if (e == "mp3")
    return "audio/mpeg";
  if (e == "wav")
    return "audio/wav";
  if (e == "flac")
    return "audio/flac";
  return std::string();
}

} // namespace

#ifdef __EMSCRIPTEN__

// clang-format off
EM_JS(int, cvc_media_open, (int id, int video, const char *url, const char *token,
                            const unsigned char *bytes, int len, const char *mime, int muted,
                            int loop), {
  try {
    var el = document.createElement(video ? 'video' : 'audio');
    el.preload = 'auto';
    el.muted = !!muted;
    el.loop = !!loop;
    el.playsInline = true;
    // In the document (a browser may not decode a detached video) and in the viewport (a phone may
    // not start or decode one far outside it), where no one sees it: 2 px in the top-left corner,
    // under the page (the canvas covers it) and all but transparent.
    el.style.position = 'fixed';
    el.style.left = '0';
    el.style.top = '0';
    el.style.width = '2px';
    el.style.height = '2px';
    el.style.opacity = '0.01';
    el.style.zIndex = '-1';
    el.style.pointerEvents = 'none';
    el.setAttribute('aria-hidden', 'true');
    document.body.appendChild(el);
    var order = Module.cvcMediaPaths;
    var st = {el: el, frame: 0, key: '', blob: null, err: '', started: false, blocked: false,
              idle: false, fetched: false, opened: performance.now(), tainted: false,
              paths: Array.isArray(order) && order.length ? order.slice() : ['2d', 'gl', 'gpu'],
              path: 0, canvases: {}, buf: null, good: 0, blank: 0, probed: -1e9, tried: ''};
    Module.cvcMedia = Module.cvcMedia || {};
    Module.cvcMedia[id] = st;
    el.addEventListener('error', function() {
      st.err = (el.error && el.error.message) || 'the browser cannot play it';
    });
    // Loading stopped before anything came (preload none: a phone on Data Saver, a low-end one):
    // the element waits for play(), so the player is Ready, and autoplay starts it. (A small file
    // fetched whole is suspended before its metadata too, but after a progress event.)
    el.addEventListener('progress', function() { st.fetched = true; });
    el.addEventListener('suspend', function() {
      if (el.readyState < 1 && el.networkState == 1 && !st.fetched) st.idle = true;
    });
    st.play = function() {
      var p = el.play();
      if (p && p.catch)
        p.catch(function(e) {
          if (e && e.name == 'NotAllowedError') st.blocked = true; // no tap on the page yet
          else if (e && e.name != 'AbortError') st.err = String((e && e.message) || e);
        });
    };
    // Refused for want of a tap (sound, or a browser that starts no video by itself): the next tap
    // or key anywhere on the page plays it, within that gesture.
    st.unlock = function() {
      if (!st.blocked || (navigator.userActivation && !navigator.userActivation.isActive)) return;
      st.blocked = false;
      st.play();
    };
    st.gestures = ['pointerup', 'touchend', 'keydown', 'click'];
    st.gestures.forEach(function(t) { document.addEventListener(t, st.unlock, true); });
    if (video && el.requestVideoFrameCallback) {
      var tick = function() { st.frame++; el.requestVideoFrameCallback(tick); };
      el.requestVideoFrameCallback(tick);
    }
    var u = UTF8ToString(url);
    var type = UTF8ToString(mime);
    var play_blob = function(b) {
      if (!Module.cvcMedia[id]) return;
      st.blob = URL.createObjectURL(b);
      el.src = st.blob;
    };
    if (len > 0) {
      play_blob(new Blob([HEAPU8.slice(bytes, bytes + len)], type ? {type: type} : {}));
    } else if (token) {
      // The credential goes to this request alone: no redirect, no cookies, the reply a blob.
      fetch(u, {headers: {'Authorization': 'Bearer ' + UTF8ToString(token)},
                credentials: 'omit', redirect: 'error', cache: 'no-store'})
        .then(function(r) {
          if (!r.ok) throw new Error('http ' + r.status);
          return r.blob();
        })
        .then(play_blob)
        .catch(function(e) { st.err = String((e && e.message) || e); });
    } else {
      el.crossOrigin = 'anonymous';
      el.src = u;
    }
    return 1;
  } catch (e) {
    return 0;
  }
});

EM_JS(void, cvc_media_close, (int id), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  if (!st) return;
  delete Module.cvcMedia[id];
  st.gestures.forEach(function(t) { document.removeEventListener(t, st.unlock, true); });
  try {
    st.el.pause();
    st.el.removeAttribute('src');
    st.el.load();
    if (st.el.parentNode) st.el.parentNode.removeChild(st.el);
  } catch (e) {}
  if (st.blob) URL.revokeObjectURL(st.blob);
});

EM_JS(void, cvc_media_play, (int id), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  if (!st) return;
  st.started = true;
  st.blocked = false;
  st.play();
});

EM_JS(void, cvc_media_pause, (int id), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  if (!st) return;
  st.blocked = false; // not to be played by the next tap now
  st.el.pause();
});

EM_JS(void, cvc_media_seek, (int id, double t), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  if (st) { try { st.el.currentTime = t; } catch (e) {} }
});

EM_JS(void, cvc_media_mute, (int id, int muted), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  if (st) st.el.muted = !!muted;
});

EM_JS(void, cvc_media_loop, (int id, int loop), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  if (st) st.el.loop = !!loop;
});

// PlayerState's order: Idle 0, Loading 1, Ready 2, Playing 3, Paused 4, Ended 5, Failed 6.
EM_JS(int, cvc_media_state, (int id), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  if (!st) return 6;
  var el = st.el;
  if (st.err) return 6;
  if (el.readyState < 1) return st.idle && !st.started ? 2 : 1;
  if (el.ended && !el.loop) return 5;
  if (!el.paused) return 3;
  return st.started ? 4 : 2;
});

EM_JS(double, cvc_media_time, (int id, int which), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  if (!st) return 0;
  var v = which ? st.el.duration : st.el.currentTime;
  return isFinite(v) ? v : 0;
});

EM_JS(char *, cvc_media_error, (int id), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  var s = !st ? 'closed' : st.err ? st.err
        : st.blocked ? 'the browser plays sound only after a click on the page: press Play'
        : st.tainted ? 'the video comes from another origin without CORS: its pictures cannot be shown'
        : '';
  var n = lengthBytesUTF8(s) + 1;
  var p = _malloc(n);
  stringToUTF8(s, p, n);
  return p;
});

// What keeps the picture away though nothing failed, in a few words for the box ("" for nothing).
EM_JS(char *, cvc_media_notice, (int id), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  var s = '';
  if (st && !st.err) {
    var el = st.el;
    if (st.blocked) s = 'tap to start it';
    else if (st.tainted) s = 'no CORS: its pictures cannot be read';
    else if (el.tagName == 'VIDEO' && !st.good && performance.now() - st.opened > 3000) {
      if (st.blank)
        s = 'its frames come out ' +
            (st.blank == 1 ? 'empty'
             : st.flat[0] + st.flat[1] + st.flat[2] <= 12 ? 'black'
             : 'all rgb(' + st.flat.join(',') + ')') + ' (' + st.tried + ')';
      else if (!el.paused && el.readyState < 2)
        s = 'playing, no picture yet (readyState ' + el.readyState + ')';
    }
  }
  var n = lengthBytesUTF8(s) + 1;
  var p = _malloc(n);
  stringToUTF8(s, p, n);
  return p;
});

// A new picture, scaled into the bw x bh box (never up), as RGBA into `out`; its size into wh.
EM_JS(int, cvc_media_frame, (int id, int bw, int bh, unsigned char *out, int *wh), {
  var st = Module.cvcMedia && Module.cvcMedia[id];
  if (!st || st.el.tagName != 'VIDEO' || st.tainted) return 0;
  var el = st.el;
  if (el.readyState < 2 || !el.videoWidth) return 0;
  var key = st.frame + ':' + el.currentTime;
  if (key == st.key) return 0;
  st.key = key;
  var k = Math.min(bw / el.videoWidth, bh / el.videoHeight, 1);
  var w = Math.max(1, Math.floor(el.videoWidth * k)), h = Math.max(1, Math.floor(el.videoHeight * k));
  // A 2D canvas: kept in memory (willReadFrequently), or GPU-backed where the browser can.
  var draw_2d = function(gpu) {
    var c = st.canvases[gpu ? 'gpu' : '2d'];
    if (!c || c.canvas.width != w || c.canvas.height != h) {
      var cv = !gpu && typeof OffscreenCanvas != 'undefined' ? new OffscreenCanvas(w, h)
                                                             : document.createElement('canvas');
      cv.width = w;
      cv.height = h;
      c = cv.getContext('2d', gpu ? {} : {willReadFrequently: true});
      if (!c) return null;
      st.canvases[gpu ? 'gpu' : '2d'] = c;
    }
    c.clearRect(0, 0, w, h); // a draw that does nothing reads back empty, not the last frame
    c.drawImage(el, 0, 0, w, h);
    return c.getImageData(0, 0, w, h).data;
  };
  // WebGL: the frame as a texture, drawn into a w x h viewport and read back. One context for the
  // page's players (a page has few to spare), made the first time a player needs it.
  var draw_gl = function() {
    if (Module.cvcMediaGL === undefined) {
      Module.cvcMediaGL = null;
      var cv = typeof OffscreenCanvas != 'undefined' ? new OffscreenCanvas(1, 1)
                                                     : document.createElement('canvas');
      var gl = cv.getContext('webgl', {alpha: true, premultipliedAlpha: false, antialias: false,
                                       depth: false, stencil: false});
      if (!gl) return null;
      var shader = function(type, src) {
        var s = gl.createShader(type);
        gl.shaderSource(s, src);
        gl.compileShader(s);
        return s;
      };
      var prog = gl.createProgram();
      // Texture row 0 is the frame's top, and readPixels gives the bottom row first: the quad's
      // bottom samples the top, so the rows come back top first.
      gl.attachShader(prog, shader(gl.VERTEX_SHADER,
          'attribute vec2 a; varying vec2 t;' +
          'void main() { t = a * 0.5 + 0.5; gl_Position = vec4(a, 0.0, 1.0); }'));
      gl.attachShader(prog, shader(gl.FRAGMENT_SHADER,
          'precision mediump float; varying vec2 t; uniform sampler2D s;' +
          'void main() { gl_FragColor = texture2D(s, t); }'));
      gl.bindAttribLocation(prog, 0, 'a');
      gl.linkProgram(prog);
      if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) return null;
      gl.useProgram(prog);
      gl.bindBuffer(gl.ARRAY_BUFFER, gl.createBuffer());
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]),
                    gl.STATIC_DRAW);
      gl.enableVertexAttribArray(0);
      gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
      gl.bindTexture(gl.TEXTURE_2D, gl.createTexture());
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
      Module.cvcMediaGL = gl;
    }
    var g = Module.cvcMediaGL;
    if (!g || g.isContextLost()) return null;
    if (g.canvas.width != w || g.canvas.height != h) {
      g.canvas.width = w;
      g.canvas.height = h;
    }
    g.viewport(0, 0, w, h);
    g.texImage2D(g.TEXTURE_2D, 0, g.RGBA, g.RGBA, g.UNSIGNED_BYTE, el);
    g.clearColor(0, 0, 0, 0);
    g.clear(g.COLOR_BUFFER_BIT);
    g.drawArrays(g.TRIANGLE_STRIP, 0, 4);
    if (!st.buf || st.buf.length != w * h * 4) st.buf = new Uint8Array(w * h * 4);
    g.readPixels(0, 0, w, h, g.RGBA, g.UNSIGNED_BYTE, st.buf);
    return st.buf;
  };
  // A path's pixels, or null where it cannot draw here. Another origin without CORS throws.
  var draw = function(path) {
    try {
      if (path == 'gl') return draw_gl();
      return path == '2d' || path == 'gpu' ? draw_2d(path == 'gpu') : null;
    } catch (e) {
      if (e && e.name == 'SecurityError') throw e;
      return null;
    }
  };
  // 0: a picture; 1: empty (nothing drawn); 2: all one colour, st.flat (black, or the green of a
  // zeroed frame). Every 3rd row and 7th column, a pixel in from the edges: through WebGL a
  // decoder's padding can tint the outer column of an all-black frame.
  var blank = function(d) {
    if (!d) return 1;
    var x0 = w > 2 ? 1 : 0, y0 = h > 2 ? 1 : 0, c = null;
    for (var y = y0; y < h - y0; y += 3) {
      for (var x = x0 + y % 7; x < w - x0; x += 7) {
        var i = (y * w + x) * 4;
        if (!d[i + 3]) continue;
        if (!c) c = [d[i], d[i + 1], d[i + 2]];
        else if (Math.abs(d[i] - c[0]) > 4 || Math.abs(d[i + 1] - c[1]) > 4 ||
                 Math.abs(d[i + 2] - c[2]) > 4) return 0;
      }
    }
    if (!c) return 1;
    st.flat = c;
    return 2;
  };
  var data, kind;
  try {
    data = draw(st.paths[st.path]);
    kind = blank(data);
    var now = performance.now();
    if (kind && now - st.probed >= 500) { // the others, twice a second at most
      st.probed = now;
      var tried = [st.paths[st.path] + (data ? '' : ' none')];
      for (var i = 0; i < st.paths.length; i++) {
        if (i == st.path) continue;
        var d = draw(st.paths[i]);
        tried.push(st.paths[i] + (d ? '' : ' none'));
        if (d && !blank(d)) {
          st.path = i;
          data = d;
          kind = 0;
          break;
        }
      }
      st.tried = tried.join(', ');
    }
  } catch (e) {
    st.tainted = true; // another origin without CORS: the sound plays, the pictures can't be read
    return 0;
  }
  if (kind) {
    st.blank = kind;
    // Never a picture yet: keep the placeholder, whose box says why (cvc_media_notice). After one,
    // a black frame is the video's own (a fade).
    if (!st.good || !data) return 0;
  } else {
    st.good++;
    st.blank = 0;
  }
  HEAPU8.set(data, out);
  setValue(wh, w, 'i32');
  setValue(wh + 4, h, 'i32');
  return 1;
});
// clang-format on

namespace {

class BrowserPlayer : public Player {
public:
  BrowserPlayer(const PlayerSource &src, const std::string &token, const std::string &bytes)
      : video_(src.video), bw_(std::max(1, src.frame_w)), bh_(std::max(1, src.frame_h)) {
    static int next_id = 0;
    id_ = ++next_id;
    if (video_)
      buf_.resize(static_cast<std::size_t>(bw_) * bh_ * 4);
    const std::string mime = mime_for(src.uri, src.video);
    open_ = cvc_media_open(id_, video_ ? 1 : 0, src.uri.c_str(),
                           token.empty() ? nullptr : token.c_str(),
                           reinterpret_cast<const unsigned char *>(bytes.data()),
                           static_cast<int>(bytes.size()), mime.c_str(),
                           src.muted || !src.audio ? 1 : 0, src.loop ? 1 : 0) != 0;
  }
  ~BrowserPlayer() override { cvc_media_close(id_); }
  void play() override { cvc_media_play(id_); }
  void pause() override { cvc_media_pause(id_); }
  void seek(double t) override { cvc_media_seek(id_, t); }
  void set_muted(bool m) override { cvc_media_mute(id_, m ? 1 : 0); }
  void set_loop(bool on) override { cvc_media_loop(id_, on ? 1 : 0); }
  PlayerState state() const override {
    return open_ ? static_cast<PlayerState>(cvc_media_state(id_)) : PlayerState::Failed;
  }
  double position() const override { return cvc_media_time(id_, 0); }
  double duration() const override { return cvc_media_time(id_, 1); }
  std::string error() const override {
    if (!open_)
      return "the page could not make a media element";
    char *s = cvc_media_error(id_);
    std::string out = s ? s : "";
    free(s);
    return out;
  }
  std::string notice() const override {
    if (!open_)
      return std::string();
    char *s = cvc_media_notice(id_);
    std::string out = s ? s : "";
    free(s);
    return out;
  }
  void set_frame_box(int w, int h) override {
    bw_ = std::max(1, w);
    bh_ = std::max(1, h);
    if (video_)
      buf_.resize(static_cast<std::size_t>(bw_) * bh_ * 4);
  }
  bool frame(cvc::image &out, long &gen) override {
    if (!video_)
      return false;
    int wh[2] = {0, 0};
    if (!cvc_media_frame(id_, bw_, bh_, buf_.data(), wh) || wh[0] <= 0 || wh[1] <= 0)
      return false;
    out = cvc::image(wh[0], wh[1], cvc::image::pixel_format::RGBA, cvc::image::data_type::u8,
                     buf_.data());
    gen = ++gen_;
    return true;
  }

private:
  int id_ = 0;
  bool video_;
  int bw_, bh_;
  bool open_ = false;
  std::vector<unsigned char> buf_;
  long gen_ = 0;
};

} // namespace

#endif // __EMSCRIPTEN__

namespace {

// The browser player for `src`: the policy and the credential decided here, in C++, where the
// host keeps them; the element only ever sees an allowed URL, or one request's token, or bytes.
std::unique_ptr<Player> make_browser_player(const PlayerSource &src) {
  if (!src.app)
    return std::make_unique<FailedPlayer>("no app to fetch it with");
  const bool http = src.uri.rfind("http://", 0) == 0 || src.uri.rfind("https://", 0) == 0;
  std::string token, bytes;
  if (http) {
    // The widget's principal's allowlist and credentials (docs/roadmap/ARIADNE_SHELL.md 4.12).
    const std::optional<NetAccess> access = net_host_access(*src.app, src.scope);
    if (!access)
      return std::make_unique<FailedPlayer>("the host has the network off");
    std::string why;
    const std::optional<cvc::net::Origin> origin = cvc::net::parse_origin(src.uri, &why);
    if (!origin)
      return std::make_unique<FailedPlayer>("http: " + why);
    if (access->policy && !access->policy->allows(*origin))
      return std::make_unique<FailedPlayer>("http: refused: " + origin->str() +
                                            " is not an origin this host allows");
    if (!src.auth.empty()) {
      const std::optional<std::string> t =
          access->credentials ? access->credentials->token_for(src.auth, *origin) : std::nullopt;
      if (!t)
        return std::make_unique<FailedPlayer>("http: no credential '" + src.auth + "' for " +
                                              origin->str() + " (sign in first)");
      token = *t;
    }
  } else {
    if (!src.auth.empty())
      return std::make_unique<FailedPlayer>("auth: is for http(s) URLs only");
    FetchResult r = fetch_bytes(*src.app, src.uri, std::string(), src.base); // local: page files
    if (!r.ok)
      return std::make_unique<FailedPlayer>(r.error);
    bytes = std::move(r.bytes);
  }
#ifdef __EMSCRIPTEN__
  std::unique_ptr<Player> p = std::make_unique<BrowserPlayer>(src, token, bytes);
  std::fill(token.begin(), token.end(), '\0');
  return p;
#else
  std::fill(token.begin(), token.end(), '\0');
  return std::make_unique<FailedPlayer>("not a browser");
#endif
}

} // namespace

bool register_browser_player(cvc::app &app) {
#ifdef __EMSCRIPTEN__
  register_player_factory(app, make_browser_player);
  return true;
#else
  (void)app;
  (void)&make_browser_player;
  return false;
#endif
}

} // namespace media
} // namespace ariadne
} // namespace cvc
