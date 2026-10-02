// clang-format off: hand-formatted JS; .clang-format is the C++ style (CI formats only C++).
// webgl_state_shadow.js -- emscripten --pre-js: answer GL state queries from a client-side
// shadow instead of a synchronous round-trip to the browser's GPU process.
//
// Part of cvcGL. A wasm app links it with cvcgl_wasm_app(<target>) (cvcGLWasm.cmake, installed
// beside cvcGLConfig.cmake; the file itself installs to share/cvcGL/wasm/). See
// docs/CVCGL_WASM.md for the app contract.
//
// Why: two per-frame sources of GL state queries in every cvcGL WebAssembly app.
//   * Dear ImGui's OpenGL3 backend (imgui_impl_opengl3.cpp, compiled into libcvcGL) backs up and
//     restores GL state around every ImGui_ImplOpenGL3_RenderDrawData() call, which
//     cvc::gl::ImGuiOverlay makes once per frame -- and Ariadne's ImGuiBackend draws through
//     ImGuiOverlay, so every cvcGL ImGui / Ariadne app pays it: ACTIVE_TEXTURE, VIEWPORT,
//     SCISSOR_BOX, the six BLEND_* values, isEnabled x5, isProgram.
//   * VTK re-reads READ_BUFFER after every read-framebuffer bind and MAX_DRAW_BUFFERS on every
//     framebuffer activation (vtkOpenGLState, vtkOpenGLFramebufferObject), several times a frame.
// In a browser, several of those queries are not cached by the WebGL implementation, so each one
// blocks until the GPU process drains its command queue: the first absorbs the frame's queued GPU
// work, and each later one adds another round trip. Every frame pays this, and the more GPU work a
// frame queues (a heavy per-frame overlay, a large scene), the longer that first query waits. The
// sync-call census (devtools/glsync.js) shows which queries still reach the browser: with the shim
// on, none of the shadowed ones should.
//
// Scope: it patches the WebGL prototypes, so it serves EVERY WebGL context on the page, cvcGL's
// canvas or not. The fuzz test (src/cvcGL/test/webgl_state_shadow_test.js) checks every answer
// against a mock WebGL with real GL/WebGL semantics, and ?glshim=verify compares each answer with
// the real value in a live app (the browser check in docs/CVCGL_WASM.md). A page that embeds a
// cvcGL module beside other WebGL code can still leave it out:
// cvcgl_wasm_app(<target> STATE_SHIM OFF) at build time, or ?glshim=0 per page load.
//
// What: every setter that can change a shadowed value is wrapped to keep a per-context
// copy current; the matching query is answered from that copy. The copy is seeded by one
// real query per value the first time a context is used, and dropped on context loss so it
// is re-seeded after restore (every wrapper passes straight through while a context is lost).
// Shadowed:
//   getParameter: SCISSOR_BOX, VIEWPORT, BLEND_{SRC,DST}_{RGB,ALPHA}, BLEND_EQUATION_{RGB,ALPHA},
//                 ACTIVE_TEXTURE (ImGui's first query of every frame, so in a browser that does
//                 not cache it -- Firefox -- the one that absorbs the whole frame's GPU work);
//                 WebGL2 also MAX_DRAW_BUFFERS and MAX_COLOR_ATTACHMENTS, implementation
//                 constants VTK re-reads per frame (on WebGL1 they depend on whether
//                 WEBGL_draw_buffers was enabled, so there they go to the real call);
//                 WebGL2 READ_BUFFER (below)
//   isEnabled:    BLEND, CULL_FACE, DEPTH_TEST, STENCIL_TEST, SCISSOR_TEST,
//                 POLYGON_OFFSET_FILL, SAMPLE_ALPHA_TO_COVERAGE, SAMPLE_COVERAGE, DITHER,
//                 RASTERIZER_DISCARD (WebGL2)
//   isProgram:    true for a program this context created and nobody has deleted; any program
//                 that was deleted (deferred-delete rules) or is foreign goes to the real call.
// READ_BUFFER is state of the bound READ framebuffer, so its shadow tracks the read binding
// (bindFramebuffer on FRAMEBUFFER / READ_FRAMEBUFFER, deleteFramebuffer of the bound one) and one
// value per framebuffer (readBuffer); a framebuffer this context did not create through the shim
// makes the binding UNKNOWN, and every READ_BUFFER query then goes to the real call until a known
// one is bound. VTK (vtkOpenGLState::vtkglBindFramebuffer) re-reads it after every read-framebuffer
// bind to seed its own cache, which then decides whether a later readBuffer is issued at all, so a
// wrong answer would blit from the wrong attachment; ?glshim=norb takes it back to the real call
// while keeping the rest, and ?glshim=verify audits it like every other shadowed value.
// Anything else -- including BLEND_COLOR, whose clamping differs by context version and
// extensions -- goes to the real implementation untouched. State changed through extension
// objects (e.g. OES_draw_buffers_indexed) is not tracked; emscripten does not enable those.
// The shim is a no-op when GL lives in a worker (OffscreenCanvas / PROXY_TO_PTHREAD).
//
// Modes:
//   on      (default) everything above
//   0 / off every call goes straight to WebGL (the A/B baseline); nothing is installed
//   norb    on, except READ_BUFFER (and its framebuffer tracking): the real call
//   verify  answer with the REAL value but compare it to the shadow, every shadowed value
//           included; mismatches are counted in window.__cvcGlShadow.stats (.mismatches, and per
//           value name in .mismatchBy) and the first few logged with that name, so the shadow can
//           be checked against the real values in a live app.
// The mode comes from the first of these that is set, so a URL can always A/B any page:
//   1. the URL:  ?glshim=0|off|on|verify|norb
//   2. the page: Module.glStateShadow, set by the host page before the module starts
//   3. the build: Module.glStateShadowDefault, written by cvcgl_wasm_app(... STATE_SHIM_MODE m)
//   4. on
// An unrecognized value (?glshim=of, Module.glStateShadow='no') means on, with a console.warn
// naming the accepted values -- so a mistyped A/B baseline does not silently measure the shim twice.
// window.__cvcGlShadow.stats records the mode, where it came from (.modeFrom: url / page / build /
// default) and .version, which names this copy: the first copy installed on a page wins, so a page
// that may load another copy can tell which one is active.
(function () {
  'use strict';
  if (typeof window === 'undefined' || typeof WebGLRenderingContext === 'undefined') return;
  if (window.__cvcGlShadow) return; // idempotent (several --pre-js / pages may include it)

  var VERSION = 'cvcGL-1';
  // Accepted: off = 0 / off / false (or boolean false); on = on / 1 / true / empty (a bare
  // ?glshim) (or boolean true); verify; norb -- any case. Anything else is still 'on', but says so:
  // a mistyped A/B baseline (?glshim=of, ?glshim=no) would otherwise measure the shim twice.
  function parseMode(v, from) {
    var t = String(v).toLowerCase();
    if (v === false || t === '0' || t === 'off' || t === 'false') return 'off';
    if (t === 'verify') return 'verify';
    if (t === 'norb') return 'norb';
    if (!(v === true || t === '' || t === 'on' || t === '1' || t === 'true')) {
      try {
        console.warn('webgl_state_shadow: unrecognized mode ' + JSON.stringify(String(v)) + ' from ' + from +
                     "; using 'on'. Accepted: 0|off|false, on|1|true, verify, norb");
      } catch (e) { /* no console */ }
    }
    return 'on';
  }
  var mode = 'on', modeFrom = 'default';
  var M = typeof Module !== 'undefined' && Module ? Module : null;
  // Bracket reads: the property names must survive a Closure-compiled module.
  if (M && M['glStateShadowDefault'] !== undefined) {
    mode = parseMode(M['glStateShadowDefault'], 'Module.glStateShadowDefault'); modeFrom = 'build';
  }
  if (M && M['glStateShadow'] !== undefined) { mode = parseMode(M['glStateShadow'], 'Module.glStateShadow'); modeFrom = 'page'; }
  try {
    var q = new URLSearchParams(window.location.search).get('glshim');
    if (q !== null) { mode = parseMode(q, '?glshim='); modeFrom = 'url'; }
  } catch (e) { /* no location: keep the page / build / default mode */ }

  var stats = { version: VERSION, mode: mode, modeFrom: modeFrom, served: 0, verified: 0, mismatches: 0,
                mismatchBy: {} };
  window.__cvcGlShadow = { stats: stats };
  if (mode === 'off') return;

  var G = typeof WebGL2RenderingContext !== 'undefined' ? WebGL2RenderingContext : WebGLRenderingContext;
  var PARAMS = [G.SCISSOR_BOX, G.VIEWPORT, G.BLEND_SRC_RGB, G.BLEND_DST_RGB,
                G.BLEND_SRC_ALPHA, G.BLEND_DST_ALPHA, G.BLEND_EQUATION_RGB, G.BLEND_EQUATION_ALPHA,
                G.ACTIVE_TEXTURE];
  // WebGL2-only enums, as literals: G is WebGLRenderingContext where there is no WebGL2.
  var MAX_DRAW_BUFFERS = 0x8824, MAX_COLOR_ATTACHMENTS = 0x8CDF, READ_BUFFER = 0x0C02,
      FRAMEBUFFER = 0x8D40, READ_FRAMEBUFFER = 0x8CA8, READ_FRAMEBUFFER_BINDING = 0x8CAA,
      BACK = 0x0405, NONE = 0, COLOR_ATTACHMENT0 = 0x8CE0;
  var PARAMS2 = [MAX_DRAW_BUFFERS, MAX_COLOR_ATTACHMENTS]; // WebGL2 only: implementation constants
  var CAPS = [G.BLEND, G.CULL_FACE, G.DEPTH_TEST, G.STENCIL_TEST, G.SCISSOR_TEST,
              G.POLYGON_OFFSET_FILL, G.SAMPLE_ALPHA_TO_COVERAGE, G.SAMPLE_COVERAGE, G.DITHER];
  var CAPS2 = [G.RASTERIZER_DISCARD]; // WebGL2 only
  // READ_BUFFER (and the framebuffer tracking behind it) everywhere but ?glshim=norb.
  var rbTrack = mode !== 'norb';
  var UNKNOWN = {}; // read-framebuffer binding the shadow cannot vouch for: ask the real call
  // Value names for ?glshim=verify's report (stats.mismatchBy and the console), precomputed so the
  // served path builds no string.
  var NAMES = {};
  ['SCISSOR_BOX', 'VIEWPORT', 'BLEND_SRC_RGB', 'BLEND_DST_RGB', 'BLEND_SRC_ALPHA', 'BLEND_DST_ALPHA',
   'BLEND_EQUATION_RGB', 'BLEND_EQUATION_ALPHA', 'ACTIVE_TEXTURE'].forEach(function (n) { NAMES[G[n]] = n; });
  NAMES[MAX_DRAW_BUFFERS] = 'MAX_DRAW_BUFFERS'; NAMES[MAX_COLOR_ATTACHMENTS] = 'MAX_COLOR_ATTACHMENTS';
  var CAP_NAMES = {};
  ['BLEND', 'CULL_FACE', 'DEPTH_TEST', 'STENCIL_TEST', 'SCISSOR_TEST', 'POLYGON_OFFSET_FILL',
   'SAMPLE_ALPHA_TO_COVERAGE', 'SAMPLE_COVERAGE', 'DITHER', 'RASTERIZER_DISCARD'].forEach(function (n) {
    if (G[n] !== undefined) CAP_NAMES[G[n]] = 'isEnabled(' + n + ')';
  });
  // WebIDL GLenum/long conversions, so the shadow keys match what the browser stores.
  function u32(x) { return x >>> 0; }
  function i32(x) { return x | 0; }

  function install(proto, isGL2) {
    if (!proto || proto.__cvcGlShadowed) return;
    proto.__cvcGlShadowed = true;
    var real = {};
    ['getParameter', 'isEnabled', 'isProgram', 'createProgram', 'deleteProgram', 'scissor',
     'viewport', 'blendFunc', 'blendFuncSeparate', 'blendEquation', 'blendEquationSeparate',
     'enable', 'disable', 'activeTexture', 'createFramebuffer', 'deleteFramebuffer',
     'bindFramebuffer', 'readBuffer'].forEach(function (n) { real[n] = proto[n]; });
    var caps = isGL2 ? CAPS.concat(CAPS2) : CAPS;
    var params = isGL2 ? PARAMS.concat(PARAMS2) : PARAMS;
    var rb = rbTrack && isGL2 && typeof real.readBuffer === 'function';
    var shadows = new WeakMap(); // context -> shadow

    function shadowOf(gl) {
      var s = shadows.get(gl);
      if (s) return s;
      s = { p: {}, en: {}, live: new WeakSet(), deleted: new WeakSet(),
            maxVp: real.getParameter.call(gl, G.MAX_VIEWPORT_DIMS),
            maxTex: real.getParameter.call(gl, G.MAX_COMBINED_TEXTURE_IMAGE_UNITS) };
      params.forEach(function (k) { s.p[k] = real.getParameter.call(gl, k); });
      caps.forEach(function (k) { s.en[k] = real.isEnabled.call(gl, k); });
      if (rb) {
        // Framebuffers this context created (through the shim) and deleted, the read buffer of
        // each (undefined = not known yet: read it on the next query) and of the default
        // framebuffer, and the read binding -- seeded from the real binding, which can only be a
        // framebuffer the shim saw created if it is not null, else it is UNKNOWN.
        s.fbLive = new WeakSet(); s.fbDeleted = new WeakSet(); s.rbOf = new WeakMap();
        s.rbDefault = undefined;
        var bound = real.getParameter.call(gl, READ_FRAMEBUFFER_BINDING);
        s.readFb = bound === null ? null : UNKNOWN;
      }
      shadows.set(gl, s);
      var c = gl.canvas;
      if (c && c.addEventListener && !c.__cvcGlShadowLoss) {
        c.__cvcGlShadowLoss = true;
        // Drop on LOSS only: every wrapper passes through while lost, so the first use after
        // restore re-seeds -- and programs an app's own 'restored' listener creates (which may
        // run before any listener of ours) land in that fresh shadow instead of being dropped.
        c.addEventListener('webglcontextlost', function () { shadows.delete(gl); }, false);
      }
      return s;
    }
    function copy(v) { return (v && v.slice) ? v.slice() : v; } // Int32Array -> fresh copy
    function same(a, b) {
      if (a && a.length !== undefined && b && b.length !== undefined) {
        if (a.length !== b.length) return false;
        for (var i = 0; i < a.length; ++i) if (!Object.is(a[i], b[i])) return false;
        return true;
      }
      return Object.is(a, b);
    }
    // `name` keys stats.mismatchBy; `detail` (optional) only goes to the console.
    function answer(gl, shadowVal, realFn, args, name, detail) {
      if (mode === 'verify') {
        var r = realFn.apply(gl, args);
        stats.verified++;
        if (!same(r, shadowVal)) {
          stats.mismatches++;
          stats.mismatchBy[name] = (stats.mismatchBy[name] || 0) + 1;
          if (stats.mismatches <= 8)
            console.warn('webgl_state_shadow mismatch', name + (detail ? ' (' + detail + ')' : ''),
                         'real=', r, 'shadow=', shadowVal);
        }
        return r;
      }
      stats.served++;
      return copy(shadowVal);
    }
    var hasParam = {}; params.forEach(function (k) { hasParam[k] = true; });
    var hasCap = {}; caps.forEach(function (k) { hasCap[k] = true; });

    proto.getParameter = function (pname) {
      if (arguments.length === 1 && hasParam[pname] && !this.isContextLost()) {
        var s = shadowOf(this), k = u32(pname);
        if (s.p[k] === undefined) s.p[k] = real.getParameter.call(this, k); // re-seed after an invalidation
        return answer(this, s.p[k], real.getParameter, arguments, NAMES[k]);
      }
      if (rb && pname === READ_BUFFER && arguments.length === 1 && !this.isContextLost()) {
        var t = shadowOf(this), fb = t.readFb;
        if (fb !== UNKNOWN) {
          var v = fb === null ? t.rbDefault : t.rbOf.get(fb);
          if (v === undefined) { // not known yet for this framebuffer: the real answer, kept
            v = real.getParameter.call(this, READ_BUFFER);
            if (fb === null) t.rbDefault = v; else t.rbOf.set(fb, v);
            return v;
          }
          return answer(this, v, real.getParameter, arguments, 'READ_BUFFER',
                        fb === null ? 'default framebuffer' : 'framebuffer object');
        }
      }
      return real.getParameter.apply(this, arguments);
    };
    proto.isEnabled = function (cap) {
      if (arguments.length === 1 && hasCap[cap] && !this.isContextLost()) {
        var k = u32(cap);
        return answer(this, shadowOf(this).en[k], real.isEnabled, arguments, CAP_NAMES[k]);
      }
      return real.isEnabled.apply(this, arguments);
    };
    proto.isProgram = function (p) {
      if (arguments.length === 1 && p && !this.isContextLost()) {
        var s = shadowOf(this);
        if (s.live.has(p) && !s.deleted.has(p))
          return answer(this, true, real.isProgram, arguments, 'isProgram');
      }
      return real.isProgram.apply(this, arguments); // null, deleted, foreign, lost: the real answer
    };
    proto.createProgram = function () {
      var p = real.createProgram.apply(this, arguments);
      if (p && !this.isContextLost()) shadowOf(this).live.add(p);
      return p;
    };
    proto.deleteProgram = function (p) {
      var r = real.deleteProgram.apply(this, arguments);
      if (p && !this.isContextLost()) shadowOf(this).deleted.add(p); // GL's deferred-delete rules
      return r;                                                      // stay with the real isProgram
    };

    function setter(name, update) {
      var f = real[name];
      proto[name] = function () {
        var r = f.apply(this, arguments);
        if (!this.isContextLost()) update(shadowOf(this), arguments);
        return r;
      };
    }
    // Setters run the real call FIRST (an exception from WebIDL conversion propagates before the
    // shadow changes). A call GL rejects (bad enum, negative size, WebGL's constant-colour/
    // constant-alpha pairing rule) leaves GL state unchanged; the shadow cannot see the error
    // without a synchronous getError(), so it only commits values it can prove valid and
    // otherwise INVALIDATES the affected entries -- the next query re-reads them.
    var FACTORS = {}; [G.ZERO, G.ONE, G.SRC_COLOR, G.ONE_MINUS_SRC_COLOR, G.DST_COLOR, G.ONE_MINUS_DST_COLOR,
      G.SRC_ALPHA, G.ONE_MINUS_SRC_ALPHA, G.DST_ALPHA, G.ONE_MINUS_DST_ALPHA, G.CONSTANT_COLOR,
      G.ONE_MINUS_CONSTANT_COLOR, G.CONSTANT_ALPHA, G.ONE_MINUS_CONSTANT_ALPHA, G.SRC_ALPHA_SATURATE
    ].forEach(function (k) { FACTORS[k] = true; });
    var EQS = {}; [G.FUNC_ADD, G.FUNC_SUBTRACT, G.FUNC_REVERSE_SUBTRACT].forEach(function (k) { EQS[k] = true; });
    if (isGL2) { EQS[G.MIN] = true; EQS[G.MAX] = true; } // WebGL1 needs EXT_blend_minmax: not assumed
    var CC = {}; CC[G.CONSTANT_COLOR] = CC[G.ONE_MINUS_CONSTANT_COLOR] = 1;
    CC[G.CONSTANT_ALPHA] = CC[G.ONE_MINUS_CONSTANT_ALPHA] = 2;
    function factorsOk(src, dst) {
      if (!FACTORS[src] || !FACTORS[dst]) return false;
      if (dst === G.SRC_ALPHA_SATURATE) return false; // src-only in ES2; not relied on either way
      return !(CC[src] && CC[dst] && CC[src] !== CC[dst]); // WebGL: INVALID_OPERATION
    }
    function drop(s, keys) { keys.forEach(function (k) { s.p[k] = undefined; }); }
    var BF = [G.BLEND_SRC_RGB, G.BLEND_DST_RGB, G.BLEND_SRC_ALPHA, G.BLEND_DST_ALPHA];
    var BE = [G.BLEND_EQUATION_RGB, G.BLEND_EQUATION_ALPHA];

    setter('scissor', function (s, a) {
      var w = i32(a[2]), h = i32(a[3]);
      if (w >= 0 && h >= 0) s.p[G.SCISSOR_BOX] = new Int32Array([i32(a[0]), i32(a[1]), w, h]);
    });
    setter('viewport', function (s, a) {
      var w = i32(a[2]), h = i32(a[3]);
      if (w < 0 || h < 0) return;
      // ANGLE clamps to MAX_VIEWPORT_DIMS on store, Firefox caches the raw size: beyond the
      // limit the answer is browser-specific, so re-read it instead of guessing.
      if (!s.maxVp || w > s.maxVp[0] || h > s.maxVp[1]) return drop(s, [G.VIEWPORT]);
      s.p[G.VIEWPORT] = new Int32Array([i32(a[0]), i32(a[1]), w, h]);
    });
    setter('blendFunc', function (s, a) {
      var sf = u32(a[0]), df = u32(a[1]);
      if (!factorsOk(sf, df)) return drop(s, BF);
      s.p[G.BLEND_SRC_RGB] = s.p[G.BLEND_SRC_ALPHA] = sf;
      s.p[G.BLEND_DST_RGB] = s.p[G.BLEND_DST_ALPHA] = df;
    });
    setter('blendFuncSeparate', function (s, a) {
      var sr = u32(a[0]), dr = u32(a[1]), sa = u32(a[2]), da = u32(a[3]);
      if (!factorsOk(sr, dr) || !factorsOk(sa, da) ||
          (CC[sr] && CC[da] && CC[sr] !== CC[da]) || (CC[sa] && CC[dr] && CC[sa] !== CC[dr]))
        return drop(s, BF); // stricter than the spec (conservative), never more permissive
      s.p[G.BLEND_SRC_RGB] = sr; s.p[G.BLEND_DST_RGB] = dr;
      s.p[G.BLEND_SRC_ALPHA] = sa; s.p[G.BLEND_DST_ALPHA] = da;
    });
    setter('blendEquation', function (s, a) {
      var m = u32(a[0]);
      if (!EQS[m]) return drop(s, BE);
      s.p[G.BLEND_EQUATION_RGB] = s.p[G.BLEND_EQUATION_ALPHA] = m;
    });
    setter('blendEquationSeparate', function (s, a) {
      var mr = u32(a[0]), ma = u32(a[1]);
      if (!EQS[mr] || !EQS[ma]) return drop(s, BE);
      s.p[G.BLEND_EQUATION_RGB] = mr; s.p[G.BLEND_EQUATION_ALPHA] = ma;
    });
    setter('enable', function (s, a) { var c = u32(a[0]); if (hasCap[c]) s.en[c] = true; });
    setter('disable', function (s, a) { var c = u32(a[0]); if (hasCap[c]) s.en[c] = false; });
    // TEXTURE0 .. TEXTURE0 + MAX_COMBINED_TEXTURE_IMAGE_UNITS - 1, else INVALID_ENUM (no change).
    setter('activeTexture', function (s, a) {
      var t = u32(a[0]);
      if (s.maxTex > 0 && t >= G.TEXTURE0 && t < G.TEXTURE0 + s.maxTex) s.p[G.ACTIVE_TEXTURE] = t;
      else drop(s, [G.ACTIVE_TEXTURE]);
    });

    if (!rb) return;
    // ---- READ_BUFFER (WebGL2; all modes but norb) ----
    function knownFb(s, f) { return s.fbLive.has(f) && !s.fbDeleted.has(f); }
    proto.createFramebuffer = function () {
      var f = real.createFramebuffer.apply(this, arguments);
      if (f && !this.isContextLost()) shadowOf(this).fbLive.add(f);
      return f;
    };
    proto.deleteFramebuffer = function (f) {
      var r = real.deleteFramebuffer.apply(this, arguments);
      if (f && !this.isContextLost()) {
        var s = shadowOf(this);
        if (knownFb(s, f)) {
          s.fbDeleted.add(f);
          s.rbOf.delete(f);
          if (s.readFb === f) s.readFb = null; // deleting the bound framebuffer binds the default one
        }
      }
      return r;
    };
    // FRAMEBUFFER and READ_FRAMEBUFFER move the read binding (DRAW_FRAMEBUFFER does not). A
    // deleted framebuffer is refused (INVALID_OPERATION, binding unchanged); one the shim did not
    // see created (another context's: refused too, but not provably) makes the binding UNKNOWN.
    proto.bindFramebuffer = function (target, f) {
      var r = real.bindFramebuffer.apply(this, arguments);
      if (!this.isContextLost()) {
        var s = shadowOf(this), t = u32(target);
        if (t === FRAMEBUFFER || t === READ_FRAMEBUFFER) {
          if (f === null || f === undefined) s.readFb = null;
          else if (knownFb(s, f)) s.readFb = f;
          else if (!s.fbLive.has(f)) s.readFb = UNKNOWN;
        }
      }
      return r;
    };
    // The default framebuffer takes BACK or NONE, a framebuffer object NONE or COLOR_ATTACHMENTi
    // below MAX_COLOR_ATTACHMENTS; anything else is refused and leaves the value unchanged, which
    // the shadow does not try to prove: it forgets the entry and re-reads it on the next query.
    // Under an UNKNOWN binding the call may have changed ANY framebuffer's value (a refused
    // foreign bind leaves the previous one bound, known or default), so all of them are forgotten.
    proto.readBuffer = function (src) {
      var r = real.readBuffer.apply(this, arguments);
      if (!this.isContextLost()) {
        var s = shadowOf(this), m = u32(src), fb = s.readFb;
        if (fb === null) {
          s.rbDefault = (m === BACK || m === NONE) ? m : undefined;
        } else if (fb === UNKNOWN) {
          s.rbDefault = undefined;
          s.rbOf = new WeakMap();
        } else {
          var maxCA = s.p[MAX_COLOR_ATTACHMENTS];
          if (m === NONE || (maxCA > 0 && m >= COLOR_ATTACHMENT0 && m < COLOR_ATTACHMENT0 + maxCA))
            s.rbOf.set(fb, m);
          else
            s.rbOf.delete(fb);
        }
      }
      return r;
    };
  }

  install(WebGLRenderingContext.prototype, false);
  if (typeof WebGL2RenderingContext !== 'undefined') install(WebGL2RenderingContext.prototype, true);
})();
