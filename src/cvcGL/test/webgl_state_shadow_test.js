// clang-format off: hand-formatted JS; .clang-format is the C++ style (CI formats only C++).
// webgl_state_shadow_test.js -- node unit test for src/cvcGL/wasm/webgl_state_shadow.js (the
// --pre-js cvcgl_wasm_app() links into every cvcGL WebAssembly app).
//
// A mock WebGL context implements the GL/WebGL semantics the shadow relies on, modelled on
// Chrome (blink + ANGLE) and Firefox (ClientWebGLContext): rejected calls leave state
// unchanged, WebIDL argument conversion, WebGL's constant-colour/constant-alpha rule, link
// status + deferred program deletion, BLEND_COLOR clamped only on WebGL1, viewport clamped to
// MAX_VIEWPORT_DIMS (Chrome) or cached raw (Firefox), the active texture unit's range, WebGL2
// framebuffers (FRAMEBUFFER / READ_FRAMEBUFFER / DRAW_FRAMEBUFFER bindings, deletion of a bound
// framebuffer, a per-framebuffer READ_BUFFER and readBuffer's default-vs-FBO rules), context
// loss/restore. Each case loads the shim into a FRESH vm context with its own mock classes, then
// checks:
//   1. after seeding, shadowed queries make no call into the (synchronous) implementation;
//   2. a seeded random fuzz of valid AND invalid state changes -- the shadowed answer equals
//      the mock's INTERNAL state (read directly, never through the shim) after every step,
//      for WebGL1/WebGL2 x clamping/raw viewport x default / ?glshim=norb; the framebuffer ops
//      draw from this context's live, deleted and pre-loss framebuffers, another context's and
//      null / undefined;
//   3. edge cases the fuzz does not reach (listener ordering on restore, setters while lost,
//      WebIDL coercion, argument pass-through, a foreign framebuffer, deleting the bound read
//      framebuffer, WebGL1's unshadowed draw-buffer constants), and where the mode comes from:
//      URL > Module.glStateShadow (page) > Module.glStateShadowDefault (build) > on, and the
//      stats.version that names this copy;
//   4. ?glshim=0 installs nothing; ?glshim=verify answers with real values and counts 0
//      mismatches over the same fuzz (READ_BUFFER audited); no window (a pthread worker) is a
//      no-op.
// Usage: node webgl_state_shadow_test.js [path/to/webgl_state_shadow.js]
//        (default: ../wasm/webgl_state_shadow.js; registered with ctest as cvcgl_webgl_state_shadow)
'use strict';
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');

const SHIM = fs.readFileSync(process.argv[2] || `${__dirname}/../wasm/webgl_state_shadow.js`, 'utf8');

// Real WebGL enum values (subset used by the shim + the mock).
const E = {
  ZERO: 0, ONE: 1, SRC_COLOR: 0x0300, ONE_MINUS_SRC_COLOR: 0x0301, SRC_ALPHA: 0x0302,
  ONE_MINUS_SRC_ALPHA: 0x0303, DST_ALPHA: 0x0304, ONE_MINUS_DST_ALPHA: 0x0305, DST_COLOR: 0x0306,
  ONE_MINUS_DST_COLOR: 0x0307, SRC_ALPHA_SATURATE: 0x0308, CONSTANT_COLOR: 0x8001,
  ONE_MINUS_CONSTANT_COLOR: 0x8002, CONSTANT_ALPHA: 0x8003, ONE_MINUS_CONSTANT_ALPHA: 0x8004,
  FUNC_ADD: 0x8006, FUNC_SUBTRACT: 0x800A, FUNC_REVERSE_SUBTRACT: 0x800B, MIN: 0x8007, MAX: 0x8008,
  BLEND_EQUATION_RGB: 0x8009, BLEND_EQUATION_ALPHA: 0x883D, BLEND_DST_RGB: 0x80C8,
  BLEND_SRC_RGB: 0x80C9, BLEND_DST_ALPHA: 0x80CA, BLEND_SRC_ALPHA: 0x80CB, BLEND_COLOR: 0x8005,
  SCISSOR_BOX: 0x0C10, VIEWPORT: 0x0BA2, MAX_VIEWPORT_DIMS: 0x0D3A, CURRENT_PROGRAM: 0x8B8D,
  BLEND: 0x0BE2, CULL_FACE: 0x0B44, DEPTH_TEST: 0x0B71, STENCIL_TEST: 0x0B90, SCISSOR_TEST: 0x0C11,
  POLYGON_OFFSET_FILL: 0x8037, SAMPLE_ALPHA_TO_COVERAGE: 0x809E, SAMPLE_COVERAGE: 0x80A0,
  DITHER: 0x0BD0, RASTERIZER_DISCARD: 0x8C89, ACTIVE_TEXTURE: 0x84E0, TEXTURE0: 0x84C0,
  MAX_COMBINED_TEXTURE_IMAGE_UNITS: 0x8B4D, MAX_DRAW_BUFFERS: 0x8824, MAX_COLOR_ATTACHMENTS: 0x8CDF,
  READ_BUFFER: 0x0C02, FRAMEBUFFER: 0x8D40, READ_FRAMEBUFFER: 0x8CA8, DRAW_FRAMEBUFFER: 0x8CA9,
  READ_FRAMEBUFFER_BINDING: 0x8CAA, DRAW_FRAMEBUFFER_BINDING: 0x8CA6, BACK: 0x0405, NONE: 0,
  COLOR_ATTACHMENT0: 0x8CE0,
};
const MAXTEX = 32, MAXDB = 8, MAXCA = 4; // texture units, draw buffers, colour attachments
const FACTORS = [E.ZERO, E.ONE, E.SRC_COLOR, E.ONE_MINUS_SRC_COLOR, E.SRC_ALPHA, E.ONE_MINUS_SRC_ALPHA,
  E.DST_ALPHA, E.ONE_MINUS_DST_ALPHA, E.DST_COLOR, E.ONE_MINUS_DST_COLOR, E.SRC_ALPHA_SATURATE,
  E.CONSTANT_COLOR, E.ONE_MINUS_CONSTANT_COLOR, E.CONSTANT_ALPHA, E.ONE_MINUS_CONSTANT_ALPHA];
const SHADOWED = ['SCISSOR_BOX', 'VIEWPORT', 'BLEND_SRC_RGB', 'BLEND_DST_RGB', 'BLEND_SRC_ALPHA',
  'BLEND_DST_ALPHA', 'BLEND_EQUATION_RGB', 'BLEND_EQUATION_ALPHA'];
const PARAMS = SHADOWED.concat(['BLEND_COLOR']); // BLEND_COLOR must pass through, exactly
const CAPS1 = ['BLEND', 'CULL_FACE', 'DEPTH_TEST', 'STENCIL_TEST', 'SCISSOR_TEST', 'POLYGON_OFFSET_FILL',
  'SAMPLE_ALPHA_TO_COVERAGE', 'SAMPLE_COVERAGE', 'DITHER'];
const MAXVP = [4096, 2048];
const u32 = (x) => x >>> 0, i32 = (x) => x | 0; // WebIDL unsigned long / long

function makeMocks(opts) {
  const clampViewport = opts.clampViewport;
  class Canvas {
    constructor() { this.l = {}; }
    addEventListener(t, f) { (this.l[t] = this.l[t] || []).push(f); }
    fire(t) { (this.l[t] || []).slice().forEach((f) => f({ type: t })); }
  }
  class Prog { constructor(gl) { this.ctx = gl; this.gen = gl.gen; this.deleted = false; this.flagged = false; this.linked = false; } }
  class Fb { constructor(gl) { this.ctx = gl; this.gen = gl.gen; this.deleted = false; this.rb = E.COLOR_ATTACHMENT0; } }
  function defaults(gl) {
    gl.s = {
      SCISSOR_BOX: [0, 0, 300, 150], VIEWPORT: [0, 0, 300, 150], BLEND_COLOR: [0, 0, 0, 0],
      BLEND_SRC_RGB: E.ONE, BLEND_DST_RGB: E.ZERO, BLEND_SRC_ALPHA: E.ONE, BLEND_DST_ALPHA: E.ZERO,
      BLEND_EQUATION_RGB: E.FUNC_ADD, BLEND_EQUATION_ALPHA: E.FUNC_ADD,
    };
    gl.en = {}; for (const c of gl.caps) gl.en[E[c]] = c === 'DITHER';
    gl.cur = null; gl.gen = (gl.gen || 0) + 1; // programs from an older generation are invalid
    gl.activeTex = E.TEXTURE0;
    gl.readFb = null; gl.drawFb = null; gl.defRB = E.BACK; // default framebuffer reads BACK
  }
  function Base(isGL2) {
    this.isGL2 = isGL2; this.caps = isGL2 ? CAPS1.concat(['RASTERIZER_DISCARD']) : CAPS1;
    this.canvas = new Canvas(); this.lost = false; this.sync = 0; this.errors = 0; defaults(this);
  }
  const need = (args, n) => { if (args.length < n) throw new TypeError('not enough arguments'); };
  const progArg = (p) => { if (p !== null && p !== undefined && !(p instanceof Prog)) throw new TypeError('not a WebGLProgram'); };
  const fbArg = (f) => { if (f !== null && f !== undefined && !(f instanceof Fb)) throw new TypeError('not a WebGLFramebuffer'); };
  const proto = {
    isContextLost() { return this.lost; },
    getParameter(p) {
      need(arguments, 1); this.sync++; if (this.lost) return null; p = u32(p);
      if (p === E.MAX_VIEWPORT_DIMS) return new Int32Array(MAXVP);
      if (p === E.CURRENT_PROGRAM) return this.cur;
      if (p === E.ACTIVE_TEXTURE) return this.activeTex;
      if (p === E.MAX_COMBINED_TEXTURE_IMAGE_UNITS) return MAXTEX;
      if (this.isGL2) { // WebGL1 without WEBGL_draw_buffers: INVALID_ENUM below
        if (p === E.MAX_DRAW_BUFFERS) return MAXDB;
        if (p === E.MAX_COLOR_ATTACHMENTS) return MAXCA;
        if (p === E.READ_BUFFER) return this.readFb ? this.readFb.rb : this.defRB;
        if (p === E.READ_FRAMEBUFFER_BINDING) return this.readFb;
        if (p === E.DRAW_FRAMEBUFFER_BINDING) return this.drawFb;
      }
      for (const k of PARAMS) if (E[k] === p) {
        const v = this.s[k];
        return Array.isArray(v) ? (k === 'BLEND_COLOR' ? new Float32Array(v) : new Int32Array(v)) : v;
      }
      this.errors++; return null;
    },
    isEnabled(c) { need(arguments, 1); this.sync++; if (this.lost) return false; c = u32(c); if (!(c in this.en)) { this.errors++; return false; } return this.en[c]; },
    createProgram() { if (this.lost) return null; return new Prog(this); },
    linkProgram(p) { progArg(p); if (!this.lost && this._valid(p)) p.linked = true; },
    _valid(p) { return p instanceof Prog && p.ctx === this && p.gen === this.gen && !p.deleted; },
    isProgram(p) { need(arguments, 1); progArg(p); this.sync++; if (this.lost) return false; return !!this._valid(p); },
    deleteProgram(p) { progArg(p); if (this.lost || !this._valid(p)) return; if (p === this.cur) p.flagged = true; else p.deleted = true; },
    useProgram(p) {
      progArg(p); if (this.lost) return;
      if (p && (!this._valid(p) || !p.linked)) { this.errors++; return; } // INVALID_OPERATION
      if (this.cur && this.cur.flagged && this.cur !== p) this.cur.deleted = true;
      this.cur = p || null;
    },
    scissor(x, y, w, h) { if (this.lost) return; if (i32(w) < 0 || i32(h) < 0) { this.errors++; return; } this.s.SCISSOR_BOX = [i32(x), i32(y), i32(w), i32(h)]; },
    viewport(x, y, w, h) {
      if (this.lost) return; w = i32(w); h = i32(h); if (w < 0 || h < 0) { this.errors++; return; }
      this.s.VIEWPORT = clampViewport ? [i32(x), i32(y), Math.min(w, MAXVP[0]), Math.min(h, MAXVP[1])] : [i32(x), i32(y), w, h];
    },
    blendColor(r, g, b, a) { // WebGL1 (no float-buffer ext) clamps on store; WebGL2 does not
      if (this.lost) return; const c = this.isGL2 ? (x) => Math.fround(+x) : (x) => Math.fround(Math.min(1, Math.max(0, +x || 0)));
      this.s.BLEND_COLOR = [c(r), c(g), c(b), c(a)];
    },
    _fOK(sf, df) {
      if (!FACTORS.includes(sf) || !FACTORS.includes(df)) return false;
      if (!this.isGL2 && df === E.SRC_ALPHA_SATURATE) return false; // ES2: src only (ES3 allows dst)
      const cc = (f) => (f === E.CONSTANT_COLOR || f === E.ONE_MINUS_CONSTANT_COLOR ? 1 : (f === E.CONSTANT_ALPHA || f === E.ONE_MINUS_CONSTANT_ALPHA ? 2 : 0));
      return !(cc(sf) && cc(df) && cc(sf) !== cc(df));
    },
    blendFunc(sf, df) {
      if (this.lost) return; sf = u32(sf); df = u32(df); if (!this._fOK(sf, df)) { this.errors++; return; }
      Object.assign(this.s, { BLEND_SRC_RGB: sf, BLEND_SRC_ALPHA: sf, BLEND_DST_RGB: df, BLEND_DST_ALPHA: df });
    },
    blendFuncSeparate(sr, dr, sa, da) {
      if (this.lost) return; sr = u32(sr); dr = u32(dr); sa = u32(sa); da = u32(da);
      // WebGL: the constant rule applies to the RGB pair; each factor must be a valid enum.
      if (!this._fOK(sr, dr) || !FACTORS.includes(sa) || !FACTORS.includes(da) || (!this.isGL2 && da === E.SRC_ALPHA_SATURATE)) { this.errors++; return; }
      Object.assign(this.s, { BLEND_SRC_RGB: sr, BLEND_DST_RGB: dr, BLEND_SRC_ALPHA: sa, BLEND_DST_ALPHA: da });
    },
    _eOK(m) { return [E.FUNC_ADD, E.FUNC_SUBTRACT, E.FUNC_REVERSE_SUBTRACT].includes(m) || (this.isGL2 && (m === E.MIN || m === E.MAX)); },
    blendEquation(m) { if (this.lost) return; m = u32(m); if (!this._eOK(m)) { this.errors++; return; } this.s.BLEND_EQUATION_RGB = this.s.BLEND_EQUATION_ALPHA = m; },
    blendEquationSeparate(a, b) { if (this.lost) return; a = u32(a); b = u32(b); if (!this._eOK(a) || !this._eOK(b)) { this.errors++; return; } this.s.BLEND_EQUATION_RGB = a; this.s.BLEND_EQUATION_ALPHA = b; },
    enable(c) { if (this.lost) return; c = u32(c); if (!(c in this.en)) { this.errors++; return; } this.en[c] = true; },
    disable(c) { if (this.lost) return; c = u32(c); if (!(c in this.en)) { this.errors++; return; } this.en[c] = false; },
    activeTexture(t) {
      need(arguments, 1); if (this.lost) return; t = u32(t);
      if (t < E.TEXTURE0 || t >= E.TEXTURE0 + MAXTEX) { this.errors++; return; } // INVALID_ENUM
      this.activeTex = t;
    },
    createFramebuffer() { if (this.lost) return null; return new Fb(this); },
    _fbValid(f) { return f instanceof Fb && f.ctx === this && f.gen === this.gen && !f.deleted; },
    bindFramebuffer(target, f) {
      need(arguments, 2); fbArg(f); if (this.lost) return; target = u32(target);
      const targets = this.isGL2 ? [E.FRAMEBUFFER, E.READ_FRAMEBUFFER, E.DRAW_FRAMEBUFFER] : [E.FRAMEBUFFER];
      if (!targets.includes(target)) { this.errors++; return; }        // INVALID_ENUM
      if (f && !this._fbValid(f)) { this.errors++; return; }           // deleted / foreign: INVALID_OPERATION
      const b = f || null;
      if (target !== E.DRAW_FRAMEBUFFER) this.readFb = b;
      if (target !== E.READ_FRAMEBUFFER) this.drawFb = b;
    },
    deleteFramebuffer(f) {
      fbArg(f); if (this.lost || !f) return;
      if (!this._fbValid(f)) { if (f.ctx !== this || f.gen !== this.gen) this.errors++; return; }
      f.deleted = true; // a bound framebuffer is unbound (as if bound to null)
      if (this.readFb === f) this.readFb = null;
      if (this.drawFb === f) this.drawFb = null;
    },
    // test hooks (not WebGL API)
    _lose() { this.lost = true; this.canvas.fire('webglcontextlost'); },
    _restore() { this.lost = false; defaults(this); this.canvas.fire('webglcontextrestored'); },
  };
  function GL1() { Base.call(this, false); }
  function GL2() { Base.call(this, true); }
  GL1.prototype = Object.assign(Object.create(null), proto);
  GL2.prototype = Object.assign(Object.create(null), proto);
  // WebGL2 only. The default framebuffer reads BACK or NONE; a framebuffer object NONE or
  // COLOR_ATTACHMENTi (i < MAX_COLOR_ATTACHMENTS); anything else is refused, state unchanged.
  GL2.prototype.readBuffer = function (src) {
    need(arguments, 1); if (this.lost) return; src = u32(src);
    if (!this.readFb) {
      if (src !== E.BACK && src !== E.NONE) { this.errors++; return; }
      this.defRB = src;
    } else {
      if (src !== E.NONE && !(src >= E.COLOR_ATTACHMENT0 && src < E.COLOR_ATTACHMENT0 + MAXCA)) { this.errors++; return; }
      this.readFb.rb = src;
    }
  };
  for (const k in E) { GL1[k] = E[k]; GL2[k] = E[k]; }
  return { GL1, GL2, Fb };
}

function load(search, opts = {}) {
  const o = Object.assign({ clampViewport: true, window: true, Module: undefined }, opts);
  const { GL1, GL2, Fb } = makeMocks(o);
  const ctx = { console: o.console || console, URLSearchParams, Math, Object, WeakMap, WeakSet, Int32Array, Float32Array, String,
                JSON };
  ctx.WebGLRenderingContext = GL1; ctx.WebGL2RenderingContext = GL2;
  if (o.window) ctx.window = { location: { search } };
  if (o.Module !== undefined) ctx.Module = o.Module;
  vm.createContext(ctx);
  vm.runInContext(SHIM, ctx);
  return { ctx, GL1, GL2, Fb };
}

// Deterministic PRNG (mulberry32) so failures reproduce.
function rng(seed) { return () => { seed |= 0; seed = (seed + 0x6D2B79F5) | 0; let t = Math.imul(seed ^ (seed >>> 15), 1 | seed); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }

function truth(gl, name) { // the mock's INTERNAL state, never read through the shim
  const v = gl.s[name]; return Array.isArray(v) ? v.slice() : v;
}
function arr(v) { return v && v.length !== undefined ? Array.from(v) : v; }

// `other` (optional): a second context of the same kind, whose framebuffers the fuzz also binds and
// deletes on `gl` (refused: INVALID_OPERATION) -- the shim cannot prove those refusals.
function fuzz(gl, steps, seed, check, other) {
  const r = rng(seed); const pick = (a) => a[Math.floor(r() * a.length)];
  const progs = [];
  // Mostly plain numbers (what emscripten passes), sometimes WebIDL-coercible forms and junk.
  const coerce = (v) => { const k = r(); return k < 0.9 ? v : k < 0.94 ? String(v) : k < 0.97 ? v + 2 ** 32 : v + 0.5; };
  const anyEnum = () => coerce(r() < 0.85 ? pick(FACTORS) : pick([0x1234, E.MIN, E.FUNC_ADD, -1]));
  const anyEq = () => coerce(r() < 0.85 ? pick([E.FUNC_ADD, E.FUNC_SUBTRACT, E.FUNC_REVERSE_SUBTRACT, E.MIN, E.MAX]) : pick([0x9999, E.ONE]));
  const anyCap = () => coerce(r() < 0.9 ? E[pick(gl.caps)] : pick([0x1111, E.RASTERIZER_DISCARD]));
  const anyUnit = () => coerce(r() < 0.85 ? E.TEXTURE0 + Math.floor(r() * MAXTEX) : pick([E.TEXTURE0 + MAXTEX, E.TEXTURE0 - 1, 0x1234, 0]));
  const anyFbTarget = () => coerce(r() < 0.9 ? pick([E.FRAMEBUFFER, E.READ_FRAMEBUFFER, E.DRAW_FRAMEBUFFER]) : pick([0x1234, E.READ_BUFFER]));
  const anyReadBuf = () => coerce(r() < 0.85
    ? pick([E.BACK, E.NONE, E.COLOR_ATTACHMENT0, E.COLOR_ATTACHMENT0 + 1, E.COLOR_ATTACHMENT0 + MAXCA - 1])
    : pick([E.COLOR_ATTACHMENT0 + MAXCA, E.COLOR_ATTACHMENT0 + 15, 0x1234, E.FRAMEBUFFER]));
  const fbs = [], foreign = [];
  const anyFb = () => { // mostly this context's (live, deleted, or from before a loss)
    const k = r();
    if (k < 0.2 || !fbs.length) return k < 0.1 ? null : (k < 0.15 ? undefined : null);
    if (k > 0.95 && other) {
      if (!foreign.length || r() < 0.2) { const f = other.createFramebuffer(); if (f) foreign.push(f); }
      if (foreign.length) return pick(foreign);
    }
    return pick(fbs);
  };
  for (let i = 0; i < steps; ++i) {
    switch (Math.floor(r() * 26)) {
      case 0: gl.scissor(Math.floor(r() * 500) - 50, Math.floor(r() * 500), Math.floor(r() * 900) - 100, Math.floor(r() * 900) - 100); break;
      case 1: gl.viewport(Math.floor(r() * 50), Math.floor(r() * 50), Math.floor(r() * 9000) - 100, Math.floor(r() * 9000) - 100); break;
      case 2: gl.blendColor(r() * 2 - 0.5, r(), r() * 3, -r()); break;
      case 3: gl.blendFunc(anyEnum(), anyEnum()); break;
      case 4: gl.blendFuncSeparate(anyEnum(), anyEnum(), anyEnum(), anyEnum()); break;
      case 5: gl.blendEquation(anyEq()); break;
      case 6: gl.blendEquationSeparate(anyEq(), anyEq()); break;
      case 7: gl.enable(anyCap()); break;
      case 8: gl.disable(anyCap()); break;
      case 9: { const p = gl.createProgram(); if (p) progs.push(p); break; }
      case 10: if (progs.length) gl.linkProgram(pick(progs)); break;
      case 11: if (progs.length) gl.deleteProgram(pick(progs)); break;
      case 12: gl.useProgram(r() < 0.2 ? null : (progs.length ? pick(progs) : null)); break;
      case 13: if (r() < 0.02) gl._lose(); break;
      case 14: if (gl.lost && r() < 0.5) gl._restore(); break;
      case 15: case 16: gl.activeTexture(anyUnit()); break;
      case 17: { const f = gl.createFramebuffer(); if (f) fbs.push(f); break; }
      case 18: case 19: case 20: gl.bindFramebuffer(anyFbTarget(), anyFb()); break;
      case 21: { // often the bound read framebuffer itself (deleting it rebinds the default one)
        const f = gl.readFb && r() < 0.3 ? gl.readFb : anyFb();
        if (f && r() < 0.5) gl.deleteFramebuffer(f);
        break;
      }
      case 22: case 23: if (gl.readBuffer) gl.readBuffer(anyReadBuf()); break;
      default: break; // queries only
    }
    check(gl, progs);
  }
}

function checkExact(gl, progs) {
  if (gl.isContextLost()) return;
  for (const k of PARAMS) assert.deepStrictEqual(arr(gl.getParameter(E[k])), truth(gl, k), k);
  for (const c of gl.caps) assert.strictEqual(gl.isEnabled(E[c]), gl.en[E[c]], c);
  for (const p of progs) assert.strictEqual(gl.isProgram(p), !!gl._valid(p), 'isProgram');
  assert.strictEqual(gl.isProgram(null), false);
  assert.strictEqual(gl.getParameter(E.ACTIVE_TEXTURE), gl.activeTex, 'ACTIVE_TEXTURE');
  if (gl.isGL2) {
    assert.strictEqual(gl.getParameter(E.MAX_DRAW_BUFFERS), MAXDB, 'MAX_DRAW_BUFFERS');
    assert.strictEqual(gl.getParameter(E.MAX_COLOR_ATTACHMENTS), MAXCA, 'MAX_COLOR_ATTACHMENTS');
    assert.strictEqual(gl.getParameter(E.READ_BUFFER), gl.readFb ? gl.readFb.rb : gl.defRB, 'READ_BUFFER');
  }
}

let passed = 0;
function test(name, fn) { fn(); passed++; console.log('ok -', name); }

test('shadowed queries make no synchronous call once seeded', () => {
  const { GL2 } = load('');
  const gl = new GL2();
  gl.getParameter(E.SCISSOR_BOX); // seeds
  const before = gl.sync;
  for (let i = 0; i < 1000; ++i) {
    for (const k of SHADOWED) gl.getParameter(E[k]);
    for (const c of gl.caps) gl.isEnabled(E[c]);
    gl.getParameter(E.ACTIVE_TEXTURE); gl.getParameter(E.MAX_DRAW_BUFFERS);
    gl.getParameter(E.MAX_COLOR_ATTACHMENTS);
    gl.scissor(1, 2, 3, 4); gl.viewport(0, 0, 640, 480); gl.activeTexture(E.TEXTURE0 + (i % MAXTEX));
    gl.blendFuncSeparate(E.SRC_ALPHA, E.ONE_MINUS_SRC_ALPHA, E.ONE, E.ONE_MINUS_SRC_ALPHA);
    const p = gl.createProgram(); gl.linkProgram(p); gl.useProgram(p); gl.isProgram(p);
  }
  assert.strictEqual(gl.sync - before, 0, 'sync calls after seeding');
  assert.strictEqual(gl.getParameter(E.ACTIVE_TEXTURE), E.TEXTURE0 + (999 % MAXTEX));
  gl.getParameter(E.CURRENT_PROGRAM); gl.getParameter(E.BLEND_COLOR); // unshadowed reach the impl
  assert.strictEqual(gl.sync - before, 2);
});

test('?glshim=norb: READ_BUFFER (only) goes to the real call', () => {
  const m = load('?glshim=norb'); const gl = new m.GL2();
  assert.strictEqual(m.ctx.window.__cvcGlShadow.stats.mode, 'norb');
  const f = gl.createFramebuffer(); gl.bindFramebuffer(E.FRAMEBUFFER, f); gl.readBuffer(E.NONE);
  gl.getParameter(E.ACTIVE_TEXTURE); // seeds
  const before = gl.sync;
  for (let i = 0; i < 10; ++i) {
    assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.NONE);
    gl.getParameter(E.ACTIVE_TEXTURE); gl.getParameter(E.MAX_COLOR_ATTACHMENTS);
  }
  assert.strictEqual(gl.sync - before, 10, 'one real READ_BUFFER per query, nothing else');
});

test('activeTexture outside the texture units changes nothing (and is re-read, not guessed)', () => {
  const { GL2 } = load('');
  const gl = new GL2(); gl.activeTexture(E.TEXTURE0 + 5);
  for (const bad of [E.TEXTURE0 + MAXTEX, E.TEXTURE0 - 1, 0x1234, -1]) {
    gl.activeTexture(bad);
    assert.strictEqual(gl.getParameter(E.ACTIVE_TEXTURE), E.TEXTURE0 + 5, `0x${(bad >>> 0).toString(16)}`);
  }
  gl.activeTexture(String(E.TEXTURE0 + 7)); // WebIDL coercion
  assert.strictEqual(gl.getParameter(E.ACTIVE_TEXTURE), E.TEXTURE0 + 7);
});

test('WebGL1: MAX_DRAW_BUFFERS / MAX_COLOR_ATTACHMENTS / READ_BUFFER go to the real call', () => {
  for (const q of ['', '?glshim=norb', '?glshim=verify']) {
    const { GL1 } = load(q);
    const gl = new GL1(); gl.getParameter(E.SCISSOR_BOX); // seeds
    const s0 = gl.sync, e0 = gl.errors;
    for (const k of ['MAX_DRAW_BUFFERS', 'MAX_COLOR_ATTACHMENTS', 'READ_BUFFER'])
      assert.strictEqual(gl.getParameter(E[k]), null, k); // INVALID_ENUM without the extension
    assert.strictEqual(gl.sync - s0, 3); assert.strictEqual(gl.errors - e0, 3);
  }
});

test('READ_BUFFER follows the read binding with no synchronous call once seeded', () => {
  const m = load(''); const gl = new m.GL2();
  assert.strictEqual(m.ctx.window.__cvcGlShadow.stats.mode, 'on');
  const a = gl.createFramebuffer(), b = gl.createFramebuffer();
  // seed the three framebuffers' read buffers (the default one and both objects) once each
  gl.getParameter(E.READ_BUFFER);
  gl.bindFramebuffer(E.READ_FRAMEBUFFER, a); gl.getParameter(E.READ_BUFFER);
  gl.bindFramebuffer(E.FRAMEBUFFER, b); gl.getParameter(E.READ_BUFFER);
  const before = gl.sync;
  for (let i = 0; i < 500; ++i) {
    gl.bindFramebuffer(E.READ_FRAMEBUFFER, a); gl.readBuffer(E.COLOR_ATTACHMENT0 + (i % MAXCA));
    assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.COLOR_ATTACHMENT0 + (i % MAXCA));
    gl.bindFramebuffer(E.DRAW_FRAMEBUFFER, null); // the draw binding does not move the read one
    assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.COLOR_ATTACHMENT0 + (i % MAXCA));
    gl.bindFramebuffer(E.FRAMEBUFFER, null); gl.readBuffer(i % 2 ? E.NONE : E.BACK);
    assert.strictEqual(gl.getParameter(E.READ_BUFFER), i % 2 ? E.NONE : E.BACK);
    gl.bindFramebuffer(E.READ_FRAMEBUFFER, b);
    assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.COLOR_ATTACHMENT0); // b's own, untouched
  }
  assert.strictEqual(gl.sync - before, 0, 'sync calls after seeding');
});

test('READ_BUFFER: refused readBuffer / bindFramebuffer leave it as the GL has it', () => {
  const m = load(''); const gl = new m.GL2();
  const f = gl.createFramebuffer();
  gl.readBuffer(E.COLOR_ATTACHMENT0); // default framebuffer: INVALID_OPERATION
  assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.BACK);
  gl.bindFramebuffer(E.READ_FRAMEBUFFER, f); gl.readBuffer(E.COLOR_ATTACHMENT0 + 2);
  for (const bad of [E.BACK, E.COLOR_ATTACHMENT0 + MAXCA, 0x1234]) {
    gl.readBuffer(bad);
    assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.COLOR_ATTACHMENT0 + 2);
  }
  gl.bindFramebuffer(0x1234, null); // INVALID_ENUM: the read binding stays f
  assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.COLOR_ATTACHMENT0 + 2);
});

test('READ_BUFFER: deleting the bound read framebuffer falls back to the default one', () => {
  const m = load(''); const gl = new m.GL2();
  gl.readBuffer(E.NONE); // default framebuffer reads NONE
  const f = gl.createFramebuffer(); gl.bindFramebuffer(E.FRAMEBUFFER, f); gl.readBuffer(E.COLOR_ATTACHMENT0 + 1);
  gl.deleteFramebuffer(f);
  assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.NONE);
  gl.bindFramebuffer(E.READ_FRAMEBUFFER, f); // deleted: INVALID_OPERATION, still the default
  assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.NONE);
  // only the READ binding of a framebuffer bound for drawing alone survives its deletion
  const g = gl.createFramebuffer(), h = gl.createFramebuffer();
  gl.bindFramebuffer(E.READ_FRAMEBUFFER, g); gl.readBuffer(E.COLOR_ATTACHMENT0 + 3);
  gl.bindFramebuffer(E.DRAW_FRAMEBUFFER, h); gl.deleteFramebuffer(h);
  assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.COLOR_ATTACHMENT0 + 3);
  // a re-created framebuffer starts from its own default, not a deleted one's value
  gl.deleteFramebuffer(g);
  const g2 = gl.createFramebuffer(); gl.bindFramebuffer(E.READ_FRAMEBUFFER, g2);
  assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.COLOR_ATTACHMENT0);
});

test('READ_BUFFER: a foreign framebuffer makes the binding UNKNOWN until a known one is bound', () => {
  const m = load(''); const a = new m.GL2(), b = new m.GL2();
  const fa = a.createFramebuffer(), fb = b.createFramebuffer();
  a.bindFramebuffer(E.READ_FRAMEBUFFER, fa); a.readBuffer(E.COLOR_ATTACHMENT0 + 3);
  a.getParameter(E.READ_BUFFER);
  a.bindFramebuffer(E.READ_FRAMEBUFFER, fb); // b's: refused, the GL still reads fa
  const s0 = a.sync;
  assert.strictEqual(a.getParameter(E.READ_BUFFER), E.COLOR_ATTACHMENT0 + 3);
  assert.strictEqual(a.sync - s0, 1, 'UNKNOWN binding: answered by the real call');
  a.bindFramebuffer(E.READ_FRAMEBUFFER, null); a.getParameter(E.READ_BUFFER);
  const s1 = a.sync;
  assert.strictEqual(a.getParameter(E.READ_BUFFER), E.BACK);
  assert.strictEqual(a.sync - s1, 0, 'known again: answered from the shadow');
  // a readBuffer under an UNKNOWN binding may have changed whichever framebuffer is really bound
  a.bindFramebuffer(E.READ_FRAMEBUFFER, fb); a.readBuffer(E.NONE); // reaches the default one
  a.bindFramebuffer(E.READ_FRAMEBUFFER, null);
  assert.strictEqual(a.getParameter(E.READ_BUFFER), E.NONE);
});

test('READ_BUFFER / ACTIVE_TEXTURE: context loss drops the shadow', () => {
  const m = load(''); const gl = new m.GL2();
  const f = gl.createFramebuffer(); gl.bindFramebuffer(E.FRAMEBUFFER, f); gl.readBuffer(E.NONE);
  gl.activeTexture(E.TEXTURE0 + 9);
  gl._lose(); gl.readBuffer(E.COLOR_ATTACHMENT0); gl.activeTexture(E.TEXTURE0 + 3); gl._restore();
  assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.BACK);
  assert.strictEqual(gl.getParameter(E.ACTIVE_TEXTURE), E.TEXTURE0);
  gl.bindFramebuffer(E.FRAMEBUFFER, f); // from before the loss: refused
  assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.BACK);
});

test('returned arrays are copies (caller mutation cannot corrupt the shadow)', () => {
  const { GL2 } = load('');
  const gl = new GL2(); gl.scissor(5, 6, 7, 8);
  const a = gl.getParameter(E.SCISSOR_BOX); a[0] = 999;
  assert.deepStrictEqual(Array.from(gl.getParameter(E.SCISSOR_BOX)), [5, 6, 7, 8]);
});

test('deleted programs follow GL deferred-delete rules (via the real call)', () => {
  const { GL2 } = load('');
  const gl = new GL2(); const p = gl.createProgram(), q = gl.createProgram(), u = gl.createProgram();
  gl.linkProgram(p); gl.linkProgram(q);
  gl.useProgram(p); gl.deleteProgram(p);
  assert.strictEqual(gl.isProgram(p), true);   // flagged while current
  gl.useProgram(u);                             // unlinked: rejected, p stays current
  assert.strictEqual(gl.isProgram(p), true);
  gl.useProgram(q);
  assert.strictEqual(gl.isProgram(p), false);
  assert.strictEqual(gl.isProgram(u), true);    // never deleted: served from the shadow
});

test('foreign-context and non-program arguments', () => {
  const { GL2 } = load('');
  const a = new GL2(), b = new GL2(); const p = a.createProgram();
  assert.strictEqual(b.isProgram(p), false);
  assert.strictEqual(a.isProgram(p), true);
  assert.throws(() => a.isProgram({}), TypeError);   // WebIDL type check still happens
  assert.throws(() => a.getParameter(), TypeError);  // arity still checked
  assert.throws(() => a.isEnabled(), TypeError);
});

test('context loss/restore re-seeds; setters while lost change nothing', () => {
  const { GL2 } = load('');
  const gl = new GL2(); gl.scissor(9, 9, 9, 9); gl.enable(E.BLEND); const p = gl.createProgram();
  gl._lose();
  assert.strictEqual(gl.isProgram(p), false);
  gl.scissor(1, 2, 3, 4); gl.enable(E.CULL_FACE); gl.blendFunc(E.SRC_ALPHA, E.ONE); // ignored while lost
  gl._restore();
  assert.deepStrictEqual(Array.from(gl.getParameter(E.SCISSOR_BOX)), [0, 0, 300, 150]);
  assert.strictEqual(gl.isEnabled(E.BLEND), false);
  assert.strictEqual(gl.isEnabled(E.CULL_FACE), false);
  assert.strictEqual(gl.getParameter(E.BLEND_SRC_RGB), E.ONE);
  assert.strictEqual(gl.isProgram(p), false);
});

test('an app restore listener registered BEFORE the shim keeps its new programs', () => {
  const { GL2 } = load('');
  const gl = new GL2(); let fresh = null;
  gl.canvas.addEventListener('webglcontextrestored', () => { fresh = gl.createProgram(); gl.scissor(3, 3, 3, 3); });
  gl.getParameter(E.SCISSOR_BOX); // shim registers its own listeners now (after the app's)
  gl._lose(); gl._restore();
  assert.ok(fresh);
  const s0 = gl.sync;
  assert.strictEqual(gl.isProgram(fresh), true);
  assert.deepStrictEqual(Array.from(gl.getParameter(E.SCISSOR_BOX)), [3, 3, 3, 3]);
  assert.strictEqual(gl.sync, s0, 'still answered from the shadow (no round-trip) after restore');
});

test('WebIDL coercion: string / wrapped / fractional enums match the browser', () => {
  const { GL2 } = load('');
  const gl = new GL2();
  gl.enable('3042'); assert.strictEqual(gl.isEnabled(E.BLEND), true);
  gl.disable(E.BLEND + 2 ** 32); assert.strictEqual(gl.isEnabled(E.BLEND), false);
  gl.enable(E.BLEND + 0.5); assert.strictEqual(gl.isEnabled(E.BLEND), true);
  gl.blendFunc('770', '771');
  assert.strictEqual(gl.getParameter(E.BLEND_SRC_RGB), 770);
  assert.strictEqual(typeof gl.getParameter(E.BLEND_DST_ALPHA), 'number');
  gl.blendEquation('32778'); assert.strictEqual(gl.getParameter(E.BLEND_EQUATION_RGB), 32778);
  gl.scissor('4', 5.9, NaN, 2 ** 32 + 7); assert.deepStrictEqual(Array.from(gl.getParameter(E.SCISSOR_BOX)), [4, 5, 0, 7]);
});

test('viewport beyond MAX_VIEWPORT_DIMS is re-read (browsers differ)', () => {
  for (const clampViewport of [true, false]) {
    const { GL2 } = load('', { clampViewport });
    const gl = new GL2(); gl.viewport(0, 0, 9000, 100);
    assert.deepStrictEqual(Array.from(gl.getParameter(E.VIEWPORT)), truth(gl, 'VIEWPORT'));
  }
});

for (const [label, Cls, query] of [['WebGL2', 'GL2', ''], ['WebGL2 ?glshim=norb', 'GL2', '?glshim=norb'],
                                     ['WebGL1', 'GL1', ''], ['WebGL1 ?glshim=norb', 'GL1', '?glshim=norb']]) {
  for (const clampViewport of [true, false]) {
    test(`${label} fuzz (viewport ${clampViewport ? 'clamped' : 'raw'}): shadow == real state after every step`, () => {
      let served = 0;
      for (let seed = 1; seed <= 10; ++seed) {
        const m = load(query, { clampViewport }); const gl = new m[Cls]();
        fuzz(gl, 4000, seed, checkExact, new m[Cls]());
        assert.ok(gl.errors > 0, 'fuzz must exercise rejected calls');
        served += m.ctx.window.__cvcGlShadow.stats.served;
      }
      assert.ok(served > 10000, 'the fuzz must be answered from the shadow');
    });
  }
}

test('?glshim=verify answers with real values and counts zero mismatches (READ_BUFFER audited)', () => {
  const m = load('?glshim=verify'); const gl = new m.GL2();
  // Count the READ_BUFFER answers the shadow could vouch for (and so was compared on).
  const st = m.ctx.window.__cvcGlShadow.stats;
  let rbChecked = 0;
  fuzz(gl, 6000, 99, (g, progs) => {
    checkExact(g, progs);
    if (g.isContextLost()) return;
    const v0 = st.verified; g.getParameter(E.READ_BUFFER); rbChecked += st.verified - v0;
  }, new m.GL2());
  assert.strictEqual(st.mode, 'verify'); assert.ok(st.verified > 1000); assert.strictEqual(st.mismatches, 0);
  assert.deepStrictEqual(Object.keys(st.mismatchBy), []);
  assert.ok(rbChecked > 1000, `READ_BUFFER audited ${rbChecked}x`);
});

test('?glshim=verify reports, by name, a value the shadow got wrong', () => {
  // State changed behind the shim's back (the mock's internals, which the shim never sees) must
  // show up as a mismatch under that value's name, not pass silently: a framebuffer's read buffer
  // and the active texture unit here.
  const m = load('?glshim=verify'); const gl = new m.GL2();
  const f = gl.createFramebuffer(); gl.bindFramebuffer(E.READ_FRAMEBUFFER, f);
  assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.COLOR_ATTACHMENT0); // seeds f's entry
  f.rb = E.NONE;
  gl.activeTexture(E.TEXTURE0 + 2); gl.activeTex = E.TEXTURE0 + 6;
  const warn = console.warn, warned = [];
  console.warn = (...a) => { warned.push(a[1]); };
  try {
    assert.strictEqual(gl.getParameter(E.READ_BUFFER), E.NONE);
    assert.strictEqual(gl.getParameter(E.ACTIVE_TEXTURE), E.TEXTURE0 + 6);
  } finally { console.warn = warn; }
  const st = m.ctx.window.__cvcGlShadow.stats;
  assert.strictEqual(st.mismatches, 2);
  assert.deepStrictEqual(Object.assign({}, st.mismatchBy), { READ_BUFFER: 1, ACTIVE_TEXTURE: 1 });
  assert.deepStrictEqual(warned, ['READ_BUFFER (framebuffer object)', 'ACTIVE_TEXTURE']);
});

test('?glshim=0 / Module.glStateShadow=false|0|"off" install nothing', () => {
  for (const [q, mod] of [['?glshim=0', undefined], ['?glshim=off', undefined], ['', false], ['', 0], ['', 'off'], ['', 'false']]) {
    const m = load(q, { Module: mod === undefined ? undefined : { glStateShadow: mod } });
    assert.strictEqual(m.ctx.window.__cvcGlShadow.stats.mode, 'off', `${q} ${mod}`);
    assert.strictEqual(m.GL2.prototype.__cvcGlShadowed, undefined);
  }
  assert.strictEqual(load('', { Module: { glStateShadow: true } }).ctx.window.__cvcGlShadow.stats.mode, 'on');
  assert.strictEqual(load('', { Module: { glStateShadow: 'norb' } }).ctx.window.__cvcGlShadow.stats.mode, 'norb');
  assert.strictEqual(load('?glshim=NORB').ctx.window.__cvcGlShadow.stats.mode, 'norb');
});

test('mode precedence: URL > Module.glStateShadow (page) > Module.glStateShadowDefault (build) > on', () => {
  const st = (q, Module) => load(q, { Module }).ctx.window.__cvcGlShadow.stats;
  const at = (q, Module) => { const s = st(q, Module); return s.mode + '/' + s.modeFrom; };
  assert.strictEqual(at('', undefined), 'on/default');
  assert.strictEqual(at('', {}), 'on/default');
  // the build-time default (cvcgl_wasm_app STATE_SHIM_MODE) applies when nothing else says
  assert.strictEqual(at('', { glStateShadowDefault: 'verify' }), 'verify/build');
  assert.strictEqual(at('', { glStateShadowDefault: 'norb' }), 'norb/build');
  // the page beats the build
  assert.strictEqual(at('', { glStateShadowDefault: 'verify', glStateShadow: 'norb' }), 'norb/page');
  assert.strictEqual(at('', { glStateShadowDefault: 'verify', glStateShadow: false }), 'off/page');
  // the URL beats both, either way round, so a URL can always A/B any page
  assert.strictEqual(at('?glshim=verify', { glStateShadow: 'off' }), 'verify/url');
  assert.strictEqual(at('?glshim=0', { glStateShadow: 'on', glStateShadowDefault: 'verify' }), 'off/url');
  assert.strictEqual(at('?glshim=norb', { glStateShadowDefault: 'verify' }), 'norb/url');
  assert.strictEqual(at('?glshim', { glStateShadow: 'off' }), 'on/url'); // a bare ?glshim means on
  assert.strictEqual(at('?other=1', { glStateShadow: 'verify' }), 'verify/page');
  // a URL that turns it off installs nothing, whatever the page and build asked for
  const m = load('?glshim=off', { Module: { glStateShadow: 'verify', glStateShadowDefault: 'norb' } });
  assert.strictEqual(m.GL2.prototype.__cvcGlShadowed, undefined);
  // a build default of verify is a real verify: real values answered, mismatches counted
  const v = load('', { Module: { glStateShadowDefault: 'verify' } }); const gl = new v.GL2();
  gl.scissor(1, 2, 3, 4); gl.getParameter(E.SCISSOR_BOX);
  assert.strictEqual(v.ctx.window.__cvcGlShadow.stats.verified, 1);
  assert.strictEqual(v.ctx.window.__cvcGlShadow.stats.served, 0);
});

test('an unrecognized mode is on, with a console.warn naming the accepted values', () => {
  const warned = [];
  const con = { warn: (...a) => warned.push(a.join(' ')), log() {}, error() {} };
  const st = (q, Module) => load(q, { Module, console: con }).ctx.window.__cvcGlShadow.stats;
  // accepted spellings never warn
  for (const [q, mod, want] of [['?glshim', undefined, 'on'], ['?glshim=on', undefined, 'on'], ['?glshim=1', undefined, 'on'],
                                ['?glshim=TRUE', undefined, 'on'], ['?glshim=Off', undefined, 'off'], ['?glshim=0', undefined, 'off'],
                                ['?glshim=verify', undefined, 'verify'], ['?glshim=norb', undefined, 'norb'],
                                ['', { glStateShadow: true }, 'on'], ['', { glStateShadow: false }, 'off'],
                                ['', { glStateShadow: 0 }, 'off'], ['', { glStateShadowDefault: 'verify' }, 'verify']]) {
    assert.strictEqual(st(q, mod).mode, want, `${q} ${JSON.stringify(mod)}`);
  }
  assert.deepStrictEqual(warned, []);
  // typos: still on (the shim is the safe default), but loudly
  assert.strictEqual(st('?glshim=of').mode, 'on');
  assert.strictEqual(st('', { glStateShadow: 'no' }).mode, 'on');
  assert.strictEqual(st('', { glStateShadowDefault: 'verfy' }).mode, 'on');
  assert.strictEqual(warned.length, 3, warned.join('\n'));
  assert.match(warned[0], /"of".*\?glshim=.*0\|off\|false.*verify.*norb/);
  assert.match(warned[1], /"no".*Module\.glStateShadow\b/);
  assert.match(warned[2], /"verfy".*Module\.glStateShadowDefault/);
});

test('stats.version names this copy of the shim, in every mode', () => {
  for (const q of ['', '?glshim=0', '?glshim=verify', '?glshim=norb']) {
    const s = load(q).ctx.window.__cvcGlShadow.stats;
    assert.strictEqual(typeof s.version, 'string', q);
    assert.match(s.version, /^cvcGL-\d+$/, q);
  }
});

test('no window (pthread worker) is a no-op', () => {
  const m = load('', { window: false });
  assert.strictEqual(m.GL2.prototype.__cvcGlShadowed, undefined);
});

console.log(`${passed} tests passed`);
