#!/usr/bin/env node
// Node test for ../glsync.js against a mock WebGL2RenderingContext prototype (no browser).
//   node tests/test_glsync.js            assertions + overhead measurement; exit 1 on failure
//   node tests/test_glsync.js --show     also print the census output of the mock-frame case
// Each case loads glsync.js into a fresh vm context whose window/location/history/console/document/
// navigator/requestAnimationFrame are fakes, drives frames, runs "paints" (the fake rAF queue, the
// census's paint clock) between them, prints PROF lines the way an app that reports its own frame
// timing does (or none, like the cvcGL gallery page, for the raf frame clock), and checks what
// glsync reports.
// Registered with ctest as cvcgl_glsync (label js); src/cvcGL/wasm/run-js-tests.sh runs it for CI.
//
// HARNESS SELF-TEST ONLY: the mock frame below (heavyOverlayFrame) has hand-picked call volumes for
// a heavy overlay frame (per-frame buffer and texture uploads, many uniform and texture binds, plus
// the sync queries VTK and ImGui's OpenGL3 backend make). Its sync and flush figures only check the
// census's bookkeeping against hand-computed values; they are not evidence about any real frame or
// about async present being lost. Only the live ?glsync output in Firefox measures that.
'use strict';
const assert = require('assert/strict');
const fs = require('fs'), vm = require('vm'), path = require('path');
const SRC = fs.readFileSync(path.join(__dirname, '..', 'glsync.js'), 'utf8');
const FX_UA = 'Mozilla/5.0 (X11; Ubuntu; Linux x86_64; rv:156.0) Gecko/20100101 Firefox/156.0';
const CR_UA = 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/141.0.0.0 Safari/537.36';

// Real WebGL2 enum values (the subset the fake frames use).
const K = {
  ACTIVE_TEXTURE: 0x84E0, READ_BUFFER: 0x0C02, MAX_DRAW_BUFFERS: 0x8824, MAX_COLOR_ATTACHMENTS: 0x8CDF,
  CURRENT_PROGRAM: 0x8B8D, TEXTURE_BINDING_2D: 0x8069, SAMPLER_BINDING: 0x8919, ARRAY_BUFFER_BINDING: 0x8894,
  VERTEX_ARRAY_BINDING: 0x85B5, VIEWPORT: 0x0BA2, SCISSOR_BOX: 0x0C10, BLEND_SRC_RGB: 0x80C9,
  BLEND_DST_RGB: 0x80C8, BLEND_SRC_ALPHA: 0x80CB, BLEND_DST_ALPHA: 0x80CA, BLEND_EQUATION: 0x8009,
  BLEND_EQUATION_RGB: 0x8009, BLEND_EQUATION_ALPHA: 0x883D, ELEMENT_ARRAY_BUFFER_BINDING: 0x8895,
  FRAMEBUFFER_BINDING: 0x8CA6, DRAW_FRAMEBUFFER_BINDING: 0x8CA6, READ_FRAMEBUFFER_BINDING: 0x8CAA,
  COMPILE_STATUS: 0x8B81, LINK_STATUS: 0x8B82, DELETE_STATUS: 0x8B80, VALIDATE_STATUS: 0x8B83,
  ATTACHED_SHADERS: 0x8B85, RGBA: 0x1908, RGB: 0x1907, UNSIGNED_BYTE: 0x1401, FLOAT: 0x1406,
  TEXTURE_2D: 0x0DE1, TEXTURE_MAG_FILTER: 0x2800, FRAMEBUFFER: 0x8D40, READ_FRAMEBUFFER: 0x8CA8,
  DRAW_FRAMEBUFFER: 0x8CA9, ARRAY_BUFFER: 0x8892, STATIC_DRAW: 0x88E4, BLEND: 0x0BE2,
  MAX_TEXTURE_SIZE: 0x0D33, CULL_FACE: 0x0B44, COLOR_ATTACHMENT0: 0x8CE0, FRAMEBUFFER_COMPLETE: 0x8CD5,
  TRIANGLES: 0x0004, SYNC_GPU_COMMANDS_COMPLETE: 0x9117, FRAMEBUFFER_ATTACHMENT_OBJECT_NAME: 0x8CD1,
  FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE: 0x8CD0, DEPTH_STENCIL_ATTACHMENT: 0x821A,
};
const DEPTH_CLAMP = 0x864F, MAX_VIEWS_OVR = 0x9631, UNMASKED_RENDERER = 0x9246;
const SYNC_PNAMES = new Set([K.ACTIVE_TEXTURE, K.READ_BUFFER, K.MAX_DRAW_BUFFERS, K.BLEND_SRC_RGB, K.BLEND_DST_RGB,
  K.BLEND_SRC_ALPHA, K.BLEND_DST_ALPHA, K.BLEND_EQUATION_RGB, K.BLEND_EQUATION_ALPHA]);
const QUEUED = ['bindFramebuffer', 'deleteFramebuffer', 'drawBuffers', 'readBuffer', 'blitFramebuffer', 'viewport',
  'scissor', 'uniform1i', 'uniform4fv', 'uniformMatrix4fv', 'drawArrays', 'drawElements', 'drawArraysInstanced',
  'bindTexture', 'activeTexture', 'bufferData', 'bufferSubData', 'texImage2D', 'texSubImage2D',
  'bindVertexArray', 'bindBuffer', 'enable', 'disable', 'blendFuncSeparate', 'blendEquationSeparate',
  'linkProgram', 'compileShader', 'fenceSync', 'flush', 'clear'];

// A fake context class. The fake "GPU process" counts queued calls; a sync call spins
// spinBase + queued * spinPerCall ms (the drain), so the drain lands on the first sync after
// a burst of draws -- small numbers, the whole suite stays well under a second of CPU.
function makeEnv(search, { spinBase = 0, spinPerCall = 0, ua = FX_UA, raf = true, profParam } = {}) {
  let queued = 0;
  const spin = (ms) => { if (ms <= 0) return; const t = performance.now(); while (performance.now() - t < ms); };
  const sync = () => { spin(spinBase + queued * spinPerCall); queued = 0; };
  class WebGL2RenderingContext { constructor() { this.canvas = { width: 800, height: 600 }; } }
  Object.assign(WebGL2RenderingContext, K);
  const P = WebGL2RenderingContext.prototype;
  for (const m of QUEUED) P[m] = function () { queued++; };
  P.getParameter = function (p) { if (SYNC_PNAMES.has(p)) sync(); return p === K.MAX_DRAW_BUFFERS ? 8 : 0; };
  P.getTexParameter = function () { sync(); return 0; };
  P.getFramebufferAttachmentParameter = function () { return null; };
  P.isEnabled = function () { return true; };
  P.isProgram = function () { return true; };
  P.isContextLost = function () { return false; };
  P.checkFramebufferStatus = function () { sync(); return K.FRAMEBUFFER_COMPLETE; };
  P.getError = function () { sync(); return 0; };
  P.readPixels = function () { if (typeof arguments[6] !== 'number') sync(); else queued++; };
  P.clientWaitSync = function () { return 0x911A; };
  P.useProgram = function (prog) { if (prog && prog.pending) { prog.pending = false; spin(0.5); } queued++; };
  P.getProgramParameter = function (prog) { if (prog && prog.pending) { prog.pending = false; spin(0.5); } return true; };
  P.getShaderParameter = function (sh) { if (sh && sh.pending) { sh.pending = false; spin(0.2); } return true; };
  const depthClamp = new (class EXT_depth_clamp {})();
  P.getExtension = function (n) {
    if (n === 'EXT_test') return new (class EXT_test { fooEXT() { queued++; } })();
    if (n === 'WEBGL_multi_draw') return new (class WEBGL_multi_draw { multiDrawArraysWEBGL() { queued++; } })();
    if (n === 'EXT_depth_clamp' || n === 'EXT_DEPTH_CLAMP') return depthClamp; // the same object each time
    return null;
  };
  Object.defineProperty(P, 'drawingBufferWidth', { get() { return 800; }, configurable: true, enumerable: true });
  const before = new Map(Object.getOwnPropertyNames(P).map((k) => [k, Object.getOwnPropertyDescriptor(P, k)]));
  const out = [];
  const hud = { textContent: '' };
  const loc = { search, href: 'http://localhost:8831/app/' + search };
  let rafQ = [], ts = 0;
  class ImageData { constructor(w, h) { this.width = w; this.height = h; this.data = new Uint8ClampedArray(w * h * 4); } }
  const ctx = {
    WebGL2RenderingContext, ImageData, performance, queueMicrotask, URL, URLSearchParams, ArrayBuffer,
    console: { log: (...a) => out.push(a.join(' ')), warn() {}, error() {} },
    location: loc,
    navigator: { userAgent: ua },
    history: { state: null, replaceState(s, t, u) { const x = new URL(u); loc.search = x.search; loc.href = u; } },
    document: { getElementById: (id) => (id === 'profhud' ? hud : null) },
  };
  if (raf) ctx.requestAnimationFrame = (cb) => { rafQ.push(cb); return rafQ.length; };
  if (profParam !== undefined) ctx.__glsyncProfParam = profParam; // what serve.py --prof-param injects
  ctx.window = ctx;
  vm.createContext(ctx);
  return {
    ctx, out, hud, P, before, loc,
    load() { vm.runInContext(SRC, ctx, { filename: 'glsync.js' }); return new WebGL2RenderingContext(); },
    log(s) { ctx.console.log(s); }, // what the page's Module.print does with a PROF line
    paint() { const q = rafQ; rafQ = []; ts += 16.7; for (const cb of q) cb(ts); return q.length; }, // a refresh tick
    rafPending: () => rafQ.length,
  };
}
const tick = () => new Promise((r) => setTimeout(r, 0)); // the app's emscripten_sleep(0) yield

// One MOCK heavy-overlay frame (harness self-test only, see the top) with NO state shim:
// 13 sync calls, 16 on a bake frame.
const heap = new Uint8Array(1 << 20), m16 = new Float32Array(16);
function heavyOverlayFrame(gl, bake) {
  // vtkOpenGLRenderWindow::Start: RenderFramebuffer->Bind() (READ_BUFFER) + ActivateDrawBuffer (MAX_DRAW_BUFFERS)
  gl.bindFramebuffer(K.DRAW_FRAMEBUFFER, 1); gl.bindFramebuffer(K.READ_FRAMEBUFFER, 1);
  gl.getParameter(K.READ_BUFFER); gl.getParameter(K.MAX_DRAW_BUFFERS); gl.drawBuffers([K.COLOR_ATTACHMENT0]);
  if (bake) { // vtkShadowMapBakerPass: FBO Bind, ActivateBuffers, StartNonOrtho's CheckFramebufferStatus
    gl.bindFramebuffer(K.FRAMEBUFFER, 5); gl.getParameter(K.READ_BUFFER); gl.getParameter(K.MAX_DRAW_BUFFERS);
    gl.drawBuffers([K.COLOR_ATTACHMENT0]); gl.checkFramebufferStatus(K.FRAMEBUFFER);
    gl.bindFramebuffer(K.FRAMEBUFFER, 1);
  }
  for (let d = 0; d < 20; d++) { // 3-D + overlay draws
    for (let i = 0; i < 32; i++) gl.uniformMatrix4fv(null, false, m16);
    for (let i = 0; i < 96; i++) gl.bindTexture(K.TEXTURE_2D, null);
    gl.drawArraysInstanced(K.TRIANGLES, 0, 3, 100);
  }
  for (let u = 0; u < 16; u++) gl.bufferSubData(K.ARRAY_BUFFER, 0, heap, 0, 16384);
  gl.texImage2D(K.TEXTURE_2D, 0, K.RGBA, 128, 128, 0, K.RGBA, K.UNSIGNED_BYTE, heap, 0); // an overlay texture upload (128x128 RGBA)
  // ImGui_ImplOpenGL3_RenderDrawData backup
  for (const p of [K.ACTIVE_TEXTURE, K.CURRENT_PROGRAM, K.TEXTURE_BINDING_2D, K.SAMPLER_BINDING, K.ARRAY_BUFFER_BINDING,
                   K.VERTEX_ARRAY_BINDING, K.VIEWPORT, K.SCISSOR_BOX, K.BLEND_SRC_RGB, K.BLEND_DST_RGB,
                   K.BLEND_SRC_ALPHA, K.BLEND_DST_ALPHA, K.BLEND_EQUATION_RGB, K.BLEND_EQUATION_ALPHA]) gl.getParameter(p);
  for (let i = 0; i < 5; i++) gl.isEnabled(K.BLEND);
  gl.isProgram({});
  for (let i = 0; i < 64; i++) gl.drawArraysInstanced(K.TRIANGLES, 0, 3, 1);
  // vtkOpenGLRenderWindow::Frame: DisplayFramebuffer->Bind, ActivateDrawBuffer, RenderFramebuffer->Bind(READ), blit
  gl.bindFramebuffer(K.DRAW_FRAMEBUFFER, 2); gl.bindFramebuffer(K.READ_FRAMEBUFFER, 2);
  gl.getParameter(K.READ_BUFFER); gl.getParameter(K.MAX_DRAW_BUFFERS); gl.drawBuffers([K.COLOR_ATTACHMENT0]);
  gl.bindFramebuffer(K.READ_FRAMEBUFFER, 1); gl.getParameter(K.READ_BUFFER); gl.readBuffer(K.COLOR_ATTACHMENT0);
  gl.blitFramebuffer(0, 0, 1, 1, 0, 0, 1, 1, 0x4000, 0x2600);
  // BlitDisplayFramebuffer: DisplayFramebuffer->Bind(READ) then blit to the canvas (default framebuffer)
  gl.bindFramebuffer(K.READ_FRAMEBUFFER, 2); gl.getParameter(K.READ_BUFFER);
  gl.bindFramebuffer(K.DRAW_FRAMEBUFFER, null);
  gl.blitFramebuffer(0, 0, 1, 1, 0, 0, 1, 1, 0x4000, 0x2600);
}
const PROF = (n) => 'PROF n=' + n + ' fps=60.0 frame=16.7ms (mock timing line)';

// The webgl_state_shadow.js state shim in miniature: answers SCISSOR_BOX / VIEWPORT / BLEND_* without
// calling down, installed AFTER glsync (as the module's --pre-js is), so glsync never sees them.
function installMiniShim(P) {
  const real = P.getParameter, mine = new Set([K.SCISSOR_BOX, K.VIEWPORT, K.BLEND_SRC_RGB, K.BLEND_DST_RGB,
    K.BLEND_SRC_ALPHA, K.BLEND_DST_ALPHA, K.BLEND_EQUATION_RGB, K.BLEND_EQUATION_ALPHA]);
  P.getParameter = function (p) { return mine.has(p) ? 0 : real.apply(this, arguments); };
}

// Frames as an Asyncify app runs them: one task per frame (the harness prints a PROF line every 10
// frames, at the top of the task), the yield, then one refresh tick (paint) unless skipped.
async function runFrames(env, gl, from, to, { bakeEvery = 6, prof = true, paintEvery = 1 } = {}) {
  for (let n = from; n <= to; n++) {
    if (prof && n > from && (n - 1) % 10 === 0) env.log(PROF(n - 1));
    heavyOverlayFrame(gl, n % bakeEvery === 0);
    await tick();
    if (n % paintEvery === 0) env.paint();
  }
  if (prof && to % 10 === 0) env.log(PROF(to));
}
const row = (env, key, cls) => env.ctx.__glsync.table().find((r) => r.key === key && (!cls || r.cls === cls));
const field = (line, re) => { const m = re.exec(line); assert.ok(m, 'no ' + re + ' in: ' + line); return m; };
const lineAfter = (env, prof) => env.out[env.out.indexOf(prof) + 1];
// the breakdown sums to lo: chunk + sync + auto + F + one present flush each
const assertSums = (w, what) => assert.equal(w.lo, w.chunk + w.sync + w.auto + w.F + w.presents, what + ': ' + JSON.stringify(w));

// ---------------------------------------------------------------------------------------------
const cases = [];
const test = (name, fn) => cases.push([name, fn]);

test('inactive without ?glsync (and with ?glsync=0 / =off / =false): prototype untouched, nothing printed', async () => {
  for (const search of ['', '?prof', '?glshim=0', '?glsync=0', '?glsync=off', '?glsync=false']) {
    const env = makeEnv(search);
    const gl = env.load();
    for (const [k, d] of env.before) {
      const now = Object.getOwnPropertyDescriptor(env.P, k);
      assert.equal(now.value, d.value, search + ': ' + k + ' was wrapped');
      assert.equal(now.get, d.get, search + ': ' + k + ' getter was wrapped');
    }
    heavyOverlayFrame(gl, false); await tick(); env.log(PROF(10));
    assert.equal(env.ctx.__glsync, undefined, search + ': __glsync defined');
    assert.equal(env.loc.search, search, search + ': URL changed');
    assert.equal(env.rafPending(), 0, search + ': rAF requested');
    assert.deepEqual(env.out, [PROF(10)], search + ': unexpected output');
  }
});

test('active with ?glsync: every method + drawingBuffer getters wrapped, URL left alone, startup line', async () => {
  const env = makeEnv('?glsync');
  env.load();
  let fns = 0;
  for (const [k, d] of env.before) {
    const now = Object.getOwnPropertyDescriptor(env.P, k);
    if (typeof d.value === 'function' && k !== 'constructor') { fns++; assert.notEqual(now.value, d.value, k + ' not wrapped'); }
  }
  assert.ok(fns >= 40, 'only ' + fns + ' methods');
  assert.notEqual(Object.getOwnPropertyDescriptor(env.P, 'drawingBufferWidth').get, env.before.get('drawingBufferWidth').get);
  // ?prof is added only on request now (?glsync=prof / serve.py --prof-param): see 'prof-param' below
  assert.equal(env.loc.search, '?glsync', 'URL changed: ' + env.loc.search);
  assert.match(env.out[0], new RegExp('^glsync: census on -- Firefox 156 sync model; shmem=100000 \\(must match ' +
    'about:config webgl\\.out-of-process\\.shmem-size, read at context creation: reload after changing it\\); ' +
    'list >= 0\\.1 ms/frame; wrapper ovh~ timed \\d+ / queued \\d+ / keyed \\d+ / draw \\d+ ns; ' +
    'paint clock requestAnimationFrame; frame clock auto \\(PROF lines, else every 60 paints\\)$'));
  assert.equal(env.ctx.__glsync.clock(), 'raf');
});

test('prof-param: ?prof is added only for ?glsync=prof or an injected __glsyncProfParam', async () => {
  const s = (search, o) => { const env = makeEnv(search, o); env.load(); return [env.loc.search, env.out[0]]; };
  assert.equal(s('?glsync')[0], '?glsync');
  assert.equal(s('?glsync=seq')[0], '?glsync=seq');
  const [q1, l1] = s('?glsync=prof,seq');
  assert.equal(new URLSearchParams(q1).has('prof'), true, q1);
  assert.match(l1, /; adds \?prof$/);
  // serve.py --prof-param prof injects window.__glsyncProfParam = "prof" ahead of glsync.js
  const [q2, l2] = s('?glsync', { profParam: 'prof' });
  assert.equal(new URLSearchParams(q2).has('prof'), true, q2);
  assert.match(l2, /; adds \?prof$/);
  // another name is honoured; ?glsync=prof wins over the injected name
  assert.equal(new URLSearchParams(s('?glsync', { profParam: 'timing' })[0]).has('timing'), true);
  assert.equal(new URLSearchParams(s('?glsync=prof', { profParam: 'stats' })[0]).has('stats'), false);
  // a malformed injected name is ignored, not written into the URL
  assert.equal(s('?glsync', { profParam: 'a b<' })[0], '?glsync');
  assert.equal(s('?glsync', { profParam: 42 })[0], '?glsync');
  // an explicit prof, profhud or glcount (or the requested name itself) is left alone
  assert.equal(s('?glsync=prof&prof=compact')[0], '?glsync=prof&prof=compact');
  assert.equal(s('?glsync=prof&profhud=compact')[0], '?glsync=prof&profhud=compact');
  assert.equal(s('?glsync&glcount', { profParam: 'prof' })[0], '?glsync&glcount');
  assert.equal(s('?glsync&timing', { profParam: 'timing' })[0], '?glsync&timing');
  assert.equal(s('?glsync&prof', { profParam: 'timing' })[0], '?glsync&prof');
  assert.equal(s('?glsync&prof=1', { profParam: 'prof' })[0], '?glsync&prof=1');
});

test('options: 1/on/true only enable; thr=, bare numbers, shmem=, unknown tokens; non-Firefox UA warning', async () => {
  for (const v of ['1', 'on', 'true', '']) {
    const env = makeEnv('?glsync=' + v); env.load();
    assert.equal(env.ctx.__glsync.opts.thr, 0.1, v + ': threshold changed');
    assert.equal(env.ctx.__glsync.opts.shmem, 100000);
    assert.doesNotMatch(env.out[0], /ignored/, v);
  }
  const o = (s) => { const env = makeEnv('?glsync=' + s); env.load(); return [env.ctx.__glsync.opts, env.out[0]]; };
  assert.equal(o('thr=1')[0].thr, 1);
  assert.equal(o('1.0')[0].thr, 1);
  assert.equal(o('0.5,seq')[0].thr, 0.5);
  assert.equal(o('2')[0].thr, 2);
  const [os, line] = o('shmem=1000000,seq');
  assert.equal(os.shmem, 1000000); assert.equal(os.seq, true);
  assert.match(line, /; shmem=1000000 \(must match about:config/);
  assert.match(o('seq,bogus,x=1')[1], /; ignored: bogus,x=1;/);
  // not Firefox: the startup line and every GLSYNC line say so
  const env = makeEnv('?glsync=nohud', { ua: CR_UA });
  const gl = env.load();
  assert.match(env.out[0], /^glsync: census on \(model is Firefox-only\) -- /);
  heavyOverlayFrame(gl, false); await tick(); env.paint(); env.log(PROF(1));
  assert.match(lineAfter(env, PROF(1)), /^GLSYNC n=1 f=1 warming \(model is Firefox-only\) \| gl /);
  const env3 = makeEnv('?glsync=nohud'); const gl3 = env3.load();
  heavyOverlayFrame(gl3, false); await tick(); env3.paint(); env3.log(PROF(1));
  assert.doesNotMatch(lineAfter(env3, PROF(1)), /Firefox-only/);
});

test('GLSYNC line right after each PROF n= line; per-method + pname aggregation; sync totals', async () => {
  const env = makeEnv('?glsync=seq');
  const gl = env.load();
  await runFrames(env, gl, 1, 20);
  const iP10 = env.out.indexOf(PROF(10)), iP20 = env.out.indexOf(PROF(20));
  assert.ok(iP10 > 0 && iP20 > iP10);
  assert.match(env.out[iP10 + 1], /^GLSYNC n=10 f=10 warming \| gl /);
  assert.match(env.out[iP10 + 2], /^GLSYNC seq n=10 worst task: /);
  const l20 = env.out[iP20 + 1];
  assert.match(l20, /^GLSYNC n=20 f=10 \| gl /);
  // mock frames 11..20 (self-test): 8 non-bake (13 sync) + 2 bake (16 sync) = 136 -> 13.6 per frame
  assert.equal(field(l20, / sync ([\d.]+)x /)[1], '13.6');
  // one paint per frame -> one present per frame, each frame's flushes on its own present
  assert.match(l20, / \| presents 10\/10f flush~/);
  assertSums(env.ctx.__glsync.lastWindow(), 'n=20');
  // presents over the budget (self-test figures): a frame's flushes are 4 chunk + 5 sync (7 on a
  // bake frame) + its present = 10 (12), so the bake frames' presents (2 in 11..20, 1 in 1..10)
  // are over even at lo; at hi (every frame ends with a draw: race) all of them are
  assert.match(env.out[iP10 + 1], / >10 10%\/100% \(lo 1 \/ hi 10\) \[/);
  assert.match(l20, / >10 20%\/100% \(lo 2 \/ hi 10\) \[/);
  assert.deepEqual([env.ctx.__glsync.lastWindow().overLo, env.ctx.__glsync.lastWindow().overHi], [2, 10]);
  // exact per-key totals over 20 frames (3 bake frames: 6, 12, 18)
  const expect = [
    ['getParameter(READ_BUFFER)', 'S', 4 * 20 + 3], ['getParameter(MAX_DRAW_BUFFERS)', 'S', 2 * 20 + 3],
    ['getParameter(ACTIVE_TEXTURE)', 'S', 20], ['getParameter(BLEND_SRC_RGB)', 'S', 20],
    ['checkFramebufferStatus', 'S', 3], ['getParameter(CURRENT_PROGRAM)', 'C', 20],
    ['getParameter(VIEWPORT)', 'C', 20], ['getParameter(SCISSOR_BOX)', 'C', 20], ['isEnabled(BLEND)', 'C', 100],
    ['isProgram', 'C', 20], ['bindTexture', 'A', 20 * 96 * 20], ['uniformMatrix4fv', 'A', 20 * 32 * 20],
    ['drawArraysInstanced', 'A', (20 + 64) * 20], ['blitFramebuffer', 'A', 40], ['bindFramebuffer', 'A', 7 * 20 + 2 * 3],
  ];
  for (const [key, cls, n] of expect) {
    const r = row(env, key, cls);
    assert.ok(r, 'no row ' + key + '[' + cls + ']');
    assert.equal(r.n, n, key + ' count');
  }
  // the window line lists every S key (with the per-frame count) even below the ms threshold
  assert.match(l20, /getParameter\(READ_BUFFER\)\[S\] 4\.2x /);
  assert.match(l20, /checkFramebufferStatus\[S\] 0\.2x /);
  assert.doesNotMatch(l20, /A untimed/);
  // HUD: a short GLSYNC line appended in a microtask
  await tick();
  assert.match(env.hud.textContent, /\nGLSYNC sync 13\.6x [\d.]+ms flush~[\d.]+\/[\d.]+ >10 \d+% top /);
});

test('pname decoding: names from the constants, alias fix-ups, extension enums, unknown -> hex', async () => {
  const env = makeEnv('?glsync=all,nohud');
  const gl = env.load();
  for (const p of [K.READ_BUFFER, K.MAX_DRAW_BUFFERS, K.ACTIVE_TEXTURE, K.BLEND_EQUATION_RGB, K.FRAMEBUFFER_BINDING,
                   0x84FF, 0x9999]) gl.getParameter(p);
  gl.getTexParameter(K.TEXTURE_2D, K.TEXTURE_MAG_FILTER);
  await tick(); env.log(PROF(1));
  const keys = env.ctx.__glsync.table().map((r) => r.key + '[' + r.cls + ']');
  for (const k of ['getParameter(READ_BUFFER)[S]', 'getParameter(MAX_DRAW_BUFFERS)[S]', 'getParameter(ACTIVE_TEXTURE)[S]',
                   'getParameter(BLEND_EQUATION_RGB)[S]', 'getParameter(DRAW_FRAMEBUFFER_BINDING)[C]',
                   'getParameter(MAX_TEXTURE_MAX_ANISOTROPY_EXT)[S]', 'getParameter(0x9999)[S]',
                   'getTexParameter(TEXTURE_MAG_FILTER)[S]'])
    assert.ok(keys.includes(k), k + ' missing from ' + keys.join(', '));
});

test('per-call classes: S1 after link/compile, readPixels, clientWaitSync, DOM / ImageData texImage, extensions', async () => {
  const env = makeEnv('?glsync=all,nohud');
  const gl = env.load();
  const prog = { pending: true }, sh = { pending: true };
  gl.linkProgram(prog); gl.useProgram(prog); gl.getProgramParameter(prog, K.LINK_STATUS); gl.useProgram(prog);
  gl.compileShader(sh); gl.getShaderParameter(sh, K.COMPILE_STATUS); gl.getShaderParameter(sh, K.COMPILE_STATUS);
  gl.readPixels(0, 0, 1, 1, K.RGBA, K.UNSIGNED_BYTE, new Uint8Array(4));
  gl.readPixels(0, 0, 1, 1, K.RGBA, K.UNSIGNED_BYTE, 0);
  gl.clientWaitSync({}, 0, 0); gl.clientWaitSync({}, 0, 1000);
  gl.texImage2D(K.TEXTURE_2D, 0, K.RGBA, K.RGBA, K.UNSIGNED_BYTE, { width: 1, height: 1 }); // <img>/ImageBitmap overload
  gl.texSubImage2D(K.TEXTURE_2D, 0, 0, 0, K.RGBA, K.UNSIGNED_BYTE, { width: 1, height: 1 });
  gl.texImage2D(K.TEXTURE_2D, 0, K.RGBA, 4, 4, 0, K.RGBA, K.UNSIGNED_BYTE, new Uint8Array(64));
  gl.getExtension('EXT_test').fooEXT();
  await tick(); env.log(PROF(1));
  const has = (key, cls, n) => { const r = row(env, key, cls); assert.ok(r, key + '[' + cls + '] missing'); assert.equal(r.n, n, key + '[' + cls + ']'); };
  has('useProgram', 'S', 1); has('useProgram', 'A', 1);       // only the first use after linkProgram syncs
  has('getProgramParameter(LINK_STATUS)', 'C', 1);
  has('getShaderParameter(COMPILE_STATUS)', 'S', 1); has('getShaderParameter(COMPILE_STATUS)', 'C', 1);
  has('readPixels(cpu)', 'S', 1); has('readPixels', 'A', 1);
  has('clientWaitSync(wait)', 'S', 1); has('clientWaitSync', 'C', 1);
  has('texImage2D(dom)', 'F', 1); has('texSubImage2D(dom)', 'F', 1); has('texImage2D', 'A', 1);
  has('getExtension(enable)', 'A', 1); has('EXT_test.fooEXT', 'A', 1);

  // ImageData is uploaded inline: queued bytes (w*h*4 + a command), no flush
  const env2 = makeEnv('?glsync=all,nohud');
  const g2 = env2.load(), G2 = env2.ctx.__glsync;
  g2.texImage2D(K.TEXTURE_2D, 0, K.RGBA, K.RGBA, K.UNSIGNED_BYTE, new env2.ctx.ImageData(10, 10));
  assert.equal(G2.model().pend, 400 + 32);
  // the first useProgram after a link waits (S) and then queues UseProgram: the next sync is a drain
  const p2 = { pending: true };
  g2.linkProgram(p2); g2.useProgram(p2);
  const m = G2.model();
  assert.equal(m.sync, 1, 'link wait flushed the queue'); assert.equal(m.pend, 32, 'UseProgram queued after the wait');
  g2.getParameter(K.READ_BUFFER);
  assert.equal(G2.model().sync, 2);
  await tick(); env2.log(PROF(1));
  assert.ok(row(env2, 'texImage2D(imagedata)', 'A'));
});

test('conditional classes: getExtension enable, extension pnames, OBJECT_NAME, drawingBufferWidth after resize', async () => {
  const env = makeEnv('?glsync=all,nohud');
  const gl = env.load();
  gl.getParameter(DEPTH_CLAMP);                  // S: EXT_depth_clamp not enabled yet
  gl.getParameter(MAX_VIEWS_OVR);                // S: OVR_multiview2 never enabled
  gl.getParameter(UNMASKED_RENDERER);            // C: client-side error without WEBGL_debug_renderer_info
  gl.getExtension('EXT_depth_clamp');            // first enable: queues RequestExtension (A)
  gl.getExtension('EXT_DEPTH_CLAMP');            // same extension (names are case-insensitive): C
  gl.getExtension('NOPE');                       // unsupported -> null: C
  gl.getParameter(DEPTH_CLAMP); gl.getParameter(DEPTH_CLAMP); // now answered client-side
  const ON = K.FRAMEBUFFER_ATTACHMENT_OBJECT_NAME;
  gl.getFramebufferAttachmentParameter(K.FRAMEBUFFER, K.COLOR_ATTACHMENT0, ON);           // default fb: S
  gl.bindFramebuffer(K.FRAMEBUFFER, 9);
  gl.getFramebufferAttachmentParameter(K.FRAMEBUFFER, K.COLOR_ATTACHMENT0, ON);           // FBO: C
  gl.getFramebufferAttachmentParameter(K.FRAMEBUFFER, K.DEPTH_STENCIL_ATTACHMENT, ON);    // DEPTH_STENCIL: S
  gl.getFramebufferAttachmentParameter(K.READ_FRAMEBUFFER, K.COLOR_ATTACHMENT0, ON);      // read FBO: C
  gl.bindFramebuffer(K.READ_FRAMEBUFFER, null);
  gl.getFramebufferAttachmentParameter(K.READ_FRAMEBUFFER, K.COLOR_ATTACHMENT0, ON);      // default read: S
  gl.getFramebufferAttachmentParameter(K.FRAMEBUFFER, K.COLOR_ATTACHMENT0, K.FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE); // S
  gl.deleteFramebuffer(9);                       // deleting the bound FBO rebinds the canvas
  gl.getFramebufferAttachmentParameter(K.FRAMEBUFFER, K.COLOR_ATTACHMENT0, ON);           // S again
  void gl.drawingBufferWidth; void gl.drawingBufferWidth; // S once, then C
  gl.canvas.width = 1024;
  void gl.drawingBufferWidth; void gl.drawingBufferWidth; // the resize clears the cache: S once more
  await tick(); env.log(PROF(1));
  const has = (key, cls, n) => { const r = row(env, key, cls); assert.ok(r, key + '[' + cls + '] missing'); assert.equal(r.n, n, key + '[' + cls + ']'); };
  has('getParameter(DEPTH_CLAMP_EXT)', 'S', 1); has('getParameter(DEPTH_CLAMP_EXT)', 'C', 2);
  has('getParameter(MAX_VIEWS_OVR)', 'S', 1);
  has('getParameter(UNMASKED_RENDERER_WEBGL)', 'C', 1);
  has('getExtension(enable)', 'A', 1); has('getExtension', 'C', 2);
  has('getFramebufferAttachmentParameter(FRAMEBUFFER_ATTACHMENT_OBJECT_NAME)', 'S', 4);
  has('getFramebufferAttachmentParameter(FRAMEBUFFER_ATTACHMENT_OBJECT_NAME)', 'C', 2);
  has('getFramebufferAttachmentParameter(FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE)', 'S', 1);
  has('drawingBufferWidth', 'S', 2); has('drawingBufferWidth', 'C', 2);
  assert.deepEqual([...env.ctx.__glsync.model().extensions(gl)], ['ext_depth_clamp']);
});

test('drawingBufferWidth: the first paint, and the first paint after a resize, refill the size cache', async () => {
  const env = makeEnv('?glsync=all,nohud');
  const gl = env.load(), G = env.ctx.__glsync;
  const nS = () => G.table().filter((r) => r.key === 'drawingBufferWidth' && r.cls === 'S').reduce((x, r) => x + r.n, 0);
  const read = () => { const b = nS(); void gl.drawingBufferWidth; return nS() > b ? 'S' : 'C'; };
  gl.bindFramebuffer(K.FRAMEBUFFER, null); gl.drawArrays(K.TRIANGLES, 0, 3);
  await tick(); env.paint();
  assert.equal(read(), 'C', 'read after the first paint');       // UpdateWebRenderCanvasData fetched it
  gl.canvas.width = 1024;
  assert.equal(read(), 'S', 'read after a resize, before a paint'); // SetDimensions cleared it
  assert.equal(read(), 'C', 'second read');
  gl.canvas.height = 700; gl.drawArrays(K.TRIANGLES, 0, 3);
  await tick();
  assert.equal(read(), 'S', 'read after a resize and a draw, no paint yet');
  gl.canvas.width = 640; gl.drawArrays(K.TRIANGLES, 0, 3);
  await tick(); env.paint();
  assert.equal(read(), 'C', 'read after the paint that followed a resize');
  gl.canvas.width = 512; gl.getExtension('WEBGL_multi_draw').multiDrawArraysWEBGL(); // an extension draw dirties it too
  await tick(); env.paint();
  assert.equal(read(), 'C', 'read after a paint dirtied by an extension draw');
  await tick(); env.log(PROF(1));
  assert.equal(row(env, 'drawingBufferWidth', 'S').n, 2);
  assert.equal(row(env, 'drawingBufferWidth', 'C').n, 4);
});

test('flush model: rAF paint clock, carried flushes, race lo/hi, explicit flush, chunks, SyncPing', async () => {
  const env = makeEnv('?glsync=all,nohud');
  const gl = env.load(), G = env.ctx.__glsync;
  const flush = (n) => field(lineAfter(env, PROF(n)), / (presents \d+\/\d+f flush~\S+ max \S+ >10 \S+ \(lo \d+ \/ hi \d+\) \[[^\]]*\] shmem=\d+(?: ping20 \d+)?(?: syncping \d+)?)/)[1];
  const done = async (n, paint = true) => { await tick(); if (paint) env.paint(); env.log(PROF(n)); assertSums(G.lastWindow(), 'n=' + n); };
  // n=1: 12 x (draw to the canvas; sync query) -> 12 sync flushes + the present = 13 > 10: sync
  // present. Nothing is queued after the last sync, so the pending auto-flush cannot race: lo = hi.
  gl.bindFramebuffer(K.FRAMEBUFFER, null);
  for (let i = 0; i < 12; i++) { gl.drawArrays(K.TRIANGLES, 0, 3); gl.getParameter(K.READ_BUFFER); }
  assert.equal(env.rafPending(), 1, 'one rAF per dirty canvas');
  await done(1);
  assert.equal(flush(1), 'presents 1/1f flush~13.0/13.0 max 13/13 >10 100%/100% (lo 1 / hi 1) [0.0 chunk + 12.0 sync + 0.0 auto + 1 present; hi +0.0 race] shmem=100000');
  // n=2: 3 syncs, then a trailing draw: its auto-flush is pending with commands queued at the
  // tick -> lo 4 (expected: the paint runs first and cancels it) / hi 5 (bound)
  for (let i = 0; i < 3; i++) { gl.drawArrays(K.TRIANGLES, 0, 3); gl.getParameter(K.READ_BUFFER); }
  gl.drawArrays(K.TRIANGLES, 0, 3);
  await done(2);
  assert.equal(flush(2), 'presents 1/1f flush~4.0/5.0 max 4/5 >10 0%/0% (lo 0 / hi 0) [0.0 chunk + 3.0 sync + 0.0 auto + 1 present; hi +1.0 race] shmem=100000');
  // n=3: draw into an FBO only (no present, no rAF); n=4: the next task starts with no paint in
  // between, so n=3's auto-flush certainly ran (auto 1), then the n=2 pattern
  gl.bindFramebuffer(K.FRAMEBUFFER, 7); gl.drawArrays(K.TRIANGLES, 0, 3);
  assert.equal(env.rafPending(), 0, 'an FBO draw requested a rAF');
  await tick();
  gl.bindFramebuffer(K.FRAMEBUFFER, null);
  for (let i = 0; i < 3; i++) { gl.drawArrays(K.TRIANGLES, 0, 3); gl.getParameter(K.READ_BUFFER); }
  gl.drawArrays(K.TRIANGLES, 0, 3);
  await done(4);
  assert.equal(flush(4), 'presents 1/2f flush~5.0/6.0 max 5/6 >10 0%/0% (lo 0 / hi 0) [0.0 chunk + 3.0 sync + 1.0 auto + 1 present; hi +1.0 race] shmem=100000');
  // n=5: a 300 KB upload first in an empty buffer gets its own buffer (no flush); the next command flushes it
  gl.getParameter(K.READ_BUFFER);
  gl.bufferData(K.ARRAY_BUFFER, new Uint8Array(300000), K.STATIC_DRAW);
  gl.drawArrays(K.TRIANGLES, 0, 3);
  await done(5);
  assert.equal(flush(5), 'presents 1/1f flush~2.0/3.0 max 2/3 >10 0%/0% (lo 0 / hi 0) [1.0 chunk + 0.0 sync + 0.0 auto + 1 present; hi +1.0 race] shmem=100000');
  // n=6: 3200 x uniformMatrix4fv(64 B) = 3200 x 96 B ~ 307 KB of commands -> 3 shmem overflows
  gl.bindFramebuffer(K.FRAMEBUFFER, null);
  for (let i = 0; i < 3200; i++) gl.uniformMatrix4fv(null, false, m16);
  gl.drawArrays(K.TRIANGLES, 0, 3);
  await done(6);
  assert.equal(flush(6), 'presents 1/1f flush~4.0/5.0 max 4/5 >10 0%/0% (lo 0 / hi 0) [3.0 chunk + 0.0 sync + 0.0 auto + 1 present; hi +1.0 race] shmem=100000');
  // n=7: a window with no draw to the canvas says so instead of a flush count
  gl.bindFramebuffer(K.FRAMEBUFFER, 3); gl.drawArrays(K.TRIANGLES, 0, 3);
  await tick(); env.paint(); env.log(PROF(7));
  assert.match(lineAfter(env, PROF(7)), / \| presents 0\/1f \(no paint of a drawn canvas\) shmem=100000 \| /);
  // n=8: WEBGL_multi_draw draws count as draws (AfterDrawCall). n=7's auto-flush ran before this
  // task (no present cancelled it): 1 + present = 2.
  gl.bindFramebuffer(K.FRAMEBUFFER, null); gl.getExtension('WEBGL_multi_draw').multiDrawArraysWEBGL();
  await done(8);
  assert.equal(flush(8), 'presents 1/1f flush~2.0/3.0 max 2/3 >10 0%/0% (lo 0 / hi 0) [0.0 chunk + 0.0 sync + 1.0 auto + 1 present; hi +1.0 race] shmem=100000');
  // n=9: an explicit gl.flush() with nothing queued still costs one flush (it queues a Flush command)
  gl.flush();
  assert.equal(G.model().F, 1);
  gl.drawArrays(K.TRIANGLES, 0, 3);
  await done(9);
  assert.equal(flush(9), 'presents 1/1f flush~2.0/3.0 max 2/3 >10 0%/0% (lo 0 / hi 0) [0.0 chunk + 0.0 sync + 0.0 auto + 1.0 F + 1 present; hi +1.0 race] shmem=100000');
  // n=10..11: a skipped paint. Frame 10 (6 syncs + trailing draw) gets no paint; frame 11 starts,
  // so frame 10's auto-flush ran (auto 1); frame 11 adds 6 syncs. One present carries both frames:
  // 6 + 1 + 6 + 1 = 14 > 10 -> a sync present that the per-task model would have missed.
  for (let i = 0; i < 6; i++) { gl.drawArrays(K.TRIANGLES, 0, 3); gl.getParameter(K.READ_BUFFER); }
  gl.drawArrays(K.TRIANGLES, 0, 3);
  await tick(); // no paint
  for (let i = 0; i < 6; i++) { gl.drawArrays(K.TRIANGLES, 0, 3); gl.getParameter(K.READ_BUFFER); }
  gl.drawArrays(K.TRIANGLES, 0, 3);
  await done(11);
  assert.equal(flush(11), 'presents 1/2f flush~14.0/15.0 max 14/15 >10 100%/100% (lo 1 / hi 1) [0.0 chunk + 12.0 sync + 1.0 auto + 1 present; hi +1.0 race] shmem=100000');
  // n=12: one present per paint of a dirty canvas: a second tick with no new draw requests no rAF
  gl.drawArrays(K.TRIANGLES, 0, 3);
  await tick(); env.paint(); await tick(); assert.equal(env.paint(), 0, 'rAF requested for a clean canvas');
  env.log(PROF(12));
  assert.match(flush(12), /^presents 1\/1f flush~1\.0\/2\.0 /);
  // n=13: 75 syncs with commands queued: the counter reaches 20 (async Ping) and passes 70
  // (SyncPing: synchronous, resets it), so the present sees 4 + 1 <= 10 although 76 flushes happened
  gl.bindFramebuffer(K.FRAMEBUFFER, 7);
  for (let i = 0; i < 75; i++) { gl.drawArrays(K.TRIANGLES, 0, 3); gl.getParameter(K.READ_BUFFER); }
  gl.bindFramebuffer(K.FRAMEBUFFER, null); gl.drawArrays(K.TRIANGLES, 0, 3);
  await done(13);
  assert.equal(flush(13), 'presents 1/1f flush~76.0/77.0 max 76/77 >10 0%/0% (lo 0 / hi 0) [0.0 chunk + 75.0 sync + 0.0 auto + 1 present; hi +1.0 race] shmem=100000 ping20 1 syncping 1');

  // ?glsync=shmem=: a 1 MB shmem holds n=6's 307 KB without an overflow
  const big = makeEnv('?glsync=all,nohud,shmem=1000000');
  const gb = big.load();
  gb.bindFramebuffer(K.FRAMEBUFFER, null);
  for (let i = 0; i < 3200; i++) gb.uniformMatrix4fv(null, false, m16);
  gb.drawArrays(K.TRIANGLES, 0, 3);
  await tick(); big.paint(); big.log(PROF(1));
  assert.match(lineAfter(big, PROF(1)), / \| presents 1\/1f flush~1\.0\/2\.0 max 1\/2 >10 0%\/0% \(lo 0 \/ hi 0\) \[0\.0 chunk \+ 0\.0 sync \+ 0\.0 auto \+ 1 present; hi \+1\.0 race\] shmem=1000000 \| /);
});

test('flush thresholds: 9 / 10 syncs before a paint (>10 budget), Ping at 20 flushes, SyncPing past 70', async () => {
  const env = makeEnv('?glsync=all,nohud');
  const gl = env.load(), G = env.ctx.__glsync;
  let n = 0;
  // k x (draw to the canvas; sync query): k sync flushes, nothing queued after the last one (no
  // race) unless a trailing draw is added; then one paint and its PROF line
  const win = async (k, trailing = false) => {
    gl.bindFramebuffer(K.FRAMEBUFFER, null);
    for (let i = 0; i < k; i++) { gl.drawArrays(K.TRIANGLES, 0, 3); gl.getParameter(K.READ_BUFFER); }
    if (trailing) gl.drawArrays(K.TRIANGLES, 0, 3);
    await tick(); env.paint(); env.log(PROF(++n));
    assertSums(G.lastWindow(), k + ' syncs');
    return field(lineAfter(env, PROF(n)), / \| (presents \S+ flush~.*? shmem=\d+(?: ping20 \d+)?(?: syncping \d+)?) \| /)[1];
  };
  const want = (lo, hi, over, k, tail = '') => 'presents 1/1f flush~' + lo + '.0/' + hi + '.0 max ' + lo + '/' + hi +
    ' >10 ' + over + ' [0.0 chunk + ' + k + '.0 sync + 0.0 auto + 1 present; hi +' + (hi - lo) + '.0 race] shmem=100000' + tail;
  // GetFrontBuffer: sync present when the counter is > 10, the present's own flush included
  assert.equal(await win(9), want(10, 10, '0%/0% (lo 0 / hi 0)', 9));          // 9 + 1 = 10: async
  assert.equal(await win(10), want(11, 11, '100%/100% (lo 1 / hi 1)', 10));    // 10 + 1 = 11: sync
  assert.equal(await win(9, true), want(10, 11, '0%/100% (lo 0 / hi 1)', 9));  // only the race tips it over
  // FlushPendingCmds: the async congestion Ping when the counter reaches exactly 20
  assert.equal(await win(19), want(20, 20, '100%/100% (lo 1 / hi 1)', 19));
  assert.equal(await win(20), want(21, 21, '100%/100% (lo 1 / hi 1)', 20, ' ping20 1'));
  // ... and the SyncPing once it passes 70, which resets the counter: 71 flushes leave it at 0,
  // so that present is async (1 <= 10); at 70 it is not reset yet
  assert.equal(await win(70), want(71, 71, '100%/100% (lo 1 / hi 1)', 70, ' ping20 1'));
  assert.equal(await win(71), want(72, 72, '0%/0% (lo 0 / hi 0)', 71, ' ping20 1 syncping 1'));
  assert.deepEqual([G.lastWindow().overLo, G.lastWindow().overHi, G.lastWindow().syncPing], [0, 0, 1]);
});

test('mock heavy-overlay frame (self-test): drain lands on ImGui ACTIVE_TEXTURE; a shim above glsync removes its keys', async () => {
  const env = makeEnv('?glsync=seq,stacks,nohud', { spinBase: 0.01, spinPerCall: 0.001 });
  const gl = env.load();
  await runFrames(env, gl, 1, 20);
  if (process.argv.includes('--show')) for (const l of env.out) console.log('       | ' + l);
  const l20 = lineAfter(env, PROF(20)), seq = env.out[env.out.indexOf(PROF(20)) + 2];
  assert.match(l20, /\| top: getParameter\(ACTIVE_TEXTURE\)\[S\] 1\.0x /, 'ACTIVE_TEXTURE is not the top key: ' + l20);
  assert.match(seq, /ACTIVE_TEXTURE\* /, 'ACTIVE_TEXTURE not marked drained: ' + seq);
  const drain = +field(l20, /drain~([\d.]+)/)[1];
  assert.ok(drain > 2, 'drain~' + drain);
  // ?glsync=stacks: the caller of the slow sync, with glsync's own frames dropped
  const st = env.out.find((l) => l.startsWith('GLSYNC stack getParameter(ACTIVE_TEXTURE): '));
  assert.ok(st, 'no stack line');
  assert.match(st, /: heavyOverlayFrame < runFrames/);
  assert.doesNotMatch(st, /stackHere|\brec\b/);
  // the same frames with the mini state shim on top: 13.6 -> 7.6 sync per frame, no BLEND_* keys
  const env2 = makeEnv('?glsync=nohud');
  const gl2 = env2.load();
  installMiniShim(env2.P);
  await runFrames(env2, gl2, 1, 20);
  const m20 = lineAfter(env2, PROF(20));
  assert.equal(field(m20, / sync ([\d.]+)x /)[1], '7.6');
  assert.doesNotMatch(m20, /BLEND_/);
  assert.equal(row(env2, 'getParameter(VIEWPORT)'), undefined);
  // every other paint skipped: half the presents, each carrying two frames' flushes
  const env3 = makeEnv('?glsync=nohud');
  const gl3 = env3.load();
  await runFrames(env3, gl3, 1, 20, { paintEvery: 2 });
  const w1 = lineAfter(env, PROF(20)), w2 = lineAfter(env3, PROF(20));
  assert.match(w2, / \| presents 5\/10f flush~/);
  // two frames per present: 9 + 1 auto + 9 (11 on a bake frame) flushes before the present: all
  // over the budget, and the 2 presents that carry a bake frame (12, 18) reach 20 (the async Ping)
  assert.match(w2, / >10 100%\/100% \(lo 5 \/ hi 5\) \[[^\]]*\] shmem=100000 ping20 2 \| /);
  const lo1 = +field(w1, /flush~([\d.]+)\//)[1], lo2 = +field(w2, /flush~([\d.]+)\//)[1];
  assert.ok(lo2 > 1.8 * lo1, 'skipped paints did not carry flushes: ' + lo1 + ' vs ' + lo2);
  assertSums(env3.ctx.__glsync.lastWindow(), 'paintEvery 2');
});

test('PROF paused / resumed restart the window (paused renders are not divided into it)', async () => {
  const env = makeEnv('?glsync=nohud');
  const gl = env.load();
  await runFrames(env, gl, 1, 10);
  env.log('PROF paused n=10');
  for (let i = 0; i < 5; i++) { heavyOverlayFrame(gl, false); heavyOverlayFrame(gl, false); await tick(); env.paint(); } // paused: renders, no frames
  env.log('PROF resumed n=10');
  await runFrames(env, gl, 11, 20);
  const l20 = lineAfter(env, PROF(20));
  assert.match(l20, /^GLSYNC n=20 f=10 \| /);
  assert.equal(field(l20, / sync ([\d.]+)x /)[1], '13.6'); // no paused renders folded in
  assert.match(l20, / \| presents 10\/10f /);
});

test('lite mode tags the gl ms "(A untimed)"; no rAF -> a paint assumed at each task end', async () => {
  const env = makeEnv('?glsync=lite,nohud');
  const gl = env.load();
  assert.match(env.out[0], /, lite;/);
  await runFrames(env, gl, 1, 10);
  assert.match(lineAfter(env, PROF(10)), /\| gl [\d.]+k?\/f [\d.]+ms \(A untimed\) \| sync /);
  const env2 = makeEnv('?glsync=nohud', { raf: false });
  const g2 = env2.load();
  assert.match(env2.out[0], /paint clock task end \(no rAF\); frame clock auto /);
  // without rAF the present is assumed right after each task that drew to the canvas:
  // task 1: 3 syncs + a trailing draw (lo 4 / hi 5); task 2: one draw (lo 1 / hi 2)
  g2.bindFramebuffer(K.FRAMEBUFFER, null);
  for (let i = 0; i < 3; i++) { g2.drawArrays(K.TRIANGLES, 0, 3); g2.getParameter(K.READ_BUFFER); }
  g2.drawArrays(K.TRIANGLES, 0, 3);
  await tick();
  g2.drawArrays(K.TRIANGLES, 0, 3);
  await tick();
  env2.log(PROF(2));
  assert.match(lineAfter(env2, PROF(2)), / \| presents 2\/2f flush~2\.5\/3\.5 /);
});

test('raf clock: a page with no PROF lines reports every 60 paints; f = the tasks that drew to the canvas', async () => {
  // The cvcGL gallery page: Module.print goes to a DOM node, no PROF line ever reaches console.log.
  const env = makeEnv('?glsync=nohud');
  const gl = env.load();
  for (let n = 1; n <= 130; n++) { heavyOverlayFrame(gl, false); await tick(); env.paint(); }
  // a paint's present is counted when the next task starts, so the 60th closes the first window
  const lines = env.out.filter((l) => l.startsWith('GLSYNC '));
  assert.equal(lines.length, 2, lines.join('\n'));
  assert.match(lines[0], /^GLSYNC n=60 f=60 warming clock=raf \| gl /);
  assert.match(lines[1], /^GLSYNC n=120 f=60 clock=raf \| gl /);
  assert.equal(field(lines[1], / sync ([\d.]+)x /)[1], '13.0'); // the mock frame's 13 sync calls, per frame
  assert.match(lines[1], / \| presents 60\/60f flush~/);
  assertSums(env.ctx.__glsync.lastWindow(), 'raf window');
  assert.equal(env.ctx.__glsync.clock(), 'raf');
});

test('raf clock: every=N, FBO-only tasks are not frames, clock=raf ignores PROF lines', async () => {
  const env = makeEnv('?glsync=nohud,clock=raf,every=5');
  const gl = env.load();
  assert.match(env.out[0], /; frame clock every 5 paints$/);
  for (let n = 1; n <= 12; n++) {
    heavyOverlayFrame(gl, false); await tick();
    // a second task per frame that draws only into a framebuffer object: not a frame
    gl.bindFramebuffer(K.FRAMEBUFFER, 7); gl.drawArrays(K.TRIANGLES, 0, 3); await tick();
    env.paint();
    if (n === 3) env.log(PROF(3));
  }
  const lines = env.out.filter((l) => l.startsWith('GLSYNC '));
  assert.equal(lines.length, 2, lines.join('\n'));
  assert.match(lines[0], /^GLSYNC n=5 f=5 warming clock=raf \| /);
  assert.match(lines[1], /^GLSYNC n=10 f=5 clock=raf \| /);
  assert.ok(!env.out.some((l) => /^GLSYNC n=3 /.test(l)), 'reported on a PROF line under clock=raf');
  assert.equal(env.ctx.__glsync.clock(), 'raf');
});

test('clock=auto moves to the PROF lines once the app prints one; clock=prof waits for them', async () => {
  const env = makeEnv('?glsync=nohud,every=5');
  const gl = env.load();
  for (let n = 1; n <= 7; n++) { heavyOverlayFrame(gl, false); await tick(); env.paint(); }
  assert.ok(env.out.some((l) => /^GLSYNC n=5 f=5 warming clock=raf \| /.test(l)), env.out.join('\n'));
  assert.equal(env.ctx.__glsync.clock(), 'raf');
  env.log(PROF(7)); // the app prints PROF lines: from now on they clock the census
  assert.equal(env.ctx.__glsync.clock(), 'prof');
  assert.match(lineAfter(env, PROF(7)), /^GLSYNC n=7 f=7 warming \| gl /);
  await runFrames(env, gl, 8, 20, { bakeEvery: 1000 });
  const after = env.out.slice(env.out.indexOf(PROF(7)));
  assert.ok(!after.some((l) => /clock=raf/.test(l)), 'a raf report after the switch');
  assert.match(lineAfter(env, PROF(10)), /^GLSYNC n=10 f=3 \| gl /);
  assert.match(lineAfter(env, PROF(20)), /^GLSYNC n=20 f=10 \| gl /);
  // clock=prof: no PROF line, no report, however many paints
  const env2 = makeEnv('?glsync=nohud,clock=prof,every=2');
  const g2 = env2.load();
  assert.match(env2.out[0], /; frame clock PROF lines$/);
  for (let n = 1; n <= 10; n++) { heavyOverlayFrame(g2, false); await tick(); env2.paint(); }
  assert.deepEqual(env2.out.filter((l) => l.startsWith('GLSYNC ')), []);
  assert.equal(env2.ctx.__glsync.clock(), 'prof');
});

// ---------------------------------------------------------------------------------------------
async function overhead() {
  // Per-call cost of the wrappers vs the raw (no-op) fake method, 1e6 calls per row, best of 3.
  // Measured by bench_glsync.js in a child process, in ONE realm as in the browser (calls across
  // a vm context boundary cost ~200 ns more each and would overstate it).
  const { execFileSync } = require('child_process');
  const res = {};
  for (const mode of ['timed', 'lite'])
    res[mode] = JSON.parse(execFileSync(process.execPath, [path.join(__dirname, 'bench_glsync.js'), mode], { encoding: 'utf8' }));
  return res;
}

(async () => {
  let fail = 0;
  for (const [name, fn] of cases) {
    try { await fn(); console.log('ok   - ' + name); }
    catch (e) { fail++; console.log('FAIL - ' + name + '\n       ' + String(e && e.stack || e).split('\n').slice(0, 6).join('\n       ')); }
  }
  const o = await overhead();
  console.log('overhead ns/call (Node ' + process.version + ', 1e6 calls, best of 3; raw = the unwrapped no-op):');
  for (const [mode, r] of Object.entries(o)) {
    console.log('  ' + mode.padEnd(5) + ' queued bindTexture ' + r.raw_bindTexture + ' -> ' + r.wrapped_bindTexture_A +
                ' | keyed getParameter(C) ' + r.raw_getParameter + ' -> ' + r.wrapped_getParameter_C +
                ' | draw ' + r.raw_draw + ' -> ' + r.wrapped_draw_A +
                ' | in-page calibration (ovh~) timed ' + r.cal_timed + ' queued ' + r.cal_queued + ' keyed ' + r.cal_keyed +
                ' draw ' + r.cal_draw);
  }
  const t = o.timed;
  const add = (w, r) => w - r;
  console.log('  -> timed: +' + add(t.wrapped_bindTexture_A, t.raw_bindTexture) + ' ns queued, +' +
              add(t.wrapped_getParameter_C, t.raw_getParameter) + ' ns keyed, +' + add(t.wrapped_draw_A, t.raw_draw) +
              ' ns draw. Rough per-frame cost at an illustrative 4000 calls/frame (not a measurement of any' +
              ' app; the live GLSYNC line has gl/f and ovh~): ~' +
              (add(t.wrapped_bindTexture_A, t.raw_bindTexture) * 4000 / 1e6).toFixed(2) + ' ms');
  try {
    for (const m of ['timed', 'lite']) for (const k of ['wrapped_bindTexture_A', 'wrapped_getParameter_C', 'wrapped_draw_A'])
      assert.ok(o[m][k] < 5000, m + ' ' + k + ' implausibly high: ' + o[m][k]);
    assert.ok(o.lite.wrapped_bindTexture_A < o.timed.wrapped_bindTexture_A, 'lite queued call not cheaper than timed');
    for (const k of ['cal_timed', 'cal_queued', 'cal_keyed', 'cal_draw']) assert.ok(o.timed[k] > 0 && o.timed[k] < 5000, k);
    console.log('ok   - overhead measured');
  } catch (e) { fail++; console.log('FAIL - overhead: ' + e.message); }
  console.log(fail ? fail + ' FAILED' : 'all ' + (cases.length + 1) + ' passed');
  process.exitCode = fail ? 1 : 0;
})();
