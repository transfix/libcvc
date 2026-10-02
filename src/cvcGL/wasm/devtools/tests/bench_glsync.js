#!/usr/bin/env node
// Per-call overhead of the glsync.js wrappers, measured in the SAME realm as the fake WebGL2
// prototype (as in the browser; a vm context adds ~200 ns of cross-realm cost per call).
//   node tests/bench_glsync.js [timed|lite]   -> one JSON line of ns/call (best of 3 x 1e6 calls)
// Rows: a queued call (bindTexture, A), a keyed getter answered client-side (getParameter, C) and
// a draw (drawArraysInstanced: the flush model + AfterDrawCall bookkeeping), each raw (the
// unwrapped no-op) and wrapped; plus glsync's own in-page calibration per wrapper kind (cal_*),
// which the GLSYNC line's ovh~ uses (an approximation; compare it with the wrapped - raw rows).
'use strict';
const fs = require('fs'), path = require('path');
const mode = process.argv[2] === 'lite' ? 'lite' : 'timed';
class WebGL2RenderingContext {}
Object.assign(WebGL2RenderingContext, { COMPILE_STATUS: 0x8B81, FRAMEBUFFER: 0x8D40, DRAW_FRAMEBUFFER: 0x8CA9,
  READ_FRAMEBUFFER: 0x8CA8, CURRENT_PROGRAM: 0x8B8D, TEXTURE_2D: 0x0DE1, READ_BUFFER: 0x0C02 });
const P = WebGL2RenderingContext.prototype;
P.bindTexture = function (a, b) { return a; };
P.getParameter = function (p) { return p; };
P.drawArraysInstanced = function (m, f, c, n) { return n; };
const raw = { bindTexture: P.bindTexture, getParameter: P.getParameter, drawArraysInstanced: P.drawArraysInstanced };
const search = mode === 'lite' ? '?glsync=lite,nohud' : '?glsync=nohud';
const origLog = console.log;
let startup = '';
// no requestAnimationFrame here: glsync falls back to a paint per task end (no effect on the cost)
Object.assign(globalThis, { WebGL2RenderingContext, window: globalThis,
  location: { search: search + '&prof', href: 'http://localhost/' + search + '&prof' },
  history: { replaceState() {} }, document: { getElementById: () => null } });
console.log = (s) => { startup = String(s); };
(0, eval)(fs.readFileSync(path.join(__dirname, '..', 'glsync.js'), 'utf8')); // glsync hooks console.log
const gl = new WebGL2RenderingContext();
const N = 1e6;
const best = (fn) => {
  let b = Infinity;
  for (let k = 0; k < 3; k++) { const t = performance.now(); fn(); b = Math.min(b, performance.now() - t); }
  return Math.round(b * 1e6 / N);
};
const TEX = 0x0DE1, CUR = 0x8B8D;
const cal = /wrapper ovh~ timed (\d+) \/ queued (\d+) \/ keyed (\d+) \/ draw (\d+) ns/.exec(startup) || [];
const r = {
  mode,
  raw_bindTexture: best(() => { for (let i = 0; i < N; i++) raw.bindTexture.call(gl, TEX, null); }),
  wrapped_bindTexture_A: best(() => { for (let i = 0; i < N; i++) gl.bindTexture(TEX, null); }),
  raw_getParameter: best(() => { for (let i = 0; i < N; i++) raw.getParameter.call(gl, CUR); }),
  wrapped_getParameter_C: best(() => { for (let i = 0; i < N; i++) gl.getParameter(CUR); }),
  raw_draw: best(() => { for (let i = 0; i < N; i++) raw.drawArraysInstanced.call(gl, 4, 0, 3, 1); }),
  wrapped_draw_A: best(() => { for (let i = 0; i < N; i++) gl.drawArraysInstanced(4, 0, 3, 1); }),
  cal_timed: +cal[1], cal_queued: +cal[2], cal_keyed: +cal[3], cal_draw: +cal[4],
};
origLog.call(console, JSON.stringify(r));
