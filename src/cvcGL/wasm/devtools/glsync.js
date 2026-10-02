// glsync.js -- Firefox synchronous-WebGL census for a cvcGL WebAssembly app. Active only with
// ?glsync. A developer tool: not part of the cvcGL SDK, see GLSYNC.md next to this file.
//
// Wraps every WebGL2RenderingContext.prototype method. The dev server (src/cvcGL/examples/wasm/
// serve.py --glsync) inserts this script BEFORE the page's first <script>, so it is the innermost
// wrapper: any other WebGL wrapper on the page, and the module's webgl_state_shadow.js
// (cvcgl_wasm_app's state shim), wrap on top of it, and a query the shadow answers never reaches
// this census -- what is counted is what reaches Firefox. Each call is timed with performance.now()
// (20 us clamp in a cross-origin-isolated page, unbiased in aggregate) and aggregated per method,
// and per method + pname for the parameter getters. Once per report window (see "Frame clock"
// below) it prints
//   GLSYNC n=.. f=.. | gl N/f ms | sync Nx ms (drain~ rt~) | presents P/Ff flush~lo/hi max lo/hi
//          >10 lo%/hi% (lo N / hi M) [c chunk + s sync + a auto + 1 present; hi +r race] shmem=B
//          | ovh~ ms | top: key[cls] Nx ms max | ...
//   GLSYNC seq n=..   (?glsync=seq) the worst JS task's synchronous calls in order (a task is a
//                     frame only on builds that yield once per frame, see "Frames and tasks");
//                     '*' = commands were queued since the previous sync, so the call waited for
//                     the GPU process to run them (the drain)
// gl/sync counts and ms are PER FRAME (window totals / PROF frame delta); the flush figures are PER
// PRESENT; max is the slowest single call. "(A untimed)" after the gl ms: ?glsync=lite, whose
// plain queued calls are counted but not timed. Approximate: lite still times the queued slots of
// texture uploads (texImage*/texSubImage*), of the S1 / two-way methods (useProgram,
// uniformBlockBinding, readPixels into a PACK buffer) and getExtension(enable).
//
// cls = what Firefox 156.0.1 does with the call (dom/canvas/ClientWebGLContext.cpp, tag
// FIREFOX_156_0_1_RELEASE): S  synchronous IPC to the GPU process (FlushPendingCmds + Send*, which
// waits for every queued command first); S1 synchronous only on the first use after linkProgram /
// compileShader (GetLinkResult / GetCompileResult): that call is listed as [S], later ones as
// [C] (queries) or [A] (useProgram, uniformBlockBinding); C  answered in the content process;
// A  queued into the command shmem; F  ships the shmem without waiting. "!n" marks a non-S key
// with n calls >= 1 ms (an unexpected stall: shader-link wait, SyncPing congestion, GC).
// Extension-dependent classes follow the extensions enabled on that context: getExtension's first
// enable of a name queues RequestExtension (getExtension(enable) [A]); getParameter(MAX_VIEWS_OVR /
// PROVOKING_VERTEX_WEBGL / DEPTH_CLAMP) is C only once OVR_multiview2 / WEBGL_provoking_vertex /
// EXT_depth_clamp is enabled, S before; UNMASKED_*_WEBGL is S with WEBGL_debug_renderer_info
// enabled, C (a client-side error) without. getFramebufferAttachmentParameter(OBJECT_NAME) is C
// with an FBO bound, S on the default framebuffer. drawingBufferWidth/Height comes from a cache
// that a canvas size change (canvas.width/height: SetDimensions) clears; the next read refills it
// (that read is S), and so does the next paint: mResetLayer starts true and SetDimensions sets it
// again, and the paint's UpdateWebRenderCanvasData then calls DrawingBufferSize (:656-660). So a
// read is S only after a size change with neither a read nor a present since, C otherwise.
//
// flush~ models WebGLChild's flush counter (flushesSinceLastCongestionCheck), which async present
// checks: GetFrontBuffer does a SYNCHRONOUS present when more than 10 flushes happened since the
// previous present (ClientWebGLContext.cpp:585-591). Firefox presents only in the refresh driver's
// paint (OnBeforePaintTransaction -> Present -> GetFrontBuffer, :475, :538-609), and an Asyncify
// app does not pace its frames to paints (emscripten_sleep(0) is a setTimeout), so the census uses
// requestAnimationFrame as the paint clock: a draw to the canvas requests one rAF, and that
// refresh tick's present is counted once the tick is over (at the next WebGL call or PROF line).
// Flushes carry over across frames until a paint, so a skipped or late paint shows as
// presents < frames and a bigger flush~; a paint between the tasks of one frame (see "Frames and
// tasks") can present a frame twice, so presents > frames is possible too. Counted per present,
// by cause:
//   chunk    a shmem overflow (shmem=, webgl.out-of-process.shmem-size; a single command bigger
//            than the shmem gets a buffer of its own: no flush when nothing is queued before it)
//   sync     a synchronous call with commands queued (incl. a SyncPing past 70 flushes)
//   auto     the deferred auto-flush a draw/clear/blit/fenceSync schedules (AfterDrawCall ->
//            AutoEnqueueFlush, webgl.auto-flush=true) when it certainly ran before the paint:
//            another task with WebGL calls started before any paint
//   F        gl.flush() (always one: it queues a Flush command, then flushes) and a DOM-source
//            texImage with commands queued
//   present  GetFrontBuffer's own flush (always 1)
// lo = chunk + sync + auto + F + 1 is the EXPECTED count: the deferred auto-flush of the last frame
// before a paint is a normal-priority runnable and the refresh tick is vsync-prioritised, so the
// paint normally runs first and Present cancels it. hi = lo + race is the BOUND: race = 1 when
// that auto-flush was still pending (with commands queued) at the tick, so it may have run first.
// ">10 lo%/hi% (lo N / hi M)" = share and count of presents over the budget, for lo and hi: those
// presents were synchronous and waited for the GPU process. "ping20 n": n times the counter
// reached 20, when Firefox sends an async congestion Ping whose reply resets the counter (not
// modeled, so the >10 verdict of a present with >= 20 flushes is uncertain); "syncping n":
// SyncPing past 70 flushes (synchronous, resets the counter; modeled).
//
// Frame clock (?glsync=clock=auto|prof|raf, default auto): when a report window ends and what a
// "frame" is.
//   prof  a report after each "PROF n=<frame>" console line an app prints (its own frame-timing
//         line; anything may follow the number); f = the frame delta since the previous one. An app
//         that pauses prints "PROF paused n=<frame>" and "PROF resumed n=<frame>" around the pause:
//         a report made while paused is tagged "paused", and resumed restarts the window at <frame>,
//         so what was drawn while paused is not divided into the next report's frames.
//   raf   every `every=N` paints (default 60) of a drawn canvas; f = the JS tasks in the window that
//         drew to the default framebuffer (the canvas). With FrameYield::App (one yield per frame)
//         a task is exactly one frame; see "Frames and tasks". For pages whose output does not
//         reach console.log -- the cvcGL gallery page routes Module.print to a DOM node.
//   auto  raf until the first PROF line, prof from then on.
// The census adds ?prof to the URL only on request: ?glsync=prof, or window.__glsyncProfParam =
// '<name>' set before it runs (serve.py --prof-param <name> injects that), which adds ?<name>
// instead. A URL that already has prof, profhud or glcount (other switches with which an app may
// already print PROF lines), or <name>, is left alone.
//
// Known gaps (not modeled; none is per-frame in the cvcGL demos): client-side validation errors
// (EnqueueError) queue a GenerateError command and schedule an auto-flush; a DOM-source upload
// that misses the GPU-copy fast path (BlitPreventReason) uploads inline (queued bytes) instead of
// F (an ImageData source is modeled as inline); a canvas resize queues Resize and dirties the
// canvas; getFramebufferAttachmentParameter(DEPTH_STENCIL_ATTACHMENT) on an FBO syncs twice
// (counted once); byte sizes are estimates (~32 B per command + payload); one WebGL context; a
// task's WebGL calls are one burst, closed at its first microtask checkpoint, so a task that
// spreads them over several microtasks counts its own pending auto-flush as run (a cvcGL app's
// tasks make all their calls in one synchronous run); WebGL calls made in a rAF callback after the
// census's own in the same tick are counted towards the next present.
// The model is Firefox 156's: in another browser the classes are wrong and every line says
// "model is Firefox-only".
//
// Frames and tasks: the census sees JS tasks and paints, not frames. A frame is one task only in
// an app that yields once per frame: cvcGL's FrameYield::App (SceneRenderer::setFrameYield, which
// every cvcGL gallery demo uses) switches off the emscripten_sleep(0) that VTK 9.5's
// vtkWebAssemblyOpenGLRenderWindow::Frame() makes inside every render. With FrameYield::Vtk
// (VTK's default behaviour) a frame spans two or more tasks: the seq line then shows the worst
// task, not the worst frame, the raf clock's f counts a frame once per task that drew, and a paint
// can land between the tasks of a frame, so one frame can be presented twice. The flush model
// follows the real task and paint boundaries either way.
//
// The Node tests (tests/test_glsync.js) drive a MOCK heavy-overlay frame with hand-picked call
// volumes. Its sync and flush figures are a harness self-test only, not evidence about any real
// app; only the live ?glsync output in Firefox measures one.
//
// URL: ?glsync[=opts], opts comma-separated:
//   1 / on / true  just on (the defaults)
//   thr=<ms>       list keys with >= ms/frame (default 0.1; S keys are always listed). A bare
//                  number other than 1 also works (0.5, 2, 1.0)
//   shmem=<bytes>  command shmem size to model (default 100000 = Firefox's default
//                  webgl.out-of-process.shmem-size). Pass what about:config says; Firefox reads the
//                  pref when the context is created, so reload the page after changing it
//   clock=auto|prof|raf  the frame clock (above; default auto); every=<N> paints per raf window
//                  (default 60)
//   prof           add ?prof to the URL (unless prof, profhud or glcount is already there) so an
//                  app that prints PROF lines with it does
//   all (list every key), seq (the worst task's sync order line), stacks (JS/wasm stack of each
//   key's first call >= 2 ms), lite (count but do not time plain queued A calls: lower overhead,
//   no stall flag on them; texture uploads and the S1 / two-way slots stay timed, see above),
//   nohud (console only: no short line appended to a #profhud element the page may have);
//   0 / off / false: disabled.
// Console: __glsync.table() per-key totals since load; __glsync.last(); __glsync.lastWindow() raw
// flush figures of the last line; __glsync.model() the flush model's state; __glsync.opts.
(() => {
  'use strict';
  if (typeof window === 'undefined' || typeof WebGL2RenderingContext === 'undefined') return;
  let qs;
  try { qs = new URLSearchParams(window.location.search); } catch (e) { return; }
  if (!qs.has('glsync')) return; // the whole cost when off: this check
  const raw = qs.get('glsync') || '';
  if (raw === '0' || raw === 'off' || raw === 'false' || window.__glsync) return;

  const opts = { thr: 0.1, all: false, seq: false, stacks: false, lite: false, hud: true, shmem: 100000,
                 stallMs: 1, stackMs: 2, top: 10, clock: 'auto', every: 60, profParam: null };
  const ignored = [];
  for (const t of raw.split(',')) {
    let m;
    if (t === '' || t === '1' || t === 'on' || t === 'true') continue; // enable only
    if ((m = /^(?:thr=)?(\d*\.?\d+)$/.exec(t))) opts.thr = +m[1];
    else if ((m = /^shmem=(\d+)$/.exec(t))) opts.shmem = +m[1];
    else if ((m = /^clock=(auto|prof|raf)$/.exec(t))) opts.clock = m[1];
    else if ((m = /^every=([1-9]\d*)$/.exec(t))) opts.every = +m[1];
    else if (t === 'prof') opts.profParam = 'prof';
    else if (t === 'all') opts.all = true;
    else if (t === 'stacks') opts.stacks = true;
    else if (t === 'nohud') opts.hud = false;
    else if (t === 'lite') opts.lite = true;
    else if (t === 'seq') opts.seq = true;
    else ignored.push(t);
  }
  // A server-side request (serve.py --prof-param <name>) for the same URL rewrite.
  const injected = window.__glsyncProfParam;
  if (opts.profParam === null && typeof injected === 'string' && /^[A-Za-z0-9_-]+$/.test(injected))
    opts.profParam = injected;
  const ua = (typeof navigator !== 'undefined' && navigator && navigator.userAgent) || '';
  const notFx = /\bFirefox\//.test(ua) ? '' : ' (model is Firefox-only)';
  // Only on request: add ?prof (or the injected name) before the page script reads the URL, for an
  // app that prints PROF lines with it (the prof clock reports on them). A URL that already has
  // prof, profhud or glcount (other switches with which an app may already print PROF lines) is
  // left alone.
  if (opts.profParam !== null && !qs.has('prof') && !qs.has('profhud') && !qs.has('glcount') &&
      !qs.has(opts.profParam)) {
    try {
      const u = new URL(window.location.href);
      u.searchParams.set(opts.profParam, '');
      window.history.replaceState(window.history.state, '', u.toString());
    } catch (e) { /* keep going: the census then reports on its raf clock */ }
  }

  const G = WebGL2RenderingContext, P = G.prototype, perf = performance;
  const S = 1, S1 = 2, C = 3, A = 4, F = 5;
  const CLS = ['', 'S', 'S1', 'C', 'A', 'F'];
  const SHMEM = opts.shmem; // webgl.out-of-process.shmem-size (read at context creation)
  const CMD = 32;           // ~bytes per serialized command (aligned header + args)
  const FLUSH_BUDGET = 10;  // GetFrontBuffer: sync present when flushesSinceLastCongestionCheck > 10
  const raf = typeof window.requestAnimationFrame === 'function' ? window.requestAnimationFrame.bind(window) : null;

  // ---- flush model state ----
  // FL persists across tasks: queued commands stay queued, and flushes count, until a paint.
  // drawFb/readFb: the bound framebuffers (null = the canvas). chunk..F: flushes since the last
  // present by cause; since: the congestion counter GetFrontBuffer checks. gl: the context whose
  // draw dirtied the canvas (the one the next present paints).
  const FL = { pend: 0, since: 0, aSince: 0, autoFl: false, dirty: false, drawFb: null, readFb: null,
               chunk: 0, sync: 0, auto: 0, F: 0, gl: null };
  // T: the refresh tick our rAF saw; closed (and its present counted) at the next WebGL call or
  // PROF line. auto/pend: the auto-flush was pending, with commands queued, when the tick started.
  const T = { open: false, auto: false, pend: 0 };
  let rafReq = false;
  const extOn = new WeakMap(); // context -> Set of extension names (lower case) it enabled
  const extEnabled = (gl, n) => { const s = extOn.get(gl); return s !== undefined && s.has(n); };
  // context -> the canvas size its cached drawing-buffer size is for (filled by a read or a present)
  const dbSize = new WeakMap();
  const canvasSize = (gl) => {
    const cv = gl && gl.canvas;
    return cv && typeof cv === 'object' ? Math.max(1, cv.width | 0) + 'x' + Math.max(1, cv.height | 0) : null;
  };

  // ---- enum names ----
  const NAMES = new Map();
  for (const k of Object.getOwnPropertyNames(G)) {
    const v = G[k];
    if (typeof v === 'number' && /^[A-Z][A-Z0-9_]*$/.test(k) && !NAMES.has(v)) NAMES.set(v, k);
  }
  [[0x8009, 'BLEND_EQUATION_RGB'], [0x8CA6, 'DRAW_FRAMEBUFFER_BINDING'],
   [0x84FF, 'MAX_TEXTURE_MAX_ANISOTROPY_EXT'], [0x88BF, 'TIME_ELAPSED_EXT'], [0x8E28, 'TIMESTAMP_EXT'],
   [0x8FBB, 'GPU_DISJOINT_EXT'], [0x9245, 'UNMASKED_VENDOR_WEBGL'], [0x9246, 'UNMASKED_RENDERER_WEBGL'],
   [0x9631, 'MAX_VIEWS_OVR'], [0x8E4F, 'PROVOKING_VERTEX_WEBGL'], [0x864F, 'DEPTH_CLAMP_EXT'],
  ].forEach(([v, k]) => NAMES.set(v, k));
  const enumName = (v) => NAMES.get(v) || ('0x' + v.toString(16).toUpperCase());

  // ---- Firefox 156 client-side answers (ClientWebGLContext::GetParameter :2193-2566) ----
  const byName = (list) => new Set(list.map((n) => G[n]).filter((v) => v !== undefined));
  const CLIENT_PNAMES = byName([
    'ARRAY_BUFFER_BINDING', 'CURRENT_PROGRAM', 'ELEMENT_ARRAY_BUFFER_BINDING', 'FRAMEBUFFER_BINDING',
    'RENDERBUFFER_BINDING', 'TEXTURE_BINDING_2D', 'TEXTURE_BINDING_CUBE_MAP', 'VERTEX_ARRAY_BINDING',
    'MAX_COMBINED_TEXTURE_IMAGE_UNITS', 'MAX_TEXTURE_SIZE', 'MAX_CUBE_MAP_TEXTURE_SIZE',
    'MAX_VERTEX_ATTRIBS', 'PACK_ALIGNMENT', 'UNPACK_ALIGNMENT', 'UNPACK_FLIP_Y_WEBGL',
    'UNPACK_PREMULTIPLY_ALPHA_WEBGL', 'UNPACK_COLORSPACE_CONVERSION_WEBGL', 'DEPTH_RANGE',
    'ALIASED_POINT_SIZE_RANGE', 'ALIASED_LINE_WIDTH_RANGE', 'COLOR_CLEAR_VALUE', 'BLEND_COLOR',
    'MAX_VIEWPORT_DIMS', 'SCISSOR_BOX', 'VIEWPORT', 'COMPRESSED_TEXTURE_FORMATS',
    'COPY_READ_BUFFER_BINDING', 'COPY_WRITE_BUFFER_BINDING', 'DRAW_FRAMEBUFFER_BINDING',
    'MAX_CLIENT_WAIT_TIMEOUT_WEBGL', 'PIXEL_PACK_BUFFER_BINDING', 'PIXEL_UNPACK_BUFFER_BINDING',
    'READ_FRAMEBUFFER_BINDING', 'SAMPLER_BINDING', 'TEXTURE_BINDING_2D_ARRAY', 'TEXTURE_BINDING_3D',
    'TRANSFORM_FEEDBACK_BINDING', 'TRANSFORM_FEEDBACK_BUFFER_BINDING', 'UNIFORM_BUFFER_BINDING',
    'MAX_TRANSFORM_FEEDBACK_SEPARATE_ATTRIBS', 'MAX_UNIFORM_BUFFER_BINDINGS',
    'UNIFORM_BUFFER_OFFSET_ALIGNMENT', 'MAX_3D_TEXTURE_SIZE', 'MAX_ARRAY_TEXTURE_LAYERS',
    'PACK_ROW_LENGTH', 'PACK_SKIP_PIXELS', 'PACK_SKIP_ROWS', 'UNPACK_IMAGE_HEIGHT',
    'UNPACK_ROW_LENGTH', 'UNPACK_SKIP_IMAGES', 'UNPACK_SKIP_PIXELS', 'UNPACK_SKIP_ROWS',
    'VENDOR', 'VERSION', 'SHADING_LANGUAGE_VERSION']);
  // answered client-side only once the extension is enabled (:2248-2282); else the sync GetNumber
  const EXT_PNAME = new Map([[0x9631, 'ovr_multiview2'], [0x8E4F, 'webgl_provoking_vertex'],
                             [0x864F, 'ext_depth_clamp']]);
  const UNMASKED = new Set([0x9245, 0x9246]); // without the extension: a client-side error (C)
  const getParameterCls = (p) => {
    const e = EXT_PNAME.get(p);
    if (e !== undefined) return (a, gl) => (extEnabled(gl, e) ? C : S);
    if (UNMASKED.has(p)) return (a, gl) => (extEnabled(gl, 'webgl_debug_renderer_info') ? S : C);
    return CLIENT_PNAMES.has(p) ? C : S;
  };
  // OBJECT_NAME is answered from the client's attachment record when an FBO is bound for that
  // target (:2718-2770); the default framebuffer, other pnames and DEPTH_STENCIL_ATTACHMENT sync.
  const OBJ_NAME = G.FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, DS_ATT = G.DEPTH_STENCIL_ATTACHMENT, RFB = G.READ_FRAMEBUFFER;
  const fbAttachCls = (p) => (p === OBJ_NAME
    ? (a) => (((a[0] === RFB ? FL.readFb : FL.drawFb) !== null && a[1] !== DS_ATT) ? C : S) : S);
  const cls1 = (set) => (p) => (set.has(p) ? C : S);
  const PROG_C = byName(['DELETE_STATUS', 'VALIDATE_STATUS', 'ATTACHED_SHADERS']);
  // method -> [pname arg index, pname -> class | (args, gl) -> class]; -S1 = once after linkProgram
  const KEYED = {
    getParameter: [0, getParameterCls],
    getIndexedParameter: [0, cls1(byName(['TRANSFORM_FEEDBACK_BUFFER_BINDING', 'UNIFORM_BUFFER_BINDING']))],
    getVertexAttrib: [1, cls1(byName(['CURRENT_VERTEX_ATTRIB', 'VERTEX_ATTRIB_ARRAY_BUFFER_BINDING']))],
    getFramebufferAttachmentParameter: [2, fbAttachCls],
    getProgramParameter: [1, (p) => (PROG_C.has(p) ? C : -S1)],
    getShaderParameter: [1, (p) => (p === G.COMPILE_STATUS ? S1 : C)],
    getTexParameter: [1, () => S], getBufferParameter: [1, () => S], getRenderbufferParameter: [1, () => S],
    getSamplerParameter: [1, () => S], getQueryParameter: [1, () => S], getInternalformatParameter: [2, () => S],
    getSyncParameter: [1, () => C], isEnabled: [0, () => C],
    getActiveUniformBlockParameter: [2, () => -S1], getActiveUniforms: [2, () => -S1],
  };
  const set = (s) => new Set(s.split(' '));
  const S_M = set('getError finish checkFramebufferStatus getBufferSubData getUniform getFragDataLocation ' +
                  'getVertexAttribOffset validateProgram');
  const C_M = set('isBuffer isFramebuffer isProgram isQuery isRenderbuffer isSampler isShader isSync ' +
                  'isTexture isTransformFeedback isVertexArray isContextLost getContextAttributes ' +
                  'getSupportedExtensions getShaderPrecisionFormat getShaderSource getAttachedShaders getQuery');
  // First use after linkProgram waits for the link (arg 0 is the program). Queries are C after
  // that; the two commands are queued (A) -- on the first use too, right after the wait.
  const LINK_Q = set('getProgramInfoLog getUniformLocation getAttribLocation getActiveAttrib getActiveUniform ' +
                     'getActiveUniformBlockName getUniformBlockIndex getUniformIndices getTransformFeedbackVarying');
  const LINK_CMD = set('useProgram uniformBlockBinding');
  const COMPILE_Q = set('getShaderInfoLog getTranslatedShaderSource');
  // AfterDrawCall(): marks the canvas dirty when drawing to the default framebuffer and schedules
  // the deferred auto-flush. drawArrays/drawElements/drawRangeElements forward to the instanced ones.
  const DRAWS = set('drawArrays drawElements drawArraysInstanced drawElementsInstanced drawRangeElements ' +
                    'clear clearBufferfv clearBufferiv clearBufferuiv clearBufferfi blitFramebuffer');
  const pendLink = new WeakSet(), pendCompile = new WeakSet();

  // ---- payload bytes (flush estimate only) ----
  const CH = new Map([[G.RGBA, 4], [G.RGB, 3], [G.RG, 2], [G.RED, 1], [G.RGBA_INTEGER, 4], [G.RGB_INTEGER, 3],
    [G.RG_INTEGER, 2], [G.RED_INTEGER, 1], [G.LUMINANCE_ALPHA, 2], [G.LUMINANCE, 1], [G.ALPHA, 1],
    [G.DEPTH_COMPONENT, 1], [G.DEPTH_STENCIL, 1]]);
  const TB = new Map([[G.UNSIGNED_BYTE, 1], [G.BYTE, 1], [G.UNSIGNED_SHORT, 2], [G.SHORT, 2], [G.HALF_FLOAT, 2],
    [G.UNSIGNED_INT, 4], [G.INT, 4], [G.FLOAT, 4]]);
  const PACKED = new Map([[G.UNSIGNED_SHORT_5_6_5, 2], [G.UNSIGNED_SHORT_4_4_4_4, 2], [G.UNSIGNED_SHORT_5_5_5_1, 2],
    [G.UNSIGNED_INT_24_8, 4], [G.UNSIGNED_INT_2_10_10_10_REV, 4], [G.UNSIGNED_INT_10F_11F_11F_REV, 4],
    [G.UNSIGNED_INT_5_9_9_9_REV, 4], [G.FLOAT_32_UNSIGNED_INT_24_8_REV, 8]]);
  const bpp = (fmt, type) => PACKED.get(type) || (CH.get(fmt) || 4) * (TB.get(type) || 1);
  const isView = ArrayBuffer.isView;
  // view slice: (view, srcOffset elems, srcLength elems) -> bytes; 0 / absent length = the rest
  const vbytes = (v, off, len) => (!v || typeof v !== 'object') ? 0 :
    (isView(v) ? (len ? len : (v.length || v.byteLength) - (off || 0)) * (v.BYTES_PER_ELEMENT || 1) : (v.length || 0) * 4);
  // texture methods: [w, h, d|-1, format, type, source] arg indexes of the long (sized) overloads;
  // the WebGL1-style short overloads (no width/height/border) take the source last
  const TEX = { texImage2D: [3, 4, -1, 6, 7, 8], texSubImage2D: [4, 5, -1, 6, 7, 8],
                texImage3D: [3, 4, 5, 7, 8, 9], texSubImage3D: [5, 6, 7, 8, 9, 10] };
  const firstView = (a, from) => { for (let i = from; i < a.length; i++) if (isView(a[i]) || Array.isArray(a[i])) return i; return -1; };
  const IMAGEDATA = typeof ImageData === 'function' ? ImageData : null;

  // ---- slots ----
  // ok = which calibrated wrapper cost applies (ovh~): st timed static, a queued static (untimed
  // under lite), key keyed getter, draw. q = the call also queues its own command: F before the
  // flush (gl.flush), S after the wait (the S1 twin of useProgram / uniformBlockBinding).
  const slots = [];
  const blank = (key, short, cls, ok) => ({ key, short, cls, ok, q: false, dyn: null, n: 0, ms: 0, max: 0,
    stalls: 0, stack: null, tn: 0, tms: 0, tmax: 0, twin: null, link: undefined });
  const mkSlot = (key, short, cls, ok) => { const s = blank(key, short, cls, ok); slots.push(s); return s; };
  const mkS1 = (key, short, link, base, ok) => { // steady-state slot (base class) + its one-off sync twin
    const s = mkSlot(key, short, base, ok);
    s.twin = mkSlot(key, short, S, ok); s.twin.q = base === A; s.link = link;
    return s;
  };
  const pick = (s, obj) => {
    const ws = s.link ? pendLink : pendCompile;
    if (obj && typeof obj === 'object' && ws.has(obj)) { ws.delete(obj); return s.twin; }
    return s;
  };

  // ---- flush model + window state ----
  const newBurst = () => ({ open: false, sn: 0, sms: 0, seq: [], drew: false });
  const newWin = () => ({ p: 0, lo: 0, race: 0, maxLo: 0, maxHi: 0, overLo: 0, overHi: 0,
                          chunk: 0, sync: 0, auto: 0, F: 0, ping20: 0, syncPing: 0,
                          dn: 0, dms: 0, rt: new Float64Array(64), rtK: 0, worst: null });
  let B = newBurst(), W = newWin(), lastW = null;
  // The frame clock (see the header): clockNow is the one reporting now (auto starts on raf and
  // moves to prof at the first PROF line). rafFrames: tasks since load that drew to the canvas
  // (the raf clock's frame count); rafPaints: paints of a drawn canvas in the current raf window.
  let clockNow = opts.clock === 'prof' ? 'prof' : 'raf', rafFrames = 0, rafPaints = 0;

  function countFlush(cause) { // WebGLChild::FlushPendingCmds with a buffer to send
    FL[cause]++;
    if (++FL.since === 20) W.ping20++;                             // async Ping (reply may reset: not modeled)
    else if (FL.since > 70) { W.syncPing++; FL.since = 0; FL.aSince = 0; } // SyncPing: sync, resets
  }
  function enqueue(bytes) {
    const need = bytes + CMD;
    // AllocPendingCmdBytes: flush when the command does not fit; a command bigger than the shmem
    // gets a buffer of its own (no flush when nothing is pending before it).
    if (FL.pend > 0 && FL.pend + need > SHMEM) { countFlush('chunk'); FL.pend = need; }
    else FL.pend += need;
    FL.aSince++;
  }
  function flushPending(cause) { if (FL.pend > 0) { countFlush(cause); FL.pend = 0; } }

  // The paint: Present() queues a command and GetFrontBuffer flushes it (+1), then resets the
  // counter; Present also cancels a pending auto-flush. After a canvas size change (and on the
  // first paint: mResetLayer starts true) the paint's UpdateWebRenderCanvasData fetches the
  // drawing-buffer size, so after any present the cache holds the size the canvas has now.
  function present(race) {
    const k = canvasSize(FL.gl);
    if (k !== null) dbSize.set(FL.gl, k);
    const lo = FL.chunk + FL.sync + FL.auto + FL.F + 1, hi = lo + race;
    const cLo = FL.since + 1, cHi = cLo + race; // the counter GetFrontBuffer checks
    W.p++; W.lo += lo; W.race += race;
    W.chunk += FL.chunk; W.sync += FL.sync; W.auto += FL.auto; W.F += FL.F;
    if (lo > W.maxLo) W.maxLo = lo;
    if (hi > W.maxHi) W.maxHi = hi;
    if (cLo > FLUSH_BUDGET) { W.overLo++; FL.aSince = 0; } // a sync present already waited for the backlog
    if (cHi > FLUSH_BUDGET) W.overHi++;
    FL.chunk = 0; FL.sync = 0; FL.auto = 0; FL.F = 0;
    FL.since = 0; FL.pend = 0; FL.dirty = false; FL.autoFl = false; FL.gl = null;
  }
  // rAF = the paint clock: a refresh tick runs rAF callbacks, then paints (a skipped tick skips both).
  function onTick() {
    rafReq = false;
    if (T.open) closeTick();
    T.open = true; T.auto = FL.autoFl; T.pend = FL.pend;
  }
  function closeTick() { // the tick (and its paint) is over
    T.open = false;
    if (!FL.dirty) return;
    present(T.auto && T.pend > 0 ? 1 : 0);
    if (clockNow === 'raf' && ++rafPaints >= opts.every) { rafPaints = 0; report(rafFrames); }
  }
  // A burst = the WebGL calls of one JS task: one frame on builds that yield once per frame (the
  // loop's emscripten_sleep(0)), part of one where VTK's Frame() yields too (DoubleBuffer on).
  function openBurst() {
    if (T.open) closeTick();
    // an auto-flush still pending from an earlier task, with no paint since to cancel it, has run
    if (FL.autoFl) { flushPending('auto'); FL.autoFl = false; }
    B.open = true;
    queueMicrotask(closeBurst);
  }
  function closeBurst() {
    if (!B.open) return;
    if (B.drew) rafFrames++; // a task that drew to the canvas: one frame on the raf clock
    if (raf === null && FL.dirty && !T.open) onTick(); // no rAF here: assume a paint right after the task
    if (B.sn > 0 && (W.worst === null || B.sms > W.worst.sms)) W.worst = B;
    B = newBurst();
  }
  function rec(s, dt, bytes) {
    s.n++; s.ms += dt; if (dt > s.max) s.max = dt;
    if (!B.open) openBurst();
    const c = s.cls;
    if (c === A) enqueue(bytes);
    else if (c === S) {
      const drained = FL.aSince > 0;
      FL.aSince = 0;
      flushPending('sync');
      if (s.q) enqueue(bytes); // useProgram after the link wait: the UseProgram command is queued
      B.sn++; B.sms += dt;
      if (drained) { W.dn++; W.dms += dt; } else if (W.rtK < 64) W.rt[W.rtK++] = dt;
      if (B.seq.length < 96) B.seq.push(s.short + (drained ? '*' : ''), dt);
      if (opts.stacks && dt >= opts.stackMs && !s.stack) s.stack = stackHere();
      return;
    } else if (c === F) { if (s.q) enqueue(0); flushPending('F'); }
    if (dt >= opts.stallMs) { s.stalls++; if (opts.stacks && !s.stack) s.stack = stackHere(); }
  }
  function afterDraw(gl) {
    FL.autoFl = true;
    if (FL.drawFb === null) {
      FL.dirty = true; FL.gl = gl; B.drew = true;
      if (!rafReq && raf !== null) { rafReq = true; raf(onTick); }
    }
  }

  // Caller frames of the wrapped call, innermost first: Firefox "name@url:l:c" (wasm frames are
  // "name@...wasm:wasm-function[N]:0x.." -- named when the wasm has a name section), V8
  // "at name (url)". Printed: the JS frame that called into wasm-side GL last (e.g. the emscripten
  // glue _glGetIntegerv) and the first 4 wasm frames; JS-only stacks: the first 5 frames.
  function stackHere() {
    const fr = [];
    let firstWasm = -1;
    for (const l of String(new Error().stack || '').split('\n')) {
      if (!l || l === 'Error' || /(?:^|[\s(@/\\])glsync\.js[:?]/.test(l)) continue;
      const v8 = /^\s*at (?:async )?(\S+)/.exec(l), fn = /wasm-function\[(\d+)\]/.exec(l);
      let nm = v8 ? v8[1] : l.split('@')[0];
      if (!nm && fn) nm = 'wasm-function[' + fn[1] + ']';
      if (fn && firstWasm < 0) firstWasm = fr.length;
      fr.push((nm || '?').replace(/\(.*$/, '').slice(0, 60));
      if (fr.length >= 40) break;
    }
    const pickd = firstWasm > 0 ? fr.slice(firstWasm - 1, firstWasm + 4) : fr.slice(0, 5);
    return pickd.join(' < ');
  }

  // ---- wrappers ----
  const timedA = !opts.lite; // ?glsync=lite: queued calls counted, not timed
  function wrapStatic(name, orig, cls) {
    const s = mkSlot(name, name, cls, cls === A ? 'a' : 'st');
    if (cls === A && !timedA) return function () { const r = orig.apply(this, arguments); rec(s, 0, 0); return r; };
    return function () { const t = perf.now(); const r = orig.apply(this, arguments); rec(s, perf.now() - t, 0); return r; };
  }
  function wrapBytes(name, orig, bytesOf) {
    const s = mkSlot(name, name, A, 'a');
    if (!timedA) return function () { const r = orig.apply(this, arguments); rec(s, 0, bytesOf(arguments)); return r; };
    return function () { const t = perf.now(); const r = orig.apply(this, arguments); rec(s, perf.now() - t, bytesOf(arguments)); return r; };
  }
  function wrapDraw(name, orig, bytes, owner) { // owner: the context of an extension's draw
    const s = mkSlot(name, name, A, 'draw');
    if (!timedA) return function () { const r = orig.apply(this, arguments); rec(s, 0, bytes); afterDraw(owner || this); return r; };
    return function () { const t = perf.now(); const r = orig.apply(this, arguments); rec(s, perf.now() - t, bytes); afterDraw(owner || this); return r; };
  }
  function wrapS1(name, orig, link, base) { // arg 0 is the program (link) / shader (compile)
    const s = mkS1(name, name, link, base, 'st');
    return function () {
      const sl = pick(s, arguments[0]);
      const t = perf.now(); const r = orig.apply(this, arguments); rec(sl, perf.now() - t, 0); return r;
    };
  }
  function wrapKeyed(name, orig, ai, clsOf) {
    const m = new Map();
    const slotFor = (p) => {
      const nm = enumName(p), key = name + '(' + nm + ')', short = name === 'getParameter' ? nm : key;
      const mk = (c) => (c === S1 ? mkS1(key, short, false, C, 'key') : c === -S1 ? mkS1(key, short, true, C, 'key')
                                  : mkSlot(key, short, c, 'key'));
      const c = clsOf(p);
      let s;
      if (typeof c === 'function') { // class decided per call (extension enabled, framebuffer bound)
        const by = [];
        s = blank(key, short, 0, 'key');
        s.dyn = (a, gl) => { const k = c(a, gl); return by[k] || (by[k] = mk(k)); };
      } else s = mk(c);
      m.set(p, s);
      return s;
    };
    return function () {
      const p = arguments[ai] >>> 0;
      let s = m.get(p);
      if (s === undefined) s = slotFor(p);
      if (s.dyn !== null) s = s.dyn(arguments, this);
      if (s.link !== undefined) s = pick(s, arguments[0]);
      const t = perf.now(); const r = orig.apply(this, arguments); rec(s, perf.now() - t, 0); return r;
    };
  }
  function wrapTwo(name, orig, which, clsA, clsB, tagB) { // per-call class choice
    const sa = mkSlot(name, name, clsA, 'st'), sb = mkSlot(name + tagB, name + tagB, clsB, 'st');
    return function () {
      const s = which(arguments) ? sb : sa;
      const t = perf.now(); const r = orig.apply(this, arguments); rec(s, perf.now() - t, 0); return r;
    };
  }
  function wrapTex(name, orig) {
    const [wi, hi, di, fi, ti, si] = TEX[name];
    const sv = mkSlot(name, name, A, 'st');                                    // view (inline) / PBO offset
    const sid = mkSlot(name + '(imagedata)', name + '(imagedata)', A, 'st');   // CPU pixels: inline
    const sdom = mkSlot(name + '(dom)', name + '(dom)', F, 'st');              // SurfaceDescriptor: flush + async SendTexImage
    return function () {
      const a = arguments, src = a.length > si ? a[si] : a[a.length - 1];
      let s = sv, bytes = 0;
      if (src !== null && typeof src === 'object' && !isView(src)) {
        if (IMAGEDATA !== null && src instanceof IMAGEDATA) { s = sid; bytes = (src.width * src.height * 4) || 0; }
        else s = sdom;
      } else if (a.length > si && isView(a[si])) bytes = (a[wi] * a[hi] * (di < 0 ? 1 : a[di]) * bpp(a[fi], a[ti])) || 0;
      const t = perf.now(); const r = orig.apply(this, a); rec(s, perf.now() - t, bytes); return r;
    };
  }
  const wrappedExt = new WeakSet();
  function wrapExtension(ext, extName, gl) { // per instance, so the shared extension prototypes stay untouched
    if (!ext || typeof ext !== 'object' || wrappedExt.has(ext)) return;
    wrappedExt.add(ext);
    const proto = Object.getPrototypeOf(ext);
    if (!proto) return;
    for (const k of Object.getOwnPropertyNames(proto)) {
      const d = Object.getOwnPropertyDescriptor(proto, k);
      if (k === 'constructor' || !d || typeof d.value !== 'function') continue;
      const key = extName + '.' + k;
      // WEBGL_multi_draw / *_base_vertex_base_instance draws go through AfterDrawCall too
      const w = /^(multiDraw|draw)/.test(k) ? wrapDraw(key, d.value, 0, gl) : wrapStatic(key, d.value, A);
      try { Object.defineProperty(ext, k, { value: w, configurable: true, writable: true }); }
      catch (e) { /* non-extensible: leave unwrapped */ }
    }
  }

  function makeWrapper(name, orig) {
    if (KEYED[name]) return wrapKeyed(name, orig, KEYED[name][0], KEYED[name][1]);
    if (TEX[name]) return wrapTex(name, orig);
    if (DRAWS.has(name)) return wrapDraw(name, orig, /^clearBuffer/.test(name) ? 16 : 0);
    switch (name) {
      case 'linkProgram': { const w = wrapStatic(name, orig, A); return function (p) { if (p) pendLink.add(p); return w.apply(this, arguments); }; }
      case 'compileShader': { const w = wrapStatic(name, orig, A); return function (sh) { if (sh) pendCompile.add(sh); return w.apply(this, arguments); }; }
      case 'bindFramebuffer': { // the DRAW binding decides whether a draw dirties the canvas
        const w = wrapStatic(name, orig, A), FB = G.FRAMEBUFFER, DFB = G.DRAW_FRAMEBUFFER;
        return function (target, fb) {
          const r = w.apply(this, arguments), o = fb || null; // emscripten binds 0 as undefined
          if (target === FB) { FL.drawFb = o; FL.readFb = o; } else if (target === DFB) FL.drawFb = o; else if (target === RFB) FL.readFb = o;
          return r;
        };
      }
      case 'deleteFramebuffer': { // deleting a bound framebuffer rebinds the default one
        const w = wrapStatic(name, orig, A);
        return function (fb) {
          const r = w.apply(this, arguments);
          if (fb) { if (FL.drawFb === fb) FL.drawFb = null; if (FL.readFb === fb) FL.readFb = null; }
          return r;
        };
      }
      case 'fenceSync': { const w = wrapStatic(name, orig, A); return function () { const r = w.apply(this, arguments); FL.autoFl = true; return r; }; }
      case 'readPixels': // into an ArrayBufferView: SendReadPixels (sync); with a PACK buffer offset: queued
        return wrapTwo(name, orig, (a) => typeof a[6] !== 'number', A, S, '(cpu)');
      case 'clientWaitSync': // timeout 0 is answered client-side (TIMEOUT_EXPIRED / ALREADY_SIGNALED)
        return wrapTwo(name, orig, (a) => +a[2] > 0, C, S, '(wait)');
      case 'getExtension': { // the first enable of a name queues RequestExtension; later calls are C
        const sc = mkSlot(name, name, C, 'st'), sa = mkSlot(name + '(enable)', name + '(enable)', A, 'st');
        return function (n) {
          const t = perf.now(); const r = orig.apply(this, arguments); const dt = perf.now() - t;
          let s = sc;
          if (r) {
            let on = extOn.get(this);
            if (on === undefined) { on = new Set(); if (this && typeof this === 'object') extOn.set(this, on); }
            const k = String(n).toLowerCase();
            if (!on.has(k)) { on.add(k); s = sa; }
          }
          rec(s, dt, 0); wrapExtension(r, String(n), this); return r;
        };
      }
      case 'flush': { const w = wrapStatic(name, orig, F); slots[slots.length - 1].q = true; return w; } // Flush(flushGl=true)
      case 'bufferData': return wrapBytes(name, orig, (a) => (isView(a[1]) ? vbytes(a[1], a[3], a[4]) : 0));
      case 'bufferSubData': return wrapBytes(name, orig, (a) => vbytes(a[2], a[3], a[4]));
    }
    if (S_M.has(name)) return wrapStatic(name, orig, S);
    if (C_M.has(name)) return wrapStatic(name, orig, C);
    if (LINK_Q.has(name)) return wrapS1(name, orig, true, C);
    if (LINK_CMD.has(name)) return wrapS1(name, orig, true, A);
    if (COMPILE_Q.has(name)) return wrapS1(name, orig, false, C);
    if (/^(uniform|vertexAttrib)/.test(name) && /v$/.test(name))
      return wrapBytes(name, orig, (a) => { const i = firstView(a, 1); return i < 0 ? 0 : vbytes(a[i], a[i + 1], a[i + 2]); });
    if (/^compressedTex/.test(name))
      return wrapBytes(name, orig, (a) => { const i = firstView(a, 1); return i < 0 ? 0 : vbytes(a[i], a[i + 1], a[i + 2]); });
    return wrapStatic(name, orig, A);
  }

  for (const name of Object.getOwnPropertyNames(P)) {
    if (name === 'constructor') continue;
    const d = Object.getOwnPropertyDescriptor(P, name);
    if (!d || !d.configurable) continue;
    if (d.get && (name === 'drawingBufferWidth' || name === 'drawingBufferHeight')) {
      // DrawingBufferSize(): a sync when the cache is empty -- after a canvas size change
      // (SetDimensions clears it) with no read and no present since (see present()) -- else C
      const sS = mkSlot(name, name, S, 'st'), sC = mkSlot(name, name, C, 'st'), g = d.get;
      Object.defineProperty(P, name, {
        get() {
          if (!B.open) openBurst(); // count a present that just ended first: it refilled the cache
          let s = sC;
          const k = canvasSize(this);
          if (k !== null && dbSize.get(this) !== k) { dbSize.set(this, k); s = sS; }
          const t = perf.now(); const r = g.call(this); rec(s, perf.now() - t, 0); return r;
        },
        configurable: true, enumerable: d.enumerable });
      continue;
    }
    if (typeof d.value !== 'function') continue;
    P[name] = makeWrapper(name, d.value);
  }

  // ---- calibration: what each wrapper kind adds to a call (2 x performance.now() + the
  // bookkeeping): wrapped JS no-ops minus the bare no-op through the same loop, after a warm-up,
  // best of 5 rounds x 5000 calls (the kinds interleaved, so a GC or JIT pause hits one round of
  // one kind), with the flush model saved and restored around it. ~40 ms once, at load. ovh~ =
  // calls x this per-call cost: an APPROXIMATION, neither a floor nor a bound (the loop's one f.call
  // site sees all five functions, and a no-op is not the page's call mix). In Node it came out
  // about equal to the measured wrapper cost for timed calls and well above it for lite's untimed
  // ones (~20 ns calibrated vs ~2 ns measured).
  const cal = { st: 0, a: 0, key: 0, draw: 0 };
  {
    const saveFL = Object.assign({}, FL), saveT = Object.assign({}, T), saveB = B, saveW = W;
    const saveReq = rafReq, nSlots = slots.length;
    B = newBurst(); W = newWin();
    FL.drawFb = {}; // a calibration draw must not dirty the canvas or request a rAF
    const nop = function () {};
    const w = { base: nop, st: wrapStatic('cal', nop, C), a: wrapStatic('cal', nop, A),
                key: wrapKeyed('cal', nop, 0, () => C), draw: wrapDraw('cal', nop, 0) };
    const kinds = Object.keys(w), t = {};
    for (const k of kinds) {
      const f = w[k];
      for (let i = 0; i < 10000; i++) f.call(null, 0x0BA2, i); // warm-up: let the JIT optimize it
      t[k] = Infinity;
    }
    for (let r = 0; r < 5; r++) {
      for (const k of kinds) {
        const f = w[k], t0 = perf.now();
        for (let i = 0; i < 5000; i++) f.call(null, 0x0BA2, i);
        t[k] = Math.min(t[k], (perf.now() - t0) / 5000);
      }
    }
    for (const k of Object.keys(cal)) cal[k] = Math.max(0, t[k] - t.base);
    Object.assign(FL, saveFL); Object.assign(T, saveT); B = saveB; W = saveW; rafReq = saveReq;
    slots.length = nSlots;
  }
  const ovhMs = (s) => s.n * cal[s.ok];

  // ---- reporting ----
  const fx = (x) => (x >= 100 ? Math.round(x).toString() : x.toFixed(1));
  const fm = (x) => (x >= 10 ? x.toFixed(0) : x >= 1 ? x.toFixed(1) : x.toFixed(2));
  const kilo = (x) => (x >= 1000 ? (x / 1000).toFixed(1) + 'k' : Math.round(x).toString());
  const ns = (ms) => Math.round(ms * 1e6);
  function median(a, k) { if (!k) return null; const v = Array.from(a.subarray(0, k)).sort((x, y) => x - y); return v[k >> 1]; }
  let lastN = null, paused = false, lastLine = '';
  const origLog = console.log;
  const say = (s) => origLog.call(console, s);

  function resetWindow() {
    for (const s of slots) {
      s.tn += s.n; s.tms += s.ms; if (s.max > s.tmax) s.tmax = s.max;
      s.n = 0; s.ms = 0; s.max = 0; s.stalls = 0; s.stack = null;
    }
    W = newWin();
  }
  function flushPart(f) {
    const tail = ' shmem=' + SHMEM + (W.ping20 ? ' ping20 ' + W.ping20 : '') + (W.syncPing ? ' syncping ' + W.syncPing : '');
    if (!W.p) return 'presents 0/' + f + 'f (no paint of a drawn canvas)' + tail;
    const p = W.p, per = (x) => fx(x / p), pct = (x) => Math.round(100 * x / p) + '%';
    return 'presents ' + p + '/' + f + 'f flush~' + (W.lo / p).toFixed(1) + '/' + ((W.lo + W.race) / p).toFixed(1) +
      ' max ' + W.maxLo + '/' + W.maxHi + ' >' + FLUSH_BUDGET + ' ' + pct(W.overLo) + '/' + pct(W.overHi) +
      ' (lo ' + W.overLo + ' / hi ' + W.overHi + ') [' + per(W.chunk) + ' chunk + ' + per(W.sync) + ' sync + ' +
      per(W.auto) + ' auto' + (W.F ? ' + ' + per(W.F) + ' F' : '') + ' + 1 present; hi +' + per(W.race) + ' race]' + tail;
  }
  function report(n) {
    closeBurst();
    if (T.open) closeTick();
    const warming = lastN === null, f = warming ? n : n - lastN;
    lastN = n;
    if (!(f > 0)) { resetWindow(); return; }
    let glN = 0, glMs = 0, sN = 0, sMs = 0, ovh = 0;
    const rows = [];
    for (const s of slots) {
      if (!s.n) continue;
      glN += s.n; glMs += s.ms; ovh += ovhMs(s);
      if (s.cls === S) { sN += s.n; sMs += s.ms; }
      if (opts.all || s.cls === S || s.stalls > 0 || s.ms / f >= opts.thr) rows.push(s);
    }
    const head = 'GLSYNC n=' + n + ' f=' + f + (warming ? ' warming' : '') + (paused ? ' paused' : '') +
      (clockNow === 'raf' ? ' clock=raf' : '') + notFx;
    if (!glN) {
      lastLine = head + ' | no WebGL2 calls on this thread';
      say(lastLine); lastW = W; resetWindow(); return;
    }
    rows.sort((a, b) => b.ms - a.ms);
    // the top N by ms, then every S key that did not make it (a sync call is never hidden)
    const shown = opts.all ? rows : rows.slice(0, opts.top).concat(rows.slice(opts.top).filter((s) => s.cls === S));
    const rt = median(W.rt, W.rtK), drain = Math.max(0, W.dms - W.dn * (rt || 0));
    const top = shown.map((s) =>
      s.key + '[' + CLS[s.cls] + (s.cls !== S && s.stalls ? '!' + s.stalls : '') + '] ' + fx(s.n / f) + 'x ' +
      fm(s.ms / f) + 'ms max ' + fm(s.max));
    const more = rows.length - top.length;
    const line = head +
      ' | gl ' + kilo(glN / f) + '/f ' + fm(glMs / f) + 'ms' + (opts.lite ? ' (A untimed)' : '') +
      ' | sync ' + fx(sN / f) + 'x ' + fm(sMs / f) + 'ms (drain~' + fm(drain / f) + ' rt~' +
      (rt === null ? '-' : Math.round(rt * 1000) + 'us') + ')' +
      ' | ' + flushPart(f) +
      ' | ovh~' + fm(ovh / f) + 'ms' +
      ' | top: ' + (top.length ? top.join(' | ') : '-') + (more > 0 ? ' | +' + more + ' more' : '');
    say(line);
    lastLine = line;
    if (opts.seq && W.worst) {
      const q = W.worst.seq, parts = [];
      for (let i = 0; i < q.length; i += 2) parts.push(q[i] + ' ' + fm(q[i + 1]));
      say('GLSYNC seq n=' + n + ' worst task: ' + W.worst.sn + ' sync ' + fm(W.worst.sms) + 'ms: ' + parts.join(' > '));
    }
    if (opts.stacks) for (const s of rows) if (s.stack) say('GLSYNC stack ' + s.key + ': ' + s.stack);
    if (opts.hud) {
      const hud = document.getElementById('profhud');
      // A page that shows its PROF line in #profhud sets that text right after its console.log; a
      // microtask runs after that (when the app's task ends), so the line is appended to the fresh
      // PROF text, not overwritten.
      if (hud) {
        const short = 'GLSYNC sync ' + fx(sN / f) + 'x ' + fm(sMs / f) + 'ms flush~' +
          (W.p ? (W.lo / W.p).toFixed(1) + '/' + ((W.lo + W.race) / W.p).toFixed(1) + ' >10 ' +
                 Math.round(100 * W.overLo / W.p) + '%' : '-') +
          ' top ' + top.slice(0, 3).map((t) => t.replace(/ max .*$/, '')).join(', ');
        queueMicrotask(() => { hud.textContent += '\n' + short; });
      }
    }
    lastW = W;
    resetWindow();
  }
  console.log = function () {
    const r = origLog.apply(this, arguments);
    const s = arguments[0];
    if (opts.clock !== 'raf' && typeof s === 'string' && s.charCodeAt(0) === 80 && s.startsWith('PROF ')) {
      let m = /^PROF n=(\d+)/.exec(s);
      if (m && clockNow === 'raf') { // clock=auto: the app prints PROF lines, so report on them from now
        clockNow = 'prof'; lastN = null; rafPaints = 0;
      }
      if (m) report(+m[1]);
      else if ((m = /^PROF (paused|resumed) n=(\d+)/.exec(s))) {
        closeBurst();
        if (T.open) closeTick();
        if (m[1] === 'paused') paused = true;
        else { paused = false; resetWindow(); lastN = +m[2]; }
      }
    }
    return r;
  };

  const winCopy = (w) => (w ? { presents: w.p, lo: w.lo, hi: w.lo + w.race, race: w.race, chunk: w.chunk,
    sync: w.sync, auto: w.auto, F: w.F, overLo: w.overLo, overHi: w.overHi, maxLo: w.maxLo, maxHi: w.maxHi,
    ping20: w.ping20, syncPing: w.syncPing } : null);
  window.__glsync = {
    opts,
    clock: () => clockNow,
    cal: Object.assign({}, cal),
    last: () => lastLine,
    lastWindow: () => winCopy(lastW),
    model: () => ({ pend: FL.pend, since: FL.since, chunk: FL.chunk, sync: FL.sync, auto: FL.auto, F: FL.F,
                    dirty: FL.dirty, autoFlush: FL.autoFl, tickOpen: T.open, extensions: (gl) => Array.from(extOn.get(gl) || []) }),
    table: () => slots.filter((s) => s.tn + s.n).map((s) => ({ key: s.key, cls: CLS[s.cls], n: s.tn + s.n,
      ms: +(s.tms + s.ms).toFixed(2), max: +Math.max(s.tmax, s.max).toFixed(2) })).sort((a, b) => b.ms - a.ms),
  };
  say('glsync: census on' + notFx + ' -- Firefox 156 sync model; shmem=' + SHMEM +
      ' (must match about:config webgl.out-of-process.shmem-size, read at context creation: reload after changing it)' +
      '; list >= ' + opts.thr + ' ms/frame' + (opts.seq ? ', seq' : '') + (opts.stacks ? ', stacks' : '') +
      (opts.lite ? ', lite' : '') + (opts.all ? ', all' : '') + (ignored.length ? '; ignored: ' + ignored.join(',') : '') +
      '; wrapper ovh~ timed ' + ns(cal.st) + ' / queued ' + ns(cal.a) + ' / keyed ' + ns(cal.key) +
      ' / draw ' + ns(cal.draw) + ' ns; paint clock ' + (raf !== null ? 'requestAnimationFrame' : 'task end (no rAF)') +
      '; frame clock ' + (opts.clock === 'prof' ? 'PROF lines'
                          : opts.clock === 'raf' ? 'every ' + opts.every + ' paints'
                          : 'auto (PROF lines, else every ' + opts.every + ' paints)') +
      (opts.profParam !== null ? '; adds ?' + opts.profParam : ''));
})();
