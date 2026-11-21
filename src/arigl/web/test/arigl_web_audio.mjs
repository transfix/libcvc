// arigl_web_audio.mjs — a conference's audio in a real (headless) Chrome: one participant sends a
// steady 440 Hz tone, another records what it plays, and this counts the gaps and the clicks in
// it, while the main thread is busy, and while either tab is hidden (docs/roadmap/
// ARIADNE_CONFERENCE_ROADMAP.md §5j, "Browser audio off the main thread").
//
//   node arigl_web_audio.mjs --web <share/arigl-web> --doc <conference .ari> --arisrv <arisrv>
//        --chrome <chrome> [--seconds 30] [--cases idle,busy,hidden-sender,hidden-receiver]
//        [--modes worklet,sdl] [--out DIR] [--expect] [--audible] [--gpu] [--csp]
//
// It runs a relay of its own (arisrv on 127.0.0.1, WebSocket for the pages) and serves the page
// with the document as main.ari and a page config that names the relay. Two Chrome processes, each
// with fake media: the sender's mic plays a steady tone (a WAV this writes), the receiver's
// silence, so the receiver sends nothing, and the sender's tab plays nothing (an audible tab is
// spared the browser's background throttling, a silent one is not). Before the page's own scripts
// run, a test script (injected with Page.addScriptToEvaluateOnNewDocument):
//
//   - turns the browser's voice processing off for the mic (echo cancellation, noise suppression
//     and gain control), so the tone arrives as sent; both modes get the same mic;
//   - records everything any AudioContext plays, with an AudioWorklet of its own on the
//     context's audio thread, so a busy main thread cannot disturb the recording itself;
//   - counts the main thread's long tasks.
//
// Each case is a fresh room. Once the receiver hears the sender, the case's condition starts:
//
//   idle             both tabs as they are (arigl's own frames, in software GL);
//   busy             both main threads blocked 150 ms in every 400 (a slow frame, a GC);
//   hidden-sender    the sender's tab behind another tab (no frames, timers about 1 Hz);
//   hidden-receiver  the receiver's tab behind another tab.
//
// Then it records --seconds and counts, in what the receiver played: GAPS, runs of 5 ms windows
// below a quarter of the tone's median level, and CLICKS, samples where the tone's own recurrence
// (x[n] = 2cos(w)x[n-1] - x[n-2]) misses by more than a fifth of its amplitude: a cut, a jump, a
// splice; and HEARD, the share of the time the tone is there at all. Modes: worklet (the AudioWorklet path, cvc/media/web_audio.h) and sdl (?audio=sdl, the
// page's CVC_WEB_AUDIO=sdl: SDL's ScriptProcessorNode, as before). --expect fails (exit 1) when
// the worklet mode, in any case, has the tone under 97% of the time; sdl is reported, not judged.
// (Measured 2026-11-20: the worklet mode 97.9-100% in every case, on the GPU and in SwiftShader;
// SDL 0% with the sender hidden and 13-19% with the receiver hidden.) --analyze A.wav ... analyzes recordings --out kept, and runs nothing. --out keeps each
// recording as a WAV. The browsers are muted (--mute-audio: the page's audio graph runs and is
// recorded, the machine's speakers stay quiet) unless --audible. WebGL is SwiftShader, where each
// of arigl's frames is a long task (about 100 ms), unless --gpu (the machine's GPU through ANGLE
// on OpenGL, as a desktop browser draws). --csp serves the pages under arigl_web_smoke.mjs's
// strict Content-Security-Policy (the relay allowed). Exit status 0 when every case ran (and, with --expect, passed).

import {spawn} from 'node:child_process';
import fs from 'node:fs';
import http from 'node:http';
import net from 'node:net';
import os from 'node:os';
import path from 'node:path';

const argv = process.argv.slice(2);
const opt = (name, dflt) => {
  const i = argv.indexOf(name);
  return i >= 0 ? argv[i + 1] : dflt;
};
const WEB = opt('--web'), DOC = opt('--doc'), ARISRV = opt('--arisrv'), CHROME = opt('--chrome');
const SECONDS = Number(opt('--seconds', '30'));
const CASES = opt('--cases', 'idle,busy,hidden-sender,hidden-receiver').split(',');
const MODES = opt('--modes', 'worklet,sdl').split(',');
const OUT = opt('--out');
const EXPECT = argv.includes('--expect');
const AUDIBLE = argv.includes('--audible'); // play through the machine's speakers (default: muted)
// The machine's GPU (ANGLE on OpenGL), as a desktop browser has; else SwiftShader, which runs
// anywhere but makes each of arigl's frames a long task.
const GPU = argv.includes('--gpu');
// arigl_web_smoke.mjs's strict Content-Security-Policy (no inline code, no eval, nothing from
// another origin but the relay), under which the worklets load only from the file beside the page.
const CSP = argv.includes('--csp');
const MIN_HEARD = 97; // percent, for --expect
if ((!WEB || !DOC || !ARISRV || !CHROME) && !argv.includes('--analyze')) {
  console.error('usage: arigl_web_audio.mjs --web DIR --doc FILE --arisrv PATH --chrome PATH ' +
                '[--seconds N] [--cases LIST] [--modes LIST] [--out DIR] [--expect]');
  process.exit(2);
}
const TONE_HZ = 440, RATE = 48000;
const sleep = ms => new Promise(ok => setTimeout(ok, ms));
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'arigl-web-audio-'));
const children = [];
const cleanup = () => {
  for (const c of children)
    try { c.kill('SIGKILL'); } catch (e) {}
};
process.on('exit', cleanup);
process.on('SIGINT', () => process.exit(130));

// ── the fake mics: 10 s of 16-bit mono, looped by Chrome (4400 whole periods, so the loop is
// seamless), and 10 s of silence ──
function wav(file, samples) {
  const data = Buffer.alloc(samples.length * 2);
  samples.forEach((v, i) => data.writeInt16LE(Math.max(-32767, Math.min(32767, Math.round(v * 32767))), i * 2));
  const h = Buffer.alloc(44);
  h.write('RIFF', 0); h.writeUInt32LE(36 + data.length, 4); h.write('WAVE', 8); h.write('fmt ', 12);
  h.writeUInt32LE(16, 16); h.writeUInt16LE(1, 20); h.writeUInt16LE(1, 22); h.writeUInt32LE(RATE, 24);
  h.writeUInt32LE(RATE * 2, 28); h.writeUInt16LE(2, 32); h.writeUInt16LE(16, 34); h.write('data', 36);
  h.writeUInt32LE(data.length, 40);
  fs.writeFileSync(file, Buffer.concat([h, data]));
}
const toneWav = path.join(tmp, 'tone.wav'), silenceWav = path.join(tmp, 'silence.wav');
wav(toneWav, Array.from({length: RATE * 10}, (_, i) => 0.3 * Math.sin(2 * Math.PI * TONE_HZ * i / RATE)));
wav(silenceWav, new Array(RATE * 10).fill(0));

// ── a free port ──
const freePort = () => new Promise(ok => {
  const s = net.createServer();
  s.listen(0, '127.0.0.1', () => { const p = s.address().port; s.close(() => ok(p)); });
});

// ── the relay ──
async function startRelay() {
  const [st, media, ws] = [await freePort(), await freePort(), await freePort()];
  const log = path.join(tmp, 'arisrv.log');
  const fd = fs.openSync(log, 'w');
  const srv = spawn(ARISRV, ['--room', 'audiotest', '--state-listen', `127.0.0.1:${st}`,
                             '--media-listen', `127.0.0.1:${media}`, '--ws-listen', `127.0.0.1:${ws}`],
                    {stdio: ['ignore', fd, fd]});
  children.push(srv);
  for (let i = 0; i < 100; i++) {
    if (fs.readFileSync(log, 'utf8').includes('relay up'))
      return {ws, log};
    if (srv.exitCode !== null)
      break;
    await sleep(200);
  }
  throw new Error('arisrv did not come up:\n' + fs.readFileSync(log, 'utf8'));
}

// ── the site: /app/ the page and the document, /test/ the tap ──
const TAP_WORKLET = `
class TestTap extends AudioWorkletProcessor {
  constructor() {
    super();
    this.buf = new Float32Array(sampleRate);
    this.n = 0;
    this.on = false;
    this.port.onmessage = e => {
      if (e.data === 'start') { this.on = true; this.n = 0; }
      else if (e.data === 'stop') { this.flush(); this.on = false; this.port.postMessage('done'); }
    };
  }
  flush() { if (this.n) this.port.postMessage(this.buf.slice(0, this.n)); this.n = 0; }
  process(inputs) {
    if (!this.on) return true;
    const x = inputs[0] && inputs[0].length ? inputs[0][0] : null;
    for (let i = 0; i < 128; i++) {
      this.buf[this.n++] = x ? x[i] : 0;
      if (this.n === this.buf.length) this.flush();
    }
    return true;
  }
}
registerProcessor('test-tap', TestTap);
`;
// Runs in every document before its own scripts.
const INJECT = `(() => {
  const md = navigator.mediaDevices;
  const plain = c => {
    if (!c || !c.audio) return c;
    const a = typeof c.audio === 'object' ? Object.assign({}, c.audio) : {};
    a.echoCancellation = false; a.noiseSuppression = false; a.autoGainControl = false;
    return Object.assign({}, c, {audio: a});
  };
  if (md && md.getUserMedia) {
    const gum = md.getUserMedia.bind(md);
    md.getUserMedia = c => gum(plain(c));
  }
  const T = window.__tap = {ctxs: [], long: {n: 0, ms: 0, max: 0}};
  try {
    new PerformanceObserver(l => {
      for (const e of l.getEntries()) { T.long.n++; T.long.ms += e.duration; T.long.max = Math.max(T.long.max, e.duration); }
    }).observe({type: 'longtask', buffered: true});
  } catch (e) {}
  const connect = AudioNode.prototype.connect;
  AudioNode.prototype.connect = function(dest, ...rest) {
    const r = connect.call(this, dest, ...rest);
    if (typeof AudioDestinationNode !== 'undefined' && dest instanceof AudioDestinationNode) {
      const ctx = dest.context;
      let t = T.ctxs.find(c => c.ctx === ctx);
      if (!t) {
        t = {ctx, node: null, pending: [], chunks: [], done: null};
        T.ctxs.push(t);
        ctx.audioWorklet.addModule('/test/tap-worklet.js').then(() => {
          t.node = new AudioWorkletNode(ctx, 'test-tap', {numberOfInputs: 1, numberOfOutputs: 1,
            outputChannelCount: [1], channelCount: 1, channelCountMode: 'explicit'});
          t.node.port.onmessage = e => {
            if (e.data === 'done') { if (t.done) t.done(); }
            else t.chunks.push(e.data);
          };
          connect.call(t.node, ctx.destination);
          for (const n of t.pending) connect.call(n, t.node);
          t.pending = [];
        });
      }
      if (t.node) connect.call(this, t.node); else t.pending.push(this);
    }
    return r;
  };
  T.start = () => { for (const t of T.ctxs) { t.chunks = []; if (t.node) t.node.port.postMessage('start'); } };
  // Stop and hand back each context's recording as 16-bit PCM in base64.
  T.take = async () => {
    await Promise.all(T.ctxs.filter(t => t.node).map(t => new Promise(ok => { t.done = ok; t.node.port.postMessage('stop'); })));
    return T.ctxs.map(t => {
      const n = t.chunks.reduce((s, c) => s + c.length, 0);
      const pcm = new Int16Array(n);
      let i = 0;
      for (const c of t.chunks) for (const v of c) pcm[i++] = Math.max(-32767, Math.min(32767, Math.round(v * 32767)));
      let bin = '';
      const u8 = new Uint8Array(pcm.buffer);
      for (let k = 0; k < u8.length; k += 0x8000) bin += String.fromCharCode.apply(null, u8.subarray(k, k + 0x8000));
      return {rate: t.ctx.sampleRate, b64: btoa(bin)};
    });
  };
  T.busy = (ms, every) => { T.busyId = setInterval(() => { const t = performance.now(); while (performance.now() - t < ms); }, every); };
  T.idle = () => clearInterval(T.busyId);
})();`;

function startSite(relayWs) {
  const files = {};
  for (const f of fs.readdirSync(WEB)) {
    const p = path.join(WEB, f);
    if (fs.statSync(p).isFile())
      files['/app/' + f] = p;
  }
  files['/app/'] = path.join(WEB, 'index.html');
  files['/app/main.ari'] = DOC;
  const config = JSON.stringify({
    title: 'audio test', src: 'main.ari',
    env: {CVC_ARIADNE_RELAY: `ws://127.0.0.1:${relayWs}`},
    url_env: {
      name: {env: 'CVC_ARIADNE_NAME', max: 32},
      room: {env: 'CVC_ARIADNE_ROOM', max: 40},
      audio: {env: 'CVC_WEB_AUDIO', match: '^(sdl|worklet)$'}
    }
  });
  const types = {'.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm',
                 '.css': 'text/css', '.json': 'application/json', '.ari': 'text/plain'};
  const srv = http.createServer((q, r) => {
    const u = q.url.split('?')[0];
    let body = null, type = 'text/plain';
    if (u === '/app/ariadne.config.json') { body = config; type = types['.json']; }
    else if (u === '/test/tap-worklet.js') { body = TAP_WORKLET; type = types['.js']; }
    else if (files[u]) { body = fs.readFileSync(files[u]); type = types[path.extname(files[u])] || type; }
    if (body === null) { r.writeHead(404); return r.end(); }
    const headers = {'Content-Type': type, 'Cache-Control': 'no-cache'};
    if (CSP)
      headers['Content-Security-Policy'] =
          "default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self'; " +
          `connect-src 'self' ws://127.0.0.1:${relayWs}; img-src 'self' data: blob:`;
    r.writeHead(200, headers);
    r.end(body);
  });
  return new Promise(ok => srv.listen(0, '127.0.0.1', () => ok({srv, port: srv.address().port})));
}

// ── Chrome over the DevTools protocol ──
async function launch(name, micWav) {
  const prof = fs.mkdtempSync(path.join(tmp, name + '-'));
  const ch = spawn(CHROME, [
    '--headless=new', '--remote-debugging-port=0', `--user-data-dir=${prof}`, '--no-first-run',
    '--no-default-browser-check', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream',
    `--use-file-for-fake-audio-capture=${micWav}`, '--autoplay-policy=no-user-gesture-required',
    ...(GPU ? ['--enable-gpu', '--ignore-gpu-blocklist', '--use-angle=gl']
            : ['--enable-unsafe-swiftshader', '--use-angle=swiftshader']),
    '--window-size=1280,800',
    ...(AUDIBLE ? [] : ['--mute-audio']), 'about:blank'
  ], {stdio: ['ignore', 'ignore', 'pipe']});
  children.push(ch);
  const url = await new Promise((ok, no) => {
    let b = '';
    ch.stderr.on('data', d => { b += d; const m = b.match(/ws:\/\/\S+/); if (m) ok(m[0]); });
    ch.on('exit', () => no(new Error(name + ': chrome exited:\n' + b)));
  });
  const ws = new WebSocket(url);
  await new Promise(ok => { ws.onopen = ok; });
  let id = 0;
  const pending = new Map();
  ws.onmessage = e => {
    const m = JSON.parse(e.data);
    if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); }
  };
  const send = (method, params = {}, sessionId) => new Promise(ok => {
    const i = ++id;
    pending.set(i, ok);
    ws.send(JSON.stringify({id: i, method, params, sessionId}));
  });
  const b = {name, send};
  b.open = async (url, inject) => {
    const {result: {targetId}} = await send('Target.createTarget', {url: 'about:blank'});
    const {result: {sessionId}} = await send('Target.attachToTarget', {targetId, flatten: true});
    const t = {targetId, sessionId};
    if (inject)
      await send('Page.addScriptToEvaluateOnNewDocument', {source: inject}, sessionId);
    await send('Page.enable', {}, sessionId);
    if (url)
      await send('Page.navigate', {url}, sessionId);
    t.eval = async expr => {
      const r = await send('Runtime.evaluate', {expression: expr, returnByValue: true, awaitPromise: true}, sessionId);
      if (r.result && r.result.exceptionDetails)
        throw new Error(name + ': ' + JSON.stringify(r.result.exceptionDetails).slice(0, 400));
      return r.result && r.result.result ? r.result.result.value : undefined;
    };
    t.front = () => send('Target.activateTarget', {targetId});
    t.close = () => send('Target.closeTarget', {targetId});
    return t;
  };
  return b;
}

// ── the analysis ──
// The tone's level is the 90th percentile of the 5 ms windows' RMS, so a recording that holds it
// only now and then still has one. HEARD is the share of windows at a quarter of that or more; a
// GAP is a run of windows below it; a CLICK is a sample where the tone's recurrence misses by more
// than a fifth of its amplitude (runs closer than a window are one click). No tone at all (a level
// under 0.02): heard 0.
function analyze(x, rate) {
  const W = Math.round(rate * 0.005);
  const rms = [];
  for (let i = 0; i + W <= x.length; i += W) {
    let s = 0;
    for (let k = 0; k < W; k++) s += x[i + k] * x[i + k];
    rms.push(Math.sqrt(s / W));
  }
  const sorted = rms.slice().sort((a, b) => a - b);
  const level = sorted[Math.floor(sorted.length * 0.9)] || 0;
  const seconds = +(x.length / rate).toFixed(1);
  if (level < 0.02)
    return {seconds, level: +level.toFixed(3), heard: 0, gaps: 0, gap_ms: 0, longest_gap_ms: 0, clicks: 0};
  let gaps = 0, gapW = 0, run = 0, longest = 0, on = 0;
  for (const v of rms.concat([Infinity])) {
    if (v < 0.25 * level) { run++; continue; }
    if (v !== Infinity) on++;
    if (run > 0) { gaps++; gapW += run; longest = Math.max(longest, run); }
    run = 0;
  }
  const c = 2 * Math.cos(2 * Math.PI * TONE_HZ / rate);
  const amp = level * Math.SQRT2;
  let clicks = 0, last = -Infinity;
  for (let n = 2; n < x.length; n++) {
    if (Math.abs(x[n] - c * x[n - 1] + x[n - 2]) > 0.2 * amp) {
      if (n - last > W) clicks++;
      last = n;
    }
  }
  return {seconds, level: +level.toFixed(3), heard: +(100 * on / rms.length).toFixed(1), gaps,
          gap_ms: Math.round(gapW * 5), longest_gap_ms: longest * 5, clicks};
}

function decode(b64) {
  const buf = Buffer.from(b64, 'base64');
  const x = new Float32Array(buf.length / 2);
  for (let i = 0; i < x.length; i++) x[i] = buf.readInt16LE(i * 2) / 32767;
  return x;
}

function writeWav(file, x, rate) {
  const data = Buffer.alloc(x.length * 2);
  for (let i = 0; i < x.length; i++) data.writeInt16LE(Math.round(Math.max(-1, Math.min(1, x[i])) * 32767), i * 2);
  const h = Buffer.alloc(44);
  h.write('RIFF', 0); h.writeUInt32LE(36 + data.length, 4); h.write('WAVE', 8); h.write('fmt ', 12);
  h.writeUInt32LE(16, 16); h.writeUInt16LE(1, 20); h.writeUInt16LE(1, 22); h.writeUInt32LE(rate, 24);
  h.writeUInt32LE(rate * 2, 28); h.writeUInt16LE(2, 32); h.writeUInt16LE(16, 34); h.write('data', 36);
  h.writeUInt32LE(data.length, 40);
  fs.writeFileSync(file, Buffer.concat([h, data]));
}

// ── one case ──
async function runCase(site, tx, rx, kase, mode, n) {
  const room = `audio-${n}-${kase}-${mode}`.slice(0, 40);
  const url = name => `http://127.0.0.1:${site.port}/app/?name=${name}&room=${room}&audio=${mode}`;
  const R = await rx.open(url('rx'), INJECT);
  await sleep(1500);
  const S = await tx.open(url('tx'), INJECT);
  const logOf = async t => (await t.eval('JSON.stringify((window.ariadneWeb && window.ariadneWeb.log) || [])').then(JSON.parse));
  // The receiver hears the sender: its voice output opened, and its tap records the tone.
  let heard = false;
  for (let i = 0; i < 120 && !heard; i++) {
    await sleep(500);
    const log = await logOf(R);
    heard = log.some(l => l.startsWith('arigl: audio out'));
    const st = await R.eval('window.ariadneWeb && window.ariadneWeb.state');
    if (st === 'failed')
      throw new Error(`${kase}/${mode}: the receiver's page failed:\n` + log.join('\n'));
  }
  if (!heard)
    throw new Error(`${kase}/${mode}: the receiver never heard the sender:\n` + (await logOf(R)).join('\n'));
  await sleep(3000); // its jitter buffer has started
  let cover = null;
  if (kase === 'busy') {
    await R.eval('__tap.busy(150, 400), 1');
    await S.eval('__tap.busy(150, 400), 1');
  } else if (kase === 'hidden-sender') {
    cover = await tx.open('about:blank');
    await cover.front();
  } else if (kase === 'hidden-receiver') {
    cover = await rx.open('about:blank');
    await cover.front();
  }
  // The sender's side: its audio context, and the capture node's blocks reaching its main thread.
  await S.eval(`(() => {
    const A = window.Module && Module.cvcAudio;
    const d = window.__caps = {n: 0};
    if (!A) return 0;
    for (const id in A.nodes)
      if (A.nodes[id].kind === 'capture' && A.nodes[id].node)
        A.nodes[id].node.port.addEventListener('message', () => d.n++);
    return 1;
  })()`);
  await sleep(2000);
  // When the playback node's ticks reach the main thread (worklet mode): the longest wait between
  // two, and how many waits were long.
  await R.eval(`(() => {
    const A = window.Module && Module.cvcAudio;
    const d = window.__ticks = {n: 0, max: 0, over100: 0, over250: 0, last: 0};
    if (!A) return 0;
    for (const id in A.nodes) {
      const rec = A.nodes[id];
      if (rec.kind !== 'playback') continue;
      rec.node.port.addEventListener('message', () => {
        const t = performance.now();
        if (d.last) { const g = t - d.last; d.max = Math.max(d.max, g); if (g > 100) d.over100++; if (g > 250) d.over250++; }
        d.last = t; d.n++;
      });
    }
    return 1;
  })()`);
  const longBefore = {R: await R.eval('JSON.stringify(__tap.long)'), S: await S.eval('JSON.stringify(__tap.long)')};
  await R.eval('__tap.start(), 1');
  await sleep(SECONDS * 1000);
  const recs = await R.eval('__tap.take()');
  const long = {R: await R.eval('JSON.stringify(__tap.long)'), S: await S.eval('JSON.stringify(__tap.long)')};
  const hidden = {R: await R.eval('document.visibilityState'), S: await S.eval('document.visibilityState')};
  const ticks = JSON.parse(await R.eval('JSON.stringify(window.__ticks || {})'));
  const sender = JSON.parse(await S.eval(`JSON.stringify({caps: (window.__caps || {}).n,
    ctx: window.Module && Module.cvcAudio && Module.cvcAudio.ctx ? Module.cvcAudio.ctx.state : 'none',
    why: window.Module && Module.cvcAudio ? Module.cvcAudio.why : 'none'})`));
  if (kase === 'busy') {
    await R.eval('__tap.idle(), 1');
    await S.eval('__tap.idle(), 1');
  }
  // The context that played the tone (SDL's and the worklets' are different contexts).
  let best = null;
  for (const r of recs || []) {
    const x = decode(r.b64);
    const a = analyze(x, r.rate);
    if (!best || a.level > best.a.level)
      best = {x, a, rate: r.rate};
  }
  const longDelta = k => {
    const a = JSON.parse(longBefore[k]), b = JSON.parse(long[k]);
    return {n: b.n - a.n, ms: Math.round(b.ms - a.ms), max: Math.round(b.max)};
  };
  const rlog = await logOf(R), slog = await logOf(S);
  const lines = log => log.filter(l => /^(arigl: (voice|audio out|conference|room)|web audio:)/.test(l)).slice(-8);
  if (OUT && best) {
    fs.mkdirSync(OUT, {recursive: true});
    writeWav(path.join(OUT, `${kase}-${mode}.wav`), best.x, best.rate);
  }
  if (cover) await cover.close();
  await S.close();
  await R.close();
  return {kase, mode, result: best ? best.a : null, hidden, ticks, sender, long_tasks: {rx: longDelta('R'), tx: longDelta('S')},
          rx_log: lines(rlog), tx_log: lines(slog)};
}

// --analyze A.wav [B.wav ...]: the analysis alone, on recordings --out kept.
if (argv.includes('--analyze')) {
  for (const f of argv.slice(argv.indexOf('--analyze') + 1)) {
    const b = fs.readFileSync(f);
    const x = new Float32Array((b.length - 44) / 2);
    for (let i = 0; i < x.length; i++) x[i] = b.readInt16LE(44 + 2 * i) / 32767;
    console.log(path.basename(f), JSON.stringify(analyze(x, b.readUInt32LE(24))));
  }
  process.exit(0);
}

const relay = await startRelay();
const site = await startSite(relay.ws);
const tx = await launch('tx', toneWav);
const rx = await launch('rx', silenceWav);
const results = [];
let n = 0, failed = false;
for (const kase of CASES)
  for (const mode of MODES) {
    n++;
    try {
      const r = await runCase(site, tx, rx, kase, mode, n);
      results.push(r);
      const a = r.result || {};
      console.log(`${kase.padEnd(16)} ${mode.padEnd(8)} heard ${String(a.heard).padStart(5)}%  gaps ${String(a.gaps).padStart(4)} ` +
                  `(${a.gap_ms} ms, longest ${a.longest_gap_ms} ms)  clicks ${String(a.clicks).padStart(4)}  ` +
                  `level ${a.level}  ${a.seconds} s  long tasks rx ${r.long_tasks.rx.n}/${r.long_tasks.rx.ms} ms ` +
                  `tx ${r.long_tasks.tx.n}/${r.long_tasks.tx.ms} ms  visibility rx ${r.hidden.R} tx ${r.hidden.S}`);
      console.log(`    rx main thread: long tasks max ${r.long_tasks.rx.max} ms; playback ticks ${r.ticks.n || 0}, ` +
                  `longest wait ${Math.round(r.ticks.max || 0)} ms, waits >100 ms ${r.ticks.over100 || 0}, >250 ms ${r.ticks.over250 || 0}`);
      console.log(`    tx: worklets ${r.sender.why}, context ${r.sender.ctx}, mic blocks on its main thread ${r.sender.caps}`);
      for (const l of r.rx_log) console.log('    rx | ' + l);
      for (const l of r.tx_log) console.log('    tx | ' + l);
      if (EXPECT && mode === 'worklet' && (!r.result || r.result.heard < MIN_HEARD)) {
        console.log(`    FAIL: ${kase}/worklet: the tone heard under ${MIN_HEARD}% of the time`);
        failed = true;
      }
    } catch (e) {
      console.log(`${kase.padEnd(16)} ${mode.padEnd(8)} ERROR ${e.message}`);
      failed = true;
    }
  }
if (OUT)
  fs.writeFileSync(path.join(OUT, 'results.json'), JSON.stringify(results, null, 2));
cleanup();
process.exit(failed ? 1 : 0);
