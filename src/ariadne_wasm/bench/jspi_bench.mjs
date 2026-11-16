#!/usr/bin/env node
// jspi_bench.mjs: JSPI against Asyncify for WebAssembly guests in the browser (G2's last part;
// docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.5). It builds the bench's guests and hosts, serves
// jspi_bench.html on 127.0.0.1 (cross-origin isolated, for the finer clock), runs it in headless
// Chrome, and prints what the page measured (jspi_bench.js says what).
//
// With emsdk's environment sourced (emcc on PATH, wasm-opt in $EMSDK/upstream/bin):
//
//   node jspi_bench.mjs --meter <ari_wasm_meter[.js]> --chrome <chrome> [--out DIR]
//        [--json FILE] [--query 'reps=9&slices=64000'] [--sizes A.wasm B.wasm ...]
//        [--wasm-opt W] [--high-priority]
//
// --meter is the gas pass as a command (meter_main.cpp): a native build's, or an Emscripten
// build's ari_wasm_meter.js, which runs in this node. --sizes meters other modules too and gives
// only their sizes and the transform's times (a real program's growth, which the bench's small
// guests can't show). --query sets the page's parameters (jspi_bench.js). --high-priority, on
// Windows, runs Chrome's processes at High. The guests take CVC_WASM_GUEST_FEATURES from
// ariadne_wasm.cmake, as cvc_ariadne_wasm_transform does.

import {spawnSync, spawn} from 'node:child_process';
import fs from 'node:fs';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import zlib from 'node:zlib';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const argv = process.argv.slice(2);
const opt = (k, d) => { const i = argv.indexOf(k); return i >= 0 ? argv[i + 1] : d; };
const list = k => {
  const i = argv.indexOf(k);
  if (i < 0) return [];
  const out = [];
  for (let j = i + 1; j < argv.length && !argv[j].startsWith('--'); ++j) out.push(argv[j]);
  return out;
};
const METER = opt('--meter');
const CHROME = opt('--chrome');
const OUT = path.resolve(opt('--out', fs.mkdtempSync(path.join(os.tmpdir(), 'jspi-bench-'))));
const JSON_OUT = opt('--json');
const QUERY = opt('--query', '');
const SIZES = list('--sizes');
const PRIORITY = argv.includes('--high-priority') && process.platform == 'win32';
if (!METER || !CHROME) {
  console.error('usage: node jspi_bench.mjs --meter <ari_wasm_meter> --chrome <chrome> ' +
                '[--out DIR] [--json FILE] [--query Q] [--sizes A.wasm ...] [--wasm-opt W] ' +
                '[--high-priority]');
  process.exit(2);
}
fs.mkdirSync(OUT, {recursive: true});

// ── building ──

const win = process.platform == 'win32';
function run(cmd, args) {
  const t0 = performance.now();
  const r = win ? spawnSync([cmd, ...args].map(a => (/[\s&()]/.test(a) ? `"${a}"` : a)).join(' '),
                            {shell: true, stdio: ['ignore', 'inherit', 'inherit']})
                : spawnSync(cmd, args, {stdio: ['ignore', 'inherit', 'inherit']});
  if (r.status !== 0) throw new Error(`${cmd} ${args.join(' ')}: exit ${r.status}`);
  return performance.now() - t0;
}
// emsdk's binaryen isn't on its PATH: wasm-opt is beside clang, in $EMSDK/upstream/bin.
const WASM_OPT = opt('--wasm-opt') || (() => {
  const f = process.env.EMSDK &&
            path.join(process.env.EMSDK, 'upstream', 'bin', win ? 'wasm-opt.exe' : 'wasm-opt');
  return f && fs.existsSync(f) ? f : 'wasm-opt';
})();
const meter = (input, output) => (METER.endsWith('.js') ? run(process.execPath, [METER, input, output])
                                                       : run(METER, [input, output]));
// cvc_ariadne_wasm_transform's features, read from where it sets them.
const FEATURES = (() => {
  const cm = fs.readFileSync(path.join(HERE, '..', 'ariadne_wasm.cmake'), 'utf8');
  const m = /set\(CVC_WASM_GUEST_FEATURES([^)]*)\)/.exec(cm);
  if (!m) throw new Error('ariadne_wasm.cmake sets no CVC_WASM_GUEST_FEATURES');
  return m[1].split(/\s+/).filter(Boolean);
})();
const o = f => path.join(OUT, f);

// A module through the transform both ways: metered, then Asyncify (today's), or only -O1 (what a
// JSPI guest needs). The times are the transform's, per step.
function transform(input, stem) {
  const t = {};
  t.binary = run(WASM_OPT, [input, ...FEATURES, '-o', o(`${stem}.binary`)]);
  t.meter = meter(o(`${stem}.binary`), o(`${stem}.metered.wasm`));
  t.asyncify = run(WASM_OPT, [o(`${stem}.metered.wasm`), ...FEATURES, '--asyncify',
                                '--pass-arg=asyncify-imports@ari.*', '-O1', '-o',
                                o(`${stem}.asyncify.wasm`)]);
  t.jspi = run(WASM_OPT, [o(`${stem}.metered.wasm`), ...FEATURES, '-O1', '-o',
                            o(`${stem}.jspi.wasm`)]);
  return t;
}
const sizeOf = f => {
  const b = fs.readFileSync(f);
  return {raw: b.length, gzip: zlib.gzipSync(b, {level: 9}).length};
};

console.log(`building into ${OUT}`);
run('emcc', [path.join(HERE, 'jspi_guest.c'), '-O2', '-sSTANDALONE_WASM', '--no-entry',
             '-sINITIAL_MEMORY=16MB', '-sSTACK_SIZE=1MB', '-o', o('guest.plain.wasm')]);
transform(o('guest.plain.wasm'), 'guest');
// The hosts (jspi_host.cpp): under Asyncify, unwinding into a buffer as large as ariglx's
// (src/arigl/CMakeLists.txt), or JSPI; with no exceptions, Emscripten's JS ones (libcvc's
// -fexceptions, whose invoke_* put JS frames between the host's), or wasm's. The page loads each
// from host_<name>.js, whose MODULARIZE factory is window.host_<name> (- as _).
const HOSTS = {
  'asyncify': ['-sASYNCIFY', '-sASYNCIFY_STACK_SIZE=131072', '-fno-exceptions'],
  'asyncify-eh': ['-sASYNCIFY', '-sASYNCIFY_STACK_SIZE=131072', '-fexceptions'],
  'jspi': ['-sJSPI', '-sJSPI_EXPORTS=run', '-fno-exceptions'],
  'jspi-eh': ['-sJSPI', '-sJSPI_EXPORTS=run', '-fexceptions'],
  'jspi-wasm-eh': ['-sJSPI', '-sJSPI_EXPORTS=run', '-fwasm-exceptions'],
};
const hostSizes = {};
for (const [name, flags] of Object.entries(HOSTS)) {
  const stem = 'host_' + name.replace(/-/g, '_');
  run('emcc', [path.join(HERE, 'jspi_host.cpp'), '-O2', '-sMODULARIZE', '-sENVIRONMENT=web',
               '-sEXPORTED_RUNTIME_METHODS=ccall', '-sEXPORTED_FUNCTIONS=_run', ...flags,
               `-sEXPORT_NAME=${stem}`, '-o', o(`${stem}.js`)]);
  hostSizes[name] = sizeOf(o(`${stem}.wasm`));
}

const realSizes = [];
for (const f of SIZES) {
  const stem = 'real.' + path.basename(f, '.wasm');
  console.log(`transforming ${f}`);
  const times = transform(path.resolve(f), stem);
  realSizes.push({module: path.basename(f), times,
                  plain: sizeOf(f), metered: sizeOf(o(`${stem}.metered.wasm`)),
                  asyncify: sizeOf(o(`${stem}.asyncify.wasm`)), jspi: sizeOf(o(`${stem}.jspi.wasm`))});
  for (const s of ['binary', 'metered.wasm', 'asyncify.wasm', 'jspi.wasm'])
    fs.rmSync(o(`${stem}.${s}`), {force: true});
}

// ── serving ──

const TYPES = {'.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm'};
const server = http.createServer((req, res) => {
  const name = decodeURIComponent(new URL(req.url, 'http://x').pathname).replace(/^\/+/, '');
  const f = ['jspi_bench.html', 'jspi_bench.js'].includes(name) ? path.join(HERE, name)
                                                                : path.join(OUT, path.basename(name));
  if (!name || !fs.existsSync(f)) {
    res.writeHead(404);
    res.end();
    return;
  }
  res.writeHead(200, {'Content-Type': TYPES[path.extname(f)] || 'application/octet-stream',
                      'Cache-Control': 'no-store', 'Cross-Origin-Opener-Policy': 'same-origin',
                      'Cross-Origin-Embedder-Policy': 'require-corp'});
  fs.createReadStream(f).pipe(res);
});
await new Promise(r => server.listen(0, '127.0.0.1', r));
const URL_ = `http://127.0.0.1:${server.address().port}/jspi_bench.html${QUERY ? '?' + QUERY : ''}`;

// ── Chrome ──

const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'jspi-bench-chrome-'));
const chrome = spawn(CHROME, ['--headless=new', '--remote-debugging-port=0',
                              `--user-data-dir=${profile}`, '--no-first-run',
                              '--no-default-browser-check', 'about:blank'],
                     {stdio: ['ignore', 'ignore', 'pipe']});
const devtools = await new Promise((resolve, reject) => {
  let err = '';
  const t = setTimeout(() => reject(new Error('Chrome did not start:\n' + err)), 30000);
  chrome.stderr.on('data', d => {
    err += d;
    const m = /DevTools listening on (ws:\/\/[^\s]+)/.exec(err);
    if (m) { clearTimeout(t); resolve(new URL(m[1])); }
  });
  chrome.on('exit', c => reject(new Error(`Chrome exited (${c}):\n` + err)));
});
const target = await (await fetch(`http://${devtools.host}/json/new?about:blank`, {method: 'PUT'})).json();
const ws = new WebSocket(target.webSocketDebuggerUrl);
await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
let id = 0;
const pending = new Map();
ws.onmessage = e => {
  const m = JSON.parse(e.data);
  if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); }
};
const send = (method, params = {}) => new Promise(res => {
  pending.set(++id, res);
  ws.send(JSON.stringify({id, method, params}));
});
const evaluate = async expr => {
  const m = await send('Runtime.evaluate', {expression: expr, returnByValue: true});
  return m.result && m.result.result ? m.result.result.value : undefined;
};
const version = (await send('Browser.getVersion')).result;
// --high-priority (Windows): Chrome and its helpers, the tab's renderer among them, at High, so a
// busy machine's other work disturbs the timings less.
if (PRIORITY) {
  const ps = `$ids = @(${chrome.pid}); $all = Get-CimInstance Win32_Process; do { $n = $ids.Count; ` +
             '$ids = @(@($ids) + @($all | Where-Object { $ids -contains $_.ParentProcessId } | ' +
             'ForEach-Object { $_.ProcessId }) | Select-Object -Unique) } while ($ids.Count -gt $n); ' +
             "foreach ($i in $ids) { try { (Get-Process -Id $i).PriorityClass = 'High' } catch {} }; " +
             '$ids.Count';
  const r = spawnSync('powershell', ['-NoProfile', '-Command', ps], {encoding: 'utf8'});
  console.log(`Chrome's ${(r.stdout || '').trim() || 'no'} processes at high priority`);
}
await send('Page.navigate', {url: URL_});
console.log(`running ${URL_} in ${version.product}`);

let done;
const deadline = Date.now() + 20 * 60 * 1000;
let shown = 0;
while (!(done = await evaluate('window.benchDone')) && Date.now() < deadline) {
  await new Promise(r => setTimeout(r, 1000));
  const lines = (await evaluate('document.getElementById("log") && document.getElementById("log").textContent')) || '';
  const all = lines.split('\n').filter(Boolean);
  for (; shown < all.length; ++shown) console.log('  ' + all[shown].slice(0, 160));
}
const result = await evaluate('JSON.stringify(window.benchResult)');
ws.close();
await new Promise(r => { chrome.once('exit', r); chrome.kill(); });
server.close();
// Chrome's helpers can hold the profile's files a moment after it exits (Windows).
try {
  fs.rmSync(profile, {recursive: true, force: true, maxRetries: 10, retryDelay: 300});
} catch (e) {
  console.error(`left ${profile}: ${e.code || e}`);
}
if (done !== 'ok') {
  console.error(done || 'the page did not finish in 20 minutes');
  process.exit(1);
}

// ── the report ──

const R = JSON.parse(result);
R.browser = version.product;
R.hostSizes = hostSizes;
R.realSizes = realSizes;
if (JSON_OUT) fs.writeFileSync(JSON_OUT, JSON.stringify(R, null, 2));

const kb = n => (n / 1024).toFixed(1);
const mb = n => (n / 1048576).toFixed(1);
const ms = n => n.toFixed(1);
const per = (t, t0, n) => (((t - t0) * 1000) / Math.max(1, n)).toFixed(2);
console.log(`\n${R.browser}, the median of ${R.env.reps} rounds, crossOriginIsolated ` +
            `${R.env.crossOriginIsolated}${PRIORITY ? ', Chrome at high priority' : ''}\n`);
console.log('| guest | raw | gzip | compile (ms) |\n|---|---|---|---|');
for (const k of ['plain', 'metered', 'asyncify'])
  console.log(`| ${k} | ${kb(R.sizes[k].raw)} KiB | ${kb(R.sizes[k].gzip)} KiB | ${R.compile[k].toFixed(2)} |`);
console.log('\nUnsliced, ms:\n\n| workload | gas | plain | metered | asyncify | jspi |\n|---|---|---|---|---|---|');
for (const w of R.workloads)
  console.log(`| ${w.name}(${w.arg}) | ${(w.gas / 1e6).toFixed(1)} M | ${ms(w.unsliced.plain)} | ` +
              `${ms(w.unsliced.metered)} | ${ms(w.unsliced.asyncify)} | ${ms(w.unsliced.jspi)} |`);
const slicedRows = (title, rows) => {
  console.log(`\n${title}, ms, each with its unsliced call in the same rounds (and a pause's cost ` +
              'over it, µs):\n');
  console.log('| workload | slice | pauses | asyncify (unsliced) | jspi (unsliced) | asyncify µs/pause | jspi µs/pause |\n|---|---|---|---|---|---|---|');
  for (const [name, s] of rows)
    console.log(`| ${name} | ${s.slice} | ${s.pauses} | ${ms(s.asyncify)} (${ms(s.asyncify0)}) | ` +
                `${ms(s.jspi)} (${ms(s.jspi0)}) | ${per(s.asyncify, s.asyncify0, s.pauses)} | ` +
                `${per(s.jspi, s.jspi0, s.pauses)} |`);
};
slicedRows('Sliced', R.workloads.flatMap(w => w.sliced.map(s => [w.name, s])));
slicedRows(`Pauses (${R.env.pauseArg} iterations; deep is 48 frames down)`,
           R.pauses.map(p => [p.name, p]));
console.log(`\nPolled from a host (spin(${R.env.pauseArg})), ms, with the guest's unsliced call ` +
            '(and a poll\'s cost over it, µs):\n');
console.log('| host | slice | depth | polls | asyncify guest (unsliced) | jspi guest (unsliced) | asyncify µs/poll | jspi µs/poll |\n|---|---|---|---|---|---|---|---|');
for (const h of R.hosts.filter(h => h.runs !== false))
  console.log(`| ${h.host} | ${h.slice} | ${h.depth} | ${h.pauses} | ${ms(h.asyncifyGuest)} (${ms(h.asyncify0)}) | ` +
              (h.error ? "can't wait | " : `${ms(h.jspiGuest)} (${ms(h.jspi0)}) | `) +
              `${per(h.asyncifyGuest, h.asyncify0, h.pauses)} | ` +
              (h.error ? '— |' : `${per(h.jspiGuest, h.jspi0, h.pauses)} |`));
for (const e of new Set(R.hosts.filter(h => h.error).map(
         h => `${h.host} ${h.runs === false ? "doesn't run from two frames down" : "can't wait for a JSPI guest"}: ${h.error}`)))
  console.log(`\n${e}`);
console.log('\nHost modules: ' +
            Object.entries(hostSizes).map(([k, v]) => `${k} ${kb(v.raw)} KiB`).join(', '));
if (realSizes.length) {
  console.log('\nReal modules, MiB raw (gzip), and the transform\'s seconds:\n');
  console.log('| module | plain | metered | metered + asyncify | metered -O1 | meter s | asyncify s |\n|---|---|---|---|---|---|---|');
  for (const r of realSizes)
    console.log(`| ${r.module} | ${mb(r.plain.raw)} (${mb(r.plain.gzip)}) | ${mb(r.metered.raw)} (${mb(r.metered.gzip)}) | ` +
                `${mb(r.asyncify.raw)} (${mb(r.asyncify.gzip)}) | ${mb(r.jspi.raw)} (${mb(r.jspi.gzip)}) | ` +
                `${(r.times.meter / 1000).toFixed(1)} | ${(r.times.asyncify / 1000).toFixed(1)} |`);
}
