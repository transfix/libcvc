// arigl_web_download.mjs — the arigl-web page's loader (arigl-web.js) downloading the module it
// starts, in a real (headless) Chrome, with no wasm build: a stand-in module (fake.js beside
// fake.wasm and fake.zip, named by the page's data-module and data-python) takes the place of
// arigl.js / ariglx.js and reports what the loader handed it.
//
// The loader fetches the module's .wasm and the Python zip itself, before the module starts. A
// download that breaks off half-way (a dropped connection; Firefox resuming a partial copy left in
// its cache by a cut-off load) must be fetched again past the browser's cache, cache: 'reload', and
// the page must run. A file that breaks on every try must fail the page with its name and how much
// of it arrived, and a file the server refuses (404) must fail it at once, with no retry.
//
//   node arigl_web_download.mjs --chrome <chrome or chromium> [--keep]
//
// Node 22 (its WebSocket drives Chrome over the DevTools protocol). Exit status 0 when every case
// passes.

import {spawn} from 'node:child_process';
import fs from 'node:fs';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

const argv = process.argv.slice(2);
const opt = name => {
  const i = argv.indexOf(name);
  return i >= 0 ? argv[i + 1] : undefined;
};
const CHROME = opt('--chrome');
if (!CHROME) {
  console.error('usage: arigl_web_download.mjs --chrome PATH [--keep]');
  process.exit(2);
}
const WEB = fileURLToPath(new URL('..', import.meta.url)); // src/arigl/web

// The stand-in module: it runs the loader's preRun (the env, and the Python zip written into a
// file system it fakes), records the sizes it was handed, and says it is running, as arigl does
// once its document is loaded.
const FAKE_JS = `(() => {
  const M = window.Module;
  window.ENV = {};
  window.FS = {mkdirTree() {}, writeFile(p, d) { window.__zip = {path: p, size: d.length}; }};
  for (const f of M.preRun) f();
  window.__wasm = M.wasmBinary ? M.wasmBinary.length : -1;
  if (M.onRuntimeInitialized) M.onRuntimeInitialized();
  M.ariadneRunning();
})();`;
// The page: arigl-web.html with its script tag naming the stand-in.
const PAGE = fs.readFileSync(path.join(WEB, 'arigl-web.html'), 'utf8').replace(
    '<script src="arigl-web.js"></script>',
    '<script src="arigl-web.js" data-module="fake" data-python="fake.zip"></script>');
if (!PAGE.includes('data-module="fake"')) {
  console.error('arigl-web.html no longer has <script src="arigl-web.js"></script>');
  process.exit(2);
}
const WASM = Buffer.alloc(3 * 1048576, 1);
const ZIP = Buffer.alloc(1048576, 2);
const CSP = "default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self'; " +
            "connect-src 'self'; img-src 'self' data: blob:";

// What the server does with fake.wasm and fake.zip in the current case: 'ok' (all of it), 'once'
// (the first request breaks off at a third, the rest are whole), 'always' (every request breaks
// off at a third) or 404. Each request is counted, with whether it asked past the cache (Chrome's
// cache: 'reload' sends Pragma: no-cache).
let mode = {};
let hits = {};
function serveBlob(name, body, req, res) {
  const h = hits[name] = hits[name] || {n: 0, reloads: 0};
  h.n++;
  if (/no-cache/.test(req.headers.pragma || '')) h.reloads++;
  const how = mode[name] || 'ok';
  if (how === 404) {
    res.writeHead(404, {'Content-Type': 'text/plain'});
    res.end('not found');
    return;
  }
  const type = name.endsWith('.wasm') ? 'application/wasm' : 'application/zip';
  res.writeHead(200, {'Content-Type': type, 'Content-Length': String(body.length),
                      'Cache-Control': 'no-cache'});
  if (how === 'ok' || (how === 'once' && h.n > 1)) {
    res.end(body);
    return;
  }
  res.write(body.subarray(0, body.length / 3), () => res.socket.destroy());
}

const server = http.createServer((req, res) => {
  const p = new URL(req.url, 'http://x').pathname;
  if (p === '/' || p === '/index.html') {
    res.writeHead(200, {'Content-Type': 'text/html', 'Content-Security-Policy': CSP});
    return res.end(PAGE);
  }
  if (p === '/fake.wasm') return serveBlob('fake.wasm', WASM, req, res);
  if (p === '/fake.zip') return serveBlob('fake.zip', ZIP, req, res);
  if (p === '/fake.js') {
    res.writeHead(200, {'Content-Type': 'text/javascript'});
    return res.end(FAKE_JS);
  }
  if (p === '/arigl-web.js' || p === '/arigl-web.css') {
    res.writeHead(200, {'Content-Type': p.endsWith('.js') ? 'text/javascript' : 'text/css',
                        'Cache-Control': 'no-cache'});
    return res.end(fs.readFileSync(path.join(WEB, p.slice(1))));
  }
  res.writeHead(404, {'Content-Type': 'text/plain'}); // ariadne.config.json: the defaults
  res.end('not found');
});
await new Promise(r => server.listen(0, '127.0.0.1', r));
const ORIGIN = `http://127.0.0.1:${server.address().port}`;

const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'arigl-web-download-'));
const chrome = spawn(CHROME, [
  '--headless=new', '--remote-debugging-port=0', `--user-data-dir=${profile}`,
  '--no-first-run', '--no-default-browser-check', '--no-sandbox', 'about:blank'],
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

// One tab per case; poll the page's state until it leaves 'loading' (or 30 s go by).
async function run(url) {
  const r = await fetch(`http://${devtools.host}/json/new?about:blank`, {method: 'PUT'});
  const target = await r.json();
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
  await send('Page.navigate', {url});
  const until = Date.now() + 30000;
  let v = null;
  while (Date.now() < until) {
    const m = await send('Runtime.evaluate', {returnByValue: true, expression:
        'JSON.stringify(window.ariadneWeb ? {state: ariadneWeb.state, wasm: window.__wasm,' +
        ' zip: window.__zip || null, status: document.getElementById("status").textContent} :' +
        ' null)'});
    const s = m.result && m.result.result ? m.result.result.value : null;
    v = s ? JSON.parse(s) : null;
    if (v && v.state !== 'loading') break;
    await new Promise(res => setTimeout(res, 200));
  }
  ws.close();
  await fetch(`http://${devtools.host}/json/close/${target.id}`).catch(() => {});
  return v || {state: 'timeout', status: ''};
}

const ran = v => v.state === 'running' && v.wasm === WASM.length && v.zip &&
                 v.zip.size === ZIP.length && v.zip.path === '/py/lib/python312.zip';
const cases = [
  {
    name: 'a clean download starts the module with both files',
    mode: {},
    check: v => ran(v) && hits['fake.wasm'].n === 1 && hits['fake.zip'].n === 1 &&
                hits['fake.wasm'].reloads === 0,
  },
  {
    name: 'a .wasm that breaks off once is fetched again past the cache, and the page runs',
    mode: {'fake.wasm': 'once'},
    check: v => ran(v) && hits['fake.wasm'].n === 2 && hits['fake.wasm'].reloads === 1 &&
                hits['fake.zip'].n === 1,
  },
  {
    name: 'the Python zip breaking off once is fetched again too',
    mode: {'fake.zip': 'once'},
    check: v => ran(v) && hits['fake.zip'].n === 2 && hits['fake.zip'].reloads === 1,
  },
  {
    name: 'a .wasm that breaks off every time fails the page with its name and size',
    mode: {'fake.wasm': 'always'},
    check: v => v.state === 'failed' && hits['fake.wasm'].n === 3 && new RegExp(
        '^Could not start: downloading fake\\.wasm broke off after 1\\.0 of 3\\.0 MB, ' +
        '3 tries \\(.+\\)\\. Reload the page').test(v.status),
  },
  {
    name: 'a refused file (404) fails the page at once, with no retry',
    mode: {'fake.zip': 404},
    check: v => v.state === 'failed' && hits['fake.zip'].n === 1 &&
                /Could not start: HTTP 404 fetching fake\.zip/.test(v.status),
  },
];

let failures = 0;
for (const c of cases) {
  mode = c.mode;
  hits = {};
  // A fresh URL per case, so no case starts with another's copy in the cache.
  const v = await run(`${ORIGIN}/?case=${cases.indexOf(c)}`);
  for (const k of ['fake.wasm', 'fake.zip']) hits[k] = hits[k] || {n: 0, reloads: 0};
  const ok = c.check(v);
  if (!ok) failures++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${c.name}`);
  if (!ok) console.log(`      state=${v.state} status=${JSON.stringify(v.status)} ` +
                       `hits=${JSON.stringify(hits)} wasm=${v.wasm} zip=${JSON.stringify(v.zip)}`);
}

// Close Chrome over its browser DevTools socket, which stops every process it started.
try {
  const ws = new WebSocket(devtools.href);
  await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  ws.send(JSON.stringify({id: 1, method: 'Browser.close'}));
  await new Promise(r => { chrome.once('exit', r); setTimeout(r, 10000); });
} catch (e) {
  chrome.kill();
}
server.close();
if (!argv.includes('--keep')) {
  try {
    fs.rmSync(profile, {recursive: true, force: true, maxRetries: 5, retryDelay: 200});
  } catch (e) {
    console.warn(`could not remove ${profile}: ${e.message}`);
  }
}
console.log(failures ? `${failures} of ${cases.length} failed` : `all ${cases.length} passed`);
process.exit(failures ? 1 : 0);
