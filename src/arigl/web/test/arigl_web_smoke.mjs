// arigl_web_smoke.mjs — arigl-web in a real (headless) Chrome: the shell loads a document set from
// its own origin, and refuses one from another origin or one that is not there.
//
//   node arigl_web_smoke.mjs --web <share/arigl-web> --docset <src/arigl/tests/docset> \
//        --chrome <chrome or chromium> [--shots DIR] [--keep]
//
// --shots writes each case's page as DIR/<n>.png.
//
// It serves a site of its own on 127.0.0.1 (no other dependency: Node 22's http and WebSocket):
//
//   /apps/app/          the arigl-web files (index.html, arigl-web.js/.css, arigl.js/.wasm)
//   /apps/app/main.ari  the docset's app/, which imports lib/panel.ari and ../shared/note.ari
//   /apps/shared/       the docset's shared/
//
// with an ordinary site's headers: no COOP/COEP (the single-threaded build needs none) and a
// strict Content-Security-Policy (no inline code, no eval), so a pass also says the page runs
// under one. Chrome is driven over the DevTools protocol; each case reads window.ariglWeb (the
// page's state and everything arigl printed). Exit status 0 when every case passes.

import {spawn} from 'node:child_process';
import fs from 'node:fs';
import http from 'node:http';
import os from 'node:os';
import path from 'node:path';

const argv = process.argv.slice(2);
const opt = name => {
  const i = argv.indexOf(name);
  return i >= 0 ? argv[i + 1] : undefined;
};
const WEB = opt('--web'), DOCSET = opt('--docset'), CHROME = opt('--chrome');
const SHOTS = opt('--shots');
if (!WEB || !DOCSET || !CHROME) {
  console.error('usage: arigl_web_smoke.mjs --web DIR --docset DIR --chrome PATH [--keep]');
  process.exit(2);
}
const TIMEOUT_MS = Number(process.env.ARIGL_WEB_SMOKE_TIMEOUT_MS || 180000);

const CSP = "default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self'; " +
            "connect-src 'self'; img-src 'self' data: blob:";
const TYPES = {'.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css',
               '.wasm': 'application/wasm', '.json': 'application/json', '.ari': 'text/plain'};

// URL path -> file: /apps/app/<shell file> from WEB, the rest of /apps/ from DOCSET.
function fileFor(urlPath) {
  const p = decodeURIComponent(urlPath);
  if (!p.startsWith('/apps/') || p.includes('..')) return null;
  const rest = p.slice('/apps/'.length);
  if (rest === 'app/' || rest === 'app/index.html') return path.join(WEB, 'index.html');
  const shell = rest.startsWith('app/') ? path.join(WEB, rest.slice(4)) : null;
  if (shell && fs.existsSync(shell) && fs.statSync(shell).isFile()) return shell;
  return path.join(DOCSET, rest);
}

const server = http.createServer((req, res) => {
  const f = fileFor(new URL(req.url, 'http://x').pathname);
  if (!f || !fs.existsSync(f) || !fs.statSync(f).isFile()) {
    res.writeHead(404, {'Content-Type': 'text/plain'});
    res.end('not found');
    return;
  }
  res.writeHead(200, {'Content-Type': TYPES[path.extname(f)] || 'application/octet-stream',
                      'Content-Security-Policy': CSP, 'Cache-Control': 'no-cache'});
  fs.createReadStream(f).pipe(res);
});
await new Promise(r => server.listen(0, '127.0.0.1', r));
const PORT = server.address().port;
const ORIGIN = `http://127.0.0.1:${PORT}`;

// Chrome, headless, its DevTools endpoint on a port it picks.
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'arigl-web-smoke-'));
const chrome = spawn(CHROME, [
  '--headless=new', '--remote-debugging-port=0', `--user-data-dir=${profile}`,
  '--no-first-run', '--no-default-browser-check', '--no-sandbox',
  '--enable-unsafe-swiftshader', '--use-angle=swiftshader', '--ignore-gpu-blocklist',
  '--window-size=1024,768', 'about:blank'], {stdio: ['ignore', 'ignore', 'pipe']});
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

// One tab per case, driven through its own DevTools socket.
async function openPage(url) {
  const r = await fetch(`http://${devtools.host}/json/new?${encodeURIComponent(url)}`,
                        {method: 'PUT'});
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
  const evaluate = async expr => {
    const m = await send('Runtime.evaluate', {expression: expr, returnByValue: true});
    return m.result && m.result.result ? m.result.result.value : undefined;
  };
  const close = async () => {
    ws.close();
    await fetch(`http://${devtools.host}/json/close/${target.id}`).catch(() => {});
  };
  const screenshot = async file => {
    const m = await send('Page.captureScreenshot', {format: 'png'});
    if (m.result && m.result.data) fs.writeFileSync(file, Buffer.from(m.result.data, 'base64'));
  };
  return {evaluate, screenshot, close};
}

// Wait for the page to leave 'loading'; the state and arigl's output.
async function settle(page) {
  const until = Date.now() + TIMEOUT_MS;
  for (;;) {
    const s = await page.evaluate(
        'JSON.stringify(window.ariglWeb ? {state: ariglWeb.state, log: ariglWeb.log,' +
        ' status: document.getElementById("status").textContent} : null)');
    const v = s ? JSON.parse(s) : null;
    if (v && v.state !== 'loading') return v;
    if (Date.now() > until) return v || {state: 'timeout', log: [], status: ''};
    await new Promise(r => setTimeout(r, 500));
  }
}

const cases = [
  {
    name: 'a same-origin document set loads and runs',
    url: `${ORIGIN}/apps/app/?src=main.ari`,
    check: v => v.state === 'running' && v.log.some(l => /3 file\(s\) from/.test(l)) &&
                !v.log.some(l => /could not be resolved|unknown unit/.test(l)),
  },
  {
    name: 'the default document is main.ari',
    url: `${ORIGIN}/apps/app/`,
    check: v => v.state === 'running',
  },
  {
    name: 'a cross-origin src is refused',
    // localhost and 127.0.0.1 are different origins on the same server.
    url: `${ORIGIN}/apps/app/?src=${encodeURIComponent(`http://localhost:${PORT}/apps/app/main.ari`)}`,
    check: v => v.state === 'failed' && v.log.some(l => /not this page's origin/.test(l)),
  },
  {
    name: 'a missing document shows the error page',
    url: `${ORIGIN}/apps/app/?src=nope.ari`,
    check: v => v.state === 'failed' && v.log.some(l => /HTTP 404/.test(l)) &&
                /Could not run/.test(v.status),
  },
];

let failures = 0;
for (const c of cases) {
  const page = await openPage(c.url);
  const v = await settle(page);
  const ok = !!(v && c.check(v));
  if (!ok) failures++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${c.name}  [${v ? v.state : 'no state'}]`);
  if (SHOTS) {
    await new Promise(r => setTimeout(r, 3000)); // a few frames past the load
    fs.mkdirSync(SHOTS, {recursive: true});
    await page.screenshot(path.join(SHOTS, `${cases.indexOf(c) + 1}.png`));
  }
  if (!ok && v) {
    console.log('  status: ' + v.status);
    for (const l of v.log.slice(-25)) console.log('  | ' + l);
  }
  await page.close();
}

chrome.kill();
server.close();
if (!argv.includes('--keep')) fs.rmSync(profile, {recursive: true, force: true});
console.log(failures ? `${failures} of ${cases.length} failed` : `all ${cases.length} passed`);
process.exit(failures ? 1 : 0);
