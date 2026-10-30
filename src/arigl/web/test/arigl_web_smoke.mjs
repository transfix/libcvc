// arigl_web_smoke.mjs — arigl-web in a real (headless) Chrome: the shell loads a document set from
// its own origin, and refuses one from another origin or one that is not there. When the package
// has ari's terminal pages (ari.html, ari_tvision.html), the same document set must load there too,
// its text on the xterm.js screen.
//
//   node arigl_web_smoke.mjs --web <share/arigl-web> --docset <src/cvc/tests/data/ariadne_docset> \
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
// with an ordinary site's headers: no COOP/COEP (the single-threaded build needs none) and a strict
// Content-Security-Policy (no inline code or style, no eval, nothing from another origin), so a
// pass also says both pages run under one. Chrome is driven over the DevTools protocol; each case
// reads window.ariadneWeb (the page's state, everything the runtime printed, and ari's terminal
// text) and fails on any CSP violation the page reports. Exit status 0 when every case passes.

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

// URL path -> file: /apps/app/<shell file> from WEB, the rest of /apps/ from DOCSET. /apps/cfg/ is
// the same app with a page config (CONFIG) that passes the page's origin as a document argument.
const CONFIG = JSON.stringify({args: ['--instance', '{origin}']});
function fileFor(urlPath) {
  const p = decodeURIComponent(urlPath);
  if (!p.startsWith('/apps/') || p.includes('..')) return null;
  let rest = p.slice('/apps/'.length);
  if (rest === 'cfg/ariadne.config.json') return {json: CONFIG};
  if (rest.startsWith('cfg/')) rest = 'app/' + rest.slice(4);
  if (rest === 'app/' || rest === 'app/index.html') return path.join(WEB, 'index.html');
  const shell = rest.startsWith('app/') ? path.join(WEB, rest.slice(4)) : null;
  if (shell && fs.existsSync(shell) && fs.statSync(shell).isFile()) return shell;
  return path.join(DOCSET, rest);
}

const server = http.createServer((req, res) => {
  const f = fileFor(new URL(req.url, 'http://x').pathname);
  if (f && f.json) {
    res.writeHead(200, {'Content-Type': 'application/json', 'Cache-Control': 'no-cache'});
    res.end(f.json);
    return;
  }
  if (!f || !fs.existsSync(f) || !fs.statSync(f).isFile()) {
    res.writeHead(404, {'Content-Type': 'text/plain'});
    res.end('not found');
    return;
  }
  const headers = {'Content-Type': TYPES[path.extname(f)] || 'application/octet-stream',
                   'Cache-Control': 'no-cache'};
  headers['Content-Security-Policy'] = CSP;
  res.writeHead(200, headers);
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

// One tab per case, driven through its own DevTools socket. It opens blank, gets a listener that
// records the page's CSP violations (injected by DevTools, so the policy does not apply to it),
// then goes to the case's URL.
async function openPage(url) {
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
  const evaluate = async expr => {
    const m = await send('Runtime.evaluate', {expression: expr, returnByValue: true});
    return m.result && m.result.result ? m.result.result.value : undefined;
  };
  await send('Page.enable');
  await send('Page.addScriptToEvaluateOnNewDocument', {
    source: 'window.__csp = []; document.addEventListener("securitypolicyviolation", ' +
            'e => window.__csp.push(e.effectiveDirective + " " + (e.blockedURI || "inline")));'});
  await send('Page.navigate', {url});
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

// Wait until the case's check passes, or the page has left 'loading' and a few seconds more have
// gone by (ari draws after it reports running); the state, the runtime's output, the terminal text.
async function settle(page, check) {
  const until = Date.now() + TIMEOUT_MS;
  let settledAt = 0;
  for (;;) {
    const s = await page.evaluate(
        'JSON.stringify(window.ariadneWeb ? {state: ariadneWeb.state, log: ariadneWeb.log,' +
        ' text: ariadneWeb.text ? ariadneWeb.text() : "", csp: window.__csp || [],' +
        ' status: document.getElementById("status").textContent} : null)');
    const v = s ? JSON.parse(s) : null;
    if (v && check(v)) return v;
    if (v && v.state !== 'loading' && !settledAt) settledAt = Date.now();
    if ((settledAt && Date.now() > settledAt + 15000) || Date.now() > until)
      return v || {state: 'timeout', log: [], text: '', status: '', csp: []};
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

// ari's terminal page, when the package has it: the same set in the FTXUI backend.
if (fs.existsSync(path.join(WEB, 'ari.html'))) {
  const shows = v => /from lib\/panel\.ari/.test(v.text) && /from shared\/note\.ari/.test(v.text);
  cases.push(
    {
      name: 'ari: a same-origin document set loads and draws in the terminal',
      url: `${ORIGIN}/apps/app/ari.html?src=main.ari`,
      check: v => v.state === 'running' && shows(v) && /Agents/.test(v.text) &&
                  !v.log.some(l => /could not be resolved|unknown unit/.test(l)),
    },
    {
      name: 'ari: the default document is main.ari',
      url: `${ORIGIN}/apps/app/ari.html`,
      check: v => v.state === 'running' && shows(v),
    },
    {
      name: 'ari: {origin} in a config argument is the page\'s origin',
      url: `${ORIGIN}/apps/cfg/ari.html`,
      check: v => v.state === 'running' && shows(v) && v.text.includes(`instance: ${ORIGIN}`),
    },
    {
      name: 'ari: a cross-origin src is refused, and the terminal says why',
      url: `${ORIGIN}/apps/app/ari.html?src=` +
           encodeURIComponent(`http://localhost:${PORT}/apps/app/main.ari`),
      check: v => v.state === 'failed' && /not this page's origin/.test(v.text),
    });
}
// The tvision desktop's page, when the package has it (cvcpkg's tvision wasm in the deps).
if (fs.existsSync(path.join(WEB, 'ari_tvision.html'))) {
  cases.push({
    name: 'ari_tvision: the same set on the tvision desktop',
    url: `${ORIGIN}/apps/app/ari_tvision.html?src=main.ari`,
    check: v => v.state === 'running' && /from lib\/panel\.ari/.test(v.text) &&
                /from shared\/note\.ari/.test(v.text),
  });
}

let failures = 0;
for (const c of cases) {
  const page = await openPage(c.url);
  const v = await settle(page, c.check);
  const ok = !!(v && c.check(v) && v.csp.length === 0);
  if (!ok) failures++;
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${c.name}  [${v ? v.state : 'no state'}]`);
  if (SHOTS) {
    await new Promise(r => setTimeout(r, 3000)); // a few frames past the load
    fs.mkdirSync(SHOTS, {recursive: true});
    await page.screenshot(path.join(SHOTS, `${cases.indexOf(c) + 1}.png`));
  }
  if (!ok && v) {
    console.log('  status: ' + v.status);
    for (const x of v.csp) console.log('  CSP violation: ' + x);
    for (const l of v.log.slice(-25)) console.log('  | ' + l);
    if (v.text) for (const l of v.text.split('\n').filter(x => x.trim()).slice(-25)) console.log('  > ' + l);
  }
  await page.close();
}

chrome.kill();
server.close();
if (!argv.includes('--keep')) fs.rmSync(profile, {recursive: true, force: true});
console.log(failures ? `${failures} of ${cases.length} failed` : `all ${cases.length} passed`);
process.exit(failures ? 1 : 0);
