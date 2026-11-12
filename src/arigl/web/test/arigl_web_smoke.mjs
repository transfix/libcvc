// arigl_web_smoke.mjs — arigl-web in a real (headless) Chrome: the shell loads a document set from
// its own origin, and refuses one from another origin or one that is not there. When the package
// has ari's terminal pages (ari.html, ari_tvision.html), the same document set must load there too,
// its text on the xterm.js screen. A document whose program fails while it runs must reach each
// page's warning hook (Module.ariadneWarning) and show there. A document that picks a file (the
// page's <input type=file>, handed a File by the test) must send it in chunks to the page's origin
// (/apps/up), every byte once, on both pages (docs/ARIADNE.md §10.8).
//
// The same runs against ariglx-web (share/ariglx-web, the shell with an embedded Python): its page
// must run the same document set, and with --pydocs (src/arigl/web/test/python) a document's
// python: block with numpy and pycvc_gl, one with the stdlib's expat and SHA-2, a Python program
// in place of the document, and a block that raises must each do what they do natively, and in
// the full tier a VTK pipeline through
// vtkmodules too. Python's own report says how long the imports and a plain 200000-step loop
// took in the page, which this prints.
//
//   node arigl_web_smoke.mjs --web <share/arigl-web> --docset <src/cvc/tests/data/ariadne_docset> \
//        --chrome <chrome or chromium> [--pydocs DIR] [--shots DIR] [--keep]
//
// --shots writes each case's page as DIR/<n>.png.
//
// It serves a site of its own on 127.0.0.1 (no other dependency: Node 22's http and WebSocket):
//
//   /apps/app/          the arigl-web files (index.html, arigl-web.js/.css, arigl.js/.wasm)
//   /apps/app/main.ari  the docset's app/, which imports lib/panel.ari and ../shared/note.ari
//   /apps/shared/       the docset's shared/
//   /apps/py/           ariglx-web's files again, beside the --pydocs documents; /apps/pyprog/ the
//                       same with a page config that runs program.py and preloads program.ari
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
const SHOTS = opt('--shots'), PYDOCS = opt('--pydocs');
if (!WEB || !DOCSET || !CHROME) {
  console.error('usage: arigl_web_smoke.mjs --web DIR --docset DIR --chrome PATH [--pydocs DIR] ' +
                '[--shots DIR] [--keep]');
  process.exit(2);
}
// ariglx-web's package (ariglx.js, python312.zip); --pydocs is for it alone.
const PYTHON = fs.existsSync(path.join(WEB, 'ariglx.js'));
if (PYDOCS && !PYTHON) {
  console.error(`--pydocs: ${WEB} is not ariglx-web (no ariglx.js)`);
  process.exit(2);
}
const TIMEOUT_MS = Number(process.env.ARIGL_WEB_SMOKE_TIMEOUT_MS || 180000);

const CSP = "default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self'; " +
            "connect-src 'self'; img-src 'self' data: blob:";
const TYPES = {'.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css',
               '.wasm': 'application/wasm', '.json': 'application/json', '.ari': 'text/plain',
               '.py': 'text/plain', '.zip': 'application/zip'};

// URL path -> file: /apps/app/<shell file> from WEB, the rest of /apps/ from DOCSET. /apps/cfg/ is
// the same app with a page config (CONFIG) as an app like nicotina's would have: the network on for
// the page's own origin (--net), and that origin as a document argument.
const CONFIG = JSON.stringify({flags: ['--net'], args: ['--instance', '{origin}']});
// /apps/pyprog/: a Python program in place of the document, and the document it loads (a program's
// own files are not followed, so the page preloads them).
const PY_CONFIG = JSON.stringify({src: 'program.py', preload: ['program.ari']});
function fileFor(urlPath) {
  const p = decodeURIComponent(urlPath);
  if (!p.startsWith('/apps/') || p.includes('..')) return null;
  let rest = p.slice('/apps/'.length);
  if (rest === 'cfg/ariadne.config.json') return {json: CONFIG};
  if (PYDOCS && rest === 'pyprog/ariadne.config.json') return {json: PY_CONFIG};
  if (PYDOCS && (rest.startsWith('py/') || rest.startsWith('pyprog/'))) {
    const name = rest.slice(rest.indexOf('/') + 1);
    if (name === '' || name === 'index.html') return path.join(WEB, 'index.html');
    const shell = path.join(WEB, name);
    if (fs.existsSync(shell) && fs.statSync(shell).isFile()) return shell;
    return path.join(PYDOCS, name);
  }
  if (rest.startsWith('cfg/')) rest = 'app/' + rest.slice(4);
  if (rest === 'app/' || rest === 'app/index.html') return path.join(WEB, 'index.html');
  const shell = rest.startsWith('app/') ? path.join(WEB, rest.slice(4)) : null;
  if (shell && fs.existsSync(shell) && fs.statSync(shell).isFile()) return shell;
  return path.join(DOCSET, rest);
}

// /apps/up: a resumable upload endpoint, as txwtf's PATCH /social/files/resumable/{id} is. A
// PATCH appends its body at Upload-Offset and answers {"offset": N}, or a 409 with the offset it
// has when the client's is elsewhere.
let uploaded = Buffer.alloc(0);
function upload(req, res) {
  const parts = [];
  req.on('data', d => parts.push(d));
  req.on('end', () => {
    const body = Buffer.concat(parts);
    const json = (status, value, headers = {}) => {
      res.writeHead(status, {'Content-Type': 'application/json', ...headers});
      res.end(JSON.stringify(value));
    };
    if (req.method !== 'PATCH') return json(405, {});
    if (req.headers['upload-offset'] !== String(uploaded.length))
      return json(409, {detail: 'Offset mismatch'}, {'Upload-Offset': String(uploaded.length)});
    uploaded = Buffer.concat([uploaded, body]);
    json(200, {offset: uploaded.length});
  });
}
// The file the test hands the page's picker: 5000 bytes, five chunks, the last one short.
const PICKED = Array.from({length: 5000}, (_, i) => (i * 7 + 3) % 251);

const server = http.createServer((req, res) => {
  if (new URL(req.url, 'http://x').pathname === '/apps/up') return upload(req, res);
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
async function openPage(url, inject) {
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
  if (inject) await send('Page.addScriptToEvaluateOnNewDocument', {source: inject});
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
  const start = Date.now(), until = start + TIMEOUT_MS;
  let settledAt = 0, runningMs = 0;
  for (;;) {
    const s = await page.evaluate(
        'JSON.stringify(window.ariadneWeb ? {state: ariadneWeb.state, log: ariadneWeb.log,' +
        ' text: ariadneWeb.text ? ariadneWeb.text() : "", csp: window.__csp || [],' +
        ' warnings: ariadneWeb.warnings || [], out: window.__out || [],' +
        ' warning: (w => w && !w.hidden ? w.textContent : "")(' +
        '    document.getElementById("warning")),' +
        ' status: document.getElementById("status").textContent} : null)');
    const v = s ? JSON.parse(s) : null;
    // How long the page took to run its document, from the navigation (to within the poll).
    if (v && v.state === 'running' && !runningMs) runningMs = Date.now() - start;
    if (v) v.runningMs = runningMs;
    if (v && check(v)) return v;
    if (v && v.state !== 'loading' && !settledAt) settledAt = Date.now();
    if ((settledAt && Date.now() > settledAt + 15000) || Date.now() > until)
      return v || {state: 'timeout', log: [], text: '', status: '', csp: [], warnings: [], out: []};
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
    name: 'a warning while the document runs shows on the page',
    url: `${ORIGIN}/apps/app/?src=warns.ari`,
    check: v => v.state === 'running' &&
                v.warnings.some(w => /on_tick: resident .* failed: ValueError: live/.test(w)) &&
                /ValueError: live in the page/.test(v.warning),
  },
  {
    name: 'a missing document shows the error page',
    url: `${ORIGIN}/apps/app/?src=nope.ari`,
    check: v => v.state === 'failed' && v.log.some(l => /HTTP 404/.test(l)) &&
                /Could not run/.test(v.status),
  },
];

// The page's file picker, handed a File by the test (a headless page has no one to choose it), and
// what the page writes to the console (Module.print).
const PICK = `window.__out = [];
const log = console.log;
console.log = function(...a) { window.__out.push(a.join(' ')); return log.apply(this, a); };
HTMLInputElement.prototype.click = function() {
  if (this.type !== 'file') return;
  const dt = new DataTransfer();
  dt.items.add(new File([new Uint8Array(${JSON.stringify(PICKED)})], 'photo.bin',
                        {type: 'application/octet-stream'}));
  this.files = dt.files;
  setTimeout(() => this.dispatchEvent(new Event('change')), 50);
};`;
const sent = line => /upload: photo\.bin 5000 of 5000 done true/.test(line);
const uploadedAll = () =>
  uploaded.length === PICKED.length && PICKED.every((b, i) => uploaded[i] === b);
cases.push({
  name: 'a picked file goes to the page\'s origin in chunks',
  url: `${ORIGIN}/apps/cfg/?src=upload.ari`,
  inject: PICK,
  before: () => { uploaded = Buffer.alloc(0); },
  check: v => v.state === 'running' && v.out.some(sent) && uploadedAll(),
});

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
      name: 'ari: a config\'s --net and {origin} in its arguments',
      url: `${ORIGIN}/apps/cfg/ari.html`,
      check: v => v.state === 'running' && shows(v) && v.text.includes(`instance: ${ORIGIN}`),
    },
    {
      name: 'ari: a warning while the document runs, in the header, stderr and the terminal',
      url: `${ORIGIN}/apps/app/ari.html?src=warns.ari`,
      check: v => v.state === 'running' &&
                  v.warnings.some(w => /failed: ValueError: live in the page/.test(w)) &&
                  /^warning: on_tick: resident/.test(v.status) &&
                  v.log.some(l => /^ari: warning: on_tick: resident .* live in the page/.test(l)) &&
                  /ari: warning: on_tick: resident/.test(v.text),
    },
    {
      name: 'ari: a cross-origin src is refused, and the terminal says why',
      url: `${ORIGIN}/apps/app/ari.html?src=` +
           encodeURIComponent(`http://localhost:${PORT}/apps/app/main.ari`),
      check: v => v.state === 'failed' && /not this page's origin/.test(v.text),
    },
    {
      name: 'ari: a picked file goes to the page\'s origin in chunks',
      url: `${ORIGIN}/apps/cfg/ari.html?src=upload.ari`,
      inject: PICK,
      before: () => { uploaded = Buffer.alloc(0); },
      check: v => v.state === 'running' && sent(v.text) && uploadedAll(),
    });
}
// ariglx-web's Python, with --pydocs: what the native ariglx ctests check, in the page.
if (PYDOCS) {
  cases.push(
    {
      name: 'ariglx: a python: block with numpy and pycvc_gl runs at load',
      url: `${ORIGIN}/apps/py/?src=numpy.ari`,
      check: v => v.state === 'running' &&
                  v.log.some(l => /^ariglx-web: numpy \S+ sum 10, poisson \[/.test(l)),
      report: v => v.log.filter(l => /^ariglx-web: numpy /.test(l)),
    },
    {
      name: 'ariglx: xml.parsers.expat and hashlib\'s SHA-2 run in the page',
      url: `${ORIGIN}/apps/py/?src=stdlib.ari`,
      check: v => v.state === 'running' && v.log.some(l =>
                    /^ariglx-web: expat a,b,c, sha256 ba7816bf8f01cfea, seeded draw \d+$/.test(l)),
      report: v => v.log.filter(l => /^ariglx-web: expat /.test(l)),
    },
    {
      name: 'ariglx: a Python program in place of the document',
      url: `${ORIGIN}/apps/pyprog/`,
      check: v => v.state === 'running' &&
                  v.log.some(l => /^program\.py: run\(\) returned 0$/.test(l)) &&
                  v.log.some(l => /^program\.py: result 14, frames 3$/.test(l)),
    },
    {
      name: 'ariglx: a python: block that raises fails the load with its traceback',
      url: `${ORIGIN}/apps/py/?src=python_error.ari`,
      check: v => v.state === 'failed' && v.log.some(l => /ZeroDivisionError/.test(l)) &&
                  /Could not run/.test(v.status),
    });
  // The full tier: vtk-python linked in, its vtkmodules in the zip (a stored zip names its files
  // in the clear).
  if (fs.readFileSync(path.join(WEB, 'python312.zip')).includes('vtkmodules/__init__.py'))
    cases.push({
      name: 'ariglx (full tier): a python: block runs a VTK pipeline through vtkmodules',
      url: `${ORIGIN}/apps/py/?src=vtk.ari`,
      check: v => v.state === 'running' &&
                  v.log.some(l => /^ariglx-web: vtkmodules vtkObject, sphere points \d+$/.test(l)),
      report: v => v.log.filter(l => /^ariglx-web: vtkmodules /.test(l)),
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
  if (c.before) c.before();
  const page = await openPage(c.url, c.inject);
  const v = await settle(page, c.check);
  const ok = !!(v && c.check(v) && v.csp.length === 0);
  if (!ok) failures++;
  const took = v && v.runningMs ? `, running after ${(v.runningMs / 1000).toFixed(1)} s` : '';
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${c.name}  [${v ? v.state : 'no state'}${took}]`);
  if (c.report && v) for (const l of c.report(v)) console.log('  ' + l);
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

// Close Chrome over its browser DevTools socket, which stops every process it started. A SIGTERM
// to the main process alone leaves its helpers writing the profile after it has exited, so
// removing the profile failed with ENOTEMPTY (in CI, and here). A profile left behind in a temp
// directory never fails the run.
async function closeChrome() {
  if (chrome.exitCode !== null || chrome.signalCode !== null) return;
  const exited = new Promise(r => chrome.once('exit', () => r('exited')));
  const after = ms => new Promise(r => setTimeout(() => r('timeout'), ms));
  try {
    const ws = new WebSocket(devtools.href);
    await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
    ws.send(JSON.stringify({id: 1, method: 'Browser.close'}));
  } catch (e) {
    chrome.kill();
  }
  if (await Promise.race([exited, after(10000)]) === 'timeout') {
    chrome.kill('SIGKILL');
    await Promise.race([exited, after(5000)]);
  }
}
await closeChrome();
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
