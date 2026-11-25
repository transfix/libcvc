// arigl_web_save.mjs — saving a picture or a video (docs/ARIADNE.md §5.29) from arigl-web in a real
// (headless) Chrome, through the page's own input: the mouse over the canvas and the keys, as
// DevTools dispatches them.
//
//   node arigl_web_save.mjs --web <share/arigl-web> --chrome <chrome or chromium> [--isolated]
//
// It serves the package with a document of its own (a picture in a window, and a button whose
// action is (save-url ...)) on 127.0.0.1, with the page's origin allowed (--net-allow), and lets
// the browser download into a folder of its own (Browser.setDownloadBehavior). Then:
//
//   1. right-clicks the picture, and its menu's Save image... downloads it, as cat.png (the name
//      from the URL, the extension from the bytes), with the bytes the server holds. The download
//      starts inside the pointer's release over the item (Safari counts only that as the user's);
//   2. the button's (save-url URL (dict "name" ...)) downloads another file under the given name;
//   3. when the package has ari's terminal page (ari.html), the same document there: Tab to the
//      button and Enter, and the same download.
//
// The menu's item is found without knowing where ImGui draws it: the page's save glue is told
// which Save item the pointer is over (Module.cvcSave.arm), so the pointer walks down from where
// the menu opened until one is armed. A press and a release there is then a real click, which the
// page's pointer listener sees. --isolated serves the page cross-origin isolated, for a threaded
// (wasm-mt) package. Exit status 0 when every step passes.

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
const WEB = opt('--web'), CHROME = opt('--chrome');
const ISOLATED = argv.includes('--isolated');
if (!WEB || !CHROME) {
  console.error('usage: arigl_web_save.mjs --web DIR --chrome PATH [--isolated]');
  process.exit(2);
}
const TIMEOUT_MS = Number(process.env.ARIGL_WEB_SMOKE_TIMEOUT_MS || 180000);

// What the server holds: a picture with no extension in its URL (as txwtf's uploads are), and a
// WebM clip. Only their first bytes matter to a save: they say what the file is.
const PNG = Buffer.concat([Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
                           Buffer.from('the picture, as far as a save knows')]);
const WEBM = Buffer.concat([Buffer.from([0x1a, 0x45, 0xdf, 0xa3, 0x9f, 0x42, 0x86, 0x81, 0x01,
                                         0x42, 0x82, 0x84]),
                            Buffer.from('webm and the clip after it')]);

// The document: the picture in a window at a known place, and the button in another.
const doc = origin => `meta:
  name: save
windows:
  - window: Pic
    pos: [40, 40]
    size: [300, 200]
    children:
      - image: cat
        src: ${origin}/save/media/cat
        size: [160, 90]
  - window: Btn
    pos: [40, 300]
    size: [240, 90]
    children:
      - button: Save the clip
        on: (print (get-attr (save-url "${origin}/save/media/clip" (dict "name" "the clip")) "name"))
`;
const TYPES = {'.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css',
               '.wasm': 'application/wasm', '.json': 'application/json'};
const ISOLATION = ISOLATED ? {'Cross-Origin-Opener-Policy': 'same-origin',
                              'Cross-Origin-Embedder-Policy': 'require-corp',
                              'Cross-Origin-Resource-Policy': 'same-origin'} : {};
let ORIGIN = '';
const server = http.createServer((req, res) => {
  const p = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  const send = (type, body) => {
    res.writeHead(200, {'Content-Type': type, 'Cache-Control': 'no-cache', ...ISOLATION});
    res.end(body);
  };
  if (p === '/save/main.ari')
    return send('text/plain', doc(ORIGIN));
  if (p === '/save/ariadne.config.json')
    return send('application/json', JSON.stringify({flags: ['--net-allow', '{origin}']}));
  if (p === '/save/media/cat')
    return send('application/octet-stream', PNG);
  if (p === '/save/media/clip')
    return send('application/octet-stream', WEBM);
  const name = p === '/save/' ? 'index.html' : p.startsWith('/save/') ? p.slice(6) : null;
  const f = name && !name.includes('..') ? path.join(WEB, name) : null;
  if (!f || !fs.existsSync(f) || !fs.statSync(f).isFile()) {
    res.writeHead(404, {'Content-Type': 'text/plain', ...ISOLATION});
    res.end('not found');
    return;
  }
  res.writeHead(200, {'Content-Type': TYPES[path.extname(f)] || 'application/octet-stream',
                      'Cache-Control': 'no-cache', ...ISOLATION});
  fs.createReadStream(f).pipe(res);
});
await new Promise(r => server.listen(0, '127.0.0.1', r));
ORIGIN = `http://127.0.0.1:${server.address().port}`;

const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'arigl-web-save-'));
const downloads = fs.mkdtempSync(path.join(os.tmpdir(), 'arigl-web-save-dl-'));
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

// The browser's own socket (downloads), and the page's; each keeps the events it hears.
async function connect(wsUrl) {
  const ws = new WebSocket(wsUrl);
  await new Promise((res, rej) => { ws.onopen = res; ws.onerror = rej; });
  let id = 0;
  const pending = new Map();
  const events = [];
  ws.onmessage = e => {
    const m = JSON.parse(e.data);
    if (m.id && pending.has(m.id)) { pending.get(m.id)(m); pending.delete(m.id); }
    else if (m.method) events.push(m);
  };
  const send = (method, params = {}) => new Promise(res => {
    pending.set(++id, res);
    ws.send(JSON.stringify({id, method, params}));
  });
  return {ws, send, events};
}
const browser = await connect(devtools.href);
const allowed = await browser.send('Browser.setDownloadBehavior',
                                   {behavior: 'allowAndName', downloadPath: downloads,
                                    eventsEnabled: true});
if (allowed.error) throw new Error('setDownloadBehavior: ' + JSON.stringify(allowed.error));

const r = await fetch(`http://${devtools.host}/json/new?about:blank`, {method: 'PUT'});
const target = await r.json();
const page = await connect(target.webSocketDebuggerUrl);
const evaluate = async expr => {
  const m = await page.send('Runtime.evaluate', {expression: expr, returnByValue: true,
                                                 awaitPromise: true});
  if (m.result && m.result.exceptionDetails)
    throw new Error(expr + ': ' + JSON.stringify(m.result.exceptionDetails));
  return m.result && m.result.result ? m.result.result.value : undefined;
};
await page.send('Page.enable');
await page.send('Page.bringToFront');
await page.send('Emulation.setFocusEmulationEnabled', {enabled: true});
// What the document prints, and, for each download link the page clicks, whether that was inside
// a pointer release's dispatch: this listener runs first (it is the first on the window), and the
// flag is cleared once the dispatch is over.
await page.send('Page.addScriptToEvaluateOnNewDocument', {source: `(() => {
window.__out = [];
const log = console.log;
console.log = function(...a) { window.__out.push(a.join(' ')); return log.apply(this, a); };
window.__inRelease = false;
window.__links = [];
window.addEventListener('pointerup', () => {
  window.__inRelease = true;
  setTimeout(() => { window.__inRelease = false; }, 0);
}, true);
const click = HTMLAnchorElement.prototype.click;
HTMLAnchorElement.prototype.click = function() {
  if (this.download) window.__links.push({name: this.download, inRelease: window.__inRelease});
  return click.apply(this, arguments);
};
})();`});
await page.send('Page.navigate', {url: `${ORIGIN}/save/?src=main.ari`});

const sleep = ms => new Promise(r => setTimeout(r, ms));
const until = async (what, check, ms = TIMEOUT_MS) => {
  const end = Date.now() + ms;
  for (;;) {
    const v = await check();
    if (v) return v;
    if (Date.now() > end) throw new Error('timed out: ' + what);
    await sleep(200);
  }
};
let failures = 0;
const step = (ok, what) => {
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${what}`);
  if (!ok) ++failures;
};

// The page's input, in CSS px over the page (the canvas fills it).
const mouse = (type, x, y, button = 'none', clickCount = 0) =>
  page.send('Input.dispatchMouseEvent', {type, x, y, button, clickCount,
                                         buttons: button === 'left' ? 1 : button === 'right' ? 2 : 0});
const move = async (x, y) => { await mouse('mouseMoved', x, y); await sleep(60); };
const click = async (x, y, button = 'left') => {
  await move(x, y);
  await mouse('mousePressed', x, y, button, 1);
  await sleep(80);
  await mouse('mouseReleased', x, y, button, 1);
  await sleep(250);
};
const key = async (k, code, vk, modifiers = 0) => {
  await page.send('Input.dispatchKeyEvent', {type: 'rawKeyDown', key: k, code, modifiers,
                                             windowsVirtualKeyCode: vk, nativeVirtualKeyCode: vk});
  await sleep(60);
  await page.send('Input.dispatchKeyEvent', {type: 'keyUp', key: k, code, modifiers,
                                             windowsVirtualKeyCode: vk, nativeVirtualKeyCode: vk});
  await sleep(250);
};
const armed = () => evaluate('Module && Module.cvcSave ? Module.cvcSave.arm : null');

// The download the browser starts next (after `from` events): its suggested name and, once it
// completed, the bytes it wrote.
async function downloaded(from, ms = 30000) {
  const begin = await until('a download', async () =>
    browser.events.slice(from).find(e => e.method === 'Browser.downloadWillBegin'), ms)
                    .catch(() => null);
  if (!begin) return null;
  const guid = begin.params.guid;
  const done = await until('the download completes', async () =>
    browser.events.find(e => e.method === 'Browser.downloadProgress' && e.params.guid === guid &&
                             e.params.state !== 'inProgress'), ms).catch(() => null);
  const file = path.join(downloads, guid);
  return {name: begin.params.suggestedFilename,
          state: done ? done.params.state : 'unfinished',
          bytes: fs.existsSync(file) ? fs.readFileSync(file) : null};
}

try {
  await until('the page runs its document', async () =>
    (await evaluate('window.ariadneWeb && ariadneWeb.state')) === 'running');
  await sleep(1500);

  // 1. The picture's Save image..., inside the pointer's release over it.
  let at = null;
  for (const [x, y] of [[90, 100], [120, 110], [70, 90], [150, 120]]) {
    await click(x, y, 'right');
    for (let dy = 4; dy < 90 && !at; dy += 3) {
      await move(x + 24, y + dy);
      await sleep(60); // a frame or two: the item under the pointer arms at the frame's end
      if (await armed()) at = {x: x + 24, y: y + dy};
    }
    if (at) break;
    await key('Escape', 'Escape', 27);
  }
  step(!!at, 'a right-click on the picture opens a menu with Save image...');
  if (at) {
    const from = browser.events.length;
    await click(at.x, at.y);
    const d = await downloaded(from);
    step(d && d.name === 'cat.png', `it downloads the picture as cat.png: ${d && d.name}`);
    step(d && d.state === 'completed' && d.bytes && d.bytes.equals(PNG),
         `with the server's bytes: ${d && d.state}`);
    const links = JSON.parse(await evaluate('JSON.stringify(window.__links)'));
    step(links.length > 0 && links[0].inRelease,
         `inside the pointer's release: ${JSON.stringify(links)}`);
  }

  // 2. The button's (save-url URL (dict "name" ...)), found by clicking down the window's first
  // lines until a download begins (a click beside it does nothing).
  {
    const from = browser.events.length;
    let d = null;
    for (let y = 318; y < 390 && !d; y += 6) {
      await click(90, y);
      d = await downloaded(from, 2500);
    }
    step(d && d.name === 'the clip.webm', `(save-url) downloads under its name: ${d && d.name}`);
    step(d && d.bytes && d.bytes.equals(WEBM), 'with the server\'s bytes');
    const said = await until('the program hears', async () =>
      JSON.parse(await evaluate('JSON.stringify(window.__out)'))
          .find(l => /the clip/.test(l)), 10000).catch(() => null);
    step(!!said, `the program gets the name back: ${said}`);
  }

  // 3. ari's terminal page (FTXUI in xterm.js), when the package has it: Tab to the button, Enter.
  if (fs.existsSync(path.join(WEB, 'ari.html'))) {
    await page.send('Page.navigate', {url: `${ORIGIN}/save/ari.html?src=main.ari`});
    await until('ari\'s page runs the document', async () =>
      (await evaluate('window.ariadneWeb && ariadneWeb.state')) === 'running' &&
      /Save the clip/.test(await evaluate('ariadneWeb.text()')));
    await sleep(1000);
    const from = browser.events.length;
    await key('Tab', 'Tab', 9);
    await key('Enter', 'Enter', 13);
    const d = await downloaded(from);
    step(d && d.name === 'the clip.webm' && d.bytes && d.bytes.equals(WEBM),
         `the terminal page downloads it too: ${d && d.name}`);
  }
} catch (e) {
  step(false, e.message);
}

page.ws.close();
browser.ws.close();
const exited = new Promise(r => chrome.once('exit', r));
chrome.kill();
await Promise.race([exited, sleep(5000)]); // Chrome writes its profile until it has gone
server.close();
if (!argv.includes('--keep')) {
  for (const dir of [profile, downloads]) {
    try {
      fs.rmSync(dir, {recursive: true, force: true, maxRetries: 10, retryDelay: 300});
    } catch (e) {
      console.warn('could not remove ' + dir + ': ' + e.message);
    }
  }
}
console.log(failures ? `FAILED (${failures})` : 'passed');
process.exit(failures ? 1 : 0);
