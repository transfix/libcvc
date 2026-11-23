// arigl_web_clipboard.mjs — the context menu's Copy, Cut and Paste (docs/ARIADNE.md §5.28) in
// arigl-web in a real (headless) Chrome, through the page's own input: the mouse over the canvas
// and the keys, as DevTools dispatches them.
//
//   node arigl_web_clipboard.mjs --web <share/arigl-web> --chrome <chrome or chromium> [--isolated]
//
// It serves the package with a document of its own (a field and a text in a window) on 127.0.0.1,
// grants the page the clipboard (Browser.grantPermissions, as a user's "Allow" does), and then:
//
//   1. right-clicks the text, and its menu's Copy puts the text on the clipboard;
//   2. right-clicks the field, and its menu's Paste reads the clipboard
//      (navigator.clipboard.readText) and puts what it read in the field, which Enter commits;
//   3. its Cut takes the field's text to the clipboard and out of the field;
//   4. with the read refused (the permission denied), Paste puts the focus in the field, and a
//      paste the browser delivers (Ctrl+V's paste event) still lands there.
//
// The menu's items are found without knowing where ImGui draws them: the page's clipboard glue
// is told which item the pointer is over (cvc::gl::clipboard::arm: window.__cvcglClip.arm), so
// the pointer walks down from where the menu opened until the item it wants is armed. A press and
// a release there is then a real click, which the page's pointer listener sees, as Safari needs.
// What the document prints (Module.print) is captured from the console. --isolated serves the
// page cross-origin isolated, for a threaded (wasm-mt) package. Exit status 0 when every step
// passes.

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
  console.error('usage: arigl_web_clipboard.mjs --web DIR --chrome PATH [--isolated]');
  process.exit(2);
}
const TIMEOUT_MS = Number(process.env.ARIGL_WEB_SMOKE_TIMEOUT_MS || 180000);

// The document: a field whose commit prints its value, and a text, in a window at a known place.
const DOC = `meta:
  name: clipboard
windows:
  - window: Clip
    pos: [40, 40]
    size: [460, 220]
    children:
      - input_text: Name
        bind: name
        on: (print (str-concat "name=" (state-get "name")))
      - text: copy me
`;
const TYPES = {'.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css',
               '.wasm': 'application/wasm', '.json': 'application/json'};
const ISOLATION = ISOLATED ? {'Cross-Origin-Opener-Policy': 'same-origin',
                              'Cross-Origin-Embedder-Policy': 'require-corp',
                              'Cross-Origin-Resource-Policy': 'same-origin'} : {};
const server = http.createServer((req, res) => {
  const p = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  if (p === '/clip/main.ari') {
    res.writeHead(200, {'Content-Type': 'text/plain', 'Cache-Control': 'no-cache', ...ISOLATION});
    res.end(DOC);
    return;
  }
  const name = p === '/clip/' ? 'index.html' : p.startsWith('/clip/') ? p.slice(6) : null;
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
const ORIGIN = `http://127.0.0.1:${server.address().port}`;

const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'arigl-web-clipboard-'));
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

// The browser's own socket (permissions), and the page's.
async function connect(wsUrl) {
  const ws = new WebSocket(wsUrl);
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
  return {ws, send};
}
const browser = await connect(devtools.href);
const grant = async on => {
  const permissions = on ? ['clipboardReadWrite', 'clipboardSanitizedWrite']
                         : ['clipboardSanitizedWrite'];
  if (!on) await browser.send('Browser.resetPermissions', {});
  const m = await browser.send('Browser.grantPermissions', {origin: ORIGIN, permissions});
  if (m.error) throw new Error('grantPermissions: ' + JSON.stringify(m.error));
  if (!on)
    await browser.send('Browser.setPermission',
                       {origin: ORIGIN, permission: {name: 'clipboard-read'}, setting: 'denied'});
};

const r = await fetch(`http://${devtools.host}/json/new?about:blank`, {method: 'PUT'});
const target = await r.json();
const page = await connect(target.webSocketDebuggerUrl);
const evaluate = async (expr, gesture = false) => {
  const m = await page.send('Runtime.evaluate', {expression: expr, returnByValue: true,
                                                 awaitPromise: true, userGesture: gesture});
  if (m.result && m.result.exceptionDetails)
    throw new Error(expr + ': ' + JSON.stringify(m.result.exceptionDetails));
  return m.result && m.result.result ? m.result.result.value : undefined;
};
await page.send('Page.enable');
await page.send('Page.addScriptToEvaluateOnNewDocument', {source: `(() => {
window.__out = [];
const log = console.log;
console.log = function(...a) { window.__out.push(a.join(' ')); return log.apply(this, a); };
})();`});
await grant(true);
await page.send('Page.navigate', {url: `${ORIGIN}/clip/?src=main.ari`});

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
const key = async (k, code, vk, modifiers = 0, commands = []) => {
  await page.send('Input.dispatchKeyEvent', {type: 'rawKeyDown', key: k, code, modifiers, commands,
                                             windowsVirtualKeyCode: vk, nativeVirtualKeyCode: vk});
  await sleep(60);
  await page.send('Input.dispatchKeyEvent', {type: 'keyUp', key: k, code, modifiers,
                                             windowsVirtualKeyCode: vk, nativeVirtualKeyCode: vk});
  await sleep(250);
};
const enter = () => key('Enter', 'Enter', 13);
const armed = () => evaluate('JSON.stringify(window.__cvcglClip ? window.__cvcglClip.arm : null)')
                        .then(s => JSON.parse(s || 'null'));
const printed = () => evaluate('JSON.stringify(window.__out)').then(s => JSON.parse(s));
const clipboard = () => evaluate('navigator.clipboard.readText()', true);

// Right-click at (x, y), then walk the pointer down the menu that opened there until WANT(arm)
// holds for the item under it: where to click it, or null when no menu item there does.
async function menuItem(x, y, want) {
  await click(x, y, 'right');
  for (let dy = 4; dy < 160; dy += 3) {
    await move(x + 24, y + dy);
    await sleep(60); // a frame or two: the item under the pointer arms in the frame's end
    const a = await armed();
    if (a && want(a)) return {x: x + 24, y: y + dy};
  }
  await key('Escape', 'Escape', 27);
  return null;
}
// The first of the candidate rows (the window's first lines) whose menu has an item WANT takes.
async function find(want) {
  for (let y = 62; y < 200; y += 8) {
    const at = await menuItem(120, y, want);
    if (at) return {row: y, at};
  }
  return null;
}

try {
  await until('the page runs its document', async () =>
    (await evaluate('window.ariadneWeb && ariadneWeb.state')) === 'running');
  await sleep(1500);

  // 1. The text's Copy.
  await evaluate('navigator.clipboard.writeText("before")', true);
  const text = await find(a => a.copy === 'copy me');
  step(!!text, 'a right-click on the text opens a menu whose Copy copies it');
  if (text) {
    await click(text.at.x, text.at.y);
    step((await clipboard()) === 'copy me', 'Copy puts the text on the clipboard');
  }

  // 2. The field's Paste, through navigator.clipboard.readText().
  await evaluate('navigator.clipboard.writeText("from the clipboard")', true);
  const field = await find(a => a.paste === true);
  step(!!field, 'a right-click on the field opens a menu with Paste');
  if (field) {
    await click(field.at.x, field.at.y);
    await sleep(1000);
    await enter();
    const out = await until('the field commits', async () =>
      (await printed()).find(l => l.startsWith('name=')), 10000).catch(() => null);
    step(out === 'name=from the clipboard', `Paste puts what the clipboard holds in it: ${out}`);

    // 3. Its Cut: the whole text (nothing selected) to the clipboard, and out of the field.
    const cut = await menuItem(120, field.row, a => a.copy === 'from the clipboard');
    step(!!cut, 'its Cut copies the field\'s text');
    if (cut) {
      await click(cut.x, cut.y);
      await sleep(500);
      await enter();
      await sleep(500);
      const lines = await printed();
      step(lines[lines.length - 1] === 'name=', `and takes it out of the field: ${lines.at(-1)}`);
      step((await clipboard()) === 'from the clipboard', 'Cut puts it on the clipboard');
    }

    // 4. The read refused: the focus goes to the field, and the browser's own paste lands there.
    await evaluate('navigator.clipboard.writeText("typed paste")', true);
    await grant(false);
    const refused = await menuItem(120, field.row, a => a.paste === true);
    if (refused) {
      await click(refused.x, refused.y);
      await sleep(1000);
      // Ctrl+V as the browser does it: the key, and its paste event with the clipboard's text.
      await evaluate(`(() => {
        const dt = new DataTransfer();
        dt.setData('text/plain', 'typed paste');
        document.dispatchEvent(new ClipboardEvent('paste', {clipboardData: dt}));
      })()`);
      await key('v', 'KeyV', 86, 2);
      await enter();
      const lines = await printed();
      step(lines.at(-1) === 'name=typed paste',
           `a refused read leaves the field focused for Ctrl+V: ${lines.at(-1)}`);
    } else {
      step(false, 'the field\'s menu again');
    }
  }
} catch (e) {
  step(false, e.message);
}

page.ws.close();
browser.ws.close();
chrome.kill();
server.close();
if (!argv.includes('--keep')) fs.rmSync(profile, {recursive: true, force: true});
console.log(failures ? `FAILED (${failures})` : 'passed');
process.exit(failures ? 1 : 0);
