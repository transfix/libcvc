// arigl_web_mac_keys.mjs — Cmd as the shortcut key in arigl-web on a Mac (docs/ARIADNE.md §5.28),
// in a real (headless) Chrome told it runs on one: DevTools sets navigator.platform and
// navigator.userAgentData.platform as a Mac's browser reports them. The keys go in as DevTools
// dispatches them, Cmd being the Meta key.
//
//   node arigl_web_mac_keys.mjs --web <share/arigl-web> --chrome <chrome or chromium> [--isolated]
//
// It serves the package with a document of its own (a field in a window) on 127.0.0.1, and:
//
//   1. on a Mac, cvcGL says Cmd is the shortcut key; Cmd+Z undoes in the field and Ctrl+Z does not;
//   2. a Mac sends no key-up for a key let go while Cmd is down (Chrome and Safari): with Cmd
//      held after Cmd+Z and no key-up for Z, it undoes once, and a second Z under Cmd once more;
//   3. Cmd+V's keydown is left to the browser, so its paste event comes, and the field takes what
//      the clipboard holds;
//   4. with ?mackeys=0 on the same Mac, Ctrl is the shortcut key again: Ctrl+Z undoes, Cmd+Z does
//      not.
//
// The field is found as arigl_web_clipboard.mjs finds it: a right-click walks the pointer down its
// menu until the page's clipboard glue says Paste is under it (window.__cvcglClip.arm). What the
// document prints (Module.print) is captured from the console. Exit status 0 when every step
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
  console.error('usage: arigl_web_mac_keys.mjs --web DIR --chrome PATH [--isolated]');
  process.exit(2);
}
const TIMEOUT_MS = Number(process.env.ARIGL_WEB_SMOKE_TIMEOUT_MS || 180000);

// The document: a field whose commit prints its value, in a window at a known place.
const DOC = `meta:
  name: mac_keys
windows:
  - window: Keys
    pos: [40, 40]
    size: [460, 220]
    children:
      - input_text: Name
        bind: name
        on: (print (str-concat "name=" (state-get "name")))
`;
const TYPES = {'.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css',
               '.wasm': 'application/wasm', '.json': 'application/json'};
const ISOLATION = ISOLATED ? {'Cross-Origin-Opener-Policy': 'same-origin',
                              'Cross-Origin-Embedder-Policy': 'require-corp',
                              'Cross-Origin-Resource-Policy': 'same-origin'} : {};
const server = http.createServer((req, res) => {
  const p = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  if (p === '/keys/main.ari') {
    res.writeHead(200, {'Content-Type': 'text/plain', 'Cache-Control': 'no-cache', ...ISOLATION});
    res.end(DOC);
    return;
  }
  const name = p === '/keys/' ? 'index.html' : p.startsWith('/keys/') ? p.slice(6) : null;
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

const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'arigl-web-mac-keys-'));
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
const granted = await browser.send('Browser.grantPermissions',
                                   {origin: ORIGIN,
                                    permissions: ['clipboardReadWrite', 'clipboardSanitizedWrite']});
if (granted.error) throw new Error('grantPermissions: ' + JSON.stringify(granted.error));

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
await page.send('Page.bringToFront');
await page.send('Emulation.setFocusEmulationEnabled', {enabled: true});
// A Mac's Chrome, as the page sees it: navigator.platform and navigator.userAgentData.platform.
const ua = (await browser.send('Browser.getVersion')).result.userAgent
               .replace(/\(X11; Linux [^)]*\)/, '(Macintosh; Intel Mac OS X 10_15_7)');
const emulated = await page.send('Emulation.setUserAgentOverride', {
  userAgent: ua, platform: 'MacIntel',
  userAgentMetadata: {platform: 'macOS', platformVersion: '15.0.0', architecture: 'arm',
                      model: '', mobile: false}});
if (emulated.error) throw new Error('setUserAgentOverride: ' + JSON.stringify(emulated.error));
// The console, and each paste event and whether each V keydown was cancelled.
await page.send('Page.addScriptToEvaluateOnNewDocument', {source: `(() => {
window.__out = [];
window.__pastes = 0;
window.__vDown = [];
const log = console.log;
console.log = function(...a) { window.__out.push(a.join(' ')); return log.apply(this, a); };
document.addEventListener('paste', () => { ++window.__pastes; });
window.addEventListener('keydown', e => {
  if (e.code === 'KeyV') window.__vDown.push({meta: e.metaKey, ctrl: e.ctrlKey,
                                              cancelled: e.defaultPrevented});
});
})();`});

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
// The modifiers DevTools takes: Alt 1, Ctrl 2, Meta (Cmd) 4, Shift 8.
const CTRL = 2, CMD = 4;
const MODS = {[CTRL]: {key: 'Control', code: 'ControlLeft', vk: 17},
              [CMD]: {key: 'Meta', code: 'MetaLeft', vk: 91}};
const down = (k, code, vk, modifiers = 0, extra = {}) =>
  page.send('Input.dispatchKeyEvent', {type: 'rawKeyDown', key: k, code, modifiers,
                                       windowsVirtualKeyCode: vk, nativeVirtualKeyCode: vk,
                                       ...extra});
const up = (k, code, vk, modifiers = 0) =>
  page.send('Input.dispatchKeyEvent', {type: 'keyUp', key: k, code, modifiers,
                                       windowsVirtualKeyCode: vk, nativeVirtualKeyCode: vk});
const key = async (k, code, vk, modifiers = 0) => {
  await down(k, code, vk, modifiers);
  await sleep(60);
  await up(k, code, vk, modifiers);
  await sleep(250);
};
const enter = () => key('Enter', 'Enter', 13);
// Typed text: a keydown with its text, which the browser follows with its keypress.
const type = async text => {
  for (const c of text) {
    const vk = c.toUpperCase().charCodeAt(0);
    await page.send('Input.dispatchKeyEvent', {type: 'keyDown', key: c, code: 'Key' + c.toUpperCase(),
                                               text: c, windowsVirtualKeyCode: vk});
    await sleep(40);
    await up(c, 'Key' + c.toUpperCase(), vk);
    await sleep(120);
  }
};
// MOD (CTRL or CMD) held over the letter C. EXTRA goes with the letter's keydown (DevTools'
// editing commands: a paste).
const chord = async (mod, c, extra = {}) => {
  const m = MODS[mod], vk = c.toUpperCase().charCodeAt(0), code = 'Key' + c.toUpperCase();
  await down(m.key, m.code, m.vk, mod);
  await sleep(60);
  await down(c, code, vk, mod, extra);
  await sleep(120);
  await up(c, code, vk, mod);
  await sleep(60);
  await up(m.key, m.code, m.vk);
  await sleep(250);
};
const armed = () => evaluate('JSON.stringify(window.__cvcglClip ? window.__cvcglClip.arm : null)')
                        .then(s => JSON.parse(s || 'null'));
const printed = () => evaluate('JSON.stringify(window.__out)')
                          .then(s => JSON.parse(s).map(l => l.replace(/^"(.*)"$/, '$1')));
// What the field holds: Enter commits it, and the document prints it.
const committed = async () => {
  const n = (await printed()).length;
  await enter();
  const line = await until('the field commits', async () =>
    (await printed()).slice(n).find(l => l.startsWith('name=')), 10000).catch(() => null);
  return line === null ? null : line.slice(5);
};

// The field: the first row whose right-click menu has Paste. Its menu is closed again, and a click
// there gives the field the keyboard (Enter takes it away, and ImGui's undo with it).
const X = 60;
let row = 0;
const refocus = () => click(X + 120, row);
async function focusField() {
  for (let y = 62; y < 220; y += 8) {
    await click(X, y, 'right');
    let paste = false;
    for (let dy = 4; dy < 160 && !paste; dy += 3) {
      await move(X + 24, y + dy);
      await sleep(60);
      const a = await armed();
      paste = !!(a && a.paste === true);
    }
    await key('Escape', 'Escape', 27);
    if (paste) {
      row = y;
      await refocus();
      return true;
    }
  }
  return false;
}

async function open(query) {
  await page.send('Page.navigate', {url: `${ORIGIN}/keys/?src=main.ari${query}`});
  await until('the page runs its document', async () =>
    (await evaluate('window.ariadneWeb && ariadneWeb.state')) === 'running');
  await sleep(1500);
  return focusField();
}
const said = async () =>
  (await printed()).find(l => l.startsWith('cvcGL: ') && l.includes('shortcut key'));

try {
  // 1. A Mac: Cmd is the shortcut key.
  step(await open(''), 'the field takes the keyboard');
  const mac = await said();
  step(/Cmd is the shortcut key/.test(mac || ''), `cvcGL says Cmd: ${mac}`);
  await type('abc');
  await chord(CMD, 'z');
  let got = await committed();
  step(got === 'ab', `Cmd+Z undoes the last character: ${got}`);
  await refocus();
  await type('d');
  await chord(CTRL, 'z');
  got = await committed();
  step(got === 'abd', `Ctrl+Z, ImGui's Super on a Mac, does not: ${got}`);

  // 2. No key-up for Z while Cmd is down: under Cmd a press is a tap. Cmd held a second and a half
  // after Cmd+Z undoes once (with Z left down ImGui repeated the undo until the field's ran out:
  // "abd"), and a second Z while Cmd is still down undoes again.
  await refocus();
  await type('efg');
  await down('Meta', 'MetaLeft', 91, CMD);
  await sleep(60);
  await down('z', 'KeyZ', 90, CMD);
  await sleep(1500);
  await down('z', 'KeyZ', 90, CMD);
  await sleep(300);
  await up('Meta', 'MetaLeft', 91);
  await sleep(250);
  got = await committed();
  step(got === 'abde', `with no key-up for Z, Cmd held after Cmd+Z undoes once, and a second Z ` +
                       `once more: ${got}`);

  // 3. Cmd+V: the keydown is not cancelled, so the browser pastes (DevTools' paste command stands
  // in for the Mac's key binding), and the field takes the paste event's text.
  await refocus();
  await evaluate('navigator.clipboard.writeText("XY")', true);
  const pastes = await evaluate('window.__pastes');
  await chord(CMD, 'v', {commands: ['paste']});
  const v = await evaluate('JSON.stringify(window.__vDown.at(-1) || null)').then(s => JSON.parse(s));
  step(!!v && v.meta && !v.cancelled, `Cmd+V's keydown is left to the browser: ${JSON.stringify(v)}`);
  step((await evaluate('window.__pastes')) > pastes, 'and its paste event comes');
  got = await committed();
  step(got === 'abdeXY', `which the field takes: ${got}`);

  // 4. ?mackeys=0 on the same Mac: Ctrl is the shortcut key.
  step(await open('&mackeys=0'), 'with ?mackeys=0 the field takes the keyboard');
  const ctrl = await said();
  step(/Ctrl is the shortcut key.*overruled by \?mackeys/.test(ctrl || ''), `cvcGL says Ctrl: ${ctrl}`);
  await type('abc');
  await chord(CMD, 'z');
  got = await committed();
  step(got === 'abc', `Cmd+Z does not undo there: ${got}`);
  await refocus();
  await type('d');
  await chord(CTRL, 'z');
  got = await committed();
  step(got === 'abc', `Ctrl+Z does: ${got}`);
} catch (e) {
  step(false, e.message);
}

page.ws.close();
browser.ws.close();
const exited = new Promise(r => chrome.once('exit', r));
chrome.kill();
await Promise.race([exited, sleep(5000)]);
server.close();
if (!argv.includes('--keep')) {
  try {
    fs.rmSync(profile, {recursive: true, force: true, maxRetries: 10, retryDelay: 300});
  } catch (e) {
    console.warn('could not remove the profile ' + profile + ': ' + e.message);
  }
}
console.log(failures ? `FAILED (${failures})` : 'passed');
process.exit(failures ? 1 : 0);
