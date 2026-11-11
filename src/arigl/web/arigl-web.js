// arigl-web.js — the arigl-web page's loader (arigl-web.html, which documents the config): read
// the config, resolve ?src= against the page, then start arigl.js on arigl.wasm with the command
// line `arigl run <config flags> <src URL> <document args>`. arigl fetches the document and the
// files it imports (cvc/ariadne/web_document.h) and calls back: Module.ariadneRunning when the
// document is loaded, Module.ariadneWarning(text) for each warning while it runs (shown in the
// corner for a few seconds; a click dismisses it), Module.ariadneExit(status) when it stops. A
// failure shows the error screen with everything arigl printed.
//
// The script tag may name another build of the shell: data-module="ariglx" starts ariglx.js on
// ariglx.wasm (ariglx-web, arigl with an embedded Python), and data-python="python312.zip" fetches
// that zip with it and writes it to /py/lib/python312.zip, the interpreter's stdlib and pure-Python
// packages, before the module starts.
(function () {
  'use strict';
  const tag = document.currentScript;
  const MODULE = (tag && tag.dataset.module) || 'arigl';
  const PYTHON_ZIP = (tag && tag.dataset.python) || '';
  const overlay = document.getElementById('overlay');
  const statusEl = document.getElementById('status');
  const detailsEl = document.getElementById('details');
  const canvas = document.getElementById('canvas');
  canvas.addEventListener('click', () => canvas.focus());
  // Right-click belongs to the document (Ariadne's context menus): keep the browser's menu off it.
  canvas.addEventListener('contextmenu', e => e.preventDefault());

  const stderrTail = []; // what arigl said, for the error page
  let failed = false;
  let srcShown = '';
  // For a test or a console: loading, running, ended or failed, what arigl printed, and the
  // warnings while the document ran.
  const warnings = [];
  const state = window.ariadneWeb = {state: 'loading', log: stderrTail, src: '', warnings};
  const warningEl = document.getElementById('warning');
  let warningTimer = 0;
  warningEl.addEventListener('click', () => { warningEl.hidden = true; });

  // Browsers suspend an AudioContext made before the user has touched the page (autoplay policy),
  // and SDL's audio output (a document's sound, a conference peer's mic) is one: wrap the
  // constructor before arigl.js makes any, keep every context, and resume them on the first click,
  // key or touch. Harmless where SDL resumes its own.
  (function () {
    const Orig = window.AudioContext || window.webkitAudioContext;
    if (!Orig) return;
    const live = [];
    function Tracked() {
      const c = new Orig(...arguments);
      live.push(c);
      return c;
    }
    Tracked.prototype = Orig.prototype;
    try { window.AudioContext = Tracked; } catch (e) {}
    try { if (window.webkitAudioContext) window.webkitAudioContext = Tracked; } catch (e) {}
    const resume = () => {
      for (const c of live)
        try { if (c.state === 'suspended') c.resume(); } catch (e) {}
    };
    for (const ev of ['pointerdown', 'keydown', 'touchend'])
      window.addEventListener(ev, resume, {capture: true, passive: true});
  })();

  function status(text) { statusEl.textContent = text; }

  // The error page: what went wrong in one line, then everything arigl printed. Text only.
  function fail(headline) {
    if (failed) return;
    failed = true;
    state.state = 'failed';
    overlay.classList.remove('hidden', 'ended');
    overlay.classList.add('failed');
    const said = stderrTail.filter(l => /^arigl(x)?: /.test(l) && !/^arigl(x)?: warning: /.test(l));
    status(headline + (said.length ? ' — ' + said[said.length - 1].replace(/^arigl(x)?: /, '') : ''));
    detailsEl.textContent = (srcShown ? 'document: ' + srcShown + '\n\n' : '') +
                            (stderrTail.length ? stderrTail.join('\n') : '(' + MODULE + ' printed nothing)');
    document.title = 'Error — ' + document.title;
  }

  async function loadConfig() {
    if (window.ARIADNE_CONFIG) return window.ARIADNE_CONFIG;
    let r;
    try {
      r = await fetch('ariadne.config.json', {cache: 'no-cache', credentials: 'same-origin'});
    } catch (e) {
      return {};
    }
    if (r.status === 404) return {};
    if (!r.ok) throw new Error('ariadne.config.json: HTTP ' + r.status);
    try {
      return await r.json();
    } catch (e) {
      throw new Error('ariadne.config.json is not JSON: ' + e.message);
    }
  }

  function list(v, name) {
    if (v === undefined || v === null) return [];
    if (!Array.isArray(v) || v.some(x => typeof x !== 'string'))
      throw new Error('config "' + name + '" must be a list of strings');
    return v;
  }

  // The config's placeholders, in flags, config arguments and env values: {origin} is this page's
  // origin (https://host), {ws_origin} the same host as a WebSocket origin (wss:// on an https
  // page, ws:// on http), {random} five random characters, the same everywhere in one page load.
  const random = Math.random().toString(36).slice(2, 7).padEnd(5, '0');
  const wsOrigin = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host;
  function fill(s) {
    return s.split('{origin}').join(location.origin).split('{ws_origin}').join(wsOrigin)
        .split('{random}').join(random);
  }

  // url_env / fragment_env: which parameters of the page's URL (its query, or its #fragment, which
  // the browser never sends to a server) may set which variable of the module's environment, and
  // how. A rule is the variable's name, or {env, match, value, max}: the value is trimmed and cut
  // to `max` characters (256), must match the regular expression `match` when one is given, and
  // `value`, when given, is what is set instead (a switch: ?synthetic=1 sets "1"). Anything the
  // config does not name is ignored, so a link can only set what the config allows.
  function envRules(spec, name) {
    if (spec === undefined || spec === null) return [];
    if (typeof spec !== 'object' || Array.isArray(spec))
      throw new Error('config "' + name + '" must be an object');
    return Object.entries(spec).map(([param, rule]) => {
      if (typeof rule === 'string') rule = {env: rule};
      if (!rule || typeof rule !== 'object' || typeof rule.env !== 'string' || !rule.env)
        throw new Error('config "' + name + '.' + param + '" must name an env variable');
      let re = null;
      if (rule.match !== undefined) {
        try {
          re = new RegExp(String(rule.match));
        } catch (e) {
          throw new Error('config "' + name + '.' + param + '.match": ' + e.message);
        }
      }
      const max = Number(rule.max) > 0 ? Number(rule.max) : 256;
      const value = rule.value === undefined ? undefined : String(rule.value);
      return {param, env: rule.env, re, max, value};
    });
  }
  function applyEnvRules(rules, from, where, env) {
    for (const r of rules) {
      const raw = from.get(r.param);
      if (raw === null) continue;
      const v = raw.trim().slice(0, r.max);
      if (!v) continue;
      if (r.re && !r.re.test(v)) {
        console.warn('arigl-web: ' + where + r.param + '= ignored: it does not match ' + r.re);
        continue;
      }
      env[r.env] = r.value === undefined ? v : r.value;
    }
  }

  // device_identity: a random id and key for this browser, made once and kept in localStorage
  // (per site, cleared with the site's data); arigl reads them as CVC_DEVICE_ID / CVC_DEVICE_KEY
  // (a conference relay shows the device and pins its key). Where storage is unavailable (blocked,
  // a sandboxed frame) they last this page load only.
  function kept(name, bytes) {
    const fresh = () => {
      const b = new Uint8Array(bytes);
      crypto.getRandomValues(b);
      return Array.from(b, x => x.toString(16).padStart(2, '0')).join('');
    };
    try {
      let v = localStorage.getItem(name);
      if (!new RegExp('^[0-9a-f]{' + 2 * bytes + '}$').test(v || '')) {
        v = fresh();
        localStorage.setItem(name, v);
      }
      return v;
    } catch (e) {
      try { return fresh(); } catch (e2) { return ''; }
    }
  }

  // fullscreen_button: a small button in the corner that puts the page in and out of fullscreen
  // (best on a phone).
  function addFullscreenButton() {
    const b = document.createElement('button');
    b.id = 'fullscreen';
    b.type = 'button';
    b.title = 'Fullscreen';
    b.textContent = '\u26f6'; // a fullscreen symbol
    b.addEventListener('click', () => {
      const el = document.documentElement;
      if (!document.fullscreenElement)
        (el.requestFullscreen || el.webkitRequestFullscreen).call(el);
      else
        document.exitFullscreen();
      canvas.focus();
    });
    document.body.appendChild(b);
    document.addEventListener('fullscreenchange',
                              () => setTimeout(() => window.dispatchEvent(new Event('resize')), 60));
  }

  // Download the files the module needs before it starts, together, with one progress line.
  async function download(names) {
    const got = names.map(() => 0);
    const show = () => {
      const mb = (got.reduce((a, b) => a + b, 0) / 1048576).toFixed(1);
      status('Downloading ' + names.join(' and ') + ' — ' + mb + ' MB');
    };
    return Promise.all(names.map(async (name, i) => {
      const resp = await fetch(name, {cache: 'no-cache'});
      if (!resp.ok) throw new Error('HTTP ' + resp.status + ' fetching ' + name);
      const reader = resp.body.getReader();
      const chunks = [];
      for (;;) {
        const {done, value} = await reader.read();
        if (done) break;
        chunks.push(value);
        got[i] += value.length;
        show();
      }
      const buf = new Uint8Array(got[i]);
      let off = 0;
      for (const c of chunks) { buf.set(c, off); off += c.length; }
      return buf;
    }));
  }

  // Download the module's .wasm (and the Python zip), then start its .js on it.
  async function startModule(Module) {
    const names = [MODULE + '.wasm'].concat(PYTHON_ZIP ? [PYTHON_ZIP] : []);
    const files = await download(names);
    Module.wasmBinary = files[0];
    let zip = files[1];
    if (zip) {
      // The interpreter's home is /py: CPython puts <home>/lib/python312.zip on sys.path. The
      // file system keeps its own copy, so this one goes.
      Module.preRun.push(() => {
        FS.mkdirTree('/py/lib');
        FS.writeFile('/py/lib/python312.zip', zip);
        zip = null;
      });
    }
    status('Starting…');
    window.Module = Module;
    const s = document.createElement('script');
    s.src = MODULE + '.js';
    s.onerror = () => fail('Could not load ' + MODULE + '.js');
    document.body.appendChild(s);
  }

  async function boot() {
    const config = await loadConfig();
    if (typeof config !== 'object' || Array.isArray(config))
      throw new Error('the page config must be a JSON object');
    if (config.title) document.title = String(config.title);
    const params = new URLSearchParams(location.search);
    const src = (config.src_param !== false && params.get('src')) || config.src || 'main.ari';
    let srcUrl;
    try {
      srcUrl = new URL(src, location.href);
    } catch (e) {
      throw new Error('src "' + src + '" is not a URL');
    }
    srcUrl.hash = '';
    srcShown = state.src = srcUrl.href;
    // The placeholders (fill) in a flag, a config argument or an env value, e.g. an app's own
    // server: --instance {origin}, or a relay at {ws_origin}/ws. Not in the URL's ?arg=: those
    // are taken as given.
    const flags = list(config.flags, 'flags').map(fill);
    const args = list(config.args, 'args').map(fill).concat(params.getAll('arg'));
    const env = {};
    if (config.env !== undefined && config.env !== null) {
      if (typeof config.env !== 'object' || Array.isArray(config.env))
        throw new Error('config "env" must be an object');
      for (const [k, v] of Object.entries(config.env)) env[k] = fill(String(v));
    }
    // What the URL may set (url_env, fragment_env), then the device; the page's own keys last, so
    // no rule can set those.
    applyEnvRules(envRules(config.url_env, 'url_env'), params, '?', env);
    applyEnvRules(envRules(config.fragment_env, 'fragment_env'),
                  new URLSearchParams(location.hash.replace(/^#/, '')), '#', env);
    if (config.device_identity === true) {
      const id = kept('cvc.device_id', 16), key = kept('cvc.device_key', 32);
      if (id) env.CVC_DEVICE_ID = id;
      if (key) env.CVC_DEVICE_KEY = key;
    }
    env.CVC_ARIADNE_SRC_ORIGINS = list(config.src_origins, 'src_origins').join(' ');
    env.CVC_ARIADNE_PRELOAD = list(config.preload, 'preload').join('\n');
    if (config.fullscreen_button === true) addFullscreenButton();

    const Module = {
      canvas,
      arguments: ['run'].concat(flags, [srcUrl.href], args),
      print: t => console.log(t),
      printErr: t => {
        stderrTail.push(t);
        if (stderrTail.length > 400) stderrTail.shift();
        console.error(t);
      },
      setStatus: t => { if (t && !failed) status(t); },
      preRun: [() => { for (const [k, v] of Object.entries(env)) ENV[k] = String(v); }],
      onRuntimeInitialized() { status('Loading ' + srcUrl.pathname + '…'); },
      // The document is loaded (cvc/ariadne/web_document.h): take the loading screen down.
      ariadneRunning() {
        if (failed) return;
        state.state = 'running';
        overlay.classList.add('hidden');
        canvas.focus();
      },
      // A warning while it runs (a program that stopped, say): the latest, for a few seconds. arigl
      // prints it to stderr as well, so it is in the console and ariadneWeb.log.
      ariadneWarning(text) {
        warnings.push(text);
        if (warnings.length > 200) warnings.shift();
        const n = warnings.length;
        warningEl.textContent = text + (n > 1 ? '  (' + n + ' warnings)' : '');
        warningEl.hidden = false;
        clearTimeout(warningTimer);
        warningTimer = setTimeout(() => { warningEl.hidden = true; }, 8000);
      },
      // arigl stopped: status 0 is the document quitting, anything else an error.
      ariadneExit(code) {
        if (code !== 0) {
          fail('Could not run ' + srcUrl.pathname);
          return;
        }
        state.state = 'ended';
        overlay.classList.remove('hidden');
        overlay.classList.add('ended');
        status(document.title + ' has ended. Reload the page to start it again.');
      },
      onAbort: what => fail(MODULE + ' stopped: ' + what),
    };
    await startModule(Module);
  }

  boot().catch(e => fail('Could not start: ' + ((e && e.message) || e)));
})();
