// arigl-web.js — the arigl-web page's loader (arigl-web.html, which documents the config): read
// the config, resolve ?src= against the page, then start arigl.js on arigl.wasm with the command
// line `arigl run <config flags> <src URL> <document args>`. arigl fetches the document and the
// files it imports (cvc/ariadne/web_document.h) and calls back: Module.ariadneRunning when the
// document is loaded, Module.ariadneExit(status) when it stops. A failure shows the error screen
// with everything arigl printed.
(function () {
  'use strict';
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
  // For a test or a console: loading, running, ended or failed, and what arigl printed.
  const state = window.ariadneWeb = {state: 'loading', log: stderrTail, src: ''};

  function status(text) { statusEl.textContent = text; }

  // The error page: what went wrong in one line, then everything arigl printed. Text only.
  function fail(headline) {
    if (failed) return;
    failed = true;
    state.state = 'failed';
    overlay.classList.remove('hidden', 'ended');
    overlay.classList.add('failed');
    const said = stderrTail.filter(l => /^arigl: /.test(l) && !/^arigl: warning: /.test(l));
    status(headline + (said.length ? ' — ' + said[said.length - 1].replace(/^arigl: /, '') : ''));
    detailsEl.textContent = (srcShown ? 'document: ' + srcShown + '\n\n' : '') +
                            (stderrTail.length ? stderrTail.join('\n') : '(arigl printed nothing)');
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

  // Download arigl.wasm with a progress line, then start arigl.js on it.
  async function startModule(Module) {
    const resp = await fetch('arigl.wasm', {cache: 'no-cache'});
    if (!resp.ok) throw new Error('HTTP ' + resp.status + ' fetching arigl.wasm');
    const total = Number(resp.headers.get('content-length')) || 0; // compressed: a rough guide
    const reader = resp.body.getReader();
    const chunks = [];
    let got = 0;
    for (;;) {
      const {done, value} = await reader.read();
      if (done) break;
      chunks.push(value);
      got += value.length;
      const mb = (got / 1048576).toFixed(1);
      status('Downloading arigl.wasm — ' + mb + ' MB' +
             (total > got ? ' (' + Math.round(100 * got / total) + '%)' : ''));
    }
    const buf = new Uint8Array(got);
    let off = 0;
    for (const c of chunks) { buf.set(c, off); off += c.length; }
    Module.wasmBinary = buf;
    status('Starting…');
    window.Module = Module;
    const s = document.createElement('script');
    s.src = 'arigl.js';
    s.onerror = () => fail('Could not load arigl.js');
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
    // {origin} in a flag or a config argument is this page's origin (e.g. an app's own server,
    // --instance {origin}). Not in the URL's ?arg=: those are taken as given.
    const here = s => s.split('{origin}').join(location.origin);
    const flags = list(config.flags, 'flags').map(here);
    const args = list(config.args, 'args').map(here).concat(params.getAll('arg'));
    const env = Object.assign({}, config.env || {});
    env.CVC_ARIADNE_SRC_ORIGINS = list(config.src_origins, 'src_origins').join(' ');
    env.CVC_ARIADNE_PRELOAD = list(config.preload, 'preload').join('\n');

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
      onAbort: what => fail('arigl stopped: ' + what),
    };
    await startModule(Module);
  }

  boot().catch(e => fail('Could not start: ' + ((e && e.message) || e)));
})();
