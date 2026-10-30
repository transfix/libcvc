// ari-web.js — the loader of ari's browser pages (ari.html, ari_tvision.html; src/ari/web): an
// xterm.js terminal wired to the module's stdio. The page names the module, its label and its
// default document in data- attributes on <body>, so this file is the same for every page and the
// page has no inline code: it runs under script-src 'self' 'wasm-unsafe-eval'.
//
//   stdin  <- every byte xterm.js produces: keys, and SGR mouse reports once the program turns
//             them on
//   stdout -> xterm.js. Both backends end each frame with a NUL (Emscripten has no flush hook), so
//             bytes are buffered and written to the terminal per frame, or at the next animation
//             frame.
//   resize -> _ftxui_on_resize(cols, rows), FTXUI's hook for the terminal size, and
//             _tvision_set_screen_size(cols, rows), tvision's, whichever the module exports.
//
// The document: ?src= (unless the page config pins its own), the config's src, or the page's
// default (data-default-src; empty: the document baked into the build). A document by URL is
// fetched by ari itself before it loads (cvc/ariadne/web_document.h).
(function () {
  'use strict';
  const page = document.body.dataset;
  const MODULE = page.module || 'ari';
  const LABEL = page.label || MODULE;
  const DEFAULT_SRC = page.defaultSrc || '';

  const statusEl = document.getElementById('status');
  // In the wasm demo gallery this page is <demo>/index.html (build-pages.py), so link back to it.
  // Served on its own it is <module>.html, with no gallery above it.
  if (location.pathname.endsWith('/')) document.getElementById('gallery').hidden = false;
  const setStatus = (text) => { statusEl.textContent = text; };

  const term = new Terminal({
    fontFamily: '"DejaVu Sans Mono", Menlo, Consolas, "Liberation Mono", monospace',
    fontSize: 14,
    scrollback: 0,
    macOptionIsMeta: true,
    theme: { background: '#0f1115' },
  });
  const fit = new FitAddon.FitAddon();
  term.loadAddon(fit);
  // The WebGL renderer draws box-drawing glyphs itself (seamless window borders) and keeps up with
  // full-screen redraws. Loaded before open(), it is the terminal's renderer from the start, so
  // xterm.js never makes its DOM renderer, which injects <style> elements a style-src 'self'
  // policy refuses. Without WebGL xterm.js falls back to the DOM renderer.
  try {
    const webgl = new WebglAddon.WebglAddon();
    webgl.onContextLoss(() => webgl.dispose());
    term.loadAddon(webgl);
  } catch (e) {
    console.warn(LABEL + ': WebGL renderer unavailable, using the DOM renderer:', e);
  }
  const termEl = document.getElementById('term');
  term.open(termEl);
  term.focus();

  // stdin: what xterm.js produces, as bytes (UTF-8 for text; onBinary carries raw bytes, e.g. X10
  // mouse reports past column 95).
  const stdin = [];
  const utf8 = new TextEncoder();
  term.onData((s) => { for (const b of utf8.encode(s)) stdin.push(b); });
  term.onBinary((s) => { for (let i = 0; i < s.length; ++i) stdin.push(s.charCodeAt(i) & 0xff); });

  // stdout: one terminal write per frame (the program ends each frame with a NUL). Anything
  // written without one goes out at the next animation frame.
  let frame = [];
  const flush = () => {
    if (frame.length) {
      term.write(new Uint8Array(frame));
      frame = [];
    }
  };
  const flushLater = () => {
    flush();
    requestAnimationFrame(flushLater);
  };
  requestAnimationFrame(flushLater);
  let errLine = [];
  // What ari printed to stderr, shown in the terminal when it exits with an error; and, for a test
  // or the console, the page's state (loading, running, ended or failed) and the terminal's text.
  const errLog = [];
  const state = window.ariadneWeb = {
    state: 'loading', log: errLog, src: '',
    text: () => {
      const b = term.buffer.active, out = [];
      for (let i = 0; i < b.length; ++i) out.push(b.getLine(i).translateToString(true));
      return out.join('\n');
    },
  };
  const showErrors = (headline) => {
    flush();
    term.write('\r\n\x1b[31m' + headline + '\x1b[0m\r\n');
    for (const l of errLog) term.write('\x1b[2m' + l.replace(/\n/g, '\r\n') + '\x1b[0m\r\n');
  };

  let started = false;
  const resize = () => {
    const dims = fit.proposeDimensions();
    if (!dims || !dims.cols || !dims.rows) return; // hidden or not laid out
    if (dims.cols !== term.cols || dims.rows !== term.rows) term.resize(dims.cols, dims.rows);
    if (!started) return;
    if (Module._ftxui_on_resize) Module._ftxui_on_resize(term.cols, term.rows);
    if (Module._tvision_set_screen_size) Module._tvision_set_screen_size(term.cols, term.rows);
  };

  const Module = window.Module = {
    arguments: [], // none: the baked document; configure() sets them for a document by URL
    stdin: () => (stdin.length ? stdin.shift() : null),
    stdout: (c) => { if (c === 0) flush(); else frame.push(c); },
    stderr: (c) => {
      if (c === 10) {
        const line = new TextDecoder().decode(new Uint8Array(errLine));
        console.warn(line);
        errLog.push(line);
        if (errLog.length > 200) errLog.shift();
        errLine = [];
      } else {
        errLine.push(c);
      }
    },
    onRuntimeInitialized() {
      // Runs before main: FTXUI and tvision size their screens from this.
      started = true;
      resize();
      new ResizeObserver(resize).observe(termEl);
      setStatus('running');
    },
    // The document is loaded (cvc/ariadne/web_document.h).
    ariadneRunning() { state.state = 'running'; },
    onExit(code) {
      state.state = code === 0 ? 'ended' : 'failed';
      if (code !== 0 && errLog.length)
        showErrors(LABEL + ' could not run ' + (state.src || 'its document') + ':');
      flush();
      term.write('\r\n\x1b[2m[' + LABEL + ' exited with status ' + code + ']\x1b[0m\r\n');
      statusEl.textContent = '';
      const again = document.createElement('button');
      again.textContent = 'run again';
      again.addEventListener('click', () => location.reload());
      statusEl.append('exited ', again);
    },
    onAbort(what) {
      state.state = 'failed';
      flush();
      setStatus('aborted: ' + what);
    },
  };

  // The page config (arigl-web's: docs/CVCGL_WASM.md lists its keys).
  const loadConfig = async () => {
    if (window.ARIADNE_CONFIG) return window.ARIADNE_CONFIG;
    let r;
    try {
      r = await fetch('ariadne.config.json', {cache: 'no-cache', credentials: 'same-origin'});
    } catch (e) {
      return {};
    }
    if (r.status === 404) return {};
    if (!r.ok) throw new Error('ariadne.config.json: HTTP ' + r.status);
    return r.json();
  };
  const list = (v, name) => {
    if (v === undefined || v === null) return [];
    if (!Array.isArray(v) || v.some((x) => typeof x !== 'string'))
      throw new Error('config "' + name + '" must be a list of strings');
    return v;
  };
  const configure = async () => {
    const config = await loadConfig();
    const params = new URLSearchParams(location.search);
    const src = (config.src_param !== false && params.get('src')) || config.src || DEFAULT_SRC;
    if (!src) return; // the baked document
    const url = new URL(src, location.href);
    url.hash = '';
    state.src = url.href;
    document.getElementById('doc').textContent = url.pathname;
    if (config.title) document.title = String(config.title);
    // {origin} in a flag or a config argument is this page's origin (e.g. an app's own server,
    // --instance {origin}). Not in the URL's ?arg=: those are taken as given.
    const here = (s) => s.split('{origin}').join(location.origin);
    const flags = list(config.flags, 'flags').map(here);
    Module.arguments = ['run'].concat(flags, [url.href], list(config.args, 'args').map(here),
                                      params.getAll('arg'));
    const env = Object.assign({}, config.env || {});
    env.CVC_ARIADNE_SRC_ORIGINS = list(config.src_origins, 'src_origins').join(' ');
    env.CVC_ARIADNE_PRELOAD = list(config.preload, 'preload').join('\n');
    Module.preRun = [() => { for (const [k, v] of Object.entries(env)) ENV[k] = String(v); }];
  };

  // Start the program only once the terminal has its real size: xterm.css and the font must be in
  // before fit can measure a cell; otherwise the first frames are drawn at a bogus size.
  //
  // Both files are fetched revalidated (cache: 'no-cache'), so a redeploy never pairs a cached
  // <module>.js with a new <module>.wasm. The wasm goes to the runtime directly
  // (Module.wasmBinary); the glue then loads from the copy just revalidated.
  const start = async () => {
    fit.fit();
    const get = async (name) => {
      const r = await fetch(name, {cache: 'no-cache'});
      if (!r.ok) throw new Error('HTTP ' + r.status + ' fetching ' + name);
      return r;
    };
    try {
      await configure();
      setStatus('loading ' + MODULE + '.wasm…');
      Module.wasmBinary = new Uint8Array(await (await get(MODULE + '.wasm')).arrayBuffer());
      await get(MODULE + '.js');
      const script = document.createElement('script');
      script.src = MODULE + '.js';
      script.addEventListener('error', () => setStatus('load failed: ' + MODULE + '.js'));
      document.body.append(script);
    } catch (e) {
      state.state = 'failed';
      setStatus('load failed: ' + ((e && e.message) || e));
      term.write('\x1b[31m' + ((e && e.message) || e) + '\x1b[0m\r\n');
    }
  };
  window.addEventListener('load', () => document.fonts.ready.then(start));
})();
