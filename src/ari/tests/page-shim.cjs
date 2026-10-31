// A stand-in for a browser page under node, for the wasm ari's sign-in check (ci.yml): the page's
// location, its sessionStorage and history.replaceState, which the redirect flow uses
// (docs/ARIADNE.md §10.5). ARI_PAGE_URL is the page's URL. ARI_PAGE_STATE names a JSON file that
// keeps sessionStorage across runs (the page leaving for the provider and loading again when it
// comes back) and records where the page went: `navigated` (location.assign) and `replaced`
// (history.replaceState). No `window` is defined, so Emscripten still sees node.
const fs = require('fs');
const file = process.env.ARI_PAGE_STATE || '';
let st = { session: {}, navigated: [], replaced: [] };
if (file && fs.existsSync(file)) st = JSON.parse(fs.readFileSync(file, 'utf8'));
const save = () => { if (file) fs.writeFileSync(file, JSON.stringify(st, null, 1)); };
let href = process.env.ARI_PAGE_URL || 'https://app.example/ari.html';
globalThis.location = {
  get href() { return href; },
  get origin() { return new URL(href).origin; },
  get search() { return new URL(href).search; },
  assign(u) { st.navigated.push(String(u)); save(); },
};
globalThis.sessionStorage = {
  getItem(k) { return Object.prototype.hasOwnProperty.call(st.session, k) ? st.session[k] : null; },
  setItem(k, v) { st.session[k] = String(v); save(); },
  removeItem(k) { delete st.session[k]; save(); },
};
globalThis.history = {
  state: null,
  replaceState(s, t, u) { href = new URL(u, href).href; st.replaced.push(href); save(); },
};
