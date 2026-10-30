// A minimal XMLHttpRequest over node's fetch, enough for emscripten's Fetch.js, so the browser
// build of ari can make requests under node (which has fetch but no XMLHttpRequest): the wasm
// ari's network check in ci.yml, `node --require src/ari/tests/xhr-shim.cjs ari.js ...`. It runs
// the same cvc::net fetch backend and Ariadne verbs a page does; a browser's CORS it does not.
class XMLHttpRequest {
  constructor() { this.readyState = 0; this.status = 0; this.statusText = ''; this.response = null;
    this.responseType = ''; this.timeout = 0; this.withCredentials = false; this._headers = {};
    this._resHeaders = ''; this.responseURL = ''; }
  open(method, url) { this._method = method; this.url = url; this.responseURL = url; this.readyState = 1; }
  setRequestHeader(k, v) { this._headers[k] = v; }
  overrideMimeType() {}
  getAllResponseHeaders() { return this._resHeaders; }
  abort() {}
  send(body) {
    const opts = { method: this._method, headers: this._headers };
    if (body != null && this._method !== 'GET' && this._method !== 'HEAD') opts.body = body;
    fetch(this.url, opts).then(async (res) => {
      this.status = res.status; this.statusText = res.statusText; this.responseURL = res.url;
      let h = ''; res.headers.forEach((v, k) => { h += k + ': ' + v + '\r\n'; }); this._resHeaders = h;
      this.response = await res.arrayBuffer();
      this.readyState = 4;
      if (this.onreadystatechange) this.onreadystatechange();
      if (this.onload) this.onload({ loaded: this.response.byteLength, total: this.response.byteLength });
    }).catch((e) => {
      this.status = 0; this.readyState = 4;
      if (this.onreadystatechange) this.onreadystatechange();
      if (this.onerror) this.onerror(e);
    });
  }
}
globalThis.XMLHttpRequest = XMLHttpRequest;
