// arigl-web: a document named by URL. See web_document.h.

#include "web_document.h"

#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/doc_fetch.h>
#include <filesystem>
#include <sstream>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

// clang-format off: JavaScript bodies
// One GET through the page's fetch(), awaited under Asyncify. The reply (or the error's text)
// waits in the module until arigl_fetch_take copies it out, so nothing here allocates in wasm.
// cache: 'no-cache' revalidates, so a redeployed document is not served stale.
EM_ASYNC_JS(int, arigl_fetch_begin, (const char *url), {
  try {
    const r = await fetch(UTF8ToString(url), {credentials : 'same-origin', cache : 'no-cache'});
    Module.__ariglFetched = new Uint8Array(await r.arrayBuffer());
    return r.status;
  } catch (e) {
    Module.__ariglFetched = new TextEncoder().encode(String((e && e.message) || e));
    return -1;
  }
});
EM_JS(int, arigl_fetch_size, (), { return Module.__ariglFetched ? Module.__ariglFetched.length : 0; });
EM_JS(void, arigl_fetch_take, (char *dst), {
  if (Module.__ariglFetched)
    HEAPU8.set(Module.__ariglFetched, dst);
  Module.__ariglFetched = null;
});
// The page's origin (location.origin), "" on a page without one (file://).
EM_JS(int, arigl_page_origin, (char *buf, int cap), {
  var s = typeof location !== 'undefined' && location.origin ? location.origin : '';
  if (s === 'null')
    s = '';
  stringToUTF8(s, buf, cap);
  return lengthBytesUTF8(s);
});
// The page's hooks (web/arigl-web.html sets them): the document is loaded and running; arigl has
// stopped (its stderr says why).
EM_JS(void, arigl_tell_page_running, (), {
  if (typeof Module !== 'undefined' && typeof Module.ariglRunning === 'function')
    Module.ariglRunning();
});
EM_JS(void, arigl_tell_page, (int status), {
  if (typeof Module !== 'undefined' && typeof Module.ariglExit === 'function')
    Module.ariglExit(status);
});
// clang-format on
#endif

namespace arigl {

namespace ari = cvc::ariadne;

namespace {
std::vector<std::string> split_env(const char *name, char sep) {
  std::vector<std::string> out;
  const char *v = std::getenv(name);
  if (!v)
    return out;
  std::istringstream in(v);
  for (std::string item; std::getline(in, item, sep);) {
    while (!item.empty() && (item.back() == ' ' || item.back() == '\r' || item.back() == '\n'))
      item.pop_back();
    if (!item.empty())
      out.push_back(item);
  }
  return out;
}
} // namespace

bool is_url_document(const std::string &doc) {
  return doc.rfind("http://", 0) == 0 || doc.rfind("https://", 0) == 0;
}

int stage_url_document(ari::HostOptions &o) {
#ifdef __EMSCRIPTEN__
  ari::DocSetOptions opts;
  char origin[512];
  if (arigl_page_origin(origin, sizeof origin) < static_cast<int>(sizeof origin))
    opts.page_origin = origin;
  opts.allow_origins = split_env("CVC_ARIGL_SRC_ORIGINS", ' ');
  opts.preload = split_env("CVC_ARIGL_PRELOAD", '\n');
  const ari::DocFetcher fetch = [](const std::string &url) {
    ari::DocFetchResponse r;
    const int status = arigl_fetch_begin(url.c_str());
    std::string body(static_cast<std::size_t>(arigl_fetch_size()), '\0');
    arigl_fetch_take(body.data());
    if (status < 0) {
      r.error = body.empty() ? "no reply" : body;
      return r;
    }
    r.ok = true;
    r.status = status;
    r.body = std::move(body);
    return r;
  };
  const ari::DocSet set = ari::fetch_document_set(o.doc, fetch, opts);
  for (const std::string &w : set.warnings)
    std::fprintf(stderr, "arigl: warning: %s\n", w.c_str());
  std::string error = set.error;
  if (set.ok && !ari::write_document_set(set, "/", &error))
    error = "staging " + o.doc + ": " + error;
  if (!set.ok || !error.empty()) {
    std::fprintf(stderr, "arigl: %s\n", error.c_str());
    return 1;
  }
  std::fprintf(stderr, "arigl: %s: %zu file(s) from %s\n", set.doc_path.c_str(), set.files.size(),
               set.src.origin().c_str());
  // Run in the document's directory, as `cd share/<app> && arigl run main.ari` would: a path a
  // document hands a scene source or a data reader is relative to the working directory.
  std::error_code ec;
  std::filesystem::current_path(std::filesystem::path(set.doc_path).parent_path(), ec);
  o.doc = set.doc_path;
  return -1;
#else
  std::fprintf(stderr,
               "arigl: %s is a URL; arigl runs a document from a path (the browser shell, "
               "arigl-web, loads documents by URL)\n",
               o.doc.c_str());
  return 2;
#endif
}

void report_running() {
#ifdef __EMSCRIPTEN__
  arigl_tell_page_running();
#endif
}

void report_exit(int status) {
#ifdef __EMSCRIPTEN__
  arigl_tell_page(status);
#else
  (void)status;
#endif
}

} // namespace arigl
