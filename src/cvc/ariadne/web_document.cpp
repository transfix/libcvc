// Ariadne — a runtime's document named by URL, in a browser. See inc/cvc/ariadne/web_document.h.

#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/doc_fetch.h>
#include <cvc/ariadne/web_document.h>
#include <filesystem>
#include <sstream>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>

// clang-format off: JavaScript bodies
// One GET through the page's fetch(), awaited under Asyncify. The reply (or the error's text)
// waits in the module until ariadne_fetch_take copies it out, so nothing here allocates in wasm.
// cache: 'no-cache' revalidates, so a redeployed document is not served stale.
EM_ASYNC_JS(int, ariadne_fetch_begin, (const char *url), {
  try {
    const r = await fetch(UTF8ToString(url), {credentials : 'same-origin', cache : 'no-cache'});
    Module.__ariadneFetched = new Uint8Array(await r.arrayBuffer());
    return r.status;
  } catch (e) {
    Module.__ariadneFetched = new TextEncoder().encode(String((e && e.message) || e));
    return -1;
  }
});
EM_JS(int, ariadne_fetch_size, (), { return Module.__ariadneFetched ? Module.__ariadneFetched.length : 0; });
EM_JS(void, ariadne_fetch_take, (char *dst), {
  if (Module.__ariadneFetched)
    HEAPU8.set(Module.__ariadneFetched, dst);
  Module.__ariadneFetched = null;
});
// The page's origin (location.origin), "" on a page without one (file://).
EM_JS(int, ariadne_page_origin, (char *buf, int cap), {
  var s = typeof location !== 'undefined' && location.origin ? location.origin : '';
  if (s === 'null')
    s = '';
  stringToUTF8(s, buf, cap);
  return lengthBytesUTF8(s);
});
// The page's hooks: the document is loaded and running; the runtime has stopped (its stderr says
// why).
EM_JS(void, ariadne_tell_page_running, (), {
  if (typeof Module !== 'undefined' && typeof Module.ariadneRunning === 'function')
    Module.ariadneRunning();
});
EM_JS(void, ariadne_tell_page_exit, (int status), {
  if (typeof Module !== 'undefined' && typeof Module.ariadneExit === 'function')
    Module.ariadneExit(status);
});
// clang-format on
#endif

namespace cvc {
namespace ariadne {

#ifdef __EMSCRIPTEN__
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
#endif

bool is_url_document(const std::string &doc) {
  return doc.rfind("http://", 0) == 0 || doc.rfind("https://", 0) == 0;
}

int stage_url_document(HostOptions &o, const std::string &runtime) {
  const char *const who = runtime.c_str();
#ifdef __EMSCRIPTEN__
  DocSetOptions opts;
  char origin[512];
  if (ariadne_page_origin(origin, sizeof origin) < static_cast<int>(sizeof origin))
    opts.page_origin = origin;
  opts.allow_origins = split_env("CVC_ARIADNE_SRC_ORIGINS", ' ');
  opts.preload = split_env("CVC_ARIADNE_PRELOAD", '\n');
  const DocFetcher fetch = [](const std::string &url) {
    DocFetchResponse r;
    const int status = ariadne_fetch_begin(url.c_str());
    std::string body(static_cast<std::size_t>(ariadne_fetch_size()), '\0');
    ariadne_fetch_take(body.data());
    if (status < 0) {
      r.error = body.empty() ? "no reply" : body;
      return r;
    }
    r.ok = true;
    r.status = status;
    r.body = std::move(body);
    return r;
  };
  const DocSet set = fetch_document_set(o.doc, fetch, opts);
  for (const std::string &w : set.warnings)
    std::fprintf(stderr, "%s: warning: %s\n", who, w.c_str());
  std::string error = set.error;
  if (set.ok && !write_document_set(set, "/", &error))
    error = "staging " + o.doc + ": " + error;
  if (!set.ok || !error.empty()) {
    std::fprintf(stderr, "%s: %s\n", who, error.c_str());
    return 1;
  }
  std::fprintf(stderr, "%s: %s: %zu file(s) from %s\n", who, set.doc_path.c_str(), set.files.size(),
               set.src.origin().c_str());
  // Run in the document's directory, as `cd share/<app> && ari run main.ari` would: a path a
  // document hands a scene source or a data reader is relative to the working directory.
  std::error_code ec;
  std::filesystem::current_path(std::filesystem::path(set.doc_path).parent_path(), ec);
  o.doc = set.doc_path;
  return -1;
#else
  std::fprintf(stderr,
               "%s: %s is a URL; %s runs a document from a path (its browser page loads "
               "documents by URL)\n",
               who, o.doc.c_str(), who);
  return 2;
#endif
}

void report_running() {
#ifdef __EMSCRIPTEN__
  ariadne_tell_page_running();
#endif
}

void report_exit(int status) {
#ifdef __EMSCRIPTEN__
  ariadne_tell_page_exit(status);
#else
  (void)status;
#endif
}

} // namespace ariadne
} // namespace cvc
