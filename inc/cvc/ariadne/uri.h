#ifndef CVC_ARIADNE_URI_H
#define CVC_ARIADNE_URI_H

// Ariadne — the shared URI resolver (roadmap §13). ONE resolver backs every external
// reference in an .ari document: `import:`/`load:` fragments and (later) node `source:` data.
// It is a thin, backend-free seam: parse a URI, dispatch by scheme to a handler, return the
// resolved bytes + a canonical identity (for relative-resolution bases and cycle guards).
//
// The `file` scheme (and a bare/relative path, which means `file`) is built in and app-free,
// so the pure-libcvc loader can pull `.ari` unit libraries off disk. Other schemes — `state`
// (a fragment held in a cvc::state node) and `http`/`https` (a remote fragment) — are added by
// registering a handler, since they need context the loader does not have (an app, an HTTP
// client). This keeps the core resolver dependency-free while the scheme set is extensible.

#include <functional>
#include <string>

namespace cvc {
namespace ariadne {

// A parsed URI. A bare/relative path (no `<scheme>://`) parses as scheme "file".
struct Uri {
  std::string scheme; // "file" | "state" | "http" | "https" | …  (lowercased)
  std::string path;   // the part after "<scheme>://" (or the whole string for a bare path)
  std::string query;  // the part after '?', if any (no leading '?')
  std::string raw;    // the original string, verbatim
};

// The result of resolving a URI: `ok` with `content` (the fetched bytes) and `canonical` (a
// stable identity — e.g. a file's absolute path — used as the base for the fragment's own
// relative references and as a cycle-guard key), or an `error` message.
struct UriResult {
  bool ok = false;
  std::string content;
  std::string canonical;
  std::string error;
};

// Parse a URI string. Never fails: a malformed/bare string falls back to scheme "file".
Uri parse_uri(const std::string &s);

// A scheme handler: resolve `u` (its scheme matches the registration), where `base` is the
// enclosing fragment's canonical location (a directory for file, empty at the top level).
using UriHandler = std::function<UriResult(const Uri &u, const std::string &base)>;

// Register (or replace) the handler for a URI scheme (e.g. "state", "http"). The built-in
// "file" scheme is always available and need not be registered (a registration for "file"
// overrides it). Process-global and thread-safe; register before load_*. Mirrors the
// register_scene_node_type / register_ari_block / register_widget_type registries.
void register_uri_handler(const std::string &scheme, UriHandler handler);

// Whether a scheme has a resolver (a registered handler, or the built-in "file").
bool has_uri_handler(const std::string &scheme);

// Resolve `uri` to its bytes. `base` is the enclosing fragment's canonical location (used to
// resolve a relative file path); pass "" at the top level. Dispatches by scheme: the built-in
// "file" handler for file/bare, else a registered handler; an unknown scheme yields an error.
UriResult resolve(const std::string &uri, const std::string &base = std::string());

// Resolve a relative `path` against `base` (a file or directory) to an absolute, normalized
// path string — the file resolver's base rule, exposed for callers that compute a nested
// fragment's base from a parent's canonical path. An absolute `path` is returned normalized.
std::string resolve_file_path(const std::string &path, const std::string &base);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_H
