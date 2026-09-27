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
#include <vector>

namespace cvc {
namespace ariadne {

// A parsed URI. A bare/relative path (no `<scheme>://`) parses as scheme "file".
struct Uri {
  std::string scheme; // "file" | "state" | "http" | "https" | …  (lowercased)
  std::string path;   // the part after "<scheme>://" (or the whole string for a bare path)
  std::string query;  // the part after '?', if any (no leading '?'). Empty for the file scheme:
                      // '?' is a legal POSIX filename byte, so file paths are never split on it.
  std::string raw;    // the original string, verbatim
};

// The result of resolving a URI: `ok` with `content` (the fetched bytes) and `canonical` (a
// stable identity — e.g. a file's absolute path — used as the base for the fragment's own
// relative references and as a cycle-guard key), or an `error` message.
//
// A handler SHOULD set a non-empty, collision-free `canonical` on success (two spellings of the
// same resource → the same canonical; distinct resources → distinct canonicals), since callers
// key dedup/cycle guards on it. `resolve()` enforces the non-empty half: an ok result with an
// empty canonical falls back to the raw URI, so a forgetful handler cannot collapse the guard.
struct UriResult {
  bool ok = false;
  std::string content;
  std::string canonical;
  std::string error;
};

// §13.10 the WRITE analogue of UriResult: `ok` with a `canonical` identity (e.g. the written
// file's absolute path) for the stored resource, or an `error`. `store()` fills an empty
// canonical with the raw URI, mirroring resolve().
struct StoreResult {
  bool ok = false;
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

// Remove a scheme's registered handler (no-op if none, and a no-op for the built-in "file",
// which is not in the registry). A host tears down its state/http handler this way before the
// context it captured (an app, a state root) is destroyed, so a later resolve() cannot call a
// handler holding a dangling pointer.
void unregister_uri_handler(const std::string &scheme);

// Whether a scheme has a resolver (a registered handler, or the built-in "file").
bool has_uri_handler(const std::string &scheme);

// Register the `cvc://` scheme — a LOCATION-INDEPENDENT import path for the reusable .ari component
// library shipped with libcvc/pycvc. `cvc://<relpath>` (e.g. `cvc://components/stage_lighting.ari`)
// resolves `<relpath>` against a search list, first hit wins:
//   1. each directory in the `CVC_ARIADNE_PATH` environment variable (`:`-separated on POSIX),
//   2. `extra_search_dirs` (in order) — e.g. a pycvc package's bundled components dir,
//   3. the compiled-in install data dir (`<prefix>/share/libcvc/ariadne`, if this build set it),
//   4. the process's current working directory.
// A hit resolves through the built-in `file` reader (so the canonical id is the found file's
// absolute path and a component's OWN relative imports resolve against its directory). App-free and
// process-global; a host calls this once at setup. So a document writes
// `import: cvc://components/foo.ari` and loads regardless of where it lives, while a plain relative
// `import: components/foo.ari` still works for a co-located demo.
void register_cvc_uri_handler(const std::vector<std::string> &extra_search_dirs = {});

// The directories the `cvc://` handler searches, in order (env + extra + install datadir + cwd) —
// exposed for diagnostics and for tooling that lists the component library.
std::vector<std::string>
cvc_uri_search_dirs(const std::vector<std::string> &extra_search_dirs = {});

// Resolve `uri` to its bytes. `base` is the enclosing fragment's canonical location (used to
// resolve a relative file path); pass "" at the top level. Dispatches by scheme: the built-in
// "file" handler for file/bare, else a registered handler; an unknown scheme yields an error.
UriResult resolve(const std::string &uri, const std::string &base = std::string());

// -------------------------------------------------------------------------------------------------
// §13.10 the WRITE side — a `store` (PUT) capability symmetric with resolve(). One scheme registry
// pair backs read (resolve) and write (store); a scheme may register a reader, a writer, or both.
// -------------------------------------------------------------------------------------------------

// A scheme's WRITE handler: store `content` at `u` (its scheme matches the registration), `base`
// as in resolve. The write analogue of UriHandler.
using UriStoreHandler =
    std::function<StoreResult(const Uri &u, const std::string &content, const std::string &base)>;

// Register (or replace) the WRITE handler for a scheme (the write analogue of
// register_uri_handler). The built-in "file" writer is always available (a registration overrides
// it). Process-global and thread-safe; register before store().
void register_uri_store_handler(const std::string &scheme, UriStoreHandler handler);

// Remove a scheme's registered WRITE handler (no-op if none / for the built-in "file"). A host
// tears down a state/http writer this way before the context it captured is destroyed.
void unregister_uri_store_handler(const std::string &scheme);

// Whether a scheme can be written (a registered writer, or the built-in "file").
bool has_uri_store_handler(const std::string &scheme);

// Store `content` at `uri`. `base` resolves a relative file path (as resolve). Dispatches by
// scheme: a registered writer wins, else the built-in "file" writer (an ATOMIC temp-write in the
// target's directory + rename over it, so a crash/concurrent reader never sees a half-written
// file), else an error. Never throws.
StoreResult store(const std::string &uri, const std::string &content,
                  const std::string &base = std::string());

// The byte caps the built-in `file` reader (resolve) and writer (store) enforce — a read/write
// past its cap is refused rather than OOM/write-something-unreadable. Both default to 16 MiB; a
// cap of 0 means UNLIMITED. Process-global and thread-safe (an atomic). The resolver stays
// app-free, so these are plain setters — a host drives them from the state tree via
// sync_resolver_caps_from_state (uri_state.h). Read and store are separate caps.
std::size_t resolve_file_byte_cap();
void set_resolve_file_byte_cap(std::size_t bytes);
std::size_t store_file_byte_cap();
void set_store_file_byte_cap(std::size_t bytes);

// Resolve a relative `path` against `base` (the enclosing fragment's directory) to an absolute,
// normalized path string — the file resolver's base rule, exposed for callers that compute a
// nested fragment's base from a parent's canonical path. An absolute `path` is returned
// normalized. When `base` is empty — the document was loaded from a string, so there is no
// source location — a relative `path` anchors to the process's current working directory.
std::string resolve_file_path(const std::string &path, const std::string &base);

// A resolved URI materialized as a LOCAL FILE PATH a byte-oriented reader (read_geometry /
// readVolumeFile / read_image) can open — the "temp-file bridge" of §13.4. For a file:// (or
// bare) URI the `path` IS the resolved file, no copy. For any other scheme (state://, http://…)
// the fetched bytes are written to a temporary file whose extension mirrors the source (so an
// extension-keyed reader still dispatches), and that temp file is REMOVED when the ResolvedFile
// is destroyed — so keep it alive for the whole read. Move-only.
struct ResolvedFile {
  bool ok = false;
  std::string path;  // a local path to open, valid while this object lives
  std::string error; // set (and path empty) on failure

  ResolvedFile() = default;
  ResolvedFile(const ResolvedFile &) = delete;
  ResolvedFile &operator=(const ResolvedFile &) = delete;
  ResolvedFile(ResolvedFile &&other) noexcept { *this = std::move(other); }
  ResolvedFile &operator=(ResolvedFile &&other) noexcept;
  ~ResolvedFile();

private:
  friend ResolvedFile resolve_to_file(const std::string &, const std::string &);
  bool is_temp_ = false; // path is a temp file this object owns and must delete
};

// Resolve `uri` (relative to `base`) to a local file path — see ResolvedFile. file:// resolves to
// the path in place; every other scheme fetches bytes into an owned temp file.
ResolvedFile resolve_to_file(const std::string &uri, const std::string &base = std::string());

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_H
