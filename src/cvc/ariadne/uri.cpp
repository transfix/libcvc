// Ariadne — the shared URI resolver (roadmap §13). See uri.h. Backend-free and app-free for
// the built-in `file` scheme; other schemes are pluggable handlers. No yaml dependency — this
// resolves bytes; the loader parses them.

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cvc/ariadne/uri.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <utility>

#ifndef _WIN32
#include <cerrno>   // EINTR
#include <cstdlib>  // mkstemps
#include <unistd.h> // write, close, ssize_t
#endif

namespace cvc {
namespace ariadne {

namespace {

std::mutex &uri_mutex() {
  static std::mutex m;
  return m;
}
std::map<std::string, UriHandler> &uri_registry() {
  static std::map<std::string, UriHandler> r;
  return r;
}
std::map<std::string, UriStoreHandler> &uri_store_registry() { // §13.10 write handlers
  static std::map<std::string, UriStoreHandler> r;
  return r;
}

std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

// A per-process, unpredictable token for temp-file names (the Windows fallback path; POSIX uses
// mkstemps for exclusive creation instead). Computed once from random_device.
[[maybe_unused]] const std::string &proc_temp_token() {
  static const std::string tok = [] {
    std::random_device rd;
    std::ostringstream os;
    os << std::hex << rd() << rd(); // ~64 bits of entropy
    return os.str();
  }();
  return tok;
}

// Whether a host has REGISTERED a "file" handler (overriding the built-in). resolve() honors such
// an override; resolve_to_file must too, rather than short-circuiting to the raw on-disk path.
bool file_scheme_overridden() {
  std::lock_guard<std::mutex> lock(uri_mutex());
  return uri_registry().find("file") != uri_registry().end();
}

// Create a FRESH file in `dir` named "<prefix>XXXXXX<suffix>", write `content`, return its path
// (empty + `err` set on failure). Created EXCLUSIVELY (mkstemps: O_CREAT|O_EXCL, fresh random name,
// no symlink-follow, mode 0600) so a predicted-name symlink cannot hijack the write (CWE-377). The
// shared write primitive for the temp bridge (into the temp dir) and the atomic file store (into
// the target's dir). POSIX; a Windows fallback uses an unpredictable random name.
std::string write_new_file(const std::filesystem::path &dir, const std::string &prefix,
                           const std::string &suffix, const std::string &content,
                           std::string &err) {
  namespace fs = std::filesystem;
#ifndef _WIN32
  std::string tmpl = (dir / (prefix + "XXXXXX" + suffix)).string();
  const int fd = ::mkstemps(&tmpl[0], static_cast<int>(suffix.size()));
  if (fd < 0) {
    err = "cannot create a file in '" + dir.string() + "'";
    return std::string();
  }
  const char *p = content.data();
  std::size_t left = content.size();
  bool ok = true;
  while (left > 0) {
    const ssize_t w = ::write(fd, p, left);
    if (w < 0) {
      if (errno == EINTR)
        continue; // an interrupted write is retryable, not a failure
      ok = false;
      break;
    }
    if (w == 0) { // a regular file should not return 0, but guard against a spin
      ok = false;
      break;
    }
    p += w;
    left -= static_cast<std::size_t>(w);
  }
  ::close(fd);
  if (!ok) {
    std::error_code rmec;
    fs::remove(tmpl, rmec);
    err = "cannot write a file in '" + dir.string() + "'";
    return std::string();
  }
  return tmpl; // mkstemps rewrote the XXXXXX in place
#else
  static std::atomic<std::uint64_t> counter{0};
  const std::uint64_t n = counter.fetch_add(1);
  const fs::path path = dir / (prefix + proc_temp_token() + "-" + std::to_string(n) + suffix);
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    err = "cannot create a file in '" + dir.string() + "'";
    return std::string();
  }
  out.write(content.data(), static_cast<std::streamsize>(content.size()));
  out.close();
  if (!out) {
    std::error_code rmec;
    fs::remove(path, rmec);
    err = "cannot write a file in '" + dir.string() + "'";
    return std::string();
  }
  return path.string();
#endif
}

// The default file read/store byte cap: an .ari fragment or a saved blob larger than this is
// almost certainly a mistake (or an attack), so the reader/writer refuses rather than OOM /
// write-something-unreadable. Runtime-configurable via set_resolve_file_byte_cap /
// set_store_file_byte_cap (0 = unlimited); a host drives them from the state tree
// (sync_resolver_caps_from_state). Meyers-singleton atomics so init order is well-defined.
constexpr std::size_t kDefaultFileByteCap = 16u * 1024u * 1024u; // 16 MiB
std::atomic<std::size_t> &resolve_cap_cell() {
  static std::atomic<std::size_t> c{kDefaultFileByteCap};
  return c;
}
std::atomic<std::size_t> &store_cap_cell() {
  static std::atomic<std::size_t> c{kDefaultFileByteCap};
  return c;
}

// The built-in `file` handler: read the resolved path's bytes. App-free. Rejects anything that
// is not an ordinary, bounded file BEFORE opening — a directory or a special file (fifo,
// /dev/zero) would otherwise throw on read or block/OOM (istreambuf never reaches EOF).
UriResult resolve_file(const Uri &u, const std::string &base) {
  namespace fs = std::filesystem;
  const std::string full = resolve_file_path(u.path, base);
  std::error_code ec;
  const fs::file_status st = fs::status(full, ec);
  if (ec || !fs::exists(st))
    return {false, std::string(), full, "ari: cannot open file '" + full + "'"};
  if (fs::is_directory(st))
    return {false, std::string(), full, "ari: '" + full + "' is a directory, not a file"};
  if (!fs::is_regular_file(st))
    return {false, std::string(), full, "ari: '" + full + "' is not a regular file"};
  std::ifstream in(full, std::ios::binary);
  if (!in)
    return {false, std::string(), full, "ari: cannot open file '" + full + "'"};
  // Bound the READ, not a pre-read stat: the byte count is enforced here, so a file that
  // under-reports its size (procfs), grows under a TOCTOU, or streams past its stat size cannot
  // exceed the cap. Read up to the (configurable) cap; one more available byte means it is too big.
  // A cap of 0 means unlimited.
  const std::size_t cap = resolve_file_byte_cap();
  std::string content;
  char buf[64 * 1024];
  while (in) {
    in.read(buf, sizeof(buf));
    const std::streamsize got = in.gcount();
    if (got <= 0)
      break;
    if (cap != 0 && content.size() + static_cast<std::size_t>(got) > cap)
      return {false, std::string(), full,
              "ari: file '" + full + "' exceeds the " + std::to_string(cap) + "-byte read cap"};
    content.append(buf, static_cast<std::size_t>(got));
  }
  if (in.bad())
    return {false, std::string(), full, "ari: read error on file '" + full + "'"};
  return {true, std::move(content), full, std::string()};
}

// §13.10 the built-in `file` WRITER: store `content` at the resolved path ATOMICALLY — write a
// fresh temp IN THE SAME DIRECTORY (so the rename stays on one filesystem and is atomic) then
// rename over the target, so a reader or a crash never observes a half-written file. App-free.
StoreResult store_file(const Uri &u, const std::string &content, const std::string &base) {
  namespace fs = std::filesystem;
  const std::string full = resolve_file_path(u.path, base);
  const fs::path target(full);
  // Reject oversize UP FRONT so a save fails loudly, rather than writing a file the read cap could
  // never load back. The store cap is separately configurable (0 = unlimited); default 16 MiB.
  if (const std::size_t cap = store_file_byte_cap(); cap != 0 && content.size() > cap)
    return {false, full,
            "ari: cannot store to '" + full + "': content exceeds the " + std::to_string(cap) +
                "-byte store cap"};
  std::error_code ec;
  if (fs::is_directory(target, ec))
    return {false, full, "ari: cannot store to '" + full + "': it is a directory"};
  const fs::path dir = target.has_parent_path() ? target.parent_path() : fs::path(".");
  std::string werr;
  const std::string tmp = write_new_file(dir, ".ari-store-", ".tmp", content, werr);
  if (tmp.empty())
    return {false, full, "ari: cannot store to '" + full + "': " + werr};
  fs::rename(tmp, target, ec); // atomic replace on the same filesystem
  if (ec) {
    std::error_code rmec;
    fs::remove(tmp, rmec);
    return {false, full, "ari: cannot store to '" + full + "': " + ec.message()};
  }
  return {true, full, std::string()};
}

} // namespace

Uri parse_uri(const std::string &s) {
  Uri u;
  u.raw = s;
  std::string rest = s;
  const std::size_t sep = s.find("://");
  if (sep != std::string::npos) {
    u.scheme = to_lower(s.substr(0, sep));
    rest = s.substr(sep + 3);
  } else {
    u.scheme = "file"; // a bare / relative path is a file reference (§13.1)
  }
  // A '?' begins a query only for schemes that use one (state/http/custom). For `file`
  // (including the bare-path fallback), '?' is a legal filename byte on POSIX — splitting it
  // off would silently truncate the path to a different (or missing) file — so never split it.
  if (u.scheme != "file") {
    const std::size_t q = rest.find('?');
    if (q != std::string::npos) {
      u.query = rest.substr(q + 1);
      u.path = rest.substr(0, q);
      return u;
    }
  }
  u.path = rest;
  return u;
}

std::string resolve_file_path(const std::string &path, const std::string &base) {
  namespace fs = std::filesystem;
  fs::path p(path);
  // An absolute path stands alone. A relative path resolves against `base` — the enclosing
  // fragment's DIRECTORY — when we know it; when we do NOT (base is empty because the document
  // was loaded from a string, not a file, so there is no source location), a relative path
  // anchors to the process's current working directory instead of being left dangling.
  fs::path full;
  if (p.is_absolute()) {
    full = p;
  } else if (!base.empty()) {
    full = fs::path(base) / p;
  } else {
    std::error_code cwd_ec;
    const fs::path cwd = fs::current_path(cwd_ec);
    full = cwd_ec ? p : (cwd / p); // CWD unavailable (rare) → best-effort lexical below
  }
  std::error_code ec;
  const fs::path canon = fs::weakly_canonical(full, ec); // normalizes `..`, resolves symlinks
  return ec ? full.lexically_normal().string() : canon.string();
}

void register_uri_handler(const std::string &scheme, UriHandler handler) {
  if (scheme.empty() || !handler)
    return;
  std::lock_guard<std::mutex> lock(uri_mutex());
  uri_registry()[to_lower(scheme)] = std::move(handler);
}

void unregister_uri_handler(const std::string &scheme) {
  std::lock_guard<std::mutex> lock(uri_mutex());
  uri_registry().erase(to_lower(scheme));
}

bool has_uri_handler(const std::string &scheme) {
  const std::string s = to_lower(scheme);
  if (s == "file")
    return true; // built-in
  std::lock_guard<std::mutex> lock(uri_mutex());
  return uri_registry().find(s) != uri_registry().end();
}

UriResult resolve(const std::string &uri, const std::string &base) {
  // A handler is host/registered code (state, http, …) that may throw; the built-in file
  // handler must not, but a filesystem edge case could. Contain every failure here so this
  // function honours its contract — always return a UriResult, never propagate an exception.
  try {
    const Uri u = parse_uri(uri);
    // A registered handler wins (so a host may override "file" too); else the built-in file
    // scheme; else the scheme is unresolvable here (e.g. state/http with no handler installed).
    UriHandler h;
    {
      std::lock_guard<std::mutex> lock(uri_mutex());
      const auto it = uri_registry().find(u.scheme);
      if (it != uri_registry().end())
        h = it->second;
    }
    UriResult r;
    if (h)
      r = h(u, base);
    else if (u.scheme == "file")
      r = resolve_file(u, base);
    else
      return {false, std::string(), std::string(),
              "ari: no handler for URI scheme '" + u.scheme +
                  "' (register one via register_uri_handler)"};
    // Guarantee the caller's invariant: a successful result has a non-empty canonical identity
    // (its dedup / cycle-guard key). A handler that neglects to set one falls back to the raw
    // URI — stable enough to dedup exact repeats and distinguish distinct references.
    if (r.ok && r.canonical.empty())
      r.canonical = uri;
    return r;
  } catch (const std::exception &e) {
    return {false, std::string(), std::string(),
            "ari: URI resolution failed for '" + uri + "': " + e.what()};
  } catch (...) {
    return {false, std::string(), std::string(),
            "ari: URI resolution failed for '" + uri + "' (unknown error)"};
  }
}

// --- §13.10 the write side ---------------------------------------------------------------------

void register_uri_store_handler(const std::string &scheme, UriStoreHandler handler) {
  if (scheme.empty() || !handler)
    return;
  std::lock_guard<std::mutex> lock(uri_mutex());
  uri_store_registry()[to_lower(scheme)] = std::move(handler);
}

void unregister_uri_store_handler(const std::string &scheme) {
  std::lock_guard<std::mutex> lock(uri_mutex());
  uri_store_registry().erase(to_lower(scheme));
}

bool has_uri_store_handler(const std::string &scheme) {
  const std::string s = to_lower(scheme);
  if (s == "file")
    return true; // built-in
  std::lock_guard<std::mutex> lock(uri_mutex());
  return uri_store_registry().find(s) != uri_store_registry().end();
}

StoreResult store(const std::string &uri, const std::string &content, const std::string &base) {
  // Contain every failure here — like resolve(), store() always returns a StoreResult, never throws
  // (a registered writer is host code that may).
  try {
    const Uri u = parse_uri(uri);
    UriStoreHandler h;
    {
      std::lock_guard<std::mutex> lock(uri_mutex());
      const auto it = uri_store_registry().find(u.scheme);
      if (it != uri_store_registry().end())
        h = it->second;
    }
    StoreResult r;
    if (h)
      r = h(u, content, base);
    else if (u.scheme == "file")
      r = store_file(u, content, base);
    else
      return {false, std::string(),
              "ari: no store handler for URI scheme '" + u.scheme +
                  "' (register one via register_uri_store_handler)"};
    if (r.ok && r.canonical.empty())
      r.canonical = uri;
    return r;
  } catch (const std::exception &e) {
    return {false, std::string(), std::string("ari: store failed for '") + uri + "': " + e.what()};
  } catch (...) {
    return {false, std::string(), "ari: store failed for '" + uri + "' (unknown error)"};
  }
}

// --- §13.10 configurable file byte caps (read + store) ------------------------------------------

std::size_t resolve_file_byte_cap() { return resolve_cap_cell().load(std::memory_order_relaxed); }
void set_resolve_file_byte_cap(std::size_t bytes) {
  resolve_cap_cell().store(bytes, std::memory_order_relaxed);
}
std::size_t store_file_byte_cap() { return store_cap_cell().load(std::memory_order_relaxed); }
void set_store_file_byte_cap(std::size_t bytes) {
  store_cap_cell().store(bytes, std::memory_order_relaxed);
}

// --- the temp-file bridge (§13.4): a resolved URI as a local path a reader can open ----------

ResolvedFile &ResolvedFile::operator=(ResolvedFile &&other) noexcept {
  if (this != &other) {
    // Drop any temp file we currently own before taking over `other`'s state.
    if (is_temp_ && !path.empty()) {
      std::error_code ec;
      std::filesystem::remove(path, ec);
    }
    ok = other.ok;
    path = std::move(other.path);
    error = std::move(other.error);
    is_temp_ = other.is_temp_;
    other.ok = false;
    other.is_temp_ = false;
    other.path.clear();
  }
  return *this;
}

ResolvedFile::~ResolvedFile() {
  if (is_temp_ && !path.empty()) {
    std::error_code ec;
    std::filesystem::remove(path, ec); // best-effort; never throw from a destructor
  }
}

ResolvedFile resolve_to_file(const std::string &uri, const std::string &base) {
  namespace fs = std::filesystem;
  ResolvedFile rf;
  const Uri u = parse_uri(uri);
  // file:// (and a bare path) with NO registered override: the reader opens the resolved path in
  // place — no fetch, no copy. If a host has OVERRIDDEN the file scheme (a sandbox/remap/virtual
  // FS handler), fall through so its bytes go through the temp bridge like any other scheme —
  // matching what resolve() does for import:/load:.
  if (u.scheme == "file" && !file_scheme_overridden()) {
    rf.ok = true;
    rf.path = resolve_file_path(u.path, base);
    rf.is_temp_ = false;
    return rf;
  }
  // Any other scheme (or an overridden file): fetch the bytes, then spill them to a temp file the
  // reader can open. Keep the source's extension so an extension-keyed reader (geometry_file_io)
  // still dispatches.
  const UriResult r = resolve(uri, base);
  if (!r.ok) {
    rf.error = r.error;
    return rf;
  }
  // Extension from the FINAL path segment only: a '.' earlier in the path — a dotted directory, or
  // (for a non-file URI, where u.path includes the host) the host's TLD dot — must not leak a '/'
  // into the temp name and break the open.
  std::string ext;
  {
    const std::size_t slash = u.path.find_last_of('/');
    const std::size_t seg = (slash == std::string::npos) ? 0 : slash + 1;
    if (const std::size_t dot = u.path.find_last_of('.'); dot != std::string::npos && dot > seg)
      ext = u.path.substr(dot);
  }
  std::error_code ec;
  const fs::path tdir = fs::temp_directory_path(ec);
  std::string werr;
  const std::string path =
      write_new_file(ec ? fs::path(".") : tdir, "ari-src-", ext, r.content, werr);
  if (path.empty()) {
    rf.error = "ari: " + werr + " for '" + uri + "'";
    return rf;
  }
  rf.ok = true;
  rf.path = path;
  rf.is_temp_ = true;
  return rf;
}

} // namespace ariadne
} // namespace cvc
