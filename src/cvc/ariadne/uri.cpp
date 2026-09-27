// Ariadne — the shared URI resolver (roadmap §13). See uri.h. Backend-free and app-free for
// the built-in `file` scheme; other schemes are pluggable handlers. No yaml dependency — this
// resolves bytes; the loader parses them.

#include <algorithm>
#include <cstdint>
#include <cvc/ariadne/uri.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <mutex>
#include <string>
#include <utility>

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

std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

// An .ari fragment is DSL text; a fragment larger than this is almost certainly a mistake (or
// an attack). Cap the slurp so a giant file yields an error, never an OOM.
constexpr std::uintmax_t kMaxFileBytes = 16u * 1024u * 1024u; // 16 MiB

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
  // exceed the cap. Read up to kMaxFileBytes; one more available byte means the file is too big.
  std::string content;
  char buf[64 * 1024];
  while (in) {
    in.read(buf, sizeof(buf));
    const std::streamsize got = in.gcount();
    if (got <= 0)
      break;
    if (content.size() + static_cast<std::size_t>(got) > kMaxFileBytes)
      return {false, std::string(), full,
              "ari: file '" + full + "' exceeds the " + std::to_string(kMaxFileBytes) +
                  "-byte fragment cap"};
    content.append(buf, static_cast<std::size_t>(got));
  }
  if (in.bad())
    return {false, std::string(), full, "ari: read error on file '" + full + "'"};
  return {true, std::move(content), full, std::string()};
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

} // namespace ariadne
} // namespace cvc
