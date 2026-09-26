// Ariadne — the shared URI resolver (roadmap §13). See uri.h. Backend-free and app-free for
// the built-in `file` scheme; other schemes are pluggable handlers. No yaml dependency — this
// resolves bytes; the loader parses them.

#include <algorithm>
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

// The built-in `file` handler: read the resolved path's bytes. App-free.
UriResult resolve_file(const Uri &u, const std::string &base) {
  const std::string full = resolve_file_path(u.path, base);
  std::ifstream in(full, std::ios::binary);
  if (!in)
    return {false, std::string(), full, "ari: cannot open file '" + full + "'"};
  std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
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
  const std::size_t q = rest.find('?');
  if (q != std::string::npos) {
    u.query = rest.substr(q + 1);
    u.path = rest.substr(0, q);
  } else {
    u.path = rest;
  }
  return u;
}

std::string resolve_file_path(const std::string &path, const std::string &base) {
  namespace fs = std::filesystem;
  fs::path p(path);
  // `base` is the enclosing fragment's DIRECTORY (empty at the top level). A relative path
  // resolves against it; an absolute path stands alone.
  fs::path full = (p.is_absolute() || base.empty()) ? p : (fs::path(base) / p);
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

bool has_uri_handler(const std::string &scheme) {
  const std::string s = to_lower(scheme);
  if (s == "file")
    return true; // built-in
  std::lock_guard<std::mutex> lock(uri_mutex());
  return uri_registry().find(s) != uri_registry().end();
}

UriResult resolve(const std::string &uri, const std::string &base) {
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
  if (h)
    return h(u, base);
  if (u.scheme == "file")
    return resolve_file(u, base);
  return {false, std::string(), std::string(),
          "ari: no handler for URI scheme '" + u.scheme +
              "' (register one via register_uri_handler)"};
}

} // namespace ariadne
} // namespace cvc
