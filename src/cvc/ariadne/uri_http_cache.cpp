// Ariadne — the app-wide caching http(s):// URI handler (roadmap §13.9). See uri_http_cache.h.
//
// It sits in front of the plain handler's transport + policy (cvc::net + uri_http_detail.h) and
// adds a state-tree cache under `sys.net.http_cache.entries.<key>`:
//   - key            = cvc::sha256_hex(normalized URL) — a clean 64-hex path segment.
//   - entry          = ONE opaque blob on the entry node's data(): a version byte + 8
//   length-prefixed
//                      fields (status/etag/last_modified/content_type/effective_url/fetched_at/
//                      freshness_ttl/body). Packing everything into a single value makes a
//                      read/write one atomic node op (no torn multi-node entry) — see the
//                      CONCURRENCY note.
//   - freshness      = SOFT: now < fetched_at + freshness_ttl → serve with NO network; else a
//                      conditional GET revalidates (304 bump / 200 replace).
//   - retention      = HARD: the node's expireAt (>= freshness) + sweepExpired() (swept on access)
//                      evicts and frees the entry blob — one mechanism, the state tree's own
//                      expiry.
//   - single-flight  = concurrent requests for one URL coalesce to a single transfer.
//
// CONCURRENCY: each entry is stored as ONE opaque blob on its node's data() (never as separate
// metadata child nodes), so a read or write is a SINGLE atomic node operation under that node's own
// _mutex — a reader sees the whole old blob or the whole new blob, never a torn mix. Lifetime is
// covered by findDescendantShared(): read_entry PINS the entry node, so a concurrent sweepExpired()
// can only UNLINK it, never free it under the reader (cvc::state's owning-accessor lifetime fix).
// Writers of one key are already serialized by the single-flight `flight_mutex`, and the retention
// sweep only unlinks pinned-safe nodes. Together these remove the need for any process-wide cache
// lock: there is no `cache_mutex`. (The cache nodes are internal, so nothing re-enters a resolve
// from sweepExpired()'s `expiring`/`childChanged` signals.)
//
// Clock is UTC wall time (boost::posix_time::microsec_clock::universal_time()) — the same clock the
// node-expiry path uses, and correct for HTTP freshness (never the sim-time world_clock). A
// CREDENTIALED request (provider headers, or URL-embedded userinfo) bypasses the cache entirely.

#include "uri_http_detail.h"

#include <algorithm>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cctype>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_http.h>
#include <cvc/ariadne/uri_http_cache.h>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_blob_store.h> // cvc::sha256_hex
#include <cvc/net/http_client.h>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

namespace pt = boost::posix_time;

// PR3 moves these to sys.net.http_cache.policy.* (default_freshness_ttl / retention_ttl / a
// max-entries + max-bytes LRU budget — until then the cache is bounded only by per-entry cap and
// retention, so a flood of distinct fresh URLs can grow it; see the PR notes).
constexpr long kDefaultFreshnessSecs = 60; // fresh window when no Cache-Control and no validator
constexpr long kRetentionSecs = 3600;      // hard-eviction floor (retention >= freshness)
constexpr char kCacheRoot[] = "sys.net.http_cache.entries";

// --- URL normalization + key -------------------------------------------------------------------

// True if the URL authority carries a `user[:pass]@` userinfo component. Such a URL is credential-
// bearing, so (like a provider-injected auth header) it bypasses the shared cache.
bool url_has_userinfo(const std::string &raw) {
  const std::size_t scheme_end = raw.find("://");
  if (scheme_end == std::string::npos)
    return false;
  const std::size_t auth_start = scheme_end + 3;
  std::size_t auth_end = raw.find_first_of("/?", auth_start);
  if (auth_end == std::string::npos)
    auth_end = raw.size();
  return raw.find('@', auth_start) < auth_end;
}

// Minimal, defensible normalization for PR2: lowercase the scheme + authority, strip a default
// :80/:443, drop any #fragment; path + query stay byte-exact. Percent-encoding and query-key
// ordering are deferred to PR3 — two spellings of the same resource key differently (a duplicate
// entry), never wrong bytes. Userinfo URLs never reach here (they bypass the cache).
std::string normalize_http_url(const std::string &raw) {
  std::string s = raw;
  const std::size_t hash = s.find('#');
  if (hash != std::string::npos)
    s.erase(hash);
  const std::size_t scheme_end = s.find("://");
  if (scheme_end == std::string::npos)
    return s; // not a scheme://authority URL — hash it as-is
  for (std::size_t i = 0; i < scheme_end; ++i)
    s[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(s[i])));
  const std::string scheme = s.substr(0, scheme_end);
  const std::size_t auth_start = scheme_end + 3;
  std::size_t auth_end = s.find_first_of("/?", auth_start);
  if (auth_end == std::string::npos)
    auth_end = s.size();
  std::string authority = s.substr(auth_start, auth_end - auth_start);
  for (char &c : authority)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (scheme == "http" && authority.size() >= 3 &&
      authority.compare(authority.size() - 3, 3, ":80") == 0)
    authority.erase(authority.size() - 3);
  else if (scheme == "https" && authority.size() >= 4 &&
           authority.compare(authority.size() - 4, 4, ":443") == 0)
    authority.erase(authority.size() - 4);
  return scheme + "://" + authority + s.substr(auth_end);
}

std::string cache_key(const std::string &raw) {
  const std::string n = normalize_http_url(raw);
  return cvc::sha256_hex(reinterpret_cast<const unsigned char *>(n.data()), n.size());
}

// --- header + Cache-Control parsing ------------------------------------------------------------

// Find a response header value by case-insensitive name (from "Name: value" lines). Leading spaces
// in the value are trimmed. Leaves `out` untouched and returns false if absent (so a caller can
// seed `out` with a fallback before calling).
bool find_header(const std::vector<std::string> &headers, const char *name, std::string &out) {
  const std::size_t nlen = std::char_traits<char>::length(name);
  for (const std::string &line : headers) {
    if (line.find(':') != nlen)
      continue;
    bool eq = true;
    for (std::size_t i = 0; i < nlen; ++i)
      if (std::tolower(static_cast<unsigned char>(line[i])) !=
          std::tolower(static_cast<unsigned char>(name[i]))) {
        eq = false;
        break;
      }
    if (!eq)
      continue;
    const std::string v = line.substr(nlen + 1);
    const std::size_t s = v.find_first_not_of(' ');
    out = (s == std::string::npos) ? std::string() : v.substr(s);
    return true;
  }
  return false;
}

struct CacheControl {
  bool no_store = false;
  bool no_cache = false;
  bool has_max_age = false;
  long max_age = 0;
};

std::string trim(const std::string &s) {
  const std::size_t a = s.find_first_not_of(" \t");
  if (a == std::string::npos)
    return std::string();
  const std::size_t b = s.find_last_not_of(" \t");
  return s.substr(a, b - a + 1);
}

// Parse Cache-Control by TOKENIZING on ',' and matching each directive by its NAME (the text before
// an optional '='), not by substring — so a quoted form like no-cache="Set-Cookie" or an unrelated
// value substring cannot mis-trigger a directive.
CacheControl parse_cache_control(const std::vector<std::string> &headers) {
  CacheControl cc;
  std::string v;
  if (!find_header(headers, "Cache-Control", v))
    return cc;
  std::size_t start = 0;
  while (start <= v.size()) {
    const std::size_t comma = v.find(',', start);
    const std::string tok =
        trim(v.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
    if (!tok.empty()) {
      const std::size_t eq = tok.find('=');
      std::string name = tok.substr(0, eq);
      for (char &c : name)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      if (name == "no-store")
        cc.no_store = true;
      else if (name == "no-cache")
        cc.no_cache = true;
      else if (name == "max-age" && eq != std::string::npos) {
        std::string val = tok.substr(eq + 1);
        const std::size_t va = val.find_first_not_of(" \t\"");
        const std::size_t vb = val.find_last_not_of(" \t\"");
        if (va != std::string::npos) {
          val = val.substr(va, vb - va + 1);
          try {
            cc.max_age = std::stol(val);
            cc.has_max_age = true;
          } catch (const std::exception &) {
            // leave has_max_age false
          }
        }
      }
    }
    if (comma == std::string::npos)
      break;
    start = comma + 1;
  }
  return cc;
}

// The response's Age (seconds since the origin generated it, per an intermediary cache); 0 if
// absent or unparseable. Anchoring freshness at `now - age` keeps a shared/CDN-cached response from
// being treated as fresh for a full max-age past when WE received it.
long parse_age(const std::vector<std::string> &headers) {
  std::string v;
  if (!find_header(headers, "Age", v))
    return 0;
  try {
    const long a = std::stol(v);
    return a < 0 ? 0 : a;
  } catch (const std::exception &) {
    return 0;
  }
}

long parse_long_or(const std::string &s, long def) {
  if (s.empty())
    return def;
  try {
    return std::stol(s);
  } catch (const std::exception &) {
    return def;
  }
}

// The soft freshness window in seconds. no-cache => 0 (always revalidate); max-age wins when
// present; otherwise 0 when the response carries a validator (revalidate cheaply via a 304) or a
// small default when it carries none. (Expires-based freshness is deferred to PR3.)
long derive_freshness_ttl(const std::vector<std::string> &headers, const CacheControl &cc) {
  if (cc.no_cache)
    return 0;
  if (cc.has_max_age)
    return cc.max_age < 0 ? 0 : cc.max_age;
  std::string v;
  const bool has_validator =
      find_header(headers, "ETag", v) || find_header(headers, "Last-Modified", v);
  return has_validator ? 0 : kDefaultFreshnessSecs;
}

// --- entry read/write (single-node blob; no cache lock) ----------------------------------------

// A decoded snapshot of a cached entry (a value copy — nothing downstream holds a node pointer).
struct EntrySnapshot {
  bool present = false;
  std::string body;
  std::string etag;
  std::string last_modified;
  std::string content_type;
  std::string effective_url;
  std::string fetched_at;
  std::string freshness_ttl;
};

// The entry is one opaque blob on the node's data(): a version byte, then 8 length-prefixed fields
// (little-endian uint64 length + bytes) in a fixed order — status, etag, last_modified,
// content_type, effective_url, fetched_at, freshness_ttl, body. Encoding everything into ONE value
// makes a read/write a single atomic node op (no torn multi-node entry), which is what lets the
// cache drop its process-wide lock (see the CONCURRENCY note).
constexpr unsigned char kEntryBlobVersion = 1;

void put_field(std::string &out, const std::string &f) {
  const std::uint64_t n = f.size();
  for (int i = 0; i < 8; ++i)
    out.push_back(static_cast<char>((n >> (8 * i)) & 0xFF));
  out.append(f);
}

bool get_field(const std::string &in, std::size_t &pos, std::string &out) {
  if (pos + 8 > in.size())
    return false;
  std::uint64_t n = 0;
  for (int i = 0; i < 8; ++i)
    n |= static_cast<std::uint64_t>(static_cast<unsigned char>(in[pos + i])) << (8 * i);
  pos += 8;
  if (n > in.size() - pos)
    return false;
  out.assign(in, pos, static_cast<std::size_t>(n));
  pos += static_cast<std::size_t>(n);
  return true;
}

std::string encode_entry(long status, const std::string &etag, const std::string &last_modified,
                         const std::string &content_type, const std::string &effective_url,
                         const std::string &fetched_at_iso, long freshness_ttl,
                         const std::string &body) {
  std::string out;
  out.reserve(1 + 8 * 8 + body.size() + 128);
  out.push_back(static_cast<char>(kEntryBlobVersion));
  put_field(out, std::to_string(status));
  put_field(out, etag);
  put_field(out, last_modified);
  put_field(out, content_type);
  put_field(out, effective_url);
  put_field(out, fetched_at_iso);
  put_field(out, std::to_string(freshness_ttl));
  put_field(out, body);
  return out;
}

EntrySnapshot decode_entry(const std::string &blob) {
  EntrySnapshot snap;
  if (blob.empty() || static_cast<unsigned char>(blob[0]) != kEntryBlobVersion)
    return snap;
  std::size_t pos = 1;
  std::string status_ignored,
      freshness_str; // status is stored for fidelity; the read path ignores it
  if (!get_field(blob, pos, status_ignored) || !get_field(blob, pos, snap.etag) ||
      !get_field(blob, pos, snap.last_modified) || !get_field(blob, pos, snap.content_type) ||
      !get_field(blob, pos, snap.effective_url) || !get_field(blob, pos, snap.fetched_at) ||
      !get_field(blob, pos, freshness_str) || !get_field(blob, pos, snap.body))
    return snap; // malformed → miss
  snap.freshness_ttl = freshness_str;
  snap.present = true;
  return snap;
}

// Read the entry at `key`. findDescendantShared PINS the node so a concurrent sweepExpired() can
// only unlink it (never free it under us); its single data() blob is then decoded — an atomic,
// parent-free read that needs no cache lock. Absent / non-blob / malformed all read as a miss.
EntrySnapshot read_entry(cvc::state &entries, const std::string &key) {
  cvc::state::state_ptr e = entries.findDescendantShared(key);
  if (!e)
    return {};
  const boost::any d = e->data(); // by value — bind before casting a pointer into it
  const std::string *blob = boost::any_cast<std::string>(&d);
  if (!blob)
    return {};
  return decode_entry(*blob);
}

bool snapshot_fresh(const EntrySnapshot &s, const pt::ptime &now) {
  const long ttl = parse_long_or(s.freshness_ttl, 0);
  if (ttl <= 0 || s.fetched_at.empty())
    return false; // ttl 0 (no-cache / validator-only) → always revalidate
  pt::ptime fetched;
  try {
    fetched = pt::from_iso_string(s.fetched_at);
  } catch (const std::exception &) {
    return false;
  }
  return now < fetched + pt::seconds(ttl);
}

// Write (create or replace) the entry as a SINGLE data() blob, then set retention. One atomic
// commit per entry node; single-flight already serializes writers of the same key, so no cache lock
// is needed. `fetched_at` is the FRESHNESS anchor (receipt time minus the response Age).
void write_entry(cvc::state &entries, const std::string &key, long status, const std::string &etag,
                 const std::string &last_modified, const std::string &content_type,
                 const std::string &effective_url, const std::string &body,
                 const pt::ptime &fetched_at, long freshness_ttl) {
  // sharedChild PINS the entry node (create-or-get), so a concurrent evict/sweepExpired (e.g. a
  // store invalidation of the same key) can only unlink it, never free it under the data()/expireAt
  // below — which lock the node and walk its _parent. The entry's ancestors are the fixed,
  // never-expiring sys.net.http_cache.entries path, so pinning the entry alone is sufficient here.
  cvc::state::state_ptr e = entries.sharedChild(key);
  e->data(boost::any(encode_entry(status, etag, last_modified, content_type, effective_url,
                                  pt::to_iso_string(fetched_at), freshness_ttl, body)));
  const long retention = std::max(kRetentionSecs, freshness_ttl);
  e->expireAt(fetched_at + pt::seconds(retention));
}

// Force-expire + sweep the entry for `key` (cache invalidation). The pin keeps the node alive
// across expireAt; the sweep is safe against a concurrent pinned reader.
void evict(cvc::state &entries, const std::string &key, const pt::ptime &now) {
  if (cvc::state::state_ptr e = entries.findDescendantShared(key))
    e->expireAt(now - pt::seconds(1));
  entries.sweepExpired();
}

// --- single-flight -----------------------------------------------------------------------------

struct Inflight {
  std::condition_variable cv;
  bool done = false;
  UriResult result;
};

std::mutex &flight_mutex() {
  static std::mutex m;
  return m;
}
std::unordered_map<std::string, std::shared_ptr<Inflight>> &flight_map() {
  static std::unordered_map<std::string, std::shared_ptr<Inflight>> m;
  return m;
}

// --- fetch + cache -----------------------------------------------------------------------------

// The leader body: do the (conditional) network fetch and update the cache; return the UriResult.
UriResult do_fetch_and_store(cvc::state &root, const std::string &key, const Uri &u,
                             const EntrySnapshot &stale, const pt::ptime &now) {
  cvc::net::HttpRequest req;
  req.url = u.raw;
  req.max_bytes = detail::kMaxHttpBytes;
  req.follow_redirects = true; // only the uncredentialed path reaches here (matches http_fetch)
  if (stale.present) {
    if (!stale.etag.empty())
      req.headers.push_back("If-None-Match: " + stale.etag);
    if (!stale.last_modified.empty())
      req.headers.push_back("If-Modified-Since: " + stale.last_modified);
  }

  const cvc::net::HttpResponse r = cvc::net::send(req); // blocking send; holds no cache lock
  if (!r.ok)
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + u.raw + "' failed: " + r.error};

  // 304 Not Modified: the stale body is still valid. Honor the revalidation response's own
  // directives, merge any validators it carries (falling back to the stale ones), and serve the
  // cached body. We only send conditional headers when we had a body, so stale.present holds here.
  if (r.status == 304 && stale.present) {
    const CacheControl cc = parse_cache_control(r.headers);
    if (cc.no_store) {
      evict(root(kCacheRoot), key, now); // must not retain; serve this body once
      return {true, stale.body, stale.effective_url, std::string()};
    }
    std::string etag = stale.etag, last_modified = stale.last_modified,
                content_type = stale.content_type;
    find_header(r.headers, "ETag", etag); // a 304 may refresh the validators
    find_header(r.headers, "Last-Modified", last_modified);
    find_header(r.headers, "Content-Type", content_type);
    const long ttl = cc.no_cache      ? 0
                     : cc.has_max_age ? std::max<long>(0, cc.max_age)
                                      : parse_long_or(stale.freshness_ttl, kDefaultFreshnessSecs);
    const long age = parse_age(r.headers);
    write_entry(root(kCacheRoot), key, 200, etag, last_modified, content_type, stale.effective_url,
                stale.body, now - pt::seconds(age), ttl);
    return {true, stale.body, stale.effective_url, std::string()};
  }

  // 200 OK: replace body + validators + TTL (unless no-store), and serve the new body.
  if (r.status == 200) {
    const CacheControl cc = parse_cache_control(r.headers);
    if (!cc.no_store) {
      std::string etag, last_modified, content_type;
      find_header(r.headers, "ETag", etag);
      find_header(r.headers, "Last-Modified", last_modified);
      find_header(r.headers, "Content-Type", content_type);
      const long age = parse_age(r.headers);
      write_entry(root(kCacheRoot), key, 200, etag, last_modified, content_type, r.canonical_url,
                  r.body, now - pt::seconds(age), derive_freshness_ttl(r.headers, cc));
    }
    return {true, r.body, r.canonical_url, std::string()};
  }

  // Any other status (>= 400, or an unsolicited 304 with no usable stale body) → the plain mapper,
  // which now treats a 304 as an error and a >= 400 as an error (follow == true, so no 3xx here).
  return detail::map_http_fetch_result(u.raw, /*follow=*/true, r);
}

// single-flight wrapper: the first caller for a key becomes the leader (does the fetch + store);
// the rest wait and receive the leader's result. Coalesces only the network-bound paths (a fresh
// hit returns before this is called). This per-key serialization is the ONLY write coordination the
// cache needs — the entry write itself is a single atomic node op (see the CONCURRENCY note).
UriResult fetch_and_cache(cvc::state &root, const std::string &key, const Uri &u,
                          const EntrySnapshot &stale, const pt::ptime &now) {
  std::shared_ptr<Inflight> mine;
  {
    std::unique_lock<std::mutex> lk(flight_mutex());
    auto it = flight_map().find(key);
    if (it != flight_map().end()) {
      std::shared_ptr<Inflight> in = it->second; // follower: wait for the leader
      in->cv.wait(lk, [&] { return in->done; });
      return in->result;
    }
    mine = std::make_shared<Inflight>();
    flight_map()[key] = mine;
  } // release the flight lock before the (blocking) network fetch

  UriResult result;
  try {
    result = do_fetch_and_store(root, key, u, stale, now);
  } catch (const std::exception &e) {
    result = {false, std::string(), std::string(),
              "ari: http fetch of '" + u.raw + "' failed: " + e.what()};
  } catch (...) {
    result = {false, std::string(), std::string(),
              "ari: http fetch of '" + u.raw + "' failed: unknown error"};
  }

  {
    std::lock_guard<std::mutex> lk(flight_mutex());
    mine->result = result;
    mine->done = true;
    flight_map().erase(key);
  }
  mine->cv.notify_all(); // wake followers even on error (result carries the error)
  return result;
}

UriResult cached_http_fetch(cvc::state &root, const Uri &u, const std::string & /*base*/) {
  const HttpRequestOptions opts = detail::consult_http_provider(u.raw, /*for_write=*/false);
  if (!opts.headers.empty() || url_has_userinfo(u.raw)) {
    // CREDENTIALED (provider headers or URL userinfo) → bypass the cache entirely (never serve one
    // principal's authorized body to another from a process-global cache). Plain un-followed fetch.
    cvc::net::HttpRequest req;
    req.url = u.raw;
    req.max_bytes = detail::kMaxHttpBytes;
    req.headers = opts.headers;
    req.follow_redirects = false;
    if (!opts.method.empty() && opts.method != "GET")
      req.method = opts.method;
    return detail::map_http_fetch_result(u.raw, /*follow=*/false, cvc::net::send(req));
  }

  const std::string key = cache_key(u.raw);
  const pt::ptime now = pt::microsec_clock::universal_time();
  EntrySnapshot snap;
  {
    cvc::state &entries = root(kCacheRoot);
    entries.sweepExpired();          // access-time eviction (frees expired blobs)
    snap = read_entry(entries, key); // read_entry PINS the node — safe against the sweep above
  }
  if (snap.present && snapshot_fresh(snap, now))
    return {true, snap.body, snap.effective_url, std::string()}; // FRESH: no network

  // stale (conditional GET) or miss (unconditional) — coalesced.
  return fetch_and_cache(root, key, u, snap, now);
}

StoreResult invalidating_store(cvc::state &root, const Uri &u, const std::string &content,
                               const std::string &base) {
  const StoreResult r = detail::http_store_uncached(u, content, base);
  if (r.ok)
    evict(root(kCacheRoot), cache_key(u.raw), pt::microsec_clock::universal_time());
  return r;
}

} // namespace

void register_cached_http_uri_handler(cvc::app &app) {
  if (!cvc::net::have_http_backend())
    return; // no transport compiled — leave the scheme unresolved
  cvc::state *root = &cvc::state::instance(app);
  register_uri_handler("http", [root](const Uri &u, const std::string &b) {
    return cached_http_fetch(*root, u, b);
  });
  register_uri_handler("https", [root](const Uri &u, const std::string &b) {
    return cached_http_fetch(*root, u, b);
  });
  register_uri_store_handler("http",
                             [root](const Uri &u, const std::string &c, const std::string &b) {
                               return invalidating_store(*root, u, c, b);
                             });
  register_uri_store_handler("https",
                             [root](const Uri &u, const std::string &c, const std::string &b) {
                               return invalidating_store(*root, u, c, b);
                             });
}

void unregister_cached_http_uri_handler() {
  unregister_uri_handler("http");
  unregister_uri_handler("https");
  unregister_uri_store_handler("http");
  unregister_uri_store_handler("https");
}

} // namespace ariadne
} // namespace cvc
