// Ariadne — the app-wide caching http(s):// URI handler (roadmap §13.9). See uri_http_cache.h.
//
// It sits in front of the plain handler's transport + policy (cvc::net + uri_http_detail.h) and
// adds a state-tree cache under `sys.net.http_cache.entries.<key>`:
//   - key            = cvc::sha256_hex(normalized URL) — a clean 64-hex path segment.
//   - entry          = ONE opaque blob on the entry node's data(): a version byte + 8
//                      length-prefixed fields (status/etag/last_modified/content_type/
//                      effective_url/fetched_at/freshness_ttl/body). Packing everything into a
//                      single value makes a read/write one atomic node op (no torn multi-node
//                      entry) — see the CONCURRENCY note. The node's value() is the blob's size in
//                      bytes, so the budget can be counted without copying a blob.
//   - freshness      = SOFT: now < fetched_at + freshness_ttl → serve with NO network; else a
//                      conditional GET revalidates (304 bump / 200 replace).
//   - retention      = HARD: the node's expireAt (>= freshness) + sweepExpired() (swept on access)
//                      evicts and frees the entry blob — one mechanism, the state tree's own
//                      expiry.
//   - budget         = policy.max_bytes: after each store the least recently used entries go until
//                      the rest fit (the "book" below keeps each entry's size and last use).
//   - single-flight  = concurrent requests for one URL coalesce to a single transfer.
//
// CONCURRENCY: each entry is stored as ONE opaque blob on its node's data() (never as separate
// metadata child nodes), so a read or write is a SINGLE atomic node operation under that node's own
// _mutex — a reader sees the whole old blob or the whole new blob, never a torn mix. Lifetime is
// covered by findDescendantShared(): read_entry PINS the entry node, so a concurrent sweepExpired()
// or removeChild() can only UNLINK it, never free it under the reader (cvc::state's owning-accessor
// lifetime fix). Writers of one key are already serialized by the single-flight `flight_mutex`.
// The book (sizes, last use, counters) has its own per-app mutex, held only for bookkeeping and
// never while the tree signals: entries are removed, and the stats written, after it is released.
// A removal can therefore race a store of the same key; the book is reconciled with the tree's
// entries after every store and after a sweep removed something, so such a race costs at most a
// re-fetch and a stale count until then. (The cache nodes are internal, so nothing re-enters a
// resolve from the tree's `expiring`/`childChanged` signals.)
//
// Clock is UTC wall time (boost::posix_time::microsec_clock::universal_time()) — the same clock the
// node-expiry path uses, and correct for HTTP freshness (never the sim-time world_clock). A
// CREDENTIALED request (provider headers, or URL-embedded userinfo) bypasses the cache entirely.

#include "uri_http_detail.h"

#include <algorithm>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/signals2/connection.hpp>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_http.h>
#include <cvc/ariadne/uri_http_cache.h>
#include <cvc/core/app.h>
#include <cvc/net/http_client.h>
#include <cvc/net/net_policy.h>
#include <cvc/state/state.h>
#include <cvc/state/state_blob_store.h> // cvc::sha256_hex
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

namespace pt = boost::posix_time;

constexpr std::uint64_t kMiB = 1024u * 1024u;

// The policy's defaults (sys.net.http_cache.policy.*, seeded at registration where unset). A
// browser's tab keeps the cache in its wasm heap, beside the browser's own HTTP cache and the
// decoded pictures, on a phone too, so its budget is smaller.
#ifdef __EMSCRIPTEN__
constexpr std::uint64_t kDefaultMaxBytes = 48 * kMiB;
#else
constexpr std::uint64_t kDefaultMaxBytes = 256 * kMiB;
#endif
constexpr std::uint64_t kDefaultMaxEntryBytes = detail::kMaxHttpBytes; // 64 MiB, the response cap
constexpr long kDefaultFreshnessSecs = 60; // fresh window when no Cache-Control and no validator
constexpr long kDefaultRetentionSecs = 3600; // hard-eviction floor (retention >= freshness)
// A policy's seconds are clamped to this (ten years), so an expiry stays a representable time.
constexpr double kMaxPolicySecs = 10.0 * 365 * 24 * 3600;

constexpr char kWatchRoot[] = "sys.net.http_cache";
constexpr char kCacheRoot[] = "sys.net.http_cache.entries";
constexpr char kPolicyRoot[] = "sys.net.http_cache.policy.";
constexpr char kStatsRoot[] = "sys.net.http_cache.stats.";
constexpr char kClearKey[] = "sys.net.http_cache.clear";

std::string lower(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

std::string trim(const std::string &s) {
  const std::size_t a = s.find_first_not_of(" \t");
  if (a == std::string::npos)
    return std::string();
  const std::size_t b = s.find_last_not_of(" \t");
  return s.substr(a, b - a + 1);
}

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

// --- the policy (sys.net.http_cache.policy.*) --------------------------------------------------

struct Policy {
  bool enabled = true;
  std::uint64_t max_bytes = kDefaultMaxBytes;
  std::uint64_t max_entry_bytes = kDefaultMaxEntryBytes;
  long freshness_secs = kDefaultFreshnessSecs;
  long retention_secs = kDefaultRetentionSecs;
};

std::string policy_value(cvc::state &root, const char *name) {
  const cvc::state::state_ptr n = root.findDescendantShared(std::string(kPolicyRoot) + name);
  return n ? n->value() : std::string();
}

// A number, as a slider writes one ("256", "256.000"); nullopt for anything else.
std::optional<double> parse_number(const std::string &v) {
  const std::string t = trim(v);
  if (t.empty())
    return std::nullopt;
  char *end = nullptr;
  const double x = std::strtod(t.c_str(), &end);
  if (end == t.c_str() || !std::isfinite(x))
    return std::nullopt;
  return x;
}

// A policy number: a negative one is 0, an unreadable or unset one `def`.
double policy_number(cvc::state &root, const char *name, double def, double hi) {
  const std::optional<double> x = parse_number(policy_value(root, name));
  if (!x)
    return def;
  return std::min(std::max(*x, 0.0), hi);
}

Policy read_policy(cvc::state &root) {
  Policy p;
  const std::string on = lower(trim(policy_value(root, "enabled")));
  p.enabled = on.empty() || !(on == "0" || on == "false" || on == "no" || on == "off");
  p.max_bytes = static_cast<std::uint64_t>(
      policy_number(root, "max_bytes", static_cast<double>(kDefaultMaxBytes), 1e18));
  p.max_entry_bytes = static_cast<std::uint64_t>(
      policy_number(root, "max_entry_bytes", static_cast<double>(kDefaultMaxEntryBytes), 1e18));
  p.freshness_secs = static_cast<long>(
      policy_number(root, "freshness_secs", kDefaultFreshnessSecs, kMaxPolicySecs));
  p.retention_secs = static_cast<long>(
      policy_number(root, "retention_secs", kDefaultRetentionSecs, kMaxPolicySecs));
  return p;
}

// "256" for a whole number of MiB, else to a tenth.
std::string mib_number(std::uint64_t bytes) {
  char buf[64];
  if (bytes % kMiB == 0)
    std::snprintf(buf, sizeof buf, "%llu", static_cast<unsigned long long>(bytes / kMiB));
  else
    std::snprintf(buf, sizeof buf, "%.1f", static_cast<double>(bytes) / kMiB);
  return buf;
}

// A size for a person: "12.3 MiB", "640.0 KiB", "0 MiB".
std::string size_text(std::uint64_t bytes) {
  char buf[64];
  if (bytes != 0 && bytes < kMiB)
    std::snprintf(buf, sizeof buf, "%.1f KiB", static_cast<double>(bytes) / 1024.0);
  else
    std::snprintf(buf, sizeof buf, "%s MiB", mib_number(bytes).c_str());
  return buf;
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
// present; otherwise 0 when the response carries a validator (revalidate cheaply via a 304) or the
// policy's freshness_secs when it carries none. (Expires-based freshness is deferred to PR3.)
long derive_freshness_ttl(const std::vector<std::string> &headers, const CacheControl &cc,
                          const Policy &pol) {
  if (cc.no_cache)
    return 0;
  if (cc.has_max_age)
    return cc.max_age < 0 ? 0 : cc.max_age;
  std::string v;
  const bool has_validator =
      find_header(headers, "ETag", v) || find_header(headers, "Last-Modified", v);
  return has_validator ? 0 : pol.freshness_secs;
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

// Write (create or replace) the entry as a SINGLE data() blob, then its size (the node's value)
// and retention. One atomic commit of the blob per entry node; single-flight already serializes
// writers of the same key, so no cache lock is needed. `fetched_at` is the FRESHNESS anchor
// (receipt time minus the response Age).
void write_entry(cvc::state &entries, const std::string &key, std::string blob,
                 const pt::ptime &fetched_at, long retention) {
  // sharedChild PINS the entry node (create-or-get), so a concurrent evict/sweepExpired (e.g. a
  // store invalidation of the same key) can only unlink it, never free it under the data()/expireAt
  // below — which lock the node and walk its _parent. The entry's ancestors are the fixed,
  // never-expiring sys.net.http_cache.entries path, so pinning the entry alone is sufficient here.
  cvc::state::state_ptr e = entries.sharedChild(key);
  const std::string size = std::to_string(blob.size());
  e->data(boost::any(std::move(blob)));
  e->value(size);
  e->expireAt(fetched_at + pt::seconds(retention));
}

// --- the book: each entry's size and last use, and the counters --------------------------------

// The cache's bookkeeping for an app. Its mutex is held only to read or change these fields: never
// while the tree is written (which signals), nor across a request.
struct cache_book {
  static constexpr const char *app_slot = "cvc.ariadne.http-cache";
  struct item {
    std::uint64_t bytes = 0; // the entry blob's size
    std::uint64_t used = 0;  // the tick of its last store or hit (least recently used goes first)
  };
  std::mutex m;
  std::map<std::string, item> items; // by key, the entries the budget counts
  std::uint64_t bytes = 0;           // the sum of items' bytes
  std::uint64_t tick = 0;
  std::uint64_t hits = 0, misses = 0, revalidated = 0, evicted = 0;
  std::uint64_t version = 0; // bumped at each change, so the stats mirror knows it is current
  bool registered = false;   // register_cached_http_uri_handler, not unregistered since
  std::shared_ptr<const cvc::net::NetPolicy> policy; // what the registration checks against
  bool watching = false;
  // The MiB forms of the two size keys as last synced (sync_mib), to tell which form changed.
  std::string max_bytes_seen, max_mib_seen, entry_bytes_seen, entry_mib_seen;
  boost::signals2::scoped_connection watch; // sys.net.http_cache's childChanged
  // Held weakly by a started request's reply, which may come after the app has gone.
  std::shared_ptr<int> life = std::make_shared<int>(0);
};

cache_book &book_of(cvc::app &app) { return app.slot<cache_book>(); }

cvc::state::state_ptr entries_of(cvc::state &root) { return root.sharedChild(kCacheRoot); }

HttpCacheStats stats_locked(const cache_book &b) {
  HttpCacheStats s;
  s.entries = b.items.size();
  s.bytes = b.bytes;
  s.hits = b.hits;
  s.misses = b.misses;
  s.revalidated = b.revalidated;
  s.evicted = b.evicted;
  return s;
}

void set_key(cvc::state &root, const std::string &path, const std::string &v) {
  root.sharedChild(path)->value(v); // an unchanged value writes (and signals) nothing
}

// Mirror the book into sys.net.http_cache.stats.*, until what was written is the book's latest (a
// concurrent change re-mirrors; a few rounds at most, the next change mirrors again).
void mirror(cvc::app &app, cvc::state &root) {
  cache_book &b = book_of(app);
  for (int round = 0; round < 4; ++round) {
    HttpCacheStats s;
    std::uint64_t version = 0;
    {
      std::lock_guard<std::mutex> lock(b.m);
      s = stats_locked(b);
      version = b.version;
    }
    const std::uint64_t budget = read_policy(root).max_bytes;
    const std::string at = kStatsRoot;
    set_key(root, at + "entries", std::to_string(s.entries));
    set_key(root, at + "bytes", std::to_string(s.bytes));
    set_key(root, at + "hits", std::to_string(s.hits));
    set_key(root, at + "misses", std::to_string(s.misses));
    set_key(root, at + "revalidated", std::to_string(s.revalidated));
    set_key(root, at + "evicted", std::to_string(s.evicted));
    const double fill = budget > 0 ? std::min(1.0, static_cast<double>(s.bytes) / budget)
                                   : (s.bytes > 0 ? 1.0 : 0.0);
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.4f", fill);
    set_key(root, at + "fill", buf);
    set_key(root, at + "used", size_text(s.bytes) + " of " + size_text(budget));
    std::lock_guard<std::mutex> lock(b.m);
    if (b.version == version)
      return;
  }
}

// Under b.m: drop the items whose entry node is gone (a retention sweep, a removal that raced a
// store) and adopt entry nodes the book has not seen that carry their size (an entry stored before
// a re-registration). A node with no size yet is a store in progress, which books itself. Returns
// how many items it dropped.
std::size_t reconcile_locked(cache_book &b, cvc::state &entries) {
  const std::vector<std::string> names = entries.childNames(); // in name order
  std::size_t dropped = 0;
  bool changed = false;
  for (auto it = b.items.begin(); it != b.items.end();) {
    if (std::binary_search(names.begin(), names.end(), it->first)) {
      ++it;
      continue;
    }
    b.bytes -= it->second.bytes;
    it = b.items.erase(it);
    ++dropped;
    changed = true;
  }
  for (const std::string &name : names) {
    if (b.items.count(name))
      continue;
    const cvc::state::state_ptr e = entries.findDescendantShared(name);
    const std::optional<double> size = e ? parse_number(e->value()) : std::nullopt;
    if (!size || *size < 0)
      continue;
    const std::uint64_t n = static_cast<std::uint64_t>(*size);
    b.items[name] = cache_book::item{n, ++b.tick};
    b.bytes += n;
    changed = true;
  }
  if (changed)
    ++b.version;
  return dropped;
}

// Hold the cache to its budget: drop the least recently used entries (never `keep`, the one just
// stored) until the rest fit, after reconciling the book with the tree. Then mirror the stats.
void enforce(cvc::app &app, cvc::state &root, const Policy &pol, const std::string &keep) {
  cache_book &b = book_of(app);
  const cvc::state::state_ptr entries = entries_of(root);
  std::vector<std::string> victims;
  {
    std::lock_guard<std::mutex> lock(b.m);
    b.evicted += reconcile_locked(b, *entries);
    if (b.bytes > pol.max_bytes) {
      std::vector<std::pair<std::uint64_t, std::string>> order; // (last use, key), oldest first
      order.reserve(b.items.size());
      for (const auto &[key, it] : b.items)
        if (key != keep)
          order.emplace_back(it.used, key);
      std::sort(order.begin(), order.end());
      for (const auto &[used, key] : order) {
        if (b.bytes <= pol.max_bytes)
          break;
        const auto it = b.items.find(key);
        b.bytes -= it->second.bytes;
        b.items.erase(it);
        victims.push_back(key);
      }
      b.evicted += victims.size();
      ++b.version;
    }
  }
  for (const std::string &key : victims)
    entries->removeChild(key);
  mirror(app, root);
}

// Book a stored entry (its size, and now as its last use).
void book_store(cvc::app &app, const std::string &key, std::uint64_t size) {
  cache_book &b = book_of(app);
  std::lock_guard<std::mutex> lock(b.m);
  cache_book::item &it = b.items[key];
  b.bytes -= it.bytes;
  it.bytes = size;
  it.used = ++b.tick;
  b.bytes += size;
  ++b.version;
}

enum class Count { hit, miss, revalidated };

// Count a read (a hit also makes `key` the most recently used), then mirror the stats.
void count(cvc::app &app, cvc::state &root, Count c, const std::string &key) {
  cache_book &b = book_of(app);
  {
    std::lock_guard<std::mutex> lock(b.m);
    if (c == Count::hit) {
      ++b.hits;
      const auto it = b.items.find(key);
      if (it != b.items.end())
        it->second.used = ++b.tick;
    } else if (c == Count::miss) {
      ++b.misses;
    } else {
      ++b.revalidated;
    }
    ++b.version;
  }
  mirror(app, root);
}

// Drop the entry for `key` now (an invalidating store, a no-store revalidation, a replacement that
// may not be kept). Not counted as evicted. The pinned reader of a removed entry keeps its copy.
void forget(cvc::app &app, cvc::state &root, const std::string &key) {
  cache_book &b = book_of(app);
  {
    std::lock_guard<std::mutex> lock(b.m);
    const auto it = b.items.find(key);
    if (it != b.items.end()) {
      b.bytes -= it->second.bytes;
      b.items.erase(it);
    }
    ++b.version;
  }
  entries_of(root)->removeChild(key);
  mirror(app, root);
}

// Empty the cache: every entry node goes, booked or not. The counters stay.
void clear_entries(cvc::app &app, cvc::state &root) {
  cache_book &b = book_of(app);
  const cvc::state::state_ptr entries = entries_of(root);
  std::vector<std::string> names;
  {
    std::lock_guard<std::mutex> lock(b.m);
    names = entries->childNames();
    b.items.clear();
    b.bytes = 0;
    ++b.version;
  }
  for (const std::string &name : names)
    entries->removeChild(name);
  mirror(app, root);
}

// The access-time sweep (retention), then the entry for `key`.
EntrySnapshot lookup(cvc::app &app, cvc::state &root, const std::string &key) {
  const cvc::state::state_ptr entries = entries_of(root);
  if (entries->sweepExpired() > 0) { // expired entries went: the book follows
    cache_book &b = book_of(app);
    {
      std::lock_guard<std::mutex> lock(b.m);
      b.evicted += reconcile_locked(b, *entries);
    }
    mirror(app, root);
  }
  return read_entry(*entries, key); // read_entry PINS the node — safe against the sweep above
}

// Write the entry for `key` and book it, then hold the cache to its budget. False, and nothing
// written, when the body may not be kept: past policy.max_entry_bytes, or past the whole budget.
bool store_entry(cvc::app &app, cvc::state &root, const std::string &key, const std::string &etag,
                 const std::string &last_modified, const std::string &content_type,
                 const std::string &effective_url, const std::string &body,
                 const pt::ptime &fetched_at, long freshness_ttl, const Policy &pol) {
  if (body.size() > pol.max_entry_bytes)
    return false;
  std::string blob = encode_entry(200, etag, last_modified, content_type, effective_url,
                                  pt::to_iso_string(fetched_at), freshness_ttl, body);
  if (blob.size() > pol.max_bytes)
    return false;
  const std::uint64_t size = blob.size();
  write_entry(*entries_of(root), key, std::move(blob), fetched_at,
              std::max(pol.retention_secs, freshness_ttl));
  book_store(app, key, size);
  enforce(app, root, pol, key);
  return true;
}

// --- the policy keys' watcher ------------------------------------------------------------------

// The byte and MiB forms of one size key follow each other: whichever changed since the last sync
// sets the other (the byte form, when both did). The MiB form is whole MiB, rounded, for a slider;
// a byte value that is not whole MiB stays as it is until the MiB form is moved.
void sync_pair(cvc::app &app, cvc::state &root, const char *bytes_key, const char *mib_key,
               std::string cache_book::*bytes_seen, std::string cache_book::*mib_seen) {
  cache_book &b = book_of(app);
  const std::string bytes = policy_value(root, bytes_key), mib = policy_value(root, mib_key);
  std::string to_bytes, to_mib;
  {
    std::lock_guard<std::mutex> lock(b.m);
    if (bytes != b.*bytes_seen) {
      if (const std::optional<double> x = parse_number(bytes); x && *x >= 0)
        to_mib = std::to_string(static_cast<std::uint64_t>(std::llround(*x / kMiB)));
    } else if (mib != b.*mib_seen) {
      if (const std::optional<double> x = parse_number(mib); x && *x >= 0)
        to_bytes = std::to_string(static_cast<std::uint64_t>(std::llround(*x * kMiB)));
    } else {
      return;
    }
    b.*bytes_seen = to_bytes.empty() ? bytes : to_bytes;
    b.*mib_seen = to_mib.empty() ? mib : to_mib;
  }
  if (!to_mib.empty() && to_mib != mib)
    set_key(root, std::string(kPolicyRoot) + mib_key, to_mib);
  if (!to_bytes.empty() && to_bytes != bytes)
    set_key(root, std::string(kPolicyRoot) + bytes_key, to_bytes);
}

void sync_mib(cvc::app &app, cvc::state &root) {
  sync_pair(app, root, "max_bytes", "max_mib", &cache_book::max_bytes_seen,
            &cache_book::max_mib_seen);
  sync_pair(app, root, "max_entry_bytes", "max_entry_mib", &cache_book::entry_bytes_seen,
            &cache_book::entry_mib_seen);
}

// A change under sys.net.http_cache (`path` relative to it): a clear request, or a policy key.
// Entries and stats change on every read; they are skipped at once.
void on_cache_change(cvc::app &app, cvc::state &root, const std::string &path) {
  if (path.rfind("entries", 0) == 0 || path.rfind("stats", 0) == 0)
    return;
  if (path == "clear") {
    const cvc::state::state_ptr n = root.findDescendantShared(kClearKey);
    if (!n || n->value().empty())
      return; // the reset below, or a write of ""
    clear_entries(app, root);
    n->value(std::string());
    return;
  }
  if (path.rfind("policy.", 0) == 0) {
    sync_mib(app, root);
    enforce(app, root, read_policy(root), std::string()); // a lowered budget applies at once
  }
}

void seed_policy(cvc::state &root) {
  const std::pair<const char *, std::string> defaults[] = {
      {"enabled", "1"},
      {"max_bytes", std::to_string(kDefaultMaxBytes)},
      {"max_entry_bytes", std::to_string(kDefaultMaxEntryBytes)},
      {"freshness_secs", std::to_string(kDefaultFreshnessSecs)},
      {"retention_secs", std::to_string(kDefaultRetentionSecs)}};
  for (const auto &[name, value] : defaults) {
    const cvc::state::state_ptr n = root.sharedChild(std::string(kPolicyRoot) + name);
    if (trim(n->value()).empty())
      n->value(value);
  }
  root.sharedChild(kClearKey); // there for a widget to bind, empty
}

// Watch sys.net.http_cache for a clear and for policy changes, from the first registration on,
// for the app's life (a clear works while the handler is unregistered too).
void watch_keys(cvc::app &app, cvc::state &root) {
  cache_book &b = book_of(app);
  const cvc::state::state_ptr node = root.sharedChild(kWatchRoot); // made outside the book's lock
  std::lock_guard<std::mutex> lock(b.m);
  if (b.watching)
    return;
  b.watching = true;
  // The app outlives its slots' connections (the book, and its scoped connection, are the app's).
  b.watch = node->childChanged.connect([a = &app, r = &root](const std::string &p) {
    try {
      on_cache_change(*a, *r, p);
    } catch (const std::exception &) {
      // a failed clear or sync must not fail the write that signalled it
    }
  });
}

// --- fetch + cache -----------------------------------------------------------------------------

cvc::net::HttpRequest plain_get(const std::string &url, const UriCancelled &cancelled) {
  cvc::net::HttpRequest req;
  req.url = url;
  req.max_bytes = detail::kMaxHttpBytes;
  req.follow_redirects = true; // uncredentialed (matches the plain handler's http_fetch)
  req.cancelled = cancelled;
  return req;
}

// The GET a read of `url` makes: conditional when there is a stale entry to revalidate.
cvc::net::HttpRequest conditional_get(const std::string &url, const EntrySnapshot &stale,
                                      const UriCancelled &cancelled) {
  cvc::net::HttpRequest req = plain_get(url, cancelled);
  if (stale.present) {
    if (!stale.etag.empty())
      req.headers.push_back("If-None-Match: " + stale.etag);
    if (!stale.last_modified.empty())
      req.headers.push_back("If-Modified-Since: " + stale.last_modified);
  }
  return req;
}

// A credentialed read: the plain handler's request (the provider's headers and method), sent
// uncached and never following a redirect.
cvc::net::HttpRequest credentialed_get(const Uri &u, const HttpRequestOptions &opts,
                                       const UriCancelled &cancelled) {
  cvc::net::HttpRequest req;
  req.url = u.raw;
  req.max_bytes = detail::kMaxHttpBytes;
  req.headers = opts.headers;
  req.follow_redirects = false;
  req.cancelled = cancelled;
  if (!opts.method.empty() && opts.method != "GET")
    req.method = opts.method;
  return req;
}

// An entry served as the reply the network would give: a 200 with its body and validators.
cvc::net::HttpResponse served(EntrySnapshot s) {
  cvc::net::HttpResponse r;
  r.ok = true;
  r.status = 200;
  r.body = std::move(s.body);
  r.canonical_url = std::move(s.effective_url);
  if (!s.content_type.empty())
    r.headers.push_back("Content-Type: " + s.content_type);
  if (!s.etag.empty())
    r.headers.push_back("ETag: " + s.etag);
  if (!s.last_modified.empty())
    r.headers.push_back("Last-Modified: " + s.last_modified);
  return r;
}

cvc::net::HttpResponse failed(const std::string &why) {
  cvc::net::HttpResponse r;
  r.error = why;
  return r;
}

// The network's reply `r` to the (conditional) GET for `key`, with the cache updated: a 304 to a
// stale entry serves that entry (as a 200) and refreshes it, a 200 replaces it (unless it may not
// be kept), and anything else (>= 400, a failure, an unsolicited 304) is returned as it came, for
// the caller's mapper. `stale` is the entry the request revalidated, if any.
cvc::net::HttpResponse absorb(cvc::app &app, cvc::state &root, const std::string &key,
                              EntrySnapshot stale, const pt::ptime &now, const Policy &pol,
                              cvc::net::HttpResponse r) {
  // 304 Not Modified: the stale body is still valid. Honor the revalidation response's own
  // directives, merge any validators it carries (falling back to the stale ones), and serve the
  // cached body. We only send conditional headers when we had a body, so stale.present holds here.
  if (r.ok && r.status == 304 && stale.present) {
    count(app, root, Count::revalidated, key);
    const CacheControl cc = parse_cache_control(r.headers);
    if (cc.no_store) {
      forget(app, root, key); // must not retain; serve this body once
      return served(std::move(stale));
    }
    find_header(r.headers, "ETag", stale.etag); // a 304 may refresh the validators
    find_header(r.headers, "Last-Modified", stale.last_modified);
    find_header(r.headers, "Content-Type", stale.content_type);
    const long ttl = cc.no_cache      ? 0
                     : cc.has_max_age ? std::max<long>(0, cc.max_age)
                                      : parse_long_or(stale.freshness_ttl, pol.freshness_secs);
    const long age = parse_age(r.headers);
    if (!store_entry(app, root, key, stale.etag, stale.last_modified, stale.content_type,
                     stale.effective_url, stale.body, now - pt::seconds(age), ttl, pol))
      forget(app, root, key); // the policy no longer keeps a body this size
    return served(std::move(stale));
  }

  count(app, root, Count::miss, key);
  // 200 OK: replace body + validators + TTL (unless no-store, or too big to keep), and serve it.
  if (r.ok && r.status == 200) {
    const CacheControl cc = parse_cache_control(r.headers);
    bool kept = false;
    if (!cc.no_store) {
      std::string etag, last_modified, content_type;
      find_header(r.headers, "ETag", etag);
      find_header(r.headers, "Last-Modified", last_modified);
      find_header(r.headers, "Content-Type", content_type);
      const long age = parse_age(r.headers);
      kept = store_entry(app, root, key, etag, last_modified, content_type, r.canonical_url, r.body,
                         now - pt::seconds(age), derive_freshness_ttl(r.headers, cc, pol), pol);
    }
    if (!kept && stale.present)
      forget(app, root, key); // the old body is not what the server serves now
  }
  // Any other status (>= 400, or an unsolicited 304 with no usable stale body) goes to the caller,
  // whose mapper treats a 304 as an error and a >= 400 as an error (follow == true, so no 3xx).
  return r;
}

// --- single-flight -----------------------------------------------------------------------------

struct Inflight {
  std::condition_variable cv;
  bool done = false;
  cvc::net::HttpResponse result;
};

// An app's fetches in flight, by cache key: only the same app's callers wait on one (the result is
// written into that app's cache).
struct flights {
  static constexpr const char *app_slot = "cvc.ariadne.http-cache-flights";
  std::mutex m;
  std::unordered_map<std::string, std::shared_ptr<Inflight>> map;
};

// single-flight wrapper: the first caller for a key becomes the leader (does the fetch + store);
// the rest wait and receive the leader's result. Coalesces only the network-bound paths (a fresh
// hit returns before this is called). This per-key serialization is the ONLY write coordination the
// cache needs — the entry write itself is a single atomic node op (see the CONCURRENCY note).
cvc::net::HttpResponse fetch_and_cache(cvc::app &app, cvc::state &root, const std::string &key,
                                       const std::string &url, const pt::ptime &now,
                                       const Policy &pol, const cvc::net::NetPolicy *policy,
                                       const UriCancelled &cancelled) {
  flights &f = app.slot<flights>();
  std::shared_ptr<Inflight> mine;
  {
    std::unique_lock<std::mutex> lk(f.m);
    auto it = f.map.find(key);
    if (it != f.map.end()) {
      std::shared_ptr<Inflight> in = it->second; // follower: wait for the leader
      in->cv.wait(lk, [&] { return in->done; });
      cvc::net::HttpResponse r = in->result;
      lk.unlock();
      // Served without a transfer of its own: a hit, unless the leader got nothing to serve.
      count(app, root, r.ok && r.status >= 200 && r.status < 300 ? Count::hit : Count::miss, key);
      return r;
    }
    mine = std::make_shared<Inflight>();
    f.map[key] = mine;
  } // release the flight lock before the (blocking) network fetch

  cvc::net::HttpResponse result;
  try {
    // Re-read the entry now that we lead. The caller's snapshot predates the flight lock, so a
    // leader that stored and left between that read and our lock is invisible to it; its store
    // happened before its erase, so this read sees it. Fresh: serve it, no second transfer.
    // Otherwise this snapshot, not the caller's, drives the fetch: newer validators, and an entry
    // evicted meanwhile (no-store, an invalidating store) must not make a conditional GET.
    EntrySnapshot snap = read_entry(*entries_of(root), key);
    if (snap.present && snapshot_fresh(snap, now)) {
      count(app, root, Count::hit, key);
      result = served(std::move(snap));
    } else {
      const cvc::net::HttpRequest req = conditional_get(url, snap, cancelled);
      // blocking send; holds no cache lock
      result =
          absorb(app, root, key, std::move(snap), now, pol, detail::send_http(app, req, policy));
    }
  } catch (const std::exception &e) {
    result = failed(e.what());
  } catch (...) {
    result = failed("unknown error");
  }

  {
    std::lock_guard<std::mutex> lk(f.m);
    mine->result = result;
    mine->done = true;
    f.map.erase(key);
  }
  mine->cv.notify_all(); // wake followers even on error (result carries the error)
  return result;
}

// A read of `url` through the cache, as the network would reply. Uncached (straight to the
// network, checked) when the URL carries userinfo or the policy turns the cache off; refused with
// no request when `policy` does not allow the URL, even with a fresh entry for it.
cvc::net::HttpResponse cached_get(cvc::app &app, cvc::state &root, const std::string &url,
                                  const cvc::net::NetPolicy *policy,
                                  const UriCancelled &cancelled) {
  if (url_has_userinfo(url)) { // credential-bearing: never cached, never following a redirect
    cvc::net::HttpRequest req = plain_get(url, cancelled);
    req.follow_redirects = false;
    return detail::send_http(app, req, policy);
  }
  const Policy pol = read_policy(root);
  if (!pol.enabled || (policy && !policy->allows_url(url)))
    return detail::send_http(app, plain_get(url, cancelled), policy); // a refusal sends nothing
  const std::string key = cache_key(url);
  const pt::ptime now = pt::microsec_clock::universal_time();
  EntrySnapshot snap = lookup(app, root, key);
  if (snap.present && snapshot_fresh(snap, now)) {
    count(app, root, Count::hit, key);
    return served(std::move(snap)); // FRESH: no network
  }
  // stale (conditional GET) or miss (unconditional) — coalesced.
  return fetch_and_cache(app, root, key, url, now, pol, policy, cancelled);
}

std::function<void()> abort_of(std::shared_ptr<cvc::net::HttpTransfer> t) {
  if (!t)
    return {};
  return [t] { t->abort(); };
}

// cached_get without blocking (cvc::net::start): `done` gets the reply, later where the transport
// answers later. Not coalesced: in a browser build without threads two pictures of one URL are
// already one ImageCache load, and a second (fetch) of a URL in flight makes its own request.
std::function<void()> start_cached(cvc::app &app, cvc::state &root,
                                   std::shared_ptr<const cvc::net::NetPolicy> policy,
                                   const std::string &url, cvc::net::HttpDone done) {
  if (url_has_userinfo(url)) {
    cvc::net::HttpRequest req = plain_get(url, {});
    req.follow_redirects = false;
    return abort_of(cvc::net::start(app, req, std::move(policy), std::move(done)));
  }
  const Policy pol = read_policy(root);
  if (!pol.enabled || (policy && !policy->allows_url(url)))
    return abort_of(cvc::net::start(app, plain_get(url, {}), std::move(policy), std::move(done)));
  const std::string key = cache_key(url);
  const pt::ptime now = pt::microsec_clock::universal_time();
  EntrySnapshot snap = lookup(app, root, key);
  if (snap.present && snapshot_fresh(snap, now)) {
    count(app, root, Count::hit, key);
    if (done)
      done(served(std::move(snap)));
    return {};
  }
  const cvc::net::HttpRequest req = conditional_get(url, snap, {});
  // The reply may come after the app has gone (a browser answers from its event loop): then it
  // is only passed on. On the one thread that answers, the app cannot go while it is stored.
  std::weak_ptr<int> life = book_of(app).life;
  return abort_of(cvc::net::start(app, req, std::move(policy),
                                  [a = &app, r = &root, life, key, snap = std::move(snap), now, pol,
                                   done = std::move(done)](cvc::net::HttpResponse reply) mutable {
                                    if (const std::shared_ptr<int> alive = life.lock()) {
                                      try {
                                        reply = absorb(*a, *r, key, std::move(snap), now, pol,
                                                       std::move(reply));
                                      } catch (const std::exception &) {
                                        // the reply as it came: the cache is only not updated
                                      }
                                    }
                                    if (done)
                                      done(std::move(reply));
                                  }));
}

// --- the URI handlers --------------------------------------------------------------------------

UriResult cached_http_fetch(cvc::app &app, cvc::state &root, const cvc::net::NetPolicy *policy,
                            const Uri &u, const UriCancelled &cancelled) {
  const HttpRequestOptions opts = detail::consult_http_provider(app, u.raw, /*for_write=*/false);
  if (!opts.headers.empty() || url_has_userinfo(u.raw))
    // CREDENTIALED (provider headers or URL userinfo) → bypass the cache entirely (never serve one
    // principal's authorized body to another from the app's cache). Plain un-followed fetch.
    return detail::map_http_fetch_result(
        u.raw, /*follow=*/false,
        detail::send_http(app, credentialed_get(u, opts, cancelled), policy));
  return detail::map_http_fetch_result(u.raw, /*follow=*/true,
                                       cached_get(app, root, u.raw, policy, cancelled));
}

std::function<void()> cached_http_fetch_async(cvc::app &app, cvc::state &root,
                                              std::shared_ptr<const cvc::net::NetPolicy> policy,
                                              const Uri &u, UriDone done) {
  const HttpRequestOptions opts = detail::consult_http_provider(app, u.raw, /*for_write=*/false);
  if (!opts.headers.empty() || url_has_userinfo(u.raw))
    return abort_of(
        cvc::net::start(app, credentialed_get(u, opts, {}), std::move(policy),
                        [raw = u.raw, done = std::move(done)](cvc::net::HttpResponse r) {
                          done(detail::map_http_fetch_result(raw, false, r));
                        }));
  return start_cached(app, root, std::move(policy), u.raw,
                      [raw = u.raw, done = std::move(done)](cvc::net::HttpResponse r) {
                        done(detail::map_http_fetch_result(raw, true, r));
                      });
}

StoreResult invalidating_store(cvc::app &app, cvc::state &root, const cvc::net::NetPolicy *policy,
                               const Uri &u, const std::string &content, const std::string &base) {
  const StoreResult r = detail::http_store_uncached(app, u, content, base, policy);
  if (r.ok)
    forget(app, root, cache_key(u.raw));
  return r;
}

} // namespace

void register_cached_http_uri_handler(cvc::app &app) {
  register_cached_http_uri_handler(app, nullptr);
}

void register_cached_http_uri_handler(cvc::app &app,
                                      std::shared_ptr<const cvc::net::NetPolicy> policy) {
  if (!cvc::net::have_http_backend())
    return; // no transport compiled — leave the scheme unresolved
  // The handlers are held in `app`'s own table, so `app` (and its state root) outlive them.
  cvc::state *root = &cvc::state::instance(app);
  seed_policy(*root);
  watch_keys(app, *root);
  sync_mib(app, *root);
  {
    cache_book &b = book_of(app);
    std::lock_guard<std::mutex> lock(b.m);
    b.registered = true;
    b.policy = policy;
    ++b.version;
  }
  enforce(app, *root, read_policy(*root), std::string()); // and mirror the stats
  const auto fetch = [&app, root, policy](const Uri &u, const std::string &,
                                          const UriCancelled &c) {
    return cached_http_fetch(app, *root, policy.get(), u, c);
  };
  const auto fetch_async = [&app, root, policy](const Uri &u, const std::string &, UriDone done) {
    return cached_http_fetch_async(app, *root, policy, u, std::move(done));
  };
  const auto store = [&app, root, policy](const Uri &u, const std::string &c,
                                          const std::string &b) {
    return invalidating_store(app, *root, policy.get(), u, c, b);
  };
  for (const char *scheme : {"http", "https"}) {
    register_uri_handler(app, scheme, fetch); // drops an older non-blocking form: register it after
    register_uri_async_handler(app, scheme, fetch_async);
    register_uri_store_handler(app, scheme, store);
  }
}

void unregister_cached_http_uri_handler(cvc::app &app) {
  unregister_uri_handler(app, "http"); // and its non-blocking form
  unregister_uri_handler(app, "https");
  unregister_uri_store_handler(app, "http");
  unregister_uri_store_handler(app, "https");
  cache_book &b = book_of(app);
  std::lock_guard<std::mutex> lock(b.m);
  b.registered = false;
  b.policy.reset();
}

bool has_http_cache(cvc::app &app) {
  cache_book &b = book_of(app);
  std::lock_guard<std::mutex> lock(b.m);
  return b.registered;
}

std::optional<cvc::net::HttpResponse> http_cache_get(cvc::app &app, const std::string &url,
                                                     const UriCancelled &cancelled) {
  std::shared_ptr<const cvc::net::NetPolicy> policy;
  {
    cache_book &b = book_of(app);
    std::lock_guard<std::mutex> lock(b.m);
    if (!b.registered)
      return std::nullopt;
    policy = b.policy;
  }
  try {
    return cached_get(app, cvc::state::instance(app), url, policy.get(), cancelled);
  } catch (const std::exception &e) {
    return failed(e.what());
  }
}

bool http_cache_start(cvc::app &app, const std::string &url, cvc::net::HttpDone done) {
  std::shared_ptr<const cvc::net::NetPolicy> policy;
  {
    cache_book &b = book_of(app);
    std::lock_guard<std::mutex> lock(b.m);
    if (!b.registered)
      return false;
    policy = b.policy;
  }
  // `done` once, also when starting fails after it answered.
  auto pending = std::make_shared<cvc::net::HttpDone>(std::move(done));
  const auto once = [pending](cvc::net::HttpResponse r) {
    if (cvc::net::HttpDone d = std::exchange(*pending, nullptr))
      d(std::move(r));
  };
  try {
    start_cached(app, cvc::state::instance(app), std::move(policy), url, once);
  } catch (const std::exception &e) {
    once(failed(e.what()));
  }
  return true;
}

void clear_http_cache(cvc::app &app) { clear_entries(app, cvc::state::instance(app)); }

HttpCacheStats http_cache_stats(cvc::app &app) {
  cache_book &b = book_of(app);
  std::lock_guard<std::mutex> lock(b.m);
  return stats_locked(b);
}

} // namespace ariadne
} // namespace cvc
