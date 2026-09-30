// Ariadne — the `state://` URI handler (roadmap §13.3 read + §13.10 write). See uri_state.h.
// Read (resolve) and write (store) the addressed node's `?value` (default) or `?data` channel,
// following a transparent link to its target on BOTH paths. Lives in its own TU so the pure
// resolver (uri.cpp) keeps zero dependency on cvc::state — this scheme is the opt-in add-on.

#include <boost/any.hpp>
#include <cstddef>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_state.h>
#include <cvc/core/state.h>
#include <exception>
#include <limits>
#include <string>

namespace cvc {
namespace ariadne {

namespace {

// The channel a `state://…?query` selects. Only `value` is served; the query may carry extra
// `&`-separated knobs (e.g. `?value&snapshot`) which we ignore for the value channel.
std::string channel_of(const std::string &query) {
  std::string ch = query;
  const std::size_t amp = ch.find('&');
  if (amp != std::string::npos)
    ch = ch.substr(0, amp);
  return ch;
}

// The effective (link-followed) node, PINNED, plus its canonical absolute path. `node` is held as
// an owning state_ptr and `path` is captured from strings we already hold (the addressed path, or
// resolveLink's visited chain) — never a post-resolution fullName() on `node`, which would walk the
// node's _parent chain and UAF if a concurrent sweepExpired() orphaned it (the confirmed cross-node
// hazard for a `state://` resolve on a compute-pool worker).
struct effective {
  cvc::state::state_ptr node;
  // The effective node PLUS its ancestor chain, kept alive so a write (value()/data() walk the
  // node's _parent chain via fullName()/childChanged()) is safe against a concurrent sweep. For a
  // followed transparent link this is the TARGET's chain; otherwise the addressed node's chain.
  std::vector<cvc::state::state_ptr> pins;
  std::string path;
};

// Follow a TRANSPARENT link to its terminal target; a non-link / opaque / broken / cyclic
// transparent link stays put (resolvedValue's fallback). SHARED by read and write so both address
// the identical node — crucially even a DEFAULT (non-writable) transparent link: a read follows it,
// so a write must follow it too, or a store would be shadowed on the link node and unreadable via
// the same URI (the write-through routing in state::value() only fires for a WRITABLE link).
// `addressed_pins` is the addressed node's own ancestor chain (from the caller's navigation), used
// when the node is not a followed link; `addressed_path` is its normalized `state://` path.
effective effective_node(cvc::state::state_ptr node,
                         std::vector<cvc::state::state_ptr> addressed_pins,
                         const std::string &addressed_path) {
  if (node && node->isLink() && node->linkMode() == cvc::state::link_mode::transparent) {
    const cvc::state::link_resolution lr = node->resolveLink();
    if (lr.kind == cvc::state::link_resolution_kind::resolved && lr.target_owned)
      // target_owned pins the terminal and target_pins its ancestor chain; visited.back() is its
      // absolute path captured during the walk (safe) — no fullName() on a possibly-orphaned node.
      return {lr.target_owned, lr.target_pins,
              lr.visited.empty() ? addressed_path : lr.visited.back()};
  }
  return {std::move(node), std::move(addressed_pins), addressed_path};
}

// The `?value` / `?data` channels are served; `?children` and anything else are not (they need the
// value_t/codec machinery a host can add). Returns "value" for the default/empty channel.
bool served_channel(const std::string &channel, std::string &normalized) {
  normalized = channel.empty() ? "value" : channel;
  return normalized == "value" || normalized == "data";
}

UriResult state_resolve(cvc::state &root, const Uri &u) {
  std::string channel;
  if (!served_channel(channel_of(u.query), channel))
    return {false, std::string(), std::string(),
            "ari: state channel '?" + channel +
                "' is not served by the built-in state handler (only '?value' / '?data'); register "
                "a custom handler for '?children'"};

  // Navigate WITHOUT creating nodes; an empty path is the root itself. PIN the addressed node AND
  // its ancestor chain so a concurrent sweepExpired() (on the scheduler/pycvc thread) cannot free
  // it — or an ancestor it walks — while this resolve, which may run on a compute-pool worker,
  // reads its value/data (parent-free getters) and, for a transparent link, resolves through it
  // (resolveLink touches the start node's fullName). Canonical is built from the normalized path
  // STRING, never a fullName() on the returned node.
  std::vector<cvc::state::state_ptr> chain;
  cvc::state::state_ptr node =
      u.path.empty() ? root.shared_from_this() : root.findDescendantShared(u.path, &chain);
  if (!node)
    return {false, std::string(), std::string(), "ari: state node '" + u.path + "' not found"};
  const effective eff =
      effective_node(node, std::move(chain),
                     cvc::state::normalize_path(u.path)); // follows a transparent link
  const std::string canonical = "state://" + eff.path + "?" + channel;

  if (channel == "data") {
    // The data channel carries a raw string/byte blob (what state_store writes here, and what the
    // §13.9 HTTP cache parks on a node). Typed data() payloads (a value_t / geometry) are a
    // different consumer (state-data-get) and are not byte-serialized here.
    const boost::any d = eff.node->data();
    if (const std::string *s = boost::any_cast<std::string>(&d))
      return {true, *s, canonical, std::string()};
    if (d.empty())
      return {true, std::string(), canonical, std::string()}; // empty data -> empty content
    return {false, std::string(), std::string(),
            "ari: state '" + u.path + "?data' holds a non-string payload (not byte-serializable)"};
  }
  return {true, eff.node->value(), canonical, std::string()};
}

// §13.10 the write analogue: store `content` into the addressed node's `?value` (default) or
// `?data` channel. operator() CREATES the node path if absent (a store may target a not-yet-
// existing node); an empty path is the root. It writes the EFFECTIVE node (following a transparent
// link to its target, exactly as the read does) so store and resolve address the same node — a
// write is never shadowed on a non-writable link.
StoreResult state_store(cvc::state &root, const Uri &u, const std::string &content) {
  std::string channel;
  if (!served_channel(channel_of(u.query), channel))
    return {false, std::string(),
            "ari: state channel '?" + channel +
                "' is not writable by the built-in state handler (only '?value' / '?data')"};
  // sharedChild CREATES the path if absent (a store may target a not-yet-existing node) and pins
  // the WHOLE chain (into `chain`). Unlike the read, the write below MUST pin ancestors:
  // value()/data() internally call fullName() and parent()->childChanged(), which walk the node's
  // _parent chain — a leaf-only pin would leave those ancestors exposed to a concurrent sweep (a
  // use-after-free). The pins (eff.pins for a followed link, else `chain`) are held alive across
  // the setter call below.
  std::vector<cvc::state::state_ptr> chain;
  cvc::state::state_ptr node =
      u.path.empty() ? root.shared_from_this() : root.sharedChild(u.path, &chain);
  const effective eff = effective_node(node, std::move(chain), cvc::state::normalize_path(u.path));
  if (channel == "data")
    eff.node->data(boost::any(content)); // store the bytes as a string blob on the data channel
  else
    eff.node->value(content);
  return {true, "state://" + eff.path + "?" + channel, std::string()};
}

} // namespace

void register_state_uri_handler(cvc::state &root) {
  cvc::state *root_ptr = &root;
  register_uri_handler("state", [root_ptr](const Uri &u, const std::string & /*base*/) {
    return state_resolve(*root_ptr, u);
  });
  register_uri_store_handler(
      "state", [root_ptr](const Uri &u, const std::string &content, const std::string & /*base*/) {
        return state_store(*root_ptr, u, content);
      });
}

void unregister_state_uri_handler() {
  unregister_uri_handler("state");       // read
  unregister_uri_store_handler("state"); // write
}

void sync_resolver_caps_from_state(cvc::state &root) {
  // Read one config node's decimal byte count and push it to `setter`; leave the cap unchanged on
  // a missing / empty / non-numeric node so partial config is fine and a typo cannot zero the cap.
  const auto apply = [&root](const char *path, void (*setter)(std::size_t)) {
    cvc::state *node = root.findDescendant(path);
    if (!node)
      return;
    const std::string v = node->value();
    if (v.empty())
      return;
    try {
      std::size_t pos = 0;
      const unsigned long long n = std::stoull(v, &pos);
      // stoull is lenient: it stops at trailing garbage ("16MiB" -> 16) and WRAPS a leading '-'
      // ("-1" -> ULLONG_MAX) instead of throwing. Require the WHOLE string to be a clean,
      // sign-free count, and (on a 32-bit size_t) reject a value that would truncate — otherwise
      // fall through to "leave the cap unchanged", honouring the never-silently-mis-set contract.
      if (pos != v.size() || v.find('-') != std::string::npos ||
          n > std::numeric_limits<std::size_t>::max())
        return;
      setter(static_cast<std::size_t>(n));
    } catch (const std::exception &) {
      // non-numeric / out-of-range -> leave the cap unchanged
    }
  };
  apply("sys.ariadne.resolver.read_cap_bytes", &set_resolve_file_byte_cap);
  apply("sys.ariadne.resolver.store_cap_bytes", &set_store_file_byte_cap);
}

} // namespace ariadne
} // namespace cvc
