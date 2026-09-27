// Ariadne — the `state://` URI handler (roadmap §13.3 read + §13.10 write). See uri_state.h.
// Read (resolve) and write (store) the addressed node's `?value` (default) or `?data` channel,
// following a transparent link to its target on BOTH paths. Lives in its own TU so the pure
// resolver (uri.cpp) keeps zero dependency on cvc::state — this scheme is the opt-in add-on.

#include <boost/any.hpp>
#include <cstddef>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_state.h>
#include <cvc/core/state.h>
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

// Follow a TRANSPARENT link to its terminal target; a non-link / opaque / broken / cyclic
// transparent link stays put (resolvedValue's fallback). SHARED by read and write so both address
// the identical node — crucially even a DEFAULT (non-writable) transparent link: a read follows it,
// so a write must follow it too, or a store would be shadowed on the link node and unreadable via
// the same URI (the write-through routing in state::value() only fires for a WRITABLE link).
cvc::state *effective_node(cvc::state *node) {
  if (node && node->isLink() && node->linkMode() == cvc::state::link_mode::transparent) {
    const cvc::state::link_resolution lr = node->resolveLink();
    if (lr.kind == cvc::state::link_resolution_kind::resolved && lr.target)
      return lr.target;
  }
  return node;
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

  // Navigate WITHOUT creating nodes; an empty path is the root itself.
  cvc::state *node = u.path.empty() ? &root : root.findDescendant(u.path);
  if (!node)
    return {false, std::string(), std::string(), "ari: state node '" + u.path + "' not found"};
  cvc::state *eff = effective_node(node); // read follows a transparent link to its target
  const std::string canonical = "state://" + eff->fullName() + "?" + channel;

  if (channel == "data") {
    // The data channel carries a raw string/byte blob (what state_store writes here, and what the
    // §13.9 HTTP cache parks on a node). Typed data() payloads (a value_t / geometry) are a
    // different consumer (state-data-get) and are not byte-serialized here.
    const boost::any d = eff->data();
    if (const std::string *s = boost::any_cast<std::string>(&d))
      return {true, *s, canonical, std::string()};
    if (d.empty())
      return {true, std::string(), canonical, std::string()}; // empty data -> empty content
    return {false, std::string(), std::string(),
            "ari: state '" + u.path + "?data' holds a non-string payload (not byte-serializable)"};
  }
  return {true, eff->value(), canonical, std::string()};
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
  cvc::state *eff = effective_node(u.path.empty() ? &root : &root(u.path));
  if (channel == "data")
    eff->data(boost::any(content)); // store the bytes as a string blob on the data channel
  else
    eff->value(content);
  return {true, "state://" + eff->fullName() + "?" + channel, std::string()};
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

} // namespace ariadne
} // namespace cvc
