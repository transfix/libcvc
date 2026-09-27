// Ariadne — the `state://` URI handler (roadmap §13.3). See uri_state.h. Resolves a state node
// reference to the VALUE channel of the addressed node (the `.ari` text it holds), following a
// transparent link to its target. Lives in its own TU so the pure resolver (uri.cpp) keeps zero
// dependency on cvc::state — this scheme is the opt-in add-on that pulls it in.

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

UriResult state_resolve(cvc::state &root, const Uri &u) {
  const std::string channel = channel_of(u.query);
  if (!channel.empty() && channel != "value")
    return {false, std::string(), std::string(),
            "ari: state channel '?" + channel +
                "' is not served by the built-in state handler (only '?value'); register a "
                "custom handler for '?data' / '?children'"};

  // Navigate WITHOUT creating nodes; an empty path is the root itself.
  cvc::state *node = u.path.empty() ? &root : root.findDescendant(u.path);
  if (!node)
    return {false, std::string(), std::string(), "ari: state node '" + u.path + "' not found"};

  // The value channel follows a transparent link to its target (matching resolvedValue). Resolve
  // the effective node ONCE so content and canonical come from the SAME node: an opaque/broken
  // link or a non-link stays put (own value), a transparent link lands on its terminal target.
  // Keying the canonical on the effective node's absolute path is what lets an alias and its
  // target collapse in the loader's dedup / cycle guard.
  cvc::state *effective = node;
  if (node->isLink() && node->linkMode() == cvc::state::link_mode::transparent) {
    const cvc::state::link_resolution lr = node->resolveLink();
    if (lr.kind == cvc::state::link_resolution_kind::resolved && lr.target)
      effective = lr.target; // broken / cyclic / budget-exhausted → fall back to the link node
  }
  std::string content = effective->value();
  const std::string canonical = "state://" + effective->fullName() + "?value";
  return {true, std::move(content), canonical, std::string()};
}

// §13.10 the write analogue: store `content` into the addressed node's value channel. operator()
// CREATES the node path if absent (a store may target a not-yet-existing node); an empty path is
// the root. Writing via value() routes through a writable transparent link to its target (the
// write analogue of state_resolve's follow), else it writes the node's own value.
StoreResult state_store(cvc::state &root, const Uri &u, const std::string &content) {
  const std::string channel = channel_of(u.query);
  if (!channel.empty() && channel != "value")
    return {false, std::string(),
            "ari: state channel '?" + channel +
                "' is not writable by the built-in state handler (only '?value')"};
  cvc::state &node = u.path.empty() ? root : root(u.path);
  node.value(content);
  return {true, "state://" + node.fullName() + "?value", std::string()};
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
