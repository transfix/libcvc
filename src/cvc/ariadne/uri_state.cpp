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

  // The value channel: follow a transparent link to its target (falls back to this node's own
  // value on a broken / cyclic / budget-exhausted chain — resolvedValue's documented contract).
  std::string content = node->resolvedValue();
  // Canonical identity for the loader's dedup / cycle guard: the resolved node's absolute path,
  // so two spellings of the same node collapse and distinct nodes stay distinct.
  const std::string canonical = "state://" + node->fullName() + "?value";
  return {true, std::move(content), canonical, std::string()};
}

} // namespace

void register_state_uri_handler(cvc::state &root) {
  cvc::state *root_ptr = &root;
  register_uri_handler("state", [root_ptr](const Uri &u, const std::string & /*base*/) {
    return state_resolve(*root_ptr, u);
  });
}

} // namespace ariadne
} // namespace cvc
