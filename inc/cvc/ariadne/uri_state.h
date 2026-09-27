#ifndef CVC_ARIADNE_URI_STATE_H
#define CVC_ARIADNE_URI_STATE_H

// Ariadne — the `state://` URI scheme (roadmap §13.3). A `state://` reference addresses a node
// in a `cvc::state` tree and resolves to that node's VALUE — the string on its value channel,
// which for a UI library is inline `.ari` text. It is NOT the node's child subtree: the query
// selects the channel, and only `?value` (the default) is served here.
//
//   state://scene.ui.forms          # ?value (default): node.value() — the fragment text
//   state://scene.ui.forms?value    #   (explicit) same channel
//
// `?data` (typed node.data()) and `?children` (the subtree as a value_t tree) are documented in
// §13.3 but need the codec / value_t machinery; the built-in handler rejects them so a host can
// register a richer one without a silent wrong answer.
//
// This scheme needs a state root, which the app-free loader does not have — so it is opt-in: a
// host with a state tree registers it (via register_state_uri_handler) before load_*, exactly
// as §13.2 intends (state:// is "in-process", registered from the intrinsics context). The
// handler holds `root` by pointer; the host MUST unregister_uri_handler("state") before that
// root is destroyed (see uri.h).

namespace cvc {
class state;

namespace ariadne {

// Register a `state://` handler resolving node paths against `root` (a path is relative to it,
// dotted — `a.b.c`; an empty path is `root` itself). Transparent links are followed
// (resolvedValue). Replaces any existing "state" handler. Not thread-safe against a concurrent
// resolve of the same scheme; register during setup, before loads run.
void register_state_uri_handler(cvc::state &root);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_STATE_H
