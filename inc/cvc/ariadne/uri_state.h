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
// as §13.2 intends (state:// is "in-process", registered from the intrinsics context). The read
// AND write handlers hold `root` by pointer; the host MUST call unregister_state_uri_handler()
// (which tears down BOTH) before that root is destroyed.

namespace cvc {
class state;

namespace ariadne {

// Register the `state://` READ and WRITE handlers against `root` (a path is relative to it,
// dotted — `a.b.c`; an empty path is `root` itself). Both read and write serve the `?value`
// (default) and `?data` channels and follow a transparent link to its target; a write (§13.10
// store) creates the node path if absent. Replaces any existing "state" handlers.
// Not thread-safe against a concurrent resolve/store of the same scheme; register during setup.
// The handlers hold `root` by pointer — call unregister_state_uri_handler() before it is destroyed.
void register_state_uri_handler(cvc::state &root);

// Remove both the `state://` read and write handlers (the paired teardown for
// register_state_uri_handler — unregisters BOTH, so a later resolve/store cannot call a handler
// holding a dangling root pointer).
void unregister_state_uri_handler();

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_STATE_H
