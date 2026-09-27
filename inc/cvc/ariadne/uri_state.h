#ifndef CVC_ARIADNE_URI_STATE_H
#define CVC_ARIADNE_URI_STATE_H

// Ariadne — the `state://` URI scheme (roadmap §13.3 read + §13.10 write). A `state://` reference
// addresses a node in a `cvc::state` tree; the query selects the channel. The built-in handler
// serves `?value` (default) and `?data` for BOTH read (resolve) and write (store):
//
//   state://scene.ui.forms          # ?value (default): node.value() — a string (e.g. .ari text)
//   state://scene.ui.forms?value    #   (explicit) same channel
//   state://scene.blob?data         # ?data: a raw string/byte blob on node.data()
//
// `?value` is a string (for a UI library, inline `.ari` text); `?data` is a raw string/byte blob on
// the data channel (what the §13.9 HTTP cache parks a body on). `?children` (the subtree as a
// value_t tree) is NOT served — it needs the codec / value_t machinery, so the built-in handler
// rejects it rather than return a silent wrong answer; a host can register a richer one.
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

// §13.10: read the resolver's file byte caps FROM THE STATE TREE and apply them to the
// process-global caps (uri.h `set_*_file_byte_cap`). The config lives under `root` at
// `sys.ariadne.resolver.read_cap_bytes` and `sys.ariadne.resolver.store_cap_bytes` — a decimal
// byte count, "0" = unlimited. A missing / empty / non-numeric node leaves that cap unchanged, so
// partial config is fine. This is the opt-in bridge that pushes state config into the app-free
// resolver: a host calls it at setup and — to track runtime edits — from a state-watch on those
// nodes. (The resolver never reads the state tree itself; the caps stay plain process globals.)
void sync_resolver_caps_from_state(cvc::state &root);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_STATE_H
