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
// It is opt-in: a host registers it on an app (register_state_uri_handler) before load_*, exactly
// as §13.2 intends (state:// is "in-process"). The handlers are that app's, resolve against its
// state tree, and go with it.

namespace cvc {
class app;

namespace ariadne {

// Register `app`'s `state://` READ and WRITE handlers against its root state (a path is relative
// to it, dotted — `a.b.c`; an empty path is the root itself). Both read and write serve the
// `?value` (default) and `?data` channels and follow a transparent link to its target; a write
// (§13.10 store) creates the node path if absent. Replaces any existing "state" handlers on `app`.
// Register during setup.
void register_state_uri_handler(cvc::app &app);

// Remove both of `app`'s `state://` read and write handlers.
void unregister_state_uri_handler(cvc::app &app);

// §13.10: read the resolver's file byte caps FROM `app`'s STATE TREE and apply them to `app`'s
// caps (uri.h `set_*_file_byte_cap`). The config lives under the root at
// `sys.ariadne.resolver.read_cap_bytes` and `sys.ariadne.resolver.store_cap_bytes` — a decimal
// byte count, "0" = unlimited. Any node that is not a clean, complete, non-negative decimal count
// — missing, empty, non-numeric ("oops"), partial ("16MiB"), signed ("-1"), or out of range —
// leaves that cap unchanged, so partial config is fine and a typo can neither zero nor uncap the
// resolver. A host calls it at setup and — to track runtime edits — from a state-watch on those
// nodes. (The resolver never reads the state tree itself.)
void sync_resolver_caps_from_state(cvc::app &app);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_STATE_H
