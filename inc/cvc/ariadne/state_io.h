#ifndef CVC_ARIADNE_STATE_IO_H
#define CVC_ARIADNE_STATE_IO_H

// Ariadne — URI-aware state persistence (roadmap §13.10). save_state / restore_state serialize a
// cvc::state subtree to / from any URI the §13 resolver can write / read (file://, state://,
// http(s)://, or a custom scheme), routing through the ONE scheme registry that also backs
// import:/load:/source:.
//
// These live at the resolver layer and operate ON a cvc::state& — the recommended option (b) of
// §13.10 — so cvc::state itself stays free of any dependency on cvc::ariadne (no layering
// inversion). A bare/relative path is the file:// case, so save_state(node, "x.json") is like
// cvc::state::save("x.json") but ATOMIC (write-temp + rename) and through the resolver. For a
// non-file scheme the matching handler must be registered first (register_state_uri_handler for
// state://, register_http_uri_handler for http(s)://).
//
// SEMANTICS mirror cvc::state::save/restore EXACTLY (these just re-route the bytes): the JSON keys
// are ABSOLUTE state paths (each node's fullName), and restore applies them RELATIVE to the target
// node. So restore into the SAME position the data was saved from — typically the app root
// (cvc::state::instance(app)) for both — to reconstruct the original layout; restoring into a
// different node re-roots the saved absolute paths beneath it.

#include <string>

namespace cvc {
class state;

namespace ariadne {

// Serialize `node`'s subtree (as JSON) and store it at `uri` via the resolver's write side
// (§13.10 store). Returns true on success; on failure returns false and, if `error` is non-null,
// sets a message. Never throws.
bool save_state(cvc::state &node, const std::string &uri, std::string *error = nullptr);

// Fetch `uri` via the resolver and load its JSON into `node`'s subtree (mirrors
// cvc::state::restore). Returns true on success; on failure returns false and sets `*error`
// (unresolvable URI, or malformed JSON). Never throws.
bool restore_state(cvc::state &node, const std::string &uri, std::string *error = nullptr);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_STATE_IO_H
