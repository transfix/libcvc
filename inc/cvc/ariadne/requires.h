#ifndef CVC_ARIADNE_REQUIRES_H
#define CVC_ARIADNE_REQUIRES_H
// requires: — what a document needs from the host it runs on (roadmap §7.6, §16.3).
//
// A document lists what it needs in a top-level `requires:`. AppRuntime::load checks every entry
// against the host before anything runs: a missing one fails the load with one error that names
// them all, or, for an entry marked `optional: true`, warns. An entry is one of:
//   - a capability token (capability_tokens()): something the backend or host can do;
//   - a function a program can call: a name that resolves in the env the document's programs run
//     in (a builtin, a stdlib function, a host verb or intrinsic), or a special form;
//   - a registered widget type.
//
// Whether or not a document declares requires:, the load also derives what each program calls (its
// free head symbols, free_heads()) and warns about any name the host does not provide. That moves
// the run-time "undefined symbol" to load. The scan is a lower bound: a name a program builds at
// run time, or hands to apply, is invisible to it. So it only warns; requires: is the authoritative
// list.
//
// A program can ask the same question at run time with (has? "name").
#include <cvc/ariadne/backend.h>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// What the host can do, beyond its backend's Capabilities.
struct HostFacts {
  Capabilities caps;
  bool scene = false;            // a scene adapter realizes `scene:` blocks
  bool scene_shaders = false;    // ... whose renderer runs the document's GLSL (runs_shaders)
  bool net = false;              // the host turned the network on (net_host.h)
  bool net_auth = false;         // ... with a credential store
  bool net_auth_persist = false; // ... that keeps credentials across runs
};

// One capability token and what it means.
struct CapabilityToken {
  const char *name;
  const char *meaning;
};

// Every capability token, in documentation order.
const std::vector<CapabilityToken> &capability_tokens();

// Is `name` a capability token (as opposed to a function or widget name)?
bool is_capability_token(const std::string &name);

// Does this host have capability `name`? False for a name that is not a token.
bool capability_holds(const std::string &name, const HostFacts &facts);

// The names a program applies as functions but does not define itself, in first-appearance order,
// each once: the heads of its calls, minus special forms, quoted data, and the names it binds
// (defun / defmacro / defclass / set targets, lambda / let / for / catch variables, defclass
// methods). `program` is the source of one or more forms. A program that does not parse, or a bare
// event name (not an s-expression), yields nothing.
std::vector<std::string> free_heads(const std::string &program);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_REQUIRES_H
