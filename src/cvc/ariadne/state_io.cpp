// Ariadne — URI-aware state persistence (roadmap §13.10). See state_io.h. Bridges cvc::state's
// JSON serialization (json() / json(std::string)) to the §13 resolver's read (resolve) and write
// (store) sides, so a state subtree round-trips over any scheme without cvc::state depending on
// the resolver.

#include <cvc/ariadne/state_io.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/state.h>
#include <exception>
#include <string>

namespace cvc {
namespace ariadne {

bool save_state(cvc::state &node, const std::string &uri, std::string *error) {
  try {
    const std::string json = node.json(); // serialize the subtree
    const StoreResult r = store(uri, json);
    if (!r.ok) {
      if (error)
        *error = r.error;
      return false;
    }
    return true;
  } catch (const std::exception &e) {
    if (error)
      *error = std::string("ari: save_state('") + uri + "') failed: " + e.what();
    return false;
  } catch (...) {
    if (error)
      *error = std::string("ari: save_state('") + uri + "') failed (unknown error)";
    return false;
  }
}

bool restore_state(cvc::state &node, const std::string &uri, std::string *error) {
  try {
    const UriResult r = resolve(uri);
    if (!r.ok) {
      if (error)
        *error = r.error;
      return false;
    }
    node.json(r.content); // parse the JSON into the subtree (throws on malformed JSON)
    return true;
  } catch (const std::exception &e) {
    if (error)
      *error = std::string("ari: restore_state('") + uri + "') failed: " + e.what();
    return false;
  } catch (...) {
    if (error)
      *error = std::string("ari: restore_state('") + uri + "') failed (unknown error)";
    return false;
  }
}

} // namespace ariadne
} // namespace cvc
