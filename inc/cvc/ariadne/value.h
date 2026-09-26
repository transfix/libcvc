#ifndef CVC_ARIADNE_VALUE_H
#define CVC_ARIADNE_VALUE_H

// Ariadne neutral value tree (cvc::ariadne). Pure libcvc — NO yaml-cpp, NO VTK.
//
// A backend-neutral, parser-agnostic representation of a chunk of a .ari document:
// a scalar (kept as its raw string, like cvc::state), a sequence, or an ordered map.
// The loader converts a YAML subtree into a Value once (so yaml-cpp stays private to
// loader.cpp), and hands Values to EXTENSION points that must not depend on yaml-cpp:
//   - a custom scene node's config bag (SceneNode::props) that a cvcGL node realizer
//     registered via register_scene_node_type() reads, and
//   - a custom top-level block's content that a parser registered via
//     register_ari_block() reads.
// The accessors mirror the loader's own str()/num()/flag() helpers so extension code
// reads config the same way the built-ins do.

#include <locale>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

struct Value {
  enum class Kind { Scalar, Sequence, Map };

  Kind kind = Kind::Scalar;
  std::string scalar;                                 // Kind::Scalar
  std::vector<Value> items;                           // Kind::Sequence
  std::vector<std::pair<std::string, Value>> entries; // Kind::Map (insertion order)

  bool is_scalar() const { return kind == Kind::Scalar; }
  bool is_seq() const { return kind == Kind::Sequence; }
  bool is_map() const { return kind == Kind::Map; }
  bool empty() const {
    return kind == Kind::Scalar ? scalar.empty()
                                : (kind == Kind::Sequence ? items.empty() : entries.empty());
  }

  // --- this value as a scalar --------------------------------------------------
  // Mirrors the loader's str(): a present scalar (even "") is returned verbatim, so an
  // explicit empty string can override a non-empty default; a non-scalar yields dflt.
  std::string as_string(const std::string &dflt = std::string()) const {
    return is_scalar() ? scalar : dflt;
  }
  // Mirrors the loader's num() (yaml-cpp's strict as<double>()): whole-token,
  // locale-independent parse. Trailing garbage ("10px"), a comma decimal, or a
  // non-number yields dflt — NOT a truncated value, and unaffected by the C locale.
  double as_double(double dflt = 0.0) const {
    if (!is_scalar())
      return dflt;
    std::istringstream ss(scalar);
    ss.imbue(std::locale::classic());
    double out = 0.0;
    if ((ss >> out) && (ss >> std::ws).eof())
      return out; // the entire scalar was a number
    return dflt;
  }
  bool as_bool(bool dflt = false) const {
    if (!is_scalar())
      return dflt;
    return scalar == "true" || scalar == "1" || scalar == "yes" || scalar == "on";
  }

  // --- map access (nullptr / default when absent or wrong kind) ----------------
  const Value *find(const std::string &key) const {
    if (is_map())
      for (const auto &e : entries)
        if (e.first == key)
          return &e.second;
    return nullptr;
  }
  bool has(const std::string &key) const { return find(key) != nullptr; }
  std::string str(const std::string &key, const std::string &dflt = std::string()) const {
    const Value *v = find(key);
    return v ? v->as_string(dflt) : dflt;
  }
  double num(const std::string &key, double dflt = 0.0) const {
    const Value *v = find(key);
    return v ? v->as_double(dflt) : dflt;
  }
  bool flag(const std::string &key, bool dflt = false) const {
    const Value *v = find(key);
    return v ? v->as_bool(dflt) : dflt;
  }
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_VALUE_H
