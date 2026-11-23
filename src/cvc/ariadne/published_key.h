// A key a publisher writes again and again (the world's telemetry, each entity's keys): its lookup
// held (cvc::state_key, as root(path): made when absent, links not followed), and what it last
// wrote there, so a value that has not changed is neither formatted nor written. Something else
// writing the key, or the tree changing shape, is seen (the node's revision, the key's walks): the
// next put writes again, as a write every frame would.
#ifndef CVC_ARIADNE_PUBLISHED_KEY_H
#define CVC_ARIADNE_PUBLISHED_KEY_H

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_key.h>
#include <string>
#include <utility>

namespace cvc {
namespace ariadne {

class published_key {
public:
  published_key() = default;
  explicit published_key(std::string path) : _key(std::move(path)) {}

  void set_path(const std::string &path) { _key.set_path(path); }
  const std::string &path() const { return _key.path(); }

  // The text, unless it is what this key last wrote and the key was not written since.
  void text(cvc::app &app, const std::string &t) {
    cvc::state &n = _key.get(app);
    if (standing(n) && _kind == kind::text && _text == t)
      return;
    n.value(t);
    _text = t;
    _kind = kind::text;
    stand(n);
  }

  // A number to `digits` significant digits (as printf's %g), formatted only when it changed. A
  // key keeps one precision.
  void number(cvc::app &app, double v, int digits = 6) {
    cvc::state &n = _key.get(app);
    if (standing(n) && _kind == kind::number && std::memcmp(&v, &_number, sizeof v) == 0)
      return;
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*g", digits, v);
    _text = buf;
    n.value(_text);
    _number = v;
    _kind = kind::number;
    stand(n);
  }

  // An integer, as std::to_string writes it.
  void integer(cvc::app &app, long long v) {
    cvc::state &n = _key.get(app);
    if (standing(n) && _kind == kind::integer && _integer == v)
      return;
    _text = std::to_string(v);
    n.value(_text);
    _integer = v;
    _kind = kind::integer;
    stand(n);
  }

  // Forget what was written: the next put writes.
  void forget() { _have = false; }

private:
  bool standing(const cvc::state &n) const {
    return _have && _walks == _key.walks() && _rev == n.revision();
  }
  void stand(const cvc::state &n) {
    _have = true;
    _walks = _key.walks();
    _rev = n.revision();
  }

  enum class kind : unsigned char { text, number, integer };
  cvc::state_key _key;
  bool _have = false;
  kind _kind = kind::text;
  std::uint64_t _walks = 0, _rev = 0;
  double _number = 0.0;
  long long _integer = 0;
  std::string _text;
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_PUBLISHED_KEY_H
