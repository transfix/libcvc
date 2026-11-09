// value_cbor.cpp — see the header.

#include "value_cbor.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace wasm {

namespace se = cvc::state_exec;

namespace {

void head(std::string &out, int major, std::uint64_t n) {
  const auto m = static_cast<char>(major << 5);
  if (n < 24) {
    out += static_cast<char>(m | static_cast<char>(n));
    return;
  }
  int bytes = 8;
  char info = 27;
  if (n <= 0xff) {
    bytes = 1;
    info = 24;
  } else if (n <= 0xffff) {
    bytes = 2;
    info = 25;
  } else if (n <= 0xffffffffu) {
    bytes = 4;
    info = 26;
  }
  out += static_cast<char>(m | info);
  for (int i = bytes - 1; i >= 0; --i)
    out += static_cast<char>((n >> (8 * i)) & 0xff);
}

void encode(std::string &out, const se::value_t &v, int depth) {
  if (depth > kMaxCborDepth)
    throw std::invalid_argument("cbor: nested deeper than " + std::to_string(kMaxCborDepth));
  const auto &x = v.v;
  if (std::holds_alternative<std::monostate>(x)) {
    out += static_cast<char>(0xf6);
  } else if (const bool *b = std::get_if<bool>(&x)) {
    out += static_cast<char>(*b ? 0xf5 : 0xf4);
  } else if (const std::int64_t *i = std::get_if<std::int64_t>(&x)) {
    if (*i >= 0)
      head(out, 0, static_cast<std::uint64_t>(*i));
    else
      head(out, 1, static_cast<std::uint64_t>(-(*i + 1)));
  } else if (const double *d = std::get_if<double>(&x)) {
    out += static_cast<char>(0xfb);
    std::uint64_t bits = 0;
    std::memcpy(&bits, d, sizeof bits);
    for (int k = 7; k >= 0; --k)
      out += static_cast<char>((bits >> (8 * k)) & 0xff);
  } else if (const std::string *s = std::get_if<std::string>(&x)) {
    head(out, 3, s->size());
    out += *s;
  } else if (const se::symbol *sym = std::get_if<se::symbol>(&x)) {
    head(out, 3, sym->name.size());
    out += sym->name;
  } else if (const se::bytes_value *by = std::get_if<se::bytes_value>(&x)) {
    head(out, 2, by->data.size());
    out += by->data;
  } else if (const se::list_ptr *l = std::get_if<se::list_ptr>(&x)) {
    const std::size_t n = *l ? (*l)->size() : 0;
    head(out, 4, n);
    for (std::size_t k = 0; k < n; ++k)
      encode(out, (**l)[k], depth + 1);
  } else if (const se::dict_ptr *m = std::get_if<se::dict_ptr>(&x)) {
    const std::size_t n = *m ? (*m)->size() : 0;
    head(out, 5, n);
    for (std::size_t k = 0; k < n; ++k) {
      const auto &[key, value] = (**m)[k];
      head(out, 3, key.size());
      out += key;
      encode(out, value, depth + 1);
    }
  } else {
    throw std::invalid_argument("cbor: a " + v.type_name() +
                                " doesn't cross to a WebAssembly guest");
  }
}

class Reader {
public:
  explicit Reader(std::string_view data) : data_(data) {}

  se::value_t item(int depth) {
    if (depth > kMaxCborDepth)
      throw std::invalid_argument("cbor: nested deeper than " + std::to_string(kMaxCborDepth));
    const auto initial = static_cast<unsigned char>(byte());
    const int major = initial >> 5;
    const int info = initial & 0x1f;
    if (major == 7)
      return simple(info);
    const std::uint64_t n = argument(info);
    switch (major) {
    case 0:
      if (n > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
        throw std::invalid_argument("cbor: an integer past int64");
      return se::value_t(static_cast<std::int64_t>(n));
    case 1:
      if (n > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
        throw std::invalid_argument("cbor: an integer past int64");
      return se::value_t(-static_cast<std::int64_t>(n) - 1);
    case 2:
      return se::make_bytes(std::string(take(n)));
    case 3:
      return se::value_t(std::string(take(n)));
    case 4: {
      std::vector<se::value_t> items;
      items.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(n, left())));
      for (std::uint64_t k = 0; k < n; ++k)
        items.push_back(item(depth + 1));
      return se::make_list(std::move(items));
    }
    case 5: {
      std::vector<std::pair<std::string, se::value_t>> entries;
      entries.reserve(static_cast<std::size_t>(std::min<std::uint64_t>(n, left())));
      for (std::uint64_t k = 0; k < n; ++k) {
        se::value_t key = item(depth + 1);
        std::string *text = std::get_if<std::string>(&key.v);
        if (!text)
          throw std::invalid_argument("cbor: a map key is a " + key.type_name() + ", not text");
        std::string name = std::move(*text);
        entries.emplace_back(std::move(name), item(depth + 1));
      }
      return se::make_dict(std::move(entries));
    }
    default: // 6: a tag, read through
      return item(depth + 1);
    }
  }

  bool done() const { return at_ == data_.size(); }

private:
  std::size_t left() const { return data_.size() - at_; }

  char byte() {
    if (at_ >= data_.size())
      throw std::invalid_argument("cbor: truncated");
    return data_[at_++];
  }

  std::string_view take(std::uint64_t n) {
    if (n > left())
      throw std::invalid_argument("cbor: truncated");
    const std::string_view s = data_.substr(at_, static_cast<std::size_t>(n));
    at_ += static_cast<std::size_t>(n);
    return s;
  }

  std::uint64_t be(int bytes) {
    std::uint64_t n = 0;
    for (int k = 0; k < bytes; ++k)
      n = (n << 8) | static_cast<unsigned char>(byte());
    return n;
  }

  std::uint64_t argument(int info) {
    if (info < 24)
      return static_cast<std::uint64_t>(info);
    switch (info) {
    case 24:
      return be(1);
    case 25:
      return be(2);
    case 26:
      return be(4);
    case 27:
      return be(8);
    case 31:
      throw std::invalid_argument("cbor: indefinite lengths aren't read");
    default:
      throw std::invalid_argument("cbor: a malformed head");
    }
  }

  se::value_t simple(int info) {
    switch (info) {
    case 20:
      return se::value_t(false);
    case 21:
      return se::value_t(true);
    case 22: // null
    case 23: // undefined
      return se::value_t();
    case 25:
      return se::value_t(half(static_cast<std::uint16_t>(be(2))));
    case 26: {
      const auto bits = static_cast<std::uint32_t>(be(4));
      float f = 0.0f;
      std::memcpy(&f, &bits, sizeof f);
      return se::value_t(static_cast<double>(f));
    }
    case 27: {
      const std::uint64_t bits = be(8);
      double d = 0.0;
      std::memcpy(&d, &bits, sizeof d);
      return se::value_t(d);
    }
    default:
      throw std::invalid_argument("cbor: an unsupported simple value " + std::to_string(info));
    }
  }

  static double half(std::uint16_t h) {
    const int exp = (h >> 10) & 0x1f;
    const int mant = h & 0x3ff;
    double v = 0.0;
    if (exp == 0)
      v = std::ldexp(mant, -24);
    else if (exp != 31)
      v = std::ldexp(mant + 1024, exp - 25);
    else
      v = mant == 0 ? std::numeric_limits<double>::infinity()
                    : std::numeric_limits<double>::quiet_NaN();
    return (h & 0x8000) ? -v : v;
  }

  std::string_view data_;
  std::size_t at_ = 0;
};

} // namespace

std::string to_cbor(const se::value_t &v) {
  std::string out;
  encode(out, v, 0);
  return out;
}

se::value_t from_cbor(std::string_view data) {
  Reader r(data);
  se::value_t v = r.item(0);
  if (!r.done())
    throw std::invalid_argument("cbor: bytes after the item");
  return v;
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
