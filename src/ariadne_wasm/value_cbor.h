// value_cbor.h — DSL values as CBOR (RFC 8949), the encoding values cross a WebAssembly guest's
// boundary in (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.4).
//
// The mapping mirrors state-tree values:
//   nil        null               bool    false / true
//   integer    unsigned / negative integer (64-bit)
//   float      float64 (float16 and float32 are read too)
//   string     text string        bytes   byte string
//   symbol     text string (it comes back as a string)
//   list       array              dict    map with text keys, in order
//   host object (a data_object): a handle, kHandleTag over an unsigned integer (§8.4.1), when the
//   call has a handle table; without one it does not cross
// A closure, a native function or a generator does not cross. Other tags are read through: a
// tagged item decodes as its content.
#pragma once

#include <cstdint>
#include <cvc/state/state_exec/types.h>
#include <string>
#include <string_view>

namespace cvc {
namespace ariadne {
namespace wasm {

// A guest instance's host objects, by handle (§8.4.1).
class Handles {
public:
  virtual ~Handles() = default;
  // A new handle for `obj`, which the table keeps until the guest drops it. Throws
  // std::invalid_argument when the table is full.
  virtual std::uint64_t put(const cvc::state_exec::data_object_ptr &obj) = 0;
  // The object `handle` names; null for one that was dropped or never given.
  virtual cvc::state_exec::data_object_ptr get(std::uint64_t handle) const = 0;
};

// The CBOR of `v`, its host objects put in `handles`. Throws std::invalid_argument for a value that
// does not cross (naming it), or one nested deeper than kMaxCborDepth.
std::string to_cbor(const cvc::state_exec::value_t &v, Handles *handles = nullptr);

// The one CBOR item `data` holds, its handles looked up in `handles`. Throws std::invalid_argument
// on a malformed or truncated item, trailing bytes, an indefinite length, a map key that is not
// text, an integer outside int64, nesting deeper than kMaxCborDepth, or a handle that names
// nothing (or any handle, without a table).
cvc::state_exec::value_t from_cbor(std::string_view data, Handles *handles = nullptr);

constexpr int kMaxCborDepth = 64;

// The tag a handle crosses under: "arih", in CBOR's first-come range of tags.
constexpr std::uint64_t kHandleTag = 0x61726968;

} // namespace wasm
} // namespace ariadne
} // namespace cvc
