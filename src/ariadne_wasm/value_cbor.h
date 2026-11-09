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
// A closure, a native function, a generator or a host object (a data_object; §8.4.1 passes those
// as handles) does not cross. Tags are read through: a tagged item decodes as its content.
#pragma once

#include <cvc/state/state_exec/types.h>
#include <string>
#include <string_view>

namespace cvc {
namespace ariadne {
namespace wasm {

// The CBOR of `v`. Throws std::invalid_argument for a value that does not cross (naming it), or
// one nested deeper than kMaxCborDepth.
std::string to_cbor(const cvc::state_exec::value_t &v);

// The one CBOR item `data` holds. Throws std::invalid_argument on a malformed or truncated item,
// trailing bytes, an indefinite length, a map key that is not text, an integer outside int64, or
// nesting deeper than kMaxCborDepth.
cvc::state_exec::value_t from_cbor(std::string_view data);

constexpr int kMaxCborDepth = 64;

} // namespace wasm
} // namespace ariadne
} // namespace cvc
