// value_cbor.h — DSL values as CBOR, as they cross a WebAssembly guest's boundary
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.4). The codec is the core one
// (cvc/state/state_exec/value_cbor.h, which has the mapping); a guest gets a symbol as text.
#pragma once

#include <cstdint>
#include <cvc/state/state_exec/value_cbor.h>
#include <string>
#include <string_view>

namespace cvc {
namespace ariadne {
namespace wasm {

// A guest instance's host objects, by handle (§8.4.1).
using Handles = cvc::state_exec::cbor_handles;

// The CBOR of `v`, its host objects put in `handles` (cvc::state_exec::encode_cbor).
inline std::string to_cbor(const cvc::state_exec::value_t &v, Handles *handles = nullptr) {
  return cvc::state_exec::encode_cbor(v, {handles, /*symbols=*/false});
}

// The one CBOR item `data` holds, its handles looked up in `handles`
// (cvc::state_exec::decode_cbor).
inline cvc::state_exec::value_t from_cbor(std::string_view data, Handles *handles = nullptr) {
  return cvc::state_exec::decode_cbor(data, {handles, /*symbols=*/false});
}

constexpr int kMaxCborDepth = cvc::state_exec::kMaxCborDepth;

// The tag a handle crosses under: "arih", in CBOR's first-come range of tags.
constexpr std::uint64_t kHandleTag = cvc::state_exec::kCborHandleTag;

} // namespace wasm
} // namespace ariadne
} // namespace cvc
