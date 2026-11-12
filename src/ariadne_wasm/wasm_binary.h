// wasm_binary.h — what the host changes in a guest's binary before it runs it
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.5). Portable C++ over the module's bytes, so it is
// tested natively, though the browser backend (browser_runtime.cpp) is what uses it.
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace cvc {
namespace ariadne {
namespace wasm {

// `module` with its memory's maximum at most `max_bytes`, in whole 64 KiB pages (rounded down): a
// memory without a maximum gets one, and a larger one is lowered; a smaller one stays. So the
// engine refuses a memory.grow past the limit (the guest sees -1), as wasmtime's store limiter
// does natively. 0 caps nothing. `max_pages`, when given, gets the maximum the memory has after
// (64 Ki pages, wasm32's own, when nothing caps it; 0 when the module defines no memory).
//
// Empty, with `error` set, for what it can't cap: an imported memory, memory64, a shared memory,
// limits of a kind it doesn't know, a minimum above the cap, or bytes that aren't a module.
std::string cap_memory(std::string_view module, std::uint64_t max_bytes, std::string &error,
                       std::uint64_t *max_pages = nullptr);

} // namespace wasm
} // namespace ariadne
} // namespace cvc
