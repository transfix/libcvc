// wasm_binary.h — what the host reads in a guest's binary, and changes in it, before it runs it
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.5). Portable C++ over the module's bytes, so it is
// tested natively, though the browser backend (browser_runtime.cpp) is what uses it.
#pragma once

#include "guest_runtime.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
namespace ariadne {
namespace wasm {

// A function a module imports, and its type. A value type is its byte in the binary format (0x7f
// i32, 0x7e i64, 0x7d f32, 0x7c f64; a reference type its first byte), one a parameter or result.
struct FunctionImport {
  std::string module;
  std::string name;
  std::string params;
  std::string results;
};

// The functions `module` imports, in its order, into `out`. False, with `error` set, when its type
// or import section can't be read, or holds a type of a kind the host doesn't know (the GC
// proposal's). The browser checks a guest's imports against the host's with it, as wasmtime's
// linker checks them natively: JS checks no import's signature.
bool function_imports(std::string_view module, std::vector<FunctionImport> &out,
                      std::string &error);

// Whether the guest declares each import it takes (`declared`, function_imports') as the host
// defines it (`host`), under the same module (`ari`, WASI's `wasi_snapshot_preview1`): the
// parameters (an i64 where Import::wide marks one) and the result (an i64, an i32 where
// Import::returns_i32 says so, or none). ari.gas is the runtime's own, (param i64). False, with
// `error` naming the import and both types, for one the host doesn't define or defines otherwise.
// wasmtime's linker checks the same natively. The browser checks with it because JS checks no
// import's signature, and WAMR because the guest it runs is the transformed one, whose
// optimization drops an import it never calls: the check is of the guest as it was written.
bool imports_match(const std::vector<FunctionImport> &declared, const std::vector<Import> &host,
                   std::string &error);

// Whether `module` has a start section (8): a function the runtime runs at instantiation, to its
// end, on the start's own allowance of gas (kStartGas, meter.h), before the host can slice it. A
// client's guest may not have one (wasm_engine.h). False, with `error` set, for bytes that aren't a
// module or a section that runs past the module's end.
bool has_start(std::string_view module, bool &start, std::string &error);

// `module` without its start section, and with the start function exported as `name` instead, so
// the host runs it once the instance's imports are in place: WAMR runs a start function as it
// instantiates, before its wasm-c-api has wired the host's imports, and a call of one there fails
// as unlinked. `had` says whether there was a start section; without one the module comes back as
// it is. Empty, with `error` set, for bytes that aren't a module, a start or export section that
// can't be read, or a module that exports `name` itself.
std::string start_as_export(std::string_view module, std::string_view name, bool &had,
                            std::string &error);

// A function type as the text format writes it: " (param i32 i32) (result i64)", "" for () -> ().
std::string type_text(std::string_view params, std::string_view results);

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
