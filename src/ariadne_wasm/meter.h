// meter.h — the gas pass (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.5): counters injected into
// a guest's code, so its work counts the same in every runtime, natively and in the browser, and a
// runtime can end the guest's slice at the start of any block. ari_wasm_meter (meter_main.cpp) is
// the pass as a command, the install-time transform's first step (cvc_ariadne_wasm_transform).
//
// The pass adds three things to a module:
//   - an exported, mutable i64 global, "ari_gas": the gas left in the slice. It starts at
//     kStartGas, the start function's allowance, so the start function runs at instantiation
//     without a runtime reaching the global before the instance exists;
//   - an import, ari.gas (param i64) (no result). The guest calls it when the slice can't pay for
//     the block it is about to run, with that block's cost. The runtime ends the slice there, or
//     traps the guest, and before the guest goes on it sets ari_gas to at least that cost;
//   - at the start of each metered block, a charge of the block's cost C:
//       global.get ari_gas, i64.const C, i64.lt_u, if (i64.const C, call ari.gas) end,
//       global.get ari_gas, i64.const C, i64.sub, global.set ari_gas.
//
// A metered block starts wherever control can come from somewhere other than the instruction
// before it: a function's entry, and the instruction after a loop, if, else, end, or conditional
// branch (br_if, br_on_null, br_on_non_null). Its cost is paid before it runs, for everything up
// to the next block. So a branch out of it, a trap or a throw is charged in full.
//
// An instruction costs what wasmtime's fuel charges for it: nop, drop, block, loop, unreachable,
// return, else and end cost nothing, and everything else costs 1. A bulk instruction (memory.fill,
// memory.copy, table.fill, ...) costs 1 whatever its size. The instance's memory limit bounds it.
//
// The new import comes after the old function imports, so every defined function's index goes up
// by one wherever it is named: calls, ref.func, element segments, exports, the start function and
// the name section. DWARF and source maps describe code offsets that the pass moves, so they're
// dropped. A module that already imports ari.gas was metered before, and is left as it is.
//
// The pass reads the core module format, with the proposals wasmtime turns on by default: sign
// extension, saturating truncation, bulk memory, reference types, multi-value, SIMD and relaxed
// SIMD, tail calls, typed function references, threads, multiple memories, memory64,
// extended constants and the exception handling of try_table. It refuses what it can't read
// (the GC instructions, the legacy exception handling, components), with what it found.
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace cvc {
namespace ariadne {
namespace wasm {

// The names the pass adds.
inline constexpr std::string_view kGasModule = "ari";
inline constexpr std::string_view kGasImport = "gas";     // (param i64): the block's cost
inline constexpr std::string_view kGasGlobal = "ari_gas"; // mutable i64: the gas left in the slice
// What ari_gas starts at: the start function runs on it, and calls ari.gas past it (a trap).
inline constexpr std::uint64_t kStartGas = 100'000'000;

struct Metered {
  std::string bytes;         // the metered module
  bool already = false;      // it imported ari.gas: bytes is the module as it was
  std::uint64_t charges = 0; // the charges the pass put in
};

// Meter the binary module `module` into `out`. False, with `error` set, for a module the pass
// can't read. The output isn't validated: a runtime validates the module it compiles.
bool meter(std::string_view module, Metered &out, std::string &error);

} // namespace wasm
} // namespace ariadne
} // namespace cvc
