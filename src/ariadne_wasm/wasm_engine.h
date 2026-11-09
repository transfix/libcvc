// wasm_engine.h — WebAssembly guests in Ariadne documents: the `wasm` ScriptEngine
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.2–8.7, phase G1; script_engine.h). A document's
// wasm: block names a module; the engine instantiates it once per document, and its exports
// become functions the document's programs call, each call an activation on the app's scheduler,
// run a slice of fuel at a time.
//
//   requires: [wasm]
//   wasm:
//     module: scorer.wasm            # beside the document (a .wat file is compiled from text)
//     export: [score_route, {name: rank, as: rank-routes}]
//     hooks: {on_tick: tick, on: {replan: replan}, watch: {route: route_changed}}
//     imports: [state.read, call]    # narrows what the guest's calls reach; never widens
//     limits: {memory_mb: 32}        # may lower ariadne.limits.wasm_max_memory, never raise it
//     priority: 5
//
// The guest ABI (§8.4), as G1 builds it:
//   - Values cross as CBOR (value_cbor.h) in the guest's memory, as a pointer and a length. The
//     guest exports `memory`, and `ari_alloc(len: i32) -> i32`, which gives the host room for a
//     value it hands over.
//   - An export the block names takes (ptr: i32, len: i32), the CBOR list of the call's
//     arguments (a hook's: [event]), and returns an i64: its result's CBOR, the pointer in the
//     high 32 bits and the length in the low 32 (length 0: nil).
//   - Imports, module "ari":
//       call(name_ptr, name_len, args_ptr, args_len: i32) -> i64: call a function of the lane
//         (§8.6) with a CBOR list of arguments. The activation parks if the call waits. >= 0: the
//         length of the result's CBOR; < 0: minus the length of the error's text, "Class:
//         message". reply() copies either out.
//       reply(dst: i32): copy the last call's result, or its error's text, to dst.
//       yield(): give the frame back; the call goes on in the next one.
//       fail(ptr, len: i32): end the activation with a WasmError, the text its message.
//   - `_initialize`, when exported, runs once after instantiation (a WASI reactor's setup).
//
// Limits: a slice is ariadne.limits.wasm_slice_steps (64) DSL steps at
// ariadne.limits.wasm_gas_per_step (1000) units of wasmtime fuel each, and what a slice uses is
// charged to its activation as steps, so an action's step cap is its gas cap. Memory:
// ariadne.limits.wasm_max_memory bytes (256 MiB). A trap is a WasmError, with `trap` its text.
#pragma once

#include <cvc/ariadne/script_engine.h>
#include <memory>
#include <string>

namespace cvc {
namespace ariadne {
namespace wasm {

// The engine, on this build's runtime (wasmtime). Null with `error` set when the runtime can't
// start. A host registers it once (register_script_engine) and hands it to each runtime it runs a
// document in (AppRuntime::use_script_engine).
std::shared_ptr<ScriptEngine> make_engine(std::string &error);

} // namespace wasm
} // namespace ariadne
} // namespace cvc
