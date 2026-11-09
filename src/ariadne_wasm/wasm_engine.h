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
//                                    # (state.read, state.write, msg, call, net, data.read,
//                                    # data.write)
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
//   - Host objects by handle (§8.4.1). A value holding a host object (a data_object: a volume, a
//     mesh, a height field) crosses as a handle, an unsigned integer under CBOR tag kHandleTag
//     (value_cbor.h), and comes back as the object when the guest hands the handle back. The
//     instance's table keeps the object, the version it was handed, until handle_drop. The data
//     calls copy only what they're asked for, as raw little-endian arrays, into the guest's
//     memory at dst; each returns the bytes written, or < 0 as call does (reply() copies the
//     error's text), and is charged a step per kDataNodeBytes (256) moved. Imports, module "ari",
//     h an i64 handle and the rest i32s:
//       handle_drop(h): let it go; one that names nothing is ignored.
//       data_info(h) -> i64: a CBOR map (reply()): type, and the sizes (a volume's dims, voxel,
//         voxel_bytes, bounds; a mesh's points, normals, tris, quads, lines, bounds; a height
//         field's rows, cols, x0, y0, dx, dy). No data moves.
//       volume_read(h, x, y, z, nx, ny, nz, lod, dst, len) -> i64: a box's voxels, every 2^lod-th
//         along each axis, x fastest, in the volume's voxel type.
//       volume_sample(h, points, n, dst, len) -> i64: f64 values interpolated at n points (f64
//         x, y, z, object coordinates).
//       mesh_read(h, attr_ptr, attr_len, first, count, dst, len) -> i64: a range of positions or
//         normals (f64 x 3), indices (u32 x 3), quads (u32 x 4) or lines (u32 x 2).
//       heightfield_read(h, row, col, rows, cols, dst, len) -> i64: f32 heights, row by row.
//     imports: without data.read refuses them as a PermissionError. Writes come later.
//   - `_initialize`, when exported, runs once after instantiation (a WASI reactor's setup), with
//     no host to call.
//   - `ari_init`, when exported, runs at every load as an activation (it may call the host), to
//     its end, under the init: lane's caps, before the document's init:. A failure fails the load.
//
// An export called from init: (which can't wait) runs to its end there.
//
// Limits: a slice is ariadne.limits.wasm_slice_steps (64) DSL steps at
// ariadne.limits.wasm_gas_per_step (1000) units of wasmtime fuel each, and what a slice uses is
// charged to its activation as steps, so an action's step cap is its gas cap. Memory:
// ariadne.limits.wasm_max_memory bytes (256 MiB); a memory.grow past it gives the guest -1, and a
// trap with the memory at its limit is a MemoryError. Any other trap is a WasmError, with `trap`
// its text.
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
