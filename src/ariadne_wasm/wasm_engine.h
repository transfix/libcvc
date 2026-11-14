// wasm_engine.h — WebAssembly guests in Ariadne documents: the `wasm` ScriptEngine
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.2–8.7, phase G1; script_engine.h). A document's
// wasm: block names a module; the engine instantiates it once per document, and its exports
// become functions the document's programs call. A call is run a slice of fuel at a time: as the
// calling program's own work, a slice per step of its process (ScriptLane::export_activation), or,
// for a -async call and a hook, as an activation of its own on the app's scheduler.
//
//   requires: [wasm]
//   wasm:
//     module: scorer.wasm            # beside the document (a .wat file is compiled from text)
//     export: [score_route, {name: rank, as: rank-routes}, {name: score, pure: true}]
//     hooks: {on_tick: tick, on: {replan: replan}, watch: {route: route_changed},
//             on_key: key, on_pointer: pointer, on_signal: signal, msg_recv: {jobs: take_job}}
//     imports: [state.read, call]    # narrows what the guest's calls reach; never widens
//                                    # (state.read, state.write: every state function that
//                                    # writes, msg, call, net, data.read, data.write)
//     limits: {memory_mb: 32}        # may lower ariadne.limits.wasm_max_memory, never raise it
//     priority: 5
//     wasi: {args: [--quiet]}        # WASI's argv after the module's name; dirs: and env: fail
//                                    # the load while no host grants them (§8.6.1)
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
//         (§8.6) with a CBOR list of arguments. The process parks if the call waits. >= 0: the
//         length of the result's CBOR; < 0: minus the length of the error's text, "Class:
//         message". reply() copies either out.
//       reply(dst: i32): copy the last call's result, or its error's text, to dst.
//       yield(): give the frame back; the call goes on in the next one.
//       fail(ptr, len: i32): end the activation with a WasmError, the text its message.
//   - The state imports (§8.4), module "ari": the lane's own state functions, the ones call()
//     reaches by name (so the same chroot, link: holes and imports: narrowing), called inside the
//     guest's slice, a step each, where call() ends the slice for the activation's loop to make
//     the call (ScriptSlice::call_now). A function the lane holds otherwise (a document function
//     by that name) is called as call() calls it. Each takes its path as text, (path, path_len),
//     then each value as CBOR, (ptr, len), an empty one nil, and returns as call() does:
//       state_get(path) -> i64           (state-get path)
//       state_exists(path) -> i64        (state-exists path)
//       state_children(path) -> i64      (state-children path)
//       state_data_get(path) -> i64      (state-data-get path): a host object comes as a handle
//       state_set(path, v) -> i64        (state-set path v)
//       state_data_set(path, v) -> i64   (state-data-set path v): a handle stores its object
//       state_delete(path) -> i64        (state-delete path)
//       state_swap(path, v) -> i64       (state-swap path v)
//       state_incr(path, d) -> i64       (state-incr path d); an empty d: (state-incr path)
//       state_cas(path, old, new) -> i64 (state-cas path old new)
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
//       volume_write(h, x, y, z, nx, ny, nz, src, len) -> i64: a box's voxels from src, x fastest,
//         in the volume's voxel type; the bytes read. It writes the guest's own copy: a volume
//         anyone else may hold (the state tree, a renderer) is copied at its first write, and h
//         names the copy from then on, so nothing else sees the write until the guest stores h
//         (state_data_set). One only the guest holds is written in place.
//       volume_new(spec, spec_len) -> i64: a new volume, zeroed, as its handle (a handle is
//         positive as an i64). spec is a CBOR map as data_info gives one: dims [nx, ny, nz];
//         voxel (unsigned char, unsigned short, unsigned int, float or double; unsigned char when
//         absent); bounds [x0, y0, z0, x1, y1, z1] (the volume's default when absent). Its bytes
//         count against the call's allocation limit, as a native's allocation does.
//     imports: without data.read refuses the readers as a PermissionError, and without
//     data.write the writers.
//   - `_initialize`, when exported, runs once after instantiation (a WASI reactor's setup), with
//     no host to call.
//   - WASI preview 1 (§8.6.1): every guest is linked the 46 functions of module
//     "wasi_snapshot_preview1", so a module built for wasm32-wasip1, or with wasi-sdk, loads in
//     every host. They are stubs (wasi_preview1.h): the clocks, random_get, sched_yield and
//     proc_exit work; poll_oneoff takes clock subscriptions alone and fires each at once, so a
//     sleep returns at once (the host never blocks in it); argv is the module's file name, then
//     the block's wasi: args; the environment is empty; stdin is empty, and stdout and stderr take
//     what they're given and drop it (at most 1024 iovecs a call); no fd is a preopened directory
//     (fd_prestat_get returns EBADF); and every other file, directory, socket and signal call, and
//     a poll on an fd, returns ENOTCAPABLE (76). A pointer outside the guest's memory is EFAULT,
//     not a trap. Each WASI call is charged a step, plus a step per kDataNodeBytes it moves.
//     proc_exit ends the guest, never the host: the call it was in returns nil for status 0 and
//     raises a WasmError with `exit` for any other, and the instance goes (wasi-libc's exit ran its
//     teardown), so every later call is a WasmError with `exited` until the document reloads. An
//     _initialize or ari_init that exits fails the load. A block's wasi: dirs: ([{host:, guest:,
//     mode: ro|rw}]) and env: ([NAME]) fail the load, naming the request, while the host grants
//     WASI no directory and no variable (arisrvx's --wasi-dir and --wasi-env come next).
//   - `ari_init`, when exported, runs at every load as an activation (it may call the host), to
//     its end, under the init: lane's caps, before the document's init:. A failure fails the load.
//
// An export called from init: (which can't wait) runs to its end there.
//
// A pure export ({name:, pure: true}) is also a function of the read lane (§8.6;
// ScriptLane::export_pure): a reactive expression (visible_when, a computed bind:) calls it, and
// its call runs to its end in that expression. Its state imports and its call() reach only the read
// lane's state readers and pure functions (a writer is a PermissionError), a yield goes straight
// on, and an instance busy with another activation is a RuntimeError at once. Its result is kept
// as the expression's is, until a path it read is written.
//
// The hooks (§8.7) call their export with [event], the dict the DSL's hook reads (ScriptLane):
// on_tick {frame}; on {event}; watch {path, value}; on_key and on_pointer the on_key:/on_pointer:
// event (kind, key, mods, repeat, x, y, dx, dy, button, clicks); on_signal {kind, signal, source},
// which takes the signal over as on_signal: does; msg_recv {channel, value}, a message on the
// channel as (msg-recv channel) would take it. Keys, the pointer, signals and messages arrive
// every one, in order.
//
// One activation per instance: each holds the instance's lock, `wasm:<stem>` under the
// document's prefix (ActivationOptions::lock), so the others wait for it as a program waits in
// with-lock: under their lane's lock timeout, with priority inheritance, and lock-query shows it.
// A call back into the guest from inside one of its own host calls, made in the same process, is
// refused with a RecursionError.
//
// Limits: a slice is ariadne.limits.wasm_slice_steps (64) DSL steps at
// ariadne.limits.wasm_gas_per_step (1000) units of fuel each (wasmtime's; in the browser, a gas
// check of the transformed guest is kFuelPerCheck, guest_runtime.h), and what a slice uses is
// charged as steps to the process it runs in (the calling program's, or the activation's), so an
// action's step cap is its gas cap. Memory:
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

// The engine, on this build's runtime (wasmtime; under Emscripten the page's own WebAssembly, which
// runs guests transformed ahead, cvc_ariadne_wasm_transform). Null with `error` set when it can't
// start. A host registers it once (register_script_engine) and hands it to each runtime it runs a
// document in (AppRuntime::use_script_engine).
std::shared_ptr<ScriptEngine> make_engine(std::string &error);

} // namespace wasm
} // namespace ariadne
} // namespace cvc
