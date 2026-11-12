// wasi_preview1.h — WASI preview 1 for every wasm guest, as our own imports
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.6.1, phase G6): the 46 functions of module
// "wasi_snapshot_preview1", which a module built for wasm32-wasip1 (Rust, C with wasi-sdk, a
// Python or MicroPython built for WASI) imports whether or not it uses them. They are stubs,
// backend-neutral, on guest_runtime.h's Import:
//   - the clocks (realtime, monotonic; the CPU-time clocks read as monotonic), random numbers
//     (std::random_device), sched_yield and proc_exit work. proc_exit ends the guest: the engine
//     reads it as the instance's end (wasi-libc's exit has run its teardown);
//   - poll_oneoff takes clock subscriptions only, and fires each at once, so a sleep returns at
//     once and never blocks the host; an fd's subscription is ENOTCAPABLE;
//   - args are the block's: the module's file name, then wasi: args;
//   - the environment is empty;
//   - stdin is empty (fd_read(0) reads nothing), and stdout and stderr take what they're given
//     and drop it (fd_write(1|2)); the three are character devices to fd_fdstat_get and
//     fd_filestat_get. A call takes at most 1024 iovecs (wasi-libc's IOV_MAX); more is EINVAL;
//   - fd_prestat_get and fd_prestat_dir_name return EBADF for every fd: there are no preopened
//     directories, and wasi-libc's scan of them (__wasilibc_populate_preopens) stops at the first
//     EBADF, where any other errno exits the guest at startup;
//   - every other file, directory, socket and signal call returns ENOTCAPABLE.
// A pointer outside the guest's memory returns EFAULT, not a trap. Each call counts in
// WasiState::calls, and the bytes it moves (the iovecs and poll records it reads too) in
// WasiState::moved, for the engine to charge as steps. The stubs need no activation: a reactor's
// _initialize calls them.
#pragma once

#include "guest_runtime.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {
namespace wasm {

// The module the imports are under.
inline constexpr char kWasiModule[] = "wasi_snapshot_preview1";

// What one instance's WASI imports share with the engine.
struct WasiState {
  std::vector<std::string> args; // argv: the module's file name, then the block's wasi: args
  std::uint64_t calls = 0;       // WASI calls since the engine last charged them
  std::uint64_t moved = 0;       // bytes they moved since then
  std::optional<int> exit;       // proc_exit's status, once the guest called it
};

// The 46 imports, each a stub over `w`, which must outlive the instance's calls.
std::vector<Import> wasi_imports(WasiState &w);

} // namespace wasm
} // namespace ariadne
} // namespace cvc
