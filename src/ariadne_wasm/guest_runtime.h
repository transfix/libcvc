// guest_runtime.h — the WebAssembly runtimes behind the wasm engine, behind one interface
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.3): compile a module, instantiate it with the
// host's imports, start a call of an export, and run it a slice of fuel at a time, pausing at an
// import that has to wait. wasmtime is the backend natively (wasmtime_runtime.cpp), and the page's
// own WebAssembly in the browser (browser_runtime.cpp, G2); WAMR comes later (G3) under the same
// calls.
//
// The browser runs guests transformed at install (cvc_ariadne_wasm_transform, ariadne_wasm.cmake:
// binaryen's `wasm-opt --log-execution=ari --asyncify --pass-arg=asyncify-imports@ari.* -O1`):
//   - a gas check, an import `ari.log_execution(i32)`, at every function's entry, every loop's
//     header and every return. Each backend defines it: the browser counts kFuelPerCheck units of
//     fuel a check, and pauses the guest there when the slice's fuel is spent; wasmtime, whose own
//     fuel meters the guest, defines it as nothing;
//   - Asyncify's exports (asyncify_start_unwind and the rest), with every `ari` import a point the
//     guest may pause at: the browser unwinds the guest's stack into its own memory there, and
//     rewinds it to the same point on the next poll.
// wasmtime runs the same transformed guests as it runs the originals.
//
// Everything here runs on the thread that drains the runtime. One call runs on an instance at a
// time (a store allows one future), and the engine enforces it.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
namespace ariadne {
namespace wasm {

// The fuel a transformed guest's gas check costs where the checks meter it (the browser): a loop's
// iteration, or a call, is about this many instructions.
constexpr std::uint64_t kFuelPerCheck = 16;

// A guest's linear memory, as the host reads and writes it. The host copies in and out: in the
// browser the memory is another WebAssembly.Memory than the host's, so no host pointer reaches it.
// Valid while an import runs, and between calls (Instance::memory()); a memory.grow changes size().
class GuestMemory {
public:
  virtual ~GuestMemory() = default;
  // Its size in bytes (0 without one).
  virtual std::uint64_t size() = 0;
  // Copy [at, at + out.size()) out, or [at, at + in.size()) in; false when that isn't inside the
  // memory (span_ok first, for the trap's message).
  virtual bool read(std::uint64_t at, std::span<std::uint8_t> out) = 0;
  virtual bool write(std::uint64_t at, std::span<const std::uint8_t> in) = 0;
  // [at, at + n) as a host pointer, where the runtime shares the host's address space (wasmtime),
  // valid until the guest runs or the memory grows; null where it doesn't (the browser), or when
  // it isn't inside the memory.
  virtual std::uint8_t *map(std::uint64_t at, std::uint64_t n) = 0;
  // [at, at + n) inside the memory, or a trap's message.
  bool span_ok(std::uint64_t at, std::uint64_t n, std::string &why) {
    const std::uint64_t size = this->size();
    if (at > size || n > size - at) {
      why = "an out-of-bounds pointer into the guest's memory";
      return false;
    }
    return true;
  }
};

// An import that has to wait (an async host function): the guest stays paused in it until the host
// sets `done`, and then gets `value` as the import's result.
struct PendingImport {
  bool done = false;
  std::int64_t value = 0;
};

// The host's side of one import, in module "ari". Its parameters are i32s (or i64s, as `wide`
// marks them), widened, and it returns an i64 or nothing. A sync import answers at once; an async
// one returns the PendingImport the guest waits on (null: answered at once with `value`). Either
// may set `trap` to trap the guest.
struct Import {
  std::string name;
  int params = 0;
  std::uint32_t wide = 0; // bit i set: parameter i is an i64 (a handle)
  bool returns = false;   // an i64
  std::function<std::int64_t(GuestMemory &, std::span<const std::int64_t>, std::string &trap)> sync;
  std::function<std::shared_ptr<PendingImport>(GuestMemory &, std::span<const std::int64_t>,
                                               std::string &trap)>
      async;
};

class Call;

class Instance {
public:
  virtual ~Instance() = default;
  // Whether the guest exports a function `name`.
  virtual bool has_export(const std::string &name) const = 0;
  // The guest's exported memory ("memory"), between calls: size() 0 without one.
  virtual GuestMemory &memory() = 0;
  // Whether the memory is as large as the instance's limit lets it grow: a memory.grow past it was
  // refused (the guest saw -1).
  virtual bool memory_at_limit() = 0;
  // Start a call of export `name` with i32/i64 arguments (as the export takes them); nothing runs
  // until the call is polled. Null with `error` set when there is no such export.
  virtual std::unique_ptr<Call> start(const std::string &name, std::vector<std::int64_t> args,
                                      std::string &error) = 0;
  // Run export `name` to its end now, with at most `fuel`: for the guest's own setup
  // (_initialize, ari_alloc), which waits on no import. True with its first result in `result`,
  // or false with `error` set (a trap, out of fuel, an import that waited). Adds the fuel it used
  // to `used`.
  virtual bool run(const std::string &name, std::vector<std::int64_t> args, std::uint64_t fuel,
                   std::int64_t &result, std::uint64_t &used, std::string &error) = 0;
};

// A call of an export, run a slice at a time.
class Call {
public:
  enum class State {
    done,    // it returned: result()
    sliced,  // the slice's fuel ran out: poll again
    waiting, // in an async import whose PendingImport isn't done: poll once it is
    trapped  // trap()
  };
  virtual ~Call() = default;
  // Run until the call returns, `fuel` more is used, or an async import waits.
  virtual State poll(std::uint64_t fuel) = 0;
  virtual std::uint64_t fuel_used() const = 0; // by the last poll
  virtual std::int64_t result() const = 0;     // the first result (0 when it returns none)
  virtual const std::string &trap() const = 0;
};

class Module {
public:
  virtual ~Module() = default;
};

struct InstanceLimits {
  // Bytes of linear memory, 0: the runtime's own default. wasmtime's store limiter refuses a
  // memory.grow past it; the browser lowers the module's own maximum to it (cap_memory).
  std::uint64_t max_memory = 0;
};

class Runtime {
public:
  virtual ~Runtime() = default;
  virtual std::string name() const = 0; // "wasmtime 45.0.0"
  // Compile `bytes`: a binary module, or the WebAssembly text format when `text` (which the
  // browser refuses: it compiles binaries only). Null with `error` set.
  virtual std::shared_ptr<Module> compile(std::string_view bytes, bool text,
                                          std::string &error) = 0;
  // Instantiate `module` with `imports` (module "ari"; an import the module needs that isn't
  // there fails the instantiation). Runs the module's start function. Null with `error` set.
  virtual std::unique_ptr<Instance> instantiate(const std::shared_ptr<Module> &module,
                                                std::vector<Import> imports,
                                                const InstanceLimits &limits,
                                                std::string &error) = 0;
};

// This build's runtime (wasmtime natively, the page's WebAssembly under Emscripten). Null with
// `error` set when it can't start.
std::unique_ptr<Runtime> make_runtime(std::string &error);

} // namespace wasm
} // namespace ariadne
} // namespace cvc
