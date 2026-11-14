// guest_runtime.h — the WebAssembly runtimes behind the wasm engine, behind one interface
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.3): compile a module, instantiate it with the
// host's imports, start a call of an export, and run it a slice of fuel at a time, pausing at an
// import that has to wait. wasmtime is the backend natively (wasmtime_runtime.cpp); WAMR and the
// browser's engine come later (G2, G3) under the same calls.
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

// A guest's linear memory, as an import sees it: valid only while the import runs (memory.grow can
// move it).
class GuestMemory {
public:
  virtual ~GuestMemory() = default;
  virtual std::span<std::uint8_t> bytes() = 0;
  // [at, at + n) inside the memory, or a trap's message.
  bool span_ok(std::uint64_t at, std::uint64_t n, std::string &why) {
    const std::span<std::uint8_t> b = bytes();
    if (at > b.size() || n > b.size() - at) {
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

// The host's side of one import, in `module` ("ari", or WASI's "wasi_snapshot_preview1"). Its
// parameters are i32s (or i64s, as `wide` marks them), widened (an i32 sign-extended: read a
// pointer as its uint32), and it returns an i64, an i32 (`returns_i32`) or nothing. A sync import
// answers at once; an async one returns the PendingImport the guest waits on (null: answered at
// once with `value`). Either may set `trap` to trap the guest.
struct Import {
  std::string module = "ari";
  std::string name;
  int params = 0;
  std::uint32_t wide = 0;   // bit i set: parameter i is an i64 (a handle)
  bool returns = false;     // an i64
  bool returns_i32 = false; // with `returns`: the result is an i32 (WASI's errno), not an i64
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
  // The guest's exported memory ("memory"), between calls. Empty without one.
  virtual std::span<std::uint8_t> memory() = 0;
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
  std::uint64_t max_memory = 0; // bytes of linear memory, 0: the runtime's own default
};

class Runtime {
public:
  virtual ~Runtime() = default;
  virtual std::string name() const = 0; // "wasmtime 45.0.0"
  // Compile `bytes`: a binary module, or the WebAssembly text format when `text`. Null with
  // `error` set.
  virtual std::shared_ptr<Module> compile(std::string_view bytes, bool text,
                                          std::string &error) = 0;
  // Instantiate `module` with `imports` (each under its Import::module; an import the module needs
  // that isn't there, or is there with another type, fails the instantiation). Runs the module's
  // start function. Null with `error` set.
  virtual std::unique_ptr<Instance> instantiate(const std::shared_ptr<Module> &module,
                                                std::vector<Import> imports,
                                                const InstanceLimits &limits,
                                                std::string &error) = 0;
};

// This build's runtime (wasmtime natively). Null with `error` set when it can't start.
std::unique_ptr<Runtime> make_runtime(std::string &error);

} // namespace wasm
} // namespace ariadne
} // namespace cvc
