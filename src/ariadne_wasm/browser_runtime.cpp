// browser_runtime.cpp — guest_runtime.h on the page's own WebAssembly, under Emscripten (G2,
// docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.5).
//
// A guest is a WebAssembly.Instance beside the host's own module, made in JS (EM_JS) with an `ari`
// import object: a trampoline per Import into cvc_ari_wasm_import, and the gas check. Its memory
// is a WebAssembly.Memory of its own, so the host copies in and out of it (BrMemory) and map() is
// null. A page has one thread and no second stack, so a guest pauses by Asyncify: it was
// transformed at install (guest_runtime.h, cvc_ariadne_wasm_transform), and a pause unwinds its
// stack into a room in its own memory and returns to the host. The next poll calls the export
// again in rewind mode, which takes the guest back to where it paused.
//   - At a gas check (ari.log_execution) once the slice's checks are spent: sliced. A check costs
//     kFuelPerCheck units of fuel, so a slice of `fuel` runs ceil(fuel / kFuelPerCheck) checks.
//   - In an async import whose PendingImport isn't done: waiting. Rewound, the import returns the
//     PendingImport's value.
// Run to its end (run(), a start function), a guest can't pause: out of checks or waiting, it
// traps. The memory limit is the module's own maximum, lowered to InstanceLimits::max_memory
// before the module is compiled (cap_memory), so the engine refuses the memory.grow. A .wat module
// (there is no WAT compiler here), or a guest that wasn't transformed, is refused.
//
// The room is InstanceLimits::unwind_bytes (ariadne.limits.wasm_unwind_bytes, kUnwindBytes when
// unset), from the guest's ari_alloc at its first call. Asyncify writes a paused stack into it
// unchecked, and finds one past it only after it has written over the guest's memory beyond the
// room: the guest is broken then, and refuses every later call (its document's reload makes a new
// one).
//
// An export's arguments go to the guest as i32s, as the ABI's take them (§8.4). JS checks no
// import's signature, so the guest's `ari` imports are checked against the host's Imports before
// it is instantiated (function_imports, wasm_binary.h), as wasmtime's linker checks them natively:
// a module that declares one with other parameters or another result is refused.
//
// A module is compiled once for each memory cap it runs under, and kept with its BrModule. The
// compile is synchronous, so a guest past the 8 MiB Chrome compiles on the page's own thread
// fails there (an asynchronous compile, awaited by the document's load, is left for ariglx-web).
//
// The JS side's state is the page's, as Module.cvcMedia is: Module.cvcWasmGuests[<the
// BrInstance's address>], made at instantiation and deleted with the instance, and
// Module.cvcWasmModules[<the BrModule's address>], deleted with the module. The BrInstance owns
// the scratch buffer the two sides pass i64s through.

#ifdef __EMSCRIPTEN__

#include "guest_runtime.h"
#include "wasm_binary.h"

#include <algorithm>
#include <array>
#include <emscripten.h>
#include <exception>
#include <utility>

// Only == and != in these bodies: clang-format and the preprocessor read them as C++ tokens and
// would split === and !== (see src/cvc/net/net_policy.cpp).
// clang-format off

// The scratch buffer's slots (kScratch i64s; the C++ side's names are below): [0, 16) an import's
// arguments, then its answer at 0 (a trap's text at 0 and 1); 16 the answer a rewound import gets;
// 17 the gas checks the last call ran; 18 its result; [20, 32) an export's arguments.

// A message for the C++ side (cvc_wasm_text): the last failure's.
EM_JS(int, cvc_wasm_text, (char *dst, int cap), {
  var b = new TextEncoder().encode(Module.cvcWasmText || "");
  if (dst && cap > 0)
    HEAPU8.set(b.subarray(0, Math.min(cap, b.length)), dst >>> 0);
  return b.length;
});

// Whether `bytes` is a module the page compiles; when not, why.
EM_JS(int, cvc_wasm_check, (const void *bytes, int n), {
  var p = bytes >>> 0;
  var b = HEAPU8.slice(p, p + n);
  if (WebAssembly.validate(b))
    return 1;
  Module.cvcWasmText = "not a valid WebAssembly module";
  try {
    new WebAssembly.Module(b);
  } catch (e) {
    Module.cvcWasmText = String((e && e.message) || e);
  }
  return 0;
});

// Instantiate the module `module_ctx` (the BrModule) under the memory cap `cap`, whose bytes are
// `bytes` (compiled the first time, then taken from Module.cvcWasmModules): 1, or 0 with the
// reason in cvcWasmText, or 2 for a module without the transform's gas checks and Asyncify
// exports. `spec` lists the imports in order, "name:returns;" each (returns 1: an i64).
EM_JS(int, cvc_wasm_instantiate, (void *ctx, void *module_ctx, const void *bytes, int n,
                                  double cap, const char *spec, int spec_len, void *scratch,
                                  double start_checks, double unwind_bytes), {
  var guests = Module.cvcWasmGuests = Module.cvcWasmGuests || {};
  var mods = Module.cvcWasmModules = Module.cvcWasmModules || {};
  var g = {ex: null, mem: null, scratch: scratch >>> 0, left: start_checks, checks: 0,
           sliced: false, pausable: false, rewinding: false, buf: 0, unwind: unwind_bytes,
           broken: ""};
  var view = function() { return new BigInt64Array(HEAPU8.buffer, g.scratch, 32); };
  // Unwind the guest's stack into the room at buf: Asyncify's header (where it writes next, and
  // where the room ends), then the room, [buf + 8, buf + 8 + unwind).
  var pause = function() {
    var m = new Uint32Array(g.mem.buffer, g.buf, 2);
    m[0] = g.buf + 8;
    m[1] = g.buf + 8 + g.unwind;
    g.ex.asyncify_start_unwind(g.buf);
  };
  // The rewind reached the call it paused in: the guest goes on from there.
  var resumed = function() {
    g.rewinding = false;
    g.ex.asyncify_stop_rewind();
  };
  var trampoline = function(index, ret) {
    return function() {
      if (g.rewinding) {
        resumed();
        return ret ? view()[16] : undefined;
      }
      var s = view();
      for (var i = 0; i < arguments.length; ++i)
        s[i] = BigInt(arguments[i]);
      var status = _cvc_ari_wasm_import(ctx, index, arguments.length);
      s = view(); // the host's memory may have grown
      if (status == 0)
        return ret ? s[0] : undefined;
      if (status == 1) {
        var p = Number(s[0]), len = Number(s[1]);
        throw new WebAssembly.RuntimeError(new TextDecoder().decode(HEAPU8.slice(p, p + len)));
      }
      pause(); // 2: it waits
      return ret ? 0n : undefined;
    };
  };
  try {
    var q = spec >>> 0;
    var compiled = mods[module_ctx] = mods[module_ctx] || {};
    var mod = compiled[cap];
    if (!mod) {
      var p = bytes >>> 0;
      try {
        mod = new WebAssembly.Module(HEAPU8.slice(p, p + n));
      } catch (e) {
        var why = String((e && e.message) || e);
        if (n > 8 * 1024 * 1024)
          why += " (the guest is " + Math.ceil(n / 1048576) +
                 " MiB: Chrome compiles at most 8 MiB on the page's own thread)";
        throw new Error(why);
      }
      compiled[cap] = mod;
    }
    var exported = WebAssembly.Module.exports(mod), imported = WebAssembly.Module.imports(mod);
    var transformed = imported.some(function(i) {
      return i.module == "ari" && i.name == "log_execution" && i.kind == "function";
    });
    ["asyncify_start_unwind", "asyncify_stop_unwind", "asyncify_start_rewind",
     "asyncify_stop_rewind", "asyncify_get_state"].forEach(function(name) {
      if (!exported.some(function(e) { return e.name == name && e.kind == "function"; }))
        transformed = false;
    });
    if (!transformed)
      return 2;
    var ari = {};
    var list = new TextDecoder().decode(HEAPU8.slice(q, q + spec_len)).split(";");
    for (var i = 0; i < list.length; ++i)
      if (list[i]) {
        var f = list[i].split(":");
        ari[f[0]] = trampoline(i, f[1] == "1");
      }
    ari.log_execution = function() {
      if (g.rewinding) {
        resumed();
        return;
      }
      ++g.checks;
      if (--g.left >= 0)
        return;
      if (!g.pausable)
        throw new WebAssembly.RuntimeError("ran out of fuel");
      g.sliced = true;
      pause();
    };
    guests[ctx] = g; // before the start function, whose imports reach the host
    var inst = new WebAssembly.Instance(mod, {ari: ari});
    g.ex = inst.exports;
    g.mem = inst.exports.memory instanceof WebAssembly.Memory ? inst.exports.memory : null;
    return 1;
  } catch (e) {
    delete guests[ctx];
    Module.cvcWasmText = String((e && e.message) || e);
    return 0;
  }
});

EM_JS(void, cvc_wasm_drop, (void *ctx), {
  if (Module.cvcWasmGuests)
    delete Module.cvcWasmGuests[ctx];
});

// The module's compiles (cvc_wasm_instantiate's), dropped with the BrModule.
EM_JS(void, cvc_wasm_module_drop, (void *module_ctx), {
  if (Module.cvcWasmModules)
    delete Module.cvcWasmModules[module_ctx];
});

// The parameters export `name` takes, or -1 when it exports no such function.
EM_JS(int, cvc_wasm_arity, (void *ctx, const char *name, int name_len), {
  var g = Module.cvcWasmGuests[ctx], p = name >>> 0;
  var f = g.ex[new TextDecoder().decode(HEAPU8.slice(p, p + name_len))];
  return typeof f == "function" ? f.length : -1;
});

// Call export `name` with the arguments at slot 20, rewinding it to where it paused when `rewind`,
// with `left` gas checks: 0 it returned (its result at 18), 1 it paused at a gas check, 2 in an
// import that waits, 3 it trapped (the trap's text in cvcWasmText). The checks it ran go to 17. A
// broken guest (its paused stack ran past the room) traps at once.
EM_JS(int, cvc_wasm_call, (void *ctx, const char *name, int name_len, int nargs, int rewind,
                           int pausable, double left), {
  var g = Module.cvcWasmGuests[ctx], p = name >>> 0;
  var s = new BigInt64Array(HEAPU8.buffer, g.scratch, 32);
  s[17] = 0n;
  if (g.broken) {
    Module.cvcWasmText = g.broken;
    return 3;
  }
  var f = g.ex[new TextDecoder().decode(HEAPU8.slice(p, p + name_len))];
  var args = [];
  for (var i = 0; i < nargs; ++i)
    args.push(Number(BigInt.asIntN(32, s[20 + i])));
  g.left = left;
  g.checks = 0;
  g.sliced = false;
  g.pausable = !!pausable;
  // After an unwind, whether the paused stack ran past the room. Asyncify writes it unchecked and
  // checks only in asyncify_stop_unwind, after it has written over the guest's memory past the
  // room, whatever the guest's allocator had put there: the guest is broken.
  var overran = function() {
    var m = new Uint32Array(g.mem.buffer, g.buf, 2);
    if (m[0] <= m[1])
      return false;
    g.broken = "a paused call's stack took " + (m[0] - g.buf - 8) + " bytes, past its room's " +
               g.unwind + " (ariadne.limits.wasm_unwind_bytes), and wrote over the guest's " +
               "memory after the room: the guest can't run again until its document is reloaded";
    return true;
  };
  var r, status = 0;
  try {
    if (rewind) {
      g.rewinding = true;
      g.ex.asyncify_start_rewind(g.buf);
    }
    r = f.apply(null, args);
    if (g.ex.asyncify_get_state() == 1) {
      if (overran())
        throw new WebAssembly.RuntimeError(g.broken);
      g.ex.asyncify_stop_unwind();
      status = g.sliced ? 1 : 2;
    } else if (g.rewinding) {
      throw new Error("the guest did not go back to where it paused");
    }
  } catch (e) {
    g.rewinding = false;
    var st = g.ex.asyncify_get_state();
    if (st == 1 && !g.broken)
      overran(); // a trap in the unwind: a store past the memory's end
    try {
      if (st == 1)
        g.ex.asyncify_stop_unwind();
      else if (st == 2)
        g.ex.asyncify_stop_rewind();
    } catch (e2) {
      // stop_unwind's own check of the room's end: the guest is broken already
    }
    Module.cvcWasmText = g.broken || String((e && e.message) || e);
    status = 3;
  }
  s = new BigInt64Array(HEAPU8.buffer, g.scratch, 32); // the host's memory may have grown
  s[17] = BigInt(g.checks);
  if (status == 0)
    s[18] = typeof r == "bigint" ? r : BigInt(Math.trunc(Number(r) || 0));
  return status;
});

EM_JS(double, cvc_wasm_memory_size, (void *ctx), {
  var g = Module.cvcWasmGuests && Module.cvcWasmGuests[ctx];
  return g && g.mem ? g.mem.buffer.byteLength : 0;
});

EM_JS(void, cvc_wasm_memory_read, (void *ctx, double at, void *dst, double n), {
  var g = Module.cvcWasmGuests[ctx];
  HEAPU8.set(new Uint8Array(g.mem.buffer, at, n), dst >>> 0);
});

EM_JS(void, cvc_wasm_memory_write, (void *ctx, double at, const void *src, double n), {
  var g = Module.cvcWasmGuests[ctx], p = src >>> 0;
  new Uint8Array(g.mem.buffer, at, n).set(HEAPU8.subarray(p, p + n));
});

EM_JS(void, cvc_wasm_set_unwind, (void *ctx, double buf), {
  Module.cvcWasmGuests[ctx].buf = buf;
});

// clang-format on

namespace cvc {
namespace ariadne {
namespace wasm {

namespace {

constexpr std::uint64_t kPage = 65536;
// The room a paused guest's stack takes, when ariadne.limits.wasm_unwind_bytes doesn't say.
constexpr std::uint64_t kUnwindBytes = 32 * 1024;
// ari_alloc takes an i32, and the room's ends are i32 addresses.
constexpr std::uint64_t kMaxUnwindBytes = 0x7fffffff - 16;
// The gas checks a start function, or the room for a pause, may run: the engine's setup fuel.
constexpr std::uint64_t kSetupChecks = 100'000'000 / kFuelPerCheck;

// The scratch buffer (the EM_JS bodies' slots).
constexpr int kScratch = 32;
constexpr int kImportArgs = 16; // [0, 16): an import's arguments; its answer at 0
constexpr int kResumed = 16;    // what a rewound import returns
constexpr int kChecks = 17;     // the checks the last call ran
constexpr int kResult = 18;     // the last call's result
constexpr int kExportArgs = 20; // [20, 32): an export's arguments

std::string take_text() {
  const int n = cvc_wasm_text(nullptr, 0);
  std::string s(static_cast<std::size_t>(n), '\0');
  if (n > 0)
    cvc_wasm_text(s.data(), n);
  return s;
}

// A compiled guest: its bytes and the functions it imports. Its compiles, one for each memory cap,
// are the page's (Module.cvcWasmModules[this]).
class BrModule : public Module {
public:
  BrModule(std::string bytes, std::vector<FunctionImport> imports)
      : bytes_(std::move(bytes)), imports_(std::move(imports)) {}
  ~BrModule() override { cvc_wasm_module_drop(this); }
  const std::string &bytes() const { return bytes_; }
  const std::vector<FunctionImport> &imports() const { return imports_; }

private:
  std::string bytes_;
  std::vector<FunctionImport> imports_;
};

// Whether the guest declares each `ari` import it takes as the host defines it: the parameters
// (an i64 where Import::wide marks one) and the result (an i64, or none). wasmtime's linker checks
// the same natively; JS checks nothing, and an import of another type would take what the
// trampoline passes.
bool imports_match(const std::vector<FunctionImport> &declared, const std::vector<Import> &host,
                   std::string &error) {
  for (const FunctionImport &f : declared) {
    if (f.module != "ari")
      continue; // the instantiation refuses an import from another module
    std::string params, results;
    if (f.name == "log_execution") {
      params = "\x7f"; // the transform's gas check, the runtime's own
    } else {
      const auto im =
          std::find_if(host.begin(), host.end(), [&](const Import &i) { return i.name == f.name; });
      if (im == host.end()) {
        error = "it imports ari." + f.name + ", which the host doesn't define";
        return false;
      }
      for (int i = 0; i < im->params; ++i)
        params.push_back((im->wide >> i) & 1u ? '\x7e' : '\x7f');
      if (im->returns)
        results = "\x7e";
    }
    if (f.params != params || f.results != results) {
      error = "it imports ari." + f.name + " as (func" + type_text(f.params, f.results) +
              "), and the host's ari." + f.name + " is (func" + type_text(params, results) + ")";
      return false;
    }
  }
  return true;
}

// The guest's memory, through the page: copies in and out, and no host pointer.
class BrMemory : public GuestMemory {
public:
  explicit BrMemory(void *ctx) : ctx_(ctx) {}
  std::uint64_t size() override { return static_cast<std::uint64_t>(cvc_wasm_memory_size(ctx_)); }
  bool read(std::uint64_t at, std::span<std::uint8_t> out) override {
    if (!fits(at, out.size()))
      return false;
    if (!out.empty())
      cvc_wasm_memory_read(ctx_, static_cast<double>(at), out.data(),
                           static_cast<double>(out.size()));
    return true;
  }
  bool write(std::uint64_t at, std::span<const std::uint8_t> in) override {
    if (!fits(at, in.size()))
      return false;
    if (!in.empty())
      cvc_wasm_memory_write(ctx_, static_cast<double>(at), in.data(),
                            static_cast<double>(in.size()));
    return true;
  }
  std::uint8_t *map(std::uint64_t, std::uint64_t) override { return nullptr; }

private:
  bool fits(std::uint64_t at, std::uint64_t n) {
    const std::uint64_t size = this->size();
    return at <= size && n <= size - at;
  }
  void *ctx_;
};

class BrInstance : public Instance {
public:
  BrInstance(std::shared_ptr<Module> module, std::vector<Import> imports, std::uint64_t max_pages,
             std::uint64_t unwind_bytes)
      : module_(std::move(module)), imports_(std::move(imports)), memory_(this),
        max_pages_(max_pages), unwind_bytes_(unwind_bytes) {}
  ~BrInstance() override { cvc_wasm_drop(this); }

  bool init(const std::string &bytes, std::string &error) {
    std::string spec;
    for (const Import &im : imports_) {
      if (im.params > kImportArgs) {
        error = "ari." + im.name + " takes more than " + std::to_string(kImportArgs) +
                " parameters, which the browser doesn't pass";
        return false;
      }
      spec += im.name + (im.returns ? ":1;" : ":0;");
    }
    pausable_ = false;
    const int r = cvc_wasm_instantiate(
        this, static_cast<BrModule *>(module_.get()), bytes.data(), static_cast<int>(bytes.size()),
        static_cast<double>(max_pages_), spec.data(), static_cast<int>(spec.size()),
        scratch_.data(), static_cast<double>(kSetupChecks), static_cast<double>(unwind_bytes_));
    if (r == 2)
      error = "it isn't transformed for the browser (it has no gas checks, ari.log_execution, "
              "or no Asyncify exports): install it through cvc_ariadne_wasm_transform "
              "(wasm-opt --log-execution=ari --asyncify --pass-arg=asyncify-imports@ari.* -O1)";
    else if (r != 1)
      error = take_text();
    return r == 1;
  }

  // An import the guest called (cvc_ari_wasm_import) with `n` arguments: 0 with its answer in slot
  // 0, 1 a trap (its text's pointer and length in slots 0 and 1), or 2 it waits on its
  // PendingImport (parked here).
  int dispatch(int index, int n) {
    trap_.clear();
    try {
      if (index < 0 || static_cast<std::size_t>(index) >= imports_.size())
        return trapped("an import the host has no side of");
      const Import &im = imports_[static_cast<std::size_t>(index)];
      // The guest's imports were checked at instantiation (imports_match), so this holds: the
      // host's side reads its parameters by position.
      if (n != im.params)
        return trapped("ari." + im.name + " takes " + std::to_string(im.params) +
                       " parameters, and the guest passed " + std::to_string(n));
      std::array<std::int64_t, kImportArgs> a{};
      std::copy(scratch_.begin(), scratch_.begin() + n, a.begin());
      const std::span<const std::int64_t> args(a.data(), static_cast<std::size_t>(n));
      if (im.sync) {
        const std::int64_t r = im.sync(memory_, args, trap_);
        if (!trap_.empty())
          return trapped(std::move(trap_));
        scratch_[0] = r;
        return 0;
      }
      std::shared_ptr<PendingImport> pending = im.async(memory_, args, trap_);
      if (!trap_.empty())
        return trapped(std::move(trap_));
      if (!pending || pending->done) {
        scratch_[0] = pending ? pending->value : 0;
        return 0;
      }
      if (!pausable_)
        return trapped("'ari." + im.name + "' waited on an import, where the guest can't pause");
      waiting_ = std::move(pending);
      return 2;
    } catch (const std::exception &e) {
      return trapped(e.what());
    } catch (...) {
      return trapped("an exception of no known type");
    }
  }

  bool has_export(const std::string &name) const override {
    return cvc_wasm_arity(const_cast<BrInstance *>(this), name.data(),
                          static_cast<int>(name.size())) >= 0;
  }

  GuestMemory &memory() override { return memory_; }

  bool memory_at_limit() override { return max_pages_ > 0 && memory_.size() / kPage >= max_pages_; }

  std::unique_ptr<Call> start(const std::string &name, std::vector<std::int64_t> args,
                              std::string &error) override;

  bool run(const std::string &name, std::vector<std::int64_t> args, std::uint64_t fuel,
           std::int64_t &result, std::uint64_t &used, std::string &error) override {
    if (!takes(name, args.size(), error))
      return false;
    std::uint64_t checks = 0;
    const int status = call(name, args, false, false, fuel, checks);
    used += checks * kFuelPerCheck;
    if (status == 0) {
      result = scratch_[kResult];
      return true;
    }
    error = status == 3 ? take_text() : "'" + name + "' paused, where it can't";
    return false;
  }

  // ── for BrCall ──

  // Call export `name` (cvc_wasm_call's statuses); `checks` gets the gas checks it ran.
  int call(const std::string &name, const std::vector<std::int64_t> &args, bool rewind,
           bool pausable, std::uint64_t fuel, std::uint64_t &checks) {
    std::copy(args.begin(), args.end(), scratch_.begin() + kExportArgs);
    pausable_ = pausable;
    const std::uint64_t left =
        std::max<std::uint64_t>((fuel + kFuelPerCheck - 1) / kFuelPerCheck, 1);
    const int status = cvc_wasm_call(this, name.data(), static_cast<int>(name.size()),
                                     static_cast<int>(args.size()), rewind ? 1 : 0,
                                     pausable ? 1 : 0, static_cast<double>(left));
    pausable_ = false;
    checks = static_cast<std::uint64_t>(scratch_[kChecks]);
    return status;
  }
  // What the guest waits on, if anything.
  const std::shared_ptr<PendingImport> &waiting() const { return waiting_; }
  // Before a rewind: what the call the guest paused in returns (0 after a gas check).
  void resume() {
    scratch_[kResumed] = waiting_ ? waiting_->value : 0;
    waiting_.reset();
  }
  void forget() { waiting_.reset(); }
  std::int64_t result() const { return scratch_[kResult]; }

private:
  // Whether export `name` is a function taking `n` arguments; when not, why.
  bool takes(const std::string &name, std::size_t n, std::string &error) const {
    const int arity =
        cvc_wasm_arity(const_cast<BrInstance *>(this), name.data(), static_cast<int>(name.size()));
    if (arity < 0) {
      error = "the guest exports no function '" + name + "'";
      return false;
    }
    if (static_cast<std::size_t>(arity) != n || n > kScratch - kExportArgs) {
      error =
          "'" + name + "' takes " + std::to_string(arity) + " arguments, not " + std::to_string(n);
      return false;
    }
    return true;
  }

  // The room a paused call's stack takes, from the guest's ari_alloc: once, at the first call. Its
  // header (8 bytes) and the room, 8-aligned.
  bool unwind_room(std::string &error) {
    if (buf_)
      return true;
    std::int64_t p = 0;
    std::uint64_t used = 0;
    const std::string what = "the room a paused call takes (" + std::to_string(unwind_bytes_) +
                             " bytes, ariadne.limits.wasm_unwind_bytes)";
    if (unwind_bytes_ > kMaxUnwindBytes) {
      error = what + " is more than ari_alloc's i32 asks for";
      return false;
    }
    if (!run("ari_alloc", {static_cast<std::int64_t>(unwind_bytes_ + 16)},
             kSetupChecks * kFuelPerCheck, p, used, error)) {
      error = "ari_alloc, for " + what + ": " + error;
      return false;
    }
    const std::uint64_t at = (static_cast<std::uint64_t>(p) + 7) & ~std::uint64_t(7);
    if (std::string why; p <= 0 || !memory_.span_ok(at, unwind_bytes_ + 8, why)) {
      error = "ari_alloc gave " + what + " outside the guest's memory";
      return false;
    }
    buf_ = at;
    cvc_wasm_set_unwind(this, static_cast<double>(at));
    return true;
  }

  int trapped(std::string text) {
    trap_ = text.empty() ? std::string("trap") : std::move(text);
    scratch_[0] = static_cast<std::int64_t>(reinterpret_cast<std::uintptr_t>(trap_.data()));
    scratch_[1] = static_cast<std::int64_t>(trap_.size());
    return 1;
  }

  std::shared_ptr<Module> module_;
  std::vector<Import> imports_;
  BrMemory memory_;
  std::uint64_t max_pages_;    // cap_memory's: the memory's maximum
  std::uint64_t unwind_bytes_; // the room a pause takes
  // Shared with the page: the slots above.
  alignas(8) std::array<std::int64_t, kScratch> scratch_{};
  std::shared_ptr<PendingImport> waiting_; // the async import the guest waits in
  std::string trap_;                       // an import's trap, for the page to read
  std::uint64_t buf_ = 0;                  // where the room is (unwind_room)
  bool pausable_ = false;                  // whether the call running may pause
};

// A call of an export, run a slice at a time: each poll calls the export on the page, the first
// time fresh, and after a pause in rewind mode, back to where it paused.
class BrCall : public Call {
public:
  BrCall(BrInstance &inst, std::string name, std::vector<std::int64_t> args)
      : inst_(inst), name_(std::move(name)), args_(std::move(args)) {}
  ~BrCall() override { inst_.forget(); }

  State poll(std::uint64_t fuel) override {
    used_ = 0;
    if (finished_)
      return trap_.empty() ? State::done : State::trapped;
    if (inst_.waiting() && !inst_.waiting()->done)
      return State::waiting; // nothing to run until the host answers
    const bool rewind = paused_;
    if (rewind)
      inst_.resume();
    std::uint64_t checks = 0;
    const int status = inst_.call(name_, args_, rewind, true, fuel ? fuel : 1, checks);
    used_ = checks * kFuelPerCheck;
    paused_ = status == 1 || status == 2;
    switch (status) {
    case 0:
      finished_ = true;
      result_ = inst_.result();
      return State::done;
    case 1:
      return State::sliced;
    case 2:
      return State::waiting;
    default:
      finished_ = true;
      trap_ = take_text();
      if (trap_.empty())
        trap_ = "trap";
      inst_.forget();
      return State::trapped;
    }
  }

  std::uint64_t fuel_used() const override { return used_; }
  std::int64_t result() const override { return result_; }
  const std::string &trap() const override { return trap_; }

private:
  BrInstance &inst_;
  std::string name_;
  std::vector<std::int64_t> args_;
  bool paused_ = false; // unwound: the next poll rewinds it
  bool finished_ = false;
  std::uint64_t used_ = 0;
  std::int64_t result_ = 0;
  std::string trap_;
};

std::unique_ptr<Call> BrInstance::start(const std::string &name, std::vector<std::int64_t> args,
                                        std::string &error) {
  if (!takes(name, args.size(), error) || !unwind_room(error))
    return nullptr;
  return std::make_unique<BrCall>(*this, name, std::move(args));
}

class BrRuntime : public Runtime {
public:
  std::string name() const override { return "the browser's WebAssembly"; }

  std::shared_ptr<Module> compile(std::string_view bytes, bool text, std::string &error) override {
    if (text) {
      error = "the browser runs .wasm only: transform it ahead (cvc_ariadne_wasm_transform)";
      return nullptr;
    }
    if (!cvc_wasm_check(bytes.data(), static_cast<int>(bytes.size()))) {
      error = take_text();
      return nullptr;
    }
    std::vector<FunctionImport> imports;
    if (!function_imports(bytes, imports, error))
      return nullptr;
    return std::make_shared<BrModule>(std::string(bytes), std::move(imports));
  }

  std::unique_ptr<Instance> instantiate(const std::shared_ptr<Module> &module,
                                        std::vector<Import> imports, const InstanceLimits &limits,
                                        std::string &error) override {
    const auto *m = dynamic_cast<const BrModule *>(module.get());
    if (!m) {
      error = "no module";
      return nullptr;
    }
    if (!imports_match(m->imports(), imports, error))
      return nullptr;
    std::uint64_t max_pages = 0;
    const std::string capped = cap_memory(m->bytes(), limits.max_memory, error, &max_pages);
    if (!error.empty())
      return nullptr;
    auto inst =
        std::make_unique<BrInstance>(module, std::move(imports), max_pages,
                                     limits.unwind_bytes ? limits.unwind_bytes : kUnwindBytes);
    if (!inst->init(capped, error))
      return nullptr;
    return inst;
  }
};

} // namespace

std::unique_ptr<Runtime> make_runtime(std::string &) { return std::make_unique<BrRuntime>(); }

} // namespace wasm
} // namespace ariadne
} // namespace cvc

// The page's trampolines call the host here (cvc_wasm_instantiate): `ctx` is the BrInstance.
extern "C" EMSCRIPTEN_KEEPALIVE int cvc_ari_wasm_import(void *ctx, int index, int n) {
  return static_cast<cvc::ariadne::wasm::BrInstance *>(ctx)->dispatch(index, n);
}

#endif // __EMSCRIPTEN__
