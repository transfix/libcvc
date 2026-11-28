// wamr_runtime.cpp — guest_runtime.h on WAMR's wasm-c-api (G3,
// docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.3, §8.5): the WebAssembly Micro Runtime's interpreter,
// natively, where there is no wasmtime (the BSDs and Haiku).
//
// WAMR runs a call to its end on the host's own stack and can't pause it, so a guest pauses by
// Asyncify, as in the browser (browser_runtime.cpp): it runs transformed, metered by the gas pass
// and then Asyncify around every `ari` import, ari.gas among them (cvc_ariadne_wasm_transform). A
// pause unwinds the guest's stack into a room in its own memory and returns to the host, and the
// next poll calls the export again in rewind mode, which takes the guest back to where it paused:
//   - at ari.gas, the gas pass's call for a block the slice can't pay for (meter.h): sliced. Each
//     call sets the guest's ari_gas to its slice first, and what the guest took from it is the
//     call's gas; the next poll's slice is at least the block's cost, which the guest pays first;
//   - in an async import whose PendingImport isn't done: waiting. Rewound, the import returns the
//     PendingImport's value.
// The host imports start and stop the unwind and the rewind by calling the guest's own Asyncify
// exports from inside the import: WAMR runs a call into the same instance from an import on the
// same execution environment, as its libc's qsort calls a guest's comparator.
//
// A guest that isn't transformed is transformed when it's compiled, as wasmtime meters one: its
// text read as a binary, metered, then Asyncify, by binaryen in process (wasm_transform.h), where
// the host links it (CVC_ARIADNE_WASM_BINARYEN). Without binaryen this backend takes, as the
// browser does, only binaries that were transformed at install. A transformed guest runs as it is
// either way.
//
// Run to its end (run(), a start function), a guest can't pause: out of gas or waiting, it traps.
// A start function runs on ari_gas's own start (kStartGas). The memory limit is the module's own
// maximum, lowered to InstanceLimits::max_memory (cap_memory, wasm_binary.h), as in the browser:
// WAMR refuses a memory.grow past it, and the guest sees -1. The room a paused stack takes is
// InstanceLimits::unwind_bytes, from the guest's ari_alloc at its first call, and a stack that ran
// past it has written over the guest's memory: the guest is broken, and refuses every later call.
//
// The c-api's objects: the Runtime holds a reference to WAMR's engine (WAMR keeps one per process
// and counts its references, so the runtime starts with the first and stops with the last). A
// store is tied to the thread that made it, so each instance has one of its own, made on the thread
// that instantiates it (the drain's), as wasmtime's are. A module lives in a store, so compile()
// keeps the transformed bytes, and each memory cap's compile is loaded once, in a store of its own
// on the instantiating thread, and shared (wasm_module_share) into each instance's store. An
// instance's imports are c-api functions with an environment, so they are its own and no table of
// natives is registered anywhere.

#include "guest_runtime.h"
#include "meter.h"
#include "wamr_load.h"
#include "wasm_binary.h"
#ifdef CVC_ARIADNE_WASM_BINARYEN
#include "wasm_transform.h"
#endif

#include <algorithm>
#include <cstring>
#include <iterator>
#include <map>
#include <mutex>
#include <utility>
#include <wasm_c_api.h>

namespace cvc {
namespace ariadne {
namespace wasm {

namespace {

constexpr std::uint64_t kPage = 65536;
// The room a paused guest's stack takes, when ariadne.limits.wasm_unwind_bytes doesn't say: the
// browser's.
constexpr std::uint64_t kUnwindBytes = 32 * 1024;
// ari_alloc takes an i32, and the room's ends are i32 addresses.
constexpr std::uint64_t kMaxUnwindBytes = 0x7fffffff - 16;
// The gas ari_alloc may use for the room a pause takes: the engine's setup gas.
constexpr std::uint64_t kSetupGas = 100'000'000;
// The interpreter's operand and frame stack for one call, in bytes. Asyncify keeps a paused call's
// frames in the guest's memory, not here.
constexpr std::uint32_t kStackBytes = 1024 * 1024;

// What the guest's start function is exported as (start_as_export, wasm_binary.h): WAMR would run
// it as it instantiates, before the instance's imports are wired, so the host runs it right after.
constexpr std::string_view kStartExport = "__ari_wamr_start";

// Asyncify's exports, which a transformed guest has.
constexpr const char *kAsyncify[] = {"asyncify_start_unwind", "asyncify_stop_unwind",
                                     "asyncify_start_rewind", "asyncify_stop_rewind",
                                     "asyncify_get_state"};

using EnginePtr = std::shared_ptr<wasm_engine_t>;

wasm_val_vec_t vec_of(wasm_val_t *data, std::size_t n) {
  wasm_val_vec_t v;
  v.size = n;
  v.data = data;
  v.num_elems = n;
  v.size_of_elem = sizeof(wasm_val_t);
  v.lock = nullptr;
  return v;
}

std::string name_of(const wasm_name_t *n) {
  std::string s = n && n->data ? std::string(n->data, n->size) : std::string();
  while (!s.empty() && s.back() == '\0')
    s.pop_back();
  return s;
}

// A trap's message, without WAMR's "Exception: " in front.
std::string take_trap(wasm_trap_t *t) {
  if (!t)
    return {};
  wasm_message_t msg;
  wasm_trap_message(t, &msg);
  std::string s = name_of(&msg);
  wasm_byte_vec_delete(&msg);
  wasm_trap_delete(t);
  constexpr std::string_view kPrefix = "Exception: ";
  if (s.compare(0, kPrefix.size(), kPrefix) == 0)
    s.erase(0, kPrefix.size());
  return s.empty() ? std::string("trap") : s;
}

// A trap from the host's side of an import. WAMR keeps an exception's text in a buffer of 128 bytes
// and cuts a host trap's at 107, so the caller takes the whole text from `full` (InstanceState's
// host_trap) when the call comes back trapped.
wasm_trap_t *new_trap(wasm_store_t *store, const std::string &text, std::string *full = nullptr) {
  if (full)
    *full = text;
  wasm_message_t msg;
  wasm_byte_vec_new(&msg, text.size() + 1, text.c_str()); // with its NUL, as WAMR's own are
  wasm_trap_t *t = wasm_trap_new(store, &msg);
  wasm_byte_vec_delete(&msg);
  return t;
}

// A store and, in it, a module of the WAMR engine, made and deleted on one thread.
struct ScratchStore {
  explicit ScratchStore(wasm_engine_t *engine) : store(wasm_store_new(engine)) {}
  ~ScratchStore() {
    if (store)
      wasm_store_delete(store);
  }
  wasm_store_t *store;
};

// A compiled guest: its transformed bytes, and, for each memory cap it has run under, that compile,
// shared between the stores of its instances.
class WamrModule : public Module {
public:
  WamrModule(EnginePtr engine, std::string bytes, std::vector<FunctionImport> declared, bool start)
      : engine_(std::move(engine)), bytes_(std::move(bytes)), declared_(std::move(declared)),
        start_(start) {}
  ~WamrModule() override {
    for (auto &[pages, shared] : shared_)
      wasm_shared_module_delete(shared);
  }

  const std::vector<FunctionImport> &declared() const { return declared_; }
  // Whether the guest has a start function, exported as kStartExport.
  bool start() const { return start_; }

  // The compile with its memory's maximum at `max_bytes` (0: as the module has it), and that
  // maximum in pages (0: no memory), loaded the first time it's asked for. Null with `error` set.
  wasm_shared_module_t *capped(std::uint64_t max_bytes, std::uint64_t &max_pages,
                               std::string &error) {
    const std::string bytes = cap_memory(bytes_, max_bytes, error, &max_pages);
    if (!error.empty())
      return nullptr;
    std::lock_guard<std::mutex> hold(lock_);
    if (auto it = shared_.find(max_pages); it != shared_.end())
      return it->second;
    ScratchStore scratch(engine_.get());
    if (!scratch.store) {
      error = "WAMR: no store";
      return nullptr;
    }
    wasm_byte_vec_t binary;
    wasm_byte_vec_new(&binary, bytes.size(), bytes.data());
    wasm_module_t *m = wasm_module_new(scratch.store, &binary);
    wasm_byte_vec_delete(&binary);
    if (!m) {
      error = "WAMR didn't load the guest under its memory cap";
      return nullptr;
    }
    wasm_shared_module_t *shared = wasm_module_share(m);
    if (!shared) {
      error = "WAMR didn't share the guest's module";
      return nullptr;
    }
    shared_.emplace(max_pages, shared);
    return shared;
  }

private:
  EnginePtr engine_;
  std::string bytes_; // transformed, uncapped
  // The functions the guest imports as it was written: the transform's optimization drops an
  // import it never calls, and the host checks the guest it was given (imports_match).
  std::vector<FunctionImport> declared_;
  bool start_ = false;
  std::mutex lock_; // shared_: an instantiation off the drain's thread (a client's guest)
  std::map<std::uint64_t, wasm_shared_module_t *> shared_;
};

// What an instance's imports share with its calls: the guest's exports a pause needs, where its
// paused stack goes, and the state of the call it's in.
struct InstanceState {
  wasm_store_t *store = nullptr;
  wasm_memory_t *memory = nullptr; // "memory", once the instance exists
  wasm_global_t *gas = nullptr;    // ari_gas
  wasm_func_t *start_unwind = nullptr, *stop_unwind = nullptr, *start_rewind = nullptr,
              *stop_rewind = nullptr, *get_state = nullptr;
  std::uint64_t unwind_bytes = kUnwindBytes;
  std::uint32_t buf = 0; // the room: its 8-byte header, then unwind_bytes
  bool instantiated = false;
  bool pausable = false;     // the call running may pause
  bool rewinding = false;    // the call was started to rewind: the next ari import resumes it
  bool sliced = false;       // it paused at ari.gas
  std::uint64_t pending = 0; // the block it paused before, where its slice ran out
  std::int64_t resumed = 0;  // what the import it paused in returns, once rewound
  std::shared_ptr<PendingImport> waiting; // the async import it waits in
  std::string broken;    // why it refuses every call: its paused stack ran past the room
  std::string host_trap; // the last trap an import made, whole (new_trap)
};

// The guest's memory in the host's address space: map() is the memory itself, valid until the
// guest runs or the memory grows, and read and write copy.
class WamrMemory : public GuestMemory {
public:
  explicit WamrMemory(InstanceState &st) : st_(st) {}
  std::uint64_t size() override { return st_.memory ? wasm_memory_data_size(st_.memory) : 0; }
  bool read(std::uint64_t at, std::span<std::uint8_t> out) override {
    if (out.empty())
      return at <= size();
    const std::uint8_t *p = map(at, out.size());
    if (p)
      std::memcpy(out.data(), p, out.size());
    return p != nullptr;
  }
  bool write(std::uint64_t at, std::span<const std::uint8_t> in) override {
    if (in.empty())
      return at <= size();
    std::uint8_t *p = map(at, in.size());
    if (p)
      std::memcpy(p, in.data(), in.size());
    return p != nullptr;
  }
  std::uint8_t *map(std::uint64_t at, std::uint64_t n) override {
    const std::uint64_t size = this->size();
    if (!st_.memory || at > size || n > size - at)
      return nullptr;
    return reinterpret_cast<std::uint8_t *>(wasm_memory_data(st_.memory)) + at;
  }

private:
  InstanceState &st_;
};

// A call of one of the guest's Asyncify exports, from inside an import: false when it trapped.
bool asyncify_call(wasm_func_t *f, std::int32_t arg, bool with_arg,
                   std::int32_t *result = nullptr) {
  wasm_val_t a[1];
  a[0].kind = WASM_I32;
  a[0].of.i32 = arg;
  wasm_val_t r[1];
  r[0].kind = WASM_I32;
  r[0].of.i32 = 0;
  wasm_val_vec_t in = vec_of(a, with_arg ? 1 : 0);
  wasm_val_vec_t out = vec_of(r, result ? 1 : 0);
  wasm_trap_t *trap = wasm_func_call(f, &in, &out);
  if (trap) {
    wasm_trap_delete(trap);
    return false;
  }
  if (result)
    *result = r[0].of.i32;
  return true;
}

// Unwind the guest's stack into the room at buf: Asyncify's header (where it writes next, and
// where the room ends), then the room, [buf + 8, buf + 8 + unwind_bytes).
bool pause(InstanceState &st) {
  WamrMemory mem(st);
  std::uint8_t *header = mem.map(st.buf, 8);
  if (!header)
    return false;
  const std::uint32_t next = st.buf + 8;
  const std::uint32_t end = static_cast<std::uint32_t>(st.buf + 8 + st.unwind_bytes);
  std::memcpy(header, &next, 4); // little-endian, as the guest reads it
  std::memcpy(header + 4, &end, 4);
  return asyncify_call(st.start_unwind, static_cast<std::int32_t>(st.buf), true);
}

// The rewind reached the call the guest paused in: it goes on from there.
bool resumed(InstanceState &st) {
  st.rewinding = false;
  return asyncify_call(st.stop_rewind, 0, false);
}

// An import's result as its type has it: an i64, or an i32 (WASI's errno). WAMR hands the import
// a vector of `size` results, and num_elems 0.
void set_result(wasm_val_vec_t *results, std::int64_t r, bool i32) {
  if (!results || results->size == 0 || !results->data)
    return;
  results->num_elems = 1;
  if (i32) {
    results->data[0].kind = WASM_I32;
    results->data[0].of.i32 = static_cast<std::int32_t>(r);
  } else {
    results->data[0].kind = WASM_I64;
    results->data[0].of.i64 = r;
  }
}

struct GasEnv {
  InstanceState *state;
};

// ari.gas(cost): the slice can't pay for the block the guest is about to run. The call pauses
// there; a call that can't pause, or the start function, is out of gas.
wasm_trap_t *gas_import(void *env, const wasm_val_vec_t *args, wasm_val_vec_t *) {
  InstanceState &st = *static_cast<GasEnv *>(env)->state;
  if (st.rewinding) {
    if (!resumed(st))
      return new_trap(st.store, "the guest's asyncify_stop_rewind trapped", &st.host_trap);
    return nullptr;
  }
  if (!st.pausable)
    return new_trap(
        st.store, st.instantiated ? "ran out of gas" : "the module's start function ran out of gas",
        &st.host_trap);
  const std::int64_t cost = args && args->num_elems > 0 ? args->data[0].of.i64 : 0;
  st.pending = cost > 0 ? static_cast<std::uint64_t>(cost) : 0;
  st.sliced = true;
  if (!pause(st))
    return new_trap(st.store, "the guest's asyncify_start_unwind trapped", &st.host_trap);
  return nullptr;
}

void free_gas_env(void *env) { delete static_cast<GasEnv *>(env); }

struct ImportEnv {
  Import import;
  InstanceState *state;
};

void free_import_env(void *env) { delete static_cast<ImportEnv *>(env); }

wasm_trap_t *host_import(void *env, const wasm_val_vec_t *args, wasm_val_vec_t *results) {
  auto *ie = static_cast<ImportEnv *>(env);
  InstanceState &st = *ie->state;
  const Import &im = ie->import;
  if (st.rewinding) {
    if (!resumed(st))
      return new_trap(st.store, "the guest's asyncify_stop_rewind trapped", &st.host_trap);
    set_result(results, st.resumed, im.returns_i32);
    return nullptr;
  }
  std::vector<std::int64_t> a;
  const std::size_t n = args ? args->num_elems : 0;
  a.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    a.push_back(args->data[i].kind == WASM_I32 ? args->data[i].of.i32 : args->data[i].of.i64);
  WamrMemory mem(st);
  std::string trap;
  try {
    if (im.sync) {
      const std::int64_t r = im.sync(mem, a, trap);
      if (!trap.empty())
        return new_trap(st.store, trap, &st.host_trap);
      set_result(results, r, im.returns_i32);
      return nullptr;
    }
    std::shared_ptr<PendingImport> pending = im.async(mem, a, trap);
    if (!trap.empty())
      return new_trap(st.store, trap, &st.host_trap);
    if (!pending || pending->done) {
      set_result(results, pending ? pending->value : 0, im.returns_i32);
      return nullptr;
    }
    if (!st.pausable)
      return new_trap(st.store,
                      "'" + im.module + "." + im.name +
                          "' waited on an import, where the guest can't pause",
                      &st.host_trap);
    st.waiting = std::move(pending);
    if (!pause(st))
      return new_trap(st.store, "the guest's asyncify_start_unwind trapped", &st.host_trap);
    set_result(results, 0, im.returns_i32);
    return nullptr;
  } catch (const std::exception &e) {
    return new_trap(st.store, e.what(), &st.host_trap);
  } catch (...) {
    return new_trap(st.store, "an exception of no known type", &st.host_trap);
  }
}

// A value type's byte in the binary format (wasm_binary.h's FunctionImport), for a c-api kind.
char type_byte(wasm_valkind_t k) {
  switch (k) {
  case WASM_I32:
    return '\x7f';
  case WASM_I64:
    return '\x7e';
  case WASM_F32:
    return '\x7d';
  case WASM_F64:
    return '\x7c';
  default:
    return '\x70';
  }
}

std::string type_bytes(const wasm_valtype_vec_t *v) {
  std::string s;
  for (std::size_t i = 0; v && i < v->num_elems; ++i)
    s.push_back(type_byte(wasm_valtype_kind(v->data[i])));
  return s;
}

// The host's type for an import. WAMR's vectors count their elements apart from their room
// (num_elems, size), and wasm_valtype_vec_new_uninitialized leaves the count at 0, so the vectors
// are made whole, with wasm_valtype_vec_new: WAMR's linker compares the counts.
wasm_functype_t *import_type(const Import &im) {
  std::vector<wasm_valtype_t *> ps, rs;
  for (int i = 0; i < im.params; ++i)
    ps.push_back((im.wide >> i) & 1u ? wasm_valtype_new_i64() : wasm_valtype_new_i32());
  if (im.returns)
    rs.push_back(im.returns_i32 ? wasm_valtype_new_i32() : wasm_valtype_new_i64());
  wasm_valtype_vec_t params, results;
  if (ps.empty())
    wasm_valtype_vec_new_empty(&params);
  else
    wasm_valtype_vec_new(&params, ps.size(), ps.data());
  if (rs.empty())
    wasm_valtype_vec_new_empty(&results);
  else
    wasm_valtype_vec_new(&results, rs.size(), rs.data());
  return wasm_functype_new(&params, &results);
}

class WamrInstance : public Instance {
public:
  WamrInstance(EnginePtr engine, std::shared_ptr<Module> module, std::vector<Import> imports,
               std::uint64_t max_pages, std::uint64_t max_memory, std::uint64_t unwind_bytes)
      : engine_(std::move(engine)), module_(std::move(module)), imports_(std::move(imports)),
        memory_(state_), max_pages_(max_pages), max_memory_(max_memory) {
    state_.unwind_bytes = unwind_bytes;
  }
  ~WamrInstance() override {
    wasm_extern_vec_delete(&exports_); // our copies
    wasm_exporttype_vec_delete(&export_types_);
    if (instance_)
      wasm_instance_delete(instance_);
    // The store deletes the instance, which WAMR keeps until then, and drops the module's share.
    if (state_.store)
      wasm_store_delete(state_.store);
    for (wasm_func_t *f : host_)
      wasm_func_delete(f); // and its environment, through its finalizer
  }

  bool init(wasm_shared_module_t *shared, bool start, std::string &error) {
    state_.store = wasm_store_new(engine_.get());
    if (!state_.store) {
      error = "WAMR: no store";
      return false;
    }
    wasm_module_t *m = wasm_module_obtain(state_.store, shared);
    if (!m) {
      error = "WAMR: the guest's module isn't there";
      return false;
    }
    // The imports in the module's order, each the host's side of the one it names, of the type it
    // declares: wasmtime's linker refuses one the host doesn't define, or defines with another
    // type, and so does this.
    wasm_importtype_vec_t wanted;
    wasm_module_imports(m, &wanted);
    std::vector<wasm_extern_t *> externs;
    bool ok = true;
    for (std::size_t i = 0; ok && i < wanted.num_elems; ++i) {
      const wasm_importtype_t *t = wanted.data[i];
      const std::string mod = name_of(wasm_importtype_module(t));
      const std::string name = name_of(wasm_importtype_name(t));
      const std::string what = mod + "." + name;
      const wasm_functype_t *ft = wasm_externtype_as_functype_const(wasm_importtype_type(t));
      if (!ft) {
        error = "it imports " + what + ", which isn't a function, and the host defines only those";
        ok = false;
        break;
      }
      const std::string params = type_bytes(wasm_functype_params(ft));
      const std::string results = type_bytes(wasm_functype_results(ft));
      std::string host_params, host_results;
      wasm_func_t *f = nullptr;
      if (mod == kGasModule && name == kGasImport) {
        host_params = "\x7e"; // the gas pass's ari.gas (meter.h), the runtime's own
        if (params == host_params && results.empty()) {
          wasm_functype_t *ty = wasm_functype_new_1_0(wasm_valtype_new_i64());
          f = wasm_func_new_with_env(state_.store, ty, gas_import, new GasEnv{&state_},
                                     free_gas_env);
          wasm_functype_delete(ty);
        }
      } else {
        const auto im = std::find_if(imports_.begin(), imports_.end(), [&](const Import &x) {
          return x.module == mod && x.name == name;
        });
        if (im == imports_.end()) {
          error = "it imports " + what + ", which the host doesn't define";
          ok = false;
          break;
        }
        for (int p = 0; p < im->params; ++p)
          host_params.push_back((im->wide >> p) & 1u ? '\x7e' : '\x7f');
        if (im->returns)
          host_results = im->returns_i32 ? "\x7f" : "\x7e";
        if (params == host_params && results == host_results) {
          wasm_functype_t *ty = import_type(*im);
          f = wasm_func_new_with_env(state_.store, ty, host_import, new ImportEnv{*im, &state_},
                                     free_import_env);
          wasm_functype_delete(ty);
        }
      }
      if (params != host_params || results != host_results) {
        error = "it imports " + what + " as (func" + type_text(params, results) +
                "), and the host's " + what + " is (func" + type_text(host_params, host_results) +
                ")";
        ok = false;
        break;
      }
      if (!f) {
        error = "WAMR didn't make the host's " + what;
        ok = false;
        break;
      }
      host_.push_back(f);
      externs.push_back(wasm_func_as_extern(f));
    }
    wasm_importtype_vec_delete(&wanted);
    if (!ok)
      return false;

    wasm_extern_vec_t import_vec;
    import_vec.size = externs.size();
    import_vec.data = externs.data();
    import_vec.num_elems = externs.size();
    import_vec.size_of_elem = sizeof(wasm_extern_t *);
    import_vec.lock = nullptr;
    InstantiationArgs args{};
    args.default_stack_size = kStackBytes;
    // The guest's start function isn't WAMR's to run (kStartExport): it runs below.
    wasm_trap_t *trap = nullptr;
    instance_ = wasm_instance_new_with_args_ex(state_.store, m, &import_vec, &trap, &args);
    if (!instance_) {
      error = trap ? take_trap(trap) : "WAMR didn't instantiate the guest";
      if (!state_.host_trap.empty())
        error = state_.host_trap;
      return false;
    }
    if (trap)
      wasm_trap_delete(trap);

    wasm_module_exports(m, &export_types_);
    wasm_instance_exports(instance_, &exports_);
    for (std::size_t i = 0; i < export_types_.num_elems && i < exports_.num_elems; ++i) {
      const std::string name = name_of(wasm_exporttype_name(export_types_.data[i]));
      // By its kind: WAMR's wasm_extern_as_func and the rest cast whatever they're given.
      wasm_extern_t *e = exports_.data[i];
      const wasm_externkind_t kind = wasm_extern_kind(e);
      if (kind == WASM_EXTERN_FUNC)
        funcs_.emplace(name, wasm_extern_as_func(e));
      else if (kind == WASM_EXTERN_MEMORY && name == "memory")
        state_.memory = wasm_extern_as_memory(e);
      else if (kind == WASM_EXTERN_GLOBAL && name == kGasGlobal)
        state_.gas = wasm_extern_as_global(e);
    }
    if (!state_.gas) {
      error = "the metered module exports no ari_gas global";
      return false;
    }
    wasm_func_t **slots[] = {&state_.start_unwind, &state_.stop_unwind, &state_.start_rewind,
                             &state_.stop_rewind, &state_.get_state};
    for (std::size_t i = 0; i < std::size(kAsyncify); ++i) {
      const auto it = funcs_.find(kAsyncify[i]);
      if (it == funcs_.end()) {
        error = std::string("the transformed module exports no ") + kAsyncify[i];
        return false;
      }
      *slots[i] = it->second;
    }
    // The start function, to its end, on ari_gas's own start (kStartGas): it may not pause, and
    // the gas it runs out of is the start's (gas_import, while `instantiated` is false).
    if (start) {
      std::int64_t ignored = 0;
      std::uint64_t used = 0;
      if (!run(std::string(kStartExport), {}, kStartGas, ignored, used, error)) {
        if (error.find("waited on an import") != std::string::npos ||
            error.find("paused, where it can't") != std::string::npos)
          error = "the module's start function waited on an import";
        return false;
      }
      funcs_.erase(std::string(kStartExport));
    }
    state_.instantiated = true;
    return true;
  }

  bool has_export(const std::string &name) const override { return funcs_.count(name) > 0; }

  GuestMemory &memory() override { return memory_; }

  bool memory_at_limit() override {
    if (!state_.memory)
      return false;
    const std::uint64_t size = wasm_memory_data_size(state_.memory);
    return (max_memory_ > 0 && size + kPage > max_memory_) ||
           (max_pages_ > 0 && size / kPage >= max_pages_);
  }

  std::unique_ptr<Call> start(const std::string &name, std::vector<std::int64_t> args,
                              std::string &error) override;

  bool run(const std::string &name, std::vector<std::int64_t> args, std::uint64_t gas,
           std::int64_t &result, std::uint64_t &used, std::string &error) override {
    wasm_func_t *f = nullptr;
    std::vector<wasm_val_t> vals;
    std::size_t nresults = 0;
    if (!prepare(name, args, f, vals, nresults, error))
      return false;
    std::uint64_t taken = 0;
    const int status = call(f, vals, nresults, false, false, gas, taken, result, error);
    used += taken;
    if (status == 0)
      return true;
    if (status != 3)
      error = "'" + name + "' paused, where it can't";
    return false;
  }

  // ── for WamrCall ──

  // The export `name`, and `args` as the values its parameters take.
  bool prepare(const std::string &name, const std::vector<std::int64_t> &args, wasm_func_t *&f,
               std::vector<wasm_val_t> &vals, std::size_t &nresults, std::string &error) {
    const auto it = funcs_.find(name);
    if (it == funcs_.end()) {
      error = "the guest exports no function '" + name + "'";
      return false;
    }
    f = it->second;
    wasm_functype_t *ty = wasm_func_type(f);
    const wasm_valtype_vec_t *params = wasm_functype_params(ty);
    nresults = wasm_functype_results(ty)->num_elems;
    bool ok = true;
    if (params->num_elems != args.size()) {
      error = "'" + name + "' takes " + std::to_string(params->num_elems) + " arguments, not " +
              std::to_string(args.size());
      ok = false;
    }
    vals.assign(args.size(), wasm_val_t{});
    for (std::size_t i = 0; ok && i < args.size(); ++i) {
      const wasm_valkind_t k = wasm_valtype_kind(params->data[i]);
      if (k == WASM_I32) {
        vals[i].kind = WASM_I32;
        vals[i].of.i32 = static_cast<std::int32_t>(args[i]);
      } else if (k == WASM_I64) {
        vals[i].kind = WASM_I64;
        vals[i].of.i64 = args[i];
      } else {
        error = "'" + name + "' takes a parameter that isn't an i32 or an i64";
        ok = false;
      }
    }
    wasm_functype_delete(ty);
    return ok;
  }

  // Call `f` with `gas` in ari_gas, rewinding it to where it paused when `rewind`: 0 it returned
  // (`result`), 1 it paused at ari.gas (pending()), 2 in an import that waits, 3 it trapped
  // (`error`). `taken` gets the gas it took.
  int call(wasm_func_t *f, std::vector<wasm_val_t> &args, std::size_t nresults, bool rewind,
           bool pausable, std::uint64_t gas, std::uint64_t &taken, std::int64_t &result,
           std::string &error) {
    taken = 0;
    if (!state_.broken.empty()) {
      error = state_.broken;
      return 3;
    }
    const std::uint64_t set = std::max<std::uint64_t>(gas, 1);
    set_gas(set);
    state_.pending = 0;
    state_.sliced = false;
    state_.pausable = pausable;
    state_.host_trap.clear();
    int status = 0;
    std::vector<wasm_val_t> results(std::max<std::size_t>(nresults, 1));
    wasm_val_vec_t in = vec_of(args.data(), args.size());
    wasm_val_vec_t out = vec_of(results.data(), nresults);
    if (rewind) {
      state_.rewinding = true;
      if (!asyncify_call(state_.start_rewind, static_cast<std::int32_t>(state_.buf), true)) {
        state_.rewinding = false;
        error = "the guest's asyncify_start_rewind trapped";
        state_.pausable = false;
        return 3;
      }
    }
    wasm_trap_t *trap = wasm_func_call(f, &in, &out);
    std::int32_t st = 0;
    if (!trap) {
      asyncify_call(state_.get_state, 0, false, &st);
      if (st == 1) { // unwound: paused
        if (overran()) {
          error = state_.broken;
          asyncify_call(state_.stop_unwind, 0, false);
          status = 3;
        } else {
          asyncify_call(state_.stop_unwind, 0, false);
          status = state_.sliced ? 1 : 2;
        }
      } else if (state_.rewinding) {
        state_.rewinding = false;
        error = "the guest did not go back to where it paused";
        status = 3;
      } else if (nresults > 0) {
        result = results[0].kind == WASM_I32 ? results[0].of.i32 : results[0].of.i64;
      } else {
        result = 0;
      }
    } else {
      state_.rewinding = false;
      error = take_trap(trap);
      if (!state_.host_trap.empty())
        error = state_.host_trap; // WAMR's copy of it is cut short
      asyncify_call(state_.get_state, 0, false, &st);
      if (st == 1 && state_.broken.empty())
        overran(); // a trap in the unwind: a store past the memory's end
      if (st == 1)
        asyncify_call(state_.stop_unwind, 0, false);
      else if (st == 2)
        asyncify_call(state_.stop_rewind, 0, false);
      if (!state_.broken.empty())
        error = state_.broken;
      status = 3;
    }
    state_.pausable = false;
    const std::uint64_t left = get_gas();
    taken = set > left ? set - left : 0;
    return status;
  }

  // After a pause at ari.gas: the cost of the block the slice couldn't pay for.
  std::uint64_t pending() const { return state_.pending; }
  // What the guest waits on, if anything.
  const std::shared_ptr<PendingImport> &waiting() const { return state_.waiting; }
  // Before a rewind: what the call the guest paused in returns (0 after ari.gas).
  void resume() {
    state_.resumed = state_.waiting ? state_.waiting->value : 0;
    state_.waiting.reset();
  }
  void forget() { state_.waiting.reset(); }

private:
  void set_gas(std::uint64_t v) {
    wasm_val_t val;
    val.kind = WASM_I64;
    val.of.i64 = static_cast<std::int64_t>(v);
    wasm_global_set(state_.gas, &val);
  }
  std::uint64_t get_gas() {
    wasm_val_t val;
    val.kind = WASM_I64;
    val.of.i64 = 0;
    wasm_global_get(state_.gas, &val);
    return val.kind == WASM_I64 && val.of.i64 > 0 ? static_cast<std::uint64_t>(val.of.i64) : 0;
  }

  // After an unwind, whether the paused stack ran past the room. Asyncify writes it unchecked and
  // checks only in asyncify_stop_unwind, after it has written over the guest's memory past the
  // room, whatever the guest's allocator had put there: the guest is broken.
  bool overran() {
    std::uint8_t *header = memory_.map(state_.buf, 8);
    if (!header)
      return false;
    std::uint32_t next = 0, end = 0;
    std::memcpy(&next, header, 4);
    std::memcpy(&end, header + 4, 4);
    if (next <= end)
      return false;
    state_.broken = "a paused call's stack took " + std::to_string(next - state_.buf - 8) +
                    " bytes, past its room's " + std::to_string(state_.unwind_bytes) +
                    " (ariadne.limits.wasm_unwind_bytes), and wrote over the guest's memory "
                    "after the room: the guest can't run again until its document is reloaded";
    return true;
  }

public:
  // The room a paused call's stack takes, from the guest's ari_alloc: once, at the first call. Its
  // header (8 bytes) and the room, 8-aligned.
  bool unwind_room(std::string &error) {
    if (state_.buf)
      return true;
    const std::string what = "the room a paused call takes (" +
                             std::to_string(state_.unwind_bytes) +
                             " bytes, ariadne.limits.wasm_unwind_bytes)";
    if (state_.unwind_bytes > kMaxUnwindBytes) {
      error = what + " is more than ari_alloc's i32 asks for";
      return false;
    }
    std::int64_t p = 0;
    std::uint64_t used = 0;
    if (!run("ari_alloc", {static_cast<std::int64_t>(state_.unwind_bytes + 16)}, kSetupGas, p, used,
             error)) {
      error = "ari_alloc, for " + what + ": " + error;
      return false;
    }
    const std::uint64_t at = (static_cast<std::uint64_t>(p) + 7) & ~std::uint64_t(7);
    if (std::string why; p <= 0 || at + state_.unwind_bytes + 8 > UINT32_MAX ||
                         !memory_.span_ok(at, state_.unwind_bytes + 8, why)) {
      error = "ari_alloc gave " + what + " outside the guest's memory";
      return false;
    }
    state_.buf = static_cast<std::uint32_t>(at);
    return true;
  }

private:
  EnginePtr engine_;
  std::shared_ptr<Module> module_;
  std::vector<Import> imports_;
  InstanceState state_;
  WamrMemory memory_;
  std::uint64_t max_pages_;  // cap_memory's: the memory's maximum
  std::uint64_t max_memory_; // InstanceLimits::max_memory
  wasm_instance_t *instance_ = nullptr;
  std::vector<wasm_func_t *> host_; // the imports, which live as long as the instance
  wasm_exporttype_vec_t export_types_{};
  wasm_extern_vec_t exports_{};
  std::map<std::string, wasm_func_t *> funcs_; // its exported functions, by name
};

// A call of an export, run a slice at a time: each poll calls the export, the first time fresh,
// and after a pause in rewind mode, back to where it paused.
class WamrCall : public Call {
public:
  WamrCall(WamrInstance &inst, wasm_func_t *f, std::vector<wasm_val_t> args, std::size_t nresults)
      : inst_(inst), f_(f), args_(std::move(args)), nresults_(nresults) {}
  ~WamrCall() override { inst_.forget(); }

  State poll(std::uint64_t gas) override {
    used_ = 0;
    if (finished_)
      return trap_.empty() ? State::done : State::trapped;
    if (inst_.waiting() && !inst_.waiting()->done)
      return State::waiting; // nothing to run until the host answers
    const bool rewind = paused_;
    if (rewind)
      inst_.resume();
    // The slice, and at least the block the last one couldn't pay for, which the guest pays first.
    const std::uint64_t set = std::max<std::uint64_t>(gas, sliced_ ? pending_ : 0);
    std::uint64_t taken = 0;
    std::string error;
    const int status = inst_.call(f_, args_, nresults_, rewind, true, set, taken, result_, error);
    used_ = taken;
    paused_ = status == 1 || status == 2;
    sliced_ = status == 1;
    pending_ = sliced_ ? inst_.pending() : 0;
    switch (status) {
    case 0:
      finished_ = true;
      return State::done;
    case 1:
      return State::sliced;
    case 2:
      return State::waiting;
    default:
      finished_ = true;
      trap_ = error.empty() ? std::string("trap") : std::move(error);
      inst_.forget();
      return State::trapped;
    }
  }

  std::uint64_t gas_used() const override { return used_; }
  std::int64_t result() const override { return result_; }
  const std::string &trap() const override { return trap_; }

private:
  WamrInstance &inst_;
  wasm_func_t *f_;
  std::vector<wasm_val_t> args_;
  std::size_t nresults_;
  bool paused_ = false;       // unwound: the next poll rewinds it
  bool sliced_ = false;       // at ari.gas
  std::uint64_t pending_ = 0; // the block the slice couldn't pay for
  bool finished_ = false;
  std::uint64_t used_ = 0;
  std::int64_t result_ = 0;
  std::string trap_;
};

std::unique_ptr<Call> WamrInstance::start(const std::string &name, std::vector<std::int64_t> args,
                                          std::string &error) {
  wasm_func_t *f = nullptr;
  std::vector<wasm_val_t> vals;
  std::size_t nresults = 0;
  if (!prepare(name, args, f, vals, nresults, error) || !unwind_room(error))
    return nullptr;
  return std::make_unique<WamrCall>(*this, f, std::move(vals), nresults);
}

class WamrRuntime : public Runtime {
public:
  explicit WamrRuntime(EnginePtr engine) : engine_(std::move(engine)) {}
  std::string name() const override { return "WAMR " + wamr_version(); }

  std::shared_ptr<Module> compile(std::string_view bytes, bool text, std::string &error) override {
    std::string wat;
    if (text) {
#ifdef CVC_ARIADNE_WASM_BINARYEN
      if (!wat_to_wasm(bytes, wat, error))
        return nullptr;
      bytes = wat;
#else
      error = "WAMR reads binary modules only, and this host has no binaryen to read the text "
              "format: transform the guest at install (cvc_ariadne_wasm_transform)";
      return nullptr;
#endif
    }
    WamrShape shape;
    if (!wamr_inspect(bytes, shape, error))
      return nullptr;
    std::vector<FunctionImport> declared;
    if (!function_imports(bytes, declared, error))
      return nullptr;
    bool start = false;
    if (shape.asyncify && shape.metered) { // transformed at install
      std::string ready = start_as_export(bytes, kStartExport, start, error);
      if (ready.empty())
        return nullptr;
      return std::make_shared<WamrModule>(engine_, std::move(ready), std::move(declared), start);
    }
    if (shape.asyncify) {
      error = "it has Asyncify's exports but isn't metered (ari.gas, ari_gas): transform the "
              "original guest (cvc_ariadne_wasm_transform)";
      return nullptr;
    }
#ifdef CVC_ARIADNE_WASM_BINARYEN
    Metered metered;
    if (!meter(bytes, metered, error)) {
      error = "the gas pass: " + error;
      return nullptr;
    }
    std::string out;
    if (!asyncify(metered.bytes, out, error)) {
      error = "Asyncify: " + error;
      return nullptr;
    }
    std::string ready = start_as_export(out, kStartExport, start, error);
    if (ready.empty())
      return nullptr;
    return std::make_shared<WamrModule>(engine_, std::move(ready), std::move(declared), start);
#else
    error = "it isn't transformed for WAMR (it isn't metered, with ari.gas and ari_gas, or it has "
            "no Asyncify exports), and this host has no binaryen to transform it: install it "
            "through cvc_ariadne_wasm_transform (ari_wasm_meter, then wasm-opt --asyncify "
            "--pass-arg=asyncify-imports@ari.* -O1)";
    return nullptr;
#endif
  }

  std::unique_ptr<Instance> instantiate(const std::shared_ptr<Module> &module,
                                        std::vector<Import> imports, const InstanceLimits &limits,
                                        std::string &error) override {
    auto *m = dynamic_cast<WamrModule *>(module.get());
    if (!m) {
      error = "no module";
      return nullptr;
    }
    // The guest's imports as it was written, as wasmtime's linker sees them.
    if (!imports_match(m->declared(), imports, error))
      return nullptr;
    std::uint64_t max_pages = 0;
    wasm_shared_module_t *shared = m->capped(limits.max_memory, max_pages, error);
    if (!shared)
      return nullptr;
    auto inst = std::make_unique<WamrInstance>(
        engine_, module, std::move(imports), max_pages, limits.max_memory,
        limits.unwind_bytes ? limits.unwind_bytes : kUnwindBytes);
    if (!inst->init(shared, m->start(), error))
      return nullptr;
    return inst;
  }

private:
  EnginePtr engine_;
};

} // namespace

std::unique_ptr<Runtime> make_runtime(std::string &error) {
  wasm_engine_t *engine = wasm_engine_new();
  if (!engine) {
    error = "WAMR: the runtime didn't start";
    return nullptr;
  }
  return std::make_unique<WamrRuntime>(EnginePtr(engine, wasm_engine_delete));
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
