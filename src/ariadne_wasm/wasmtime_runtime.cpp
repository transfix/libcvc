// wasmtime_runtime.cpp — guest_runtime.h on wasmtime's C API, natively (§8.3).
//
// Every store consumes fuel, and every call runs through the async API: wasmtime_func_call_async
// runs the guest on a stack of its own, and each Call::poll is one wasmtime_call_future_poll. A
// poll ends when the call returns, when the slice's fuel is used (the store yields every `fuel`
// units: wasmtime_context_fuel_async_yield_interval), or when the guest is inside an async import
// whose PendingImport isn't done yet (its continuation says so).

#include "guest_runtime.h"

#include <utility>
#include <wasmtime.h>

namespace cvc {
namespace ariadne {
namespace wasm {

namespace {

// The fuel a store starts with: enough never to run out; each poll's slice bounds the work.
constexpr std::uint64_t kStoreFuel = std::uint64_t(1) << 62;

std::string take_error(wasmtime_error_t *e) {
  if (!e)
    return {};
  wasm_name_t msg;
  wasmtime_error_message(e, &msg);
  std::string s(msg.data, msg.size);
  wasm_byte_vec_delete(&msg);
  wasmtime_error_delete(e);
  return s;
}

std::string take_trap(wasm_trap_t *t) {
  if (!t)
    return {};
  wasm_message_t msg;
  wasm_trap_message(t, &msg);
  std::string s(msg.data, msg.size);
  wasm_byte_vec_delete(&msg);
  wasm_trap_delete(t);
  while (!s.empty() && s.back() == '\0')
    s.pop_back();
  return s;
}

using EnginePtr = std::shared_ptr<wasm_engine_t>;

class WtModule : public Module {
public:
  WtModule(EnginePtr engine, wasmtime_module_t *m) : engine_(std::move(engine)), m_(m) {}
  ~WtModule() override { wasmtime_module_delete(m_); }
  wasmtime_module_t *get() const { return m_; }

private:
  EnginePtr engine_;
  wasmtime_module_t *m_;
};

class WtMemory : public GuestMemory {
public:
  WtMemory(wasmtime_context_t *ctx, const wasmtime_memory_t *mem) : ctx_(ctx) {
    if (mem) {
      mem_ = *mem;
      has_ = true;
    }
  }
  std::span<std::uint8_t> bytes() override {
    if (!has_)
      return {};
    return {wasmtime_memory_data(ctx_, &mem_), wasmtime_memory_data_size(ctx_, &mem_)};
  }

private:
  wasmtime_context_t *ctx_;
  wasmtime_memory_t mem_{};
  bool has_ = false;
};

WtMemory caller_memory(wasmtime_caller_t *caller) {
  wasmtime_extern_t item;
  const bool found =
      wasmtime_caller_export_get(caller, "memory", 6, &item) && item.kind == WASMTIME_EXTERN_MEMORY;
  return WtMemory(wasmtime_caller_context(caller), found ? &item.of.memory : nullptr);
}

std::vector<std::int64_t> widen(const wasmtime_val_t *args, size_t n) {
  std::vector<std::int64_t> a;
  a.reserve(n);
  for (size_t i = 0; i < n; ++i)
    a.push_back(args[i].kind == WASMTIME_I32 ? args[i].of.i32 : args[i].of.i64);
  return a;
}

// What an instance's imports share with its calls: the async import the guest waits in, if any.
struct InstanceState {
  std::shared_ptr<PendingImport> waiting;
};

struct ImportEnv {
  Import import;
  std::shared_ptr<InstanceState> state;
};

void free_import_env(void *env) { delete static_cast<ImportEnv *>(env); }

wasm_trap_t *sync_import(void *env, wasmtime_caller_t *caller, const wasmtime_val_t *args,
                         size_t nargs, wasmtime_val_t *results, size_t nresults) {
  auto *ie = static_cast<ImportEnv *>(env);
  const std::vector<std::int64_t> a = widen(args, nargs);
  WtMemory mem = caller_memory(caller);
  std::string trap;
  std::int64_t r = 0;
  try {
    r = ie->import.sync(mem, a, trap);
  } catch (const std::exception &e) {
    trap = e.what();
  }
  if (!trap.empty())
    return wasmtime_trap_new(trap.data(), trap.size());
  if (nresults > 0) {
    results[0].kind = WASMTIME_I64;
    results[0].of.i64 = r;
  }
  return nullptr;
}

struct Continuation {
  std::shared_ptr<PendingImport> pending;
  wasmtime_val_t *results;
  size_t nresults;
};

bool continuation_done(void *env) {
  auto *c = static_cast<Continuation *>(env);
  if (!c->pending->done)
    return false;
  if (c->nresults > 0) {
    c->results[0].kind = WASMTIME_I64;
    c->results[0].of.i64 = c->pending->value;
  }
  return true;
}

void free_continuation(void *env) { delete static_cast<Continuation *>(env); }

void async_import(void *env, wasmtime_caller_t *caller, const wasmtime_val_t *args, size_t nargs,
                  wasmtime_val_t *results, size_t nresults, wasm_trap_t **trap_ret,
                  wasmtime_async_continuation_t *continuation_ret) {
  auto *ie = static_cast<ImportEnv *>(env);
  const std::vector<std::int64_t> a = widen(args, nargs);
  WtMemory mem = caller_memory(caller);
  std::string trap;
  std::shared_ptr<PendingImport> pending;
  try {
    pending = ie->import.async(mem, a, trap);
  } catch (const std::exception &e) {
    trap = e.what();
  }
  if (!pending) {
    pending = std::make_shared<PendingImport>();
    pending->done = true;
  }
  if (!trap.empty()) {
    *trap_ret = wasmtime_trap_new(trap.data(), trap.size());
    pending->done = true;
  }
  if (!pending->done)
    ie->state->waiting = pending;
  continuation_ret->callback = continuation_done;
  continuation_ret->env = new Continuation{pending, results, nresults};
  continuation_ret->finalizer = free_continuation;
}

wasm_functype_t *import_type(const Import &im) {
  wasm_valtype_vec_t params, results;
  wasm_valtype_vec_new_uninitialized(&params, static_cast<size_t>(im.params));
  for (int i = 0; i < im.params; ++i)
    params.data[i] = (im.wide >> i) & 1u ? wasm_valtype_new_i64() : wasm_valtype_new_i32();
  if (im.returns) {
    wasm_valtype_vec_new_uninitialized(&results, 1);
    results.data[0] = wasm_valtype_new_i64();
  } else {
    wasm_valtype_vec_new_empty(&results);
  }
  return wasm_functype_new(&params, &results);
}

class WtInstance;

class WtCall : public Call {
public:
  WtCall(EnginePtr engine, wasmtime_context_t *ctx, wasmtime_func_t func,
         std::vector<wasmtime_val_t> args, size_t nresults, std::shared_ptr<InstanceState> state)
      : engine_(std::move(engine)), ctx_(ctx), func_(func), args_(std::move(args)),
        results_(nresults), state_(std::move(state)) {}
  ~WtCall() override {
    if (future_)
      wasmtime_call_future_delete(future_);
    state_->waiting.reset();
    take_trap(trap_);
    take_error(error_);
  }

  State poll(std::uint64_t fuel) override {
    used_ = 0;
    if (finished_)
      return trap_text_.empty() ? State::done : State::trapped;
    if (state_->waiting && state_->waiting->done)
      state_->waiting.reset();
    if (state_->waiting)
      return State::waiting; // nothing to run until the host answers
    if (std::string e =
            take_error(wasmtime_context_fuel_async_yield_interval(ctx_, fuel ? fuel : 1));
        !e.empty())
      return fail(e);
    if (!future_)
      future_ = wasmtime_func_call_async(ctx_, &func_, args_.data(), args_.size(), results_.data(),
                                         results_.size(), &trap_, &error_);
    std::uint64_t before = 0, after = 0;
    wasmtime_context_get_fuel(ctx_, &before);
    const bool done = wasmtime_call_future_poll(future_);
    wasmtime_context_get_fuel(ctx_, &after);
    used_ = before > after ? before - after : 0;
    if (!done)
      return state_->waiting && !state_->waiting->done ? State::waiting : State::sliced;
    wasmtime_call_future_delete(future_);
    future_ = nullptr;
    finished_ = true;
    if (trap_) {
      trap_text_ = take_trap(trap_);
      trap_ = nullptr;
      if (trap_text_.empty())
        trap_text_ = "trap";
      return State::trapped;
    }
    if (error_) {
      trap_text_ = take_error(error_);
      error_ = nullptr;
      if (trap_text_.empty())
        trap_text_ = "error";
      return State::trapped;
    }
    if (!results_.empty())
      result_ = results_[0].kind == WASMTIME_I32 ? results_[0].of.i32 : results_[0].of.i64;
    return State::done;
  }

  std::uint64_t fuel_used() const override { return used_; }
  std::int64_t result() const override { return result_; }
  const std::string &trap() const override { return trap_text_; }

private:
  State fail(std::string why) {
    finished_ = true;
    trap_text_ = std::move(why);
    return State::trapped;
  }

  EnginePtr engine_;
  wasmtime_context_t *ctx_;
  wasmtime_func_t func_;
  std::vector<wasmtime_val_t> args_;    // kept alive, unmodified, while the future lives
  std::vector<wasmtime_val_t> results_; // ditto
  std::shared_ptr<InstanceState> state_;
  wasmtime_call_future_t *future_ = nullptr;
  wasm_trap_t *trap_ = nullptr;
  wasmtime_error_t *error_ = nullptr;
  bool finished_ = false;
  std::uint64_t used_ = 0;
  std::int64_t result_ = 0;
  std::string trap_text_;
};

class WtInstance : public Instance {
public:
  WtInstance(EnginePtr engine, std::shared_ptr<Module> module)
      : engine_(std::move(engine)), module_(std::move(module)),
        state_(std::make_shared<InstanceState>()) {}
  ~WtInstance() override {
    if (linker_)
      wasmtime_linker_delete(linker_);
    if (store_)
      wasmtime_store_delete(store_);
  }

  bool init(std::vector<Import> imports, const InstanceLimits &limits, std::string &error) {
    store_ = wasmtime_store_new(engine_.get(), nullptr, nullptr);
    ctx_ = wasmtime_store_context(store_);
    max_memory_ = limits.max_memory;
    if (limits.max_memory > 0)
      wasmtime_store_limiter(store_, static_cast<std::int64_t>(limits.max_memory), -1, -1, -1, -1);
    if ((error = take_error(wasmtime_context_set_fuel(ctx_, kStoreFuel))), !error.empty())
      return false;
    linker_ = wasmtime_linker_new(engine_.get());
    for (Import &im : imports) {
      wasm_functype_t *ty = import_type(im);
      const bool async = static_cast<bool>(im.async);
      const std::string name = im.name;
      auto *env = new ImportEnv{std::move(im), state_};
      wasmtime_error_t *e =
          async ? wasmtime_linker_define_async_func(linker_, "ari", 3, name.data(), name.size(), ty,
                                                    async_import, env, free_import_env)
                : wasmtime_linker_define_func(linker_, "ari", 3, name.data(), name.size(), ty,
                                              sync_import, env, free_import_env);
      wasm_functype_delete(ty);
      if ((error = take_error(e)), !error.empty())
        return false;
    }
    const auto *m = static_cast<const WtModule *>(module_.get());
    wasm_trap_t *trap = nullptr;
    wasmtime_error_t *err = nullptr;
    wasmtime_call_future_t *f =
        wasmtime_linker_instantiate_async(linker_, ctx_, m->get(), &instance_, &trap, &err);
    // A start function runs here, to its end: it may not wait on an import.
    for (int polls = 0; !wasmtime_call_future_poll(f); ++polls)
      if (state_->waiting || polls > 1000000) {
        wasmtime_call_future_delete(f);
        state_->waiting.reset();
        take_trap(trap);
        take_error(err);
        error = "the module's start function waited on an import";
        return false;
      }
    wasmtime_call_future_delete(f);
    if (trap) {
      error = take_trap(trap);
      take_error(err);
      return false;
    }
    if (err) {
      error = take_error(err);
      return false;
    }
    return true;
  }

  bool has_export(const std::string &name) const override {
    wasmtime_extern_t item;
    if (!wasmtime_instance_export_get(ctx_, &instance_, name.data(), name.size(), &item))
      return false;
    const bool func = item.kind == WASMTIME_EXTERN_FUNC;
    wasmtime_extern_delete(&item);
    return func;
  }

  std::span<std::uint8_t> memory() override {
    wasmtime_extern_t item;
    if (!wasmtime_instance_export_get(ctx_, &instance_, "memory", 6, &item))
      return {};
    std::span<std::uint8_t> bytes;
    if (item.kind == WASMTIME_EXTERN_MEMORY)
      bytes = {wasmtime_memory_data(ctx_, &item.of.memory),
               wasmtime_memory_data_size(ctx_, &item.of.memory)};
    wasmtime_extern_delete(&item);
    return bytes;
  }

  bool memory_at_limit() override {
    wasmtime_extern_t item;
    if (!wasmtime_instance_export_get(ctx_, &instance_, "memory", 6, &item))
      return false;
    bool at = false;
    if (item.kind == WASMTIME_EXTERN_MEMORY) {
      constexpr std::uint64_t kPage = 65536;
      const std::uint64_t size = wasmtime_memory_data_size(ctx_, &item.of.memory);
      at = max_memory_ > 0 && size + kPage > max_memory_;
      wasm_memorytype_t *ty = wasmtime_memory_type(ctx_, &item.of.memory);
      std::uint64_t max_pages = 0;
      if (wasmtime_memorytype_maximum(ty, &max_pages) && size / kPage >= max_pages)
        at = true; // the module's own maximum
      wasm_memorytype_delete(ty);
    }
    wasmtime_extern_delete(&item);
    return at;
  }

  std::unique_ptr<Call> start(const std::string &name, std::vector<std::int64_t> args,
                              std::string &error) override {
    wasmtime_extern_t item;
    if (!wasmtime_instance_export_get(ctx_, &instance_, name.data(), name.size(), &item) ||
        item.kind != WASMTIME_EXTERN_FUNC) {
      error = "the guest exports no function '" + name + "'";
      return nullptr;
    }
    const wasmtime_func_t func = item.of.func;
    wasm_functype_t *ty = wasmtime_func_type(ctx_, &func);
    const wasm_valtype_vec_t *params = wasm_functype_params(ty);
    const wasm_valtype_vec_t *results = wasm_functype_results(ty);
    if (params->size != args.size()) {
      error = "'" + name + "' takes " + std::to_string(params->size) + " arguments, not " +
              std::to_string(args.size());
      wasm_functype_delete(ty);
      return nullptr;
    }
    std::vector<wasmtime_val_t> vals(args.size());
    for (size_t i = 0; i < args.size(); ++i) {
      const wasm_valkind_t k = wasm_valtype_kind(params->data[i]);
      if (k == WASM_I32) {
        vals[i].kind = WASMTIME_I32;
        vals[i].of.i32 = static_cast<std::int32_t>(args[i]);
      } else if (k == WASM_I64) {
        vals[i].kind = WASMTIME_I64;
        vals[i].of.i64 = args[i];
      } else {
        error = "'" + name + "' takes a parameter that isn't an i32 or an i64";
        wasm_functype_delete(ty);
        return nullptr;
      }
    }
    const size_t nresults = results->size;
    wasm_functype_delete(ty);
    return std::make_unique<WtCall>(engine_, ctx_, func, std::move(vals), nresults, state_);
  }

  bool run(const std::string &name, std::vector<std::int64_t> args, std::uint64_t fuel,
           std::int64_t &result, std::uint64_t &used, std::string &error) override {
    std::unique_ptr<Call> call = start(name, std::move(args), error);
    if (!call)
      return false;
    const Call::State s = call->poll(fuel);
    used += call->fuel_used();
    switch (s) {
    case Call::State::done:
      result = call->result();
      return true;
    case Call::State::trapped:
      error = call->trap();
      return false;
    case Call::State::sliced:
      error = "'" + name + "' ran out of fuel";
      return false;
    case Call::State::waiting:
      error = "'" + name + "' waited on an import";
      return false;
    }
    return false;
  }

private:
  EnginePtr engine_;
  std::shared_ptr<Module> module_;
  std::shared_ptr<InstanceState> state_;
  wasmtime_store_t *store_ = nullptr;
  wasmtime_context_t *ctx_ = nullptr;
  wasmtime_linker_t *linker_ = nullptr;
  wasmtime_instance_t instance_{};
  std::uint64_t max_memory_ = 0; // InstanceLimits::max_memory
};

class WtRuntime : public Runtime {
public:
  explicit WtRuntime(EnginePtr engine) : engine_(std::move(engine)) {}
  std::string name() const override { return "wasmtime"; }

  std::shared_ptr<Module> compile(std::string_view bytes, bool text, std::string &error) override {
    wasm_byte_vec_t wasm{};
    if (text) {
      if ((error = take_error(wasmtime_wat2wasm(bytes.data(), bytes.size(), &wasm))),
          !error.empty())
        return nullptr;
      bytes = std::string_view(wasm.data, wasm.size);
    }
    wasmtime_module_t *m = nullptr;
    error = take_error(wasmtime_module_new(
        engine_.get(), reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size(), &m));
    if (text)
      wasm_byte_vec_delete(&wasm);
    if (!error.empty())
      return nullptr;
    return std::make_shared<WtModule>(engine_, m);
  }

  std::unique_ptr<Instance> instantiate(const std::shared_ptr<Module> &module,
                                        std::vector<Import> imports, const InstanceLimits &limits,
                                        std::string &error) override {
    if (!module) {
      error = "no module";
      return nullptr;
    }
    auto inst = std::make_unique<WtInstance>(engine_, module);
    if (!inst->init(std::move(imports), limits, error))
      return nullptr;
    return inst;
  }

private:
  EnginePtr engine_;
};

} // namespace

std::unique_ptr<Runtime> make_runtime(std::string &error) {
  wasm_config_t *config = wasm_config_new();
  if (!config) {
    error = "wasmtime: no config";
    return nullptr;
  }
  wasmtime_config_consume_fuel_set(config, true);
  wasm_engine_t *engine = wasm_engine_new_with_config(config); // takes the config
  if (!engine) {
    error = "wasmtime: the engine didn't start";
    return nullptr;
  }
  return std::make_unique<WtRuntime>(EnginePtr(engine, wasm_engine_delete));
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
