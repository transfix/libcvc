// wasm_engine.cpp — see the header. The wasm: block, a guest instance per document, its imports,
// and the activation that runs one export call on it.

#include "wasm_engine.h"

#include "guest_runtime.h"
#include "value_cbor.h"

#include <charconv>
#include <cstring>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/value.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/exceptions.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {
namespace wasm {

namespace {

namespace se = cvc::state_exec;
namespace fs = std::filesystem;

constexpr std::uint64_t kGasPerStep = 1000;                    // ariadne.limits.wasm_gas_per_step
constexpr std::uint64_t kMaxMemory = std::uint64_t(256) << 20; // ariadne.limits.wasm_max_memory
constexpr std::uint64_t kSetupFuel = 100'000'000; // _initialize, ari_alloc: the guest's own setup

const char *const kKeys[] = {"module", "export", "hooks", "imports", "limits", "priority"};

std::uint64_t count_setting(cvc::state &root, const char *key, std::uint64_t fallback) {
  cvc::state *node = root.findDescendant(key);
  if (!node)
    return fallback;
  const std::string v = node->value();
  std::uint64_t n = 0;
  const auto [end, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
  return !v.empty() && ec == std::errc() && end == v.data() + v.size() ? n : fallback;
}

// The groups a block's imports: names (§8.6), by the function's name.
const char *import_group(const std::string &f) {
  if (f.rfind("state-", 0) == 0)
    return f.find("set") != std::string::npos || f.find("delete") != std::string::npos
               ? "state.write"
               : "state.read";
  if (f.rfind("msg-", 0) == 0)
    return "msg";
  if (f.rfind("http-", 0) == 0 || f.rfind("ws-", 0) == 0 || f.rfind("net-", 0) == 0)
    return "net";
  return "call";
}

// The block's imports: as a narrowing of what call() reaches; null: no narrowing.
std::function<bool(const std::string &)> narrowing(const Value *imports) {
  if (!imports)
    return nullptr;
  std::set<std::string> groups;
  const auto add = [&](const std::string &g) {
    static const std::set<std::string> known = {"state.read", "state.write", "msg", "call", "net"};
    if (!known.count(g))
      throw std::runtime_error("wasm: imports: '" + g +
                               "' is no group (state.read, state.write, msg, call, net)");
    groups.insert(g);
  };
  if (imports->is_seq())
    for (const Value &v : imports->items)
      add(v.as_string());
  else if (!imports->as_string().empty())
    add(imports->as_string());
  return [groups](const std::string &f) { return groups.count(import_group(f)) != 0; };
}

std::int64_t packed(std::uint32_t ptr, std::uint32_t len) {
  return static_cast<std::int64_t>((static_cast<std::uint64_t>(ptr) << 32) | len);
}

// One instance of a block's module, for one document. Its imports reach the activation running
// on it (one at a time: busy), through what they leave here.
struct Guest {
  std::unique_ptr<Instance> instance;
  std::string module; // the module's name, for messages
  std::uint64_t gas_per_step = kGasPerStep;
  std::set<std::string> exports; // the guest's own exports, which its calls may not re-enter
  const void *busy = nullptr;    // the activation whose call is on the instance
  // What the call or yield import asked for, for the activation's next slice.
  struct Request {
    bool yield = false;
    std::string function;
    std::vector<se::value_t> args;
    std::shared_ptr<PendingImport> pending;
  };
  std::optional<Request> request;
  std::string reply;   // the last call's CBOR result, or its error's text, for reply()
  std::string failure; // fail()'s text
};

// The text of guest memory [ptr, ptr + len), or a trap's message.
bool read_guest(GuestMemory &mem, std::int64_t ptr, std::int64_t len, std::string &out,
                std::string &trap) {
  if (ptr < 0 || len < 0 ||
      !mem.span_ok(static_cast<std::uint64_t>(ptr), static_cast<std::uint64_t>(len), trap)) {
    if (trap.empty())
      trap = "an out-of-bounds pointer into the guest's memory";
    return false;
  }
  const std::span<std::uint8_t> b = mem.bytes();
  out.assign(reinterpret_cast<const char *>(b.data()) + ptr, static_cast<std::size_t>(len));
  return true;
}

std::vector<Import> imports_of(Guest *g) {
  std::vector<Import> out;
  Import call;
  call.name = "call";
  call.params = 4;
  call.returns = true;
  call.async = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                   std::string &trap) -> std::shared_ptr<PendingImport> {
    if (!g->busy) {
      trap = "ari.call: no activation is running (a guest's setup can't call the host)";
      return nullptr;
    }
    std::string name, args;
    if (!read_guest(mem, a[0], a[1], name, trap) || !read_guest(mem, a[2], a[3], args, trap))
      return nullptr;
    auto pending = std::make_shared<PendingImport>();
    se::value_t list;
    try {
      list = args.empty() ? se::make_list() : from_cbor(args);
    } catch (const std::exception &e) {
      g->reply = std::string("ValueError: ") + e.what();
      pending->done = true;
      pending->value = -static_cast<std::int64_t>(g->reply.size());
      return pending;
    }
    const auto *items = std::get_if<se::list_ptr>(&list.v);
    if (!items || !*items) {
      g->reply = "TypeError: ari.call: the arguments are a CBOR array";
      pending->done = true;
      pending->value = -static_cast<std::int64_t>(g->reply.size());
      return pending;
    }
    if (g->exports.count(name)) { // a call back into this instance would wait on itself
      g->reply = "RuntimeError: ari.call: '" + name +
                 "' is this guest's own export, and a call can't re-enter the guest it came from";
      pending->done = true;
      pending->value = -static_cast<std::int64_t>(g->reply.size());
      return pending;
    }
    g->request = Guest::Request{false, std::move(name), **items, pending};
    return pending;
  };
  out.push_back(std::move(call));

  Import reply;
  reply.name = "reply";
  reply.params = 1;
  reply.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                   std::string &trap) -> std::int64_t {
    if (a[0] < 0 || !mem.span_ok(static_cast<std::uint64_t>(a[0]), g->reply.size(), trap)) {
      if (trap.empty())
        trap = "an out-of-bounds pointer into the guest's memory";
      return 0;
    }
    std::memcpy(mem.bytes().data() + a[0], g->reply.data(), g->reply.size());
    return 0;
  };
  out.push_back(std::move(reply));

  Import yield;
  yield.name = "yield";
  yield.async = [g](GuestMemory &, std::span<const std::int64_t>,
                    std::string &) -> std::shared_ptr<PendingImport> {
    if (!g->busy)
      return nullptr; // nothing to give the frame back to: go on
    auto pending = std::make_shared<PendingImport>();
    g->request = Guest::Request{true, {}, {}, pending};
    return pending;
  };
  out.push_back(std::move(yield));

  Import fail;
  fail.name = "fail";
  fail.params = 2;
  fail.sync = [g](GuestMemory &mem, std::span<const std::int64_t> a,
                  std::string &trap) -> std::int64_t {
    std::string text;
    if (read_guest(mem, a[0], a[1], text, trap)) {
      g->failure = text.empty() ? "fail" : text;
      trap = g->failure;
    }
    return 0;
  };
  out.push_back(std::move(fail));
  return out;
}

// One call of an export on a guest, as an activation (script_engine.h): its arguments go in
// through ari_alloc, and each step polls the call for one slice's fuel.
class GuestCall : public ScriptActivation {
public:
  GuestCall(std::shared_ptr<Guest> g, std::string name, se::value_t args)
      : g_(std::move(g)), name_(std::move(name)), args_(std::move(args)) {}
  ~GuestCall() override { release(); }

  void step(ScriptSlice &s) override {
    if (!call_) {
      if (g_->busy && g_->busy != this) {
        s.yield_frame(); // one activation per instance: the next frame, once it is free
        return;
      }
      begin(s);
    }
    if (s.has_reply() && g_->request && !g_->request->yield)
      answer(s);
    const std::uint64_t gas = g_->gas_per_step ? g_->gas_per_step : 1;
    const Call::State state = call_->poll(s.budget() * gas);
    s.charge((call_->fuel_used() + gas - 1) / gas);
    switch (state) {
    case Call::State::done:
      s.finish(result());
      release();
      return;
    case Call::State::trapped: {
      const std::string what = g_->failure.empty() ? call_->trap() : g_->failure;
      release();
      throw se::dsl_error("WasmError", g_->module + ": " + name_ + ": " + what, "wasm",
                          {{"trap", se::value_t(what)}});
    }
    case Call::State::waiting:
      if (!g_->request) {
        s.yield_frame();
        return;
      }
      if (g_->request->yield) {
        g_->request->pending->done = true;
        g_->request.reset();
        s.yield_frame();
        return;
      }
      s.call(g_->request->function, g_->request->args);
      return;
    case Call::State::sliced:
      return; // more to do: the next slice
    }
  }

  void cancel() override { release(); }

private:
  void begin(ScriptSlice &s) {
    g_->busy = this;
    g_->failure.clear();
    const std::string cbor = to_cbor(args_); // a value that can't cross raises a ValueError
    std::int64_t ptr = 0;
    std::uint64_t used = 0;
    std::string error;
    if (!g_->instance->run("ari_alloc", {static_cast<std::int64_t>(cbor.size())}, kSetupFuel, ptr,
                           used, error))
      fail("ari_alloc: " + error);
    const std::span<std::uint8_t> mem = g_->instance->memory();
    if (ptr < 0 || static_cast<std::uint64_t>(ptr) > mem.size() ||
        cbor.size() > mem.size() - static_cast<std::uint64_t>(ptr))
      fail("ari_alloc gave room outside the guest's memory");
    std::memcpy(mem.data() + ptr, cbor.data(), cbor.size());
    call_ = g_->instance->start(name_, {ptr, static_cast<std::int64_t>(cbor.size())}, error);
    if (!call_)
      fail(error);
    const std::uint64_t gas = g_->gas_per_step ? g_->gas_per_step : 1;
    s.charge((used + gas - 1) / gas);
  }

  // The last slice's call came back: its CBOR, or its error's text, waits for reply().
  void answer(ScriptSlice &s) {
    std::shared_ptr<PendingImport> pending = g_->request->pending;
    g_->request.reset();
    if (s.failed()) {
      g_->reply = s.error();
      pending->value = -static_cast<std::int64_t>(g_->reply.size());
    } else {
      try {
        g_->reply = to_cbor(s.reply());
        pending->value = static_cast<std::int64_t>(g_->reply.size());
      } catch (const std::exception &e) {
        g_->reply = std::string("TypeError: ") + e.what();
        pending->value = -static_cast<std::int64_t>(g_->reply.size());
      }
    }
    pending->done = true;
  }

  se::value_t result() {
    const auto r = static_cast<std::uint64_t>(call_->result());
    const auto ptr = static_cast<std::uint32_t>(r >> 32);
    const auto len = static_cast<std::uint32_t>(r & 0xffffffffu);
    if (len == 0)
      return se::value_t();
    const std::span<std::uint8_t> mem = g_->instance->memory();
    if (ptr > mem.size() || len > mem.size() - ptr)
      fail("its result points outside the guest's memory");
    try {
      return from_cbor(std::string_view(reinterpret_cast<const char *>(mem.data()) + ptr, len));
    } catch (const std::exception &e) {
      fail(std::string("its result isn't CBOR: ") + e.what());
    }
    return se::value_t();
  }

  [[noreturn]] void fail(const std::string &what) {
    release();
    throw se::dsl_error("WasmError", g_->module + ": " + name_ + ": " + what, "wasm",
                        {{"trap", se::value_t(what)}});
  }

  void release() {
    call_.reset();
    if (g_ && g_->busy == this) {
      g_->busy = nullptr;
      g_->request.reset();
    }
  }

  std::shared_ptr<Guest> g_;
  std::string name_;
  se::value_t args_;
  std::unique_ptr<Call> call_;
};

std::string read_file(const fs::path &p) {
  std::ifstream in(p, std::ios::binary);
  if (!in)
    throw std::runtime_error("wasm: module: can't read " + p.string());
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

class WasmEngine : public ScriptEngine {
public:
  explicit WasmEngine(std::unique_ptr<Runtime> rt) : runtime_(std::move(rt)) {}

  std::string language() const override { return "wasm"; }

  void load_block(const ScriptBlock &block, ScriptLane &lane,
                  std::vector<std::string> &warnings) override {
    Value b = block.content;
    if (b.is_scalar()) { // the module alone
      Value m;
      m.kind = Value::Kind::Map;
      m.entries.emplace_back("module", b);
      b = std::move(m);
    }
    if (!b.is_map())
      throw std::runtime_error("wasm: is a module's path, or a map of module:, export:, hooks:, "
                               "imports:, limits:, priority:");
    for (const Value::Entry &e : b.entries) {
      bool known = false;
      for (const char *k : kKeys)
        known = known || e.first == k;
      if (!known)
        warnings.push_back("wasm: unknown key '" + e.first +
                           "' ignored (it takes module, export, hooks, imports, limits, priority)");
    }
    const std::string module = b.find("module") ? b.find("module")->as_string() : std::string();
    if (module.empty())
      throw std::runtime_error("wasm: module: names the guest's .wasm (or .wat) file");
    if (module.rfind("cvcpkg:", 0) == 0)
      throw std::runtime_error("wasm: module: cvcpkg: packages come later (phase G5); name a "
                               "file beside the document");
    fs::path path(module);
    if (path.is_relative() && !block.dir.empty())
      path = fs::path(block.dir) / path;
    std::shared_ptr<Module> compiled = compile(path);

    cvc::state &root = cvc::state::instance(lane.app());
    InstanceLimits limits;
    limits.max_memory = count_setting(root, "ariadne.limits.wasm_max_memory", kMaxMemory);
    if (const Value *l = b.find("limits"))
      if (const Value *mb = l->find("memory_mb")) {
        const double v = mb->as_double(-1.0);
        if (v < 0)
          throw std::runtime_error("wasm: limits: memory_mb is a number of MiB");
        const auto bytes = static_cast<std::uint64_t>(v * 1024.0 * 1024.0);
        if (limits.max_memory == 0 || bytes < limits.max_memory)
          limits.max_memory = bytes; // a block may lower the host's cap, never raise it
      }

    auto g = std::make_shared<Guest>();
    g->module = path.filename().string();
    g->gas_per_step = count_setting(root, "ariadne.limits.wasm_gas_per_step", kGasPerStep);
    std::string error;
    g->instance = runtime_->instantiate(compiled, imports_of(g.get()), limits, error);
    if (!g->instance)
      throw std::runtime_error("wasm: " + g->module + ": " + error);
    if (g->instance->has_export("_initialize")) {
      std::int64_t ignored = 0;
      std::uint64_t used = 0;
      if (!g->instance->run("_initialize", {}, kSetupFuel, ignored, used, error))
        throw std::runtime_error("wasm: " + g->module + ": _initialize: " + error);
    }

    ActivationOptions options;
    options.may_call = narrowing(b.find("imports"));
    if (const Value *p = b.find("priority"))
      options.priority = static_cast<int>(p->as_double(0.0));
    const std::string stem = path.stem().string();
    const std::weak_ptr<Guest> weak = g;
    // What runs `name` on the guest with `args`, as long as the document's instance is there.
    const auto runner = [weak](const std::string &name) {
      return [weak, name](se::value_t args) -> std::shared_ptr<ScriptActivation> {
        std::shared_ptr<Guest> guest = weak.lock();
        if (!guest)
          throw std::runtime_error(name + ": its guest has gone (the document was reloaded)");
        return std::make_shared<GuestCall>(std::move(guest), name, std::move(args));
      };
    };
    const auto needs = [&](const std::string &name, const char *what) {
      if (!g->instance->has_export(name))
        throw std::runtime_error("wasm: " + g->module + " exports no function '" + name + "' (" +
                                 what + ")");
      g->exports.insert(name);
    };
    const bool calls = b.find("export") || b.find("hooks");
    if (calls && (!g->instance->has_export("ari_alloc") || g->instance->memory().empty()))
      throw std::runtime_error("wasm: " + g->module +
                               " exports no ari_alloc or memory, so no value can cross to it");

    if (const Value *ex = b.find("export")) {
      std::vector<Value> entries = ex->is_seq() ? ex->items : std::vector<Value>{*ex};
      for (const Value &e : entries) {
        const std::string name =
            e.is_map() ? (e.find("name") ? e.find("name")->as_string() : "") : e.as_string();
        if (name.empty())
          throw std::runtime_error("wasm: export: an entry is a name or {name:, as:}");
        const std::string as = e.is_map() && e.find("as") ? e.find("as")->as_string() : name;
        needs(name, "export:");
        ActivationOptions o = options;
        o.name = "wasm/" + stem + ":" + name;
        const auto run = runner(name);
        lane.export_activation(
            as,
            [run](std::span<const se::value_t> args) {
              return run(se::make_list(std::vector<se::value_t>(args.begin(), args.end())));
            },
            o);
      }
    }

    if (const Value *hooks = b.find("hooks")) {
      const auto hook = [&](const std::string &name, const std::string &what) {
        needs(name, what.c_str());
        const auto run = runner(name);
        return ScriptHook([run](const se::value_t &event) { return run(se::make_list({event})); });
      };
      const auto named = [&](const char *kind) {
        ActivationOptions o = options;
        o.name = "wasm/" + stem + ":" + kind;
        return o;
      };
      if (const Value *t = hooks->find("on_tick"))
        lane.on_frame(hook(t->as_string(), "hooks: on_tick"), named("on_tick"));
      if (const Value *on = hooks->find("on"))
        for (const Value::Entry &e : on->entries)
          lane.on_event(e.first, hook(e.second.as_string(), "hooks: on"), named("on"));
      if (const Value *w = hooks->find("watch"))
        for (const Value::Entry &e : w->entries)
          lane.on_watch(e.first, hook(e.second.as_string(), "hooks: watch"), named("watch"));
      for (const Value::Entry &e : hooks->entries)
        if (e.first != "on_tick" && e.first != "on" && e.first != "watch")
          warnings.push_back("wasm: hooks: '" + e.first +
                             "' is not a hook yet (on_tick, on, watch); ignored");
    }
    guests_[&lane] = std::move(g);
  }

  void unload(ScriptLane &lane) override { guests_.erase(&lane); }

private:
  // The module at `path`, compiled once while the file stays the same.
  std::shared_ptr<Module> compile(const fs::path &path) {
    std::error_code ec;
    const std::string key =
        std::to_string(fs::file_size(path, ec)) + ":" +
        std::to_string(fs::last_write_time(path, ec).time_since_epoch().count());
    Compiled &c = modules_[fs::absolute(path, ec).string()];
    if (c.module && c.key == key)
      return c.module;
    const std::string bytes = read_file(path);
    std::string error;
    std::shared_ptr<Module> m = runtime_->compile(bytes, path.extension() == ".wat", error);
    if (!m)
      throw std::runtime_error("wasm: " + path.filename().string() + ": " + error);
    c = Compiled{key, m};
    return m;
  }

  struct Compiled {
    std::string key;
    std::shared_ptr<Module> module;
  };
  std::unique_ptr<Runtime> runtime_;
  std::map<std::string, Compiled> modules_;
  std::map<ScriptLane *, std::shared_ptr<Guest>> guests_;
};

} // namespace

std::shared_ptr<ScriptEngine> make_engine(std::string &error) {
  std::unique_ptr<Runtime> rt = make_runtime(error);
  if (!rt)
    return nullptr;
  return std::make_shared<WasmEngine>(std::move(rt));
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
