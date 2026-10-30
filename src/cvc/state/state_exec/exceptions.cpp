// DSL exceptions: the built-in classes, exception values, and the mapping from C++ exceptions
// (docs/STATE_EXEC_EXCEPTIONS.md, section 2).
#include "closure_natives.h" // detail::class_constructor

#include <cstdlib>
#include <cvc/state/state_exec/evaluator.h> // evaluation_interrupted / evaluation_timeout
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/parser.h> // parse_error
#include <memory>
#include <new>
#include <optional>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#if defined(__GNUG__)
#include <cxxabi.h>
#endif

namespace cvc::state_exec {

namespace {

// A built-in class's init: (init self [message [data]]) sets the fields and returns self.
struct exception_init {
  value_t operator()(std::span<const value_t> args) const {
    if (args.empty())
      throw dsl_error("TypeError", "init: expected the instance");
    const value_t &self = args[0];
    if (!std::holds_alternative<dict_ptr>(self.v))
      throw dsl_error("TypeError", "init: expected an instance, got " + self.type_name());
    std::string message;
    if (args.size() > 1)
      message = std::holds_alternative<std::string>(args[1].v) ? std::get<std::string>(args[1].v)
                                                               : to_string(args[1]);
    set_field(self, "message", value_t(message));
    set_field(self, "data", args.size() > 2 ? args[2] : nil_value);
    if (!field(self, "cause"))
      set_field(self, "cause", nil_value);
    if (!field(self, "origin"))
      set_field(self, "origin", value_t(std::string("dsl")));
    if (!field(self, "traceback"))
      set_field(self, "traceback", nil_value);
    return self;
  }
};

struct builtin_classes {
  std::vector<std::string> names; // parents before children
  std::unordered_map<std::string, dict_ptr> by_name;
};

// Built once per process and never freed: they are shared by every environment for the life of
// the process, and leaking them keeps static destruction order out of it.
const builtin_classes &classes() {
  static const builtin_classes *built = [] {
    auto *c = new builtin_classes;
    const std::pair<const char *, const char *> hierarchy[] = {
        {"Exception", nullptr},          {"RuntimeError", "Exception"},
        {"TypeError", "Exception"},      {"ValueError", "Exception"},
        {"NameError", "Exception"},      {"KeyError", "Exception"},
        {"IndexError", "Exception"},     {"ArithmeticError", "Exception"},
        {"MemoryError", "Exception"},    {"ParseError", "Exception"},
        {"TimeoutError", "Exception"},   {"NativeError", "RuntimeError"},
        {"PythonError", "RuntimeError"}, {"LockError", "RuntimeError"},
        {"DeadlockError", "LockError"},  {"PermissionError", "Exception"},
    };
    for (const auto &[name, parent] : hierarchy) {
      auto cls = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
      cls->emplace_back("__name__", value_t(std::string(name)));
      if (parent)
        cls->emplace_back("__super__", value_t(c->by_name.at(parent)));
      cls->emplace_back("init", value_t(native_fn{exception_init{}}));
      c->names.emplace_back(name);
      c->by_name.emplace(name, std::move(cls));
    }
    return c;
  }();
  return *built;
}

std::string type_name_of(const std::exception &e) {
  const char *raw = typeid(e).name();
#if defined(__GNUG__)
  int status = 0;
  std::unique_ptr<char, void (*)(void *)> demangled(
      abi::__cxa_demangle(raw, nullptr, nullptr, &status), std::free);
  if (status == 0 && demangled)
    return demangled.get();
  return raw;
#else
  std::string name = raw; // MSVC: "class foo::bar" / "struct foo"
  for (const char *prefix : {"class ", "struct "})
    if (name.rfind(prefix, 0) == 0)
      return name.substr(std::char_traits<char>::length(prefix));
  return name;
#endif
}

value_t native_exception(const char *cls, const std::exception &e) {
  return make_exception(exception_class(cls), e.what(), nil_value, "native");
}

} // namespace

dsl_exception::dsl_exception(value_t exc, std::exception_ptr original)
    : std::runtime_error(exception_summary(exc)), exc_(std::move(exc)),
      original_(std::move(original)) {}

std::string dsl_exception::class_name() const {
  return cvc::state_exec::class_name(class_of_instance(exc_));
}

std::vector<std::string> dsl_exception::class_chain() const {
  return cvc::state_exec::class_chain(class_of_instance(exc_));
}

dict_ptr exception_class(std::string_view name) {
  const auto &c = classes();
  const auto it = c.by_name.find(std::string(name));
  return it == c.by_name.end() ? nullptr : it->second;
}

const std::vector<std::string> &exception_class_names() { return classes().names; }

void register_exception_classes(const environment_ptr &env) {
  for (const auto &name : exception_class_names()) {
    const dict_ptr cls = exception_class(name);
    env->set(name, value_t(native_fn{detail::class_constructor{cls}}));
    env->set(name + ".__cls_dict__", value_t(cls));
  }
}

const value_t *field(const value_t &dict, std::string_view key) {
  const auto *d = std::get_if<dict_ptr>(&dict.v);
  if (!d || !*d)
    return nullptr;
  for (const auto &kv : **d)
    if (kv.first == key)
      return &kv.second;
  return nullptr;
}

void set_field(const value_t &dict, const std::string &key, value_t v) {
  const auto *d = std::get_if<dict_ptr>(&dict.v);
  if (!d || !*d)
    return;
  for (auto &kv : **d)
    if (kv.first == key) {
      kv.second = std::move(v);
      return;
    }
  (*d)->emplace_back(key, std::move(v));
}

value_t make_exception(const dict_ptr &cls, const std::string &message, value_t data,
                       const std::string &origin) {
  auto inst = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
  inst->emplace_back("__class__", value_t(cls ? cls : exception_class("Exception")));
  inst->emplace_back("message", value_t(message));
  inst->emplace_back("data", std::move(data));
  inst->emplace_back("cause", nil_value);
  inst->emplace_back("origin", value_t(origin));
  inst->emplace_back("traceback", nil_value);
  return value_t(std::move(inst));
}

dict_ptr class_of_instance(const value_t &v) {
  const value_t *cls = field(v, "__class__");
  if (!cls)
    return nullptr;
  const auto *d = std::get_if<dict_ptr>(&cls->v);
  return d ? *d : nullptr;
}

dict_ptr class_of_constructor(const value_t &ctor) {
  const auto *fn = std::get_if<native_fn>(&ctor.v);
  if (!fn)
    return nullptr;
  const auto *made = fn->target<detail::class_constructor>();
  return made ? made->cls_dict : nullptr;
}

bool is_instance(const value_t &v, const dict_ptr &cls) {
  if (!cls)
    return false;
  // A snapshot writes a class dict once for each value holding it, so after a restore an
  // instance's class can be a copy of the one a program names: the same class by its chain of
  // names. Built lazily: most matches are by identity.
  std::optional<std::vector<std::string>> wanted;
  dict_ptr at = class_of_instance(v);
  for (int hop = 0; at && hop < 4096; ++hop) { // bounded: a class dict could be made to loop
    if (at.get() == cls.get())
      return true;
    if (!class_name(at).empty() && class_name(at) == class_name(cls)) {
      if (!wanted)
        wanted = class_chain(cls);
      if (class_chain(at) == *wanted)
        return true;
    }
    const value_t *super = field(value_t(at), "__super__");
    const auto *next = super ? std::get_if<dict_ptr>(&super->v) : nullptr;
    at = next ? *next : nullptr;
  }
  return false;
}

bool is_exception(const value_t &v) { return is_instance(v, exception_class("Exception")); }

std::string class_name(const dict_ptr &cls) {
  if (!cls)
    return "";
  const value_t *name = field(value_t(cls), "__name__");
  const auto *s = name ? std::get_if<std::string>(&name->v) : nullptr;
  return s ? *s : "";
}

std::vector<std::string> class_chain(const dict_ptr &cls) {
  std::vector<std::string> chain;
  dict_ptr at = cls;
  for (int hop = 0; at && hop < 4096; ++hop) {
    chain.push_back(class_name(at));
    const value_t *super = field(value_t(at), "__super__");
    const auto *next = super ? std::get_if<dict_ptr>(&super->v) : nullptr;
    at = next ? *next : nullptr;
  }
  return chain;
}

std::string exception_message(const value_t &exc) {
  const value_t *message = field(exc, "message");
  if (!message)
    return "";
  const auto *s = std::get_if<std::string>(&message->v);
  return s ? *s : to_string(*message);
}

std::string exception_summary(const value_t &exc) {
  return class_name(class_of_instance(exc)) + ": " + exception_message(exc);
}

value_t exception_from(const std::exception &e) {
  if (const auto *d = dynamic_cast<const dsl_exception *>(&e))
    return d->value();
  if (const auto *d = dynamic_cast<const dsl_error *>(&e)) {
    dict_ptr cls = exception_class(d->class_name());
    value_t exc = make_exception(cls ? cls : exception_class("RuntimeError"), e.what(), nil_value,
                                 d->origin());
    for (const auto &[key, v] : d->fields())
      set_field(exc, key, v);
    return exc;
  }
  if (const auto *p = dynamic_cast<const parse_error *>(&e)) {
    value_t exc = make_exception(exception_class("ParseError"), e.what(), nil_value, "builtin");
    set_field(exc, "line", value_t(static_cast<int64_t>(p->line())));
    set_field(exc, "col", value_t(static_cast<int64_t>(p->col())));
    return exc;
  }
  if (dynamic_cast<const std::invalid_argument *>(&e))
    return native_exception("ValueError", e);
  if (dynamic_cast<const std::out_of_range *>(&e))
    return native_exception("IndexError", e);
  if (dynamic_cast<const std::bad_alloc *>(&e))
    return native_exception("MemoryError", e);
  if (typeid(e) == typeid(std::runtime_error))
    return native_exception("RuntimeError", e);
  value_t exc = native_exception("NativeError", e);
  set_field(exc, "native_type", value_t(type_name_of(e)));
  return exc;
}

bool is_uncatchable(const std::exception &e) {
  return dynamic_cast<const evaluation_interrupted *>(&e) != nullptr ||
         dynamic_cast<const evaluation_timeout *>(&e) != nullptr;
}

bool same_exception(const value_t &a, const value_t &b) {
  const auto *da = std::get_if<dict_ptr>(&a.v);
  const auto *db = std::get_if<dict_ptr>(&b.v);
  return da && db && *da == *db;
}

bool in_cause_chain(const value_t &handled, const value_t &exc) {
  const value_t *at = &handled;
  for (int hop = 0; at && hop < 4096; ++hop) {
    if (same_exception(*at, exc))
      return true;
    at = field(*at, "cause");
  }
  return false;
}

// -- the try form ---------------------------------------------------------------------------------

namespace {
// The head of a try clause, "catch" or "finally"; "" for any other form.
std::string clause_head(const value_t &form) {
  const auto *lp = std::get_if<list_ptr>(&form.v);
  if (!lp || !*lp || (*lp)->empty())
    return "";
  const auto *head = std::get_if<symbol>(&(**lp)[0].v);
  return head && (head->name == "catch" || head->name == "finally") ? head->name : "";
}

// A catch clause's spec: (Type var), (Type), (var) or ().
const std::vector<value_t> &catch_spec(const value_t &clause) {
  return *std::get<list_ptr>((*std::get<list_ptr>(clause.v))[1].v);
}
} // namespace

try_form parse_try(std::span<const value_t> args) {
  try_form form;
  std::size_t clauses = 0;
  while (clauses < args.size() && clause_head(args[clauses]).empty())
    ++clauses;
  form.body.assign(args.begin(), args.begin() + static_cast<std::ptrdiff_t>(clauses));
  for (std::size_t i = clauses; i < args.size(); ++i) {
    const std::string head = clause_head(args[i]);
    if (head.empty())
      throw dsl_error("TypeError",
                      "try: expected a catch or finally clause, got " + to_string(args[i]));
    if (!form.cleanup.is_nil())
      throw dsl_error("TypeError", "try: finally must be the last clause");
    const auto &clause = *std::get<list_ptr>(args[i].v);
    if (head == "finally") {
      std::vector<value_t> forms{value_t{symbol{"begin"}}}; // a list even when it is empty
      forms.insert(forms.end(), clause.begin() + 1, clause.end());
      form.cleanup = make_list(std::move(forms));
      continue;
    }
    const auto *spec = clause.size() >= 2 ? std::get_if<list_ptr>(&clause[1].v) : nullptr;
    bool ok = spec && *spec && (*spec)->size() <= 2;
    for (std::size_t s = 0; ok && s < (*spec)->size(); ++s)
      ok = std::holds_alternative<symbol>((**spec)[s].v);
    if (!ok)
      throw dsl_error("TypeError",
                      "try: expected (catch (Type var) handler...), got " + to_string(args[i]));
    form.catches.push_back(args[i]);
  }
  if (form.catches.empty() && form.cleanup.is_nil())
    throw dsl_error("TypeError", "try: expected a catch or finally clause");
  return form;
}

dict_ptr class_named(const environment_ptr &env, const std::string &name) {
  if (const value_t *cd = env->lookup(name + ".__cls_dict__"))
    if (const auto *d = std::get_if<dict_ptr>(&cd->v); d && *d)
      return *d;
  if (const value_t *ctor = env->lookup(name))
    return class_of_constructor(*ctor);
  return nullptr;
}

bool catch_matches(const environment_ptr &env, const value_t &clause, const value_t &exc,
                   std::string &var) {
  const auto &spec = catch_spec(clause);
  dict_ptr cls;
  if (spec.size() == 2) {
    const std::string &type = std::get<symbol>(spec[0].v).name;
    cls = class_named(env, type);
    if (!cls)
      throw dsl_error("NameError", "catch: not a class: " + type);
    var = std::get<symbol>(spec[1].v).name;
  } else if (spec.size() == 1) {
    const std::string &name = std::get<symbol>(spec[0].v).name;
    cls = class_named(env, name);
    if (!cls)
      var = name;
  }
  return !cls || is_instance(exc, cls);
}

std::span<const value_t> catch_handler(const value_t &clause) {
  return std::span<const value_t>(*std::get<list_ptr>(clause.v)).subspan(2);
}

// -- tracebacks -----------------------------------------------------------------------------------

namespace {
constexpr std::size_t kTracebackLine = 120; // characters per traceback line

// `v` printed into `out`, eliding the elements of a list past `limit` characters.
void brief_into(std::string &out, const value_t &v, std::size_t limit) {
  const auto *lp = std::get_if<list_ptr>(&v.v);
  if (!lp || !*lp) {
    out += to_string(v);
    return;
  }
  out += '(';
  for (std::size_t i = 0; i < (*lp)->size(); ++i) {
    if (out.size() >= limit) {
      out += "...";
      break;
    }
    if (i)
      out += ' ';
    brief_into(out, (**lp)[i], limit);
  }
  out += ')';
}
} // namespace

std::string traceback_line(const value_t &expr) {
  std::string line;
  brief_into(line, expr, kTracebackLine);
  if (line.size() > kTracebackLine) {
    std::size_t keep = kTracebackLine - 3;
    while (keep > 0 && (static_cast<unsigned char>(line[keep]) & 0xC0) == 0x80)
      --keep; // not inside a UTF-8 sequence
    line.resize(keep);
    line += "...";
  }
  return line;
}

} // namespace cvc::state_exec
