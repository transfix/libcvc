#include <cvc/state_exec/builtins.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

namespace {

/// Extract a numeric value as double, supporting int64 and double.
double as_double(const value_t& v) {
    if (auto* i = std::get_if<int64_t>(&v.v)) return static_cast<double>(*i);
    if (auto* d = std::get_if<double>(&v.v))  return *d;
    throw std::runtime_error("expected number, got " + v.type_name());
}

/// True when the value is an integer.
bool is_int(const value_t& v) {
    return std::holds_alternative<int64_t>(v.v);
}

/// True when every arg in [first, last) is integral (recurses into lists).
bool all_int(std::span<const value_t> args) {
    for (auto& a : args) {
        if (auto* lp = std::get_if<list_ptr>(&a.v)) {
            for (auto& item : **lp)
                if (!is_int(item)) return false;
        } else if (!is_int(a)) {
            return false;
        }
    }
    return true;
}

void expect_min(std::span<const value_t> args, size_t n, const char* name) {
    if (args.size() < n)
        throw std::runtime_error(
            std::string(name) + ": expected at least " +
            std::to_string(n) + " argument(s), got " +
            std::to_string(args.size()));
}

void expect_exact(std::span<const value_t> args, size_t n, const char* name) {
    if (args.size() != n)
        throw std::runtime_error(
            std::string(name) + ": expected " + std::to_string(n) +
            " argument(s), got " + std::to_string(args.size()));
}

const std::vector<value_t>& as_list(const value_t& v, const char* name) {
    if (auto* p = std::get_if<list_ptr>(&v.v))
        return **p;
    throw std::runtime_error(
        std::string(name) + ": expected list, got " + v.type_name());
}

std::vector<value_t>& as_list_mut(const value_t& v, const char* name) {
    if (auto* p = std::get_if<list_ptr>(&v.v))
        return **p;
    throw std::runtime_error(
        std::string(name) + ": expected list, got " + v.type_name());
}

using dict_type = std::vector<std::pair<std::string, value_t>>;

dict_type& as_dict_mut(const value_t& v, const char* name) {
    if (auto* p = std::get_if<dict_ptr>(&v.v))
        return **p;
    throw std::runtime_error(
        std::string(name) + ": expected dict, got " + v.type_name());
}

const dict_type& as_dict(const value_t& v, const char* name) {
    return as_dict_mut(v, name);
}

const std::string& as_string(const value_t& v, const char* name) {
    if (auto* s = std::get_if<std::string>(&v.v))
        return *s;
    throw std::runtime_error(
        std::string(name) + ": expected string, got " + v.type_name());
}

int64_t as_int(const value_t& v, const char* name) {
    if (auto* i = std::get_if<int64_t>(&v.v))
        return *i;
    throw std::runtime_error(
        std::string(name) + ": expected integer, got " + v.type_name());
}

/// Find an entry in a dict by key; returns nullptr if not found.
value_t* dict_find(dict_type& d, const std::string& key) {
    for (auto& [k, v] : d)
        if (k == key) return &v;
    return nullptr;
}

/// Walk the __class__ / __super__ chain to find a method.
value_t dispatch_method(const value_t& obj, const std::string& method,
                        std::span<const value_t> args, bool super) {
    auto& obj_dict = as_dict_mut(obj, "send");
    // find __class__
    value_t* cls_val = dict_find(obj_dict, "__class__");
    if (!cls_val)
        throw std::runtime_error("send: object has no __class__");

    auto* cls_dict = &as_dict_mut(*cls_val, "send");

    if (super) {
        // skip to __super__
        value_t* sp = dict_find(*cls_dict, "__super__");
        if (!sp) throw std::runtime_error("send: no super class");
        cls_dict = &as_dict_mut(*sp, "send");
    }

    while (cls_dict) {
        if (value_t* m = dict_find(*cls_dict, method)) {
            // Invoke: first arg is self
            if (auto* fn = std::get_if<native_fn>(&m->v)) {
                std::vector<value_t> call_args;
                call_args.reserve(1 + args.size());
                call_args.push_back(obj);
                call_args.insert(call_args.end(), args.begin(), args.end());
                return (*fn)(call_args);
            }
            if (auto* c = std::get_if<closure_ptr>(&m->v))
                throw std::runtime_error(
                    "send: closure method dispatch requires evaluator "
                    "(use evaluator-level send)");
            throw std::runtime_error("send: method is not callable");
        }
        // walk __super__
        value_t* sp = dict_find(*cls_dict, "__super__");
        if (!sp) break;
        cls_dict = &as_dict_mut(*sp, "send");
    }
    throw std::runtime_error("send: method '" + method + "' not found");
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Arithmetic
// ---------------------------------------------------------------------------

value_t builtin_add(std::span<const value_t> args) {
    if (args.empty()) return value_t{int64_t{0}};
    if (all_int(args)) {
        int64_t r = 0;
        for (auto& a : args) {
            // flatten lists
            if (auto* lp = std::get_if<list_ptr>(&a.v)) {
                for (auto& item : **lp)
                    r += std::get<int64_t>(item.v);
            } else {
                r += std::get<int64_t>(a.v);
            }
        }
        return value_t{r};
    }
    double r = 0.0;
    for (auto& a : args) {
        if (auto* lp = std::get_if<list_ptr>(&a.v)) {
            for (auto& item : **lp)
                r += as_double(item);
        } else {
            r += as_double(a);
        }
    }
    return value_t{r};
}

value_t builtin_sub(std::span<const value_t> args) {
    if (args.empty()) return value_t{int64_t{0}};
    if (args.size() == 1) {
        if (is_int(args[0])) return value_t{-std::get<int64_t>(args[0].v)};
        return value_t{-as_double(args[0])};
    }
    if (all_int(args)) {
        int64_t r = std::get<int64_t>(args[0].v);
        for (size_t i = 1; i < args.size(); ++i)
            r -= std::get<int64_t>(args[i].v);
        return value_t{r};
    }
    double r = as_double(args[0]);
    for (size_t i = 1; i < args.size(); ++i)
        r -= as_double(args[i]);
    return value_t{r};
}

value_t builtin_mul(std::span<const value_t> args) {
    if (args.empty()) return value_t{int64_t{1}};
    if (all_int(args)) {
        int64_t r = 1;
        for (auto& a : args) {
            if (auto* lp = std::get_if<list_ptr>(&a.v)) {
                for (auto& item : **lp)
                    r *= std::get<int64_t>(item.v);
            } else {
                r *= std::get<int64_t>(a.v);
            }
        }
        return value_t{r};
    }
    double r = 1.0;
    for (auto& a : args) {
        if (auto* lp = std::get_if<list_ptr>(&a.v)) {
            for (auto& item : **lp)
                r *= as_double(item);
        } else {
            r *= as_double(a);
        }
    }
    return value_t{r};
}

value_t builtin_div(std::span<const value_t> args) {
    if (args.empty()) return value_t{int64_t{1}};
    if (args.size() == 1) return value_t{1.0 / as_double(args[0])};
    // Division always produces double (like Python3)
    double r = as_double(args[0]);
    for (size_t i = 1; i < args.size(); ++i) {
        double d = as_double(args[i]);
        if (d == 0.0) throw std::runtime_error("/: division by zero");
        r /= d;
    }
    return value_t{r};
}

value_t builtin_mod(std::span<const value_t> args) {
    expect_exact(args, 2, "%");
    if (all_int(args)) {
        int64_t b = std::get<int64_t>(args[1].v);
        if (b == 0) throw std::runtime_error("%: division by zero");
        return value_t{std::get<int64_t>(args[0].v) % b};
    }
    double b = as_double(args[1]);
    if (b == 0.0) throw std::runtime_error("%: division by zero");
    return value_t{std::fmod(as_double(args[0]), b)};
}

// ---------------------------------------------------------------------------
// Comparison
// ---------------------------------------------------------------------------

value_t builtin_lt(std::span<const value_t> args) {
    expect_exact(args, 2, "<");
    return value_t{as_double(args[0]) < as_double(args[1])};
}

value_t builtin_gt(std::span<const value_t> args) {
    expect_exact(args, 2, ">");
    return value_t{as_double(args[0]) > as_double(args[1])};
}

value_t builtin_le(std::span<const value_t> args) {
    expect_exact(args, 2, "<=");
    return value_t{as_double(args[0]) <= as_double(args[1])};
}

value_t builtin_ge(std::span<const value_t> args) {
    expect_exact(args, 2, ">=");
    return value_t{as_double(args[0]) >= as_double(args[1])};
}

value_t builtin_eq(std::span<const value_t> args) {
    expect_exact(args, 2, "=");
    return value_t{values_equal(args[0], args[1])};
}

value_t builtin_ne(std::span<const value_t> args) {
    expect_exact(args, 2, "!=");
    return value_t{!values_equal(args[0], args[1])};
}

// ---------------------------------------------------------------------------
// String
// ---------------------------------------------------------------------------

value_t builtin_str_concat(std::span<const value_t> args) {
    std::string result;
    for (auto& a : args) {
        if (auto* s = std::get_if<std::string>(&a.v))
            result += *s;  // raw string, no quoting
        else
            result += to_string(a);
    }
    return value_t{std::move(result)};
}

value_t builtin_str(std::span<const value_t> args) {
    expect_exact(args, 1, "str");
    return value_t{to_string(args[0])};
}

// ---------------------------------------------------------------------------
// List
// ---------------------------------------------------------------------------

value_t builtin_list(std::span<const value_t> args) {
    return make_list(std::vector<value_t>(args.begin(), args.end()));
}

value_t builtin_car(std::span<const value_t> args) {
    expect_exact(args, 1, "car");
    auto& lst = as_list(args[0], "car");
    if (lst.empty())
        throw std::runtime_error("car: empty list");
    return lst.front();
}

value_t builtin_cdr(std::span<const value_t> args) {
    expect_exact(args, 1, "cdr");
    auto& lst = as_list(args[0], "cdr");
    if (lst.empty()) return make_list();
    return make_list(std::vector<value_t>(lst.begin() + 1, lst.end()));
}

value_t builtin_cons(std::span<const value_t> args) {
    expect_exact(args, 2, "cons");
    auto& lst = as_list(args[1], "cons");
    std::vector<value_t> result;
    result.reserve(1 + lst.size());
    result.push_back(args[0]);
    result.insert(result.end(), lst.begin(), lst.end());
    return make_list(std::move(result));
}

value_t builtin_nth(std::span<const value_t> args) {
    expect_exact(args, 2, "nth");
    auto& lst = as_list(args[0], "nth");
    int64_t idx = as_int(args[1], "nth");
    if (idx < 0 || static_cast<size_t>(idx) >= lst.size())
        throw std::runtime_error("nth: index out of range");
    return lst[static_cast<size_t>(idx)];
}

value_t builtin_set_nth(std::span<const value_t> args) {
    expect_exact(args, 3, "set-nth");
    auto& lst = as_list_mut(args[0], "set-nth");
    int64_t idx = as_int(args[1], "set-nth");
    if (idx < 0 || static_cast<size_t>(idx) >= lst.size())
        throw std::runtime_error("set-nth: index out of range");
    lst[static_cast<size_t>(idx)] = args[2];
    return args[2];
}

value_t builtin_length(std::span<const value_t> args) {
    expect_exact(args, 1, "length");
    if (auto* lp = std::get_if<list_ptr>(&args[0].v))
        return value_t{static_cast<int64_t>((*lp)->size())};
    if (auto* s = std::get_if<std::string>(&args[0].v))
        return value_t{static_cast<int64_t>(s->size())};
    if (auto* dp = std::get_if<dict_ptr>(&args[0].v))
        return value_t{static_cast<int64_t>((*dp)->size())};
    throw std::runtime_error("length: expected list, string, or dict");
}

value_t builtin_append(std::span<const value_t> args) {
    expect_exact(args, 2, "append");
    auto& lst = as_list_mut(args[0], "append");
    lst.push_back(args[1]);
    return args[0];
}

value_t builtin_slice(std::span<const value_t> args) {
    if (args.size() < 2 || args.size() > 3)
        throw std::runtime_error("slice: expected 2 or 3 arguments");
    auto& lst = as_list(args[0], "slice");
    int64_t start = as_int(args[1], "slice");
    int64_t end = (args.size() == 3)
        ? as_int(args[2], "slice")
        : static_cast<int64_t>(lst.size());
    if (start < 0) start = 0;
    if (end < 0) end = 0;
    if (static_cast<size_t>(start) > lst.size())
        start = static_cast<int64_t>(lst.size());
    if (static_cast<size_t>(end) > lst.size())
        end = static_cast<int64_t>(lst.size());
    if (start >= end) return make_list();
    return make_list(std::vector<value_t>(
        lst.begin() + start, lst.begin() + end));
}

value_t builtin_del_nth(std::span<const value_t> args) {
    expect_exact(args, 2, "del-nth");
    auto& lst = as_list_mut(args[0], "del-nth");
    int64_t idx = as_int(args[1], "del-nth");
    if (idx < 0 || static_cast<size_t>(idx) >= lst.size())
        throw std::runtime_error("del-nth: index out of range");
    value_t removed = lst[static_cast<size_t>(idx)];
    lst.erase(lst.begin() + idx);
    return removed;
}

// ---------------------------------------------------------------------------
// Dict
// ---------------------------------------------------------------------------

value_t builtin_dict(std::span<const value_t> args) {
    if (args.size() % 2 != 0)
        throw std::runtime_error("dict: expected even number of arguments");
    std::vector<std::pair<std::string, value_t>> entries;
    entries.reserve(args.size() / 2);
    for (size_t i = 0; i < args.size(); i += 2) {
        std::string key;
        if (auto* s = std::get_if<std::string>(&args[i].v))
            key = *s;
        else
            key = to_string(args[i]);
        entries.emplace_back(std::move(key), args[i + 1]);
    }
    return make_dict(std::move(entries));
}

value_t builtin_get_attr(std::span<const value_t> args) {
    expect_exact(args, 2, "get-attr");
    auto& d = as_dict(args[0], "get-attr");
    const std::string& key = as_string(args[1], "get-attr");
    for (auto& [k, v] : d)
        if (k == key) return v;
    throw std::runtime_error("get-attr: key '" + key + "' not found");
}

value_t builtin_set_attr(std::span<const value_t> args) {
    expect_exact(args, 3, "set-attr");
    auto& d = as_dict_mut(args[0], "set-attr");
    const std::string& key = as_string(args[1], "set-attr");
    for (auto& [k, v] : d) {
        if (k == key) { v = args[2]; return args[2]; }
    }
    d.emplace_back(key, args[2]);
    return args[2];
}

value_t builtin_del_attr(std::span<const value_t> args) {
    expect_exact(args, 2, "del-attr");
    auto& d = as_dict_mut(args[0], "del-attr");
    const std::string& key = as_string(args[1], "del-attr");
    for (auto it = d.begin(); it != d.end(); ++it) {
        if (it->first == key) {
            value_t removed = it->second;
            d.erase(it);
            return removed;
        }
    }
    throw std::runtime_error("del-attr: key '" + key + "' not found");
}

// ---------------------------------------------------------------------------
// OOP
// ---------------------------------------------------------------------------

value_t builtin_send(std::span<const value_t> args) {
    expect_min(args, 2, "send");
    const std::string& method = as_string(args[1], "send");
    auto method_args = args.subspan(2);
    return dispatch_method(args[0], method, method_args, /*super=*/false);
}

// ---------------------------------------------------------------------------
// Higher-order
// ---------------------------------------------------------------------------

value_t builtin_apply(std::span<const value_t> args) {
    expect_exact(args, 2, "apply");
    auto* fn = std::get_if<native_fn>(&args[0].v);
    if (!fn)
        throw std::runtime_error(
            "apply: first argument must be callable (native_fn); "
            "closure dispatch requires evaluator");
    auto& arg_list = as_list(args[1], "apply");
    return (*fn)(arg_list);
}

// ---------------------------------------------------------------------------
// Type predicates
// ---------------------------------------------------------------------------

value_t builtin_is_null(std::span<const value_t> args) {
    expect_exact(args, 1, "is-null");
    return value_t{args[0].is_nil()};
}

value_t builtin_is_list(std::span<const value_t> args) {
    expect_exact(args, 1, "is-list");
    return value_t{std::holds_alternative<list_ptr>(args[0].v)};
}

value_t builtin_type_of(std::span<const value_t> args) {
    expect_exact(args, 1, "type-of");
    return value_t{args[0].type_name()};
}

// ---------------------------------------------------------------------------
// I/O
// ---------------------------------------------------------------------------

value_t builtin_print(std::span<const value_t> args) {
    for (size_t i = 0; i < args.size(); ++i) {
        if (i > 0) std::cout << ' ';
        std::cout << to_string(args[i]);
    }
    std::cout << '\n';
    return nil_value;
}

// ---------------------------------------------------------------------------
// Logic
// ---------------------------------------------------------------------------

value_t builtin_not(std::span<const value_t> args) {
    expect_exact(args, 1, "not");
    return value_t{!args[0].is_truthy()};
}

value_t builtin_and(std::span<const value_t> args) {
    for (auto& a : args)
        if (!a.is_truthy()) return false_value;
    return args.empty() ? true_value : args.back();
}

value_t builtin_or(std::span<const value_t> args) {
    for (auto& a : args)
        if (a.is_truthy()) return a;
    return false_value;
}

// ---------------------------------------------------------------------------
// Registry
// ---------------------------------------------------------------------------

void builtins::register_fn(environment_ptr env,
                           const std::string& name,
                           native_fn fn) {
    env->set(name, value_t{std::move(fn)});
}

environment_ptr builtins::make_default_environment() {
    auto env = std::make_shared<environment>();

    // Arithmetic
    register_fn(env, "+",          builtin_add);
    register_fn(env, "-",          builtin_sub);
    register_fn(env, "*",          builtin_mul);
    register_fn(env, "/",          builtin_div);
    register_fn(env, "%",          builtin_mod);

    // Comparison
    register_fn(env, "<",          builtin_lt);
    register_fn(env, ">",          builtin_gt);
    register_fn(env, "<=",         builtin_le);
    register_fn(env, ">=",         builtin_ge);
    register_fn(env, "=",          builtin_eq);
    register_fn(env, "!=",         builtin_ne);

    // String
    register_fn(env, "str-concat", builtin_str_concat);
    register_fn(env, "str",        builtin_str);

    // List
    register_fn(env, "list",       builtin_list);
    register_fn(env, "car",        builtin_car);
    register_fn(env, "cdr",        builtin_cdr);
    register_fn(env, "cons",       builtin_cons);
    register_fn(env, "nth",        builtin_nth);
    register_fn(env, "set-nth",    builtin_set_nth);
    register_fn(env, "length",     builtin_length);
    register_fn(env, "append",     builtin_append);
    register_fn(env, "slice",      builtin_slice);
    register_fn(env, "del-nth",    builtin_del_nth);

    // Dict
    register_fn(env, "dict",       builtin_dict);
    register_fn(env, "get-attr",   builtin_get_attr);
    register_fn(env, "set-attr",   builtin_set_attr);
    register_fn(env, "del-attr",   builtin_del_attr);

    // OOP
    register_fn(env, "send",       builtin_send);

    // Higher-order
    register_fn(env, "apply",      builtin_apply);

    // Type predicates
    register_fn(env, "is-null",    builtin_is_null);
    register_fn(env, "is-list",    builtin_is_list);
    register_fn(env, "type-of",    builtin_type_of);

    // I/O
    register_fn(env, "print",      builtin_print);

    // Logic
    register_fn(env, "not",        builtin_not);
    register_fn(env, "and",        builtin_and);
    register_fn(env, "or",         builtin_or);

    return env;
}

} // namespace cvc::state_exec
