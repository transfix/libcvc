#include <cvc/state_exec/stdlib.h>
#include <cvc/state_exec/builtins.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>

namespace cvc::state_exec {

namespace {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

void expect_exact(std::span<const value_t> args, size_t n, const char* name) {
    if (args.size() != n)
        throw std::runtime_error(
            std::string(name) + ": expected " + std::to_string(n) +
            " argument(s), got " + std::to_string(args.size()));
}

void expect_min(std::span<const value_t> args, size_t n, const char* name) {
    if (args.size() < n)
        throw std::runtime_error(
            std::string(name) + ": expected at least " +
            std::to_string(n) + " argument(s), got " +
            std::to_string(args.size()));
}

const std::string& as_string(const value_t& v, const char* name) {
    if (auto* s = std::get_if<std::string>(&v.v)) return *s;
    throw std::runtime_error(
        std::string(name) + ": expected string, got " + v.type_name());
}

int64_t as_int(const value_t& v, const char* name) {
    if (auto* i = std::get_if<int64_t>(&v.v)) return *i;
    throw std::runtime_error(
        std::string(name) + ": expected integer, got " + v.type_name());
}

double as_number(const value_t& v, const char* name) {
    if (auto* i = std::get_if<int64_t>(&v.v)) return static_cast<double>(*i);
    if (auto* d = std::get_if<double>(&v.v))  return *d;
    throw std::runtime_error(
        std::string(name) + ": expected number, got " + v.type_name());
}

const std::vector<value_t>& as_list(const value_t& v, const char* name) {
    if (auto* p = std::get_if<list_ptr>(&v.v)) return **p;
    throw std::runtime_error(
        std::string(name) + ": expected list, got " + v.type_name());
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// String module
// ---------------------------------------------------------------------------

void stdlib_registry::register_string_module() {
    register_function("string", "string.split", [](std::span<const value_t> args) -> value_t {
        expect_min(args, 1, "string.split");
        auto& s = as_string(args[0], "string.split");
        std::string delim = " ";
        if (args.size() > 1) delim = as_string(args[1], "string.split");
        std::vector<value_t> parts;
        if (delim.empty()) {
            for (char c : s) parts.emplace_back(std::string(1, c));
        } else {
            size_t start = 0;
            while (start <= s.size()) {
                auto pos = s.find(delim, start);
                if (pos == std::string::npos) {
                    parts.emplace_back(s.substr(start));
                    break;
                }
                parts.emplace_back(s.substr(start, pos - start));
                start = pos + delim.size();
            }
        }
        return make_list(std::move(parts));
    });

    register_function("string", "string.join", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "string.join");
        auto& lst = as_list(args[0], "string.join");
        auto& sep = as_string(args[1], "string.join");
        std::string result;
        for (size_t i = 0; i < lst.size(); ++i) {
            if (i > 0) result += sep;
            result += as_string(lst[i], "string.join");
        }
        return value_t(std::move(result));
    });

    register_function("string", "string.replace", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 3, "string.replace");
        auto s   = as_string(args[0], "string.replace");
        auto& from = as_string(args[1], "string.replace");
        auto& to   = as_string(args[2], "string.replace");
        if (from.empty()) return value_t(std::move(s));
        size_t pos = 0;
        while ((pos = s.find(from, pos)) != std::string::npos) {
            s.replace(pos, from.size(), to);
            pos += to.size();
        }
        return value_t(std::move(s));
    });

    register_function("string", "string.trim", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "string.trim");
        auto s = as_string(args[0], "string.trim");
        auto start = s.find_first_not_of(" \t\n\r");
        if (start == std::string::npos) return value_t(std::string());
        auto end = s.find_last_not_of(" \t\n\r");
        return value_t(s.substr(start, end - start + 1));
    });

    register_function("string", "string.upper", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "string.upper");
        auto s = as_string(args[0], "string.upper");
        std::transform(s.begin(), s.end(), s.begin(), ::toupper);
        return value_t(std::move(s));
    });

    register_function("string", "string.lower", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "string.lower");
        auto s = as_string(args[0], "string.lower");
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        return value_t(std::move(s));
    });

    register_function("string", "string.starts-with", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "string.starts-with");
        auto& s   = as_string(args[0], "string.starts-with");
        auto& pfx = as_string(args[1], "string.starts-with");
        return value_t(s.size() >= pfx.size() && s.compare(0, pfx.size(), pfx) == 0);
    });

    register_function("string", "string.ends-with", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "string.ends-with");
        auto& s   = as_string(args[0], "string.ends-with");
        auto& sfx = as_string(args[1], "string.ends-with");
        return value_t(s.size() >= sfx.size() &&
                       s.compare(s.size() - sfx.size(), sfx.size(), sfx) == 0);
    });

    register_function("string", "string.contains", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "string.contains");
        auto& s   = as_string(args[0], "string.contains");
        auto& sub = as_string(args[1], "string.contains");
        return value_t(s.find(sub) != std::string::npos);
    });

    register_function("string", "string.substring", [](std::span<const value_t> args) -> value_t {
        expect_min(args, 2, "string.substring");
        auto& s     = as_string(args[0], "string.substring");
        int64_t pos = as_int(args[1], "string.substring");
        if (pos < 0) pos = 0;
        if (static_cast<size_t>(pos) >= s.size()) return value_t(std::string());
        if (args.size() > 2) {
            int64_t len = as_int(args[2], "string.substring");
            if (len < 0) len = 0;
            return value_t(s.substr(static_cast<size_t>(pos), static_cast<size_t>(len)));
        }
        return value_t(s.substr(static_cast<size_t>(pos)));
    });

    register_function("string", "string.char-at", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "string.char-at");
        auto& s   = as_string(args[0], "string.char-at");
        int64_t i = as_int(args[1], "string.char-at");
        if (i < 0 || static_cast<size_t>(i) >= s.size())
            throw std::runtime_error("string.char-at: index out of range");
        return value_t(std::string(1, s[static_cast<size_t>(i)]));
    });

    register_function("string", "string.length", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "string.length");
        auto& s = as_string(args[0], "string.length");
        return value_t(static_cast<int64_t>(s.size()));
    });
}

// ---------------------------------------------------------------------------
// Math module
// ---------------------------------------------------------------------------

void stdlib_registry::register_math_module() {
    register_function("math", "math.sqrt", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "math.sqrt");
        return value_t(std::sqrt(as_number(args[0], "math.sqrt")));
    });

    register_function("math", "math.abs", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "math.abs");
        if (auto* i = std::get_if<int64_t>(&args[0].v))
            return value_t(std::abs(*i));
        return value_t(std::abs(as_number(args[0], "math.abs")));
    });

    register_function("math", "math.floor", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "math.floor");
        return value_t(static_cast<int64_t>(std::floor(as_number(args[0], "math.floor"))));
    });

    register_function("math", "math.ceil", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "math.ceil");
        return value_t(static_cast<int64_t>(std::ceil(as_number(args[0], "math.ceil"))));
    });

    register_function("math", "math.round", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "math.round");
        return value_t(static_cast<int64_t>(std::round(as_number(args[0], "math.round"))));
    });

    register_function("math", "math.pow", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "math.pow");
        return value_t(std::pow(as_number(args[0], "math.pow"),
                                as_number(args[1], "math.pow")));
    });

    register_function("math", "math.log", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "math.log");
        return value_t(std::log(as_number(args[0], "math.log")));
    });

    register_function("math", "math.sin", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "math.sin");
        return value_t(std::sin(as_number(args[0], "math.sin")));
    });

    register_function("math", "math.cos", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "math.cos");
        return value_t(std::cos(as_number(args[0], "math.cos")));
    });

    register_function("math", "math.tan", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "math.tan");
        return value_t(std::tan(as_number(args[0], "math.tan")));
    });

    register_function("math", "math.min", [](std::span<const value_t> args) -> value_t {
        expect_min(args, 2, "math.min");
        double result = as_number(args[0], "math.min");
        for (size_t i = 1; i < args.size(); ++i)
            result = std::min(result, as_number(args[i], "math.min"));
        return value_t(result);
    });

    register_function("math", "math.max", [](std::span<const value_t> args) -> value_t {
        expect_min(args, 2, "math.max");
        double result = as_number(args[0], "math.max");
        for (size_t i = 1; i < args.size(); ++i)
            result = std::max(result, as_number(args[i], "math.max"));
        return value_t(result);
    });

    register_function("math", "math.clamp", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 3, "math.clamp");
        double val = as_number(args[0], "math.clamp");
        double lo  = as_number(args[1], "math.clamp");
        double hi  = as_number(args[2], "math.clamp");
        return value_t(std::clamp(val, lo, hi));
    });

    register_function("math", "math.pi", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 0, "math.pi");
        return value_t(3.14159265358979323846);
    });

    register_function("math", "math.e", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 0, "math.e");
        return value_t(2.71828182845904523536);
    });

    register_function("math", "math.random", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 0, "math.random");
        static thread_local std::mt19937 gen{std::random_device{}()};
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        return value_t(dist(gen));
    });
}

// ---------------------------------------------------------------------------
// Collections module
// ---------------------------------------------------------------------------

void stdlib_registry::register_collections_module() {
    register_function("collections", "collections.map", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "collections.map");
        auto& lst = as_list(args[0], "collections.map");
        auto* fn = std::get_if<native_fn>(&args[1].v);
        if (!fn)
            throw std::runtime_error("collections.map: second argument must be a function");
        std::vector<value_t> result;
        result.reserve(lst.size());
        for (auto& item : lst) {
            result.push_back((*fn)(std::span<const value_t>(&item, 1)));
        }
        return make_list(std::move(result));
    });

    register_function("collections", "collections.filter", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "collections.filter");
        auto& lst = as_list(args[0], "collections.filter");
        auto* fn = std::get_if<native_fn>(&args[1].v);
        if (!fn)
            throw std::runtime_error("collections.filter: second argument must be a function");
        std::vector<value_t> result;
        for (auto& item : lst) {
            auto v = (*fn)(std::span<const value_t>(&item, 1));
            if (v.is_truthy()) result.push_back(item);
        }
        return make_list(std::move(result));
    });

    register_function("collections", "collections.reduce", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 3, "collections.reduce");
        auto& lst = as_list(args[0], "collections.reduce");
        auto* fn = std::get_if<native_fn>(&args[1].v);
        if (!fn)
            throw std::runtime_error("collections.reduce: second argument must be a function");
        value_t acc = args[2];
        for (auto& item : lst) {
            value_t pair[2] = {acc, item};
            acc = (*fn)(std::span<const value_t>(pair, 2));
        }
        return acc;
    });

    register_function("collections", "collections.zip", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "collections.zip");
        auto& a = as_list(args[0], "collections.zip");
        auto& b = as_list(args[1], "collections.zip");
        size_t n = std::min(a.size(), b.size());
        std::vector<value_t> result;
        result.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            result.push_back(make_list({a[i], b[i]}));
        }
        return make_list(std::move(result));
    });

    register_function("collections", "collections.flatten", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "collections.flatten");
        auto& lst = as_list(args[0], "collections.flatten");
        std::vector<value_t> result;
        for (auto& item : lst) {
            if (auto* sub = std::get_if<list_ptr>(&item.v)) {
                for (auto& x : **sub) result.push_back(x);
            } else {
                result.push_back(item);
            }
        }
        return make_list(std::move(result));
    });

    register_function("collections", "collections.sort", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "collections.sort");
        auto elems = as_list(args[0], "collections.sort"); // copy
        std::sort(elems.begin(), elems.end(), [](const value_t& a, const value_t& b) {
            // Sort by numeric value or string comparison
            if (auto* ai = std::get_if<int64_t>(&a.v)) {
                if (auto* bi = std::get_if<int64_t>(&b.v)) return *ai < *bi;
                if (auto* bd = std::get_if<double>(&b.v)) return static_cast<double>(*ai) < *bd;
            }
            if (auto* ad = std::get_if<double>(&a.v)) {
                if (auto* bi = std::get_if<int64_t>(&b.v)) return *ad < static_cast<double>(*bi);
                if (auto* bd = std::get_if<double>(&b.v)) return *ad < *bd;
            }
            if (auto* as = std::get_if<std::string>(&a.v)) {
                if (auto* bs = std::get_if<std::string>(&b.v)) return *as < *bs;
            }
            return false;
        });
        return make_list(std::move(elems));
    });

    register_function("collections", "collections.reverse", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "collections.reverse");
        auto elems = as_list(args[0], "collections.reverse"); // copy
        std::reverse(elems.begin(), elems.end());
        return make_list(std::move(elems));
    });

    register_function("collections", "collections.range", [](std::span<const value_t> args) -> value_t {
        expect_min(args, 1, "collections.range");
        int64_t start = 0, stop, step = 1;
        if (args.size() == 1) {
            stop = as_int(args[0], "collections.range");
        } else {
            start = as_int(args[0], "collections.range");
            stop  = as_int(args[1], "collections.range");
            if (args.size() > 2)
                step = as_int(args[2], "collections.range");
        }
        if (step == 0)
            throw std::runtime_error("collections.range: step cannot be zero");
        std::vector<value_t> result;
        if (step > 0) {
            for (int64_t i = start; i < stop; i += step)
                result.emplace_back(i);
        } else {
            for (int64_t i = start; i > stop; i += step)
                result.emplace_back(i);
        }
        return make_list(std::move(result));
    });

    register_function("collections", "collections.unique", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "collections.unique");
        auto& lst = as_list(args[0], "collections.unique");
        std::vector<value_t> result;
        for (auto& item : lst) {
            bool found = false;
            for (auto& r : result) {
                if (values_equal(r, item)) { found = true; break; }
            }
            if (!found) result.push_back(item);
        }
        return make_list(std::move(result));
    });

    register_function("collections", "collections.dict-keys", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "collections.dict-keys");
        auto* d = std::get_if<dict_ptr>(&args[0].v);
        if (!d)
            throw std::runtime_error("collections.dict-keys: expected dict");
        std::vector<value_t> keys;
        keys.reserve((**d).size());
        for (auto& [k, _] : **d) keys.emplace_back(k);
        return make_list(std::move(keys));
    });

    register_function("collections", "collections.dict-values", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 1, "collections.dict-values");
        auto* d = std::get_if<dict_ptr>(&args[0].v);
        if (!d)
            throw std::runtime_error("collections.dict-values: expected dict");
        std::vector<value_t> vals;
        vals.reserve((**d).size());
        for (auto& [_, v] : **d) vals.push_back(v);
        return make_list(std::move(vals));
    });

    register_function("collections", "collections.dict-merge", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "collections.dict-merge");
        auto* d1 = std::get_if<dict_ptr>(&args[0].v);
        auto* d2 = std::get_if<dict_ptr>(&args[1].v);
        if (!d1 || !d2)
            throw std::runtime_error("collections.dict-merge: expected two dicts");
        // Start with copy of d1
        auto result = std::make_shared<std::vector<std::pair<std::string, value_t>>>(**d1);
        // Merge d2: update existing keys, append new ones
        for (auto& [k, v] : **d2) {
            bool found = false;
            for (auto& [rk, rv] : *result) {
                if (rk == k) { rv = v; found = true; break; }
            }
            if (!found) result->emplace_back(k, v);
        }
        return value_t(std::move(result));
    });
}

// ---------------------------------------------------------------------------
// stdlib_registry public interface
// ---------------------------------------------------------------------------

stdlib_registry::stdlib_registry() {
    register_string_module();
    register_math_module();
    register_collections_module();
}

void stdlib_registry::register_function(const std::string& module,
                                        const std::string& func_name,
                                        native_fn fn) {
    modules_[module].functions[func_name] = std::move(fn);
}

std::vector<std::string> stdlib_registry::list_functions(
    const std::string& module) const {
    auto it = modules_.find(module);
    if (it == modules_.end()) return {};
    std::vector<std::string> names;
    names.reserve(it->second.functions.size());
    for (auto& [k, _] : it->second.functions) names.push_back(k);
    std::sort(names.begin(), names.end());
    return names;
}

std::vector<std::string> stdlib_registry::list_modules() const {
    std::vector<std::string> names;
    names.reserve(modules_.size());
    for (auto& [k, _] : modules_) names.push_back(k);
    std::sort(names.begin(), names.end());
    return names;
}

void stdlib_registry::import_module(const std::string& module,
                                    environment_ptr env,
                                    const std::vector<std::string>& specific_fns) const {
    auto it = modules_.find(module);
    if (it == modules_.end())
        throw std::runtime_error("stdlib: unknown module '" + module + "'");

    if (specific_fns.empty()) {
        // Import all functions
        for (auto& [name, fn] : it->second.functions) {
            builtins::register_fn(env, name, fn);
        }
    } else {
        for (auto& name : specific_fns) {
            auto fit = it->second.functions.find(name);
            if (fit == it->second.functions.end())
                throw std::runtime_error(
                    "stdlib: module '" + module + "' has no function '" + name + "'");
            builtins::register_fn(env, name, fit->second);
        }
    }
}

const native_fn* stdlib_registry::lookup_qualified(
    const std::string& qualified_name) const {
    // Try "module.function" format
    auto dot = qualified_name.find('.');
    if (dot != std::string::npos) {
        auto mod = qualified_name.substr(0, dot);
        auto it = modules_.find(mod);
        if (it != modules_.end()) {
            auto fit = it->second.functions.find(qualified_name);
            if (fit != it->second.functions.end())
                return &fit->second;
        }
    }
    // Scan all modules
    for (auto& [_, mod] : modules_) {
        auto fit = mod.functions.find(qualified_name);
        if (fit != mod.functions.end())
            return &fit->second;
    }
    return nullptr;
}

} // namespace cvc::state_exec
