/**
 * @file types.h
 * @brief Core value types and environment model for the state_exec DSL.
 *
 * Defines value_t (a tagged union over nil, bool, int64, double, string,
 * symbol, list, dict, lambda, native_fn, error), along with environment_t
 * (lexically-scoped variable bindings) and execute_options (per-process
 * resource limits).
 */
#ifndef CVC_STATE_EXEC_TYPES_H
#define CVC_STATE_EXEC_TYPES_H

#include <boost/any.hpp>

#include <cstdint>
#include <compare>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace cvc::state_exec {

// Forward declarations
struct closure;
struct environment;

using closure_ptr     = std::shared_ptr<closure>;
using environment_ptr = std::shared_ptr<environment>;

/// Symbol name in the DSL.
struct symbol {
    std::string name;
    auto operator<=>(const symbol&) const = default;
    bool operator==(const symbol& o) const { return name == o.name; }
};

/// Typed data object wrapping boost::any from state::data().
struct data_object {
    boost::any  payload;
    std::string type_name;
    bool is_type(const std::string& t) const { return type_name == t; }
};
using data_object_ptr = std::shared_ptr<data_object>;

// Ordered map preserving insertion order for dicts
using ordered_map = std::vector<std::pair<std::string, struct value_tag>>;

// Forward-declare value_t (variant needs complete types for some alternatives;
// we use shared_ptr to break the recursion)
struct value_tag;
using list_ptr = std::shared_ptr<std::vector<value_tag>>;
using dict_ptr = std::shared_ptr<std::vector<std::pair<std::string, value_tag>>>;
using native_fn = std::function<value_tag(std::span<const value_tag>)>;

/// The DSL value variant.  All runtime values are represented as value_t.
///
/// Variants:
///   monostate  → nil
///   bool       → #t / #f
///   int64_t    → integer
///   double     → float
///   string     → string literal
///   symbol     → symbol name
///   list_ptr   → ordered list
///   closure_ptr → lambda / defun closure
///   dict_ptr   → ordered dictionary
///   native_fn  → C++ callable (built-in or stdlib bridge)
///   data_object_ptr → typed data from state::data()
struct value_tag {
    using variant_type = std::variant<
        std::monostate,
        bool,
        int64_t,
        double,
        std::string,
        symbol,
        list_ptr,
        closure_ptr,
        dict_ptr,
        native_fn,
        data_object_ptr
    >;

    variant_type v;

    value_tag() : v(std::monostate{}) {}

    // Implicit constructors for convenience
    value_tag(std::monostate) : v(std::monostate{}) {}
    value_tag(bool b) : v(b) {}
    value_tag(int64_t i) : v(i) {}
    value_tag(int i) : v(static_cast<int64_t>(i)) {}
    value_tag(double d) : v(d) {}
    value_tag(const std::string& s) : v(s) {}
    value_tag(std::string&& s) : v(std::move(s)) {}
    value_tag(const char* s) : v(std::string(s)) {}
    value_tag(symbol s) : v(std::move(s)) {}
    value_tag(list_ptr l) : v(std::move(l)) {}
    value_tag(closure_ptr c) : v(std::move(c)) {}
    value_tag(dict_ptr d) : v(std::move(d)) {}
    value_tag(native_fn f) : v(std::move(f)) {}
    value_tag(data_object_ptr d) : v(std::move(d)) {}

    /// Check if the value is nil (monostate).
    bool is_nil() const { return std::holds_alternative<std::monostate>(v); }

    /// Truthiness: nil and false are falsy, everything else is truthy.
    bool is_truthy() const {
        if (is_nil()) return false;
        if (auto* b = std::get_if<bool>(&v)) return *b;
        return true;
    }

    /// Type name string for error messages / introspection.
    std::string type_name() const;
};

using value_t = value_tag;

/// Closure: a lambda body + captured environment.
struct closure {
    std::vector<symbol> params;
    bool variadic = false;   // &rest parameter
    std::vector<value_t> body;
    environment_ptr env_snapshot;
};

/// Lexically-scoped environment chain.
struct environment {
    std::unordered_map<std::string, value_t> bindings;
    std::shared_ptr<environment> outer;

    /// Look up a name, walking the scope chain.
    value_t* lookup(const std::string& name);
    const value_t* lookup(const std::string& name) const;

    /// Set a binding in this scope.
    void set(const std::string& name, value_t val);

    /// Set a binding in the nearest enclosing scope that has it,
    /// or in this scope if not found.
    void set_existing(const std::string& name, value_t val);

    /// Create a new child scope.
    static environment_ptr extend(environment_ptr parent);
};

// ---- Convenience helpers ---------------------------------------------------

/// Make a list value from a vector of values.
inline value_t make_list(std::vector<value_t> elems) {
    return value_t(std::make_shared<std::vector<value_t>>(std::move(elems)));
}

/// Make an empty list.
inline value_t make_list() {
    return value_t(std::make_shared<std::vector<value_t>>());
}

/// Make a dict value from key-value pairs.
inline value_t make_dict(
    std::vector<std::pair<std::string, value_t>> entries = {})
{
    return value_t(std::make_shared<
        std::vector<std::pair<std::string, value_t>>>(std::move(entries)));
}

/// The nil value.
inline const value_t nil_value{};

/// Boolean true value.
inline const value_t true_value{true};

/// Boolean false value.
inline const value_t false_value{false};

/// Convert a value_t to a human-readable string representation.
std::string to_string(const value_t& val);

/// Check structural equality of two values.
bool values_equal(const value_t& a, const value_t& b);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_TYPES_H
