/**
 * @file builtins.h
 * @brief Built-in operator registry for the state_exec DSL.
 *
 * Provides make_builtins() which returns an environment containing all
 * core DSL operators: arithmetic, comparison, logic, string, list, dict,
 * OOP (class/new/method-call), higher-order (map/filter/reduce/apply),
 * and control flow (if/cond/begin/let/define/set!/lambda/defmacro).
 */
#ifndef CVC_STATE_EXEC_BUILTINS_H
#define CVC_STATE_EXEC_BUILTINS_H

#include <cvc/state_exec/types.h>

#include <string>
#include <unordered_map>

namespace cvc::state_exec {

/// Registry of built-in functions.
///
/// All operators from the DSL (arithmetic, comparison, string, list, dict,
/// OOP, higher-order) are registered here as native_fn values.  The registry
/// is returned as an environment_ptr that can be used as the global scope
/// for any evaluator variant.
class builtins {
public:
    /// Create the default built-in registry with all standard operators.
    static environment_ptr make_default_environment();

    /// Register a single built-in function by name.
    static void register_fn(environment_ptr env,
                            const std::string& name,
                            native_fn fn);
};

// --- Individual built-in implementations (for direct use / testing) ---

/// @name Arithmetic
/// @{
value_t builtin_add(std::span<const value_t> args);
value_t builtin_sub(std::span<const value_t> args);
value_t builtin_mul(std::span<const value_t> args);
value_t builtin_div(std::span<const value_t> args);
value_t builtin_mod(std::span<const value_t> args);
/// @}

/// @name Comparison
/// @{
value_t builtin_lt(std::span<const value_t> args);
value_t builtin_gt(std::span<const value_t> args);
value_t builtin_le(std::span<const value_t> args);
value_t builtin_ge(std::span<const value_t> args);
value_t builtin_eq(std::span<const value_t> args);
value_t builtin_ne(std::span<const value_t> args);
/// @}

/// @name String
/// @{
value_t builtin_str_concat(std::span<const value_t> args);
value_t builtin_str(std::span<const value_t> args);
/// @}

/// @name List
/// @{
value_t builtin_list(std::span<const value_t> args);
value_t builtin_car(std::span<const value_t> args);
value_t builtin_cdr(std::span<const value_t> args);
value_t builtin_cons(std::span<const value_t> args);
value_t builtin_nth(std::span<const value_t> args);
value_t builtin_set_nth(std::span<const value_t> args);
value_t builtin_length(std::span<const value_t> args);
value_t builtin_append(std::span<const value_t> args);
value_t builtin_slice(std::span<const value_t> args);
value_t builtin_del_nth(std::span<const value_t> args);
/// @}

/// @name Dict
/// @{
value_t builtin_dict(std::span<const value_t> args);
value_t builtin_get_attr(std::span<const value_t> args);
value_t builtin_set_attr(std::span<const value_t> args);
value_t builtin_del_attr(std::span<const value_t> args);
/// @}

/// @name OOP
/// @{
value_t builtin_send(std::span<const value_t> args);
/// @}

/// @name Higher-order
/// @{
value_t builtin_apply(std::span<const value_t> args);
/// @}

/// @name Type predicates
/// @{
value_t builtin_is_null(std::span<const value_t> args);
value_t builtin_is_list(std::span<const value_t> args);
value_t builtin_type_of(std::span<const value_t> args);
/// @}

/// @name I/O
/// @{
value_t builtin_print(std::span<const value_t> args);
/// @}

/// @name Logic
/// @{
value_t builtin_not(std::span<const value_t> args);
value_t builtin_and(std::span<const value_t> args);
value_t builtin_or(std::span<const value_t> args);
/// @}

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_BUILTINS_H
