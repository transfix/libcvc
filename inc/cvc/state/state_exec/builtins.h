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

#include <cvc/state/state_exec/types.h>
#include <string>
#include <unordered_map>

namespace cvc {
class app;
} // namespace cvc

namespace cvc::state_exec {

/// Registry of built-in functions.
///
/// All operators from the DSL (arithmetic, comparison, string, list, dict,
/// OOP, higher-order) are registered here as native_fn values.  The registry
/// is returned as an environment_ptr that can be used as the global scope
/// for any evaluator variant.
class builtins {
public:
  /// Create the default built-in registry with all standard operators. Each call builds a new
  /// one, its outer a new frozen stdlib scope (stdlib_registry::make_scope()): nothing in it is
  /// shared with another environment or app.
  static environment_ptr make_default_environment();

  /// Register a single built-in function by name.
  static void register_fn(environment_ptr env, const std::string &name, native_fn fn);
};

/// `app`'s frozen default environment: built at the first call for the app, then frozen, so the
/// app's groups on any thread share it (docs/STATE_EXEC_THREADED.md 4.2). A group's global env is
/// environment::extend() of it, so a set lands in the group's own scope: set() on this one
/// throws. Its outer is a stdlib scope of its own. One per app, destroyed with it: two apps never
/// share one. Bare schedulers, the app's and a plain pycvc Exec keep their own
/// make_default_environment().
environment_ptr frozen_builtins(cvc::app &app);

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
value_t builtin_eq(call_args args);
value_t builtin_ne(call_args args);
/// @}

/// @name String
/// @{
value_t builtin_str_concat(call_args args);
value_t builtin_str(call_args args);
value_t builtin_url_encode(std::span<const value_t> args); // (url-encode s) -> percent-encoded
value_t builtin_url_decode(std::span<const value_t> args); // (url-decode s) -> percent-decoded
/// @}

/// Percent-encode `s` per RFC 3986: the unreserved set (A-Za-z0-9 and -._~) passes through, every
/// other byte becomes %XX (uppercase hex; space -> %20). Shared by (url-encode) and Ariadne's HTTP
/// query/form builders so a query string and (url-encode) agree byte for byte.
std::string percent_encode(const std::string &s);
/// Reverse of percent_encode: %XX -> the byte (a malformed escape is kept literally); '+' -> space
/// (the application/x-www-form-urlencoded convention, so a decoded query value reads naturally).
std::string percent_decode(const std::string &s);

/// @name JSON (make_json_parse_job, make_json_encode_job in data_job.h)
/// @{
value_t builtin_json_parse(call_args args);  // (json-parse text) -> value
value_t builtin_json_encode(call_args args); // (json-encode v [indent]) -> text
/// @}

/// @name List
/// @{
value_t builtin_list(std::span<const value_t> args);
value_t builtin_car(std::span<const value_t> args);
value_t builtin_cdr(std::span<const value_t> args);
value_t builtin_cons(call_args args);
value_t builtin_nth(std::span<const value_t> args);
value_t builtin_set_nth(call_args args);
value_t builtin_length(std::span<const value_t> args);
value_t builtin_append(call_args args);
value_t builtin_slice(call_args args);
value_t builtin_del_nth(std::span<const value_t> args);
/// @}

/// @name Dict
/// @{
value_t builtin_dict(std::span<const value_t> args);
value_t builtin_get_attr(std::span<const value_t> args);
value_t builtin_set_attr(call_args args);
value_t builtin_del_attr(std::span<const value_t> args);
/// @}

/// @name OOP
/// @{
value_t builtin_send(call_args args);
/// @}

/// @name Higher-order
/// @{
value_t builtin_apply(call_args args);
/// @}

/// @name Type predicates
/// @{
value_t builtin_is_null(std::span<const value_t> args);
value_t builtin_is_list(std::span<const value_t> args);
value_t builtin_type_of(std::span<const value_t> args);
value_t builtin_isinstance(std::span<const value_t> args); // (isinstance x Class)
/// @}

/// @name Exceptions (docs/STATE_EXEC_EXCEPTIONS.md)
/// @{
value_t builtin_error(std::span<const value_t> args);  // (error message [data]): a RuntimeError
value_t builtin_assert(std::span<const value_t> args); // (assert cond [message [data]]): an
                                                       // AssertionError when cond is false
/// @}

/// @name Hashing and secure randomness
/// @{
value_t builtin_sha256(std::span<const value_t> args);     // (sha256 x): 32 bytes
value_t builtin_sha256_hex(std::span<const value_t> args); // (sha256-hex x): 64 lowercase hex
/// (random-token nbytes ["base64url"|"hex"]): nbytes (1..kRandomTokenMaxBytes) from the OS CSPRNG
value_t builtin_random_token(std::span<const value_t> args);
inline constexpr int64_t kRandomTokenMaxBytes = 256;
/// @}

/// @name I/O
/// @{
value_t builtin_print(call_args args);
/// @}

/// @name Logic
/// @{
value_t builtin_not(std::span<const value_t> args);
value_t builtin_and(std::span<const value_t> args);
value_t builtin_or(std::span<const value_t> args);
/// @}

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_BUILTINS_H
