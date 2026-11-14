/**
 * @file stdlib.h
 * @brief Standard library modules for the state_exec DSL.
 *
 * Provides modules (string, bytes, math, collections) of native utility functions, named
 * with their module prefix (string.length, math.sqrt, ...). Every environment from
 * builtins::make_default_environment() reaches all of them through a frozen scope of its own
 * (stdlib_registry::make_scope()), so a DSL program calls (string.length "café") without
 * importing anything.
 */
#ifndef CVC_STATE_EXEC_STDLIB_H
#define CVC_STATE_EXEC_STDLIB_H

#include <cstdint>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc::state_exec {

/// The stream math.random draws from in a group (docs/STATE_EXEC_THREADED.md 4.10): owned by the
/// group, set on its scheduler (set_random_stream), which sets it in the context its evaluator
/// runs in (exec_context::random), so math.random draws from it whichever thread pumps the group.
/// Seeded on first use: from `seed` when it has one, else from several random_device words. With
/// none bound, math.random draws from its registry's own stream (each stdlib_registry has one,
/// under a mutex), as bare evaluators do.
struct random_stream {
  std::mt19937 engine;
  bool seeded = false;
  std::optional<std::uint64_t> seed;
};

/// Registry for standard library modules.
///
/// Modules are collections of named native functions. Built-in modules (string, bytes, math,
/// collections) are registered at construction; downstream code can build its own registry
/// and add modules via register_function(), then import_module() them into an environment.
/// Nothing is shared between registries: each is built anew (there is no process-wide one).
class stdlib_registry {
public:
  stdlib_registry();

  /// A new frozen scope holding every function of a new registry's built-in modules.
  /// builtins::make_default_environment() links one as its outer scope. A program's
  /// (set string.length ...) shadows the name in its own scope; nothing writes the scope, so
  /// threads may read it concurrently.
  static environment_ptr make_scope();

  /// Register a native function into a module.
  void register_function(const std::string &module, const std::string &func_name, native_fn fn);

  /// Get all function names in a module.
  std::vector<std::string> list_functions(const std::string &module) const;

  /// Get all module names.
  std::vector<std::string> list_modules() const;

  /// Import a module's functions into an environment.
  /// If specific_fns is non-empty, only import those functions.
  void import_module(const std::string &module, environment_ptr env,
                     const std::vector<std::string> &specific_fns = {}) const;

  /// Look up a qualified function name (e.g., "string.split").
  const native_fn *lookup_qualified(const std::string &qualified_name) const;

private:
  struct module_entry {
    std::unordered_map<std::string, native_fn> functions;
  };
  std::unordered_map<std::string, module_entry> modules_;

  void register_string_module();
  void register_bytes_module();
  void register_math_module();
  void register_collections_module();
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_STDLIB_H
