/**
 * @file stdlib.h
 * @brief Standard library modules for the state_exec DSL.
 *
 * Provides loadable modules (string, math, collections) containing
 * native utility functions.  Modules are registered via stdlib_registry
 * and imported into a DSL program's environment with (import "module").
 */
#ifndef CVC_STATE_EXEC_STDLIB_H
#define CVC_STATE_EXEC_STDLIB_H

#include <cvc/state_exec/types.h>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc::state_exec {

/// Registry for standard library modules.
///
/// Modules are collections of named native functions that can be imported
/// into a DSL environment via (require "module").  Built-in modules
/// (string, math, collections) are registered at construction; downstream
/// code can add custom modules via register_function().
class stdlib_registry {
public:
  stdlib_registry();

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
  void register_math_module();
  void register_collections_module();
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_STDLIB_H
