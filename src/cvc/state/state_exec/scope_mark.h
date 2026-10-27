// scope_mark.h — where a call found its function, and whether the scopes are still as they were
// (private to the evaluators and types.cpp).
#ifndef CVC_STATE_EXEC_SCOPE_MARK_H
#define CVC_STATE_EXEC_SCOPE_MARK_H

#include <cstdint>
#include <cvc/state/state_exec/types.h>
#include <string>
#include <utility>

namespace cvc::state_exec::detail {

/// The scopes a lookup walked, from the scope it started in out to the one that had the name:
/// how many (`depth`, 0 for none), and a stamp of them. The stamp's low half is the sum of their
/// sizes, which any binding they gain changes (none loses one while it lives); its high half
/// mixes which scopes they are (each one's address and serial), which a rewired chain
/// (environment::outer set by a host) changes, but for a 2^-32 chance. Not the address alone:
/// a scope a host makes in place of one it dropped is likely at that one's address (extend()
/// hands out the block the last dropped scope left), but never has its serial.
struct scope_mark {
  std::uint32_t depth = 0;
  std::uint64_t stamp = 0;
};

/// Add `e` to a stamp.
inline std::uint64_t stamp_with(std::uint64_t stamp, const environment *e) noexcept {
  const auto a = reinterpret_cast<std::uintptr_t>(e);
  // Two scopes at one address differ in serial, so in id (a product with an odd number is one
  // to one): a chain with one scope in place of another there always has another stamp.
  const auto id = static_cast<std::uint32_t>((a >> 4) ^ (static_cast<std::uint64_t>(a) >> 32)) ^
                  (e->serial.value * 0x85EBCA6Bu);
  const std::uint32_t ids = (static_cast<std::uint32_t>(stamp >> 32) ^ id) * 0x9E3779B1u;
  const std::uint32_t sizes =
      static_cast<std::uint32_t>(stamp) + static_cast<std::uint32_t>(e->bindings.size());
  return (static_cast<std::uint64_t>(ids) << 32) | sizes;
}

/// environment::lookup(name) from `env`, also marking the scopes it walked (depth 0 when nothing
/// binds the name, or the chain is too long to mark). The binding, or null.
const std::pair<const std::string, value_t> *find_marked(const environment *env,
                                                         const std::string &name, scope_mark &mark);

/// Whether the scopes from `env` are still the ones `mark` marked, as they were: then a lookup
/// from `env` of the name that made the mark finds what it found then.
inline bool mark_holds(const environment *env, const scope_mark &mark) noexcept {
  std::uint64_t stamp = 0;
  for (std::uint32_t i = 0; i < mark.depth; ++i, env = env->outer.get()) {
    if (!env)
      return false;
    stamp = stamp_with(stamp, env);
  }
  return mark.depth != 0 && stamp == mark.stamp;
}

} // namespace cvc::state_exec::detail

#endif // CVC_STATE_EXEC_SCOPE_MARK_H
