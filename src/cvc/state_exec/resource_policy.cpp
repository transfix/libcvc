#include <algorithm>
#include <cvc/state_exec/resource_policy.h>
#include <stdexcept>

namespace cvc::state_exec {

namespace {

template <typename T> T clamp_limit(T val, T lo, T hi, const char *field, resource_policy::mode m) {
  // 0 means "not specified" for both bounds and value
  T effective_lo = lo;
  T effective_hi = hi;
  if (effective_lo == T{0} && effective_hi == T{0})
    return val; // no constraints

  if (effective_lo != T{0} && val != T{0} && val < effective_lo) {
    if (m == resource_policy::mode::strict)
      throw std::runtime_error(std::string("resource_policy: ") + field + " below minimum");
    val = effective_lo;
  }
  if (effective_hi != T{0} && val != T{0} && val > effective_hi) {
    if (m == resource_policy::mode::strict)
      throw std::runtime_error(std::string("resource_policy: ") + field + " above maximum");
    val = effective_hi;
  }
  return val;
}

template <typename T> T apply_default(T val, T def) {
  return (val == T{0} && def != T{0}) ? def : val;
}

} // anonymous namespace

process_limits validate_limits(const resource_policy &policy, const process_limits &requested) {
  process_limits result = requested;

  // Apply defaults for unspecified limits
  result.max_steps = apply_default(result.max_steps, policy.max_steps_default);
  result.max_time = apply_default(result.max_time, policy.max_time_default);
  result.max_memory = apply_default(result.max_memory, policy.max_memory_default);
  result.max_messages = apply_default(result.max_messages, policy.max_messages_default);

  // Clamp/validate each field
  result.max_steps = clamp_limit(result.max_steps, policy.max_steps_min, policy.max_steps_max,
                                 "max_steps", policy.enforce);

  result.max_time = clamp_limit(result.max_time, policy.max_time_min, policy.max_time_max,
                                "max_time", policy.enforce);

  result.max_memory = clamp_limit(result.max_memory, policy.max_memory_min, policy.max_memory_max,
                                  "max_memory", policy.enforce);

  result.max_messages = clamp_limit(result.max_messages, policy.max_messages_min,
                                    policy.max_messages_max, "max_messages", policy.enforce);

  return result;
}

} // namespace cvc::state_exec
