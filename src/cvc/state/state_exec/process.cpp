#include <algorithm>
#include <chrono>
#include <cvc/state/state_exec/process.h>

namespace cvc::state_exec {

const char *to_string(process_status s) {
  switch (s) {
  case process_status::ready:
    return "ready";
  case process_status::running:
    return "running";
  case process_status::paused:
    return "paused";
  case process_status::waiting:
    return "waiting";
  case process_status::terminated:
    return "terminated";
  case process_status::killed:
    return "killed";
  }
  return "unknown";
}

process_status parse_process_status(const std::string &s) {
  if (s == "ready")
    return process_status::ready;
  if (s == "running")
    return process_status::running;
  if (s == "paused")
    return process_status::paused;
  if (s == "waiting")
    return process_status::waiting;
  if (s == "terminated")
    return process_status::terminated;
  if (s == "killed")
    return process_status::killed;
  return process_status::ready;
}

double process::elapsed_time() const {
  double t = accumulated_time;
  if (status == process_status::running) {
    auto now = std::chrono::steady_clock::now();
    t += std::chrono::duration<double>(now - last_run_start).count();
  }
  return t;
}

std::optional<double> process::time_left() const {
  if (max_time <= 0.0)
    return std::nullopt;
  const double left =
      charges_cpu_time() ? 0.5 * max_time - cpu_time_charged : max_time - elapsed_time();
  return std::max(0.0, left);
}

std::optional<std::size_t> process::alloc_limit(std::size_t used_bytes) const {
  std::optional<std::size_t> limit;
  if (max_alloc_bytes > 0)
    limit = static_cast<std::size_t>(std::min<uint64_t>(max_alloc_bytes, SIZE_MAX));
  if (max_memory > 0) {
    const uint64_t left = max_memory > used_bytes ? max_memory - used_bytes : 0;
    const std::size_t left_bytes = static_cast<std::size_t>(std::min<uint64_t>(left, SIZE_MAX));
    limit = limit ? std::min(*limit, left_bytes) : left_bytes;
  }
  return limit;
}

} // namespace cvc::state_exec
