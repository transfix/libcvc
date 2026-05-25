#include <chrono>
#include <cvc/state_exec/process.h>

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

} // namespace cvc::state_exec
