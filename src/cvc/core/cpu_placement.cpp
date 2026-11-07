#include <cvc/core/cpu_placement.h>
#include <thread>

namespace cvc {

// Stage 1 interfaces: stubs that report nothing and pin nothing. S1.6 lifts state_exec_bench's
// CPU-set and affinity code here.

std::vector<logical_cpu> logical_cpus() { return {}; }

unsigned performance_core_count() {
  const unsigned n = std::thread::hardware_concurrency();
  return n > 0 ? n : 1;
}

std::vector<logical_cpu> placement_cpus(cpu_set) { return {}; }

bool place_current_thread(cpu_set, unsigned) { return false; }

pin_result pin_current_thread(int) {
  pin_result r;
  r.how = "not pinned: no placement in this build";
  return r;
}

} // namespace cvc
