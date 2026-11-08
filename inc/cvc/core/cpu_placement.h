#ifndef CVC_CORE_CPU_PLACEMENT_H
#define CVC_CORE_CPU_PLACEMENT_H

// Placing threads on CPUs (docs/STATE_EXEC_THREADED.md 4.1, 4.4): which logical CPUs the process
// may use, which of them are performance cores on a hybrid part, and pinning the calling thread to
// one. The exec_pool places its workers with it, and state_exec_bench pins its driver with it.
//
// - Windows: CPU sets, resolved at run time (Windows 10 and later), and the thread's affinity
//   mask; processor group 0 only. A CPU's efficiency class is its EfficiencyClass.
// - Linux: sched_getaffinity and sched_setaffinity; the efficiency class comes from cpu_capacity,
//   or the hybrid cpu_core list where the kernel gives no capacity.
// - macOS, Emscripten and anything else: no placement; the calls report nothing and pin nothing.

#include <string>
#include <vector>

namespace cvc {

// Which CPUs a pool's workers go on. `performance`: the highest efficiency class only (td Q8).
// `all`: every CPU the process may use, the performance cores first.
enum class cpu_set { performance, all };

// One logical CPU the process may run on.
struct logical_cpu {
  int index = -1;          // its logical processor number
  int core = -1;           // the physical core it is on (-1: unknown)
  unsigned efficiency = 0; // its efficiency class: higher is faster; all 0 on a uniform part
};

// The logical CPUs the process may run on (its affinity mask), with their cores and efficiency
// classes, in index order. Empty where the platform says nothing.
std::vector<logical_cpu> logical_cpus();

// The logical CPUs in the highest efficiency class the process may use; hardware_concurrency()
// (at least 1) where that is unknown.
unsigned performance_core_count();

// The CPUs `set` names, in the order a pool places its workers on them (worker i on entry i;
// the pool lets a worker beyond them float). The faster class first; in each class one logical
// CPU per physical core before any hyperthread sibling, so the first workers never share a core,
// with logical CPU 0's core (where most interrupts land) after the others. Last comes the CPU
// pin_current_thread(-1) takes: off CPU 0's core, and never used by a pool of fewer workers than
// CPUs. Empty where the platform says nothing.
std::vector<logical_cpu> placement_cpus(cpu_set set);

// Pin the calling thread to the `index`-th CPU of placement_cpus(set) (wrapping). False when the
// platform has no placement or the OS refused it.
bool place_current_thread(cpu_set set, unsigned index);

// What pin_current_thread did.
struct pin_result {
  int cpu = -1;         // the logical CPU the thread is pinned to, or -1 (not pinned)
  std::string how;      // how it was chosen, or why it was not pinned
  std::string siblings; // the other logical CPUs on its core, comma-separated ("" when none)
};

// Pin the calling thread to logical CPU `cpu`, or, with `cpu` below 0, to a performance core off
// the core logical CPU 0 is on that placement_cpus(cpu_set::performance) lists last, which no
// pool worker takes (the bench's --pin auto).
pin_result pin_current_thread(int cpu = -1);

} // namespace cvc

#endif // CVC_CORE_CPU_PLACEMENT_H
