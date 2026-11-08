// Placing threads on CPUs (cpu_placement.h; docs/STATE_EXEC_THREADED.md 4.1, 4.4). Lifted from
// state_exec_bench's driver pinning (S1.md S1.6): Windows CPU sets, resolved at run time; Linux
// sched_setaffinity with cpu_capacity or the hybrid cpu_core list; nothing elsewhere.

#include <algorithm>
#include <cvc/core/cpu_placement.h>
#include <thread>
#include <tuple>

#if defined(__EMSCRIPTEN__)
// No placement.
#elif defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__linux__)
#include <cstdlib>
#include <fstream>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#endif

namespace cvc {

namespace {

unsigned hardware_threads() {
  const unsigned n = std::thread::hardware_concurrency();
  return n > 0 ? n : 1;
}

#if defined(_WIN32) && !defined(__EMSCRIPTEN__)

// The CPU set API is Windows 10+; _WIN32_WINNT is 0x0601 repo-wide, so it is resolved at run time.
using get_sets_fn = BOOL(WINAPI *)(PSYSTEM_CPU_SET_INFORMATION, ULONG, PULONG, HANDLE, ULONG);
using set_sets_fn = BOOL(WINAPI *)(HANDLE, const ULONG *, ULONG);

struct win_cpu {
  ULONG id; // its CPU set id
  logical_cpu cpu;
};

set_sets_fn set_sets_api() {
  static const set_sets_fn fn = reinterpret_cast<set_sets_fn>(reinterpret_cast<void *>(
      GetProcAddress(GetModuleHandleW(L"kernel32.dll"), "SetThreadSelectedCpuSets")));
  return fn;
}

// The CPU sets of processor group 0 the process's affinity mask allows, in index order.
std::vector<win_cpu> win_cpus() {
  std::vector<win_cpu> out;
  const auto get_sets = reinterpret_cast<get_sets_fn>(reinterpret_cast<void *>(
      GetProcAddress(GetModuleHandleW(L"kernel32.dll"), "GetSystemCpuSetInformation")));
  if (!get_sets)
    return out;
  DWORD_PTR process_mask = 0, system_mask = 0;
  if (!GetProcessAffinityMask(GetCurrentProcess(), &process_mask, &system_mask))
    process_mask = ~DWORD_PTR(0);
  ULONG len = 0;
  get_sets(nullptr, 0, &len, GetCurrentProcess(), 0);
  if (len == 0)
    return out;
  std::vector<unsigned char> buf(len);
  if (!get_sets(reinterpret_cast<PSYSTEM_CPU_SET_INFORMATION>(buf.data()), len, &len,
                GetCurrentProcess(), 0))
    return out;
  for (ULONG off = 0; off < len;) {
    const auto *e = reinterpret_cast<const SYSTEM_CPU_SET_INFORMATION *>(buf.data() + off);
    if (e->Type == CpuSetInformation && e->CpuSet.Group == 0) {
      const int lp = e->CpuSet.LogicalProcessorIndex;
      if (lp < int(8 * sizeof(DWORD_PTR)) && (process_mask & (DWORD_PTR(1) << lp)) != 0)
        out.push_back({e->CpuSet.Id, {lp, int(e->CpuSet.CoreIndex), e->CpuSet.EfficiencyClass}});
    }
    off += e->Size;
  }
  std::sort(out.begin(), out.end(),
            [](const win_cpu &a, const win_cpu &b) { return a.cpu.index < b.cpu.index; });
  return out;
}

bool pin_to(int cpu, std::string *siblings) {
  if (cpu < 0 || cpu >= int(8 * sizeof(DWORD_PTR)))
    return false;
  bool ok = SetThreadAffinityMask(GetCurrentThread(), DWORD_PTR(1) << cpu) != 0;
  const std::vector<win_cpu> cpus = win_cpus();
  int core = -1;
  for (const win_cpu &c : cpus)
    if (c.cpu.index == cpu) {
      core = c.cpu.core;
      if (const set_sets_fn set_sets = set_sets_api())
        ok = set_sets(GetCurrentThread(), &c.id, 1) && ok;
    }
  if (siblings)
    for (const win_cpu &c : cpus)
      if (core >= 0 && c.cpu.core == core && c.cpu.index != cpu)
        *siblings += (siblings->empty() ? "" : ",") + std::to_string(c.cpu.index);
  return ok;
}

#elif defined(__linux__) && !defined(__EMSCRIPTEN__)

std::string read_first_line(const std::string &path) {
  std::ifstream f(path);
  std::string s;
  std::getline(f, s);
  return s;
}

// "0-3,8,10-11" -> {0,1,2,3,8,10,11}.
std::vector<int> parse_cpu_list(const std::string &s) {
  std::vector<int> out;
  std::stringstream in(s);
  std::string part;
  while (std::getline(in, part, ',')) {
    if (part.empty())
      continue;
    const auto dash = part.find('-');
    const int lo = std::atoi(part.c_str());
    const int hi = dash == std::string::npos ? lo : std::atoi(part.c_str() + dash + 1);
    for (int c = lo; c <= hi && c >= 0 && c < CPU_SETSIZE; ++c)
      out.push_back(c);
  }
  return out;
}

std::vector<int> smt_siblings(int cpu) {
  return parse_cpu_list(read_first_line("/sys/devices/system/cpu/cpu" + std::to_string(cpu) +
                                        "/topology/thread_siblings_list"));
}

bool pin_to(int cpu, std::string *siblings) {
  if (cpu < 0 || cpu >= CPU_SETSIZE)
    return false;
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (sched_setaffinity(0, sizeof set, &set) != 0)
    return false;
  if (siblings)
    for (int c : smt_siblings(cpu))
      if (c != cpu)
        *siblings += (siblings->empty() ? "" : ",") + std::to_string(c);
  return true;
}

#endif

// The core logical CPU 0 is on (where most interrupts and DPCs land), or -1.
int core_of_cpu0(const std::vector<logical_cpu> &cpus) {
  for (const logical_cpu &c : cpus)
    if (c.index == 0)
      return c.core;
  return -1;
}

} // namespace

std::vector<logical_cpu> logical_cpus() {
  std::vector<logical_cpu> out;
#if defined(__EMSCRIPTEN__)
#elif defined(_WIN32)
  for (const win_cpu &c : win_cpus())
    out.push_back(c.cpu);
#elif defined(__linux__)
  cpu_set_t mask;
  if (sched_getaffinity(0, sizeof mask, &mask) != 0)
    return out;
  // Hybrid and big.LITTLE parts report a per-CPU capacity; x86 hybrid kernels that do not list
  // their performance cores under cpu_core.
  bool have_capacity = true;
  for (int i = 0; i < CPU_SETSIZE; ++i) {
    if (!CPU_ISSET(i, &mask))
      continue;
    logical_cpu c;
    c.index = i;
    const std::vector<int> siblings = smt_siblings(i);
    c.core = siblings.empty() ? i : *std::min_element(siblings.begin(), siblings.end());
    std::ifstream f("/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpu_capacity");
    long capacity = -1;
    if (f >> capacity && capacity >= 0)
      c.efficiency = static_cast<unsigned>(capacity);
    else
      have_capacity = false;
    out.push_back(c);
  }
  if (!have_capacity) {
    const std::vector<int> p_cores = parse_cpu_list(read_first_line("/sys/devices/cpu_core/cpus"));
    for (logical_cpu &c : out)
      c.efficiency = std::find(p_cores.begin(), p_cores.end(), c.index) != p_cores.end() ? 1 : 0;
  }
#endif
  return out;
}

unsigned performance_core_count() {
  const std::vector<logical_cpu> cpus = logical_cpus();
  if (cpus.empty())
    return hardware_threads();
  unsigned best = 0;
  for (const logical_cpu &c : cpus)
    best = std::max(best, c.efficiency);
  unsigned n = 0;
  for (const logical_cpu &c : cpus)
    n += c.efficiency == best ? 1u : 0u;
  return std::max(n, 1u);
}

std::vector<logical_cpu> placement_cpus(cpu_set set) {
  std::vector<logical_cpu> cpus = logical_cpus();
  if (cpus.empty())
    return cpus;
  unsigned best = 0;
  for (const logical_cpu &c : cpus)
    best = std::max(best, c.efficiency);
  if (set == cpu_set::performance)
    cpus.erase(std::remove_if(cpus.begin(), cpus.end(),
                              [best](const logical_cpu &c) { return c.efficiency != best; }),
               cpus.end());
  const int core0 = core_of_cpu0(cpus);
  // Each CPU's rank on its core: 0 for the lowest-index logical CPU of the core, 1 for its first
  // hyperthread sibling, and so on (an unknown core counts as a core of its own).
  std::vector<int> rank(cpus.size(), 0);
  for (std::size_t i = 0; i < cpus.size(); ++i)
    for (std::size_t j = 0; j < cpus.size(); ++j)
      if (cpus[i].core >= 0 && cpus[j].core == cpus[i].core && cpus[j].index < cpus[i].index)
        ++rank[i];
  // The driver's CPU (pin_current_thread(-1)) goes last, so a pool of fewer workers than CPUs never
  // uses it: the first logical CPU of the fastest class's highest core other than logical CPU 0's
  // (where most interrupts land), or of CPU 0's core when it is the only one.
  int driver_core = -1;
  for (const logical_cpu &c : cpus)
    if (c.efficiency == best && c.core != core0 && c.core > driver_core)
      driver_core = c.core;
  if (driver_core < 0)
    driver_core = core0;
  std::size_t driver = cpus.size();
  for (std::size_t i = 0; i < cpus.size(); ++i)
    if (cpus[i].efficiency == best && rank[i] == 0 &&
        (driver_core >= 0 ? cpus[i].core == driver_core : cpus[i].index == cpus.back().index))
      driver = i;
  // The faster class first; in each class one logical CPU per physical core before any
  // hyperthread sibling, so the first workers never share a core; at each rank, logical CPU 0's
  // core and the driver's after the others; then by index.
  std::vector<std::size_t> order;
  for (std::size_t i = 0; i < cpus.size(); ++i)
    if (i != driver)
      order.push_back(i);
  const auto key = [&](std::size_t i) {
    const bool on_core0 = core0 >= 0 && cpus[i].core == core0;
    const bool on_driver = driver_core >= 0 && cpus[i].core == driver_core;
    return std::make_tuple(-static_cast<long long>(cpus[i].efficiency), rank[i], on_driver,
                           on_core0, cpus[i].index);
  };
  std::sort(order.begin(), order.end(),
            [&](std::size_t a, std::size_t b) { return key(a) < key(b); });
  std::vector<logical_cpu> out;
  out.reserve(cpus.size());
  for (std::size_t i : order)
    out.push_back(cpus[i]);
  if (driver < cpus.size())
    out.push_back(cpus[driver]);
  return out;
}

bool place_current_thread(cpu_set set, unsigned index) {
#if defined(__EMSCRIPTEN__) || (!defined(_WIN32) && !defined(__linux__))
  (void)set;
  (void)index;
  return false;
#else
  const std::vector<logical_cpu> cpus = placement_cpus(set);
  if (cpus.empty())
    return false;
  return pin_to(cpus[index % cpus.size()].index, nullptr);
#endif
}

pin_result pin_current_thread(int cpu) {
  pin_result r;
#if defined(__EMSCRIPTEN__) || (!defined(_WIN32) && !defined(__linux__))
  (void)cpu;
  r.how = "pinning not supported on this platform";
  return r;
#else
  const bool automatic = cpu < 0;
  std::string unit;
  if (automatic) {
    // The performance CPU placement_cpus lists last: off the core logical CPU 0 sits on (where
    // most interrupts land), and never used by a pool of fewer workers than CPUs.
    const std::vector<logical_cpu> cpus = placement_cpus(cpu_set::performance);
    if (cpus.empty()) {
#if defined(_WIN32)
      r.how = "auto: no CPU set information";
#else
      r.how = "auto: no cpu_capacity or cpu_core list (not pinned)";
#endif
      return r;
    }
    cpu = cpus.back().index;
  }
  for (const logical_cpu &c : logical_cpus())
    if (c.index == cpu)
      unit = " efficiency class " + std::to_string(c.efficiency);
  std::string siblings;
  const bool ok = pin_to(cpu, &siblings);
  r.how =
      std::string(automatic ? "auto:" : "explicit:") + unit + (ok ? "" : " (affinity call failed)");
  if (ok) {
    r.cpu = cpu;
    r.siblings = siblings;
  }
  return r;
#endif
}

} // namespace cvc
