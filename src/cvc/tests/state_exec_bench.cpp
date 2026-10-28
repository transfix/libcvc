/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// state_exec_bench — the performance footprint of the state_exec DSL runtime: the recursive
// evaluator, the stackless evaluator, the sync scheduler and the async scheduler (the one
// app::exec_scheduler() builds, pumped the way Ariadne and pycvc pump it), under single programs,
// floods of small and large programs, async functions on the app's pools and lanes, messaging,
// native calls, idle footprint, resource guards and parsing. docs/STATE_EXEC_BENCH.md says what
// each scenario measures and why; tools/bench/state_exec_bench.py runs it a process per scenario
// and compares runs.
//
//   state_exec_bench --list
//   state_exec_bench --scenario 'eval.*' --reps 5 --pin auto --json base.jsonl
//   state_exec_bench --scenario flood.small --size n=100,1000 --executor async
//   state_exec_bench --smoke
//   state_exec_bench --scenario eval.fib --executor stackless --reps 1 --sample prof/fib
//
// Every scenario checks what its programs returned: a record whose programs failed carries
// ok:false and the exit code is 1. Numbers are only comparable between runs with the same pinning,
// build and sizes; the JSON records carry all three.

// clang-format off
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <dbghelp.h>
#include <intrin.h>
#include <psapi.h>
#include <tlhelp32.h>
#include <winternl.h>
#else
#include <dirent.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <mach/mach.h>
#include <malloc/malloc.h>
#else
#include <malloc.h>
#include <pthread.h>
#include <sched.h>
#include <sys/syscall.h>
#endif
#endif
// clang-format on

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/core/thread_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/types.h>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <new>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace se = cvc::state_exec;

// ---------------------------------------------------------------------------------------------
// Allocation counting. Allocation counts are deterministic, so they show what an optimization
// saves well below the wall-time noise. The count is per thread (a plain thread_local: an atomic
// would cost every allocation a locked add and perturb what is measured), read on the driver.
// POSIX: this executable replaces operator new, which every shared library's operator new
// resolves to. Windows: each DLL links its own operator new, so a replacement here would see only
// this executable's; instead the import table of every non-system module (cvc.dll among them) is
// patched to send malloc, calloc and realloc through a counter (bench_alloc::install).
// ---------------------------------------------------------------------------------------------

namespace bench_alloc {
thread_local uint64_t count = 0;
bool active = false;
const char *scope = "none";

#if defined(_WIN32)
using malloc_fn = void *(__cdecl *)(size_t);
using calloc_fn = void *(__cdecl *)(size_t, size_t);
using realloc_fn = void *(__cdecl *)(void *, size_t);
malloc_fn real_malloc = nullptr;
calloc_fn real_calloc = nullptr;
realloc_fn real_realloc = nullptr;
void *__cdecl counting_malloc(size_t n) {
  ++count;
  return real_malloc(n);
}
void *__cdecl counting_calloc(size_t n, size_t s) {
  ++count;
  return real_calloc(n, s);
}
void *__cdecl counting_realloc(void *p, size_t n) {
  ++count;
  return real_realloc(p, n);
}

// Point module m's imports of the CRT heap's malloc/calloc/realloc at the counting ones.
int patch_module(HMODULE m) {
  auto *base = reinterpret_cast<unsigned char *>(m);
  const auto *dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(base);
  if (dos->e_magic != IMAGE_DOS_SIGNATURE)
    return 0;
  const auto *nt = reinterpret_cast<const IMAGE_NT_HEADERS *>(base + dos->e_lfanew);
  const IMAGE_DATA_DIRECTORY &dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
  if (!dir.VirtualAddress)
    return 0;
  int patched = 0;
  for (auto *imp = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR *>(base + dir.VirtualAddress);
       imp->Name; ++imp) {
    const char *dll = reinterpret_cast<const char *>(base + imp->Name);
    if (_strnicmp(dll, "api-ms-win-crt-heap", 19) != 0 && _stricmp(dll, "ucrtbase.dll") != 0)
      continue;
    if (!imp->OriginalFirstThunk)
      continue;
    auto *names = reinterpret_cast<IMAGE_THUNK_DATA *>(base + imp->OriginalFirstThunk);
    auto *addrs = reinterpret_cast<IMAGE_THUNK_DATA *>(base + imp->FirstThunk);
    for (; names->u1.AddressOfData; ++names, ++addrs) {
      if (IMAGE_SNAP_BY_ORDINAL(names->u1.Ordinal))
        continue;
      const auto *ibn =
          reinterpret_cast<const IMAGE_IMPORT_BY_NAME *>(base + names->u1.AddressOfData);
      const char *fn = reinterpret_cast<const char *>(ibn->Name);
      void *repl = nullptr;
      if (std::strcmp(fn, "malloc") == 0)
        repl = reinterpret_cast<void *>(&counting_malloc);
      else if (std::strcmp(fn, "calloc") == 0)
        repl = reinterpret_cast<void *>(&counting_calloc);
      else if (std::strcmp(fn, "realloc") == 0)
        repl = reinterpret_cast<void *>(&counting_realloc);
      if (!repl)
        continue;
      DWORD old = 0;
      if (!VirtualProtect(&addrs->u1.Function, sizeof(void *), PAGE_READWRITE, &old))
        continue;
      addrs->u1.Function = reinterpret_cast<ULONG_PTR>(repl);
      VirtualProtect(&addrs->u1.Function, sizeof(void *), old, &old);
      ++patched;
    }
  }
  return patched;
}

// Patch every loaded module outside the Windows directory: this executable, cvc.dll and the
// project's dependencies. Called once from main, before any measured work.
void install() {
  HMODULE crt = GetModuleHandleW(L"ucrtbase.dll");
  if (!crt)
    return;
  real_malloc =
      reinterpret_cast<malloc_fn>(reinterpret_cast<void *>(GetProcAddress(crt, "malloc")));
  real_calloc =
      reinterpret_cast<calloc_fn>(reinterpret_cast<void *>(GetProcAddress(crt, "calloc")));
  real_realloc =
      reinterpret_cast<realloc_fn>(reinterpret_cast<void *>(GetProcAddress(crt, "realloc")));
  if (!real_malloc || !real_calloc || !real_realloc)
    return;
  wchar_t windir[MAX_PATH] = {};
  const UINT wl = GetSystemWindowsDirectoryW(windir, MAX_PATH);
  std::vector<HMODULE> mods(1024);
  DWORD need = 0;
  if (!EnumProcessModules(GetCurrentProcess(), mods.data(), DWORD(mods.size() * sizeof(HMODULE)),
                          &need))
    return;
  mods.resize(std::min<std::size_t>(mods.size(), need / sizeof(HMODULE)));
  int patched = 0;
  for (HMODULE m : mods) {
    wchar_t path[MAX_PATH] = {};
    if (!GetModuleFileNameW(m, path, MAX_PATH))
      continue;
    if (wl && _wcsnicmp(path, windir, wl) == 0)
      continue;
    patched += patch_module(m);
  }
  if (patched) {
    active = true;
    scope = "malloc/calloc/realloc calls from non-system modules";
  }
}
#else
void install() {
  active = true;
  scope = "operator new calls";
}
#endif
} // namespace bench_alloc

#if !defined(_WIN32)
// The replacement every module's operator new resolves to (see above). The array and nothrow
// forms call this one by default.
void *operator new(std::size_t n) {
  ++bench_alloc::count;
  if (void *p = std::malloc(n ? n : 1))
    return p;
  throw std::bad_alloc();
}
#endif

// Threads the process has started, counted as they attach (Windows: a TLS callback in this
// executable sees DLL_THREAD_ATTACH for every new thread). A thread that starts and exits between
// two samples of the thread count still shows here; elsewhere there is no cheap equivalent.
namespace bench_threads {
std::atomic<int64_t> created{0};
}
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_ARM64))
extern "C" void NTAPI bench_tls_callback(PVOID, DWORD reason, PVOID) {
  if (reason == DLL_THREAD_ATTACH)
    bench_threads::created.fetch_add(1, std::memory_order_relaxed);
}
#pragma comment(linker, "/INCLUDE:_tls_used")
#pragma comment(linker, "/INCLUDE:bench_tls_callback_ptr")
#pragma const_seg(".CRT$XLB")
extern "C" const PIMAGE_TLS_CALLBACK bench_tls_callback_ptr = bench_tls_callback;
#pragma const_seg()
#define BENCH_COUNTS_THREADS 1
#endif

namespace {

using clk = std::chrono::steady_clock;

int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(clk::now().time_since_epoch())
      .count();
}

// ---------------------------------------------------------------------------------------------
// Platform probes: CPU time, memory, threads, handles, pinning. Each is read outside the measured
// region (meter::begin/end order them around the clocks), so their own cost never lands in it.
// ---------------------------------------------------------------------------------------------

struct mem_info {
  uint64_t ws = 0;        // resident set / working set
  uint64_t peak_ws = 0;   // its peak over the process's life (VmHWM on Linux)
  uint64_t priv = 0;      // private (Windows commit; Linux RssAnon; macOS none)
  uint64_t peak_priv = 0; // peak commit (Windows only)
  uint64_t heap = 0;      // bytes the C heap has handed out and not had back
  bool has_priv = false, has_peak_priv = false, has_heap = false;
};

#if defined(_WIN32)

uint64_t filetime_100ns(const FILETIME &t) {
  return (static_cast<uint64_t>(t.dwHighDateTime) << 32) | t.dwLowDateTime;
}

// The rate QueryProcessCycleTime counts at: the time-stamp counter's, measured against the
// performance counter as thread_cpu_seconds() measures it (types.cpp), or 0 where the TSC isn't
// invariant or there is none (ARM64).
double cycle_rate() {
#if defined(_M_X64) || defined(_M_IX86)
  int info[4] = {};
  __cpuid(info, static_cast<int>(0x80000000));
  if (static_cast<unsigned>(info[0]) < 0x80000007u)
    return 0.0;
  __cpuid(info, static_cast<int>(0x80000007));
  if ((info[3] & (1 << 8)) == 0)
    return 0.0;
  LARGE_INTEGER freq, q0, q1;
  if (!QueryPerformanceFrequency(&freq) || freq.QuadPart <= 0)
    return 0.0;
  QueryPerformanceCounter(&q0);
  const unsigned long long c0 = __rdtsc();
  Sleep(20);
  QueryPerformanceCounter(&q1);
  const unsigned long long c1 = __rdtsc();
  const double seconds = double(q1.QuadPart - q0.QuadPart) / double(freq.QuadPart);
  return seconds > 0.0 ? double(c1 - c0) / seconds : 0.0;
#else
  return 0.0;
#endif
}

// CPU time of every thread of the process. GetProcessTimes advances at the scheduler tick (about
// 15.6 ms), which most regions here are shorter than, so the process's cycle count is used where
// the TSC rate is known; it includes threads that have already exited.
int64_t process_cpu_ns() {
  static const double rate = cycle_rate();
  if (rate > 0.0) {
    ULONG64 cycles = 0;
    if (QueryProcessCycleTime(GetCurrentProcess(), &cycles))
      return static_cast<int64_t>(double(cycles) / rate * 1e9);
  }
  FILETIME created, exited, kernel, user;
  if (!GetProcessTimes(GetCurrentProcess(), &created, &exited, &kernel, &user))
    return 0;
  return static_cast<int64_t>((filetime_100ns(kernel) + filetime_100ns(user)) * 100);
}

mem_info memory(bool with_heap) {
  mem_info m;
  PROCESS_MEMORY_COUNTERS_EX pmc{};
  pmc.cb = sizeof pmc;
  if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS *>(&pmc),
                           sizeof pmc)) {
    m.ws = pmc.WorkingSetSize;
    m.peak_ws = pmc.PeakWorkingSetSize;
    m.priv = pmc.PrivateUsage;
    m.peak_priv = pmc.PeakPagefileUsage;
    m.has_priv = m.has_peak_priv = true;
  }
  if (with_heap) {
    // The UCRT's malloc allocates from the process heap, so its allocated total is what the
    // program holds, whatever the heap keeps committed for reuse (a working-set delta isn't,
    // after a warmup freed the same amount).
    HEAP_SUMMARY hs{};
    hs.cb = sizeof hs;
    if (HeapSummary(GetProcessHeap(), 0, &hs)) {
      m.heap = hs.cbAllocated;
      m.has_heap = true;
    }
  }
  return m;
}

int thread_count() {
  // One NtQuerySystemInformation(SystemProcessInformation) call lists every process with its
  // thread count in a few ms; a Toolhelp thread snapshot walks every thread on the system and
  // took ~60 ms a call here, which made per-rep sampling dominate a smoke run.
  using query_fn = LONG(WINAPI *)(SYSTEM_INFORMATION_CLASS, PVOID, ULONG, PULONG);
  static const auto query = reinterpret_cast<query_fn>(reinterpret_cast<void *>(
      GetProcAddress(GetModuleHandleW(L"ntdll.dll"), "NtQuerySystemInformation")));
  static std::vector<unsigned char> buf(1 << 20);
  const DWORD self = GetCurrentProcessId();
  if (query) {
    for (int attempt = 0; attempt < 4; ++attempt) {
      ULONG need = 0;
      const LONG st =
          query(SystemProcessInformation, buf.data(), static_cast<ULONG>(buf.size()), &need);
      if (st == 0) {
        for (std::size_t off = 0;;) {
          const auto *p = reinterpret_cast<const SYSTEM_PROCESS_INFORMATION *>(buf.data() + off);
          if (static_cast<DWORD>(reinterpret_cast<uintptr_t>(p->UniqueProcessId)) == self)
            return static_cast<int>(p->NumberOfThreads);
          if (!p->NextEntryOffset)
            break;
          off += p->NextEntryOffset;
        }
        break;
      }
      buf.resize(std::max<std::size_t>(buf.size() * 2, need + (64 << 10)));
    }
  }
  HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
  if (snap == INVALID_HANDLE_VALUE)
    return -1;
  THREADENTRY32 te{};
  te.dwSize = sizeof te;
  int n = 0;
  if (Thread32First(snap, &te)) {
    do {
      if (te.th32OwnerProcessID == self)
        ++n;
      te.dwSize = sizeof te;
    } while (Thread32Next(snap, &te));
  }
  CloseHandle(snap);
  return n;
}

int64_t handle_count() {
  DWORD n = 0;
  return GetProcessHandleCount(GetCurrentProcess(), &n) ? static_cast<int64_t>(n) : -1;
}

#else // POSIX

int64_t process_cpu_ns() {
  timespec ts{};
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0)
    return int64_t(ts.tv_sec) * 1000000000 + ts.tv_nsec;
  rusage ru{};
  if (getrusage(RUSAGE_SELF, &ru) != 0)
    return 0;
  const auto us = [](const timeval &t) { return int64_t(t.tv_sec) * 1000000 + t.tv_usec; };
  return (us(ru.ru_utime) + us(ru.ru_stime)) * 1000;
}

mem_info memory(bool with_heap) {
  mem_info m;
#if defined(__APPLE__)
  mach_task_basic_info info{};
  mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
  if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, reinterpret_cast<task_info_t>(&info),
                &count) == KERN_SUCCESS) {
    m.ws = info.resident_size;
    m.peak_ws = info.resident_size_max;
  }
  if (with_heap) {
    malloc_statistics_t st{};
    malloc_zone_statistics(nullptr, &st);
    m.heap = st.size_in_use;
    m.has_heap = true;
  }
#else
  std::ifstream in("/proc/self/status");
  std::string line;
  const auto kb = [](const std::string &l) {
    return static_cast<uint64_t>(std::strtoull(l.c_str() + l.find(':') + 1, nullptr, 10)) * 1024;
  };
  while (std::getline(in, line)) {
    if (line.rfind("VmRSS:", 0) == 0)
      m.ws = kb(line);
    else if (line.rfind("VmHWM:", 0) == 0)
      m.peak_ws = kb(line);
    else if (line.rfind("RssAnon:", 0) == 0) {
      m.priv = kb(line);
      m.has_priv = true;
    }
  }
#if defined(__GLIBC__) && (__GLIBC__ > 2 || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 33))
  if (with_heap) {
    const struct mallinfo2 mi = mallinfo2();
    m.heap = mi.uordblks + mi.hblkhd;
    m.has_heap = true;
  }
#else
  (void)with_heap;
#endif
#endif
  return m;
}

int thread_count() {
#if defined(__APPLE__)
  thread_act_array_t threads = nullptr;
  mach_msg_type_number_t n = 0;
  if (task_threads(mach_task_self(), &threads, &n) != KERN_SUCCESS)
    return -1;
  for (mach_msg_type_number_t i = 0; i < n; ++i)
    mach_port_deallocate(mach_task_self(), threads[i]);
  vm_deallocate(mach_task_self(), reinterpret_cast<vm_address_t>(threads), n * sizeof(*threads));
  return static_cast<int>(n);
#else
  DIR *d = opendir("/proc/self/task");
  if (!d)
    return -1;
  int n = 0;
  while (dirent *e = readdir(d))
    if (e->d_name[0] != '.')
      ++n;
  closedir(d);
  return n;
#endif
}

// Open file descriptors: the POSIX counterpart of a Windows handle count.
int64_t handle_count() {
#if defined(__APPLE__)
  DIR *d = opendir("/dev/fd");
#else
  DIR *d = opendir("/proc/self/fd");
#endif
  if (!d)
    return -1;
  int64_t n = 0;
  while (dirent *e = readdir(d))
    if (e->d_name[0] != '.')
      ++n;
  closedir(d);
  return n - 1; // the directory stream's own descriptor
}

#endif

int64_t threads_created() {
#if defined(BENCH_COUNTS_THREADS)
  return bench_threads::created.load(std::memory_order_relaxed);
#else
  return -1;
#endif
}

// Start a fresh peak-memory window, where the OS allows it (Linux: VmHWM resets on writing 5 to
// clear_refs). Elsewhere the peak is the process's, and a record says when it may belong to an
// earlier point (peak_inherited).
bool reset_peak_memory() {
#if defined(__linux__)
  std::ofstream f("/proc/self/clear_refs");
  if (!f)
    return false;
  f << "5";
  f.flush();
  return static_cast<bool>(f);
#else
  return false;
#endif
}

// Pinning. Only the driver thread is meant to be pinned: the app's pools, lanes and the threads a
// background kernel starts float, as in a real host. Windows gives a new thread the process's
// affinity, so pinning the driver pins nothing else. Linux gives a new thread its creator's mask,
// so there the bench (a) builds apps, lanes and its own threads under unpinned_scope, (b) widens
// every other thread back to the process's mask when a measured region begins, and (c) leaves the
// driver unpinned for scenarios whose drivers start threads inside the region (spawns_threads),
// where (a) and (b) can't reach; those records carry pin_cpu -1.
struct pin_state {
  int cpu = -1;        // the driver's logical CPU, or -1
  std::string how;     // how it was chosen, or why not
  std::string sibling; // the other logical CPUs on the same core ("" when none or unknown)
#if defined(__linux__)
  bool have_orig = false;
  cpu_set_t orig{}; // the process's mask before pinning
#endif
};

pin_state &pinning() {
  static pin_state p;
  return p;
}

#if defined(__linux__)
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

bool set_thread_mask(pid_t tid, const cpu_set_t &set) {
  return sched_setaffinity(tid, sizeof set, &set) == 0;
}

cpu_set_t single_cpu(int cpu) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  return set;
}
#endif

// Pin the calling (driver) thread. Returns the logical CPU it is pinned to, or -1.
int pin_driver(const std::string &mode) {
  pin_state &ps = pinning();
  std::string &how = ps.how;
  if (mode == "none" || mode.empty()) {
    how = "none";
    return -1;
  }
  int cpu = -1;
  if (mode != "auto") {
    char *end = nullptr;
    cpu = static_cast<int>(std::strtol(mode.c_str(), &end, 10));
    if (!end || *end || cpu < 0) {
      how = "bad --pin value";
      return -1;
    }
  }
#if defined(_WIN32)
  // The CPU set API is Windows 10+; _WIN32_WINNT is 0x0601 repo-wide, so resolve it at run time.
  using get_sets_fn = BOOL(WINAPI *)(PSYSTEM_CPU_SET_INFORMATION, ULONG, PULONG, HANDLE, ULONG);
  using set_sets_fn = BOOL(WINAPI *)(HANDLE, const ULONG *, ULONG);
  HMODULE k32 = GetModuleHandleW(L"kernel32.dll");
  auto get_sets = reinterpret_cast<get_sets_fn>(
      reinterpret_cast<void *>(GetProcAddress(k32, "GetSystemCpuSetInformation")));
  auto set_sets = reinterpret_cast<set_sets_fn>(
      reinterpret_cast<void *>(GetProcAddress(k32, "SetThreadSelectedCpuSets")));
  struct cpu_set {
    ULONG id;
    int lp, core;
    unsigned eff;
  };
  std::vector<cpu_set> sets;
  if (get_sets) {
    ULONG len = 0;
    get_sets(nullptr, 0, &len, GetCurrentProcess(), 0);
    std::vector<unsigned char> buf(len);
    if (len && get_sets(reinterpret_cast<PSYSTEM_CPU_SET_INFORMATION>(buf.data()), len, &len,
                        GetCurrentProcess(), 0)) {
      for (ULONG off = 0; off < len;) {
        auto *e = reinterpret_cast<SYSTEM_CPU_SET_INFORMATION *>(buf.data() + off);
        if (e->Type == CpuSetInformation && e->CpuSet.Group == 0)
          sets.push_back({e->CpuSet.Id, e->CpuSet.LogicalProcessorIndex, e->CpuSet.CoreIndex,
                          e->CpuSet.EfficiencyClass});
        off += e->Size;
      }
    }
  }
  if (cpu < 0) {
    // A performance core: the highest EfficiencyClass. Prefer one off the core logical CPU 0
    // sits on, where most interrupts and DPCs land.
    unsigned best = 0;
    int core0 = -1;
    for (const auto &s : sets) {
      best = std::max(best, s.eff);
      if (s.lp == 0)
        core0 = s.core;
    }
    for (const auto &s : sets)
      if (s.eff == best && s.core != core0) {
        cpu = s.lp;
        break;
      }
    if (cpu < 0)
      for (const auto &s : sets)
        if (s.eff == best) {
          cpu = s.lp;
          break;
        }
    if (cpu < 0) {
      how = "auto: no CPU set information";
      return -1;
    }
  }
  if (cpu >= 64) {
    how = "cpu beyond processor group 0";
    return -1;
  }
  bool ok = SetThreadAffinityMask(GetCurrentThread(), DWORD_PTR(1) << cpu) != 0;
  int core = -1;
  for (const auto &s : sets)
    if (s.lp == cpu) {
      core = s.core;
      if (set_sets)
        ok = set_sets(GetCurrentThread(), &s.id, 1) && ok;
    }
  for (const auto &s : sets)
    if (s.core == core && s.lp != cpu)
      ps.sibling += (ps.sibling.empty() ? "" : ",") + std::to_string(s.lp);
  std::string eff;
  for (const auto &s : sets)
    if (s.lp == cpu)
      eff = " efficiency class " + std::to_string(s.eff);
  how = (mode == "auto" ? "auto:" : "explicit:") + eff + (ok ? "" : " (affinity call failed)");
  ps.cpu = ok ? cpu : -1;
  return ps.cpu;
#elif defined(__linux__)
  if (sched_getaffinity(0, sizeof ps.orig, &ps.orig) != 0) {
    how = "sched_getaffinity failed";
    return -1;
  }
  ps.have_orig = true;
  if (cpu < 0) {
    // Hybrid and big.LITTLE parts report a per-CPU capacity; the biggest is a performance core.
    // x86 hybrid kernels may not, but list their performance cores under cpu_core. Prefer a CPU
    // off the core CPU 0 sits on (its SMT siblings included), where most interrupts land.
    std::vector<int> candidates;
    long best = -1;
    for (int i = 0; i < CPU_SETSIZE; ++i) {
      if (!CPU_ISSET(i, &ps.orig))
        continue;
      std::ifstream f("/sys/devices/system/cpu/cpu" + std::to_string(i) + "/cpu_capacity");
      long c = -1;
      if (!(f >> c))
        continue;
      if (c > best) {
        best = c;
        candidates.clear();
      }
      if (c == best)
        candidates.push_back(i);
    }
    std::string source = "highest cpu_capacity";
    if (candidates.empty()) {
      for (int c : parse_cpu_list(read_first_line("/sys/devices/cpu_core/cpus")))
        if (CPU_ISSET(c, &ps.orig))
          candidates.push_back(c);
      source = "cpu_core (hybrid P-cores)";
    }
    if (candidates.empty()) {
      how = "auto: no cpu_capacity or cpu_core list (not pinned)";
      return -1;
    }
    std::vector<int> core0 = smt_siblings(0);
    if (core0.empty())
      core0 = {0};
    for (int c : candidates)
      if (std::find(core0.begin(), core0.end(), c) == core0.end()) {
        cpu = c;
        break;
      }
    if (cpu < 0)
      cpu = candidates.front();
    how = "auto: " + source;
  } else {
    how = "explicit";
  }
  if (cpu >= CPU_SETSIZE || !set_thread_mask(0, single_cpu(cpu))) {
    how += " (sched_setaffinity failed)";
    return -1;
  }
  for (int c : smt_siblings(cpu))
    if (c != cpu)
      ps.sibling += (ps.sibling.empty() ? "" : ",") + std::to_string(c);
  ps.cpu = cpu;
  return cpu;
#else
  (void)cpu; // macOS has no thread affinity to set
  how = "pinning not supported on this platform";
  return -1;
#endif
}

// While alive, the calling thread runs on the process's whole mask, so threads it starts don't
// inherit the driver's pin (Linux; a no-op elsewhere, where they don't).
struct unpinned_scope {
  bool active = false;
  unpinned_scope() {
#if defined(__linux__)
    const pin_state &ps = pinning();
    if (ps.cpu >= 0 && ps.have_orig)
      active = set_thread_mask(0, ps.orig);
#endif
  }
  ~unpinned_scope() {
#if defined(__linux__)
    if (active)
      set_thread_mask(0, single_cpu(pinning().cpu));
#endif
  }
  unpinned_scope(const unpinned_scope &) = delete;
  unpinned_scope &operator=(const unpinned_scope &) = delete;
};

// Linux: give every thread but the caller the process's whole mask again, undoing the pin that
// threads started from the driver inherited. Called as a region begins, outside its clocks.
void widen_other_threads() {
#if defined(__linux__)
  const pin_state &ps = pinning();
  if (ps.cpu < 0 || !ps.have_orig)
    return;
  const pid_t self = static_cast<pid_t>(syscall(SYS_gettid));
  DIR *d = opendir("/proc/self/task");
  if (!d)
    return;
  while (dirent *e = readdir(d)) {
    if (e->d_name[0] == '.')
      continue;
    const pid_t tid = static_cast<pid_t>(std::atoi(e->d_name));
    if (tid != self)
      set_thread_mask(tid, ps.orig);
  }
  closedir(d);
#endif
}

// For a scenario whose driver starts threads inside the region: drop the driver's pin for its
// points (Linux), so they don't all inherit one CPU. Restored by the destructor.
struct driver_unpinned_for_point {
  bool active = false;
  explicit driver_unpinned_for_point(bool spawns) {
#if defined(__linux__)
    const pin_state &ps = pinning();
    if (spawns && ps.cpu >= 0 && ps.have_orig)
      active = set_thread_mask(0, ps.orig);
#else
    (void)spawns;
#endif
  }
  ~driver_unpinned_for_point() {
#if defined(__linux__)
    if (active)
      set_thread_mask(0, single_cpu(pinning().cpu));
#endif
  }
  driver_unpinned_for_point(const driver_unpinned_for_point &) = delete;
  driver_unpinned_for_point &operator=(const driver_unpinned_for_point &) = delete;
};

std::string raise_priority() {
#if defined(_WIN32)
  if (!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS))
    return "SetPriorityClass failed";
  return "high";
#else
  errno = 0;
  if (setpriority(PRIO_PROCESS, 0, -10) != 0)
    return "setpriority(-10) failed (needs privileges)";
  return "nice -10";
#endif
}

std::string compiler_name() {
#if defined(__clang__)
  return std::string("clang ") + __clang_version__;
#elif defined(_MSC_VER)
  return "msvc " + std::to_string(_MSC_FULL_VER);
#elif defined(__GNUC__)
  return std::string("gcc ") + __VERSION__;
#else
  return "unknown";
#endif
}

// ---------------------------------------------------------------------------------------------
// JSON output
// ---------------------------------------------------------------------------------------------

std::string json_str(const std::string &s) {
  std::string o = "\"";
  for (unsigned char c : s) {
    switch (c) {
    case '"':
      o += "\\\"";
      break;
    case '\\':
      o += "\\\\";
      break;
    case '\n':
      o += "\\n";
      break;
    case '\r':
      o += "\\r";
      break;
    case '\t':
      o += "\\t";
      break;
    default:
      if (c < 0x20) {
        char b[8];
        std::snprintf(b, sizeof b, "\\u%04x", c);
        o += b;
      } else {
        o += static_cast<char>(c);
      }
    }
  }
  return o + "\"";
}

std::string json_num(double v) {
  if (!std::isfinite(v))
    return "null";
  char b[64];
  std::snprintf(b, sizeof b, "%.6g", v);
  return b;
}

// An object built in field order.
struct jobj {
  std::string body;
  jobj &raw(const std::string &k, const std::string &v) {
    if (!body.empty())
      body += ", ";
    body += json_str(k) + ": " + v;
    return *this;
  }
  jobj &i(const std::string &k, int64_t v) { return raw(k, std::to_string(v)); }
  jobj &d(const std::string &k, double v) { return raw(k, json_num(v)); }
  jobj &s(const std::string &k, const std::string &v) { return raw(k, json_str(v)); }
  jobj &b(const std::string &k, bool v) { return raw(k, v ? "true" : "false"); }
  std::string str() const { return "{" + body + "}"; }
};

// An integer in canonical form ("0", "12", "-3"; not "010" or "-0"), which JSON can carry as a
// number as written.
bool is_integer(const std::string &s) {
  if (s.empty())
    return false;
  std::size_t i = s[0] == '-' ? 1 : 0;
  if (i == s.size())
    return false;
  if (s[i] == '0' && (s.size() > i + 1 || i == 1))
    return false;
  for (; i < s.size(); ++i)
    if (s[i] < '0' || s[i] > '9')
      return false;
  return true;
}

// The nearest-rank q-quantile of sorted `v`. The one definition behind the JSON percentiles and
// the stdout line.
int64_t pct_at(const std::vector<int64_t> &v, double q) {
  const std::size_t r = static_cast<std::size_t>(std::ceil(q * double(v.size())));
  return v[std::min(v.size() - 1, r == 0 ? 0 : r - 1)];
}

// p50/p90/p99/max of `v` (nearest rank), in its units.
jobj percentiles(std::vector<int64_t> v) {
  jobj o;
  if (v.empty())
    return o;
  std::sort(v.begin(), v.end());
  o.i("p50", pct_at(v, 0.50))
      .i("p90", pct_at(v, 0.90))
      .i("p99", pct_at(v, 0.99))
      .i("max", v.back());
  o.i("n", static_cast<int64_t>(v.size()));
  return o;
}

// ---------------------------------------------------------------------------------------------
// The in-process sampler (--sample; docs/STATE_EXEC_BENCH.md, "Profiling"). Windows x64 only: WPR,
// xperf and the Visual Studio profiler need elevation, this needs nothing. While a sampled rep's
// measured region runs, a thread of its own suspends the driver every --sample-us, unwinds its
// stack in place (RtlLookupFunctionEntry + RtlVirtualUnwind) into a ring allocated up front, and
// resumes it. Nothing it does while the driver is suspended allocates, locks or prints: the driver
// may hold the heap lock. A second thread resumes the driver if one sample holds it over 50 ms (a
// lock the unwinder needs, held by the suspended driver) and turns sampling off for the region.
// After the region, with the driver running freely, DbgHelp (loaded then and unloaded once the
// rep's files are written) symbolizes the stacks into a folded file (flamegraph.pl, speedscope)
// and a top list. Without --sample none of this exists: no thread, no buffer, no dbghelp.dll, one
// false bool per region.
// ---------------------------------------------------------------------------------------------

struct sampler_opts {
  std::string prefix; // output file prefix; "" = no sampling
  int64_t interval_us = 1000;
  int depth = 64;
  bool all_reps = false;
};

#if defined(_WIN32) && defined(_M_X64) && defined(_MSC_VER)
#define BENCH_SAMPLER 1
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

int64_t qpc_now() {
  LARGE_INTEGER t;
  QueryPerformanceCounter(&t);
  return t.QuadPart;
}

int64_t qpc_freq() {
  static const int64_t f = [] {
    LARGE_INTEGER x;
    QueryPerformanceFrequency(&x);
    return x.QuadPart;
  }();
  return f;
}

// [base, end) of a loaded module, from its PE header; {0, 0} when it isn't loaded.
std::pair<uintptr_t, uintptr_t> module_range(const wchar_t *name) {
  const auto *base = reinterpret_cast<const unsigned char *>(GetModuleHandleW(name));
  if (!base)
    return {0, 0};
  const auto *dos = reinterpret_cast<const IMAGE_DOS_HEADER *>(base);
  const auto *nt = reinterpret_cast<const IMAGE_NT_HEADERS *>(base + dos->e_lfanew);
  return {uintptr_t(base), uintptr_t(base) + nt->OptionalHeader.SizeOfImage};
}

// Unwind *ctx (the suspended driver's context) in place, writing up to `depth` code addresses to
// out. RtlVirtualUnwind where the function has unwind info; the top frame alone may be a leaf
// without any, whose return address is at [RSP]. A deeper frame without unwind info is not a leaf,
// so the walk stops there (*stopped) instead of following a guessed PC, which would also send
// RtlLookupFunctionEntry down its slow path (the dynamic function tables, under a lock). Its own
// reads stay in [the suspended RSP, hi), off the stack's guard page. It stops at a null RIP, at an
// RSP that fails to grow, and as soon as *busy no longer holds gen (the watchdog resumed the
// driver, so the stack is live again). Allocates nothing and takes no lock of its own; a stack torn
// under it can still fault, and __try turns that into a short stack, so this function holds no
// C++ objects.
int unwind_into(CONTEXT *ctx, uint64_t *out, int depth, uint64_t hi,
                const std::atomic<uint64_t> *busy, uint64_t gen, bool *truncated, bool *stopped) {
  int n = 0;
  *truncated = *stopped = false;
  const DWORD64 lo = ctx->Rsp;
  UNWIND_HISTORY_TABLE history; // caches function entries across the frames of this one sample
  std::memset(&history, 0, sizeof history);
  __try {
    for (;;) {
      if (busy->load(std::memory_order_acquire) != gen)
        break;
      const DWORD64 pc = ctx->Rip;
      if (pc < 0x10000)
        break;
      if (n == depth) {
        *truncated = true;
        break;
      }
      out[n++] = pc;
      const DWORD64 sp = ctx->Rsp;
      DWORD64 image = 0;
      PRUNTIME_FUNCTION f = RtlLookupFunctionEntry(pc, &image, &history);
      if (f) {
        PVOID data = nullptr;
        DWORD64 frame = 0;
        RtlVirtualUnwind(UNW_FLAG_NHANDLER, image, pc, f, ctx, &data, &frame, nullptr);
      } else if (n == 1) {
        if (sp < lo || sp + 8 > hi)
          break;
        ctx->Rip = *reinterpret_cast<const DWORD64 *>(sp);
        ctx->Rsp = sp + 8;
      } else {
        *stopped = true;
        break;
      }
      if (ctx->Rsp <= sp || ctx->Rsp > hi)
        break;
    }
  } __except (EXCEPTION_EXECUTE_HANDLER) {
  }
  return n;
}

// The start of the function holding pc, from its image's .pdata (following chained and indirect
// entries back to the primary one), or 0 for a leaf without unwind info.
uint64_t function_start(uint64_t pc) {
  DWORD64 image = 0;
  const RUNTIME_FUNCTION *f = RtlLookupFunctionEntry(pc, &image, nullptr);
  for (int i = 0; f && i < 16; ++i) {
    if (f->UnwindData & 1) { // indirect: it points at another entry
      f = reinterpret_cast<const RUNTIME_FUNCTION *>(image + (f->UnwindData & ~DWORD(1)));
      continue;
    }
    const auto *ui = reinterpret_cast<const unsigned char *>(image + f->UnwindData);
    if (!((ui[0] >> 3) & UNW_FLAG_CHAININFO))
      return image + f->BeginAddress;
    const unsigned codes = (ui[2] + 1u) & ~1u; // the chained entry follows the (even) codes
    f = reinterpret_cast<const RUNTIME_FUNCTION *>(ui + 4 + 2 * codes);
  }
  return 0;
}

// Whether a system-call stub blocks (a wait, a sleep, a completion-port dequeue) rather than
// returning once its work is done (NtQueryInformationThread, NtAllocateVirtualMemory,
// NtYieldExecution, ...). name is the stub's export: Nt/Zw in ntdll, NtUser/NtGdi in win32u.
bool blocking_stub(std::string name) {
  if (name.compare(0, 2, "Nt") == 0 || name.compare(0, 2, "Zw") == 0)
    name.erase(0, 2);
  if (name.compare(0, 4, "User") == 0)
    name.erase(0, 4);
  for (const char *w :
       {"Wait", "SignalAndWait", "DelayExecution", "RemoveIoCompletion", "ReplyWait", "MsgWait"})
    if (name.compare(0, std::strlen(w), w) == 0)
      return true;
  return false;
}

// The name of the last export at or below addr in the module at base, from its export directory
// (no DbgHelp): ntdll and win32u export every system-call stub. Zw and Nt share an address; either
// name may come back.
std::string export_below(uintptr_t base, uintptr_t addr) {
  if (!base)
    return {};
  const auto *b = reinterpret_cast<const unsigned char *>(base);
  const auto *nt = reinterpret_cast<const IMAGE_NT_HEADERS *>(
      b + reinterpret_cast<const IMAGE_DOS_HEADER *>(b)->e_lfanew);
  const IMAGE_DATA_DIRECTORY &d = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXPORT];
  if (!d.VirtualAddress)
    return {};
  const auto *ed = reinterpret_cast<const IMAGE_EXPORT_DIRECTORY *>(b + d.VirtualAddress);
  const auto *names = reinterpret_cast<const DWORD *>(b + ed->AddressOfNames);
  const auto *ords = reinterpret_cast<const WORD *>(b + ed->AddressOfNameOrdinals);
  const auto *funcs = reinterpret_cast<const DWORD *>(b + ed->AddressOfFunctions);
  const uintptr_t rva = addr - base;
  DWORD best = 0;
  const char *name = "";
  for (DWORD i = 0; i < ed->NumberOfNames; ++i)
    if (const DWORD f = funcs[ords[i]]; f <= rva && f >= best) {
      best = f;
      name = reinterpret_cast<const char *>(b + names[i]);
    }
  return name;
}

std::string narrow(const std::wstring &w) {
  if (w.empty())
    return {};
  const int n =
      WideCharToMultiByte(CP_UTF8, 0, w.data(), int(w.size()), nullptr, 0, nullptr, nullptr);
  std::string s(static_cast<std::size_t>(n), '\0');
  WideCharToMultiByte(CP_UTF8, 0, w.data(), int(w.size()), s.data(), n, nullptr, nullptr);
  return s;
}

std::wstring module_path(HMODULE m) {
  wchar_t buf[MAX_PATH * 2];
  const DWORD n = GetModuleFileNameW(m, buf, DWORD(std::size(buf)));
  return std::wstring(buf, n);
}

std::wstring dir_of(const std::wstring &path) {
  const auto slash = path.find_last_of(L"\\/");
  return slash == std::wstring::npos ? std::wstring() : path.substr(0, slash);
}

// A symbol as the folded files show it: without "cvc::state_exec::", with long template argument
// lists cut to <...> and std::string spelled so, and with no ';' (the folded format's separator).
std::string short_symbol(std::string s) {
  const auto erase_all = [&s](const std::string &what, const std::string &with) {
    for (std::size_t p; (p = s.find(what)) != std::string::npos;)
      s.replace(p, what.size(), with);
  };
  erase_all("cvc::state_exec::", "");
  erase_all("`anonymous namespace'::", "");
  erase_all("std::basic_string<char,std::char_traits<char>,std::allocator<char> >", "std::string");
  const auto ends_with = [](const std::string &a, const char *b) {
    const std::size_t n = std::strlen(b);
    return a.size() >= n && a.compare(a.size() - n, n, b) == 0;
  };
  std::string out;
  for (std::size_t i = 0; i < s.size();) {
    if (s[i] != '<' || ends_with(out, "operator") || ends_with(out, "operator<")) {
      out += s[i] == ';' ? ':' : s[i];
      ++i;
      continue;
    }
    int depth = 0;
    std::size_t j = i;
    for (; j < s.size(); ++j) {
      depth += s[j] == '<' ? 1 : s[j] == '>' ? -1 : 0;
      if (depth == 0)
        break;
    }
    if (j >= s.size()) { // unbalanced (DbgHelp cut a long name short)
      out += "<...>";
      break;
    }
    const std::string inner = s.substr(i + 1, j - i - 1);
    out += inner.size() > 24 && inner.compare(0, 7, "lambda_") != 0
               ? std::string("<...>")
               : "<" + short_symbol(inner) + ">";
    i = j + 1;
  }
  return out;
}

// The DbgHelp entry points the sampler uses, resolved at run time: a load-time import would load
// dbghelp.dll into every run, sampled or not.
struct dbghelp_api {
  HMODULE dll = LoadLibraryW(L"dbghelp.dll");
  decltype(&SymGetOptions) get_options = nullptr;
  decltype(&SymSetOptions) set_options = nullptr;
  decltype(&SymInitializeW) initialize = nullptr;
  decltype(&SymCleanup) cleanup = nullptr;
  decltype(&SymGetModuleBase64) module_base = nullptr;
  decltype(&SymFromAddr) from_addr = nullptr;
  decltype(&SymGetLineFromAddr64) line_from_addr = nullptr;
  dbghelp_api() {
    if (!dll)
      return;
    load(get_options, "SymGetOptions");
    load(set_options, "SymSetOptions");
    load(initialize, "SymInitializeW");
    load(cleanup, "SymCleanup");
    load(module_base, "SymGetModuleBase64");
    load(from_addr, "SymFromAddr");
    load(line_from_addr, "SymGetLineFromAddr64");
  }
  dbghelp_api(const dbghelp_api &) = delete;
  dbghelp_api &operator=(const dbghelp_api &) = delete;
  ~dbghelp_api() {
    if (dll)
      FreeLibrary(dll);
  }
  bool ok() const {
    return get_options && set_options && initialize && cleanup && module_base && from_addr &&
           line_from_addr;
  }

private:
  template <class F> void load(F &fn, const char *name) {
    fn = reinterpret_cast<F>(reinterpret_cast<void *>(GetProcAddress(dll, name)));
  }
};
#endif

class sampler {
public:
  sampler() = default;
  sampler(const sampler &) = delete;
  sampler &operator=(const sampler &) = delete;
  ~sampler() {
#if defined(BENCH_SAMPLER)
    stop_threads();
#endif
  }

  bool on() const { return on_; }
  const sampler_opts &opts() const { return o_; }

  // From main on the driver thread, before it is pinned (the threads start floating; avoid_cpus
  // then keeps them off the driver's CPU). Returns the header line, or why sampling stays off.
  std::string start(const sampler_opts &o) {
    o_ = o;
#if defined(BENCH_SAMPLER)
    // 4 Mi frame slots (32 MB at most) for at most 1 Mi samples, faulted in now so no region's
    // working set pays for them.
    cap_ = std::clamp<int64_t>((int64_t(1) << 22) / o.depth, 1024, int64_t(1) << 20);
    const std::size_t fbytes = std::size_t(cap_) * std::size_t(o.depth) * sizeof(uint64_t);
    frames_ = static_cast<uint64_t *>(
        VirtualAlloc(nullptr, fbytes, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    hdrs_ = static_cast<hdr *>(VirtualAlloc(nullptr, std::size_t(cap_) * sizeof(hdr),
                                            MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    if (!frames_ || !hdrs_)
      return "--sample: cannot allocate the sample ring; sampling off";
    std::memset(frames_, 0, fbytes);
    std::memset(hdrs_, 0, std::size_t(cap_) * sizeof(hdr));
    if (!open_driver())
      return "--sample: OpenThread on the driver failed; sampling off";
    ntdll_ = module_range(L"ntdll.dll");
    win32u_ = module_range(L"win32u.dll");
    interval_ticks_ = std::max<int64_t>(1, o.interval_us * qpc_freq() / 1000000);
    limit_ticks_ = qpc_freq() / 20; // 50 ms
    go_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    go_wd_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    idle_ = CreateEventW(nullptr, FALSE, FALSE, nullptr);
    if (!go_ || !go_wd_ || !idle_)
      return "--sample: CreateEvent failed; sampling off";
    sampler_th_ = std::thread([this] { sample_main(); });
    WaitForSingleObject(idle_, INFINITE); // calibrated
    watch_th_ = std::thread([this] { watch_main(); });
    on_ = true;
    char b[512];
    std::snprintf(b, sizeof b,
                  "sampling the driver every %lld us (%s: calibrated mean %.0f us%s%s), depth %d, "
                  "ring %lld samples, %s, to ",
                  static_cast<long long>(o.interval_us), timer_name(), calibrated_us_,
                  rejected_.empty() ? "" : "; ", rejected_.c_str(), o.depth,
                  static_cast<long long>(cap_),
                  o.all_reps ? "every measured rep" : "the first measured rep");
    return b + o.prefix + ".*";
#else
    return "--sample is Windows x64 only (an in-process SuspendThread/RtlVirtualUnwind sampler); "
           "on Linux use perf (perf record -F 999 -g --call-graph dwarf -- state_exec_bench ...). "
           "Continuing unsampled";
#endif
  }

  // From main once the driver is pinned to cpu (-1: not pinned): moves both threads off it and its
  // SMT siblings, so the time-critical sampler (spinning, with the QPC fallback) never takes the
  // driver's core. Returns what to add to the header line.
  std::string avoid_cpus(int cpu, const std::string &siblings) {
#if defined(BENCH_SAMPLER)
    if (!on_ || cpu < 0 || cpu >= 64)
      return "";
    DWORD_PTR proc = 0, sys = 0;
    if (!GetProcessAffinityMask(GetCurrentProcess(), &proc, &sys))
      return "";
    DWORD_PTR mask = proc & ~(DWORD_PTR(1) << cpu);
    for (const char *p = siblings.c_str(); *p;) {
      char *end = nullptr;
      const long c = std::strtol(p, &end, 10);
      if (end == p)
        break;
      if (c >= 0 && c < 64)
        mask &= ~(DWORD_PTR(1) << c);
      p = *end == ',' ? end + 1 : end;
    }
    if (!mask)
      return "; the sampler shares the driver's CPU (no other is available)";
    const bool ok = SetThreadAffinityMask(sampler_th_.native_handle(), mask) != 0 &&
                    SetThreadAffinityMask(watch_th_.native_handle(), mask) != 0;
    if (!ok)
      return "; could not keep the sampler off the driver's CPU";
    return siblings.empty() ? "; sampler kept off the driver's CPU"
                            : "; sampler kept off the driver's CPU and its SMT sibling";
#else
    (void)cpu;
    (void)siblings;
    return "";
#endif
  }

  // Whether the next measured region is sampled; arming clears the previous samples.
  void arm(bool a) {
    armed_ = a && on_;
#if defined(BENCH_SAMPLER)
    if (armed_) {
      kept_ = failed_ = regions_ = region_ticks_ = 0;
      held_n_ = held_ticks_ = held_max_ = 0;
      tripped_ = false;
      warning_.clear();
      win32u_ = module_range(L"win32u.dll"); // it may have loaded since the last rep
      if (!open_driver()) {
        warning_ = "OpenThread on the driver failed";
        armed_ = false;
      }
    }
#endif
  }

  // The meter calls these: begin just before its clocks start, stop right after they stop, join
  // after its last probe (it waits for the sampler to go idle).
  void region_begin() {
    if (!armed_)
      return;
#if defined(BENCH_SAMPLER)
    if (GetCurrentThreadId() != driver_tid_) {
      warning_ = "the measured region began on another thread than the rep";
      return;
    }
    stack_hi_ = reinterpret_cast<uintptr_t>(reinterpret_cast<NT_TIB *>(NtCurrentTeb())->StackBase);
    busy_ = 0;
    ++regions_;
    joining_ = true;
    // The timeBeginPeriod(1) fallback raises the system timer rate for the region only: on for
    // the whole run, it would make every rep's 1 ms sleeps last 1 ms.
    if (timer_ == timer_kind::sleep && begin_period_)
      period_on_ = begin_period_(1) == 0;
    region_t0_ = qpc_now();
    sampling_ = true;
    active_ = true;
    SetEvent(go_);
    SetEvent(go_wd_);
#endif
  }
  void region_stop() {
#if defined(BENCH_SAMPLER)
    if (active_) {
      active_ = false;
      region_ticks_ += qpc_now() - region_t0_;
    }
#endif
  }
  void region_join() {
#if defined(BENCH_SAMPLER)
    if (!joining_)
      return;
    joining_ = false;
    if (WaitForSingleObject(idle_, 5000) != WAIT_OBJECT_0) {
      warning_ = "the sampler thread did not go idle within 5 s; sampling off for the run";
      stuck_ = true;
      on_ = armed_ = false;
    }
    if (period_on_) {
      end_period_(1);
      period_on_ = false;
    }
#endif
  }

  // After a sampled rep, on the driver: copy its samples out of the ring and return the record's
  // sample fields. finish() writes the files at the end of the run: DbgHelp loaded here would keep
  // its PDBs (over 100 MB with cvc.pdb) and their handles in every later record's figures, and in
  // the per-process peaks for good.
  jobj write(const std::string &stem, const std::string &title) {
    jobj j;
    j.b("sampled", true);
#if defined(BENCH_SAMPLER)
    if (!regions_ || stuck_) { // stuck: the sampler may still be writing the ring
      j.s("sample_warning", warning_.empty() ? "no measured region ran" : warning_);
      return j;
    }
    pending_rep p;
    p.stem = stem;
    p.title = title;
    const int64_t kept = std::min(kept_, cap_);
    const int64_t first = kept_ > cap_ ? kept_ % cap_ : 0;
    p.hdrs.reserve(std::size_t(kept));
    int64_t t_first = 0, t_last = 0;
    for (int64_t k = 0; k < kept; ++k) {
      const int64_t slot = (first + k) % cap_;
      hdr h = hdrs_[slot];
      const uint64_t *f = frames_ + slot * o_.depth;
      if (k == 0)
        t_first = h.qpc;
      t_last = h.qpc;
      if (h.flags & f_syscall) {
        // A wait or another system call, by the stub's export name (no DbgHelp needed).
        auto it = stub_blocks_.find(f[0]);
        if (it == stub_blocks_.end()) {
          const auto &m = f[0] >= ntdll_.first && f[0] < ntdll_.second ? ntdll_ : win32u_;
          it = stub_blocks_.emplace(f[0], blocking_stub(export_below(m.first, f[0]))).first;
        }
        if (it->second) {
          h.flags |= f_wait;
          ++p.waiting;
        } else {
          ++p.in_syscall;
        }
      }
      p.truncated += h.flags & f_truncated ? 1 : 0;
      p.stopped += h.flags & f_stopped ? 1 : 0;
      p.frames.insert(p.frames.end(), f, f + h.n);
      p.hdrs.push_back(h);
    }
    const double tick_us = 1e6 / double(qpc_freq());
    p.kept = kept;
    p.mean_us = kept > 1 ? double(t_last - t_first) * tick_us / double(kept - 1) : 0.0;
    p.region_ms = double(region_ticks_) * tick_us / 1e3;
    p.held_mean_us = held_n_ ? double(held_ticks_) * tick_us / double(held_n_) : 0.0;
    p.held_max_us = double(held_max_) * tick_us;
    p.failed = failed_;
    p.overwritten = kept_ > cap_ ? kept_ - cap_ : 0;
    if (tripped_) {
      char b[200];
      std::snprintf(b, sizeof b,
                    "a sample held the driver suspended over 50 ms (%.1f ms); the driver was "
                    "resumed and sampling stopped for the rest of the region",
                    double(trip_ticks_.load()) * tick_us / 1e3);
      warning_ = b;
    }
    p.warning = warning_;
    j.s("sample_file", stem + ".folded").s("sample_top_file", stem + ".top.txt");
    j.i("samples", kept).i("samples_waiting", p.waiting).i("samples_syscall", p.in_syscall);
    j.d("sample_interval_us", p.mean_us);
    j.s("sample_timer", timer_name()).i("sample_depth", o_.depth);
    j.d("sample_held_us_mean", p.held_mean_us).d("sample_held_us_max", p.held_max_us);
    if (p.truncated)
      j.i("samples_truncated", p.truncated);
    if (p.stopped)
      j.i("samples_unwind_stopped", p.stopped);
    if (p.failed)
      j.i("samples_failed", p.failed);
    if (p.overwritten)
      j.i("samples_overwritten", p.overwritten);
    if (!warning_.empty()) {
      j.s("sample_warning", warning_);
      std::fprintf(stderr, "state_exec_bench: warning: sampling: %s\n", warning_.c_str());
    }
    std::printf("    sampled: %lld samples, mean interval %.0f us, driver held %.1f us mean / %.0f "
                "us max per sample, %.1f%% waiting -> %s.folded (written at the end of the run)\n",
                static_cast<long long>(kept), p.mean_us, p.held_mean_us, p.held_max_us,
                kept ? 100.0 * double(p.waiting) / double(kept) : 0.0, stem.c_str());
    pending_.push_back(std::move(p));
#else
    (void)stem;
    (void)title;
#endif
    return j;
  }

  // At the end of the run, every point measured: symbolize the sampled reps' stacks and write
  // their <stem>.folded and <stem>.top.txt.
  void finish() {
#if defined(BENCH_SAMPLER)
    if (pending_.empty())
      return;
    symbols sy;
    if (!sy.error.empty())
      std::fprintf(stderr, "state_exec_bench: warning: sampling: %s\n", sy.error.c_str());
    int written = 0;
    for (const pending_rep &p : pending_)
      written += write_files(sy, p) ? 1 : 0;
    std::printf("[state_exec_bench] wrote the sample files of %d sampled rep%s, %s.*\n", written,
                written == 1 ? "" : "s", o_.prefix.c_str());
    if (written < int(pending_.size()))
      std::fprintf(stderr,
                   "state_exec_bench: warning: sampling: %d sampled reps' files could not "
                   "be written\n",
                   int(pending_.size()) - written);
    pending_.clear();
#endif
  }

private:
  bool on_ = false, armed_ = false;
  sampler_opts o_;
#if defined(BENCH_SAMPLER)
  struct hdr {
    int64_t qpc;
    uint32_t n, flags;
  };
  // f_wait: a syscall stub that blocks, set by write() from the stub's export name.
  enum : uint32_t { f_syscall = 1, f_truncated = 2, f_stopped = 4, f_wait = 8 };
  enum class timer_kind { hr, sleep, spin };

  struct sym_entry {
    std::string label, module; // label: as shown; module: its file name
    std::string name;          // the short symbol alone (a syscall stub's export name)
  };
  // DbgHelp, on the driver thread between regions only (DbgHelp is single-threaded). One per
  // write(): its destructor unloads the PDBs and dbghelp.dll.
  struct symbols {
    dbghelp_api api;
    HANDLE proc = GetCurrentProcess();
    bool ok = false;
    std::string error; // why frames are named module+offset, if they are
    std::unordered_map<uint64_t, int> by_addr, line_by_addr;
    std::vector<sym_entry> funcs;
    std::map<std::string, int> by_key;     // module + '\n' + full name -> funcs index
    std::map<std::string, int> label_uses; // how many functions show a short label
    std::vector<std::string> lines;
    std::map<std::string, int> by_line;
    std::unordered_map<uint64_t, std::string> modules; // base -> file name
    std::wstring exe_dir;
    std::string exe_name;

    symbols() {
      const std::wstring exe = module_path(nullptr);
      exe_dir = dir_of(exe);
      exe_name = narrow(exe.substr(exe_dir.size() + (exe_dir.empty() ? 0 : 1)));
      if (!api.ok()) {
        error = "cannot load dbghelp.dll; frames are named module+offset";
        return;
      }
      api.set_options(api.get_options() | SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS |
                      SYMOPT_LOAD_LINES);
      // The exe's directory and the directory of the cvc.dll the loader found on PATH.
      std::wstring path = exe_dir;
      std::wstring cvc;
      if (HMODULE m = GetModuleHandleW(L"cvc.dll"))
        cvc = module_path(m);
      else {
        wchar_t buf[MAX_PATH * 2];
        const DWORD n =
            SearchPathW(nullptr, L"cvc.dll", nullptr, DWORD(std::size(buf)), buf, nullptr);
        if (n && n < std::size(buf))
          cvc.assign(buf, n);
      }
      if (!cvc.empty() && _wcsicmp(dir_of(cvc).c_str(), exe_dir.c_str()) != 0)
        path += L";" + dir_of(cvc);
      // An explicit search path replaces DbgHelp's default, so keep the user's symbol paths (a
      // symbol server there names ntdll's internals instead of their nearest export).
      for (const wchar_t *var : {L"_NT_SYMBOL_PATH", L"_NT_ALT_SYMBOL_PATH"}) {
        wchar_t buf[4096];
        const DWORD n = GetEnvironmentVariableW(var, buf, DWORD(std::size(buf)));
        if (n && n < std::size(buf))
          path += L";" + std::wstring(buf, n);
      }
      ok = api.initialize(proc, path.c_str(), TRUE) != FALSE;
      if (!ok)
        error = "SymInitialize failed (error " + std::to_string(GetLastError()) +
                "); frames are named module+offset";
    }
    ~symbols() {
      if (ok)
        api.cleanup(proc);
    }
    symbols(const symbols &) = delete;
    symbols &operator=(const symbols &) = delete;

    // One entry per (module, full name); a short label that two of them share gets " #2", ... on
    // the later ones, so overloads and <...> instantiations stay apart.
    int intern(const std::string &key, std::string label, const std::string &module,
               const std::string &name) {
      if (const auto it = by_key.find(key); it != by_key.end())
        return it->second;
      if (const int uses = ++label_uses[label]; uses > 1)
        label += " #" + std::to_string(uses);
      const int id = int(funcs.size());
      funcs.push_back({label, module, name});
      by_key.emplace(key, id);
      return id;
    }
    int synthetic(const std::string &label, const std::string &module) {
      return intern("\n" + label, label, module, label);
    }
    std::string module(uint64_t addr, uint64_t *base) {
      *base = ok ? api.module_base(proc, addr) : 0;
      if (!*base) {
        HMODULE m = nullptr;
        if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                   GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                               reinterpret_cast<LPCWSTR>(addr), &m))
          *base = reinterpret_cast<uint64_t>(m);
      }
      if (!*base)
        return "?";
      auto it = modules.find(*base);
      if (it == modules.end()) {
        std::string name = narrow(module_path(reinterpret_cast<HMODULE>(*base)));
        name = name.substr(name.find_last_of("\\/") + 1);
        it = modules.emplace(*base, name.empty() ? "?" : name).first;
      }
      return it->second;
    }
    int func(uint64_t addr) {
      if (const auto it = by_addr.find(addr); it != by_addr.end())
        return it->second;
      alignas(SYMBOL_INFO) char buf[sizeof(SYMBOL_INFO) + 1024];
      auto *si = reinterpret_cast<SYMBOL_INFO *>(buf);
      std::memset(si, 0, sizeof(SYMBOL_INFO));
      si->SizeOfStruct = sizeof(SYMBOL_INFO);
      si->MaxNameLen = 1024;
      DWORD64 disp = 0;
      uint64_t base = 0;
      const std::string mod = module(addr, &base);
      std::string key, label, name;
      char b[96];
      if (ok && api.from_addr(proc, addr, &disp, si)) {
        key = std::string(si->Name, std::min<ULONG>(si->NameLen, 1023));
        name = short_symbol(key);
        if (si->Flags & SYMFLAG_EXPORT) {
          // No PDB, so this is the nearest export below addr. When .pdata puts the start of the
          // function holding addr elsewhere, it is an internal function: name it by its start.
          const uint64_t start = function_start(addr);
          if (start > si->Address) {
            std::snprintf(b, sizeof b, "+0x%llx",
                          static_cast<unsigned long long>(start - si->Address));
            name += b;
          } else if (start && start < si->Address) {
            std::snprintf(b, sizeof b, "fn+0x%llx", static_cast<unsigned long long>(start - base));
            name = b;
          }
          key = name;
        }
        // System and runtime modules carry their name, the bench's and cvc's code doesn't.
        label =
            _stricmp(mod.c_str(), "cvc.dll") == 0 || _stricmp(mod.c_str(), exe_name.c_str()) == 0
                ? name
                : mod.substr(0, mod.find_last_of('.')) + "!" + name;
      } else {
        if (base)
          std::snprintf(b, sizeof b, "%s+0x%llx", mod.c_str(),
                        static_cast<unsigned long long>(addr - base));
        else
          std::snprintf(b, sizeof b, "0x%llx", static_cast<unsigned long long>(addr));
        key = label = name = b;
      }
      const int id = intern(mod + "\n" + key, label, mod, name);
      by_addr.emplace(addr, id);
      return id;
    }
    // "dir/file.cpp:123" for addr, or -1.
    int line(uint64_t addr) {
      if (const auto it = line_by_addr.find(addr); it != line_by_addr.end())
        return it->second;
      IMAGEHLP_LINE64 ln{};
      ln.SizeOfStruct = sizeof ln;
      DWORD disp = 0;
      int id = -1;
      if (ok && api.line_from_addr(proc, addr, &disp, &ln) && ln.FileName) {
        std::string file = ln.FileName;
        std::replace(file.begin(), file.end(), '\\', '/');
        const auto last = file.find_last_of('/');
        if (last != std::string::npos && last > 0) {
          const auto prev = file.find_last_of('/', last - 1);
          if (prev != std::string::npos)
            file = file.substr(prev + 1);
        }
        const std::string key = file + ":" + std::to_string(ln.LineNumber);
        const auto [it, fresh] = by_line.emplace(key, int(lines.size()));
        if (fresh)
          lines.push_back(key);
        id = it->second;
      }
      line_by_addr.emplace(addr, id);
      return id;
    }
  };

  // One sampled rep's stacks, copied out of the ring by write(), and its figures.
  struct pending_rep {
    std::string stem, title, warning;
    std::vector<hdr> hdrs;
    std::vector<uint64_t> frames; // hdrs[i].n frames each, leaf first
    int64_t kept = 0, waiting = 0, in_syscall = 0, truncated = 0, stopped = 0, failed = 0;
    int64_t overwritten = 0;
    double mean_us = 0, region_ms = 0, held_mean_us = 0, held_max_us = 0;
  };

  // Symbolize p and write <stem>.folded and <stem>.top.txt. Returns whether both were written.
  bool write_files(symbols &sy, const pending_rep &p) {
    std::map<std::string, int64_t> folded;
    std::unordered_map<int, int64_t> self, incl;
    std::map<std::pair<int, int>, int64_t> line_self; // (line, function) -> samples
    std::map<std::string, int64_t> waits_by_caller;
    std::vector<int> ids;
    std::size_t at = 0;
    for (const hdr &h : p.hdrs) {
      const uint64_t *f = p.frames.data() + at;
      at += h.n;
      ids.clear();
      for (uint32_t i = 0; i < h.n; ++i) // a return address points past its call: look up addr-1
        ids.push_back(sy.func(i == 0 ? f[i] : f[i] - 1));
      const bool wait = h.flags & f_wait, call = (h.flags & f_syscall) && !wait;
      std::string stack = h.flags & f_truncated ? "[truncated]"
                          : h.flags & f_stopped ? "[unwind stopped]"
                                                : "";
      for (std::size_t i = ids.size(); i-- > 0;)
        stack += (stack.empty() ? "" : ";") + sy.funcs[std::size_t(ids[i])].label;
      stack += wait ? ";[waiting]" : call ? ";[syscall]" : "";
      ++folded[stack];
      const sym_entry leaf = sy.funcs[std::size_t(ids[0])];
      if (wait) {
        // Who waited: the first frame outside the system's wait wrappers.
        std::string caller = "?";
        for (int id : ids) {
          const std::string &m = sy.funcs[std::size_t(id)].module;
          if (_stricmp(m.c_str(), "ntdll.dll") != 0 && _stricmp(m.c_str(), "KERNELBASE.dll") != 0 &&
              _stricmp(m.c_str(), "KERNEL32.DLL") != 0 && _stricmp(m.c_str(), "win32u.dll") != 0) {
            caller = sy.funcs[std::size_t(id)].label;
            break;
          }
        }
        ++waits_by_caller[caller];
        ++self[sy.synthetic("[waiting] " + leaf.label, leaf.module)];
      } else if (call) {
        ++self[sy.synthetic("[syscall] " + leaf.label, leaf.module)];
      } else {
        ++self[ids[0]];
      }
      const int l = sy.line(f[0]);
      if (l >= 0)
        ++line_self[{l, ids[0]}];
      std::sort(ids.begin(), ids.end());
      ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
      for (int id : ids)
        ++incl[id];
    }

    const std::string folded_path = p.stem + ".folded", top_path = p.stem + ".top.txt";
    bool wrote = false;
    if (std::FILE *out = std::fopen(folded_path.c_str(), "wb")) {
      for (const auto &[stack, n] : folded)
        std::fprintf(out, "%s %lld\n", stack.c_str(), static_cast<long long>(n));
      wrote = std::fclose(out) == 0;
    }
    std::FILE *out = std::fopen(top_path.c_str(), "wb");
    if (!out)
      return false;
    const int64_t kept = p.kept;
    const auto pct = [kept](int64_t n) { return kept ? 100.0 * double(n) / double(kept) : 0.0; };
    std::fprintf(out, "%s\n", p.title.c_str());
    std::fprintf(out,
                 "samples %lld over %.1f ms of region, mean interval %.1f us (asked %lld us, %s)\n",
                 static_cast<long long>(kept), p.region_ms, p.mean_us,
                 static_cast<long long>(o_.interval_us), timer_name());
    std::fprintf(out, "driver held suspended per sample: mean %.1f us, max %.1f us\n",
                 p.held_mean_us, p.held_max_us);
    const int64_t running = kept - p.waiting - p.in_syscall;
    std::fprintf(out,
                 "running %lld (%.1f%%), blocked in a wait %lld (%.1f%%), in another system call "
                 "%lld (%.1f%%)\n",
                 static_cast<long long>(running), pct(running), static_cast<long long>(p.waiting),
                 pct(p.waiting), static_cast<long long>(p.in_syscall), pct(p.in_syscall));
    if (!waits_by_caller.empty()) {
      std::vector<std::pair<int64_t, std::string>> v;
      for (const auto &[caller, n] : waits_by_caller)
        v.push_back({-n, caller});
      std::sort(v.begin(), v.end());
      std::fprintf(out, "waits by caller:");
      for (std::size_t i = 0; i < v.size() && i < 10; ++i)
        std::fprintf(out, "%s %s %lld", i ? "," : "", v[i].second.c_str(),
                     static_cast<long long>(-v[i].first));
      std::fprintf(out, "\n");
    }
    if (p.truncated)
      std::fprintf(out, "stacks cut at depth %d: %lld (inclusive counts miss their outer frames)\n",
                   o_.depth, static_cast<long long>(p.truncated));
    if (p.stopped)
      std::fprintf(out,
                   "stacks stopped at a frame without unwind info: %lld (their outer frames are "
                   "missing)\n",
                   static_cast<long long>(p.stopped));
    if (p.failed)
      std::fprintf(out, "failed samples (SuspendThread/GetThreadContext): %lld\n",
                   static_cast<long long>(p.failed));
    if (p.overwritten)
      std::fprintf(out, "ring wrapped: the first %lld samples were overwritten\n",
                   static_cast<long long>(p.overwritten));
    if (!p.warning.empty())
      std::fprintf(out, "WARNING: %s\n", p.warning.c_str());
    if (!sy.error.empty())
      std::fprintf(out, "WARNING: %s\n", sy.error.c_str());
    const auto table = [&](const char *heading, const std::unordered_map<int, int64_t> &m,
                           std::size_t limit) {
      std::vector<std::pair<int64_t, int>> v;
      for (const auto &[id, n] : m)
        v.push_back({-n, id});
      std::sort(v.begin(), v.end());
      std::fprintf(out, "\n%s\n  %7s %7s  %s\n", heading, "%", "samples", "function [module]");
      for (std::size_t i = 0; i < v.size() && i < limit; ++i) {
        const sym_entry &e = sy.funcs[std::size_t(v[i].second)];
        std::fprintf(out, "  %6.2f%% %7lld  %s [%s]\n", pct(-v[i].first),
                     static_cast<long long>(-v[i].first), e.label.c_str(), e.module.c_str());
      }
    };
    table("top 40 by self samples", self, 40);
    table("top 40 by inclusive samples", incl, 40);
    std::vector<std::pair<int64_t, std::pair<int, int>>> lv;
    for (const auto &[lf, n] : line_self)
      lv.push_back({-n, lf});
    std::sort(lv.begin(), lv.end());
    std::fprintf(out, "\ntop 20 source lines by self samples\n  %7s %7s  %s\n", "%", "samples",
                 "file:line  function");
    for (std::size_t i = 0; i < lv.size() && i < 20; ++i)
      std::fprintf(out, "  %6.2f%% %7lld  %s  %s\n", pct(-lv[i].first),
                   static_cast<long long>(-lv[i].first),
                   sy.lines[std::size_t(lv[i].second.first)].c_str(),
                   sy.funcs[std::size_t(lv[i].second.second)].label.c_str());
    return std::fclose(out) == 0 && wrote;
  }

  // The driver's handle: opened by start() outside any region (opened inside one, it would show
  // in that rep's handle_count), and again by arm() only if the rep runs on another thread.
  bool open_driver() {
    const DWORD tid = GetCurrentThreadId();
    if (driver_ && tid == driver_tid_)
      return true;
    if (driver_)
      CloseHandle(driver_);
    driver_ = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION,
                         FALSE, tid);
    driver_tid_ = driver_ ? tid : 0;
    return driver_ != nullptr;
  }

  const char *timer_name() const {
    switch (timer_) {
    case timer_kind::hr:
      return "high-resolution waitable timer";
    case timer_kind::sleep:
      return "timeBeginPeriod(1) + Sleep";
    default:
      return "spinning on QueryPerformanceCounter";
    }
  }

  void wait_until(int64_t target) {
    for (;;) {
      const int64_t left = target - qpc_now();
      if (left <= 0)
        return;
      if (timer_ == timer_kind::hr) {
        LARGE_INTEGER due;
        due.QuadPart = -std::max<int64_t>(1, left * 10000000 / qpc_freq()); // 100 ns, relative
        if (!SetWaitableTimer(hr_, &due, 0, nullptr, nullptr, FALSE)) {
          timer_ = timer_kind::spin;
          continue;
        }
        WaitForSingleObject(hr_, INFINITE);
      } else if (timer_ == timer_kind::sleep) {
        Sleep(DWORD(std::max<int64_t>(1, left * 1000 / qpc_freq())));
        return;
      } else {
        YieldProcessor();
      }
    }
  }
  // Wait for the next tick after `next` and return it. A late wake shortens the next wait (the
  // high-resolution timer fires on a ~0.5 ms grid, so single waits of 1 ms take 1.5 ms but the
  // ticks average 1 ms); one more than an interval late (the sampler was descheduled) starts over
  // from now rather than bursting to catch up.
  int64_t pace(int64_t next) {
    next += interval_ticks_;
    const int64_t now = qpc_now();
    if (now - next > interval_ticks_)
      next = now;
    wait_until(next);
    return next;
  }
  // The mean interval of 20 paced ticks with timer k, in us.
  double try_timer(timer_kind k) {
    timer_ = k;
    const int64_t t0 = qpc_now();
    int64_t next = t0;
    for (int i = 0; i < 20; ++i)
      next = pace(next);
    return double(qpc_now() - t0) * 1e6 / double(qpc_freq()) / 20.0;
  }
  // The first timer that keeps the interval (within 25 % + 100 us): a high-resolution waitable
  // timer (Windows 10 1803+), then timeBeginPeriod(1) + Sleep (whole milliseconds only; the period
  // is raised again only while a sampled region runs), else spinning on QPC, which always does but
  // costs a core.
  void calibrate() {
    const double ok = double(o_.interval_us) * 1.25 + 100.0;
    const auto rejected = [this](const char *what) {
      char b[96];
      std::snprintf(b, sizeof b, "%s%s took %.0f us", rejected_.empty() ? "" : ", ", what,
                    calibrated_us_);
      rejected_ += b;
    };
    hr_ = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                 TIMER_ALL_ACCESS);
    if (hr_) {
      if ((calibrated_us_ = try_timer(timer_kind::hr)) <= ok)
        return;
      rejected("high-resolution timer");
    }
    if (o_.interval_us >= 1000) {
      if (HMODULE winmm = LoadLibraryW(L"winmm.dll")) {
        begin_period_ = reinterpret_cast<period_fn>(
            reinterpret_cast<void *>(GetProcAddress(winmm, "timeBeginPeriod")));
        end_period_ = reinterpret_cast<period_fn>(
            reinterpret_cast<void *>(GetProcAddress(winmm, "timeEndPeriod")));
        if (begin_period_ && end_period_ && begin_period_(1) == 0) {
          calibrated_us_ = try_timer(timer_kind::sleep);
          end_period_(1);
          if (calibrated_us_ <= ok)
            return;
          rejected("timeBeginPeriod(1) + Sleep");
        }
        begin_period_ = end_period_ = nullptr;
      }
    }
    calibrated_us_ = try_timer(timer_kind::spin);
  }

  // RIP at the return of a system-call stub in ntdll or win32u: the driver is in the kernel, not
  // running user code. Read from the module's own code, so no guess: every x64 stub is
  // "syscall; ret; int 2Eh; ret" (0F 05 C3 CD 2E C3), which a stray 0F 05 inside other code
  // (RtlAllocateHeap has some) does not match. write() splits waits from other system calls.
  bool at_syscall(uint64_t rip) const {
    const bool in = (rip >= ntdll_.first + 2 && rip + 4 <= ntdll_.second) ||
                    (rip >= win32u_.first + 2 && rip + 4 <= win32u_.second);
    if (!in)
      return false;
    const auto *p = reinterpret_cast<const unsigned char *>(rip);
    return p[-2] == 0x0F && p[-1] == 0x05 && p[0] == 0xC3 && p[1] == 0xCD && p[2] == 0x2E &&
           p[3] == 0xC3;
  }

  // Resume the driver unless the watchdog already has: whoever clears busy_ owns the
  // ResumeThread. Returns whether this call did.
  bool release(uint64_t gen, int64_t t) {
    uint64_t expect = gen;
    if (!busy_.compare_exchange_strong(expect, 0))
      return false;
    ResumeThread(driver_);
    const int64_t held = qpc_now() - t;
    ++held_n_;
    held_ticks_ += held;
    held_max_ = std::max(held_max_, held);
    return true;
  }

  // One sample, on the sampler thread. Between SuspendThread and ResumeThread: no allocation, no
  // lock, no output.
  void take_one() {
    const uint64_t gen = ++gen_;
    const int64_t t = qpc_now();
    if (SuspendThread(driver_) == DWORD(-1)) {
      ++failed_;
      return;
    }
    // Watched only from here on. Published before SuspendThread, a sampler descheduled in between
    // for 50 ms would let the watchdog "resume" a running driver, then suspend it with nobody left
    // to resume it. Descheduled here instead, it resumes the driver once it runs again.
    busy_since_ = qpc_now();
    busy_ = gen;
    if (!active_) { // the region ended since the last look: the driver is in end() or the join
      release(gen, t);
      return;
    }
    CONTEXT ctx;
    ctx.ContextFlags = CONTEXT_CONTROL | CONTEXT_INTEGER; // RIP, RSP and the integer registers
    const int64_t slot = kept_ % cap_;
    uint64_t *out = frames_ + slot * o_.depth;
    int n = 0;
    uint32_t flags = 0;
    if (GetThreadContext(driver_, &ctx)) {
      if (at_syscall(ctx.Rip))
        flags |= f_syscall;
      bool trunc = false, stop = false;
      n = unwind_into(&ctx, out, o_.depth, stack_hi_, &busy_, gen, &trunc, &stop);
      flags |= (trunc ? f_truncated : 0) | (stop ? f_stopped : 0);
    } else {
      ++failed_;
    }
    if (!release(gen, t))
      n = 0; // the watchdog resumed the driver mid-sample: the stack may be torn
    if (n > 0) {
      hdrs_[slot] = {t, uint32_t(n), flags};
      ++kept_;
    }
  }

  void sample_main() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    calibrate();
    SetEvent(idle_);
    for (;;) {
      WaitForSingleObject(go_, INFINITE);
      if (quit_)
        return;
      int64_t next = qpc_now();
      while (active_) {
        next = pace(next);
        if (!active_)
          break;
        if (!tripped_)
          take_one();
      }
      sampling_ = false; // the watchdog watches until here, past the region's end
      SetEvent(idle_);
    }
  }

  // Resumes the driver when one sample has held it suspended over 50 ms, and stops sampling for
  // the region. It watches until the sampler is done (sampling_), not just until the region ends:
  // a sample taken as the region ends still suspends the driver.
  void watch_main() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    for (;;) {
      WaitForSingleObject(go_wd_, INFINITE);
      if (quit_)
        return;
      while (sampling_ && !quit_) {
        Sleep(5);
        const uint64_t g = busy_;
        if (!g)
          continue;
        const int64_t since = busy_since_;
        if (busy_ != g || qpc_now() - since < limit_ticks_)
          continue;
        uint64_t expect = g;
        if (busy_.compare_exchange_strong(expect, 0)) {
          ResumeThread(driver_);
          trip_ticks_ = qpc_now() - since;
          tripped_ = true;
        }
      }
    }
  }

  void stop_threads() {
    quit_ = true;
    active_ = sampling_ = false;
    if (go_)
      SetEvent(go_);
    if (go_wd_)
      SetEvent(go_wd_);
    // A sampler that never went idle may never return; with every watchdog disarmed at exit,
    // joining it could hang the process.
    if (stuck_) {
      if (sampler_th_.joinable())
        sampler_th_.detach();
      if (watch_th_.joinable())
        watch_th_.detach();
    } else {
      if (sampler_th_.joinable())
        sampler_th_.join();
      if (watch_th_.joinable())
        watch_th_.join();
    }
    if (period_on_ && end_period_)
      end_period_(1);
  }

  using period_fn = unsigned(WINAPI *)(unsigned);
  int64_t cap_ = 0, interval_ticks_ = 1, limit_ticks_ = 1;
  uint64_t *frames_ = nullptr;
  hdr *hdrs_ = nullptr;
  std::pair<uintptr_t, uintptr_t> ntdll_{0, 0}, win32u_{0, 0};
  HANDLE go_ = nullptr, go_wd_ = nullptr, idle_ = nullptr, hr_ = nullptr;
  HANDLE driver_ = nullptr;
  DWORD driver_tid_ = 0;
  uintptr_t stack_hi_ = 0;
  timer_kind timer_ = timer_kind::spin;
  double calibrated_us_ = 0;
  std::string rejected_; // the timers calibrate() passed over, and what their interval took
  period_fn begin_period_ = nullptr, end_period_ = nullptr;
  bool period_on_ = false;
  std::thread sampler_th_, watch_th_;
  std::atomic<bool> quit_{false}, active_{false}, sampling_{false}, tripped_{false};
  std::atomic<uint64_t> busy_{0};
  std::atomic<int64_t> busy_since_{0}, trip_ticks_{0};
  uint64_t gen_ = 0;
  // Written by the sampler thread inside a region, read on the driver after region_join.
  int64_t kept_ = 0, failed_ = 0;
  int64_t held_n_ = 0, held_ticks_ = 0, held_max_ = 0; // how long samples held the driver
  int64_t regions_ = 0, region_t0_ = 0, region_ticks_ = 0;
  bool joining_ = false, stuck_ = false;
  std::string warning_;
  std::vector<pending_rep> pending_;               // sampled reps whose files finish() writes
  std::unordered_map<uint64_t, bool> stub_blocks_; // syscall stub return -> blocking_stub()
#endif
};

sampler &the_sampler() {
  static sampler s;
  return s;
}

// ---------------------------------------------------------------------------------------------
// Scenario framework: sizes, sweep points, reps, the measured region, records.
// ---------------------------------------------------------------------------------------------

using sizes_t = std::vector<std::pair<std::string, std::vector<std::string>>>;
using params_t = std::vector<std::pair<std::string, std::string>>;

struct host_info {
  int64_t sleep_1ms_ns = 0; // what std::this_thread::sleep_for(1 ms) really takes here
  int pinned_cpu = -1;
  std::string pin_how, pin_sibling;
  std::string priority = "normal";
  unsigned logical_cpus = 0;
  unsigned pool_workers = 0; // the app's background pool size (app::getThreadPoolSize)
  bool peak_resets = false;  // reset_peak_memory works here
};

struct run_config {
  int reps = 5;
  int warmup = 1;
  bool reps_set = false, warmup_set = false;
  bool smoke = false;
  bool bg_oversubscribe = false;
  double timeout = 120.0;
  std::string json_path;
  std::map<std::string, std::vector<std::string>> size_over;
  std::vector<std::string> executors; // filter; empty = all
  host_info host;
};

struct sample {
  int64_t wall = 0, thread_cpu = 0, proc_cpu = 0;
  uint64_t allocs = 0;
  mem_info mem;
  int threads = -1;
  int64_t handles = -1, created = -1;
  int64_t app_threads = -1, app_active = -1, app_pending = -1;
};

// The measured region of one rep: begin() just before the first submit, end() right after the
// last program finished. The probes run outside the clocks, and the ones that may allocate (the
// thread count's buffer) before the memory reading.
struct meter {
  bool want_heap = false;
  cvc::app *app = nullptr; // when set, its thread map and pool counts are sampled too
  bool begun = false, ended = false;
  sample a, b;
  void probe(sample &s) {
    s.threads = thread_count();
    s.handles = handle_count();
    s.created = threads_created();
    if (app) {
      // app::threads() copies the map under its mutex: the entries it has never erased.
      s.app_threads = static_cast<int64_t>(app->threads().size());
      s.app_active = app->getActiveThreadCount();
      s.app_pending = app->getPendingThreadCount();
    }
  }
  void begin() {
    widen_other_threads();
    probe(a);
    a.mem = memory(want_heap);
    a.proc_cpu = process_cpu_ns();
    a.thread_cpu = static_cast<int64_t>(se::thread_cpu_seconds() * 1e9);
    begun = true;
    the_sampler().region_begin(); // a no-op unless this rep is sampled (--sample)
    a.allocs = bench_alloc::count;
    a.wall = now_ns();
  }
  void end() {
    b.wall = now_ns();
    the_sampler().region_stop();
    b.allocs = bench_alloc::count;
    b.thread_cpu = static_cast<int64_t>(se::thread_cpu_seconds() * 1e9);
    b.proc_cpu = process_cpu_ns();
    b.mem = memory(want_heap);
    probe(b);
    ended = true;
    the_sampler().region_join();
  }
  int64_t wall() const { return b.wall - a.wall; }
};

// Steps by kind, summed over processes (process_info).
struct step_kinds {
  bool have = false;
  uint64_t eval = 0, read = 0, data = 0, async = 0, async_calls = 0;
  void add(const se::process_info &p) {
    have = true;
    eval += p.eval_steps;
    read += p.read_steps;
    data += p.data_steps;
    async += p.async_steps;
    async_calls += p.async_calls;
  }
  void add(const se::evaluation_stats &s) {
    have = true;
    eval += s.steps(se::step_kind::eval);
    read += s.steps(se::step_kind::read);
    data += s.steps(se::step_kind::data);
    async += s.steps(se::step_kind::async);
  }
};

// What one rep found, beyond the meter's samples.
struct rep_out {
  bool ok = true;
  std::string error;
  std::optional<uint64_t> steps_total;
  step_kinds kinds;
  std::optional<int64_t> programs, iterations, async_calls;
  std::vector<int64_t> latency_ns, async_latency_ns;
  jobj extra;
  jobj sample; // the sampler's fields (--sample), top level in the record
  void fail(const std::string &why) {
    if (ok) {
      ok = false;
      error = why;
    }
  }
};

struct point_ctx;

struct scenario {
  std::string name;
  std::string what;
  std::vector<std::string> executors; // {""}: no executor applies (parse.*)
  sizes_t sizes, smoke;
  // Keys that only vary some executors' runs (the async scheduler's driving, say): for the
  // others they take their first value and stay out of the params. async_only is the common case.
  std::set<std::string> async_only;
  std::map<std::string, std::vector<std::string>> varies_for;
  bool want_heap = false;
  int warmup = -1; // the scenario's own default warmup (-1: the run's)
  // The size that counts concurrent calls on the app's background pool, if any. A point with more
  // of them than the pool has workers runs only with --bg-oversubscribe (with --smoke it is
  // clamped to the pool size): tasks queued past the pool get the same thread key
  // (app::uniqueThreadKey sees only started threads), and starting one waits up to 5 s for its
  // namesake (app::threads), which can also hang the pool outright (docs/STATE_EXEC_BENCH.md,
  // "Known pathologies").
  std::string bg_calls_key;
  // The driver starts threads inside the measured region (background kernels, the bench's own
  // threads), which on Linux would inherit a pinned driver's CPU: such points run unpinned there.
  bool spawns_threads = false;
  std::function<bool(const std::string &exec, const point_ctx &)> accept;
  std::function<void(point_ctx &)> run;

  // Whether `key` is swept for executor `exec`.
  bool varies(const std::string &key, const std::string &exec) const {
    if (async_only.count(key))
      return exec == "async";
    const auto it = varies_for.find(key);
    if (it == varies_for.end())
      return true;
    return std::find(it->second.begin(), it->second.end(), exec) != it->second.end();
  }
};

struct point_ctx {
  const scenario &sc;
  std::string executor;
  params_t params;
  const run_config &cfg;
  std::vector<jobj> records;
  int printed = 0;
  bool failed = false;
  bool peak_inherited = false; // an earlier point of this process may own the peak figures
  int pin_cpu = -1;            // where the driver runs for this point (-1: not pinned)

  std::string s(const std::string &key, const std::string &dflt = "") const {
    for (const auto &[k, v] : params)
      if (k == key)
        return v;
    return dflt;
  }
  int64_t i(const std::string &key, int64_t dflt = 0) const {
    const std::string v = s(key);
    return v.empty() ? dflt : std::strtoll(v.c_str(), nullptr, 10);
  }

  template <class F> void reps(F fn);
  void record(int rep, const meter &m, rep_out &o);
  std::string params_json() const;
};

std::string params_text(const params_t &p) {
  std::string t;
  for (const auto &[k, v] : p)
    t += (t.empty() ? "" : " ") + k + "=" + v;
  return t;
}

std::string point_ctx::params_json() const {
  jobj p;
  for (const auto &[k, v] : params)
    p.raw(k, is_integer(v) ? v : json_str(v));
  return p.str();
}

void point_ctx::record(int rep, const meter &m, rep_out &o) {
  jobj r;
  r.s("scenario", sc.name);
  if (!executor.empty())
    r.s("executor", executor);
  r.raw("params", params_json()).i("rep", rep).b("ok", o.ok);
  if (!o.ok)
    r.s("error", o.error);
  const int64_t wall = m.wall();
  r.i("wall_ns", wall)
      .i("driver_cpu_ns", m.b.thread_cpu - m.a.thread_cpu)
      .i("process_cpu_ns", m.b.proc_cpu - m.a.proc_cpu);
  if (o.steps_total) {
    r.i("steps_total", static_cast<int64_t>(*o.steps_total));
    if (*o.steps_total)
      r.d("ns_per_step", double(wall) / double(*o.steps_total));
  }
  if (o.kinds.have) {
    jobj k;
    k.i("eval", o.kinds.eval).i("read", o.kinds.read).i("data", o.kinds.data);
    k.i("async", o.kinds.async);
    r.raw("steps_by_kind", k.str());
  }
  if (o.iterations && *o.iterations > 0)
    r.i("iterations", *o.iterations).d("ns_per_iter", double(wall) / double(*o.iterations));
  if (bench_alloc::active) {
    // The driver thread's allocations in the region (bench_alloc).
    const auto allocs = static_cast<int64_t>(m.b.allocs - m.a.allocs);
    r.i("allocs_driver", allocs);
    if (o.steps_total && *o.steps_total)
      r.d("allocs_per_step", double(allocs) / double(*o.steps_total));
    if (o.iterations && *o.iterations > 0)
      r.d("allocs_per_iter", double(allocs) / double(*o.iterations));
  }
  if (o.programs) {
    r.i("programs", *o.programs);
    if (wall > 0)
      r.d("programs_per_s", double(*o.programs) * 1e9 / double(wall));
  }
  if (!o.latency_ns.empty())
    r.raw("latency_ns", percentiles(o.latency_ns).str());
  if (o.async_calls)
    r.i("async_calls", *o.async_calls);
  if (!o.async_latency_ns.empty())
    r.raw("async_latency_ns", percentiles(o.async_latency_ns).str());
  r.i("peak_working_set_bytes", static_cast<int64_t>(m.b.mem.peak_ws))
      .i("working_set_bytes", static_cast<int64_t>(m.b.mem.ws))
      .i("working_set_delta_bytes", int64_t(m.b.mem.ws) - int64_t(m.a.mem.ws));
  if (m.b.mem.has_priv)
    r.i("private_bytes", static_cast<int64_t>(m.b.mem.priv))
        .i("private_delta_bytes", int64_t(m.b.mem.priv) - int64_t(m.a.mem.priv));
  if (m.b.mem.has_peak_priv)
    r.i("peak_private_bytes", static_cast<int64_t>(m.b.mem.peak_priv));
  if (peak_inherited)
    r.b("peak_inherited", true);
  if (m.a.mem.has_heap && m.b.mem.has_heap)
    r.i("heap_delta_bytes", int64_t(m.b.mem.heap) - int64_t(m.a.mem.heap));
  r.i("thread_count_before", m.a.threads).i("thread_count_after", m.b.threads);
  if (m.a.created >= 0)
    r.i("threads_created", m.b.created - m.a.created);
  if (m.a.handles >= 0)
    r.i("handle_count_before", m.a.handles).i("handle_count_after", m.b.handles);
  if (m.a.app_threads >= 0)
    r.i("app_threads_before", m.a.app_threads)
        .i("app_threads_after", m.b.app_threads)
        .i("app_pool_active_after", m.b.app_active)
        .i("app_pool_pending_after", m.b.app_pending);
  if (!o.extra.body.empty())
    r.raw("extra", o.extra.str());
  if (!o.sample.body.empty())
    r.body += ", " + o.sample.body;
  r.i("pin_cpu", pin_cpu);
  if (pin_cpu >= 0 && !cfg.host.pin_sibling.empty())
    r.s("pin_sibling", cfg.host.pin_sibling);
  r.i("logical_cpus", cfg.host.logical_cpus)
      .i("sleep_1ms_ns", cfg.host.sleep_1ms_ns)
      .b("smoke", cfg.smoke);
  records.push_back(r);

  // One human line per rep (outside every measured region).
  std::printf("  %-28s %-9s %-34s rep %d  %10.3f ms", sc.name.c_str(), executor.c_str(),
              params_text(params).c_str(), rep, double(wall) / 1e6);
  if (o.steps_total && *o.steps_total)
    std::printf("  %8.1f ns/step", double(wall) / double(*o.steps_total));
  else if (o.iterations && *o.iterations)
    std::printf("  %8.1f ns/iter", double(wall) / double(*o.iterations));
  if (!o.latency_ns.empty()) {
    auto v = o.latency_ns;
    std::sort(v.begin(), v.end());
    std::printf("  p99 %.3f ms", double(pct_at(v, 0.99)) / 1e6);
  }
  std::printf("  thr %d->%d", m.a.threads, m.b.threads);
  if (m.a.app_threads >= 0)
    std::printf(" app %lld->%lld", static_cast<long long>(m.a.app_threads),
                static_cast<long long>(m.b.app_threads));
  std::printf("  %s%s\n", o.ok ? "ok" : "FAIL: ", o.ok ? "" : o.error.c_str());
  std::fflush(stdout);
  if (!o.ok)
    failed = true;
}

// The drivers give up on a rep after --timeout, but only between slices: a driver thread blocked
// inside the runtime (a deadlock in the app's background pool, say) never gets back to check. The
// watchdog notices a rep that outlives its timeout by a grace period, or a sweep point (its setup
// and teardown included: ~app joins every thread it ever started) that outlives its whole
// budget, writes a failed record, and ends the process, so a hang shows in the JSON and the
// runner moves on.
class watchdog {
public:
  ~watchdog() {
    {
      std::lock_guard<std::mutex> lk(mu_);
      quit_ = true;
    }
    cv_.notify_all();
    if (th_.joinable())
      th_.join();
  }
  // Start the thread now (from main, before the driver is pinned, so it doesn't inherit the pin).
  void start() {
    std::lock_guard<std::mutex> lk(mu_);
    if (!th_.joinable())
      th_ = std::thread([this] { watch(); });
  }
  void set_output(std::FILE *json) { json_ = json; }
  void arm_point(double seconds, std::string label, std::string failure_record) {
    std::lock_guard<std::mutex> lk(mu_);
    point_.deadline = now_ns() + static_cast<int64_t>(seconds * 1e9);
    point_.label = std::move(label);
    point_.failure = std::move(failure_record);
  }
  void disarm_point() {
    std::lock_guard<std::mutex> lk(mu_);
    point_.deadline = 0;
  }
  void arm(double seconds, std::string label, std::string failure_record) {
    std::lock_guard<std::mutex> lk(mu_);
    rep_.deadline = now_ns() + static_cast<int64_t>(seconds * 1e9);
    rep_.label = std::move(label);
    rep_.failure = std::move(failure_record);
  }
  void disarm() {
    std::lock_guard<std::mutex> lk(mu_);
    rep_.deadline = 0;
  }

private:
  struct guard {
    int64_t deadline = 0;
    std::string label, failure;
  };
  [[noreturn]] void fire(const guard &g, const char *what) {
    std::printf("  %s  FAIL: hung (%s); exiting\n", g.label.c_str(), what);
    std::fflush(stdout);
    if (json_) {
      std::fprintf(json_, "%s\n", g.failure.c_str());
      std::fflush(json_);
    }
    std::_Exit(3);
  }
  void watch() {
    std::unique_lock<std::mutex> lk(mu_);
    while (!quit_) {
      cv_.wait_for(lk, std::chrono::seconds(1));
      const int64_t now = now_ns();
      if (rep_.deadline && now > rep_.deadline)
        fire(rep_, "the driver is blocked inside the runtime");
      if (point_.deadline && now > point_.deadline)
        fire(point_, "the sweep point's setup or teardown outlived its budget");
    }
  }
  std::mutex mu_;
  std::condition_variable cv_;
  std::thread th_;
  bool quit_ = false;
  guard rep_, point_;
  std::FILE *json_ = nullptr;
};

watchdog &the_watchdog() {
  static watchdog w;
  return w;
}

// The record the watchdog writes for a hung rep or point.
std::string hung_record(const point_ctx &pc, int rep, const std::string &why) {
  jobj hung;
  hung.s("scenario", pc.sc.name);
  if (!pc.executor.empty())
    hung.s("executor", pc.executor);
  hung.raw("params", pc.params_json()).i("rep", rep).b("ok", false);
  hung.s("error", why);
  hung.i("pin_cpu", pc.pin_cpu).b("smoke", pc.cfg.smoke);
  return hung.str();
}

// Where a sampled rep's files go: <prefix>.<scenario>.<executor>.<point>[.repN], the part after
// the prefix made safe for a file name.
std::string sample_stem(const point_ctx &pc, int rep) {
  std::string point;
  for (const auto &[k, v] : pc.params)
    point += (point.empty() ? "" : ",") + k + "=" + v;
  std::string s = pc.sc.name + "." + (pc.executor.empty() ? "none" : pc.executor) + "." +
                  (point.empty() ? "default" : point);
  if (the_sampler().opts().all_reps)
    s += ".rep" + std::to_string(rep);
  for (char &c : s)
    if (std::strchr("<>:\"/\\|?* ", c))
      c = '_';
  return the_sampler().opts().prefix + "." + s;
}

template <class F> void point_ctx::reps(F fn) {
  const int warm = cfg.warmup_set || sc.warmup < 0 ? cfg.warmup : sc.warmup;
  for (int n = 0; n < warm + cfg.reps; ++n) {
    meter m;
    m.want_heap = sc.want_heap;
    rep_out o;
    // --sample: the first measured rep only (or every one with --sample-all-reps), so the other
    // reps' timings stay clean.
    const bool sampled =
        the_sampler().on() && n >= warm && (the_sampler().opts().all_reps || n == warm);
    the_sampler().arm(sampled);
    the_watchdog().arm(cfg.timeout + 60.0, sc.name + " " + executor + " " + params_text(params),
                       hung_record(*this, n < warm ? -1 : n - warm,
                                   "hung: the rep outlived --timeout by 60 s with the driver "
                                   "blocked inside the runtime; the process was ended"));
    try {
      fn(m, o);
    } catch (const std::exception &e) {
      o.fail(std::string("exception: ") + e.what());
    }
    the_watchdog().disarm();
    the_sampler().arm(false);
    if (m.begun && !m.ended)
      m.end(); // stops a sampled region the scenario left open
    if (sampled)
      o.sample =
          the_sampler().write(sample_stem(*this, n - warm),
                              "state_exec_bench --sample: " + sc.name + " " + executor + " " +
                                  params_text(params) + " rep " + std::to_string(n - warm));
    else if (the_sampler().on() && n >= warm)
      o.sample.b("sampled", false);
    if (!m.begun)
      m.begin();
    if (!m.ended)
      m.end();
    if (n < warm) {
      if (!o.ok) { // a warmup that fails won't measure anything either
        record(-1, m, o);
        return;
      }
      continue;
    }
    record(n - warm, m, o);
  }
}

// ---------------------------------------------------------------------------------------------
// Worlds: an environment, a scheduler, the app's tree and pools, an async host, the natives.
// ---------------------------------------------------------------------------------------------

int64_t int_arg(std::span<const se::value_t> args, std::size_t i, int64_t dflt) {
  if (i < args.size())
    if (const auto *n = std::get_if<int64_t>(&args[i].v))
      return *n;
  return dflt;
}

// The natives every benchmark environment has.
void add_bench_natives(const se::environment_ptr &env) {
  // (bench-inc x): x + 1. A trivial native, the unit of the native-call scenarios.
  se::builtins::register_fn(env, "bench-inc", [](std::span<const se::value_t> a) -> se::value_t {
    return se::value_t(int_arg(a, 0, 0) + 1);
  });
}

// What the async kernels report: each call's end time by index, the OS threads kernels ran on,
// the compute fan-outs' timings, and the latch the blocking kernel waits on. Shared with the
// kernels by shared_ptr, so a kernel still running when a failed rep unwinds (~world waits for
// it) never writes into freed memory.
struct kernel_log {
  std::unique_ptr<std::atomic<int64_t>[]> end_ns;
  std::size_t n = 0;
  std::atomic<int> started{0};
  std::mutex mu;
  std::set<std::thread::id> threads;   // guarded by mu
  std::vector<int64_t> fanout_wait_ns; // parallel_for call -> its first chunk starting; mu
  std::vector<int64_t> fanout_ns;      // a whole parallel_for; mu
  std::condition_variable latch_cv;    // bench-block waits until `released`
  bool released = false;               // guarded by mu
  void reset(std::size_t count) {
    end_ns.reset(new std::atomic<int64_t>[count ? count : 1]);
    n = count;
    for (std::size_t i = 0; i < n; ++i)
      end_ns[i].store(0);
  }
  void mark(int64_t i) {
    if (i >= 0 && static_cast<std::size_t>(i) < n)
      end_ns[static_cast<std::size_t>(i)].store(now_ns(), std::memory_order_release);
    std::lock_guard<std::mutex> lk(mu);
    threads.insert(std::this_thread::get_id());
  }
  int64_t at(std::size_t i) const { return i < n ? end_ns[i].load(std::memory_order_acquire) : 0; }
  void release() {
    {
      std::lock_guard<std::mutex> lk(mu);
      released = true;
    }
    latch_cv.notify_all();
  }
  std::size_t thread_count() {
    std::lock_guard<std::mutex> lk(mu);
    return threads.size();
  }
};
using kernel_log_ptr = std::shared_ptr<kernel_log>;

// Releases a kernel_log's latch when it goes out of scope. Declared after the world, so it runs
// before ~world waits for the kernels blocked on it.
struct latch_release {
  kernel_log_ptr log;
  ~latch_release() {
    if (log)
      log->release();
  }
};

void spin_for_ns(int64_t ns) {
  const int64_t until = now_ns() + ns;
  while (now_ns() < until) {
  }
}

// A blocking wait of 1 ms, as an I/O-bound kernel would block. On Windows sleep_for(1 ms) lasts a
// timer tick (~15.6 ms) unless the timer resolution was raised, which would make the sleep1ms
// kernels measure the OS timer rather than the executors, so they wait on a high-resolution
// waitable timer (Windows 10 1803+) where there is one.
void block_1ms() {
#if defined(_WIN32)
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif
  if (HANDLE t = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                                        TIMER_ALL_ACCESS)) {
    LARGE_INTEGER due;
    due.QuadPart = -10000; // 1 ms, relative, in 100 ns units
    const bool ok = SetWaitableTimer(t, &due, 0, nullptr, nullptr, FALSE) &&
                    WaitForSingleObject(t, INFINITE) == WAIT_OBJECT_0;
    CloseHandle(t);
    if (ok)
      return;
  }
#endif
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

// An app, built where its threads won't inherit the driver's pin (unpinned_scope).
std::unique_ptr<cvc::app> make_app(bool with_compute_pool = false) {
  unpinned_scope u;
  auto app = std::make_unique<cvc::app>();
  if (with_compute_pool)
    app->computePool(); // built lazily otherwise, by whichever thread first fans out
  return app;
}

struct world {
  cvc::app *app = nullptr;
  cvc::state *tree = nullptr;
  std::unique_ptr<se::scheduler> sync;
  std::unique_ptr<se::async_scheduler> async;
  std::vector<std::unique_ptr<cvc::async_lane>> lanes;
  std::unique_ptr<se::async_host> host;
  se::intrinsics_context ictx;
  se::process_ptr ctx_proc = se::make_process();
  se::environment_ptr env = se::builtins::make_default_environment();

  // `exec` is "sync" or "async"; `a` gives the scheduler the app's tree as its watch root and the
  // intrinsics their root, as app::exec_scheduler() and pycvc's Exec do.
  world(const std::string &exec, cvc::app *a) : app(a) {
    if (app)
      tree = &cvc::state::instance(*app);
    if (exec == "sync") {
      sync = std::make_unique<se::scheduler>();
      if (tree)
        sync->set_watch_root(tree);
      ictx.sched = sync.get();
    } else {
      async = std::make_unique<se::async_scheduler>();
      // What app::exec_scheduler() and pycvc's Exec give theirs: scripts read inside the
      // process, deep data a slice per step.
      async->set_parser_defaults(se::async_scheduler::host_parser_defaults());
      async->set_data_defaults(se::async_scheduler::host_data_defaults());
      if (tree)
        async->set_watch_root(tree);
      ictx.sched = async.get();
    }
    add_bench_natives(env);
    if (tree) {
      ictx.root = tree;
      ctx_proc->pid = 0;
      ictx.proc = ctx_proc;
      ictx.pid = 0;
      ictx.uid = "bench";
      ictx.cluster_id = "local";
      ictx.node_id = "local";
      se::register_intrinsics(env, &ictx);
    }
  }
  world(const world &) = delete;
  world &operator=(const world &) = delete;
  ~world() {
    // No kernel may outlive the scheduler it posts to, nor a lane job the host it reports to.
    if (host)
      host->shutdown();
    host.reset();
    lanes.clear();
    async.reset();
    sync.reset();
  }

  se::scheduler_base &base() { return sync ? static_cast<se::scheduler_base &>(*sync) : *async; }

  // An async host whose kernels run `where`: background (the app's task pool), lane (a
  // cvc::async_lane named "bench"), or inline (the sync scheduler: in the step that calls).
  void make_host(const std::string &where) {
    if (sync) {
      host = std::make_unique<se::async_host>(sync.get());
      return;
    }
    host = std::make_unique<se::async_host>(*async, app);
    if (where == "lane")
      add_lane("bench");
  }
  void add_lane(const std::string &name) {
    unpinned_scope u; // the lane's thread must not inherit the driver's pin
    lanes.push_back(std::make_unique<cvc::async_lane>(name));
    host->add_lane(name, *lanes.back());
  }

  // The environment a process runs in: its own scope over the shared one ("extend": the
  // builtins, intrinsics and natives shared, so concurrent programs don't clobber each other's
  // globals; one scope level more than pycvc, which runs one program at a time in its shared
  // environment), or a whole default environment of its own ("own": as Ariadne builds one per
  // action).
  se::environment_ptr process_env(const std::string &mode) {
    if (mode != "own")
      return se::environment::extend(env);
    auto own = se::builtins::make_default_environment();
    add_bench_natives(own);
    if (tree)
      se::register_intrinsics(own, &ictx);
    return own;
  }

  int submit(const std::string &src, se::execute_options opts) {
    return sync ? sync->execute(src, opts) : async->execute(src, opts);
  }
  int submit(const se::value_t &ast, se::execute_options opts) {
    return sync ? sync->execute(ast, opts) : async->execute(ast, opts);
  }
  std::optional<se::process_info> info(int pid) const {
    return sync ? sync->get_process_info(pid) : async->get_process_info(pid);
  }
  uint64_t total_steps() const {
    return sync ? sync->get_stats().total_steps : async->get_stats().total_steps;
  }
  int process_count() const { return sync ? sync->process_count() : async->process_count(); }
  bool kill(int pid) { return sync ? sync->kill(pid) : async->kill(pid); }
};

// Register the async functions the async scenarios call, on w's host, run `where`.
void register_kernels(world &w, const kernel_log_ptr &log, const std::string &where) {
  se::async_options opts;
  if (where == "lane")
    opts.where = se::executor::on_lane("bench");
  else if (where == "inline")
    opts.where = se::executor::inline_call();
  // (bench-noop i): i, at once. Pure overhead of an async call.
  se::register_async_fn(
      w.env, *w.host, "bench-noop",
      [log](std::span<const se::value_t> a) -> se::async_kernel {
        const int64_t i = int_arg(a, 0, 0);
        return [log, i](se::async_call &) -> se::value_t {
          log->mark(i);
          return se::value_t(i);
        };
      },
      opts);
  // (bench-sleep1ms i): i after a 1 ms blocking wait (I/O-like).
  se::register_async_fn(
      w.env, *w.host, "bench-sleep1ms",
      [log](std::span<const se::value_t> a) -> se::async_kernel {
        const int64_t i = int_arg(a, 0, 0);
        return [log, i](se::async_call &) -> se::value_t {
          block_1ms();
          log->mark(i);
          return se::value_t(i);
        };
      },
      opts);
  // (bench-cpu100us i): i after 100 us of busy CPU.
  se::register_async_fn(
      w.env, *w.host, "bench-cpu100us",
      [log](std::span<const se::value_t> a) -> se::async_kernel {
        const int64_t i = int_arg(a, 0, 0);
        return [log, i](se::async_call &) -> se::value_t {
          spin_for_ns(100000);
          log->mark(i);
          return se::value_t(i);
        };
      },
      opts);
  // (bench-big i): a 1 MiB string. What the host keeps of a call's result shows in memory.
  se::register_async_fn(
      w.env, *w.host, "bench-big",
      [log](std::span<const se::value_t> a) -> se::async_kernel {
        const int64_t i = int_arg(a, 0, 0);
        return [log, i](se::async_call &) -> se::value_t {
          log->mark(i);
          return se::value_t(std::string(std::size_t(1) << 20, 'x'));
        };
      },
      opts);
  // (bench-block i): i, once the bench releases the log's latch. A call that stays in flight.
  se::register_async_fn(
      w.env, *w.host, "bench-block",
      [log](std::span<const se::value_t> a) -> se::async_kernel {
        const int64_t i = int_arg(a, 0, 0);
        return [log, i](se::async_call &) -> se::value_t {
          log->started.fetch_add(1);
          std::unique_lock<std::mutex> lk(log->mu);
          log->latch_cv.wait(lk, [&] { return log->released; });
          lk.unlock();
          log->mark(i);
          return se::value_t(i);
        };
      },
      opts);
  // (bench-fanout chunks us [max_par]): a parallel_for of `chunks` chunks of `us` busy
  // microseconds over app.computePool(), at most max_par wide (0: the pool's width); returns the
  // chunks run. Logs the wait for its first chunk to start, and the whole fan-out's time.
  if (cvc::app *app = w.app) {
    se::register_async_fn(
        w.env, *w.host, "bench-fanout",
        [app, log](std::span<const se::value_t> a) -> se::async_kernel {
          const int64_t chunks = int_arg(a, 0, 64), us = int_arg(a, 1, 50);
          const int64_t max_par = int_arg(a, 2, 0);
          return [app, log, chunks, us, max_par](se::async_call &) -> se::value_t {
            std::atomic<int64_t> ran{0};
            std::atomic<int64_t> first{0};
            const int64_t t0 = now_ns();
            app->computePool().parallel_for(
                static_cast<int>(chunks),
                [&](int) {
                  int64_t expect = 0;
                  first.compare_exchange_strong(expect, now_ns());
                  if (us)
                    spin_for_ns(us * 1000);
                  ran.fetch_add(1, std::memory_order_relaxed);
                },
                static_cast<int>(max_par));
            const int64_t t1 = now_ns();
            {
              std::lock_guard<std::mutex> lk(log->mu);
              log->fanout_wait_ns.push_back(first.load() - t0);
              log->fanout_ns.push_back(t1 - t0);
              log->threads.insert(std::this_thread::get_id());
            }
            return se::value_t(ran.load());
          };
        },
        opts);
  }
}

// Per-program bookkeeping: when each was submitted and finished, and what it returned. Filled by
// execute_options::on_complete, which the schedulers call as a process terminates.
struct tracker {
  std::vector<int> pid;
  std::vector<int64_t> submit_ns, finish_ns;
  std::vector<se::value_t> result;
  std::vector<std::size_t> fresh; // finished since the last sweep
  std::vector<std::size_t> live;  // submitted and not reaped (host=ariadne sweeps these)
  std::size_t completed = 0;
  explicit tracker(std::size_t n) : pid(n, -1), submit_ns(n, 0), finish_ns(n, 0), result(n) {
    fresh.reserve(n);
    live.reserve(n);
  }
  std::function<void(se::value_t)> hook(std::size_t i) {
    return [this, i](se::value_t v) {
      finish_ns[i] = now_ns();
      result[i] = std::move(v);
      ++completed;
      fresh.push_back(i);
    };
  }
  bool all_done() const { return completed == pid.size(); }
};

// After every slice, on the async scheduler: reap what finished, as the hosts do. host=bare looks
// at the processes that just finished (pycvc's run() reaps its one program); host=ariadne walks
// every live process with get_process_info, as Ariadne's pump_and_sweep_actions does each frame
// (O(live) per pump), and reaps the finished ones. The reaped processes' step counts go in kinds.
void sweep(world &w, tracker &t, bool reap, bool ariadne, step_kinds &kinds) {
  if (w.async && reap && ariadne) {
    std::size_t keep = 0;
    for (std::size_t j = 0; j < t.live.size(); ++j) {
      const std::size_t i = t.live[j];
      auto info = w.async->get_process_info(t.pid[i]);
      if (info && !se::is_finished(info->status)) {
        t.live[keep++] = i;
        continue;
      }
      if (info)
        kinds.add(*info);
      w.async->reap(t.pid[i]);
    }
    t.live.resize(keep);
  } else if (w.async && reap) {
    for (std::size_t i : t.fresh) {
      if (auto info = w.async->get_process_info(t.pid[i]))
        kinds.add(*info);
      w.async->reap(t.pid[i]);
    }
  }
  t.fresh.clear();
}

// Why programs didn't finish: the first one that ended badly, or the first still running.
std::string first_failure(world &w, const tracker &t) {
  std::string still;
  for (std::size_t i = 0; i < t.pid.size(); ++i) {
    if (t.finish_ns[i])
      continue;
    auto info = w.info(t.pid[i]);
    if (!info)
      return "program " + std::to_string(i) + " (pid " + std::to_string(t.pid[i]) + ") vanished";
    if (se::is_finished(info->status))
      return "program " + std::to_string(i) + " " + se::to_string(info->status) + ": " +
             info->exit_error.value_or("?");
    if (still.empty())
      still = "program " + std::to_string(i) + " still " + se::to_string(info->status);
  }
  return still;
}

struct drive_cfg {
  std::string driver = "spin"; // async: spin (yield when idle) or poll1ms (sleep 1 ms when idle)
  uint64_t slice_steps = 200000;
  double slice_secs = 0.1; // async only: Ariadne's per-pump time backstop
  double timeout = 120.0;
  bool sleepers = false; // processes sleep: idle stretches are expected
};

// Pump w's scheduler until done(). The sync scheduler runs in run_steps(slice_steps) slices with
// no time budget, as the CLI's run() has none (a budget would add a clock read per step); the
// slicing only lets after() and the timeout check run. The async one is pumped as Ariadne's
// pump_and_sweep_actions and pycvc's Exec::run pump it: drain_ingress (the kernels' results and
// posts), wake_awaiting, then a sync_run_steps slice (what pycvc's Exec runs: done() reads the
// results it needs, so no slice builds the map of every finished process); it has no wakeup
// signal, so when nothing is runnable the driver yields (spin) or sleeps 1 ms (poll1ms, as
// pycvc did). after() runs after every slice. Returns "" or why it gave up.
template <class Done, class After>
std::string drive(world &w, const drive_cfg &d, Done done, After after) {
  const auto deadline = clk::now() + std::chrono::duration<double>(d.timeout);
  if (w.sync) {
    while (!done()) {
      if (!w.sync->has_runnable())
        return "stalled: nothing runnable";
      w.sync->run_steps(d.slice_steps, std::nullopt); // results read by done(), not the map
      after();
      if (!done() && clk::now() > deadline)
        return "timeout";
    }
    return "";
  }
  auto &a = *w.async;
  auto last_busy = clk::now();
  while (!done()) {
    a.drain_ingress();
    a.wake_awaiting();
    const bool runnable = a.has_runnable();
    if (runnable)
      a.sync_run_steps(d.slice_steps, d.slice_secs);
    after();
    if (done())
      break;
    const auto now = clk::now();
    if (now > deadline)
      return "timeout";
    // A scenario whose processes (sleep ...) says so: a wait with a deadline is not a stall.
    if (runnable || d.sleepers || (w.host && w.host->in_flight() > 0) ||
        a.async_calls_in_flight() > 0) {
      last_busy = now;
    } else if (now - last_busy > std::chrono::seconds(2)) {
      return "stalled: nothing runnable or in flight";
    }
    if (!runnable) {
      if (d.driver == "poll1ms")
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      else
        std::this_thread::yield();
    }
  }
  return "";
}

drive_cfg driver_of(const point_ctx &pc) {
  drive_cfg d;
  d.driver = pc.s("driver", "spin");
  d.timeout = pc.cfg.timeout;
  return d;
}

// Ariadne's default action limits (ariadne.cpp: ariadne.limits.action_max_steps 5000,
// action_max_time 0.1 s, action_max_memory 4 MiB), which it arms on every action.
void arm_ariadne_limits(se::execute_options &opts) {
  opts.max_steps = 5000;
  opts.max_time = 0.1;
  opts.max_memory = uint64_t(4) << 20;
}

// Submit every program in `src` at once, drive them all to completion, check what each returned.
// The measured region is submit through completion. `env_mode`: see world::process_env. The
// point's `host` param (bare|ariadne) picks the sweep and the limits each program gets.
struct flood_out {
  std::vector<int64_t> latency; // per program, submit -> finish
  std::vector<int64_t> finish;  // per program, finish relative to the first submit
};

flood_out run_flood(world &w, point_ctx &pc, meter &m, rep_out &o,
                    const std::vector<std::string> &src, const std::vector<std::string> &expect,
                    const std::string &env_mode, bool reap,
                    std::function<void(se::execute_options &, std::size_t)> tune = nullptr) {
  tracker t(src.size());
  step_kinds kinds;
  const drive_cfg d = driver_of(pc);
  const bool ariadne = pc.s("host", "bare") == "ariadne";
  const uint64_t steps0 = w.total_steps();
  m.begin();
  for (std::size_t i = 0; i < src.size(); ++i) {
    se::execute_options opts;
    opts.env = w.process_env(env_mode);
    opts.on_complete = t.hook(i);
    if (ariadne)
      arm_ariadne_limits(opts);
    if (tune)
      tune(opts, i);
    t.submit_ns[i] = now_ns();
    t.pid[i] = w.submit(src[i], opts);
    t.live.push_back(i);
  }
  const int64_t submitted = now_ns();
  const std::string why =
      drive(w, d, [&] { return t.all_done(); }, [&] { sweep(w, t, reap, ariadne, kinds); });
  m.end();
  o.steps_total = w.total_steps() - steps0;
  if (!why.empty())
    o.fail(why + ": " + first_failure(w, t));
  // What the reaped processes ran is in kinds already; the rest are still in the scheduler.
  if (!(w.async && reap))
    for (std::size_t i = 0; i < src.size(); ++i)
      if (auto info = w.info(t.pid[i]))
        kinds.add(*info);
  o.kinds = kinds;
  o.programs = static_cast<int64_t>(src.size());
  if (!src.empty()) {
    // Submitting is part of the region (parse or environment per program, the async
    // scheduler's settings reads); this is its share.
    o.extra.i("submit_ns_total", submitted - t.submit_ns[0])
        .d("ns_per_submit", double(submitted - t.submit_ns[0]) / double(src.size()));
  }
  flood_out f;
  for (std::size_t i = 0; i < src.size(); ++i) {
    if (!t.finish_ns[i])
      continue;
    const std::string got = se::to_string(t.result[i]);
    if (got != expect[i])
      o.fail("program " + std::to_string(i) + " returned " + got + ", expected " + expect[i]);
    f.latency.push_back(t.finish_ns[i] - t.submit_ns[i]);
    f.finish.push_back(t.finish_ns[i] - m.a.wall);
  }
  if (kinds.have && kinds.async_calls)
    o.async_calls = static_cast<int64_t>(kinds.async_calls);
  return f;
}

// ---------------------------------------------------------------------------------------------
// The DSL programs of the evaluator and single-process scenarios: the same source for every
// executor, sized by the sweep point, with the result each must return.
// ---------------------------------------------------------------------------------------------

std::string istr(int64_t v) { return std::to_string(v); }

int64_t fib(int64_t n) { return n < 2 ? n : fib(n - 1) + fib(n - 2); }

struct eval_prog {
  std::string name, what;
  std::function<std::string(const point_ctx &)> src;
  std::function<std::string(const point_ctx &)> expect;
  std::function<int64_t(const point_ctx &)> iters; // the unit ns_per_iter divides by
  bool recursive_ok;                               // the recursive evaluator has every form used
  bool sched;                                      // also a sched.single.* scenario
  sizes_t sizes, smoke;
};

// `n` iterations of `body` in a while loop over i, after `setup`; returns `result`.
std::string loop_src(int64_t n, const std::string &setup, const std::string &body,
                     const std::string &result) {
  return "(begin " + setup + " (set i 0) (while (< i " + istr(n) + ") (begin " + body +
         " (set i (+ i 1)))) " + result + ")";
}

const std::vector<eval_prog> &eval_programs() {
  using P = const point_ctx &;
  const auto n_iters = [](P pc) { return pc.i("n"); };
  const auto n_sizes = [](int64_t n, int64_t smoke) {
    return std::pair<sizes_t, sizes_t>{{{"n", {istr(n)}}}, {{"n", {istr(smoke)}}}};
  };
  static const std::vector<eval_prog> p = [&] {
    std::vector<eval_prog> v;
    const auto add = [&](std::string name, std::string what, auto src, auto expect, auto iters,
                         bool rec, bool sched, sizes_t sizes, sizes_t smoke) {
      v.push_back({std::move(name), std::move(what), src, expect, iters, rec, sched,
                   std::move(sizes), std::move(smoke)});
    };
    {
      auto [s, sm] = n_sizes(40000, 300);
      add(
          "arith_loop", "a while loop summing 0..n-1",
          [](P pc) { return loop_src(pc.i("n"), "(set s 0)", "(set s (+ s i))", "s"); },
          [](P pc) { return istr(pc.i("n") * (pc.i("n") - 1) / 2); }, n_iters, true, true, s, sm);
    }
    {
      auto [s, sm] = n_sizes(20, 8);
      add(
          "fib", "recursive (fib n); an iteration is one call",
          [](P pc) {
            return "(begin (defun fib (n) (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) (fib " +
                   istr(pc.i("n")) + "))";
          },
          [](P pc) { return istr(fib(pc.i("n"))); },
          [](P pc) { return 2 * fib(pc.i("n") + 1) - 1; }, true, true, s, sm);
    }
    {
      auto [s, sm] = n_sizes(30000, 300);
      add(
          "closure_calls", "a lambda called n times",
          [](P pc) {
            return loop_src(pc.i("n"), "(set f (lambda (x) (+ x 1))) (set s 0)", "(set s (f s))",
                            "s");
          },
          [](P pc) { return istr(pc.i("n")); }, n_iters, true, true, s, sm);
    }
    {
      auto [s, sm] = n_sizes(40000, 300);
      add(
          "closure_map",
          "collections.map with a lambda over a list of n (the recursive evaluator's natives "
          "can't call DSL functions)",
          [](P pc) {
            const int64_t n = pc.i("n");
            return "(begin (set l (collect (range " + istr(n) +
                   "))) (set m (collections.map l (lambda (x) (* x 2)))) (nth m " + istr(n - 1) +
                   "))";
          },
          [](P pc) { return istr(2 * (pc.i("n") - 1)); }, n_iters, false, true, s, sm);
    }
    {
      auto [s, sm] = n_sizes(30000, 300);
      add(
          "list_build_walk", "append n items to a list, then sum it with for",
          [](P pc) {
            return loop_src(pc.i("n"), "(set l (list))", "(append l i)",
                            "(begin (set s 0) (for x l (set s (+ s x))) s)");
          },
          [](P pc) { return istr(pc.i("n") * (pc.i("n") - 1) / 2); }, n_iters, true, true, s, sm);
    }
    add(
        "dict_ops", "get-attr + set-attr on a dict of k keys (dicts are linear: sweep k), n times",
        [](P pc) {
          const int64_t k = pc.i("k");
          return "(begin (set d (dict)) (set keys (list)) (set j 0) (while (< j " + istr(k) +
                 ") (begin (set kk (str-concat \"key\" (str j))) (append keys kk) (set-attr d kk "
                 "0) (set j (+ j 1)))) (set i 0) (while (< i " +
                 istr(pc.i("n")) + ") (begin (set kk (nth keys (% i " + istr(k) +
                 "))) (set-attr d kk (+ (get-attr d kk) 1)) (set i (+ i 1)))) (set tot 0) (for "
                 "kk keys (set tot (+ tot (get-attr d kk)))) tot)";
        },
        [](P pc) { return istr(pc.i("n")); }, n_iters, true, true,
        {{"n", {"20000"}}, {"k", {"4", "64", "512"}}}, {{"n", {"300"}}, {"k", {"16"}}});
    add(
        "string_ops",
        "str, str-concat, string.substring, string.length, n times; lit=long uses a 24-char "
        "literal (past the 15-char small-string buffer)",
        [](P pc) {
          const std::string lit =
              pc.s("lit", "short") == "long" ? "item-with-a-long-prefix-" : "item-";
          return loop_src(pc.i("n"), "(set c 0)",
                          "(set x (str-concat \"" + lit +
                              "\" (str i))) (set c (+ c (string.length (string.substring x 0 5))))",
                          "c");
        },
        [](P pc) { return istr(5 * pc.i("n")); }, n_iters, true, true,
        {{"n", {"20000"}}, {"lit", {"short", "long"}}},
        {{"n", {"300"}}, {"lit", {"short", "long"}}});
    {
      auto [s, sm] = n_sizes(5000, 100);
      add(
          "exceptions", "try / raise ValueError / catch, n times",
          [](P pc) {
            return loop_src(pc.i("n"), "(set c 0)",
                            "(set c (+ c (try (raise (ValueError \"boom\")) (catch (ValueError e) "
                            "1))))",
                            "c");
          },
          [](P pc) { return istr(pc.i("n")); }, n_iters, true, true, s, sm);
    }
    {
      auto [s, sm] = n_sizes(40000, 300);
      add(
          "generators", "next over (range n) until it is done",
          [](P pc) {
            return "(begin (set g (range " + istr(pc.i("n")) +
                   ")) (set s 0) (set x (next g)) (while (not (is-null x)) (begin (set s (+ s x)) "
                   "(set x (next g)))) s)";
          },
          [](P pc) { return istr(pc.i("n") * (pc.i("n") - 1) / 2); }, n_iters, true, true, s, sm);
    }
    {
      auto [s, sm] = n_sizes(40000, 300);
      add(
          "for_range", "for over the generator (range n) (the recursive for takes lists only)",
          [](P pc) {
            return "(begin (set s 0) (for x (range " + istr(pc.i("n")) + ") (set s (+ s x))) s)";
          },
          [](P pc) { return istr(pc.i("n") * (pc.i("n") - 1) / 2); }, n_iters, false, true, s, sm);
    }
    {
      auto [s, sm] = n_sizes(40000, 300);
      add(
          "native_calls", "a registered trivial native (bench-inc) called n times",
          [](P pc) { return loop_src(pc.i("n"), "(set s 0)", "(set s (bench-inc s))", "s"); },
          [](P pc) { return istr(pc.i("n")); }, n_iters, true, true, s, sm);
    }
    add(
        "closure_body",
        "a defun of p parameters with a body of `forms` forms (a body of more than one is "
        "rebuilt as a (begin ...) list on every call), called n times",
        [](P pc) {
          const int64_t p = pc.i("p"), forms = pc.i("forms");
          std::string params, args, sum = "(+ 1";
          for (int64_t j = 0; j < p; ++j) {
            params += (j ? " a" : "a") + istr(j);
            args += " 1";
            sum += " a" + istr(j);
          }
          sum += ")";
          const std::string body = forms > 1 ? "(set tt 1) (set uu 2) " + sum : sum;
          return loop_src(pc.i("n"), "(defun f (" + params + ") " + body + ") (set s 0)",
                          "(set s (+ s (f" + args + ")))", "s");
        },
        [](P pc) { return istr(pc.i("n") * (1 + pc.i("p"))); }, n_iters, true, false,
        {{"n", {"30000"}}, {"p", {"0", "1", "4"}}, {"forms", {"1", "3"}}},
        {{"n", {"300"}}, {"p", {"0", "4"}}, {"forms", {"1", "3"}}});
    add(
        "lookup_depth",
        "the arith loop over globals (s, i, g) run inside `depth` nested calls, so each "
        "reference walks depth+1 scopes; names=long uses 24-character names",
        [](P pc) {
          const bool lng = pc.s("names", "short") == "long";
          const std::string s = lng ? "accumulated_sum_variable" : "s";
          const std::string i = lng ? "loop_index_variable_name" : "i";
          const std::string g = lng ? "global_increment_by_name" : "g";
          std::string loop = "(begin (set " + s + " 0) (set " + i + " 0) (while (< " + i + " " +
                             istr(pc.i("n")) + ") (begin (set " + s + " (+ " + s + " " + g +
                             ")) (set " + i + " (+ " + i + " 1)))) " + s + ")";
          // Each level is a lambda made inside the call of the one around it, so its scope
          // extends that call's: the loop runs `depth` scopes below the globals. (A head can't
          // be a lambda form, so each is bound, to a global, then called.)
          for (int64_t d = pc.i("depth") - 1; d >= 0; --d)
            loop = "(begin (set f" + istr(d) + " (lambda () " + loop + ")) (f" + istr(d) + "))";
          return "(begin (set " + g + " 1) (set " + s + " 0) (set " + i + " 0) " + loop + ")";
        },
        [](P pc) { return istr(pc.i("n")); }, n_iters, true, false,
        {{"n", {"30000"}}, {"depth", {"0", "4", "16"}}, {"names", {"short", "long"}}},
        {{"n", {"300"}}, {"depth", {"0", "4"}}, {"names", {"short", "long"}}});
    {
      auto [s, sm] = n_sizes(20000, 300);
      add(
          "try_noraise", "a try whose body doesn't raise, n times (the try form's setup alone)",
          [](P pc) {
            return loop_src(pc.i("n"), "(set s 0)",
                            "(set s (+ s (try 1 (catch (ValueError e) 0))))", "s");
          },
          [](P pc) { return istr(pc.i("n")); }, n_iters, true, false, s, sm);
    }
    add(
        "raise_depth",
        "a raise d calls deep, caught at the top (traceback capture and the unwinding), n times",
        [](P pc) {
          return loop_src(
              pc.i("n"),
              "(defun r (d) (if (= d 0) (raise (ValueError \"x\")) (+ 1 (r (- d 1))))) "
              "(set c 0)",
              "(set c (+ c (try (r " + istr(pc.i("d")) + ") (catch (ValueError e) 1))))", "c");
        },
        [](P pc) { return istr(pc.i("n")); }, n_iters, true, false,
        {{"n", {"2000"}}, {"d", {"0", "16", "64"}}}, {{"n", {"50"}}, {"d", {"0", "16"}}});
    {
      auto [s, sm] = n_sizes(40000, 300);
      add(
          "yield_gen", "for over a DSL generator that yields n values (stackless only: yield)",
          [](P pc) {
            return "(begin (set g (generator (lambda () (begin (set j 0) (while (< j " +
                   istr(pc.i("n")) +
                   ") (begin (yield j) (set j (+ j 1)))))))) (set s 0) (for x g (set s (+ s x))) "
                   "s)";
          },
          [](P pc) { return istr(pc.i("n") * (pc.i("n") - 1) / 2); }, n_iters, false, false, s, sm);
    }
    add(
        "tail_rec",
        "a self-recursive defun in tail position n deep (no TCO: the stack and heap grow; "
        "stackless and schedulers only, the recursive evaluator would overflow the C++ stack)",
        [](P pc) {
          return "(begin (defun lp (n acc) (if (= n 0) acc (lp (- n 1) (+ acc 1)))) (lp " +
                 istr(pc.i("n")) + " 0))";
        },
        [](P pc) { return istr(pc.i("n")); }, n_iters, false, true,
        {{"n", {"1000", "10000", "100000"}}}, {{"n", {"1000"}}});
    {
      auto [s, sm] = n_sizes(20000, 300);
      add(
          "range_create", "(next (range 3)) per iteration: a generator built and stepped once",
          [](P pc) {
            return loop_src(pc.i("n"), "(set s 0)", "(set s (+ s 1 (next (range 3))))", "s");
          },
          [](P pc) { return istr(pc.i("n")); }, n_iters, true, false, s, sm);
    }
    add(
        "method_send",
        "(send o \"m\" s) where m is defined `depth` classes up the __super__ chain (stackless "
        "only: send calls a DSL method from a native)",
        [](P pc) {
          const int64_t depth = std::max<int64_t>(1, pc.i("depth"));
          std::string setup = "(defclass A0 (m (self x) (+ x 1)))";
          for (int64_t d = 1; d < depth; ++d)
            setup += " (defclass A" + istr(d) + " (A" + istr(d - 1) + "))";
          setup += " (set o (A" + istr(depth - 1) + ")) (set s 0)";
          return loop_src(pc.i("n"), setup, "(set s (send o \"m\" s))", "s");
        },
        [](P pc) { return istr(pc.i("n")); }, n_iters, false, false,
        {{"n", {"20000"}}, {"depth", {"1", "4"}}}, {{"n", {"300"}}, {"depth", {"1", "4"}}});
    return v;
  }();
  return p;
}

void check_result(rep_out &o, const se::value_t &got, const std::string &expect) {
  const std::string g = se::to_string(got);
  if (g != expect)
    o.fail("returned " + g + ", expected " + expect);
}

// A program with its process killed or failed: the reason, for the record.
std::string end_reason(world &w, int pid) {
  auto info = w.info(pid);
  if (!info)
    return "pid " + std::to_string(pid) + " vanished";
  return std::string(se::to_string(info->status)) + ": " + info->exit_error.value_or("");
}

// Run one pre-parsed program as a single process to completion on w; the measured region is
// submit through completion. Returns its pid.
int run_single(world &w, point_ctx &pc, meter &m, rep_out &o, const se::value_t &ast,
               const std::string &expect,
               std::function<void(se::execute_options &)> tune = nullptr) {
  tracker t(1);
  step_kinds kinds;
  se::execute_options opts;
  opts.env = w.process_env("extend");
  opts.on_complete = t.hook(0);
  if (tune)
    tune(opts);
  const uint64_t steps0 = w.total_steps();
  m.begin();
  t.submit_ns[0] = now_ns();
  t.pid[0] = w.submit(ast, opts);
  const std::string why = drive(w, driver_of(pc), [&] { return t.all_done(); }, [] {});
  m.end();
  o.steps_total = w.total_steps() - steps0;
  if (auto info = w.info(t.pid[0]))
    kinds.add(*info);
  o.kinds = kinds;
  if (!why.empty())
    o.fail(why + ": " + end_reason(w, t.pid[0]));
  else if (!t.finish_ns[0])
    o.fail(end_reason(w, t.pid[0]));
  else
    check_result(o, t.result[0], expect);
  return t.pid[0];
}

// Run a pre-parsed program to completion on a bare stackless evaluator in env, measured.
void run_stackless(meter &m, rep_out &o, const se::environment_ptr &env, const se::value_t &ast,
                   const std::string &expect) {
  se::stackless_evaluator ev(env);
  m.begin();
  se::evaluator_state st = ev.create_state(ast);
  while (!ev.step(st)) {
  }
  m.end();
  o.steps_total = st.stats.get_step_count();
  o.kinds.add(st.stats);
  check_result(o, st.result, expect);
}

// Processes waiting now (on a message, a sleep or a call).
int64_t waiting_count(world &w) {
  const se::scheduler_stats s = w.base().get_stats();
  return s.total_processes - s.running - s.ready - s.paused - s.terminated - s.killed - s.failed;
}

// Run w until `want` processes wait (outside any measured region). The sync scheduler counts
// message waiters as runnable, so it runs bounded slices and counts. Returns "" or why not.
std::string wait_parked(world &w, int64_t want, double timeout) {
  const auto deadline = clk::now() + std::chrono::duration<double>(timeout);
  for (;;) {
    if (w.async) {
      w.async->drain_ingress();
      if (w.async->has_runnable())
        w.async->sync_run(200000, 0.1);
    } else {
      w.sync->run(static_cast<uint64_t>(4 * want + 100), std::nullopt);
    }
    const int64_t waiting = waiting_count(w);
    if (waiting >= want)
      return "";
    if (clk::now() > deadline || (w.async && !w.async->has_runnable()))
      return istr(waiting) + " of " + istr(want) + " processes parked";
  }
}

// Submit b bystander processes, each `body(i)`, and park them all.
std::string park_bystanders(world &w, int64_t b, const std::function<std::string(int64_t)> &body,
                            double timeout) {
  if (b <= 0)
    return "";
  const int64_t before = waiting_count(w);
  for (int64_t i = 0; i < b; ++i) {
    se::execute_options opts;
    opts.env = w.process_env("extend");
    w.submit(body(i), opts);
  }
  return wait_parked(w, before + b, timeout);
}

std::string parked_on_never(int64_t) { return "(begin (msg-recv \"bench.never\") 1)"; }

// Exactly `steps` scheduler steps (for loops that never finish), outside or inside a region.
void run_steps(world &w, uint64_t steps) {
  if (w.sync)
    w.sync->run(steps, std::nullopt);
  else
    w.async->sync_run(steps, std::nullopt);
}

// A loop that runs far longer than any measurement: killed once measured.
const se::value_t &forever_ast() {
  static const se::value_t a =
      se::parse("(begin (set i 0) (while (< i 1000000000) (set i (+ i 1))) i)");
  return a;
}

// A JSON array of integers.
std::string json_ints(const std::vector<int64_t> &v) {
  std::string s = "[";
  for (std::size_t i = 0; i < v.size(); ++i)
    s += (i ? ", " : "") + istr(v[i]);
  return s + "]";
}

int64_t median_of(std::vector<int64_t> v) {
  if (v.empty())
    return 0;
  std::sort(v.begin(), v.end());
  return pct_at(v, 0.5);
}

// Submit `src` (one program each), drive them to completion and check them, outside any meter:
// a batch of a scenario that times several. Returns "" or why it failed.
std::string run_batch(world &w, const point_ctx &pc, const std::vector<std::string> &src,
                      const std::vector<std::string> &expect, bool reap, step_kinds &kinds,
                      std::vector<int64_t> *latency = nullptr) {
  tracker t(src.size());
  for (std::size_t i = 0; i < src.size(); ++i) {
    se::execute_options opts;
    opts.env = w.process_env("extend");
    opts.on_complete = t.hook(i);
    t.submit_ns[i] = now_ns();
    t.pid[i] = w.submit(src[i], opts);
  }
  const std::string why = drive(
      w, driver_of(pc), [&] { return t.all_done(); }, [&] { sweep(w, t, reap, false, kinds); });
  if (!why.empty())
    return why + ": " + first_failure(w, t);
  for (std::size_t i = 0; i < src.size(); ++i) {
    const std::string got = se::to_string(t.result[i]);
    if (got != expect[i])
      return "program " + istr(int64_t(i)) + " returned " + got + ", expected " + expect[i];
    if (latency)
      latency->push_back(t.finish_ns[i] - t.submit_ns[i]);
  }
  return "";
}

#if defined(_WIN32)
// The efficiency class of logical CPU lp (Windows CPU sets), or -1.
int lp_efficiency(int lp) {
  static const std::vector<int> eff = [] {
    std::vector<int> e;
    using get_sets_fn = BOOL(WINAPI *)(PSYSTEM_CPU_SET_INFORMATION, ULONG, PULONG, HANDLE, ULONG);
    auto get_sets = reinterpret_cast<get_sets_fn>(reinterpret_cast<void *>(
        GetProcAddress(GetModuleHandleW(L"kernel32.dll"), "GetSystemCpuSetInformation")));
    if (!get_sets)
      return e;
    ULONG len = 0;
    get_sets(nullptr, 0, &len, GetCurrentProcess(), 0);
    std::vector<unsigned char> buf(len);
    if (!len || !get_sets(reinterpret_cast<PSYSTEM_CPU_SET_INFORMATION>(buf.data()), len, &len,
                          GetCurrentProcess(), 0))
      return e;
    for (ULONG off = 0; off < len;) {
      auto *s = reinterpret_cast<SYSTEM_CPU_SET_INFORMATION *>(buf.data() + off);
      if (s->Type == CpuSetInformation && s->CpuSet.Group == 0) {
        if (e.size() <= s->CpuSet.LogicalProcessorIndex)
          e.resize(s->CpuSet.LogicalProcessorIndex + 1, -1);
        e[s->CpuSet.LogicalProcessorIndex] = s->CpuSet.EfficiencyClass;
      }
      off += s->Size;
    }
    return e;
  }();
  return lp >= 0 && std::size_t(lp) < eff.size() ? eff[std::size_t(lp)] : -1;
}
#endif

// The efficiency class of the CPU the calling thread runs on now (Windows), or -1.
int current_cpu_efficiency() {
#if defined(_WIN32)
  PROCESSOR_NUMBER pn{};
  GetCurrentProcessorNumberEx(&pn);
  return pn.Group == 0 ? lp_efficiency(pn.Number) : -1;
#else
  return -1;
#endif
}

// ---------------------------------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------------------------------

std::vector<scenario> make_scenarios() {
  std::vector<scenario> all;
  const std::vector<std::string> both_sched = {"sync", "async"};

  // A. eval.*: the bare evaluators, run to completion on a pre-parsed program.
  for (const eval_prog &p : eval_programs()) {
    scenario sc;
    sc.name = "eval." + p.name;
    sc.what = "bare evaluator: " + p.what;
    sc.executors = p.recursive_ok ? std::vector<std::string>{"recursive", "stackless"}
                                  : std::vector<std::string>{"stackless"};
    sc.sizes = p.sizes;
    sc.smoke = p.smoke;
    sc.want_heap = true;
    const eval_prog *pp = &p;
    sc.run = [pp](point_ctx &pc) {
      auto env = se::builtins::make_default_environment();
      add_bench_natives(env);
      const se::value_t ast = se::parse(pp->src(pc));
      const std::string expect = pp->expect(pc);
      const bool recursive = pc.executor == "recursive";
      pc.reps([&](meter &m, rep_out &o) {
        o.iterations = pp->iters(pc);
        o.programs = 1;
        if (recursive) {
          se::evaluator ev(env);
          m.begin();
          const se::value_t r = ev.evaluate(ast);
          m.end();
          o.steps_total = ev.stats().get_step_count();
          o.kinds.add(ev.stats());
          check_result(o, r, expect);
        } else {
          run_stackless(m, o, env, ast, expect);
        }
      });
    };
    all.push_back(std::move(sc));
  }

  // B. sched.single.*: the same programs as one process under each scheduler.
  for (const eval_prog &p : eval_programs()) {
    if (!p.sched)
      continue;
    scenario sc;
    sc.name = "sched.single." + p.name;
    sc.what = "one process on a scheduler: " + p.what;
    sc.executors = both_sched;
    sc.sizes = p.sizes;
    sc.smoke = p.smoke;
    sc.sizes.push_back({"driver", {"spin"}});
    sc.smoke.push_back({"driver", {"spin"}});
    sc.async_only = {"driver"};
    sc.want_heap = true;
    const eval_prog *pp = &p;
    sc.run = [pp](point_ctx &pc) {
      const se::value_t ast = se::parse(pp->src(pc));
      const std::string expect = pp->expect(pc);
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        o.iterations = pp->iters(pc);
        o.programs = 1;
        run_single(w, pc, m, o, ast, expect);
      });
    };
    all.push_back(std::move(sc));
  }

  // The flood programs: (+ 1 2) and a 10-iteration loop, alternating.
  const auto tiny_flood = [](int64_t n, std::vector<std::string> &src,
                             std::vector<std::string> &expect) {
    for (int64_t i = 0; i < n; ++i) {
      if (i % 2 == 0) {
        src.push_back("(+ 1 2)");
        expect.push_back("3");
      } else {
        src.push_back(loop_src(10, "(set s 0)", "(set s (+ s i))", "s"));
        expect.push_back("45");
      }
    }
  };

  // C. flood.small: n tiny programs at once.
  {
    scenario sc;
    sc.name = "flood.small";
    sc.what = "n tiny programs ((+ 1 2) and a 10-iteration loop) submitted at once, run to the "
              "end; per-program latency, throughput, memory; reap or not on async; each in its "
              "own scope (env=extend) or a whole default environment (env=own, as Ariadne)";
    sc.executors = both_sched;
    // Every step scans every process the scheduler holds (no run queue), so a flood submitted at
    // once costs O(n^2): n=1000 takes ~2-3 s a rep here and n=10000 several minutes. Sweep further
    // with --size n=3000,10000 --timeout 900.
    sc.sizes = {{"n", {"100", "300", "1000"}},
                {"reap", {"1", "0"}},
                {"env", {"extend", "own"}},
                {"driver", {"spin"}}};
    sc.smoke = {
        {"n", {"50"}}, {"reap", {"1", "0"}}, {"env", {"extend", "own"}}, {"driver", {"spin"}}};
    sc.async_only = {"reap", "driver"};
    sc.run = [tiny_flood](point_ctx &pc) {
      std::vector<std::string> src, expect;
      tiny_flood(pc.i("n"), src, expect);
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        auto f = run_flood(w, pc, m, o, src, expect, pc.s("env", "extend"), pc.i("reap", 1) != 0);
        o.latency_ns = std::move(f.latency);
        o.extra.i("processes_left", w.process_count());
      });
    };
    all.push_back(std::move(sc));
  }

  // C'. flood.small_host: the same flood driven as Ariadne drives its actions.
  {
    scenario sc;
    sc.name = "flood.small_host";
    sc.what = "flood.small on the async scheduler, host=bare (reap what just finished, no "
              "limits) vs host=ariadne (Ariadne's action limits armed, every live process swept "
              "each pump); watch=0 drops the tree watch root (the per-submit settings reads)";
    sc.executors = {"async"};
    sc.sizes = {{"n", {"300"}},
                {"host", {"bare", "ariadne"}},
                {"watch", {"1", "0"}},
                {"env", {"extend", "own"}},
                {"driver", {"spin"}}};
    sc.smoke = {{"n", {"30"}},
                {"host", {"bare", "ariadne"}},
                {"watch", {"1", "0"}},
                {"env", {"extend", "own"}},
                {"driver", {"spin"}}};
    sc.run = [tiny_flood](point_ctx &pc) {
      std::vector<std::string> src, expect;
      tiny_flood(pc.i("n"), src, expect);
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        if (pc.i("watch", 1) == 0)
          w.async->set_watch_root(nullptr);
        auto f = run_flood(w, pc, m, o, src, expect, pc.s("env", "extend"), true);
        o.latency_ns = std::move(f.latency);
        o.extra.i("processes_left", w.process_count());
      });
    };
    all.push_back(std::move(sc));
  }

  // D. flood.large: p long programs; the same total work at every p.
  {
    scenario sc;
    sc.name = "flood.large";
    sc.what = "p loops at once, work/p iterations each (the same total work at every p, so at "
              "large p each is short and parse/submit/exit weigh in: see sched.scaling for the "
              "pure per-step cost); ns/step vs p and the spread of finish times (fairness)";
    sc.executors = both_sched;
    sc.sizes = {{"p", {"1", "10", "100", "1000"}}, {"work", {"10000"}}, {"driver", {"spin"}}};
    sc.smoke = {{"p", {"1", "10"}}, {"work", {"2000"}}, {"driver", {"spin"}}};
    sc.async_only = {"driver"};
    sc.run = [](point_ctx &pc) {
      const int64_t p = std::max<int64_t>(1, pc.i("p"));
      const int64_t l = std::max<int64_t>(1, pc.i("work") / p);
      std::vector<std::string> src(p, loop_src(l, "(set s 0)", "(set s (+ s i))", "s"));
      std::vector<std::string> expect(p, istr(l * (l - 1) / 2));
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        auto f = run_flood(w, pc, m, o, src, expect, "extend", true);
        o.iterations = p * l;
        if (!f.finish.empty()) {
          const auto [lo, hi] = std::minmax_element(f.finish.begin(), f.finish.end());
          o.extra.i("iterations_per_program", l)
              .i("first_finish_ns", *lo)
              .i("last_finish_ns", *hi)
              .d("finish_spread", *hi > 0 ? double(*hi - *lo) / double(*hi) : 0.0);
        }
        o.latency_ns = std::move(f.latency);
      });
    };
    all.push_back(std::move(sc));
  }

  // E. flood.mixed: many small programs behind a few large ones, all submitted at once.
  {
    scenario sc;
    sc.name = "flood.mixed";
    sc.what = "large long loops submitted first, then small programs, all at once: the small "
              "ones' latency behind the large (and, on sync, the large loops' steps scanning the "
              "finished small ones; see flood.mixed_stream for a steady live set)";
    sc.executors = both_sched;
    sc.sizes = {{"small", {"1000"}}, {"large", {"4"}}, {"l", {"2000"}}, {"driver", {"spin"}}};
    sc.smoke = {{"small", {"50"}}, {"large", {"2"}}, {"l", {"1000"}}, {"driver", {"spin"}}};
    sc.async_only = {"driver"};
    sc.run = [](point_ctx &pc) {
      const int64_t small = pc.i("small"), large = pc.i("large"), l = pc.i("l");
      std::vector<std::string> src, expect;
      for (int64_t i = 0; i < large; ++i) {
        src.push_back(loop_src(l, "(set s 0)", "(set s (+ s i))", "s"));
        expect.push_back(istr(l * (l - 1) / 2));
      }
      for (int64_t i = 0; i < small; ++i) {
        src.push_back("(+ 1 2)");
        expect.push_back("3");
      }
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        auto f = run_flood(w, pc, m, o, src, expect, "extend", true);
        if (f.latency.size() == src.size()) {
          o.latency_ns.assign(f.latency.begin() + large, f.latency.end()); // the small ones
          std::vector<int64_t> big(f.latency.begin(), f.latency.begin() + large);
          o.extra.raw("latency_large_ns", percentiles(big).str());
        }
      });
    };
    all.push_back(std::move(sc));
  }

  // E'. flood.mixed_stream: small programs one per pump, behind a steady set of long ones.
  {
    scenario sc;
    sc.name = "flood.mixed_stream";
    sc.what = "`live` endless loops running, then `small` (+ 1 2) programs submitted one per "
              "pump of `slice` steps, as Ariadne submits an action per frame: each small one's "
              "submit->finish latency at a fixed live set";
    sc.executors = both_sched;
    sc.sizes = {{"live", {"10", "100"}}, {"small", {"300"}}, {"slice", {"2000"}}};
    sc.smoke = {{"live", {"2"}}, {"small", {"20"}}, {"slice", {"500"}}};
    sc.run = [](point_ctx &pc) {
      const int64_t live = pc.i("live"), small = pc.i("small");
      const uint64_t slice = static_cast<uint64_t>(pc.i("slice"));
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        std::vector<int> loops;
        for (int64_t j = 0; j < live; ++j) {
          se::execute_options opts;
          opts.env = w.process_env("extend");
          loops.push_back(w.submit(forever_ast(), opts));
        }
        run_steps(w, 1000);
        tracker t(static_cast<std::size_t>(small));
        step_kinds kinds;
        const uint64_t steps0 = w.total_steps();
        const auto deadline = clk::now() + std::chrono::duration<double>(pc.cfg.timeout);
        std::size_t next = 0;
        m.begin();
        while (!t.all_done()) {
          if (w.async) {
            w.async->drain_ingress();
            w.async->wake_awaiting();
          }
          if (next < t.pid.size()) {
            se::execute_options opts;
            opts.env = w.process_env("extend");
            opts.on_complete = t.hook(next);
            t.submit_ns[next] = now_ns();
            t.pid[next] = w.submit(std::string("(+ 1 2)"), opts);
            ++next;
          }
          if (w.sync)
            w.sync->run(slice, std::nullopt);
          else
            w.async->sync_run(slice, 0.1);
          sweep(w, t, true, false, kinds);
          if (clk::now() > deadline) {
            o.fail("timeout: " + first_failure(w, t));
            break;
          }
        }
        m.end();
        o.steps_total = w.total_steps() - steps0;
        o.programs = small;
        for (std::size_t i = 0; i < t.pid.size(); ++i) {
          if (!t.finish_ns[i])
            continue;
          if (se::to_string(t.result[i]) != "3")
            o.fail("program " + istr(int64_t(i)) + " returned " + se::to_string(t.result[i]));
          o.latency_ns.push_back(t.finish_ns[i] - t.submit_ns[i]);
        }
        for (int pid : loops) {
          auto info = w.info(pid);
          if (!info || se::is_finished(info->status))
            o.fail("a long loop ended: " + end_reason(w, pid));
          w.kill(pid);
        }
        o.extra.i("processes_left", w.process_count());
      });
    };
    all.push_back(std::move(sc));
  }

  // E''. flood.stream: batches into one scheduler, each run to the end before the next.
  {
    scenario sc;
    sc.name = "flood.stream";
    sc.what = "r batches of b (+ 1 2) programs into ONE scheduler, each batch run to the end "
              "before the next: batch time against the batch index is what the processes kept "
              "after they finished cost (the sync scheduler never erases one; async reap=0)";
    sc.executors = both_sched;
    sc.sizes = {{"r", {"20"}}, {"b", {"500"}}, {"reap", {"1", "0"}}, {"driver", {"spin"}}};
    sc.smoke = {{"r", {"3"}}, {"b", {"20"}}, {"reap", {"1", "0"}}, {"driver", {"spin"}}};
    sc.async_only = {"reap", "driver"};
    sc.run = [](point_ctx &pc) {
      const int64_t r = pc.i("r"), b = pc.i("b");
      const std::vector<std::string> src(b, "(+ 1 2)"), expect(b, "3");
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        step_kinds kinds;
        std::vector<int64_t> batch_ns;
        const uint64_t steps0 = w.total_steps();
        m.begin();
        for (int64_t j = 0; j < r && o.ok; ++j) {
          const int64_t t0 = now_ns();
          const std::string why = run_batch(w, pc, src, expect, pc.i("reap", 1) != 0, kinds);
          batch_ns.push_back(now_ns() - t0);
          if (!why.empty())
            o.fail("batch " + istr(j) + ": " + why);
        }
        m.end();
        o.steps_total = w.total_steps() - steps0;
        o.programs = r * b;
        if (!batch_ns.empty())
          o.extra.i("batch_first_ns", batch_ns.front())
              .i("batch_last_ns", batch_ns.back())
              .d("batch_last_over_first", double(batch_ns.back()) / double(batch_ns.front()))
              .raw("batch_ns", json_ints(batch_ns));
        o.extra.i("processes_left", w.process_count());
      });
    };
    all.push_back(std::move(sc));
  }

  // sched.scaling / sched.bystanders / sched.watch_poll: steady-state ns per step.
  {
    scenario sc;
    sc.name = "sched.scaling";
    sc.what = "p endless loops, warmed up; then exactly `steps` steps timed: ns/step against the "
              "number of runnable processes (the per-step scans alone, no submit or exit), and "
              "the min/max steps a process got (fairness)";
    sc.executors = both_sched;
    // p=3000 took 6 s (sync) and 10 s (async) a rep here.
    sc.sizes = {{"p", {"1", "10", "100", "1000", "2000"}}, {"steps", {"50000"}}};
    sc.smoke = {{"p", {"1", "10"}}, {"steps", {"2000"}}};
    sc.run = [](point_ctx &pc) {
      const int64_t p = pc.i("p");
      const uint64_t steps = static_cast<uint64_t>(pc.i("steps"));
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        std::vector<int> pids;
        for (int64_t j = 0; j < p; ++j) {
          se::execute_options opts;
          opts.env = w.process_env("extend");
          pids.push_back(w.submit(forever_ast(), opts));
        }
        run_steps(w, static_cast<uint64_t>(2 * p + 1000)); // every process has started
        const uint64_t steps0 = w.total_steps();
        m.begin();
        run_steps(w, steps);
        m.end();
        o.steps_total = w.total_steps() - steps0;
        if (*o.steps_total != steps)
          o.fail("ran " + istr(int64_t(*o.steps_total)) + " steps, expected " +
                 istr(int64_t(steps)));
        uint64_t lo = UINT64_MAX, hi = 0;
        for (int pid : pids) {
          auto info = w.info(pid);
          if (!info || se::is_finished(info->status)) {
            o.fail("a loop ended: " + end_reason(w, pid));
            continue;
          }
          lo = std::min(lo, info->eval_steps);
          hi = std::max(hi, info->eval_steps);
        }
        if (hi)
          o.extra.i("process_steps_min", int64_t(lo))
              .i("process_steps_max", int64_t(hi))
              .d("fairness", double(lo) / double(hi));
        for (int pid : pids)
          w.kill(pid);
      });
    };
    all.push_back(std::move(sc));
  }
  for (const bool watch : {false, true}) {
    scenario sc;
    sc.name = watch ? "sched.watch_poll" : "sched.bystanders";
    sc.what = watch ? "one endless loop beside w parked processes that each registered a "
                      "state-watch (watch=1) or not (watch=0, the control): the loop's ns/step "
                      "against w, every step polling every watch"
                    : "one endless loop beside b processes parked on msg-recv: the loop's ns/step "
                      "against b (the scans walk parked processes too)";
    sc.executors = both_sched;
    const std::string key = watch ? "w" : "b";
    if (watch) {
      sc.sizes = {{"w", {"0", "10", "100", "1000"}}, {"watch", {"1", "0"}}, {"steps", {"50000"}}};
      sc.smoke = {{"w", {"0", "5"}}, {"watch", {"1", "0"}}, {"steps", {"2000"}}};
    } else {
      sc.sizes = {{"b", {"0", "100", "1000", "3000"}}, {"steps", {"50000"}}};
      sc.smoke = {{"b", {"0", "10"}}, {"steps", {"2000"}}};
    }
    sc.run = [key](point_ctx &pc) {
      const int64_t b = pc.i(key);
      const bool watches = pc.i("watch", 0) != 0;
      const uint64_t steps = static_cast<uint64_t>(pc.i("steps"));
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        const std::string why = park_bystanders(
            w, b,
            [watches](int64_t i) {
              return watches ? "(begin (state-watch \"bench.w" + istr(i) +
                                   "\" (lambda (v) v)) (msg-recv \"bench.never\") 1)"
                             : parked_on_never(i);
            },
            pc.cfg.timeout);
        if (!why.empty()) {
          o.fail("parking: " + why);
          return;
        }
        se::execute_options opts;
        opts.env = w.process_env("extend");
        const int loop = w.submit(forever_ast(), opts);
        run_steps(w, 1000);
        const uint64_t loop0 = w.info(loop) ? w.info(loop)->eval_steps : 0;
        const uint64_t steps0 = w.total_steps();
        m.begin();
        run_steps(w, steps);
        m.end();
        o.steps_total = w.total_steps() - steps0;
        auto info = w.info(loop);
        if (!info || se::is_finished(info->status))
          o.fail("the loop ended: " + end_reason(w, loop));
        else
          o.extra.d("loop_share_of_steps", double(info->eval_steps - loop0) /
                                               double(std::max<uint64_t>(1, *o.steps_total)));
        if (waiting_count(w) != b)
          o.fail(istr(waiting_count(w)) + " processes parked, expected " + istr(b));
        o.extra.i("parked", b);
        w.kill(loop);
      });
    };
    all.push_back(std::move(sc));
  }

  // sched.sleepers: processes that sleep; what the driver burns while they do.
  {
    scenario sc;
    sc.name = "sched.sleepers";
    sc.what = "n processes each sleeping 1 ms 20 times: driver CPU / wall (1.0 = the driver "
              "spins while everything sleeps), wake lateness (latency - 20 ms) and steps per "
              "wall second";
    sc.executors = both_sched;
    // n=1000 takes 13-21 s a rep here (every wake and every idle spin scans every process).
    sc.sizes = {{"n", {"1", "100", "300"}}, {"driver", {"spin", "poll1ms"}}};
    sc.smoke = {{"n", {"1", "10"}}, {"driver", {"spin", "poll1ms"}}};
    sc.async_only = {"driver"};
    sc.run = [](point_ctx &pc) {
      const int64_t n = pc.i("n");
      const std::vector<std::string> src(n, loop_src(20, "", "(sleep 0.001)", "i")),
          expect(n, "20");
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        tracker t(static_cast<std::size_t>(n));
        step_kinds kinds;
        drive_cfg d = driver_of(pc);
        d.sleepers = true;
        const uint64_t steps0 = w.total_steps();
        m.begin();
        for (int64_t i = 0; i < n; ++i) {
          se::execute_options opts;
          opts.env = w.process_env("extend");
          opts.on_complete = t.hook(std::size_t(i));
          t.submit_ns[std::size_t(i)] = now_ns();
          t.pid[std::size_t(i)] = w.submit(src[std::size_t(i)], opts);
        }
        const std::string why =
            drive(w, d, [&] { return t.all_done(); }, [&] { sweep(w, t, true, false, kinds); });
        m.end();
        o.steps_total = w.total_steps() - steps0;
        o.programs = n;
        if (!why.empty())
          o.fail(why + ": " + first_failure(w, t));
        std::vector<int64_t> late;
        for (std::size_t i = 0; i < t.pid.size(); ++i) {
          if (!t.finish_ns[i])
            continue;
          if (se::to_string(t.result[i]) != expect[i])
            o.fail("program " + istr(int64_t(i)) + " returned " + se::to_string(t.result[i]));
          o.latency_ns.push_back(t.finish_ns[i] - t.submit_ns[i]);
          late.push_back(t.finish_ns[i] - t.submit_ns[i] - 20000000);
        }
        const double wall = double(m.wall());
        o.extra.raw("lateness_ns", percentiles(late).str())
            .d("driver_cpu_per_wall", wall > 0 ? double(m.b.thread_cpu - m.a.thread_cpu) / wall : 0)
            .d("steps_per_wall_s", wall > 0 ? double(*o.steps_total) * 1e9 / wall : 0);
      });
    };
    all.push_back(std::move(sc));
  }

  // sched.gc_spike: the longest single steps (collection and release are unsliced).
  {
    scenario sc;
    sc.name = "sched.gc_spike";
    sc.what = "one process holding a list of n items while it makes 2000 closures in fresh "
              "scopes, stepped one step() at a time with each step timed: step latency "
              "p50/p99/max (an unsliced collection or release shows as the max)";
    sc.executors = {"sync"};
    sc.sizes = {{"n", {"0", "10000", "100000"}}};
    sc.smoke = {{"n", {"0", "1000"}}};
    sc.run = [](point_ctx &pc) {
      const int64_t n = pc.i("n");
      const se::value_t ast =
          se::parse("(begin (set big (collect (range " + istr(n) +
                    "))) (defun mk (x) (lambda () x)) (set i 0) (while (< i 2000) (begin (set f "
                    "(mk i)) (set i (+ i 1)))) (length big))");
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        tracker t(1);
        se::execute_options opts;
        opts.env = w.process_env("extend");
        opts.on_complete = t.hook(0);
        std::vector<int64_t> step_ns;
        step_ns.reserve(200000);
        const uint64_t steps0 = w.total_steps();
        const auto deadline = clk::now() + std::chrono::duration<double>(pc.cfg.timeout);
        m.begin();
        t.pid[0] = w.submit(ast, opts);
        while (!t.all_done() && w.sync->has_runnable()) {
          const int64_t t0 = now_ns();
          w.sync->step();
          step_ns.push_back(now_ns() - t0);
          if ((step_ns.size() & 4095) == 0 && clk::now() > deadline)
            break;
        }
        m.end();
        o.steps_total = w.total_steps() - steps0;
        if (!t.all_done())
          o.fail(end_reason(w, t.pid[0]));
        else
          check_result(o, t.result[0], istr(n));
        o.extra.raw("step_ns", percentiles(step_ns).str());
      });
    };
    all.push_back(std::move(sc));
  }

  // F. async.flood.*: n programs, each calling an async function once.
  const auto where_ok = [](const std::string &exec, const point_ctx &pc) {
    // Inline kernels are the sync scheduler's; background and lane need a parking scheduler.
    return (exec == "sync") == (pc.s("where") == "inline");
  };
  for (const char *kernel : {"noop", "sleep1ms", "cpu100us"}) {
    scenario sc;
    sc.name = std::string("async.flood.") + kernel;
    sc.what = std::string("n programs each calling an async function whose kernel is ") + kernel +
              "; background (app pool), lane (async_lane) or inline (sync scheduler); a fresh "
              "app per rep";
    sc.executors = {"async", "sync"};
    sc.sizes = {{"n", {"10", "100", "1000"}},
                {"where", {"background", "lane", "inline"}},
                {"driver", {"spin", "poll1ms"}}};
    sc.smoke = {{"n", {"10"}},
                {"where", {"background", "lane", "inline"}},
                {"driver", {"spin", "poll1ms"}}};
    sc.async_only = {"driver"};
    sc.accept = where_ok;
    sc.bg_calls_key = "n";
    sc.spawns_threads = true;
    const std::string fn = std::string("bench-") + kernel;
    sc.run = [fn](point_ctx &pc) {
      const int64_t n = pc.i("n");
      std::vector<std::string> src, expect;
      for (int64_t i = 0; i < n; ++i) {
        src.push_back("(" + fn + " " + istr(i) + ")");
        expect.push_back(istr(i));
      }
      pc.reps([&](meter &m, rep_out &o) {
        // A fresh app per rep (built before the region, destroyed after it): the background
        // executor accumulates state in the app, which async.pool_aging measures on its own.
        auto app = make_app();
        auto log = std::make_shared<kernel_log>();
        log->reset(static_cast<std::size_t>(n));
        world w(pc.executor, app.get());
        w.make_host(pc.s("where"));
        register_kernels(w, log, pc.s("where"));
        m.app = app.get();
        auto f = run_flood(w, pc, m, o, src, expect, "extend", true);
        o.async_calls = n;
        o.latency_ns = f.latency;
        for (std::size_t i = 0; i < f.finish.size() && f.finish.size() == std::size_t(n); ++i) {
          const int64_t end = log->at(i);
          if (end)
            o.async_latency_ns.push_back(m.a.wall + f.finish[i] - end);
        }
        o.extra.s("kernel", fn.substr(6)).i("kernel_threads", int64_t(log->thread_count()));
      });
    };
    all.push_back(std::move(sc));
  }

  // F'. async.futures: one program launching k futures and awaiting them all.
  {
    scenario sc;
    sc.name = "async.futures";
    sc.what = "one program launching k <fn>-async futures, then await-all; a fresh app per rep";
    sc.executors = {"async", "sync"};
    sc.sizes = {{"k", {"10", "100", "1000"}},
                {"kernel", {"noop", "sleep1ms"}},
                {"where", {"background", "lane", "inline"}},
                {"driver", {"spin", "poll1ms"}}};
    sc.smoke = {{"k", {"10"}},
                {"kernel", {"noop", "sleep1ms"}},
                {"where", {"background", "lane", "inline"}},
                {"driver", {"spin", "poll1ms"}}};
    sc.async_only = {"driver"};
    sc.accept = where_ok;
    sc.bg_calls_key = "k";
    sc.spawns_threads = true;
    sc.run = [](point_ctx &pc) {
      const int64_t k = pc.i("k");
      const std::string fn = "bench-" + pc.s("kernel") + "-async";
      const std::string src = "(begin (set fs (list)) (set i 0) (while (< i " + istr(k) +
                              ") (begin (append fs (" + fn +
                              " i)) (set i (+ i 1)))) (apply + (await-all fs)))";
      pc.reps([&](meter &m, rep_out &o) {
        auto app = make_app();
        auto log = std::make_shared<kernel_log>();
        log->reset(static_cast<std::size_t>(k));
        world w(pc.executor, app.get());
        w.make_host(pc.s("where"));
        register_kernels(w, log, pc.s("where"));
        m.app = app.get();
        auto f = run_flood(w, pc, m, o, {src}, {istr(k * (k - 1) / 2)}, "extend", true);
        o.async_calls = k;
        o.latency_ns = f.latency;
        int64_t last = 0;
        for (std::size_t i = 0; i < std::size_t(k); ++i)
          last = std::max(last, log->at(i));
        if (!f.finish.empty() && last)
          o.extra.i("await_all_tail_ns", m.a.wall + f.finish[0] - last);
        o.extra.i("kernel_threads", int64_t(log->thread_count()));
      });
    };
    all.push_back(std::move(sc));
  }

  // async.pool_aging: one app, k sequential calls; per-call time early against late.
  {
    scenario sc;
    sc.name = "async.pool_aging";
    sc.what = "ONE program making k sequential async calls (noop kernel) on one app, at most one "
              "in flight: ns per call over the first and the last 10 % of calls (from the "
              "kernels' end times), the OS threads the kernels ran on, and the app's thread "
              "map before and after (entries it never erases)";
    sc.executors = {"async"};
    sc.sizes = {{"k", {"100", "1000", "3000"}}, {"where", {"background", "lane"}}};
    sc.smoke = {{"k", {"20"}}, {"where", {"background", "lane"}}};
    sc.spawns_threads = true;
    sc.run = [](point_ctx &pc) {
      const int64_t k = pc.i("k");
      const se::value_t ast = se::parse(loop_src(k, "", "(bench-noop i)", "i"));
      pc.reps([&](meter &m, rep_out &o) {
        auto app = make_app();
        auto log = std::make_shared<kernel_log>();
        log->reset(static_cast<std::size_t>(k));
        world w(pc.executor, app.get());
        w.make_host(pc.s("where"));
        register_kernels(w, log, pc.s("where"));
        m.app = app.get();
        o.iterations = k;
        o.async_calls = k;
        run_single(w, pc, m, o, ast, istr(k));
        std::vector<int64_t> ends;
        for (std::size_t i = 0; i < std::size_t(k); ++i)
          ends.push_back(log->at(i));
        const std::size_t d = std::max<std::size_t>(1, std::size_t(k) / 10);
        if (o.ok && ends.size() > d &&
            std::all_of(ends.begin(), ends.end(), [](int64_t e) { return e != 0; })) {
          const double first = double(ends[d] - ends[0]) / double(d);
          const double last = double(ends.back() - ends[ends.size() - 1 - d]) / double(d);
          o.extra.d("first_ns_per_call", first)
              .d("last_ns_per_call", last)
              .d("last_over_first", first > 0 ? last / first : 0.0);
        }
        o.extra.i("kernel_threads", int64_t(log->thread_count()));
      });
    };
    all.push_back(std::move(sc));
  }

  // async.pool_batches: batches of concurrent calls into one app.
  {
    scenario sc;
    sc.name = "async.pool_batches";
    sc.what = "r batches of b concurrent async calls (noop kernel, b programs each calling once) "
              "into ONE app and scheduler: batch time against the batch index";
    sc.executors = {"async"};
    sc.sizes = {{"r", {"20"}}, {"b", {"16"}}, {"where", {"background", "lane"}}};
    sc.smoke = {{"r", {"3"}}, {"b", {"4"}}, {"where", {"background", "lane"}}};
    sc.bg_calls_key = "b";
    sc.spawns_threads = true;
    sc.run = [](point_ctx &pc) {
      const int64_t r = pc.i("r"), b = pc.i("b");
      std::vector<std::string> src, expect;
      for (int64_t i = 0; i < b; ++i) {
        src.push_back("(bench-noop " + istr(i) + ")");
        expect.push_back(istr(i));
      }
      pc.reps([&](meter &m, rep_out &o) {
        auto app = make_app();
        auto log = std::make_shared<kernel_log>();
        log->reset(static_cast<std::size_t>(b));
        world w(pc.executor, app.get());
        w.make_host(pc.s("where"));
        register_kernels(w, log, pc.s("where"));
        m.app = app.get();
        step_kinds kinds;
        std::vector<int64_t> batch_ns;
        const uint64_t steps0 = w.total_steps();
        m.begin();
        for (int64_t j = 0; j < r && o.ok; ++j) {
          const int64_t t0 = now_ns();
          const std::string why = run_batch(w, pc, src, expect, true, kinds);
          batch_ns.push_back(now_ns() - t0);
          if (!why.empty())
            o.fail("batch " + istr(j) + ": " + why);
        }
        m.end();
        o.steps_total = w.total_steps() - steps0;
        o.async_calls = r * b;
        o.programs = r * b;
        if (!batch_ns.empty())
          o.extra.i("batch_first_ns", batch_ns.front())
              .i("batch_last_ns", batch_ns.back())
              .d("batch_last_over_first", double(batch_ns.back()) / double(batch_ns.front()))
              .raw("batch_ns", json_ints(batch_ns));
        o.extra.i("kernel_threads", int64_t(log->thread_count()));
      });
    };
    all.push_back(std::move(sc));
  }

  // async.pool_retention: what the host keeps of finished calls' results.
  {
    scenario sc;
    sc.name = "async.pool_retention";
    sc.what = "ONE program making k sequential calls whose kernel returns a 1 MiB string, "
              "results dropped: private and heap bytes per call left at the end (near 1 MiB = "
              "every call's result kept)";
    sc.executors = {"async"};
    sc.sizes = {{"k", {"100"}}, {"where", {"background", "lane"}}};
    sc.smoke = {{"k", {"5"}}, {"where", {"background", "lane"}}};
    sc.want_heap = true;
    sc.spawns_threads = true;
    sc.run = [](point_ctx &pc) {
      const int64_t k = pc.i("k");
      const se::value_t ast = se::parse(loop_src(k, "", "(bench-big i)", "i"));
      pc.reps([&](meter &m, rep_out &o) {
        auto app = make_app();
        auto log = std::make_shared<kernel_log>();
        log->reset(static_cast<std::size_t>(k));
        world w(pc.executor, app.get());
        w.make_host(pc.s("where"));
        register_kernels(w, log, pc.s("where"));
        m.app = app.get();
        o.iterations = k;
        o.async_calls = k;
        const int pid = run_single(w, pc, m, o, ast, istr(k));
        w.async->reap(pid);
        if (m.b.mem.has_priv)
          o.extra.d("private_bytes_per_call",
                    (double(m.b.mem.priv) - double(m.a.mem.priv)) / double(k));
        if (m.a.mem.has_heap)
          o.extra.d("heap_bytes_per_call",
                    double(int64_t(m.b.mem.heap) - int64_t(m.a.mem.heap)) / double(k));
      });
    };
    all.push_back(std::move(sc));
  }

  // async.inflight_tax: unrelated steps while async calls are in flight.
  {
    scenario sc;
    sc.name = "async.inflight_tax";
    sc.what = "c callers blocked in async calls that stay in flight (background pool, or c "
              "lanes), then one arith_loop process timed: its ns/step against c (every step "
              "copies the in-flight calls and charges each started one, reading the step rate "
              "from the tree unless the caller set it: rate=opt); watch=0 drops the watch root";
    sc.executors = {"async"};
    // rate=tree costs ~0.3 us a step per in-flight call here (c=20 on lanes: ~90 us a step), so
    // the loop is short and c stops at 8 by default; sweep further with --size c=20.
    sc.sizes = {{"c", {"0", "1", "4", "8"}},
                {"where", {"background", "lane"}},
                {"rate", {"tree", "opt"}},
                {"watch", {"1"}},
                {"n", {"10000"}}};
    sc.smoke = {{"c", {"0", "2"}},
                {"where", {"background", "lane"}},
                {"rate", {"tree", "opt"}},
                {"watch", {"1", "0"}},
                {"n", {"300"}}};
    sc.bg_calls_key = "c";
    sc.spawns_threads = true;
    sc.run = [](point_ctx &pc) {
      const int64_t c = pc.i("c"), n = pc.i("n");
      const bool lanes = pc.s("where") == "lane";
      const se::value_t loop = se::parse(loop_src(n, "(set s 0)", "(set s (+ s i))", "s"));
      pc.reps([&](meter &m, rep_out &o) {
        auto app = make_app();
        auto log = std::make_shared<kernel_log>();
        log->reset(static_cast<std::size_t>(c));
        world w(pc.executor, app.get());
        w.make_host("background");
        register_kernels(w, log, "background");
        latch_release rel{log}; // after w: releases the kernels before ~world waits for them
        if (lanes) {
          // One lane per caller, so c calls start at once without the pool.
          for (int64_t j = 0; j < c; ++j) {
            w.add_lane("bl" + istr(j));
            se::async_options lo;
            lo.where = se::executor::on_lane("bl" + istr(j));
            lo.future_variant = false;
            se::register_async_fn(
                w.env, *w.host, "bench-block-l" + istr(j),
                [log](std::span<const se::value_t> a) -> se::async_kernel {
                  const int64_t i = int_arg(a, 0, 0);
                  return [log, i](se::async_call &) -> se::value_t {
                    log->started.fetch_add(1);
                    std::unique_lock<std::mutex> lk(log->mu);
                    log->latch_cv.wait(lk, [&] { return log->released; });
                    lk.unlock();
                    log->mark(i);
                    return se::value_t(i);
                  };
                },
                lo);
          }
        }
        if (pc.i("watch", 1) == 0)
          w.async->set_watch_root(nullptr);
        tracker callers(static_cast<std::size_t>(c));
        step_kinds kinds;
        for (int64_t j = 0; j < c; ++j) {
          se::execute_options opts;
          opts.env = w.process_env("extend");
          opts.on_complete = callers.hook(std::size_t(j));
          if (pc.s("rate") == "opt")
            opts.async_steps_per_cpu_ms = se::kDefaultAsyncStepsPerCpuMs;
          callers.pid[std::size_t(j)] =
              w.submit("(" + std::string(lanes ? "bench-block-l" + istr(j) : "bench-block") + " " +
                           istr(j) + ")",
                       opts);
        }
        std::string why = drive(w, driver_of(pc), [&] { return log->started.load() >= c; }, [] {});
        if (!why.empty()) {
          o.fail("starting the blocked calls: " + why);
          return;
        }
        o.iterations = n;
        run_single(w, pc, m, o, loop, istr(n * (n - 1) / 2));
        o.extra.i("in_flight", int64_t(w.async->async_calls_in_flight()));
        log->release();
        why = drive(
            w, driver_of(pc), [&] { return callers.all_done(); },
            [&] { sweep(w, callers, true, false, kinds); });
        if (!why.empty())
          o.fail("the blocked callers: " + why + ": " + first_failure(w, callers));
        for (std::size_t j = 0; j < callers.pid.size(); ++j)
          if (callers.finish_ns[j] && se::to_string(callers.result[j]) != istr(int64_t(j)))
            o.fail("caller " + istr(int64_t(j)) + " returned " + se::to_string(callers.result[j]));
      });
    };
    all.push_back(std::move(sc));
  }

  // G. async.compute_fanout*: kernels that parallel_for over app.computePool().
  struct fanout_kind {
    const char *name, *what;
    sizes_t sizes, smoke;
  };
  for (const fanout_kind &fk : std::initializer_list<fanout_kind>{
           {"async.compute_fanout",
            "m processes, each running r async kernels that parallel_for `chunks` chunks of `us` "
            "us over app.computePool() (wide fan-outs: each saturates the pool)",
            {{"m", {"1", "4", "16"}},
             {"r", {"10"}},
             {"chunks", {"64"}},
             {"us", {"50"}},
             {"maxpar", {"0"}},
             {"driver", {"spin"}}},
            {{"m", {"2"}},
             {"r", {"2"}},
             {"chunks", {"16"}},
             {"us", {"20"}},
             {"maxpar", {"0"}},
             {"driver", {"spin"}}}},
           {"async.compute_fanout_narrow",
            "m processes, each running r fan-outs of 4 chunks of 1 ms, at most 4 wide: concurrent "
            "fan-outs would take ~r ms in all, serialized ones ~m*r ms (fanout_stretch ~m)",
            {{"m", {"1", "4"}},
             {"r", {"5"}},
             {"chunks", {"4"}},
             {"us", {"1000"}},
             {"maxpar", {"4"}},
             {"driver", {"spin"}}},
            {{"m", {"1", "2"}},
             {"r", {"2"}},
             {"chunks", {"4"}},
             {"us", {"200"}},
             {"maxpar", {"4"}},
             {"driver", {"spin"}}}},
           {"async.compute_fanout_herd",
            "r tiny fan-outs (22 chunks of 1 us, at most 2 wide): ns per parallel_for, mostly "
            "waking the pool's workers",
            {{"m", {"1"}},
             {"r", {"200"}},
             {"chunks", {"22"}},
             {"us", {"1"}},
             {"maxpar", {"2"}},
             {"driver", {"spin"}}},
            {{"m", {"1"}},
             {"r", {"10"}},
             {"chunks", {"22"}},
             {"us", {"1"}},
             {"maxpar", {"2"}},
             {"driver", {"spin"}}}}}) {
    scenario sc;
    sc.name = fk.name;
    sc.what = fk.what;
    sc.executors = {"async"};
    sc.bg_calls_key = "m";
    sc.spawns_threads = true;
    sc.sizes = fk.sizes;
    sc.smoke = fk.smoke;
    sc.run = [](point_ctx &pc) {
      const int64_t mm = pc.i("m"), r = pc.i("r"), chunks = pc.i("chunks"), us = pc.i("us");
      const int64_t maxpar = pc.i("maxpar");
      const std::string src = loop_src(r, "(set c 0)",
                                       "(set c (+ c (bench-fanout " + istr(chunks) + " " +
                                           istr(us) + " " + istr(maxpar) + ")))",
                                       "c");
      std::vector<std::string> srcs(mm, src), expect(mm, istr(r * chunks));
      pc.reps([&](meter &m, rep_out &o) {
        auto app = make_app(true);
        const unsigned conc = app->computePool().concurrency();
        auto log = std::make_shared<kernel_log>();
        world w(pc.executor, app.get());
        w.make_host("background");
        register_kernels(w, log, "background");
        m.app = app.get();
        auto f = run_flood(w, pc, m, o, srcs, expect, "extend", true);
        o.latency_ns = std::move(f.latency);
        const double work = double(mm * r * chunks * us) * 1000.0;
        const double wall = double(m.wall());
        o.async_calls = mm * r;
        // The time one fan-out would take alone: ceil(chunks / width) rounds of `us`.
        const int64_t width = std::max<int64_t>(
            1, std::min<int64_t>(maxpar > 0 ? maxpar : int64_t(conc), int64_t(conc)));
        const double ideal = double((chunks + width - 1) / width * us) * 1000.0;
        std::vector<int64_t> waits, fans;
        {
          std::lock_guard<std::mutex> lk(log->mu);
          waits = log->fanout_wait_ns;
          fans = log->fanout_ns;
        }
        o.extra.i("pool_concurrency", conc)
            .d("chunk_work_ns", work)
            .d("pool_efficiency", wall > 0 ? work / (wall * conc) : 0.0)
            .raw("fanout_wait_ns", percentiles(waits).str())
            .raw("fanout_ns", percentiles(fans).str());
        if (ideal > 0 && !fans.empty())
          o.extra.d("fanout_stretch", double(median_of(fans)) / ideal);
      });
    };
    all.push_back(std::move(sc));
  }

  // H. msg.*: messaging between processes.
  {
    scenario sc;
    sc.name = "msg.pingpong";
    sc.what = "two processes, r round trips of msg-send / msg-recv, beside `bystanders` "
              "processes parked on another channel (the delivery scan walks them)";
    sc.executors = both_sched;
    sc.sizes = {{"r", {"2000"}}, {"bystanders", {"0", "100", "1000"}}, {"driver", {"spin"}}};
    sc.smoke = {{"r", {"100"}}, {"bystanders", {"0", "10"}}, {"driver", {"spin"}}};
    sc.async_only = {"driver"};
    sc.run = [](point_ctx &pc) {
      const int64_t r = pc.i("r");
      const std::string a =
          loop_src(r, "", "(msg-send \"bench.ping\" \"p\") (msg-recv \"bench.pong\")", "i");
      const std::string b =
          loop_src(r, "", "(msg-recv \"bench.ping\") (msg-send \"bench.pong\" \"q\")", "i");
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        const std::string why =
            park_bystanders(w, pc.i("bystanders"), parked_on_never, pc.cfg.timeout);
        if (!why.empty()) {
          o.fail("parking: " + why);
          return;
        }
        run_flood(w, pc, m, o, {b, a}, {istr(r), istr(r)}, "extend", true);
        o.iterations = r;
        o.extra.d("round_trip_ns", double(m.wall()) / double(r));
      });
    };
    all.push_back(std::move(sc));
  }
  {
    scenario sc;
    sc.name = "msg.producer_consumer";
    sc.what = "one producer sending m messages to one consumer (unbounded pending queue)";
    sc.executors = both_sched;
    sc.sizes = {{"m", {"20000"}}, {"driver", {"spin"}}};
    sc.smoke = {{"m", {"200"}}, {"driver", {"spin"}}};
    sc.async_only = {"driver"};
    sc.run = [](point_ctx &pc) {
      const int64_t mm = pc.i("m");
      const std::string prod = loop_src(mm, "", "(msg-send \"bench.q\" \"x\")", "i");
      const std::string cons = "(begin (set c 0) (while (< c " + istr(mm) +
                               ") (begin (msg-recv \"bench.q\") (set c (+ c 1)))) c)";
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        if (w.sync)
          w.sync->max_pending_messages = 0;
        else
          w.async->max_pending_messages = 0;
        run_flood(w, pc, m, o, {cons, prod}, {istr(mm), istr(mm)}, "extend", true);
        o.iterations = mm;
        o.extra.d("messages_per_s", double(mm) * 1e9 / double(m.wall()));
      });
    };
    all.push_back(std::move(sc));
  }
  {
    scenario sc;
    sc.name = "msg.broadcast";
    sc.what = "one sender, k receivers parked on one channel, m rounds of send + k acks; the "
              "sender polls (bench-parked) until all k are parked before each send; the polls "
              "and their time are recorded so they can be told from the messaging";
    sc.executors = both_sched;
    sc.sizes = {{"k", {"10", "100"}}, {"m", {"200"}}, {"driver", {"spin"}}};
    sc.smoke = {{"k", {"4"}}, {"m", {"10"}}, {"driver", {"spin"}}};
    sc.async_only = {"driver"};
    sc.run = [](point_ctx &pc) {
      const int64_t k = pc.i("k"), mm = pc.i("m");
      const std::string recv = "(begin (set c 0) (while (< c " + istr(mm) +
                               ") (begin (msg-recv \"bench.bc\") (msg-send \"bench.ack\" \"a\") "
                               "(set c (+ c 1)))) c)";
      const std::string send =
          "(begin (set r 0) (while (< r " + istr(mm) + ") (begin (while (< (bench-parked) " +
          istr(k) + ") 0) (msg-send \"bench.bc\" \"x\") (set a 0) (while (< a " + istr(k) +
          ") (begin (msg-recv \"bench.ack\") (set a (+ a 1)))) (set r (+ r "
          "1)))) r)";
      std::vector<std::string> src(k, recv), expect(k, istr(mm));
      src.push_back(send);
      expect.push_back(istr(mm));
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        if (w.sync)
          w.sync->max_pending_messages = 0;
        else
          w.async->max_pending_messages = 0;
        // (bench-parked): how many receivers wait now. Their pids are the first k the scheduler
        // hands out (1..k): nothing else runs on this fresh scheduler. Counts its calls and the
        // time spent in them (natives run on the driver thread).
        world *wp = &w;
        int64_t polls = 0, poll_ns = 0;
        se::builtins::register_fn(
            w.env, "bench-parked",
            [wp, k, &polls, &poll_ns](std::span<const se::value_t>) -> se::value_t {
              const int64_t t0 = now_ns();
              int64_t parked = 0;
              for (int pid = 1; pid <= k; ++pid) {
                auto info = wp->info(pid);
                if (info && info->status == se::process_status::waiting)
                  ++parked;
              }
              ++polls;
              poll_ns += now_ns() - t0;
              return se::value_t(parked);
            });
        run_flood(w, pc, m, o, src, expect, "extend", true);
        o.iterations = k * mm;
        o.extra.d("ns_per_delivery", double(m.wall()) / double(k * mm))
            .i("parked_polls", polls)
            .i("parked_poll_ns", poll_ns)
            .d("ns_per_delivery_without_polls", double(m.wall() - poll_ns) / double(k * mm));
      });
    };
    all.push_back(std::move(sc));
  }
  {
    scenario sc;
    sc.name = "msg.fanout";
    sc.what = "one sender cycling m rounds over k receivers, each on its own channel (no gate: a "
              "send to a receiver that isn't parked queues): ns per delivery against k";
    sc.executors = both_sched;
    // Each send scans every process (k=1000: ~24 s a rep); sweep further with --size k=1000.
    sc.sizes = {{"k", {"10", "100", "300"}}, {"m", {"20"}}, {"driver", {"spin"}}};
    sc.smoke = {{"k", {"4"}}, {"m", {"5"}}, {"driver", {"spin"}}};
    sc.async_only = {"driver"};
    sc.run = [](point_ctx &pc) {
      const int64_t k = pc.i("k"), mm = pc.i("m");
      std::vector<std::string> src, expect;
      for (int64_t j = 0; j < k; ++j) {
        src.push_back("(begin (set c 0) (while (< c " + istr(mm) + ") (begin (msg-recv \"bench.r" +
                      istr(j) + "\") (set c (+ c 1)))) c)");
        expect.push_back(istr(mm));
      }
      src.push_back("(begin (set chans (list)) (set j 0) (while (< j " + istr(k) +
                    ") (begin (append chans (str-concat \"bench.r\" (str j))) (set j (+ j 1)))) "
                    "(set n 0) (set r 0) (while (< r " +
                    istr(mm) +
                    ") (begin (for ch chans (begin (msg-send ch \"x\") (set n (+ n 1)))) (set r "
                    "(+ r 1)))) n)");
      expect.push_back(istr(k * mm));
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        if (w.sync)
          w.sync->max_pending_messages = 0;
        else
          w.async->max_pending_messages = 0;
        run_flood(w, pc, m, o, src, expect, "extend", true);
        o.iterations = k * mm;
        o.extra.d("ns_per_delivery", double(m.wall()) / double(k * mm));
      });
    };
    all.push_back(std::move(sc));
  }
  {
    scenario sc;
    sc.name = "msg.ingress";
    sc.what = "t native threads calling async_scheduler::post_message (the thread-safe entry) "
              "into r parked receivers: messages/s and post->receive latency (the ingress "
              "mutex, the drain on every step)";
    sc.executors = {"async"};
    sc.sizes = {{"t", {"1", "4"}}, {"r", {"4"}}, {"per", {"4000"}}, {"driver", {"spin"}}};
    sc.smoke = {{"t", {"1", "2"}}, {"r", {"2"}}, {"per", {"100"}}, {"driver", {"spin"}}};
    sc.spawns_threads = true;
    sc.run = [](point_ctx &pc) {
      const int64_t nt = pc.i("t"), r = std::max<int64_t>(1, pc.i("r"));
      const int64_t per = pc.i("per") / r * r; // a multiple of r: every receiver gets the same
      const int64_t each = nt * per / r;
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        w.async->max_pending_messages = 0;
        std::vector<int64_t> lat;
        lat.reserve(std::size_t(nt * per));
        se::builtins::register_fn(w.env, "bench-lat",
                                  [&lat](std::span<const se::value_t> a) -> se::value_t {
                                    lat.push_back(now_ns() - int_arg(a, 0, 0));
                                    return se::value_t(int64_t(0));
                                  });
        std::vector<std::string> src, expect;
        for (int64_t j = 0; j < r; ++j) {
          src.push_back("(begin (set c 0) (while (< c " + istr(each) +
                        ") (begin (bench-lat (msg-recv \"bench.in" + istr(j) +
                        "\")) (set c (+ c 1)))) c)");
          expect.push_back(istr(each));
        }
        std::atomic<bool> go{false};
        std::vector<std::thread> posters;
        // Joins the posters however the rep ends (before ~world: declared after it).
        struct joiner {
          std::vector<std::thread> &v;
          std::atomic<bool> &go;
          ~joiner() {
            go.store(true);
            for (auto &th : v)
              if (th.joinable())
                th.join();
          }
        } join_posters{posters, go};
        {
          unpinned_scope u;
          for (int64_t j = 0; j < nt; ++j)
            posters.emplace_back([&, j] {
              (void)j;
              while (!go.load(std::memory_order_acquire))
                std::this_thread::yield();
              for (int64_t i = 0; i < per; ++i)
                w.async->post_message("bench.in" + istr(i % r), se::value_t(now_ns()));
            });
        }
        tracker t(src.size());
        step_kinds kinds;
        const uint64_t steps0 = w.total_steps();
        m.begin();
        for (std::size_t i = 0; i < src.size(); ++i) {
          se::execute_options opts;
          opts.env = w.process_env("extend");
          opts.on_complete = t.hook(i);
          t.submit_ns[i] = now_ns();
          t.pid[i] = w.submit(src[i], opts);
        }
        go.store(true, std::memory_order_release);
        drive_cfg d = driver_of(pc);
        d.sleepers = true; // receivers parked while the posters fill the ingress
        const std::string why =
            drive(w, d, [&] { return t.all_done(); }, [&] { sweep(w, t, true, false, kinds); });
        m.end();
        for (auto &th : posters)
          if (th.joinable())
            th.join();
        o.steps_total = w.total_steps() - steps0;
        if (!why.empty())
          o.fail(why + ": " + first_failure(w, t));
        for (std::size_t i = 0; i < src.size(); ++i)
          if (t.finish_ns[i] && se::to_string(t.result[i]) != expect[i])
            o.fail("receiver " + istr(int64_t(i)) + " returned " + se::to_string(t.result[i]));
        o.iterations = nt * per;
        o.extra.d("messages_per_s", double(nt * per) * 1e9 / double(m.wall()))
            .raw("post_to_receive_ns", percentiles(lat).str());
      });
    };
    all.push_back(std::move(sc));
  }

  // I. native.*: the cost of one call of a kind, over the same loop without it.
  struct native_kind {
    const char *name, *what;
    sizes_t sizes, smoke;
  };
  for (const native_kind &nk : std::initializer_list<native_kind>{
           {"plain", "a registered native, (bench-inc i)", {{"n", {"20000"}}}, {{"n", {"300"}}}},
           {"apply",
            "apply of a native, (apply bench-inc (list i)), the list included",
            {{"n", {"20000"}}},
            {{"n", {"300"}}}},
           {"state_get",
            "state-get of a tree node `depth` levels down",
            {{"n", {"20000"}}, {"depth", {"1", "4", "8"}}},
            {{"n", {"300"}}, {"depth", {"1", "4"}}}},
           {"state_set",
            "state-set of a tree node `depth` levels down (watch root set)",
            {{"n", {"20000"}}, {"depth", {"1", "4", "8"}}},
            {{"n", {"300"}}, {"depth", {"1", "4"}}}},
           {"state_delete",
            "state-set then state-delete of a node, with `nodes` other nodes under the root "
            "(state-delete sweeps the whole tree for expired nodes)",
            {{"n", {"2000"}}, {"nodes", {"100", "10000"}}},
            {{"n", {"100"}}, {"nodes", {"100"}}}},
           {"msg_send",
            "msg-send with no receiver (queued, then dropped past max_pending_messages)",
            {{"n", {"20000"}}},
            {{"n", {"300"}}}}}) {
    scenario sc;
    sc.name = std::string("native.") + nk.name;
    sc.what = std::string("per-call cost of ") + nk.what +
              ": the loop with the call against the loop without it, `rounds` of each "
              "alternated in every rep (the fastest of each kept)";
    sc.executors = both_sched;
    sc.sizes = nk.sizes;
    sc.smoke = nk.smoke;
    sc.sizes.push_back({"rounds", {"3"}});
    sc.smoke.push_back({"rounds", {"1"}});
    sc.sizes.push_back({"driver", {"spin"}});
    sc.smoke.push_back({"driver", {"spin"}});
    sc.async_only = {"driver"};
    const std::string kind = nk.name;
    sc.run = [kind](point_ctx &pc) {
      const int64_t n = pc.i("n"), rounds = std::max<int64_t>(1, pc.i("rounds"));
      std::string path = "bench";
      for (int64_t d = 1; d < pc.i("depth", 1); ++d)
        path += ".d" + istr(d);
      std::string call;
      if (kind == "plain")
        call = "(bench-inc i)";
      else if (kind == "apply")
        call = "(apply bench-inc (list i))";
      else if (kind == "state_get")
        call = "(state-get \"" + path + "\")";
      else if (kind == "state_set")
        call = "(state-set \"" + path + "\" i)";
      else if (kind == "state_delete")
        call = "(state-set \"bench.k\" i) (state-delete \"bench.k\")";
      else
        call = "(msg-send \"bench.sink\" \"x\")";
      const se::value_t with = se::parse(loop_src(n, "", call, "i"));
      const se::value_t without = se::parse(loop_src(n, "", "", "i"));
      auto app = make_app();
      cvc::state &root = cvc::state::instance(*app);
      root(path).value("7");
      for (int64_t j = 0; j < pc.i("nodes", 0); ++j)
        root("pop" + istr(j)).value("1");
      pc.reps([&](meter &m, rep_out &o) {
        int64_t best_base = INT64_MAX, best_with = INT64_MAX;
        uint64_t base_steps = 0, with_steps = 0;
        for (int64_t r = 0; r < rounds && o.ok; ++r) {
          {
            world w(pc.executor, app.get());
            meter bm;
            rep_out bo;
            run_single(w, pc, bm, bo, without, istr(n));
            if (!bo.ok)
              o.fail("baseline loop: " + bo.error);
            if (bm.wall() < best_base) {
              best_base = bm.wall();
              base_steps = bo.steps_total.value_or(0);
            }
          }
          world w(pc.executor, app.get());
          meter wm;
          wm.want_heap = m.want_heap;
          rep_out wo;
          run_single(w, pc, wm, wo, with, istr(n));
          if (!wo.ok)
            o.fail(wo.error);
          if (wm.wall() < best_with) {
            best_with = wm.wall();
            with_steps = wo.steps_total.value_or(0);
            m = wm; // the record is the fastest run with the call
            o.steps_total = wo.steps_total;
            o.kinds = wo.kinds;
          }
        }
        o.iterations = n;
        if (!o.ok || !base_steps)
          return;
        const double ns_per_call = double(best_with - best_base) / double(n);
        const double steps_per_call = double(with_steps - base_steps) / double(n);
        const double base_ns_per_step = double(best_base) / double(base_steps);
        o.extra.i("baseline_wall_ns", best_base)
            .i("baseline_steps", int64_t(base_steps))
            .d("steps_per_call", steps_per_call)
            .d("ns_per_call", ns_per_call)
            .d("native_ns_per_call", ns_per_call - steps_per_call * base_ns_per_step)
            .i("rounds", rounds);
      });
    };
    all.push_back(std::move(sc));
  }

  // J. footprint.*: memory per idle process, per closure, per environment.
  {
    scenario sc;
    sc.name = "footprint.idle_process";
    sc.what = "n processes parked on msg-recv: memory per idle process (heap, working set and "
              "private deltas / n)";
    sc.executors = both_sched;
    // Parking n processes costs the sync scheduler O(n^2) steps' worth of scans (n=10000: ~20 s
    // a rep), and the per-process figure is flat by n=1000.
    sc.sizes = {{"n", {"1000", "3000"}}};
    sc.smoke = {{"n", {"100"}}};
    sc.want_heap = true;
    sc.run = [](point_ctx &pc) {
      const int64_t n = pc.i("n");
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        m.begin();
        for (int64_t i = 0; i < n; ++i) {
          se::execute_options opts;
          opts.env = w.process_env("extend");
          // A top-level call can't park (nothing receives its value), so it is wrapped.
          w.submit(parked_on_never(i), opts);
        }
        const std::string why = wait_parked(w, n, pc.cfg.timeout);
        m.end();
        if (!why.empty()) {
          std::string detail = why;
          for (const auto &info : w.base().list_processes())
            if (se::is_finished(info.status)) {
              detail += "; pid " + istr(info.pid) + " " + info.exit_error.value_or("");
              break;
            }
          o.fail(detail);
        }
        o.programs = n;
        o.steps_total = w.total_steps();
        if (m.a.mem.has_heap)
          o.extra.d("heap_bytes_per_process",
                    double(int64_t(m.b.mem.heap) - int64_t(m.a.mem.heap)) / double(n));
        o.extra.d("working_set_bytes_per_process",
                  (double(m.b.mem.ws) - double(m.a.mem.ws)) / double(n));
        if (m.b.mem.has_priv)
          o.extra.d("private_bytes_per_process",
                    (double(m.b.mem.priv) - double(m.a.mem.priv)) / double(n));
      });
    };
    all.push_back(std::move(sc));
  }
  struct footprint_kind {
    const char *name, *what, *setup, *item;
  };
  for (const footprint_kind &fk : std::initializer_list<footprint_kind>{
           {"closure", "n closures sharing one environment, kept in a list", "", "(lambda (x) x)"},
           {"closure_forms",
            "n closures of a two-form body sharing one environment, kept in a list", "",
            "(lambda (x) (+ x 1) x)"},
           {"environment",
            "n closures each over an environment of its own (a call's), kept in a list",
            "(set mk (lambda (x) (lambda () x))) ", "(mk i)"}}) {
    scenario sc;
    sc.name = std::string("footprint.") + fk.name;
    sc.what = std::string("memory per item: ") + fk.what;
    sc.executors = {"stackless"};
    sc.sizes = {{"n", {"100000"}}};
    sc.smoke = {{"n", {"1000"}}};
    sc.want_heap = true;
    const std::string setup = fk.setup, item = fk.item;
    sc.run = [setup, item](point_ctx &pc) {
      const int64_t n = pc.i("n");
      auto env = se::builtins::make_default_environment();
      const se::value_t ast =
          se::parse(loop_src(n, setup + "(set l (list))", "(append l " + item + ")", "(length l)"));
      pc.reps([&](meter &m, rep_out &o) {
        se::stackless_evaluator ev(env);
        m.begin();
        se::evaluator_state st = ev.create_state(ast);
        while (!ev.step(st)) {
        }
        m.end(); // the list is still alive in st's environment
        o.steps_total = st.stats.get_step_count();
        o.iterations = n;
        check_result(o, st.result, istr(n));
        if (m.a.mem.has_heap)
          o.extra.d("heap_bytes_per_item",
                    double(int64_t(m.b.mem.heap) - int64_t(m.a.mem.heap)) / double(n));
        if (m.b.mem.has_priv)
          o.extra.d("private_bytes_per_item",
                    (double(m.b.mem.priv) - double(m.a.mem.priv)) / double(n));
      });
    };
    all.push_back(std::move(sc));
  }

  // K. limits.*: the arithmetic loop with each resource guard armed (and never reached).
  struct limit_kind {
    const char *name, *what;
    std::vector<std::string> executors;
    sizes_t sizes, smoke;
  };
  for (const limit_kind &lk : std::initializer_list<limit_kind>{
           {"none", "no limits", {"sync", "async"}, {{"n", {"40000"}}}, {{"n", {"300"}}}},
           {"max_time",
            "max_time = 1e6 s (a deadline guard and clock reads every step; never past half the "
            "budget)",
            {"sync", "async"},
            {{"n", {"40000"}}},
            {{"n", {"300"}}}},
           {"max_time_ariadne",
            "max_time = 0.1 s, Ariadne's action default, on a loop sized to run ~70 ms, so its "
            "second half runs past the budget's midpoint (two thread-CPU reads more per step); "
            "it must still finish",
            {"sync", "async"},
            {{"n", {"8000"}}},
            // A Debug build runs the loop ~50 times slower: 300 iterations take ~150 ms on the
            // async scheduler there, past the budget, and the process is killed. The smoke run
            // only checks that the scenario works.
            {{"n", {"30"}}}},
           {"max_memory",
            "max_memory = 1 GiB (alloc guard, heap accounting)",
            {"sync", "async"},
            {{"n", {"40000"}}},
            {{"n", {"300"}}}},
           {"max_steps",
            "max_steps = 2^62",
            {"sync", "async"},
            {{"n", {"40000"}}},
            {{"n", {"300"}}}},
           // Only the async scheduler applies max_step_time (async_scheduler.cpp); the sync one
           // ignores it, so a sync point would measure limits.none again.
           {"max_step_time",
            "max_step_time = 1e6 s (a per-step deadline)",
            {"async"},
            {{"n", {"40000"}}},
            {{"n", {"300"}}}}}) {
    scenario sc;
    sc.name = std::string("limits.") + lk.name;
    sc.what = std::string("the arith_loop process with ") + lk.what;
    sc.executors = lk.executors;
    sc.sizes = lk.sizes;
    sc.smoke = lk.smoke;
    sc.sizes.push_back({"driver", {"spin"}});
    sc.smoke.push_back({"driver", {"spin"}});
    sc.async_only = {"driver"};
    const std::string limit = lk.name;
    sc.run = [limit](point_ctx &pc) {
      const int64_t n = pc.i("n");
      const se::value_t ast = se::parse(loop_src(n, "(set s 0)", "(set s (+ s i))", "s"));
      const std::string expect = istr(n * (n - 1) / 2);
      auto app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        o.iterations = n;
        run_single(w, pc, m, o, ast, expect, [&](se::execute_options &opts) {
          if (limit == "max_time")
            opts.max_time = 1e6;
          else if (limit == "max_time_ariadne")
            opts.max_time = 0.1;
          else if (limit == "max_memory")
            opts.max_memory = uint64_t(1) << 30;
          else if (limit == "max_steps")
            opts.max_steps = uint64_t(1) << 62;
          else if (limit == "max_step_time")
            opts.max_step_time = 1e6;
        });
      });
    };
    all.push_back(std::move(sc));
  }
  {
    scenario sc;
    sc.name = "limits.max_memory_bigheap";
    sc.what = "the arith_loop process holding a list of b items, with mem=none, mem=1g "
              "(max_memory 1 GiB) or mem=near (max_memory = what it measures to x 2, so the "
              "heap is re-measured near the budget): the heap measurement's price on a real heap";
    sc.executors = both_sched;
    sc.sizes = {{"b", {"100000"}}, {"n", {"40000"}}, {"mem", {"none", "1g", "near"}}};
    sc.smoke = {{"b", {"1000"}}, {"n", {"300"}}, {"mem", {"none", "1g", "near"}}};
    sc.run = [](point_ctx &pc) {
      const int64_t b = pc.i("b"), n = pc.i("n");
      const se::value_t ast = se::parse(loop_src(
          n, "(set big (collect (range " + istr(b) + "))) (set s 0)", "(set s (+ s i))", "s"));
      const std::string expect = istr(n * (n - 1) / 2);
      const std::string mem = pc.s("mem");
      auto app = make_app();
      uint64_t near_budget = 0;
      if (mem == "near") {
        // What the process's heap measures to, unlimited, from one run outside the reps.
        world w(pc.executor, app.get());
        meter cm;
        rep_out co;
        const int pid = run_single(w, pc, cm, co, ast, expect, [](se::execute_options &o) {
          o.max_memory = uint64_t(1) << 40;
        });
        if (auto info = w.info(pid))
          near_budget = static_cast<uint64_t>(
              double(std::max({info->heap_memory, info->peak_memory, info->current_memory})) * 2);
      }
      pc.reps([&](meter &m, rep_out &o) {
        world w(pc.executor, app.get());
        o.iterations = n;
        if (mem == "near" && !near_budget) {
          o.fail("couldn't measure the process's heap for mem=near");
          return;
        }
        const int pid = run_single(w, pc, m, o, ast, expect, [&](se::execute_options &opts) {
          if (mem == "1g")
            opts.max_memory = uint64_t(1) << 30;
          else if (mem == "near")
            opts.max_memory = near_budget;
        });
        if (auto info = w.info(pid))
          o.extra.i("heap_measurements", int64_t(info->heap_measurements))
              .i("heap_memory", int64_t(info->heap_memory))
              .i("max_memory", int64_t(info->max_memory));
      });
    };
    all.push_back(std::move(sc));
  }

  // data.*: deep data: copies, equality, printing, releasing.
  struct data_kind {
    const char *name, *what;
    std::string (*src)(int64_t n);
    const char *big; // the larger n
  };
  for (const data_kind &dk : std::initializer_list<data_kind>{
           {"copy_equal", "(collect (range n)), a slice copy of it, then (= l m)",
            [](int64_t n) {
              return "(begin (set l (collect (range " + istr(n) + "))) (set m (slice l 0 " +
                     istr(n) + ")) (if (= l m) " + istr(n) + " -1))";
            },
            "100000"},
           {"print_floats",
            "(str ...) of a list of n floats (one formatted double each), its length returned",
            [](int64_t n) {
              return "(length (str (collections.map (collect (range " + istr(n) +
                     ")) (lambda (x) (* x 1.5)))))";
            },
            // str caps its result at 64 KiB, which 10000 floats would pass.
            "5000"},
           {"release", "build an n-element nested list, drop it, finish",
            [](int64_t n) {
              return loop_src(n, "(set l (list))", "(append l (list i (list i i)))",
                              "(begin (set l nil) " + istr(n) + ")");
            },
            "100000"}}) {
    scenario sc;
    sc.name = std::string("data.") + dk.name;
    sc.what = std::string("deep data: ") + dk.what +
              "; on the schedulers with deadline=1 a max_time is armed (clock reads per node in "
              "the walks)";
    sc.executors = {"stackless", "sync", "async"};
    sc.sizes = {{"n", {"1000", dk.big}}, {"deadline", {"0", "1"}}, {"driver", {"spin"}}};
    sc.smoke = {{"n", {"1000"}}, {"deadline", {"0", "1"}}, {"driver", {"spin"}}};
    sc.varies_for = {{"deadline", {"sync", "async"}}};
    sc.async_only = {"driver"};
    sc.want_heap = true;
    const auto srcf = dk.src;
    const bool printed = std::string(dk.name) == "print_floats";
    sc.run = [srcf, printed](point_ctx &pc) {
      const int64_t n = pc.i("n");
      const se::value_t ast = se::parse(srcf(n));
      auto env = se::builtins::make_default_environment();
      add_bench_natives(env);
      std::string expect = istr(n);
      if (printed) {
        // The printed length depends on the float format: the stackless evaluator's own
        // answer is the reference every executor must match.
        se::stackless_evaluator ev(env);
        se::evaluator_state st = ev.create_state(ast);
        while (!ev.step(st)) {
        }
        expect = se::to_string(st.result);
      }
      std::unique_ptr<cvc::app> app;
      if (pc.executor != "stackless")
        app = make_app();
      pc.reps([&](meter &m, rep_out &o) {
        o.iterations = n;
        if (pc.executor == "stackless") {
          run_stackless(m, o, env, ast, expect);
        } else {
          world w(pc.executor, app.get());
          run_single(w, pc, m, o, ast, expect, [&](se::execute_options &opts) {
            if (pc.i("deadline", 0))
              opts.max_time = 1e6;
          });
        }
        if (printed && o.ok && std::strtoll(expect.c_str(), nullptr, 10) <= n)
          o.fail("printed only " + expect + " characters for " + istr(n) + " floats");
        o.extra.d("ns_per_node", double(m.wall()) / double(n));
      });
    };
    all.push_back(std::move(sc));
  }

  // mt.independent: DSL on several threads at once, nothing shared but the runtime.
  {
    scenario sc;
    sc.name = "mt.independent";
    sc.what = "t threads, each with its own stackless evaluator and default environment, "
              "running a program `runs` times; ast=private parses per thread, ast=shared runs "
              "one parsed AST on every thread: aggregate throughput, speedup over one thread, "
              "and the CPUs' efficiency classes (refcounts, global tables and the allocator "
              "under contention: the before numbers for threaded execution)";
    sc.executors = {"stackless"};
    sc.sizes = {{"t", {"1", "2", "4", "8"}},
                {"prog", {"fib", "exceptions"}},
                {"ast", {"private", "shared"}},
                {"runs", {"5"}}};
    sc.smoke = {{"t", {"1", "2"}},
                {"prog", {"fib", "exceptions"}},
                {"ast", {"private", "shared"}},
                {"runs", {"1"}}};
    sc.spawns_threads = true;
    sc.run = [](point_ctx &pc) {
      const int64_t nt = std::max<int64_t>(1, pc.i("t")), runs = std::max<int64_t>(1, pc.i("runs"));
      const bool is_fib = pc.s("prog") == "fib";
      const int64_t size = is_fib ? (pc.cfg.smoke ? 10 : 18) : (pc.cfg.smoke ? 50 : 2000);
      const std::string src =
          is_fib ? "(begin (defun fib (n) (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) (fib " +
                       istr(size) + "))"
                 : loop_src(size, "(set c 0)",
                            "(set c (+ c (try (raise (ValueError \"boom\")) (catch (ValueError e) "
                            "1))))",
                            "c");
      const std::string expect = is_fib ? istr(fib(size)) : istr(size);
      const int64_t iters_per_run = is_fib ? 2 * fib(size + 1) - 1 : size;
      const bool shared = pc.s("ast") == "shared";
      const se::value_t shared_ast = se::parse(src);

      struct worker {
        std::thread th;
        int64_t start = 0, end = 0;
        uint64_t allocs = 0; // the thread's own, over its runs (bench_alloc)
        int eff = -1;
        std::string error;
      };
      // One thread's work: `runs` runs, timed from the barrier. Returns "" or what went wrong.
      const auto work = [&](std::atomic<bool> &go, std::atomic<int64_t> &ready, worker &me) {
        try {
          auto env = se::builtins::make_default_environment();
          const se::value_t ast = shared ? shared_ast : se::parse(src);
          se::stackless_evaluator ev(env);
          ready.fetch_add(1);
          while (!go.load(std::memory_order_acquire))
            std::this_thread::yield();
          const uint64_t a0 = bench_alloc::count;
          me.start = now_ns();
          for (int64_t r = 0; r < runs; ++r) {
            se::evaluator_state st = ev.create_state(ast);
            while (!ev.step(st)) {
            }
            if (se::to_string(st.result) != expect && me.error.empty())
              me.error = "returned " + se::to_string(st.result) + ", expected " + expect;
          }
          me.end = now_ns();
          me.allocs = bench_alloc::count - a0;
          me.eff = current_cpu_efficiency();
        } catch (const std::exception &e) {
          me.error = std::string("exception: ") + e.what();
          ready.fetch_add(1);
        }
      };
      // Run `count` workers from a barrier; returns the wall time from the barrier to the last
      // join, and fills `ws`.
      const auto run_workers = [&](int64_t count, std::vector<worker> &ws, meter *m) {
        ws.clear();
        ws.resize(std::size_t(count));
        std::atomic<bool> go{false};
        std::atomic<int64_t> ready{0};
        {
          unpinned_scope u;
          for (auto &w : ws)
            w.th = std::thread([&, wp = &w] { work(go, ready, *wp); });
        }
        while (ready.load() < count)
          std::this_thread::yield();
        if (m)
          m->begin();
        const int64_t t0 = now_ns();
        go.store(true, std::memory_order_release);
        for (auto &w : ws)
          w.th.join();
        const int64_t wall = now_ns() - t0;
        if (m)
          m->end();
        return wall;
      };
      pc.reps([&](meter &m, rep_out &o) {
        std::vector<worker> ws;
        const int64_t one = run_workers(1, ws, nullptr); // the single-thread reference
        if (!ws[0].error.empty())
          o.fail("one thread: " + ws[0].error);
        run_workers(nt, ws, &m);
        std::vector<int64_t> per;
        uint64_t allocs = 0;
        int on_low = 0, top = -1;
        for (const auto &w : ws)
          top = std::max(top, w.eff);
        for (const auto &w : ws) {
          if (!w.error.empty())
            o.fail(w.error);
          per.push_back(w.end - w.start);
          allocs += w.allocs;
          if (w.eff >= 0 && w.eff < top)
            ++on_low;
        }
        o.iterations = nt * runs * iters_per_run;
        o.programs = nt * runs;
        const double wall = double(m.wall());
        o.extra.d("speedup", wall > 0 ? double(nt) * double(one) / wall : 0.0)
            .d("thread_ns_per_iter", double(median_of(per)) / double(runs * iters_per_run))
            .d("single_thread_ns_per_iter", double(one) / double(runs * iters_per_run))
            .i("threads_on_lower_efficiency_class", on_low);
        if (bench_alloc::active)
          o.extra.d("allocs_per_iter_all_threads", double(allocs) / double(*o.iterations));
      });
    };
    all.push_back(std::move(sc));
  }

  // L. parse.*
  {
    scenario sc;
    sc.name = "parse.small";
    sc.what = "parse() of a small program (a fib defun), r times";
    sc.executors = {""};
    sc.sizes = {{"r", {"20000"}}};
    sc.smoke = {{"r", {"200"}}};
    sc.run = [](point_ctx &pc) {
      const int64_t r = pc.i("r");
      const std::string src =
          "(begin (defun fib (n) (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) (fib 20))";
      pc.reps([&](meter &m, rep_out &o) {
        std::size_t forms = 0;
        m.begin();
        for (int64_t i = 0; i < r; ++i) {
          const se::value_t v = se::parse(src);
          forms += std::holds_alternative<se::list_ptr>(v.v) ? 1 : 0;
        }
        m.end();
        if (forms != std::size_t(r))
          o.fail("parse did not return a list");
        o.iterations = r;
        const double wall = double(m.wall());
        o.extra.i("bytes", int64_t(src.size())).d("ns_per_byte", wall / double(r * src.size()));
      });
    };
    all.push_back(std::move(sc));
  }
  {
    scenario sc;
    sc.name = "parse.large";
    sc.what = "parse() of a ~kb KiB program (defuns with strings, numbers, nesting), r times";
    sc.executors = {""};
    sc.sizes = {{"kb", {"100"}}, {"r", {"20"}}};
    sc.smoke = {{"kb", {"10"}}, {"r", {"2"}}};
    sc.run = [](point_ctx &pc) {
      const int64_t kb = pc.i("kb"), r = pc.i("r");
      std::string src = "(begin\n";
      int64_t defs = 0;
      while (int64_t(src.size()) < kb * 1024) {
        src += " (defun f" + istr(defs) +
               " (a b) (begin (set x (+ a b 1.25)) (if (< x 10) (str-concat \"s\" (str x)) (list "
               "a b x \"a string literal\" (dict \"k\" 3)))))\n";
        ++defs;
      }
      src += " (f0 1 2))\n";
      pc.reps([&](meter &m, rep_out &o) {
        std::size_t len = 0;
        m.begin();
        for (int64_t i = 0; i < r; ++i) {
          const se::value_t v = se::parse(src);
          if (const auto *l = std::get_if<se::list_ptr>(&v.v))
            len = (*l)->size();
        }
        m.end();
        if (len != std::size_t(defs + 2))
          o.fail("parsed " + istr(int64_t(len)) + " forms, expected " + istr(defs + 2));
        o.iterations = r;
        const double wall = double(m.wall());
        o.extra.i("bytes", int64_t(src.size()))
            .d("ns_per_byte", wall / double(r * int64_t(src.size())))
            .d("mb_per_s", double(r * int64_t(src.size())) / (wall / 1e9) / 1e6);
      });
    };
    all.push_back(std::move(sc));
  }
  return all;
}

// ---------------------------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------------------------

bool glob_match(const char *p, const char *s) {
  for (; *p; ++p, ++s) {
    if (*p == '*') {
      for (const char *t = s;; ++t) {
        if (glob_match(p + 1, t))
          return true;
        if (!*t)
          return false;
      }
    }
    if (!*s || (*p != '?' && *p != *s))
      return false;
  }
  return !*s;
}

std::vector<std::string> split(const std::string &s, char c) {
  std::vector<std::string> out;
  std::string cur;
  std::istringstream in(s);
  while (std::getline(in, cur, c))
    if (!cur.empty())
      out.push_back(cur);
  return out;
}

const sizes_t &sizes_for(const scenario &sc, bool smoke) { return smoke ? sc.smoke : sc.sizes; }

std::string sizes_text(const sizes_t &s) {
  std::string t;
  for (const auto &[k, v] : s) {
    std::string vals;
    for (const auto &x : v)
      vals += (vals.empty() ? "" : ",") + x;
    t += (t.empty() ? "" : " ") + k + "=" + vals;
  }
  return t;
}

void usage() {
  std::fprintf(
      stderr,
      "usage: state_exec_bench [--list | --list-json] [--scenario NAME|GLOB]... [--executor E]...\n"
      "         [--reps N] [--warmup N] [--json FILE] [--smoke] [--size KEY=V[,V...]]...\n"
      "         [--pin auto|none|CPU] [--priority high] [--timeout SECONDS]\n"
      "         [--bg-oversubscribe]\n"
      "         [--sample PREFIX [--sample-us N] [--sample-depth N] [--sample-all-reps]]\n"
      "  executors: recursive stackless sync async\n");
}

// A whole non-negative integer argument, or exit 2.
int64_t int_option(const std::string &opt, const std::string &v, int64_t lo) {
  char *end = nullptr;
  errno = 0;
  const long long x = std::strtoll(v.c_str(), &end, 10);
  if (v.empty() || !end || *end || errno || x < lo) {
    std::fprintf(stderr, "state_exec_bench: %s expects an integer >= %lld, got '%s'\n", opt.c_str(),
                 static_cast<long long>(lo), v.c_str());
    std::exit(2);
  }
  return x;
}

double seconds_option(const std::string &opt, const std::string &v) {
  char *end = nullptr;
  const double x = std::strtod(v.c_str(), &end);
  if (v.empty() || !end || *end || !(x > 0)) {
    std::fprintf(stderr, "state_exec_bench: %s expects a positive number, got '%s'\n", opt.c_str(),
                 v.c_str());
    std::exit(2);
  }
  return x;
}

// A --size value as the records will carry it: an integer in canonical form ("010" -> "10"), so
// the JSON stays valid and the same size always reads the same.
std::string canonical_size(const std::string &v) {
  bool digits = !v.empty();
  for (std::size_t i = (v[0] == '-' ? 1 : 0); i < v.size(); ++i)
    digits = digits && v[i] >= '0' && v[i] <= '9';
  if (!digits || v == "-")
    return v;
  return std::to_string(std::strtoll(v.c_str(), nullptr, 10));
}

} // namespace

int main(int argc, char **argv) {
  bench_alloc::install();
  run_config cfg;
  std::vector<std::string> patterns;
  bool list = false, list_json = false;
  std::string pin = "none", priority;
  sampler_opts sampling;
  bool sample_tuned = false; // --sample-us/-depth/-all-reps given
  const std::set<std::string> known_executors = {"recursive", "stackless", "sync", "async"};
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto value = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "state_exec_bench: %s needs a value\n", a.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--list")
      list = true;
    else if (a == "--list-json")
      list_json = true;
    else if (a == "--scenario")
      patterns.push_back(value());
    else if (a == "--executor") {
      const std::string e = value();
      if (!known_executors.count(e)) {
        std::fprintf(stderr, "state_exec_bench: unknown executor '%s'\n", e.c_str());
        usage();
        return 2;
      }
      cfg.executors.push_back(e);
    } else if (a == "--reps") {
      cfg.reps = static_cast<int>(int_option(a, value(), 1));
      cfg.reps_set = true;
    } else if (a == "--warmup") {
      cfg.warmup = static_cast<int>(int_option(a, value(), 0));
      cfg.warmup_set = true;
    } else if (a == "--json")
      cfg.json_path = value();
    else if (a == "--smoke")
      cfg.smoke = true;
    else if (a == "--bg-oversubscribe")
      cfg.bg_oversubscribe = true;
    else if (a == "--timeout")
      cfg.timeout = seconds_option(a, value());
    else if (a == "--size") {
      const std::string kv = value();
      const auto eq = kv.find('=');
      if (eq == std::string::npos || eq == 0) {
        std::fprintf(stderr, "state_exec_bench: --size expects KEY=VALUE[,VALUE...]\n");
        return 2;
      }
      std::vector<std::string> vals = split(kv.substr(eq + 1), ',');
      if (vals.empty()) {
        std::fprintf(stderr, "state_exec_bench: --size %s has no values\n", kv.c_str());
        return 2;
      }
      for (auto &v : vals)
        v = canonical_size(v);
      cfg.size_over[kv.substr(0, eq)] = std::move(vals);
    } else if (a == "--pin")
      pin = value();
    else if (a == "--priority")
      priority = value();
    else if (a == "--sample") {
      sampling.prefix = value();
      if (sampling.prefix.empty()) {
        std::fprintf(stderr, "state_exec_bench: --sample needs a non-empty path prefix\n");
        return 2;
      }
    } else if (a == "--sample-us") {
      sampling.interval_us = int_option(a, value(), 100);
      sample_tuned = true;
    } else if (a == "--sample-depth") {
      sampling.depth = static_cast<int>(std::min<int64_t>(int_option(a, value(), 1), 1024));
      sample_tuned = true;
    } else if (a == "--sample-all-reps") {
      sampling.all_reps = true;
      sample_tuned = true;
    } else if (a == "--help" || a == "-h") {
      usage();
      return 0;
    } else {
      std::fprintf(stderr, "state_exec_bench: unknown argument '%s'\n", a.c_str());
      usage();
      return 2;
    }
  }
  if (cfg.smoke) {
    if (!cfg.reps_set)
      cfg.reps = 1;
    if (!cfg.warmup_set)
      cfg.warmup = 0;
  }

  const std::vector<scenario> all = make_scenarios();
  std::vector<const scenario *> chosen;
  for (const scenario &sc : all) {
    bool hit = patterns.empty();
    for (const auto &p : patterns)
      hit = hit || glob_match(p.c_str(), sc.name.c_str());
    if (hit)
      chosen.push_back(&sc);
  }
  if (list || list_json) {
    if (list_json) {
      std::printf("[\n");
      for (std::size_t i = 0; i < chosen.size(); ++i) {
        const scenario &sc = *chosen[i];
        jobj o;
        o.s("name", sc.name).s("what", sc.what);
        std::string ex;
        for (const auto &e : sc.executors)
          if (!e.empty())
            ex += (ex.empty() ? "" : ", ") + json_str(e);
        o.raw("executors", "[" + ex + "]");
        const auto sizes_json = [](const sizes_t &sz) {
          jobj s;
          for (const auto &[k, v] : sz) {
            std::string vals;
            for (const auto &x : v)
              vals += (vals.empty() ? "" : ", ") + (is_integer(x) ? x : json_str(x));
            s.raw(k, "[" + vals + "]");
          }
          return s.str();
        };
        o.raw("sizes", sizes_json(sc.sizes)).raw("smoke", sizes_json(sc.smoke));
        std::string ao;
        for (const auto &k : sc.async_only)
          ao += (ao.empty() ? "" : ", ") + json_str(k);
        o.raw("async_only", "[" + ao + "]");
        jobj vf;
        for (const auto &[k, execs] : sc.varies_for) {
          std::string l;
          for (const auto &e : execs)
            l += (l.empty() ? "" : ", ") + json_str(e);
          vf.raw(k, "[" + l + "]");
        }
        o.raw("varies_for", vf.str());
        if (!sc.bg_calls_key.empty())
          o.s("bg_calls_key", sc.bg_calls_key);
        o.b("spawns_threads", sc.spawns_threads);
        std::printf("  %s%s\n", o.str().c_str(), i + 1 < chosen.size() ? "," : "");
      }
      std::printf("]\n");
      return 0;
    }
    std::printf("%-30s %-22s %s\n", "scenario", "executors", "sizes (default | smoke)");
    for (const scenario *sc : chosen) {
      std::string ex;
      for (const auto &e : sc->executors)
        ex += (ex.empty() ? "" : ",") + (e.empty() ? std::string("-") : e);
      std::printf("%-30s %-22s %s | %s\n", sc->name.c_str(), ex.c_str(),
                  sizes_text(sc->sizes).c_str(), sizes_text(sc->smoke).c_str());
      std::printf("    %s\n", sc->what.c_str());
    }
    return 0;
  }
  if (chosen.empty()) {
    std::fprintf(stderr, "state_exec_bench: no scenario matches\n");
    return 2;
  }

  // Probes that allocate on first use (the thread count's buffer) run once here, so the first
  // measured region doesn't carry their memory.
  (void)thread_count();
  (void)handle_count();
  // The watchdog's thread starts before the driver is pinned (on Linux it would inherit the pin).
  the_watchdog().start();

  cfg.host.logical_cpus = std::thread::hardware_concurrency();
  {
    // The poll1ms driver sleeps for 1 ms when idle, as pycvc's pump does. On Windows that lasts a
    // timer tick (~15.6 ms) unless something raised the timer resolution, so the real figure goes
    // in every record.
    std::vector<int64_t> naps;
    for (int i = 0; i < 5; ++i) {
      const int64_t t0 = now_ns();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      naps.push_back(now_ns() - t0);
    }
    std::sort(naps.begin(), naps.end());
    cfg.host.sleep_1ms_ns = naps[naps.size() / 2];
  }
  // The app's background pool size, for the scenarios with bg_calls_key (app() sizes it to the
  // hardware concurrency). Built before pinning, like every app (make_app).
  {
    cvc::app probe;
    cfg.host.pool_workers = probe.getThreadPoolSize();
  }
  if (sample_tuned && sampling.prefix.empty())
    std::fprintf(stderr, "state_exec_bench: warning: --sample-us, --sample-depth and "
                         "--sample-all-reps do nothing without --sample\n");
  // The sampler's threads start before the driver is pinned, like the watchdog's.
  std::string sample_line;
  if (!sampling.prefix.empty()) {
    sample_line = the_sampler().start(sampling);
    if (!the_sampler().on())
      std::fprintf(stderr, "state_exec_bench: %s\n", sample_line.c_str());
  }
  cfg.host.pinned_cpu = pin_driver(pin);
  cfg.host.pin_how = pinning().how;
  cfg.host.pin_sibling = pinning().sibling;
  if (the_sampler().on())
    sample_line += the_sampler().avoid_cpus(cfg.host.pinned_cpu, cfg.host.pin_sibling);
  if (priority == "high")
    cfg.host.priority = raise_priority();
  else if (!priority.empty() && priority != "normal") {
    std::fprintf(stderr, "state_exec_bench: --priority expects high or normal\n");
    return 2;
  }
  cfg.host.peak_resets = reset_peak_memory();

#if defined(NDEBUG)
  const bool ndebug = true;
#else
  const bool ndebug = false;
#endif
  std::printf("[state_exec_bench] compiler %s | NDEBUG %d | sizeof value_t %zu, process %zu, "
              "eval_frame %zu, evaluator_state %zu, environment %zu, closure %zu, native_fn %zu, "
              "std::string %zu\n",
              compiler_name().c_str(), ndebug ? 1 : 0, sizeof(se::value_t), sizeof(se::process),
              sizeof(se::eval_frame), sizeof(se::evaluator_state), sizeof(se::environment),
              sizeof(se::closure), sizeof(se::native_fn), sizeof(std::string));
  std::printf("[state_exec_bench] logical CPUs %u | driver pinned to %d (%s)%s%s | priority %s | "
              "sleep_for(1ms) takes %.2f ms | app pool %u workers | reps %d, warmup %d%s\n",
              cfg.host.logical_cpus, cfg.host.pinned_cpu, cfg.host.pin_how.c_str(),
              cfg.host.pin_sibling.empty() ? "" : ", SMT sibling ", cfg.host.pin_sibling.c_str(),
              cfg.host.priority.c_str(), double(cfg.host.sleep_1ms_ns) / 1e6, cfg.host.pool_workers,
              cfg.reps, cfg.warmup, cfg.smoke ? " | smoke" : "");
  std::printf("[state_exec_bench] allocations counted: %s | peak memory %s\n", bench_alloc::scope,
              cfg.host.peak_resets ? "reset per sweep point" : "per process (not resettable)");
  if (the_sampler().on())
    std::printf("[state_exec_bench] %s\n", sample_line.c_str());
  std::fflush(stdout);

  std::FILE *json = nullptr;
  if (!cfg.json_path.empty()) {
    json = std::fopen(cfg.json_path.c_str(), "ab");
    if (!json) {
      std::fprintf(stderr, "state_exec_bench: cannot open %s\n", cfg.json_path.c_str());
      return 2;
    }
  }

  for (const auto &[k, v] : cfg.size_over) {
    bool used = false;
    for (const scenario *sc : chosen)
      for (const auto &[sk, sv] : sizes_for(*sc, cfg.smoke))
        used = used || sk == k;
    if (!used)
      std::fprintf(stderr, "state_exec_bench: warning: no chosen scenario has size '%s'\n",
                   k.c_str());
  }
  the_watchdog().set_output(json);

  bool any_failed = false, any_measured = false, any_point = false;
  const auto t_start = clk::now();
  for (const scenario *scp : chosen) {
    const scenario &sc = *scp;
    for (const std::string &exec : sc.executors) {
      sizes_t sizes = sizes_for(sc, cfg.smoke);
      for (auto &[k, v] : sizes)
        if (auto it = cfg.size_over.find(k); it != cfg.size_over.end())
          v = it->second;
      if (!cfg.executors.empty() && !exec.empty() &&
          std::find(cfg.executors.begin(), cfg.executors.end(), exec) == cfg.executors.end())
        continue;
      // The cartesian product of the sweep keys; a key this executor doesn't vary takes its first
      // value and stays out of the params.
      std::vector<params_t> points{{}};
      for (const auto &[k, vals] : sizes) {
        if (!sc.varies(k, exec))
          continue;
        std::vector<params_t> next;
        for (const auto &pt : points)
          for (const auto &v : vals) {
            params_t q = pt;
            q.push_back({k, v});
            next.push_back(std::move(q));
          }
        points = std::move(next);
      }
      for (params_t &pt : points) {
        point_ctx pc{sc, exec, pt, cfg, {}, 0, false};
        // A combination the scenario rules out (an inline kernel on the async scheduler, say) is
        // not a usage error: the runner's --split asks for each combination by itself.
        any_point = true;
        if (sc.accept && !sc.accept(exec, pc))
          continue;
        if (!sc.bg_calls_key.empty() && exec == "async" &&
            pc.s("where", "background") == "background" &&
            pc.i(sc.bg_calls_key) > static_cast<int64_t>(cfg.host.pool_workers)) {
          if (cfg.smoke) {
            // A smoke run checks the path, not the size: clamp to what the pool runs at once
            // (hosted CI runners have 2-4 workers).
            for (auto &[k, v] : pc.params)
              if (k == sc.bg_calls_key)
                v = istr(std::max<unsigned>(1, cfg.host.pool_workers));
          } else if (!cfg.bg_oversubscribe) {
            std::printf("  %-28s %-9s %-34s skipped: %s background calls at once exceed the app "
                        "pool's %u workers (5 s key-collision stalls, possible hang); run it "
                        "with --bg-oversubscribe\n",
                        sc.name.c_str(), exec.c_str(), params_text(pt).c_str(),
                        pc.s(sc.bg_calls_key).c_str(), cfg.host.pool_workers);
            continue;
          }
        }
        // Peak memory belongs to this point when the OS can reset it; else to the process, and
        // an earlier point's peak may show in this one's records.
        const bool reset = cfg.host.peak_resets && reset_peak_memory();
        pc.peak_inherited = !reset && any_measured;
        any_measured = true;
        driver_unpinned_for_point unpinned(sc.spawns_threads);
        pc.pin_cpu = unpinned.active ? -1 : cfg.host.pinned_cpu;
        // Setup and teardown (~app joins every thread it started) are watched too.
        const int warm = cfg.warmup_set || sc.warmup < 0 ? cfg.warmup : sc.warmup;
        the_watchdog().arm_point((warm + cfg.reps) * (cfg.timeout + 60.0) + 120.0,
                                 sc.name + " " + exec + " " + params_text(pc.params),
                                 hung_record(pc, -1,
                                             "hung: the sweep point outlived its budget (its "
                                             "setup or teardown blocked); the process was ended"));
        try {
          sc.run(pc);
        } catch (const std::exception &e) {
          meter m;
          m.begin();
          m.end();
          rep_out o;
          o.fail(std::string("setup: ") + e.what());
          pc.record(-1, m, o);
        }
        the_watchdog().disarm_point();
        // Thread and handle counts once the point's app and worlds are gone: what leaked.
        const int threads = thread_count();
        const int64_t handles = handle_count();
        for (jobj &r : pc.records) {
          r.i("thread_count_teardown", threads);
          if (handles >= 0)
            r.i("handle_count_teardown", handles);
          if (json)
            std::fprintf(json, "%s\n", r.str().c_str());
        }
        if (json)
          std::fflush(json);
        any_failed = any_failed || pc.failed;
      }
    }
  }
  the_sampler().finish(); // --sample: symbolize and write the sampled reps' files
  if (!any_point) {
    std::fprintf(stderr, "state_exec_bench: the selection runs no sweep point (check --executor "
                         "against the scenarios' executors)\n");
    if (json)
      std::fclose(json);
    return 2;
  }
  std::printf("[state_exec_bench] done in %.1f s%s%s\n",
              std::chrono::duration<double>(clk::now() - t_start).count(),
              any_measured ? "" : " -- every point was skipped",
              any_failed ? " -- SOME CHECKS FAILED" : "");
  if (json)
    std::fclose(json);
  return any_failed ? 1 : 0;
}
