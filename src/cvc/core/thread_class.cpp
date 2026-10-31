// cvc::thread_class — see inc/cvc/core/thread_class.h (docs/STATE_EXEC_LOCKS.md 7.6).

#include <cvc/core/thread_class.h>

#if defined(__EMSCRIPTEN__)
// No OS classes.
#elif defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__linux__)
#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <dirent.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#elif defined(__APPLE__)
#include <pthread.h>
#include <pthread/qos.h>
#include <sys/resource.h>
#endif

namespace cvc {

#if defined(__EMSCRIPTEN__)

bool set_current_thread_class(thread_class) { return false; }

bool set_process_class(thread_class) { return false; }

#elif defined(_WIN32)

bool set_current_thread_class(thread_class cls) {
  const HANDLE self = GetCurrentThread();
  if (cls == thread_class::background)
    // A thread already in background mode fails with ERROR_THREAD_MODE_ALREADY_BACKGROUND: it is
    // in the class it asked for.
    return SetThreadPriority(self, THREAD_MODE_BACKGROUND_BEGIN) != 0 ||
           GetLastError() == ERROR_THREAD_MODE_ALREADY_BACKGROUND;
  // Leave background mode first (it also lowers the thread's I/O and memory priority). A thread
  // not in it fails with ERROR_THREAD_MODE_NOT_BACKGROUND, which is no error here.
  if (SetThreadPriority(self, THREAD_MODE_BACKGROUND_END) == 0 &&
      GetLastError() != ERROR_THREAD_MODE_NOT_BACKGROUND)
    return false;
  return SetThreadPriority(self, cls == thread_class::utility ? THREAD_PRIORITY_BELOW_NORMAL
                                                              : THREAD_PRIORITY_NORMAL) != 0;
}

bool set_process_class(thread_class cls) {
  const HANDLE self = GetCurrentProcess();
  if (cls == thread_class::background)
    return SetPriorityClass(self, PROCESS_MODE_BACKGROUND_BEGIN) != 0 ||
           GetLastError() == ERROR_PROCESS_MODE_ALREADY_BACKGROUND;
  if (SetPriorityClass(self, PROCESS_MODE_BACKGROUND_END) == 0 &&
      GetLastError() != ERROR_PROCESS_MODE_NOT_BACKGROUND)
    return false;
  return SetPriorityClass(self, cls == thread_class::utility ? BELOW_NORMAL_PRIORITY_CLASS
                                                             : NORMAL_PRIORITY_CLASS) != 0;
}

#elif defined(__linux__)

namespace {

// The nice value set_process_class stored, for the workers that start after it; -1 for none.
std::atomic<int> g_process_nice{-1};

int nice_of(thread_class cls) {
  switch (cls) {
  case thread_class::background:
    return 10;
  case thread_class::utility:
    return 5;
  case thread_class::normal:
    break;
  }
  return 0;
}

// Linux's nice value is per thread: setpriority(PRIO_PROCESS, tid) sets one thread's.
bool set_thread_nice(long tid, int nice) {
  return setpriority(PRIO_PROCESS, static_cast<id_t>(tid), nice) == 0;
}

long current_tid() { return static_cast<long>(syscall(SYS_gettid)); }

} // namespace

bool set_current_thread_class(thread_class cls) {
  return set_thread_nice(current_tid(), nice_of(cls));
}

bool set_process_class(thread_class cls) {
  const int nice = nice_of(cls);
  // Stored first, so a worker that starts during the walk applies it too.
  g_process_nice.store(nice, std::memory_order_relaxed);
  DIR *dir = opendir("/proc/self/task");
  if (dir == nullptr)
    return false;
  bool all = true;
  while (const dirent *entry = readdir(dir)) {
    char *end = nullptr;
    const long tid = std::strtol(entry->d_name, &end, 10);
    if (end == entry->d_name || *end != '\0' || tid <= 0)
      continue; // "." and ".."
    // Demoting a thread of the same user needs no privilege; raising one back does. A thread that
    // ended since the listing (ESRCH) is no failure.
    if (!set_thread_nice(tid, nice) && errno != ESRCH)
      all = false;
  }
  closedir(dir);
  return all;
}

namespace detail {

void apply_process_class_here() {
  const int nice = g_process_nice.load(std::memory_order_relaxed);
  if (nice >= 0)
    (void)set_thread_nice(current_tid(), nice);
}

} // namespace detail

#elif defined(__APPLE__)

bool set_current_thread_class(thread_class cls) {
  const qos_class_t qos = cls == thread_class::background ? QOS_CLASS_BACKGROUND
                          : cls == thread_class::utility  ? QOS_CLASS_UTILITY
                                                          : QOS_CLASS_DEFAULT;
  return pthread_set_qos_class_self_np(qos, 0) == 0;
}

bool set_process_class(thread_class cls) {
#if defined(PRIO_DARWIN_PROCESS) && defined(PRIO_DARWIN_BG)
  switch (cls) {
  case thread_class::background:
    return setpriority(PRIO_DARWIN_PROCESS, 0, PRIO_DARWIN_BG) == 0;
  case thread_class::normal:
    return setpriority(PRIO_DARWIN_PROCESS, 0, 0) == 0;
  case thread_class::utility:
    break; // no process-wide utility class
  }
#else
  (void)cls;
#endif
  return false;
}

#else

bool set_current_thread_class(thread_class) { return false; }

bool set_process_class(thread_class) { return false; }

#endif

} // namespace cvc
