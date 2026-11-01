#ifndef CVC_CORE_THREAD_CLASS_H
#define CVC_CORE_THREAD_CLASS_H

// cvc::thread_class: how much of the machine a thread, or the whole process, asks the OS for
// (docs/STATE_EXEC_LOCKS.md 7.6). For hosts only: no DSL intrinsic reaches it. Never above normal,
// never realtime. Every call returns whether the OS took it; the callers log the result.
//
// - Windows: background is THREAD_MODE_BACKGROUND_BEGIN, utility THREAD_PRIORITY_BELOW_NORMAL,
//   normal THREAD_MODE_BACKGROUND_END and THREAD_PRIORITY_NORMAL.
// - Linux: the thread's nice value (setpriority on its tid), 10, 5 and 0. An unprivileged thread
//   cannot raise it back (that needs CAP_SYS_NICE or an RLIMIT_NICE that allows it), so going
//   from background to utility or normal returns false there.
// - macOS: QOS_CLASS_BACKGROUND, QOS_CLASS_UTILITY and QOS_CLASS_DEFAULT. Background and utility
//   also steer threads to the efficiency cores.
// - Emscripten, and any other platform: false.

namespace cvc {

enum class thread_class { background, utility, normal };

// Put the calling thread in `cls`. False when the OS refused it or the platform has none.
bool set_current_thread_class(thread_class cls);

// Put the whole process in `cls`, from any thread. Windows: SetPriorityClass (background is
// PROCESS_MODE_BACKGROUND_BEGIN, utility BELOW_NORMAL_PRIORITY_CLASS). macOS: background is
// PRIO_DARWIN_BG, which also throttles I/O; utility returns false. Linux has no process-wide class:
// the class is stored, every thread that exists now (the main thread included) is set to it
// through /proc/self/task, and the library's worker loops (thread_pool, async_lane, the app's pool)
// apply it to each worker they start (detail::apply_process_class_here). Any other new thread
// inherits the nice value of the thread that creates it. False when any existing thread refused.
bool set_process_class(thread_class cls);

namespace detail {
// Linux: give the calling thread the class set_process_class stored, if any (one relaxed load when
// none was set). A worker loop calls it as it starts. Elsewhere the class is process-wide already.
#if defined(__linux__) && !defined(__EMSCRIPTEN__)
void apply_process_class_here();
#else
inline void apply_process_class_here() {}
#endif
} // namespace detail

} // namespace cvc

#endif // CVC_CORE_THREAD_CLASS_H
