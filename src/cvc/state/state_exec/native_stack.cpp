/*
  Copyright 2026 The University of Texas at Austin

  native_stack — where the calling thread's native stack ends, so nesting native calls can stop
                 short of it (detail::native_nesting).
*/

#include "native_stack.h"

#include <cstdint>
#include <limits>

#if defined(_MSC_VER)
#include <intrin.h>
#endif
#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif defined(__EMSCRIPTEN__)
#include <emscripten/stack.h>
#elif defined(__APPLE__) || defined(__linux__)
#include <pthread.h>
#endif

namespace cvc::state_exec::detail {

namespace {

// An address on this thread's stack, in this function's frame. Not a local's address: under
// AddressSanitizer's use-after-return check (on by default with gcc's and clang's runtimes on
// Linux) a function's locals live in a "fake stack" the runtime allocates off the thread's stack,
// so measuring from one said there was no stack left at all, and every native call threw. The
// frame address and the return address's slot stay on the real stack.
std::uintptr_t stack_here() {
#if defined(_MSC_VER)
  return reinterpret_cast<std::uintptr_t>(_AddressOfReturnAddress());
#elif (defined(__GNUC__) || defined(__clang__)) && !defined(__EMSCRIPTEN__)
  return reinterpret_cast<std::uintptr_t>(__builtin_frame_address(0));
#else
  // Emscripten: the stack whose end it reports is in linear memory, where a local's address is.
  const char here = 0;
  return reinterpret_cast<std::uintptr_t>(&here);
#endif
}

// The lowest address of this thread's stack, or 0 when the platform does not say.
std::uintptr_t stack_low() {
#if defined(_WIN32)
  // The reservation a stack address belongs to starts at the stack's lowest address (the guard
  // pages are inside it). VirtualQuery, unlike GetCurrentThreadStackLimits, is in every Windows
  // the build targets (_WIN32_WINNT is 0x0601).
  MEMORY_BASIC_INFORMATION info{};
  if (VirtualQuery(reinterpret_cast<const void *>(stack_here()), &info, sizeof info) == 0)
    return 0;
  return reinterpret_cast<std::uintptr_t>(info.AllocationBase);
#elif defined(__EMSCRIPTEN__)
  return static_cast<std::uintptr_t>(emscripten_stack_get_end());
#elif defined(__APPLE__)
  const pthread_t self = pthread_self();
  const auto high = reinterpret_cast<std::uintptr_t>(pthread_get_stackaddr_np(self));
  return high - pthread_get_stacksize_np(self);
#elif defined(__linux__)
  pthread_attr_t attr;
  if (pthread_getattr_np(pthread_self(), &attr) != 0)
    return 0;
  void *addr = nullptr;
  std::size_t size = 0;
  const int rc = pthread_attr_getstack(&attr, &addr, &size);
  pthread_attr_destroy(&attr);
  return rc == 0 ? reinterpret_cast<std::uintptr_t>(addr) : 0;
#else
  return 0;
#endif
}

} // namespace

std::size_t native_stack_left() {
  thread_local const std::uintptr_t low = stack_low();
  if (low == 0)
    return std::numeric_limits<std::size_t>::max();
  const std::uintptr_t sp = stack_here();
  return sp > low ? static_cast<std::size_t>(sp - low) : 0;
}

} // namespace cvc::state_exec::detail
