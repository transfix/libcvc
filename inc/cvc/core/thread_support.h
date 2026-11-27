/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#pragma once

// CVC_NO_THREADS: defined (to 1) where this build cannot start a thread.
//
//  - Emscripten without -pthread (CVC_WASM_PTHREADS=OFF): no SharedArrayBuffer worker behind
//    std::thread. With -pthread Emscripten defines __EMSCRIPTEN_PTHREADS__.
//  - wasm32-wasip1 (wasi-sdk, CMake's WASI): wasi-libc and libc++ keep the thread API, so a mutex
//    or a condition variable works, but starting a std::thread (or pthread_create) fails with
//    "Not supported". The wasm32-wasip1-threads target defines _REENTRANT.
//
// Code that would start a worker takes its single-threaded path under CVC_NO_THREADS: work runs
// inline, or where the host pumps it. What it may not do there is a browser's (an Emscripten
// timer, a JS callback); that stays behind __EMSCRIPTEN__.
#if (defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)) ||                              \
    (defined(__wasi__) && !defined(_REENTRANT))
#define CVC_NO_THREADS 1
#endif
