// jspi_host.cpp: the host side of the JSPI-against-Asyncify bench (jspi_bench.mjs). It stands in
// for ariglx, which polls a guest from deep in its scheduler: run(depth, wait) recurses `depth`
// frames, each holding locals across the call and an object with a destructor, as a real host's
// frames do, then polls the page's Module.benchPoll() until the guest's call is done, and returns
// how many polls it took.
//
// It is built several ways (jspi_bench.mjs): under -sASYNCIFY (as ariglx is) or -sJSPI, and with
// no exceptions, with Emscripten's JS exceptions (-fexceptions, as libcvc builds: a call past an
// object with a destructor then goes through an invoke_* in JS, as ariglx's do), or with wasm's
// own (-fwasm-exceptions). A poll that runs the next slice at once (an Asyncify guest) is a plain
// call (poll_now). A poll that has to wait for the next slice (a JSPI guest, which resumes from
// the event loop) is an async import (poll_wait), so the host's own stack unwinds and rewinds
// around each one, or is suspended and resumed.

#include <emscripten.h>

#define NOINLINE __attribute__((noinline))

EM_JS(int, poll_now, (void), { return Module.benchPoll(); });
EM_ASYNC_JS(int, poll_wait, (void), { return await Module.benchPoll(); });

static volatile int sink;

namespace {

// A frame's object with a destructor: with exceptions on, each call it outlives has a landing pad.
struct Held {
  int v;
  ~Held() { sink = sink + v; }
};

NOINLINE int deep(int depth, int wait) {
  const int a = depth * 3 + sink, b = depth ^ sink, d = depth + 7 * sink;
  Held held{depth & 1};
  if (depth > 0) {
    const int n = deep(depth - 1, wait);
    // a, b and d live across the call; reading sink after it keeps the recursion a recursion (an
    // addition after the call alone, LLVM would make a loop of).
    return n + ((a ^ b) + sink - d == 12345);
  }
  int n = 0;
  while (wait ? poll_wait() : poll_now())
    ++n;
  return n + 1;
}

} // namespace

extern "C" EMSCRIPTEN_KEEPALIVE int run(int depth, int wait) { return deep(depth, wait); }
