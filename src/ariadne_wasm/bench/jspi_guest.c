// jspi_guest.c: the guests of the JSPI-against-Asyncify bench (jspi_bench.mjs; roadmap
// ARIADNE_SCRIPTING_HOSTS.md §8.5). Plain C with no imports, built as a standalone module, then
// metered (ari_wasm_meter) and, for the Asyncify side, transformed as cvc_ariadne_wasm_transform
// does. Each workload takes its size and returns a checksum, so every variant can be checked
// against the unmetered one.
//
//   spin    a counted loop: no calls, one metered block per iteration
//   deep    spin, 48 frames down, for a pause's cost with a deep stack
//   fib     naive recursion: calls everywhere, and a deep stack wherever a slice ends
//   sort    libc's qsort with a comparator: library code, an indirect call per comparison
//   sha     SHA-256 over a buffer: straight-line arithmetic in large blocks
//   interp  a small bytecode interpreter dispatching through a table of handlers, as a scripting
//           guest's eval loop does: an indirect call per instruction

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define EXPORT(name) __attribute__((export_name(name)))
#define NOINLINE __attribute__((noinline))

// The room an Asyncify pause unwinds the guest's stack into (the bench's driver sets its header).
static unsigned char room[64 * 1024];
EXPORT("room") uint32_t bench_room(void) { return (uint32_t)(uintptr_t)room; }
EXPORT("room_size") uint32_t bench_room_size(void) { return sizeof room; }

EXPORT("spin") uint32_t spin(uint32_t n) {
  uint32_t acc = 0;
  for (uint32_t i = 0; i < n; ++i)
    acc = acc * 31u + (i ^ (acc >> 7));
  return acc;
}

// spin, 48 frames down: where Asyncify's pause unwinds and rewinds a deep stack.
static NOINLINE uint32_t nest_(uint32_t d, uint32_t n) {
  if (d == 0)
    return spin(n);
  volatile uint32_t keep = d; // read after the call, so it isn't a loop
  const uint32_t r = nest_(d - 1, n);
  return r + keep;
}
EXPORT("deep") uint32_t deep(uint32_t n) { return nest_(48, n); }

static NOINLINE uint32_t fib_(uint32_t n) { return n < 2 ? n : fib_(n - 1) + fib_(n - 2); }
EXPORT("fib") uint32_t fib(uint32_t n) { return fib_(n); }

#define SORT_MAX (1u << 20)
static uint32_t keys[SORT_MAX];
static int by_value(const void *a, const void *b) {
  const uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
  return (x > y) - (x < y);
}
EXPORT("sort") uint32_t sort(uint32_t n) {
  if (n > SORT_MAX)
    n = SORT_MAX;
  uint32_t s = 2463534242u;
  for (uint32_t i = 0; i < n; ++i) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    keys[i] = s;
  }
  qsort(keys, n, sizeof keys[0], by_value);
  uint32_t acc = 0;
  for (uint32_t i = 0; i < n; i += 97)
    acc = acc * 33u + keys[i];
  return acc;
}

// SHA-256 (FIPS 180-4), over `kib` KiB of a fixed pattern.
static const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
#define ROR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
static NOINLINE void sha_block(uint32_t h[8], const unsigned char *p) {
  uint32_t w[64];
  for (int i = 0; i < 16; ++i)
    w[i] = (uint32_t)p[4 * i] << 24 | (uint32_t)p[4 * i + 1] << 16 | (uint32_t)p[4 * i + 2] << 8 |
           p[4 * i + 3];
  for (int i = 16; i < 64; ++i) {
    const uint32_t s0 = ROR(w[i - 15], 7) ^ ROR(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const uint32_t s1 = ROR(w[i - 2], 17) ^ ROR(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], k = h[7];
  for (int i = 0; i < 64; ++i) {
    const uint32_t t1 =
        k + (ROR(e, 6) ^ ROR(e, 11) ^ ROR(e, 25)) + ((e & f) ^ (~e & g)) + K[i] + w[i];
    const uint32_t t2 = (ROR(a, 2) ^ ROR(a, 13) ^ ROR(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
    k = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  h[0] += a, h[1] += b, h[2] += c, h[3] += d, h[4] += e, h[5] += f, h[6] += g, h[7] += k;
}
static unsigned char chunk[1024];
EXPORT("sha") uint32_t sha(uint32_t kib) {
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                   0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  for (uint32_t i = 0; i < sizeof chunk; ++i)
    chunk[i] = (unsigned char)(i * 7 + 3);
  for (uint32_t k = 0; k < kib; ++k) {
    chunk[0] = (unsigned char)k;
    for (uint32_t off = 0; off < sizeof chunk; off += 64)
      sha_block(h, chunk + off);
  }
  return h[0] ^ h[7];
}

// A stack machine whose handlers are called through a table, the shape of an interpreter's eval
// loop. Its program sums (i * i) % 7 over [0, n).
typedef struct Vm {
  int32_t stack[16];
  int sp, pc;
  int32_t acc, i, n;
} Vm;
typedef void (*Op)(Vm *);
static NOINLINE void op_i(Vm *v) { v->stack[v->sp++] = v->i; }
static NOINLINE void op_dup(Vm *v) { v->stack[v->sp] = v->stack[v->sp - 1], ++v->sp; }
static NOINLINE void op_mul(Vm *v) { --v->sp, v->stack[v->sp - 1] *= v->stack[v->sp]; }
static NOINLINE void op_seven(Vm *v) { v->stack[v->sp++] = 7; }
static NOINLINE void op_mod(Vm *v) { --v->sp, v->stack[v->sp - 1] %= v->stack[v->sp]; }
static NOINLINE void op_add(Vm *v) { v->acc += v->stack[--v->sp]; }
static NOINLINE void op_next(Vm *v) { v->pc = ++v->i < v->n ? -1 : 100; }
static Op ops[] = {op_i, op_dup, op_mul, op_seven, op_mod, op_add, op_next};
static const unsigned char program[] = {0, 1, 2, 3, 4, 5, 6};
EXPORT("interp") uint32_t interp(uint32_t n) {
  Vm v = {.sp = 0, .pc = 0, .acc = 0, .i = 0, .n = (int32_t)n};
  if (n == 0)
    return 0;
  while (v.pc < (int)sizeof program) {
    ops[program[v.pc]](&v);
    ++v.pc;
  }
  return (uint32_t)v.acc;
}
