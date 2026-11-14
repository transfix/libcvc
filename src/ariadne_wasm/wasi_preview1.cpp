// wasi_preview1.cpp — see the header: WASI preview 1's 46 functions as stubs, from a table of their
// names and signatures (wasi_snapshot_preview1.witx).

#include "wasi_preview1.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <exception>
#include <iterator>
#include <random>
#include <span>
#include <vector>

namespace cvc {
namespace ariadne {
namespace wasm {

namespace {

// The errnos the stubs return (preview 1's errno enum).
constexpr std::int64_t kSuccess = 0;
constexpr std::int64_t kBadf = 8;        // no such descriptor
constexpr std::int64_t kFault = 21;      // a pointer outside the guest's memory
constexpr std::int64_t kInval = 28;      // an argument out of range
constexpr std::int64_t kIo = 29;         // the host's random numbers failed
constexpr std::int64_t kNotCapable = 76; // nothing grants it

using Args = std::span<const std::int64_t>;

// An i32 parameter as WASI means it, unsigned: widen() sign-extends, so a pointer above 2 GiB
// arrives negative.
std::uint32_t u32(std::int64_t v) { return static_cast<std::uint32_t>(v); }

// The stubs read and write the guest's memory by copies (GuestMemory::read, write): in the browser
// it isn't the host's, so no host pointer reaches it.

// Whether [at, at + n) is inside the guest's memory: where it isn't, the call's EFAULT.
bool inside(GuestMemory &mem, std::uint32_t at, std::uint64_t n) {
  const std::uint64_t size = mem.size();
  return size > 0 && at <= size && n <= size - at;
}

// Little-endian, as the guest's memory is.
void put_u32(std::uint8_t *p, std::uint32_t v) {
  for (int i = 0; i < 4; ++i)
    p[i] = static_cast<std::uint8_t>(v >> (8 * i));
}

void put_u64(std::uint8_t *p, std::uint64_t v) {
  for (int i = 0; i < 8; ++i)
    p[i] = static_cast<std::uint8_t>(v >> (8 * i));
}

std::uint32_t get_u32(const std::uint8_t *p) {
  return std::uint32_t(p[0]) | std::uint32_t(p[1]) << 8 | std::uint32_t(p[2]) << 16 |
         std::uint32_t(p[3]) << 24;
}

std::uint64_t get_u64(const std::uint8_t *p) {
  return std::uint64_t(get_u32(p)) | std::uint64_t(get_u32(p + 4)) << 32;
}

// A u32 or a u64 written at `at`: false where that isn't inside the memory.
bool store_u32(GuestMemory &mem, std::uint32_t at, std::uint32_t v) {
  std::array<std::uint8_t, 4> b;
  put_u32(b.data(), v);
  return mem.write(at, b);
}

bool store_u64(GuestMemory &mem, std::uint32_t at, std::uint64_t v) {
  std::array<std::uint8_t, 8> b;
  put_u64(b.data(), v);
  return mem.write(at, b);
}

std::uint64_t arg_bytes(const WasiState &w) {
  std::uint64_t n = 0;
  for (const std::string &s : w.args)
    n += s.size() + 1;
  return n;
}

// The most (i)ovecs one call takes, wasi-libc's IOV_MAX: more is EINVAL, as writev has it. It
// bounds the host's walk of them inside one slice, which a step per call couldn't (zero-length
// iovecs cost the guest next to nothing).
constexpr std::uint32_t kIovMax = 1024;

// The (i)ovecs at [iovs, iovs + 8 n), each a pointer and a length: their bytes in all in `total`,
// or EFAULT when one isn't inside the memory, EINVAL when there are more than kIovMax or they pass
// a u32. Walking them moves their 8 n bytes, charged whatever the walk finds.
std::int64_t iov_bytes(WasiState &w, GuestMemory &mem, std::uint32_t iovs, std::uint32_t n,
                       std::uint64_t &total) {
  if (n > kIovMax)
    return kInval;
  std::vector<std::uint8_t> v(8 * std::size_t(n));
  if (!inside(mem, iovs, v.size()) || !mem.read(iovs, v))
    return kFault;
  w.moved += 8 * std::uint64_t(n);
  total = 0;
  for (std::uint32_t i = 0; i < n; ++i) {
    const std::uint8_t *e = v.data() + 8 * std::size_t(i);
    const std::uint32_t len = get_u32(e + 4);
    if (!inside(mem, get_u32(e), len))
      return kFault;
    total += len;
  }
  return total > 0xffffffffu ? kInval : kSuccess;
}

// ── the stubs: (state, memory, i32/i64 parameters, trap) -> errno ────────────────────────────────

using Stub = std::int64_t (*)(WasiState &, GuestMemory &, Args, std::string &);

std::int64_t not_capable(WasiState &, GuestMemory &, Args, std::string &) { return kNotCapable; }

// fd_prestat_get, fd_prestat_dir_name: no fd is a preopened directory, so wasi-libc's scan of them
// ends at fd 3 (any other errno there exits the guest at startup).
std::int64_t bad_fd(WasiState &, GuestMemory &, Args, std::string &) { return kBadf; }

std::int64_t sched_yield(WasiState &, GuestMemory &, Args, std::string &) { return kSuccess; }

// args_sizes_get(argc, argv_buf_size)
std::int64_t args_sizes_get(WasiState &w, GuestMemory &mem, Args a, std::string &) {
  if (!inside(mem, u32(a[0]), 4) || !inside(mem, u32(a[1]), 4))
    return kFault;
  store_u32(mem, u32(a[0]), static_cast<std::uint32_t>(w.args.size()));
  store_u32(mem, u32(a[1]), static_cast<std::uint32_t>(arg_bytes(w)));
  return kSuccess;
}

// args_get(argv, argv_buf): the strings, each NUL-terminated, one after the other in argv_buf, and
// a pointer to each in argv.
std::int64_t args_get(WasiState &w, GuestMemory &mem, Args a, std::string &) {
  const std::uint64_t bytes = arg_bytes(w);
  if (!inside(mem, u32(a[0]), 4 * std::uint64_t(w.args.size())) || !inside(mem, u32(a[1]), bytes))
    return kFault;
  std::vector<std::uint8_t> argv(4 * w.args.size()), buf;
  buf.reserve(static_cast<std::size_t>(bytes));
  std::uint32_t at = u32(a[1]);
  for (std::size_t i = 0; i < w.args.size(); ++i) {
    const std::string &s = w.args[i];
    put_u32(argv.data() + 4 * i, at);
    buf.insert(buf.end(), s.begin(), s.end());
    buf.push_back(0);
    at += static_cast<std::uint32_t>(s.size() + 1);
  }
  mem.write(u32(a[0]), argv);
  mem.write(u32(a[1]), buf);
  w.moved += bytes + 4 * w.args.size();
  return kSuccess;
}

// environ_sizes_get(count, buf_size): no variables.
std::int64_t environ_sizes_get(WasiState &, GuestMemory &mem, Args a, std::string &) {
  if (!inside(mem, u32(a[0]), 4) || !inside(mem, u32(a[1]), 4))
    return kFault;
  store_u32(mem, u32(a[0]), 0);
  store_u32(mem, u32(a[1]), 0);
  return kSuccess;
}

// environ_get(environ, environ_buf): nothing to write.
std::int64_t environ_get(WasiState &, GuestMemory &, Args, std::string &) { return kSuccess; }

// The clocks: realtime (0), monotonic (1), and the process's and the thread's CPU time (2, 3),
// which read as monotonic.
constexpr std::uint32_t kClocks = 4;

// clock_res_get(id, resolution): a microsecond, what the stubs promise of every clock.
std::int64_t clock_res_get(WasiState &, GuestMemory &mem, Args a, std::string &) {
  if (u32(a[0]) >= kClocks)
    return kInval;
  return store_u64(mem, u32(a[1]), 1000) ? kSuccess : kFault;
}

// clock_time_get(id, precision: i64, time): nanoseconds, since the epoch for realtime.
std::int64_t clock_time_get(WasiState &, GuestMemory &mem, Args a, std::string &) {
  using namespace std::chrono;
  const std::uint32_t id = u32(a[0]);
  if (id >= kClocks)
    return kInval;
  const std::int64_t ns =
      id == 0 ? duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count()
              : duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
  return store_u64(mem, u32(a[2]), ns > 0 ? static_cast<std::uint64_t>(ns) : 0) ? kSuccess : kFault;
}

// random_get(buf, len): the host's random numbers (std::random_device).
std::int64_t random_get(WasiState &w, GuestMemory &mem, Args a, std::string &) {
  const std::uint32_t n = u32(a[1]);
  if (!inside(mem, u32(a[0]), n))
    return kFault;
  try {
    std::random_device rd;
    // In chunks, so a large request never takes a host buffer its size. A 64-bit index: a u32 one
    // wraps past 0xfffffffc, and the loop wouldn't end.
    std::array<std::uint8_t, 4096> chunk;
    for (std::uint64_t done = 0; done < n;) {
      const std::size_t m =
          static_cast<std::size_t>(std::min<std::uint64_t>(chunk.size(), n - done));
      for (std::size_t i = 0; i < m; i += 4) {
        const std::uint32_t r = rd();
        for (std::size_t k = 0; k < 4 && i + k < m; ++k)
          chunk[i + k] = static_cast<std::uint8_t>(r >> (8 * k));
      }
      mem.write(u32(a[0]) + static_cast<std::uint32_t>(done), std::span(chunk.data(), m));
      done += m;
    }
  } catch (const std::exception &) {
    return kIo;
  }
  w.moved += n;
  return kSuccess;
}

// proc_exit(code): ends the guest (a trap the engine reads as the exit, and the instance's end),
// never the host.
std::int64_t proc_exit(WasiState &w, GuestMemory &, Args a, std::string &trap) {
  const std::uint32_t code = u32(a[0]);
  w.exit = static_cast<int>(code);
  trap = "proc_exit(" + std::to_string(code) + ")";
  return 0;
}

// The three stdio descriptors are character devices: stdin readable, stdout and stderr writable.
constexpr std::uint8_t kCharacterDevice = 2;
constexpr std::uint64_t kRightFdRead = std::uint64_t(1) << 1;
constexpr std::uint64_t kRightFdWrite = std::uint64_t(1) << 6;

// fd_fdstat_get(fd, fdstat): filetype (u8), flags (u16 at 2), rights base and inheriting (u64s
// at 8 and 16).
std::int64_t fd_fdstat_get(WasiState &, GuestMemory &mem, Args a, std::string &) {
  const std::uint32_t fd = u32(a[0]);
  if (fd > 2)
    return kNotCapable;
  std::array<std::uint8_t, 24> st{};
  st[0] = kCharacterDevice;
  put_u64(st.data() + 8, fd == 0 ? kRightFdRead : kRightFdWrite);
  return mem.write(u32(a[1]), st) ? kSuccess : kFault;
}

// fd_filestat_get(fd, filestat): a character device (filetype, a u8 at 16); its device, inode,
// links, size and times 0.
std::int64_t fd_filestat_get(WasiState &, GuestMemory &mem, Args a, std::string &) {
  if (u32(a[0]) > 2)
    return kNotCapable;
  std::array<std::uint8_t, 64> st{};
  st[16] = kCharacterDevice;
  return mem.write(u32(a[1]), st) ? kSuccess : kFault;
}

// fd_write(fd, iovs, iovs_len, nwritten): stdout and stderr take every byte, and drop it.
std::int64_t fd_write(WasiState &w, GuestMemory &mem, Args a, std::string &) {
  const std::uint32_t fd = u32(a[0]);
  if (fd != 1 && fd != 2)
    return kNotCapable;
  std::uint64_t total = 0;
  if (const std::int64_t e = iov_bytes(w, mem, u32(a[1]), u32(a[2]), total))
    return e;
  if (!store_u32(mem, u32(a[3]), static_cast<std::uint32_t>(total)))
    return kFault;
  w.moved += total;
  return kSuccess;
}

// fd_read(fd, iovs, iovs_len, nread): stdin is at its end.
std::int64_t fd_read(WasiState &w, GuestMemory &mem, Args a, std::string &) {
  if (u32(a[0]) != 0)
    return kNotCapable;
  std::uint64_t room = 0;
  if (const std::int64_t e = iov_bytes(w, mem, u32(a[1]), u32(a[2]), room))
    return e;
  return store_u32(mem, u32(a[3]), 0) ? kSuccess : kFault;
}

// poll_oneoff's records: a subscription (48 bytes: userdata u64, the event type u8 at 8, a clock's
// id u32 at 16), and an event (32 bytes: userdata u64, errno u16 at 8, the type u8 at 10).
constexpr std::uint64_t kSubscriptionBytes = 48;
constexpr std::uint64_t kEventBytes = 32;
constexpr std::uint8_t kEventClock = 0, kEventFdRead = 1, kEventFdWrite = 2;
constexpr std::uint32_t kMaxSubscriptions = 1024; // per call, as kIovMax bounds the iovecs

// poll_oneoff(in, out, nsubscriptions, nevents): the clocks alone, and none waits. Each clock
// subscription fires at once, as if its time had come, so a guest's sleep returns at once and
// never blocks the loop that steps the host. An fd's subscription is ENOTCAPABLE (nothing grants an
// fd to wait on). No subscription, or more than kMaxSubscriptions, is EINVAL, as an unknown type or
// clock is.
std::int64_t poll_oneoff(WasiState &w, GuestMemory &mem, Args a, std::string &) {
  const std::uint32_t n = u32(a[2]);
  if (n == 0 || n > kMaxSubscriptions)
    return kInval;
  std::vector<std::uint8_t> in(static_cast<std::size_t>(kSubscriptionBytes * n));
  if (!inside(mem, u32(a[0]), in.size()) || !inside(mem, u32(a[1]), kEventBytes * n) ||
      !inside(mem, u32(a[3]), 4) || !mem.read(u32(a[0]), in))
    return kFault;
  w.moved += kSubscriptionBytes * n;
  // Every subscription is read before an event is written: the two may overlap.
  std::vector<std::uint64_t> fired;
  fired.reserve(n);
  for (std::uint32_t i = 0; i < n; ++i) {
    const std::uint8_t *s = in.data() + kSubscriptionBytes * i;
    const std::uint8_t type = s[8];
    if (type == kEventFdRead || type == kEventFdWrite)
      return kNotCapable;
    if (type != kEventClock || get_u32(s + 16) >= kClocks)
      return kInval;
    fired.push_back(get_u64(s));
  }
  std::vector<std::uint8_t> out(static_cast<std::size_t>(kEventBytes * fired.size()), 0);
  for (std::size_t i = 0; i < fired.size(); ++i) {
    put_u64(out.data() + kEventBytes * i, fired[i]);
    out[kEventBytes * i + 10] = kEventClock;
  }
  mem.write(u32(a[1]), out);
  store_u32(mem, u32(a[3]), static_cast<std::uint32_t>(fired.size()));
  w.moved += kEventBytes * fired.size();
  return kSuccess;
}

// One preview-1 function: its name and its signature, every parameter an i32 but those `wide`
// marks, and an i32 errno as its result but for proc_exit's, which returns nothing.
struct Function {
  const char *name;
  int params;
  std::uint32_t wide; // bit i set: parameter i is an i64
  bool result;
  Stub stub;
};

constexpr std::uint32_t bit(int i) { return std::uint32_t(1) << i; }

// wasi_snapshot_preview1, in the witx's order.
constexpr Function kPreview1[] = {
    {"args_get", 2, 0, true, args_get},
    {"args_sizes_get", 2, 0, true, args_sizes_get},
    {"environ_get", 2, 0, true, environ_get},
    {"environ_sizes_get", 2, 0, true, environ_sizes_get},
    {"clock_res_get", 2, 0, true, clock_res_get},
    {"clock_time_get", 3, bit(1), true, clock_time_get},
    {"fd_advise", 4, bit(1) | bit(2), true, not_capable},
    {"fd_allocate", 3, bit(1) | bit(2), true, not_capable},
    {"fd_close", 1, 0, true, not_capable},
    {"fd_datasync", 1, 0, true, not_capable},
    {"fd_fdstat_get", 2, 0, true, fd_fdstat_get},
    {"fd_fdstat_set_flags", 2, 0, true, not_capable},
    {"fd_fdstat_set_rights", 3, bit(1) | bit(2), true, not_capable},
    {"fd_filestat_get", 2, 0, true, fd_filestat_get},
    {"fd_filestat_set_size", 2, bit(1), true, not_capable},
    {"fd_filestat_set_times", 4, bit(1) | bit(2), true, not_capable},
    {"fd_pread", 5, bit(3), true, not_capable},
    {"fd_prestat_get", 2, 0, true, bad_fd},
    {"fd_prestat_dir_name", 3, 0, true, bad_fd},
    {"fd_pwrite", 5, bit(3), true, not_capable},
    {"fd_read", 4, 0, true, fd_read},
    {"fd_readdir", 5, bit(3), true, not_capable},
    {"fd_renumber", 2, 0, true, not_capable},
    {"fd_seek", 4, bit(1), true, not_capable},
    {"fd_sync", 1, 0, true, not_capable},
    {"fd_tell", 2, 0, true, not_capable},
    {"fd_write", 4, 0, true, fd_write},
    {"path_create_directory", 3, 0, true, not_capable},
    {"path_filestat_get", 5, 0, true, not_capable},
    {"path_filestat_set_times", 7, bit(4) | bit(5), true, not_capable},
    {"path_link", 7, 0, true, not_capable},
    {"path_open", 9, bit(5) | bit(6), true, not_capable},
    {"path_readlink", 6, 0, true, not_capable},
    {"path_remove_directory", 3, 0, true, not_capable},
    {"path_rename", 6, 0, true, not_capable},
    {"path_symlink", 5, 0, true, not_capable},
    {"path_unlink_file", 3, 0, true, not_capable},
    {"poll_oneoff", 4, 0, true, poll_oneoff},
    {"proc_exit", 1, 0, false, proc_exit},
    {"proc_raise", 1, 0, true, not_capable},
    {"sched_yield", 0, 0, true, sched_yield},
    {"random_get", 2, 0, true, random_get},
    {"sock_accept", 3, 0, true, not_capable},
    {"sock_recv", 6, 0, true, not_capable},
    {"sock_send", 5, 0, true, not_capable},
    {"sock_shutdown", 2, 0, true, not_capable},
};
static_assert(std::size(kPreview1) == 46, "wasi_snapshot_preview1 has 46 functions");

} // namespace

std::vector<Import> wasi_imports(WasiState &w) {
  std::vector<Import> out;
  out.reserve(std::size(kPreview1));
  for (const Function &f : kPreview1) {
    Import im;
    im.module = kWasiModule;
    im.name = f.name;
    im.params = f.params;
    im.wide = f.wide;
    im.returns = f.result;
    im.returns_i32 = f.result;
    im.sync = [&w, stub = f.stub](GuestMemory &mem, Args a, std::string &trap) -> std::int64_t {
      ++w.calls;
      return stub(w, mem, a, trap);
    };
    out.push_back(std::move(im));
  }
  return out;
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
