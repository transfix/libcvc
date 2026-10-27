/*
  Copyright 2003-2026 The University of Texas at Austin

        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// A rewrite of VolumeRover's `contourtree` library (inc/contourtree/HeightField.h and
// src/contourtree/computeCT.cpp, Copyright 2003 The University of Texas at Austin, Bajaj group,
// LGPL-2.1): the same triangulation, vertex order and tree, on flat arrays instead of a 72-byte
// struct plus four mallocs per voxel, with Carr's leaf condition restored in the combine step and a
// seeded layout. The original's cell queue (a separately copyrighted file) is not used.
//
// Everything runs as a resumable state machine (contour_tree_builder, contour_tree_simplifier):
// every loop keeps its cursor in the object and takes its items from a per-step budget, and the
// two loops whose single iteration is unbounded (the combine's lazy splice and the reduce's walk
// along an arc) are resumable mid-walk. The one-shot functions run the same machines to the end.

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cvc/volume/contour_tree.h>
#include <cvc/volume/voxels.h>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <new>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif !defined(__EMSCRIPTEN__) && (defined(__unix__) || defined(__APPLE__))
#include <sys/mman.h>
#if !defined(MAP_ANONYMOUS) && defined(MAP_ANON)
#define MAP_ANONYMOUS MAP_ANON
#endif
#define CVC_CT_MMAP 1
#endif
#ifndef CVC_CT_MMAP
#define CVC_CT_MMAP 0
#endif

namespace cvc {

namespace {

using u32 = std::uint32_t;
using u8 = std::uint8_t;
constexpr u32 kNone = std::numeric_limits<u32>::max();
constexpr uint64 kMaxVoxels = uint64(1) << 32;

[[noreturn]] void internal_error(const char *what) {
  throw contour_tree_error(std::string("internal error: ") + what);
}

// An uninitialized array: every working array is written before it is read, and value-initializing
// (a memset of the whole array) would be one unsliced O(n) pass.
template <class T> std::unique_ptr<T[]> uninit(std::size_t n) {
  static_assert(std::is_trivially_default_constructible_v<T>);
  return std::unique_ptr<T[]>(new T[n]);
}

// ---- budgets --------------------------------------------------------------------------------

// What a step may still spend, in units. Every loop takes its items through take(), so a step
// never does more than its budget; 0 means unlimited.
class budget {
public:
  explicit budget(std::size_t units) : left_(units), unlimited_(units == 0) {}
  // Of `want` items, how many to do now (all of them when unlimited); they are charged.
  std::size_t take(std::size_t want) {
    if (unlimited_)
      return want;
    const std::size_t k = want < left_ ? want : left_;
    left_ -= k;
    return k;
  }
  bool take_one() { return take(1) == 1; }
  bool unlimited() const { return unlimited_; }

private:
  std::size_t left_;
  bool unlimited_;
};

// The next `cursor .. stop` slice of [cursor, end) the budget allows.
template <class I> I slice_end(I cursor, I end, budget &b) {
  return I(cursor + I(b.take(std::size_t(end - cursor))));
}

// ---- working memory -------------------------------------------------------------------------

// Big working arrays come straight from the system (VirtualAlloc / mmap) so that a spent one can be
// given back a slice at a time: returning a touched multi-megabyte block in one call costs about
// 0.25 ms per MB on Windows, an O(n) pause inside a single step. Small arrays, and wasm (where
// free() is O(1) and memory never goes back to the system), use operator new. Either way a page is
// only committed when first written, so an allocation itself is O(1).
#if defined(_WIN32)
constexpr bool kSystemPages = true;
unsigned char *pages_alloc(std::size_t bytes) {
  return static_cast<unsigned char *>(
      VirtualAlloc(nullptr, bytes, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
}
void pages_free_part(unsigned char *p, std::size_t bytes) { VirtualFree(p, bytes, MEM_DECOMMIT); }
void pages_free_rest(unsigned char *base, std::size_t, std::size_t) {
  VirtualFree(base, 0, MEM_RELEASE);
}
#elif CVC_CT_MMAP
constexpr bool kSystemPages = true;
unsigned char *pages_alloc(std::size_t bytes) {
  void *p = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  return p == MAP_FAILED ? nullptr : static_cast<unsigned char *>(p);
}
void pages_free_part(unsigned char *p, std::size_t bytes) { munmap(p, bytes); }
void pages_free_rest(unsigned char *base, std::size_t done, std::size_t size) {
  munmap(base + done, size - done);
}
#else
constexpr bool kSystemPages = false;
unsigned char *pages_alloc(std::size_t) { return nullptr; }
void pages_free_part(unsigned char *, std::size_t) {}
void pages_free_rest(unsigned char *, std::size_t, std::size_t) {}
#endif

class region {
public:
  // Released 64 KiB at a time (a multiple of every page size, and of Windows' allocation
  // granularity, so a slice never splits a page).
  static constexpr std::size_t kSlice = std::size_t(64) << 10;

  region() = default;
  explicit region(std::size_t bytes) : size_(bytes) {
    if (bytes == 0)
      return;
    if (kSystemPages && bytes >= kSystemMin) {
      p_ = pages_alloc(bytes);
      if (!p_)
        throw std::bad_alloc();
      system_ = true;
    } else {
      p_ = static_cast<unsigned char *>(::operator new(bytes));
    }
  }
  region(region &&o) noexcept
      : p_(std::exchange(o.p_, nullptr)), size_(std::exchange(o.size_, 0)),
        done_(std::exchange(o.done_, 0)), system_(std::exchange(o.system_, false)) {}
  region &operator=(region &&o) noexcept {
    if (this != &o) {
      reset();
      p_ = std::exchange(o.p_, nullptr);
      size_ = std::exchange(o.size_, 0);
      done_ = std::exchange(o.done_, 0);
      system_ = std::exchange(o.system_, false);
    }
    return *this;
  }
  region(const region &) = delete;
  region &operator=(const region &) = delete;
  ~region() { reset(); }

  void *data() const { return p_; }
  explicit operator bool() const { return p_ != nullptr; }
  std::size_t remaining() const { return size_ - done_; }

  // Give back the next slice from the front (the whole block when it is not system pages, or
  // when no more than a slice is left); true once nothing is left.
  bool release_slice() {
    if (!p_)
      return true;
    if (!system_ || remaining() <= kSlice) {
      reset();
      return true;
    }
    pages_free_part(p_ + done_, kSlice);
    done_ += kSlice;
    return false;
  }

  void reset() noexcept {
    if (!p_)
      return;
    if (system_)
      pages_free_rest(p_, done_, size_);
    else
      ::operator delete(p_);
    p_ = nullptr;
    size_ = done_ = 0;
    system_ = false;
  }

private:
  static constexpr std::size_t kSystemMin = std::size_t(1) << 20;
  unsigned char *p_ = nullptr;
  std::size_t size_ = 0, done_ = 0;
  bool system_ = false;
};

// n uninitialized T in a region: every working array is written before it is read.
template <class T> class work_array {
  static_assert(std::is_trivially_default_constructible_v<T> &&
                std::is_trivially_destructible_v<T>);

public:
  work_array() = default;
  explicit work_array(std::size_t n) : r_(bytes(n)) {}
  T *get() const { return static_cast<T *>(r_.data()); }
  T &operator[](std::size_t i) const { return get()[i]; }
  explicit operator bool() const { return bool(r_); }
  region release() { return std::move(r_); }

private:
  static std::size_t bytes(std::size_t n) {
    if (n > std::numeric_limits<std::size_t>::max() / sizeof(T))
      throw std::bad_alloc();
    return n * sizeof(T);
  }
  region r_;
};

// Spent arrays, given back one 64 KiB slice at a time at one unit per KiB (about what a voxel
// costs), so no step pays for a whole array; unlimited, each goes back in one call.
class release_queue {
public:
  void add(region &&r) {
    if (r)
      q_.push_back(std::move(r));
  }
  template <class T> void add(work_array<T> &a) { add(a.release()); }
  bool empty() const { return q_.empty(); }

  bool run(budget &b) {
    while (!q_.empty()) {
      region &r = q_.back();
      if (b.unlimited()) {
        r.reset();
        q_.pop_back();
        continue;
      }
      const std::size_t units = (std::min(r.remaining(), region::kSlice) + 1023) / 1024;
      paid_ = std::min(paid_, units);
      paid_ += b.take(units - paid_);
      if (paid_ < units)
        return false;
      paid_ = 0;
      if (r.release_slice())
        q_.pop_back();
    }
    return true;
  }

private:
  std::vector<region> q_;
  std::size_t paid_ = 0;
};

// ---- the triangulation ----------------------------------------------------------------------

// HeightField::GetNeighbor's 14 offsets: the edges of the 6-tetrahedron split of every cube along
// its (+1,+1,-1) diagonal. Another diagonal is a different interpolant with different saddles, so
// this must stay as is for trees to match VolumeRover's.
constexpr int kStencil[14][3] = {{-1, 0, 0}, {1, 0, 0},  {0, -1, 0},  {0, 1, 0},  {0, 0, -1},
                                 {0, 0, 1},  {1, 1, 0},  {-1, -1, 0}, {0, 1, -1}, {0, -1, 1},
                                 {1, 0, -1}, {-1, 0, 1}, {1, 1, -1},  {-1, -1, 1}};

struct grid {
  u32 nx, ny, nz, nxy, n;
  u32 delta[14]; // linear offsets, modulo 2^32, so v + delta wraps to the right index

  grid(u32 x, u32 y, u32 z) : nx(x), ny(y), nz(z), nxy(x * y), n(x * y * z) {
    for (int t = 0; t < 14; ++t)
      delta[t] = static_cast<u32>(static_cast<std::int64_t>(kStencil[t][0]) +
                                  static_cast<std::int64_t>(kStencil[t][1]) * nx +
                                  static_cast<std::int64_t>(kStencil[t][2]) * nxy);
  }

  int neighbours(u32 v, u32 *out) const {
    const u32 i = v % nx, j = (v / nx) % ny, k = v / nxy;
    if (i > 0 && i + 1 < nx && j > 0 && j + 1 < ny && k > 0 && k + 1 < nz) {
      for (int t = 0; t < 14; ++t)
        out[t] = v + delta[t];
      return 14;
    }
    int c = 0;
    for (int t = 0; t < 14; ++t) {
      const int *d = kStencil[t];
      if ((d[0] < 0 && i == 0) || (d[0] > 0 && i + 1 == nx) || (d[1] < 0 && j == 0) ||
          (d[1] > 0 && j + 1 == ny) || (d[2] < 0 && k == 0) || (d[2] > 0 && k + 1 == nz))
        continue;
      out[c++] = v + delta[t];
    }
    return c;
  }
};

// ---- vertex order ---------------------------------------------------------------------------

// Simulation of simplicity: ties broken by linear index, exactly compareHeightVoid's order. The
// !(b < a) form keeps -0.0 and 0.0 tied, as the sort keys below do.
template <class T> inline bool order_less(const T *f, u32 a, u32 b) {
  return f[a] < f[b] || (!(f[b] < f[a]) && a < b);
}

// An unsigned key with the value's order (-0.0 and 0.0 tie): 32 bits for types up to 4 bytes,
// 64 for the rest.
template <class T>
using key_type = std::conditional_t<(sizeof(T) <= 4), std::uint32_t, std::uint64_t>;

template <class T> key_type<T> sort_key(T v) {
  using K = key_type<T>;
  if constexpr (std::is_floating_point_v<T>) {
    static_assert(sizeof(K) == sizeof(T));
    const T z = v == T(0) ? T(0) : v;
    K b;
    std::memcpy(&b, &z, sizeof b);
    constexpr K sign = K(K(1) << (8 * sizeof(K) - 1));
    return (b & sign) ? K(~b) : K(b | sign);
  } else if constexpr (std::is_signed_v<T>) {
    using U = std::make_unsigned_t<T>;
    return K(U(U(v) ^ U(U(1) << (8 * sizeof(T) - 1))));
  } else {
    return K(v);
  }
}

// One pass of the LSD radix sort: scatter [begin, end) by the digit (k >> shift) & mask. The
// first pass reads keys from the field (in index order); the last writes only indices.
template <bool FromField, bool Last, class W, class K>
void scatter(const W *f, K kmin, const K *skey, const u32 *sidx, K *dkey, u32 *didx, u32 *off,
             unsigned shift, K mask, u32 begin, u32 end) {
  for (u32 i = begin; i < end; ++i) {
    K k;
    u32 id;
    if constexpr (FromField) {
      k = K(sort_key(f[i]) - kmin);
      id = i;
    } else {
      k = skey[i];
      id = sidx[i];
    }
    const u32 pos = off[u32((k >> shift) & mask)]++;
    if constexpr (!Last)
      dkey[pos] = k;
    didx[pos] = id;
  }
}

// ---- union-find -----------------------------------------------------------------------------

// Path halving under union by rank: a set's tree is at most log2(n) < 32 deep, so a find is at
// most 32 hops.
inline u32 find_root(u32 *uf, u32 x) {
  while (uf[x] != x) {
    uf[x] = uf[uf[x]];
    x = uf[x];
  }
  return x;
}

// Union by rank; the set's extremum becomes v, the vertex being swept.
inline u32 unite(u32 *uf, u8 *rank, u32 *ext, u32 a, u32 b, u32 v) {
  if (rank[a] < rank[b])
    std::swap(a, b);
  uf[b] = a;
  if (rank[a] == rank[b])
    ++rank[a];
  ext[a] = v;
  return a;
}

contour_tree::node_kind kind_of(u32 up, u32 down) {
  using k = contour_tree::node_kind;
  if (down == 0)
    return k::minimum;
  if (up == 0)
    return k::maximum;
  if (up >= 2 && down >= 2)
    return k::join_split;
  return up >= 2 ? k::join : k::split;
}

// ---- the builder's phases -------------------------------------------------------------------

enum class ct_phase : std::uint8_t { preparing, sorting, join, split, merging, done };

constexpr const char *kPhaseName[] = {"preparing",  "sorting", "join tree",
                                      "split tree", "merging", "done"};
// Where each phase starts on the progress bar: its measured share of the time on 128^3 volumes
// (preparing < 1%, sorting 2-6%, each sweep ~35%, merging ~27%).
constexpr double kPhaseStart[] = {0.0, 0.01, 0.05, 0.40, 0.74, 1.0};

double overall_progress(ct_phase p, double fraction) {
  const int i = int(p);
  if (i >= int(ct_phase::done))
    return 1.0;
  const double f = fraction < 0.0 ? 0.0 : (fraction > 0.999 ? 0.999 : fraction);
  return kPhaseStart[i] + (kPhaseStart[i + 1] - kPhaseStart[i]) * f;
}

double ratio(uint64 done, uint64 total) { return total == 0 ? 1.0 : double(done) / double(total); }

class ct_task {
public:
  virtual ~ct_task() = default;
  virtual bool run(budget &b) = 0; // true once the tree is ready
  virtual ct_phase phase() const = 0;
  virtual double fraction() const = 0; // through the current phase
  virtual bool uses_input() const = 0; // whether the caller's voxels are still read
  virtual contour_tree take() = 0;
};

// ---- the tree -------------------------------------------------------------------------------

// The contour tree of f on an nx*ny*nz grid (nx*ny*nz < 2^32), from the sort to the reduced tree.
// Peak memory is 23 bytes per vertex in the sweeps: order, union-find, extremum, the join tree's
// down and the split tree's up pointers (4 bytes each), and three byte arrays (union rank,
// up-degree, down-degree); the combine reuses the union-find as its queue and the extremum as the
// parent. The sort's scratch (up to 16 bytes per vertex for 32-bit keys, 24 for 64-bit ones)
// becomes the sweeps' 32-bit arrays where the types allow, and the rest is given back before the
// sweeps allocate. Every array a loop writes in random order is first written once in order (a
// sliced "touch" pass), so its page faults are spread over the steps instead of landing thousands
// at a time in one; spent arrays go back through `spent` a slice at a time.
template <class W> class core {
public:
  using K = key_type<W>;

  // [kmin, kmin + krange] spans the field's keys; krange > 0 (a constant field never gets here).
  core(const W *f, const std::array<uint64, 3> &dims, double vmin, double vmax, bool quantized,
       K kmin, K krange, release_queue &spent)
      : f_(f), g_(u32(dims[0]), u32(dims[1]), u32(dims[2])), n_(g_.n), quantized_(quantized),
        kmin_(kmin), spent_(spent) {
    out_.dims = dims;
    out_.value_min = vmin;
    out_.value_max = vmax;
    // Digits for an LSD radix sort on (key - kmin): one counting-sort pass when the keys span at
    // most 2^16 values (8- and 16-bit data, quantized data, narrow integer ranges), otherwise
    // passes of at most 11 bits (3 for 32-bit keys, 6 for 64-bit ones).
    const unsigned bits = unsigned(std::bit_width(krange));
    if (bits <= 16) {
      passes_ = 1;
      bins_ = u32(krange) + 1;
      mask_ = K(~K(0));
    } else {
      passes_ = (bits + 10) / 11;
      width_ = (bits + passes_ - 1) / passes_;
      bins_ = u32(1) << width_;
      mask_ = K(bins_ - 1);
    }
    hist_ = uninit<u32>(std::size_t(passes_) * bins_);
    total_ = 2 * uint64(passes_) * bins_ + (1 + uint64(passes_)) * n_;
  }

  bool run(budget &b) {
    for (;;) {
      switch (stage_) {
      case stage::clear:
        if (!clear(b))
          return false;
        next(stage::histogram);
        break;
      case stage::histogram:
        if (!histogram(b))
          return false;
        next(stage::prefix);
        break;
      case stage::prefix:
        if (!prefix(b))
          return false;
        next(stage::pass_begin);
        break;
      case stage::pass_begin:
        next(begin_pass() ? stage::touch_pass : stage::scatter);
        break;
      case stage::touch_pass:
        if (!touch_pass(b))
          return false;
        next(stage::scatter);
        break;
      case stage::scatter:
        if (!scatter_pass(b))
          return false;
        if (exec_at_ < nexec_) {
          next(stage::pass_begin);
          break;
        }
        finish_sort();
        next(stage::give_back);
        break;
      case stage::give_back:
        if (!spent_.run(b))
          return false;
        allocate_sweeps();
        next(stage::touch_sweep, 2 * uint64(n_));
        break;
      case stage::touch_sweep:
        if (!touch_sweep(b))
          return false;
        next(stage::join);
        break;
      case stage::join:
        if (!join_sweep(b))
          return false;
        next(stage::split, n_);
        break;
      case stage::split:
        if (!split_sweep(b))
          return false;
        next(stage::flags, 4 * uint64(n_));
        break;
      case stage::flags:
        if (!flags(b))
          return false;
        next(stage::combine);
        break;
      case stage::combine:
        if (!combine(b))
          return false;
        finish_combine();
        next(stage::nodes);
        break;
      case stage::nodes:
        if (!nodes(b))
          return false;
        out_.arcs.reserve(ncritical_ - 1);
        next(stage::arcs);
        break;
      case stage::arcs:
        if (!arcs(b))
          return false;
        finish_arcs();
        next(stage::give_back_all);
        break;
      case stage::give_back_all:
        if (!spent_.run(b))
          return false;
        next(stage::done);
        break;
      case stage::done:
        return true;
      }
    }
  }

  ct_phase phase() const {
    switch (stage_) {
    case stage::clear:
    case stage::histogram:
    case stage::prefix:
    case stage::pass_begin:
    case stage::touch_pass:
    case stage::scatter:
    case stage::give_back:
      return ct_phase::sorting;
    case stage::touch_sweep:
    case stage::join:
      return ct_phase::join;
    case stage::split:
      return ct_phase::split;
    case stage::done:
      return ct_phase::done;
    default:
      return ct_phase::merging;
    }
  }
  double fraction() const { return ratio(done_, total_); }
  contour_tree take() { return std::move(out_); }

private:
  enum class stage : u8 {
    clear,       // sorting: zero the digit histograms
    histogram,   // sorting: count every pass's digits in one read of the field
    prefix,      // sorting: exclusive prefix sums, and which passes have one digit only
    pass_begin,  // sorting: the next pass's buffers, reused or new (O(1))
    touch_pass,  // sorting: write the new ones once, in order
    scatter,     // sorting: one stable pass per useful digit, least significant first
    give_back,   // sorting: return the scratch the sweeps cannot use, before they allocate
    touch_sweep, // join tree: write the sweeps' new arrays once, in order
    join,
    split,
    flags,         // merging: node kinds, the leaf queue, the parent pointers
    combine,       // merging: peel leaves (Carr, Snoeyink & Axen)
    nodes,         // merging: the critical vertices in order
    arcs,          // merging: an arc from each critical vertex to the next one toward the root
    give_back_all, // merging: return every array
    done
  };
  static constexpr u8 kRemoved = 0x80, kKindMask = 0x7f;

  // A new stage; `total` > 0 starts a new progress phase of that many units.
  void next(stage s, uint64 total = 0) {
    stage_ = s;
    cursor_ = 0;
    if (total > 0) {
      done_ = 0;
      total_ = total;
    }
  }

  bool critical(u32 v) const { return (ufr_[v] & kKindMask) != 0; }
  double value_of(W v) const {
    if constexpr (std::is_same_v<W, u8>) {
      if (quantized_) {
        const double vmin = out_.value_min, vmax = out_.value_max, range = vmax - vmin;
        return v == 255 ? vmax : vmin + double(v) * range / 255.0;
      }
    }
    return double(v);
  }

  // ---- sorting ----

  bool clear(budget &b) {
    const u32 size = passes_ * bins_;
    const u32 stop = slice_end(cursor_, size, b);
    std::fill(hist_.get() + cursor_, hist_.get() + stop, 0u);
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == size;
  }

  bool histogram(budget &b) {
    const u32 stop = slice_end(cursor_, n_, b);
    const W *const f = f_;
    u32 *const h = hist_.get();
    const K kmin = kmin_;
    if (passes_ == 1) {
      for (u32 v = cursor_; v < stop; ++v)
        ++h[u32(K(sort_key(f[v]) - kmin))];
    } else {
      const K mask = mask_;
      const unsigned width = width_, passes = passes_, bins = bins_;
      for (u32 v = cursor_; v < stop; ++v) {
        const K k = K(sort_key(f[v]) - kmin);
        for (unsigned d = 0; d < passes; ++d)
          ++h[d * bins + u32((k >> (d * width)) & mask)];
      }
    }
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == n_;
  }

  bool prefix(budget &b) {
    const u32 size = passes_ * bins_;
    const u32 stop = slice_end(cursor_, size, b);
    u32 *const h = hist_.get();
    for (u32 c = cursor_; c < stop; ++c) {
      if (c % bins_ == 0)
        sum_ = 0;
      const u32 count = h[c];
      if (count == n_)
        trivial_[c / bins_] = true; // every key has this digit: the pass would not move anything
      h[c] = sum_;
      sum_ += count;
    }
    done_ += stop - cursor_;
    cursor_ = stop;
    if (cursor_ < size)
      return false;
    for (u32 d = 0; d < passes_; ++d)
      if (!trivial_[d])
        exec_[nexec_++] = d;
      else
        done_ += n_;
    if (nexec_ == 0)
      internal_error("every key is the same in a non-constant field");
    return true;
  }

  // Pass j reads slot (j - 1) & 1 (the field itself for j == 0) and writes slot j & 1: keys and
  // indices, or only indices on the last pass. True when a buffer is new and needs its touch.
  bool begin_pass() {
    const u32 j = exec_at_;
    const bool last = j + 1 == nexec_;
    const int dst = int(j & 1);
    touch_idx_ = !idx_[dst];
    if (touch_idx_)
      idx_[dst] = work_array<u32>(n_);
    touch_key_ = !last && !key_[dst];
    if (touch_key_)
      key_[dst] = work_array<K>(n_);
    return touch_idx_ || touch_key_;
  }

  bool touch_pass(budget &b) {
    const u32 stop = slice_end(cursor_, n_, b);
    const int dst = int(exec_at_ & 1);
    if (touch_idx_)
      std::fill(idx_[dst].get() + cursor_, idx_[dst].get() + stop, 0u);
    if (touch_key_)
      std::fill(key_[dst].get() + cursor_, key_[dst].get() + stop, K(0));
    cursor_ = stop;
    return cursor_ == n_;
  }

  // The current pass (exec_[exec_at_]); true once it is complete.
  bool scatter_pass(budget &b) {
    const u32 j = exec_at_, d = exec_[j];
    const bool last = j + 1 == nexec_;
    const int dst = int(j & 1), src = 1 - dst;
    const u32 stop = slice_end(cursor_, n_, b);
    u32 *const off = hist_.get() + std::size_t(d) * bins_;
    const unsigned shift = d * width_;
    const K *skey = key_[src].get();
    const u32 *sidx = idx_[src].get();
    K *dkey = key_[dst].get();
    u32 *didx = idx_[dst].get();
    if (j == 0) {
      if (last)
        scatter<true, true>(f_, kmin_, skey, sidx, dkey, didx, off, shift, mask_, cursor_, stop);
      else
        scatter<true, false>(f_, kmin_, skey, sidx, dkey, didx, off, shift, mask_, cursor_, stop);
    } else {
      if (last)
        scatter<false, true>(f_, kmin_, skey, sidx, dkey, didx, off, shift, mask_, cursor_, stop);
      else
        scatter<false, false>(f_, kmin_, skey, sidx, dkey, didx, off, shift, mask_, cursor_, stop);
    }
    done_ += stop - cursor_;
    cursor_ = stop;
    if (cursor_ < n_)
      return false;
    ++exec_at_;
    return true;
  }

  // The last pass's indices are the order; the other index buffer (and 32-bit key buffers) become
  // the sweeps' 32-bit arrays; 64-bit key buffers go back.
  void finish_sort() {
    const int last = int((nexec_ - 1) & 1);
    order_ = std::move(idx_[last]);
    std::vector<work_array<u32>> spare;
    if (idx_[1 - last])
      spare.push_back(std::move(idx_[1 - last]));
    for (int s = 0; s < 2; ++s) {
      if constexpr (std::is_same_v<K, u32>) {
        if (key_[s])
          spare.push_back(std::move(key_[s]));
      } else {
        spent_.add(key_[s]);
      }
    }
    work_array<u32> *const slots[] = {&uf_, &ext_, &jt_down_, &st_up_};
    for (std::size_t i = 0; i < spare.size() && i < 4; ++i)
      *slots[i] = std::move(spare[i]);
    hist_.reset();
  }

  // The sweep arrays the sort could not lend, new (and touched next).
  void allocate_sweeps() {
    work_array<u32> *const slots[] = {&uf_, &ext_, &jt_down_, &st_up_};
    for (int i = 0; i < 4; ++i)
      if (!*slots[i]) {
        *slots[i] = work_array<u32>(n_);
        fresh_ |= 1u << i;
      }
    ufr_ = work_array<u8>(n_);
    jt_up_ = work_array<u8>(n_);
    st_dn_ = work_array<u8>(n_);
  }

  bool touch_sweep(budget &b) {
    const u32 stop = slice_end(cursor_, n_, b);
    work_array<u32> *const slots[] = {&uf_, &ext_, &jt_down_, &st_up_};
    for (int i = 0; i < 4; ++i)
      if (fresh_ & (1u << i))
        std::fill(slots[i]->get() + cursor_, slots[i]->get() + stop, 0u);
    for (work_array<u8> *a : {&ufr_, &jt_up_, &st_dn_})
      std::fill(a->get() + cursor_, a->get() + stop, u8(0));
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == n_;
  }

  // ---- the sweeps ----

  // Join tree, sweeping down: each swept higher neighbour in another set is a superlevel component
  // that reaches v, whose lowest vertex therefore hangs from v. A vertex's join-tree slots are set
  // when it is swept: before that nothing can point at it.
  bool join_sweep(budget &b) {
    const u32 n = n_, stop = slice_end(cursor_, n, b);
    const grid g = g_;
    const W *const f = f_;
    const u32 *const order = order_.get();
    u32 *const uf = uf_.get(), *const ext = ext_.get(), *const jd = jt_down_.get();
    u8 *const rank = ufr_.get(), *const ju = jt_up_.get();
    u32 nb[14];
    for (u32 s = cursor_; s < stop; ++s) {
      const u32 v = order[n - 1 - s];
      uf[v] = v;
      rank[v] = 0;
      ext[v] = v;
      jd[v] = kNone;
      ju[v] = 0;
      u32 rv = v;
      const int c = g.neighbours(v, nb);
      for (int t = 0; t < c; ++t) {
        const u32 u = nb[t];
        if (!order_less(f, v, u))
          continue;
        const u32 ru = find_root(uf, u);
        if (ru == rv)
          continue;
        jd[ext[ru]] = v;
        ++ju[v];
        rv = unite(uf, rank, ext, ru, rv, v);
      }
    }
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == n;
  }

  // Split tree: the mirror image, sweeping up.
  bool split_sweep(budget &b) {
    const u32 n = n_, stop = slice_end(cursor_, n, b);
    const grid g = g_;
    const W *const f = f_;
    const u32 *const order = order_.get();
    u32 *const uf = uf_.get(), *const ext = ext_.get(), *const su = st_up_.get();
    u8 *const rank = ufr_.get(), *const sd = st_dn_.get();
    u32 nb[14];
    for (u32 s = cursor_; s < stop; ++s) {
      const u32 v = order[s];
      uf[v] = v;
      rank[v] = 0;
      ext[v] = v;
      su[v] = kNone;
      sd[v] = 0;
      u32 rv = v;
      const int c = g.neighbours(v, nb);
      for (int t = 0; t < c; ++t) {
        const u32 u = nb[t];
        if (!order_less(f, u, v))
          continue;
        const u32 ru = find_root(uf, u);
        if (ru == rv)
          continue;
        su[ext[ru]] = v;
        ++sd[v];
        rv = unite(uf, rank, ext, ru, rv, v);
      }
    }
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == n;
  }

  // ---- merging ----

  // On a box the contour tree's up-degree is the join tree's and its down-degree the split
  // tree's, so the node kinds are known now, before the combine consumes the degrees. The flag
  // (in the spent union rank) holds 1 + kind for a critical vertex (0 for a regular one) and the
  // removed bit; the queue (in the union-find) starts with every leaf in index order.
  bool flags(budget &b) {
    const u32 stop = slice_end(cursor_, n_, b);
    const u8 *const ju = jt_up_.get(), *const sd = st_dn_.get();
    u8 *const flag = ufr_.get();
    u32 *const parent = ext_.get(), *const queue = uf_.get();
    u32 ncritical = ncritical_, tail = tail_;
    for (u32 v = cursor_; v < stop; ++v) {
      const bool regular = ju[v] == 1 && sd[v] == 1;
      flag[v] = regular ? 0 : u8(1 + int(kind_of(ju[v], sd[v])));
      ncritical += regular ? 0 : 1;
      parent[v] = kNone;
      if (ju[v] + sd[v] == 1)
        queue[tail++] = v;
    }
    ncritical_ = ncritical;
    tail_ = tail;
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == n_;
  }

  // Carr, Snoeyink & Axen: peel a vertex only when it is an upper leaf of the join tree with one
  // split-tree child, or a lower leaf of the split tree with one join-tree parent (up + down == 1).
  // VolumeRover re-queued a vertex once it was a leaf in the tree it came from, ignoring the
  // other tree, which built wrong (even disconnected) trees on 2D data and thin slabs. Splicing
  // the peeled vertex out of the other tree is lazy: the next lookup skips removed vertices and
  // compresses the path. One lookup can walk a long chain, so the walk and the compression are
  // resumable hop by hop (one unit per hop, one per peeled vertex).
  bool combine(budget &b) {
    u8 *const flag = ufr_.get(), *const ju = jt_up_.get(), *const sd = st_dn_.get();
    u32 *const queue = uf_.get(), *const parent = ext_.get();
    for (;;) {
      if (cm_v_ == kNone) {
        if (tail_ - head_ <= 1)
          return true;
        if (!b.take_one())
          return false;
        cm_v_ = queue[head_++];
        ++done_;
        cm_join_ = ju[cm_v_] == 0;
        cm_w_ = (cm_join_ ? jt_down_ : st_up_)[cm_v_];
        cm_walking_ = true;
      }
      u32 *const p = cm_join_ ? jt_down_.get() : st_up_.get();
      if (cm_walking_) {
        while (cm_w_ != kNone && (flag[cm_w_] & kRemoved)) {
          if (!b.take_one())
            return false;
          cm_w_ = p[cm_w_];
        }
        cm_walking_ = false;
        cm_x_ = p[cm_v_];
      }
      while (cm_x_ != cm_w_) {
        if (!b.take_one())
          return false;
        const u32 y = p[cm_x_];
        p[cm_x_] = cm_w_;
        cm_x_ = y;
      }
      const u32 v = cm_v_, w = cm_w_;
      p[v] = w;
      cm_v_ = kNone;
      if (w == kNone)
        internal_error(cm_join_ ? "an upper leaf without a join-tree parent"
                                : "a lower leaf without a split-tree parent");
      if (cm_join_)
        --ju[w];
      else
        --sd[w];
      flag[v] |= kRemoved;
      parent[v] = w;
      if (ju[w] + sd[w] == 1)
        queue[tail_++] = w;
    }
  }

  void finish_combine() {
    if (tail_ - head_ != 1 || head_ != n_ - 1)
      internal_error("the combine did not reduce the grid to one vertex");
    root_ = uf_[head_];
    spent_.add(jt_down_);
    spent_.add(st_up_);
    spent_.add(jt_up_);
    spent_.add(st_dn_);
    out_.nodes.reserve(ncritical_);
  }

  // Reduce: nodes in (value, index) order, so an arc's hi is simply its larger node index. Every
  // critical vertex but the root owns the arc toward the root, past the regular vertices on it;
  // a regular root sits inside one arc, reached from both sides. node_of reuses the spent queue.
  bool nodes(budget &b) {
    const u32 stop = slice_end(cursor_, n_, b);
    const u32 *const order = order_.get();
    u32 *const node_of = uf_.get();
    for (u32 r = cursor_; r < stop; ++r) {
      const u32 v = order[r];
      if (!critical(v))
        continue;
      node_of[v] = u32(out_.nodes.size());
      contour_tree::node nd;
      nd.voxel = v;
      nd.value = value_of(f_[v]);
      nd.kind = contour_tree::node_kind((ufr_[v] & kKindMask) - 1);
      out_.nodes.push_back(nd);
    }
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == n_;
  }

  void add_arc(u32 a, u32 b) {
    contour_tree::arc arc;
    arc.hi = std::max(a, b);
    arc.lo = std::min(a, b);
    out_.arcs.push_back(arc);
  }

  // The walk from a critical vertex to the next one is resumable hop by hop (one unit per hop and
  // one per vertex scanned): one arc may pass most of the grid.
  bool arcs(budget &b) {
    const u32 *const order = order_.get(), *const parent = ext_.get(), *const node_of = uf_.get();
    for (;;) {
      if (rd_v_ == kNone) {
        if (cursor_ == n_)
          return true;
        if (!b.take_one())
          return false;
        const u32 v = order[cursor_++];
        ++done_;
        if (!critical(v) || v == root_)
          continue;
        rd_v_ = v;
        rd_x_ = parent[v];
        if (rd_x_ == kNone)
          internal_error("a broken parent chain");
      }
      while (rd_x_ != root_ && !critical(rd_x_)) {
        if (!b.take_one())
          return false;
        rd_x_ = parent[rd_x_];
        if (rd_x_ == kNone)
          internal_error("a broken parent chain");
      }
      const u32 v = rd_v_, x = rd_x_;
      rd_v_ = kNone;
      if (x == root_ && !critical(root_)) {
        if (ndangling_ < 2)
          dangling_[ndangling_] = v;
        ++ndangling_;
        continue;
      }
      add_arc(node_of[v], node_of[x]);
    }
  }

  void finish_arcs() {
    const u32 *const node_of = uf_.get();
    if (ndangling_ == 2)
      add_arc(node_of[dangling_[0]], node_of[dangling_[1]]);
    else if (ndangling_ != 0)
      internal_error("a regular root not on exactly one arc");
    if (out_.arcs.size() + 1 != out_.nodes.size())
      internal_error("the reduced tree is not a tree");
    spent_.add(order_);
    spent_.add(uf_);
    spent_.add(ext_);
    spent_.add(ufr_);
  }

  const W *f_;
  grid g_;
  u32 n_;
  bool quantized_;
  K kmin_, mask_ = 0;
  u32 passes_ = 1, width_ = 0, bins_ = 1;
  release_queue &spent_;

  stage stage_ = stage::clear;
  u32 cursor_ = 0;
  uint64 done_ = 0, total_ = 1; // progress through the current phase

  // sorting
  std::unique_ptr<u32[]> hist_; // at most 65536 counters
  u32 sum_ = 0;
  bool trivial_[8] = {};
  u32 exec_[8] = {}, nexec_ = 0, exec_at_ = 0;
  work_array<K> key_[2];
  work_array<u32> idx_[2];
  bool touch_idx_ = false, touch_key_ = false;

  // sweeps and merge
  work_array<u32> order_, uf_, ext_, jt_down_, st_up_;
  work_array<u8> ufr_, jt_up_, st_dn_;
  unsigned fresh_ = 0; // which of uf_, ext_, jt_down_, st_up_ are new (bits 0-3)
  u32 ncritical_ = 0, head_ = 0, tail_ = 0, root_ = kNone;
  u32 cm_v_ = kNone, cm_w_ = kNone, cm_x_ = kNone;
  bool cm_join_ = false, cm_walking_ = false;
  u32 rd_v_ = kNone, rd_x_ = kNone, dangling_[2] = {kNone, kNone};
  int ndangling_ = 0;

  contour_tree out_;
};

// ---- input preparation ----------------------------------------------------------------------

template <class T> T from_double(double v) {
  if constexpr (std::is_integral_v<T>) {
    // Truncate like voxels::setValueRaw, but clamp first: a 64-bit maximum rounds up to 2^64 as a
    // double, and converting that back is undefined.
    if (v <= double(std::numeric_limits<T>::lowest()))
      return std::numeric_limits<T>::lowest();
    if (v >= double(std::numeric_limits<T>::max()))
      return std::numeric_limits<T>::max();
    return static_cast<T>(v);
  } else {
    return static_cast<T>(v);
  }
}

bool checked_product(uint64 a, uint64 b, uint64 c, uint64 &out) {
  const uint64 m = std::numeric_limits<uint64>::max();
  if (a != 0 && b > m / a)
    return false;
  const uint64 ab = a * b;
  if (ab != 0 && c > m / ab)
    return false;
  out = ab * c;
  return true;
}

// Trilinear resampling taps, sampling the source at i*(old-1)/(new-1) per axis as voxels::resize
// does. Not voxels::resize itself: it computes 0/0 on a size-1 axis (any 2D image), it makes NaN of
// infinities (see blend), and the raw float overload has no cvc::app to build voxels with.
struct tap {
  std::size_t i0, i1;
  double w;
};

tap make_tap(uint64 from, uint64 count, uint64 i) {
  const double s = count > 1 ? double(from - 1) / double(count - 1) : 0.0;
  const double x = double(i) * s;
  const uint64 i0 = std::min<uint64>(uint64(x), from - 1);
  return {std::size_t(i0), std::size_t(std::min<uint64>(i0 + 1, from - 1)), x - double(i0)};
}

// One interpolation step between taps p and q, w in [0, 1): p + (q - p) * w as voxels::resize
// computes it wherever that is finite, extended so that it never makes a NaN of non-NaN values
// (the NaN scan has already passed, and a NaN key would break the sort and the sweeps): inf - inf
// and inf * 0 are NaN, so a tap of weight 0 is left out, an infinity outweighs a finite value,
// +inf against -inf takes the nearer tap (p at the midpoint), and finite doubles too far apart to
// subtract (opposite signs beyond DBL_MAX / 2) blend as p (1 - w) + q w, clamped into [p, q].
double blend(double p, double q, double w) {
  if (std::isfinite(p) && std::isfinite(q)) {
    const double r = p + (q - p) * w;
    if (std::isfinite(r))
      return r;
    const double s = p * (1.0 - w) + q * w;
    return std::min(std::max(s, std::min(p, q)), std::max(p, q));
  }
  if (w == 0.0 || p == q)
    return p;
  if (std::isfinite(p))
    return q;
  if (std::isfinite(q))
    return p;
  return w > 0.5 ? q : p;
}

// Checks the input, scans it for NaN, resamples, finds the range and quantizes -- each a sliced
// pass, one unit per voxel (per tap for the resampling taps) -- then hands the working array to a
// core.
template <class T> class prep final : public ct_task {
public:
  // The O(1) input checks; they throw on the first step, where the builder makes this.
  prep(const T *in, uint64 nx, uint64 ny, uint64 nz, uint64 max_dim, bool quantize_u8)
      : in_(in), src_dims_{{nx, ny, nz}}, dims_{{nx, ny, nz}} {
    if (!in || nx == 0 || ny == 0 || nz == 0)
      throw contour_tree_error("empty volume");
    if (!checked_product(nx, ny, nz, in_total_) ||
        in_total_ > std::numeric_limits<std::size_t>::max())
      throw contour_tree_error("the volume is too large to address");
    resample_ = max_dim > 0 && (nx > max_dim || ny > max_dim || nz > max_dim);
    if (resample_)
      for (uint64 &d : dims_)
        d = std::min(d, max_dim);
    uint64 n64 = 0;
    if (!checked_product(dims_[0], dims_[1], dims_[2], n64) || n64 >= kMaxVoxels)
      throw contour_tree_error("2^32 or more voxels; set contour_tree_options::max_dim");
    n_ = u32(n64);
    if constexpr (!std::is_same_v<T, unsigned char>)
      quantize_ = quantize_u8;
    if constexpr (std::is_floating_point_v<T>)
      total_ += in_total_;
    if (resample_)
      total_ += dims_[0] + dims_[1] + dims_[2] + n_;
    total_ += n_ + (quantize_ ? n_ : 0);
    data_ = in_;
    stage_ = std::is_floating_point_v<T> ? stage::nan : resample_ ? stage::taps : stage::minmax;
  }

  bool run(budget &b) override {
    for (;;) {
      switch (stage_) {
      case stage::nan:
        if (!scan_nan(b))
          return false;
        next(resample_ ? stage::taps : stage::minmax);
        break;
      case stage::taps:
        if (!taps(b))
          return false;
        resampled_ = work_array<T>(n_);
        next(stage::resample);
        break;
      case stage::resample:
        if (!resample(b))
          return false;
        data_ = resampled_.get();
        std::vector<tap>().swap(tx_);
        std::vector<tap>().swap(ty_);
        std::vector<tap>().swap(tz_);
        next(stage::minmax);
        break;
      case stage::minmax:
        if (!minmax(b))
          return false;
        if (!(lo_ < hi_)) {
          constant();
          spent_.add(resampled_);
          next(stage::release);
          break;
        }
        if (quantize_) {
          if (!std::isfinite(vmax_ - vmin_))
            throw contour_tree_error("quantize_u8 needs a finite value range");
          q_ = work_array<u8>(n_);
          next(stage::quantize);
          break;
        }
        // An unsigned char volume is used raw in quantize_u8 mode too, as VolumeRover2 did.
        raw_ = std::make_unique<core<T>>(data_, dims_, vmin_, vmax_, false, sort_key(lo_),
                                         key_type<T>(sort_key(hi_) - sort_key(lo_)), spent_);
        next(stage::build);
        break;
      case stage::quantize:
        if (!quantize(b))
          return false;
        spent_.add(resampled_); // given back before the sweeps allocate
        data_ = nullptr;
        quantized_ =
            std::make_unique<core<u8>>(q_.get(), dims_, vmin_, vmax_, true, 0u, 255u, spent_);
        next(stage::build);
        break;
      case stage::build:
        if (!(raw_ ? raw_->run(b) : quantized_->run(b)))
          return false;
        tree_ = raw_ ? raw_->take() : quantized_->take();
        raw_.reset(); // its arrays are given back already
        quantized_.reset();
        built_ = true;
        spent_.add(resampled_);
        spent_.add(q_);
        next(stage::release);
        break;
      case stage::release:
        if (!spent_.run(b))
          return false;
        next(stage::done);
        return true;
      case stage::done:
        return true;
      }
    }
  }

  ct_phase phase() const override {
    if (stage_ == stage::done)
      return ct_phase::done;
    if (stage_ == stage::build)
      return raw_ ? raw_->phase() : quantized_->phase();
    if (stage_ == stage::release && built_)
      return ct_phase::merging;
    return ct_phase::preparing;
  }
  double fraction() const override {
    if (stage_ == stage::build)
      return raw_ ? raw_->fraction() : quantized_->fraction();
    if (stage_ == stage::release && built_)
      return 1.0;
    return ratio(done_, total_);
  }
  bool uses_input() const override {
    switch (stage_) {
    case stage::nan:
    case stage::taps:
    case stage::resample:
      return true;
    case stage::minmax:
    case stage::quantize:
      return !resample_;
    case stage::build:
      return !resample_ && raw_ != nullptr;
    case stage::release:
    case stage::done:
      break;
    }
    return false;
  }
  contour_tree take() override { return std::move(tree_); }

private:
  enum class stage : u8 { nan, taps, resample, minmax, quantize, build, release, done };

  void next(stage s) {
    stage_ = s;
    cursor_ = 0;
  }

  bool scan_nan(budget &b) {
    if constexpr (std::is_floating_point_v<T>) {
      const std::size_t total = std::size_t(in_total_), stop = slice_end(cursor_, total, b);
      const T *const in = in_;
      for (std::size_t i = cursor_; i < stop; ++i)
        if (std::isnan(in[i]))
          throw contour_tree_error("NaN value at voxel " + std::to_string(i));
      done_ += stop - cursor_;
      cursor_ = stop;
      return cursor_ == total;
    } else {
      return true;
    }
  }

  // Every axis's taps, x then y then z, one unit each.
  bool taps(budget &b) {
    const std::size_t total = std::size_t(dims_[0] + dims_[1] + dims_[2]);
    if (cursor_ == 0) {
      tx_.reserve(std::size_t(dims_[0]));
      ty_.reserve(std::size_t(dims_[1]));
      tz_.reserve(std::size_t(dims_[2]));
    }
    const std::size_t stop = slice_end(cursor_, total, b);
    for (std::size_t c = cursor_; c < stop; ++c) {
      std::size_t i = c;
      int a = 0;
      while (i >= dims_[a]) {
        i -= std::size_t(dims_[a]);
        ++a;
      }
      std::vector<tap> &t = a == 0 ? tx_ : a == 1 ? ty_ : tz_;
      t.push_back(make_tap(src_dims_[a], dims_[a], i));
    }
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == total;
  }

  bool resample(budget &b) {
    const std::size_t stop = slice_end(cursor_, std::size_t(n_), b);
    const T *const src = in_;
    T *const dst = resampled_.get();
    const std::size_t sx = std::size_t(src_dims_[0]),
                      sxy = std::size_t(src_dims_[0] * src_dims_[1]);
    auto at = [&](std::size_t i, std::size_t j, std::size_t kk) {
      return double(src[i + j * sx + kk * sxy]);
    };
    for (std::size_t o = cursor_; o < stop; ++o) {
      const tap &a = tx_[ri_], &bb = ty_[rj_], &c = tz_[rk_];
      const double v0 = blend(blend(at(a.i0, bb.i0, c.i0), at(a.i1, bb.i0, c.i0), a.w),
                              blend(at(a.i0, bb.i1, c.i0), at(a.i1, bb.i1, c.i0), a.w), bb.w);
      const double v1 = blend(blend(at(a.i0, bb.i0, c.i1), at(a.i1, bb.i0, c.i1), a.w),
                              blend(at(a.i0, bb.i1, c.i1), at(a.i1, bb.i1, c.i1), a.w), bb.w);
      dst[o] = from_double<T>(blend(v0, v1, c.w));
      if (++ri_ == tx_.size()) {
        ri_ = 0;
        if (++rj_ == ty_.size()) {
          rj_ = 0;
          ++rk_;
        }
      }
    }
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == n_;
  }

  bool minmax(budget &b) {
    const T *const data = data_;
    if (cursor_ == 0)
      lo_ = hi_ = data[0];
    const std::size_t stop = slice_end(cursor_, std::size_t(n_), b);
    T lo = lo_, hi = hi_;
    for (std::size_t v = cursor_; v < stop; ++v) {
      if (data[v] < lo)
        lo = data[v];
      if (hi < data[v])
        hi = data[v];
    }
    lo_ = lo;
    hi_ = hi;
    done_ += stop - cursor_;
    cursor_ = stop;
    if (cursor_ < n_)
      return false;
    vmin_ = double(lo_);
    vmax_ = double(hi_);
    return true;
  }

  // Simulation of simplicity would turn a constant field into an index ramp with a spurious arc;
  // there is no topology to report.
  void constant() {
    tree_.dims = dims_;
    tree_.value_min = vmin_;
    tree_.value_max = vmax_;
    tree_.nodes.emplace_back();
    tree_.nodes.back().value = vmin_;
  }

  // VolumeRover2 parity: q = trunc(255*(v-min)/(max-min)).
  bool quantize(budget &b) {
    const std::size_t stop = slice_end(cursor_, std::size_t(n_), b);
    const T *const data = data_;
    u8 *const q = q_.get();
    const double vmin = vmin_, range = vmax_ - vmin_;
    for (std::size_t v = cursor_; v < stop; ++v) {
      const double x = (double(data[v]) - vmin) / range * 255.0;
      // VolumeRover2 stored the mapped value in the volume's own type before the uchar cast, so a
      // float volume rounds to float first (86.9999999 becomes bin 87); integer types truncate
      // either way. volatile because MSVC folds uint8(float(x)) into uint8(x) even under
      // /fp:strict.
      if constexpr (std::is_floating_point_v<T>) {
        const volatile T stored = static_cast<T>(x);
        q[v] = u8(stored);
      } else {
        q[v] = u8(x);
      }
    }
    done_ += stop - cursor_;
    cursor_ = stop;
    return cursor_ == n_;
  }

  const T *in_;
  std::array<uint64, 3> src_dims_, dims_;
  uint64 in_total_ = 0;
  u32 n_ = 0;
  bool resample_ = false, quantize_ = false;
  stage stage_ = stage::minmax;
  std::size_t cursor_ = 0;
  uint64 done_ = 0, total_ = 0;

  std::vector<tap> tx_, ty_, tz_;
  std::size_t ri_ = 0, rj_ = 0, rk_ = 0;
  work_array<T> resampled_;
  const T *data_ = nullptr; // the working array: the input or the resampled copy
  T lo_{}, hi_{};
  double vmin_ = 0.0, vmax_ = 0.0;
  work_array<u8> q_;
  release_queue spent_; // before the cores, which hold a reference to it
  bool built_ = false;
  std::unique_ptr<core<T>> raw_;
  std::unique_ptr<core<u8>> quantized_;
  contour_tree tree_;
};

template <class T>
std::unique_ptr<ct_task> make_prep(const void *p, uint64 nx, uint64 ny, uint64 nz,
                                   const contour_tree_options &opt) {
  return std::make_unique<prep<T>>(static_cast<const T *>(p), nx, ny, nz, opt.max_dim,
                                   opt.quantize_u8);
}

// ---- simplification -------------------------------------------------------------------------

// Leaf pruning with lazy priority-queue entries. An arc only grows when its saddle is suppressed
// (the leaf then reaches the next node), so a stale entry carries a smaller span than the leaf's
// current one and is recognised on pop; every change that can make a leaf prunable pushes it anew.
// The heap pops in (span, leaf) order, a total order on distinct entries (a span is never NaN: a
// NaN node value is refused as malformed, and equal values, two infinities of one sign among them,
// are 0 apart), so the prunes do not depend on how the heap is built or laid out. Each node's arc
// list holds exactly its live arcs (every arc knows its slot in both endpoint lists, so an arc
// leaves or moves in O(1)), which makes every prune O(log heap) -- one budget unit -- however many
// arcs a saddle once had.
struct heap_entry {
  double span;
  u32 leaf;
  bool operator>(const heap_entry &o) const {
    return span > o.span || (span == o.span && leaf > o.leaf);
  }
};

// A binary min-heap in fixed blocks of 4096 entries (64 KiB): growing allocates one block (and now
// and then moves the block table, 1/4096 of the size) where a vector would copy every entry, so no
// push costs more than O(log size) plus that; the blocks go back through a release queue.
class block_heap {
public:
  bool empty() const { return size_ == 0; }
  std::size_t size() const { return size_; }
  const heap_entry &top() const { return at(0); }
  void push(const heap_entry &e) {
    if ((size_ >> kShift) == blocks_.size())
      blocks_.emplace_back(kBlock);
    std::size_t i = size_++;
    while (i > 0) {
      const std::size_t p = (i - 1) / 2;
      if (!(at(p) > e))
        break;
      at(i) = at(p);
      i = p;
    }
    at(i) = e;
  }
  void pop() {
    if (--size_ == 0)
      return;
    const heap_entry last = at(size_);
    std::size_t i = 0;
    for (;;) {
      std::size_t c = 2 * i + 1;
      if (c >= size_)
        break;
      if (c + 1 < size_ && at(c) > at(c + 1))
        ++c;
      if (!(last > at(c)))
        break;
      at(i) = at(c);
      i = c;
    }
    at(i) = last;
  }
  void clear() { size_ = 0; }
  void give_back(release_queue &q) {
    for (work_array<heap_entry> &block : blocks_)
      q.add(block);
    blocks_.clear();
    size_ = 0;
  }

private:
  static constexpr unsigned kShift = 12;
  static constexpr std::size_t kBlock = std::size_t(1) << kShift;
  heap_entry &at(std::size_t i) { return blocks_[i >> kShift][i & (kBlock - 1)]; }
  const heap_entry &at(std::size_t i) const { return blocks_[i >> kShift][i & (kBlock - 1)]; }

  std::vector<work_array<heap_entry>> blocks_;
  std::size_t size_ = 0;
};

// The span between two node values: |a - b|, but 0 between equal ones, so that a plateau at +inf
// is 0 high as a finite one is (inf - inf is NaN). Between an infinity and anything else, or
// finite values too far apart to subtract, it is +inf.
double spread(double a, double b) { return a == b ? 0.0 : std::abs(a - b); }

} // namespace

struct contour_tree_simplifier::impl {
  enum class stage : u8 {
    range,    // preparing: the value range, when the tree's own is empty
    clear,    // preparing: degrees and list heads to zero
    count,    // preparing: check and count every arc
    lists,    // preparing: list offsets
    fill,     // preparing: every arc into both endpoint lists
    suppress, // preparing: splice out regular nodes
    heap,     // preparing: every prunable leaf
    prune,
    nodes,   // rebuilding
    arcs,    // rebuilding
    release, // rebuilding: give the working arrays back
    done
  };

  // The input; its arcs are rewritten in place as saddles are suppressed. Kept until the
  // simplifier is destroyed: freeing a big tree inside a step would be an O(nodes) pause.
  contour_tree t;
  // By persistence, only leaves whose span is below `limit` go (`limited`); by node count there is
  // no such bound, so a leaf of infinite span goes too, after every finite one.
  double limit = std::numeric_limits<double>::infinity(), min_persistence = 0.0;
  bool limited = false;
  std::size_t max_nodes = 0;
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  stage st = stage::done;
  u32 cursor = 0, m = 0, na = 0;
  work_array<u32> up, dn, start, end, adj, pos;
  work_array<u8> arc_alive, alive;
  release_queue spent;
  std::size_t alive_count = 0, target = 2;
  block_heap heap;
  bool pushing = false;
  contour_tree out;
  std::exception_ptr error;
  double progress = 0.0;
  uint64 done_units = 0, total_units = 1;
  bool taken = false;
  std::size_t paid = 0; // units paid toward the next pop

  void begin(contour_tree &&tree) {
    t = std::move(tree);
    m = u32(std::min<std::size_t>(t.nodes.size(), kNone));
    na = u32(std::min<std::size_t>(t.arcs.size(), kNone));
  }
  void no_work() {
    out = std::move(t);
    st = stage::done;
    progress = 1.0;
  }
  void next(stage s) {
    st = s;
    cursor = 0;
  }

  bool step(budget &b);
  const char *phase() const;
  double fraction() const;

  bool prunable(u32 x, u32 &e, u32 &y, double &span) const;
  void push(u32 x);
  void detach(u32 e, u32 y);
  void prune(u32 x, u32 e, u32 y);
  void suppress(u32 y);
};

namespace {
template <class F> bool sliced(u32 &cursor, u32 end, budget &b, uint64 &done, F &&body) {
  const u32 stop = slice_end(cursor, end, b);
  for (u32 i = cursor; i < stop; ++i)
    body(i);
  done += stop - cursor;
  cursor = stop;
  return cursor == end;
}
} // namespace

// A leaf may go when its neighbour is not a leaf too (never remove the last arc) and, per Carr,
// Snoeyink & Axen's simplification, when that saddle keeps another arc on the leaf's side: then
// each prune cancels an extremum-saddle pair and the result is still a contour tree of some
// field. A reduced tree with two or more arcs always has such a leaf.
bool contour_tree_simplifier::impl::prunable(u32 x, u32 &e, u32 &y, double &span) const {
  if (!alive[x] || up[x] + dn[x] != 1)
    return false;
  e = adj[start[x]]; // the one live arc
  const bool upper = t.arcs[e].hi == x;
  y = upper ? t.arcs[e].lo : t.arcs[e].hi;
  if (up[y] + dn[y] == 1)
    return false;
  if (upper ? up[y] < 2 : dn[y] < 2)
    return false;
  span = spread(t.nodes[x].value, t.nodes[y].value);
  return true;
}

void contour_tree_simplifier::impl::push(u32 x) {
  u32 e = kNone, y = kNone;
  double span = 0.0;
  if (pushing && prunable(x, e, y, span))
    heap.push({span, x});
}

// Arc e leaves the list of its endpoint y: the list's last arc takes its slot.
void contour_tree_simplifier::impl::detach(u32 e, u32 y) {
  const u32 slot = pos[2 * e + (t.arcs[e].hi == y ? 0 : 1)];
  const u32 last = --end[y];
  const u32 f = adj[last];
  adj[slot] = f;
  if (f != e)
    pos[2 * f + (t.arcs[f].hi == y ? 0 : 1)] = slot;
}

void contour_tree_simplifier::impl::prune(u32 x, u32 e, u32 y) {
  arc_alive[e] = 0;
  if (t.arcs[e].hi == x) {
    --dn[x];
    --up[y];
  } else {
    --up[x];
    --dn[y];
  }
  detach(e, y);
  alive[x] = 0;
  --alive_count;
  if (up[y] == 1 && dn[y] == 1)
    suppress(y);
  else
    push(y);
}

// Splice out a node with one arc up and one down: the up arc takes over the down arc's lower end
// (and its slot in that node's list).
void contour_tree_simplifier::impl::suppress(u32 y) {
  if (end[y] - start[y] != 2)
    internal_error("suppressing a node that is not regular");
  u32 eu = adj[start[y]], ed = adj[start[y] + 1];
  if (t.arcs[eu].lo != y)
    std::swap(eu, ed);
  if (t.arcs[eu].lo != y || t.arcs[ed].hi != y)
    internal_error("suppressing a node that is not regular");
  const u32 a = t.arcs[eu].hi, bnode = t.arcs[ed].lo;
  if (a == bnode)
    throw contour_tree_error("malformed tree: a cycle");
  t.arcs[eu].lo = bnode;
  arc_alive[ed] = 0;
  const u32 slot = pos[2 * ed + 1];
  adj[slot] = eu;
  pos[2 * eu + 1] = slot;
  alive[y] = 0;
  up[y] = dn[y] = 0;
  --alive_count;
  push(a);
  push(bnode);
}

bool contour_tree_simplifier::impl::step(budget &b) {
  for (;;) {
    switch (st) {
    case stage::range:
      if (!sliced(cursor, m, b, done_units, [this](u32 i) {
            lo = std::min(lo, t.nodes[i].value);
            hi = std::max(hi, t.nodes[i].value);
          }))
        return false;
      limit = min_persistence * spread(hi, lo);
      next(stage::clear);
      break;
    case stage::clear:
      if (cursor == 0) {
        if (t.nodes.size() >= kNone || t.arcs.size() >= kNone / 2)
          throw contour_tree_error("tree too large");
        up = work_array<u32>(m);
        dn = work_array<u32>(m);
        start = work_array<u32>(std::size_t(m) + 1);
        end = work_array<u32>(m);
        adj = work_array<u32>(2 * std::size_t(na));
        pos = work_array<u32>(2 * std::size_t(na));
        arc_alive = work_array<u8>(na);
        alive = work_array<u8>(m);
        start[m] = 0;
      }
      if (!sliced(cursor, m, b, done_units, [this](u32 i) {
            if (std::isnan(t.nodes[i].value))
              throw contour_tree_error("malformed tree: a NaN node value");
            up[i] = dn[i] = start[i] = 0;
            alive[i] = 1;
          }))
        return false;
      next(stage::count);
      break;
    case stage::count:
      if (!sliced(cursor, na, b, done_units, [this](u32 e) {
            const contour_tree::arc &a = t.arcs[e];
            if (a.hi >= m || a.lo >= m || a.hi == a.lo)
              throw contour_tree_error("malformed tree: an arc with a bad node index");
            ++dn[a.hi];
            ++up[a.lo];
            ++start[a.hi + 1];
            ++start[a.lo + 1];
            arc_alive[e] = 1;
            adj[2 * e] = adj[2 * e + 1] = 0; // first written in order, so its pages fault evenly
          }))
        return false;
      next(stage::lists);
      break;
    case stage::lists:
      if (!sliced(cursor, m, b, done_units, [this](u32 i) {
            start[i + 1] += start[i];
            end[i] = start[i];
          }))
        return false;
      next(stage::fill);
      break;
    case stage::fill:
      if (!sliced(cursor, na, b, done_units, [this](u32 e) {
            const u32 h = t.arcs[e].hi, l = t.arcs[e].lo;
            pos[2 * e] = end[h];
            adj[end[h]++] = e;
            pos[2 * e + 1] = end[l];
            adj[end[l]++] = e;
          }))
        return false;
      alive_count = m;
      pushing = false; // the heap is built from scratch next
      next(stage::suppress);
      break;
    case stage::suppress:
      if (!sliced(cursor, m, b, done_units, [this](u32 i) {
            if (alive[i] && up[i] == 1 && dn[i] == 1)
              suppress(i);
          }))
        return false;
      heap.clear();
      pushing = true;
      next(stage::heap);
      break;
    case stage::heap:
      if (!sliced(cursor, m, b, done_units, [this](u32 i) { push(i); }))
        return false;
      next(stage::prune);
      break;
    case stage::prune:
      // Prune the least persistent prunable leaf while more than `max_nodes` nodes remain and (by
      // persistence) its span is below `limit`. A pop costs one unit per heap level it can sift
      // through (1 + log2 of the heap's size; it is a cache miss each on a big heap), paid across
      // steps if need be, so a pruning step takes about as long as a sweep step of the same budget.
      while (!heap.empty() && alive_count > max_nodes) {
        const std::size_t cost = 1 + std::size_t(std::bit_width(heap.size()));
        paid = std::min(paid, cost);
        paid += b.take(cost - paid);
        if (paid < cost)
          return false;
        paid = 0;
        const heap_entry top = heap.top();
        heap.pop();
        u32 e = kNone, y = kNone;
        double span = 0.0;
        if (!prunable(top.leaf, e, y, span) || span != top.span)
          continue;
        if (limited && !(span < limit))
          break;
        prune(top.leaf, e, y);
      }
      heap.give_back(spent);
      out.dims = t.dims;
      out.value_min = t.value_min;
      out.value_max = t.value_max;
      out.nodes.reserve(alive_count);
      next(stage::nodes);
      break;
    case stage::nodes: {
      u32 *const remap = start.get(); // the list offsets are spent
      if (!sliced(cursor, m, b, done_units, [this, remap](u32 i) {
            if (!alive[i])
              return;
            remap[i] = u32(out.nodes.size());
            contour_tree::node nd = t.nodes[i];
            if (up[i] + dn[i] > 0)
              nd.kind = kind_of(up[i], dn[i]);
            out.nodes.push_back(nd);
          }))
        return false;
      out.arcs.reserve(alive_count > 0 ? alive_count - 1 : 0);
      next(stage::arcs);
      break;
    }
    case stage::arcs: {
      const u32 *const remap = start.get();
      if (!sliced(cursor, na, b, done_units, [this, remap](u32 e) {
            if (!arc_alive[e])
              return;
            contour_tree::arc a;
            a.hi = remap[t.arcs[e].hi];
            a.lo = remap[t.arcs[e].lo];
            out.arcs.push_back(a);
          }))
        return false;
      for (work_array<u32> *a : {&up, &dn, &start, &end, &adj, &pos})
        spent.add(*a);
      spent.add(arc_alive);
      spent.add(alive);
      next(stage::release);
      break;
    }
    case stage::release:
      if (!spent.run(b))
        return false;
      next(stage::done);
      return true;
    case stage::done:
      return true;
    }
  }
}

const char *contour_tree_simplifier::impl::phase() const {
  if (error)
    return "failed";
  switch (st) {
  case stage::prune:
    return "pruning";
  case stage::nodes:
  case stage::arcs:
  case stage::release:
    return "rebuilding";
  case stage::done:
    return "done";
  default:
    return "preparing";
  }
}

// preparing [0, 0.3), pruning [0.3, 0.9), rebuilding [0.9, 0.99), giving back [0.99, 1).
double contour_tree_simplifier::impl::fraction() const {
  switch (st) {
  case stage::prune: {
    const double removed = double(m) - double(alive_count),
                 most = double(m) - double(std::min<std::size_t>(target, m));
    return 0.3 + 0.6 * (most > 0.0 ? std::min(1.0, removed / most) : 1.0);
  }
  case stage::nodes:
    return 0.9 + 0.09 * ratio(cursor, uint64(m) + na);
  case stage::arcs:
    return 0.9 + 0.09 * ratio(uint64(m) + cursor, uint64(m) + na);
  case stage::release:
    return 0.99;
  case stage::done:
    return 1.0;
  default:
    return 0.3 * std::min(1.0, ratio(done_units, total_units));
  }
}

// ---- layout ---------------------------------------------------------------------------------

namespace {

// cot(theta) for theta = 60..89 degrees, correctly rounded, so the layout never depends on a
// platform's tan(); cot(90) = 0 and cot(180 - theta) = -cot(theta) cover the rest of 60..119.
constexpr double kCot[30] = {
    0.5773502691896257,  0.5543090514527689,  0.5317094316614788,  0.5095254494944288,
    0.48773258856586144, 0.4663076581549986,  0.44522868530853615, 0.42447481620960476,
    0.4040262258351568,  0.3838640350354158,  0.36397023426620234, 0.34432761328966527,
    0.32491969623290634, 0.30573068145866034, 0.2867453857588079,  0.2679491924311227,
    0.24932800284318068, 0.23086819112556312, 0.21255656167002213, 0.19438030913771848,
    0.17632698070846498, 0.1583844403245363,  0.14054083470239145, 0.12278456090290459,
    0.10510423526567646, 0.08748866352592401, 0.06992681194351041, 0.0524077792830412,
    0.03492076949174773, 0.017455064928217585};

double cot_degrees(int theta) {
  if (theta == 90)
    return 0.0;
  return theta < 90 ? kCot[theta - 60] : -kCot[120 - theta];
}

// The level set's components at w: arcs with lo <= w < hi. At the top value nothing spans that
// way, so count the arcs arriving there from below; arcs lying inside a plateau at the top (both
// ends equal, from simulation of simplicity) are pieces of one component and do not count.
template <class Visit> void visit_arcs_at(const contour_tree &t, double w, Visit visit) {
  if (t.arcs.empty() || std::isnan(w))
    return;
  double top = -std::numeric_limits<double>::infinity();
  for (const contour_tree::node &nd : t.nodes)
    top = std::max(top, nd.value);
  const std::size_t m = t.nodes.size();
  for (std::size_t e = 0; e < t.arcs.size(); ++e) {
    const contour_tree::arc &a = t.arcs[e];
    if (a.hi >= m || a.lo >= m)
      continue;
    const double lo = t.nodes[a.lo].value, hi = t.nodes[a.hi].value;
    if (w == top ? (hi == top && lo < top) : (lo <= w && w < hi))
      visit(std::uint32_t(e));
  }
}

// How often the one-shot functions report progress: about as often as the old per-2^16-voxel
// ticks did.
constexpr std::size_t kOneShotChunk = std::size_t(1) << 16;

} // namespace

// ---- contour_tree queries -------------------------------------------------------------------

std::size_t contour_tree::components_at(double w) const {
  std::size_t count = 0;
  visit_arcs_at(*this, w, [&count](std::uint32_t) { ++count; });
  return count;
}

std::vector<std::uint32_t> contour_tree::arcs_at(double w) const {
  std::vector<std::uint32_t> out;
  visit_arcs_at(*this, w, [&out](std::uint32_t e) { out.push_back(e); });
  return out;
}

// ---- contour_tree_builder -------------------------------------------------------------------

struct contour_tree_builder::impl {
  std::optional<voxels> vol; // a shallow copy: shares the caller's buffer
  const float *values = nullptr;
  uint64 nx = 0, ny = 0, nz = 0;
  contour_tree_options opt;
  std::unique_ptr<ct_task> task;
  std::exception_ptr error;
  contour_tree result;
  double progress = 0.0;
  ct_phase phase = ct_phase::preparing;
  bool finished = false, taken = false;

  void start() {
    if (!vol) {
      task = make_prep<float>(values, nx, ny, nz, opt);
      return;
    }
    const void *p = std::as_const(*vol).data_ptr();
    const uint64 x = vol->XDim(), y = vol->YDim(), z = vol->ZDim();
    switch (vol->voxelType()) {
    case UChar:
      task = make_prep<unsigned char>(p, x, y, z, opt);
      return;
    case UShort:
      task = make_prep<unsigned short>(p, x, y, z, opt);
      return;
    case UInt:
      task = make_prep<unsigned int>(p, x, y, z, opt);
      return;
    case Float:
      task = make_prep<float>(p, x, y, z, opt);
      return;
    case Double:
      task = make_prep<double>(p, x, y, z, opt);
      return;
    case UInt64:
      task = make_prep<uint64>(p, x, y, z, opt);
      return;
    case Char:
      task = make_prep<signed char>(p, x, y, z, opt);
      return;
    case Int:
      task = make_prep<int>(p, x, y, z, opt);
      return;
    case Int64:
      task = make_prep<int64>(p, x, y, z, opt);
      return;
    case Undefined:
      break;
    }
    throw contour_tree_error("undefined voxel type");
  }

  // The working arrays stay until the builder is destroyed: freeing them here would be an O(n)
  // pause inside the step.
  [[noreturn]] void fail(std::exception_ptr e) {
    error = e;
    vol.reset();
    std::rethrow_exception(e);
  }
};

contour_tree_builder::contour_tree_builder(const voxels &vol, const contour_tree_options &opt)
    : _impl(std::make_unique<impl>()) {
  _impl->vol.emplace(vol);
  _impl->opt = opt;
}

contour_tree_builder::contour_tree_builder(const float *values, uint64 nx, uint64 ny, uint64 nz,
                                           const contour_tree_options &opt)
    : _impl(std::make_unique<impl>()) {
  _impl->values = values;
  _impl->nx = nx;
  _impl->ny = ny;
  _impl->nz = nz;
  _impl->opt = opt;
}

contour_tree_builder::~contour_tree_builder() = default;
contour_tree_builder::contour_tree_builder(contour_tree_builder &&) noexcept = default;
contour_tree_builder &contour_tree_builder::operator=(contour_tree_builder &&) noexcept = default;

bool contour_tree_builder::step(std::size_t units) {
  if (!_impl)
    return true;
  impl &m = *_impl;
  if (m.error)
    std::rethrow_exception(m.error);
  if (m.finished)
    return true;
  try {
    budget b(units);
    if (!m.task)
      m.start();
    if (m.task->run(b)) {
      m.result = m.task->take();
      m.task.reset();
      m.vol.reset();
      m.finished = true;
      m.phase = ct_phase::done;
      m.progress = 1.0;
    } else {
      m.phase = m.task->phase();
      m.progress = std::max(m.progress, overall_progress(m.phase, m.task->fraction()));
      if (!m.task->uses_input())
        m.vol.reset(); // let the caller's voxels go (and skip a copy-on-write if it writes them)
    }
    if (m.opt.progress && !m.opt.progress(m.progress))
      throw contour_tree_error("cancelled");
  } catch (const std::bad_alloc &) {
    m.finished = false;
    m.fail(std::make_exception_ptr(contour_tree_error("out of memory")));
  } catch (const std::length_error &) {
    m.finished = false;
    m.fail(std::make_exception_ptr(contour_tree_error("out of memory")));
  } catch (...) {
    m.finished = false;
    m.fail(std::current_exception());
  }
  return m.finished;
}

bool contour_tree_builder::done() const { return !_impl || _impl->finished || _impl->error; }

double contour_tree_builder::progress() const { return _impl ? _impl->progress : 1.0; }

const char *contour_tree_builder::phase() const {
  if (!_impl)
    return "done";
  if (_impl->error)
    return "failed";
  return kPhaseName[int(_impl->phase)];
}

contour_tree contour_tree_builder::take() {
  if (!_impl)
    throw contour_tree_error("no tree: the builder was moved from");
  impl &m = *_impl;
  if (m.error)
    std::rethrow_exception(m.error);
  if (!m.finished)
    throw contour_tree_error("the contour tree is not built yet");
  if (m.taken)
    throw contour_tree_error("the contour tree was already taken");
  m.taken = true;
  return std::move(m.result);
}

// ---- contour_tree_simplifier ----------------------------------------------------------------

contour_tree_simplifier::contour_tree_simplifier() : _impl(std::make_unique<impl>()) {}
contour_tree_simplifier::~contour_tree_simplifier() = default;
contour_tree_simplifier::contour_tree_simplifier(contour_tree_simplifier &&) noexcept = default;
contour_tree_simplifier &
contour_tree_simplifier::operator=(contour_tree_simplifier &&) noexcept = default;

contour_tree_simplifier contour_tree_simplifier::by_persistence(contour_tree t,
                                                                double min_persistence) {
  contour_tree_simplifier s;
  impl &m = *s._impl;
  m.begin(std::move(t));
  if (!(min_persistence > 0.0) || m.t.arcs.size() < 2) {
    m.no_work();
    return s;
  }
  m.min_persistence = min_persistence;
  m.limited = true;
  m.max_nodes = 0;
  m.target = 2;
  const double range = m.t.value_max - m.t.value_min;
  if (range > 0.0) {
    m.limit = min_persistence * range;
    m.st = impl::stage::clear;
  } else {
    m.st = impl::stage::range;
  }
  m.total_units = (m.st == impl::stage::range ? uint64(m.m) : 0) + 4 * uint64(m.m) + 2 * m.na;
  return s;
}

contour_tree_simplifier contour_tree_simplifier::to_nodes(contour_tree t, std::size_t max_nodes) {
  contour_tree_simplifier s;
  impl &m = *s._impl;
  m.begin(std::move(t));
  if (m.t.nodes.size() <= max_nodes || m.t.arcs.size() < 2) {
    m.no_work();
    return s;
  }
  m.max_nodes = max_nodes;
  m.target = std::max<std::size_t>(max_nodes, 2);
  m.st = impl::stage::clear;
  m.total_units = 4 * uint64(m.m) + 2 * m.na;
  return s;
}

bool contour_tree_simplifier::step(std::size_t units) {
  if (!_impl)
    return true;
  impl &m = *_impl;
  if (m.error)
    std::rethrow_exception(m.error);
  if (m.st == impl::stage::done)
    return true;
  try {
    budget b(units);
    const bool finished = m.step(b);
    m.progress = std::max(m.progress, finished ? 1.0 : m.fraction());
    return finished;
  } catch (const std::bad_alloc &) {
    m.error = std::make_exception_ptr(contour_tree_error("out of memory"));
  } catch (const std::length_error &) {
    m.error = std::make_exception_ptr(contour_tree_error("out of memory"));
  } catch (...) {
    m.error = std::current_exception();
  }
  std::rethrow_exception(m.error); // the arrays stay until destruction: no O(n) free in a step
}

bool contour_tree_simplifier::done() const {
  return !_impl || _impl->st == impl::stage::done || _impl->error;
}

double contour_tree_simplifier::progress() const { return _impl ? _impl->progress : 1.0; }

const char *contour_tree_simplifier::phase() const { return _impl ? _impl->phase() : "done"; }

contour_tree contour_tree_simplifier::take() {
  if (!_impl)
    throw contour_tree_error("no tree: the simplifier was moved from");
  impl &m = *_impl;
  if (m.error)
    std::rethrow_exception(m.error);
  if (m.st != impl::stage::done)
    throw contour_tree_error("the simplified tree is not ready yet");
  if (m.taken)
    throw contour_tree_error("the simplified tree was already taken");
  m.taken = true;
  return std::move(m.out);
}

// ---- public entry points --------------------------------------------------------------------

namespace {
contour_tree run_to_end(contour_tree_builder &b, const contour_tree_options &opt) {
  const std::size_t chunk = opt.progress ? kOneShotChunk : 0;
  while (!b.step(chunk)) {
  }
  return b.take();
}
contour_tree run_to_end(contour_tree_simplifier &&s) {
  while (!s.step(0)) {
  }
  return s.take();
}
} // namespace

contour_tree compute_contour_tree(const voxels &vol, const contour_tree_options &opt) {
  contour_tree_builder b(vol, opt);
  return run_to_end(b, opt);
}

contour_tree compute_contour_tree(const float *values, uint64 nx, uint64 ny, uint64 nz,
                                  const contour_tree_options &opt) {
  contour_tree_builder b(values, nx, ny, nz, opt);
  return run_to_end(b, opt);
}

contour_tree simplify_contour_tree(const contour_tree &t, double min_persistence) {
  return run_to_end(contour_tree_simplifier::by_persistence(t, min_persistence));
}

contour_tree simplify_contour_tree_to(const contour_tree &t, std::size_t max_nodes) {
  return run_to_end(contour_tree_simplifier::to_nodes(t, max_nodes));
}

std::vector<float> layout_contour_tree(const contour_tree &t, std::uint32_t seed) {
  const std::size_t m = t.nodes.size();
  std::vector<float> out(m, 0.5f);
  if (m < 2 || t.arcs.empty())
    return out;

  // Neighbours up first, then down, each in arc order: HeightField::processVtx's visiting order.
  std::vector<u32> ustart(m + 1, 0), dstart(m + 1, 0);
  for (const contour_tree::arc &a : t.arcs)
    if (a.hi < m && a.lo < m && a.hi != a.lo) {
      ++ustart[a.lo + 1];
      ++dstart[a.hi + 1];
    }
  for (std::size_t i = 0; i < m; ++i) {
    ustart[i + 1] += ustart[i];
    dstart[i + 1] += dstart[i];
  }
  std::vector<u32> uadj(ustart[m]), dadj(dstart[m]);
  {
    std::vector<u32> uf(ustart.begin(), ustart.end() - 1), df(dstart.begin(), dstart.end() - 1);
    for (const contour_tree::arc &a : t.arcs)
      if (a.hi < m && a.lo < m && a.hi != a.lo) {
        uadj[uf[a.lo]++] = a.hi;
        dadj[df[a.hi]++] = a.lo;
      }
  }

  auto above = [&t](std::size_t a, std::size_t b) {
    const contour_tree::node &p = t.nodes[a], &q = t.nodes[b];
    return p.value > q.value || (p.value == q.value && p.voxel > q.voxel);
  };
  std::size_t root = 0;
  for (std::size_t i = 1; i < m; ++i)
    if (above(i, root))
      root = i;

  // VolumeRover2's drawing: a depth-first walk from the global maximum, each newly reached node
  // offset sideways from its discoverer by (f_child - f_parent) / tan(theta) with theta a whole
  // number of degrees in [60, 120). Its rand() was never seeded; a fixed engine with explicit
  // modulo (the standard distributions differ between libraries) and an explicit fma (so no
  // compiler contracts it differently) make the result the same everywhere.
  std::minstd_rand rng(seed);
  std::vector<double> x(m, 0.0);
  std::vector<std::uint8_t> seen(m, 0);
  std::vector<u32> stack, kids;
  auto grow = [&](std::size_t r) {
    seen[r] = 1;
    stack.push_back(u32(r));
    while (!stack.empty()) {
      const u32 v = stack.back();
      stack.pop_back();
      kids.clear();
      auto place = [&](u32 c) {
        if (seen[c])
          return;
        seen[c] = 1;
        const int theta = 60 + int(rng() % 60);
        const double df = t.nodes[c].value - t.nodes[v].value;
        x[c] = (df == 0.0 || !std::isfinite(df)) ? x[v] : std::fma(df, cot_degrees(theta), x[v]);
        kids.push_back(c);
      };
      for (u32 k = ustart[v]; k < ustart[v + 1]; ++k)
        place(uadj[k]);
      for (u32 k = dstart[v]; k < dstart[v + 1]; ++k)
        place(dadj[k]);
      stack.insert(stack.end(), kids.begin(), kids.end());
    }
  };
  grow(root);
  for (std::size_t i = 0; i < m; ++i)
    if (!seen[i])
      grow(i);

  const auto [lo, hi] = std::minmax_element(x.begin(), x.end());
  const double x0 = *lo, span = *hi - *lo;
  if (span > 0.0)
    for (std::size_t i = 0; i < m; ++i)
      out[i] = float((x[i] - x0) / span);
  return out;
}

} // namespace cvc
