/*
  Copyright 2003, 2026 The University of Texas at Austin

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

// GenSeg's fast marching (transfix/volrover src/Segmentation/GenSeg/segment.cpp, at the sha in
// tools/volrover-goldens/volrover.sha), vendored as genseg_context: the legacy file's globals
// are its members, so two runs never share anything (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md
// §1.3), and its functions are its members, their bodies as they were. Segment() and its search
// for a seed's class are coroutines that yield a unit at a time (§1.9), so cvc::segment_stepper
// (segmentation.cpp) can run them in slices. Please cite the paper GenSeg implements when you use
// it:
//
//   C. Bajaj, Z. Yu, M. Auer, "Volumetric feature extraction and visualization of tomographic
//   molecular imaging", Journal of Structural Biology 144(1-2):132-143, 2003.
//
// Private to the segmentation port: cvc/volume/segmentation.h is the API.

#ifndef CVC_VOLUME_SEGMENTATION_GENSEG_H
#define CVC_VOLUME_SEGMENTATION_GENSEG_H

#include <coroutine>
#include <cstddef>
#include <exception>
#include <memory>
#include <utility>
#include <vector>

namespace cvc::genseg {

// A routine that runs a unit at a time: each co_yield hands control back to whoever resumed it.
// Destroying the task destroys its frame (a cancel frees everything at once). What the routine
// throws escapes from resume().
class task {
public:
  struct promise_type {
    std::exception_ptr error;
    task get_return_object() {
      return task(std::coroutine_handle<promise_type>::from_promise(*this));
    }
    std::suspend_always initial_suspend() noexcept { return {}; }
    std::suspend_always final_suspend() noexcept { return {}; }
    std::suspend_always yield_value(int) noexcept { return {}; }
    void return_void() noexcept {}
    void unhandled_exception() { error = std::current_exception(); }
  };

  task() = default;
  explicit task(std::coroutine_handle<promise_type> h) : _h(h) {}
  task(task &&o) noexcept : _h(std::exchange(o._h, {})) {}
  task &operator=(task &&o) noexcept {
    if (this != &o) {
      if (_h)
        _h.destroy();
      _h = std::exchange(o._h, {});
    }
    return *this;
  }
  task(const task &) = delete;
  task &operator=(const task &) = delete;
  ~task() {
    if (_h)
      _h.destroy();
  }

  // Runs to the next yield; true once the routine has returned. Rethrows what it threw.
  bool resume() {
    if (!_h || _h.done())
      return true;
    _h.resume();
    if (_h.promise().error)
      std::rethrow_exception(std::exchange(_h.promise().error, nullptr));
    return _h.done();
  }
  explicit operator bool() const { return bool(_h); }

private:
  std::coroutine_handle<promise_type> _h;
};

// An array the legacy code malloc()ed: allocated without being filled (the routine's own loops
// fill it, a unit at a time, so allocating costs no work proportional to its size).
template <class T> class buffer {
public:
  void allocate(std::size_t n) {
    _p.reset(new T[n]);
    _n = n;
  }
  void release() {
    _p.reset();
    _n = 0;
  }
  T &operator[](std::size_t i) { return _p[i]; }
  const T &operator[](std::size_t i) const { return _p[i]; }
  T *data() { return _p.get(); }
  const T *data() const { return _p.get(); }
  std::size_t size() const { return _n; }

private:
  std::unique_ptr<T[]> _p;
  std::size_t _n = 0;
};

// The legacy MinHeapS: a binary min-heap of voxels by arrival time, as four parallel arrays.
// The arrays grow when an insert would overflow them (the legacy code wrote past their end in
// that case, which only a heap of duplicates can reach).
struct MinHeapS {
  buffer<int> x, y, z;
  buffer<float> t;
  int size = 0;
  void allocate(std::size_t n);
  void grow(); // doubles the arrays, keeping the first `size` entries
  void release();
};

struct INTVEC {
  int x;
  int y;
  int z;
};

// The legacy SeedPoint list, its pointers as indices into one array (-1 is NULL).
struct SDPNT {
  int x;
  int y;
  int z;
  int next;
};

// One call of the legacy Segment(): its globals, its locals that outlive a unit, and its
// functions. Fill the inputs, then resume run() until it returns; the outputs are then in
// seed_index (each voxel's class, or 254 / 255 for none) and tdata (MAX_TIME + 1 where two
// fronts met). Not thread-safe, but it may move between threads between units.
class genseg_context {
public:
  static constexpr float MAX_TIME = 9999999.0f;

  // ---- inputs ----
  int XDIM = 0, YDIM = 0, ZDIM = 0;
  // The voxels, x fastest, as floats (read_data's dataset before its normalisation), every one
  // finite and not all equal; run() normalises them in place to 0..255 as read_data did.
  buffer<float> dataset;
  // Segment()'s tlow and thigh: on the normalised scale, or, with data_unit_thresholds, in the
  // voxels' units, which run() turns into the normalised scale as it does the voxels.
  float tlow = 0.0f, thigh = 0.0f;
  bool data_unit_thresholds = false;
  float ALPHA = 0.12f;         // the legacy constant: a step costs 0.01 exp(ALPHA |grad|)
  bool automatic_seeds = true; // GetSeeds(): the local maxima above thigh (else none)
  // The seeds of each class (Segment()'s seed_list), in order.
  std::vector<std::vector<INTVEC>> seed_list;

  // ---- the routine ----
  // run() does a unit of work at a time, and yields once it has done `units_left` of them (so
  // set units_left before each resume); `units` counts them, and restarts at each phase.
  std::size_t units_left = 0;
  std::size_t units = 0;
  const char *phase = "normalising";
  void begin(const char *name) {
    phase = name;
    units = 0;
  }
  // For progress: the voxels, the automatic seeds found, the voxels the final march popped.
  std::size_t voxel_count() const { return std::size_t(XDIM) * YDIM * ZDIM; }
  int seeds_found = 0;
  int seeds_displaced = 0; // of them, where a point class has a seed
  int seeds_left = 0;      // unclassified once the threshold sweep ended
  std::size_t marched = 0;
  float minraw = 0.0f, maxraw = 0.0f; // read_data's

  task run();

  // ---- the legacy globals (segment.cpp) ----
  int min_x = 0, min_y = 0, min_z = 0;
  float min_t = 0.0f;
  MinHeapS min_heap;
  MinHeapS T_min_heap;
  unsigned char T_index = 0;
  buffer<unsigned char> seed_index;
  float t_low = 0.0f, t_high = 0.0f;
  buffer<float> tdata;
  buffer<float> T_tdata;
  buffer<float> ImgGrad;
  int SeedNum = 0;
  int PhaseNum = 0;
  int start_ptr = 0, end_ptr = 0;
  buffer<INTVEC> IndexArray;
  std::vector<SDPNT> critical; // the list of automatic seeds' nodes

private:
  std::size_t IndexVect(int i, int j, int k) const {
    return (std::size_t(k) * YDIM + std::size_t(j)) * XDIM + std::size_t(i);
  }

  task normalise();
  task GetGradient(float *edge_mag);
  task GetSeeds(unsigned char *result);
  task Segment();
  task SearchIndex(int critical_start);
  void GetMinimum();
  void InsertHeap(int, int, int);
  void GetTime();
  void T_GetMinimum();
  void T_InsertHeap(int, int, int);
  void T_GetTime();

  // SearchIndex's results: the voxel found (*m, *n, *l) and its class (255: none).
  int search_m = 0, search_n = 0, search_l = 0;
  unsigned char search_index = 255;
};

} // namespace cvc::genseg

// One unit of work done in a coroutine of the context `ctx`: yields once the slice is used up.
#define GS_UNIT_OF(ctx)                                                                            \
  do {                                                                                             \
    ++(ctx).units;                                                                                 \
    if (--(ctx).units_left == 0)                                                                   \
      co_yield 0;                                                                                  \
  } while (0)
// Run a coroutine (a task) to its end, yielding whenever it yields.
#define GS_RUN(call)                                                                               \
  do {                                                                                             \
    ::cvc::genseg::task gs_sub_ = (call);                                                          \
    while (!gs_sub_.resume())                                                                      \
      co_yield 0;                                                                                  \
  } while (0)

#endif // CVC_VOLUME_SEGMENTATION_GENSEG_H
