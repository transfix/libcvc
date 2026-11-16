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
// tools/volrover-goldens/volrover.sha), ported as genseg_context: the legacy file's globals are
// its members, so two runs never share anything (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md
// §1.3). Segment() and its search for a seed's class are coroutines that yield a unit at a time
// (§1.9), so cvc::segment_stepper (segmentation.cpp) can run them in slices. genseg.cpp says what
// changed from the legacy code; the labels are the same, voxel for voxel. Please cite the paper
// GenSeg implements when you use it:
//
//   C. Bajaj, Z. Yu, M. Auer, "Volumetric feature extraction and visualization of tomographic
//   molecular imaging", Journal of Structural Biology 144(1-2):132-143, 2003.
//
// Private to the segmentation port: cvc/volume/segmentation.h is the API.

#ifndef CVC_VOLUME_SEGMENTATION_GENSEG_H
#define CVC_VOLUME_SEGMENTATION_GENSEG_H

#include <coroutine>
#include <cstddef>
#include <cstdint>
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

// The legacy MinHeapS: a binary min-heap of voxels (their linear indices) by arrival time, with
// the legacy code's sift rules, so voxels of equal times come out in the legacy order (that order
// decides the labels where fronts meet). The arrays grow when a push would overflow them.
struct min_heap {
  buffer<float> t;
  buffer<std::size_t> v;
  std::size_t size = 0;

  void allocate(std::size_t n);
  void release();
  void clear() { size = 0; }
  void push(std::size_t voxel, float time);   // InsertHeap()
  void append(std::size_t voxel, float time); // at the end, as the legacy code placed the seeds
  bool pop(std::size_t &voxel, float &time);  // GetMinimum(); false when empty

private:
  void grow();
};

// A voxel and its coordinates.
struct voxel_at {
  int x, y, z;
  std::size_t v;
};

struct INTVEC {
  int x;
  int y;
  int z;
};

// One call of the legacy Segment(): its globals, its locals that outlive a unit, and its
// functions. Fill the inputs, then resume run() until it returns; the outputs are then in
// seed_index (each voxel's class, or 254 / 255 for none) and tdata (MAX_TIME + 1 where a front
// stopped). Not thread-safe, but it may move between threads between units.
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
  std::size_t voxel_count() const { return std::size_t(XDIM) * YDIM * ZDIM; }
  // For progress and the result: the automatic seeds found, those a point class's seed covered,
  // those left unclassified once the sweep ended, and the voxels the final march popped.
  int seeds_found = 0;
  int seeds_displaced = 0;
  int seeds_left = 0;
  std::size_t marched = 0;
  float minraw = 0.0f, maxraw = 0.0f; // read_data's

  task run();

  // ---- the legacy globals that are left ----
  buffer<unsigned char> seed_index; // each voxel's class, 254 an automatic seed, 255 none
  buffer<float> tdata;              // the final march's arrival times
  float t_low = 0.0f, t_high = 0.0f;
  int SeedNum = 0; // automatic seeds not yet classified

private:
  std::size_t IndexVect(int i, int j, int k) const {
    return (std::size_t(k) * YDIM + std::size_t(j)) * XDIM + std::size_t(i);
  }
  voxel_at at(std::size_t v) const {
    const std::size_t xy = std::size_t(XDIM) * YDIM;
    return {int(v % XDIM), int((v / XDIM) % YDIM), int(v / xy), v};
  }
  // The six face neighbours of c in the legacy order (x - 1, x + 1, y - 1, y + 1, z - 1, z + 1),
  // each clamped to the volume: at an edge, c itself stands in for the missing neighbour.
  void neighbours(const voxel_at &c, voxel_at out[6]) const;

  task normalise();
  task GetGradient(float *edge_mag);
  task step_costs();
  task GetSeeds(unsigned char *result);
  task Segment();
  task SearchIndex();
  void GetTime(const voxel_at &m);
  void T_GetTime(const voxel_at &m);
  // The threshold sweep's shortcut (Segment()): sweep_tree() builds the component tree of the
  // voxels a search front can pass, by level; sweep_labelled() brings it up to date with the
  // voxels the sweep has labelled since; reach_level() is the highest threshold at which a search
  // can reach a labelled voxel at all, or -1.
  task sweep_tree();
  task sweep_labelled();
  int reach_level() const;
  int face_neighbours(std::size_t v, std::size_t out[6]) const;
  std::uint32_t tree_node(std::uint32_t v) const;
  void mark_up(std::uint32_t v);

  buffer<float> step_cost; // per voxel, 0.01 exp(ALPHA |grad|)
  buffer<float> T_tdata;   // the sweep's search times
  min_heap heap, T_heap;
  unsigned char T_index = 0;
  std::vector<std::size_t> unclassified; // the automatic seeds the sweep has yet to classify
  std::vector<std::size_t> touched;      // the voxels whose T_tdata the last search set

  // SearchIndex()'s results: the seed it found, and its class (255: none).
  std::size_t search_voxel = 0;
  unsigned char search_index = 255;

  // The sweep's shortcut. A voxel's level is the floor of the least of its six neighbours'
  // values (as neighbours() gives them): a search front passes it at every threshold up to its
  // level. An automatic seed's level is 256 (fronts start there at any threshold); voxels below
  // sweep_floor, the lowest threshold the sweep searches at, are left out (tree_out). A node of
  // the component tree is a set of voxels at levels l and up, connected through face neighbours,
  // at level l: tree_parent gives each voxel the voxel that names its node, and a node's name the
  // name of the node above it (itself at a root). Per node, by its name: tree_seeds, how many
  // unclassified seeds it holds, and tree_marked, whether it holds a voxel next to a labelled
  // one; qualifying[l] counts the nodes of level l with both.
  static constexpr std::uint16_t tree_out = 0xffffu;
  bool sweep_shortcut = false;
  int sweep_floor = 0;
  std::vector<std::uint16_t> tree_level;
  std::vector<std::uint32_t> tree_parent, tree_seeds;
  std::vector<unsigned char> tree_marked;
  std::size_t qualifying[257] = {};
  // During the sweep, GetTime() records the voxels it labels, and the automatic seeds among them.
  bool log_labels = false;
  std::vector<std::uint32_t> labelled_log, classified_log;
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
