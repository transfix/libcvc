/*
  Copyright 2026 The University of Texas at Austin

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

// LodPyramidCache.h -- mesh LOD ladders for LodGraphicsNodes, built once per
// distinct mesh, off the render thread where the build has threads.
//
// attach(node, mesh) gives a LodGraphicsNode its ladder. A mesh whose pyramid
// is already cached gets it at once (setPyramid: no QEM pass). Any other mesh is
// shown at full detail straight away as a one-rung ladder (setBase), and its
// pyramid is built in the background; poll(), called once per frame, hands the
// finished pyramid to every node still waiting for it (appendRungs, which keeps
// each node's rung 0 -- its node, its style and its uploaded buffers). Meshes
// are keyed by key_of(): cvc::lod::content_hash at a 1 mm lattice mixed with the
// pyramid parameters, so two reads of one asset -- two nodes, or the same node
// re-realized after a scene change -- share one build and one cached pyramid,
// whatever their vertex or triangle order. (Two meshes the hash cannot tell
// apart, e.g. ones that differ only below the 1 mm lattice, share the first
// one's pyramid: its rung 0 is what both draw.)
//
// --- Builds ------------------------------------------------------------------
//
// Builds run on one lazily created cvc::async_lane, one at a time, in attach
// order, each a plain cvc::lod::build_mesh_pyramid(mesh, params, nullptr). No
// thread_pool is ever made or borrowed: starting a std::thread aborts on
// Emscripten without pthreads, and app::computePool() is shared with per-frame
// work. Where the build has no threads (async_lane_has_threads false) the lane
// is deferred and poll() runs at most ONE build per call, on the render thread
// -- so a build's QEM pass is a one-off stall of that frame. max_build_tris
// bounds it: a larger mesh keeps its single rung, never builds, and leaves a
// warning (take_warnings). default_max_build_tris() is 2M triangles with
// threads and 60k without.
//
// A build works on a deep copy of its mesh and shares nothing with the scene
// while it runs; its result is read only after its done flag (release/acquire).
//
// --- Lifetime ----------------------------------------------------------------
//
// Cached pyramids live as long as the cache (no eviction): keep it beside the
// scene for as long as that scene can be re-realized, so a re-realize re-attaches
// from the cache. The cache holds nodes weakly: a node destroyed before its
// build lands is skipped, and the build still lands in the cache for the next
// attach. Destroying the cache cancels every build still queued (each returns
// at once) and waits for the one running, if any -- and on wasm with pthreads
// that wait blocks the browser main thread, so destroy it there only at exit.
//
// Threading: every member function runs on the scene's owner thread (the one
// that populates and renders it), like the nodes it feeds.

#ifndef __CVC_GL_LOD_PYRAMID_CACHE_H__
#define __CVC_GL_LOD_PYRAMID_CACHE_H__

#include <cstddef>
#include <cstdint>
#include <cvc/core/async_lane.h>
#include <cvc/geometry/geometry.h>
#include <cvc/lod/pyramid.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace cvc {
namespace gl {

class LodGraphicsNode;

class LodPyramidCache {
public:
  // `params` shapes every pyramid this cache builds. A mesh with more than
  // `max_build_tris` triangles (quads count as two) is never built. `mode`
  // picks the lane: threaded (the default) builds on a worker thread; deferred
  // builds one mesh per poll() on the caller, which is what a build without
  // threads gets whatever is asked for.
  explicit LodPyramidCache(cvc::lod::pyramid_params params = cvc::lod::pyramid_params(),
                           std::uint64_t max_build_tris = default_max_build_tris(),
                           cvc::lane_mode mode = cvc::lane_mode::threaded);
  // Cancels the queued builds, then waits for the running one (see the header).
  ~LodPyramidCache();

  LodPyramidCache(const LodPyramidCache &) = delete;
  LodPyramidCache &operator=(const LodPyramidCache &) = delete;
  LodPyramidCache(LodPyramidCache &&) = delete;
  LodPyramidCache &operator=(LodPyramidCache &&) = delete;

  // Give `node` the ladder for `mesh`, replacing whatever rungs it had. Returns
  // true on a cache hit (the whole pyramid is attached now). Otherwise `mesh`
  // is attached as rung 0 and the node waits for the pyramid, which one build
  // per key produces -- an attach of a mesh already being built joins that
  // build. A node attached again stops waiting for its earlier mesh. Populate a
  // waiting node only through the cache: appendRungs would replace a rung 0
  // that is no longer the mesh it waited for. A null node does nothing.
  bool attach(const std::shared_ptr<LodGraphicsNode> &node, const cvc::geometry &mesh);

  // Once per frame: run one queued build when the lane is deferred, move every
  // finished pyramid into the cache, and append its rungs to the nodes still
  // waiting for it. Returns how many builds finished during this call (a
  // failed one included: its nodes keep their one rung, with a warning).
  std::size_t poll();

  // Builds started over the cache's life (one per distinct key).
  std::size_t builds() const { return m_builds; }
  // Pyramids held.
  std::size_t cached() const { return m_cache.size(); }
  // Builds started whose results poll() has not taken in yet.
  std::size_t in_flight() const { return m_pending.size(); }
  // Wall time of the most recently landed build, in milliseconds (0 before one).
  double last_build_ms() const { return m_lastBuildMs; }

  // Non-fatal issues since the last call -- a mesh over the build cap (once per
  // mesh), a build that threw -- in the order they happened; clears them.
  std::vector<std::string> take_warnings();

  // The cache key of `mesh` under `params`: lod::content_hash(mesh, 1 mm), the
  // geometry type, and every pyramid_params field a mesh build reads.
  static std::uint64_t key_of(const cvc::geometry &mesh, const cvc::lod::pyramid_params &params);
  // 2'000'000 triangles where async_lane_has_threads, 60'000 where it is not.
  static std::uint64_t default_max_build_tris();

private:
  struct build_job; // the lane <-> owner handoff (LodPyramidCache.cpp)

  struct pending_build {
    std::shared_ptr<build_job> job;
    cvc::geometry base; // the waiting nodes' rung 0 (a copy sharing its storage)
    std::vector<std::weak_ptr<LodGraphicsNode>> waiters;
  };

  // key_of, memoized by an exact (bit-for-bit, order-dependent) fingerprint of
  // the arrays content_hash reads: a mesh read again with the same bytes skips
  // the canonical sort, which dominates attach().
  std::uint64_t key_for(const cvc::geometry &mesh);
  // Stop `node` waiting for any build; drop waiters whose node is gone.
  void forget(const LodGraphicsNode *node);
  cvc::async_lane &lane();

  const cvc::lod::pyramid_params m_params;
  const std::uint64_t m_maxBuildTris;
  const cvc::lane_mode m_mode;

  std::unordered_map<std::uint64_t, cvc::lod::mesh_pyramid> m_cache;
  std::unordered_map<std::uint64_t, pending_build> m_pending;
  std::unordered_set<std::uint64_t> m_unbuilt;             // keys whose build failed
  std::unordered_set<std::uint64_t> m_overCap;             // triangle counts warned as over the cap
  std::unordered_map<std::uint64_t, std::uint64_t> m_keys; // fingerprint -> key_of
  std::vector<std::string> m_warnings;
  std::size_t m_builds = 0;
  double m_lastBuildMs = 0.0;
  // Declared last, so it is destroyed (drained and joined) before what its
  // jobs' results land in.
  std::unique_ptr<cvc::async_lane> m_lane;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_LOD_PYRAMID_CACHE_H__
