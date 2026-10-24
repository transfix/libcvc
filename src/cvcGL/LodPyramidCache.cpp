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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/LodPyramidCache.h>
#include <cvc/lod/tiles.h> // content_hash
#include <exception>
#include <system_error>
#include <utility>

namespace cvc {
namespace gl {

namespace {

// The position lattice of the cache key: 1 mm, for metre-scale scenes.
constexpr double kHashQuantumM = 0.001;

// 64-bit word hashing: each word goes through murmur3's finalizer, then FNV-1a's
// xor-multiply. In-memory only (nothing persists it), so word-at-a-time is
// enough and keeps a 70k-triangle fingerprint well under a millisecond.
struct word_hash {
  std::uint64_t h = 14695981039346656037ull;

  void word(std::uint64_t w) {
    w ^= w >> 33;
    w *= 0xff51afd7ed558ccdull;
    w ^= w >> 33;
    h = (h ^ w) * 1099511628211ull;
  }
  void real(double d) {
    std::uint64_t w = 0;
    std::memcpy(&w, &d, sizeof(w));
    word(w);
  }
  template <class C> void reals(const C &c) {
    word(c.size());
    for (const auto &e : c)
      for (double d : e)
        real(d);
  }
  template <class C> void indices(const C &c) {
    word(c.size());
    for (const auto &e : c)
      for (std::uint64_t i : e)
        word(i);
  }
};

// Exact, order-dependent fingerprint of every array content_hash reads, plus
// the geometry type key_of mixes in. Equal fingerprints mean byte-equal arrays
// (up to a 64-bit collision), hence an equal key_of -- never the reverse.
std::uint64_t fingerprint(const cvc::geometry &g) {
  word_hash h;
  h.word(static_cast<std::uint64_t>(g.get_geometry_type()));
  h.reals(g.const_points());
  h.reals(g.const_normals());
  h.reals(g.const_colors());
  h.reals(g.const_uvs());
  h.reals(g.const_tangents());
  h.indices(g.const_tris());
  h.indices(g.const_quads());
  return h.h;
}

std::uint64_t triangles(const cvc::geometry &g) { return g.num_tris() + 2 * g.num_quads(); }

} // namespace

// The lane <-> owner handoff. The lane writes `result`, `error` and `ms`, then
// sets `done` (release); the owner reads them only after seeing it (acquire).
// `src` is a deep copy no one else holds, released on the lane once used.
struct LodPyramidCache::build_job {
  std::shared_ptr<const cvc::geometry> src;
  cvc::lod::pyramid_params params;
  std::uint64_t tris = 0;
  std::atomic<bool> cancel{false};
  std::atomic<bool> done{false};
  cvc::lod::mesh_pyramid result;
  std::string error;
  double ms = 0.0;

  // Runs on the lane (the worker thread, or the poll()ing thread when
  // deferred). Never throws: whatever happens is reported through the job.
  void run() {
    const auto t0 = std::chrono::steady_clock::now();
    if (!cancel.load(std::memory_order_acquire)) {
      try {
        // pool = nullptr: serial, and no thread is started for it.
        result = cvc::lod::build_mesh_pyramid(*src, params, nullptr);
      } catch (const std::exception &e) {
        error = e.what();
      } catch (...) {
        error = "unknown error";
      }
    }
    src.reset();
    ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    done.store(true, std::memory_order_release);
  }
};

LodPyramidCache::LodPyramidCache(cvc::lod::pyramid_params params, std::uint64_t max_build_tris,
                                 cvc::lane_mode mode)
    : m_params(std::move(params)), m_maxBuildTris(max_build_tris),
      m_mode(cvc::async_lane_has_threads ? mode : cvc::lane_mode::deferred) {}

LodPyramidCache::~LodPyramidCache() {
  // Cancel first: stop() runs every job still queued (on the worker, or here
  // for a deferred lane), and a cancelled one returns without building. Only a
  // build already running is waited for.
  for (auto &kv : m_pending)
    kv.second.job->cancel.store(true, std::memory_order_release);
  if (m_lane)
    m_lane->stop();
}

std::uint64_t LodPyramidCache::default_max_build_tris() {
  // With threads the QEM pass runs beside the render loop and the cap only
  // bounds memory; without them it runs ON a frame, so the cap bounds that stall.
  return cvc::async_lane_has_threads ? 2000000u : 60000u;
}

std::uint64_t LodPyramidCache::key_of(const cvc::geometry &mesh,
                                      const cvc::lod::pyramid_params &p) {
  word_hash h;
  h.word(cvc::lod::content_hash(mesh, kHashQuantumM));
  // content_hash sees triangles only; the type is what GeometryNode draws them as.
  h.word(static_cast<std::uint64_t>(mesh.get_geometry_type()));
  h.word(static_cast<std::uint64_t>(static_cast<std::int64_t>(p.max_rungs)));
  h.real(p.mesh_ratio);
  h.word(p.mesh_min_tris);
  h.word((p.preserve_boundary ? 1u : 0u) | (p.weld_seams ? 2u : 0u) |
         (p.recompute_normals ? 4u : 0u));
  h.real(p.seam_epsilon);
  return h.h;
}

std::uint64_t LodPyramidCache::key_for(const cvc::geometry &mesh) {
  const std::uint64_t f = fingerprint(mesh);
  const auto it = m_keys.find(f);
  if (it != m_keys.end())
    return it->second;
  const std::uint64_t key = key_of(mesh, m_params);
  m_keys.emplace(f, key);
  return key;
}

cvc::async_lane &LodPyramidCache::lane() {
  if (!m_lane) {
    try {
      m_lane = std::make_unique<cvc::async_lane>("lod", m_mode);
    } catch (const std::system_error &e) {
      // No worker thread to be had: build on poll() instead, as a build
      // without threads always does.
      m_warnings.push_back(std::string("LodPyramidCache: no build thread (") + e.what() +
                           "); building one mesh per poll() instead");
      m_lane = std::make_unique<cvc::async_lane>("lod", cvc::lane_mode::deferred);
    }
  }
  return *m_lane;
}

void LodPyramidCache::forget(const LodGraphicsNode *node) {
  for (auto &kv : m_pending) {
    std::vector<std::weak_ptr<LodGraphicsNode>> &w = kv.second.waiters;
    w.erase(std::remove_if(w.begin(), w.end(),
                           [node](const std::weak_ptr<LodGraphicsNode> &p) {
                             const std::shared_ptr<LodGraphicsNode> n = p.lock();
                             return !n || n.get() == node;
                           }),
            w.end());
  }
}

bool LodPyramidCache::attach(const std::shared_ptr<LodGraphicsNode> &node,
                             const cvc::geometry &mesh) {
  if (!node)
    return false;
  // Whatever the node waited for before, it shows `mesh` now.
  forget(node.get());
  const std::uint64_t key = key_for(mesh);

  const auto hit = m_cache.find(key);
  if (hit != m_cache.end()) {
    node->setPyramid(hit->second);
    return true;
  }

  node->setBase(mesh); // full detail at once; the coarser rungs follow
  const auto pending = m_pending.find(key);
  if (pending != m_pending.end()) {
    pending->second.waiters.push_back(node);
    return false;
  }
  if (m_unbuilt.count(key))
    return false; // warned about when it was first refused

  const std::uint64_t tris = triangles(mesh);
  if (tris > m_maxBuildTris) {
    m_unbuilt.insert(key);
    m_warnings.push_back("LodPyramidCache: a " + std::to_string(tris) +
                         "-triangle mesh is over the " + std::to_string(m_maxBuildTris) +
                         "-triangle LOD build cap; it draws at full detail only");
    return false;
  }

  cvc::async_lane &builder = lane(); // before any bookkeeping: making it may throw
  auto job = std::make_shared<build_job>();
  auto src = std::make_shared<cvc::geometry>();
  src->copy(mesh, /*deepCopy=*/true); // nothing shared with the scene while it builds
  job->src = std::move(src);
  job->params = m_params;
  job->tris = tris;
  pending_build &p = m_pending[key];
  p.job = job;
  p.base = mesh; // shares the storage of the node's rung 0
  p.waiters.push_back(node);
  ++m_builds;
  // The job captures only its handoff struct: the last reference to it may drop
  // on the lane, and nothing in it outlives that.
  if (!builder.submit([job]() { job->run(); })) {
    job->error = "the build lane is stopped";
    job->done.store(true, std::memory_order_release);
  }
  return false;
}

std::size_t LodPyramidCache::poll() {
  if (m_lane && !m_lane->threaded())
    m_lane->run_deferred(1); // one build per frame: it runs on this thread
  std::size_t finished = 0;
  for (auto it = m_pending.begin(); it != m_pending.end();) {
    pending_build &p = it->second;
    if (!p.job->done.load(std::memory_order_acquire)) {
      ++it;
      continue;
    }
    ++finished;
    m_lastBuildMs = p.job->ms;
    if (!p.job->error.empty() || p.job->result.rungs.empty()) {
      m_unbuilt.insert(it->first);
      m_warnings.push_back("LodPyramidCache: building the LOD pyramid of a " +
                           std::to_string(p.job->tris) + "-triangle mesh failed (" +
                           (p.job->error.empty() ? std::string("no rungs") : p.job->error) +
                           "); it draws at full detail only");
    } else {
      cvc::lod::mesh_pyramid &pyr = m_cache[it->first];
      pyr = std::move(p.job->result);
      // Rung 0 was built from a deep copy of `base`: hold `base` itself, which
      // shares the waiting nodes' storage, so appendRungs recognises it by
      // pointer and the cache keeps no second copy of the finest mesh.
      pyr.rungs[0] = p.base;
      for (const std::weak_ptr<LodGraphicsNode> &w : p.waiters)
        if (const std::shared_ptr<LodGraphicsNode> node = w.lock())
          node->appendRungs(pyr);
    }
    it = m_pending.erase(it);
  }
  return finished;
}

std::vector<std::string> LodPyramidCache::take_warnings() {
  std::vector<std::string> out;
  out.swap(m_warnings);
  return out;
}

} // namespace gl
} // namespace cvc
