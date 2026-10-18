/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The Jolt backend of cvc::physics.
//
// This file is compiled as its own target (cvc_physics_jolt, src/cvc/CMakeLists.txt)
// with Jolt::Jolt linked PRIVATE, so Jolt's JPH_* compile definitions reach only
// this translation unit, never libcvc's other sources or its consumers. It sees
// the std-only cvc/physics headers and nothing else of libcvc.
//
// Rules from docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md:
//  - Z up. Jolt's height fields are Y up in shape space; the body that carries one
//    is rotated so shape (x, y, z) lands on world (y, z, x), with rows along +y.
//  - One μ for every plant: friction is combined from cvc materials attached to
//    every shape and height-field cell, so the tire (P2) and a box read one table.
//  - Contact events are sorted by the facade, so worker threads never reorder them.
//  - Jolt is built without exceptions: nothing here throws across a Jolt callback.
//    Callbacks record into a buffer under a mutex and flag, never throw, on failure.

// clang-format off
#include <Jolt/Jolt.h> // first: every other Jolt header needs the configuration it sets
// clang-format on

#include "../backend_impl.h"
#include "jolt_material.h"

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/IssueReporting.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorder.h>
#include <Jolt/RegisterTypes.h>
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

namespace {

// --- process-wide Jolt setup --------------------------------------------------

void trace_impl(const char *fmt, ...) {
  std::va_list args;
  va_start(args, fmt);
  std::fputs("cvc::physics (Jolt): ", stderr);
  std::vfprintf(stderr, fmt, args);
  std::fputc('\n', stderr);
  va_end(args);
}

#ifdef JPH_ENABLE_ASSERTS
bool assert_impl(const char *expr, const char *msg, const char *file, JPH::uint line) {
  std::fprintf(stderr, "cvc::physics (Jolt): assertion %s failed at %s:%u: %s\n", expr, file,
               static_cast<unsigned>(line), msg ? msg : "");
  return false; // do not break into a debugger
}
#endif

// Jolt keeps its allocator hooks, its factory and its type registry in process
// globals. The library requires them, so they cannot be avoided, only scoped:
// every backend holds a jolt_runtime, the first one sets Jolt up and the last one
// tears it down, so nothing outlives the worlds that use it and there is no
// singleton of ours. If something else in the process set Jolt up first (an
// application using Jolt directly), that setup is used as is and left in place.
//
// The version check comes before RegisterTypes(), which aborts the process on a
// mismatch between the JPH_* options libJolt was built with and the ones this
// file sees; a library should refuse with an error instead.
class jolt_runtime {
public:
  jolt_runtime() {
    shared &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    if (s.users == 0) {
      if (JPH::Factory::sInstance != nullptr) {
        s.owned = false;
      } else {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = trace_impl;
        JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = assert_impl;)
        s.owned = true;
      }
      if (!JPH::VerifyJoltVersionID())
        throw std::runtime_error(
            "the Jolt library was built with different JPH_* options than libcvc was compiled "
            "against (JPH_VERSION_ID mismatch); rebuild libcvc against this Jolt");
      if (s.owned) {
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
      }
    }
    ++s.users;
  }

  ~jolt_runtime() {
    shared &s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    if (--s.users == 0 && s.owned) {
      JPH::UnregisterTypes();
      delete JPH::Factory::sInstance;
      JPH::Factory::sInstance = nullptr;
      s.owned = false;
    }
  }

  jolt_runtime(const jolt_runtime &) = delete;
  jolt_runtime &operator=(const jolt_runtime &) = delete;

private:
  // The use count of Jolt's globals: plain data guarding them, not an instance of
  // anything.
  struct shared {
    std::mutex mutex;
    int users = 0;
    bool owned = false;
  };
  static shared &state() {
    static shared s;
    return s;
  }
};

// --- layers ---------------------------------------------------------------------

namespace layers {
constexpr JPH::ObjectLayer non_moving = 0;
constexpr JPH::ObjectLayer moving = 1;
constexpr JPH::uint count = 2;
} // namespace layers

class object_pair_filter final : public JPH::ObjectLayerPairFilter {
public:
  bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
    return a == layers::moving || b == layers::moving; // static against static never collides
  }
};

class broad_phase_layers final : public JPH::BroadPhaseLayerInterface {
public:
  JPH::uint GetNumBroadPhaseLayers() const override { return layers::count; }
  JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
    return JPH::BroadPhaseLayer(static_cast<JPH::BroadPhaseLayer::Type>(layer));
  }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
    return layer == JPH::BroadPhaseLayer(layers::non_moving) ? "non_moving" : "moving";
  }
#endif
};

class object_vs_broad_phase_filter final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
  bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer bp) const override {
    return layer == layers::moving || bp == JPH::BroadPhaseLayer(layers::moving);
  }
};

// --- materials ------------------------------------------------------------------

// cvc_material (jolt_material.h) is a cvc material as Jolt sees it. Every shape
// and height-field cell this backend creates carries one, so the combine functions
// below find a cvc material on both sides of every contact.

const cvc_material *as_cvc(const JPH::PhysicsMaterial *m) {
  if (m == nullptr || m == JPH::PhysicsMaterial::sDefault.GetPtr())
    return nullptr;
  return static_cast<const cvc_material *>(m);
}

material_id material_of(const JPH::Body &b, const JPH::SubShapeID &s) {
  const cvc_material *m = as_cvc(b.GetShape()->GetMaterial(s));
  return m ? m->id : material_id(0);
}

template <combine_rule R> float combine_values(float a, float b) {
  switch (R) {
  case combine_rule::product:
    return a * b;
  case combine_rule::min:
    return std::min(a, b);
  case combine_rule::max:
    return std::max(a, b);
  case combine_rule::geometric_mean:
    return std::sqrt(a * b);
  case combine_rule::average:
    return 0.5f * (a + b);
  }
  return a * b;
}

// Material 0's values stand in for a shape without a cvc material (none should
// exist; this keeps a stray Jolt default material neutral).
template <combine_rule R, bool Friction>
float combine(const JPH::Body &b1, const JPH::SubShapeID &s1, const JPH::Body &b2,
              const JPH::SubShapeID &s2) {
  const cvc_material *m1 = as_cvc(b1.GetShape()->GetMaterial(s1));
  const cvc_material *m2 = as_cvc(b2.GetShape()->GetMaterial(s2));
  const float neutral = Friction ? 1.0f : 0.0f;
  const float a = m1 ? (Friction ? m1->friction : m1->restitution) : neutral;
  const float b = m2 ? (Friction ? m2->friction : m2->restitution) : neutral;
  return combine_values<R>(a, b);
}

template <bool Friction>
JPH::ContactConstraintManager::CombineFunction pick_combine(combine_rule r) {
  switch (r) {
  case combine_rule::product:
    return &combine<combine_rule::product, Friction>;
  case combine_rule::min:
    return &combine<combine_rule::min, Friction>;
  case combine_rule::max:
    return &combine<combine_rule::max, Friction>;
  case combine_rule::geometric_mean:
    return &combine<combine_rule::geometric_mean, Friction>;
  case combine_rule::average:
    return &combine<combine_rule::average, Friction>;
  }
  return &combine<combine_rule::product, Friction>;
}

// --- contacts -------------------------------------------------------------------

// Records contact events from Jolt's worker threads. Never throws: an allocation
// failure sets `overflow` and drops the event.
class contact_collector final : public JPH::ContactListener {
public:
  contact_reporting mode = contact_reporting::added_removed;
  vec3 origin;

  void begin(const vec3 &o) {
    origin = o;
    events.clear();
    removed.clear();
    overflow = false;
  }

  void OnContactAdded(const JPH::Body &b1, const JPH::Body &b2, const JPH::ContactManifold &m,
                      JPH::ContactSettings &s) override {
    if (mode != contact_reporting::none)
      record(contact_event::kind::added, b1, b2, m, s);
  }

  void OnContactPersisted(const JPH::Body &b1, const JPH::Body &b2, const JPH::ContactManifold &m,
                          JPH::ContactSettings &s) override {
    if (mode == contact_reporting::all)
      record(contact_event::kind::persisted, b1, b2, m, s);
  }

  void OnContactRemoved(const JPH::SubShapeIDPair &p) override {
    if (mode == contact_reporting::none)
      return;
    std::lock_guard<std::mutex> lock(mutex);
    try {
      removed.push_back(p);
    } catch (...) {
      overflow = true;
    }
  }

  std::mutex mutex;
  std::vector<contact_event> events;
  std::vector<JPH::SubShapeIDPair> removed; // materials are looked up after the step
  bool overflow = false;

private:
  void record(contact_event::kind k, const JPH::Body &b1, const JPH::Body &b2,
              const JPH::ContactManifold &m, const JPH::ContactSettings &s) {
    contact_event e;
    e.type = k;
    e.body_a.value = b1.GetID().GetIndexAndSequenceNumber();
    e.body_b.value = b2.GetID().GetIndexAndSequenceNumber();
    e.sub_shape_a = m.mSubShapeID1.GetValue();
    e.sub_shape_b = m.mSubShapeID2.GetValue();
    e.material_a = material_of(b1, m.mSubShapeID1);
    e.material_b = material_of(b2, m.mSubShapeID2);
    e.friction = s.mCombinedFriction;
    e.restitution = s.mCombinedRestitution;
    // The midpoint of both surfaces' contact points, so swapping a and b below
    // leaves it unchanged.
    JPH::Vec3 sum = JPH::Vec3::sZero();
    const JPH::uint n1 = m.mRelativeContactPointsOn1.size(),
                    n2 = m.mRelativeContactPointsOn2.size();
    for (JPH::uint i = 0; i < n1; ++i)
      sum += m.mRelativeContactPointsOn1[i];
    for (JPH::uint i = 0; i < n2; ++i)
      sum += m.mRelativeContactPointsOn2[i];
    const JPH::Vec3 rel = (n1 + n2) > 0 ? sum / static_cast<float>(n1 + n2) : JPH::Vec3::sZero();
    e.point = {origin.x + static_cast<double>(m.mBaseOffset.GetX()) + rel.GetX(),
               origin.y + static_cast<double>(m.mBaseOffset.GetY()) + rel.GetY(),
               origin.z + static_cast<double>(m.mBaseOffset.GetZ()) + rel.GetZ()};
    // Jolt's normal moves body 2 out of body 1: it points from 1 toward 2.
    e.normal = {m.mWorldSpaceNormal.GetX(), m.mWorldSpaceNormal.GetY(), m.mWorldSpaceNormal.GetZ()};
    e.depth = m.mPenetrationDepth;
    if (e.body_b < e.body_a) {
      std::swap(e.body_a, e.body_b);
      std::swap(e.sub_shape_a, e.sub_shape_b);
      std::swap(e.material_a, e.material_b);
      e.normal = e.normal * -1.0;
    }
    std::lock_guard<std::mutex> lock(mutex);
    try {
      events.push_back(e);
    } catch (...) {
      overflow = true;
    }
  }
};

// --- saved state ----------------------------------------------------------------

// Jolt's state as bytes. Not JPH::StateRecorderImpl: that keeps a std::stringstream
// behind inline accessors, so a libcvc built against a different C++ runtime than
// libJolt (an MSVC Debug libcvc on a Release Jolt.dll) would disagree with Jolt
// about its layout. Here every byte is stored by this translation unit. Jolt has
// no exceptions, so a failed allocation is recorded, never thrown through it.
class byte_recorder final : public JPH::StateRecorder {
public:
  void WriteBytes(const void *data, size_t n) override {
    try {
      bytes.append(static_cast<const char *>(data), n);
    } catch (...) {
      failed = true;
    }
  }
  void ReadBytes(void *data, size_t n) override {
    if (n > bytes.size() - pos) {
      std::memset(data, 0, n);
      failed = true;
      return;
    }
    std::memcpy(data, bytes.data() + pos, n);
    pos += n;
  }
  bool IsEOF() const override { return pos >= bytes.size(); }
  bool IsFailed() const override { return failed; }

  std::string bytes;
  size_t pos = 0;
  bool failed = false;
};

// --- shapes ---------------------------------------------------------------------

// The rotation that carries a Jolt height field (Y up in shape space) into the
// world (Z up): shape x -> world y, shape y -> world z, shape z -> world x. It is
// the 120-degree turn about (1, 1, 1), whose quaternion is exact in float.
const JPH::Quat kHeightfieldRotation(0.5f, 0.5f, 0.5f, 0.5f);

JPH::Vec3 to_jolt(const vec3 &v) {
  return JPH::Vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}
JPH::Quat to_jolt(const quat &q) {
  return JPH::Quat(static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z),
                   static_cast<float>(q.w))
      .Normalized();
}
vec3 from_jolt(JPH::Vec3Arg v) { return {v.GetX(), v.GetY(), v.GetZ()}; }
quat from_jolt(JPH::QuatArg q) { return {q.GetW(), q.GetX(), q.GetY(), q.GetZ()}; }

std::string jolt_error(const JPH::ShapeSettings::ShapeResult &r, const char *what) {
  return std::string(what) + ": " + r.GetError().c_str();
}

struct built_shape {
  JPH::ShapeRefC shape;
  bool heightfield = false;
};

// Builds the Jolt shape for a cvc shape. `mats` maps material ids to Jolt
// materials; `body_material` is the body's own.
struct shape_builder {
  const std::vector<JPH::Ref<cvc_material>> &mats;
  material_id body_material;
  double density;

  const JPH::PhysicsMaterial *mat(material_id id) const { return mats.at(id).GetPtr(); }

  JPH::ShapeRefC create(const JPH::ShapeSettings &s, const char *what) const {
    const JPH::ShapeSettings::ShapeResult r = s.Create();
    if (r.HasError())
      throw std::invalid_argument(jolt_error(r, what));
    return r.Get();
  }

  built_shape operator()(const sphere &s) const {
    JPH::SphereShapeSettings settings(static_cast<float>(s.radius), mat(body_material));
    settings.SetDensity(static_cast<float>(density));
    return {create(settings, "sphere"), false};
  }

  built_shape operator()(const box &b) const {
    const JPH::Vec3 half = to_jolt(b.half_extents);
    const float radius = std::min(JPH::cDefaultConvexRadius, 0.5f * half.ReduceMin());
    JPH::BoxShapeSettings settings(half, radius, mat(body_material));
    settings.SetDensity(static_cast<float>(density));
    return {create(settings, "box"), false};
  }

  built_shape operator()(const convex_hull &h) const {
    JPH::Array<JPH::Vec3> points;
    points.reserve(h.points.size());
    for (const vec3 &p : h.points)
      points.push_back(to_jolt(p));
    JPH::ConvexHullShapeSettings settings(points, JPH::cDefaultConvexRadius, mat(body_material));
    settings.SetDensity(static_cast<float>(density));
    return {create(settings, "convex_hull"), false};
  }

  built_shape operator()(const mesh_data &m) const {
    JPH::VertexList vertices;
    vertices.reserve(m.vertices.size());
    for (const vec3 &v : m.vertices)
      vertices.push_back(
          JPH::Float3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)));
    // A per-mesh palette of the materials the triangles use.
    JPH::PhysicsMaterialList palette;
    std::map<material_id, JPH::uint32> slot;
    auto slot_of = [&](material_id id) {
      auto it = slot.find(id);
      if (it != slot.end())
        return it->second;
      const JPH::uint32 s = static_cast<JPH::uint32>(palette.size());
      palette.push_back(mat(id));
      slot.emplace(id, s);
      return s;
    };
    JPH::IndexedTriangleList triangles;
    const size_t n = m.indices.size() / 3;
    triangles.reserve(n);
    for (size_t t = 0; t < n; ++t) {
      const material_id id = m.triangle_materials.empty() ? body_material : m.triangle_materials[t];
      triangles.push_back(JPH::IndexedTriangle(m.indices[3 * t], m.indices[3 * t + 1],
                                               m.indices[3 * t + 2], slot_of(id)));
    }
    JPH::MeshShapeSettings settings(std::move(vertices), std::move(triangles), std::move(palette));
    return {create(settings, "mesh"), false};
  }

  built_shape operator()(const terrain_data &h) const {
    // Jolt wants a square grid whose side is a multiple of the block size (and at
    // least two blocks). Pad with holes.
    const JPH::uint block = 2;
    JPH::uint n = static_cast<JPH::uint>(std::max(h.rows, h.cols));
    n = std::max<JPH::uint>(2 * block, (n + block - 1) / block * block);
    // Shape sample (x, y) is at shape position (x * dy, height, y * dx), so x is the
    // row and y the column; Jolt stores sample (x, y) at index y * n + x.
    std::vector<float> samples(static_cast<size_t>(n) * n,
                               JPH::HeightFieldShapeConstants::cNoCollisionValue);
    for (int r = 0; r < h.rows; ++r)
      for (int c = 0; c < h.cols; ++c) {
        const float z = h.heights[static_cast<size_t>(r) * h.cols + c];
        if (!std::isnan(z))
          samples[static_cast<size_t>(c) * n + r] = z;
      }
    std::vector<JPH::uint8> cells(static_cast<size_t>(n - 1) * (n - 1), 0);
    if (!h.cells.empty())
      for (int r = 0; r + 1 < h.rows; ++r)
        for (int c = 0; c + 1 < h.cols; ++c)
          cells[static_cast<size_t>(c) * (n - 1) + r] =
              h.cells[static_cast<size_t>(r) * (h.cols - 1) + c];
    JPH::PhysicsMaterialList palette;
    for (material_id id : h.palette)
      palette.push_back(mat(id));
    JPH::HeightFieldShapeSettings settings(
        samples.data(), JPH::Vec3::sZero(),
        JPH::Vec3(static_cast<float>(h.dy), 1.0f, static_cast<float>(h.dx)), n, cells.data(),
        palette);
    settings.mBlockSize = block;
    settings.mBitsPerSample = std::min<JPH::uint32>(
        JPH::HeightFieldShapeConstants::cMaxBitsPerSample,
        settings.CalculateBitsPerSampleForError(static_cast<float>(h.max_error)));
    JPH::ShapeRefC shape = create(settings, "terrain");

    // The compression is lossy: check every sample against max_error rather than
    // trusting the bit count, and refuse a field Jolt cannot hold that closely.
    const auto *hf = static_cast<const JPH::HeightFieldShape *>(shape.GetPtr());
    double worst = 0.0;
    for (int r = 0; r < h.rows; ++r)
      for (int c = 0; c < h.cols; ++c) {
        const float z = h.heights[static_cast<size_t>(r) * h.cols + c];
        if (std::isnan(z))
          continue;
        if (hf->IsNoCollision(static_cast<JPH::uint>(r), static_cast<JPH::uint>(c)))
          throw std::invalid_argument("terrain: a sample became a hole in the engine");
        const double got =
            hf->GetPosition(static_cast<JPH::uint>(r), static_cast<JPH::uint>(c)).GetY();
        worst = std::max(worst, std::fabs(got - static_cast<double>(z)));
      }
    if (worst > h.max_error)
      throw std::invalid_argument("terrain: the engine stores heights within " +
                                  std::to_string(worst) + " m here, more than max_error (" +
                                  std::to_string(h.max_error) +
                                  " m); raise max_error or smooth the terrain");
    return {shape, true};
  }
};

// --- the backend ----------------------------------------------------------------

class jolt_backend final : public world_backend {
public:
  explicit jolt_backend(const world_config &cfg) : _cfg(cfg), _origin(cfg.origin) {
#ifndef JPH_CROSS_PLATFORM_DETERMINISTIC
    if (cfg.determinism == determinism::cross_platform)
      throw std::runtime_error("this Jolt build is not cross-platform deterministic");
#endif
    _temp =
        std::make_unique<JPH::TempAllocatorImpl>(static_cast<JPH::uint>(cfg.temp_allocator_bytes));
    if (cfg.threads > 0) {
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
      throw std::runtime_error("worker threads are not available in a wasm build without pthreads");
#else
      _jobs = std::make_unique<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs,
                                                         JPH::cMaxPhysicsBarriers, cfg.threads);
#endif
    } else {
      _jobs = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    }
    _ps = std::make_unique<JPH::PhysicsSystem>();
    _ps->Init(cfg.max_bodies, 0, cfg.max_body_pairs, cfg.max_contact_constraints, _bp_layers,
              _object_vs_bp, _object_pairs);
    _ps->SetContactListener(&_contacts);
    reconfigure(cfg);
  }

  // The settings that may change between steps (world::reconfigure).
  void reconfigure(const world_config &cfg) override {
    _ps->SetGravity(to_jolt(cfg.gravity));
    JPH::PhysicsSettings settings = _ps->GetPhysicsSettings();
    settings.mNumVelocitySteps = static_cast<JPH::uint>(cfg.velocity_steps);
    settings.mNumPositionSteps = static_cast<JPH::uint>(cfg.position_steps);
    settings.mPenetrationSlop = static_cast<float>(cfg.penetration_slop);
    settings.mDeterministicSimulation = true;
    _ps->SetPhysicsSettings(settings);
    _ps->SetCombineFriction(pick_combine<true>(cfg.friction));
    _ps->SetCombineRestitution(pick_combine<false>(cfg.restitution));
    _contacts.mode = cfg.contacts;
    _cfg.gravity = cfg.gravity;
    _cfg.collision_steps = cfg.collision_steps;
    _cfg.velocity_steps = cfg.velocity_steps;
    _cfg.position_steps = cfg.position_steps;
    _cfg.penetration_slop = cfg.penetration_slop;
    _cfg.friction = cfg.friction;
    _cfg.restitution = cfg.restitution;
    _cfg.contacts = cfg.contacts;
  }

  ~jolt_backend() override {
    // Bodies go before the system; materials (refcounted) after both.
    JPH::BodyIDVector ids;
    _ps->GetBodies(ids);
    JPH::BodyInterface &bi = _ps->GetBodyInterface();
    for (const JPH::BodyID &id : ids) {
      bi.RemoveBody(id);
      bi.DestroyBody(id);
    }
    _ps.reset();
  }

  void add_material(material_id id, const material &m) override {
    if (_mats.size() <= id)
      _mats.resize(static_cast<size_t>(id) + 1);
    _mats[id] = new cvc_material(id, m.friction, m.restitution);
  }

  // Between steps only (the world is driven from one thread), so the combine
  // functions never read a value mid-change.
  void set_material(material_id id, const material &m) override {
    _mats.at(id)->friction = m.friction;
    _mats.at(id)->restitution = m.restitution;
  }

  body_id add_body(const body_desc &d, const engine_shape &shape) override {
    const built_shape s = std::visit(shape_builder{_mats, d.material, d.density}, shape);
    vec3 position = d.position;
    JPH::Quat rotation = to_jolt(d.rotation);
    if (s.heightfield) {
      position = std::get<terrain_data>(shape).origin;
      rotation = kHeightfieldRotation;
    }
    const JPH::EMotionType motion = d.motion == motion_type::static_ ? JPH::EMotionType::Static
                                    : d.motion == motion_type::kinematic
                                        ? JPH::EMotionType::Kinematic
                                        : JPH::EMotionType::Dynamic;
    const JPH::ObjectLayer layer =
        d.motion == motion_type::static_ ? layers::non_moving : layers::moving;
    JPH::BodyCreationSettings bcs(s.shape, to_engine(position), rotation, motion, layer);
    if (d.motion != motion_type::static_) {
      bcs.mLinearVelocity = to_jolt(d.linear_velocity);
      bcs.mAngularVelocity = to_jolt(d.angular_velocity);
    }
    bcs.mLinearDamping = static_cast<float>(d.linear_damping);
    bcs.mAngularDamping = static_cast<float>(d.angular_damping);
    bcs.mAllowSleeping = d.allow_sleeping;
    bcs.mUserData = d.user_data;
    if (d.motion == motion_type::dynamic && d.mass > 0.0) {
      bcs.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
      bcs.mMassPropertiesOverride.mMass = static_cast<float>(d.mass);
    }
    const JPH::EActivation activation = d.motion == motion_type::static_
                                            ? JPH::EActivation::DontActivate
                                            : JPH::EActivation::Activate;
    const JPH::BodyID id = _ps->GetBodyInterface().CreateAndAddBody(bcs, activation);
    if (id.IsInvalid())
      throw std::length_error("the engine is out of bodies (world_config::max_bodies)");
    _added_since_step = true;
    body_id out;
    out.value = id.GetIndexAndSequenceNumber();
    return out;
  }

  void remove_body(body_id id) override {
    const JPH::BodyID jid = alive(id);
    JPH::BodyInterface &bi = _ps->GetBodyInterface();
    bi.RemoveBody(jid);
    bi.DestroyBody(jid);
  }

  bool contains(body_id id) const override {
    if (!id.valid())
      return false;
    JPH::BodyLockRead lock(_ps->GetBodyLockInterface(), JPH::BodyID(id.value));
    return lock.Succeeded();
  }

  std::size_t body_count() const override { return _ps->GetNumBodies(); }

  body_state state(body_id id) const override {
    JPH::BodyLockRead lock(_ps->GetBodyLockInterface(), checked(id));
    if (!lock.Succeeded())
      throw std::invalid_argument("unknown or removed body id " + std::to_string(id.value));
    const JPH::Body &b = lock.GetBody();
    body_state s;
    s.position = from_engine(b.GetPosition());
    // A height field's body carries the Y-up-to-Z-up turn; in world terms it is
    // unrotated.
    s.rotation = is_heightfield(b) ? quat{} : from_jolt(b.GetRotation());
    s.linear_velocity = from_jolt(b.GetLinearVelocity());
    s.angular_velocity = from_jolt(b.GetAngularVelocity());
    s.active = b.IsActive();
    return s;
  }

  void set_state(body_id id, const body_state &s) override {
    const JPH::BodyID jid = alive(id);
    JPH::BodyInterface &bi = _ps->GetBodyInterface();
    const bool hf = [&] {
      JPH::BodyLockRead lock(_ps->GetBodyLockInterface(), jid);
      return is_heightfield(lock.GetBody());
    }();
    const JPH::Quat q = hf ? kHeightfieldRotation : to_jolt(s.rotation);
    if (bi.GetMotionType(jid) == JPH::EMotionType::Static)
      bi.SetPositionAndRotation(jid, to_engine(s.position), q, JPH::EActivation::DontActivate);
    else
      bi.SetPositionRotationAndVelocity(jid, to_engine(s.position), q, to_jolt(s.linear_velocity),
                                        to_jolt(s.angular_velocity));
  }

  void add_force(body_id id, const vec3 &f) override {
    _ps->GetBodyInterface().AddForce(alive(id), to_jolt(f));
  }

  void add_impulse(body_id id, const vec3 &j) override {
    _ps->GetBodyInterface().AddImpulse(alive(id), to_jolt(j));
  }

  bool shape_interior(body_id id, const std::vector<vec3> &points,
                      std::vector<std::uint8_t> &inside, double &volume,
                      vec3 &center_of_mass) const override {
    JPH::BodyLockRead lock(_ps->GetBodyLockInterface(), checked(id));
    if (!lock.Succeeded())
      throw std::invalid_argument("unknown or removed body id " + std::to_string(id.value));
    const JPH::Shape *shape = lock.GetBody().GetShape();
    // Body-local coordinates are about the body's origin; CollidePoint wants them
    // about the shape's centre of mass.
    const JPH::Vec3 com = shape->GetCenterOfMass();
    center_of_mass = from_jolt(com);
    volume = shape->GetVolume();
    inside.assign(points.size(), 0);
    for (size_t i = 0; i < points.size(); ++i) {
      JPH::AnyHitCollisionCollector<JPH::CollidePointCollector> hit;
      shape->CollidePoint(to_jolt(points[i]) - com, JPH::SubShapeIDCreator(), hit);
      inside[i] = hit.HadHit() ? 1 : 0;
    }
    return true;
  }

  void add_field_load(body_id id, const vec3 &f, const vec3 &t) override {
    _ps->GetBodyInterface().AddForceAndTorque(alive(id), to_jolt(f), to_jolt(t),
                                              JPH::EActivation::DontActivate);
  }

  void wake(body_id id) override {
    const JPH::BodyID jid = alive(id);
    JPH::BodyInterface &bi = _ps->GetBodyInterface();
    if (bi.GetMotionType(jid) == JPH::EMotionType::Dynamic)
      bi.ActivateBody(jid);
  }

  void move_kinematic(body_id id, const vec3 &p, const quat &q) override {
    const JPH::BodyID jid = alive(id);
    JPH::BodyInterface &bi = _ps->GetBodyInterface();
    if (bi.GetMotionType(jid) != JPH::EMotionType::Kinematic)
      throw std::invalid_argument("move_kinematic: the body is not kinematic");
    bi.MoveKinematic(jid, to_engine(p), to_jolt(q), static_cast<float>(_cfg.fixed_dt));
  }

  void step(std::vector<contact_event> &out, step_report &report) override {
    if (_added_since_step) {
      _ps->OptimizeBroadPhase();
      _added_since_step = false;
    }
    _contacts.begin(_origin);
    const JPH::EPhysicsUpdateError err = _ps->Update(
        static_cast<float>(_cfg.fixed_dt), _cfg.collision_steps, _temp.get(), _jobs.get());
    report.saturated = err != JPH::EPhysicsUpdateError::None || _contacts.overflow;
    report.active_bodies = _ps->GetNumActiveBodies(JPH::EBodyType::RigidBody);
    out.insert(out.end(), _contacts.events.begin(), _contacts.events.end());
    // A removed contact names its sub-shapes; look their materials up now that
    // the step is over and the bodies can be locked (a removed body reads as 0).
    for (const JPH::SubShapeIDPair &p : _contacts.removed) {
      contact_event e;
      e.type = contact_event::kind::removed;
      e.body_a.value = p.GetBody1ID().GetIndexAndSequenceNumber();
      e.body_b.value = p.GetBody2ID().GetIndexAndSequenceNumber();
      e.sub_shape_a = p.GetSubShapeID1().GetValue();
      e.sub_shape_b = p.GetSubShapeID2().GetValue();
      e.material_a = material_or_zero(p.GetBody1ID(), p.GetSubShapeID1());
      e.material_b = material_or_zero(p.GetBody2ID(), p.GetSubShapeID2());
      if (e.body_b < e.body_a) {
        std::swap(e.body_a, e.body_b);
        std::swap(e.sub_shape_a, e.sub_shape_b);
        std::swap(e.material_a, e.material_b);
      }
      out.push_back(e);
    }
  }

  std::optional<ray_hit> raycast(const vec3 &from, const vec3 &to) const override {
    const JPH::RRayCast ray(to_engine(from), to_jolt(to - from));
    JPH::RayCastResult hit;
    if (!_ps->GetNarrowPhaseQuery().CastRay(ray, hit))
      return std::nullopt;
    JPH::BodyLockRead lock(_ps->GetBodyLockInterface(), hit.mBodyID);
    if (!lock.Succeeded())
      return std::nullopt;
    const JPH::Body &b = lock.GetBody();
    const JPH::RVec3 p = ray.GetPointOnRay(hit.mFraction);
    ray_hit h;
    h.body.value = hit.mBodyID.GetIndexAndSequenceNumber();
    h.fraction = hit.mFraction;
    h.point = from_engine(p);
    h.normal = from_jolt(b.GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, p));
    h.material = material_of(b, hit.mSubShapeID2);
    h.sub_shape = hit.mSubShapeID2.GetValue();
    return h;
  }

  std::string save_engine() const override {
    byte_recorder rec;
    _ps->SaveState(rec);
    if (rec.failed)
      throw std::bad_alloc();
    return std::move(rec.bytes);
  }

  void restore_engine(const std::string &bytes, const vec3 &origin) override {
    byte_recorder rec;
    rec.bytes = bytes;
    if (!_ps->RestoreState(rec) || rec.IsFailed() || !rec.IsEOF())
      throw std::runtime_error("the engine refused the saved state (a different body set?)");
    _origin = origin;
  }

  void rebase(const vec3 &new_origin) override {
    JPH::BodyIDVector ids;
    _ps->GetBodies(ids);
    JPH::BodyInterface &bi = _ps->GetBodyInterface();
    for (const JPH::BodyID &id : ids) {
      const vec3 world = from_engine(bi.GetPosition(id));
      bi.SetPosition(id, to_engine(world, new_origin), JPH::EActivation::DontActivate);
    }
    _origin = new_origin;
  }

private:
  JPH::RVec3 to_engine(const vec3 &p) const { return to_engine(p, _origin); }
  static JPH::RVec3 to_engine(const vec3 &p, const vec3 &origin) {
    return JPH::RVec3(static_cast<JPH::Real>(p.x - origin.x),
                      static_cast<JPH::Real>(p.y - origin.y),
                      static_cast<JPH::Real>(p.z - origin.z));
  }
  vec3 from_engine(JPH::RVec3Arg p) const {
    return {_origin.x + static_cast<double>(p.GetX()), _origin.y + static_cast<double>(p.GetY()),
            _origin.z + static_cast<double>(p.GetZ())};
  }

  static bool is_heightfield(const JPH::Body &b) {
    return b.GetShape()->GetSubType() == JPH::EShapeSubType::HeightField;
  }

  static JPH::BodyID checked(body_id id) {
    if (!id.valid())
      throw std::invalid_argument("invalid body id");
    return JPH::BodyID(id.value);
  }

  JPH::BodyID alive(body_id id) const {
    const JPH::BodyID jid = checked(id);
    JPH::BodyLockRead lock(_ps->GetBodyLockInterface(), jid);
    if (!lock.Succeeded())
      throw std::invalid_argument("unknown or removed body id " + std::to_string(id.value));
    return jid;
  }

  material_id material_or_zero(const JPH::BodyID &id, const JPH::SubShapeID &s) const {
    JPH::BodyLockRead lock(_ps->GetBodyLockInterface(), id);
    return lock.Succeeded() ? material_of(lock.GetBody(), s) : material_id(0);
  }

  jolt_runtime _runtime; // first: constructed before, and destroyed after, every Jolt object
  world_config _cfg;
  vec3 _origin;
  std::vector<JPH::Ref<cvc_material>> _mats;
  std::unique_ptr<JPH::TempAllocatorImpl> _temp;
  std::unique_ptr<JPH::JobSystem> _jobs;
  broad_phase_layers _bp_layers;
  object_vs_broad_phase_filter _object_vs_bp;
  object_pair_filter _object_pairs;
  contact_collector _contacts;
  std::unique_ptr<JPH::PhysicsSystem> _ps;
  bool _added_since_step = false;
};

} // namespace

std::unique_ptr<world_backend> make_jolt_backend(const world_config &cfg) {
  return std::make_unique<jolt_backend>(cfg);
}

std::uint32_t jolt_capabilities() {
  std::uint32_t caps = cap_dynamics | cap_collision | cap_heightfield | cap_mesh | cap_raycast |
                       cap_determinism_same_binary | cap_exact_save_restore;
#ifdef JPH_CROSS_PLATFORM_DETERMINISTIC
  caps |= cap_determinism_cross_platform;
#endif
#if !(defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__))
  caps |= cap_threads;
#endif
  return caps;
}

std::string jolt_version() {
  std::string v = "Jolt " + std::to_string(JPH_VERSION_MAJOR) + "." +
                  std::to_string(JPH_VERSION_MINOR) + "." + std::to_string(JPH_VERSION_PATCH);
#ifdef JPH_CROSS_PLATFORM_DETERMINISTIC
  v += " (cross-platform deterministic)";
#endif
#ifdef JPH_DOUBLE_PRECISION
  v += " (double precision)";
#endif
  return v;
}

} // namespace detail
} // namespace physics
} // namespace cvc
