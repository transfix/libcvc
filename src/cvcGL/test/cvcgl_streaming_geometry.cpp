/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_streaming_geometry -- the streaming-geometry building blocks:
//
//   * SceneGraph::postEventCoalesced: at most one apply per key per drain,
//     latest wins, queue position of the first post, posts during a drain.
//   * Mapper choice: Auto resolves exactly as GeometryNode's factory mapper.
//   * The GLES texture-buffer emulation path (texel row spans + their
//     glTexSubImage2D upload), compiled everywhere and checked here against a
//     real 2-D RGB32F texture.
//   * HeightFieldTexture::sample, the CPU twin of the draping shader.
//   * StreamingGeometryNode, on BOTH mapper families: partial uploads (node
//     stats and a GL-level counter), draw ranges and uniforms with no upload,
//     capacity overflow (throws), off-thread writes marshalled and coalesced
//     into one apply, RibbonNode capacity growth.
//
// The GL sections skip (rc 0) where nothing rasterises, unless
// CVC_REQUIRE_RENDER=1. Synthetic geometry only.

#include "streaming_test_util.h"

#include <atomic>
#include <cmath>
#include <cvc/gl/DrapedLinkNode.h>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/RibbonNode.h>
#include <cvc/gl/StreamingMappers.h>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vtkActor.h>
#include <vtkOpenGLLowMemoryPolyDataMapper.h>
#include <vtkOpenGLShaderProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkTextureObject.h>
#include <vtk_glad.h>

using namespace cvcgl_test;
using cvc::gl::DrapedLinkNode;
using cvc::gl::HeightFieldTexture;
using cvc::gl::RibbonNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::StreamingGeometryNode;
using cvc::gl::StreamingLayout;
using cvc::gl::StreamingMapperKind;

namespace {

// ── postEventCoalesced ──────────────────────────────────────────────────────
void testCoalescedEvents(cvc::app &app) {
  std::printf("SceneGraph::postEventCoalesced\n");
  SceneGraph sg(app, "coalesce");
  std::vector<std::string> log;
  int keyA = 0, keyB = 0;
  sg.checkAndResetRenderNeeded();
  std::thread producer([&]() {
    sg.postEvent([&log]() { log.push_back("E1"); });
    sg.postEventCoalesced(&keyA, [&log]() { log.push_back("A1"); });
    sg.postEvent([&log]() { log.push_back("E2"); });
    sg.postEventCoalesced(&keyA, [&log]() { log.push_back("A2"); });
    sg.postEventCoalesced(&keyB, [&log]() { log.push_back("B1"); });
    sg.postEventCoalesced(&keyA, [&log]() { log.push_back("A3"); });
    sg.postEventCoalesced(&keyB, [&log]() { log.push_back("B2"); });
  });
  producer.join();
  check(sg.checkAndResetRenderNeeded(), "a coalesced post requests a render");
  sg.processEvents();
  std::string got;
  for (auto &s : log)
    got += s + " ";
  check(got == "E1 A3 E2 B2 ", "one apply per key, latest wins, at the first post's position", got);

  // Posts made while the queue drains: the slot that already ran queues a new
  // one (next drain); a slot still waiting runs the newest callback.
  log.clear();
  sg.postEventCoalesced(&keyA, [&]() {
    log.push_back("A4");
    sg.postEventCoalesced(&keyA, [&log]() { log.push_back("A5"); });
    sg.postEventCoalesced(&keyB, [&log]() { log.push_back("B4"); });
  });
  sg.postEventCoalesced(&keyB, [&log]() { log.push_back("B3"); });
  sg.processEvents();
  got.clear();
  for (auto &s : log)
    got += s + " ";
  check(got == "A4 B4 ", "a re-post of a key that already ran waits for the next drain", got);
  log.clear();
  sg.processEvents();
  check(log.size() == 1 && log[0] == "A5", "... and runs there, once");
  log.clear();
  sg.processEvents();
  check(log.empty(), "nothing left behind");
}

// ── a node's apply key is its own ───────────────────────────────────────────
// Anyone else coalescing "per node" (the SceneGraph doc once suggested the
// node's `this`) used to replace the node's own apply, which then never ran
// again: every later off-thread write was staged and never applied.
void testApplyKeyIsPrivate(cvc::app &app) {
  std::printf("apply key: a caller coalescing by the node pointer does not evict the apply\n");
  SceneGraph sg(app, "applykey");
  auto rib = sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>(
      "track", 64, 1.0f, cvc::bounding_box(-100, -100, -1, 100, 100, 1),
      StreamingMapperKind::Classic);
  const std::uint64_t a0 = rib->streamStats().applies;
  int userRuns = 0;
  std::thread([&]() {
    rib->append(0, 0, 0);
    sg.postEventCoalesced(rib.get(), [&userRuns]() { ++userRuns; });
  }).join();
  sg.processEvents();
  check(userRuns == 1 && rib->streamStats().applies == a0 + 1,
        "both ran: the caller's callback and the node's apply",
        "user " + std::to_string(userRuns) + ", applies +" +
            std::to_string(rib->streamStats().applies - a0));
  for (int i = 1; i <= 5; ++i) {
    std::thread([&]() { rib->append(static_cast<float>(i), 0, 0); }).join();
    sg.processEvents();
  }
  check(rib->streamStats().applies == a0 + 6,
        "and every later off-thread append is still applied (one apply per drain)",
        "applies +" + std::to_string(rib->streamStats().applies - a0));
}

// ── unattached nodes, and slots their scene never ran ──────────────────────
void testUnattachedAndDroppedSlot(cvc::app &app) {
  std::printf("unattached writes stage only; a dead scene's slot cannot strand them\n");
  StreamingLayout l;
  l.capacity_points = 8;
  l.triangles = {{{0, 1, 2}}};
  l.reserved_bounds = cvc::bounding_box(-1, -1, -1, 1, 1, 1);
  l.mapper = StreamingMapperKind::Classic;
  auto node = std::make_shared<StreamingGeometryNode>(app, "stream_unattached.n", "n", l);
  std::thread([&]() {
    const float p[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    node->writePoints(0, p, 3);
    node->setDrawRange(0, 1);
  }).join();
  check(node->streamStats().writes == 1 && node->streamStats().applies == 0,
        "no scene: an off-thread write is staged, nothing applied on the producer thread");
  {
    SceneGraph sg(app, "stream_attach");
    sg.getGraphicsRoot()->addGraphicsChild(node);
    check(node->streamStats().applies == 1,
          "attaching applies everything staged, once, on the owner thread");
    sg.getGraphicsRoot()->removeGraphicsChild(node);
  }

  // A slot posted to a scene that is never pumped, then the node moves on: the
  // old one-slot-in-flight flag stayed set, and the node never posted again.
  auto sg1 = std::make_unique<SceneGraph>(app, "stream_unpumped");
  auto rib = sg1->getGraphicsRoot()->addGraphicsChild<RibbonNode>(
      "r", 16, 1.0f, cvc::bounding_box(-10, -10, -1, 10, 10, 1), StreamingMapperKind::Classic);
  const std::uint64_t a0 = rib->streamStats().applies;
  std::thread([&]() { rib->append(1, 2, 0); }).join(); // posted to sg1, which nobody pumps
  check(rib->streamStats().applies == a0, "posted to an unpumped scene: not applied yet");
  sg1->getGraphicsRoot()->removeGraphicsChild(rib);
  SceneGraph sg2(app, "stream_alive");
  sg2.getGraphicsRoot()->addGraphicsChild(rib);
  check(rib->streamStats().applies == a0 + 1,
        "moving it to a live scene applies what the first one never ran");
  std::thread([&]() { rib->append(3, 4, 0); }).join();
  sg2.processEvents();
  check(rib->streamStats().applies == a0 + 2 && rib->centerCount() == 2,
        "later off-thread writes still post and apply (no stuck 'already posted' flag)");
  sg1.reset(); // its dtor drains the stale slot
  check(rib->streamStats().applies == a0 + 2, "the stale slot finds nothing left to apply");
}

// ── a throwing callback strands nothing ────────────────────────────────────
// processEvents used to drop the rest of its batch when a callback threw,
// leaving their coalesced keys registered with no queued slot: every later post
// for those keys (a streaming node's applies) was swallowed for good.
void testThrowingCallback(cvc::app &app) {
  std::printf("processEvents: a throwing callback strands nothing\n");
  SceneGraph sg(app, "throwing");
  int key = 0;
  std::vector<std::string> log;
  auto joined = [&log]() {
    std::string s;
    for (auto &e : log)
      s += e + " ";
    return s;
  };
  sg.postEvent([]() { throw std::runtime_error("boom"); });
  sg.postEventCoalesced(&key, [&log]() { log.push_back("K1"); });
  sg.postEvent([&log]() { log.push_back("E"); });
  bool threw = false;
  try {
    sg.processEvents();
  } catch (const std::runtime_error &) {
    threw = true;
  }
  check(threw && log.empty(), "the exception propagates; nothing after it has run yet");
  sg.postEventCoalesced(&key, [&log]() { log.push_back("K2"); });
  sg.postEvent([&log]() { log.push_back("F"); });
  sg.processEvents();
  check(joined() == "K2 E F ",
        "the unrun events keep their slots and order; the key runs once, latest callback",
        joined());

  auto rib = sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>(
      "r", 16, 1.0f, cvc::bounding_box(-10, -10, -1, 10, 10, 1), StreamingMapperKind::Classic);
  const std::uint64_t a0 = rib->streamStats().applies;
  sg.postEvent([]() { throw std::runtime_error("boom"); });
  std::thread([&]() { rib->append(1, 2, 0); }).join();
  try {
    sg.processEvents();
  } catch (const std::runtime_error &) {
  }
  std::thread([&]() { rib->append(3, 4, 0); }).join();
  sg.processEvents();
  check(rib->streamStats().applies == a0 + 1 && rib->centerCount() == 2,
        "a streaming node's apply survives a throwing neighbour in the queue",
        "applies +" + std::to_string(rib->streamStats().applies - a0));
}

// ── a slot left in a scene the node moved out of ───────────────────────────
void testMovedNodeStaleSlot(cvc::app &app) {
  std::printf("a stale slot in the scene a node left does not apply it\n");
  SceneGraph from(app, "move_from"), to(app, "move_to");
  auto rib = from.getGraphicsRoot()->addGraphicsChild<RibbonNode>(
      "r", 16, 1.0f, cvc::bounding_box(-10, -10, -1, 10, 10, 1), StreamingMapperKind::Classic);
  const std::uint64_t a0 = rib->streamStats().applies;
  std::thread([&]() { rib->append(1, 2, 0); }).join(); // a slot in `from`
  // Moved by a thread that owns neither scene: the move posts its apply to `to`.
  std::thread([&]() {
    from.getGraphicsRoot()->removeGraphicsChild(rib);
    to.getGraphicsRoot()->addGraphicsChild(rib);
  }).join();
  from.processEvents();
  check(rib->streamStats().applies == a0,
        "the old scene's slot leaves a node that now belongs to another scene alone");
  to.processEvents();
  check(rib->streamStats().applies == a0 + 1 && rib->centerCount() == 1,
        "the scene it belongs to applies it");
}

// ── bounds: ribbons grow theirs, links derive theirs unless pinned ─────────
bool boxCovers(const cvc::bounding_box &b, double x, double y, double z) {
  return b.minx <= x && x <= b.maxx && b.miny <= y && y <= b.maxy && b.minz <= z && z <= b.maxz;
}

void testBounds(cvc::app &app) {
  std::printf("reserved bounds: ribbon growth, link pinning, height-field refresh\n");
  SceneGraph sg(app, "bounds");
  const cvc::bounding_box small(-10, -10, -1, 10, 10, 1);
  auto rib = sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>("track", 8, 1.5f, small,
                                                                StreamingMapperKind::Classic);
  for (int k = 0; k < 30; ++k)
    rib->append(10.0f * k, 0.5f * k, 2.0f);
  cvc::bounding_box b = rib->reservedBounds();
  check(boxCovers(b, 290 + 3.0, 14.5, 2.0) && boxCovers(b, -10, -10, -1),
        "appends past the box grow it (by the mitred width), never shrinking it",
        "maxx " + std::to_string(b.maxx));
  const double *vb = rib->prop()->GetBounds(); // what VTK culls by
  check(vb[1] >= 293.0 && vb[0] <= -10.0, "VTK culls by the grown box",
        std::to_string(vb[0]) + ".." + std::to_string(vb[1]));
  const float route[6] = {-400, 50, 0, -380, 60, 0};
  rib->assign(route, 2);
  b = rib->reservedBounds();
  check(boxCovers(b, -403, 50, 0) && boxCovers(b, 293, 14.5, 2.0), "assign grows it too");
  rib->setHalfWidth(20.0f);
  check(boxCovers(rib->reservedBounds(), -440, 50, 0), "setHalfWidth grows it to the new width");

  auto hf = std::make_shared<HeightFieldTexture>(8, 8, 0.0, 0.0, 10.0, 10.0); // flat
  auto link = sg.getGraphicsRoot()->addGraphicsChild<DrapedLinkNode>("link", hf, 8,
                                                                     StreamingMapperKind::Classic);
  link->setEndpoints(10, 10, 60, 10);
  const cvc::bounding_box derived = link->reservedBounds();
  const cvc::bounding_box mine(-1000, -1000, -1000, 1000, 1000, 1000);
  link->setReservedBounds(mine);
  link->setStyle(3.0f, 2.0f, 1, 0, 0);
  check(link->reservedBounds() == mine, "setStyle keeps a box the caller pinned");
  link->refreshReservedBounds();
  check(link->reservedBounds().maxx < 1000 && link->reservedBounds().maxz < 10,
        "refreshReservedBounds derives it again");
  hf->setHeights(std::vector<float>(64, 300.0f)); // terrain arrives after the link
  vb = link->prop()->GetBounds();                 // asks the mapper, as culling does
  check(vb[5] >= 300.0 && link->reservedBounds().maxz >= 300.0,
        "new heights re-derive the box the next time VTK asks (culled or not)",
        "maxz " + std::to_string(vb[5]));
  link->setReservedBounds(mine);
  hf->setHeights(std::vector<float>(64, 900.0f));
  vb = link->prop()->GetBounds();
  check(vb[5] == 1000.0 && link->reservedBounds() == mine, "... but leave a pinned box alone");
  (void)derived;
}

// ── internal shader replacements ───────────────────────────────────────────
std::string replacementText(cvc::gl::GraphicsNode &n, vtkShader::Type type,
                            const std::string &anchor) {
  auto *actor = vtkActor::SafeDownCast(n.prop());
  auto *sp = actor ? vtkOpenGLShaderProperty::SafeDownCast(actor->GetShaderProperty()) : nullptr;
  if (!sp)
    return "<no shader property>";
  for (const auto &kv : sp->GetAllShaderReplacements())
    if (kv.first.ShaderType == type && kv.first.OriginalValue == anchor && kv.first.ReplaceFirst)
      return kv.second.Replacement;
  return "";
}

void testInternalShaderReplacements(cvc::app &app) {
  std::printf("internal shader replacements survive the public replacement API\n");
  SceneGraph sg(app, "repl");
  auto hf = std::make_shared<HeightFieldTexture>(4, 4, 0.0, 0.0, 1.0, 1.0);
  auto link = sg.getGraphicsRoot()->addGraphicsChild<DrapedLinkNode>("link", hf, 4,
                                                                     StreamingMapperKind::Classic);
  const std::string anchor = "//VTK::Clip::Impl";
  const std::string drape = replacementText(*link, vtkShader::Vertex, anchor);
  check(drape.find("cvc_link_drape") != std::string::npos &&
            drape.find("#define vertexMC") < drape.find(anchor),
        "the drape (and its #define) is spliced in AHEAD of VTK's clip code");
  link->addVertexShaderReplacement(anchor, anchor + "\n  // caller-marker\n");
  const std::string both = replacementText(*link, vtkShader::Vertex, anchor);
  check(both.find("cvc_link_drape") != std::string::npos &&
            both.find("caller-marker") != std::string::npos &&
            both.find("#define vertexMC") < both.find("caller-marker"),
        "a caller's replacement on the same anchor composes after it");
  link->clearShaderReplacements();
  const std::string after = replacementText(*link, vtkShader::Vertex, anchor);
  check(after == drape, "clearShaderReplacements() removes the caller's, keeps the drape");
  check(replacementText(*link, vtkShader::Fragment, "//VTK::Color::Impl").find("cvcLinkColor") !=
            std::string::npos,
        "... and the link's fragment colour");

  auto plain = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::GeometryNode>("plain");
  plain->addVertexShaderReplacement(anchor, anchor + "\n  // plain-marker\n");
  check(replacementText(*plain, vtkShader::Vertex, anchor).find("plain-marker") !=
            std::string::npos,
        "a plain GeometryNode's replacement is installed as before");
  plain->clearShaderReplacements();
  check(replacementText(*plain, vtkShader::Vertex, anchor).empty(), "... and cleared as before");

  auto rib = sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>(
      "rib", 8, 1.0f, cvc::bounding_box(-1, -1, -1, 1, 1, 1), StreamingMapperKind::Classic);
  rib->clearShaderReplacements();
  check(replacementText(*rib, vtkShader::Fragment, "//VTK::UniformFlow::Impl")
                .find("cvcRibbonClipLo") != std::string::npos,
        "a ribbon keeps its fragment clip through clearShaderReplacements()");
}

// ── mapper selection ────────────────────────────────────────────────────────
void testMapperSelection(cvc::app &app) {
  std::printf("mapper selection\n");
  vtkSmartPointer<vtkPolyDataMapper> factory = vtkSmartPointer<vtkPolyDataMapper>::New();
  const StreamingMapperKind expect = factory->IsA("vtkOpenGLLowMemoryPolyDataMapper")
                                         ? StreamingMapperKind::LowMemory
                                         : StreamingMapperKind::Classic;
  check(cvc::gl::resolveStreamingMapperKind(StreamingMapperKind::Auto) == expect,
        "Auto resolves to the factory's family", factory->GetClassName());

  SceneGraph sg(app, "selection");
  for (StreamingMapperKind k :
       {StreamingMapperKind::Auto, StreamingMapperKind::Classic, StreamingMapperKind::LowMemory}) {
    StreamingLayout l;
    l.capacity_points = 3;
    l.triangles = {{{0, 1, 2}}};
    l.reserved_bounds = cvc::bounding_box(0, 0, 0, 1, 1, 1);
    l.mapper = k;
    auto n = sg.getGraphicsRoot()->addGraphicsChild<StreamingGeometryNode>(
        std::string("sel_") + kindName(k), l);
    const StreamingMapperKind want = k == StreamingMapperKind::Auto ? expect : k;
    auto *actor = vtkActor::SafeDownCast(n->prop());
    vtkMapper *m = actor ? actor->GetMapper() : nullptr;
    const bool family = m && (want == StreamingMapperKind::LowMemory
                                  ? m->IsA("vtkOpenGLLowMemoryPolyDataMapper") != 0
                                  : m->IsA("vtkOpenGLPolyDataMapper") != 0);
    check(n->mapperKind() == want && family &&
              cvc::gl::streamingCore(vtkPolyDataMapper::SafeDownCast(m)),
          std::string("requested ") + kindName(k) + " -> " + kindName(n->mapperKind()),
          m ? m->GetClassName() : "no mapper");
  }
}

// ── GLES emulation: texel row spans ─────────────────────────────────────────
void testTexelSpans() {
  std::printf("GLES texture-buffer emulation: row spans\n");
  using cvc::gl::TexelRowSpan;
  TexelRowSpan s[3];
  int n = cvc::gl::computeTexelRowSpans(7, 3, 25, s);
  check(n == 3 && s[0].x == 3 && s[0].y == 0 && s[0].width == 4 && s[0].rows == 1 &&
            s[0].first == 3 && s[1].x == 0 && s[1].y == 1 && s[1].width == 7 && s[1].rows == 2 &&
            s[1].first == 7 && s[2].x == 0 && s[2].y == 3 && s[2].width == 4 && s[2].rows == 1 &&
            s[2].first == 21,
        "[3, 25) in a 7-wide texture = head + 2 whole rows + tail");
  n = cvc::gl::computeTexelRowSpans(7, 0, 14, s);
  check(n == 1 && s[0].x == 0 && s[0].y == 0 && s[0].width == 7 && s[0].rows == 2,
        "row-aligned range = one call");
  n = cvc::gl::computeTexelRowSpans(7, 2, 5, s);
  check(n == 1 && s[0].x == 2 && s[0].width == 3 && s[0].rows == 1, "inside one row = one call");
  n = cvc::gl::computeTexelRowSpans(7, 5, 9, s);
  check(n == 2 && s[0].x == 5 && s[0].width == 2 && s[1].x == 0 && s[1].y == 1 && s[1].width == 2,
        "straddling a row edge = head + tail");
  check(cvc::gl::computeTexelRowSpans(7, 4, 4, s) == 0 &&
            cvc::gl::computeTexelRowSpans(0, 0, 4, s) == 0,
        "empty range / zero width = nothing");
  // Every element covered exactly once, for every range in a small texture.
  bool exact = true;
  for (vtkIdType w = 1; w <= 5 && exact; ++w)
    for (vtkIdType lo = 0; lo < 23 && exact; ++lo)
      for (vtkIdType hi = lo + 1; hi <= 23 && exact; ++hi) {
        std::vector<int> hit(23, 0);
        n = cvc::gl::computeTexelRowSpans(w, lo, hi, s);
        for (int k = 0; k < n; ++k)
          for (vtkIdType r = 0; r < s[k].rows; ++r)
            for (vtkIdType c = 0; c < s[k].width; ++c) {
              const vtkIdType i = (s[k].y + r) * w + s[k].x + c;
              exact = exact && i == s[k].first + r * w + c && i >= 0 && i < 23;
              if (i >= 0 && i < 23)
                ++hit[static_cast<size_t>(i)];
            }
        for (vtkIdType i = 0; i < 23; ++i)
          exact = exact && hit[static_cast<size_t>(i)] == ((i >= lo && i < hi) ? 1 : 0);
      }
  check(exact, "exhaustive: every range covers exactly its elements, once, in <= 3 spans");
}

// ── HeightFieldTexture::sample ──────────────────────────────────────────────
void testHeightSample() {
  std::printf("HeightFieldTexture::sample (CPU twin of the draping shader)\n");
  HeightFieldTexture hf(4, 3, 10.0, 20.0, 2.0, 5.0);
  std::vector<float> h(12);
  for (int j = 0; j < 3; ++j)
    for (int i = 0; i < 4; ++i)
      h[static_cast<size_t>(j) * 4 + i] = static_cast<float>(i + 10 * j);
  hf.setHeights(h);
  check(std::fabs(hf.sample(14.0, 25.0) - 12.0) < 1e-9, "exact at a sample (i=2, j=1)");
  check(std::fabs(hf.sample(11.0, 22.5) - 5.5) < 1e-9, "bilinear between samples");
  check(std::fabs(hf.sample(-100.0, -100.0) - 0.0) < 1e-9 &&
            std::fabs(hf.sample(1000.0, 1000.0) - 23.0) < 1e-9,
        "clamped to the edge outside the grid");
  const cvc::bounding_box e = hf.extent();
  check(e.minx == 10.0 && e.maxx == 16.0 && e.miny == 20.0 && e.maxy == 30.0 && e.minz == 0.0 &&
            e.maxz == 23.0,
        "extent: grid XY, height range Z");
  const float row[4] = {100, 100, 100, 100};
  hf.updateRows(1, 1, row);
  check(std::fabs(hf.sample(12.0, 25.0) - 100.0) < 1e-9 && hf.extent().maxz == 100.0,
        "updateRows patches the grid and widens the extent");
  bool threw = false;
  try {
    hf.updateRows(2, 2, row);
  } catch (const std::out_of_range &) {
    threw = true;
  }
  check(threw, "updateRows past the last row throws std::out_of_range");
  threw = false;
  try {
    hf.setHeights(std::vector<float>(5, 0.0f));
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "setHeights with the wrong count throws std::invalid_argument");
}

// ── GL: the emulation upload against a real 2-D texture ─────────────────────
void testTexelUploadGL(SceneRenderer &sr) {
  std::printf("GLES texture-buffer emulation: glTexSubImage2D against a real texture\n");
  auto *win = vtkOpenGLRenderWindow::SafeDownCast(sr.renderWindow());
  win->MakeCurrent();
  const int W = 7, H = 5, N = W * H;
  std::vector<float> base(3 * N), next(3 * N);
  for (int i = 0; i < 3 * N; ++i) {
    base[i] = static_cast<float>(i);
    next[i] = 1000.0f + static_cast<float>(i);
  }
  vtkSmartPointer<vtkTextureObject> tex = vtkSmartPointer<vtkTextureObject>::New();
  tex->SetContext(win);
  tex->SetInternalFormat(GL_RGB32F); // what VTK's emulation creates for xyz floats
  tex->SetFormat(GL_RGB);
  tex->SetDataType(GL_FLOAT);
  tex->Create2DFromRaw(W, H, 3, VTK_FLOAT, base.data());
  const GLCount before = glcountRead();
  const int calls = cvc::gl::uploadTexelRange(tex, win, next.data(), 3, 25);
  const GLCount d = glcountRead() - before;
  check(calls == 3 && d.v[K_TEXSUB] == 3 && d.v[K_TEXSUB_B] == 22 * 12 && d.createdNothing(),
        "elements [3, 25): 3 glTexSubImage2D, 264 B, nothing created", glcountStr(d));
  std::vector<float> back(3 * N, -1.0f);
  tex->Activate();
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_FLOAT, back.data());
  tex->Deactivate();
  bool ok = true;
  for (int i = 0; i < N; ++i)
    for (int c = 0; c < 3; ++c)
      ok = ok && back[3 * i + c] == ((i >= 3 && i < 25) ? next[3 * i + c] : base[3 * i + c]);
  check(ok, "read back: exactly the range changed, element i at texel (i % w, i / w)");
  tex->ReleaseGraphicsResources(win);
}

// ── GL: a generic StreamingGeometryNode ─────────────────────────────────────
// Two unit quads side by side in [0, 2] x [0, 1] (points 0-3 and 4-7), tinted
// by a uniform in the fragment shader.
StreamingLayout twoQuads(StreamingMapperKind k) {
  StreamingLayout l;
  l.capacity_points = 8;
  l.triangles = {{{0, 1, 2}}, {{0, 2, 3}}, {{4, 5, 6}}, {{4, 6, 7}}};
  l.reserved_bounds = cvc::bounding_box(-0.5, -0.5, -1, 2.5, 1.5, 1);
  l.mapper = k;
  return l;
}
void quadPoints(float x0, float out[12]) {
  const float q[12] = {x0, 0, 0, x0 + 1, 0, 0, x0 + 1, 1, 0, x0, 1, 0};
  std::copy(q, q + 12, out);
}

void testGenericNode(cvc::app &app, StreamingMapperKind kind) {
  std::printf("StreamingGeometryNode [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("generic_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  auto node =
      sg.getGraphicsRoot()->addGraphicsChild<StreamingGeometryNode>("quads", twoQuads(kind));
  node->addFragmentShaderReplacement("//VTK::Color::Dec",
                                     "//VTK::Color::Dec\nuniform vec3 cvcTestTint;\n");
  node->addFragmentShaderReplacement("//VTK::Color::Impl", "//VTK::Color::Impl\n"
                                                           "  ambientColor = cvcTestTint;\n"
                                                           "  diffuseColor = vec3(0.0);\n");
  node->setSpecular(0.0);
  node->setUniform("cvcTestTint", 1.0f, 0.1f, 0.1f);
  SceneRenderer sr(sg, 96, 64, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, 1.0, 0.5, 1.0);

  float q[12];
  quadPoints(0, q);
  node->writePoints(0, q, 4);
  node->setDrawRange(0, 2);
  Frame f = grab(sr);
  glcountInstall();
  const auto c0 = toPx(sr, 0.5, 0.5, 0), c1 = toPx(sr, 1.5, 0.5, 0);
  check(hueNear(f, c0, 1, 0) && bgNear(f, c1, 2),
        "first quad written + drawn (red); unwritten quad outside the draw range");
  check(node->streamStats().fullUploads == 1, "first draw = the one full upload");

  // A uniform: read at draw time, no upload.
  GLCount g0 = glcountRead();
  node->setUniform("cvcTestTint", 0.1f, 1.0f, 0.1f);
  f = grab(sr);
  GLCount d = glcountRead() - g0;
  check(hueNear(f, c0, 1, 1) && d.uploadCalls() == 0 && d.createdNothing(),
        "setUniform recolours with 0 uploads", glcountStr(d));

  // The second quad: exactly its 4 points go up.
  const cvc::gl::StreamStats s0 = node->streamStats();
  g0 = glcountRead();
  quadPoints(1, q);
  node->writePoints(4, q, 4);
  node->setDrawRange(0, StreamingGeometryNode::kAllTriangles);
  f = grab(sr);
  d = glcountRead() - g0;
  const cvc::gl::StreamStats s1 = node->streamStats();
  check(hueNear(f, c0, 1, 1) && hueNear(f, c1, 1, 1), "second quad drawn");
  check(s1.uploads - s0.uploads == 1 && s1.uploadBytes - s0.uploadBytes == 48 &&
            s1.fullUploads == s0.fullUploads,
        "node stats: 1 sub-range upload of 48 B, no full upload");
  check(d.uploadCalls() == 1 && d.uploadBytes() == 48 && d.createdNothing(),
        "GL: one 48-byte sub-upload, nothing created", glcountStr(d));

  // A true sub-range (first triangle > 0): only the second quad, no upload.
  g0 = glcountRead();
  node->setDrawRange(2, 2);
  f = grab(sr);
  d = glcountRead() - g0;
  check(bgNear(f, c0, 2) && hueNear(f, c1, 1, 1) && d.uploadCalls() == 0,
        "setDrawRange(2, 2) draws only triangles 2-3, 0 uploads", glcountStr(d));

  // Capacity overflow THROWS and stages nothing.
  const std::uint64_t writes = node->streamStats().writes;
  bool threw = false;
  try {
    node->writePoints(6, q, 4);
  } catch (const std::out_of_range &) {
    threw = true;
  }
  check(threw && node->streamStats().writes == writes,
        "writePoints past the capacity throws std::out_of_range, stages nothing");
  node->writePoints(8, q, 0); // an empty write at the end is fine
  threw = false;
  try {
    StreamingLayout bad = twoQuads(kind);
    bad.triangles.push_back({{6, 7, 8}});
    sg.getGraphicsRoot()->addGraphicsChild<StreamingGeometryNode>("bad", bad);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "a triangle indexing past the capacity throws std::invalid_argument");

  // Off-thread: many writes + range changes from a worker are marshalled and
  // COALESCED -- nothing touches VTK until the owner drains, then one apply
  // moves the merged range and the mapper uploads it once.
  node->setDrawRange(0, 0);
  sr.render();
  const cvc::gl::StreamStats t0 = node->streamStats();
  g0 = glcountRead();
  std::thread worker([&]() {
    float p[12];
    for (int it = 0; it < 50; ++it) {
      quadPoints(0.02f * it, p);
      node->writePoints(it % 2 ? 0 : 4, p, 4); // both quads, many times
      node->setDrawRange(0, 2 + (it % 3));
    }
    quadPoints(0, p);
    node->writePoints(0, p, 4);
    quadPoints(1, p);
    node->writePoints(4, p, 4);
    node->setDrawRange(0, 4);
    node->setUniform("cvcTestTint", 0.1f, 0.1f, 1.0f);
  });
  worker.join();
  const cvc::gl::StreamStats t1 = node->streamStats();
  check(t1.writes - t0.writes == 52 && t1.applies == t0.applies,
        "52 off-thread writes staged; none applied before the owner drains");
  f = grab(sr);
  d = glcountRead() - g0;
  const cvc::gl::StreamStats t2 = node->streamStats();
  check(t2.applies - t1.applies == 1, "one coalesced apply for 52 writes + 51 range changes",
        std::to_string(t2.applies - t1.applies) + " applies");
  check(t2.uploads - t1.uploads == 1 && t2.uploadBytes - t1.uploadBytes == 8 * 12 &&
            d.uploadCalls() == 1 && d.uploadBytes() == 96 && d.createdNothing(),
        "one merged 96-byte sub-upload", glcountStr(d));
  check(hueNear(f, c0, 1, 2) && hueNear(f, c1, 1, 2), "final off-thread state drawn (blue)");
  // On the owner thread a write applies inline.
  const std::uint64_t a0 = node->streamStats().applies;
  node->writePoints(0, q, 1);
  check(node->streamStats().applies == a0 + 1, "an owner-thread write applies immediately");

  // The header's claim: a material change uploads nothing (it only bumps the
  // property MTime).
  sr.render();
  g0 = glcountRead();
  const std::uint64_t full0 = node->streamStats().fullUploads;
  node->setColor(0.5, 0.5, 0.5);
  sr.render();
  d = glcountRead() - g0;
  check(d.uploadCalls() == 0 && d.createdNothing() && node->streamStats().fullUploads == full0,
        "a material change (setColor) uploads nothing", glcountStr(d));
  // ... and re-showing a hidden node is one full upload (the renderer released it).
  node->setVisible(false);
  sr.render();
  node->setVisible(true);
  sr.render();
  check(node->streamStats().fullUploads == full0 + 1, "hide + show = one full re-upload",
        std::to_string(node->streamStats().fullUploads - full0));
  check(ErrorCounter::errors() == 0, "no VTK errors");
}

// ── GL: RibbonNode capacity growth ──────────────────────────────────────────
void testRibbonGrowth(cvc::app &app, StreamingMapperKind kind) {
  std::printf("RibbonNode capacity growth [%s]\n", kindName(kind));
  SceneGraph sg(app, std::string("grow_") + kindName(kind));
  sg.setDiagnosticChromeVisible(false);
  auto rib = sg.getGraphicsRoot()->addGraphicsChild<RibbonNode>(
      "track", 4, 0.5f, cvc::bounding_box(-1, -2, -1, 12, 2, 1), kind);
  flat(*rib, 1.0, 0.1, 0.1);
  SceneRenderer sr(sg, 128, 48, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  topView(sr, 5.0, 0.0, 3.0);
  sr.render();
  const std::uint64_t full0 = rib->streamStats().fullUploads;
  for (int k = 0; k < 10; ++k) {
    rib->append(static_cast<float>(k), 0.0f, 0.0f);
    sr.render();
  }
  const Frame f = grab(sr);
  check(rib->centerCount() == 10 && rib->centerCapacity() == 16,
        "10 appends into capacity 4 grew it to 16 (doubling)",
        std::to_string(rib->centerCapacity()));
  check(rib->streamStats().fullUploads - full0 == 2, "each growth = one full re-upload (4->8->16)",
        std::to_string(rib->streamStats().fullUploads - full0));
  check(hueNear(f, toPx(sr, 0.5, 0, 0), 1, 0) && hueNear(f, toPx(sr, 8.5, 0, 0), 1, 0),
        "history kept across growth: first and last segments drawn");
  check(bgNear(f, toPx(sr, 10.5, 0, 0), 2), "nothing past the last centre");
  check(std::fabs(rib->arcLength() - 9.0) < 1e-9 &&
            std::fabs(rib->centerAtArcLength(2.5) - 2.5) < 1e-9,
        "arc length and arc-length -> centre index");
}

} // namespace

int main() {
  installErrorCounter();
  cvc::app app;
  testCoalescedEvents(app);
  testApplyKeyIsPrivate(app);
  testUnattachedAndDroppedSlot(app);
  testThrowingCallback(app);
  testMovedNodeStaleSlot(app);
  testBounds(app);
  testInternalShaderReplacements(app);
  testMapperSelection(app);
  testTexelSpans();
  testHeightSample();

  std::printf("GL sections\n");
  if (renderAvailable(app)) {
    {
      SceneGraph sg(app, "gl_probe");
      sg.setDiagnosticChromeVisible(false);
      SceneRenderer sr(sg, 16, 16, /*offscreen=*/true);
      sr.render();
      printRenderer(sr);
      glcountInstall();
      testTexelUploadGL(sr);
    }
    for (StreamingMapperKind k : {StreamingMapperKind::Classic, StreamingMapperKind::LowMemory}) {
      testGenericNode(app, k);
      testRibbonGrowth(app, k);
    }
  }
  check(ErrorCounter::errors() == 0, "no VTK errors overall",
        std::to_string(ErrorCounter::errors()));
  return finish("cvcgl_streaming_geometry");
}
