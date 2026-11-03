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

/*
  Portions of this file -- the regions between "BEGIN VTK 9.5.0-DERIVED" and
  "END VTK 9.5.0-DERIVED" below -- are derived from VTK 9.5.0
  (Rendering/OpenGL2: vtkGLSLModCoincidentTopology.cxx,
  vtkOpenGLLowMemoryPolyDataMapper.cxx, vtkOpenGLLowMemoryCellTypeAgent.cxx,
  vtkOpenGLLowMemoryVerticesAgent.cxx, vtkOpenGLLowMemoryLinesAgent.cxx,
  vtkOpenGLLowMemoryPolygonsAgent.cxx) and are used under VTK's license:

  SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  SPDX-License-Identifier: BSD-3-Clause

  Copyright (c) 1993-2015 Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

   * Neither name of Ken Martin, Will Schroeder, or Bill Lorensen nor the names
     of any contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  The rest of the file is libcvc's own (LGPL-2.1, above). The notice is also
  recorded in THIRD_PARTY_NOTICES.md.
*/

// No vtkOpenGL*ErrorMacro anywhere in this file: those macros are inline in the
// including translation unit, so a cvcGL build without NDEBUG would issue a
// glGetError per draw -- a synchronous round trip on WebGL.
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <cvc/gl/LowMemoryPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCellType.h>
#include <vtkCollection.h>
#include <vtkInformation.h>
#include <vtkInformationObjectBaseVectorKey.h>
#include <vtkMath.h>
#include <vtkObjectFactory.h>
#include <vtkOpenGLRenderPass.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLTexture.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkShaderProgram.h>
#include <vtkShaderProperty.h>
#include <vtkTexture.h>
#include <vtkVersionMacros.h>

// The replica is VTK 9.5.0's own agent code, so it is on only for an UNPATCHED
// 9.5.0 (the contract is in the class header):
//   VTK_CVC_LOWMEM_PATCHLEVEL       defined by any libcvc-deps VTK recipe patch
//                                   to the low-memory / vtkDrawTexturedElements
//                                   code, in the installed
//                                   vtkOpenGLLowMemoryPolyDataMapper.h;
//   CVC_GL_LOWMEM_REPLICA_DISABLED  set by cvcGL's configure-time check when an
//                                   installed header differs from 9.5.0's.
#if VTK_MAJOR_VERSION == 9 && VTK_MINOR_VERSION == 5 && VTK_BUILD_VERSION == 0 &&                  \
    !defined(VTK_CVC_LOWMEM_PATCHLEVEL) && !defined(CVC_GL_LOWMEM_REPLICA_DISABLED)
#define CVC_GL_LOWMEM_REPLICA 1
#else
#define CVC_GL_LOWMEM_REPLICA 0
#endif

namespace cvc {
namespace gl {

namespace {

int drawPathFromEnvironment() {
  const char *v = std::getenv("CVCGL_LOWMEM_DRAW");
  if (v && std::strcmp(v, "stock") == 0)
    return static_cast<int>(LowMemoryPolyDataMapper::DrawPath::Stock);
  if (v && std::strcmp(v, "all") == 0)
    return static_cast<int>(LowMemoryPolyDataMapper::DrawPath::AllCellTypes);
  return static_cast<int>(LowMemoryPolyDataMapper::DrawPath::Fast);
}

std::atomic<int> &drawPathStore() {
  static std::atomic<int> path{drawPathFromEnvironment()};
  return path;
}

int policyFromEnvironment() {
  const char *v = std::getenv("CVCGL_LOWMEM_MAPPER");
  if (v && std::strcmp(v, "force") == 0)
    return static_cast<int>(LowMemoryMapperPolicy::Force);
  if (v && std::strcmp(v, "off") == 0)
    return static_cast<int>(LowMemoryMapperPolicy::Off);
  return static_cast<int>(LowMemoryMapperPolicy::Auto);
}

std::atomic<int> &policyStore() {
  static std::atomic<int> policy{policyFromEnvironment()};
  return policy;
}

bool passProgramsFromEnvironment() {
  const char *v = std::getenv("CVCGL_LOWMEM_PASS_PROGRAMS");
  return !(v && std::strcmp(v, "0") == 0);
}

std::atomic<bool> &passProgramsStore() {
  static std::atomic<bool> on{passProgramsFromEnvironment()};
  return on;
}

// The most pass sets one mapper keeps a shader for beside its current one. Shadows make two (the
// bake's and the shadow pass's); a frame before the first bake, or a pick, adds one.
constexpr std::size_t kMaxPassPrograms = 4;

// Whether the render passes on the actor are exactly `passes`, in order. Allocates nothing: it runs
// on every draw.
bool actorHasPasses(vtkActor *act, const std::vector<const vtkObjectBase *> &passes) {
  vtkInformation *info = act->GetPropertyKeys();
  if (!info || !info->Has(vtkOpenGLRenderPass::RenderPasses()))
    return passes.empty();
  const int n = info->Length(vtkOpenGLRenderPass::RenderPasses());
  if (n != static_cast<int>(passes.size()))
    return false;
  for (int i = 0; i < n; ++i)
    if (info->Get(vtkOpenGLRenderPass::RenderPasses(), i) != passes[static_cast<std::size_t>(i)])
      return false;
  return true;
}

// Constant-initialised: no guard on the per-stage load.
std::atomic<LowMemoryPolyDataMapper::StageObserver> g_stageObserver{nullptr};

// Does VTK's object factory hand out the low-memory mapper on this build? Asked
// once, on the first node construction (the OpenGL2 factory is registered by
// then: cvcGL's sources carry the module auto-init).
bool factoryIsLowMemory() {
  static const bool lowMemory =
      vtkSmartPointer<vtkPolyDataMapper>::New()->IsA("vtkOpenGLLowMemoryPolyDataMapper") != 0;
  return lowMemory;
}

#if CVC_GL_LOWMEM_REPLICA
// ---- BEGIN VTK 9.5.0-DERIVED ------------------------------------------------
// Portions derived from VTK 9.5.0, Copyright (c) Ken Martin, Will Schroeder,
// Bill Lorensen; All rights reserved. SPDX-License-Identifier: BSD-3-Clause
// (full notice at the top of this file). coincidentValue() below is
// vtkGLSLModCoincidentTopology::GetCoincidentParameters.
//
// The depth offset vtkGLSLModCoincidentTopology::GetCoincidentParameters (VTK
// 9.5.0) computes for one primitive class, as the floats it would upload.
// slot: 0 points, 1 lines, 2 polygons. `set` false: the mod uploads nothing.
struct CoincidentValue {
  bool set = false;
  float factor = 0.0f, offset = 0.0f;
  bool operator==(const CoincidentValue &o) const {
    return set == o.set && (!set || (factor == o.factor && offset == o.offset));
  }
  bool operator!=(const CoincidentValue &o) const { return !(*this == o); }
};

CoincidentValue coincidentValue(vtkMapper *mapper, vtkProperty *prop, int slot) {
  float factor = 0.0f, offset = 0.0f;
  if (vtkMapper::GetResolveCoincidentTopology() == VTK_RESOLVE_SHIFT_ZBUFFER)
    offset = static_cast<float>(vtkMapper::GetResolveCoincidentTopologyZShift() * 4.0);
  if (vtkMapper::GetResolveCoincidentTopology() == VTK_RESOLVE_POLYGON_OFFSET ||
      (prop->GetEdgeVisibility() && prop->GetRepresentation() == VTK_SURFACE)) {
    double f = 0.0, u = 0.0;
    if (slot == 0)
      mapper->GetCoincidentTopologyPointOffsetParameter(u);
    else if (slot == 1)
      mapper->GetCoincidentTopologyLineOffsetParameters(f, u);
    else
      mapper->GetCoincidentTopologyPolygonOffsetParameters(f, u);
    factor = static_cast<float>(f);
    offset = static_cast<float>(u);
  }
  CoincidentValue v;
  v.set = factor != 0.0f || offset != 0.0f;
  v.factor = factor;
  v.offset = offset;
  return v;
}
// ---- END VTK 9.5.0-DERIVED --------------------------------------------------
#endif

} // namespace

vtkStandardNewMacro(LowMemoryPolyDataMapper);

LowMemoryPolyDataMapper::LowMemoryPolyDataMapper() {
#if VTK_MAJOR_VERSION == 9
  // VTK 9.5.0 declares these without initialisers (9.5.1 initialises the bool
  // only). With DISABLE_SHIFT_SCALE nothing ever assigns them, so whether the
  // mapper uploaded a (garbage) shift/scale copy of the positions depended on a
  // heap byte.
  this->CoordinateShiftAndScaleInUse = false;
  this->ShiftValues.fill(0.0);
  this->ScaleValues.fill(1.0);
#endif
}

void LowMemoryPolyDataMapper::setDrawPath(DrawPath path) {
  drawPathStore().store(static_cast<int>(path), std::memory_order_relaxed);
}

LowMemoryPolyDataMapper::DrawPath LowMemoryPolyDataMapper::drawPath() {
  return static_cast<DrawPath>(drawPathStore().load(std::memory_order_relaxed));
}

bool LowMemoryPolyDataMapper::replicaActive() { return CVC_GL_LOWMEM_REPLICA != 0; }

void LowMemoryPolyDataMapper::setPassProgramsEnabled(bool on) {
  passProgramsStore().store(on, std::memory_order_relaxed);
}

bool LowMemoryPolyDataMapper::passProgramsEnabled() {
  return passProgramsStore().load(std::memory_order_relaxed);
}

void LowMemoryPolyDataMapper::setStageObserver(StageObserver observer) {
  g_stageObserver.store(observer, std::memory_order_relaxed);
}

void LowMemoryPolyDataMapper::invalidateProgramCache() {
  m_cache = nullptr;
  m_builtStamp = 0;
}

vtkShader *LowMemoryPolyDataMapper::GetShader(vtkShader::Type shaderType) {
  // The caller may edit the source: the next draw must resolve it again.
  invalidateProgramCache();
  return this->vtkDrawTexturedElements::GetShader(shaderType);
}

void LowMemoryPolyDataMapper::RenderPieceStart(vtkRenderer *ren, vtkActor *act) {
  // First the shader for this draw's render passes (pass programs, in the class header), so the
  // tests below and VTK's own judge that one.
  swapPassProgram(act);
  // VTK's low-memory mapper (9.5 through 9.7) never looks at the actor's shader
  // property once its program is built: IsShaderUpToDate ignores it, where the
  // classic mapper compares GetShaderMTime(). A replacement added or cleared
  // after the first draw (GeometryNode::add*ShaderReplacement /
  // clearShaderReplacements), or a custom uniform declared after it, would
  // never reach the GPU. Dropping the program makes the base rebuild it
  // (UpdateShaders + a new ShaderBuildTimeStamp, so the draw that follows also
  // takes the full shader-cache lookup). Uniform VALUE changes do not move
  // GetShaderMTime, so they never cost a rebuild.
  if (vtkShaderProperty *sp = act->GetShaderProperty())
    if (sp->GetShaderMTime() > this->ShaderBuildTimeStamp.GetMTime())
      this->ShaderProgram = nullptr;
  const vtkMTimeType built = this->ShaderBuildTimeStamp.GetMTime();
  Superclass::RenderPieceStart(ren, act);
  if (this->ShaderBuildTimeStamp.GetMTime() != built) {
    ++m_stats.shaderBuilds;
    m_activeTextures = actorTextures(act);
  }
  // VTK rebuilds the current shader when the color or normal source changes (and the array
  // declarations come with the data); the kept ones were built for the old ones.
  const int color = static_cast<int>(this->ShaderColorSource);
  const int normal = static_cast<int>(this->ShaderNormalSource);
  if (color != m_colorSource || normal != m_normalSource || !sameDeclarations()) {
    m_passPrograms.clear();
    m_colorSource = color;
    m_normalSource = normal;
    m_declarations = this->ShaderDecls;
  }
}

std::vector<const vtkObjectBase *> LowMemoryPolyDataMapper::actorPasses(vtkActor *act) {
  std::vector<const vtkObjectBase *> passes;
  vtkInformation *info = act->GetPropertyKeys();
  if (!info || !info->Has(vtkOpenGLRenderPass::RenderPasses()))
    return passes;
  const int n = info->Length(vtkOpenGLRenderPass::RenderPasses());
  passes.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i)
    passes.push_back(info->Get(vtkOpenGLRenderPass::RenderPasses(), i));
  return passes;
}

std::vector<const vtkTexture *> LowMemoryPolyDataMapper::actorTextures(vtkActor *act) {
  // GetTextures(act)'s list without its names (a string apiece): it runs at every pass switch.
  std::vector<const vtkTexture *> textures;
  if (this->ColorTextureMap)
    textures.push_back(this->InternalColorTexture);
  if (vtkTexture *t = act->GetTexture())
    textures.push_back(t);
  for (const auto &kv : act->GetProperty()->GetAllTextures())
    textures.push_back(kv.second);
  return textures;
}

bool LowMemoryPolyDataMapper::sameDeclarations() const {
  if (m_declarations.size() != this->ShaderDecls.size())
    return false;
  for (std::size_t i = 0; i < m_declarations.size(); ++i) {
    const vtkOpenGLShaderDeclaration &a = m_declarations[i], &b = this->ShaderDecls[i];
    if (a.QualifierType != b.QualifierType || a.PrecisionType != b.PrecisionType ||
        a.DataType != b.DataType || a.AttributeType != b.AttributeType ||
        a.VariableName.GetId() != b.VariableName.GetId())
      return false;
  }
  return true;
}

void LowMemoryPolyDataMapper::swapPassProgram(vtkActor *act) {
  if (actorHasPasses(act, m_activePasses))
    return; // the passes the current shader was built for: VTK's test alone
  const bool keep = passProgramsEnabled();
  if (!keep) {
    m_passPrograms.clear();
  } else if (this->ShaderProgram) {
    PassProgram kept;
    kept.passes = m_activePasses;
    kept.alive = m_activeAlive;
    kept.program = this->ShaderProgram;
    kept.vertex = this->Shaders[vtkShader::Vertex]->GetSource();
    kept.fragment = this->Shaders[vtkShader::Fragment]->GetSource();
    kept.geometry = this->Shaders[vtkShader::Geometry]->GetSource();
    this->GLSLMods->InitTraversal();
    while (vtkObject *mod = this->GLSLMods->GetNextItemAsObject())
      kept.mods.emplace_back(mod);
    kept.built = this->ShaderBuildTimeStamp;
    kept.cache = m_cache;
    kept.lookupStamp = m_builtStamp;
    kept.textures = m_activeTextures;
    for (auto it = m_passPrograms.begin(); it != m_passPrograms.end(); ++it)
      if (it->passes == kept.passes) {
        m_passPrograms.erase(it);
        break;
      }
    if (m_passPrograms.size() >= kMaxPassPrograms)
      m_passPrograms.erase(m_passPrograms.begin());
    m_passPrograms.push_back(std::move(kept));
  }
  // This draw's passes are the current shader's from here: put back below, or built by VTK.
  m_activePasses = actorPasses(act);
  m_activeAlive.clear();
  for (const vtkObjectBase *p : m_activePasses)
    m_activeAlive.emplace_back(const_cast<vtkObjectBase *>(p));
  if (!keep)
    return;
  auto it = m_passPrograms.begin();
  while (it != m_passPrograms.end() && it->passes != m_activePasses)
    ++it;
  if (it == m_passPrograms.end())
    return;
  PassProgram found = std::move(*it);
  m_passPrograms.erase(it);
  // Still the shader VTK would build here? Its passes all alive (an address can be reused), and
  // the actor's property and textures as they were (VTK's test reads neither).
  for (std::size_t i = 0; i < found.passes.size(); ++i)
    if (found.alive[i].GetPointer() != found.passes[i])
      return;
  if (act->GetProperty()->GetMTime() > found.built.GetMTime() ||
      actorTextures(act) != found.textures)
    return;
  this->ShaderProgram = found.program;
  this->Shaders[vtkShader::Vertex]->SetSource(found.vertex);
  this->Shaders[vtkShader::Fragment]->SetSource(found.fragment);
  this->Shaders[vtkShader::Geometry]->SetSource(found.geometry);
  this->GLSLMods->RemoveAllItems();
  for (const vtkSmartPointer<vtkObject> &mod : found.mods)
    this->GLSLMods->AddItem(mod);
  this->ShaderBuildTimeStamp = found.built;
  m_cache = found.cache;
  m_builtStamp = found.lookupStamp;
  m_activeTextures = std::move(found.textures);
  // VTK's test compares this draw's passes with the last draw's (and rebuilds on any difference):
  // make the last draw's these, so it judges the shader by the passes' shader-stage MTimes.
  if (vtkInformation *info = act->GetPropertyKeys())
    this->LastRenderPassInfo->CopyEntry(info, vtkOpenGLRenderPass::RenderPasses());
  else
    this->LastRenderPassInfo->Clear();
  ++m_stats.passProgramsRestored;
}

void LowMemoryPolyDataMapper::ReleaseGraphicsResources(vtkWindow *win) {
  // The cached program belongs to that window's context; look it up afresh.
  invalidateProgramCache();
  m_passPrograms.clear();
  Superclass::ReleaseGraphicsResources(win);
}

#if !CVC_GL_LOWMEM_REPLICA

void LowMemoryPolyDataMapper::RenderPieceDraw(vtkRenderer *ren, vtkActor *act) {
  ++m_stats.draws;
  ++m_stats.stockDraws;
  Superclass::RenderPieceDraw(ren, act);
}
void LowMemoryPolyDataMapper::readyProgram(vtkRenderer *ren, bool) {
  this->vtkDrawTexturedElements::ReadyShaderProgram(ren);
}
bool LowMemoryPolyDataMapper::renderable(int) const { return true; }
bool LowMemoryPolyDataMapper::coincidentSkipSafe(vtkActor *) const { return false; }
void LowMemoryPolyDataMapper::agentPreDraw(int, vtkRenderer *, vtkActor *) {}
void LowMemoryPolyDataMapper::agentDraw(int, vtkRenderer *, vtkActor *) {}

#else

namespace {
inline void observeStage(LowMemoryPolyDataMapper::StageObserver observe,
                         LowMemoryPolyDataMapper::DrawStage stage, int arg = 0) {
  if (observe)
    observe(stage, arg);
}
} // namespace

// ---- BEGIN VTK 9.5.0-DERIVED ------------------------------------------------
// Portions derived from VTK 9.5.0, Copyright (c) Ken Martin, Will Schroeder,
// Bill Lorensen; All rights reserved. SPDX-License-Identifier: BSD-3-Clause
// (full notice at the top of this file). RenderPieceDraw's agent loop is
// vtkOpenGLLowMemoryPolyDataMapper::RenderPieceDraw; renderable(), agentPreDraw()
// and agentDraw() are vtkOpenGLLowMemoryCellTypeAgent::PreDraw / Draw and the
// vtkOpenGLLowMemory{Vertices,Lines,Polygons}Agent::PreDrawInternal they call.
// readyProgram() and coincidentSkipSafe() are libcvc's own.
void LowMemoryPolyDataMapper::RenderPieceDraw(vtkRenderer *ren, vtkActor *act) {
  ++m_stats.draws;
  const DrawPath path = drawPath();
  // Selection (picking state, point-picking sizes) and the vertex-visibility
  // pass stay on VTK's own path; the replica covers the plain draw only.
  if (path == DrawPath::Stock || ren->GetSelector() || act->GetProperty()->GetVertexVisibility()) {
    ++m_stats.stockDraws;
    Superclass::RenderPieceDraw(ren, act);
    return;
  }
  const bool fast = path == DrawPath::Fast;
  const StageObserver observe = g_stageObserver.load(std::memory_order_relaxed);
  const bool skip = fast && coincidentSkipSafe(act); // CPU only (no GL)
  observeStage(observe, DrawStage::DrawBegin, skip ? 1 : 0);
  readyProgram(ren, fast);
  // Uniforms common to every cell type; also fires UpdateShaderEvent, which
  // GeometryNode's custom shader textures hang off.
  this->SetShaderParameters(ren, act);
  if (!this->ShaderProgram) {
    observeStage(observe, DrawStage::DrawEnd);
    return; // compile/link failure: VTK's agents would dereference null here
  }
  if (fast && !skip)
    ++m_stats.coincidentFallbacks;
  for (int t = 0; t < 4; ++t) { // verts, lines, polys, strips: VTK's order
    if (skip && !renderable(t)) {
      ++m_stats.cellTypesSkipped;
      observeStage(observe, DrawStage::AgentSkipped, t);
      continue;
    }
    observeStage(observe, DrawStage::AgentBegin, t);
    agentPreDraw(t, ren, act);
    agentDraw(t, ren, act);
    // vtkOpenGLLowMemoryCellTypeAgent::PostDraw; every 9.5.0 PostDrawInternal is empty.
    this->vtkDrawTexturedElements::PostDraw(ren, act, this);
    observeStage(observe, DrawStage::AgentEnd, t);
  }
  observeStage(observe, DrawStage::DrawEnd);
}

void LowMemoryPolyDataMapper::readyProgram(vtkRenderer *ren, bool allowSkip) {
  const StageObserver observe = g_stageObserver.load(std::memory_order_relaxed);
  auto *win = vtkOpenGLRenderWindow::SafeDownCast(ren->GetRenderWindow());
  vtkOpenGLShaderCache *cache = win ? win->GetShaderCache() : nullptr;
  const vtkMTimeType stamp = this->ShaderBuildTimeStamp.GetMTime();
  // this->Shaders only changes in UpdateShaders, which RenderPieceStart follows
  // with ShaderBuildTimeStamp.Modified() (GetShader() and
  // invalidateProgramCache() clear m_cache for any other route); while the
  // stamp and the cache are the ones the last lookup saw, that lookup would
  // find the same program again. ReadyShaderProgram(program) still compiles it
  // if its context was released and binds it exactly as the lookup's tail
  // would, so the two issue the same GL calls.
  if (allowSkip && cache && this->ShaderProgram && cache == m_cache.GetPointer() &&
      stamp == m_builtStamp && this->ElementType != AbstractPatches) {
    observeStage(observe, DrawStage::RebindBegin);
    this->ShaderProgram = cache->ReadyShaderProgram(this->ShaderProgram.GetPointer());
    observeStage(observe, DrawStage::RebindEnd);
    ++m_stats.programLookupsSkipped;
    return;
  }
  observeStage(observe, DrawStage::LookupBegin);
  this->vtkDrawTexturedElements::ReadyShaderProgram(ren); // copy, substitute, MD5, find, bind
  observeStage(observe, DrawStage::LookupEnd);
  ++m_stats.programLookups;
  m_cache = cache;
  m_builtStamp = stamp;
}

bool LowMemoryPolyDataMapper::renderable(int t) const {
  // vtkOpenGLLowMemoryCellTypeAgent::Draw draws cell group 0 iff it CanRender.
  const auto &groups = this->Primitives[t].CellGroups;
  return !groups.empty() && groups[0].CanRender;
}

bool LowMemoryPolyDataMapper::coincidentSkipSafe(vtkActor *act) const {
  // vtkGLSLModCoincidentTopology sets cOffset/cFactor in every PreDraw, per
  // primitive class, and only when the value is non-zero; otherwise the program
  // keeps whatever was set last. So a cell type whose own offset is zero sees
  // what an earlier cell type -- possibly one with nothing to draw -- left
  // behind, and the last one leaves a value for the next draw of that program.
  // Skipping is safe when every drawn type, and the program afterwards, ends up
  // with the same value either way. Conservative: the mod declares the offset
  // uniforms only if the shader was BUILT under a non-zero offset, which this
  // does not ask -- with the global mode switched on after the build (an offset
  // VTK then never applies) it falls back for nothing.
  vtkProperty *prop = act->GetProperty();
  const int mode = vtkMapper::GetResolveCoincidentTopology();
  if (mode != VTK_RESOLVE_POLYGON_OFFSET && mode != VTK_RESOLVE_SHIFT_ZBUFFER &&
      !(prop->GetEdgeVisibility() && prop->GetRepresentation() == VTK_SURFACE))
    return true; // no offset uniform is ever set (VTK's default)
  auto *self = const_cast<LowMemoryPolyDataMapper *>(this);
  const bool points = prop->GetRepresentation() == VTK_POINTS;
  CoincidentValue stock, skipped; // unset: unchanged since before this draw
  for (int t = 0; t < 4; ++t) {
    const int slot = points ? 0 : (t == 0 ? 0 : (t == 1 ? 1 : 2));
    const CoincidentValue v = coincidentValue(self, prop, slot);
    if (v.set)
      stock = v;
    if (renderable(t)) {
      if (v.set)
        skipped = v;
      if (stock != skipped)
        return false;
    }
  }
  return stock == skipped;
}

void LowMemoryPolyDataMapper::agentPreDraw(int t, vtkRenderer *ren, vtkActor *act) {
  // vtkOpenGLLowMemory{Vertices,Lines,Polygons}Agent::PreDrawInternal (the
  // strips agent is the polygons agent) ...
  vtkProperty *prop = act->GetProperty();
  int pointsPerPrimitive = 3;
  if (t == 0) {
    pointsPerPrimitive = 1;
    this->ElementType = vtkDrawTexturedElements::ElementShape::Point;
    this->NumberOfInstances = 1;
    this->ShaderProgram->SetUniformi("cellType", VTK_VERTEX);
  } else if (t == 1) {
    pointsPerPrimitive = 2;
    this->ElementType = vtkDrawTexturedElements::ElementShape::Line;
    if (prop->GetLineWidth() > 1)
      this->NumberOfInstances = 2 * vtkMath::Ceil(prop->GetLineWidth());
    else
      this->NumberOfInstances = 1;
    this->ShaderProgram->SetUniformi("cellType", VTK_LINE);
  } else {
    this->ElementType = vtkDrawTexturedElements::ElementShape::Triangle;
    this->NumberOfInstances = 1;
    this->ShaderProgram->SetUniformi("cellType", VTK_TRIANGLE);
  }
  // ... then vtkOpenGLLowMemoryCellTypeAgent::PreDraw (never in the vertex
  // visibility pass, which takes the stock draw).
  if (prop->GetRepresentation() == VTK_POINTS)
    this->ElementType = vtkDrawTexturedElements::ElementShape::Point;
  bool needLighting = false;
  if (prop->GetRepresentation() == VTK_POINTS) {
    needLighting = prop->GetInterpolation() != VTK_FLAT && this->HasPointNormals;
  } else {
    const bool isTrisOrStrips = pointsPerPrimitive >= 3;
    needLighting = isTrisOrStrips || (!isTrisOrStrips && prop->GetInterpolation() != VTK_FLAT &&
                                      this->HasPointNormals);
  }
  this->ShaderProgram->SetUniformi("enable_lights", needLighting);
  this->ShaderProgram->SetUniformi("vertex_pass", false);
  switch (this->ElementType) {
  case vtkDrawTexturedElements::ElementShape::Point:
    this->ShaderProgram->SetUniformi("primitiveSize", 1);
    break;
  case vtkDrawTexturedElements::ElementShape::Line:
    this->ShaderProgram->SetUniformi("primitiveSize", 2);
    break;
  case vtkDrawTexturedElements::ElementShape::Triangle:
  default:
    this->ShaderProgram->SetUniformi("primitiveSize", 3);
    break;
  }
  // PointPicking is only ever set with a selector, which takes the stock draw.
  this->ShaderProgram->SetUniformf("pointSize", prop->GetPointSize());
  this->vtkDrawTexturedElements::PreDraw(ren, act, this);
}

void LowMemoryPolyDataMapper::agentDraw(int t, vtkRenderer *ren, vtkActor *act) {
  // vtkOpenGLLowMemoryCellTypeAgent::Draw, cell group 0.
  const auto &groups = this->Primitives[t].CellGroups;
  if (groups.empty() || !groups[0].CanRender)
    return;
  const auto &group = groups[0];
  const auto &offsets = group.Offsets;
  this->FirstVertexId = offsets.VertexIdOffset;
  this->NumberOfElements = group.NumberOfElements;
  // when rendering vertices, increase number of elements and draw 1 instance.
  if (act->GetProperty()->GetRepresentation() == VTK_POINTS) {
    this->NumberOfElements *= (t == 0 ? 1 : (t == 1 ? 2 : 3));
    this->NumberOfInstances = 1;
  }
  vtkShaderProgram *program = this->ShaderProgram;
  program->SetUniformi("cellIdOffset", offsets.CellIdOffset);
  program->SetUniformi("vertexIdOffset", offsets.VertexIdOffset);
  program->SetUniformi("edgeValueBufferOffset", offsets.EdgeValueBufferOffset);
  program->SetUniformi("pointIdOffset", offsets.PointIdOffset);
  program->SetUniformi("primitiveIdOffset", offsets.PrimitiveIdOffset);
  program->SetUniformi("usesCellMap", group.UsesCellMapBuffer);
  program->SetUniformi("usesEdgeValues", group.UsesEdgeValueBuffer);
  this->vtkDrawTexturedElements::DrawInstancedElementsImpl(ren, act, this);
}
// ---- END VTK 9.5.0-DERIVED --------------------------------------------------

#endif // CVC_GL_LOWMEM_REPLICA

void setLowMemoryMapperPolicy(LowMemoryMapperPolicy policy) {
  policyStore().store(static_cast<int>(policy), std::memory_order_relaxed);
}

LowMemoryMapperPolicy lowMemoryMapperPolicy() {
  return static_cast<LowMemoryMapperPolicy>(policyStore().load(std::memory_order_relaxed));
}

vtkSmartPointer<vtkPolyDataMapper> newPolyDataMapper() {
  const LowMemoryMapperPolicy policy = lowMemoryMapperPolicy();
  if (policy == LowMemoryMapperPolicy::Force ||
      (policy == LowMemoryMapperPolicy::Auto && factoryIsLowMemory()))
    return vtkSmartPointer<LowMemoryPolyDataMapper>::New();
  return vtkSmartPointer<vtkPolyDataMapper>::New();
}

} // namespace gl
} // namespace cvc
