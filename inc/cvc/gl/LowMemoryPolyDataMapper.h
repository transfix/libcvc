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

// LowMemoryPolyDataMapper -- VTK 9.5.0's low-memory (GLES3/WebGL2) poly-data
// mapper with two pixel-identical cuts to its per-draw cost.
//
// On GLES3/WebGL2 VTK's object factory hands every vtkPolyDataMapper::New() a
// vtkOpenGLLowMemoryPolyDataMapper. Its draw runs four cell-type "agents" in a
// row (verts, lines, polys, strips), and each one does the full shader setup
// whether or not it has anything to draw: every array texture re-bound, every
// camera, material, shadow and agent uniform re-sent, the VAO bound and
// unbound. A typical mesh has one cell type, so three quarters of that is
// thrown away -- about 110 of the ~160 GL calls a lit mesh costs per draw. On
// top of that, every draw re-resolves its shader program through the shader
// cache: five source copies, re-substitution and an MD5 of ~16 KB, to find the
// program it already has.
//
// This subclass:
//   1. skips the cell types that draw nothing (a cell type draws iff its first
//      cell group can render, exactly the condition VTK's agent tests), and
//   2. re-binds the program it resolved last time while the shader has not been
//      rebuilt since (same ShaderBuildTimeStamp, same render window shader cache)
//      instead of looking it up again.
// Nothing else changes: the drawn cell type gets the same uniforms, textures and
// draw call in the same order, so frames are byte-identical (the cvcgl_lowmem_
// fastdraw test pins that, and pins the replica against VTK call-for-call).
//
// The cell-type agents are VTK private classes (headers not installed, symbols
// hidden), so the drawn agent's PreDraw/Draw/PostDraw is replicated here from
// VTK 9.5.0 (BSD-3-Clause; the notice is in LowMemoryPolyDataMapper.cpp and
// THIRD_PARTY_NOTICES.md). Picking, vertex visibility and a coincident-offset
// layout where skipping a cell type would change the depth offset a drawn one
// sees all take the stock (all four agents) draw.
//
// The replica is active (replicaActive()) only against an UNPATCHED VTK 9.5.0.
// The version macros alone cannot tell a patched 9.5.0 from a pristine one, so:
//   * Contract for the libcvc-deps VTK recipe: any patch that touches the
//     Rendering/OpenGL2 low-memory mapper, its cell-type agents,
//     vtkDrawTexturedElements or the GLSL mods must also add
//         #define VTK_CVC_LOWMEM_PATCHLEVEL <n>   // n >= 1
//     to the installed vtkOpenGLLowMemoryPolyDataMapper.h. The replica turns
//     itself off whenever that macro is defined (every DrawPath is then the
//     stock draw) until it is re-validated against the patched code and taught
//     that patch level.
//   * At configure time cvcGL also hashes the installed
//     vtkOpenGLLowMemoryPolyDataMapper.h, vtkDrawTexturedElements.h and
//     vtkGLSLModCoincidentTopology.h against pristine 9.5.0 and builds the
//     replica off (CVC_GL_LOWMEM_REPLICA_DISABLED) if any of them differs.
// cvcgl_lowmem_fastdraw pins the replica against the stock draw call for call,
// so it is the gate for any VTK recipe change.
//
// VTK v9.7.0 (tagged 2026-08-14) is NOT a drop-in replacement. It carries
// upstream f59c2d0b297 (vtkDrawTexturedElements re-binds its cached program
// unless GetShader() was called -- this class's cut 2) and 15ed6edc478
// (per-program uniform-location and last-value caches in vtkShaderProgram, the
// GLSL mods and the low-memory mapper), but it still runs all four cell-type
// agents per draw. A VTK upgrade switches the replica off (version check), so
// the empty-agent skip (cut 1) must be re-ported against 9.7's agents or that
// saving is lost. 9.7.0 also contains d1cde2af1a8, which makes every
// vtkOpenGLState::vtkglBlitFramebuffer issue two glCheckFramebufferStatus calls
// first -- at a cvcGL frame's two blits, 4 synchronous round trips per frame on
// WebGL; revert or avoid it in the wasm VTK build.
//
// The shader-cache skip (cut 2) relies on one invariant: this->Shaders (the
// vtkShader sources vtkDrawTexturedElements resolves its program from) changes
// only inside UpdateShaders, and RenderPieceStart always follows UpdateShaders
// with ShaderBuildTimeStamp.Modified(). vtkDrawTexturedElements::GetShader() is
// public and hands out a mutable vtkShader, so this class hides it with a
// version that drops the cached program first (as upstream f59c2d0b297 does);
// anything that edits shader sources by another route must call
// invalidateProgramCache(). Nothing in cvcGL edits them outside
// UpdateShaders today.
//
// For every VTK version, RenderPieceStart also rebuilds the shader program when
// the actor's shader property (replacements, custom uniform declarations)
// changed after the program was built: VTK's low-memory mapper (9.5 through
// 9.7) ignores the shader property once its program exists, so a
// GeometryNode::add*ShaderReplacement made after the first draw would never
// reach the GPU on WebGL.
//
// It also zero-initialises the shift/scale members VTK 9.5.0 leaves
// uninitialised (with DISABLE_SHIFT_SCALE nothing ever assigns them, so a
// garbage "in use" byte made the mapper upload a shift/scale copy of the
// positions).
//
// And it keeps one built shader per set of render passes it is drawn under
// (pass programs). VTK builds a mapper's shader for the render passes on its
// actor (vtkOpenGLRenderPass::RenderPasses) and rebuilds it whenever that set
// differs from the last draw's -- so under shadows, where every caster is drawn
// once by the shadow baker and once by the shadow pass, each caster rebuilt its
// shader source twice a frame (a dozen-odd Substitute calls over ~16 KB, then
// the shader cache's copy and MD5), the program found being the one it had two
// draws before. On leaving a pass set this class keeps that set's shader --
// sources, GLSL mods, program and build stamp -- and on coming back to it puts
// them back, so VTK's own up-to-date test (the passes' shader-stage MTimes, the
// mods, selection, PBR and the shader property, all against that program's build
// stamp) decides as if the mapper had never left. A kept shader is dropped when a
// pass it was built for is deleted, when the actor's property changed after it
// was built or its textures differ (VTK's low-memory mapper never looks at
// either; the alternation used to rebuild them anyway), and with every other one
// kept when the mapper's color or normal source or its array declarations change
// (VTK's rebuild of the current one does not reach the others).
#ifndef CVC_GL_LOW_MEMORY_POLY_DATA_MAPPER_H
#define CVC_GL_LOW_MEMORY_POLY_DATA_MAPPER_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <vtkOpenGLLowMemoryPolyDataMapper.h>
#include <vtkOpenGLShaderCache.h>
#include <vtkShader.h>
#include <vtkSmartPointer.h>
#include <vtkTimeStamp.h>
#include <vtkWeakPointer.h>

class vtkTexture;

namespace cvc {
class app;
namespace gl {

class LowMemoryPolyDataMapper : public vtkOpenGLLowMemoryPolyDataMapper {
public:
  static LowMemoryPolyDataMapper *New();
  vtkTypeMacro(LowMemoryPolyDataMapper, vtkOpenGLLowMemoryPolyDataMapper);

  // How the LowMemoryPolyDataMappers of `app` draw (those newPolyDataMapper(app)
  // made, or setApp(app) gave it). The initial value comes from CVCGL_LOWMEM_DRAW
  // (stock | all | fast, default fast), read when the app's settings are first
  // used; the other two paths exist for A/B measurement and for the tests:
  //   Stock         VTK's own RenderPieceDraw (all four agents, full lookup).
  //   AllCellTypes  the replica with nothing skipped -- issues the same GL calls
  //                 as Stock, call for call.
  //   Fast          the replica, empty cell types skipped, program re-bound.
  enum class DrawPath : int { Stock = 0, AllCellTypes = 1, Fast = 2 };
  static void setDrawPath(cvc::app &app, DrawPath path);
  static DrawPath drawPath(cvc::app &app);

  // True when this build carries the VTK 9.5.0 replica (an unpatched 9.5.0, see
  // above). False means every path is the stock draw.
  static bool replicaActive();

  // Whether `app`'s LowMemoryPolyDataMappers keep a shader per render-pass set
  // (pass programs, above). The initial value comes from
  // CVCGL_LOWMEM_PASS_PROGRAMS (0 turns it off; default on), for A/B
  // measurement and the tests. Turning it off drops nothing already kept until
  // the next draw that would use it.
  static void setPassProgramsEnabled(cvc::app &app, bool on);
  static bool passProgramsEnabled(cvc::app &app);

  // The app whose settings (above) this mapper draws by, and whose mappers it
  // shares its shaders' source copies with (newPolyDataMapper(app) sets it; a
  // node that is handed a mapper sets its own app's). A mapper with none draws by
  // the environment's defaults and shares with no other.
  void setApp(cvc::app &app);

  // Per-mapper counters, read and reset on the render thread.
  struct Stats {
    std::uint64_t draws = 0;                 // RenderPieceDraw calls
    std::uint64_t stockDraws = 0;            //   of which drew through VTK's own path
    std::uint64_t cellTypesSkipped = 0;      // agents not run because they draw nothing
    std::uint64_t coincidentFallbacks = 0;   // draws that ran all four for the depth offset
    std::uint64_t programLookups = 0;        // full shader-cache lookups (copy + MD5)
    std::uint64_t programLookupsSkipped = 0; // cached program re-bound instead
    std::uint64_t shaderBuilds = 0;          // UpdateShaders runs (new shader source)
    std::uint64_t passProgramsRestored = 0;  // a kept pass set's shader put back instead
  };
  const Stats &stats() const { return m_stats; }
  void resetStats() { m_stats = Stats{}; }

  // Test instrumentation: when set, the replica of `app`'s mappers reports where
  // each stage of a draw begins and ends (the cvcgl_lowmem_fastdraw GL trace
  // interleaves these with the GL calls to cut a trace into draws and cell-type
  // blocks). Null -- the default -- costs one relaxed load per stage. Stock draws
  // report nothing.
  enum class DrawStage : int {
    DrawBegin,   // arg: 1 if this draw skips its empty cell types (Fast only)
    LookupBegin, // full shader-cache lookup
    LookupEnd,
    RebindBegin, // cached program re-bound instead
    RebindEnd,
    AgentBegin, // arg: cell type 0-3
    AgentEnd,   // arg: cell type 0-3
    AgentSkipped,
    DrawEnd
  };
  using StageObserver = void (*)(DrawStage stage, int arg);
  static void setStageObserver(cvc::app &app, StageObserver observer);

  // An app's settings and shared source copies (the .cpp's).
  struct Settings;

  // Drop the program the last lookup resolved, so the next draw looks it up in
  // the shader cache again. For anything that edits this->Shaders outside
  // UpdateShaders (see the invariant above).
  void invalidateProgramCache();
  // vtkDrawTexturedElements::GetShader (public, non-virtual) hands out a
  // mutable source; this hides it so a caller holding this class also drops
  // the cached program.
  vtkShader *GetShader(vtkShader::Type shaderType);

  void RenderPieceStart(vtkRenderer *ren, vtkActor *act) override;
  void RenderPieceDraw(vtkRenderer *ren, vtkActor *act) override;
  void ReleaseGraphicsResources(vtkWindow *win) override;

protected:
  LowMemoryPolyDataMapper();
  ~LowMemoryPolyDataMapper() override = default;

  // Bind this draw's shader program: the full lookup, or -- when allowSkip and
  // nothing it depends on changed -- the program the last lookup returned.
  void readyProgram(vtkRenderer *ren, bool allowSkip);
  // Whether cell type t (0 verts, 1 lines, 2 polys, 3 strips) draws anything.
  bool renderable(int t) const;
  // Whether skipping the non-drawing cell types leaves every drawn one, and the
  // program afterwards, with the depth offset the stock draw would.
  bool coincidentSkipSafe(vtkActor *act) const;
  // VTK 9.5.0's vtkOpenGLLowMemoryCellTypeAgent::PreDraw / Draw for cell type t.
  void agentPreDraw(int t, vtkRenderer *ren, vtkActor *act);
  void agentDraw(int t, vtkRenderer *ren, vtkActor *act);

  // The sources a program was looked up from (this->Shaders'), one copy per
  // program however many kept shaders hold it: the shader cache has one program
  // per distinct source, so a thousand casters drawn alike keep one copy, not a
  // thousand (~16 KB each).
  struct ShaderSources;
  // Pass programs: the shader built for one set of render passes, kept while
  // the mapper draws under another.
  struct PassProgram {
    std::vector<const vtkObjectBase *> passes;        // the set, in the actor's order
    std::vector<vtkWeakPointer<vtkObjectBase>> alive; // the same passes: null once one is gone
    vtkSmartPointer<vtkShaderProgram> program;
    std::shared_ptr<const ShaderSources> sources; // what `program` was looked up from
    std::vector<vtkSmartPointer<vtkObject>> mods; // this->GLSLMods' items
    vtkTimeStamp built;                           // ShaderBuildTimeStamp
    vtkWeakPointer<vtkOpenGLShaderCache> cache;   // readyProgram's m_cache ...
    vtkMTimeType lookupStamp = 0;                 // ... and m_builtStamp
    std::vector<const vtkTexture *> textures;     // the actor's, when built
  };
  std::shared_ptr<const ShaderSources> sharedSources(vtkShaderProgram *program, ShaderMap &shaders);
  // The settings this mapper draws by: its app's, or (none set) its own defaults.
  Settings &settings();
  // Before VTK's RenderPieceStart: keep the current shader if this draw's pass
  // set is another, and put back the one kept for this draw's set if it is
  // still good.
  void swapPassProgram(vtkActor *act);
  // The render passes on the actor (vtkOpenGLRenderPass::RenderPasses), in order.
  static std::vector<const vtkObjectBase *> actorPasses(vtkActor *act);
  std::vector<const vtkTexture *> actorTextures(vtkActor *act);
  bool sameDeclarations() const;

  vtkWeakPointer<vtkOpenGLShaderCache> m_cache; // cache the program was last resolved from
  vtkMTimeType m_builtStamp = 0;                // ShaderBuildTimeStamp at that lookup
  Stats m_stats;
  // The pass set the current shader was built for (and, weakly, its passes, taken
  // while they were alive), the actor's textures when it was built, and the
  // shaders kept for other sets.
  std::vector<const vtkObjectBase *> m_activePasses;
  std::vector<vtkWeakPointer<vtkObjectBase>> m_activeAlive;
  std::vector<const vtkTexture *> m_activeTextures;
  std::vector<PassProgram> m_passPrograms;
  // What every shader depends on beside the passes, as the last draw left it: a
  // change rebuilds the current shader (VTK) and makes the kept ones stale.
  std::vector<vtkOpenGLShaderDeclaration> m_declarations;
  int m_colorSource = -1, m_normalSource = -1;
  std::shared_ptr<Settings> m_settings; // setApp's (settings())

private:
  LowMemoryPolyDataMapper(const LowMemoryPolyDataMapper &) = delete;
  void operator=(const LowMemoryPolyDataMapper &) = delete;
};

// Which mapper newPolyDataMapper(app) returns for `app`. The initial value comes
// from the CVCGL_LOWMEM_MAPPER environment variable (auto | force | off, default
// auto), read when the app's settings are first used:
//   Auto   LowMemoryPolyDataMapper wherever VTK's factory would hand out a
//          vtkOpenGLLowMemoryPolyDataMapper (GLES3/WebGL2), the factory's mapper
//          everywhere else (vtkOpenGLPolyDataMapper on desktop GL).
//   Force  LowMemoryPolyDataMapper everywhere, desktop GL included -- for native
//          tests and GL-call measurement of the WebGL path.
//   Off    the factory's mapper everywhere.
enum class LowMemoryMapperPolicy : int { Auto = 0, Force = 1, Off = 2 };
void setLowMemoryMapperPolicy(cvc::app &app, LowMemoryMapperPolicy policy);
LowMemoryMapperPolicy lowMemoryMapperPolicy(cvc::app &app);

// The poly-data mapper for a cvcGL node on `app`, chosen by the app's policy above (a
// LowMemoryPolyDataMapper drawing by the app's settings).
vtkSmartPointer<vtkPolyDataMapper> newPolyDataMapper(cvc::app &app);

} // namespace gl
} // namespace cvc

#endif // CVC_GL_LOW_MEMORY_POLY_DATA_MAPPER_H
