#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/mesh_ops.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LowMemoryPolyDataMapper.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/line_normals.h>
#include <cvc/image/image.h>
#include <cvc/state/state.h>
#include <limits>
#include <set>
#include <sstream>
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkLine.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLState.h>
#include <vtkOpenGLTexture.h>
#include <vtkOpenGLVertexBufferObject.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkShaderProgram.h>
#include <vtkShaderProperty.h>
#include <vtkTexture.h>
#include <vtkTextureObject.h>
#include <vtkTextureUnitManager.h>
#include <vtkTransform.h>
#include <vtkUniforms.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertex.h>
#include <vtk_glad.h>

namespace cvc {
namespace gl {

GeometryNode::GeometryNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GeometryNode(ctx, statePath, name, newPolyDataMapper()) {}

GeometryNode::GeometryNode(cvc::app &ctx, const std::string &statePath, const std::string &name,
                           vtkSmartPointer<vtkPolyDataMapper> mapper)
    : GraphicsNode(ctx, statePath, name), m_hasGeometry(false),
      m_renderMode(GeometryRenderMode::TRIS), m_useSingleColor(false),
      m_scalarMin(std::numeric_limits<double>::quiet_NaN()),
      m_scalarMax(std::numeric_limits<double>::quiet_NaN()),
      m_scalarRangeUsed(std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::quiet_NaN()),
      m_actor(vtkSmartPointer<vtkActor>::New()), m_mapper(mapper ? mapper : newPolyDataMapper()),
      m_polyData(vtkSmartPointer<vtkPolyData>::New()), m_textureFlipV(false) {
  m_mapper->SetInputData(m_polyData);
  m_actor->SetMapper(m_mapper);

  // Set default material properties
  m_actor->GetProperty()->SetColor(0.8, 0.8, 0.9);
  m_actor->GetProperty()->SetSpecular(0.3);
  m_actor->GetProperty()->SetSpecularPower(20);

  // Initialize state tree with all rendering attributes
  if (!statePath.empty()) {
    getState("visible").value(1); // Visible by default

    // Render mode
    getState("render_mode").value(renderModeToString(m_renderMode));

    // Single color mode (default: false - use per-vertex colors if available)
    getState("use_single_color").value(false);

    // Material color (RGB 0-1)
    getState("color_r").value(0.8);
    getState("color_g").value(0.8);
    getState("color_b").value(0.9);

    // Material properties
    getState("specular").value(0.3);
    getState("specular_power").value(20.0);
    getState("ambient").value(0.0); // VTK default
    getState("diffuse").value(1.0); // VTK default
    getState("opacity").value(1.0);

    // Point/line rendering properties
    getState("point_size").value(3.0);
    getState("line_width").value(1.0);

    // Scalar-field colouring (see setScalarField); empty range = auto
    getState("color_by").value(m_colorBy);
    getState("colormap").value(std::string("viridis")); // m_colorMap
    getState("scalar_min").value(std::string());
    getState("scalar_max").value(std::string());
  }
}

GeometryNode::~GeometryNode() {
  m_dataConnection.disconnect();
  // Detach the UpdateShaderEvent binder: the ref-counted mapper/actor can outlive
  // this node (still registered with a renderer), and a later draw would fire the
  // callback into freed member maps (see setShaderTexture). Mirrors the FpsHud /
  // ImGuiOverlay observer-teardown pattern.
  if (m_shaderTexObserverInstalled && m_mapper && m_shaderTexCb)
    m_mapper->RemoveObserver(m_shaderTexCb);
}

void GeometryNode::applyTransformToVTK() {
  // Use generic helper to apply world transform
  applyWorldTransformToProps({m_actor});
}

void GeometryNode::applyClipPlanes(vtkPlaneCollection *planes) {
  if (m_mapper) {
    if (planes && planes->GetNumberOfItems() > 0) {
      m_mapper->SetClippingPlanes(planes);
    } else {
      // Detach, never RemoveAllClippingPlanes(): that empties the collection the
      // mapper holds -- the parent's own, shared by all its children -- so
      // clipping could never be switched back on.
      m_mapper->SetClippingPlanes(static_cast<vtkPlaneCollection *>(nullptr));
    }
  }
}

int GeometryNode::maxClipPlanes() const {
  // vtkOpenGLPolyDataMapper::ReplaceShaderClip: "OpenGL has a limit of 6
  // clipping planes". The low-memory mapper (and cvcGL's subclasses of it) emits
  // no clip code at all.
  if (!m_mapper || m_mapper->IsA("vtkOpenGLLowMemoryPolyDataMapper"))
    return 0;
  return 6;
}

void GeometryNode::handleStateChanged(const std::string &childState) {
  // Handle geometry-specific state changes
  // All VTK operations MUST be wrapped in runOnMainThread() for thread safety
  if (childState == "render_mode") {
    runOnMainThread([this]() {
      std::string renderModeStr = getState("render_mode").value<std::string>();
      GeometryRenderMode newMode = stringToRenderMode(renderModeStr);
      if (m_renderMode != newMode) {
        m_renderMode = newMode;
        updateRenderModeVTK();
      }
    });
  } else if (childState == "color_r" || childState == "color_g" || childState == "color_b") {
    runOnMainThread([this]() {
      // Only update if all color components can be read and actor exists
      if (!m_actor)
        return;
      try {
        double r = getState("color_r").value<double>();
        double g = getState("color_g").value<double>();
        double b = getState("color_b").value<double>();
        m_actor->GetProperty()->SetColor(r, g, b);
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - values not fully initialized yet
      }
    });
  } else if (childState == "specular") {
    runOnMainThread([this]() {
      if (!m_actor)
        return;
      double specular = getState("specular").value<double>();
      m_actor->GetProperty()->SetSpecular(specular);
    });
  } else if (childState == "specular_power") {
    runOnMainThread([this]() {
      if (!m_actor)
        return;
      double specularPower = getState("specular_power").value<double>();
      m_actor->GetProperty()->SetSpecularPower(specularPower);
    });
  } else if (childState == "ambient") {
    runOnMainThread([this]() {
      if (!m_actor)
        return;
      double ambient = getState("ambient").value<double>();
      m_actor->GetProperty()->SetAmbient(ambient);
    });
  } else if (childState == "diffuse") {
    runOnMainThread([this]() {
      if (!m_actor)
        return;
      double diffuse = getState("diffuse").value<double>();
      m_actor->GetProperty()->SetDiffuse(diffuse);
    });
  } else if (childState == "opacity") {
    runOnMainThread([this]() {
      if (!m_actor)
        return;
      double opacity = getState("opacity").value<double>();
      m_actor->GetProperty()->SetOpacity(opacity);
    });
  } else if (childState == "point_size") {
    runOnMainThread([this]() {
      if (!m_actor)
        return;
      double pointSize = getState("point_size").value<double>();
      m_actor->GetProperty()->SetPointSize(pointSize);
    });
  } else if (childState == "line_width") {
    runOnMainThread([this]() {
      if (!m_actor)
        return;
      double lineWidth = getState("line_width").value<double>();
      m_actor->GetProperty()->SetLineWidth(lineWidth);
    });
  } else if (childState == "use_single_color") {
    runOnMainThread([this]() {
      try {
        bool useSingleColor = getState("use_single_color").value<bool>();
        if (m_useSingleColor != useSingleColor) {
          m_useSingleColor = useSingleColor;
          // Re-apply geometry colors
          if (m_hasGeometry && m_geometry) {
            updatePolyData(*m_geometry);
          }
        }
      } catch (...) {
        // Ignore if state not available
      }
    });
  } else if (childState == "color_by" || childState == "colormap" || childState == "scalar_min" ||
             childState == "scalar_max") {
    runOnMainThread([this, childState]() {
      if (readScalarState(childState))
        recolor();
    });
  } else {
    // Delegate to parent for common graphics fields
    // Parent will handle its own runOnMainThread wrapping
    GraphicsNode::handleStateChanged(childState);
  }
}

std::string GeometryNode::renderModeToString(GeometryRenderMode mode) {
  return std::to_string(static_cast<int>(mode));
}

GeometryRenderMode GeometryNode::stringToRenderMode(const std::string &str) {
  try {
    int mode = std::stoi(str);
    if (mode >= 0 && mode <= 5) {
      return static_cast<GeometryRenderMode>(mode);
    }
  } catch (...) {
  }
  return GeometryRenderMode::TRIS; // Default
}

void GeometryNode::setRenderMode(GeometryRenderMode mode) {
  if (m_renderMode == mode)
    return;

  m_renderMode = mode;

  // Update state tree
  getState("render_mode").value(renderModeToString(mode));

  // Update VTK rendering on main thread
  runOnMainThread([this]() { updateRenderModeVTK(); });
}

void GeometryNode::updateRenderModeVTK() {
  // Guard: Don't update VTK if actor not initialized
  if (!m_actor)
    return;

  // Update VTK rendering based on mode
  switch (m_renderMode) {
  case GeometryRenderMode::POINTS:
    m_actor->GetProperty()->SetRepresentationToPoints();
    m_actor->GetProperty()->SetPointSize(getState("point_size").value<double>());
    break;

  case GeometryRenderMode::LINES:
    m_actor->GetProperty()->SetRepresentationToWireframe();
    m_actor->GetProperty()->SetLineWidth(getState("line_width").value<double>());
    break;

  case GeometryRenderMode::TRIS:
  case GeometryRenderMode::QUADS:
    m_actor->GetProperty()->SetRepresentationToSurface();
    break;

  case GeometryRenderMode::TETS:
    // The tet edges until updatePolyData finds the boundary surface (it switches
    // the representation to surface then).
    m_actor->GetProperty()->SetRepresentationToWireframe();
    break;

  case GeometryRenderMode::HEXS:
    // Placeholder: For now, render as wireframe
    // TODO: Implement proper volumetric mesh rendering
    m_actor->GetProperty()->SetRepresentationToWireframe();
    break;
  }

  // Trigger re-render if we have geometry
  if (m_hasGeometry && m_geometry) {
    updatePolyData(*m_geometry);
  }

  // Mark everything as modified to trigger re-render
  if (m_polyData)
    m_polyData->Modified();
  if (m_mapper)
    m_mapper->Modified();
  if (m_actor)
    m_actor->Modified();

  // Request a render update — flag-only; see GraphicsNode::handleStateChanged
  // for why a synchronous Render() here is catastrophic under per-frame
  // property animation (setColor alone writes three state keys).
  if (SceneGraph *sg = getSceneGraph()) {
    sg->requestRender();
  } else if (m_renderer && m_renderer->GetRenderWindow()) {
    m_renderer->GetRenderWindow()->Render();
  }
}

void GeometryNode::setColor(double r, double g, double b) {
  getState("color_r").value(r);
  getState("color_g").value(g);
  getState("color_b").value(b);
}

void GeometryNode::setSpecular(double value) { getState("specular").value(value); }

void GeometryNode::setSpecularPower(double value) { getState("specular_power").value(value); }

void GeometryNode::setAmbient(double value) { getState("ambient").value(value); }

void GeometryNode::setDiffuse(double value) { getState("diffuse").value(value); }

void GeometryNode::setOpacity(double value) { getState("opacity").value(value); }

void GeometryNode::setPointSize(double size) { getState("point_size").value(size); }

void GeometryNode::setLineWidth(double width) { getState("line_width").value(width); }

void GeometryNode::setUseSingleColor(bool useSingleColor) {
  if (m_useSingleColor == useSingleColor)
    return;

  m_useSingleColor = useSingleColor;
  getState("use_single_color").value(useSingleColor);

  // Re-apply geometry colors on main thread
  runOnMainThread([this]() {
    if (m_hasGeometry && m_geometry) {
      updatePolyData(*m_geometry);
    }
  });
}

vtkProp *GeometryNode::getProp() { return m_actor; }

vtkActor *GeometryNode::actor() const { return m_actor; }

vtkPolyDataMapper *GeometryNode::mapper() const { return m_mapper; }

vtkPolyData *GeometryNode::polyData() const { return m_polyData; }

void GeometryNode::setTexture(const cvc::image &img, bool zeroCopy) {
  if (!m_actor)
    return;
  if (img.empty()) {
    clearTexture();
    return;
  }

  // CRITICAL: the VTK work below — vtkImageData/vtkTexture creation, updatePolyData()'s
  // mutation of the shared m_polyData, and m_actor->SetTexture() — must run on the owner
  // thread. setTexture() is exposed to Python (pycvc_gl) and callable from any thread;
  // racing the renderer on m_polyData/m_actor corrupts frames or crashes (the same reason
  // setGeometry()/setColor()/setRenderMode() all marshal). Capture the image BY VALUE so
  // its (COW-shared) buffer stays alive until the marshaled lambda runs — and, in the
  // zero-copy path, for the texture's lifetime via m_textureStorage.
  runOnMainThread([this, img, zeroCopy]() {
    // Alias the image's RGBA8 buffer directly when possible (zero-copy); otherwise
    // convert/flip/copy into a fresh vtkImageData (fallback).
    const bool canAlias = zeroCopy && img.format() == cvc::image::pixel_format::RGBA &&
                          img.type() == cvc::image::data_type::u8;

    auto id = vtkSmartPointer<vtkImageData>::New();
    bool mipmap = true;
    if (canAlias) {
      // ── Zero-copy: the vtkTexture samples the SAME bytes the cvc::image owns (no
      // memcpy). The GeometryNode holds a ref to the buffer (m_textureStorage) for
      // the texture's lifetime, so an in-place pixel edit through an aliased view
      // (pycvc image.numpy()) + texture_modified() shows live with no re-copy. The
      // top-left-vs-bottom-left origin mismatch is resolved by flipping the TCoords'
      // V (below), NOT the pixels, so the aliasing is preserved.
      cvc::image shared = img;                                     // shares the buffer (COW)
      boost::shared_array<unsigned char> store = shared.storage(); // non-detaching owner
      const vtkIdType n = static_cast<vtkIdType>(shared.size_bytes());

      auto arr = vtkSmartPointer<vtkUnsignedCharArray>::New();
      arr->SetNumberOfComponents(4);
      arr->SetArray(store.get(), n, /*save=*/1); // save=1: VTK must not free our buffer

      id->SetDimensions(shared.width(), shared.height(), 1);
      id->GetPointData()->SetScalars(arr); // wrap the aliased buffer as RGBA scalars

      m_textureStorage = store; // keep the aliased buffer alive
      m_textureFlipV = true;    // flip the TCoords, not the pixels
      // No mipmaps: they would be rebuilt from the input on every edit; a live
      // texture stays crisp and cheap without them.
      mipmap = false;
    } else {
      // ── Fallback copy: convert to RGBA8 + flip the PIXELS, then memcpy. ────────
      cvc::image rgba =
          (img.format() == cvc::image::pixel_format::RGBA &&
           img.type() == cvc::image::data_type::u8)
              ? img
              : img.converted(cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
      rgba = rgba.flipped_vertical();
      id->SetDimensions(rgba.width(), rgba.height(), 1);
      id->AllocateScalars(VTK_UNSIGNED_CHAR, 4);
      std::memcpy(id->GetScalarPointer(), rgba.data(), rgba.size_bytes());
      m_textureStorage.reset(); // nothing aliased to keep alive
      m_textureFlipV = false;   // pixels already flipped -> TCoords must not be
    }

    // Regenerate the polydata TCoords to honor the (possibly just-changed) flip
    // flag before attaching the texture; idempotent across repeated setTexture.
    if (m_hasGeometry && m_geometry)
      updatePolyData(*m_geometry);

    auto tex = vtkSmartPointer<vtkTexture>::New();
    tex->SetInputData(id);
    tex->InterpolateOn();
    if (mipmap)
      tex->MipmapOn();
    else
      tex->MipmapOff();
    m_texture = tex;
    m_textureImageData = id;
    m_actor->SetTexture(tex);
    // The texture supplies the surface color; geom.colors() would tint it. Set this
    // LAST -- the updatePolyData above ran before m_texture was set, so it may have
    // enabled scalar visibility for a colored mesh. The rule is applyVertexColors':
    // only a scalar field (an explicit request) still shows over the texture.
    m_mapper->SetScalarVisibility(m_scalarActive && !m_useSingleColor && m_polyData &&
                                  m_polyData->GetPointData()->GetScalars() != nullptr);
    m_actor->Modified();
  });
}

void GeometryNode::clearTexture() {
  // Marshaled to the owner thread: mutates shared VTK state (m_actor's texture and,
  // via updatePolyData, the shared m_polyData) that the renderer reads.
  runOnMainThread([this]() {
    const bool hadTexture = m_texture.Get() != nullptr;
    m_texture = nullptr;
    m_textureImageData = nullptr;
    m_textureStorage.reset();
    if (m_actor) {
      m_actor->SetTexture(nullptr);
      m_actor->Modified();
    }
    // Restore un-flipped TCoords now that no top-left texture is active, and
    // with them the per-vertex colours the texture was hiding (applyVertexColors).
    const bool reflip = m_textureFlipV;
    m_textureFlipV = false;
    if (m_hasGeometry && m_geometry) {
      if (reflip)
        updatePolyData(*m_geometry);
      else if (hadTexture)
        recolor();
    }
  });
}

void GeometryNode::texture_modified() {
  // The pixels were edited in place through the aliased buffer; bump the MTime of
  // the texture and its input image data so VTK re-samples on the next render — no
  // data re-copy. Marshaled to the owner thread: it bumps shared VTK MTime state the
  // renderer reads, and the caller may be a Python worker thread.
  runOnMainThread([this]() {
    if (m_textureImageData) {
      if (vtkDataArray *scalars = m_textureImageData->GetPointData()->GetScalars())
        scalars->Modified();
      m_textureImageData->Modified();
    }
    if (m_texture)
      m_texture->Modified();
    if (m_actor)
      m_actor->Modified();
  });
}

void GeometryNode::texture_modified_rows(int row0, int row1) {
  // Full-width rows: INT_MAX is clamped to the width below.
  texture_modified_rect(0, row0, std::numeric_limits<int>::max(), row1);
}

void GeometryNode::texture_modified_rect(int x0, int y0, int x1, int y1) {
  runOnMainThread([this, x0, y0, x1, y1]() {
    if (!m_texture || !m_textureImageData)
      return;
    if (!m_textureStorage) {
      // The copy path: the texture holds its own flipped copy, which the
      // caller's edit never reached. Keep texture_modified()'s behaviour.
      texture_modified();
      return;
    }
    int dims[3] = {0, 0, 0};
    m_textureImageData->GetDimensions(dims);
    const int w = dims[0], h = dims[1];
    const int cx0 = std::max(0, x0), cy0 = std::max(0, y0);
    const int cx1 = std::min(w, x1), cy1 = std::min(h, y1);
    if (cx0 >= cx1 || cy0 >= cy1)
      return;

    // The texture vtkOpenGLTexture::Load made from this buffer: RGBA8 (the
    // zero-copy path is RGBA8 only), w x h, no mipmaps, row r of the image is
    // row r of the texture (the V flip lives in the TCoords, not the pixels).
    auto *otex = vtkOpenGLTexture::SafeDownCast(m_texture);
    vtkTextureObject *to = otex ? otex->GetTextureObject() : nullptr;
    vtkOpenGLRenderWindow *ctx = to ? to->GetContext() : nullptr;
    if (!to || !ctx || to->GetHandle() == 0)
      return; // not on the GPU yet: its first Load uploads every pixel, edits included
    if (static_cast<int>(to->GetWidth()) != w || static_cast<int>(to->GetHeight()) != h ||
        to->GetComponents() != 4) {
      texture_modified(); // not the texture we think it is: re-upload the lot
      return;
    }

    if (!ctx->IsCurrent())
      ctx->MakeCurrent();
    vtkOpenGLState *gl = ctx->GetState();
    to->Activate();
    // Client memory, not a pixel-unpack buffer (VTK's GLES texture-buffer
    // emulation binds one while it uploads).
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    gl->vtkglPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    const bool fullRows = cx0 == 0 && cx1 == w;
    if (!fullRows)
      gl->vtkglPixelStorei(GL_UNPACK_ROW_LENGTH, w); // stride of the whole image
    const std::size_t first = (static_cast<std::size_t>(cy0) * w + cx0) * 4;
    glTexSubImage2D(to->GetTarget(), 0, cx0, cy0, cx1 - cx0, cy1 - cy0, GL_RGBA, GL_UNSIGNED_BYTE,
                    m_textureStorage.get() + first);
    if (!fullRows)
      gl->vtkglPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    to->Deactivate();

    // Nothing was marked modified, so nothing else will ask for the frame.
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::setGeometry(const cvc::geometry &geom) {
  cvc::thread_info ti(app(), BOOST_CURRENT_FUNCTION);

  // CRITICAL: Entire method must run on main thread to avoid Qt threading errors
  // Even creating std::shared_ptr or setting member variables can trigger VTK
  // smart pointer operations that touch Qt objects
  runOnMainThread([this, geom]() {
    // Store the geometry object
    m_geometry = std::make_shared<cvc::geometry>(geom);
    m_hasGeometry = true; // Set this BEFORE setRenderMode so it can update

    // Derived from the previous geometry: computed curvatures, the tet boundary,
    // and an explicit scalar field that no longer has one value per point.
    m_curvatures.clear();
    m_curvaturesTried = false;
    m_tetSurface.clear();
    m_tetSurfaceTried = false;
    if (m_hasScalarField && m_scalarField.size() != geom.num_points()) {
      app().log(1, "GeometryNode::setGeometry[" + getName() + "]: the scalar field has " +
                       std::to_string(m_scalarField.size()) + " values for " +
                       std::to_string(geom.num_points()) + " points — dropping it");
      m_hasScalarField = false;
      m_scalarField.clear();
    }

    // Auto-detect render mode from geometry type
    GeometryRenderMode autoMode = GeometryRenderMode::TRIS; // default

    switch (geom.get_geometry_type()) {
    case cvc::geometry::SURFACE_TRI:
      autoMode = GeometryRenderMode::TRIS;
      break;
    case cvc::geometry::SURFACE_QUAD:
      autoMode = GeometryRenderMode::QUADS;
      break;
    case cvc::geometry::VOLUME_TET:
      autoMode = GeometryRenderMode::TETS;
      break;
    case cvc::geometry::VOLUME_HEX:
      autoMode = GeometryRenderMode::HEXS;
      break;
    case cvc::geometry::MIXED:
      // For mixed, prefer tris if available, otherwise quads
      if (geom.num_tris() > 0) {
        autoMode = GeometryRenderMode::TRIS;
      } else if (geom.num_quads() > 0) {
        autoMode = GeometryRenderMode::QUADS;
      } else if (geom.num_tets() > 0) {
        autoMode = GeometryRenderMode::TETS;
      } else if (geom.num_hexs() > 0) {
        autoMode = GeometryRenderMode::HEXS;
      }
      break;
    }

    // Update render mode and geometry data
    m_renderMode = autoMode;
    getState("render_mode").value(renderModeToString(autoMode));
    updateRenderModeVTK(); // This calls updatePolyData() internally
    ownBoundsChanged();

    updateMetadata(geom);
    // The mesh's extent may have changed: cameras re-fit their clipping range.
    if (SceneGraph *sg = getSceneGraph())
      sg->markContentChanged();

    // Notify parent to resync bounds if it's a NullGraphicNode with auto-sync enabled
    if (m_parent) {
      auto nullParent = dynamic_cast<NullGraphicNode *>(m_parent);
      if (nullParent) {
        nullParent->syncBoundsToChildren();
      }
    }
  });
}

void GeometryNode::updateVertices(const std::vector<double> &xyz) {
  cvc::thread_info ti(app(), BOOST_CURRENT_FUNCTION);

  // Topology-preserving fast path (docs/RENDER_PERFORMANCE.md fix #3, route C):
  // overwrite the existing vtkPoints coordinates in place and mark modified, so
  // the next frame re-uploads the mesh WITHOUT rebuilding cells/colours/tcoords or
  // re-running ensureNormals (the expensive part of setGeometry). `xyz` is flat
  // [x,y,z, ...] with the SAME point count as the current geometry; a mismatch
  // logs and no-ops rather than corrupt the mesh. Normals stay at bind pose on
  // purpose — the shadow map is depth (positions), which ARE updated, so shadows
  // track the sway; the small shading error from a per-frame rotation is not worth
  // a vtkPolyDataNormals pass per frame. Runs on the main thread like setGeometry.
  runOnMainThread([this, xyz]() {
    if (!m_polyData)
      return;
    vtkPoints *pts = m_polyData->GetPoints();
    const size_t n = xyz.size() / 3;
    if (!pts || static_cast<size_t>(pts->GetNumberOfPoints()) != n) {
      app().log(1, "GeometryNode::updateVertices[" + getName() + "]: point-count mismatch (" +
                       std::to_string(n) + " given, " +
                       std::to_string(pts ? pts->GetNumberOfPoints() : 0) +
                       " in mesh) — ignoring; call setGeometry to change topology");
      return;
    }
    // Bulk raw-buffer write. vtkPoints::SetPoint is a per-point virtual +
    // type-dispatch call; for the per-frame agent pack (N x a multi-thousand-vert
    // Humvee streamed every frame) that loop dominates the CPU frame time. When the
    // backing array is the common contiguous float or double 3-tuple layout,
    // overwrite its buffer directly in one tight pass with no virtual dispatch;
    // only fall back to SetPoint for an exotic array type. vtkPoints defaults to a
    // vtkFloatArray, so the float path is the one that runs here.
    vtkDataArray *da = pts->GetData();
    const size_t n3 = n * 3;
    if (auto *fa = vtkFloatArray::SafeDownCast(da)) {
      float *dst = fa->GetPointer(0);
      for (size_t i = 0; i < n3; ++i)
        dst[i] = static_cast<float>(xyz[i]);
    } else if (auto *dbl = vtkDoubleArray::SafeDownCast(da)) {
      std::copy(xyz.begin(), xyz.end(), dbl->GetPointer(0));
    } else {
      for (size_t i = 0; i < n; ++i)
        pts->SetPoint(static_cast<vtkIdType>(i), xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    }
    pts->Modified();
    m_polyData->Modified();
    // Request a redraw the same way handleStateChanged does — never render
    // synchronously here (see GraphicsNode). The host frame loop drains it.
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::updateColors(const std::vector<unsigned char> &rgb) {
  cvc::thread_info ti(app(), BOOST_CURRENT_FUNCTION);

  // The color twin of updateVertices: overwrite the existing per-vertex uchar RGB
  // scalar array in place and mark modified — one buffer re-upload, no rebuild of
  // cells/normals/tcoords. Requires a mesh that already has per-vertex colors
  // (setUseSingleColor(false) + colored setGeometry) with a matching point count.
  runOnMainThread([this, rgb]() {
    if (!m_polyData)
      return;
    auto *colors = vtkUnsignedCharArray::SafeDownCast(m_polyData->GetPointData()->GetScalars());
    const size_t n = rgb.size() / 3;
    if (!colors || colors->GetNumberOfComponents() != 3 ||
        static_cast<size_t>(colors->GetNumberOfTuples()) != n) {
      app().log(1, "GeometryNode::updateColors[" + getName() + "]: color-array mismatch (" +
                       std::to_string(n) + " RGB tuples given, " +
                       std::to_string(colors ? colors->GetNumberOfTuples() : 0) +
                       " in mesh) — ignoring; needs a per-vertex-coloured mesh with the same "
                       "point count");
      return;
    }
    std::memcpy(colors->GetPointer(0), rgb.data(), rgb.size());
    colors->Modified();
    m_polyData->Modified();
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::updateNormals(const std::vector<double> &xyz) {
  cvc::thread_info ti(app(), BOOST_CURRENT_FUNCTION);

  // The normal twin of updateVertices: overwrite the existing per-vertex normals
  // in place and mark modified — one buffer re-upload, no rebuild of cells /
  // colours / tcoords. For a CPU-deformed surface (an FFT ocean) whose normals
  // must track the displacement so it shades and reflects correctly, without a
  // per-frame vtkPolyDataNormals pass. `xyz` is flat [nx,ny,nz, ...], unit-length,
  // with the SAME point count as the current geometry; a mismatch logs and no-ops.
  runOnMainThread([this, xyz]() {
    if (!m_polyData)
      return;
    vtkDataArray *normals = m_polyData->GetPointData()->GetNormals();
    const size_t n = xyz.size() / 3;
    if (!normals || normals->GetNumberOfComponents() != 3 ||
        static_cast<size_t>(normals->GetNumberOfTuples()) != n) {
      app().log(1, "GeometryNode::updateNormals[" + getName() + "]: normal-array mismatch (" +
                       std::to_string(n) + " given, " +
                       std::to_string(normals ? normals->GetNumberOfTuples() : 0) +
                       " in mesh) — ignoring; needs a mesh that already carries normals");
      return;
    }
    for (size_t i = 0; i < n; ++i)
      normals->SetTuple3(static_cast<vtkIdType>(i), xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    normals->Modified();
    m_polyData->Modified();
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

// ── Scalar-field colouring ────────────────────────────────────────────────────
namespace {
const double kAutoRange = std::numeric_limits<double>::quiet_NaN();

// NaN-aware equality: two "auto" bounds are the same bound.
bool sameBound(double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); }

// A range bound as stored in the state tree: empty = auto.
std::string boundToState(double v) {
  return std::isfinite(v) ? boost::lexical_cast<std::string>(v) : std::string();
}

double boundFromState(const std::string &s) {
  try {
    const double v = boost::lexical_cast<double>(s);
    return std::isfinite(v) ? v : kAutoRange;
  } catch (const boost::bad_lexical_cast &) {
    return kAutoRange; // empty or non-numeric
  }
}

bool isColorBy(const std::string &s) {
  return s == "none" || s == "colors" || s == "function" || s == "k1" || s == "k2" || s == "mean" ||
         s == "gaussian";
}
} // namespace

void GeometryNode::setScalarField(const std::vector<double> &perVertex, cvc::colormap_kind cm,
                                  double lo, double hi) {
  cvc::thread_info ti(app(), BOOST_CURRENT_FUNCTION);
  runOnMainThread([this, field = perVertex, cm, lo, hi]() mutable {
    const size_t n = m_geometry ? m_geometry->num_points() : 0;
    if (!m_hasGeometry || field.size() != n) {
      app().log(1, "GeometryNode::setScalarField[" + getName() +
                       "]: " + std::to_string(field.size()) + " values for " + std::to_string(n) +
                       " points — ignoring; needs one value per point of the current geometry");
      return;
    }
    m_scalarField = std::move(field);
    m_hasScalarField = true;
    m_colorMap = cm;
    m_scalarMin = std::isfinite(lo) ? lo : kAutoRange;
    m_scalarMax = std::isfinite(hi) ? hi : kAutoRange;
    writeScalarState();
    recolor();
  });
}

void GeometryNode::setScalarRange(double lo, double hi) {
  runOnMainThread([this, lo, hi]() {
    const double a = std::isfinite(lo) ? lo : kAutoRange;
    const double b = std::isfinite(hi) ? hi : kAutoRange;
    const bool changed = !sameBound(a, m_scalarMin) || !sameBound(b, m_scalarMax);
    m_scalarMin = a;
    m_scalarMax = b;
    writeScalarState();
    if (changed)
      recolor();
  });
}

void GeometryNode::setColorMap(cvc::colormap_kind cm) {
  runOnMainThread([this, cm]() {
    const bool changed = cm != m_colorMap;
    m_colorMap = cm;
    writeScalarState();
    if (changed)
      recolor();
  });
}

void GeometryNode::clearScalarField() {
  runOnMainThread([this]() {
    if (!m_hasScalarField)
      return;
    m_hasScalarField = false;
    std::vector<double>().swap(m_scalarField);
    recolor();
  });
}

// Mirror the members into the state tree. The handlers this fires find nothing
// changed (readScalarState), so the caller recolours exactly once.
void GeometryNode::writeScalarState() {
  getState("colormap").value(cvc::to_string(m_colorMap));
  getState("scalar_min").value(boundToState(m_scalarMin));
  getState("scalar_max").value(boundToState(m_scalarMax));
}

// Take one scalar-colouring state key into its member; true if it changed.
bool GeometryNode::readScalarState(const std::string &key) {
  std::string v;
  try {
    v = getState(key).value<std::string>();
  } catch (...) {
    return false;
  }
  if (key == "color_by") {
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (v.empty())
      v = "none";
    if (!isColorBy(v)) {
      app().log(1, "GeometryNode[" + getName() + "]: unknown color_by '" + v +
                       "' (none, colors, function, k1, k2, mean, gaussian) — ignoring");
      return false;
    }
    if (v == m_colorBy)
      return false;
    m_colorBy = v;
    return true;
  }
  if (key == "colormap") {
    cvc::colormap_kind k;
    if (v.empty())
      return false;
    if (!cvc::colormap_from_string(v, k)) {
      app().log(1, "GeometryNode[" + getName() + "]: unknown colormap '" + v + "' — ignoring");
      return false;
    }
    if (k == m_colorMap)
      return false;
    m_colorMap = k;
    return true;
  }
  double &bound = key == "scalar_min" ? m_scalarMin : m_scalarMax;
  const double b = boundFromState(v);
  if (sameBound(b, bound))
    return false;
  bound = b;
  return true;
}

// Re-colour after a scalar-colouring change: the colour array only, in place
// when its size allows -- cells, points, normals and tcoords are untouched.
void GeometryNode::recolor() {
  if (!m_hasGeometry || !m_geometry || !m_polyData) {
    m_scalarActive = false;
    m_scalarRangeUsed = {kAutoRange, kAutoRange};
    return;
  }
  if (static_cast<size_t>(m_polyData->GetNumberOfPoints()) != m_geometry->num_points())
    updatePolyData(*m_geometry);
  else
    applyVertexColors(*m_geometry, /*inPlace=*/true);
  m_polyData->Modified();
  if (SceneGraph *sg = getSceneGraph())
    sg->requestRender();
}

// The field to colour by, if any: the explicit one, else the color_by
// selection. `robust` asks for a percentile auto-range (the curvature kinds).
bool GeometryNode::resolveScalarField(const cvc::geometry &geom, std::vector<double> &values,
                                      bool &robust) {
  const size_t n = geom.num_points();
  robust = false;
  if (n == 0)
    return false;
  if (m_hasScalarField) {
    if (m_scalarField.size() != n)
      return false;
    values = m_scalarField;
    return true;
  }
  if (m_colorBy == "function") {
    if (geom.functions().size() != n)
      return false;
    values.assign(geom.functions().begin(), geom.functions().end());
    return true;
  }
  const bool k1 = m_colorBy == "k1", k2 = m_colorBy == "k2", mean = m_colorBy == "mean";
  if (!k1 && !k2 && !mean && m_colorBy != "gaussian")
    return false;
  const cvc::geometry::curvatures_t *curv = vertexCurvatures(geom);
  if (!curv)
    return false;
  values.resize(n);
  for (size_t i = 0; i < n; ++i) {
    const double a = (*curv)[i][0], b = (*curv)[i][1];
    values[i] = k1 ? a : k2 ? b : mean ? 0.5 * (a + b) : a * b;
  }
  robust = true;
  return true;
}

// geom.curvatures() when it has one (k1, k2) per point; otherwise, for the
// stored geometry and with libigl, computed once by cvc::compute_curvature.
// Without surface triangles there is nothing to fit: a tet mesh is measured on
// its boundary (what TETS mode draws), with NaN -- grey, and outside the auto
// range -- for the interior points; points or lines have no curvature field at
// all, rather than an all-zero one.
const cvc::geometry::curvatures_t *GeometryNode::vertexCurvatures(const cvc::geometry &geom) {
  const size_t n = geom.num_points();
  if (geom.curvatures().size() == n)
    return &geom.curvatures();
  if (&geom != m_geometry.get() || !cvc::mesh_ops_available())
    return nullptr;
  if (!m_curvaturesTried) {
    m_curvaturesTried = true;
    try {
      cvc::geometry g(geom);        // shares geom's arrays; only what is written detaches
      std::vector<char> onBoundary; // empty: geom's own triangles are the surface
      if (geom.num_tris() + geom.num_quads() == 0) {
        cvc::geometry::tris_t scratch;
        const cvc::geometry::tris_t &boundary = tetBoundaryTris(geom, scratch);
        if (boundary.empty()) {
          app().log(1, "GeometryNode[" + getName() +
                           "]: no surface triangles — no curvature to colour by");
          return nullptr;
        }
        onBoundary.assign(n, 0);
        for (const auto &t : boundary)
          for (int c = 0; c < 3; ++c)
            if (static_cast<size_t>(t[c]) < n)
              onBoundary[static_cast<size_t>(t[c])] = 1;
        g.tris() = boundary;
      }
      cvc::compute_curvature(g);
      if (g.const_curvatures().size() == n) {
        m_curvatures = g.const_curvatures();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (size_t i = 0; i < onBoundary.size(); ++i)
          if (!onBoundary[i])
            m_curvatures[i] = cvc::geometry::curvature_t{{nan, nan}};
      }
    } catch (const std::exception &e) {
      app().log(1, "GeometryNode[" + getName() + "]: curvature failed (" + e.what() +
                       ") — not colouring by curvature");
    } catch (...) {
      app().log(1,
                "GeometryNode[" + getName() + "]: curvature failed — not colouring by curvature");
    }
  }
  return m_curvatures.size() == n ? &m_curvatures : nullptr;
}

// The per-vertex colour array: a scalar field through its colormap, else
// geom.colors(), else none (the actor's single colour). Also records what
// hasScalarField()/scalarRange() report. `inPlace` overwrites an existing
// colour array of the right size instead of replacing it (one re-upload).
//
// The colours are an UNSIGNED CHAR (0..255) array: VTK treats a 3-component
// uchar scalar array as LITERAL colors — never routed through the mapper's
// lookup table — so the direct-color behavior is intrinsic to the data type
// and can't be re-broken by a stray SetColorMode* elsewhere (or by a fresh
// mapper). A vtkFloatArray, by contrast, renders through the LUT unless
// SetColorModeToDirectScalars() is ALSO set — the omission that turned red
// meshes blue. This uchar path is also texture-ready: when UVs land later they
// go in the dedicated SetTCoords slot and never collide with these colors.
// geom.colors() are RGB doubles in [0,1] (geometry.h color_t).
void GeometryNode::applyVertexColors(const cvc::geometry &geom, bool inPlace) {
  const size_t n = geom.num_points();
  std::vector<double> field;
  bool robust = false;
  std::vector<unsigned char> rgb;
  m_scalarActive = resolveScalarField(geom, field, robust);
  m_scalarRangeUsed = {kAutoRange, kAutoRange};
  if (m_scalarActive) {
    double lo = m_scalarMin, hi = m_scalarMax;
    if (std::isnan(lo) || std::isnan(hi)) {
      std::pair<double, double> autoRange(kAutoRange, kAutoRange);
      if (robust) {
        autoRange = cvc::robust_range(field);
      } else {
        for (double v : field)
          if (std::isfinite(v)) {
            autoRange.first = std::isnan(autoRange.first) ? v : std::min(autoRange.first, v);
            autoRange.second = std::isnan(autoRange.second) ? v : std::max(autoRange.second, v);
          }
      }
      if (std::isnan(lo))
        lo = autoRange.first;
      if (std::isnan(hi))
        hi = autoRange.second;
    }
    if (lo == hi) { // a constant field: centre it rather than divide by zero
      lo -= 0.5;
      hi += 0.5;
    }
    m_scalarRangeUsed = {lo, hi};
    if (!m_useSingleColor) {
      rgb = cvc::colormap_rgb(field, m_colorMap, lo, hi);
      if (rgb.size() != 3 * n)
        rgb.clear();
    }
  }

  if (m_useSingleColor || (rgb.empty() && geom.colors().size() != n)) {
    // Use single color from actor property - clear per-vertex colors
    m_polyData->GetPointData()->SetScalars(nullptr);
    m_mapper->ScalarVisibilityOff();
    return;
  }

  vtkUnsignedCharArray *colors =
      inPlace ? vtkUnsignedCharArray::SafeDownCast(m_polyData->GetPointData()->GetScalars())
              : nullptr;
  vtkSmartPointer<vtkUnsignedCharArray> fresh;
  if (!colors || colors->GetNumberOfComponents() != 3 ||
      static_cast<size_t>(colors->GetNumberOfTuples()) != n) {
    fresh = vtkSmartPointer<vtkUnsignedCharArray>::New();
    fresh->SetNumberOfComponents(3);
    fresh->SetNumberOfTuples(n);
    fresh->SetName("Colors");
    colors = fresh;
  }

  if (!rgb.empty()) {
    std::memcpy(colors->GetPointer(0), rgb.data(), rgb.size());
  } else {
    for (size_t i = 0; i < n; ++i) {
      const auto &c = geom.colors()[i];
      unsigned char px[3] = {
          static_cast<unsigned char>(std::lround(std::clamp(c[0], 0.0, 1.0) * 255.0)),
          static_cast<unsigned char>(std::lround(std::clamp(c[1], 0.0, 1.0) * 255.0)),
          static_cast<unsigned char>(std::lround(std::clamp(c[2], 0.0, 1.0) * 255.0))};
      colors->SetTypedTuple(i, px);
    }
  }

  if (fresh)
    m_polyData->GetPointData()->SetScalars(fresh);
  else
    colors->Modified();
  // uchar scalars are used directly as colors; select point-data + enable.
  m_mapper->SetScalarModeToUsePointData();
  // A texture supplies the surface colour (setTexture): geom.colors() must not
  // tint it. A scalar field (explicit or color_by) is an explicit request and
  // still shows. The array stays in place either way, ready for clearTexture.
  m_mapper->SetScalarVisibility(!m_texture || !rgb.empty());
}

void GeometryNode::setRenderLinesAsTubes(bool on) {
  runOnMainThread([this, on]() {
    if (!m_actor)
      return;
    m_actor->GetProperty()->SetRenderLinesAsTubes(on);
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::setRenderPointsAsSpheres(bool on) {
  runOnMainThread([this, on]() {
    if (!m_actor)
      return;
    m_actor->GetProperty()->SetRenderPointsAsSpheres(on);
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::setDepthOffset(double units) {
  runOnMainThread([this, units]() {
    if (!m_mapper)
      return;
    // Per-mapper RELATIVE coincident-topology offsets (VTK's global resolve mode
    // defaults to polygon offset). Negative GL polygon-offset units are closer to
    // the camera, so a positive `units` here pulls the node toward the viewer.
    m_mapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(0.0, -units);
    m_mapper->SetRelativeCoincidentTopologyLineOffsetParameters(0.0, -units);
    m_mapper->SetRelativeCoincidentTopologyPointOffsetParameter(-units); // point form is units-only
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::addVertexShaderReplacement(const std::string &original,
                                              const std::string &replacement) {
  runOnMainThread([this, original, replacement]() {
    m_userShaderRepl[{0, original}] = replacement;
    installShaderReplacement({0, original});
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::addFragmentShaderReplacement(const std::string &original,
                                                const std::string &replacement) {
  runOnMainThread([this, original, replacement]() {
    m_userShaderRepl[{1, original}] = replacement;
    installShaderReplacement({1, original});
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::clearShaderReplacements() {
  runOnMainThread([this]() {
    m_userShaderRepl.clear();
    if (m_actor && m_actor->GetShaderProperty()) {
      m_actor->GetShaderProperty()->ClearAllVertexShaderReplacements();
      m_actor->GetShaderProperty()->ClearAllFragmentShaderReplacements();
    }
    // The subclass's own replacements are not the caller's to clear.
    for (const auto &kv : m_internalShaderRepl)
      installShaderReplacement(kv.first);
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::addInternalVertexShaderReplacement(const std::string &anchor,
                                                      const std::string &text) {
  m_internalShaderRepl[{0, anchor}] = text;
  installShaderReplacement({0, anchor});
}

void GeometryNode::addInternalFragmentShaderReplacement(const std::string &anchor,
                                                        const std::string &text) {
  m_internalShaderRepl[{1, anchor}] = text;
  installShaderReplacement({1, anchor});
}

void GeometryNode::installShaderReplacement(const ShaderReplKey &key) {
  vtkShaderProperty *sp = m_actor ? m_actor->GetShaderProperty() : nullptr;
  if (!sp)
    return;
  const std::string &anchor = key.second;
  const auto in = m_internalShaderRepl.find(key);
  const auto user = m_userShaderRepl.find(key);
  if (in == m_internalShaderRepl.end() && user == m_userShaderRepl.end()) {
    if (key.first == 0)
      sp->ClearVertexShaderReplacement(anchor, true);
    else
      sp->ClearFragmentShaderReplacement(anchor, true);
    return;
  }
  // vtkShaderProperty keeps ONE replacement per anchor, so compose: the
  // subclass's text first, then the caller's applied to the anchor that text
  // re-emits -- exactly what applying the two in turn to the shader would do.
  std::string text;
  if (in != m_internalShaderRepl.end()) {
    text = in->second;
    if (user != m_userShaderRepl.end()) {
      const std::size_t pos = text.find(anchor);
      if (pos != std::string::npos)
        text.replace(pos, anchor.size(), user->second);
    }
  } else {
    text = user->second;
  }
  if (key.first == 0)
    sp->AddVertexShaderReplacement(anchor, true, text, false);
  else
    sp->AddFragmentShaderReplacement(anchor, true, text, false);
}

void GeometryNode::disableCoordinateShiftScale() {
  runOnMainThread([this]() {
    // SetVBOShiftScaleMethod is virtual on vtkPolyDataMapper, so dispatch straight to
    // whichever concrete mapper the object factory handed us. The previous code cast to
    // vtkOpenGLPolyDataMapper — correct on desktop GL, but on GLES3/WebGL2 (wasm) the
    // factory returns vtkOpenGLLowMemoryPolyDataMapper, which is a vtkPolyDataMapper but
    // NOT a vtkOpenGLPolyDataMapper. There the cast silently failed, leaving shift-scale
    // ON, so world-space-vertexMC shaders (the GPU FFT ocean, the ground/bark detail)
    // received coordinates scaled to ~[-1,1] and rendered dead flat / wrong.
    if (m_mapper)
      m_mapper->SetVBOShiftScaleMethod(vtkOpenGLVertexBufferObject::DISABLE_SHIFT_SCALE);
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

// ── Custom shader inputs ──────────────────────────────────────────────────────
void GeometryNode::setShaderUniformf(const std::string &name, float v) {
  runOnMainThread([this, name, v]() {
    m_uniformsF[name] = v;
    ensureShaderTexObserver();
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}
void GeometryNode::setShaderUniformi(const std::string &name, int v) {
  runOnMainThread([this, name, v]() {
    m_uniformsI[name] = v;
    ensureShaderTexObserver();
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}
void GeometryNode::setShaderUniform3f(const std::string &name, float x, float y, float z) {
  runOnMainThread([this, name, x, y, z]() {
    m_uniforms3F[name] = {x, y, z};
    ensureShaderTexObserver();
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

void GeometryNode::onUpdateShader(vtkObject *, unsigned long, void *clientData, void *callData) {
  auto *self = static_cast<GeometryNode *>(clientData);
  auto *program = static_cast<vtkShaderProgram *>(callData);
  if (!self || !program)
    return;
  // Push registered uniforms + bind registered textures for THIS shader program.
  // Uniforms optimised out of a given program (e.g. the depth-only shadow shader)
  // are simply not found and skipped.
  for (auto &kv : self->m_uniformsF)
    if (program->IsUniformUsed(kv.first.c_str()))
      program->SetUniformf(kv.first.c_str(), kv.second);
  for (auto &kv : self->m_uniformsI)
    if (program->IsUniformUsed(kv.first.c_str()))
      program->SetUniformi(kv.first.c_str(), kv.second);
  for (auto &kv : self->m_uniforms3F)
    if (program->IsUniformUsed(kv.first.c_str()))
      program->SetUniform3f(kv.first.c_str(), kv.second.data());
  if (self->m_shaderTextures.empty())
    return;
  std::vector<std::pair<std::string, vtkTextureObject *>> textures;
  textures.reserve(self->m_shaderTextures.size());
  for (auto &kv : self->m_shaderTextures)
    textures.emplace_back(kv.first, kv.second.GetPointer());
  bindShaderTextures(program, textures);
}

void GeometryNode::bindShaderTextures(
    vtkShaderProgram *program,
    const std::vector<std::pair<std::string, vtkTextureObject *>> &textures) {
  if (!program)
    return;
  // Bind our custom textures onto units the mapper won't reuse. The wasm
  // low-memory poly-data mapper has no real VBOs: it emulates every vertex buffer
  // (positions, normals, cell/edge id maps) as sampler2D/isampler2D/usampler2D
  // texture units, and it activates those onto the LOWEST free units later, during
  // the draw — after this UpdateShaderEvent fires. If we let Activate() hand our
  // own textures those same low units now, one of the mapper's sampler2D buffers
  // ends up sharing a unit with e.g. a sampler3D volume: GL rejects the draw with
  // "two textures of different types use the same sampler location" and the result
  // is undefined. (The native mapper uses real VBOs, needs no vertex-data texture
  // units, and never hits this.) So reserve a block of low units first — pushing
  // our Activate() above the mapper's emulated buffers — then release the
  // reservation so the mapper still gets its low units back for the draw.
  vtkTextureUnitManager *tum = nullptr;
  for (auto &kv : textures)
    if (kv.second && kv.second->GetContext()) {
      tum = kv.second->GetContext()->GetTextureUnitManager();
      break;
    }
  constexpr int kMapperReserve = 8; // comfortably above the mapper's emulated buffers
  int reserved[kMapperReserve];
  int nReserved = 0;
  if (tum)
    for (int i = 0; i < kMapperReserve; ++i) {
      int u = tum->Allocate();
      if (u < 0)
        break;
      reserved[nReserved++] = u;
    }
  for (auto &kv : textures) {
    if (kv.second && kv.second->GetContext() && program->IsUniformUsed(kv.first.c_str())) {
      kv.second->Activate();
      program->SetUniformi(kv.first.c_str(), kv.second->GetTextureUnit());
    }
  }
  if (tum)
    for (int i = 0; i < nReserved; ++i)
      tum->Free(reserved[i]);
}

void GeometryNode::ensureShaderTexObserver() {
  if (m_shaderTexObserverInstalled || !m_mapper)
    return;
  m_shaderTexCb = vtkSmartPointer<vtkCallbackCommand>::New();
  m_shaderTexCb->SetClientData(this);
  m_shaderTexCb->SetCallback(&GeometryNode::onUpdateShader);
  m_mapper->AddObserver(vtkCommand::UpdateShaderEvent, m_shaderTexCb);
  m_shaderTexObserverInstalled = true;
}

void GeometryNode::setShaderTexture(const std::string &name, vtkTextureObject *tex) {
  runOnMainThread([this, name, tex]() {
    if (tex)
      m_shaderTextures[name] = tex;
    else
      m_shaderTextures.erase(name);
    ensureShaderTexObserver();
    if (SceneGraph *sg = getSceneGraph())
      sg->requestRender();
  });
}

// The boundary triangles of geom's tets (cvc::tet_boundary_surface, which keeps
// every point), or none without libigl or when it rejects the mesh. Cached for
// the stored geometry (until the next setGeometry); any other geometry's are
// computed into `scratch`.
const cvc::geometry::tris_t &GeometryNode::tetBoundaryTris(const cvc::geometry &geom,
                                                           cvc::geometry::tris_t &scratch) {
  scratch.clear();
  if (geom.num_tets() == 0 || !cvc::mesh_ops_available())
    return scratch;
  const bool cached = &geom == m_geometry.get();
  if (cached && m_tetSurfaceTried)
    return m_tetSurface;
  try {
    const cvc::geometry surface = cvc::tet_boundary_surface(geom);
    if (surface.num_points() == geom.num_points())
      scratch = surface.const_tris();
  } catch (const std::exception &e) {
    app().log(1, "GeometryNode[" + getName() + "]: no tet boundary surface (" + e.what() +
                     ") — TETS draws the tet edges, and there is no boundary curvature");
  } catch (...) {
    app().log(1, "GeometryNode[" + getName() +
                     "]: no tet boundary surface — TETS draws the tet edges, and there is no "
                     "boundary curvature");
  }
  if (!cached)
    return scratch;
  m_tetSurface = std::move(scratch);
  m_tetSurfaceTried = true;
  return m_tetSurface;
}

// TETS mode: append the boundary triangles of geom's tets to `cells`. False --
// draw the tet edges instead -- without libigl or when cvc::tet_boundary_surface
// rejects the mesh.
bool GeometryNode::tetSurfaceCells(const cvc::geometry &geom, vtkCellArray *cells) {
  cvc::geometry::tris_t scratch;
  const cvc::geometry::tris_t &tris = tetBoundaryTris(geom, scratch);
  if (tris.empty())
    return false;
  for (const auto &tri : tris) {
    cells->InsertNextCell(3);
    cells->InsertCellPoint(tri[0]);
    cells->InsertCellPoint(tri[1]);
    cells->InsertCellPoint(tri[2]);
  }
  return true;
}

void GeometryNode::updatePolyData(const cvc::geometry &geom) {
  // Create VTK points from geometry
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetNumberOfPoints(geom.num_points());

  for (size_t i = 0; i < geom.num_points(); ++i) {
    const auto &pt = geom.points()[i];
    points->SetPoint(i, pt[0], pt[1], pt[2]);
  }

  // Clear existing cells
  m_polyData->SetVerts(nullptr);
  m_polyData->SetLines(nullptr);
  m_polyData->SetPolys(nullptr);

  // Create cells based on render mode
  switch (m_renderMode) {
  case GeometryRenderMode::POINTS: {
    // Render as point cloud
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    for (size_t i = 0; i < geom.num_points(); ++i) {
      vertices->InsertNextCell(1);
      vertices->InsertCellPoint(i);
    }
    m_polyData->SetVerts(vertices);
    break;
  }

  case GeometryRenderMode::LINES: {
    // Render as wireframe using edge connectivity
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    // Add lines from line array if available
    for (size_t i = 0; i < geom.num_lines(); ++i) {
      const auto &line = geom.lines()[i];
      lines->InsertNextCell(2);
      lines->InsertCellPoint(line[0]);
      lines->InsertCellPoint(line[1]);
    }

    // Add triangle edges
    for (size_t i = 0; i < geom.num_tris(); ++i) {
      const auto &tri = geom.tris()[i];
      // Three edges per triangle
      lines->InsertNextCell(2);
      lines->InsertCellPoint(tri[0]);
      lines->InsertCellPoint(tri[1]);

      lines->InsertNextCell(2);
      lines->InsertCellPoint(tri[1]);
      lines->InsertCellPoint(tri[2]);

      lines->InsertNextCell(2);
      lines->InsertCellPoint(tri[2]);
      lines->InsertCellPoint(tri[0]);
    }

    // Add quad edges
    for (size_t i = 0; i < geom.num_quads(); ++i) {
      const auto &quad = geom.quads()[i];
      // Four edges per quad
      lines->InsertNextCell(2);
      lines->InsertCellPoint(quad[0]);
      lines->InsertCellPoint(quad[1]);

      lines->InsertNextCell(2);
      lines->InsertCellPoint(quad[1]);
      lines->InsertCellPoint(quad[2]);

      lines->InsertNextCell(2);
      lines->InsertCellPoint(quad[2]);
      lines->InsertCellPoint(quad[3]);

      lines->InsertNextCell(2);
      lines->InsertCellPoint(quad[3]);
      lines->InsertCellPoint(quad[0]);
    }

    m_polyData->SetLines(lines);
    break;
  }

  case GeometryRenderMode::TRIS: {
    // Render triangles as solid surface
    vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();

    for (size_t i = 0; i < geom.num_tris(); ++i) {
      const auto &tri = geom.tris()[i];
      triangles->InsertNextCell(3);
      triangles->InsertCellPoint(tri[0]);
      triangles->InsertCellPoint(tri[1]);
      triangles->InsertCellPoint(tri[2]);
    }

    m_polyData->SetPolys(triangles);
    break;
  }

  case GeometryRenderMode::QUADS: {
    // Render quads as solid surface
    vtkSmartPointer<vtkCellArray> quads = vtkSmartPointer<vtkCellArray>::New();

    for (size_t i = 0; i < geom.num_quads(); ++i) {
      const auto &quad = geom.quads()[i];
      quads->InsertNextCell(4);
      quads->InsertCellPoint(quad[0]);
      quads->InsertCellPoint(quad[1]);
      quads->InsertCellPoint(quad[2]);
      quads->InsertCellPoint(quad[3]);
    }

    m_polyData->SetPolys(quads);
    break;
  }

  case GeometryRenderMode::TETS: {
    // The tets' boundary as an outward-wound surface (cvc::tet_boundary_surface,
    // libigl). It keeps every point, so per-vertex colours, scalars and normals
    // still line up; interior points are simply unreferenced.
    vtkSmartPointer<vtkCellArray> boundary = vtkSmartPointer<vtkCellArray>::New();
    if (tetSurfaceCells(geom, boundary)) {
      m_polyData->SetPolys(boundary);
      m_actor->GetProperty()->SetRepresentationToSurface();
      break;
    }

    // Without libigl, or for tets it rejects: the wireframe edges
    m_actor->GetProperty()->SetRepresentationToWireframe();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    for (size_t i = 0; i < geom.num_tets(); ++i) {
      const auto &tet = geom.tets()[i];
      // 6 edges per tet: (0,1), (0,2), (0,3), (1,2), (1,3), (2,3)
      const int edges[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
      for (int e = 0; e < 6; ++e) {
        lines->InsertNextCell(2);
        lines->InsertCellPoint(tet[edges[e][0]]);
        lines->InsertCellPoint(tet[edges[e][1]]);
      }
    }

    m_polyData->SetLines(lines);
    break;
  }

  case GeometryRenderMode::HEXS: {
    // TODO: Implement hexahedral mesh rendering
    // For now, render as wireframe edges
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

    for (size_t i = 0; i < geom.num_hexs(); ++i) {
      const auto &hex = geom.hexs()[i];
      // 12 edges per hex
      const int edges[12][2] = {
          {0, 1}, {1, 2}, {2, 3}, {3, 0}, // Bottom face
          {4, 5}, {5, 6}, {6, 7}, {7, 4}, // Top face
          {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Vertical edges
      };
      for (int e = 0; e < 12; ++e) {
        lines->InsertNextCell(2);
        lines->InsertCellPoint(hex[edges[e][0]]);
        lines->InsertCellPoint(hex[edges[e][1]]);
      }
    }

    m_polyData->SetLines(lines);
    break;
  }
  }

  // Update polydata points
  m_polyData->SetPoints(points);

  // Add normals if available
  if (geom.normals().size() == geom.num_points()) {
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
    normals->SetNumberOfComponents(3);
    normals->SetNumberOfTuples(geom.num_points());
    normals->SetName("Normals");

    for (size_t i = 0; i < geom.num_points(); ++i) {
      const auto &n = geom.normals()[i];
      normals->SetTuple3(i, n[0], n[1], n[2]);
    }

    m_polyData->GetPointData()->SetNormals(normals);
  } else {
    m_polyData->GetPointData()->SetNormals(nullptr);
  }

  // Per-vertex colors (a scalar field or geom.colors()) unless single color
  // mode is on -- see applyVertexColors.
  applyVertexColors(geom, /*inPlace=*/false);

  // Texture coordinates (UVs) — the dedicated SetTCoords slot, orthogonal to the
  // color scalars above. A textured mesh (glTF/OBJ carrying cvc::geometry uvs)
  // samples the vtkTexture set via setTexture(). vtkFloatArray, 2 components (u,v).
  if (geom.uvs().size() == geom.num_points()) {
    vtkSmartPointer<vtkFloatArray> tcoords = vtkSmartPointer<vtkFloatArray>::New();
    tcoords->SetNumberOfComponents(2);
    tcoords->SetNumberOfTuples(geom.num_points());
    tcoords->SetName("TCoords");
    for (size_t i = 0; i < geom.num_points(); ++i) {
      const auto &uv = geom.uvs()[i];
      // When a texture is active the V axis is flipped here (not in the pixels):
      // cvc::image is top-left origin, VTK samples bottom-left, so flipping the
      // TCoords' V keeps the zero-copy pixel buffer aliased (no flip-copy).
      tcoords->SetTuple2(i, uv[0], m_textureFlipV ? (1.0 - uv[1]) : uv[1]);
    }
    m_polyData->GetPointData()->SetTCoords(tcoords);
  } else {
    m_polyData->GetPointData()->SetTCoords(nullptr);
  }

  ensureNormals();

  m_polyData->Modified();
}

void GeometryNode::ensureNormals() {
  // A mesh WITHOUT normals is not merely flat-looking: VTK's polydata mapper
  // takes an unlit shader path for it, so a directional light does nothing and
  // the shadow-map snippet spliced into the lit template fails to compile
  // ("'vertexVC' : undeclared identifier"). Supplying normals is what makes
  // lighting and shadows work at all, and callers should not have to know that.
  //
  if (!m_polyData)
    return;
  if (m_polyData->GetPointData()->GetNormals())
    return; // the geometry carried its own; do not second-guess them

  const vtkIdType numPts = m_polyData->GetNumberOfPoints();
  if (numPts == 0)
    return;

  // A mesh with no POLYS -- a LINES cluster (pine needles, paths) or bare
  // points -- has no surface to derive normals from, and running
  // vtkPolyDataNormals over one strips the lines out of the output entirely.
  // It still needs SOME normal array, though, for the reason above: the mapper
  // picks the shader path on the mere presence of normals, so a single line
  // primitive anywhere in the scene is enough to break every shader the moment
  // shadows are switched on. That is not hypothetical -- it is what the forest
  // example hit, where the trees' needles are lines.
  //
  // A constant normal is the honest choice: a line genuinely has no surface
  // orientation, so any direction is arbitrary. +Z just makes it shade evenly.
  if (m_polyData->GetNumberOfPolys() == 0) {
    cvc::gl::ensureLineNormals(m_polyData);
    return;
  }

  vtkSmartPointer<vtkPolyDataNormals> filter = vtkSmartPointer<vtkPolyDataNormals>::New();
  filter->SetInputData(m_polyData);
  filter->SplittingOff();          // keep the vertex count, so per-vertex colours still line up
  filter->ConsistencyOn();         // fix wind-order disagreements between neighbouring tris
  filter->ComputePointNormalsOn(); // smooth shading across the surface
  filter->ComputeCellNormalsOff();
  filter->AutoOrientNormalsOff(); // needs a closed surface; terrain and canopies are not
  filter->Update();

  if (vtkPolyData *out = filter->GetOutput()) {
    if (vtkDataArray *n = out->GetPointData()->GetNormals())
      m_polyData->GetPointData()->SetNormals(n);
  }
}

cvc::bounding_box GeometryNode::getBoundingBox() const {
  if (m_geometry) {
    try {
      return m_geometry->extents();
    } catch (...) {
      // extents() can throw for empty/invalid geometry
      return cvc::bounding_box(0, 0, 0, 0, 0, 0);
    }
  }
  // Return empty bounding box
  return cvc::bounding_box(0, 0, 0, 0, 0, 0);
}

// Note: syncToState and syncFromState removed - state_object handles state synchronization
// automatically
bool GeometryNode::isComputedMetadata(const std::string &key) {
  // These metadata keys are computed from geometry data and should be read-only
  static const std::set<std::string> computedKeys = {"num_vertices",
                                                     "num_triangles",
                                                     "num_quads",
                                                     "num_lines",
                                                     "bbox_min_x",
                                                     "bbox_min_y",
                                                     "bbox_min_z",
                                                     "bbox_max_x",
                                                     "bbox_max_y",
                                                     "bbox_max_z",
                                                     "extent_x",
                                                     "extent_y",
                                                     "extent_z",
                                                     "center_x",
                                                     "center_y",
                                                     "center_z",
                                                     "bounding_box",
                                                     "type",
                                                     "filename",
                                                     "combined_bbox_min_x",
                                                     "combined_bbox_min_y",
                                                     "combined_bbox_min_z",
                                                     "combined_bbox_max_x",
                                                     "combined_bbox_max_y",
                                                     "combined_bbox_max_z",
                                                     "combined_extent_x",
                                                     "combined_extent_y",
                                                     "combined_extent_z",
                                                     "combined_center_x",
                                                     "combined_center_y",
                                                     "combined_center_z"};

  return computedKeys.find(key) != computedKeys.end();
}

void GeometryNode::updateMetadata(const cvc::geometry &geom) {
  // Update all geometry statistics as metadata
  setMetadata("num_vertices", static_cast<int>(geom.num_points()));
  setMetadata("num_triangles", static_cast<int>(geom.num_tris()));
  setMetadata("num_quads", static_cast<int>(geom.num_quads()));

  // Only compute bounding box if geometry has points
  if (geom.num_points() > 0) {
    try {
      // Get bounding box extents
      auto bbox = geom.extents();

      setMetadata("bbox_min_x", bbox.minx);
      setMetadata("bbox_min_y", bbox.miny);
      setMetadata("bbox_min_z", bbox.minz);
      setMetadata("bbox_max_x", bbox.maxx);
      setMetadata("bbox_max_y", bbox.maxy);
      setMetadata("bbox_max_z", bbox.maxz);

      // Store combined bounding box string for computeGraphicsBounds()
      std::string bboxStr = std::to_string(bbox.minx) + "," + std::to_string(bbox.miny) + "," +
                            std::to_string(bbox.minz) + "," + std::to_string(bbox.maxx) + "," +
                            std::to_string(bbox.maxy) + "," + std::to_string(bbox.maxz);
      setMetadata("bounding_box", bboxStr);

      // Compute extents (dimensions)
      double extentX = bbox.maxx - bbox.minx;
      double extentY = bbox.maxy - bbox.miny;
      double extentZ = bbox.maxz - bbox.minz;

      setMetadata("extent_x", extentX);
      setMetadata("extent_y", extentY);
      setMetadata("extent_z", extentZ);

      // Compute center point
      setMetadata("center_x", (bbox.minx + bbox.maxx) / 2.0);
      setMetadata("center_y", (bbox.miny + bbox.maxy) / 2.0);
      setMetadata("center_z", (bbox.minz + bbox.maxz) / 2.0);
    } catch (...) {
      // Failed to compute bounding box for empty or invalid geometry
    }
  }

  // Add geometry type
  std::string geomType = "mesh";
  if (geom.num_tris() > 0 && geom.num_quads() == 0) {
    geomType = "triangle_mesh";
  } else if (geom.num_quads() > 0 && geom.num_tris() == 0) {
    geomType = "quad_mesh";
  } else if (geom.num_tris() > 0 && geom.num_quads() > 0) {
    geomType = "mixed_mesh";
  } else if (geom.num_points() == 0) {
    geomType = "empty";
  }
  setMetadata("type", geomType);
}

void GeometryNode::onDataChanged() {
  // Called when state data changes - reload geometry from state
  // Note: With state_object, we access state via getState() instead of m_stateNode
  if (getState().isData<cvc::geometry>()) {
    try {
      const cvc::geometry &geom = boost::any_cast<const cvc::geometry &>(getState().data());
      setGeometry(geom);
    } catch (...) {
      // Failed to load geometry from state
    }
  }
}

} // namespace gl
} // namespace cvc
