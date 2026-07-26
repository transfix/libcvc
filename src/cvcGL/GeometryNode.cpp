#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/context.h>
#include <cvc/image/image.h>
#include <set>
#include <sstream>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkLine.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTexture.h>
#include <vtkTransform.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertex.h>

GeometryNode::GeometryNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GraphicsNode(ctx, statePath, name), m_hasGeometry(false),
      m_renderMode(GeometryRenderMode::TRIS), m_useSingleColor(false),
      m_actor(vtkSmartPointer<vtkActor>::New()),
      m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New()),
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
  }
}

GeometryNode::~GeometryNode() { m_dataConnection.disconnect(); }

void GeometryNode::applyTransformToVTK() {
  // Use generic helper to apply world transform
  applyWorldTransformToProps({m_actor});
}

void GeometryNode::applyClipPlanes(vtkPlaneCollection *planes) {
  if (m_mapper) {
    if (planes && planes->GetNumberOfItems() > 0) {
      m_mapper->SetClippingPlanes(planes);
    } else {
      m_mapper->RemoveAllClippingPlanes();
    }
  }
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

  // Request a render update (if we have a renderer with a render window)
  if (m_renderer && m_renderer->GetRenderWindow()) {
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

void GeometryNode::setTexture(const cvc::image &img, bool zeroCopy) {
  if (!m_actor)
    return;
  if (img.empty()) {
    clearTexture();
    return;
  }

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
        (img.format() == cvc::image::pixel_format::RGBA && img.type() == cvc::image::data_type::u8)
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
  // The texture supplies the surface color; per-vertex color scalars would tint
  // it. Set this LAST — after updatePolyData, which may re-enable scalar
  // visibility for a colored mesh — so the texture stands on its own.
  m_mapper->ScalarVisibilityOff();
  m_actor->Modified();
}

void GeometryNode::clearTexture() {
  m_texture = nullptr;
  m_textureImageData = nullptr;
  m_textureStorage.reset();
  if (m_actor) {
    m_actor->SetTexture(nullptr);
    m_actor->Modified();
  }
  // Restore un-flipped TCoords now that no top-left texture is active.
  if (m_textureFlipV) {
    m_textureFlipV = false;
    if (m_hasGeometry && m_geometry)
      updatePolyData(*m_geometry);
  }
}

void GeometryNode::texture_modified() {
  // The pixels were edited in place through the aliased buffer; bump the MTime of
  // the texture and its input image data so VTK re-samples on the next render —
  // no data re-copy.
  if (m_textureImageData) {
    if (vtkDataArray *scalars = m_textureImageData->GetPointData()->GetScalars())
      scalars->Modified();
    m_textureImageData->Modified();
  }
  if (m_texture)
    m_texture->Modified();
  if (m_actor)
    m_actor->Modified();
}

void GeometryNode::setGeometry(const cvc::geometry &geom) {
  cvc::thread_info ti(cvc::gl::context(), BOOST_CURRENT_FUNCTION);

  // CRITICAL: Entire method must run on main thread to avoid Qt threading errors
  // Even creating std::shared_ptr or setting member variables can trigger VTK
  // smart pointer operations that touch Qt objects
  runOnMainThread([this, geom]() {
    // Store the geometry object
    m_geometry = std::make_shared<cvc::geometry>(geom);
    m_hasGeometry = true; // Set this BEFORE setRenderMode so it can update

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
    updateBoundingBoxNode();

    updateMetadata(geom);

    // Notify parent to resync bounds if it's a NullGraphicNode with auto-sync enabled
    if (m_parent) {
      auto nullParent = dynamic_cast<NullGraphicNode *>(m_parent);
      if (nullParent) {
        nullParent->syncBoundsToChildren();
      }
    }
  });
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
    // TODO: Implement tetrahedral mesh rendering
    // For now, render as wireframe edges
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

  // Add per-vertex colors if available AND single color mode is disabled.
  // Store them as an UNSIGNED CHAR (0..255) array: VTK treats a 3-component
  // uchar scalar array as LITERAL colors — never routed through the mapper's
  // lookup table — so the direct-color behavior is intrinsic to the data type
  // and can't be re-broken by a stray SetColorMode* elsewhere (or by a fresh
  // mapper). A vtkFloatArray, by contrast, renders through the LUT unless
  // SetColorModeToDirectScalars() is ALSO set — the omission that turned red
  // meshes blue. This uchar path is also texture-ready: when UVs land later they
  // go in the dedicated SetTCoords slot and never collide with these colors.
  // geom.colors() are RGB doubles in [0,1] (geometry.h color_t).
  if (!m_useSingleColor && geom.colors().size() == geom.num_points()) {
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetNumberOfTuples(geom.num_points());
    colors->SetName("Colors");

    for (size_t i = 0; i < geom.num_points(); ++i) {
      const auto &c = geom.colors()[i];
      unsigned char rgb[3] = {
          static_cast<unsigned char>(std::lround(std::clamp(c[0], 0.0, 1.0) * 255.0)),
          static_cast<unsigned char>(std::lround(std::clamp(c[1], 0.0, 1.0) * 255.0)),
          static_cast<unsigned char>(std::lround(std::clamp(c[2], 0.0, 1.0) * 255.0))};
      colors->SetTypedTuple(i, rgb);
    }

    m_polyData->GetPointData()->SetScalars(colors);
    // uchar scalars are used directly as colors; select point-data + enable.
    m_mapper->SetScalarModeToUsePointData();
    m_mapper->ScalarVisibilityOn();
  } else {
    // Use single color from actor property - clear per-vertex colors
    m_polyData->GetPointData()->SetScalars(nullptr);
    m_mapper->ScalarVisibilityOff();
  }

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

  m_polyData->Modified();
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
