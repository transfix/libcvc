#ifndef GEOMETRYNODE_H
#define GEOMETRYNODE_H

#include <boost/shared_array.hpp>
#include <cvc/gl/GraphicsNode.h>
#include <memory>
#include <vtkSmartPointer.h>

class vtkActor;
class vtkPolyDataMapper;
class vtkPolyData;
class vtkTexture;
class vtkImageData;

namespace cvc {
class geometry;
class image;
class state;
} // namespace cvc

/**
 * @brief Geometry rendering modes
 */
enum class GeometryRenderMode {
  POINTS, // Render as point cloud
  LINES,  // Render as wireframe
  TRIS,   // Render triangles as solid surface
  QUADS,  // Render quads as solid surface
  TETS,   // Render tetrahedral mesh (placeholder)
  HEXS    // Render hexahedral mesh (placeholder)
};

/**
 * @brief GeometryNode renders cvc::geometry objects with full transform support
 *
 * Extends GraphicsNode to provide:
 * - Geometry-specific rendering (triangles, quads)
 * - Bounding box computation from geometry extents
 * - State tree synchronization for geometry data
 *
 * Inherits from GraphicsNode:
 * - Transforms (position, rotation, scale)
 * - Metadata storage
 * - Bounding box display
 * - Hierarchical structure
 */
class GeometryNode : public GraphicsNode {
public:
  GeometryNode(cvc::app &ctx, const std::string &statePath, const std::string &name = "geometry");
  ~GeometryNode() override;

  // Generic setData for template compatibility
  void setData(const cvc::geometry &geom) { setGeometry(geom); }

  void setGeometry(const cvc::geometry &geom);
  bool hasGeometry() const { return m_hasGeometry; }
  const cvc::geometry *getGeometry() const { return m_geometry.get(); }

  // Apply a texture (a cvc::image) to this mesh — sampled through the geometry's
  // UVs (SetTCoords). Meaningful only when the geometry carries uvs.
  //
  // zeroCopy (default): when `img` is already RGBA8, the vtkTexture ALIASES the
  // image's pixel buffer (vtkUnsignedCharArray::SetArray, no memcpy) — the
  // GeometryNode holds a ref to the buffer for the texture's lifetime, so a later
  // in-place pixel edit (e.g. via pycvc image.numpy()) followed by
  // texture_modified() shows live with no re-upload copy. The top-left-origin vs
  // VTK-bottom-left mismatch is resolved by flipping the TCoords' V (no pixel
  // copy). When `img` is not RGBA8, or zeroCopy is false, it falls back to the
  // convert-flip-and-copy path. clearTexture() removes the texture and drops the
  // aliased buffer.
  void setTexture(const cvc::image &img, bool zeroCopy = true);
  void clearTexture();
  // Signal that the texture's pixels were edited in place (through an aliased
  // zero-copy buffer): marks the vtkTexture + its input image data Modified() so
  // the next render re-samples the new bytes WITHOUT any re-copy.
  void texture_modified();

  // Render mode control
  void setRenderMode(GeometryRenderMode mode);
  GeometryRenderMode getRenderMode() const { return m_renderMode; }

  // Single color mode control
  void setUseSingleColor(bool useSingleColor);
  bool getUseSingleColor() const { return m_useSingleColor; }

  // Material property setters (sync with state tree)
  void setColor(double r, double g, double b);
  void setSpecular(double value);
  void setSpecularPower(double value);
  void setAmbient(double value);
  void setDiffuse(double value);
  void setOpacity(double value);
  void setPointSize(double size);
  void setLineWidth(double width);

  // Helper to convert render mode to/from string
  static std::string renderModeToString(GeometryRenderMode mode);
  static GeometryRenderMode stringToRenderMode(const std::string &str);

  // Implement GraphicsNode abstract methods
  cvc::bounding_box getBoundingBox() const override;

  // Check if a metadata key is computed (read-only)
  static bool isComputedMetadata(const std::string &key);

protected:
  vtkProp *getProp() override;
  void handleStateChanged(const std::string &childState) override;
  void applyTransformToVTK() override;                       // Apply transform to actor
  void applyClipPlanes(vtkPlaneCollection *planes) override; // Apply clip planes to mapper
  void updatePolyData(const cvc::geometry &geom);
  void updateRenderModeVTK(); // Helper to update VTK properties from render mode
  void updateMetadata(const cvc::geometry &geom);
  void onDataChanged();

private:
  bool m_hasGeometry;
  std::shared_ptr<cvc::geometry> m_geometry;
  GeometryRenderMode m_renderMode;
  bool m_useSingleColor; // When true, use single color; when false, use per-vertex colors

  vtkSmartPointer<vtkActor> m_actor;
  vtkSmartPointer<vtkPolyDataMapper> m_mapper;
  vtkSmartPointer<vtkPolyData> m_polyData;
  vtkSmartPointer<vtkTexture> m_texture;
  vtkSmartPointer<vtkImageData> m_textureImageData; // the texture's input (for texture_modified())
  boost::shared_array<unsigned char> m_textureStorage; // keeps the aliased zero-copy buffer alive
  bool m_textureFlipV; // true when a texture is active: UVs' V is flipped (top-left image -> VTK)

  boost::signals2::connection m_dataConnection;
};

#endif // GEOMETRYNODE_H
