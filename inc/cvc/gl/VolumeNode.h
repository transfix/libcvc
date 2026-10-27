#ifndef VOLUMENODE_H
#define VOLUMENODE_H

#include <cvc/gl/GraphicsNode.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkSmartPointer.h>

class vtkVolume;
class vtkSmartVolumeMapper;
class vtkImageData;
class vtkColorTransferFunction;
class vtkPiecewiseFunction;
class vtkVolumeProperty;

namespace cvc {
class volume;
class state;
} // namespace cvc

namespace cvc {
namespace gl {
/**
 * @brief VolumeNode renders cvc::volume objects with full transform support
 *
 * Extends GraphicsNode to provide:
 * - Volume-specific rendering (ray casting, GPU volume rendering)
 * - Transfer function control (color and opacity)
 * - Bounding box computation from volume bounds
 * - State tree synchronization for volume data
 *
 * Inherits from GraphicsNode:
 * - Transforms (position, rotation, scale)
 * - Metadata storage
 * - Bounding box display
 * - Hierarchical structure
 */

class VolumeNode : public GraphicsNode {
public:
  VolumeNode(cvc::app &ctx, const std::string &statePath, const std::string &name = "volume");
  ~VolumeNode() override;

  // Generic setData for template compatibility
  void setData(const cvc::volume &vol) { setVolume(vol); }

  void setVolume(const cvc::volume &vol);

  // Lightweight in-place refresh of the voxel values: overwrite the scalars WITHOUT
  // re-importing (no realloc, no scalar-range rescan, no transfer-function reset, no
  // logging). `data` must match the current volume's voxel count and be laid out the
  // same as the float volume last set via setVolume(). This is the per-frame
  // animation fast path — orders of magnitude cheaper than setVolume(), which
  // reallocates the image, rescans the range, and resets the transfer function.
  void updateScalars(const std::vector<float> &data);
  bool hasVolume() const { return m_hasVolume; }
  const cvc::volume *getVolume() const { return m_volume.get(); }
  // The same volume by its shared owner (null before setVolume). Each setVolume makes a new one,
  // so a weak_ptr to it tells a replaced volume from the one it saw even when the new one lands at
  // the old one's address (a raw pointer cannot).
  std::shared_ptr<const cvc::volume> getSharedVolume() const { return m_volume; }

  // The transfer function is two-way: these setters apply the tables and mirror them into the
  // `transfer_function.color` ("s,r,g,b,...") / `transfer_function.opacity` ("s,a,...") keys, and
  // a write to those keys (an editor's tf_binding, a script, a replicated peer) is applied live.
  // A volume reload (setVolume) or a data_min/data_max change re-fits only the DEFAULT ramp; a
  // transfer function anyone authored survives it.
  void setTransferFunction(const std::vector<double> &colorTable,
                           const std::vector<double> &opacityTable);
  void setDefaultTransferFunction();

  // What VTK draws (its control points, sorted, a duplicate scalar collapsed), as the flat tables.
  std::vector<double> getTransferFunctionColorTable() const;
  std::vector<double> getTransferFunctionOpacityTable() const;

  // Volume rendering property getters and setters
  void setShading(bool enabled);
  bool getShading() const { return m_shading; }

  void setAmbient(double value);
  double getAmbient() const { return m_ambient; }

  void setDiffuse(double value);
  double getDiffuse() const { return m_diffuse; }

  void setSpecular(double value);
  double getSpecular() const { return m_specular; }

  void setSpecularPower(double value);
  double getSpecularPower() const { return m_specularPower; }

  void setScalarOpacityUnitDistance(double value);
  double getScalarOpacityUnitDistance() const { return m_scalarOpacityUnitDistance; }

  void setSampleDistance(double value);
  double getSampleDistance() const { return m_sampleDistance; }

  void setAutoAdjustSampleDistances(bool enabled);
  bool getAutoAdjustSampleDistances() const { return m_autoAdjustSampleDistances; }

  // Volumetric scattering / self-shadowing (GPU ray-cast; only active when
  // shading is on). `blend` in [0,2]: 0 = surfacic approximation only (no
  // volumetric shadows), 1 = smart blend of the two models, 2 = full volumetric
  // multi-scattering — this is what sculpts a soft field into lit tops and dim
  // undersides. `reach` in [0,1] trades shadow locality (0, cheap) for full
  // shadows (1, costlier). `g` in [-1,1] is the Henyey-Greenstein phase function
  // asymmetry: > 0 forward-scatters (as clouds do), 0 is isotropic.
  void setVolumetricScattering(double blend);
  double getVolumetricScattering() const { return m_volumetricScattering; }
  void setGlobalIlluminationReach(double reach);
  double getGlobalIlluminationReach() const { return m_giReach; }
  void setScatteringAnisotropy(double g);
  double getScatteringAnisotropy() const { return m_scatteringAnisotropy; }

  // Implement GraphicsNode abstract methods
  cvc::bounding_box getBoundingBox() const override;

  // Override to add logging
  void addToRenderer(vtkRenderer *renderer) override;

  // Check if a metadata key is computed (read-only)
  static bool isComputedMetadata(const std::string &key);

  // Clip planes: 8, vtkOpenGLGPUVolumeRayCastMapper's in_clippingPlanes[49] (a
  // count, then origin + normal per plane).
  int maxClipPlanes() const override { return 8; }

protected:
  vtkProp *getProp() override;
  void handleStateChanged(const std::string &childState) override;
  void applyTransformToVTK() override;                       // Apply transform to volume
  void applyClipPlanes(vtkPlaneCollection *planes) override; // Apply clip planes to volume mapper
  void updateImageData(const cvc::volume &vol);
  void updateTransferFunctions();
  void updateMetadata(const cvc::volume &vol);
  void onDataChanged();

private:
  // The tables -> the VTK functions only (no state write).
  void applyTablesToVTK(const std::vector<double> &colorTable,
                        const std::vector<double> &opacityTable);
  // Re-read transfer_function.color/.opacity and apply them; false (VTK untouched) when either
  // is malformed, so a half-written pair never reaches the mapper.
  bool applyTransferFunctionState();
  bool transferFunctionIsDefault() const;
  // Write one transfer_function.color/.opacity mirror key as this node's OWN write, and recognise
  // its echo: the handler call for that key, on this thread, during the write, while the key still
  // holds what we wrote. Anything else -- another thread's write above all -- is applied.
  void mirrorTfKey(const std::string &key, const std::string &value);
  bool isOwnTfEcho(const std::string &key) const;

  bool m_hasVolume;
  std::shared_ptr<cvc::volume> m_volume;

  vtkSmartPointer<vtkVolume> m_vtkVolume;
  vtkSmartPointer<vtkSmartVolumeMapper> m_mapper;
  vtkSmartPointer<vtkImageData> m_imageData;
  vtkSmartPointer<vtkColorTransferFunction> m_colorFunc;
  vtkSmartPointer<vtkPiecewiseFunction> m_opacityFunc;
  vtkSmartPointer<vtkVolumeProperty> m_volumeProperty;

  double m_dataMin;
  double m_dataMax;

  // The state strings setDefaultTransferFunction last wrote (what "still the default" means).
  std::string m_defaultColorState, m_defaultOpacityState;

  // Volume rendering properties
  bool m_shading;
  double m_ambient;
  double m_diffuse;
  double m_specular;
  double m_specularPower;
  double m_scalarOpacityUnitDistance;
  double m_sampleDistance;
  bool m_autoAdjustSampleDistances;
  double m_volumetricScattering;
  double m_giReach;
  double m_scatteringAnisotropy;

  cvc::state *m_stateNode;
  boost::signals2::connection m_dataConnection;
  boost::signals2::connection m_shadingConnection;
  boost::signals2::connection m_ambientConnection;
  boost::signals2::connection m_diffuseConnection;
  boost::signals2::connection m_specularConnection;
  boost::signals2::connection m_specularPowerConnection;
  boost::signals2::connection m_scalarOpacityUnitDistanceConnection;
  boost::signals2::connection m_sampleDistanceConnection;
  boost::signals2::connection m_autoAdjustSampleDistancesConnection;
};

} // namespace gl
} // namespace cvc

#endif // VOLUMENODE_H
