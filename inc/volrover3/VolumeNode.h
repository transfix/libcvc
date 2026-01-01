#ifndef VOLUMENODE_H
#define VOLUMENODE_H

#include <volrover3/GraphicsNode.h>
#include <vtkSmartPointer.h>
#include <vector>
#include <memory>

class vtkVolume;
class vtkSmartVolumeMapper;
class vtkImageData;
class vtkColorTransferFunction;
class vtkPiecewiseFunction;
class vtkVolumeProperty;

namespace cvc {
    class volume;
    class state;
}

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
class VolumeNode : public GraphicsNode
{
public:
    VolumeNode(const std::string& name = "volume");
    ~VolumeNode() override;

    void setVolume(const cvc::volume &vol);
    bool hasVolume() const { return m_hasVolume; }
    const cvc::volume* getVolume() const { return m_volume.get(); }
    
    void setTransferFunction(const std::vector<double> &colorTable,
                            const std::vector<double> &opacityTable);
    void setDefaultTransferFunction();
    
    // Implement GraphicsNode abstract methods
    cvc::bounding_box getBoundingBox() const override;
    void syncToState(cvc::state& parentState) override;
    void syncFromState(cvc::state& parentState) override;
    
    // Override to add logging
    void addToRenderer(vtkRenderer* renderer) override;
    
    // Check if a metadata key is computed (read-only)
    static bool isComputedMetadata(const std::string& key);

protected:
    vtkProp* getProp() override;
    void updateImageData(const cvc::volume &vol);
    void updateTransferFunctions();
    void updateMetadata(const cvc::volume &vol);
    void onDataChanged();

private:
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
    
    // Volume rendering properties
    bool m_shading;
    double m_ambient;
    double m_diffuse;
    double m_specular;
    double m_specularPower;
    double m_scalarOpacityUnitDistance;
    double m_sampleDistance;
    bool m_autoAdjustSampleDistances;
    
    cvc::state* m_stateNode;
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

#endif // VOLUMENODE_H
