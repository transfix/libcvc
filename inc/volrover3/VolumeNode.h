#ifndef VOLUMENODE_H
#define VOLUMENODE_H

#include <volrover3/SceneNode.h>
#include <vtkSmartPointer.h>
#include <vector>

class vtkVolume;
class vtkSmartVolumeMapper;
class vtkImageData;
class vtkColorTransferFunction;
class vtkPiecewiseFunction;
class vtkVolumeProperty;

namespace cvc {
    class volume;
}

class VolumeNode : public SceneNode
{
public:
    VolumeNode();
    ~VolumeNode() override;

    void setVolume(const cvc::volume &vol);
    void setTransferFunction(const std::vector<double> &colorTable,
                            const std::vector<double> &opacityTable);

protected:
    vtkProp* getProp() override;

private:
    void updateImageData(const cvc::volume &vol);
    void updateTransferFunctions();

    vtkSmartPointer<vtkVolume> m_volume;
    vtkSmartPointer<vtkSmartVolumeMapper> m_mapper;
    vtkSmartPointer<vtkImageData> m_imageData;
    vtkSmartPointer<vtkColorTransferFunction> m_colorFunc;
    vtkSmartPointer<vtkPiecewiseFunction> m_opacityFunc;
    vtkSmartPointer<vtkVolumeProperty> m_volumeProperty;

    double m_dataMin;
    double m_dataMax;
};

#endif // VOLUMENODE_H
