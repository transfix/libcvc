#include <volrover3/VolumeNode.h>
#include <cvc/volume.h>
#include <vtkVolume.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkImageData.h>
#include <vtkColorTransferFunction.h>
#include <vtkPiecewiseFunction.h>
#include <vtkVolumeProperty.h>

VolumeNode::VolumeNode()
    : m_volume(vtkSmartPointer<vtkVolume>::New())
    , m_mapper(vtkSmartPointer<vtkSmartVolumeMapper>::New())
    , m_imageData(vtkSmartPointer<vtkImageData>::New())
    , m_colorFunc(vtkSmartPointer<vtkColorTransferFunction>::New())
    , m_opacityFunc(vtkSmartPointer<vtkPiecewiseFunction>::New())
    , m_volumeProperty(vtkSmartPointer<vtkVolumeProperty>::New())
    , m_dataMin(0.0)
    , m_dataMax(1.0)
{
    m_mapper->SetInputData(m_imageData);
    m_volume->SetMapper(m_mapper);

    // Set up volume property
    m_volumeProperty->SetColor(m_colorFunc);
    m_volumeProperty->SetScalarOpacity(m_opacityFunc);
    m_volumeProperty->ShadeOn();
    m_volumeProperty->SetInterpolationTypeToLinear();
    
    // Set scalar opacity unit distance for better opacity control
    m_volumeProperty->SetScalarOpacityUnitDistance(1.0);
    
    // Use composite blending for proper opacity
    m_mapper->SetBlendModeToComposite();

    m_volume->SetProperty(m_volumeProperty);

    // Initialize with default transfer function
    m_colorFunc->AddRGBPoint(0.0, 0.0, 0.0, 0.0);
    m_colorFunc->AddRGBPoint(1.0, 1.0, 1.0, 1.0);

    m_opacityFunc->AddPoint(0.0, 0.0);
    m_opacityFunc->AddPoint(1.0, 1.0);
}

VolumeNode::~VolumeNode()
{
}

vtkProp* VolumeNode::getProp()
{
    return m_volume;
}

void VolumeNode::setVolume(const cvc::volume &vol)
{
    updateImageData(vol);
    m_dataMin = vol.min();
    m_dataMax = vol.max();
    
    // Calculate appropriate scalar opacity unit distance based on volume diagonal
    double dx = vol.XSpan();
    double dy = vol.YSpan();
    double dz = vol.ZSpan();
    double diagonal = std::sqrt(dx*dx + dy*dy + dz*dz);
    m_volumeProperty->SetScalarOpacityUnitDistance(diagonal / 100.0);
    
    updateTransferFunctions();
}

void VolumeNode::updateImageData(const cvc::volume &vol)
{
    // Get dimensions
    int dims[3] = {
        static_cast<int>(vol.XDim()),
        static_cast<int>(vol.YDim()),
        static_cast<int>(vol.ZDim())
    };

    // Get spacing from bounding box
    double spacing[3] = {
        vol.XSpan(),
        vol.YSpan(),
        vol.ZSpan()
    };

    // Get origin
    double origin[3] = {
        vol.XMin(),
        vol.YMin(),
        vol.ZMin()
    };

    // Determine VTK scalar type
    int scalarType;
    switch (vol.voxelType()) {
        case cvc::UChar:  scalarType = VTK_UNSIGNED_CHAR; break;
        case cvc::UShort: scalarType = VTK_UNSIGNED_SHORT; break;
        case cvc::UInt:   scalarType = VTK_UNSIGNED_INT; break;
        case cvc::Float:  scalarType = VTK_FLOAT; break;
        case cvc::Double: scalarType = VTK_DOUBLE; break;
        default:          scalarType = VTK_FLOAT; break;
    }

    // Set up image data
    m_imageData->SetDimensions(dims);
    m_imageData->SetSpacing(spacing);
    m_imageData->SetOrigin(origin);
    m_imageData->AllocateScalars(scalarType, 1);

    // Copy voxel data
    void *vtkPtr = m_imageData->GetScalarPointer();
    const unsigned char *cvcPtr = *vol;

    size_t numVoxels = vol.XDim() * vol.YDim() * vol.ZDim();
    size_t bytesPerVoxel = vol.voxelSize();
    memcpy(vtkPtr, cvcPtr, numVoxels * bytesPerVoxel);

    m_imageData->Modified();
}

void VolumeNode::setTransferFunction(const std::vector<double> &colorTable,
                                    const std::vector<double> &opacityTable)
{
    // Clear existing functions
    m_colorFunc->RemoveAllPoints();
    m_opacityFunc->RemoveAllPoints();

    // Add color points (RGB triplets)
    for (size_t i = 0; i < colorTable.size() / 4; ++i) {
        double scalar = colorTable[i * 4 + 0];
        double r = colorTable[i * 4 + 1];
        double g = colorTable[i * 4 + 2];
        double b = colorTable[i * 4 + 3];
        m_colorFunc->AddRGBPoint(scalar, r, g, b);
    }

    // Add opacity points
    for (size_t i = 0; i < opacityTable.size() / 2; ++i) {
        double scalar = opacityTable[i * 2 + 0];
        double opacity = opacityTable[i * 2 + 1];
        m_opacityFunc->AddPoint(scalar, opacity);
    }

    updateTransferFunctions();
}

void VolumeNode::updateTransferFunctions()
{
    m_colorFunc->Modified();
    m_opacityFunc->Modified();
    m_volumeProperty->Modified();
    m_volume->Modified();
}
