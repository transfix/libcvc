#include <volrover3/VolumeNode.h>
#include <cvc/volume.h>
#include <cvc/state.h>
#include <cvc/app.h>
#include <vtkVolume.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkImageData.h>
#include <vtkColorTransferFunction.h>
#include <vtkPiecewiseFunction.h>
#include <vtkVolumeProperty.h>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>

VolumeNode::VolumeNode(const std::string& name)
    : GraphicsNode(name)
    , m_hasVolume(false)
    , m_vtkVolume(vtkSmartPointer<vtkVolume>::New())
    , m_mapper(vtkSmartPointer<vtkSmartVolumeMapper>::New())
    , m_imageData(vtkSmartPointer<vtkImageData>::New())
    , m_colorFunc(vtkSmartPointer<vtkColorTransferFunction>::New())
    , m_opacityFunc(vtkSmartPointer<vtkPiecewiseFunction>::New())
    , m_volumeProperty(vtkSmartPointer<vtkVolumeProperty>::New())
    , m_dataMin(0.0)
    , m_dataMax(1.0)
    , m_stateNode(nullptr)
{
    // Initialize with empty 1x1x1 volume to avoid VTK errors before data is loaded
    m_imageData->SetDimensions(1, 1, 1);
    m_imageData->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
    unsigned char *ptr = static_cast<unsigned char*>(m_imageData->GetScalarPointer());
    ptr[0] = 0;
    
    m_mapper->SetInputData(m_imageData);
    m_vtkVolume->SetMapper(m_mapper);

    // Set up volume property
    m_volumeProperty->SetColor(m_colorFunc);
    m_volumeProperty->SetScalarOpacity(m_opacityFunc);
    m_volumeProperty->ShadeOn();
    m_volumeProperty->SetInterpolationTypeToLinear();
    
    // Set scalar opacity unit distance for better opacity control
    m_volumeProperty->SetScalarOpacityUnitDistance(1.0);
    
    // Use composite blending for proper opacity
    m_mapper->SetBlendModeToComposite();

    m_vtkVolume->SetProperty(m_volumeProperty);

    // Initialize with default transfer function
    setDefaultTransferFunction();
}

VolumeNode::~VolumeNode()
{
    m_dataConnection.disconnect();
}

vtkProp* VolumeNode::getProp()
{
    return m_vtkVolume;
}

void VolumeNode::setVolume(const cvc::volume &vol)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Store the volume object
    m_volume = std::make_shared<cvc::volume>(vol);
    
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
    updateMetadata(vol);
    m_hasVolume = true;
    
    // Update bbox to match volume bounds
    updateBoundingBoxNode();
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
        vol.XSpan() / vol.XDim(),
        vol.YSpan() / vol.YDim(),
        vol.ZSpan() / vol.ZDim()
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
    std::memcpy(vtkPtr, cvcPtr, numVoxels * bytesPerVoxel);

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

void VolumeNode::setDefaultTransferFunction()
{
    m_colorFunc->RemoveAllPoints();
    m_opacityFunc->RemoveAllPoints();
    
    // Default grayscale color map
    m_colorFunc->AddRGBPoint(0.0, 0.0, 0.0, 0.0);
    m_colorFunc->AddRGBPoint(1.0, 1.0, 1.0, 1.0);

    // Default opacity ramp
    m_opacityFunc->AddPoint(0.0, 0.0);
    m_opacityFunc->AddPoint(1.0, 1.0);
}

void VolumeNode::updateTransferFunctions()
{
    m_colorFunc->Modified();
    m_opacityFunc->Modified();
    m_volumeProperty->Modified();
    m_vtkVolume->Modified();
}

cvc::bounding_box VolumeNode::getBoundingBox() const
{
    if (m_volume) {
        try {
            return cvc::bounding_box(
                m_volume->XMin(), m_volume->YMin(), m_volume->ZMin(),
                m_volume->XMax(), m_volume->YMax(), m_volume->ZMax()
            );
        } catch (...) {
            // Bounding box calculations can throw for empty/invalid volumes
            return cvc::bounding_box(0, 0, 0, 0, 0, 0);
        }
    }
    // Return empty bounding box
    return cvc::bounding_box(0, 0, 0, 0, 0, 0);
}

void VolumeNode::syncToState(cvc::state& parentState)
{
    cvc::state& myState = parentState(m_name);
    myState.comment("Graphics object with volume data and transform");
    
    // Store volume data
    if (m_volume) {
        myState.data(*m_volume);
    }
    
    // Store transform as comma-separated string (row-major 4x4 matrix)
    std::string transformStr;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!transformStr.empty()) transformStr += ",";
            transformStr += std::to_string(m_transform->GetElement(i, j));
        }
    }
    cvc::state& transformState = myState("transform");
    transformState.value(transformStr);
    transformState.comment("4x4 transformation matrix in row-major order");
    
    // Store metadata with appropriate readOnly flags and comments
    cvc::state& metadataState = myState("metadata");
    metadataState.comment("Computed volume statistics and properties");
    
    for (const auto& [key, value] : m_metadata) {
        try {
            cvc::state& metaEntry = metadataState(key);
            
            if (value.type() == typeid(std::string)) {
                metaEntry.value(std::any_cast<std::string>(value));
            } else if (value.type() == typeid(double)) {
                metaEntry.value(std::any_cast<double>(value));
            } else if (value.type() == typeid(int)) {
                metaEntry.value(std::any_cast<int>(value));
            } else if (value.type() == typeid(bool)) {
                bool boolVal = std::any_cast<bool>(value);
                metaEntry.value(boolVal ? "true" : "false");
            }
            
            // Set readOnly flag for computed metadata
            metaEntry.readOnly(isComputedMetadata(key));
            
            // Set descriptive comments for known metadata keys
            if (key == "dim_x") metaEntry.comment("Volume dimension in X (number of voxels)");
            else if (key == "dim_y") metaEntry.comment("Volume dimension in Y (number of voxels)");
            else if (key == "dim_z") metaEntry.comment("Volume dimension in Z (number of voxels)");
            else if (key == "bbox_min_x") metaEntry.comment("Minimum X coordinate of bounding box");
            else if (key == "bbox_min_y") metaEntry.comment("Minimum Y coordinate of bounding box");
            else if (key == "bbox_min_z") metaEntry.comment("Minimum Z coordinate of bounding box");
            else if (key == "bbox_max_x") metaEntry.comment("Maximum X coordinate of bounding box");
            else if (key == "bbox_max_y") metaEntry.comment("Maximum Y coordinate of bounding box");
            else if (key == "bbox_max_z") metaEntry.comment("Maximum Z coordinate of bounding box");
            else if (key == "spacing_x") metaEntry.comment("Voxel spacing in X dimension");
            else if (key == "spacing_y") metaEntry.comment("Voxel spacing in Y dimension");
            else if (key == "spacing_z") metaEntry.comment("Voxel spacing in Z dimension");
            else if (key == "data_range_min") metaEntry.comment("Minimum voxel value in volume");
            else if (key == "data_range_max") metaEntry.comment("Maximum voxel value in volume");
            else if (key == "voxel_type") metaEntry.comment("Data type of voxels");
            else if (key == "bounding_box") metaEntry.comment("Complete bounding box as comma-separated values");
            else if (key == "filename") metaEntry.comment("Source filename for the volume");
            
        } catch (...) {
            // Skip metadata that can't be serialized
        }
    }
    
    // Store bbox flag
    myState("show_bbox").value(m_showBBox ? "true" : "false");
    
    // Recursively sync children under a "children" container
    if (!m_graphicsChildren.empty()) {
        cvc::state& childrenState = myState("children");
        childrenState.comment("Child graphics objects");
        for (const auto& child : m_graphicsChildren) {
            child->syncToState(childrenState);
        }
    }
}

bool VolumeNode::isComputedMetadata(const std::string& key)
{
    // These metadata keys are computed from volume data and should be read-only
    static const std::set<std::string> computedKeys = {
        "dim_x", "dim_y", "dim_z",
        "bbox_min_x", "bbox_min_y", "bbox_min_z",
        "bbox_max_x", "bbox_max_y", "bbox_max_z",
        "spacing_x", "spacing_y", "spacing_z",
        "data_range_min", "data_range_max",
        "voxel_type", "bounding_box", "filename"
    };
    
    return computedKeys.find(key) != computedKeys.end();
}

void VolumeNode::syncFromState(cvc::state& parentState)
{
    try {
        cvc::state& myState = parentState(m_name);
        
        m_stateNode = &myState;
        m_dataConnection.disconnect(); // Disconnect any previous connection
        m_dataConnection = myState.dataChanged.connect([this]() {
            onDataChanged();
        });
        
        // Load volume data
        if (myState.isData<cvc::volume>()) {
            try {
                const cvc::volume& vol = boost::any_cast<const cvc::volume&>(myState.data());
                setVolume(vol);
            } catch (...) {}
        }
        
        // Load transform
        try {
            std::string transformStr = myState("transform").value();
            std::vector<double> values;
            std::stringstream ss(transformStr);
            std::string token;
            while (std::getline(ss, token, ',')) {
                values.push_back(std::stod(token));
            }
            
            if (values.size() == 16) {
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        m_transform->SetElement(i, j, values[i * 4 + j]);
                    }
                }
                updateTransform();
            }
        } catch (...) {}
        
        // Load metadata
        try {
            cvc::state& metadataState = myState("metadata");
            std::vector<std::string> metadataKeys = metadataState.children();
            
            // Filter to direct children only
            std::string metadataPath = metadataState.fullName();
            int expectedDepth = std::count(metadataPath.begin(), metadataPath.end(), '.') + 1;
            
            for (const auto& descendant : metadataKeys) {
                int depth = std::count(descendant.begin(), descendant.end(), '.');
                if (depth == expectedDepth) {
                    // Extract just the key name
                    size_t lastDot = descendant.find_last_of('.');
                    std::string key = (lastDot != std::string::npos) ? descendant.substr(lastDot + 1) : descendant;
                    
                    try {
                        std::string value = metadataState(key).value();
                        
                        // Store all metadata as strings when loading from state
                        // The state tree stores everything as strings anyway
                        if (value == "true") {
                            setMetadata(key, true);
                        } else if (value == "false") {
                            setMetadata(key, false);
                        } else {
                            setMetadata(key, value);
                        }
                    } catch (...) {}
                }
            }
        } catch (...) {}
        
        // Load bbox flag
        try {
            std::string showBBoxStr = myState("show_bbox").value();
            setShowBBox(showBBoxStr == "true");
        } catch (...) {}
    } catch (...) {
        // State doesn't exist or can't be loaded
    }
}

void VolumeNode::updateMetadata(const cvc::volume &vol)
{
    // Store volume dimensions
    setMetadata("dim_x", static_cast<int>(vol.XDim()));
    setMetadata("dim_y", static_cast<int>(vol.YDim()));
    setMetadata("dim_z", static_cast<int>(vol.ZDim()));
    
    // Store bounding box
    setMetadata("bbox_min_x", vol.XMin());
    setMetadata("bbox_min_y", vol.YMin());
    setMetadata("bbox_min_z", vol.ZMin());
    setMetadata("bbox_max_x", vol.XMax());
    setMetadata("bbox_max_y", vol.YMax());
    setMetadata("bbox_max_z", vol.ZMax());
    
    // Store combined bounding box string for computeGraphicsBounds()
    std::string bboxStr = std::to_string(vol.XMin()) + "," +
                          std::to_string(vol.YMin()) + "," +
                          std::to_string(vol.ZMin()) + "," +
                          std::to_string(vol.XMax()) + "," +
                          std::to_string(vol.YMax()) + "," +
                          std::to_string(vol.ZMax());
    setMetadata("bounding_box", bboxStr);
    
    // Store spacing
    setMetadata("spacing_x", vol.XSpan() / vol.XDim());
    setMetadata("spacing_y", vol.YSpan() / vol.YDim());
    setMetadata("spacing_z", vol.ZSpan() / vol.ZDim());
    
    // Store data range
    setMetadata("data_min", vol.min());
    setMetadata("data_max", vol.max());
    
    // Store volume type
    std::string typeStr;
    switch (vol.voxelType()) {
        case cvc::UChar: typeStr = "unsigned_char"; break;
        case cvc::UShort: typeStr = "unsigned_short"; break;
        case cvc::UInt: typeStr = "unsigned_int"; break;
        case cvc::Float: typeStr = "float"; break;
        case cvc::Double: typeStr = "double"; break;
        default: typeStr = "unknown"; break;
    }
    setMetadata("voxel_type", typeStr);
}

void VolumeNode::onDataChanged()
{
    // Called when state data changes - reload volume from state
    if (!m_stateNode) return;
    
    if (m_stateNode->isData<cvc::volume>()) {
        try {
            const cvc::volume& vol = boost::any_cast<const cvc::volume&>(m_stateNode->data());
            setVolume(vol);
        } catch (...) {
            // Failed to load volume from state
        }
    }
}
