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
#include <vtkRenderer.h>
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
    , m_shading(true)
    , m_ambient(0.3)
    , m_diffuse(0.6)
    , m_specular(0.2)
    , m_specularPower(10.0)
    , m_scalarOpacityUnitDistance(1.0)
    , m_sampleDistance(0.5)
    , m_autoAdjustSampleDistances(true)
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
    m_volumeProperty->SetShade(m_shading ? 1 : 0);
    m_volumeProperty->SetInterpolationTypeToLinear();
    
    // Set lighting properties
    m_volumeProperty->SetAmbient(m_ambient);
    m_volumeProperty->SetDiffuse(m_diffuse);
    m_volumeProperty->SetSpecular(m_specular);
    m_volumeProperty->SetSpecularPower(m_specularPower);
    
    // Set scalar opacity unit distance
    m_volumeProperty->SetScalarOpacityUnitDistance(m_scalarOpacityUnitDistance);
    
    // Use composite blending for proper opacity
    m_mapper->SetBlendModeToComposite();
    
    // Configure the smart volume mapper
    m_mapper->SetAutoAdjustSampleDistances(m_autoAdjustSampleDistances ? 1 : 0);
    m_mapper->SetSampleDistance(m_sampleDistance);

    m_vtkVolume->SetProperty(m_volumeProperty);

    // Initialize with default transfer function
    setDefaultTransferFunction();
}

VolumeNode::~VolumeNode()
{
    m_dataConnection.disconnect();
    m_shadingConnection.disconnect();
    m_ambientConnection.disconnect();
    m_diffuseConnection.disconnect();
    m_specularConnection.disconnect();
    m_specularPowerConnection.disconnect();
    m_scalarOpacityUnitDistanceConnection.disconnect();
    m_sampleDistanceConnection.disconnect();
    m_autoAdjustSampleDistancesConnection.disconnect();
}

vtkProp* VolumeNode::getProp()
{
    return m_vtkVolume;
}

void VolumeNode::addToRenderer(vtkRenderer* renderer)
{
    cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Adding to renderer");
    GraphicsNode::addToRenderer(renderer);
    
    // Verify it was actually added and log detailed info
    if (renderer && renderer->GetVolumes()->IsItemPresent(m_vtkVolume)) {
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: CONFIRMED - Volume is in renderer");
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Total volumes in renderer: " + 
                   std::to_string(renderer->GetVolumes()->GetNumberOfItems()));
        
        // Log volume property details
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Volume visibility: " + 
                   std::to_string(m_vtkVolume->GetVisibility()));
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Volume pickable: " + 
                   std::to_string(m_vtkVolume->GetPickable()));
        
        // Log image data details
        int dims[3];
        m_imageData->GetDimensions(dims);
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Image data dimensions: [" + 
                   std::to_string(dims[0]) + ", " + std::to_string(dims[1]) + ", " + std::to_string(dims[2]) + "]");
        
        double* bounds = m_vtkVolume->GetBounds();
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Volume bounds: [" + 
                   std::to_string(bounds[0]) + ", " + std::to_string(bounds[1]) + ", " + 
                   std::to_string(bounds[2]) + ", " + std::to_string(bounds[3]) + ", " + 
                   std::to_string(bounds[4]) + ", " + std::to_string(bounds[5]) + "]");
        
        // Log transfer function ranges
        double colorRange[2], opacityRange[2];
        m_colorFunc->GetRange(colorRange);
        m_opacityFunc->GetRange(opacityRange);
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Color TF range: [" + 
                   std::to_string(colorRange[0]) + ", " + std::to_string(colorRange[1]) + "]");
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Opacity TF range: [" + 
                   std::to_string(opacityRange[0]) + ", " + std::to_string(opacityRange[1]) + "]");
        
        // Log opacity at a few sample points
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Opacity at dataMin(" + 
                   std::to_string(m_dataMin) + "): " + std::to_string(m_opacityFunc->GetValue(m_dataMin)));
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Opacity at dataMid: " + 
                   std::to_string(m_opacityFunc->GetValue((m_dataMin + m_dataMax) / 2.0)));
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Opacity at dataMax(" + 
                   std::to_string(m_dataMax) + "): " + std::to_string(m_opacityFunc->GetValue(m_dataMax)));
        
        // Log scalar range from image data
        double* scalarRange = m_imageData->GetScalarRange();
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: Image data scalar range: [" + 
                   std::to_string(scalarRange[0]) + ", " + std::to_string(scalarRange[1]) + "]");
    } else {
        cvcapp.log(0, "VolumeNode::addToRenderer[" + getName() + "]: WARNING - Volume NOT in renderer!");
    }
}

void VolumeNode::setVolume(const cvc::volume &vol)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    cvcapp.log(0, "\n=== VolumeNode::setVolume[" + getName() + "] ===");
    
    // Store the volume object
    m_volume = std::make_shared<cvc::volume>(vol);
    
    updateImageData(vol);
    m_dataMin = vol.min();
    m_dataMax = vol.max();
    
    cvcapp.log(0, "  Data range: [" + std::to_string(m_dataMin) + ", " + std::to_string(m_dataMax) + "]");
    cvcapp.log(0, "  Dimensions: [" + std::to_string(vol.XDim()) + ", " + std::to_string(vol.YDim()) + ", " + std::to_string(vol.ZDim()) + "]");
    cvcapp.log(0, "  Bounding box: [" + std::to_string(vol.XMin()) + "," + std::to_string(vol.XMax()) + "], [" +
               std::to_string(vol.YMin()) + "," + std::to_string(vol.YMax()) + "], [" +
               std::to_string(vol.ZMin()) + "," + std::to_string(vol.ZMax()) + "]");
    cvcapp.log(0, "  Spans: [" + std::to_string(vol.XSpan()) + ", " + std::to_string(vol.YSpan()) + ", " + std::to_string(vol.ZSpan()) + "]");
    
    // Calculate appropriate scalar opacity unit distance based on volume diagonal
    double dx = vol.XSpan();
    double dy = vol.YSpan();
    double dz = vol.ZSpan();
    double diagonal = std::sqrt(dx*dx + dy*dy + dz*dz);
    cvcapp.log(0, "  Diagonal: " + std::to_string(diagonal) + ", ScalarOpacityUnitDistance: " + std::to_string(diagonal / 100.0));
    m_volumeProperty->SetScalarOpacityUnitDistance(diagonal / 100.0);
    
    // Set transfer function using actual data range
    cvcapp.log(0, "  Setting default transfer function...");
    setDefaultTransferFunction();
    updateTransferFunctions();
    updateMetadata(vol);
    m_hasVolume = true;
    
    // Update bbox to match volume bounds
    updateBoundingBoxNode();
    cvcapp.log(0, "=================================\n");
}

void VolumeNode::updateImageData(const cvc::volume &vol)
{
    cvcapp.log(0, "\n  VolumeNode::updateImageData - Copying volume data to VTK...");
    
    // Get dimensions
    int dims[3] = {
        static_cast<int>(vol.XDim()),
        static_cast<int>(vol.YDim()),
        static_cast<int>(vol.ZDim())
    };
    
    cvcapp.log(0, "    CVC Volume bounds: X=[" + std::to_string(vol.XMin()) + ", " + std::to_string(vol.XMax()) + "]");
    cvcapp.log(0, "                       Y=[" + std::to_string(vol.YMin()) + ", " + std::to_string(vol.YMax()) + "]");
    cvcapp.log(0, "                       Z=[" + std::to_string(vol.ZMin()) + ", " + std::to_string(vol.ZMax()) + "]");
    cvcapp.log(0, "    CVC XSpan/YSpan/ZSpan: [" + std::to_string(vol.XSpan()) + ", " + 
               std::to_string(vol.YSpan()) + ", " + std::to_string(vol.ZSpan()) + "]");

    // CRITICAL FIX: Calculate spacing directly from bounding box, not from Span() methods
    // The Span() methods appear to return incorrect values for some volumes
    double spacing[3] = {
        (vol.XMax() - vol.XMin()) / vol.XDim(),
        (vol.YMax() - vol.YMin()) / vol.YDim(),
        (vol.ZMax() - vol.ZMin()) / vol.ZDim()
    };
    
    cvcapp.log(0, "    Calculated spacing: [" + std::to_string(spacing[0]) + ", " + 
               std::to_string(spacing[1]) + ", " + std::to_string(spacing[2]) + "]");

    // Get origin
    double origin[3] = {
        vol.XMin(),
        vol.YMin(),
        vol.ZMin()
    };
    
    cvcapp.log(0, "    Origin: [" + std::to_string(origin[0]) + ", " + 
               std::to_string(origin[1]) + ", " + std::to_string(origin[2]) + "]");

    // Determine VTK scalar type
    int scalarType;
    std::string scalarTypeName;
    switch (vol.voxelType()) {
        case cvc::UChar:  scalarType = VTK_UNSIGNED_CHAR; scalarTypeName = "UChar"; break;
        case cvc::UShort: scalarType = VTK_UNSIGNED_SHORT; scalarTypeName = "UShort"; break;
        case cvc::UInt:   scalarType = VTK_UNSIGNED_INT; scalarTypeName = "UInt"; break;
        case cvc::Float:  scalarType = VTK_FLOAT; scalarTypeName = "Float"; break;
        case cvc::Double: scalarType = VTK_DOUBLE; scalarTypeName = "Double"; break;
        default:          scalarType = VTK_FLOAT; scalarTypeName = "Float (default)"; break;
    }
    
    cvcapp.log(0, "    Voxel type: " + scalarTypeName);

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
    size_t totalBytes = numVoxels * bytesPerVoxel;
    
    cvcapp.log(0, "    Total voxels: " + std::to_string(numVoxels) + 
                  ", bytes per voxel: " + std::to_string(bytesPerVoxel) +
                  ", total bytes: " + std::to_string(totalBytes));
    cvcapp.log(0, "    CVC data pointer: " + std::string(cvcPtr ? "VALID" : "NULL"));
    cvcapp.log(0, "    VTK data pointer: " + std::string(vtkPtr ? "VALID" : "NULL"));
    
    if (cvcPtr && vtkPtr) {
        std::memcpy(vtkPtr, cvcPtr, totalBytes);
        cvcapp.log(0, "    \u2713 Data copied successfully");
    } else {
        cvcapp.log(0, "    \u2717 ERROR: Cannot copy data - null pointer!");
    }

    m_imageData->Modified();
}

void VolumeNode::setTransferFunction(const std::vector<double> &colorTable,
                                    const std::vector<double> &opacityTable)
{
    cvcapp.log(0, "\nVolumeNode::setTransferFunction[" + getName() + "]: " + 
               std::to_string(colorTable.size() / 4) + " color pts, " +
               std::to_string(opacityTable.size() / 2) + " opacity pts");
    
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
        
        if (i < 3) {  // Log first few points
            cvcapp.log(0, "  Opacity[" + std::to_string(i) + "]: scalar=" + std::to_string(scalar) + 
                       ", opacity=" + std::to_string(opacity));
        }
    }

    updateTransferFunctions();
}

void VolumeNode::setDefaultTransferFunction()
{
    m_colorFunc->RemoveAllPoints();
    m_opacityFunc->RemoveAllPoints();
    
    // Default grayscale color map using actual data range
    m_colorFunc->AddRGBPoint(m_dataMin, 0.0, 0.0, 0.0);
    m_colorFunc->AddRGBPoint(m_dataMax, 1.0, 1.0, 1.0);

    // Default opacity ramp using actual data range
    m_opacityFunc->AddPoint(m_dataMin, 0.0);
    m_opacityFunc->AddPoint(m_dataMax, 1.0);
}

void VolumeNode::setShading(bool enabled)
{
    m_shading = enabled;
    if (m_volumeProperty) {
        m_volumeProperty->SetShade(enabled);
    }
}

void VolumeNode::setAmbient(double value)
{
    m_ambient = value;
    if (m_volumeProperty) {
        m_volumeProperty->SetAmbient(value);
    }
}

void VolumeNode::setDiffuse(double value)
{
    m_diffuse = value;
    if (m_volumeProperty) {
        m_volumeProperty->SetDiffuse(value);
    }
}

void VolumeNode::setSpecular(double value)
{
    m_specular = value;
    if (m_volumeProperty) {
        m_volumeProperty->SetSpecular(value);
    }
}

void VolumeNode::setSpecularPower(double value)
{
    m_specularPower = value;
    if (m_volumeProperty) {
        m_volumeProperty->SetSpecularPower(value);
    }
}

void VolumeNode::setScalarOpacityUnitDistance(double value)
{
    m_scalarOpacityUnitDistance = value;
    if (m_volumeProperty) {
        m_volumeProperty->SetScalarOpacityUnitDistance(value);
    }
}

void VolumeNode::setSampleDistance(double value)
{
    m_sampleDistance = value;
    if (m_mapper) {
        m_mapper->SetSampleDistance(value);
    }
}

void VolumeNode::setAutoAdjustSampleDistances(bool enabled)
{
    m_autoAdjustSampleDistances = enabled;
    if (m_mapper) {
        m_mapper->SetAutoAdjustSampleDistances(enabled);
    }
}

void VolumeNode::updateTransferFunctions()
{
    static int callCount = 0;
    if (callCount++ == 0) {
        cvcapp.log(0, "\nVolumeNode::updateTransferFunctions[" + getName() + "]: First call");
        cvcapp.log(0, "  Data range: [" + std::to_string(m_dataMin) + ", " + std::to_string(m_dataMax) + "]");
    }
    
    m_colorFunc->Modified();
    m_opacityFunc->Modified();
    m_volumeProperty->Modified();
    m_vtkVolume->Modified();
    m_mapper->Modified();
    m_imageData->Modified();
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
    
    // Store label settings
    myState("show_label").value(m_showLabel ? "true" : "false");
    myState("label_text").value(m_labelText);
    myState("label_size").value(std::to_string(m_labelSize));
    std::ostringstream labelColorStr;
    labelColorStr << m_labelColor[0] << "," << m_labelColor[1] << "," << m_labelColor[2];
    myState("label_color").value(labelColorStr.str());
    
    // Store volume rendering properties
    cvc::state& renderingState = myState("rendering");
    renderingState.comment("Volume rendering properties");
    
    renderingState("shading").value(m_shading ? "true" : "false");
    renderingState("shading").comment("Enable/disable shading");
    
    renderingState("ambient").value(m_ambient);
    renderingState("ambient").comment("Ambient lighting coefficient (0.0-1.0)");
    
    renderingState("diffuse").value(m_diffuse);
    renderingState("diffuse").comment("Diffuse lighting coefficient (0.0-1.0)");
    
    renderingState("specular").value(m_specular);
    renderingState("specular").comment("Specular lighting coefficient (0.0-1.0)");
    
    renderingState("specular_power").value(m_specularPower);
    renderingState("specular_power").comment("Specular power (shininess)");
    
    renderingState("scalar_opacity_unit_distance").value(m_scalarOpacityUnitDistance);
    renderingState("scalar_opacity_unit_distance").comment("Scalar opacity unit distance");
    
    renderingState("sample_distance").value(m_sampleDistance);
    renderingState("sample_distance").comment("Ray casting sample distance");
    
    renderingState("auto_adjust_sample_distances").value(m_autoAdjustSampleDistances ? "true" : "false");
    renderingState("auto_adjust_sample_distances").comment("Auto-adjust sample distances");
    
    // Save common graphics attributes (transform, bbox, label, children, combined bbox if has children)
    saveCommonStateAttributes(myState);
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
        "voxel_type", "bounding_box", "filename",
        "combined_bbox_min_x", "combined_bbox_min_y", "combined_bbox_min_z",
        "combined_bbox_max_x", "combined_bbox_max_y", "combined_bbox_max_z",
        "combined_extent_x", "combined_extent_y", "combined_extent_z",
        "combined_center_x", "combined_center_y", "combined_center_z"
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
        
        // Load label settings
        try {
            std::string showLabelStr = myState("show_label").value();
            setShowLabel(showLabelStr == "true");
        } catch (...) {}
        
        try {
            std::string labelText = myState("label_text").value();
            setLabelText(labelText);
        } catch (...) {}
        
        try {
            int labelSize = std::stoi(myState("label_size").value());
            setLabelSize(labelSize);
        } catch (...) {}
        
        try {
            std::string colorStr = myState("label_color").value();
            std::istringstream iss(colorStr);
            double r, g, b;
            char comma;
            if (iss >> r >> comma >> g >> comma >> b) {
                setLabelColor(r, g, b);
            }
        } catch (...) {}
        
        // Load volume rendering properties
        try {
            cvc::state& renderingState = myState("rendering");
            
            // Disconnect existing connections
            m_shadingConnection.disconnect();
            m_ambientConnection.disconnect();
            m_diffuseConnection.disconnect();
            m_specularConnection.disconnect();
            m_specularPowerConnection.disconnect();
            m_scalarOpacityUnitDistanceConnection.disconnect();
            m_sampleDistanceConnection.disconnect();
            m_autoAdjustSampleDistancesConnection.disconnect();
            
            // Load shading
            try {
                std::string shadingStr = renderingState("shading").value();
                m_shading = (shadingStr == "true");
                m_volumeProperty->SetShade(m_shading ? 1 : 0);
                m_shadingConnection = renderingState("shading").valueChanged.connect([this, &renderingState]() {
                    m_shading = (renderingState("shading").value() == "true");
                    m_volumeProperty->SetShade(m_shading ? 1 : 0);
                    m_volumeProperty->Modified();
                    m_vtkVolume->Modified();
                });
            } catch (...) {}
            
            // Load ambient
            try {
                m_ambient = std::stod(renderingState("ambient").value());
                m_volumeProperty->SetAmbient(m_ambient);
                m_ambientConnection = renderingState("ambient").valueChanged.connect([this, &renderingState]() {
                    m_ambient = std::stod(renderingState("ambient").value());
                    m_volumeProperty->SetAmbient(m_ambient);
                    m_volumeProperty->Modified();
                    m_vtkVolume->Modified();
                });
            } catch (...) {}
            
            // Load diffuse
            try {
                m_diffuse = std::stod(renderingState("diffuse").value());
                m_volumeProperty->SetDiffuse(m_diffuse);
                m_diffuseConnection = renderingState("diffuse").valueChanged.connect([this, &renderingState]() {
                    m_diffuse = std::stod(renderingState("diffuse").value());
                    m_volumeProperty->SetDiffuse(m_diffuse);
                    m_volumeProperty->Modified();
                    m_vtkVolume->Modified();
                });
            } catch (...) {}
            
            // Load specular
            try {
                m_specular = std::stod(renderingState("specular").value());
                m_volumeProperty->SetSpecular(m_specular);
                m_specularConnection = renderingState("specular").valueChanged.connect([this, &renderingState]() {
                    m_specular = std::stod(renderingState("specular").value());
                    m_volumeProperty->SetSpecular(m_specular);
                    m_volumeProperty->Modified();
                    m_vtkVolume->Modified();
                });
            } catch (...) {}
            
            // Load specular power
            try {
                m_specularPower = std::stod(renderingState("specular_power").value());
                m_volumeProperty->SetSpecularPower(m_specularPower);
                m_specularPowerConnection = renderingState("specular_power").valueChanged.connect([this, &renderingState]() {
                    m_specularPower = std::stod(renderingState("specular_power").value());
                    m_volumeProperty->SetSpecularPower(m_specularPower);
                    m_volumeProperty->Modified();
                    m_vtkVolume->Modified();
                });
            } catch (...) {}
            
            // Load scalar opacity unit distance
            try {
                m_scalarOpacityUnitDistance = std::stod(renderingState("scalar_opacity_unit_distance").value());
                m_volumeProperty->SetScalarOpacityUnitDistance(m_scalarOpacityUnitDistance);
                m_scalarOpacityUnitDistanceConnection = renderingState("scalar_opacity_unit_distance").valueChanged.connect([this, &renderingState]() {
                    m_scalarOpacityUnitDistance = std::stod(renderingState("scalar_opacity_unit_distance").value());
                    m_volumeProperty->SetScalarOpacityUnitDistance(m_scalarOpacityUnitDistance);
                    m_volumeProperty->Modified();
                    m_vtkVolume->Modified();
                });
            } catch (...) {}
            
            // Load sample distance
            try {
                m_sampleDistance = std::stod(renderingState("sample_distance").value());
                m_mapper->SetSampleDistance(m_sampleDistance);
                m_sampleDistanceConnection = renderingState("sample_distance").valueChanged.connect([this, &renderingState]() {
                    m_sampleDistance = std::stod(renderingState("sample_distance").value());
                    m_mapper->SetSampleDistance(m_sampleDistance);
                    m_mapper->Modified();
                });
            } catch (...) {}
            
            // Load auto adjust sample distances
            try {
                std::string autoAdjustStr = renderingState("auto_adjust_sample_distances").value();
                m_autoAdjustSampleDistances = (autoAdjustStr == "true");
                m_mapper->SetAutoAdjustSampleDistances(m_autoAdjustSampleDistances ? 1 : 0);
                m_autoAdjustSampleDistancesConnection = renderingState("auto_adjust_sample_distances").valueChanged.connect([this, &renderingState]() {
                    m_autoAdjustSampleDistances = (renderingState("auto_adjust_sample_distances").value() == "true");
                    m_mapper->SetAutoAdjustSampleDistances(m_autoAdjustSampleDistances ? 1 : 0);
                    m_mapper->Modified();
                });
            } catch (...) {}
            
        } catch (...) {
            // Rendering state doesn't exist - use defaults
        }
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
