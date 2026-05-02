#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/volume.h>
#include <iomanip>
#include <set>
#include <sstream>
#include <volrover3/NullGraphicNode.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/volrover3_app.h>
#include <vtkColorTransferFunction.h>
#include <vtkImageData.h>
#include <vtkPiecewiseFunction.h>
#include <vtkRenderer.h>
#include <vtkSmartVolumeMapper.h>
#include <vtkTransform.h>
#include <vtkVolume.h>
#include <vtkVolumeProperty.h>

VolumeNode::VolumeNode(const std::string &statePath, const std::string &name)
    : GraphicsNode(statePath, name), m_hasVolume(false),
      m_vtkVolume(vtkSmartPointer<vtkVolume>::New()),
      m_mapper(vtkSmartPointer<vtkSmartVolumeMapper>::New()),
      m_imageData(vtkSmartPointer<vtkImageData>::New()),
      m_colorFunc(vtkSmartPointer<vtkColorTransferFunction>::New()),
      m_opacityFunc(vtkSmartPointer<vtkPiecewiseFunction>::New()),
      m_volumeProperty(vtkSmartPointer<vtkVolumeProperty>::New()), m_dataMin(0.0), m_dataMax(1.0),
      m_shading(true), m_ambient(0.3), m_diffuse(0.6), m_specular(0.2), m_specularPower(10.0),
      m_scalarOpacityUnitDistance(1.0), m_sampleDistance(0.5), m_autoAdjustSampleDistances(true) {
  // Initialize with empty 1x1x1 volume to avoid VTK errors before data is loaded
  m_imageData->SetDimensions(1, 1, 1);
  m_imageData->AllocateScalars(VTK_UNSIGNED_CHAR, 1);
  unsigned char *ptr = static_cast<unsigned char *>(m_imageData->GetScalarPointer());
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

  // Initialize state tree with all rendering attributes
  if (!statePath.empty()) {
    getState("visible").value(1); // Visible by default

    // Shading properties
    getState("shading").value(m_shading ? 1 : 0);
    getState("ambient").value(m_ambient);
    getState("diffuse").value(m_diffuse);
    getState("specular").value(m_specular);
    getState("specular_power").value(m_specularPower);

    // Sampling properties
    getState("scalar_opacity_unit_distance").value(m_scalarOpacityUnitDistance);
    getState("sample_distance").value(m_sampleDistance);
    getState("auto_adjust_sample_distances").value(m_autoAdjustSampleDistances ? 1 : 0);

    // Data range (will be updated when volume is loaded)
    getState("data_min").value(m_dataMin);
    getState("data_max").value(m_dataMax);

    // Transfer function state (stored as serialized arrays)
    getState("transfer_function.color").value("");
    getState("transfer_function.opacity").value("");
  }

  // Initialize with default transfer function
  setDefaultTransferFunction();
}

VolumeNode::~VolumeNode() {
  // Disconnect all signal connections to prevent new handlers from queuing
  m_dataConnection.disconnect();
  m_shadingConnection.disconnect();
  m_ambientConnection.disconnect();
  m_diffuseConnection.disconnect();
  m_specularConnection.disconnect();
  m_specularPowerConnection.disconnect();
  m_scalarOpacityUnitDistanceConnection.disconnect();
  m_sampleDistanceConnection.disconnect();
  m_autoAdjustSampleDistancesConnection.disconnect();

  // Note: Do NOT call waitForHandlers() here!
  // The base class state_object<VolumeNode> destructor will handle it,
  // but only AFTER our VTK members are destroyed. Calling it here would
  // allow handlers to access destroyed VTK objects.
}

vtkProp *VolumeNode::getProp() { return m_vtkVolume; }

void VolumeNode::applyTransformToVTK() {
  // Use generic helper to apply world transform
  applyWorldTransformToProps({m_vtkVolume});
}

void VolumeNode::applyClipPlanes(vtkPlaneCollection *planes) {
  if (m_mapper) {
    if (planes && planes->GetNumberOfItems() > 0) {
      m_mapper->SetClippingPlanes(planes);
    } else {
      m_mapper->RemoveAllClippingPlanes();
    }
  }
}

void VolumeNode::addToRenderer(vtkRenderer *renderer) {
  volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() + "]: Adding to renderer");
  GraphicsNode::addToRenderer(renderer);

  // Verify it was actually added and log detailed info
  if (renderer && renderer->GetVolumes()->IsItemPresent(m_vtkVolume)) {
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() +
                                "]: CONFIRMED - Volume is in renderer");
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() +
                                "]: Total volumes in renderer: " +
                                std::to_string(renderer->GetVolumes()->GetNumberOfItems()));

    // Log volume property details
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() + "]: Volume visibility: " +
                                std::to_string(m_vtkVolume->GetVisibility()));
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() + "]: Volume pickable: " +
                                std::to_string(m_vtkVolume->GetPickable()));

    // Log image data details
    int dims[3];
    m_imageData->GetDimensions(dims);
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() +
                                "]: Image data dimensions: [" + std::to_string(dims[0]) + ", " +
                                std::to_string(dims[1]) + ", " + std::to_string(dims[2]) + "]");

    double *bounds = m_vtkVolume->GetBounds();
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() + "]: Volume bounds: [" +
                                std::to_string(bounds[0]) + ", " + std::to_string(bounds[1]) +
                                ", " + std::to_string(bounds[2]) + ", " +
                                std::to_string(bounds[3]) + ", " + std::to_string(bounds[4]) +
                                ", " + std::to_string(bounds[5]) + "]");

    // Log transfer function ranges
    double colorRange[2], opacityRange[2];
    m_colorFunc->GetRange(colorRange);
    m_opacityFunc->GetRange(opacityRange);
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() + "]: Color TF range: [" +
                                std::to_string(colorRange[0]) + ", " +
                                std::to_string(colorRange[1]) + "]");
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() + "]: Opacity TF range: [" +
                                std::to_string(opacityRange[0]) + ", " +
                                std::to_string(opacityRange[1]) + "]");

    // Log opacity at a few sample points
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() + "]: Opacity at dataMin(" +
                                std::to_string(m_dataMin) +
                                "): " + std::to_string(m_opacityFunc->GetValue(m_dataMin)));
    volrover3::app().log(
        0, "VolumeNode::addToRenderer[" + getName() + "]: Opacity at dataMid: " +
               std::to_string(m_opacityFunc->GetValue((m_dataMin + m_dataMax) / 2.0)));
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() + "]: Opacity at dataMax(" +
                                std::to_string(m_dataMax) +
                                "): " + std::to_string(m_opacityFunc->GetValue(m_dataMax)));

    // Log scalar range from image data
    double *scalarRange = m_imageData->GetScalarRange();
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() +
                                "]: Image data scalar range: [" + std::to_string(scalarRange[0]) +
                                ", " + std::to_string(scalarRange[1]) + "]");
  } else {
    volrover3::app().log(0, "VolumeNode::addToRenderer[" + getName() +
                                "]: WARNING - Volume NOT in renderer!");
  }
}

void VolumeNode::setVolume(const cvc::volume &vol) {
  cvc::thread_info ti(volrover3::app(), BOOST_CURRENT_FUNCTION);

  volrover3::app().log(0, "\n=== VolumeNode::setVolume[" + getName() + "] ===");

  // Store the volume object
  m_volume = std::make_shared<cvc::volume>(vol);

  updateImageData(vol);
  m_dataMin = vol.min();
  m_dataMax = vol.max();

  // Update state tree with data range
  getState("data_min").value(m_dataMin);
  getState("data_max").value(m_dataMax);

  volrover3::app().log(0, "  Data range: [" + std::to_string(m_dataMin) + ", " +
                              std::to_string(m_dataMax) + "]");
  volrover3::app().log(0, "  Dimensions: [" + std::to_string(vol.XDim()) + ", " +
                              std::to_string(vol.YDim()) + ", " + std::to_string(vol.ZDim()) + "]");
  volrover3::app().log(0, "  Bounding box: [" + std::to_string(vol.XMin()) + "," +
                              std::to_string(vol.XMax()) + "], [" + std::to_string(vol.YMin()) +
                              "," + std::to_string(vol.YMax()) + "], [" +
                              std::to_string(vol.ZMin()) + "," + std::to_string(vol.ZMax()) + "]");
  volrover3::app().log(0, "  Spans: [" + std::to_string(vol.XSpan()) + ", " +
                              std::to_string(vol.YSpan()) + ", " + std::to_string(vol.ZSpan()) +
                              "]");

  // Calculate appropriate scalar opacity unit distance based on volume diagonal
  double dx = vol.XSpan();
  double dy = vol.YSpan();
  double dz = vol.ZSpan();
  double diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
  volrover3::app().log(0, "  Diagonal: " + std::to_string(diagonal) +
                              ", ScalarOpacityUnitDistance: " + std::to_string(diagonal / 100.0));
  m_volumeProperty->SetScalarOpacityUnitDistance(diagonal / 100.0);

  // Set transfer function using actual data range
  volrover3::app().log(0, "  Setting default transfer function...");
  setDefaultTransferFunction();
  updateTransferFunctions();
  updateMetadata(vol);
  m_hasVolume = true;

  // Update bbox to match volume bounds
  updateBoundingBoxNode();

  // Notify parent to resync bounds if it's a NullGraphicNode with auto-sync enabled
  if (m_parent) {
    auto nullParent = dynamic_cast<NullGraphicNode *>(m_parent);
    if (nullParent) {
      nullParent->syncBoundsToChildren();
    }
  }

  volrover3::app().log(0, "=================================\n");
}

void VolumeNode::updateImageData(const cvc::volume &vol) {
  volrover3::app().log(0, "\n  VolumeNode::updateImageData - Copying volume data to VTK...");

  // Get dimensions
  int dims[3] = {static_cast<int>(vol.XDim()), static_cast<int>(vol.YDim()),
                 static_cast<int>(vol.ZDim())};

  volrover3::app().log(0, "    CVC Volume bounds: X=[" + std::to_string(vol.XMin()) + ", " +
                              std::to_string(vol.XMax()) + "]");
  volrover3::app().log(0, "                       Y=[" + std::to_string(vol.YMin()) + ", " +
                              std::to_string(vol.YMax()) + "]");
  volrover3::app().log(0, "                       Z=[" + std::to_string(vol.ZMin()) + ", " +
                              std::to_string(vol.ZMax()) + "]");
  volrover3::app().log(0, "    CVC XSpan/YSpan/ZSpan: [" + std::to_string(vol.XSpan()) + ", " +
                              std::to_string(vol.YSpan()) + ", " + std::to_string(vol.ZSpan()) +
                              "]");

  // CRITICAL FIX: Calculate spacing directly from bounding box, not from Span() methods
  // The Span() methods appear to return incorrect values for some volumes
  double spacing[3] = {(vol.XMax() - vol.XMin()) / vol.XDim(),
                       (vol.YMax() - vol.YMin()) / vol.YDim(),
                       (vol.ZMax() - vol.ZMin()) / vol.ZDim()};

  volrover3::app().log(0, "    Calculated spacing: [" + std::to_string(spacing[0]) + ", " +
                              std::to_string(spacing[1]) + ", " + std::to_string(spacing[2]) + "]");

  // Get origin
  double origin[3] = {vol.XMin(), vol.YMin(), vol.ZMin()};

  volrover3::app().log(0, "    Origin: [" + std::to_string(origin[0]) + ", " +
                              std::to_string(origin[1]) + ", " + std::to_string(origin[2]) + "]");

  // Determine VTK scalar type
  int scalarType;
  std::string scalarTypeName;
  switch (vol.voxelType()) {
  case cvc::UChar:
    scalarType = VTK_UNSIGNED_CHAR;
    scalarTypeName = "UChar";
    break;
  case cvc::UShort:
    scalarType = VTK_UNSIGNED_SHORT;
    scalarTypeName = "UShort";
    break;
  case cvc::UInt:
    scalarType = VTK_UNSIGNED_INT;
    scalarTypeName = "UInt";
    break;
  case cvc::Float:
    scalarType = VTK_FLOAT;
    scalarTypeName = "Float";
    break;
  case cvc::Double:
    scalarType = VTK_DOUBLE;
    scalarTypeName = "Double";
    break;
  default:
    scalarType = VTK_FLOAT;
    scalarTypeName = "Float (default)";
    break;
  }

  volrover3::app().log(0, "    Voxel type: " + scalarTypeName);

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

  volrover3::app().log(0, "    Total voxels: " + std::to_string(numVoxels) +
                              ", bytes per voxel: " + std::to_string(bytesPerVoxel) +
                              ", total bytes: " + std::to_string(totalBytes));
  volrover3::app().log(0, "    CVC data pointer: " + std::string(cvcPtr ? "VALID" : "NULL"));
  volrover3::app().log(0, "    VTK data pointer: " + std::string(vtkPtr ? "VALID" : "NULL"));

  if (cvcPtr && vtkPtr) {
    std::memcpy(vtkPtr, cvcPtr, totalBytes);
    volrover3::app().log(0, "    \u2713 Data copied successfully");
  } else {
    volrover3::app().log(0, "    \u2717 ERROR: Cannot copy data - null pointer!");
  }

  m_imageData->Modified();
}

void VolumeNode::setTransferFunction(const std::vector<double> &colorTable,
                                     const std::vector<double> &opacityTable) {
  volrover3::app().log(0, "\nVolumeNode::setTransferFunction[" + getName() +
                              "]: " + std::to_string(colorTable.size() / 4) + " color pts, " +
                              std::to_string(opacityTable.size() / 2) + " opacity pts");

  // DEBUG: Log first few color values to see what we're getting
  if (colorTable.size() >= 8) {
    volrover3::app().log(0, "  First 2 color points:");
    volrover3::app().log(0, "    [0]: scalar=" + std::to_string(colorTable[0]) + ", rgb=(" +
                                std::to_string(colorTable[1]) + "," +
                                std::to_string(colorTable[2]) + "," +
                                std::to_string(colorTable[3]) + ")");
    volrover3::app().log(0, "    [1]: scalar=" + std::to_string(colorTable[4]) + ", rgb=(" +
                                std::to_string(colorTable[5]) + "," +
                                std::to_string(colorTable[6]) + "," +
                                std::to_string(colorTable[7]) + ")");
  }

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

    if (i < 3) { // Log first few points
      volrover3::app().log(0, "  Opacity[" + std::to_string(i) + "]: scalar=" +
                                  std::to_string(scalar) + ", opacity=" + std::to_string(opacity));
    }
  }

  updateTransferFunctions();

  // Save to state tree only if values changed
  // Build strings for comparison
  std::ostringstream colorStr, opacityStr;
  for (size_t i = 0; i < colorTable.size(); ++i) {
    if (i > 0)
      colorStr << ",";
    colorStr << std::fixed << std::setprecision(6) << colorTable[i];
  }
  for (size_t i = 0; i < opacityTable.size(); ++i) {
    if (i > 0)
      opacityStr << ",";
    opacityStr << std::fixed << std::setprecision(6) << opacityTable[i];
  }

  // Only update state if values actually changed
  std::string currentColorStr = getState("transfer_function.color").value();
  std::string currentOpacityStr = getState("transfer_function.opacity").value();

  if (colorStr.str() != currentColorStr) {
    getState("transfer_function.color").value(colorStr.str());
  }
  if (opacityStr.str() != currentOpacityStr) {
    getState("transfer_function.opacity").value(opacityStr.str());
  }
}

void VolumeNode::setDefaultTransferFunction() {
  m_colorFunc->RemoveAllPoints();
  m_opacityFunc->RemoveAllPoints();

  // Default grayscale color map using actual data range
  m_colorFunc->AddRGBPoint(m_dataMin, 0.0, 0.0, 0.0);
  m_colorFunc->AddRGBPoint(m_dataMax, 1.0, 1.0, 1.0);

  // Default opacity ramp using actual data range
  m_opacityFunc->AddPoint(m_dataMin, 0.0);
  m_opacityFunc->AddPoint(m_dataMax, 1.0);

  // Save to state tree
  std::ostringstream colorStr, opacityStr;
  colorStr << m_dataMin << ",0,0,0," << m_dataMax << ",1,1,1";
  opacityStr << m_dataMin << ",0," << m_dataMax << ",1";
  getState("transfer_function.color").value(colorStr.str());
  getState("transfer_function.opacity").value(opacityStr.str());
}

std::vector<double> VolumeNode::getTransferFunctionColorTable() const {
  std::string tableStr = getState("transfer_function.color").value();
  std::vector<double> table;

  if (tableStr.empty()) {
    return table;
  }

  std::istringstream iss(tableStr);
  std::string value;
  while (std::getline(iss, value, ',')) {
    table.push_back(std::stod(value));
  }

  return table;
}

std::vector<double> VolumeNode::getTransferFunctionOpacityTable() const {
  std::string tableStr = getState("transfer_function.opacity").value();
  std::vector<double> table;

  if (tableStr.empty()) {
    return table;
  }

  std::istringstream iss(tableStr);
  std::string value;
  while (std::getline(iss, value, ',')) {
    table.push_back(std::stod(value));
  }

  return table;
}

void VolumeNode::setShading(bool enabled) { getState("shading").value(enabled ? 1 : 0); }

void VolumeNode::setAmbient(double value) { getState("ambient").value(value); }

void VolumeNode::setDiffuse(double value) { getState("diffuse").value(value); }

void VolumeNode::setSpecular(double value) { getState("specular").value(value); }

void VolumeNode::setSpecularPower(double value) { getState("specular_power").value(value); }

void VolumeNode::setScalarOpacityUnitDistance(double value) {
  getState("scalar_opacity_unit_distance").value(value);
}

void VolumeNode::setSampleDistance(double value) { getState("sample_distance").value(value); }

void VolumeNode::setAutoAdjustSampleDistances(bool enabled) {
  getState("auto_adjust_sample_distances").value(enabled ? 1 : 0);
}

void VolumeNode::updateTransferFunctions() {
  static int callCount = 0;
  if (callCount++ == 0) {
    volrover3::app().log(0, "\nVolumeNode::updateTransferFunctions[" + getName() + "]: First call");
    volrover3::app().log(0, "  Data range: [" + std::to_string(m_dataMin) + ", " +
                                std::to_string(m_dataMax) + "]");
  }

  m_colorFunc->Modified();
  m_opacityFunc->Modified();
  m_volumeProperty->Modified();
  m_vtkVolume->Modified();
  m_mapper->Modified();
  m_imageData->Modified();
}

cvc::bounding_box VolumeNode::getBoundingBox() const {
  if (m_volume) {
    try {
      return cvc::bounding_box(m_volume->XMin(), m_volume->YMin(), m_volume->ZMin(),
                               m_volume->XMax(), m_volume->YMax(), m_volume->ZMax());
    } catch (...) {
      // Bounding box calculations can throw for empty/invalid volumes
      return cvc::bounding_box(0, 0, 0, 0, 0, 0);
    }
  }
  // Return empty bounding box
  return cvc::bounding_box(0, 0, 0, 0, 0, 0);
}

// Note: syncToState and syncFromState removed - state_object handles state synchronization
// automatically

void VolumeNode::handleStateChanged(const std::string &childState) {
  volrover3::app().log(2, str(boost::format("VolumeNode::handleStateChanged(%s) for '%s'") %
                              childState % getName()));

  // Handle volume-specific state changes
  if (childState == "shading") {
    runOnMainThread([this]() {
      m_shading = getState("shading").value<bool>();
      if (m_volumeProperty) {
        m_volumeProperty->SetShade(m_shading ? 1 : 0);
      }
    });
  } else if (childState == "ambient") {
    runOnMainThread([this]() {
      m_ambient = getState("ambient").value<double>();
      if (m_volumeProperty) {
        m_volumeProperty->SetAmbient(m_ambient);
      }
    });
  } else if (childState == "diffuse") {
    runOnMainThread([this]() {
      m_diffuse = getState("diffuse").value<double>();
      if (m_volumeProperty) {
        m_volumeProperty->SetDiffuse(m_diffuse);
      }
    });
  } else if (childState == "specular") {
    runOnMainThread([this]() {
      m_specular = getState("specular").value<double>();
      if (m_volumeProperty) {
        m_volumeProperty->SetSpecular(m_specular);
      }
    });
  } else if (childState == "specular_power") {
    runOnMainThread([this]() {
      m_specularPower = getState("specular_power").value<double>();
      if (m_volumeProperty) {
        m_volumeProperty->SetSpecularPower(m_specularPower);
      }
    });
  } else if (childState == "scalar_opacity_unit_distance") {
    runOnMainThread([this]() {
      m_scalarOpacityUnitDistance = getState("scalar_opacity_unit_distance").value<double>();
      if (m_volumeProperty) {
        m_volumeProperty->SetScalarOpacityUnitDistance(m_scalarOpacityUnitDistance);
      }
    });
  } else if (childState == "sample_distance") {
    runOnMainThread([this]() {
      m_sampleDistance = getState("sample_distance").value<double>();
      if (m_mapper) {
        m_mapper->SetSampleDistance(m_sampleDistance);
      }
    });
  } else if (childState == "auto_adjust_sample_distances") {
    runOnMainThread([this]() {
      m_autoAdjustSampleDistances = getState("auto_adjust_sample_distances").value<bool>();
      if (m_mapper) {
        m_mapper->SetAutoAdjustSampleDistances(m_autoAdjustSampleDistances ? 1 : 0);
      }
    });
  } else if (childState == "data_min" || childState == "data_max") {
    runOnMainThread([this]() {
      try {
        // Data range changed - update transfer functions
        m_dataMin = getState("data_min").value<double>();
        m_dataMax = getState("data_max").value<double>();
        setDefaultTransferFunction();
        updateTransferFunctions();
      } catch (const boost::bad_lexical_cast &) {
        // Ignore - state initialization may trigger before all components are set
      }
    });
  } else {
    // Delegate to parent for common graphics fields
    GraphicsNode::handleStateChanged(childState);
  }
}

bool VolumeNode::isComputedMetadata(const std::string &key) {
  // These metadata keys are computed from volume data and should be read-only
  static const std::set<std::string> computedKeys = {"dim_x",
                                                     "dim_y",
                                                     "dim_z",
                                                     "bbox_min_x",
                                                     "bbox_min_y",
                                                     "bbox_min_z",
                                                     "bbox_max_x",
                                                     "bbox_max_y",
                                                     "bbox_max_z",
                                                     "spacing_x",
                                                     "spacing_y",
                                                     "spacing_z",
                                                     "data_range_min",
                                                     "data_range_max",
                                                     "voxel_type",
                                                     "bounding_box",
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

void VolumeNode::updateMetadata(const cvc::volume &vol) {
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
  std::string bboxStr = std::to_string(vol.XMin()) + "," + std::to_string(vol.YMin()) + "," +
                        std::to_string(vol.ZMin()) + "," + std::to_string(vol.XMax()) + "," +
                        std::to_string(vol.YMax()) + "," + std::to_string(vol.ZMax());
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
  case cvc::UChar:
    typeStr = "unsigned_char";
    break;
  case cvc::UShort:
    typeStr = "unsigned_short";
    break;
  case cvc::UInt:
    typeStr = "unsigned_int";
    break;
  case cvc::Float:
    typeStr = "float";
    break;
  case cvc::Double:
    typeStr = "double";
    break;
  default:
    typeStr = "unknown";
    break;
  }
  setMetadata("voxel_type", typeStr);
}

void VolumeNode::onDataChanged() {
  // Called when state data changes - reload volume from state
  // Note: With state_object, we access state via getState() instead of m_stateNode
  if (getState().isData<cvc::volume>()) {
    try {
      const cvc::volume &vol = boost::any_cast<const cvc::volume &>(getState().data());
      setVolume(vol);
    } catch (...) {
      // Failed to load volume from state
    }
  }
}
