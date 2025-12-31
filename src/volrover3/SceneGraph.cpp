#include <volrover3/SceneGraph.h>
#include <volrover3/SceneNode.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/GridNode.h>
#include <volrover3/AxisNode.h>
#include <volrover3/BBoxNode.h>
#include <volrover3/AppState.h>
#include <cvc/geometry.h>
#include <cvc/volume.h>
#include <cvc/state.h>
#include <cvc/app.h>
#include <vtkRenderer.h>
#include <vtkMultiVolume.h>
#include <vtkGPUVolumeRayCastMapper.h>
#include <algorithm>
#include <limits>

SceneGraph::SceneGraph(const std::string& statePrefix)
    : m_renderer(nullptr)
    , m_statePrefix(statePrefix)
    , m_geometryNode(std::make_shared<GeometryNode>())
    , m_volumeNode(std::make_shared<VolumeNode>())
    , m_gridNode(std::make_shared<GridNode>())
    , m_axisNode(std::make_shared<AxisNode>())
    , m_worldBBoxNode(std::make_shared<BBoxNode>())
    , m_graphicsRoot(std::make_shared<GeometryNode>("graphics_root"))
    , m_volumeGraphicsRoot(std::make_shared<VolumeNode>("volume_graphics_root"))
    , m_multiVolumeRenderingEnabled(false)
{
    m_rootNodes.push_back(m_geometryNode);
    m_rootNodes.push_back(m_volumeNode);
    m_rootNodes.push_back(m_gridNode);
    m_rootNodes.push_back(m_axisNode);
    m_rootNodes.push_back(m_worldBBoxNode);
    m_rootNodes.push_back(m_graphicsRoot); // Add graphics root to scene
    
    // Set colors for nodes from AppState
    double r, g, b;
    AppState::instance().getGridColor(r, g, b);
    m_gridNode->setColor(r, g, b);
    
    AppState::instance().getWorldBBoxCoordinateColor(r, g, b);
    m_worldBBoxNode->setColor(1.0, 1.0, 1.0); // White for world bbox
    m_worldBBoxNode->setTickLabelColor(r, g, b);
    m_worldBBoxNode->setTickLabelFontSize(AppState::instance().worldBBoxCoordinateFontSize());
    m_worldBBoxNode->setTickInterval(0.0); // No interval - only show at vertices
    m_worldBBoxNode->setTicksVisible(AppState::instance().worldBBoxCoordinatesVisible());
    m_worldBBoxNode->setVisible(false); // Hidden by default
}

SceneGraph::~SceneGraph()
{
    if (m_renderer) {
        for (auto &node : m_rootNodes) {
            node->removeFromRenderer(m_renderer);
        }
    }
}

void SceneGraph::setRenderer(vtkRenderer *renderer)
{
    if (m_renderer) {
        for (auto &node : m_rootNodes) {
            node->removeFromRenderer(m_renderer);
        }
    }

    m_renderer = renderer;

    if (m_renderer) {
        for (auto &node : m_rootNodes) {
            node->addToRenderer(m_renderer);
        }
    }
}

void SceneGraph::update()
{
    for (auto &node : m_rootNodes) {
        node->update();
    }
}

void SceneGraph::setGeometry(const cvc::geometry &geom)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    m_geometryNode->setGeometry(geom);
}

void SceneGraph::setVolume(const cvc::volume &vol)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    m_volumeNode->setVolume(vol);
}

void SceneGraph::setGridVisible(bool visible)
{
    m_gridNode->setVisible(visible);
}

void SceneGraph::setAxisVisible(bool visible)
{
    m_axisNode->setVisible(visible);
}

void SceneGraph::setGridColor(double r, double g, double b)
{
    m_gridNode->setColor(r, g, b);
}

void SceneGraph::updateGrid(const cvc::bounding_box& bounds)
{
    m_gridNode->setBounds(bounds);
    
    // Scale axis length to be proportional to bounding box size
    double spanX = bounds[3] - bounds[0];
    double spanY = bounds[4] - bounds[1];
    double spanZ = bounds[5] - bounds[2];
    double maxSpan = std::max({spanX, spanY, spanZ});
    
    // Set axis to be about 20% of the maximum span
    double axisLength = maxSpan * 0.2;
    if (axisLength > 0.0) {
        m_axisNode->setAxisLength(axisLength);
    }
}

void SceneGraph::setGridPlaneVisibility(bool yz, bool xz, bool xy)
{
    m_gridNode->setYZPlaneVisible(yz);
    m_gridNode->setXZPlaneVisible(xz);
    m_gridNode->setXYPlaneVisible(xy);
}

void SceneGraph::setGridDivisions(int x, int y, int z)
{
    m_gridNode->setGridDivisions(x, y, z);
}

void SceneGraph::setGridTickIntervals(int x, int y, int z)
{
    m_gridNode->setTickIntervals(x, y, z);
}

void SceneGraph::setGridPlaneColors(double yzR, double yzG, double yzB,
                                     double xzR, double xzG, double xzB,
                                     double xyR, double xyG, double xyB)
{
    m_gridNode->setYZPlaneColor(yzR, yzG, yzB);
    m_gridNode->setXZPlaneColor(xzR, xzG, xzB);
    m_gridNode->setXYPlaneColor(xyR, xyG, xyB);
}

void SceneGraph::setGridTickLabelProperties(double r, double g, double b, int fontSize)
{
    m_gridNode->setTickLabelColor(r, g, b);
    m_gridNode->setTickLabelFontSize(fontSize);
}

void SceneGraph::setWorldBBoxVisible(bool visible)
{
    m_worldBBoxNode->setVisible(visible);
}

void SceneGraph::setWorldBBoxColor(double r, double g, double b)
{
    m_worldBBoxNode->setColor(r, g, b);
}

void SceneGraph::updateWorldBBox(const cvc::bounding_box& bounds)
{
    m_worldBBoxNode->setBoundingBox(bounds);
}

void SceneGraph::setWorldBBoxCoordinates(bool visible, double r, double g, double b, int fontSize)
{
    m_worldBBoxNode->setTicksVisible(visible);
    m_worldBBoxNode->setTickInterval(0.0); // No interval - only vertices
    m_worldBBoxNode->setTickLabelColor(r, g, b);
    m_worldBBoxNode->setTickLabelFontSize(fontSize);
}

void SceneGraph::updateTransferFunction(const std::vector<double> &colorTable,
                                        const std::vector<double> &opacityTable)
{
    m_volumeNode->setTransferFunction(colorTable, opacityTable);
}

cvc::bounding_box SceneGraph::computeGraphicsBounds() const
{
    cvc::bounding_box combinedBounds;
    bool first = true;
    
    // Helper function to process graphics nodes recursively
    std::function<void(const std::shared_ptr<GraphicsNode>&)> processBounds = 
        [&](const std::shared_ptr<GraphicsNode>& node) {
            if (!node) return;
            
            // Get geometry if available (try casting to GeometryNode)
            auto geomNode = std::dynamic_pointer_cast<GeometryNode>(node);
            if (geomNode && geomNode->hasGeometry() && geomNode->getGeometry()) {
                cvc::bounding_box geomBounds = geomNode->getGeometry()->extents();
                
                if (first) {
                    combinedBounds = geomBounds;
                    first = false;
                } else {
                    // Expand to include this geometry
                    combinedBounds[0] = std::min(combinedBounds[0], geomBounds[0]);
                    combinedBounds[1] = std::min(combinedBounds[1], geomBounds[1]);
                    combinedBounds[2] = std::min(combinedBounds[2], geomBounds[2]);
                    combinedBounds[3] = std::max(combinedBounds[3], geomBounds[3]);
                    combinedBounds[4] = std::max(combinedBounds[4], geomBounds[4]);
                    combinedBounds[5] = std::max(combinedBounds[5], geomBounds[5]);
                }
            }
            
            // Process children recursively
            for (const auto& child : node->getGraphicsChildren()) {
                processBounds(child);
            }
        };
    
    // Start from root graphics node
    if (m_graphicsRoot) {
        processBounds(m_graphicsRoot);
    }
    
    return combinedBounds;
}

// Multi-object graphics management
std::shared_ptr<GraphicsNode> SceneGraph::addGraphics(const std::string& name, const cvc::geometry& geom)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Check if name already exists
    if (m_graphicsNodes.find(name) != m_graphicsNodes.end()) {
        cvcapp.log(0, "SceneGraph::addGraphics: Graphics object '" + name + "' already exists, replacing");
        removeGraphics(name);
    }
    
    // Create new geometry node
    auto graphicsNode = std::make_shared<GeometryNode>(name);
    graphicsNode->setGeometry(geom);
    
    // Add to graphics root
    m_graphicsRoot->addGraphicsChild(graphicsNode);
    
    // Add to lookup map
    m_graphicsNodes[name] = graphicsNode;
    
    // Sync to state tree
    syncGraphicsToState();
    
    return graphicsNode;
}

std::shared_ptr<GraphicsNode> SceneGraph::addGraphics(const std::string& name)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Check if name already exists
    if (m_graphicsNodes.find(name) != m_graphicsNodes.end()) {
        cvcapp.log(0, "SceneGraph::addGraphics: Graphics object '" + name + "' already exists, replacing");
        removeGraphics(name);
    }
    
    // Create new empty geometry node (for hierarchy/grouping)
    auto graphicsNode = std::make_shared<GeometryNode>(name);
    
    // Add to graphics root
    m_graphicsRoot->addGraphicsChild(graphicsNode);
    
    // Add to lookup map
    m_graphicsNodes[name] = graphicsNode;
    
    // Sync to state tree
    syncGraphicsToState();
    
    return graphicsNode;
}

void SceneGraph::removeGraphics(const std::string& name)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    auto it = m_graphicsNodes.find(name);
    if (it == m_graphicsNodes.end()) {
        cvcapp.log(0, "SceneGraph::removeGraphics: Graphics object '" + name + "' not found");
        return;
    }
    
    auto graphicsNode = it->second;
    
    // Remove from graphics root
    m_graphicsRoot->removeGraphicsChild(graphicsNode);
    
    // Remove from lookup map
    m_graphicsNodes.erase(it);
    
    // Sync to state tree
    syncGraphicsToState();
}

std::shared_ptr<GraphicsNode> SceneGraph::getGraphics(const std::string& name)
{
    auto it = m_graphicsNodes.find(name);
    if (it != m_graphicsNodes.end()) {
        return it->second;
    }
    return nullptr;
}

void SceneGraph::registerGraphics(const std::string& name, std::shared_ptr<GraphicsNode> node)
{
    if (node) {
        m_graphicsNodes[name] = node;
    }
}

void SceneGraph::syncGraphicsToState()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Get or create the graphics state node
    cvc::state& graphicsState = cvc::state::instance()(m_statePrefix)("graphics");
    
    // Sync the graphics root and all children
    m_graphicsRoot->syncToState(graphicsState);
}

void SceneGraph::syncGraphicsFromState()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Get the graphics state node
    cvc::state& graphicsState = cvc::state::instance()(m_statePrefix)("graphics");
    
    if (!graphicsState.initialized()) {
        return; // No graphics state to load
    }
    
    // Sync the graphics root and all children
    m_graphicsRoot->syncFromState(graphicsState);
}

// Volume graphics management
std::shared_ptr<VolumeNode> SceneGraph::addVolumeGraphics(const std::string& name, const cvc::volume& vol)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Check if name already exists
    if (m_volumeGraphicsNodes.find(name) != m_volumeGraphicsNodes.end()) {
        cvcapp.log(0, "SceneGraph::addVolumeGraphics: Volume graphics object '" + name + "' already exists, replacing");
        removeVolumeGraphics(name);
    }
    
    // Create new volume graphics node
    auto volumeGraphicsNode = std::make_shared<VolumeNode>(name);
    volumeGraphicsNode->setVolume(vol);
    
    // Add to volume graphics root
    m_volumeGraphicsRoot->addGraphicsChild(volumeGraphicsNode);
    
    // Add to lookup map
    m_volumeGraphicsNodes[name] = volumeGraphicsNode;
    
    // Sync to state tree
    syncVolumesToState();
    
    // Update multi-volume rendering if needed
    updateVolumeRendering();
    
    return volumeGraphicsNode;
}

std::shared_ptr<VolumeNode> SceneGraph::addVolumeGraphics(const std::string& name)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Check if name already exists
    if (m_volumeGraphicsNodes.find(name) != m_volumeGraphicsNodes.end()) {
        cvcapp.log(0, "SceneGraph::addVolumeGraphics: Volume graphics object '" + name + "' already exists, replacing");
        removeVolumeGraphics(name);
    }
    
    // Create new empty volume graphics node (for hierarchy/grouping)
    auto volumeGraphicsNode = std::make_shared<VolumeNode>(name);
    
    // Add to volume graphics root
    m_volumeGraphicsRoot->addGraphicsChild(volumeGraphicsNode);
    
    // Add to lookup map
    m_volumeGraphicsNodes[name] = volumeGraphicsNode;
    
    // Sync to state tree
    syncVolumesToState();
    
    return volumeGraphicsNode;
}

void SceneGraph::removeVolumeGraphics(const std::string& name)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    auto it = m_volumeGraphicsNodes.find(name);
    if (it == m_volumeGraphicsNodes.end()) {
        cvcapp.log(0, "SceneGraph::removeVolumeGraphics: Volume graphics object '" + name + "' not found");
        return;
    }
    
    auto volumeGraphicsNode = it->second;
    
    // Remove from volume graphics root
    m_volumeGraphicsRoot->removeGraphicsChild(volumeGraphicsNode);
    
    // Remove from lookup map
    m_volumeGraphicsNodes.erase(it);
    
    // Sync to state tree
    syncVolumesToState();
    
    // Update multi-volume rendering
    updateVolumeRendering();
}

std::shared_ptr<VolumeNode> SceneGraph::getVolumeGraphics(const std::string& name)
{
    auto it = m_volumeGraphicsNodes.find(name);
    if (it != m_volumeGraphicsNodes.end()) {
        return it->second;
    }
    return nullptr;
}

std::vector<std::shared_ptr<VolumeNode>> SceneGraph::getAllVolumeGraphics()
{
    std::vector<std::shared_ptr<VolumeNode>> volumes;
    for (const auto& pair : m_volumeGraphicsNodes) {
        volumes.push_back(pair.second);
    }
    return volumes;
}

void SceneGraph::registerVolumeGraphics(const std::string& name, std::shared_ptr<VolumeNode> node)
{
    if (node) {
        m_volumeGraphicsNodes[name] = node;
    }
}

size_t SceneGraph::getVolumeGraphicsCount() const
{
    return m_volumeGraphicsNodes.size();
}

void SceneGraph::syncVolumesToState()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Get or create the volume graphics state node
    cvc::state& volumeGraphicsState = cvc::state::instance()(m_statePrefix)("volume_graphics");
    
    // Sync the volume graphics root and all children
    m_volumeGraphicsRoot->syncToState(volumeGraphicsState);
}

void SceneGraph::syncVolumesFromState()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Get the volume graphics state node
    cvc::state& volumeGraphicsState = cvc::state::instance()(m_statePrefix)("volume_graphics");
    
    if (!volumeGraphicsState.initialized()) {
        return; // No volume graphics state to load
    }
    
    // Sync the volume graphics root and all children
    m_volumeGraphicsRoot->syncFromState(volumeGraphicsState);
}

cvc::bounding_box SceneGraph::computeVolumeBounds() const
{
    cvc::bounding_box combinedBounds;
    bool first = true;
    
    // Helper function to process volume graphics nodes recursively
    std::function<void(const std::shared_ptr<GraphicsNode>&)> processBounds = 
        [&](const std::shared_ptr<GraphicsNode>& node) {
            if (!node) return;
            
            // Check if this is a VolumeNode
            if (auto volNode = std::dynamic_pointer_cast<VolumeNode>(node)) {
                // Get volume if available
                if (volNode->hasVolume() && volNode->getVolume()) {
                    cvc::bounding_box volBounds = volNode->getVolume()->boundingBox();
                    
                    if (first) {
                        combinedBounds = volBounds;
                        first = false;
                    } else {
                        // Expand to include this volume
                        combinedBounds[0] = std::min(combinedBounds[0], volBounds[0]);
                        combinedBounds[1] = std::min(combinedBounds[1], volBounds[1]);
                        combinedBounds[2] = std::min(combinedBounds[2], volBounds[2]);
                        combinedBounds[3] = std::max(combinedBounds[3], volBounds[3]);
                        combinedBounds[4] = std::max(combinedBounds[4], volBounds[4]);
                        combinedBounds[5] = std::max(combinedBounds[5], volBounds[5]);
                    }
                }
            }
            
            // Process children recursively
            for (const auto& child : node->getGraphicsChildren()) {
                processBounds(child);
            }
        };
    
    // Start from root volume graphics node
    if (m_volumeGraphicsRoot) {
        processBounds(m_volumeGraphicsRoot);
    }
    
    return combinedBounds;
}

void SceneGraph::enableMultiVolumeRendering(bool enable)
{
    if (m_multiVolumeRenderingEnabled == enable) {
        return; // No change
    }
    
    m_multiVolumeRenderingEnabled = enable;
    
    if (enable) {
        setupMultiVolumeRendering();
    } else {
        teardownMultiVolumeRendering();
    }
}

bool SceneGraph::isMultiVolumeRenderingEnabled() const
{
    return m_multiVolumeRenderingEnabled;
}

void SceneGraph::setupMultiVolumeRendering()
{
    if (!m_renderer) {
        return;
    }
    
    // Create multi-volume if not already created
    if (!m_multiVolume) {
        m_multiVolume = vtkSmartPointer<vtkMultiVolume>::New();
    }
    
    // Collect all volume graphics nodes
    auto allVolumes = getAllVolumeGraphics();
    
    if (allVolumes.size() <= 1) {
        return; // No need for multi-volume rendering with 0 or 1 volume
    }
    
    // TODO: Implement proper multi-volume rendering with GraphicsNode architecture
    // For now, individual volumes are rendered separately
    // Remove individual volume props from renderer
    /*
    for (const auto& volNode : allVolumes) {
        volNode->removeFromRenderer(m_renderer);
    }
    
    // Add all volumes to the multi-volume
    int port = 0;
    for (const auto& volNode : allVolumes) {
        // Note: vtkMultiVolume SetVolume takes a port number, not a transform
        // Transforms should be already applied to individual vtkVolume actors
        m_multiVolume->SetVolume(vol, port++);
    }
    
    // Add multi-volume to renderer
    m_renderer->AddViewProp(m_multiVolume);
    */
}

void SceneGraph::teardownMultiVolumeRendering()
{
    if (!m_renderer || !m_multiVolume) {
        return;
    }
    
    // TODO: Implement proper multi-volume teardown with GraphicsNode architecture
    // For now, individual volumes are rendered separately
    /*
    // Remove multi-volume from renderer
    m_renderer->RemoveViewProp(m_multiVolume);
    
    // Re-add individual volume props
    auto allVolumes = getAllVolumeGraphics();
    for (const auto& volNode : allVolumes) {
        volNode->addToRenderer(m_renderer);
    }
    */
}

void SceneGraph::updateVolumeRendering()
{
    if (!m_renderer) {
        return;
    }
    
    size_t volumeCount = getVolumeGraphicsCount();
    
    // Enable multi-volume rendering if we have more than 1 volume
    if (volumeCount > 1 && !m_multiVolumeRenderingEnabled) {
        enableMultiVolumeRendering(true);
    } else if (volumeCount <= 1 && m_multiVolumeRenderingEnabled) {
        enableMultiVolumeRendering(false);
    }
}
