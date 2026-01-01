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
    , m_gridNode(std::make_shared<GridNode>())
    , m_axisNode(std::make_shared<AxisNode>())
    , m_worldBBoxNode(std::make_shared<BBoxNode>())
    , m_graphicsRoot(std::make_shared<GeometryNode>("graphics"))
    , m_multiVolumeRenderingEnabled(false)
{
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
    m_worldBBoxNode->setCoordinateLabelColor(r, g, b);
    m_worldBBoxNode->setCoordinateLabelFontSize(AppState::instance().worldBBoxCoordinateFontSize());
    m_worldBBoxNode->setCoordinatesVisible(AppState::instance().worldBBoxCoordinatesVisible());
    // Set bbox visibility from AppState before renderer is attached
    m_worldBBoxNode->setVisible(AppState::instance().worldBBoxVisible());
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
    m_worldBBoxNode->setCoordinatesVisible(visible);
    m_worldBBoxNode->setCoordinateLabelColor(r, g, b);
    m_worldBBoxNode->setCoordinateLabelFontSize(fontSize);
}

void SceneGraph::updateTransferFunction(const std::vector<double> &colorTable,
                                        const std::vector<double> &opacityTable)
{
    // Apply transfer function to all volume nodes
    auto volumes = getAllVolumeGraphics();
    for (auto& volNode : volumes) {
        volNode->setTransferFunction(colorTable, opacityTable);
    }
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
    graphicsState.comment("Unified graphics tree (geometry and volumes)");
    
    // Sync children directly to graphics state (not the root node itself)
    // This avoids creating graphics/graphics_root/... redundancy
    for (const auto& child : m_graphicsRoot->getGraphicsChildren()) {
        child->syncToState(graphicsState);
    }
}

void SceneGraph::syncGraphicsFromState()
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Get the graphics state node
    cvc::state& graphicsState = cvc::state::instance()(m_statePrefix)("graphics");
    
    if (!graphicsState.initialized()) {
        return; // No graphics state to load
    }
    
    // Load children directly from graphics state
    // Note: For now, we don't automatically reconstruct the tree from state.
    // Graphics are added via addGraphics() which handles both geometry and volumes,
    // managing both memory and state. This method would need a type registry to
    // automatically instantiate the correct node types.
    
    // TODO: Implement full state-to-memory reconstruction when needed
}

// Volume graphics management
std::shared_ptr<VolumeNode> SceneGraph::addGraphics(const std::string& name, const cvc::volume& vol)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Check if name already exists
    if (m_graphicsNodes.find(name) != m_graphicsNodes.end()) {
        cvcapp.log(0, "SceneGraph::addGraphics: Volume '" + name + "' already exists, replacing");
        removeGraphics(name);
    }
    
    // Create new volume node
    auto volumeNode = std::make_shared<VolumeNode>(name);
    volumeNode->setVolume(vol);
    
    // Add to graphics root
    m_graphicsRoot->addGraphicsChild(volumeNode);
    
    // Add to lookup map
    m_graphicsNodes[name] = volumeNode;
    
    // Sync to state tree
    syncGraphicsToState();
    
    // Update multi-volume rendering if needed
    updateVolumeRendering();
    
    return volumeNode;
}



std::vector<std::shared_ptr<VolumeNode>> SceneGraph::getAllVolumeGraphics()
{
    std::vector<std::shared_ptr<VolumeNode>> volumes;
    // Filter volume nodes from the unified graphics tree
    for (const auto& pair : m_graphicsNodes) {
        auto volumeNode = std::dynamic_pointer_cast<VolumeNode>(pair.second);
        if (volumeNode) {
            volumes.push_back(volumeNode);
        }
    }
    return volumes;
}



size_t SceneGraph::getVolumeGraphicsCount() const
{
    // Count volume nodes in the unified graphics tree
    size_t count = 0;
    for (const auto& pair : m_graphicsNodes) {
        if (std::dynamic_pointer_cast<VolumeNode>(pair.second)) {
            ++count;
        }
    }
    return count;
}

// Volume sync methods removed - volumes are now part of the unified graphics tree
// and sync automatically via syncGraphicsToState() / syncGraphicsFromState()

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
    
    // Start from unified graphics root (includes volumes)
    if (m_graphicsRoot) {
        processBounds(m_graphicsRoot);
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
