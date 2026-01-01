#include <volrover3/SceneGraph.h>
#include <volrover3/SceneNode.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/GraphicsNode.h>
#include <volrover3/NullGraphicNode.h>
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
    , m_graphicsRoot(nullptr)
    , m_nullGraphic(nullptr)
    , m_multiVolumeRenderingEnabled(false)
{
    m_rootNodes.push_back(m_gridNode);
    m_rootNodes.push_back(m_axisNode);
    
    // Create null graphic as THE root graphics node (all graphics go under this)
    m_nullGraphic = std::make_shared<NullGraphicNode>("root");
    m_nullGraphic->setShowBBox(true);  // Show bbox by default
    m_nullGraphic->setBounds(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5);  // Default unit cube when empty
    m_graphicsRoot = m_nullGraphic;  // NullGraphic IS the graphics root
    m_rootNodes.push_back(m_graphicsRoot);
    
    // Set colors for nodes from AppState
    double r, g, b;
    AppState::instance().getGridColor(r, g, b);
    m_gridNode->setColor(r, g, b);
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
    
    // Process each direct child of the graphics root
    // Each child's getCombinedBoundingBox() already includes its descendants recursively
    if (m_graphicsRoot) {
        for (const auto& child : m_graphicsRoot->getGraphicsChildren()) {
            if (!child) continue;
            
            // Get combined bbox of this child (includes all its descendants in local space)
            cvc::bounding_box childBBox = child->getCombinedBoundingBox();
            
            // Skip invalid bounding boxes
            if (childBBox[0] > childBBox[3] || 
                childBBox[1] > childBBox[4] || 
                childBBox[2] > childBBox[5]) {
                continue;
            }
            
            // Apply world transform to the bounding box by transforming all 8 corners
            vtkSmartPointer<vtkMatrix4x4> worldTransform = child->getWorldTransform();
            
            double corners[8][3] = {
                {childBBox[0], childBBox[1], childBBox[2]},  // min, min, min
                {childBBox[3], childBBox[1], childBBox[2]},  // max, min, min
                {childBBox[0], childBBox[4], childBBox[2]},  // min, max, min
                {childBBox[3], childBBox[4], childBBox[2]},  // max, max, min
                {childBBox[0], childBBox[1], childBBox[5]},  // min, min, max
                {childBBox[3], childBBox[1], childBBox[5]},  // max, min, max
                {childBBox[0], childBBox[4], childBBox[5]},  // min, max, max
                {childBBox[3], childBBox[4], childBBox[5]}   // max, max, max
            };
            
            // Transform all corners and find new axis-aligned bounds
            double minx = std::numeric_limits<double>::max();
            double miny = std::numeric_limits<double>::max();
            double minz = std::numeric_limits<double>::max();
            double maxx = std::numeric_limits<double>::lowest();
            double maxy = std::numeric_limits<double>::lowest();
            double maxz = std::numeric_limits<double>::lowest();
            
            for (int i = 0; i < 8; ++i) {
                double in[4] = {corners[i][0], corners[i][1], corners[i][2], 1.0};
                double out[4];
                worldTransform->MultiplyPoint(in, out);
                
                minx = std::min(minx, out[0]);
                miny = std::min(miny, out[1]);
                minz = std::min(minz, out[2]);
                maxx = std::max(maxx, out[0]);
                maxy = std::max(maxy, out[1]);
                maxz = std::max(maxz, out[2]);
            }
            
            // Merge with combined bounds
            if (first) {
                combinedBounds = cvc::bounding_box(minx, miny, minz, maxx, maxy, maxz);
                first = false;
            } else {
                combinedBounds[0] = std::min(combinedBounds[0], minx);
                combinedBounds[1] = std::min(combinedBounds[1], miny);
                combinedBounds[2] = std::min(combinedBounds[2], minz);
                combinedBounds[3] = std::max(combinedBounds[3], maxx);
                combinedBounds[4] = std::max(combinedBounds[4], maxy);
                combinedBounds[5] = std::max(combinedBounds[5], maxz);
            }
        }
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
    
    // Remove null graphic since we now have real graphics
    removeNullGraphicIfPresent();
    
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
    
    // Remove null graphic since we now have real graphics
    removeNullGraphicIfPresent();
    
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
    
    // If scene is now empty, add null graphic back
    ensureNullGraphicIfEmpty();
    
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
    
    // Sync the graphics root (null graphic) to state
    // This ensures all graphics appear as children of the null graphic in the state tree
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
    
    // Remove null graphic since we now have real graphics
    removeNullGraphicIfPresent();
    
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

void SceneGraph::ensureNullGraphicIfEmpty()
{
    // With new architecture: NullGraphicNode IS the graphics root, always present
    // No need to add/remove it
}

void SceneGraph::removeNullGraphicIfPresent()
{
    // With new architecture: NullGraphicNode IS the graphics root, always present
    // No need to add/remove it
}
