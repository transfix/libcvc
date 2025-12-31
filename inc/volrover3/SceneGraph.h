#ifndef SCENEGRAPH_H
#define SCENEGRAPH_H

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <cvc/bounding_box.h>
#include <vtkSmartPointer.h>

class vtkRenderer;
class vtkMultiVolume;
class SceneNode;
class GeometryNode;
class GraphicsNode;
class VolumeNode;
class VolumeNode;
class GridNode;
class AxisNode;
class BBoxNode;

namespace cvc {
    class geometry;
    class volume;
    class state;
}

class SceneGraph
{
public:
    SceneGraph(const std::string& statePrefix = "volrover3");
    ~SceneGraph();
    
    // Get the state prefix for this scene graph
    std::string getStatePrefix() const { return m_statePrefix; }

    void setRenderer(vtkRenderer *renderer);
    void update();

    // Legacy scene content management (single objects)
    void setGeometry(const cvc::geometry &geom);
    void setVolume(const cvc::volume &vol);
    
    // Multi-object graphics management
    std::shared_ptr<GraphicsNode> addGraphics(const std::string& name, const cvc::geometry& geom);
    std::shared_ptr<GraphicsNode> addGraphics(const std::string& name); // Empty graphics node for hierarchy
    void removeGraphics(const std::string& name);
    std::shared_ptr<GraphicsNode> getGraphics(const std::string& name);
    std::shared_ptr<GraphicsNode> getGraphicsRoot() { return m_graphicsRoot; }
    const std::map<std::string, std::shared_ptr<GraphicsNode>>& getAllGraphics() const { return m_graphicsNodes; }
    void registerGraphics(const std::string& name, std::shared_ptr<GraphicsNode> node); // For manual registration
    
    // Multi-object volume graphics management
    std::shared_ptr<VolumeNode> addVolumeGraphics(const std::string& name, const cvc::volume& vol);
    std::shared_ptr<VolumeNode> addVolumeGraphics(const std::string& name); // Empty volume node for hierarchy
    void removeVolumeGraphics(const std::string& name);
    std::shared_ptr<VolumeNode> getVolumeGraphics(const std::string& name);
    std::shared_ptr<VolumeNode> getVolumeGraphicsRoot() { return m_volumeGraphicsRoot; }
    std::vector<std::shared_ptr<VolumeNode>> getAllVolumeGraphics();
    size_t getVolumeGraphicsCount() const;
    void registerVolumeGraphics(const std::string& name, std::shared_ptr<VolumeNode> node); // For manual registration
    
    // Multi-volume rendering control
    void enableMultiVolumeRendering(bool enable);
    bool isMultiVolumeRenderingEnabled() const;
    
    // State synchronization
    void syncGraphicsToState();
    void syncGraphicsFromState();
    void syncVolumesToState();
    void syncVolumesFromState();
    
    // Scene element visibility
    void setGridVisible(bool visible);
    void setAxisVisible(bool visible);
    void setWorldBBoxVisible(bool visible);
    
    // Scene element colors
    void setGridColor(double r, double g, double b);
    void setWorldBBoxColor(double r, double g, double b);
    
    // Update world bounding box
    void updateWorldBBox(const cvc::bounding_box& bounds);
    
    // World bbox coordinate controls (no interval - shows at vertices only)
    void setWorldBBoxCoordinates(bool visible, double r, double g, double b, int fontSize);
    
    // Grid plane visibility
    void setGridPlaneVisibility(bool yz, bool xz, bool xy);
    
    // Grid divisions
    void setGridDivisions(int x, int y, int z);
    
    // Grid tick intervals
    void setGridTickIntervals(int x, int y, int z);
    
    // Per-plane grid colors
    void setGridPlaneColors(double yzR, double yzG, double yzB,
                            double xzR, double xzG, double xzB,
                            double xyR, double xyG, double xyB);
    
    // Grid tick label properties
    void setGridTickLabelProperties(double r, double g, double b, int fontSize);
    
    // Update grid to match bounds
    void updateGrid(const cvc::bounding_box& bounds);
    
    // Compute combined bounding box of all graphics
    cvc::bounding_box computeGraphicsBounds() const;
    
    // Compute combined bounding box of all volumes
    cvc::bounding_box computeVolumeBounds() const;

    // Transfer function update
    void updateTransferFunction(const std::vector<double> &colorTable,
                                const std::vector<double> &opacityTable);

private:
    vtkRenderer *m_renderer;
    std::string m_statePrefix;
    
    // Legacy single-object nodes
    std::shared_ptr<GeometryNode> m_geometryNode;
    std::shared_ptr<VolumeNode> m_volumeNode;
    std::shared_ptr<GridNode> m_gridNode;
    std::shared_ptr<AxisNode> m_axisNode;
    std::shared_ptr<BBoxNode> m_worldBBoxNode;

    std::vector<std::shared_ptr<SceneNode>> m_rootNodes;
    
    // Multi-object graphics system
    std::shared_ptr<GraphicsNode> m_graphicsRoot; // Root node for all graphics
    std::map<std::string, std::shared_ptr<GraphicsNode>> m_graphicsNodes; // Flat lookup by name
    
    // Multi-object volume graphics system
    std::shared_ptr<VolumeNode> m_volumeGraphicsRoot; // Root node for all volumes
    std::map<std::string, std::shared_ptr<VolumeNode>> m_volumeGraphicsNodes; // Flat lookup by name
    
    // Multi-volume rendering state
    bool m_multiVolumeRenderingEnabled;
    vtkSmartPointer<vtkMultiVolume> m_multiVolume; // For multi-volume rendering when needed
    
    // Private helper methods for multi-volume rendering
    void setupMultiVolumeRendering();
    void teardownMultiVolumeRendering();
    void updateVolumeRendering();
};

#endif // SCENEGRAPH_H
