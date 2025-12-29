#ifndef SCENEGRAPH_H
#define SCENEGRAPH_H

#include <memory>
#include <vector>
#include <cvc/bounding_box.h>

class vtkRenderer;
class SceneNode;
class GeometryNode;
class VolumeNode;
class GridNode;
class AxisNode;
class BBoxNode;

namespace cvc {
    class geometry;
    class volume;
}

class SceneGraph
{
public:
    SceneGraph();
    ~SceneGraph();

    void setRenderer(vtkRenderer *renderer);
    void update();

    // Scene content management
    void setGeometry(const cvc::geometry &geom);
    void setVolume(const cvc::volume &vol);
    
    // Scene element visibility
    void setGridVisible(bool visible);
    void setAxisVisible(bool visible);
    void setGeometryBBoxVisible(bool visible);
    void setVolumeBBoxVisible(bool visible);
    
    // Update grid to match bounding box
    void updateGrid(const cvc::bounding_box& bounds);

    // Transfer function update
    void updateTransferFunction(const std::vector<double> &colorTable,
                                const std::vector<double> &opacityTable);

private:
    vtkRenderer *m_renderer;
    
    std::shared_ptr<GeometryNode> m_geometryNode;
    std::shared_ptr<VolumeNode> m_volumeNode;
    std::shared_ptr<GridNode> m_gridNode;
    std::shared_ptr<AxisNode> m_axisNode;
    std::shared_ptr<BBoxNode> m_geometryBBoxNode;
    std::shared_ptr<BBoxNode> m_volumeBBoxNode;
    
    std::vector<std::shared_ptr<SceneNode>> m_rootNodes;
};

#endif // SCENEGRAPH_H
