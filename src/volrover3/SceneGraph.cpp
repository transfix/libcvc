#include <volrover3/SceneGraph.h>
#include <volrover3/SceneNode.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/GridNode.h>
#include <volrover3/AxisNode.h>
#include <volrover3/BBoxNode.h>
#include <volrover3/AppState.h>
#include <cvc/geometry.h>
#include <cvc/volume.h>
#include <cvc/app.h>
#include <vtkRenderer.h>
#include <algorithm>

SceneGraph::SceneGraph()
    : m_renderer(nullptr)
    , m_geometryNode(std::make_shared<GeometryNode>())
    , m_volumeNode(std::make_shared<VolumeNode>())
    , m_gridNode(std::make_shared<GridNode>())
    , m_axisNode(std::make_shared<AxisNode>())
    , m_geometryBBoxNode(std::make_shared<BBoxNode>())
    , m_volumeBBoxNode(std::make_shared<BBoxNode>())
{
    m_rootNodes.push_back(m_geometryNode);
    m_rootNodes.push_back(m_volumeNode);
    m_rootNodes.push_back(m_gridNode);
    m_rootNodes.push_back(m_axisNode);
    m_rootNodes.push_back(m_geometryBBoxNode);
    m_rootNodes.push_back(m_volumeBBoxNode);
    
    // Set colors for bbox nodes from AppState
    double r, g, b;
    AppState::instance().getGridColor(r, g, b);
    m_gridNode->setColor(r, g, b);
    
    AppState::instance().getGeometryBBoxColor(r, g, b);
    m_geometryBBoxNode->setColor(r, g, b);
    
    AppState::instance().getVolumeBBoxColor(r, g, b);
    m_volumeBBoxNode->setColor(r, g, b);
    
    // Start with bboxes hidden
    m_geometryBBoxNode->setVisible(false);
    m_volumeBBoxNode->setVisible(false);
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
    if (geom.num_points() > 0) {
        m_geometryBBoxNode->setBoundingBox(geom.extents());
    }
}

void SceneGraph::setVolume(const cvc::volume &vol)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    m_volumeNode->setVolume(vol);
    m_volumeBBoxNode->setBoundingBox(vol.boundingBox());
}

void SceneGraph::setGridVisible(bool visible)
{
    m_gridNode->setVisible(visible);
}

void SceneGraph::setAxisVisible(bool visible)
{
    m_axisNode->setVisible(visible);
}

void SceneGraph::setGeometryBBoxVisible(bool visible)
{
    m_geometryBBoxNode->setVisible(visible);
}

void SceneGraph::setVolumeBBoxVisible(bool visible)
{
    m_volumeBBoxNode->setVisible(visible);
}

void SceneGraph::setGridColor(double r, double g, double b)
{
    m_gridNode->setColor(r, g, b);
}

void SceneGraph::setGeometryBBoxColor(double r, double g, double b)
{
    m_geometryBBoxNode->setColor(r, g, b);
}

void SceneGraph::setVolumeBBoxColor(double r, double g, double b)
{
    m_volumeBBoxNode->setColor(r, g, b);
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

void SceneGraph::setVolumeBBoxTicks(bool visible, double interval, double r, double g, double b, int fontSize)
{
    m_volumeBBoxNode->setTicksVisible(visible);
    m_volumeBBoxNode->setTickInterval(interval);
    m_volumeBBoxNode->setTickLabelColor(r, g, b);
    m_volumeBBoxNode->setTickLabelFontSize(fontSize);
}

void SceneGraph::updateTransferFunction(const std::vector<double> &colorTable,
                                        const std::vector<double> &opacityTable)
{
    m_volumeNode->setTransferFunction(colorTable, opacityTable);
}
