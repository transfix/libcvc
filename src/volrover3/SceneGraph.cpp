#include <volrover3/SceneGraph.h>
#include <volrover3/SceneNode.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/GridNode.h>
#include <volrover3/AxisNode.h>
#include <volrover3/BBoxNode.h>
#include <cvc/geometry.h>
#include <cvc/volume.h>
#include <vtkRenderer.h>

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
    
    // Set colors for bbox nodes
    m_geometryBBoxNode->setColor(0.0, 1.0, 0.0); // Green for geometry
    m_volumeBBoxNode->setColor(1.0, 0.0, 1.0);   // Magenta for volume
    
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
    m_geometryNode->setGeometry(geom);
    if (geom.num_points() > 0) {
        m_geometryBBoxNode->setBoundingBox(geom.extents());
    }
}

void SceneGraph::setVolume(const cvc::volume &vol)
{
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

void SceneGraph::updateGrid(const cvc::bounding_box& bounds)
{
    m_gridNode->setBounds(bounds);
}

void SceneGraph::updateTransferFunction(const std::vector<double> &colorTable,
                                        const std::vector<double> &opacityTable)
{
    m_volumeNode->setTransferFunction(colorTable, opacityTable);
}
