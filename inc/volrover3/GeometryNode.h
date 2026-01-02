#ifndef GEOMETRYNODE_H
#define GEOMETRYNODE_H

#include <volrover3/GraphicsNode.h>
#include <vtkSmartPointer.h>
#include <memory>

class vtkActor;
class vtkPolyDataMapper;
class vtkPolyData;

namespace cvc {
    class geometry;
    class state;
}

/**
 * @brief GeometryNode renders cvc::geometry objects with full transform support
 * 
 * Extends GraphicsNode to provide:
 * - Geometry-specific rendering (triangles, quads)
 * - Bounding box computation from geometry extents
 * - State tree synchronization for geometry data
 * 
 * Inherits from GraphicsNode:
 * - Transforms (position, rotation, scale)
 * - Metadata storage
 * - Bounding box display
 * - Hierarchical structure
 */
class GeometryNode : public GraphicsNode
{
public:
    GeometryNode(const std::string& name = "geometry");
    ~GeometryNode() override;

    void setGeometry(const cvc::geometry &geom);
    bool hasGeometry() const { return m_hasGeometry; }
    const cvc::geometry* getGeometry() const { return m_geometry.get(); }
    
    // Implement GraphicsNode abstract methods
    cvc::bounding_box getBoundingBox() const override;
    void syncToState(cvc::state& parentState) override;
    void syncFromState(cvc::state& parentState) override;

    // Check if a metadata key is computed (read-only)
    static bool isComputedMetadata(const std::string& key);

protected:
    vtkProp* getProp() override;
    void updatePolyData(const cvc::geometry &geom);
    void updateMetadata(const cvc::geometry &geom);
    void onDataChanged();

private:
    bool m_hasGeometry;
    std::shared_ptr<cvc::geometry> m_geometry;
    
    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    vtkSmartPointer<vtkPolyData> m_polyData;
    
    cvc::state* m_stateNode;
    boost::signals2::connection m_dataConnection;
};

#endif // GEOMETRYNODE_H
