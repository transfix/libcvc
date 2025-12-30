#ifndef GRAPHICSNODE_H
#define GRAPHICSNODE_H

#include <volrover3/SceneNode.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <string>
#include <map>
#include <any>

class vtkActor;
class vtkPolyDataMapper;
class vtkPolyData;
class vtkTransform;

namespace cvc {
    class geometry;
    class state;
}

/**
 * @brief GraphicsNode represents a single graphics object in the scene with transformation
 * 
 * Each GraphicsNode can contain:
 * - A geometry object to render
 * - A 4x4 transformation matrix (position, rotation, scale)
 * - Child graphics nodes with their own transformations (relative to parent)
 * - Metadata stored as key-value pairs
 * - A unique name/ID
 * 
 * The hierarchical transformation system means:
 * - Child nodes are transformed relative to their parent
 * - Transforming a parent automatically transforms all children
 * - Each node maintains its own local transform matrix
 */
class GraphicsNode : public SceneNode
{
public:
    GraphicsNode(const std::string& name = "");
    ~GraphicsNode() override;

    // Identity and naming
    void setName(const std::string& name) { m_name = name; }
    std::string getName() const { return m_name; }
    
    // Geometry management
    void setGeometry(const cvc::geometry& geom);
    bool hasGeometry() const { return m_hasGeometry; }
    const cvc::geometry* getGeometry() const { return m_geometry.get(); }
    
    // Transform management
    void setTransform(vtkMatrix4x4* matrix);
    void setTransform(const double matrix[16]); // Row-major 4x4 matrix
    vtkMatrix4x4* getTransform() { return m_transform; }
    const vtkMatrix4x4* getTransform() const { return m_transform; }
    
    // Convenience transform methods
    void setPosition(double x, double y, double z);
    void setRotation(double x, double y, double z); // Euler angles in degrees
    void setScale(double x, double y, double z);
    void resetTransform(); // Set to identity matrix
    
    // Get world transform (accumulated from all parents)
    vtkSmartPointer<vtkMatrix4x4> getWorldTransform() const;
    
    // Hierarchical structure
    void addGraphicsChild(std::shared_ptr<GraphicsNode> child);
    void removeGraphicsChild(std::shared_ptr<GraphicsNode> child);
    std::shared_ptr<GraphicsNode> findChildByName(const std::string& name);
    const std::vector<std::shared_ptr<GraphicsNode>>& getGraphicsChildren() const { return m_graphicsChildren; }
    
    // Metadata management
    void setMetadata(const std::string& key, const std::any& value);
    std::any getMetadata(const std::string& key) const;
    bool hasMetadata(const std::string& key) const;
    const std::map<std::string, std::any>& getAllMetadata() const { return m_metadata; }
    
    // Override visibility to sync with metadata
    void setVisible(bool visible);
    
    // State tree integration
    void syncToState(cvc::state& parentState);
    void syncFromState(const cvc::state& parentState);
    
    // Override update to handle transform changes
    void update() override;

protected:
    vtkProp* getProp() override;
    void updateTransform();
    void updatePolyData(const cvc::geometry& geom);

private:
    std::string m_name;
    bool m_hasGeometry;
    std::shared_ptr<cvc::geometry> m_geometry; // Store the actual geometry
    
    // VTK rendering components
    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    vtkSmartPointer<vtkPolyData> m_polyData;
    vtkSmartPointer<vtkMatrix4x4> m_transform;
    
    // Hierarchical graphics children (separate from SceneNode children)
    std::vector<std::shared_ptr<GraphicsNode>> m_graphicsChildren;
    GraphicsNode* m_parent; // Weak pointer to parent for world transform calculation
    
    // Metadata storage
    std::map<std::string, std::any> m_metadata;
};

#endif // GRAPHICSNODE_H
