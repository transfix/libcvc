#ifndef GRAPHICSNODE_H
#define GRAPHICSNODE_H

#include <volrover3/SceneNode.h>
#include <cvc/bounding_box.h>
#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
#include <boost/signals2.hpp>
#include <string>
#include <map>
#include <any>

class vtkTransform;
class BBoxNode;

namespace cvc {
    class state;
}

/**
 * @brief Abstract base class for all graphics objects in the scene
 * 
 * GraphicsNode provides common functionality for all renderable graphics objects:
 * - Transformation (position, rotation, scale)
 * - Hierarchical structure (parent/child relationships)
 * - Metadata storage
 * - Bounding box display
 * - Visibility control
 * - State tree synchronization
 * 
 * Subclasses must implement:
 * - getBoundingBox() - return the untransformed bounding box
 * - getProp() - return the VTK prop for rendering
 * - syncToState() / syncFromState() - state tree integration
 */
class GraphicsNode : public SceneNode
{
public:
    GraphicsNode(const std::string& name = "");
    virtual ~GraphicsNode();

    // Identity and naming
    void setName(const std::string& name) { m_name = name; }
    std::string getName() const { return m_name; }
    
    // Pure virtual methods that subclasses must implement
    virtual cvc::bounding_box getBoundingBox() const = 0;  // Return untransformed bounding box
    virtual void syncToState(cvc::state& parentState) = 0;
    virtual void syncFromState(cvc::state& parentState) = 0;
    
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
    
    // Bounding box visibility
    void setShowBBox(bool show);
    bool getShowBBox() const { return m_showBBox; }
    
    // Override visibility to sync with metadata
    void setVisible(bool visible);
    
    // Override update to handle transform changes
    void update() override;
    
    // Override addToRenderer/removeFromRenderer to handle bbox
    void addToRenderer(vtkRenderer* renderer) override;
    void removeFromRenderer(vtkRenderer* renderer) override;

protected:
    void updateTransform();
    void updateBoundingBoxNode();  // Update bbox node with current bounds + transform

    // Protected members for subclass access
    std::string m_name;
    vtkSmartPointer<vtkMatrix4x4> m_transform;
    std::vector<std::shared_ptr<GraphicsNode>> m_graphicsChildren;
    GraphicsNode* m_parent; // Weak pointer to parent for world transform calculation
    std::map<std::string, std::any> m_metadata;
    bool m_showBBox;
    std::shared_ptr<BBoxNode> m_bboxNode;
};

#endif // GRAPHICSNODE_H
