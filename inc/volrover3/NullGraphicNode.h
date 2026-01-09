#ifndef NULLGRAPHICNODE_H
#define NULLGRAPHICNODE_H

#include <volrover3/GraphicsNode.h>
#include <cvc/bounding_box.h>
#include <vtkSmartPointer.h>

class vtkActor;

namespace cvc {
    class state;
}

/**
 * @brief A graphics node that has no visual data, only a bounding box
 * 
 * NullGraphicNode is used as a placeholder when no graphics are loaded.
 * Unlike other graphics nodes, its bounding box extents are user-modifiable
 * rather than being computed from data.
 * 
 * Primary use case: Default graphic when scene is empty, showing only
 * a bounding box to define the coordinate system and scene extents.
 */
class NullGraphicNode : public GraphicsNode
{
public:
    NullGraphicNode(const std::string& statePath, const std::string& name = "null");
    ~NullGraphicNode() override;

    // Set custom bounding box extents (user-modifiable)
    void setBounds(const cvc::bounding_box& bbox);
    void setBounds(double minX, double minY, double minZ,
                   double maxX, double maxY, double maxZ);
    
    // Control whether this node's own bounds contribute to combined bbox
    // When false, only children's bounds are included (useful for root nodes)
    // When true, this node's bounds are included (useful for clipping regions)
    void setIncludeOwnBounds(bool include);
    bool getIncludeOwnBounds() const { return m_includeOwnBounds; }
    
    // Control whether this node's bounds automatically sync with children's combined bounds
    // When true (default), bounds expand to encompass all children
    // When false, bounds stay fixed (useful for clipping regions)
    void setSyncBoundsWithChildren(bool sync);
    bool getSyncBoundsWithChildren() const { return m_syncBoundsWithChildren; }
    
    // Manually trigger bounds sync to children
    void syncBoundsToChildren();
    
    // Override child management to auto-sync bounds
    // Bring template version from base class into scope
    using GraphicsNode::addGraphicsChild;
    void addGraphicsChild(std::shared_ptr<GraphicsNode> child) override;
    
    using GraphicsNode::removeGraphicsChild;
    void removeGraphicsChild(std::shared_ptr<GraphicsNode> child) override;
    
    // Implement GraphicsNode abstract methods
    cvc::bounding_box getBoundingBox() const override;

protected:
    vtkProp* getProp() override;
    void handleStateChanged(const std::string& childState) override;

private:
    cvc::bounding_box m_bounds;
    vtkSmartPointer<vtkActor> m_dummyActor;  // Empty actor (never rendered)
    bool m_includeOwnBounds;  // Whether to include own bounds in combined bbox
    bool m_syncBoundsWithChildren;  // Whether to auto-update bounds to match children
};

#endif // NULLGRAPHICNODE_H
