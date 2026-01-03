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
    void setIncludeOwnBounds(bool include) { m_includeOwnBounds = include; }
    bool getIncludeOwnBounds() const { return m_includeOwnBounds; }
    
    // Implement GraphicsNode abstract methods
    cvc::bounding_box getBoundingBox() const override;

protected:
    vtkProp* getProp() override;
    void handleStateChanged(const std::string& childState) override;

private:
    cvc::bounding_box m_bounds;
    vtkSmartPointer<vtkActor> m_dummyActor;  // Empty actor (never rendered)
    bool m_includeOwnBounds;  // Whether to include own bounds in combined bbox
};

#endif // NULLGRAPHICNODE_H
