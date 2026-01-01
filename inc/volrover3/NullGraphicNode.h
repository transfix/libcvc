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
    NullGraphicNode(const std::string& name = "null");
    ~NullGraphicNode() override;

    // Set custom bounding box extents (user-modifiable)
    void setBounds(const cvc::bounding_box& bbox);
    void setBounds(double minX, double minY, double minZ,
                   double maxX, double maxY, double maxZ);
    
    // Implement GraphicsNode abstract methods
    cvc::bounding_box getBoundingBox() const override;
    void syncToState(cvc::state& parentState) override;
    void syncFromState(cvc::state& parentState) override;

protected:
    vtkProp* getProp() override;

private:
    cvc::bounding_box m_bounds;
    vtkSmartPointer<vtkActor> m_dummyActor;  // Empty actor (never rendered)
    
    cvc::state* m_stateNode;
    boost::signals2::connection m_dataConnection;
};

#endif // NULLGRAPHICNODE_H
