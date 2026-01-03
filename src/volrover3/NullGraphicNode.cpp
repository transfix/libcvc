#include <volrover3/NullGraphicNode.h>
#include <cvc/state.h>
#include <cvc/app.h>
#include <vtkActor.h>
#include <sstream>
#include <limits>
#include <algorithm>

NullGraphicNode::NullGraphicNode(const std::string& statePath, const std::string& name)
    : GraphicsNode(statePath, name)
    , m_bounds(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5)  // Default 1x1x1 box centered at origin
    , m_dummyActor(vtkSmartPointer<vtkActor>::New())
    , m_includeOwnBounds(false)  // Don't include own bounds by default (typical for root nodes)
{
    // Dummy actor has no mapper, won't render anything
    // This node exists only to provide bounding box extents
    
    // Initialize bounds in state tree
    if (!statePath.empty()) {
        std::ostringstream oss;
        oss << m_bounds.minx << "," << m_bounds.miny << "," << m_bounds.minz << ","
            << m_bounds.maxx << "," << m_bounds.maxy << "," << m_bounds.maxz;
        getState("bounds").value(oss.str());
    }
}

NullGraphicNode::~NullGraphicNode()
{
}

vtkProp* NullGraphicNode::getProp()
{
    // Return dummy actor that won't render anything
    return m_dummyActor;
}

void NullGraphicNode::setBounds(const cvc::bounding_box& bbox)
{
    m_bounds = bbox;
    updateBoundingBoxNode();
}

void NullGraphicNode::setBounds(double minX, double minY, double minZ,
                                double maxX, double maxY, double maxZ)
{
    m_bounds = cvc::bounding_box(minX, minY, minZ, maxX, maxY, maxZ);
    updateBoundingBoxNode();
}

cvc::bounding_box NullGraphicNode::getBoundingBox() const
{
    // Return this node's own bounds
    // Note: If we have children, getCombinedBoundingBox() (inherited from GraphicsNode)
    // will handle merging children's transformed bboxes with our bounds
    return m_bounds;
}

void NullGraphicNode::handleStateChanged(const std::string& childState)
{
    // Handle bounds state changes (no VTK calls, so no runOnMainThread needed)
    if (childState == "bounds") {
        std::string boundsStr = getState("bounds").value<std::string>();
        std::istringstream iss(boundsStr);
        double minX, minY, minZ, maxX, maxY, maxZ;
        char comma;
        if (iss >> minX >> comma >> minY >> comma >> minZ >> comma 
                >> maxX >> comma >> maxY >> comma >> maxZ) {
            setBounds(minX, minY, minZ, maxX, maxY, maxZ);
        }
    }
    else {
        // Delegate to parent for common graphics fields
        // Parent will handle its own runOnMainThread wrapping
        GraphicsNode::handleStateChanged(childState);
    }
}
