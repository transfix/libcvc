#include <volrover3/NullGraphicNode.h>
#include <cvc/state.h>
#include <vtkActor.h>
#include <sstream>
#include <limits>
#include <algorithm>

NullGraphicNode::NullGraphicNode(const std::string& name)
    : GraphicsNode(name)
    , m_bounds(-100.0, -100.0, -100.0, 100.0, 100.0, 100.0)  // Default 200x200x200 box
    , m_dummyActor(vtkSmartPointer<vtkActor>::New())
    , m_stateNode(nullptr)
{
    // Dummy actor has no mapper, won't render anything
    // This node exists only to provide bounding box extents
}

NullGraphicNode::~NullGraphicNode()
{
    m_dataConnection.disconnect();
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

void NullGraphicNode::syncToState(cvc::state& parentState)
{
    cvc::state& myState = parentState(m_name);
    myState.comment("Null graphics object (defines bounding box extents only)");
    
    // Store bounding box (user-modifiable unlike other graphics)
    cvc::state& boundsState = myState("bounds");
    boundsState.comment("User-defined bounding box extents");
    
    boundsState("min_x").value(m_bounds[0]);
    boundsState("min_x").comment("Minimum X coordinate");
    
    boundsState("min_y").value(m_bounds[1]);
    boundsState("min_y").comment("Minimum Y coordinate");
    
    boundsState("min_z").value(m_bounds[2]);
    boundsState("min_z").comment("Minimum Z coordinate");
    
    boundsState("max_x").value(m_bounds[3]);
    boundsState("max_x").comment("Maximum X coordinate");
    
    boundsState("max_y").value(m_bounds[4]);
    boundsState("max_y").comment("Maximum Y coordinate");
    
    boundsState("max_z").value(m_bounds[5]);
    boundsState("max_z").comment("Maximum Z coordinate");
    
    // Store bbox flag
    myState("show_bbox").value(m_showBBox ? "true" : "false");
    
    // Store label settings
    myState("show_label").value(m_showLabel ? "true" : "false");
    myState("label_text").value(m_labelText);
    myState("label_size").value(std::to_string(m_labelSize));
    std::ostringstream labelColorStr;
    labelColorStr << m_labelColor[0] << "," << m_labelColor[1] << "," << m_labelColor[2];
    myState("label_color").value(labelColorStr.str());
    
    // Sync children if we have any
    const auto& children = getGraphicsChildren();
    if (!children.empty()) {
        cvc::state& childrenState = myState("children");
        childrenState.comment("Child graphics objects");
        for (const auto& child : children) {
            child->syncToState(childrenState);
        }
    }
}

void NullGraphicNode::syncFromState(cvc::state& parentState)
{
    try {
        cvc::state& myState = parentState(m_name);
        
        m_stateNode = &myState;
        m_dataConnection.disconnect();
        m_dataConnection = myState.dataChanged.connect([this]() {
            // No data to reload for null graphic
        });
        
        // Load bounding box
        try {
            cvc::state& boundsState = myState("bounds");
            
            double minX = std::stod(boundsState("min_x").value());
            double minY = std::stod(boundsState("min_y").value());
            double minZ = std::stod(boundsState("min_z").value());
            double maxX = std::stod(boundsState("max_x").value());
            double maxY = std::stod(boundsState("max_y").value());
            double maxZ = std::stod(boundsState("max_z").value());
            
            setBounds(minX, minY, minZ, maxX, maxY, maxZ);
        } catch (...) {}
        
        // Load bbox flag
        try {
            std::string showBBoxStr = myState("show_bbox").value();
            setShowBBox(showBBoxStr == "true");
        } catch (...) {}
        
        // Load label settings
        try {
            std::string showLabelStr = myState("show_label").value();
            setShowLabel(showLabelStr == "true");
        } catch (...) {}
        
        try {
            std::string labelText = myState("label_text").value();
            setLabelText(labelText);
        } catch (...) {}
        
        try {
            int labelSize = std::stoi(myState("label_size").value());
            setLabelSize(labelSize);
        } catch (...) {}
        
        try {
            std::string colorStr = myState("label_color").value();
            std::istringstream iss(colorStr);
            double r, g, b;
            char comma;
            if (iss >> r >> comma >> g >> comma >> b) {
                setLabelColor(r, g, b);
            }
        } catch (...) {}
    } catch (...) {
        // State doesn't exist or can't be loaded
    }
}
