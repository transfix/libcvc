#include <volrover3/GraphicsNode.h>
#include <volrover3/BBoxNode.h>
#include <cvc/state.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <vtkRenderer.h>
#include <vtkActor2D.h>
#include <vtkTextMapper.h>
#include <vtkTextProperty.h>
#include <cmath>
#include <algorithm>

GraphicsNode::GraphicsNode(const std::string& name)
    : SceneNode()
    , m_name(name)
    , m_transform(vtkSmartPointer<vtkMatrix4x4>::New())
    , m_parent(nullptr)
    , m_showBBox(false)
    , m_bboxNode(std::make_shared<BBoxNode>())
    , m_showLabel(false)
    , m_labelText(name)
    , m_labelSize(14)
    , m_labelActor(vtkSmartPointer<vtkActor2D>::New())
{
    // Initialize transform to identity
    m_transform->Identity();
    
    // Initialize label color to white
    m_labelColor[0] = m_labelColor[1] = m_labelColor[2] = 1.0;
    
    // Setup label actor
    vtkSmartPointer<vtkTextMapper> textMapper = vtkSmartPointer<vtkTextMapper>::New();
    textMapper->SetInput(m_labelText.c_str());
    textMapper->GetTextProperty()->SetFontSize(m_labelSize);
    textMapper->GetTextProperty()->SetColor(m_labelColor);
    textMapper->GetTextProperty()->SetJustificationToCentered();
    textMapper->GetTextProperty()->SetVerticalJustificationToCentered();
    m_labelActor->SetMapper(textMapper);
    m_labelActor->GetPositionCoordinate()->SetCoordinateSystemToWorld();
    m_labelActor->SetVisibility(m_showLabel);
    
    // Set default visible metadata flag to true
    setMetadata("visible", true);
}

GraphicsNode::~GraphicsNode()
{
}

void GraphicsNode::setTransform(vtkMatrix4x4* matrix)
{
    if (matrix) {
        m_transform->DeepCopy(matrix);
        updateTransform();
    }
}

void GraphicsNode::setTransform(const double matrix[16])
{
    // Input is row-major, VTK uses row-major storage
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            m_transform->SetElement(i, j, matrix[i * 4 + j]);
        }
    }
    updateTransform();
}

void GraphicsNode::setPosition(double x, double y, double z)
{
    m_transform->SetElement(0, 3, x);
    m_transform->SetElement(1, 3, y);
    m_transform->SetElement(2, 3, z);
    updateTransform();
}

void GraphicsNode::setRotation(double x, double y, double z)
{
    // Create transform with rotation
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Identity();
    transform->RotateZ(z);
    transform->RotateY(y);
    transform->RotateX(x);
    
    // Preserve current translation
    double tx = m_transform->GetElement(0, 3);
    double ty = m_transform->GetElement(1, 3);
    double tz = m_transform->GetElement(2, 3);
    
    m_transform->DeepCopy(transform->GetMatrix());
    m_transform->SetElement(0, 3, tx);
    m_transform->SetElement(1, 3, ty);
    m_transform->SetElement(2, 3, tz);
    
    updateTransform();
}

void GraphicsNode::setScale(double x, double y, double z)
{
    // Get current translation
    double tx = m_transform->GetElement(0, 3);
    double ty = m_transform->GetElement(1, 3);
    double tz = m_transform->GetElement(2, 3);
    
    // Extract rotation part (normalize the 3x3 upper-left)
    vtkSmartPointer<vtkMatrix4x4> rotation = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int i = 0; i < 3; ++i) {
        double len = 0.0;
        for (int j = 0; j < 3; ++j) {
            double val = m_transform->GetElement(i, j);
            len += val * val;
        }
        len = std::sqrt(len);
        if (len > 0.0) {
            for (int j = 0; j < 3; ++j) {
                rotation->SetElement(i, j, m_transform->GetElement(i, j) / len);
            }
        }
    }
    
    // Apply new scale to rotation
    for (int i = 0; i < 3; ++i) {
        double scale = (i == 0) ? x : (i == 1) ? y : z;
        for (int j = 0; j < 3; ++j) {
            m_transform->SetElement(i, j, rotation->GetElement(i, j) * scale);
        }
    }
    
    // Restore translation
    m_transform->SetElement(0, 3, tx);
    m_transform->SetElement(1, 3, ty);
    m_transform->SetElement(2, 3, tz);
    
    updateTransform();
}

void GraphicsNode::resetTransform()
{
    m_transform->Identity();
    updateTransform();
}

vtkSmartPointer<vtkMatrix4x4> GraphicsNode::getWorldTransform() const
{
    vtkSmartPointer<vtkMatrix4x4> worldTransform = vtkSmartPointer<vtkMatrix4x4>::New();
    
    if (m_parent) {
        // Get parent's world transform
        vtkSmartPointer<vtkMatrix4x4> parentWorld = m_parent->getWorldTransform();
        // Multiply: worldTransform = parentWorld * m_transform
        vtkMatrix4x4::Multiply4x4(parentWorld, m_transform, worldTransform);
    } else {
        // No parent, local transform is world transform
        worldTransform->DeepCopy(m_transform);
    }
    
    return worldTransform;
}

void GraphicsNode::updateTransform()
{
    // Update all children
    for (auto& child : m_graphicsChildren) {
        child->updateTransform();
    }
    
    // Update bbox if visible
    if (m_showBBox) {
        updateBoundingBoxNode();
    }
}

void GraphicsNode::updateBoundingBoxNode()
{
    if (!m_bboxNode) return;
    
    // Get COMBINED bounding box (this node + all children) for visualization
    // This ensures the bbox shows the full extent including children
    cvc::bounding_box bbox = getCombinedBoundingBox();
    
    // Apply world transform to bounding box
    vtkSmartPointer<vtkMatrix4x4> worldTransform = getWorldTransform();
    
    // Transform all 8 corners of the bounding box
    double corners[8][3] = {
        {bbox.minx, bbox.miny, bbox.minz},
        {bbox.maxx, bbox.miny, bbox.minz},
        {bbox.minx, bbox.maxy, bbox.minz},
        {bbox.maxx, bbox.maxy, bbox.minz},
        {bbox.minx, bbox.miny, bbox.maxz},
        {bbox.maxx, bbox.miny, bbox.maxz},
        {bbox.minx, bbox.maxy, bbox.maxz},
        {bbox.maxx, bbox.maxy, bbox.maxz}
    };
    
    double minx = std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double minz = std::numeric_limits<double>::max();
    double maxx = std::numeric_limits<double>::lowest();
    double maxy = std::numeric_limits<double>::lowest();
    double maxz = std::numeric_limits<double>::lowest();
    
    for (int i = 0; i < 8; ++i) {
        double in[4] = {corners[i][0], corners[i][1], corners[i][2], 1.0};
        double out[4];
        worldTransform->MultiplyPoint(in, out);
        
        minx = std::min(minx, out[0]);
        miny = std::min(miny, out[1]);
        minz = std::min(minz, out[2]);
        maxx = std::max(maxx, out[0]);
        maxy = std::max(maxy, out[1]);
        maxz = std::max(maxz, out[2]);
    }
    
    m_bboxNode->setBoundingBox(cvc::bounding_box(minx, miny, minz, maxx, maxy, maxz));
}

void GraphicsNode::update()
{
    SceneNode::update();
    
    // Update label position if visible
    if (m_showLabel && isVisible()) {
        updateLabel();
    }
    
    // Read visible flag from metadata and apply
    if (hasMetadata("visible")) {
        try {
            bool visible = std::any_cast<bool>(getMetadata("visible"));
            SceneNode::setVisible(visible);
        } catch (...) {
            // If cast fails, default to visible
            SceneNode::setVisible(true);
        }
    }
}

void GraphicsNode::addGraphicsChild(std::shared_ptr<GraphicsNode> child)
{
    if (!child) return;
    
    // Add to graphics children list
    m_graphicsChildren.push_back(child);
    
    // Set parent pointer
    child->m_parent = this;
    
    // Also add as SceneNode child so it gets rendered
    addChild(child);
    
    // Update child's transform to reflect new parent
    child->updateTransform();
    
    // Update this node's bounding box to include the new child
    if (m_showBBox) {
        updateBoundingBoxNode();
    }
}

void GraphicsNode::removeGraphicsChild(std::shared_ptr<GraphicsNode> child)
{
    if (!child) return;
    
    // Remove from graphics children
    auto it = std::find(m_graphicsChildren.begin(), m_graphicsChildren.end(), child);
    if (it != m_graphicsChildren.end()) {
        m_graphicsChildren.erase(it);
        child->m_parent = nullptr;
        child->updateTransform();
    }
    
    // Also remove as SceneNode child
    removeChild(child);
    
    // Update this node's bounding box after removing child
    if (m_showBBox) {
        updateBoundingBoxNode();
    }
}

std::shared_ptr<GraphicsNode> GraphicsNode::findChildByName(const std::string& name)
{
    for (auto& child : m_graphicsChildren) {
        if (child->getName() == name) {
            return child;
        }
        // Recursively search in child's children
        auto found = child->findChildByName(name);
        if (found) {
            return found;
        }
    }
    return nullptr;
}

cvc::bounding_box GraphicsNode::getCombinedBoundingBox() const
{
    // Start with this node's own bounding box
    cvc::bounding_box combined = getBoundingBox();
    
    // Expand to include all children (transformed to this node's local space)
    for (const auto& child : m_graphicsChildren) {
        if (!child) continue;
        
        // Get child's combined bbox (includes child's descendants in child's local space)
        cvc::bounding_box childBBox = child->getCombinedBoundingBox();
        
        // Skip invalid bounding boxes
        if (childBBox[0] > childBBox[3] || 
            childBBox[1] > childBBox[4] || 
            childBBox[2] > childBBox[5]) {
            continue;
        }
        
        // Transform child's bbox by child's local transform to get it in this node's space
        vtkMatrix4x4* childTransform = child->getTransform();
        
        // Transform all 8 corners of child's bbox
        double corners[8][3] = {
            {childBBox[0], childBBox[1], childBBox[2]},  // min, min, min
            {childBBox[3], childBBox[1], childBBox[2]},  // max, min, min
            {childBBox[0], childBBox[4], childBBox[2]},  // min, max, min
            {childBBox[3], childBBox[4], childBBox[2]},  // max, max, min
            {childBBox[0], childBBox[1], childBBox[5]},  // min, min, max
            {childBBox[3], childBBox[1], childBBox[5]},  // max, min, max
            {childBBox[0], childBBox[4], childBBox[5]},  // min, max, max
            {childBBox[3], childBBox[4], childBBox[5]}   // max, max, max
        };
        
        double minx = std::numeric_limits<double>::max();
        double miny = std::numeric_limits<double>::max();
        double minz = std::numeric_limits<double>::max();
        double maxx = std::numeric_limits<double>::lowest();
        double maxy = std::numeric_limits<double>::lowest();
        double maxz = std::numeric_limits<double>::lowest();
        
        for (int i = 0; i < 8; ++i) {
            double in[4] = {corners[i][0], corners[i][1], corners[i][2], 1.0};
            double out[4];
            childTransform->MultiplyPoint(in, out);
            
            minx = std::min(minx, out[0]);
            miny = std::min(miny, out[1]);
            minz = std::min(minz, out[2]);
            maxx = std::max(maxx, out[0]);
            maxy = std::max(maxy, out[1]);
            maxz = std::max(maxz, out[2]);
        }
        
        // Expand combined box to include transformed child
        combined[0] = std::min(combined[0], minx);
        combined[1] = std::min(combined[1], miny);
        combined[2] = std::min(combined[2], minz);
        combined[3] = std::max(combined[3], maxx);
        combined[4] = std::max(combined[4], maxy);
        combined[5] = std::max(combined[5], maxz);
    }
    
    return combined;
}

void GraphicsNode::saveCommonStateAttributes(cvc::state& myState)
{
    // Store transform as comma-separated string (row-major 4x4 matrix)
    std::string transformStr;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!transformStr.empty()) transformStr += ",";
            transformStr += std::to_string(m_transform->GetElement(i, j));
        }
    }
    cvc::state& transformState = myState("transform");
    transformState.value(transformStr);
    transformState.comment("4x4 transformation matrix in row-major order");
    
    // Store bbox flag
    myState("show_bbox").value(m_showBBox ? "true" : "false");
    
    // Store label settings
    myState("show_label").value(m_showLabel ? "true" : "false");
    myState("label_text").value(m_labelText);
    myState("label_size").value(std::to_string(m_labelSize));
    std::ostringstream labelColorStr;
    labelColorStr << m_labelColor[0] << "," << m_labelColor[1] << "," << m_labelColor[2];
    myState("label_color").value(labelColorStr.str());
    
    // If this node has children, add combined bounding box to metadata
    if (!m_graphicsChildren.empty()) {
        cvc::bounding_box combinedBBox = getCombinedBoundingBox();
        
        // Store combined bbox min coordinates
        m_metadata["combined_bbox_min_x"] = combinedBBox[0];
        m_metadata["combined_bbox_min_y"] = combinedBBox[1];
        m_metadata["combined_bbox_min_z"] = combinedBBox[2];
        
        // Store combined bbox max coordinates
        m_metadata["combined_bbox_max_x"] = combinedBBox[3];
        m_metadata["combined_bbox_max_y"] = combinedBBox[4];
        m_metadata["combined_bbox_max_z"] = combinedBBox[5];
        
        // Store combined extents
        m_metadata["combined_extent_x"] = combinedBBox[3] - combinedBBox[0];
        m_metadata["combined_extent_y"] = combinedBBox[4] - combinedBBox[1];
        m_metadata["combined_extent_z"] = combinedBBox[5] - combinedBBox[2];
        
        // Store combined center
        m_metadata["combined_center_x"] = (combinedBBox[0] + combinedBBox[3]) / 2.0;
        m_metadata["combined_center_y"] = (combinedBBox[1] + combinedBBox[4]) / 2.0;
        m_metadata["combined_center_z"] = (combinedBBox[2] + combinedBBox[5]) / 2.0;
        
        // Recursively sync children under a "children" container
        cvc::state& childrenState = myState("children");
        childrenState.comment("Child graphics objects");
        for (const auto& child : m_graphicsChildren) {
            child->syncToState(childrenState);
        }
    }
}

void GraphicsNode::setMetadata(const std::string& key, const std::any& value)
{
    m_metadata[key] = value;
}

std::any GraphicsNode::getMetadata(const std::string& key) const
{
    auto it = m_metadata.find(key);
    if (it != m_metadata.end()) {
        return it->second;
    }
    return std::any();
}

bool GraphicsNode::hasMetadata(const std::string& key) const
{
    return m_metadata.find(key) != m_metadata.end();
}

void GraphicsNode::setVisible(bool visible)
{
    SceneNode::setVisible(visible);
    setMetadata("visible", visible);
    
    // Update label visibility
    if (m_labelActor) {
        m_labelActor->SetVisibility(m_showLabel && visible);
    }
}

void GraphicsNode::setShowBBox(bool show)
{
    if (m_showBBox == show)
        return;
    
    m_showBBox = show;
    
    if (m_bboxNode && m_renderer) {
        if (show) {
            updateBoundingBoxNode();
            m_bboxNode->addToRenderer(m_renderer);
        } else {
            m_bboxNode->removeFromRenderer(m_renderer);
        }
    }
}

void GraphicsNode::setBBoxColor(double r, double g, double b)
{
    if (m_bboxNode) {
        m_bboxNode->setColor(r, g, b);
    }
}

void GraphicsNode::getBBoxColor(double& r, double& g, double& b) const
{
    if (m_bboxNode) {
        m_bboxNode->getColor(r, g, b);
    } else {
        r = g = b = 1.0;
    }
}

void GraphicsNode::setShowLabel(bool show)
{
    if (m_showLabel == show)
        return;
        
    m_showLabel = show;
    m_labelActor->SetVisibility(m_showLabel && isVisible());
    
    // Add or remove from renderer if needed
    if (m_renderer) {
        if (m_showLabel && isVisible()) {
            updateLabel();
            m_renderer->AddActor2D(m_labelActor);
        } else {
            m_renderer->RemoveActor2D(m_labelActor);
        }
    }
}

void GraphicsNode::setLabelText(const std::string& text)
{
    m_labelText = text;
    vtkTextMapper* mapper = vtkTextMapper::SafeDownCast(m_labelActor->GetMapper());
    if (mapper) {
        mapper->SetInput(m_labelText.c_str());
    }
}

void GraphicsNode::setLabelSize(int size)
{
    m_labelSize = std::max(1, size);
    vtkTextMapper* mapper = vtkTextMapper::SafeDownCast(m_labelActor->GetMapper());
    if (mapper) {
        mapper->GetTextProperty()->SetFontSize(m_labelSize);
    }
}

void GraphicsNode::setLabelColor(double r, double g, double b)
{
    m_labelColor[0] = r;
    m_labelColor[1] = g;
    m_labelColor[2] = b;
    vtkTextMapper* mapper = vtkTextMapper::SafeDownCast(m_labelActor->GetMapper());
    if (mapper) {
        mapper->GetTextProperty()->SetColor(r, g, b);
    }
}

void GraphicsNode::getLabelColor(double& r, double& g, double& b) const
{
    r = m_labelColor[0];
    g = m_labelColor[1];
    b = m_labelColor[2];
}

void GraphicsNode::updateLabel()
{
    // Position label at center of bounding box
    cvc::bounding_box bbox = getBoundingBox();
    double centerX = (bbox[0] + bbox[3]) / 2.0;
    double centerY = (bbox[1] + bbox[4]) / 2.0;
    double centerZ = (bbox[2] + bbox[5]) / 2.0;
    
    m_labelActor->GetPositionCoordinate()->SetValue(centerX, centerY, centerZ);
}

void GraphicsNode::addToRenderer(vtkRenderer* renderer)
{
    // Call base implementation to add the main prop
    SceneNode::addToRenderer(renderer);
    
    // Add bbox if it should be visible
    if (m_showBBox && m_bboxNode) {
        updateBoundingBoxNode();
        m_bboxNode->addToRenderer(renderer);
    }
    
    // Add label if it should be visible
    if (m_showLabel && m_labelActor) {
        updateLabel();
        renderer->AddActor2D(m_labelActor);
    }
}

void GraphicsNode::removeFromRenderer(vtkRenderer* renderer)
{
    // Remove label
    if (m_labelActor) {
        renderer->RemoveActor2D(m_labelActor);
    }
    
    // Remove bbox
    if (m_bboxNode) {
        m_bboxNode->removeFromRenderer(renderer);
    }
    
    // Call base implementation to remove the main prop
    SceneNode::removeFromRenderer(renderer);
}
