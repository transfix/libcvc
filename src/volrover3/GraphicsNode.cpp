#include <volrover3/GraphicsNode.h>
#include <volrover3/BBoxNode.h>
#include <cvc/state.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <cmath>
#include <algorithm>

GraphicsNode::GraphicsNode(const std::string& name)
    : SceneNode()
    , m_name(name)
    , m_transform(vtkSmartPointer<vtkMatrix4x4>::New())
    , m_parent(nullptr)
    , m_showBBox(false)
    , m_bboxNode(std::make_shared<BBoxNode>())
{
    // Initialize transform to identity
    m_transform->Identity();
    
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
    
    // Get untransformed bounding box from subclass
    cvc::bounding_box bbox = getBoundingBox();
    
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

void GraphicsNode::addToRenderer(vtkRenderer* renderer)
{
    // Call base implementation to add the main prop
    SceneNode::addToRenderer(renderer);
    
    // Add bbox if it should be visible
    if (m_showBBox && m_bboxNode) {
        updateBoundingBoxNode();
        m_bboxNode->addToRenderer(renderer);
    }
}

void GraphicsNode::removeFromRenderer(vtkRenderer* renderer)
{
    // Remove bbox
    if (m_bboxNode) {
        m_bboxNode->removeFromRenderer(renderer);
    }
    
    // Call base implementation to remove the main prop
    SceneNode::removeFromRenderer(renderer);
}
