#include <volrover3/GraphicsNode.h>
#include <cvc/geometry.h>
#include <cvc/state.h>
#include <cvc/app.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkProperty.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkTransform.h>
#include <vtkMatrix4x4.h>
#include <algorithm>

GraphicsNode::GraphicsNode(const std::string& name)
    : SceneNode()
    , m_name(name)
    , m_hasGeometry(false)
    , m_actor(vtkSmartPointer<vtkActor>::New())
    , m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_polyData(vtkSmartPointer<vtkPolyData>::New())
    , m_transform(vtkSmartPointer<vtkMatrix4x4>::New())
    , m_parent(nullptr)
{
    m_mapper->SetInputData(m_polyData);
    m_actor->SetMapper(m_mapper);
    
    // Set default material properties
    m_actor->GetProperty()->SetColor(0.8, 0.8, 0.9);
    m_actor->GetProperty()->SetSpecular(0.3);
    m_actor->GetProperty()->SetSpecularPower(20);
    
    // Initialize transform to identity
    m_transform->Identity();
    
    // Set default visible metadata flag to true
    setMetadata("visible", true);
}

GraphicsNode::~GraphicsNode()
{
}

vtkProp* GraphicsNode::getProp()
{
    return m_actor;
}

void GraphicsNode::setGeometry(const cvc::geometry& geom)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    updatePolyData(geom);
    m_hasGeometry = true;
}

void GraphicsNode::updatePolyData(const cvc::geometry& geom)
{
    // Create VTK points from geometry
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(geom.num_points());

    for (size_t i = 0; i < geom.num_points(); ++i) {
        const auto& pt = geom.points()[i];
        points->SetPoint(i, pt[0], pt[1], pt[2]);
    }

    // Create VTK cells (triangles)
    vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
    
    for (size_t i = 0; i < geom.num_tris(); ++i) {
        const auto& tri = geom.tris()[i];
        triangles->InsertNextCell(3);
        triangles->InsertCellPoint(tri[0]);
        triangles->InsertCellPoint(tri[1]);
        triangles->InsertCellPoint(tri[2]);
    }

    // Update polydata
    m_polyData->SetPoints(points);
    m_polyData->SetPolys(triangles);

    // Add normals if available
    if (geom.normals().size() == geom.num_points()) {
        vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
        normals->SetNumberOfComponents(3);
        normals->SetNumberOfTuples(geom.num_points());
        normals->SetName("Normals");

        for (size_t i = 0; i < geom.num_points(); ++i) {
            const auto& n = geom.normals()[i];
            normals->SetTuple3(i, n[0], n[1], n[2]);
        }

        m_polyData->GetPointData()->SetNormals(normals);
    } else {
        m_polyData->GetPointData()->SetNormals(nullptr);
    }

    // Add colors if available
    if (geom.colors().size() == geom.num_points()) {
        vtkSmartPointer<vtkFloatArray> colors = vtkSmartPointer<vtkFloatArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetNumberOfTuples(geom.num_points());
        colors->SetName("Colors");

        for (size_t i = 0; i < geom.num_points(); ++i) {
            const auto& c = geom.colors()[i];
            colors->SetTuple3(i, c[0], c[1], c[2]);
        }

        m_polyData->GetPointData()->SetScalars(colors);
    }

    m_polyData->Modified();
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
    // Get current rotation and translation
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
        len = sqrt(len);
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
    // Get world transform and apply to actor
    vtkSmartPointer<vtkMatrix4x4> worldTransform = getWorldTransform();
    m_actor->SetUserMatrix(worldTransform);
    
    // Update all children
    for (auto& child : m_graphicsChildren) {
        child->updateTransform();
    }
}

void GraphicsNode::update()
{
    SceneNode::update();
    updateTransform();
    
    // Sync visibility from metadata
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

void GraphicsNode::syncToState(cvc::state& parentState)
{
    cvc::state& myState = parentState(m_name);
    
    // Store visibility from metadata
    if (hasMetadata("visible")) {
        try {
            bool visible = std::any_cast<bool>(getMetadata("visible"));
            myState("visible").value(visible ? "true" : "false");
        } catch (...) {
            myState("visible").value("true");
        }
    }
    
    // Store transform matrix (row-major)
    std::string matrixStr;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!matrixStr.empty()) matrixStr += ",";
            matrixStr += std::to_string(m_transform->GetElement(i, j));
        }
    }
    myState("transform").value(matrixStr);
    
    // Store metadata
    for (const auto& [key, value] : m_metadata) {
        // Store metadata in a sub-state
        try {
            if (value.type() == typeid(std::string)) {
                myState("metadata")(key).value(std::any_cast<std::string>(value));
            } else if (value.type() == typeid(double)) {
                myState("metadata")(key).value(std::any_cast<double>(value));
            } else if (value.type() == typeid(int)) {
                myState("metadata")(key).value(std::any_cast<int>(value));
            } else if (value.type() == typeid(bool)) {
                bool boolVal = std::any_cast<bool>(value);
                myState("metadata")(key).value(boolVal ? "true" : "false");
            }
            // Add more types as needed
        } catch (...) {
            // Skip metadata that can't be serialized
        }
    }
    
    // Recursively sync children
    for (auto& child : m_graphicsChildren) {
        child->syncToState(myState("children"));
    }
}

void GraphicsNode::syncFromState(const cvc::state& parentState)
{
    cvc::state& myState = const_cast<cvc::state&>(parentState)(m_name);
    
    if (!myState.initialized()) return;
    
    // Load visibility
    if (myState("visible").initialized()) {
        setVisible(myState("visible").value() == "true");
    }
    
    // Load transform matrix
    if (myState("transform").initialized()) {
        std::string matrixStr = myState("transform").value();
        // Parse comma-separated values
        double matrix[16];
        size_t pos = 0;
        for (int i = 0; i < 16; ++i) {
            size_t nextPos = matrixStr.find(',', pos);
            std::string val = (nextPos == std::string::npos) ? 
                matrixStr.substr(pos) : matrixStr.substr(pos, nextPos - pos);
            matrix[i] = std::stod(val);
            pos = nextPos + 1;
        }
        setTransform(matrix);
    }
    
    // Load metadata
    cvc::state& metadataState = myState("metadata");
    if (metadataState.initialized()) {
        auto metadataChildren = metadataState.children();
        for (const auto& key : metadataChildren) {
            cvc::state& metaState = metadataState(key);
            if (metaState.initialized()) {
                // Try to determine type and store
                std::string valueStr = metaState.value();
                
                // Special handling for bool values
                if (valueStr == "true" || valueStr == "false") {
                    setMetadata(key, valueStr == "true");
                } else {
                    setMetadata(key, valueStr); // Store as string by default
                }
            }
        }
    }
}
