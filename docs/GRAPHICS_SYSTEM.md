# Multi-Object Graphics System

## Overview

The SceneGraph now supports loading and managing multiple geometry objects simultaneously in a hierarchical structure with transformations. This system allows for:

- Multiple independent geometry objects in the scene
- Hierarchical parent-child relationships with relative transformations
- Per-object transformation matrices (position, rotation, scale)
- Metadata storage for each graphics object
- State tree integration for persistence

## Architecture

### GraphicsNode

`GraphicsNode` is the core class that represents a single graphics object with:

- **Geometry Data**: Optional 3D mesh (points, triangles, normals, colors)
- **Transform Matrix**: 4x4 matrix for position, rotation, and scale
- **Hierarchy**: Parent-child relationships where child transforms are relative to parent
- **Metadata**: Key-value storage for arbitrary data (filename, tags, etc.)
- **State Integration**: Automatic synchronization with the state tree

### SceneGraph Integration

The `SceneGraph` manages all graphics objects through:

- **Graphics Root**: A root `GraphicsNode` that contains all top-level graphics
- **Flat Lookup**: `std::map<std::string, std::shared_ptr<GraphicsNode>>` for fast access by name
- **State Sync**: Automatic synchronization with `volrover3.graphics` state node

## Usage Examples

### Adding a Single Geometry Object

```cpp
// Load geometry from file
cvc::geometry geom = cvc::read_geometry("bunny.obj");

// Add to scene with a unique name
auto graphicsNode = sceneGraph->addGraphics("bunny1", geom);

// Optional: Set position
graphicsNode->setPosition(0.0, 0.0, 5.0);

// Optional: Set rotation (Euler angles in degrees)
graphicsNode->setRotation(45.0, 0.0, 0.0);

// Optional: Set scale
graphicsNode->setScale(2.0, 2.0, 2.0);
```

### Adding Multiple Geometry Objects

```cpp
// Load multiple geometries
cvc::geometry bunny = cvc::read_geometry("bunny.obj");
cvc::geometry dragon = cvc::read_geometry("dragon.obj");
cvc::geometry teapot = cvc::read_geometry("teapot.obj");

// Add them at different positions
auto bunny1 = sceneGraph->addGraphics("bunny1", bunny);
bunny1->setPosition(-5.0, 0.0, 0.0);

auto dragon1 = sceneGraph->addGraphics("dragon1", dragon);
dragon1->setPosition(0.0, 0.0, 0.0);
dragon1->setScale(0.5, 0.5, 0.5);

auto teapot1 = sceneGraph->addGraphics("teapot1", teapot);
teapot1->setPosition(5.0, 0.0, 0.0);
teapot1->setRotation(0.0, 45.0, 0.0);
```

### Creating Hierarchical Structures

```cpp
// Create a parent container node (no geometry)
auto parent = sceneGraph->addGraphics("robot");
parent->setPosition(0.0, 0.0, 0.0);

// Load body parts
cvc::geometry body = cvc::read_geometry("robot_body.obj");
cvc::geometry arm = cvc::read_geometry("robot_arm.obj");

// Create body as child of parent
auto bodyNode = std::make_shared<GraphicsNode>("body");
bodyNode->setGeometry(body);
bodyNode->setPosition(0.0, 0.0, 0.0); // Relative to parent
parent->addGraphicsChild(bodyNode);

// Create arm as child of body
auto armNode = std::make_shared<GraphicsNode>("left_arm");
armNode->setGeometry(arm);
armNode->setPosition(-1.0, 0.5, 0.0); // Relative to body
armNode->setRotation(0.0, 0.0, 30.0); // Relative to body
bodyNode->addGraphicsChild(armNode);

// Now transforming 'parent' will transform both body and arm
// Transforming 'body' will transform arm relative to body's new position
parent->setRotation(0.0, 90.0, 0.0); // Rotates entire robot
```

### Metadata Management

```cpp
auto graphicsNode = sceneGraph->addGraphics("bunny1", bunny);

// Store metadata
graphicsNode->setMetadata("filename", std::string("bunny.obj"));
graphicsNode->setMetadata("load_time", std::string("2025-12-30"));
graphicsNode->setMetadata("num_vertices", bunny.num_points());

// Retrieve metadata
if (graphicsNode->hasMetadata("filename")) {
    auto filename = std::any_cast<std::string>(
        graphicsNode->getMetadata("filename")
    );
}
```

### Removing Graphics Objects

```cpp
// Remove by name
sceneGraph->removeGraphics("bunny1");

// Or get reference first, then remove
auto node = sceneGraph->getGraphics("dragon1");
if (node) {
    sceneGraph->removeGraphics("dragon1");
}
```

### Accessing and Manipulating Graphics

```cpp
// Get graphics node by name
auto node = sceneGraph->getGraphics("bunny1");
if (node) {
    // Change visibility
    node->setVisible(false);
    
    // Modify transform
    node->setPosition(1.0, 2.0, 3.0);
    node->setRotation(45.0, 30.0, 60.0);
    
    // Get world transform (includes all parent transforms)
    vtkSmartPointer<vtkMatrix4x4> worldTransform = node->getWorldTransform();
}

// Iterate over all graphics
for (const auto& [name, node] : sceneGraph->getAllGraphics()) {
    std::cout << "Graphics: " << name << " visible: " << node->isVisible() << std::endl;
}
```

### State Tree Integration

Graphics objects are automatically synchronized with the state tree under `volrover3.graphics`:

```cpp
// Save all graphics to state tree
sceneGraph->syncGraphicsToState();

// Later, load graphics from state tree
sceneGraph->syncGraphicsFromState();
```

The state tree stores:
- Transform matrices
- Visibility flags
- Metadata
- Hierarchical structure

## Transformation Details

### Transform Matrix

Each `GraphicsNode` maintains a 4x4 homogeneous transformation matrix:

```
[ R11  R12  R13  Tx ]
[ R21  R22  R23  Ty ]
[ R31  R32  R33  Tz ]
[  0    0    0    1 ]
```

Where:
- R11-R33: Rotation and scale
- Tx, Ty, Tz: Translation

### World Transform

Child nodes inherit their parent's transformation. The world transform is calculated by multiplying the local transform with the parent's world transform:

```
WorldTransform(child) = WorldTransform(parent) × LocalTransform(child)
```

### Convenience Methods

```cpp
// Identity matrix (reset all transformations)
graphicsNode->resetTransform();

// Set position only
graphicsNode->setPosition(x, y, z);

// Set rotation only (Euler angles XYZ, degrees)
graphicsNode->setRotation(rx, ry, rz);

// Set scale only
graphicsNode->setScale(sx, sy, sz);

// Set full 4x4 matrix (row-major)
double matrix[16] = {...};
graphicsNode->setTransform(matrix);

// Or use VTK matrix
vtkSmartPointer<vtkMatrix4x4> vtk_matrix = ...;
graphicsNode->setTransform(vtk_matrix);
```

## API Reference

### SceneGraph Methods

```cpp
// Add graphics with geometry
std::shared_ptr<GraphicsNode> addGraphics(const std::string& name, 
                                          const cvc::geometry& geom);

// Add empty graphics node (for grouping/hierarchy)
std::shared_ptr<GraphicsNode> addGraphics(const std::string& name);

// Remove graphics by name
void removeGraphics(const std::string& name);

// Get graphics by name
std::shared_ptr<GraphicsNode> getGraphics(const std::string& name);

// Get graphics root node
std::shared_ptr<GraphicsNode> getGraphicsRoot();

// Get all graphics (flat map)
const std::map<std::string, std::shared_ptr<GraphicsNode>>& getAllGraphics() const;

// State synchronization
void syncGraphicsToState();
void syncGraphicsFromState();
```

### GraphicsNode Methods

```cpp
// Naming
void setName(const std::string& name);
std::string getName() const;

// Geometry
void setGeometry(const cvc::geometry& geom);
bool hasGeometry() const;

// Transform
void setTransform(vtkMatrix4x4* matrix);
void setTransform(const double matrix[16]);
vtkMatrix4x4* getTransform();
void setPosition(double x, double y, double z);
void setRotation(double x, double y, double z); // degrees
void setScale(double x, double y, double z);
void resetTransform();
vtkSmartPointer<vtkMatrix4x4> getWorldTransform() const;

// Hierarchy
void addGraphicsChild(std::shared_ptr<GraphicsNode> child);
void removeGraphicsChild(std::shared_ptr<GraphicsNode> child);
std::shared_ptr<GraphicsNode> findChildByName(const std::string& name);
const std::vector<std::shared_ptr<GraphicsNode>>& getGraphicsChildren() const;

// Metadata
void setMetadata(const std::string& key, const std::any& value);
std::any getMetadata(const std::string& key) const;
bool hasMetadata(const std::string& key) const;
const std::map<std::string, std::any>& getAllMetadata() const;

// Visibility (inherited from SceneNode)
void setVisible(bool visible);
bool isVisible() const;

// State integration
void syncToState(cvc::state& parentState);
void syncFromState(const cvc::state& parentState);
```

## Migration from Single Geometry

The old `setGeometry()` method still exists for backward compatibility:

```cpp
// Old way (still works)
sceneGraph->setGeometry(geom);

// New way (recommended)
auto node = sceneGraph->addGraphics("geometry1", geom);
```

The old method creates a single `GeometryNode`, while the new system uses `GraphicsNode` objects stored in a hierarchy.

## Performance Considerations

- Each `GraphicsNode` creates its own VTK actor and mapper
- Transform updates propagate to all children recursively
- State tree synchronization should be done explicitly when needed
- Use the flat lookup map (`getAllGraphics()`) for fast access by name
- Hierarchical searches use `findChildByName()` which is recursive

## Future Enhancements

Potential additions:
- Bounding box visualization per graphics object
- Material/color properties per object
- Selection/picking support
- Animation/keyframe system
- LOD (Level of Detail) support
- Instancing for repeated geometry
