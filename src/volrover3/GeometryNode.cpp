#include <volrover3/GeometryNode.h>
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
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkVertex.h>
#include <vtkTransform.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <sstream>
#include <algorithm>
#include <set>

GeometryNode::GeometryNode(const std::string& statePath, const std::string& name)
    : GraphicsNode(statePath, name)
    , m_hasGeometry(false)
    , m_renderMode(GeometryRenderMode::TRIS)
    , m_actor(vtkSmartPointer<vtkActor>::New())
    , m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_polyData(vtkSmartPointer<vtkPolyData>::New())
{
    m_mapper->SetInputData(m_polyData);
    m_actor->SetMapper(m_mapper);
    
    // Set default material properties
    m_actor->GetProperty()->SetColor(0.8, 0.8, 0.9);
    m_actor->GetProperty()->SetSpecular(0.3);
    m_actor->GetProperty()->SetSpecularPower(20);
    
    // Initialize state tree with all rendering attributes
    if (!statePath.empty()) {
        getState("visible").value(1);  // Visible by default
        
        // Render mode
        getState("render_mode").value(renderModeToString(m_renderMode));
        
        // Material color (RGB 0-1)
        getState("color_r").value(0.8);
        getState("color_g").value(0.8);
        getState("color_b").value(0.9);
        
        // Material properties
        getState("specular").value(0.3);
        getState("specular_power").value(20.0);
        getState("ambient").value(0.0);  // VTK default
        getState("diffuse").value(1.0);  // VTK default
        getState("opacity").value(1.0);
        
        // Point/line rendering properties
        getState("point_size").value(3.0);
        getState("line_width").value(1.0);
    }
}

GeometryNode::~GeometryNode()
{
    m_dataConnection.disconnect();
}

void GeometryNode::applyTransformToVTK()
{
    // Use generic helper to apply world transform
    applyWorldTransformToProps({m_actor});
}

void GeometryNode::applyClipPlanes(vtkPlaneCollection* planes)
{
    if (m_mapper) {
        if (planes && planes->GetNumberOfItems() > 0) {
            m_mapper->SetClippingPlanes(planes);
        } else {
            m_mapper->RemoveAllClippingPlanes();
        }
    }
}

void GeometryNode::handleStateChanged(const std::string& childState)
{
    // Handle geometry-specific state changes
    // All VTK operations MUST be wrapped in runOnMainThread() for thread safety
    if (childState == "render_mode") {
        runOnMainThread([this]() {
            std::string renderModeStr = getState("render_mode").value<std::string>();
            GeometryRenderMode newMode = stringToRenderMode(renderModeStr);
            if (m_renderMode != newMode) {
                m_renderMode = newMode;
                updateRenderModeVTK();
            }
        });
    }
    else if (childState == "color_r" || childState == "color_g" || childState == "color_b") {
        runOnMainThread([this]() {
            // Only update if all color components can be read
            try {
                double r = getState("color_r").value<double>();
                double g = getState("color_g").value<double>();
                double b = getState("color_b").value<double>();
                m_actor->GetProperty()->SetColor(r, g, b);
            } catch (const boost::bad_lexical_cast&) {
                // Ignore - values not fully initialized yet
            }
        });
    }
    else if (childState == "specular") {
        runOnMainThread([this]() {
            double specular = getState("specular").value<double>();
            m_actor->GetProperty()->SetSpecular(specular);
        });
    }
    else if (childState == "specular_power") {
        runOnMainThread([this]() {
            double specularPower = getState("specular_power").value<double>();
            m_actor->GetProperty()->SetSpecularPower(specularPower);
        });
    }
    else if (childState == "ambient") {
        runOnMainThread([this]() {
            double ambient = getState("ambient").value<double>();
            m_actor->GetProperty()->SetAmbient(ambient);
        });
    }
    else if (childState == "diffuse") {
        runOnMainThread([this]() {
            double diffuse = getState("diffuse").value<double>();
            m_actor->GetProperty()->SetDiffuse(diffuse);
        });
    }
    else if (childState == "opacity") {
        runOnMainThread([this]() {
            double opacity = getState("opacity").value<double>();
            m_actor->GetProperty()->SetOpacity(opacity);
        });
    }
    else if (childState == "point_size") {
        runOnMainThread([this]() {
            double pointSize = getState("point_size").value<double>();
            m_actor->GetProperty()->SetPointSize(pointSize);
        });
    }
    else if (childState == "line_width") {
        runOnMainThread([this]() {
            double lineWidth = getState("line_width").value<double>();
            m_actor->GetProperty()->SetLineWidth(lineWidth);
        });
    }
    else {
        // Delegate to parent for common graphics fields
        // Parent will handle its own runOnMainThread wrapping
        GraphicsNode::handleStateChanged(childState);
    }
}

std::string GeometryNode::renderModeToString(GeometryRenderMode mode)
{
    return std::to_string(static_cast<int>(mode));
}

GeometryRenderMode GeometryNode::stringToRenderMode(const std::string& str)
{
    try {
        int mode = std::stoi(str);
        if (mode >= 0 && mode <= 5) {
            return static_cast<GeometryRenderMode>(mode);
        }
    } catch (...) {}
    return GeometryRenderMode::TRIS; // Default
}

void GeometryNode::setRenderMode(GeometryRenderMode mode)
{
    if (m_renderMode == mode) return;
    
    m_renderMode = mode;
    
    // Update state tree
    getState("render_mode").value(renderModeToString(mode));
    
    // Update VTK rendering on main thread
    runOnMainThread([this]() {
        updateRenderModeVTK();
    });
}

void GeometryNode::updateRenderModeVTK()
{
    // Update VTK rendering based on mode
    switch (m_renderMode) {
        case GeometryRenderMode::POINTS:
            m_actor->GetProperty()->SetRepresentationToPoints();
            m_actor->GetProperty()->SetPointSize(getState("point_size").value<double>());
            break;
            
        case GeometryRenderMode::LINES:
            m_actor->GetProperty()->SetRepresentationToWireframe();
            m_actor->GetProperty()->SetLineWidth(getState("line_width").value<double>());
            break;
            
        case GeometryRenderMode::TRIS:
        case GeometryRenderMode::QUADS:
            m_actor->GetProperty()->SetRepresentationToSurface();
            break;
            
        case GeometryRenderMode::TETS:
        case GeometryRenderMode::HEXS:
            // Placeholder: For now, render as wireframe
            // TODO: Implement proper volumetric mesh rendering
            m_actor->GetProperty()->SetRepresentationToWireframe();
            break;
    }
    
    // Trigger re-render if we have geometry
    if (m_hasGeometry && m_geometry) {
        updatePolyData(*m_geometry);
    }
    
    // Mark everything as modified to trigger re-render
    m_polyData->Modified();
    m_mapper->Modified();
    m_actor->Modified();
    
    // Request a render update (if we have a renderer with a render window)
    if (m_renderer && m_renderer->GetRenderWindow()) {
        m_renderer->GetRenderWindow()->Render();
    }
}

void GeometryNode::setColor(double r, double g, double b)
{
    getState("color_r").value(r);
    getState("color_g").value(g);
    getState("color_b").value(b);
}

void GeometryNode::setSpecular(double value)
{
    getState("specular").value(value);
}

void GeometryNode::setSpecularPower(double value)
{
    getState("specular_power").value(value);
}

void GeometryNode::setAmbient(double value)
{
    getState("ambient").value(value);
}

void GeometryNode::setDiffuse(double value)
{
    getState("diffuse").value(value);
}

void GeometryNode::setOpacity(double value)
{
    getState("opacity").value(value);
}

void GeometryNode::setPointSize(double size)
{
    getState("point_size").value(size);
}

void GeometryNode::setLineWidth(double width)
{
    getState("line_width").value(width);
}

vtkProp* GeometryNode::getProp()
{
    return m_actor;
}

void GeometryNode::setGeometry(const cvc::geometry& geom)
{
    cvc::thread_info ti(BOOST_CURRENT_FUNCTION);
    
    // Store the geometry object
    m_geometry = std::make_shared<cvc::geometry>(geom);
    m_hasGeometry = true;  // Set this BEFORE setRenderMode so it can update
    
    // Auto-detect render mode from geometry type
    GeometryRenderMode autoMode = GeometryRenderMode::TRIS; // default
    
    switch (geom.get_geometry_type()) {
        case cvc::geometry::SURFACE_TRI:
            autoMode = GeometryRenderMode::TRIS;
            break;
        case cvc::geometry::SURFACE_QUAD:
            autoMode = GeometryRenderMode::QUADS;
            break;
        case cvc::geometry::VOLUME_TET:
            autoMode = GeometryRenderMode::TETS;
            break;
        case cvc::geometry::VOLUME_HEX:
            autoMode = GeometryRenderMode::HEXS;
            break;
        case cvc::geometry::MIXED:
            // For mixed, prefer tris if available, otherwise quads
            if (geom.num_tris() > 0) {
                autoMode = GeometryRenderMode::TRIS;
            } else if (geom.num_quads() > 0) {
                autoMode = GeometryRenderMode::QUADS;
            } else if (geom.num_tets() > 0) {
                autoMode = GeometryRenderMode::TETS;
            } else if (geom.num_hexs() > 0) {
                autoMode = GeometryRenderMode::HEXS;
            }
            break;
    }
    
    // Update render mode and geometry data immediately on main thread
    // This ensures the geometry is ready when the node is added to the renderer
    runOnMainThread([this, autoMode]() {
        m_renderMode = autoMode;
        getState("render_mode").value(renderModeToString(autoMode));
        updateRenderModeVTK();  // This calls updatePolyData() internally
        updateBoundingBoxNode();
    });
    
    updateMetadata(geom);
}

void GeometryNode::updatePolyData(const cvc::geometry& geom)
{
    // Create VTK points from geometry
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(geom.num_points());

    for (size_t i = 0; i < geom.num_points(); ++i) {
        const auto& pt = geom.points()[i];
        points->SetPoint(i, pt[0], pt[1], pt[2]);
    }

    // Clear existing cells
    m_polyData->SetVerts(nullptr);
    m_polyData->SetLines(nullptr);
    m_polyData->SetPolys(nullptr);
    
    // Create cells based on render mode
    switch (m_renderMode) {
        case GeometryRenderMode::POINTS: {
            // Render as point cloud
            vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
            for (size_t i = 0; i < geom.num_points(); ++i) {
                vertices->InsertNextCell(1);
                vertices->InsertCellPoint(i);
            }
            m_polyData->SetVerts(vertices);
            break;
        }
        
        case GeometryRenderMode::LINES: {
            // Render as wireframe using edge connectivity
            vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
            
            // Add lines from line array if available
            for (size_t i = 0; i < geom.num_lines(); ++i) {
                const auto& line = geom.lines()[i];
                lines->InsertNextCell(2);
                lines->InsertCellPoint(line[0]);
                lines->InsertCellPoint(line[1]);
            }
            
            // Add triangle edges
            for (size_t i = 0; i < geom.num_tris(); ++i) {
                const auto& tri = geom.tris()[i];
                // Three edges per triangle
                lines->InsertNextCell(2);
                lines->InsertCellPoint(tri[0]);
                lines->InsertCellPoint(tri[1]);
                
                lines->InsertNextCell(2);
                lines->InsertCellPoint(tri[1]);
                lines->InsertCellPoint(tri[2]);
                
                lines->InsertNextCell(2);
                lines->InsertCellPoint(tri[2]);
                lines->InsertCellPoint(tri[0]);
            }
            
            // Add quad edges
            for (size_t i = 0; i < geom.num_quads(); ++i) {
                const auto& quad = geom.quads()[i];
                // Four edges per quad
                lines->InsertNextCell(2);
                lines->InsertCellPoint(quad[0]);
                lines->InsertCellPoint(quad[1]);
                
                lines->InsertNextCell(2);
                lines->InsertCellPoint(quad[1]);
                lines->InsertCellPoint(quad[2]);
                
                lines->InsertNextCell(2);
                lines->InsertCellPoint(quad[2]);
                lines->InsertCellPoint(quad[3]);
                
                lines->InsertNextCell(2);
                lines->InsertCellPoint(quad[3]);
                lines->InsertCellPoint(quad[0]);
            }
            
            m_polyData->SetLines(lines);
            break;
        }
        
        case GeometryRenderMode::TRIS: {
            // Render triangles as solid surface
            vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
            
            for (size_t i = 0; i < geom.num_tris(); ++i) {
                const auto& tri = geom.tris()[i];
                triangles->InsertNextCell(3);
                triangles->InsertCellPoint(tri[0]);
                triangles->InsertCellPoint(tri[1]);
                triangles->InsertCellPoint(tri[2]);
            }
            
            m_polyData->SetPolys(triangles);
            break;
        }
        
        case GeometryRenderMode::QUADS: {
            // Render quads as solid surface
            vtkSmartPointer<vtkCellArray> quads = vtkSmartPointer<vtkCellArray>::New();
            
            for (size_t i = 0; i < geom.num_quads(); ++i) {
                const auto& quad = geom.quads()[i];
                quads->InsertNextCell(4);
                quads->InsertCellPoint(quad[0]);
                quads->InsertCellPoint(quad[1]);
                quads->InsertCellPoint(quad[2]);
                quads->InsertCellPoint(quad[3]);
            }
            
            m_polyData->SetPolys(quads);
            break;
        }
        
        case GeometryRenderMode::TETS: {
            // TODO: Implement tetrahedral mesh rendering
            // For now, render as wireframe edges
            vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
            
            for (size_t i = 0; i < geom.num_tets(); ++i) {
                const auto& tet = geom.tets()[i];
                // 6 edges per tet: (0,1), (0,2), (0,3), (1,2), (1,3), (2,3)
                const int edges[6][2] = {{0,1}, {0,2}, {0,3}, {1,2}, {1,3}, {2,3}};
                for (int e = 0; e < 6; ++e) {
                    lines->InsertNextCell(2);
                    lines->InsertCellPoint(tet[edges[e][0]]);
                    lines->InsertCellPoint(tet[edges[e][1]]);
                }
            }
            
            m_polyData->SetLines(lines);
            break;
        }
        
        case GeometryRenderMode::HEXS: {
            // TODO: Implement hexahedral mesh rendering
            // For now, render as wireframe edges
            vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
            
            for (size_t i = 0; i < geom.num_hexs(); ++i) {
                const auto& hex = geom.hexs()[i];
                // 12 edges per hex
                const int edges[12][2] = {
                    {0,1}, {1,2}, {2,3}, {3,0},  // Bottom face
                    {4,5}, {5,6}, {6,7}, {7,4},  // Top face
                    {0,4}, {1,5}, {2,6}, {3,7}   // Vertical edges
                };
                for (int e = 0; e < 12; ++e) {
                    lines->InsertNextCell(2);
                    lines->InsertCellPoint(hex[edges[e][0]]);
                    lines->InsertCellPoint(hex[edges[e][1]]);
                }
            }
            
            m_polyData->SetLines(lines);
            break;
        }
    }

    // Update polydata points
    m_polyData->SetPoints(points);

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

cvc::bounding_box GeometryNode::getBoundingBox() const
{
    if (m_geometry) {
        try {
            return m_geometry->extents();
        } catch (...) {
            // extents() can throw for empty/invalid geometry
            return cvc::bounding_box(0, 0, 0, 0, 0, 0);
        }
    }
    // Return empty bounding box
    return cvc::bounding_box(0, 0, 0, 0, 0, 0);
}

// Note: syncToState and syncFromState removed - state_object handles state synchronization automatically
bool GeometryNode::isComputedMetadata(const std::string& key)
{
    // These metadata keys are computed from geometry data and should be read-only
    static const std::set<std::string> computedKeys = {
        "num_vertices", "num_triangles", "num_quads", "num_lines",
        "bbox_min_x", "bbox_min_y", "bbox_min_z",
        "bbox_max_x", "bbox_max_y", "bbox_max_z",
        "extent_x", "extent_y", "extent_z",
        "center_x", "center_y", "center_z",
        "bounding_box", "type", "filename",
        "combined_bbox_min_x", "combined_bbox_min_y", "combined_bbox_min_z",
        "combined_bbox_max_x", "combined_bbox_max_y", "combined_bbox_max_z",
        "combined_extent_x", "combined_extent_y", "combined_extent_z",
        "combined_center_x", "combined_center_y", "combined_center_z"
    };
    
    return computedKeys.find(key) != computedKeys.end();
}

void GeometryNode::updateMetadata(const cvc::geometry& geom)
{
    // Update all geometry statistics as metadata
    setMetadata("num_vertices", static_cast<int>(geom.num_points()));
    setMetadata("num_triangles", static_cast<int>(geom.num_tris()));
    setMetadata("num_quads", static_cast<int>(geom.num_quads()));
    
    // Only compute bounding box if geometry has points
    if (geom.num_points() > 0) {
        try {
            // Get bounding box extents
            auto bbox = geom.extents();
            
            setMetadata("bbox_min_x", bbox.minx);
            setMetadata("bbox_min_y", bbox.miny);
            setMetadata("bbox_min_z", bbox.minz);
            setMetadata("bbox_max_x", bbox.maxx);
            setMetadata("bbox_max_y", bbox.maxy);
            setMetadata("bbox_max_z", bbox.maxz);
            
            // Store combined bounding box string for computeGraphicsBounds()
            std::string bboxStr = std::to_string(bbox.minx) + "," +
                                  std::to_string(bbox.miny) + "," +
                                  std::to_string(bbox.minz) + "," +
                                  std::to_string(bbox.maxx) + "," +
                                  std::to_string(bbox.maxy) + "," +
                                  std::to_string(bbox.maxz);
            setMetadata("bounding_box", bboxStr);
            
            // Compute extents (dimensions)
            double extentX = bbox.maxx - bbox.minx;
            double extentY = bbox.maxy - bbox.miny;
            double extentZ = bbox.maxz - bbox.minz;
            
            setMetadata("extent_x", extentX);
            setMetadata("extent_y", extentY);
            setMetadata("extent_z", extentZ);
            
            // Compute center point
            setMetadata("center_x", (bbox.minx + bbox.maxx) / 2.0);
            setMetadata("center_y", (bbox.miny + bbox.maxy) / 2.0);
            setMetadata("center_z", (bbox.minz + bbox.maxz) / 2.0);
        } catch (...) {
            // Failed to compute bounding box for empty or invalid geometry
        }
    }
    
    // Add geometry type
    std::string geomType = "mesh";
    if (geom.num_tris() > 0 && geom.num_quads() == 0) {
        geomType = "triangle_mesh";
    } else if (geom.num_quads() > 0 && geom.num_tris() == 0) {
        geomType = "quad_mesh";
    } else if (geom.num_tris() > 0 && geom.num_quads() > 0) {
        geomType = "mixed_mesh";
    } else if (geom.num_points() == 0) {
        geomType = "empty";
    }
    setMetadata("type", geomType);
}

void GeometryNode::onDataChanged()
{
    // Called when state data changes - reload geometry from state
    // Note: With state_object, we access state via getState() instead of m_stateNode
    if (getState().isData<cvc::geometry>()) {
        try {
            const cvc::geometry& geom = boost::any_cast<const cvc::geometry&>(getState().data());
            setGeometry(geom);
        } catch (...) {
            // Failed to load geometry from state
        }
    }
}
