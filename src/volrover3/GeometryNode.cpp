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
#include <sstream>
#include <algorithm>
#include <set>

GeometryNode::GeometryNode(const std::string& name)
    : GraphicsNode(name)
    , m_hasGeometry(false)
    , m_actor(vtkSmartPointer<vtkActor>::New())
    , m_mapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_polyData(vtkSmartPointer<vtkPolyData>::New())
    , m_stateNode(nullptr)
{
    m_mapper->SetInputData(m_polyData);
    m_actor->SetMapper(m_mapper);
    
    // Set default material properties
    m_actor->GetProperty()->SetColor(0.8, 0.8, 0.9);
    m_actor->GetProperty()->SetSpecular(0.3);
    m_actor->GetProperty()->SetSpecularPower(20);
}

GeometryNode::~GeometryNode()
{
    m_dataConnection.disconnect();
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
    
    updatePolyData(geom);
    updateMetadata(geom);
    m_hasGeometry = true;
    
    // Update bbox to match geometry bounds
    updateBoundingBoxNode();
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

void GeometryNode::syncToState(cvc::state& parentState)
{
    cvc::state& myState = parentState(m_name);
    myState.comment("Graphics object with geometry data and transform");
    
    // Store geometry data
    if (m_geometry) {
        myState.data(*m_geometry);
    }
    
    // Save common graphics attributes (transform, bbox, label, children, combined bbox if has children)
    saveCommonStateAttributes(myState);
    
    // Store metadata with appropriate readOnly flags and comments
    cvc::state& metadataState = myState("metadata");
    metadataState.comment("Computed geometry statistics and properties");
    
    for (const auto& [key, value] : m_metadata) {
        try {
            cvc::state& metaEntry = metadataState(key);
            
            if (value.type() == typeid(std::string)) {
                metaEntry.value(std::any_cast<std::string>(value));
            } else if (value.type() == typeid(double)) {
                metaEntry.value(std::any_cast<double>(value));
            } else if (value.type() == typeid(int)) {
                metaEntry.value(std::any_cast<int>(value));
            } else if (value.type() == typeid(bool)) {
                bool boolVal = std::any_cast<bool>(value);
                metaEntry.value(boolVal ? "true" : "false");
            }
            
            // Set readOnly flag for computed metadata
            metaEntry.readOnly(isComputedMetadata(key));
            
            // Set descriptive comments for known metadata keys
            if (key == "num_vertices") metaEntry.comment("Number of vertices in the geometry");
            else if (key == "num_triangles") metaEntry.comment("Number of triangles in the geometry");
            else if (key == "num_quads") metaEntry.comment("Number of quads in the geometry");
            else if (key == "bbox_min_x") metaEntry.comment("Minimum X coordinate of bounding box");
            else if (key == "bbox_min_y") metaEntry.comment("Minimum Y coordinate of bounding box");
            else if (key == "bbox_min_z") metaEntry.comment("Minimum Z coordinate of bounding box");
            else if (key == "bbox_max_x") metaEntry.comment("Maximum X coordinate of bounding box");
            else if (key == "bbox_max_y") metaEntry.comment("Maximum Y coordinate of bounding box");
            else if (key == "bbox_max_z") metaEntry.comment("Maximum Z coordinate of bounding box");
            else if (key == "extent_x") metaEntry.comment("Width (X dimension) of bounding box");
            else if (key == "extent_y") metaEntry.comment("Height (Y dimension) of bounding box");
            else if (key == "extent_z") metaEntry.comment("Depth (Z dimension) of bounding box");
            else if (key == "center_x") metaEntry.comment("X coordinate of bounding box center");
            else if (key == "center_y") metaEntry.comment("Y coordinate of bounding box center");
            else if (key == "center_z") metaEntry.comment("Z coordinate of bounding box center");
            else if (key == "combined_bbox_min_x") metaEntry.comment("Minimum X coordinate of combined bounding box (this + children)");
            else if (key == "combined_bbox_min_y") metaEntry.comment("Minimum Y coordinate of combined bounding box (this + children)");
            else if (key == "combined_bbox_min_z") metaEntry.comment("Minimum Z coordinate of combined bounding box (this + children)");
            else if (key == "combined_bbox_max_x") metaEntry.comment("Maximum X coordinate of combined bounding box (this + children)");
            else if (key == "combined_bbox_max_y") metaEntry.comment("Maximum Y coordinate of combined bounding box (this + children)");
            else if (key == "combined_bbox_max_z") metaEntry.comment("Maximum Z coordinate of combined bounding box (this + children)");
            else if (key == "combined_extent_x") metaEntry.comment("Width (X dimension) of combined bounding box");
            else if (key == "combined_extent_y") metaEntry.comment("Height (Y dimension) of combined bounding box");
            else if (key == "combined_extent_z") metaEntry.comment("Depth (Z dimension) of combined bounding box");
            else if (key == "combined_center_x") metaEntry.comment("X coordinate of combined bounding box center");
            else if (key == "combined_center_y") metaEntry.comment("Y coordinate of combined bounding box center");
            else if (key == "combined_center_z") metaEntry.comment("Z coordinate of combined bounding box center");
            else if (key == "type") metaEntry.comment("Geometry type (triangle_mesh, quad_mesh, etc.)");
            else if (key == "bounding_box") metaEntry.comment("Complete bounding box as comma-separated values");
            else if (key == "filename") metaEntry.comment("Source filename for the geometry");
            
        } catch (...) {
            // Skip metadata that can't be serialized
        }
    }
}

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

void GeometryNode::syncFromState(cvc::state& parentState)
{
    try {
        cvc::state& myState = parentState(m_name);
        
        m_stateNode = &myState;
        m_dataConnection.disconnect(); // Disconnect any previous connection
        m_dataConnection = myState.dataChanged.connect([this]() {
            onDataChanged();
        });
        
        // Load geometry data
        if (myState.isData<cvc::geometry>()) {
            try {
                const cvc::geometry& geom = boost::any_cast<const cvc::geometry&>(myState.data());
                setGeometry(geom);
            } catch (...) {}
        }
        
        // Load transform
        try {
            std::string transformStr = myState("transform").value();
            std::vector<double> values;
            std::stringstream ss(transformStr);
            std::string token;
            while (std::getline(ss, token, ',')) {
                values.push_back(std::stod(token));
            }
            
            if (values.size() == 16) {
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        m_transform->SetElement(i, j, values[i * 4 + j]);
                    }
                }
                updateTransform();
            }
        } catch (...) {}
        
        // Load metadata
        try {
            cvc::state& metadataState = myState("metadata");
            std::vector<std::string> metadataKeys = metadataState.children();
            
            // Filter to direct children only
            std::string metadataPath = metadataState.fullName();
            int expectedDepth = std::count(metadataPath.begin(), metadataPath.end(), '.') + 1;
            
            for (const auto& descendant : metadataKeys) {
                int depth = std::count(descendant.begin(), descendant.end(), '.');
                if (depth == expectedDepth) {
                    // Extract just the key name
                    size_t lastDot = descendant.find_last_of('.');
                    std::string key = (lastDot != std::string::npos) ? descendant.substr(lastDot + 1) : descendant;
                    
                    try {
                        std::string value = metadataState(key).value();
                        
                        // Store all metadata as strings when loading from state
                        // The state tree stores everything as strings anyway
                        if (value == "true") {
                            setMetadata(key, true);
                        } else if (value == "false") {
                            setMetadata(key, false);
                        } else {
                            setMetadata(key, value);
                        }
                    } catch (...) {}
                }
            }
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
    if (!m_stateNode) return;
    
    if (m_stateNode->isData<cvc::geometry>()) {
        try {
            const cvc::geometry& geom = boost::any_cast<const cvc::geometry&>(m_stateNode->data());
            setGeometry(geom);
        } catch (...) {
            // Failed to load geometry from state
        }
    }
}
