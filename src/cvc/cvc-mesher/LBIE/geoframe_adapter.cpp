/******************************************************************************
Week 4 Modernization: geoframe_adapter implementation
Purpose: One-copy adapter between geoframe and geometry (50% reduction vs before)

Week 4 Design:
  - Copies geometry data into geoframe on construction (sync_from_geometry)
  - Copies geoframe data back to geometry on destruction (sync_to_geometry)
  - Cannot achieve true zero-copy due to geoframe's owned std::vector members
  - Still provides 50% reduction: 1 copy instead of 2 (geometry→geoframe→geometry)

Week 5 Plan (True Zero-Copy):
  - Replace geoframe's std::vector with pointers to geometry's CUDA unified memory
  - LBIE accesses geometry data directly via pointers (no copy)
  - Geometry uses cudaMallocManaged() for GPU-accessible memory
  - Result: 100% elimination of copies
******************************************************************************/

#include "geoframe_adapter.h"
#include <stdexcept>

namespace LBIE
{

geoframe_adapter::geoframe_adapter(cvc::geometry& geom)
  : geoframe(), _geom(geom), _dirty(false)
{
  sync_from_geometry();
}

geoframe_adapter::~geoframe_adapter()
{
  // Sync back to geometry if modified
  if (_dirty) {
    sync_to_geometry();
  }
}

geoframe_adapter::geoframe_adapter(const geoframe_adapter& other)
  : geoframe(other), _geom(other._geom), _dirty(false)
{
  // Shared reference semantics - both adapters point to same geometry
}

geoframe_adapter& geoframe_adapter::operator=(const geoframe_adapter& other)
{
  if (this != &other) {
    // Sync current state back
    if (_dirty) {
      sync_to_geometry();
    }
    
    // Copy base geoframe data
    geoframe::operator=(other);
    
    // Note: _geom reference cannot be rebound, this is intentional
    // Each adapter is bound to its geometry for lifetime
    _dirty = false;
  }
  return *this;
}

void geoframe_adapter::sync_from_geometry()
{
  // Clear existing geoframe data
  geoframe::reset();
  
  // Week 4: Copy data from geometry to geoframe
  // Week 5 TODO: Replace with direct pointer references to CUDA unified memory
  
  const auto& points = _geom.const_points();
  const auto& normals = _geom.const_normals();
  const auto& colors = _geom.const_colors();
  const auto& curvatures = _geom.const_curvatures();
  const auto& functions = _geom.const_functions();
  const auto& tris = _geom.const_tris();
  const auto& quads = _geom.const_quads();
  const auto& tets = _geom.const_tets();
  const auto& hexs = _geom.const_hexs();
  
  // Copy vertex data
  // Week 5: Replace with: verts.data() = reinterpret_cast<float_3*>(points.data())
  numverts = points.size();
  verts.resize(numverts);
  for (size_t i = 0; i < points.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      verts[i][j] = static_cast<float>(points[i][j]);
    }
  }
  
  // Copy normals
  if (normals.size() >= points.size()) {
    this->normals.resize(numverts);
    for (size_t i = 0; i < normals.size() && i < points.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
        this->normals[i][j] = static_cast<float>(normals[i][j]);
      }
    }
  }
  
  // Copy colors
  if (colors.size() >= points.size()) {
    this->color.resize(numverts);
    for (size_t i = 0; i < colors.size() && i < points.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
        this->color[i][j] = static_cast<float>(colors[i][j]);
      }
    }
  }
  
  // Copy curvatures
  if (curvatures.size() >= points.size()) {
    this->curvatures.resize(numverts);
    for (size_t i = 0; i < curvatures.size() && i < points.size(); ++i) {
      for (int j = 0; j < 2; ++j) {
        this->curvatures[i][j] = static_cast<float>(curvatures[i][j]);
      }
    }
  }
  
  // Copy functions
  if (functions.size() >= points.size()) {
    this->funcs.resize(numverts);
    for (size_t i = 0; i < functions.size() && i < points.size(); ++i) {
      this->funcs[i][0] = static_cast<float>(functions[i]);
    }
  }
  
  // Determine mesh type and copy element data
  bool has_tets = !tets.empty();
  bool has_hexs = !hexs.empty();
  bool has_tris = !tris.empty();
  bool has_quads = !quads.empty();
  
  if (has_tets && !has_hexs) {
    // Tetrahedral mesh - encode tets as triangles (4 tris per tet)
    mesh_type = geoframe::TETRA;
    numtris = tets.size() * 4;
    triangles.resize(numtris);
    
    for (size_t i = 0; i < tets.size(); ++i) {
      const auto& tet = tets[i];
      // Encode tet faces as triangles
      triangles[i*4 + 0][0] = static_cast<unsigned int>(tet[0]);
      triangles[i*4 + 0][1] = static_cast<unsigned int>(tet[1]);
      triangles[i*4 + 0][2] = static_cast<unsigned int>(tet[2]);
      
      triangles[i*4 + 1][0] = static_cast<unsigned int>(tet[0]);
      triangles[i*4 + 1][1] = static_cast<unsigned int>(tet[1]);
      triangles[i*4 + 1][2] = static_cast<unsigned int>(tet[3]);
      
      triangles[i*4 + 2][0] = static_cast<unsigned int>(tet[1]);
      triangles[i*4 + 2][1] = static_cast<unsigned int>(tet[2]);
      triangles[i*4 + 2][2] = static_cast<unsigned int>(tet[3]);
      
      triangles[i*4 + 3][0] = static_cast<unsigned int>(tet[0]);
      triangles[i*4 + 3][1] = static_cast<unsigned int>(tet[2]);
      triangles[i*4 + 3][2] = static_cast<unsigned int>(tet[3]);
    }
  }
  else if (has_hexs && !has_tets) {
    // Hexahedral mesh - encode hexs as quads (6 quads per hex)
    mesh_type = geoframe::HEXA;
    numquads = hexs.size() * 6;
    this->quads.resize(numquads);  // Use this->quads to get non-const member
    
    for (size_t i = 0; i < hexs.size(); ++i) {
      const auto& hex = hexs[i];
      // Encode hex faces as quads
      this->quads[i*6 + 0][0] = static_cast<unsigned int>(hex[0]);
      this->quads[i*6 + 0][1] = static_cast<unsigned int>(hex[1]);
      this->quads[i*6 + 0][2] = static_cast<unsigned int>(hex[2]);
      this->quads[i*6 + 0][3] = static_cast<unsigned int>(hex[3]);
      
      this->quads[i*6 + 1][0] = static_cast<unsigned int>(hex[4]);
      this->quads[i*6 + 1][1] = static_cast<unsigned int>(hex[5]);
      this->quads[i*6 + 1][2] = static_cast<unsigned int>(hex[6]);
      this->quads[i*6 + 1][3] = static_cast<unsigned int>(hex[7]);
      
      this->quads[i*6 + 2][0] = static_cast<unsigned int>(hex[0]);
      this->quads[i*6 + 2][1] = static_cast<unsigned int>(hex[1]);
      this->quads[i*6 + 2][2] = static_cast<unsigned int>(hex[5]);
      this->quads[i*6 + 2][3] = static_cast<unsigned int>(hex[4]);
      
      this->quads[i*6 + 3][0] = static_cast<unsigned int>(hex[1]);
      this->quads[i*6 + 3][1] = static_cast<unsigned int>(hex[2]);
      this->quads[i*6 + 3][2] = static_cast<unsigned int>(hex[6]);
      this->quads[i*6 + 3][3] = static_cast<unsigned int>(hex[5]);
      
      this->quads[i*6 + 4][0] = static_cast<unsigned int>(hex[2]);
      this->quads[i*6 + 4][1] = static_cast<unsigned int>(hex[3]);
      this->quads[i*6 + 4][2] = static_cast<unsigned int>(hex[7]);
      this->quads[i*6 + 4][3] = static_cast<unsigned int>(hex[6]);
      
      this->quads[i*6 + 5][0] = static_cast<unsigned int>(hex[3]);
      this->quads[i*6 + 5][1] = static_cast<unsigned int>(hex[0]);
      this->quads[i*6 + 5][2] = static_cast<unsigned int>(hex[4]);
      this->quads[i*6 + 5][3] = static_cast<unsigned int>(hex[7]);
    }
  }
  else if (has_tris && !has_quads) {
    // Triangle surface mesh
    mesh_type = geoframe::SINGLE;
    numtris = tris.size();
    triangles.resize(numtris);
    
    for (size_t i = 0; i < tris.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
        triangles[i][j] = static_cast<unsigned int>(tris[i][j]);
      }
    }
  }
  else if (has_quads && !has_tris) {
    // Quad surface mesh
    mesh_type = geoframe::QUAD;
    numquads = quads.size();
    this->quads.resize(numquads);
    
    for (size_t i = 0; i < quads.size(); ++i) {
      for (int j = 0; j < 4; ++j) {
        this->quads[i][j] = static_cast<unsigned int>(quads[i][j]);
      }
    }
  }
  else {
    // Mixed or empty
    mesh_type = geoframe::SINGLE;
    
    numtris = tris.size();
    triangles.resize(numtris);
    for (size_t i = 0; i < tris.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
        triangles[i][j] = static_cast<unsigned int>(tris[i][j]);
      }
    }
    
    numquads = quads.size();
    this->quads.resize(numquads);
    for (size_t i = 0; i < quads.size(); ++i) {
      for (int j = 0; j < 4; ++j) {
        this->quads[i][j] = static_cast<unsigned int>(quads[i][j]);
      }
    }
  }
  
  // Copy bounding box info
  auto bbox = _geom.extents();
  min_x = bbox.minx;
  min_y = bbox.miny;
  min_z = bbox.minz;
  max_x = bbox.maxx;
  max_y = bbox.maxy;
  max_z = bbox.maxz;
  
  span[0] = max_x - min_x;
  span[1] = max_y - min_y;
  span[2] = max_z - min_z;
  
  _dirty = false;
}

void geoframe_adapter::sync_to_geometry()
{
  // Copy vertex data back to geometry
  auto& points = _geom.points();
  points.resize(numverts);
  for (int i = 0; i < numverts; ++i) {
    for (int j = 0; j < 3; ++j) {
      points[i][j] = verts[i][j];
    }
  }
  
  // Copy normals back
  if (!normals.empty()) {
    auto& geom_normals = _geom.normals();
    geom_normals.resize(numverts);
    for (int i = 0; i < numverts && i < static_cast<int>(normals.size()); ++i) {
      for (int j = 0; j < 3; ++j) {
        geom_normals[i][j] = normals[i][j];
      }
    }
  }
  
  // Copy colors back
  if (!color.empty()) {
    auto& geom_colors = _geom.colors();
    geom_colors.resize(numverts);
    for (int i = 0; i < numverts && i < static_cast<int>(color.size()); ++i) {
      for (int j = 0; j < 3; ++j) {
        geom_colors[i][j] = color[i][j];
      }
    }
  }
  
  // Copy curvatures back
  if (!curvatures.empty()) {
    auto& geom_curvatures = _geom.curvatures();
    geom_curvatures.resize(numverts);
    for (int i = 0; i < numverts && i < static_cast<int>(curvatures.size()); ++i) {
      for (int j = 0; j < 2; ++j) {
        geom_curvatures[i][j] = curvatures[i][j];
      }
    }
  }
  
  // Copy functions back
  if (!funcs.empty()) {
    auto& geom_functions = _geom.functions();
    geom_functions.resize(numverts);
    for (int i = 0; i < numverts && i < static_cast<int>(funcs.size()); ++i) {
      geom_functions[i] = funcs[i][0];
    }
  }
  
  // Decode and copy element data based on mesh_type
  if (mesh_type == geoframe::TETRA || mesh_type == geoframe::TETRA2) {
    // Decode triangles back to tets
    int num_tets = numtris / 4;
    auto& tets = _geom.tets();
    tets.clear();
    tets.reserve(num_tets);
    
    for (int i = 0; i < num_tets; ++i) {
      cvc::geometry::tet_t tet;
      // Decode from triangle encoding
      tet[0] = triangles[i*4 + 0][0];
      tet[1] = triangles[i*4 + 0][1];
      tet[2] = triangles[i*4 + 0][2];
      tet[3] = triangles[i*4 + 1][2];
      tets.push_back(tet);
    }
    
    _geom.set_geometry_type(cvc::geometry::VOLUME_TET);
  }
  else if (mesh_type == geoframe::HEXA) {
    // Decode quads back to hexs
    int num_hexs = numquads / 6;
    auto& hexs = _geom.hexs();
    hexs.clear();
    hexs.reserve(num_hexs);
    
    for (int i = 0; i < num_hexs; ++i) {
      cvc::geometry::hex_t hex;
      // Decode from quad encoding
      hex[0] = quads[i*6 + 0][0];
      hex[1] = quads[i*6 + 0][1];
      hex[2] = quads[i*6 + 0][2];
      hex[3] = quads[i*6 + 0][3];
      hex[4] = quads[i*6 + 1][0];
      hex[5] = quads[i*6 + 1][1];
      hex[6] = quads[i*6 + 1][2];
      hex[7] = quads[i*6 + 1][3];
      hexs.push_back(hex);
    }
    
    _geom.set_geometry_type(cvc::geometry::VOLUME_HEX);
  }
  else if (mesh_type == geoframe::QUAD) {
    // Copy quads
    auto& geom_quads = _geom.quads();
    geom_quads.resize(numquads);
    for (int i = 0; i < numquads; ++i) {
      for (int j = 0; j < 4; ++j) {
        geom_quads[i][j] = quads[i][j];
      }
    }
    _geom.set_geometry_type(cvc::geometry::SURFACE_QUAD);
  }
  else {
    // Copy triangles (SINGLE or other)
    auto& geom_tris = _geom.tris();
    geom_tris.resize(numtris);
    for (int i = 0; i < numtris; ++i) {
      for (int j = 0; j < 3; ++j) {
        geom_tris[i][j] = triangles[i][j];
      }
    }
    _geom.set_geometry_type(cvc::geometry::SURFACE_TRI);
  }
  
  _dirty = false;
}

void geoframe_adapter::reset()
{
  geoframe::reset();
  _geom.clear();
  _dirty = false;
}

// Helper functions

cvc::geometry to_geometry(const geoframe& gf)
{
  cvc::geometry geom;
  
  // Copy vertex data
  auto& points = geom.points();
  points.resize(gf.numverts);
  for (int i = 0; i < gf.numverts; ++i) {
    for (int j = 0; j < 3; ++j) {
      points[i][j] = gf.verts[i][j];
    }
  }
  
  // Copy normals
  if (!gf.normals.empty()) {
    auto& normals = geom.normals();
    normals.resize(gf.numverts);
    for (int i = 0; i < gf.numverts && i < static_cast<int>(gf.normals.size()); ++i) {
      for (int j = 0; j < 3; ++j) {
        normals[i][j] = gf.normals[i][j];
      }
    }
  }
  
  // Copy colors
  if (!gf.color.empty()) {
    auto& colors = geom.colors();
    colors.resize(gf.numverts);
    for (int i = 0; i < gf.numverts && i < static_cast<int>(gf.color.size()); ++i) {
      for (int j = 0; j < 3; ++j) {
        colors[i][j] = gf.color[i][j];
      }
    }
  }
  
  // Copy curvatures
  if (!gf.curvatures.empty()) {
    auto& curvatures = geom.curvatures();
    curvatures.resize(gf.numverts);
    for (int i = 0; i < gf.numverts && i < static_cast<int>(gf.curvatures.size()); ++i) {
      for (int j = 0; j < 2; ++j) {
        curvatures[i][j] = gf.curvatures[i][j];
      }
    }
  }
  
  // Copy functions
  if (!gf.funcs.empty()) {
    auto& functions = geom.functions();
    functions.resize(gf.numverts);
    for (int i = 0; i < gf.numverts && i < static_cast<int>(gf.funcs.size()); ++i) {
      functions[i] = gf.funcs[i][0];
    }
  }
  
  // Decode element data based on mesh_type
  if (gf.mesh_type == geoframe::TETRA || gf.mesh_type == geoframe::TETRA2) {
    // Decode triangles to tets
    int num_tets = gf.numtris / 4;
    auto& tets = geom.tets();
    tets.reserve(num_tets);
    
    for (int i = 0; i < num_tets; ++i) {
      cvc::geometry::tet_t tet;
      tet[0] = gf.triangles[i*4 + 0][0];
      tet[1] = gf.triangles[i*4 + 0][1];
      tet[2] = gf.triangles[i*4 + 0][2];
      tet[3] = gf.triangles[i*4 + 1][2];
      tets.push_back(tet);
    }
    
    geom.set_geometry_type(cvc::geometry::VOLUME_TET);
  }
  else if (gf.mesh_type == geoframe::HEXA) {
    // Decode quads to hexs
    int num_hexs = gf.numquads / 6;
    auto& hexs = geom.hexs();
    hexs.reserve(num_hexs);
    
    for (int i = 0; i < num_hexs; ++i) {
      cvc::geometry::hex_t hex;
      hex[0] = gf.quads[i*6 + 0][0];
      hex[1] = gf.quads[i*6 + 0][1];
      hex[2] = gf.quads[i*6 + 0][2];
      hex[3] = gf.quads[i*6 + 0][3];
      hex[4] = gf.quads[i*6 + 1][0];
      hex[5] = gf.quads[i*6 + 1][1];
      hex[6] = gf.quads[i*6 + 1][2];
      hex[7] = gf.quads[i*6 + 1][3];
      hexs.push_back(hex);
    }
    
    geom.set_geometry_type(cvc::geometry::VOLUME_HEX);
  }
  else if (gf.mesh_type == geoframe::QUAD) {
    // Copy quads
    auto& quads = geom.quads();
    quads.resize(gf.numquads);
    for (int i = 0; i < gf.numquads; ++i) {
      for (int j = 0; j < 4; ++j) {
        quads[i][j] = gf.quads[i][j];
      }
    }
    geom.set_geometry_type(cvc::geometry::SURFACE_QUAD);
  }
  else {
    // Copy triangles
    auto& tris = geom.tris();
    tris.resize(gf.numtris);
    for (int i = 0; i < gf.numtris; ++i) {
      for (int j = 0; j < 3; ++j) {
        tris[i][j] = gf.triangles[i][j];
      }
    }
    geom.set_geometry_type(cvc::geometry::SURFACE_TRI);
  }
  
  return geom;
}

geoframe to_geoframe(const cvc::geometry& geom)
{
  geoframe gf;
  
  // Create working copy of geometry to ensure normals are computed
  cvc::geometry working_geom(geom);
  
  // Ensure normals are computed if we have triangles or quads
  const auto& normals = working_geom.const_normals();
  const auto& tris = working_geom.const_tris();
  const auto& quads = working_geom.const_quads();
  
  if (normals.size() < working_geom.num_points() && (!tris.empty() || !quads.empty())) {
    working_geom.compute_normals();
  }
  
  const auto& points = working_geom.const_points();
  const auto& computed_normals = working_geom.const_normals();
  const auto& colors = working_geom.const_colors();
  const auto& curvatures = working_geom.const_curvatures();
  const auto& functions = working_geom.const_functions();
  const auto& tets = working_geom.const_tets();
  const auto& hexs = working_geom.const_hexs();
  
  // Copy vertex data
  gf.numverts = points.size();
  gf.verts.resize(gf.numverts);
  for (size_t i = 0; i < points.size(); ++i) {
    for (int j = 0; j < 3; ++j) {
      gf.verts[i][j] = static_cast<float>(points[i][j]);
    }
  }
  
  // Initialize bound_sign (0 = interior vertex, 1 = boundary vertex)
  // Quality improvement algorithms need this initialized
  gf.bound_sign.resize(gf.numverts, 0);
  
  // Copy normals (now guaranteed to exist if we have triangles/quads)
  if (computed_normals.size() >= points.size()) {
    gf.normals.resize(gf.numverts);
    for (size_t i = 0; i < computed_normals.size() && i < points.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
        gf.normals[i][j] = static_cast<float>(computed_normals[i][j]);
      }
    }
  }
  
  // Copy colors
  if (colors.size() >= points.size()) {
    gf.color.resize(gf.numverts);
    for (size_t i = 0; i < colors.size() && i < points.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
        gf.color[i][j] = static_cast<float>(colors[i][j]);
      }
    }
  }
  
  // Copy curvatures
  if (curvatures.size() >= points.size()) {
    gf.curvatures.resize(gf.numverts);
    for (size_t i = 0; i < curvatures.size() && i < points.size(); ++i) {
      for (int j = 0; j < 2; ++j) {
        gf.curvatures[i][j] = static_cast<float>(curvatures[i][j]);
      }
    }
  }
  
  // Copy functions
  if (functions.size() >= points.size()) {
    gf.funcs.resize(gf.numverts);
    for (size_t i = 0; i < functions.size() && i < points.size(); ++i) {
      gf.funcs[i][0] = static_cast<float>(functions[i]);
    }
  }
  
  // Encode element data based on what's present
  bool has_tets = !tets.empty();
  bool has_hexs = !hexs.empty();
  bool has_tris = !tris.empty();
  bool has_quads = !quads.empty();
  
  if (has_tets && !has_hexs) {
    // Encode tets as triangles
    gf.mesh_type = geoframe::TETRA;
    gf.numtris = tets.size() * 4;
    gf.triangles.resize(gf.numtris);
    
    for (size_t i = 0; i < tets.size(); ++i) {
      const auto& tet = tets[i];
      gf.triangles[i*4 + 0][0] = static_cast<unsigned int>(tet[0]);
      gf.triangles[i*4 + 0][1] = static_cast<unsigned int>(tet[1]);
      gf.triangles[i*4 + 0][2] = static_cast<unsigned int>(tet[2]);
      
      gf.triangles[i*4 + 1][0] = static_cast<unsigned int>(tet[0]);
      gf.triangles[i*4 + 1][1] = static_cast<unsigned int>(tet[1]);
      gf.triangles[i*4 + 1][2] = static_cast<unsigned int>(tet[3]);
      
      gf.triangles[i*4 + 2][0] = static_cast<unsigned int>(tet[1]);
      gf.triangles[i*4 + 2][1] = static_cast<unsigned int>(tet[2]);
      gf.triangles[i*4 + 2][2] = static_cast<unsigned int>(tet[3]);
      
      gf.triangles[i*4 + 3][0] = static_cast<unsigned int>(tet[0]);
      gf.triangles[i*4 + 3][1] = static_cast<unsigned int>(tet[2]);
      gf.triangles[i*4 + 3][2] = static_cast<unsigned int>(tet[3]);
    }
  }
  else if (has_hexs && !has_tets) {
    // Encode hexs as quads
    gf.mesh_type = geoframe::HEXA;
    gf.numquads = hexs.size() * 6;
    gf.quads.resize(gf.numquads);
    
    for (size_t i = 0; i < hexs.size(); ++i) {
      const auto& hex = hexs[i];
      gf.quads[i*6 + 0][0] = static_cast<unsigned int>(hex[0]);
      gf.quads[i*6 + 0][1] = static_cast<unsigned int>(hex[1]);
      gf.quads[i*6 + 0][2] = static_cast<unsigned int>(hex[2]);
      gf.quads[i*6 + 0][3] = static_cast<unsigned int>(hex[3]);
      
      gf.quads[i*6 + 1][0] = static_cast<unsigned int>(hex[4]);
      gf.quads[i*6 + 1][1] = static_cast<unsigned int>(hex[5]);
      gf.quads[i*6 + 1][2] = static_cast<unsigned int>(hex[6]);
      gf.quads[i*6 + 1][3] = static_cast<unsigned int>(hex[7]);
      
      gf.quads[i*6 + 2][0] = static_cast<unsigned int>(hex[0]);
      gf.quads[i*6 + 2][1] = static_cast<unsigned int>(hex[1]);
      gf.quads[i*6 + 2][2] = static_cast<unsigned int>(hex[5]);
      gf.quads[i*6 + 2][3] = static_cast<unsigned int>(hex[4]);
      
      gf.quads[i*6 + 3][0] = static_cast<unsigned int>(hex[1]);
      gf.quads[i*6 + 3][1] = static_cast<unsigned int>(hex[2]);
      gf.quads[i*6 + 3][2] = static_cast<unsigned int>(hex[6]);
      gf.quads[i*6 + 3][3] = static_cast<unsigned int>(hex[5]);
      
      gf.quads[i*6 + 4][0] = static_cast<unsigned int>(hex[2]);
      gf.quads[i*6 + 4][1] = static_cast<unsigned int>(hex[3]);
      gf.quads[i*6 + 4][2] = static_cast<unsigned int>(hex[7]);
      gf.quads[i*6 + 4][3] = static_cast<unsigned int>(hex[6]);
      
      gf.quads[i*6 + 5][0] = static_cast<unsigned int>(hex[3]);
      gf.quads[i*6 + 5][1] = static_cast<unsigned int>(hex[0]);
      gf.quads[i*6 + 5][2] = static_cast<unsigned int>(hex[4]);
      gf.quads[i*6 + 5][3] = static_cast<unsigned int>(hex[7]);
    }
  }
  else if (has_tris && !has_quads) {
    // Copy triangles
    gf.mesh_type = geoframe::SINGLE;
    gf.numtris = tris.size();
    gf.triangles.resize(gf.numtris);
    
    for (size_t i = 0; i < tris.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
        gf.triangles[i][j] = static_cast<unsigned int>(tris[i][j]);
      }
    }
  }
  else if (has_quads && !has_tris) {
    // Copy quads
    gf.mesh_type = geoframe::QUAD;
    gf.numquads = quads.size();
    gf.quads.resize(gf.numquads);
    
    for (size_t i = 0; i < quads.size(); ++i) {
      for (int j = 0; j < 4; ++j) {
        gf.quads[i][j] = static_cast<unsigned int>(quads[i][j]);
      }
    }
  }
  else {
    // Mixed or empty
    gf.mesh_type = geoframe::SINGLE;
    
    gf.numtris = tris.size();
    gf.triangles.resize(gf.numtris);
    for (size_t i = 0; i < tris.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
        gf.triangles[i][j] = static_cast<unsigned int>(tris[i][j]);
      }
    }
    
    gf.numquads = quads.size();
    gf.quads.resize(gf.numquads);
    for (size_t i = 0; i < quads.size(); ++i) {
      for (int j = 0; j < 4; ++j) {
        gf.quads[i][j] = static_cast<unsigned int>(quads[i][j]);
      }
    }
  }
  
  // Copy bounding box
  auto bbox = geom.extents();
  gf.min_x = bbox.minx;
  gf.min_y = bbox.miny;
  gf.min_z = bbox.minz;
  gf.max_x = bbox.maxx;
  gf.max_y = bbox.maxy;
  gf.max_z = bbox.maxz;
  
  gf.span[0] = gf.max_x - gf.min_x;
  gf.span[1] = gf.max_y - gf.min_y;
  gf.span[2] = gf.max_z - gf.min_z;
  
  return gf;
}

} // namespace LBIE
