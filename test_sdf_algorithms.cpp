/*
 * Test program to verify both SDF v1 and v2 algorithms work correctly
 */

#include <cvc/volmagick.h>
#include <cvc/algorithm.h>
#include <cvc/geometry.h>
#include <iostream>

using namespace cvc;

int main(int argc, char** argv)
{
  try {
    std::cout << "Testing SDF Algorithm Switching\n";
    std::cout << "================================\n\n";
    
    // Create a simple test geometry (cube)
    geometry cube;
    
    // Define 8 vertices of a unit cube centered at origin
    cube.points().resize(8);
    cube.points()[0] = geometry::point_t(-0.5, -0.5, -0.5);
    cube.points()[1] = geometry::point_t( 0.5, -0.5, -0.5);
    cube.points()[2] = geometry::point_t( 0.5,  0.5, -0.5);
    cube.points()[3] = geometry::point_t(-0.5,  0.5, -0.5);
    cube.points()[4] = geometry::point_t(-0.5, -0.5,  0.5);
    cube.points()[5] = geometry::point_t( 0.5, -0.5,  0.5);
    cube.points()[6] = geometry::point_t( 0.5,  0.5,  0.5);
    cube.points()[7] = geometry::point_t(-0.5,  0.5,  0.5);
    
    // Define 12 triangles (2 per face)
    cube.tris().resize(12);
    // Front face
    cube.tris()[0] = geometry::tri_t(0, 1, 2);
    cube.tris()[1] = geometry::tri_t(0, 2, 3);
    // Back face
    cube.tris()[2] = geometry::tri_t(4, 6, 5);
    cube.tris()[3] = geometry::tri_t(4, 7, 6);
    // Left face
    cube.tris()[4] = geometry::tri_t(0, 3, 7);
    cube.tris()[5] = geometry::tri_t(0, 7, 4);
    // Right face
    cube.tris()[6] = geometry::tri_t(1, 5, 6);
    cube.tris()[7] = geometry::tri_t(1, 6, 2);
    // Top face
    cube.tris()[8] = geometry::tri_t(3, 2, 6);
    cube.tris()[9] = geometry::tri_t(3, 6, 7);
    // Bottom face
    cube.tris()[10] = geometry::tri_t(0, 4, 5);
    cube.tris()[11] = geometry::tri_t(0, 5, 1);
    
    std::cout << "Created test cube geometry:\n";
    std::cout << "  Vertices: " << cube.num_points() << "\n";
    std::cout << "  Triangles: " << cube.num_tris() << "\n\n";
    
    // Define volume parameters
    dimension dim(32, 32, 32);
    bounding_box bbox(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
    
    std::cout << "Computing SDF with V1 algorithm...\n";
    volume sdf_v1 = sdf(cube, dim, bbox, SDF_V1);
    std::cout << "  Description: " << sdf_v1.desc() << "\n";
    std::cout << "  Min: " << sdf_v1.min() << "\n";
    std::cout << "  Max: " << sdf_v1.max() << "\n";
    std::cout << "  Dimensions: " << sdf_v1.XDim() << "x" 
              << sdf_v1.YDim() << "x" << sdf_v1.ZDim() << "\n\n";
    
    std::cout << "Computing SDF with V2 algorithm...\n";
    volume sdf_v2 = sdf(cube, dim, bbox, SDF_V2);
    std::cout << "  Description: " << sdf_v2.desc() << "\n";
    std::cout << "  Min: " << sdf_v2.min() << "\n";
    std::cout << "  Max: " << sdf_v2.max() << "\n";
    std::cout << "  Dimensions: " << sdf_v2.XDim() << "x" 
              << sdf_v2.YDim() << "x" << sdf_v2.ZDim() << "\n\n";
    
    // Save results for inspection
    if (argc > 1) {
      std::string prefix = argv[1];
      std::cout << "Saving results...\n";
      sdf_v1.write(prefix + "_sdf_v1.rawiv");
      sdf_v2.write(prefix + "_sdf_v2.rawiv");
      std::cout << "  Saved " << prefix << "_sdf_v1.rawiv\n";
      std::cout << "  Saved " << prefix << "_sdf_v2.rawiv\n";
    }
    
    std::cout << "\nSUCCESS: Both SDF algorithms completed successfully!\n";
    
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    return 1;
  }
}
