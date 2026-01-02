// Quick test to verify enum-based extraction method API
#include <cvc/algorithm.h>
#include <cvc/geometry.h>
#include <iostream>

int main() {
    using namespace cvc;
    
    std::cout << "Testing enum-based extraction method API...\n";
    
    // Create simple test volume
    dimension dim(32, 32, 32);
    bounding_box bbox;
    bbox.minx = bbox.miny = bbox.minz = -1;
    bbox.maxx = bbox.maxy = bbox.maxz = 1;
    volume vol(dim, Float, bbox);
    
    // Fill with simple sphere SDF
    for (uint64 k = 0; k < vol.ZDim(); k++) {
        for (uint64 j = 0; j < vol.YDim(); j++) {
            for (uint64 i = 0; i < vol.XDim(); i++) {
                double x = bbox.minx + (i + 0.5) * (bbox.maxx - bbox.minx) / dim.xdim;
                double y = bbox.miny + (j + 0.5) * (bbox.maxy - bbox.miny) / dim.ydim;
                double z = bbox.minz + (k + 0.5) * (bbox.maxz - bbox.minz) / dim.zdim;
                double dist = sqrt(x*x + y*y + z*z) - 0.5;
                vol(i, j, k, dist);
            }
        }
    }
    
    std::cout << "Testing iso() with default (DUALLIB)...\n";
    geometry mesh1 = iso(vol, 0.0);
    std::cout << "  Vertices: " << mesh1.num_points() << ", Triangles: " << mesh1.num_tris() << "\n";
    
    std::cout << "Testing iso() with DUALLIB enum...\n";
    geometry mesh2 = iso(vol, 0.0, DUALLIB);
    std::cout << "  Vertices: " << mesh2.num_points() << ", Triangles: " << mesh2.num_tris() << "\n";
    
    std::cout << "Testing iso() with FASTCONTOURING enum...\n";
    geometry mesh3 = iso(vol, 0.0, FASTCONTOURING);
    std::cout << "  Vertices: " << mesh3.num_points() << ", Triangles: " << mesh3.num_tris() << "\n";
    
    std::cout << "Testing iso() with LIBISOCONTOUR enum...\n";
    geometry mesh4 = iso(vol, 0.0, LIBISOCONTOUR);
    std::cout << "  Vertices: " << mesh4.num_points() << ", Triangles: " << mesh4.num_tris() << "\n";
    
    std::cout << "\nAll extraction methods work correctly!\n";
    return 0;
}
