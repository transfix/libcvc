/*
  Copyright 2026 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cassert>
#include <cmath>
#include <cvc/algorithm.h>
#include <cvc/geometry.h>
#include <iostream>

using namespace CVC_NAMESPACE;

// Test sphere generation
void test_sphere() {
  std::cout << "\n=== Testing Sphere Generation ===" << std::endl;

  // Generate a unit sphere at origin
  geometry sphere = generate_sphere(0.0, 0.0, 0.0, 1.0, 16, 8);

  std::cout << "Sphere vertices: " << sphere.num_points() << std::endl;
  std::cout << "Sphere triangles: " << sphere.num_tris() << std::endl;
  std::cout << "Sphere normals: " << sphere.normals().size() << std::endl;

  // Verify counts
  int thetaRes = 16, phiRes = 8;
  int expectedVerts = 2 + (phiRes - 1) * thetaRes; // poles + middle rings
  int expectedTris =
      thetaRes + (phiRes - 2) * thetaRes * 2 + thetaRes; // top cap + middle + bottom cap

  assert(sphere.num_points() == expectedVerts);
  assert(sphere.num_tris() == expectedTris);
  assert(sphere.normals().size() == sphere.num_points());

  // Verify all vertices are on sphere surface (within tolerance)
  double tolerance = 1e-6;
  for (const auto &p : sphere.points()) {
    double dist = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    assert(std::abs(dist - 1.0) < tolerance);
  }

  // Verify normals are unit length
  for (const auto &n : sphere.normals()) {
    double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    assert(std::abs(len - 1.0) < tolerance);
  }

  std::cout << "✓ Sphere generation tests passed" << std::endl;
}

// Test cube generation
void test_cube() {
  std::cout << "\n=== Testing Cube Generation ===" << std::endl;

  // Generate a unit cube at origin
  geometry cube = generate_cube(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

  std::cout << "Cube vertices: " << cube.num_points() << std::endl;
  std::cout << "Cube triangles: " << cube.num_tris() << std::endl;
  std::cout << "Cube normals: " << cube.normals().size() << std::endl;

  // Verify counts (6 faces * 4 vertices per face = 24 vertices)
  // (6 faces * 2 triangles per face = 12 triangles)
  assert(cube.num_points() == 24);
  assert(cube.num_tris() == 12);
  assert(cube.normals().size() == 24);

  // Verify all vertices are on cube surface
  double tolerance = 1e-6;
  for (const auto &p : cube.points()) {
    // Each coordinate should be ±0.5
    assert(std::abs(std::abs(p[0]) - 0.5) < tolerance ||
           std::abs(std::abs(p[1]) - 0.5) < tolerance ||
           std::abs(std::abs(p[2]) - 0.5) < tolerance);
  }

  // Verify normals are unit length and axis-aligned
  for (const auto &n : cube.normals()) {
    double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    assert(std::abs(len - 1.0) < tolerance);

    // Each normal should be axis-aligned (one component is ±1, others are 0)
    int nonzero = 0;
    if (std::abs(std::abs(n[0]) - 1.0) < tolerance)
      nonzero++;
    if (std::abs(std::abs(n[1]) - 1.0) < tolerance)
      nonzero++;
    if (std::abs(std::abs(n[2]) - 1.0) < tolerance)
      nonzero++;
    assert(nonzero == 1);
  }

  std::cout << "✓ Cube generation tests passed" << std::endl;
}

// Test torus generation
void test_torus() {
  std::cout << "\n=== Testing Torus Generation ===" << std::endl;

  // Generate a torus at origin
  double majorRadius = 1.0, minorRadius = 0.25;
  int majorRes = 16, minorRes = 8;
  geometry torus = generate_torus(0.0, 0.0, 0.0, majorRadius, minorRadius, majorRes, minorRes);

  std::cout << "Torus vertices: " << torus.num_points() << std::endl;
  std::cout << "Torus triangles: " << torus.num_tris() << std::endl;
  std::cout << "Torus normals: " << torus.normals().size() << std::endl;

  // Verify counts
  int expectedVerts = majorRes * minorRes;
  int expectedTris = majorRes * minorRes * 2;

  assert(torus.num_points() == expectedVerts);
  assert(torus.num_tris() == expectedTris);
  assert(torus.normals().size() == torus.num_points());

  // Verify normals are unit length
  double tolerance = 1e-6;
  for (const auto &n : torus.normals()) {
    double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    assert(std::abs(len - 1.0) < tolerance);
  }

  // Verify vertex positions are valid (within torus bounds)
  for (const auto &p : torus.points()) {
    double r = std::sqrt(p[0] * p[0] + p[2] * p[2]);
    // Distance from tube center axis should be within range
    assert(r >= majorRadius - minorRadius - tolerance);
    assert(r <= majorRadius + minorRadius + tolerance);
  }

  std::cout << "✓ Torus generation tests passed" << std::endl;
}

// Test cone generation
void test_cone() {
  std::cout << "\n=== Testing Cone Generation ===" << std::endl;

  // Generate a cone at origin
  double radius = 0.5, height = 1.0;
  int res = 16;
  geometry cone = generate_cone(0.0, 0.0, 0.0, radius, height, res);

  std::cout << "Cone vertices: " << cone.num_points() << std::endl;
  std::cout << "Cone triangles: " << cone.num_tris() << std::endl;
  std::cout << "Cone normals: " << cone.normals().size() << std::endl;

  // Verify counts
  // 1 apex + res side vertices + 1 base center + res base ring vertices
  int expectedVerts = 1 + res + 1 + res;
  int expectedTris = res + res; // res side triangles + res base cap triangles

  assert(cone.num_points() == expectedVerts);
  assert(cone.num_tris() == expectedTris);
  assert(cone.normals().size() == cone.num_points());

  // Verify normals are unit length
  double tolerance = 1e-6;
  for (const auto &n : cone.normals()) {
    double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    assert(std::abs(len - 1.0) < tolerance);
  }

  // Verify apex is at expected position
  const auto &apex = cone.points()[0];
  assert(std::abs(apex[0] - 0.0) < tolerance);
  assert(std::abs(apex[1] - height / 2.0) < tolerance);
  assert(std::abs(apex[2] - 0.0) < tolerance);

  std::cout << "✓ Cone generation tests passed" << std::endl;
}

// Test different resolutions
void test_resolutions() {
  std::cout << "\n=== Testing Different Resolutions ===" << std::endl;

  // Low resolution sphere
  geometry low_sphere = generate_sphere(0, 0, 0, 1.0, 8, 4);
  std::cout << "Low-res sphere: " << low_sphere.num_points() << " vertices, "
            << low_sphere.num_tris() << " triangles" << std::endl;

  // High resolution sphere
  geometry high_sphere = generate_sphere(0, 0, 0, 1.0, 64, 32);
  std::cout << "High-res sphere: " << high_sphere.num_points() << " vertices, "
            << high_sphere.num_tris() << " triangles" << std::endl;

  assert(high_sphere.num_points() > low_sphere.num_points());
  assert(high_sphere.num_tris() > low_sphere.num_tris());

  std::cout << "✓ Resolution tests passed" << std::endl;
}

// Test non-unit sizes and positions
void test_transforms() {
  std::cout << "\n=== Testing Transforms ===" << std::endl;

  // Sphere at different position
  geometry sphere = generate_sphere(5.0, 10.0, -3.0, 2.5, 16, 8);

  // Check bounding box
  auto bbox = sphere.extents();
  std::cout << "Sphere bbox: [" << bbox[0] << ", " << bbox[3] << "] x [" << bbox[1] << ", "
            << bbox[4] << "] y [" << bbox[2] << ", " << bbox[5] << "]" << std::endl;

  double tolerance = 1e-5;
  assert(std::abs(bbox[0] - (5.0 - 2.5)) < tolerance);
  assert(std::abs(bbox[3] - (5.0 + 2.5)) < tolerance);
  assert(std::abs(bbox[1] - (10.0 - 2.5)) < tolerance);
  assert(std::abs(bbox[4] - (10.0 + 2.5)) < tolerance);

  // Non-uniform cube
  geometry cube = generate_cube(1.0, 2.0, 3.0, 4.0, 6.0, 8.0);
  bbox = cube.extents();
  std::cout << "Cube bbox: [" << bbox[0] << ", " << bbox[3] << "] x [" << bbox[1] << ", " << bbox[4]
            << "] y [" << bbox[2] << ", " << bbox[5] << "]" << std::endl;

  assert(std::abs(bbox[0] - (1.0 - 2.0)) < tolerance);
  assert(std::abs(bbox[3] - (1.0 + 2.0)) < tolerance);
  assert(std::abs(bbox[1] - (2.0 - 3.0)) < tolerance);
  assert(std::abs(bbox[4] - (2.0 + 3.0)) < tolerance);
  assert(std::abs(bbox[2] - (3.0 - 4.0)) < tolerance);
  assert(std::abs(bbox[5] - (3.0 + 4.0)) < tolerance);

  std::cout << "✓ Transform tests passed" << std::endl;
}

int main(int argc, char **argv) {
  std::cout << "====================================" << std::endl;
  std::cout << "Procedural Geometry Generation Tests" << std::endl;
  std::cout << "====================================" << std::endl;

  try {
    test_sphere();
    test_cube();
    test_torus();
    test_cone();
    test_resolutions();
    test_transforms();

    std::cout << "\n====================================" << std::endl;
    std::cout << "✓ All procedural geometry tests passed!" << std::endl;
    std::cout << "====================================" << std::endl;

    return 0;
  } catch (const std::exception &e) {
    std::cerr << "\n✗ Test failed with exception: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "\n✗ Test failed with unknown exception" << std::endl;
    return 1;
  }
}
