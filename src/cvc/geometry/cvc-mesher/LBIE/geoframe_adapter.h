/******************************************************************************
				Copyright   

This code is developed within the Computational Visualization Center at The 
University of Texas at Austin.

This code has been made available to you under the auspices of a Lesser General 
Public License (LGPL) (http://www.ices.utexas.edu/cvc/software/license.html) 
and terms that you have agreed to.

Week 4 Modernization: geoframe_adapter
Purpose: Zero-copy adapter providing geoframe interface over CVC::geometry
This allows LBIE code to remain unchanged while using geometry at API boundaries.
Designed for future CUDA unified memory support.
******************************************************************************/

#ifndef __GEOFRAME_ADAPTER_H__
#define __GEOFRAME_ADAPTER_H__

#include "LBIE_geoframe.h"
#include <cvc/geometry/geometry.h>

namespace LBIE
{

// Forward declaration
class geoframe_adapter;

// ----------------------
// geoframe_adapter
// ----------------------
// Purpose:
//   Adapter providing geoframe interface over CVC::geometry with reduced copying.
//   Week 4: One-time copy on construction/destruction (50% reduction vs double conversion)
//   Week 5: Will use CUDA unified memory for true zero-copy via direct pointers
// 
// Current Limitations (Week 4):
//   - sync_from_geometry() copies geometry → geoframe (constructor)
//   - sync_to_geometry() copies geoframe → geometry (destructor)
//   - Cannot achieve true zero-copy without changing geoframe's owned std::vector members
//   - Still 50% better than previous double-conversion approach
//
// Design Goals:
//   1. Week 4: Minimize conversion overhead at API boundaries (one copy vs two)
//   2. Compatible with existing LBIE code (drop-in geoframe replacement)
//   3. Week 5: Support CUDA unified memory (geometry vectors can be GPU-accessible)
//   4. Week 5: True zero-copy via direct pointer access to unified memory
//
// Usage:
//   CVC::geometry geom;
//   geoframe_adapter adapter(geom);  // Copy geometry → geoframe
//   octree.mesh_extract(adapter, err);  // LBIE modifies adapter
//   // Destructor copies geoframe → geometry automatically
//
class geoframe_adapter : public geoframe
{
public:
  // Constructor wrapping a geometry object (reference semantics)
  explicit geoframe_adapter(cvc::geometry& geom);
  
  // Destructor - sync back to geometry
  ~geoframe_adapter();
  
  // Copy constructor - shares reference to same geometry
  geoframe_adapter(const geoframe_adapter& other);
  
  // Assignment - sync and redirect to new geometry
  geoframe_adapter& operator=(const geoframe_adapter& other);
  
  // Sync geometry data TO adapter (load from geometry)
  void sync_from_geometry();
  
  // Sync adapter data TO geometry (save to geometry)
  void sync_to_geometry();
  
  // Get underlying geometry
  cvc::geometry& get_geometry() { return _geom; }
  const cvc::geometry& get_geometry() const { return _geom; }
  
  // Override reset to also clear geometry
  void reset();
  
private:
  cvc::geometry& _geom;  // Reference to wrapped geometry
  bool _dirty;                     // Track if adapter modified (needs sync)
};

// Helper functions for conversion at API boundaries

// Create adapter from geometry (zero-copy reference)
inline geoframe_adapter make_adapter(cvc::geometry& geom) {
  return geoframe_adapter(geom);
}

// Convert geoframe to geometry (requires copy since geoframe owns data)
cvc::geometry to_geometry(const geoframe& gf);

// Convert geometry to geoframe (requires copy)
geoframe to_geoframe(const cvc::geometry& geom);

} // namespace LBIE

#endif // __GEOFRAME_ADAPTER_H__
