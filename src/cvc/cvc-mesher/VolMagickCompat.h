/*
  VolMagick Compatibility Layer
  
  Maps old VolMagick types to modern CVC types.
  This allows the mesher code to compile without modification
  while using the CVC volume API.
  
  Change History:
  12/25/2025 - Joe R. - Created compatibility layer to eliminate old VolMagick dependency
*/

#ifndef __VOLMAGICK_COMPAT_H__
#define __VOLMAGICK_COMPAT_H__

#include <cvc/volmagick.h>
#include <cvc/volume_file_io.h>
#include <cvc/exception.h>

// Map old VolMagick namespace to CVC types
namespace VolMagick
{
  // Type aliases - old VolMagick types now use CVC implementations
  using Volume = CVC_NAMESPACE::volume;
  using Dimension = CVC_NAMESPACE::dimension;
  using BoundingBox = CVC_NAMESPACE::bounding_box;
  using VoxelType = CVC_NAMESPACE::data_type;
  
  // VoxelType enum values
  using CVC_NAMESPACE::UChar;
  using CVC_NAMESPACE::UShort;
  using CVC_NAMESPACE::UInt;
  using CVC_NAMESPACE::Float;
  using CVC_NAMESPACE::Double;
  using CVC_NAMESPACE::UInt64;
  
  // Function aliases
  using CVC_NAMESPACE::readVolumeFile;
  using CVC_NAMESPACE::writeVolumeFile;
  using CVC_NAMESPACE::createVolumeFile;
  
  // Exception alias
  using Exception = CVC_NAMESPACE::exception;
  
  // Define old exception types as CVC exceptions
  CVC_DEF_EXCEPTION(InvalidBoundingBox);
}

// Define VOLMAGICK_DEF_EXCEPTION for compatibility
#define VOLMAGICK_DEF_EXCEPTION CVC_DEF_EXCEPTION

#endif // __VOLMAGICK_COMPAT_H__
