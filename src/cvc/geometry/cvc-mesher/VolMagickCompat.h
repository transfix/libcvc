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

#include <cvc/volume/volmagick.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/core/exception.h>

// Map old VolMagick namespace to CVC types
namespace VolMagick
{
  // Type aliases - old VolMagick types now use CVC implementations
  using Volume = cvc::volume;
  using Dimension = cvc::dimension;
  using BoundingBox = cvc::bounding_box;
  using VoxelType = cvc::data_type;
  
  // VoxelType enum values
  using cvc::UChar;
  using cvc::UShort;
  using cvc::UInt;
  using cvc::Float;
  using cvc::Double;
  using cvc::UInt64;
  
  // Function aliases
  using cvc::readVolumeFile;
  using cvc::writeVolumeFile;
  using cvc::createVolumeFile;
  
  // Exception alias
  using Exception = cvc::exception;
  
  // Define old exception types as CVC exceptions
  CVC_DEF_EXCEPTION(InvalidBoundingBox);
}

// Define VOLMAGICK_DEF_EXCEPTION for compatibility
#define VOLMAGICK_DEF_EXCEPTION CVC_DEF_EXCEPTION

#endif // __VOLMAGICK_COMPAT_H__
