/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cstdio>
#include <cstdlib>
#include <cvc/app.h>
#include <cvc/endians.h>
#include <cvc/volmagick.h>
#include <errno.h>
#include <format>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#ifdef __WINDOWS__
#define SNPRINTF _snprintf
#define FSEEK fseek
#else
#define SNPRINTF snprintf
#define FSEEK fseeko
#endif

using namespace std;

static inline void geterrstr(int errnum, char *strerrbuf, size_t buflen) {
#ifdef HAVE_STRERROR_R
  strerror_r(errnum, strerrbuf, buflen);
#else
  SNPRINTF(strerrbuf, buflen, "%s",
           strerror(errnum)); /* hopefully this is thread-safe on the target system! */
#endif
}

struct rawiv_header {
  float min[3];
  float max[3];
  unsigned int numVerts;
  unsigned int numCells;
  unsigned int dim[3];
  float origin[3];
  float span[3];
} rawivHeader;

namespace CVC_NAMESPACE {
CVC_DEF_EXCEPTION(invalid_rawiv_header);

// --------
// rawiv_io
// --------
// Purpose:
//   Provides RawIV file support.
// ---- Change History ----
// 11/13/2009 -- Joe R. -- Initially implemented.
struct rawiv_io : public volume_file_io {
  // ------------------
  // rawiv_io::rawiv_io
  // ------------------
  // Purpose:
  //   Initializes the extension list and id. If strict span is true,
  //   rawiv_io will reject rawiv files with incorrectly calculated spans.
  //   Many older rawiv files still have miscalculated spans due to errors
  //   in old programs.
  // ---- Change History ----
  // 11/13/2009 -- Joe R. -- Creation.
  rawiv_io(bool strictSpan = false) : _id("rawiv_io : v1.0"), _strictSpan(strictSpan) {
    _extensions.push_back(".rawiv");
  }

  // ------------
  // rawiv_io::id
  // ------------
  // Purpose:
  //   Returns a string that identifies this volume_file_io object.  This should
  //   be unique, but is freeform.
  // ---- Change History ----
  // 11/13/2009 -- Joe R. -- Creation.
  virtual const std::string &id() const { return _id; }

  // --------------------
  // rawiv_io::extensions
  // --------------------
  // Purpose:
  //   Returns a list of extensions that this volume_file_io object supports.
  // ---- Change History ----
  // 11/13/2009 -- Joe R. -- Creation.
  virtual const extension_list &extensions() const { return _extensions; }

  // ---------------------------
  // rawiv_io::getVolumeFileInfo
  // ---------------------------
  // Purpose:
  //   Writes to a structure containing all info that VolMagick needs
  //   from a volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/13/2009 -- Joe R. -- Converted to a volume_file_io class.
  virtual void getVolumeFileInfo(app & /*ctx*/, volume_file_info::data &data,
                                 const std::string &filename) const {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    char buf[256];
    rawiv_header rawivHeader;

    FILE *input;
    size_t i;
    data_type vt;

    memset(buf, 0, 256);

    if ((input = fopen(filename.c_str(), "rb")) == NULL) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error opening file '" + filename + "': " + buf;
      throw read_error(errStr);
    }

    if (fread(&rawivHeader, sizeof(rawivHeader), 1, input) != 1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error reading file '" + filename + "': " + buf;
      fclose(input);
      throw read_error(errStr);
    }

    if (!big_endian()) {
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.min[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.max[i]));
      SWAP_32(&(rawivHeader.numVerts));
      SWAP_32(&(rawivHeader.numCells));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.dim[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.origin[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.span[i]));
    }

    struct stat s;
    if (stat(filename.c_str(), &s) == -1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error stat-ing file '" + filename + "': " + buf;
      fclose(input);
      throw read_error(errStr);
    }

    /* error checking */
    if (rawivHeader.dim[0] == 0 || rawivHeader.dim[1] == 0 || rawivHeader.dim[2] == 0) {
      fclose(input);
      throw invalid_rawiv_header("Dimension values cannot be zero");
    }
    if (rawivHeader.numVerts != (rawivHeader.dim[0] * rawivHeader.dim[1] * rawivHeader.dim[2])) {
      fclose(input);
      throw invalid_rawiv_header("Number of vertices does not match calculated value");
    }
    if (rawivHeader.numCells !=
        ((rawivHeader.dim[0] - 1) * (rawivHeader.dim[1] - 1) * (rawivHeader.dim[2] - 1))) {
      fclose(input);
      throw invalid_rawiv_header("Number of cells does not match calculated value");
    }

    if (_strictSpan)
      for (i = 0; i < 3; i++)
        if (fabs(rawivHeader.span[i] -
                 ((rawivHeader.max[i] - rawivHeader.min[i]) / (rawivHeader.dim[i] - 1))) > 0.01f) {
          SNPRINTF(buf, 256, "Span value (%f) does not match calculation (%f)", rawivHeader.span[i],
                   ((rawivHeader.max[i] - rawivHeader.min[i]) / (rawivHeader.dim[i] - 1)));
          fclose(input);
          throw invalid_rawiv_header(buf);
        }

    /* get voxel type */
    uint64 voxelTypeSize =
        uint64(uint64(s.st_size - 68) / (uint64(rawivHeader.dim[0]) * uint64(rawivHeader.dim[1]) *
                                         uint64(rawivHeader.dim[2])));
    switch (voxelTypeSize) {
    case 1:
      vt = UChar;
      break;
    case 2:
      vt = UShort;
      break;
    case 4:
      vt = Float;
      break;
    case 8:
      vt = Double;
      break;
    default:
      fclose(input);
      SNPRINTF(buf, 256, "Cannot determine voxel type (size %lld)",
               (long long unsigned int)voxelTypeSize);
      throw invalid_rawiv_header(buf);
    }

    if (uint64(s.st_size - 68) != (uint64(rawivHeader.dim[0]) * uint64(rawivHeader.dim[1]) *
                                   uint64(rawivHeader.dim[2]) * uint64(data_type_sizes[vt]))) {
      SNPRINTF(buf, 256,
               "Volume dimensions do not match filesize. (file size: %lld, calculated size: %lld)",
               (long long unsigned int)uint64(s.st_size),
               (long long unsigned int)(uint64(rawivHeader.dim[0]) * uint64(rawivHeader.dim[1]) *
                                            uint64(rawivHeader.dim[2]) *
                                            uint64(data_type_sizes[vt]) +
                                        68));
      fclose(input);
      throw invalid_rawiv_header(buf);
    }

    /**** at this point, the volume header has no errors, so we may start modifiying this object
     * ****/
    data._filename = filename;
    data._numVariables = 1;
    data._numTimesteps = 1;
    data._tmin = data._tmax = 0.0;

    data._dimension = dimension(rawivHeader.dim);
    data._boundingBox = bounding_box(rawivHeader.min[0], rawivHeader.min[1], rawivHeader.min[2],
                                     rawivHeader.max[0], rawivHeader.max[1], rawivHeader.max[2]);

    data._voxelTypes.clear();
    data._voxelTypes.push_back(vt);
    data._names.clear();
    data._names.push_back("No Name");

    /* new volume, so min/max is now unset */

    data._minIsSet.clear();
    data._minIsSet.resize(data._numVariables);
    for (i = 0; i < data._minIsSet.size(); i++)
      data._minIsSet[i].resize(data._numTimesteps);
    data._min.clear();
    data._min.resize(data._numVariables);
    for (i = 0; i < data._min.size(); i++)
      data._min[i].resize(data._numTimesteps);
    data._maxIsSet.clear();
    data._maxIsSet.resize(data._numVariables);
    for (i = 0; i < data._maxIsSet.size(); i++)
      data._maxIsSet[i].resize(data._numTimesteps);
    data._max.clear();
    data._max.resize(data._numVariables);
    for (i = 0; i < data._max.size(); i++)
      data._max[i].resize(data._numTimesteps);

    /*
       Check if min/max is available.

       Here we use an extension to the rawiv format if it's available.  Since the origin values are
       not really used, we can use them to encode the min and max density values.  If the origin X
       value has the bytes 0xBAADBEEF, then origin y is the minimum density value and origin z is
       the maximum.
    */
    unsigned int *tmp_int =
        (unsigned int *)(&(rawivHeader.origin[0])); /* use a pointer so the compiler doesn't try to
                                                       convert the float value to int */
    if (*tmp_int == 0xBAADBEEF) {
      data._min[0][0] = rawivHeader.origin[1];
      data._max[0][0] = rawivHeader.origin[2];
      data._minIsSet[0][0] = data._maxIsSet[0][0] = true;
    }
    fclose(input);
  }

  // ------------------------
  // rawiv_io::readVolumeFile
  // ------------------------
  // Purpose:
  //   Writes to a Volume object after reading from a volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/13/2009 -- Joe R. -- Converted to a volume_file_io class
  virtual void readVolumeFile(app & /*ctx*/, volume &vol, const std::string &filename,
                              unsigned int var, unsigned int time, uint64 off_x, uint64 off_y,
                              uint64 off_z, const dimension &subvoldim) const {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    char buf[256];
    rawiv_header rawivHeader;

    FILE *input;
    size_t i, j, k;
    data_type vt;

    memset(buf, 0, 256);

    if (var > 0)
      throw index_out_of_bounds("Variable index out of bounds.");
    if (time > 0)
      throw index_out_of_bounds("Timestep index out of bounds.");
    if (subvoldim.isNull())
      throw index_out_of_bounds("Specified subvolume dimension is null.");

    if ((input = fopen(filename.c_str(), "rb")) == NULL) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error opening file '" + filename + "': " + buf;
      throw read_error(errStr);
    }

    if (fread(&rawivHeader, sizeof(rawivHeader), 1, input) != 1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error reading file '" + filename + "': " + buf;
      fclose(input);
      throw read_error(errStr);
    }

    if (!big_endian()) {
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.min[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.max[i]));
      SWAP_32(&(rawivHeader.numVerts));
      SWAP_32(&(rawivHeader.numCells));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.dim[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.origin[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.span[i]));
    }

    struct stat s;
    stat(filename.c_str(), &s);

    /* error checking */
    if (rawivHeader.dim[0] == 0 || rawivHeader.dim[1] == 0 || rawivHeader.dim[2] == 0) {
      fclose(input);
      throw invalid_rawiv_header("Dimension values cannot be zero");
    }
    if (rawivHeader.numVerts != (rawivHeader.dim[0] * rawivHeader.dim[1] * rawivHeader.dim[2])) {
      fclose(input);
      throw invalid_rawiv_header("Number of vertices does not match calculated value");
    }
    if (rawivHeader.numCells !=
        ((rawivHeader.dim[0] - 1) * (rawivHeader.dim[1] - 1) * (rawivHeader.dim[2] - 1))) {
      fclose(input);
      throw invalid_rawiv_header("Number of cells does not match calculated value");
    }

    if (_strictSpan)
      for (i = 0; i < 3; i++)
        if (fabs(rawivHeader.span[i] -
                 ((rawivHeader.max[i] - rawivHeader.min[i]) / (rawivHeader.dim[i] - 1))) > 0.01f) {
          SNPRINTF(buf, 256, "Span value (%f) does not match calculation (%f)", rawivHeader.span[i],
                   ((rawivHeader.max[i] - rawivHeader.min[i]) / (rawivHeader.dim[i] - 1)));
          fclose(input);
          throw invalid_rawiv_header(buf);
        }

    /* get voxel type */
    uint64 voxelTypeSize =
        uint64(uint64(s.st_size - 68) / (uint64(rawivHeader.dim[0]) * uint64(rawivHeader.dim[1]) *
                                         uint64(rawivHeader.dim[2])));
    switch (voxelTypeSize) {
    case 1:
      vt = UChar;
      break;
    case 2:
      vt = UShort;
      break;
    case 4:
      vt = Float;
      break;
    case 8:
      vt = Double;
      break;
    default:
      fclose(input);
      SNPRINTF(buf, 256, "Cannot determine voxel type (size %lld)",
               (long long unsigned int)voxelTypeSize);
      throw invalid_rawiv_header(buf);
    }

    if (uint64(s.st_size - 68) != (uint64(rawivHeader.dim[0]) * uint64(rawivHeader.dim[1]) *
                                   uint64(rawivHeader.dim[2]) * uint64(data_type_sizes[vt]))) {
      SNPRINTF(buf, 256,
               "Volume dimensions do not match filesize. (file size: %lld, calculated size: %lld)",
               (long long unsigned int)uint64(s.st_size),
               (long long unsigned int)(uint64(rawivHeader.dim[0]) * uint64(rawivHeader.dim[1]) *
                                            uint64(rawivHeader.dim[2]) *
                                            uint64(data_type_sizes[vt]) +
                                        68));
      fclose(input);
      throw invalid_rawiv_header(buf);
    }

    if ((off_x + subvoldim[0] - 1 >= rawivHeader.dim[0]) ||
        (off_y + subvoldim[1] - 1 >= rawivHeader.dim[1]) ||
        (off_z + subvoldim[2] - 1 >= rawivHeader.dim[2])) {
      fclose(input);
      throw index_out_of_bounds("Subvolume specified is outside volume dimensions");
    }

    /**** all errors have been checked for, at this point the rawiv file should be valid... we may
     * now write to 'vol' ****/
    try {
      vol.voxelType(vt);

      /*
         Check if min/max is available.

         Here we use an extension to the rawiv format if it's available.  Since the origin values
         are not really used, we can use them to encode the min and max density values.  If the
         origin X value has the bytes 0xBAADBEEF, then origin y is the minimum density value and
         origin z is the maximum.
      */
      unsigned int *tmp_int =
          (unsigned int *)(&(rawivHeader.origin[0])); /* use a pointer so the compiler doesn't try
                                                         to convert the float value to int */
      if (*tmp_int == 0xBAADBEEF) {
        vol.min(rawivHeader.origin[1]);
        vol.max(rawivHeader.origin[2]);
      }

      vol.voxel_dimensions(subvoldim);
      bounding_box subvolbox;
      // sometimes the span value is slightly incorrect in some headers, so lets calculate it
      // ourselves
      double span[3] = {(rawivHeader.max[0] - rawivHeader.min[0]) / double(rawivHeader.dim[0] - 1),
                        (rawivHeader.max[1] - rawivHeader.min[1]) / double(rawivHeader.dim[1] - 1),
                        (rawivHeader.max[2] - rawivHeader.min[2]) / double(rawivHeader.dim[2] - 1)};
      subvolbox.setMin(rawivHeader.min[0] + span[0] * double(off_x),
                       rawivHeader.min[1] + span[1] * double(off_y),
                       rawivHeader.min[2] + span[2] * double(off_z));
      subvolbox.setMax(rawivHeader.min[0] + span[0] * double(off_x + subvoldim[0] - 1),
                       rawivHeader.min[1] + span[1] * double(off_y + subvoldim[1] - 1),
                       rawivHeader.min[2] + span[2] * double(off_z + subvoldim[2] - 1));
      vol.boundingBox(subvolbox);
    } catch (memory_allocation_error &e) {
      fclose(input);
      throw e;
    }

    /*
      read the volume data
    */
    off_t file_offx, file_offy, file_offz;
    for (k = off_z; k <= (off_z + subvoldim[2] - 1); k++) {
      file_offz = 68 + k * rawivHeader.dim[0] * rawivHeader.dim[1] * vol.voxelSize();
      for (j = off_y; j <= (off_y + subvoldim[1] - 1); j++) {
        file_offy = j * rawivHeader.dim[0] * vol.voxelSize();
        file_offx = off_x * vol.voxelSize();
        // seek and read a scanline at a time
        if (FSEEK(input, file_offx + file_offy + file_offz, SEEK_SET) == -1) {
          geterrstr(errno, buf, 256);
          std::string errStr = "Error reading volume data in file '" + filename + "': " + buf;
          fclose(input);
          throw read_error(errStr);
        }
        if (fread(*vol + (k - off_z) * vol.XDim() * vol.YDim() * vol.voxelSize() +
                      (j - off_y) * vol.XDim() * vol.voxelSize(),
                  vol.voxelSize(), vol.XDim(), input) != vol.XDim()) {
          geterrstr(errno, buf, 256);
          std::string errStr = "Error reading volume data in file '" + filename + "': " + buf;
          fclose(input);
          throw read_error(errStr);
        }
      }
    }

    /* swap the volume data if on little endian machine */
    if (!big_endian()) {
      size_t len = vol.XDim() * vol.YDim() * vol.ZDim();
      switch (vol.voxelType()) {
      case UShort:
        for (i = 0; i < len; i++)
          SWAP_16(*vol + i * vol.voxelSize());
        break;
      case Float:
        for (i = 0; i < len; i++)
          SWAP_32(*vol + i * vol.voxelSize());
        break;
      case Double:
        for (i = 0; i < len; i++)
          SWAP_64(*vol + i * vol.voxelSize());
        break;
      default:
        break; /* no swapping needed for unsigned char data, and unsigned int is not defined for
                  rawiv */
      }
    }

    fclose(input);
  }

  // --------------------------
  // rawiv_io::createVolumeFile
  // --------------------------
  // Purpose:
  //   Creates an empty volume file to be later filled in by writeVolumeFile
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/13/2009 -- Joe R. -- Converted to a volume_file_io class
  virtual void createVolumeFile(app & /*ctx*/, const std::string &filename,
                                const bounding_box &boundingBox, const dimension &dimension,
                                const std::vector<data_type> &voxelTypes, unsigned int numVariables,
                                unsigned int numTimesteps, double min_time, double max_time) const {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    char buf[256];
    rawiv_header rawivHeader;

    FILE *output;
    size_t i, j, k;

    memset(buf, 0, 256);

    if (boundingBox.isNull())
      throw invalid_bounding_box("Bounding box must not be null");
    if (dimension.isNull())
      throw invalid_bounding_box("Dimension must not be null");
    if (numVariables > 1) {
      throw invalid_rawiv_header(
          std::format("RawIV format only supports 1 variable ({} requested)", numVariables));
    }
    if (numTimesteps > 1) {
      throw invalid_rawiv_header(
          std::format("RawIV format only supports 1 timestep ({} requested)", numTimesteps));
    }
    if (voxelTypes.size() > 1)
      throw invalid_rawiv_header(
          "RawIV format only supports 1 variable and 1 timestep. (too many voxel types specified)");
    if (min_time != max_time)
      throw invalid_rawiv_header("RawIV format does not support multiple timesteps. (min time and "
                                 "max time must be the same)");

    memset(&rawivHeader, 0, sizeof(rawiv_header));

    rawivHeader.min[0] = boundingBox.minx;
    rawivHeader.min[1] = boundingBox.miny;
    rawivHeader.min[2] = boundingBox.minz;
    rawivHeader.max[0] = boundingBox.maxx;
    rawivHeader.max[1] = boundingBox.maxy;
    rawivHeader.max[2] = boundingBox.maxz;
    rawivHeader.numVerts = dimension[0] * dimension[1] * dimension[2];
    rawivHeader.numCells = (dimension[0] - 1) * (dimension[1] - 1) * (dimension[2] - 1);
    rawivHeader.dim[0] = dimension[0];
    rawivHeader.dim[1] = dimension[1];
    rawivHeader.dim[2] = dimension[2];
    rawivHeader.span[0] = (boundingBox.maxx - boundingBox.minx) / (dimension[0] - 1);
    rawivHeader.span[1] = (boundingBox.maxy - boundingBox.miny) / (dimension[1] - 1);
    rawivHeader.span[2] = (boundingBox.maxz - boundingBox.minz) / (dimension[2] - 1);

    /* write extended rawiv header volumes in which min and max voxel values are encoded in the
     * origin values */
    /* use a pointer so the compiler doesn't try to convert the float value to int */
    unsigned int *tmp_int = (unsigned int *)(&(rawivHeader.origin[0]));
    *tmp_int = 0xBAADBEEF;
    rawivHeader.origin[1] = 0.0;
    rawivHeader.origin[2] = 0.0;

    if (!big_endian()) {
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.min[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.max[i]));
      SWAP_32(&(rawivHeader.numVerts));
      SWAP_32(&(rawivHeader.numCells));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.dim[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.origin[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.span[i]));
    }

    if ((output = fopen(filename.c_str(), "wb")) == NULL) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error opening file '" + filename + "': " + buf;
      throw write_error(errStr);
    }

    if (fwrite(&rawivHeader, sizeof(rawivHeader), 1, output) != 1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error writing header to file '" + filename + "': " + buf;
      fclose(output);
      throw write_error(errStr);
    }

    unsigned char *scanline =
        (unsigned char *)calloc(dimension[0] * data_type_sizes[voxelTypes[0]], 1);
    if (scanline == NULL) {
      fclose(output);
      throw memory_allocation_error("Unable to allocate memory for write buffer");
    }

    // write a scanline at a time
    for (k = 0; k < dimension[2]; k++)
      for (j = 0; j < dimension[1]; j++) {
        if (fwrite(scanline, data_type_sizes[voxelTypes[0]], dimension[0], output) !=
            dimension[0]) {
          geterrstr(errno, buf, 256);
          std::string errStr = "Error writing volume data to file '" + filename + "': " + buf;
          free(scanline);
          fclose(output);
          throw write_error(errStr);
        }
      }
    free(scanline);
    fclose(output);
  }

  // -------------------------
  // rawiv_io::writeVolumeFile
  // -------------------------
  // Purpose:
  //   Writes the volume contained in wvol to the specified volume file. Should create
  //   a volume file if the filename provided doesn't exist.  Else it will simply
  //   write data to the existing file.  A common user error arises when you try to
  //   write over an existing volume file using this function for unrelated volumes.
  //   If what you desire is to overwrite an existing volume file, first run
  //   createVolumeFile to replace the volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/13/2009 -- Joe R. -- Converted to a volume_file_io class
  virtual void writeVolumeFile(app &ctx, const volume &wvol, const std::string &filename,
                               unsigned int var, unsigned int time, uint64 off_x, uint64 off_y,
                               uint64 off_z) const {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    volume_file_info volinfo;
    char buf[256];
    rawiv_header rawivHeader;
    bool creatingNewFile = false;

    FILE *output;
    size_t i, j, k;

    uint64 outvol_xdim, outvol_ydim, outvol_zdim;

    memset(buf, 0, 256);

    if (var > 0)
      throw index_out_of_bounds("Variable index out of bounds.");
    if (time > 0)
      throw index_out_of_bounds("Timestep index out of bounds.");

    volume vol(wvol);

    // check if the file exists and we can write the specified subvolume to it
    try {
      volinfo.read(filename);

      using namespace std;

      // if(!(Dimension(off_x+vol.XDim(),off_y+vol.YDim(),off_z+vol.ZDim()) <=
      // volinfo.voxel_dimensions()))
      if (off_x + vol.XDim() > volinfo.voxel_dimensions()[0] &&
          off_y + vol.YDim() > volinfo.voxel_dimensions()[1] &&
          off_z + vol.ZDim() > volinfo.voxel_dimensions()[2]) {
        std::string errStr =
            "File '" + filename + "' exists but is too small to write volume at specified offset";
        throw index_out_of_bounds(errStr);
      }
      vol.voxelType(volinfo.voxelType()); // change the volume's voxel type to match that of the
                                          // file

    } catch (read_error &e) {
      // create a blank file since file doesn't exist (or there was an error reading the existing
      // file)
      bounding_box box(vol.boundingBox());
      box.minx -= off_x * vol.XSpan();
      box.miny -= off_y * vol.YSpan();
      box.minz -= off_z * vol.ZSpan();
      dimension dim(vol.voxel_dimensions());
      dim[0] += off_x;
      dim[1] += off_y;
      dim[2] += off_z;
      CVC_NAMESPACE::createVolumeFile(ctx, filename, box, dim,
                                      std::vector<data_type>(1, vol.voxelType()), 1, 1, 0.0, 0.0);
      volinfo.read(filename);

      if (var >= volinfo.numVariables()) {
        std::string errStr =
            "Variable index exceeds number of variables in file '" + filename + "'";
        throw index_out_of_bounds(errStr);
      }
      if (time >= volinfo.numTimesteps()) {
        std::string errStr =
            "Timestep index exceeds number of timesteps in file '" + filename + "'";
        throw index_out_of_bounds(errStr);
      }

      creatingNewFile = true;
    }

    if ((output = fopen(filename.c_str(), "r+b")) == NULL) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error opening file '" + filename + "': " + buf;
      throw write_error(errStr);
    }

    // read the header to set the min/max values for rawiv ext.
    if (fread(&rawivHeader, sizeof(rawivHeader), 1, output) != 1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error reading file '" + filename + "': " + buf;
      fclose(output);
      throw read_error(errStr);
    }

    if (!big_endian()) {
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.origin[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.dim[i]));
    }

    /* write extended rawiv header volumes in which min and max voxel values are encoded in the
     * origin values */
    /* use a pointer so the compiler doesn't try to convert the float value to int */
    unsigned int *tmp_int = (unsigned int *)(&(rawivHeader.origin[0]));
    // if(*tmp_int != 0xBAADBEEF) //if the header doesn't have a min/max extension, write one
    {
      *tmp_int = 0xBAADBEEF;

      // FIX: arand, I think this code messes up the min/max values
      //      in the header when using volconvert
      if (creatingNewFile) // the blank file is zero filled...
      {
        rawivHeader.origin[1] = MIN(0.0, vol.min());
        rawivHeader.origin[2] = MAX(0.0, vol.max());
      } else {
        using namespace std;

        rawivHeader.origin[1] = MIN(volinfo.min(), vol.min());
        rawivHeader.origin[2] = MAX(volinfo.max(), vol.max());
      }
    }

    outvol_xdim = rawivHeader.dim[0];
    outvol_ydim = rawivHeader.dim[1];
    outvol_zdim = rawivHeader.dim[2];

    if (!big_endian()) {
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.origin[i]));
      for (i = 0; i < 3; i++)
        SWAP_32(&(rawivHeader.dim[i]));
    }

    if (FSEEK(output, 0, SEEK_SET) == -1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error seeking in file '" + filename + "': " + buf;
      fclose(output);
      throw read_error(errStr);
    }

    if (fwrite(&rawivHeader, sizeof(rawivHeader), 1, output) != 1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error writing header to file '" + filename + "': " + buf;
      fclose(output);
      throw write_error(errStr);
    }

    unsigned char *scanline = (unsigned char *)malloc(vol.XDim() * vol.voxelSize());
    if (scanline == NULL) {
      fclose(output);
      throw memory_allocation_error("Unable to allocate memory for write buffer");
    }

    /*
      write the volume data
    */
    off_t file_offx, file_offy, file_offz;
    for (k = off_z; k <= (off_z + vol.ZDim() - 1); k++) {
      file_offz = 68 + k * outvol_xdim * outvol_ydim * vol.voxelSize();
      for (j = off_y; j <= (off_y + vol.YDim() - 1); j++) {
        file_offy = j * outvol_xdim * vol.voxelSize();
        file_offx = off_x * vol.voxelSize();

        // seek and write a scanline at a time
        if (FSEEK(output, file_offx + file_offy + file_offz, SEEK_SET) == -1) {
          geterrstr(errno, buf, 256);
          std::string errStr = "Error seeking in file '" + filename + "': " + buf;
          fclose(output);
          throw read_error(errStr);
        }

        memcpy(scanline,
               *vol + ((k - off_z) * vol.XDim() * vol.YDim() * vol.voxelSize()) +
                   ((j - off_y) * vol.XDim() * vol.voxelSize()),
               vol.XDim() * vol.voxelSize());

        /* swap the volume data if on little endian machine */
        if (!big_endian()) {
          size_t len = vol.XDim();
          switch (vol.voxelType()) {
          case UShort:
            for (i = 0; i < len; i++)
              SWAP_16(scanline + i * vol.voxelSize());
            break;
          case Float:
            for (i = 0; i < len; i++)
              SWAP_32(scanline + i * vol.voxelSize());
            break;
          case Double:
            for (i = 0; i < len; i++)
              SWAP_64(scanline + i * vol.voxelSize());
            break;
          default:
            break; /* no swapping needed for unsigned char data, and unsigned int is not defined for
                      rawiv */
          }
        }

        if (fwrite(scanline, vol.voxelSize(), vol.XDim(), output) != vol.XDim()) {
          geterrstr(errno, buf, 256);
          std::string errStr = "Error writing volume data to file '" + filename + "': " + buf;
          free(scanline);
          fclose(output);
          throw write_error(errStr);
        }
      }
    }

    free(scanline);

    // fix a bug with min/max values in header... if we use this function to replace a whole
    // volume's set of voxels, the min/max calculation above can be wrong!

    // arand: this code is supposed to correct the min/max values but actually is causing problems.
#if 0
      if (0)
	{
	  volume minmax_vol;
	  VolMagick::readVolumeFile(minmax_vol,volinfo.filename());
	  minmax_vol.unsetMinMax();
	  float realminmax[2];
	  realminmax[0] = minmax_vol.min();
	  realminmax[1] = minmax_vol.max();    

	  ptrdiff_t offset = (unsigned char*)(&(rawivHeader.origin[1])) - (unsigned char *)(&(rawivHeader.min[0]));

	  if(FSEEK(output,offset,SEEK_SET) == -1)
	    {
	      geterrstr(errno,buf,256);
	      std::string errStr = "Error seeking in file '" + filename + "': " + buf;
	      fclose(output);
	      throw read_error(errStr);
	    }

	  if(!big_endian())
	    {
	      SWAP_32(&realminmax[0]);
	      SWAP_32(&realminmax[1]);
	    }
      
	  if(fwrite(realminmax,sizeof(float),2,output) != 2)
	    {
	      geterrstr(errno,buf,256);
	      std::string errStr = "Error writing min/max data to file '" + filename + "': " + buf;
	      fclose(output);
	      throw write_error(errStr);
	    }
	  std::cout<<"rawiv_io 820: min max" << realminmax[0] <<" " << realminmax[1] << std::endl;

	}
      fclose(output);
#endif
  }

protected:
  std::string _id;
  extension_list _extensions;
  bool _strictSpan;
};
} // namespace CVC_NAMESPACE

namespace CVC_NAMESPACE {
void register_rawiv_io(app & /*ctx*/) {
  volume_file_io::insertHandler(volume_file_io::ptr(new rawiv_io));
}
} // namespace CVC_NAMESPACE
