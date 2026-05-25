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

#include <cvc/core/app.h>
#include <cvc/volume/endians.h>
#include <cvc/volume/volmagick.h>

#ifdef CVC_USING_IMOD_MRC
#include <iimage.h>
#endif

#include <boost/format.hpp>
#include <boost/scoped_array.hpp>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <errno.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>
#ifdef __SOLARIS__
#include <ieeefp.h>
#endif

using namespace std;

///\struct MrcHeader
///\brief The header of an MRC file.
typedef struct {

  //! # of columns ( fastest changing in the map
  int nx;
  //! # of rows
  int ny;
  //! # of sections (slowest changing in the map
  int nz;

  //! data type
  //! 0 = image data in bytes
  //! 1 = image data in short integer
  //! 2 = image data in floats
  //! 3 = complex data in complex short integers
  //! 4 = complex data in complex reals
  int mode;

  //! number of first column in map (default = 0)
  int nxstart;
  //! number of first row in map (default = 0)
  int nystart;
  //! number of first ssection in map (default = 0)
  int nzstart;

  //! number of intervals along X
  int mx;
  //! number of intervals along Y
  int my;
  //! number of intervals along Z
  int mz;

  //! cell dimensions in X (angstrom)
  float xlength;
  //! cell dimensions in Y (angstrom)
  float ylength;
  //! cell dimensions in Z (angstrom)
  float zlength;

  //! cell angles between Y and Z
  float alpha;
  //! cell angles between X and Z
  float beta;
  //! cell angles between X and Y
  float gamma;

  //! number of axis corresponding to columns (X)
  int mapc;
  //! number of axis corresponding to rows (Y)
  int mapr;
  //! number of axis corresponding to sections (Z)
  int maps;

  //! minimum density value
  float amin;
  //! maximum density value
  float amax;
  //! mean density value
  float amean;

  //! space group number (0 for images)
  int ispg;
  //! # of bytes for symmetry operators
  int nsymbt;

  //! user defined storage space
  int extra[25];

  //! X phase origin
  float xorigin;
  //! Y phase origin
  float yorigin;
  //! Z phase origin
  float zorigin;

  //! character string 'MAP '
  char map[4];

  //! machine stamp
  int machst;

  //! rms deviation of map from mean density
  float rms;

  //! # of labels being used in the MRC header
  int nlabl;

  //! actual text labels
  char label[10][80];

} mrc_header;

typedef struct {
  float aTilt;
  float bTilt;
  float xStage;
  float yStage;
  float zStage;
  float xShift;
  float yShift;
  float defocus;
  float expTime;
  float meanInt;
  float tiltAxis;
  float pixelSize;
  float imageMag;
  char filler[76];
} extended_mrc_header;

#ifdef __WINDOWS__
#define SNPRINTF _snprintf
#define FSEEK fseek
#else
#define SNPRINTF snprintf
#define FSEEK fseeko
#endif

static inline void geterrstr(int errnum, char *strerrbuf, size_t buflen) {
#ifdef HAVE_STRERROR_R
  strerror_r(errnum, strerrbuf, buflen);
#else
  SNPRINTF(strerrbuf, buflen, "%s",
           strerror(errnum)); /* hopefully this is thread-safe on the target system! */
#endif
}

// Use std::isfinite for portability (finite() removed on macOS/MSVC)
using std::isfinite;

namespace cvc {
CVC_DEF_EXCEPTION(invalid_mrc_header);
CVC_DEF_EXCEPTION(invalid_mrc_file);
CVC_DEF_EXCEPTION(unsupported_mrc_file);

// -----------
// checkHeader
// -----------
// Purpose:
//   Checks MRC header for correctness.
// ---- Change History ----
// ??/??/2007 -- Joe R. -- Creation.
static inline bool checkHeader(mrc_header &header, size_t fileSize) {
  size_t sizes[] = {1, 2, 4};

  // check for details we dont support
  if (header.mode < 0 || header.mode > 2)
    return false;

  // check the fileSize
  if (header.nsymbt == 0) {
    if (sizes[header.mode] * header.nx * header.ny * header.nz + sizeof(mrc_header) != fileSize)
      return false;
  } else {
    if (sizes[header.mode] * header.nx * header.ny * header.nz + sizeof(mrc_header) +
            sizeof(extended_mrc_header) !=
        fileSize)
      return false;
  }

  return true;
}

// ----------
// swapHeader
// ----------
// Purpose:
//   Swaps the order of bytes for each field in the header.
// ---- Change History ----
// ??/??/2007 -- Joe R. -- Creation.
static inline void swapHeader(mrc_header &header) {
  SWAP_32(&(header.nx));
  SWAP_32(&(header.ny));
  SWAP_32(&(header.nz));
  SWAP_32(&(header.mode));
  SWAP_32(&(header.nxstart));
  SWAP_32(&(header.nystart));
  SWAP_32(&(header.nzstart));

  SWAP_32(&(header.mx));
  SWAP_32(&(header.my));
  SWAP_32(&(header.mz));

  SWAP_32(&(header.xlength));
  SWAP_32(&(header.ylength));
  SWAP_32(&(header.zlength));

  SWAP_32(&(header.alpha));
  SWAP_32(&(header.beta));
  SWAP_32(&(header.gamma));

  SWAP_32(&(header.mapc));
  SWAP_32(&(header.mapr));
  SWAP_32(&(header.maps));

  SWAP_32(&(header.amin));
  SWAP_32(&(header.amax));
  SWAP_32(&(header.amean));

  SWAP_32(&(header.ispg));
  SWAP_32(&(header.nsymbt));

  for (unsigned int i = 0; i < 25; i++)
    SWAP_32(&(header.extra[i]));

  SWAP_32(&(header.xorigin));
  SWAP_32(&(header.yorigin));
  SWAP_32(&(header.zorigin));

  SWAP_32(&(header.rms));

  SWAP_32(&(header.nlabl));
}

// ------
// mrc_io
// ------
// Purpose:
//   Provides MRC file support.
// ---- Change History ----
// 11/20/2009 -- Joe R. -- Initially implemented.
struct mrc_io : public volume_file_io {
  // --------------
  // mrc_io::mrc_io
  // --------------
  // Purpose:
  //   Initializes the extension list and id.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class
  mrc_io() : _id("mrc_io : v1.0") {
    _extensions.push_back(".mrc");
    _extensions.push_back(".map");
  }

  // ----------
  // mrc_io::id
  // ----------
  // Purpose:
  //   Returns a string that identifies this volume_file_io object.  This should
  //   be unique, but is freeform.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class
  virtual const std::string &id() const { return _id; }

  // ------------------
  // mrc_io::extensions
  // ------------------
  // Purpose:
  //   Returns a list of extensions that this volume_file_io object supports.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class
  virtual const extension_list &extensions() const { return _extensions; }

  // -------------------------
  // mrc_io::getVolumeFileInfo
  // -------------------------
  // Purpose:
  //   Writes to a structure containing all info that VolMagick needs
  //   from a volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class
  virtual void getVolumeFileInfo(app &ctx, volume_file_info::data &d,
                                 const std::string &filename) const {
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);

    char buf[256];
    FILE *input;
    size_t i;
    data_type mrcTypes[] = {UChar, UShort, Float};
    bool mustSwap = false;

    mrc_header header;
    extended_mrc_header extheader;

    memset(buf, 0, 256);

    if ((input = fopen(filename.c_str(), "rb")) == NULL) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error opening file '" + filename + "': " + buf;
      throw read_error(errStr);
    }

    if (fread(&header, sizeof(mrc_header), 1, input) != 1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error reading file '" + filename + "': " + buf;
      fclose(input);
      throw read_error(errStr);
    }

    struct stat s;
    if (stat(filename.c_str(), &s) == -1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error stat-ing file '" + filename + "': " + buf;
      fclose(input);
      throw read_error(errStr);
    }

    // try to figure out the endianness of the file
    if (checkHeader(header, s.st_size))
      mustSwap = false;
    else {
      // swap and try again
      swapHeader(header);
      if (checkHeader(header, s.st_size))
        mustSwap = true;
      else {
        // we dont support wierd or exotic endianness
        fclose(input);
        throw invalid_mrc_file("Cannot determine endianness");
      }
    }

    if (!(header.map[0] == 'M' && header.map[1] == 'A' && header.map[2] == 'P')) {
      // interpret old header style
      d._dimension = dimension(header.nx, header.ny, header.nz);
      d._numTimesteps = 1;
      d._numVariables = 1;
      d._names.clear();
      d._names.push_back("No Name");
      d._voxelTypes.clear();

      if (header.mode > 2)
        throw invalid_mrc_file("Unsupported datatype");
      d._voxelTypes.push_back(mrcTypes[header.mode]);

      // we need to double check the meaning of xlength
      // (plus some extra paranoia)
      if (header.xlength <= 0.0 || header.ylength <= 0.0 || header.zlength <= 0.0 ||
          !isfinite(header.xlength) || !isfinite(header.ylength) || !isfinite(header.zlength))
        d._boundingBox = bounding_box(0.0, 0.0, 0.0, header.nx, header.ny, header.nz);
      else
        d._boundingBox =
            bounding_box(0.0, 0.0, 0.0, header.xlength, header.ylength, header.zlength);
    } else {
      double tmpmin[3], tmpmax[3];

      // new MRC file format
      d._dimension = dimension(header.nx, header.ny, header.nz);
      d._numTimesteps = 1;
      d._numVariables = 1;
      d._names.clear();
      d._names.push_back("No Name");
      d._voxelTypes.clear();
      d._voxelTypes.push_back(mrcTypes[header.mode]);

      // make sure we aren't using garbage values
      if (!isfinite(header.xorigin) || !isfinite(header.yorigin) || !isfinite(header.zorigin)) {
        tmpmin[0] = 0.0;
        tmpmin[1] = 0.0;
        tmpmin[2] = 0.0;
      } else {
        tmpmin[0] = header.xorigin;
        tmpmin[1] = header.yorigin;
        tmpmin[2] = header.zorigin;
      }

      // we need to double check the meaning of xlength
      // (plus some extra paranoia)
      // xlength, ylength, zlength means the size of the volume, that means
      // xlength=boundingbox.xmax()-boundingbox.xmin() similar to ylenght and zlength. So xlength,
      // ylength, zlength are positive.
      if (header.xlength <= 0.0 || header.ylength <= 0.0 || header.zlength <= 0.0 ||
          !isfinite(header.xlength) || !isfinite(header.ylength) || !isfinite(header.zlength)) {
        // hmm, this is wierd //not necessary.
        tmpmax[0] = tmpmin[0] + header.nx;
        tmpmax[1] = tmpmin[1] + header.ny;
        tmpmax[2] = tmpmin[2] + header.nz;
      } else {
        tmpmax[0] = tmpmin[0] + header.xlength;
        tmpmax[1] = tmpmin[1] + header.ylength;
        tmpmax[2] = tmpmin[2] + header.zlength;
      }

      d._boundingBox =
          bounding_box(tmpmin[0], tmpmin[1], tmpmin[2], tmpmax[0], tmpmax[1], tmpmax[2]);
    }

    d._tmin = d._tmax = 0.0;

    /* new volume, so min/max is now unset */
    d._minIsSet.clear();
    d._minIsSet.resize(d._numVariables);
    for (i = 0; i < d._minIsSet.size(); i++)
      d._minIsSet[i].resize(d._numTimesteps);
    d._min.clear();
    d._min.resize(d._numVariables);
    for (i = 0; i < d._min.size(); i++)
      d._min[i].resize(d._numTimesteps);
    d._maxIsSet.clear();
    d._maxIsSet.resize(d._numVariables);
    for (i = 0; i < d._maxIsSet.size(); i++)
      d._maxIsSet[i].resize(d._numTimesteps);
    d._max.clear();
    d._max.resize(d._numVariables);
    for (i = 0; i < d._max.size(); i++)
      d._max[i].resize(d._numTimesteps);

    /* the min and max values are in the header */
    d._min[0][0] = header.amin;
    d._max[0][0] = header.amax;
    d._minIsSet[0][0] = true;
    d._maxIsSet[0][0] = true;

    d._filename = filename;

    fclose(input);
  }

  // ----------------------
  // mrc_io::readVolumeFile
  // ----------------------
  // Purpose:
  //   Writes to a volume object after reading from a volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class
  virtual void readVolumeFile(app &ctx, volume &vol, const std::string &filename, unsigned int var,
                              unsigned int time, uint64 off_x, uint64 off_y, uint64 off_z,
                              const dimension &subvoldim) const {
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);

    char buf[256];

    FILE *input;
    size_t i, j, k;
    bool mustSwap = false;
    mrc_header header;
    extended_mrc_header extheader;

    memset(buf, 0, 256);

    if (var > 0)
      throw index_out_of_bounds("Variable index out of bounds.");
    if (time > 0)
      throw index_out_of_bounds("Timestep index out of bounds.");
    if (subvoldim.isNull())
      throw index_out_of_bounds("Specified subvolume dimension is null.");

    volume_file_info vfi(ctx, filename);

    if ((off_x + subvoldim[0] - 1 >= vfi.XDim()) || (off_y + subvoldim[1] - 1 >= vfi.YDim()) ||
        (off_z + subvoldim[2] - 1 >= vfi.ZDim())) {
      throw index_out_of_bounds("Subvolume specified is outside volume dimensions");
    }

    /** errors checked, now we can start modifying the volume object */
    vol.voxelType(vfi.voxelType());
    vol.voxel_dimensions(subvoldim);
    vol.boundingBox(bounding_box(vfi.XMin() + off_x * vfi.XSpan(), vfi.YMin() + off_y * vfi.YSpan(),
                                 vfi.ZMin() + off_z * vfi.ZSpan(),
                                 vfi.XMin() + (off_x + subvoldim[0] - 1) * vfi.XSpan(),
                                 vfi.YMin() + (off_y + subvoldim[1] - 1) * vfi.YSpan(),
                                 vfi.ZMin() + (off_z + subvoldim[2] - 1) * vfi.ZSpan()));
    vol.min(vfi.min());
    vol.max(vfi.max());

    /*
      read the volume data
    */
    if ((input = fopen(filename.c_str(), "rb")) == NULL) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error opening file '" + filename + "': " + buf;
      throw read_error(errStr);
    }

    // #if 0
    /* determine if we must swap values or not */
    struct stat s;
    if (stat(filename.c_str(), &s) == -1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error stat-ing file '" + filename + "': " + buf;
      fclose(input);
      throw read_error(errStr);
    }

    if (fread(&header, sizeof(mrc_header), 1, input) != 1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error reading file '" + filename + "': " + buf;
      fclose(input);
      throw read_error(errStr);
    }
    if (checkHeader(header, s.st_size))
      mustSwap = false;
    else {
      // swap and try again
      swapHeader(header);
      if (checkHeader(header, s.st_size))
        mustSwap = true;
      else {
        // we dont support wierd or exotic endianness
        fclose(input);
        throw invalid_mrc_file("Cannot determine endianness");
      }
    }

    if (header.nsymbt) // if there is extended header info
    {
      if (fread(&extheader, sizeof(extended_mrc_header), 1, input) != 1) {
        geterrstr(errno, buf, 256);
        std::string errStr = "Error reading file '" + filename + "': " + buf;
        fclose(input);
        throw read_error(errStr);
      }

      // no need to swap the extended header because we're ignoring it...
    }
    // #endif

    off_t file_offx, file_offy, file_offz;
    for (k = off_z; k <= (off_z + subvoldim[2] - 1); k++) {
      file_offz = 1024 + k * vol.XDim() * vol.YDim() * vol.voxelSize();
      for (j = off_y; j <= (off_y + subvoldim[1] - 1); j++) {
        file_offy = j * vol.XDim() * vol.voxelSize();
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

    if (mustSwap) {
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
        break; /* no swapping needed for unsigned char data, and unsigned int is not defined for mrc
                */
      }
    }

    // convert signed values to unsigned since volmagick doesnt support signed
    switch (vol.voxelType()) {
    case UChar: {
      // arand: hacked this because the shifts were causing problems
      float shift = 0.0;
      if (vol.min() < 0) {
        shift = -1.0 * vol.min();
      }
      // vol.min(((vol.min() - SCHAR_MIN)/(SCHAR_MAX - SCHAR_MIN))*UCHAR_MAX);
      // vol.max(((vol.max() - SCHAR_MIN)/(SCHAR_MAX - SCHAR_MIN))*UCHAR_MAX);
      size_t len = vol.XDim() * vol.YDim() * vol.ZDim();
      for (i = 0; i < len; i++) {
        char c = *((char *)(*vol + i * vol.voxelSize()));
        //*((unsigned char*)(*vol+i*vol.voxelSize())) = ((float(c) - SCHAR_MIN)/(SCHAR_MAX -
        // SCHAR_MIN))*UCHAR_MAX;
        *((unsigned char *)(*vol + i * vol.voxelSize())) = float(c) + shift;
      }
    } break;
    case UShort: {

      // arand: hacked this because I think this is now correct.

      float shift = 0.0;
      if (vol.min() < 0) {
        shift = -1.0 * vol.min();
      }
      // vol.min(((vol.min() - SHRT_MIN)/(SHRT_MAX - SHRT_MIN))*USHRT_MAX);
      // vol.max(((vol.max() - SHRT_MIN)/(SHRT_MAX - SHRT_MIN))*USHRT_MAX);

      size_t len = vol.XDim() * vol.YDim() * vol.ZDim();
      for (i = 0; i < len; i++) {
        short c = *((short *)(*vol + i * vol.voxelSize()));
        //*((unsigned short*)(*vol+i*vol.voxelSize())) = ((float(c) - SHRT_MIN)/(SHRT_MAX -
        // SHRT_MIN))*USHRT_MAX;

        *((unsigned short *)(*vol + i * vol.voxelSize())) = float(c) + shift;
      }
      vol.min(vol.min() + shift);
      vol.max(vol.max() + shift);

    } break;
    default:
      break;
    }

    fclose(input);
  }

  // ------------------------
  // mrc_io::createvolumeFile
  // ------------------------
  // Purpose:
  //   Creates an empty volume file to be later filled in by writeVolumeFile
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class
  virtual void createVolumeFile(app &ctx, const std::string &filename,
                                const bounding_box &boundingBox, const dimension &dimension,
                                const std::vector<data_type> &voxelTypes, unsigned int numVariables,
                                unsigned int numTimesteps, double min_time, double max_time) const {
    using namespace boost;
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);

    mrc_header mrcHeader;

    FILE *output;
    size_t i, j, k;

    if (boundingBox.isNull())
      throw invalid_bounding_box("Bounding box must not be null");
    if (dimension.isNull())
      throw invalid_bounding_box("Dimension must not be null");
    if (numVariables > 1)
      throw invalid_mrc_header(
          str(boost::format("MRC format only supports 1 variable (%1% requested)") % numVariables));
    if (numTimesteps > 1)
      throw invalid_mrc_header(
          str(boost::format("MRC format only supports 1 timestep (%1% requested)") % numTimesteps));
    if (voxelTypes.size() > 1)
      throw invalid_mrc_header(
          "MRC format only supports 1 variable and 1 timestep. (too many voxel types specified)");
    if (min_time != max_time)
      throw invalid_mrc_header("MRC format does not support multiple timesteps. (min time and max "
                               "time must be the same)");

    // check for unsupported type
    if (voxelTypes[0] == UInt || voxelTypes[0] == Double || voxelTypes[0] == UInt64)
      throw invalid_mrc_header(
          str(boost::format("Unsupported type: %1%") % data_type_strings[voxelTypes[0]]));

    memset(&mrcHeader, 0, sizeof(mrc_header));

    int type_conv[] = {0, 1, 2, 2, 2, 2};
    int type_sizes[] = {1, 2, 4, 4, 4, 4};
    mrcHeader.nx = dimension[0];
    mrcHeader.ny = dimension[1];
    mrcHeader.nz = dimension[2];
    mrcHeader.mx = dimension[0];
    mrcHeader.my = dimension[1];
    mrcHeader.mz = dimension[2];
    mrcHeader.mode = type_conv[int(voxelTypes[0])];
    strcpy(mrcHeader.map, "MAP");
    mrcHeader.xorigin = boundingBox.minx;
    mrcHeader.yorigin = boundingBox.miny;
    mrcHeader.zorigin = boundingBox.minz;
    mrcHeader.xlength = boundingBox.maxx - boundingBox.minx;
    mrcHeader.ylength = boundingBox.maxy - boundingBox.miny;
    mrcHeader.zlength = boundingBox.maxz - boundingBox.minz;
    mrcHeader.mapc = 1;
    mrcHeader.mapr = 2;
    mrcHeader.maps = 3;

    if (!big_endian()) {
      swapHeader(mrcHeader);
    }

    if ((output = fopen(filename.c_str(), "wb")) == NULL) {
      char buf[256] = {0};
      geterrstr(errno, buf, 256);
      std::string errStr = "Error opening file '" + filename + "': " + buf;
      throw write_error(errStr);
    }

    if (fwrite(&mrcHeader, sizeof(mrcHeader), 1, output) != 1) {
      char buf[256] = {0};
      geterrstr(errno, buf, 256);
      std::string errStr = "Error writing header to file '" + filename + "': " + buf;
      fclose(output);
      throw write_error(errStr);
    }

    scoped_array<unsigned char> scanline;
    try {
      scanline.reset(new unsigned char[dimension[0] * data_type_sizes[voxelTypes[0]]]);
    } catch (std::bad_alloc &e) {
      fclose(output);
      throw memory_allocation_error("Unable to allocate memory for write buffer");
    }
    memset(scanline.get(), 0,
           dimension[0] * data_type_sizes[voxelTypes[0]] * sizeof(unsigned char));
    // write a scanline at a time
    for (k = 0; k < dimension[2]; k++)
      for (j = 0; j < dimension[1]; j++) {
        if (fwrite(scanline.get(), data_type_sizes[voxelTypes[0]], dimension[0], output) !=
            dimension[0]) {
          char buf[256] = {0};
          geterrstr(errno, buf, 256);
          std::string errStr = "Error writing volume data to file '" + filename + "': " + buf;
          fclose(output);
          throw write_error(errStr);
        }
      }

    fclose(output);
  }

  // -----------------------
  // mrc_io::writeVolumeFile
  // -----------------------
  // Purpose:
  //   Writes the volume contained in wvol to the specified volume file. Should create
  //   a volume file if the filename provided doesn't exist.  Else it will simply
  //   write data to the existing file.  A common user error arises when you try to
  //   write over an existing volume file using this function for unrelated volumes.
  //   If what you desire is to overwrite an existing volume file, first run
  //   createVolumeFile to replace the volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/20/2009 -- Joe R. -- Converted to a volume_file_io class
  virtual void writeVolumeFile(app &ctx, const volume &wvol, const std::string &filename,
                               unsigned int var, unsigned int time, uint64 off_x, uint64 off_y,
                               uint64 off_z) const {
    using namespace boost;
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);

    volume_file_info volinfo;
    char buf[256];
    mrc_header header;
    extended_mrc_header extheader;
    bool creatingNewFile = false;
    bool mustSwap = false;

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
      volinfo.read(ctx, filename);
      // if(!(dimension(off_x+vol.XDim(),off_y+vol.YDim(),off_z+vol.ZDim()) <=
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
    } catch (read_error e) {
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

      cvc::createVolumeFile(ctx, filename, box, dim, std::vector<data_type>(1, vol.voxelType()), 1,
                            1, 0.0, 0.0);
      volinfo.read(ctx, filename);

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

    /* determine if we must swap values or not */
    struct stat s;
    if (stat(filename.c_str(), &s) == -1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error stat-ing file '" + filename + "': " + buf;
      fclose(output);

      throw read_error(errStr);
    }

    if (fread(&header, sizeof(mrc_header), 1, output) != 1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error reading file '" + filename + "': " + buf;
      fclose(output);
      throw read_error(errStr);
    }
    if (checkHeader(header, s.st_size))
      mustSwap = false;
    else {
      // swap and try again
      swapHeader(header);
      if (checkHeader(header, s.st_size))
        mustSwap = true;
      else {
        // we dont support wierd or exotic endianness
        fclose(output);
        throw invalid_mrc_file("Cannot determine endianness");
      }
    }

    if (header.nsymbt) // if there is extended header info
    {
      if (fread(&extheader, sizeof(extended_mrc_header), 1, output) != 1) {
        geterrstr(errno, buf, 256);
        std::string errStr = "Error reading file '" + filename + "': " + buf;
        fclose(output);
        throw read_error(errStr);
      }

      // no need to swap the extended header because we're ignoring it...
    }

    // TODO: correctly calculate the new min/max values if type is UChar or UShort
    // set the header's min/max values
    if (creatingNewFile) {
      // FIX: arand, I think this code messes up the min/max values
      //      in the header when using volconvert
      header.amin = MIN(0.0, vol.min());
      header.amax = MAX(0.0, vol.max());

    } else {
      header.amin = MIN(volinfo.min(), vol.min());
      header.amax = MAX(volinfo.max(), vol.max());
    }

    outvol_xdim = header.nx;
    outvol_ydim = header.ny;
    outvol_zdim = header.nz;

    if (!big_endian())
      swapHeader(header); // always write big endian data so it's similar to rawiv

    if (FSEEK(output, 0, SEEK_SET) == -1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error seeking in file '" + filename + "': " + buf;
      fclose(output);
      throw read_error(errStr);
    }

    if (fwrite(&header, sizeof(header), 1, output) != 1) {
      geterrstr(errno, buf, 256);
      std::string errStr = "Error writing header to file '" + filename + "': " + buf;
      fclose(output);
      throw write_error(errStr);
    }

    scoped_array<unsigned char> scanline;
    try {
      scanline.reset(new unsigned char[vol.XDim() * vol.voxelSize()]);
    } catch (std::bad_alloc &e) {
      fclose(output);
      throw memory_allocation_error("Unable to allocate memory for write buffer");
    }

    /*
      write the volume data
    */
    off_t file_offx, file_offy, file_offz;
    for (k = off_z; k <= (off_z + vol.ZDim() - 1); k++) {
      // arand: changed 68 (from the RawIV header) to sizeof(mrc_header).
      //        there may be some special cases when the extended
      //        mrcheader is being used but I haven't handled it.
      file_offz = sizeof(mrc_header) + k * outvol_xdim * outvol_ydim * vol.voxelSize();
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

        memcpy(scanline.get(),
               *vol + ((k - off_z) * vol.XDim() * vol.YDim() * vol.voxelSize()) +
                   ((j - off_y) * vol.XDim() * vol.voxelSize()),
               vol.XDim() * vol.voxelSize());

        switch (vol.voxelType()) {
        case UChar:
          break;
        case UShort:
          break;
        default:
          break;
        }

        /* swap the volume data if on little endian machine */
        if (!big_endian()) {
          size_t len = vol.XDim();
          switch (vol.voxelType()) {
          case UShort:
            for (i = 0; i < len; i++)
              SWAP_16(scanline.get() + i * vol.voxelSize());
            break;
          case Float:
            for (i = 0; i < len; i++)
              SWAP_32(scanline.get() + i * vol.voxelSize());
            break;
          case Double:
            for (i = 0; i < len; i++)
              SWAP_64(scanline.get() + i * vol.voxelSize());
            break;
          default:
            break; /* no swapping needed for unsigned char data, and unsigned int is not defined for
                      rawiv */
          }
        }

        if (fwrite(scanline.get(), vol.voxelSize(), vol.XDim(), output) != vol.XDim()) {
          geterrstr(errno, buf, 256);
          std::string errStr = "Error writing volume data to file '" + filename + "': " + buf;
          fclose(output);
          throw write_error(errStr);
        }
      }
    }

    fclose(output);
  }

protected:
  std::string _id;
  std::list<std::string> _extensions;
};

#ifdef CVC_USING_IMOD_MRC
// -----------
// imod_mrc_io
// -----------
// Purpose:
//   Uses IMOD's reading routines to read troublesome MRC files that the
//   regular mrc_io doesn't seem to support.
// ---- Change History ----
// 11/20/2009 -- Joe R. -- Creation
struct imod_mrc_io : public mrc_io {
  // ------------------------
  // imod_mrc_io::imod_mrc_io
  // ------------------------
  // Purpose:
  //   Initializes the extension list and id.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  imod_mrc_io() { _id = "imod_mrc_io : v1.0"; }

  // ------------------------------
  // imod_mrc_io::getVolumeFileInfo
  // ------------------------------
  // Purpose:
  //   Writes to a structure containing all info that VolMagick needs
  //   from a volume file.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  virtual void getVolumeFileInfo(app &ctx, volume_file_info::data &data,
                                 const std::string &filename) const {
    using namespace std;
    using namespace boost;
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);

    data_type mrcTypes[] = {UChar, UShort, Float};
    MrcHeader header;
    scoped_array<char> tmpFilename(new char[filename.size()]);
    memcpy(tmpFilename.get(), filename.c_str(), filename.size());
    char mode[3];
    strcpy(mode, "rb");
    ImodImageFile *iif = iiOpen(tmpFilename.get(), mode);
    if (iif == NULL)
      throw read_error("Error opening MRC file via libiimod");

    data._dimension = dimension(iif->nx, iif->ny, iif->nz);
    data._numTimesteps = 1;
    data._numVariables = 1;
    data._names.clear();
    data._names.push_back("No Name");
    data._voxelTypes.clear();
    if (iif->mode > 3)
      throw invalid_mrc_file("Invalid mode");
    data._voxelTypes.push_back(mrcTypes[iif->mode]);

    // read the header directly because it doesn't seem that bounding box info
    // is kept in the ImodImageFile struct.
    if (mrc_head_read(iif->fp, &header)) {
      iiClose(iif);
      iiDelete(iif);
      throw read_error("Error reading MRC header via libiimod");
    }

    data._boundingBox =
        bounding_box(header.nxstart, header.nystart, header.nzstart, header.nxstart + header.xlen,
                     header.nystart + header.ylen, header.nzstart + header.zlen);

    // only one timestep
    data._tmin = data._tmax = 0.0;

    /* new volume, so min/max is now unset */
    data._minIsSet.clear();
    data._minIsSet.resize(data._numVariables);
    for (unsigned int i = 0; i < data._minIsSet.size(); i++)
      data._minIsSet[i].resize(data._numTimesteps);
    data._min.clear();
    data._min.resize(data._numVariables);
    for (unsigned int i = 0; i < data._min.size(); i++)
      data._min[i].resize(data._numTimesteps);
    data._maxIsSet.clear();
    data._maxIsSet.resize(data._numVariables);
    for (unsigned int i = 0; i < data._maxIsSet.size(); i++)
      data._maxIsSet[i].resize(data._numTimesteps);
    data._max.clear();
    data._max.resize(data._numVariables);
    for (unsigned int i = 0; i < data._max.size(); i++)
      data._max[i].resize(data._numTimesteps);

    /* the min and max values are in the header */
    data._min[0][0] = header.amin;
    data._max[0][0] = header.amax;
    data._minIsSet[0][0] = true;
    data._maxIsSet[0][0] = true;

    iiClose(iif);
    iiDelete(iif);
  }

  // ---------------------------
  // imod_mrc_io::readVolumeFile
  // ---------------------------
  // Purpose:
  //   Writes to a Volume object after reading from a volume file.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  virtual void readVolumeFile(app &ctx, volume &vol, const std::string &filename, unsigned int var,
                              unsigned int time, uint64 off_x, uint64 off_y, uint64 off_z,
                              const dimension &subvoldim) const {
    using namespace std;
    using namespace boost;
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);

    if (var > 0)
      throw index_out_of_bounds("Variable index out of bounds.");
    if (time > 0)
      throw index_out_of_bounds("Timestep index out of bounds.");
    if (subvoldim.isNull())
      throw index_out_of_bounds("Specified subvolume dimension is null.");

    volume_file_info vfi(ctx, filename);

    if ((off_x + subvoldim[0] - 1 >= vfi.XDim()) || (off_y + subvoldim[1] - 1 >= vfi.YDim()) ||
        (off_z + subvoldim[2] - 1 >= vfi.ZDim()))
      throw index_out_of_bounds("Subvolume specified is outside volume dimensions");

    /** errors checked, now we can start modifying the volume object */
    vol.voxelType(vfi.voxelType());
    vol.voxel_dimensions(subvoldim);
    vol.boundingBox(bounding_box(vfi.XMin() + off_x * vfi.XSpan(), vfi.YMin() + off_y * vfi.YSpan(),
                                 vfi.ZMin() + off_z * vfi.ZSpan(),
                                 vfi.XMin() + (off_x + subvoldim[0] - 1) * vfi.XSpan(),
                                 vfi.YMin() + (off_y + subvoldim[1] - 1) * vfi.YSpan(),
                                 vfi.ZMin() + (off_z + subvoldim[2] - 1) * vfi.ZSpan()));
    vol.min(vfi.min());
    vol.max(vfi.max());

    // Finally read in the data, one section at a time. Extract out the
    // part of the section we need as specified in the subvoldim.
    scoped_array<char> tmpFilename(new char[filename.size()]);
    memcpy(tmpFilename.get(), filename.c_str(), filename.size());
    char mode[3];
    strcpy(mode, "rb");
    ImodImageFile *iif = iiOpen(tmpFilename.get(), mode);
    if (iif == NULL)
      throw read_error("Error opening MRC file via libiimod");
    boost::scoped_array<char> section(new char[vfi.XDim() * vfi.YDim() * vfi.voxelSize()]);
    for (size_t k = off_z; k <= (off_z + subvoldim[2] - 1); k++) {
      if (iiReadSection(iif, section.get(), k) == -1) {
        iiClose(iif);
        iiDelete(iif);
        throw read_error("Error reading MRC file via libiimod");
      }
      for (size_t j = off_y; j <= (off_y + subvoldim[1] - 1); j++)
        memcpy(*vol + (k - off_z) * vol.XDim() * vol.YDim() * vol.voxelSize() +
                   (j - off_y) * vol.XDim() * vol.voxelSize(),
               section.get() + (j * vfi.XDim() + off_x) * vfi.voxelSize(),
               vol.XDim() * vol.voxelSize());
    }

    // convert signed values to unsigned since volmagick doesnt support signed
    switch (vol.voxelType()) {
    case UChar: {
      vol.min(((vol.min() - double(SCHAR_MIN)) / (double(SCHAR_MAX) - double(SCHAR_MIN))) *
              double(UCHAR_MAX));
      vol.max(((vol.max() - double(SCHAR_MIN)) / (double(SCHAR_MAX) - double(SCHAR_MIN))) *
              double(UCHAR_MAX));
      size_t len = vol.XDim() * vol.YDim() * vol.ZDim();
      for (int i = 0; i < len; i++) {
        char c = *((char *)(*vol + i * vol.voxelSize()));
        *((unsigned char *)(*vol + i * vol.voxelSize())) =
            (unsigned char)(((double(c) - double(SCHAR_MIN)) /
                             (double(SCHAR_MAX) - double(SCHAR_MIN))) *
                            double(UCHAR_MAX));
      }
    } break;
    case UShort: {
      vol.min(((vol.min() - double(SHRT_MIN)) / (double(SHRT_MAX) - double(SHRT_MIN))) *
              double(USHRT_MAX));
      vol.max(((vol.max() - double(SHRT_MIN)) / (double(SHRT_MAX) - double(SHRT_MIN))) *
              double(USHRT_MAX));
      size_t len = vol.XDim() * vol.YDim() * vol.ZDim();
      for (int i = 0; i < len; i++) {
        short c = *((short *)(*vol + i * vol.voxelSize()));
        *((unsigned short *)(*vol + i * vol.voxelSize())) =
            (unsigned short)(((float(c) - double(SHRT_MIN)) /
                              (double(SHRT_MAX) - double(SHRT_MIN))) *
                             double(USHRT_MAX));
      }
    } break;
    default:
      break;
    }

    iiClose(iif);
    iiDelete(iif);
  }
};
#endif
} // namespace cvc

namespace cvc {
void register_mrc_io(app &ctx) { volume_file_io::insertHandler(volume_file_io::ptr(new mrc_io)); }
} // namespace cvc
