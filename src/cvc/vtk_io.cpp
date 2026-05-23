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

/* readvtk/writevtk:
 * Copyright (c) The Technical University of Denmark
 * Author: Ojaswa Sharma
 * E-mail: os@imm.dtu.dk
 * File: vtkIO.h
 * .vtk I/O
 */

#include <boost/format.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/app.h>
#include <cvc/endians.h>
#include <cvc/volmagick.h>
#include <errno.h>

#ifdef __WINDOWS__
#define SNPRINTF _snprintf
#define FSEEK fseek
#else
#define SNPRINTF snprintf
#define FSEEK fseeko
#endif

namespace {
#define LINE_LENGTH 256
typedef float FLOAT; // doubles are supported by VTK format, but here we just want to read as float.

struct DataInfo {
  unsigned int n[3];       // Number of elements in each dimension
  unsigned int n_input[3]; // NUmber of elements in the input volume
  unsigned long nTotal;    // Total number of voxels = n[0]*n[1]*n[3]
  char dataType[256];      // data type of values. -ve value indicated signed data
  bool volumeCovered;      // cover the input volume with a one voxel layer around - binary flag.
};

FLOAT *readvtk(const char *filename, DataInfo &di, unsigned int subvol_dim,
               bool coverVolume = false);
void writevtk(const char *filename, const DataInfo &di, const FLOAT *data, float value_increment);

static inline void geterrstr(int errnum, char *strerrbuf, size_t buflen) {
#ifdef HAVE_STRERROR_R
  strerror_r(errnum, strerrbuf, buflen);
#else
  SNPRINTF(strerrbuf, buflen, "%s",
           strerror(errnum)); /* hopefully this is thread-safe on the target system! */
#endif
}

// -------
// readvtk
// -------
// Purpose:
//   Reads a VTK file and returns a pointer to the data. The binary file is in Big-Endian format.
// ---- Change History ----
// ??/??/2008 -- Ojaswa S. -- Creation.
FLOAT *readvtk(const char *filename, DataInfo &di, unsigned int subvol_dim, bool coverVolume) {
  using namespace std;
  using namespace boost;

  char buf[256];
  // Read file supplied as an argument...
  fprintf(stderr, "Reading datafile %s...", filename);
  FILE *fd = fopen(filename, "rb");
  if (fd == NULL) {
    geterrstr(errno, buf, 256);
    string errStr = "Error opening file '" + string(filename) + "': " + string(buf);
    throw CVC_NAMESPACE::read_error(errStr);
  }

  char version[LINE_LENGTH];
  char comments[LINE_LENGTH];
  char format_[LINE_LENGTH];    //"BINARY", "ASCII"
  char type[LINE_LENGTH];       //"DATASET STRUCTURED_POINTS"
  char dimensions[LINE_LENGTH]; //"DIMENSIONS NX NY NZ"
  char origin[LINE_LENGTH];     //"ORIGIN OX OY OZ"
  char spacing[LINE_LENGTH];    //"SPACING SX SY SZ"
  char num_pts[LINE_LENGTH];    //"POINT_DATA NN", NN = NX*NY*NZ
  char data_type[LINE_LENGTH];  //"SCALARS name data_type", e.g. "image_data unsigned_short"
  char lookup_t[LINE_LENGTH];   //"LOOKUP_TABLE default"

  fgets(version, LINE_LENGTH, fd);
  fgets(comments, LINE_LENGTH, fd);
  fgets(format_, LINE_LENGTH, fd);
  fgets(type, LINE_LENGTH, fd);
  fgets(dimensions, LINE_LENGTH, fd);
  fgets(origin, LINE_LENGTH, fd);
  fgets(spacing, LINE_LENGTH, fd);
  fgets(num_pts, LINE_LENGTH, fd);
  fgets(data_type, LINE_LENGTH, fd);
  fgets(lookup_t, LINE_LENGTH, fd);

  if (strncmp(format_, "BINARY", 6) != 0) {
    string errStr =
        "Error reading file '" + string(filename) + "': Only binary .vtk files are supported.";
    fclose(fd);
    throw CVC_NAMESPACE::read_error(errStr);
  }

  char _type[256];
  sscanf(type, "%*s %s", _type);
  if (strncmp(_type, "STRUCTURED_POINTS", 17) != 0) {
    string errStr =
        "Error reading file '" + string(filename) + "': Only structured points are supported.";
    fclose(fd);
    throw CVC_NAMESPACE::read_error(errStr);
  }

  char _dimstr[256];
  sscanf(dimensions, "%s %u %u %u", _dimstr, &(di.n_input[0]), &(di.n_input[1]), &(di.n_input[2]));
  if (strncmp(_dimstr, "DIMENSIONS", 10) != 0) {
    string errStr = "Error reading file '" + string(filename) + "': Corrupt header for dimension.";
    fclose(fd);
    throw CVC_NAMESPACE::read_error(errStr);
  }

  di.volumeCovered = coverVolume;
  if (coverVolume) {
    di.n[0] = di.n_input[0] + 2;
    di.n[1] = di.n_input[1] + 2;
    di.n[2] = di.n_input[2] + 2;
  }
  // pad to match subvolume multiples
  subvol_dim -= 2; // Since we have a 2 voxel overlap between subvolumes.
  unsigned int n =
      ((di.n[0] % subvol_dim) != 0) ? (di.n[0] / subvol_dim + 1) : (di.n[0] / subvol_dim);
  di.n[0] = 2 + n * subvol_dim;
  n = ((di.n[1] % subvol_dim) != 0) ? (di.n[1] / subvol_dim + 1) : (di.n[1] / subvol_dim);
  di.n[1] = 2 + n * subvol_dim;
  n = ((di.n[2] % subvol_dim) != 0) ? (di.n[2] / subvol_dim + 1) : (di.n[2] / subvol_dim);
  di.n[2] = 2 + n * subvol_dim;

  char _numptsstr[256];
  sscanf(num_pts, "%s %lu", _numptsstr, &(di.nTotal));
  if (strncmp(_numptsstr, "POINT_DATA", 10) != 0) {
    string errStr = "Error reading file '" + string(filename) + "': Corrupt header for data type.";
    fclose(fd);
    throw CVC_NAMESPACE::read_error(errStr);
  }

  char _datastr[256], _dataname[256];
  sscanf(data_type, "%s %s %s", _datastr, _dataname, di.dataType);
  if (strncmp(_datastr, "SCALARS", 7) != 0 || strncmp(_dataname, "image_data", 10) != 0) {
    string errStr =
        "Error reading file '" + string(filename) +
        "': Only scalar data of type \"image_data\" of type \"unsigned_short\" is supported.";
    fclose(fd);
    throw CVC_NAMESPACE::read_error(errStr);
  }

  fprintf(stderr, "\nInput volume size:  %d x %d x %d, %s\n", di.n_input[0], di.n_input[1],
          di.n_input[2], di.dataType);
  fprintf(stderr, "\nPadded volume size: %d x %d x %d\n", di.n[0], di.n[1], di.n[2]);
  // fprintf(stderr, "%s.\n%s.\n%s.\n%s.\n%s.\n%s.\n%s.\n%s.\n%s.\n%s.\n", version, comments,
  // format, type, dimensions, origin, spacing, num_pts, data_type, lookup_t);

  // Allocate memory
  unsigned long nMem = (di.n[0] * di.n[1] * di.n[2]);
  unsigned byte_size = 0;
  if (strncmp(di.dataType, "unsigned_char", 13) == 0)
    byte_size = sizeof(unsigned char);
  else if (strncmp(di.dataType, "char", 4) == 0)
    byte_size = sizeof(char);
  else if (strncmp(di.dataType, "unsigned_short", 14) == 0)
    byte_size = sizeof(unsigned short);
  else if (strncmp(di.dataType, "short", 5) == 0)
    byte_size = sizeof(short);
  else if (strncmp(di.dataType, "unsigned_int", 12) == 0)
    byte_size = sizeof(unsigned int);
  else if (strncmp(di.dataType, "int", 3) == 0)
    byte_size = sizeof(int);
  else if (strncmp(di.dataType, "float", 5) == 0)
    byte_size = sizeof(float);
  else if (strncmp(di.dataType, "double", 6) == 0)
    byte_size = sizeof(double);
  // N.B.: our data currently is 16 bit unsigned int (unsigned short). We therefore do not worry
  // about other types!!!
  if (strncmp(di.dataType, "unsigned_short", 14) != 0) {
    string errStr = "Error reading file '" + string(filename) +
                    "': Only 16 bit unsignd integers are supported at the moment.";
    fclose(fd);
    throw CVC_NAMESPACE::read_error(errStr);
  }

  FLOAT *dataPtr = (FLOAT *)calloc(
      nMem, sizeof(FLOAT)); // initialize mem to zero - so that padded part is set to zero!
  if (dataPtr == NULL) {
    string errStr(
        str(boost::format("Error reading file '%1%': Could not allocate %2% bytes of memory!") %
            string(filename) % (nMem * sizeof(FLOAT))));
    fclose(fd);
    throw CVC_NAMESPACE::read_error(errStr);
  }

  unsigned short dataVal;
  unsigned int _r, _c, _d;
  FLOAT *ptr = NULL;
  for (_d = 0; _d < di.n_input[2]; _d++)
    for (_c = 0; _c < di.n_input[1]; _c++)
      for (_r = 0; _r < di.n_input[0]; _r++) {
        if (feof(fd) > 0) {
          string errStr =
              "Error reading file '" + string(filename) + "': Insufficient elements in the file.";
          free(dataPtr);
          fclose(fd);
          throw CVC_NAMESPACE::read_error(errStr);
        }
        fread(&dataVal, byte_size, 1, fd);
        // dataVal = ntohs(dataVal);
        if (!CVC_NAMESPACE::big_endian())
          SWAP_16(&dataVal);
        ptr = coverVolume ? (dataPtr + _r + 1 + (_c + 1) * di.n[0] + (_d + 1) * di.n[0] * di.n[1])
                          : (dataPtr + _r + _c * di.n[0] + _d * di.n[0] * di.n[1]);
        *ptr = (FLOAT)(dataVal);
      }

  fclose(fd);
  fprintf(stderr, "Done.\n");
  return dataPtr; // return pointer to data
}

// --------
// writevtk
// --------
// Purpose:
//   Writes a VTK file.
// ---- Change History ----
// ??/??/2008 -- Ojaswa S. -- Creation.
void writevtk(const char *filename, const DataInfo &di, const FLOAT *dataPtr,
              float value_increment) {
  using namespace std;
  using namespace boost;

  char buf[256];
  // write file supplied as an argument...
  // bool stripCover = di.volumeCovered;
  fprintf(stderr, "Writing datafile %s...", filename);
  FILE *fd = fopen(filename, "wb");
  if (fd == NULL) {
    geterrstr(errno, buf, 256);
    string errStr = "Error opening file '" + string(filename) + "': " + string(buf);
    throw CVC_NAMESPACE::write_error(errStr);
  }
  unsigned long nMem = di.n[0] * di.n[1] * di.n[2];
  fputs("# vtk DataFile Version 3.0\n", fd);
  {
    char _buf[128];
    SNPRINTF(_buf, sizeof(_buf), "created by levelSet (Ojaswa Sharma). Zero level: %f\n",
             value_increment);
    fputs(_buf, fd);
  }
  fputs("BINARY\n", fd);
  fputs("DATASET STRUCTURED_POINTS\n", fd);
  {
    char _buf[128];
    SNPRINTF(_buf, sizeof(_buf), "DIMENSIONS %lu %lu %lu\n", (unsigned long)di.n[0],
             (unsigned long)di.n[1], (unsigned long)di.n[2]);
    fputs(_buf, fd);
  }
  fputs("ORIGIN 0.000000 0.000000 0.000000\n", fd);
  fputs("SPACING 1.000000 1.000000 1.000000\n", fd);
  {
    char _buf[64];
    SNPRINTF(_buf, sizeof(_buf), "POINT_DATA %lu\n", (unsigned long)nMem);
    fputs(_buf, fd);
  }
  fputs("SCALARS image_data unsigned_short\n", fd);
  fputs("LOOKUP_TABLE default\n", fd);
  unsigned short dataval;
  FLOAT datavalf;
  unsigned int _r, _c, _d;
  const FLOAT *ptr = NULL;
  for (_d = 0; _d < di.n[2]; _d++)
    for (_c = 0; _c < di.n[1]; _c++)
      for (_r = 0; _r < di.n[0]; _r++) {
        ptr = dataPtr + _r + _c * di.n[0] + _d * di.n[0] * di.n[1];
        datavalf = *ptr + value_increment;
        if (datavalf < 0.0)
          datavalf = 0.0;
        // dataval = (unsigned short)(round(datavalf));
        dataval = (unsigned short)(floor(datavalf + 0.5));
        // dataval = htons(dataval);
        if (!CVC_NAMESPACE::big_endian())
          SWAP_16(&dataval);
        if (fwrite(&dataval, sizeof(unsigned short), 1, fd) != 1) {
          geterrstr(errno, buf, 256);
          std::string errStr =
              "Error writing volume data to file '" + string(filename) + "': " + buf;
          fclose(fd);
          throw CVC_NAMESPACE::write_error(errStr);
        }
      }
  fclose(fd);
  fprintf(stderr, "Done\n");
}
} // namespace

namespace CVC_NAMESPACE {
// ------
// vtk_io
// ------
// Purpose:
//   Provides VTK file support.
// ---- Change History ----
// 11/20/2009 -- Joe R. -- Creation.
struct vtk_io : public volume_file_io {
  // --------------
  // vtk_io::vtk_io
  // --------------
  // Purpose:
  //   Initializes the extension list and id.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  vtk_io() : _id("vtk_io : v1.0") { _extensions.push_back(".vtk"); }

  // ----------
  // vtk_io::id
  // ----------
  // Purpose:
  //   Returns a string that identifies this volume_file_io object.  This should
  //   be unique, but is freeform.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  virtual const std::string &id() const { return _id; }

  // ------------------
  // vtk_io::extensions
  // ------------------
  // Purpose:
  //   Returns a list of extensions that this volume_file_io object supports.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  virtual const extension_list &extensions() const { return _extensions; }

  // -------------------------
  // vtk_io::getVolumeFileInfo
  // -------------------------
  // Purpose:
  //   Writes to a structure containing all info that VolMagick needs
  //   from a volume file.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  virtual void getVolumeFileInfo(app &ctx, volume_file_info::data & /*d*/,
                                 const std::string & /*filename*/) const {
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);
    throw read_error("Reading VTK files doesn't work yet!");
  }

  // ----------------------
  // vtk_io::readVolumeFile
  // ----------------------
  // Purpose:
  //   Writes to a Volume object after reading from a volume file.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  virtual void readVolumeFile(app &ctx, volume & /*vol*/, const std::string & /*filename*/,
                              unsigned int /*var*/, unsigned int /*time*/, uint64 /*off_x*/,
                              uint64 /*off_y*/, uint64 /*off_z*/,
                              const dimension & /*subvoldim*/) const {
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);
    throw read_error("Reading VTK files doesn't work yet!");
  }

  // ------------------------
  // vtk_io::createVolumeFile
  // ------------------------
  // Purpose:
  //   Creates an empty volume file to be later filled in by writeVolumeFile
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  virtual void createVolumeFile(app &ctx, const std::string & /*filename*/,
                                const bounding_box & /*boundingBox*/,
                                const dimension & /*dimension*/,
                                const std::vector<data_type> & /*voxelTypes*/,
                                unsigned int /*numVariables*/, unsigned int /*numTimesteps*/,
                                double /*min_time*/, double /*max_time*/) const {
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);
    throw CVC_NAMESPACE::write_error("Writing VTK files doesn't work yet!");
  }

  // -----------------------
  // vtk_io::writeVolumeFile
  // -----------------------
  // Purpose:
  //   Writes the volume contained in wvol to the specified volume file. Should create
  //   a volume file if the filename provided doesn't exist.  Else it will simply
  //   write data to the existing file.  A common user error arises when you try to
  //   write over an existing volume file using this function for unrelated volumes.
  //   If what you desire is to overwrite an existing volume file, first run
  //   createVolumeFile to replace the volume file.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  virtual void writeVolumeFile(app &ctx, const volume & /*wvol*/, const std::string & /*filename*/,
                               unsigned int /*var*/, unsigned int /*time*/, uint64 /*off_x*/,
                               uint64 /*off_y*/, uint64 /*off_z*/) const {
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);
    throw CVC_NAMESPACE::write_error("Writing VTK files doesn't work yet!");
  }

protected:
  std::string _id;
  std::list<std::string> _extensions;
};
} // namespace CVC_NAMESPACE

namespace CVC_NAMESPACE {
void register_vtk_io(app &ctx) { volume_file_io::insertHandler(volume_file_io::ptr(new vtk_io)); }
} // namespace CVC_NAMESPACE
