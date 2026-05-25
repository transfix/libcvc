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

#ifndef __VOLMAGICK_UTILITY_H__
#define __VOLMAGICK_UTILITY_H__

#include <boost/any.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/version.hpp>
#include <cmath>
#include <cvc/geometry/geometry.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <xmlrpc/XmlRpc.h>

#if BOOST_VERSION < 105000
#define TIME_UTC_ TIME_UTC
#endif

namespace cvc {
std::string get_local_ip_address();

static inline unsigned int upToPowerOfTwo(unsigned int value) {
  unsigned int c = 0;
  unsigned int v = value;

  // round down to nearest power of two
  while (v > 1) {
    v = v >> 1;
    c++;
  }

  // if that isn't exactly the original value
  if ((v << c) != value) {
    // return the next power of two
    return (v << (c + 1));
  } else {
    // return this power of two
    return (v << c);
  }
}

static inline bool ends_with(const std::string &haystack, const std::string &needle) {
  return haystack.rfind(needle) == haystack.size() - needle.size();
}

// arand: the mac doesn't like VERSION... so I changed it to VM_VERSION
//        also, I moved this to the header... I have no idea why this was working
//        in the .cpp file but the mac was complaining
const uint64 VM_VERSION = 0x00010206;

template <class T> const T &MIN(const T &a, const T &b) { return std::min(a, b); }
template <class T> const T &MAX(const T &a, const T &b) { return std::max(a, b); }

// trilinear interpolation function
static inline double getTriVal(double val[8], double x, double y, double z, double resX,
                               double resY, double resZ) {
  double x_ratio, y_ratio, z_ratio;
  double temp1, temp2, temp3, temp4, temp5, temp6;

  x_ratio = x / resX;
  y_ratio = y / resY;
  z_ratio = z / resZ;

  if (x_ratio == 1)
    x_ratio = 0;
  if (y_ratio == 1)
    y_ratio = 0;
  if (z_ratio == 1)
    z_ratio = 0;

  temp1 = val[0] + (val[1] - val[0]) * x_ratio;
  temp2 = val[4] + (val[5] - val[4]) * x_ratio;
  temp3 = val[2] + (val[3] - val[2]) * x_ratio;
  temp4 = val[6] + (val[7] - val[6]) * x_ratio;
  temp5 = temp1 + (temp3 - temp1) * y_ratio;
  temp6 = temp2 + (temp4 - temp2) * y_ratio;

  return temp5 + (temp6 - temp5) * z_ratio;
}

/*
  Shortcut for creating a volume file based on a volume file info object.
  All overloads require an explicit cvc::app context.
*/
static inline void createVolumeFile(app &ctx, const std::string &filename,
                                    const volume_file_info &volinfo) {
  createVolumeFile(ctx, filename, volinfo.boundingBox(), volinfo.voxel_dimensions(),
                   volinfo.voxelTypes(), volinfo.numVariables(), volinfo.numTimesteps(),
                   volinfo.TMin(), volinfo.TMax());
}

// ----------------
// createVolumeFile
// ----------------
// Purpose:
//   Shortcut for creating a volume file based on a volume object.  Also
//   writes the volume data in the object to the specified file.
// ---- Change History ----
// 01/04/2010 -- Joe R. -- Creation.
static inline void createVolumeFile(app &ctx, const volume &vol, const std::string &filename) {
  createVolumeFile(ctx, filename, vol.boundingBox(), vol.voxel_dimensions(),
                   std::vector<data_type>(1, vol.voxelType()));
  writeVolumeFile(ctx, vol, filename);
}

// ----------------
// createVolumeFile
// ----------------
// Purpose:
//   Same as above, except with arguments in the order consistent with the full
//   createVolumeFile call
// ---- Change History ----
// 01/04/2010 -- Joe R. -- Creation.
static inline void createVolumeFile(app &ctx, const std::string &filename, const volume &vol) {
  createVolumeFile(ctx, vol, filename);
}

/*
  Calculates the gradient vector field of the input voxels, and returns the xyz vector values as 3
  volumes in 'grad' 'vt' is the voxel type of the gradient volumes.  If 'vt' is of integral type
  (UChar, UShort, UInt), the first half of the set of integers maps to [-1.0,0) and the last half
  maps to (0,1.0].
*/
// ---- All free functions take an explicit cvc::app context. ----
void calcGradient(app &ctx, std::vector<volume> &grad, const volume &vol, data_type vt = Float);

void sub(app &ctx, volume &dest, const volume &vol, uint64 off_x, uint64 off_y, uint64 off_z,
         const dimension &subvoldim);

void volconvert(app &ctx, const std::string &input_volume_file,
                const std::string &output_volume_file);

boost::any load(app &ctx, const std::string &filename);
void save(app &ctx, const boost::any &data, const std::string &filename);

/*
 * Some typical vector math utility functions:
 *  cross, dot, normalize
 *
 * Change Log:
 * 04/02/2010 - Moved this code into it's own header from cvcraw_geometry.h
 */
template <class Vector_3> void cross(Vector_3 &dest, const Vector_3 &v1, const Vector_3 &v2) {
  dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
  dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
  dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

template <class Vector_3> double dot(const Vector_3 &v1, const Vector_3 &v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

template <class Vector_3> void normalize(Vector_3 &v) {
  double len = sqrt(static_cast<double>(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
  if (len != 0.0) {
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
  } else {
    v[0] = 1.0;
  }
}

// 01/12/2014 - Joe R. - Creation.
std::string json(const boost::property_tree::ptree &pt);
boost::property_tree::ptree json(const std::string &pt_str);

const int XMLRPC_DEFAULT_PORT = 23196;

// 01/13/2014 - Joe R. - Creation.  Implementation is in xmlrpc_client.cpp
XmlRpc::XmlRpcValue
rpc_call(app &ctx, const std::string &host, int port, const std::string &method_name,
         const XmlRpc::XmlRpcValue &params, bool sync = true,
         boost::posix_time::ptime mod_time = boost::posix_time::microsec_clock::universal_time());

XmlRpc::XmlRpcValue
rpc_call(app &ctx, const std::string &host, int port, const std::string &method_name,
         const std::vector<std::string> &params, bool sync = true,
         boost::posix_time::ptime mod_time = boost::posix_time::microsec_clock::universal_time());

XmlRpc::XmlRpcValue
rpc(app &ctx, const std::string &url, const std::string &method_name,
    const std::vector<std::string> &params, bool sync = true,
    boost::posix_time::ptime mod_time = boost::posix_time::microsec_clock::universal_time());

// Splits a string of the form <host>:<port>
// 01/13/1024 - Joe R. - Creation.
boost::tuple<std::string, int> get_xmlrpc_host_and_port(const std::string &host_and_port);

/*
 * save/restore utils, added 1/12/2014 - Joe R.
 */
bool is_geometry(const boost::any &data);
bool is_volume(const boost::any &data);
bool is_volume_file_info(const boost::any &data);
bool is_geometry_filename(const std::string &filename);
bool is_volume_filename(const std::string &filename);
} // namespace cvc
#endif
