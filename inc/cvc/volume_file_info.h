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

#ifndef __VOLMAGICK_VOLUMEFILEINFO_H__
#define __VOLMAGICK_VOLUMEFILEINFO_H__

#include <cvc/bounding_box.h>
#include <cvc/dimension.h>
#include <cvc/types.h>
#include <string>
#include <vector>

namespace CVC_NAMESPACE {
class app; // forward-declared to avoid pulling all of app.h into headers

class volume_file_info {
public:
  volume_file_info()
      : _dimension(_data._dimension), _boundingBox(_data._boundingBox), _minIsSet(_data._minIsSet),
        _min(_data._min), _maxIsSet(_data._maxIsSet), _max(_data._max),
        _numVariables(_data._numVariables), _numTimesteps(_data._numTimesteps),
        _voxelTypes(_data._voxelTypes), _filename(_data._filename), _names(_data._names),
        _tmin(_data._tmin), _tmax(_data._tmax), _ctx(nullptr) {}

  volume_file_info(app &ctx, const std::string &file)
      : _dimension(_data._dimension), _boundingBox(_data._boundingBox), _minIsSet(_data._minIsSet),
        _min(_data._min), _maxIsSet(_data._maxIsSet), _max(_data._max),
        _numVariables(_data._numVariables), _numTimesteps(_data._numTimesteps),
        _voxelTypes(_data._voxelTypes), _filename(_data._filename), _names(_data._names),
        _tmin(_data._tmin), _tmax(_data._tmax), _ctx(&ctx) {
    read(ctx, file);
  }

  volume_file_info(const volume_file_info &vfi)
      : _dimension(_data._dimension), _boundingBox(_data._boundingBox), _minIsSet(_data._minIsSet),
        _min(_data._min), _maxIsSet(_data._maxIsSet), _max(_data._max),
        _numVariables(_data._numVariables), _numTimesteps(_data._numTimesteps),
        _voxelTypes(_data._voxelTypes), _filename(_data._filename), _names(_data._names),
        _tmin(_data._tmin), _tmax(_data._tmax), _data(vfi._data), _ctx(vfi._ctx) {}
  ~volume_file_info() {}

  volume_file_info &operator=(const volume_file_info &vfi) {
    if (this == &vfi)
      return *this;
    _data = vfi._data;
    _ctx = vfi._ctx;
    return *this;
  }

  /*
    call volume_file_info::read() to fill out this object from
    the info in the supplied file header.
  */
  void read(app &ctx, const std::string &filename);

  /***** Volume info accessors *****/
  /*
    Volume dimensions
  */
  dimension &voxel_dimensions() { return _data._dimension; }
  const dimension &voxel_dimensions() const { return _data._dimension; }
  void voxel_dimensions(const dimension &d) { _data._dimension = d; }
  uint64 XDim() const { return voxel_dimensions().xdim; }
  uint64 YDim() const { return voxel_dimensions().ydim; }
  uint64 ZDim() const { return voxel_dimensions().zdim; }

  /*
    Bounding box in object space
   */
  bounding_box &boundingBox() { return _data._boundingBox; }
  const bounding_box &boundingBox() const { return _data._boundingBox; }
  void boundingBox(const bounding_box &box) { _data._boundingBox = box; }

  double XMin() const { return boundingBox().minx; }
  double XMax() const { return boundingBox().maxx; }
  double YMin() const { return boundingBox().miny; }
  double YMax() const { return boundingBox().maxy; }
  double ZMin() const { return boundingBox().minz; }
  double ZMax() const { return boundingBox().maxz; }
  void TMin(double t) { _data._tmin = t; }
  double TMin() const { return _data._tmin; }
  void TMax(double t) { _data._tmax = t; }
  double TMax() const { return _data._tmax; }

  double XSpan() const {
    return XDim() - 1 == 0 ? 1.0 : (boundingBox().maxx - boundingBox().minx) / (XDim() - 1);
  }
  double YSpan() const {
    return YDim() - 1 == 0 ? 1.0 : (boundingBox().maxy - boundingBox().miny) / (YDim() - 1);
  }
  double ZSpan() const {
    return ZDim() - 1 == 0 ? 1.0 : (boundingBox().maxz - boundingBox().minz) / (ZDim() - 1);
  }
  double TSpan() const { return (TMax() - TMin()) / numTimesteps(); }

  /* min and max voxel values */
  double min(unsigned int var = 0, unsigned int time = 0) const {
    if (!isSet())
      return std::numeric_limits<double>::max();
    if (!_data._minIsSet[var][time])
      calcMinMax(var, time);
    return _data._min[var][time];
  }
  void min(double val, unsigned int var, unsigned int time) {
    if (isSet()) {
      _data._min[var][time] = val;
      _data._minIsSet[var][time] = true;
    }
  }
  double max(unsigned int var = 0, unsigned int time = 0) const {
    if (!isSet())
      return -std::numeric_limits<double>::max();
    if (!_data._maxIsSet[var][time])
      calcMinMax(var, time);
    return _data._max[var][time];
  }
  void max(double val, unsigned int var, unsigned int time) {
    if (isSet()) {
      _data._max[var][time] = val;
      _data._maxIsSet[var][time] = true;
    }
  }

  void numVariables(unsigned int vars) {
    _data._numVariables = vars;
    _data._voxelTypes.resize(vars);
  }
  unsigned int numVariables() const { return _data._numVariables; }
  void numTimesteps(unsigned int times) { _data._numTimesteps = times; }
  unsigned int numTimesteps() const { return _data._numTimesteps; }
  std::vector<data_type> voxelTypes() const { return _data._voxelTypes; }
  std::vector<data_type> &voxelTypes() { return _data._voxelTypes; }
  data_type voxelTypes(unsigned int vt) const { return _data._voxelTypes[vt]; }
  data_type &voxelTypes(unsigned int vt) { return _data._voxelTypes[vt]; }
  data_type voxelType() const { return voxelTypes(0); }
  std::string voxelTypeStr(unsigned vt = 0) const {
    return std::string(data_type_strings[voxelTypes(vt)]);
  }
  uint64 voxelSizes(unsigned int vt = 0) const { return data_type_sizes[voxelTypes(vt)]; }
  uint64 voxelSize() const { return voxelSizes(); }

  std::string filename() const { return _data._filename; }

  std::string name(unsigned int var = 0) const { return _data._names[var]; }

  bool isSet() const { return !filename().empty(); }

  struct data {
    data() : _numVariables(0), _numTimesteps(0), _tmin(0.0), _tmax(0.0) {}

    dimension _dimension;
    bounding_box _boundingBox;
    mutable std::vector<std::vector<bool>> _minIsSet;
    mutable std::vector<std::vector<double>> _min;
    mutable std::vector<std::vector<bool>> _maxIsSet;
    mutable std::vector<std::vector<double>> _max;
    unsigned int _numVariables;
    unsigned int _numTimesteps;
    std::vector<data_type> _voxelTypes;
    std::string _filename;
    std::vector<std::string> _names;
    double _tmin, _tmax;
  };

private:
  void calcMinMax(unsigned int var = 0, unsigned int time = 0) const;

  // references to the members of _data
  dimension &_dimension;
  bounding_box &_boundingBox;
  std::vector<std::vector<bool>> &_minIsSet;
  std::vector<std::vector<double>> &_min;
  std::vector<std::vector<bool>> &_maxIsSet;
  std::vector<std::vector<double>> &_max;
  unsigned int &_numVariables;
  unsigned int &_numTimesteps;
  std::vector<data_type> &_voxelTypes;
  std::string &_filename;
  std::vector<std::string> &_names;
  double &_tmin;
  double &_tmax;

  data _data;
  // Non-owning pointer to the app context associated with this object.
  // Set by the (app&, filename) ctor / read(app&, ...) overload so that
  // later operations (e.g. calcMinMax) do not require app::instance().
  app *_ctx;
};
} // namespace CVC_NAMESPACE

#endif
