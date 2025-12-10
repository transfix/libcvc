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

#ifndef __VOLMAGICK_VOXELS_H__
#define __VOLMAGICK_VOXELS_H__

#include <cvc/namespace.h>
#include <cvc/types.h>
#include <cvc/dimension.h>
#include <cvc/exception.h>

#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>
#include <boost/tuple/tuple.hpp>

#include <algorithm>
#include <cstring>

namespace CVC_NAMESPACE
{
  class composite_function;

  /**
   * @class voxels
   * @brief Multi-dimensional voxel data container with configurable copy semantics
   * 
   * IMPORTANT - MEMORY SEMANTICS:
   * =============================
   * Voxels uses SHALLOW COPY semantics by default via boost::shared_ptr to boost::multi_array.
   * 
   * SHALLOW COPY (default - shares data):
   * - Copy constructor: voxels v2(v1);         // Shares underlying data
   * - Assignment operator: v2 = v1;            // Shares underlying data  
   * - copy() method: v2.copy(v1);              // Shares underlying data (default)
   * - copy() method: v2.copy(v1, false);       // Shares underlying data (explicit)
   * 
   * With shallow copy, all copies share the same underlying voxel data array.
   * Modifications through any copy will affect all other copies!
   * 
   * DEEP COPY (independent data):
   * - copy() method: v2.copy(v1, true);        // Creates independent copy
   * - sub() method: v2.sub(0, 0, 0, v1.voxel_dimensions());  // Creates independent copy
   * 
   * With deep copy, each copy has its own independent voxel data array.
   * Modifications to one copy will NOT affect other copies.
   * 
   * This design enables efficient passing without copying large arrays by default,
   * while still providing explicit deep copy when needed.
   */
  class voxels
  {
  public:
    voxels(const dimension& d = dimension(4,4,4), data_type vt = UChar);
    voxels(const void *v, const dimension& d, data_type vt);
    voxels(const voxels& v);
    virtual ~voxels();

    dimension& voxel_dimensions() { return _dimension; }
    const dimension& voxel_dimensions() const { return _dimension; }
    virtual void voxel_dimensions(const dimension& d, 
				  boost::shared_ptr<boost::multi_array<unsigned char, 1>> voxels = boost::shared_ptr<boost::multi_array<unsigned char, 1>>());
    uint64 XDim() const { return voxel_dimensions().xdim; }
    uint64 YDim() const { return voxel_dimensions().ydim; }
    uint64 ZDim() const { return voxel_dimensions().zdim; }

    /*
      Voxel I/O
    */
    double operator()(uint64 i) const /* reading a voxel value */
    {
      if(i >= XDim()*YDim()*ZDim()) 
	throw index_out_of_bounds("");
      
      unsigned char* data = _voxels->data();
      switch(voxelType())
	{
	case UChar:
	  return double(*((unsigned char *)(data+i*voxelSize())));
	case UShort:
	  return double(*((unsigned short *)(data+i*voxelSize())));
	case UInt:
	  return double(*((unsigned int *)(data+i*voxelSize())));
	case Float:
	  return double(*((float *)(data+i*voxelSize())));
	case Double:
	  return double(*((double *)(data+i*voxelSize())));
	case UInt64:
	  return double(*((uint64 *)(data+i*voxelSize())));
	}
      return 0;
    }
    double operator()(uint64 i, uint64 j, uint64 k) const /* reading a voxel value */
    {
      return (*this)(i+j*XDim()+k*XDim()*YDim());
    }
    
    void operator()(uint64 i, double val) /* writing a voxel value */
    {
      if(i >= XDim()*YDim()*ZDim()) 
	throw index_out_of_bounds("");

      preWrite();

      unsigned char* data = _voxels->data();
      switch(voxelType())
	{
	case UChar:
	  *((unsigned char *)(data+i*voxelSize())) = (unsigned char)(val);
	  break;
	case UShort:
	  *((unsigned short *)(data+i*voxelSize())) = (unsigned short)(val);
	  break;
	case UInt:
	  *((unsigned int *)(data+i*voxelSize())) = (unsigned int)(val);
	  break;
	case Float:
	  *((float *)(data+i*voxelSize())) = float(val);
	  break;
	case Double:
	  *((double *)(data+i*voxelSize())) = double(val);
	  break;
	case UInt64:
	  *((uint64 *)(data+i*voxelSize())) = uint64(val);
	}

      //NOTE: we cant modify min/max here because it would mess up a map() operation, and perhaps other things
      //if(_minIsSet && val < min()) min(val);
      //if(_maxIsSet && val > max()) max(val);
    }
    void operator()(uint64 i, uint64 j, uint64 k, double val) /* writing a voxel value */
    {
      (*this)(i+j*XDim()+k*XDim()*YDim(),val);
    }
    
    unsigned char * operator*() { preWrite(); return _voxels->data(); }
    const unsigned char * operator*() const { return _voxels->data(); }

    data_type voxelType() const { return _voxelType; }
    void voxelType(data_type);
    uint64 voxelSize() const { return data_type_sizes[voxelType()]; }
    const char * voxelTypeStr() const { return data_type_strings[voxelType()]; }

     /* min and max values */
    double min() const { if(!_minIsSet) calcMinMax(); return _min; }
    void min(double m) { _min = m; _minIsSet = true; }
    double max() const { if(!_maxIsSet) calcMinMax(); return _max; }
    void max(double m) { _max = m; _maxIsSet = true; }
    void unsetMinMax() { _minIsSet = _maxIsSet = false; }
    bool minIsSet() const { return _minIsSet; }
    bool maxIsSet() const { return _maxIsSet; }

    /* calculate min and max values for selected subvolumes */
    double min(uint64 off_x, uint64 off_y, uint64 off_z,
	       const dimension& dim) const;
    double max(uint64 off_x, uint64 off_y, uint64 off_z,
	       const dimension& dim) const;

    voxels& operator=(const voxels& vox) { copy(vox); return *this; }

    bool operator==(const voxels& vox) const 
      { 
        // Check if same shared_ptr pointer (shared data)
        if(_voxels == vox._voxels) return true;
        
        // Check if different dimensions or types
        if(voxel_dimensions() != vox.voxel_dimensions()) return false;
        if(voxelType() != vox.voxelType()) return false;
        
        // Compare actual data bytes (size * voxelSize, not just size!)
        return strncmp(reinterpret_cast<const char*>(_voxels->data()),
                       reinterpret_cast<const char*>(vox._voxels->data()),
                       voxel_dimensions().size() * voxelSize()) == 0;
      }

    bool operator!=(const voxels& vox) const
    {
      return !(*this == vox);
    }

    boost::tuple<const uint64 *,uint64> 
      histogram(uint64 size = 256) const 
    { 
      calcHistogram(size); return boost::make_tuple(_histogram.get(), _histogramSize);
    }

    /*
      operations!
    */
    virtual voxels& copy(const voxels& vox, bool deepCopy = false); //turns this object into a copy of vox (shallow by default, deep if requested)
    //subvolume extraction: removes voxels outside of the subvolume specified
    virtual voxels& sub(uint64 off_x, uint64 off_y, uint64 off_z,
			const dimension& subvoldim);
    voxels& fill(double val); //set all voxels to the specified value
    voxels& fillsub(uint64 off_x, uint64 off_y, uint64 off_z,
		    const dimension& subvoldim, double val); //set all voxels in specified subvolume to val
    voxels& map(double min_, double max_); //maps voxels from min to max
    voxels& resize(const dimension& newdim); //resizes this object to the specified dimension using trilinear interpolation
    voxels& bilateralFilter(double radiometricSigma = 200.0, double spatialSigma = 1.5, unsigned int filterRadius = 2);
    //voxels& rotate(double deg_x, double deg_y, double deg_z); //rotates the object about the x,y,z axis
    /*
      compose vox into this object using the specified composite function.  Yes, the offset may be negative.
      Only the voxels that overlap will be subject to the composition function.
    */
    virtual voxels& composite(const voxels& compVox, int64 off_x, int64 off_y, int64 off_z, const composite_function& func);
    
    /*
     * Zeyun's Contrast enhancement: enhances contrast between voxel values. 'resistor' must be a value between 0.0 and 1.0
     * Requres memory to hold the original volume + 6x the original volume using float values for voxels...
     */
    virtual voxels& contrastEnhancement(double resistor = 0.95);

    /*
     * Zeyun's anisotropic diffusion: filters noise but preserves edges more than bilateral filter
     */
    virtual voxels& anisotropicDiffusion(unsigned int iterations = 20);

    /*
     * Dr. Zhang's gdtv filter.
     */
    virtual voxels& gdtvFilter(double parameterq, double lambda, unsigned int iteration, unsigned int neigbour);

    //special access to the multi_array - careful with this!
    // 01/11/2014 - Joe R. - creation
    // 12/09/2024 - Updated to use boost::multi_array
    const boost::shared_ptr<boost::multi_array<unsigned char, 1>>& data() const { return _voxels; }
    boost::shared_ptr<boost::multi_array<unsigned char, 1>>& data() { return _voxels; }
    
    // Direct data pointer access for legacy compatibility
    unsigned char* data_ptr() { return _voxels->data(); }
    const unsigned char* data_ptr() const { return _voxels->data(); }
    
    // Legacy compatibility: convert to shared_array for old VolMagick code
    // Note: This creates a new shared_array that shares ownership with the multi_array  
    boost::shared_array<unsigned char> data_as_shared_array() const {
      // Create shared_array with custom deleter that keeps multi_array alive
      auto multi_copy = _voxels; // Capture shared_ptr to keep it alive
      return boost::shared_array<unsigned char>(
        _voxels->data(),
        [multi_copy](unsigned char*) mutable { multi_copy.reset(); }
      );
    }

  protected:
    void calcMinMax() const;
    void preWrite()
    {
      _histogramDirty = true; //invalidate the histogram

      if(_voxels.unique()) return; //nothing to copy if our voxels are already unique

      try
	{
	  boost::shared_ptr<boost::multi_array<unsigned char, 1>> tmp(_voxels);
	  uint64 size = XDim()*YDim()*ZDim()*voxelSize();
	  _voxels.reset(new boost::multi_array<unsigned char, 1>(boost::extents[size]));
	  memcpy(_voxels->data(), tmp->data(), size);
	}
      catch(std::bad_alloc& e)
	{
	  throw memory_allocation_error("Could not allocate memory for voxels during copy-on-write!");
	}
    }
    void calcHistogram(uint64 size) const;

    boost::shared_ptr<boost::multi_array<unsigned char, 1>> _voxels;

    dimension _dimension;

    data_type _voxelType;

    mutable bool _minIsSet;
    mutable double _min;
    mutable bool _maxIsSet;
    mutable double _max;

    //computed on demand even for const reference so declare as mutable
    mutable boost::shared_ptr<uint64[]> _histogram;
    mutable uint64 _histogramSize;
    mutable bool _histogramDirty;
  };
}

#endif
