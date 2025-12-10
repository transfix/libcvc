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

#include <cvc/voxels.h>
#include <cvc/composite_function.h>
#include <cvc/utility.h>

#include <cvc/app.h>

#include <boost/current_function.hpp>

namespace CVC_NAMESPACE
{
  voxels::voxels(const dimension& d, data_type vt) 
    : _dimension(d), _voxelType(vt), _minIsSet(false), _maxIsSet(false),
      _histogram(nullptr), _histogramSize(0), _histogramDirty(true),
      _using_cuda(false), _cuda_device_id(-1)
  {
    try
      {
	uint64 size = XDim()*YDim()*ZDim()*voxelSize();
	_voxels.reset(new unsigned char[size]);
	std::memset(_voxels.get(), 0, size);
      }
    catch(std::bad_alloc& e)
      {
	throw memory_allocation_error("Could not allocate memory for voxels!");
      }
  }

  voxels::voxels(const void *v, const dimension& d, data_type vt)
    : _dimension(d), _voxelType(vt), _minIsSet(false), _maxIsSet(false),
      _histogram(nullptr), _histogramSize(0), _histogramDirty(true),
      _using_cuda(false), _cuda_device_id(-1)
  {
    try
      {
	uint64 size = XDim()*YDim()*ZDim()*voxelSize();
	_voxels.reset(new unsigned char[size]);
	std::memcpy(_voxels.get(), v, size);
      }
    catch(std::bad_alloc& e)
      {
	throw memory_allocation_error("Could not allocate memory for voxels!");
      }
  }

  voxels::voxels(const voxels& v)
    : _dimension(v.voxel_dimensions()), 
      _voxelType(v.voxelType()), _minIsSet(false), _maxIsSet(false),
      _histogram(nullptr), _histogramSize(0), _histogramDirty(true),
      _using_cuda(v._using_cuda), _cuda_device_id(v._cuda_device_id),
      _voxels(v._voxels),
      _cuda_unified_ptr(v._cuda_unified_ptr)
  {
    // Shallow copy: share both CPU and CUDA memory via reference counting
    // boost::shared_array for CPU memory, std::shared_ptr for CUDA unified memory
    
    if(v.minIsSet() && v.maxIsSet())
      {
	min(v.min());
	max(v.max());
      }
  }

  voxels::~voxels() {
    // No explicit cleanup needed:
    // - boost::shared_array automatically frees CPU memory when refcount hits 0
    // - std::shared_ptr with CudaManagedDeleter automatically frees CUDA memory when refcount hits 0
  }

  // ------------------------
  // voxels::voxel_dimensions
  // ------------------------
  // Purpose:
  //   Changes the dimensions of this voxels dataset.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 12/09/2024 -- Joe R. -- Updated for typed 3D arrays
  void voxels::voxel_dimensions(const dimension& d)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if(d.isNull()) throw null_dimension("Null volume dimension.");

    if(voxel_dimensions() == d) return;

    // Shallow copy constructor creates backup sharing memory
    voxels bak(*this);

#ifdef CVC_USING_CUDA
    // Reset CUDA memory reference (shared_ptr will auto-cleanup if last reference)
    if (_using_cuda) {
      _using_cuda = false;
      _cuda_device_id = -1;
      _cuda_unified_ptr.reset();
    }
#endif

    //allocate for the new dimension
    try
      {
        size_t data_size = d.size() * voxelSize();
        _voxels.reset(new unsigned char[data_size]);
        std::memset(_voxels.get(), 0, data_size);
      }
    catch(std::bad_alloc& e)
      {
        throw memory_allocation_error("Could not allocate memory for voxels!");
      }

    _dimension = d;
    
    //copy the voxels back
    for(uint64 k = 0; k < ZDim() && k < bak.ZDim(); k++)
      for(uint64 j = 0; j < YDim() && j < bak.YDim(); j++)
        for(uint64 i = 0; i < XDim() && i < bak.XDim(); i++)
          (*this)(i,j,k, bak(i,j,k));
  }
  
  void voxels::voxelType(data_type vt)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if(voxelType() == vt) return;

    voxels bak(*this); // backup voxels into bak
    _voxelType = vt;

    //allocate for the new voxel type
    try
      {
	uint64 size = XDim()*YDim()*ZDim()*voxelSize();
	_voxels.reset(new unsigned char[size]);
      }
    catch(std::bad_alloc& e)
      {
	throw memory_allocation_error("Could not allocate memory for voxels!");
      }

    //copy the voxels back with type conversion
    uint64 len = XDim()*YDim()*ZDim();
    for(uint64 i = 0; i<len; i++)
      (*this)(i,bak(i));
  }

  void voxels::calcMinMax() const
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    double val;
    size_t len = XDim()*YDim()*ZDim();
    if(len == 0) return;
    val = (*this)(0, 0, 0);
    _min = _max = val;

    switch(voxelType())
      {
      case UChar:
	{
	  unsigned char uchar_min = static_cast<unsigned char>(_min);
	  unsigned char uchar_max = static_cast<unsigned char>(_max);
	  unsigned char* data = reinterpret_cast<unsigned char*>(_voxels.get());
	  for(size_t i=0; i<len; i++)
	    {
	      unsigned char v = data[i];
	      if(v < uchar_min) uchar_min = v;
	      if(v > uchar_max) uchar_max = v;
	      if((i % (XDim()*YDim())) == 0)
                {
                  cvcapp.threadProgress(float(i/(XDim()*YDim()))/float(ZDim()));
                }
	    }
	  _min = double(uchar_min);
	  _max = double(uchar_max);
	  break;
	}
      case UShort:
	{
	  unsigned short ushort_min = static_cast<unsigned short>(_min);
	  unsigned short ushort_max = static_cast<unsigned short>(_max);
	  unsigned short* data = reinterpret_cast<unsigned short*>(_voxels.get());
	  for(size_t i=0; i<len; i++)
	    {
	      unsigned short v = data[i];
	      if(v < ushort_min) ushort_min = v;
	      if(v > ushort_max) ushort_max = v;
	      if((i % (XDim()*YDim())) == 0)
                {
                  cvcapp.threadProgress(float(i/(XDim()*YDim()))/float(ZDim()));
                }
	    }
	  _min = double(ushort_min);
	  _max = double(ushort_max);
	  break;
	}
      case UInt:
	{
	  unsigned int uint_min = static_cast<unsigned int>(_min);
	  unsigned int uint_max = static_cast<unsigned int>(_max);
	  unsigned int* data = reinterpret_cast<unsigned int*>(_voxels.get());
	  for(size_t i=0; i<len; i++)
	    {
	      unsigned int v = data[i];
	      if(v < uint_min) uint_min = v;
	      if(v > uint_max) uint_max = v;
	      if((i % (XDim()*YDim())) == 0)
                {
                  cvcapp.threadProgress(float(i/(XDim()*YDim()))/float(ZDim()));
                }
	    }
	  _min = double(uint_min);
	  _max = double(uint_max);
	  break;
	}
      case Float:
	{
	  float float_min = static_cast<float>(_min);
	  float float_max = static_cast<float>(_max);
	  float* data = reinterpret_cast<float*>(_voxels.get());
	  for(size_t i=0; i<len; i++)
	    {
	      float v = data[i];
	      if(v < float_min) float_min = v;
	      if(v > float_max) float_max = v;
	      if((i % (XDim()*YDim())) == 0)
                {
                  cvcapp.threadProgress(float(i/(XDim()*YDim()))/float(ZDim()));
                }
	    }
	  _min = double(float_min);
	  _max = double(float_max);
	  break;
	}
      case Double:
	{
	  double double_min = _min;
	  double double_max = _max;
	  double* data = reinterpret_cast<double*>(_voxels.get());
	  for(size_t i=0; i<len; i++)
	    {
	      double v = data[i];
	      if(v < double_min) double_min = v;
	      if(v > double_max) double_max = v;
	      if((i % (XDim()*YDim())) == 0)
                {
                  cvcapp.threadProgress(float(i/(XDim()*YDim()))/float(ZDim()));
                }
	    }
	  _min = double_min;
	  _max = double_max;
	  break;
	}
      case UInt64:
	{
	  uint64 uint64_min = static_cast<uint64>(_min);
	  uint64 uint64_max = static_cast<uint64>(_max);
	  uint64* data = reinterpret_cast<uint64*>(_voxels.get());
	  for(size_t i=0; i<len; i++)
	    {
	      uint64 v = data[i];
	      if(v < uint64_min) uint64_min = v;
	      if(v > uint64_max) uint64_max = v;
	      if((i % (XDim()*YDim())) == 0)
                {
                  cvcapp.threadProgress(float(i/(XDim()*YDim()))/float(ZDim()));
                }
	    }
	  _min = double(uint64_min);
	  _max = double(uint64_max);
	  break;
	}
      }

    _minIsSet = _maxIsSet = true;
    cvcapp.threadProgress(1.0f);
  }

  void voxels::calcHistogram(uint64 size) const
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if(!_histogramDirty && _histogramSize == size) return;

    _histogramSize = size;
    _histogram.reset(new uint64[size]);
    memset(_histogram.get(),0,sizeof(uint64)*size);

    for(uint64 k = 0; k<ZDim(); k++)
      {
	for(uint64 j = 0; j<YDim(); j++)
	  for(uint64 i = 0; i<XDim(); i++)
	    {
	      uint64 offset = 
		uint64((((*this)(i,j,k) - min())/(max() - min())) * double(size-1));
	      _histogram[offset]++;
	    }
        cvcapp.threadProgress(float(k)/float(ZDim()));
      }

    _histogramDirty = false;
    cvcapp.threadProgress(1.0f);
  }

  double voxels::min(uint64 off_x, uint64 off_y, uint64 off_z,
		     const dimension& dim) const
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    double val;
    uint64 i,j,k;
    val = (*this)(0,0,0);
    for(k=0; k<dim[2]; k++)
      {
	for(j=0; j<dim[1]; j++)
	  for(i=0; i<dim[0]; i++)
	    if(val > (*this)(i+off_x,j+off_y,k+off_z))
	      val = (*this)(i+off_x,j+off_y,k+off_z);
        cvcapp.threadProgress(float(k)/float(dim[2]));
      }
    
    cvcapp.threadProgress(1.0f);
    return val;
  }

  double voxels::max(uint64 off_x, uint64 off_y, uint64 off_z,
		     const dimension& dim) const
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    double val;
    uint64 i,j,k;
    val = (*this)(0,0,0);
    for(k=0; k<dim[2]; k++)
      {
	for(j=0; j<dim[1]; j++)
	  for(i=0; i<dim[0]; i++)
	    if(val < (*this)(i+off_x,j+off_y,k+off_z))
	      val = (*this)(i+off_x,j+off_y,k+off_z);
        cvcapp.threadProgress(float(k)/float(dim[2]));        
      }
    
    cvcapp.threadProgress(1.0f);
    return val;
  }
  
  voxels& voxels::copy(const voxels& vox, bool deepCopy)
  {
    if(this == &vox)
      return *this;

    _voxelType = vox._voxelType;
    _dimension = vox._dimension;
    
    if(deepCopy)
      {
        // Deep copy: allocate new memory and copy data
        try
          {
            size_t data_size = XDim()*YDim()*ZDim()*voxelSize();
            _voxels.reset(new unsigned char[data_size]);
            std::memcpy(_voxels.get(), vox._voxels.get(), data_size);
            
#ifdef CVC_USING_CUDA
            // Deep copy doesn't preserve CUDA state - new independent copy
            _using_cuda = false;
            _cuda_device_id = -1;
            _cuda_unified_ptr.reset(); // Release reference to CUDA memory
#endif
          }
        catch(std::bad_alloc& e)
          {
            throw memory_allocation_error("Could not allocate memory for deep copy of voxels!");
          }
      }
    else
      {
        // Shallow copy: share the underlying data via reference counting
        _voxels = vox._voxels;
        
#ifdef CVC_USING_CUDA
        // Shallow copy preserves CUDA state and shares CUDA memory
        _using_cuda = vox._using_cuda;
        _cuda_device_id = vox._cuda_device_id;
        _cuda_unified_ptr = vox._cuda_unified_ptr; // shared_ptr handles reference counting
#endif
      }
    
    if(vox.minIsSet() && vox.maxIsSet())
      {
	min(vox.min());
	max(vox.max());
      }
    else
      unsetMinMax();

    _histogram = vox._histogram;
    _histogramSize = vox._histogramSize;
    _histogramDirty = vox._histogramDirty;

    return *this;
  }

  voxels& voxels::sub(uint64 off_x, uint64 off_y, uint64 off_z,
		      const dimension& subvoldim)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if(off_x+subvoldim[0]-1 >= voxel_dimensions()[0] || 
       off_y+subvoldim[1]-1 >= voxel_dimensions()[1] || 
       off_z+subvoldim[2]-1 >= voxel_dimensions()[2])
      throw index_out_of_bounds("Subvolume offset and/or dimension is out of bounds");

    // Deep copy constructor creates independent backup
    voxels tmp(*this);

    voxel_dimensions(subvoldim); // change this object's dimension to the subvolume dimension

    //copy the subvolume voxels
    for(uint64 k=0; k<voxel_dimensions()[2]; k++)
      {
	for(uint64 j=0; j<voxel_dimensions()[1]; j++)
	  for(uint64 i=0; i<voxel_dimensions()[0]; i++)
	    (*this)(i,j,k,tmp(i+off_x,j+off_y,k+off_z));
        cvcapp.threadProgress(float(k)/float(voxel_dimensions()[2]));
      }

    cvcapp.threadProgress(1.0f);
    return *this;
  }

  voxels& voxels::fill(double val)
  {
    return fillsub(0,0,0,voxel_dimensions(),val);
  }

  voxels& voxels::fillsub(uint64 off_x, uint64 off_y, uint64 off_z,
			  const dimension& subvoldim, double val)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if(off_x+subvoldim[0]-1 >= voxel_dimensions()[0] || 
       off_y+subvoldim[1]-1 >= voxel_dimensions()[1] || 
       off_z+subvoldim[2]-1 >= voxel_dimensions()[2])
      throw index_out_of_bounds("Subvolume offset and/or dimension is out of bounds");

    for(uint64 k=0; k<subvoldim[2]; k++)
      {
	for(uint64 j=0; j<subvoldim[1]; j++)
	  for(uint64 i=0; i<subvoldim[0]; i++)
	    (*this)(i+off_x,j+off_y,k+off_z,val);
        cvcapp.threadProgress(float(k)/float(subvoldim[2]));
      }

    cvcapp.threadProgress(1.0f);
    return *this;
  }

  voxels& voxels::map(double min_, double max_)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    uint64 len = XDim()*YDim()*ZDim(), count=0;
    for(uint64 i=0; i<len; i++)
      {
	(*this)(i,min_ + (((*this)(i) - min())/(max() - min()))*(max_ - min_));
	if((i % (XDim()*YDim())) == 0)
          {
            cvcapp.threadProgress(float(count)/float(ZDim()));
            count++;
          }
      }
    min(min_); max(max_); // set the new min and max
    return *this;
  }

  voxels& voxels::resize(const dimension& newdim)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    double inSpaceX, inSpaceY, inSpaceZ;
    double val[8];
    uint64 resXIndex = 0, resYIndex = 0, resZIndex = 0;
    uint64 ValIndex[8];
    double xPosition = 0, yPosition = 0, zPosition = 0;
    double xRes = 0, yRes = 0, zRes = 0;
    uint64 i,j,k;
    double x,y,z;

    if(newdim.isNull()) throw null_dimension("Null voxels dimension.");

    if(voxel_dimensions() == newdim) return *this; //nothing needs to be done

    voxels newvox(newdim,voxelType());

    //we require a dimension of at least 2^3 
    if(newdim < dimension(2,2,2)) 
      {
	//resize this object as if it was 2x2x2
	resize(dimension(2,2,2));

	//copy it into newvox
	newvox.copy(*this);

	//change this object's dimension to the real dimension (destroying voxel values, hence the backup)
	voxel_dimensions(newdim);

	for(k=0; k<ZDim(); k++)
	  for(j=0; j<YDim(); j++)
	    for(i=0; i<XDim(); i++)
	      (*this)(i,j,k,newvox(i,j,k));

	return *this;
      }

    // inSpace calculation
    inSpaceX = (double)(voxel_dimensions()[0]-1)/(newdim[0]-1);
    inSpaceY = (double)(voxel_dimensions()[1]-1)/(newdim[1]-1);
    inSpaceZ = (double)(voxel_dimensions()[2]-1)/(newdim[2]-1);

    for(k = 0; k < newvox.ZDim(); k++)
      {
	z = double(k)*inSpaceZ;
	resZIndex = uint64(z);
	zPosition = z - uint64(z);
	zRes = 1;
	
	for(j = 0; j < newvox.YDim(); j++)
	  {
	    y = double(j)*inSpaceY;
	    resYIndex = uint64(y);
	    yPosition = y - uint64(y);
	    yRes =  1;

	    for(i = 0; i < newvox.XDim(); i++)
	      {
		x = double(i)*inSpaceX;
		resXIndex = uint64(x);
		xPosition = x - uint64(x);
		xRes = 1;

		// find index to get eight voxel values
		ValIndex[0] = resZIndex*voxel_dimensions()[0]*voxel_dimensions()[1] + resYIndex*voxel_dimensions()[0] + resXIndex;
		ValIndex[1] = ValIndex[0] + 1;
		ValIndex[2] = resZIndex*voxel_dimensions()[0]*voxel_dimensions()[1] + (resYIndex+1)*voxel_dimensions()[0] + resXIndex;
		ValIndex[3] = ValIndex[2] + 1;
		ValIndex[4] = (resZIndex+1)*voxel_dimensions()[0]*voxel_dimensions()[1] + resYIndex*voxel_dimensions()[0] + resXIndex;
		ValIndex[5] = ValIndex[4] + 1;
		ValIndex[6] = (resZIndex+1)*voxel_dimensions()[0]*voxel_dimensions()[1] + (resYIndex+1)*voxel_dimensions()[0] + resXIndex;
		ValIndex[7] = ValIndex[6] + 1;

		if(resXIndex>=voxel_dimensions()[0]-1)
		  {
		    ValIndex[1] = ValIndex[0];
		    ValIndex[3] = ValIndex[2];
		    ValIndex[5] = ValIndex[4];
		    ValIndex[7] = ValIndex[6];
		  }
		if(resYIndex>=voxel_dimensions()[1]-1)
		  {
		    ValIndex[2] = ValIndex[0];
		    ValIndex[3] = ValIndex[1];
		    ValIndex[6] = ValIndex[4];
		    ValIndex[7] = ValIndex[5];
		  }
		if(resZIndex>=voxel_dimensions()[2]-1) 
		  {
		    ValIndex[4] = ValIndex[0];
		    ValIndex[5] = ValIndex[1];
		    ValIndex[6] = ValIndex[2];
		    ValIndex[7] = ValIndex[3];
		  }

		for(int Index = 0; Index < 8; Index++) 
		  val[Index] = (*this)(ValIndex[Index]);
		  
		newvox(i,j,k,
		       getTriVal(val, xPosition, yPosition, zPosition, xRes, yRes, zRes));
	      }
	  }

        cvcapp.threadProgress(float(k)/float(newvox.ZDim()));
      }

    copy(newvox); //make this into a copy of the interpolated voxels
    cvcapp.threadProgress(1.0f);

    return *this;
  }

  voxels& voxels::composite(const voxels& compVox, int64 off_x, int64 off_y, int64 off_z, const composite_function& func)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    uint64 i,j,k;

    for(k=0; k<compVox.ZDim(); k++)
      {
	for(j=0; j<compVox.YDim(); j++)
	  for(i=0; i<compVox.XDim(); i++)
	    if((int64(i)+off_x >= 0) && (int64(i)+off_x < int64(XDim())) &&
	       (int64(j)+off_y >= 0) && (int64(j)+off_y < int64(YDim())) &&
	       (int64(k)+off_z >= 0) && (int64(k)+off_z < int64(ZDim())))
	      (*this)(int64(i) + off_x, int64(j) + off_y, int64(k) + off_z,
		      func(compVox,i,j,k,
			   *this, int64(i) + off_x, int64(j) + off_y, int64(k) + off_z));
        cvcapp.threadProgress(float(k)/float(compVox.ZDim()));
      }

    cvcapp.threadProgress(1.0f);
    return *this;
  }

  // ============================================================================
  // CUDA Unified Memory Support
  // ============================================================================

  bool voxels::cuda_available() {
    return cuda_device_manager::cuda_available();
  }

  int voxels::cuda_device_count() {
    return cuda_device_manager::device_count();
  }

  std::vector<gpu_device_info> voxels::get_gpu_info() {
    return cuda_device_manager::get_device_info();
  }

  int voxels::get_current_gpu() {
    return cuda_device_manager::get_current_device();
  }

  void voxels::set_current_gpu(int device_id) {
    cuda_device_manager::set_current_device(device_id);
  }

  void voxels::enableCUDA(int device_id) {
#ifdef CVC_USING_CUDA
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if (_using_cuda) {
      // Already using CUDA - maybe switch devices?
      if (device_id >= 0 && device_id != _cuda_device_id) {
        switchGPU(device_id);
      }
      return;
    }

    if (!cuda_available()) {
      throw cuda_not_available("CUDA not available on this system");
    }

    // Use current device if not specified
    if (device_id < 0) {
      device_id = cuda_device_manager::get_current_device();
      if (device_id < 0) device_id = 0; // Default to device 0
    }

    // Validate device
    if (device_id >= cuda_device_count()) {
      throw cuda_error("Invalid CUDA device ID");
    }

    if (!cuda_device_manager::supports_unified_memory(device_id)) {
      throw cuda_error("Device does not support unified memory");
    }

    // Migrate data to CUDA unified memory
    migrate_to_cuda(device_id);

    _using_cuda = true;
    _cuda_device_id = device_id;

    cvcapp.log(3, "CUDA unified memory enabled on device " + 
               std::to_string(device_id));
#else
    throw cuda_not_available("CVC was not compiled with CUDA support");
#endif
  }

  void voxels::disableCUDA() {
#ifdef CVC_USING_CUDA
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if (!_using_cuda) {
      return; // Already disabled
    }

    // Migrate data back to system RAM
    migrate_from_cuda();

    _using_cuda = false;
    _cuda_device_id = -1;

    cvcapp.log(3, "CUDA unified memory disabled");
#endif
  }

  void voxels::switchGPU(int new_device_id) {
#ifdef CVC_USING_CUDA
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if (!_using_cuda) {
      // Not currently using CUDA, just enable on the new device
      enableCUDA(new_device_id);
      return;
    }

    if (new_device_id == _cuda_device_id) {
      return; // Already on this device
    }

    if (!cuda_available() || new_device_id >= cuda_device_count()) {
      throw cuda_error("Invalid CUDA device ID");
    }

    if (!cuda_device_manager::supports_unified_memory(new_device_id)) {
      throw cuda_error("Target device does not support unified memory");
    }

    int old_device = _cuda_device_id;
    
    // Check if peer access is possible
    bool can_peer_access = cuda_device_manager::can_access_peer(new_device_id, old_device);
    
    if (can_peer_access) {
      // Enable peer access for direct GPU-to-GPU copy
      try {
        cuda_device_manager::enable_peer_access(new_device_id, old_device);
      } catch (...) {
        // Peer access failed, fall back to host copy
        can_peer_access = false;
      }
    }

    // Save old data pointer
    byte* old_data = get_data_ptr();
    uint64 data_size = XDim() * YDim() * ZDim() * voxelSize();

    // Allocate new memory on target device
    int current_device = cuda_device_manager::get_current_device();
    cuda_device_manager::set_current_device(new_device_id);
    
    try {
      allocate_cuda_memory(_voxelType);
    } catch (...) {
      cuda_device_manager::set_current_device(current_device);
      throw;
    }

    byte* new_data = get_data_ptr();

    if (can_peer_access) {
      // Direct peer-to-peer copy
      CUDA_CHECK(cudaMemcpyPeer(new_data, new_device_id, old_data, old_device, data_size));
      cvcapp.log(3, "Performed peer-to-peer GPU copy from device " +
                 std::to_string(old_device) + " to device " + std::to_string(new_device_id));
    } else {
      // Copy through host memory
      std::vector<byte> temp_buffer(data_size);
      
      // Set to old device and copy to host
      cuda_device_manager::set_current_device(old_device);
      CUDA_CHECK(cudaMemcpy(temp_buffer.data(), old_data, data_size, cudaMemcpyDeviceToHost));
      
      // Set to new device and copy from host
      cuda_device_manager::set_current_device(new_device_id);
      CUDA_CHECK(cudaMemcpy(new_data, temp_buffer.data(), data_size, cudaMemcpyHostToDevice));
      
      cvcapp.log(3, "Performed host-mediated GPU copy from device " +
                 std::to_string(old_device) + " to device " + std::to_string(new_device_id));
    }

    // Restore original device
    cuda_device_manager::set_current_device(current_device);

    _cuda_device_id = new_device_id;
#else
    throw cuda_not_available("CVC was not compiled with CUDA support");
#endif
  }

  void voxels::allocate_cuda_memory(data_type vt) {
#ifdef CVC_USING_CUDA
    // Reset any existing CUDA memory reference (shared_ptr will auto-cleanup if last reference)
    _cuda_unified_ptr.reset();
    
    uint64 size = XDim() * YDim() * ZDim();
    uint64 byte_size = size * voxelSize();
    
    // Allocate CUDA unified memory
    void* raw_ptr = nullptr;
    CUDA_CHECK(cudaMallocManaged(&raw_ptr, byte_size));
    
    // Wrap in shared_ptr with custom deleter for automatic cleanup
    _cuda_unified_ptr = std::shared_ptr<void>(raw_ptr, CudaManagedDeleter());
    
    // Initialize to zero
    CUDA_CHECK(cudaMemset(_cuda_unified_ptr.get(), 0, byte_size));
#endif
  }

  void voxels::free_cuda_memory() {
#ifdef CVC_USING_CUDA
    // Reset shared_ptr (will call CudaManagedDeleter if this is the last reference)
    _cuda_unified_ptr.reset();
#endif
  }

  void voxels::migrate_to_cuda(int device_id) {
#ifdef CVC_USING_CUDA
    // Set device
    int old_device = cuda_device_manager::get_current_device();
    cuda_device_manager::set_current_device(device_id);

    // Allocate CUDA unified memory and copy data
    uint64 data_size = XDim() * YDim() * ZDim() * voxelSize();
    allocate_cuda_memory(_voxelType);
    std::memcpy(_cuda_unified_ptr.get(), _voxels.get(), data_size);
    
    // Synchronize to ensure data is uploaded
    CUDA_CHECK(cudaDeviceSynchronize());

    // Restore device
    cuda_device_manager::set_current_device(old_device);
#endif
  }

  void voxels::migrate_from_cuda() {
#ifdef CVC_USING_CUDA
    if (!_cuda_unified_ptr) {
      return; // Nothing to migrate
    }
    
    // Ensure data is synchronized to host
    if (_cuda_device_id >= 0) {
      int old_device = cuda_device_manager::get_current_device();
      cuda_device_manager::set_current_device(_cuda_device_id);
      CUDA_CHECK(cudaDeviceSynchronize());
      cuda_device_manager::set_current_device(old_device);
    }
    
    // Copy CUDA data back to system memory
    uint64 data_size = XDim() * YDim() * ZDim() * voxelSize();
    std::memcpy(_voxels.get(), _cuda_unified_ptr.get(), data_size);

    // Free CUDA memory (reset shared_ptr)
    free_cuda_memory();
#endif
  }
}

