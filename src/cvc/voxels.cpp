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
      _histogramSize(0), _histogramDirty(true)
  {
    try
      {
	_voxels.reset(new unsigned char[XDim()*YDim()*ZDim()*voxelSize()]);
	memset(_voxels.get(),0,XDim()*YDim()*ZDim()*voxelSize());
      }
    catch(std::bad_alloc& e)
      {
	throw memory_allocation_error("Could not allocate memory for voxels!");
      }
  }

  voxels::voxels(const void *v, const dimension& d, data_type vt)
    : _dimension(d), _voxelType(vt), _minIsSet(false), _maxIsSet(false),
      _histogramSize(0), _histogramDirty(true)
  {
    try
      {
	_voxels.reset(new unsigned char[XDim()*YDim()*ZDim()*voxelSize()]);
	memcpy(_voxels.get(),v,XDim()*YDim()*ZDim()*data_type_sizes[_voxelType]);
      }
    catch(std::bad_alloc& e)
      {
	throw memory_allocation_error("Could not allocate memory for voxels!");
      }
  }

  voxels::voxels(const voxels& v)
    : _dimension(v.voxel_dimensions()), 
      _voxelType(v.voxelType()), _minIsSet(false), _maxIsSet(false),
      _histogramSize(0), _histogramDirty(true)
  {
    _voxels = v._voxels;
    if(v.minIsSet() && v.maxIsSet())
      {
	min(v.min());
	max(v.max());
      }
  }

  voxels::~voxels() { }

  // ------------------------
  // voxels::voxel_dimensions
  // ------------------------
  // Purpose:
  //   Changes the dimensions of this voxels dataset.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 08/26/2011 -- Joe R. -- Added voxels argument
  void voxels::voxel_dimensions(const dimension& d, boost::shared_array<unsigned char> vox)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if(d.isNull()) throw null_dimension("Null volume dimension.");

    if(voxel_dimensions() == d && !vox) return;

    //If we have voxels initialized for us, just use those.  Else make one for ourselves
    //that is the right size for dimension d.
    if(vox)
      {
        _dimension = d;
        _voxels = vox;
      }
    else
      {
        voxels bak(*this); //backup voxels into bak

        //allocate for the new dimension
        try
          {
            //in case this throws...
            boost::shared_array<unsigned char> tmp(new unsigned char[d.xdim*d.ydim*d.zdim*voxelSize()]);
            _voxels = tmp;
          }
        catch(std::bad_alloc& e)
          {
            throw memory_allocation_error("Could not allocate memory for voxels!");
          }
    
        _dimension = d;
        memset(_voxels.get(),0,XDim()*YDim()*ZDim()*voxelSize());
        
        //copy the voxels back
        for(uint64 k = 0; k < ZDim() && k < bak.ZDim(); k++)
          for(uint64 j = 0; j < YDim() && j < bak.YDim(); j++)
            for(uint64 i = 0; i < XDim() && i < bak.XDim(); i++)
              (*this)(i,j,k, bak(i,j,k));
      }
  }
  
  void voxels::voxelType(data_type vt)
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    if(voxelType() == vt) return;

    voxels bak(*this); // backup voxels into bak

    //allocate for the new voxel size
    try
      {
	//in case this throws...
	boost::shared_array<unsigned char> tmp(new unsigned char[XDim()*YDim()*ZDim()*data_type_sizes[vt]]);
	_voxels = tmp;
      }
    catch(std::bad_alloc& e)
      {
	throw memory_allocation_error("Could not allocate memory for voxels!");
      }

    _voxelType = vt;
    memset(_voxels.get(),0,XDim()*YDim()*ZDim()*voxelSize());

    //copy the voxels back
    uint64 len = XDim()*YDim()*ZDim();
    for(uint64 i = 0; i<len; i++)
      (*this)(i,bak(i));
  }

  void voxels::calcMinMax() const
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    double val;
    size_t len = XDim()*YDim()*ZDim(), i, count=0;
    size_t slice_len = XDim()*YDim();
    if(len == 0) return;
    val = (*this)(0);
    _min = _max = val;

    switch(voxelType())
      {
      case UChar:
	{
	  register unsigned char v;
	  register unsigned char uchar_min = (unsigned char)(_min);
	  register unsigned char uchar_max = (unsigned char)(_max);
	  for(i=0; i<len; i++)
	    {
	      v = *((unsigned char *)(_voxels.get()+i*sizeof(unsigned char)));
	      if(v < uchar_min) uchar_min = v;
	      if(v > uchar_max) uchar_max = v;
	      if((i % slice_len) == 0)
                {
                  cvcapp.threadProgress(float(count)/float(ZDim()));
                  count++;
                }
	    }
	  _min = double(uchar_min);
	  _max = double(uchar_max);
	  break;
	}
      case UShort:
	{
	  register unsigned short v;
	  register unsigned short ushort_min = (unsigned short)(_min);
	  register unsigned short ushort_max = (unsigned short)(_max);
	  for(i=0; i<len; i++)
	    {
	      v = *((unsigned short *)(_voxels.get()+i*sizeof(unsigned short)));
	      if(v < ushort_min) ushort_min = v;
	      if(v > ushort_max) ushort_max = v;
	      if((i % slice_len) == 0)
                {
                  cvcapp.threadProgress(float(count)/float(ZDim()));
                  count++;
                }
	    }
	  _min = double(ushort_min);
	  _max = double(ushort_max);
	  break;
	}
      case UInt:
	{
	  register unsigned int v;
	  register unsigned int uint_min = (unsigned int)(_min);
	  register unsigned int uint_max = (unsigned int)(_max);
	  for(i=0; i<len; i++)
	    {
	      v = *((unsigned int *)(_voxels.get()+i*sizeof(unsigned int)));
	      if(v < uint_min) uint_min = v;
	      if(v > uint_max) uint_max = v;
	      if((i % slice_len) == 0)
                {
                  cvcapp.threadProgress(float(count)/float(ZDim()));
                  count++;
                }
	    }
	  _min = double(uint_min);
	  _max = double(uint_max);
	  break;
	}
      case Float:
	{
	  register float v;
	  register float float_min = (float)(_min);
	  register float float_max = (float)(_max);
	  for(i=0; i<len; i++)
	    {
	      v = *((float *)(_voxels.get()+i*sizeof(float)));
	      if(v < float_min) float_min = v;
	      if(v > float_max) float_max = v;
	      if((i % slice_len) == 0)
                {
                  cvcapp.threadProgress(float(count)/float(ZDim()));
                  count++;
                }
	    }
	  _min = double(float_min);
	  _max = double(float_max);
	  break;
	}
      case Double:
	{
	  register double v;
	  register double double_min = (double)(_min);
	  register double double_max = (double)(_max);
	  for(i=0; i<len; i++)
	    {
	      v = *((double *)(_voxels.get()+i*sizeof(double)));
	      if(v < double_min) double_min = v;
	      if(v > double_max) double_max = v;
	      if((i % slice_len) == 0)
                {
                  cvcapp.threadProgress(float(count)/float(ZDim()));
                  count++;
                }
	    }
	  _min = double(double_min);
	  _max = double(double_max);
	  break;
	}
      case UInt64:
	{
	  register uint64 v;
	  register uint64 uint64_min = (uint64)(_min);
	  register uint64 uint64_max = (uint64)(_max);
	  for(i=0; i<len; i++)
	    {
	      v = *((uint64 *)(_voxels.get()+i*sizeof(uint64)));
	      if(v < uint64_min) uint64_min = v;
	      if(v > uint64_max) uint64_max = v;
	      if((i % slice_len) == 0)
                {
                  cvcapp.threadProgress(float(count)/float(ZDim()));
                  count++;
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
            uint64 size = vox.XDim() * vox.YDim() * vox.ZDim() * vox.voxelSize();
            _voxels.reset(new unsigned char[size]);
            memcpy(_voxels.get(), vox._voxels.get(), size);
          }
        catch(std::bad_alloc& e)
          {
            throw memory_allocation_error("Could not allocate memory for deep copy of voxels!");
          }
      }
    else
      {
        // Shallow copy: share the underlying data
        _voxels = vox._voxels;
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

    voxels tmp(*this); // back up this object into tmp

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
}

