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

/* $Id: BilateralFilter.cpp 4742 2011-10-21 22:09:44Z transfix $ */

#include <math.h>
#include <cvc/volmagick.h>
#include <cvc/app.h>

namespace CVC_NAMESPACE
{
  voxels& voxels::bilateralFilter(double radiometricSigma, double spatialSigma, unsigned int filterRadius)
  {
    thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

    int i,j,k,x,y,z,c,index;
    int filterDiameter = filterRadius*2+1;
    double fsample, weight, normalizedDiff, factor;
    double sum, denom;
    bool bool1, bool2;
    double radiometricTable[256];
    double *spatialMask = new double[filterDiameter*filterDiameter*filterDiameter];

    uint64 numSteps = ZDim();

    // Cache original min/max before filtering, as we modify values in-place
    // and need consistent normalization throughout
    double origMin = min();
    double origMax = max();
    double valueRange = origMax - origMin;

    //compute the radiometric table
    for(c=0; c<256; c++)
      {
	factor = c * (valueRange / 255.0);
	radiometricTable[c] = exp((double)(factor*factor)/(-radiometricSigma*radiometricSigma*2.0));
      }

    //compute the spatial mask
    index = 0;
    for (k=-int(filterRadius); k<=int(filterRadius); k++)
      for (j=-int(filterRadius); j<=int(filterRadius); j++)
	for (i=-int(filterRadius); i<=int(filterRadius); i++)
	  spatialMask[index++] = exp((double)(k*k+j*j+i*i)/(-spatialSigma*spatialSigma*2.0));

#ifdef CVC_USING_CUDA
    // Use CUDA kernel if CUDA is enabled and unified memory is available
    if (_using_cuda && _cuda_unified_ptr) {
      try {
        // Create temporary buffer and copy source data
        voxels temp(_ctx, voxel_dimensions(), voxelType());
        temp.copy(*this, true);  // Deep copy first
        temp.enableCUDA(_cuda_device_id);  // Then enable CUDA (migrates to GPU)
        
        // Allocate CUDA unified memory for destination
        voxels result(_ctx, voxel_dimensions(), voxelType());
        result.enableCUDA(_cuda_device_id);
        
        // Launch CUDA kernel for bilateral filtering
        cuda_bilateral_filter(
            temp._cuda_unified_ptr.get(),      // source data
            result._cuda_unified_ptr.get(),    // destination data
            XDim(), YDim(), ZDim(),
            radiometricSigma,
            spatialSigma,
            filterRadius,
            valueRange,
            voxelType());
        
        // Copy the result
        copy(result);
        delete [] spatialMask;
        _ctx.threadProgress(1.0f);
        
        return *this;
      } catch (const cuda_error& e) {
        // Fall back to CPU implementation if CUDA fails
        _ctx.log(2, std::string("CUDA bilateral filter failed: ") + e.what() + ", falling back to CPU");
      }
    }
#endif
    
    // CPU implementation (fallback or when CUDA not available)
    // Use temporary buffer with deep copy to avoid race conditions with OpenMP
    voxels temp(_ctx, voxel_dimensions(), voxelType());
    temp.copy(*this, true);  // Deep copy
    
    // Call preWrite() once BEFORE parallel region to avoid race on _histogramDirty flag
    // and ensure unique voxel data (copy-on-write if needed)
    preWrite();
    byte* data = get_data_ptr();  // Get pointer once for direct writes
    
    for(k=0; k<int(ZDim()); k++)
      {
#ifdef _OPENMP
	#pragma omp parallel for private(i,j,x,y,z,fsample,sum,denom,normalizedDiff,weight,bool1,bool2) schedule(dynamic)
#endif
	for(j=0; j<int(YDim()); j++)
	  for(i=0; i<int(XDim()); i++)
	    {
	      fsample = temp(i,j,k);
	      sum = 0; denom = 0;
	      
	      for(z=0; z<filterDiameter; z++)
		{
		  bool1 = k+z>=int(filterRadius) && k+z<int(ZDim())+int(filterRadius);
		  for(y=0; y<filterDiameter; y++)
		    {
		      bool2 = bool1 && (j+y>=int(filterRadius) && j+y<int(YDim())+int(filterRadius));
		  for(x=0; x<filterDiameter; x++)
			{
			  if(i+x>=int(filterRadius) && i+x<int(XDim())+int(filterRadius) && bool2)
			    {
			      normalizedDiff = fabs(fsample - temp(i+x-filterRadius,j+y-filterRadius,k+z-filterRadius));
			      normalizedDiff /= valueRange;
			      normalizedDiff *= 255.0;
			      weight = radiometricTable[int(normalizedDiff)]*
				spatialMask[z*filterDiameter*filterDiameter+y*filterDiameter+x];
			      denom += weight;
			      sum += weight * temp(i+x-filterRadius,j+y-filterRadius,k+z-filterRadius);
			    }
			}
		    }
		}
	      
	      // Direct write bypassing operator() to avoid preWrite() race
	      uint64 idx = i + j*XDim() + k*XDim()*YDim();
	      double new_val = sum/denom;
	      switch(voxelType())
		{
		case Float:
		  reinterpret_cast<float*>(data)[idx] = static_cast<float>(new_val);
		  break;
		case Double:
		  reinterpret_cast<double*>(data)[idx] = static_cast<double>(new_val);
		  break;
		default:
		  reinterpret_cast<unsigned char*>(data)[idx] = static_cast<unsigned char>(new_val);
		  break;
		}
	    }

        _ctx.threadProgress(float(k)/float(numSteps));
      }

    _ctx.threadProgress(1.0f);
    delete [] spatialMask;
    return *this;
  }
};
