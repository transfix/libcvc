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

#include <cvc/volume.h>
#include <cvc/utility.h>
#include <cvc/app.h>

#include <boost/format.hpp>

namespace CVC_NAMESPACE
{
  volume& volume::copy(const volume& vol)
  {
    if(this == &vol)
      return *this;

    voxels::copy(vol);
    boundingBox(vol.boundingBox());
    desc(vol.desc());
    return *this;
  }

  volume& volume::sub(uint64 off_x, uint64 off_y, uint64 off_z,
                      const dimension& subvoldim
#ifdef _MSC_VER 
                        , int brain_damage //avoiding VC++ error C2555
#endif
                          )
  {
    // CRITICAL: Save the original bounding box and spans BEFORE modifying dimensions
    // The bounding box update must use the OLD coordinate system to correctly
    // map voxel indices to object space coordinates.
    double origXMin = XMin(), origYMin = YMin(), origZMin = ZMin();
    double origXSpan = XSpan();
    double origYSpan = YSpan();
    double origZSpan = ZSpan();
    
    voxels::sub(off_x,off_y,off_z,subvoldim);
    
    // Update bounding box using the ORIGINAL Min and spans to map voxel indices
    // to object space coordinates correctly
    double newXMin = origXMin + origXSpan * off_x;
    double newYMin = origYMin + origYSpan * off_y;
    double newZMin = origZMin + origZSpan * off_z;
    double newXMax = newXMin + origXSpan * (subvoldim[0] - 1);
    double newYMax = newYMin + origYSpan * (subvoldim[1] - 1);
    double newZMax = newZMin + origZSpan * (subvoldim[2] - 1);
    
    boundingBox().setMin(newXMin, newYMin, newZMin);
    boundingBox().setMax(newXMax, newYMax, newZMax);
    return *this;
  }

#if 0
  volume& volume::compositeObj(const volume& compVol, double off_x, double off_y, double off_z, const CompositeFunction& func)
  {
    return *this; // finish me!
  }
#endif

  volume& volume::sub(const bounding_box& subvolbox)
  {
    //keep the span of the subvolume as close as possible to the original volume
    if(!subvolbox.isWithin(boundingBox()))
      throw sub_volume_out_of_bounds("The subvolume bounding box must be within the file's bounding box.");
    uint64 off_x = uint64((subvolbox.minx - XMin())/XSpan());
    uint64 off_y = uint64((subvolbox.miny - YMin())/YSpan());
    uint64 off_z = uint64((subvolbox.minz - ZMin())/ZSpan());
    dimension dim;
    dim[0] = uint64((subvolbox.maxx - subvolbox.minx)/XSpan())+1;
    dim[1] = uint64((subvolbox.maxy - subvolbox.miny)/YSpan())+1;
    dim[2] = uint64((subvolbox.maxz - subvolbox.minz)/ZSpan())+1;
    for(int i = 0; i < 3; i++) if(dim[i] == 0) dim[i]=1;
    if(dim[0] + off_x > XDim()) dim[0] = XDim() - off_x;
    if(dim[1] + off_y > YDim()) dim[1] = YDim() - off_y;
    if(dim[2] + off_z > ZDim()) dim[2] = ZDim() - off_z;
    sub(off_x,off_y,off_z,dim);

    //just force the bounding box for now.. this might lead to aliasing errors
    boundingBox(subvolbox);    

    return *this;
  }

  volume& volume::sub(const bounding_box& subvolbox, const dimension& subvoldim)
  {
    thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

    if(!subvolbox.isWithin(boundingBox()))
      throw sub_volume_out_of_bounds("Subvolume bounding box must be within the bounding box of the original volume.");

    volume subvol(_ctx, subvoldim,
                  voxelType(),
                  subvolbox);

    subvol.desc(desc());

    uint64 i,j,k;
    for(k=0; k<subvol.ZDim(); k++)
      {
        for(j=0; j<subvol.YDim(); j++)
          for(i=0; i<subvol.XDim(); i++)
            {
              try
                {
                  subvol(i,j,k, this->interpolate(subvolbox.minx + i*subvol.XSpan(),
                                                  subvolbox.miny + j*subvol.YSpan(),
                                                  subvolbox.minz + k*subvol.ZSpan()));
                }
              catch(index_out_of_bounds& e)
                {
                  //Some really slight floating point errors pop up sometimes...
                  //              fprintf(stderr,"Volume::sub(): caught index_out_of_bounds exception (%s)."
                  //                      " Possible floating point error\n",e.what());
                }
            }
        _ctx.threadProgress(float(k)/float(subvol.ZDim()));
      }

    copy(subvol);
    return *this;
  }

  double volume::interpolate(double obj_x, double obj_y, double obj_z) const
  {
    using namespace boost;

    //double inSpaceX, inSpaceY, inSpaceZ;
    double val[8];
    uint64 resXIndex = 0, resYIndex = 0, resZIndex = 0;
    uint64 ValIndex[8];
    double xPosition = 0, yPosition = 0, zPosition = 0;
    double xRes = 1, yRes = 1, zRes = 1;
    /*
      uint64 i,j,k;
      double x,y,z;
    */

    if(!boundingBox().contains(obj_x,obj_y,obj_z)) 
      throw index_out_of_bounds(str(boost::format("Coordinates are outside of bounding box :: "
                                           "bbox (%f,%f,%f),(%f,%f,%f) coord (%f,%f,%f)")
                                    % boundingBox().minx
                                    % boundingBox().miny
                                    % boundingBox().minz
                                    % boundingBox().maxx
                                    % boundingBox().maxy
                                    % boundingBox().maxz
                                    % obj_x
                                    % obj_y
                                    % obj_z));
    
    resXIndex = uint64((obj_x - XMin())/XSpan());
    resYIndex = uint64((obj_y - YMin())/YSpan());
    resZIndex = uint64((obj_z - ZMin())/ZSpan());

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

    xPosition = obj_x - (double(resXIndex)*XSpan() + XMin());
    yPosition = obj_y - (double(resYIndex)*YSpan() + YMin());
    zPosition = obj_z - (double(resZIndex)*ZSpan() + ZMin());

    xRes = XSpan();
    yRes = YSpan();
    zRes = ZSpan();

    return getTriVal(val, xPosition, yPosition, zPosition, xRes, yRes, zRes);
  }

  volume& volume::resize(const bounding_box& new_bbox)
  {
    thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

    // If bounding boxes are the same, no work needed
    if(_boundingBox == new_bbox) return *this;

    // Call the voxels resize method with old and new bounding boxes
    voxels::resize(_boundingBox, new_bbox);

    // Update our bounding box
    _boundingBox = new_bbox;

    return *this;
  }

  volume& volume::combineWith(const volume& vol, const dimension& dim)
  {
    thread_info ti(_ctx, BOOST_CURRENT_FUNCTION);

    bounding_box combbox = boundingBox() + vol.boundingBox();
    volume combvol(_ctx, dim,voxelType(),combbox);

    for(uint64 k = 0; k < combvol.ZDim(); k++)
      {
        for(uint64 j = 0; j < combvol.YDim(); j++)
          for(uint64 i = 0; i < combvol.XDim(); i++)
            {
              double x = i*combvol.XSpan() + XMin();
              double y = j*combvol.YSpan() + YMin();
              double z = k*combvol.ZSpan() + ZMin();
              
              //TODO: consider using composite algoritms or average these or something...
              if(vol.boundingBox().contains(x,y,z))
                combvol(i,j,k, vol.interpolate(x,y,z));
              else if(boundingBox().contains(x,y,z))
                combvol(i,j,k, interpolate(x,y,z));
            }

        _ctx.threadProgress(float(k)/float(dim[2]));
      }

    (*this) = combvol;
    _ctx.threadProgress(1.0f);
    return (*this);
  }

  volume& volume::combineWith(const volume& vol)
  {
    return combineWith(vol,voxel_dimensions());
  }

  // 12/29/2013 -- Joe R. -- Creation.
  volume& volume::read(const std::string& filename,
		       unsigned int var, unsigned int time,
		       const bounding_box& subvolbox)
  {
    if(!subvolbox.isNull())
      readVolumeFile(ctx(), *this, filename, var, time, subvolbox);
    else
      readVolumeFile(ctx(), *this, filename, var, time);
    return *this;
  }

  // 12/29/2013 -- Joe R. -- Creation.
  volume& volume::write(const std::string& filename)
  {
    createVolumeFile(ctx(), filename,
		     boundingBox(),
		     voxel_dimensions(),
		     std::vector<data_type>(1, voxelType()));
    writeVolumeFile(ctx(), *this, filename);
    return *this;
  }
}
