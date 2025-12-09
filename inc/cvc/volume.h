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

#ifndef __VOLMAGICK_VOLUME_H__
#define __VOLMAGICK_VOLUME_H__

#include <cvc/voxels.h>
#include <cvc/bounding_box.h>

#include <string>

namespace CVC_NAMESPACE
{
  CVC_DEF_EXCEPTION(sub_volume_out_of_bounds);

  /**
   * @class volume
   * @brief Volume data with spatial coordinate system (extends voxels)
   * 
   * The volume class extends voxels by adding a bounding box in object space,
   * enabling spatial operations like interpolation at arbitrary coordinates.
   * 
   * IMPORTANT - MEMORY SEMANTICS:
   * =============================
   * Volume and voxels use SHALLOW COPY semantics via boost::shared_array.
   * 
   * - Copy constructor: volume vol2(vol1);        // Shares underlying data
   * - Assignment operator: vol2 = vol1;           // Shares underlying data
   * - copy() method: vol2.copy(vol1);             // Shares underlying data
   * 
   * All three methods create a NEW volume object but share the SAME underlying
   * voxel data array. Modifications to one volume will affect all copies!
   * 
   * To create a true DEEP COPY with independent data:
   * 
   *   volume vol2(vol1.voxel_dimensions(), vol1.voxelType(), vol1.boundingBox());
   *   for(uint64 i = 0; i < vol1.voxel_dimensions().size(); ++i)
   *     vol2(i, vol1(i));
   *   vol2.desc(vol1.desc());
   * 
   * Or use sub() to extract a subvolume, which creates a new data array:
   * 
   *   volume vol2(vol1);
   *   vol2.sub(0, 0, 0, vol1.voxel_dimensions());  // Full volume deep copy
   * 
   * This shallow copy design enables efficient passing of volumes without
   * copying large data arrays, but requires awareness when modifying shared data.
   */
  class volume : public voxels
  {
  public:
    volume(const dimension& d = dimension(4,4,4), 
	   data_type vt = UChar, 
	   const bounding_box& box = bounding_box(-0.5,-0.5,-0.5,0.5,0.5,0.5)) 
      : voxels(d,vt), _boundingBox(box), _desc("No Name") {}
    volume(const unsigned char *v, 
	   const dimension& d, 
	   data_type vt, 
	   const bounding_box& box = bounding_box(-0.5,-0.5,-0.5,0.5,0.5,0.5))
      : voxels(v,d,vt), _boundingBox(box), _desc("No Name") {}
    volume(const voxels& vox,
	   const bounding_box& box = bounding_box(-0.5,-0.5,-0.5,0.5,0.5,0.5))
      : voxels(vox), _boundingBox(box), _desc("No Name") {}
    volume(const volume& vol)
      : voxels(vol), _boundingBox(vol.boundingBox()), _desc(vol.desc()) {}
    volume(const std::string& filename)
      { read(filename); }
    ~volume() {}

    /*
      Bounding box in object space
     */
    bounding_box& boundingBox() { return _boundingBox; }
    const bounding_box& boundingBox() const { return _boundingBox; }
    void boundingBox(const bounding_box& box) { _boundingBox = box; }

    double XMin() const { return boundingBox().minx; }
    double XMax() const { return boundingBox().maxx; }
    double YMin() const { return boundingBox().miny; }
    double YMax() const { return boundingBox().maxy; }
    double ZMin() const { return boundingBox().minz; }
    double ZMax() const { return boundingBox().maxz; }

    double XSpan() const { return XDim()-1 == 0 ? 0.0 : (boundingBox().maxx-boundingBox().minx)/(XDim()-1); }
    double YSpan() const { return YDim()-1 == 0 ? 0.0 : (boundingBox().maxy-boundingBox().miny)/(YDim()-1); }
    double ZSpan() const { return ZDim()-1 == 0 ? 0.0 : (boundingBox().maxz-boundingBox().minz)/(ZDim()-1); }

    /*
      volume description (used when this object is being saved and the volume
      format supports volume descriptions)
    */
    const std::string& desc() const { return _desc; }
    void desc(const std::string& d) { _desc = d; }

    volume& operator=(const volume& vol) { copy(vol); return *this; }

    bool operator==(const volume& vol)
    {
      return voxels::operator==(vol) &&
        boundingBox() == vol.boundingBox();
    }

    bool operator!=(const volume& vol)
    {
      return !(*this == vol);
    }

    /*
      Operations!
    */
    /**
     * @brief Shallow copy - shares underlying voxel data with vol
     * 
     * WARNING: This is NOT a deep copy! The underlying voxel data array
     * is shared via boost::shared_array. Both volumes will reference the
     * same memory, so modifications to one affect the other.
     * 
     * Use sub() or manual element-by-element copy for deep copies.
     * 
     * @param vol Source volume to copy from
     * @return Reference to this volume
     */
    virtual volume& copy(const volume& vol);
    virtual volume& sub(uint64 off_x, uint64 off_y, uint64 off_z,
			const dimension& subvoldim
#ifdef _MSC_VER
			, int brain_damage = 1 //avoiding VC++ error C2555
#endif
			);
    /*
      compose volumes using object space coordinates.  Makes a duplicate of compVol and resizes it to match
      the grid resolution of this volume, then does normal voxel composition.
    */
    //virtual volume& compositeObj(const volume& compVol, double off_x, double off_y, double off_z, const CompositeFunction& func);

    virtual volume& sub(const bounding_box& subvolbox); //Gets a subvolume from a bounding box.
                                                       //Aims to keep the span of the subvolume
                                                       //as close as possible to the original.

    //Creates a subvolume with a bounding box == subvolbox, and a dimension == subvoldim
    virtual volume& sub(const bounding_box& subvolbox, const dimension& subvoldim);

    //returns a linearly interpolated voxel value for the object coordinates supplied.  The coordinates must
    //be inside the bounding box, or an exception is thrown.
    double interpolate(double obj_x, double obj_y, double obj_z) const;

    //makes this volume into a new volume that contains both this volume and the volume specified, bounding box and all
    //If dimension is specified, this volume will be resized to that dimension
    volume& combineWith(const volume& vol, const dimension& dim);
    volume& combineWith(const volume& vol);

    virtual volume& read(const std::string& filename,
			 unsigned int var = 0, unsigned int time = 0,
			 const bounding_box& subvolbox = bounding_box());
    virtual volume& write(const std::string& filename);

  protected:
    bounding_box _boundingBox;
    std::string _desc;
  };
}

#endif
