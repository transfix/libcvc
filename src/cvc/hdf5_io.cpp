/*
  Copyright 2007-2011 The University of Texas at Austin
  
  Authors: Jose Rivera <transfix@ices.utexas.edu>
  Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of Volume Rover.

  Volume Rover is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  Volume Rover is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with iotree; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <cvc/volmagick.h>
#include <cvc/endians.h>
#include <cvc/utility.h>
#include <cvc/hdf5_utils.h>

#if defined(WIN32)
#include <cpp/H5PredType.h>
#include <cpp/H5Cpp.h>
#else
#include <H5PredType.h>
#include <H5Cpp.h>
#endif

#include <boost/current_function.hpp>
#include <boost/format.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/array.hpp>

#include <iostream>
#include <algorithm>
#include <limits>
#include <cmath>

#ifndef CVC_VERSION_STRING
#define CVC_VERSION_STRING "1.0"
#endif

namespace
{
  // --------------
  // build_hierarchy
  // --------------
  // Purpose:
  //   Thread for building the multi-res hierarchy for a volume dataset
  // ---- Change History ----
  // 09/09/2011 -- Joe R. -- Creation.
  // 09/11/2011 -- Joe R. -- Finishes hierarchy calculation without throwing an exception
  // 09/18/2011 -- Joe R. -- Using property volmagick.hdf5_io.buildhierarchy.current to report
  //                         info about the hierarchy dataset currently being processed.  This
  //                         gives clients a chance to update themselves when more data is
  //                         available.
  // 09/30/2011 -- Joe R. -- Counting the number of steps for useful thread progress reporting.
  //                         Also added chunk_size property.
  // 10/09/2011 -- Joe R. -- Using startThread
  // 01/17/2014 -- Joe R. -- using TIME_UTC_
  class build_hierarchy
  {
  public:
    build_hierarchy(const std::string& threadKey,
		    const std::string& hdf5_filename,
		    const std::string& hdf5_volumeDataSet)
      : _threadKey(threadKey),
        _hdf5_filename(hdf5_filename),
        _hdf5_volumeDataSet(hdf5_volumeDataSet) {}

    build_hierarchy(const build_hierarchy& t)
      : _threadKey(t._threadKey),
        _hdf5_filename(t._hdf5_filename),
        _hdf5_volumeDataSet(t._hdf5_volumeDataSet) {}

    build_hierarchy& operator=(const build_hierarchy& t)
    {
      _threadKey = t._threadKey;
      _hdf5_filename = t._hdf5_filename;
      _hdf5_volumeDataSet = t._hdf5_volumeDataSet;
      return *this;
    }

    //lazy way to count the number of steps
    CVC_NAMESPACE::uint64 countNumSteps(const CVC_NAMESPACE::dimension& fullDim, 
                                        const CVC_NAMESPACE::bounding_box& bbox)
    {
      using namespace CVC_NAMESPACE;

      dimension prevDim(fullDim);
      uint64 steps = 0;
      while(1)
        {
          uint64 maxdim = std::max(prevDim.xdim,std::max(prevDim.ydim,prevDim.zdim));
          maxdim = upToPowerOfTwo(maxdim) >> 1; //power of 2 less than maxdim
          dimension curDim(maxdim,maxdim,maxdim);
          for(int i = 0; i < 3; i++)
            curDim[i] = std::min(curDim[i],prevDim[i]);
          if(curDim.size()==1) break; //we're done if the dims hit 1
          
          {
            dimension targetDim = curDim;
            const uint64 maxdim_size = 
              cvcapp.properties<uint64>("volmagick.hdf5_io.buildhierarchy.chunk_size");
            boost::array<double,3> theSize =
              {
                maxdim_size*bbox.XSpan(fullDim),
                maxdim_size*bbox.YSpan(fullDim),
                maxdim_size*bbox.ZSpan(fullDim)
              };
            for(double off_z = bbox.minz;
                off_z < bbox.maxz;
                off_z += theSize[2])
              for(double off_y = bbox.miny;
                  off_y < bbox.maxy;
                  off_y += theSize[1])
                for(double off_x = bbox.minx;
                    off_x < bbox.maxx;
                    off_x += theSize[0])
                  steps++;
          }

          prevDim = curDim;
        }
      return steps;
    }

    void operator()()
    {
      using namespace CVC_NAMESPACE;
      using namespace hdf5_utils;
      using namespace boost;

      thread_feedback feedback;

      //read/write 128^3 chunks by default
      if(!cvcapp.hasProperty("volmagick.hdf5_io.buildhierarchy.chunk_size"))
        cvcapp.properties("volmagick.hdf5_io.buildhierarchy.chunk_size",uint64(128));

      //Sleep for a second before beginning so we don't thrash about
      //if writeVolumeFile is called several times in succession.
      {
        thread_info ti("sleeping");
        boost::xtime xt;
        boost::xtime_get( &xt, boost::TIME_UTC_ );
        xt.sec++;
        boost::thread::sleep( xt );
      }

      try
        {
          dimension fullDim = getObjectDimension(_hdf5_filename,_hdf5_volumeDataSet);
          bounding_box bbox = getObjectBoundingBox(_hdf5_filename,_hdf5_volumeDataSet);
          dimension prevDim(fullDim);

          uint64 numSteps = countNumSteps(fullDim,bbox);
          uint64 steps = 0;

          while(1)
            {
              uint64 maxdim = std::max(prevDim.xdim,std::max(prevDim.ydim,prevDim.zdim));
              maxdim = upToPowerOfTwo(maxdim) >> 1; //power of 2 less than maxdim
              dimension curDim(maxdim,maxdim,maxdim);
              for(int i = 0; i < 3; i++)
                curDim[i] = std::min(curDim[i],prevDim[i]);
              if(curDim.size()==1) break; //we're done if the dims hit 1

              std::string hier_volume_name =
                str(format("%1%_%2%x%3%x%4%") 
                    % _hdf5_volumeDataSet
                    % curDim[0] % curDim[1] % curDim[2]);

              int isDirty = 0;
              getAttribute(_hdf5_filename, hier_volume_name, "dirty", isDirty);
              if(isDirty)
                {
                  cvcapp.log(1,str(format("%1% :: computing %2%\n")
                                   % BOOST_CURRENT_FUNCTION
                                   % hier_volume_name));

                  {
                    dimension targetDim = curDim;
                    const uint64 maxdim_size = 
                      cvcapp.properties<uint64>("volmagick.hdf5_io.buildhierarchy.chunk_size");
                    boost::array<double,3> theSize =
                      {
                        maxdim_size*bbox.XSpan(fullDim),
                        maxdim_size*bbox.YSpan(fullDim),
                        maxdim_size*bbox.ZSpan(fullDim)
                      };
                    for(double off_z = bbox.minz;
                        off_z < bbox.maxz;
                        off_z += theSize[2])
                      {
                        for(double off_y = bbox.miny;
                            off_y < bbox.maxy;
                            off_y += theSize[1])
                          for(double off_x = bbox.minx;
                              off_x < bbox.maxx;
                              off_x += theSize[0])
                            {
                              volume vol(cvcapp);
                              bounding_box subvolbox(
						     off_x,off_y,off_z,
						     std::min(off_x+theSize[0],bbox.maxx),
						     std::min(off_y+theSize[1],bbox.maxy),
						     std::min(off_z+theSize[2],bbox.maxz)
						     );
                              readVolumeFile(
					     vol,
					     _hdf5_filename + "|" + _hdf5_volumeDataSet,
					     0,0,subvolbox
					     );
                              writeVolumeFile(
					      vol,
					      _hdf5_filename + "|" + hier_volume_name,
					      0,0,subvolbox
					      );
                              cvcapp.threadProgress(float(++steps)/float(numSteps));
                            }
                      }
                  }

                  //Done, now mark this one clean.
                  setAttribute(_hdf5_filename, hier_volume_name, "dirty", 0);

                  cvcapp.properties("volmagick.hdf5_io.buildhierarchy.latest",
                                    _hdf5_filename + "|" + hier_volume_name);

                }
              else
                {
                  cvcapp.log(1,str(format("%1% :: %2% not dirty\n")
                                   % BOOST_CURRENT_FUNCTION
                                   % hier_volume_name));
                }

              prevDim = curDim;
            }

          cvcapp.threadProgress(1.0f);
        }
      catch(boost::thread_interrupted&)
        {
          cvcapp.log(6,str(format("%1% :: thread %2% interrupted\n")
                           % BOOST_CURRENT_FUNCTION
                           % cvcapp.threadKey()));
        }
      catch(CVC_NAMESPACE::exception& e)
        {
          cvcapp.log(1,str(format("%1% :: ERROR :: %2%\n")
                           % BOOST_CURRENT_FUNCTION
                           % e.what()));
        }
    }

    static void start(const std::string& threadKey,
                      const std::string& hdf5_filename,
                      const std::string& hdf5_volumeDataSet)
    {
      cvcapp.startThread(threadKey,
                         build_hierarchy(threadKey,
					 hdf5_filename,
					 hdf5_volumeDataSet));
    }

  protected:
    std::string _threadKey;
    std::string _hdf5_filename;
    std::string _hdf5_volumeDataSet;
  };
}

namespace CVC_NAMESPACE
{
  // -------
  // hdf5_io
  // -------
  // Purpose: 
  //   Provides HDF5 file support.
  // ---- Change History ----
  // 12/04/2009 -- Joe R. -- Creation.
  struct hdf5_io : public volume_file_io
  {
    /*
     * CVC hdf5 schema -- Joe R. -- 01/04/2010
     *  / - root
     *  |_ cvc/ - root cvc data hierarchy
     *     |_ geometry/ - placeholder, define fully later
     *     |_ transfer_functions/ - placeholder, define fully later
     *     |_ volumes/
     *        |_ <volume name> - volume group containing image or volume data
     *           |               Default volume name is 'volume'.  This group has the following attribs:
     *           |               - VolMagick_version (uint64)
     *           |               - XMin, YMin, ZMin,
     *           |                 XMax, YMax, ZMax  (double) - bounding box
     *           |               - XDim, YDim, ZDim (uint64) - volume dimensions
     *           |               - voxelTypes (uint64 array) - the type of each variable
     *           |               - numVariables (uint64)
     *           |               - numTimesteps (uint64)
     *           |               - min_time (double)
     *           |               - max_time (double)
     *           |_ <volume name>:<variable (int)>:<timestep (int)> - dataset for a volume.  each variable
     *                                                                of each timestep has it's own dataset.
     *                                                                Each volume dataset has the following
     *                                                                attributes:
     *                                                                - min (double) - min voxel value
     *                                                                - max (double) - max voxel value
     *                                                                - name (string) - variable name
     *                                                                - voxelType (uint64) - type of this dataset
     *  
     */
    static const hsize_t VOLUME_ATTRIBUTE_STRING_MAXLEN = 255;

    // ----------------
    // hdf5_io::hdf5_io
    // ----------------
    // Purpose:
    //   Initializes the extension list and id.
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    // 01/04/2009 -- Joe R. -- Adding maxdim arg used with the bounding
    //                         box version of readVolumeFile
    // 09/17/2011 -- Joe R. -- Maxdim is now on the property map.
    // 09/30/2011 -- Joe R. -- Checking that the maxdim property doesn't exist
    //                         before setting it.
    hdf5_io()
      : _id("hdf5_io : v1.0")
    {
      _extensions.push_back(".h5");
      _extensions.push_back(".hdf5");
      _extensions.push_back(".hdf");
      _extensions.push_back(".cvc");
      
      if(!cvcapp.hasProperty("volmagick.hdf5_io.maxdim"))
        cvcapp.properties("volmagick.hdf5_io.maxdim","128,128,128");
    }

    // -----------
    // hdf5_io::id
    // -----------
    // Purpose:
    //   Returns a string that identifies this volume_file_io object.  This should
    //   be unique, but is freeform.
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    virtual const std::string& id() const
    {
      return _id;
    }

    // -------------------
    // hdf5_io::extensions
    // -------------------
    // Purpose:
    //   Returns a list of extensions that this volume_file_io object supports.
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    virtual const extension_list& extensions() const
    {
      return _extensions;
    }

    // --------------------------
    // hdf5_io::getVolumeFileInfo
    // --------------------------
    // Purpose:
    //   Writes to a structure containing all info that VolMagick needs
    //   from a volume file.
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    // 07/24/2011 -- Joe R. -- Using hdf5_utils.
    // 07/30/2011 -- Joe R. -- Fixed some issues with the move to hdf5_utils
    // 09/02/2011 -- Joe R. -- Forgot to copy filename to data.
    // 09/09/2011 -- Joe R. -- Adding support for ungrouped, lone datasets to make
    //                         multi-res hierarchy thread code simpler.
    virtual void getVolumeFileInfo(volume_file_info::data& d,
                                   const std::string& filename) const
    {
      using namespace hdf5_utils;
      using namespace boost;

      thread_info ti(BOOST_CURRENT_FUNCTION);

      std::string actualFileName;
      std::string objectName;

      boost::tie(actualFileName, objectName) =
        splitRawFilename(filename);

      d._filename = filename;

      //check if it is an old style volmagick cvc-hdf5 file
      bool oldVolMagick = false;
      try
        {
          uint64 vm_version;
          getAttribute(actualFileName,objectName,"VolMagick_version",vm_version);
          oldVolMagick = true;
        }
      catch(hdf5_exception &)
        {
          std::string version;
          getAttribute(actualFileName,objectName,"libcvc_version",version);
          oldVolMagick = false;
        }

      uint64 numVariables, numTimesteps;
      if(!oldVolMagick)
        {
          cvcapp.log(6,str(format("%s :: using new (Aug2011) VolMagick cvc-hdf5 format\n")
                           % BOOST_CURRENT_FUNCTION));

          d._boundingBox = getObjectBoundingBox(actualFileName,objectName);
          d._dimension   = getObjectDimension(actualFileName,objectName);

          getAttribute(actualFileName,objectName,"numVariables",numVariables);
          getAttribute(actualFileName,objectName,"numTimesteps",numTimesteps);
          d._numVariables = numVariables;
          d._numTimesteps = numTimesteps;
        
          getAttribute(actualFileName,objectName,"min_time",d._tmin);
          getAttribute(actualFileName,objectName,"max_time",d._tmax);
        }
      else
        {
          cvcapp.log(5,str(format("%s :: using old VolMagick cvc-hdf5 format\n")
                           % BOOST_CURRENT_FUNCTION));

          //some older files will have attributes named like this...
          getAttribute(actualFileName,objectName,"XMin",d._boundingBox.minx);
          getAttribute(actualFileName,objectName,"YMin",d._boundingBox.miny);
          getAttribute(actualFileName,objectName,"ZMin",d._boundingBox.minz);
          getAttribute(actualFileName,objectName,"XMax",d._boundingBox.maxx);
          getAttribute(actualFileName,objectName,"YMax",d._boundingBox.maxy);
          getAttribute(actualFileName,objectName,"ZMax",d._boundingBox.maxz);

          getAttribute(actualFileName,objectName,"XDim",d._dimension.xdim);
          getAttribute(actualFileName,objectName,"YDim",d._dimension.ydim);
          getAttribute(actualFileName,objectName,"ZDim",d._dimension.zdim);

          getAttribute(actualFileName,objectName,"numVariables",numVariables);
          getAttribute(actualFileName,objectName,"numTimesteps",numTimesteps);
          d._numVariables = numVariables;
          d._numTimesteps = numTimesteps;

          getAttribute(actualFileName,objectName,"min_time",d._tmin);
          getAttribute(actualFileName,objectName,"max_time",d._tmax);
        }

      d._minIsSet.resize(numVariables);
      d._min.resize(numVariables);
      d._maxIsSet.resize(numVariables);
      d._max.resize(numVariables);
      d._voxelTypes.resize(numVariables);
      d._names.resize(numVariables);

      //The current hdf5_io implementation prefers storing 3D datasets in groups.
      //Check if the object is a group.  If not, read it as a 1 var 1 timestep lone dataset.
      if(isGroup(actualFileName,objectName))
        {
          for(unsigned int i = 0; i < d._numVariables; i++)
            {
              d._minIsSet[i].resize(numTimesteps);
              d._min[i].resize(numTimesteps);
              d._maxIsSet[i].resize(numTimesteps);
              d._max[i].resize(numTimesteps);
            
              for(unsigned int j = 0; j < d._numTimesteps; j++)
                {
                  std::string volume_name =
                    boost::str(
                               boost::format("%1%/%2%:%3%:%4%") %
                               objectName %
                               DEFAULT_VOLUME_NAME %
                               i % j
                               );

                  d._min[i][j] = getDataSetMinimum(actualFileName,volume_name);
                  d._minIsSet[i][j]=true;
                  d._max[i][j] = getDataSetMaximum(actualFileName,volume_name);
                  d._maxIsSet[i][j]=true;
                  d._names[i] = getDataSetInfo(actualFileName,volume_name);
                  uint64 voxelType;
                  if(oldVolMagick)
                    getAttribute(actualFileName,volume_name,"voxelType",voxelType);
                  else
                    getAttribute(actualFileName,volume_name,"dataType",voxelType);
                  d._voxelTypes[i]=data_type(voxelType);
                }
            }
        }
      else
        {
          if(numVariables > 1 || numTimesteps > 1)
            cvcapp.log(1,str(format("%s :: WARNING - hdf5_io doesn't yet support lone datasets "
                                    "with more than one variable or timestep!\n")
                             % BOOST_CURRENT_FUNCTION));

          std::string volume_name = objectName;
          unsigned int i = 0, j = 0;

          numVariables = 1; numTimesteps = 1;
          d._minIsSet.resize(numVariables);
          d._min.resize(numVariables);
          d._maxIsSet.resize(numVariables);
          d._max.resize(numVariables);
          d._voxelTypes.resize(numVariables);
          d._names.resize(numVariables);
          d._minIsSet[i].resize(numTimesteps);
          d._min[i].resize(numTimesteps);
          d._maxIsSet[i].resize(numTimesteps);
          d._max[i].resize(numTimesteps);

          d._min[i][j] = getDataSetMinimum(actualFileName,volume_name);
          d._minIsSet[i][j]=true;
          d._max[i][j] = getDataSetMaximum(actualFileName,volume_name);
          d._maxIsSet[i][j]=true;
          d._names[i] = getDataSetInfo(actualFileName,volume_name);
          uint64 voxelType;
          if(oldVolMagick)
            getAttribute(actualFileName,volume_name,"voxelType",voxelType);
          else
            getAttribute(actualFileName,volume_name,"dataType",voxelType);
          d._voxelTypes[i]=data_type(voxelType);
        }
    }

    // -----------------------
    // hdf5_io::readVolumeFile
    // -----------------------
    // Purpose:
    //   Writes to a volume object after reading from a volume file.
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    // 08/05/2011 -- Joe R. -- Using HDF5 Utilities now.
    // 09/09/2011 -- Joe R. -- Adding support for ungrouped, lone datasets to make
    //                         multi-res hierarchy thread code simpler.
    virtual void readVolumeFile(volume& vol,
                                const std::string& filename, 
                                unsigned int var, unsigned int time,
                                uint64 off_x, uint64 off_y, uint64 off_z,
                                const dimension& subvoldim) const
    {
      using namespace H5;
      using namespace hdf5_utils;
      using namespace boost;

      thread_info ti(BOOST_CURRENT_FUNCTION);

      std::string actualFileName;
      std::string objectName;

      if(subvoldim.isNull())
        throw hdf5_exception("Null subvoldim");

      boost::tie(actualFileName, objectName) =
        splitRawFilename(filename);

      std::string volume_name;

      if(isGroup(actualFileName,objectName))
        {
          //Name of actual dataset.  The group name in 'objectName' can contain several
          //instances of the same dataset at various resolutions.  This function assumes
          //you want the highest resolution dataset, which uses the following naming convention.
          volume_name =
            boost::str(
                       boost::format("%1%/%2%:%3%:%4%") %
                       objectName %
                       DEFAULT_VOLUME_NAME %
                       var % time
                       );
        }
      else //Ungrouped dataset
        {
          if(var > 0 || time > 0)
            cvcapp.log(1,str(format("%s :: WARNING - hdf5_io doesn't yet support lone datasets "
                                    "with more than one variable or timestep!\n")
                             % BOOST_CURRENT_FUNCTION));
          var = 0; time = 0;
          volume_name = objectName;
        }
    
      volume_file_info vfi(filename);
      bounding_box boundingBox = vfi.boundingBox();
      dimension dimension = vfi.voxel_dimensions();
    
      if(off_x + subvoldim[0] > dimension[0] ||
         off_y + subvoldim[1] > dimension[1] ||
         off_z + subvoldim[2] > dimension[2])
        throw invalid_hdf5_file("Dimension and offset out of bounds");

      //calculate the subvolume boundingbox for the requested region
      bounding_box subbbox
        ( 
         boundingBox.minx + 
         (boundingBox.maxx - boundingBox.minx)*
         (off_x/(dimension.xdim-1)),
         boundingBox.miny + 
         (boundingBox.maxy - boundingBox.miny)*
         (off_y/(dimension.ydim-1)),
         boundingBox.minz + 
         (boundingBox.maxz - boundingBox.minz)*
         (off_z/(dimension.zdim-1)),
         boundingBox.minx + 
         (boundingBox.maxx - boundingBox.minx)*
         ((off_x+subvoldim.xdim)/(dimension.xdim-1)),
         boundingBox.miny + 
         (boundingBox.maxy - boundingBox.miny)*
         ((off_y+subvoldim.ydim)/(dimension.ydim-1)),
         boundingBox.minz + 
         (boundingBox.maxz - boundingBox.minz)*
         ((off_z+subvoldim.zdim)/(dimension.zdim-1))
         );
    
      vol.voxelType(vfi.voxelTypes(var));
      vol.desc(vfi.name(var));
      vol.boundingBox(subbbox);
      vol.voxel_dimensions(subvoldim);

      //set the vol's min/max if it is to be the same size as the 
      //volume in the file.
      if(off_x == 0 && off_y == 0 && off_z == 0 &&
         vfi.voxel_dimensions() == subvoldim)
        {
          vol.min(getDataSetMinimum(actualFileName,volume_name));
          vol.max(getDataSetMaximum(actualFileName,volume_name));
        }
    
      readDataSet(actualFileName,volume_name,
                  off_x,off_y,off_z,
                  subvoldim,
                  vol.voxelType(),
                  *vol);
    }

    // -----------------------------
    // volume_file_io::readVolumeFile
    // -----------------------------
    // Purpose:
    //   Same as above except uses a bounding box for specifying the
    //   subvol.  Uses maxdim to define a stride to use when reading
    //   for subsampling.
    // ---- Change History ----
    // 01/04/2010 -- Joe R. -- Creation.
    // 08/26/2011 -- Joe R. -- Using HDF5 Utilities now.
    // 09/09/2011 -- Joe R. -- Adding support for ungrouped, lone datasets to make
    //                         multi-res hierarchy thread code simpler.
    // 09/17/2011 -- Joe R. -- Picking out the closest dimension to the maxdim in
    //                         the hierarchy.
    virtual void readVolumeFile(volume& vol, 
                                const std::string& filename, 
                                unsigned int var,
                                unsigned int time,
                                const bounding_box& subvolbox) const
    {
      using namespace hdf5_utils;
      using namespace boost;

      thread_info ti(BOOST_CURRENT_FUNCTION);

      std::string actualFileName;
      std::string objectName;

      boost::tie(actualFileName, objectName) =
        splitRawFilename(filename);

      std::string volume_name;
      if(isGroup(actualFileName,objectName))
        {
          //The group name in 'objectName' can contain several instances of the same dataset
          //at various resolutions.

          //get the maximum dimensions to extract
          std::vector<uint64> maxdim_vec = cvcapp.listProperty<uint64>("volmagick.hdf5_io.maxdim");
          while(maxdim_vec.size() < 3)
            maxdim_vec.push_back(128);
          dimension maxdim(maxdim_vec);

          // ---
          // Find the dataset with size closest to the maxdim
          // ---
        
          //filter out other variables and timesteps
          std::string filter = str(format("%1%:%2%") % var % time);

          std::vector<std::string> hierarchy_objects = 
            getChildObjects(actualFileName, objectName, filter);
          if(hierarchy_objects.empty())
            throw hdf5_exception(str(format("%s :: no child objects!")
				     % BOOST_CURRENT_FUNCTION));
        
          dimension dim = 
            getDataSetDimensionForBoundingBox(actualFileName,
                                              objectName+"/"+hierarchy_objects[0],
                                              subvolbox);
          std::string hierarchy_object = hierarchy_objects[0];

          //find the first non dirty dataset
          BOOST_FOREACH(std::string obj, hierarchy_objects)
            {
              int isDirty = 0;
              try
                {
                  getAttribute(actualFileName,
                               objectName+"/"+obj,
                               "dirty",isDirty);
                }
              catch(std::exception&){}
              if(!isDirty)
                {
                  hierarchy_object = obj;
                  break;
                }
            }

          //now select a dataset
          BOOST_FOREACH(std::string obj, hierarchy_objects)
            {
              cvcapp.log(3,str(format("%s: %s\n")
                               % BOOST_CURRENT_FUNCTION
                               % obj));

              //if this object is dirty, lets skip it for now.
              int isDirty = 0;
              try
                {
                  getAttribute(actualFileName,
                               objectName+"/"+obj,
                               "dirty",isDirty);
                }
              catch(std::exception&){}

              dimension newdim = 
                getDataSetDimensionForBoundingBox(actualFileName,
                                                  objectName+"/"+obj,
                                                  subvolbox);
              if(!isDirty &&
                 std::abs(int64(newdim.size()) - int64(maxdim.size())) <
                 std::abs(int64(dim.size()) - int64(maxdim.size())))
                {
                  dim = newdim;
                  hierarchy_object = obj;
                }
            }

          cvcapp.log(2,str(format("%s: selected object %s\n")
                           % BOOST_CURRENT_FUNCTION
                           % hierarchy_object));

          volume_name = objectName+"/"+hierarchy_object;
        }
      else //Ungrouped dataset
        {
          if(var > 0 || time > 0)
            cvcapp.log(1,str(format("%s :: WARNING - hdf5_io doesn't yet support lone datasets "
                                    "with more than one variable or timestep!\n")
                             % BOOST_CURRENT_FUNCTION));
          var = 0; time = 0;
          volume_name = objectName;
        }

      volume_file_info vfi(filename);

      vol.voxelType(vfi.voxelTypes(var));
      vol.desc(vfi.name(var));
      vol.boundingBox(subvolbox);

      //set the vol's min/max if it is to be the same size as the 
      //volume in the file.
      if(vfi.boundingBox() == subvolbox)
        {
          vol.min(getDataSetMinimum(actualFileName,volume_name));
          vol.max(getDataSetMaximum(actualFileName,volume_name));
        }

      boost::shared_array<unsigned char> data;
      dimension dim;
    
      boost::tie(data,dim) = 
        readDataSet(actualFileName, volume_name,
                    subvolbox, vol.voxelType());
    
      vol.voxel_dimensions(dim);
      std::memcpy(*vol, data.get(), dim.size() * vol.voxelSize());      
    }

    // -------------------------
    // hdf5_io::createVolumeFile
    // -------------------------
    // Purpose:
    //   Creates an empty volume file to be later filled in by writeVolumeFile
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    // 12/28/2009 -- Joe R. -- HDF5 file schema implementation
    // 08/26/2011 -- Joe R. -- Using HDF5 Utilities now.
    // 09/02/2011 -- Joe R. -- Calling new createHDF5File function.
    // 09/08/2011 -- Joe R. -- Only creating a file if none exists, else just re-using it.
    // 09/30/2011 -- Joe R. -- Added objectType attribute.
    virtual void createVolumeFile(const std::string& filename,
                                  const bounding_box& boundingBox,
                                  const dimension& dimension,
                                  const std::vector<data_type>& voxelTypes,
                                  unsigned int numVariables, unsigned int numTimesteps,
                                  double min_time, double max_time) const
    {
      using namespace hdf5_utils;
      namespace fs = boost::filesystem;
      using boost::filesystem::path;

      thread_info ti(BOOST_CURRENT_FUNCTION);

      std::string actualFileName;
      std::string objectName;

      boost::tie(actualFileName, objectName) =
        splitRawFilename(filename);

      if(voxelTypes.empty())
        throw invalid_hdf5_file("voxelTypes array not large enough!");

      //create it if it doesn't exist, else just reuse the file.
      fs::path full_path(actualFileName);
      if(!fs::exists(full_path))
        createHDF5File(actualFileName);

      createGroup(actualFileName, objectName, true);
      setAttribute(actualFileName, objectName, "objectType", "cvc::volume");
      setAttribute(actualFileName, objectName, "libcvc_version", CVC_VERSION_STRING);
      setObjectBoundingBox(actualFileName, objectName, boundingBox);
      setObjectDimension(actualFileName, objectName, dimension);
      setAttribute(actualFileName, objectName, "dataTypes", voxelTypes.size(), &(voxelTypes[0]));
      setAttribute(actualFileName, objectName, "numVariables", numVariables);
      setAttribute(actualFileName, objectName, "numTimesteps", numTimesteps);
      setAttribute(actualFileName, objectName, "min_time", min_time);
      setAttribute(actualFileName, objectName, "max_time", max_time);
    
      unsigned int steps = numVariables*numTimesteps;
      unsigned int cur_step = 0;
      for(unsigned int var = 0; var < numVariables; var++)
        for(unsigned int time = 0; time < numTimesteps; time++)
          {
            cvcapp.threadProgress(float(cur_step)/float(steps));
          
            //Name of actual dataset.  The group name in 'objectName' can contain several
            //instances of the same dataset at various resolutions.  This function just makes space
            //for the highest resolution dataset.  writeVolumeFile should trigger updates to the
            //hierarchy via a seperate thread for use by the bounding_box based readVolumeFile above.
            std::string volume_name =
              boost::str(
                         boost::format("%1%/%2%:%3%:%4%") %
                         objectName %
                         DEFAULT_VOLUME_NAME %
                         var % time
                         );

            if(voxelTypes.size() <= var)
              throw invalid_hdf5_file("voxelTypes array not large enough!");

            createDataSet(actualFileName, volume_name, 
                          boundingBox, dimension, voxelTypes[var]);
          }

      cvcapp.threadProgress(1.0);      
    }

    // ------------------------
    // hdf5_io::writeVolumeFile
    // ------------------------
    // Purpose:
    //   Writes the volume contained in wvol to the specified volume file. Should create
    //   a volume file if the filename provided doesn't exist.  Else it will simply
    //   write data to the existing file.  A common user error arises when you try to
    //   write over an existing volume file using this function for unrelated volumes.
    //   If what you desire is to overwrite an existing volume file, first run
    //   createVolumeFile to replace the volume file.
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    // 08/28/2011 -- Joe R. -- Using HDF5 Utilities now.
    // 09/09/2011 -- Joe R. -- Adding support for ungrouped, lone datasets to make
    //                         multi-res hierarchy thread code simpler.
    virtual void writeVolumeFile(const volume& wvol, 
                                 const std::string& filename,
                                 unsigned int var, unsigned int time,
                                 uint64 off_x, uint64 off_y, uint64 off_z) const
    {
      using namespace hdf5_utils;
      using namespace boost;

      thread_info ti(BOOST_CURRENT_FUNCTION);

      std::string actualFileName;
      std::string objectName;

      boost::tie(actualFileName, objectName) =
	splitRawFilename(filename);

      std::string volume_name;
      bool doBuildHierarchy = false;
      if(isGroup(actualFileName,objectName))
	{
	  //Name of actual dataset.  The group name in 'objectName' can contain several
	  //instances of the same dataset at various resolutions.  This function assumes
	  //you want the highest resolution dataset, which uses the following naming convention.
	  volume_name =
	    str(
		format("%1%/%2%:%3%:%4%") %
		objectName %
		DEFAULT_VOLUME_NAME %
		var % time
		);

	  doBuildHierarchy = true;

	  //If we have a thread running already computing the hierarchy, stop it!
	  std::string threadKey(volume_name + " hierarchy_thread");
	  if(cvcapp.hasThread(threadKey))
	    {
	      thread_ptr t = cvcapp.threads(threadKey);
	      t->interrupt(); //initiate thread quit
	      t->join(); //wait for it to quit
	    }

	  //Now mark all of the hierarchy datasets dirty, creating them if they don't exist.
	  //The build_hierarchy thread fills these in with proper data later.
	  dimension prevDim(getObjectDimension(actualFileName,objectName));
	  while(1)
	    {
	      uint64 maxdim = std::max(prevDim.xdim,std::max(prevDim.ydim,prevDim.zdim));
	      maxdim = upToPowerOfTwo(maxdim) >> 1; //power of 2 less than maxdim
	      dimension curDim(maxdim,maxdim,maxdim);
	      for(int i = 0; i < 3; i++)
		curDim[i] = std::min(curDim[i],prevDim[i]);
	      if(curDim.size()==1) break; //we're done if the dims hit zero

	      std::string hier_volume_name =
		str(format("%1%_%2%x%3%x%4%") 
		    % volume_name
		    % curDim[0] % curDim[1] % curDim[2]);

	      cvcapp.log(10,str(format("%1% :: marking %2% dirty\n")
				% BOOST_CURRENT_FUNCTION
				% hier_volume_name));

	      if(!isDataSet(actualFileName,hier_volume_name))
		createVolumeDataSet(actualFileName,hier_volume_name,
				    getObjectBoundingBox(actualFileName,volume_name),
				    curDim, wvol.voxelType());
	      setAttribute(actualFileName, hier_volume_name, "dirty", 1);

	      prevDim = curDim;
	    }
	}
      else //Ungrouped dataset
	{
	  if(var > 0 || time > 0)
	    cvcapp.log(1,str(format("%s :: WARNING - hdf5_io doesn't yet support lone datasets "
				    "with more than one variable or timestep!\n")
			     % BOOST_CURRENT_FUNCTION));
	  var = 0; time = 0;
	  volume_name = objectName;
	}

      writeDataSet(actualFileName, volume_name, 
		   off_x, off_y, off_z,
		   wvol.voxel_dimensions(),
		   wvol.voxelType(),
		   *wvol,
		   wvol.min(), wvol.max());
    
      setAttribute(actualFileName, volume_name, "info", wvol.desc());

      if(doBuildHierarchy)
	{
	  std::string threadKey("build_hierarchy_" + volume_name);
	  build_hierarchy::start(threadKey,
				 actualFileName,
				 volume_name);
	}
    }

    // -------------------------------
    // volume_file_io::writeBoundingBox
    // -------------------------------
    // Purpose:
    //   Writes the specified bounding box to the file.
    // ---- Change History ----
    // 04/06/2012 -- Joe R. -- Creation.
    virtual void writeBoundingBox(const bounding_box& bbox, const std::string& filename) const
    {
      using namespace hdf5_utils;
      using namespace boost;

      thread_info ti(BOOST_CURRENT_FUNCTION);

      std::string actualFileName;
      std::string objectName;

      boost::tie(actualFileName, objectName) =
        splitRawFilename(filename);
    
      if(isGroup(actualFileName,objectName))
        {
          std::vector<std::string> children = getChildObjects(actualFileName,
                                                              objectName);
          BOOST_FOREACH(std::string val, children)
            setObjectBoundingBox(actualFileName, objectName + "/" + val, bbox);
        }

      setObjectBoundingBox(actualFileName, objectName, bbox);
    }

    // ----------------------------
    // hdf5_io::createVolumeDataSet
    // ----------------------------
    // Purpose:
    //   Creates a volume dataset without a group.  Used for building the multi-res
    //   hierarchy.  TODO: this should probably be moved to hdf5_utils
    // ---- Change History ----
    // 09/09/2011 -- Joe R. -- Creation.
    // 09/30/2011 -- Joe R. -- Added objectType attribute.
    static void createVolumeDataSet(const std::string& hdf5_filename,
                                    const std::string& volumeDataSet,
                                    const bounding_box& boundingBox,
                                    const dimension& dimension,
                                    data_type voxelType)
    {
      using namespace hdf5_utils;

      createDataSet(hdf5_filename, volumeDataSet,
                    boundingBox, dimension, voxelType, true);
      setAttribute(hdf5_filename, volumeDataSet, 
                   "objectType", "cvc::volume");
      setAttribute(hdf5_filename, volumeDataSet, 
                   "libcvc_version", CVC_VERSION_STRING);
      setObjectBoundingBox(hdf5_filename, volumeDataSet, boundingBox);
      setObjectDimension(hdf5_filename, volumeDataSet, dimension);
      setAttribute(hdf5_filename, volumeDataSet, "dataTypes", 1, &voxelType);
      setAttribute(hdf5_filename, volumeDataSet, "numVariables", 1);
      setAttribute(hdf5_filename, volumeDataSet, "numTimesteps", 1);
      setAttribute(hdf5_filename, volumeDataSet, "min_time", 0.0);
      setAttribute(hdf5_filename, volumeDataSet, "max_time", 0.0);
    }

  protected:
    std::string _id;
    extension_list _extensions;
  };
}

namespace CVC_NAMESPACE
{
  void register_hdf5_io()
  {
    volume_file_io::insertHandler(
      volume_file_io::ptr(new hdf5_io)
    );
  }
}
