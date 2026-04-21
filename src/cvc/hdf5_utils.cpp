/*
  Copyright 2010-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define H5_SIZEOF_SSIZE_T 0

#include <cvc/hdf5_utils.h>

#if defined(WIN32)
#include <cpp/H5PredType.h>
#include <cpp/H5StrType.h>
#else
#include <H5PredType.h>
#include <H5StrType.h>
#endif

#include <boost/current_function.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem/operations.hpp>

namespace CVC_NAMESPACE
{
  namespace hdf5_utils
  {
    // --------------------
    // getPredType
    // --------------------
    // Purpose:
    //   Returns a PredType given a VoxelType.
    // ---- Change History ----
    // 12/31/2009 -- Joe R. -- Creation.
    // 06/24/2011 -- Joe R. -- Throwing an exception on error. 
    // 07/22/2011 -- Joe R. -- Adding Char.
    // 09/09/2011 -- Joe R. -- Adding Int and Int64.
    H5::PredType getPredType(data_type vt)
    {
      using namespace H5;
      switch(vt)
        {
        case UChar: return PredType::NATIVE_UCHAR;
        case UShort: return PredType::NATIVE_USHORT;
        case UInt: return PredType::NATIVE_UINT;
        case Float: return PredType::NATIVE_FLOAT;
        case Double: return PredType::NATIVE_DOUBLE;
        case UInt64: return PredType::NATIVE_UINT64;
        case Char: return PredType::C_S1;
        case Int: return PredType::NATIVE_INT;
        case Int64: return PredType::NATIVE_INT64;
        case Undefined: throw hdf5_exception("No HDF5 PredType for undefined type");
        default: throw hdf5_exception("No H5::PredType for cvc::data_type " + 
                                      boost::lexical_cast<std::string>(uint64(vt)));
        }
    }

    // ------------------
    // getH5File
    // ------------------
    // Purpose:
    //   Gets an H5File object for the provided filename, either creating or
    //   opening a file to do so.
    // ---- Change History ----
    // 12/29/2009 -- Joe R. -- Creation.
    // 08/27/2010 -- Joe R. -- Simplifying logic and forcing TRUNC if create == true
    // 01/27/2012 -- Joe R. -- Allowing RDONLY access.
    // 04/06/2012 -- Joe R. -- Catching exception thrown from isHdf5.
    boost::shared_ptr<H5::H5File> getH5File(const std::string& filename,
                                            bool create)
    {
      using namespace H5;

      boost::shared_ptr<H5File> file;

      H5::Exception::dontPrint();

      try
        {
          if(!H5File::isHdf5(filename) || create)
            file.reset(new H5File(filename, H5F_ACC_TRUNC));
          else
            {
              try
                {
                  file.reset(new H5File(filename, H5F_ACC_RDWR));
                }
              catch(H5::Exception&) //if RDWR access doesn't work, just try RDONLY
                {
                  file.reset(new H5File(filename, H5F_ACC_RDONLY));
                }
            }
        }
      catch(H5::Exception&) //sometimes isHdf5 throws if no file exists, just truncate if so
        {
          file.reset(new H5File(filename, H5F_ACC_TRUNC));          
        }

      return file;
    }

    // -----------------
    // getGroup
    // -----------------
    // Purpose:
    //   Gets a Group object for the provided file and object path, either creating or
    //   opening groups along the way to do so.
    // ---- Change History ----
    // 12/29/2009 -- Joe R. -- Creation.
    boost::shared_ptr<H5::Group> getGroup(const H5::H5File& file,
                                          const std::string& groupPath,
                                          bool create)
    {
      using namespace H5;

      typedef std::vector<std::string> split_vector_type;
      split_vector_type groupNames;
      boost::algorithm::split( groupNames, groupPath, boost::algorithm::is_any_of("/") );

      //try to open groups, else create them
      boost::shared_ptr<Group> cvc_group;
      while(!groupNames.empty())
        {
          std::string name = *groupNames.begin();
          groupNames.erase(groupNames.begin());
          if(name.empty()) continue;

          try
            {
              cvc_group.reset(new Group( cvc_group ? 
                                         cvc_group->openGroup(name) :
                                         file.openGroup(name) ));
            }
          catch(H5::Exception& e)
            {
              if(create)
                cvc_group.reset(new Group( cvc_group ? 
                                           cvc_group->createGroup(name) :
                                           file.createGroup(name) ));
              else
                throw e; //pass along the exception
            }
        }

      return cvc_group;
    }

    // -----------------
    // getDataSet
    // -----------------
    // Purpose:
    //   Gets a DataSet object for the provided file and object path, opening
    //   groups along the way.
    // ---- Change History ----
    // 06/17/2011 -- Joe R. -- Creation.
    // 09/05/2011 -- Joe R. -- Fixed a bug preventing creation of dataset at root
    boost::shared_ptr<H5::DataSet> getDataSet(const H5::H5File& file,
                                              const std::string& dataSetPath,
                                              bool create)
    {
      using namespace H5;

      typedef std::vector<std::string> split_vector_type;
      split_vector_type groupNames;
      boost::algorithm::split( groupNames, dataSetPath, boost::algorithm::is_any_of("/") );
      
      //the dataset name is the last string in the path
      std::string dataSetName;
      if(!groupNames.empty())
        dataSetName = *boost::prior(groupNames.end());
      groupNames.pop_back();

      //try to open groups, else create them
      boost::shared_ptr<Group> cvc_group;
      boost::shared_ptr<DataSet> cvc_dataset;
      while(!groupNames.empty())
        {
          std::string name = *groupNames.begin();
          groupNames.erase(groupNames.begin());
          if(name.empty()) continue;

          try
            {
              cvc_group.reset(new Group( cvc_group ? 
                                         cvc_group->openGroup(name) :
                                         file.openGroup(name) ));
            }
          catch(H5::Exception& e)
            {
              if(create)
                cvc_group.reset(new Group( cvc_group ? 
                                           cvc_group->createGroup(name) :
                                           file.createGroup(name) ));
              else
                throw e; //pass along the exception
            }
        }

      // Use appropriate object to open dataset (Group or File)
      if(cvc_group)
        cvc_dataset.reset(new DataSet(cvc_group->openDataSet(dataSetName)));
      else
        cvc_dataset.reset(new DataSet(file.openDataSet(dataSetName)));
      
      return cvc_dataset;
    }

    // -----------------
    // unlink
    // -----------------
    // Purpose:
    //  Unlinks the object at the specified path.
    // ---- Change History ----
    // 07/15/2011 -- Joe R. -- Creation.    
    void unlink(const H5::H5File& file,
                const std::string& objectPath)
    {
      using namespace H5;

      typedef std::vector<std::string> split_vector_type;
      split_vector_type groupNames;
      boost::algorithm::split( groupNames, objectPath, boost::algorithm::is_any_of("/") );

      //the unlink target name is the last string in the path
      std::string unlinkName;
      if(!groupNames.empty())
        unlinkName = *boost::prior(groupNames.end());
      groupNames.pop_back();
      
      //try to open groups along the path 
      boost::shared_ptr<Group> cvc_group;
      while(!groupNames.empty())
        {
          std::string name = *groupNames.begin();
          groupNames.erase(groupNames.begin());
          if(name.empty()) continue;

          cvc_group.reset(new Group( cvc_group ? 
                                     cvc_group->openGroup(name) :
                                     file.openGroup(name) ));
        }

      //if we found a group, unlink from the group.  Else it must be at the root,
      //so unlink from the file itself.
      if(cvc_group) cvc_group->unlink(unlinkName);
      else const_cast<H5::H5File&>(file).unlink(unlinkName);
    }

    // ---------------------
    // hasAttribute
    // ---------------------
    // Purpose:
    //   Returns true if object has named attribute
    // ---- Change History ----
    // 12/29/2009 -- Joe R. -- Creation.    
    bool hasAttribute(const H5::H5Object& obj,
                      const std::string& name)
    {
      using namespace H5;

      try
        {
          Attribute attr = obj.openAttribute(name);
        }
      catch(AttributeIException&)
        {
          return false;
        }
      return true;
    }

    // ---------------------
    // isGroup
    // ---------------------
    // Purpose:
    //   testing for group in HDF5 file
    // ---- Change History ----
    // 06/24/2011 -- Joe R. -- Creation.
    // 07/15/2011 -- Joe R. -- Using a mutex to protect file access
    bool isGroup(app& ctx,
                             const std::string& hdf5_filename,
                 const std::string& hdf5_objname)
    {
      scoped_lock lock(ctx,hdf5_filename,BOOST_CURRENT_FUNCTION);
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
			       % BOOST_CURRENT_FUNCTION
			       % hdf5_filename
			       % hdf5_objname));

      try
        {
          /*
           * Turn off the auto-printing when failure occurs so that we can
           * handle the errors appropriately
           */
          H5::Exception::dontPrint();

          boost::shared_ptr<H5::H5File> f = getH5File(hdf5_filename);
          boost::shared_ptr<H5::Group>  g = getGroup(*f,hdf5_objname,false);
        }
      catch(H5::Exception& e)
        {
          return false;
        }
      return true;
    }

    // ---------------------
    // isDataSet
    // ---------------------
    // Purpose:
    //   testing for dataset in HDF5 file
    // ---- Change History ----
    // 06/24/2011 -- Joe R. -- Creation.
    // 07/15/2011 -- Joe R. -- Using a mutex to protect file access
    bool isDataSet(app& ctx,
                             const std::string& hdf5_filename,
                   const std::string& hdf5_objname)
    {
      scoped_lock lock(ctx,hdf5_filename,BOOST_CURRENT_FUNCTION);
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
			       % BOOST_CURRENT_FUNCTION
			       % hdf5_filename
			       % hdf5_objname));
      
      try
        {
          /*
           * Turn off the auto-printing when failure occurs so that we can
           * handle the errors appropriately
           */
          H5::Exception::dontPrint();

          boost::shared_ptr<H5::H5File>  f = getH5File(hdf5_filename);
          boost::shared_ptr<H5::DataSet> d = getDataSet(*f,hdf5_objname,false);
        }
      catch(H5::Exception& e)
        {
          return false;
        }
      return true;
    }

    // ---------------------
    // objectExists
    // ---------------------
    // Purpose:
    //   Returns true of the specified object is a
    //   dataset or group in the specified hdf5 file.
    // ---- Change History ----
    // 07/15/2011 -- Joe R. -- Creation.
    bool objectExists(app& ctx,
                             const std::string& hdf5_filename,
                      const std::string& hdf5_objname)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
			       % BOOST_CURRENT_FUNCTION
			       % hdf5_filename
			       % hdf5_objname));
      return 
        isGroup(ctx, hdf5_filename,hdf5_objname) ||
        isDataSet(ctx, hdf5_filename,hdf5_objname);
    }

    // ---------------------
    // removeObject
    // ---------------------
    // Purpose:
    //   Removes the specified object from the HDF5
    //   file.
    // ---- Change History ----
    // 07/15/2011 -- Joe R. -- Creation.
    // 09/02/2011 -- Joe R. -- Catching H5::Exception
    void removeObject(app& ctx,
                             const std::string& hdf5_filename,
                      const std::string& hdf5_objname)
    {
      using namespace boost;

      scoped_lock lock(ctx,hdf5_filename,BOOST_CURRENT_FUNCTION);
      ctx.log(10,str(format("%1%: %2%, %3%\n") 
			% BOOST_CURRENT_FUNCTION
			% hdf5_filename
			% hdf5_objname));

      try
        {
          shared_ptr<H5::H5File>  f = getH5File(hdf5_filename);
          unlink(*f,hdf5_objname);
        }
      catch(H5::Exception& error)
        {
          throw hdf5_exception(str(format("filename: %s, object: %s, msg: %s")
                                   % hdf5_filename
                                   % hdf5_objname
                                   % error.getDetailMsg()));
        }
    }

    // --------------
    // createHDF5File
    // --------------
    // Purpose:
    //   Creates a new HDF5 File.
    // ---- Change History ----
    // 09/02/2011 -- Joe R. -- Creation.
    void createHDF5File(app& ctx,
                             const std::string& hdf5_filename)
    {
      using namespace boost;

      scoped_lock lock(ctx,hdf5_filename,BOOST_CURRENT_FUNCTION);
      ctx.log(10,str(format("%1%: %2%\n") 
			% BOOST_CURRENT_FUNCTION
			% hdf5_filename));

      try
        {
          shared_ptr<H5::H5File>  f = getH5File(hdf5_filename,true);
        }
      catch(H5::Exception& error)
        {
          throw hdf5_exception(str(format("filename: %s, msg: %s")
                                   % hdf5_filename
                                   % error.getDetailMsg()));
        }
    }

    // ---------------------
    // createGroup
    // ---------------------
    // Purpose:
    //   Creates a group, overwriting anything at the
    //   specified object path if necessary.
    // ---- Change History ----
    // 07/15/2011 -- Joe R. -- Creation.
    // 08/28/2011 -- Joe R. -- Throwing an exception instead of using
    //                         boolean return values.
    // 09/02/2011 -- Joe R. -- Fixing bug where file was being truncated each call
    void createGroup(app& ctx,
                             const std::string& hdf5_filename,
                     const std::string& hdf5_objname,
                     bool replace)
    {
      using namespace boost;
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      if(objectExists(ctx, hdf5_filename,hdf5_objname))
        if(replace)
          removeObject(ctx, hdf5_filename,hdf5_objname);
        else
          throw hdf5_exception(str(format("filename: %s, object: %s, msg: %s")
                                   % hdf5_filename
                                   % hdf5_objname
                                   % "object exists!"));

      {
        scoped_lock lock(ctx,hdf5_filename,BOOST_CURRENT_FUNCTION);
        try
          {
            boost::shared_ptr<H5::H5File> f = getH5File(hdf5_filename);
            boost::shared_ptr<H5::Group>  g = getGroup(*f,hdf5_objname,true);
          }
        catch(H5::Exception& error)
          {
            throw hdf5_exception(str(format("filename: %s, object: %s, msg: %s")
                                     % hdf5_filename
                                     % hdf5_objname
                                     % error.getDetailMsg()));
          }
      }
    }

    // ---------------------
    // createDataSet
    // ---------------------
    // Purpose:
    //   Creates a dataset, overwriting anything at the
    //   specified object path if necessary.
    // ---- Change History ----
    // 07/15/2011 -- Joe R. -- Creation, adapted from VolMagick/HDF5_IO.cpp
    // 08/26/2011 -- Joe R. -- Adding more detailed exception string
    // 08/28/2011 -- Joe R. -- Throwing exception instead of using boolean ret val
    // 09/02/2011 -- Joe R. -- Adding replace arg to allow for inline dataset obj replacement
    void createDataSet(app& ctx,
                             const std::string& hdf5_filename,
                       const std::string& hdf5_objname,
                       const bounding_box& boundingBox,
                       const dimension& dim,
                       data_type dataType,
                       const bool replace,
                       const bool createGroups)
    {
      using namespace H5;
      using namespace boost;

      ctx.log(10,str(format("%1%: %2%, %3%\n") 
                        % BOOST_CURRENT_FUNCTION
                        % hdf5_filename
                        % hdf5_objname));

      if(objectExists(ctx, hdf5_filename,hdf5_objname))
        if(replace)
          removeObject(ctx, hdf5_filename,hdf5_objname);
        else
          throw hdf5_exception(str(format("filename: %s, object: %s, msg: %s")
                                   % hdf5_filename
                                   % hdf5_objname
                                   % "object exists!"));

      {
        scoped_lock lock(ctx,hdf5_filename,BOOST_CURRENT_FUNCTION);
        const dimension maxdim(256,256,256);

        typedef std::vector<std::string> split_vector_type;
        split_vector_type groupNames;
        algorithm::split( groupNames, hdf5_objname, algorithm::is_any_of("/") );
      
        //the dataset name is the last string in the path
        std::string dataSetName;
        if(!groupNames.empty())
          dataSetName = *prior(groupNames.end());
        groupNames.pop_back();

        try
          {
            /*
             * Turn off the auto-printing when failure occurs so that we can
             * handle the errors appropriately
             */
            H5::Exception::dontPrint();
      
            shared_ptr<H5File> file = getH5File(hdf5_filename);

            std::string groupPath = algorithm::join(groupNames,"/");
            ctx.log(10,str(format("%1%: group: %2%\n")
			      % BOOST_CURRENT_FUNCTION
			      % groupPath));
            shared_ptr<Group> cvc_group = 
              getGroup(*file,groupPath,createGroups);        

            /*
             * Define the size of the array and create the data space for fixed
             * size dataset.
             */
            const int RANK = 3;
            hsize_t dimsf[RANK];     // dataset dimensions
            for(int i = 0; i < RANK; i++)
              dimsf[i] = dim[RANK-1-i];
            DataSpace dataspace( RANK, dimsf );

            //don't do chunking if input dimension is less than maxdim
            bool do_chunking = !(dim.size() < maxdim.size());

            //if we found a group, add to the group.  Else it must be at the root,
            //so add via the file itself.
            try
              {
                if(cvc_group)
                  cvc_group->unlink(dataSetName);
                else
                  file->unlink(dataSetName);
              }
            catch(H5::Exception& e)
              {}
                
            hsize_t chunk_dim[RANK] = 
              { 
                std::min(maxdim[2],dim[2]), 
                std::min(maxdim[1],dim[1]), 
                std::min(maxdim[0],dim[0]) 
              };

            shared_ptr<H5::DataSet> dataset;
                
            switch(dataType)
              {
              case UChar:
                {
                  /*
                   * Create property list for a dataset and set up fill values.
                   */
                  unsigned char fillvalue = 0;   /* Fill value for the dataset */
                  DSetCreatPropList plist;
                  plist.setFillValue(PredType::NATIVE_UCHAR, &fillvalue);
                
                  //set the chunk size to be what maxdim is set to.
                  if(do_chunking)
                    {
                      plist.setChunk(RANK, chunk_dim);
                    }
                
                  if(cvc_group)
                    dataset.reset(new DataSet(cvc_group->createDataSet(dataSetName, PredType::NATIVE_UCHAR, dataspace, plist)));
                  else
                    dataset.reset(new DataSet(file->createDataSet(dataSetName, PredType::NATIVE_UCHAR, dataspace, plist)));
                }
                break;
              case UShort:
                {
                  /*
                   * Create property list for a dataset and set up fill values.
                   */
                  unsigned short fillvalue = 0;   /* Fill value for the dataset */
                  DSetCreatPropList plist;
                  plist.setFillValue(PredType::NATIVE_USHORT, &fillvalue);
                
                  //set the chunk size to be what maxdim is set to.
                  if(do_chunking)
                    {
                      plist.setChunk(RANK, chunk_dim);
                    }
                
                  if(cvc_group)
                    dataset.reset(new DataSet(cvc_group->createDataSet(dataSetName, PredType::NATIVE_USHORT, dataspace, plist)));
                  else
                    dataset.reset(new DataSet(file->createDataSet(dataSetName, PredType::NATIVE_USHORT, dataspace, plist)));
                }
                break;
              case UInt:
                {
                  /*
                   * Create property list for a dataset and set up fill values.
                   */
                  unsigned int fillvalue = 0;   /* Fill value for the dataset */
                  DSetCreatPropList plist;
                  plist.setFillValue(PredType::NATIVE_UINT, &fillvalue);
                
                  //set the chunk size to be what maxdim is set to.
                  if(do_chunking)
                    {
                      plist.setChunk(RANK, chunk_dim);
                    }
                
                  if(cvc_group)
                    dataset.reset(new DataSet(cvc_group->createDataSet(dataSetName, PredType::NATIVE_UINT, dataspace, plist)));
                  else
                    dataset.reset(new DataSet(file->createDataSet(dataSetName, PredType::NATIVE_UINT, dataspace, plist)));                  
                }
                break;
              case Float:
                {
                  /*
                   * Create property list for a dataset and set up fill values.
                   */
                  float fillvalue = 0.0;   /* Fill value for the dataset */
                  DSetCreatPropList plist;
                  plist.setFillValue(PredType::NATIVE_FLOAT, &fillvalue);
                
                  //set the chunk size to be what maxdim is set to.
                  if(do_chunking)
                    {
                      plist.setChunk(RANK, chunk_dim);
                    }
            
                  if(cvc_group)
                    dataset.reset(new DataSet(cvc_group->createDataSet(dataSetName, PredType::NATIVE_FLOAT, dataspace, plist)));
                  else
                    dataset.reset(new DataSet(file->createDataSet(dataSetName, PredType::NATIVE_FLOAT, dataspace, plist)));                  
                }
                break;
              case Double:
                {
                  /*
                   * Create property list for a dataset and set up fill values.
                   */
                  double fillvalue = 0.0;   /* Fill value for the dataset */
                  DSetCreatPropList plist;
                  plist.setFillValue(PredType::NATIVE_DOUBLE, &fillvalue);
                
                  //set the chunk size to be what maxdim is set to.
                  if(do_chunking)
                    {
                      plist.setChunk(RANK, chunk_dim);
                    }
                
                  if(cvc_group)
                    dataset.reset(new DataSet(cvc_group->createDataSet(dataSetName, PredType::NATIVE_DOUBLE, dataspace, plist)));
                  else
                    dataset.reset(new DataSet(file->createDataSet(dataSetName, PredType::NATIVE_DOUBLE, dataspace, plist)));                  
                }
                break;
              case UInt64:
                {
                  /*
                   * Create property list for a dataset and set up fill values.
                   */
                  uint64 fillvalue = 0;   /* Fill value for the dataset */
                  DSetCreatPropList plist;
                  plist.setFillValue(PredType::NATIVE_UINT64, &fillvalue);
                
                  //set the chunk size to be what maxdim is set to.
                  if(do_chunking)
                    {
                      plist.setChunk(RANK, chunk_dim);
                    }
            
                  if(cvc_group)
                    dataset.reset(new DataSet(cvc_group->createDataSet(dataSetName, PredType::NATIVE_UINT64, dataspace, plist)));
                  else
                    dataset.reset(new DataSet(file->createDataSet(dataSetName, PredType::NATIVE_UINT64, dataspace, plist)));                  
                }
                break;
              case Char:
                {
                  /*
                   * Create property list for a dataset and set up fill values.
                   */
                  char fillvalue = 0;   /* Fill value for the dataset */
                  DSetCreatPropList plist;
                  plist.setFillValue(PredType::C_S1, &fillvalue);
                
                  //set the chunk size to be what maxdim is set to.
                  if(do_chunking)
                    {
                      plist.setChunk(RANK, chunk_dim);
                    }
            
                  if(cvc_group)
                    dataset.reset(new DataSet(cvc_group->createDataSet(dataSetName, PredType::C_S1, dataspace, plist)));
                  else
                    dataset.reset(new DataSet(file->createDataSet(dataSetName, PredType::C_S1, dataspace, plist)));                  
                }
                break;
              default:
                throw write_error(
                  str(
                    format("%1% : invalid type :: %2%") 
		    % BOOST_CURRENT_FUNCTION 
		    % dataType
                  )
                );
                break;
              }
          }
        catch( H5::Exception& error )
          {
            using namespace boost;
            throw hdf5_exception(str(format("filename: %s, object: %s, msg: %s")
                                     % hdf5_filename
                                     % hdf5_objname
                                     % error.getDetailMsg()));
          }
      }

      //set up some standard dataset attributes
      setAttribute(ctx, hdf5_filename, hdf5_objname, "libcvc_version", CVC_VERSION_STRING);
      setObjectBoundingBox(ctx, hdf5_filename, hdf5_objname, boundingBox);
      setObjectDimension(ctx, hdf5_filename, hdf5_objname, dim);
      setAttribute(ctx, hdf5_filename, hdf5_objname, "min", std::numeric_limits<double>::max());
      setAttribute(ctx, hdf5_filename, hdf5_objname, "max", -std::numeric_limits<double>::max());
      setAttribute(ctx, hdf5_filename, hdf5_objname, "info", "No Name");
      setAttribute(ctx, hdf5_filename, hdf5_objname, "dataType", dataType);
      setAttribute(ctx, hdf5_filename, hdf5_objname, "dataTypes", 1, &dataType);
      setAttribute(ctx, hdf5_filename, hdf5_objname, "numVariables", 1);
      setAttribute(ctx, hdf5_filename, hdf5_objname, "numTimesteps", 1);
      setAttribute(ctx, hdf5_filename, hdf5_objname, "min_time", 0.0);
      setAttribute(ctx, hdf5_filename, hdf5_objname, "max_time", 0.0);
    }

    // ---------------------
    // createDataSet
    // ---------------------
    // Purpose:
    //   Creates a string dataset and writes the specified
    //   string to it.
    // ---- Change History ----
    // 07/22/2011 -- Joe R. -- Creation.
    // 08/28/2011 -- Joe R. -- Throwing exception instead of using boolean ret val
    void createDataSet(app& ctx,
                             const std::string& hdf5_filename,
                       const std::string& hdf5_objname,
                       const std::string& value,
                       bool createGroups)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%, %4%\n")
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname
                               % value));
      createDataSet(ctx, hdf5_filename,
                    hdf5_objname,
                    dimension(value.size(),1,1),
                    Char,
                    createGroups);
      writeDataSet(ctx, hdf5_filename,
                   hdf5_objname,
                   value);
    }

    // ---------------------
    // readDataSet
    // ---------------------
    // Purpose:
    //   Read string dataset into string
    // ---- Change History ----
    // 07/22/2011 -- Joe R. -- Creation.
    void readDataSet(app& ctx,
                             const std::string& hdf5_filename,
                     const std::string& hdf5_objname,
                     std::string& value)
    {
      ctx.log(10,boost::str(boost::format("%1%: called\n") % BOOST_CURRENT_FUNCTION));

      dimension dim = getObjectDimension(ctx, hdf5_filename,
                                         hdf5_objname);
      value.resize(dim.size());
      readDataSet(ctx, hdf5_filename,
                  hdf5_objname,
                  0,0,0,
                  dim,
                  &(value[0]));
    }

    // ---------------------
    // writeDataSet
    // ---------------------
    // Purpose:
    //   write string dataset
    // ---- Change History ----
    // 07/22/2011 -- Joe R. -- Creation.
    void writeDataSet(app& ctx,
                             const std::string& hdf5_filename,
                      const std::string& hdf5_objname,
                      const std::string& value)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      //Not certain why I need to specify the template parameter
      //here and not for the above function.  Something to do with
      //const?
      writeDataSet<char>(ctx, hdf5_filename,
                         hdf5_objname,
                         0,0,0,
                         dimension(value.size(),1,1),
                         &(value[0]),
                         -128,
                         127);
    }

    // ---------------------
    // getObjectDimension
    // ---------------------
    // Purpose:
    //   Returns the dimension attributes of the HDF5 object
    // ---- Change History ----
    // 07/17/2011 -- Joe R. -- Creation.
    // 08/05/2011 -- Joe R. -- Renamed and generalized for both datasets and groups.
    // 08/26/2011 -- Joe R. -- Adding more detailed exception string
    dimension getObjectDimension(app& ctx,
                             const std::string& hdf5_filename,
                                 const std::string& hdf5_objname)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      if(!objectExists(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such object " + hdf5_objname);

      dimension dim;
      getAttribute(ctx, hdf5_filename,hdf5_objname,"xdim",dim.xdim);
      getAttribute(ctx, hdf5_filename,hdf5_objname,"ydim",dim.ydim);
      getAttribute(ctx, hdf5_filename,hdf5_objname,"zdim",dim.zdim);

      return dim;
    }

    // ---------------------
    // setObjectDimension
    // ---------------------
    // Purpose:
    //   Sets the dimensions of the specified object
    // ---- Change History ----
    // 08/26/2011 -- Joe R. -- Creation.
    void setObjectDimension(app& ctx,
                             const std::string& hdf5_filename,
                            const std::string& hdf5_objname,
                            const dimension& dim)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      if(!objectExists(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such object " + hdf5_objname);

      setAttribute(ctx, hdf5_filename,hdf5_objname,"xdim",dim.xdim);
      setAttribute(ctx, hdf5_filename,hdf5_objname,"ydim",dim.ydim);
      setAttribute(ctx, hdf5_filename,hdf5_objname,"zdim",dim.zdim);
    }

    // ---------------------------------
    // getDataSetDimensionForBoundingBox
    // ---------------------------------
    // Purpose:
    //   Returns the dimensions of a sub-dataset defined by the
    //   bounding box.
    // ---- Change History ----
    // 09/04/2011 -- Joe R. -- Creation.
    dimension getDataSetDimensionForBoundingBox(app& ctx,
                             const std::string& hdf5_filename,
                                                const std::string& hdf5_objname,
                                                const bounding_box& subvolbox)
    {
      using namespace boost;

      ctx.log(10,str(format("%1%: %2%, %3%\n") 
                        % BOOST_CURRENT_FUNCTION
                        % hdf5_filename
                        % hdf5_objname));

      if(!isDataSet(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such dataset " + hdf5_objname);

      bounding_box boundingBox = getObjectBoundingBox(ctx, hdf5_filename,
                                                      hdf5_objname);
      dimension dim = getObjectDimension(ctx, hdf5_filename,
                                               hdf5_objname);
      double xspan = dim.xdim == 0 ? 1.0 : (boundingBox.maxx-boundingBox.minx)/(dim.xdim-1);
      double yspan = dim.ydim == 0 ? 1.0 : (boundingBox.maxy-boundingBox.miny)/(dim.ydim-1);
      double zspan = dim.zdim == 0 ? 1.0 : (boundingBox.maxz-boundingBox.minz)/(dim.zdim-1);

      dimension fulldim(
        1+(subvolbox.maxx-subvolbox.minx)/xspan,
        1+(subvolbox.maxy-subvolbox.miny)/yspan,
        1+(subvolbox.maxz-subvolbox.minz)/zspan
      );

      return fulldim;
    }

    // ---------------------
    // getDataSetDimension
    // ---------------------
    // Purpose:
    //   Returns the dimensions of a sub-dataset/group defined by the
    //   bounding box.
    // ---- Change History ----
    // 07/22/2011 -- Joe R. -- Creation.
    // 08/26/2011 -- Joe R. -- Adding more detailed exception string
    dimension getDataSetDimension(app& ctx,
                             const std::string& hdf5_filename,
                                  const std::string& hdf5_objname,
                                  const bounding_box& subvolbox,
                                  const dimension& maxdim)
    {
      using namespace boost;

      ctx.log(10,str(format("%1%: %2%, %3%\n") 
                        % BOOST_CURRENT_FUNCTION
                        % hdf5_filename
                        % hdf5_objname));

      if(!isDataSet(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such dataset " + hdf5_objname);

      bounding_box boundingBox = getObjectBoundingBox(ctx, hdf5_filename,
                                                      hdf5_objname);
      dimension dim = getObjectDimension(ctx, hdf5_filename,
                                         hdf5_objname);
      double xspan = dim.xdim == 0 ? 1.0 : (boundingBox.maxx-boundingBox.minx)/(dim.xdim-1);
      double yspan = dim.ydim == 0 ? 1.0 : (boundingBox.maxy-boundingBox.miny)/(dim.ydim-1);
      double zspan = dim.zdim == 0 ? 1.0 : (boundingBox.maxz-boundingBox.minz)/(dim.zdim-1);

      dimension fulldim(
        1+(subvolbox.maxx-subvolbox.minx)/xspan,
        1+(subvolbox.maxy-subvolbox.miny)/yspan,
        1+(subvolbox.maxz-subvolbox.minz)/zspan
      );

      if(fulldim.isNull())
        {
          throw hdf5_exception(str(format("filename: %s, object: %s, msg: %s")
                                  % hdf5_filename
                                  % hdf5_objname
                                  % "Null voxel selection"));
        }

      hsize_t off_x =
        (subvolbox.minx-boundingBox.minx)/xspan;
      hsize_t off_y =
        (subvolbox.miny-boundingBox.miny)/yspan;
      hsize_t off_z =
        (subvolbox.minz-boundingBox.minz)/zspan;

      const int RANK = 3;
      hsize_t offset[RANK] =
        { off_z, off_y, off_x };

      for(int i = 0; i < RANK; i++)
        ctx.log(10,str(format("offset[%1%]: %2%\n") % i % offset[i]));

      hsize_t stride[RANK];
      for(int i = 0; i < RANK; i++)
        {
          stride[i] = fulldim[RANK-1-i]/maxdim[RANK-1-i];
          if(stride[i] == 0) stride[i]=1;
          if(fulldim[RANK-1-i] / stride[i] > maxdim[RANK-1-i])
            stride[i]++;
        }

      for(int i = 0; i < RANK; i++)
        ctx.log(10,str(format("stride[%1%]: %2%\n") % i % stride[i]));
      
      hsize_t count[RANK];
      for(int i = 0; i < RANK; i++)
        count[i] = fulldim[RANK-1-i]/stride[i];

      for(int i = 0; i < RANK; i++)
        ctx.log(10,str(format("count[%1%]: %2%\n") % i % count[i]));
      
      return dimension(count[2],count[1],count[0]);
    }

    // ---------------------
    // getObjectBoundingBox
    // ---------------------
    // Purpose:
    //   Returns the bounding box of the dataset
    // ---- Change History ----
    // 07/17/2011 -- Joe R. -- Creation.
    // 08/05/2011 -- Joe R. -- Renamed and generalized for both datasets and groups.
    // 08/26/2011 -- Joe R. -- Adding more detailed exception string
    bounding_box getObjectBoundingBox(app& ctx,
                             const std::string& hdf5_filename,
                                     const std::string& hdf5_objname)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      if(!objectExists(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such object " + hdf5_objname);

      bounding_box boundingBox;
      getAttribute(ctx, hdf5_filename,hdf5_objname,"xmin",boundingBox.minx);
      getAttribute(ctx, hdf5_filename,hdf5_objname,"ymin",boundingBox.miny);
      getAttribute(ctx, hdf5_filename,hdf5_objname,"zmin",boundingBox.minz);
      getAttribute(ctx, hdf5_filename,hdf5_objname,"xmax",boundingBox.maxx);
      getAttribute(ctx, hdf5_filename,hdf5_objname,"ymax",boundingBox.maxy);
      getAttribute(ctx, hdf5_filename,hdf5_objname,"zmax",boundingBox.maxz);

      return boundingBox;
    }

    // ---------------------
    // setObjectBoundingBox
    // ---------------------
    // Purpose:
    //   Sets the bounding box of the specified object
    // ---- Change History ----
    // 08/26/2011 -- Joe R. -- Creation.
    void setObjectBoundingBox(app& ctx,
                             const std::string& hdf5_filename,
                              const std::string& hdf5_objname,
                              const bounding_box& boundingBox)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      if(!objectExists(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such object " + hdf5_objname);

      setAttribute(ctx, hdf5_filename,hdf5_objname,"xmin",boundingBox.minx);
      setAttribute(ctx, hdf5_filename,hdf5_objname,"ymin",boundingBox.miny);
      setAttribute(ctx, hdf5_filename,hdf5_objname,"zmin",boundingBox.minz);
      setAttribute(ctx, hdf5_filename,hdf5_objname,"xmax",boundingBox.maxx);
      setAttribute(ctx, hdf5_filename,hdf5_objname,"ymax",boundingBox.maxy);
      setAttribute(ctx, hdf5_filename,hdf5_objname,"zmax",boundingBox.maxz);
    }

    // ---------------------
    // getDataSetInfo
    // ---------------------
    // Purpose:
    //   Returns the dataset type
    // ---- Change History ----
    // 07/17/2011 -- Joe R. -- Creation.
    // 08/26/2011 -- Joe R. -- Adding more detailed exception string
    std::string getDataSetInfo(app& ctx,
                             const std::string& hdf5_filename,
                               const std::string& hdf5_objname)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      if(!isDataSet(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such dataset " + hdf5_objname);

      std::string info;
      getAttribute(ctx, hdf5_filename,hdf5_objname,"info",info);
      
      return info;
    }

    // ---------------------
    // getDataSetType
    // ---------------------
    // Purpose:
    //   Returns the type of the dataset
    // ---- Change History ----
    // 07/17/2011 -- Joe R. -- Creation.
    // 08/26/2011 -- Joe R. -- Adding more detailed exception string
    data_type getDataSetType(app& ctx,
                             const std::string& hdf5_filename,
                             const std::string& hdf5_objname)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      if(!isDataSet(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such dataset " + hdf5_objname);

      uint64 dataTypeInt;
      getAttribute(ctx, hdf5_filename,hdf5_objname,"dataType",dataTypeInt);
      data_type dataType = data_type(dataTypeInt);
      
      return dataType;
    }

    // ---------------
    // getChildObjects
    // ---------------
    // Purpose:
    //   Gets a list of child objects of the specified object.
    // ---- Change History ----
    // 09/02/2011 -- Joe R. -- Creation.
    // 09/17/2011 -- Joe R. -- Adding filter parameter.  A string isn't added
    //                         to the list if the filter IS NOT in the string
    std::vector<std::string> getChildObjects(app& ctx,
                             const std::string& hdf5_filename,
                                             const std::string& hdf5_objname,
                                             const std::string& filter)
    {
      using namespace H5;
      using namespace boost;

      std::vector<std::string> objnames;
      ctx.log(10,str(format("%1%: %2%, %3%\n") 
                        % BOOST_CURRENT_FUNCTION
                        % hdf5_filename
                        % hdf5_objname));

      {
        scoped_lock lock(ctx,hdf5_filename,BOOST_CURRENT_FUNCTION);

        try
          {
            /*
             * Turn off the auto-printing when failure occurs so that we can
             * handle the errors appropriately
             */
            H5::Exception::dontPrint();
      
            shared_ptr<H5File> file = getH5File(hdf5_filename);
            shared_ptr<Group> cvc_group = getGroup(*file, hdf5_objname, false);        
            size_t numObjs = cvc_group->getNumObjs();
            for(size_t i = 0; i < numObjs; i++)
              {
                std::string objname(cvc_group->getObjnameByIdx(i));
                if(!filter.empty())
                  {
                    //if the filter string is contained, add it to the list
                    if(objname.find(filter) != std::string::npos)
                      objnames.push_back(objname);
                  }
                else
                  objnames.push_back(objname);
              }
          }
        catch( H5::Exception& error )
          {
            throw hdf5_exception(str(format("filename: %s, object: %s, msg: %s")
                                     % hdf5_filename
                                     % hdf5_objname
                                     % error.getDetailMsg()));
          }        
      }

      return objnames;
    }

    // ---------------------
    // getDataSetMinimum
    // ---------------------
    // Purpose:
    //   Returns the minimum value of a dataset
    // ---- Change History ----
    // 07/17/2011 -- Joe R. -- Creation.
    // 08/26/2011 -- Joe R. -- Adding more detailed exception string
    double getDataSetMinimum(app& ctx,
                             const std::string& hdf5_filename,
                             const std::string& hdf5_objname)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      if(!isDataSet(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such dataset " + hdf5_objname);

      double min_val;
      getAttribute(ctx, hdf5_filename,hdf5_objname,"min",min_val);
      return min_val;
    }

    // ---------------------
    // getDataSetMaximum
    // ---------------------
    // Purpose:
    //   Returns the maximum value of a dataset
    // ---- Change History ----
    // 07/17/2011 -- Joe R. -- Creation.
    // 08/26/2011 -- Joe R. -- Adding more detailed exception string
    double getDataSetMaximum(app& ctx,
                             const std::string& hdf5_filename,
                             const std::string& hdf5_objname)
    {
      ctx.log(10,boost::str(boost::format("%1%: %2%, %3%\n") 
                               % BOOST_CURRENT_FUNCTION
                               % hdf5_filename
                               % hdf5_objname));

      if(!isDataSet(ctx, hdf5_filename,hdf5_objname))
        throw hdf5_exception("filename: " + hdf5_filename + ", No such dataset " + hdf5_objname);

      double max_val;
      getAttribute(ctx, hdf5_filename,hdf5_objname,"max",max_val);
      return max_val;
    }
  }
}
