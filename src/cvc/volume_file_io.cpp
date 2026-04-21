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

#include <cvc/volume_file_io.h>

#include <cvc/utility.h>
#include <cvc/app.h>
#include <cvc/exception.h>

#include <boost/foreach.hpp>

namespace CVC_NAMESPACE
{
  // 09/05/2011 -- Joe R. -- moved the following 2 const vars from HDF5_IO
  //The group where volumes are stored in files that support hierarchical data structures
  const char* volume_file_io::CVC_VOLUME_GROUP = "/cvc/volumes";
  
  //The name to use when no volume name is specified
  const char* volume_file_io::DEFAULT_VOLUME_NAME = "volume";

  //A regex to extract a filename extension
  const char* volume_file_io::FILE_EXTENSION_EXPR = "^(.*)(\\.\\S*)$";

  // -------------------------------
  // volume_file_io::splitRawFilename
  // -------------------------------
  // Purpose:
  //   Splits a filename into an actual file name and an object
  //   path.
  // ---- Change History ----
  // 07/24/2011 -- Joe R. -- Creation.
  // 08/05/2011 -- Joe R. -- Using a '|' instead of ':'
  // 09/05/2011 -- Joe R. -- Moved here from HDF5_IO
  boost::tuple<
    std::string, /* actual file name */
    std::string  /* hdf5 object name */
  > volume_file_io::splitRawFilename(const std::string& filename)
  {
    typedef std::vector<std::string> split_vector_type;
    split_vector_type names;
    boost::algorithm::split( names, filename, boost::algorithm::is_any_of("|") );
    names.resize(2);

    //Use default name if no object name was specified
    if(names[1].empty())
      names[1] = 
        std::string(CVC_VOLUME_GROUP) + "/" + 
        std::string(DEFAULT_VOLUME_NAME);

    return boost::make_tuple(names[0],names[1]);
  }

  // -----------------------------
  // volume_file_io::readVolumeFile
  // -----------------------------
  // Purpose:
  //   Same as above except uses a bounding box for specifying the
  //   subvol.  A default implementation is provided. 
  // ---- Change History ----
  // 01/03/2010 -- Joe R. -- Creation.
  // 05/11/2010 -- Joe R. -- Fixing off-by-one indexing problem.
  void volume_file_io::readVolumeFile(app& ctx,
                                     volume& vol, 
				     const std::string& filename, 
				     unsigned int var,
				     unsigned int time,
				     const bounding_box& subvolbox) const
  {
    volume_file_info volinfo(ctx, filename);
    if(!subvolbox.isWithin(volinfo.boundingBox()))
      throw sub_volume_out_of_bounds("The subvolume bounding box must be within the file's bounding box.");
    uint64 off_x = uint64((subvolbox.minx - volinfo.XMin())/volinfo.XSpan());
    uint64 off_y = uint64((subvolbox.miny - volinfo.YMin())/volinfo.YSpan());
    uint64 off_z = uint64((subvolbox.minz - volinfo.ZMin())/volinfo.ZSpan());
    dimension dim;
    dim[0] = uint64((subvolbox.maxx - subvolbox.minx)/volinfo.XSpan())+1;
    dim[1] = uint64((subvolbox.maxy - subvolbox.miny)/volinfo.YSpan())+1;
    dim[2] = uint64((subvolbox.maxz - subvolbox.minz)/volinfo.ZSpan())+1;
    for(int i = 0; i < 3; i++) if(dim[i] == 0) dim[i]=1;
    if(dim[0] + off_x > volinfo.XDim()) dim[0] = volinfo.XDim() - off_x;
    if(dim[1] + off_y > volinfo.YDim()) dim[1] = volinfo.YDim() - off_y;
    if(dim[2] + off_z > volinfo.ZDim()) dim[2] = volinfo.ZDim() - off_z;
    readVolumeFile(ctx, vol, filename, var, time, off_x, off_y, off_z, dim);
    //vol.sub(subvolbox,dim); //get a subvolume that is exactly the size of subvolbox
    //just force the bounding box for now.. this might lead to aliasing errors
    vol.boundingBox(subvolbox);
  }
  
  // --------------------------------
  // volume_file_io::writeBoundingBox
  // --------------------------------
  // Purpose:
  //   Writes the specified bounding box to the file.  The default implementation is slow
  //   because it has to read the entire file.  This can be sped up on an individual file type
  //   basis.
  // ---- Change History ----
  // 04/06/2012 -- Joe R. -- Creation.
  void volume_file_io::writeBoundingBox(app& ctx,
                                        const bounding_box& bbox,
                                        const std::string& filename) const
  {
    std::vector<volume> vols;
    volume_file_info vfi(ctx, filename);
    vfi.boundingBox(bbox);
    CVC_NAMESPACE::readVolumeFile(ctx, vols, filename);
    BOOST_FOREACH(volume& vol, vols)
      vol.boundingBox(bbox);
    CVC_NAMESPACE::createVolumeFile(ctx, filename, vfi); //TODO: don't overwrite existing file until temp file write is complete
    CVC_NAMESPACE::writeVolumeFile(ctx, vols, filename);
  }

  // ---------------------------
  // volume_file_io::handlersMap
  // ---------------------------
  // Purpose:
  //   Static initialization of handler map.  Clients use the handler_map
  //   to add themselves to the collection of objects that are to be used
  //   to perform volume file i/o operations.
  // ---- Change History ----
  // 11/13/2009 -- Joe R. -- Creation.
  // 12/27/2013 -- Joe R. -- Handlers now responsible for adding themselves to the map.
  volume_file_io::handler_map& volume_file_io::handlerMap()
  {
    //It's ok to leak: http://www.parashift.com/c++-faq-lite/ctors.html#faq-10.15
    static handler_map* p = initializeMap();
    return *p;
  }

  // -----------------------------
  // volume_file_io::insertHandler
  // -----------------------------
  // Purpose:
  //   Convenence function for adding objects to the map.
  // ---- Change History ----
  // 11/13/2009 -- Joe R. -- Creation.
  // 11/20/2009 -- Joe R. -- Removed extension arg.  Now using what the object provides.
  void volume_file_io::insertHandler(const ptr& vfio)
  {
    insertHandler(handlerMap(),vfio);
  }

  // -----------------------------
  // volume_file_io::removeHandler
  // -----------------------------
  // Purpose:
  //   Convenence function for removing objects from the map.
  // ---- Change History ----
  // 11/13/2009 -- Joe R. -- Creation.
  void volume_file_io::removeHandler(const ptr& vfio)
  {
    for(handler_map::iterator i = handlerMap().begin();
	i != handlerMap().end();
	i++)
      {
	handlers h;
	for(handlers::iterator j = i->second.begin();
	    j != i->second.end();
	    j++)
	  {
	    if(*j != vfio)
	      h.push_back(*j);
	  }
	i->second = h;
      }
  }

  // ----------------------------
  // volume_file_io::removeHandler
  // ----------------------------
  // Purpose:
  //   Convenence function for removing objects from the map.
  // ---- Change History ----
  // 11/13/2009 -- Joe R. -- Creation.
  void volume_file_io::removeHandler(const std::string& id)
  {
    for(handler_map::iterator i = handlerMap().begin();
	i != handlerMap().end();
	i++)
      {
	handlers h;
	for(handlers::iterator j = i->second.begin();
	    j != i->second.end();
	    j++)
	  {
	    if((*j)->id() != id)
	      h.push_back(*j);
	  }
	i->second = h;
      }    
  }

  // ----------------------------
  // volume_file_io::getExtensions
  // ----------------------------
  // Purpose:
  //   Returns the list of supported file extensions.
  // ---- Change History ----
  // 09/18/2011 -- Joe R. -- Creation.    
  std::vector<std::string> volume_file_io::getExtensions()
  {
    std::vector<std::string> ret;
    BOOST_FOREACH(handler_map::value_type& i, handlerMap())
      {
        ret.push_back(i.first);
      }
    return ret;
  }

  // ----------------------------
  // volume_file_io::initializeMap
  // ----------------------------
  // Purpose:
  //   Adds the standard volume_file_io objects to a new handler_map object
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  volume_file_io::handler_map *volume_file_io::initializeMap()
  {
    handler_map *map = new handler_map;    
    return map;
  }

  // ----------------------------
  // volume_file_io::insertHandler
  // ----------------------------
  // Purpose:
  //   Convenence function for adding objects to the specified map.
  // ---- Change History ----
  // 11/20/2009 -- Joe R. -- Creation.
  void volume_file_io::insertHandler(handler_map& hm,
				     const ptr& vfio)
  {
    if(!vfio) return;
    for(volume_file_io::extension_list::const_iterator i =   
	  vfio->extensions().begin();                      
	i != vfio->extensions().end();                     
	i++)
      {
	hm[*i].push_back(vfio);
      }
  }

  void readVolumeFile(volume& vol, 
		      const std::string& filename,
		      unsigned int var, unsigned int time)
  {
    readVolumeFile(app::instance(), vol, filename, var, time);
  }

  // --------------
  // readVolumeFile
  // --------------
  // Purpose:
  //   The main readVolumeFile function.  Refers to the handler map to choose
  //   an appropriate IO object for reading the requested volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/13/2009 -- Joe R. -- Re-implemented using volume_file_io handler map.
  // 12/28/2009 -- Joe R. -- Collecting exception error strings
  // 09/05/2011 -- Joe R. -- Using splitRawFilename to extract real filename
  //                         if the provided filename is a file|obj tuple.
  void readVolumeFile(volume& vol,
		      const std::string& filename, 
		      unsigned int var, unsigned int time,
		      uint64 off_x, uint64 off_y, uint64 off_z,
		      const dimension& subvoldim)
  {
    readVolumeFile(app::instance(), vol, filename, var, time,
                   off_x, off_y, off_z, subvoldim);
  }

  // --------------
  // readVolumeFile (ctx-aware)
  // --------------
  void readVolumeFile(app& ctx, volume& vol,
                      const std::string& filename,
                      unsigned int var, unsigned int time,
                      uint64 off_x, uint64 off_y, uint64 off_z,
                      const dimension& subvoldim)
  {
    vol.unsetMinMax();

    std::string errors;
    boost::smatch what;

    std::string actualFileName;
    std::string objectName;

    boost::tie(actualFileName, objectName) =
      volume_file_io::splitRawFilename(filename);

    const boost::regex file_extension(volume_file_io::FILE_EXTENSION_EXPR);
    if(boost::regex_match(actualFileName, what, file_extension))
      {
	if(volume_file_io::handlerMap()[what[2]].empty())
	  throw unsupported_volume_file_type(std::string(BOOST_CURRENT_FUNCTION) + 
					     std::string(": Cannot read ") + filename);
	volume_file_io::handlers& h = volume_file_io::handlerMap()[what[2]];
	//use the first handler that succeds
	for(volume_file_io::handlers::iterator i = h.begin();
	    i != h.end();
	    i++)
	  try
	    {
	      if(*i)
		{
		  (*i)->readVolumeFile(ctx, vol, filename, var, time,
				       off_x, off_y, off_z, subvoldim);
		  return;
		}
	    }
	  catch(exception& e)
	    {
	      errors += std::string(" :: ") + e.what();
	    }
      }
    throw unsupported_volume_file_type(
      boost::str(
	boost::format("%1% : Cannot read '%2%'%3%") % 
	BOOST_CURRENT_FUNCTION %
	filename %
	errors
      )
    );
  }

  // --------------
  // readVolumeFile
  // --------------
  // Purpose:
  //    Same as above except it uses a bounding box.
  //   
  // ---- Change History ----
  // ??/??/2009 -- Joe R. -- Creation.
  // 01/03/2010 -- Joe R. -- Re-implemented using volume_file_io handler map.
  // 09/05/2011 -- Joe R. -- Using splitRawFilename to extract real filename
  //                         if the provided filename is a file|obj tuple.
  void readVolumeFile(volume& vol,
		      const std::string& filename, 
		      unsigned int var, unsigned int time,
		      const bounding_box& subvolbox)
  {
    readVolumeFile(app::instance(), vol, filename, var, time, subvolbox);
  }

  // --------------
  // readVolumeFile (ctx-aware, bbox)
  // --------------
  void readVolumeFile(app& ctx, volume& vol,
                      const std::string& filename,
                      unsigned int var, unsigned int time,
                      const bounding_box& subvolbox)
  {
    vol.unsetMinMax();

    std::string errors;
    boost::smatch what;

    std::string actualFileName;
    std::string objectName;

    boost::tie(actualFileName, objectName) =
      volume_file_io::splitRawFilename(filename);

    const boost::regex file_extension(volume_file_io::FILE_EXTENSION_EXPR);
    if(boost::regex_match(actualFileName, what, file_extension))
      {
	if(volume_file_io::handlerMap()[what[2]].empty())
	  throw unsupported_volume_file_type(std::string(BOOST_CURRENT_FUNCTION) + 
					     std::string(": Cannot read ") + filename);
	volume_file_io::handlers& h = volume_file_io::handlerMap()[what[2]];
	//use the first handler that succeds
	for(volume_file_io::handlers::iterator i = h.begin();
	    i != h.end();
	    i++)
	  try
	    {
	      if(*i)
		{
		  (*i)->readVolumeFile(ctx, vol, filename, var, time,
				       subvolbox);
		  return;
		}
	    }
	  catch(exception& e)
	    {
	      errors += std::string(" :::: ") + e.what();
	    }
      }
    throw unsupported_volume_file_type(
      boost::str(
	boost::format("%1% : Cannot read '%2%'%3%") % 
	BOOST_CURRENT_FUNCTION %
	filename %
	errors
      )
    );
  }

  void readVolumeFile(std::vector<volume>& vols,
		      const std::string& filename)
  {
    readVolumeFile(app::instance(), vols, filename);
  }

  // ---------------
  // writeVolumeFile
  // ---------------
  // Purpose:
  //   The main writeVolumeFile function.  Refers to the handler map to choose
  //   an appropriate IO object for writing to the requested volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/13/2009 -- Joe R. -- Re-implemented using volume_file_io handler map.
  // 12/28/2009 -- Joe R. -- Collecting exception error strings
  // 09/05/2011 -- Joe R. -- Using splitRawFilename to extract real filename
  //                         if the provided filename is a file|obj tuple.
  void writeVolumeFile(const volume& vol, 
		       const std::string& filename,
		       unsigned int var, unsigned int time,
		       uint64 off_x, uint64 off_y, uint64 off_z)
  {
    writeVolumeFile(app::instance(), vol, filename, var, time,
                    off_x, off_y, off_z);
  }

  // ---------------
  // writeVolumeFile (ctx-aware, offset)
  // ---------------
  void writeVolumeFile(app& ctx, const volume& vol,
                       const std::string& filename,
                       unsigned int var, unsigned int time,
                       uint64 off_x, uint64 off_y, uint64 off_z)
  {
    std::string errors;
    boost::smatch what;

    std::string actualFileName;
    std::string objectName;

    boost::tie(actualFileName, objectName) =
      volume_file_io::splitRawFilename(filename);

    const boost::regex file_extension(volume_file_io::FILE_EXTENSION_EXPR);
    if(boost::regex_match(actualFileName, what, file_extension))
      {
	if(volume_file_io::handlerMap()[what[2]].empty())
	  throw unsupported_volume_file_type(std::string(BOOST_CURRENT_FUNCTION) + 
					     std::string(": Cannot write ") + filename);
	volume_file_io::handlers& h = volume_file_io::handlerMap()[what[2]];
	//use the first handler that succeds
	for(volume_file_io::handlers::iterator i = h.begin();
	    i != h.end();
	    i++)
	  try
	    {
	      if(*i)
		{
		  (*i)->writeVolumeFile(ctx, vol, filename, var, time,
                                        off_x, off_y, off_z);
		  return;
		}
	    }
	  catch(exception& e)
	    {
	      errors += std::string(" :: ") + e.what();
	    }
      }
    throw unsupported_volume_file_type(
      boost::str(
	boost::format("%1% : Cannot read '%2%'%3%") % 
	BOOST_CURRENT_FUNCTION %
	filename %
	errors
      )
    );
  }

  // ---------------
  // writeVolumeFile
  // ---------------
  // Purpose:
  //   Writes a volume to the specified filename, using the specified bounding box
  //   as the target to fill in the write volume.  Interpolates the input volume
  //   appropriately.
  // ---- Change History ----
  // ??/??/2010 -- Joe R. -- I think this came about sometime in 2010.  It's a nasty
  //                         and slow function but I think it does the trick.
  // 09/10/2011 -- Joe R. -- Adding thread progress feedback.
  // 09/11/2011 -- Joe R. -- Fixing an indexing bug.
  void writeVolumeFile(app& ctx, const volume& vol, 
		       const std::string& filename,
		       unsigned int var, unsigned int time,
		       const bounding_box& subvolbox)
  {
    thread_info ti(ctx, BOOST_CURRENT_FUNCTION);

    volume localvol(vol);
    volume_file_info volinfo(ctx, filename);
    if(!subvolbox.isWithin(volinfo.boundingBox()))
      throw sub_volume_out_of_bounds("The subvolume bounding box must be within the file's bounding box.");

    //set up a subvolume to represent the chunk we are writing to in the file
    uint64 off_x = uint64((subvolbox.minx - volinfo.XMin())/volinfo.XSpan());
    uint64 off_y = uint64((subvolbox.miny - volinfo.YMin())/volinfo.YSpan());
    uint64 off_z = uint64((subvolbox.minz - volinfo.ZMin())/volinfo.ZSpan());
    dimension dim;
    dim[0] = uint64((subvolbox.maxx - subvolbox.minx)/volinfo.XSpan()+1);
    dim[1] = uint64((subvolbox.maxy - subvolbox.miny)/volinfo.YSpan()+1);
    dim[2] = uint64((subvolbox.maxz - subvolbox.minz)/volinfo.ZSpan()+1);
    if(dim[0] + off_x > volinfo.XDim()) dim[0] = volinfo.XDim() <= 1 ? 1 : volinfo.XDim() - off_x;
    if(dim[1] + off_y > volinfo.YDim()) dim[1] = volinfo.YDim() <= 1 ? 1 : volinfo.YDim() - off_y;
    if(dim[2] + off_z > volinfo.ZDim()) dim[2] = volinfo.ZDim() <= 1 ? 1 : volinfo.ZDim() - off_z;

    volume subvol(ctx, dim,volinfo.voxelTypes(var),
		  bounding_box(volinfo.XMin() + off_x*volinfo.XSpan(),
					 volinfo.YMin() + off_y*volinfo.YSpan(),
					 volinfo.ZMin() + off_z*volinfo.ZSpan(),
					 volinfo.XMin() + (off_x+dim[0])*volinfo.XSpan(),
					 volinfo.YMin() + (off_y+dim[1])*volinfo.YSpan(),
					 volinfo.ZMin() + (off_z+dim[2])*volinfo.ZSpan()));
    
    //reset the bounding box for easy interpolation
    localvol.boundingBox(bounding_box(0.0,0.0,0.0,
						1.0,1.0,1.0));

    double xmax = subvol.XDim() > 1 ? double(subvol.XDim()-1) : 1.0;
    double ymax = subvol.YDim() > 1 ? double(subvol.YDim()-1) : 1.0;
    double zmax = subvol.ZDim() > 1 ? double(subvol.ZDim()-1) : 1.0;
    for(uint64 k = 0; k < subvol.ZDim(); k++)
      {
        for(uint64 j = 0; j < subvol.YDim(); j++)
          for(uint64 i = 0; i < subvol.XDim(); i++)
            {
              subvol(i,j,k, localvol.interpolate(double(i)/xmax,
                                                 double(j)/ymax,
                                                 double(k)/zmax));
            }
        ctx.threadProgress(float(k)/float(subvol.ZDim()));
      }
    
    ctx.threadProgress(1.0);
    writeVolumeFile(ctx, subvol, filename, var, time, off_x, off_y, off_z);
  }

  void writeVolumeFile(const volume& vol, 
		       const std::string& filename,
		       unsigned int var, unsigned int time,
		       const bounding_box& subvolbox)
  {
    writeVolumeFile(app::instance(), vol, filename, var, time, subvolbox);
  }

  // ---------------
  // writeVolumeFile
  // ---------------
  // Purpose:
  //   Writes a collection of volumes to the specified file if the file type supports
  //   such an operation.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  void writeVolumeFile(const std::vector<volume>& vols,
		       const std::string& filename)
  {
    writeVolumeFile(app::instance(), vols, filename);
  }

  // ----------------
  // createVolumeFile
  // ----------------
  // Purpose:
  //   The main createVolumeFile function.  Refers to the handler map to choose
  //   an appropriate IO object for creating the requested volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Creation.
  // 11/13/2009 -- Joe R. -- Re-implemented using volume_file_io handler map.
  // 12/28/2009 -- Joe R. -- Collecting exception error strings
  // 09/05/2011 -- Joe R. -- Using splitRawFilename to extract real filename
  //                         if the provided filename is a file|obj tuple.
  void createVolumeFile(const std::string& filename,
			const bounding_box& boundingBox,
			const dimension& dimension,
			const std::vector<data_type>& voxelTypes,
			unsigned int numVariables, unsigned int numTimesteps,
			double min_time, double max_time)
  {
    createVolumeFile(app::instance(), filename, boundingBox, dimension,
		     voxelTypes, numVariables, numTimesteps,
		     min_time, max_time);
  }

  // ---------------
  // readBoundingBox
  // ---------------
  // Purpose:
  //  Returns a volume file's bounding box.
  // ---- Change History ----
  // 04/06/2012 -- Joe R. -- Creation.
  bounding_box readBoundingBox(const std::string& filename)
  {
    return volume_file_info(filename).boundingBox();
  }

  // ----------------
  // writeBoundingBox
  // ----------------
  // Purpose:
  //  Changes a volume file's bounding box.
  // ---- Change History ----
  // 04/06/2012 -- Joe R. -- Creation.
  void writeBoundingBox(const bounding_box& bbox, const std::string& filename)
  {
    writeBoundingBox(app::instance(), bbox, filename);
  }

  void writeBoundingBox(app& ctx,
                        const bounding_box& bbox,
                        const std::string& filename)
  {
    std::string errors;
    boost::smatch what;

    std::string actualFileName;
    std::string objectName;

    boost::tie(actualFileName, objectName) =
      volume_file_io::splitRawFilename(filename);

    const boost::regex file_extension(volume_file_io::FILE_EXTENSION_EXPR);
    if(boost::regex_match(actualFileName, what, file_extension))
      {
	if(volume_file_io::handlerMap()[what[2]].empty())
	  throw unsupported_volume_file_type(std::string(BOOST_CURRENT_FUNCTION) + 
					     std::string(": Cannot write ") + filename);
	volume_file_io::handlers& h = volume_file_io::handlerMap()[what[2]];
	//use the first handler that succeds
	for(volume_file_io::handlers::iterator i = h.begin();
	    i != h.end();
	    i++)
	  try
	    {
	      if(*i)
		{
		  (*i)->writeBoundingBox(ctx, bbox, filename);
		  return;
		}
	    }
	  catch(exception& e)
	    {
	      errors += std::string(" :: ") + e.what();
	    }
      }
    throw unsupported_volume_file_type(
      boost::str(
	boost::format("%1% : Cannot write '%2%'%3%") % 
	BOOST_CURRENT_FUNCTION %
	filename %
	errors
      )
    );
  }

  // ---- app& overloads ----
  // These accept an explicit app context instead of using the singleton.
  // The ctx-aware versions are primary; legacy overloads delegate to them
  // via app::instance() for backward compatibility.

  void readVolumeFile(app& ctx, volume& vol,
		      const std::string& filename,
		      unsigned int var, unsigned int time)
  {
    volume_file_info volinfo(ctx, filename);
    readVolumeFile(ctx, vol, filename, var, time, 0, 0, 0,
                   volinfo.voxel_dimensions());
  }

  void readVolumeFile(app& ctx, std::vector<volume>& vols,
		      const std::string& filename)
  {
    volume_file_info volinfo(ctx, filename);
    volume vol(ctx);
    vols.clear();
    for(unsigned int var=0; var<volinfo.numVariables(); var++)
      for(unsigned int time=0; time<volinfo.numTimesteps(); time++)
	{
	  readVolumeFile(ctx, vol, filename, var, time);
	  vols.push_back(vol);
	}
  }

  void writeVolumeFile(app& ctx, const std::vector<volume>& vols,
		       const std::string& filename)
  {
    uint64 i;

    if(vols.size() == 0) return;

    std::vector<data_type> voxelTypes;
    for(i=0; i<vols.size(); i++) voxelTypes.push_back(vols[i].voxelType());

    createVolumeFile(ctx, filename, vols[0].boundingBox(),
		     vols[0].voxel_dimensions(), voxelTypes, vols.size());
    for(i=0; i<vols.size(); i++)
      writeVolumeFile(ctx, vols[i], filename, i);
  }

  void createVolumeFile(app& ctx, const std::string& filename,
			const bounding_box& boundingBox,
			const dimension& dimension,
			const std::vector<data_type>& voxelTypes,
			unsigned int numVariables, unsigned int numTimesteps,
			double min_time, double max_time)
  {
    std::string errors;
    boost::smatch what;

    std::string actualFileName;
    std::string objectName;

    boost::tie(actualFileName, objectName) =
      volume_file_io::splitRawFilename(filename);

    const boost::regex file_extension(volume_file_io::FILE_EXTENSION_EXPR);
    if(boost::regex_match(actualFileName, what, file_extension))
      {
	if(volume_file_io::handlerMap()[what[2]].empty())
	  throw unsupported_volume_file_type(std::string(BOOST_CURRENT_FUNCTION) +
					     std::string(": Cannot create ") + filename);
	volume_file_io::handlers& h = volume_file_io::handlerMap()[what[2]];
	//use the first handler that succeds
	for(volume_file_io::handlers::iterator i = h.begin();
	    i != h.end();
	    i++)
	  try
	    {
	      if(*i)
		{
		  (*i)->createVolumeFile(ctx, filename, boundingBox, dimension,
					 voxelTypes, numVariables, numTimesteps,
					 min_time, max_time);
		  return;
		}
	    }
	  catch(exception& e)
	    {
	      errors += std::string(" :: ") + e.what();
	    }
      }
    throw unsupported_volume_file_type(
      boost::str(
	boost::format("%1% : Cannot read '%2%'%3%") %
	BOOST_CURRENT_FUNCTION %
	filename %
	errors
      )
    );
  }

}
