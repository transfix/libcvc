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

#ifndef __VOLMAGICK_VOLUMEFILE_IO_H__
#define __VOLMAGICK_VOLUMEFILE_IO_H__

#include <cvc/volume_file_info.h>
#include <cvc/volume.h>
#include <cvc/exception.h>

#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/regex.hpp>

#include <string>
#include <list>
#include <map>
#include <vector>

namespace CVC_NAMESPACE
{
  class app; // Forward declaration for app& overloads

  CVC_DEF_EXCEPTION(unsupported_volume_file_type);

  // --------------
  // volume_file_io
  // --------------
  // Purpose: 
  //   Provides functions for accessing and creating volume files. Meant
  //   to be inherited by clients of VolMagick whishing to add support
  //   for any particular volume file format.  The default implementation
  //   does nothing.
  // ---- Change History ----
  // 11/13/2009 -- Joe R. -- Creation.
  // 09/05/2011 -- Joe R. -- Moved splitRawFilename and 2 const strings here
  //                         from HDF5_IO.
  // 09/10/2011 -- Joe R. -- Just using char* and constructing regexes locally to avoid
  //                         crashes if the main thread finishes before child threads.
  struct volume_file_io
  {
    static const char* CVC_VOLUME_GROUP; // /cvc/volumes
    static const char* DEFAULT_VOLUME_NAME;
    static const char* FILE_EXTENSION_EXPR;

    // --------------------------------
    // volume_file_io::splitRawFilename
    // --------------------------------
    // Purpose:
    //   Splits a filename into an actual file name and an object
    //   path.
    // ---- Change History ----
    // 07/24/2011 -- Joe R. -- Creation.
    // 09/05/2011 -- Joe R. -- Moved here from HDF5_IO
    static boost::tuple<
      std::string, /* actual file name */
      std::string  /* hdf5 object name */
    > splitRawFilename(const std::string& filename);

    // ------------------
    // volume_file_io::id
    // ------------------
    // Purpose:
    //   Returns a string that identifies this volume_file_io object.  This should
    //   be unique, but is freeform.
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    virtual const std::string& id() const = 0;

    typedef std::list<std::string> extension_list;

    // --------------------------
    // volume_file_io::extensions
    // --------------------------
    // Purpose:
    //   Returns a list of extensions that this volume_file_io object supports.
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    virtual const extension_list& extensions() const = 0;

    // ---------------------------------
    // volume_file_io::getVolumeFileInfo
    // ---------------------------------
    // Purpose:
    //   Writes to a structure containing all info that VolMagick needs
    //   from a volume file.
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    virtual void getVolumeFileInfo(app& /*ctx*/,
                                   volume_file_info::data& /*data*/,
                                   const std::string& /*filename*/) const = 0;

    // -----------------------------
    // volume_file_io::readVolumeFile
    // -----------------------------
    // Purpose:
    //   Writes to a Volume object after reading from a volume file.  
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    virtual void readVolumeFile(app& /*ctx*/,
                                volume& /*vol*/,
				const std::string& /*filename*/, 
				unsigned int /*var*/,
				unsigned int /*time*/,
				uint64 /*off_x*/,
				uint64 /*off_y*/,
				uint64 /*off_z*/,
				const dimension& /*subvoldim*/) const = 0;

    // ------------------------------
    // volume_file_io::readVolumeFile
    // ------------------------------
    // Purpose:
    //   Same as above except uses a bounding box for specifying the
    //   subvol.  A default implementation is provided. 
    // ---- Change History ----
    // 01/03/2010 -- Joe R. -- Creation.
    virtual void readVolumeFile(app& ctx,
                                volume& vol, 
				const std::string& filename, 
				unsigned int var,
				unsigned int time,
				const bounding_box& subvolbox) const;

    // --------------------------------
    // volume_file_io::createVolumeFile
    // --------------------------------
    // Purpose:
    //   Creates an empty volume file to be later filled in by writeVolumeFile
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    virtual void createVolumeFile(app& /*ctx*/,
                                  const std::string& /*filename*/,
                                  const bounding_box& /*boundingBox*/,
                                  const dimension& /*dimension*/,
                                  const std::vector<data_type>& /*voxelTypes*/,
                                  unsigned int /*numVariables*/,
                                  unsigned int /*numTimesteps*/,
                                  double /*min_time*/,
                                  double /*max_time*/) const = 0;

    // -------------------------------
    // volume_file_io::writeVolumeFile
    // -------------------------------
    // Purpose:
    //   Writes the volume contained in wvol to the specified volume file. Should create
    //   a volume file if the filename provided doesn't exist.  Else it will simply
    //   write data to the existing file.  A common user error arises when you try to
    //   write over an existing volume file using this function for unrelated volumes.
    //   If what you desire is to overwrite an existing volume file, first run
    //   createVolumeFile to replace the volume file.
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    virtual void writeVolumeFile(app& /*ctx*/,
                                 const volume& /*wvol*/,
				 const std::string& /*filename*/,
				 unsigned int /*var*/,
				 unsigned int /*time*/,
				 uint64 /*off_x*/,
				 uint64 /*off_y*/,
				 uint64 /*off_z*/) const = 0;

    // --------------------------------
    // volume_file_io::writeBoundingBox
    // --------------------------------
    // Purpose:
    //   Writes the specified bounding box to the file.  The default implementation is slow
    //   because it has to read the entire file.  This can be sped up on an individual file type
    //   basis.
    // ---- Change History ----
    // 04/06/2012 -- Joe R. -- Creation.
    virtual void writeBoundingBox(app& ctx,
                                  const bounding_box& bbox,
                                  const std::string& filename) const;

    typedef boost::shared_ptr<volume_file_io> ptr;
    typedef std::vector<ptr> handlers;
    typedef std::map<
      std::string, /* file extension */
      handlers > handler_map;

    // --------------------------
    // volume_file_io::handlerMap
    // --------------------------
    // Purpose:
    //   Static initialization of handler map.  Clients use the handler_map
    //   to add themselves to the collection of objects that are to be used
    //   to perform volume file i/o operations.
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    static handler_map& handlerMap();

    // -----------------------------
    // volume_file_io::insertHandler
    // -----------------------------
    // Purpose:
    //   Convenience function for adding objects to the map.
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    static void insertHandler(const ptr& vfio);

    // -----------------------------
    // volume_file_io::removeHandler
    // -----------------------------
    // Purpose:
    //   Convenience function for removing objects from the map.
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    static void removeHandler(const ptr& vfio);

    // -----------------------------
    // volume_file_io::removeHandler
    // -----------------------------
    // Purpose:
    //   Convenience function for removing objects from the map.
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    static void removeHandler(const std::string& name);

    // -----------------------------
    // volume_file_io::getExtensions
    // -----------------------------
    // Purpose:
    //   Returns the list of supported file extensions.
    // ---- Change History ----
    // 09/18/2011 -- Joe R. -- Creation.    
    static std::vector<std::string> getExtensions();

  private:
    // -----------------------------
    // volume_file_io::initializeMap
    // -----------------------------
    // Purpose:
    //   Adds the standard volume_file_io objects to a new handler_map object
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    static handler_map *initializeMap();

    // -----------------------------
    // volume_file_io::insertHandler
    // -----------------------------
    // Purpose:
    //   Convenience function for adding objects to the specified map.
    // ---- Change History ----
    // 11/13/2009 -- Joe R. -- Creation.
    static void insertHandler(handler_map& hm, const ptr& vfio);
  };

  // ------------------------- Volume I/O API

  /*
    read the specified subvolume from the specified file and copy it to the object
    vol.
  */
  void readVolumeFile(volume& vol,
		      const std::string& filename, 
		      unsigned int var, unsigned int time,
		      uint64 off_x, uint64 off_y, uint64 off_z,
		      const dimension& subvoldim);

  /*
    read the specified subvolume from the specified file and copy it to the object
    vol.
  */
  void readVolumeFile(volume& vol,
		      const std::string& filename, 
		      unsigned int var, unsigned int time,
		      const bounding_box& subvolbox);

  /*
    read the entire volume from the specified file and copy it to the object vol.
  */
  void readVolumeFile(volume& vol, 
		      const std::string& filename,
		      unsigned int var = 0, unsigned int time = 0);

  /*
    Read multi-volume file and add each volume to the vector vols.
  */
  void readVolumeFile(std::vector<volume>& vols,
		      const std::string& filename);

  /*
    write the specified volume to the specified offset in file 'filename'
  */
  void writeVolumeFile(const volume& vol, 
		       const std::string& filename,
		       unsigned int var = 0, unsigned int time = 0,
		       uint64 off_x = 0, uint64 off_y = 0, uint64 off_z = 0);

  /*
    write the specified volume to the specified subvolume bounding box in file 'filename'
  */
  void writeVolumeFile(const volume& vol, 
		       const std::string& filename,
		       unsigned int var, unsigned int time,
		       const bounding_box& subvolbox);

  /*
    Writes the vector 'vols' to the specified file.  Make sure that the file extension
    specified is for a volume file type that supports multi-volumes. Assumes 1 timestep.
  */
  void writeVolumeFile(const std::vector<volume>& vols,
		       const std::string& filename);

  /*
    Creates a volume file using the specified information.
  */
  void createVolumeFile(const std::string& filename,
			const bounding_box& boundingBox,
			const dimension& dimension,
			const std::vector<data_type>& voxelTypes = std::vector<data_type>(1, UChar),
			unsigned int numVariables = 1, unsigned int numTimesteps = 1,
			double min_time = 0.0, double max_time = 0.0);

  //Functions for reading and writing volume bounding boxes
  bounding_box readBoundingBox(const std::string& filename);
  void writeBoundingBox(const bounding_box& bbox, const std::string& filename);

  // ---- Overloads accepting explicit app& context ----
  // These avoid the app::instance() singleton. Legacy overloads above
  // delegate to app::instance() internally.

  void readVolumeFile(app& ctx, volume& vol,
                      const std::string& filename,
                      unsigned int var = 0, unsigned int time = 0);

  void readVolumeFile(app& ctx, volume& vol,
                      const std::string& filename,
                      unsigned int var, unsigned int time,
                      uint64 off_x, uint64 off_y, uint64 off_z,
                      const dimension& subvoldim);

  void readVolumeFile(app& ctx, volume& vol,
                      const std::string& filename,
                      unsigned int var, unsigned int time,
                      const bounding_box& subvolbox);

  void readVolumeFile(app& ctx, std::vector<volume>& vols,
                      const std::string& filename);

  void writeVolumeFile(app& ctx, const volume& vol,
                       const std::string& filename,
                       unsigned int var = 0, unsigned int time = 0,
                       uint64 off_x = 0, uint64 off_y = 0, uint64 off_z = 0);

  void writeVolumeFile(app& ctx, const volume& vol,
                       const std::string& filename,
                       unsigned int var, unsigned int time,
                       const bounding_box& subvolbox);

  void writeVolumeFile(app& ctx, const std::vector<volume>& vols,
                       const std::string& filename);

  void createVolumeFile(app& ctx, const std::string& filename,
                        const bounding_box& boundingBox,
                        const dimension& dimension,
                        const std::vector<data_type>& voxelTypes = std::vector<data_type>(1, UChar),
                        unsigned int numVariables = 1, unsigned int numTimesteps = 1,
                        double min_time = 0.0, double max_time = 0.0);

  void writeBoundingBox(app& ctx,
                        const bounding_box& bbox,
                        const std::string& filename);

  bounding_box readBoundingBox(app& ctx, const std::string& filename);
}

#endif
