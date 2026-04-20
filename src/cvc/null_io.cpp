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

#include <boost/current_function.hpp>
#include <boost/format.hpp>

#include <cvc/volume_file_io.h>

namespace CVC_NAMESPACE
{
  // -------
  // null_io
  // -------
  // Purpose: 
  //   Template for IO functionality.  Copy this class and fill out the functions
  //   when adding support for a new file type.
  // ---- Change History ----
  // 12/04/2009 -- Joe R. -- Creation.
  struct null_io : public volume_file_io
  {
    // ----------------
    // null_io::null_io
    // ----------------
    // Purpose:
    //   Initializes the extension list and id.
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    null_io()
      : _id("null_io : v1.0")
    {
      _extensions.push_back(".nothing");
    }

    // -----------
    // null_io::id
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
    // null_io::extensions
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
    // null_io::getVolumeFileInfo
    // --------------------------
    // Purpose:
    //   Writes to a structure containing all info that VolMagick needs
    //   from a volume file.
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    virtual void getVolumeFileInfo(volume_file_info::data& /*data*/,
				   const std::string& /*filename*/) const
    {
      throw read_error(
        boost::str(
	  boost::format("%1% unimplemented") % BOOST_CURRENT_FUNCTION
        )
      );
    }

    // -----------------------
    // null_io::readVolumeFile
    // -----------------------
    // Purpose:
    //   Writes to a volume object after reading from a volume file.
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    virtual void readVolumeFile(volume& /*vol*/,
				const std::string& /*filename*/, 
				unsigned int /*var*/, unsigned int /*time*/,
				uint64 /*off_x*/, uint64 /*off_y*/, uint64 /*off_z*/,
				const dimension& /*subvoldim*/) const
    {
      throw read_error(
        boost::str(
          boost::format("%1% unimplemented") % BOOST_CURRENT_FUNCTION
        )
      );
    }

    // -------------------------
    // null_io::createVolumeFile
    // -------------------------
    // Purpose:
    //   Creates an empty volume file to be later filled in by writeVolumeFile
    // ---- Change History ----
    // 12/04/2009 -- Joe R. -- Creation.
    virtual void createVolumeFile(const std::string& /*filename*/,
				  const bounding_box& /*boundingBox*/,
				  const dimension& /*dimension*/,
				  const std::vector<data_type>& /*voxelTypes*/,
				  unsigned int /*numVariables*/, unsigned int /*numTimesteps*/,
				  double /*min_time*/, double /*max_time*/) const
    {
      throw write_error(
        boost::str(
          boost::format("%1% unimplemented") % BOOST_CURRENT_FUNCTION
        )
      );
    }

    // ------------------------
    // null_io::writeVolumeFile
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
    virtual void writeVolumeFile(const volume& /*wvol*/, 
				 const std::string& /*filename*/,
				 unsigned int /*var*/, unsigned int /*time*/,
				 uint64 /*off_x*/, uint64 /*off_y*/, uint64 /*off_z*/) const
    {
      throw write_error(
        boost::str(
          boost::format("%1% unimplemented") % BOOST_CURRENT_FUNCTION
        )
      );
    }

  protected:
    std::string _id;
    extension_list _extensions;
  };
}

namespace CVC_NAMESPACE
{
  void register_null_io(app& /*ctx*/)
  {
    volume_file_io::insertHandler(
      volume_file_io::ptr(new null_io)
    );
  }
}
