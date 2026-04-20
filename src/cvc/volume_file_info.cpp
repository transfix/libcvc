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

#include <cvc/volume_file_info.h>
#include <cvc/exception.h>
#include <cvc/volume_file_io.h>
#include <cvc/utility.h>
#include <cvc/app.h>

#include <boost/regex.hpp>

namespace CVC_NAMESPACE
{
  // --------------------
  // volume_file_info::read
  // --------------------
  // Purpose:
  //   Refers to the handler map to choose an appropriate IO object for reading
  //    the requested volume file.  Use this function to initialize the volume_file_info
  //    object with info from a volume file.
  // ---- Change History ----
  // ??/??/2007 -- Joe R. -- Initially implemented.
  // 11/13/2009 -- Joe R. -- Re-implemented using VolumeFile_IO handler map
  // 12/28/2009 -- Joe R. -- Collecting exception error strings
  // 09/08/2011 -- Joe R. -- Using splitRawFilename to extract real filename
  //                         if the provided filename is a file|obj tuple.
  void volume_file_info::read(const std::string& filename)
  {
    std::string errors;
    boost::regex file_extension("^(.*)(\\.\\S*)$");
    boost::smatch what;

    std::string actualFileName;
    std::string objectName;

    boost::tie(actualFileName, objectName) =
      volume_file_io::splitRawFilename(filename);

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
		  (*i)->getVolumeFileInfo(_data,filename);
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

  void volume_file_info::calcMinMax(unsigned int var, unsigned int time) const
  {
    thread_info ti(BOOST_CURRENT_FUNCTION);

    volume vol(app::instance());
    const uint64 maxdim = 128; //read in 128^3 chunks
    for(unsigned int off_z = 0; off_z < ZDim(); off_z+=maxdim)
      for(unsigned int off_y = 0; off_y < YDim(); off_y+=maxdim)
	for(unsigned int off_x = 0; off_x < XDim(); off_x+=maxdim)
	  {
	    dimension read_dim(std::min(XDim()-off_x,maxdim),
			       std::min(YDim()-off_y,maxdim),
			       std::min(ZDim()-off_z,maxdim));
	    readVolumeFile(vol,filename(),var,time,
			   off_x,off_y,off_z,read_dim);
	    if(off_x==0 && off_y==0 && off_z==0)
	      {
		_data._min[var][time] = vol.min();
		_data._max[var][time] = vol.max();
	      }
	    else
	      {
		if(_data._min[var][time] > vol.min())
		  _data._min[var][time] = vol.min();
		if(_data._max[var][time] < vol.max())
		  _data._max[var][time] = vol.max();
	      }
	  }

    _data._minIsSet[var][time] = true;
    _data._maxIsSet[var][time] = true;
  }
}
