/*
  Copyright 2002-2003 The University of Texas at Austin
  
	Authors: Anthony Thane <thanea@ices.utexas.edu>
	Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of Volume Rover.

  Volume Rover is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  Volume Rover is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// RawFile.h: interface for the RawFile class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RAWFILE_H__C647E6C6_7669_4CFE_AFBC_BB072A6A8EC7__INCLUDED_)
#define AFX_RAWFILE_H__C647E6C6_7669_4CFE_AFBC_BB072A6A8EC7__INCLUDED_

#include "GeometryFileType.h"

//using std::string;

class RawFile : public GeometryFileType  
{
public:
	virtual ~RawFile();

	virtual Geometry* loadFile(const string& fileName);
	virtual bool checkType(const string& fileName);
	virtual bool saveFile(const Geometry* geometry, const string& fileName);

	virtual string extension() { return "raw"; };
	virtual string filter() { return "Raw files (*.raw)"; };

	static RawFile ms_RawFileRepresentative;
	static GeometryFileType* getRepresentative();

protected:
	RawFile();


};

#endif // !defined(AFX_RAWFILE_H__C647E6C6_7669_4CFE_AFBC_BB072A6A8EC7__INCLUDED_)
