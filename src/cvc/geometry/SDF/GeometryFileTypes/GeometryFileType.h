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

// GeometryFileType.h: interface for the GeometryFileType class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_GEOMETRYFILETYPE_H__3D1E7183_20B6_4CBE_A5E4_F0C92642AB4E__INCLUDED_)
#define AFX_GEOMETRYFILETYPE_H__3D1E7183_20B6_4CBE_A5E4_F0C92642AB4E__INCLUDED_

//#include <qstring.h>
#include <string>
using std::string;

class Geometry;

class GeometryFileType  
{
public:
	GeometryFileType();
	virtual ~GeometryFileType();

	virtual Geometry* loadFile(const string& fileName) = 0;
	virtual bool checkType(const string& fileName) = 0;
	virtual bool saveFile(const Geometry* geometry, const string& fileName) = 0;

	virtual string extension() = 0;
	virtual string filter() = 0;

};

#endif // !defined(AFX_GEOMETRYFILETYPE_H__3D1E7183_20B6_4CBE_A5E4_F0C92642AB4E__INCLUDED_)
