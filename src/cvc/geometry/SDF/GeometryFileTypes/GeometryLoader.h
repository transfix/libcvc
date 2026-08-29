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

// GeometryLoader.h: interface for the GeometryLoader class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_GEOMETRYLOADER_H__9C808369_F58F_40BD_A59C_C06B93A64FEF__INCLUDED_)
#define AFX_GEOMETRYLOADER_H__9C808369_F58F_40BD_A59C_C06B93A64FEF__INCLUDED_

//#include <qmap.h>
#include <string>
#include <map>
using std::string;
//using std::map;

class GeometryFileType;
class Geometry;

class GeometryLoader  
{
public:
	GeometryLoader();
	virtual ~GeometryLoader();

	bool saveFile(const string& fileName, const string& selectedFilter, Geometry* geometry);
	Geometry* loadFile(const string& fileName);
	bool saveFile(const string& fileName, Geometry* geometry);

	string getLoadFilterString();
	string getSaveFilterString();

	bool isValidExtension( string extension );

protected:
	string getAllExtensions();
	bool endsWith(string str, string substr);
	Geometry* tryAll(const string& fileName);
	void addGeometryFileType(GeometryFileType* type);
	std::map<string, GeometryFileType*> m_ExtensionMap;
	std::map<string, GeometryFileType*> m_FilterMap;

};

#endif // !defined(AFX_GEOMETRYLOADER_H__9C808369_F58F_40BD_A59C_C06B93A64FEF__INCLUDED_)
