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

// RawnFile.h: interface for the RawnFile class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RAWNFILE_H__693A5AE3_831C_4120_A062_74EB8F04875E__INCLUDED_)
#define AFX_RAWNFILE_H__693A5AE3_831C_4120_A062_74EB8F04875E__INCLUDED_

#include "GeometryFileType.h"

class RawnFile : public GeometryFileType  
{
public:
	virtual ~RawnFile();

	virtual Geometry* loadFile(const string& fileName);
	virtual bool checkType(const string& fileName);
	virtual bool saveFile(const Geometry* geometry, const string& fileName);

	virtual string extension() { return "rawn"; };
	virtual string filter() { return "Rawn files (*.rawn)"; };

	static RawnFile ms_RawnFileRepresentative;
	static GeometryFileType* getRepresentative();

protected:
	RawnFile();

};

#endif // !defined(AFX_RAWNFILE_H__693A5AE3_831C_4120_A062_74EB8F04875E__INCLUDED_)
