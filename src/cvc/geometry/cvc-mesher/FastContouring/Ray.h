/*
  Copyright 2002-2003 The University of Texas at Austin
  
    Authors: Anthony Thane <thanea@ices.utexas.edu>
             Vinay Siddavanahalli <skvinay@cs.utexas.edu>
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

// Ray.h: interface for the Ray class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_RAY_H__8760CFFE_A656_460F_A893_4C81FF806264__INCLUDED_)
#define AFX_RAY_H__8760CFFE_A656_460F_A893_4C81FF806264__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Vector.h"

namespace FastContouring
{

class Ray  
{
public:
	Ray();
	Ray(const Vector& origin, const Vector& dir);
	virtual ~Ray();

	Vector getPointOnRay(float t) const;

	float nearestTOnXAxis(Vector Origin = Vector(0.0, 0.0, 0.0, 1.0)) const;
	float nearestTOnYAxis(Vector Origin = Vector(0.0, 0.0, 0.0, 1.0)) const;
	float nearestTOnZAxis(Vector Origin = Vector(0.0, 0.0, 0.0, 1.0)) const;

	Vector nearestPointOnXAxis(Vector Origin = Vector(0.0, 0.0, 0.0, 1.0)) const;
	Vector nearestPointOnYAxis(Vector Origin = Vector(0.0, 0.0, 0.0, 1.0)) const;
	Vector nearestPointOnZAxis(Vector Origin = Vector(0.0, 0.0, 0.0, 1.0)) const;

	float distanceToXAxis(Vector Origin = Vector(0.0, 0.0, 0.0, 1.0)) const;
	float distanceToYAxis(Vector Origin = Vector(0.0, 0.0, 0.0, 1.0)) const;
	float distanceToZAxis(Vector Origin = Vector(0.0, 0.0, 0.0, 1.0)) const;

	Vector m_Origin;
	Vector m_Dir;

};

}

#endif // !defined(AFX_RAY_H__8760CFFE_A656_460F_A893_4C81FF806264__INCLUDED_)
