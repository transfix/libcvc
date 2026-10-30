/*
  Copyright (c): Xiaoyu Zhang (xiaoyu@csusm.edu)

  This file is part of sdf (signed distance function).

  sdf is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  sdf is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
#pragma once

#include "FaceVertSet3D.h"
#include "reg3data.h"
#include "RawivParser.h"

#include <cstddef>
#include <vector>

// Forward declaration for app context
namespace cvc { class app; }

class Cell {
public:
	Cell() {
		
	}
	~Cell() {
		
	}
	
	bool hasTriangle() const {
		return triList.length() > 0;
	}

	dynamic_array<int> triList;
};

class DistanceTransform
{
	cvc::app& _ctx;
	Reg3Data<float> *p_Data;
	Cell*	p_Cells;
	FaceVertSet3D* p_Surf;
public:
	// Constructors with explicit app context
	// sx, sy, sz: scale factor of the bounding volume
	DistanceTransform(cvc::app& ctx, FaceVertSet3D& fvs, int dim[3], float Distance = 20, float sx=2, float sy=2, float sz=2);

	// Constructor with user-specified center (allows arbitrary bounding boxes)
	// center: center point of the grid [cx, cy, cz]
	// sx, sy, sz: scale factors to control grid extent
	DistanceTransform(cvc::app& ctx, FaceVertSet3D& fvs, int dim[3], const float center[3], float Distance = 20, float sx=2, float sy=2, float sz=2);

	DistanceTransform(cvc::app& ctx, FaceVertSet3D& fvs, const Reg3Data<float>& reg3);

	// A slice at a time: made with `deferred`, the constructor does none of init()'s work, and
	// step(budget) does init() and then transform() in about `budget` units (a cell a triangle
	// touches, a voxel, a voxel of a 1-D line), returning true once the distances are done. The
	// loops are the same as init()'s and transform()'s, so the result is too.
	struct deferred_t {};
	DistanceTransform(cvc::app& ctx, FaceVertSet3D& fvs, int dim[3], const float center[3],
					  float Distance, float sx, float sy, float sz, deferred_t);
	bool step(std::size_t budget);
	double progress() const;
	// "cells", "near surface", "x lines", "y lines", "z lines", "near distances", "far distances",
	// then "done".
	const char* phase() const;

	~DistanceTransform(void);

	void transform();

	void writeRawiv(const char* fname) {
		RawivParser parser;
		parser.write(*p_Data, fname);
	}

	//////////////////////////////////////////////////////////////////////////
	void transform1D(int len, float f[], float d[], int parent[], float span);
//private:
	const static float MAX_FLOAT;
	///
	bool intersectCell(const Point3f& v0, const Vector3f& norm, int i, int j, int k);

	//////////////////////////////////////////////////////////////////////////
	bool nearSurface(int i, int j, int k);

	//////////////////////////////////////////////////////////////////////////
	float computeNearDistance(int ix, int iy, int iz, Point3f& pnt);

	//////////////////////////////////////////////////////////////////////////
	double distance2Triangle(const Point3f& pnt, int nt, Point3f& ne);

	//////////////////////////////////////////////////////////////////////////
	double distance2Edge(const Point3f& pnt, const Point3f& v1, const Point3f& v2, Point3f& ne);

	//////////////////////////////////////////////////////////////////////////
	bool pointInTriangle(const Point3f& res, const Point3f& v0, const Point3f& v1, 
						 const Point3f& v2, const Vector3f& norm);

	//////////////////////////////////////////////////////////////////////////
	int nearestPlane(const dynamic_array<int>& triList, const Point3f& vert);

	///
	double rayTriangleIntersection(int nt, const Point3f& begin, const Point3f& end);

	/**
	 * @return -1, if point is inside, return 1 if point is outside
	 */
	int inOrOut(const dynamic_array<int>& triList, const Point3f& vert, const Point3f& nearPnt);

	/************************************************************************/
	/* @return 1 if they intersect                                          */
	/************************************************************************/
	bool TriangleCubeIntersection(int nt, const Point3f& lower, const Point3f& upper);

	const Reg3Data<float> & getReg3Data() {
		return *p_Data;
	}
private:
	void init();
	void setGrid(int dim[3], const float center[3], float sx, float sy, float sz);
	// Run the stages up to (not including) `until`, at most about `budget` units; true once there.
	bool advance(std::size_t budget, int until);

	int _stage = 0;
	std::size_t _pos = 0;
	std::vector<int> _parent;	// the 1-D passes' nearest near-surface voxel of each voxel
	std::vector<Point3f> _closest;	// a near-surface voxel's nearest surface point
	std::vector<float> _f, _d;
	std::vector<int> _p;
};
