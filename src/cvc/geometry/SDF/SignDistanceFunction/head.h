/*****************************************************************************/
/*                             ______________________                        */
/*                            / _ _ _ _ _ _ _ _ _ _ _)                       */
/*            ____  ____  _  / /__  __  _____  __                            */
/*           (_  _)( ___)( \/ /(  \/  )(  _  )(  )                           */
/*             )(   )__)  )  (  )    (  )(_)(  )(__                          */
/*            (__) (____)/ /\_)(_/\/\_)(_____)(____)                         */
/*            _ _ _ _ __/ /                                                  */
/*           (___________/                     ___  ___                      */
/*                                      \  )| |   ) _ _|\   )                */
/*                                 ---   \/ | |  / |___| \_/                 */
/*                                                       _/                  */
/*                                                                           */
/*   Copyright (C) The University of Texas at Austin                         */
/*                                                                           */
/*     Author:     Lalit Karlapalem <ckl@ices.utexas.edu>         2004-2005  */
/*                                                                           */
/*     Principal Investigator: Chandrajit Bajaj <bajaj@ices.utexas.edu>      */
/*                                                                           */
/*         Professor of Computer Sciences,                                   */
/*         Computational and Applied Mathematics Chair in Visualization,     */
/*         Director, Computational Visualization Center (CVC),               */
/*         Institute of Computational Engineering and Sciences (ICES)        */
/*         The University of Texas at Austin,                                */
/*         201 East 24th Street, ACES 2.324A,                                */
/*         1 University Station, C0200                                       */
/*         Austin, TX 78712-0027                                             */
/*         http://www.cs.utexas.edu/~bajaj                                   */
/*                                                                           */
/*         http://www.ices.utexas.edu/CVC                                    */
/*  This software comes with a license. Using this code implies that you     */
/*  read, understood and agreed to all the terms and conditions in that      */
/*  license.                                                                 */
/*                                                                           */
/*  We request that you agree to acknowledge the use of the software that    */
/*  results in any published work, including scientific papers, films and    */
/*  videotapes by citing the reference listed below                          */
/*                                                                           */
/*    C. Bajaj, P. Djeu, V. Siddavanahalli, A. Thane,                        */
/*    Interactive Visual Exploration of Large Flexible Multi-component       */
/*    Molecular Complexes,                                                   */
/*    Proc. of the Annual IEEE Visualization Conference, October 2004,       */
/*    Austin, Texas, IEEE Computer Society Press, pp. 243-250.               */
/*                                                                           */
/*****************************************************************************/


#ifndef CCV_SDF_HEAD_H
#define CCV_SDF_HEAD_H

#include <vector>
#include <memory>

namespace SDFLibrary {

	#define MAX_TRIS_PER_VERT 100
	
	// Point structure for 3D coordinates
	struct myPoint {
		double x;
		double y;
		double z;
		char isNull;
		
		myPoint() : x(0.0), y(0.0), z(0.0), isNull(0) {}
		myPoint(double x_, double y_, double z_) 
			: x(x_), y(y_), z(z_), isNull(0) {}
	};

	// Vertex structure with associated triangles
	struct myVert {
		double x;
		double y;
		double z;
		char isNull;

		std::vector<int> tris;  // Triangles sharing this vertex
		int trisUsed;           // Number of elements used

		myVert() : x(0.0), y(0.0), z(0.0), isNull(0), trisUsed(0) {}
		myVert(double x_, double y_, double z_) 
			: x(x_), y(y_), z(z_), isNull(0), trisUsed(0) {}
	};


	// Triangle structure
	struct triangle {
		int v1;
		int v2;
		int v3;
		int type; // default = -1; done = 1; wrong = 3;
		
		triangle() : v1(0), v2(0), v3(0), type(-1) {}
		triangle(int v1_, int v2_, int v3_) 
			: v1(v1_), v2(v2_), v3(v3_), type(-1) {}
	};

	// Octree cell structure - stores list of triangle indices
	struct cell {
		char useful;        // 0 - no triangles; 1 - has triangles
		char type;          // 0 - interior node; 1 - leaf node with triangles
		long int no;
		std::vector<int> tindex;   // Triangle indices for this cell
		
		cell() : useful(0), type(0), no(0) {}
		~cell() = default;
		
		// Default copy/move operations work correctly with vector
		cell(const cell& other) = default;
		cell& operator=(const cell& other) = default;
		cell(cell&& other) noexcept = default;
		cell& operator=(cell&& other) noexcept = default;
	};

	// Ray structure for ray tracing
	struct ray {
		double ox;
		double oy;
		double oz;
		double dx; 
		double dy;
		double dz;
		
		ray() : ox(0.0), oy(0.0), oz(0.0), dx(0.0), dy(0.0), dz(0.0) {}
	};

	// Voxel structure for distance field
	struct voxel {
		float value;
		signed char signe;  // -1 = inside, 1 = outside
		bool processed;     // true = propagated distance FROM here
		int closestV;       // Closest triangle on the surface
		
		voxel() : value(0.0f), signe(1), processed(false), closestV(-1) {}
	};

	
}; //namespace SDFLibrary

#endif
