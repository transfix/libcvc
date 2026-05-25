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


#ifndef CCV_SDF_SDFLIB_H
#define CCV_SDF_SDFLIB_H

#include <memory>
#include "SDFContext.h"

namespace SDFLibrary {

// RAWIV header structure for volume information
typedef struct RAWIV_header
{
	float minext[3];	// Co-ords of the first voxel
	float maxext[3];	// Co-ords of the last voxel
	float origin[3];	// Co-ords of the first voxel a.k.a. Origin
	float span[3];		// Span between grid points
	int dim[3];			// Number of grid points

	int ngridpts;		// Total grid points
	int ncells;			// Total cells
	int size;			// Octree size

} RAWIV_header;

// ============================================================================
// NEW THREAD-SAFE API (Recommended for new code)
// ============================================================================

/**
 * Compute SDF using a dedicated context (thread-safe)
 * 
 * This is the recommended way to compute SDF as it allows multiple
 * computations to run in parallel without interfering with each other.
 * 
 * @param nverts Number of vertices in the mesh
 * @param verts  Vertex array (3*nverts floats: x1,y1,z1, x2,y2,z2, ...)
 * @param ntris  Number of triangles
 * @param tris   Triangle index array (3*ntris ints: v1,v2,v3, ...)
 * @param size   Grid size (must be power of 2: 16, 32, 64, 128, 256, 512, 1024)
 * @param isNormalFlip Whether to flip normals
 * @param mins   Bounding box minimum (3 floats: minx, miny, minz)
 * @param maxs   Bounding box maximum (3 floats: maxx, maxy, maxz)
 * @return       Unique pointer to SDF values array, or nullptr on failure
 */
std::unique_ptr<float[]> computeSDF_MT(int nverts, const float* verts, 
                                        int ntris, const int* tris,
                                        int size, int isNormalFlip,
                                        const float* mins, const float* maxs);

/**
 * Create a new SDF computation context
 * 
 * Use this for manual control over the SDF computation pipeline.
 * The context can be reused for multiple computations.
 * 
 * @return A unique pointer to a new SDFContext
 */
std::unique_ptr<SDFContext> createContext();

// ============================================================================
// LEGACY API (Deprecated - not thread-safe, uses global state)
// ============================================================================

// Set parameters of the SDF grid (not thread-safe)
void setParameters(int size, int isNormalFlip, float* mins, float* maxs);

// Compute SDF using global state (not thread-safe)
// Returns raw pointer that caller must delete[]
float* computeSDF(int nverts, float* verts, int ntris, int* tris);

// Get volume information (not thread-safe)
RAWIV_header* getVolumeInfo();

}; // namespace SDFLibrary

#endif