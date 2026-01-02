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


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "common.h"
#include "sdfLib.h"

using namespace SDFLibrary;

// Thread-local context for backward compatibility with global API
static thread_local SDFContext* g_context = nullptr;

// Legacy function - no longer needed with SDFContext and unique_ptr
// void free_memory()
// {
// 	int i, j, k;
// 	SDFLibrary::listnode* temp;
// 	SDFLibrary::listnode* currNode;

// 	printf("starting memory de-allocation\n");

// 	//1. Octree
// 	for (i = 0; i < SDFLibrary::size; i++)
// 	{
// 		for (j = 0; j < SDFLibrary::size; j++)
// 		{
// 			for (k = 0; k < SDFLibrary::size; k++)
// 			{
// 				currNode = SDFLibrary::sdf[i][j][k].tindex;

// 				while(currNode != NULL)
// 				{
// 					temp = currNode;
// 					currNode = currNode->next;
// 					free(temp);
// 				}
// 			}
// 			free(SDFLibrary::sdf[i][j]);
// 		}
// 		free(SDFLibrary::sdf[i]);
// 	}	
// 	free(SDFLibrary::sdf);

// 	free(SDFLibrary::values);

// 	// Use delete[] instead of free for myVert to properly destruct std::vector members
// 	if (SDFLibrary::vertices != NULL)
// 		delete[] SDFLibrary::vertices;

// 	if (SDFLibrary::surface != NULL)
// 		free(SDFLibrary::surface);

// 	if (SDFLibrary::normals != NULL)
// 		free(SDFLibrary::normals);

// 	if (SDFLibrary::distances != NULL)
// 		free(SDFLibrary::distances);

// 	if (SDFLibrary::queues != NULL)
// 		free(SDFLibrary::queues);

// 	if (SDFLibrary::bverts != NULL)
// 		free(SDFLibrary::bverts);

// 	printf("Memory de-allocated successfully! \n");
// }

// Legacy function - no longer needed with SDFContext
// void SDFLibrary::setParameters(int Size, int isNormalFlip, float* mins, float* maxs)
// {
// 	//First the default values.
// 	SDFLibrary::init_all_vars();
// 	
// 	//Then, assign the actual input values.
// 	SDFLibrary::size = Size;
// 	SDFLibrary::flipNormals = isNormalFlip;

// 	SDFLibrary::minext[0] = mins[0];	SDFLibrary::minext[1] = mins[1];	SDFLibrary::minext[2] = mins[2];
// 	SDFLibrary::maxext[0] = maxs[0];	SDFLibrary::maxext[1] = maxs[1];	SDFLibrary::maxext[2] = maxs[2];
// 	SDFLibrary::span[0] = (maxs[0]-mins[0])/(SDFLibrary::size);
// 	SDFLibrary::span[1] = (maxs[1]-mins[1])/(SDFLibrary::size);
// 	SDFLibrary::span[2] = (maxs[2]-mins[2])/(SDFLibrary::size);

// 	if ((Size!=16) && (Size!=32) &&(Size!=64) && (Size!=128) && (Size!=256) &&(Size!=512) &&(Size!=1024))
// 	{
// 		printf("size is incorrect\n");
// 		exit(1);
// 	}
// }

// Legacy computeSDF removed - use computeSDF_MT instead
/*
float* SDFLibrary::computeSDF(int nverts, float* verts, int ntris, int* tris)
{
	... removed ...
}
*/

// Legacy getVolumeInfo removed - information is now in SDFContext
/*
RAWIV_header* SDFLibrary::getVolumeInfo()
{
	... removed ...
}
*/

// ============================================================================
// NEW THREAD-SAFE API IMPLEMENTATION
// ============================================================================

std::unique_ptr<SDFContext> SDFLibrary::createContext()
{
	return std::make_unique<SDFContext>();
}

std::unique_ptr<float[]> SDFLibrary::computeSDF_MT(
	int nverts, const float* verts,
	int ntris, const int* tris,
	int grid_size, int isNormalFlip,
	const float* mins, const float* maxs)
{
	// Create a new context for this computation
	auto ctx = createContext();
	
	// Configure parameters
	ctx->setParameters(grid_size, isNormalFlip, mins, maxs);
	
	// Initialize
	if (!ctx->initSDF()) {
		fprintf(stderr, "SDFLibrary::computeSDF_MT: initSDF() failed\n");
		return nullptr;
	}
	
	// Read geometry
	ctx->readGeom(nverts, verts, ntris, tris);
	
	// Adjust data and build octree
	ctx->adjustData();
	
	// Compute SDF
	ctx->compute();
	
	// Extract and return results
	return ctx->releaseValues();
}
