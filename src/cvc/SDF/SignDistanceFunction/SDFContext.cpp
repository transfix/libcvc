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
/*     Refactored: Joe Rivera                                     2025       */
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

#include "SDFContext.h"
#include "common.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
#include <sys/types.h>
#include <sys/timeb.h>
#else
#include <sys/time.h>
#endif

// Old update_bounding_box removed - used global sdf

namespace SDFLibrary {

// Constructor
SDFContext::SDFContext()
    : MAX_DIST(0.0)
    , size(64)
    , total_points(0)
    , total_triangles(0)
    , all_verts_touched(0)
    , minx(0.0), miny(0.0), minz(0.0)
    , maxx(0.0), maxy(0.0), maxz(0.0)
    , octree_depth(0)
    , flipNormals(0)
{
    minext[0] = minext[1] = minext[2] = 10000.0;
    maxext[0] = maxext[1] = maxext[2] = -10000.0;
    span[0] = span[1] = span[2] = 1.0;
}

// Destructor
SDFContext::~SDFContext() {
    // boost::multi_array and unique_ptr automatically clean up all memory
    // No manual deletion needed - RAII handles everything
}

void SDFContext::setParameters(int grid_size, int isNormalFlip, 
                                const float* mins, const float* maxs) {
    size = grid_size;
    flipNormals = isNormalFlip;
    
    minext[0] = mins[0];
    minext[1] = mins[1];
    minext[2] = mins[2];
    maxext[0] = maxs[0];
    maxext[1] = maxs[1];
    maxext[2] = maxs[2];
    
    span[0] = (maxs[0] - mins[0]) / size;
    span[1] = (maxs[1] - mins[1]) / size;
    span[2] = (maxs[2] - mins[2]) / size;
    
    if ((size != 16) && (size != 32) && (size != 64) && 
        (size != 128) && (size != 256) && (size != 512) && (size != 1024)) {
        fprintf(stderr, "Warning: size %d is not a power of 2 between 16 and 1024\n", size);
    }
}

void SDFContext::object2octree(double xmin, double ymin, double zmin, 
                               double xmax, double ymax, double zmax, 
                               int &ci, int &cj, int &ck) const {
    int i, j, k;
    
    ci = static_cast<int>((xmin - minext[0]) / span[0]);
    i = static_cast<int>((xmax - minext[0]) / span[0]);
    ci = (i + ci) / 2;
    
    cj = static_cast<int>((ymin - minext[1]) / span[1]);
    j = static_cast<int>((ymax - minext[1]) / span[1]);
    cj = (j + cj) / 2;
    
    ck = static_cast<int>((zmin - minext[2]) / span[2]);
    k = static_cast<int>((zmax - minext[2]) / span[2]);
    ck = (k + ck) / 2;
    
    if ((i != ci + 1) || (j != cj + 1) || (k != ck + 1)) {
        printf("Warning: cannot make a good Octree\n");
    }
}

bool SDFContext::isZero(const myPoint& one) {
    double val = std::sqrt(one.x * one.x + one.y * one.y + one.z * one.z);
    return isZero(val);
}

bool SDFContext::isSame(const myPoint& one, const myPoint& two) {
    double val = std::sqrt((one.x - two.x) * (one.x - two.x) + 
                          (one.y - two.y) * (one.y - two.y) + 
                          (one.z - two.z) * (one.z - two.z));
    return isZero(val);
}

// Forward declarations for helper functions
static void check_bounds_ctx(SDFContext* ctx, int i);
static bool setOctree_depth_ctx(SDFContext* ctx);
static void reverse_ptrs_ctx(SDFContext* ctx);
static void compute_ctx(SDFContext* ctx);
void start_fireworks(SDFContext* ctx);

// External functions from octree.cpp
extern void update_bounding_box_ctx(SDFContext* ctx, long int current_triangle,
                             double xmin, double xmax, double ymin, double ymax,
                             double zmin, double zmax, int cur_level);



bool SDFContext::initSDF() {
    int i, j, k;
    
    MAX_DIST = size * std::sqrt(3.0);
    minx = miny = minz = 10000.0;
    maxx = maxy = maxz = -10000.0;
    
    if (!setOctree_depth_ctx(this)) {
        return false;
    }
    
    // Allocate octree using boost::multi_array (RAII-compliant)
    sdf.resize(boost::extents[size][size][size]);
    
    // Initialize all cells
    for (i = 0; i < size; i++) {
        for (j = 0; j < size; j++) {
            for (k = 0; k < size; k++) {
                sdf[i][j][k].useful = 0;
                sdf[i][j][k].type = 0;
                sdf[i][j][k].no = 0;
                sdf[i][j][k].tindex.clear();
            }
        }
    }
    
    // Allocate voxel values and working arrays
    k = (size + 1) * (size + 1) * (size + 1);
    voxel_values = std::make_unique<voxel[]>(k);
    bverts = std::make_unique<bool[]>(k);
    queues = std::make_unique<int[]>(k);
    
    for (i = 0; i < k; i++) {
        voxel_values[i].value = static_cast<float>(MAX_DIST);
        voxel_values[i].signe = 0;
        voxel_values[i].processed = false;
        voxel_values[i].closestV = 0;
        bverts[i] = false;
    }
    
    return true;
}

void SDFContext::readGeom(int nverts, const float* verts, int ntris, const int* tris) {
    int i;
    int maxInd = -1;
    
    total_points = nverts;
    total_triangles = ntris;
    
    printf("vert= %d and tri = %d \n", total_points, total_triangles);
    
    // Use smart pointers for automatic cleanup
    vertices = std::make_unique<myVert[]>(total_points);
    surface = std::make_unique<triangle[]>(total_triangles);
    normals = std::make_unique<myPoint[]>(total_triangles);
    distances = std::make_unique<double[]>(total_triangles);
    
    for (i = 0; i < total_points; i++) {
        vertices[i].x = verts[3 * i + 0];
        vertices[i].y = verts[3 * i + 1];
        vertices[i].z = verts[3 * i + 2];
        check_bounds_ctx(this, i);
        vertices[i].isNull = 0;
        vertices[i].trisUsed = 0;
        
        if (!(i % 5000))
            printf("still working on points !!!! %d \n", i);
    }
    
    printf("Finished reading the Vertices.. Now reading the Triangles\n");
    
    for (i = 0; i < total_triangles; i++) {
        surface[i].v1 = tris[3 * i + 0];
        surface[i].v2 = tris[3 * i + 1];
        surface[i].v3 = tris[3 * i + 2];
        
        if (maxInd < surface[i].v1) maxInd = surface[i].v1;
        if (maxInd < surface[i].v2) maxInd = surface[i].v2;
        if (maxInd < surface[i].v3) maxInd = surface[i].v3;
        
        if (!(i % 5000))
            printf("still working on Triangles !!!! %d \n", i);
    }
    
    printf("Bounding box is: %f %f %f to %f %f %f \n", minx, miny, minz, maxx, maxy, maxz);
}

void SDFContext::adjustData() {
    // Recalculate span
    span[0] = (maxext[0] - minext[0]) / size;
    span[1] = (maxext[1] - minext[1]) / size;
    span[2] = (maxext[2] - minext[2]) / size;
    
    printf("\n\nSurface Bounding box is: %f %f %f to %f %f %f \n", 
           minx, miny, minz, maxx, maxy, maxz);
    printf("\nVolume Bounding box is %f %f %f to %f %f %f \n", 
           minext[0], minext[1], minext[2], maxext[0], maxext[1], maxext[2]);
    
    // Calculate normals and back-pointers
    reverse_ptrs_ctx(this);
    
    // Flip normals if needed
    if (flipNormals)
        SDFLibrary::start_fireworks(this);
    
    // Build octree
    build_octree();
}

void SDFContext::build_octree() {
    double t1, t2;
    
    t1 = getTime();
    
    for (int i = 0; i < total_triangles; i++) {
        update_bounding_box_ctx(this, (long)i, minext[0], maxext[0], 
                               minext[1], maxext[1], minext[2], maxext[2], 0);
        if (i % 1000 == 0)
            printf("%d processed in octree\n", i);
    }
    
    t2 = getTime();
    printf("Octree constructed for the data in %f seconds\n", (t2 - t1));
}

// ... other code ...

void SDFContext::process_triangle(int i) {
    double p1x, p1y, p1z, p2x, p2y, p2z;
    double nx, ny, nz;
    double denom;
    int v1, v2, v3;
    
    v1 = surface[i].v1;
    v2 = surface[i].v2;
    v3 = surface[i].v3;
    
    // Calculate normal using cross product
    p1x = vertices[v3].x - vertices[v2].x;
    p1y = vertices[v3].y - vertices[v2].y;
    p1z = vertices[v3].z - vertices[v2].z;
    p2x = vertices[v1].x - vertices[v2].x;
    p2y = vertices[v1].y - vertices[v2].y;
    p2z = vertices[v1].z - vertices[v2].z;
    
    nx = ((p1y * p2z) - (p1z * p2y));
    ny = ((p1z * p2x) - (p1x * p2z));
    nz = ((p1x * p2y) - (p1y * p2x));
    
    denom = std::sqrt(nx * nx + ny * ny + nz * nz);
    
    nx /= denom;
    ny /= denom;
    nz /= denom;
    
    normals[i].x = nx;
    normals[i].y = ny;
    normals[i].z = nz;
    
    // Calculate distance from origin
    distances[i] = -1 * ((nx * vertices[v1].x) + (ny * vertices[v1].y) + (nz * vertices[v1].z));
    surface[i].type = -1;
}

// Helper function implementations
static void check_bounds_ctx(SDFContext* ctx, int i) {
    if (ctx->vertices[i].x < ctx->minx) ctx->minx = ctx->vertices[i].x;
    if (ctx->vertices[i].y < ctx->miny) ctx->miny = ctx->vertices[i].y;
    if (ctx->vertices[i].z < ctx->minz) ctx->minz = ctx->vertices[i].z;
    
    if (ctx->vertices[i].x > ctx->maxx) ctx->maxx = ctx->vertices[i].x;
    if (ctx->vertices[i].y > ctx->maxy) ctx->maxy = ctx->vertices[i].y;
    if (ctx->vertices[i].z > ctx->maxz) ctx->maxz = ctx->vertices[i].z;
}

static bool setOctree_depth_ctx(SDFContext* ctx) {
    switch (ctx->size) {
    case 16:  ctx->octree_depth = 4; break;
    case 32:  ctx->octree_depth = 5; break;
    case 64:  ctx->octree_depth = 6; break;
    case 128: ctx->octree_depth = 7; break;
    case 256: ctx->octree_depth = 8; break;
    case 512: ctx->octree_depth = 9; break;
    case 1024: ctx->octree_depth = 10; break;
    default:
        printf("This version can only deal with Volumes of sizes 16, 32, 64, 128, 256, 512 or 1024\n");
        return false;
    }
    return true;
}

static void reverse_ptrs_ctx(SDFContext* ctx) {
    for (int i = 0; i < ctx->total_triangles; i++) {
        ctx->process_triangle(i);
        
        ctx->vertices[ctx->surface[i].v1].tris.push_back(i);
        ctx->vertices[ctx->surface[i].v1].trisUsed++;
        ctx->vertices[ctx->surface[i].v2].tris.push_back(i);
        ctx->vertices[ctx->surface[i].v2].trisUsed++;
        ctx->vertices[ctx->surface[i].v3].tris.push_back(i);
        ctx->vertices[ctx->surface[i].v3].trisUsed++;
    }
}

// Forward declarations for refactored functions (implemented in compute.cpp, propagate.cpp)
// update_bounding_box_ctx is declared earlier
extern void compute_signs(SDFContext* ctx);
extern void compute_boundarySDF(SDFContext* ctx);

static void compute_ctx(SDFContext* ctx) {
    double t1, t2;
    int grid_pts;
    int i, j, k, m, ind, prevMin, prevVerts;
    
    // Compute signs
    t1 = getTime();
    compute_signs(ctx);
    t2 = getTime();
    printf("Sign computations done in %f seconds\n", (t2-t1));
    
    // Compute boundary SDF
    t1 = getTime();
    compute_boundarySDF(ctx);	
    t2 = getTime();
    printf("Function evaluated at the %d boundary vertices in %f seconds\n", 
           ctx->all_verts_touched, (t2-t1));

    // Propagate distances using iterative approach
    grid_pts = (ctx->size+1)*(ctx->size+1)*(ctx->size+1);
    printf("total grid points: %d and starting with %d points\n", 
           grid_pts, ctx->all_verts_touched);

    m = 0;
    prevMin = 0;
    prevVerts = ctx->all_verts_touched;
    t1 = getTime();

    do {
        // For each voxel in the boundary_verts queue
        for (ind = prevMin; ind < prevVerts; ind++) {
            ctx->vert2index(ctx->queues[ind], i, j, k);

            if ((prevMin != 0) && (ctx->voxel_values[ctx->queues[ind]].processed == 1))
                continue;

            ctx->apply_distance_transform(i, j, k);

            ctx->voxel_values[ctx->queues[ind]].processed = 1;
            if (ind % 10000 == 0)
                printf("iter#%d: %d processed\n", m, ind);
        }

        prevMin = prevVerts;
        prevVerts = ctx->all_verts_touched;
        m++;
        printf("in Iteration# %d, with %d vertices in the queue\n", m, prevVerts);

        if (prevMin == prevVerts) {
            printf("SDF propagation saturated.\n");
            break;
        }

    } while (ctx->all_verts_touched != grid_pts);

    t2 = getTime();
    printf("Distance Propagation for %d grid points done in %f seconds\n", 
           ctx->all_verts_touched, (t2-t1));
}

// Forward declaration for helper function (implemented in propagate.cpp)
extern void update_distance_2_vertex(SDFContext* ctx, int ind, int vi, int vj, int vk);

void SDFContext::apply_distance_transform(int vi, int vj, int vk)
{
	int ind;  // Current vertex

	ind = index2vert(vi, vj, vk);

	// Front Y slice
	update_distance_2_vertex(this, ind, vi-1, vj-1, vk-1);
	update_distance_2_vertex(this, ind, vi,   vj-1, vk-1);
	update_distance_2_vertex(this, ind, vi+1, vj-1, vk-1);
	
	update_distance_2_vertex(this, ind, vi-1, vj-1, vk);
	update_distance_2_vertex(this, ind, vi,   vj-1, vk);
	update_distance_2_vertex(this, ind, vi+1, vj-1, vk);

	update_distance_2_vertex(this, ind, vi-1, vj-1, vk+1);
	update_distance_2_vertex(this, ind, vi,   vj-1, vk+1);
	update_distance_2_vertex(this, ind, vi+1, vj-1, vk+1);

	// Middle Y slice
	update_distance_2_vertex(this, ind, vi-1, vj,	 vk-1);
	update_distance_2_vertex(this, ind, vi,   vj,	 vk-1);
	update_distance_2_vertex(this, ind, vi+1, vj,	 vk-1);
	
	update_distance_2_vertex(this, ind, vi-1, vj,	 vk);
  // update_distance_2_vertex(this, ind, vi,   vj,	 vk); // Current vertex
	update_distance_2_vertex(this, ind, vi+1, vj,	 vk);

	update_distance_2_vertex(this, ind, vi-1, vj,	 vk+1);
	update_distance_2_vertex(this, ind, vi,   vj,	 vk+1);
	update_distance_2_vertex(this, ind, vi+1, vj,	 vk+1);

	// Back Y slice
	update_distance_2_vertex(this, ind, vi-1, vj+1, vk-1);
	update_distance_2_vertex(this, ind, vi,   vj+1, vk-1);
	update_distance_2_vertex(this, ind, vi+1, vj+1, vk-1);
	
	update_distance_2_vertex(this, ind, vi-1, vj+1, vk);
	update_distance_2_vertex(this, ind, vi,   vj+1, vk);
	update_distance_2_vertex(this, ind, vi+1, vj+1, vk);

	update_distance_2_vertex(this, ind, vi-1, vj+1, vk+1);
	update_distance_2_vertex(this, ind, vi,   vj+1, vk+1);
	update_distance_2_vertex(this, ind, vi+1, vj+1, vk+1);
}

void SDFContext::insert_bound_vert(int vert)
{
	if(bverts[vert] == 0) // ie not found
	{
		bverts[vert] = 1;
		queues[all_verts_touched++] = vert;
	}
}

void SDFContext::compute() {
    // Forward to external compute implementation (to be refactored)
    compute_ctx(this);
    
    // Copy results to output buffer
    int numb = (size + 1) * (size + 1) * (size + 1);
    values = std::make_unique<float[]>(numb);
    
    for (int i = 0; i < numb; i++) {
        values[i] = voxel_values[i].value * voxel_values[i].signe;
    }
}

} // namespace SDFLibrary

