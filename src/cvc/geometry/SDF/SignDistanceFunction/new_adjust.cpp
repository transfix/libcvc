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
/*     Refactored: SDFContext support                              2025      */
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

#ifdef _MSC_VER
#pragma warning(disable:4786)
#endif

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <map>

#include "common.h"
#include "SDFContext.h"

// Triangle orientation adjustment functions - refactored for SDFContext

#include "SDFContext.h"

using namespace SDFLibrary;

namespace {
    // Helper function: Check if two vertex positions in a triangle follow the expected order (1->2, 2->3, 3->1)
    inline bool isAligned(int ver1, int ver2) {
        if (ver1 == 1) return (ver2 == 2);
        if (ver1 == 2) return (ver2 == 3);
        if (ver1 == 3) return (ver2 == 1);
        return false;
    }
    
    // Exchange two vertices in a triangle to flip its orientation
    void exchangeVerts(SDFContext* ctx, int tri, int ver1, int ver2) {
        if (ctx->surface[tri].v1 == ver1) {
            ctx->surface[tri].v1 = ver2;
            if (ctx->surface[tri].v2 == ver2)
                ctx->surface[tri].v2 = ver1;
            else
                ctx->surface[tri].v3 = ver1;
        }
        else if (ctx->surface[tri].v2 == ver1) {
            ctx->surface[tri].v2 = ver2;
            if (ctx->surface[tri].v1 == ver2)
                ctx->surface[tri].v1 = ver1;
            else
                ctx->surface[tri].v3 = ver1;
        }
        else if (ctx->surface[tri].v3 == ver1) {
            ctx->surface[tri].v3 = ver2;
            if (ctx->surface[tri].v1 == ver2)
                ctx->surface[tri].v1 = ver1;
            else
                ctx->surface[tri].v2 = ver1;
        }
    }
    
    // Compare triangle vertex ordering and exchange if needed
    bool triangle_angles(SDFContext* ctx, int one, int two, int ver1, int ver2) {
        int v1 = -1, v2 = -1, c1 = -1, c2 = -1;
        
        // Find positions of ver1 and ver2 in triangle 'one'
        if (ctx->surface[one].v1 == ver1) v1 = 1;
        if (ctx->surface[one].v1 == ver2) v2 = 1;
        if (ctx->surface[one].v2 == ver1) v1 = 2;
        if (ctx->surface[one].v2 == ver2) v2 = 2;
        if (ctx->surface[one].v3 == ver1) v1 = 3;
        if (ctx->surface[one].v3 == ver2) v2 = 3;
        
        // Find positions of ver1 and ver2 in triangle 'two'
        if (ctx->surface[two].v1 == ver1) c1 = 1;
        if (ctx->surface[two].v1 == ver2) c2 = 1;
        if (ctx->surface[two].v2 == ver1) c1 = 2;
        if (ctx->surface[two].v2 == ver2) c2 = 2;
        if (ctx->surface[two].v3 == ver1) c1 = 3;
        if (ctx->surface[two].v3 == ver2) c2 = 3;
        
        if ((v1 == -1) || (v2 == -1) || (c1 == -1) || (c2 == -1)) {
            printf("Error in triangle_angles: %d %d %d %d\n", one, two, ver1, ver2);
            return true;
        }
        
        // Check if orientations match
        bool one_aligned = isAligned(v1, v2);
        bool two_aligned = isAligned(c1, c2);
        
        if (one_aligned == two_aligned) {
            // Misaligned - need to flip
            exchangeVerts(ctx, two, ver1, ver2);
            return false;
        }
        // Properly aligned
        return true;
    }
}


// Main triangle orientation adjustment functions

namespace SDFLibrary {

// Re-orient all normals to point consistently (all outward or all inward)
void re_orient_all(SDFContext* ctx) {
    int i, closestTri;
    int inside_point = -1;
    double err, dist;
    
    // Find the vertex closest to the bounding box minimum corner
    err = ctx->size * ctx->size * ctx->size;
    
    for (i = 0; i < ctx->total_points; i++) {
        dist = 0.0;
        dist += (ctx->vertices[i].x - ctx->minx) * (ctx->vertices[i].x - ctx->minx);
        dist += (ctx->vertices[i].y - ctx->miny) * (ctx->vertices[i].y - ctx->miny);
        dist += (ctx->vertices[i].z - ctx->minz) * (ctx->vertices[i].z - ctx->minz);
        
        if (fabs(dist) < err) {
            err = fabs(dist);
            inside_point = i;
        }
    }
    
    printf("Bounding box min: %f %f %f, closest vertex: %f %f %f\n",
           ctx->minx, ctx->miny, ctx->minz,
           ctx->vertices[inside_point].x, ctx->vertices[inside_point].y, ctx->vertices[inside_point].z);
    
    // Find a triangle that contains this vertex
    for (i = 0; i < ctx->total_triangles; i++) {
        if ((ctx->surface[i].v1 == inside_point) || 
            (ctx->surface[i].v2 == inside_point) || 
            (ctx->surface[i].v3 == inside_point)) {
            break;
        }
    }
    closestTri = i;
    
    // Check if the normal points outward by comparing plane distance
    if (ctx->distances[closestTri] > 0) {
        printf("Normals are correctly oriented\n");
    }
    else {
        // Flip all normals
        for (i = 0; i < ctx->total_triangles; i++) {
            ctx->normals[i].x *= -1;
            ctx->normals[i].y *= -1;
            ctx->normals[i].z *= -1;
            ctx->distances[i] *= -1;
        }
        printf("All normals were flipped to be correctly oriented\n");
    }
}

} // namespace SDFLibrary

// Anonymous namespace for internal helper functions
namespace {
    // Helper: Insert triangle into processing queue
    void insert_tri(SDFContext* ctx, int tri, std::map<int,int>& myMap, 
                    int* neighbors, int& usedNeighs, int& total_done) {
        if (ctx->surface[tri].type == -1) return;
        
        if (myMap.find(tri) == myMap.end()) {
            myMap[tri] = tri;
            neighbors[usedNeighs++] = tri;
            total_done++;
        }
    }
    
    // Align triangle 'what' with triangle 'with' sharing vertex 'vert'
    void align_us(SDFContext* ctx, int with, int what, int vert,
                  std::map<int,int>& myMap, int* neighbors, int& usedNeighs, int& total_done) {
        if (ctx->surface[what].type != -1) return;
        
        int v1[3], v2[3];
        int flag = -1;
        
        v1[0] = ctx->surface[with].v1;
        v1[1] = ctx->surface[with].v2;
        v1[2] = ctx->surface[with].v3;
        v2[0] = ctx->surface[what].v1;
        v2[1] = ctx->surface[what].v2;
        v2[2] = ctx->surface[what].v3;
        
        // Find the shared edge (two shared vertices)
        for (int i = 0; i < 3; i++) {
            if (v1[i] == vert) continue;
            
            for (int j = 0; j < 3; j++) {
                if (v2[j] == vert) continue;
                
                if (v1[i] == v2[j]) {
                    flag = v1[i];
                    break;
                }
            }
            if (flag != -1) break;
        }
        
        if (flag == -1) return;
        
        // Compare triangle orientations and align if needed
        if (triangle_angles(ctx, with, what, vert, flag)) {
            ctx->surface[what].type = ctx->surface[with].type;
        }
        else {
            // Flip normal and orientation
            ctx->normals[what].x *= -1;
            ctx->normals[what].y *= -1;
            ctx->normals[what].z *= -1;
            ctx->distances[what] *= -1;
            ctx->surface[what].type = !(ctx->surface[with].type);
        }
        
        insert_tri(ctx, what, myMap, neighbors, usedNeighs, total_done);
    }
    
    // Orient all triangles connected to this vertex
    void orient_vert(SDFContext* ctx, int tri, int vert,
                     std::map<int,int>& myMap, int* neighbors, int& usedNeighs, int& total_done) {
        for (int i = 0; i < ctx->vertices[vert].trisUsed; i++) {
            if (tri != ctx->vertices[vert].tris[i]) {
                align_us(ctx, tri, ctx->vertices[vert].tris[i], vert,
                        myMap, neighbors, usedNeighs, total_done);
            }
        }
    }
    
    // Orient all triangles adjacent to this triangle
    void correct_tri(SDFContext* ctx, int tri,
                     std::map<int,int>& myMap, int* neighbors, int& usedNeighs, int& total_done) {
        orient_vert(ctx, tri, ctx->surface[tri].v1, myMap, neighbors, usedNeighs, total_done);
        orient_vert(ctx, tri, ctx->surface[tri].v2, myMap, neighbors, usedNeighs, total_done);
        orient_vert(ctx, tri, ctx->surface[tri].v3, myMap, neighbors, usedNeighs, total_done);
    }
    
    // Find and mark the next unprocessed component
    int getNextComponent(SDFContext* ctx, std::map<int,int>& myMap, 
                        int* neighbors, int& usedNeighs, int& prevUsed, int& total_done) {
        for (int i = 0; i < ctx->total_triangles; i++) {
            if (ctx->surface[i].type == -1) {
                ctx->surface[i].type = 1;
                insert_tri(ctx, i, myMap, neighbors, usedNeighs, total_done);
                prevUsed = usedNeighs;
                return i;
            }
        }
        return -1;
    }
} // end anonymous namespace

namespace SDFLibrary {

// Main function: Ensure all triangle normals are consistently oriented
void start_fireworks(SDFContext* ctx) {
    int* neighbors = (int*)malloc(sizeof(int) * ctx->total_triangles);
    
    printf("\n<start_fireworks> started...\n");
    
    std::map<int,int> myMap;
    int usedNeighs = 0, prevUsed = 0, total_done = 0;
    int lastone = 0;
    int component_count = 0;
    
    while (true) {
        prevUsed = usedNeighs;
        printf("Processing %d triangles...\n", prevUsed);
        
        if (lastone == prevUsed) {
            int comp = getNextComponent(ctx, myMap, neighbors, usedNeighs, prevUsed, total_done);
            if (comp != -1) {
                component_count++;
            }
        }
        else {
            lastone = prevUsed;
        }
        
        // Process all triangles in the current queue
        for (int i = 0; i < prevUsed; i++) {
            correct_tri(ctx, neighbors[i], myMap, neighbors, usedNeighs, total_done);
        }
        
        if (total_done == ctx->total_triangles) {
            printf("Normal orientation adjustment complete.\n");
            break;
        }
        
        // Rebuild neighbor list from map
        int j = 0;
        for (std::map<int,int>::const_iterator iter = myMap.begin(); 
             iter != myMap.end(); ++iter) {
            neighbors[j++] = iter->first;
        }
        usedNeighs = j;
    }
    
    free(neighbors);
    
    printf("Number of disconnected components found: %d\n", component_count);
    
    // Final re-orientation to ensure all normals point outward
    re_orient_all(ctx);
}

} // namespace SDFLibrary
