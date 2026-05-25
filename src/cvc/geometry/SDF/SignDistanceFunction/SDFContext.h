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

#ifndef CCV_SDF_CONTEXT_H
#define CCV_SDF_CONTEXT_H

#include <memory>
#include <vector>
// Always disable bounds checking in boost::multi_array for performance
// (even in debug mode - it's prohibitively slow otherwise)
#define BOOST_DISABLE_ASSERTS
#include <boost/multi_array.hpp>
#include "head.h"

namespace SDFLibrary {

/**
 * SDFContext - Thread-safe encapsulation of SDF computation state
 * 
 * This class encapsulates all the state needed for SDF computation,
 * replacing the global variables in the original implementation.
 * Each instance can run independently in separate threads.
 * 
 * Memory is managed using smart pointers for automatic cleanup.
 */
class SDFContext {
public:
    // Constructor and destructor
    SDFContext();
    ~SDFContext();
    
    // Prevent copying (use separate instances for separate threads)
    SDFContext(const SDFContext&) = delete;
    SDFContext& operator=(const SDFContext&) = delete;
    
    // Allow moving
    SDFContext(SDFContext&&) = default;
    SDFContext& operator=(SDFContext&&) = default;
    
    // Configuration
    void setParameters(int size, int isNormalFlip, const float* mins, const float* maxs);
    
    // Main computation pipeline
    bool initSDF();
    void readGeom(int nverts, const float* verts, int ntris, const int* tris);
    void adjustData();
    void compute();
    
    // Result access
    float* getValues() const { return values.get(); }
    std::unique_ptr<float[]> releaseValues() { return std::move(values); }
    
    // Geometry access (for helper functions)
    inline int index2vert(int i, int j, int k) const {
        return k * (size + 1) * (size + 1) + j * (size + 1) + i;
    }
    
    inline void vert2index(int c, int &i, int &j, int &k) const {
        i = c % (size + 1);
        int _left = c / (size + 1);
        j = _left % (size + 1);
        _left = _left / (size + 1);
        k = _left;
        
        if (i < 0) i = 0;
        if (j < 0) j = 0;
        if (k < 0) k = 0;
        if (i > size + 1) i = size + 1;
        if (j > size + 1) j = size + 1;
        if (k > size + 1) k = size + 1;
    }
    
    inline int index2cell(int i, int j, int k) const {
        return k * size * size + j * size + i;
    }
    
    inline void cell2index(int c, int &i, int &j, int &k) const {
        i = c % size;
        int _left = c / size;
        j = _left % size;
        _left = _left / size;
        k = _left;
        
        if (i < 0) i = 0;
        if (j < 0) j = 0;
        if (k < 0) k = 0;
        if (i > size) i = size;
        if (j > size) j = size;
        if (k > size) k = size;
    }
    
    inline double xCoord(int i) const {
        return minext[0] + i * span[0];
    }
    
    inline double yCoord(int j) const {
        return minext[1] + j * span[1];
    }
    
    inline double zCoord(int k) const {
        return minext[2] + k * span[2];
    }
    
    void object2octree(double xmin, double ymin, double zmin, 
                      double xmax, double ymax, double zmax, 
                      int &ci, int &cj, int &ck) const;
    
    // Utility functions
    static inline bool isEqual(double one, double two) {
        return (-TOLERANCE <= (one - two)) && ((one - two) <= TOLERANCE);
    }
    
    static inline bool isZero(double num) {
        return (-TOLERANCE <= num) && (num <= TOLERANCE);
    }
    
    static inline bool isNegative(double num) {
        return num < 0;
    }
    
    static inline bool isBetween(double one, double two, double num) {
        return ((one <= num) && (num <= two)) || 
               (isEqual(num, one)) || (isEqual(num, two));
    }
    
    static bool isZero(const myPoint& one);
    static bool isSame(const myPoint& one, const myPoint& two);
    
    // Distance propagation function
    void apply_distance_transform(int vi, int vj, int vk);
    void insert_bound_vert(int vert);
    
    // Octree functions
    void build_octree();
    // update_bounding_box removed - uses global sdf, replaced by update_bounding_box_ctx
    
    // Process triangle for normal computation
    void process_triangle(int i);
    
    // Constants
    static constexpr double TOLERANCE = 1e-5;
    
    // Public member variables (accessed by other modules)
    double MAX_DIST;
    int size;
    int total_points, total_triangles, all_verts_touched;
    double minx, miny, minz, maxx, maxy, maxz;
    int octree_depth;
    int flipNormals;
    
    double minext[3];
    double maxext[3];
    double span[3];
    
    // Geometry data (using smart pointers for automatic cleanup)
    std::unique_ptr<triangle[]> surface;
    std::unique_ptr<myVert[]> vertices;
    std::unique_ptr<myPoint[]> normals;
    std::unique_ptr<double[]> distances;
    std::unique_ptr<bool[]> bverts;
    std::unique_ptr<int[]> queues;
    
    // 3D octree structure (using boost::multi_array for RAII management)
    boost::multi_array<cell, 3> sdf;
    
    // Voxel values
    std::unique_ptr<voxel[]> voxel_values;
    
private:
    // Result buffer
    std::unique_ptr<float[]> values;
};

} // namespace SDFLibrary

#endif // CCV_SDF_CONTEXT_H
