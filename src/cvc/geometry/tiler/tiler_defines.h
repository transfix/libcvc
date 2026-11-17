#ifndef __TILER_DEFINES_H__
#define __TILER_DEFINES_H__

#include <ContourTiler/common.h>
#include <ContourTiler/CGAL_hash.h>
#include <ContourTiler/Vertex_map.h>
#include <ContourTiler/Correspondences.h>
#include <ContourTiler/Tiling_region.h>
#include <ContourTiler/Vertex_completion_map.h>
#include <ContourTiler/OTV_pair.h>
#include <ContourTiler/Vertices.h>

CONTOURTILER_BEGIN_NAMESPACE

typedef Vertex_map<Point_3> OTV_table;
typedef Vertices::const_iterator Vertex_iterator;

CONTOURTILER_END_NAMESPACE

#endif
