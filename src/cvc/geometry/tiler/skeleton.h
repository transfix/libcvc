#ifndef __SKELETON_H__
#define __SKELETON_H__

#include <list>

#include <ContourTiler/common.h>
#include <ContourTiler/Tiler_workspace.h>
#include <ContourTiler/Untiled_region.h>

CONTOURTILER_BEGIN_NAMESPACE

void triangulate_to_medial_axis(const Untiled_region& contour, Vertices& vertices, Tiler_workspace& w);

template <typename Tile_iterator, typename ID_factory>
void medial_axis_stable(const Untiled_region& region, Number_type zmid, Tile_iterator tiles, 
			ID_factory& id_factory, Tiler_workspace& tw);

void to_simple(const Untiled_region& region);

CONTOURTILER_END_NAMESPACE

#endif
