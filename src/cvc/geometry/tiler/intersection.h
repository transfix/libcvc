#ifndef __INTERSECTION_H__
#define __INTERSECTION_H__

#include <boost/unordered_map.hpp>

#include <ContourTiler/common.h>
#include <ContourTiler/mtiler_operations.h>
#include <ContourTiler/Segment_3_undirected.h>
#include <ContourTiler/Intersections.h>
#include <ContourTiler/triangle_utils.h>
#include <ContourTiler/Polyline.h>
#include <ContourTiler/Distance_functor.h>
#include <ContourTiler/Z_adjustments.h>

CONTOURTILER_BEGIN_NAMESPACE

Polyline_2 trim_forward(const Polygon_2& p, const Polyline_2& c);
Polyline_2 trim_backward(const Polygon_2& p, const Polyline_2& c);

// Removes intersections between yellow and green tiles
// template <typename Out_iter>
// void remove_intersections(TW_handle yellow,
// 			  TW_handle green,
// 			  Number_type yz, Number_type gz,
// 			  Out_iter new_yellow, Out_iter new_green,
// 			  Number_type epsilon);

template <typename Tile_iter, typename Out_iter>
void remove_intersections(TW_handle yellow,
			  TW_handle green,
			  Tile_iter yellow_begin, Tile_iter yellow_end,
			  Tile_iter green_begin, Tile_iter green_end,
			  Number_type yz, Number_type gz,
			  Out_iter new_yellow, Out_iter new_green,
			  Number_type epsilon,
			  boost::unordered_map<Segment_3_undirected, list<Point_3> >& edge2points);

template <typename Tile_handle>
bool neighbor(Tile_handle green, Tile_handle yellow, const Point_3& gp, int g, Point_3& gn, const Intersections<Tile_handle>& intersections);

template <typename Tile_handle>
Polyline_2 find_exit(TW_handle tw_yellow, TW_handle tw_green, Tile_handle green, Tile_handle yellow, 
		     const Point_3& gp, int g, const Intersections<Tile_handle>& intersections,
		     Z_adjustments<Tile_handle>& z_adjustments);

template <typename Tile_handle>
void get_polyline_cuts(TW_handle tw_yellow, TW_handle tw_green, Intersections<Tile_handle>& intersections, 
		       boost::unordered_map<Tile_handle, boost::unordered_set<Polyline_2> >& cuts,
		       Z_adjustments<Tile_handle>& z_adjustments);

//------------------------------------------------------------------------------
// Intersection removal is done on two components at a time, a yellow and a
// green component.  Each component has a home slice value, i.e., a z-value
// for the contour from which the component derives.  This is called z_home
// in the code.
//------------------------------------------------------------------------------

template <typename Tile_iter>
Intersections<typename std::iterator_traits<Tile_iter>::value_type>
get_intersections(TW_handle tw_yellow,
		  TW_handle tw_green,
		  Tile_iter ybegin, Tile_iter yend, Tile_iter gbegin, Tile_iter gend, 
		  Z_adjustments<typename std::iterator_traits<Tile_iter>::value_type>& z_adjustments);

template <typename Tile_handle, typename Cuts_iter>
boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> > 
map_point2edges(Tile_handle tile,
		Cuts_iter cuts_begin, Cuts_iter cuts_end,
		const Intersections<Tile_handle>& ints, int i);

CONTOURTILER_END_NAMESPACE

#endif
