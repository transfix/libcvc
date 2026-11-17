#ifndef __REMOVE_CONTOUR_INTERSECTIONS_H__
#define __REMOVE_CONTOUR_INTERSECTIONS_H__

#include <list>

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

/// delta - After calling this function, all contours will be
/// separated by at least delta.
template <typename Contour_iter, typename Out_iter>
void remove_contour_intersections(Contour_iter begin, Contour_iter end, 
				  Number_type delta, 
				  Out_iter out);

// template <typename Contour_iter, typename Out_iter, typename Failure_iter>
// void remove_contour_intersections2(Contour_iter begin, Contour_iter end, 
// 				   Number_type delta, 
// 				   Out_iter out,
// 				   Failure_iter failures);

CONTOURTILER_END_NAMESPACE

#endif
