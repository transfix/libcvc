/*! \file symmetric_intersection.cpp
 * Computing the symmetric intersection of two polygons with holes.
 */

#include <CGAL/Cartesian.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <list>
#include <ContourTiler/common.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/kernel_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

// Removed 5/24/11.  Didn't look like it was being used.
// Polygon_2 convert(const Bso_polygon_2& bp, Number_type z) {
//   Polygon_2 p;
//   for (Bso_polygon_2::Vertex_iterator pit = bp.vertices_begin(); pit != bp.vertices_end(); ++pit) {
//     p.push_back(Point_2(CGAL::to_double(pit->x()), CGAL::to_double(pit->y()), z));
//   }
//   return p;
// }

// template <typename Out_iter>
// void polygon_intersection(const Bso_polygon_2& P, const Bso_polygon_2& Q, 
// 			Out_iter out)
// {
//   typedef std::list<Bso_polygon_with_holes_2>            Pwh_list_2;

//   // Compute the symmetric intersection of P and Q.
//   Pwh_list_2 intersections;
//   Pwh_list_2::const_iterator it;

//   CGAL::intersection(P, Q, std::back_inserter(out));
// }

// template <typename Out_iter>
// void polygon_intersection(const Bso_polygon_with_holes_2& P, const Bso_polygon_with_holes_2& Q, 
// 			Out_iter P_out, Out_iter Q_out)
// {
//   typedef std::list<Bso_polygon_with_holes_2>            Pwh_list_2;

//   // Compute the symmetric intersection of P and Q.
//   Pwh_list_2 diff;
//   Pwh_list_2::const_iterator it;

// //   CGAL::symmetric_intersection (P, Q, std::back_inserter(diff));
//   CGAL::intersection(P, Q, std::back_inserter(diff));
//   for (it = diff.begin(); it != diff.end(); ++it) {
//     *P_out++ = *it;
//   }
//   // one-sided
//   *Q_out++ = Q;
//   // This two-sided implementation makes big holes in areas of large overlap
//   // diff.clear();
//   // CGAL::intersection(Q, P, std::back_inserter(diff));
//   // for (it = diff.begin(); it != diff.end(); ++it) {
//   //   *Q_out++ = *it;
//   // }
// }

// // Split polygon at any point that there is a shared point.
// template <typename Out_iter>
// void split_at_coincident(const Bso_polygon_2& p, Out_iter out)
// {
//   bool split = false;
//   Bso_polygon_2::Vertex_circulator first = p.vertices_circulator();
//   Bso_polygon_2::Vertex_circulator cur = first;
//   do {
//     Bso_polygon_2::Vertex_circulator test = cur;
//     ++test;
//     do {
//       if (*cur == (*test)) {
// 	Bso_polygon_2 newp(cur, test);
// 	if (newp.size() > 0)
// 	  *out++ = newp;

// 	newp = Bso_polygon_2(test, cur);
// 	if (newp.size() > 0)
// 	  split_at_coincident(newp, out);

// 	split = true;
//       }
//       ++test;
//     } while (test != first);
    
//     ++cur;
//   } while (cur != first);
//   if (!split) {
//     if (!p.is_simple())
//       throw new logic_error("split_at_coincident: Polygon is not simple");
//     *out++ = p;
//   }
// }

// void verify(list<Bso_polygon_with_holes_2>& polygons)
// {
//   typedef list<Bso_polygon_with_holes_2>::iterator Iter;
//   Iter it = polygons.begin();
//   while (it != polygons.end()) {
//     if (it->outer_boundary().size() == 0) {
//       it = polygons.erase(it);
//     }
//     else {
//       // First fix any non-simple holes

//       // Erase all original holes (keep all_holes as a copy)
//       list<Bso_polygon_2> all_holes(it->holes_begin(), it->holes_end());
//       while (it->holes_begin() != it->holes_end()) {
// 	it->erase_hole(it->holes_begin());
//       }

//       // Now go through the holes, fix each and re-add result to polygon_with_holes.
//       for (list<Bso_polygon_2>::const_iterator hole_it = all_holes.begin(); hole_it != all_holes.end(); ++hole_it) {
// 	if (!hole_it->is_simple()) {
// 	  list<Bso_polygon_2> temp;
// 	  split_at_coincident(*hole_it, back_inserter(temp));
// 	  for (list<Bso_polygon_2>::const_iterator new_it = temp.begin(); new_it != temp.end(); ++new_it) {
// 	    it->add_hole(*new_it);
// 	  }
// 	}
// 	else {
// 	  it->add_hole(*hole_it);
// 	}
//       }

//       // Now fix the outer boundary if it is non-simple.  This code currently does not
//       // support fixing the outer boundary if there are holes.
//       if (!it->outer_boundary().is_simple()) {
// 	if (it->holes_begin() != it->holes_end()) {
// 	  throw runtime_error("fixing non-simple outer boundaries not yet supported when there are holes");
// 	}
// 	list<Bso_polygon_2> temp;
// 	split_at_coincident(it->outer_boundary(), back_inserter(temp));
// 	it = polygons.erase(it);
// 	for (list<Bso_polygon_2>::const_iterator new_it = temp.begin(); new_it != temp.end(); ++new_it) {
// 	  polygons.push_back(Bso_polygon_with_holes_2(*new_it));
// 	}
//       }
//       else {
// 	++it;
//       }
//     }
//   }
// }

template <typename Out_iter>
void polygon_intersection(const Polygon_2& P, const Polygon_2& Q, 
			Out_iter out) {

  Number_type z = P[0].z();
  Bso_polygon_2 bso_P, bso_Q;
  for (Polygon_2::Vertex_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it) {
    bso_P.push_back(Bso_point_2(it->x(), it->y()));
  }
  for (Polygon_2::Vertex_iterator it = Q.vertices_begin(); it != Q.vertices_end(); ++it) {
    bso_Q.push_back(Bso_point_2(it->x(), it->y()));
  }

  list<Bso_polygon_with_holes_2> bso_out;
  CGAL::intersection(bso_P, bso_Q, std::back_inserter(bso_out));
  // polygon_intersection(bso_P, bso_Q, back_inserter(bso_new_P), back_inserter(bso_new_Q));

  // Split into new polygons if necessary
  // verify(bso_new_P);
  // verify(bso_new_Q);

  for (list<Bso_polygon_with_holes_2>::const_iterator it = bso_out.begin(); it != bso_out.end(); ++it) {
    *out++ = to_common(*it, z);
  }
  // for (list<Bso_polygon_with_holes_2>::const_iterator it = bso_new_Q.begin(); it != bso_new_Q.end(); ++it) {
  //   *Q_out++ = to_common(*it, z);
  // }
}

template
void polygon_intersection(const Polygon_2& P, const Polygon_2& Q, 
			back_insert_iterator<vector<Polygon_with_holes_2> > out);

template
void polygon_intersection(const Polygon_2& P, const Polygon_2& Q, 
			back_insert_iterator<list<Polygon_with_holes_2> > out);

// Bso_polygon_2 to_bso(const Polygon_2& P)
// {
//   Bso_polygon_2 bso_P;
//   for (Polygon_2::Vertex_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it) {
//     bso_P.push_back(Bso_point_2(it->x(), it->y()));
//   }
//   return bso_P;
// }

// Bso_polygon_with_holes_2 to_bso(const Polygon_with_holes_2& P)
// {
//   Bso_polygon_with_holes_2 bso_P(to_bso(P));
//   for (Polygon_with_holes_2::Hole_const_iterator it = P.holes_begin(); it != P.holes_end(); ++it) {
//     bso_P.add_hole(to_bso(*it));
//   }
//   return bso_P;
// }

// template <typename Out_iter>
// void polygon_intersection(const Polygon_with_holes_2& P, const Polygon_with_holes_2& Q, 
// 			Out_iter P_out, Out_iter Q_out) {

//   Number_type z = P.outer_boundary()[0].z();
//   Bso_polygon_with_holes_2 bso_P = change_kernel<Bso_kernel>(P);
//   Bso_polygon_with_holes_2 bso_Q = change_kernel<Bso_kernel>(Q);

//   list<Bso_polygon_with_holes_2> bso_new_P, bso_new_Q;
//   polygon_intersection(bso_P, bso_Q, back_inserter(bso_new_P), back_inserter(bso_new_Q));

//   // Split into new polygons if necessary
//   verify(bso_new_P);
//   verify(bso_new_Q);

//   for (list<Bso_polygon_with_holes_2>::const_iterator it = bso_new_P.begin(); it != bso_new_P.end(); ++it) {
//     *P_out++ = to_common(*it, z);
//   }
//   for (list<Bso_polygon_with_holes_2>::const_iterator it = bso_new_Q.begin(); it != bso_new_Q.end(); ++it) {
//     *Q_out++ = to_common(*it, z);
//   }
// }

// template
// void polygon_intersection(const Polygon_with_holes_2& P, const Polygon_with_holes_2& Q, 
// 			back_insert_iterator<list<Polygon_with_holes_2> > P_out, back_insert_iterator<list<Polygon_with_holes_2> > Q_out);

CONTOURTILER_END_NAMESPACE
