/*! \file symmetric_difference.cpp
 * Computing the symmetric difference of two polygons with holes.
 */

#include <CGAL/Cartesian.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <list>
#include <ContourTiler/common.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/kernel_utils.h>
#include <boost/foreach.hpp>

CONTOURTILER_BEGIN_NAMESPACE

// Removed 5/24/11.  Didn't look like it was being used.
// Polygon_2 convert(const Bso_polygon_2& bp, Number_type z) {
//   Polygon_2 p;
//   for (Bso_polygon_2::Vertex_iterator pit = bp.vertices_begin(); pit != bp.vertices_end(); ++pit) {
//     p.push_back(Point_2(CGAL::to_double(pit->x()), CGAL::to_double(pit->y()), z));
//   }
//   return p;
// }

template <typename Out_iter>
void polygon_difference(const Bso_polygon_2& P, const Bso_polygon_2& Q, 
			Out_iter P_out, Out_iter Q_out)
{
  typedef std::list<Bso_polygon_with_holes_2>            Pwh_list_2;

  // Compute the symmetric difference of P and Q.
  Pwh_list_2 diff;
  Pwh_list_2::const_iterator it;

//   CGAL::symmetric_difference (P, Q, std::back_inserter(diff));
  CGAL::difference(P, Q, std::back_inserter(diff));
  for (it = diff.begin(); it != diff.end(); ++it) {
    *P_out++ = *it;
  }
  diff.clear();
  CGAL::difference(Q, P, std::back_inserter(diff));
  for (it = diff.begin(); it != diff.end(); ++it) {
    *Q_out++ = *it;
  }
}

template <typename Out_iter>
void polygon_difference(const Bso_polygon_with_holes_2& P, const Bso_polygon_with_holes_2& Q, 
			Out_iter P_out, Out_iter Q_out)
{
  typedef std::list<Bso_polygon_with_holes_2>            Pwh_list_2;

  // Compute the symmetric difference of P and Q.
  Pwh_list_2 diff;
  Pwh_list_2::const_iterator it;

//   CGAL::symmetric_difference (P, Q, std::back_inserter(diff));
  CGAL::difference(P, Q, std::back_inserter(diff));
  for (it = diff.begin(); it != diff.end(); ++it) {
    *P_out++ = *it;
  }
  // one-sided
  *Q_out++ = Q;
  // This two-sided implementation makes big holes in areas of large overlap
  // diff.clear();
  // CGAL::difference(Q, P, std::back_inserter(diff));
  // for (it = diff.begin(); it != diff.end(); ++it) {
  //   *Q_out++ = *it;
  // }
}

// // Split polygon at any point that there is a shared point.
// template <typename Out_iter>
// void split_at_coincident_old(const Bso_polygon_2& p, Out_iter out)
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

// Split polygon at any point that there is a shared point.
template <typename Out_iter>
void split_at_coincident(const Bso_polygon_2& p, Out_iter out)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.split_at_coincident");

  // Find all coincident points
  const int n = p.size();
  map<Bso_point_2, int> point_set;
  set<int> coincident_set;
  for (int i = 0; i < n; ++i) {
    if (point_set.find(p[i]) != point_set.end()) {
      coincident_set.insert(i);
      coincident_set.insert(point_set[p[i]]);
    }
    point_set[p[i]] = i;
  }

  if (coincident_set.empty()) {
    *out++ = p;
    return;
  }

  // start at a knot and split into two
  int cur = *coincident_set.begin();
  const int beg = cur;
  Bso_polygon_2 p1, p2;
  do {
    p1.push_back(p[cur]);
    cur = (cur+1)%n;
  } while (p[cur] != p[beg]);

  do {
    p2.push_back(p[cur]);
    cur = (cur+1)%n;
  } while (p[cur] != p[beg]);

  // recurse
  split_at_coincident(p1, out);
  split_at_coincident(p2, out);
}

void verify(list<Bso_polygon_with_holes_2>& polygons)
{
  typedef list<Bso_polygon_with_holes_2>::iterator Iter;
  Iter it = polygons.begin();
  while (it != polygons.end()) {
    if (it->outer_boundary().size() == 0) {
      it = polygons.erase(it);
    }
    else {
      // First fix any non-simple holes

      // Erase all original holes (keep all_holes as a copy)
      list<Bso_polygon_2> all_holes(it->holes_begin(), it->holes_end());
      while (it->holes_begin() != it->holes_end()) {
	it->erase_hole(it->holes_begin());
      }

      // Now go through the holes, fix each and re-add result to polygon_with_holes.
      for (list<Bso_polygon_2>::const_iterator hole_it = all_holes.begin(); hole_it != all_holes.end(); ++hole_it) {
	if (!hole_it->is_simple()) {
	  list<Bso_polygon_2> temp;
	  split_at_coincident(*hole_it, back_inserter(temp));
	  for (list<Bso_polygon_2>::const_iterator new_it = temp.begin(); new_it != temp.end(); ++new_it) {
	    it->add_hole(*new_it);
	  }
	}
	else {
	  it->add_hole(*hole_it);
	}
      }

      // Now fix the outer boundary if it is non-simple.  This code currently does not
      // support fixing the outer boundary if there are holes.
      if (!it->outer_boundary().is_simple()) {
	if (it->holes_begin() != it->holes_end()) {
	  throw runtime_error("fixing non-simple outer boundaries not yet supported when there are holes");
	}
	list<Bso_polygon_2> temp;
	split_at_coincident(it->outer_boundary(), back_inserter(temp));
	it = polygons.erase(it);
	for (list<Bso_polygon_2>::const_iterator new_it = temp.begin(); new_it != temp.end(); ++new_it) {
	  polygons.push_back(Bso_polygon_with_holes_2(*new_it));
	}
      }
      else {
	++it;
      }
    }
  }
}

// Polygon_2 to_poly(const Bso_polygon_2& P, Number_type z)
// {
//   Polygon_2 p;
//   for (Bso_polygon_2::Vertex_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it) {
//     p.push_back(Point_2(CGAL::to_double(it->x()), CGAL::to_double(it->y()), z));
//   }
//   return p;
// }

// Polygon_with_holes_2 to_poly(const Bso_polygon_with_holes_2& P, Number_type z)
// {
//   Polygon_with_holes_2 p(to_poly(P.outer_boundary(), z));
//   for (Bso_polygon_with_holes_2::Hole_const_iterator it = P.holes_begin(); it != P.holes_end(); ++it) {
//     p.add_hole(to_poly(*it, z));
//   }
//   return p;
// }

template <typename Out_iter>
void polygon_difference(const Polygon_2& P, const Polygon_2& Q, 
			Out_iter P_out, Out_iter Q_out) {

  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.polygon_difference");

  LOG4CPLUS_TRACE(logger, "P = " << pp(P));
  LOG4CPLUS_TRACE(logger, "Q = " << pp(Q));

  Number_type z = P[0].z();
  Bso_polygon_2 bso_P, bso_Q;
  for (Polygon_2::Vertex_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it) {
    bso_P.push_back(Bso_point_2(it->x(), it->y()));
  }
  for (Polygon_2::Vertex_iterator it = Q.vertices_begin(); it != Q.vertices_end(); ++it) {
    bso_Q.push_back(Bso_point_2(it->x(), it->y()));
  }

  list<Bso_polygon_with_holes_2> bso_new_P, bso_new_Q;
  polygon_difference(bso_P, bso_Q, back_inserter(bso_new_P), back_inserter(bso_new_Q));

  // Split into new polygons if necessary
  verify(bso_new_P);
  verify(bso_new_Q);

  for (list<Bso_polygon_with_holes_2>::const_iterator it = bso_new_P.begin(); it != bso_new_P.end(); ++it) {
    *P_out++ = to_common(*it, z);
    LOG4CPLUS_TRACE(logger, "new_P = " << pp(to_common(*it, z)));
  }
  for (list<Bso_polygon_with_holes_2>::const_iterator it = bso_new_Q.begin(); it != bso_new_Q.end(); ++it) {
    *Q_out++ = to_common(*it, z);
    LOG4CPLUS_TRACE(logger, "new_Q = " << pp(to_common(*it, z)));
  }
}

template
void polygon_difference(const Polygon_2& P, const Polygon_2& Q, 
			back_insert_iterator<vector<Polygon_with_holes_2> > P_out, back_insert_iterator<vector<Polygon_with_holes_2> > Q_out);

template
void polygon_difference(const Polygon_2& P, const Polygon_2& Q, 
			back_insert_iterator<list<Polygon_with_holes_2> > P_out, back_insert_iterator<list<Polygon_with_holes_2> > Q_out);

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

template <typename Out_iter>
void polygon_difference(const Polygon_with_holes_2& P, const Polygon_with_holes_2& Q, 
			Out_iter P_out, Out_iter Q_out) {

  Number_type z = P.outer_boundary()[0].z();
  Bso_polygon_with_holes_2 bso_P = change_kernel<Bso_kernel>(P);
  Bso_polygon_with_holes_2 bso_Q = change_kernel<Bso_kernel>(Q);

  list<Bso_polygon_with_holes_2> bso_new_P, bso_new_Q;
  polygon_difference(bso_P, bso_Q, back_inserter(bso_new_P), back_inserter(bso_new_Q));

  // Split into new polygons if necessary
  verify(bso_new_P);
  verify(bso_new_Q);

  for (list<Bso_polygon_with_holes_2>::const_iterator it = bso_new_P.begin(); it != bso_new_P.end(); ++it) {
    *P_out++ = to_common(*it, z);
  }
  for (list<Bso_polygon_with_holes_2>::const_iterator it = bso_new_Q.begin(); it != bso_new_Q.end(); ++it) {
    *Q_out++ = to_common(*it, z);
  }
}

template
void polygon_difference(const Polygon_with_holes_2& P, const Polygon_with_holes_2& Q, 
			back_insert_iterator<list<Polygon_with_holes_2> > P_out, back_insert_iterator<list<Polygon_with_holes_2> > Q_out);

CONTOURTILER_END_NAMESPACE
