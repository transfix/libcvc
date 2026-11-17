#include<vector>
#include<iterator>
#include<iostream>
#include<iomanip>
#include<string>
#include<limits>

#include<boost/shared_ptr.hpp>
#include<boost/foreach.hpp>

#include<CGAL/basic.h>
#include<CGAL/Cartesian.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/Straight_skeleton_builder_2.h>
#include<CGAL/Polygon_offset_builder_2.h>
#include<CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include<CGAL/compute_outer_frame_margin.h>
#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Exact_predicates_exact_constructions_kernel.h>

#include<CGAL/create_offset_polygons_2.h>

#include <ContourTiler/bso_rational_nt.h>
#include <ContourTiler/offset_polygon.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/kernel_utils.h>

// // This is the recommended kernel
typedef CGAL::Exact_predicates_inexact_constructions_kernel Offset_kernel;

typedef Offset_kernel::Point_2 Offset_point_2;
typedef CGAL::Polygon_2<Offset_kernel>    Offset_contour;
typedef CGAL::Polygon_2<Offset_kernel>    Offset_contour;
typedef boost::shared_ptr<Offset_contour> Offset_contourPtr;
typedef std::vector<Offset_contourPtr>    Offset_contourSequence ;
typedef Offset_kernel::FT                 Offset_FT;

typedef CGAL::Straight_skeleton_2<Offset_kernel> Ss;

typedef Ss::Halfedge_iterator Halfedge_iterator;
typedef Ss::Halfedge_handle   Halfedge_handle;
typedef Ss::Vertex_handle     Vertex_handle;

typedef CGAL::Straight_skeleton_builder_traits_2<Offset_kernel>      SsBuilderTraits;
typedef CGAL::Straight_skeleton_builder_2<SsBuilderTraits,Ss> SsBuilder;

typedef CGAL::Polygon_offset_builder_traits_2<Offset_kernel>                  OffsetBuilderTraits;
typedef CGAL::Polygon_offset_builder_2<Ss,OffsetBuilderTraits,Offset_contour> OffsetBuilder;


CONTOURTILER_BEGIN_NAMESPACE

Number_type temp_round(Number_type d, int dec) {
  d *= pow((Number_type)10, (Number_type)dec);
  d = (d < 0) ? ceil(d-0.5) : floor(d+0.5);
  d /= pow((Number_type)10, (Number_type)dec);
  return d;
}

void temp_round(Polygon_2& p, int dec) {
  Polygon_2::Vertex_iterator vit;
  for (vit = p.vertices_begin(); vit != p.vertices_end(); ++vit) {
    Point_2 pnt = *vit;
    pnt = Point_25_<Kernel>(temp_round(pnt.x(), dec), temp_round(pnt.y(), dec), pnt.z(), pnt.id());
    p.set(vit, pnt);
  }

  p = Polygon_2(p.vertices_begin(), unique(p.vertices_begin(), p.vertices_end()));
}

//------------------------------------------------------------
// fix_simple
//
// Some points are so close that is_simple() is returning false.
// Quantize the points and remove duplicates.  Quantize to 10 decimal
// places (10 is arbitrary).
//------------------------------------------------------------
Polygon_2 fix_simple(const Polygon_2& P) 
{
  Polygon_2 ret(P);
  temp_round(ret, 10);
  Polygon_2 temp;
  for (Polygon_2::const_iterator it = ret.vertices_begin(); it != ret.vertices_end(); ++it) {
    Point_2 p = *it;
    if (temp.size() == 0 || (!xy_equal(temp[temp.size()-1], p) && !xy_equal(temp[0], p))) {
      temp.push_back(p);
    }
  }
  ret = temp;
  return ret;
  // end quantization
}

//------------------------------------------------------------
// fix_simple
//
// Some points are so close that is_simple() is returning false.
// Quantize the points and remove duplicates.  Quantize to 10 decimal
// places (10 is arbitrary).
//------------------------------------------------------------
Polygon_with_holes_2 fix_simple(const Polygon_with_holes_2& P) 
{
  Polygon_with_holes_2 ret(fix_simple(P.outer_boundary()));
  for (Polygon_with_holes_2::Hole_const_iterator it = P.holes_begin(); it != P.holes_end(); ++it) {
    ret.add_hole(fix_simple(*it));
  }
  return ret;
}

//------------------------------------------------------------
// offset_polygon
//------------------------------------------------------------
template <typename Out_iter>
// Polygon_with_holes_2 offset_polygon(const Polygon_2& polygon, Number_type offset) {
void offset_polygon(const Polygon_2& polygon, Number_type offset, Out_iter out) {
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("offset_polygon");

  if (polygon.size() < 3 || !polygon.is_simple()) {
    stringstream ss;
    ss << "offset_polygon: input polygon is not simple:"
       << "  polygon = " << pp(polygon) ;
    throw logic_error(ss.str());
  }

  LOG4CPLUS_TRACE(logger, "P = " << pp(polygon));

  typedef CGAL::Polygon_2<Offset_kernel> Offset_polygon_2;
  typedef CGAL::Polygon_with_holes_2<Offset_kernel> Offset_polygon_with_holes_2;
  typedef boost::shared_ptr<Offset_polygon_with_holes_2> PolygonPtr;
  typedef std::vector<PolygonPtr> PolygonPtrVector2;

  Number_type z = polygon[0].z();
  Offset_polygon_2 points(change_kernel<Offset_kernel>(polygon));
  if (!points.is_simple()) {
    stringstream ss;
    ss << "offset_polygon: input polygon is not simple after kernel change:"
       << "  polygon (before kernel change) = " << pp(polygon) ;
    throw logic_error(ss.str());
  }

  if (!points.is_counterclockwise_oriented())
    points.reverse_orientation();

  if (!points.is_simple()) {
    stringstream ss;
    ss << "offset_polygon: input polygon is not simple after kernel change:"
       << "  polygon (before kernel change) = " << pp(polygon) ;
    throw logic_error(ss.str());
  }

  PolygonPtrVector2 outer_offset_polygons;
  list<Offset_polygon_with_holes_2> opolys;

  Offset_polygon_with_holes_2 in_poly(points);
  if (offset < 0) {
    // See comment below.
    // outer_offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(-offset,in_poly);

    // We need to do this in two steps: the offset polygons may not be simple, which was killing
    // the call to arrange.
    typedef boost::shared_ptr<Offset_polygon_2> P1;
    typedef std::vector<P1> PV1;
    PV1 polys = create_interior_skeleton_and_offset_polygons_2(-offset,in_poly);
    PV1 good_polys;
    LOG4CPLUS_TRACE(logger, "Negative offset resulted in " << polys.size() << " polygons");
    for (PV1::iterator it = polys.begin(); it != polys.end(); ++it) {
      LOG4CPLUS_TRACE(logger, "Offset result size: " << (*it)->size());
      if ((*it)->size() > 0 && !(*it)->is_simple()) {
	LOG4CPLUS_ERROR(logger, "Offset polygon is not simple before arranging -- skipping");
      }
      else {
	good_polys.push_back(*it);
      }
    }
    outer_offset_polygons = CGAL::arrange_offset_polygons_2<Offset_polygon_with_holes_2, Offset_polygon_2>(good_polys);

    for (const auto& oop : outer_offset_polygons) {
      opolys.push_back(*oop);
    }

    // // Take the bigger one
    // int max_size = -1;
    // set<int> sizes;
    // for (const auto& oop : outer_offset_polygons) {
    //   sizes.insert(oop->outer_boundary().size());
    //   if (oop->outer_boundary().size() > max_size) {
    //     max_size = oop->outer_boundary().size();
    //     op = *oop;
    //   }
    // }
    // if (sizes.size() > 1) {
    //   LOG4CPLUS_WARN(logger, "Discarding polygon of size " << *sizes.begin() << 
    //                  " after erosion returned multiple polygons");
    //   LOG4CPLUS_TRACE(logger, "Polygon: " << pp(polygon));
    //   throw logic_error("Discarding polygon");
    // }
  }
  else {
    typedef boost::shared_ptr<Offset_polygon_2> P1;
    typedef std::vector<P1> PV1;
    PV1 polys = CGAL::create_exterior_skeleton_and_offset_polygons_2(offset,points);

    // Discard the first polygon (the exterior frame or new enclosing polygon)
    polys.erase(polys.begin());

    // Reverse the orientations (they are backwards because CGAL creates an exterior frame and treats
    // the original polygons as holes, then inner-offsets.  We are responsible for removing the
    // temporary frame and reversing orientations
    for (PV1::iterator it = polys.begin(); it != polys.end(); ++it) {
      if ((*it)->size() > 0 && !(*it)->is_simple())
	LOG4CPLUS_ERROR(logger, "Offset polygon is not simple before reversing orientation.");
      (*it)->reverse_orientation();
    }
    // Arrange the polygons as a polygon with holes
    outer_offset_polygons = CGAL::arrange_offset_polygons_2<Offset_polygon_with_holes_2, Offset_polygon_2>(polys);
    LOG4CPLUS_TRACE(logger, "Number of polygons resulting in offset: " << outer_offset_polygons.size());
    // opolys = **outer_offset_polygons.rbegin();
    opolys.push_back(**outer_offset_polygons.rbegin());
  }

  for (const auto& op : opolys) {
    Polygon_2 outer_boundary = fix_simple(to_common(op.outer_boundary(), z));
    list<Polygon_2> holes;
    for (Offset_polygon_with_holes_2::Hole_const_iterator it = op.holes_begin(); it != op.holes_end(); ++it) {
      holes.push_back(fix_simple(to_common(*it, z)));
    }
    Polygon_with_holes_2 ret(outer_boundary, holes.begin(), holes.end());

    if (ret.outer_boundary().size() > 0 && !ret.outer_boundary().is_simple()) {
      stringstream ss;
      ss << "Offset polygon is not simple.  offset = " << offset 
         << "   Original polygon = " << pp(polygon) << "  Offset polygon = " << pp(ret.outer_boundary())
         << "\n\n***** This is probably failing because some of the points are collinear.  Try removing "
         << "collinear points in the contours first.  This can be done using the -e option.";
      throw runtime_error(ss.str());
    }
    
    *out++ = ret;
  }

  // return ret;
}

Polygon_with_holes_2 offset_polygon_positive(const Polygon_2& polygon, Number_type offset)
{
  vector<Polygon_with_holes_2> polys;
  offset_polygon(polygon, offset, back_inserter(polys));
  if (polys.size() > 1) throw logic_error("unexpected number of polygons");
  return polys[0];
}

template <typename Out_iter>
void offset_polygon_negative(const Polygon_2& polygon, Number_type offset, Out_iter out)
{
  offset_polygon(polygon, offset, out);

  // // testing...
  // vector<Polygon_with_holes_2> polys;
  // offset_polygon(polygon, offset, back_inserter(polys));
  // if (!polys.empty())
  //   *out++ = polys[0];
}

template
void offset_polygon_negative(const Polygon_2& polygon, Number_type offset, 
                             back_insert_iterator<list<Polygon_with_holes_2> > out);

template
void offset_polygon_negative(const Polygon_2& polygon, Number_type offset, 
                             back_insert_iterator<vector<Polygon_with_holes_2> > out);

// //------------------------------------------------------------
// // is_simple
// //------------------------------------------------------------
// bool is_simple(const Polygon_with_holes_2& polygon)
// {
//   if (!polygon.outer_boundary().is_simple()) return false;
//   for (Polygon_with_holes_2::Hole_const_iterator it = polygon.holes_begin(); it != polygon.holes_end(); ++it) {
//     if (!it->is_simple()) return false;
//   }
//   return true;
// }

// //------------------------------------------------------------
// // is_simple
// //------------------------------------------------------------
// template <typename Iter>
// bool is_simple(Iter begin, Iter end)
// {
//   for (Iter it = begin; it != end; ++it) {
//     if (!is_simple(*it)) return false;
//   }
//   return true;
// }

// //------------------------------------------------------------
// // offset_polygon_impl
// //
// // Outputs polygon_with_holes_2 objects
// //------------------------------------------------------------
// template <typename Out_iter>
// void offset_polygon_impl(const Polygon_with_holes_2& polygon, Number_type offset, Out_iter out) {
//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("offset_polygon");

//   // if (polygon.outer_boundary().size() < 3 || !polygon.outer_boundary().is_simple()) {
//   if (!is_simple(polygon)) {
//     stringstream ss;
//     ss << "offset_polygon: input polygon is not simple:"
//        << "  polygon = " << pp(polygon) ;
//     throw runtime_error(ss.str());
//   }

//   typedef CGAL::Polygon_2<Offset_kernel> Offset_polygon_2;
//   typedef CGAL::Polygon_with_holes_2<Offset_kernel> Offset_polygon_with_holes_2;
//   typedef boost::shared_ptr<Offset_polygon_with_holes_2> Pwh_ptr;
//   typedef std::vector<Pwh_ptr> Pwh_ptr_vector;
//   typedef boost::shared_ptr<Offset_polygon_2> P_ptr;
//   typedef std::vector<P_ptr> P_ptr_vector;

//   Number_type z = polygon.outer_boundary()[0].z();
//   Offset_polygon_with_holes_2 in_poly(change_kernel<Offset_kernel>(polygon));

//   Pwh_ptr_vector outer_offset_polygons;
//   Offset_polygon_with_holes_2 op;

//   // Erosion
//   if (offset < 0) {
//     outer_offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(-offset, in_poly);
//     // LOG4CPLUS_DEBUG(logger, outer_offset_polygons.size());
//   }
//   // Dilation
//   else {
//     P_ptr_vector polys = CGAL::create_exterior_skeleton_and_offset_polygons_2(offset, in_poly.outer_boundary());

//     // Discard the first polygon (the exterior frame or new enclosing polygon)
//     polys.erase(polys.begin());

//     // Reverse the orientations (they are backwards because CGAL creates an exterior frame and treats
//     // the original polygons as holes, then inner-offsets.  We are responsible for removing the
//     // temporary frame and reversing orientations
//     for (P_ptr_vector::iterator it = polys.begin(); it != polys.end(); ++it) {
//       if ((*it)->size() > 0 && !(*it)->is_simple())
// 	LOG4CPLUS_ERROR(logger, "Offset polygon is not simple before reversing orientation.");
//       (*it)->reverse_orientation();
//     }

//     // Now erode each hole
//     for (Offset_polygon_with_holes_2::Hole_iterator hit = in_poly.holes_begin(); hit != in_poly.holes_end(); ++hit) {
//       Offset_polygon_2 hole = *hit;
//       hole.reverse_orientation();
//       P_ptr_vector new_holes = CGAL::create_interior_skeleton_and_offset_polygons_2(-offset, hole);
//       for (P_ptr_vector::iterator nhit = new_holes.begin(); nhit != new_holes.end(); ++nhit) {
// 	(*nhit)->reverse_orientation();
// 	polys.push_back(*nhit);
//       }
//     }

//     // Arrange the polygons as a polygon with holes
//     outer_offset_polygons = CGAL::arrange_offset_polygons_2(polys);
//   }

//   for (Pwh_ptr_vector::const_iterator it = outer_offset_polygons.begin(); it != outer_offset_polygons.end(); ++it) {
//     *out++ = fix_simple(to_common(**it, z));
//   }
// }

// //------------------------------------------------------------
// // offset_polygon
// //------------------------------------------------------------
// template <typename Out_iter>
// void offset_polygon(const Polygon_with_holes_2& polygon, Number_type offset, Out_iter out) {
//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("offset_polygon");

//   list<Polygon_with_holes_2> temp;
//   offset_polygon_impl(polygon, offset, back_inserter(temp));
//   const int tries = 3;
//   int i = 0;
//   for (; i < tries && !is_simple(temp.begin(), temp.end()); ++i);
//   if (i > 1) {
//     LOG4CPLUS_DEBUG(logger, "Offsetting polygon by " << offset << " required " << i << " attempts");
//   }

//   if (!is_simple(temp.begin(), temp.end())) {
//     stringstream ss;
//     ss << "offset_polygon: failed to create a simple offset polygon after " << tries << " tries:"
//        << "  polygon = " << pp(polygon) ;
//     throw runtime_error(ss.str());
//   }

//   for (list<Polygon_with_holes_2>::iterator it = temp.begin(); it != temp.end(); ++it) {
//     *out++ = *it;
//   }
// }

// template void offset_polygon(const Polygon_with_holes_2& polygon, Number_type offset, back_insert_iterator<list<Polygon_with_holes_2> > out);

CONTOURTILER_END_NAMESPACE
