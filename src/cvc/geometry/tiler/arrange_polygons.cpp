#include <ContourTiler/arrange_polygons.h>
// #include "print_utils.h"
// #include "polygon_utils.h"
// #include "Contour.h"

// #include <iostream>
// #include <vector>
// #include <list>
// #include <stdexcept>

// #include <boost/shared_ptr.hpp>
// #include <boost/weak_ptr.hpp>
// #include <boost/tuple/tuple.hpp>
// #include <boost/unordered_map.hpp>

// #include <CGAL/Gmpz.h>
// #include <CGAL/Lazy_exact_nt.h>
// #include <CGAL/Cartesian.h>
// #include <CGAL/Polygon_2.h>
// #include <CGAL/Polygon_with_holes_2.h>
// #include <CGAL/Boolean_set_operations_2.h>

// CONTOURTILER_BEGIN_NAMESPACE

// typedef boost::shared_ptr<Polygon_2> Polygon_handle;
// typedef boost::weak_ptr<Polygon_2>                      Child_handle;
// typedef boost::weak_ptr<Polygon_2>                      Parent_handle;
// typedef std::list<Child_handle>                       Children_container;
// typedef boost::unordered_map<Polygon_handle, Children_container>  Children_map;
// typedef boost::unordered_map<Polygon_handle, Parent_handle>       Parent_map;
// typedef Children_container::iterator                  iterator;
// typedef Children_container::const_iterator            const_iterator;
  
// const Polygon_handle find_parent(const Point_2& point, Polygon_handle ancestor, const Children_map& _children)
// {
//   Polygon_handle parent;
//   if (ancestor->has_on_positive_side(point))
//   {
//     // Loop through all children of the ancestor until we find a parent.
//     // This function relies on the fact that all polygons are either
//     // parent-child or sibling -- that there are no intersections of 
//     // polygon boundaries.
//     const Children_container& children = _children.find(ancestor)->second;
//     for (const_iterator it = children.begin(); 
// 	 !parent && it != children.end(); 
// 	 ++it)
//     {
//       Polygon_handle child(*it);
//       parent = find_parent(point, child, _children);
//     }
//     if (!parent)
//       parent = ancestor;
//   }
//   return parent;
// }

// /// Finds polygon's parent given another polygon ancestor
// /// that is guaranteed to contain polygon.
// const Polygon_handle find_parent(Polygon_handle polygon, Polygon_handle ancestor, const Children_map& _children)
// {
//   // Call find_parent with one of the vertices
//   Point_2 point = *(polygon->vertices_begin());
//   return find_parent(point, ancestor, _children);
// }

// /// All elements in children container that are children of polygon are added to filtered.
// template <typename OutputIterator>
// void filter_children(Polygon_handle polygon, const Children_container& children, OutputIterator filtered)
// {
//   for (const_iterator it = children.begin(); it != children.end(); ++it) {
//     Polygon_handle child(*it);
//     const Point_2& child_point = *(child->vertices_begin());
//     if (polygon->has_on_positive_side(child_point)) {
//       *filtered++ = *it;
//     }
//   }
// }

// void add_child(Polygon_handle child, Polygon_handle parent, Children_map& _children, Parent_map& _parents)
// { 
//   // Transfer any children of the parent that should 
//   // now be children of the child
//   Children_container lost_children;
//   // find_children(child, parent, std::back_inserter(lost_children), _children);
//   filter_children(child, _children[parent], std::back_inserter(lost_children));
//   Children_container& children = _children[parent];
//   for (iterator it = lost_children.begin(); it != lost_children.end(); ++it) {
//     Polygon_handle lost_child(*it);
//     for (iterator it2 = children.begin(); it2 != children.end(); ++it2) {
//       Polygon_handle tmp(*it2);
//       if (lost_child == tmp) {
// 	children.erase(it2);
// 	break;
//       }
//     }
//     // children.erase(find(children.begin(), children.end(), lost_child));
//     // Have to do this after child has been added
//     // add_child(Polygon_handle(*it), child);
//   }
//   _children[parent].push_back(child);
//   _parents[child] = parent;
//   // Make sure we have an entry for the children
//   _children[child];

//   for (iterator it = lost_children.begin(); it != lost_children.end(); ++it) {
//     add_child(Polygon_handle(*it), child, _children, _parents);
//   }
// }

// bool is_even(int level)
// { return (level % 2) == 0; }

// void force_orientations(Polygon_handle polygon, int level, Children_map& _children)
// { 
//   // A polygon at level 0 should be CCW, enclosing CW polygons at level 1.
//   // So: even = CCW; odd = CW
//   bool even = is_even(level);
//   if (polygon->is_counterclockwise_oriented() && !even)
//     polygon->reverse_orientation();
//   else if (polygon->is_clockwise_oriented() && even)
//     polygon->reverse_orientation();
//   for (iterator it = _children[polygon].begin(); it != _children[polygon].end(); ++it)
//   {
//     Polygon_handle child(*it);
//     force_orientations(child, level+1, _children);
//   }
// }

// void force_orientations(Polygon_handle root, Children_map& _children)
// { force_orientations(root, -1, _children); }

// template <class InputPolygonIterator, class OutputPolygonWithHolesIterator>
// void arrange_polygons(InputPolygonIterator begin,
// 		      InputPolygonIterator end,
// 		      OutputPolygonWithHolesIterator out)
// {
//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.arrange_polygons");

//   // Throw if any boundaries of any polygons intersect
//   list<Point_2> pts;
//   get_boundary_intersections<InputPolygonIterator, back_insert_iterator<list<Point_2> > >(begin, end, std::back_inserter(pts), false);
//   if (pts.size() > 0) {
//     LOG4CPLUS_ERROR(logger, "The polygons' boundaries intersect at these points: ");
//     for (list<Point_2>::const_iterator it = pts.begin(); it != pts.end(); ++it)
//       LOG4CPLUS_ERROR(logger, "  " <<  pp(*it));
//     LOG4CPLUS_ERROR(logger, "Polygons: ");
//     for (InputPolygonIterator it = begin; it != end; ++it)
//       LOG4CPLUS_ERROR(logger, "  " <<  pp((*it)));;
//     throw runtime_error("The polygons' boundaries intersect");
//   }

//   list<Polygon_handle> all;
//   for (InputPolygonIterator it = begin; it != end; ++it) {
//     Polygon_handle p(new Polygon_2(it->vertices_begin(), it->vertices_end()));
//     if (!p->is_counterclockwise_oriented()) {
//       p->reverse_orientation();
//     }
//     all.push_back(p);
//   }

//   // all now contains pointers to all polygons.  All polygons are CCW.

//   Polygon_2 super = super_polygon(begin, end);
//   Polygon_handle _root(new Polygon_2(super.vertices_begin(), super.vertices_end()));
//   Children_map _children;
//   Parent_map _parents;

//   // Make sure we have child and parent entries for root
//   _children[_root];
//   _parents[_root];

//   // Find all parent/child relationships
//   for (list<Polygon_handle>::iterator it = all.begin(); it != all.end(); ++it) {
//     Polygon_handle polygon = *it;
//     Polygon_handle parent = find_parent(polygon, _root, _children);
//     add_child(polygon, parent, _children, _parents);
//   }

//   force_orientations(_root, _children);

//   // Now build the polygons with holes
//   for (list<Polygon_handle>::iterator it = all.begin(); it != all.end(); ++it) {
//     Polygon_handle polygon = *it;
//     if (polygon->is_counterclockwise_oriented()) {
//       Polygon_with_holes_2 pwh(*polygon);

//       for (iterator it = _children[polygon].begin(); it != _children[polygon].end(); ++it) {
// 	Polygon_handle child(*it);
// 	pwh.add_hole(*child);
//       }

//       LOG4CPLUS_TRACE(logger, pp(pwh));
      
//       *out++ = pwh;
//     }
//   }
// }

// template void arrange_polygons(list<Polygon_2>::iterator begin,
// 			       list<Polygon_2>::iterator end,
// 			       back_insert_iterator<list<Polygon_with_holes_2> > out);
// // template void arrange_polygons(list<Polygon_2>::const_iterator begin,
// // 			       list<Polygon_2>::const_iterator end,
// // 			       back_insert_iterator<list<Polygon_with_holes_2> > out);
// template void arrange_polygons(vector<Polygon_2>::iterator begin,
// 			       vector<Polygon_2>::iterator end,
// 			       back_insert_iterator<list<Polygon_with_holes_2> > out);
// // template void arrange_polygons(vector<Polygon_2>::const_iterator begin,
// // 			       vector<Polygon_2>::const_iterator end,
// // 			       back_insert_iterator<list<Polygon_with_holes_2> > out);

// CONTOURTILER_END_NAMESPACE
