// #include <ContourTiler/Slice2.h>
// #include <ContourTiler/remove_contour_intersections.h>
// #include <ContourTiler/print_utils.h>
// #include <ContourTiler/polygon_utils.h>
// #include <ContourTiler/Distance_functor.h>

// #include <sstream>

// CONTOURTILER_BEGIN_NAMESPACE

// void remove_collinear(Slice2& slice, Number_type epsilon)
// {
//   for (Slice2::iterator it = slice.begin(); it != slice.end(); ++it) {
//     Contour2_handle contour = it->second;

//     for (Contour2::Polygon_iterator pit = contour->begin(); pit != contour->end(); ++pit) {
//       Polygon_with_holes_2& pwh = *pit;
//       Polygon_with_holes_2 new_pwh(CONTOURTILER_NAMESPACE::remove_collinear(pwh.outer_boundary(), epsilon));
//       for (Polygon_with_holes_2::Hole_iterator hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
//       	Polygon_2& p = *hit;
//       	new_pwh.add_hole(CONTOURTILER_NAMESPACE::remove_collinear(p, epsilon));
//       }
//       pwh = new_pwh;
//     }
//   }
// }

// void remove_intersections(Slice2& slice, Number_type delta)
// {
//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("slice.remove_intersections");

//   // Put all polygons in the slice into polys, regardless of component.
//   // vector<Contour_handle> contours;
//   // list<Contour_handle> nested;

//   // // Add all contours to a hierarchy
//   // list<Contour_handle> all_contours;
//   // for (Slice2::iterator it = slice.begin(); it != slice.end(); ++it) {
//   //   Contour_container& comp_contours = it->second;
//   //   for (Contour_iterator c_it = comp_contours.begin(); 
//   // 	 c_it != comp_contours.end(); 
//   // 	 ++c_it)
//   //   {
//   //     Contour_handle c = *c_it;
//   //     all_contours.push_back(c);
//   //   }
//   // }

//   // for (Slice2::iterator it = slice.begin(); it != slice.end(); ++it) {
//   //   Contour_container& comp_contours = it->second;
//   //   for (Contour_iterator c_it = comp_contours.begin(); 
//   // 	 c_it != comp_contours.end(); 
//   // 	 ++c_it)
//   //   {
//   //     Contour_handle c = *c_it;
//   //     if (!c->polygon().is_simple()) {
//   // 	throw logic_error("Contour is not simple before intersection removal...");
//   //     }
//   //     contours.push_back(c);
//   //   }
//   // }

//   list<Contour2_handle> contours;
//   for (Slice2::iterator it = slice.begin(); it != slice.end(); ++it) {
//     contours.push_back(it->second);
//   }

//   // Remove all contour intersections by dilating, removing
//   // intersections, then eroding.
//   list<Contour2_handle> new_contours;
//   list<string> failures;
//   remove_contour_intersections2(contours.begin(), contours.end(), delta, back_inserter(new_contours), back_inserter(failures));

//   for (list<string>::const_iterator it = failures.begin(); it != failures.end(); ++it) {
//     LOG4CPLUS_WARN(logger, "Failed to remove contour intersections: " << *it);
//   }

//   slice.clear();
//   for (list<Contour2_handle>::iterator it = new_contours.begin(); it != new_contours.end(); ++it) {
//     slice[(*it)->info().name()] = *it;
//   }
//   // insert(new_contours.begin(), new_contours.end());
//   // insert(nested.begin(), nested.end());
// }

// void validate(const Slice2& slice)
// {
//   for (Slice2::const_iterator it = slice.begin(); it != slice.end(); ++it) {
//     it->second->validate();
//   }
// }

// // Number_type Slice::z() const
// // {
// //   for (Map::const_iterator it = _map.begin(); it != _map.end(); ++it)
// //   {
// //     const Contour_container& comp_contours = it->second;
// //     for (Contour_const_iterator c_it = comp_contours.begin(); 
// // 	 c_it != comp_contours.end(); 
// // 	 ++c_it)
// //     {
// //       Contour_handle c = *c_it;
// //       return c->slice();
// //     }
// //   }
// // }

// // void Slice::augment(const boost::unordered_map<Segment_3_undirected, list<Point_3> >& edge2points)
// // {
// //   for (Map::const_iterator it = _map.begin(); it != _map.end(); ++it)
// //   {
// //     const Contour_container& comp_contours = it->second;
// //     for (Contour_const_iterator c_it = comp_contours.begin(); 
// // 	 c_it != comp_contours.end(); 
// // 	 ++c_it)
// //     {
// //       Contour_handle c = *c_it;
// //       Polygon_2& p = c->polygon();
// //       Polygon_2 newp;
// //       Polygon_2::Edge_const_circulator begin = p.edges_circulator();
// //       Polygon_2::Edge_const_circulator ci = begin;
// //       do {
// // 	Segment_2 seg = *ci;
// // 	Segment_3_undirected useg(Segment_3(seg.source(), seg.target()));
// // 	if (edge2points.find(useg) == edge2points.end()) {
// // 	  newp.push_back(seg.source());
// // 	}
// // 	else {
// // 	  // There are points on this edge that need to be augmented.
// // 	  list<Point_3> points = edge2points.find(useg)->second;
// // 	  points.sort(Distance_functor<Point_3>(seg.source()));
// // 	  points.push_front(seg.source());
// // 	  list<Point_3>::iterator new_end = unique(points.begin(), points.end());
// // 	  new_end = remove(points.begin(), new_end, seg.target());
// // 	  for (list<Point_3>::iterator p_it = points.begin(); p_it != new_end; ++p_it) {
// // 	    newp.push_back(*p_it);
// // 	  }
// // 	}
// // 	++ci;
// //       } while (ci != begin);
// //       p = newp;
// //     }
// //   }
// // }

// // string Slice::to_string() const
// // {
// //   stringstream ss;
// //   ss << "z = " << z() << "; ";
// //   for (Map::const_iterator it = _map.begin(); it != _map.end(); ++it)
// //   {
// //     const string& component = it->first;
// //     ss << component << ": ";
// //     const Contour_container& comp_contours = it->second;
// //     for (Contour_const_iterator c_it = comp_contours.begin(); 
// // 	 c_it != comp_contours.end(); 
// // 	 ++c_it)
// //     {
// //       Contour_handle c = *c_it;
// //       ss << pp(c->polygon()) << " ";
// //     }
// //     ss << "; ";
// //   }
// //   return ss.str();
// // }

// CONTOURTILER_END_NAMESPACE

