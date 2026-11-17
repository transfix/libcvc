#include <ContourTiler/remove_contour_intersections.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/sweep_line_visitors.h>
#include <ContourTiler/offset_polygon.h>
#include <ContourTiler/polygon_difference.h>
#include <ContourTiler/Contour_info.h>
#include <ContourTiler/contour_utils.h>
#include <ContourTiler/Contour2.h>

#include <deque>

#include <boost/foreach.hpp>

CONTOURTILER_BEGIN_NAMESPACE

// Takes an original contour and finds all derivative contours resulting from
// the difference operation on the original contour.
//
// OutputIterator iterates over Contour_handles.
template <typename OutputIterator, typename Contour_handle>
void get_current_contours(const boost::unordered_map<Contour_handle, list<Contour_handle> >& changing, 
			  Contour_handle orig, OutputIterator current)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("get_current_contours");

  if (changing.find(orig) != changing.end()) {
    if (orig->info().name() == "a100") {
      LOG4CPLUS_TRACE(logger, "final a100 - changing");
    }

    for (const auto& c : changing.find(orig)->second) {
      get_current_contours(changing, c, current);
    }
    // const list<Contour_handle>& new_contours = changing.find(orig)->second;
    // for (typename list<Contour_handle>::const_iterator it = new_contours.begin(); it != new_contours.end(); ++it) {
    //   get_current_contours(changing, *it, current);
    // }
  }
  else {
    if (orig) {
      *current++ = orig;

      if (orig->info().name() == "a100") {
        LOG4CPLUS_TRACE(logger, "final a100 - final");
      }
    }
  }
}

template <typename Out_iter>
void new_contours(const Polygon_2& p, const Contour::Info& info, Out_iter out)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("new_contours");

  if (p.size() < 3) {
    LOG4CPLUS_WARN(logger, "Non-simple polygon: " << pp(p) << " - using empty contour");
    return;
  }

  list<Polygon_2> polygons;
  if (!p.is_simple()) {
    split_nonsimple(p, back_inserter(polygons));

    // debug
    if (info.name() == "a100") {
      // LOG4CPLUS_TRACE(logger, "a100 contour: " << pp(p));
      LOG4CPLUS_TRACE(logger, "a100 splitting non-simple");
      // for (const auto& sp : polygons) {
      //   // LOG4CPLUS_TRACE(logger, "  result: " << pp(sp));
      //   LOG4CPLUS_TRACE(logger, "  result: " << pp(sp));
      // }
    }
  }
  else {
    if (info.name() == "a100") {
      LOG4CPLUS_TRACE(logger, "a100 simple");
    }
    polygons.push_back(p);
  }

  for (const auto& subp : polygons) {
    Contour_handle newc = Contour::create(subp, info);
    *out++ = newc;

    // debug
    if (info.name() == "a100") {
      LOG4CPLUS_TRACE(logger, "  adding " << newc->size() << " (" << newc.get() << ")");
    }
  }
}

template <typename Out_iter>
void diffed_contours(const list<Polygon_with_holes_2>& polygons, const Contour::Info& info, 
                     log4cplus::Logger& logger, Out_iter out)
{
  for (const auto& pwh : polygons) {
    new_contours(pwh.outer_boundary(), info, out);
    for (auto hole_it = pwh.holes_begin(); hole_it != pwh.holes_end(); ++hole_it) {
      Polygon_2 hole = *hole_it;
      hole.reverse_orientation();
      new_contours(hole, info, out);
    }
  }
}

// Takes two lists of contours.  Each contour in one list will be differenced with
// each contour in the other list.  The changing map will be updated accordingly.
template <typename InputIterator>
void remove_differences(InputIterator pairs_begin, InputIterator pairs_end,
			boost::unordered_map<Contour_handle, list<Contour_handle> >& changing)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("remove_differences");

  typedef pair<Contour_handle, Contour_handle> Pair;
  deque<Pair> pairs(pairs_begin, pairs_end);
  while (!pairs.empty()) {
    Pair cpair = pairs.front();
    pairs.pop_front();
    Contour_handle p = cpair.first;
    Contour_handle q = cpair.second;

    if (!p || !q) continue;

    if (q->info().name() == "a100") swap(p,q);

    // Check to see if p or q has been modified since this pair was enqueued.
    if (changing.find(p) != changing.end()) {
      for (const auto& c : changing.find(p)->second) {
	pairs.push_back(make_pair(c, q));
      }

      // debug
      if (p->info().name() == "a100") {
        list<Contour_handle> a100;
        get_current_contours(changing, p, back_inserter(a100));
        LOG4CPLUS_TRACE(logger, "a100 " << p.get() << " children");
        for (const auto& c : a100) {
          LOG4CPLUS_TRACE(logger, "  size = " << c->size() << " (" << c.get() << ")");
        }
      }
    }
    else if (changing.find(q) != changing.end()) {
      for (const auto& c : changing.find(q)->second) {
	pairs.push_back(make_pair(p, c));
      }
    }
    else {
      if (p->info().name() == "a100") {
        LOG4CPLUS_TRACE(logger, p->info().name() << " - " << q->info().name() << " (" << p.get() << ")");
      }

      // Go ahead and do difference - p nor q has changed.
      list<Polygon_with_holes_2> pdiff, qdiff;
      polygon_difference(p->polygon(), q->polygon(), back_inserter(pdiff), back_inserter(qdiff));
      
      if (p->info().name() == "a100") {
        LOG4CPLUS_TRACE(logger, "  p -> " << pdiff.size());
        if (pdiff.size() == 3) {
          LOG4CPLUS_TRACE(logger, "  origp = " << pp(p->polygon()));
          LOG4CPLUS_TRACE(logger, "  origq = " << pp(q->polygon()));
          for (const auto& pwh : pdiff) {
            LOG4CPLUS_TRACE(logger, "  " << pp(pwh.outer_boundary()));
          }
        }
      }

      diffed_contours(pdiff, p->info(), logger, back_inserter(changing[p]));
      diffed_contours(qdiff, q->info(), logger, back_inserter(changing[q]));
    }
  }
}


template <typename Contour_iter, typename Out_iter>
void remove_contour_intersections(Contour_iter begin, Contour_iter end, 
				  Number_type delta, 
				  Out_iter out)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("slice.remove_contour_intersections");

  Number_type delta2 = delta / 2.0;

  // This must be a list because we'll be inserting and deleting a lot
  list<Contour_handle> contours;
  for (Contour_iter it = begin; it != end; ++it) {
    if (!(*it)->is_counterclockwise_oriented()) {
      throw logic_error("Polygon is not counter clockwise!");
    }
    Contour_handle c = (*it)->copy();
    try {
      LOG4CPLUS_TRACE(logger, "Dilating: " << pp(c->polygon()));
      Polygon_with_holes_2 polys = offset_polygon_positive(c->polygon(), delta2);
      for (Polygon_with_holes_2::Hole_const_iterator hit = polys.holes_begin(); hit != polys.holes_end(); ++hit) {
	contours.push_back(Contour::create(*hit, c->info()));
      }
      c->polygon() = polys.outer_boundary();
      LOG4CPLUS_TRACE(logger, "Result: " << pp(c->polygon()));
    }
    catch (logic_error& e) {
      throw logic_error("Component = " + c->info().object_name() + "  " + e.what());
    }
    contours.push_back(c);
  }

  typedef list<Contour_handle>::iterator Iter;

  // This is a map with all contours that have been or will be changed from the difference
  // operation.  The key is the original contour.  The value is a list of new contours.
  // Keep in mind that the new contours may also be changed, so the final contour is to
  // be found iteratively.  See get_current_contour().
  boost::unordered_map<Contour_handle, list<Contour_handle> > changing;
  
  list<pair<Contour_handle, Contour_handle> > pairs;
  get_intersecting_pairs(contours.begin(), contours.end(), back_inserter(pairs));
  remove_differences(pairs.begin(), pairs.end(), changing);

  list<Contour_handle> temp;
  for (const auto& c : contours) {
    get_current_contours(changing, c, back_inserter(temp));

    // debug
    if (c->info().name() == "a100") {
      list<Contour_handle> a100;
      get_current_contours(changing, c, back_inserter(a100));
      LOG4CPLUS_TRACE(logger, "Final a100 contours");
      for (const auto& c100 : a100) {
        LOG4CPLUS_TRACE(logger, "  " << pp(c100->polygon()));
      }
    }
  }
  // for (Iter it = contours.begin(); it != contours.end(); ++it) {
  //   get_current_contours(changing, *it, back_inserter(temp));
  // }
  contours = temp;

  // This check doesn't work since we now have end-end intersections
  // pairs.clear();
  // get_intersecting_pairs(contours.begin(), contours.end(), back_inserter(pairs));
  // if (!pairs.empty()) {
  //   throw logic_error("Didn't remove intersections!");
  // }

  // for (Iter p = contours.begin(); p != contours.end(); ++p) {
  //   Contour_handle c = *p;
  for (const auto& c : contours) {
    LOG4CPLUS_TRACE(logger, "Eroding: " << pp(c->polygon()));

    list<Polygon_with_holes_2> polys;
    try {
      offset_polygon_negative(c->polygon(), -delta2, back_inserter(polys));
    }
    catch (logic_error& e) {
      LOG4CPLUS_ERROR(logger, "Discarded polygon in contour " << c->info().name());
      throw e;
    }
    for (const auto& pwh : polys) {
      if (pwh.holes_begin() != pwh.holes_end()) {
        LOG4CPLUS_ERROR(logger, "Can't yet handle polygons with holes when eroding: " << pp(c->polygon()));
        // return;
      }
      const Polygon_2 p = pwh.outer_boundary();
      // c->polygon() = pwh.outer_boundary();
      LOG4CPLUS_TRACE(logger, "Result: " << pp(p));
      if (p.size() > 0) {
        if (!p.is_simple()) {
          LOG4CPLUS_ERROR(logger, "Polygon is not simple after intersection removal -- skipping");
          LOG4CPLUS_ERROR(logger, "  Component = " << c->info().object_name());
          LOG4CPLUS_ERROR(logger, "  Polygon = " << pp(p));
          // throw logic_error("Polygon is not simple!");
        }
        else {
          *out++ = Contour::create(p, c->info());
          // *out++ = c;
        }
      }
    }
  }
}

template
void remove_contour_intersections(vector<Contour_handle>::iterator begin, 
				  vector<Contour_handle>::iterator end, 
				  Number_type delta,
				  back_insert_iterator<list<Contour_handle> > out);

template
void remove_contour_intersections(vector<Contour_handle>::iterator begin, 
				  vector<Contour_handle>::iterator end, 
				  Number_type delta,
				  back_insert_iterator<vector<Contour_handle> > out);

template
void remove_contour_intersections(list<Contour_handle>::iterator begin, 
				  list<Contour_handle>::iterator end, 
				  Number_type delta,
				  back_insert_iterator<list<Contour_handle> > out);

template
void remove_contour_intersections(list<Contour_handle>::iterator begin, 
				  list<Contour_handle>::iterator end, 
				  Number_type delta,
				  back_insert_iterator<vector<Contour_handle> > out);


//----------------------------------------------------------------------
// New stuff

class Polygon_ext
{
public:
  Polygon_ext(const Polygon_with_holes_2& p, const Contour2::Info& info) : _p(p), _info(info) {}
  
  const Polygon_with_holes_2& polygon() const { return _p; }
  Polygon_with_holes_2& polygon() { return _p; }
  const Contour2::Info& info() const { return _info; }

private:
  Polygon_with_holes_2 _p;
  Contour2::Contour2::Info _info;
};

typedef boost::shared_ptr<Polygon_ext> Polygon_ext_handle;

// Takes two lists of contours.  Each contour in one list will be differenced with
// each contour in the other list.  The changing map will be updated accordingly.
template <typename InputIterator>
void remove_differences2(InputIterator pairs_begin, InputIterator pairs_end,
			boost::unordered_map<Polygon_ext_handle, list<Polygon_ext_handle> >& changing)
{
  typedef pair<Polygon_ext_handle, Polygon_ext_handle> Pair;
  deque<Pair> pairs(pairs_begin, pairs_end);
  while (!pairs.empty()) {
    Pair cpair = pairs.front();
    pairs.pop_front();
    const Polygon_ext_handle p = cpair.first;
    const Polygon_ext_handle q = cpair.second;

    // Check to see if p or q has been modified since this pair was enqueued.
    if (changing.find(p) != changing.end()) {
      const list<Polygon_ext_handle>& new_contours = changing.find(p)->second;
      for (list<Polygon_ext_handle>::const_iterator it = new_contours.begin(); it != new_contours.end(); ++it) {
	pairs.push_back(make_pair(*it, q));
      }
    }
    else if (changing.find(q) != changing.end()) {
      const list<Polygon_ext_handle>& new_contours = changing.find(q)->second;
      for (list<Polygon_ext_handle>::const_iterator it = new_contours.begin(); it != new_contours.end(); ++it) {
	pairs.push_back(make_pair(p, *it));
      }
    }
    else {
      // Go ahead and do difference - p nor q has changed.
      list<Polygon_with_holes_2> pdiff, qdiff;
      polygon_difference(p->polygon(), q->polygon(), back_inserter(pdiff), back_inserter(qdiff));

      for (list<Polygon_with_holes_2>::iterator pit = pdiff.begin(); pit != pdiff.end(); ++pit) {
	changing[p].push_back(Polygon_ext_handle(new Polygon_ext(*pit, p->info())));
      }
      for (list<Polygon_with_holes_2>::iterator qit = qdiff.begin(); qit != qdiff.end(); ++qit) {
	changing[q].push_back(Polygon_ext_handle(new Polygon_ext(*qit, q->info())));
      }
    }
  }
}

/// InputIterator iterates over Contour_handle objects.
/// OutputIterator iterates over std::pair<Contour_handle, Contour_handle> objects.
/// @param intersecting_pairs pairs of contours that intersect each other
template <typename InputIterator, typename OutputIterator>
void get_intersecting_polygon_pairs(InputIterator begin, InputIterator end, OutputIterator intersecting_pairs)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("get_intersecting_polygon_pairs");

  boost::unordered_map<Segment_2, Polygon_ext_handle> seg2Polygon;
  vector<Polygon_2> polygons;
  for (InputIterator it = begin; it != end; ++it) {
    Polygon_ext_handle ph = *it;

    // Add outer boundary to master list of polygons
    const Polygon_2& polygon = ph->polygon().outer_boundary();
    for (Polygon_2::Edge_const_iterator eit = polygon.edges_begin(); eit != polygon.edges_end(); ++eit) {
      seg2Polygon[*eit] = ph;
    }
    polygons.push_back(polygon);

    // Add holes to master list of polygons
    for (Polygon_with_holes_2::Hole_iterator hit = ph->polygon().holes_begin(); hit != ph->polygon().holes_end(); ++hit) {
      const Polygon_2& hole = *hit;
      for (Polygon_2::Edge_const_iterator eit = hole.edges_begin(); eit != hole.edges_end(); ++eit) {
	seg2Polygon[*eit] = ph;
      }
      polygons.push_back(hole);
    }
  }
  
  // Perform sweep line to get intersections
  list<SL_intersection> ints;
  typedef Intersection_visitor<back_insert_iterator<list<SL_intersection> > > Visitor;
  sweep_line_multi(polygons, Visitor(back_inserter(ints)));

  // Go through intersections and find which polygons intersect
  typedef boost::unordered_set<pair<Polygon_ext_handle, Polygon_ext_handle> > Pair_set;
  Pair_set unique_pairs;
  for (list<SL_intersection>::iterator it = ints.begin(); it != ints.end(); ++it) {
    const list<Segment_2>& ints = it->interiors();
    const list<Segment_2>& ends = it->ends();
    list<Segment_2> segs(ints.begin(), ints.end());
    segs.insert(segs.end(), ends.begin(), ends.end());
    if (segs.size() != 2) {
      throw logic_error("Unexpected number of segments in intersection");
    }
    Polygon_ext_handle p = seg2Polygon[*segs.begin()];
    Polygon_ext_handle q = seg2Polygon[*segs.rbegin()];
    if (p.get() > q.get()) {
      swap(p, q);
    }
    unique_pairs.insert(make_pair(p, q));
  }
  for (Pair_set::const_iterator it = unique_pairs.begin(); it != unique_pairs.end(); ++it) {
    *intersecting_pairs++ = *it;
  }
}

// template <typename Contour_iter, typename Out_iter, typename Failure_iter>
// void remove_contour_intersections2(Contour_iter begin, Contour_iter end, 
// 				   Number_type delta, 
// 				   Out_iter out,
// 				   Failure_iter failures) {
//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("slice.remove_contour_intersections");

//   Number_type delta2 = delta / 2.0;

//   // Put all contour polygons into a master list of polygons with holes.
//   // This is also where the polygons are dilated.
//   list<Polygon_ext_handle> polygons;
//   for (Contour_iter it = begin; it != end; ++it) {
//     for (Contour2::Polygon_iterator pit = (*it)->begin(); pit != (*it)->end(); ++pit) {
//       list<Polygon_with_holes_2> pwh;
//       try {
// 	offset_polygon(*pit, delta2, back_inserter(pwh));;
//       }
//       catch (exception& e) {
// 	*failures++ = "Component = " + (*it)->info().object_name() + "  " + e.what();
//       }
//       for (list<Polygon_with_holes_2>::iterator pwhit = pwh.begin(); pwhit != pwh.end(); ++pwhit) {
// 	Polygon_ext_handle ph(new Polygon_ext(*pwhit, (*it)->info()));
// 	polygons.push_back(ph);
//       }
//     }
//   }

//   typedef list<Polygon_ext_handle>::iterator Iter;

//   // This is a map with all contours that have been or will be changed from the difference
//   // operation.  The key is the original contour.  The value is a list of new contours.
//   // Keep in mind that the new contours may also be changed, so the final contour is to
//   // be found iteratively.  See get_current_contour().
//   boost::unordered_map<Polygon_ext_handle, list<Polygon_ext_handle> > changing;
  
//   list<pair<Polygon_ext_handle, Polygon_ext_handle> > pairs;
//   get_intersecting_polygon_pairs(polygons.begin(), polygons.end(), back_inserter(pairs));
//   remove_differences2(pairs.begin(), pairs.end(), changing);

//   // Update our list after removing differences.
//   list<Polygon_ext_handle> temp;
//   for (Iter it = polygons.begin(); it != polygons.end(); ++it) {
//     get_current_contours(changing, *it, back_inserter(temp));
//   }
//   polygons = temp;

//   // Now erode
//   typedef boost::unordered_map<Contour2::Info, list<Polygon_with_holes_2> > Info2Pwhs;
//   Info2Pwhs map;
//   for (Iter p = polygons.begin(); p != polygons.end(); ++p) {
//     Polygon_ext_handle ph = *p;
//     map[ph->info()]; // make sure we have an entry
//     try {
//       list<Polygon_with_holes_2> pwh;
//       offset_polygon(ph->polygon(), -delta2, back_inserter(pwh));;
//       for (list<Polygon_with_holes_2>::iterator pwhit = pwh.begin(); pwhit != pwh.end(); ++pwhit) {
// 	map[ph->info()].push_back(*pwhit);
//       }
//     }
//     catch (exception& e) {
//       *failures++ = "Component = " + ph->info().object_name() + "  " + e.what();
//       // throw logic_error("Component = " + ph->info().object_name() + "  " + e.what());
//     }
//   }

//   for (Info2Pwhs::iterator it = map.begin(); it != map.end(); ++it) {
//     *out++ = Contour2_handle(Contour2::create_from_pwhs(it->second.begin(), it->second.end(), it->first));
//   }
//     //   Polygon_with_holes_2 polys = offset_polygon(c->polygon(), -delta2);

//     //   c->polygon() = polys.outer_boundary();
//     //   LOG4CPLUS_TRACE(logger, "Result: " << pp(c->polygon()));
//     // }
//     // catch (logic_error& e) {
//     //   throw logic_error("Component = " + c->info().object_name() + "  " + e.what());
//     // }
//     // if (c->size() > 0) {
//     //   if (!c->polygon().is_simple()) {
//     // 	LOG4CPLUS_ERROR(logger, "Polygon is not simple after intersection removal.  Component = " << c->info().object_name() 
//     // 			<< " Polygon = " << pp(c->polygon()));
//     // 	throw logic_error("Polygon is not simple!");
//     //   }
//     //   *out++ = c;
//     // }
//   // }
// }

// template void remove_contour_intersections2(list<Contour2_handle>::iterator begin, list<Contour2_handle>::iterator end, 
// 					    Number_type delta, 
// 					    back_insert_iterator<list<Contour2_handle> > out,
// 					    back_insert_iterator<list<string> > failures);


CONTOURTILER_END_NAMESPACE

