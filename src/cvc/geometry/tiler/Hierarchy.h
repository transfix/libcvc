#ifndef __HIERARCHY_H__
#define __HIERARCHY_H__

#include <iostream>
#include <vector>
#include <list>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/unordered_map.hpp>

#include <CGAL/Gmpz.h>
#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>

#include <ContourTiler/polygon_utils.h>
#include <ContourTiler/Contour.h>
#include <ContourTiler/contour_utils.h>
//#include "print_utils.h"

CONTOURTILER_BEGIN_NAMESPACE

std::string pp(const Point_2& point);
std::string pp(const Polygon_2& poly);

/// This class represents the hierarchical relationships between contours on
/// a single slice.
class Hierarchy
{
private:
  typedef boost::weak_ptr<Contour>                      Child_handle;
  typedef boost::weak_ptr<Contour>                      Parent_handle;
  typedef std::list<Child_handle>                       Children_container;
  typedef boost::unordered_map<Contour_handle, Children_container>  Children_map;
  typedef boost::unordered_map<Contour_handle, Parent_handle>       Parent_map;
  typedef Children_container::iterator                  iterator;
  typedef Children_container::const_iterator            const_iterator;

public:
  Hierarchy() {}

  template <typename ContourIterator>
  Hierarchy(ContourIterator start, ContourIterator end, Hierarchy_policy policy);

  ~Hierarchy() {}

public:

  //--------------------------------------------------------
  // Accessors
  //--------------------------------------------------------

  /// Throws if the given contour is not a part of this hierarchy
  Contour_handle parent(Contour_handle contour);

  /// Throws if the given contour is not a part of this hierarchy
  const Contour_handle parent(Contour_handle contour) const;

  /// Throws if the given contour is not a part of this hierarchy
  iterator children_begin(Contour_handle contour);

  /// Throws if the given contour is not a part of this hierarchy
  iterator children_end(Contour_handle contour);

  //--------------------------------------------------------
  // Accessors that use terminology used in the Bajaj96 paper
  //--------------------------------------------------------
  
  /// Throws if the given contour is not a part of this hierarchy
  Contour_handle NEC(Contour_handle contour);

  /// Throws if the given contour is not a part of this hierarchy
  const Contour_handle NEC(Contour_handle contour) const;

  Contour_handle NEC(Point_2 point);

  const Contour_handle NEC(Point_2 point) const;

  /// Infers the orientation from the hierarchy of the contour
  CGAL::Orientation orientation(Contour_handle contour) const;

  /// Infers the orientation from the hierarchy of the contour
  bool is_CCW(const Contour_handle& contour) const;

  /// Infers the orientation from the hierarchy of the contour
  bool is_CW(const Contour_handle& contour) const;

  boost::tuple<Contour_handle, Contour_handle, Polygon_2::Vertex_circulator> is_overlapping(const Point_2& point) const;

  boost::tuple<Vertex_sign, Contour_handle> vertex_sign(const Point_2& point, CGAL::Orientation point_orientation) const;

private:

  //--------------------------------------------------------
  // Private utility methods
  //--------------------------------------------------------

  Contour_handle is_overlapping(const Point_2& point, Contour_handle nec) const;

  int level(Contour_handle contour) const;

  /// Throws if the given contour is not a part of this hierarchy
  void assert_member(Contour_handle contour) const;

  /// Finds <tt>contour</tt>'s parent given another contour <tt>ancestor</tt>
  /// that is guaranteed to contain <tt>contour</tt>.
  const Contour_handle find_parent(Contour_handle contour, Contour_handle ancestor) const;

  const Contour_handle find_parent(const Point_2& point, Contour_handle ancestor) const;

  void add_child(Contour_handle child, Contour_handle parent);

  /// Finds all children of the ancestor contour that are also children
  /// of the given contour.
  template <typename OutputIterator>
  void find_children(Contour_handle contour, Contour_handle ancestor, OutputIterator children)
  {
    for (const_iterator it = children_begin(ancestor); it != children_end(ancestor); ++it)
    {
      Contour_handle child(*it);
      const Point_2& child_point = *(child->polygon().vertices_begin());
      if (contour->polygon().has_on_positive_side(child_point))
      {
	*children = *it;
	++children;
      }
    }
  }

  void force_orientations();

  void force_orientations(Contour_handle contour, int level);

  bool check_orientations();

  bool check_orientations(Contour_handle contour, int level);

  bool is_even(int level) const;

private:
  Contour_handle _root;
  Children_map _children;
  Parent_map _parents;
};

typedef std::map<Number_type, Hierarchy> Hierarchies;

CONTOURTILER_END_NAMESPACE

#endif
