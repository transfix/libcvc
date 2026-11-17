#include <ContourTiler/Hierarchy.h>
#include <ContourTiler/print_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

// A contour given to this class will be forced to CCW upon instantiation
// of this object and will be restored to it's original orientation when
// this object goes out of scope or is destroyed.
class Temp_CCW
{
public:
  Temp_CCW(Contour_handle contour) : _contour(contour)
  {
    _CW = contour->is_clockwise_oriented();
    _contour->force_counterclockwise();
  }
  ~Temp_CCW()
  {
    if (_CW) _contour->force_clockwise();
  }
private:
  Contour_handle _contour;
  bool _CW;
};

template <typename ContourIterator>
Hierarchy::Hierarchy(ContourIterator start, ContourIterator end, Hierarchy_policy policy)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.Hierarchy");
  using namespace std;

  // Make a list of the original polygons
  list<Polygon_2> polygons;
  for (ContourIterator it = start; it != end; ++it)
  {
    const Polygon_2& p = (*it)->polygon();
    
    // Make sure the polygons are simple
    if (!p.is_simple())
    {
      set_pp_precision(12);
      LOG4CPLUS_ERROR(logger, "The polygon is not simple: " << pp(p));
      throw runtime_error("The polygon is not simple");
    }

    polygons.push_back(p);
  }

  // Throw if any boundaries of any polygons intersect
  list<Point_2> pts;
  CONTOURTILER_NAMESPACE::get_boundary_intersections(polygons.begin(), polygons.end(), std::back_inserter(pts), false);
  if (pts.size() > 0)
  {
    LOG4CPLUS_ERROR(logger, "The polygons' boundaries intersect at these points: ");
    for (list<Point_2>::const_iterator it = pts.begin(); it != pts.end(); ++it)
      LOG4CPLUS_ERROR(logger, "  " <<  pp(*it));
    LOG4CPLUS_ERROR(logger, "Polygons: ");
    for (ContourIterator it = start; it != end; ++it)
      LOG4CPLUS_ERROR(logger, "  " <<  pp((*it)->polygon()));;
    throw runtime_error("The polygons' boundaries intersect");
  }

  _root = Contour::create(super_polygon(polygons.begin(), polygons.end()));
  // Make sure we have child and parent entries for root
  _children[_root];
  _parents[_root];
  for (ContourIterator it = start; it != end; ++it)
  {
    Contour_handle contour = *it;
    Contour_handle parent = find_parent(contour, _root);
    add_child(contour, parent);
  }
  _root->force_clockwise();

  // Orientation check
  if (policy == Hierarchy_policy::STRICT)
  {
    if (!check_orientations())
      throw std::runtime_error("Orientations invalid for a hierarchical, strict policy");
  }
  else if (policy == Hierarchy_policy::FORCE)
  {
    force_orientations();
  }
  else if (policy == Hierarchy_policy::FORCE_CCW)
  {
    for (typename Parent_map::iterator it = _parents.begin(); it != _parents.end(); ++it)
      it->first->force_counterclockwise();
  }
  else if (policy == Hierarchy_policy::NATURAL)
  {
    // do nothing
  }
  else
    throw std::logic_error("Unknown policy");
}

template
Hierarchy::Hierarchy(vector<Contour_handle>::iterator start, vector<Contour_handle>::iterator end, 
	  Hierarchy_policy policy);
template
Hierarchy::Hierarchy(vector<Contour_handle>::const_iterator start, vector<Contour_handle>::const_iterator end, 
	  Hierarchy_policy policy);
template
Hierarchy::Hierarchy(list<Contour_handle>::iterator start, list<Contour_handle>::iterator end, 
	  Hierarchy_policy policy);
template
Hierarchy::Hierarchy(list<Contour_handle>::const_iterator start, list<Contour_handle>::const_iterator end, 
	  Hierarchy_policy policy);

/// Throws if the given contour is not a part of this hierarchy
Contour_handle Hierarchy::parent(Contour_handle contour)
{ 
  assert_member(contour); 
  Contour_handle nec = Contour_handle(_parents[contour]); 
  return (nec == _root) ? Contour_handle() : nec; 
}

/// Throws if the given contour is not a part of this hierarchy
const Contour_handle Hierarchy::parent(Contour_handle contour) const
{ 
  assert_member(contour); 
  Contour_handle nec = Contour_handle(_parents.find(contour)->second); 
  return (nec == _root) ? Contour_handle() : nec; 
}

/// Throws if the given contour is not a part of this hierarchy
Hierarchy::iterator Hierarchy::children_begin(Contour_handle contour)
{ 
  assert_member(contour); 
  return _children[contour].begin(); 
}

/// Throws if the given contour is not a part of this hierarchy
Hierarchy::iterator Hierarchy::children_end(Contour_handle contour)
{ 
  assert_member(contour); 
  return _children[contour].end(); 
}

/// Throws if the given contour is not a part of this hierarchy
Contour_handle Hierarchy::NEC(Contour_handle contour)
{ 
  if (_parents.find(contour) != _parents.end())
    return parent(contour); 
  return find_parent(contour, _root);
}

/// Throws if the given contour is not a part of this hierarchy
const Contour_handle Hierarchy::NEC(Contour_handle contour) const
{ 
  if (_parents.find(contour) != _parents.end())
    return parent(contour); 
  return find_parent(contour, _root);
}

Contour_handle Hierarchy::NEC(Point_2 point)
{ 
  Contour_handle nec = find_parent(point, _root); 
  return (nec == _root) ? Contour_handle() : nec; 
}

const Contour_handle Hierarchy::NEC(Point_2 point) const
{ 
  Contour_handle nec = find_parent(point, _root); 
  return (nec == _root) ? Contour_handle() : nec; 
}

/// Infers the orientation from the hierarchy of the contour
CGAL::Orientation Hierarchy::orientation(Contour_handle contour) const
{
  if (is_CCW(contour)) return CGAL::COUNTERCLOCKWISE;
  return CGAL::CLOCKWISE;
}

/// Infers the orientation from the hierarchy of the contour
bool Hierarchy::is_CCW(const Contour_handle& contour) const
{ 
  return is_even(level(contour)); 
}

/// Infers the orientation from the hierarchy of the contour
bool Hierarchy::is_CW(const Contour_handle& contour) const
{ 
  return !is_CCW(contour); 
}

boost::tuple<Contour_handle, Contour_handle, Polygon_2::Vertex_circulator> Hierarchy::is_overlapping(const Point_2& point) const
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.Hierarchy.is_overlapping");

  Contour_handle nec = NEC(point);
  Contour_handle contour = is_overlapping(point, nec);
  Polygon_2::Vertex_circulator overlapped_point;
  bool found = false;
  if (contour)
  {
    Polygon_2::Vertex_circulator ci = contour->polygon().vertices_circulator();
    do
    {
      if (xy_equal(*ci, point)) {
	overlapped_point = ci;
	found = true;
      }
      ++ci;
    } while (ci != contour->polygon().vertices_circulator());

    if (!found) {
      LOG4CPLUS_DEBUG(logger, "Numerical error caused a false positive overlapping point: " << pp(point));
      contour.reset();
    }
  }
  return boost::make_tuple(contour, nec, overlapped_point);
}

boost::tuple<Vertex_sign, Contour_handle> Hierarchy::vertex_sign(const Point_2& point, CGAL::Orientation point_orientation) const
{
  Contour_handle parent = NEC(point);
  Vertex_sign sign = Vertex_sign::POSITIVE;
  CGAL::Orientation parent_orientation = parent ? orientation(parent) : CGAL::CLOCKWISE;
  // Check for overlapping vertex
  if (is_overlapping(point, parent))
    sign = Vertex_sign::OVERLAPPING;
  else if (point_orientation == parent_orientation)
    sign = Vertex_sign::NEGATIVE;
  else
    sign = Vertex_sign::POSITIVE;
  return boost::make_tuple(sign, parent);
}

Contour_handle Hierarchy::is_overlapping(const Point_2& point, Contour_handle nec) const
{
  if (!nec) nec = _root;

  const Children_container& children = _children.find(nec)->second;
  for (const_iterator it = children.begin(); it != children.end(); ++it)
  {
    if (intersects_boundary(point, Contour_handle(*it)->polygon()))
      return Contour_handle(*it);
  }
  return Contour_handle();
}

int Hierarchy::level(Contour_handle contour) const
{
  int lvl = -1;
  while (contour)
  {
    lvl++;
    contour = parent(contour);
  }
  return lvl;
}

/// Throws if the given contour is not a part of this hierarchy
void Hierarchy::assert_member(Contour_handle contour) const
{
  if (_parents.find(contour) == _parents.end())
  {
    stringstream ss;
    ss << "Contour does not exist in this hierarchy: " << pp(contour->polygon());
      throw std::runtime_error(ss.str());
  }
}

/// Finds <tt>contour</tt>'s parent given another contour <tt>ancestor</tt>
/// that is guaranteed to contain <tt>contour</tt>.
const Contour_handle Hierarchy::find_parent(Contour_handle contour, Contour_handle ancestor) const
{
  // Call find_parent with one of the vertices
  Point_2 point = *(contour->polygon().vertices_begin());
  return find_parent(point, ancestor);
}

const Contour_handle Hierarchy::find_parent(const Point_2& point, Contour_handle ancestor) const
{
  // Force ancestor to temporarily be CCW
  Temp_CCW temp_CCW(ancestor);

  Contour_handle parent;
  // Need to find the root of the problem
  // if (CGAL::orientation_2(ancestor->polygon().vertices_begin(), ancestor->polygon().vertices_end()) != CGAL::COLLINEAR &&
  //     ancestor->polygon().has_on_positive_side(point))
  if (ancestor->polygon().has_on_positive_side(point))
  {
    // Loop through all children of the ancestor until we find a parent.
    // This function relies on the fact that all contours are either
    // parent-child or sibling -- that there are no intersections of 
    // contour boundaries.
    const Children_container& children = _children.find(ancestor)->second;
    for (const_iterator it = children.begin(); 
	 !parent && it != children.end(); 
	 ++it)
    {
      Contour_handle child(*it);
      parent = find_parent(point, child);
    }
    if (!parent)
      parent = ancestor;
  }
  return parent;
}

void Hierarchy::add_child(Contour_handle child, Contour_handle parent)
{ 
  // Force child to temporarily be CCW
  Temp_CCW temp_CCW(child);
    
  // Transfer any children of the parent that should 
  // now be children of the child
  Children_container lost_children;
  find_children(child, parent, std::back_inserter(lost_children));
  Children_container& children = _children[parent];
  for (iterator it = lost_children.begin(); it != lost_children.end(); ++it) {
    Contour_handle lost_child(*it);
    for (iterator it2 = children.begin(); it2 != children.end(); ++it2) {
      Contour_handle tmp(*it2);
      if (lost_child == tmp) {
	children.erase(it2);
	break;
      }
    }
    // children.erase(find(children.begin(), children.end(), lost_child));
    // Have to do this after child has been added
    // add_child(Contour_handle(*it), child);
  }
  _children[parent].push_back(child);
  _parents[child] = parent;
  // Make sure we have an entry for the children
  _children[child];

  for (iterator it = lost_children.begin(); it != lost_children.end(); ++it) {
    add_child(Contour_handle(*it), child);
  }
}

void Hierarchy::force_orientations()
{ force_orientations(_root, -1); }

void Hierarchy::force_orientations(Contour_handle contour, int level)
{ 
  // A contour at level 0 should be CCW, enclosing CW contours at level 1.
  // So: even = CCW; odd = CW
  bool even = is_even(level);
  if (contour->is_counterclockwise_oriented() && !even)
    contour->reverse_orientation();
  else if (contour->is_clockwise_oriented() && even)
    contour->reverse_orientation();
  for (iterator it = _children[contour].begin(); it != _children[contour].end(); ++it)
  {
    Contour_handle child(*it);
    force_orientations(child, level+1);
  }
}

bool Hierarchy::check_orientations()
{ return check_orientations(_root, -1); }

bool Hierarchy::check_orientations(Contour_handle contour, int level)
{
  // A contour at level 0 should be CCW, enclosing CW contours at level 1.
  // So: even = CCW; odd = CW
  bool even = is_even(level);
  if (contour->is_counterclockwise_oriented() && !even)
    return false;
  else if (contour->is_clockwise_oriented() && even)
    return false;
  bool valid = true;
  for (iterator it = _children[contour].begin(); valid && it != _children[contour].end(); ++it)
  {
    Contour_handle child(*it);
    valid = check_orientations(child, level+1);
  }
  return valid;
}

bool Hierarchy::is_even(int level) const
{
  return (level % 2) == 0;
}

CONTOURTILER_END_NAMESPACE
