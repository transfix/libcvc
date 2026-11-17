#include <ContourTiler/Tiler_workspace.h>
#include <ContourTiler/print_utils.h>
#include <iostream>
#include <ContourTiler/segment_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

log4cplus::Logger logger_tw = log4cplus::Logger::getInstance("tiler.workspace");

void Tiler_workspace::add_tile(Point_3 p0, Point_3 p1, Point_3 p2)
{
  Contour_handle c0 = contour(p0);
  Contour_handle c1 = contour(p1);
  Contour_handle c2 = contour(p2);

  // Only swap if we're doing a contour-to-contour tile.  Don't do anything
  // if it is a contour-to-midslice tile
  if (c0 && c1 && c2 && c0 == c1 && p0.z() > p2.z())
  {
    LOG4CPLUS_TRACE(logger_tw, "Swapping before tile insert" << pp(p0) << ", " << pp(p2));
    std::swap(p0, p2);
  }

//   add_tile(Tile_handle(new Tile(p0, p1, p2, *this)));
  add_tile(Tile_handle(new Tile(p0, p1, p2)));
}

void Tiler_workspace::add_tile(Tile_handle t)
{
  _tiles.push_back(t);
  Contour_list contours = find_contours(t);
  for (Contour_list::iterator it = contours.begin();
       it != contours.end();
       ++it)
  {
    _contour2tile[*it].push_back(t);
  }

  Tile& tile = *t;
  LOG4CPLUS_TRACE(logger_tw, "Inserted tile: " << pp(tile[0]) << ", " << pp(tile[1]) << ", " << pp(tile[2]));

  _callback->tile_added(*this);
}

/// Returns all unique contours touching this tile
Contour_list Tiler_workspace::find_contours(Tile_handle tile_h)
{
  Contour_list contours;

  Tile& tile = *tile_h;
  Contour_handle c0 = contour(tile[0]);
  Contour_handle c1 = contour(tile[1]);
  Contour_handle c2 = contour(tile[2]);
  if (c0)
    contours.push_back(c0);
  if (c1 && c1 != c0)
    contours.push_back(c1);
  if (c2 && c2 != c0 && c2 != c1)
    contours.push_back(c2);
  return contours;
}

void Tiler_workspace::remove(Tile_handle tile)
{
  _tiles.remove(tile);

  Contour_list contours = find_contours(tile);
  for (Contour_list::iterator it = contours.begin(); it != contours.end(); ++it)
    _contour2tile[*it].remove(tile);
}

void Tiler_workspace::set_z_home(const Point_3& p, Number_type z_home)
{
  if (!p.is_valid()) {
    throw logic_error("Point must have a valid ID for the z_home to be set");
  }

  // If there is a region then the point's z_home is taken care of by
  // the region.  This is important to respect, as regions correctly
  // handle overlap points, for which z_home is ambiguous.
  if (tiling_regions.contains(p)) {
    HTiling_region region = tiling_regions[p];
    if (!region) {
      _z_home[p.id()] = z_home;
    }
  }
  else {
    _z_home[p.id()] = z_home;
  }
}

bool Tiler_workspace::has_z_home(const Point_3& p)
{
  Number_type z_home = z_home_nothrow(p);
  return z_home == z_home;
}

Number_type Tiler_workspace::z_home(const Point_3& p)
{
  if (!has_z_home(p))
    throw logic_error("No z_home");

  return z_home_nothrow(p);
}

template <typename Point_iter>
Number_type Tiler_workspace::z_home(Point_iter begin, Point_iter end)
{
  for (Point_iter it = begin; it != end; ++it) {
    if (has_z_home(*it)) {
      return z_home(*it);
    }
  }
  throw logic_error("No z_home in container");
}

Number_type Tiler_workspace::z_home_nothrow(const Point_3& p)
{
  if (_z_home.find(p.id()) != _z_home.end()) {
    return _z_home[p.id()];
  }
  else if (tiling_regions.contains(p)) {
    HTiling_region region = tiling_regions[p];
    if (region) {
      return region->z_home_nothrow();
    }
  }
  return Tiling_region::AMBIGUOUS_Z_HOME;
}

Number_type Tiler_workspace::z_home(Tile_handle tile)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tw.z_home");

  Tile& t = *tile;
  int invalid = -1;
  int overlapping = -1;

  Number_type z_home;
  bool found = false;

  for (int i = 0; i < 3 && !found; ++i) {
    if (!t[i].is_valid()) {
      throw logic_error("Point must have a valid ID to get the z_home");
    }

    if (_z_home.find(t[i].id()) != _z_home.end()) {
      z_home = _z_home[t[i].id()];
      found = true;
    }
//     else if (t[i].is_valid()) {
    else if (tiling_regions.contains(t[i])) {
      HTiling_region region = tiling_regions[t[i]];
      if (region && region->has_z_home()) {
	z_home = region->z_home();
	found = true;
      }
      else if (!region) {
	LOG4CPLUS_TRACE(logger, "No region: " + pp_tri(*tile));
      }
      else {
	LOG4CPLUS_TRACE(logger, "No z_home: " + pp_tri(*tile));
	overlapping = i;
      }
    }
    else {
      invalid = i;
    }
  }

  if (invalid != -1 && overlapping != -1 && !found) {
    const Point_3& ip = t[invalid];
    const Point_3& op = t[overlapping];
    
    Contour_handle c = contour(op);
    if (!c->polygon().has_on_negative_side(ip)) {
      z_home = c->slice();
      found = true;
    }
    else {
      z_home = (c->slice() == zmin()) ? zmax() : zmin();
      found = true;
    }
  }

  if (found) {
    for (int i = 0; i < 3 && !found; ++i) {
      _z_home[t[i].id()] = z_home;
    }
  }

  if (!found)
    throw logic_error("Expected z_home value: " + pp_tri(*tile));

  return z_home;
}

template <typename Point_iter>
void Tiler_workspace::propagate_z_home(Point_iter begin, Point_iter end)
{
  Number_type zhome;
  bool found = false;
  for (Point_iter it = begin; it != end; ++it) {
    if (has_z_home(*it)) {
      if (found && z_home(*it) != zhome) {
	throw logic_error("Inconsistent z_home");
      }
      zhome = z_home(*it);
      found = true;
    }
  }
  if (!found) {
    throw logic_error("No z_home in container");
  }
  propagate_z_home(begin, end, zhome);
}

template <typename Point_iter>
void Tiler_workspace::propagate_z_home(Point_iter begin, Point_iter end, Number_type z_home)
{
  for (Point_iter it = begin; it != end; ++it) {
    set_z_home(*it, z_home);
  }
}

template
Number_type Tiler_workspace::z_home(Polygon_2::Vertex_iterator begin, Polygon_2::Vertex_iterator end);

template
void Tiler_workspace::propagate_z_home(Polygon_2::Vertex_iterator begin, Polygon_2::Vertex_iterator end);

template
void Tiler_workspace::propagate_z_home(Polygon_2::Vertex_iterator begin, Polygon_2::Vertex_iterator end, Number_type z_home);

void Tiler_workspace::ensure_z_home(Polyline_2& cut, Number_type home)
{
  for (Polyline_2::iterator cut_iter = cut.begin(); cut_iter != cut.end(); ++cut_iter) {
    if (!cut_iter->is_valid()) {
      cut_iter->id() = vertices.unique_id();
    }
    set_z_home(*cut_iter, home);
  }
}

// Number_type Tiler_workspace::zhome(const Point_2& p) const
// {
//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("Tiler_workspace");

//   const Hierarchy& hmin = hierarchies.find(_zmin)->second;
//   const Hierarchy& hmax = hierarchies.find(_zmax)->second;
//   for (Contours::const_iterator it = contours.begin(); it != contours.end(); ++it)
//   {
//     const Contour_handle c = *it;
//     Number_type z = c->polygon()[0].z();
//     const Hierarchy& h = hierarchies.find(z)->second;
//     if (h.is_CCW(c) && !c->polygon().has_on_negative_side(p))
//       return z;
//   }
//   LOG4CPLUS_ERROR(logger, "Didn't expect to be here: " << pp(p));
//   throw logic_error("Didn't expect to be here");
// }

// Number_type Tiler_workspace::zhome(const Point_2& p, const Point_2& q) const
// {
//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("Tiler_workspace");

//   list<size_t> p_contours, q_contours;

//   const Hierarchy& hmin = hierarchies.find(_zmin)->second;
//   const Hierarchy& hmax = hierarchies.find(_zmax)->second;
//   for (Contours::const_iterator it = contours.begin(); it != contours.end(); ++it)
//   {
//     const Contour_handle c = *it;
//     Number_type z = c->polygon()[0].z();
//     const Hierarchy& h = hierarchies.find(z)->second;
//     if (h.is_CCW(c))
//     {
//       if (!c->polygon().has_on_negative_side(p))
// 	p_contours.push_back(z);
//       if (!c->polygon().has_on_negative_side(q))
// 	q_contours.push_back(z);
//     }
//   }
//   if (p_contours.size() > 1 && q_contours.size() > 1)
//   {
//     Point_2 r((p.x()+q.x())/2, (p.y()+q.y())/2, (p.z()+q.z())/2);
//     return zhome(r);
// //     LOG4CPLUS_ERROR(logger, "P and Q are both in two contours: " << pp(p) << " " << pp(q));
// //     throw logic_error("P and Q are both in two contours");
//   }
//   if (p_contours.size() == 1 && q_contours.size() == 1 && p_contours.front() != q_contours.front())
//   {
//     LOG4CPLUS_ERROR(logger, "P and Q are both in one contour but have different z values: " << pp(p) << " " << pp(q));
//     throw logic_error("P and Q are both in one contour but have different z values: ");
//   }
//   if (p_contours.size() == 1)
//     return p_contours.front();
//   return q_contours.front();
// }

CONTOURTILER_END_NAMESPACE
