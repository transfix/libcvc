#ifndef __VERTEX_CHANGES_H__
#define __VERTEX_CHANGES_H__

#include <stdexcept>
#include <sstream>
#include <list>
#include <boost/unordered_map.hpp>

#include <ContourTiler/common.h>
#include <ContourTiler/print_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

template <typename T>
inline int sgn(T a)
{
  if (a > 0) return 1;
  if (a < 0) return -1;
  return 0;
}

/// Solves a quadratic equation.  Returns only real solutions.
/// ax^2 + bx + c = 0
/// x = (-b + sqrt(b*b - 4 * a * c)) / (2 * a)
/// x = (-b - sqrt(b*b - 4 * a * c)) / (2 * a)
inline bool solve_quad(Number_type a, Number_type b, Number_type c, Number_type& r1, Number_type& r2, Number_type& c1, Number_type& c2)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("solve_quad");

  bool ret = false;

  Number_type dis = b*b - 4 * a * c;
  if (a == 0) {
    if (b == 0) {
      throw std::logic_error("Degenerate quadratic");
    }
    r1 = r2 = -c/b;
    c1 = c2 = 0;
    ret = true;
  }
  else if (dis >= 0) {
    if (b != 0) {
      Number_type sdis = sqrt(dis);
      Number_type q = -(b + sgn(b)*sdis)/2.0;
      r1 = q/a;
      r2 = c/q;
    }
    else {
      Number_type sdis = sqrt(-a*c);
      r1 = sdis/a;
      r2 = -r1;
    }
    c1 = c2 = 0;
    ret = true;
  }
  else {
    r1 = r2 = -b/(2*a);
    c1 = sqrt(-dis)/(2*a);
    c2 = -c1;
    ret = false;
  }

  LOG4CPLUS_TRACE(logger, "a = " << a << " b = " << b << " c = " << c << " r1,r2 = " << r1 << " c1 = " << c1);
//   LOG4CPLUS_TRACE(logger, "discriminant = " << dis << " r1,r2 = " << r1 << " c1 = " << c1);
  return ret;
}

/// Returns true if b is lexicographically in-between a and c.
template <typename T>
bool in_between(T a, T b, T c)
{
  if (a > c)
    swap(a, c);
  return b > a && c > b;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Z_adjustments class
//
/// If a yellow and green point are xy_equal but their z-values render them
/// illegally placed, they are added to this container which stores their
/// original values and also their new z-values required to make them legal.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
template <typename Tile_handle>
class Z_adjustments
{
private:
  typedef boost::unordered_map<Point_3, Point_3> Adj_container;
  typedef Adj_container::iterator Adj_iterator;
  typedef Adj_container::const_iterator Adj_const_iterator;
  typedef typename std::list<Tile_handle> Tile_container;
  typedef typename boost::unordered_map<Point_2, Tile_container> Point2tiles;

  typedef std::logic_error logic_error;
  typedef std::stringstream stringstream;

public:
  typedef typename Tile_container::const_iterator Tile_const_iterator;

public:
  Z_adjustments() : ilogger(log4cplus::Logger::getInstance("Z_adjustments")) {}
  Z_adjustments(Number_type z0, Number_type z1, Number_type epsilon)
    : ilogger(log4cplus::Logger::getInstance("Z_adjustments"))
  { 
//     set_z_home(z0, z1); 
    _epsilon = epsilon;
  }
  ~Z_adjustments() {}

  void add(Tile_handle t)
  {
    _tiles.push_back(t);
  }

  Point_3 get_qy(const Point_3& yp, const Point_3& gp, int yi, int gi, Tile_handle yt, Tile_handle gt, Number_type y_z_home)
  {
    Point_3 qy = vertex(0, *yt);
//     if (!in_between(z_home(yi), yp.z(), qy.z()))
    if (!in_between(y_z_home, yp.z(), qy.z()))
      qy = vertex(1, *yt);
//     if (!in_between(z_home(yi), yp.z(), qy.z()))
    if (!in_between(y_z_home, yp.z(), qy.z()))
      qy = vertex(2, *yt);
    return qy;
  }

  void add(const Point_3& yp, const Point_3& gp, int yi, int gi, Tile_handle yt, Tile_handle gt, Number_type y_z_home, Number_type g_z_home)
  {
    Point_3 qy = vertex(0, *yt);
//     if (!in_between(z_home(yi), yp.z(), qy.z()))
    if (!in_between(y_z_home, yp.z(), qy.z()))
      qy = vertex(1, *yt);
//     if (!in_between(z_home(yi), yp.z(), qy.z()))
    if (!in_between(y_z_home, yp.z(), qy.z()))
      qy = vertex(2, *yt);

    add(yp, gp, yi, gi, qy, y_z_home, g_z_home);

    if (!gt || !yt)
      throw logic_error("Tile handles passed into Z_adjustments::add() must be non-null");

    _tiles.push_back(gt);
    _tiles.push_back(yt);

    _point2tiles[yi][yp].push_back(yt);
    _point2tiles[gi][gp].push_back(gt);
  }

  template <typename Tile_iter>
  void add(const Point_3& py, const Point_3& pg, int yi, int gi, 
	   const Point_3& qy,
	   Tile_iter ytiles_begin, Tile_iter ytiles_end,
	   Tile_iter gtiles_begin, Tile_iter gtiles_end,
	   Number_type y_z_home, Number_type g_z_home)
  {
    add(py, pg, yi, gi, qy, y_z_home, g_z_home);

    if (ytiles_begin == ytiles_end)
      throw logic_error("Tile handles passed into Z_adjustments::add() must be non-null");

    _tiles.insert(_tiles.end(), ytiles_begin, ytiles_end);

    _point2tiles[yi][py].insert(_point2tiles[yi][py].end(), ytiles_begin, ytiles_end);
    _point2tiles[gi][pg].insert(_point2tiles[gi][pg].end(), gtiles_begin, gtiles_end);
  }

  Tile_handle update(int ci, Tile_handle tile) const
  {
    const Adj_container& changes = _adjustments[ci];
    Tile_handle ret(new typename Tile_handle::element_type(*tile));
//     Tile_handle ret = tile;
    for (int j = 0; j < 3; ++j)
    {
      Point_3& vertex = (*ret)[j];
      if (changes.find(vertex) != changes.end())
      {
	vertex = changes.find(vertex)->second;
// 	LOG4CPLUS_TRACE(logger, "Changed vertex (" << i << "): " << pp(vertex_adjustments[i].find((*tile)[j])->second));
      }
    }
    return ret;
  }

  Polygon_2 update(int ci, const Polygon_2& p) const
  {
    const Adj_container& changes = _adjustments[ci];
    Polygon_2 ret;
    for (Polygon_2::Vertex_const_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it)
    {
      Point_3 vertex = *it;
      if (changes.find(vertex) != changes.end())
	vertex = changes.find(vertex)->second;
      ret.push_back(vertex);
    }
    return ret;
  }

  bool contains(const Point_3& vertex, int ci)
  {
    const Adj_container& changes = _adjustments[ci];
    return changes.find(vertex) != changes.end();
  }

  //--------------------------------------
  // get_points
  //
  /// Adds all points that require z adjustments to the 
  /// output iterator points.
  /// @param i component id
  //--------------------------------------
  template <typename Point_iter>
  void get_points(int i, Point_iter points) const
  {
    for (Adj_const_iterator it = _adjustments[i].begin(); it != _adjustments[i].end(); ++it)
    {
      *points = it->first;
      ++points;
    }
  }

  //--------------------------------------
  // tiles_begin
  // tiles_end
  //--------------------------------------
  Tile_const_iterator tiles_begin() const { return _tiles.begin(); }
  Tile_const_iterator tiles_end() const { return _tiles.end(); }

  Tile_const_iterator tiles_begin(const Point_2& p, int ci) const
  { return _point2tiles[ci].find(p)->second.begin(); }

  Tile_const_iterator tiles_end(const Point_2& p, int ci) const
  { return _point2tiles[ci].find(p)->second.end(); }

  //--------------------------------------
  // is_legal
  //
  /// Returns true if bz is closer to a's 
  /// z home than az
  //--------------------------------------
  bool is_legal(const Point_3& py, const Point_3& pg, const Point_3& qy,
		Number_type y_z_home, Number_type g_z_home);

//   Number_type mult_sign(int i) const
//   {
//     check_z();
//     return (_z_home[i] > _z_home[1-i]) ? 1 : -1;
//   }

  /// dir: 1 if epsilon is expected to be positive, -1 if otherwise.
  Number_type epsilon(const Point_3& A, const Point_3& B, Number_type delta, int dir) const;

  /// d - delta
  Number_type epsilon(const Point_3& A, const Point_3& B, Number_type d, int dir, Number_type sg) const;

//   Number_type z_home(int i)
//   { return _z_home[i]; }

private:
  Number_type epsilon() const;

  void add(const Point_3& py, const Point_3& pg, int yi, int gi, const Point_3& qy, Number_type y_z_home, Number_type g_z_home);

  //--------------------------------------
  // set_z_home
  //--------------------------------------
//   void set_z_home(Number_type z0, Number_type z1)
//   { 
//     if (z0 == z1)
//     {
//       stringstream ss;
//       ss << "z_home values cannot be the same: " << z0;
//       throw logic_error(ss.str());
//     }

//     _z_home[0] = z0; 
//     _z_home[1] = z1; 
//   }

//   void check_z() const
//   { 
//     if (_z_home[0] == _z_home[1])
//       throw logic_error("z_home values are uninitialized");
//   }

private:
  Adj_container _adjustments[2];
  // Affected tiles
  Tile_container _tiles;
  Number_type _epsilon;
//   Number_type _z_home[2];
  Point2tiles _point2tiles[2];

  log4cplus::Logger ilogger;
};

CONTOURTILER_END_NAMESPACE

#endif
