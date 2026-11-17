#ifndef TILER_WORKSPACE
#define TILER_WORKSPACE

#include <ContourTiler/common.h>
#include <ContourTiler/tiler_defines.h>
#include <ContourTiler/Boundary_slice_chords.h>
#include <ContourTiler/Untiled_region.h>
#include <ContourTiler/Vertices.h>
#include <ContourTiler/triangle_utils.h>
#include <ContourTiler/Tiler_callback.h>
#include <ContourTiler/Polyline.h>

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

CONTOURTILER_BEGIN_NAMESPACE

typedef std::list<Segment_3> Chord_list;
typedef boost::unordered_set<Segment_3> Chord_set;

typedef Triangle Tile;

typedef boost::shared_ptr<Tile> Tile_handle;
typedef std::list<Contour_handle> Contour_list;
typedef std::list<Tile_handle> Tile_list;
typedef Tile_list::iterator Tile_iterator;
typedef Tile_list::const_iterator Tile_const_iterator;
typedef boost::unordered_set<Point_3> Banned;
typedef Banned::iterator Banned_iterator;
typedef Banned::const_iterator Banned_const_iterator;

class Tiler_workspace
{
public:
  typedef std::vector<Contour_handle> Contours;
  typedef boost::shared_ptr<Tiler_workspace> TW_handle;

public:
  static TW_handle create(const Contours& contours_,
			  const Contours& bottom_,
			  const Contours& top_,
			  const Correspondences& correspondences_,
			  const Vertices& vertices_,
			  const Vertex_map<HTiling_region>& tiling_regions_,
			  const OTV_table& otv_table_, 
			  const Hierarchies& hierarchies_,
			  const Number_type& midz_)
  {
    return TW_handle(new Tiler_workspace(contours_, bottom_, top_, correspondences_, vertices_, tiling_regions_,
					 otv_table_, hierarchies_, midz_));
  }


private:
  Tiler_workspace() : midz(0) 
  { throw std::logic_error("Illegal call to default constructor"); }

  Tiler_workspace(const Contours& contours_,
		  const Contours& bottom_,
		  const Contours& top_,
		  const Correspondences& correspondences_,
		  const Vertices& vertices_,
		  const Vertex_map<HTiling_region>& tiling_regions_,
		  const OTV_table& otv_table_, 
		  const Hierarchies& hierarchies_,
		  const Number_type& midz_)
    : contours(contours_), bottom(bottom_), top(top_), correspondences(correspondences_), vertices(vertices_), 
      tiling_regions(tiling_regions_), otv_table(otv_table_),
      hierarchies(hierarchies_), midz(midz_), _callback(new Tiler_callback()) 
  {
    _zmin = (*contours.begin())->slice();
    _zmax = (*contours.begin())->slice();
    for (Contours::const_iterator it = contours.begin(); it != contours.end(); ++it)
    {
      _zmin = (_zmin < (*it)->slice()) ? _zmin : (*it)->slice();
      _zmax = (_zmax > (*it)->slice()) ? _zmax : (*it)->slice();
    }
  }

public:
  ~Tiler_workspace() {}

public:
  Vertex_iterator vertices_begin() const { return vertices.begin(); }
  Vertex_iterator vertices_end() const { return vertices.end(); }

  Contour_handle contour(const Point_3& v) const { return vertices.contour(v); }

  void add_tile(Point_3 p0, Point_3 p1, Point_3 p2);
  void add_tile(Tile_handle tile);

  template <typename Tile_iter>
  void add_tiles(Tile_iter begin, Tile_iter end)
  {
    for (Tile_iter it = begin; it != end; ++it)
      add_tile(*it);
  }

  void set_callback(boost::shared_ptr<Tiler_callback> callback)
  { _callback = callback; }

  Tile_list& tiles(Contour_handle c) { return _contour2tile[c]; }

  const Tile_list& tiles(Contour_handle c) const { return _contour2tile.find(c)->second; }

  void remove(Tile_handle tile);

  Tile_iterator tiles_begin()
  { return _tiles.begin(); }

  Tile_iterator tiles_end()
  { return _tiles.end(); }

  Tile_const_iterator tiles_begin() const
  { return _tiles.begin(); }

  Tile_const_iterator tiles_end() const
  { return _tiles.end(); }

  Number_type zmin() const
  { return _zmin; }

  Number_type zmax() const
  { return _zmax; }

//   Number_type zhome(const Point_2& p) const;
  
//   Number_type zhome(const Point_2& p, const Point_2& q) const;
  
  template <typename Point_iter>
  void propagate_z_home(Point_iter begin, Point_iter end);

  template <typename Point_iter>
  void propagate_z_home(Point_iter begin, Point_iter end, Number_type z_home);

  void set_z_home(const Point_3& p, Number_type z_home);

  bool has_z_home(const Point_3& p);

  Number_type z_home(const Point_3& p);

  template <typename Point_iter>
  Number_type z_home(Point_iter begin, Point_iter end);

  Number_type z_home_nothrow(const Point_3& p);

  Number_type z_home(Tile_handle tile);

  void ensure_z_home(Polyline_2& cut, Number_type home);

  void set_banned(const Banned& banned)
  { _banned = banned; }

  void add_banned(const Point_3& p)
  { _banned.insert(p); }

  bool is_banned(const Point_3& p)
  { return _banned.find(p) != _banned.end(); }

private:
  std::list<Contour_handle> find_contours(Tile_handle tile);

public:
  Contours const contours, bottom, top;
  Correspondences const  correspondences;
  Vertices vertices;
  Vertex_map<HTiling_region> const tiling_regions;
  OTV_table const otv_table;
  Hierarchies const hierarchies;
  Boundary_slice_chords bscs;
  Vertex_completion_map completion_map;

  const Number_type midz;

  boost::shared_ptr<Tiler_callback> _callback;

private:
  Tile_list _tiles;
  boost::unordered_map<Contour_handle, Tile_list> _contour2tile;
  Number_type _zmin, _zmax;
  Banned _banned;
  boost::unordered_map<size_t, Number_type> _z_home;
};

typedef Tiler_workspace::TW_handle TW_handle;

CONTOURTILER_END_NAMESPACE

#endif
