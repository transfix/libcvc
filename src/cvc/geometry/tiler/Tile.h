#ifndef __TILE_H__
#define __TILE_H__

#include <ContourTiler/Tiles.h>

#endif

// #include <boost/tuple/tuple.hpp>
// #include <common.h>

// CONTOURTILER_BEGIN_NAMESPACE

// class Tiles
// {
//   // _vertices is a map with key/value of Point_3/size_t
//   // where size_t is the index of the point in iteration
//   // order.
//   //
//   // _tiles is a list of iterators, one for each point in
//   // the tile.

// private:
//   typedef std::map<Point_3, size_t> Vertex_list;
//   typedef Vertex_list::iterator Vertex_iterator;
//   typedef std::vector<Vertex_iterator> Tile_list;
//   typedef Tile_list::iterator Tile_iterator;
//   typedef Tile_list::const_iterator Tile_const_iterator;

// public:
//   Tiles() {}
//   ~Tiles() {}

//   std::list<Point_3> vertices() const;
// //   {
// //     std::list<Point_3> verts;
// // //     vertices(std::back_inserter(verts));
// //     for (Vertex_list::const_iterator it = _vertices.begin(); it != _vertices.end(); ++it)
// //       verts.push_back(it->first);
// //     return verts;
// //   }

//   template <typename Output_iterator>
//   void vertices(Output_iterator verts) const
//   {
//     for (Vertex_list::const_iterator it = _vertices.begin(); it != _vertices.end(); ++it)
//     {
//       *verts = it->first;
//       ++verts;
//     }
//   }

//   std::list<size_t> tile_indices() const;
// //   {
// //     // Update indices of vertices
// //     size_t index = 0;
// //     Vertex_list& verts = const_cast<Vertex_list&>(_vertices);
// //     for (Vertex_list::iterator it = verts.begin(); it != verts.end(); ++it)
// //       it->second = index++;

// //     std::list<size_t> indices;
// //     for(Tile_const_iterator it = _tiles.begin(); it != _tiles.end(); ++it)
// //     {
// //       indices.push_back((*it)->second);
// //     }
// //     return indices;
// //   }

//   void insert(const Point_3& v0, const Point_3& v1, const Point_3& v2);
// //   {
// //     using namespace std;
// //     Vertex_iterator it0 = _vertices.insert(make_pair(v0, 0)).first;
// //     Vertex_iterator it1 = _vertices.insert(make_pair(v1, 0)).first;
// //     Vertex_iterator it2 = _vertices.insert(make_pair(v2, 0)).first;
// //     _tiles.push_back(it0);
// //     _tiles.push_back(it1);
// //     _tiles.push_back(it2);
// //   }

//   void insert(const Tiles& tiles);
// //   {
// //     std::list<Point_3> verts_list = tiles.vertices();
// //     std::vector<Point_3> vertices(verts_list.begin(), verts_list.end());
// // //     tiles.vertices(std::back_inserter(vertices));
// //     std::list<size_t> indices = tiles.tile_indices();
// //     for (std::list<size_t>::iterator it = indices.begin();
// // 	 it != indices.end();)
// //     {
// //       Point_3 p0(vertices[*(it++)]);
// //       Point_3 p1(vertices[*(it++)]);
// //       Point_3 p2(vertices[*(it++)]);
// //       insert(p0, p1, p2);
// //     }
// //   }

//   template <typename Output_iterator>
//   void as_single_array(Output_iterator tiles) const
//   {
//     for (Tile_const_iterator it = _tiles.begin(); it != _tiles.end(); ++it)
//     {
//       *tiles = (*it)->first;
//       ++tiles;
//     }
//   }

//   size_t num_verts() const { return _vertices.size(); }
//   size_t num_tiles() const { return _tiles.size() / 3; }

// private:
//   Vertex_list _vertices;
//   Tile_list _tiles;
// };

// CONTOURTILER_END_NAMESPACE

// #endif
