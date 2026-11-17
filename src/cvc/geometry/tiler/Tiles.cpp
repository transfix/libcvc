#include <ContourTiler/Tiles.h>

CONTOURTILER_BEGIN_NAMESPACE

std::list<std::pair<Point_3, Tiles::Color> > Tiles::vertices() const
{
  std::list<std::pair<Point_3, Color> > verts;
  vertices(std::back_inserter(verts));
  return verts;
}

std::list<size_t> Tiles::tile_indices() const
{
  // Update indices of vertices
  size_t index = 0;
  Vertex_list& verts = const_cast<Vertex_list&>(*_vertices);
  for (Vertex_list::iterator it = verts.begin(); it != verts.end(); ++it)
    it->second.first = index++;

  std::list<size_t> indices;
  for(Tile_const_iterator it = _tiles->begin(); it != _tiles->end(); ++it)
  {
    indices.push_back((*it)->second.first);
  }
  return indices;
}

void Tiles::insert(const Point_3& v0, const Point_3& v1, const Point_3& v2)
{
  insert(v0, v1, v2, 1, 1, 1);
}

void Tiles::insert(const Point_3& v0, const Point_3& v1, const Point_3& v2, double r, double g, double b)
{
  using namespace std;
  pair<size_t, Color> val = make_pair(0, Color(r, g, b));
  Vertex_iterator it0 = _vertices->insert(make_pair(v0, val)).first;
  Vertex_iterator it1 = _vertices->insert(make_pair(v1, val)).first;
  Vertex_iterator it2 = _vertices->insert(make_pair(v2, val)).first;
  _tiles->push_back(it0);
  _tiles->push_back(it1);
  _tiles->push_back(it2);
}

void Tiles::insert(const Tiles& tiles)
{
//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.Tiles");

  std::vector<std::pair<Point_3, Color> > vertices;
  tiles.vertices(std::back_inserter(vertices));
  std::list<size_t> indices = tiles.tile_indices();
  for (std::list<size_t>::iterator it = indices.begin();
       it != indices.end();)
  {
    const Point_3& p0 = (vertices[*(it++)].first);
    const Point_3& p1 = (vertices[*(it++)].first);
    const Point_3& p2 = (vertices[*(it++)].first);
    insert(p0, p1, p2);
  }
}


CONTOURTILER_END_NAMESPACE
