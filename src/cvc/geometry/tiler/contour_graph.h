#ifndef __CONTOUR_GRAPH_H__
#define __CONTOUR_GRAPH_H__

#include <ContourTiler/contour_utils.h>

#include <utility> // std::pair
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
// #include <boost/graph/graphml.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/visitors.hpp>


CONTOURTILER_BEGIN_NAMESPACE

struct Element_type_def
{
  enum type { AXON, DENDRITE, SYNAPSE };
};

typedef Right_enum<Element_type_def> Element_type;

//--------------------------------------------------------
// Contour_entity class
// Used as a property for vertices
//--------------------------------------------------------
template <typename Contour_handle>
class Contour_entity
{
private:
  typedef std::list<Contour_handle> Container;

public:
  Contour_entity() {}

  template <typename InputIterator>
  Contour_entity(const std::string& name, InputIterator start, InputIterator end) : _name(name), _contours(start, end) {}

  ~Contour_entity() {}

  std::string& name() { return _name; }
  const std::string& name() const { return _name; }

  template <typename InputIterator>
  void assign_contours(InputIterator start, InputIterator end)
  { _contours.assign(start, end); }

  Element_type type() const
  { return type(_name); }

  static Element_type type(const std::string& name) 
  { 
    if (name[0] == 'a') return Element_type::AXON;
//     if (name[0] == 'd' && name.find('c') == -1) return Element_type::DENDRITE;
    if (name[0] == 'd' && name.length() < 6) return Element_type::DENDRITE;
    return Element_type::SYNAPSE;
  }

private:
  std::string _name;
  Container _contours;
};

//--------------------------------------------------------
// DFS visitor
//--------------------------------------------------------
class dfs_visitor : public boost::default_dfs_visitor 
{
public:
//   dfs_visitor(TimeMap dmap, TimeMap fmap, T & t)
//    :  m_dtimemap(dmap), m_ftimemap(fmap), m_time(t) {
//   }

  template <typename Vertex, typename Graph>
  void discover_vertex(Vertex v, const Graph & g) const
  {
    int deg = degree(v, g);
    std::cout << g[v].name() << "(" << deg << ")" << std::endl;
//     put(m_dtimemap, u, m_time++);
  }
//   template < typename Vertex, typename Graph >
//   void finish_vertex(Vertex u, const Graph & g) const
//   {
//     put(m_ftimemap, u, m_time++);
//   }
//   TimeMap m_dtimemap;
//   TimeMap m_ftimemap;
//   T & m_time;
};

//--------------------------------------------------------
// label_writer class
// Used for output to Graphviz format
//--------------------------------------------------------
template <class Graph>
class label_writer {
public:
  label_writer(Graph graph) : _graph(graph) {}
  template <class VertexOrEdge>
  void operator()(std::ostream& out, const VertexOrEdge& v) const 
  { 
    out << "[label=\"" << _graph[v].name() << "\", style=filled"; 
    if (_graph[v].type() == Element_type::AXON)
      out << ", shape=box, fillcolor=cornsilk";
    else if (_graph[v].type() == Element_type::DENDRITE)
      out << ", fillcolor=aliceblue";
    else
      out << ", fillcolor=gold1";
    out << "]";
  }
private:
  Graph _graph;
};

template <typename Graph>
void compute_degrees(Graph g)
{
  using namespace boost;
  using namespace std;

  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  typedef graph_traits<Graph> GraphTraits;
  typename graph_traits<Graph>::vertex_iterator v_it, v_end;
  typename graph_traits<Graph>::adjacency_iterator a_it, a_end;
  typename GraphTraits::in_edge_iterator in_it, in_end;
  typename GraphTraits::edge_descriptor edge;

  Statistics<double> axon_stats, dendrite_stats;

  for (tie(v_it, v_end) = vertices(g); v_it != v_end; ++v_it) 
  {
    Vertex v = *v_it;
    int deg = degree(v, g);

    cout << g[v].name() << "(" << deg << ")";
    if (g[v].type() == Element_type::AXON) axon_stats.add(deg);
    else if (g[v].type() == Element_type::DENDRITE) dendrite_stats.add(deg);
      
    if (deg > 0)
    {
      std::cout << " - ";

      // Out vertices
      for (tie(a_it, a_end) = adjacent_vertices(v, g);
	   a_it != a_end; ++a_it) 
      {
	std::cout << g[*a_it].name();
	if (boost::next(a_it) != a_end)
	  std::cout << ", ";
      }
      
      // In vertices
      for (tie(in_it, in_end) = in_edges(v, g);
	   in_it != in_end; ++in_it) 
      {
	edge = *in_it;
	Vertex s = source(edge, g);
	cout << g[s].name();
	if (boost::next(in_it) != in_end)
	  std::cout << ", ";
      }
    }
    std::cout << std::endl;

  }

  cout << "Axon degree:     " << axon_stats << endl;
  cout << "Dendrite degree: " << dendrite_stats << endl;

}

void test_graph();

template <class NewGraph, class Tag>
struct graph_copier 
  : public boost::base_visitor<graph_copier<NewGraph, Tag> >
{
  typedef Tag event_filter;

  graph_copier(NewGraph& graph) : new_g(graph) { }

  template <class Edge, class Graph>
  void operator()(Edge e, Graph& g) {
    boost::add_edge(boost::source(e, g), boost::target(e, g), new_g);
  }
private:
  NewGraph& new_g;
};

template <class NewGraph, class Tag>
inline graph_copier<NewGraph, Tag>
copy_graph(NewGraph& g, Tag) {
  return graph_copier<NewGraph, Tag>(g);
}
//--------------------------------------------------------
// test_graph
//--------------------------------------------------------
/// @param start first contour
/// @param end last + 1 contour
/// @param synapses list of synapses: synapse[index]<synapse_name, dendrite_name, axon_name>
template <typename InputIterator, typename ApSynIterator>
void test_graph(InputIterator start, InputIterator end, 
		const std::list<boost::tuple<std::string, std::string, std::string> >& synapses,
		ApSynIterator ap_syn_begin, ApSynIterator ap_syn_end)
{
  using namespace std;
  using namespace boost;

  typedef typename iterator_traits<InputIterator>::value_type Contour_handle;
  typedef Contour_entity<Contour_handle> Entity;
//   typedef adjacency_list<vecS, vecS, bidirectionalS, Entity> Graph;
  typedef adjacency_list<vecS, vecS, undirectedS, Entity> Graph;
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  typedef boost::unordered_map<string, list<Contour_handle> > by_name_t;
  typedef list<boost::tuple<string, string, string> > Synapses;

  // Get a map of the contours by name
  by_name_t by_name = contours_by_name(start, end);
  Graph g;
  map<string, Vertex> name_2_vertex;
  for (typename by_name_t::const_iterator it = by_name.begin(); it != by_name.end(); ++it)
  {
    if (Entity::type(it->first) != Element_type::SYNAPSE)
    {
      Vertex v = add_vertex(g);
      g[v].assign_contours(it->second.begin(), it->second.end());
      g[v].name() = it->first;
      name_2_vertex[it->first] = v;
    }
  }

  for (ApSynIterator it = ap_syn_begin; it != ap_syn_end; ++it)
  {
    Vertex v = add_vertex(g);
    g[v].name() = it->name();
    name_2_vertex[it->name()] = v;
    if (it != ap_syn_begin)
    {
      ApSynIterator prev = it;
      --prev;
      Vertex vp = name_2_vertex[prev->name()];
      add_edge(vp, v, g);
    }
  }

  for (Synapses::const_iterator it = synapses.begin(); it != synapses.end(); ++it)
  {
    if (name_2_vertex.count(get<1>(*it)) == 0)
      cout << "Unknown dendrite: " << get<1>(*it) << endl;
    else if (name_2_vertex.count(get<2>(*it)) == 0)
      cout << "Unknown axon: " << get<1>(*it) << endl;
    else
    {
      string synapse_name = get<0>(*it);
      string dendrite_name = get<1>(*it);
      string axon_name = get<2>(*it);
      Vertex dendrite_vertex = name_2_vertex[dendrite_name];
//       if (name_2_vertex.find(synapse_name) != name_2_vertex.end())
// 	dendrite_vertex = name_2_vertex.find(synapse_name)->second;
      Vertex axon_vertex = name_2_vertex[axon_name];
      if (!add_edge(axon_vertex, dendrite_vertex, g).second)
	throw std::logic_error("Failed to add synapse edge to graph");
    }
  }

//   dfs_visitor vis;
//   depth_first_search(g, visitor(vis));

  compute_degrees(g);

  Vertex bfs_start = name_2_vertex["d000A"];
  size_t num_verts = boost::num_vertices(g);
  std::vector<typename boost::graph_traits<Graph>::vertices_size_type> distances(num_verts);
  std::fill_n(distances.begin(), num_verts, 0);
  std::vector<Vertex> paths(num_verts);
  paths[bfs_start] = bfs_start;
  Graph G_copy(num_verts);
  
  boost::breadth_first_search
    (g, bfs_start, 
     boost::visitor(boost::make_bfs_visitor
		    (std::make_pair(boost::record_distances(&distances[0], boost::on_tree_edge()),
				    boost::record_predecessors(&paths[0], boost::on_tree_edge())))));

  // Print the distances from each axon to the apical dendrite (d000A)
  // if the distance is greater than 0.
  typename graph_traits<Graph>::vertex_iterator cur = boost::vertices(g).first;
  typename graph_traits<Graph>::vertex_iterator e = boost::vertices(g).second;
  for(; cur != e; ++cur)
  {
    size_t dist = distances[*cur];
    if (dist > 0 && g[*cur].type() == Element_type::AXON)
      cout << g[*cur].name() << " " << dist << endl;
  }

//   // to convert:
//   // dot -Kpdf -Tpdf graph.dot > graph.pdf
//   // dot -Kpng -Tpdf graph.dot > graph.png
//   ofstream out("graph.dot");
//   write_graphviz(out, g, label_writer<Graph>(g));
//   out.close();
}

CONTOURTILER_END_NAMESPACE

#endif
