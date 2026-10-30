#ifndef __OCTREE_STEP_H__
#define __OCTREE_STEP_H__

// The state of Octree::step(): the loops of the octree's operations with their counters, queues
// and scratch, which the one-shot calls kept on the stack. See Octree::begin_set_volume().

#include <algorithm>
#include <cstddef>
#include <memory>
#include <vector>
#include <VolMagickCompat.h>
#include "LBIE_geoframe.h"

namespace LBIE
{

// Take `units` out of a step's budget, down to no less than 0.
inline void spend(std::size_t& budget, std::size_t units)
{
	budget = units >= budget ? 0 : budget - units;
}

// resize(n, val) a slice at a time, 64 elements to a unit of budget: true once v has n elements.
template <class T>
bool grow_some(std::vector<T>& v, std::size_t n, const T& val, std::size_t& budget)
{
	if (v.size() >= n) return true;
	if (v.capacity() < n) v.reserve(n);
	const std::size_t left = n - v.size();
	const std::size_t take = budget >= (left + 63)/64 ? left : budget*64;
	v.insert(v.end(), take, val);
	spend(budget, (take + 63)/64);
	return v.size() >= n;
}

// std::fill(v.begin() + pos, v.end(), val) a slice at a time, 64 elements to a unit of budget,
// moving pos on: true once it reaches the end.
template <class T>
bool fill_some(std::vector<T>& v, std::size_t& pos, const T& val, std::size_t& budget)
{
	const std::size_t n = v.size();
	if (pos >= n) return true;
	const std::size_t left = n - pos;
	const std::size_t take = budget >= (left + 63)/64 ? left : budget*64;
	std::fill(v.begin() + pos, v.begin() + (pos + take), val);
	pos += take;
	spend(budget, (take + 63)/64);
	return pos >= n;
}

// The cells of one level of the octree, a cell at a time, and the cells of it that were refined,
// whose children make the next level: breadth first, in the order collapse() and traverse_qef()
// took cells from their two CellQueues.
struct cell_levels {
	std::vector<int> level, refined, next;
	std::size_t at = 0, expanded = 0;
	void start(int root)
	{
		level.assign(1, root);
		refined.clear();
		next.clear();
		at = expanded = 0;
	}
};

// The leaf polygonize(), tetrahedralize() and extract_hexa() are on: cut_array[i]'s cell.
struct leaf_cell {
	int valid_leaf = 0, level = 0, cell_size = 0, x = 0, y = 0, z = 0;
	float val[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

// extract_hexa()'s loops: untangle_snapped() runs them a second time, into its own mesh. Its
// stages, and untangle_snapped()'s:
enum { HX_VTX, HX_REFINE_CLEAR, HX_FIRST, HX_FIRST_VFLAGS, HX_PASS_START, HX_PASS, HX_PASS_VFLAGS,
       HX_LEAVES, HX_DONE };
enum { UN_CORNERS, UN_SCAN, UN_CLEAR, UN_UNSNAPPED, UN_FIRST, UN_COUNT, UN_PREFIX, UN_NEXT,
       UN_AROUND, UN_SNAPPED, UN_ROUNDS, UN_FIX, UN_STALE, UN_PASS, UN_DONE };
struct hexa_extract_state {
	geoframe* g = nullptr;
	float err_tol = 0.0f;
	int stage = 0, refine_sign = 0, j = 0;
	std::size_t pos = 0, passes = 0;
	leaf_cell leaf;
};

// untangle_snapped()'s loops and the arrays they share.
struct untangle_state {
	int stage = 0, nh = 0, nv = 0, h = 0, round = 0, pass = 0;
	bool again = false, advanced = false;
	std::size_t pos = 0;
	std::vector<unsigned int> corner, moved_back, pending, still;
	std::vector<int> first, around, next;
	std::vector<double> floor, along;
	std::vector<geoframe::float_3> snapped;
	std::vector<char> stale;
	geoframe unsnapped;
	hexa_extract_state hx;
};

// geometric_flow_*()'s neighbour tables and rounds.
struct flow_state {
	int kind = 0, stage = 0, index = 0;
	std::size_t pos = 0;
	geoframe* g = nullptr;
	std::vector<int> nb;			// tri, quad, hex: width entries a vertex, -1 past the last
	int width = 0;
	std::vector< std::vector<int> > tets;	// tet: the tets around each vertex
	std::vector<float> tet_cos, tet_vol, moved_cos, moved_vol;
};

// edge_contraction()'s rounds.
struct contraction_state {
	int index = 0;
	double new_edge_ratio = 0.0;
};

// smoothing_joeliu_volume()'s stages and rounds.
struct joeliu_state {
	int stage = 0, sign = 0, index = 0, ntet = 0, a_vert = -1, minAspIndx = 0, optRunNum = 0;
	double aspect_min = 0.0, aspect_max = 0.0, biggest_aspect_min = 0.0;
	std::vector<double> aspectArr;
};

// optimization()'s rounds and the per-vertex statistics they keep.
struct optimization_state {
	int stage = 0, index = 0, minAspIndx = 0, hexaIndx = 0, a_vert = 0;
	bool on_grid = false;
	double orientation = 1.0;
	float cond_max1 = 0.0f, cond_min = 0.0f, cond_max = 0.0f, pick = -1.0f;
	std::vector<float> conditionArr, jacobianArr, oddyArr;
	std::vector<char> stuck;
	std::vector<int> around;
};

struct octree_step_state {
	int op = 0;			// Octree::step()'s operation (octree.cpp's oct_op)
	int stage = 0;			// where it is
	std::size_t pos = 0;		// the stage's counter
	std::size_t sub = 0;		// ... and an inner one
	std::size_t seen = 0;		// cells the traversals have taken, for step_progress()
	int grow = 0;			// initMem()'s array being filled
	geoframe* g = nullptr;
	float err_tol = 0.0f;
	int method = 0;
	bool on_grid = false;

	// setVolume()
	std::shared_ptr<VolMagick::Volume> vol;
	std::vector<float> line;

	// collapse(): isovalue_off_samples()'s search
	float target = 0.0f, eps = 0.0f;
	bool first = true;

	// collapse() and traverse_qef()
	cell_levels q;
	int level = 0, pass = 0, boundary_level = 0, finest_boundary = 0;

	// mesh_extract()
	leaf_cell leaf;
	hexa_extract_state hx;
	untangle_state un;

	// quality_improve()
	flow_state flow;
	contraction_state ec;
	joeliu_state jl;
	optimization_state opt;
};

}

#endif
