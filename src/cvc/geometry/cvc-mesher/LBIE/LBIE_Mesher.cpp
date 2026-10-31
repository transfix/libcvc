#include <iostream>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

#include "LBIE_Mesher.h"

//libcontour headers
#include <contour.h>
#include <datasetreg3.h>
#include <libcontour_mutex.h>

namespace LBIE
{
  static inline unsigned int upToPowerOfTwo(unsigned int value)
  {
    unsigned int c = 0;
    unsigned int v = value;
  
    // round down to nearest power of two
    while (v>1) {
      v = v>>1;
      c++;
    }
  
    // if that isn't exactly the original value
    if ((v<<c)!=value) {
      // return the next power of two
      return (v<<(c+1));
    }
    else {
      // return this power of two
      return (v<<c);
    }
  }

  // The median length of the edges of g's triangles and quads, or 0 if it has none.
  static double medianEdgeLength(const geoframe& g)
  {
    std::vector<double> len;
    len.reserve(4*g.quads.size() + 3*g.triangles.size());
    for(size_t q=0; q<g.quads.size(); q++)
      for(int k=0; k<4; k++)
	{
	  const geoframe::float_3& a = g.verts[g.quads[q][k]];
	  const geoframe::float_3& b = g.verts[g.quads[q][(k+1)%4]];
	  len.push_back(std::sqrt(double(a[0]-b[0])*(a[0]-b[0]) +
				  double(a[1]-b[1])*(a[1]-b[1]) +
				  double(a[2]-b[2])*(a[2]-b[2])));
	}
    for(size_t t=0; t<g.triangles.size(); t++)
      for(int k=0; k<3; k++)
	{
	  const geoframe::float_3& a = g.verts[g.triangles[t][k]];
	  const geoframe::float_3& b = g.verts[g.triangles[t][(k+1)%3]];
	  len.push_back(std::sqrt(double(a[0]-b[0])*(a[0]-b[0]) +
				  double(a[1]-b[1])*(a[1]-b[1]) +
				  double(a[2]-b[2])*(a[2]-b[2])));
	}
    if(len.empty())
      return 0.0;
    std::nth_element(len.begin(), len.begin() + len.size()/2, len.end());
    return len[len.size()/2];
  }

  void Mesher::setVolume(const VolMagick::Volume& vol)
  {
    using namespace std;

    cerr << "LBIE::Mesher::setVolume(): set volume: "; 
    VolMagick::Volume localvol(vol);
    _contourExtractor.setVolume(vol);
    cerr << "done" << endl;

    _octree_volume_updated = false;
  }

  namespace
  {
    // Mesher::step()'s operations, and their stages: extractMesh()'s (with do_duallib_extract()'s
    // and set_octree_volume()'s) and qualityImprove()'s.
    enum { MOP_NONE, MOP_EXTRACT, MOP_IMPROVE, MOP_RELEASE };
    enum { MX_BEGIN, MX_DUAL, MX_LAYER_BEGIN, MX_LAYER, MX_RESAMPLE, MX_RESIZE, MX_SETVOL, MX_LAYER_SET,
	   MX_COLLAPSE, MX_QEF, MX_TRAVERSE, MX_EXTRACT, MX_IMPROVE, MX_WORLD, MX_EXTENTS, MX_DONE };
    enum { MQ_BEGIN, MQ_MAP, MQ_PASS, MQ_UNMAP, MQ_DONE };

    // a/b in [0,1] (1 when there is nothing to do)
    double frac(double a, double b)
    {
      return b > 0.0 ? std::min(1.0, a/b) : 1.0;
    }
  }

  // step() to the end of the operation.
  void Mesher::run_steps()
  {
    while(_st.op != MOP_NONE)
      {
	std::size_t budget = std::size_t(-1);
	step(budget);
      }
  }

  bool Mesher::step(std::size_t& budget)
  {
    bool done = true;
    if(_st.op == MOP_EXTRACT)
      done = step_extract(budget);
    else if(_st.op == MOP_IMPROVE)
      done = step_improve(budget);
    else if(_st.op == MOP_RELEASE)
      done = _octree.step(budget);
    if(done)
      _st.op = MOP_NONE;
    return done;
  }

  void Mesher::begin_release()
  {
    _st.op = MOP_RELEASE;
    _octree.begin_release();
  }

  void Mesher::begin_extract_mesh(const VolMagick::Volume& vol)
  {
    setVolume(vol);
    begin_extract();
  }

  void Mesher::begin_extract()
  {
    _st.op = MOP_EXTRACT;
    _st.stage = MX_BEGIN;
    _st.pos = 0;
    _st.started = false;
  }

  // extractMesh() a slice at a time. DUALLIB is do_duallib_extract(): the volume (TETRA2's layer
  // a voxel at a time, 4 a unit) resampled to (2^n+1)^3 (cvc::resize_stepper, 4 voxels a unit)
  // and set into the octree
  // (Octree::begin_set_volume()), then the octree's collapse, QEFs, traversal, extraction and
  // first quality improvement, then the vertices mapped to world coordinates and the mesh's
  // extents (64 vertices a unit). FASTCONTOURING and LIBISOCONTOUR extract in one piece.
  bool Mesher::step_extract(std::size_t& budget)
  {
    using namespace std;
    step_state& s = _st;

    while(budget > 0)
      switch(s.stage)
	{
	case MX_BEGIN:
	  switch(extractionMethod())
	    {
	    case DUALLIB:
	      s.stage = MX_DUAL;
	      break;
	    case FASTCONTOURING:
	      do_fastcontouring_extract();
	      s.stage = MX_EXTENTS;
	      break;
	    case LIBISOCONTOUR:
	      do_libisocontour_extract();
	      s.stage = MX_EXTENTS;
	      break;
	    default:
	      s.stage = MX_EXTENTS;
	      break;
	    }
	  s.pos = 0;
	  spend(budget, 1);
	  break;

	case MX_DUAL:
	  cerr << "LBIE::Mesher::extractMesh(): DUALLIB mesh extraction" << endl;

	  _geoframe.setSpan(_octree.spans[0],_octree.spans[1],_octree.spans[2]);
	  _geoframe.setMin(_octree.minext[0],_octree.minext[1],_octree.minext[2]);

	  if(_meshType != TETRA2 && !_octree_volume_updated)
	    {
	      s.vol.reset(new VolMagick::Volume(_contourExtractor.getVolume()));
	      s.next = MX_COLLAPSE;
	      s.stage = MX_RESAMPLE;
	    }
	  else if(_meshType == TETRA2)
	    s.stage = MX_LAYER_BEGIN;
	  else
	    s.stage = MX_COLLAPSE;
	  spend(budget, 1);
	  break;

	case MX_LAYER_BEGIN:
	  {
	    // A layer of tetrahedra between two isovalues lo < hi is the solid min(v - lo, hi - v) >= 0,
	    // whose boundary is both isosurfaces, meshed like any other solid. Octree::tetrahedralize_interval
	    // built the layer from the interval's octree directly, and wrongly: it chose which isosurface an
	    // edge crosses by its cell, not by the edge, gave an edge that crosses both a pyramid whose apex
	    // is outside the layer, and made a new grid vertex for every pyramid, so neighbouring tets did
	    // not share their corners.
	    s.lo = std::min(isovalue(), isovalue_in());
	    s.hi = std::max(isovalue(), isovalue_in());
	    s.vol.reset(new VolMagick::Volume(_contourExtractor.getVolume()));
	    const bool convert = s.vol->voxelType() != VolMagick::Float;
	    s.vol->voxelType(VolMagick::Float);
	    s.pos = 0;
	    s.stage = MX_LAYER;
	    spend(budget, convert ? std::size_t(s.vol->XDim()*s.vol->YDim()*s.vol->ZDim())/16 + 1 : 1);
	  }
	  break;

	case MX_LAYER:
	  {
	    VolMagick::Volume& layer = *s.vol;
	    const double lo = s.lo, hi = s.hi;
	    const cvc::uint64 n = layer.XDim()*layer.YDim()*layer.ZDim();
	    const cvc::uint64 end = budget >= (n - s.pos + 3)/4 ? n : s.pos + cvc::uint64(budget)*4;
	    for(cvc::uint64 i = s.pos; i < end; i++)
	      {
		const double v = layer(i);
		layer(i, std::min(v - lo, hi - v));
	      }
	    spend(budget, std::size_t((end - s.pos + 3)/4));
	    s.pos = end;
	    if(s.pos >= n)
	      {
		s.next = MX_LAYER_SET;
		s.stage = MX_RESAMPLE;
	      }
	  }
	  break;

	case MX_RESAMPLE:
	  {
	    // set_octree_volume()
	    cerr << "LBIE::Mesher::set_octree_volume(): set volume: ";
	    //LBIE requires volumes with dimension (2^n+1)^3: any other is resized (resize()'s
	    //voxels, a range at a time) first
	    const VolMagick::Volume& vol = *s.vol;
	    unsigned int dim[3] = { static_cast<unsigned int>(vol.XDim()), static_cast<unsigned int>(vol.YDim()), static_cast<unsigned int>(vol.ZDim()) };
	    unsigned int maxdim = *std::max_element(dim,dim+3);
	    if((dim[0] != dim[1]) ||
	       (dim[0] != dim[2]) ||
	       upToPowerOfTwo(maxdim-1) != (maxdim-1))
	      {
		s.resize = std::make_shared<cvc::resize_stepper>(vol, VolMagick::Dimension(upToPowerOfTwo(maxdim)+1,
										   upToPowerOfTwo(maxdim)+1,
										   upToPowerOfTwo(maxdim)+1));
		s.stage = MX_RESIZE;
	      }
	    else
	      {
		_octree.begin_set_volume(vol);
		s.stage = MX_SETVOL;
	      }
	    s.vol.reset();
	    spend(budget, 1);
	  }
	  break;

	case MX_RESIZE:
	  {
	    // 4 voxels a unit; a bounded step that finishes it ends there, an unbounded one (run to
	    // the end) carries on
	    const bool unbounded = budget >= std::numeric_limits<std::size_t>::max()/4;
	    if(s.resize->step(unbounded ? 0 : budget*4))
	      {
		_octree.begin_set_volume(s.resize->take());
		s.resize.reset();
		s.stage = MX_SETVOL;
	      }
	    spend(budget, unbounded ? 1 : budget);
	  }
	  break;

	case MX_SETVOL:
	  if(_octree.step(budget))
	    {
	      cerr << "done" << endl;
	      _octree_volume_updated = true;
	      s.stage = s.next;
	    }
	  break;

	case MX_LAYER_SET:
	  _octree_volume_updated = false; // the octree holds the layer, not the volume
	  _octree.set_isovalue(0.0f);
	  _octree.set_isovalue_in(-FLT_MAX); // no inner isovalue: min_vtx() asks it of a cell
	  _octree.setMeshType(int(TETRA));
	  s.stage = MX_COLLAPSE;
	  break;

	case MX_COLLAPSE:
	  if(_meshType != TETRA2 && _dual)
	    {
	      // the interval's octree, in one piece
	      _octree.collapse_interval();
	      cerr << "LBIE::Mesher::extractMesh(): finished collapse interval" << endl;
	      _octree.compute_qef_interval();
	      cerr << "LBIE::Mesher::extractMesh(): finished compute qef interval" << endl;
	      _octree.traverse_qef_interval(_err,_err_in);
	      cerr << "LBIE::Mesher::extractMesh(): finished traverse qef interval" << endl;
	      s.stage = MX_EXTRACT;
	      spend(budget, 1);
	      break;
	    }
	  if(!s.started)
	    {
	      _octree.begin_collapse();
	      s.started = true;
	    }
	  if(_octree.step(budget))
	    {
	      s.started = false;
	      cerr << "LBIE::Mesher::extractMesh(): finished collapse" << endl;
	      s.stage = MX_QEF;
	    }
	  break;

	case MX_QEF:
	  if(!s.started)
	    {
	      _octree.begin_compute_qef();
	      s.started = true;
	    }
	  if(_octree.step(budget))
	    {
	      s.started = false;
	      cerr << "LBIE::Mesher::extractMesh(): finished compute qef" << endl;
	      s.stage = MX_TRAVERSE;
	    }
	  break;

	case MX_TRAVERSE:
	  if(!s.started)
	    {
	      _octree.begin_traverse_qef(_err);
	      s.started = true;
	    }
	  if(_octree.step(budget))
	    {
	      s.started = false;
	      cerr << "LBIE::Mesher::extractMesh(): finished traverse qef" << endl;
	      s.stage = MX_EXTRACT;
	    }
	  break;

	case MX_EXTRACT:
	  if(!s.started)
	    {
	      _octree.begin_mesh_extract(_geoframe,_err);
	      s.started = true;
	    }
	  if(_octree.step(budget))
	    {
	      s.started = false;
	      cerr << "LBIE::Mesher::extractMesh(): finished mesh extraction" << endl;
	      if(_meshType == TETRA2)
		{
		  _octree.set_isovalue(_isovalue);
		  _octree.set_isovalue_in(_isovalue_in);
		  _octree.setMeshType(int(_meshType));
		}
	      s.stage = MX_IMPROVE;
	    }
	  break;

	case MX_IMPROVE:
	  if(!s.started)
	    {
	      _octree.begin_quality_improve(_geoframe,_improveMethod,true);
	      s.started = true;
	    }
	  if(_octree.step(budget))
	    {
	      s.started = false;
	      _geoframe.mesh_type = geoframe::GEOTYPE(_meshType);
	      s.pos = 0;
	      s.stage = MX_WORLD;
	    }
	  break;

	case MX_WORLD:
	  //re-adjust vertices so they're back to their original location
	  while(budget > 0 && s.pos < std::size_t(std::max(_geoframe.numverts, 0)))
	    {
	      const std::size_t end = std::min(std::size_t(_geoframe.numverts), s.pos + 64);
	      for(int i=int(s.pos); i<int(end); i++)
		for(int j=0; j<3; j++)
		  _geoframe.verts[i][j] =
		    _octree.minext[j] + _octree.spans[j] * _geoframe.verts[i][j];
	      s.pos = end;
	      spend(budget, 1);
	    }
	  if(s.pos >= std::size_t(std::max(_geoframe.numverts, 0)))
	    {
	      _mesh_from_octree = true;
	      s.pos = 0;
	      s.stage = MX_EXTENTS;
	    }
	  break;

	case MX_EXTENTS:
	  // _geoframe.calculateExtents(), 64 vertices a unit
	  do
	    {
	      const int from = int(s.pos), to = std::min(_geoframe.numverts, from + 64);
	      _geoframe.calculateExtents(from, to);
	      s.pos = std::size_t(std::max(to, from));
	      spend(budget, 1);
	    }
	  while(budget > 0 && int(s.pos) < _geoframe.numverts);
	  if(int(s.pos) >= _geoframe.numverts)
	    s.stage = MX_DONE;
	  break;

	default:
	  return true;
	}
    return s.stage == MX_DONE;
  }

  void Mesher::do_fastcontouring_extract()
  {
    using namespace std;

    cerr << "LBIE::Mesher::extractMesh(): FASTCONTOURING mesh extraction" << endl;

    _geoframe.reset();
    _mesh_from_octree = false;
    meshType(SINGLE);
    _geoframe.mesh_type = geoframe::GEOTYPE(_meshType);

    _geoframe.setSpan(_octree.spans[0],_octree.spans[1],_octree.spans[2]);
    _geoframe.setMin(_octree.minext[0],_octree.minext[1],_octree.minext[2]);

    FastContouring::TriSurf result = _contourExtractor.extractContour(isovalue());

    //convert the result to a geoframe
    _geoframe.numverts = result.verts.size()/3;
    _geoframe.numtris = result.tris.size()/3;
    
    _geoframe.verts.resize(_geoframe.numverts);
    _geoframe.normals.resize(_geoframe.numverts);
    _geoframe.color.resize(_geoframe.numverts);
    _geoframe.triangles.resize(_geoframe.numtris);

    for(int i=0;i<_geoframe.numverts;i++)
      {
	for(int j=0; j<3; j++)
	  _geoframe.verts[i][j] = result.verts[i*3+j];
	for(int j=0; j<3; j++)
	  _geoframe.normals[i][j] = result.normals[i*3+j];
	for(int j=0; j<3; j++)
	  _geoframe.color[i][j] = result.colors[i*3+j];
      }
    for (int i=0;i<_geoframe.numtris;i++)
      for(int j=0; j<3; j++)
	_geoframe.triangles[i][j] = result.tris[i*3+j];
  }

  void Mesher::do_libisocontour_extract()
  {
    using namespace std;

    cerr << "LBIE::Mesher::extractMesh(): LIBISOCONTOUR mesh extraction" << endl;

    _geoframe.reset();
    _mesh_from_octree = false;
    meshType(SINGLE);
    _geoframe.mesh_type = geoframe::GEOTYPE(_meshType);

    _geoframe.setSpan(_octree.spans[0],_octree.spans[1],_octree.spans[2]);
    _geoframe.setMin(_octree.minext[0],_octree.minext[1],_octree.minext[2]);

    ConDataset* the_data;
    Contour3dData *contour3d;
    int dim[3];
    float span[3], orig[3];

    //just grab the volume from _contourExtractor
    VolMagick::Volume vol(_contourExtractor.getVolume());

    dim[0] = vol.XDim(); dim[1] = vol.YDim(); dim[2] = vol.ZDim();
    span[0] = vol.XSpan(); span[1] = vol.YSpan(); span[2] = vol.ZSpan();
    orig[0] = vol.XMin(); orig[1] = vol.YMin(); orig[2] = vol.ZMin();

    //libcontour is not re-entrant (libcontour_mutex.h): hold its lock from the dataset's
    //creation through the deletes below
    std::lock_guard<std::mutex> lock(cvc::libcontour_mutex());

    //convert it to a supported libcontour type and load it into libisocontour
    switch(vol.voxelType())
      {
      case VolMagick::UInt:
      case VolMagick::Double:
      case VolMagick::UInt64:
      case VolMagick::Float:
	vol.voxelType(VolMagick::Float);
	the_data = newDatasetReg(CONTOUR_FLOAT, CONTOUR_REG_3D, 1, 1, dim, *vol);
	break;
      case VolMagick::UChar:
	the_data = newDatasetReg(CONTOUR_UCHAR, CONTOUR_REG_3D, 1, 1, dim, *vol);
	break;
      case VolMagick::UShort:
	the_data = newDatasetReg(CONTOUR_USHORT, CONTOUR_REG_3D, 1, 1, dim, *vol);
	break;
      }

    ((Datareg3 *)the_data->data->getData(0))->setOrig(orig);
    ((Datareg3 *)the_data->data->getData(0))->setSpan(span);

    //actually extract the contour
    contour3d = getContour3d(the_data,0,0,isovalue(),NO_COLOR_VARIABLE);

    //convert the result to a geoframe
    _geoframe.numverts = contour3d->nvert;
    _geoframe.numtris = contour3d->ntri;
    
    _geoframe.verts.resize(_geoframe.numverts);
    _geoframe.normals.resize(_geoframe.numverts);
    _geoframe.color.resize(_geoframe.numverts);
    _geoframe.triangles.resize(_geoframe.numtris);

    for(int i=0;i<_geoframe.numverts;i++)
      {
	for(int j=0; j<3; j++)
	  _geoframe.verts[i][j] = contour3d->vert[i][j];
	for(int j=0; j<3; j++)
	  _geoframe.normals[i][j] = contour3d->vnorm[i][j];
	for(int j=0; j<3; j++)
	  _geoframe.color[i][j] = 1.0;
      }
    for (int i=0;i<_geoframe.numtris;i++)
      for(int j=0; j<3; j++)
	_geoframe.triangles[i][j] = contour3d->tri[i][j];

    delete the_data;
    delete contour3d;
  }

  geoframe& Mesher::extractMesh(const VolMagick::Volume& vol)
  {
    begin_extract_mesh(vol);
    run_steps();
    return _geoframe;
  }

  geoframe& Mesher::extractMesh()
  {
    begin_extract();
    run_steps();
    return _geoframe;
  }

  geoframe& Mesher::qualityImprove(unsigned int iterations)
  {
    begin_quality_improve(iterations);
    run_steps();
    return _geoframe;
  }

  void Mesher::begin_quality_improve(unsigned int iterations)
  {
    _st.op = MOP_IMPROVE;
    _st.stage = MQ_BEGIN;
    _st.pos = 0;
    _st.started = false;
    _st.iterations = iterations;
    _st.pass = 0;
  }

  // qualityImprove() a slice at a time: the mesh mapped onto a grid (a unit a vertex), the passes
  // (Octree::begin_quality_improve()), and the vertices mapped back (a unit a vertex).
  bool Mesher::step_improve(std::size_t& budget)
  {
    using namespace std;
    step_state& s = _st;

    while(budget > 0)
      switch(s.stage)
	{
	case MQ_BEGIN:
	  {
	    const unsigned int iterations = s.iterations;
	    std::size_t units = 1;

    //make sure the mesh type is the same as the geoframe
    if(_geoframe.mesh_type != int(_meshType))
      meshType(MeshType(_geoframe.mesh_type));

    // The improvement passes are written for grid units: optimization's first step is half a grid
    // cell, and geometric flow on tetrahedra and hexahedra skips elements with a volume under 1e-5.
    // do_duallib_extract runs a pass on the octree's grid and then maps the vertices to world
    // coordinates, so the passes here used to get world coordinates. On the bunny, whose cells are
    // 0.01 across, optimization threw a vertex far outside the volume, and geometric flow moved only
    // 178 of 592 vertices. Map a mesh from the octree back onto its grid for the passes. Any other
    // mesh gets a grid whose cell is its median edge, so that it is improved the same way in any
    // units.
    double* origin = s.origin;
    double* cell = s.cell;
    for(int j=0; j<3; j++)
      {
	origin[j] = 0.0;
	cell[j] = 1.0;
      }
    bool& on_grid = s.on_grid;
    bool& mapped = s.mapped;
    on_grid = mapped = false;
    if(iterations > 0 && _improveMethod != NO_IMPROVE && _geoframe.numverts > 0)
      {
	if(_mesh_from_octree &&
	   _octree.spans[0] > 0 && _octree.spans[1] > 0 && _octree.spans[2] > 0)
	  {
	    for(int j=0; j<3; j++)
	      {
		origin[j] = _octree.minext[j];
		cell[j] = _octree.spans[j];
	      }
	    on_grid = mapped = true;
	  }
	else
	  {
	    const double h = medianEdgeLength(_geoframe);
	    units += std::size_t(_geoframe.numverts)/16;
	    if(h > 0.0 && std::isfinite(h))
	      {
		for(int j=0; j<3; j++)
		  {
		    origin[j] = HUGE_VAL;
		    for(int i=0; i<_geoframe.numverts; i++)
		      if(_geoframe.verts[i][j] < origin[j])
			origin[j] = _geoframe.verts[i][j];
		    if(!std::isfinite(origin[j]))
		      origin[j] = 0.0;
		    cell[j] = h;
		  }
		mapped = true;
	      }
	  }
      }

	    s.world.clear();
	    s.grid.clear();
	    s.pos = 0;
	    s.stage = mapped ? MQ_MAP : MQ_PASS;
	    spend(budget, units);
	  }
	  break;

	case MQ_MAP:
	  // world = the vertices; the vertices mapped onto the grid; grid = the vertices
	  while(budget > 0 && s.pos < _geoframe.verts.size())
	    {
	      const std::size_t end = std::min(_geoframe.verts.size(), s.pos + 16);
	      for(std::size_t i = s.pos; i < end; i++)
		{
		  s.world.push_back(_geoframe.verts[i]);
		  if(int(i) < _geoframe.numverts)
		    for(int j=0; j<3; j++)
		      _geoframe.verts[i][j] = float((s.world[i][j] - s.origin[j]) / s.cell[j]);
		  s.grid.push_back(_geoframe.verts[i]);
		}
	      s.pos = end;
	      spend(budget, 1);
	    }
	  if(s.pos >= _geoframe.verts.size())
	    s.stage = MQ_PASS;
	  break;

	case MQ_PASS:
	  if(s.pass >= s.iterations)
	    {
	      s.pos = 0;
	      s.stage = s.mapped ? MQ_UNMAP : MQ_DONE;
	      break;
	    }
	  if(!s.started)
	    {
	      _octree.begin_quality_improve(_geoframe,_improveMethod,s.on_grid);
	      s.started = true;
	    }
	  if(_octree.step(budget))
	    {
	      s.started = false;
	      cerr <<
		"LBIE::Mesher::qualityImprove(): "
		"Quality improvement iteration " << s.pass << " finished." << endl;
	      s.pass++;
	    }
	  break;

	case MQ_UNMAP:
	  // A vertex the passes did not move gets its world coordinates back exactly, not after a
	  // round trip through the grid.
	  while(budget > 0 && int(s.pos) < _geoframe.numverts)
	    {
	      const int end = std::min(_geoframe.numverts, int(s.pos) + 16);
	      for(int i=int(s.pos); i<end; i++)
		{
		  const bool moved = size_t(i) >= s.grid.size() || _geoframe.verts[i] != s.grid[i];
		  for(int j=0; j<3; j++)
		    _geoframe.verts[i][j] = moved ?
		      float(s.origin[j] + s.cell[j] * _geoframe.verts[i][j]) : s.world[i][j];
		}
	      s.pos = std::size_t(end);
	      spend(budget, 1);
	    }
	  if(int(s.pos) >= _geoframe.numverts)
	    {
	      s.world = std::vector<geoframe::float_3>();
	      s.grid = std::vector<geoframe::float_3>();
	      s.stage = MQ_DONE;
	    }
	  break;

	default:
	  return true;
	}
    return s.stage == MQ_DONE;
  }

  const char* Mesher::step_phase() const
  {
    if(_st.op == MOP_EXTRACT)
      switch(_st.stage)
	{
	case MX_BEGIN: case MX_DUAL: case MX_LAYER_BEGIN: case MX_LAYER: case MX_RESAMPLE:
	  return "preparing";
	case MX_RESIZE:
	  return "resampling";
	case MX_SETVOL:
	  return _octree.step_phase();
	case MX_LAYER_SET: case MX_COLLAPSE:
	  return _st.started ? _octree.step_phase() : "collapse";
	case MX_QEF: case MX_TRAVERSE: case MX_EXTRACT: case MX_IMPROVE:
	  return _st.started ? _octree.step_phase() : "extracting";
	case MX_WORLD: case MX_EXTENTS:
	  return "output";
	default:
	  return "done";
	}
    if(_st.op == MOP_IMPROVE)
      switch(_st.stage)
	{
	case MQ_PASS:
	  return _st.started ? _octree.step_phase() : "improving";
	case MQ_DONE:
	  return "done";
	default:
	  return "output";
	}
    if(_st.op == MOP_RELEASE)
      return "output";
    return "done";
  }

  double Mesher::step_progress() const
  {
    const double oct = _st.started ? _octree.step_progress() : 0.0;
    if(_st.op == MOP_EXTRACT)
      switch(_st.stage)
	{
	case MX_BEGIN: case MX_DUAL: case MX_LAYER_BEGIN: return 0.0;
	case MX_LAYER: return 0.04*frac(double(_st.pos), _st.vol ? double(_st.vol->XDim()*_st.vol->YDim()*_st.vol->ZDim()) : 0.0);
	case MX_RESAMPLE: return 0.04;
	case MX_RESIZE: return 0.04 + 0.01*(_st.resize ? _st.resize->progress() : 1.0);
	case MX_SETVOL: return 0.05 + 0.2*_octree.step_progress();
	case MX_LAYER_SET: return 0.25;
	case MX_COLLAPSE: return 0.25 + 0.1*oct;
	case MX_QEF: return 0.35 + 0.1*oct;
	case MX_TRAVERSE: return 0.45 + 0.1*oct;
	case MX_EXTRACT: return 0.55 + 0.25*oct;
	case MX_IMPROVE: return 0.8 + 0.15*oct;
	case MX_WORLD: return 0.95 + 0.03*frac(double(_st.pos), _geoframe.numverts);
	case MX_EXTENTS: return 0.98 + 0.02*frac(double(_st.pos), _geoframe.numverts);
	default: return 1.0;
	}
    if(_st.op == MOP_IMPROVE)
      switch(_st.stage)
	{
	case MQ_BEGIN: return 0.0;
	case MQ_MAP: return 0.05*frac(double(_st.pos), double(_geoframe.verts.size()));
	case MQ_PASS: return 0.05 + 0.9*frac(_st.pass + oct, _st.iterations);
	case MQ_UNMAP: return 0.95 + 0.05*frac(double(_st.pos), _geoframe.numverts);
	default: return 1.0;
	}
    if(_st.op == MOP_RELEASE)
      return _octree.step_progress();
    return 1.0;
  }

  void Mesher::interpolateProperties(const VolMagick::Volume& propertyVol)
  {
    using namespace std;
    
    cerr << "LBIE::Mesher::interpolateProperties(): "
         << "Interpolating property values to mesh vertices..." << endl;
    
    // Enable property interpolation and call func_val
    _octree.set_prop_flag(true);
    _octree.func_val(_geoframe, propertyVol);
    
    cerr << "LBIE::Mesher::interpolateProperties(): "
         << "Property interpolation complete. "
         << _geoframe.funcs.size() << " values assigned." << endl;
  }
}
