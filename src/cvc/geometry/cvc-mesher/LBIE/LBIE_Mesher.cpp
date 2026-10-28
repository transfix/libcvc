#include <iostream>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>

#include "LBIE_Mesher.h"

//libcontour headers
#include <contour.h>
#include <datasetreg3.h>

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

  void Mesher::set_octree_volume(const VolMagick::Volume &vol)
  {
    using namespace std;

    VolMagick::Volume localvol(vol);
    
    cerr << "LBIE::Mesher::set_octree_volume(): set volume: ";
    //LBIE requires volumes with dimension (2^n+1)^3
    unsigned int dim[3] = { static_cast<unsigned int>(localvol.XDim()), static_cast<unsigned int>(localvol.YDim()), static_cast<unsigned int>(localvol.ZDim()) };
    unsigned int maxdim = *std::max_element(dim,dim+3);
    if((dim[0] != dim[1]) ||
       (dim[0] != dim[2]) ||
       upToPowerOfTwo(maxdim-1) != (maxdim-1))
      localvol.resize(VolMagick::Dimension(upToPowerOfTwo(maxdim)+1,
					   upToPowerOfTwo(maxdim)+1,
					   upToPowerOfTwo(maxdim)+1));
    _octree.setVolume(localvol);
    cerr << "done" << endl;

    _octree_volume_updated = true;
  }

  void Mesher::do_duallib_extract()
  {
    using namespace std;

    cerr << "LBIE::Mesher::extractMesh(): DUALLIB mesh extraction" << endl;
    
    _geoframe.setSpan(_octree.spans[0],_octree.spans[1],_octree.spans[2]);
    _geoframe.setMin(_octree.minext[0],_octree.minext[1],_octree.minext[2]);
    
    if(_meshType != TETRA2 && !_octree_volume_updated)
      set_octree_volume(_contourExtractor.getVolume());
    
    if(_meshType == TETRA2)
      {
	// A layer of tetrahedra between two isovalues lo < hi is the solid min(v - lo, hi - v) >= 0,
	// whose boundary is both isosurfaces, meshed like any other solid. Octree::tetrahedralize_interval
	// built the layer from the interval's octree directly, and wrongly: it chose which isosurface an
	// edge crosses by its cell, not by the edge, gave an edge that crosses both a pyramid whose apex
	// is outside the layer, and made a new grid vertex for every pyramid, so neighbouring tets did
	// not share their corners.
	const double lo = std::min(isovalue(), isovalue_in()), hi = std::max(isovalue(), isovalue_in());
	VolMagick::Volume layer(_contourExtractor.getVolume());
	layer.voxelType(VolMagick::Float);
	const cvc::uint64 n = layer.XDim()*layer.YDim()*layer.ZDim();
	for(cvc::uint64 i = 0; i < n; i++)
	  {
	    const double v = layer(i);
	    layer(i, std::min(v - lo, hi - v));
	  }
	set_octree_volume(layer);
	_octree_volume_updated = false; // the octree holds the layer, not the volume
	_octree.set_isovalue(0.0f);
	_octree.set_isovalue_in(-FLT_MAX); // no inner isovalue: min_vtx() asks it of a cell
	_octree.setMeshType(int(TETRA));
	_octree.collapse();
	cerr << "LBIE::Mesher::extractMesh(): finished collapse" << endl;
	_octree.compute_qef();
	cerr << "LBIE::Mesher::extractMesh(): finished compute qef" << endl;
	_octree.traverse_qef(_err);
	cerr << "LBIE::Mesher::extractMesh(): finished traverse qef" << endl;
	_octree.mesh_extract(_geoframe,_err);
	cerr << "LBIE::Mesher::extractMesh(): finished mesh extraction" << endl;
	_octree.set_isovalue(_isovalue);
	_octree.set_isovalue_in(_isovalue_in);
	_octree.setMeshType(int(_meshType));
      }
    else if(!_dual)
      {
	_octree.collapse();
	cerr << "LBIE::Mesher::extractMesh(): finished collapse" << endl;
	_octree.compute_qef();
	cerr << "LBIE::Mesher::extractMesh(): finished compute qef" << endl;
	_octree.traverse_qef(_err);
	cerr << "LBIE::Mesher::extractMesh(): finished traverse qef" << endl;
	_octree.mesh_extract(_geoframe,_err);
	cerr << "LBIE::Mesher::extractMesh(): finished mesh extraction" << endl;
      }
    else
      {
	_octree.collapse_interval();
	cerr << "LBIE::Mesher::extractMesh(): finished collapse interval" << endl;
	_octree.compute_qef_interval();
	cerr << "LBIE::Mesher::extractMesh(): finished compute qef interval" << endl;
	_octree.traverse_qef_interval(_err,_err_in);
	cerr << "LBIE::Mesher::extractMesh(): finished traverse qef interval" << endl;
	_octree.mesh_extract(_geoframe,_err);
	cerr << "LBIE::Mesher::extractMesh(): finished mesh extraction" << endl;
      }
    
    _octree.quality_improve(_geoframe,_improveMethod,true);
    _geoframe.mesh_type = geoframe::GEOTYPE(_meshType);
    
    //re-adjust vertices so they're back to their original location
    for(int i=0; i<_geoframe.numverts; i++)
      for(int j=0; j<3; j++)
	_geoframe.verts[i][j] =
	  _octree.minext[j] + _octree.spans[j] * _geoframe.verts[i][j];
    _mesh_from_octree = true;
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
    setVolume(vol);

    switch(extractionMethod())
      {
      case DUALLIB:
	do_duallib_extract();
	break;
      case FASTCONTOURING:
	do_fastcontouring_extract();
	break;
      case LIBISOCONTOUR:
	do_libisocontour_extract();
	break;
      }

    _geoframe.calculateExtents();
    return _geoframe;
  }

  geoframe& Mesher::extractMesh()
  {
    switch(extractionMethod())
      {
      case DUALLIB:
	do_duallib_extract();
	break;
      case FASTCONTOURING:
	do_fastcontouring_extract();
	break;
      case LIBISOCONTOUR:
	do_libisocontour_extract();
	break;
      }

    _geoframe.calculateExtents();
    return _geoframe;
  }

  geoframe& Mesher::qualityImprove(unsigned int iterations)
  {
    using namespace std;

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
    double origin[3] = {0.0, 0.0, 0.0}, cell[3] = {1.0, 1.0, 1.0};
    bool on_grid = false, mapped = false;
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

    std::vector<geoframe::float_3> world, grid;
    if(mapped)
      {
	world = _geoframe.verts;
	for(int i=0; i<_geoframe.numverts; i++)
	  for(int j=0; j<3; j++)
	    _geoframe.verts[i][j] = float((world[i][j] - origin[j]) / cell[j]);
	grid = _geoframe.verts;
      }

    for(unsigned int i = 0; i < iterations; i++)
      {
	_octree.quality_improve(_geoframe,_improveMethod,on_grid);
	cerr << 
	  "LBIE::Mesher::qualityImprove(): "
	  "Quality improvement iteration " << i << " finished." << endl;
      }

    // A vertex the passes did not move gets its world coordinates back exactly, not after a round
    // trip through the grid.
    if(mapped)
      for(int i=0; i<_geoframe.numverts; i++)
	{
	  const bool moved = size_t(i) >= grid.size() || _geoframe.verts[i] != grid[i];
	  for(int j=0; j<3; j++)
	    _geoframe.verts[i][j] = moved ?
	      float(origin[j] + cell[j] * _geoframe.verts[i][j]) : world[i][j];
	}
    return _geoframe;
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
