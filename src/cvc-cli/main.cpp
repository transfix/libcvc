/*
  Copyright 2007-2025 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvc - Unified CLI for libcvc
//
// Consolidates the old libcvc CLI, volutils, and geometry/volume
// processing commands into a single tool with subcommands.

#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/utility/algorithm.h>
#include <cvc/utility/utility.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_ops.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static cvc::app &cvc_app() {
  static cvc::app app;
  return app;
}

static std::string type_to_string(cvc::data_type t) {
  return std::string(cvc::data_type_strings[t]);
}

static cvc::data_type string_to_type(const std::string &s) {
  static const struct {
    const char *alias;
    cvc::data_type dt;
  } type_map[] = {
      {"UChar", cvc::UChar},           {"unsigned char", cvc::UChar},
      {"UShort", cvc::UShort},         {"unsigned short", cvc::UShort},
      {"UInt", cvc::UInt},             {"unsigned int", cvc::UInt},
      {"Float", cvc::Float},           {"float", cvc::Float},
      {"Double", cvc::Double},         {"double", cvc::Double},
  };
  for (const auto &t : type_map)
    if (s == t.alias)
      return t.dt;
  throw std::runtime_error("Unknown voxel type: " + s +
                           ". Valid: UChar, UShort, UInt, Float, Double");
}

// ---------------------------------------------------------------------------
// File information commands
// ---------------------------------------------------------------------------

static int cmd_info(int argc, char **argv) {
  po::options_description desc("cvc info - display file metadata (volume or geometry)");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input file");

  po::positional_options_description pos;
  pos.add("input", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto &app = cvc_app();
  std::string input = vm["input"].as<std::string>();

  if (cvc::is_geometry_filename(input)) {
    cvc::geometry geo(app, input);
    if (geo.num_points() > 0)
      std::cout << "Vertices:  " << geo.num_points() << "\n";
    if (geo.num_lines() > 0)
      std::cout << "Lines:     " << geo.num_lines() << "\n";
    if (geo.num_tris() > 0)
      std::cout << "Triangles: " << geo.num_tris() << "\n";
    if (geo.num_quads() > 0)
      std::cout << "Quads:     " << geo.num_quads() << "\n";

    auto pmin = geo.min_point();
    auto pmax = geo.max_point();
    std::cout << "BBox min:  " << pmin[0] << ", " << pmin[1] << ", " << pmin[2] << "\n";
    std::cout << "BBox max:  " << pmax[0] << ", " << pmax[1] << ", " << pmax[2] << "\n";
  } else if (cvc::is_volume_filename(input)) {
    cvc::volume_file_info vfi;
    vfi.read(app, input);
    std::cout << "File:       " << vfi.filename() << "\n"
              << "Dimensions: " << vfi.XDim() << " x " << vfi.YDim() << " x " << vfi.ZDim() << "\n"
              << "BBox:       [" << vfi.boundingBox().minx << ", " << vfi.boundingBox().miny
              << ", " << vfi.boundingBox().minz << "] - [" << vfi.boundingBox().maxx << ", "
              << vfi.boundingBox().maxy << ", " << vfi.boundingBox().maxz << "]\n"
              << "Span:       " << vfi.XSpan() << " x " << vfi.YSpan() << " x " << vfi.ZSpan() << "\n"
              << "Variables:  " << vfi.numVariables() << "\n"
              << "Timesteps:  " << vfi.numTimesteps() << "\n";

    for (unsigned v = 0; v < vfi.numVariables(); ++v) {
      std::cout << "  Var " << v << ": name=" << vfi.name(v)
                << " type=" << vfi.voxelTypeStr(v);
      for (unsigned t = 0; t < vfi.numTimesteps(); ++t) {
        std::cout << " min=" << vfi.min(v, t) << " max=" << vfi.max(v, t);
      }
      std::cout << "\n";
    }
  } else {
    throw std::runtime_error("Unknown file type: " + input);
  }
  return 0;
}

static int cmd_stats(int argc, char **argv) {
  po::options_description desc("cvc stats - compute volume statistics");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file");

  po::positional_options_description pos;
  pos.add("input", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume_stats s = cvc::compute_stats(vol);

  std::cout << std::setprecision(12)
            << "Min:     " << s.min << "\n"
            << "Max:     " << s.max << "\n"
            << "Mean:    " << s.mean << "\n"
            << "StdDev:  " << s.std_dev << "\n"
            << "Voxels:  " << s.num_voxels << "\n";
  return 0;
}

// ---------------------------------------------------------------------------
// File conversion / copy
// ---------------------------------------------------------------------------

static int cmd_copy(int argc, char **argv) {
  po::options_description desc("cvc copy - copy/convert files (auto-detects volume or geometry)");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input file")
    ("output,o", po::value<std::string>()->required(), "output file");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::save(app, cvc::load(app, vm["input"].as<std::string>()),
            vm["output"].as<std::string>());
  return 0;
}

static int cmd_convert(int argc, char **argv) {
  po::options_description desc("cvc convert - convert between volume formats or voxel types");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output volume file")
    ("type,t", po::value<std::string>(),
     "output voxel type (UChar, UShort, UInt, Float, Double)");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  if (vm.count("type"))
    vol.voxelType(string_to_type(vm["type"].as<std::string>()));
  vol.write(vm["output"].as<std::string>());
  return 0;
}

// ---------------------------------------------------------------------------
// Geometry commands  (SDF, isosurface, tetrahedralize, hexahedralize)
// ---------------------------------------------------------------------------

#ifdef CVC_ENABLE_SDF
static int cmd_sdf(int argc, char **argv) {
  po::options_description desc(
    "cvc sdf - compute signed distance field from geometry\n\n"
    "Algorithms:\n"
    "  v1  Original SDFLibrary (octree-based, thread-safe) [default]\n"
    "  v2  DistanceTransform (brute-force)\n");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input geometry file")
    ("output,o", po::value<std::string>()->required(), "output volume file")
    ("dim,d", po::value<std::string>()->default_value("64,64,64"),
     "output dimensions (NxNxN or X,Y,Z)")
    ("bbox,b", po::value<std::string>(),
     "bounding box (minx,miny,minz,maxx,maxy,maxz); defaults to geometry extents")
    ("algorithm,a", po::value<std::string>()->default_value("v1"),
     "SDF algorithm: v1 or v2")
    ("flip-normals", "flip normals to invert inside/outside");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::geometry geom(app, vm["input"].as<std::string>());

  // Parse dimension string
  cvc::dimension dim(vm["dim"].as<std::string>());

  // Parse bounding box
  cvc::bounding_box bbox = geom.extents();
  if (vm.count("bbox"))
    bbox = cvc::bounding_box(vm["bbox"].as<std::string>());

  // Parse algorithm
  cvc::sdf_algorithm algo = cvc::SDF_V1;
  std::string algo_str = vm["algorithm"].as<std::string>();
  if (algo_str == "v2" || algo_str == "V2")
    algo = cvc::SDF_V2;

  bool flip = vm.count("flip-normals") > 0;

  cvc::volume result = cvc::sdf(app, geom, dim, bbox, algo, flip);
  result.write(vm["output"].as<std::string>());
  std::cout << "Wrote SDF volume to " << vm["output"].as<std::string>() << "\n";
  return 0;
}
#endif // CVC_ENABLE_SDF

#ifdef CVC_ENABLE_MESHER
static int cmd_iso(int argc, char **argv) {
  po::options_description desc(
    "cvc iso - extract isosurface geometry from a volume\n\n"
    "Extraction methods:\n"
    "  duallib         Dual contouring library [default]\n"
    "  fastcontouring  Fast contouring\n"
    "  libisocontour   ISO contouring library\n\n"
    "Normal types:\n"
    "  bspline-conv    B-spline convolution [default]\n"
    "  central-diff    Central difference\n"
    "  bspline-interp  B-spline interpolation\n");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output geometry file")
    ("isovalue,v", po::value<double>()->required(), "isovalue")
    ("method,m", po::value<std::string>()->default_value("duallib"),
     "extraction method")
    ("improve,q", po::value<int>()->default_value(0),
     "quality improvement iterations (0 = none)")
    ("normals,n", po::value<std::string>()->default_value("bspline-conv"),
     "normal computation method")
    ("property-vol,p", po::value<std::string>(),
     "optional property volume for interpolation");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());

  // Parse extraction method
  cvc::extraction_method method = cvc::DUALLIB;
  std::string mstr = vm["method"].as<std::string>();
  if (mstr == "fastcontouring") method = cvc::FASTCONTOURING;
  else if (mstr == "libisocontour") method = cvc::LIBISOCONTOUR;

  // Parse normal type
  cvc::normal_type normals = cvc::BSPLINE_CONVOLUTION;
  std::string nstr = vm["normals"].as<std::string>();
  if (nstr == "central-diff") normals = cvc::CENTRAL_DIFFERENCE;
  else if (nstr == "bspline-interp") normals = cvc::BSPLINE_INTERPOLATION;

  int improve = vm["improve"].as<int>();
  double isovalue = vm["isovalue"].as<double>();

  cvc::geometry result;
  if (vm.count("property-vol")) {
    cvc::volume prop(app);
    prop.read(vm["property-vol"].as<std::string>());
    result = cvc::iso(vol, isovalue, method, improve, normals, prop);
  } else {
    result = cvc::iso(vol, isovalue, method, improve, normals);
  }

  result.write(vm["output"].as<std::string>());
  std::cout << "Wrote isosurface (" << result.num_points() << " verts, "
            << result.num_tris() << " tris) to " << vm["output"].as<std::string>() << "\n";
  return 0;
}

static int cmd_tetrahedralize(int argc, char **argv) {
  po::options_description desc(
    "cvc tetrahedralize - extract tetrahedral mesh from volume\n\n"
    "Improvement methods:\n"
    "  none          No improvement [default]\n"
    "  geo-flow      Geometric flow smoothing\n"
    "  edge-contract Edge contraction\n"
    "  joe-liu       Joe-Liu method\n"
    "  minimal-vol   Minimal volume\n"
    "  optimization  Optimization-based\n");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output geometry file")
    ("isovalue,v", po::value<double>()->required(), "isovalue")
    ("method,m", po::value<std::string>()->default_value("duallib"),
     "extraction method (duallib, fastcontouring, libisocontour)")
    ("improve", po::value<std::string>()->default_value("none"),
     "improvement method")
    ("normals,n", po::value<std::string>()->default_value("bspline-conv"),
     "normal type (bspline-conv, central-diff, bspline-interp)")
    ("iterations,q", po::value<int>()->default_value(0),
     "improvement iterations")
    ("property-vol,p", po::value<std::string>(),
     "optional property volume");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());

  cvc::extraction_method method = cvc::DUALLIB;
  std::string mstr = vm["method"].as<std::string>();
  if (mstr == "fastcontouring") method = cvc::FASTCONTOURING;
  else if (mstr == "libisocontour") method = cvc::LIBISOCONTOUR;

  cvc::improvement_method improve = cvc::NO_IMPROVE;
  std::string istr = vm["improve"].as<std::string>();
  if (istr == "geo-flow") improve = cvc::GEO_FLOW;
  else if (istr == "edge-contract") improve = cvc::EDGE_CONTRACT;
  else if (istr == "joe-liu") improve = cvc::JOE_LIU;
  else if (istr == "minimal-vol") improve = cvc::MINIMAL_VOL;
  else if (istr == "optimization") improve = cvc::OPTIMIZATION;

  cvc::normal_type normals = cvc::BSPLINE_CONVOLUTION;
  std::string nstr = vm["normals"].as<std::string>();
  if (nstr == "central-diff") normals = cvc::CENTRAL_DIFFERENCE;
  else if (nstr == "bspline-interp") normals = cvc::BSPLINE_INTERPOLATION;

  int iters = vm["iterations"].as<int>();
  double isovalue = vm["isovalue"].as<double>();

  cvc::geometry result;
  if (vm.count("property-vol")) {
    cvc::volume prop(app);
    prop.read(vm["property-vol"].as<std::string>());
    result = cvc::tetrahedralize(vol, isovalue, method, improve, normals, iters, prop);
  } else {
    result = cvc::tetrahedralize(vol, isovalue, method, improve, normals, iters);
  }

  result.write(vm["output"].as<std::string>());
  std::cout << "Wrote tetrahedral mesh (" << result.num_points() << " verts) to "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}

static int cmd_hexahedralize(int argc, char **argv) {
  po::options_description desc("cvc hexahedralize - extract hexahedral mesh from volume");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output geometry file")
    ("isovalue,v", po::value<double>()->required(), "isovalue")
    ("method,m", po::value<std::string>()->default_value("duallib"),
     "extraction method (duallib, fastcontouring, libisocontour)")
    ("improve", po::value<std::string>()->default_value("none"),
     "improvement method (none, geo-flow, edge-contract, joe-liu, minimal-vol, optimization)")
    ("normals,n", po::value<std::string>()->default_value("bspline-conv"),
     "normal type (bspline-conv, central-diff, bspline-interp)")
    ("iterations,q", po::value<int>()->default_value(0), "improvement iterations")
    ("property-vol,p", po::value<std::string>(), "optional property volume");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());

  cvc::extraction_method method = cvc::DUALLIB;
  std::string mstr = vm["method"].as<std::string>();
  if (mstr == "fastcontouring") method = cvc::FASTCONTOURING;
  else if (mstr == "libisocontour") method = cvc::LIBISOCONTOUR;

  cvc::improvement_method improve = cvc::NO_IMPROVE;
  std::string istr = vm["improve"].as<std::string>();
  if (istr == "geo-flow") improve = cvc::GEO_FLOW;
  else if (istr == "edge-contract") improve = cvc::EDGE_CONTRACT;
  else if (istr == "joe-liu") improve = cvc::JOE_LIU;
  else if (istr == "minimal-vol") improve = cvc::MINIMAL_VOL;
  else if (istr == "optimization") improve = cvc::OPTIMIZATION;

  cvc::normal_type normals = cvc::BSPLINE_CONVOLUTION;
  std::string nstr = vm["normals"].as<std::string>();
  if (nstr == "central-diff") normals = cvc::CENTRAL_DIFFERENCE;
  else if (nstr == "bspline-interp") normals = cvc::BSPLINE_INTERPOLATION;

  int iters = vm["iterations"].as<int>();
  double isovalue = vm["isovalue"].as<double>();

  cvc::geometry result;
  if (vm.count("property-vol")) {
    cvc::volume prop(app);
    prop.read(vm["property-vol"].as<std::string>());
    result = cvc::hexahedralize(vol, isovalue, method, improve, normals, iters, prop);
  } else {
    result = cvc::hexahedralize(vol, isovalue, method, improve, normals, iters);
  }

  result.write(vm["output"].as<std::string>());
  std::cout << "Wrote hexahedral mesh (" << result.num_points() << " verts) to "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}
#endif // CVC_ENABLE_MESHER

// ---------------------------------------------------------------------------
// Volume arithmetic commands
// ---------------------------------------------------------------------------

static int cmd_add(int argc, char **argv) {
  po::options_description desc("cvc add - add two volumes element-wise");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
     "two input volume files")
    ("output,o", po::value<std::string>()->required(), "output volume file");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  if (inputs.size() != 2)
    throw std::runtime_error("add requires exactly 2 input files");

  auto &app = cvc_app();
  cvc::volume a(app), b(app);
  a.read(inputs[0]);
  b.read(inputs[1]);
  cvc::volume result = cvc::vol_add(a, b);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_subtract(int argc, char **argv) {
  po::options_description desc("cvc subtract - subtract second volume from first");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
     "two input volume files")
    ("output,o", po::value<std::string>()->required(), "output volume file");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  if (inputs.size() != 2)
    throw std::runtime_error("subtract requires exactly 2 input files");

  auto &app = cvc_app();
  cvc::volume a(app), b(app);
  a.read(inputs[0]);
  b.read(inputs[1]);
  cvc::volume result = cvc::vol_subtract(a, b);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_scale(int argc, char **argv) {
  po::options_description desc("cvc scale - multiply volume by scalar");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output volume file")
    ("factor,f", po::value<double>()->required(), "scale factor");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_scale(vol, vm["factor"].as<double>());
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_normalize(int argc, char **argv) {
  po::options_description desc("cvc normalize - remap voxel values to [min, max]");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output volume file")
    ("min", po::value<double>()->default_value(0.0), "new minimum")
    ("max", po::value<double>()->default_value(1.0), "new maximum");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_normalize(vol, vm["min"].as<double>(), vm["max"].as<double>());
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_clip(int argc, char **argv) {
  po::options_description desc("cvc clip - zero voxels above threshold");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output volume file")
    ("threshold,t", po::value<double>()->required(), "clipping threshold");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_clip(vol, vm["threshold"].as<double>());
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_negate(int argc, char **argv) {
  po::options_description desc("cvc negate - negate all voxel values");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output volume file");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_negate(vol);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_mask(int argc, char **argv) {
  po::options_description desc("cvc mask - zero voxels where mask is nonzero");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("mask,m", po::value<std::string>()->required(), "mask volume file")
    ("output,o", po::value<std::string>()->required(), "output volume file")
    ("inverse", "use inverse mask (zero where mask IS zero)");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app), mask_vol(app);
  vol.read(vm["input"].as<std::string>());
  mask_vol.read(vm["mask"].as<std::string>());

  cvc::volume result = vm.count("inverse")
    ? cvc::vol_inverse_mask(vol, mask_vol)
    : cvc::vol_mask(vol, mask_vol);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_downsample(int argc, char **argv) {
  po::options_description desc("cvc downsample - reduce volume resolution");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output volume file")
    ("factor,f", po::value<unsigned int>()->default_value(2), "downsample factor");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  unsigned int f = vm["factor"].as<unsigned int>();
  cvc::volume result = cvc::vol_downsample(vol, f, f, f);
  result.write(vm["output"].as<std::string>());
  return 0;
}

// ---------------------------------------------------------------------------
// Volume transform commands
// ---------------------------------------------------------------------------

static int cmd_rotate(int argc, char **argv) {
  po::options_description desc("cvc rotate - rotate volume around Z-axis");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output volume file")
    ("angle,a", po::value<double>()->required(), "rotation angle in degrees")
    ("count,n", po::value<int>()->default_value(1),
     "number of evenly-spaced rotations");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  std::string output = vm["output"].as<std::string>();
  int count = vm["count"].as<int>();

  if (count <= 1) {
    double angle_rad = vm["angle"].as<double>() * M_PI / 180.0;
    cvc::volume result = cvc::vol_rotate_z(vol, angle_rad);
    result.write(output);
    std::cout << "Rotated " << vm["angle"].as<double>() << " degrees -> " << output << "\n";
  } else {
    for (int n = 0; n < count; n++) {
      double angle_rad = 2.0 * M_PI * n / count;
      cvc::volume result = cvc::vol_rotate_z(vol, angle_rad);
      std::ostringstream ss;
      ss << output << "." << std::setw(4) << std::setfill('0') << n << ".rawiv";
      result.write(ss.str());
    }
    std::cout << "Wrote " << count << " rotations\n";
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Analysis commands
// ---------------------------------------------------------------------------

static int cmd_ssim(int argc, char **argv) {
  po::options_description desc("cvc ssim - compute Structural Similarity Index (SSIM)");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
     "two input volume files")
    ("output,o", po::value<std::string>(), "output SSIM map volume file")
    ("window,w", po::value<int>()->default_value(11), "Gaussian window size (odd)")
    ("sigma,s", po::value<double>()->default_value(1.5), "Gaussian sigma");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  if (inputs.size() != 2)
    throw std::runtime_error("ssim requires exactly 2 input files");

  auto &app = cvc_app();
  cvc::volume a(app), b(app);
  a.read(inputs[0]);
  b.read(inputs[1]);

  cvc::ssim_result result = cvc::vol_ssim(a, b, vm["window"].as<int>(),
                                           vm["sigma"].as<double>());
  std::cout << std::setprecision(12) << "Mean SSIM: " << result.mean_ssim << "\n";

  if (vm.count("output")) {
    result.ssim_map.write(vm["output"].as<std::string>());
    std::cout << "Wrote SSIM map to " << vm["output"].as<std::string>() << "\n";
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Projection / reconstruction commands
// ---------------------------------------------------------------------------

static int cmd_project(int argc, char **argv) {
  po::options_description desc("cvc project - forward ray projection of volume");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("output,o", po::value<std::string>()->required(), "output projection volume")
    ("angles,a", po::value<std::string>()->required(),
     "file with angles in degrees (one per line)")
    ("step", po::value<double>()->default_value(0.5), "ray step size");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  std::vector<double> angles;
  {
    std::ifstream f(vm["angles"].as<std::string>());
    if (!f)
      throw std::runtime_error("Cannot open angles file");
    double deg;
    while (f >> deg)
      angles.push_back(deg * M_PI / 180.0);
  }

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_project(vol, angles, vm["step"].as<double>());
  result.write(vm["output"].as<std::string>());
  std::cout << "Projected " << angles.size() << " angles -> "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}

static int cmd_backproject(int argc, char **argv) {
  po::options_description desc(
    "cvc backproject - filtered back-projection (tomographic reconstruction)");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input projection volume")
    ("output,o", po::value<std::string>()->required(), "output reconstructed volume")
    ("angles,a", po::value<std::string>()->required(),
     "file with angles in degrees (one per line)")
    ("dim,d", po::value<unsigned int>()->required(), "output cube dimension")
    ("no-filter", "disable FFT ramp filter");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  std::vector<double> angles;
  {
    std::ifstream f(vm["angles"].as<std::string>());
    if (!f)
      throw std::runtime_error("Cannot open angles file");
    double deg;
    while (f >> deg)
      angles.push_back(deg * M_PI / 180.0);
  }

  cvc::volume proj(cvc_app());
  proj.read(vm["input"].as<std::string>());
  bool filter = !vm.count("no-filter");
  cvc::volume result = cvc::vol_back_project(proj, angles,
                                              vm["dim"].as<unsigned int>(), filter);
  result.write(vm["output"].as<std::string>());
  std::cout << "Reconstructed " << vm["dim"].as<unsigned int>() << "^3 volume -> "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}

// ---------------------------------------------------------------------------
// Image / slice commands
// ---------------------------------------------------------------------------

static int cmd_vol2img(int argc, char **argv) {
  po::options_description desc("cvc vol2img - export volume slices as images");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::string>()->required(), "input volume file")
    ("dir,d", po::value<std::string>()->required(), "output directory")
    ("format,f", po::value<std::string>()->default_value("slice_%05d.png"),
     "filename pattern (printf-style)");

  po::positional_options_description pos;
  pos.add("input", 1).add("dir", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());

  std::string dir = vm["dir"].as<std::string>();
  fs::create_directories(dir);

  cvc::vol_to_slices(vol, dir, vm["format"].as<std::string>());
  std::cout << "Exported " << vol.ZDim() << " slices to " << dir << "/\n";
  return 0;
}

static int cmd_img2vol(int argc, char **argv) {
  po::options_description desc("cvc img2vol - import image stack into volume");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
     "input image files (in Z order)")
    ("output,o", po::value<std::string>()->required(), "output volume file");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  cvc::volume result = cvc::slices_to_volume(cvc_app(), inputs);
  result.write(vm["output"].as<std::string>());
  std::cout << "Imported " << inputs.size() << " images -> "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}

static int cmd_rgba_merge(int argc, char **argv) {
  po::options_description desc("cvc rgba-merge - merge 4 volumes into RGBA");
  desc.add_options()
    ("help,h", "show help")
    ("input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
     "4 input volume files (R G B A)")
    ("output,o", po::value<std::string>()->required(), "output volume file");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  if (inputs.size() != 4)
    throw std::runtime_error("rgba-merge requires exactly 4 input files (R G B A)");

  auto &app = cvc_app();
  cvc::volume r(app), g(app), b(app), a(app);
  r.read(inputs[0]);
  g.read(inputs[1]);
  b.read(inputs[2]);
  a.read(inputs[3]);
  cvc::volume result = cvc::vol_rgba_merge(r, g, b, a);
  result.write(vm["output"].as<std::string>());
  return 0;
}

// ---------------------------------------------------------------------------
// Built-in test data
// ---------------------------------------------------------------------------

static int cmd_bunny(int argc, char **argv) {
  po::options_description desc("cvc bunny - output Stanford bunny geometry or SDF volume");
  desc.add_options()
    ("help,h", "show help")
    ("output,o", po::value<std::string>()->required(),
     "output file (.off for geometry, .rawiv/.mrc for volume)")
    ("volume", "output SDF volume instead of geometry")
    ("dims,d", po::value<unsigned int>()->default_value(64), "volume dimensions (cube)")
    ("padding,p", po::value<double>()->default_value(0.1), "bounding box padding factor")
    ("algorithm,a", po::value<std::string>()->default_value("v1"),
     "SDF algorithm: v1 (octree) or v2 (distance transform)");

  po::positional_options_description pos;
  pos.add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  std::string output = vm["output"].as<std::string>();

  cvc::geometry bunny;
  bunny.read("builtin.bunny");

  if (vm.count("volume")) {
    unsigned int d = vm["dims"].as<unsigned int>();
    double pad = vm["padding"].as<double>();

    auto pmin = bunny.min_point();
    auto pmax = bunny.max_point();
    double ext[3];
    for (int i = 0; i < 3; ++i)
      ext[i] = pmax[i] - pmin[i];
    double max_ext = std::max({ext[0], ext[1], ext[2]});
    double half = max_ext * (1.0 + pad) * 0.5;
    double cx = (pmin[0] + pmax[0]) * 0.5;
    double cy = (pmin[1] + pmax[1]) * 0.5;
    double cz = (pmin[2] + pmax[2]) * 0.5;
    cvc::bounding_box bbox(cx - half, cy - half, cz - half,
                           cx + half, cy + half, cz + half);

#ifdef CVC_ENABLE_SDF
    cvc::sdf_algorithm algo = cvc::SDF_V1;
    std::string algo_str = vm["algorithm"].as<std::string>();
    if (algo_str == "v2" || algo_str == "V2")
      algo = cvc::SDF_V2;
    cvc::volume sdf_vol = cvc::sdf(cvc_app(), bunny, cvc::dimension(d, d, d), bbox, algo);
    sdf_vol.write(output);
#else
    throw std::runtime_error("SDF support not enabled (CVC_ENABLE_SDF=OFF)");
#endif
    std::cout << "Wrote bunny SDF volume " << d << "^3 to " << output << "\n";
  } else {
    bunny.write(output);
    std::cout << "Wrote bunny geometry (" << bunny.num_points() << " verts, "
              << bunny.num_tris() << " tris) to " << output << "\n";
  }
  return 0;
}

// ---------------------------------------------------------------------------
// XMLRPC commands (conditional)
// ---------------------------------------------------------------------------

#ifdef USING_XMLRPC
#include <cvc/core/state.h>

static int cmd_server(int argc, char **argv) {
  po::options_description desc("cvc server - start XMLRPC server");
  desc.add_options()
    ("help,h", "show help")
    ("port,p", po::value<int>()->default_value(cvc::XMLRPC_DEFAULT_PORT),
     "server port");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto &app = cvc_app();
  int port = vm["port"].as<int>();
  cvc::state::instance(app)("__system.xmlrpc.port").value(port);
  cvc::state::instance(app)("__system.xmlrpc").value(int(1));
  app.wait();
  return 0;
}

static int cmd_client(int argc, char **argv) {
  po::options_description desc("cvc client - call XMLRPC method on remote server");
  desc.add_options()
    ("help,h", "show help")
    ("host", po::value<std::string>()->required(), "host:port")
    ("method", po::value<std::string>()->required(), "RPC method name")
    ("args", po::value<std::vector<std::string>>()->multitoken(), "method arguments");

  po::positional_options_description pos;
  pos.add("host", 1).add("method", 1).add("args", -1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) { std::cout << desc << "\n"; return 0; }
  po::notify(vm);

  auto &app = cvc_app();
  std::vector<std::string> rpc_args;
  if (vm.count("args"))
    rpc_args = vm["args"].as<std::vector<std::string>>();
  std::string result = cvc::rpc(app, vm["host"].as<std::string>(),
                                vm["method"].as<std::string>(), rpc_args);
  if (!result.empty())
    std::cout << result << "\n";
  return 0;
}
#endif // USING_XMLRPC

// ===========================================================================
// Command dispatch table
// ===========================================================================

struct command_entry {
  const char *name;
  const char *category;
  const char *help;
  int (*func)(int, char **);
};

// clang-format off
static const command_entry commands[] = {
  // ── File info ──
  {"info",           "File Info",       "display file metadata (volume or geometry)",     cmd_info},
  {"stats",          "File Info",       "compute volume statistics",                      cmd_stats},

  // ── File conversion ──
  {"copy",           "Conversion",      "copy/convert files (auto-detects type)",         cmd_copy},
  {"convert",        "Conversion",      "convert volume format or voxel type",            cmd_convert},

  // ── Geometry processing ──
#ifdef CVC_ENABLE_SDF
  {"sdf",            "Geometry",        "compute signed distance field from geometry",    cmd_sdf},
#endif
#ifdef CVC_ENABLE_MESHER
  {"iso",            "Geometry",        "extract isosurface from volume",                 cmd_iso},
  {"tetrahedralize", "Geometry",        "extract tetrahedral mesh from volume",           cmd_tetrahedralize},
  {"hexahedralize",  "Geometry",        "extract hexahedral mesh from volume",            cmd_hexahedralize},
#endif

  // ── Volume arithmetic ──
  {"add",            "Vol Arithmetic",  "add two volumes element-wise",                   cmd_add},
  {"subtract",       "Vol Arithmetic",  "subtract two volumes element-wise",              cmd_subtract},
  {"scale",          "Vol Arithmetic",  "multiply volume by scalar",                      cmd_scale},
  {"normalize",      "Vol Arithmetic",  "remap voxel values to [min, max]",               cmd_normalize},
  {"clip",           "Vol Arithmetic",  "zero voxels above threshold",                    cmd_clip},
  {"negate",         "Vol Arithmetic",  "negate all voxel values",                        cmd_negate},
  {"mask",           "Vol Arithmetic",  "apply mask volume",                              cmd_mask},
  {"downsample",     "Vol Arithmetic",  "reduce volume resolution",                      cmd_downsample},

  // ── Volume transforms ──
  {"rotate",         "Vol Transform",   "rotate volume around Z-axis",                   cmd_rotate},

  // ── Analysis ──
  {"ssim",           "Analysis",        "compute SSIM between two volumes",              cmd_ssim},

  // ── Projection / Reconstruction ──
  {"project",        "Projection",      "forward ray projection",                        cmd_project},
  {"backproject",    "Projection",      "filtered back-projection (FBP)",                cmd_backproject},

  // ── Image I/O ──
  {"vol2img",        "Image I/O",       "export volume slices as images",                cmd_vol2img},
  {"img2vol",        "Image I/O",       "import image stack as volume",                  cmd_img2vol},
  {"rgba-merge",     "Image I/O",       "merge 4 volumes into RGBA",                     cmd_rgba_merge},

  // ── Test data ──
  {"bunny",          "Test Data",       "output Stanford bunny geometry or SDF",         cmd_bunny},

  // ── Network ──
#ifdef USING_XMLRPC
  {"server",         "Network",         "start XMLRPC server",                           cmd_server},
  {"client",         "Network",         "call XMLRPC method on remote server",           cmd_client},
#endif
};
// clang-format on

static const int num_commands = sizeof(commands) / sizeof(commands[0]);

static void print_usage() {
  std::cout << "cvc - Unified CLI for the CVC computational geometry library\n\n"
            << "Usage: cvc <command> [options]\n\n";

  // Print commands grouped by category
  std::string current_category;
  for (int i = 0; i < num_commands; ++i) {
    if (current_category != commands[i].category) {
      current_category = commands[i].category;
      std::cout << "  " << current_category << ":\n";
    }
    std::cout << "    " << std::left << std::setw(18) << commands[i].name
              << commands[i].help << "\n";
  }
  std::cout << "\nRun 'cvc <command> --help' for command-specific options.\n";
}

int main(int argc, char **argv) {
  if (argc < 2) {
    print_usage();
    return 1;
  }

  std::string cmd = argv[1];
  if (cmd == "--help" || cmd == "-h") {
    print_usage();
    return 0;
  }
  if (cmd == "--version" || cmd == "-V") {
    std::cout << "cvc (libcvc) built " << __DATE__ << "\n";
    return 0;
  }

  for (int i = 0; i < num_commands; ++i) {
    if (cmd == commands[i].name) {
      try {
        return commands[i].func(argc - 1, argv + 1);
      } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
      }
    }
  }

  std::cerr << "Unknown command: " << cmd << "\n\n";
  print_usage();
  return 1;
}
