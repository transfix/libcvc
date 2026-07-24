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

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/distributed_state_session.h>
#include <cvc/core/state.h>
#include <cvc/core/state_distributed_admin.h>
#include <cvc/core/state_exec/exec_coordinator.h>
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/types.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/utility/algorithm.h>
#include <cvc/utility/utility.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_ops.h>
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
      {"UChar", cvc::UChar},           {"unsigned char", cvc::UChar}, {"UShort", cvc::UShort},
      {"unsigned short", cvc::UShort}, {"UInt", cvc::UInt},           {"unsigned int", cvc::UInt},
      {"Float", cvc::Float},           {"float", cvc::Float},         {"Double", cvc::Double},
      {"double", cvc::Double},
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
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input file");

  po::positional_options_description pos;
  pos.add("input", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
              << "BBox:       [" << vfi.boundingBox().minx << ", " << vfi.boundingBox().miny << ", "
              << vfi.boundingBox().minz << "] - [" << vfi.boundingBox().maxx << ", "
              << vfi.boundingBox().maxy << ", " << vfi.boundingBox().maxz << "]\n"
              << "Span:       " << vfi.XSpan() << " x " << vfi.YSpan() << " x " << vfi.ZSpan()
              << "\n"
              << "Variables:  " << vfi.numVariables() << "\n"
              << "Timesteps:  " << vfi.numTimesteps() << "\n";

    for (unsigned v = 0; v < vfi.numVariables(); ++v) {
      std::cout << "  Var " << v << ": name=" << vfi.name(v) << " type=" << vfi.voxelTypeStr(v);
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
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "region,r", po::value<std::string>(),
      "restrict stats to an object-space region \"minx,miny,minz,maxx,maxy,maxz\"");

  po::positional_options_description pos;
  pos.add("input", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume_stats s =
      vm.count("region")
          ? cvc::compute_stats(vol, cvc::bounding_box(vm["region"].as<std::string>()))
          : cvc::compute_stats(vol);

  std::cout << std::setprecision(12) << "Min:     " << s.min << "\n"
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
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input file")(
      "output,o", po::value<std::string>()->required(), "output file");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::save(app, cvc::load(app, vm["input"].as<std::string>()), vm["output"].as<std::string>());
  return 0;
}

static int cmd_convert(int argc, char **argv) {
  po::options_description desc("cvc convert - convert between volume formats or voxel types");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output volume file")(
      "type,t", po::value<std::string>(), "output voxel type (UChar, UShort, UInt, Float, Double)");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  po::options_description desc("cvc sdf - compute signed distance field from geometry\n\n"
                               "Algorithms:\n"
                               "  v1  Original SDFLibrary (octree-based, thread-safe) [default]\n"
                               "  v2  DistanceTransform (brute-force)\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input geometry file")(
      "output,o", po::value<std::string>()->required(),
      "output volume file")("dim,d", po::value<std::string>()->default_value("64,64,64"),
                            "output dimensions (NxNxN or X,Y,Z)")(
      "bbox,b", po::value<std::string>(),
      "bounding box (minx,miny,minz,maxx,maxy,maxz); defaults to geometry extents")(
      "algorithm,a", po::value<std::string>()->default_value("v1"),
      "SDF algorithm: v1 or v2")("flip-normals", "flip normals to invert inside/outside");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  po::options_description desc("cvc iso - extract isosurface geometry from a volume\n\n"
                               "Extraction methods:\n"
                               "  duallib         Dual contouring library [default]\n"
                               "  fastcontouring  Fast contouring\n"
                               "  libisocontour   ISO contouring library\n\n"
                               "Normal types:\n"
                               "  bspline-conv    B-spline convolution [default]\n"
                               "  central-diff    Central difference\n"
                               "  bspline-interp  B-spline interpolation\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output geometry file")("isovalue,v", po::value<double>()->required(), "isovalue")(
      "method,m", po::value<std::string>()->default_value("duallib"), "extraction method")(
      "improve,q", po::value<int>()->default_value(0), "quality improvement iterations (0 = none)")(
      "normals,n", po::value<std::string>()->default_value("bspline-conv"),
      "normal computation method")("property-vol,p", po::value<std::string>(),
                                   "optional property volume for interpolation");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());

  // Parse extraction method
  cvc::extraction_method method = cvc::DUALLIB;
  std::string mstr = vm["method"].as<std::string>();
  if (mstr == "fastcontouring")
    method = cvc::FASTCONTOURING;
  else if (mstr == "libisocontour")
    method = cvc::LIBISOCONTOUR;

  // Parse normal type
  cvc::normal_type normals = cvc::BSPLINE_CONVOLUTION;
  std::string nstr = vm["normals"].as<std::string>();
  if (nstr == "central-diff")
    normals = cvc::CENTRAL_DIFFERENCE;
  else if (nstr == "bspline-interp")
    normals = cvc::BSPLINE_INTERPOLATION;

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
  std::cout << "Wrote isosurface (" << result.num_points() << " verts, " << result.num_tris()
            << " tris) to " << vm["output"].as<std::string>() << "\n";
  return 0;
}

static int cmd_tetrahedralize(int argc, char **argv) {
  po::options_description desc("cvc tetrahedralize - extract tetrahedral mesh from volume\n\n"
                               "Improvement methods:\n"
                               "  none          No improvement [default]\n"
                               "  geo-flow      Geometric flow smoothing\n"
                               "  edge-contract Edge contraction\n"
                               "  joe-liu       Joe-Liu method\n"
                               "  minimal-vol   Minimal volume\n"
                               "  optimization  Optimization-based\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output geometry file")("isovalue,v", po::value<double>()->required(), "isovalue")(
      "method,m", po::value<std::string>()->default_value("duallib"),
      "extraction method (duallib, fastcontouring, libisocontour)")(
      "improve", po::value<std::string>()->default_value("none"),
      "improvement method")("normals,n", po::value<std::string>()->default_value("bspline-conv"),
                            "normal type (bspline-conv, central-diff, bspline-interp)")(
      "iterations,q", po::value<int>()->default_value(0), "improvement iterations")(
      "property-vol,p", po::value<std::string>(), "optional property volume");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());

  cvc::extraction_method method = cvc::DUALLIB;
  std::string mstr = vm["method"].as<std::string>();
  if (mstr == "fastcontouring")
    method = cvc::FASTCONTOURING;
  else if (mstr == "libisocontour")
    method = cvc::LIBISOCONTOUR;

  cvc::improvement_method improve = cvc::NO_IMPROVE;
  std::string istr = vm["improve"].as<std::string>();
  if (istr == "geo-flow")
    improve = cvc::GEO_FLOW;
  else if (istr == "edge-contract")
    improve = cvc::EDGE_CONTRACT;
  else if (istr == "joe-liu")
    improve = cvc::JOE_LIU;
  else if (istr == "minimal-vol")
    improve = cvc::MINIMAL_VOL;
  else if (istr == "optimization")
    improve = cvc::OPTIMIZATION;

  cvc::normal_type normals = cvc::BSPLINE_CONVOLUTION;
  std::string nstr = vm["normals"].as<std::string>();
  if (nstr == "central-diff")
    normals = cvc::CENTRAL_DIFFERENCE;
  else if (nstr == "bspline-interp")
    normals = cvc::BSPLINE_INTERPOLATION;

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
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output geometry file")("isovalue,v", po::value<double>()->required(), "isovalue")(
      "method,m", po::value<std::string>()->default_value("duallib"),
      "extraction method (duallib, fastcontouring, libisocontour)")(
      "improve", po::value<std::string>()->default_value("none"),
      "improvement method (none, geo-flow, edge-contract, joe-liu, minimal-vol, optimization)")(
      "normals,n", po::value<std::string>()->default_value("bspline-conv"),
      "normal type (bspline-conv, central-diff, bspline-interp)")(
      "iterations,q", po::value<int>()->default_value(0), "improvement iterations")(
      "property-vol,p", po::value<std::string>(), "optional property volume");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());

  cvc::extraction_method method = cvc::DUALLIB;
  std::string mstr = vm["method"].as<std::string>();
  if (mstr == "fastcontouring")
    method = cvc::FASTCONTOURING;
  else if (mstr == "libisocontour")
    method = cvc::LIBISOCONTOUR;

  cvc::improvement_method improve = cvc::NO_IMPROVE;
  std::string istr = vm["improve"].as<std::string>();
  if (istr == "geo-flow")
    improve = cvc::GEO_FLOW;
  else if (istr == "edge-contract")
    improve = cvc::EDGE_CONTRACT;
  else if (istr == "joe-liu")
    improve = cvc::JOE_LIU;
  else if (istr == "minimal-vol")
    improve = cvc::MINIMAL_VOL;
  else if (istr == "optimization")
    improve = cvc::OPTIMIZATION;

  cvc::normal_type normals = cvc::BSPLINE_CONVOLUTION;
  std::string nstr = vm["normals"].as<std::string>();
  if (nstr == "central-diff")
    normals = cvc::CENTRAL_DIFFERENCE;
  else if (nstr == "bspline-interp")
    normals = cvc::BSPLINE_INTERPOLATION;

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

static int cmd_tetrahedralize2(int argc, char **argv) {
  po::options_description desc(
      "cvc tetrahedralize2 - extract dual tetrahedral (tet2) mesh from volume\n\n"
      "Produces a dual tetrahedral mesh (TETRA2 element type).\n\n"
      "Improvement methods:\n"
      "  none          No improvement [default]\n"
      "  geo-flow      Geometric flow smoothing\n"
      "  edge-contract Edge contraction\n"
      "  joe-liu       Joe-Liu method\n"
      "  minimal-vol   Minimal volume\n"
      "  optimization  Optimization-based\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output geometry file")("isovalue,v", po::value<double>()->required(), "isovalue")(
      "method,m", po::value<std::string>()->default_value("duallib"),
      "extraction method (duallib, fastcontouring, libisocontour)")(
      "improve", po::value<std::string>()->default_value("none"),
      "improvement method")("normals,n", po::value<std::string>()->default_value("bspline-conv"),
                            "normal type (bspline-conv, central-diff, bspline-interp)")(
      "iterations,q", po::value<int>()->default_value(0), "improvement iterations");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());

  cvc::extraction_method method = cvc::DUALLIB;
  std::string mstr = vm["method"].as<std::string>();
  if (mstr == "fastcontouring")
    method = cvc::FASTCONTOURING;
  else if (mstr == "libisocontour")
    method = cvc::LIBISOCONTOUR;

  cvc::improvement_method improve = cvc::NO_IMPROVE;
  std::string istr = vm["improve"].as<std::string>();
  if (istr == "geo-flow")
    improve = cvc::GEO_FLOW;
  else if (istr == "edge-contract")
    improve = cvc::EDGE_CONTRACT;
  else if (istr == "joe-liu")
    improve = cvc::JOE_LIU;
  else if (istr == "minimal-vol")
    improve = cvc::MINIMAL_VOL;
  else if (istr == "optimization")
    improve = cvc::OPTIMIZATION;

  cvc::normal_type normals = cvc::BSPLINE_CONVOLUTION;
  std::string nstr = vm["normals"].as<std::string>();
  if (nstr == "central-diff")
    normals = cvc::CENTRAL_DIFFERENCE;
  else if (nstr == "bspline-interp")
    normals = cvc::BSPLINE_INTERPOLATION;

  int iters = vm["iterations"].as<int>();
  double isovalue = vm["isovalue"].as<double>();

  cvc::geometry result = cvc::tetrahedralize2(vol, isovalue, method, improve, normals, iters);

  result.write(vm["output"].as<std::string>());
  std::cout << "Wrote dual-tet mesh (" << result.num_points() << " verts) to "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}

static int cmd_layer_mesh(int argc, char **argv) {
  po::options_description desc(
      "cvc layer-mesh - extract tetrahedral mesh of layer between two isosurfaces\n\n"
      "Produces a volumetric tet2 mesh of the region between an outer and inner\n"
      "isovalue. Useful for meshing shells, cortical layers, or material boundaries.\n\n"
      "Improvement methods:\n"
      "  none          No improvement [default]\n"
      "  geo-flow      Geometric flow smoothing\n"
      "  edge-contract Edge contraction\n"
      "  joe-liu       Joe-Liu method\n"
      "  minimal-vol   Minimal volume\n"
      "  optimization  Optimization-based\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output geometry file")("isovalue-outer", po::value<double>()->required(), "outer isovalue")(
      "isovalue-inner", po::value<double>()->required(),
      "inner isovalue")("method,m", po::value<std::string>()->default_value("duallib"),
                        "extraction method (duallib, fastcontouring, libisocontour)")(
      "improve", po::value<std::string>()->default_value("none"),
      "improvement method")("normals,n", po::value<std::string>()->default_value("bspline-conv"),
                            "normal type (bspline-conv, central-diff, bspline-interp)")(
      "iterations,q", po::value<int>()->default_value(0), "improvement iterations");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());

  cvc::extraction_method method = cvc::DUALLIB;
  std::string mstr = vm["method"].as<std::string>();
  if (mstr == "fastcontouring")
    method = cvc::FASTCONTOURING;
  else if (mstr == "libisocontour")
    method = cvc::LIBISOCONTOUR;

  cvc::improvement_method improve = cvc::NO_IMPROVE;
  std::string istr = vm["improve"].as<std::string>();
  if (istr == "geo-flow")
    improve = cvc::GEO_FLOW;
  else if (istr == "edge-contract")
    improve = cvc::EDGE_CONTRACT;
  else if (istr == "joe-liu")
    improve = cvc::JOE_LIU;
  else if (istr == "minimal-vol")
    improve = cvc::MINIMAL_VOL;
  else if (istr == "optimization")
    improve = cvc::OPTIMIZATION;

  cvc::normal_type normals = cvc::BSPLINE_CONVOLUTION;
  std::string nstr = vm["normals"].as<std::string>();
  if (nstr == "central-diff")
    normals = cvc::CENTRAL_DIFFERENCE;
  else if (nstr == "bspline-interp")
    normals = cvc::BSPLINE_INTERPOLATION;

  int iters = vm["iterations"].as<int>();
  double iso_outer = vm["isovalue-outer"].as<double>();
  double iso_inner = vm["isovalue-inner"].as<double>();

  cvc::geometry result =
      cvc::tetrahedralize2(vol, iso_outer, iso_inner, method, improve, normals, iters);

  result.write(vm["output"].as<std::string>());
  std::cout << "Wrote layer mesh (" << result.num_points() << " verts) to "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}
#endif // CVC_ENABLE_MESHER

// ---------------------------------------------------------------------------
// Volume arithmetic commands
// ---------------------------------------------------------------------------

static int cmd_add(int argc, char **argv) {
  po::options_description desc("cvc add - add two volumes element-wise");
  desc.add_options()("help,h", "show help")(
      "input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
      "two input volume files")("output,o", po::value<std::string>()->required(),
                                "output volume file");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  desc.add_options()("help,h", "show help")(
      "input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
      "two input volume files")("output,o", po::value<std::string>()->required(),
                                "output volume file");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output volume file")("factor,f", po::value<double>()->required(), "scale factor");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_scale(vol, vm["factor"].as<double>());
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_normalize(int argc, char **argv) {
  po::options_description desc("cvc normalize - remap voxel values to [min, max]");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output volume file")("min", po::value<double>()->default_value(0.0), "new minimum")(
      "max", po::value<double>()->default_value(1.0), "new maximum");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_normalize(vol, vm["min"].as<double>(), vm["max"].as<double>());
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_clip(int argc, char **argv) {
  po::options_description desc("cvc clip - zero voxels above threshold");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output volume file")("threshold,t", po::value<double>()->required(), "clipping threshold");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_clip(vol, vm["threshold"].as<double>());
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_negate(int argc, char **argv) {
  po::options_description desc("cvc negate - negate all voxel values");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output volume file");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_negate(vol);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_mask(int argc, char **argv) {
  po::options_description desc("cvc mask - zero voxels where mask is nonzero");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "mask,m", po::value<std::string>()->required(),
      "mask volume file")("output,o", po::value<std::string>()->required(), "output volume file")(
      "inverse", "use inverse mask (zero where mask IS zero)");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::volume vol(app), mask_vol(app);
  vol.read(vm["input"].as<std::string>());
  mask_vol.read(vm["mask"].as<std::string>());

  cvc::volume result =
      vm.count("inverse") ? cvc::vol_inverse_mask(vol, mask_vol) : cvc::vol_mask(vol, mask_vol);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_downsample(int argc, char **argv) {
  po::options_description desc("cvc downsample - reduce volume resolution");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output volume file")(
      "factor,f", po::value<unsigned int>()->default_value(2), "downsample factor");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output volume file")(
      "angle,a", po::value<double>()->required(), "rotation angle in degrees")(
      "count,n", po::value<int>()->default_value(1), "number of evenly-spaced rotations");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  desc.add_options()("help,h", "show help")(
      "input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
      "two input volume files")("output,o", po::value<std::string>(),
                                "output SSIM map volume file")(
      "window,w", po::value<int>()->default_value(11), "Gaussian window size (odd)")(
      "sigma,s", po::value<double>()->default_value(1.5), "Gaussian sigma");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  if (inputs.size() != 2)
    throw std::runtime_error("ssim requires exactly 2 input files");

  auto &app = cvc_app();
  cvc::volume a(app), b(app);
  a.read(inputs[0]);
  b.read(inputs[1]);

  cvc::ssim_result result = cvc::vol_ssim(a, b, vm["window"].as<int>(), vm["sigma"].as<double>());
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
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output projection volume")(
      "angles,a", po::value<std::string>()->required(),
      "file with angles in degrees (one per line)")("step", po::value<double>()->default_value(0.5),
                                                    "ray step size");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  std::cout << "Projected " << angles.size() << " angles -> " << vm["output"].as<std::string>()
            << "\n";
  return 0;
}

static int cmd_backproject(int argc, char **argv) {
  po::options_description desc(
      "cvc backproject - filtered back-projection (tomographic reconstruction)");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input projection volume")(
      "output,o", po::value<std::string>()->required(),
      "output reconstructed volume")("angles,a", po::value<std::string>()->required(),
                                     "file with angles in degrees (one per line)")(
      "dim,d", po::value<unsigned int>()->required(),
      "output cube dimension")("no-filter", "disable FFT ramp filter");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  cvc::volume result = cvc::vol_back_project(proj, angles, vm["dim"].as<unsigned int>(), filter);
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
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "dir,d", po::value<std::string>()->required(),
      "output directory")("format,f", po::value<std::string>()->default_value("slice_%05d.png"),
                          "filename pattern (printf-style)");

  po::positional_options_description pos;
  pos.add("input", 1).add("dir", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  desc.add_options()("help,h", "show help")(
      "input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
      "input image files (in Z order)")("output,o", po::value<std::string>()->required(),
                                        "output volume file");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  cvc::volume result = cvc::slices_to_volume(cvc_app(), inputs);
  result.write(vm["output"].as<std::string>());
  std::cout << "Imported " << inputs.size() << " images -> " << vm["output"].as<std::string>()
            << "\n";
  return 0;
}

static int cmd_rgba_merge(int argc, char **argv) {
  po::options_description desc("cvc rgba-merge - merge 4 volumes into RGBA");
  desc.add_options()("help,h", "show help")(
      "input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
      "4 input volume files (R G B A)")("output,o", po::value<std::string>()->required(),
                                        "output volume file");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  desc.add_options()("help,h",
                     "show help")("output,o", po::value<std::string>()->required(),
                                  "output file (.off for geometry, .rawiv/.mrc for volume)")(
      "volume", "output SDF volume instead of geometry")(
      "dims,d", po::value<unsigned int>()->default_value(64), "volume dimensions (cube)")(
      "padding,p", po::value<double>()->default_value(0.1),
      "bounding box padding factor")("algorithm,a", po::value<std::string>()->default_value("v1"),
                                     "SDF algorithm: v1 (octree) or v2 (distance transform)");

  po::positional_options_description pos;
  pos.add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
    cvc::bounding_box bbox(cx - half, cy - half, cz - half, cx + half, cy + half, cz + half);

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
    std::cout << "Wrote bunny geometry (" << bunny.num_points() << " verts, " << bunny.num_tris()
              << " tris) to " << output << "\n";
  }
  return 0;
}

// ---------------------------------------------------------------------------
// State server (distributed_state_session)
// ---------------------------------------------------------------------------

static std::atomic<bool> g_serve_running{true};

static void serve_signal_handler(int) { g_serve_running.store(false); }

static int cmd_serve(int argc, char **argv) {
  po::options_description desc(
      "cvc serve - run a headless CVC state server\n\n"
      "Starts a distributed state server that volrover3 instances (or other\n"
      "cvc clients) can connect to. Supports standalone, peer clustering,\n"
      "TLS, bearer-token auth, and subtree delegation.\n\n"
      "Transport modes:\n"
      "  ipc    Unix domain socket (same host)\n"
      "  grpc   gRPC over TCP (networked, requires CVC_ENABLE_GRPC)\n");
  desc.add_options()("help,h",
                     "show help")("listen,l", po::value<std::string>()->required(),
                                  "listen address (socket path for ipc, host:port for grpc)")(
      "transport,t", po::value<std::string>()->default_value("grpc"), "transport: ipc or grpc")(
      "cluster-id", po::value<std::string>()->default_value("cvc-cluster"), "cluster identifier")(
      "node-id", po::value<std::string>(), "node identifier (default: random UUID)")(
      "seed,s", po::value<std::vector<std::string>>()->multitoken(),
      "peer endpoint(s) to connect to for clustering")("root-path",
                                                       po::value<std::string>()->default_value(""),
                                                       "subtree to replicate (empty = whole tree)")(
      "sync-mode", po::value<std::string>()->default_value("read-write"),
      "default sync mode: read-only, read-write, authoritative")(
      "enforce-authority", "enforce authority map on remote mutations")("enforce-write-policy",
                                                                        "enforce write policies")(
      "resolve-conflicts", "enable LWW conflict resolution")("tls-cert", po::value<std::string>(),
                                                             "TLS server certificate PEM file")(
      "tls-key", po::value<std::string>(), "TLS server private key PEM file")(
      "tls-ca", po::value<std::string>(), "TLS root CA PEM file")("tls-require-client-auth",
                                                                  "require mutual TLS")(
      "auth-token", po::value<std::string>(),
      "bearer token for authentication (both expected and outbound)")(
      "enable-exec", "enable state_exec script execution engine")(
      "blob-store-path", po::value<std::string>(), "path for blob storage (default: memory-only)")(
      "pump-interval", po::value<int>()->default_value(10),
      "pump loop interval in ms (0 = no pump thread)")(
      "delegate", po::value<std::vector<std::string>>()->multitoken(),
      "delegate subtree: path:cluster_id:endpoint[:lease_seconds]");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::distributed_state_config cfg;
  cfg.cluster_id = vm["cluster-id"].as<std::string>();

  if (vm.count("node-id")) {
    cfg.node_id = vm["node-id"].as<std::string>();
  } else {
    // Generate a simple unique node ID
    std::ostringstream oss;
    oss << "node-" << std::chrono::steady_clock::now().time_since_epoch().count();
    cfg.node_id = oss.str();
  }

  cfg.root_path = vm["root-path"].as<std::string>();
  cfg.listen_address = vm["listen"].as<std::string>();

  std::string tstr = vm["transport"].as<std::string>();
  if (tstr == "ipc")
    cfg.transport = cvc::transport_kind::ipc;
  else if (tstr == "grpc")
    cfg.transport = cvc::transport_kind::grpc;
  else
    throw std::runtime_error("Unknown transport: " + tstr + " (use ipc or grpc)");

  if (vm.count("seed"))
    cfg.seeds = vm["seed"].as<std::vector<std::string>>();

  // Parse sync mode
  cvc::sync_mode smode = cvc::sync_mode::read_write;
  std::string sstr = vm["sync-mode"].as<std::string>();
  if (sstr == "read-only")
    smode = cvc::sync_mode::read_only;
  else if (sstr == "authoritative")
    smode = cvc::sync_mode::authoritative;
  cfg.mounts.push_back({cfg.root_path, smode});

  cfg.enforce_authority = vm.count("enforce-authority");
  cfg.enforce_write_policy = vm.count("enforce-write-policy");
  cfg.resolve_conflicts = vm.count("resolve-conflicts");

  // TLS
  auto read_file_to_string = [](const std::string &path) -> std::string {
    std::ifstream f(path);
    if (!f)
      throw std::runtime_error("Cannot read file: " + path);
    return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
  };
  if (vm.count("tls-cert"))
    cfg.tls_server_cert_pem = read_file_to_string(vm["tls-cert"].as<std::string>());
  if (vm.count("tls-key"))
    cfg.tls_server_key_pem = read_file_to_string(vm["tls-key"].as<std::string>());
  if (vm.count("tls-ca"))
    cfg.tls_root_ca_pem = read_file_to_string(vm["tls-ca"].as<std::string>());
  cfg.tls_require_client_auth = vm.count("tls-require-client-auth") > 0;

  // Auth
  if (vm.count("auth-token")) {
    std::string token = vm["auth-token"].as<std::string>();
    cfg.auth_expected_token = token;
    cfg.auth_outbound_token = token;
  }

  if (vm.count("blob-store-path"))
    cfg.blob_store_path = vm["blob-store-path"].as<std::string>();
  cfg.pump_interval_ms = static_cast<uint32_t>(vm["pump-interval"].as<int>());

  // Join session
  std::cout << "Starting CVC state server...\n"
            << "  cluster: " << cfg.cluster_id << "\n"
            << "  node:    " << cfg.node_id << "\n"
            << "  listen:  " << cfg.listen_address << "\n"
            << "  transport: " << tstr << "\n";
  if (!cfg.seeds.empty()) {
    std::cout << "  seeds:";
    for (auto &s : cfg.seeds)
      std::cout << " " << s;
    std::cout << "\n";
  }

  auto session = cvc::distributed_state_session::join(app, cfg);
  std::cout << "Server running.\n";

  // Process delegations
  if (vm.count("delegate")) {
    for (auto &spec : vm["delegate"].as<std::vector<std::string>>()) {
      // Parse path:cluster_id:endpoint[:lease_seconds]
      std::vector<std::string> parts;
      std::istringstream ss(spec);
      std::string part;
      while (std::getline(ss, part, ':'))
        parts.push_back(part);
      if (parts.size() < 3)
        throw std::runtime_error("Invalid delegation spec: " + spec +
                                 " (expected path:cluster_id:endpoint[:lease_seconds])");
      cvc::delegation_target dt;
      dt.cluster_id = parts[1];
      dt.endpoint = parts[2];
      if (parts.size() > 3)
        dt.lease_duration_ns = std::stoull(parts[3]) * 1000000000ULL;
      session->delegate(parts[0], dt);
      std::cout << "  delegated " << parts[0] << " -> " << dt.cluster_id << " @ " << dt.endpoint
                << "\n";
    }
  }

  // Optional exec coordinator
  std::unique_ptr<cvc::state_exec::scheduler> sched;
  std::unique_ptr<cvc::state_exec::exec_coordinator> coord;
  if (vm.count("enable-exec")) {
    sched = std::make_unique<cvc::state_exec::scheduler>();
    sched->set_watch_root(&cvc::state::instance(app));
    sched->set_id("serve");
    sched->load_settings();
    coord = std::make_unique<cvc::state_exec::exec_coordinator>();
    coord->set_node_id(cfg.node_id);
    coord->set_cluster_id(cfg.cluster_id);
    coord->attach_scheduler(sched.get());
    coord->attach_shard(&session->shard());
    coord->attach_message_bus(&session->shard().message_bus());
    coord->start();
    std::cout << "  exec coordinator: enabled\n";
  }

  // Block until SIGINT/SIGTERM
  std::signal(SIGINT, serve_signal_handler);
  std::signal(SIGTERM, serve_signal_handler);
  std::cout << "Press Ctrl+C to stop.\n";
  while (g_serve_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // If exec enabled, step the scheduler
    if (sched && sched->has_runnable())
      sched->step();
  }

  std::cout << "\nShutting down...\n";
  if (coord)
    coord->stop();
  session->stop();
  std::cout << "Server stopped.\n";
  return 0;
}

// ---------------------------------------------------------------------------
// State exec — run scripts locally or submit to cluster
// ---------------------------------------------------------------------------

static int cmd_exec(int argc, char **argv) {
  po::options_description desc(
      "cvc exec - run state_exec scripts\n\n"
      "Executes a state_exec (Scheme-like) script against the local state\n"
      "tree. The script can read/write state values, perform computations,\n"
      "and interact with the CVC data model.\n\n"
      "Examples:\n"
      "  cvc exec -e '(+ 1 2 3)'\n"
      "  cvc exec -f script.sx\n"
      "  cvc exec -e '(state-set! \"scene.camera.x\" 1.5)'\n");
  desc.add_options()("help,h", "show help")("expression,e", po::value<std::string>(),
                                            "expression to evaluate")(
      "file,f", po::value<std::string>(), "script file to execute")(
      "max-steps", po::value<uint64_t>()->default_value(0), "max steps (0 = unlimited)")(
      "max-time", po::value<double>()->default_value(0.0), "max time in seconds (0 = unlimited)")(
      "name,n", po::value<std::string>()->default_value("cli"), "process name");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  if (!vm.count("expression") && !vm.count("file"))
    throw std::runtime_error("Either -e <expression> or -f <file> is required");

  std::string script;
  if (vm.count("file")) {
    std::ifstream f(vm["file"].as<std::string>());
    if (!f)
      throw std::runtime_error("Cannot read file: " + vm["file"].as<std::string>());
    script = std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
  } else {
    script = vm["expression"].as<std::string>();
  }

  auto &app = cvc_app();
  cvc::state_exec::scheduler sched;
  sched.set_watch_root(&cvc::state::instance(app));
  sched.set_id("exec");
  sched.load_settings();

  cvc::state_exec::execute_options opts;
  opts.name = vm["name"].as<std::string>();
  opts.max_steps = vm["max-steps"].as<uint64_t>();
  opts.max_time = vm["max-time"].as<double>();

  int pid = sched.execute(script, opts);

  // Run the scheduler to completion
  auto results = sched.run();
  auto result = sched.get_result(pid);
  if (result) {
    std::string output = cvc::state_exec::to_string(*result);
    if (!output.empty() && output != "nil")
      std::cout << output << "\n";
  }

  auto info = sched.get_process_info(pid);
  if (info && info->status == cvc::state_exec::process_status::killed)
    return 1;
  return 0;
}

// ---------------------------------------------------------------------------
// State tree commands — get/set/list/export
// ---------------------------------------------------------------------------

static int cmd_state(int argc, char **argv) {
  po::options_description desc("cvc state - query and modify the state tree\n\n"
                               "Operations:\n"
                               "  get <path>            get state value at path\n"
                               "  set <path> <value>    set state value at path\n"
                               "  list [path]           list children of a state node\n"
                               "  json [path]           export subtree as JSON\n"
                               "  delete <path>         delete a state node\n");
  desc.add_options()("help,h", "show help")("op", po::value<std::string>()->required(),
                                            "operation: get, set, list, json, delete")(
      "path", po::value<std::string>()->default_value(""), "state path (dot-separated)")(
      "value", po::value<std::string>(), "value to set (for 'set' operation)")(
      "args", po::value<std::vector<std::string>>(), "additional arguments");

  po::positional_options_description pos;
  pos.add("op", 1).add("path", 1).add("value", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  auto &root = cvc::state::instance(app);
  std::string op = vm["op"].as<std::string>();
  std::string path = vm["path"].as<std::string>();

  if (op == "get") {
    if (path.empty())
      throw std::runtime_error("Path required for 'get'");
    auto &node = root(path);
    if (!node.initialized())
      throw std::runtime_error("State path not initialized: " + path);
    std::cout << node.value() << "\n";
  } else if (op == "set") {
    if (path.empty())
      throw std::runtime_error("Path required for 'set'");
    if (!vm.count("value"))
      throw std::runtime_error("Value required for 'set'");
    root(path).value(vm["value"].as<std::string>());
    std::cout << "Set " << path << " = " << vm["value"].as<std::string>() << "\n";
  } else if (op == "list") {
    auto &node = path.empty() ? root : root(path);
    auto children = node.children();
    for (auto &child : children)
      std::cout << child << "\n";
    if (children.empty())
      std::cout << "(no children)\n";
  } else if (op == "json") {
    auto &node = path.empty() ? root : root(path);
    std::cout << node.json() << "\n";
  } else if (op == "delete") {
    if (path.empty())
      throw std::runtime_error("Path required for 'delete'");
    // Touch with empty to mark; actual deletion depends on state impl
    root(path).value(std::string(""));
    std::cout << "Cleared " << path << "\n";
  } else {
    throw std::runtime_error("Unknown state operation: " + op +
                             " (use get, set, list, json, or delete)");
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Cluster status — admin report
// ---------------------------------------------------------------------------

static int cmd_cluster_status(int argc, char **argv) {
  po::options_description desc("cvc cluster-status - display cluster health report\n\n"
                               "Starts a temporary session connected to a cluster and prints\n"
                               "the admin status report (peers, delegations, blob store, etc.).\n");
  desc.add_options()("help,h", "show help")("listen,l", po::value<std::string>()->required(),
                                            "listen address for temporary session")(
      "transport,t", po::value<std::string>()->default_value("grpc"), "transport: ipc or grpc")(
      "cluster-id", po::value<std::string>()->default_value("cvc-cluster"), "cluster identifier")(
      "seed,s", po::value<std::vector<std::string>>()->multitoken()->required(),
      "peer endpoint(s) to connect to")("auth-token", po::value<std::string>(), "bearer token");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  cvc::distributed_state_config cfg;
  cfg.cluster_id = vm["cluster-id"].as<std::string>();
  {
    std::ostringstream oss;
    oss << "status-" << std::chrono::steady_clock::now().time_since_epoch().count();
    cfg.node_id = oss.str();
  }
  cfg.listen_address = vm["listen"].as<std::string>();
  std::string tstr = vm["transport"].as<std::string>();
  if (tstr == "ipc")
    cfg.transport = cvc::transport_kind::ipc;
  else if (tstr == "grpc")
    cfg.transport = cvc::transport_kind::grpc;
  else
    throw std::runtime_error("Unknown transport: " + tstr);
  cfg.seeds = vm["seed"].as<std::vector<std::string>>();
  cfg.mounts.push_back({"", cvc::sync_mode::read_only});
  if (vm.count("auth-token")) {
    std::string t = vm["auth-token"].as<std::string>();
    cfg.auth_expected_token = t;
    cfg.auth_outbound_token = t;
  }
  cfg.pump_interval_ms = 50;

  auto session = cvc::distributed_state_session::join(app, cfg);

  // Give the session a moment to sync
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  auto &admin = session->admin();
  std::cout << admin.to_text() << "\n";

  auto status = session->status();
  std::cout << "Session status:\n"
            << "  running:     " << (status.running ? "yes" : "no") << "\n"
            << "  peers:       " << status.peer_count << "\n"
            << "  local seq:   " << status.local_sequence << "\n"
            << "  pump cycles: " << status.pump_cycles << "\n";

  session->stop();
  return 0;
}

// ---------------------------------------------------------------------------
// Process listing (state_exec scheduler)
// ---------------------------------------------------------------------------

static int cmd_ps(int argc, char **argv) {
  po::options_description desc(
      "cvc ps - list running state_exec processes\n\n"
      "Shows processes managed by the local state_exec scheduler.\n"
      "Use with 'cvc serve --enable-exec' to see cluster-wide processes.\n");
  desc.add_options()("help,h", "show help");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }

  // Create a scheduler and report (useful mainly for embedded use;
  // in standalone mode there are no persistent processes)
  auto &app = cvc_app();
  cvc::state_exec::scheduler sched;
  sched.set_watch_root(&cvc::state::instance(app));
  sched.set_id("ps");
  sched.load_settings();
  auto procs = sched.list_processes();
  if (procs.empty()) {
    std::cout << "No running processes.\n";
    return 0;
  }
  std::cout << std::left << std::setw(6) << "PID" << std::setw(16) << "NAME" << std::setw(12)
            << "STATUS" << std::setw(10) << "STEPS" << std::setw(10) << "TIME" << std::setw(10)
            << "MEM" << "\n";
  for (auto &p : procs) {
    const char *status_str = "unknown";
    switch (p.status) {
    case cvc::state_exec::process_status::ready:
      status_str = "ready";
      break;
    case cvc::state_exec::process_status::running:
      status_str = "running";
      break;
    case cvc::state_exec::process_status::paused:
      status_str = "paused";
      break;
    case cvc::state_exec::process_status::waiting:
      status_str = "waiting";
      break;
    case cvc::state_exec::process_status::terminated:
      status_str = "done";
      break;
    case cvc::state_exec::process_status::killed:
      status_str = "killed";
      break;
    }
    std::cout << std::left << std::setw(6) << p.pid << std::setw(16) << p.name << std::setw(12)
              << status_str << std::setw(10) << p.step_count << std::setw(10) << std::fixed
              << std::setprecision(2) << p.elapsed_time << std::setw(10) << p.current_memory
              << "\n";
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
  desc.add_options()("help,h", "show help")(
      "port,p", po::value<int>()->default_value(cvc::XMLRPC_DEFAULT_PORT), "server port");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
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
  desc.add_options()("help,h", "show help")("host", po::value<std::string>()->required(),
                                            "host:port")(
      "method", po::value<std::string>()->required(), "RPC method name")(
      "args", po::value<std::vector<std::string>>()->multitoken(), "method arguments");

  po::positional_options_description pos;
  pos.add("host", 1).add("method", 1).add("args", -1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto &app = cvc_app();
  std::vector<std::string> rpc_args;
  if (vm.count("args"))
    rpc_args = vm["args"].as<std::vector<std::string>>();
  std::string result =
      cvc::rpc(app, vm["host"].as<std::string>(), vm["method"].as<std::string>(), rpc_args);
  if (!result.empty())
    std::cout << result << "\n";
  return 0;
}
#endif // USING_XMLRPC

// ===========================================================================
// VolUtils capability wrappers (issue #123)
//
// Part A — expose existing libcvc capabilities as `cvc` subcommands: the noise
// filters (voxels.h), arbitrary resample-to-dims (volume::resize), the trivial
// volume-op wrappers (volume_ops.h), and variable/timestep extraction.  Region-
// restricted stats is wired into `stats --region` above.  Part B novel bit:
// `compare` — a voxel abs-diff pass/fail with a nonzero exit code, the CI-checker
// contract that the similarity-scored `ssim` does not provide.
// ===========================================================================

// ── Filters (voxels.h; legacy VolUtils defaults) ──

static int cmd_bilateral(int argc, char **argv) {
  po::options_description desc("cvc bilateral - edge-preserving bilateral noise filter");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output volume file")(
      "radiometric", po::value<double>()->default_value(200.0),
      "radiometric sigma")("spatial", po::value<double>()->default_value(1.5), "spatial sigma")(
      "radius", po::value<unsigned int>()->default_value(2), "filter radius in voxels");
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  vol.bilateralFilter(vm["radiometric"].as<double>(), vm["spatial"].as<double>(),
                      vm["radius"].as<unsigned int>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_contrast(int argc, char **argv) {
  po::options_description desc("cvc contrast - Zeyun's contrast enhancement");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output volume file")(
      "resistor", po::value<double>()->default_value(0.95), "resistor in [0, 1]");
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  vol.contrastEnhancement(vm["resistor"].as<double>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_anisotropic(int argc, char **argv) {
  po::options_description desc("cvc anisotropic - Zeyun's anisotropic diffusion (edge-preserving)");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output volume file")(
      "iterations,n", po::value<unsigned int>()->default_value(20), "diffusion iterations");
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  vol.anisotropicDiffusion(vm["iterations"].as<unsigned int>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_gdtv(int argc, char **argv) {
  po::options_description desc("cvc gdtv - Dr. Zhang's GDTV filter");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output volume file")(
      "q", po::value<double>()->default_value(1.5), "parameter q (nonlinearity)")(
      "lambda", po::value<double>()->default_value(0.3), "lambda (data fidelity)")(
      "iterations,n", po::value<unsigned int>()->default_value(3),
      "filter iterations")("neighbours", po::value<unsigned int>()->default_value(0),
                           "neighbourhood mode (0 = 6-neighbour)");
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  vol.gdtvFilter(vm["q"].as<double>(), vm["lambda"].as<double>(),
                 vm["iterations"].as<unsigned int>(), vm["neighbours"].as<unsigned int>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

// ── Arbitrary resample-to-dims (volume::resize) ──

static int cmd_resize(int argc, char **argv) {
  po::options_description desc("cvc resize - resample a volume to new voxel dimensions");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(), "output volume file")(
      "dims,d", po::value<std::vector<cvc::uint64>>()->multitoken()->required(),
      "target dimensions: X Y Z");
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto d = vm["dims"].as<std::vector<cvc::uint64>>();
  if (d.size() != 3)
    throw std::runtime_error("resize requires exactly 3 dimensions: --dims X Y Z");

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  vol.resize(cvc::dimension(d[0], d[1], d[2]));
  vol.write(vm["output"].as<std::string>());
  return 0;
}

// ── Trivial op wrappers (volume_ops.h) ──

static int cmd_difference(int argc, char **argv) {
  po::options_description desc("cvc difference - absolute difference |a - b| of two volumes");
  desc.add_options()("help,h", "show help")(
      "input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
      "two input volume files")("output,o", po::value<std::string>()->required(),
                                "output volume file");
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  if (inputs.size() != 2)
    throw std::runtime_error("difference requires exactly 2 input files");

  auto &app = cvc_app();
  cvc::volume a(app), b(app);
  a.read(inputs[0]);
  b.read(inputs[1]);
  cvc::vol_difference(a, b).write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_clamp_min(int argc, char **argv) {
  po::options_description desc("cvc clamp-min - raise voxels below a floor up to that floor");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output volume file")("min,m", po::value<double>()->required(), "minimum value floor");
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>());
  cvc::vol_clamp_min(vol, vm["min"].as<double>()).write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_average(int argc, char **argv) {
  po::options_description desc("cvc average - element-wise average of N volumes (pairwise fold)");
  desc.add_options()("help,h", "show help")(
      "input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
      "two or more input volume files")("output,o", po::value<std::string>()->required(),
                                        "output volume file");
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  if (inputs.size() < 2)
    throw std::runtime_error("average requires at least 2 input files");

  auto &app = cvc_app();
  cvc::volume acc(app);
  acc.read(inputs[0]);
  // N-way fold over the pairwise vol_average, matching the legacy VolUtils
  // behaviour: acc <- avg(acc, next) for each subsequent input.
  for (std::size_t i = 1; i < inputs.size(); ++i) {
    cvc::volume next(app);
    next.read(inputs[i]);
    acc = cvc::vol_average(acc, next);
  }
  acc.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_extract(int argc, char **argv) {
  po::options_description desc("cvc extract - extract a variable/timestep from a multi-var volume");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output volume file")("var", po::value<unsigned int>()->default_value(0), "variable index")(
      "time,t", po::value<unsigned int>()->default_value(0), "timestep index");
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::volume vol(cvc_app());
  vol.read(vm["input"].as<std::string>(), vm["var"].as<unsigned int>(),
           vm["time"].as<unsigned int>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

// ── compare (Part B): voxel abs-diff pass/fail with a nonzero exit code ──

static int cmd_compare(int argc, char **argv) {
  po::options_description desc(
      "cvc compare - exact voxel comparison; exits nonzero on any mismatch");
  desc.add_options()("help,h", "show help")(
      "input,i", po::value<std::vector<std::string>>()->multitoken()->required(),
      "two input volume files")("tolerance,t", po::value<double>()->default_value(0.0),
                                "max allowed absolute per-voxel difference");
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  auto inputs = vm["input"].as<std::vector<std::string>>();
  if (inputs.size() != 2)
    throw std::runtime_error("compare requires exactly 2 input files");

  auto &app = cvc_app();
  cvc::volume a(app), b(app);
  a.read(inputs[0]);
  b.read(inputs[1]);

  if (a.voxel_dimensions() != b.voxel_dimensions()) {
    std::cerr << "MISMATCH: dimensions differ (" << a.XDim() << "x" << a.YDim() << "x" << a.ZDim()
              << " vs " << b.XDim() << "x" << b.YDim() << "x" << b.ZDim() << ")\n";
    return 1;
  }

  const double tol = vm["tolerance"].as<double>();
  cvc::uint64 mismatches = 0;
  double max_diff = 0.0;
  for (cvc::uint64 k = 0; k < a.ZDim(); ++k)
    for (cvc::uint64 j = 0; j < a.YDim(); ++j)
      for (cvc::uint64 i = 0; i < a.XDim(); ++i) {
        double diff = std::abs(a(i, j, k) - b(i, j, k));
        if (diff > max_diff)
          max_diff = diff;
        if (diff > tol)
          ++mismatches;
      }

  if (mismatches > 0) {
    std::cerr << std::setprecision(12) << "MISMATCH: " << mismatches
              << " voxel(s) exceed tolerance " << tol << " (max abs diff " << max_diff << ")\n";
    return 1;
  }
  std::cout << std::setprecision(12) << "OK: volumes match within tolerance " << tol
            << " (max abs diff " << max_diff << ")\n";
  return 0;
}

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
  {"extract",        "Conversion",      "extract a variable/timestep from a volume",      cmd_extract},

  // ── Geometry processing ──
#ifdef CVC_ENABLE_SDF
  {"sdf",            "Geometry",        "compute signed distance field from geometry",    cmd_sdf},
#endif
#ifdef CVC_ENABLE_MESHER
  {"iso",            "Geometry",        "extract isosurface from volume",                 cmd_iso},
  {"tetrahedralize", "Geometry",        "extract tetrahedral mesh from volume",           cmd_tetrahedralize},
  {"hexahedralize",  "Geometry",        "extract hexahedral mesh from volume",            cmd_hexahedralize},
  {"tetrahedralize2","Geometry",        "extract dual-tet (tet2) mesh from volume",       cmd_tetrahedralize2},
  {"layer-mesh",     "Geometry",        "mesh layer between two isosurfaces",             cmd_layer_mesh},
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
  {"difference",     "Vol Arithmetic",  "absolute difference |a - b| of two volumes",    cmd_difference},
  {"clamp-min",      "Vol Arithmetic",  "raise voxels below a floor up to it",           cmd_clamp_min},
  {"average",        "Vol Arithmetic",  "element-wise average of N volumes",             cmd_average},

  // ── Volume transforms ──
  {"rotate",         "Vol Transform",   "rotate volume around Z-axis",                   cmd_rotate},
  {"resize",         "Vol Transform",   "resample volume to new voxel dimensions",       cmd_resize},

  // ── Filters ──
  {"bilateral",      "Filters",         "edge-preserving bilateral noise filter",        cmd_bilateral},
  {"contrast",       "Filters",         "Zeyun's contrast enhancement",                  cmd_contrast},
  {"anisotropic",    "Filters",         "anisotropic diffusion (edge-preserving)",       cmd_anisotropic},
  {"gdtv",           "Filters",         "Dr. Zhang's GDTV filter",                       cmd_gdtv},

  // ── Analysis ──
  {"ssim",           "Analysis",        "compute SSIM between two volumes",              cmd_ssim},
  {"compare",        "Analysis",        "exact voxel comparison (nonzero exit on mismatch)", cmd_compare},

  // ── Projection / Reconstruction ──
  {"project",        "Projection",      "forward ray projection",                        cmd_project},
  {"backproject",    "Projection",      "filtered back-projection (FBP)",                cmd_backproject},

  // ── Image I/O ──
  {"vol2img",        "Image I/O",       "export volume slices as images",                cmd_vol2img},
  {"img2vol",        "Image I/O",       "import image stack as volume",                  cmd_img2vol},
  {"rgba-merge",     "Image I/O",       "merge 4 volumes into RGBA",                     cmd_rgba_merge},

  // ── Test data ──
  {"bunny",          "Test Data",       "output Stanford bunny geometry or SDF",         cmd_bunny},

  // ── State & Distributed ──
  {"serve",          "State Server",    "run headless CVC state server",                 cmd_serve},
  {"exec",           "State Exec",      "run state_exec script",                         cmd_exec},
  {"state",          "State",           "get/set/list state tree values",                cmd_state},
  {"cluster-status", "State Server",    "display cluster health report",                 cmd_cluster_status},
  {"ps",             "State Exec",      "list state_exec processes",                     cmd_ps},

  // ── Network (legacy XMLRPC) ──
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
    std::cout << "    " << std::left << std::setw(18) << commands[i].name << commands[i].help
              << "\n";
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
