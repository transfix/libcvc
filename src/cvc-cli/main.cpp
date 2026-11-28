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
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/host_signals.h>
#include <cvc/core/app.h>
#include <cvc/core/input_trace.h>
#include <cvc/core/param_cli.h>
#include <cvc/core/types.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/geometry/skeleton.h>
#include <cvc/geometry/skeleton_params.h>
#include <cvc/geometry/tiler.h>
#include <cvc/geometry/tiler_params.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/nav_panels.h>
#include <cvc/nav/nav_scrub.h>
#include <cvc/nav/run_log.h>
#include <cvc/nav/sim_world_trace.h>
#include <cvc/ph/compose.h>
#include <cvc/ph/microgrid.h>
#include <cvc/ph/se2_train.h>
#include <cvc/ph/train.h>
#include <cvc/physics/mujoco_robot.h>
#include <cvc/physics/plant_log_scene.h>
#include <cvc/physics/scrub_plant.h>
#include <cvc/state/distributed_state_session.h>
#include <cvc/state/state.h>
#include <cvc/state/state_distributed_admin.h>
#include <cvc/state/state_exec/exec_coordinator.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/utility/algorithm.h>
#include <cvc/utility/utility.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/seed_generators.h>
#include <cvc/volume/seed_generators_params.h>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/segmentation_params.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_filter.h>
#include <cvc/volume/volume_ops.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

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

static int cmd_info(cvc::app &app, int argc, char **argv) {
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

  std::string input = vm["input"].as<std::string>();

  if (cvc::is_geometry_filename(app, input)) {
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
  } else if (cvc::is_volume_filename(app, input)) {
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

static int cmd_stats(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
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

static int cmd_copy(cvc::app &app, int argc, char **argv) {
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

  cvc::save(app, cvc::load(app, vm["input"].as<std::string>()), vm["output"].as<std::string>());
  return 0;
}

static int cmd_convert(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
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
static int cmd_sdf(cvc::app &app, int argc, char **argv) {
  po::options_description desc("cvc sdf - compute signed distance field from geometry\n\n"
                               "Algorithms:\n"
                               "  v1   Original SDFLibrary (octree-based, thread-safe) [default]\n"
                               "  v2   DistanceTransform (brute-force)\n"
                               "  igl  libigl exact distance, winding-number sign (any dims)\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input geometry file")(
      "output,o", po::value<std::string>()->required(),
      "output volume file")("dim,d", po::value<std::string>()->default_value("64,64,64"),
                            "output dimensions (NxNxN or X,Y,Z)")(
      "bbox,b", po::value<std::string>(),
      "bounding box (minx,miny,minz,maxx,maxy,maxz); defaults to geometry extents")(
      "algorithm,a", po::value<std::string>()->default_value("v1"),
      "SDF algorithm: v1, v2 or igl")("flip-normals", "flip normals to invert inside/outside");

  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

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
  else if (algo_str == "igl" || algo_str == "IGL")
    algo = cvc::SDF_IGL;

  bool flip = vm.count("flip-normals") > 0;

  cvc::volume result = cvc::sdf(app, geom, dim, bbox, algo, flip);
  result.write(vm["output"].as<std::string>());
  std::cout << "Wrote SDF volume to " << vm["output"].as<std::string>() << "\n";
  return 0;
}
#endif // CVC_ENABLE_SDF

#ifdef CVC_ENABLE_MESHER
static int cmd_iso(cvc::app &app, int argc, char **argv) {
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

  cvc::write_geometry(app, result, vm["output"].as<std::string>());
  std::cout << "Wrote isosurface (" << result.num_points() << " verts, " << result.num_tris()
            << " tris) to " << vm["output"].as<std::string>() << "\n";
  return 0;
}

static int cmd_tetrahedralize(cvc::app &app, int argc, char **argv) {
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

  cvc::write_geometry(app, result, vm["output"].as<std::string>());
  std::cout << "Wrote tetrahedral mesh (" << result.num_points() << " verts) to "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}

static int cmd_hexahedralize(cvc::app &app, int argc, char **argv) {
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

  cvc::write_geometry(app, result, vm["output"].as<std::string>());
  std::cout << "Wrote hexahedral mesh (" << result.num_points() << " verts) to "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}

// ---------------------------------------------------------------------------
// cvc skeleton: cvc::skeletonize (cvc/geometry/skeleton.h), the VolumeRover Skeletonization port
// ---------------------------------------------------------------------------

// The legacy Skeletonization tool's flags (its main.cpp), rewritten to this command's: -r
// (--robust), -bbr r (--big-ball-ratio r^2), -thif / -thff degrees (--theta-if / --theta-ff in
// radians), -plc n (--sheet-count), -medth (--medial-theta), -medr (--medial-ratio). -mr and -hc
// took a value the legacy code never used; they are accepted and ignored.
static std::vector<std::string> skeleton_legacy_flags(int argc, char **argv,
                                                      std::vector<std::string> &notes) {
  std::vector<std::string> out;
  const double deg = 3.14159265358979323846 / 180.0;
  auto value = [&](int &i, const std::string &flag) -> std::string {
    if (i + 1 >= argc)
      throw std::runtime_error("cvc skeleton: " + flag + " needs a value");
    return argv[++i];
  };
  // argv[0] is the command, as command_line_parser(argc, argv) skips it.
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-r") {
      out.push_back("--robust");
    } else if (a == "-bbr") {
      const double r = std::stod(value(i, a));
      out.push_back("--big-ball-ratio=" + boost::lexical_cast<std::string>(r * r));
    } else if (a == "-thif" || a == "-thff") {
      const double d = std::stod(value(i, a));
      out.push_back(std::string(a == "-thif" ? "--theta-if=" : "--theta-ff=") +
                    boost::lexical_cast<std::string>(d * deg));
    } else if (a == "-plc") {
      out.push_back("--sheet-count=" + value(i, a));
    } else if (a == "-medth") {
      out.push_back("--medial-theta=" + value(i, a));
    } else if (a == "-medr") {
      out.push_back("--medial-ratio=" + value(i, a));
    } else if (a == "-mr" || a == "-hc") {
      value(i, a);
      notes.push_back(a + " is accepted and ignored, as the legacy tool ignored it");
    } else {
      out.push_back(a);
    }
  }
  return out;
}

static int cmd_skeleton(cvc::app &app, int argc, char **argv) {
  po::options_description desc(
      "cvc skeleton - the skeleton of a point cloud or a mesh's vertices (the VolumeRover "
      "Skeletonization port)\n\n"
      "Writes the strands (lines: the polylines and the stars of small planar clusters) to the "
      "output, and the sheets (triangles) beside it as <stem>_sheets<ext>, unless --sheets says "
      "where. .rawc keeps the colours. The legacy flags -r -bbr -thif -thff -plc -medth -medr "
      "(and the ignored -mr -hc) are accepted.\n\n"
      "Example:\n"
      "  cvc skeleton points.off -o skel.rawc --robust\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input geometry (only its points are read)")(
      "output,o", po::value<std::string>()->required(), "output geometry for the strands")(
      "sheets", po::value<std::string>(), "output geometry for the sheets")(
      "helix", po::value<std::string>(),
      "output geometry for the helix strips")("seconds", po::value<double>()->default_value(0.0),
                                              "report progress every this many seconds (0: never)");
  cvc::add_param_options<cvc::skeleton_params>(desc);

  std::vector<std::string> notes;
  const std::vector<std::string> args = skeleton_legacy_flags(argc, argv, notes);
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(args).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);
  for (const std::string &n : notes)
    std::cerr << "cvc skeleton: " << n << "\n";
  const cvc::param_result<cvc::skeleton_params> pr =
      cvc::params_from_options<cvc::skeleton_params>(vm);
  if (!pr.ok())
    throw std::runtime_error("cvc skeleton: " + pr.message());

  const std::string input = vm["input"].as<std::string>();
  const std::string output = vm["output"].as<std::string>();
  cvc::geometry in(app, input);
  if (in.num_points() == 0)
    throw std::runtime_error("cvc skeleton: " + input + " has no points");

  // The stepper, so a long run can report its progress.
  cvc::skeleton_stepper stepper(in, pr.value);
  const double every = vm["seconds"].as<double>();
  auto last = std::chrono::steady_clock::now();
  while (!stepper.step(4096)) {
    if (every > 0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last).count() >= every) {
      std::cerr << "cvc skeleton: " << stepper.status() << "\n";
      last = std::chrono::steady_clock::now();
    }
  }
  const cvc::skeleton s = stepper.take();

  std::string sheets = vm.count("sheets") ? vm["sheets"].as<std::string>() : std::string();
  if (sheets.empty()) {
    const std::size_t dot = output.find_last_of('.');
    const std::size_t sep = output.find_last_of("/\\");
    const bool has_ext = dot != std::string::npos && (sep == std::string::npos || dot > sep);
    sheets = has_ext ? output.substr(0, dot) + "_sheets" + output.substr(dot) : output + "_sheets";
  }
  cvc::write_geometry(app, s.strands, output);
  cvc::write_geometry(app, s.sheets, sheets);
  if (vm.count("helix"))
    cvc::write_geometry(app, s.helix, vm["helix"].as<std::string>());
  for (const std::string &w : s.warnings)
    std::cerr << "cvc skeleton: warning: " << w << "\n";
  std::cout << "Wrote " << s.polylines << " strands (" << s.strands.num_lines() << " lines) to "
            << output << " and " << s.sheet_clusters << " sheets (" << s.sheets.num_tris()
            << " triangles, of " << s.clusters << " planar clusters) to " << sheets << "\n";
  return 0;
}

static int cmd_tetrahedralize2(cvc::app &app, int argc, char **argv) {
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

  cvc::write_geometry(app, result, vm["output"].as<std::string>());
  std::cout << "Wrote dual-tet mesh (" << result.num_points() << " verts) to "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}

static int cmd_layer_mesh(cvc::app &app, int argc, char **argv) {
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

  cvc::write_geometry(app, result, vm["output"].as<std::string>());
  std::cout << "Wrote layer mesh (" << result.num_points() << " verts) to "
            << vm["output"].as<std::string>() << "\n";
  return 0;
}
#endif // CVC_ENABLE_MESHER

// ---------------------------------------------------------------------------
// cvc tile: cvc::tile_contours (cvc/geometry/tiler.h), the VolumeRover ContourTiler port
// ---------------------------------------------------------------------------

// A component name as a regular expression matching exactly it, quoted for a list flag's value.
static std::string tile_name_item(const std::string &name) {
  std::string re;
  for (char ch : name) {
    if (std::strchr("\\^$.|?*+()[]{}", ch))
      re += '\\';
    re += ch;
  }
  std::string quoted = "\"";
  for (char ch : re) {
    if (ch == '\\' || ch == '"')
      quoted += '\\';
    quoted += ch;
  }
  return quoted + "\"";
}

// The legacy contour_tiler's flags (its cl_options.cpp), rewritten to this command's:
//   -C/--curationdelta d, -e/--collinearepsilon e, -r/--removeintersections and
//   -v/--correspondoverlap a are the tiling parameters; -c/--componentname and
//   -x/--componentskip (each repeatable, an exact name) become --include and --exclude;
//   -s/--slices first last is --z-first and --z-last; -z/--zscale s is --z-spacing (the slice
//   spacing); --color r g b (0-255) is --color; -d/--outdir, -b/--basename and -n/--indir keep
//   their meaning; -o raw is --extension rawc; --printcomponents is --print-components.
// Accepted and ignored, with a note: -f (the reader goes by the input's extension), -m (the
// legacy read it only for its .dat inputs), -F (the legacy never read it), -S -1, and the debug
// outputs -i -p -R (each with a value) and -B -T -O -P -V -U. Refused: -o gnuplot, -I and any
// other -S, which the port does not do.
static std::vector<std::string> tile_legacy_flags(int argc, char **argv,
                                                  std::vector<std::string> &notes) {
  std::vector<std::string> out, include, exclude;
  auto value = [&](int &i, const std::string &flag) -> std::string {
    if (i + 1 >= argc)
      throw std::runtime_error("cvc tile: " + flag + " needs a value");
    return argv[++i];
  };
  auto is = [](const std::string &a, const char *s, const char *l) { return a == s || a == l; };
  // argv[0] is the command, as command_line_parser(argc, argv) skips it.
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (is(a, "-C", "--curationdelta")) {
      out.push_back("--curation-delta=" + value(i, a));
    } else if (is(a, "-e", "--collinearepsilon")) {
      out.push_back("--collinear-epsilon=" + value(i, a));
    } else if (is(a, "-r", "--removeintersections")) {
      out.push_back("--remove-intersections");
    } else if (is(a, "-v", "--correspondoverlap")) {
      out.push_back("--correspondence-overlap=" + value(i, a));
    } else if (is(a, "-c", "--componentname")) {
      include.push_back(tile_name_item(value(i, a)));
    } else if (is(a, "-x", "--componentskip")) {
      exclude.push_back(tile_name_item(value(i, a)));
    } else if (is(a, "-s", "--slices")) {
      out.push_back("--z-first=" + value(i, a));
      out.push_back("--z-last=" + value(i, a));
    } else if (is(a, "-z", "--zscale")) {
      out.push_back("--z-spacing=" + value(i, a));
    } else if (a == "--color") {
      std::string c = "--color=[";
      for (int k = 0; k < 3; ++k)
        c += (k ? ", " : "") + boost::lexical_cast<std::string>(std::stod(value(i, a)) / 255.0);
      out.push_back(c + "]");
    } else if (is(a, "-d", "--outdir")) {
      out.push_back("--outdir=" + value(i, a));
    } else if (is(a, "-b", "--basename")) {
      out.push_back("--basename=" + value(i, a));
    } else if (is(a, "-n", "--indir")) {
      out.push_back("--indir=" + value(i, a));
    } else if (is(a, "-o", "--outformat")) {
      const std::string f = value(i, a);
      if (f != "raw")
        throw std::runtime_error("cvc tile: " + a + " " + f +
                                 " is not supported: the surfaces are geometry files "
                                 "(--extension rawc, off, ply, ...)");
      out.push_back("--extension=rawc");
    } else if (a == "--printcomponents") {
      out.push_back("--print-components");
    } else if (is(a, "-f", "--format")) {
      value(i, a);
      notes.push_back(a + " is accepted and ignored: the input's extension picks its reader");
    } else if (is(a, "-m", "--multi") || is(a, "-F", "--nointerpur")) {
      notes.push_back(a + " is accepted and ignored, as the legacy tool ignored it for a series");
    } else if (is(a, "-S", "--smoothingfactor")) {
      const std::string s = value(i, a);
      if (std::stod(s) != -1)
        throw std::runtime_error("cvc tile: " + a + " " + s +
                                 " is not supported: the port does not smooth contours");
    } else if (is(a, "-I", "--2dintremove")) {
      throw std::runtime_error("cvc tile: " + a +
                               " (2D intersection removal only) is not supported");
    } else if (is(a, "-i", "--intformat") || is(a, "-p", "--phaseformat") ||
               is(a, "-R", "--defaultprecision")) {
      value(i, a);
      notes.push_back(a + " (debug output) is accepted and ignored");
    } else if (is(a, "-B", "--outputbottom") || is(a, "-T", "--outputtop") ||
               is(a, "-O", "--outputotv") || is(a, "-P", "--outputotvpairs") ||
               is(a, "-V", "--outputvertexlabels") || is(a, "-U", "--outputuntiledregions")) {
      notes.push_back(a + " (debug output) is accepted and ignored");
    } else {
      out.push_back(a);
    }
  }
  auto list = [](const std::vector<std::string> &items) {
    std::string s = "[";
    for (std::size_t k = 0; k < items.size(); ++k)
      s += (k ? ", " : "") + items[k];
    return s + "]";
  };
  if (!include.empty())
    out.push_back("--include=" + list(include));
  if (!exclude.empty())
    out.push_back("--exclude=" + list(exclude));
  return out;
}

static int cmd_tile(cvc::app &app, int argc, char **argv) {
  po::options_description desc(
      "cvc tile - surfaces between the contours of a stack of slices (the VolumeRover "
      "ContourTiler port)\n\n"
      "Reads a contour stack (a Reconstruct .ser series, .contour, .cvcc, ...) and writes one "
      "surface per component, <outdir>/<basename><component>_tiles.<ext>, and every component "
      "together as <outdir>/<basename>_tiles.<ext>. .rawc keeps the colours. The legacy flags "
      "-C -e -r -v -c -x -s -z --color -d -b -n -o raw --printcomponents are accepted (and the "
      "debug flags, ignored).\n\n"
      "Examples:\n"
      "  cvc tile series.ser -d out -C 0.01 -e 1e-15 -o raw\n"
      "  cvc tile series.ser --outdir out --include a001 --z-first 40 --z-last 80\n");
  desc.add_options()("help,h", "show help")("input", po::value<std::string>()->required(),
                                            "the contour stack")(
      "outdir", po::value<std::string>()->default_value("."), "directory for the surfaces")(
      "basename", po::value<std::string>()->default_value(""),
      "prefix of the surfaces' file names")("indir", po::value<std::string>()->default_value(""),
                                            "directory the input is in")(
      "extension", po::value<std::string>()->default_value("rawc"),
      "the surfaces' format, as a file extension (rawc, off, ply, ...)")(
      "z-spacing", po::value<double>()->default_value(0.0),
      "the slice spacing in world units, in place of the file's (0: the file's)")(
      "print-components", po::bool_switch(),
      "list the stack's components and stop")("seconds", po::value<double>()->default_value(0.0),
                                              "report progress every this many seconds (0: never)");
  cvc::add_param_options<cvc::tile_params>(desc);

  std::vector<std::string> notes;
  const std::vector<std::string> args = tile_legacy_flags(argc, argv, notes);
  po::positional_options_description pos;
  pos.add("input", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(args).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);
  for (const std::string &n : notes)
    std::cerr << "cvc tile: " << n << "\n";
  const cvc::param_result<cvc::tile_params> pr = cvc::params_from_options<cvc::tile_params>(vm);
  if (!pr.ok())
    throw std::runtime_error("cvc tile: " + pr.message());
  const cvc::tile_params &p = pr.value;

  std::string input = vm["input"].as<std::string>();
  const std::string indir = vm["indir"].as<std::string>();
  if (!indir.empty())
    input = (fs::path(indir) / input).string();
  cvc::contour_read_params rp;
  rp.first = p.z_first;
  rp.last = p.z_last;
  rp.include = p.include;
  rp.exclude = p.exclude;
  rp.z_spacing = vm["z-spacing"].as<double>();
  const cvc::contour_stack stack = cvc::read_contours(app, input, rp);
  if (vm["print-components"].as<bool>()) {
    for (const std::string &name : stack.component_names())
      std::cout << name << "\n";
    return 0;
  }
  if (stack.empty())
    throw std::runtime_error("cvc tile: " + input + " has no contours in the slices asked for");

  // The stepper, so a long run can report its progress.
  cvc::tile_stepper stepper(stack, p);
  const double every = vm["seconds"].as<double>();
  auto last = std::chrono::steady_clock::now();
  while (!stepper.step(4096)) {
    if (every > 0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last).count() >= every) {
      std::cerr << "cvc tile: " << stepper.status() << "\n";
      last = std::chrono::steady_clock::now();
    }
  }
  cvc::tile_result r = stepper.take();

  const fs::path outdir(vm["outdir"].as<std::string>());
  if (!outdir.empty())
    fs::create_directories(outdir);
  const std::string base = vm["basename"].as<std::string>();
  const std::string ext = "." + vm["extension"].as<std::string>();
  cvc::geometry all(app);
  std::uint64_t triangles = 0;
  for (auto &[name, g] : r.components) {
    const std::string file = (outdir / (base + name + "_tiles" + ext)).string();
    cvc::write_geometry(app, g, file);
    std::cout << "  " << name << ": " << g.num_tris() << " triangles to " << file << "\n";
    triangles += g.num_tris();
    all.merge(g);
  }
  for (const std::string &w : r.warnings)
    std::cerr << "cvc tile: warning: " << w << "\n";
  const std::string file = (outdir / (base + "_tiles" + ext)).string();
  cvc::write_geometry(app, all, file);
  std::cout << "Wrote " << r.components.size() << " surfaces (" << triangles << " triangles) to "
            << file << "\n";
  return 0;
}

// ---------------------------------------------------------------------------
// Volume arithmetic commands
// ---------------------------------------------------------------------------

static int cmd_add(cvc::app &app, int argc, char **argv) {
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

  cvc::volume a(app), b(app);
  a.read(inputs[0]);
  b.read(inputs[1]);
  cvc::volume result = cvc::vol_add(a, b);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_subtract(cvc::app &app, int argc, char **argv) {
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

  cvc::volume a(app), b(app);
  a.read(inputs[0]);
  b.read(inputs[1]);
  cvc::volume result = cvc::vol_subtract(a, b);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_scale(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_scale(vol, vm["factor"].as<double>());
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_normalize(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_normalize(vol, vm["min"].as<double>(), vm["max"].as<double>());
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_clip(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_clip(vol, vm["threshold"].as<double>());
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_negate(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_negate(vol);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_mask(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app), mask_vol(app);
  vol.read(vm["input"].as<std::string>());
  mask_vol.read(vm["mask"].as<std::string>());

  cvc::volume result =
      vm.count("inverse") ? cvc::vol_inverse_mask(vol, mask_vol) : cvc::vol_mask(vol, mask_vol);
  result.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_downsample(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  unsigned int f = vm["factor"].as<unsigned int>();
  cvc::volume result = cvc::vol_downsample(vol, f, f, f);
  result.write(vm["output"].as<std::string>());
  return 0;
}

// ---------------------------------------------------------------------------
// Volume transform commands
// ---------------------------------------------------------------------------

static int cmd_rotate(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
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

static int cmd_ssim(cvc::app &app, int argc, char **argv) {
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

static int cmd_project(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  cvc::volume result = cvc::vol_project(vol, angles, vm["step"].as<double>());
  result.write(vm["output"].as<std::string>());
  std::cout << "Projected " << angles.size() << " angles -> " << vm["output"].as<std::string>()
            << "\n";
  return 0;
}

static int cmd_backproject(cvc::app &app, int argc, char **argv) {
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

  cvc::volume proj(app);
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

static int cmd_vol2img(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());

  std::string dir = vm["dir"].as<std::string>();
  fs::create_directories(dir);

  cvc::vol_to_slices(vol, dir, vm["format"].as<std::string>());
  std::cout << "Exported " << vol.ZDim() << " slices to " << dir << "/\n";
  return 0;
}

static int cmd_img2vol(cvc::app &app, int argc, char **argv) {
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
  cvc::volume result = cvc::slices_to_volume(app, inputs);
  result.write(vm["output"].as<std::string>());
  std::cout << "Imported " << inputs.size() << " images -> " << vm["output"].as<std::string>()
            << "\n";
  return 0;
}

static int cmd_rgba_merge(cvc::app &app, int argc, char **argv) {
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

static int cmd_bunny(cvc::app &app, int argc, char **argv) {
  po::options_description desc("cvc bunny - output Stanford bunny geometry or SDF volume");
  desc.add_options()("help,h",
                     "show help")("output,o", po::value<std::string>()->required(),
                                  "output file (.off for geometry, .rawiv/.mrc for volume)")(
      "volume", "output SDF volume instead of geometry")(
      "dims,d", po::value<unsigned int>()->default_value(64), "volume dimensions (cube)")(
      "padding,p", po::value<double>()->default_value(0.1),
      "bounding box padding factor")("algorithm,a", po::value<std::string>()->default_value("v1"),
                                     "SDF algorithm: v1 (octree), v2 (distance transform) or "
                                     "igl (libigl exact distance)");

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

  cvc::geometry bunny(app); // its app's handlers read and write it
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
    else if (algo_str == "igl" || algo_str == "IGL")
      algo = cvc::SDF_IGL;
    cvc::volume sdf_vol = cvc::sdf(app, bunny, cvc::dimension(d, d, d), bbox, algo);
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

static int cmd_serve(cvc::app &app, int argc, char **argv) {
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
      "cluster-id", po::value<std::string>()->default_value("cvc_cluster"), "cluster identifier")(
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

  cvc::distributed_state_config cfg;
  cfg.cluster_id = vm["cluster-id"].as<std::string>();

  if (vm.count("node-id")) {
    cfg.node_id = vm["node-id"].as<std::string>();
  } else {
    // Generate a simple unique node ID. Must be a valid C identifier
    // (state::isValidStateName forbids '-'), so use an underscore separator.
    std::ostringstream oss;
    oss << "node_" << std::chrono::steady_clock::now().time_since_epoch().count();
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

  // Install the shutdown handlers BEFORE join() binds the listening socket.
  // The socket appearing is what tells the outside world this server exists,
  // so anything watching for it can signal us the instant it shows up. With
  // the handlers installed afterwards there is a window -- join() returning,
  // through delegation parsing and exec-coordinator startup, to the old
  // install site below -- in which SIGINT still had its default disposition
  // and killed the process outright, discarding the buffered log with it. They set this app's
  // stop flag (host_signals.h), which the loop below polls; the route goes before the app.
  cvc::ariadne::signal_route stopRoute(app, cvc::ariadne::signal_route::mode::stop);
  const cvc::ariadne::host_signals &signals = cvc::ariadne::signals_of(app);

  auto session = cvc::distributed_state_session::join(app, cfg);
  // Flush the readiness banner: stdout is fully buffered when it is a file or
  // a pipe, so without this the startup lines exist only in memory and are
  // lost if the process is later killed hard.
  std::cout << "Server running." << std::endl;

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

  // Block until SIGINT/SIGTERM (handlers were installed before join() above).
  std::cout << "Press Ctrl+C to stop." << std::endl;
  while (!signals.stop_requested()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // If exec enabled, step the scheduler: for a runnable process, or for a command posted to it
    // (the coordinator's bus threads post_control), which only a step applies.
    if (sched && (sched->has_runnable() || sched->has_pending_control())) {
      try {
        sched->step();
      } catch (const std::exception &e) {
        // A posted command that throws (a kill whose lock release or tree write a slot refuses),
        // or a step's own error: logged, and the server goes on. The commands after it stay
        // queued for the next step.
        std::cerr << "exec: " << e.what() << "\n";
      }
    }
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

static int cmd_exec(cvc::app &app, int argc, char **argv) {
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

static int cmd_state(cvc::app &app, int argc, char **argv) {
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

static int cmd_cluster_status(cvc::app &app, int argc, char **argv) {
  po::options_description desc("cvc cluster-status - display cluster health report\n\n"
                               "Starts a temporary session connected to a cluster and prints\n"
                               "the admin status report (peers, delegations, blob store, etc.).\n");
  desc.add_options()("help,h", "show help")("listen,l", po::value<std::string>()->required(),
                                            "listen address for temporary session")(
      "transport,t", po::value<std::string>()->default_value("grpc"), "transport: ipc or grpc")(
      "cluster-id", po::value<std::string>()->default_value("cvc_cluster"), "cluster identifier")(
      "seed,s", po::value<std::vector<std::string>>()->multitoken()->required(),
      "peer endpoint(s) to connect to")("auth-token", po::value<std::string>(), "bearer token");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  cvc::distributed_state_config cfg;
  cfg.cluster_id = vm["cluster-id"].as<std::string>();
  {
    // Node id must be a valid C identifier (state::isValidStateName forbids
    // '-'), so use an underscore separator.
    std::ostringstream oss;
    oss << "status_" << std::chrono::steady_clock::now().time_since_epoch().count();
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

static int cmd_ps(cvc::app &app, int argc, char **argv) {
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
#include <cvc/state/state.h>

static int cmd_server(cvc::app &app, int argc, char **argv) {
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

  int port = vm["port"].as<int>();
  cvc::state::instance(app)("__system.xmlrpc.port").value(port);
  cvc::state::instance(app)("__system.xmlrpc").value(int(1));
  app.wait();
  return 0;
}

static int cmd_client(cvc::app &app, int argc, char **argv) {
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

static int cmd_bilateral(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  vol.bilateralFilter(vm["radiometric"].as<double>(), vm["spatial"].as<double>(),
                      vm["radius"].as<unsigned int>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_contrast(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  vol.contrastEnhancement(vm["resistor"].as<double>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_anisotropic(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  vol.anisotropicDiffusion(vm["iterations"].as<unsigned int>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_gdtv(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  vol.gdtvFilter(vm["q"].as<double>(), vm["lambda"].as<double>(),
                 vm["iterations"].as<unsigned int>(), vm["neighbours"].as<unsigned int>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

// ── The filter jobs (cvc/volume/volume_filter.h; VOLROVER-TEXMOL-APPS-ROADMAP.md V0) ──
//
// The four filters above through cvc::volume_filter_stepper, the resumable form the DSL's
// volume.filter, pycvc and Ariadne's volume_filters component run: flags and defaults from the
// parameter table (VolumeRover2's dialogs' defaults), every value in the volume's own voxel type.
// The legacy commands keep VolUtils' flags and defaults.

static int cmd_filter(cvc::app &app, int argc, char **argv) {
  po::options_description desc(
      "cvc filter - VolumeRover2's volume filters (bilateral, contrast, anisotropic, gdtv) as a "
      "resumable job\n\n"
      "--filter picks one; each reads only its own flags. The defaults are VolumeRover2's "
      "dialogs'. "
      "Each value is written in the input's voxel type, and the result equals the legacy commands' "
      "(bilateral, contrast, anisotropic, gdtv) voxel for voxel, but bilateral's on a type other "
      "than float and double, which the legacy writes as bytes.\n\n"
      "Examples:\n"
      "  cvc filter ct.rawiv smooth.rawiv --filter anisotropic --anisotropic-iterations 10\n"
      "  cvc filter ct.rawiv tv.rawiv --filter gdtv --gdtv-exponent 1 --gdtv-iterations 3\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume file")(
      "output,o", po::value<std::string>()->required(),
      "output volume file")("seconds", po::value<double>()->default_value(0.0),
                            "report progress every this many seconds (0: never)");
  cvc::add_param_options<cvc::volume_filter_params>(desc);
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);
  const cvc::param_result<cvc::volume_filter_params> pr =
      cvc::params_from_options<cvc::volume_filter_params>(vm);
  if (!pr.ok())
    throw std::runtime_error("cvc filter: " + pr.message());

  const std::string output = vm["output"].as<std::string>();
  auto in = std::make_shared<cvc::volume>(app);
  in->read(vm["input"].as<std::string>());
  cvc::volume_filter_stepper stepper(in, pr.value);
  const double every = vm["seconds"].as<double>();
  auto last = std::chrono::steady_clock::now();
  while (!stepper.step(65536)) {
    if (every > 0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last).count() >= every) {
      std::cerr << "cvc filter: " << stepper.status() << "\n";
      last = std::chrono::steady_clock::now();
    }
  }
  const std::shared_ptr<const cvc::volume> out = stepper.take();
  cvc::volume result(*out);
  result.write(output);
  std::cout << "Wrote " << output << ": " << stepper.status() << "\n";
  return 0;
}

// ── Arbitrary resample-to-dims (volume::resize) ──

static int cmd_resize(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  vol.resize(cvc::dimension(d[0], d[1], d[2]));
  vol.write(vm["output"].as<std::string>());
  return 0;
}

// ── Trivial op wrappers (volume_ops.h) ──

static int cmd_difference(cvc::app &app, int argc, char **argv) {
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

  cvc::volume a(app), b(app);
  a.read(inputs[0]);
  b.read(inputs[1]);
  cvc::vol_difference(a, b).write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_clamp_min(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  cvc::vol_clamp_min(vol, vm["min"].as<double>()).write(vm["output"].as<std::string>());
  return 0;
}

static int cmd_average(cvc::app &app, int argc, char **argv) {
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

static int cmd_extract(cvc::app &app, int argc, char **argv) {
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

  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>(), vm["var"].as<unsigned int>(),
           vm["time"].as<unsigned int>());
  vol.write(vm["output"].as<std::string>());
  return 0;
}

// ── compare (Part B): voxel abs-diff pass/fail with a nonzero exit code ──

static int cmd_compare(cvc::app &app, int argc, char **argv) {
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

// ---------------------------------------------------------------------------
// Vehicle physics
// ---------------------------------------------------------------------------

static int cmd_plant_log(cvc::app &app, int argc, char **argv) {
  po::options_description desc(
      "cvc plant-log - nav driving physics vehicles on Jolt, logged for fitting a plant model\n\n"
      "Logs every vehicle tick by tick as a planar rigid body, cuts the runs into flights\n"
      "(flights.npy, meta.json; tools/liesphgp_pack.py packs them as a LieSPHGP dataset) and\n"
      "reports the dynamic single-track model's one-tick residual against them\n"
      "(residual.json). Scenes (cvc/physics/plant_log_scene.h):\n"
      "  swerve  open flat ground, each agent slaloming\n"
      "  city    the coef trainer's city, scaled to the vehicle\n"
      "  bundle  a cvc::world city bundle (--bundle DIR), driven on vehicle routes\n\n"
      "  cvc plant-log --vehicle humvee --scene swerve --seeds 100,101,102 --out runs/swerve\n");
  cvc::physics::plant_log_options o;
  std::string seeds = "100,101,102,103", steer = "actual", policy;
  desc.add_options()("help,h", "show help")("out,o", po::value<std::string>(&o.out)->required(),
                                            "output directory")(
      "vehicle", po::value<std::string>(&o.vehicle)->default_value(o.vehicle),
      "humvee or roboracer")("scene", po::value<std::string>(&o.scene)->default_value(o.scene),
                             "swerve, city or bundle")("bundle", po::value<std::string>(&o.bundle),
                                                       "city bundle directory (--scene bundle)")(
      "seeds", po::value<std::string>(&seeds)->default_value(seeds),
      "comma-separated seeds, one run per agent each")(
      "agents", po::value<int>(&o.agents)->default_value(o.agents),
      "vehicles per seed (< 0: 4 in swerve, 8 otherwise)")(
      "seconds", po::value<double>(&o.seconds)->default_value(o.seconds),
      "seconds per seed")("dt", po::value<double>(&o.dt)->default_value(o.dt),
                          "nav's tick, a whole number of 0.01 s physics steps")(
      "mu", po::value<double>(&o.mu)->default_value(o.mu), "the ground's friction")(
      "lateral-g", po::value<double>(&o.lateral_g)->default_value(o.lateral_g),
      "nav's corner cap in g (< 0: 0.9 in swerve, 0.5 otherwise)")(
      "flight-seconds", po::value<double>(&o.flight_seconds)->default_value(o.flight_seconds),
      "a flight's length")("stride-seconds",
                           po::value<double>(&o.stride_seconds)->default_value(o.stride_seconds),
                           "from one flight's start to the next's")(
      "steer", po::value<std::string>(&steer)->default_value(steer),
      "the steer input: actual (the road-wheel angle's mean over the tick) or command")(
      "nsub", po::value<int>(&o.nsub)->default_value(o.nsub),
      "residual substeps a tick (0: the physics steps a tick)")(
      "grid", po::value<int>(&o.grid)->default_value(o.grid),
      "bundle: grid cells a side")("inflate", po::value<int>(&o.inflate)->default_value(o.inflate),
                                   "bundle: obstacle inflation in cells")(
      "accel-margin", po::value<double>(&o.accel_margin)->default_value(o.accel_margin),
      "the share of the vehicle's acceleration nav plans with (<= 0: the plant's)")(
      "brake-margin", po::value<double>(&o.brake_margin)->default_value(o.brake_margin),
      "the share of its braking (<= 0: the plant's)")(
      "policy", po::value<std::string>(&policy), "a .cvcnav policy (default: the built-in one)")(
      "verbose", po::bool_switch(&o.verbose), "every agent's state every 5 s")(
      "samples", po::bool_switch(&o.samples), "also write samples.csv, every tick of every run")(
      "keep-contact", po::bool_switch(&o.keep_contact),
      "keep the flights with a contact or an impact, with a last column, the contact feature "
      "(the contacts the interval began; cut_flights says what a learned plant can learn from "
      "it)")(
      "wide", po::bool_switch(&o.wide),
      "flights.npy in the wide layout: also the attitude, the wheels' speeds, the pedals and the "
      "ledger's shaft and brake power (18 columns; meta.json's layout names them)")(
      "fit-dyn", po::bool_switch(&o.fit_dyn),
      "fit dyn_bike to every seed but the last and report the last")(
      "fit-power", po::bool_switch(&o.fit_power),
      "fit the power model (P8c) to the ledger of every seed but the last and report the last");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  o.seeds.clear();
  std::stringstream ss(seeds);
  for (std::string t; std::getline(ss, t, ',');)
    o.seeds.push_back(static_cast<unsigned>(std::stoul(t)));
  if (steer == "actual")
    o.steer = cvc::physics::steer_input::actual;
  else if (steer == "command")
    o.steer = cvc::physics::steer_input::command;
  else
    throw std::invalid_argument("--steer is actual or command");
  o.resolve();
  o.report = stdout;
  const cvc::nav::coef_mlp net =
      policy.empty() ? cvc::nav::coef_mlp::default_biased() : cvc::nav::coef_mlp::load(policy);
  cvc::physics::write_plant_log(o, cvc::physics::drive_plant_log_scene(app, o, net));
  return 0;
}

// ── trace: an input trace (cvc.input_trace/v1), what it holds and whether it replays ──

static int cmd_trace(cvc::app &app, int argc, char **argv) {
  po::options_description desc(
      "cvc trace - an input trace (cvc.input_trace/v1, docs/INPUT_TRACE.md)\n\n"
      "Operations:\n"
      "  info <file>      what it holds: the world, the step, the span, the agents and their\n"
      "                   policies, and the inputs by key and verb\n"
      "  verify <file>    replay it headlessly (a kinematic cvc.nav.sim_world/v1 world) and\n"
      "                   compare every state hash it holds; exit 1 when the replay departs\n\n"
      "  cvc trace verify session.trace\n"
      "  cvc trace verify session.trace --policy 1=other.cvcnav   # another build of policy 1\n");
  std::string op, file;
  std::vector<std::string> policies;
  int threads = 0;
  desc.add_options()("help,h", "show help")("op", po::value<std::string>(&op)->required(),
                                            "info or verify")(
      "file", po::value<std::string>(&file)->required(), "the trace")(
      "policy", po::value<std::vector<std::string>>(&policies)->composing(),
      "verify: INDEX=FILE, a .cvcnav in place of the recorded policy INDEX (repeatable)")(
      "threads", po::value<int>(&threads)->default_value(0),
      "verify: threads a step runs on (0: the trace's)");
  po::positional_options_description pos;
  pos.add("op", 1).add("file", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  const cvc::input_trace t = cvc::input_trace::read(file);
  if (op == "info") {
    std::cout << cvc::describe(t);
    return 0;
  }
  if (op != "verify")
    throw std::runtime_error("Unknown trace operation: " + op + " (use info or verify)");
  cvc::nav::replay_options o;
  o.num_threads = threads;
  for (const std::string &p : policies) {
    const std::size_t eq = p.find('=');
    if (eq == std::string::npos || eq == 0 || eq + 1 == p.size())
      throw std::invalid_argument("--policy is INDEX=FILE, not " + p);
    o.policy_files[std::stoi(p.substr(0, eq))] = p.substr(eq + 1);
  }
  const cvc::nav::replay_result r = cvc::nav::replay_trace(app, t, o);
  std::cout << cvc::nav::describe(r);
  return r.matched ? 0 : 1;
}

// ── mujoco: a MuJoCo robot (cvc::physics::mujoco_robot) on libcvc terrain ──

static int cmd_mujoco(cvc::app &, int argc, char **argv) {
  po::options_description desc(
      "cvc mujoco - a MuJoCo robot on a libcvc height field (docs/MUJOCO.md); needs a libcvc\n"
      "built with CVC_WITH_MUJOCO\n\n"
      "Operations:\n"
      "  info <model>     the model: MuJoCo's version, sizes, mass, step, actuators, bodies\n"
      "  stand <model>    a standing trial on a plane, settled under the servos, then pushed on\n"
      "                   its base: prints the result and the energy books as JSON\n\n"
      "  cvc mujoco info  <prefix>/share/mujoco-menagerie/unitree_go2/scene.xml\n"
      "  cvc mujoco stand <...>/unitree_go2/scene.xml --slope 10 --push 0,60,0\n");
  std::string op, model, push;
  cvc::physics::mujoco_trial_options t;
  // clang-format off
  desc.add_options()
    ("help,h", "show help")
    ("op", po::value<std::string>(&op)->required(), "info or stand")
    ("model", po::value<std::string>(&model)->required(), "the MJCF file")
    ("slope", po::value<double>(&t.slope_deg)->default_value(0.0), "stand: the plane's slope (degrees, rising along +x)")
    ("yaw", po::value<double>(&t.yaw)->default_value(0.0), "stand: the robot's heading (rad; 0 faces uphill)")
    ("push", po::value<std::string>(&push)->default_value("0,0,0"), "stand: FX,FY,FZ on the base (N, world axes)")
    ("push-at", po::value<double>(&t.settle)->default_value(1.0), "stand: when the push starts (s)")
    ("push-for", po::value<double>(&t.push_duration)->default_value(0.1), "stand: how long it lasts (s)")
    ("duration", po::value<double>(&t.duration)->default_value(4.0), "stand: the trial's length (s)")
    ("kp", po::value<double>(&t.kp)->default_value(60.0), "stand: the servos' stiffness (N m / rad)")
    ("kd", po::value<double>(&t.kd)->default_value(2.0), "stand: their damping (N m s / rad)")
    ("timestep", po::value<double>(&t.timestep)->default_value(0.0), "stand: the step (s; 0: the model's)");
  // clang-format on
  po::positional_options_description pos;
  pos.add("op", 1).add("model", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);
  if (!cvc::physics::mujoco_robot::available())
    throw std::runtime_error("this libcvc was built without MuJoCo (CVC_WITH_MUJOCO)");
  if (op == "info") {
    cvc::physics::mujoco_robot_options o;
    o.model = model;
    o.keyframe.clear(); // the model as written
    std::cout << cvc::physics::describe(cvc::physics::mujoco_robot(o));
    return 0;
  }
  if (op != "stand")
    throw std::runtime_error("Unknown mujoco operation: " + op + " (use info or stand)");
  double f[3];
  char tail = 0;
  if (std::sscanf(push.c_str(), "%lf,%lf,%lf%c", &f[0], &f[1], &f[2], &tail) != 3)
    throw std::invalid_argument("--push is FX,FY,FZ, not " + push);
  t.model = model;
  t.push = {f[0], f[1], f[2]};
  std::cout << cvc::physics::to_json(cvc::physics::run_mujoco_trial(t));
  return 0;
}

// ── ph: composed port-Hamiltonian systems (cvc::ph composition) ──

static std::vector<double> ph_numbers(const std::string &s, const char *what) {
  std::vector<double> out;
  std::stringstream in(s);
  std::string item;
  while (std::getline(in, item, ',')) {
    if (item.find_first_not_of(" \t") == std::string::npos)
      continue;
    try {
      out.push_back(std::stod(item));
    } catch (const std::exception &) {
      throw std::invalid_argument(std::string("--") + what + ": '" + item + "' is not a number");
    }
  }
  return out;
}

static int cmd_ph(cvc::app &, int argc, char **argv) {
  po::options_description desc(
      "cvc ph - composed port-Hamiltonian systems (cvc-ph-system/1, docs/PH_COMPOSE.md)\n\n"
      "Operations:\n"
      "  info <system>       the blocks, their state, ports and inputs, and the pairing\n"
      "  simulate <system>   roll it out from --x0 under --u held (or one row per step) and\n"
      "                      print the energy books; --csv writes every state\n"
      "  check <system>      the power balance at random states: exit 1 when\n"
      "                      the pairing does work or dH/dt departs from -loss + supplied\n"
      "  microgrid <out>     write a DC microgrid (DGUs and RL lines, docs/PH_COMPOSE.md) as a\n"
      "                      system file\n"
      "  train-dgu <out>     fit a DGU law to trajectories of the reference nonlinear DGU (the\n"
      "                      C-PHAST electrical calibration), score it on held-out rollouts\n"
      "                      against the fitted-linear control, and write it as cvc-ph-law/1\n\n"
      "  cvc ph microgrid grid.json --dgus 4 --edges 0-1,1-2,2-3 --line-r 0.3,0.4,0.5\n"
      "  cvc ph simulate grid.json --physical 1,0,1.1,0,0.9,0,1,0,0,0,0 --u "
      "-0.1,1,-0.1,1,-0.1,1,-0.1,1\n");
  std::string op, file, x0s, physical, us, csv, coupling, scheme, edges, line_r, line_l, dgu_kind;
  std::string v_hidden, m_hidden, d_hidden;
  int steps = 100, substeps = 2, local = 1, samples = 100, max_iter = 10, dgus = 4;
  int updates = 2000, constraint_first = 0, batch = 128, threads = 1, train_runs = 32,
      test_runs = 16;
  int transitions = 200, test_transitions = 500, log_every = 0;
  unsigned data_seed = 1, init_seed = 7;
  double lr = 1e-4;
  unsigned seed = 1;
  double dt = 0.01, tol = 1e-6, scale = 1.0, dgu_c = 2.2, dgu_l = 1.8, dgu_r = 1.2;
  // clang-format off
  desc.add_options()
    ("help,h", "show help")
    ("op", po::value<std::string>(&op)->required(), "info, simulate, check or microgrid")
    ("file", po::value<std::string>(&file)->required(), "the system file (microgrid: the one to write)")
    ("x0", po::value<std::string>(&x0s), "simulate: the start state, comma separated (default zeros)")
    ("physical", po::value<std::string>(&physical), "simulate: the start as every block's efforts instead (a DGU's v and i, a line's i), mapped to its state")
    ("u", po::value<std::string>(&us), "simulate: the inputs, one row held or one row per step (default zeros)")
    ("steps", po::value<int>(&steps)->default_value(100), "simulate: steps")
    ("dt", po::value<double>(&dt)->default_value(0.01), "simulate: step (s)")
    ("scheme", po::value<std::string>(&scheme)->default_value("split"), "simulate: split, rk4 or implicit_midpoint")
    ("coupling", po::value<std::string>(&coupling)->default_value("midpoint"), "simulate: the split step's coupling, euler or midpoint")
    ("substeps", po::value<int>(&substeps)->default_value(2), "simulate: the split step's substeps")
    ("local-substeps", po::value<int>(&local)->default_value(1), "simulate: each block's own substeps")
    ("max-iter", po::value<int>(&max_iter)->default_value(10), "simulate: Picard iterations at most")
    ("tol", po::value<double>(&tol)->default_value(1e-6), "simulate: Picard tolerance")
    ("csv", po::value<std::string>(&csv), "simulate: write t, the state, H and the books per step")
    ("samples", po::value<int>(&samples)->default_value(100), "check: random states")
    ("seed", po::value<unsigned>(&seed)->default_value(1), "check: their seed")
    ("scale", po::value<double>(&scale)->default_value(1.0), "check: their standard deviation")
    ("updates", po::value<int>(&updates)->default_value(2000), "train-dgu: combined-loss updates (the paper: 100000, or 75000 after the constraint-first ones)")
    ("constraint-first", po::value<int>(&constraint_first)->default_value(0), "train-dgu: constraint-only updates before them (the paper's staged schedule: 25000)")
    ("batch", po::value<int>(&batch)->default_value(128), "train-dgu: samples per update")
    ("lr", po::value<double>(&lr)->default_value(1e-4), "train-dgu: Adam's step")
    ("threads", po::value<int>(&threads)->default_value(1), "train-dgu: threads for the gradient")
    ("train-runs", po::value<int>(&train_runs)->default_value(32), "train-dgu: training trajectories")
    ("transitions", po::value<int>(&transitions)->default_value(200), "train-dgu: transitions per training trajectory")
    ("test-runs", po::value<int>(&test_runs)->default_value(16), "train-dgu: held-out trajectories")
    ("test-transitions", po::value<int>(&test_transitions)->default_value(500), "train-dgu: transitions per held-out trajectory (5 s at 0.01 s)")
    ("data-seed", po::value<unsigned>(&data_seed)->default_value(1), "train-dgu: the data's seed (held-out: data-seed + 1)")
    ("init-seed", po::value<unsigned>(&init_seed)->default_value(7), "train-dgu: the nets' and the batches' seed")
    ("v-hidden", po::value<std::string>(&v_hidden)->default_value("32,32"), "train-dgu: the potential net's hidden widths")
    ("m-hidden", po::value<std::string>(&m_hidden)->default_value("32,32"), "train-dgu: the mass net's hidden widths")
    ("d-hidden", po::value<std::string>(&d_hidden)->default_value("32"), "train-dgu: the damping net's hidden widths")
    ("log-every", po::value<int>(&log_every)->default_value(0), "train-dgu: print the batch losses every this many updates")
    ("dgus", po::value<int>(&dgus)->default_value(4), "microgrid: buses, one DGU each")
    ("edges", po::value<std::string>(&edges), "microgrid: lines as FROM-TO, comma separated (default a chain)")
    ("line-r", po::value<std::string>(&line_r)->default_value("0.3,0.4,0.5"), "microgrid: line resistances (ohm), one or one per line (the last repeats)")
    ("line-l", po::value<std::string>(&line_l)->default_value("0.8"), "microgrid: line inductances (H), likewise")
    ("dgu", po::value<std::string>(&dgu_kind)->default_value("reference"), "microgrid: reference (q = 2.2 (v + v^3/3), 1.8 H, 1.2 ohm) or linear")
    ("dgu-c", po::value<double>(&dgu_c)->default_value(2.2), "microgrid: a linear DGU's capacitance (F)")
    ("dgu-l", po::value<double>(&dgu_l)->default_value(1.8), "microgrid: a linear DGU's inductance (H)")
    ("dgu-r", po::value<double>(&dgu_r)->default_value(1.2), "microgrid: a linear DGU's resistance (ohm)");
  // clang-format on
  po::positional_options_description pos;
  pos.add("op", 1).add("file", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);

  if (op == "train-dgu") {
    auto widths = [](const std::string &s, const char *what) {
      std::vector<int> w;
      for (double v : ph_numbers(s, what))
        w.push_back(static_cast<int>(v));
      return w;
    };
    const cvc::ph::law_ptr ref = cvc::ph::reference_dgu();
    cvc::ph::dgu_dataset_options d;
    d.trajectories = train_runs;
    d.transitions = transitions;
    d.seed = data_seed;
    const cvc::ph::dgu_dataset train = cvc::ph::make_dgu_dataset(*ref, d);
    d.trajectories = test_runs;
    d.transitions = test_transitions;
    d.seed = data_seed + 1;
    const cvc::ph::dgu_dataset test = cvc::ph::make_dgu_dataset(*ref, d);
    const cvc::ph::linear_fit fit = cvc::ph::fit_linear_dgu(train);
    cvc::ph::dgu_family fam;
    fam.v_hidden = widths(v_hidden, "v-hidden");
    fam.m_hidden = widths(m_hidden, "m-hidden");
    fam.d_hidden = widths(d_hidden, "d-hidden");
    fam.seed = init_seed;
    const cvc::ph::canonical_spec init = cvc::ph::warm_start_dgu(fit, fam);
    cvc::ph::dgu_train_options o;
    o.updates = updates;
    o.constraint_first = constraint_first;
    o.batch = batch;
    o.lr = lr;
    o.threads = threads;
    o.log_every = log_every;
    o.seed = init_seed + 1000;
    std::cout << std::setprecision(4) << "train-dgu: " << train.samples.size()
              << " training samples (" << train_runs << " x " << transitions << "), " << test_runs
              << " held-out runs of " << test_transitions << "\n  linear fit: C " << fit.C
              << " F, L " << fit.L << " H, R " << fit.R << " ohm\n  "
              << cvc::ph::trainable_parameters(init).size() << " parameters, " << constraint_first
              << " constraint-first + " << updates << " combined updates\n";
    const auto t0 = std::chrono::steady_clock::now();
    const cvc::ph::dgu_train_result r = cvc::ph::train_dgu(init, train, o);
    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    for (const cvc::ph::dgu_train_record &h : r.history)
      std::cout << "  update " << h.update << ": step " << h.batch.step << ", constraint "
                << h.batch.constraint << ", total " << h.batch.total << "\n";
    const cvc::ph::canonical_law trained(r.law), linear(cvc::ph::linear_dgu_spec(fit));
    const cvc::ph::dgu_eval el = cvc::ph::evaluate_dgu(linear, *ref, test),
                            et = cvc::ph::evaluate_dgu(trained, *ref, test);
    std::cout << "  trained in " << secs
              << " s\n  held-out rollouts, capacitor voltage / inductor current RMSE:\n"
              << "    fitted linear: " << 1e3 * el.voltage_rmse << " mV / " << 1e3 * el.current_rmse
              << " mA\n"
              << "    trained:       " << 1e3 * et.voltage_rmse << " mV / " << 1e3 * et.current_rmse
              << " mA" << (et.finite ? "" : " (a rollout left the reals)") << "\n";
    std::ofstream out(file);
    if (!out)
      throw std::runtime_error("cannot write " + file);
    out << cvc::ph::law_json(trained) << "\n";
    std::cout << "  wrote " << file << "\n";
    return et.finite ? 0 : 1;
  }
  if (op == "microgrid") {
    std::vector<cvc::ph::line_spec> lines;
    if (edges.empty())
      for (int i = 0; i + 1 < dgus; ++i)
        lines.push_back({i, i + 1, 0.0, 1.0});
    else {
      std::stringstream in(edges);
      std::string item;
      while (std::getline(in, item, ',')) {
        const size_t dash = item.find('-');
        if (dash == std::string::npos)
          throw std::invalid_argument("--edges: '" + item + "' is not FROM-TO");
        lines.push_back(
            {std::stoi(item.substr(0, dash)), std::stoi(item.substr(dash + 1)), 0.0, 1.0});
      }
    }
    const std::vector<double> rs = ph_numbers(line_r, "line-r"), ls = ph_numbers(line_l, "line-l");
    if (rs.empty() || ls.empty())
      throw std::invalid_argument("--line-r and --line-l need at least one value");
    for (size_t k = 0; k < lines.size(); ++k) {
      lines[k].R = rs[std::min(k, rs.size() - 1)];
      lines[k].L = ls[std::min(k, ls.size() - 1)];
    }
    cvc::ph::law_ptr dgu;
    if (dgu_kind == "reference")
      dgu = cvc::ph::reference_dgu();
    else if (dgu_kind == "linear")
      dgu = cvc::ph::linear_dgu(dgu_c, dgu_l, dgu_r);
    else
      throw std::invalid_argument("--dgu: '" + dgu_kind + "' (use reference or linear)");
    const cvc::ph::microgrid g = cvc::ph::make_microgrid(dgu, dgus, lines);
    std::ofstream out(file);
    if (!out)
      throw std::runtime_error("cannot write " + file);
    out << cvc::ph::system_json(g.system) << "\n";
    std::cout << cvc::ph::describe(g.system) << "  wrote " << file << "\n";
    return 0;
  }
  const cvc::ph::composite s = cvc::ph::load_system(file);
  if (op == "info") {
    std::cout << cvc::ph::describe(s);
    return 0;
  }
  if (op == "check") {
    const cvc::ph::balance_summary b = cvc::ph::check_balance_random(s, samples, seed, scale);
    std::cout << std::setprecision(3) << "ph check: " << b.samples << " states\n"
              << "  dH/dt against -loss + supplied: " << b.worst.rate_error << " (relative)\n"
              << "  the pairing's net power:        " << b.worst.internal_error << " (relative)\n"
              << "  max |C + C^T|:                  " << b.worst.skew_error << "\n";
    const bool ok =
        b.worst.rate_error < 1e-10 && b.worst.internal_error < 1e-12 && b.worst.skew_error == 0;
    std::cout << (ok ? "  balanced\n" : "  NOT balanced\n");
    return ok ? 0 : 1;
  }
  if (op != "simulate")
    throw std::runtime_error("Unknown ph operation: " + op +
                             " (use info, simulate, check or microgrid)");
  cvc::ph::transition_opts o;
  o.dt = dt;
  o.substeps = substeps;
  o.local_substeps = local;
  o.coupling = cvc::ph::coupling_scheme_of(coupling);
  o.scheme = cvc::ph::step_scheme_of(scheme);
  o.solve.max_iter = max_iter;
  o.solve.tol = tol;
  std::vector<double> x0 = ph_numbers(x0s, "x0"), u = ph_numbers(us, "u");
  if (!physical.empty()) {
    if (!x0.empty())
      throw std::invalid_argument("give --x0 or --physical, not both");
    const cvc::ph::storage_result st =
        cvc::ph::storage_from_efforts(s, ph_numbers(physical, "physical"));
    if (!st.converged)
      throw std::runtime_error("--physical: no state reaches those efforts (residual " +
                               std::to_string(st.residual) + " after " +
                               std::to_string(st.iterations) + " iterations)");
    std::cout << "start from efforts: " << st.iterations << " Newton iterations, residual "
              << st.residual << "\n";
    x0 = st.x;
  }
  if (x0.empty())
    x0.assign(s.state_dim(), 0.0);
  if (u.empty())
    u.assign(s.input_dim(), 0.0);
  const cvc::ph::rollout_result r = cvc::ph::rollout(s, o, x0, u, steps);
  std::cout << cvc::ph::describe(s, o, r);
  if (!csv.empty()) {
    std::ofstream out(csv);
    if (!out)
      throw std::runtime_error("cannot write " + csv);
    cvc::ph::write_csv(out, s, o, r);
    std::cout << "  wrote " << csv << "\n";
  }
  return 0;
}

// ── nav: cvc::nav's lab (docs/NAV_LAB.md): a run log's Stats ──

// A run log's arms, legs and Stats rows at its last tick, as the Stats window shows them.
static void print_run_log(const cvc::nav::run_log &log, std::ostream &out) {
  const cvc::nav::run_log_header &h = log.header();
  out << "run log " << cvc::nav::kRunLogSchema << " (libcvc " << h.libcvc << "): " << log.arms()
      << (log.arms() == 1 ? " arm, " : " arms, ") << h.agents
      << (h.agents == 1 ? " agent each, " : " agents each, ") << log.ticks() << " ticks of " << h.dt
      << " s, " << log.legs().size() << (log.legs().size() == 1 ? " leg" : " legs")
      << (log.finished() ? "" : ", not finished") << "\n";
  out << "  scene " << (h.scene.empty() ? "-" : h.scene) << " on "
      << (h.plant.empty() ? "kinematic" : h.plant) << ", seed " << h.seed << "\n";
  for (const cvc::nav::run_log_arm &a : h.arms)
    out << "  arm " << a.slot << " " << a.name << ": "
        << (a.uri.empty() ? std::string("the built-in policy") : a.uri)
        << (a.sha256.empty() ? std::string() : " (sha256 " + a.sha256.substr(0, 16) + ")") << "\n";
  const std::vector<cvc::nav::panel_table> t = cvc::nav::arms_tables(log, log.ticks() - 1, 0);
  const cvc::nav::panel_table &stats = t.front();
  out << "  Stats, the last leg:\n   ";
  for (const std::string &c : stats.columns)
    out << " " << std::setw(c == "arm" ? 14 : 10) << c;
  out << "\n";
  for (const std::vector<std::string> &r : stats.rows) {
    out << "   ";
    for (std::size_t c = 0; c < r.size(); ++c)
      out << " " << std::setw(stats.columns[c] == "arm" ? 14 : 10) << r[c];
    out << "\n";
  }
  for (const std::string &n : log.notes())
    out << "  note: " << n << "\n";
}

static int cmd_nav(cvc::app &, int argc, char **argv) {
  po::options_description desc(
      "cvc nav - cvc::nav's lab: policies side by side, and watching one learn "
      "(docs/NAV_LAB.md)\n\n"
      "Operations:\n"
      "  log <file>        a run log (cvc.nav.run_log/v1): its arms, legs and Stats rows\n"
      "  scrub <history>   deploy every checkpoint a training history names on one scenario\n"
      "                    (cached at <history>.scrub/), and print each one's metrics\n\n"
      "  cvc nav log run.cvcrun\n"
      "  cvc nav scrub run/history.jsonl --agents 8 --ticks 900 --report scrub.json\n"
      "  cvc nav scrub run/history.jsonl --plant kinematic\n");
  std::string op, file, scenario, report, plant = cvc::physics::scrub_default_plant();
  int run = -1, agents = 4;
  unsigned seed = 0;
  long ticks = 600;
  bool no_cache = false;
  desc.add_options()("help,h", "show help")("op", po::value<std::string>(&op)->required(),
                                            "log or scrub")(
      "file", po::value<std::string>(&file)->required(),
      "the run log, or the training history")("run", po::value<int>(&run)->default_value(-1),
                                              "scrub: which run of the history (-1: the last)")(
      "scenario", po::value<std::string>(&scenario),
      "scrub: an input trace whose world and agents to deploy on (default: the built-in blocks)")(
      "agents", po::value<int>(&agents)->default_value(4), "scrub: the built-in scenario's agents")(
      "seed", po::value<unsigned>(&seed)->default_value(0), "scrub: the built-in scenario's seed")(
      "ticks", po::value<long>(&ticks)->default_value(600), "scrub: each checkpoint's ticks")(
      "plant", po::value<std::string>(&plant)->default_value(plant),
      "scrub: what the agents drive: humvee (Jolt, colliding with the walls; the default with "
      "Jolt), roboracer, dynamic:humvee, ..., or kinematic (nav's bicycle, which drives through "
      "walls)")("report", po::value<std::string>(&report),
                "scrub: write every checkpoint's metrics here (cvc.nav.scrub/v1)")(
      "no-cache", po::bool_switch(&no_cache),
      "scrub: deploy every checkpoint again, cache nothing");
  po::positional_options_description pos;
  pos.add("op", 1).add("file", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);
  if (op == "log") {
    print_run_log(cvc::nav::run_log::read(file), std::cout);
    return 0;
  }
  if (op != "scrub")
    throw std::runtime_error("Unknown nav operation: " + op + " (use log or scrub)");
  const cvc::nav::training_history h = cvc::nav::read_training_history(file, run);
  for (const std::string &m : h.missing)
    std::cerr << "cvc nav scrub: not there: " << m << "\n";
  for (const std::string &s : h.skipped)
    std::cerr << "cvc nav scrub: not a .cvcnav, skipped: " << s << "\n";
  if (h.checkpoints.empty()) {
    std::cerr << "cvc nav scrub: " << file << " names no checkpoint (a .cvcnav) that is there\n";
    return 2;
  }
  cvc::nav::scrub_scenario s = scenario.empty() ? cvc::nav::default_scenario(agents, seed, ticks)
                                                : cvc::nav::scenario_from_trace(scenario, ticks);
  cvc::physics::plant_host::check(plant);
  s.plant = plant;
  cvc::nav::scrub_options o;
  o.use_cache = !no_cache;
  o.say = [](const std::string &line) { std::cout << line << "\n"; };
  o.plant = cvc::physics::scrub_plant();
  std::cout << "cvc nav scrub: " << h.checkpoints.size() << " checkpoints of " << file << " (run "
            << h.run + 1 << " of " << h.runs << ") on " << s.name << ", on " << s.plant << ", "
            << s.ticks << " ticks (scenario " << cvc::nav::scenario_hash(s) << ")\n";
  const std::vector<cvc::nav::scrub_entry> e = cvc::nav::scrub(h, s, o);
  const cvc::nav::panel_table t = cvc::nav::scrub_table(e, -1);
  for (const std::string &c : t.columns)
    std::cout << " " << std::setw(c == "at" ? 2 : 11) << (c == "at" ? "" : c);
  std::cout << "\n";
  for (const std::vector<std::string> &r : t.rows) {
    for (std::size_t c = 0; c < r.size(); ++c)
      std::cout << " " << std::setw(t.columns[c] == "at" ? 2 : 11) << r[c];
    std::cout << "\n";
  }
  if (!report.empty()) {
    std::ofstream out(report);
    out << cvc::nav::scrub_report_json(h, s, e) << "\n";
    if (!out)
      throw std::runtime_error("cannot write " + report);
    std::cout << "cvc nav scrub: the report is " << report << "\n";
  }
  return 0;
}

// ── ph-train: fit the learned port-Hamiltonian vehicle (cvc::ph) to plant logs ──

static int cmd_ph_train(cvc::app &, int argc, char **argv) {
  po::options_description desc(
      "cvc ph-train - fit cvc::ph's learned SE(2) port-Hamiltonian vehicle to cvc-plant-log runs\n"
      "(LieSPHGP's SE(2) recipe with its rollout loss; no Python)");
  // clang-format off
  desc.add_options()
    ("help,h", "show help")
    ("runs", po::value<std::vector<std::string>>()->multitoken()->required(),
     "cvc-plant-log --out directories (flights.npy + meta.json)")
    ("out,o", po::value<std::string>()->required(),
     "the fitted model (JSON, cvc-ph-se2/1; /2 with --steer-modulates)")
    ("steer-modulates",
     "steering (the runs' last input) modulates D and G instead of being a port, so only the "
     "drive supplies power; fit it to runs logged with --steer actual")
    ("inputs", po::value<std::string>(),
     "the inputs, comma-separated column names of the runs' layout (meta.json), ports first and "
     "the steering last (e.g. drive,steer or rim_front,rim_rear,steer from plant-log --wide); "
     "default: every input column")
    ("potential", po::value<std::string>()->default_value("world"),
     "world: V(x, y), learned on world position; none: no potential; terrain: the ground's logged "
     "slope under the wheels (ground_grade, ground_roll; else the body's grade, roll) times two "
     "learned coefficients (needs plant-log --wide runs)")
    ("eval-runs", po::value<std::vector<std::string>>()->multitoken(),
     "after training, report the mean model's rollout loss on these runs' windows (every seed, "
     "no noise): another environment's, held out")
    ("test-seeds", po::value<std::vector<int>>()->multitoken()->default_value({101}, "101"),
     "seeds held out: monitored, never trained on")
    ("noise", po::value<double>()->default_value(0.05, "0.05"), "observation noise sigma added to x, y, psi, v")
    ("noise-seed", po::value<std::uint64_t>()->default_value(0), "the noise's random seed")
    ("seed", po::value<std::uint64_t>()->default_value(0), "features, initial values, batches, draws")
    ("steps", po::value<int>()->default_value(5000), "optimizer steps")
    ("batch", po::value<int>()->default_value(16), "windows per step")
    ("window-points", po::value<int>()->default_value(101), "samples per training window")
    ("window-stride", po::value<int>()->default_value(75), "samples between window starts")
    ("substeps", po::value<int>()->default_value(1), "Lie-IMEX steps per sample interval")
    ("features", po::value<int>()->default_value(64), "random features per GP")
    ("no-v-gp", "V(x, y) linear only (no GP)")
    ("v-length", po::value<double>()->default_value(50.0), "V's GP length scale (m)")
    ("lr", po::value<double>()->default_value(3e-3), "Adam's learning rate (cosine-decayed)")
    ("level-lr", po::value<double>()->default_value(0.25), "Prodigy's schedule on the levels")
    ("final-lr-fraction", po::value<double>()->default_value(0.1), "the decay's floor")
    ("clip", po::value<double>()->default_value(1.0), "global gradient-norm clip (0: none)")
    ("kl-beta-max", po::value<double>()->default_value(1.0), "the KL weight's final value")
    ("kl-anneal", po::value<int>()->default_value(1000), "steps for the KL weight to rise")
    ("eval-every", po::value<int>()->default_value(500), "steps between monitor evaluations")
    ("eval-windows", po::value<int>()->default_value(64), "held-out windows the monitor reads")
    ("threads", po::value<int>()->default_value(0), "worker threads (0: all cores)")
    ("init", po::value<std::string>(), "continue from this model instead of a fresh one")
    ("dump-dataset", po::value<std::string>(),
     "also write the noisy train/test flights (npy) and their statistics to this directory")
    ("log-every", po::value<int>()->default_value(50), "steps between log lines");
  // clang-format on
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);
  namespace ph = cvc::ph;
  const auto runs = vm["runs"].as<std::vector<std::string>>();
  const auto test_seeds = vm["test-seeds"].as<std::vector<int>>();
  const double noise = vm["noise"].as<double>();
  const std::uint64_t noise_seed = vm["noise-seed"].as<std::uint64_t>();

  const std::string potential = vm["potential"].as<std::string>();
  if (potential != "world" && potential != "none" && potential != "terrain")
    throw std::invalid_argument("--potential is world, none or terrain");
  // The inputs: the named columns (and the attitude's, last, for the terrain potential).
  std::vector<std::string> inputs;
  if (vm.count("inputs")) {
    std::stringstream in(vm["inputs"].as<std::string>());
    for (std::string t; std::getline(in, t, ',');)
      if (!t.empty())
        inputs.push_back(t);
  }
  const auto pick = [&](const ph::se2_flights &f) {
    if (inputs.empty() && potential != "terrain")
      return f;
    std::vector<std::string> cols = inputs;
    if (cols.empty()) {
      if (f.columns.size() != static_cast<std::size_t>(f.width))
        throw std::runtime_error("--potential terrain needs runs whose meta.json names columns");
      cols.assign(f.columns.begin() + 6, f.columns.end());
    }
    if (potential == "terrain") {
      // The ground's slope under the wheels where the runs have it; else the body's attitude,
      // which also carries the suspension's pitch and roll.
      const bool ground =
          std::find(f.columns.begin(), f.columns.end(), "ground_grade") != f.columns.end();
      cols.push_back(ground ? "ground_grade" : "grade");
      cols.push_back(ground ? "ground_roll" : "roll");
    }
    return ph::select_inputs(f, cols);
  };
  const ph::se2_flights all = pick(ph::load_plant_log_runs(runs));
  const int terrain_dim = potential == "terrain" ? 2 : 0;
  const ph::se2_flights noisy =
      noise > 0.0 ? ph::add_observation_noise(all, noise, noise_seed) : all;
  const ph::se2_flights train_f = ph::select_seeds(noisy, test_seeds, false);
  const ph::se2_flights test_f = ph::select_seeds(noisy, test_seeds, true);
  if (train_f.count == 0)
    throw std::runtime_error("every flight is held out");
  const std::array<double, 3> twist = ph::twist_rms(train_f);
  std::vector<double> control = ph::control_rms(train_f);
  // An input that never moved in training (the ground's slope on flat ground, round-off of order
  // 1e-8) keeps a unit scale: dividing by an RMS of about zero would blow up every run where it
  // does move.
  for (double &c : control)
    if (!(c > 1e-6))
      c = 1.0;
  if (vm.count("dump-dataset")) {
    const fs::path dir(vm["dump-dataset"].as<std::string>());
    fs::create_directories(dir);
    const auto shape = [](const ph::se2_flights &f) {
      return std::vector<std::size_t>{f.count, f.rows, static_cast<std::size_t>(f.width)};
    };
    ph::write_npy((dir / "train.npy").string(), train_f.data, shape(train_f));
    ph::write_npy((dir / "test_noisy.npy").string(), test_f.data, shape(test_f));
    const ph::se2_flights clean = ph::select_seeds(all, test_seeds, true);
    ph::write_npy((dir / "test.npy").string(), clean.data, shape(clean));
    std::ofstream js((dir / "dataset.json").string());
    js << std::setprecision(17) << "{\"dt\": " << all.dt << ", \"noise\": " << noise
       << ", \"noise_seed\": " << noise_seed << ", \"train_flights\": " << train_f.count
       << ", \"test_flights\": " << test_f.count << "}\n";
  }

  const int points = vm["window-points"].as<int>();
  const int stride = vm["window-stride"].as<int>();
  ph::se2_model model;
  if (vm.count("init")) {
    // A model continues with its own scales: the inputs are divided by the RMS it was fit with.
    model = ph::se2_model::load(vm["init"].as<std::string>());
    if (model.setup.input_dim() != all.control_dim())
      throw std::runtime_error("--init's model has " + std::to_string(model.setup.input_dim()) +
                               " inputs; the runs have " + std::to_string(all.control_dim()));
    if ((model.setup.modulation_dim > 0) != (vm.count("steer-modulates") > 0))
      throw std::runtime_error("--init's model and --steer-modulates disagree on the steering");
    if (model.setup.terrain_dim() != terrain_dim)
      throw std::runtime_error("--init's model and --potential disagree");
  } else {
    ph::se2_init init;
    const int inputs_dim = all.control_dim() - terrain_dim; // the ports and the steering
    init.control_dim = inputs_dim;
    if (vm.count("steer-modulates")) {
      if (inputs_dim < 2)
        throw std::runtime_error("--steer-modulates needs a drive and a steering input");
      init.control_dim = inputs_dim - 1;
      init.modulation_dim = 1;
    }
    init.potential = potential == "none" ? ph::se2_potential::none
                                         : (potential == "terrain" ? ph::se2_potential::terrain
                                                                   : ph::se2_potential::world);
    init.features = vm["features"].as<int>();
    init.v_gp = !vm.count("no-v-gp");
    init.v_length = vm["v-length"].as<double>();
    model = ph::se2_model::initialize(init, vm["seed"].as<std::uint64_t>(), twist, control, all.dt,
                                      vm["substeps"].as<int>());
  }
  const ph::se2_windows train =
      ph::cut_windows(ph::scale_controls(train_f, model.control_scale), points, stride);
  const ph::se2_windows test =
      test_f.count
          ? ph::cut_windows(ph::scale_controls(test_f, model.control_scale), points, stride)
          : ph::se2_windows{};
  ph::se2_train_config cfg;
  cfg.seed = vm["seed"].as<std::uint64_t>();
  cfg.total_steps = vm["steps"].as<int>();
  cfg.batch_size = vm["batch"].as<int>();
  cfg.kl_beta_max = vm["kl-beta-max"].as<double>();
  cfg.kl_anneal_steps = vm["kl-anneal"].as<int>();
  cfg.eval_every = vm["eval-every"].as<int>();
  cfg.eval_windows = vm["eval-windows"].as<int>();
  cfg.threads = vm["threads"].as<int>();
  cfg.optimizer.learning_rate = vm["lr"].as<double>();
  cfg.optimizer.level_learning_rate = vm["level-lr"].as<double>();
  cfg.optimizer.final_learning_fraction = vm["final-lr-fraction"].as<double>();
  cfg.optimizer.gradient_clip_norm = vm["clip"].as<double>();
  const int log_every = std::max(1, vm["log-every"].as<int>());

  std::cout << std::setprecision(6) << "ph-train: " << all.count << " flights x " << all.rows
            << " rows at dt " << all.dt << " s; " << train_f.count << " train / " << test_f.count
            << " held out; windows " << train.count << " / " << test.count << " of " << points
            << " points\n  twist scale " << model.twist_scale[0] << " " << model.twist_scale[1]
            << " " << model.twist_scale[2] << "; control scale";
  for (double c : model.control_scale)
    std::cout << " " << c;
  std::cout << "; parameters " << model.params.size() << "\n";
  const auto started = std::chrono::steady_clock::now();
  const std::size_t batch = std::min<std::size_t>(std::max(cfg.batch_size, 1), train.count);
  std::size_t dropped = 0; // training windows whose rollout left the finite numbers
  ph::se2_train_rollout(
      model, train, test.count ? &test : nullptr, cfg, [&](const ph::se2_train_event &ev) {
        if (ev.objective.healthy < batch) {
          dropped += batch - ev.objective.healthy;
          std::cout << "step " << ev.step << ": " << batch - ev.objective.healthy
                    << " window(s) diverged; left out of the loss and the gradient\n";
        }
        if (ev.step % log_every && ev.step != 1 && !ev.evaluated)
          return;
        const double s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        std::cout << "step " << std::setw(6) << ev.step << " loss " << ev.objective.value
                  << " rollout " << ev.objective.rollout << " kl " << ev.objective.kl << " beta "
                  << ev.beta << " |g| " << ev.gradient_norm << " (" << s << " s)";
        if (ev.evaluated)
          std::cout << "  MONITOR test " << ev.test_loss << " train subset " << ev.train_loss
                    << " prodigy d " << ev.prodigy_d[0] << " " << ev.prodigy_d[1] << " "
                    << ev.prodigy_d[2] << " " << ev.prodigy_d[3];
        std::cout << "\n" << std::flush;
      });
  // The mean model's rollout loss on clean windows: the training seeds', the held-out seeds' and,
  // with --eval-runs, another environment's.
  // The rollout loss averages the windows that stayed finite: each is reported with how many did.
  std::size_t finite = 0, windows = 0;
  const auto loss_on = [&](const ph::se2_flights &f) {
    finite = windows = 0;
    if (f.count == 0)
      return std::numeric_limits<double>::quiet_NaN();
    const ph::se2_windows w =
        ph::cut_windows(ph::scale_controls(f, model.control_scale), points, stride);
    const ph::se2_objective o = ph::se2_rollout_objective(model, w, nullptr, nullptr, 0.0);
    finite = o.healthy;
    windows = w.count;
    return o.healthy ? o.rollout : std::numeric_limits<double>::quiet_NaN();
  };
  const auto count = [&]() {
    return " (" + std::to_string(finite) + " of " + std::to_string(windows) + " windows finite)";
  };
  const double final_train = loss_on(ph::select_seeds(all, test_seeds, false));
  std::cout << "final, the mean model on clean windows: train " << final_train << count();
  const double final_test = loss_on(ph::select_seeds(all, test_seeds, true));
  std::cout << ", held-out seeds " << final_test << count();
  double final_eval = std::numeric_limits<double>::quiet_NaN();
  if (vm.count("eval-runs")) {
    final_eval =
        loss_on(pick(ph::load_plant_log_runs(vm["eval-runs"].as<std::vector<std::string>>())));
    std::cout << ", eval runs " << final_eval << count();
  }
  std::cout << "\n";
  std::string run_list;
  for (const auto &r : runs)
    run_list += (run_list.empty() ? "" : ";") + r;
  std::string seeds;
  for (int s : test_seeds)
    seeds += (seeds.empty() ? "" : ",") + std::to_string(s);
  model.provenance = {{"source", "cvc ph-train (libcvc cvc::ph, rollout loss)"},
                      {"runs", run_list},
                      {"test_seeds", seeds},
                      {"noise", std::to_string(noise)},
                      {"noise_seed", std::to_string(noise_seed)},
                      {"seed", std::to_string(cfg.seed)},
                      {"steps", std::to_string(cfg.total_steps)},
                      {"batch", std::to_string(cfg.batch_size)},
                      {"window_points", std::to_string(points)},
                      {"window_stride", std::to_string(stride)}};
  model.provenance.emplace_back("diverged_windows", std::to_string(dropped));
  model.provenance.emplace_back("steering", model.setup.modulation_dim > 0 ? "modulation" : "port");
  if (vm.count("inputs"))
    model.provenance.emplace_back("inputs", vm["inputs"].as<std::string>());
  model.provenance.emplace_back("potential", potential);
  model.provenance.emplace_back("final_train_loss", std::to_string(final_train));
  model.provenance.emplace_back("final_test_loss", std::to_string(final_test));
  if (vm.count("eval-runs"))
    model.provenance.emplace_back("final_eval_loss", std::to_string(final_eval));
  model.save(vm["out"].as<std::string>());
  std::cout << "wrote " << vm["out"].as<std::string>() << " (" << dropped
            << " diverged training windows left out)\n";
  return 0;
}

// ===========================================================================
// Command dispatch table
// ===========================================================================

// ---------------------------------------------------------------------------
// cvc segment: cvc::segment_seeded (cvc/volume/segmentation.h), the VolumeRover GenSeg port
// ---------------------------------------------------------------------------

// The legacy flags: VolumeGridRover's Segmentation tab and GenSeg's entry took the thresholds as
// integers on the 0..255 scale the volume is rescaled to. --tlow v / --thigh v become
// --t-low=v / --t-high=v with --normalized-thresholds.
static std::vector<std::string> segment_legacy_flags(int argc, char **argv) {
  std::vector<std::string> out;
  bool normalized = false;
  // argv[0] is the command, as command_line_parser(argc, argv) skips it.
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const bool low = a == "--tlow" || a.rfind("--tlow=", 0) == 0;
    const bool high = a == "--thigh" || a.rfind("--thigh=", 0) == 0;
    if (!low && !high) {
      out.push_back(a);
      continue;
    }
    std::string v;
    const std::size_t eq = a.find('=');
    if (eq != std::string::npos) {
      v = a.substr(eq + 1);
    } else {
      if (i + 1 >= argc)
        throw std::runtime_error("cvc segment: " + a + " needs a value");
      v = argv[++i];
    }
    out.push_back(std::string(low ? "--t-low=" : "--t-high=") + v);
    if (!normalized) {
      out.push_back("--normalized-thresholds");
      normalized = true;
    }
  }
  return out;
}

// "name:i,j,k" (or "name=i,j,k"): one seed of class `name`.
static void add_seed_arg(cvc::point_classes &pc, const std::string &s, int variable, int timestep) {
  const std::size_t sep = s.find_last_of(":=");
  int ijk[3];
  char tail = 0;
  if (sep == std::string::npos || sep == 0 ||
      std::sscanf(s.c_str() + sep + 1, "%d,%d,%d%c", &ijk[0], &ijk[1], &ijk[2], &tail) != 3)
    throw std::runtime_error("cvc segment: --seed " + s + " is not name:i,j,k");
  pc.add(s.substr(0, sep), ijk[0], ijk[1], ijk[2], variable, timestep);
}

static int cmd_segment(cvc::app &app, int argc, char **argv) {
  po::options_description desc(
      "cvc segment - seeded segmentation of a volume (the VolumeRover GenSeg port, Bajaj, Yu and "
      "Auer, J. Struct. Biol. 2003)\n\n"
      "Fronts grow from each point class's seeds by fast marching, slowing at edges, and stop "
      "where they meet another class's front or a voxel below --t-low; the smoothed volume's "
      "local maxima above --t-high join the class whose front reaches them first. Writes the "
      "labels (UChar: the class index, 255 for none) to the output, and each class's voxels "
      "(zero elsewhere) beside it as <stem>_class_<n><ext>. The seeds come from a point-class "
      "file (--seeds: VolumeGridRover's .vgr, or .pts) or the command line (--seed). The legacy "
      "--tlow / --thigh take the thresholds on the 0..255 scale the volume is rescaled to.\n\n"
      "--automatic critical-points is the paper's automatic mode: the critical points of the "
      "volume's gradient field after anisotropic gradient vector diffusion seed two more "
      "classes, \"feature\" (maxima) and \"background\" (minima), so no seeds are needed; the "
      "--cp-* flags tune it (--cp-feature-threshold moves dim maxima to the background, "
      "--cp-background-threshold bright minima to the feature).\n\n"
      "Examples:\n"
      "  cvc segment map.rawiv --seeds classes.vgr --tlow 40 --thigh 200 -o labels.rawiv\n"
      "  cvc segment map.mrc --seed virus:40,40,40 --seed solvent:2,2,2 --t-low 0.1 --t-high "
      "0.8 -o labels.rawiv\n"
      "  cvc segment map.rawiv --automatic critical-points -o labels.rawiv\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume")(
      "output,o", po::value<std::string>()->required(), "output volume for the labels")(
      "seeds,s", po::value<std::string>(), "point-class file of the seeds (.vgr, .pts)")(
      "seed", po::value<std::vector<std::string>>()->composing(),
      "a seed, name:i,j,k (repeatable): a voxel of class `name`")(
      "border", po::value<std::string>(),
      "write the border voxels (where a front stopped), \"i j k\" a line, to this file")(
      "seconds", po::value<double>()->default_value(0.0),
      "report progress every this many seconds (0: never)");
  cvc::add_param_options<cvc::seeded_segmentation_params>(desc);

  const std::vector<std::string> args = segment_legacy_flags(argc, argv);
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(args).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);
  const cvc::param_result<cvc::seeded_segmentation_params> pr =
      cvc::params_from_options<cvc::seeded_segmentation_params>(vm);
  if (!pr.ok())
    throw std::runtime_error("cvc segment: " + pr.message());
  const cvc::seeded_segmentation_params &p = pr.value;

  const std::string input = vm["input"].as<std::string>();
  const std::string output = vm["output"].as<std::string>();
  cvc::volume vol(app);
  vol.read(input);

  cvc::point_classes seeds;
  if (vm.count("seeds")) {
    cvc::point_class_read_params rp;
    rp.dims = {int(vol.XDim()), int(vol.YDim()), int(vol.ZDim())};
    rp.bbox = {vol.XMin(), vol.YMin(), vol.ZMin(), vol.XMax(), vol.YMax(), vol.ZMax()};
    rp.variable = p.variable;
    rp.timestep = p.timestep;
    seeds = cvc::read_point_classes(app, vm["seeds"].as<std::string>(), rp);
  }
  if (vm.count("seed"))
    for (const std::string &s : vm["seed"].as<std::vector<std::string>>())
      add_seed_arg(seeds, s, p.variable, p.timestep);
  if (seeds.point_count() == 0 && p.automatic != cvc::seed_mode::critical_points)
    throw std::runtime_error("cvc segment: no seeds: give --seeds <file> or --seed name:i,j,k "
                             "(or --automatic critical-points, which needs none)");

  // The stepper, so a long run can report its progress. A step of 2^18 units is a few hundredths
  // of a second, and big enough for the per-voxel passes to spread over the app's compute pool
  // (a smaller one runs them on this thread).
  cvc::segment_stepper stepper(vol, std::move(seeds), p);
  const double every = vm["seconds"].as<double>();
  auto last = std::chrono::steady_clock::now();
  while (!stepper.step(std::size_t(1) << 18)) {
    if (every > 0 &&
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last).count() >= every) {
      std::cerr << "cvc segment: " << stepper.status() << "\n";
      last = std::chrono::steady_clock::now();
    }
  }
  cvc::seeded_segmentation r = stepper.take();

  r.labels.write(output);
  const std::size_t dot = output.find_last_of('.');
  const std::size_t sep = output.find_last_of("/\\");
  const bool has_ext = dot != std::string::npos && (sep == std::string::npos || dot > sep);
  const std::string stem = has_ext ? output.substr(0, dot) : output;
  const std::string ext = has_ext ? output.substr(dot) : std::string();
  for (std::size_t c = 0; c < r.classes.size(); ++c) {
    char n[16];
    std::snprintf(n, sizeof n, "_class_%02zu", c);
    r.classes[c].write(stem + n + ext);
  }
  if (vm.count("border")) {
    const std::string file = vm["border"].as<std::string>();
    std::ofstream b(file);
    if (!b)
      throw std::runtime_error("cvc segment: cannot write " + file);
    for (const auto &v : r.border)
      b << v[0] << " " << v[1] << " " << v[2] << "\n";
  }
  for (const std::string &w : r.warnings)
    std::cerr << "cvc segment: warning: " << w << "\n";
  std::size_t labelled = 0;
  for (std::size_t n : r.class_voxels)
    labelled += n;
  std::cout << "Wrote the labels of " << r.class_names.size() << " classes (" << labelled
            << " voxels labelled, " << r.border.size() << " on borders, " << r.classified_seeds
            << " of " << r.automatic_seeds << " automatic seeds classified) to " << output;
  if (!r.classes.empty())
    std::cout << ", and the class volumes to " << stem << "_class_NN" << ext;
  std::cout << "\n";
  return 0;
}

// ---------------------------------------------------------------------------
// cvc capsid-seeds / cvc rotate-seeds: VolUtils' seed generators (cvc/volume/seed_generators.h,
// transfix/libcvc#123)
// ---------------------------------------------------------------------------

static int cmd_capsid_seeds(cvc::app &app, int argc, char **argv) {
  po::options_description desc(
      "cvc capsid-seeds - seeds for a virus map (VolUtils' criticalPntsToSeed, SegSubunit's "
      "FindCriticalPoints)\n\n"
      "The map's local maxima above --t-low, thresholded so that about 3000 T of them are kept "
      "(T = h^2 + hk + k^2, the capsid's triangulation number), as class \"maxima\"; when there "
      "are few, its gradient magnitude's local maxima too, as class \"saddles\". Writes them as "
      "point classes (.vgr, or .pts) for cvc segment --seeds or rotate-seeds.\n\n"
      "Examples:\n"
      "  cvc capsid-seeds map.rawiv -o seeds.vgr --h 2 --k 1 --t-low 40\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "input volume")(
      "output,o", po::value<std::string>()->required(), "output point classes (.vgr, .pts)");
  cvc::add_param_options<cvc::capsid_seed_params>(desc);
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);
  const auto pr = cvc::params_from_options<cvc::capsid_seed_params>(vm);
  if (!pr.ok())
    throw std::runtime_error("cvc capsid-seeds: " + pr.message());
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  const cvc::point_classes pc = cvc::capsid_critical_points(vol, pr.value);
  const std::string out = vm["output"].as<std::string>();
  cvc::write_point_classes(app, pc, out);
  std::cout << "Wrote " << pc.classes.at(0).size() << " maxima and " << pc.classes.at(1).size()
            << " saddles to " << out << "\n";
  return 0;
}

static int cmd_rotate_seeds(cvc::app &app, int argc, char **argv) {
  po::options_description desc(
      "cvc rotate-seeds - N-fold rotational copies of seeds (VolUtils' SeedProduce, "
      "SeedProduceRegardingAxis)\n\n"
      "Every point of --seeds, rotated by 2 pi n / --folds about an axis (--axis x,y,z through "
      "--center x,y,z, world units; by default z through the volume's centre), as the nearest "
      "voxel of the volume: class \"Class n\" holds the n-th rotation. --from names the volume "
      "the seeds were marked on, when that is a cut of this one. Writes point classes.\n\n"
      "Examples:\n"
      "  cvc rotate-seeds map.rawiv --seeds one_unit.vgr --folds 5 -o five.vgr\n"
      "  cvc rotate-seeds map.rawiv --seeds s.vgr --from cut.rawiv --folds 3 --axis [0,1,1] "
      "-o three.vgr\n");
  desc.add_options()("help,h", "show help")("input,i", po::value<std::string>()->required(),
                                            "the volume the copies are voxels of")(
      "output,o", po::value<std::string>()->required(), "output point classes (.vgr, .pts)")(
      "seeds,s", po::value<std::string>()->required(), "the seeds' point classes (.vgr, .pts)")(
      "from", po::value<std::string>(), "the volume the seeds were marked on (default: input)");
  cvc::add_param_options<cvc::rotation_seed_params>(desc);
  po::positional_options_description pos;
  pos.add("input", 1).add("output", 1);
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).run(), vm);
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 0;
  }
  po::notify(vm);
  const auto pr = cvc::params_from_options<cvc::rotation_seed_params>(vm);
  if (!pr.ok())
    throw std::runtime_error("cvc rotate-seeds: " + pr.message());
  cvc::volume vol(app);
  vol.read(vm["input"].as<std::string>());
  cvc::dimension from_dims = vol.voxel_dimensions();
  cvc::bounding_box from_box = vol.boundingBox();
  if (vm.count("from")) {
    const cvc::volume_file_info info(app, vm["from"].as<std::string>());
    from_dims = info.voxel_dimensions();
    from_box = info.boundingBox();
  }
  cvc::point_class_read_params rp;
  rp.dims = {int(from_dims.XDim()), int(from_dims.YDim()), int(from_dims.ZDim())};
  rp.bbox = {from_box.minx, from_box.miny, from_box.minz,
             from_box.maxx, from_box.maxy, from_box.maxz};
  rp.variable = pr.value.variable;
  rp.timestep = pr.value.timestep;
  const cvc::point_classes seeds = cvc::read_point_classes(app, vm["seeds"].as<std::string>(), rp);
  const cvc::point_classes pc = cvc::rotated_seeds(seeds, from_dims, from_box, vol, pr.value);
  const std::string out = vm["output"].as<std::string>();
  cvc::write_point_classes(app, pc, out);
  std::cout << "Wrote " << pc.point_count() << " points in " << pc.classes.size() << " classes to "
            << out << "\n";
  return 0;
}

struct command_entry {
  const char *name;
  const char *category;
  const char *help;
  int (*func)(cvc::app &, int, char **);
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
  {"skeleton",       "Geometry",        "skeleton of a point cloud (strands and sheets)", cmd_skeleton},
  {"hexahedralize",  "Geometry",        "extract hexahedral mesh from volume",            cmd_hexahedralize},
  {"tetrahedralize2","Geometry",        "extract dual-tet (tet2) mesh from volume",       cmd_tetrahedralize2},
  {"layer-mesh",     "Geometry",        "mesh layer between two isosurfaces",             cmd_layer_mesh},
#endif
  {"tile",           "Geometry",        "surfaces between a contour stack's slices",      cmd_tile},

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
  {"filter",         "Filters",         "the four filters as a resumable job (VolumeRover2's defaults)", cmd_filter},

  // ── Analysis ──
  {"ssim",           "Analysis",        "compute SSIM between two volumes",              cmd_ssim},
  {"compare",        "Analysis",        "exact voxel comparison (nonzero exit on mismatch)", cmd_compare},
  {"segment",        "Analysis",        "seeded segmentation (GenSeg): labels and class volumes", cmd_segment},
  {"capsid-seeds",   "Analysis",        "seeds for a virus map: maxima kept for its T-number (VolUtils)", cmd_capsid_seeds},
  {"rotate-seeds",   "Analysis",        "N-fold rotational copies of seeds about an axis (VolUtils)", cmd_rotate_seeds},

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

  // ── Vehicle physics ──
  {"plant-log",      "Vehicle Physics", "log nav driving Jolt vehicles for plant fitting", cmd_plant_log},
  {"ph-train",       "Vehicle Physics", "fit the learned port-Hamiltonian vehicle to plant logs", cmd_ph_train},
  {"ph",             "Vehicle Physics", "composed port-Hamiltonian systems: info, simulate, check", cmd_ph},

  // ── Robots ──
  {"mujoco",         "Robots",          "a MuJoCo robot on libcvc terrain: info, or a stand and push trial", cmd_mujoco},

  // ── Replay ──
  {"trace",          "Replay",          "an input trace: info, or verify by replaying it", cmd_trace},
  {"nav",            "Replay",          "cvc::nav's lab: a run log's Stats, or scrub a training run's checkpoints", cmd_nav},

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
        cvc::app app; // the command's, gone (its threads joined) before main returns
        return commands[i].func(app, argc - 1, argv + 1);
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
