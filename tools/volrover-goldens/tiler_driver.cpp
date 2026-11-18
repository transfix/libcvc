// Runs the legacy ContourTiler (transfix/volrover src/ContourTiler, at the sha in ../volrover.sha)
// on a Reconstruct series, as its command line did with a .ser input (main.cpp's ser branch):
//
//   tiler_legacy <series base> <first> <last> <outdir> [name=value ...]
//
// <series base> is the series' path without ".ser"; its sections are <base>.<n>. Sections first to
// last are read with the legacy reader (xml/Ser_reader, every component kept), put into one Slice
// per section index, and tiled with tile(). The parameters are the legacy Tiler_options' (-e
// collinear_epsilon, -C curation_delta, -r remove_intersections, -v correspondence_overlap, -z
// z_scale), the rest at their defaults. tile() writes <outdir>/_tiles.rawc and one
// <outdir>/<component>_tiles.rawc per component; CMakeLists.txt patches raw_print_tiles_impl to
// write coordinates with 17 digits and z as the tiler's own (the section index for a contour
// vertex), not (z + 0.5) * z_scale. Part of tools/volrover-goldens: never built into libcvc.

#include <ContourTiler/Contour.h>
#include <ContourTiler/Slice.h>
#include <ContourTiler/Tiler_options.h>
#include <ContourTiler/tiler.h>
#include <ContourTiler/xml/Ser_reader.h>
#include <boost/unordered_map.hpp>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

using namespace ContourTiler;

int main(int argc, char **argv) {
  if (argc < 5) {
    std::cerr << "usage: tiler_legacy <series base> <first> <last> <outdir> [name=value ...]\n";
    return 2;
  }
  const std::string base = argv[1];
  const int first = std::atoi(argv[2]), last = std::atoi(argv[3]);
  Tiler_options options;
  options.output_dir() = argv[4];
  options.base_name() = "";
  options.output_raw() = true;
  for (int i = 5; i < argc; ++i) {
    const char *eq = std::strchr(argv[i], '=');
    if (!eq) {
      std::cerr << "tiler_legacy: '" << argv[i] << "' is not name=value\n";
      return 2;
    }
    const std::string name(argv[i], eq - argv[i]);
    const double v = std::atof(eq + 1);
    if (name == "collinear_epsilon")
      options.collinear_epsilon() = v;
    else if (name == "curation_delta")
      options.contour_curation_delta() = v;
    else if (name == "remove_intersections")
      options.remove_intersections() = v != 0;
    else if (name == "correspondence_overlap")
      options.correspondence_overlap() = v;
    else if (name == "z_scale")
      options.z_scale() = v;
    else {
      std::cerr << "tiler_legacy: no parameter '" << name << "'\n";
      return 2;
    }
  }

  // read_contours_ser_new (reader_ser.h), without the ContourFilter headers that file includes.
  std::vector<Ser_section> sections;
  std::vector<std::string> warnings;
  read_ser(base, first, last, std::back_inserter(sections), std::back_inserter(warnings), true);
  for (const std::string &w : warnings)
    std::cerr << "warning: " << w << "\n";
  std::vector<Slice> slices(last - first + 1);
  for (const Ser_section &section : sections) {
    const int z = section.index();
    for (const Ser_contour &contour : section) {
      Polygon_2 P;
      for (const Ser_point &point : contour)
        P.push_back(Point_2(point.x(), point.y(), z));
      const Contour::Info info(z, contour.name(), contour.name());
      slices[z - first].push_back(contour.name(), Contour::create(P, info));
    }
  }

  boost::unordered_map<std::string, Color> comp2color;
  try {
    tile(slices.begin(), slices.end(), comp2color, options);
  } catch (const std::exception &e) {
    std::cerr << "tiler_legacy: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
