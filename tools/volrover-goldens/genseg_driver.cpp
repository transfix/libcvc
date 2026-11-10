// Runs the legacy GenSeg (transfix/volrover src/Segmentation/GenSeg, at the sha in ../volrover.sha)
// as its XML-RPC entry generalSegmentation (general_segmentation.cpp) did, from the command line:
//
//   genseg_legacy <input.rawiv> <tlow> <thigh> <output_prefix> <seeds.txt>
//
// seeds.txt has one seed a line, "<class> <x> <y> <z>", classes numbered from 0; within a class
// the seeds keep their order, as the XML-RPC call's list did. tlow and thigh are integers, as
// generalSegmentation read them. The legacy code then reads the rawiv file (normalizing it to
// 0..255) and writes <prefix>_indices.txt, <prefix>_indices.pcd and <prefix>_subunit_NN.rawiv into
// the current directory. Part of tools/volrover-goldens: never built into libcvc.

#include <Segmentation/GenSeg/genseg.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 6) {
    std::cerr << "usage: genseg_legacy <input.rawiv> <tlow> <thigh> <output_prefix> <seeds.txt>\n";
    return 2;
  }
  const std::string input = argv[1];
  const int tlow = std::atoi(argv[2]);
  const int thigh = std::atoi(argv[3]);
  const std::string prefix = argv[4];
  std::ifstream in(argv[5]);
  if (!in) {
    std::cerr << "genseg_legacy: cannot read " << argv[5] << "\n";
    return 2;
  }
  std::map<int, std::vector<int>> seeds; // class -> x y z ...
  int num_classes = 0;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ls(line);
    int c, x, y, z;
    if (!(ls >> c >> x >> y >> z))
      continue;
    if (c < 0) {
      std::cerr << "genseg_legacy: a negative class\n";
      return 2;
    }
    seeds[c].insert(seeds[c].end(), {x, y, z});
    num_classes = std::max(num_classes, c + 1);
  }
  if (num_classes == 0) {
    std::cerr << "genseg_legacy: no seeds\n";
    return 2;
  }
  // The seed lists, built as generalSegmentation built them.
  GenSeg::SDPNT **seed_list = new GenSeg::SDPNT *[num_classes];
  for (int i = 0; i < num_classes; ++i) {
    seed_list[i] = nullptr;
    GenSeg::SDPNT *cur = nullptr;
    const std::vector<int> &s = seeds[i];
    for (std::size_t j = 0; j < s.size(); j += 3) {
      GenSeg::SDPNT *p = new GenSeg::SDPNT;
      p->x = s[j];
      p->y = s[j + 1];
      p->z = s[j + 2];
      p->next = nullptr;
      if (cur)
        cur->next = p;
      else
        seed_list[i] = p;
      cur = p;
    }
  }
  int xdim = 0, ydim = 0, zdim = 0;
  float *image = nullptr;
  GenSeg::read_data(&xdim, &ydim, &zdim, &image, input.c_str());
  GenSeg::Segment(xdim, ydim, zdim, image, tlow, thigh, seed_list, num_classes, prefix.c_str());
  for (int i = 0; i < num_classes; ++i)
    for (GenSeg::SDPNT *p = seed_list[i]; p;) {
      GenSeg::SDPNT *n = p->next;
      delete p;
      p = n;
    }
  delete[] seed_list;
  std::free(image);
  return 0;
}
