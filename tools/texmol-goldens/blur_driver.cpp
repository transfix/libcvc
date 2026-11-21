// blur_legacy: TexMol's PDB parser and GOABlur, as they are at the pinned sha, on one PDB file.
// It stands in for BlurMapsDataManager::getBlurredVolume (the resolution entry), which the real
// BlurMapsDataManager.cpp can't be built without the rest of TexMol: it parses the file
// (parserPDBtoGOA::parsePDBModels), flattens it (FlattenGOA), makes the grid's dims from the
// resolution as getBlurredVolume does (dim = (max - min) / resolution + 3, the box from
// getBoundingBox), and calls GOABlur::generateBlurmap for the electron density with van der Waals
// radii and no colour.
//
//   blur_legacy <in.pdb> <resolution> <blobbiness> <out prefix>
//
// writes <prefix>.f32 (the density, float32 little-endian, x fastest), <prefix>.atoms.txt (the
// flattened atoms in order: x y z radius, %.9g) and <prefix>.grid.txt (dims, then the box's min and
// max, %.9g).

#include <Blurmaps/GOABlur.h>
#include <PDBParser/FlattenGOA.h>
#include <PDBParser/parserPDBtoGOA.h>
#include <SimpleVolumeData/SimpleVolumeData.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 5) {
    std::fprintf(stderr, "usage: blur_legacy <in.pdb> <resolution> <blobbiness> <out prefix>\n");
    return 2;
  }
  const std::string in = argv[1], prefix = argv[4];
  const double resolution = std::atof(argv[2]), blob = std::atof(argv[3]);

  PDBParser::parserPDBtoGOA parser;
  PDBParser::GroupOfAtoms *molecule = parser.parsePDBModels(in.c_str());
  if (!molecule) {
    std::fprintf(stderr, "blur_legacy: can't parse %s\n", in.c_str());
    return 1;
  }

  // getBlurredVolume's grid (BlurMapsDataManager.cpp), with the radius type and the flattening
  // generateBlurmap uses below.
  const PDBParser::GroupOfAtoms::RADIUS_TYPE radiusType = PDBParser::GroupOfAtoms::VDW_RADIUS;
  std::vector<PDBParser::Atom *> atomList;
  PDBParser::CollectionData *collectionData = 0;
  if (molecule->type == PDBParser::COLLECTION_TYPE)
    collectionData = molecule->m_CollectionData;
  FlattenGOA(molecule, atomList, collectionData, 0, 0, 0, radiusType, 0, false);
  float min[3] = {0, 0, 0}, max[3] = {0, 0, 0};
  BlurMapsDataManager::getBoundingBox(atomList, min, max, radiusType, 0, blob);
  const int dim1 = (max[0] - min[0]) / resolution + 3;
  const int dim2 = (max[1] - min[1]) / resolution + 3;
  const int dim3 = (max[2] - min[2]) / resolution + 3;

  SimpleVolumeData *vol = new SimpleVolumeData(dim1, dim2, dim3);
  GOABlur blur;
  blur.setBlobbyness(blob);
  if (!blur.generateBlurmap(molecule, vol, PDBParser::GroupOfAtoms::ELECTRON_DENSITY, 0, false,
                            PDBParser::GroupOfAtoms::ATOM, radiusType, 0)) {
    std::fprintf(stderr, "blur_legacy: generateBlurmap failed\n");
    return 1;
  }

  const float *dens = static_cast<const float *>(vol->getData(0));
  const std::size_t n = static_cast<std::size_t>(dim1) * dim2 * dim3;
  FILE *f = std::fopen((prefix + ".f32").c_str(), "wb");
  if (!f || std::fwrite(dens, sizeof(float), n, f) != n)
    return 1;
  std::fclose(f);

  FILE *a = std::fopen((prefix + ".atoms.txt").c_str(), "w");
  if (!a)
    return 1;
  for (PDBParser::Atom *at : atomList)
    std::fprintf(a, "%.9g %.9g %.9g %.9g\n", at->m_Position[0], at->m_Position[1],
                 at->m_Position[2], at->getRadius(radiusType));
  std::fclose(a);

  FILE *g = std::fopen((prefix + ".grid.txt").c_str(), "w");
  if (!g)
    return 1;
  std::fprintf(g, "%d %d %d\n%.9g %.9g %.9g\n%.9g %.9g %.9g\n", dim1, dim2, dim3, vol->getMinX(),
               vol->getMinY(), vol->getMinZ(), vol->getMaxX(), vol->getMaxY(), vol->getMaxZ());
  std::fclose(g);
  std::printf("%s: %zu atoms, %dx%dx%d\n", in.c_str(), atomList.size(), dim1, dim2, dim3);
  return 0;
}
