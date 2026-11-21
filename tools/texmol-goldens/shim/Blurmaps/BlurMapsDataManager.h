// Stands in for TexMol's Blurmaps/BlurMapsDataManager.h, which GOABlur.h includes. The real header
// pulls in most of TexMol (Geometry, Curvatures, the volume and geometry file types); GOABlur uses
// only getBoundingBox from it. CMakeLists.txt copies that function, as it is at the pinned sha, out
// of BlurMapsDataManager.cpp into the build tree.
#ifndef TEXMOL_GOLDENS_BLURMAPSDATAMANAGER_H
#define TEXMOL_GOLDENS_BLURMAPSDATAMANAGER_H

#include <PDBParser/Atom.h>
#include <PDBParser/GroupOfAtoms.h>
#include <vector>

class BlurMapsDataManager {
public:
  static void getBoundingBox(std::vector<PDBParser::Atom *> &grp, float min[3], float max[3],
                             PDBParser::GroupOfAtoms::RADIUS_TYPE radiusType, int probeRadius,
                             double blobbiness);
};

#endif
