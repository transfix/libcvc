// parser_legacy: TexMol's PDBParser on a PDB file, its group tree and torsions written out for
// cvc::mol's parity test (molecule_groups_test). Built only by tools/texmol-goldens; never part of
// libcvc.
//
//   parser_legacy <in.pdb> <out.txt>
//
// One line per group of the tree parsePDBModels builds, depth first (the order FlattenGOA walks),
// each indented one blank per level:
//
//   collection <models>
//    model <chains>
//     chain <id> protein|nucleic|null <groups>
//      helix|sheet|turn|none <residues>
//       residue <chain> <seq> <icode> <atoms> <phi> <psi>
//
// A blank chain or insertion code is written as '_'. phi and psi are the radians TexMol's
// finalizeParsing stored (calculateProteinChainTorsionAngles), %.9g; a residue with no ResidueData
// writes "- -".

#include <PDBParser/GroupOfAtoms.h>
#include <PDBParser/ResidueData.h>
#include <PDBParser/ResidueHeader.h>
#include <PDBParser/parserPDBtoGOA.h>
#include <cstdio>
#include <string>

namespace {

char show(char c) { return c == ' ' || c == '\0' ? '_' : c; }

void walk(FILE *f, PDBParser::GroupOfAtoms *g, int depth) {
  const std::string pad(static_cast<std::size_t>(depth), ' ');
  switch (g->type) {
  case PDBParser::COLLECTION_TYPE:
    std::fprintf(f, "%scollection %zu\n", pad.c_str(), g->m_SubGroups.size());
    break;
  case PDBParser::PROTEIN_TYPE:
    std::fprintf(f, "%smodel %zu\n", pad.c_str(), g->m_SubGroups.size());
    break;
  case PDBParser::CHAIN_TYPE:
    std::fprintf(f, "%schain %c %s %zu\n", pad.c_str(), show(static_cast<char>(g->resID)),
                 g->resPos == PDBParser::CHAIN_NUCLEIC   ? "nucleic"
                 : g->resPos == PDBParser::CHAIN_PROTEIN ? "protein"
                                                         : "null",
                 g->m_SubGroups.size());
    break;
  case PDBParser::SS_TYPE: {
    const char *kind = g->resID == PDBParser::SS_HELIX   ? "helix"
                       : g->resID == PDBParser::SS_SHEET ? "sheet"
                       : g->resID == PDBParser::SS_TURN  ? "turn"
                                                         : "none";
    std::fprintf(f, "%s%s %zu\n", pad.c_str(), kind, g->m_SubGroups.size());
    break;
  }
  case PDBParser::RESIDUE_TYPE: {
    const PDBParser::ResidueHeader *h = g->resHeader;
    std::fprintf(f, "%sresidue %c %d %c %zu", pad.c_str(), show(h ? h->chainID : ' '),
                 h ? h->seqNum : g->m_ResSeq, show(h ? h->iCode : ' '), g->m_Atoms.size());
    if (g->m_ResidueData)
      std::fprintf(f, " %.9g %.9g\n", g->m_ResidueData->m_Phi, g->m_ResidueData->m_Psi);
    else
      std::fprintf(f, " - -\n");
    break;
  }
  default:
    std::fprintf(f, "%sgroup %d %zu\n", pad.c_str(), g->type, g->m_SubGroups.size());
    break;
  }
  for (PDBParser::GroupOfAtoms *c : g->m_SubGroups)
    walk(f, c, depth + 1);
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::fprintf(stderr, "usage: parser_legacy <in.pdb> <out.txt>\n");
    return 2;
  }
  PDBParser::parserPDBtoGOA parser;
  PDBParser::GroupOfAtoms *molecule = parser.parsePDBModels(argv[1]);
  if (!molecule) {
    std::fprintf(stderr, "parser_legacy: can't parse %s\n", argv[1]);
    return 1;
  }
  FILE *f = std::fopen(argv[2], "w");
  if (!f)
    return 1;
  walk(f, molecule, 0);
  std::fclose(f);
  std::printf("%s: tree written to %s\n", argv[1], argv[2]);
  return 0;
}
