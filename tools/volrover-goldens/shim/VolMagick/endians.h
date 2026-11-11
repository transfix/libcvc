// legacy VolMagick's big_endian(), which legacy GenSeg's rawiv reader and writer call: whether this
// machine is big-endian (rawiv files are). Part of tools/volrover-goldens: never built into libcvc.

#ifndef VOLROVER_GOLDENS_SHIM_ENDIANS_H
#define VOLROVER_GOLDENS_SHIM_ENDIANS_H

static inline int big_endian() {
  const unsigned one = 1;
  return *reinterpret_cast<const unsigned char *>(&one) == 0;
}

#endif
