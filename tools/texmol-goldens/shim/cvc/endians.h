// Stands in for libcvc 3.x's cvc/endians.h, which TexMol's ByteOrder/ByteSwapping.h includes for
// one function. The goldens build links no libcvc.
#ifndef TEXMOL_GOLDENS_CVC_ENDIANS_H
#define TEXMOL_GOLDENS_CVC_ENDIANS_H

#include <cstdint>
#include <cstring>

namespace cvc {
inline bool big_endian() {
  const std::uint16_t one = 1;
  unsigned char first = 0;
  std::memcpy(&first, &one, 1);
  return first == 0;
}
} // namespace cvc

#endif
