/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#ifndef __CVC_IMAGE_ENCODE_H__
#define __CVC_IMAGE_ENCODE_H__

// cvc::encode_png — an image as PNG bytes in memory, with no file and no image handler (stb's
// encoder, built in everywhere, the browser included). The same pixels always encode to the same
// bytes, so a file made from them hashes the same on every platform (a world bundle's ground.png).

#include <cvc/image/image.h>
#include <string>

namespace cvc {

// `img` as PNG, its first row at the top. Grey, grey + alpha, RGB and RGBA u8 keep their channels;
// grey u16 is written at 16 bits (a height map: a world bundle's export/heightmap.png); any other
// format or type is converted to RGBA u8 first. Throws std::runtime_error on an empty
// image or a failed encode. Thread-safe.
std::string encode_png(const image &img);

} // namespace cvc

#endif // __CVC_IMAGE_ENCODE_H__
