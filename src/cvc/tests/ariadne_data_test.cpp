/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
// Ariadne data (cvc/ariadne/data.h) and the archive reader under it (cvc/utility/archive.h): tar,
// gzip, zstd and zip from memory; the store's sources (a directory, an archive file, cvcpkg through
// a canned registry), its decoders, links and lazy loads; and the loader's data: block.

#include <atomic>
#include <boost/any.hpp>
#include <cstdint>
#include <cstdlib>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/height_field.h>
#include <cvc/image/image.h>
#include <cvc/image/image_decode.h>
#include <cvc/image/image_encode.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/mol/molecule.h>
#include <cvc/net/http_client.h>
#include <cvc/state/state.h>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <cvc/utility/archive.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_info.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/world/world_geometry.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace ar = cvc::archive;
using cvc::ariadne::DataSpec;
using cvc::ariadne::DataStore;

namespace {
// A document checked against a fresh app's registries: the built-in widget types, blocks and
// decoders only. (A test that registers its own loads against that app instead.)
[[maybe_unused]] cvc::ariadne::LoadResult fresh_load(const std::string &yaml) {
  cvc::app app;
  return cvc::ariadne::load_string(app, yaml);
}
[[maybe_unused]] cvc::ariadne::LoadResult fresh_load_file(const std::string &path) {
  cvc::app app;
  return cvc::ariadne::load_file(app, path);
}
} // namespace

namespace {

// Fixtures made with Python's gzip and zipfile (zlib): kGzipDynamic is 600 words of a small
// vocabulary at level 9 (a dynamic-Huffman block with long matches; 4324 bytes, CRC-32 0x3faa167d),
// kGzipFixed a short string at level 1 (a fixed-Huffman block), and kZip that text deflated beside
// a stored entry.
const unsigned char kGzipDynamic[] = {
    0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0a, 0x8d, 0x97, 0x6b, 0x6e, 0xc2, 0x40,
    0x0c, 0x84, 0xaf, 0xc2, 0xd5, 0x52, 0x11, 0x41, 0xa4, 0x96, 0x48, 0x81, 0xb4, 0xd7, 0x2f, 0x62,
    0x37, 0xad, 0xbf, 0xf1, 0x38, 0xe1, 0x0f, 0x90, 0x7d, 0xda, 0xe3, 0x99, 0x71, 0x58, 0xe6, 0xf5,
    0x31, 0x9e, 0x86, 0xf5, 0xfe, 0x98, 0x6e, 0xa7, 0xcb, 0x32, 0x9c, 0xc7, 0xd3, 0x63, 0x5c, 0x96,
    0xe1, 0xf9, 0xf4, 0xb1, 0x4e, 0x9f, 0xe7, 0xe9, 0x76, 0xb9, 0x87, 0x5f, 0xcb, 0x6b, 0xf9, 0xb6,
    0xe2, 0xba, 0x7e, 0x7d, 0x8f, 0x6e, 0x43, 0x3b, 0xa8, 0x7d, 0xfe, 0x8f, 0xf6, 0xe5, 0xba, 0x2c,
    0x6f, 0x97, 0x73, 0xb9, 0x4c, 0x26, 0x7b, 0xe8, 0xcf, 0x25, 0xd3, 0xdc, 0x57, 0xf6, 0xa1, 0x10,
    0xf5, 0x6b, 0x32, 0x0d, 0xf7, 0x93, 0x5a, 0x4e, 0x2e, 0x5b, 0xb9, 0xf2, 0x7e, 0x1d, 0xce, 0xf3,
    0x4f, 0xbf, 0xa4, 0xed, 0xea, 0x43, 0xfd, 0xab, 0x8d, 0xb5, 0xdb, 0xfa, 0x9e, 0x7e, 0x69, 0x4a,
    0xbd, 0x2d, 0xc2, 0xbe, 0xed, 0xe1, 0x35, 0xe3, 0xc2, 0x41, 0x76, 0x4b, 0x2c, 0x1c, 0x22, 0xcb,
    0x78, 0xf6, 0xb8, 0xc2, 0x27, 0xe3, 0xb6, 0x95, 0x8e, 0x01, 0x66, 0x48, 0x18, 0x7f, 0x00, 0x9f,
    0x0c, 0x41, 0x82, 0xa9, 0x00, 0x7d, 0x56, 0x60, 0x46, 0xb5, 0xfa, 0x58, 0x24, 0x54, 0x8a, 0xc9,
    0x81, 0x80, 0x43, 0xe2, 0x50, 0x8c, 0xb2, 0xfd, 0x66, 0xa5, 0xd2, 0xb1, 0x0c, 0x44, 0xd8, 0xd7,
    0xef, 0x04, 0x09, 0xdb, 0xa7, 0xf0, 0x33, 0x5e, 0x0b, 0x34, 0xe4, 0xa4, 0x98, 0xa8, 0x57, 0x91,
    0xad, 0x75, 0xe2, 0x17, 0x29, 0x9b, 0x15, 0x81, 0x2a, 0xe5, 0xf2, 0xbe, 0xbd, 0x01, 0xf5, 0x21,
    0x0c, 0x99, 0x6e, 0x8e, 0x67, 0x64, 0xa2, 0xe8, 0x49, 0xa4, 0x50, 0x44, 0x05, 0x2e, 0x0a, 0xac,
    0xd0, 0xb8, 0xcc, 0xe9, 0xf1, 0x31, 0x6e, 0xa8, 0x0c, 0x87, 0x90, 0x08, 0xc8, 0x9f, 0x63, 0xe0,
    0xfe, 0x76, 0xe7, 0x8e, 0x7e, 0x70, 0x0b, 0x1d, 0x45, 0x1c, 0xea, 0xa0, 0xda, 0x2e, 0x5c, 0x91,
    0x1a, 0x8e, 0xaf, 0xbd, 0xc6, 0xb2, 0x28, 0x52, 0x59, 0x37, 0x80, 0x9c, 0x95, 0x94, 0xa4, 0x0c,
    0x94, 0x2f, 0x9f, 0x50, 0x0c, 0xd9, 0x97, 0xc9, 0x88, 0x00, 0xbd, 0x60, 0x09, 0x75, 0xc6, 0xdd,
    0xf4, 0x91, 0xbf, 0xad, 0xb1, 0x30, 0x58, 0xe1, 0xa5, 0xec, 0xd3, 0xe4, 0x57, 0xed, 0xaf, 0x5e,
    0x37, 0x42, 0x6b, 0xe4, 0x91, 0xf1, 0x90, 0xa2, 0xa7, 0x09, 0xb2, 0x3b, 0x32, 0x52, 0x98, 0x14,
    0x71, 0x72, 0x64, 0x28, 0x68, 0x50, 0xf0, 0xb6, 0x78, 0x2f, 0x28, 0xd4, 0x4e, 0x3d, 0x46, 0x5b,
    0x67, 0x90, 0x45, 0x0c, 0xb4, 0x6f, 0x44, 0x5d, 0xb6, 0x02, 0xaf, 0x3a, 0x0a, 0xd9, 0xf5, 0x51,
    0xbe, 0xb0, 0x44, 0x42, 0x39, 0x89, 0xa5, 0xb8, 0x93, 0x13, 0xb9, 0x46, 0x72, 0xd0, 0xd8, 0x9c,
    0x7c, 0xf4, 0x5c, 0xdb, 0x9a, 0xaa, 0x45, 0x3a, 0xbe, 0x13, 0xbf, 0x2e, 0x10, 0x54, 0x22, 0xd0,
    0xc8, 0xa2, 0x6c, 0x62, 0x48, 0x59, 0xa2, 0x06, 0x69, 0x8c, 0xaa, 0x6d, 0x67, 0x66, 0x05, 0x71,
    0xac, 0x7e, 0x5b, 0x4b, 0x4d, 0x7a, 0x34, 0xae, 0x87, 0x58, 0x9c, 0x15, 0x1b, 0x1b, 0x95, 0xdc,
    0x2a, 0xec, 0xad, 0x2d, 0x69, 0x30, 0x06, 0x0b, 0x6f, 0x4c, 0x80, 0x46, 0x6b, 0x95, 0x5a, 0x38,
    0x52, 0xf0, 0x6e, 0x6a, 0x7b, 0x6c, 0x28, 0x7a, 0xf6, 0x1f, 0x03, 0xd5, 0x31, 0xef, 0xec, 0x9f,
    0x15, 0x3e, 0x19, 0x56, 0x14, 0xfb, 0x21, 0x63, 0x27, 0x31, 0x5e, 0x70, 0xc4, 0x58, 0x65, 0x11,
    0x3c, 0xac, 0xa0, 0xcd, 0x3e, 0xf3, 0x6c, 0x05, 0x9d, 0x67, 0x14, 0xe8, 0x2a, 0x8e, 0xf6, 0xf5,
    0x85, 0x36, 0xb6, 0x5b, 0xad, 0x43, 0xb1, 0x06, 0xd7, 0xb3, 0x40, 0x16, 0x6f, 0x22, 0xde, 0x5c,
    0x30, 0xeb, 0x3a, 0x54, 0x02, 0xa4, 0x68, 0x26, 0x96, 0xdf, 0x69, 0xb3, 0xf9, 0x3f, 0x43, 0xa0,
    0x1c, 0xdc, 0xc6, 0x62, 0x30, 0xa4, 0x1d, 0x64, 0x07, 0xf3, 0xc2, 0x07, 0xdc, 0x52, 0xd7, 0x07,
    0xa2, 0xc1, 0xbe, 0xd1, 0xbc, 0xbc, 0x36, 0xad, 0xc4, 0x63, 0x72, 0x47, 0x7f, 0xf3, 0xb3, 0xcf,
    0x57, 0x2f, 0xdc, 0x1c, 0x77, 0xc6, 0x1e, 0x93, 0x81, 0xcd, 0x38, 0xf5, 0x57, 0x6a, 0xfc, 0x05,
    0x7d, 0x16, 0xaa, 0x3f, 0xe4, 0x10, 0x00, 0x00,
};
const unsigned char kGzipFixed[] = {
    0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x0a, 0xcb, 0x48, 0xcd, 0xc9,
    0xc9, 0x57, 0xc8, 0x40, 0x90, 0x3a, 0x0a, 0x89, 0x45, 0x99, 0x89, 0x29, 0x79, 0xa9,
    0x0a, 0x29, 0x89, 0x25, 0x89, 0x00, 0x5c, 0x6c, 0x3c, 0x9a, 0x1f, 0x00, 0x00, 0x00,
};
const unsigned char kZip[] = {
    0x50, 0x4b, 0x03, 0x04, 0x14, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x21, 0x5c, 0x7d, 0x16,
    0xaa, 0x3f, 0x46, 0x02, 0x00, 0x00, 0xe4, 0x10, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x73, 0x68,
    0x61, 0x72, 0x65, 0x2f, 0x64, 0x6f, 0x63, 0x2f, 0x6e, 0x6f, 0x74, 0x65, 0x73, 0x2e, 0x74, 0x78,
    0x74, 0x8d, 0x97, 0x6b, 0x6e, 0xc2, 0x40, 0x0c, 0x84, 0xaf, 0xc2, 0xd5, 0x52, 0x11, 0x41, 0xa4,
    0x96, 0x48, 0x81, 0xb4, 0xd7, 0x2f, 0x62, 0x37, 0xad, 0xbf, 0xf1, 0x38, 0xe1, 0x0f, 0x90, 0x7d,
    0xda, 0xe3, 0x99, 0x71, 0x58, 0xe6, 0xf5, 0x31, 0x9e, 0x86, 0xf5, 0xfe, 0x98, 0x6e, 0xa7, 0xcb,
    0x32, 0x9c, 0xc7, 0xd3, 0x63, 0x5c, 0x96, 0xe1, 0xf9, 0xf4, 0xb1, 0x4e, 0x9f, 0xe7, 0xe9, 0x76,
    0xb9, 0x87, 0x5f, 0xcb, 0x6b, 0xf9, 0xb6, 0xe2, 0xba, 0x7e, 0x7d, 0x8f, 0x6e, 0x43, 0x3b, 0xa8,
    0x7d, 0xfe, 0x8f, 0xf6, 0xe5, 0xba, 0x2c, 0x6f, 0x97, 0x73, 0xb9, 0x4c, 0x26, 0x7b, 0xe8, 0xcf,
    0x25, 0xd3, 0xdc, 0x57, 0xf6, 0xa1, 0x10, 0xf5, 0x6b, 0x32, 0x0d, 0xf7, 0x93, 0x5a, 0x4e, 0x2e,
    0x5b, 0xb9, 0xf2, 0x7e, 0x1d, 0xce, 0xf3, 0x4f, 0xbf, 0xa4, 0xed, 0xea, 0x43, 0xfd, 0xab, 0x8d,
    0xb5, 0xdb, 0xfa, 0x9e, 0x7e, 0x69, 0x4a, 0xbd, 0x2d, 0xc2, 0xbe, 0xed, 0xe1, 0x35, 0xe3, 0xc2,
    0x41, 0x76, 0x4b, 0x2c, 0x1c, 0x22, 0xcb, 0x78, 0xf6, 0xb8, 0xc2, 0x27, 0xe3, 0xb6, 0x95, 0x8e,
    0x01, 0x66, 0x48, 0x18, 0x7f, 0x00, 0x9f, 0x0c, 0x41, 0x82, 0xa9, 0x00, 0x7d, 0x56, 0x60, 0x46,
    0xb5, 0xfa, 0x58, 0x24, 0x54, 0x8a, 0xc9, 0x81, 0x80, 0x43, 0xe2, 0x50, 0x8c, 0xb2, 0xfd, 0x66,
    0xa5, 0xd2, 0xb1, 0x0c, 0x44, 0xd8, 0xd7, 0xef, 0x04, 0x09, 0xdb, 0xa7, 0xf0, 0x33, 0x5e, 0x0b,
    0x34, 0xe4, 0xa4, 0x98, 0xa8, 0x57, 0x91, 0xad, 0x75, 0xe2, 0x17, 0x29, 0x9b, 0x15, 0x81, 0x2a,
    0xe5, 0xf2, 0xbe, 0xbd, 0x01, 0xf5, 0x21, 0x0c, 0x99, 0x6e, 0x8e, 0x67, 0x64, 0xa2, 0xe8, 0x49,
    0xa4, 0x50, 0x44, 0x05, 0x2e, 0x0a, 0xac, 0xd0, 0xb8, 0xcc, 0xe9, 0xf1, 0x31, 0x6e, 0xa8, 0x0c,
    0x87, 0x90, 0x08, 0xc8, 0x9f, 0x63, 0xe0, 0xfe, 0x76, 0xe7, 0x8e, 0x7e, 0x70, 0x0b, 0x1d, 0x45,
    0x1c, 0xea, 0xa0, 0xda, 0x2e, 0x5c, 0x91, 0x1a, 0x8e, 0xaf, 0xbd, 0xc6, 0xb2, 0x28, 0x52, 0x59,
    0x37, 0x80, 0x9c, 0x95, 0x94, 0xa4, 0x0c, 0x94, 0x2f, 0x9f, 0x50, 0x0c, 0xd9, 0x97, 0xc9, 0x88,
    0x00, 0xbd, 0x60, 0x09, 0x75, 0xc6, 0xdd, 0xf4, 0x91, 0xbf, 0xad, 0xb1, 0x30, 0x58, 0xe1, 0xa5,
    0xec, 0xd3, 0xe4, 0x57, 0xed, 0xaf, 0x5e, 0x37, 0x42, 0x6b, 0xe4, 0x91, 0xf1, 0x90, 0xa2, 0xa7,
    0x09, 0xb2, 0x3b, 0x32, 0x52, 0x98, 0x14, 0x71, 0x72, 0x64, 0x28, 0x68, 0x50, 0xf0, 0xb6, 0x78,
    0x2f, 0x28, 0xd4, 0x4e, 0x3d, 0x46, 0x5b, 0x67, 0x90, 0x45, 0x0c, 0xb4, 0x6f, 0x44, 0x5d, 0xb6,
    0x02, 0xaf, 0x3a, 0x0a, 0xd9, 0xf5, 0x51, 0xbe, 0xb0, 0x44, 0x42, 0x39, 0x89, 0xa5, 0xb8, 0x93,
    0x13, 0xb9, 0x46, 0x72, 0xd0, 0xd8, 0x9c, 0x7c, 0xf4, 0x5c, 0xdb, 0x9a, 0xaa, 0x45, 0x3a, 0xbe,
    0x13, 0xbf, 0x2e, 0x10, 0x54, 0x22, 0xd0, 0xc8, 0xa2, 0x6c, 0x62, 0x48, 0x59, 0xa2, 0x06, 0x69,
    0x8c, 0xaa, 0x6d, 0x67, 0x66, 0x05, 0x71, 0xac, 0x7e, 0x5b, 0x4b, 0x4d, 0x7a, 0x34, 0xae, 0x87,
    0x58, 0x9c, 0x15, 0x1b, 0x1b, 0x95, 0xdc, 0x2a, 0xec, 0xad, 0x2d, 0x69, 0x30, 0x06, 0x0b, 0x6f,
    0x4c, 0x80, 0x46, 0x6b, 0x95, 0x5a, 0x38, 0x52, 0xf0, 0x6e, 0x6a, 0x7b, 0x6c, 0x28, 0x7a, 0xf6,
    0x1f, 0x03, 0xd5, 0x31, 0xef, 0xec, 0x9f, 0x15, 0x3e, 0x19, 0x56, 0x14, 0xfb, 0x21, 0x63, 0x27,
    0x31, 0x5e, 0x70, 0xc4, 0x58, 0x65, 0x11, 0x3c, 0xac, 0xa0, 0xcd, 0x3e, 0xf3, 0x6c, 0x05, 0x9d,
    0x67, 0x14, 0xe8, 0x2a, 0x8e, 0xf6, 0xf5, 0x85, 0x36, 0xb6, 0x5b, 0xad, 0x43, 0xb1, 0x06, 0xd7,
    0xb3, 0x40, 0x16, 0x6f, 0x22, 0xde, 0x5c, 0x30, 0xeb, 0x3a, 0x54, 0x02, 0xa4, 0x68, 0x26, 0x96,
    0xdf, 0x69, 0xb3, 0xf9, 0x3f, 0x43, 0xa0, 0x1c, 0xdc, 0xc6, 0x62, 0x30, 0xa4, 0x1d, 0x64, 0x07,
    0xf3, 0xc2, 0x07, 0xdc, 0x52, 0xd7, 0x07, 0xa2, 0xc1, 0xbe, 0xd1, 0xbc, 0xbc, 0x36, 0xad, 0xc4,
    0x63, 0x72, 0x47, 0x7f, 0xf3, 0xb3, 0xcf, 0x57, 0x2f, 0xdc, 0x1c, 0x77, 0xc6, 0x1e, 0x93, 0x81,
    0xcd, 0x38, 0xf5, 0x57, 0x6a, 0xfc, 0x05, 0x50, 0x4b, 0x03, 0x04, 0x14, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x21, 0x5c, 0xb1, 0x5d, 0xbd, 0x5a, 0x09, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00,
    0x00, 0x11, 0x00, 0x00, 0x00, 0x73, 0x68, 0x61, 0x72, 0x65, 0x2f, 0x64, 0x6f, 0x63, 0x2f, 0x72,
    0x61, 0x77, 0x2e, 0x62, 0x69, 0x6e, 0x00, 0x01, 0x02, 0x73, 0x74, 0x6f, 0x72, 0x65, 0x64, 0x50,
    0x4b, 0x01, 0x02, 0x14, 0x00, 0x14, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x21, 0x5c, 0x7d,
    0x16, 0xaa, 0x3f, 0x46, 0x02, 0x00, 0x00, 0xe4, 0x10, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x73, 0x68, 0x61,
    0x72, 0x65, 0x2f, 0x64, 0x6f, 0x63, 0x2f, 0x6e, 0x6f, 0x74, 0x65, 0x73, 0x2e, 0x74, 0x78, 0x74,
    0x50, 0x4b, 0x01, 0x02, 0x14, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x5c,
    0xb1, 0x5d, 0xbd, 0x5a, 0x09, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x77, 0x02, 0x00, 0x00, 0x73, 0x68,
    0x61, 0x72, 0x65, 0x2f, 0x64, 0x6f, 0x63, 0x2f, 0x72, 0x61, 0x77, 0x2e, 0x62, 0x69, 0x6e, 0x50,
    0x4b, 0x05, 0x06, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x80, 0x00, 0x00, 0x00, 0xaf,
    0x02, 0x00, 0x00, 0x00, 0x00,
};

std::string bytes_of(const unsigned char *p, std::size_t n) {
  return std::string(reinterpret_cast<const char *>(p), n);
}

// A fresh directory under the system's temp directory, removed afterwards.
struct TempDir {
  fs::path dir;
  TempDir() {
    std::random_device rd;
    dir = fs::temp_directory_path() /
          ("cvc-data-test-" + std::to_string(rd()) + std::to_string(rd()));
    fs::create_directories(dir);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(dir, ec);
  }
  std::string put(const std::string &rel, const std::string &bytes) const {
    const fs::path p = dir / rel;
    fs::create_directories(p.parent_path());
    std::ofstream(p, std::ios::binary)
        .write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return p.generic_string();
  }
};

// A tiny city bundle: a 3 x 3 terrain over [-10, 10]^2, no buildings.
const char *kTerrainJson = R"({"bounds": {"min_x": -10, "min_y": -10, "max_x": 10, "max_y": 10},
  "rows": 3, "cols": 3, "grid": [[2, 2, 2], [1, 1, 1], [0, 0, 0]]})";

std::string status_of(cvc::app &app, const std::string &path) {
  return cvc::ariadne::data_status(app, path);
}

} // namespace

// ── the archive reader ───────────────────────────────────────────────────────────────────────────

TEST(Archive, TarRoundTripsItsFilesAndLongNames) {
  const std::string long_name = std::string(120, 'a') + "/b.txt";
  const std::string tar = ar::pack_tar({{"x/one.txt", "one"}, {long_name, "two"}, {"empty", ""}});
  EXPECT_EQ(ar::sniff(tar), ar::format::tar);
  const auto files = ar::unpack(tar);
  ASSERT_EQ(files.size(), 3u);
  EXPECT_EQ(files[0].path, "x/one.txt");
  EXPECT_EQ(*files[0].bytes, "one");
  EXPECT_EQ(files[1].path, long_name);
  EXPECT_EQ(*files[1].bytes, "two");
  EXPECT_EQ(files[2].bytes->size(), 0u);
}

TEST(Archive, KeepChoosesTheFilesAndTheRestAreSkipped) {
  const std::string tar = ar::pack_tar({{"a/1", "1"}, {"b/2", "2"}, {"a/3", "3"}});
  const auto files =
      ar::unpack(tar, {}, [](const std::string &p) { return p.rfind("a/", 0) == 0; });
  ASSERT_EQ(files.size(), 2u);
  EXPECT_EQ(files[1].path, "a/3");
}

TEST(Archive, GzipAndZstdTarsAreRecognizedByTheirBytes) {
  const std::string tar = ar::pack_tar({{"share/x.json", "{}"}});
  for (const std::string &packed : {ar::gzip(tar), ar::zstd(tar)}) {
    EXPECT_EQ(ar::classify(packed), ar::kind::tar);
    const auto files = ar::unpack(packed);
    ASSERT_EQ(files.size(), 1u);
    EXPECT_EQ(files[0].path, "share/x.json");
  }
  EXPECT_EQ(ar::classify(ar::gzip("just text")), ar::kind::compressed);
  EXPECT_EQ(ar::decompress(ar::gzip("just text")), "just text");
  EXPECT_EQ(ar::classify("plain"), ar::kind::plain);
  EXPECT_THROW(ar::unpack("plain"), std::runtime_error);
}

TEST(Archive, InflateDecodesDynamicAndFixedHuffmanBlocks) {
  const std::string dyn = ar::decompress(bytes_of(kGzipDynamic, sizeof kGzipDynamic));
  EXPECT_EQ(dyn.size(), 4324u);
  EXPECT_EQ(ar::crc32(dyn), 0x3faa167du);
  EXPECT_EQ(ar::decompress(bytes_of(kGzipFixed, sizeof kGzipFixed)),
            "hello hello hello, ariadne data");
  // A larger stream of stored blocks (more than one 64 KiB block, and past the window trim).
  std::string big(9u << 20, '\0');
  for (std::size_t i = 0; i < big.size(); ++i)
    big[i] = static_cast<char>((i * 2654435761u) >> 13);
  EXPECT_EQ(ar::decompress(ar::gzip(big)), big);
}

TEST(Archive, CorruptAndOversizedDataIsRefused) {
  std::string gz = ar::gzip("some bytes to check");
  gz[gz.size() - 9] ^= 1; // a data byte: the CRC no longer matches
  EXPECT_THROW(ar::decompress(gz), std::runtime_error);
  EXPECT_THROW(ar::decompress(ar::gzip(std::string(4096, 'x')), 1000), std::runtime_error);
  ar::limits lim;
  lim.max_entries = 1;
  EXPECT_THROW(ar::unpack(ar::pack_tar({{"a", "1"}, {"b", "2"}}), lim), std::runtime_error);
  lim = ar::limits{};
  lim.max_bytes = 3;
  EXPECT_THROW(ar::unpack(ar::pack_tar({{"a", "1234"}}), lim), std::runtime_error);
}

TEST(Archive, UnsafePathsAreRefused) {
  EXPECT_EQ(ar::safe_path("./a//b/./c"), "a/b/c");
  EXPECT_EQ(ar::safe_path("a\\b"), "a/b");
  EXPECT_EQ(ar::safe_path("/etc/passwd"), "");
  EXPECT_EQ(ar::safe_path("a/../../b"), "");
  EXPECT_EQ(ar::safe_path("C:/Windows"), "");
  EXPECT_THROW(ar::unpack(ar::pack_tar({{"../escape", "x"}})), std::runtime_error);
}

TEST(Archive, ZipStoredAndDeflatedEntries) {
  const std::string zip = bytes_of(kZip, sizeof kZip);
  EXPECT_EQ(ar::classify(zip), ar::kind::zip);
  const auto files = ar::unpack(zip);
  ASSERT_EQ(files.size(), 2u);
  EXPECT_EQ(files[0].path, "share/doc/notes.txt");
  EXPECT_EQ(files[0].bytes->size(), 4324u);
  EXPECT_EQ(ar::crc32(*files[0].bytes), 0x3faa167du);
  EXPECT_EQ(files[1].path, "share/doc/raw.bin");
  EXPECT_EQ(*files[1].bytes, std::string("\x00\x01\x02stored", 9));
}

// pack_zip (a saved world, GB4): what it packs unpacks to the same files in the same order, a
// file that packs small deflated and one that does not stored, the same files to the same bytes.
TEST(Archive, PackZipRoundTripsDeflatedAndStoredEntries) {
  std::string text;
  for (int i = 0; i < 400; ++i)
    text += "row " + std::to_string(i % 7) + " of a height map\n";
  std::string noise(3000, '\0');
  std::uint32_t s = 12345;
  for (char &c : noise) {
    s = s * 1664525u + 1013904223u;
    c = static_cast<char>(s >> 24);
  }
  const std::vector<std::pair<std::string, std::string>> in = {
      {"world.json", "{}\n"}, {"export/terrain.txt", text}, {"noise.bin", noise}, {"empty", ""}};
  const std::string zip = ar::pack_zip(in);
  EXPECT_EQ(ar::classify(zip), ar::kind::zip);
  EXPECT_EQ(ar::pack_zip(in), zip);
  EXPECT_LT(zip.size(), text.size()); // the text deflated
  const auto out = ar::unpack(zip);
  ASSERT_EQ(out.size(), in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    EXPECT_EQ(out[i].path, in[i].first);
    EXPECT_EQ(*out[i].bytes, in[i].second) << in[i].first;
  }
  // The deflate stream alone, inflated by the gzip reader around it.
  const std::string raw = ar::deflate(text);
  std::string gz("\x1F\x8B\x08\x00\x00\x00\x00\x00\x00\xFF", 10);
  gz += raw;
  for (std::uint32_t v : {ar::crc32(text), static_cast<std::uint32_t>(text.size())})
    for (int k = 0; k < 4; ++k)
      gz += static_cast<char>((v >> (8 * k)) & 0xFF);
  EXPECT_EQ(ar::decompress(gz), text);

  EXPECT_THROW(ar::pack_zip({{"../out", "x"}}), std::invalid_argument);
  EXPECT_THROW(ar::pack_zip({{"/abs", "x"}}), std::invalid_argument);
  EXPECT_THROW(ar::pack_zip({{"a", "x"}, {"a", "y"}}), std::invalid_argument);
}

// encode_png keeps a grey u16 image at 16 bits (a world's export/heightmap.png): the header says
// 16-bit grey, the rows inflate to the Sub-filtered big-endian samples, and an 8-bit reader reads
// their high bytes.
TEST(ImageEncode, AGrey16PngKeepsItsSamples) {
  const int w = 5, h = 3;
  const std::vector<std::uint16_t> px = {0,    1,    255,   256,   65535, 1000,  2000, 3000,
                                         4000, 5000, 40000, 30000, 20000, 10000, 0};
  const cvc::image img(w, h, cvc::image::pixel_format::GRAY, cvc::image::data_type::u16, px.data());
  const std::string png = cvc::encode_png(img);
  ASSERT_GT(png.size(), 33u);
  EXPECT_EQ(png.substr(0, 8), std::string("\x89PNG\r\n\x1A\n", 8));
  EXPECT_EQ(png.substr(12, 4), "IHDR");
  EXPECT_EQ(static_cast<unsigned char>(png[24]), 16u); // bit depth
  EXPECT_EQ(static_cast<unsigned char>(png[25]), 0u);  // grey
  // The IDAT's zlib stream, as a gzip of what the rows should be.
  const std::size_t at = png.find("IDAT");
  ASSERT_NE(at, std::string::npos);
  const std::size_t len = (std::size_t(static_cast<unsigned char>(png[at - 4])) << 24) |
                          (std::size_t(static_cast<unsigned char>(png[at - 3])) << 16) |
                          (std::size_t(static_cast<unsigned char>(png[at - 2])) << 8) |
                          std::size_t(static_cast<unsigned char>(png[at - 1]));
  const std::string z = png.substr(at + 4, len);
  std::string rows;
  for (int y = 0; y < h; ++y) {
    rows += '\x01';
    std::uint16_t prev = 0;
    for (int x = 0; x < w; ++x) {
      const std::uint16_t v = px[std::size_t(y) * w + x];
      rows += static_cast<char>(((v >> 8) - (prev >> 8)) & 0xFF);
      rows += static_cast<char>((v - prev) & 0xFF);
      prev = v;
    }
  }
  std::string gz("\x1F\x8B\x08\x00\x00\x00\x00\x00\x00\xFF", 10);
  gz += z.substr(2, z.size() - 6);
  for (std::uint32_t v : {ar::crc32(rows), static_cast<std::uint32_t>(rows.size())})
    for (int k = 0; k < 4; ++k)
      gz += static_cast<char>((v >> (8 * k)) & 0xFF);
  EXPECT_EQ(ar::decompress(gz), rows);
  std::string why;
  const cvc::image back = cvc::decode_image(png.data(), png.size(), &why);
  ASSERT_EQ(back.width(), w) << why;
  ASSERT_EQ(back.height(), h);
  const auto *rgba = static_cast<const unsigned char *>(back.data());
  for (int i = 0; i < w * h; ++i)
    EXPECT_EQ(rgba[i * 4], px[i] >> 8) << i;
}

// ── the store ────────────────────────────────────────────────────────────────────────────────────

TEST(AriadneData, ADirectoryLoadsItsFilesIntoTheTree) {
  cvc::app app;
  TempDir t;
  t.put("set/a.txt", "alpha");
  t.put("set/sub/b.bin", std::string("\x00\x01", 2));
  auto store = DataStore::of(app);
  DataSpec spec;
  spec.uri = (t.dir / "set").generic_string();
  const std::string node = store->load(spec);
  ASSERT_TRUE(store->wait(node, 30)) << cvc::state::instance(app)(node + ".error").value();
  EXPECT_EQ(cvc::ariadne::data_files(app, node), (std::vector<std::string>{"a.txt", "sub/b.bin"}));
  const cvc::ariadne::DataBytes a = cvc::ariadne::data_file(app, node, "a.txt");
  ASSERT_TRUE(a);
  EXPECT_EQ(*a, "alpha");
  EXPECT_EQ(cvc::state::instance(app)(node + ".as").value(), "files");
  EXPECT_EQ(cvc::state::instance(app)(node + ".rev").value(), "1");
  // The same request again is the same node, and it is not loaded twice.
  EXPECT_EQ(store->load(spec), node);
  EXPECT_EQ(cvc::state::instance(app)(node + ".rev").value(), "1");
}

TEST(AriadneData, AnArchiveFilesPathKeepsOneDirectoryAndDecodesIt) {
  cvc::app app;
  TempDir t;
  const std::string tar =
      ar::pack_tar({{"share/libcvc-deps/x/manifest.yaml", "m"},
                    {"share/cvc-scenes/tiny/terrain.json", kTerrainJson},
                    {"share/cvc-scenes/tiny/notes.json", R"({"a.b": [1, 2.5, true], "s": "x"})"}});
  const std::string file = t.put("tiny.tar.zst", ar::gzip(tar)); // named zst, holding gzip
  auto store = DataStore::of(app);

  DataSpec city;
  city.uri = file;
  city.path = "share/cvc-scenes/tiny";
  city.as = "city_bundle";
  const std::string node = store->load(city);
  ASSERT_TRUE(store->wait(node, 30)) << cvc::state::instance(app)(node + ".error").value();
  EXPECT_EQ(cvc::ariadne::data_files(app, node),
            (std::vector<std::string>{"terrain.json", "notes.json"}));
  auto world = boost::any_cast<std::shared_ptr<const cvc::world::world_geometry>>(
      cvc::state::instance(app)(node + ".world").data());
  ASSERT_TRUE(world && world->terrain());
  EXPECT_EQ(world->terrain()->rows(), 3);
  auto terrain = boost::any_cast<std::shared_ptr<const cvc::height_field>>(
      cvc::state::instance(app)(node + ".terrain").data());
  EXPECT_EQ(terrain, world->terrain()); // one object, not a copy
  EXPECT_EQ(cvc::state::instance(app)(node + ".bounds.max_x").value<double>(), 10.0);
  EXPECT_FALSE(cvc::state::instance(app)(node + ".buildings_error").value().empty());

  DataSpec notes;
  notes.uri = file;
  notes.path = "share/cvc-scenes/tiny/notes.json";
  const std::string jnode = store->load(notes);
  ASSERT_TRUE(store->wait(jnode, 30)) << cvc::state::instance(app)(jnode + ".error").value();
  EXPECT_EQ(cvc::state::instance(app)(jnode + ".as").value(), "json");
  EXPECT_EQ(cvc::state::instance(app)(jnode + ".json.a_b").value(), "3");
  EXPECT_EQ(cvc::state::instance(app)(jnode + ".json.a_b.1").value(), "2.5");
  EXPECT_EQ(cvc::state::instance(app)(jnode + ".json.a_b.2").value(), "1");
  EXPECT_EQ(cvc::state::instance(app)(jnode + ".json.s").value(), "x");
}

TEST(AriadneData, AMeshDecodesFromItsBytesInMemory) {
  if (!cvc::can_read_model_from_memory(".obj"))
    GTEST_SKIP() << "libcvc built without Assimp";
  cvc::app app;
  TempDir t;
  const std::string file = t.put(
      "mesh.tgz", ar::gzip(ar::pack_tar({{"m/tri.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"}})));
  auto store = DataStore::of(app);
  DataSpec spec;
  spec.uri = file;
  spec.path = "m/tri.obj"; // one file: as: mesh, from its extension
  const std::string node = store->load(spec);
  ASSERT_TRUE(store->wait(node, 30)) << cvc::state::instance(app)(node + ".error").value();
  EXPECT_EQ(cvc::state::instance(app)(node + ".as").value(), "mesh");
  auto g = boost::any_cast<std::shared_ptr<const cvc::geometry>>(
      cvc::state::instance(app)(node + ".geometry").data());
  ASSERT_TRUE(g);
  EXPECT_EQ(g->num_points(), 3u);
  const std::string obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
  EXPECT_EQ(cvc::read_model_from_memory(app, obj.data(), obj.size(), "obj").merged().num_points(),
            3u);
  EXPECT_THROW(cvc::read_model_from_memory(app, "x", 1, ".nope"), std::exception);
}

// A node that edits data reads and writes no file: the app loads its data (the point_classes and
// volume_grid_session decoders) and saves them (data_save, through an encoder), and a load of what
// was saved is what was saved.
TEST(AriadneData, PointClassesAndSessionsLoadAndSaveThroughTheApp) {
  cvc::app app;
  TempDir t;
  auto store = DataStore::of(app);
  cvc::state &root = cvc::state::instance(app);
  cvc::point_classes pc;
  pc.add("membrane", 1, 2, 3);
  pc.add("membrane", 2, 2, 3);
  pc.add("background", 7, 7, 1);
  root("work.points").data(boost::any(std::make_shared<const cvc::point_classes>(pc)));
  EXPECT_EQ(cvc::ariadne::default_data_encoder(root("work.points").data()), "point_classes");
  const std::string vgr = (t.dir / "seeds.vgr").generic_string();
  cvc::ariadne::data_save(app, "work.points", "seeds.vgr", "", t.dir.generic_string());
  ASSERT_TRUE(fs::exists(vgr));
  DataSpec spec;
  spec.uri = vgr; // as: point_classes, by its extension
  const std::string node = store->load(spec);
  ASSERT_TRUE(store->wait(node, 30)) << root(node + ".error").value();
  EXPECT_EQ(root(node + ".as").value(), "point_classes");
  auto back =
      boost::any_cast<std::shared_ptr<const cvc::point_classes>>(root(node + ".classes").data());
  ASSERT_TRUE(back);
  EXPECT_EQ(*back, pc);
  EXPECT_EQ(root(node + ".classes").value(), "3");

  cvc::volume_grid_session s;
  s.keys = {{"grid.view", "zy"}, {"grid.slice.z", "4"}};
  s.transfer_function = "{\"color\":[[0,0,0,0],[1,1,1,1]]}";
  s.classes = pc;
  root("work.session").data(boost::any(std::make_shared<const cvc::volume_grid_session>(s)));
  const std::string json = (t.dir / "grid.session.json").generic_string();
  cvc::ariadne::data_save(app, "work.session", json); // an absolute file
  DataSpec sspec;
  sspec.uri = json;
  sspec.as = "volume_grid_session"; // .json alone is json
  const std::string snode = store->load(sspec);
  ASSERT_TRUE(store->wait(snode, 30)) << root(snode + ".error").value();
  auto sback = boost::any_cast<std::shared_ptr<const cvc::volume_grid_session>>(
      root(snode + ".session").data());
  ASSERT_TRUE(sback);
  EXPECT_EQ(sback->keys, s.keys);
  EXPECT_EQ(sback->transfer_function, s.transfer_function);
  EXPECT_EQ(sback->classes, pc);

  // A volume, in a format by its extension; bytes; and the errors.
  cvc::volume v(app, cvc::dimension(4, 3, 2), cvc::Float);
  v(1, 2, 1, 7.5);
  root("work.volume").data(boost::any(std::make_shared<const cvc::volume>(v)));
  const std::string rawiv = (t.dir / "v.rawiv").generic_string();
  cvc::ariadne::data_save(app, "work.volume", rawiv);
  cvc::volume vback(app, rawiv);
  EXPECT_EQ(vback.XDim(), 4u);
  EXPECT_EQ(vback.ZDim(), 2u);
  EXPECT_DOUBLE_EQ(vback(1, 2, 1), 7.5);
  // A list of volumes: one file's variables (a volume_grid's segmented classes, as
  // VolumeGridRover's SegTmp.rawv), named after each volume.
  cvc::volume w(app, cvc::dimension(4, 3, 2), cvc::Float);
  w(0, 0, 0, 2.0);
  v.desc("left");
  w.desc("right");
  root("work.classes")
      .data(boost::any(
          std::make_shared<const std::vector<cvc::volume>>(std::vector<cvc::volume>{v, w})));
  EXPECT_EQ(cvc::ariadne::default_data_encoder(root("work.classes").data()), "volume");
  const std::string rawv = (t.dir / "segments.rawv").generic_string();
  cvc::ariadne::data_save(app, "work.classes", rawv, "volume");
  const cvc::volume_file_info info(app, rawv);
  EXPECT_EQ(info.numVariables(), 2u);
  EXPECT_EQ(info.name(0), "left");
  EXPECT_EQ(info.name(1), "right");
  cvc::volume second(app);
  cvc::readVolumeFile(app, second, rawv, 1, 0);
  EXPECT_DOUBLE_EQ(second(0, 0, 0), 2.0);
  root("work.empty").data(boost::any(std::make_shared<const std::vector<cvc::volume>>()));
  root("work.text").data(boost::any(std::string("hello")));
  cvc::ariadne::data_save(app, "work.text", (t.dir / "t.txt").generic_string());
  std::ifstream tin(t.dir / "t.txt");
  std::string text;
  std::getline(tin, text);
  EXPECT_EQ(text, "hello");
  const auto says = [&](const std::string &path, const std::string &file, const std::string &as,
                        const std::string &base, const std::string &want) {
    try {
      cvc::ariadne::data_save(app, path, file, as, base);
    } catch (const std::runtime_error &e) {
      EXPECT_NE(std::string(e.what()).find(want), std::string::npos) << e.what();
      return;
    }
    ADD_FAILURE() << "no error for " << path << " (" << want << ")";
  };
  says("work.none", "x.vgr", "", t.dir.generic_string(), "holds no data");
  root("work.odd").data(boost::any(42));
  says("work.odd", "x.bin", "", t.dir.generic_string(), "no built-in encoder takes");
  says("work.empty", "x.rawv", "volume", t.dir.generic_string(), "the list of volumes is empty");
  says("work.points", "x.vgr", "nope", t.dir.generic_string(), "no encoder as: nope");
  says("work.points", "x.vgr", "", "https://example.org/docs", "not a local file");
  EXPECT_FALSE(fs::exists(t.dir / "x.vgr"));

  // Encoders are per app, each starting with the built-ins.
  cvc::app other;
  cvc::ariadne::register_data_encoder(
      other, "mine", [](const cvc::ariadne::DataEncode &) { return std::string("m"); });
  EXPECT_TRUE(cvc::ariadne::has_data_encoder(other, "mine"));
  EXPECT_FALSE(cvc::ariadne::has_data_encoder(app, "mine"));
  for (const char *b : {"bytes", "volume", "point_classes", "volume_grid_session"})
    EXPECT_TRUE(cvc::ariadne::has_data_encoder(app, b) && cvc::ariadne::has_data_encoder(other, b))
        << b;
  cvc::ariadne::register_data_encoder(other, "point_classes", nullptr); // removed from one app
  EXPECT_FALSE(cvc::ariadne::has_data_encoder(other, "point_classes"));
  EXPECT_TRUE(cvc::ariadne::has_data_encoder(app, "point_classes"));
}

// A contour stack (a volume_grid's grid.contours, or contour_lab's) saved through the app in a
// one-file format and loaded back by its extension; a format that writes files beside it is
// refused, saying which to use.
TEST(AriadneData, ContoursLoadAndSaveThroughTheApp) {
  cvc::app app;
  TempDir t;
  auto store = DataStore::of(app);
  cvc::state &root = cvc::state::instance(app);
  cvc::contour_stack s;
  s.z_spacing = 0.5;
  s.add(3, "axon", {0, 0, 1, 0, 1, 1, 0, 1});
  s.add(4, "axon", {0, 0, 2, 0, 1, 1});
  root("work.contours").data(boost::any(std::make_shared<const cvc::contour_stack>(s)));
  EXPECT_EQ(cvc::ariadne::default_data_encoder(root("work.contours").data()), "contours");
  for (const char *name : {"c.cvcc", "c.contour"}) {
    cvc::ariadne::data_save(app, "work.contours", name, "", t.dir.generic_string());
    ASSERT_TRUE(fs::exists(t.dir / name)) << name;
    DataSpec spec;
    spec.uri = (t.dir / name).generic_string(); // as: contours, by its extension
    const std::string node = store->load(spec);
    ASSERT_TRUE(store->wait(node, 30)) << root(node + ".error").value();
    EXPECT_EQ(root(node + ".as").value(), "contours");
    auto back =
        boost::any_cast<std::shared_ptr<const cvc::contour_stack>>(root(node + ".contours").data());
    ASSERT_TRUE(back) << name;
    EXPECT_EQ(back->polygon_count(), 2u) << name;
    EXPECT_EQ(root(node + ".contours").value(), "2");
    if (std::string(name) == "c.cvcc")
      EXPECT_EQ(*back, s) << "libcvc's own format keeps everything";
  }
  try {
    cvc::ariadne::data_save(app, "work.contours", "c.ser", "", t.dir.generic_string());
    ADD_FAILURE() << "a .ser saved as one file";
  } catch (const std::exception &e) {
    EXPECT_NE(std::string(e.what()).find("save as .cvcc"), std::string::npos) << e.what();
  }
}

// A molecule (cvc/mol/molecule.h) loads by its extension, decoded `as: molecule` at `molecule`,
// whose value is its atom count; a file with no atoms fails the load, saying so.
TEST(AriadneData, MoleculesLoadByTheirExtension) {
  cvc::app app;
  TempDir t;
  auto store = DataStore::of(app);
  cvc::state &root = cvc::state::instance(app);
  std::ofstream(t.dir / "two.pdb")
      << "ATOM      1  N   ALA A   1      11.104   6.134  -6.504  1.00  0.00           N  \n"
         "ATOM      2  CA  ALA A   1      11.639   6.071  -5.147  1.00  0.00           C  \n"
         "END\n";
  std::ofstream(t.dir / "three.xyzr") << "0 0 0 1.5\n1 0 0 1.5\n2 0 0 1.7\n";
  std::ofstream(t.dir / "none.pdb") << "REMARK nothing here\nEND\n";
  for (const auto &[name, atoms] :
       std::vector<std::pair<std::string, std::size_t>>{{"two.pdb", 2}, {"three.xyzr", 3}}) {
    DataSpec spec;
    spec.uri = (t.dir / name).generic_string();
    const std::string node = store->load(spec);
    ASSERT_TRUE(store->wait(node, 30)) << root(node + ".error").value();
    EXPECT_EQ(root(node + ".as").value(), "molecule") << name;
    auto m =
        boost::any_cast<std::shared_ptr<const cvc::mol::molecule>>(root(node + ".molecule").data());
    ASSERT_TRUE(m) << name;
    EXPECT_EQ(m->atoms.size(), atoms) << name;
    EXPECT_EQ(root(node + ".molecule").value(), std::to_string(atoms)) << name;
  }
  DataSpec spec;
  spec.uri = (t.dir / "none.pdb").generic_string();
  const std::string node = store->load(spec);
  EXPECT_FALSE(store->wait(node, 30));
  EXPECT_EQ(status_of(app, node), "error");
  EXPECT_NE(root(node + ".error").value().find("holds no atoms"), std::string::npos)
      << root(node + ".error").value();
  EXPECT_TRUE(cvc::ariadne::has_data_decoder(app, "molecule"));
}

// A generator that makes no files and names no as: has written its outputs: no decoder runs.
TEST(AriadneData, AGeneratorThatWritesItsOutputsNeedsNoDecoder) {
  cvc::app app;
  cvc::ariadne::register_data_generator(app, "made", [](cvc::ariadne::DataDecode &d) {
    d.out("answer").value(std::string("42"));
    return std::vector<cvc::archive::file>();
  });
  auto store = DataStore::of(app);
  DataSpec spec;
  spec.name = "m";
  spec.generate = "made";
  store->bind("", spec);
  ASSERT_TRUE(store->wait("data.m", 30)) << cvc::state::instance(app)("data.m.error").value();
  cvc::state *answer = cvc::state::instance(app).findThroughLinks("data.m.answer");
  ASSERT_NE(answer, nullptr);
  EXPECT_EQ(answer->value(), "42");
}

TEST(AriadneData, AFailedLoadSaysWhy) {
  cvc::app app;
  auto store = DataStore::of(app);
  DataSpec missing;
  missing.uri = "no/such/dir/here.tar.gz";
  const std::string node = store->load(missing);
  EXPECT_FALSE(store->wait(node, 30));
  EXPECT_EQ(status_of(app, node), "error");
  EXPECT_NE(cvc::state::instance(app)(node + ".error").value().find("here.tar.gz"),
            std::string::npos);

  DataSpec offline; // no network: the host never turned it on, and this store was given none
  offline.uri = "https://example.org/x.tar.gz";
  const std::string onode = store->load(offline);
  EXPECT_FALSE(store->wait(onode, 30));
  EXPECT_NE(cvc::state::instance(app)(onode + ".error").value().find("network is off"),
            std::string::npos);
}

TEST(AriadneData, ADocumentsEntryIsALinkAndALazyOneLoadsWhenAsked) {
  cvc::app app;
  TempDir t;
  t.put("w/a.txt", "a");
  auto store = DataStore::of(app);
  DataSpec spec;
  spec.name = "set";
  spec.uri = (t.dir / "w").generic_string();
  spec.lazy = true;
  const std::string node = store->bind("ui.docs.ari", spec);
  EXPECT_EQ(status_of(app, "ui.docs.ari.data.set"), "idle");
  EXPECT_EQ(store->resolve("ui.docs.ari.data.set"), node);
  ASSERT_TRUE(store->wait("ui.docs.ari.data.set", 30)); // waiting asks for it
  EXPECT_EQ(status_of(app, "ui.docs.ari.data.set"), "ready");
  // A second document naming the same data shares the node.
  DataSpec again = spec;
  again.lazy = false;
  EXPECT_EQ(store->bind("ui.docs.other", again), node);
  const cvc::ariadne::DataBytes a = cvc::ariadne::data_file(app, "ui.docs.other.data.set", "a.txt");
  ASSERT_TRUE(a);
  EXPECT_EQ(*a, "a");
}

// A registry and download server in memory: /v1/packages/NAME/latest answers with the package row,
// /v1/download/FILE with the archive.
class FakeRegistry : public cvc::net::HttpClient {
public:
  std::string archive;
  std::atomic<int> downloads{0};
  std::string last_query;
  cvc::net::HttpResponse send(const cvc::net::HttpRequest &req) override {
    cvc::net::HttpResponse r;
    r.ok = true;
    if (req.url.find("/v1/packages/scene-tiny/latest") != std::string::npos) {
      last_query = req.url;
      r.status = 200;
      r.body =
          std::string(R"({"name": "scene-tiny", "version": "1.0.0+cvc.1", "sha256": ")") + sha +
          R"(", "size_bytes": )" + std::to_string(archive.size()) +
          R"(, "archive_url": "/v1/download/scene-tiny-1.0.0+cvc.1-any-noarch-none-none.tar.zst"})";
    } else if (req.url.find("/v1/download/scene-tiny-") != std::string::npos) {
      ++downloads;
      r.status = 200;
      r.body = archive;
    } else {
      r.status = 404;
    }
    return r;
  }
  std::string sha;
};

// An app's client that sends to one FakeRegistry the test keeps, so several apps (one after
// another) share the registry and its counts.
class ToRegistry : public cvc::net::HttpClient {
public:
  explicit ToRegistry(FakeRegistry &r) : r_(r) {}
  cvc::net::HttpResponse send(const cvc::net::HttpRequest &req) override { return r_.send(req); }

private:
  FakeRegistry &r_;
};

static void set_env(const char *name, const std::string &value) {
#ifdef _WIN32
  _putenv_s(name, value.c_str());
#else
  if (value.empty())
    unsetenv(name);
  else
    setenv(name, value.c_str(), 1);
#endif
}

TEST(AriadneData, ACvcpkgPackageIsFetchedCheckedCachedAndDecoded) {
  TempDir cache;
  set_env("CVC_DATA_CACHE", cache.dir.string());
  set_env("CVCPKG_URL", "https://registry.test/");
  auto fake = std::make_unique<FakeRegistry>(); // outlives every app (their loads end first)
  FakeRegistry *registry = fake.get();
  registry->archive =
      ar::gzip(ar::pack_tar({{"share/libcvc-deps/scene-tiny/manifest.yaml", "m"},
                             {"share/cvc-scenes/tiny/terrain.json", kTerrainJson}}));
  registry->sha = std::string(64, '0'); // not the archive's: refused
  DataSpec spec;
  spec.uri = "cvcpkg://cy-pca/scene-tiny";
  spec.path = "share/cvc-scenes/tiny";
  spec.as = "city_bundle";
  {
    cvc::app app;
    cvc::net::set_http_client(app, std::make_unique<ToRegistry>(*registry));
    auto store = DataStore::of(app);
    store->set_network(nullptr);
    const std::string node = store->load(spec);
    EXPECT_FALSE(store->wait(node, 30));
    EXPECT_NE(cvc::state::instance(app)(node + ".error").value().find("sha256"), std::string::npos);
  }
  registry->sha = cvc::sha256_hex(reinterpret_cast<const unsigned char *>(registry->archive.data()),
                                  registry->archive.size());
  registry->downloads = 0;
  for (int run = 0; run < 2; ++run) { // the second run reads the cache
    cvc::app app;
    cvc::net::set_http_client(app, std::make_unique<ToRegistry>(*registry));
    auto store = DataStore::of(app);
    store->set_network(nullptr);
    const std::string node = store->load(spec);
    ASSERT_TRUE(store->wait(node, 30)) << cvc::state::instance(app)(node + ".error").value();
    auto world = boost::any_cast<std::shared_ptr<const cvc::world::world_geometry>>(
        cvc::state::instance(app)(node + ".world").data());
    ASSERT_TRUE(world && world->terrain());
    EXPECT_EQ(world->terrain()->cols(), 3);
    EXPECT_EQ(cvc::state::instance(app)(node + ".canonical").value(),
              "cvcpkg://cy-pca/scene-tiny@1.0.0+cvc.1");
    EXPECT_EQ(cvc::state::instance(app)(node + ".sha256").value(), registry->sha);
  }
  EXPECT_EQ(registry->downloads.load(), 1);
  for (const char *part : {"https://registry.test/v1/packages/scene-tiny/latest?", "org=cy-pca",
                           "platform=any", "arch=noarch", "build_type=none", "link=none"})
    EXPECT_NE(registry->last_query.find(part), std::string::npos) << registry->last_query;
  set_env("CVC_DATA_CACHE", "");
  set_env("CVCPKG_URL", "");
}

// ── the loader ───────────────────────────────────────────────────────────────────────────────────

TEST(AriadneData, TheLoaderReadsTheDataBlockAndDataSources) {
  if (!cvc::ariadne::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  const auto r = fresh_load(R"(
meta: { name: data, min_libcvc: 3.4.0 }
data:
  city:
    uri: cvcpkg://cy-pca/scene-austin-south-small
    path: share/cvc-scenes/austin_south
    as: city_bundle
  notes: notes.json
  later: { uri: big.tar.gz, load: lazy }
  bad.name: x.json
  odd: { uri: y.json, as: no_such_decoder }
scene:
  nodes:
    - node: city
      source: { data: city }
    - node: blocks
      source: { data: city, item: buildings }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.data.size(), 4u);
  EXPECT_EQ(r.data[0].name, "city");
  EXPECT_EQ(r.data[0].uri, "cvcpkg://cy-pca/scene-austin-south-small");
  EXPECT_EQ(r.data[0].path, "share/cvc-scenes/austin_south");
  EXPECT_EQ(r.data[0].as, "city_bundle");
  EXPECT_FALSE(r.data[0].lazy);
  EXPECT_EQ(r.data[1].uri, "notes.json");
  EXPECT_TRUE(r.data[2].lazy);
  const auto warned = [&](const std::string &s) {
    for (const auto &w : r.warnings)
      if (w.find(s) != std::string::npos)
        return true;
    return false;
  };
  EXPECT_TRUE(warned("bad.name"));
  EXPECT_TRUE(warned("no_such_decoder"));
  ASSERT_EQ(r.scene.nodes.size(), 2u);
  EXPECT_EQ(r.scene.nodes[0].source_data, "city");
  EXPECT_TRUE(r.scene.nodes[0].source_item.empty());
  EXPECT_EQ(r.scene.nodes[1].source_item, "buildings");
}
