/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeRover2 ColorTable2 transfer-function files, read into / written from a cvc::tf::model.
//
// .vinay is plain text (not XML):
//   Anthony and Vinay are Great.
//   Alphamap / Number of nodes / <N> / Position and opacity / N x "<pos> <opacity>"
//   ColorMap / Number of nodes / <N> / Position and RGB     / N x "<pos> <r> <g> <b>"
//   IsocontourMap / Number of nodes / <N> / Position        / N x "<pos>"
// in any order, positions normalized to [0,1]. The reader is tolerant where the original threw or
// silently skipped (CRLF line ends, tabs and runs of spaces, trailing blanks, a wrong magic line);
// it reports those in `warnings`. The format stores no isocontour ids: they are named iso1, ...
// .ctbl is the baked table: "<size>" then size rows "r g b a" (bytes).
//
// Reader details: labels match case-insensitively with blank runs collapsed; a UTF-8 BOM, blank
// lines and stray lines between sections are skipped (stray ones warned); a missing label line
// warns. Isocontour ids go in ascending position (iso1 = lowest), and a repeated isocontour
// position is dropped with a warning, as ColorTable2's position-keyed set did. Input with neither
// the magic line nor any section is not a .vinay file and throws. Messages read
// "line N: ..." ("<path>, line N: ..." from read_vinay_file); a node list cut short reports
// "line N (end of file)". The model's domain, opacity_cubed and resolution are not in the format:
// they read back as the defaults. A section the file lacks is an empty list, as in ColorTable2,
// so normalize() makes it black->white / 0->1 (unlike from_string, which keeps default_model()'s).

#ifndef CVC_TF_VINAY_H
#define CVC_TF_VINAY_H

#include <cvc/core/exception.h>
#include <cvc/tf/model.h>
#include <iosfwd>
#include <string>
#include <vector>

namespace cvc {
namespace tf {

CVC_DEF_EXCEPTION(vinay_error);

// Parse .vinay text into a normalized model. Throws vinay_error (with the line number) on a
// structural error: a section header without a count, a short or malformed node line.
model read_vinay(std::istream &in, std::vector<std::string> *warnings = nullptr);
model read_vinay_string(const std::string &text, std::vector<std::string> *warnings = nullptr);
model read_vinay_file(const std::string &path, std::vector<std::string> *warnings = nullptr);

// Write `m` as .vinay (ColorTable2's layout: the first and last node of each section, then the
// interior ones; full double precision so a round trip is exact). Isocontour ids/colours are lost.
void write_vinay(std::ostream &out, const model &m);
std::string write_vinay_string(const model &m);
void write_vinay_file(const std::string &path, const model &m);

// The baked .ctbl table (`size` RGBA byte rows; ColorTable2 always wrote 256). It is the
// normalized model's bake_rgba8(size) (size < 2 is raised to 2) and honours opacity_cubed, where
// ColorTable2's writer hard-coded cubing.
std::string write_ctbl_string(const model &m, int size = 256);

} // namespace tf
} // namespace cvc

#endif // CVC_TF_VINAY_H
