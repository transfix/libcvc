/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Golden-image comparison for the cvcGL render tests: PNG in/out (VTK's PNG
// reader/writer, so no ImageMagick coder is involved), a per-channel tolerance
// plus a maximum mismatch fraction, and on failure an actual.png + diff.png for a
// person to look at. Images are held TOP-DOWN (row 0 is the top of the picture,
// as in a PNG file); SceneRenderer::frameRGB() is bottom-up, so go through
// fromFrameRGB().
//
//   CVC_UPDATE_GOLDEN=1   rewrite the golden from the actual frame instead of
//                         comparing (the caller decides whether that is allowed).
#ifndef CVCGL_TEST_GOLDEN_UTIL_H
#define CVCGL_TEST_GOLDEN_UTIL_H

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>
#include <vtkUnsignedCharArray.h>

namespace cvcgl_golden {

struct Image {
  int w = 0, h = 0;
  std::vector<unsigned char> rgb; // top-down, 3 bytes a pixel
  bool empty() const { return w <= 0 || h <= 0 || rgb.empty(); }
  const unsigned char *at(int x, int y) const { return &rgb[3 * (static_cast<size_t>(y) * w + x)]; }
};

// A SceneRenderer::frameRGB() buffer (bottom-up) as a top-down Image.
inline Image fromFrameRGB(const std::vector<unsigned char> &bottomUp, int w, int h) {
  Image img;
  if (w <= 0 || h <= 0 || bottomUp.size() < static_cast<size_t>(w) * h * 3)
    return img;
  img.w = w;
  img.h = h;
  img.rgb.resize(static_cast<size_t>(w) * h * 3);
  const size_t row = static_cast<size_t>(w) * 3;
  for (int y = 0; y < h; ++y)
    std::memcpy(&img.rgb[y * row], &bottomUp[(h - 1 - y) * row], row);
  return img;
}

inline bool envFlag(const char *name) {
  const char *v = std::getenv(name);
  return v && *v && std::string(v) != "0";
}

// Read a PNG as RGB (grey and alpha are expanded / dropped). False if unreadable.
inline bool readPNG(const std::string &path, Image &out) {
  out = Image();
  vtkNew<vtkPNGReader> reader;
  if (!reader->CanReadFile(path.c_str()))
    return false;
  reader->SetFileName(path.c_str());
  reader->Update();
  vtkImageData *id = reader->GetOutput();
  if (!id || id->GetScalarType() != VTK_UNSIGNED_CHAR)
    return false;
  int dims[3];
  id->GetDimensions(dims);
  const int nc = id->GetNumberOfScalarComponents();
  if (dims[0] <= 0 || dims[1] <= 0 || nc < 1)
    return false;
  const auto *src = static_cast<const unsigned char *>(id->GetScalarPointer());
  out.w = dims[0];
  out.h = dims[1];
  out.rgb.resize(static_cast<size_t>(out.w) * out.h * 3);
  // VTK's image row 0 is the BOTTOM of the picture.
  for (int y = 0; y < out.h; ++y)
    for (int x = 0; x < out.w; ++x) {
      const unsigned char *s = src + (static_cast<size_t>(out.h - 1 - y) * out.w + x) * nc;
      unsigned char *d = &out.rgb[(static_cast<size_t>(y) * out.w + x) * 3];
      d[0] = s[0];
      d[1] = nc >= 3 ? s[1] : s[0];
      d[2] = nc >= 3 ? s[2] : s[0];
    }
  return true;
}

inline bool writePNG(const std::string &path, const Image &img) {
  if (img.empty())
    return false;
  std::error_code ec;
  const std::filesystem::path parent = std::filesystem::path(path).parent_path();
  if (!parent.empty())
    std::filesystem::create_directories(parent, ec);
  vtkNew<vtkImageData> id;
  id->SetDimensions(img.w, img.h, 1);
  id->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
  auto *dst = static_cast<unsigned char *>(id->GetScalarPointer());
  const size_t row = static_cast<size_t>(img.w) * 3;
  for (int y = 0; y < img.h; ++y)
    std::memcpy(dst + (img.h - 1 - y) * row, &img.rgb[y * row], row);
  vtkNew<vtkPNGWriter> writer;
  writer->SetFileName(path.c_str());
  writer->SetInputData(id);
  writer->Write();
  return writer->GetErrorCode() == 0 && std::filesystem::exists(path, ec);
}

// How close is close enough: a pixel MISMATCHES when any channel differs by more
// than `channel`, and the comparison passes while the mismatched share of the
// pixels stays at or below `maxFraction` (0 = not one pixel).
struct Tolerance {
  int channel = 0;
  double maxFraction = 0.0;
};

struct Comparison {
  bool sizeMatch = false;
  long mismatched = 0; // pixels beyond the channel tolerance
  long total = 0;
  int maxDelta = 0;                     // the largest channel difference anywhere
  int x0 = 0, y0 = 0, x1 = -1, y1 = -1; // bounding box of the mismatches (top-down)
  // How the differences spread: pixels whose largest channel difference exceeds 0, 8, 24, 40. The
  // numbers to read when choosing a tolerance for another rasterizer.
  long over[4] = {0, 0, 0, 0};
  bool pass = false;
  double fraction() const { return total > 0 ? double(mismatched) / double(total) : 1.0; }
};
constexpr int kSpreadSteps[4] = {0, 8, 24, 40};

inline Comparison compare(const Image &actual, const Image &golden, const Tolerance &tol) {
  Comparison c;
  c.sizeMatch = actual.w == golden.w && actual.h == golden.h && !actual.empty();
  if (!c.sizeMatch)
    return c;
  c.total = static_cast<long>(actual.w) * actual.h;
  c.x0 = actual.w;
  c.y0 = actual.h;
  for (int y = 0; y < actual.h; ++y)
    for (int x = 0; x < actual.w; ++x) {
      const unsigned char *a = actual.at(x, y), *g = golden.at(x, y);
      int d = 0;
      for (int k = 0; k < 3; ++k)
        d = std::max(d, std::abs(int(a[k]) - int(g[k])));
      c.maxDelta = std::max(c.maxDelta, d);
      for (int s = 0; s < 4; ++s)
        c.over[s] += d > kSpreadSteps[s] ? 1 : 0;
      if (d > tol.channel) {
        ++c.mismatched;
        c.x0 = std::min(c.x0, x);
        c.y0 = std::min(c.y0, y);
        c.x1 = std::max(c.x1, x);
        c.y1 = std::max(c.y1, y);
      }
    }
  c.pass = c.fraction() <= tol.maxFraction;
  return c;
}

// The picture a person reads when a comparison fails: the golden faded to a dim
// grey for context, mismatched pixels in red (brighter = larger difference), and
// differences within the tolerance in dark blue, so "where" and "how much" are
// both visible at a glance.
inline Image diffImage(const Image &actual, const Image &golden, int channelTol) {
  Image d;
  if (actual.w != golden.w || actual.h != golden.h || actual.empty())
    return d;
  d.w = actual.w;
  d.h = actual.h;
  d.rgb.resize(actual.rgb.size());
  for (size_t i = 0; i + 2 < actual.rgb.size(); i += 3) {
    int delta = 0;
    for (int k = 0; k < 3; ++k)
      delta = std::max(delta, std::abs(int(actual.rgb[i + k]) - int(golden.rgb[i + k])));
    const int grey = (int(golden.rgb[i]) + golden.rgb[i + 1] + golden.rgb[i + 2]) / 3;
    unsigned char *p = &d.rgb[i];
    if (delta > channelTol) {
      p[0] = static_cast<unsigned char>(std::min(255, 128 + delta));
      p[1] = p[2] = 0;
    } else if (delta > 0) {
      p[0] = p[1] = 0;
      p[2] = 140;
    } else {
      p[0] = p[1] = p[2] = static_cast<unsigned char>(grey / 4);
    }
  }
  return d;
}

// One line saying what a comparison found.
inline std::string describe(const Comparison &c, const Tolerance &tol) {
  char buf[400];
  if (!c.sizeMatch) {
    std::snprintf(buf, sizeof buf, "size mismatch");
    return buf;
  }
  char spread[160];
  std::snprintf(spread, sizeof spread,
                "largest channel difference %d; px differing by >%d/>%d/>%d/>%d: %ld/%ld/%ld/%ld",
                c.maxDelta, kSpreadSteps[0], kSpreadSteps[1], kSpreadSteps[2], kSpreadSteps[3],
                c.over[0], c.over[1], c.over[2], c.over[3]);
  if (c.mismatched == 0) {
    std::snprintf(buf, sizeof buf, "no pixel beyond +-%d (%s)", tol.channel, spread);
    return buf;
  }
  std::snprintf(buf, sizeof buf,
                "%ld of %ld px beyond +-%d (%.4f%%, allowed %.4f%%), within x %d..%d y %d..%d "
                "(top-down); %s",
                c.mismatched, c.total, tol.channel, 100.0 * c.fraction(), 100.0 * tol.maxFraction,
                c.x0, c.x1, c.y0, c.y1, spread);
  return buf;
}

// Compare `actual` with the golden PNG at `goldenPath`. On a failed comparison
// write <outDir>/<name>.actual.png and <name>.diff.png and name them in `report`.
// With `update` true, write the golden instead and pass.
inline bool checkGolden(const std::string &name, const Image &actual, const std::string &goldenPath,
                        const std::string &outDir, const Tolerance &tol, bool update,
                        std::string &report) {
  if (update) {
    const bool ok = writePNG(goldenPath, actual);
    report = ok ? "updated " + goldenPath : "could not write " + goldenPath;
    return ok;
  }
  const std::string actualPath = (std::filesystem::path(outDir) / (name + ".actual.png")).string();
  const std::string diffPath = (std::filesystem::path(outDir) / (name + ".diff.png")).string();
  Image golden;
  if (!readPNG(goldenPath, golden)) {
    writePNG(actualPath, actual);
    report = "no readable golden " + goldenPath + " (wrote " + actualPath +
             "; run with CVC_UPDATE_GOLDEN=1 to create it)";
    return false;
  }
  const Comparison c = compare(actual, golden, tol);
  if (!c.sizeMatch) {
    writePNG(actualPath, actual);
    report = "size " + std::to_string(actual.w) + "x" + std::to_string(actual.h) + " vs golden " +
             std::to_string(golden.w) + "x" + std::to_string(golden.h) + " (wrote " + actualPath +
             ")";
    return false;
  }
  report = describe(c, tol);
  if (!c.pass) {
    writePNG(actualPath, actual);
    writePNG(diffPath, diffImage(actual, golden, tol.channel));
    report += "\n         golden " + goldenPath + "\n         actual " + actualPath +
              "\n         diff   " + diffPath;
  }
  return c.pass;
}

} // namespace cvcgl_golden

#endif
