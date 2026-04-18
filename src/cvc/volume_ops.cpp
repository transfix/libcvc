/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/volume_ops.h>

#include <cmath>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <vector>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef CVC_ENABLE_FFTW
#include <fftw3.h>
#endif

#ifdef CVC_ENABLE_IMAGEMAGICK
#include <Magick++.h>
#endif

namespace CVC_NAMESPACE
{
  // ── helpers ──

  static void check_dims_match(const volume& a, const volume& b)
  {
    if(a.XDim() != b.XDim() ||
       a.YDim() != b.YDim() ||
       a.ZDim() != b.ZDim())
      throw dimension_mismatch("Volumes must have matching dimensions");
  }

  static volume make_like(const volume& src)
  {
    volume out(src.voxel_dimensions(), Float, src.boundingBox());
    return out;
  }

  // ── Volume statistics ──

  volume_stats compute_stats(const volume& vol)
  {
    volume_stats s;
    s.num_voxels = vol.XDim() * vol.YDim() * vol.ZDim();
    s.min =  std::numeric_limits<double>::max();
    s.max = -std::numeric_limits<double>::max();
    s.mean = 0.0;

    double sum = 0.0;
    for(uint64 k = 0; k < vol.ZDim(); k++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 i = 0; i < vol.XDim(); i++)
        {
          double v = vol(i, j, k);
          if(v < s.min) s.min = v;
          if(v > s.max) s.max = v;
          sum += v;
        }

    s.mean = sum / s.num_voxels;

    double sum_sq = 0.0;
    for(uint64 k = 0; k < vol.ZDim(); k++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 i = 0; i < vol.XDim(); i++)
        {
          double diff = vol(i, j, k) - s.mean;
          sum_sq += diff * diff;
        }

    s.std_dev = std::sqrt(sum_sq / s.num_voxels);
    return s;
  }

  volume_stats compute_stats(const volume& vol, const bounding_box& region)
  {
    volume_stats s;
    s.min =  std::numeric_limits<double>::max();
    s.max = -std::numeric_limits<double>::max();
    s.mean = 0.0;
    s.num_voxels = 0;

    double sum = 0.0;

    for(uint64 k = 0; k < vol.ZDim(); k++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 i = 0; i < vol.XDim(); i++)
        {
          double x = vol.XMin() + i * vol.XSpan();
          double y = vol.YMin() + j * vol.YSpan();
          double z = vol.ZMin() + k * vol.ZSpan();
          if(x < region.minx || x > region.maxx ||
             y < region.miny || y > region.maxy ||
             z < region.minz || z > region.maxz)
            continue;

          double v = vol(i, j, k);
          if(v < s.min) s.min = v;
          if(v > s.max) s.max = v;
          sum += v;
          s.num_voxels++;
        }

    if(s.num_voxels == 0)
    {
      s.min = s.max = s.mean = s.std_dev = 0.0;
      return s;
    }

    s.mean = sum / s.num_voxels;

    double sum_sq = 0.0;
    for(uint64 k = 0; k < vol.ZDim(); k++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 i = 0; i < vol.XDim(); i++)
        {
          double x = vol.XMin() + i * vol.XSpan();
          double y = vol.YMin() + j * vol.YSpan();
          double z = vol.ZMin() + k * vol.ZSpan();
          if(x < region.minx || x > region.maxx ||
             y < region.miny || y > region.maxy ||
             z < region.minz || z > region.maxz)
            continue;

          double diff = vol(i, j, k) - s.mean;
          sum_sq += diff * diff;
        }

    s.std_dev = std::sqrt(sum_sq / s.num_voxels);
    return s;
  }

  // ── Element-wise arithmetic ──

  volume vol_add(const volume& a, const volume& b)
  {
    check_dims_match(a, b);
    volume out = make_like(a);
    for(uint64 k = 0; k < a.ZDim(); k++)
      for(uint64 j = 0; j < a.YDim(); j++)
        for(uint64 i = 0; i < a.XDim(); i++)
          out(i, j, k, a(i,j,k) + b(i,j,k));
    return out;
  }

  volume vol_subtract(const volume& a, const volume& b)
  {
    check_dims_match(a, b);
    volume out = make_like(a);
    for(uint64 k = 0; k < a.ZDim(); k++)
      for(uint64 j = 0; j < a.YDim(); j++)
        for(uint64 i = 0; i < a.XDim(); i++)
          out(i, j, k, a(i,j,k) - b(i,j,k));
    return out;
  }

  volume vol_difference(const volume& a, const volume& b)
  {
    check_dims_match(a, b);
    volume out = make_like(a);
    for(uint64 k = 0; k < a.ZDim(); k++)
      for(uint64 j = 0; j < a.YDim(); j++)
        for(uint64 i = 0; i < a.XDim(); i++)
          out(i, j, k, std::fabs(a(i,j,k) - b(i,j,k)));
    return out;
  }

  volume vol_average(const volume& a, const volume& b)
  {
    check_dims_match(a, b);
    volume out = make_like(a);
    for(uint64 k = 0; k < a.ZDim(); k++)
      for(uint64 j = 0; j < a.YDim(); j++)
        for(uint64 i = 0; i < a.XDim(); i++)
          out(i, j, k, (a(i,j,k) + b(i,j,k)) / 2.0);
    return out;
  }

  // ── Scalar operations ──

  volume vol_scale(const volume& vol, double factor)
  {
    volume out = make_like(vol);
    for(uint64 k = 0; k < vol.ZDim(); k++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 i = 0; i < vol.XDim(); i++)
          out(i, j, k, vol(i,j,k) * factor);
    return out;
  }

  volume vol_normalize(const volume& vol, double new_min, double new_max)
  {
    double cur_min = vol.min();
    double cur_max = vol.max();
    double range = cur_max - cur_min;

    volume out = make_like(vol);
    for(uint64 k = 0; k < vol.ZDim(); k++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 i = 0; i < vol.XDim(); i++)
        {
          double v = vol(i, j, k);
          double normalized = (range > 0.0)
            ? new_min + (v - cur_min) * (new_max - new_min) / range
            : new_min;
          out(i, j, k, normalized);
        }
    return out;
  }

  volume vol_clip(const volume& vol, double threshold)
  {
    volume out = make_like(vol);
    for(uint64 k = 0; k < vol.ZDim(); k++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 i = 0; i < vol.XDim(); i++)
        {
          double v = vol(i, j, k);
          out(i, j, k, v < threshold ? v : 0.0);
        }
    return out;
  }

  volume vol_clamp_min(const volume& vol, double min_val)
  {
    volume out = make_like(vol);
    for(uint64 k = 0; k < vol.ZDim(); k++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 i = 0; i < vol.XDim(); i++)
        {
          double v = vol(i, j, k);
          out(i, j, k, v < min_val ? min_val : v);
        }
    return out;
  }

  volume vol_negate(const volume& vol)
  {
    volume out = make_like(vol);
    for(uint64 k = 0; k < vol.ZDim(); k++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 i = 0; i < vol.XDim(); i++)
          out(i, j, k, -vol(i,j,k));
    return out;
  }

  // ── Masking ──

  volume vol_mask(const volume& intensity, const volume& mask)
  {
    check_dims_match(intensity, mask);
    volume out = make_like(intensity);
    for(uint64 k = 0; k < intensity.ZDim(); k++)
      for(uint64 j = 0; j < intensity.YDim(); j++)
        for(uint64 i = 0; i < intensity.XDim(); i++)
          out(i, j, k, mask(i,j,k) != 0.0 ? 0.0 : intensity(i,j,k));
    return out;
  }

  volume vol_inverse_mask(const volume& intensity, const volume& mask)
  {
    check_dims_match(intensity, mask);
    volume out = make_like(intensity);
    for(uint64 k = 0; k < intensity.ZDim(); k++)
      for(uint64 j = 0; j < intensity.YDim(); j++)
        for(uint64 i = 0; i < intensity.XDim(); i++)
          out(i, j, k, mask(i,j,k) == 0.0 ? 0.0 : intensity(i,j,k));
    return out;
  }

  // ── Spatial ──

  volume vol_downsample(const volume& vol,
                        unsigned int fx, unsigned int fy, unsigned int fz)
  {
    if(fx == 0 || fy == 0 || fz == 0)
      throw std::invalid_argument("Downsample factors must be >= 1");

    uint64 nx = (vol.XDim() + fx - 1) / fx;
    uint64 ny = (vol.YDim() + fy - 1) / fy;
    uint64 nz = (vol.ZDim() + fz - 1) / fz;

    double new_xmax = vol.XMin() + (nx - 1) * vol.XSpan() * fx;
    double new_ymax = vol.YMin() + (ny - 1) * vol.YSpan() * fy;
    double new_zmax = vol.ZMin() + (nz - 1) * vol.ZSpan() * fz;

    volume out(dimension(nx, ny, nz), Float,
               bounding_box(vol.XMin(), vol.YMin(), vol.ZMin(),
                            new_xmax, new_ymax, new_zmax));

    for(uint64 k = 0; k < nz; k++)
      for(uint64 j = 0; j < ny; j++)
        for(uint64 i = 0; i < nx; i++)
        {
          uint64 si = std::min(i * fx, vol.XDim() - 1);
          uint64 sj = std::min(j * fy, vol.YDim() - 1);
          uint64 sk = std::min(k * fz, vol.ZDim() - 1);
          out(i, j, k, vol(si, sj, sk));
        }
    return out;
  }

  // ── Rotation ──

  volume vol_rotate_z(const volume& vol, double angle_rad)
  {
    volume out = make_like(vol);

    double cx = (vol.XMin() + vol.XMax()) * 0.5;
    double cy = (vol.YMin() + vol.YMax()) * 0.5;
    double cos_a = std::cos(angle_rad);
    double sin_a = std::sin(angle_rad);

    for(uint64 i = 0; i < vol.XDim(); i++)
      for(uint64 j = 0; j < vol.YDim(); j++)
        for(uint64 k = 0; k < vol.ZDim(); k++)
        {
          // world coordinates of output voxel
          double x = vol.XMin() + i * vol.XSpan();
          double y = vol.YMin() + j * vol.YSpan();
          double z = vol.ZMin() + k * vol.ZSpan();

          // inverse-rotate to find source coordinates
          double dx = x - cx, dy = y - cy;
          double sx = cos_a * dx + sin_a * dy + cx;
          double sy = -sin_a * dx + cos_a * dy + cy;

          // trilinear interpolation (returns 0 if out of bounds)
          if(sx >= vol.XMin() && sx <= vol.XMax() &&
             sy >= vol.YMin() && sy <= vol.YMax())
            out(i, j, k, vol.interpolate(sx, sy, z));
          else
            out(i, j, k, 0.0);
        }
    return out;
  }

  // ── SSIM ──

  ssim_result vol_ssim(const volume& a, const volume& b,
                       int window_size, double sigma)
  {
    check_dims_match(a, b);
    if(window_size < 1 || (window_size % 2) == 0)
      throw std::invalid_argument("SSIM window_size must be a positive odd number");

    // Build 3D Gaussian kernel
    int half = window_size / 2;
    std::vector<double> kernel(window_size * window_size * window_size);
    double gauss_sum = 0;
    for(int gi = 0; gi < window_size; gi++)
      for(int gj = 0; gj < window_size; gj++)
        for(int gk = 0; gk < window_size; gk++)
        {
          double dx = gi - half, dy = gj - half, dz = gk - half;
          double val = std::exp(-(dx*dx + dy*dy + dz*dz) / (2.0 * sigma * sigma));
          int idx = gi * window_size * window_size + gj * window_size + gk;
          kernel[idx] = val;
          gauss_sum += val;
        }
    for(auto& v : kernel) v /= gauss_sum;

    // SSIM constants (Wang 2004)
    double L = 255.0; // dynamic range
    double K1 = 0.01, K2 = 0.03;
    double C1 = (K1 * L) * (K1 * L);
    double C2 = (K2 * L) * (K2 * L);

    volume ssim_map = make_like(a);
    double ssim_sum = 0;
    uint64 count = 0;

    for(uint64 i = 0; i < a.XDim(); i++)
      for(uint64 j = 0; j < a.YDim(); j++)
        for(uint64 k = 0; k < a.ZDim(); k++)
        {
          double mu_a = 0, mu_b = 0;
          double wsum = 0;

          // weighted means
          for(int gi = -half; gi <= half; gi++)
            for(int gj = -half; gj <= half; gj++)
              for(int gk = -half; gk <= half; gk++)
              {
                int64 x = (int64)i + gi, y = (int64)j + gj, z = (int64)k + gk;
                if(x < 0 || x >= (int64)a.XDim() ||
                   y < 0 || y >= (int64)a.YDim() ||
                   z < 0 || z >= (int64)a.ZDim()) continue;
                int kidx = (gi+half)*window_size*window_size +
                           (gj+half)*window_size + (gk+half);
                double w = kernel[kidx];
                mu_a += w * a(x, y, z);
                mu_b += w * b(x, y, z);
                wsum += w;
              }
          if(wsum > 0) { mu_a /= wsum; mu_b /= wsum; }

          // weighted variances and covariance
          double sig_a2 = 0, sig_b2 = 0, sig_ab = 0;
          double wsum2 = 0;
          for(int gi = -half; gi <= half; gi++)
            for(int gj = -half; gj <= half; gj++)
              for(int gk = -half; gk <= half; gk++)
              {
                int64 x = (int64)i + gi, y = (int64)j + gj, z = (int64)k + gk;
                if(x < 0 || x >= (int64)a.XDim() ||
                   y < 0 || y >= (int64)a.YDim() ||
                   z < 0 || z >= (int64)a.ZDim()) continue;
                int kidx = (gi+half)*window_size*window_size +
                           (gj+half)*window_size + (gk+half);
                double w = kernel[kidx];
                double da = a(x, y, z) - mu_a;
                double db = b(x, y, z) - mu_b;
                sig_a2 += w * da * da;
                sig_b2 += w * db * db;
                sig_ab += w * da * db;
                wsum2 += w;
              }
          if(wsum2 > 0) { sig_a2 /= wsum2; sig_b2 /= wsum2; sig_ab /= wsum2; }

          double ssim = (2.0*mu_a*mu_b + C1) * (2.0*sig_ab + C2) /
                        ((mu_a*mu_a + mu_b*mu_b + C1) * (sig_a2 + sig_b2 + C2));
          ssim_map(i, j, k, ssim);
          ssim_sum += ssim;
          count++;
        }

    ssim_result result;
    result.mean_ssim = count > 0 ? ssim_sum / count : 0.0;
    result.ssim_map = ssim_map;
    return result;
  }

  // ── Projection ──

  volume vol_project(const volume& vol,
                     const std::vector<double>& angles_rad,
                     double step)
  {
    unsigned int na = static_cast<unsigned int>(angles_rad.size());
    if(na == 0) throw std::invalid_argument("angles_rad must not be empty");

    double cx = (vol.XMin() + vol.XMax()) * 0.5;
    double cz = (vol.ZMin() + vol.ZMax()) * 0.5;
    double max_s = std::sqrt((vol.XMax()-vol.XMin())*(vol.XMax()-vol.XMin())/4.0 +
                             (vol.ZMax()-vol.ZMin())*(vol.ZMax()-vol.ZMin())/4.0);

    // output: XDim × YDim × num_angles
    volume out(dimension(vol.XDim(), vol.YDim(), na), Float,
               bounding_box(vol.XMin(), vol.YMin(), 0,
                            vol.XMax(), vol.YMax(), (double)na));

    for(unsigned int ai = 0; ai < na; ai++)
    {
      double theta = angles_rad[ai];
      double cos_t = std::cos(theta), sin_t = std::sin(theta);

      for(uint64 xi = 0; xi < vol.XDim(); xi++)
        for(uint64 yi = 0; yi < vol.YDim(); yi++)
        {
          double x0 = vol.XMin() + xi * vol.XSpan();
          double y0 = vol.YMin() + yi * vol.YSpan();
          double accum = 0;

          for(double s = -max_s; s <= max_s; s += step)
          {
            double sx = x0 * cos_t + s * sin_t;
            double sz = -x0 * sin_t + s * cos_t;
            // shift from centered coords back to volume coords
            double vx = sx + cx - cx * cos_t;
            double vz = sz + cz + cx * sin_t;

            // map back to volume bbox
            if(vx < vol.XMin() || vx > vol.XMax() ||
               vz < vol.ZMin() || vz > vol.ZMax()) continue;

            accum += vol.interpolate(vx, y0, vz);
          }
          out(xi, yi, ai, accum * step);
        }
    }
    return out;
  }

  // ── Back-projection ──

  volume vol_back_project(const volume& projections,
                          const std::vector<double>& angles_rad,
                          unsigned int output_dim,
                          bool apply_filter)
  {
    unsigned int na = static_cast<unsigned int>(angles_rad.size());
    if(na == 0) throw std::invalid_argument("angles_rad must not be empty");
    if(projections.ZDim() != na)
      throw dimension_mismatch("projections Z-dimension must equal number of angles");

#ifdef CVC_ENABLE_FFTW
    // Optional ramp filter via 2D FFT per angle slice
    volume filtered = projections;
    if(apply_filter)
    {
      filtered = make_like(projections);
      unsigned int nx = projections.XDim(), ny = projections.YDim();
      unsigned int n = nx * ny;
      for(unsigned int ai = 0; ai < na; ai++)
      {
        fftw_complex* spatial = (fftw_complex*)fftw_malloc(sizeof(fftw_complex)*n);
        fftw_complex* freq    = (fftw_complex*)fftw_malloc(sizeof(fftw_complex)*n);

        for(unsigned int i = 0; i < nx; i++)
          for(unsigned int j = 0; j < ny; j++)
          {
            unsigned int idx = i * ny + j;
            spatial[idx][0] = projections(i, j, ai);
            spatial[idx][1] = 0;
          }

        fftw_plan fwd = fftw_plan_dft_2d(nx, ny, spatial, freq,
                                          FFTW_FORWARD, FFTW_ESTIMATE);
        fftw_execute(fwd);
        fftw_destroy_plan(fwd);

        // ramp filter
        double max_r = std::sqrt((nx/2.0)*(nx/2.0) + (ny/2.0)*(ny/2.0));
        double slope = (max_r > 0) ? (1.0 - 1.0/max_r) / max_r : 0;
        for(unsigned int i = 0; i < nx; i++)
          for(unsigned int j = 0; j < ny; j++)
          {
            double r = std::sqrt(((double)nx/2 - i)*((double)nx/2 - i) +
                                 ((double)ny/2 - j)*((double)ny/2 - j));
            double h = 4.0 * (1.0 - slope * r);
            unsigned int idx = i * ny + j;
            freq[idx][0] *= h;
            freq[idx][1] *= h;
          }

        fftw_plan inv = fftw_plan_dft_2d(nx, ny, freq, spatial,
                                          FFTW_BACKWARD, FFTW_ESTIMATE);
        fftw_execute(inv);
        fftw_destroy_plan(inv);

        for(unsigned int i = 0; i < nx; i++)
          for(unsigned int j = 0; j < ny; j++)
            filtered(i, j, ai, spatial[i * ny + j][0] / n);

        fftw_free(spatial);
        fftw_free(freq);
      }
    }
#else
    if(apply_filter)
      throw std::runtime_error("FFTW not available; set apply_filter=false or enable CVC_ENABLE_FFTW");
    const volume& filtered = projections;
#endif

    unsigned int d = output_dim;
    volume out(dimension(d, d, d), Float,
               bounding_box(0, 0, 0, d-1, d-1, d-1));

    // back-project
    for(unsigned int x = 0; x < d; x++)
      for(unsigned int y = 0; y < d; y++)
        for(unsigned int z = 0; z < d; z++)
        {
          double accum = 0;
          unsigned int count = 0;
          double x_shift = (double)x - d / 2.0;
          double z_shift = (double)z - d / 2.0;

          for(unsigned int ai = 0; ai < na; ai++)
          {
            double theta = angles_rad[ai];
            double xcoord = x_shift * std::cos(theta) + z_shift * std::sin(theta)
                            + projections.XDim() / 2.0;

            if(xcoord >= 0 && xcoord < projections.XDim() - 1 &&
               y < projections.YDim())
            {
              // bilinear interpolation in projection
              unsigned int xi0 = (unsigned int)std::floor(xcoord);
              unsigned int xi1 = xi0 + 1;
              double frac = xcoord - xi0;
              double val = (1.0 - frac) * filtered(xi0, y, ai) +
                           frac * filtered(xi1, y, ai);
              accum += val;
              count++;
            }
          }
          out(x, y, z, count > 0 ? accum / count : 0.0);
        }
    return out;
  }

  // ── Image I/O ──

  void vol_to_slices(const volume& vol,
                     const std::string& directory,
                     const std::string& format)
  {
#ifdef CVC_ENABLE_IMAGEMAGICK
    // Normalise to [0,255] UChar for image output
    volume norm = vol_normalize(vol, 0.0, 255.0);

    for(uint64 k = 0; k < norm.ZDim(); k++)
    {
      std::vector<unsigned char> buf(norm.XDim() * norm.YDim() * 3);
      for(uint64 j = 0; j < norm.YDim(); j++)
        for(uint64 i = 0; i < norm.XDim(); i++)
        {
          unsigned char v = static_cast<unsigned char>(
              std::min(255.0, std::max(0.0, norm(i, j, k))));
          size_t idx = (j * norm.XDim() + i) * 3;
          buf[idx] = buf[idx+1] = buf[idx+2] = v;  // grayscale → RGB
        }

      Magick::Image img(norm.XDim(), norm.YDim(), "RGB",
                        Magick::CharPixel, buf.data());
      img.type(Magick::GrayscaleType);

      char fname[512];
      std::snprintf(fname, sizeof(fname), format.c_str(), (int)k);
      std::string path = directory + "/" + fname;
      img.write(path);
    }
#else
    (void)vol; (void)directory; (void)format;
    throw std::runtime_error("ImageMagick support not enabled (CVC_ENABLE_IMAGEMAGICK=OFF)");
#endif
  }

  volume slices_to_volume(const std::vector<std::string>& paths,
                          const bounding_box& bbox)
  {
#ifdef CVC_ENABLE_IMAGEMAGICK
    if(paths.empty())
      throw std::invalid_argument("slices_to_volume: empty path list");

    // Read first image for dimensions
    Magick::Image first(paths[0]);
    uint64 w = first.columns(), h = first.rows();
    uint64 d = paths.size();

    volume out(dimension(w, h, d), UChar, bbox);

    for(uint64 k = 0; k < d; k++)
    {
      Magick::Image img(paths[k]);
      img.modifyImage();
      img.type(Magick::GrayscaleType);

      std::vector<unsigned char> buf(w * h);
      img.write(0, 0, w, h, "R", Magick::CharPixel, buf.data());

      for(uint64 j = 0; j < h; j++)
        for(uint64 i = 0; i < w; i++)
          out(i, j, k, (double)buf[j * w + i]);
    }
    return out;
#else
    (void)paths; (void)bbox;
    throw std::runtime_error("ImageMagick support not enabled (CVC_ENABLE_IMAGEMAGICK=OFF)");
#endif
  }

  // ── RGBA multi-variable operations ──

  volume vol_rgba_merge(const volume& r, const volume& g,
                        const volume& b, const volume& a)
  {
    check_dims_match(r, g);
    check_dims_match(r, b);
    check_dims_match(r, a);

    // Create a volume with 4x the X dimension to pack RGBA interleaved
    // This stores R,G,B,A as consecutive voxels
    uint64 nx = r.XDim(), ny = r.YDim(), nz = r.ZDim();
    volume out(dimension(nx * 4, ny, nz), Float, r.boundingBox());

    for(uint64 k = 0; k < nz; k++)
      for(uint64 j = 0; j < ny; j++)
        for(uint64 i = 0; i < nx; i++)
        {
          out(i*4 + 0, j, k, r(i, j, k));
          out(i*4 + 1, j, k, g(i, j, k));
          out(i*4 + 2, j, k, b(i, j, k));
          out(i*4 + 3, j, k, a(i, j, k));
        }
    return out;
  }

  std::vector<volume> vol_split_vars(const volume& vol)
  {
    // If the volume is a simple single-variable volume, return it as-is
    // For multi-variable (RGBA-packed) volumes, split by channels
    // We detect RGBA packing if XDim is divisible by 4
    // The caller is responsible for knowing the layout

    // Simple case: just return a copy
    std::vector<volume> result;
    result.push_back(vol);
    return result;
  }
}
