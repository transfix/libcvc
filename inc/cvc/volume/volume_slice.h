/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Axis-aligned slices of a volume: one plane of voxels, cut from a volume in memory or read from a
// volume file one plane at a time, and coloured through a transfer function
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §6 item 1, P7). This is the data side of
// cvcGL's `volume_slice` scene node and of VolumeGridRover's slice views:
//
//   cvc::slice_source src(vol);                                // or (app, "big.rawiv"): region
//   reads auto s = src.slice(cvc::slice_axis::z, 40);                // the plane k = 40
//   std::vector<std::uint8_t> px = cvc::color_slice(*s, model, src.min(), src.max(), true);
//
// The plane's two axes, right then up, are the ones VolumeGridRover's views showed: looking down z,
// x right and y up (its XY view); down y, x right and z up (XZ); down x, z right and y up (ZY).
// A slice's values are a width x height grid along them, row 0 the bottom (the smallest up
// coordinate). Voxels sit at the bounding box's grid points, as everywhere in cvc::volume: voxel i
// along x is at minx + i * (maxx - minx) / (xdim - 1).
//
// Everything here holds no state of its own: no statics, no globals (§1.3). A slice_source keeps
// its cache in itself, behind its own lock.

#ifndef CVC_VOLUME_VOLUME_SLICE_H
#define CVC_VOLUME_VOLUME_SLICE_H

#include <boost/any.hpp>
#include <boost/describe/enum.hpp>
#include <cstddef>
#include <cstdint>
#include <cvc/core/param_table.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/dimension.h>
#include <cvc/volume/volume.h>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
class app;
namespace tf {
class model;
}

// The axis a slice looks down: the plane holds every voxel with that index along it.
enum class slice_axis { x, y, z };
BOOST_DESCRIBE_ENUM(slice_axis, x, y, z)

// How a slice's texture is sampled between voxel centres: the nearest voxel (each voxel a square,
// the default, as a segmentation tool wants it) or linearly.
enum class slice_filter { nearest, linear };
BOOST_DESCRIBE_ENUM(slice_filter, nearest, linear)

// An axis by name: x, y or z, 0, 1 or 2, or the name of the view that looks down it: xy (z),
// xz or zx (y), zy or yz (x). Case and surrounding blanks are ignored. False for anything else.
bool parse_slice_axis(std::string_view text, slice_axis &out);
// "x", "y" or "z".
const char *slice_axis_name(slice_axis a);
// The view's name: "xy", "xz" or "zy" (VolumeGridRover's tabs).
const char *slice_view_name(slice_axis a);
// The plane's axes (0 x, 1 y, 2 z), right then up: z -> (0, 1), y -> (0, 2), x -> (2, 1).
void slice_plane_axes(slice_axis a, int &right, int &up);

// The grid slices are cut from: its voxel dimensions and its bounding box, the voxels at the box's
// grid points.
struct slice_grid {
  dimension dim;
  bounding_box box;

  // How many voxels lie along `axis` (0 x, 1 y, 2 z), and so how many slices look down it.
  std::uint64_t count(int axis) const;
  std::uint64_t count(slice_axis a) const { return count(static_cast<int>(a)); }
  // The distance between neighbouring voxel centres along `axis`; with one voxel along it, the
  // box's extent there, or 1 when that is 0 (the voxel is drawn that big).
  double spacing(int axis) const;
  // The box's lower and upper bound along `axis`.
  double lo(int axis) const;
  double hi(int axis) const;
  // Where slice `index` looking down `a` lies along `a` (index clamped to the grid).
  double plane(slice_axis a, std::uint64_t index) const;
  // The slice whose plane is nearest `coord` along `a` (clamped to the grid).
  std::uint64_t nearest_index(slice_axis a, double coord) const;
  // The voxel nearest the object-space point `p` (indices rounded, not clamped), and whether it
  // lies inside the grid.
  bool voxel_of(const double p[3], std::int64_t ijk[3]) const;
  // The object-space centre of voxel `ijk`.
  void voxel_center(const std::int64_t ijk[3], double p[3]) const;
};
slice_grid grid_of(const volume &v);

// One slice: `width` x `height` values along the plane's right and up axes, row 0 the bottom.
struct volume_slice_data {
  slice_axis axis = slice_axis::z;
  std::uint64_t index = 0;
  std::uint64_t width = 0, height = 0;
  std::vector<double> values; // values[row * width + column]
  double at(std::uint64_t column, std::uint64_t row) const { return values[row * width + column]; }
  // The voxel (i, j, k) at a column and row.
  void voxel(std::uint64_t column, std::uint64_t row, std::uint64_t ijk[3]) const;
};

// Slice `index` of `vol` looking down `axis`. Throws std::out_of_range, naming the volume's
// dimensions, when the index lies outside it, and std::invalid_argument for an empty volume.
volume_slice_data extract_slice(const volume &vol, slice_axis axis, std::uint64_t index);

// The pixels of slice `s` through the transfer function `m` over the data range [lo, hi] (the
// model's own domain when it is explicit: cvc::tf::effective_domain), as RGBA bytes in image
// order: the top row (the slice's last) first, as cvc::image keeps them. Each value takes the
// colour of the model's baked table (m.resolution entries) at its normalized position. With
// `opaque` every pixel's alpha is 255 (the slice shows every voxel in its colour); otherwise it is
// the model's opacity there. A NaN value is transparent black either way.
std::vector<std::uint8_t> color_slice(const volume_slice_data &s, const tf::model &m, double lo,
                                      double hi, bool opaque);

// The slices of one volume: a volume in memory, or a volume file whose header is read at once and
// whose planes are read one at a time when asked for (a region read: readVolumeFile with a
// one-voxel-thick subvolume), so stepping through a volume larger than memory never reads it all.
// The last `cache` planes asked for are kept (the newest first), so scrubbing back and forth
// re-reads nothing. Safe to call from several threads.
class slice_source {
public:
  // A volume in memory; its voxels are shared, not copied (cvc::volume is copy-on-write).
  explicit slice_source(const volume &vol, std::size_t cache = 8);
  // A volume file: `var` and `time` pick the variable and timestep. Reads the header now (and the
  // data range: from the header when it holds one, else by a pass over the file). Throws
  // std::runtime_error naming the file when it can't be read.
  slice_source(app &ctx, const std::string &file, unsigned int var = 0, unsigned int time = 0,
               std::size_t cache = 8);

  const slice_grid &grid() const { return _grid; }
  // The volume's data range.
  double min() const { return _min; }
  double max() const { return _max; }
  // Whether the volume is in memory (else its planes come from the file).
  bool in_memory() const { return _volume.has_value(); }
  // The volume in memory, or null for a file.
  const volume *in_memory_volume() const { return _volume ? &*_volume : nullptr; }
  const std::string &file() const { return _file; }

  // Slice `index` looking down `a`, from the cache or read now. Throws as extract_slice does, and
  // std::runtime_error naming the file when a region read fails.
  std::shared_ptr<const volume_slice_data> slice(slice_axis a, std::uint64_t index);
  // The value of voxel (i, j, k): from memory, or from the slice down `a` that holds it (read
  // when it is not cached). Throws std::out_of_range outside the grid.
  double value(const std::uint64_t ijk[3], slice_axis a = slice_axis::z);
  // How many planes have been read from the file (0 for a volume in memory), for tests.
  std::size_t reads() const;

private:
  struct cached {
    slice_axis axis;
    std::uint64_t index;
    std::shared_ptr<const volume_slice_data> data;
  };
  app *_ctx = nullptr;
  std::optional<volume> _volume;
  std::string _file;
  unsigned int _var = 0, _time = 0;
  slice_grid _grid;
  double _min = 0.0, _max = 0.0;
  std::size_t _capacity = 8;
  mutable std::mutex _m;
  std::deque<cached> _cache; // newest first
  std::size_t _reads = 0;
};

// A volume as a state key's data channel holds one (docs/roadmap/DATA_FROM_STATE_TREE.md): a
// std::shared_ptr<const volume> (a data: entry's), a std::shared_ptr<volume>, a volume, or a
// std::shared_ptr<slice_source> (a host's large file read plane by plane), as such or as the payload
// of a data object a program stored there (state-data-set: a DSL volume.filter's result, say).
// `src` is then the slices to cut, and `vol` the volume itself: the very object the key's pointer
// holds (a slice_source's in
// memory, kept by the source; null for a file read plane by plane), or for a plain volume a new
// handle on its voxels. False, leaving both alone, when `d` holds none of them or a null pointer.
bool volume_payload(const boost::any &d, std::shared_ptr<slice_source> &src,
                    std::shared_ptr<const volume> &vol);

// The object a volume payload's pointer holds, or null (a plain volume): what tells a new payload
// from the same one written again (cvc::followed_key's pointee).
const void *volume_payload_pointee(const boost::any &d);

// How a slice node draws (§1.5): one key per field under <node>.slice.
struct volume_slice_params {
  slice_axis axis = slice_axis::z;
  std::int64_t index = -1; // -1: the middle of the axis
  slice_filter filter = slice_filter::nearest;
  bool opaque = true;
  double overlay_offset = 2.0;
};

template <> struct param_table<volume_slice_params> : param_fields<volume_slice_params> {
  using P = volume_slice_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("axis", &P::axis, doc("the axis the slice looks down: x, y or z")),
      field("index", &P::index, unit("voxel"), range(-1.0, none),
            doc("which plane along the axis, from 0; -1 is the middle one")),
      field("filter", &P::filter,
            doc("nearest draws each voxel as a square; linear blends between voxel centres")),
      field("opaque", &P::opaque,
            doc("every voxel in its transfer-function colour at full opacity; off takes the "
                "opacity from the transfer function")),
      field("overlay_offset", &P::overlay_offset, range(0.0, none),
            doc("how far the slice is pushed back in depth so overlays drawn in its plane stay "
                "on top (polygon-offset units)")),
  };
};

} // namespace cvc

#endif // CVC_VOLUME_VOLUME_SLICE_H
