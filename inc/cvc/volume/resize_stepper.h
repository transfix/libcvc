/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef CVC_VOLUME_RESIZE_STEPPER_H
#define CVC_VOLUME_RESIZE_STEPPER_H

#include <cstddef>
#include <cvc/volume/volume.h>
#include <memory>

namespace cvc {

// volume::resize(dimension) a range of output voxels at a time, for a caller that must not block
// (the mesher's resample to (2^n+1)^3, fe_lab's volume source on single-threaded wasm): the same
// trilinear taps, edge rule and conversion to the voxel type as resize()'s CPU path, so the voxels
// are bit-identical to its (a CUDA build's resize() may differ). The result keeps the source's
// bounding box, voxel type and description. A target equal to the source's dimensions is the
// source itself; one within 2x2x2 (resize()'s special case) is resize()'s own, in one step. Holds a
// copy-on-write copy of the source, so the caller's volume may go away. Throws null_dimension for a
// null target, and from the constructor what resize() would for an undefined voxel type.
class resize_stepper {
public:
  resize_stepper(const volume &src, const dimension &target);
  resize_stepper(resize_stepper &&) noexcept;
  resize_stepper &operator=(resize_stepper &&) noexcept;
  ~resize_stepper();
  resize_stepper(const resize_stepper &) = delete;
  resize_stepper &operator=(const resize_stepper &) = delete;

  // Works through `budget` output voxels (0: to the end); true once the volume is ready.
  bool step(std::size_t budget);
  bool done() const;
  double progress() const;
  // The resized volume, once step() has returned true.
  volume take();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace cvc

#endif
