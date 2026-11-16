/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_OVERLAPPING_VOLUME_MAPPER_H__
#define __CVC_GL_OVERLAPPING_VOLUME_MAPPER_H__

#include <vtkSmartVolumeMapper.h>

namespace cvc {
namespace gl {

// OverlappingVolumeMapper — vtkSmartVolumeMapper without the GPU wait at the end of every draw.
//
// VTK's GPU ray caster (vtkOpenGLGPUVolumeRayCastMapper::GPURender) ends each draw with
// glFinish(), so that the CPU timer around it (vtkGPUVolumeRayCastMapper::Render's TimeToDraw)
// measures the GPU's work. The CPU then waits for everything queued before it (the scene's whole
// draw so far, and the volume's ray march) and the GPU sits idle while the CPU prepares the rest of
// the frame and the next: the CPU's and the GPU's times add instead of overlapping. In
// terrain_lab, with the clouds' near volumes, that wait was over half of every frame (25 ms of 45
// on an Intel Arc iGPU).
//
// The draw only needs that timing when it adjusts its sample distance by it (auto-adjust: on in
// an interactive render when InteractiveAdjustSampleDistances is, else as AutoAdjustSampleDistances
// says). In every other draw (a still render, which is what a scene's frames are unless a host
// raises the window's desired update rate) this mapper makes the GPU ray caster's glFinish a no-op
// for the length of its own Render, so the draw is queued as any other and the frame's CPU and GPU
// work overlap. What is drawn is the same: glFinish only waits.
//
// It does this through VTK's GL loader (glad): the loader's glFinish pointer is swapped for an
// empty function for the draw and put back after, and left alone when another draw has already
// swapped it. The WebAssembly build has no loader pointer to swap and keeps VTK's behaviour.
class OverlappingVolumeMapper : public vtkSmartVolumeMapper {
public:
  static OverlappingVolumeMapper *New();
  vtkTypeMacro(OverlappingVolumeMapper, vtkSmartVolumeMapper);

  void Render(vtkRenderer *ren, vtkVolume *vol) override;

  // Whether a draw in renderer `ren` adjusts its sample distance by its time (and so keeps VTK's
  // glFinish): what vtkSmartVolumeMapper::Render sets on its GPU mapper.
  bool TimesItsDraws(vtkRenderer *ren);

protected:
  OverlappingVolumeMapper() = default;
  ~OverlappingVolumeMapper() override = default;

private:
  OverlappingVolumeMapper(const OverlappingVolumeMapper &) = delete;
  void operator=(const OverlappingVolumeMapper &) = delete;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_OVERLAPPING_VOLUME_MAPPER_H__
