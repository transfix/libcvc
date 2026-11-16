/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/gl/OverlappingVolumeMapper.h>
#include <vtkObjectFactory.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtk_glad.h>

namespace cvc {
namespace gl {

vtkStandardNewMacro(OverlappingVolumeMapper);

namespace {
#if !defined(__EMSCRIPTEN__)
// What the GPU ray caster's glFinish runs during an untimed draw: nothing.
void GLAD_API_PTR skip_finish(void) {}
#endif
} // namespace

bool OverlappingVolumeMapper::TimesItsDraws(vtkRenderer *ren) {
  if (this->GetInteractiveAdjustSampleDistances())
    return ren && ren->GetRenderWindow() &&
           ren->GetRenderWindow()->GetDesiredUpdateRate() >= this->GetInteractiveUpdateRate();
  return this->GetAutoAdjustSampleDistances() != 0;
}

void OverlappingVolumeMapper::Render(vtkRenderer *ren, vtkVolume *vol) {
#if !defined(__EMSCRIPTEN__)
  // glad's pointer, swapped for the draw unless the draw is timed, or the loader has not loaded it,
  // or an enclosing draw has swapped it already (then that draw puts it back).
  if (!this->TimesItsDraws(ren) && glad_glFinish && glad_glFinish != &skip_finish) {
    const PFNGLFINISHPROC finish = glad_glFinish;
    glad_glFinish = &skip_finish;
    this->vtkSmartVolumeMapper::Render(ren, vol);
    if (glad_glFinish == &skip_finish) // (left as it is if something reloaded the loader)
      glad_glFinish = finish;
    return;
  }
#endif
  this->vtkSmartVolumeMapper::Render(ren, vol);
}

} // namespace gl
} // namespace cvc
