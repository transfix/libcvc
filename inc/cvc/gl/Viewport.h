/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_VIEWPORT_H__
#define __CVC_GL_VIEWPORT_H__

#include <functional>
#include <memory>
#include <string>

class vtkProp; // VTK (global namespace)
class vtkRenderer;

namespace cvc {
class app;
namespace gl {
class SceneGraph;
class CameraController;
class ViewportManager;

// -----------
// Viewport
// -----------
// One rectangle of a ViewportManager's single window: its OWN vtkRenderer,
// CameraController and the SceneGraph it draws. It never opens a GL context —
// it is a vtkRenderer inside the manager's one vtkRenderWindow, placed by a
// NORMALIZED region and a layer, composited with the others in one render.
//
// Two flavours (both first-class): a SCENE viewport draws its own distinct
// SceneGraph (a clean, independent renderer); a MIRROR viewport is an alternate
// view of a scene another viewport already draws (the minimap / PiP case).
// This header is the shared surface; the manager's addSceneViewport /
// addMirrorViewport decide the flavour.
//
// Each viewport owns a CameraController at its own state path
// ("<scene prefix>.viewers.<name>.camera"), so a PiP inset aims / follows
// independently of the main view. The controller is built with the low-level
// (app, path) ctor and driven through update()/event feeds — it does NOT attach
// to the window interactor (the manager's one input router owns that).
class Viewport {
public:
  ~Viewport();
  Viewport(const Viewport &) = delete;
  Viewport &operator=(const Viewport &) = delete;

  const std::string &name() const;

  // Placement. region is normalized [x0, y0, x1, y1] in VTK's y-up window space;
  // {0,0,1,1} is the whole window. Stored normalized, so it survives a resize and
  // is re-applied to the renderer; move it every frame to track a draggable inset.
  // A viewport drawn into a texture (setTextureSize) is shown elsewhere: region() is
  // where it shows, and may run past the window's edges.
  void setRegion(double x0, double y0, double x1, double y1);
  void region(double out[4]) const;

  // Composite layer: 0 is the base (opaque) layer; >0 is an overlay drawn on top.
  // The manager keeps the window's layer count >= max(layer)+1.
  void setLayer(int layer);
  int layer() const;

  // Cheap show/hide: a hidden viewport is skipped in the composite (its renderer
  // is left in the window but not drawn), so a PiP can be toggled per frame.
  void setVisible(bool on);
  bool visible() const;

  // Background colour + whether this viewport CLEARS its rect. An opaque overlay
  // panel clears (so it reads as a solid inset); a translucent overlay must NOT
  // clear, or it wipes the base view under it.
  void setBackground(double r, double g, double b, bool opaque = true);

  CameraController &camera(); // this viewport's own controller / vtkCamera
  SceneGraph &scene() const;  // the scene it draws (its own, or a mirror source)
  vtkRenderer *renderer() const;

  bool isMirror() const;
  // For a mirror viewport, the source viewport's renderer whose props it echoes
  // each frame (with its own camera); nullptr for a scene viewport. The manager
  // reads this in render() to keep the mirror in sync with the live scene.
  vtkRenderer *mirrorSource() const;

  // Whether the manager re-syncs this mirror from its source EVERY frame (default
  // true) or only ONCE, at add time (a FROZEN snapshot). A frozen mirror still
  // reflects geometry/visibility changes to props it already holds (they are the
  // SAME shared vtkProp pointers), but it does NOT gain props added to the source
  // afterwards and its lights are not overwritten each frame — so a caller can
  // give it a fixed look (e.g. a flat overview with its own headlight) that a
  // live re-sync would otherwise clobber. Set via addMirrorViewport(liveSync).
  bool mirrorLive() const;

  // What a mirror draws for each of the source's props. With a filter, each re-sync adds what it
  // returns: the prop itself, another prop in its place (a restyled twin sharing its mapper, so
  // the geometry is not copied), or nullptr to leave it out. So the mirror shows a subset of the
  // scene, restyled: a viewport widget leaves out the nodes whose tags it does not show and draws
  // its override:s. No filter (the default) copies every 3-D prop. The filter runs in
  // ViewportManager::render(), once per prop per frame.
  using PropFilter = std::function<vtkProp *(vtkProp *)>;
  void setMirrorFilter(PropFilter keep);
  const PropFilter &mirrorFilter() const;

  // Whether this viewport receives routed input (default true). A minimap / HUD
  // inset set false is skipped by ViewportManager::viewportAt, so clicks fall
  // through to the viewport beneath it — an overview minimap is non-interactive
  // by design ("a minimap that pans or dollies stops being a map"). Does not affect
  // compositing.
  void setInputEnabled(bool on);
  bool inputEnabled() const;

  // Redraw rate, for an inset whose picture changes slowly: a map, an overview. 0 (the default)
  // draws its scene in every ViewportManager::render(). hz > 0 draws it at most hz times a second;
  // the renders in between skip its scene (and a mirror's re-sync) and show the pixels of its last
  // draw, kept in a colour texture the size of the window. A new size or place on screen, being
  // shown again, or invalidate() draws it at the next render. Only an opaque viewport is throttled
  // (setBackground(..., opaque = true)): a translucent one draws over what is under it, so its last
  // pixels hold a stale picture of that too, and it draws every frame whatever the rate.
  void setRefreshHz(double hz);
  double refreshHz() const;
  void invalidate(); // draw it at the next render(), whatever the rate
  // How many times this viewport's scene has been drawn (renders that showed its last pixels do
  // not count). For tests and measurements.
  long draws() const;

  // Drawn into a texture of its own instead of into its rect of the window. With a size (pixels),
  // the viewport draws its scene into a colour texture that size, in the window's render, and
  // draws nothing into the window: something else shows the texture where region() says, a UI's
  // image (an Ariadne `viewport` widget), so the UI's windows stack over it, clip it and scroll it
  // as they do any of their content. 0 x 0 (the default) draws into the window. The texture is no
  // larger than the window: a larger size draws at the window's (the image stretches it).
  //
  // It draws at the window's own multisampling and is resolved as the window's pixels are (with
  // VTK's gamma-correct resolve where the window shows its frame, with a blit where its pixels are
  // read back: an offscreen window), so it looks as it would have in the window. Its renderer's
  // viewport is the texture's rect at the
  // window's lower left, so its display coordinates are texture pixels (windowToRenderer). A
  // throttled one (setRefreshHz) keeps its last picture in the texture between draws, and draws at
  // once on a new size or when it is shown again; moving it does not draw it. A mirror re-syncs
  // its props on the renders it draws, as the render reaches it, so a show or a size set during
  // the frame (a UI walk inside the render) takes effect in that frame.
  void setTextureSize(int w, int h);
  bool textured() const;
  void textureSize(int out[2]) const; // the size it draws at: the requested one, within the window
  // The GL name of its texture, for a UI to draw (0 when it is not textured, or the window has no
  // GL context yet). Made or resized here, at the size setTextureSize asked for, so it can be
  // handed to a UI image before the window draws; it makes the window's context current. Its
  // rows run bottom-up (GL's): an image that shows it flips v.
  unsigned int texture();
  // Window display coordinates (pixels from the lower left) to its renderer's: pixels of its
  // texture by region() when it is textured, the same point otherwise. For a pick or a zoom about
  // the pointer at a place on screen.
  void windowToRenderer(double x, double y, double out[2]) const;
  // ... and back: where a point of its renderer's display (a world point's WorldToDisplay) shows in
  // the window.
  void rendererToWindow(double x, double y, double out[2]) const;

private:
  friend class ViewportManager; // the manager constructs and owns Viewports
  // Manager-only, once per render() before the window draws: whether this viewport draws its scene
  // in this render (true) or shows its last pixels (false; a hidden viewport draws nothing). A
  // textured viewport decides as it draws (beforeDraw).
  bool beginRender();
  // Manager-only: what a textured viewport runs before each draw of its scene (a mirror's re-sync).
  void setBeforeDraw(std::function<void()> fn);
  // Manager-only: free its GL objects while the window's context is still there.
  void releaseGraphics();
  // `managed` (default) builds the eager CameraController + ViewportLayout state
  // this viewport normally owns. `managed == false` builds it BARE — no
  // controller and no ".viewers.<name>.layout" state at construction; camera()
  // lazily builds the controller only if something asks for it. HostStyle
  // (the SceneRenderer facade) uses bare so its state footprint matches the
  // classic single-view renderer, which writes no such state. `viewer` names its layout state
  // (".viewers.<viewer>.layout"); empty: `name` does.
  Viewport(cvc::app &app, SceneGraph &scene, const std::string &cameraStatePath,
           const std::string &name, bool mirror, bool managed = true,
           const std::string &viewer = std::string());
  void setMirrorSource(vtkRenderer *src); // manager-only, at addMirrorViewport
  void setMirrorLive(bool on);            // manager-only, at addMirrorViewport

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_VIEWPORT_H__
