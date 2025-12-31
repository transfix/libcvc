# Information Report: Development Options for Adapting Legacy Computational Graphics Software to Modern VTK and Graphics Hardware

## Executive Summary
This report synthesizes discussions on integrating legacy rendering components from our computational geometry package into the Visualization Toolkit (VTK), leveraging modern graphics hardware such as GPUs via OpenGL and CUDA. The primary goals are to enhance performance for volume rendering, isosurface extraction, arbitrary geometry, and volumetric meshes while preserving custom legacy renderers. Key options include using VTK's extensible mapper-actor pipeline for standard and custom rendering, with support for multi-volume scenes and depth-aware compositing. Implementation directions emphasize subclassing VTK classes for custom integrations, performance benchmarking, and handling overlaps in multi-volume scenarios. This will inform our planning document for a hardened implementation path, prioritizing GPU acceleration and compatibility with existing data structures.

## Background and Objectives
Our legacy software includes specialized renderers: an OpenGL ARB fragment program-based volume renderer (high-speed, legacy shaders) and a raycasting volume/isosurface renderer ("volren") being ported to CUDA for acceleration. The package handles volume data, arbitrary polygonal geometry (triangles/quads with colors/textures), tetrahedral/hexahedral meshes, and isosurfaces. Adaptation to VTK aims to:
- Utilize VTK's robust visualization pipeline for scene management, interaction, and data filtering.
- Integrate custom renderers as extensions for performance comparisons and hybrid use.
- Support multiple volumes in scenes with proper depth blending to avoid artifacts.
- Leverage modern hardware (e.g., NVIDIA GPUs for CUDA) while maintaining fallback options.

VTK (version 9.x+ as of late 2025) provides excellent extensibility through subclassing and modular builds, making it suitable for our needs. Challenges include migrating legacy ARB shaders to GLSL, ensuring CUDA-OpenGL interoperability, and optimizing for overlapping volumes.

## VTK Rendering Options for Key Data Types
VTK offers a variety of mappers and actors tailored to our data types, typically connected via a pipeline: data source → filter → mapper → actor → renderer. These can be GPU-accelerated for real-time performance.

### Volume Data (3D Texture with Volume Rendering)
For scalar fields in structured grids (e.g., vtkImageData), VTK uses ray casting or texture slicing:
- **vtkSmartVolumeMapper**: Adaptive, hardware-optimal (often GPU ray casting with 3D textures).
- **vtkGPUVolumeRayCastMapper**: GPU-based for high-speed rendering with shading.
- **vtkFixedPointVolumeRayCastMapper**: CPU fallback for precision.
- **vtkVolumeTextureMapper3D** (legacy): Direct 3D texture compositing.
- Supporting: **vtkVolume** (actor), **vtkVolumeProperty** (transfer functions), **vtkVolumeTexture** (data management).

### Arbitrary Geometry (Triangles, Quads with Colored Vertices or Texture Mapping)
For polygonal data (vtkPolyData):
- **vtkPolyDataMapper**: Core for tris/quads, supports vertex coloring and 2D/3D textures.
- **vtkCompositePolyDataMapper**: For multi-block datasets.
- **vtkDataSetMapper**: General-purpose with scalar/texture support.
- **vtkTexture**: For 2D/3D mapping; multi-texturing for layers.
- Supporting: **vtkActor**, **vtkProperty**.

### Tetrahedral Volumetric Meshes (vtkUnstructuredGrid with VTK_TETRA)
- **vtkUnstructuredGridVolumeRayCastMapper**: Ray casting for volumes.
- **vtkProjectedTetrahedraMapper**: GPU-accelerated projection.
- **vtkUnstructuredGridVolumeZSweepMapper**: Z-sweep rendering.
- Surface: **vtkUnstructuredGridMapper**, **vtkCellGridMapper**.
- Generation: **vtkDelaunay3D** filter.

### Hexahedral Volumetric Meshes (vtkUnstructuredGrid with VTK_HEXAHEDRON)
- **vtkUnstructuredGridVolumeRayCastMapper**: For hex volumes.
- **vtkUnstructuredGridVolumeZSweepMapper**: Z-sweep.
- **vtkSmartVolumeMapper**: Fallback for structured hexes.
- Surface: **vtkUnstructuredGridMapper**, **vtkCellGridMapper**/**vtkCompositeCellGridMapper**.
- Extensions: vtkHDF for polyhedral cells.

### Isosurface Rendering
- **Extraction-Based**: Generate vtkPolyData for surface rendering.
  - **vtkMarchingCubes**: For structured data.
  - **vtkContourFilter**: General, multi-value contours.
  - Optimized: **vtkFlyingEdges**, **vtkContourGrid**, **vtkContour3DLinearGrid**, **vtkmContour**.
  - **vtkExtractSurface**: For signed distance fields.
- **Direct Volume-Based**: Use transfer functions in **vtkSmartVolumeMapper**/**vtkGPUVolumeRayCastMapper** with ISOSURFACE_BLEND mode.

Integration Tips: Use filters like vtkImageImport for custom data; GPU for speed, CPU for fallback.

## Custom Renderer Integration and Performance Comparison
To adapt legacy renderers:

### OpenGL ARB Fragment Program-Based Volume Renderer
- **Approach**: Subclass **vtkAbstractVolumeMapper** or **vtkGPUVolumeRayCastMapper**; implement **Render()** with ARB logic (convert to GLSL for modern compatibility).
- **Shader Customization**: Use **AddShaderReplacement** or **SetFragmentShaderCode** for partial/full overrides.
- **Extension Build**: Create a VTK module via CMake; load dynamically.
- **Performance Comparison**: Use **vtkTimerLog** for frame times; test with profilers (Nsight). Expect gains in simple cases but validate against stock.

### CUDA-Ported Volren Raycaster (Volume and Isosurfaces)
- **Likelihood**: High (80-90%) via CUDA-OpenGL interop.
- **Approach**: Subclass **vtkAbstractVolumeMapper**; in **Render()**, register GL textures with CUDA, launch kernels to write color/depth, unmap, and composite quad with depth blending (use **gl_FragDepth**).
- **Alternative**: Subclass **vtkRenderPass** for post-processing compositing.
- **Depth Handling**: Preserve scene depth (**SetPreserveDepthBuffer(1)**); test/write via textures.
- **Build**: Compile VTK with CUDA; link custom module.

## Multi-Volume Scene Support
- **Non-Overlapping**: Multiple **vtkVolume** actors with individual mappers; auto-sort via **vtkFrustumCoverageCuller**.
- **Overlapping**: Use **vtkMultiVolume** with shared **vtkGPUVolumeRayCastMapper** for unified raycasting; supports properties per volume.
- **Custom Integration**: Extend mappers for multi-input; expect strong support with testing for artifacts.
- **Depth Buffer Management**: Volumes render post-opaque geometry; multi-volume computes unified depth. Use **vtkDualDepthPeelingPass** for translucents.
- **Level of Support**: Robust (up to several volumes); performance scales with GPU resources.

## Recommendations for Implementation Direction
1. **Prioritize GPU Pathways**: Start with stock VTK GPU mappers; integrate customs via subclasses for hybrid testing.
2. **Modular Build**: Develop custom mappers as VTK modules for easy swapping and benchmarking.
3. **Migration Steps**: Convert ARB to GLSL; ensure CUDA interop syncs with VTK state.
4. **Testing Focus**: Benchmark with **vtkTimerLog**; validate multi-volume overlaps and depth blending on varied datasets.
5. **Resources**: VTK docs/examples; upgrade to latest for multi-volume enhancements.
6. **Risks and Mitigations**: Overhead in interop—profile early; fallback to CPU for non-NVIDIA hardware.

This framework positions us to modernize our software efficiently, blending legacy strengths with VTK's ecosystem. Next steps: Prototype a custom mapper and multi-volume scene.