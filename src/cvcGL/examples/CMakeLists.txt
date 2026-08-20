# cvcGL examples — standalone C++ programs that drive the scene graph directly.
#
# Each links the in-tree cvcGL target plus the VTK modules cvcGL does NOT already
# pull for a consumer: RenderingUI (the onscreen interactor factory) and IOImage
# (SceneRenderer::writePNG's vtkPNGWriter). InteractionStyle/RenderingOpenGL2 come
# transitively from cvcGL.

find_package(VTK REQUIRED COMPONENTS RenderingUI IOImage)
find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(lsystem_forest lsystem_forest.cpp)
target_link_libraries(lsystem_forest PRIVATE cvcGL ${VTK_LIBRARIES} Boost::program_options)
vtk_module_autoinit(TARGETS lsystem_forest MODULES ${VTK_LIBRARIES})

if(EMSCRIPTEN)
  # Browser build: the demo keeps its own while-loop and yields to the event
  # loop via emscripten_sleep, which needs Asyncify. VTK's patched FindOpenGL
  # already injects the WebGL2/GLES3 link flags; RenderingUI injects its
  # --js-library glue. Memory must be growable — the forest + sea/sky volumes
  # comfortably exceed the 16 MB default heap.
  target_link_options(lsystem_forest PRIVATE
    -sASYNCIFY=1
    -sASYNCIFY_STACK_SIZE=131072
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=2097152
    -sEXIT_RUNTIME=0
  )
  if(CVC_WASM_PTHREADS)
    # Pre-spawn workers so std::thread creation (the scene's state_publisher)
    # doesn't stall waiting for a worker to boot on the busy main thread.
    target_link_options(lsystem_forest PRIVATE -sPTHREAD_POOL_SIZE=4)
  endif()
  # Keep the output dir directly servable: python3 -m http.server -d <here>.
  add_custom_command(TARGET lsystem_forest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_CURRENT_SOURCE_DIR}/wasm/index.html
      $<TARGET_FILE_DIR:lsystem_forest>/index.html)
endif()
