# cvcGL examples — standalone C++ programs that drive the scene graph directly.
#
# Each links the in-tree cvcGL target plus the VTK modules cvcGL does NOT already
# pull for a consumer: RenderingUI (the onscreen interactor factory) and IOImage
# (SceneRenderer::writePNG's vtkPNGWriter). InteractionStyle/RenderingOpenGL2 come
# transitively from cvcGL.

find_package(VTK REQUIRED COMPONENTS RenderingUI IOImage)

add_executable(lsystem_forest lsystem_forest.cpp)
target_link_libraries(lsystem_forest PRIVATE cvcGL ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS lsystem_forest MODULES ${VTK_LIBRARIES})
