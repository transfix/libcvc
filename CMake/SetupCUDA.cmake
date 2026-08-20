#
# Modern CUDA setup for libcvc
# Requires CMake 3.17+ for native CUDA language support
#

function(SetupCUDA TargetName)
  if(NOT CVC_ENABLE_CUDA)
    return()
  endif()
  
  # Ensure CUDA language is enabled
  if(NOT CMAKE_CUDA_COMPILER)
    message(WARNING "CUDA compiler not found despite CVC_ENABLE_CUDA=ON")
    return()
  endif()
  
  message(STATUS "Configuring CUDA for target: ${TargetName}")
  
  # CUDA include directories (typically found automatically)
  if(CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES)
    target_include_directories(${TargetName} 
      PRIVATE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    )
  endif()
  
  # Enable separable compilation for device code
  set_target_properties(${TargetName} PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
  )
  
  # Find and link CUDA toolkit libraries
  find_package(CUDAToolkit REQUIRED)

  # Link CUDA runtime.
  #
  # CMake exposes BOTH CUDA::cudart (the *shared* runtime) and
  # CUDA::cudart_static, regardless of CMAKE_CUDA_RUNTIME_LIBRARY.
  # That global variable only affects how nvcc-compiled translation
  # units pull in the runtime; an explicit target_link_libraries on
  # CUDA::cudart still drags in cudart64_12X.dll and forces end users
  # to install the CUDA toolkit just to launch the binary.
  #
  # For redistribution we want the static cudart so the only thing the
  # end user needs is the NVIDIA driver. Honor CMAKE_CUDA_RUNTIME_LIBRARY
  # when set, otherwise default to Static.
  if(NOT DEFINED CMAKE_CUDA_RUNTIME_LIBRARY OR CMAKE_CUDA_RUNTIME_LIBRARY STREQUAL "Static")
    if(TARGET CUDA::cudart_static)
      target_link_libraries(${TargetName} PUBLIC CUDA::cudart_static)
    elseif(TARGET CUDA::cudart)
      target_link_libraries(${TargetName} PUBLIC CUDA::cudart)
    else()
      target_link_libraries(${TargetName} PUBLIC ${CUDA_LIBRARIES})
      target_include_directories(${TargetName} PRIVATE ${CUDA_INCLUDE_DIRS})
    endif()
  else()
    if(TARGET CUDA::cudart)
      target_link_libraries(${TargetName} PUBLIC CUDA::cudart)
    else()
      target_link_libraries(${TargetName} PUBLIC ${CUDA_LIBRARIES})
      target_include_directories(${TargetName} PRIVATE ${CUDA_INCLUDE_DIRS})
    endif()
  endif()
  
  # Optional: Link additional CUDA libraries as needed
  # target_link_libraries(${TargetName} PUBLIC 
  #   CUDA::cublas
  #   CUDA::cufft
  # )
  
  # CUDA compile options
  target_compile_options(${TargetName} PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:
      --expt-relaxed-constexpr  # Allow constexpr functions in device code
      --expt-extended-lambda    # Allow extended lambda features
    >
  )
  
  # Platform-specific CUDA settings
  if(UNIX AND NOT APPLE)
    # Linux-specific CUDA options
    target_compile_options(${TargetName} PRIVATE
      $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler -fPIC>
    )
  endif()
  
  # Debug settings (device debug/line info).
  target_compile_options(${TargetName} PRIVATE
    $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CONFIG:Debug>>:
      -G              # Generate debug info for device code
      -lineinfo       # Generate line number info
    >
  )
  # NOTE: --use_fast_math is deliberately NOT applied target-wide here. It maps
  # expf/sinf/etc. to their approximate intrinsics (via __CUDA_FAST_MATH__) and
  # sets ftz/imprecise-div/sqrt, which cannot be undone per source — so a
  # target-wide flag would silently break the float-equivalence contract of any
  # cvc::nav parity .cu (docs/CVCNAV_CUDA_ASSESSMENT.md, verified). Volume
  # rendering that wants fast math opts in per source (see src/cvc/CMakeLists.txt).
  
  message(STATUS "CUDA configuration complete for ${TargetName}")
endfunction(SetupCUDA)
