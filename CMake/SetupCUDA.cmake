#
# Modern CUDA setup for trans-cvc
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
  
  # Link CUDA runtime
  target_link_libraries(${TargetName} PUBLIC 
    CUDA::cudart
  )
  
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
  
  # Debug vs Release settings
  target_compile_options(${TargetName} PRIVATE
    $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CONFIG:Debug>>:
      -G              # Generate debug info for device code
      -lineinfo       # Generate line number info
    >
    $<$<AND:$<COMPILE_LANGUAGE:CUDA>,$<CONFIG:Release>>:
      --use_fast_math # Use fast math for better performance
    >
  )
  
  message(STATUS "CUDA configuration complete for ${TargetName}")
endfunction(SetupCUDA)
