#
# Modern macro for setting up Boost for a target
# Requires CMake 3.15+ for better Boost support
#

function(SetupBoost TargetName)
  if(EMSCRIPTEN)
    # The wasm Boost bundle is static-only (wasm builds force CVC_LINK=static).
    set(Boost_USE_STATIC_LIBS ON)
  else()
    set(Boost_USE_STATIC_LIBS OFF)
  endif()
  set(Boost_USE_MULTITHREADED ON)
  
  # Find required Boost components
  # system and chrono are header-only on newer Boost (>=1.69/1.72) but
  # still compiled on older versions — use OPTIONAL_COMPONENTS so the
  # lookup succeeds either way.
  find_package(Boost 1.58 REQUIRED COMPONENTS 
    thread 
    date_time 
    regex 
    filesystem
    OPTIONAL_COMPONENTS
    system
    chrono
  )
  
  if(Boost_FOUND)
    message(STATUS "Boost version: ${Boost_VERSION}")
    message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost libraries: ${Boost_LIBRARIES}")
    
    # Use modern imported targets so that the installed cvc export file
    # references Boost::thread etc. instead of hard-coded absolute paths.
    # Wrap each include dir in $<BUILD_INTERFACE:...> so a bundled
    # toolchain (e.g. libcvc-deps under a parent project's build tree)
    # does not leak a build-dir path into the installed cvc target's
    # INTERFACE_INCLUDE_DIRECTORIES.
    set(_boost_targets Boost::thread Boost::date_time Boost::regex Boost::filesystem)
    foreach(_bt IN ITEMS Boost::system Boost::chrono)
      if(TARGET ${_bt})
        list(APPEND _boost_targets ${_bt})
      endif()
    endforeach()
    target_link_libraries(${TargetName} PUBLIC ${_boost_targets})
    
    # Platform-specific definitions
    if(NOT EMSCRIPTEN)
      target_compile_definitions(${TargetName} PUBLIC BOOST_ALL_DYN_LINK)
    endif()
    
    if(MSVC)
      target_compile_definitions(${TargetName} PRIVATE 
        _VARIADIC_MAX=10
        BOOST_ALL_NO_LIB
      )
    endif()
  else()
    message(FATAL_ERROR 
      "Boost not found! Please set BOOST_ROOT, BOOST_INCLUDEDIR, or BOOST_LIBRARYDIR environment variables.")
  endif()
endfunction(SetupBoost)
