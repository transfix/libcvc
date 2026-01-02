#
# Modern macro for setting up FFTW (float or double version)
#

function(SetupFFTW)
  option(USE_FFTWD "Use double precision FFTW if found" ON)
  option(USE_FFTWF "Use single precision FFTW if found" ON)
  
  mark_as_advanced(USE_FFTWD USE_FFTWF)

  find_package(FFTW)

  # Set the FFTW_LIB variable to the version we found
  # Prefer the double precision version if both found
  set(FFTW_FOUND FALSE PARENT_SCOPE)

  if(USE_FFTWF AND FFTWF_FOUND)
    set(FFTW_LIB ${FFTWF_LIB} PARENT_SCOPE)
    set(FFTW_FOUND TRUE PARENT_SCOPE)
    message(STATUS "FFTW: Using single precision (float)")
  endif()

  if(USE_FFTWD AND FFTWD_FOUND)
    set(FFTW_LIB ${FFTWD_LIB} PARENT_SCOPE)
    set(FFTW_FOUND TRUE PARENT_SCOPE)
    message(STATUS "FFTW: Using double precision")
  endif()

  if(FFTW_FOUND)
    message(STATUS "FFTW library: ${FFTW_LIB}")
  else()
    message(STATUS "FFTW not found - FFTW features disabled")
  endif()

endfunction(SetupFFTW)
