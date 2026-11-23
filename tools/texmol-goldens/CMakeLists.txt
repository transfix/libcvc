# TexMol's PDB parser and GOABlur, built from a transfix/TexMol checkout at the pinned sha
# (texmol.sha), to make cvc::mol's T1 goldens (README.md). Never part of libcvc's build:
# make_goldens.py configures this directory on its own. TexMol's libraries are UT LGPL-2.1, as
# libcvc is; nothing built here is linked into libcvc.
cmake_minimum_required(VERSION 3.16)
project(texmol_goldens CXX)

set(TEXMOL_DIR "" CACHE PATH "A transfix/TexMol checkout at the sha in texmol.sha")
if(NOT EXISTS "${TEXMOL_DIR}/src/Blurmaps/GOABlur.cpp" OR
   NOT EXISTS "${TEXMOL_DIR}/src/PDBParser/parserPDBtoGOA.cpp")
  message(FATAL_ERROR "TEXMOL_DIR (${TEXMOL_DIR}) is not a TexMol checkout with PDBParser and "
                      "Blurmaps")
endif()
set(CMAKE_CXX_STANDARD 17)

# getBoundingBox, copied as it is out of BlurMapsDataManager.cpp (the rest of that file needs most of
# TexMol): from its definition to the next one's.
set(_bmdm "${TEXMOL_DIR}/src/Blurmaps/BlurMapsDataManager.cpp")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_bmdm}")
file(READ "${_bmdm}" _text)
string(FIND "${_text}" "void BlurMapsDataManager::getBoundingBox(" _from)
string(FIND "${_text}" "SimpleVolumeData* BlurMapsDataManager::getMultiLevelBlurredVolume(" _to)
if(_from LESS 0 OR _to LESS_EQUAL _from)
  message(FATAL_ERROR "getBoundingBox not found in ${_bmdm}: has texmol.sha moved?")
endif()
math(EXPR _len "${_to} - ${_from}")
string(SUBSTRING "${_text}" ${_from} ${_len} _fn)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/get_bounding_box.cpp"
  "// Generated from ${_bmdm}: BlurMapsDataManager::getBoundingBox, as it is.\n"
  "#include <Blurmaps/BlurMapsDataManager.h>\n#include <cmath>\nusing namespace std;\n${_fn}")

file(GLOB _pdbparser "${TEXMOL_DIR}/src/PDBParser/*.cpp")
file(GLOB _usefulmath "${TEXMOL_DIR}/src/UsefulMath/*.cpp")
add_executable(blur_legacy blur_driver.cpp
  ${_pdbparser}
  ${_usefulmath}
  ${TEXMOL_DIR}/src/Utility/utility.cpp
  ${TEXMOL_DIR}/src/ByteOrder/ByteSwapping.cpp
  ${TEXMOL_DIR}/src/ColorManager/Color.cpp
  ${TEXMOL_DIR}/src/ColorManager/ColorGenerator.cpp
  ${TEXMOL_DIR}/src/ColorManager/PastelColorGenerator.cpp
  ${TEXMOL_DIR}/src/SimpleVolumeData/SimpleVolumeData.cpp
  ${TEXMOL_DIR}/src/Blurmaps/GOABlur.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/get_bounding_box.cpp)
# The shim first, so <Blurmaps/BlurMapsDataManager.h> resolves to it.
target_include_directories(blur_legacy PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/shim ${TEXMOL_DIR}/inc)
# Legacy code as it is: its warnings off, never -Werror, and no floating-point contraction, so the
# sums are the ones TexMol's own builds made.
if(WIN32)
  target_compile_definitions(blur_legacy PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
endif()
if(MSVC)
  target_compile_options(blur_legacy PRIVATE /w /fp:precise)
else()
  target_compile_options(blur_legacy PRIVATE -w -ffp-contract=off)
endif()
