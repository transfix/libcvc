# cvcpkg/recipes/pycvc-gl-cp31X/build.ps1 — build the pycvc_gl scriptable-scene
#
# Windows builds go through the shared cvcpkg helper, which is the whole
# point of _common/env-windows.ps1: it forces CC/CXX=cl, imports the MSVC
# developer environment when cl.exe is not already on PATH, strips
# MinGW/MSYS dirs from PATH for the CMake run, and passes
# -DCMAKE_MSVC_RUNTIME_LIBRARY so the CRT matches the link mode.
#
# This is an ABI requirement, not a preference. Every cvcpkg windows dep
# bundle is MSVC-built (gmp.lib, fftw3.lib, ... — MinGW would emit
# libgmp.a), so a gcc-compiled object could not link them. Hand-rolling
# `-G Ninja` with no compiler pin let CMake take the first c++ on PATH,
# which on the GitHub runner images is C:\mingw64\bin\c++.exe.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_SOURCE_DIR) { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }

# This recipe builds a subdirectory of the checkout. Invoke-CvcCMakeBuild
# configures $env:CVC_SOURCE_DIR, so point it at the subtree.
$env:CVC_SOURCE_DIR = "$env:CVC_SOURCE_DIR/bindings/pycvc"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\..\_common\env-windows.ps1"

# CMAKE_PREFIX_PATH is deliberately NOT passed here: env-windows.ps1 sets it
# to CVC_DEPS_PREFIX;CVC_BUILD_PREFIX, and a -D would override that and drop
# the build prefix.
Invoke-CvcCMakeBuild @(
  '-DCVC_BUILD_PYCVC_GL=ON'
)
