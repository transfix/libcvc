# recipes/libcvc/build.ps1 — build libcvc on Windows with CMake.
$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\..\_common\env-windows.ps1"

# Configure, build, and install with CMake. libcvc needs C++20 (the shared
# helper defaults to 17); ship the library only (tests + CUDA off), with the
# CGAL mesher and SDF paths on.
Invoke-CvcCMakeBuild @(
    '-DCMAKE_CXX_STANDARD=20',
    '-DBUILD_SHARED_LIBS=ON',
    '-DCVC_BUILD_TESTS=OFF',
    '-DCVC_ENABLE_CUDA=OFF',
    '-DDISABLE_CGAL=OFF',
    '-DCVC_ENABLE_MESHER=ON',
    '-DCVC_ENABLE_SDF=ON'
)
