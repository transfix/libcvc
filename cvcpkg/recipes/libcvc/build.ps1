# cvcpkg/recipes/libcvc/build.ps1 - build libcvc from the in-repo source tree.
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

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\..\_common\env-windows.ps1"

# Optional test gate (default OFF). `CVC_BUILD_TESTS=ON cvcpkg build libcvc ...`
# builds the Google Test suite into the BUILD tree and runs it below. It is OFF
# by default so the packaging/release path stays fast and network-light (Google
# Test is fetched via FetchContent). No test target has an install() rule and
# package.files selects only lib/ and include/, so the tests are never copied
# into CVC_INSTALL_DIR or the published bundle -- build-and-run only.
$cvcTests = if ($env:CVC_BUILD_TESTS) { $env:CVC_BUILD_TESTS } else { 'OFF' }

if ($cvcTests -eq 'ON') {
  # gtest_discover_tests runs each test exe at BUILD time (POST_BUILD) to
  # enumerate its cases, so the freshly-built cvc.dll (in the build tree's bin)
  # and the dependency DLLs (in the deps prefix's bin) must be locatable then.
  # env-windows.ps1 sets CMAKE_PREFIX_PATH but not PATH, so prepend both here.
  $testDllDirs = @(
    (Join-Path $env:CVC_DEPS_PREFIX 'bin'),
    (Join-Path $env:CVC_BUILD_DIR 'bin')
  )
  $env:PATH = ($testDllDirs -join ';') + ';' + $env:PATH
}

# CMAKE_PREFIX_PATH is deliberately NOT passed here: env-windows.ps1 sets it
# to CVC_DEPS_PREFIX;CVC_BUILD_PREFIX, and a -D would override that and drop
# the build prefix.
Invoke-CvcCMakeBuild @(
  "-DCVC_BUILD_TESTS=$cvcTests",
  # The `cvc` CLI ships as its own package (cvc-cli / cvc-cli-cuda); keep it OUT,
  # of the SDK bundle.,
  '-DCVC_BUILD_CLI=OFF',
  # CUDA opt-in: the `libcvc` recipe leaves this OFF; `libcvc-cuda` sets,
  # CVC_ENABLE_CUDA=ON in its build.matrix env to reuse this script for the GPU,
  # variant.,
  "-DCVC_ENABLE_CUDA=$(if ($env:CVC_ENABLE_CUDA) { $env:CVC_ENABLE_CUDA } else { 'OFF' })",
  '-DCVC_ENABLE_GRPC=OFF',
  # Enable the XMLRPC module so the bundle exports cvc::xmlrpc (+ libxmlrpc),
  # for downstream consumers like VolumeRover 2.x. Self-contained STATIC lib.,
  '-DCVC_USING_XMLRPC=ON'
)

if ($cvcTests -eq 'ON') {
  # Run the suite from the build tree, after the helper's build+install (the
  # shared helper fuses those two, so unlike build.sh the tests run post-install
  # here). ctest needs the same DLLs on PATH, which the block above arranged. A
  # failure throws, which fails the recipe, so cvcpkg never packages or
  # publishes a build whose tests did not pass.
  Push-Location $env:CVC_BUILD_DIR
  try {
    & ctest --output-on-failure -j $env:CVC_JOBS
    if ($LASTEXITCODE -ne 0) { throw 'ctest reported failures' }
  } finally {
    Pop-Location
  }
}
