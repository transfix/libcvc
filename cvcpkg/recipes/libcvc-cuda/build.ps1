# cvcpkg/recipes/libcvc-cuda/build.ps1 - build libcvc with CUDA, through the libcvc recipe's script.
#
# The GPU variant is the CPU SDK with CVC_ENABLE_CUDA=ON and nothing else changed, so this runs
# cvcpkg/recipes/libcvc/build.ps1 (cl and Ninja, through _common/env-windows.ps1) instead of keeping
# a copy of it. The copy it replaces had drifted: it built without gRPC, WebSocket or XMLRPC, and did
# not keep arisrv out of the install. The publish workflow's Windows CUDA job runs this script too,
# so `cvcpkg build libcvc-cuda` and the published bundle come from the same flags.
#
# With Ninja, CMake compiles .cu files by running nvcc itself, with cl as the host compiler. So nvcc
# must be on PATH (or named by CUDACXX), and cl must be a Visual Studio that the toolkit's nvcc
# accepts.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }

$env:CVC_ENABLE_CUDA = 'ON'
& (Join-Path $PSScriptRoot '..\libcvc\build.ps1')

# See build.sh beside this for why: CVC_ENABLE_CUDA=ON is a request, and a build that found no nvcc
# still succeeds, as a CPU-only libcvc. CMAKE_CUDA_COMPILER is set only when nvcc was found.
$cache = Join-Path $env:CVC_BUILD_DIR 'CMakeCache.txt'
$line = Select-String -Path $cache -Pattern '^CMAKE_CUDA_COMPILER:' | Select-Object -First 1
$cc = if ($line) { ($line.Line -split '=', 2)[1] } else { '' }
if (-not $cc -or $cc -match 'NOTFOUND') {
  throw "libcvc-cuda: CUDA did not enable (CMAKE_CUDA_COMPILER='$cc'): this is a CPU-only build"
}
Write-Host "libcvc-cuda: CUDA compiler $cc"

# And device code for several GPU generations, not one (see build.sh).
$line = Select-String -Path $cache -Pattern '^CMAKE_CUDA_ARCHITECTURES:' | Select-Object -First 1
$arches = if ($line) { ($line.Line -split '=', 2)[1] } else { '' }
Write-Host "libcvc-cuda: CUDA architectures $arches"
if ($arches -notmatch ';') {
  throw "libcvc-cuda: CMAKE_CUDA_ARCHITECTURES='$arches' is a single architecture"
}
