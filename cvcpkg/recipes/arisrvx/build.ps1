# cvcpkg/recipes/arisrvx/build.ps1 — build `arisrvx`, the arisrv relay with an embedded Python,
# standalone against an installed libcvc SDK and the deps prefix's python312, on Windows. As
# build.sh does: src/arisrv with -DARISRV_WITH_PYTHON=ON (main.cpp plus src/ariadne_python's engine
# and its server part), linking the pre-installed cvc::cvc and python312.dll, so libcvc is NOT
# rebuilt. Installs just bin/arisrvx.exe.
#
# _common/env-windows.ps1 supplies the MSVC environment (CC/CXX=cl, vcvars when cl.exe is not on
# PATH, CMAKE_PREFIX_PATH over the deps roots), as for arisrv's build.ps1. Its Invoke-CvcCMakeBuild
# is not used: it builds and installs every target, which would put arisrv.exe in this bundle too,
# so the same steps run inline for the arisrvx target and install component alone.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_SOURCE_DIR) { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\..\_common\env-windows.ps1"

# The interpreter is named outright, as build.sh names it: the first <root>\python.exe of the deps
# roots (CVC_DEPS_PREFIX, then CVC_BUILD_PREFIX, each possibly ;-separated) that is a 3.12, since
# arisrvx must embed the one it runs beside. FindPython3 left to itself takes the first python.exe
# under CMAKE_PREFIX_PATH, whatever its version, and with none there a registry Python.
$pyRoots = @($env:CVC_DEPS_PREFIX, $env:CVC_BUILD_PREFIX) |
    Where-Object { $_ } | ForEach-Object { $_ -split ';' } |
    Where-Object { $_ } | Select-Object -Unique
$pyRoot = $null
foreach ($root in $pyRoots) {
  $candidate = Join-Path $root 'python.exe'
  if (-not (Test-Path -LiteralPath $candidate)) { continue }
  $found = & $candidate -I -c 'import sys; print(*sys.version_info[:2], sep=chr(46))'
  if ($LASTEXITCODE -eq 0 -and "$found".Trim() -eq '3.12') { $pyRoot = $root; break }
}
if (-not $pyRoot) { throw "no python.exe for Python 3.12 at any deps root: $($pyRoots -join ', ')" }
# Installed, bin\arisrvx.exe loads python312.dll from bin\. python312 bundles before
# 3.12.10+cvc.21 (cy-pca/cvcpkg#542) kept it only at the prefix root, beside python.exe, so an
# arisrvx built against one would die at launch with STATUS_DLL_NOT_FOUND (0xC0000135).
if (-not (Test-Path -LiteralPath (Join-Path $pyRoot 'bin\python312.dll'))) {
  throw "$pyRoot has no bin\python312.dll: install python312 3.12.10+cvc.22 or later"
}
$pyExe = Join-Path $pyRoot 'python.exe'

# MSVC builds have no use for anything under C:\msys64\, and CMake's find_* search each PATH dir's
# ../include and ../lib, where MinGW's gcc-only headers would be found (see Invoke-CvcCMakeBuild).
$origPath = $env:PATH
$env:PATH = ($env:PATH -split ';' |
    Where-Object { $_ -notmatch '(?i)\\msys64\\' -and $_ -notmatch '(?i)\\msys32\\' }) -join ';'
try {
  & cmake -G Ninja -S "$env:CVC_SOURCE_DIR/src/arisrv" -B $env:CVC_BUILD_DIR `
    "-DCMAKE_INSTALL_PREFIX=$env:CVC_INSTALL_DIR" `
    "-DCMAKE_BUILD_TYPE=$cmakeBuildType" `
    "-DBUILD_SHARED_LIBS=$buildSharedLibs" `
    "-DCMAKE_MSVC_RUNTIME_LIBRARY=$msvcRuntime" `
    '-DCMAKE_POLICY_VERSION_MINIMUM=3.5' `
    '-DCMAKE_C_COMPILER=cl' `
    '-DCMAKE_CXX_COMPILER=cl' `
    '-DARISRV_WITH_PYTHON=ON' `
    "-DPython3_EXECUTABLE=$pyExe" `
    '-DPython3_FIND_REGISTRY=NEVER'
  if ($LASTEXITCODE -ne 0) { throw 'cmake configure failed' }

  & cmake --build $env:CVC_BUILD_DIR -j $env:CVC_JOBS --target arisrvx
  if ($LASTEXITCODE -ne 0) { throw 'cmake build failed' }

  & cmake --install $env:CVC_BUILD_DIR --component arisrvx
  if ($LASTEXITCODE -ne 0) { throw 'cmake install failed' }
} finally {
  $env:PATH = $origPath
}

Invoke-CvcRewriteInstallPaths
