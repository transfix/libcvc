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
#
# One script serves every interpreter column, as build.sh does: the cvcpkg
# builder exports CVC_PYTHON_INTERPRETER (e.g. python312) from the recipe's
# `python:` block, and the version is derived from it below (3.11 fallback).
# Each pycvc-gl-cp31X recipe dir carries an identical copy of this script.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_SOURCE_DIR) { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }

# Dotted Python version of the recipe's interpreter column (python311 -> 3.11,
# python312 -> 3.12, ...). A trailing free-threaded `t` is stripped
# defensively; no `t` columns exist for pycvc-gl.
$pyInterp = if ($env:CVC_PYTHON_INTERPRETER) { $env:CVC_PYTHON_INTERPRETER } else { 'python311' }
$pyDigits = ($pyInterp -replace '^python', '') -replace 't$', ''
$pyVer = '{0}.{1}' -f $pyDigits.Substring(0, 1), $pyDigits.Substring(1)

# This recipe builds a subdirectory of the checkout. Invoke-CvcCMakeBuild
# configures $env:CVC_SOURCE_DIR, so point it at the subtree.
$env:CVC_SOURCE_DIR = "$env:CVC_SOURCE_DIR/bindings/pycvc"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\..\_common\env-windows.ps1"

# Pin the interpreter, and its numpy headers, to this column's cvcpkg Python:
# the first <root>\python.exe of the deps roots (CVC_DEPS_PREFIX, then
# CVC_BUILD_PREFIX, each possibly ;-separated) that reports $pyVer. Left to
# itself, FindPython3 takes the first python.exe under CMAKE_PREFIX_PATH,
# whatever its version, and with none there a registry Python: the module
# would build against whatever Python the builder has, a silent hermeticity
# break. So a prefix without this column's Python fails here instead, and
# Python3_FIND_REGISTRY=NEVER keeps the registry out.
$pyRoots = @($env:CVC_DEPS_PREFIX, $env:CVC_BUILD_PREFIX) |
    Where-Object { $_ } | ForEach-Object { $_ -split ';' } |
    Where-Object { $_ } | Select-Object -Unique
$pyExe = $null
foreach ($root in $pyRoots) {
  $candidate = Join-Path $root 'python.exe'
  if (-not (Test-Path -LiteralPath $candidate)) { continue }
  $found = & $candidate -I -c 'import sys; print(*sys.version_info[:2], sep=chr(46))'
  if ($LASTEXITCODE -eq 0 -and "$found".Trim() -eq $pyVer) { $pyExe = $candidate; break }
}
if (-not $pyExe) {
  throw "no python.exe for Python $pyVer ($pyInterp) at any deps root: $($pyRoots -join ', ')"
}
$npyInc = & $pyExe -I -c 'import numpy; print(numpy.get_include())'
if ($LASTEXITCODE -ne 0 -or -not $npyInc) { throw "numpy is not importable from $pyExe" }
Write-Host "-- pycvc-gl: Python $pyVer at $pyExe, numpy headers at $npyInc"

# CMAKE_PREFIX_PATH is deliberately NOT passed here: env-windows.ps1 sets it
# to CVC_DEPS_PREFIX;CVC_BUILD_PREFIX, and a -D would override that and drop
# the build prefix.
Invoke-CvcCMakeBuild @(
  # Scene module ONLY, as build.sh builds it: the core pycvc module is the
  # matching pycvc-cp31X package (a runtime dependency), so this one installs
  # just the pycvc_gl package dir and the two live side by side.
  '-DCVC_BUILD_PYCVC_CORE=OFF',
  '-DCVC_BUILD_PYCVC_GL=ON',
  "-DPython3_EXECUTABLE=$pyExe",
  '-DPython3_FIND_REGISTRY=NEVER',
  "-DPython3_NumPy_INCLUDE_DIR=$("$npyInc".Trim())"
)
