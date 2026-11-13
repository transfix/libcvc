# cvcpkg/recipes/pycvc-cp31X/build.ps1 — build the core pycvc Python bindings
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
# One script serves every interpreter column, as build.sh does, and the
# version is derived below from the recipe's column. Each pycvc-cp31X recipe
# dir carries an identical copy of this script.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_SOURCE_DIR) { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }

# The recipe's interpreter column (python312 -> 3.12), never guessed. Three
# things can name it: CVC_PYTHON_INTERPRETER, which the native cvcpkg builder
# exports from the recipe's `python:` block; CVC_COMPONENT, the recipe name
# (e.g. pycvc-cp312), which a Windows-host delegated build (cvcpkg's winhost)
# exports when it does not export the first; and this script's recipe dir.
# Every one that names a column must name the same one, and a build where none
# does stops here. A trailing free-threaded `t` is accepted defensively; no `t`
# columns exist for pycvc.
$recipeDirName = Split-Path -Leaf (Split-Path -Parent $MyInvocation.MyCommand.Path)
if ($env:CVC_PYTHON_INTERPRETER -and $env:CVC_PYTHON_INTERPRETER -notmatch '^python3\d+t?$') {
  throw "CVC_PYTHON_INTERPRETER is '$env:CVC_PYTHON_INTERPRETER', not a pythonXY name"
}
$colNames = @()
$colDigits = @()
foreach ($name in @($env:CVC_PYTHON_INTERPRETER, $env:CVC_COMPONENT, $recipeDirName)) {
  if ("$name" -match '^python(3\d+)t?$|-cp(3\d+)(-|$)') {
    $colNames += $name
    $colDigits += $(if ($Matches[1]) { $Matches[1] } else { $Matches[2] })
  }
}
$colDigits = @($colDigits | Select-Object -Unique)
if ($colDigits.Count -eq 0) {
  throw ("no Python column: CVC_PYTHON_INTERPRETER is unset, and neither CVC_COMPONENT " +
         "('$env:CVC_COMPONENT') nor the recipe dir ('$recipeDirName') names a -cp3XY one")
}
if ($colDigits.Count -gt 1) {
  throw "the Python column is ambiguous: $($colNames -join ', ') name different interpreters"
}
$pyInterp = "python$($colDigits[0])"
$pyVer = '3.' + $colDigits[0].Substring(1)

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
Write-Host "-- pycvc: Python $pyVer at $pyExe, numpy headers at $npyInc"

# CMAKE_PREFIX_PATH is deliberately NOT passed here: env-windows.ps1 sets it
# to CVC_DEPS_PREFIX;CVC_BUILD_PREFIX, and a -D would override that and drop
# the build prefix.
Invoke-CvcCMakeBuild @(
  '-DCVC_BUILD_PYCVC_GL=OFF',
  "-DPython3_EXECUTABLE=$pyExe",
  '-DPython3_FIND_REGISTRY=NEVER',
  "-DPython3_NumPy_INCLUDE_DIR=$("$npyInc".Trim())"
)
