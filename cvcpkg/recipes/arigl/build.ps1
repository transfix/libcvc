# cvcpkg/recipes/arigl/build.ps1 — build the `arigl` GL runner standalone against the installed
# libcvc and cvcGL SDKs, on Windows. Compiles only src/arigl and links the pre-installed
# cvc::cvcGL (and cvc::cvc through it), so neither is rebuilt. Installs just bin/arigl.exe.
#
# Windows builds go through the shared cvcpkg helper (_common/env-windows.ps1), as ari's build.ps1
# does: it forces CC/CXX=cl, imports the MSVC developer environment when cl.exe is not already on
# PATH, strips MinGW/MSYS dirs from PATH for the CMake run, and passes
# -DCMAKE_MSVC_RUNTIME_LIBRARY so the CRT matches the link mode. Every cvcpkg windows bundle,
# libcvc's cvc.dll and cvcGL's included, is MSVC-built, so a MinGW compiler picked off PATH could
# not link them.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_SOURCE_DIR) { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }

# This recipe builds a subdirectory of the checkout. Invoke-CvcCMakeBuild
# configures $env:CVC_SOURCE_DIR, so point it at the subtree.
$env:CVC_SOURCE_DIR = "$env:CVC_SOURCE_DIR/src/arigl"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\..\_common\env-windows.ps1"

# CMAKE_PREFIX_PATH is deliberately NOT passed here: env-windows.ps1 sets it
# to CVC_DEPS_PREFIX;CVC_BUILD_PREFIX (the installed libcvc and cvcGL SDKs and
# VTK are in the deps prefix), and a -D would override that and drop the build prefix.
Invoke-CvcCMakeBuild @()
