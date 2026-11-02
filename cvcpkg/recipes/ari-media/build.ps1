# cvcpkg/recipes/ari-media/build.ps1 — build ari-media (ari with cvcMedia's SDL3 audio) standalone
# against an installed libcvc SDK and the sdl3 package, on Windows. Compiles src/ari and
# src/cvcMedia (static, linked into the binary). Installs just bin/ari-media.exe. The MSVC
# environment comes from the shared helper, as the ari recipe's build.ps1 explains.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_SOURCE_DIR) { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }

$env:CVC_SOURCE_DIR = "$env:CVC_SOURCE_DIR/src/ari"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\..\_common\env-windows.ps1"

Invoke-CvcCMakeBuild @('-DARI_WITH_MEDIA=ON')
# The install holds bin/ari.exe too; this package is bin/ari-media.exe alone.
Remove-Item -Force -ErrorAction SilentlyContinue "$env:CVC_INSTALL_DIR/bin/ari.exe"
