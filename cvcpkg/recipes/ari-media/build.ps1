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

$cmakeArgs = @('-DARI_WITH_MEDIA=ON')
# FFmpeg in a prefix of its own (CVC_FFMPEG_PREFIX, as publish-cvcpkg.yml installs it), named
# outright: that prefix carries its own zlib, libpng and the like, so it stays off CMAKE_PREFIX_PATH.
if ($env:CVC_FFMPEG_PREFIX) {
    $ffmpeg = $env:CVC_FFMPEG_PREFIX -replace '\\', '/'
    $cmakeArgs += "-DCVC_FFMPEG_INCLUDE_DIR=$ffmpeg/include"
    foreach ($lib in @('avformat', 'avcodec', 'swscale', 'swresample', 'avutil')) {
        $cmakeArgs += "-DCVC_FFMPEG_$($lib)_LIBRARY=$ffmpeg/lib/$lib.lib"
    }
}
Invoke-CvcCMakeBuild $cmakeArgs
# The install holds bin/ari.exe too; this package is bin/ari-media.exe alone.
Remove-Item -Force -ErrorAction SilentlyContinue "$env:CVC_INSTALL_DIR/bin/ari.exe"
