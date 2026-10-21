# cvcpkg/recipes/libcvc-examples/build.ps1 — build libcvc's example programs
# (examples/: the cvcGL scene demos, the GRL-SNAM nav demos and the headless nav
# CLIs) on Windows via MSVC. This mirrors recipes/cvcgl/build.ps1; the Linux path
# is build.sh. The examples build with
#   cvcpkg build libcvc-examples --prefix ./deps
# on Windows exactly as on Linux.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_SOURCE_DIR) { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }

# This recipe configures the src/cvcGL subtree (with examples ON, which adds the
# repo's examples/), same as the cvcgl recipe and build.sh. Invoke-CvcCMakeBuild
# configures $env:CVC_SOURCE_DIR.
$env:CVC_SOURCE_DIR = "$env:CVC_SOURCE_DIR/src/cvcGL"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\..\_common\env-windows.ps1"

# cvcGL is built STATIC into the example binaries (build.sh does the same): no
# soname coupling to the cvcgl SDK bundle; libcvc/VTK/Boost stay shared from the
# deps prefix. CGAL_Boost_USE_STATIC_LIBS=OFF asks find_package(Boost) for the
# shared variant cvcpkg actually ships (see build.sh for the full rationale).
# These -D args come AFTER the helper's defaults, so BUILD_SHARED_LIBS=OFF here
# overrides the default shared setting for the examples build.
Invoke-CvcCMakeBuild @(
    '-DCVC_BUILD_EXAMPLES=ON',
    '-DBUILD_SHARED_LIBS=OFF',
    '-DCGAL_Boost_USE_STATIC_LIBS=OFF'
)

# ABI guard (see build.sh for the rationale): the demos link the SHARED libcvc
# from the deps prefix but were compiled against the in-tree cvc/nav headers. Run
# the headless nav_abi_smoke against that libcvc so a stale/lagging one — its
# sim_world layout out of sync with these headers — fails HERE with a clear
# message instead of shipping demos that segfault at launch. A missing binary is
# an error as well (build.sh says why).
$smoke = Join-Path $env:CVC_BUILD_DIR 'examples\nav_abi_smoke.exe'
if (-not (Test-Path $smoke)) {
    throw "the libcvc ABI guard $smoke was not built"
}
Write-Host 'libcvc-examples: running nav_abi_smoke (libcvc ABI guard)'
if ($env:CVC_DEPS_PREFIX) {
    $env:PATH = (Join-Path $env:CVC_DEPS_PREFIX 'bin') + ';' + $env:PATH
}
& $smoke
if ($LASTEXITCODE -ne 0) {
    throw ('nav_abi_smoke failed: the linked libcvc''s sim_world ABI does not match the ' +
           'example headers — rebuild/republish libcvc so the examples link a matching version')
}
