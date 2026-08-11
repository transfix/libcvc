# cvcpkg/recipes/pycvc-gl-cp311-cuda/build.ps1 — build the pycvc_gl scene
# Python bindings standalone against an installed libcvc SDK (Windows, CUDA
# deps). CVC_BUILD_PYCVC_GL=ON ⇒ scene module (cvcGL + VTK); the core module
# is the separate pycvc-cp311-cuda recipe.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_SOURCE_DIR) { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }
if (-not $env:CVC_BUILD_TYPE) { $env:CVC_BUILD_TYPE = 'Release' }
if (-not $env:CVC_JOBS) { $env:CVC_JOBS = [Environment]::ProcessorCount }

$cmakeBuildType = if ($env:CVC_BUILD_TYPE.ToLower() -eq 'debug') { 'Debug' } else { 'Release' }

$args = @(
  '-G', 'Ninja',
  '-S', "$env:CVC_SOURCE_DIR/bindings/pycvc",
  '-B', $env:CVC_BUILD_DIR,
  "-DCMAKE_INSTALL_PREFIX=$env:CVC_INSTALL_DIR",
  "-DCMAKE_BUILD_TYPE=$cmakeBuildType",
  '-DCVC_BUILD_PYCVC_GL=ON'
)

if ($env:CVC_DEPS_PREFIX) {
  $args += "-DCMAKE_PREFIX_PATH=$env:CVC_DEPS_PREFIX"
}

& cmake @args
if ($LASTEXITCODE -ne 0) { throw 'cmake configure failed' }

& cmake --build $env:CVC_BUILD_DIR -j $env:CVC_JOBS
if ($LASTEXITCODE -ne 0) { throw 'cmake build failed' }

& cmake --install $env:CVC_BUILD_DIR
if ($LASTEXITCODE -ne 0) { throw 'cmake install failed' }
