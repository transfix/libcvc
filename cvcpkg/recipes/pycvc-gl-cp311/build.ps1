# cvcpkg/recipes/pycvc-gl-cp31X/build.ps1 — build the pycvc_gl scriptable-scene
# Python bindings (over cvcGL + VTK) standalone against an installed libcvc SDK
# (Windows). CVC_BUILD_PYCVC_GL=ON ⇒ scene module (the core module is the
# separate pycvc-cp31X recipe). One identical copy per interpreter column
# (interpreter selection comes from CMAKE_PREFIX_PATH; the builder exports
# CVC_PYTHON_INTERPRETER if a script ever needs to pin it explicitly,
# mirroring build.sh).
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
