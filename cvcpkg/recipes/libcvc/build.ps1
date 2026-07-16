# cvcpkg/recipes/libcvc/build.ps1 - build libcvc from the in-repo source tree.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_SOURCE_DIR) { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR) { throw 'CVC_BUILD_DIR must be set' }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }
if (-not $env:CVC_BUILD_TYPE) { $env:CVC_BUILD_TYPE = 'Release' }
if (-not $env:CVC_LINK) { $env:CVC_LINK = 'shared' }
if (-not $env:CVC_JOBS) { $env:CVC_JOBS = [Environment]::ProcessorCount }

$cmakeBuildType = if ($env:CVC_BUILD_TYPE.ToLower() -eq 'debug') { 'Debug' } else { 'Release' }
$buildSharedLibs = if ($env:CVC_LINK -eq 'static') { 'OFF' } else { 'ON' }

$args = @(
  '-G', 'Ninja',
  '-S', $env:CVC_SOURCE_DIR,
  '-B', $env:CVC_BUILD_DIR,
  "-DCMAKE_INSTALL_PREFIX=$env:CVC_INSTALL_DIR",
  "-DCMAKE_BUILD_TYPE=$cmakeBuildType",
  "-DBUILD_SHARED_LIBS=$buildSharedLibs",
  '-DCVC_BUILD_TESTS=OFF',
  '-DCVC_ENABLE_CUDA=OFF',
  '-DCVC_ENABLE_GRPC=OFF'
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
