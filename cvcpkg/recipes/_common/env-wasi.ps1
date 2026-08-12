# recipes/_common/env-wasi.ps1 — shared environment for wasi cross-compilation on Windows.
#
# Dot-sourced by build-wasi.ps1 scripts.  Loads env-windows.ps1 first,
# then configures the wasi-sdk toolchain and provides Invoke-CvcWasiCMakeBuild.
$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\env-windows.ps1"

if (-not $env:CVC_WASI_SDK_DIR) { throw 'CVC_WASI_SDK_DIR must point to the installed wasi-sdk' }

# WASI builds are always static.
$env:CVC_LINK = 'static'
$buildSharedLibs = 'OFF'

# Set compiler environment variables.
$env:CC = Join-Path $env:CVC_WASI_SDK_DIR 'bin\clang.exe'
$env:CXX = Join-Path $env:CVC_WASI_SDK_DIR 'bin\clang++.exe'
$env:AR = Join-Path $env:CVC_WASI_SDK_DIR 'bin\llvm-ar.exe'
$env:RANLIB = Join-Path $env:CVC_WASI_SDK_DIR 'bin\llvm-ranlib.exe'

# Locate the wasi-sdk CMake toolchain file.
$wasiToolchain = Join-Path $env:CVC_WASI_SDK_DIR 'share\cmake\wasi-sdk.cmake'
$hasToolchainFile = Test-Path $wasiToolchain
$wasiSysroot = Join-Path $env:CVC_WASI_SDK_DIR 'share\wasi-sysroot'

function Invoke-CvcWasiCMakeBuild {
    param(
        [string[]]$ExtraArgs = @(),
        [string]$SourceDir = $env:CVC_SOURCE_DIR
    )
    $findRootPathArgs = @()
    if ($env:CVC_DEPS_PREFIX) {
        $findRootPathArgs += "-DCMAKE_FIND_ROOT_PATH=$env:CVC_DEPS_PREFIX"
    }

    $toolchainArgs = @()
    if ($hasToolchainFile) {
        $toolchainArgs += "-DCMAKE_TOOLCHAIN_FILE=$wasiToolchain"
    } else {
        $toolchainArgs += @(
            '-DCMAKE_SYSTEM_NAME=WASI',
            '-DCMAKE_SYSTEM_PROCESSOR=wasm32',
            "-DCMAKE_C_COMPILER=$env:CC",
            "-DCMAKE_CXX_COMPILER=$env:CXX",
            "-DCMAKE_AR=$env:AR",
            "-DCMAKE_RANLIB=$env:RANLIB",
            "-DCMAKE_SYSROOT=$wasiSysroot",
            '-DCMAKE_C_COMPILER_TARGET=wasm32-wasip1',
            '-DCMAKE_CXX_COMPILER_TARGET=wasm32-wasip1'
        )
    }

    $allArgs = @(
        '-G', 'Ninja',
        '-S', $SourceDir,
        '-B', $env:CVC_BUILD_DIR,
        "-DCMAKE_INSTALL_PREFIX=$env:CVC_INSTALL_DIR",
        "-DCMAKE_BUILD_TYPE=$cmakeBuildType",
        '-DBUILD_SHARED_LIBS=OFF',
        '-DCMAKE_POSITION_INDEPENDENT_CODE=ON',
        '-DCMAKE_CXX_STANDARD=17',
        '-DCMAKE_POLICY_VERSION_MINIMUM=3.5'
    ) + $toolchainArgs + $findRootPathArgs + $ExtraArgs

    & cmake @allArgs
    if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

    & cmake --build $env:CVC_BUILD_DIR -j $env:CVC_JOBS
    if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

    & cmake --install $env:CVC_BUILD_DIR
    if ($LASTEXITCODE -ne 0) { throw "cmake install failed" }

    Invoke-CvcRewriteInstallPaths
}

function Invoke-CvcWasiAutotoolsBuild {
    <#
    .SYNOPSIS
      ./configure + make + make install via Git Bash, with wasi-sdk clang
      as the cross-compiler.
    .PARAMETER ConfigureArgs
      Extra arguments to pass to ./configure (beyond --prefix + --host).
    .PARAMETER Jobs
      Override make -j parallelism.  Defaults to $env:CVC_JOBS.
    .DESCRIPTION
      wasi-sdk is a clang toolchain — no emconfigure wrapper is needed.
      We run configure with CC / CXX / AR / RANLIB pointing at wasi-sdk
      binaries and inject --target=wasm32-wasip1 + --sysroot=... into
      CFLAGS / CXXFLAGS / LDFLAGS so autoconf feature-probes emit valid
      wasi objects.
    #>
    param(
        [string[]]$ConfigureArgs = @(),
        [int]$Jobs = 0
    )
    if ($Jobs -le 0) { $Jobs = [int]$env:CVC_JOBS }
    if ($Jobs -le 0) { $Jobs = 1 }

    $bash        = Get-CvcGitBash
    $msysPrefix  = ConvertTo-CvcMsysPath $env:CVC_INSTALL_DIR
    $msysSource  = ConvertTo-CvcMsysPath $env:CVC_SOURCE_DIR
    $msysDeps    = if ($env:CVC_DEPS_PREFIX) { ConvertTo-CvcMsysPath $env:CVC_DEPS_PREFIX } else { '' }

    $msysWasiCC     = ConvertTo-CvcMsysPath $env:CC
    $msysWasiCXX    = ConvertTo-CvcMsysPath $env:CXX
    $msysWasiAR     = ConvertTo-CvcMsysPath $env:AR
    $msysWasiRANLIB = ConvertTo-CvcMsysPath $env:RANLIB
    $msysWasiSysroot = ConvertTo-CvcMsysPath $wasiSysroot

    # Force plain msys — we don't need /mingw64/bin gcc, we're using
    # wasi-sdk clang, but we do need bash + m4 + make + autoconf on PATH.
    $env:MSYSTEM         = 'MSYS'
    $env:MSYS_NO_PATHCONV = '1'
    $env:CHERE_INVOKING  = '1'

    # Detect native build triplet using MSYS's own gcc (any host cc will
    # do — configure only uses --build to distinguish the target).
    $buildTriplet = & $bash -lc "gcc -dumpmachine 2>/dev/null || uname -m"
    $buildTriplet = $buildTriplet.Trim()
    if (-not $buildTriplet) { $buildTriplet = 'x86_64-pc-msys' }

    $wasiTargetFlags = "--target=wasm32-wasip1 --sysroot=$msysWasiSysroot"

    $extras   = ($ConfigureArgs -join ' ')
    $depsFlag = if ($msysDeps) { "PATH='$msysDeps/bin:'`$PATH" } else { '' }

    $configureCmd = @(
        $depsFlag,
        "cd '$msysSource' &&",
        "CC='$msysWasiCC'",
        "CXX='$msysWasiCXX'",
        "AR='$msysWasiAR'",
        "RANLIB='$msysWasiRANLIB'",
        "CFLAGS='$wasiTargetFlags'",
        "CXXFLAGS='$wasiTargetFlags'",
        "LDFLAGS='$wasiTargetFlags'",
        "./configure",
        "--prefix='$msysPrefix'",
        "--host=wasm32-wasi",
        "--build='$buildTriplet'",
        "--disable-shared",
        "--enable-static",
        $extras
    ) -join ' '

    Write-Host "cvcpkg: bash -lc `"$configureCmd`""
    & $bash -lc $configureCmd
    if ($LASTEXITCODE -ne 0) {
        $cfgLog = Join-Path $env:CVC_SOURCE_DIR 'config.log'
        if (Test-Path $cfgLog) {
            Write-Host '--- config.log (last 80 lines) ---'
            Get-Content $cfgLog -Tail 80
        }
        throw "configure failed"
    }

    & $bash -lc "cd '$msysSource' && make -j $Jobs"
    if ($LASTEXITCODE -ne 0) { throw "make failed" }

    & $bash -lc "cd '$msysSource' && make install"
    if ($LASTEXITCODE -ne 0) { throw "make install failed" }
}

Write-Host "-- env-wasi.ps1 loaded --"
Write-Host "  WASI_SDK=$env:CVC_WASI_SDK_DIR  BUILD_TYPE=$cmakeBuildType  LINK=static  JOBS=$env:CVC_JOBS"
