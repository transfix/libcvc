# recipes/_common/env-wasm.ps1 — shared environment for wasm cross-compilation on Windows.
#
# Dot-sourced by build-wasm.ps1 scripts.  Loads env-windows.ps1 first,
# then activates Emscripten and provides Invoke-CvcWasmCMakeBuild.
$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
. "$scriptDir\env-windows.ps1"

if (-not $env:CVC_EMSDK_DIR) { throw 'CVC_EMSDK_DIR must point to the activated emsdk bundle' }

# Remove MSYS2/Git Bash env vars that confuse emsdk.py into emitting
# UNIX-format paths (`:` separator, `/c/` prefixes) inside PowerShell.
# This happens when the cvcpkg workflow step uses `shell: bash` on Windows.
Remove-Item Env:\MSYSTEM -ErrorAction SilentlyContinue
Remove-Item Env:\MSYSTEM_PREFIX -ErrorAction SilentlyContinue
Remove-Item Env:\MSYSTEM_CHOST -ErrorAction SilentlyContinue

# Activate Emscripten.
$emsdkEnv = Join-Path $env:CVC_EMSDK_DIR 'emsdk_env.ps1'
if (-not (Test-Path $emsdkEnv)) {
    # Fallback: emsdk_env.bat via cmd interop
    $emsdkBat = Join-Path $env:CVC_EMSDK_DIR 'emsdk_env.bat'
    if (-not (Test-Path $emsdkBat)) { throw "Cannot find emsdk_env.ps1 or emsdk_env.bat in $env:CVC_EMSDK_DIR" }
    & cmd /c "`"$emsdkBat`" && set" | ForEach-Object {
        if ($_ -match '^([^=]+)=(.*)$') {
            [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process')
        }
    }
} else {
    . $emsdkEnv
}

# Wasm builds are always static.
$env:CVC_LINK = 'static'
$buildSharedLibs = 'OFF'

# Locate the Emscripten toolchain file.
$emscriptenToolchain = Join-Path $env:CVC_EMSDK_DIR 'upstream\emscripten\cmake\Modules\Platform\Emscripten.cmake'
if (-not (Test-Path $emscriptenToolchain)) { throw "Emscripten toolchain file not found: $emscriptenToolchain" }

function Invoke-CvcWasmCMakeBuild {
    param(
        [string[]]$ExtraArgs = @(),
        [string]$SourceDir = $env:CVC_SOURCE_DIR
    )
    $findRootPathArgs = @()
    if ($env:CVC_DEPS_PREFIX) {
        $findRootPathArgs += "-DCMAKE_FIND_ROOT_PATH=$env:CVC_DEPS_PREFIX"
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
        '-DCMAKE_POLICY_VERSION_MINIMUM=3.5',
        "-DCMAKE_TOOLCHAIN_FILE=$emscriptenToolchain"
    ) + $findRootPathArgs + $ExtraArgs

    & cmake @allArgs
    if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

    & cmake --build $env:CVC_BUILD_DIR -j $env:CVC_JOBS
    if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

    & cmake --install $env:CVC_BUILD_DIR
    if ($LASTEXITCODE -ne 0) { throw "cmake install failed" }

    Invoke-CvcRewriteInstallPaths
}

function ConvertTo-MsysPath {
    # Convert a Windows path (C:\foo\bar) to MSYS/Git-Bash style (/c/foo/bar)
    # so that autotools configure scripts invoked via `bash` work correctly.
    param([string]$Path)
    if ($Path -match '^([A-Za-z]):(.*)$') {
        $drive = $Matches[1].ToLower()
        $rest = $Matches[2] -replace '\\','/'
        return "/$drive$rest"
    }
    return ($Path -replace '\\','/')
}

# Locate Git Bash explicitly to avoid Python's CreateProcess finding WSL's
# bash.exe in C:\Windows\System32 before Git Bash on PATH.
$script:gitBash = $null
foreach ($candidate in @(
    "$env:ProgramFiles\Git\usr\bin\bash.exe",
    "$env:ProgramFiles\Git\bin\bash.exe",
    "${env:ProgramFiles(x86)}\Git\usr\bin\bash.exe"
)) {
    if (Test-Path $candidate) { $script:gitBash = $candidate; break }
}
if (-not $script:gitBash) {
    $found = Get-Command bash -ErrorAction SilentlyContinue |
             Where-Object { $_.Source -notmatch 'System32' } |
             Select-Object -First 1
    if ($found) { $script:gitBash = $found.Source }
}
if (-not $script:gitBash) { throw "Cannot find Git Bash (non-WSL). Install Git for Windows." }

# Add Git's usr/bin to PATH so autotools Makefiles can find Unix utilities
# (rm, cp, mv, install, etc.) when invoked via emmake/mingw32-make.
# Use the 8.3 short path so that mingw32-make's SHELL auto-detection
# (which scans PATH for sh.exe) produces a space-free path.  Without
# this, SHELL becomes "C:/Program Files/Git/usr/bin/sh.exe" and any
# Makefile recipe that uses $(SHELL) — such as libtool — breaks.
$gitUsrBin = Split-Path $script:gitBash
$fso = New-Object -ComObject Scripting.FileSystemObject
$gitUsrBinShort = $fso.GetFolder($gitUsrBin).ShortPath
if (-not ($env:PATH -split ';' | Where-Object { $_ -eq $gitUsrBinShort })) {
    $env:PATH = "$gitUsrBinShort;$env:PATH"
}

# Ensure MAKE points to mingw32-make so that autotools' recursive $(MAKE)
# calls resolve correctly.  Without this, configure detects "make does not
# set $(MAKE)" and hardcodes MAKE=make in generated Makefiles, but there is
# no make.exe on Windows — only mingw32-make.exe (from Strawberry Perl).
$mgm = Get-Command mingw32-make -ErrorAction SilentlyContinue
if ($mgm) { $env:MAKE = 'mingw32-make' }

# Provide MSYS-style paths for Emscripten compiler tools.  emconfigure sets
# CC/CXX/AR/RANLIB to full Windows paths (C:\...\emcc.bat) but GMP and other
# autotools projects' custom configure macros pass those values through shell
# operations that strip the backslashes.  Recipes override CC/CXX etc. inside
# their bash commands using these MSYS paths (/c/.../emcc.bat) instead.
$emscriptenDir = Join-Path $env:CVC_EMSDK_DIR 'upstream\emscripten'
$script:emToolExports = (
    "export CC='$(ConvertTo-MsysPath (Join-Path $emscriptenDir 'emcc.bat'))'" +
    " CXX='$(ConvertTo-MsysPath (Join-Path $emscriptenDir 'em++.bat'))'" +
    " AR='$(ConvertTo-MsysPath (Join-Path $emscriptenDir 'emar.bat'))'" +
    " RANLIB='$(ConvertTo-MsysPath (Join-Path $emscriptenDir 'emranlib.bat'))'" +
    " CONFIG_SHELL=/usr/bin/sh SHELL=/usr/bin/sh"
)

Write-Host "-- env-wasm.ps1 loaded --"
Write-Host "  EMSDK=$env:CVC_EMSDK_DIR  BUILD_TYPE=$cmakeBuildType  LINK=static  JOBS=$env:CVC_JOBS"
Write-Host "  BASH=$script:gitBash"
