# recipes/_common/env-windows.ps1 — shared environment for Windows recipe builds.
#
# Dot-sourced by every build.ps1 on Windows.
$ErrorActionPreference = 'Stop'

if (-not $env:CVC_BUILD_TYPE)  { $env:CVC_BUILD_TYPE  = 'Release' }
if (-not $env:CVC_LINK)        { $env:CVC_LINK        = 'shared' }
if (-not $env:CVC_JOBS)        { $env:CVC_JOBS        = [Environment]::ProcessorCount }
if (-not $env:CVC_INSTALL_DIR) { throw 'CVC_INSTALL_DIR must be set' }
if (-not $env:CVC_SOURCE_DIR)  { throw 'CVC_SOURCE_DIR must be set' }
if (-not $env:CVC_BUILD_DIR)   { throw 'CVC_BUILD_DIR must be set' }

$cmakeBuildType = switch ($env:CVC_BUILD_TYPE.ToLower()) {
    'release' { 'Release' }
    'debug'   { 'Debug' }
    default   { 'Release' }
}

$buildSharedLibs = if ($env:CVC_LINK -eq 'static') { 'OFF' } else { 'ON' }

# MSVC runtime library: static link -> /MT (static CRT), shared -> /MD (dynamic CRT)
# The Debug infix goes BEFORE DLL: MultiThreaded[Debug][DLL]
$msvcRuntime = if ($env:CVC_LINK -eq 'static') {
    'MultiThreaded$<$<CONFIG:Debug>:Debug>'
} else {
    'MultiThreaded$<$<CONFIG:Debug>:Debug>DLL'
}

# CMAKE_PREFIX_PATH spans both dependency roots:
#   CVC_DEPS_PREFIX  — the runtime closure (install prefix; these ship)
#   CVC_BUILD_PREFIX — the build closure (build prefix; stripped on install)
# Both are searchable at build time; only the former is part of the deliverable.
# For legacy single-prefix layouts CVC_BUILD_PREFIX is unset or equal, so this
# collapses to the old behaviour.
$cvcRoots = @($env:CVC_DEPS_PREFIX, $env:CVC_BUILD_PREFIX) |
    Where-Object { $_ } | Select-Object -Unique
if ($cvcRoots) {
    $env:CMAKE_PREFIX_PATH = ($cvcRoots -join ';')
}

# On Windows we ALWAYS build with MSVC (cl.exe).  A prior builder image
# happened to have Strawberry Perl's g++ ahead of cl on PATH, and the
# Ninja generator silently picked it, producing GNU-ar '.a' archives
# with Itanium-mangled symbols that MSVC-mangled downstream consumers
# (protobuf, grpc, etc.) cannot resolve — 100+ "unresolved external"
# link errors.  Force cl.exe here so the mangling and archive format
# stay consistent across the whole dep set.
$env:CC  = 'cl'
$env:CXX = 'cl'

# ── Auto-import MSVC developer environment ──────────────────────────
#
# When the builder daemon is launched as a service (or from a plain
# PowerShell prompt) cl.exe / link.exe / rc.exe are NOT on PATH.
# GitHub Actions runs ilammy/msvc-dev-cmd before invoking us; the
# self-hosted builder does not, so do it ourselves here if needed.
function Import-CvcMsvcEnv {
    if (Get-Command cl.exe -ErrorAction SilentlyContinue) { return }

    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) {
        $vswhere = "$env:ProgramFiles\Microsoft Visual Studio\Installer\vswhere.exe"
    }
    if (-not (Test-Path $vswhere)) {
        throw "cl.exe not on PATH and vswhere.exe not found — install Visual Studio Build Tools or run from a Developer PowerShell."
    }

    $vsRoot = & $vswhere -latest -products '*' `
        -requires 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64' `
        -property installationPath 2>$null | Select-Object -First 1
    if (-not $vsRoot) {
        throw "vswhere.exe found no Visual Studio install with the C++ workload."
    }

    $vcvars = Join-Path $vsRoot 'VC\Auxiliary\Build\vcvars64.bat'
    if (-not (Test-Path $vcvars)) {
        throw "vcvars64.bat not found under $vsRoot"
    }

    # Run vcvars64.bat via a small temp .cmd (properly quoted inside the
    # batch) and dump the resulting environment.  Invoking a single clean
    # helper path avoids the fragile inline `cmd /c "..."` quote passing,
    # whose handling of embedded quotes differs across PowerShell editions
    # (Windows PowerShell 5.1 vs pwsh 7) and can silently mangle the quoted
    # vcvars path (which lives under "C:\Program Files (x86)\...").
    $helper = Join-Path ([System.IO.Path]::GetTempPath()) ("cvc-vcvars-{0}.cmd" -f ([guid]::NewGuid().ToString('N')))
    $body = "@echo off`r`ncall `"$vcvars`" >nul 2>&1`r`nset`r`n"
    Set-Content -LiteralPath $helper -Value $body -Encoding Ascii
    try {
        $dump = & cmd.exe /c $helper
    } finally {
        Remove-Item -LiteralPath $helper -ErrorAction SilentlyContinue
    }
    foreach ($line in $dump) {
        if ($line -match '^([^=]+)=(.*)$') {
            $name = $matches[1]
            $val  = $matches[2]
            # Skip a few obviously-per-process env vars we don't want
            # to clobber.
            if ($name -in @('_', 'PROMPT')) { continue }
            [Environment]::SetEnvironmentVariable($name, $val, 'Process')
        }
    }
    if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
        throw "Import-CvcMsvcEnv ran but cl.exe still not on PATH — vcvars64.bat likely failed silently."
    }
}
Import-CvcMsvcEnv

function Invoke-CvcCMakeBuild {
    param([string[]]$ExtraArgs = @())

    # Strip MinGW/MSYS2 dirs from PATH for the duration of the MSVC
    # cmake build.  CMake's find_path()/find_library() default search
    # includes "for each dir in PATH, look in ../include and ../lib" —
    # which means if C:\msys64\mingw64\bin is on Machine PATH the
    # MinGW-w64 headers (stdio.h with __asm__, __builtin_va_list, etc.)
    # get picked up by cl.exe.  Those headers are gcc-only and produce
    # thousands of parse errors under MSVC.  MSVC builds have zero use
    # for anything under C:\msys64\, so drop them entirely here.
    $origPath = $env:PATH
    $filteredPath = ($env:PATH -split ';' |
        Where-Object { $_ -notmatch '(?i)\\msys64\\' -and $_ -notmatch '(?i)\\msys32\\' }) -join ';'
    $env:PATH = $filteredPath

    try {
        $allArgs = @(
            '-G', 'Ninja',
            '-S', $env:CVC_SOURCE_DIR,
            '-B', $env:CVC_BUILD_DIR,
            "-DCMAKE_INSTALL_PREFIX=$env:CVC_INSTALL_DIR",
            "-DCMAKE_BUILD_TYPE=$cmakeBuildType",
            "-DBUILD_SHARED_LIBS=$buildSharedLibs",
            "-DCMAKE_MSVC_RUNTIME_LIBRARY=$msvcRuntime",
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
            '-DCMAKE_C_COMPILER=cl',
            '-DCMAKE_CXX_COMPILER=cl'
        ) + $ExtraArgs

        & cmake @allArgs
        if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

        & cmake --build $env:CVC_BUILD_DIR -j $env:CVC_JOBS
        if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

        & cmake --install $env:CVC_BUILD_DIR
        if ($LASTEXITCODE -ne 0) { throw "cmake install failed" }
    } finally {
        $env:PATH = $origPath
    }

    Invoke-CvcRewriteInstallPaths
}

# ── Relocatability helper ───────────────────────────────────────────
#
# Rewrite absolute $env:CVC_INSTALL_DIR paths inside installed .pc and
# .cmake files so downstream consumers keep working when the package is
# unpacked at a different prefix.  .pc files anchor at ${pcfiledir}
# and .cmake files at ${CMAKE_CURRENT_LIST_DIR}; the ../ suffix is
# computed per-file from its depth under $env:CVC_INSTALL_DIR.  CMake
# on Windows writes paths with forward slashes into generated files,
# but MSVC/PowerShell APIs return backslash form, so we search for
# both variants.  Idempotent.

function Invoke-CvcRewriteInstallPaths {
    $root = $env:CVC_INSTALL_DIR
    if (-not $root) { return }
    $root = $root.TrimEnd('\','/')
    if (-not (Test-Path -LiteralPath $root)) { return }

    $rootBack = $root
    $rootFwd  = $root -replace '\\','/'
    $forms = @($rootBack, $rootFwd) | Sort-Object -Unique

    $files = Get-ChildItem -Path $root -Recurse -File -ErrorAction SilentlyContinue |
        Where-Object { $_.Extension -eq '.pc' -or $_.Extension -eq '.cmake' }
    $count = 0
    foreach ($f in $files) {
        $text = Get-Content -Raw -LiteralPath $f.FullName -ErrorAction SilentlyContinue
        if (-not $text) { continue }
        $needsRewrite = $false
        foreach ($form in $forms) {
            if ($text.Contains($form)) { $needsRewrite = $true; break }
        }
        if (-not $needsRewrite) { continue }

        $dirNorm = ($f.DirectoryName -replace '\\','/')
        $rootFwdTrim = $rootFwd
        $remainder = $dirNorm.Substring($rootFwdTrim.Length).TrimStart('/')
        $depth = if ($remainder) { ($remainder -split '/').Length } else { 0 }
        $rel = if ($depth -gt 0) { (('../' * $depth)).TrimEnd('/') } else { '.' }
        $anchor = if ($f.Extension -eq '.pc') { '${pcfiledir}' } else { '${CMAKE_CURRENT_LIST_DIR}' }
        $prefix = "$anchor/$rel"

        foreach ($form in $forms) {
            $text = $text -replace [regex]::Escape($form), $prefix.Replace('$','$$')
        }
        Set-Content -LiteralPath $f.FullName -Value $text -NoNewline
        $count++
    }
    if ($count -gt 0) {
        Write-Host "── Invoke-CvcRewriteInstallPaths: normalized $count file(s) under $root ──"
    }
}

# ── MSYS2 / MinGW autotools helper ──────────────────────────────────
#
# A handful of dependencies (gmp, mpfr, gsl) have no working native
# MSVC build (upstream autotools is Unix-only, no CMake, GNU-only
# assembly).  For those we build via MinGW-w64 gcc + GNU make driven
# through Git Bash — this produces cdecl-C DLLs that MSVC downstream
# consumers can link against through the mingw-generated import
# library.  When CVC_LINK=static we use the mingw static archive
# directly; it links cleanly from MSVC for pure-C libraries with no
# libgcc/libstdc++ dependencies (which is the case for gmp/mpfr/gsl).
#
# The builder is expected to have Git Bash on PATH (as env-wasm.ps1
# already assumes) plus MSYS2 mingw-w64 gcc + make + m4 + libtool.
# See vm-provisioning docs for the required MSYS2 packages.

function Get-CvcGitBash {
    # Return a bash.exe that has (or can find) mingw-w64 gcc + make +
    # autotools on its PATH.  Priority order:
    #   1. CVC_MSYS2_DIR env var (manual override)
    #   2. CVC_DEPS_PREFIX\msys2 (installed by the msys2 cvcpkg recipe)
    #   3. Well-known MSYS2 system paths (C:\msys64, C:\tools\msys64)
    #   4. Git Bash fallback (no MinGW — usable only for non-compile scripts)
    $candidates = [System.Collections.Generic.List[string]]@()
    if ($env:CVC_MSYS2_DIR) {
        $candidates.Add((Join-Path $env:CVC_MSYS2_DIR 'usr\bin\bash.exe'))
    }
    if ($env:CVC_DEPS_PREFIX) {
        $candidates.Add((Join-Path $env:CVC_DEPS_PREFIX 'msys2\usr\bin\bash.exe'))
    }
    $candidates.AddRange([string[]]@(
        'C:\msys64\usr\bin\bash.exe',
        'C:\tools\msys64\usr\bin\bash.exe',
        "$env:ProgramFiles\Git\usr\bin\bash.exe",
        "$env:ProgramFiles\Git\bin\bash.exe",
        "${env:ProgramFiles(x86)}\Git\usr\bin\bash.exe"
    ))
    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) { return $candidate }
    }
    $found = Get-Command bash -ErrorAction SilentlyContinue |
             Where-Object { $_.Source -notmatch 'System32' } |
             Select-Object -First 1
    if ($found) { return $found.Source }
    throw 'bash.exe not found (looked under CVC_MSYS2_DIR, CVC_DEPS_PREFIX\msys2\usr\bin\, C:\msys64\usr\bin\, Git\usr\bin\, and PATH)'
}

function ConvertTo-CvcMsysPath {
    # Convert C:\foo\bar → /c/foo/bar for autotools scripts.
    param([string]$Path)
    if ($Path -match '^([A-Za-z]):(.*)$') {
        $drive = $Matches[1].ToLower()
        $rest  = $Matches[2] -replace '\\','/'
        return "/$drive$rest"
    }
    return ($Path -replace '\\','/')
}

function Invoke-CvcMsysAutotoolsBuild {
    <#
    .SYNOPSIS
      Configure + make + make install via Git Bash + MinGW gcc.
    .PARAMETER ConfigureArgs
      Extra arguments to pass to ./configure (beyond --prefix + --host).
    .PARAMETER HostTriple
      Cross-compilation host triple.  Defaults to x86_64-w64-mingw32
      (native MinGW-w64 on Windows).
    .PARAMETER Jobs
      Override make -j parallelism.  Defaults to $env:CVC_JOBS.
      Set to 1 for fork-heavy libtool builds that deadlock MSYS2 under
      the SYSTEM account (gmp, mpfr, ...).
    #>
    param(
        [string[]]$ConfigureArgs = @(),
        [string]$HostTriple = 'x86_64-w64-mingw32',
        [int]$Jobs = 0
    )
    if ($Jobs -le 0) { $Jobs = [int]$env:CVC_JOBS }
    if ($Jobs -le 0) { $Jobs = 1 }

    $bash        = Get-CvcGitBash
    $msysPrefix  = ConvertTo-CvcMsysPath $env:CVC_INSTALL_DIR
    $msysSource  = ConvertTo-CvcMsysPath $env:CVC_SOURCE_DIR
    $msysDeps    = if ($env:CVC_DEPS_PREFIX) { ConvertTo-CvcMsysPath $env:CVC_DEPS_PREFIX } else { '' }
    $depsFlag    = if ($msysDeps) { "PATH='$msysDeps/bin:'`$PATH" } else { '' }

    # Force the MinGW-w64 64-bit subsystem so that /mingw64/bin
    # (gcc, make, libtool, autoconf, m4, ...) is on the shell PATH.
    # Without MSYSTEM set, MSYS2's bash defaults to the plain msys
    # environment where gcc is not present.  MSYS_NO_PATHCONV=1
    # prevents MSYS from mangling Windows-style arguments passed to
    # non-MSYS binaries invoked from configure/libtool.
    $env:MSYSTEM         = 'MINGW64'
    $env:MSYS_NO_PATHCONV = '1'
    $env:CHERE_INVOKING  = '1'

    # Clear MSVC compiler env inherited from the outer PowerShell
    # session — env-windows.ps1 sets CC=cl / CXX=cl for Invoke-CvcCMakeBuild,
    # but for the autotools path we need bash's PATH to select the
    # MinGW-w64 gcc from /mingw64/bin.  Autoconf's ./configure would
    # otherwise take CC=cl at face value, produce nonsense build-system
    # detection, and choke on the first Unix-only linker flag (e.g.
    # -lm  →  LINK: cannot open input file 'm.lib').
    Remove-Item Env:CC  -ErrorAction SilentlyContinue
    Remove-Item Env:CXX -ErrorAction SilentlyContinue
    Remove-Item Env:LD  -ErrorAction SilentlyContinue
    Remove-Item Env:AR  -ErrorAction SilentlyContinue
    Remove-Item Env:NM  -ErrorAction SilentlyContinue
    Remove-Item Env:RANLIB -ErrorAction SilentlyContinue
    Remove-Item Env:CFLAGS  -ErrorAction SilentlyContinue
    Remove-Item Env:CXXFLAGS -ErrorAction SilentlyContinue
    Remove-Item Env:LDFLAGS -ErrorAction SilentlyContinue

    $sharedFlags = if ($env:CVC_LINK -eq 'static') {
        '--enable-static --disable-shared'
    } else {
        '--enable-shared --enable-static'
    }

    # Pre-flight: verify that required MSYS2/MinGW tools are available.
    # These should be provided by their own cvcpkg recipes (m4, autoconf,
    # automake, libtool) declared as host_tools in the calling recipe.
    # The $depsFlag prepends CVC_DEPS_PREFIX/bin so shim wrappers installed
    # by those recipes are found before any system copies.
    $probe = & $bash -lc "$depsFlag command -v m4 >/dev/null && command -v libtool >/dev/null && command -v autoconf >/dev/null && command -v automake >/dev/null && command -v make >/dev/null && echo OK"
    if ($probe -notmatch 'OK') {
        throw 'MSYS2 autotools tools (m4, autoconf, automake, libtool, make) not found. Declare them as host_tools in recipe.yaml.'
    }

    # Build one big command line for bash; the caller-provided extras
    # win over defaults because they come last.
    $extras   = ($ConfigureArgs -join ' ')
    $cmd = "$depsFlag cd '$msysSource' && ./configure --prefix='$msysPrefix' --host='$HostTriple' $sharedFlags $extras"
    Write-Host "cvcpkg: bash -lc `"$cmd`""
    & $bash -lc $cmd
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

    # Post-process installed libraries so MSVC downstream can link:
    #   libfoo.dll.a  → foo.lib      (import library for shared)
    #   libfoo.a      → foo.lib      (static archive; only used when
    #                                 CVC_LINK=static — safe for
    #                                 pure-C libs without libgcc deps)
    # Copies rather than renames so both mingw-style names and
    # MSVC-style names are present, and downstream find_library() can
    # locate whichever it looks for.
    $installLib = Join-Path $env:CVC_INSTALL_DIR 'lib'
    if (Test-Path $installLib) {
        foreach ($f in Get-ChildItem -Path $installLib -File -Filter 'lib*.dll.a' -ErrorAction SilentlyContinue) {
            $stem = $f.Name.Substring(3, $f.Name.Length - 3 - 6)   # strip "lib" prefix + ".dll.a" suffix
            $dest = Join-Path $installLib ($stem + '.lib')
            if (-not (Test-Path $dest)) { Copy-Item -Force $f.FullName $dest }
        }
        if ($env:CVC_LINK -eq 'static') {
            foreach ($f in Get-ChildItem -Path $installLib -File -Filter 'lib*.a' -ErrorAction SilentlyContinue) {
                if ($f.Name -like '*.dll.a') { continue }
                $stem = $f.Name.Substring(3, $f.Name.Length - 3 - 2)  # strip "lib" prefix + ".a" suffix
                $dest = Join-Path $installLib ($stem + '.lib')
                if (-not (Test-Path $dest)) { Copy-Item -Force $f.FullName $dest }
            }
        }
    }
}

# ── Meson / MSVC helper ─────────────────────────────────────────────
#
# Build a Meson project with the native MSVC toolchain (cl.exe).  This
# produces proper Windows .dll/.lib files (MSVC ABI, not MinGW ABI) so
# consumers built with cl.exe can link directly.
#
# Meson and Ninja must be on PATH.  Declare them as host_tools in the
# calling recipe.yaml (depends.host_tools: [meson, ninja]).  The PATH
# check below will throw a clear error if they are missing.
# MinGW/MSYS2 directories are stripped from PATH for the duration of
# the build to prevent gcc headers from contaminating the MSVC build.
#
# Usage:
#   Invoke-CvcMesonBuild [-MesonArgs <string[]>]
function Invoke-CvcMesonBuild {
    param([string[]]$MesonArgs = @())

    # Verify meson and ninja are available; they must come from their
    # own cvcpkg recipes declared as host_tools — do NOT install here.
    if (-not (Get-Command meson -ErrorAction SilentlyContinue)) {
        throw 'meson not found on PATH. Declare "meson" as a host_tool in recipe.yaml.'
    }
    if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
        throw 'ninja not found on PATH. Declare "ninja" as a host_tool in recipe.yaml.'
    }

    # Strip MinGW/MSYS2 from PATH so meson selects cl.exe, not gcc.
    $origPath = $env:PATH
    $filteredPath = ($env:PATH -split ';' |
        Where-Object { $_ -notmatch '(?i)\\msys64\\' -and $_ -notmatch '(?i)\\msys32\\' }) -join ';'
    $env:PATH = $filteredPath

    # Meson defaults: shared/static, release/debug, MSVC runtime.
    $defaultLibrary = if ($env:CVC_LINK -eq 'static') { 'static' } else { 'shared' }
    $buildtype      = $cmakeBuildType.ToLower()

    $pkgConfigPath = if ($env:CVC_DEPS_PREFIX) {
        Join-Path $env:CVC_DEPS_PREFIX 'lib\pkgconfig'
    } else { '' }

    try {
        $setupArgs = @(
            'setup',
            "--prefix=$env:CVC_INSTALL_DIR",
            "--buildtype=$buildtype",
            '--libdir=lib',
            "--default-library=$defaultLibrary"
        )
        if ($pkgConfigPath -and (Test-Path $pkgConfigPath)) {
            $setupArgs += "--pkg-config-path=$pkgConfigPath"
        }
        if ($env:CVC_DEPS_PREFIX) {
            $setupArgs += "-Dcmake_prefix_path=$env:CVC_DEPS_PREFIX"
        }
        $setupArgs += $MesonArgs
        $setupArgs += $env:CVC_BUILD_DIR
        $setupArgs += $env:CVC_SOURCE_DIR

        & meson @setupArgs
        if ($LASTEXITCODE -ne 0) { throw 'meson setup failed' }

        & meson compile -C $env:CVC_BUILD_DIR -j $env:CVC_JOBS
        if ($LASTEXITCODE -ne 0) { throw 'meson compile failed' }

        & meson install -C $env:CVC_BUILD_DIR
        if ($LASTEXITCODE -ne 0) { throw 'meson install failed' }
    } finally {
        $env:PATH = $origPath
    }

    Invoke-CvcRewriteInstallPaths
}

Write-Host "-- env-windows.ps1 loaded --"
Write-Host "  BUILD_TYPE=$cmakeBuildType  LINK=$env:CVC_LINK  JOBS=$env:CVC_JOBS"
