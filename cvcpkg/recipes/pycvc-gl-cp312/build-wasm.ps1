# recipes/pycvc-gl-cp312/build-wasm.ps1 — Windows-host cross-build of pycvc +
# pycvc_gl (BRIDGE=ON) to WebAssembly as STATIC archives. Windows sibling of
# build-wasm.sh: same closure-from-root approach, driven by Invoke-CvcWasmCMakeBuild
# (env-wasm.ps1) which uses native cmake/ninja + the Emscripten toolchain file.
$ErrorActionPreference = 'Stop'
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# ── (1) native host tools ──
# SWIG (not on the box) + cmake/ninja from the catalog. There is NO windows
# python312 in the catalog (it is a documented gap), so use a NATIVE host
# python3.12 for CMake's FindPython3 version probe — the wasm libpython in the
# deps prefix is the actual link target, not this interpreter. Resolve it from
# CVC_HOST_PYTHON, then the `py -3.12` launcher, then `python` on PATH (the fleet
# windows runner's setup-python provides 3.12).
$hostEnv = Join-Path $env:CVC_BUILD_DIR 'hostenv'
$cvc = (Get-Command cvcpkg -ErrorAction SilentlyContinue)
$cvcExe = if ($cvc) { 'cvcpkg' } else { 'python -m cvcpkg' }
& cmd /c "$cvcExe install swig cmake ninja --platform windows --config release --link shared --prefix `"$hostEnv`" --no-fallback-to-source"
if ($LASTEXITCODE -ne 0) { throw "host-tool provisioning failed" }
$env:PATH = "$hostEnv\bin;$env:PATH"
$swigExe = Join-Path $hostEnv 'bin\swig.exe'
# The cvcpkg-packaged SWIG reports a stale -swiglib after relocation, so FindSWIG
# cannot locate SWIG_DIR (the .i library). Point it at the real library dir
# (share/swig/<ver>) via both the env var swig itself reads and the CMake var.
$swigLibDir = Get-ChildItem (Join-Path $hostEnv 'share\swig') -Directory -ErrorAction SilentlyContinue | Select-Object -First 1
if ($swigLibDir) {
    $env:SWIG_LIB = $swigLibDir.FullName
    $swigDir = $swigLibDir.FullName
} else {
    $swigDir = ''
}

$pyNative = $null
if ($env:CVC_HOST_PYTHON -and (Test-Path $env:CVC_HOST_PYTHON)) {
    $pyNative = $env:CVC_HOST_PYTHON
} else {
    $pyNative = (& py -3.12 -c "import sys;print(sys.executable)" 2>$null)
    if (-not $pyNative) { $pyNative = (Get-Command python -ErrorAction SilentlyContinue).Source }
}
if (-not $pyNative -or -not (Test-Path $pyNative)) {
    throw "no native python3.12 for FindPython3 (set CVC_HOST_PYTHON)"
}

# numpy C headers are architecture-independent; take them from the wasm numpy in
# the deps prefix (the wasm interpreter can't run, so don't introspect it).
$numpyInc = Join-Path $env:CVC_DEPS_PREFIX 'lib\python3.12\site-packages\numpy\_core\include'
# FindPython3 in a cross build won't derive the TARGET (wasm) headers/lib from the
# native executable — pin them explicitly to the wasm libpython in the deps prefix
# (Development.Module + the include dir); the native exe supplies only the version.
$pyIncDir = Join-Path $env:CVC_DEPS_PREFIX 'include\python3.12'
$pyLib    = Join-Path $env:CVC_DEPS_PREFIX 'lib\libpython3.12.a'

# ── (2) emsdk + toolchain + CVC_WASM_THREADS flavor (dot-source the helper) ──
. "$scriptDir\..\_common\env-wasm.ps1"
$pthreads = if ($env:CVC_WASM_THREADS -eq '1') { 'ON' } else { 'OFF' }

# Point config-mode find_package(Boost) (CMP0167=NEW) straight at the cvcpkg boost
# config dir — avoids the emscripten cross FIND_ROOT_PATH re-rooting trap.
$boostDir = (Get-ChildItem (Join-Path $env:CVC_DEPS_PREFIX 'lib\cmake') -Directory -Filter 'Boost-*' -ErrorAction SilentlyContinue | Select-Object -First 1).FullName

# ── (3) configure + build + install the trimmed closure (static, BRIDGE=ON) ──
# Same OFF set as build-wasm.sh / cvcgl-examples, plus the pycvc bindings.
Invoke-CvcWasmCMakeBuild -ExtraArgs @(
    # cmake >=3.30 deprecates the legacy FindBoost module (CMP0167); its module
    # mode fails to locate the cvcpkg boost layout under the cross FIND_ROOT_PATH.
    # NEW = use boost's BoostConfig.cmake (config mode), shipped by the wasm boost
    # package. Fixes "Could NOT find Boost" at configure.
    '-DCMAKE_POLICY_DEFAULT_CMP0167=NEW',
    "-DBoost_DIR=$boostDir",
    '-DCVC_ENABLE_CUDA=OFF',
    '-DCVC_BUILD_TESTS=OFF',
    '-DCVC_BUILD_CLI=OFF',
    '-DCVC_ENABLE_OPENMP=OFF',
    '-DDISABLE_CGAL=ON',
    '-DCVC_USING_HDF5=OFF',
    '-DCVC_USING_IMOD_MRC=OFF',
    '-DCVC_ENABLE_IMAGEMAGICK=OFF',
    '-DCVC_ENABLE_FFTW=OFF',
    '-DCVC_FFT_PROVIDER=none',
    '-DCVC_ENABLE_ASSIMP=ON',
    '-DCVC_ENABLE_MESHER=OFF',
    '-DCVC_ENABLE_SDF=ON',
    '-DCVC_STATE_EXEC=OFF',
    '-DCVC_BUILD_CVCGL=ON',
    '-DCVC_BUILD_EXAMPLES=OFF',
    "-DCVC_WASM_PTHREADS=$pthreads",
    '-DCVC_BUILD_PYCVC=ON',
    '-DCVC_BUILD_PYCVC_CORE=ON',
    '-DCVC_BUILD_PYCVC_GL=ON',
    '-DCVC_PYCVCGL_VTK_BRIDGE=ON',
    "-DPython3_EXECUTABLE=$pyNative",
    "-DPython3_INCLUDE_DIR=$pyIncDir",
    "-DPython3_LIBRARY=$pyLib",
    "-DPython3_NumPy_INCLUDE_DIR=$numpyInc",
    "-DPython3_NumPy_INCLUDE_DIRS=$numpyInc",
    "-DSWIG_EXECUTABLE=$swigExe",
    "-DSWIG_DIR=$swigDir"
)

# ── (4) link the CPython-wasm host (embeds pycvc + pycvc_gl + numpy) ──────────
# Invoke-CvcWasmCMakeBuild already installed the archives + .py proxies. Drive the
# proven link-host.sh under Git Bash (env-wasm.ps1 provides $script:gitBash +
# ConvertTo-MsysPath). Non-fatal — the archives are the primary deliverable.
$share = Join-Path $env:CVC_INSTALL_DIR 'share\pycvc-gl-wasm'
New-Item -ItemType Directory -Force -Path $share | Out-Null
Copy-Item (Join-Path $env:CVC_SOURCE_DIR 'bindings\pycvc\wasm\pycvc_host.cpp') $share
$nodeExe = (Get-ChildItem (Join-Path $env:CVC_EMSDK_DIR 'node') -Recurse -Filter node.exe -ErrorAction SilentlyContinue | Select-Object -First 1).FullName
$bashEnv = @(
    "DEPS='$(ConvertTo-MsysPath $env:CVC_DEPS_PREFIX)'",
    "INST='$(ConvertTo-MsysPath $env:CVC_INSTALL_DIR)'",
    "SRC='$(ConvertTo-MsysPath $env:CVC_SOURCE_DIR)'",
    "EMSDK='$(ConvertTo-MsysPath $env:CVC_EMSDK_DIR)'",
    "OUT='$(ConvertTo-MsysPath $share)'"
) -join ' '
if ($nodeExe) { $bashEnv += " NODE='$(ConvertTo-MsysPath $nodeExe)'" }
$linkScript = ConvertTo-MsysPath (Join-Path $env:CVC_SOURCE_DIR 'bindings\pycvc\wasm\link-host.sh')
& $script:gitBash -lc "$bashEnv bash '$linkScript'"
if ($LASTEXITCODE -eq 0) { Write-Host "pycvc-gl(wasm): host binary pycvc_host.wasm linked" }
else { Write-Host "pycvc-gl(wasm): host link failed — archives installed; link-host.sh is standalone-runnable" }
Write-Host "pycvc-gl(wasm) build complete"
