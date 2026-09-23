# recipes/pycvc-gl-cp312/build-wasm.ps1 — Windows-host cross-build of pycvc +
# pycvc_gl (BRIDGE=ON) to WebAssembly as STATIC archives. Windows sibling of
# build-wasm.sh: same closure-from-root approach, driven by Invoke-CvcWasmCMakeBuild
# (env-wasm.ps1) which uses native cmake/ninja + the Emscripten toolchain file.
$ErrorActionPreference = 'Stop'
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# ── (1) native host tools: SWIG (not on the box) + a native python3.12 for
# CMake's FindPython3/NumPy introspection. cmake + ninja come from the host env.
$hostEnv = Join-Path $env:CVC_BUILD_DIR 'hostenv'
$cvc = (Get-Command cvcpkg -ErrorAction SilentlyContinue)
$cvcExe = if ($cvc) { 'cvcpkg' } else { 'python -m cvcpkg' }
& cmd /c "$cvcExe install python312 swig cmake ninja --platform windows --config release --link shared --prefix `"$hostEnv`" --no-fallback-to-source"
if ($LASTEXITCODE -ne 0) { throw "host-tool provisioning failed" }
$env:PATH = "$hostEnv\bin;$env:PATH"
$pyNative = Join-Path $hostEnv 'bin\python.exe'
if (-not (Test-Path $pyNative)) { $pyNative = Join-Path $hostEnv 'python.exe' }
$swigExe = Join-Path $hostEnv 'bin\swig.exe'

# numpy C headers are architecture-independent; take them from the wasm numpy in
# the deps prefix (the wasm interpreter can't run, so don't introspect it).
$numpyInc = Join-Path $env:CVC_DEPS_PREFIX 'lib\python3.12\site-packages\numpy\_core\include'

# ── (2) emsdk + toolchain + CVC_WASM_THREADS flavor (dot-source the helper) ──
. "$scriptDir\..\_common\env-wasm.ps1"
$pthreads = if ($env:CVC_WASM_THREADS -eq '1') { 'ON' } else { 'OFF' }

# ── (3) configure + build + install the trimmed closure (static, BRIDGE=ON) ──
# Same OFF set as build-wasm.sh / cvcgl-examples, plus the pycvc bindings.
Invoke-CvcWasmCMakeBuild -ExtraArgs @(
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
    "-DPython3_NumPy_INCLUDE_DIRS=$numpyInc",
    "-DSWIG_EXECUTABLE=$swigExe"
)

# ── (4) stage the CPython-wasm host source (the archives + .py proxies are placed
# by the install() rules above; the single-.wasm host link is the next step). ──
$share = Join-Path $env:CVC_INSTALL_DIR 'share\pycvc-gl-wasm'
New-Item -ItemType Directory -Force -Path $share | Out-Null
Copy-Item (Join-Path $env:CVC_SOURCE_DIR 'bindings\pycvc\wasm\pycvc_host.cpp') $share
Write-Host "pycvc-gl(wasm) build complete (static archives + proxies installed; host source staged)"
