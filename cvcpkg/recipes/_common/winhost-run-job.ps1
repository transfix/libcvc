# recipes/_common/winhost-run-job.ps1 — Windows-host runner for WSL-delegated builds.
#
# Invoked ON THE WINDOWS HOST (through WSL interop) by the cvcpkg winhost
# module (src/cvcpkg/winhost.py) when a builder inside a WSL distro takes a
# windows/x86_64 cross job.  It receives a job file describing the recipe
# build (paths as the host sees them + the CVC_* environment), applies the
# environment, and runs the recipe's normal Windows build script
# (build.ps1 + env-windows.ps1 do the MSVC setup themselves).
#
# Runs under Windows PowerShell 5.1 (always present, launchable through
# interop); the recipe script itself is executed in a child shell —
# PowerShell 7 (pwsh) when one is available, 5.1 otherwise.  The Microsoft
# Store pwsh alias under WindowsApps is skipped: it is not usable from
# non-interactive sessions.
#
# Usage: powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass `
#            -File winhost-run-job.ps1 -JobFile <path\to\winhost-job.json>
param(
    [Parameter(Mandatory = $true)][string]$JobFile
)
$ErrorActionPreference = 'Stop'

function Find-CvcBuildShell {
    param([string]$DepsPrefix)
    # 1. pwsh provided by the cvcpkg 'powershell' recipe in the deps prefix
    #    (same lookup order as cvcpkg's _find_pwsh on native Windows builders).
    if ($DepsPrefix) {
        foreach ($rel in @('lib\powershell\pwsh.exe', 'bin\pwsh.exe')) {
            $p = Join-Path $DepsPrefix $rel
            if (Test-Path -LiteralPath $p) { return $p }
        }
    }
    # 2. A system PowerShell 7 install.
    foreach ($p in @(
            "$env:ProgramFiles\PowerShell\7\pwsh.exe",
            "${env:ProgramFiles(x86)}\PowerShell\7\pwsh.exe")) {
        if (Test-Path -LiteralPath $p) { return $p }
    }
    # 3. pwsh on PATH — but never the Store alias (WindowsApps), which
    #    refuses to launch from non-interactive sessions.
    $cmd = Get-Command pwsh.exe -ErrorAction SilentlyContinue |
        Where-Object { $_.Source -notmatch '\\WindowsApps\\' } |
        Select-Object -First 1
    if ($cmd) { return $cmd.Source }
    # 4. Fall back to this Windows PowerShell 5.1.
    $fallback = "$env:SystemRoot\System32\WindowsPowerShell\v1.0\powershell.exe"
    Write-Host "winhost-run-job: WARNING - PowerShell 7 (pwsh) not found; running the build script under Windows PowerShell 5.1. Install pwsh 7 on the host (choco install powershell-core) if the build fails on syntax."
    return $fallback
}

if (-not (Test-Path -LiteralPath $JobFile)) {
    Write-Host "winhost-run-job: job file not found: $JobFile"
    exit 3
}
$job = Get-Content -Raw -LiteralPath $JobFile | ConvertFrom-Json

Write-Host "== winhost-run-job: $($job.recipe) ($($job.mode) mode) on $env:COMPUTERNAME =="

# Apply the job environment (CVC_* dirs in host-visible form, matrix env).
foreach ($prop in $job.env.PSObject.Properties) {
    [Environment]::SetEnvironmentVariable($prop.Name, [string]$prop.Value, 'Process')
}

foreach ($required in @('CVC_SOURCE_DIR', 'CVC_BUILD_DIR', 'CVC_INSTALL_DIR', 'CVC_RECIPE_DIR')) {
    if (-not [Environment]::GetEnvironmentVariable($required)) {
        Write-Host "winhost-run-job: job env is missing $required"
        exit 3
    }
}

New-Item -ItemType Directory -Force -Path $env:CVC_BUILD_DIR | Out-Null
New-Item -ItemType Directory -Force -Path $env:CVC_INSTALL_DIR | Out-Null

# Host tools installed into the deps prefix (cmake, ninja, ...) win over
# system copies — mirrors cvcpkg's _build_env PATH handling on Linux.
$pathAdd = @()
if ($env:CVC_DEPS_PREFIX) { $pathAdd += (Join-Path $env:CVC_DEPS_PREFIX 'bin') }
$pathAdd += (Join-Path $env:CVC_INSTALL_DIR 'bin')
$env:PATH = (($pathAdd + $env:PATH) -join ';')

$scriptPath = Join-Path $env:CVC_RECIPE_DIR $job.script
if (-not (Test-Path -LiteralPath $scriptPath)) {
    Write-Host "winhost-run-job: build script not found: $scriptPath"
    exit 3
}

$shell = Find-CvcBuildShell -DepsPrefix $env:CVC_DEPS_PREFIX
Write-Host "winhost-run-job: shell=$shell"
Write-Host "winhost-run-job: script=$scriptPath"
Write-Host "winhost-run-job: source=$env:CVC_SOURCE_DIR"
Write-Host "winhost-run-job: install=$env:CVC_INSTALL_DIR"

# Run the recipe build script from the build dir, like run_build does.
Set-Location -LiteralPath $env:CVC_BUILD_DIR
& $shell -NoProfile -NonInteractive -ExecutionPolicy Bypass -File $scriptPath
$code = $LASTEXITCODE

if ($code -ne 0) {
    Write-Host "winhost-run-job: build script exited with code $code"
    exit $code
}
Write-Host "== winhost-run-job: $($job.recipe) OK =="
exit 0
