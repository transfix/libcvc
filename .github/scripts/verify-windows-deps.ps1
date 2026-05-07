# Verify that all non-system DLL dependencies of the given binaries are
# resolved within the same directory.
#
# Usage:
#   .\verify-windows-deps.ps1 -BinDir <path> [-Binaries volrover3.exe,cvc.dll]
#
# Exits with non-zero status (and prints what's missing) if any required
# DLL is absent from <BinDir>. System DLLs (kernel32, user32, msvcp140,
# vcruntime140, ucrtbase, api-ms-*, ext-ms-*, etc.) are ignored.
#
# Designed to run in GitHub Actions windows-* runners, which have
# dumpbin.exe available via the Visual Studio "Developer PowerShell"
# environment. The CI workflow activates that environment via
# ilammy/msvc-dev-cmd or the vcvars64.bat shim before invoking us.

param(
  [Parameter(Mandatory=$true)] [string]   $BinDir,
  [Parameter(Mandatory=$false)] [string[]] $Binaries = @()
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path $BinDir)) {
  Write-Error "BinDir does not exist: $BinDir"
  exit 2
}

$BinDir = (Resolve-Path $BinDir).Path
Write-Host "Verifying runtime dependencies under: $BinDir"

# Auto-discover binaries if the caller did not name them.
if (-not $Binaries -or $Binaries.Count -eq 0) {
  $Binaries = @()
  Get-ChildItem -Path $BinDir -Filter *.exe -File | ForEach-Object { $Binaries += $_.Name }
  Get-ChildItem -Path $BinDir -Filter *.dll -File | ForEach-Object { $Binaries += $_.Name }
}

if (-not (Get-Command dumpbin -ErrorAction SilentlyContinue)) {
  Write-Error "dumpbin.exe not on PATH. Run from a Visual Studio developer shell (vcvars64.bat / ilammy/msvc-dev-cmd)."
  exit 2
}

# DLLs that are part of Windows / the MSVC runtime — never required to
# ship alongside our binaries. Conservative list; matched case-insensitively.
$systemDllPatterns = @(
  '^kernel32\.dll$', '^user32\.dll$', '^gdi32\.dll$', '^advapi32\.dll$',
  '^shell32\.dll$', '^ole32\.dll$', '^oleaut32\.dll$', '^comctl32\.dll$',
  '^comdlg32\.dll$', '^ws2_32\.dll$', '^wsock32\.dll$', '^crypt32\.dll$',
  '^bcrypt\.dll$', '^ncrypt\.dll$', '^secur32\.dll$', '^iphlpapi\.dll$',
  '^dnsapi\.dll$', '^netapi32\.dll$', '^userenv\.dll$', '^psapi\.dll$',
  '^version\.dll$', '^winmm\.dll$', '^winspool\.drv$', '^uxtheme\.dll$',
  '^dwmapi\.dll$', '^dbghelp\.dll$', '^imm32\.dll$', '^rpcrt4\.dll$',
  '^setupapi\.dll$', '^shlwapi\.dll$', '^urlmon\.dll$', '^wininet\.dll$',
  '^d3d9\.dll$', '^d3d11\.dll$', '^d3d12\.dll$', '^dxgi\.dll$',
  '^opengl32\.dll$', '^glu32\.dll$', '^msimg32\.dll$',
  '^mf\.dll$', '^mfplat\.dll$', '^mfreadwrite\.dll$',
  '^msvcp140\.dll$', '^msvcp140_1\.dll$', '^msvcp140_2\.dll$',
  '^vcruntime140\.dll$', '^vcruntime140_1\.dll$',
  '^concrt140\.dll$', '^vccorlib140\.dll$',
  '^ucrtbase\.dll$', '^ucrtbased\.dll$', '^msvcrt\.dll$',
  '^api-ms-.*\.dll$', '^ext-ms-.*\.dll$',
  '^hvsifiletrust\.dll$', '^pdmutilities\.dll$',
  # Windows codecs / WIC
  '^windowscodecs\.dll$', '^propsys\.dll$', '^msctf\.dll$',
  '^combase\.dll$', '^cfgmgr32\.dll$',
  # GPU vendor user-mode drivers loaded by the OS, not by us
  '^nvcuda\.dll$', '^nvapi.*\.dll$',
  # CUDA driver API (lives with the NVIDIA driver, NOT the toolkit)
  '^cuda\.dll$'
)

function Test-IsSystemDll([string]$name) {
  foreach ($pat in $systemDllPatterns) {
    if ($name -imatch $pat) { return $true }
  }
  return $false
}

function Get-DependentDlls([string]$path) {
  $out = & dumpbin /dependents $path 2>&1
  $deps = @()
  $inSection = $false
  foreach ($line in $out) {
    if ($line -match '^\s*Image has the following dependencies:') {
      $inSection = $true
      continue
    }
    if ($inSection) {
      if ($line -match '^\s*Summary') { break }
      if ($line -match '^\s*([A-Za-z0-9_.+-]+\.dll)\s*$') {
        $deps += $Matches[1]
      }
    }
  }
  return $deps
}

$missing = @{}
$visited = New-Object System.Collections.Generic.HashSet[string]
$queue   = New-Object System.Collections.Generic.Queue[string]
foreach ($b in $Binaries) { [void]$queue.Enqueue($b) }

while ($queue.Count -gt 0) {
  $current = $queue.Dequeue()
  $key = $current.ToLowerInvariant()
  if (-not $visited.Add($key)) { continue }

  $full = Join-Path $BinDir $current
  if (-not (Test-Path $full)) {
    if (-not (Test-IsSystemDll $current)) {
      if (-not $missing.ContainsKey($key)) { $missing[$key] = $current }
    }
    continue
  }

  Write-Host "  walking $current"
  $deps = Get-DependentDlls $full
  foreach ($d in $deps) {
    $dKey = $d.ToLowerInvariant()
    if ($visited.Contains($dKey)) { continue }
    if (Test-IsSystemDll $d) { continue }
    $bundled = Join-Path $BinDir $d
    if (Test-Path $bundled) {
      [void]$queue.Enqueue($d)
    } else {
      if (-not $missing.ContainsKey($dKey)) { $missing[$dKey] = $d }
    }
  }
}

if ($missing.Count -gt 0) {
  Write-Host ""
  Write-Host "::error::Missing non-system DLL dependencies under $BinDir :"
  foreach ($name in ($missing.Values | Sort-Object -Unique)) {
    Write-Host "  - $name"
  }
  exit 1
}

Write-Host ""
Write-Host "OK: all non-system DLL dependencies resolve within $BinDir."
