# Helpers for publish-cvcpkg.yml's Windows cp312 steps (pycvc-cp312, pycvc-gl-cp312, arix, arisrvx,
# ariglx). Dot-source it: `. ./.github/scripts/windows-xhost.ps1`.
#
# Each package is built by its recipe's own build.ps1, in a child pwsh: pycvc's script rewrites
# CVC_SOURCE_DIR, and _common/env-windows.ps1 sets the MSVC environment and CMAKE_PREFIX_PATH for
# the process it runs in, so a script run in the step's own process would leak both into the next.
#
# Checks run from prefix-xhost ($XHost), laid out as an install of the bundles lays them down (the
# family-windows step makes it, and each package's stage is merged into it once built). A host runs
# there with no PYTHONHOME, no PYTHONPATH and a PATH of the Windows system dirs alone, so it must
# find its Python, its DLLs and its packages where an install puts them. Start-Process reports a
# missing DLL as 0xC0000135, which Git Bash would misreport.
$ErrorActionPreference = 'Stop'

$Workspace = $env:GITHUB_WORKSPACE
$XHost = "$Workspace/prefix-xhost"
$SystemPath = "$env:SystemRoot\System32;$env:SystemRoot;$env:SystemRoot\System32\Wbem"

# Copy every file under $From to the same relative path under $To. With -NoClobber a file $To
# already has is left alone.
function Merge-XTree([string]$From, [string]$To, [switch]$NoClobber) {
  $root = (Resolve-Path -LiteralPath $From).Path.TrimEnd('\', '/')
  foreach ($f in Get-ChildItem -LiteralPath $root -Recurse -File) {
    $dest = Join-Path $To $f.FullName.Substring($root.Length + 1)
    if ($NoClobber -and (Test-Path -LiteralPath $dest)) { continue }
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $dest) | Out-Null
    Copy-Item -LiteralPath $f.FullName -Destination $dest -Force
  }
}

# Build $Package with its recipe's build.ps1 against the ;-separated $Deps roots, into
# stage-$Stage, and merge that stage into prefix-xhost.
function Invoke-XRecipe([string]$Package, [string]$Deps, [string]$Stage) {
  $env:CVC_SOURCE_DIR = $Workspace
  $env:CVC_BUILD_DIR = "$Workspace/build-$Stage"
  $env:CVC_INSTALL_DIR = "$Workspace/stage-$Stage"
  $env:CVC_DEPS_PREFIX = $Deps
  $env:CVC_BUILD_TYPE = 'Release'
  $env:CVC_LINK = 'shared'
  pwsh -NoProfile -File "$Workspace/cvcpkg/recipes/$Package/build.ps1"
  if ($LASTEXITCODE -ne 0) { throw "$Package build.ps1 failed ($LASTEXITCODE)" }
  Merge-XTree "$Workspace/stage-$Stage" $XHost
}

# Run prefix-xhost\bin\<Exe>.exe <ArgLine> from the checkout with the clean environment above,
# killed after $Timeout seconds. Returns its exit code and output, and logs both.
function Invoke-XHost([string]$Name, [string]$Exe, [string]$ArgLine, [int]$Timeout = 120) {
  $o = "$Workspace/xhost-$Name.out"
  $e = "$Workspace/xhost-$Name.err"
  $saved = @{ PATH = $env:PATH; PYTHONHOME = $env:PYTHONHOME; PYTHONPATH = $env:PYTHONPATH }
  $env:PATH = $SystemPath
  $env:PYTHONHOME = $null
  $env:PYTHONPATH = $null
  try {
    $p = Start-Process -FilePath "$XHost/bin/$Exe.exe" -ArgumentList $ArgLine -WorkingDirectory $Workspace `
      -NoNewWindow -PassThru -RedirectStandardOutput $o -RedirectStandardError $e
    if (-not $p.WaitForExit($Timeout * 1000)) {
      $p.Kill()
      $p.WaitForExit()
      throw "$Name ran past $Timeout s"
    }
  } finally {
    $env:PATH = $saved.PATH
    $env:PYTHONHOME = $saved.PYTHONHOME
    $env:PYTHONPATH = $saved.PYTHONPATH
  }
  $text = "$(Get-Content -Raw $o)$(Get-Content -Raw $e)"
  Write-Host ("-- {0}: exit {1} (0x{1:X8})" -f $Name, $p.ExitCode)
  Write-Host $text
  return [pscustomobject]@{ Code = $p.ExitCode; Text = $text }
}

# Invoke-XHost, which must exit 0 and, given $Pattern, print a line matching it.
function Assert-XHost([string]$Name, [string]$Exe, [string]$ArgLine, [string]$Pattern = '') {
  $r = Invoke-XHost $Name $Exe $ArgLine
  if ($r.Code -ne 0) { throw ('{0}: exit {1} (0x{1:X8})' -f $Name, $r.Code) }
  if ($Pattern -and -not ($r.Text -split "`r?`n" | Select-String -Pattern $Pattern -Quiet)) {
    throw "${Name}: no line of its output matches /$Pattern/"
  }
  return $r
}

# Pack stage-$Stage as $Package's windows/release/shared bundle at +cvc.$Revision, and publish it
# when DO_PUBLISH is 1 (a tag push, or a dispatch with dry_run=false).
function Publish-XPackage([string]$Package, [string]$Stage, [string]$Revision) {
  New-Item -ItemType Directory -Force -Path "$Workspace/dist" | Out-Null
  cvcpkg pack "$Workspace/cvcpkg/recipes/$Package" --from-prefix "$Workspace/stage-$Stage" `
    --platform windows --config release --link shared --local `
    --cvc-revision $Revision --output-dir "$Workspace/dist"
  if ($LASTEXITCODE -ne 0) { throw "$Package pack failed ($LASTEXITCODE)" }
  if ($env:DO_PUBLISH -ne '1') {
    Write-Host "dry-run: skipping $Package publish"
    return
  }
  if (-not $env:CVCPKG_TOKEN) { throw 'CVCPKG_TOKEN secret is required to publish' }
  cvcpkg publish $Package `
    --server "$env:CVCPKG_SERVER" --org "$env:CVCPKG_ORG" `
    --platform windows --config release --link shared --output-dir "$Workspace/dist" `
    --token "$env:CVCPKG_TOKEN"
  if ($LASTEXITCODE -ne 0) { throw "$Package publish failed ($LASTEXITCODE)" }
}
