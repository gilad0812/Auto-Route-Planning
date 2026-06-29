<#
.SYNOPSIS
    Build the LiDAR Route Planner desktop .exe and bake HELIOS++ into the bundle,
    producing a single self-contained folder to copy to the air-gapped machine.

.DESCRIPTION
    1. Stops any running instance (it would lock files in the bundle).
    2. Runs PyInstaller (onedir) into a build root OUTSIDE OneDrive, to dodge
       OneDrive file locks and Windows MAX_PATH issues during the build.
    3. Copies the HELIOS++ install into the bundle as `helios\`, which the app
       finds first via find_helios_binary() when frozen.
    4. Verifies the bundle is self-contained (app exe + helios++.exe present).

    Run from an activated venv that has pyinstaller + the app's deps:
        .\.venv\Scripts\Activate.ps1
        .\build.ps1

.PARAMETER HeliosSource
    Folder holding the HELIOS++ install to bake in. Default C:\helios_bin.

.PARAMETER BuildRoot
    Where PyInstaller writes build/ and dist/. Default C:\route_planner_build
    (kept off OneDrive on purpose).

.PARAMETER SkipHelios
    Build the exe only; do not copy HELIOS into the bundle.
#>
[CmdletBinding()]
param(
    [string]$HeliosSource = "C:\helios_bin",
    [string]$BuildRoot    = "C:\route_planner_build",
    [switch]$SkipHelios
)

$ErrorActionPreference = "Stop"
$AppName  = "LidarRoutePlanner"
$RepoRoot = $PSScriptRoot
$DistPath = Join-Path $BuildRoot "dist"
$WorkPath = Join-Path $BuildRoot "build"
$Bundle   = Join-Path $DistPath $AppName

function Info($m) { Write-Host "[build] $m" -ForegroundColor Cyan }
function Ok($m)   { Write-Host "[ ok ] $m"  -ForegroundColor Green }
function Die($m)  { Write-Host "[fail] $m"  -ForegroundColor Red; exit 1 }

# 0. Sanity: spec present, pyinstaller importable.
if (-not (Test-Path (Join-Path $RepoRoot "desktop.spec"))) {
    Die "desktop.spec not found in $RepoRoot - run this from the repo root."
}
try { python -c "import PyInstaller" 2>$null; if (-not $?) { throw } }
catch { Die "PyInstaller not available. Activate the venv first: .\.venv\Scripts\Activate.ps1" }

# 1. Stop a running instance so it can't lock files in the bundle.
$running = Get-Process -Name $AppName -ErrorAction SilentlyContinue
if ($running) {
    Info "Stopping running $AppName ($($running.Count) process(es))..."
    $running | Stop-Process -Force
    Start-Sleep -Milliseconds 500
}

# 2. PyInstaller build (onedir) into the off-OneDrive build root.
Info "Building with PyInstaller -> $DistPath"
Push-Location $RepoRoot
try {
    pyinstaller desktop.spec --noconfirm --distpath $DistPath --workpath $WorkPath
    if (-not $?) { Die "PyInstaller build failed." }
}
finally { Pop-Location }

$AppExe = Join-Path $Bundle "$AppName.exe"
if (-not (Test-Path $AppExe)) { Die "Build finished but $AppExe is missing." }
Ok "App built: $AppExe"

# 3. Bake HELIOS++ into the bundle as helios\.
if ($SkipHelios) {
    Info "SkipHelios set - not bundling HELIOS."
}
else {
    if (-not (Test-Path $HeliosSource)) {
        Die "HELIOS source '$HeliosSource' not found. Pass -HeliosSource <dir> or -SkipHelios."
    }
    $HeliosDest = Join-Path $Bundle "helios"
    Info "Copying HELIOS++ from $HeliosSource -> $HeliosDest (this can take a while)..."
    if (Test-Path $HeliosDest) { Remove-Item $HeliosDest -Recurse -Force }
    New-Item -ItemType Directory -Path $HeliosDest -Force | Out-Null
    Copy-Item (Join-Path $HeliosSource "*") $HeliosDest -Recurse -Force

    $heliosExe = Get-ChildItem -Path $HeliosDest -Recurse -Filter "helios++.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $heliosExe) { Die "Copied HELIOS but helios++.exe was not found under $HeliosDest." }
    Ok "HELIOS baked in: $($heliosExe.FullName)"
}

# 4. Done.
Ok "Self-contained bundle ready:"
Write-Host "      $Bundle" -ForegroundColor Green
Write-Host "Copy that whole folder to the standalone machine and run $AppName.exe." -ForegroundColor Green
