<#
.SYNOPSIS
    Build the LiDAR Route Planner desktop .exe and bake HELIOS++ into the bundle,
    producing a single self-contained folder to copy to the air-gapped machine.

.DESCRIPTION
    1. Stops any running instance (it would lock files in the bundle).
    2. Runs PyInstaller (onedir) into a build root (default: the repo root, which
       now lives off OneDrive). build/ and dist/ there are already gitignored.
    3. Copies the HELIOS++ install into the bundle as `helios\`, which the app
       finds first via find_helios_binary() when frozen.
    4. Verifies the bundle is self-contained (app exe + helios++.exe present).

    Run from an activated venv that has pyinstaller + the app's deps:
        .\.venv\Scripts\Activate.ps1
        .\build.ps1

.PARAMETER HeliosSource
    Folder holding the HELIOS++ install to bake in. Default C:\helios_bin.

.PARAMETER BuildRoot
    Where PyInstaller writes build/ and dist/. Default: the repo root, so both land
    inside the repo (already gitignored). Pass a different path to build elsewhere.

.PARAMETER SkipHelios
    Build the exe only; do not copy HELIOS beside it.

.PARAMETER OneFile
    Produce a single movable dist\LidarRoutePlanner.exe instead of a onedir folder.
    It self-extracts to a temp dir on each launch (slower startup) and cannot hold
    HELIOS inside it. For a truly single file, combine with -SkipHelios; if you need
    HELIOS validation on the target, the helios\ folder is placed next to the exe and
    the two must travel together.

.EXAMPLE
    .\build.ps1 -OneFile -SkipHelios    # one movable .exe, no HELIOS
#>
[CmdletBinding()]
param(
    [string]$HeliosSource = "C:\helios_bin",
    [string]$BuildRoot    = $PSScriptRoot,
    [switch]$SkipHelios,
    [switch]$OneFile
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

# 2. PyInstaller build. Onedir (default) -> dist\LidarRoutePlanner\ folder;
#    -OneFile -> a single dist\LidarRoutePlanner.exe (via RP_ONEFILE in the spec).
$mode = if ($OneFile) { "onefile (single .exe)" } else { "onedir (folder)" }
Info "Building with PyInstaller [$mode] -> $DistPath"
Push-Location $RepoRoot
try {
    if ($OneFile) { $env:RP_ONEFILE = "1" } else { Remove-Item Env:RP_ONEFILE -ErrorAction SilentlyContinue }
    pyinstaller desktop.spec --noconfirm --distpath $DistPath --workpath $WorkPath
    if (-not $?) { Die "PyInstaller build failed." }
}
finally { Pop-Location; Remove-Item Env:RP_ONEFILE -ErrorAction SilentlyContinue }

# Where the app landed, and where a bundled HELIOS must sit to be found (next to the
# exe, per find_helios_binary): the bundle folder for onedir, dist\ for onefile.
if ($OneFile) {
    $AppExe       = Join-Path $DistPath "$AppName.exe"
    $HeliosParent = $DistPath
} else {
    $AppExe       = Join-Path $Bundle "$AppName.exe"
    $HeliosParent = $Bundle
}
if (-not (Test-Path $AppExe)) { Die "Build finished but $AppExe is missing." }
Ok "App built: $AppExe"

# 3. Place HELIOS++ as helios\ next to the exe (skipped for a truly single-file exe
#    unless you want validation on the target — then it rides alongside the exe).
if ($SkipHelios) {
    Info "SkipHelios set - not bundling HELIOS."
}
else {
    if (-not (Test-Path $HeliosSource)) {
        Die "HELIOS source '$HeliosSource' not found. Pass -HeliosSource <dir> or -SkipHelios."
    }
    $HeliosDest = Join-Path $HeliosParent "helios"
    Info "Copying HELIOS++ from $HeliosSource -> $HeliosDest (this can take a while)..."
    if (Test-Path $HeliosDest) { Remove-Item $HeliosDest -Recurse -Force }
    New-Item -ItemType Directory -Path $HeliosDest -Force | Out-Null
    Copy-Item (Join-Path $HeliosSource "*") $HeliosDest -Recurse -Force

    $heliosExe = Get-ChildItem -Path $HeliosDest -Recurse -Filter "helios++.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $heliosExe) { Die "Copied HELIOS but helios++.exe was not found under $HeliosDest." }
    Ok "HELIOS placed beside the exe: $($heliosExe.FullName)"
}

# 4. Done.
if ($OneFile) {
    Ok "Single-file app ready:"
    Write-Host "      $AppExe" -ForegroundColor Green
    if ($SkipHelios) {
        Write-Host "Move just that one .exe to the other machine and run it." -ForegroundColor Green
    } else {
        Write-Host "Move LidarRoutePlanner.exe AND the helios\ folder together (HELIOS must sit beside the exe)." -ForegroundColor Green
    }
} else {
    Ok "Self-contained bundle ready:"
    Write-Host "      $Bundle" -ForegroundColor Green
    Write-Host "Copy that whole folder to the standalone machine and run $AppName.exe." -ForegroundColor Green
}
