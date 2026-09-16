<#
.SYNOPSIS
    Build the LiDAR Route Planner desktop .exe, producing a single self-contained
    folder to copy to the air-gapped machine.

.DESCRIPTION
    1. Stops any running instance (it would lock files in the bundle).
    2. Runs PyInstaller (onedir) into a build root (default: the repo root, which
       now lives off OneDrive). build/ and dist/ there are already gitignored.
    3. Verifies the bundle is self-contained (app exe present).

    Run from an activated venv that has pyinstaller + the app's deps:
        .\.venv\Scripts\Activate.ps1
        .\build.ps1

.PARAMETER BuildRoot
    Where PyInstaller writes build/ and dist/. Default: the repo root, so both land
    inside the repo (already gitignored). Pass a different path to build elsewhere.

.PARAMETER OneFile
    Produce a single movable dist\LidarRoutePlanner.exe instead of a onedir folder.
    It self-extracts to a temp dir on each launch (slower startup).

.EXAMPLE
    .\build.ps1 -OneFile    # one movable .exe
#>
[CmdletBinding()]
param(
    [string]$BuildRoot = $PSScriptRoot,
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
    # Invoke via `python -m PyInstaller` (not the bare `pyinstaller` script) so the build
    # uses the active interpreter and doesn't depend on Scripts\ being on PATH.
    python -m PyInstaller desktop.spec --noconfirm --distpath $DistPath --workpath $WorkPath
    if (-not $?) { Die "PyInstaller build failed." }
}
finally { Pop-Location; Remove-Item Env:RP_ONEFILE -ErrorAction SilentlyContinue }

# Where the app landed: a single exe for onefile, the bundle folder for onedir.
if ($OneFile) {
    $AppExe = Join-Path $DistPath "$AppName.exe"
} else {
    $AppExe = Join-Path $Bundle "$AppName.exe"
}
if (-not (Test-Path $AppExe)) { Die "Build finished but $AppExe is missing." }
Ok "App built: $AppExe"

# 3. Done.
if ($OneFile) {
    Ok "Single-file app ready:"
    Write-Host "      $AppExe" -ForegroundColor Green
    Write-Host "Move just that one .exe to the other machine and run it." -ForegroundColor Green
} else {
    Ok "Self-contained bundle ready:"
    Write-Host "      $Bundle" -ForegroundColor Green
    Write-Host "Copy that whole folder to the standalone machine and run $AppName.exe." -ForegroundColor Green
}
