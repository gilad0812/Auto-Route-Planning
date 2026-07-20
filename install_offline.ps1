<#
.SYNOPSIS
    Stand up the LiDAR Route Planner DEV environment on an air-gapped machine from
    an offline kit (built by make_offline_kit.ps1). No internet used.

.DESCRIPTION
    Run this from inside the kit folder on the standalone machine. It:
      1. Checks Python is present (points you at the bundled installer if not).
      2. Clones repo.bundle -> RepoRoot if the repo isn't there yet (needs git).
      3. Creates RepoRoot\.venv.
      4. pip install --no-index --find-links wheelhouse -r requirements-desktop.txt.
      5. Verifies the key imports.

    After it finishes you can edit, run 'python desktop.py', and 'build.ps1' - all
    offline.

.PARAMETER RepoRoot
    Where the repo lives (or should be cloned to). Keep it OFF OneDrive.
    Default: %USERPROFILE%\projects\Auto-Route-Planning.

.PARAMETER KitDir
    The kit folder (holds wheelhouse\ + repo.bundle). Default: this script's folder.

.PARAMETER VenvName
    Virtual-env directory name inside RepoRoot. Default .venv.
#>
[CmdletBinding()]
param(
    [string]$RepoRoot = (Join-Path $env:USERPROFILE "projects\Auto-Route-Planning"),
    [string]$KitDir   = $PSScriptRoot,
    [string]$VenvName = ".venv"
)

$ErrorActionPreference = "Stop"
function Info($m) { Write-Host "[setup] $m" -ForegroundColor Cyan }
function Ok($m)   { Write-Host "[ ok ] $m"  -ForegroundColor Green }
function Warn($m) { Write-Host "[warn] $m"  -ForegroundColor Yellow }
function Die($m)  { Write-Host "[fail] $m"  -ForegroundColor Red; exit 1 }

$wheelhouse = Join-Path $KitDir "wheelhouse"
if (-not (Test-Path $wheelhouse)) { Die "wheelhouse\ not found in $KitDir - run this from inside the kit folder." }

# 1. Python present?
try { $pv = (python --version) 2>&1; if (-not $?) { throw } }
catch {
    $inst = Get-ChildItem $KitDir -Filter "python-*-amd64.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($inst) { Die "Python not on PATH. Install it first: $($inst.FullName)  (tick 'Add python.exe to PATH'), then re-run." }
    Die "Python not on PATH and no installer in the kit. Install the target Python, then re-run."
}
Info "Using $pv"

# 2. Get the repo if it isn't there.
if (-not (Test-Path (Join-Path $RepoRoot "requirements-desktop.txt"))) {
    $bundle = Join-Path $KitDir "repo.bundle"
    if (Test-Path $bundle) {
        try {
            git --version *> $null
            if (-not $?) { throw "git missing" }
            Info "Cloning repo.bundle -> $RepoRoot"
            git clone $bundle $RepoRoot
            if (-not $?) { Die "git clone from repo.bundle failed." }
        } catch { Die "Repo not at $RepoRoot and can't clone repo.bundle. Copy the repo there manually, or pass -RepoRoot." }
    } else {
        Die "Repo not found at $RepoRoot and no repo.bundle in the kit. Copy the repo there, or pass -RepoRoot <path>."
    }
}
Ok "Repo: $RepoRoot"

# 3. venv.
$venv = Join-Path $RepoRoot $VenvName
$vpy  = Join-Path $venv "Scripts\python.exe"
if (-not (Test-Path $vpy)) {
    Info "Creating venv -> $venv"
    python -m venv $venv
    if (-not $?) { Die "venv creation failed." }
} else { Info "Reusing existing venv $venv" }

# 4. Offline install (no index - wheelhouse only).
$req = Join-Path $RepoRoot "requirements-desktop.txt"
Info "Installing dependencies from wheelhouse (no network)..."
& $vpy -m pip install --no-index --find-links $wheelhouse -r $req
if (-not $?) { Die "Offline pip install failed - the wheelhouse Python version likely does not match this machine's Python. Rebuild the kit with the matching -PyVersion." }
Ok "Dependencies installed."

# 5. Verify.
Info "Verifying imports..."
& $vpy -c "import PySide6, rasterio, shapely, numpy, matplotlib, laspy; print('imports OK')"
if (-not $?) { Die "Import check failed." }

Ok "Dev environment ready."
Write-Host ""
Write-Host "Next:" -ForegroundColor Green
Write-Host "  cd $RepoRoot" -ForegroundColor Green
Write-Host "  .\$VenvName\Scripts\Activate.ps1" -ForegroundColor Green
Write-Host "  python desktop.py        # run from source" -ForegroundColor Green
Write-Host "  .\build.ps1              # build the .exe" -ForegroundColor Green
