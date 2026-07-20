<#
.SYNOPSIS
    Build a self-contained OFFLINE DEV KIT for the LiDAR Route Planner, so the
    air-gapped machine can develop (edit + run from source + build the .exe) with
    no internet. The online step happens once, here.

.DESCRIPTION
    Run this on a machine WITH internet. It produces a kit folder containing:
      wheelhouse\          every Python dependency (+ transitive) as .whl, built
                           for the TARGET Python version / win_amd64.
      python-<ver>.exe     the matching CPython installer (unless -SkipPythonInstaller).
      repo.bundle          a full-history git bundle of this repo (unless -SkipBundle),
                           so the standalone can 'git clone' it offline.
      install_offline.ps1  the companion installer to run on the standalone.
      requirements*.txt    copied so the offline install resolves the same set.
      KIT_README.txt       instructions.

    Copy the whole kit folder to the air-gapped machine and run install_offline.ps1
    there. The wheelhouse is version-specific: -PyVersion MUST match the Python that
    will run on the standalone.

.PARAMETER PyVersion
    Full CPython version to target, e.g. 3.12.10. The wheelhouse and the bundled
    installer are built for this. Default 3.12.10 (strong binary-wheel coverage).

.PARAMETER KitDir
    Output folder for the kit. Default C:\arp_offline_kit (kept off OneDrive).

.PARAMETER RepoRoot
    Repo to package. Default: this script's folder.

.PARAMETER SkipPythonInstaller
    Do not download the CPython installer (the standalone already has that version).

.PARAMETER SkipBundle
    Do not create repo.bundle (you move the code another way).
#>
[CmdletBinding()]
param(
    [string]$PyVersion = "3.12.10",
    [string]$KitDir    = "C:\arp_offline_kit",
    [string]$RepoRoot  = $PSScriptRoot,
    [switch]$SkipPythonInstaller,
    [switch]$SkipBundle
)

$ErrorActionPreference = "Stop"
function Info($m) { Write-Host "[kit] $m"  -ForegroundColor Cyan }
function Ok($m)   { Write-Host "[ ok ] $m" -ForegroundColor Green }
function Warn($m) { Write-Host "[warn] $m" -ForegroundColor Yellow }
function Die($m)  { Write-Host "[fail] $m" -ForegroundColor Red; exit 1 }

# major.minor for pip's --python-version (e.g. 3.12 from 3.12.10)
$parts = $PyVersion -split '\.'
if ($parts.Count -lt 2) { Die "PyVersion must look like 3.12.10 (got '$PyVersion')." }
$PyMM = "$($parts[0]).$($parts[1])"

$req = Join-Path $RepoRoot "requirements-desktop.txt"
if (-not (Test-Path $req)) { Die "requirements-desktop.txt not found in $RepoRoot." }
try { python --version *> $null; if (-not $?) { throw } }
catch { Die "Python not found on PATH - run this on the connected build machine." }

Info "Target Python $PyVersion (wheels tagged py$PyMM / win_amd64)"
New-Item -ItemType Directory -Force -Path $KitDir | Out-Null
$wheelhouse = Join-Path $KitDir "wheelhouse"
if (Test-Path $wheelhouse) { Remove-Item $wheelhouse -Recurse -Force }
New-Item -ItemType Directory -Force -Path $wheelhouse | Out-Null

# 1. Wheelhouse - cross-version download (--only-binary is required for this).
#    Args are splatted (not backtick-continued) so the native parser can't misread
#    '--only-binary=:all:'.
Info "Downloading wheels -> $wheelhouse (this can take a few minutes / ~300+ MB)"
$pipArgs = @(
    '-m', 'pip', 'download', '-r', $req,
    '--only-binary=:all:', '--python-version', $PyMM, '--platform', 'win_amd64',
    '--dest', $wheelhouse
)
python @pipArgs
if (-not $?) {
    Die "pip download failed. Usually means a dep has no win_amd64 wheel for py$PyMM - try a different -PyVersion (3.12 / 3.11 have the best coverage)."
}
$n = (Get-ChildItem $wheelhouse -Filter *.whl).Count
Ok "Wheelhouse ready: $n wheels."

# 2. Matching CPython installer.
if (-not $SkipPythonInstaller) {
    $exeName = "python-$PyVersion-amd64.exe"
    $exeDest = Join-Path $KitDir $exeName
    $url = "https://www.python.org/ftp/python/$PyVersion/$exeName"
    Info "Fetching CPython installer: $url"
    try {
        Invoke-WebRequest -Uri $url -OutFile $exeDest -UseBasicParsing
        Ok "Bundled installer: $exeName"
    } catch {
        Warn "Could not download $exeName. Add it to the kit by hand, or re-run with a valid -PyVersion."
    }
}

# 3. Full-history repo bundle (offline clone source).
if (-not $SkipBundle) {
    try {
        git --version *> $null
        if ($?) {
            $bundle = Join-Path $KitDir "repo.bundle"
            git -C $RepoRoot bundle create $bundle --all
            if ($?) { Ok "Repo bundle: repo.bundle (git clone it on the standalone)" }
            else    { Warn "git bundle failed - moving code another way is fine." }
        } else { Warn "git not available - skipping repo.bundle." }
    } catch { Warn "git bundle skipped." }
}

# 4. Installer script + requirements copies.
$selfInstaller = Join-Path $RepoRoot "install_offline.ps1"
if (Test-Path $selfInstaller) { Copy-Item $selfInstaller $KitDir -Force }
else { Warn "install_offline.ps1 not found next to this script - copy it into the kit manually." }
Copy-Item (Join-Path $RepoRoot "requirements.txt") $KitDir -Force -ErrorAction SilentlyContinue
Copy-Item $req $KitDir -Force

# 5. README.
$readme = @"
OFFLINE DEV KIT - LiDAR Route Planner
Target Python: $PyVersion (win_amd64).  Built: $(Get-Date -Format s)

On the AIR-GAPPED machine:
  1. If Python $PyMM is not installed, run python-$PyVersion-amd64.exe
     (tick "Add python.exe to PATH").
  2. Open PowerShell in this kit folder and run:
         .\install_offline.ps1 -RepoRoot C:\Users\<you>\projects\Auto-Route-Planning
     It will (git clone repo.bundle if the repo isn't there), create .venv, and
     pip install every dependency from wheelhouse\ with NO network.
  3. Develop:  .\.venv\Scripts\Activate.ps1 ; python desktop.py
     Build exe: .\build.ps1

Everything here is version-specific to Python $PyMM. For a different Python, rebuild
the kit with make_offline_kit.ps1 -PyVersion <x.y.z>.
"@
Set-Content -Path (Join-Path $KitDir "KIT_README.txt") -Value $readme -Encoding UTF8

Ok "Offline kit complete:"
Write-Host "      $KitDir" -ForegroundColor Green
Write-Host "Copy that whole folder to the air-gapped machine and run install_offline.ps1." -ForegroundColor Green
