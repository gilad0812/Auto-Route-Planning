<#
.SYNOPSIS
    Compile the LiDAR Route Planner installer (Setup.exe) from the frozen bundle.

.DESCRIPTION
    Wraps the PyInstaller ONEDIR bundle (dist\LidarRoutePlanner\, from build.ps1) into a
    single per-user Setup.exe via Inno Setup. No admin needed to install; the target
    machine needs no Python.

    Prerequisites on THIS (build) machine:
      1. The bundle exists — run  .\build.ps1  first.
      2. Inno Setup 6 is installed (provides ISCC.exe). Air-gapped: copy the Inno Setup
         installer over once and install it, or pass -Iscc <path to ISCC.exe>.

    Output:  dist\installer\LidarRoutePlanner-Setup-<version>.exe

.PARAMETER Version
    Version stamped into the installer + filename. Default 1.0.0.

.PARAMETER SourceDir
    The onedir bundle to package. Default: dist\LidarRoutePlanner (build.ps1 default).

.PARAMETER Iscc
    Full path to ISCC.exe, if it isn't auto-found on PATH or in the usual install dirs.

.EXAMPLE
    .\build.ps1 ; .\make_installer.ps1 -Version 1.2.0
#>
[CmdletBinding()]
param(
    [string]$Version   = "1.0.0",
    [string]$SourceDir = (Join-Path $PSScriptRoot "dist\LidarRoutePlanner"),
    [string]$Iscc      = ""
)

$ErrorActionPreference = "Stop"
function Info($m) { Write-Host "[installer] $m" -ForegroundColor Cyan }
function Ok($m)   { Write-Host "[ ok ] $m"      -ForegroundColor Green }
function Die($m)  { Write-Host "[fail] $m"      -ForegroundColor Red; exit 1 }

$Iss = Join-Path $PSScriptRoot "installer\LidarRoutePlanner.iss"
if (-not (Test-Path $Iss)) { Die "Missing $Iss" }

# 1. The frozen bundle must exist and hold the app exe.
$AppExe = Join-Path $SourceDir "LidarRoutePlanner.exe"
if (-not (Test-Path $AppExe)) {
    Die "Bundle not found: $AppExe`n       Run .\build.ps1 first (onedir), or pass -SourceDir <folder with LidarRoutePlanner.exe>."
}
Ok "Bundling: $SourceDir"

# 2. Locate ISCC.exe (Inno Setup compiler) across the usual install roots.
if (-not $Iscc) {
    $roots = @($env:LOCALAPPDATA, ${env:ProgramFiles(x86)}, $env:ProgramFiles) |
             Where-Object { $_ }
    $subs = @("Programs\Inno Setup 6\ISCC.exe", "Inno Setup 6\ISCC.exe",
              "Inno Setup 5\ISCC.exe")
    foreach ($r in $roots) {
        foreach ($s in $subs) {
            $c = Join-Path $r $s
            if (Test-Path $c) { $Iscc = $c; break }
        }
        if ($Iscc) { break }
    }
    if (-not $Iscc) {
        $cmd = Get-Command iscc -ErrorAction SilentlyContinue
        if ($cmd) { $Iscc = $cmd.Source }
    }
}
if (-not $Iscc -or -not (Test-Path $Iscc)) {
    Die ("Inno Setup compiler (ISCC.exe) not found.`n" +
         "       Install Inno Setup 6 (https://jrsoftware.org/isdl.php) on this build`n" +
         "       machine, or pass -Iscc <path to ISCC.exe>.")
}
Info "ISCC: $Iscc"

# 3. Compile. Pass an ABSOLUTE source dir so [Files] resolves regardless of cwd.
$outDir = Join-Path $PSScriptRoot "dist\installer"
Info "Compiling installer (version $Version)..."
& $Iscc "/DAppVersion=$Version" "/DSourceDir=$SourceDir" $Iss
if ($LASTEXITCODE -ne 0) { Die "ISCC failed (exit $LASTEXITCODE)." }

$setup = Join-Path $outDir "LidarRoutePlanner-Setup-$Version.exe"
if (-not (Test-Path $setup)) { Die "ISCC reported success but $setup is missing." }
Ok "Installer built:"
Write-Host "      $setup" -ForegroundColor Green
Write-Host "Copy that one Setup.exe to the standalone machine and run it (no admin needed)." -ForegroundColor Green
