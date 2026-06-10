
"""
Auto-detect and install HELIOS++ (https://github.com/3dgeo-heidelberg/helios).

Install target : <project_root>/helios_bin/
Persisted state: <project_root>/helios_bin/.helios_config.json

Detection order
───────────────
1. Saved path in .helios_config.json (fastest, used after first install)
2. Recursive search inside helios_bin/
3. System PATH (shutil.which)

Installation
────────────
Downloads the latest release from GitHub, then runs the installer silently:
  Windows        – NSIS .exe              →  <installer> /S /D=<install_dir>
  Linux / macOS  – conda-constructor .sh  →  <installer> -b -f -p <install_dir>
"""

from __future__ import annotations

import json
import os
import platform
import shutil
import stat
import subprocess
import urllib.request
from pathlib import Path
from typing import Callable, Optional

# ── Install locations ─────────────────────────────────────────────────────────

_PROJECT_ROOT = Path(__file__).parent.parent

# On Windows, install to a short local path to avoid OneDrive sync issues
# and Windows MAX_PATH (260 char) limitations during Conda extraction.
if platform.system() == "Windows":
    INSTALL_DIR = Path("C:/helios_bin")
else:
    INSTALL_DIR = _PROJECT_ROOT / "helios_bin"

_CONFIG_FILE  = _PROJECT_ROOT / "helios_bin" / ".helios_config.json"

_IS_WIN = platform.system() == "Windows"
_BIN_NAME = "helios++.exe" if _IS_WIN else "helios++"

# ── GitHub API ────────────────────────────────────────────────────────────────

_API_LATEST = "https://api.github.com/repos/3dgeo-heidelberg/helios/releases/latest"


# ── Custom scanner assets ─────────────────────────────────────────────────────
# Scanner definitions used by this project that are not shipped with the
# HELIOS++ release. They are appended (idempotently) to the installed
# data/scanners_als.xml so survey references like
# "data/scanners_als.xml#riegl_vux_120_23" resolve on any fresh install.

_CUSTOM_ALS_SCANNERS: dict[str, str] = {
    "riegl_vux_120_23": """\
  <!-- ##### BEGIN RIEGL VUX-120-23 (added by auto-route-planner) ##### -->
  <scanner  id                         = "riegl_vux_120_23"
            accuracy_m                 = "0.01"
            beamDivergence_rad         = "0.0004"
            name                       = "RIEGL VUX-120-23"
            optics                     = "oscillating"
            pulseFreqs_Hz              = "150000,300000,600000,1200000,1800000,2400000"
            pulseLength_ns             = "4"
            rangeMin_m                 = "5"
            scanAngleMax_deg           = "100"
            scanAngleEffectiveMax_deg  = "100"
            scanFreqMin_Hz             = "50"
            scanFreqMax_Hz             = "400">
    <beamOrigin x="0" y="0.085" z="0.06">
      <rot axis="x" angle_deg="90" />
      <rot axis="z" angle_deg="90" />
    </beamOrigin>
    <headRotateAxis x="0" y="0" z="1"/>
  </scanner>
  <!-- ##### END RIEGL VUX-120-23 ##### -->
""",
}


def patch_custom_scanners(binary: Path, log: Optional[Callable[[str], None]] = None) -> None:
    """Append project-specific scanner definitions to the installed
    data/scanners_als.xml (no-op for scanners already present)."""
    def _log(msg: str) -> None:
        if log:
            log(msg)

    xml_path = find_helios_root(binary) / "data" / "scanners_als.xml"
    if not xml_path.is_file():
        _log(f"Warning: scanners_als.xml not found at {xml_path}; skipping custom scanner patch")
        return

    content = xml_path.read_text(encoding="utf-8")
    changed = False
    for scanner_id, block in _CUSTOM_ALS_SCANNERS.items():
        if f'"{scanner_id}"' in content:
            continue
        content = content.replace("</document>", block + "\n</document>", 1)
        changed = True
        _log(f"Added custom scanner '{scanner_id}' to {xml_path}")
    if changed:
        xml_path.write_text(content, encoding="utf-8")


# ─────────────────────────────────────────────────────────────────────────────
# Config persistence
# ─────────────────────────────────────────────────────────────────────────────

def _load_config() -> dict:
    if _CONFIG_FILE.exists():
        try:
            return json.loads(_CONFIG_FILE.read_text(encoding="utf-8"))
        except Exception:
            pass
    return {}


def _save_config(binary: Path) -> None:
    _CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
    cfg = _load_config()
    cfg["binary_path"] = str(binary)
    cfg["version"] = binary.parent.name  # best-effort: folder name often has version
    _CONFIG_FILE.write_text(json.dumps(cfg, indent=2), encoding="utf-8")


# ─────────────────────────────────────────────────────────────────────────────
# Detection
# ─────────────────────────────────────────────────────────────────────────────

def _find_binary_under(directory: Path) -> Optional[Path]:
    """Recursively locate the helios++ binary under *directory*."""
    if not directory.exists():
        return None
    for candidate in directory.rglob(_BIN_NAME):
        if candidate.is_file():
            return candidate
    return None


def find_helios_root(binary: Path) -> Path:
    """
    Return the HELIOS++ root directory (the one that contains the data/ folder),
    walking up at most 4 levels from the binary.  Falls back to binary.parent.
    """
    candidate = binary.parent
    for _ in range(4):
        if (candidate / "data").is_dir():
            return candidate
        candidate = candidate.parent
    return binary.parent


def find_helios_binary() -> Optional[Path]:
    """
    Return path to helios++ binary if already available, else None.

    Checks in order: saved config → project install dir → system PATH.
    """
    # 1. Saved path
    cfg = _load_config()
    saved = cfg.get("binary_path")
    if saved:
        p = Path(saved)
        if p.is_file():
            return p

    # 2. Project install dir
    found = _find_binary_under(INSTALL_DIR)
    if found:
        _save_config(found)
        return found

    # 3. System PATH
    which = shutil.which("helios++")
    if which:
        p = Path(which)
        _save_config(p)
        return p

    return None


# ─────────────────────────────────────────────────────────────────────────────
# GitHub release resolution
# ─────────────────────────────────────────────────────────────────────────────

def _fetch_latest_release() -> dict:
    req = urllib.request.Request(
        _API_LATEST,
        headers={
            "Accept": "application/vnd.github.v3+json",
            "User-Agent": "auto-route-planner",
        },
    )
    with urllib.request.urlopen(req, timeout=20) as resp:
        return json.loads(resp.read())


def _pick_asset(assets: list) -> tuple[str, str]:
    """
    Return (filename, download_url) for the best matching release asset.

    Preference: current OS + x86_64 → current OS only → first available.
    """
    sys_name = platform.system().lower()
    machine  = platform.machine().lower()

    os_keys: list[str]
    if sys_name == "windows":
        os_keys = ["windows"]
    elif sys_name == "linux":
        os_keys = ["linux"]
    elif sys_name == "darwin":
        os_keys = ["macosx", "darwin", "mac"]
    else:
        os_keys = []

    arch_keys = ["x86_64", "amd64", "x64"] if "64" in machine else ["arm64", "aarch64"]

    def score(name: str) -> int:
        n = name.lower()
        return (any(k in n for k in os_keys) * 2) + any(k in n for k in arch_keys)

    ranked = sorted(assets, key=lambda a: score(a["name"]), reverse=True)
    for asset in ranked:
        return asset["name"], asset["browser_download_url"]

    raise RuntimeError(
        f"No suitable HELIOS++ release asset found for {platform.system()}.\n"
        "Download manually: https://github.com/3dgeo-heidelberg/helios/releases"
    )


# ─────────────────────────────────────────────────────────────────────────────
# Installation
# ─────────────────────────────────────────────────────────────────────────────

def _download(url: str, dest: Path, log: Callable[[str], None]) -> None:
    """Stream-download *url* to *dest*, calling *log* with progress updates."""
    _last_pct: list[int] = [-1]

    def _reporthook(count: int, block: int, total: int) -> None:
        if total > 0:
            pct = min(100, count * block * 100 // total)
            if pct != _last_pct[0]:  # log only when percentage changes
                _last_pct[0] = pct
                log(f"Downloading… {pct}%  ({count * block // 1_048_576} / {total // 1_048_576} MB)")
        else:
            mb = count * block // 1_048_576
            if mb != _last_pct[0]:
                _last_pct[0] = mb
                log(f"Downloading… {mb} MB received")

    urllib.request.urlretrieve(url, str(dest), reporthook=_reporthook)


def _install_windows(installer: Path, install_dir: Path, log: Callable[[str], None]) -> Path:
    """Run NSIS installer silently with a custom install directory."""
    log("Running installer (silent)…")
    install_dir.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(
        [str(installer), "/S", f"/D={install_dir.resolve()}"],
        timeout=600,
    )
    binary = _find_binary_under(install_dir)
    if binary is None:
        raise RuntimeError(
            f"Installer exited {result.returncode}, helios++ binary not found under {install_dir}.\n"
            "The installer may have used a different path — check the HELIOS++ installer log."
        )
    if result.returncode != 0:
        log(f"Note: installer returned exit code {result.returncode} (non-fatal — binary found at {binary})")
    return binary


def _install_unix(installer: Path, install_dir: Path, log: Callable[[str], None]) -> Path:
    """Run the conda-constructor installer with a custom prefix.

    The HELIOS++ Unix release is a conda-constructor (Miniforge-style) self-
    extracting shell script, not a CPack installer. Its flags are: -b (batch
    mode, accept license non-interactively), -f (allow existing prefix dir),
    -p PREFIX (install location).
    """
    log(f"Running installer (silent)…")
    install_dir.mkdir(parents=True, exist_ok=True)
    # Make executable
    installer.chmod(installer.stat().st_mode | stat.S_IEXEC)
    subprocess.run(
        [str(installer), "-b", "-f", "-p", str(install_dir.resolve())],
        check=True,
        timeout=600,
    )
    binary = _find_binary_under(install_dir)
    if binary is None:
        raise RuntimeError(
            f"Installer ran but helios++ binary not found under {install_dir}.\n"
            "The -p prefix flag may have been ignored — try running the installer manually."
        )
    return binary


def download_and_install(
    log: Optional[Callable[[str], None]] = None,
) -> Path:
    """
    Download the latest HELIOS++ release from GitHub and install it to
    <project_root>/helios_bin/.

    Args:
        log: optional callback(message) for progress updates (e.g. st.write).

    Returns:
        Path to the installed helios++ binary.

    Raises:
        RuntimeError:  if the download, extraction, or binary search fails.
    """
    def _log(msg: str) -> None:
        if log:
            log(msg)

    _log("Fetching latest HELIOS++ release metadata from GitHub…")
    release = _fetch_latest_release()
    version = release.get("tag_name", "?")
    assets  = release.get("assets", [])

    asset_name, url = _pick_asset(assets)
    _log(f"Selected asset: {asset_name}  ({version})")

    INSTALL_DIR.mkdir(parents=True, exist_ok=True)
    installer_path = INSTALL_DIR / asset_name
    install_target = INSTALL_DIR / f"helios-{version}"

    _download(url, installer_path, _log)
    _log("Download complete.")

    if _IS_WIN:
        binary = _install_windows(installer_path, install_target, _log)
    else:
        binary = _install_unix(installer_path, install_target, _log)

    # Clean up installer file
    try:
        installer_path.unlink()
    except OSError:
        pass

    patch_custom_scanners(binary, _log)

    _save_config(binary)
    _log(f"HELIOS++ ready: {binary}")
    return binary


# ─────────────────────────────────────────────────────────────────────────────
# Public convenience
# ─────────────────────────────────────────────────────────────────────────────

def get_or_install(log: Optional[Callable[[str], None]] = None) -> Path:
    """Return the helios++ binary path, installing it first if necessary."""
    existing = find_helios_binary()
    if existing:
        if log:
            log(f"Found existing HELIOS++ installation: {existing}")
        patch_custom_scanners(existing, log)
        _save_config(existing)
        return existing
    return download_and_install(log)
