"""Run from the project root to auto-download and install HELIOS++."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from helios_setup import find_helios_binary, download_and_install

def log(msg):
    print(msg, flush=True)

existing = find_helios_binary()
if existing:
    print(f"HELIOS++ already installed: {existing}")
else:
    print("HELIOS++ not found — downloading and installing...")
    binary = download_and_install(log=log)
    print(f"\nInstallation complete: {binary}")
