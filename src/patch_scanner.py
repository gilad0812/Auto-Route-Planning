"""Correct the bundled RIEGL VUX-120-23 scanner so it matches the real datasheet.

HELIOS++ ships its own scanner library (scanners_als.xml), and its
`riegl_vux_120_23` entry is wrong in two ways vs. the manufacturer datasheet:

  * optics = "oscillating"  → should be "rotating" (the VUX uses a rotating
    polygon mirror producing parallel scan lines; an oscillating mirror models a
    different, edge-heavy point distribution).
  * scanAngleMax_deg = "100" → should be "50". HELIOS's scan angle is the
    HALF-angle from nadir, and the sensor's FOV is ±50° (100° total).

This rewrites that scanner block in place. Run after the HELIOS install (see the
Dockerfile) and any time the library is reinstalled. Idempotent and only touches
the riegl_vux_120_23 block, so other scanners are untouched.
"""

import glob
import os
import re

_SCANNER_ID = "riegl_vux_120_23"


def patch(roots=None):
    """Find every scanners_als.xml under `roots` and correct the VUX block.
    Returns the list of files actually modified."""
    roots = roots or ["/app", os.path.expanduser("~"), "."]
    files, seen, changed = [], set(), []
    for r in roots:
        files += glob.glob(os.path.join(r, "**", "scanners_als.xml"), recursive=True)
    for f in files:
        f = os.path.abspath(f)
        if f in seen or not os.path.exists(f):
            continue
        seen.add(f)
        s = open(f, encoding="utf-8").read()
        m = re.search(r'<scanner\s+id\s*=\s*"%s".*?</scanner>' % _SCANNER_ID, s, re.S)
        if not m:
            continue
        block = m.group(0)
        fixed = re.sub(r'(optics\s*=\s*)"oscillating"', r'\1"rotating"', block)
        fixed = re.sub(r'(scanAngle(?:Effective)?Max_deg\s*=\s*)"100"', r'\1"50"', fixed)
        if fixed != block:
            open(f, "w", encoding="utf-8").write(s.replace(block, fixed))
            changed.append(f)
    return changed


if __name__ == "__main__":
    ch = patch()
    print("patch_scanner: corrected", ch if ch else "nothing (already correct or not found)")
