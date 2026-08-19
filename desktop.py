"""Entry point for the desktop (PySide6) build of the route planner.

    python desktop.py                         → launch the GUI
    python desktop.py --helios-selftest DTM   → headless HELIOS run (diagnostics)

The self-test exists so the packaged .exe can be verified end-to-end without
the GUI: it computes a small route on the given DTM, runs HELIOS++ validation,
and writes everything (including the [diag] env lines) to a log file. Use it to
confirm HELIOS works in the frozen build on the target machine.
"""
import os
import sys
import math
import tempfile

# Give numba a WRITABLE on-disk JIT cache (set before anything imports numba). In the
# frozen bundle the modules live in a read-only dir, so without this the kernel would
# recompile on every launch; here it compiles once ever and reloads from cache.
_nb_cache = os.path.join(os.environ.get('LOCALAPPDATA', tempfile.gettempdir()),
                         'LidarRoutePlanner', 'numba_cache')
try:
    os.makedirs(_nb_cache, exist_ok=True)
    os.environ.setdefault('NUMBA_CACHE_DIR', _nb_cache)
except OSError:
    pass


def _prewarm_numba():
    """Trigger the numba JIT compile on a tiny synthetic estimate, in the background,
    so the operator's first real Compute isn't stalled by compilation. Silent + best
    effort — any failure (no numba, etc.) just leaves the NumPy fallback in place."""
    try:
        import types
        import numpy as np
        from affine import Affine
        _src = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src')
        if os.path.isdir(_src) and _src not in sys.path:
            sys.path.insert(0, _src)                    # running from source
        from density_estimate_nb import estimate_density_grid_nb
        n = 24
        arr = (300.0 + np.add.outer(np.arange(n) * 0.1, np.arange(n) * 0.1))
        dtm = types.SimpleNamespace(array=arr,
                                    transform=Affine(1.0, 0.0, 0.0, 0.0, -1.0, float(n)),
                                    nodata=None)
        region = [(2, 2), (20, 2), (20, 20), (2, 20), (2, 2)]
        route = [{'x': 6.0, 'y': 4.0, 'z': 360.0, 'pass_id': 0},
                 {'x': 6.0, 'y': 18.0, 'z': 360.0, 'pass_id': 0}]
        estimate_density_grid_nb(
            route, dtm, region, pulse_freq_hz=600_000, scan_freq_hz=224.4,
            scan_half_angle_deg=50.0, speed_ms=6.0, min_points=50,
            is_geo=False, cell_size_m=1.0)
    except Exception:
        pass


def _helios_selftest(dtm_path):
    log_path = os.path.join(tempfile.gettempdir(), 'helios_selftest.log')
    lines = []

    def log(msg):
        lines.append(str(msg))
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))

    log(f'frozen={getattr(sys, "frozen", False)}  dtm={dtm_path}')
    try:
        from ui.planning import load_dtm, centered_box, compute_plan, PlanParams
        from terrain_converter import dtm_to_obj
        from helios_integration import run_feedback_loop
        from helios_setup import find_helios_binary
        from helios_config import DEFAULT_SCANNER_REF, DEFAULT_PLATFORM_REF

        dtm = load_dtm(dtm_path)
        poly = centered_box(dtm, frac=0.08)
        params = PlanParams()
        res_plan = compute_plan(dtm, poly, params, is_geo=True)
        wps = [w for w in res_plan.route
               if not (isinstance(w['z'], float) and math.isnan(w['z']))]
        log(f'route waypoints={len(wps)}')

        ref_lon = sum(w['x'] for w in wps) / len(wps)
        ref_lat = sum(w['y'] for w in wps) / len(wps)
        half = params.fov_deg / 2.0
        swath = 2.0 * params.altitude_m * math.tan(math.radians(half))
        work = os.path.join(tempfile.gettempdir(), 'helios_selftest_work')
        os.makedirs(work, exist_ok=True)
        obj = os.path.join(work, 'terrain.obj')
        log('building OBJ…')
        dtm_to_obj(dtm_path, obj, step_m=5.0, ref_lon=ref_lon, ref_lat=ref_lat,
                   crop_bounds=poly.bounds, margin_m=swath)
        hb = find_helios_binary()
        log(f'helios_bin={hb}')
        res = run_feedback_loop(
            route=res_plan.route, helios_bin=str(hb), scene_obj_path=obj,
            work_dir=work, is_geo=True, ref_lon=ref_lon, ref_lat=ref_lat,
            altitude_m=params.altitude_m, min_points=params.min_points,
            speed_ms=params.speed_ms, pulse_freq_hz=params.pulse_freq_hz,
            scan_freq_hz=params.scan_freq_hz, scan_angle_deg=half,
            scanner_ref=DEFAULT_SCANNER_REF, platform_ref=DEFAULT_PLATFORM_REF,
            dtm=dtm, region_polygon=list(poly.exterior.coords),
            log=log)
        log(f'RESULT error={res.get("error")} passed={res.get("passed")} '
            f'stats={res.get("density_stats")}')
        log('SELFTEST DONE')
    except Exception as e:
        import traceback
        log('EXCEPTION:\n' + traceback.format_exc())
    print(f'self-test log written to: {log_path}')


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == '--helios-selftest':
        _helios_selftest(sys.argv[2] if len(sys.argv) > 2
                         else 'data/dtm_ca_hills_2m.tif')
        return
    from PySide6.QtWidgets import QApplication
    from ui.main_window import MainWindow
    from ui.style import apply_dark_theme
    app = QApplication(sys.argv)
    app.setApplicationName('LiDAR Route Planner')
    apply_dark_theme(app)
    win = MainWindow()
    win.show()
    # Compile the numba kernel in the background while the operator loads a DTM /
    # draws the polygon, so the first Compute isn't stalled by JIT.
    import threading
    threading.Thread(target=_prewarm_numba, daemon=True).start()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
