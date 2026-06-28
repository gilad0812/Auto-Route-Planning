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
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
