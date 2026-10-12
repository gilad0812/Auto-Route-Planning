"""End-to-end self-test of the app — run from source or from the packaged .exe:

    python desktop.py --self-test [OUT_DIR]
    LidarRoutePlanner.exe --self-test [OUT_DIR]

It exercises what pytest can't see in a frozen build: every native library actually
loads inside the bundle (GDAL, PROJ data, GEOS, the LAZ codec, the numba/LLVM JIT,
matplotlib, Qt platform plugins), and the real main window opens a DTM, takes a polygon
and computes a route + density estimate. Writes OUT_DIR/selftest.log (one PASS/FAIL line
per check) and OUT_DIR/selftest.png (the window after Compute), and exits 0 only if every
check passed. CI runs it on each build; it's also a quick check on the standalone machine.
OUT_DIR defaults to the system temp folder.
"""
import io
import os
import sys
import tempfile
import time
import traceback


def run(out_dir=None):
    out_dir = out_dir or tempfile.gettempdir()
    os.makedirs(out_dir, exist_ok=True)
    log_path = os.path.join(out_dir, 'selftest.log')
    lines, failed = [], []

    def log(msg):
        lines.append(msg)
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')

    def check(name, fn):
        t0 = time.perf_counter()
        try:
            detail = fn() or ''
            log(f'PASS  {name}  ({time.perf_counter() - t0:.1f}s) {detail}'.rstrip())
        except Exception:
            failed.append(name)
            log(f'FAIL  {name}\n' + traceback.format_exc())

    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src')
    if os.path.isdir(src) and src not in sys.path:
        sys.path.insert(0, src)                       # running from source
    from ui.version import __version__
    log(f'LiDAR Route Planner self-test — version {__version__}, '
        f'frozen={getattr(sys, "frozen", False)}, python {sys.version.split()[0]}')

    tmp = tempfile.mkdtemp(prefix='rp_selftest_')
    dtm_path = os.path.join(tmp, 'selftest_dtm.tif')
    n = 600

    def gdal_write_read():
        import numpy as np
        import rasterio
        from rasterio.transform import from_origin
        yy, xx = np.mgrid[0:n, 0:n]
        z = (300 + 0.05 * xx + 10 * np.sin(yy / 60.0)).astype('float32')
        with rasterio.open(dtm_path, 'w', driver='GTiff', height=n, width=n, count=1,
                           dtype='float32', crs='EPSG:32636', nodata=-9999.0,
                           transform=from_origin(700000, 3500000 + n, 1, 1)) as d:
            d.write(z, 1)
        with rasterio.open(dtm_path) as d:
            assert d.crs.to_epsg() == 32636 and d.read(1).shape == (n, n)
        return f'rasterio {rasterio.__version__}, GDAL {rasterio.__gdal_version__}'

    def proj_transform():
        import pyproj
        tr = pyproj.Transformer.from_crs('EPSG:4326', 'EPSG:32636', always_xy=True)
        e, nn = tr.transform(35.0, 31.5)                 # needs the bundled PROJ database
        assert abs(e - 690_000) < 30_000 and abs(nn - 3_486_000) < 30_000, (e, nn)
        return f'pyproj {pyproj.__version__}'

    def geos_ops():
        import shapely
        from shapely.geometry import Polygon
        p = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
        assert abs(p.buffer(1).area - (140 + 3.1416)) < 0.05      # square + rounded rim
        return f'shapely {shapely.__version__}'

    def laz_roundtrip():
        import laspy
        import numpy as np
        las = laspy.create(point_format=3, file_version='1.2')
        las.x = np.arange(100, dtype=float); las.y = las.x; las.z = las.x
        buf = io.BytesIO()
        las.write(buf, do_compress=True)                 # LAZ needs the lazrs backend
        buf.seek(0)
        assert len(laspy.read(buf).points) == 100
        return f'laspy {laspy.__version__}'

    def matplotlib_render():
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(2, 2)); ax.plot([0, 1], [0, 1])
        buf = io.BytesIO(); fig.savefig(buf, format='png'); plt.close(fig)
        assert buf.tell() > 500
        return f'matplotlib {matplotlib.__version__}'

    def numba_matches_numpy():
        import numba
        from density_estimate import estimate_density_grid
        from density_estimate_nb import estimate_density_grid_nb
        from dtm import DTM
        dtm = DTM(dtm_path)
        x0, y0, x1, y1 = 700100, 3500100, 700400, 3500400
        region = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        route = []
        for i, x in enumerate(range(x0, x1, 45)):
            route += [{'x': float(x), 'y': float(y0), 'z': 400.0, 'pass_id': i},
                      {'x': float(x), 'y': float(y1), 'z': 400.0, 'pass_id': i}]
        kw = dict(pulse_freq_hz=600_000, scan_freq_hz=224.4, scan_half_angle_deg=50.0,
                  speed_ms=6.0, min_points=50, is_geo=False, cell_size_m=1.0)
        a = estimate_density_grid(route, dtm, region, **kw)
        b = estimate_density_grid_nb(route, dtm, region, **kw)   # JIT-compiles in-bundle
        for k in ('n_fail', 'n_cells', 'median_density', 'min_density'):
            assert a[k] == b[k], f'{k}: numpy={a[k]} numba={b[k]}'
        return f'numba {numba.__version__}, {a["n_cells"]} cells identical'

    def gui_compute():
        from unittest import mock
        from PySide6.QtWidgets import QApplication
        from shapely.geometry import mapping, box
        from ui.main_window import MainWindow
        app = QApplication.instance() or QApplication(sys.argv[:1])
        w = MainWindow(); w.resize(1280, 800); w.show(); app.processEvents()
        assert __version__ in w.windowTitle(), w.windowTitle()
        with mock.patch('ui.main_window.QFileDialog.getOpenFileName',
                        return_value=(dtm_path, '')):
            w._open_dtm()
        app.processEvents()
        assert w.dtm is not None, 'DTM did not open'
        w._on_polygon_drawn(mapping(box(700100, 3500100, 700450, 3500450)))
        with mock.patch('ui.main_window.QMessageBox.critical') as crit:
            w._compute(); app.processEvents()
        assert not crit.called, crit.call_args
        r = w.result
        assert r is not None and r.route, 'no route computed'
        est = r.estimate
        passes = len({p['pass_id'] for p in r.route})
        w.grab().save(os.path.join(out_dir, 'selftest.png'))
        w.close()
        return (f'{passes} lines, {len(r.route)} waypoints, '
                f'{est["n_cells"]} cells estimated, Qt platform {app.platformName()}')

    check('GeoTIFF write/read (GDAL)', gdal_write_read)
    check('Coordinate transform (PROJ data)', proj_transform)
    check('Geometry ops (GEOS)', geos_ops)
    check('LAZ write/read (lazrs)', laz_roundtrip)
    check('Chart render (matplotlib)', matplotlib_render)
    check('Density estimate: numba JIT == NumPy', numba_matches_numpy)
    check('Main window: open DTM, polygon, Compute', gui_compute)

    log('SELFTEST ' + ('FAILED: ' + ', '.join(failed) if failed else 'PASSED'))
    print('\n'.join(lines))
    print(f'log: {log_path}')
    return 0 if not failed else 1
