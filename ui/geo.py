"""Display / input coordinate conversion: DTM-frame lon-lat  <->  UTM zone 36N.

The app works internally in the DTM's own CRS (WGS84 lon/lat for a geographic DTM),
but the operator works in UTM 36N (EPSG:32636). These helpers convert only what is
SHOWN on screen and what is TYPED into the coordinate dialogs — the planning and
density maths are untouched. Transformers are cached (hover calls this per mouse-move).

If pyproj is unavailable the helpers pass values through unchanged, so the app still
runs (just showing the raw DTM-frame coordinates).
"""
UTM_EPSG = 32636  # WGS84 / UTM zone 36N — the assumed survey zone

try:
    from pyproj import Transformer, CRS
    _HAVE_PYPROJ = True
except Exception:                                          # pragma: no cover
    _HAVE_PYPROJ = False

_cache = {}


def _wkt(dtm_crs):
    if dtm_crs is None:
        return None
    return dtm_crs.to_wkt() if hasattr(dtm_crs, 'to_wkt') else str(dtm_crs)


def _tf(dtm_crs, to_utm):
    """Cached Transformer between the DTM CRS and UTM 36N (always_xy: x=E/lon first)."""
    wkt = _wkt(dtm_crs)
    if wkt is None:
        return None
    key = (wkt, to_utm)
    t = _cache.get(key)
    if t is None:
        src = CRS.from_user_input(wkt)
        utm = CRS.from_epsg(UTM_EPSG)
        a, b = (src, utm) if to_utm else (utm, src)
        t = Transformer.from_crs(a, b, always_xy=True)
        _cache[key] = t
    return t


def to_utm(dtm_crs, lon, lat):
    """DTM-frame (lon, lat) -> UTM (easting, northing). Pass-through if no pyproj/CRS."""
    tf = _tf(dtm_crs, True) if _HAVE_PYPROJ else None
    if tf is None:
        return lon, lat
    return tf.transform(lon, lat)


def from_utm(dtm_crs, easting, northing):
    """UTM (easting, northing) -> DTM-frame (lon, lat). Pass-through if no pyproj/CRS."""
    tf = _tf(dtm_crs, False) if _HAVE_PYPROJ else None
    if tf is None:
        return easting, northing
    return tf.transform(easting, northing)


def fmt_utm(dtm_crs, lon, lat):
    """A compact UTM readout: 'E 500123  N 3456789'."""
    e, n = to_utm(dtm_crs, lon, lat)
    return f'E {e:,.0f}  N {n:,.0f}'
