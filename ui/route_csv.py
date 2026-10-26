"""Route CSVs that round-trip: read a lines CSV keeping every row and column, then write
the same file back with the lines as they are after planning/editing.

A route CSV has one line per row in a WKT column, plus whatever else the operator's
tools put there (ID, X/Y = the line's midpoint, name, description, dates…). On export:

  * a row keeps its original line, with your edits applied: the stretches you flew are
    replaced by their edited form (moved, re-altituded), stretches you deleted are cut
    out, and everything else — parts outside the polygons, parts not selected — stays as
    it was. Cropping to a polygon is not an edit. X / Y = midpoint of the new line's start
    and end. Parts that no longer connect — or, in 3D, meet at different altitudes —
    become a MULTILINESTRING; an unedited line is written exactly as read;
  * a row whose every flown stretch was deleted in Edit Route is removed;
  * a line added with Add Pass becomes a new row, copying every column from the nearest
    original line except X / Y (its own midpoint), WKT, and ID (a fresh one).

Coordinates are written with the precision the row already used, and a 2D WKT stays 2D.
"""
import csv
import math
import os
import re

from shapely.geometry import Point
from shapely.wkt import loads as wkt_loads

# Column names an exported CSV might use for the WKT geometry.
_WKT_COLUMNS = {'wkt', 'geometry', 'geom', 'the_geom', 'wkt_geom', 'wkt_geometry',
                'shape', 'well_known_text'}

_NUM = re.compile(r'-?\d+(?:\.(\d+))?(?:[eE][-+]?\d+)?')


def _looks_like_wkt(s):
    return s.upper().lstrip().startswith((
        'POLYGON', 'MULTIPOLYGON', 'LINESTRING', 'MULTILINESTRING',
        'POINT', 'MULTIPOINT', 'GEOMETRYCOLLECTION', 'LINEARRING'))


def _locate_wkt_column(rows):
    """(column index, data rows) for the WKT column in parsed CSV `rows`, or (None, rows).
    Prefers a known header name; otherwise finds a column whose values look like WKT
    (treating the first row as data if it is itself WKT — a header-less file)."""
    header = rows[0]
    for i, h in enumerate(header):
        if (h or '').strip().lower() in _WKT_COLUMNS:
            return i, rows[1:]
    ncol = max(len(r) for r in rows)
    for i in range(ncol):
        cell = lambda r: (r[i] if i < len(r) else '').strip()
        if _looks_like_wkt(cell(header)):
            return i, rows                      # header row was actually data
        if any(_looks_like_wkt(cell(r)) for r in rows[1:3]):
            return i, rows[1:]
    return None, rows


class RouteTable:
    """One route CSV as read: header, data rows (lists of cell strings), which column is
    which, and the file's format (separator, BOM, line ending) to write it back alike."""

    def __init__(self, path, delim, header, rows, wkt_col, bom, newline):
        self.path, self.delim, self.header, self.rows = path, delim, header, rows
        self.wkt_col, self.bom, self.newline = wkt_col, bom, newline
        low = [(h or '').strip().lower() for h in (header or [])]
        col = lambda n: low.index(n) if n in low else None
        self.x_col, self.y_col, self.id_col = col('x'), col('y'), col('id')

    @property
    def name(self):
        return os.path.basename(self.path)


def read_route_table(path):
    """Read a route CSV keeping every row. Returns (RouteTable, [(row index, geometry)]).
    Raises ValueError with a clear message, like the plain WKT reader."""
    raw = open(path, 'rb').read()
    bom = raw.startswith(b'\xef\xbb\xbf')
    content = raw.decode('utf-8-sig')
    if not content.strip():
        raise ValueError('CSV is empty.')
    newline = '\r\n' if '\r\n' in content else '\n'
    lines = content.splitlines()
    parse_err = None
    for delim in (',', ';', '\t', '|'):         # pick the delimiter that yields a WKT column
        rows = [r for r in csv.reader(lines, delimiter=delim)
                if any((c or '').strip() for c in r)]
        if not rows:
            continue
        col, data = _locate_wkt_column(rows)
        if col is None:
            continue
        header = rows[0] if data is not rows else None
        geoms = []
        try:
            for i, r in enumerate(data):
                v = (r[col] if col < len(r) else '').strip()
                if v:
                    geoms.append((i, wkt_loads(v)))
        except Exception as e:                  # this separator cut the WKT apart
            parse_err = e
            continue
        if geoms:
            return RouteTable(path, delim, header, data, col, bom, newline), geoms
    if parse_err is not None:
        raise ValueError(f'Could not read the WKT in the CSV: {parse_err}')
    raise ValueError('No WKT column found (expected a "WKT"/"geometry" column, or WKT '
                     'text in a column).')


# ----------------------------------------------------------------------------- writing
def _decimals(text):
    """Most decimals any number in `text` is written with (0 if none)."""
    return max((len(m.group(1) or '') for m in _NUM.finditer(text or '')), default=0)


def _fmt(v, nd):
    return f'{v:.{nd}f}' if nd > 0 else str(int(round(v)))


def _z(p):
    return float(p[2]) if len(p) > 2 and p[2] is not None else None


def _chain(pieces, tol, has_z):
    """Join consecutive pieces [((x,y,z),(x,y,z))] that touch end-to-start into polylines.
    For a 3D line they must also meet at the same altitude: a vertex holds one Z, so two
    edges flown at different heights stay separate parts and each keeps its altitude."""
    parts = []
    for a, b in pieces:
        if parts:
            e = parts[-1][-1]
            same_xy = math.hypot(e[0] - a[0], e[1] - a[1]) <= tol
            same_z = (not has_z or _z(e) is None or _z(a) is None
                      or abs(_z(e) - _z(a)) <= 1e-6)
            if same_xy and same_z:
                parts[-1].append(b); continue
        parts.append([a, b])
    return parts


def _same_line(parts, geom, has_z, tol):
    """True if `parts` is exactly the original line `geom` (nothing changed)."""
    orig = ([list(geom.coords)] if geom.geom_type == 'LineString'
            else [list(g.coords) for g in getattr(geom, 'geoms', [])
                  if g.geom_type == 'LineString'])
    if len(orig) != len(parts) or any(len(o) != len(p) for o, p in zip(orig, parts)):
        return False
    return all(math.hypot(o[0] - p[0], o[1] - p[1]) <= tol
               and (not has_z or abs(float(o[2]) - (_z(p) or 0.0)) <= 1e-6)
               for op, pp in zip(orig, parts) for o, p in zip(op, pp))


def _wkt(parts, has_z, nd_xy, nd_z):
    def pt(p):
        s = f'{_fmt(p[0], nd_xy)} {_fmt(p[1], nd_xy)}'
        if has_z:
            z = p[2] if len(p) > 2 and p[2] is not None else 0.0
            s += f' {_fmt(z, nd_z)}'
        return s
    tag = ' Z' if has_z else ''
    body = lambda part: '(' + ', '.join(pt(p) for p in part) + ')'
    if len(parts) == 1:
        return f'LINESTRING{tag} {body(parts[0])}'
    return f'MULTILINESTRING{tag} (' + ', '.join(body(p) for p in parts) + ')'


def _at(a, b, t):
    """The point at fraction t along a→b (Z interpolated when the edge has it)."""
    p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
    if len(a) > 2 and len(b) > 2:
        p += (a[2] + (b[2] - a[2]) * t,)
    return p


def _unchanged(a, b, t0, t1, pc, tol):
    """True if piece `pc` is still exactly the stretch [t0, t1] of edge a→b (flown but not
    edited). Z counts only when the edge itself has Z (a 2D file has no altitude)."""
    for t, p in ((t0, pc[0]), (t1, pc[1])):
        q = _at(a, b, t)
        if math.hypot(q[0] - p[0], q[1] - p[1]) > tol:
            return False
        if len(q) > 2 and (_z(p) is None or abs(q[2] - _z(p)) > 1e-6):
            return False
    return True


def _edge_pieces(a, b, flown, tol=1e-9, eps=1e-9):
    """Edge a→b as written back: the original edge with each flown stretch (t0, t1) swapped
    for its piece as flown now, or cut out if the piece is None (deleted). A stretch that
    was flown unchanged is not an edit, so the original edge stays whole there."""
    out, t = [], 0.0
    flown = [(t0, t1, pc) for t0, t1, pc in flown
             if pc is None or not _unchanged(a, b, t0, t1, pc, tol)]
    for t0, t1, pc in sorted(flown, key=lambda e: e[0]):
        if t0 > t + eps:
            out.append((a if t <= eps else _at(a, b, t), _at(a, b, t0)))
        if pc is not None:
            out.append(pc)
        t = max(t, t1)
    if t < 1.0 - eps:
        out.append((a if t <= eps else _at(a, b, t), b))
    return out


def row_segments(geom):
    """The straight segments of a row's line geometry, in order: [((x,y[,z]), (x,y[,z]))].
    Loading splits a row into exactly these (one selectable pass per segment)."""
    if geom.geom_type == 'LineString':
        lines = [geom]
    elif geom.geom_type in ('MultiLineString', 'GeometryCollection'):
        lines = [g for g in geom.geoms if g.geom_type == 'LineString']
    else:
        lines = []
    segs = []
    for ln in lines:
        if ln.is_empty or len(ln.coords) < 2:
            continue
        cs = list(ln.coords)
        segs.extend(zip(cs, cs[1:]))
    return segs


def write_route_table(table, row_geoms, edits, added, out_path):
    """Write `table` to `out_path` with the planning results applied.

    row_geoms: {row index: original geometry} for the rows that hold a line.
    edits:     {row index: {segment index: [(t0, t1, piece), …]}} — the flown stretches
               [t0, t1] (0 = edge start, 1 = edge end) of a row's edges, each with its
               piece as flown now ((x,y,z), (x,y,z)), or None if deleted. The rest of the
               edge — outside the polygons, not selected — keeps its original geometry.
               Rows not listed are written exactly as read.
    added:     [((x,y,z), (x,y,z)), …] lines with no row of their own (Add Pass): each
               becomes a new row copied from the nearest original line of this table.
    Returns (rows changed, rows removed, rows added)."""
    out_rows, changed, removed = [], 0, 0
    for i, row in enumerate(table.rows):
        row = list(row)
        if i not in edits:
            out_rows.append(row); continue
        geom = row_geoms[i]
        orig_wkt = row[table.wkt_col]
        if all(pc is None for ev in edits[i].values() for _, _, pc in ev):
            removed += 1; continue                       # you deleted this line
        tol = max(geom.length, 1e-9) * 1e-9
        pieces = []                                      # original + edits, in edge order
        for k, (a, b) in enumerate(row_segments(geom)):
            pieces.extend(_edge_pieces(a, b, edits[i].get(k, []), tol))
        has_z = geom.has_z
        nd_xy, nd_z = _xy_decimals(orig_wkt), _z_decimals(orig_wkt)
        parts = _chain(pieces, tol, has_z)
        if _same_line(parts, geom, has_z, tol):          # flown exactly as it was
            out_rows.append(row); continue
        row[table.wkt_col] = _wkt(parts, has_z, nd_xy, nd_z)
        _set_midpoint(table, row, parts[0][0], parts[-1][-1])
        out_rows.append(row); changed += 1

    if added:
        base = [(i, g) for i, g in row_geoms.items()]
        ids = _id_source(table, out_rows)
        for a, b in added:
            mid = Point((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            i_near = min(base, key=lambda ig: ig[1].distance(mid))[0] if base else None
            row = (list(table.rows[i_near]) if i_near is not None
                   else [''] * len(table.header or [table.wkt_col + 1]))
            while len(row) <= table.wkt_col:
                row.append('')
            src_wkt = table.rows[i_near][table.wkt_col] if i_near is not None else ''
            has_z = row_geoms[i_near].has_z if i_near is not None else True
            row[table.wkt_col] = _wkt([[a, b]], has_z, _xy_decimals(src_wkt) or 3,
                                      _z_decimals(src_wkt) or 2)
            _set_midpoint(table, row, a, b)
            if table.id_col is not None and table.id_col < len(row):
                row[table.id_col] = next(ids)
            out_rows.append(row)

    enc = 'utf-8-sig' if table.bom else 'utf-8'
    with open(out_path, 'w', encoding=enc, newline='') as f:
        wr = csv.writer(f, delimiter=table.delim, lineterminator=table.newline)
        if table.header is not None:
            wr.writerow(table.header)
        wr.writerows(out_rows)
    return changed, removed, len(added)


def _coords_text(wkt_text):
    """The coordinate tuples of a WKT string as lists of number strings."""
    body = (wkt_text or '')[max((wkt_text or '').find('('), 0):]
    return [t.split() for t in re.split(r'[(),]', body) if t.strip()]


def _xy_decimals(wkt_text):
    return max((max(_decimals(t[0]), _decimals(t[1])) for t in _coords_text(wkt_text)
                if len(t) >= 2), default=0)


def _z_decimals(wkt_text):
    return max((_decimals(t[2]) for t in _coords_text(wkt_text) if len(t) >= 3), default=0)


def _set_midpoint(table, row, start, end):
    """X / Y columns = the midpoint of the line's start and end, in the column's precision."""
    for col, v in ((table.x_col, (start[0] + end[0]) / 2),
                   (table.y_col, (start[1] + end[1]) / 2)):
        if col is None:
            continue
        while len(row) <= col:
            row.append('')
        row[col] = _fmt(v, _decimals(row[col]) if row[col].strip() else 3)


def _id_source(table, rows):
    """Fresh IDs for new rows: max + 1, … when the IDs are whole numbers, else
    '<nearest>-new1', … made unique."""
    if table.id_col is None:
        while True:
            yield ''
    vals = [r[table.id_col].strip() for r in rows if table.id_col < len(r)]
    if vals and all(re.fullmatch(r'-?\d+', v) for v in vals if v):
        n = max((int(v) for v in vals if v), default=0)
        while True:
            n += 1
            yield str(n)
    taken, k = set(vals), 0
    while True:
        k += 1
        cand = f'new{k}'
        if cand not in taken:
            taken.add(cand)
            yield cand
