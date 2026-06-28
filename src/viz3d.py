"""Interactive 3D terrain + route view (plotly), UgCS-style.

Renders the DTM as a surface clipped to the route's bounding box, drapes the
planned path on top coloured by terrain clearance, and (optionally) extrudes
no-fly polygons as translucent walls. Returns a plotly Figure for st.plotly_chart.
"""
import math
import numpy as np

_LAT_M = 111139.0


def _route_bounds(route, pad_frac=0.08):
    xs = [w['x'] for w in route]
    ys = [w['y'] for w in route]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    px = (maxx - minx) * pad_frac or 1e-4
    py = (maxy - miny) * pad_frac or 1e-4
    return minx - px, miny - py, maxx + px, maxy + py


def route_3d_figure(dtm, route, clearances, is_geo=True,
                    ceiling_m=120.0, floor_m=30.0, max_surf=180):
    """Build the 3D figure.

    dtm        : DTM object (.array, .transform, .nodata)
    route      : list of waypoint dicts {x,y,z,...} (NaN-z already filtered)
    clearances : list of per-waypoint clearance (m), aligned with route
    Returns a plotly.graph_objects.Figure.
    """
    import plotly.graph_objects as go

    minx, miny, maxx, maxy = _route_bounds(route)
    inv = ~dtm.transform
    c0, r0 = inv * (minx, maxy)      # top-left
    c1, r1 = inv * (maxx, miny)      # bottom-right
    arr = dtm.array
    H, W = arr.shape
    j0, j1 = sorted((int(max(0, min(W - 1, c0))), int(max(0, min(W - 1, c1)))))
    i0, i1 = sorted((int(max(0, min(H - 1, r0))), int(max(0, min(H - 1, r1)))))
    j1 = max(j1, j0 + 1)
    i1 = max(i1, i0 + 1)

    # Downsample the window so the surface stays light.
    sub = arr[i0:i1 + 1, j0:j1 + 1].astype(float)
    if dtm.nodata is not None:
        sub = np.where(sub == dtm.nodata, np.nan, sub)
    sh, sw = sub.shape
    si = max(1, -(-sh // max_surf))      # ceil-divide so result <= max_surf
    sj = max(1, -(-sw // max_surf))
    sub = sub[::si, ::sj]

    rows = np.arange(i0, i1 + 1, si)[:sub.shape[0]]
    cols = np.arange(j0, j1 + 1, sj)[:sub.shape[1]]
    # cell centres -> world coords
    xs = np.array([(dtm.transform * (c + 0.5, 0))[0] for c in cols])
    ys = np.array([(dtm.transform * (0, r + 0.5))[1] for r in rows])
    GX, GY = np.meshgrid(xs, ys)

    # Equalise horizontal scale (lon degrees are shorter than lat in metres).
    lat0 = float(np.nanmean(GY))
    if is_geo:
        xfac = _LAT_M * math.cos(math.radians(lat0))
        yfac = _LAT_M
    else:
        xfac = yfac = 1.0
    GXm = (GX - GX.min()) * xfac
    GYm = (GY - GY.min()) * yfac

    fig = go.Figure()
    fig.add_trace(go.Surface(
        x=GXm, y=GYm, z=sub, colorscale='earth', opacity=0.92,
        showscale=False, name='terrain',
        contours={'z': {'show': True, 'usecolormap': True,
                        'project_z': False, 'width': 1}},
    ))

    # Route, coloured by clearance, segmented per pass so connectors are visible.
    rx = (np.array([w['x'] for w in route]) - GX.min()) * xfac
    ry = (np.array([w['y'] for w in route]) - GY.min()) * yfac
    rz = np.array([w['z'] for w in route])
    cl = np.array(clearances)
    # clamp colour scale to the meaningful band
    cmin, cmax = floor_m, max(ceiling_m, floor_m + 1)
    fig.add_trace(go.Scatter3d(
        x=rx, y=ry, z=rz, mode='lines+markers',
        line=dict(color=cl, colorscale='RdYlGn', cmin=cmin, cmax=cmax, width=5),
        marker=dict(size=2, color=cl, colorscale='RdYlGn', cmin=cmin, cmax=cmax,
                    colorbar=dict(title='Clearance (m)')),
        name='route',
        hovertext=[f'alt {z:.0f} m · clr {c:.0f} m' for z, c in zip(rz, cl)],
        hoverinfo='text',
    ))

    fig.update_layout(
        scene=dict(
            xaxis_title='E (m)', yaxis_title='N (m)', zaxis_title='Elev (m)',
            aspectmode='data',
        ),
        margin=dict(l=0, r=0, t=0, b=0), height=640,
        showlegend=False,
    )
    return fig
