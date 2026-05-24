import argparse
import json
import pandas as pd
from shapely.geometry import shape
from .dtm import DTM
from .route_planner import plan_route


def load_polygon(path):
    with open(path, 'r', encoding='utf-8') as f:
        gj = json.load(f)
    geom = gj.get('type') == 'FeatureCollection' and gj['features'][0]['geometry'] or gj.get('geometry') or gj
    return shape(geom)


def main():
    p = argparse.ArgumentParser(description='Auto drone route planning (initial prototype)')
    p.add_argument('--dtm', required=True, help='Path to DTM GeoTIFF')
    p.add_argument('--polygon', required=True, help='Path to polygon GeoJSON')
    p.add_argument('--distance', type=float, required=True, help='Desired distance above surface (m)')
    p.add_argument('--error', type=float, default=0.5, help='Allowed error tolerance (m)')
    p.add_argument('--spacing', type=float, default=10.0, help='Lawnmower pass spacing')
    p.add_argument('--step', type=float, default=5.0, help='Waypoint step along pass')
    p.add_argument('--out', default='route.csv', help='Output CSV file')
    args = p.parse_args()

    dtm = DTM(args.dtm)
    polygon = load_polygon(args.polygon)
    route = plan_route(dtm, polygon, args.distance, args.error, spacing=args.spacing, step=args.step)
    df = pd.DataFrame(route)
    df.to_csv(args.out, index=False)
    print(f'Wrote {len(df)} waypoints to {args.out}')

if __name__ == '__main__':
    main()
