"""PySide6 desktop UI for the LiDAR drone route planner.

Layout: parameter sidebar (left), 2D Leaflet map (centre — draw the AOI, see
the route + under-density overlay), results sidebar (right). Wired to the
framework-free model in ``src/`` (route_planner, density_estimate).
"""
