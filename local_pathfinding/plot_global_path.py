import csv
import sys

import plotly.graph_objects as go
from custom_interfaces.msg import HelperLatLon

from local_pathfinding.node_mock_global_path import MOCK_GPS, MockGlobalPath


def main():
    file_name = sys.argv[1]
    file_path = f"/workspaces/sailbot_workspace/src/local_pathfinding/global_paths/{file_name}"
    lats = []
    lons = []

    with open(file_path, "r") as file:
        reader = csv.reader(file)
        # skip header
        reader.__next__()
        for row in reader:
            lats.append(float(row[0]))
            lons.append(float(row[1]))

    if len(lats) < 2:
        interval_spacing = float(sys.argv[2])
        pos = HelperLatLon(
            latitude=MOCK_GPS.lat_lon.latitude, longitude=MOCK_GPS.lat_lon.longitude
        )
        dest = HelperLatLon(latitude=lats[0], longitude=lons[0])
        path = MockGlobalPath.generate_path(
            dest=dest,
            interval_spacing=interval_spacing,
            pos=pos,
            dst_file_path=file_path,
        )
        lats = []
        lons = []
        for item in path.waypoints:
            lats.append(item.latitude)
            lons.append(item.longitude)

    fig = go.Figure(
        data=go.Scattergeo(
            lat=lats,
            lon=lons,
            mode="markers+lines",
            line=dict(width=2, color="blue"),
        )
    )

    fig.update_layout(
        title_text="Mock Global Path Plot",
        showlegend=True,
        geo=dict(
            showland=True,
            showcountries=True,
            showocean=True,
            countrywidth=0.5,
            landcolor="rgb(230, 145, 56)",
            lakecolor="rgb(0, 255, 255)",
            oceancolor="rgb(0, 255, 255)",
            projection=dict(type="orthographic", rotation=dict(lon=-100, lat=40, roll=0)),
            lonaxis=dict(showgrid=True, gridcolor="rgb(102, 102, 102)", gridwidth=0.5),
            lataxis=dict(showgrid=True, gridcolor="rgb(102, 102, 102)", gridwidth=0.5),
        ),
    )

    fig.show()


if __name__ == "__main__":
    sys.exit(main())
