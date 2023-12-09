""""This is a script to plot the global path from a csv file.

Command Line Arguments:

    file_path (Str): The relative path to the global path file
    interval_spacing (float): The distance between waypoints km, default 30.0
    write (Bool): Whether to write the generated path to a new file
    pos (Tuple): Starting position in latitude and longitude

usage in local_pathfinding terminal:

python3 global_paths/plot_global_path.py <file_path> --interval_spacing <interval_spacing> --write
--pos <latitude> <longitude>


"""
import argparse
import csv
from typing import Dict, List, Tuple

import plotly.graph_objects as go
from custom_interfaces.msg import HelperLatLon

from local_pathfinding.node_mock_global_path import MOCK_GPS, MockGlobalPath

# TODO look into GUI for creating a global path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path", help="The relative path to the global path file")
    parser.add_argument(
        "-i",
        "--interval_spacing",
        type=float,
        default=30.0,
        help="The distance between waypoints km, default 30.0",
    )
    parser.add_argument("--write", action="store_true", help="Write the path to a new file")
    parser.add_argument(
        "--pos",
        nargs=2,
        type=float,
        default=[MOCK_GPS.lat_lon.latitude, MOCK_GPS.lat_lon.longitude],
        help="Starting position",
    )

    args = parser.parse_args()

    lats, lons = get_lats_and_lons(args.file_path, args.interval_spacing, args.pos, args.write)
    plot_global_path(lats, lons)
    print("global path:", lats_and_lons_to_dict(lats, lons), sep="\n")


def get_lats_and_lons(
    file_path: str, interval_spacing: float, pos: tuple, write: bool
) -> Tuple[List[float], List[float]]:
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
        pos = HelperLatLon(latitude=float(pos[0]), longitude=float(pos[1]))
        dest = HelperLatLon(latitude=lats[0], longitude=lons[0])
        path = MockGlobalPath.generate_path(
            dest=dest,
            interval_spacing=interval_spacing,
            pos=pos,
            write=write,
            file_path=file_path,
        )
        lats = []
        lons = []
        for item in path.waypoints:
            lats.append(item.latitude)
            lons.append(item.longitude)

    return lats, lons


def plot_global_path(lats, lons):
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


def lats_and_lons_to_dict(
    lats: List[float], lons: List[float], num_decimals: int = 4
) -> Dict[int, str]:
    return {
        i: f"({lats[i]:.{num_decimals}f}, {lons[i]:.{num_decimals}f})" for i in range(len(lats))
    }


if __name__ == "__main__":
    main()
