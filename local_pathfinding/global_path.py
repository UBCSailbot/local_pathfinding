"""The Global Path Module, which retrieves the global path from a specified source and sends it
to NET via POST request."""

import argparse
import csv
import json
import os
import time
from urllib.request import urlopen

from custom_interfaces.msg import HelperLatLon, Path

from local_pathfinding.node_mock_global_path import (
    calculate_interval_spacing,
    generate_path,
    interpolate_path,
    path_to_dict,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path", help="The path to the global path csv file.")
    parser.add_argument("--interval", help="Desired path interval length.", type=float)
    parser.add_argument(
        "--write", help="Whether to overwrite the path with the formatted path", type=bool
    )
    args = parser.parse_args()
    path_mod_tmstmp = None
    try:
        print(path_to_dict(get_path(args.file_path)))
    except FileNotFoundError:
        print("File not found. Please enter a valid file path.")
        exit(1)

    while True:
        # check when global path was changed last
        timestamp = time.ctime(os.path.getmtime(args.file_path))

        # update global path if it has been changed
        if not (timestamp == path_mod_tmstmp):
            path = get_path(args.file_path)

            if args.interval is not None:
                path = format_path(
                    path=path,
                    interval_spacing=args.interval,
                    file_path=args.file_path,
                    write=args.write,
                )

            post_path(path)

            path_mod_tmstmp = timestamp


def get_path(file_path: str) -> Path:
    """Returns the global path from the specified file path."""
    path = Path()

    with open(file_path, "r") as file:
        reader = csv.reader(file)
        # skip header
        reader.__next__()
        for row in reader:
            path.waypoints.append(HelperLatLon(latitude=float(row[0]), longitude=float(row[1])))
    return path


def post_path(path: Path):
    """Sends the global path to NET via POST request."""
    waypoints = [
        {"lat": float(item.latitude), "lon": float(item.longitude)} for item in path.waypoints
    ]
    data = {"waypoints": waypoints}
    json_data = json.dumps(data).encode("utf-8")
    urlopen("http://localhost:8081/global-path", json_data)
    print(json_data)


def format_path(path: Path, interval_spacing: float, file_path: str, write: bool) -> Path:
    """Interpolates path to ensure the interval lengths are less than or equal to the specified
    interval spacing."""

    # TODO will need to deserialize this data from JSON into HelperLatLon
    pos = urlopen("http://localhost:3005/api/gps").read()
    pos = json.loads(pos)

    # obtain the actual distances between every waypoint in the path
    path_spacing = calculate_interval_spacing(pos, path.waypoints)

    # check if global path is just a destination point
    if len(path.waypoints) < 2:
        path = generate_path(
            dest=path.waypoints[0],
            interval_spacing=interval_spacing,
            pos=pos,
            write=write,
            file_path=file_path,
        )
    # Check if any waypoints are too far apart
    elif max(path_spacing) > interval_spacing:
        path = interpolate_path(
            global_path=path,
            interval_spacing=interval_spacing,
            pos=pos,
            path_spacing=path_spacing,
            write=write,
            file_path=file_path,
        )

    else:
        return path


if __name__ == "__main__":
    main()
