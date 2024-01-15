"""The Global Path Module, which retrieves the global path from a specified source and sends it
to NET via POST request."""

import argparse
import csv
import json
import os
import time
from urllib.error import HTTPError, URLError
from urllib.request import urlopen

from custom_interfaces.msg import HelperLatLon, Path

from local_pathfinding.coord_systems import GEODESIC, meters_to_km
from local_pathfinding.node_mock_global_path import (
    calculate_interval_spacing,
    generate_path,
    interpolate_path,
    path_to_dict,
)

# TODO Everyrhing works as expected. just need to write some tests and write up docs

GPS_URL = "http://localhost:3005/api/gps"
PATH_URL = "http://localhost:8081/global-path"
GLOBAL_PATHS_FILE_PATH = "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path", help="The path to the global path csv file.")
    parser.add_argument("--interval", help="Desired path interval length.", type=float)

    args = parser.parse_args()
    path_mod_tmstmp = None
    file_path = args.file_path

    try:
        print(path_to_dict(get_path(file_path)))
    except FileNotFoundError:
        print("File not found. Please enter a valid file path.")
        exit(1)

    # Check if the global path should be updated
    while True:
        timestamp = time.ctime(os.path.getmtime(file_path))

        # We should try to retrieve the position on every loop
        position = None

        try:
            position = json.loads(urlopen("http://localhost:3005/api/gps").read())
        except HTTPError as http_error:
            print(f"HTTP Error: {http_error.code}")
        except URLError as url_error:
            print(f"URL Error: {url_error.reason}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

        if position is None:
            raise Exception(f"Failed to retrieve position from {GPS_URL}")

        lat_lon = position["lat_lon"]
        pos = HelperLatLon(latitude=lat_lon["latitude"], longitude=lat_lon["longitude"])
        path = get_path(file_path)

        position_delta = meters_to_km(
            GEODESIC.inv(
                lats1=pos.latitude,
                lons1=pos.longitude,
                lats2=path.waypoints[0].latitude,
                lons2=path.waypoints[0].longitude,
            )[2]
        )

        # update global path if it has been changed or position is too far from start of path
        if (timestamp == path_mod_tmstmp) and (
            (args.interval is None) or position_delta <= args.interval
        ):
            continue

        if args.interval is not None:
            # format path will interpolate new path and save it to a new csv file
            path = format_path(
                path=path,
                pos=pos,
                interval_spacing=args.interval,
                file_path=file_path,
            )

        post_path(path)

        path_mod_tmstmp = timestamp

        # An alternate option is to have the functions generate_path and interpolate_path return
        # a tuple containing the new path and the file path of the new csv file
        # but this would be a breaking change to multiple files, so this is a quick workaround
        file_path = get_most_recent_file(GLOBAL_PATHS_FILE_PATH)

        # we don't need to update multiple times per second
        time.sleep(1)


def get_most_recent_file(directory_path: str) -> str:
    all_files = os.listdir(directory_path)

    # Filter out directories and get the full file paths
    files = [
        os.path.join(directory_path, file)
        for file in all_files
        if os.path.isfile(os.path.join(directory_path, file))
    ]

    # Sort the files based on their last modification time
    files.sort(key=lambda x: os.path.getmtime(x), reverse=True)

    if files:
        return files[0]
    else:
        return ""


def get_path(file_path: str) -> Path:
    """Returns the global path from the specified file path.

    Args:
        file_path (str): The path to the global path csv file.
    """
    path = Path()

    with open(file_path, "r") as file:
        reader = csv.reader(file)
        # skip header
        reader.__next__()
        for row in reader:
            path.waypoints.append(HelperLatLon(latitude=float(row[0]), longitude=float(row[1])))
    return path


def post_path(path: Path):
    """Sends the global path to NET via POST request.

    Args:
        path (Path): The global path.
    """
    waypoints = [
        {"lat": float(item.latitude), "lon": float(item.longitude)} for item in path.waypoints
    ]
    data = {"waypoints": waypoints}
    json_data = json.dumps(data).encode("utf-8")
    try:
        urlopen("http://localhost:8081/global-path", json_data)
    except HTTPError as http_error:
        print(f"HTTP Error: {http_error.code}")
    except URLError as url_error:
        print(f"URL Error: {url_error.reason}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


def format_path(path: Path, pos: HelperLatLon, interval_spacing: float, file_path: str) -> Path:
    """Interpolates path to ensure the interval lengths are less than or equal to the specified
    interval spacing.

    Args:
        path (Path): The global path.
        pos (HelperLatLon): The current position of the vehicle.
        interval_spacing (float): The desired interval spacing.
        file_path (str): The path to the global path csv file.
        write (bool): Whether to overwrite the path with the formatted path.
    """

    # obtain the actual distances between every waypoint in the path
    path_spacing = calculate_interval_spacing(pos, path.waypoints)

    # check if global path is just a destination point
    if len(path.waypoints) < 2:
        path = generate_path(
            dest=path.waypoints[0],
            interval_spacing=interval_spacing,
            pos=pos,
            write=True,
            file_path=file_path,
        )
    # Check if any waypoints are too far apart
    elif max(path_spacing) > interval_spacing:
        path = interpolate_path(
            global_path=path,
            interval_spacing=interval_spacing,
            pos=pos,
            path_spacing=path_spacing,
            write=True,
            file_path=file_path,
        )

    return path


if __name__ == "__main__":
    main()
