"""The Global Path Module, which retrieves the global path from a specified http source and
sends it to NET via POST request.

The main function accepts two CLI arguments:
    file_path (str): The path to the global path csv file.
    --interval (float, Optional): The desired path interval length in km.
"""

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
    _interpolate_path,
    calculate_interval_spacing,
    generate_path,
    path_to_dict,
)

GPS_URL = "http://localhost:3005/api/gps"
PATH_URL = "http://localhost:8081/global-path"
GLOBAL_PATHS_FILE_PATH = "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file_path", help="The path to the global path csv file.")
    parser.add_argument("--interval", help="Desired path interval length.", type=float)
    args = parser.parse_args()

    file_path = args.file_path
    path_mod_tmstmp = None
    pos = None

    try:
        path = get_path(file_path)
        print(f"retrieved path from {file_path}", path_to_dict(path))
    except FileNotFoundError:
        print(f"{file_path} not found. Please enter a valid file path.")
        exit(1)

    # Main service loop
    while True:
        timestamp = time.ctime(os.path.getmtime(file_path))

        # We should try to retrieve the position on every loop
        pos = get_pos()

        if pos is None:
            print(f"Failed to retrieve position from {GPS_URL}")
            continue

        position_delta = meters_to_km(
            GEODESIC.inv(
                lats1=pos.latitude,
                lons1=pos.longitude,
                lats2=path.waypoints[0].latitude,
                lons2=path.waypoints[0].longitude,
            )[2]
        )

        # exit loop if the path has not been modified or interval lengths are fine
        if (timestamp == path_mod_tmstmp) and (
            (args.interval is None) or position_delta <= args.interval
        ):
            continue

        if args.interval is not None:
            # interpolate path will interpolate new path and save it to a new csv file
            path = interpolate_path(
                path=path,
                pos=pos,
                interval_spacing=args.interval,
                file_path=file_path,
            )

        if post_path(path):
            print("Global path successfully updated.")
            file_path = get_most_recent_file(GLOBAL_PATHS_FILE_PATH)
        else:
            # if the post was unsuccessful, we should try again
            # so don't update the timestamp
            continue

        path_mod_tmstmp = timestamp

        # An alternate option is to have the functions generate_path and interpolate_path return
        # a tuple containing the new path and the file path of the new csv file
        # but this would be a breaking change to multiple files, so this is a quick workaround

        # we don't need to update multiple times per second
        time.sleep(1)


def get_most_recent_file(directory_path: str) -> str:
    """
    Returns the most recently modified file in the specified directory.

    Args:
        directory_path (str): The path to the directory containing the files.

    Returns:
        str: The path to the most recently modified file.
    """
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

    Returns:
        Path: The global path retrieved from the csv file.
    """
    path = Path()

    with open(file_path, "r") as file:
        reader = csv.reader(file)
        # skip header
        reader.__next__()
        for row in reader:
            path.waypoints.append(HelperLatLon(latitude=float(row[0]), longitude=float(row[1])))
    return path


def post_path(path: Path) -> bool:
    """Sends the global path to NET via POST request.

    Args:
        path (Path): The global path.

    Returns:
        bool: Whether or not the global path was successfully posted.
    """
    waypoints = [
        {"lat": float(item.latitude), "lon": float(item.longitude)} for item in path.waypoints
    ]
    data = {"waypoints": waypoints}
    json_data = json.dumps(data).encode("utf-8")
    try:
        urlopen(PATH_URL, json_data)
        return True
    except HTTPError as http_error:
        print(f"HTTP Error: {http_error.code}")
    except URLError as url_error:
        print(f"URL Error: {url_error.reason}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    return False


def get_pos() -> HelperLatLon:
    """Returns the current position of sailbot, retrieved from the an http GET request.

    Returns:
        HelperLatLon: The current position of sailbot.
    """
    try:
        position = json.loads(urlopen("http://localhost:3005/api/gps").read())
    except HTTPError as http_error:
        print(f"HTTP Error: {http_error.code}")
    except URLError as url_error:
        print(f"URL Error: {url_error.reason}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    if len(position["data"]) == 0:
        print("No position data available.")
        return None

    latitude = position["data"][-1]["latitude"]
    longitude = position["data"][-1]["longitude"]
    pos = HelperLatLon(latitude=latitude, longitude=longitude)

    return pos


def interpolate_path(
    path: Path,
    pos: HelperLatLon,
    interval_spacing: float,
    file_path: str,
    write=True,
) -> Path:
    """Interpolates path to ensure the interval lengths are less than or equal to the specified
    interval spacing.

    Args:
        path (Path): The global path.
        pos (HelperLatLon): The current position of the vehicle.
        interval_spacing (float): The desired interval spacing.
        file_path (str): The path to the global path csv file.
        write (bool, optional): Whether or not to write the new path to a csv file. Default True.

    Returns:
        Path: The interpolated path.
    """

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
        path = _interpolate_path(
            global_path=path,
            interval_spacing=interval_spacing,
            pos=pos,
            path_spacing=path_spacing,
            write=write,
            file_path=file_path,
        )

    return path


if __name__ == "__main__":
    main()
