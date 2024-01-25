import get_server
import post_server
import pytest
from custom_interfaces.msg import HelperLatLon, Path

from local_pathfinding.global_path import get_path, get_pos, interpolate_path, post_path
from local_pathfinding.node_mock_global_path import calculate_interval_spacing


# ------------------------- TEST GET_PATH -------------------------
@pytest.mark.parametrize(
    "file_path",
    [("/workspaces/sailbot_workspace/src/local_pathfinding/global_paths/mock_global_path.csv")],
)
def test_get_path(file_path: str):
    """ "
    Args:
        file_path (str): The path to the global path csv file.
    """
    global_path = get_path(file_path)

    assert isinstance(global_path, Path)

    # Check that the path is formatted correctly
    for waypoint in global_path.waypoints:
        assert isinstance(waypoint, HelperLatLon), "Waypoint is not a HelperLatLon"
        assert isinstance(waypoint.latitude, float), "Waypoint latitude is not a float"
        assert isinstance(waypoint.longitude, float), "Waypoint longitude is not a float"


# ------------------------- TEST POST_PATH -------------------------
@pytest.mark.parametrize(
    "global_path",
    [
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=48.95, longitude=123.56),
                    HelperLatLon(latitude=38.95, longitude=133.36),
                    HelperLatLon(latitude=28.95, longitude=143.36),
                ]
            )
        )
    ],
)
def test_post_path(global_path: Path):
    """
    Args:
        global_path (Path): The global path to post.
    """

    # Launch http server
    server = post_server.run_server()

    assert post_path(global_path), "Failed to post global path"

    post_server.shutdown_server(httpd=server)


# ------------------------- TEST GET_POS -------------------------
@pytest.mark.parametrize(
    "pos", [HelperLatLon(latitude=49.34175775635472, longitude=-123.35453636335373)]
)
def test_get_pos(pos: HelperLatLon):
    """
    Args:
        pos (HelperLatLon): The position of the Sailbot.
    """

    # Launch http server
    server = get_server.run_server()

    assert get_pos() == pos, "Position is not correct"

    get_server.shutdown_server(httpd=server)


# ------------------------- TEST INTERPOLATE_PATH -------------------------
@pytest.mark.parametrize(
    "path,pos,interval_spacing",
    [
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=48.95, longitude=123.56),
                    HelperLatLon(latitude=38.95, longitude=133.36),
                    HelperLatLon(latitude=28.95, longitude=143.36),
                ]
            ),
            HelperLatLon(latitude=58.95, longitude=113.56),
            50.0,
        )
    ],
)
def test_interpolate_path(path: Path, pos: HelperLatLon, interval_spacing: float):
    """
    Args:
        path (Path): The global path.
        pos (HelperLatLon): The position of the Sailbot.
        interval_spacing (float): The spacing between each waypoint.
    """
    formatted_path = interpolate_path(
        path=path, pos=pos, interval_spacing=interval_spacing, file_path="", write=False
    )

    assert isinstance(formatted_path, Path), "Formatted path is not a Path"

    # Check that the path is formatted correctly
    for waypoint in formatted_path.waypoints:
        assert isinstance(waypoint, HelperLatLon), "Waypoint is not a HelperLatLon"
        assert isinstance(waypoint.latitude, float), "Waypoint latitude is not a float"
        assert isinstance(waypoint.longitude, float), "Waypoint longitude is not a float"

    path_spacing = calculate_interval_spacing(pos, formatted_path.waypoints)
    assert max(path_spacing) <= interval_spacing, "Path spacing is too large"
    assert max(path_spacing) <= interval_spacing, "Path spacing is too large"
