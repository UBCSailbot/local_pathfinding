import pytest
from custom_interfaces.msg import HelperLatLon, Path
from pyproj import Geod

from local_pathfinding.node_mock_global_path import MockGlobalPath

ELLIPSOID = Geod(ellps="WGS84")


@pytest.mark.parametrize(
    "pos,dest,interval_spacing",
    [
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            HelperLatLon(latitude=38.95, longitude=133.36),
            30.0,
        ),
        (
            HelperLatLon(latitude=-48.95, longitude=123.56),
            HelperLatLon(latitude=38.95, longitude=-133.36),
            20.0,
        ),
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            HelperLatLon(latitude=48.95, longitude=123.55),
            5.0,
        ),
    ],
)
def test_generate_path(dest: HelperLatLon, pos: HelperLatLon, interval_spacing: float):
    """Test the generate_path method of MockGlobalPath.

    Args:
        dest (HelperLatLon): The destination of the global path.
        pos (HelperLatLon): The position of Sailbot.
        interval_spacing (float): The desired spacing between waypoints.
    """
    global_path = MockGlobalPath.generate_path(
        dest=dest, pos=pos, interval_spacing=interval_spacing
    )

    assert isinstance(global_path, Path)

    assert global_path.waypoints[-1].latitude == pytest.approx(
        expected=dest.latitude
    ), "final waypoint latitude is not correct"
    assert global_path.waypoints[-1].longitude == pytest.approx(
        expected=dest.longitude
    ), "final waypoint longitude is not correct"

    # Ensure proper spacing between waypoints
    for i in range(1, len(global_path.waypoints)):
        dist = ELLIPSOID.inv(
            global_path.waypoints[i - 1].longitude,
            global_path.waypoints[i - 1].latitude,
            global_path.waypoints[i].longitude,
            global_path.waypoints[i].latitude,
        )[2]
        dist *= 0.001  # convert to km
        assert dist <= interval_spacing, "Interval spacing is not correct"


@pytest.mark.parametrize(
    "path,expected",
    [
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=48.123446, longitude=123.123446),
                    HelperLatLon(latitude=38.123456, longitude=133.123456),
                ]
            ),
            {0: "(48.1234, 123.1234)", 1: "(38.1235, 133.1235)"},
        ),
    ],
)
def test_path_to_dict(path: Path, expected: dict[int, str]):
    path_dict = MockGlobalPath.path_to_dict(path)
    assert path_dict == expected, "Did not correctly convert path to dictionary"
