from typing import Union

import pytest
from custom_interfaces.msg import HelperLatLon, Path

from local_pathfinding.coord_systems import GEODESIC, meters_to_km
from local_pathfinding.node_mock_global_path import MockGlobalPath


@pytest.mark.parametrize(
    "pos,waypoints",
    [
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            [
                HelperLatLon(latitude=48.95, longitude=123.55),
                HelperLatLon(latitude=85.95, longitude=13.56),
            ],
        ),
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            [
                HelperLatLon(latitude=48.95, longitude=123.55),
                HelperLatLon(latitude=85.95, longitude=13.56),
                HelperLatLon(latitude=85.00, longitude=13.00),
            ],
        ),
    ],
)
def test_interval_spacing(pos: HelperLatLon, waypoints: list[HelperLatLon]):
    """Test the greatest_interval method of MockGlobalPath.

    Args:
        pos (HelperLatLon): The start position.
        waypoints (list[HelperLatLon]): The waypoints of the global path.
    """
    greatest_interval = max(MockGlobalPath.interval_spacing(pos, waypoints))
    expected_interval = meters_to_km(
        GEODESIC.inv(
            lats1=waypoints[0].latitude,
            lons1=waypoints[0].longitude,
            lats2=waypoints[1].latitude,
            lons2=waypoints[1].longitude,
        )[2]
    )

    assert greatest_interval == pytest.approx(
        expected_interval
    ), "Greatest interval is not correct"


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
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            [
                HelperLatLon(latitude=48.95, longitude=123.55),
                HelperLatLon(latitude=48.95, longitude=123.54),
            ],
            10.0,
        ),
    ],
)
def test_generate_path(
    pos: HelperLatLon,
    dest: Union[HelperLatLon, list[HelperLatLon]],
    interval_spacing: float,
):
    """Test the generate_path method of MockGlobalPath.

    Args:
        dest (HelperLatLon): The destination of the global path.
        pos (HelperLatLon): The position of Sailbot.
        interval_spacing (float): The desired spacing between waypoints.
    """
    global_path = MockGlobalPath.generate_path(
        dest=dest,
        interval_spacing=interval_spacing,
        pos=pos,
    )

    assert isinstance(global_path, Path)

    if isinstance(dest, list):
        assert global_path.waypoints[-1].latitude == pytest.approx(
            dest[-1].latitude
        ), "final waypoint latitude is not correct"
        assert global_path.waypoints[-1].longitude == pytest.approx(
            dest[-1].longitude
        ), "final waypoint longitude is not correct"
    else:
        assert global_path.waypoints[-1].latitude == pytest.approx(
            expected=dest.latitude
        ), "final waypoint latitude is not correct"
        assert global_path.waypoints[-1].longitude == pytest.approx(
            expected=dest.longitude
        ), "final waypoint longitude is not correct"

    # Ensure proper spacing between waypoints
    for i in range(1, len(global_path.waypoints)):
        dist = GEODESIC.inv(
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
