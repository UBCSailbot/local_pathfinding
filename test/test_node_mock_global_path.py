import pytest
import rclpy
from custom_interfaces.msg import GPS, HelperLatLon, Path
from pyproj import Geod

from local_pathfinding.node_mock_global_path import MockGlobalPath

ELLIPSOID = Geod(ellps="WGS84")


@pytest.mark.parametrize(
    "pos,dest",
    [
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            HelperLatLon(latitude=38.95, longitude=133.36),
        ),
        (
            HelperLatLon(latitude=-48.95, longitude=123.56),
            HelperLatLon(latitude=38.95, longitude=-133.36),
        ),
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            HelperLatLon(latitude=48.95, longitude=123.55),
        ),
    ],
)
def test_generate_path(dest: HelperLatLon, pos: HelperLatLon):
    """Test the generate_path method of MockGlobalPath.

    Args:
        dest (HelperLatLon): The destination of the global path.
        pos (HelperLatLon): The position of the robot.
        mgp_node (MockGlobalPath): The MockGlobalPath node.
    """
    rclpy.init(args=None)
    mgp_node = MockGlobalPath()
    mgp_node.gps = GPS(lat_lon=pos)
    mgp_node.generate_path(dest=dest)

    assert isinstance(mgp_node.global_path, Path)

    assert mgp_node.global_path.waypoints[-1].latitude == pytest.approx(
        expected=dest.latitude
    ), "final waypoint latitude is not correct"
    assert mgp_node.global_path.waypoints[-1].longitude == pytest.approx(
        expected=dest.longitude
    ), "final waypoint longitude is not correct"

    # Ensure proper spacing between waypoints
    for i in range(1, len(mgp_node.global_path.waypoints)):
        dist = ELLIPSOID.inv(
            mgp_node.global_path.waypoints[i - 1].longitude,
            mgp_node.global_path.waypoints[i - 1].latitude,
            mgp_node.global_path.waypoints[i].longitude,
            mgp_node.global_path.waypoints[i].latitude,
        )[2]
        dist *= 0.001  # convert to km
        assert dist <= 30.0, "Interval spacing is not correct"

    mgp_node.destroy_node()
    rclpy.shutdown()
