from typing import List

import pytest
from custom_interfaces.msg import HelperLatLon, Path

from local_pathfinding.node_mock_global_path import MockGlobalPath as mgp


@pytest.mark.parametrize(
    "global_path_str_list,global_path",
    [
        (["49.263,123.138"], Path(waypoints=[HelperLatLon(latitude=49.263, longitude=123.138)])),
        (
            ["49.263,123.138", "49.263,123.138"],
            Path(
                waypoints=[
                    HelperLatLon(latitude=49.263, longitude=123.138),
                    HelperLatLon(latitude=49.263, longitude=123.138),
                ]
            ),
        ),
        (
            ["49.263,123.138", "49.263,123.138", "49.263,123.138"],
            Path(
                waypoints=[
                    HelperLatLon(latitude=49.263, longitude=123.138),
                    HelperLatLon(latitude=49.263, longitude=123.138),
                    HelperLatLon(latitude=49.263, longitude=123.138),
                ]
            ),
        ),
    ],
)
def test_str_list_to_path(global_path_str_list: List[str], global_path: Path):
    global_path_converted = mgp.str_list_to_path(global_path_str_list)

    for i in range(len(global_path_converted.waypoints) - 1):
        assert global_path_converted.waypoints[i].latitude == pytest.approx(
            global_path.waypoints[i].latitude
        ), "incorrect path conversion"
        assert global_path_converted.waypoints[i].longitude == pytest.approx(
            global_path.waypoints[i].longitude
        ), "incorrect path conversion"
