from typing import List

import pytest
from custom_interfaces.msg import HelperLatLon, Path

from local_pathfinding.node_mock_global_path import MockGlobalPath as mgp


@pytest.mark.parametrize(
    "global_path_str_list,global_path",
    [
        (["49.263,123.138"], Path(lat_lon=[HelperLatLon(latitude=49.263, longitude=123.138)])),
        (
            ["49.263,123.138", "49.263,123.138"],
            Path(
                lat_lon=[
                    HelperLatLon(latitude=49.263, longitude=123.138),
                    HelperLatLon(latitude=49.263, longitude=123.138),
                ]
            ),
        ),
        (
            ["49.263,123.138", "49.263,123.138", "49.263,123.138"],
            Path(
                lat_lon=[
                    HelperLatLon(latitude=49.263, longitude=123.138),
                    HelperLatLon(latitude=49.263, longitude=123.138),
                    HelperLatLon(latitude=49.263, longitude=123.138),
                ]
            ),
        ),
    ],
)
def test_str_list_to_path(global_path_str_list: List[str], global_path: Path):
    assert mgp.str_list_to_path(global_path_str_list) == pytest.approx(
        global_path
    ), "incorrect path conversion"
