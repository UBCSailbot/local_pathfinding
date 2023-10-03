import pytest

import local_pathfinding.coord_systems as coord_systems
from local_pathfinding.coord_systems import XY, LatLon


@pytest.mark.parametrize(
    "cartesian,true_bearing",
    [
        (0.0, 90.0),
        (90.0, 0.0),
        (180.0, 270.0),
        (270.0, 180.0),
    ],
)
def test_cartesian_to_true_bearing(cartesian: float, true_bearing: float):
    assert coord_systems.cartesian_to_true_bearing(cartesian) == pytest.approx(
        true_bearing
    ), "incorrect angle conversion"


@pytest.mark.parametrize(
    "reference,latlon,xy",
    [
        (
            LatLon(latitude=30.0, longitude=-123.0),
            LatLon(latitude=30.270624453997037, longitude=-123.0),
            XY(x=0.0, y=30.0),
        ),
        (
            LatLon(latitude=30.0, longitude=-123.0),
            LatLon(latitude=30.19117726329345, longitude=-122.77971962373852),
            XY(x=21.213203435597134, y=21.213203435596135),
        ),
        (
            LatLon(latitude=30.0, longitude=-123.0),
            LatLon(latitude=29.99963284667579, longitude=-122.68907572868501),
            XY(x=30.0, y=0.0),
        ),
        (
            LatLon(latitude=60.0, longitude=-123.0),
            LatLon(latitude=60.26926459981188, longitude=-123.0),
            XY(x=0.0, y=30.0),
        ),
        (
            LatLon(latitude=60.0, longitude=-123.0),
            LatLon(latitude=60.189849426556385, longitude=-122.61764080128867),
            XY(x=21.213203435596185, y=21.21320343559584),
        ),
        (
            LatLon(latitude=60.0, longitude=-123.0),
            LatLon(latitude=59.99890592957226, longitude=-122.46237744068965),
            XY(x=30.0, y=0.0),
        ),
    ],
)
def test_latlon_to_xy(reference: LatLon, latlon: LatLon, xy: XY):
    assert coord_systems.latlon_to_xy(reference, latlon) == pytest.approx(
        xy
    ), "incorrect coordinate conversion"


def get_latlon_to_xy_test_cases():
    test_cases = []
    start_lon = -123
    dist_km = 30
    for start_lat in [30, 60]:
        start = LatLon(latitude=start_lat, longitude=start_lon)
        for course_deg in [0, 45, 90]:
            dest_lon, dest_lat, _ = coord_systems.GEODESIC.fwd(
                lons=start_lon, lats=start_lat, az=course_deg, dist=dist_km * 1000
            )
            dest = LatLon(latitude=dest_lat, longitude=dest_lon)
            xy = coord_systems.latlon_to_xy(start, dest)
            test_cases.append((start, dest, xy))
    return test_cases


if __name__ == "__main__":
    print("latlon_to_xy() test cases:", get_latlon_to_xy_test_cases(), sep="\n")
