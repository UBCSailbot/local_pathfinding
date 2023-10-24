import numpy as np
import pytest
from custom_interfaces.msg import (
    HelperAISShip,
    HelperDimension,
    HelperHeading,
    HelperLatLon,
    HelperROT,
    HelperSpeed,
)
from shapely.geometry import Polygon

from local_pathfinding.coord_systems import XY, LatLon, latlon_to_xy
from local_pathfinding.obstacles import Boat, Obstacle


@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed,invalid_point,valid_point",
    [
        (
            LatLon(52.268119490007756, -136.9133983613776),
            LatLon(51.95785651405779, -136.26282894969611),
            HelperAISShip(
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=0.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0.0),
            ),
            15.0,
            latlon_to_xy(
                LatLon(52.268119490007756, -136.9133983613776),
                LatLon(52.174842845359755, -137.10372451905042),
            ),
            latlon_to_xy(
                LatLon(52.268119490007756, -136.9133983613776),
                LatLon(49.30499213908291, -123.31330140816111),
            ),
        )
    ],
)
def test_is_valid(
    reference_point: LatLon,
    sailbot_position: LatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
    invalid_point: XY,
    valid_point: XY,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)
    assert not boat1.is_valid(invalid_point)
    assert boat1.is_valid(valid_point)


@pytest.mark.parametrize(
    "reference_point,sailbot_position,sailbot_speed,invalid_point,valid_point",
    [
        (
            LatLon(52.268119490007756, -136.9133983613776),
            LatLon(51.95785651405779, -136.26282894969611),
            15.0,
            latlon_to_xy(
                LatLon(52.268119490007756, -136.9133983613776),
                LatLon(52.174842845359755, -137.10372451905042),
            ),
            latlon_to_xy(
                LatLon(52.268119490007756, -136.9133983613776),
                LatLon(49.30499213908291, -123.31330140816111),
            ),
        )
    ],
)
def test_is_valid_no_collision_zone(
    reference_point: LatLon,
    sailbot_position: LatLon,
    sailbot_speed: float,
    invalid_point: XY,
    valid_point: XY,
):
    # If you only create an obstacle, then a collision zone is not created
    # is_valid() should raise a ValueError
    obstacle = Obstacle(reference_point, sailbot_position, sailbot_speed)

    with pytest.raises(ValueError):
        obstacle.is_valid(invalid_point)
    with pytest.raises(ValueError):
        obstacle.is_valid(valid_point)


@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            LatLon(52.268119490007756, -136.9133983613776),
            LatLon(51.95785651405779, -136.26282894969611),
            HelperAISShip(
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0.0),
            ),
            15.0,
        )
    ],
)
def test_create_collision_cone(
    reference_point: LatLon,
    sailbot_position: LatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)
    boat1.collision_zone = boat1.create_collision_zone(ais_ship)
    assert boat1.collision_zone is Polygon
    if boat1.collision_zone is not None:
        assert boat1.collision_zone.exterior.coords is not None


if __name__ == "__main__":
    """VISUAL TESTS

    The projected distance length was verified visually, using the plotly chart and this graph:
    https://www.desmos.com/calculator/py6socdmne

    According to calculations, the cone should be about 47km long + a 0.5km buffer on both ends
    which is what can be observed in the plotly chart.

    Modifying the COG also modifies the collision cone's orientation as expected.
    """
    import plotly.graph_objects as go

    # Create a boat object
    boat1 = Boat(
        LatLon(52.268119490007756, -136.9133983613776),
        LatLon(51.95785651405779, -136.26282894969611),
        30.0,
        HelperAISShip(
            lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
            cog=HelperHeading(heading=30.0),
            sog=HelperSpeed(speed=18.52),
            width=HelperDimension(dimension=20.0),
            length=HelperDimension(dimension=100.0),
            rot=HelperROT(rot=0.0),
        ),
    )

    # Choose some states for visual inspection
    valid_state = LatLon(50.42973337261916, -134.12018940923838)
    invalid_state = LatLon(52.174842845359755, -137.10372451905042)

    # Extract coordinates for sailbot
    sailbot_x, sailbot_y = boat1.sailbot_position
    sailbot = go.Scatter(x=[sailbot_x], y=[sailbot_y], mode="markers", name="Sailbot Position")

    # Extract coordinates for valid and invalid states
    valid_state_x, valid_state_y = latlon_to_xy(boat1.reference, valid_state)
    valid_state = go.Scatter(
        x=[valid_state_x], y=[valid_state_y], mode="markers", name="Valid State"
    )

    invalid_state_x, invalid_state_y = latlon_to_xy(boat1.reference, invalid_state)
    invalid_state = go.Scatter(
        x=[invalid_state_x], y=[invalid_state_y], mode="markers", name="Invalid State"
    )

    # Create a Plotly figure to represent the boat's collision cone for manual inspection
    fig1 = go.Figure(sailbot)
    fig1.add_trace(valid_state)
    fig1.add_trace(invalid_state)

    # Extract exterior coordinates for boat1's collision cone
    if boat1.collision_zone is not None:
        boat_x, boat_y = np.array(boat1.collision_zone.exterior.coords.xy)
        boat_x = np.array(boat_x)
        boat_y = np.array(boat_y)
        boat = go.Scatter(x=boat_x, y=boat_y, fill="toself", name="Boat Collision Cone")
        fig1.add_trace(boat)

    fig1.update_layout(yaxis_range=[-200, 200], xaxis_range=[-200, 750])

    fig1.show()
