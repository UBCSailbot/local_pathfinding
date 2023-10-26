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
from shapely.geometry import Point, Polygon

from local_pathfinding.coord_systems import XY, LatLon, latlon_to_xy, meters_to_km
from local_pathfinding.obstacles import COLLISION_ZONE_SAFETY_BUFFER, Boat, Obstacle


# Test is_valid
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


# Test is_valid raises error when collision zone has not been set
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
    obstacle = Obstacle(reference_point, sailbot_position, sailbot_speed)
    with pytest.raises(ValueError):
        obstacle.is_valid(invalid_point)
    with pytest.raises(ValueError):
        obstacle.is_valid(valid_point)


# Test collision zone is created successfully
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            LatLon(52.268119490007756, -136.9133983613776),
            LatLon(51.95785651405779, -136.26282894969611),
            HelperAISShip(
                id=1,
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
def test_create_collision_zone(
    reference_point: LatLon,
    sailbot_position: LatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)
    boat1.collision_zone = boat1.create_boat_collision_zone(ais_ship)

    assert isinstance(boat1.collision_zone, Polygon)
    if boat1.collision_zone is not None:
        assert boat1.collision_zone.exterior.coords is not None


# Test calculate projected distance
# Boat and Sailbot in same location
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            LatLon(52.268119490007756, -136.9133983613776),
            LatLon(51.957, -136.262),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.957, longitude=-136.262),
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
def test_calculate_projected_distance(
    reference_point: LatLon,
    sailbot_position: LatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)

    assert boat1.calculate_projected_distance(ais_ship) == pytest.approx(
        0.0
    ), "incorrect projected distance"


# Test calculate projected time
# Boat and Sailbot in same location
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            LatLon(52.268119490007756, -136.9133983613776),
            LatLon(51.957, -136.262),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.957, longitude=-136.262),
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
def test_calculate_time_intersection(
    reference_point: LatLon,
    sailbot_position: LatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)

    assert boat1.calculate_time_to_intersection(ais_ship) == pytest.approx(
        0.0
    ), "incorrect intersection time"


# Test update collision zone raises error when id of passed ais_ship does not match self's id
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship_1,ais_ship_2,sailbot_speed",
    [
        (
            LatLon(52.268119490007756, -136.9133983613776),
            LatLon(51.95785651405779, -136.26282894969611),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0.0),
            ),
            HelperAISShip(
                id=2,
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
def test_update_collision_zone_id_mismatch(
    reference_point: LatLon,
    sailbot_position: LatLon,
    ais_ship_1: HelperAISShip,
    ais_ship_2: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship_1)

    with pytest.raises(ValueError):
        boat1.create_boat_collision_zone(ais_ship_2)


# TODO add unit tests for helper functions like proj dist an proj time

if __name__ == "__main__":
    """VISUAL TESTS

    The collision zone length can be verified visually, using the plotly chart below.
    """
    import plotly.graph_objects as go
    from numpy import ndarray

    # Sample AIS SHIP message
    ais_ship = HelperAISShip(
        id=1,
        lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
        cog=HelperHeading(heading=0.0),
        sog=HelperSpeed(speed=18.52),
        width=HelperDimension(dimension=20.0),
        length=HelperDimension(dimension=100.0),
        rot=HelperROT(rot=0.0),
    )

    # Create a boat object
    boat1 = Boat(
        LatLon(52.268119490007756, -136.9133983613776),
        LatLon(51.95785651405779, -136.26282894969611),
        30.0,
        ais_ship,
    )

    # Choose some states for visual inspection
    valid_state = LatLon(50.42973337261916, -134.12018940923838)
    invalid_state = LatLon(52.174842845359755, -137.10372451905042)

    # Extract coordinates for sailbot
    sailbot_x, sailbot_y = boat1.sailbot_position
    sailbot = go.Scatter(x=[sailbot_x], y=[sailbot_y], mode="markers", name="Sailbot Position")

    fig1 = go.Figure(sailbot)

    # Extract coordinates for valid and invalid states
    valid_state_x, valid_state_y = latlon_to_xy(boat1.reference, valid_state)
    valid_state = go.Scatter(
        x=[valid_state_x], y=[valid_state_y], mode="markers", name="Valid State"
    )

    fig1.add_trace(valid_state)

    invalid_state_x, invalid_state_y = latlon_to_xy(boat1.reference, invalid_state)
    invalid_state = go.Scatter(
        x=[invalid_state_x], y=[invalid_state_y], mode="markers", name="Invalid State"
    )

    fig1.add_trace(invalid_state)

    # Extract exterior coordinates for boat1's collision cone
    if boat1.collision_zone is not None:
        boat_x, boat_y = np.array(boat1.collision_zone.exterior.coords.xy)
        boat_x = np.array(boat_x)
        boat_y = np.array(boat_y)
        boat = go.Scatter(x=boat_x, y=boat_y, fill="toself", name="Boat Collision Cone")
        fig1.add_trace(boat)

    # Manually calculate the length of the collision zone based on:
    # - the boat's projected distance
    # - the boat's length
    # - the safety buffer
    collision_zone_length = round(
        (
            boat1.calculate_projected_distance(ais_ship)
            + 2 * COLLISION_ZONE_SAFETY_BUFFER
            + meters_to_km(boat1.length)
        ),
        4,
    )

    fig1.add_annotation(
        text="Calculated Length of Collision Zone : " + str(collision_zone_length) + " km",
        align="center",
        showarrow=False,
        xref="paper",
        yref="paper",
        x=1,
        y=1,
        bordercolor="black",
        borderwidth=1,
    )

    # Measure the length of the collision zone based on the points of the polygon
    x: ndarray
    y: ndarray

    if boat1.collision_zone is not None:
        x, y = boat1.collision_zone.exterior.coords.xy

    mid_1 = Point((x[1] + x[2]) / 2, (y[1] + y[2]) / 2)
    mid_2 = Point((x[0] + x[3]) / 2, (y[0] + y[3]) / 2)

    length = round(mid_1.distance(mid_2), 4)

    fig1.add_annotation(
        text="Measured Length of Collision Zone : " + str(length) + " km",
        align="center",
        showarrow=False,
        xref="paper",
        yref="paper",
        x=0.7,
        y=1,
        bordercolor="black",
        borderwidth=1,
    )

    fig1.update_layout(yaxis_range=[-200, 200], xaxis_range=[-200, 750])
    fig1.show()
