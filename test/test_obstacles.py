import numpy as np
import pytest
from custom_interfaces.msg import (
    HelperAISShip,
    HelperCOG,
    HelperDimensions,
    HelperLatLon,
    HelperROT,
    HelperSOG,
)

from local_pathfinding.coord_systems import LatLon, latlon_to_xy
from local_pathfinding.obstacles import Boat


@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed,state_point",
    [
        (
            LatLon(52.268119490007756, -136.9133983613776),
            LatLon(51.95785651405779, -136.26282894969611),
            HelperAISShip(
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperCOG(cog=0.0),
                sog=HelperSOG(sog=20.0),
                dimensions=HelperDimensions(length=100.0, width=20.0),
                rot=HelperROT(rot=0.0),
            ),
            15.0,
            LatLon(52.174842845359755, -137.10372451905042),
        )
    ],
)
def test_is_valid(
    reference_point: LatLon,
    sailbot_position: LatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
    state_point: LatLon,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)
    assert not boat1.is_valid(reference_point, state_point)
    assert boat1.is_valid(reference_point, LatLon(49.30499213908291, -123.31330140816111))


"""
VISUAL TESTS

The projected distance length was verified visually, using the plotly chart and this desmos graph:
https://www.desmos.com/calculator/4qepsjekrj

According to calculations, the cone should be about 47km long which is what can be observed
in the plotly chart.

Modifying the COG also modifies the collision cone's orientation as expected
"""


if __name__ == "__main__":
    import plotly.graph_objects as go

    # Create a boat object
    boat1 = Boat(
        LatLon(52.268119490007756, -136.9133983613776),
        LatLon(51.95785651405779, -136.26282894969611),
        30.0,
        HelperAISShip(
            lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
            cog=HelperCOG(cog=0.0),
            sog=HelperSOG(sog=10.0),
            dimensions=HelperDimensions(length=100.0, width=20.0),
            rot=HelperROT(rot=0.0),
        ),
    )

    # Choose some states for visual inspection
    valid_state = LatLon(50.42973337261916, -134.12018940923838)
    invalid_state = LatLon(52.174842845359755, -137.10372451905042)

    # Extract exterior coordinates for boat1's collision cone
    boat_x, boat_y = np.array(boat1.collision_zone.exterior.coords.xy)
    boat_x = np.array(boat_x)
    boat_y = np.array(boat_y)
    boat = go.Scatter(x=boat_x, y=boat_y, fill="toself", name="Boat Collision Cone")

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
    fig1 = go.Figure(boat)
    fig1.add_trace(sailbot)
    fig1.add_trace(valid_state)
    fig1.add_trace(invalid_state)

    fig1.update_layout(yaxis_range=[-200, 200], xaxis_range=[-200, 750])

    fig1.show()
