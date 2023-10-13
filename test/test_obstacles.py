import numpy as np
import pytest
from custom_interfaces.msg import (
    HelperAISShip,
    HelperCOG,
    HelperDimensions,
    HelperLatLon,
    HelperSOG,
)

from local_pathfinding.coord_systems import LatLon
from local_pathfinding.obstacles import Boat


@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,state_point",
    [
        (
            LatLon(52.268119490007756, -136.9133983613776),
            LatLon(51.95785651405779, -136.26282894969611),
            HelperAISShip(
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperCOG(cog=0.0),
                sog=HelperSOG(sog=20.0),
                dimensions=HelperDimensions(length=100.0, width=20.0),
            ),
            LatLon(52.174842845359755, -137.10372451905042),
        )
    ],
)
def test_is_valid(
    reference_point: LatLon, sailbot_position: LatLon, ais_ship: HelperAISShip, state_point: LatLon
):
    boat1 = Boat(reference_point, sailbot_position, ais_ship)
    assert not boat1.is_valid(state_point)


boat1 = Boat(
    LatLon(52.268119490007756, -136.9133983613776),
    LatLon(51.95785651405779, -136.26282894969611),
    HelperAISShip(
        lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
        cog=HelperCOG(cog=0.0),
        sog=HelperSOG(sog=20.0),
        dimensions=HelperDimensions(length=100.0, width=20.0),
    ),
)

if __name__ == "__main__":
    import plotly.graph_objects as go

    # Extract exterior coordinates for boat1's collision cone
    boat_x, boat_y = np.array(boat1.collision_zone.exterior.coords.xy)
    boat_x = np.array(boat_x)
    boat_y = np.array(boat_y)
    boat = go.Scatter(x=boat_x, y=boat_y, fill="toself", name="Boat Collision Cone")

    # Extract coordinates for sailbot
    sailbot_x, sailbot_y = boat1.sailbot_position
    sailbot_x = np.array(sailbot_x)
    sailbot_y = np.array(sailbot_y)
    sailbot = go.Scatter(x=sailbot_x, y=sailbot_y, mode="markers", name="Sailbot Position")

    # Create a Plotly figure to represent the boat's collision cone for manual inspection
    fig1 = go.Figure(boat)
    fig1.add_trace(sailbot)

    fig1.update_layout(yaxis_range=[-200, 200], xaxis_range=[-200, 750])

    fig1.show()
