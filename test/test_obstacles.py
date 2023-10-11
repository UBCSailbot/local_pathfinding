import numpy as np
import plotly.graph_objects as go
from shapely.geometry import Point

from local_pathfinding.coord_systems import XY, LatLon
from local_pathfinding.obstacles import Boat

# from shapely.geometry import Point


position = LatLon(49.261, -123.248)
reference = LatLon(49.742, -123.341)

boat1 = Boat(999, 26, 52, position, reference, 30, 10, 70)

# Extract exterior coordinates for boat1's collision cone
x = np.array(boat1.collision_cone.exterior.coords.xy[0])
y = np.array(boat1.collision_cone.exterior.coords.xy[1])

# Create a Plotly figure for boat1's collision cone with a hole representing the hull
fig1 = go.Figure(go.Scatter(x=x, y=y, fill="toself"))

fig1.update_layout(yaxis_range=[-200, 200], xaxis_range=[-200, 750])

# check the is_valid function
# assert not boat1.is_valid(Point(100, 50))
assert boat1.is_valid(Point(100, 100))

# Show the figures
fig1.show()
