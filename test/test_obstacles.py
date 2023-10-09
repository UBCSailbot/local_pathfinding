import numpy as np
import plotly.graph_objects as go

from local_pathfinding.coord_systems import XY
from local_pathfinding.obstacles import Boat

# from shapely.geometry import Point


position = XY(0, 0)

boat1 = Boat(999, 26, 52, position, 30, 10, 70)

# Extract exterior coordinates for boat1's collision cone
x = np.array(boat1.collision_cone.exterior.coords.xy[0])
y = np.array(boat1.collision_cone.exterior.coords.xy[1])

# Create a Plotly figure for boat1's collision cone with a hole representing the hull
fig1 = go.Figure(go.Scatter(x=x, y=y, fill="toself"))

fig1.update_layout(yaxis_range=[-200, 200], xaxis_range=[-200, 750])

# Show the figures
fig1.show()
