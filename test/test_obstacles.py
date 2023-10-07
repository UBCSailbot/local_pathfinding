import numpy as np
import plotly.graph_objects as go
from shapely.geometry import Point

from local_pathfinding.obstacles import Boat

boat1 = Boat(999, 26, 52, (0, 0), 30, 10, 0)

# Extract exterior coordinates for boat1's collision cone
x1_exterior = boat1.collision_cone.exterior.xy[0]
y1_exterior = boat1.collision_cone.exterior.xy[1]

# Extract interior coordinates for boat1's hull
x1_hole = boat1.collision_cone.interiors[0].xy[0]
y1_hole = boat1.collision_cone.interiors[0].xy[1]

# Create a list of coordinates for the filled polygon
x1_filled = np.array(x1_exterior + x1_hole[::-1])
y1_filled = np.array(y1_exterior + y1_hole[::-1])

# Create a Plotly figure for boat1's collision cone with a hole representing the hull
fig1 = go.Figure(go.Scatter(x=x1_filled, y=y1_filled, fill="toself"))

print(boat1.is_valid(Point(1000, 0)))

# Show the figures
fig1.show()
