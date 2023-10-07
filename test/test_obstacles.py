import numpy as np
import plotly.graph_objects as go

from local_pathfinding.obstacles import Boat

boat1 = Boat(999, 5, 10, (0, 0), 1, 10, 0)
boat2 = Boat(888, 5, 10, (20, 25), 1, 11, 45)

# print(boat1.collision_cone)
# print(boat2.collision_cone)
x1 = np.array(boat1.collision_cone.exterior.coords.xy[0])
y1 = np.array(boat1.collision_cone.exterior.coords.xy[1])

x2 = np.array(boat2.collision_cone.exterior.coords.xy[0])
y2 = np.array(boat2.collision_cone.exterior.coords.xy[1])


go.Figure(go.Scatter(x=x1, y=y1, fill="toself")).show()
# go.Figure(go.Scatter(x=x2, y=y2, fill="toself")).show()
