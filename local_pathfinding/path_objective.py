import math
import sys

from ompl import base as ob
from ompl import util as ou


class Distanceobjective:
    def __init__(self, space_information):
        self.si = space_information

    def get_path_length_objective(self):
        path_objective = ob.PathLengthOptimizationObjective(self.si)

        return path_objective

    def get_euclidean_path_length_objective(self, state):
        start_x, start_y = state.start_state
        goal_x, goal_y = state.goal_state

        s1 = (start_x, start_y)
        s2 = (goal_x, goal_y)

        return ((goal_y - start_y)**2 + (goal_x - start_x)**2)**(0.5)

    def get_latlon_path_length_objective(self, state):
        """
        I am assuming that we are using the lat and long coordinates in determining the distance
        between two points.
        """

        start_lat, start_lon = state.start_state
        goal_lat, goal_lon = state.goal_state

        # Example lat and lon coordinates
        # start_lat, start_lon = -66.18541, -113.62386
        # goal_lat, goal_lon =  -66.18436, -113.62286

        s1 = (start_lat, start_lon)
        s2 = (goal_lat, goal_lon)

        return math.acos(math.sin(start_lat)*math.sin(goal_lat) + math.cos(start_lat)*math.cos(goal_lat)*math.cos(start_lon - goal_lon)) * 6371


