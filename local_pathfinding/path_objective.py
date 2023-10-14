import math
import sys

from ompl import base as ob
from ompl import util as ou

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0

# Upwind downwind constants
UPWIND_MAX_ANGLE_DEGREES = 40.0
DOWNWIND_MAX_ANGLE_DEGREES = 20.0


class Distanceobjective(ob.StateCostIntegralObjective):
    def __init__(self, space_information):
        super(Distanceobjective, self).__init__(space_information, True)
        self.space_information = space_information

    def motionCost(self, s1, s2):
        return 0

    def get_path_length_objective(self):
        return NotImplementedError

    def get_euclidean_path_length_objective(self, s1, s2):
        return NotImplementedError

    def get_latlon_path_length_objective(self, s1, s2):
        return NotImplementedError


class MinimumTurningObjective(ob.StateCostIntegralObjective):
    def __init__(self, space_information, simple_setup, heading_degrees):
        super(MinimumTurningObjective, self).__init__(space_information, True)
        self.space_information = space_information
        self.simple_setup = simple_setup
        self.heading_degrees = heading_degrees

    def motionCost(self, s1, s2):
        return 0

    def goalPathTurnCost(self, s1, s2):
        return NotImplementedError

    def goalHeadingTurnCost(self, s1):
        return NotImplementedError

    def headingPathTurnCost(self, s1, s2):
        return NotImplementedError


class WindObjective(ob.StateCostIntegralObjective):
    def __init__(self, space_information, wind_direction_degrees):
        super(WindObjective, self).__init__(space_information, True)
        self.space_information = space_information
        self.wind_direction_degrees = wind_direction_degrees

    # This objective function punishes the boat for going up/downwind
    def motionCost(self, s1, s2):
        return 0


def isUpwind(wind_direction_radians, boat_direction_radians):
    return NotImplementedError


def isDownwind(wind_direction_radians, boat_direction_radians):
    return NotImplementedError


def allocate_objective(space_information, simple_setup, heading_degrees, wind_direction_degrees):
    objective = ob.MultiOptimizationObjective(space_information)
    objective.addObjective(Distanceobjective(space_information), 1.0)
    objective.addObjective(
        MinimumTurningObjective(space_information, simple_setup, heading_degrees), 100.0
    )
    objective.addObjective(WindObjective(space_information, wind_direction_degrees), 1.0)

    return objective
