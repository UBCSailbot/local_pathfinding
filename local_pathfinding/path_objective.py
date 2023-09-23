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
        self.si = space_information

    def get_path_length_objective(self):
        path_objective = ob.PathLengthOptimizationObjective(self.si)

        return path_objective

    def get_euclidean_path_length_objective(self, state):
        start_x, start_y = state.start_state
        goal_x, goal_y = state.goal_state

        s1 = (start_x, start_y)
        s2 = (goal_x, goal_y)

        return ((goal_y - start_y) ** 2 + (goal_x - start_x) ** 2) ** (0.5)

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

        return (
            math.acos(
                math.sin(start_lat) * math.sin(goal_lat)
                + math.cos(start_lat) * math.cos(goal_lat) * math.cos(start_lon - goal_lon)
            )
            * 6371
        )


class HeadingObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, initial_heading_degrees, heading_degrees):
        super(HeadingObjective, self).__init__(si, True)
        self.si = si
        self.last_direction_radians = math.radians(heading_degrees)
        self.initial_direction_radians = math.radians(initial_heading_degrees)

    def desiredHeadingTurnCost(self, s1, s2):

        direction_radians = math.atan2(s2.getY() - s1.getY(), s2.getX() - s1.getX())

        # Calculate turn size
        turn_size_radians = abs(direction_radians - self.last_direction_radians)

        large_turn_threshold = math.radians(2 * max(40, 20))
        if turn_size_radians > large_turn_threshold:
            return 500 * turn_size_radians
        else:
            return 10 * turn_size_radians

    def initialHeadingTurnCost(self, s1, s2):

        direction_radians = math.atan2(s2.getY() - s1.getY(), s2.getX() - s1.getX())

        # Calculate turn size
        turn_size_radians = abs_angle_dist_radians(
            self.initial_direction_radians, direction_radians
        )

        large_turn_threshold = math.radians(2 * max(40, 20))
        if turn_size_radians > large_turn_threshold:
            return 500 * turn_size_radians
        else:
            return 10 * turn_size_radians


class WindObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, windDirectionDegrees):
        super(WindObjective, self).__init__(si, True)
        self.si_ = si
        self.windDirectionDegrees = windDirectionDegrees

    # This objective function punishes the boat for going up/downwind
    def motionCost(self, s1, s2):
        distance = ((s2.getY() - s1.getY()) ** 2 + (s2.getX() - s2.getY()) ** 2) ** 0.5
        boatDirectionRadians = math.atan2(s2.getY() - s1.getY(), s2.getX() - s1.getX())

        if isUpwind(math.radians(self.windDirectionDegrees), boatDirectionRadians):
            return UPWIND_MULTIPLIER * distance
        elif isDownwind(math.radians(self.windDirectionDegrees), boatDirectionRadians):
            return DOWNWIND_MULTIPLIER * distance
        else:
            return 0.0


def isUpwind(windDirectionRadians, boatDirectionRadians):
    diffRadians = abs_angle_dist_radians(windDirectionRadians, boatDirectionRadians)
    return math.fabs(diffRadians - math.radians(180)) < math.radians(UPWIND_MAX_ANGLE_DEGREES)


def isDownwind(windDirectionRadians, boatDirectionRadians):
    diffRadians = abs_angle_dist_radians(windDirectionRadians, boatDirectionRadians)
    return math.fabs(diffRadians) < math.radians(DOWNWIND_MAX_ANGLE_DEGREES)


def abs_angle_dist_radians(a1, a2):
    """Computes the absolute difference between two angles in radians"""
    return abs((a1 - a2 + math.pi) % (2 * math.pi) - math.pi)


def allocate_objective(
    space_information, heading_degrees, initial_heading_degrees, windDirectionDegrees
):
    """Allocates a SO2 objective function with the given weight for each of the components of the objective function"""
    objective = ob.MultiOptimizationObjective(space_information)
    objective.addObjective(Distanceobjective(space_information), 1.0)
    objective.addObjective(
        HeadingObjective(space_information, heading_degrees=45, initial_heading_degrees=0), 100.0
    )
    objective.addObjective(WindObjective(space_information, windDirectionDegrees=8), 1.0)
    return objective
