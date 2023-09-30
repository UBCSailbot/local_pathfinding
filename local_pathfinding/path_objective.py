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
    def __init__(self, si, ss, heading_degrees):
        super(HeadingObjective, self).__init__(si, True)
        self.si = si
        self.goal_x = ss.getGoal().getState().getX()
        self.goal_y = ss.getGoal().getState().getY()
        self.heading_degrees = heading_degrees

    def motionCost(self, s1, s2):
        # Calculate the mininum turning cost between s1-goal and heading
        turn_cost = self.goalHeadingTurnCost(s1)

        # Calculate the minimum turning cost between s1-s1 and s1-goal
        #   turn_cost = self.goalHeadingTurnCost(s1, s2)

        # Calculate the minimum turning cost from sl-s2 and heading
        #   turn_cost = self.headingPathTurnCost(s1, s2)

        return turn_cost

    def goalPathTurnCost(self, s1, s2):
        """Determine the smallest turn angle between s1-s2 and s1-goal

        Args:
            s1 (class): coordinates in the form (x, y) of the start state
            s2 (_type_): coordinates in the form (x, y) of the local goal state

        Returns:
            int: the turning cost from s2 to s1
        """
        large_turn_threshold = math.pi / 2

        # Calculate the angle of the s1-s2 line segment from North
        path_direction = math.atan2(s2.getY() - s1.getY(), s2.getX() - s1.getX())

        # Calculate the angle from s1-goal line segment from North
        global_goal_direction = math.atan2(self.goal_y - s1.getY(), self.goal_x - s1.getX())

        # Calculate the uncorrected turn size [0, 2pi]
        turn_size_bias = math.fabs(global_goal_direction - path_direction)

        # Correct the angle in between [0, pi]
        if turn_size_bias > math.pi:
            turn_size_unbias = turn_size_bias - math.pi
        else:
            turn_size_unbias = turn_size_bias

        if turn_size_unbias > large_turn_threshold:
            return 500 * turn_size_unbias
        else:
            return 10 * turn_size_unbias

    def goalHeadingTurnCost(self, s1):
        """Determine the smallest turn angle between s1-s2 and heading

        Args:
            s1 (class): coordinates in the form (x, y) of the start state
            s2 (_type_): coordinates in the form (x, y) of the local goal state

        Returns:
            int: the turning cost from s2 to s1
        """
        large_turn_threshold = math.pi / 2

        # Calculate the angle from s1-goal line segment from North
        global_goal_direction = math.atan2(self.goal_y - s1.getY(), self.goal_x - s1.getX())

        # Calculate the uncorrected turn size [0, 2pi]
        turn_size_bias = math.fabs(global_goal_direction - self.heading_degrees)

        # Correct the angle in between [0, pi]
        if turn_size_bias > math.pi:
            turn_size_unbias = turn_size_bias - math.pi
        else:
            turn_size_unbias = turn_size_bias

        if turn_size_unbias > large_turn_threshold:
            return 500 * turn_size_unbias
        else:
            return 10 * turn_size_unbias

    def headingPathTurnCost(self, s1, s2):
        """Generates the turning cost between s1-s2 and heading of the sailbot

        Args:
            s1 (class): The coordinates of the initial state in the form (x, y)
            s2 (_type_): The coordinates of the local goal state in the form (x, y)

        Returns:
            int: The minimum turning cost between s1-s2 and heading
        """
        large_turn_threshold = math.pi / 2

        # Calculate the angle of the s1-s2 line segment from North
        path_direction = math.atan2(s2.getY() - s1.getY(), s2.getX() - s1.getX())

        # Calculate turn size
        turn_size_bias = math.fabs(path_direction - self.heading_degrees)

        # Correct the angle in between [0, pi]
        if turn_size_bias > math.pi:
            turn_size_unbias = turn_size_bias - math.pi
        else:
            turn_size_unbias = turn_size_bias

        if turn_size_bias > large_turn_threshold:
            return 500 * turn_size_unbias
        else:
            return 10 * turn_size_unbias


class WindObjective(ob.StateCostIntegralObjective):
    def __init__(self, si, windDirectionDegrees):
        super(WindObjective, self).__init__(si, True)
        self.si_ = si
        self.windDirectionDegrees = windDirectionDegrees

    # This objective function punishes the boat for going up/downwind
    def motionCost(self, s1, s2):
        """
        1. Convert the measured wind direction to radians
        2. Convert the measured wind direction to true wind direction
        3. Check if the angle between the true wind direction and the North direction is greater than 180 degrees
        4. If the angle is greater than 180 degrees, then subtract 360 degrees from the angle
        Ask whether we need to keep the angle negative or positive (since it is a cost function then we do not need to worry about the sign)

        We need to decide whether the wind is coming from or blowing towards the boat. WE ARE USING COMING FROM

        RANGE: [-180, 180]

        """
        distance = ((s2.getY() - s1.getY()) ** 2 + (s2.getX() - s1.getX()) ** 2) ** 0.5
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
    """What is the output of this function?"""

    """Computes the absolute difference between two angles in radians"""
    return abs((a1 - a2 + math.pi) % (2 * math.pi) - math.pi)


def allocate_objective(space_information, simple_setup, heading_degrees, windDirectionDegrees):
    """Allocates a SO2 objective function with the given weight for each of the components of the objective function"""
    objective = ob.MultiOptimizationObjective(space_information)
    objective.addObjective(Distanceobjective(space_information), 1.0)
    objective.addObjective(
        HeadingObjective(space_information, simple_setup, heading_degrees=45), 100.0
    )
    objective.addObjective(WindObjective(space_information, windDirectionDegrees=8), 1.0)

    return objective
