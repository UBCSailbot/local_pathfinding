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

        # Calculate the minimum turning cost between s1-s2 and s1-goal
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
        path_direction = math.atan2(s2.getX() - s1.getX(), s2.getY() - s1.getY())

        # Calculate the angle from s1-goal line segment from North
        global_goal_direction = math.atan2(self.goal_x - s1.getX(), self.goal_y - s1.getY())

        # Calculate the uncorrected turn size [0, 2pi]
        turn_size_bias = math.fabs(global_goal_direction - path_direction)

        # Correct the angle in between [0, pi]
        if turn_size_bias > math.pi:
            turn_size_unbias = turn_size_bias - 2 * math.pi
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
        global_goal_direction = math.atan2(self.goal_x - s1.getX(), self.goal_y - s1.getY())

        # Calculate the uncorrected turn size [0, 2pi]
        turn_size_bias = math.fabs(global_goal_direction - self.heading_degrees)

        # Correct the angle in between [0, pi]
        if turn_size_bias > math.pi:
            turn_size_unbias = turn_size_bias - 2 * math.pi
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
        path_direction = math.atan2(s2.getX() - s1.getX(), s2.getY() - s1.getY())

        # Calculate turn size
        turn_size_bias = math.fabs(path_direction - self.heading_degrees)

        # Correct the angle in between [0, pi]
        if turn_size_bias > math.pi:
            turn_size_unbias = turn_size_bias - 2 * math.pi
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
        boatDirectionRadians = math.atan2(s2.getX() - s1.getX(), s2.getY() - s1.getY())

        if isUpwind(math.radians(self.windDirectionDegrees), boatDirectionRadians):
            return UPWIND_MULTIPLIER * distance
        elif isDownwind(math.radians(self.windDirectionDegrees), boatDirectionRadians):
            return DOWNWIND_MULTIPLIER * distance
        else:
            return 0.0


def isUpwind(windDirectionRadians, boatDirectionRadians):
    """Determines whether the boat is upwind or not and its associated cost

    Args:
        windDirectionRadians (float): The true wind direction (radians). [-pi, pi)
        boatDirectionRadians (float): The direction of the boat (radians). [-pi, pi)

    Returns:
        float: The cost associated with the upwind direction
    """
    thetamin = math.degrees(windDirectionRadians - math.radians(UPWIND_MAX_ANGLE_DEGREES))
    thetamax = math.degrees(windDirectionRadians + math.radians(UPWIND_MAX_ANGLE_DEGREES))

    return is_angle_between(thetamin, math.degrees(boatDirectionRadians), thetamax)


def isDownwind(windDirectionRadians, boatDirectionRadians):
    downwind_windDirectionRadians = math.radians(bound_to_180(windDirectionRadians + math.pi))

    thetamin = math.degrees(
        downwind_windDirectionRadians - math.radians(DOWNWIND_MAX_ANGLE_DEGREES)
    )
    thetamax = math.degrees(
        downwind_windDirectionRadians + math.radians(DOWNWIND_MAX_ANGLE_DEGREES)
    )

    return is_angle_between(thetamin, math.degrees(boatDirectionRadians), thetamax)


def bound_to_180(angle):
    """Bounds the provided angle between [-180, 180) degrees.

    e.g.)
        bound_to_180(135) = 135.0
        bound_to_180(200) = -160.0

    Args:
        angle (float): The input angle in degrees.

    Returns:
        float: The bounded angle in degrees.
    """

    # in360 bounds the angle within 0-360 degrees
    # in180 bounds the angle within 0-180 degrees

    if angle < 0:  # Checks whether the angle is negative or positive
        in360 = angle % -360.0  # Bounds the angle within 360 degrees
        if in360 < -180:
            in180 = abs(angle) % 180
        elif in360 == -180:
            #  in180 = -180 % -180 == 0
            in180 = -180
        else:
            in180 = angle % -180
    else:
        in360 = angle % 360.0
        if in360 > 180:
            in180 = -angle % -180
        elif in360 == 180:
            #  in180 = 180 % 180 = 0 not -180
            in180 = -180
        else:
            in180 = angle % 180

    return in180


def is_angle_between(first_angle, middle_angle, second_angle):
    """Determines whether an angle is between two other angles.

    e.g.)
        is_angle_between(0, 45, 90) = True
        is_angle_between(45, 90, 270) = False

    Args:
        first_angle (float): The first bounding angle in degrees.
        middle_angle (float): The angle in question in degrees.
        second_angle (float): The second bounding angle in degrees.

    Returns:
        bool: True when `middle_angle` is not in the reflex angle of `first_angle` and `second_angle`, false otherwise.
    """

    first_angle, second_angle = bound_to_180(first_angle), bound_to_180(second_angle)

    # Assuming all angles are positive
    diff_angle = abs(first_angle - second_angle)
    if second_angle >= first_angle:
        max_angle, min_angle = second_angle, first_angle
    else:
        max_angle, min_angle = first_angle, second_angle

    if diff_angle > 180:  # Reflex angle region
        #  Checks whether the angle is outside the reflex angle region
        if middle_angle > max_angle or middle_angle < min_angle:
            return True
        else:
            return False
    else:
        if min_angle <= middle_angle <= max_angle:  # Checks whether within anti-reflex region.
            return True
        else:
            return False


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
