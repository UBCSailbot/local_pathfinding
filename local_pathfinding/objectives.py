"""Our custom OMPL optimization objectives."""

import math
from ompl import base as ob

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0

# Upwind downwind constants
UPWIND_MAX_ANGLE_DEGREES = 40.0
DOWNWIND_MAX_ANGLE_DEGREES = 20.0


class Objective(ob.StateCostIntegralObjective):
    """All of our optimization objectives inherit from this class.

    Notes:
    - This class inherits from the OMPL class StateCostIntegralObjective:
        https://ompl.kavrakilab.org/classompl_1_1base_1_1StateCostIntegralObjective.html
    - Camelcase is used for functions that override OMPL functions, as that is their convention.

    Attributes:
        space_information: https://ompl.kavrakilab.org/classompl_1_1base_1_1SpaceInformation.html
    """

    def __init__(self, space_information):
        super().__init__(si=space_information, enableMotionCostInterpolation=True)
        self.space_information = space_information

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        return ob.Cost(0.0)


class DistanceObjective(Objective):
    """Generates a distance objective function

    Atributes:
        space_information (class): The space information of the OMPL problem
    """

    def __init__(self, space_information):
        super().__init__(space_information)
        self.space_information = space_information

    def motionCost(self, s1, s2):
        """Generates the distance between two points

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            class/int: The distance between two points object or integer (currently it is returning a object)
        """

        # Generates the euclidean distance between two points
        euclideanPathObjective = self.get_euclidean_path_length_objective(s1, s2)

        # Generates the latlon distance between two points
        latlonPathObjective = self.get_latlon_path_length_objective(s1, s2)

        return latlonPathObjective

    def get_ompl_path_length_objective(self):
        """Generates an OMPL Path Length Objective

        Returns:
            PathLengthOptimizationObjective: An OMPL path length objective object
        """
        path_objective = ob.PathLengthOptimizationObjective(self.space_information)

        return path_objective

    def get_euclidean_path_length_objective(self, s1, s2):
        """Generates the euclidean distance between two points

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: The euclidean distance between the two points
        """

        return ((s2.getY() - s1.getY()) ** 2 + (s2.getX() - s1.getX()) ** 2) ** (0.5)

    def get_latlon_path_length_objective(self, s1, s2):
        """Generates the "great circle" distance between two points

        I am assuming that we are using the lat and long coordinates in determining the distance
        between two points.

        Returns:
            float: The great circle distance between two points
        """

        return (
            math.acos(
                (
                    math.sin(s1.getX()) * math.sin(s2.getX())
                    + math.cos(s1.getX()) * math.cos(s2.getX())
                )
                * math.cos(s1.getY() - s2.getY())
            )
            * 6371
        )


class MinimumTurningObjective(Objective):
    """Generates a minimum turning objective function

    Attributes:
        space_information (class): The space information of the OMPL problem
        goal_x (float): The x coordinate of the goal state
        goal_y (float): The y coordinate of the goal state
        heading_degrees (float): The heading of the sailbot in degrees
    """

    def __init__(self, space_information, simple_setup, heading_degrees):
        super().__init__(space_information)
        self.space_information = space_information
        self.goal_x = simple_setup.getGoal().getState().getX()
        self.goal_y = simple_setup.getGoal().getState().getY()
        self.heading_degrees = heading_degrees

    def motionCost(self, s1, s2):
        """Generates the turning cost between s1, s2, heading or the goal position

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            int: The minimum turning cost
        """
        # Calculate the mininum turning cost between s1-goal and heading
        s1_goal__heading = self.goalHeadingTurnCost(s1)

        # Calculate the minimum turning cost between s1-s2 and s1-goal
        s1_s2__s1_goal = self.goalPathTurnCost(s1, s2)

        # Calculate the minimum turning cost from sl-s2 and heading
        s1_s2__heading = self.headingPathTurnCost(s1, s2)

        return s1_s2__s1_goal

    def goalPathTurnCost(self, s1, s2):
        """Determine the smallest turn angle between s1-s2 and s1-goal

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

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
            return 5000 * math.fabs(turn_size_unbias)

        return 100 * math.fabs(turn_size_unbias)

    def goalHeadingTurnCost(self, s1):
        """Determine the smallest turn angle between s1-s2 and heading

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

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
            return 500 * math.fabs(turn_size_unbias)

        return 10 * math.fabs(turn_size_unbias)

    def headingPathTurnCost(self, s1, s2):
        """Generates the turning cost between s1-s2 and heading of the sailbot

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

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
            return 500 * math.fabs(turn_size_unbias)

        return 10 * math.fabs(turn_size_unbias)


class WindObjective(Objective):
    """Generates a wind objective function

    Attributes:
        space_information (class): The space information of the OMPL problem
    """

    def __init__(self, space_information, wind_direction_degrees):
        super().__init__(space_information)
        self.space_information = space_information
        self.wind_direction_degrees = wind_direction_degrees

    # This objective function punishes the boat for going up/downwind
    def motionCost(self, s1, s2):
        """Generates the cost associated with the upwind and downwind directions of the boat in relation to the wind

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: The cost of going upwind or downwind
        """
        distance = ((s2.getY() - s1.getY()) ** 2 + (s2.getX() - s1.getX()) ** 2) ** 0.5
        boat_direction_radians = math.atan2(s2.getX() - s1.getX(), s2.getY() - s1.getY())

        if isUpwind(math.radians(self.wind_direction_degrees), boat_direction_radians):
            return UPWIND_MULTIPLIER * distance
        elif isDownwind(math.radians(self.wind_direction_degrees), boat_direction_radians):
            return DOWNWIND_MULTIPLIER * distance
        else:
            return 0.0


def isUpwind(wind_direction_radians, boat_direction_radians):
    """Determines whether the boat is upwind or not and its associated cost

    Args:
        wind_direction_radians (float): The true wind direction (radians). [-pi, pi)
        boat_direction_radians (float): The direction of the boat (radians). [-pi, pi)

    Returns:
        bool: The cost associated with the upwind direction
    """
    theta_min = math.degrees(wind_direction_radians - math.radians(UPWIND_MAX_ANGLE_DEGREES))
    theta_max = math.degrees(wind_direction_radians + math.radians(UPWIND_MAX_ANGLE_DEGREES))

    return is_angle_between(theta_min, math.degrees(boat_direction_radians), theta_max)


def isDownwind(wind_direction_radians, boat_direction_radians):
    """Generates the cost associated with the downwind direction

    Args:
        wind_direction_radians (float): The true wind direction (radians). [-pi, pi)
        boat_direction_radians (float)): The direction of the boat (radians). [-pi, pi)

    Returns:
        bool: The cost associated with the downwind direction
    """
    downwind_wind_direction_radians = math.radians(bound_to_180(wind_direction_radians + math.pi))

    theta_min = math.degrees(
        downwind_wind_direction_radians - math.radians(DOWNWIND_MAX_ANGLE_DEGREES)
    )
    theta_max = math.degrees(
        downwind_wind_direction_radians + math.radians(DOWNWIND_MAX_ANGLE_DEGREES)
    )

    return is_angle_between(theta_min, math.degrees(boat_direction_radians), theta_max)


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


def get_sailing_objective(
    space_information, simple_setup, heading_degrees: float, wind_direction_degrees: float
) -> ob.OptimizationObjective:
    objective = ob.MultiOptimizationObjective(si=space_information)
    objective.addObjective(objective=DistanceObjective(space_information), weight=1.0)
    objective.addObjective(
        objective=MinimumTurningObjective(space_information, simple_setup, heading_degrees),
        weight=100.0,
    )
    objective.addObjective(
        objective=WindObjective(space_information, wind_direction_degrees), weight=1.0
    )

    return objective
