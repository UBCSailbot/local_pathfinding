"""Our custom OMPL optimization objectives."""

import math
from enum import Enum, auto

from ompl import base as ob

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0

# Upwind downwind constants
UPWIND_MAX_ANGLE_DEGREES = math.radians(40.0)
DOWNWIND_MAX_ANGLE_DEGREES = math.radians(20.0)


class DistanceMethod(Enum):
    """Determines the method of distance objective function

    Attributes:
        euclidean (str): The euclidean distance objective function
        latlon (str): The latlon distance objective function
    """

    EUCLIDEAN = auto()
    LATLON = auto()
    OMPL_PATH_LENGTH = auto()


class MinimumTurningMethod(Enum):
    """Determines the method of minimum turning objective function

    Attributes:
        goal_heading (str): The goal heading objective function
        goal_path (str): The goal path objective function
        heading_path (str): The heading path objective function
    """

    GOAL_HEADING = auto()
    GOAL_PATH = auto()
    HEADING_PATH = auto()


class Objective(ob.StateCostIntegralObjective):
    """All of our optimization objectives inherit from this class.

    Notes:
    - This class inherits from the OMPL class StateCostIntegralObjective:
        https://ompl.kavrakilab.org/classompl_1_1base_1_1StateCostIntegralObjective.html
    - Camelcase is used for functions that override OMPL functions, as that is their convention.

    Attributes:
        space_information (StateSpacePtr): Contains all the information about
            the space planning is done in.
    """

    def __init__(self, space_information):
        super().__init__(si=space_information, enableMotionCostInterpolation=True)
        self.space_information = space_information

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        raise NotImplementedError


class DistanceObjective(Objective):
    """Generates a distance objective function

    Attributes:
        implementation (str): The implementation of the distance objective function

    """

    def __init__(self, space_information, implementation: DistanceMethod):
        super().__init__(space_information)
        self.implementation = implementation

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the distance between two points

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            class/int: The distance between two points object or integer
                       (currently it is returning a object)
        """
        if self.implementation == DistanceMethod.EUCLIDEAN:
            # Generates the euclidean distance between two points
            return self.get_euclidean_path_length_objective(s1, s2)
        elif self.implementation == DistanceMethod.LATLON:
            # Generates the latlon distance between two points
            return self.get_latlon_path_length_objective(s1, s2)
        else:
            ValueError(f"Implementation {self.implementation} not supported")

    def get_ompl_path_length_objective(self):
        """Generates an OMPL Path Length Objective

        Returns:
            PathLengthOptimizationObjective: An OMPL path length objective object
        """
        path_objective = ob.PathLengthOptimizationObjective(self.space_information)

        return path_objective

    def get_euclidean_path_length_objective(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace):
        """Generates the euclidean distance between two points

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: The euclidean distance between the two points
        """

        return ((s2.getY() - s1.getY()) ** 2.0 + (s2.getX() - s1.getX()) ** 2.0) ** (0.5)

    def get_latlon_path_length_objective(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace):
        """Generates the "great circle" distance between two points

        I am assuming that we are using the lat and long coordinates in determining the distance
        between two points.

        Returns:
            float: The great circle distance between two points
        """

        return (
            math.acos(
                (
                    math.sin(math.radians(s1.getX())) * math.sin(math.radians(s2.getX()))
                    + math.cos(math.radians(s1.getX())) * math.cos(math.radians(s2.getX()))
                )
                * math.cos(math.radians(s1.getY() - s2.getY()))
            )
            * 6378
        )


class MinimumTurningObjective(Objective):
    """Generates a minimum turning objective function

    Attributes:
        goal_x (float): The x coordinate of the goal state
        goal_y (float): The y coordinate of the goal state
        heading (float): The heading of the sailbot in radians (-pi, pi]
        implementation (str): The implementation of the minimum turning objective function
    """

    def __init__(
        self,
        space_information,
        simple_setup,
        heading_degrees,
        implementation: MinimumTurningMethod,
    ):
        super().__init__(space_information)
        self.goal_x = simple_setup.getGoal().getState().getX()
        self.goal_y = simple_setup.getGoal().getState().getY()
        self.heading = math.radians(heading_degrees)
        self.implementation = implementation

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the turning cost between s1, s2, heading or the goal position

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: The minimum turning angle (degrees)
        """

        if self.implementation == MinimumTurningMethod.GOAL_HEADING:
            # Calculate the minimum turning cost between s1-goal and heading
            return self.goal_heading_turn_cost(s1)
        elif self.implementation == MinimumTurningMethod.GOAL_PATH:
            # Calculate the minimum turning cost between s1-s2 and s1-goal
            return self.goal_path_turn_cost(s1, s2)
        elif self.implementation == MinimumTurningMethod.HEADING_PATH:
            # Calculate the minimum turning cost from sl-s2 and heading
            return self.heading_path_turn_cost(s1, s2)
        else:
            ValueError(f"Implementation {self.implementation} not supported")

    def goal_path_turn_cost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace):
        """Determine the smallest turn angle between s1-s2 and s1-goal

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: the turning angle from s2 to s1 (degrees)
        """

        # Calculate the angle of the s1-s2 line segment from North
        path_direction = math.atan2(s2.getX() - s1.getX(), s2.getY() - s1.getY())

        # Calculate the angle from s1-goal line segment from North
        global_goal_direction = math.atan2(self.goal_x - s1.getX(), self.goal_y - s1.getY())

        return self.min_turn_angle(global_goal_direction, path_direction)

    def goal_heading_turn_cost(self, s1: ob.SE2StateSpace):
        """Determine the smallest turn angle between s1-s2 and heading

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: the turning angle from s2 to s1 (degrees)
        """

        # Calculate the angle from s1-goal line segment from North
        global_goal_direction = math.atan2(self.goal_x - s1.getX(), self.goal_y - s1.getY())

        return self.min_turn_angle(global_goal_direction, self.heading)

    def heading_path_turn_cost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace):
        """Generates the turning cost between s1-s2 and heading of the sailbot

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: The minimum turning angle between s1-s2 and heading  (degrees)
        """

        # Calculate the angle of the s1-s2 line segment from North
        path_direction = math.atan2(s2.getX() - s1.getX(), s2.getY() - s1.getY())

        return self.min_turn_angle(path_direction, self.heading)

    @staticmethod
    def min_turn_angle(angle1: float, angle2: float) -> float:
        """Calculates the minimum turning angle between two angles

        Args:
            angle1 (float): The first angle in radians
            angle2 (float): The second angle in radians

        Returns:
            float: The minimum turning angle between the two angles (degrees)
        """

        # Calculate the uncorrected turn size [0, 2pi]
        turn_size_bias = math.fabs(angle1 - angle2)

        # Correct the angle in between [0, pi]
        if turn_size_bias > math.pi:
            turn_size_unbias = 2 * math.pi - turn_size_bias
        else:
            turn_size_unbias = turn_size_bias

        return math.degrees(math.fabs(turn_size_unbias))


class WindObjective(Objective):
    """Generates a wind objective function

    Attributes:
        wind_direction (float): The direction of the wind in radians [-pi, pi)
    """

    def __init__(self, space_information, wind_direction: float):
        super().__init__(space_information)
        self.wind_direction = math.radians(wind_direction)

    # This objective function punishes the boat for going up/downwind
    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the cost associated with the upwind and downwind directions of the boat in
           relation to the wind

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: The cost of going upwind or downwind
        """
        distance = ((s2.getY() - s1.getY()) ** 2 + (s2.getX() - s1.getX()) ** 2) ** 0.5
        boat_direction_radians = math.atan2(s2.getX() - s1.getX(), s2.getY() - s1.getY())

        if WindObjective.is_upwind(self.wind_direction, boat_direction_radians):
            return UPWIND_MULTIPLIER * distance
        elif WindObjective.is_downwind(self.wind_direction, boat_direction_radians):
            return DOWNWIND_MULTIPLIER * distance
        else:
            return 0.0

    @staticmethod
    def is_upwind(wind_direction: float, boat_direction: float) -> bool:
        """Determines whether the boat is upwind or not and its associated cost

        Args:
            wind_direction (float): The true wind direction (radians). [-pi, pi)
            boat_direction (float): The direction of the boat (radians). [-pi, pi)

        Returns:
            bool: The cost associated with the upwind direction
        """
        theta_min = wind_direction - UPWIND_MAX_ANGLE_DEGREES
        theta_max = wind_direction + UPWIND_MAX_ANGLE_DEGREES

        return WindObjective.is_angle_between(theta_min, boat_direction, theta_max)

    @staticmethod
    def is_downwind(wind_direction: float, boat_direction: float) -> bool:
        """Generates the cost associated with the downwind direction

        Args:
            wind_direction (float): The true wind direction (radians). [-pi, pi)
            boat_direction_radians (float)): The direction of the boat (radians). [-pi, pi)

        Returns:
            bool: The cost associated with the downwind direction
        """
        downwind_wind_direction = (wind_direction + math.pi) % (2 * math.pi)

        theta_min = downwind_wind_direction - DOWNWIND_MAX_ANGLE_DEGREES

        theta_max = downwind_wind_direction + DOWNWIND_MAX_ANGLE_DEGREES

        return WindObjective.is_angle_between(theta_min, boat_direction, theta_max)

    @staticmethod
    def is_angle_between(first_angle: float, middle_angle: float, second_angle: float) -> bool:
        """Determines whether an angle is between two other angles.

        Args:
            first_angle (float): The first bounding angle in radians.
            middle_angle (float): The angle in question in radians.
            second_angle (float): The second bounding angle in radians.

        Returns:
            bool: True when `middle_angle` is not in the reflex angle of
                `first_angle` and `second_angle`, false otherwise.
        """
        if first_angle < second_angle:
            if (
                second_angle - math.pi == first_angle
            ):  # Assume all angles are between first and second when pi degrees apart
                return middle_angle != first_angle and middle_angle != second_angle
            elif second_angle - math.pi < first_angle:
                return middle_angle > first_angle and middle_angle < second_angle
            else:
                return middle_angle < first_angle or middle_angle > second_angle
        else:
            return WindObjective.is_angle_between(second_angle, middle_angle, first_angle)


def get_sailing_objective(
    space_information, simple_setup, heading_degrees: float, wind_direction_degrees: float
) -> ob.OptimizationObjective:
    objective = ob.MultiOptimizationObjective(si=space_information)
    objective.addObjective(
        objective=DistanceObjective(space_information, DistanceMethod.EUCLIDEAN), weight=1.0
    )
    objective.addObjective(
        objective=MinimumTurningObjective(
            space_information, simple_setup, heading_degrees, MinimumTurningMethod.GOAL_HEADING
        ),
        weight=100.0,
    )
    objective.addObjective(
        objective=WindObjective(space_information, wind_direction_degrees), weight=1.0
    )

    return objective
