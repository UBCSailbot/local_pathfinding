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
        space_information (StateSpacePtr): Contains all the information about
            the space planning is done in.
    """

    def __init__(self, space_information):
        super().__init__(si=space_information, enableMotionCostInterpolation=True)
        self.space_information = space_information

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        raise NotImplementedError


class DistanceObjective(Objective):
    """Generates a distance objective function"""

    def __init__(self, space_information):
        super().__init__(space_information)

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the distance between two points

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            class/int: The distance between two points object or integer
                       (currently it is returning a object)
        """

        # Generates the euclidean distance between two points
        # euclideanPathObjective = self.get_euclidean_path_length_objective(s1, s2)

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
        heading_degrees (float): The heading of the sailbot in degrees
    """

    def __init__(self, space_information, simple_setup, heading_degrees):
        super().__init__(space_information)
        self.goal_x = simple_setup.getGoal().getState().getX()
        self.goal_y = simple_setup.getGoal().getState().getY()
        self.heading_degrees = math.radians(heading_degrees)

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the turning cost between s1, s2, heading or the goal position

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: The minimum turning angle (degrees)
        """
        # Calculate the mininum turning cost between s1-goal and heading
        # s1_goal__heading = self.goal_heading_turn_cost(s1)

        # Calculate the minimum turning cost between s1-s2 and s1-goal
        s1_s2__s1_goal = self.goal_path_turn_cost(s1, s2)

        # Calculate the minimum turning cost from sl-s2 and heading
        # s1_s2__heading = self.heading_path_turn_cost(s1, s2)

        return s1_s2__s1_goal

    def goal_path_turn_cost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace):
        """Determine the smallest turn angle between s1-s2 and s1-goal

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: the turning angle from s2 to s1 (degrees)
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
            return math.degrees(math.fabs(turn_size_unbias))

        return math.degrees(math.fabs(turn_size_unbias))

    def goal_heading_turn_cost(self, s1: ob.SE2StateSpace):
        """Determine the smallest turn angle between s1-s2 and heading

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: the turning angle from s2 to s1 (degrees)
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
            return math.degrees(math.fabs(turn_size_unbias))

        return math.degrees(math.fabs(turn_size_unbias))

    def heading_path_turn_cost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace):
        """Generates the turning cost between s1-s2 and heading of the sailbot

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            float: The minimum turning angle between s1-s2 and heading  (degrees)
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
            return math.degrees(math.fabs(turn_size_unbias))

        return math.degrees(math.fabs(turn_size_unbias))


class WindObjective(Objective):
    """Generates a wind objective function"""

    def __init__(self, space_information, wind_direction_degrees):
        super().__init__(space_information)
        self.wind_direction_degrees = wind_direction_degrees

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

        if WindObjective.is_upwind(
            math.radians(self.wind_direction_degrees), boat_direction_radians
        ):
            return UPWIND_MULTIPLIER * distance
        elif WindObjective.is_downwind(
            math.radians(self.wind_direction_degrees), boat_direction_radians
        ):
            return DOWNWIND_MULTIPLIER * distance
        else:
            return 0.0

    @staticmethod
    def is_upwind(wind_direction_radians: float, boat_direction_radians: float) -> bool:
        """Determines whether the boat is upwind or not and its associated cost

        Args:
            wind_direction_radians (float): The true wind direction (radians). [-pi, pi)
            boat_direction_radians (float): The direction of the boat (radians). [-pi, pi)

        Returns:
            bool: The cost associated with the upwind direction
        """
        theta_min = math.degrees(wind_direction_radians - math.radians(UPWIND_MAX_ANGLE_DEGREES))
        theta_max = math.degrees(wind_direction_radians + math.radians(UPWIND_MAX_ANGLE_DEGREES))

        return WindObjective.is_angle_between(
            theta_min, math.degrees(boat_direction_radians), theta_max
        )

    @staticmethod
    def is_downwind(wind_direction_radians: float, boat_direction_radians: float) -> bool:
        """Generates the cost associated with the downwind direction

        Args:
            wind_direction_radians (float): The true wind direction (radians). [-pi, pi)
            boat_direction_radians (float)): The direction of the boat (radians). [-pi, pi)

        Returns:
            bool: The cost associated with the downwind direction
        """
        downwind_wind_direction_radians = math.radians(
            WindObjective.bound_to_180(wind_direction_radians + math.pi)
        )

        theta_min = math.degrees(
            downwind_wind_direction_radians - math.radians(DOWNWIND_MAX_ANGLE_DEGREES)
        )
        theta_max = math.degrees(
            downwind_wind_direction_radians + math.radians(DOWNWIND_MAX_ANGLE_DEGREES)
        )

        return WindObjective.is_angle_between(
            theta_min, math.degrees(boat_direction_radians), theta_max
        )

    @staticmethod
    def bound_to_180(angle: float) -> float:
        """Bounds the provided angle between [-180, 180) degrees.

        Args:
            angle (float): The input angle in degrees.

        Returns:
            float: The bounded angle in degrees.
        """
        return angle - 360 * ((angle + 180) // 360)

    @staticmethod
    def is_angle_between(first_angle: float, middle_angle: float, second_angle: float) -> bool:
        """Determines whether an angle is between two other angles.

        Args:
            first_angle (float): The first bounding angle in degrees.
            middle_angle (float): The angle in question in degrees.
            second_angle (float): The second bounding angle in degrees.

        Returns:
            bool: True when `middle_angle` is not in the reflex angle of
                `first_angle` and `second_angle`, false otherwise.
        """
        if first_angle < second_angle:
            if (
                second_angle - 180 == first_angle
            ):  # Assume all angles are between first and second when 180 degrees apart
                return middle_angle != first_angle and middle_angle != second_angle
            elif second_angle - 180 < first_angle:
                return middle_angle > first_angle and middle_angle < second_angle
            else:
                return middle_angle < first_angle or middle_angle > second_angle
        else:
            return WindObjective.is_angle_between(second_angle, middle_angle, first_angle)


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
