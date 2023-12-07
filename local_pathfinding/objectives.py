"""Our custom OMPL optimization objectives."""

import math
from enum import Enum, auto

import numpy as np
from ompl import base as ob

import local_pathfinding.coord_systems as cs

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0

# Upwind downwind constants
HIGHEST_UPWIND_ANGLE_RADIANS = math.radians(40.0)
LOWEST_DOWNWIND_ANGLE_RADIANS = math.radians(20.0)


class DistanceMethod(Enum):
    """Enumeration for distance objective methods"""

    EUCLIDEAN = auto()
    LATLON = auto()
    OMPL_PATH_LENGTH = auto()


class MinimumTurningMethod(Enum):
    """Enumeration for minimum turning objective methods"""

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
        method (DistanceMethod): The method of the distance objective function
        ompl_path_objective (ob.PathLengthOptimizationObjective): The OMPL path length objective.
            Only defined if the method is OMPL path length.
        reference (cs.LatLon): The XY origin when converting from latlon to XY.
            Only defined if the method is latlon.
    """

    def __init__(
        self,
        space_information,
        method: DistanceMethod,
        reference=cs.LatLon(0, 0),
        num_samples: int = 10,
    ):
        super().__init__(space_information)
        self.method = method
        if self.method == DistanceMethod.OMPL_PATH_LENGTH:
            self.ompl_path_objective = ob.PathLengthOptimizationObjective(self.space_information)
        elif self.method == DistanceMethod.LATLON:
            self.reference = reference

        self.sampled_states = self._sample_states(si=space_information, num_samples=num_samples)
        self.max_motionCost = 1
        self.max_motionCost = self._find_maximum_motion_cost()

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the distance between two points

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            ob.Cost: The distance between two points object

        Raises:
            ValueError: If the distance method is not supported
        """
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        if self.method == DistanceMethod.EUCLIDEAN:
            distance = DistanceObjective.get_euclidean_path_length_objective(s1_xy, s2_xy)
            cost = ob.Cost(distance)
        elif self.method == DistanceMethod.LATLON:
            distance = DistanceObjective.get_latlon_path_length_objective(
                s1_xy, s2_xy, self.reference
            )
            cost = ob.Cost(distance)
        elif self.method == DistanceMethod.OMPL_PATH_LENGTH:
            cost = self.ompl_path_objective.motionCost(s1_xy, s2_xy)
        else:
            ValueError(f"Method {self.method} not supported")
        return cost

    @staticmethod
    def get_euclidean_path_length_objective(s1: cs.XY, s2: cs.XY) -> float:
        """Generates the euclidean distance between two points

        Args:
            s1 (cs.XY): The starting point of the local start state
            s2 (cs.XY): The ending point of the local goal state

        Returns:
            float: The euclidean distance between the two points
        """
        return math.hypot(s2.y - s1.y, s2.x - s1.x)

    @staticmethod
    def get_latlon_path_length_objective(s1: cs.XY, s2: cs.XY, reference: cs.LatLon) -> float:
        """Generates the "great circle" distance between two points

        I am assuming that we are using the lat and long coordinates in determining the distance
        between two points.

        Args:
            s1 (cs.XY): The starting point of the local start state
            s2 (cs.XY): The ending point of the local goal state

        Returns:
            float: The great circle distance between two points
        """
        latlon1 = cs.xy_to_latlon(reference, s1)
        latlon2 = cs.xy_to_latlon(reference, s2)

        _, _, distance_m = cs.GEODESIC.inv(
            latlon1.longitude, latlon1.latitude, latlon2.longitude, latlon2.latitude
        )

        return distance_m

    def _find_maximum_motion_cost(self):
        max_cost = 0
        n = len(self.sampled_states)
        for i in range(n):
            for j in range(i + 1, n):
                cost = self.motionCost(self.sampled_states[i], self.sampled_states[j])
                max_cost = max(max_cost, cost)

        return max_cost

    def _sample_states(self, si: ob.SpaceInformation, num_samples: int):
        sampler = si.getStateSpace().allocDefaultStateSampler()

        sampled_states = []

        for _ in range(num_samples):
            state = si.getStateSpace().allocState()
            sampler.sampleUniform(state)
            state_array = np.array([state[0], state[1]])
            sampled_states.append(state_array)

        return sampled_states


class MinimumTurningObjective(Objective):
    """Generates a minimum turning objective function

    Attributes:
        goal (cs.XY): The goal position of the sailbot
        heading (float): The heading of the sailbot in radians (-pi, pi]
        method (MinimumTurningMethod): The method of the minimum turning objective function
    """

    def __init__(
        self,
        space_information,
        simple_setup,
        heading_degrees: float,
        method: MinimumTurningMethod,
    ):
        super().__init__(space_information)
        self.goal = cs.XY(
            simple_setup.getGoal().getState().getX(), simple_setup.getGoal().getState().getY()
        )
        assert -180 < heading_degrees <= 180
        self.heading = math.radians(heading_degrees)
        self.method = method

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the turning cost between s1, s2, heading or the goal position

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            ob.Cost: The minimum turning angle in degrees

        Raises:
            ValueError: If the minimum turning method is not supported
        """
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        if self.method == MinimumTurningMethod.GOAL_HEADING:
            angle = self.goal_heading_turn_cost(s1_xy, self.goal, self.heading)
        elif self.method == MinimumTurningMethod.GOAL_PATH:
            angle = self.goal_path_turn_cost(s1_xy, s2_xy, self.goal)
        elif self.method == MinimumTurningMethod.HEADING_PATH:
            angle = self.heading_path_turn_cost(s1_xy, s2_xy, self.heading)
        else:
            ValueError(f"Method {self.method} not supported")
        return ob.Cost(angle)

    @staticmethod
    def goal_heading_turn_cost(s1: cs.XY, goal: cs.XY, heading: float) -> float:
        """Determine the smallest turn angle between s1-goal and heading

        Args:
            s1 (cs.XY): The starting point of the local start state
            goal (cs.XY): The goal position of the sailbot
            heading (float): The heading of the sailbot in radians (-pi, pi]

        Returns:
            float: the turning angle from s2 to s1 in degrees
        """
        # Calculate the true bearing of the goal from s1
        global_goal_direction = math.atan2(goal.x - s1.x, goal.y - s1.y)

        return MinimumTurningObjective.min_turn_angle(global_goal_direction, heading)

    @staticmethod
    def goal_path_turn_cost(s1: cs.XY, s2: cs.XY, goal: cs.XY) -> float:
        """Determine the smallest turn angle between s1-s2 and s1-goal

        Args:
            s1 (cs.XY): The starting point of the local start state
            s2 (cs.XY): The ending point of the local goal state
            goal (cs.XY): The goal position of the sailbot

        Returns:
            float: the turning angle from s2 to s1 in degrees
        """
        # Calculate the true bearing of s2 from s1
        path_direction = math.atan2(s2.x - s1.x, s2.y - s1.y)

        # Calculate the true bearing of the goal from s1
        global_goal_direction = math.atan2(goal.x - s1.x, goal.y - s1.y)

        return MinimumTurningObjective.min_turn_angle(global_goal_direction, path_direction)

    @staticmethod
    def heading_path_turn_cost(s1: cs.XY, s2: cs.XY, heading: float) -> float:
        """Generates the turning cost between s1-s2 and heading of the sailbot

        Args:
            s1 (cs.XY): The starting point of the local start state
            s2 (cs.XY): The ending point of the local goal state
            heading (float): The heading of the sailbot in radians (-pi, pi]

        Returns:
            float: The minimum turning angle between s1-s2 and heading in degrees
        """
        # Calculate the true bearing of s2 from s1
        path_direction = math.atan2(s2.x - s1.x, s2.y - s1.y)

        return MinimumTurningObjective.min_turn_angle(path_direction, heading)

    @staticmethod
    def min_turn_angle(angle1: float, angle2: float) -> float:
        """Calculates the minimum turning angle between two angles

        Args:
            angle1 (float): The first angle in radians
            angle2 (float): The second angle in radians
                Must be bounded within 2pi radians of `angle1`

        Returns:
            float: The minimum turning angle between the two angles in degrees
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
        wind_direction (float): The direction of the wind in radians (-pi, pi]
    """

    def __init__(self, space_information, wind_direction_degrees: float):
        super().__init__(space_information)
        assert -180 < wind_direction_degrees <= 180
        self.wind_direction = math.radians(wind_direction_degrees)

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        """Generates the cost associated with the upwind and downwind directions of the boat in
        relation to the wind.

        Args:
            s1 (SE2StateInternal): The starting point of the local start state
            s2 (SE2StateInternal): The ending point of the local goal state

        Returns:
            ob.Cost: The cost of going upwind or downwind
        """
        s1_xy = cs.XY(s1.getX(), s1.getY())
        s2_xy = cs.XY(s2.getX(), s2.getY())
        return ob.Cost(WindObjective.wind_direction_cost(s1_xy, s2_xy, self.wind_direction))

    @staticmethod
    def wind_direction_cost(s1: cs.XY, s2: cs.XY, wind_direction: float) -> float:
        """Punishes the boat for going up/downwind.

        Args:
            s1 (cs.XY): The starting point of the local start state
            s2 (cs.XY): The ending point of the local goal state
            wind_direction (float): The direction of the wind in radians (-pi, pi]

        Returns:
            float: The cost of going upwind or downwind
        """
        distance = math.hypot(s2.y - s1.y, s2.x - s1.x)
        boat_direction_radians = math.atan2(s2.x - s1.x, s2.y - s1.y)
        assert -math.pi <= boat_direction_radians <= math.pi

        if WindObjective.is_upwind(wind_direction, boat_direction_radians):
            return UPWIND_MULTIPLIER * distance
        elif WindObjective.is_downwind(wind_direction, boat_direction_radians):
            return DOWNWIND_MULTIPLIER * distance
        else:
            return 0.0

    @staticmethod
    def is_upwind(wind_direction: float, boat_direction: float) -> bool:
        """Determines whether the boat is upwind or not and its associated cost

        Args:
            wind_direction (float): The true wind direction (radians). (-pi, pi]
            boat_direction (float): The direction of the boat (radians). [-pi, pi]

        Returns:
            bool: The cost associated with the upwind direction
        """
        theta_min = wind_direction - HIGHEST_UPWIND_ANGLE_RADIANS
        theta_max = wind_direction + HIGHEST_UPWIND_ANGLE_RADIANS

        return WindObjective.is_angle_between(theta_min, boat_direction, theta_max)

    @staticmethod
    def is_downwind(wind_direction: float, boat_direction: float) -> bool:
        """Generates the cost associated with the downwind direction

        Args:
            wind_direction (float): The true wind direction (radians). (-pi, pi]
            boat_direction_radians (float)): The direction of the boat (radians). [-pi, pi]

        Returns:
            bool: The cost associated with the downwind direction
        """
        downwind_wind_direction = (wind_direction + math.pi) % (2 * math.pi)

        theta_min = downwind_wind_direction - LOWEST_DOWNWIND_ANGLE_RADIANS

        theta_max = downwind_wind_direction + LOWEST_DOWNWIND_ANGLE_RADIANS

        return WindObjective.is_angle_between(theta_min, boat_direction, theta_max)

    @staticmethod
    def is_angle_between(first_angle: float, middle_angle: float, second_angle: float) -> bool:
        """Determines whether an angle is between two other angles

        Args:
            first_angle (float): The first bounding angle in radians
            middle_angle (float): The angle in question in radians
            second_angle (float): The second bounding angle in radians

        Returns:
            bool: True when `middle_angle` is not in the reflex angle of
                `first_angle` and `second_angle`, false otherwise.
        """
        # Bound the angles to [0, 2pi)
        first_angle = first_angle % (2 * math.pi)
        middle_angle = middle_angle % (2 * math.pi)
        second_angle = second_angle % (2 * math.pi)

        if first_angle <= second_angle:
            if second_angle - math.pi == first_angle:
                # Assume all angles are between first and second
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
        objective=DistanceObjective(space_information, DistanceMethod.LATLON),
        weight=1.0,
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
