"""Our custom OMPL optimization objectives."""

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
        raise NotImplementedError


class DistanceObjective(Objective):
    def __init__(self, space_information):
        super().__init__(space_information)

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        return ob.Cost()

    def _get_path_length_objective(self):
        raise NotImplementedError

    def _get_euclidean_path_length_objective(self, s1, s2):
        raise NotImplementedError

    def _get_latlon_path_length_objective(self, s1, s2):
        raise NotImplementedError


class MinimumTurningObjective(Objective):
    def __init__(self, space_information, simple_setup, heading_degrees: float):
        super().__init__(space_information)
        self.simple_setup = simple_setup
        self.heading_degrees = heading_degrees

    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        return ob.Cost()

    def _goal_path_turn_cost(self, s1, s2):
        raise NotImplementedError

    def _goal_heading_turn_cost(self, s1):
        raise NotImplementedError

    def _heading_path_turn_cost(self, s1, s2):
        raise NotImplementedError


class WindObjective(Objective):
    def __init__(self, space_information, wind_direction_degrees: float):
        super().__init__(space_information)
        self.wind_direction_degrees = wind_direction_degrees

    # This objective function punishes the boat for going up/downwind
    def motionCost(self, s1: ob.SE2StateSpace, s2: ob.SE2StateSpace) -> ob.Cost:
        return ob.Cost()

    @staticmethod
    def _is_upwind(wind_direction_radians: float, boat_direction_radians: float) -> bool:
        raise NotImplementedError

    @staticmethod
    def _is_downwind(wind_direction_radians: float, boat_direction_radians: float) -> bool:
        raise NotImplementedError


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
