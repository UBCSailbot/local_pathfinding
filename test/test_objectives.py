import math  # for math.degrees()

import pytest
from ompl import base as ob
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.objectives as objectives
import local_pathfinding.ompl_path as ompl_path

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0


PATH = ompl_path.OMPLPath(
    parent_logger=RcutilsLogger(),
    max_runtime=1,
    local_path_state=None,  # type: ignore[arg-type] # None is placeholder
)


def test_distance_objective():
    distance_objective = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())
    assert distance_objective is not None


def test_get_path_length_objective():
    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    assert dist_object.get_ompl_path_length_objective() is not None


@pytest.mark.parametrize(
    "cs1,cs2,expected",
    [
        ((0, 0), (0, 0), 0),
        ((0.5, 0.5), (0.1, 0.2), 0.5),
    ],
)
def test_get_euclidean_path_length_objective(cs1: tuple, cs2: tuple, expected: float):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s1().setXY(cs1[0], cs1[1])

    s2 = ob.State(space)
    s2().setXY(cs2[0], cs2[1])

    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    assert dist_object.get_euclidean_path_length_objective(s1(), s2()) == expected


@pytest.mark.parametrize(
    "cs1,cs2,expected",
    [
        ((0, 0), (0, 0), 0),
        ((13.205724, 29.828011), (13.205824, 29.828111), 0.01552),
    ],
)
def test_get_latlon_path_length_objective(cs1: tuple, cs2: tuple, expected: float):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s1().setXY(cs1[0], cs1[1])

    s2 = ob.State(space)
    s2().setXY(cs2[0], cs2[1])

    dist_object = objectives.DistanceObjective(PATH._simple_setup.getSpaceInformation())

    assert round(dist_object.get_latlon_path_length_objective(s1(), s2()), 2) == round(expected, 2)


def test_minimum_turning_objective():
    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), PATH._simple_setup, PATH.state.heading_direction
    )
    assert minimum_turning_objective is not None


@pytest.mark.parametrize(
    "cs1,cs2,sf,expected",
    [
        ((0, 0), (0, 0), (0, 0), 0),
        ((-1, -1), (2, 1), (0.1, 0.2), 13.799),
    ],
)
def test_goal_path_turn_cost(cs1: tuple, cs2: tuple, sf: tuple, expected: float):
    space = ob.SE2StateSpace()

    goal = ob.State(space)

    s1 = ob.State(space)
    s2 = ob.State(space)

    goal().setXY(sf[0], sf[1])
    PATH._simple_setup.setGoalState(goal)

    s1().setXY(cs1[0], cs1[1])
    s2().setXY(cs2[0], cs2[1])

    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), PATH._simple_setup, PATH.state.heading_direction
    )
    assert minimum_turning_objective.goal_path_turn_cost(s1(), s2()) == pytest.approx(
        expected, abs=1e-3
    )


@pytest.mark.parametrize(
    "cs1,sf,heading,expected",
    [
        ((0, 0), (0, 0), 0, 0),
        ((-1, -1), (0.1, 0.2), 45, 2.490),
    ],
)
def test_goal_heading_turn_cost(cs1: tuple, sf: tuple, heading: float, expected: float):
    space = ob.SE2StateSpace()

    goal = ob.State(space)

    s1 = ob.State(space)

    goal().setXY(sf[0], sf[1])
    PATH._simple_setup.setGoalState(goal)

    s1().setXY(cs1[0], cs1[1])

    PATH.state.heading_direction = heading

    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), PATH._simple_setup, PATH.state.heading_direction
    )
    assert minimum_turning_objective.goal_heading_turn_cost(s1()) == pytest.approx(
        expected, abs=1e-3
    )


@pytest.mark.parametrize(
    "cs1,cs2,heading,expected",
    [
        ((0, 0), (0, 0), 0.0, 0),
        ((-1, -1), (2, 1), 45.0, 11.310),
    ],
)
def test_heading_path_turn_cost(cs1: tuple, cs2: tuple, heading: float, expected: float):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s2 = ob.State(space)

    s1().setXY(cs1[0], cs1[1])
    s2().setXY(cs2[0], cs2[1])

    PATH.state.heading_direction = heading

    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(), PATH._simple_setup, PATH.state.heading_direction
    )
    assert minimum_turning_objective.heading_path_turn_cost(s1(), s2()) == pytest.approx(
        expected, abs=1e-3
    )


@pytest.mark.parametrize(
    "cs1,cs2,wind_direction,expected",
    [
        ((0, 0), (0, 0), 0.0, 0 * UPWIND_MULTIPLIER),
        ((-1, -1), (2, 1), 45.0, 3.605551275 * UPWIND_MULTIPLIER),
    ],
)
def test_wind_objective(cs1: tuple, cs2: tuple, wind_direction: float, expected: float):
    space = ob.SE2StateSpace()

    s1 = ob.State(space)
    s2 = ob.State(space)

    s1().setXY(cs1[0], cs1[1])
    s2().setXY(cs2[0], cs2[1])

    PATH.state.wind_direction = wind_direction

    wind_objective = objectives.WindObjective(
        PATH._simple_setup.getSpaceInformation(), PATH.state.wind_direction
    )

    assert wind_objective.motionCost(s1(), s2()) == pytest.approx(expected, abs=1e-3)


@pytest.mark.parametrize(
    "wind_direction,heading,expected",
    [
        (0, 0.0, True),
        (0.0, 45.0, False),
    ],
)
def test_is_upwind(wind_direction: float, heading: float, expected: float):
    PATH.state.heading_direction = heading
    PATH.state.wind_direction = wind_direction

    assert (
        objectives.WindObjective.is_upwind(
            math.radians(PATH.state.wind_direction), math.radians(PATH.state.heading_direction)
        )
        == expected
    )


@pytest.mark.parametrize(
    "wind_direction,heading,expected",
    [
        (0.0, 0.0, True),
        (25.0, 45.0, False),
    ],
)
def test_is_downwind(wind_direction: float, heading: float, expected: float):
    PATH.state.heading_direction = heading
    PATH.state.wind_direction = wind_direction

    assert (
        objectives.WindObjective.is_downwind(
            math.radians(PATH.state.wind_direction), math.radians(PATH.state.heading_direction)
        )
        == expected
    )


""" Tests for bound_to_180() """


@pytest.mark.parametrize(
    "ain,expected",
    [
        (0, 0),
        (-180, -180),
        (180, -180),
        (540, -180),
        (720, 0),
        (60, 60),
        (-45, -45),
        (120, 120),
        (-125, -125),
        (179, 179),
        (-179, -179),
        (-3779, -179),
        (3779, 179),
        (360, 0),
    ],
)
def test_bound_basic1(ain: float, expected: float):
    assert objectives.WindObjective.bound_to_180(ain) == expected


""" Tests for is_angle_between() """


@pytest.mark.parametrize(
    "afir,amid,asec,expected",
    [
        (0, 1, 2, 1),
        (0, 20, 360, 0),
        (-20, 10, 40, 1),
        (0, 30, 60, 1),
        (-170, -130, -90, 1),
        (-170, -130, 100, 0),
    ],
)
def test_between_basic1(afir: float, amid: float, asec: float, expected: float):
    """Checks different situations such as boundary conditions.
    For example, what happens when par1 == par2 == par3?
    In addition, what happens if we change the order of the parameters
    """
    assert objectives.WindObjective.is_angle_between(afir, amid, asec) == expected
